#include "stmes/demos.h"
#include "stmes/drivers/sdmmc.h"
#include "stmes/fatfs.h"
#include "stmes/gpio.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"
#include "stmes/kernel/time.h"
#include "stmes/utils.h"
#include "stmes/video/framebuf.h"
#include "stmes/video/vga.h"
#include "stmes/video/vga_color.h"
#include <ff.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_gpio.h>

static void button_task_fn(void* user_data) {
  volatile bool* button_pressed = user_data;
  while (true) {
    task_wait(&gpio_button_notification, NO_DEADLINE);
    task_sleep(20);
    *button_pressed = LL_GPIO_IsInputPinSet(BLTN_KEY_GPIO_PORT, BLTN_KEY_PIN);
  }
}

struct BufferedReader {
  u8 buffer[SDMMC_BLOCK_SIZE * 16];
  FSIZE_t offset_start;
  FSIZE_t offset_end;
};

static u8* buffered_read(struct BufferedReader* self, FIL* file, FSIZE_t pos, FSIZE_t len) {
  const FSIZE_t capacity = SIZEOF(self->buffer);
  ASSERT(len <= capacity);
  if (unlikely(!(self->offset_start <= pos && pos + len <= self->offset_end))) {
    self->offset_start = align_to(pos, SDMMC_BLOCK_SIZE);
    self->offset_end = self->offset_start;
    if (f_tell(file) != self->offset_start) {
      check_fs_error(f_lseek(file, self->offset_start));
    }
    UINT bytes_read;
    check_fs_error(f_read(file, self->buffer, capacity, &bytes_read));
    self->offset_end = self->offset_start + bytes_read;
    ASSERT(self->offset_start <= pos && pos + len <= self->offset_end);
  }
  return &self->buffer[pos - self->offset_start];
}

void video_player_demo(void) {
  volatile bool button_pressed;
  static struct Task button_task;
  static u8 button_task_stack[256] __ALIGNED(8);
  struct TaskParams button_task_params = {
    .stack_start = button_task_stack,
    .stack_size = sizeof(button_task_stack),
    .func = button_task_fn,
    .user_data = (void*)&button_pressed,
  };
  task_spawn(&button_task, &button_task_params);

  static FATFS SDFatFS;
  static FIL SDFile;

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_open(&SDFile, "bebop_palette.bin", FA_READ));
  const u32 VIDEO_FRAME_RATE = 24;

  struct __PACKED video_header {
    u16 width;
    u16 height;
    u32 frames_len;
    u32 deltas_len;
    u32 pixels_len;
  } video_header = { 0 };
  UINT bytes_read = 0;
  check_fs_error(f_read(&SDFile, &video_header, sizeof(video_header), &bytes_read));
  u16 video_width = video_header.width;
  u16 video_height = video_header.height;
  u32 frames_len = video_header.frames_len;

  FSIZE_t file_ptr = 0;
  file_ptr += sizeof(video_header);
  FSIZE_t video_frames_start = file_ptr;
  FSIZE_t video_frames_size = sizeof(u16) * video_header.frames_len;
  file_ptr += video_frames_size;
  FSIZE_t video_deltas_start = file_ptr;
  FSIZE_t video_deltas_size = sizeof(u8) * video_header.deltas_len;
  file_ptr += video_deltas_size;
  FSIZE_t video_pixels_start = file_ptr;
  FSIZE_t video_pixels_size = sizeof(u8) * video_header.pixels_len;
  file_ptr += video_pixels_size;
  ASSERT(file_ptr == f_size(&SDFile));

  static struct frame_row {
    u8* data;
    u16 len, cap;
  } frame_rows[FRAME_HEIGHT];
  for (u32 y = 0; y < FRAME_HEIGHT; y++) {
    struct frame_row* row = &frame_rows[y];
    row->len = 0;
    row->cap = 8;
    row->data = malloc(sizeof(*row->data) * row->cap);
    ASSERT(row->data != NULL);
  }

  usize frame_nr = 0;
  u32 next_row_offset = 0, prev_row_offset = 0, frame_deltas_offset = 0;
  u32 video_palette[8];

  Systime video_start_time = systime_now();

  bool show_console = false;
  while (true) {
    if (unlikely(button_pressed)) {
      button_pressed = false;
      show_console = !show_console;
      struct PixelDmaBuffer* frontbuf = swap_pixel_dma_buffers();
      struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
      vga_fast_memset(frontbuf->data, 0, FRAME_WIDTH);
      vga_fast_memset(backbuf->data, 0, FRAME_WIDTH);
    }

    bool load_next_frame = false;
    if (unlikely(vga_control.entering_vblank)) {
      vga_control.entering_vblank = false;
      Systime next_frame_time =
        video_start_time + systime_from_millis(frame_nr * 1000) / VIDEO_FRAME_RATE;
      if (systime_now() >= next_frame_time) {
        load_next_frame = true;
      }

      struct VgaFrameConfig frame = {
        .line_length = FRAME_WIDTH,
        .lines_count = FRAME_HEIGHT * PIXEL_SCALE,
        .pixel_scale = PIXEL_SCALE - 1,
        .line_repeats = PIXEL_SCALE - 1,
      };
      vga_set_frame_config(&frame);
    }

    if (unlikely(load_next_frame)) {
      if (frame_nr >= frames_len) {
        frame_nr = 0;
        next_row_offset = 0;
        prev_row_offset = 0;
        frame_deltas_offset = 0;
        video_start_time = systime_now();
      }

      static struct BufferedReader frames_reader;
      u8* frame_ptr = buffered_read(
        &frames_reader, &SDFile, video_frames_start + sizeof(u16) * frame_nr, sizeof(u16)
      );
      u16 deltas_len = __UNALIGNED_UINT16_READ(frame_ptr);

      static struct BufferedReader deltas_reader;
      const u8* deltas_ptr = buffered_read(
        &deltas_reader, &SDFile, video_deltas_start + frame_deltas_offset, deltas_len
      );
      const u8* deltas_end = &deltas_ptr[deltas_len];

      for (usize i = 0; i < SIZEOF(video_palette); i++, deltas_ptr += 3) {
        u8 r8 = deltas_ptr[0], g8 = deltas_ptr[1], b8 = deltas_ptr[2];
        u32 rgb12 = (r8 >> 4 << 8) | (g8 >> 4 << 4) | (b8 >> 4);
        video_palette[i] = rgb12_to_vga_pins(rgb12);
      }

      u16 row_y = -1;
      while (deltas_ptr != deltas_end) {
        u8 packed_flags = *deltas_ptr++;
        struct delta_flags {
          bool repeat_previous : 1;
          bool duplicate_row : 1;
          bool no_common_pixels : 1;
          bool increment_y : 1;
          bool compact_row_length : 1;
          u8 inline_row_length : 3;
        } flags = *(struct delta_flags*)&packed_flags;

        if (!flags.increment_y) {
          row_y = __UNALIGNED_UINT16_READ(deltas_ptr);
          deltas_ptr += 2;
        }

        u8 common = 0;
        if (!flags.no_common_pixels) {
          common = *deltas_ptr++;
        }

        u8 row_len = 0;
        if (flags.compact_row_length) {
          row_len = flags.inline_row_length + 1;
        } else {
          row_len = *deltas_ptr++;
        }

        u16 repeats = 1;
        if (flags.repeat_previous) {
          repeats = 2 + *deltas_ptr++;
        }

        for (u16 i = 0; i < repeats; i++) {
          if (flags.increment_y) {
            row_y += 1;
          }

          u32 row_offset;
          if (flags.duplicate_row) {
            row_offset = prev_row_offset;
          } else {
            row_offset = next_row_offset;
          }

          struct frame_row* row = &frame_rows[row_y];
          row->len = common + row_len;
          if (row->cap < row->len) {
            row->data = realloc(row->data, sizeof(*row->data) * row->len);
            ASSERT(row->data != NULL);
            row->cap = row->len;
          }

          static struct BufferedReader pixels_reader;
          u8* row_pixels =
            buffered_read(&pixels_reader, &SDFile, video_pixels_start + row_offset, row_len);
          fast_memcpy_u8(row->data + common, row_pixels, row_len);

          next_row_offset = row_offset + row_len;
          prev_row_offset = row_offset;
        }
      }

      frame_nr += 1;
      frame_deltas_offset += deltas_len;
    }

    u16 vga_line = 0;
    if (unlikely(vga_take_scanline_request(&vga_line))) {
      u32 start_time = DWT->CYCCNT;

      struct PixelDmaBuffer* real_backbuf = swap_pixel_dma_buffers();
      vga_set_next_scanline(real_backbuf->data);
      static struct PixelDmaBuffer fake_backbuf;
      struct PixelDmaBuffer* backbuf = show_console ? &fake_backbuf : real_backbuf;

      vga_fast_memset(backbuf->data, 0, FRAME_WIDTH);
      backbuf->data[0] = backbuf->data[FRAME_WIDTH - 1] = VGA_ALL_RGB_PINS_RESET;

      u32 video_y = vga_line / PIXEL_SCALE - (FRAME_HEIGHT - video_height) / 2;
      if (video_y < video_height) {
        struct frame_row* row = &frame_rows[video_y];
        usize pixel_idx = (FRAME_WIDTH - video_width) / 2;
        for (u32 i = 0; i < row->len; i++) {
          u8 encoded = row->data[i];
          u32 repeats = (encoded >> 3) + 1;
          u32 color = video_palette[encoded & MASK(3)];
          backbuf->data[pixel_idx] = color;
          pixel_idx += repeats;
        }
        ASSERT(pixel_idx <= FRAME_WIDTH);
        backbuf->data[MIN(pixel_idx, FRAME_WIDTH - 1)] = VGA_ALL_RGB_PINS_RESET;
      }

      u32 end_time = DWT->CYCCNT;
      if (show_console) {
        pixel_dma_buf_reset(real_backbuf);
        pixel_dma_buf_set(real_backbuf, 0, VGA_ALL_RGB_PINS);
        u32 len = MIN(1 + (end_time - start_time) * (FRAME_WIDTH - 1) / 6400, FRAME_WIDTH - 1);
        pixel_dma_buf_set(real_backbuf, len, VGA_ALL_RGB_PINS_RESET);
      }
    }

    WAIT_FOR_INTERRUPT();
  }
}
