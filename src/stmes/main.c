#include "stmes/main.h"
#include "stmes/dma.h"
#include "stmes/fatfs.h"
#include "stmes/gpio.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/mpu.h"
#include "stmes/kernel/task.h"
#include "stmes/kernel/time.h"
#include "stmes/sdio.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/framebuf.h"
#include "stmes/video/vga.h"
#include <ff.h>
#include <printf.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <string.h>

struct BufferedReader {
  u8 buffer[BLOCKSIZE * 8];
  FSIZE_t offset_start;
  FSIZE_t offset_end;
};

static u8* buffered_read(struct BufferedReader* self, FIL* file, FSIZE_t pos, FSIZE_t len) {
  const FSIZE_t capacity = SIZEOF(self->buffer);
  ASSERT(len <= capacity);
  if (unlikely(!(self->offset_start <= pos && pos + len <= self->offset_end))) {
    self->offset_start = pos & ~(BLOCKSIZE - 1); // align to 512-byte boundaries
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

static __NO_RETURN void terminal_demo(void);
static __NO_RETURN void video_player_demo(void);
static __NO_RETURN void image_viewer_demo(void);

static TaskFunc render_task_fn;
static u8 render_task_stack[1024] __ALIGNED(8);
static struct Task render_task;

static TaskFunc test_task_fn;
static u8 test_task_stack[1024] __ALIGNED(8);
static struct Task test_task;

static TaskFunc progress_task_fn;
static u8 progress_task_stack[1024] __ALIGNED(8);
static struct Task progress_task;

static TaskFunc print_task_fn;
static struct {
  u8 stack[1024] __ALIGNED(8);
  struct Task task;
} print_tasks[8];

static struct Notification progress_task_notify;

void prestart(void) {
  extern u32 _sdata, _edata, _sidata, _sbss, _ebss;
  fast_memcpy_u32(&_sdata, &_sidata, &_edata - &_sdata);
  fast_memset_u32(&_sbss, 0, &_ebss - &_sbss);

  // Flush the pipeline in case the initialization of the memory sections above
  // has caused any side effects, or placed any functions into RAM.
  __DSB();
  __ISB();

  SystemInit(); // Initializes the FPU among other things

  crash_init_hard_faults();
  mpu_init();

  // Enable the internal CPU cycle counter.
  // <https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit/CYCCNT-cycle-counter-and-related-timers>
  // <https://developer.arm.com/documentation/ka001499/latest> - getting accurate executed instruction counts
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#ifdef ARM_SEMIHOSTING_ENABLE
  // Actually enable the semihosting machinery only when the debugger is attached.
  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
    extern void initialise_monitor_handles(void);
    initialise_monitor_handles();
  }
#endif

  extern void __libc_init_array(void);
  // Runs the initializers of static variables and functions marked with
  // __attribute__((constructor)).
  __libc_init_array();
}

int main(void) {
  HAL_Init();
  // TODO: this function needs HAL_GetTick to be ready
  SystemClock_Config();

  hwtimer_init();

  console_init();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();

  vga_init();
  vga_apply_timings(&VGA_TIMINGS_640x480_57hz);
  vga_start();

  task_notify_init(&progress_task_notify);

  struct TaskParams render_task_params = {
    .stack_start = render_task_stack,
    .stack_size = sizeof(render_task_stack),
    .func = &render_task_fn,
  };
  task_spawn(&render_task, &render_task_params);

  struct TaskParams test_task_params = {
    .stack_start = test_task_stack,
    .stack_size = sizeof(test_task_stack),
    .func = &test_task_fn,
  };
  task_spawn(&test_task, &test_task_params);

  struct TaskParams progress_task_params = {
    .stack_start = progress_task_stack,
    .stack_size = sizeof(progress_task_stack),
    .func = &progress_task_fn,
  };
  task_spawn(&progress_task, &progress_task_params);

  // for (usize i = 0; i < SIZEOF(print_tasks); i++) {
  //   struct TaskParams params = {
  //     .stack_start = print_tasks[i].stack,
  //     .stack_size = SIZEOF(print_tasks[i].stack),
  //     .func = &print_task_fn,
  //     .user_data = (void*)i,
  //   };
  //   task_spawn(&print_tasks[i].task, &params);
  // }

  vga_control.entering_vblank = true;
  task_start_scheduling();
  // video_player_demo();
  // terminal_demo();
  // image_viewer_demo();
  // console_main_loop();
}

struct Task* scheduler(struct Task* prev_task) {
  static bool in_video_phase = false;
  while (true) {
    if (likely(in_video_phase)) {
      if (likely(vga_control.next_scanline_requested)) {
        return &render_task;
      }
      if (vga_control.entering_vblank) {
        in_video_phase = false;
        return &render_task;
      }
    } else {
      if (vga_control.entering_frame) {
        vga_control.entering_frame = false;
        in_video_phase = true;
        return &render_task;
      }
      if (vga_control.entering_vblank) {
        return &render_task;
      }
    }
    struct Task* next_task = task_standard_scheduler(prev_task);
    if (next_task != NULL) {
      return next_task;
    }
    WAIT_FOR_INTERRUPT();
  }
}

static FATFS SDFatFS;
static FIL SDFile;
static DIR SDDir;

static void test_task_fn(__UNUSED void* user_data) {
  while (BSP_SD_Init() != HAL_OK) {
    printf(".");
    task_sleep(1000);
  }

  console_clear_screen();

  check_fs_error(f_mount(&SDFatFS, "", 1));

again:
  check_fs_error(f_open(&SDFile, "bebop_palette.bin", FA_READ));

  printf("loading %lu\n", f_size(&SDFile));
  task_yield();

  usize total_bytes = 0;
  static char buf[BLOCKSIZE * 16];
  Instant start_time = systime_now();

  while (true) {
    task_yield();
    task_notify(&progress_task_notify);
    usize bytes_read = 0;
    if (f_read(&SDFile, buf, sizeof(buf), &bytes_read) != FR_OK) {
      break;
    }
    if (bytes_read == 0) {
      break;
    }
    total_bytes += bytes_read;
  }
  task_notify(&progress_task_notify);
  task_yield();

  Instant end_time = systime_now();
  printf("\n");

  float elapsed_seconds = (float)(u32)(end_time - start_time) / 1000.0f;
  printf("%" PRIuPTR " %" PRIu32 "\n", total_bytes, (u32)(end_time - start_time));
  printf("%f B/sec\n", total_bytes / elapsed_seconds);
  printf("%f kB/sec\n", (total_bytes / 1024.0f) / elapsed_seconds);
  printf("%f MB/sec\n", (total_bytes / 1024.0f / 1024.0f) / elapsed_seconds);

  task_sleep(2000);
  f_close(&SDFile);
  goto again;

  while (true) {
    task_yield();
  }
}

static void progress_task_fn(__UNUSED void* user_data) {
  task_wait(&progress_task_notify, NO_DEADLINE);
  Instant start_time = systime_now();
  while (true) {
    task_wait(&progress_task_notify, NO_DEADLINE);
    console_clear_cursor_line();
    u32 percent = f_tell(&SDFile) * 100 / f_size(&SDFile);
    printf("\r%" PRIu32 "%% %" PRIu32, percent, (u32)(systime_now() - start_time));
    task_sleep(100);
  }
}

static void print_task_fn(__UNUSED void* user_data) {
  u32 task_idx = (u32)user_data;
  u32 counter = 0;
  while (true) {
    console_set_cursor_line(task_idx);
    console_clear_cursor_line();
    printf("%" PRIu32 " %" PRIu32 "\n", task_idx, counter);
    counter += 1;
    task_sleep(task_idx + 1);
  }
}

static void render_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_wait(&vga_notification, NO_DEADLINE);
    if (vga_control.next_scanline_requested) {
      vga_control.next_scanline_requested = false;
      console_render_scanline(vga_control.next_scanline_nr);
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
    }
  }
}

static __NO_RETURN void terminal_demo(void) {
  while (BSP_SD_Init() != HAL_OK) {
    HAL_Delay(500);
  }

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_opendir(&SDDir, "/"));

  static struct {
    u8 color;
    const char* str;
  } code_tokens[] = {
    // clang-format off
    {5, "#include"}, {7, " "}, {2, "<stdio.h>"}, {7, "\n"},
    {5, "#include"}, {7, " "}, {2, "<stm32f4xx_hal.h>"}, {6, "\n"},
    {3, "int"}, {7, " "}, {4, "main"}, {7, "("}, {3, "int"}, {7, " "}, {1, "argc"}, {7, ", "}, {3, "char"}, {7, "* "}, {1, "argv"}, {7, "[]"}, {7, ") {\n"},
    {7, "  "}, {4, "printf"}, {7, "("}, {2, "\"Hello world from STM32!"}, {3, "\\n"}, {2, "\""}, {7, ");\n"},
    {7, "  "}, {5, "while"}, {7, " ("}, {3, "true"}, {7, ") {\n"},
    {7, "    "}, {4, "HAL_GPIO_TogglePin"}, {7, "("}, {3, "GPIOC"}, {7, ", "}, {3, "GPIO_PIN_13"}, {7, ");\n"},
    {7, "    "}, {4, "HAL_Delay"}, {7, "("}, {3, "100"}, {7, ");\n"},
    {7, "  }\n"},
    {7, "  "}, {5, "return"}, {7, " "}, {3, "0"}, {7, ";\n"},
    {7, "}\n"},
    // clang-format on
  };

  for (usize i = 0; i < SIZEOF(code_tokens); i++) {
    console_set_color(code_tokens[i].color);
    console_print(code_tokens[i].str);
  }

  while (true) {
    u16 vga_line = 0;
    if (vga_take_scanline_request(&vga_line)) {
      console_render_scanline(vga_line);
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
      static u32 prev_tick;
      u32 tick = HAL_GetTick();
      if (tick >= prev_tick + 500) {
        prev_tick = tick;
        static FILINFO fno;
        check_fs_error(f_readdir(&SDDir, &fno));
        if (fno.fname[0] != '\0') {
          printf(
            "%s%s%s%s%s %04d-%02d-%02d %02d:%02d:%02d %6luk %s\n",
            fno.fattrib & AM_ARC ? "A" : "-",
            fno.fattrib & AM_DIR ? "D" : "-",
            fno.fattrib & AM_SYS ? "S" : "-",
            fno.fattrib & AM_HID ? "H" : "-",
            fno.fattrib & AM_RDO ? "R" : "-",
            ((fno.fdate >> 9) & MASK(7)) + 1980,
            (fno.fdate >> 5) & MASK(4),
            fno.fdate & MASK(5),
            (fno.ftime >> 11) & MASK(5),
            (fno.ftime >> 5) & MASK(6),
            fno.ftime & MASK(5),
            fno.fsize / 1024,
            fno.fname
          );
        }
      }
    }
    WAIT_FOR_INTERRUPT();
  }
}

static __NO_RETURN void video_player_demo(void) {
  while (BSP_SD_Init() != HAL_OK) {
    HAL_Delay(500);
  }

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_open(&SDFile, "bebop_palette.bin", FA_READ));

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

  usize frame_nr = 0, frame_counter = 0;
  u32 next_row_offset = 0, prev_row_offset = 0, frame_deltas_offset = 0;
  u32 video_palette[8];

  while (true) {
    bool load_next_frame = false;
    if (unlikely(vga_control.entering_vblank)) {
      vga_control.entering_vblank = false;
      frame_counter++;
      if (frame_counter % 2 == 0) {
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
      struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
      vga_set_next_scanline(backbuf->data);

      vga_fast_memset(backbuf->data, 0, FRAME_WIDTH);
      backbuf->data[0] = backbuf->data[FRAME_WIDTH - 1] = VGA_PIXEL_ALL_PINS_RESET;

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
        backbuf->data[MIN(pixel_idx, FRAME_WIDTH - 1)] = VGA_PIXEL_ALL_PINS_RESET;
      }
    }

    WAIT_FOR_INTERRUPT();
  }
}

static __NO_RETURN void image_viewer_demo(void) {
  while (BSP_SD_Init() != HAL_OK) {
    HAL_Delay(500);
  }

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_open(&SDFile, "penguins_palette.bin", FA_READ));

  struct __PACKED image_header {
    u16 width;
    u16 height;
    u16 palette_len;
    u32 pixels_len;
  } image_header = { 0 };
  UINT bytes_read = 0;
  check_fs_error(f_read(&SDFile, &image_header, sizeof(image_header), &bytes_read));
  u16 image_width = image_header.width;
  u16 image_height = image_header.height;

  FSIZE_t file_ptr = 0;
  file_ptr += sizeof(image_header);
  FSIZE_t image_palette_start = file_ptr;
  FSIZE_t image_palette_size = sizeof(u8) * image_header.palette_len;
  file_ptr += image_palette_size;
  FSIZE_t image_pixels_start = file_ptr;
  FSIZE_t image_pixels_size = sizeof(u8) * image_header.pixels_len;
  file_ptr += image_pixels_size;
  ASSERT(file_ptr == f_size(&SDFile));

  u8* image_palette = malloc(image_palette_size);
  check_fs_error(f_lseek(&SDFile, image_palette_start));
  check_fs_error(f_read(&SDFile, image_palette, image_palette_size, &bytes_read));

  u8* image_pixels = malloc(image_pixels_size);
  check_fs_error(f_lseek(&SDFile, image_pixels_start));
  check_fs_error(f_read(&SDFile, image_pixels, image_pixels_size, &bytes_read));

  static u32 video_palette[1 << 8];
  for (usize i = 0; i < MIN(image_header.palette_len, SIZEOF(video_palette) * 3); i += 3) {
    u8 red = image_palette[i], green = image_palette[i + 1], blue = image_palette[i + 2];
    red >>= 4, green >>= 4, blue >>= 4;
    video_palette[i / 3] = rgb12_to_vga_pins((red << 8) | (green << 4) | blue);
  }

  while (true) {
    if (unlikely(vga_control.entering_vblank)) {
      vga_control.entering_vblank = false;
      struct VgaFrameConfig frame = {
        .line_length = FRAME_WIDTH,
        .lines_count = FRAME_HEIGHT * PIXEL_SCALE,
        .pixel_scale = PIXEL_SCALE - 1,
        .line_repeats = PIXEL_SCALE - 1,
      };
      vga_set_frame_config(&frame);
    }

    u16 vga_line = 0;
    if (unlikely(vga_take_scanline_request(&vga_line))) {
      struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
      vga_set_next_scanline(backbuf->data);

      backbuf->data[0] = backbuf->data[FRAME_WIDTH - 1] = VGA_PIXEL_ALL_PINS_RESET;

      u32 video_y = vga_line / PIXEL_SCALE - (FRAME_HEIGHT - image_height) / 2;
      // video_y = (video_y + frame_counter) % video_height;
      if (video_y < image_height) {
        usize pixel_idx = (FRAME_WIDTH - image_width) / 2;
        u8* row = &image_pixels[video_y * image_width];
        VgaPixel* pixel_ptr = &backbuf->data[pixel_idx];
        for (u8* row_end = row + image_width; row != row_end; row += 8, pixel_ptr += 8) {
          u32 quad1 = ((u32*)row)[0], quad2 = ((u32*)row)[1];
          u32 byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8;
          byte1 = quad1 & 0xFF, byte2 = (quad1 >> 8) & 0xFF;
          byte3 = (quad1 >> 16) & 0xFF, byte4 = quad1 >> 24;
          byte5 = quad2 & 0xFF, byte6 = (quad2 >> 8) & 0xFF;
          byte7 = (quad2 >> 16) & 0xFF, byte8 = quad2 >> 24;
          byte1 = row[0], byte2 = row[1], byte3 = row[2], byte4 = row[3];
          byte5 = row[4], byte6 = row[5], byte7 = row[6], byte8 = row[7];
          byte1 = video_palette[byte1], byte2 = video_palette[byte2];
          byte3 = video_palette[byte3], byte4 = video_palette[byte4];
          byte5 = video_palette[byte5], byte6 = video_palette[byte6];
          byte7 = video_palette[byte7], byte8 = video_palette[byte8];
          pixel_ptr[0] = byte1, pixel_ptr[1] = byte2, pixel_ptr[2] = byte3, pixel_ptr[3] = byte4;
          pixel_ptr[4] = byte5, pixel_ptr[5] = byte6, pixel_ptr[6] = byte7, pixel_ptr[7] = byte8;
        }
        pixel_idx = pixel_ptr - &backbuf->data[0];
        if (pixel_idx < FRAME_WIDTH) {
          backbuf->data[pixel_idx] = VGA_PIXEL_ALL_PINS_RESET;
        }
      }
    }

    WAIT_FOR_INTERRUPT();
  }
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriority(SVCall_IRQn, 15, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

void SystemClock_Config(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitTypeDef rcc_osc_init = {
    .OscillatorType = RCC_OSCILLATORTYPE_HSE,
    .HSEState = RCC_HSE_ON,
    .PLL = {
      .PLLState = RCC_PLL_ON,
      .PLLSource = RCC_PLLSOURCE_HSE,
      .PLLM = 25,
      .PLLN = 192,
      .PLLP = RCC_PLLP_DIV2,
      .PLLQ = 4,
    },
  };
  check_hal_error(HAL_RCC_OscConfig(&rcc_osc_init));

  RCC_ClkInitTypeDef rcc_clk_init = {
    .ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
    .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
    .AHBCLKDivider = RCC_SYSCLK_DIV1,
    .APB1CLKDivider = RCC_HCLK_DIV2,
    .APB2CLKDivider = RCC_HCLK_DIV1,
  };
  u32 flash_latency = FLASH_LATENCY_3;
  check_hal_error(HAL_RCC_ClockConfig(&rcc_clk_init, flash_latency));
}
