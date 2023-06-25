#include "stmes/main.h"
#include "stmes/dma.h"
#include "stmes/fatfs.h"
#include "stmes/gpio.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/mpu.h"
#include "stmes/sdio.h"
#include "stmes/timers.h"
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

int main(void) {
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

  HAL_Init();
  SystemClock_Config();

  console_init();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();

  vga_init();
  vga_apply_timings(&VGA_TIMINGS_640x480_57hz);
  vga_start();

  HAL_StatusTypeDef hal_status;
  while ((hal_status = BSP_SD_Init()) != HAL_OK) {
    HAL_Delay(200);
  }

  static FATFS SDFatFS;
  static FIL SDFile;
  check_fs_error(f_mount(&SDFatFS, "", 1));

  static DIR SDDir;
  check_fs_error(f_opendir(&SDDir, "/"));

  static struct {
    u8 color;
    const char* str;
  } code_tokens[] = {
    // clang-format off
    {5, "#include"}, {7, " "}, {2, "<stdio.h>"}, {7, "\n"},
    {5, "#include"}, {7, " "}, {2, "<stm32f4xx_hal.h>"}, {6, "\n"},
    {3, "int"}, {7, " "}, {4, "main"}, {7, "("}, {3, "int"}, {7, " "}, {1, "argc"}, {7, ", "}, {3, "char"}, {7, " "}, {1, "argv"}, {7, "[]"}, {7, ") {\n"},
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

  while (false) {
    __WAIT_FOR_INTERRUPT();
    u16 vga_line = 0;
    if (vga_take_scanline_request(&vga_line)) {
      console_render_scanline(vga_line);
    }
    if (vga_registers.next_frame_request) {
      vga_registers.next_frame_request = false;
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
            ((fno.fdate >> 9) & 0x7F) + 1980,
            (fno.fdate >> 5) & 0x0F,
            fno.fdate & 0x1F,
            (fno.ftime >> 11) & 0x1F,
            (fno.ftime >> 5) & 0x3F,
            fno.ftime & 0x1F,
            fno.fsize / 1024,
            fno.fname
          );
        }
      }
    }
  }

  check_fs_error(f_open(&SDFile, "bebop_palette.bin", FA_READ));
  FSIZE_t video_file_len = f_size(&SDFile);

  struct __PACKED video_header {
    u16 width;
    u16 height;
    u32 frames_len;
    u32 deltas_len;
    u32 pixels_len;
  } video_header = { 0 };
  check_fs_error(f_read(&SDFile, &video_header, sizeof(video_header), NULL));
  u16 video_width = video_header.width;
  u16 video_height = video_header.height;
  u32 frames_len = video_header.frames_len;

  FSIZE_t video_ptr = 0;
  video_ptr += sizeof(video_header);
  FSIZE_t video_frames_start = video_ptr;
  FSIZE_t video_frames_size = sizeof(u16) * video_header.frames_len;
  video_ptr += video_frames_size;
  FSIZE_t video_deltas_start = video_ptr;
  FSIZE_t video_deltas_size = sizeof(u8) * video_header.deltas_len;
  video_ptr += video_deltas_size;
  FSIZE_t video_pixels_start = video_ptr;
  FSIZE_t video_pixels_size = sizeof(u8) * video_header.pixels_len;
  video_ptr += video_pixels_size;
  ASSERT(video_ptr == video_file_len);

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

  // Wait a bit for the display to initialize
  HAL_Delay(500);

  u32 video_palette[8];

  while (true) {
    bool load_next_frame = false;
    if (unlikely(vga_registers.next_frame_request)) {
      vga_registers.next_frame_request = false;
      frame_counter++;
      if (frame_counter % 2 == 0) {
        load_next_frame = true;
      }
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
      struct VgaScanline scanline = {
        .buffer = backbuf->data,
        .length = FRAME_WIDTH,
        .pixel_scale = PIXEL_SCALE - 1,
        .repeats = PIXEL_SCALE - 1,
      };
      vga_set_next_scanline(&scanline);

      fast_memset_u32(backbuf->data, 0, FRAME_WIDTH - 1);
      backbuf->data[0] = backbuf->data[FRAME_WIDTH - 1] = VGA_PIXEL_ALL_PINS_RESET;

      u32 video_y = vga_line / PIXEL_SCALE - (FRAME_HEIGHT - video_height) / 2;
      if (video_y < video_height) {
        struct frame_row* row = &frame_rows[video_y];
        usize pixel_idx = (FRAME_WIDTH - video_width) / 2;
        for (u32 i = 0; i < row->len; i++) {
          u8 encoded = row->data[i];
          u32 repeats = (encoded >> 3) + 1;
          u32 color = video_palette[encoded & 0x7];
          backbuf->data[pixel_idx] = color;
          pixel_idx += repeats;
        }
        ASSERT(pixel_idx <= FRAME_WIDTH);
        if (pixel_idx < FRAME_WIDTH) {
          backbuf->data[pixel_idx] = VGA_PIXEL_ALL_PINS_RESET;
        }
      }
    }

    __WAIT_FOR_INTERRUPT();
  }
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
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

HAL_StatusTypeDef HAL_InitTick(u32 priority) {
  UNUSED(priority);
  MX_TIM2_Init();
  check_hal_error(HAL_TIM_Base_Start(&htim2));
  return HAL_OK;
}

u32 HAL_GetTick(void) {
  return TIM2->CNT / 2;
}
