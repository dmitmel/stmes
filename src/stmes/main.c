#include "stmes/main.h"
#include "stmes/dma.h"
#include "stmes/gpio.h"
#include "stmes/sdio.h"
#include "stmes/timers.h"
#include "stmes/utils.h"
#include "stmes/video/vga.h"
#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_tim.h>
#include <string.h>

#define PIXEL_SCALE 2
#define FRAME_WIDTH (640 / PIXEL_SCALE)
#define FRAME_HEIGHT (480 / PIXEL_SCALE)
#define COLOR_BIT_DEPTH 12

static struct pixel_dma_buf {
  u32 data[FRAME_WIDTH];
  u16 non_zeroes[FRAME_WIDTH];
  u16 non_zeroes_len;
} dma_buf1 = { 0 }, dma_buf2 = { 0 };
static bool dma_buffers_swapped = false;
static u32 frame_counter = 0;

__STATIC_INLINE void dma_buf_set_pixel(struct pixel_dma_buf* buf, u16 index, u32 value) {
  buf->data[index] = value;
  buf->non_zeroes[buf->non_zeroes_len++] = index;
}

static void dma_buf_reset(struct pixel_dma_buf* buf) {
  u16 len = buf->non_zeroes_len;
  u16* ptr = buf->non_zeroes;
  u16* end = ptr;
  for (end += len - len % 4; ptr != end; ptr += 4) {
    u16 a = ptr[0], b = ptr[1], c = ptr[2], d = ptr[3];
    buf->data[a] = buf->data[b] = buf->data[c] = buf->data[d] = 0;
  }
  for (end += len % 4; ptr != end; ptr += 1) {
    buf->data[*ptr] = 0;
  }
  buf->non_zeroes_len = 0;
}

extern void initialise_monitor_handles(void);

struct BufferedReader {
  u8 buffer[BLOCKSIZE * 8];
  FSIZE_t offset_start;
  FSIZE_t offset_end;
};

static u8* buffered_read(struct BufferedReader* self, FIL* file, FSIZE_t pos, FSIZE_t len) {
  const FSIZE_t capacity = SIZEOF(self->buffer);
  if (unlikely(len > capacity)) {
    Error_Handler();
  }
  if (unlikely(!(self->offset_start <= pos && pos + len <= self->offset_end))) {
    self->offset_start = pos & ~(BLOCKSIZE - 1); // align to 512-byte boundaries
    self->offset_end = self->offset_start;
    if (f_tell(file) != self->offset_start) {
      check_fs_error(f_lseek(file, self->offset_start));
    }
    UINT bytes_read;
    check_fs_error(f_read(file, self->buffer, capacity, &bytes_read));
    self->offset_end = self->offset_start + bytes_read;
    if (unlikely(!(self->offset_start <= pos && pos + len <= self->offset_end))) {
      Error_Handler();
    }
  }
  return &self->buffer[pos - self->offset_start];
}

__STATIC_FORCEINLINE u32 color_to_pins(u16 color) {
  u32 all_pins = VGA_PIXEL_ALL_PINS;
  u32 color_pins = 0;
  for (u32 mask = 1; mask != (1 << COLOR_BIT_DEPTH); mask <<= 1) {
    u32 pin = all_pins & -all_pins; // Extract the lowest set bit
    color_pins |= (color & mask) != 0 ? pin : pin << 16U;
    all_pins &= ~pin; // Strip off the pin
  }
  return color_pins;
}

int main(void) {
  // Enable the internal CPU cycle counter.
  // <https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit/CYCCNT-cycle-counter-and-related-timers?lang=en>
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#ifdef ARM_SEMIHOSTING_ENABLE
  // Actually enable the semihosting machinery only when the debugger is attached.
  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
    initialise_monitor_handles();
  }
#endif

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SDIO_SD_Init();

  vga_init();

  HAL_StatusTypeDef hal_status;
  while ((hal_status = BSP_SD_Init()) != HAL_OK) {
    HAL_Delay(200);
  }

  vga_apply_timings(&VGA_TIMINGS_640x480_57hz);
  vga_start();

  static FATFS SDFatFS;
  static FIL SDFile;
  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_open(&SDFile, "bebop_color_3bit.bin", FA_READ));
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
  if (video_ptr != video_file_len) {
    Error_Handler();
  }

  static struct frame_row {
    u8* data;
    u16 len, cap;
  } frame_rows[FRAME_HEIGHT];
  for (u32 y = 0; y < FRAME_HEIGHT; y++) {
    struct frame_row* row = &frame_rows[y];
    row->len = 0;
    row->cap = 8;
    row->data = malloc(sizeof(*row->data) * row->cap);
  }

  usize frame_nr = 0;
  u32 next_row_offset = 0, prev_row_offset = 0, frame_deltas_offset = 0;

  // Wait a bit for the display to initialize
  HAL_Delay(500);

  static u32 video_palette[8] = { 0x000, 0x00F, 0x0F0, 0x0FF, 0xF00, 0xF0F, 0xFF0, 0xFFF };
  for (usize i = 0; i < SIZEOF(video_palette); i++) {
    u32 color = video_palette[i];
    u32 all_pins = VGA_PIXEL_ALL_PINS;
    u32 color_pins = 0;
    for (u32 mask = 1; mask != (1 << COLOR_BIT_DEPTH); mask <<= 1) {
      u32 pin = all_pins & -all_pins; // Extract the lowest set bit
      color_pins |= (color & mask) != 0 ? pin : pin << 16U;
      all_pins &= ~pin; // Strip off the pin
    }
    video_palette[i] = color_pins;
  }

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
      u8* deltas_ptr = buffered_read(
        &deltas_reader, &SDFile, video_deltas_start + frame_deltas_offset, deltas_len
      );
      u8* deltas_end = &deltas_ptr[deltas_len];

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
      struct pixel_dma_buf* backbuf = dma_buffers_swapped ? &dma_buf2 : &dma_buf1;
      struct VgaScanline scanline = {
        .buffer = backbuf->data,
        .length = FRAME_WIDTH,
        .pixel_scale = PIXEL_SCALE - 1,
        .repeats = PIXEL_SCALE - 1,
      };
      vga_set_next_scanline(&scanline);
      dma_buffers_swapped = !dma_buffers_swapped;

      dma_buf_reset(backbuf);
      dma_buf_set_pixel(backbuf, 0, VGA_PIXEL_ALL_PINS << 16U);
      dma_buf_set_pixel(backbuf, FRAME_WIDTH - 1, VGA_PIXEL_ALL_PINS << 16U);

      u32 video_y = vga_line / PIXEL_SCALE - (FRAME_HEIGHT - video_height) / 2;
      if (video_y >= video_height) {
        Error_Handler();
      } else {
        struct frame_row* row = &frame_rows[video_y];
        usize pixel_idx = (FRAME_WIDTH - video_width) / 2;
        u32 prev_color = 0;
        for (u32 i = 0; i < row->len; i++) {
          u8 encoded = row->data[i];
          u32 repeats = (encoded >> 3) + 1;
          u32 color = video_palette[(encoded & 0x7)];
          if (color != prev_color) {
            dma_buf_set_pixel(backbuf, pixel_idx, color);
            prev_color = color;
          }
          pixel_idx += repeats;
        }
        if (pixel_idx > FRAME_WIDTH) {
          Error_Handler();
        }
        if (pixel_idx < FRAME_WIDTH) {
          dma_buf_set_pixel(backbuf, pixel_idx, VGA_PIXEL_ALL_PINS << 16U);
        }
      }
    }

    __WFE(); // Wait for event
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

__NO_RETURN void Error_Handler(void) {
  __disable_irq();
  while (true) {
    const u32 blink_delay = 200;
    LL_GPIO_SetOutputPin(BLTN_LED_GPIO_Port, BLTN_LED_Pin);
    HAL_Delay(blink_delay);
    LL_GPIO_ResetOutputPin(BLTN_LED_GPIO_Port, BLTN_LED_Pin);
    HAL_Delay(blink_delay);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) {
  // printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif
