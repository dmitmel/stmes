#include "stmes/main.h"
#include "stmes/dma.h"
#include "stmes/gpio.h"
#include "stmes/sdio.h"
#include "stmes/timers.h"
#include "stmes/utils.h"
#include <ff.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <string.h>

static const struct VgaTiming VGA_TIMING_640x480_60hz = {
  .pixel_freq_hz = 25175000,
  .visible_width = 640,
  .horz_front_porch = 16,
  .hsync_polarity = SYNC_PULSE_POLARITY_NEGATIVE,
  .hsync_pulse = 96,
  .horz_back_porch = 48,
  .visible_height = 480,
  .vert_front_porch = 10,
  .vsync_polarity = SYNC_PULSE_POLARITY_NEGATIVE,
  .vsync_pulse = 2,
  .vert_back_porch = 33,
};

static const struct VgaTiming VGA_TIMING_800x600_60hz = {
  .pixel_freq_hz = 40000000,
  .visible_width = 800,
  .horz_front_porch = 40,
  .hsync_polarity = SYNC_PULSE_POLARITY_POSITIVE,
  .hsync_pulse = 128,
  .horz_back_porch = 88,
  .visible_height = 600,
  .vert_front_porch = 1,
  .vsync_polarity = SYNC_PULSE_POLARITY_POSITIVE,
  .vsync_pulse = 4,
  .vert_back_porch = 23,
};

#define FRAME_WIDTH 480
#define FRAME_HEIGHT 480
static u32 dma_buf1[FRAME_WIDTH], dma_buf2[FRAME_WIDTH];
static u32 *dma_frontbuf = dma_buf1, *dma_backbuf = dma_buf2;
static volatile bool inside_frame = false;
static volatile bool line_paint_request = false;
static volatile u32 frame_counter = 0;
static volatile bool next_frame_request = false;

static void swap_dma_buffers(void) {
  u32* tmp = dma_backbuf;
  dma_backbuf = dma_frontbuf;
  dma_frontbuf = tmp;
}

extern void initialise_monitor_handles(void);

#define check_fs_error(expr) ((expr) != FR_OK ? Error_Handler() : (void)0)

struct BufferedReader {
  u8 buffer[BLOCKSIZE * 8];
  FSIZE_t offset_start;
  FSIZE_t offset_end;
};

static u8* buffered_read(struct BufferedReader* self, FIL* file, FSIZE_t pos, FSIZE_t len) {
  const FSIZE_t capacity = sizeof(self->buffer) / sizeof(*self->buffer);
  if (len > capacity) {
    Error_Handler();
  }
  if (!(self->offset_start <= pos && pos + len <= self->offset_end)) {
    self->offset_start = pos & ~(BLOCKSIZE - 1); // align to 512-byte boundaries
    self->offset_end = self->offset_start;
    if (f_tell(file) != self->offset_start) {
      check_fs_error(f_lseek(file, self->offset_start));
    }
    UINT bytes_read;
    check_fs_error(f_read(file, self->buffer, capacity, &bytes_read));
    self->offset_end = self->offset_start + bytes_read;
    if (!(self->offset_start <= pos && pos + len <= self->offset_end)) {
      Error_Handler();
    }
  }
  return &self->buffer[pos - self->offset_start];
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SDIO_SD_Init();

  HAL_StatusTypeDef hal_status;
  while ((hal_status = BSP_SD_Init()) != HAL_OK) {
    HAL_Delay(200);
  }

  check_hal_error(HAL_TIM_Base_Start_IT(&htim4));
  check_hal_error(HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2));
  check_hal_error(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3));
  check_hal_error(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2));
  check_hal_error(HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3));
  check_hal_error(HAL_TIM_Base_Start(&htim3));

  static FATFS SDFatFS;
  static FIL SDFile;
  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_open(&SDFile, "apple_3bit.bin", FA_READ));
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
  next_frame_request = true;
  line_paint_request = true;

  u32 next_row_offset = 0, prev_row_offset = 0, frame_deltas_offset = 0;

  // Wait a bit for the display to initialize
  HAL_Delay(500);

  static u32 color_pins_table[1 << 4];
  for (usize color = 0; color < (1 << 4); color++) {
    u32 pins = 0;
    pins |= VGA_PIXEL1_Pin << ((color & 1) ? 0U : 16U);
    pins |= VGA_PIXEL2_Pin << ((color & 2) ? 0U : 16U);
    pins |= VGA_PIXEL3_Pin << ((color & 4) ? 0U : 16U);
    pins |= VGA_PIXEL4_Pin << ((color & 8) ? 0U : 16U);
    color_pins_table[color] = pins;
  }

  while (true) {
    if (next_frame_request) {
      next_frame_request = false;

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

    if (line_paint_request) {
      line_paint_request = false;

      const int PIXEL_SCALE = 1;
      u32 vga_row = __HAL_TIM_GET_COUNTER(&htim4) - 33;
      u32 video_y = (vga_row - (FRAME_HEIGHT - video_height * PIXEL_SCALE) / 2) / PIXEL_SCALE;
      if (video_y < video_height) {
        struct frame_row* row = &frame_rows[video_y];
        u32* backbuf_ptr = dma_backbuf + (FRAME_WIDTH / PIXEL_SCALE - video_width) / 2;
        for (u32 i = 0; i != row->len; i++) {
          u8 encoded = row->data[i];
          u32 repeats = ((encoded >> 3) + 1) * PIXEL_SCALE;
          u32 color = color_pins_table[(encoded & 0x7) << 1];
          fast_memset_u32(backbuf_ptr, color, repeats);
          backbuf_ptr += repeats;
        }
        dma_backbuf[FRAME_WIDTH - 1] = VGA_PIXEL_ALL_PINS << 16U;
      } else {
        fast_memset_u32(dma_backbuf, VGA_PIXEL_ALL_PINS << 16U, FRAME_WIDTH);
      }

      swap_dma_buffers();
    }
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM3) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      // Reached the end of a line
      __HAL_TIM_DISABLE(&htim1);
      GPIO_RESET_PIN(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
      HAL_DMA_Abort(&hdma_tim1_up);
      __HAL_DMA_DISABLE(&hdma_tim1_up); // flush the FIFO
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      HAL_DMA_Start(
        &hdma_tim1_up, (usize)dma_frontbuf, (usize)&VGA_PIXEL_GPIO_Port->BSRR, FRAME_WIDTH
      );
      GPIO_RESET_PIN(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      // At the start of a new line
      if (inside_frame) {
        line_paint_request = true;
      }
    }
  } else if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      inside_frame = false;
      frame_counter++;
      if (frame_counter % 2 == 0) {
        next_frame_request = true;
      }
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      inside_frame = true;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM4) {}
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

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) {
  // printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif
