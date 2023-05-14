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

int main(void) {
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

  check_hal_error(BSP_SD_Init());

  check_hal_error(HAL_TIM_Base_Start(&htim2));

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

  FRESULT fres;
  if ((fres = f_mount(&SDFatFS, "", 1)) != FR_OK) {
    Error_Handler();
  }

  const struct __PACKED video_header {
    u16 width;
    u16 height;
    u32 frames_count;
    u32 frame_offsets_len;
    u32 deltas_len;
    u32 pixels_len;
  }* video_header = NULL;
  const u32* video_frame_offsets = NULL;
  const u8* video_deltas = NULL;
  const u8* video_pixels = NULL;
  u16 video_width = 0, video_height = 0;
  u32 frames_count = 0;

  struct frame_row {
    u8* data;
    u16 len, cap;
  }* frame_rows = NULL;

  frame_rows = malloc(sizeof(*frame_rows) * FRAME_HEIGHT);
  for (u32 y = 0; y < FRAME_HEIGHT; y++) {
    struct frame_row* row = &frame_rows[y];
    row->len = 0;
    row->cap = 8;
    row->data = malloc(sizeof(*row->data) * row->cap);
  }

  usize frame_nr = 0;
  next_frame_request = true;
  line_paint_request = true;

  u32 next_row_offset = 0, prev_row_offset = 0;

  // Wait a bit for the display to initialize
  HAL_Delay(500);

  usize video_file_buf_cap = 96 * 1024;
  u8* video_file_buf = malloc(video_file_buf_cap);
  usize video_file_len = 0;
  u32 batch_nr = 0;

  while (true) {
    if (next_frame_request) {
      next_frame_request = false;

      if (frame_nr >= frames_count) {
        frame_nr = 0;
        next_row_offset = 0;
        prev_row_offset = 0;

        char fname[32] = "";
        while (true) {
          batch_nr += 1;
          snprintf(fname, sizeof(fname), "apple/%04" PRIu32 ".bin", batch_nr);
          fres = f_open(&SDFile, fname, FA_READ);
          if (fres == FR_OK) {
            break;
          } else if (fres == FR_NO_FILE) {
            batch_nr = 0;
          } else {
            Error_Handler();
          }
        }

        video_file_len = f_size(&SDFile);
        if (video_file_len > video_file_buf_cap) {
          Error_Handler();
        }
        if ((fres = f_read(&SDFile, video_file_buf, video_file_len, NULL)) != FR_OK) {
          Error_Handler();
        }
        if ((fres = f_close(&SDFile)) != FR_OK) {
          Error_Handler();
        }

        usize video_ptr = (usize)video_file_buf;
        video_header = (const void*)video_ptr;
        video_ptr += sizeof(*video_header);
        video_frame_offsets = (const void*)video_ptr;
        video_ptr += sizeof(*video_frame_offsets) * video_header->frame_offsets_len;
        video_deltas = (const void*)video_ptr;
        video_ptr += sizeof(*video_deltas) * video_header->deltas_len;
        video_pixels = (const void*)video_ptr;
        video_ptr += sizeof(*video_pixels) * video_header->pixels_len;
        video_width = video_header->width;
        video_height = video_header->height;
        frames_count = video_header->frames_count;
      }

      u32 prev_row_y = -1;
      const u8* deltas_ptr = &video_deltas[video_frame_offsets[frame_nr]];
      const u8* deltas_end = &video_deltas[video_frame_offsets[frame_nr + 1]];
      while (deltas_ptr != deltas_end) {
        u8 packed_flags = *deltas_ptr++;
        struct delta_flags {
          bool contiguous_offset : 1;
          bool unchanged_offset : 1;
          bool no_common_pixels : 1;
          bool increment_y : 1;
          bool compact_row_length : 1;
          u8 inline_row_length : 3;
        } flags = *(struct delta_flags*)&packed_flags;

        u16 y;
        if (flags.increment_y) {
          y = prev_row_y + 1;
        } else {
          y = __UNALIGNED_UINT16_READ(deltas_ptr);
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

        u32 row_offset;
        if (flags.contiguous_offset) {
          row_offset = next_row_offset;
        } else if (flags.unchanged_offset) {
          row_offset = prev_row_offset;
        } else {
          row_offset = __UNALIGNED_UINT32_READ(deltas_ptr);
          deltas_ptr += 4;
        }

        next_row_offset = row_offset + row_len;
        prev_row_offset = row_offset;
        prev_row_y = y;

        struct frame_row* row = &frame_rows[y];
        row->len = common + row_len;
        if (row->cap < row->len) {
          row->data = realloc(row->data, sizeof(*row->data) * row->len);
          row->cap = row->len;
        }
        fast_memcpy_u8(row->data + common, video_pixels + row_offset, row_len);
      }

      frame_nr += 1;
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
          u32 repeats = (encoded >> 1) + 1;
          u32 value = encoded & 1;
          fast_memset_u32(backbuf_ptr, VGA_PIXEL_Pin << (value ? 0U : 16U), repeats * PIXEL_SCALE);
          backbuf_ptr += repeats * PIXEL_SCALE;
        }
        dma_backbuf[FRAME_WIDTH - 1] = VGA_PIXEL_Pin << 16U;
      } else {
        fast_memset_u32(dma_backbuf, VGA_PIXEL_Pin << 16U, FRAME_WIDTH);
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
      GPIO_RESET_PIN(VGA_PIXEL_GPIO_Port, VGA_PIXEL_Pin);
      HAL_DMA_Abort(&hdma_tim1_up);
      __HAL_DMA_DISABLE(&hdma_tim1_up); // flush the FIFO
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      HAL_DMA_Start(
        &hdma_tim1_up, (usize)dma_frontbuf, (usize)&VGA_PIXEL_GPIO_Port->BSRR, FRAME_WIDTH
      );
      GPIO_RESET_PIN(VGA_PIXEL_GPIO_Port, VGA_PIXEL_Pin);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      // At the start of a new line
      if (inside_frame) {
        line_paint_request = true;
      }
    }
  } else if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      inside_frame = false;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      inside_frame = true;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM4) {
    frame_counter++;
    if (frame_counter % 2 == 0) {
      next_frame_request = true;
    }
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

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) {
  // printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif
