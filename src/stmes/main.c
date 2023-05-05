#include "stmes/main.h"
#include "stmes/gpio.h"
#include "stmes/timers.h"
#include "stmes/utils.h"
#include <stdio.h>
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

extern void initialise_monitor_handles(void);

// void* blackbox(void* ptr) {
//   return ptr;
// }

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  check_hal_error(HAL_TIM_Base_Start(&htim2));

  // static u8 pixels_arr[320] = { 0 };
  // static u32 dma_arr[320] = { 0 };
  // u8* pixels = blackbox(pixels_arr);
  // u32* dma = blackbox(dma_arr);
  //
  // volatile u32 start_time = TIM2->CNT;
  //
  // u8 prev = 0, pixel, diff, set, reset;
  // for (u32* dma_end = dma + 320; dma != dma_end;) {
  //   pixel = *pixels++;
  //   diff = pixel ^ prev;
  //   set = pixel & diff, reset = ~pixel & diff;
  //   *dma++ = (reset << 24) | (set << 8);
  //   prev = pixel;
  // }
  //
  // volatile u32 end_time = TIM2->CNT;
  // printf("elapsed ticks = %" PRIu32 "\n", end_time - start_time);
  //
  // blackbox(dma);

  check_hal_error(HAL_TIM_Base_Start_IT(&htim4));
  check_hal_error(HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2));
  check_hal_error(HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3));

  while (1) {
    volatile u32 horz_pos = TIM3->CNT, vert_pos = TIM4->CNT;
    if (48 <= horz_pos && horz_pos < 48 + 640 && vert_pos < 480) {
      bool on = ((horz_pos) + (vert_pos >> 5)) & 1;
      VGA_PIXEL_GPIO_Port->BSRR = VGA_PIXEL_Pin << (on ? 0U : 16U);
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
