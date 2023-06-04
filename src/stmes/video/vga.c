#include "stmes/video/vga.h"
#include "stmes/gpio.h"
#include "stmes/main.h"
#include "stmes/utils.h"
#include <math.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_tim.h>

const struct VgaTimings VGA_TIMINGS_640x480_57hz = {
  .pixel_clock_freq = 24000000,
  .active_width = 640,
  .horz_front_porch = 16,
  .hsync_pulse = -96,
  .horz_back_porch = 48,
  .active_height = 480,
  .vert_front_porch = 10,
  .vsync_pulse = -2,
  .vert_back_porch = 33,
};

const struct VgaTimings VGA_TIMINGS_640x480_60hz = {
  .pixel_clock_freq = 25175000,
  .active_width = 640,
  .horz_front_porch = 16,
  .hsync_pulse = -96,
  .horz_back_porch = 48,
  .active_height = 480,
  .vert_front_porch = 10,
  .vsync_pulse = -2,
  .vert_back_porch = 33,
};

const struct VgaTimings VGA_TIMINGS_800x600_60hz = {
  .pixel_clock_freq = 40000000,
  .active_width = 800,
  .horz_front_porch = 40,
  .hsync_pulse = 128,
  .horz_back_porch = 88,
  .active_height = 600,
  .vert_front_porch = 1,
  .vsync_pulse = 4,
  .vert_back_porch = 23,
};

const struct VgaTimings VGA_TIMINGS_1024x768_60hz = {
  .pixel_clock_freq = 65000000,
  .active_width = 1024,
  .horz_front_porch = 24,
  .hsync_pulse = 136,
  .horz_back_porch = 160,
  .active_height = 768,
  .vert_front_porch = 3,
  .vsync_pulse = 6,
  .vert_back_porch = 29,
};

TIM_HandleTypeDef vga_pixel_timer;
TIM_HandleTypeDef vga_hsync_timer;
TIM_HandleTypeDef vga_vsync_timer;
DMA_HandleTypeDef vga_pixel_dma;

void vga_init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  {
    LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_PIXEL_ALL_PINS,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    HAL_GPIO_Init(VGA_PIXEL_GPIO_Port, &gpio_init);
  }

  {
    DMA_HandleTypeDef* hdma = &vga_pixel_dma;
    hdma->Instance = DMA2_Stream5;
    hdma->Init = (DMA_InitTypeDef){
      .Channel = DMA_CHANNEL_6,
      .Direction = DMA_PERIPH_TO_MEMORY,
      .PeriphInc = DMA_PINC_ENABLE,
      .MemInc = DMA_MINC_DISABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_NORMAL,
      .Priority = DMA_PRIORITY_HIGH,
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL,
      .MemBurst = DMA_MBURST_SINGLE,
      .PeriphBurst = DMA_PBURST_INC4,
    };
    check_hal_error(HAL_DMA_Init(hdma));
  }

  {
    TIM_HandleTypeDef* htim = &vga_pixel_timer;
    htim->Instance = TIM1;
    htim->Init = (TIM_Base_InitTypeDef){
      .Prescaler = 4 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 2 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    check_hal_error(HAL_TIM_Base_Init(htim));

    TIM_ClockConfigTypeDef clock_init = {
      .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
    };
    check_hal_error(HAL_TIM_ConfigClockSource(htim, &clock_init));

    TIM_SlaveConfigTypeDef slave_init = {
      .SlaveMode = TIM_SLAVEMODE_TRIGGER,
      .InputTrigger = TIM_TS_ITR2,
    };
    check_hal_error(HAL_TIM_SlaveConfigSynchro(htim, &slave_init));

    TIM_MasterConfigTypeDef master_init = {
      .MasterOutputTrigger = TIM_TRGO_RESET,
      .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
    };
    check_hal_error(HAL_TIMEx_MasterConfigSynchronization(htim, &master_init));

    __HAL_LINKDMA(&vga_pixel_timer, hdma[TIM_DMA_ID_UPDATE], vga_pixel_dma);
    __HAL_TIM_ENABLE_DMA(&vga_pixel_timer, TIM_DMA_UPDATE);
  }

  {
    TIM_HandleTypeDef* htim = &vga_hsync_timer;
    htim->Instance = TIM3;
    htim->Init = (TIM_Base_InitTypeDef){
      .Prescaler = 0,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 0,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    check_hal_error(HAL_TIM_Base_Init(htim));

    TIM_ClockConfigTypeDef clock_init = {
      .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
    };
    check_hal_error(HAL_TIM_ConfigClockSource(htim, &clock_init));

    check_hal_error(HAL_TIM_OC_Init(htim));
    check_hal_error(HAL_TIM_PWM_Init(htim));

    TIM_MasterConfigTypeDef master_init = {
      .MasterOutputTrigger = TIM_TRGO_OC1,
      .MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE,
    };
    check_hal_error(HAL_TIMEx_MasterConfigSynchronization(htim, &master_init));

    TIM_OC_InitTypeDef channel1_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel1_init, TIM_CHANNEL_1));
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_1);

    TIM_OC_InitTypeDef channel2_init = {
      .OCMode = TIM_OCMODE_PWM1,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_PWM_ConfigChannel(htim, &channel2_init, TIM_CHANNEL_2));

    TIM_OC_InitTypeDef channel3_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel3_init, TIM_CHANNEL_3));
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_3);

    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_HSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF2_TIM3,
    };
    HAL_GPIO_Init(VGA_HSYNC_GPIO_Port, &gpio_init);
  }

  {
    TIM_HandleTypeDef* htim = &vga_vsync_timer;
    htim->Instance = TIM9;
    htim->Init = (TIM_Base_InitTypeDef){
      .Prescaler = 0,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 0,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    check_hal_error(HAL_TIM_Base_Init(htim));
    check_hal_error(HAL_TIM_OC_Init(htim));
    check_hal_error(HAL_TIM_PWM_Init(htim));

    TIM_SlaveConfigTypeDef slave_init = {
      .SlaveMode = TIM_SLAVEMODE_EXTERNAL1,
      .InputTrigger = TIM_TS_ITR1,
    };
    check_hal_error(HAL_TIM_SlaveConfigSynchro(htim, &slave_init));

    TIM_MasterConfigTypeDef master_init = {
      .MasterOutputTrigger = TIM_TRGO_RESET,
      .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
    };
    check_hal_error(HAL_TIMEx_MasterConfigSynchronization(htim, &master_init));

    TIM_OC_InitTypeDef channel1_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel1_init, TIM_CHANNEL_1));

    TIM_OC_InitTypeDef channel2_init = {
      .OCMode = TIM_OCMODE_PWM1,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_PWM_ConfigChannel(htim, &channel2_init, TIM_CHANNEL_2));

    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_VSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF3_TIM9,
    };
    HAL_GPIO_Init(VGA_VSYNC_GPIO_Port, &gpio_init);
  }
}

static u32 calc_timer_prescaler(u32 apbclk, u32 frequency, u32 max_prescaler) {
  if (frequency >= apbclk) return 1;
  u32 prescaler = round_div(apbclk, frequency);
  // This will divide the prescaler until it is smaller than the max value.
  return prescaler / ceil_div(prescaler, max_prescaler);
}

void vga_apply_timings(const struct VgaTimings* ts) {
  // APB1 timers: TIM2, TIM3, TIM4, TIM5
  u32 apb1_freq = HAL_RCC_GetPCLK1Freq() * (LL_RCC_GetAPB1Prescaler() == RCC_HCLK_DIV1 ? 1 : 2);
  // APB2 timers: TIM1, TIM9, TIM10, TIM11
  u32 apb2_freq = HAL_RCC_GetPCLK2Freq();

  TIM_TypeDef* pixel_tim = vga_pixel_timer.Instance; // TIM1, clocked from APB2
  TIM_TypeDef* hsync_tim = vga_hsync_timer.Instance; // TIM3, clocked from APB1
  TIM_TypeDef* vsync_tim = vga_vsync_timer.Instance; // TIM9, clocked from TIM3

  u32 hsync_width = ABS(ts->hsync_pulse), vsync_width = ABS(ts->vsync_pulse),
      hsync_polarity = ts->hsync_pulse < 0 ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW,
      vsync_polarity = ts->vsync_pulse < 0 ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW;
  u32 active_width = ts->active_width, active_height = ts->active_height,
      horz_front_porch = ts->horz_front_porch, horz_back_porch = ts->horz_back_porch,
      vert_front_porch = ts->vert_front_porch, vert_back_porch = ts->vert_back_porch;
  u32 whole_line = active_width + horz_front_porch + hsync_width + horz_back_porch,
      whole_frame = active_height + vert_front_porch + vsync_width + vert_back_porch;

  u32 apb1_pixel_prescaler = calc_timer_prescaler(apb1_freq, ts->pixel_clock_freq, UINT16_MAX),
      apb2_pixel_prescaler = calc_timer_prescaler(apb2_freq, ts->pixel_clock_freq, UINT16_MAX);

  if (apb1_freq / apb1_pixel_prescaler != ts->pixel_clock_freq) {
    u32 pixel_clk_freq = apb1_freq / apb1_pixel_prescaler;
    float pixel_freq_ratio = (float)pixel_clk_freq / (float)ts->pixel_clock_freq;
    hsync_width = roundf(hsync_width * pixel_freq_ratio);
    active_width = roundf(active_width * pixel_freq_ratio);
    horz_back_porch = roundf(horz_back_porch * pixel_freq_ratio);
    horz_front_porch = roundf(horz_front_porch * pixel_freq_ratio);
    whole_line = roundf(whole_line * pixel_freq_ratio);
  }

  LL_TIM_SetPrescaler(hsync_tim, apb1_pixel_prescaler - 1);
  LL_TIM_SetAutoReload(hsync_tim, whole_line - 1);
  LL_TIM_OC_SetCompareCH1(hsync_tim, horz_back_porch);
  LL_TIM_OC_SetCompareCH2(hsync_tim, whole_line - hsync_width);
  LL_TIM_OC_SetPolarity(hsync_tim, LL_TIM_CHANNEL_CH2, hsync_polarity);
  LL_TIM_OC_SetCompareCH3(hsync_tim, horz_back_porch + active_width);

  LL_TIM_SetPrescaler(vsync_tim, 0);
  LL_TIM_SetAutoReload(vsync_tim, whole_frame - 1);
  LL_TIM_OC_SetCompareCH1(vsync_tim, vert_back_porch + active_height);
  LL_TIM_OC_SetCompareCH2(vsync_tim, whole_frame - vsync_width);
  LL_TIM_OC_SetPolarity(vsync_tim, LL_TIM_CHANNEL_CH2, vsync_polarity);

  // Apply the prescaler changes immediately
  LL_TIM_GenerateEvent_UPDATE(pixel_tim);
  LL_TIM_GenerateEvent_UPDATE(hsync_tim);
  LL_TIM_GenerateEvent_UPDATE(vsync_tim);
}

void vga_start(void) {
  check_hal_error(HAL_TIM_Base_Start(&vga_vsync_timer));
  check_hal_error(HAL_TIM_OC_Start_IT(&vga_vsync_timer, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_PWM_Start(&vga_vsync_timer, TIM_CHANNEL_2));
  check_hal_error(HAL_TIM_PWM_Start(&vga_hsync_timer, TIM_CHANNEL_2));
  check_hal_error(HAL_TIM_OC_Start_IT(&vga_hsync_timer, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_OC_Start_IT(&vga_hsync_timer, TIM_CHANNEL_3));
  check_hal_error(HAL_TIM_Base_Start(&vga_hsync_timer));
}

void vga_deinit(void) {
  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn);

  HAL_TIM_Base_DeInit(&vga_pixel_timer);
  HAL_TIM_Base_DeInit(&vga_hsync_timer);
  HAL_TIM_Base_DeInit(&vga_vsync_timer);
  HAL_DMA_DeInit(&vga_pixel_dma);

  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM9_CLK_DISABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
}
