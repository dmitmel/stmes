#include "stmes/timers.h"
#include "stmes/dma.h"
#include "stmes/gpio.h"
#include "stmes/main.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void MX_TIM1_Init(void) {
  htim1.Instance = TIM1;
  htim1.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 0,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = 2 - 1,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  TIM_ClockConfigTypeDef clock_source_init = {
    .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
  };
  TIM_SlaveConfigTypeDef slave_init = {
    .SlaveMode = TIM_SLAVEMODE_TRIGGER,
    .InputTrigger = TIM_TS_ITR2,
  };
  TIM_MasterConfigTypeDef master_init = {
    .MasterOutputTrigger = TIM_TRGO_RESET,
    .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
  };
  check_hal_error(HAL_TIM_Base_Init(&htim1));
  check_hal_error(HAL_TIM_ConfigClockSource(&htim1, &clock_source_init));
  check_hal_error(HAL_TIM_SlaveConfigSynchro(&htim1, &slave_init));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim1, &master_init));
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
}

void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 48000 - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = UINT32_MAX,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  TIM_ClockConfigTypeDef clock_source_init = {
    .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
  };
  TIM_MasterConfigTypeDef master_init = {
    .MasterOutputTrigger = TIM_TRGO_RESET,
    .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
  };
  check_hal_error(HAL_TIM_Base_Init(&htim2));
  check_hal_error(HAL_TIM_ConfigClockSource(&htim2, &clock_source_init));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_init));
  __HAL_DBGMCU_FREEZE_TIM2(); // Pause during debug
}

void MX_TIM3_Init(void) {
  htim3.Instance = TIM3;
  htim3.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 4 - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = 800 - 1,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  TIM_ClockConfigTypeDef clock_source_init = {
    .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
  };
  TIM_MasterConfigTypeDef master_init = {
    .MasterOutputTrigger = TIM_TRGO_OC1,
    .MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE,
  };
  TIM_OC_InitTypeDef channel1_init = {
    .OCMode = TIM_OCMODE_TIMING,
    .Pulse = 48,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  TIM_OC_InitTypeDef channel2_init = {
    .OCMode = TIM_OCMODE_PWM1,
    .Pulse = 48 + 640 + 16,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  TIM_OC_InitTypeDef channel3_init = {
    .OCMode = TIM_OCMODE_TIMING,
    .Pulse = 48 + 640,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  check_hal_error(HAL_TIM_Base_Init(&htim3));
  check_hal_error(HAL_TIM_ConfigClockSource(&htim3, &clock_source_init));
  check_hal_error(HAL_TIM_OC_Init(&htim3));
  check_hal_error(HAL_TIM_PWM_Init(&htim3));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim3, &master_init));
  check_hal_error(HAL_TIM_OC_ConfigChannel(&htim3, &channel1_init, TIM_CHANNEL_1));
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  check_hal_error(HAL_TIM_PWM_ConfigChannel(&htim3, &channel2_init, TIM_CHANNEL_2));
  check_hal_error(HAL_TIM_OC_ConfigChannel(&htim3, &channel3_init, TIM_CHANNEL_3));
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_3);
  HAL_TIM_MspPostInit(&htim3);
}

void MX_TIM4_Init(void) {
  htim4.Instance = TIM4;
  htim4.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 0,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = 525 - 1,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  TIM_SlaveConfigTypeDef slave_init = {
    .SlaveMode = TIM_SLAVEMODE_EXTERNAL1,
    .InputTrigger = TIM_TS_ITR2,
  };
  TIM_MasterConfigTypeDef master_init = {
    .MasterOutputTrigger = TIM_TRGO_RESET,
    .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
  };
  TIM_OC_InitTypeDef channel1_init = {
    .OCMode = TIM_OCMODE_TIMING,
    .Pulse = 33,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  TIM_OC_InitTypeDef channel2_init = {
    .OCMode = TIM_OCMODE_TIMING,
    .Pulse = 33 + 480,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  TIM_OC_InitTypeDef channel3_init = {
    .OCMode = TIM_OCMODE_PWM1,
    .Pulse = 33 + 480 + 10,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  check_hal_error(HAL_TIM_Base_Init(&htim4));
  check_hal_error(HAL_TIM_OC_Init(&htim4));
  check_hal_error(HAL_TIM_PWM_Init(&htim4));
  check_hal_error(HAL_TIM_SlaveConfigSynchro(&htim4, &slave_init));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim4, &master_init));
  check_hal_error(HAL_TIM_OC_ConfigChannel(&htim4, &channel1_init, TIM_CHANNEL_1));
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
  check_hal_error(HAL_TIM_OC_ConfigChannel(&htim4, &channel2_init, TIM_CHANNEL_2));
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_2);
  check_hal_error(HAL_TIM_OC_ConfigChannel(&htim4, &channel3_init, TIM_CHANNEL_3));
  HAL_TIM_MspPostInit(&htim4);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    hdma_tim1_up.Instance = DMA2_Stream5;
    hdma_tim1_up.Init = (DMA_InitTypeDef){
      .Channel = DMA_CHANNEL_6,
      .Direction = DMA_MEMORY_TO_PERIPH,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_NORMAL,
      .Priority = DMA_PRIORITY_HIGH,
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_FULL,
      .MemBurst = DMA_MBURST_SINGLE,
      .PeriphBurst = DMA_PBURST_SINGLE,
    };
    check_hal_error(HAL_DMA_Init(&hdma_tim1_up));
    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_UPDATE], hdma_tim1_up);
  } else if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  } else if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  } else if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM3) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_HSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF2_TIM3,
    };
    HAL_GPIO_Init(VGA_HSYNC_GPIO_Port, &gpio_init);
  } else if (htim->Instance == TIM4) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_VSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF2_TIM4,
    };
    HAL_GPIO_Init(VGA_VSYNC_GPIO_Port, &gpio_init);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_DMA_DeInit(htim->hdma[TIM_DMA_ID_UPDATE]);
  } else if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
  } else if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  } else if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
}
