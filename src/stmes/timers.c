#include "stmes/timers.h"
#include "stmes/gpio.h"
#include "stmes/main.h"
#include "stmes/utils.h"
#include <stdio.h>
#include <stm32f4xx_hal.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 0,
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
    .MasterOutputTrigger = TIM_TRGO_UPDATE,
    .MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE,
  };
  TIM_OC_InitTypeDef channel1_init = {
    .OCMode = TIM_OCMODE_TIMING,
    .Pulse = (48 - 20) - 1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  TIM_OC_InitTypeDef channel2_init = {
    .OCMode = TIM_OCMODE_PWM1,
    .Pulse = (48 + 640 + 16) - 1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
    .OCFastMode = TIM_OCFAST_DISABLE,
  };
  TIM_OC_InitTypeDef channel3_init = {
    .OCMode = TIM_OCMODE_TIMING,
    .Pulse = (48 + 640 - 26) - 1,
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
  check_hal_error(HAL_TIM_Base_Init(&htim4));
  check_hal_error(HAL_TIM_SlaveConfigSynchro(&htim4, &slave_init));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim4, &master_init));
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
  if (tim_baseHandle->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  } else if (tim_baseHandle->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  } else if (tim_baseHandle->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle) {
  GPIO_InitTypeDef gpio_init;
  if (timHandle->Instance == TIM3) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init = (GPIO_InitTypeDef){
      .Pin = VGA_HSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF2_TIM3,
    };
    HAL_GPIO_Init(VGA_HSYNC_GPIO_Port, &gpio_init);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) {
  if (tim_baseHandle->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
  } else if (tim_baseHandle->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  } else if (tim_baseHandle->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
}
