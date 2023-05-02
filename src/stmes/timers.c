#include "stmes/timers.h"
#include "main.h"
#include <stm32f4xx_hal.h>

TIM_HandleTypeDef htim4;

void MX_TIM4_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  htim4.Instance = TIM4;
  htim4.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 9600 - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = 10000 - 1,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
  if (tim_baseHandle->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) {
  if (tim_baseHandle->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
}
