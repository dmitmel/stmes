#include "stmes/timers.h"
#include "stmes/kernel/crash.h"
#include <stm32f4xx_hal.h>

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init = (TIM_Base_InitTypeDef){
    .Prescaler = 48000 - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = UINT32_MAX,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  TIM_ClockConfigTypeDef clock_init = {
    .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
  };
  TIM_MasterConfigTypeDef master_init = {
    .MasterOutputTrigger = TIM_TRGO_RESET,
    .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
  };
  check_hal_error(HAL_TIM_Base_Init(&htim2));
  check_hal_error(HAL_TIM_ConfigClockSource(&htim2, &clock_init));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_init));
  __HAL_DBGMCU_FREEZE_TIM2(); // Pause during debug
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
}
