#include "stmes/timers.h"
#include "stmes/kernel/crash.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_tim.h>

TIM_HandleTypeDef htim5;

void MX_TIM5_Init(void) {
  htim5.Instance = TIM5;
  htim5.Init = (TIM_Base_InitTypeDef){
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
  check_hal_error(HAL_TIM_Base_Init(&htim5));
  check_hal_error(HAL_TIM_ConfigClockSource(&htim5, &clock_init));
  check_hal_error(HAL_TIMEx_MasterConfigSynchronization(&htim5, &master_init));
  __HAL_DBGMCU_FREEZE_TIM5(); // Pause during debug
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_DISABLE();
  }
}

u32 HAL_GetTick(void) {
  return LL_TIM_GetCounter(TIM5) / 2;
}

void HAL_SuspendTick(void) {
  LL_TIM_DisableCounter(TIM5);
}

void HAL_ResumeTick(void) {
  LL_TIM_EnableCounter(TIM5);
}
