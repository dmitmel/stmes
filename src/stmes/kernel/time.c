#include "stmes/kernel/time.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_tim.h>

// #define HWTIMER_PRESCALER (1 << 8)
#define HWTIMER_PRESCALER 2

static TIM_HandleTypeDef hwtimer;

void hwtimer_init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  TIM_HandleTypeDef* htim = &hwtimer;
  htim->Instance = TIM5;
  htim->Init = (TIM_Base_InitTypeDef){
    .Prescaler = 96000 / HWTIMER_PRESCALER,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = UINT32_MAX,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
  };
  check_hal_error(HAL_TIM_Base_Init(htim));

  TIM_ClockConfigTypeDef clock_init = {
    .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
  };
  check_hal_error(HAL_TIM_ConfigClockSource(htim, &clock_init));

  check_hal_error(HAL_TIM_OC_Init(htim));

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

  __HAL_DBGMCU_FREEZE_TIM5(); // Pause during debug

  check_hal_error(HAL_TIM_OC_Start_IT(&hwtimer, TIM_CHANNEL_1));
  __HAL_TIM_CLEAR_IT(&hwtimer, TIM_IT_UPDATE);
}

void hwtimer_deinit(void) {
  HAL_NVIC_DisableIRQ(TIM5_IRQn);

  check_hal_error(HAL_TIM_Base_DeInit(&hwtimer));

  __HAL_RCC_TIM5_CLK_DISABLE();
}

__STATIC_FORCEINLINE u32 __hwtimer_read(void) {
  return LL_TIM_GetCounter(TIM5) / HWTIMER_PRESCALER;
}

// TODO: change to ticks
u32 hwtimer_read(void) {
  return __hwtimer_read();
}

void hwtimer_set_alarm(u32 value) {
  LL_TIM_OC_SetCompareCH1(TIM5, value * HWTIMER_PRESCALER);
}

void TIM5_IRQHandler(void) {
  TIM_TypeDef* hwtimer = TIM5;
  if (likely(LL_TIM_IsActiveFlag_CC1(hwtimer))) {
    LL_TIM_ClearFlag_CC1(hwtimer);
    yield_from_isr();
  }
}

Instant systime_now(void) {
  TIM_TypeDef* hwtimer = TIM5;
  u32 primask = __get_PRIMASK();
  __disable_irq();
  while (true) {
    static volatile u32 systime_epoch;
    u32 epoch = systime_epoch;
    u32 hwtime = __hwtimer_read();
    // Check if the timer has overflown before calling this function or while
    // we were reading the counter values in the previous two lines.
    // TODO: Can this be safely down without establishing a critical section?
    if (unlikely(LL_TIM_IsActiveFlag_UPDATE(hwtimer))) {
      LL_TIM_ClearFlag_UPDATE(hwtimer);
      // TODO: Do something about overflows!
      systime_epoch += 1;
      continue;
    }
    __set_PRIMASK(primask);
    // Now we can be sure that both counters represent the same epoch.
    return ((u64)epoch << 32) | (u64)hwtime;
  }
}

// clang-format off
HAL_StatusTypeDef HAL_InitTick(u32 priority) { UNUSED(priority); return HAL_OK; }
u32 HAL_GetTick(void) { return __hwtimer_read(); }
void HAL_SuspendTick(void) {}
void HAL_ResumeTick(void) {}
// clang-format on

void SysTick_Handler(void) {
  HAL_IncTick();
}
