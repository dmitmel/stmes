#include "stmes/kernel/time.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_tim.h>

static TIM_HandleTypeDef hwtimer;
static volatile u32 systime_epoch;

void hwtimer_init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM5_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  u32 apb1_freq = HAL_RCC_GetPCLK1Freq() * (LL_RCC_GetAPB1Prescaler() == RCC_HCLK_DIV1 ? 1 : 2);
  u32 hwtimer_freq = 1000 * SYSTIME_TICKS_PER_MS;
  ASSERT(apb1_freq % hwtimer_freq == 0);
  u32 prescaler = apb1_freq / hwtimer_freq;
  ASSERT(prescaler - 1 <= UINT16_MAX);

  TIM_HandleTypeDef* htim = &hwtimer;
  htim->Instance = TIM5;
  htim->Init = (TIM_Base_InitTypeDef){
    .Prescaler = prescaler - 1,
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

u32 hwtimer_read(void) {
  return LL_TIM_GetCounter(TIM5);
}

void hwtimer_set_alarm(u32 value) {
  LL_TIM_OC_SetCompareCH1(TIM5, value);
}

void TIM5_IRQHandler(void) {
  TIM_TypeDef* hwtimer = TIM5;
  if (likely(LL_TIM_IsActiveFlag_CC1(hwtimer))) {
    LL_TIM_ClearFlag_CC1(hwtimer);
    task_yield_from_isr();
  }
  if (unlikely(LL_TIM_IsActiveFlag_UPDATE(hwtimer))) {
    LL_TIM_ClearFlag_UPDATE(hwtimer);
    // The addition doesn't need to be atomic since this interrupt is the only
    // piece of code modifying this variable.
    systime_epoch += 1;
  }
}

// NOTE: This function shouldn't be called from within interrupt handlers, as
// it will miss overflows of the hardware timer used for timekeeping!
Systime systime_now(void) {
  // u32 irq = interrupt_number();
  // ASSERT(irq == 0 || irq == SVCall_IRQn + 16 || irq == PendSV_IRQn + 16);
  while (true) {
    u32 epoch = systime_epoch;
    u32 hwtime = LL_TIM_GetCounter(TIM5);
    u32 epoch2 = systime_epoch;
    // Check that the timer has not overflown while we were reading the counter value.
    if (epoch2 == epoch) {
      // Now we can be sure that both counters represent the same epoch.
      return ((u64)epoch << 32) | (u64)hwtime;
    }
  }
}

// clang-format off
HAL_StatusTypeDef HAL_InitTick(u32 priority) { UNUSED(priority); return HAL_OK; }
u32 HAL_GetTick(void) { return systime_as_millis(systime_now()); }
void HAL_Delay(u32 delay) { task_sleep(delay); }
void HAL_SuspendTick(void) {}
void HAL_ResumeTick(void) {}
// clang-format on
