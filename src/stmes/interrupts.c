#include "stmes/interrupts.h"
#include "stmes/gpio.h"
#include "stmes/timers.h"
#include <stm32f4xx_hal.h>

void NMI_Handler(void) {
  while (1) {}
}

void HardFault_Handler(void) {
  while (1) {}
}

void MemManage_Handler(void) {
  while (1) {}
}

void BusFault_Handler(void) {
  while (1) {}
}

void UsageFault_Handler(void) {
  while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
  HAL_IncTick();
}

void TIM4_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim == &htim4) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}
