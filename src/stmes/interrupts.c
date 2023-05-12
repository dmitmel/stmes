#include "stmes/interrupts.h"
#include "stmes/dma.h"
#include "stmes/timers.h"
#include "stmes/utils.h"
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

void DMA2_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_tim1_up);
}

void TIM2_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim4);
}
