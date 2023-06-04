#include "stmes/interrupts.h"
#include "stmes/dma.h"
#include "stmes/main.h"
#include "stmes/sdio.h"
#include "stmes/timers.h"
#include "stmes/video/vga.h"
#include <stm32f4xx_hal.h>

void NMI_Handler(void) {
  while (true) {}
}

void HardFault_Handler(void) {
  Error_Handler();
}

void MemManage_Handler(void) {
  Error_Handler();
}

void BusFault_Handler(void) {
  Error_Handler();
}

void UsageFault_Handler(void) {
  Error_Handler();
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
  HAL_IncTick();
}

void SDIO_IRQHandler(void) {
  HAL_SD_IRQHandler(&hsd);
}

void DMA2_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

void DMA2_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_sdio_tx);
}

void TIM1_BRK_TIM9_IRQHandler(void) {
  HAL_TIM_IRQHandler(&vga_vsync_timer);
  HAL_TIM_IRQHandler(&vga_pixel_timer);
}

void TIM2_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&vga_hsync_timer);
}
