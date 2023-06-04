#include "stmes/interrupts.h"
#include "stmes/dma.h"
#include "stmes/main.h"
#include "stmes/sdio.h"
#include "stmes/timers.h"
#include "stmes/video/vga.h"
#include <stm32f4xx_hal.h>

__USED void NMI_Handler(void) {
  while (true) {}
}

__USED void HardFault_Handler(void) {
  Error_Handler();
}

__USED void MemManage_Handler(void) {
  Error_Handler();
}

__USED void BusFault_Handler(void) {
  Error_Handler();
}

__USED void UsageFault_Handler(void) {
  Error_Handler();
}

__USED void SVC_Handler(void) {}

__USED void DebugMon_Handler(void) {}

__USED void PendSV_Handler(void) {}

__USED void SysTick_Handler(void) {
  HAL_IncTick();
}

__USED void SDIO_IRQHandler(void) {
  HAL_SD_IRQHandler(&hsd);
}

__USED void DMA2_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

__USED void DMA2_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_sdio_tx);
}

__USED void TIM1_BRK_TIM9_IRQHandler(void) {
  HAL_TIM_IRQHandler(&vga_vsync_timer);
  HAL_TIM_IRQHandler(&vga_pixel_timer);
}

__USED void TIM2_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim2);
}

__USED void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&vga_hsync_timer);
}
