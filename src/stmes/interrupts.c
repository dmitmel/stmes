#include "stmes/interrupts.h"
#include "stmes/dma.h"
#include "stmes/sdio.h"
#include "stmes/video/vga.h"
#include <stm32f4xx_hal.h>

// A workaround for errata entry 1.2: "VDIV or VSQRT instructions might not
// complete correctly when very short ISRs are used".
__STATIC_FORCEINLINE void nop_interrupt(void) {
  // The errata recommends having no less than 3 instructions (including the
  // `bx lr`) in every ISR, but let's add more to be sure.
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

__USED void NMI_Handler(void) {
  while (true) {}
}

__USED void SVC_Handler(void) {
  nop_interrupt();
}

__USED void DebugMon_Handler(void) {
  nop_interrupt();
}

__USED void PendSV_Handler(void) {
  nop_interrupt();
}

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
  vga_vsync_timer_isr();
}

__USED void TIM2_IRQHandler(void) {
  vga_hsync_timer_isr();
}
