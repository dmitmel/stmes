#include "stmes/drivers/dma.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/time.h"

void dma_deinit_stream(DMA_Stream_TypeDef* dma) {
  // Disable the common interrupts.
  CLEAR_BIT(dma->CR, DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_TCIE);
  // Disable the FIFO interrupts.
  CLEAR_BIT(dma->FCR, DMA_SxFCR_FEIE);
  dma_disable_stream(dma);
  // A 5 millisecond timeout is used by the HAL library.
  Systime stop_deadline = timeout_to_deadline(5);
  while (dma_is_stream_enabled(dma)) {
    ASSERT(systime_now() < stop_deadline);
  }
  dma_clear_interrupt_flags(dma, DMA_ALL_INTERRUPT_FLAGS);
}
