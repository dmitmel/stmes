#include "stmes/dma.h"
#include <stm32f4xx_hal.h>

DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}
