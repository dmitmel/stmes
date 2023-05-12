#include "stmes/dma.h"
#include <stm32f4xx_hal.h>

DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim3_ch4_up;

void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}
