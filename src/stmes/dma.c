#include "stmes/dma.h"
#include <stm32f4xx_hal.h>

void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
}
