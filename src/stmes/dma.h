#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif
