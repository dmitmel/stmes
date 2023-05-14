#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void);

HAL_StatusTypeDef BSP_SD_Init(void);
bool BSP_SD_IsDetected(void);

#ifdef __cplusplus
}
#endif
