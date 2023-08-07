#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

void MX_SDIO_SD_Init(void);

HAL_StatusTypeDef BSP_SD_Init(void);
bool BSP_SD_IsDetected(void);

enum SdioResponseFormat {
  SDIO_RESPONSE_NONE,
  SDIO_RESPONSE_R1,
  SDIO_RESPONSE_R2,
  SDIO_RESPONSE_R3,
  SDIO_RESPONSE_R4, // TODO
  SDIO_RESPONSE_R5, // TODO
  SDIO_RESPONSE_R6,
  SDIO_RESPONSE_R7,
};

u32 sdio_send_command(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd, enum SdioResponseFormat res);
u32 sdio_send_command_r1(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd);

#ifdef __cplusplus
}
#endif
