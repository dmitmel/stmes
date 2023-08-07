#pragma once

#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void);

HAL_StatusTypeDef BSP_SD_Init(void);
bool BSP_SD_IsDetected(void);

enum SdioResponseFormat {
  SDIO_RESPONSE_NONE,
  SDIO_RESPONSE_R1,
  SDIO_RESPONSE_R2,
  SDIO_RESPONSE_R3,
  SDIO_RESPONSE_R4,
  SDIO_RESPONSE_R5,
  SDIO_RESPONSE_R6,
  SDIO_RESPONSE_R7,
};

u32 sdio_send_command(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd, enum SdioResponseFormat res);
u32 sdio_send_command_r1(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd);

__NO_RETURN void crash_on_sd_error(u32 code, const char* file, u32 line);

#define check_sd_error(hsd, expr)                              \
  do {                                                         \
    HAL_StatusTypeDef code = (expr);                           \
    if (unlikely(code != HAL_OK)) {                            \
      crash_collect_registers();                               \
      crash_on_sd_error((hsd)->ErrorCode, __FILE__, __LINE__); \
    }                                                          \
  } while (0)

#ifdef __cplusplus
}
#endif
