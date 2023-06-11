#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);

__NO_RETURN void Error_Handler(void);

#ifdef __cplusplus
}
#endif
