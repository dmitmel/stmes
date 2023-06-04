#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void);

#ifdef __cplusplus
}
#endif
