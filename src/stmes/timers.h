#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif
