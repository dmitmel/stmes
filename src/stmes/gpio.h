#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_hal.h>

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
