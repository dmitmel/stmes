#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_hal.h>

#define VGA_PIXEL_Pin GPIO_PIN_5
#define VGA_PIXEL_GPIO_Port GPIOA
#define VGA_HSYNC_Pin GPIO_PIN_7
#define VGA_HSYNC_GPIO_Port GPIOA
#define VGA_VSYNC_Pin GPIO_PIN_8
#define VGA_VSYNC_GPIO_Port GPIOB

#define GPIO_SET_PIN(port, pin) ((port)->BSRR = (pin));
#define GPIO_RESET_PIN(port, pin) ((port)->BSRR = (pin) << 16U);

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
