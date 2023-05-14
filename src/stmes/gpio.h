#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLTN_LED_Pin GPIO_PIN_13
#define BLTN_LED_GPIO_Port GPIOC
#define BLTN_KEY_Pin GPIO_PIN_0
#define BLTN_KEY_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_1
#define SDIO_CD_GPIO_Port GPIOA
#define VGA_PIXEL_Pin GPIO_PIN_5
#define VGA_PIXEL_GPIO_Port GPIOA
#define SDIO_CMD_Pin GPIO_PIN_6
#define SDIO_CMD_GPIO_Port GPIOA
#define VGA_HSYNC_Pin GPIO_PIN_7
#define VGA_HSYNC_GPIO_Port GPIOA
#define SDIO_CLK_Pin GPIO_PIN_15
#define SDIO_CLK_GPIO_Port GPIOB
#define SDIO_D1_Pin GPIO_PIN_8
#define SDIO_D1_GPIO_Port GPIOA
#define SDIO_D2_Pin GPIO_PIN_9
#define SDIO_D2_GPIO_Port GPIOA
#define SDIO_D0_Pin GPIO_PIN_4
#define SDIO_D0_GPIO_Port GPIOB
#define SDIO_D3_Pin GPIO_PIN_5
#define SDIO_D3_GPIO_Port GPIOB
#define VGA_VSYNC_Pin GPIO_PIN_8
#define VGA_VSYNC_GPIO_Port GPIOB

#define GPIO_SET_PIN(port, pin) ((port)->BSRR = (pin));
#define GPIO_RESET_PIN(port, pin) ((port)->BSRR = (pin) << 16U);

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
