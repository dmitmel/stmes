#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLTN_LED_Pin GPIO_PIN_13
#define BLTN_LED_GPIO_Port GPIOC
#define BLTN_KEY_Pin GPIO_PIN_0
#define BLTN_KEY_GPIO_Port GPIOA
#define BLTN_KEY_EXTI_IRQn EXTI0_IRQn

#define SDIO_CD_Pin GPIO_PIN_1
#define SDIO_CD_GPIO_Port GPIOA
#define SDIO_CMD_Pin GPIO_PIN_6
#define SDIO_CMD_GPIO_Port GPIOA
#define SDIO_CLK_Pin GPIO_PIN_15
#define SDIO_CLK_GPIO_Port GPIOB
#define SDIO_D0_Pin GPIO_PIN_4
#define SDIO_D0_GPIO_Port GPIOB
#define SDIO_D1_Pin GPIO_PIN_8
#define SDIO_D1_GPIO_Port GPIOA
#define SDIO_D2_Pin GPIO_PIN_9
#define SDIO_D2_GPIO_Port GPIOA
#define SDIO_D3_Pin GPIO_PIN_5
#define SDIO_D3_GPIO_Port GPIOB

#define VGA_HSYNC_GPIO_Port GPIOA
#define VGA_HSYNC_Pin GPIO_PIN_2
#define VGA_VSYNC_GPIO_Port GPIOA
#define VGA_VSYNC_Pin GPIO_PIN_3
#define VGA_RGB_GPIO_Port GPIOB
#define VGA_BLUE1_Pin GPIO_PIN_0
#define VGA_BLUE2_Pin GPIO_PIN_1
#define VGA_BLUE3_Pin GPIO_PIN_2
#define VGA_BLUE4_Pin GPIO_PIN_3
#define VGA_GREEN1_Pin GPIO_PIN_6
#define VGA_GREEN2_Pin GPIO_PIN_7
#define VGA_GREEN3_Pin GPIO_PIN_8
#define VGA_GREEN4_Pin GPIO_PIN_9
#define VGA_RED1_Pin GPIO_PIN_10
#define VGA_RED2_Pin GPIO_PIN_12
#define VGA_RED3_Pin GPIO_PIN_13
#define VGA_RED4_Pin GPIO_PIN_14

#define VGA_ALL_RED_PINS (VGA_RED1_Pin | VGA_RED2_Pin | VGA_RED3_Pin | VGA_RED4_Pin)
#define VGA_ALL_GREEN_PINS (VGA_GREEN1_Pin | VGA_GREEN2_Pin | VGA_GREEN3_Pin | VGA_GREEN4_Pin)
#define VGA_ALL_BLUE_PINS (VGA_BLUE1_Pin | VGA_BLUE2_Pin | VGA_BLUE3_Pin | VGA_BLUE4_Pin)
#define VGA_ALL_RGB_PINS (VGA_ALL_RED_PINS | VGA_ALL_GREEN_PINS | VGA_ALL_BLUE_PINS)
#define VGA_ALL_RGB_PINS_RESET (VGA_ALL_RGB_PINS << 16)

extern struct Notification gpio_button_notification;

void gpio_init(void);

#ifdef __cplusplus
}
#endif
