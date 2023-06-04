#include "stmes/gpio.h"
#include <stm32f4xx_hal.h>

void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(BLTN_LED_GPIO_Port, BLTN_LED_Pin, GPIO_PIN_SET);

  GPIO_InitTypeDef gpio_init;

  gpio_init = (GPIO_InitTypeDef){
    .Pin = BLTN_KEY_Pin,
    .Mode = GPIO_MODE_INPUT,
    .Pull = GPIO_PULLUP,
  };
  HAL_GPIO_Init(BLTN_KEY_GPIO_Port, &gpio_init);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = BLTN_LED_Pin,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(BLTN_LED_GPIO_Port, &gpio_init);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = SDIO_CD_Pin,
    .Mode = GPIO_MODE_INPUT,
    .Pull = GPIO_PULLUP,
  };
  HAL_GPIO_Init(SDIO_CD_GPIO_Port, &gpio_init);

  gpio_init = (GPIO_InitTypeDef){
    .Pin =
      GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15,
    .Mode = GPIO_MODE_ANALOG,
    .Pull = GPIO_NOPULL,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
}
