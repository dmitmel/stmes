#include "stmes/gpio.h"
#include <stm32f4xx_hal.h>

void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init;

  HAL_GPIO_WritePin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_Pin, GPIO_PIN_RESET);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = VGA_PIXEL_Pin,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(VGA_PIXEL_GPIO_Port, &gpio_init);
}
