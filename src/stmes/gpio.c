#include "stmes/gpio.h"

void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init;

  HAL_GPIO_WritePin(GPIOA, VGA_VSYNC_Pin | VGA_PIXEL_Pin, GPIO_PIN_RESET);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = VGA_VSYNC_Pin | VGA_PIXEL_Pin,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
}
