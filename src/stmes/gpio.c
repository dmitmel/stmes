#include "stmes/gpio.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/task.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_exti.h>
#include <stm32f4xx_ll_gpio.h>

struct Notification gpio_button_notification;

void gpio_init(void) {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  LL_GPIO_SetOutputPin(BLTN_LED_GPIO_PORT, BLTN_LED_PIN);

  GPIO_InitTypeDef gpio_init;

  gpio_init = (GPIO_InitTypeDef){
    .Pin = BLTN_KEY_PIN,
    .Mode = GPIO_MODE_IT_RISING_FALLING,
    .Pull = GPIO_PULLUP,
  };
  HAL_GPIO_Init(BLTN_KEY_GPIO_PORT, &gpio_init);
  HAL_NVIC_SetPriority(BLTN_KEY_EXTI_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(BLTN_KEY_EXTI_IRQn);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = BLTN_LED_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(BLTN_LED_GPIO_PORT, &gpio_init);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = SDIO_CD_PIN,
    .Mode = GPIO_MODE_INPUT,
    .Pull = GPIO_PULLUP,
  };
  HAL_GPIO_Init(SDIO_CD_GPIO_PORT, &gpio_init);

  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_15,
    .Mode = GPIO_MODE_ANALOG,
    .Pull = GPIO_NOPULL,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
}

void EXTI0_IRQHandler(void) {
  if (LL_EXTI_IsActiveFlag_0_31(BLTN_KEY_PIN)) {
    LL_EXTI_ClearFlag_0_31(BLTN_KEY_PIN);
    if (task_notify(&gpio_button_notification)) {
      task_yield_from_isr();
    }
  }
}
