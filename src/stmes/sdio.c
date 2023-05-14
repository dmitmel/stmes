#include "stmes/sdio.h"
#include "stmes/gpio.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void) {
  hsd.Instance = SDIO;
  hsd.Init = (SDIO_InitTypeDef){
    // NOTE: SDIO_CLOCK_EDGE_FALLING shouldn't be used, see the errata.
    .ClockEdge = SDIO_CLOCK_EDGE_RISING,
    .ClockBypass = SDIO_CLOCK_BYPASS_DISABLE,
    .ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE,
    // .BusWide = SDIO_BUS_WIDE_4B,
    .BusWide = SDIO_BUS_WIDE_1B,
    // NOTE: Enabling HW flow control may cause data corruption, see the errata.
    .HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE,
    .ClockDiv = 0,
  };
}

void HAL_SD_MspInit(SD_HandleTypeDef* hsd) {
  if (hsd->Instance == SDIO) {
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
      .Alternate = GPIO_AF12_SDIO,
    };
    gpio_init.Pin = SDIO_CLK_Pin;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Pin = SDIO_D1_Pin | SDIO_D2_Pin | SDIO_CMD_Pin;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = SDIO_D0_Pin | SDIO_D3_Pin;
    HAL_GPIO_Init(GPIOB, &gpio_init);
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd) {
  if (hsd->Instance == SDIO) {
    __HAL_RCC_SDIO_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, SDIO_CMD_Pin | SDIO_D1_Pin | SDIO_D2_Pin);
    HAL_GPIO_DeInit(GPIOB, SDIO_CLK_Pin | SDIO_D0_Pin | SDIO_D3_Pin);
  }
}

HAL_StatusTypeDef BSP_SD_Init(void) {
  if (!BSP_SD_IsDetected()) {
    return HAL_ERROR;
  }
  if (HAL_SD_Init(&hsd) != HAL_OK) {
    return HAL_ERROR;
  }
  // if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
  //   return HAL_ERROR;
  // }
  return HAL_OK;
}

bool BSP_SD_IsDetected(void) {
  return HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin) == GPIO_PIN_RESET;
}
