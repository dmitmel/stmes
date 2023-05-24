#include "stmes/sdio.h"
#include "stmes/dma.h"
#include "stmes/gpio.h"
#include "stmes/main.h"
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
    .BusWide = SDIO_BUS_WIDE_1B,
    // NOTE: Enabling HW flow control may cause data corruption, see the errata.
    .HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE,
    .ClockDiv = SDIO_INIT_CLK_DIV,
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

    hdma_sdio_rx.Instance = DMA2_Stream3;
    hdma_sdio_rx.Init = (DMA_InitTypeDef){
      .Channel = DMA_CHANNEL_4,
      .Direction = DMA_PERIPH_TO_MEMORY,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_PFCTRL,
      .Priority = DMA_PRIORITY_MEDIUM,
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_FULL,
      .MemBurst = DMA_MBURST_INC4,
      .PeriphBurst = DMA_PBURST_INC4,
    };
    check_hal_error(HAL_DMA_Init(&hdma_sdio_rx));
    __HAL_LINKDMA(hsd, hdmarx, hdma_sdio_rx);

    hdma_sdio_tx.Instance = DMA2_Stream6;
    hdma_sdio_tx.Init = (DMA_InitTypeDef){
      .Channel = DMA_CHANNEL_4,
      .Direction = DMA_MEMORY_TO_PERIPH,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_PFCTRL,
      .Priority = DMA_PRIORITY_MEDIUM,
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_FULL,
      .MemBurst = DMA_MBURST_INC4,
      .PeriphBurst = DMA_PBURST_INC4,
    };
    check_hal_error(HAL_DMA_Init(&hdma_sdio_tx));
    __HAL_LINKDMA(hsd, hdmatx, hdma_sdio_tx);

    HAL_NVIC_SetPriority(SDIO_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
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

  HAL_StatusTypeDef error = HAL_OK;
  // <https://community.st.com/s/question/0D53W00000MgkxxSAB/code-stuck-in-sdfindscr-function>
  // <https://community.st.com/s/question/0D50X0000AeY5nHSQS/stm32l4-sd-stm32cubemx-v500-sd-clock-speed-during-call-to-sdmmccmdsendscr-as-part-of-switch-to-4bit-bus-is-too-fast-for-many-sd-cards>

  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.ClockDiv = SDIO_INIT_CLK_DIV;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  if ((error = HAL_SD_Init(&hsd)) != HAL_OK) {
    return error;
  }

  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.ClockDiv = SDIO_TRANSFER_CLK_DIV;
  // My board isn't fast enough for this, unfortunately.
  // hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_ENABLE;
  if ((error = HAL_SD_ConfigWideBusOperation(&hsd, hsd.Init.BusWide)) != HAL_OK) {
    return error;
  }

  return HAL_OK;
}

bool BSP_SD_IsDetected(void) {
  return HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin) == GPIO_PIN_RESET;
}
