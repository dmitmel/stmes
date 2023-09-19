// TODO: Implement our own SDIO driver, the default one is too bloated.
// <https://github.com/KimJorgensen/KungFuFlash/blob/v1.47/firmware/stm32f4xx/diskio.c>
// <https://github.com/LonelyWolf/stm32/blob/287e216a799fbacf00a8b3e363514c9bb82eafcd/stm32l4-sdio/src/sdcard.c>
// <https://github.com/thebiguno/microcontroller-projects/blob/309f5e6951803bee0a6f6ad73d96c2fa9dc339b7/inc/arm/SD/SD.cpp>
// <https://github.com/westlicht/performer/blob/dac9d12621b4683fc0e98303a422b34ac6f3d23a/src/apps/bootloader/SdCard.cpp>
// <https://github.com/mriscoc/Ender3V2S1/blob/20230805/Marlin/src/HAL/STM32F1/sdio.cpp>
// <https://github.com/Sergey1560/Marlin_FB4S/blob/2.1.2_5/Marlin/src/libs/fatfs/sdio_driver_f4.cpp>
// <https://github.com/insane-adding-machines/frosted/blob/5c3aa21f146a1b840badfd72d1d8b96e8405a8d8/kernel/drivers/stm32_sdio.c>
// <https://github.com/ChuckM/stm32f4-sdio-driver/blob/fc38f0b0a38ce67de62bd6cd3cbd7f7e0d6ae853/sdio.c>
// <https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/341cd894516f747f14108de5da593dad99900ae0/STM32F4/cores/maple/libmaple/sdio.c>
// <https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/341cd894516f747f14108de5da593dad99900ae0/STM32F4/libraries/SDIO/SdioF4.cpp>
// <https://habr.com/ru/articles/213803/>
// <https://luckyresistor.me/cat-protector/software/sdcard-2/>
// <http://alumni.cs.ucr.edu/~amitra/sdcard/Additional/sdcard_appnote_foust.pdf>

#define HAL_PATCHES_IMPLEMENT_SDMMC

#include "stmes/sdio.h"
#include "stmes/gpio.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"
#include "stmes/kernel/time.h"
#include <printf.h>
#include <stm32f4xx_hal.h>

SD_HandleTypeDef hsd;
static DMA_HandleTypeDef hdma_sdio_rx, hdma_sdio_tx;

void SDIO_IRQHandler(void) {
  HAL_SD_IRQHandler(&hsd);
}

void DMA2_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

void DMA2_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_sdio_tx);
}

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
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
      .Alternate = GPIO_AF12_SDIO,
    };
    gpio_init.Pin = SDIO_CLK_PIN;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Pin = SDIO_D1_PIN | SDIO_D2_PIN | SDIO_CMD_PIN;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = SDIO_D0_PIN | SDIO_D3_PIN;
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
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd) {
  if (hsd->Instance == SDIO) {
    __HAL_RCC_SDIO_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, SDIO_CMD_PIN | SDIO_D1_PIN | SDIO_D2_PIN);
    HAL_GPIO_DeInit(GPIOB, SDIO_CLK_PIN | SDIO_D0_PIN | SDIO_D3_PIN);
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
  return HAL_GPIO_ReadPin(SDIO_CD_GPIO_PORT, SDIO_CD_PIN) == GPIO_PIN_RESET;
}

// Some constants that aren't defined in the HAL headers:

// clang-format off
#define SDMMC_R4_ILLEGAL_CMD            0x00000004u
#define SDMMC_R4_COM_CRC_FAILED         0x00000008u
#define SDMMC_R4_INVALID_FUNCTION_NUM   0x00000010u
#define SDMMC_R4_INVALID_PARAMETER      0x00000040u
#define SDMMC_R4_ERRORBITS              0x0000005Cu

#define SDMMC_R5_ARG_OUT_OF_RANGE       0x00000001u
#define SDMMC_R5_INVALID_FUNCTION_NUM   0x00000002u
#define SDMMC_R5_GENERAL_UNKNOWN_ERROR  0x00000008u
#define SDMMC_R5_ILLEGAL_CMD            0x00000040u
#define SDMMC_R5_COM_CRC_FAILED         0x00000080u
#define SDMMC_R5_ERRORBITS              0x000000CBu

#define SDMMC_R6_ERRORBITS              0x0000E000u
// clang-format on

// The error bits in the mapping tables are stored not as 32-bit masks (as they
// are defined in the HAL headers), but as bit positions, which gives a 4x
// reduction in size.
struct SdioErrorDef {
  u8 response_bit, error_bit;
};

// Extracts the position of a set bit from a bit mask, otherwise (if the mask
// is not a power of 2, i.e. more than one bit is set) triggers a compile-time
// error.
#define ERROR_BIT(mask) \
  (__builtin_choose_expr((mask & (mask - 1)) == 0, __builtin_ctz(mask), (void)0))

static const struct SdioErrorDef SDIO_ERRORS_R1[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_OCR_ADDR_OUT_OF_RANGE),     ERROR_BIT(SDMMC_ERROR_ADDR_OUT_OF_RANGE)    },
  { ERROR_BIT(SDMMC_OCR_ADDR_MISALIGNED),       ERROR_BIT(SDMMC_ERROR_ADDR_MISALIGNED)      },
  { ERROR_BIT(SDMMC_OCR_BLOCK_LEN_ERR),         ERROR_BIT(SDMMC_ERROR_BLOCK_LEN_ERR)        },
  { ERROR_BIT(SDMMC_OCR_ERASE_SEQ_ERR),         ERROR_BIT(SDMMC_ERROR_ERASE_SEQ_ERR)        },
  { ERROR_BIT(SDMMC_OCR_BAD_ERASE_PARAM),       ERROR_BIT(SDMMC_ERROR_BAD_ERASE_PARAM)      },
  { ERROR_BIT(SDMMC_OCR_WRITE_PROT_VIOLATION),  ERROR_BIT(SDMMC_ERROR_WRITE_PROT_VIOLATION) },
  { ERROR_BIT(SDMMC_OCR_LOCK_UNLOCK_FAILED),    ERROR_BIT(SDMMC_ERROR_LOCK_UNLOCK_FAILED)   },
  { ERROR_BIT(SDMMC_OCR_COM_CRC_FAILED),        ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)       },
  { ERROR_BIT(SDMMC_OCR_ILLEGAL_CMD),           ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)          },
  { ERROR_BIT(SDMMC_OCR_CARD_ECC_FAILED),       ERROR_BIT(SDMMC_ERROR_CARD_ECC_FAILED)      },
  { ERROR_BIT(SDMMC_OCR_CC_ERROR),              ERROR_BIT(SDMMC_ERROR_CC_ERR)               },
  { ERROR_BIT(SDMMC_OCR_GENERAL_UNKNOWN_ERROR), ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)  },
  { ERROR_BIT(SDMMC_OCR_STREAM_READ_UNDERRUN),  ERROR_BIT(SDMMC_ERROR_STREAM_READ_UNDERRUN) },
  { ERROR_BIT(SDMMC_OCR_STREAM_WRITE_OVERRUN),  ERROR_BIT(SDMMC_ERROR_STREAM_WRITE_OVERRUN) },
  { ERROR_BIT(SDMMC_OCR_CID_CSD_OVERWRITE),     ERROR_BIT(SDMMC_ERROR_CID_CSD_OVERWRITE)    },
  { ERROR_BIT(SDMMC_OCR_WP_ERASE_SKIP),         ERROR_BIT(SDMMC_ERROR_WP_ERASE_SKIP)        },
  { ERROR_BIT(SDMMC_OCR_CARD_ECC_DISABLED),     ERROR_BIT(SDMMC_ERROR_CARD_ECC_DISABLED)    },
  { ERROR_BIT(SDMMC_OCR_ERASE_RESET),           ERROR_BIT(SDMMC_ERROR_ERASE_RESET)          },
  { ERROR_BIT(SDMMC_OCR_AKE_SEQ_ERROR),         ERROR_BIT(SDMMC_ERROR_AKE_SEQ_ERR)          },
  // clang-format on
};

static const struct SdioErrorDef SDIO_ERRORS_R4[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_R4_ILLEGAL_CMD),            ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)            },
  { ERROR_BIT(SDMMC_R4_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)         },
  { ERROR_BIT(SDMMC_R4_INVALID_FUNCTION_NUM),   ERROR_BIT(SDMMC_ERROR_REQUEST_NOT_APPLICABLE) },
  { ERROR_BIT(SDMMC_R4_INVALID_PARAMETER),      ERROR_BIT(SDMMC_ERROR_INVALID_PARAMETER)      },
  { ERROR_BIT(SDMMC_R4_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)         },
  // clang-format on
};

static const struct SdioErrorDef SDIO_ERRORS_R5[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_R5_ARG_OUT_OF_RANGE),       ERROR_BIT(SDMMC_ERROR_ADDR_OUT_OF_RANGE)      },
  { ERROR_BIT(SDMMC_R5_INVALID_FUNCTION_NUM),   ERROR_BIT(SDMMC_ERROR_REQUEST_NOT_APPLICABLE) },
  { ERROR_BIT(SDMMC_R5_GENERAL_UNKNOWN_ERROR),  ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)    },
  { ERROR_BIT(SDMMC_R5_ILLEGAL_CMD),            ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)            },
  { ERROR_BIT(SDMMC_R5_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)         },
  // clang-format on
};

static const struct SdioErrorDef SDIO_ERRORS_R6[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_R6_ILLEGAL_CMD),            ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)          },
  { ERROR_BIT(SDMMC_R6_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)       },
  { ERROR_BIT(SDMMC_R6_GENERAL_UNKNOWN_ERROR),  ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)  },
  // clang-format on
};

// The functions in the STM32 HAL use a large sequence of else-ifs to check for
// every error bit in the response and return an appropriate response code:
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c#L1239-L1318>.
// Needless to say, this generates a lot of machine code (a branch for every
// case, 32-bit `tst` and `mov` instructions) for code that is normally never
// even executed. I replace that with a generic (for all response types)
// function that uses a much more compact (in terms of flash usage) mapping
// table between the specific bits in the responses and the HAL error codes.
static u32 sdio_check_response_error(
  u32 response, const struct SdioErrorDef* error_defs, usize error_defs_len
) {
  u32 error_bits = SDMMC_ERROR_NONE;
  for (usize i = 0; i < error_defs_len; i++) {
    if (response & BIT(error_defs[i].response_bit)) {
      error_bits |= error_defs[i].error_bit;
    }
  }
  return error_bits;
}

// Sends an SD/MMC command with a specified argument, waits for a response and
// appropriately handles the errors according to the response format parameter
// (the reply from the SD card can be read from the SDIO_RESP[1-4] registers
// afterwards). Combines the functionality of the SDIO_SendCommand and
// SDMMC_GetCmdResp[1-4] STM32 HAL functions. Note that the first two arguments
// of this function match the arguments of the other functions which will be
// invoking this one to reduce the number of `mov` instructions.
//
// This function is specialized for different response types using a poor-man's
// templates-in-C technique: the generic function is forcibly inlined into
// wrappers with certain select parameters, but otherwise isn't used directly.
// Specifically, we want a specialized variant only for R1 responses since
// those are the most common kind both in quantity and invocation frequency.
__STATIC_FORCEINLINE u32
sdio_send_command_impl(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd, enum SdioResponseFormat res) {
  // The default implementation leaves it up to the user of SDIO_SendCommand to
  // pass the response type, wait type and CPSM-enable (command path state
  // machine) bits for the SDIO_CMD register in an options struct, even though
  // they can simply be inferred from the response format.
  u32 cmd_params = ((u32)cmd << SDIO_CMD_CMDINDEX_Pos) & SDIO_CMD_CMDINDEX_Msk;
  cmd_params |= SDIO_WAIT_NO | SDIO_CPSM_ENABLE;
  switch (res) {
    case SDIO_RESPONSE_NONE: cmd_params |= SDIO_RESPONSE_NO; break;
    case SDIO_RESPONSE_R2: cmd_params |= SDIO_RESPONSE_LONG; break;
    default: cmd_params |= SDIO_RESPONSE_SHORT; break;
  }
  WRITE_REG(SDIOx->ARG, arg);
  MODIFY_REG(SDIOx->CMD, CMD_CLEAR_MASK, cmd_params);

  // The timeouts used by the library here are something like 63 seconds for
  // normal commands and 28 hours for the stop transfer command, not sure if
  // such large values are practical.
  u32 timeout = cmd == SDMMC_CMD_STOP_TRANSMISSION ? SDIO_STOPTRANSFERTIMEOUT : SDIO_CMDTIMEOUT;
  // NOTE: All of these are mutually exclusive and need to be checked, only one
  // of these flags will be set upon receiving the response.
  u32 completion_flags = SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT;
  if (unlikely(res == SDIO_RESPONSE_NONE)) completion_flags = SDIO_FLAG_CMDSENT;

  // The stop transfer command (CMD12) is sometimes sent inside interrupts,
  // yielding in that context is wildly unsafe.
  // bool should_yield = !in_interrupt_handler();
  u32 start_time = hwtimer_read();
  u32 sdio_sta;
  while (true) {
    sdio_sta = READ_REG(SDIOx->STA);
    // Stop if the command transfer is not active and any of the completion
    // flags have been set.
    if (!(sdio_sta & SDIO_FLAG_CMDACT) && (sdio_sta & completion_flags)) break;
    if (hwtimer_read() - start_time >= timeout) return SDMMC_ERROR_TIMEOUT;
    // if (should_yield) task_yield();
  }
  // This clears all kinds of completion flags:
  __SDIO_CLEAR_FLAG(SDIOx, SDIO_STATIC_CMD_FLAGS);

  if (unlikely(res == SDIO_RESPONSE_NONE)) {
    return SDMMC_ERROR_NONE;
  }

  if (unlikely(sdio_sta & SDIO_FLAG_CTIMEOUT)) {
    return SDMMC_ERROR_CMD_RSP_TIMEOUT;
  } else if (unlikely(sdio_sta & SDIO_FLAG_CCRCFAIL)) {
    // Workaround for errata 2.7.2: the SDIO peripheral calculates the CRC of
    // the response even if it doesn't contain a CRC field, which makes all
    // commands with such response formats invariably fail with a CRC error.
    // Since this normally can't happen by definition, the CRC error is treated
    // as a success condition for responses where the SD protocol doesn't
    // specify a field for the CRC.
    if (likely(res != SDIO_RESPONSE_R3 && res != SDIO_RESPONSE_R4)) {
      return SDMMC_ERROR_CMD_CRC_FAIL;
    }
  }

  // Not every response format has a command index field.
  if (likely(res != SDIO_RESPONSE_R2 && res != SDIO_RESPONSE_R3 && res != SDIO_RESPONSE_R4)) {
    u8 res_cmd = (READ_REG(SDIOx->RESPCMD) & SDIO_RESPCMD_RESPCMD_Msk) >> SDIO_RESPCMD_RESPCMD_Pos;
    if (unlikely(res_cmd != cmd)) {
      return SDMMC_ERROR_CMD_CRC_FAIL;
    }
  }

  // All responses are first checked against a mask with every error bit set to
  // determine if the expensive error checking procedure should be entered.
  if (likely(res == SDIO_RESPONSE_R1)) {
    u32 res_r1 = READ_REG(SDIOx->RESP1);
    if ((res_r1 & SDMMC_OCR_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R1, SIZEOF(SDIO_ERRORS_R1));
    }
  } else if (unlikely(res == SDIO_RESPONSE_R4)) {
    u32 res_r1 = READ_REG(SDIOx->RESP1);
    if ((res_r1 & SDMMC_R4_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R4, SIZEOF(SDIO_ERRORS_R4));
    }
  } else if (unlikely(res == SDIO_RESPONSE_R5)) {
    u32 res_r1 = READ_REG(SDIOx->RESP1);
    if ((res_r1 & SDMMC_R5_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R5, SIZEOF(SDIO_ERRORS_R5));
    }
  } else if (unlikely(res == SDIO_RESPONSE_R6)) {
    u32 res_r1 = READ_REG(SDIOx->RESP1);
    if ((res_r1 & SDMMC_R6_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R6, SIZEOF(SDIO_ERRORS_R6));
    }
  }

  return SDMMC_ERROR_NONE;
}

u32 sdio_send_command(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd, enum SdioResponseFormat res) {
  return sdio_send_command_impl(SDIOx, arg, cmd, res);
}

u32 sdio_send_command_r1(SDIO_TypeDef* SDIOx, u32 arg, u8 cmd) {
  return sdio_send_command_impl(SDIOx, arg, cmd, SDIO_RESPONSE_R1);
}

// The following functions are patched versions of the STM32 HAL functions for
// sending particular SD/MMC commands, which are necessary for two reasons:
// 1. To yield while in the busy-wait loop, waiting for the response.
// 2. To reduce the flash usage by the compiled code. The default
//    implementation bloats the binary a lot by duplicating a large block of
//    setup code for every single command.
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c#L490-L1170>

u32 SDMMC_CmdGoIdleState(SDIO_TypeDef* SDIOx) {
  return sdio_send_command(SDIOx, 0, SDMMC_CMD_GO_IDLE_STATE, SDIO_RESPONSE_NONE);
}

u32 SDMMC_CmdOpCondition(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command(SDIOx, arg, SDMMC_CMD_SEND_OP_COND, SDIO_RESPONSE_R3);
}

u32 SDMMC_CmdSendCID(SDIO_TypeDef* SDIOx) {
  return sdio_send_command(SDIOx, 0, SDMMC_CMD_ALL_SEND_CID, SDIO_RESPONSE_R2);
}

u32 SDMMC_CmdSetRelAdd(SDIO_TypeDef* SDIOx, u16* rca) {
  u32 error = sdio_send_command(SDIOx, 0, SDMMC_CMD_SET_REL_ADDR, SDIO_RESPONSE_R6);
  if (error == SDMMC_ERROR_NONE) {
    *rca = READ_REG(SDIOx->RESP1) >> 16;
  }
  return error;
}

u32 SDMMC_CmdSwitch(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_HS_SWITCH);
}

u32 SDMMC_CmdSelDesel(SDIO_TypeDef* SDIOx, u64 arg) {
  return sdio_send_command_r1(SDIOx, (u32)arg, SDMMC_CMD_SEL_DESEL_CARD);
}

u32 SDMMC_CmdOperCond(SDIO_TypeDef* SDIOx) {
  u32 arg = SDMMC_CHECK_PATTERN;
  return sdio_send_command(SDIOx, arg, SDMMC_CMD_HS_SEND_EXT_CSD, SDIO_RESPONSE_R7);
}

u32 SDMMC_CmdSendEXTCSD(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_HS_SEND_EXT_CSD);
}

u32 SDMMC_CmdSendCSD(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command(SDIOx, arg, SDMMC_CMD_SEND_CSD, SDIO_RESPONSE_R2);
}

u32 SDMMC_CmdStopTransfer(SDIO_TypeDef* SDIOx) {
  return sdio_send_command_r1(SDIOx, 0, SDMMC_CMD_STOP_TRANSMISSION);
}

u32 SDMMC_CmdSendStatus(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_SEND_STATUS);
}

u32 SDMMC_CmdBlockLength(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_SET_BLOCKLEN);
}

u32 SDMMC_CmdReadSingleBlock(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_READ_SINGLE_BLOCK);
}

u32 SDMMC_CmdReadMultiBlock(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_READ_MULT_BLOCK);
}

u32 SDMMC_CmdWriteSingleBlock(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_WRITE_SINGLE_BLOCK);
}

u32 SDMMC_CmdWriteMultiBlock(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_WRITE_MULT_BLOCK);
}

u32 SDMMC_CmdSDEraseStartAdd(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_SD_ERASE_GRP_START);
}

u32 SDMMC_CmdSDEraseEndAdd(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_SD_ERASE_GRP_END);
}

u32 SDMMC_CmdEraseStartAdd(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_ERASE_GRP_START);
}

u32 SDMMC_CmdEraseEndAdd(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_ERASE_GRP_END);
}

u32 SDMMC_CmdErase(SDIO_TypeDef* SDIOx) {
  return sdio_send_command_r1(SDIOx, 0, SDMMC_CMD_ERASE);
}

u32 SDMMC_CmdAppCommand(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_APP_CMD);
}

u32 SDMMC_CmdBusWidth(SDIO_TypeDef* SDIOx, u32 arg) {
  return sdio_send_command_r1(SDIOx, arg, SDMMC_CMD_APP_SD_SET_BUSWIDTH);
}

u32 SDMMC_CmdStatusRegister(SDIO_TypeDef* SDIOx) {
  return sdio_send_command_r1(SDIOx, 0, SDMMC_CMD_SD_APP_STATUS);
}

u32 SDMMC_CmdAppOperCommand(SDIO_TypeDef* SDIOx, u32 arg) {
  arg |= SDMMC_VOLTAGE_WINDOW_SD;
  return sdio_send_command(SDIOx, arg, SDMMC_CMD_SD_APP_OP_COND, SDIO_RESPONSE_R3);
}

u32 SDMMC_CmdSendSCR(SDIO_TypeDef* SDIOx) {
  return sdio_send_command_r1(SDIOx, 0, SDMMC_CMD_SD_APP_SEND_SCR);
}

__NO_RETURN void crash_on_sd_error(u32 code, const char* file, u32 line) {
  static const char* const ERROR_NAMES[] = {
    [ERROR_BIT(SDMMC_ERROR_CMD_CRC_FAIL)] = "CMD_CRC_FAIL",
    [ERROR_BIT(SDMMC_ERROR_DATA_CRC_FAIL)] = "DATA_CRC_FAIL",
    [ERROR_BIT(SDMMC_ERROR_CMD_RSP_TIMEOUT)] = "CMD_RSP_TIMEOUT",
    [ERROR_BIT(SDMMC_ERROR_DATA_TIMEOUT)] = "DATA_TIMEOUT",
    [ERROR_BIT(SDMMC_ERROR_TX_UNDERRUN)] = "TX_UNDERRUN",
    [ERROR_BIT(SDMMC_ERROR_RX_OVERRUN)] = "RX_OVERRUN",
    [ERROR_BIT(SDMMC_ERROR_ADDR_MISALIGNED)] = "ADDR_MISALIGNED",
    [ERROR_BIT(SDMMC_ERROR_BLOCK_LEN_ERR)] = "BLOCK_LEN_ERR",
    [ERROR_BIT(SDMMC_ERROR_ERASE_SEQ_ERR)] = "ERASE_SEQ_ERR",
    [ERROR_BIT(SDMMC_ERROR_BAD_ERASE_PARAM)] = "BAD_ERASE_PARAM",
    [ERROR_BIT(SDMMC_ERROR_WRITE_PROT_VIOLATION)] = "WRITE_PROT_VIOLATION",
    [ERROR_BIT(SDMMC_ERROR_LOCK_UNLOCK_FAILED)] = "LOCK_UNLOCK_FAILED",
    [ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)] = "COM_CRC_FAILED",
    [ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)] = "ILLEGAL_CMD",
    [ERROR_BIT(SDMMC_ERROR_CARD_ECC_FAILED)] = "CARD_ECC_FAILED",
    [ERROR_BIT(SDMMC_ERROR_CC_ERR)] = "CC_ERR",
    [ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)] = "GENERAL_UNKNOWN_ERR",
    [ERROR_BIT(SDMMC_ERROR_STREAM_READ_UNDERRUN)] = "STREAM_READ_UNDERRUN",
    [ERROR_BIT(SDMMC_ERROR_STREAM_WRITE_OVERRUN)] = "STREAM_WRITE_OVERRUN",
    [ERROR_BIT(SDMMC_ERROR_CID_CSD_OVERWRITE)] = "CID_CSD_OVERWRITE",
    [ERROR_BIT(SDMMC_ERROR_WP_ERASE_SKIP)] = "WP_ERASE_SKIP",
    [ERROR_BIT(SDMMC_ERROR_CARD_ECC_DISABLED)] = "CARD_ECC_DISABLED",
    [ERROR_BIT(SDMMC_ERROR_ERASE_RESET)] = "ERASE_RESET",
    [ERROR_BIT(SDMMC_ERROR_AKE_SEQ_ERR)] = "AKE_SEQ_ERR",
    [ERROR_BIT(SDMMC_ERROR_INVALID_VOLTRANGE)] = "INVALID_VOLTRANGE",
    [ERROR_BIT(SDMMC_ERROR_ADDR_OUT_OF_RANGE)] = "ADDR_OUT_OF_RANGE",
    [ERROR_BIT(SDMMC_ERROR_REQUEST_NOT_APPLICABLE)] = "REQUEST_NOT_APPLICABLE",
    [ERROR_BIT(SDMMC_ERROR_INVALID_PARAMETER)] = "INVALID_PARAMETER",
    [ERROR_BIT(SDMMC_ERROR_UNSUPPORTED_FEATURE)] = "UNSUPPORTED_FEATURE",
    [ERROR_BIT(SDMMC_ERROR_BUSY)] = "BUSY",
    [ERROR_BIT(SDMMC_ERROR_DMA)] = "DMA",
    [ERROR_BIT(SDMMC_ERROR_TIMEOUT)] = "TIMEOUT",
  };

  char msg[64];
  usize msg_pos = 0;
  i32 msg_written = snprintf(msg, sizeof(msg), "0x%08" PRIX32, code);
  msg_pos += MAX(0, msg_written);

  if (code == 0) {
    code = SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
  }
  while (code != 0) {
    u32 bit = 31 - __CLZ(code);
    code ^= BIT(bit);

    const char* name = bit < SIZEOF(ERROR_NAMES) ? ERROR_NAMES[bit] : "UNKNOWN";
    msg_written = snprintf(msg + msg_pos, sizeof(msg) - msg_pos, "/SD_%s", name);
    msg_pos += MAX(0, msg_written);
  }

  crash(msg, file, line);
}
