#include "stmes/sdio.h"
#include "stmes/utils.h"
#include <ff.h>
#include <stm32f4xx_hal.h>

#include <diskio.h>

#define SD_TIMEOUT 30 * 1000
#define SD_DEFAULT_BLOCK_SIZE 512

__ALIGN_BEGIN static u8 dma_scratch[BLOCKSIZE] __ALIGN_END;

static volatile DSTATUS sd_status = STA_NOINIT;
static volatile bool sd_write_done = false, sd_read_done = false;

static HAL_StatusTypeDef sd_wait_for_card_state(HAL_SD_CardStateTypedef state, u32 timeout) {
  u32 start_time = HAL_GetTick();
  do {
    if (HAL_SD_GetCardState(&hsd) == state) {
      return HAL_OK;
    }
  } while (HAL_GetTick() - start_time < timeout);
  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef sd_wait_until_flag_set(volatile bool* flag, u32 timeout) {
  u32 start_time = HAL_GetTick();
  do {
    if (*flag != false) {
      return HAL_OK;
    }
  } while (HAL_GetTick() - start_time < timeout);
  return HAL_TIMEOUT;
}

static DSTATUS sd_check_status(void) {
  sd_status = STA_NOINIT;
  if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) {
    sd_status &= ~STA_NOINIT;
  }
  return sd_status;
}

DSTATUS disk_initialize(BYTE pdrv) {
  UNUSED(pdrv);
  sd_status = STA_NOINIT;
  if (BSP_SD_Init() == HAL_OK) {
    sd_status = sd_check_status();
  }
  return sd_status;
}

DSTATUS disk_status(BYTE pdrv) {
  UNUSED(pdrv);
  return sd_check_status();
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
  UNUSED(pdrv);
  if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
    return RES_ERROR;
  }

  // Take the fast path if the output buffer is 4-byte aligned
  if ((usize)buff % 4 == 0) {
    sd_read_done = false;
    if (HAL_SD_ReadBlocks_DMA(&hsd, buff, sector, count) != HAL_OK) {
      return RES_ERROR;
    }
    if (sd_wait_until_flag_set(&sd_read_done, SD_TIMEOUT) != HAL_OK) {
      return RES_ERROR;
    }
    sd_read_done = false;
    if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
      return RES_ERROR;
    }
    return RES_OK;
  }

  // Otherwise, fetch each sector to an aligned buffer and copy it to the destination
  for (UINT last = sector + count; sector < last; sector++) {
    sd_read_done = false;
    if (HAL_SD_ReadBlocks_DMA(&hsd, dma_scratch, sector, 1) != HAL_OK) {
      return RES_ERROR;
    }
    if (sd_wait_until_flag_set(&sd_read_done, SD_TIMEOUT) != HAL_OK) {
      return RES_ERROR;
    }
    sd_read_done = false;
    fast_memcpy_u8(buff, dma_scratch, BLOCKSIZE);
    buff += BLOCKSIZE;
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
  UNUSED(pdrv);
  if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
    return RES_ERROR;
  }

  // Take the fast path if the output buffer is 4-byte aligned
  if ((usize)buff % 4 == 0) {
    sd_write_done = false;
    if (HAL_SD_WriteBlocks_DMA(&hsd, (BYTE*)buff, sector, count) != HAL_OK) {
      return RES_ERROR;
    }
    if (sd_wait_until_flag_set(&sd_write_done, SD_TIMEOUT) != HAL_OK) {
      return RES_ERROR;
    }
    sd_write_done = false;
    if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
      return RES_ERROR;
    }
    return RES_OK;
  }

  // Otherwise, copy each sector to an aligned buffer and write it
  for (UINT last = sector + count; sector < last; sector++) {
    fast_memcpy_u8(dma_scratch, buff, BLOCKSIZE);
    buff += BLOCKSIZE;
    sd_write_done = false;
    if (HAL_SD_WriteBlocks_DMA(&hsd, dma_scratch, sector, 1) != HAL_OK) {
      return RES_ERROR;
    }
    if (sd_wait_until_flag_set(&sd_write_done, SD_TIMEOUT) != HAL_OK) {
      return RES_ERROR;
    }
    sd_write_done = false;
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  UNUSED(pdrv);
  if (sd_status & STA_NOINIT) return RES_NOTRDY;
  switch (cmd) {
    case CTRL_SYNC: {
      if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
        return RES_ERROR;
      }
      return RES_OK;
    }
    case GET_SECTOR_COUNT: {
      *(DWORD*)buff = hsd.SdCard.LogBlockNbr;
      return RES_OK;
    }
    case GET_SECTOR_SIZE: {
      *(WORD*)buff = hsd.SdCard.LogBlockSize;
      return RES_OK;
    }
    case GET_BLOCK_SIZE: {
      *(DWORD*)buff = hsd.SdCard.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
      return RES_OK;
    }
    case CTRL_TRIM: {
      DWORD start = ((DWORD*)buff)[0], end = ((DWORD*)buff)[1];
      if (HAL_SD_Erase(&hsd, start, end) != HAL_OK) {
        return RES_ERROR;
      }
      return RES_OK;
    }
    default: {
      return RES_PARERR;
    }
  }
}

DWORD get_fattime(void) {
  return 0;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
  sd_write_done = true;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
  sd_read_done = true;
}

void HAL_SD_AbortCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
}
