#include "stmes/sdio.h"
#include <ff.h>
#include <stm32f4xx_hal.h>

#include <diskio.h>

#define SD_TIMEOUT 30 * 1000
#define SD_DEFAULT_BLOCK_SIZE 512

static volatile DSTATUS sd_status = STA_NOINIT;

static DSTATUS SD_CheckStatus(void) {
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
    sd_status = SD_CheckStatus();
  }
  return sd_status;
}

DSTATUS disk_status(BYTE pdrv) {
  UNUSED(pdrv);
  return SD_CheckStatus();
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
  UNUSED(pdrv);
  if (HAL_SD_ReadBlocks(&hsd, buff, (uint32_t)sector, count, SD_TIMEOUT) == HAL_OK) {
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
    return RES_OK;
  }
  return RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
  UNUSED(pdrv);
  if (HAL_SD_WriteBlocks(&hsd, (BYTE*)buff, (uint32_t)sector, count, SD_TIMEOUT) == HAL_OK) {
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
    return RES_OK;
  }
  return RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  UNUSED(pdrv);
  if (sd_status & STA_NOINIT) return RES_NOTRDY;
  HAL_SD_CardInfoTypeDef CardInfo;
  switch (cmd) {
    case CTRL_SYNC: {
      return RES_OK;
    }
    case GET_SECTOR_COUNT: {
      HAL_SD_GetCardInfo(&hsd, &CardInfo);
      *(DWORD*)buff = CardInfo.LogBlockNbr;
      return RES_OK;
    }
    case GET_SECTOR_SIZE: {
      HAL_SD_GetCardInfo(&hsd, &CardInfo);
      *(WORD*)buff = CardInfo.LogBlockSize;
      return RES_OK;
    }
    case GET_BLOCK_SIZE: {
      HAL_SD_GetCardInfo(&hsd, &CardInfo);
      *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
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
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
}

void HAL_SD_AbortCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef* hsd) {
  UNUSED(hsd);
}
