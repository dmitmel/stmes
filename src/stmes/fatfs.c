#include "stmes/fatfs.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/sync.h"
#include "stmes/sdio.h"
#include "stmes/utils.h"
#include <ff.h>
#include <printf.h>
#include <stm32f4xx_hal.h>

#include <diskio.h>

#define SD_TIMEOUT 3000
#define SD_DEFAULT_BLOCK_SIZE 512

static u8 dma_scratch[BLOCKSIZE] __ALIGNED(4);

static volatile DSTATUS sd_status = STA_NOINIT;
static volatile bool sd_write_done = false, sd_read_done = false;

void* ff_memalloc(UINT msize) {
  return ff_malloc(msize);
}

void ff_memfree(void* mblock) {
  ff_free(mblock);
}

int ff_cre_syncobj(BYTE volume, struct Mutex** mutex) {
  static struct Mutex ff_mutexes[_VOLUMES];
  *mutex = &ff_mutexes[volume];
  mutex_init(*mutex);
  return true;
}

int ff_del_syncobj(struct Mutex* mutex) {
  UNUSED(mutex);
  return true;
}

int ff_req_grant(struct Mutex* mutex) {
  mutex_lock(mutex);
  return true;
}

void ff_rel_grant(struct Mutex* mutex) {
  mutex_unlock(mutex);
}

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

__STATIC_INLINE HAL_StatusTypeDef sd_read_aligned(u8* buf, u32 sector, u32 count) {
  HAL_StatusTypeDef res;
  sd_read_done = false;
  if ((res = HAL_SD_ReadBlocks_DMA(&hsd, buf, sector, count)) != HAL_OK) return res;
  if ((res = sd_wait_until_flag_set(&sd_read_done, SD_TIMEOUT)) != HAL_OK) return res;
  sd_read_done = false;
  return HAL_OK;
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
  UNUSED(pdrv);
  if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
    return RES_ERROR;
  }

  // Take the fast path if the output buffer is 4-byte aligned
  if ((usize)buff % 4 == 0) {
    if (sd_read_aligned(buff, sector, count) != HAL_OK) return RES_ERROR;
    if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) return RES_ERROR;
    return RES_OK;
  }

  // Otherwise, fetch each sector to an aligned buffer and copy it to the destination
  for (UINT last = sector + count; sector < last; sector++) {
    if (sd_read_aligned(dma_scratch, sector, 1) != HAL_OK) return RES_ERROR;
    fast_memcpy_u8(buff, dma_scratch, BLOCKSIZE);
    buff += BLOCKSIZE;
  }
  return RES_OK;
}

__STATIC_INLINE HAL_StatusTypeDef sd_write_aligned(const u8* buf, u32 sector, u32 count) {
  HAL_StatusTypeDef res;
  sd_write_done = false;
  if ((res = HAL_SD_WriteBlocks_DMA(&hsd, (u8*)buf, sector, count)) != HAL_OK) return res;
  if ((res = sd_wait_until_flag_set(&sd_write_done, SD_TIMEOUT)) != HAL_OK) return res;
  sd_write_done = false;
  return HAL_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
  UNUSED(pdrv);
  if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) {
    return RES_ERROR;
  }

  // Take the fast path if the output buffer is 4-byte aligned
  if ((usize)buff % 4 == 0) {
    if (sd_write_aligned(buff, sector, count) != HAL_OK) return RES_ERROR;
    if (sd_wait_for_card_state(HAL_SD_CARD_TRANSFER, SD_TIMEOUT) != HAL_OK) return RES_ERROR;
    return RES_OK;
  }

  // Otherwise, copy each sector to an aligned buffer and write it
  for (UINT last = sector + count; sector < last; sector++) {
    fast_memcpy_u8(dma_scratch, buff, BLOCKSIZE);
    buff += BLOCKSIZE;
    if (sd_write_aligned(dma_scratch, sector, 1) != HAL_OK) return RES_ERROR;
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

__NO_RETURN void crash_on_fs_error(FRESULT code, const char* file, u32 line) {
  static const char* const ERROR_NAMES[] = {
    [FR_OK] = "OK",
    [FR_DISK_ERR] = "DISK_ERR",
    [FR_INT_ERR] = "INT_ERR",
    [FR_NOT_READY] = "NOT_READY",
    [FR_NO_FILE] = "NO_FILE",
    [FR_NO_PATH] = "NO_PATH",
    [FR_INVALID_NAME] = "INVALID_NAME",
    [FR_DENIED] = "DENIED",
    [FR_EXIST] = "EXIST",
    [FR_INVALID_OBJECT] = "INVALID_OBJECT",
    [FR_WRITE_PROTECTED] = "WRITE_PROTECTED",
    [FR_INVALID_DRIVE] = "INVALID_DRIVE",
    [FR_NOT_ENABLED] = "NOT_ENABLED",
    [FR_NO_FILESYSTEM] = "NO_FILESYSTEM",
    [FR_MKFS_ABORTED] = "MKFS_ABORTED",
    [FR_TIMEOUT] = "TIMEOUT",
    [FR_LOCKED] = "LOCKED",
    [FR_NOT_ENOUGH_CORE] = "NOT_ENOUGH_CORE",
    [FR_TOO_MANY_OPEN_FILES] = "TOO_MANY_OPEN_FILES",
    [FR_INVALID_PARAMETER] = "INVALID_PARAMETER",
  };
  const char* name = (u32)code < SIZEOF(ERROR_NAMES) ? ERROR_NAMES[code] : "UNKNOWN";
  char msg[32];
  snprintf(msg, sizeof(msg), "0x%02" PRIX32 "/FR_%s", (u32)code, name);
  crash(msg, file, line);
}
