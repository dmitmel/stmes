#include "stmes/fatfs.h"
#include "stmes/drivers/sdmmc.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/sync.h"
#include "stmes/utils.h"
#include <ff.h>
#include <printf.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_sdmmc.h>

#include <diskio.h>

#define SD_TIMEOUT 1000

static u8 dma_scratch[SDMMC_BLOCK_SIZE] __ALIGNED(4);

static volatile DSTATUS sd_status = STA_NOINIT;

#if FF_USE_LFN == 3
void* ff_memalloc(UINT msize) {
  return malloc(msize);
}
void ff_memfree(void* mblock) {
  free(mblock);
}
#endif

#if FF_FS_REENTRANT
static struct Mutex* get_fatfs_mutex(int volume) {
  ASSERT(0 <= volume && volume <= FF_VOLUMES);
  static struct Mutex ff_mutexes[FF_VOLUMES + 1];
  return &ff_mutexes[volume];
}

int ff_mutex_create(int volume) {
  mutex_init(get_fatfs_mutex(volume));
  return true;
}

void ff_mutex_delete(int volume) {
  UNUSED(volume);
}

int ff_mutex_take(int volume) {
  mutex_lock(get_fatfs_mutex(volume));
  return true;
}

void ff_mutex_give(int volume) {
  mutex_unlock(get_fatfs_mutex(volume));
}
#endif

static HAL_StatusTypeDef sd_wait_for_card_state(enum SdmmcCardState state, Systime deadline) {
  do {
    union SdmmcCSR card_status;
    if (sdmmc_get_card_status(&card_status) != SDMMC_ERROR_NONE) return HAL_ERROR;
    if (card_status.bits.current_state == state) return HAL_OK;
    task_yield();
  } while (systime_now() < deadline);
  return HAL_TIMEOUT;
}

static DSTATUS sd_check_status(void) {
  sd_status = STA_NOINIT;
  union SdmmcCSR card_status;
  u32 err = sdmmc_get_card_status(&card_status);
  if (err == SDMMC_ERROR_NONE && card_status.bits.current_state == SDMMC_STATE_TRANSFER) {
    sd_status &= ~STA_NOINIT;
  }
  return sd_status;
}

DSTATUS disk_initialize(BYTE pdrv) {
  UNUSED(pdrv);
  sd_status = STA_NOINIT;
  sdmmc_init_gpio();
  const struct SdmmcHostCapabilities capabilities = {
    .high_speed_mode = true,
    .use_4bit_data_bus = true,
  };
  if (sdmmc_init_card(&capabilities) == SDMMC_ERROR_NONE) {
    sd_status = sd_check_status();
  }
  return sd_status;
}

DSTATUS disk_status(BYTE pdrv) {
  UNUSED(pdrv);
  return sd_check_status();
}

DRESULT disk_read(BYTE pdrv, BYTE* buffer, LBA_t sector, UINT count) {
  UNUSED(pdrv);
  Systime deadline = timeout_to_deadline(SD_TIMEOUT);
  if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;

  // Take the fast path if the output buffer is 4-byte aligned
  if ((usize)buffer % 4 == 0) {
    if (sdmmc_read(buffer, sector, count, deadline) != HAL_OK) return RES_ERROR;
    if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
    return RES_OK;
  }

  // Otherwise, fetch each sector to an aligned buffer and copy it to the destination
  for (UINT last = sector + count; sector < last; sector++) {
    if (sdmmc_read(dma_scratch, sector, 1, deadline) != HAL_OK) return RES_ERROR;
    fast_memcpy_u8(buffer, dma_scratch, SDMMC_BLOCK_SIZE);
    buffer += SDMMC_BLOCK_SIZE;
  }
  if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buffer, LBA_t sector, UINT count) {
  UNUSED(pdrv);
  Systime deadline = timeout_to_deadline(SD_TIMEOUT);
  if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;

  // Take the fast path if the output buffer is 4-byte aligned
  if ((usize)buffer % 4 == 0) {
    if (sdmmc_write(buffer, sector, count, deadline) != HAL_OK) return RES_ERROR;
    if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
    return RES_OK;
  }

  // Otherwise, copy each sector to an aligned buffer and write it
  for (UINT last = sector + count; sector < last; sector++) {
    fast_memcpy_u8(dma_scratch, buffer, SDMMC_BLOCK_SIZE);
    buffer += SDMMC_BLOCK_SIZE;
    if (sdmmc_write(dma_scratch, sector, 1, deadline) != HAL_OK) return RES_ERROR;
  }
  if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* out) {
  UNUSED(pdrv);
  if (sd_status & STA_NOINIT) return RES_NOTRDY;
  switch (cmd) {
    case CTRL_SYNC: {
      Systime deadline = timeout_to_deadline(SD_TIMEOUT);
      if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
      return RES_OK;
    }
    case GET_SECTOR_COUNT: {
      *(LBA_t*)out = sdmmc_get_blocks_count(sdmmc_get_card());
      return RES_OK;
    }
    case GET_SECTOR_SIZE: {
      *(WORD*)out = SDMMC_BLOCK_SIZE;
      return RES_OK;
    }
    case GET_BLOCK_SIZE: {
      *(DWORD*)out = sdmmc_get_eraseable_sector_size(sdmmc_get_card());
      return RES_OK;
    }
    case CTRL_TRIM: {
      LBA_t start = ((LBA_t*)out)[0], end = ((LBA_t*)out)[1];
      // TODO: Erasing may take a very long time, calculate the timeout.
      Systime deadline = timeout_to_deadline(NO_DEADLINE);
      if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
      if (sdmmc_erase(start, end, deadline) != HAL_OK) return RES_ERROR;
      if (sd_wait_for_card_state(SDMMC_STATE_TRANSFER, deadline) != HAL_OK) return RES_ERROR;
      return RES_OK;
    }
    default: {
      return RES_PARERR;
    }
  }
}

__NO_RETURN void crash_on_fs_error(FRESULT code, const char* file, u32 line) {
  const char* name;
  switch (code) {
    case FR_OK: name = "OK"; break;
    case FR_DISK_ERR: name = "DISK_ERR"; break;
    case FR_INT_ERR: name = "INT_ERR"; break;
    case FR_NOT_READY: name = "NOT_READY"; break;
    case FR_NO_FILE: name = "NO_FILE"; break;
    case FR_NO_PATH: name = "NO_PATH"; break;
    case FR_INVALID_NAME: name = "INVALID_NAME"; break;
    case FR_DENIED: name = "DENIED"; break;
    case FR_EXIST: name = "EXIST"; break;
    case FR_INVALID_OBJECT: name = "INVALID_OBJECT"; break;
    case FR_WRITE_PROTECTED: name = "WRITE_PROTECTED"; break;
    case FR_INVALID_DRIVE: name = "INVALID_DRIVE"; break;
    case FR_NOT_ENABLED: name = "NOT_ENABLED"; break;
    case FR_NO_FILESYSTEM: name = "NO_FILESYSTEM"; break;
    case FR_MKFS_ABORTED: name = "MKFS_ABORTED"; break;
    case FR_TIMEOUT: name = "TIMEOUT"; break;
    case FR_LOCKED: name = "LOCKED"; break;
    case FR_NOT_ENOUGH_CORE: name = "NOT_ENOUGH_CORE"; break;
    case FR_TOO_MANY_OPEN_FILES: name = "TOO_MANY_OPEN_FILES"; break;
    case FR_INVALID_PARAMETER: name = "INVALID_PARAMETER"; break;
    default: name = "UNKNOWN";
  }
  char msg[32];
  snprintf(msg, sizeof(msg), "0x%02" PRIX32 "/FR_%s", (u32)code, name);
  crash(msg, file, line);
}
