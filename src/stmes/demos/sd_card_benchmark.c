#include "stmes/demos.h"
#include "stmes/drivers/sdmmc.h"
#include "stmes/fatfs.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <ff.h>
#include <math.h>
#include <printf.h>
#include <stm32f4xx_hal.h>

#include <diskio.h>

static u8 render_task_stack[1024] __ALIGNED(8);
static struct Task render_task;

static u8 test_task_stack[1024] __ALIGNED(8);
static struct Task test_task;

static u8 progress_task_stack[1024] __ALIGNED(8);
static struct Task progress_task;

static struct Notification progress_task_notify;

static FATFS SDFatFS;
static FIL SDFile;
#if !FF_FS_READONLY
static FIL SDFile2;
#endif

static void test_task_fn(__UNUSED void* user_data) {
  static char buf[SDMMC_BLOCK_SIZE * 32];

  FRESULT fres = f_mount(&SDFatFS, "", 1);
  if (fres == FR_NO_FILESYSTEM) {
#if !FF_FS_READONLY && FF_USE_MKFS
    fres = f_mkfs("", FM_ANY, 0, buf, sizeof(buf));
#endif
  }
  check_fs_error(fres);

  const struct SdmmcCard* card = sdmmc_get_card();
  printf(
    "CID: %08" PRIX32 " %08" PRIX32 " %08" PRIX32 " %08" PRIX32 "\n",
    card->cid.words[3],
    card->cid.words[2],
    card->cid.words[1],
    card->cid.words[0]
  );
  const struct SdCID* cid = &card->cid.sd;
  printf(" Manufacturer ID    : 0x%02X\n", cid->manufacturer_id);
  printf(" OEM/Application ID : %.2s\n", cid->oem_application_id);
  const char* name = cid->product_name;
  printf(" Product Name       : %c%c%c%c%c\n", name[4], name[3], name[2], name[1], name[0]);
  printf(
    " Product Revision   : %d.%d\n",
    (cid->product_revision >> 4) & 0xF,
    cid->product_revision & 0xF
  );
  printf(" Serial Number      : 0x%08" PRIX32 "\n", cid->serial_number);
  printf(
    " Manufacturing Date : %04d/%02d\n", cid->manufacturing_year + 2000, cid->manufacturing_month
  );

  task_sleep(3000);

  while (true) {
    check_fs_error(f_open(&SDFile, "bebop_palette.bin", FA_READ));
#if !FF_FS_READONLY
    check_fs_error(f_open(&SDFile2, "copy.bin", FA_WRITE | FA_CREATE_ALWAYS));
#endif

    printf("loading %" PRIu32 "\n", f_size(&SDFile));
    task_yield();

    usize total_bytes = 0;
    Systime start_time = systime_now();

    while (true) {
      task_notify(&progress_task_notify);
      task_yield();
#if 1
      usize bytes_read = 0;
      check_fs_error(f_read(&SDFile, buf, sizeof(buf), &bytes_read));
      if (bytes_read == 0) break;
      total_bytes += bytes_read;
#if !FF_FS_READONLY
      check_fs_error(f_write(&SDFile2, buf, bytes_read, &bytes_read));
#endif
#else
      // disk_read(0, (BYTE*)buf, total_bytes / SDMMC_BLOCK_SIZE, sizeof(buf) / SDMMC_BLOCK_SIZE);
      sdmmc_read(
        (u8*)buf, total_bytes / SDMMC_BLOCK_SIZE, sizeof(buf) / SDMMC_BLOCK_SIZE, NO_DEADLINE
      );
      total_bytes += sizeof(buf);
      if (total_bytes >= 1024 * 1024) {
        break;
      }
#endif
    }

    task_notify(&progress_task_notify);
    task_yield();

    Systime end_time = systime_now();
    printf("\n");

    double elapsed_seconds = (double)(u32)systime_as_millis(end_time - start_time) / 1000;
    printf("%" PRIuPTR " %" PRIu32 "\n", total_bytes, (u32)(end_time - start_time));
    printf("%f B/sec\n", (double)total_bytes / elapsed_seconds);
    printf("%f kB/sec\n", (double)total_bytes / 1024 / elapsed_seconds);
    printf("%f MB/sec\n", (double)total_bytes / 1024 / 1024 / elapsed_seconds);

    task_sleep(2000);
    f_close(&SDFile);
#if !FF_FS_READONLY
    f_close(&SDFile2);
    break;
#endif
  }

  check_fs_error(f_mount(0, "", 0));
}

static void progress_task_fn(__UNUSED void* user_data) {
  task_wait(&progress_task_notify, NO_DEADLINE);
  Systime start_time = systime_now();
  while (true) {
    task_wait(&progress_task_notify, NO_DEADLINE);
    console_clear_cursor_line();
    u32 percent = f_tell(&SDFile) * 100 / f_size(&SDFile);
    Systime elapsed_time = systime_now() - start_time;
    printf("\r%" PRIu32 "%% %" PRIu32, percent, (u32)systime_as_millis(elapsed_time));
    task_sleep(100);
  }
}

static void render_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_wait(&vga_notification, NO_DEADLINE);
    if (vga_control.next_scanline_requested) {
      vga_control.next_scanline_requested = false;
      console_render_scanline(vga_control.next_scanline_nr);
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
    }
  }
}

void sd_card_benchmark(void) {
  task_notify_init(&progress_task_notify);

  struct TaskParams render_task_params = {
    .stack_start = render_task_stack,
    .stack_size = sizeof(render_task_stack),
    .func = &render_task_fn,
  };
  task_spawn(&render_task, &render_task_params);

  struct TaskParams test_task_params = {
    .stack_start = test_task_stack,
    .stack_size = sizeof(test_task_stack),
    .func = &test_task_fn,
  };
  task_spawn(&test_task, &test_task_params);

  struct TaskParams progress_task_params = {
    .stack_start = progress_task_stack,
    .stack_size = sizeof(progress_task_stack),
    .func = &progress_task_fn,
  };
  task_spawn(&progress_task, &progress_task_params);
}
