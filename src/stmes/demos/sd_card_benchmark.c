#include "stmes/demos.h"
#include "stmes/fatfs.h"
#include "stmes/kernel/task.h"
#include "stmes/sdio.h"
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

static void test_task_fn(__UNUSED void* user_data) {
  while (BSP_SD_Init() != HAL_OK) {
    printf(".");
    task_sleep(1000);
  }

  console_clear_screen();

  check_fs_error(f_mount(&SDFatFS, "", 1));

  while (true) {
    check_fs_error(f_open(&SDFile, "bebop_palette.bin", FA_READ));

    printf("loading %lu\n", f_size(&SDFile));
    task_yield();

    usize total_bytes = 0;
    static char buf[BLOCKSIZE * 8];
    Systime start_time = systime_now();

    while (true) {
      task_notify(&progress_task_notify);
      // yield();
      usize bytes_read = 0;
      if (f_read(&SDFile, buf, sizeof(buf), &bytes_read) != FR_OK) {
        break;
      }
      if (bytes_read == 0) {
        break;
      }
      total_bytes += bytes_read;
    }

    // for (u32 i = 0, sectors = sizeof(buf) / BLOCKSIZE; true; i += sectors) {
    //   task_notify(&progress_task_notify);
    //   // yield();
    //   disk_read(0, (BYTE*)buf, i, sectors);
    //   u32 bytes_read = BLOCKSIZE * sectors;
    //   if (total_bytes >= 1024 * 1024) {
    //     break;
    //   }
    //   total_bytes += bytes_read;
    // }

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
  }
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
