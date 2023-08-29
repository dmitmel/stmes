#include "stmes/demos.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <printf.h>

static u8 render_task_stack[1024] __ALIGNED(8);
static struct Task render_task;

static u8 timings_task_stack[1024] __ALIGNED(8);
static struct Task timings_task;

static u32 scanline_timings[FRAME_HEIGHT] = { 0 };
static usize scanline_timings_count = 0;
static struct Notification vblank_notification;

static void render_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_wait(&vga_notification, NO_DEADLINE);
    u16 vga_line;
    if (vga_take_scanline_request(&vga_line)) {
      u32 start_time = DWT->CYCCNT;
      bool did_render = console_render_scanline(vga_line);
      u32 end_time = DWT->CYCCNT;
      if (did_render && scanline_timings_count < SIZEOF(scanline_timings)) {
        scanline_timings[scanline_timings_count++] = end_time - start_time;
      }
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
      task_notify(&vblank_notification);
    }
  }
}

static void timings_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_sleep(200);
    task_wait(&vblank_notification, NO_DEADLINE);

    usize len = MIN(scanline_timings_count, SIZEOF(scanline_timings));
    static u32 copied_timings[SIZEOF(scanline_timings)];
    fast_memcpy_u32(copied_timings, scanline_timings, len);

    u32 min = UINT32_MAX, max = 0, sum = 0;
    for (usize i = 0; i < len; i++) {
      u32 time = copied_timings[i];
      min = MIN(min, time), max = MAX(max, time), sum += time;
    }

    for (usize i = 0; i < len; i++) {
      console_set_color(i % 16);
      printf("%" PRIu32 " ", copied_timings[i]);
    }

    console_set_color(CONSOLE_TEXT_ATTRS_RESET);
    printf("\n");
    printf("min: %" PRIu32 ", max: %" PRIu32 ", avg: %" PRIu32, min, max, sum / len);

    scanline_timings_count = 0;
  }
}

void text_rendering_benchmark(void) {
  task_notify_init(&vblank_notification);

  struct TaskParams render_task_params = {
    .stack_start = render_task_stack,
    .stack_size = sizeof(render_task_stack),
    .func = &render_task_fn,
  };
  task_spawn(&render_task, &render_task_params);

  struct TaskParams timings_task_params = {
    .stack_start = timings_task_stack,
    .stack_size = sizeof(timings_task_stack),
    .func = &timings_task_fn,
  };
  task_spawn(&timings_task, &timings_task_params);
}
