#include "stmes/demos.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <printf.h>

static u8 render_task_stack[1024] __ALIGNED(8);
static struct Task render_task;

static TaskFunc print_task_fn;
static struct {
  u8 stack[1024] __ALIGNED(8);
  struct Task task;
} print_tasks[8];

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

static void print_task_fn(__UNUSED void* user_data) {
  u32 task_idx = (u32)user_data;
  u32 counter = 0;
  while (true) {
    console_set_cursor_line(task_idx);
    console_clear_cursor_line();
    printf("%" PRIu32 " %" PRIu32 "\n", task_idx, counter);
    counter += 1;
    task_sleep(task_idx + 1);
  }
}

void print_tasks_demo(void) {
  struct TaskParams render_task_params = {
    .stack_start = render_task_stack,
    .stack_size = sizeof(render_task_stack),
    .func = &render_task_fn,
  };
  task_spawn(&render_task, &render_task_params);

  for (usize i = 0; i < SIZEOF(print_tasks); i++) {
    struct TaskParams params = {
      .stack_start = print_tasks[i].stack,
      .stack_size = SIZEOF(print_tasks[i].stack),
      .func = &print_task_fn,
      .user_data = (void*)i,
    };
    task_spawn(&print_tasks[i].task, &params);
  }
}
