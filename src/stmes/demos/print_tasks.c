#include "stmes/demos.h"
#include "stmes/kernel/sync.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include <printf.h>

static struct {
  u8 stack[1024] __ALIGNED(8);
  struct Task task;
} print_tasks[8];

static struct Mutex console_mutex;

static void print_task_fn(__UNUSED void* user_data) {
  u32 task_idx = (u32)user_data;
  u32 counter = 0;
  while (true) {
    mutex_lock(&console_mutex);
    console_set_cursor_line(task_idx);
    console_clear_cursor_line();
    printf("%" PRIu32 " %" PRIu32 "\n", task_idx, counter);
    counter += 1;
    mutex_unlock(&console_mutex);
    task_sleep(task_idx + 1);
  }
}

void print_tasks_demo(void) {
  mutex_init(&console_mutex);

  start_console_render_task();

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
