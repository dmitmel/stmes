#pragma once

#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* TaskFunc(void* user_data);
typedef struct Task* TaskSchedulerFunc(struct Task* prev_task);

struct Task {
  // NOTE: The inline assembly in the implementation relies on this being the
  // first member of the struct!!!
  u8* stack_ptr;
  u8* stack_start;
  usize stack_size;
  TaskFunc* func;
  void* user_data;
  bool dead;
  struct Task* next;
};

extern struct Task* current_task;
extern TaskSchedulerFunc* task_scheduler;

void task_init(struct Task* task);

usize task_stack_high_watermark(struct Task* task);

void task_yield(void);
void task_sleep(u32 ms);

#ifdef __cplusplus
}
#endif
