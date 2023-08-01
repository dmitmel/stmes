#pragma once

#include "stmes/kernel/time.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef u8 TaskId;
typedef u16 TasksMask;

#define MAX_ALIVE_TASKS (sizeof(TasksMask) * 8)
#define DEAD_TASK_ID (~(TaskId)0)

typedef void* TaskFunc(void* user_data);
typedef struct Task* TaskSchedulerFunc(struct Task* prev_task);

struct TaskParams {
  u8* stack_start;
  usize stack_size;
  TaskFunc* func;
  void* user_data;
};

struct Task {
  // NOTE: The inline assembly in the implementation relies on this being the
  // first member of the struct!!!
  u8* stack_ptr;
  TaskId id;
  Instant wait_deadline;
  u32 execution_time;
  u8* stack_start;
  usize stack_size;
  TaskFunc* func;
  void* user_data;
};

extern TaskSchedulerFunc* task_scheduler;

struct Task* get_task_by_id(TaskId id);
struct Task* get_current_task(void);

void task_spawn(struct Task* task, const struct TaskParams* params);
usize task_stack_high_watermark(struct Task* task);
void task_yield(void);
void task_start_scheduling(void);
struct Task* task_standard_scheduler(struct Task* prev_task);
void task_process_timers(void);
void task_wait_for_events(Instant deadline);
void task_sleep(u32 delay);

struct Notification {
  volatile TasksMask waiters;
};

void task_notify_init(struct Notification* self);
void task_wait_until(struct Notification* notify, Instant deadline);
bool task_notify(struct Notification* notify);

__STATIC_FORCEINLINE void task_wait(struct Notification* notify) {
  task_wait_until(notify, NO_DEADLINE);
}

#ifdef __cplusplus
}
#endif
