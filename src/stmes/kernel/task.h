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

typedef void TaskFunc(void* user_data);

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
  // NOTE: This must be the second member of the struct!!!
  u32 last_switch_time;
  TaskId id;
  Instant execution_time;
  Instant wait_deadline;
  u8* stack_start;
  usize stack_size;
};

enum TaskStatus {
  TASK_DEAD,
  TASK_READY,
  TASK_SLEEPING,
};

struct Task* scheduler(struct Task* prev_task);

struct Task* get_task_by_id(TaskId id);
struct Task* get_current_task(void);

void task_spawn(struct Task* task, const struct TaskParams* params);
usize task_stack_high_watermark(struct Task* task);
struct Task* task_standard_scheduler(struct Task* prev_task);
__NO_RETURN void task_start_scheduling(void);

// Immediately performs a cooperative context switch.
__STATIC_FORCEINLINE void task_yield(void) {
  // The SVC instruction is used for this because it makes debugging much
  // easier (the debugger can step inside the `SVC_Handler`), unlike the PendSV
  // mechanism (while stepping the exception is not triggered at all, only when
  // resuming execution, as it is an imprecise interrupt). Also, when inlined,
  // the `SVC` instruction is as short as calling the function with `BL` (both
  // are 32-bit).
  __ASM volatile("svc #0" ::: "memory");
}

// Requests a context switch after the current interrupt (and all late-arriving
// ones) have been handled.
__STATIC_INLINE void task_yield_from_isr(void) {
  // PendSV is used for this type of switch (this is literally what it was
  // designed for).
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  __DSB(); // Flush all memory writes
  __ISB(); // Flush the pipeline (to execute the above instruction *right now*)
}

void task_wait_for_events(Instant deadline);
void task_sleep(u32 delay);

enum TaskStatus get_task_status(TaskId id);

struct Notification {
  volatile TasksMask waiters;
};

void task_notify_init(struct Notification* self);
void task_wait(struct Notification* notify, Instant deadline);
bool task_notify(struct Notification* notify);

#ifdef __cplusplus
}
#endif
