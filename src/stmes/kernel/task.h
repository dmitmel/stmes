#pragma once

#include "stmes/kernel/time.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

enum Syscall {
  SYSCALL_SLEEP,
  SYSCALL_YIELD,
  SYSCALL_SPAWN,
  SYSCALL_EXIT,
  SYSCALLS_COUNT,
};

// The system call ABI is described in the implementation file.

__STATIC_FORCEINLINE void syscall_0(enum Syscall nr) {
  register usize r7 __ASM("r7") = nr;
  __ASM volatile("svc #0" ::"r"(r7) : "memory");
}

__STATIC_FORCEINLINE void syscall_1(enum Syscall nr, usize a) {
  register usize r7 __ASM("r7") = nr, r4 __ASM("r4") = a;
  __ASM volatile("svc #0" ::"r"(r7), "r"(r4) : "memory");
}

__STATIC_FORCEINLINE void syscall_2(enum Syscall nr, usize a, usize b) {
  register usize r7 __ASM("r7") = nr, r4 __ASM("r4") = a, r5 __ASM("r5") = b;
  __ASM volatile("svc #0" ::"r"(r7), "r"(r4), "r"(r5) : "memory");
}

__STATIC_FORCEINLINE void syscall_3(enum Syscall nr, usize a, usize b, usize c) {
  register usize r7 __ASM("r7") = nr, r4 __ASM("r4") = a, r5 __ASM("r5") = b, r6 __ASM("r6") = c;
  __ASM volatile("svc #0" ::"r"(r7), "r"(r4), "r"(r5), "r"(r6) : "memory");
}

typedef u8 TaskId;
typedef u8 TaskPriority;
typedef u32 TasksMask;

#define MAX_ALIVE_TASKS (sizeof(TasksMask) * 8)
#define DEAD_TASK_ID ((TaskId)-1)

typedef void TaskFunc(void* user_data);

struct TaskParams {
  u8* stack_start;
  usize stack_size;
  TaskFunc* func;
  void* user_data;
};

struct Task {
  u8* stack_ptr;
  TaskId id;
  TaskPriority priority;
  struct Task *next, *prev;
  Instant wait_deadline;
  u8* stack_start;
  usize stack_size;
};

__STATIC_INLINE struct Task* get_current_task(void) {
  // A wrapper function is used to hide the variable and thus make it read-only
  // outside the implementation file. Thanks to inlining though, calls to this
  // function will compile down to a memory load instead of a full-blown call.
  extern struct Task* current_task;
  return current_task;
}

struct Task* task_scheduler(enum Syscall syscall_nr, struct Task* prev_task);
void start_task_scheduler(void);
void task_spawn(struct Task* task, const struct TaskParams* params);
usize task_stack_high_watermark(struct Task* task);

// Immediately performs a cooperative context switch.
__STATIC_FORCEINLINE void yield(void) {
  // The SVC instruction is used for this because it makes debugging much
  // easier (the debugger can step inside the `SVC_Handler`), unlike the PendSV
  // mechanism (while stepping the exception is not triggered at all, only when
  // resuming execution, as it is an imprecise interrupt).
  syscall_0(SYSCALL_YIELD);
}

// Requests a context switch after the current interrupt (and all late-arriving
// ones) have been handled.
__STATIC_INLINE void yield_from_isr(void) {
  // PendSV is used for this type of switch (this is literally what it was
  // designed for).
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  // __DSB(); // Flush all memory writes
  // __ISB(); // Flush the pipeline (to execute the above instruction *right now*)
}

__STATIC_INLINE void task_wait_for_events(Instant deadline) {
  // Even though writes of 64-bit ints are not atomic on ARMv7m, it doesn't
  // matter since the value of `wait_deadline` is considered valid only after
  // the task has been put to sleep (basically, the execution status of the
  // task acts as a lock here).
  get_current_task()->wait_deadline = deadline;
  syscall_0(SYSCALL_SLEEP);
}

__STATIC_INLINE __NO_RETURN void task_exit(void) {
  syscall_0(SYSCALL_EXIT);
  __builtin_unreachable();
}

void task_sleep(u32 delay);
void task_join(struct Task* other_task); // TODO, requires its own syscall

struct Notification {
  volatile TasksMask waiters;
};

void task_notify_init(struct Notification* self);
void task_wait(struct Notification* notify, Instant deadline);
TasksMask task_notify(struct Notification* notify);

#ifdef __cplusplus
}
#endif
