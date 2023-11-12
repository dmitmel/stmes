#pragma once

#include "stmes/kernel/time.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

enum Syscall {
  SYSCALL_YIELD,
  SYSCALL_WAIT,
  SYSCALL_SPAWN,
  SYSCALL_EXIT,
  SYSCALLS_COUNT,
};

// The system call ABI is described in the implementation file.

__STATIC_FORCEINLINE usize syscall_0(enum Syscall nr) {
  register usize ret __ASM("r0");
  __ASM volatile("svc %1" : "=r"(ret) : "n"(nr) : "memory");
  return ret;
}

__STATIC_FORCEINLINE usize syscall_1(enum Syscall nr, usize a) {
  register usize ret __ASM("r0"), r0 __ASM("r0") = a;
  __ASM volatile("svc %1" : "=r"(ret) : "n"(nr), "r"(r0) : "memory");
  return ret;
}

__STATIC_FORCEINLINE usize syscall_2(enum Syscall nr, usize a, usize b) {
  register usize ret __ASM("r0"), r0 __ASM("r0") = a, r1 __ASM("r1") = b;
  __ASM volatile("svc %1" : "=r"(ret) : "n"(nr), "r"(r0), "r"(r1) : "memory");
  return ret;
}

__STATIC_FORCEINLINE usize syscall_3(enum Syscall nr, usize a, usize b, usize c) {
  register usize ret __ASM("r0"), r0 __ASM("r0") = a, r1 __ASM("r1") = b, r2 __ASM("r2") = c;
  __ASM volatile("svc %1" : "=r"(ret) : "n"(nr), "r"(r0), "r"(r1), "r"(r2) : "memory");
  return ret;
}

__STATIC_FORCEINLINE usize syscall_4(enum Syscall nr, usize a, usize b, usize c, usize d) {
  register usize ret __ASM("r0");
  register usize r0 __ASM("r0") = a, r1 __ASM("r1") = b, r2 __ASM("r2") = c, r3 __ASM("r3") = d;
  __ASM volatile("svc %1" : "=r"(ret) : "n"(nr), "r"(r0), "r"(r1), "r"(r2), "r"(r3) : "memory");
  return ret;
}

typedef u8 TaskId;
typedef u8 TaskPriority;
typedef u32 TasksMask;

struct Notification {
  volatile TasksMask waiters;
};

#define NOTIFICATION_INIT \
  { .waiters = 0 }

#define MAX_ALIVE_TASKS (sizeof(TasksMask) * 8)
#define DEAD_TASK_ID ((TaskId)-1)

typedef void TaskFn(void* user_data);

struct TaskParams {
  u8* stack_start;
  usize stack_size;
  TaskFn* func;
  void* user_data;
};

struct Task {
  u8* stack_ptr;
  TaskId id;
  TaskPriority priority;
  struct Task *next, *prev;
  Systime wait_deadline;
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

void start_task_scheduler(void);
void task_notify_init(struct Notification* self);
TasksMask task_notify(struct Notification* notification);
void task_sleep(u32 delay_ms);
void task_sleep_until(Systime deadline);
void task_join(struct Task* other_task); // TODO, requires its own syscall
void task_spawn(struct Task* task, const struct TaskParams* params);
usize task_stack_high_watermark(struct Task* task);

// Immediately performs a cooperative context switch.
__STATIC_FORCEINLINE void task_yield(void) {
  // The SVC instruction is used for this because it makes debugging much
  // easier (the debugger can step inside the `SVC_Handler`), unlike the PendSV
  // mechanism (while stepping the exception is not triggered at all, only when
  // resuming execution, as it is an imprecise interrupt).
  syscall_0(SYSCALL_YIELD);
}

// Requests a context switch after the current interrupt (and all late-arriving
// ones) have been handled.
__STATIC_INLINE void task_yield_from_isr(void) {
  // PendSV is used for this type of switch (this is literally what it was
  // designed for).
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

__STATIC_FORCEINLINE __NO_RETURN void task_exit(void) {
  syscall_0(SYSCALL_EXIT);
  __builtin_unreachable();
}

__STATIC_FORCEINLINE void task_wait(struct Notification* notification, Systime deadline) {
  syscall_3(SYSCALL_WAIT, (u32)deadline, (u32)(deadline >> 32), (usize)notification);
}

#ifdef __cplusplus
}
#endif
