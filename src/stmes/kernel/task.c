// The "Callkhoz" cooperative multitasking system.

#include "stmes/kernel/task.h"
#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <stdlib.h>

#define STACK_CHECK_VALUE 0x57ACCE4D // STACCEND or something

static struct TaskSchedulerState {
  struct Task* current_task;
  struct Task* alive_tasks[MAX_ALIVE_TASKS];
  TasksMask alive_tasks_mask, sleeping_tasks_mask, woken_up_tasks_mask, active_timers_mask;
} scheduler_state;

TaskSchedulerFunc* task_scheduler;

struct TaskStackedContext {
  u32 control, r4, r5, r6, r7, r8, r9, r10, r11, pc;
};

struct Task* get_task_by_id(TaskId id) {
  struct TaskSchedulerState* state = &scheduler_state;
  ASSERT(id < SIZEOF(state->alive_tasks));
  struct Task* task = state->alive_tasks[id];
  ASSERT(task != NULL);
  return task;
}

struct Task* get_current_task(void) {
  return scheduler_state.current_task;
}

static void task_launchpad(void) {
  struct Task* task = scheduler_state.current_task;
  task->user_data = task->func(task->user_data);
  task_yield();
}

void task_spawn(struct Task* task, const struct TaskParams* params) {
  ASSERT(task != NULL);
  ASSERT(params != NULL);
  struct TaskSchedulerState* state = &scheduler_state;
  for (usize i = 0; i < SIZEOF(state->alive_tasks); i++) {
    ASSERT(state->alive_tasks[i] != task);
  }

  usize free_idx = 0;
  for (; free_idx < SIZEOF(state->alive_tasks); free_idx++) {
    if (state->alive_tasks[free_idx] == NULL) break;
  }
  ASSERT(free_idx < SIZEOF(state->alive_tasks));

  ASSERT(params->func != NULL);
  ASSERT(params->stack_start != NULL);
  ASSERT(params->stack_size != 0);
  ASSERT((usize)params->stack_start % 8 == 0);
  ASSERT((usize)params->stack_size % 8 == 0);
  task->stack_ptr = params->stack_start + params->stack_size;
  task->id = free_idx;
  task->wait_deadline = NO_DEADLINE;
  task->execution_time = 0;
  task->stack_start = params->stack_start;
  task->stack_size = params->stack_size;
  task->func = params->func;
  task->user_data = params->user_data;

  fast_memset_u32((u32*)task->stack_start, STACK_CHECK_VALUE, task->stack_size / sizeof(u32));

  // Push the initial context needed for cranking the task.
  task->stack_ptr -= sizeof(struct TaskStackedContext);
  struct TaskStackedContext* ctx = (void*)task->stack_ptr;
  // The CONTROL register is initialized in the following way:
  // 1. nPRIV=0 - run in the privileged mode.
  // 2. SPSEL=1 - use the PSP as the current stack.
  // 3. FPCA=0 - floating-point context inactive, i.e. no FPU instructions have
  //    been issued so far and no FP registers have been stacked.
  ctx->control = (0 << CONTROL_nPRIV_Pos) | (1 << CONTROL_SPSEL_Pos) | (0 << CONTROL_FPCA_Pos);
  ctx->r4 = 4, ctx->r5 = 5, ctx->r6 = 6, ctx->r7 = 7;
  ctx->r8 = 8, ctx->r9 = 9, ctx->r10 = 10, ctx->r11 = 11;
  ctx->pc = (usize)task_launchpad;

  state->alive_tasks[free_idx] = task;
  __atomic_fetch_or(&state->alive_tasks_mask, BIT(task->id), __ATOMIC_RELEASE);
}

// Approximately measures the highest stack usage of a task.
usize task_stack_high_watermark(struct Task* task) {
  u32* ptr = (u32*)task->stack_start;
  u32* end = (u32*)(task->stack_start + task->stack_size);
  while (ptr < end && *ptr == STACK_CHECK_VALUE) ptr++;
  return (usize)end - (usize)ptr;
}

struct Task* task_standard_scheduler(struct Task* prev_task) {
  struct TaskSchedulerState* state = &scheduler_state;
  Instant time = systime_now();
  UNUSED(time);

  TasksMask alive_mask = __atomic_load_n(&state->alive_tasks_mask, __ATOMIC_ACQUIRE);
  TasksMask wake_up_mask = __atomic_exchange_n(&state->woken_up_tasks_mask, 0, __ATOMIC_ACQUIRE);
  TasksMask sleeping_mask =
    __atomic_and_fetch(&state->sleeping_tasks_mask, alive_mask & ~wake_up_mask, __ATOMIC_ACQ_REL);
  TasksMask ready_mask = alive_mask & ~sleeping_mask;

  if (ready_mask == 0) {
    return NULL;
  }

  u32 roundrobin_rotation = prev_task->id + 1;
  u32 ready_mask_rotated = __ROR((u32)ready_mask, roundrobin_rotation);
  TaskId id = (__builtin_ctz(ready_mask_rotated) + roundrobin_rotation) % (sizeof(ready_mask) * 8);
  return state->alive_tasks[id];
}

void task_process_timers(void) {
  struct TaskSchedulerState* state = &scheduler_state;
  Instant time = systime_now();
  u32 next_closest_deadline = UINT32_MAX;

  TasksMask wake_up_mask = 0;
  TasksMask alive_mask = __atomic_load_n(&state->alive_tasks_mask, __ATOMIC_ACQUIRE);
  TasksMask timers_mask = __atomic_load_n(&state->active_timers_mask, __ATOMIC_ACQUIRE);
  timers_mask &= alive_mask;

  while (timers_mask != 0) {
    TaskId id = __builtin_ctz(timers_mask);
    timers_mask &= ~BIT(id);
    struct Task* task = state->alive_tasks[id];
    Instant deadline = task->wait_deadline;
    if (time >= deadline) {
      wake_up_mask |= BIT(id);
    } else if ((u32)deadline < next_closest_deadline) {
      next_closest_deadline = (u32)deadline;
    }
  }

  __atomic_fetch_and(&state->active_timers_mask, alive_mask & ~wake_up_mask, __ATOMIC_RELEASE);
  __atomic_fetch_or(&state->woken_up_tasks_mask, wake_up_mask, __ATOMIC_RELEASE);
  // TODO: Race condition here?
  hwtimer_set_alarm(next_closest_deadline);
}

// Calls the scheduler and performs a context switch. Has to be written in
// assembly to reduce the task switching overhead and because we are directly
// dealing with the CPU registers here.
__attribute__((naked)) void task_yield(void) {
  // A notable benefit of using a function to do the context switching is that
  // the calling convention mandates that the registers r0-r3, r12, lr, APSR
  // and s0-s15 don't need to be preserved by the callee, which for us has
  // three useful consequences:
  // 1. The compiler will avoid using those around the calls to `yield`.
  // 2. We need to save and restore half the total number of registers,
  //    speeding up the context switches and reducing the required stack size.
  // 3. We may disregard the values of those registers and use them for local
  //    variables.
  __ASM volatile( //
    // Back up the stack pointer before exchanging the stacks.
    "mov r1, sp\n\t"
    // Before entering the scheduler we want to switch from PSP (Process Stack
    // Pointer, set to the task's stack) to the MSP (Main Stack Pointer). This
    // is done so as to reduce the maximum possible stack usage: if the context
    // switch was performed while we are on the PSP, the task's stack would
    // have to be big enough for the saved registers, AND the space used by the
    // scheduler, AND the registers which may be stacked by an interrupt,
    // including the space reserved for the FP state because the FPU is active.
    "mrs r3, control\n\t"
    // The bit 1, CONTROL.SPSEL, determines which stack pointer is active at
    // the moment: 0 corresponds to the MSP, 1 to the PSP. I write the modified
    // value of CONTROL to a different register because we need to keep the
    // original one around to store it in the task.
    "bic r2, r3, #2\n\t"
    "msr control, r2\n\t"
    // Confirm the write to the CONTROL register.
    "isb\n\t"
#if __FPU_USED == 1
    // The bit 2, CONTROL.FPCA, indicates whether any FPU instructions have
    // been executed so far, and thus whether the task has its own FP state.
    "tst r3, #4\n\t"
    "it ne\n\t"
    // Store the FP registers only if the floating-point context has actually
    // been used (if not, just a single cycle will be wasted stepping over this
    // instruction). They are pushed first so that the core registers lie first
    // on the stack.
    "vstmdbne r1!, {s16-s31}\n\t"
#endif
    // Push the core registers, plus the value of CONTROL which is currently in
    // r3, plus the return address in LR, so that we can resume the execution
    // of the task. Note that the CONTROL is now on top of the task, thus the
    // length of a task's stacked context can be immediately determined. Also,
    // now that the r4-r11 have been saved, they can too be used for our
    // purposes.
    "stmdb r1!, {r3,r4-r11,lr}\n\t"
    // The next two instructions put the address of current_task into r4. The
    // instruction `ldr r4, =%0` (load from the constant pool) is not used
    // because it takes 2-5 cycles (see ARM 100166 section 3.3.3 "Load/store
    // timings") at best, usually 5, and a movw+movt combination predictably
    // takes just 2 cycles. This address is kept in a callee-saved register
    // because we will need it later again.
    "movw r4, #:lower16:%0\n\t"
    "movt r4, #:upper16:%0\n\t"
    // Load the pointer to the current Task.
    "ldr r0, [r4]\n\t"
    // Store the task stack pointer (plus the stacked context) in the task.
    "str r1, [r0, #0]\n\t"
    // Load the address of task_scheduler, the note about movw+movt from above
    // applies here as well.
    "movw r5, #:lower16:%1\n\t"
    "movt r5, #:upper16:%1\n\t"
    "1:\n\t"
    // Load the scheduler function pointer.
    "ldr r2, [r5]\n\t"
    // Call the scheduler function. The pointer to the current task is in r0
    // currently, the stack pointer is in r1 - the 1st and 2nd argument
    // registers respectively.
    "blx r2\n\t"
    "cbz r0, 2f\n\t"
    // The scheduler returns a pointer to the next task (in r0) we want to
    // switch to. Store it in current_task.
    "str r0, [r4]\n\t"
    // Begin the context switch: start by loading the new stack pointer.
    "ldr r1, [r0, #0]\n\t"
    // Pop the other task's core registers from its stack, and its value of the
    // CONTROL register into r3.
    "ldmia r1!, {r3,r4-r11,lr}\n\t"
#if __FPU_USED == 1
    // Reload the next task's floating-point context. See the note about
    // preserving it above.
    "tst r3, #4\n\t"
    "it ne\n\t"
    "vldmiane r1!, {s16-s31}\n\t"
#endif
    // Set the PSP to the task's stack. Note that we are still on the MSP.
    "msr psp, r1\n\t"
    // Exchange the stack pointers.
    "orr r3, #2\n\t"
    "msr control, r3\n\t"
    // Confirm the changes to the CONTROL register.
    "isb\n\t"
    // And, finally, return from this function, jumping into the next task.
    "bx lr\n\t"
    "2:\n\t"
    "ldr r0, [r4]\n\t"
    "ldr r1, [r0, #0]\n\t"
    "b 1b\n\t" :: //
    "i"(&scheduler_state.current_task),
    "i"(&task_scheduler)
  );
}

void task_start_scheduling(void) {
  static struct Task fake_task;
  scheduler_state.current_task = &fake_task;
  task_yield();
}

void task_wait_for_events(Instant deadline) {
  struct TaskSchedulerState* state = &scheduler_state;
  struct Task* task = state->current_task;
  if (likely(deadline != NO_DEADLINE)) {
    __atomic_fetch_and(&state->active_timers_mask, ~BIT(task->id), __ATOMIC_ACQUIRE);
    task->wait_deadline = deadline;
    __atomic_fetch_or(&state->active_timers_mask, BIT(task->id), __ATOMIC_RELEASE);
    task_process_timers();
  }
  __atomic_fetch_or(&state->sleeping_tasks_mask, BIT(task->id), __ATOMIC_RELEASE);
  task_yield();
}

void task_sleep(u32 delay) {
  Instant deadline = systime_now() + delay;
  // TODO: Ensure minimum sleep time?
  while (systime_now() < deadline) {
    task_wait_for_events(deadline);
  }
}

void task_notify_init(struct Notification* self) {
  __atomic_store_n(&self->waiters, 0, __ATOMIC_RELAXED);
}

void task_wait_until(struct Notification* notify, Instant deadline) {
  __atomic_fetch_or(&notify->waiters, BIT(scheduler_state.current_task->id), __ATOMIC_RELAXED);
  task_wait_for_events(deadline);
}

bool task_notify(struct Notification* notify) {
  TasksMask waiters = __atomic_exchange_n(&notify->waiters, 0, __ATOMIC_RELAXED);
  __atomic_fetch_or(&scheduler_state.woken_up_tasks_mask, waiters, __ATOMIC_RELEASE);
  return waiters != 0;
}
