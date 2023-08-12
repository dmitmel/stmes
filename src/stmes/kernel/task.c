#include "stmes/kernel/task.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <stdlib.h>

#define STACK_CHECK_VALUE 0x57ACCE4D // STACCEND or something

static struct TaskSchedulerState {
  struct Task* current_task;
  TasksMask alive_tasks_mask, sleeping_tasks_mask, woken_up_tasks_mask, new_timers_mask;
  u32 pendsv_entry_time;
  Instant next_closest_deadline;
  struct Task* alive_tasks[MAX_ALIVE_TASKS];
} scheduler_state;

// The scheduler will be unlocked only when it has been started.
static volatile bool scheduler_state_lock = true;

__STATIC_INLINE void acquire_scheduler_state(void) {
  bool was_locked = __atomic_test_and_set(&scheduler_state_lock, __ATOMIC_ACQUIRE);
  ASSERT(!was_locked);
}

__STATIC_INLINE void release_scheduler_state(void) {
  __atomic_clear(&scheduler_state_lock, __ATOMIC_RELEASE);
}

struct ExceptionStackedContext {
  u32 r0, r1, r2, r3, r12, lr, pc, xpsr;
};

struct TaskStackedContext {
  u32 control, r4, r5, r6, r7, r8, r9, r10, r11, exc_return;
};

__WEAK struct Task* scheduler(struct Task* prev_task) {
  return prev_task;
}

// TODO: Unsafe, the task may die while we are holding the pointer to it.
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

// This function is the bottom-most frame of every task stack and exists
// largely be a barrier for stack trace unwinding by the debugger (by being
// implemented in assembly and having no hints whatsoever).
static __NO_RETURN __NAKED void task_launchpad(__UNUSED void* user_data, __UNUSED TaskFunc* func) {
  // Call the function passed in the second parameter with the user data as its
  // first parameter which is already in r0.
  __ASM volatile("blx r1");
  // TODO: Task termination isn't supported yet, a return must cause a fault.
  __ASM volatile("udf");
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

  u32 min_stack_size = sizeof(struct TaskStackedContext) + sizeof(struct ExceptionStackedContext);
#if __FPU_USED == 1
  // 32 floating-point registers plus the FPSCR and a word for alignment:
  min_stack_size += sizeof(float) * 32 + sizeof(u32) * 2;
#endif
  ASSERT(params->stack_size >= min_stack_size);

  task->stack_ptr = params->stack_start + params->stack_size;
  task->id = free_idx;
  task->wait_deadline = NO_DEADLINE;
  task->execution_time = 0;
  task->stack_start = params->stack_start;
  task->stack_size = params->stack_size;

  fast_memset_u32((u32*)task->stack_start, STACK_CHECK_VALUE, task->stack_size / sizeof(u32));

  // Manufacture an initial context needed for cranking the task.
  {
    task->stack_ptr -= sizeof(struct ExceptionStackedContext);
    struct ExceptionStackedContext* ctx = (void*)task->stack_ptr;
    // Pass the arguments for `task_launchpad` in r0 and r1.
    ctx->r0 = (usize)params->user_data;
    ctx->r1 = (usize)params->func;
    // The other general-purpose registers are set to dummy values.
    ctx->r2 = 2, ctx->r3 = 3, ctx->r12 = 12;
    // Write garbage into LR so that returning causes an explosion.
    ctx->lr = 0xFFFFFFFF;
    // Execution will start here:
    ctx->pc = (usize)task_launchpad;
    // The EPSR.T (Thumb State) bit needs to be set depending on the last bit
    // of PC (which will always be 1 on Cortex-M CPUs since they only executed
    // Thumb code), which matches the CPU reset behavior:
    // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Reset-behavior>
    ctx->xpsr = (ctx->pc & 1) << xPSR_T_Pos;
  }
  {
    task->stack_ptr -= sizeof(struct TaskStackedContext);
    struct TaskStackedContext* ctx = (void*)task->stack_ptr;
    // The bits of the CONTROL registers are largely irrelevant and must be
    // reset, only CONTROL.nPRIV matters.
    ctx->control = (0 << CONTROL_nPRIV_Pos) | (0 << CONTROL_SPSEL_Pos) | (0 << CONTROL_FPCA_Pos);
    // The other general-purpose registers are set to dummy values.
    ctx->r4 = 4, ctx->r5 = 5, ctx->r6 = 6, ctx->r7 = 7;
    ctx->r8 = 8, ctx->r9 = 9, ctx->r10 = 10, ctx->r11 = 11;
    // The initial EXC_RETURN value is such that the return happens into Thread
    // mode, using PSP as the stack and the floating-point context is inactive,
    // i.e. no FPU instructions have been issued so far and no FP registers
    // have been stacked.
    ctx->exc_return = EXC_RETURN_THREAD_PSP;
  }

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

static TasksMask task_process_timers(TasksMask timers_mask, u64 now) {
  struct TaskSchedulerState* state = &scheduler_state;
  TasksMask wake_up_mask = 0;
  u64 closest_deadline = NO_DEADLINE;
  while (timers_mask != 0) {
    TaskId id = __builtin_ctz(timers_mask);
    timers_mask &= ~BIT(id);
    struct Task* task = state->alive_tasks[id];
    Instant deadline = task->wait_deadline;
    if (now >= deadline) {
      wake_up_mask |= BIT(id);
    } else if (deadline < closest_deadline) {
      closest_deadline = deadline;
    }
  }
  state->next_closest_deadline = closest_deadline;
  // TODO: hwtimer_set_alarm(closest_deadline);
  return wake_up_mask;
}

struct Task* task_standard_scheduler(struct Task* prev_task) {
  acquire_scheduler_state();
  struct TaskSchedulerState* state = &scheduler_state;

  TasksMask alive_mask = __atomic_load_n(&state->alive_tasks_mask, __ATOMIC_ACQUIRE);
  TasksMask wake_up_mask = __atomic_exchange_n(&state->woken_up_tasks_mask, 0, __ATOMIC_ACQUIRE);
  TasksMask sleeping_mask =
    __atomic_and_fetch(&state->sleeping_tasks_mask, alive_mask & ~wake_up_mask, __ATOMIC_ACQUIRE);
  TasksMask new_timers = __atomic_exchange_n(&state->new_timers_mask, 0, __ATOMIC_ACQUIRE);

  Instant now = systime_now();
  if (unlikely(now >= state->next_closest_deadline || new_timers != 0)) {
    wake_up_mask = task_process_timers(sleeping_mask, now);
    sleeping_mask =
      __atomic_and_fetch(&state->sleeping_tasks_mask, ~wake_up_mask, __ATOMIC_ACQ_REL);
  }

  TasksMask ready_mask = alive_mask & ~sleeping_mask;
  struct Task* task = NULL;
  if (ready_mask != 0) {
    u32 rotation = prev_task->id + 1;
    u32 ready_mask_rotated = __ROR((u32)ready_mask, rotation);
    TaskId id = (__builtin_ctz(ready_mask_rotated) + rotation) % (sizeof(ready_mask) * 8);
    task = state->alive_tasks[id];
  }
  release_scheduler_state();
  return task;
}

static void on_task_switch(struct Task* task, u32 current_time) {
  // Didn't want to write out the inline assembly for this, so have put this
  // into a C function.
  // TODO: This will also take time spent in the interrupt handlers called
  // while the task was executing into account.
  task->execution_time += current_time - task->last_switch_time;
}

// Since the only thing we currently implement through the SVC exception is
// cooperative context switching with `task_yield` (we don't yet have a syscall
// interface), to reduce the calling overhead the SVC handler is simply aliased
// to the PendSV one. Implementing both exceptions with the same handler is
// safe since they are set up to run at the same priority and thus can't
// preempt each other.
__ALIAS("PendSV_Handler") void SVC_Handler(void);

// The core of the context switcher, based around the PendSV exception. Calls
// the scheduler and performs a switch if necessary. Has to be written in
// assembly to reduce the task switching overhead and because we are directly
// dealing with the CPU registers here.
__NAKED void PendSV_Handler(void) {
  // A quick note before we start off: before entering the interrupt, the
  // hardware has already stacked a bunch of CPU registers, switched into
  // privileged mode and set the stack pointer to MSP (Main Stack Pointer),
  // which will all be undone after leaving the exception. All of this usually
  // takes ~10 cycles for entering and another ~10 for leaving, though the
  // actual numbers will vary. In any case, at the start the registers r0-r3
  // can be freely clobbered by us since the hardware will restore them.
  // <https://developer.arm.com/documentation/ddi0439/b/Programmers-Model/Exceptions/Exception-handling>

  __ASM volatile( //
    // First things first, load the address of the CPU cycle counter to measure
    // it to increase the execution time of the current task. This must be done
    // first so as to not bill the task for the run time of the scheduler,
    // though it will include the overhead of interrupt entry.
    "movw r1, #:lower16:%2\n\t"
    "movt r1, #:upper16:%2\n\t"
    // Load the value of `DWT->CYCCNT`.
    "ldr r1, [r1]\n\t"

    // We need to back up the LR register (since we will need its EXC_RETURN
    // value for returning from the exception) before calling the scheduler
    // function, but to keep the stack aligned, another register has to be
    // backed up as well (which will come in handy later).
    "push {r4, lr}\n\t"
#ifdef __PLATFORMIO_BUILD_DEBUG__
    // The CFI directives make the assembler put a certain section into the
    // binary that informs the debugger how to unwind the stack and recover
    // local variables in caller frames. They don't generate any machine code
    // and were added purely for the convenience of debugging. We need to mark
    // the registers that have been pushed on the stack:
    ".cfi_adjust_cfa_offset 8\n\t"
    ".cfi_rel_offset r4, 0\n\t"
    ".cfi_rel_offset lr, 4\n\t"
#endif

    // The next two instructions load the address of `current_task`. The
    // instruction `ldr rX, =%0` (load from the constant pool) is not used
    // because it takes 2-5 cycles (see ARM 100166 section 3.3.3 "Load/store
    // timings") at best, usually 5, and a MOVW+MOVT combination predictably
    // takes just 2 cycles.
    "movw r0, #:lower16:%0\n\t"
    "movt r0, #:upper16:%0\n\t"
    // Load the pointer to the current task and put it into r4, which has been
    // backed up, so we can now use it for our purposes. The pointer is kept in
    // a callee-saved register because we will need it later after invoking the
    // scheduler.
    "ldr r4, [r0]\n\t"

    // Skip over the next block if the current_task was NULL (we will need to
    // talk about this again later).
    "cbz r4, 3f\n\t"
    // Call `on_task_switch`, which will perform some preparations for calling
    // the scheduler. The task pointer for the `task` argument needs to be
    // moved into r0, and the value for `current_time` is already in r1.
    "mov r0, r4\n\t"
    "bl %3\n\t"
    "3:\n\t"
    // The `scheduler` function may now be called. The task pointer needs to be
    // once again moved into r0, and the pointer to the next task will be
    // returned into the same register; r4 will effectively contain the pointer
    // to the previous task.
    "mov r0, r4\n\t"
    "bl %1\n\t"

    // Now that the next task has been determined, prepare to actually switch
    // the tasks. We first want to pop the registers which were pushed above.
    // The value preserved in r4 needs to be moved to another register first,
    // otherwise it will be lost:
    "mov r1, r4\n\t"
    "pop {r4, lr}\n\t"
#ifdef __PLATFORMIO_BUILD_DEBUG__
    // Here come the CFI directives once again, this time to inform the
    // debugger that the values of the backed up registers are the same that
    // they were at the beginning of the function.
    ".cfi_adjust_cfa_offset -8\n\t"
    ".cfi_restore r4\n\t"
    ".cfi_restore lr\n\t"
#endif
    // At this point: r0 - next task, r1 - prev task, r2 - task stack pointer.

    // We can take a shortcut if the next task and the previous one are the
    // same, skipping over the context switch:
    "cmp r0, r1\n\t"
    "beq 2f\n\t"

    // Load the address of the `current_task` variable once again.
    "movw r3, #:lower16:%0\n\t"
    "movt r3, #:upper16:%0\n\t"
    // Store the next task pointer in `current_task`.
    "str r0, [r3]\n\t"

    // NOTE: Here is where the actual context switching begins. It is
    // implemented with some assistance from hardware: after all, since the CPU
    // saves a part of context by pushing it on the stack upon exception entry,
    // the stack pointer may simply be edited to point to a different block of
    // saved context, which may, for instance, reside on the stack of another
    // task. Hence, returning from this handler will jump into an entirely
    // different task! The problem is, the hardware stacks only half the
    // normal and FPU registers, and we must stack the rest ourselves.

    // Saving the state must be skipped if the previous task was NULL.
    // TODO: This will only ever happen once for the entire run time of the
    // system, when entering the very first task, can this check be avoided?
    "cbz r1, 1f\n\t"
    // Alright, all preparations and pre-flight checks have been completed.
    // Load the current PSP (Process Stack Pointer) - it is used while the CPU
    // is running in the Thread mode (normal execution of tasks), and MSP (Main
    // Stack Pointer) is used in the Handler mode (processing of interrupts).
    "mrs r2, psp\n\t"
    // Read the CONTROL register. The only bit we are interested in preserving
    // is CONTROL.nPRIV, which determines whether the CPU is in the privileged
    // execution mode or not. The bits CONTROL.SPSEL (MSP is always used in the
    // Handler mode, and we always use PSP for tasks anyway) and CONTROL.FPCA
    // are reset upon entering the interrupt.
    "mrs r3, control\n\t"
#if __FPU_USED == 1
    // The bit 4 of EXC_RETURN determines whether the stacked state includes
    // the FPU registers, and thus whether the task has issued any FPU
    // instructions so far.
    "tst lr, #10\n\t"
    "it eq\n\t"
    // Store the FP registers only if the floating-point context has actually
    // been used (if not, just a single cycle will be wasted stepping over this
    // instruction). They are pushed first so that the core registers lie on
    // the top of the stack. Note that if lazy FP state preservation is enabled
    // (which it always is, I guess), issuing this instruction will also
    // implicitly cause the registers s0-s15 to be stored as well in the space
    // allocated in the stack frame for the exception. This is 32 memory
    // transactions caused by a single instruction!
    "vstmdbeq r2!, {s16-s31}\n\t"
#endif
    // Push the rest of the core registers, plus the value of CONTROL which is
    // currently in r3, plus the EXC_RETURN in LR, so that we can resume the
    // execution of the task. The bits of EXC_RETURN can be used to determine
    // if any FP context has been pushed.
    "stmdb r2!, {r3, r4-r11, lr}\n\t"
    // Store the adjusted stack pointer in the previous task struct.
    "str r2, [r1, #0]\n\t"

    "1:\n\t"
    // Load the stack pointer of the next task.
    "ldr r2, [r0, #0]\n\t"
    // Pop the other task's core registers from its stack, its value of the
    // CONTROL register into r3 and its EXC_RETURN value into LR.
    "ldmia r2!, {r3, r4-r11, lr}\n\t"
#if __FPU_USED == 1
    // Reload the next task's floating-point context. See the note about
    // preserving it above.
    "tst lr, #10\n\t"
    "it eq\n\t"
    "vldmiaeq r2!, {s16-s31}\n\t"
#endif
    // Write the new value of the CONTROL register.
    "msr control, r3\n\t"
    // Lastly, exchange the stack pointers.
    "msr psp, r2\n\t"
    // NOTE: An instruction synchronization barrier is not needed here because
    // the return from an exception acts as one.

    "2:\n\t"
    // Before we will be done, we must save the value of the CPU cycle counter
    // at the time of switch into the other task. Load its address:
    "movw r3, #:lower16:%2\n\t"
    "movt r3, #:upper16:%2\n\t"
    // Then load the value of `DWT->CYCCNT`:
    "ldr r3, [r3]\n\t"
    // And save it into the next task struct.
    "str r3, [r0, #4]\n\t"

    // Finally, return from the interrupt handler, jumping into the next task.
    "bx lr\n\t" :: //

    "i"(&scheduler_state.current_task),
    "i"(&scheduler),
    "i"(&DWT->CYCCNT),
    "i"(&on_task_switch)
  );
}

__NO_RETURN void task_start_scheduling(void) {
  __atomic_clear(&scheduler_state_lock, __ATOMIC_RELAXED);
  task_yield();
  __builtin_unreachable();
}

void task_wait_for_events(Instant deadline) {
  struct TaskSchedulerState* state = &scheduler_state;
  struct Task* task = state->current_task;
  TasksMask task_mask = BIT(task->id);
  task->wait_deadline = deadline;
  if (unlikely(deadline != NO_DEADLINE)) {
    __atomic_fetch_or(&state->new_timers_mask, task_mask, __ATOMIC_RELEASE);
  }
  __atomic_fetch_or(&state->sleeping_tasks_mask, task_mask, __ATOMIC_RELEASE);
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

void task_wait(struct Notification* notify, Instant deadline) {
  __atomic_fetch_or(&notify->waiters, BIT(scheduler_state.current_task->id), __ATOMIC_RELAXED);
  task_wait_for_events(deadline);
}

bool task_notify(struct Notification* notify) {
  TasksMask waiters = __atomic_exchange_n(&notify->waiters, 0, __ATOMIC_RELAXED);
  __atomic_fetch_or(&scheduler_state.woken_up_tasks_mask, waiters, __ATOMIC_RELEASE);
  return waiters != 0;
}

enum TaskStatus get_task_status(TaskId id) {
  struct TaskSchedulerState* state = &scheduler_state;
  TasksMask alive_mask = __atomic_load_n(&state->alive_tasks_mask, __ATOMIC_RELAXED);
  TasksMask sleeping_mask = __atomic_load_n(&state->sleeping_tasks_mask, __ATOMIC_RELAXED);
  if (!(alive_mask & BIT(id))) {
    return TASK_DEAD;
  } else if (sleeping_mask & BIT(id)) {
    return TASK_SLEEPING;
  } else {
    return TASK_READY;
  }
}
