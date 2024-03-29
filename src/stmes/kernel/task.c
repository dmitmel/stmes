// The heart of my improvised nanokernel - a preemptive task scheduler with a
// round-robin scheduling policy. Implemented in a more or less conventional
// manner of writing RTOSes for Cortex-M CPUs, it utilizes the SVCall interrupt
// for making syscalls and the PendSV one for preemptively entering the
// scheduler.
//
// Context switching is performed with the help of hardware: on exception entry
// the CPU pushes a subset of registers onto the stack, which we complement by
// saving the rest of the important state ourselves. The PendSV exception is
// particularly helpful here: it fires only when there are no other pending
// interrupts, so by being assigned the absolute lowest priority it can't
// preempt any other ISR, which guarantees that manipulating the saved context
// (in particular rewriting the stack pointer) is always safe and returning
// from it will always resume the execution of a task. Other interrupt handlers
// may set the ICSR.PENDSVSET bit to essentially request a deferred context
// switch after the processing of the current interrupt is complete.
//
// The SVCall interrupt is given the lowest possible priority out of related
// concerns: for one, it must be at that same priority level as PendSV, so that
// the "kernel" itself can't preempt itself (and break the fragile scheduling
// logic). This also means that the kernel and the scheduler may be interrupted
// by higher-priority IRQs, though they aren't supposed to touch the kernel
// directly or make syscalls, so that is perfectly fine and results in lower
// interrupt latency. As an additional point, in the current implementation
// system calls always cause a context switch.
//
// Although the scheduler itself is currently rather primitive, it makes use of
// doubly-linked circular lists as ready queues, which give excellent task
// switching performance. Another significant optimization is the use of
// bitmasks for waking up tasks with notifications - a single 32-bit mask can
// essentially encode a list of 32 tasks, which greatly simplifies the structs
// of synchronization primitives and makes them a lot more compact.

// Some code and resources that have been useful or influenced my implementation:
// <https://github.com/shinyblink/sled/blob/5bf73fe619aa48f2f697ab77dbd89a83f4c555d6/src/os/os_nrf51.c>
// <https://www.youtube.com/watch?v=yHqaspeGJRw> - A deep-dive into the Chromium-EC OS
// <https://github.com/coreboot/chrome-ec/blob/f00367769f565573e84e657ceef825ea2e07ac6d/core/cortex-m/task.c>
// <https://github.com/coreboot/chrome-ec/blob/f00367769f565573e84e657ceef825ea2e07ac6d/core/cortex-m/switch.S>
// <https://github.com/coreboot/chrome-ec/blob/f00367769f565573e84e657ceef825ea2e07ac6d/common/timer.c>
// <https://github.com/coreboot/chrome-ec/blob/f00367769f565573e84e657ceef825ea2e07ac6d/chip/stm32/hwtimer32.c>
// <https://github.com/oxidecomputer/hubris/blob/4176179293d3fcf0e77345ae26e71418a9ceeef5/sys/kern/src/arch/arm_m.rs>
// <https://github.com/tock/tock/blob/1a111a3e748815117b0ef939ab730b31d77a409d/kernel/src/scheduler/round_robin.rs>
// <https://interrupt.memfault.com/blog/cortex-m-rtos-context-switching>
// <https://interrupt.memfault.com/blog/arm-cortex-m-exceptions-and-nvic>
// <https://graphitemaster.github.io/fibers/>
// <https://medium.com/@dheeptuck/building-a-real-time-operating-system-rtos-ground-up-a70640c64e93>
// <https://www.adamh.cz/blog/2016/07/context-switch-on-the-arm-cortex-m0/>
// <http://www.ethernut.de/en/documents/arm-inline-asm.html>
// <https://github.com/oxidecomputer/hubris>
// <https://www.youtube.com/watch?v=cypmufnPfLw> - The Pragmatism of Hubris
// <http://cliffle.com/p/lilos/>

// TODO: Time slicing.

// TODO: Task priorities and a multi-level feedback queue scheduler.

// TODO: Yield hints. A couple of notes:
// 1. They can be passed as a parameter to the yield syscall, but for yields
//    triggered from ISRs through PendSV they need to be accumulated in an
//    atomic variable (__atomic_fetch_or).
// 2. Since syscalls always yield in my implementation, the syscall functions
//    themselves should return a yield hint, e.g. the spawn syscall may want to
//    request to yield to the newly created task.

// TODO: Terminated tasks currently can't clear their own state, such as
// freeing the stack if it was dynamically allocated.

#include "stmes/kernel/task.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <stdlib.h>

#define STACK_CHECK_VALUE 0x57ACCE4D // STACCEND or something

struct Task* current_task;

static struct TaskSchedulerState {
  struct Task *ready_queue_head, *sleeping_queue_head;
  TasksMask woken_up_tasks_mask; // must be updated atomically
  Systime next_closest_deadline;
} scheduler_state;

// The scheduler will be unlocked only after it has been started. The kernel
// functions should then lock the scheduler state before using it.
static volatile bool scheduler_state_lock = true;

void start_task_scheduler(void) {
  struct TaskSchedulerState* state = &scheduler_state;
  *state = (struct TaskSchedulerState){ 0 }; // Zero-initialize the whole struct
  state->next_closest_deadline = NO_DEADLINE;

  HAL_NVIC_SetPriority(SVCall_IRQn, 0xF, 0xF);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0xF, 0xF);

  __atomic_clear(&scheduler_state_lock, __ATOMIC_RELEASE);
}

// Inserts a task at the head of the queue in O(1) time and returns the
// updated queue head.
__STATIC_INLINE struct Task* task_queue_insert(struct Task* head, struct Task* node) {
  if (unlikely(head == NULL)) { // The list was empty
    node->prev = node->next = node;
    return node;
  }
  struct Task *next = head, *prev = head->prev;
  node->next = next, node->prev = prev; // Link the node to the surrounding nodes
  prev->next = next->prev = node;       // Link the node into the list
  return node;
}

// Removes a task from the queue in O(1) time (thanks to its doubly-linked
// nature) and returns the updated head.
__STATIC_INLINE struct Task* task_queue_remove(struct Task* head, struct Task* node) {
  struct Task *prev = node->prev, *next = node->next;
  if (unlikely(next == node)) { // This was the last node
    return NULL;
  }
  prev->next = next, next->prev = prev; // Unlink the node out of the list
  return head == node ? next : head;
}

__STATIC_INLINE struct Task* task_queue_find_by_id(struct Task* head, TaskId id) {
  if (head == NULL) return NULL;
  struct Task* task = head;
  do {
    if (task->id == id) {
      return task;
    }
    task = task->next;
  } while (task != head);
  return NULL;
}

// This function shouldn't be inlined: timers expire relatively rarely, and
// since this function uses a lot of registers for local variables, inlining it
// will make the main scheduler function push a ton of registers every time.
static TasksMask process_task_timers(Systime now) {
  struct TaskSchedulerState* state = &scheduler_state;
  TasksMask wake_up_mask = 0;
  struct Task* first_task = state->sleeping_queue_head;
  if (unlikely(first_task == NULL)) return wake_up_mask;

  Systime closest_deadline = NO_DEADLINE;
  struct Task* task = first_task;
  do {
    Systime deadline = task->wait_deadline;
    if (now >= deadline) {
      // TODO: Idk, assembling a mask for the tasks with expired timers and
      // waking them up through the common wakeup routine seems to work faster.
      wake_up_mask |= BIT(task->id);
    } else if (deadline < closest_deadline) {
      closest_deadline = deadline;
    }
    task = task->next;
  } while (task != first_task);

  state->next_closest_deadline = closest_deadline;
  // TODO: hwtimer_set_alarm((u32)closest_deadline);
  return wake_up_mask;
}

// This function shouldn't be inlined as well.
static void wake_up_tasks(TasksMask mask) {
  static struct TaskSchedulerState* state = &scheduler_state;
  struct Task* first_task = state->sleeping_queue_head;
  if (unlikely(first_task == NULL)) return;

  // A fast path which is taken surprisingly often, like in the ~80% of cases
  // (judging by my demos). NOTE that this will have to be removed if we start
  // supporting running more tasks than TasksMask has bits.
  if (likely(mask == BIT(first_task->id))) {
    state->sleeping_queue_head = task_queue_remove(state->sleeping_queue_head, first_task);
    state->ready_queue_head = task_queue_insert(state->ready_queue_head, first_task);
    return;
  }

  struct Task* task = first_task;
  while (true) {
    struct Task* next_task = task->next;
    bool is_last = next_task == first_task;

    if (mask & BIT(task->id)) {
      state->sleeping_queue_head = first_task =
        task_queue_remove(state->sleeping_queue_head, task);
      state->ready_queue_head = task_queue_insert(state->ready_queue_head, task);
      // Exit if we have fully cleared out the sleeping queue.
      if (first_task == NULL) break;
    }

    if (is_last) break;
    task = next_task;
  }
}

// Referencing the scheduler_state directly in this function generates useless
// address loads for its every field, so it's better to break the optimizer's
// assumptions and pass this address from outside. A schizo-optimization, I
// must admit.
static void syscall_wait(
  struct TaskSchedulerState* state, Systime deadline, struct Notification* notification
) {
  struct Task* task = current_task;
  if (notification != NULL) {
    // NOTE: The actual assignment of a task to a notification happens inside
    // the WAIT syscall function as a workaround for a subtle race condition.
    // In the way it had been done before, i.e. having the atomic-or operation
    // to register ourselves as waiting for the notification inside `task_wait`
    // function and then making the WAIT syscall, introduced a bug which would
    // cause notifications to be dropped and create a sleeping task that would
    // never wake up. Suppose an interrupt happens in between the registration
    // and making the syscall, which posts a notification and requests a
    // deferred context switch through PendSV. Consequently, after that
    // interrupt returns, the scheduler will be entered, see that there has
    // been a notification and request to wake up our current task (which, mind
    // you, hasn't gone to sleep yet), process it and resume the execution of
    // that task. And thus, the WAIT syscall will be reached without the
    // knowledge that the task is waiting for the notification since it has
    // already been "woken up". This could be solved by establishing a critical
    // section over the registration and the syscall, but the key here is the
    // scheduler preempting `task_wait`, so we just have to move this whole
    // logic into the kernel (as it can't preempt itself).
    __atomic_or_fetch(&notification->waiters, BIT(task->id), __ATOMIC_RELAXED);
  }
  state->ready_queue_head = task_queue_remove(state->ready_queue_head, task);
  state->sleeping_queue_head = task_queue_insert(state->sleeping_queue_head, task);
  // Re-checking all timers isn't required here, we only need to decrease the
  // closest deadline if necessary.
  task->wait_deadline = deadline;
  if (deadline < state->next_closest_deadline) {
    state->next_closest_deadline = deadline;
    // TODO: hwtimer_set_alarm((u32)deadline);
  }
}

// NOTE: This function will be executed very frequently (at least on every VGA
// scanline), performance is CRITICAL here!
static struct Task* task_scheduler(void) {
  struct TaskSchedulerState* state = &scheduler_state;

  struct Task* task;
  while (true) {
    TasksMask wake_up_mask = __atomic_exchange_n(&state->woken_up_tasks_mask, 0, __ATOMIC_RELAXED);
    Systime now = systime_now();
    if (likely(now >= state->next_closest_deadline)) {
      wake_up_mask |= process_task_timers(now);
    }
    if (likely(wake_up_mask != 0)) {
      wake_up_tasks(wake_up_mask);
    }

    // Anyway, now that the ready queue has been constructed, scheduling is
    // just a matter of pulling the next task and rotating the circular queue.
    task = state->ready_queue_head;
    if (likely(task != NULL)) {
      state->ready_queue_head = task->next;
      break;
    } else {
      // No task is ready, only an interrupt occuring can change the situation
      // (e.g. by waking up a task waiting for I/O), wait until then.
      // TODO: This creates a subtle race condition:
      // <https://electronics.stackexchange.com/questions/12601/best-pattern-for-wfi-wait-for-interrupt-on-cortex-arm-microcontrolers>.
      // I believe that using the sleep-on-exit mode of the CPU can solve this:
      // if the scheduler decides to sleep, it sets SCR.SLEEPONEXIT and exits
      // the interrupt. if another interrupt has preempted the scheduler in the
      // meantime and deferred a PendSV, it will fire instead of the CPU going
      // to sleep and reactive the scheduler.
      __WFI();
    }
  }

  return task;
}

void task_notify_init(struct Notification* self) {
  __atomic_store_n(&self->waiters, 0, __ATOMIC_RELAXED);
}

TasksMask task_notify(struct Notification* notify) {
  TasksMask waiters = __atomic_exchange_n(&notify->waiters, 0, __ATOMIC_RELAXED);
  __atomic_fetch_or(&scheduler_state.woken_up_tasks_mask, waiters, __ATOMIC_RELAXED);
  return waiters;
}

void task_sleep(u32 delay_ms) {
  Systime now = systime_now();
  Systime deadline = now + systime_from_millis(delay_ms);
  // TODO: Ensure minimum sleep time?
  while (now < deadline) {
    task_wait(NULL, deadline);
    now = systime_now();
  }
}

void task_sleep_until(Systime deadline) {
  Systime now = systime_now();
  while (now < deadline) {
    task_wait(NULL, deadline);
    now = systime_now();
  }
}

static void syscall_spawn(struct Task* task) {
  struct TaskSchedulerState* state = &scheduler_state;

  // This is O(n^2), but who cares, we won't be spawning lots of tasks anyway.
  for (TaskId free_id = 0; free_id < MAX_ALIVE_TASKS && free_id != DEAD_TASK_ID; free_id++) {
    if (task_queue_find_by_id(state->ready_queue_head, free_id) != NULL) {
      continue;
    }
    if (task_queue_find_by_id(state->sleeping_queue_head, free_id) != NULL) {
      continue;
    }
    task->id = free_id;
    break;
  }
  ASSERT(task->id != DEAD_TASK_ID);

  state->ready_queue_head = task_queue_insert(state->ready_queue_head, task);
}

static void syscall_exit(void) {
  struct TaskSchedulerState* state = &scheduler_state;

  struct Task* task = current_task;
  state->ready_queue_head = task_queue_remove(state->ready_queue_head, task);
  task->id = DEAD_TASK_ID;
  // Tell the context switcher not to save any state, we won't be returning to
  // this task anyway. TODO: Or not? Is inspecting the task's registers after
  // it has died useful in any way? Perhaps we should have a reaper task?
  current_task = NULL;
}

static struct Task* kernel_entry(enum Syscall syscall_nr, usize args[4]) {
  if (__atomic_test_and_set(&scheduler_state_lock, __ATOMIC_ACQUIRE)) {
    CRASH("kernel state already locked");
  }
  switch (syscall_nr) {
    case SYSCALL_YIELD: {
      // The context switcher will be entered immediately after return, so the
      // yield syscall essentially "does nothing".
      break;
    }
    case SYSCALL_WAIT: {
      Systime deadline = ((u64)args[1] << 32) | (u64)args[0];
      struct Notification* notification = (struct Notification*)args[2];
      syscall_wait(&scheduler_state, deadline, notification);
      break;
    }
    case SYSCALL_SPAWN: syscall_spawn((struct Task*)args[0]); break;
    case SYSCALL_EXIT: syscall_exit(); break;
    default: CRASH("unknown syscall");
  }
  struct Task* next_task = task_scheduler();
  __atomic_clear(&scheduler_state_lock, __ATOMIC_RELEASE);
  return next_task;
}

// This function is the bottom-most frame of every task stack and exists
// largely to be a barrier for debugger's stack trace unwinder (by being
// implemented in assembly and having no hint directives whatsoever).
static CLANG_ATTRIBUTE(nouwtable) __NO_RETURN __NAKED
  void task_launchpad(__UNUSED void* user_data, __UNUSED TaskFn* func) {
  __ASM volatile( //
  // Clang doesn't support the ARM unwinding directives in naked functions.
  // Well, at least it has an attribute for marking functions as CANTUNWIND.
#if !defined(__clang__) && ARM_UNWIND_DIRECTIVES
    // Generate a "refuse to unwind" opcode instead of using the ".cantunwind"
    // directive to make the runtime stack unwinder stop at this function, but
    // not ignore it, and include it in the backtrace.
    ".unwind_raw 0, 0x80, 0x00\n\t"
#endif
    // Call the function passed in the second parameter with the user data as
    // its first parameter which is already in r0.
    "blx r1\n\t"
    // Call task_exit() if the task function has returned.
    "bl %0\n\t"
    // The task has been terminated, a return here must cause a fault.
    "udf #0" :: //
    "i"(&task_exit)
  );
}

void task_spawn(struct Task* task, const struct TaskParams* params) {
  ASSERT(task != NULL);
  ASSERT(params != NULL);

  ASSERT(params->func != NULL);
  ASSERT(params->stack_start != NULL);
  ASSERT(params->stack_size != 0);
  ASSERT((usize)params->stack_start % 8 == 0);
  ASSERT((usize)params->stack_size % 8 == 0);

  struct TaskStackedContext {
    u32 control, r4, r5, r6, r7, r8, r9, r10, r11, exc_return;
    u32 r0, r1, r2, r3, r12, lr, pc, xpsr;
  };

  u32 min_stack_size = sizeof(struct TaskStackedContext);
#if __FPU_USED
  // 32 floating-point registers plus the FPSCR and a word for alignment:
  min_stack_size += sizeof(float) * 32 + sizeof(u32) + sizeof(u32);
#endif
  ASSERT(params->stack_size >= min_stack_size);

  task->stack_ptr = params->stack_start + params->stack_size;
  task->id = DEAD_TASK_ID;
  task->priority = 0;
  task->prev = task->next = NULL;
  task->wait_deadline = NO_DEADLINE;
  task->stack_start = params->stack_start;
  task->stack_size = params->stack_size;

  fast_memset_u32((u32*)task->stack_start, STACK_CHECK_VALUE, task->stack_size / sizeof(u32));

  // Manufacture an initial context needed for cranking the task.
  task->stack_ptr -= sizeof(struct TaskStackedContext);
  struct TaskStackedContext* ctx = (void*)task->stack_ptr;

  // Pass the arguments for `task_launchpad` in r0 and r1.
  ctx->r0 = (usize)params->user_data;
  ctx->r1 = (usize)params->func;
  // The other general-purpose registers are set to dummy values.
  ctx->r2 = 2, ctx->r3 = 3, ctx->r12 = 12;
  // Write garbage into LR so that returning causes an explosion.
  ctx->lr = 0;
  // Execution will start here:
  ctx->pc = (usize)task_launchpad;
  // The EPSR.T (Thumb State) bit needs to be set depending on the last bit of
  // PC (which will always be 1 on Cortex-M CPUs since they only executed Thumb
  // code), which matches the CPU reset behavior:
  // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Reset-behavior>
  ctx->xpsr = (ctx->pc & 1) << xPSR_T_Pos;
  // The bits of the CONTROL registers are largely irrelevant and must be
  // reset, only CONTROL.nPRIV matters.
  ctx->control = (0 << CONTROL_nPRIV_Pos) | (0 << CONTROL_SPSEL_Pos) | (0 << CONTROL_FPCA_Pos);
  // The other general-purpose registers are set to dummy values.
  ctx->r4 = 4, ctx->r5 = 5, ctx->r6 = 6, ctx->r7 = 7;
  ctx->r8 = 8, ctx->r9 = 9, ctx->r10 = 10, ctx->r11 = 11;
  // The initial EXC_RETURN value is such that the return happens into Thread
  // mode, using PSP as the stack and the floating-point context is inactive,
  // i.e. no FPU instructions have been issued and no FP registers have been
  // stacked.
  ctx->exc_return = EXC_RETURN_THREAD_PSP;

  syscall_1(SYSCALL_SPAWN, (usize)task);
}

// Approximately measures the highest stack usage of a task.
usize task_stack_high_watermark(struct Task* task) {
  u32* ptr = (u32*)task->stack_start;
  u32* end = (u32*)(task->stack_start + task->stack_size);
  while (ptr < end && *ptr == STACK_CHECK_VALUE) ptr++;
  return (usize)end - (usize)ptr;
}

// Takes the pointer to the next scheduled task and performs a context switch
// if necessary. Must be invoked at the end of an interrupt handler, with the
// LR set to the appropriate EXC_RETURN value, all callee-saved registers
// (r4-r11) unchanged from their values in the task we are switching out of,
// and with no frames on the stack in between.
static __NAKED void context_switch(__UNUSED struct Task* next_task) {
  // A quick note before we start off: before entering the interrupt, the
  // hardware has already stacked a bunch of CPU registers, switched into
  // privileged mode and set the stack pointer to MSP (Main Stack Pointer),
  // which will all be undone after leaving the exception. All of this usually
  // takes ~10 cycles for entering and another ~10 for leaving, though the
  // actual numbers will vary. In any case, at the start the registers r0-r3
  // can be freely clobbered by us since the hardware will restore them.
  // <https://developer.arm.com/documentation/ddi0439/b/Programmers-Model/Exceptions/Exception-handling>

  __ASM volatile( //
    // The following two instructions load the address of `current_task`. The
    // instruction `ldr r3, =%0` (load from the constant pool) is not used
    // because it takes 2-5 cycles (see ARM 100166 section 3.3.3 "Load/store
    // timings") at best, usually 5, and a MOVW+MOVT combination predictably
    // takes just 2 cycles.
    "movw r3, #:lower16:%[current_task]\n\t"
    "movt r3, #:upper16:%[current_task]\n\t"
    // Load the pointer to the previous task and put it into r1. The next task
    // is already in the first argument register, r0.
    "ldr r1, [r3]\n\t"
    // We can take a shortcut if the next task and the previous one are the
    // same, skipping over the context switch:
    "cmp r0, r1\n\t"
    "beq 2f\n\t"
    // Store the next task pointer in `current_task`.
    "str r0, [r3]\n\t"

    // NOTE: Here is where the actual context switching begins. It is
    // implemented with some assistance from hardware: after all, since a part
    // of context has already been pushed it on the stack upon exception entry,
    // the stack pointer may simply be edited to point to a different block of
    // saved context, which may, for instance, reside on the stack of another
    // task. Hence, returning from this handler will jump into an entirely
    // different task! The problem is, the hardware stacks only half the normal
    // and FPU registers, and we must stack the rest ourselves.

    // r0 - next_task, r1 - prev_task, r2 - task_stack_ptr.

    // Saving the state must be skipped if the previous task was NULL.
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
    // are reset upon entering the interrupt, and are restored from the bits of
    // EXC_RETURN on exit.
    "mrs r3, control\n\t"
#if __FPU_USED
    // The bit 4 of EXC_RETURN determines whether the stacked state includes
    // the FPU registers, and thus whether the task has issued any FPU
    // instructions so far.
    "tst lr, #16\n\t"
    "it eq\n\t"
    // Store the FP registers only if the floating-point context has actually
    // been used (if not, just a single cycle will be wasted stepping over this
    // instruction). They are pushed first so that the core registers lie on
    // the top of the stack. Note that if lazy FP state preservation is enabled
    // (which it always is, I guess), issuing this instruction will also
    // implicitly cause the registers s0-s15 to be stored as well in the space
    // allocated in the stack frame for the exception - that's 32 memory
    // transactions caused by a single instruction!
    "vstmdbeq r2!, {s16-s31}\n\t"
#endif
    // Push the rest of the core registers, plus the value of CONTROL which is
    // currently in r3, plus the EXC_RETURN in LR, so that we can resume the
    // execution of the task. The bits of EXC_RETURN can be used to determine
    // if any FP context has been pushed.
    "stmdb r2!, {r3, r4-r11, lr}\n\t"
    // Store the adjusted stack pointer in the previous task struct.
    "str r2, [r1, %[task_stack_ptr]]\n\t"

    "1:\n\t"
    // Load the stack pointer of the next task.
    "ldr r2, [r0, %[task_stack_ptr]]\n\t"
    // Pop the other task's core registers from its stack, its value of the
    // CONTROL register into r3 and its EXC_RETURN value into LR.
    "ldmia r2!, {r3, r4-r11, lr}\n\t"
#if __FPU_USED
    // Reload the next task's floating-point context. See the note about
    // preserving it above.
    "tst lr, #16\n\t"
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
    // Finally, return from the interrupt handler, jumping into the next task.
    "bx lr\n\t" :: //

      [current_task] "i"(&current_task),
    [task_stack_ptr] "J"(offsetof(struct Task, stack_ptr))
  );
}

// The common code for entering the kernel from the PendSV and SVCall
// exceptions, and making a context switch afterwards. Must be tail-called from
// the ISR functions.
static __NAKED void
handle_kernel_irq(__UNUSED enum Syscall syscall_nr, __UNUSED usize syscall_args[4]) {
  __ASM volatile( //
    // We need to back up the LR register (since we will need its EXC_RETURN
    // value for returning from the exception) before calling the kernel_entry
    // function, plus another register to keep the stack aligned.
    "push {r4, lr}\n\t"
#if CFI_DIRECTIVES
    // The CFI directives make the assembler put a certain section into the
    // binary that informs the debugger on how to unwind the stack and recover
    // local variables in the caller frames. They don't generate any machine
    // code and were added purely for the convenience of debugging. We use them
    // to mark the registers that have been pushed on the stack:
    ".cfi_adjust_cfa_offset 8\n\t"
    ".cfi_rel_offset r4, 0\n\t"
    ".cfi_rel_offset lr, 4\n\t"
#endif
#if ARM_UNWIND_DIRECTIVES && !defined(__clang__)
    // The ARM unwinding directives serve a similar purpose to the CFI ones,
    // but are used for throwing C++ exceptions and generating backtraces at
    // runtime.
    ".save {r4, lr}\n\t"
#endif
    // The `kernel_entry` may now be called. The arguments are already in place
    // in the appropriate registers since this function has the same signature.
    // The kernel will call the scheduler and return the next task in r0.
    "bl %[kernel_entry]\n\t"
    // Pop the registers which were pushed above.
    "pop {r4, lr}\n\t"
#if CFI_DIRECTIVES
    // Here come the CFI directives once again, this time to inform the
    // debugger that the values of the backed up registers have been restored
    // and are now the same that they were at the beginning of the function.
    ".cfi_adjust_cfa_offset -8\n\t"
    ".cfi_restore r4\n\t"
    ".cfi_restore lr\n\t"
#endif
    // Tail-call the context switcher to actually switch the tasks. It is
    // separated into its own function because of the unwinding directives: the
    // ARM specific ones (i.e. `.save`) applies to the entire function and
    // cannot be restricted to limited sections of code.
    "b %[context_switch]" :: //
      [kernel_entry] "i"(&kernel_entry),
    [context_switch] "i"(&context_switch)
  );
}

__NAKED void PendSV_Handler(void) {
  // The PendSV handler effectively just emulates the YIELD syscall.
  __ASM volatile(     //
    "movs r0, %0\n\t" // syscall_nr
    "movs r1, #0\n\t" // syscall_args
    "b %1" ::         //
    "n"(SYSCALL_YIELD),
    "i"(&handle_kernel_irq)
  );
}

__NAKED void SVC_Handler(void) {
  // NOTE: On the syscall ABI: the low caller-saved registers r0-r3 are used
  // for the syscall parameters, r0 is used for the return value and the
  // syscall number is embedded into the immediate parameter of the `SVC`
  // instruction. There is, however, an important consideration relating to the
  // choice of caller-saved registers due to how interrupts on Cortex-M
  // interact with the ARM calling convention.
  //
  // Suppose an SVC instruction is issued by the application, and during the
  // stacking process a higher-priority (in other words: any) interrupt
  // arrives. The CPU will handle it first, and tail-chain the SVCall exception
  // handler without pushing any more context on the stack, which is perfectly
  // safe as long as everyone respects the calling convention. However, ISRs
  // are free to contaminate the caller-saved registers (r0-r3 and r12) since
  // all of them are on the stack and will be restored by the hardware.
  // However, for us this means that if we want to use r0-r3 for passing
  // syscall parameters, an early-arriving interrupt could clobber them all.
  // The solution to that, of course, is simply loading their values from the
  // stack in the ISR. In contrast, since the software is responsible for
  // restoring r4-r11 at the end of every function, even if this exception gets
  // tail-chained, the register values will be those at the point of the `SVC`
  // instruction.
  //
  // However, the choice of r0-r3 has its upside: because these registers
  // reside on the stack, we can just get a pointer to the start of the
  // interrupt frame and pass it around like an array, whose first 4 elements
  // will be: r0, r1, r2 and r3! If any syscall then needs to check any
  // parameters or write a return value, it can simply use this array - this
  // greatly simplifies their implementations in C and the necessary
  // supplementary assembly code.
  __ASM volatile( //
    // The bit 2 of LR specifies which stack was in use prior to entering the
    // interrupt. Use it to figure out whether the caller was using MSP or PSP.
    "tst lr, #4\n\t"
    "ite eq\n\t"
    "mrseq r1, msp\n\t" // if (LR & 4) == 0
    "mrsne r1, psp\n\t" // if (LR & 4) != 0
    // Load the PC from the stacked context.
    "ldr r0, [r1, #24]\n\t"
    // Load a halfword from memory with the instruction's immediate, which is
    // the syscall number. Put it into r0 - the first argument register.
    "ldrb r0, [r0, #-2]\n\t"
    // Enter the kernel code. The argument values for the syscall number and
    // the pointer to the stacked registers are in r0 and r1 respectively.
    "b %0" :: //
    "i"(&handle_kernel_irq)
  );
}
