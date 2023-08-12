// The critical error handler and the "screen of death" module. The general
// point here is to be able to salvage the system from critical states such as
// running out of RAM or a stack overflow, or errors on task context switching
// etc (though the situation may not necessarily be that dire, e.g. illegal
// memory accesses and assertion failures can happen because of logic errors in
// the code under normal conditions too), recovering it enough to enter a state
// where we can display an error message to the user and try to do an autopsy
// of the system (within the bounds of what is possible to determine directly
// on the MCU). Consequently, complicated drivers and subsystems, such as the
// GUI and task scheduler (when they will be implemented, of course) which are
// themselves prone to bugs shouldn't be used here, as the system objects might
// be in an invalid state during the crash, rendering those subsystems
// unusable. In other words, only small and trusted components should be used
// within the crash handler, which won't allocate memory dynamically and so on.
//
// Anyway, all of this is REALLY hacky and not portable.
//
// Some useful links:
// <https://interrupt.memfault.com/blog/cortex-m-hardfault-debug>
// <https://interrupt.memfault.com/blog/cortex-m-rtos-context-switching>
// <https://interrupt.memfault.com/blog/arm-cortex-m-exceptions-and-nvic>
// <https://wiki.segger.com/Cortex-M_Fault>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-entry-behavior>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-return-behavior>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/Configurable-Fault-Status-Register--CFSR>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/HardFault-Status-Register--HFSR>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/MemManage-Fault-Address-Register--MMFAR>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/BusFault-Address-Register--BFAR>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/Auxiliary-Fault-Status-Register--AFSR>
// <https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/Debug-register-support-in-the-SCS/Debug-Fault-Status-Register--DFSR>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Fault-behavior>
// <https://developer.arm.com/documentation/dui0646/c/Cortex-M7-Peripherals/System-control-block/Configurable-Fault-Status-Register>
// <https://developer.arm.com/documentation/dui0646/c/The-Cortex-M7-Processor/Fault-handling/Fault-types>
// <https://www.keil.com/appnotes/files/apnt209.pdf>
// <https://interrupt.memfault.com/blog/asserts-in-embedded-systems>
// <http://robitzki.de/blog/Assert_Hash>
// <https://barrgroup.com/Embedded-Systems/How-To/Define-Assert-Macro>
// <https://developer.arm.com/documentation/dai0298/latest/>
//
// HardFault handlers from other projects:
// <https://github.com/cvra/arm-cortex-tools/blob/1481ac8d5570a4dc2e0d6033abe3ec758f47a8b6/fault.c>
// <https://github.com/MarlinFirmware/Marlin/blob/441416728cd7f0e9b6ebf94f895d1d27fe59d25a/Marlin/src/HAL/shared/cpu_exception/exception_arm.cpp>
// <https://github.com/micropython/micropython/blob/ed962f1f233eb74edf2cee83dc488d3cac5e02ee/ports/stm32/stm32_it.c#L134-L230>
// <https://github.com/zephyrproject-rtos/zephyr/blob/f32fae07f299aa3264ec736517baceacc5398e43/arch/arm/core/aarch32/cortex_m/fault.c>
// <https://github.com/ARMmbed/mbed-os/blob/f347b89d0df6d99b5e3f0e48cf0d0e8d34a42e45/platform/source/TARGET_CORTEX_M/mbed_fault_handler.c>

// TODO: A CrashReporter API for formatting the error messages after the crash
// and outputting more contextual information.

// TODO: Backtrace and/or stack unwinder. Some ideas:
// <https://github.com/armink/CmBacktrace/blob/8d07e7ba078fbd57af89286fc217157bc9a6d2ac/cm_backtrace/cm_backtrace.c>
// <https://github.com/MarlinFirmware/Marlin/tree/441416728cd7f0e9b6ebf94f895d1d27fe59d25a/Marlin/src/HAL/shared/backtrace>
// <https://github.com/red-rocket-computing/backtrace>
// <http://yosefk.com/blog/getting-the-call-stack-without-a-frame-pointer.html>

// TODO: Add more CFI directives to the inline assembly because the debugger
// probably won't be able to unwind and restore local variables correctly due
// to us carelessly messing with the registers. More information:
// <https://developer.arm.com/documentation/100748/0618/Assembling-Assembly-Code/How-to-get-a-backtrace-through-assembler-functions>
// <https://www.imperialviolet.org/2017/01/18/cfi.html>
// <https://sourceware.org/binutils/docs/as/CFI-directives.html>

#include "stmes/kernel/crash.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/mpu.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <printf.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_gpio.h>

// The memory for storing the crash context information is allocated statically
// to ensure that we always have it on hand, even if we run out of heap or stack.
static struct CrashContext {

  enum __packed CrashType {
    CRASH_TYPE_NONE = 0,
    CRASH_TYPE_ASSERTION = 1,
    CRASH_TYPE_HARDFAULT = 2,
  } type;

  bool cpu_registers_collected;

  // TODO: Perhaps saving the crash timestamp may also be worthwhile

  // NOTE: Inline assembly relies on the exact layout of this struct!!!
  // Although perhaps the registers could be rearranged to reduce the number of
  // instructions needed for storing them in the HardFault handler.
  struct CrashCpuRegisters {
    u32 r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, sp, lr, pc, xpsr;
  } cpu_registers;

  union CrashPayload {
    // The assertion system is largely based on these recommendations:
    // <https://interrupt.memfault.com/blog/asserts-in-embedded-systems>
    struct CrashAssertion {
      // The (approximate) value of the program counter at the point of crash.
      u32 address;
      // Can be NULL.
      const char* message;
      // To help quickly identify the crash location (without breaking out
      // addr2line) the filename and line number of the file with the failing
      // assertion is (optionally) stored. Despite not being as convenient as
      // storing the full conditional expression, it is more efficient because
      // the file path strings can be deduplicated and reused.
      const char* src_file;
      u32 src_line;
    } assertion;

    struct CrashHardfault {
      // Maybe saving all of these registers is a bit redundant.
      u32 exc_return, cfsr, hfsr, dfsr, mmfar, bfar, afsr;
      enum MpuFaultDiagnosis mpu_diagnosis;
    } hardfault;
  } payload;

} crash_context;

// This function should be called as close as possible to the error location,
// preferably right after it has been detected, so that the registers haven't
// been clobbered yet by the formatting of the error message and such. It also
// had to be written in hand-written assembly so as to be really careful and
// not overwrite any of the registers. Unfortunately, the original value of the
// link register (LR) at the call site can't be preserved because it has to be
// changed when branching to this function to be able to return, though this
// stored LR will at least reflect the PC at the caller's location.
__NAKED void crash_collect_registers(void) {
  __ASM volatile(
    // TODO: movw/movt
    "push {lr}\n\t"           // Save the LR because we are going to use it to store the address
    "ldr lr, =%0\n\t"         // of cpu_registers. The calling convention permits the use of LR as
                              // a general-purpose register, which we want because we are now left
    "stmia lr!, {r0-r12}\n\t" // with all the general-purpose registers unspoiled, which we can
                              // store as a contiguous range in one go.
    "pop {r1}\n\t"            // Load the real value of LR into r1, will come in handy later.
    "mov r0, sp\n\t"          // The stack pointer is now also back to its original value.
    "mov r2, pc\n\t"          // The special registers such as PC and SP can't be passed as the
                              // arguments to the `stmia` instruction, let alone the xPSR which has
    "mrs r3, xpsr\n\t"        // to be loaded with a special instruction first.
    "stmia lr!, {r0-r3}\n\t"  // Instead we move them all into r0-r3 (scratch registers in the
                              // calling convention) in the order they appear in the cpu_registers
                              // struct (whose address is at this point still in the LR), so that
                              // we can also store them in one contiguous slice.
    "ldr r0, =%1\n\t"         // Load the address of cpu_registers_collected.
    "movs r2, #1\n\t"         // Set the flag now that we are done saving the registers.
    "strb r2, [r0]\n\t"       // cpu_registers_collected = 1
    "mov lr, r1\n\t"          // Strictly speaking we aren't obliged to restore the original value
    "bx lr" ::                // of LR before returning, but let's be nice.
    "i"(&crash_context.cpu_registers),
    "i"(&crash_context.cpu_registers_collected)
  );
}

// This function can be called at any place at any time, but remember to also
// call crash_collect_registers beforehand.
// TODO: Support assertion crashing from within interrupts.
__NO_RETURN void crash(const char* message, const char* src_file, u32 src_line) {
  __disable_irq();
  crash_context.type = CRASH_TYPE_ASSERTION;
  struct CrashAssertion* ctx = &crash_context.payload.assertion;
  if (crash_context.cpu_registers_collected) {
    // See the comments to crash_collect_registers for why we are using the LR.
    ctx->address = crash_context.cpu_registers.lr;
  } else {
    // In case I forgot to collect the registers, try to figure out something
    // by looking at the return address (this function is called with `bl`
    // despite being marked no-return). These GCC builtins effectively just
    // read the LR register, but are portable (not that we need that).
    ctx->address = (u32)__builtin_extract_return_addr(__builtin_return_address(0));
  }
  ctx->message = message, ctx->src_file = src_file, ctx->src_line = src_line;
  enter_crash_screen();
}

static u32 hardfault_handler_impl(u32 exc_return, u32 msp, u32 psp);

// A wrapper around the real HardFault handler which saves the secondary
// registers (only the general-purpose ones though) which aren't saved by the
// hardware into the cpu_registers struct and arranges the stack pointer and
// EXC_RETURN stuff. We need a bit of assembly to do this, however, since C is
// kind of too high-level for that. Fortunately C functions can be fully
// written in assembly by marking them as `naked` - this disables the
// generation of a standard function prologue and epilogue by the compiler,
// putting us in charge of maintaining the calling convention and such.
// <https://stackoverflow.com/questions/33175120/access-c-non-pod-class-data-from-naked-asm-function>
// <https://stackoverflow.com/questions/75699618/cortex-m4-svc-code-appears-to-always-pass-in-255-for-the-svc-number>
static __NAKED void hardfault_handler_entry(void) {
  __ASM volatile(
    // TODO: movw/movt
    "cpsid i\n\t"            // __disable_irq()
    "push {lr}\n\t"          // I don't think we should really be ascetic with the stack because
                             // the MPU is turned off in fault handlers, so we won't get a stack
    "ldr lr, =%1\n\t"        // overflow error here (at least yet). Anyway, back up the LR and use
                             // it to store the address of cpu_registers.
    "stmia lr, {r0-r12}\n\t" // Save all general-purpose registers (as you probably guessed by now,
                             // so far this function closely mirrors crash_collect_registers).
    "pop {lr}\n\t"           // Reload the backed up LR, which contains the EXC_RETURN value, and
                             // also restore the active stack pointer to its original address.
    "mrs r1, msp\n\t"        // Put the stack pointers into registers for arguments. We must do
    "mrs r2, psp\n\t"        // this now because the `push` in the function prologue of the actual
                             // handler written in C will shift them.
    "mov r0, lr\n\t"         // Finally, place LR into the first argument register...
#ifdef __PLATFORMIO_BUILD_DEBUG__
    ".cfi_register lr,r0\n\t" // (this directive informs the debugger that LR's value is in r0)
#endif                        //
    "bl %0\n\t"               // ...and we call the actual handler. We then use its return value
    "bx r0\n\t" ::            // as EXC_RETURN, letting the handler alter it.
    "i"(&hardfault_handler_impl),
    "i"(&crash_context.cpu_registers)
  );
}

// NOTE: This is specific to our ARM variant! Other architectures may have more
// flags in the EXC_RETURN value, see the list of available ones here:
// <https://github.com/HotRays/zephyr/blob/a47cbffe1e3db496ed7667c84f0151666c5dfca2/arch/arm/core/fault.c#L46-L92>
__STATIC_INLINE bool validate_exc_return(u32 ret) {
  ret = ~ret; // Comparing negated values produces shorter instructions.
  if (ret == ~EXC_RETURN_HANDLER) return true;
  if (ret == ~EXC_RETURN_THREAD_MSP) return true;
  if (ret == ~EXC_RETURN_THREAD_PSP) return true;
#if __FPU_PRESENT == 1
  if (ret == ~EXC_RETURN_HANDLER_FPU) return true;
  if (ret == ~EXC_RETURN_THREAD_MSP_FPU) return true;
  if (ret == ~EXC_RETURN_THREAD_PSP_FPU) return true;
#endif
  return false;
}

// Okay, here's where the interesting part begins. In short, here's the deal:
// before entering an interrupt handler the CPU pushes some of the registers on
// the stack (r0-r3, r12, lr, pc, xPSR, and, if the FPU is enabled, s0-s15 and
// FPSCR, though the FP state stacking is a bit of a complicated topic), sets
// the LR to a special marker value called EXC_RETURN, then jumps to the
// corresponding handler, lets it run, and after it returns (by jumping to
// EXC_RETURN, which really just means a normal `bx lr` or such), the CPU pops
// (unstacks) the registers off the stack and proceeds to do what it was doing
// before the interrupt has occurred. A notable consequence of this is that the
// CPU hardware implements the ARM calling convention (AAPCS), meaning that the
// ISRs can be written as normal C functions (they themselves will back up and
// restore the other registers, as mandated by the convention). What's more
// important is that we can examine the pre-interrupt state ourselves by
// looking at the stack, and by editing it affect what happens after exiting
// the interrupt.
//
// More information on what happens on interrupt entry/return:
// <https://interrupt.memfault.com/blog/cortex-m-hardfault-debug#registers-prior-to-exception>
// <https://interrupt.memfault.com/blog/cortex-m-rtos-context-switching#context-state-stacking>
//
// Sections 2.3.2 "Exception types", 2.3.7 "Exception entry and return", 2.4
// "Fault handling", 2.4.1 "Fault types", 2.4.2 "Fault escalation and hard
// faults", 4.4.10-4.4.17 of PM0214 from STM give a more detailed explanation
// and an overview of all related registers. AN209 from Keil gives all the
// relevant excerpts with the exact same information in a single document.
//
// The exact specifications for ARMv7m can be found in sections B1.5 (in
// particular B1.5.6 "Exception entry behavior", B1.5.7 "Stack alignment on
// exception entry" and B1.5.8 "Exception return behavior"), B3.2.15-B3.2.19
// and C1.6.1 of DDI0403E from ARM.
//
// NOTE: Is called from inline assembly, which relies on the order of arguments!
// TODO: Currently this function might not be able to handle a stack overflow
// situation because the crash screen functions still require a few bytes of
// stack for pushing the local registers. Perhaps we can exchange the stacks,
// or extend the limit on the current stack?
static u32 hardfault_handler_impl(u32 exc_return, u32 msp, u32 psp) {
  // Returning from this function will jump back to the code which generated
  // the fault, which is desirable in some cases, but may not always be
  // possible (since e.g. BusFaults may be imprecise). Also, I am pretty sure
  // the PC will point to the instruction that caused the fault, which may be
  // useful for retrying, but otherwise will cause an infinite loop.

  crash_context.type = CRASH_TYPE_HARDFAULT;
  struct CrashHardfault* ctx = &crash_context.payload.hardfault;

  // Bit 2 of EXC_RETURN determines which stack pointer was in use prior to
  // entering the exception handler, which, consequently, contains the frame
  // with the stacked registers.
  u32 active_sp = (exc_return & BIT(2)) == 0 ? msp : psp;

  // See PM0214 section 2.3.7. The hardware ensures correct stack alignment, so
  // this struct doesn't need to have the __PACKED attribute (as suggested by
  // the articles linked above).
  struct ExceptionStackedRegisters {
    u32 r0, r1, r2, r3, r12, lr, pc, xpsr;
  }* stacked_regs = (void*)active_sp;

  struct CrashCpuRegisters* regs = &crash_context.cpu_registers;
  regs->r0 = stacked_regs->r0, regs->r1 = stacked_regs->r1, regs->r2 = stacked_regs->r2,
  regs->r3 = stacked_regs->r3, regs->r12 = stacked_regs->r12, regs->lr = stacked_regs->lr,
  regs->pc = stacked_regs->pc, regs->xpsr = stacked_regs->xpsr, regs->sp = active_sp;
  // The wrapper function above has collected all other registers (r4-r11).
  crash_context.cpu_registers_collected = 1;

  // The memory addresses should be read first, see the recommendations in
  // PM0214 section 4.4.18.
  ctx->mmfar = SCB->MMFAR, ctx->bfar = SCB->BFAR;

  // Record the fault status registers.
  ctx->cfsr = SCB->CFSR, ctx->hfsr = SCB->HFSR, ctx->dfsr = SCB->DFSR, ctx->afsr = SCB->AFSR;

  ctx->exc_return = exc_return;

  ctx->mpu_diagnosis = MPU_FAULT_UNKNOWN;
  if (ctx->cfsr & SCB_CFSR_MEMFAULTSR_Msk) {
    u32 ipsr = regs->xpsr & 0x1FF;
    // This won't tell apart unprivileged load instructions (LDR*T)... Not a
    // big deal though.
    bool privileged = ipsr != 0 || (__get_CONTROL() & CONTROL_nPRIV_Msk) == 0;
    bool instruction_access = ctx->cfsr & SCB_CFSR_IACCVIOL_Msk;
    if (instruction_access) {
      ctx->mpu_diagnosis = mpu_diagnose_memfault(regs->pc, instruction_access, privileged);
    } else if (ctx->cfsr & SCB_CFSR_MMARVALID_Msk) {
      ctx->mpu_diagnosis = mpu_diagnose_memfault(ctx->mmfar, instruction_access, privileged);
    }
  }

  // The debug enabled flag doesn't get reset when the CPU is reset after the
  // debugger is disconnected, so this is not a good idea.
  // if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
  //   __BKPT();
  // }

  // Now, prepare for recovery from the fault!
  // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-return-behavior>
  // <https://interrupt.memfault.com/blog/cortex-m-hardfault-debug#recovering-from-a-fault>

  // Clear the fault status registers first. These registers are
  // write-to-clear, see DDI0403E section B3.2.15. I guess normally the handler
  // would reset the faults it has explicitly handled, but in our case all we
  // are doing is reporting them all.
  SCB->CFSR = SCB->CFSR, SCB->HFSR = SCB->HFSR, SCB->DFSR = SCB->DFSR, SCB->AFSR = SCB->AFSR;

  u32 stacked_pc = stacked_regs->pc;
  // Next, we abuse the fact that the return address is stored in the stack
  // frame, and we can simply overwrite it to point to our own function, which
  // the hardware will happily jump to after return from the fault interrupt.
  stacked_regs->pc = (u32)&enter_crash_screen;
  // We have to prepare other registers as well, most importantly the xPSR.
  // Among other things, it tracks the execution state of interrupted LDM/STM
  // and IT instructions, which needs to be reset, or else we risk getting fun
  // bugs like here: <https://stackoverflow.com/q/9526077/12005228>. It also
  // contains the previously executed exception if the fault has occurred
  // inside another interrupt, which has to be reset too. Only the "T" bit
  // (Thumb state bit) needs to be set, which matches what happens on reset:
  // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Reset-behavior>
  // See also PM0214 section 2.1.3, "Execution program status register" and
  // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/Registers/The-special-purpose-program-status-registers--xPSR>
  stacked_regs->xpsr = (stacked_regs->pc & 1) << xPSR_T_Pos;
  // Lastly, since the function executed after recovery isn't supposed to jump
  // back into the offending code, the memfault article recommends setting LR
  // to some invalid address (0xDEADBEEF), returning to which will immediately
  // cause a fault. However, since I use the __NO_RETURN attribute to guarantee
  // that the function doesn't return, I can kind of stitch the frames of the
  // crash screen function and the function which caused the fault, which
  // creates a nice backtrace in the debugger.
  stacked_regs->lr = stacked_pc;

  // This handles the really bad INVPC faults when the EXC_RETURN value is
  // completely off. Thing is, for this kind of faults the hardware doesn't
  // set LR to a fresh EXC_RETURN value, leaving the wrong one in the register
  // (so that the fault handler can see it), which makes it impossible to
  // return from the fault handler normally.
  if (!validate_exc_return(exc_return)) {
    // In that case, since we don't have any way of determining the initial
    // exception return configuration, we will have to improvise. The value
    // below means "return to handler mode" (since we **definitely** got the
    // INVPC fault from another ISR), "the stack is MSP and doesn't contain
    // floating-point state".
    exc_return = EXC_RETURN_HANDLER;
  }

  // In case we have received the fault from another interrupt handler (in
  // which case the bit 3 of EXC_RETURN will be unset), some care needs to be
  // taken to exit into the crash screen instead of the handler.
  if ((exc_return & BIT(3)) == 0) {
    // First, set the 3rd bit to indicate that we want to return into the
    // Thread mode. The bits relating to the stacked state are left as-is.
    exc_return |= BIT(3);
    // Secondly, the hardware performs a bunch of checks on exception return to
    // ensure the processor will be put into a valid execution state (described
    // in section B1.5.8 of DDI0403E under the heading "Integrity checks on
    // exception return"). One of these checks is that if there was another
    // exception active besides the current one, the return must happen into
    // the Handler mode. Thankfully, this exact check may be disabled. I don't
    // particularly like this solution, however, I am not aware of a way to
    // reliably clear the active status of every exception.
    SCB->CCR |= SCB_CCR_NONBASETHRDENA_Msk;
  }

  // HACK: A temporary solution to recovery from stack overflows: let's just
  // ignore the problem.
  mpu_disable_region(MPU_REGION_STACK_BARRIER);

  __set_CONTROL(__get_CONTROL() & ~CONTROL_nPRIV_Msk);

  // Data synchronization barrier: ensure that all memory writes have been
  // completed before we give control back to the hardware.
  __DSB();

  // And finally, return! And, hopefully, jump into the crash screen.
  return exc_return;
}

__ALIAS("hardfault_handler_entry") void HardFault_Handler(void);
__ALIAS("hardfault_handler_entry") void MemManage_Handler(void);
__ALIAS("hardfault_handler_entry") void BusFault_Handler(void);
__ALIAS("hardfault_handler_entry") void UsageFault_Handler(void);

static void print_number(u32 value, u32 base) {
  base = MAX(base, 1);
  base = MIN(base, 36);

  usize digits = 1;
  u32 factor = 1;
  for (u32 tmp = value / base; tmp != 0; tmp /= base) {
    digits += 1, factor *= base;
  }

  // The number is printed backwards to avoid using stack space for a string
  // that we would then have to reverse.
  for (usize i = 0; i < digits; i++) {
    u32 digit = value / factor % base;
    console_putchar(digit < 10 ? digit + '0' : digit - 10 + 'A');
    factor /= base;
  }
}

static void print_address(u32 value) {
  u8 text_color = console_get_current_color();
  // Make the foreground color dimmer (for the leading zeroes).
  console_set_color(text_color | 0x8);
  bool leading_zeroes = true;
  for (usize i = 0, digits_count = 8; i < digits_count; i++) {
    u32 digit = (value >> 28) & 0xF;
    if (leading_zeroes && (digit != 0 || i + 1 == digits_count)) {
      leading_zeroes = false;
      // Once we reached the end leading zeroes or the final zero, switch back
      // to the normal (bright) color.
      console_set_color(text_color);
    }
    console_putchar(digit < 10 ? digit + '0' : digit - 10 + 'A');
    value <<= 4;
  }
}

static void print_register(const char* name, u32 value) {
  console_print(name);
  console_putchar('=');
  u8 prev_color = console_get_current_color();
  console_set_color(0x06);
  print_address(value);
  console_set_color(prev_color);
}

static void crash_screen_hardfault_title(void) {
  struct CrashHardfault* ctx = &crash_context.payload.hardfault;
  usize fault_types = 0;
  if (ctx->cfsr & (SCB_CFSR_MEMFAULTSR_Msk & ~SCB_CFSR_MMARVALID_Msk)) {
    console_print("MemFault");
    fault_types++;
  }
  if (ctx->cfsr & (SCB_CFSR_BUSFAULTSR_Msk & ~SCB_CFSR_BFARVALID_Msk)) {
    if (fault_types > 0) console_print(", ");
    console_print("BusFault");
    fault_types++;
  }
  if (ctx->cfsr & SCB_CFSR_USGFAULTSR_Msk) {
    if (fault_types > 0) console_print(", ");
    console_print("UsageFault");
    fault_types++;
  }
  if (fault_types == 0) {
    console_print("HardFault");
  } else if (ctx->hfsr & SCB_HFSR_FORCED_Msk) {
    console_print(", escalated to HardFault");
  }
}

static void crash_screen_hardfault_reason(void) {
  struct CrashHardfault* ctx = &crash_context.payload.hardfault;
  struct CrashCpuRegisters* regs = &crash_context.cpu_registers;
  u32 cfsr = ctx->cfsr, hfsr = ctx->hfsr;

  if (cfsr & SCB_CFSR_IACCVIOL_Msk) {
    console_print("IACCVIOL/Instruction access violation on "), print_address(regs->pc);
    console_putchar('\n');
  }
  if (cfsr & SCB_CFSR_DACCVIOL_Msk) {
    console_print("DACCVIOL/Data access violation on "), print_address(ctx->mmfar);
    console_putchar('\n');
  }
  if (cfsr & SCB_CFSR_MUNSTKERR_Msk) {
    console_print("MUNSTKERR/MemFault during unstacking after exception\n");
  }
  if (cfsr & SCB_CFSR_MSTKERR_Msk) {
    console_print("MSTKERR/MemFault during stacking before exception\n");
  }
  if (cfsr & SCB_CFSR_MLSPERR_Msk) {
    console_print("MLSPERR/MemFault during floating-point state stacking\n");
  }

  if (cfsr & SCB_CFSR_IBUSERR_Msk) {
    console_print("IBUSERR/Bus error during instruction prefetch\n");
  }
  if (cfsr & SCB_CFSR_PRECISERR_Msk) {
    console_print("PRECISERR/Bus error on access to "), print_address(ctx->bfar);
    console_putchar('\n');
  }
  if (cfsr & SCB_CFSR_IMPRECISERR_Msk) {
    console_print("IMPRECISERR/Imprecise bus error\n");
  }
  if (cfsr & SCB_CFSR_UNSTKERR_Msk) {
    console_print("UNSTKERR/Bus error during unstacking after exception\n");
  }
  if (cfsr & SCB_CFSR_STKERR_Msk) {
    console_print("STKERR/Bus error during stacking before exception\n");
  }
  if (cfsr & SCB_CFSR_LSPERR_Msk) {
    console_print("LSPERR/Bus error during floating-point state stacking\n");
  }

  if (cfsr & SCB_CFSR_UNDEFINSTR_Msk) {
    console_print("UNDEFINSTR/Undefined instruction at "), print_address(regs->pc);
    console_putchar('\n');
  }
  if (cfsr & SCB_CFSR_INVSTATE_Msk) {
    console_print("INVSTATE/Attempt to enter invalid execution state\n");
    console_print("(Most likely caused by a jump to an address without\n");
    console_print("the thumb mode bit set)\n");
  }
  if (cfsr & SCB_CFSR_INVPC_Msk) {
    console_print("INVPC/Invalid value of LR on return from exception\n");
  }
  if (cfsr & SCB_CFSR_NOCP_Msk) {
    console_print("NOCP/Coprocessor is not available\n");
  }
  if (cfsr & SCB_CFSR_UNALIGNED_Msk) {
    if (cfsr & SCB_CFSR_MMARVALID_Msk) {
      console_print("UNALIGNED/Unaligned access to address ");
      print_address(ctx->mmfar);
    } else {
      console_print("UNALIGNED/Unaligned access to unknown address\n");
    }
  }
  if (cfsr & SCB_CFSR_DIVBYZERO_Msk) {
    console_print("DIVBYZERO/Division by zero error\n");
  }

  if (hfsr & SCB_HFSR_VECTTBL_Msk) {
    console_print("VECTTBL/Failed to fetch an address from vector table\n");
  }
  if (hfsr & SCB_HFSR_DEBUGEVT_Msk) {
    console_print("DEBUGEVT/The debug system is disabled\n");
  }
}

static void crash_screen_mpu_diagnosis(void) {
  struct CrashHardfault* ctx = &crash_context.payload.hardfault;
  if (ctx->mpu_diagnosis) {
    console_print("MPU//");
    switch (ctx->mpu_diagnosis) {
      case MPU_FAULT_UNKNOWN: console_print("unknown\n"); break;
      case MPU_FAULT_NULL_POINTER: console_print("NULL pointer error\n"); break;
      case MPU_FAULT_STACK_OVERFLOW: console_print("stack overflow\n"); break;
      case MPU_FAULT_NOT_MAPPED: console_print("access into an unmapped region\n"); break;
      case MPU_FAULT_ACCESS_BLOCKED: console_print("access into region is blocked\n"); break;
      case MPU_FAULT_WRITE_TO_READONLY: console_print("write to a readonly region\n"); break;
      case MPU_FAULT_NOT_EXECUTABLE: console_print("jump into a non-executable region\n"); break;
      case MPU_FAULT_UNPRIV_ACCESS_BLOCKED:
        console_print("unprivileged access into region is blocked\n");
        break;
      case MPU_FAULT_UNPRIV_WRITE_TO_READONLY:
        console_print("region is readonly for unprivileged access\n");
        break;
      case MPU_FAULT_UNPRIV_NOT_EXECUTABLE:
        console_print("unprivileged execution from region is blocked\n");
        break;
    }
  }
}

__NO_RETURN void enter_crash_screen(void) {
  // At least light up the error LED in case we crash further down below.
  LL_GPIO_ResetOutputPin(BLTN_LED_GPIO_Port, BLTN_LED_Pin);

  // Disable all interrupts with priority 1 and below (the minimum priority
  // level needs to be shifted because the lower 4 bits of the register are
  // ignored by the hardware).
  __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));

  // We need at least the VGA interrupts to output something to the screen.
  __enable_irq();

  console_clear_screen();
  console_set_color(0x97);
  console_clear_cursor_line();
  console_set_color(0x17);
  console_print(" CRITICAL ERROR ");
  console_set_color(0x97);
  console_putchar(' ');

  if (crash_context.type == CRASH_TYPE_ASSERTION) {
    struct CrashAssertion* ctx = &crash_context.payload.assertion;
    if (ctx->message != NULL) {
      console_print(ctx->message);
    }
  } else if (crash_context.type == CRASH_TYPE_HARDFAULT) {
    crash_screen_hardfault_title();
  }

  console_putchar(' ');
  console_set_color(0x07);
  console_putchar('\n');
  console_set_color(0x01);

  struct CrashCpuRegisters* regs = &crash_context.cpu_registers;

  if (crash_context.type == CRASH_TYPE_ASSERTION) {
    struct CrashAssertion* ctx = &crash_context.payload.assertion;
    if (ctx->src_file != NULL) {
      console_print(ctx->src_file), console_putchar(':'), print_number(ctx->src_line, 10);
      console_putchar(' ');
    }
    console_print("[pc="), print_address((u32)ctx->address), console_print("]\n");
  } else if (crash_context.type == CRASH_TYPE_HARDFAULT) {
    crash_screen_hardfault_reason();
    crash_screen_mpu_diagnosis();
  }

  console_putchar('\n');
  console_set_color(0x07);

  u32 interrupt_nr = crash_context.cpu_registers.xpsr & 0x1FF;
  if (interrupt_nr != 0) {
    console_print("[in ISR ");
    print_number(interrupt_nr, 10);
    console_print("]\n");
  }

  if (crash_context.cpu_registers_collected) {
    print_register("r0", regs->r0), console_putchar(' ');
    print_register("r1", regs->r1), console_putchar(' ');
    print_register("r2", regs->r2), console_putchar(' ');
    print_register("r3", regs->r3), console_putchar('\n');
    print_register("r4", regs->r4), console_putchar(' ');
    print_register("r5", regs->r5), console_putchar(' ');
    print_register("r6", regs->r6), console_putchar(' ');
    print_register("r7", regs->r7), console_putchar('\n');
    print_register("r8", regs->r8), console_putchar(' ');
    print_register("r9", regs->r9), console_putchar(' ');
    print_register("sl", regs->r10), console_putchar(' ');
    print_register("fp", regs->r11), console_putchar('\n');
    print_register("ip", regs->r12), console_putchar(' ');
    print_register("sp", regs->sp), console_putchar(' ');
    print_register("lr", regs->lr), console_putchar(' ');
    print_register("pc", regs->pc), console_putchar('\n');
    print_register("xPSR", regs->xpsr), console_putchar(' ');
  } else {
    console_print("[CPU registers have not been collected]\n");
  }

  if (crash_context.type == CRASH_TYPE_HARDFAULT) {
    struct CrashHardfault* ctx = &crash_context.payload.hardfault;
    print_register("EXC_RETURN", ctx->exc_return), console_putchar('\n');
    print_register("CSFR", ctx->cfsr), console_putchar(' ');
    print_register("HSFR", ctx->hfsr), console_putchar(' ');
    print_register("DSFR", ctx->dfsr), console_putchar('\n');
    print_register("AFSR", ctx->afsr), console_putchar(' ');
    if (ctx->cfsr & SCB_CFSR_MMARVALID_Msk) {
      print_register("MFAR", ctx->mmfar), console_putchar(' ');
    }
    if (ctx->cfsr & SCB_CFSR_BFARVALID_Msk) {
      print_register("BFAR", ctx->bfar), console_putchar(' ');
    }
    console_putchar('\n');
  } else {
    console_putchar('\n');
  }

  u32 prev_tick = HAL_GetTick();
  const u32 BLINK_DELAY = 250;
  while (true) {
    WAIT_FOR_INTERRUPT();
    u16 vga_line = 0;
    if (vga_take_scanline_request(&vga_line)) {
      console_render_scanline(vga_line);
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
    }
    u32 tick = HAL_GetTick();
    if (tick - prev_tick >= BLINK_DELAY) {
      prev_tick = tick;
      LL_GPIO_TogglePin(BLTN_LED_GPIO_Port, BLTN_LED_Pin);
    }
  }
}

__NO_RETURN void crash_on_hal_error(HAL_StatusTypeDef code, const char* file, u32 line) {
  static const char* const ERROR_NAMES[] = {
    [HAL_OK] = "OK",
    [HAL_ERROR] = "ERROR",
    [HAL_BUSY] = "BUSY",
    [HAL_TIMEOUT] = "TIMEOUT",
  };
  const char* name = (u32)code < SIZEOF(ERROR_NAMES) ? ERROR_NAMES[code] : "UNKNOWN";
  char msg[32];
  snprintf(msg, sizeof(msg), "0x%02" PRIX32 "/HAL_%s", (u32)code, name);
  crash(msg, file, line);
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) {
  crash("STM32 HAL assertion failed", (const char*)file, line);
}
#endif

void crash_init_hard_faults(void) {
  // Enable UsageFaults on division by zero and unaligned memory access.
  // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/Configuration-and-Control-Register--CCR>
  SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk;

  // Enable UsageFault, BusFault and MemFault.
  // <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/System-Control-Space--SCS-/System-Handler-Control-and-State-Register--SHCSR>
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

  // To debug imprecise BusFaults, write buffering may be disabled (though it
  // will slow down everything).
  // SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;

  // Ensure these settings have been applied before returning.
  __DSB();
  __ISB();
}
