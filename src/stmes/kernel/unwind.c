// This module performs stack unwinding by utilizing the metadata used for C++
// exception handling. For an exception to be able to "bubble up" the call
// stack until it reaches a try-catch block, the compiler generates special
// tables with instructions on how to interpret the stack frame created by any
// given function, even in the absence of a frame pointer (i.e. when using
// -fomit-frame-pointer), plus some additional information like where on the
// stack do local variables reside. These tables can also be generated when
// compiling C functions, so that exceptions may be thrown through them (even
// though they have no way of catching them) when mixing C and C++ code. The
// language runtime then uses this information when an exception is thrown to
// essentially forcibly perform an early return from every function by
// restoring the registers from the stack, running the destructors of local
// variables in the process, and one by one pop each frame off the stack before
// reaching the enclosing catch block and resuming execution there - this is
// what's called stack unwinding. However, this metadata may also be used for
// other purposes, e.g. to just analyze and recover the function call stack and
// produce a backtrace, which this module's primary use-case.
//
// It turns out that ARM defines its own special ABI for the stack unwinding
// tables, found in sections `.ARM.extab` and `.ARM.exidx` of the binary, which
// are pretty compact and relatively easy to parse and interpret. The format of
// these tables is principally described in the ARM EHABI32 specification:
// <https://github.com/ARM-software/abi-aa/releases/download/2023Q3/ehabi32.pdf>
// <https://github.com/ARM-software/abi-aa/blob/2023Q3/ehabi32/ehabi32.rst>
// (with the relevant info being contained in sections 6, 7 and 10).
//
// Additional resources that were helpful in understanding that stuff:
// <https://alexkalmuk.medium.com/how-stack-trace-on-arm-works-5634b35ddca1>
// <https://github.com/red-rocket-computing/backtrace/blob/master/doc/libunwind-LDS.pdf>
// <https://stackoverflow.com/questions/57451208/whats-the-structure-of-arm-extab-entry-in-armcc>
// <https://stackoverflow.com/questions/15752188/arm-link-register-and-frame-pointer>
// <https://blog.bachi.net/?p=7830> - even more interesting links on the matter
// <https://maskray.me/blog/2020-11-08-stack-unwinding>
// <https://sourceware.org/binutils/docs/as/ARM-Unwinding-Tutorial.html>
// <https://sourceware.org/binutils/docs/as/ARM-Directives.html>
//
// Some existing unwinder implementations:
// <https://github.com/red-rocket-computing/backtrace/blob/a438f6474f373224cf0bf8da69b18e18c4f364b3/backtrace/backtrace.c>
// <https://github.com/torvalds/linux/blob/v6.6/arch/arm/kernel/unwind.c>
// <https://github.com/RT-Thread/rt-thread/blob/df0d8b4230394a04b97482a9ea546e26e46b72a8/libcpu/arm/cortex-a/backtrace.c>
// <https://github.com/apache/nuttx/blob/nuttx-12.3.0/arch/arm/src/common/arm_backtrace_unwind.c>
// (the last two are largely derived from the Linux code)
//
// These are involved in implementing the whole exception handling runtime
// infrastructure, not only unwinding:
// <https://github.com/llvm/llvm-project/blob/llvmorg-17.0.4/libunwind/src/Unwind-EHABI.cpp>
// <https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.2.0/libgcc/unwind-arm-common.inc>
// <https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.2.0/libgcc/config/arm/unwind-arm.c>
// <https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.2.0/libgcc/config/arm/libunwind.S>
// <https://github.com/libunwind/libunwind/blob/v1.7.2/src/arm/Gex_tables.c>
// <https://github.com/libunwind/libunwind/blob/v1.7.2/src/arm/Gstep.c>
//
// There exist alternative approaches of stack unwinding, based on analyzing
// the function prologues and epilogues:
// <https://www.mcternan.me.uk/ArmStackUnwinding/>
// <https://github.com/MarlinFirmware/Marlin/tree/2.1.2.1/Marlin/src/HAL/shared/backtrace>
// <https://github.com/MarlinFirmware/Marlin/blob/2.1.2.1/Marlin/src/HAL/shared/backtrace/unwarm_thumb.cpp>
// <https://yosefk.com/blog/getting-the-call-stack-without-a-frame-pointer.html>
// Or using frame pointers, which turn the stack into a linked list of frames:
// <https://github.com/embox/embox/blob/v0.6.0/src/arch/arm/lib/debug/stack_iter.c>
// Or by looking for LR pointers on the stack with sophisticated heuristics:
// <https://github.com/armink/CmBacktrace/blob/1.4.1/cm_backtrace/cm_backtrace.c>
//
// This link is not super relevant, but contains interesting discussions on the
// internals of the ARM architecture:
// <https://stackoverflow.com/questions/24091566/why-does-the-arm-pc-register-point-to-the-instruction-after-the-next-one-to-be-e>

// TODO: Unwinding through interrupt frames
// TODO: Stack pointer range checks

#include "stmes/kernel/unwind.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include <printf.h>

#if 0
#define unwind_log(...) printf(__VA_ARGS__)
#else
#define unwind_log(...) ((void)0)
#endif

// An entry in the `.ARM.exidx` table.
struct UnwindIndex {
  // An R_ARM_PREL31 relocation or a so-called "prel31 offset" which points to
  // the start of a function covered by this entry. The region of this function
  // ends where that of the next entry starts.
  u32 function_addr;
  // Contains either a prel31 offset to the list of unwinding instructions in
  // the `.ARM.extab` segment or an inlined list of instructions when it is
  // short enough (which it often is for C functions).
  u32 instructions;
};

// The pointers to the boundaries of the `.ARM.exidx` segment, are provided by
// the linker script.
extern const struct UnwindIndex __exidx_start[], __exidx_end[];

// Decodes a prel31 offset to an absolute address.
__STATIC_FORCEINLINE usize prel31_to_addr(const u32* ptr) {
  // Sign-extend the low 31 bits (the 32nd bit is used for data) to a 32-bit
  // int. <https://stackoverflow.com/questions/12578934/longptr-1-1>
  i32 offset = ((i32)*ptr) << 1 >> 1;
  return (usize)ptr + offset;
}

// The `.ARM.exidx` table is sorted by the function address in ascending order,
// which means we can binary-search it.
static const struct UnwindIndex*
unwind_search_index(usize addr, const struct UnwindIndex* start, const struct UnwindIndex* end) {
  if (end <= start) {
    return NULL; // Return immediately if the table is empty
  }
  while (start < end - 1) { // Unsure about the plus/minus ones here, but, well, it works
    const struct UnwindIndex* middle = start + (end - start + 1) / 2;
    if (addr < prel31_to_addr(&middle->function_addr)) {
      end = middle;
    } else {
      start = middle;
    }
  }
  if (addr < prel31_to_addr(&start->function_addr)) {
    // The requested address points outside the index table.
    // NOTE: This branch only handles addresses before the index with the
    // smallest address. For catching addresses after the end of the memory
    // region covered by the unwind tables the compiler emits an entry with the
    // EXIDX_CANTUNWIND bit at the very end of the index table.
    return NULL;
  } else {
    return start;
  }
}

// A crude way of getting the symbol names from within the firmware without
// access to the symbol table, depends on the conveniently available (since at
// least 1999) option -mpoke-function-name:
// <https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html#index-mpoke-function-name>
// <https://gcc.gnu.org/legacy-ml/gcc-patches/1999-04n/msg01000.html>
const char* peek_function_name(usize addr, usize* name_len) {
#if ARM_POKE_FUNCTION_NAME
  const u32 flag_bits = 0xFF000000;
  u32 word = *(const u32*)(addr - 4);
  if (test_all_bits(word, flag_bits)) {
    *name_len = clear_bit(word, flag_bits);
    return (const char*)(addr - 4 - *name_len);
  }
#else
  UNUSED(addr);
#endif
  *name_len = 0;
  return NULL;
}

struct UnwindReader {
  const u32* ptr;
  usize len, pos;
  u32 modified_registers;
};

static enum UnwindError
unwind_create_reader(struct UnwindReader* reader, const struct UnwindIndex* index) {
  const u32* instructions;
  bool is_inline = false;
  if (index->instructions & BIT(31)) {
    is_inline = true; // An inline entry embedded in `.ARM.exidx`
    instructions = &index->instructions;
  } else {
    // A prel31 pointer to an entry in `.ARM.extab`
    instructions = (const u32*)prel31_to_addr(&index->instructions);
  }

  bool is_compact = *instructions & BIT(31);
  if (!is_compact) return UNWIND_NOT_COMPACT_ENTRY;
  reader->ptr = instructions;

  u8 personality = EXTRACT_BITS(*instructions, 24, 7);
  if (personality == 0) {
    // A short description inlined into the remaining bytes of the word.
    reader->pos = 1;
    reader->len = 4;
    return UNWIND_OK;
  } else if (is_inline) {
    // Entries inlined into the index can only have personality 0.
    return UNWIND_UNSUPPORTED_PERSONALITY;
  } else if (personality == 1 || personality == 2) {
    // A long description consisting of N words, plus two inlined bytes.
    reader->pos = 2;
    reader->len = 4 + 4 * EXTRACT_BITS(*instructions, 16, 8);
    return UNWIND_OK;
  } else {
    return UNWIND_UNSUPPORTED_PERSONALITY;
  }
}

static u8 unwind_read_byte(struct UnwindReader* reader) {
  u32 pos = reader->pos;
  ASSERT(pos < reader->len); // TODO: Handle malformed instructions more gracefully...
  reader->pos = pos + 1;
  u8 byte = reader->ptr[pos / 4] >> ((3 - pos % 4) * 8);
  unwind_log("%02" PRIX8 " ", byte);
  return byte;
}

static u32 unwind_read_uleb128(struct UnwindReader* reader) {
  // <https://en.wikipedia.org/wiki/LEB128#Decode_unsigned_integer>
  // NOTE: I chose to decode at most 4 bytes, which can encode a 28 bit number
  // with the max value of 0xFFFFFFF. This gives a maximum possible SP
  // increment of 1073742336, which is more than enough for all practical
  // intents and purposes.
  u32 result = 0;
  u32 shift = 0;
  u8 byte;
  do {
    ASSERT(shift < sizeof(u32) * 7);
    byte = unwind_read_byte(reader);
    result |= (byte & MASK(7)) << shift;
    shift += 7;
  } while (byte & BIT(7));
  return result;
}

static enum UnwindError
unwind_pop_registers(struct UnwindReader* reader, struct UnwindContext* ctx, u32 regs_mask) {
  if (regs_mask == 0) {
    return UNWIND_RESERVED_SPARE_INSTRUCTION;
  }
  unwind_log("pop {");
  const u32* sp = (const u32*)ctx->registers[REG_SP];
  // Find the first and the last set bits in the mask to reduce the number of
  // useless iterations in the loop below. Typically, a contiguous range of
  // core registers will be popped.
  u32 first_reg = __builtin_ctz(regs_mask), last_reg = 31 - __builtin_clz(regs_mask);
  last_reg = MIN(last_reg, SIZEOF(ctx->registers) - 1);
  for (u32 i = first_reg; i <= last_reg; i++) {
    if (regs_mask & BIT(i)) {
      ctx->registers[i] = *sp++;
      unwind_log("%sr%" PRIu32, i != first_reg ? ", " : "", i);
    }
  }
  if (!(regs_mask & BIT(REG_SP))) {
    ctx->registers[REG_SP] = (usize)sp;
  }
  reader->modified_registers |= regs_mask | BIT(REG_SP);
  unwind_log("}\n");
  return UNWIND_OK;
}

// What the hell is the FSTMFDX instruction?
// <https://developer.arm.com/documentation/ddi0406/cb/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/F---former-Floating-point-instruction-mnemonics>
enum FpuFrame { FPU_FRAME_VPUSH, FPU_FRAME_FSTMFDX };

static enum UnwindError unwind_pop_fpu_registers(
  struct UnwindReader* reader,
  struct UnwindContext* ctx,
  u32 first_reg,
  u32 regs_count,
  enum FpuFrame frame_type
) {
  u32 max_reg = frame_type == FPU_FRAME_FSTMFDX ? 16 : 32;
  if (first_reg >= max_reg || first_reg + regs_count > max_reg || regs_count < 1) {
    return UNWIND_RESERVED_SPARE_INSTRUCTION;
  }
  // The FPU registers aren't actually tracked right now, only the SP is
  // sufficiently adjusted AS IF a bunch of double-precision FPU registers have
  // been popped.
  ctx->registers[REG_SP] += 8 * regs_count + (frame_type == FPU_FRAME_FSTMFDX ? 4 : 0);
  reader->modified_registers |= BIT(REG_SP);
  if (regs_count > 1) {
    unwind_log("vpop {d%" PRIu32 "-d%" PRIu32 "}\n", first_reg, first_reg + regs_count - 1);
  } else {
    unwind_log("vpop {d%" PRIu32 "}\n", first_reg);
  }
  return UNWIND_OK;
}

// A comprehensive summary of the unwind instruction encodings:
// <https://github.com/ARM-software/abi-aa/blob/2023Q3/ehabi32/ehabi32.rst#ehabi32-table4>
static enum UnwindError
unwind_exec_instruction(struct UnwindReader* reader, struct UnwindContext* ctx) {
  u8 insn = unwind_read_byte(reader);

  if ((insn & BIT(7)) == 0) {
    // 00xxxxxx: vsp = vsp + (xxxxxx << 2) + 4
    // 01xxxxxx: vsp = vsp - (xxxxxx << 2) - 4
    u32 offset = 4 + (insn & MASK(6)) * 4;
    if ((insn & BIT(6)) == 0) {
      ctx->registers[REG_SP] += offset;
    } else {
      ctx->registers[REG_SP] -= offset;
    }
    reader->modified_registers |= BIT(REG_SP);
    unwind_log("sp %c= %" PRIu32 "\n", (insn & BIT(6)) ? '-' : '+', offset);
    return UNWIND_OK;
  }

  switch ((insn >> 4) & MASK(3)) {
    case 0: {
      // 1000iiii iiiiiiii: pop under masks {r15-r12}, {r11-r4}
      u8 insn2 = unwind_read_byte(reader);
      u32 regs_mask = ((insn & MASK(4)) << 12) | ((insn2 & MASK(8)) << 4);
      if (regs_mask == 0) {
        // 10000000 00000000: refuse to unwind
        unwind_log("refused\n");
        return UNWIND_REFUSED;
      }
      return unwind_pop_registers(reader, ctx, regs_mask);
    }

    case 1: {
      // 1001nnnn: vsp = r[nnnn]
      u32 reg = insn & MASK(4);
      if (reg == 13 || reg == 15) {
        // 10011101: reserved for register-to-register moves
        // 10011111: reserved for Intel WMMX register to ARM register moves
        return UNWIND_RESERVED_SPARE_INSTRUCTION;
      }
      ctx->registers[REG_SP] = ctx->registers[reg];
      reader->modified_registers |= BIT(REG_SP);
      unwind_log("sp = r%" PRIu32 "\n", reg);
      return UNWIND_OK;
    }

    case 2: {
      // 10100nnn: pop r4-r[4+nnn]
      // 10101nnn: pop r4-r[4+nnn], r14
      u32 last_reg = 4 + (insn & MASK(3));
      u32 regs_mask = MASK(last_reg + 1) & ~MASK(4);
      if ((insn & BIT(3)) != 0) regs_mask |= BIT(REG_LR);
      return unwind_pop_registers(reader, ctx, regs_mask);
    }

    case 3: {
      if ((insn & BIT(3)) != 0) {
        // 10111nnn: pop D[8]-D[8+nnn] saved with FSTMFDX
        u32 regs_count = insn & MASK(3);
        return unwind_pop_fpu_registers(reader, ctx, 8, regs_count, FPU_FRAME_FSTMFDX);
      }

      switch (insn & MASK(3)) {
        case 0: {
          // 10110000: finish
          unwind_log("finish\n");
          reader->pos = reader->len; // Skip to the end of the instruction stream
          return UNWIND_OK;
        }

        case 1: {
          // 10110001 0000iiii: pop under mask {r3,r2,r1,r0}
          u8 insn2 = unwind_read_byte(reader);
          u32 regs_mask = insn2 & MASK(4);
          if (regs_mask == 0 || (insn2 & ~MASK(4)) != 0) {
            // 10110001 00000000: spare
            // 10110001 xxxxyyyy: spare (xxxx != 0)
            return UNWIND_RESERVED_SPARE_INSTRUCTION;
          }
          return unwind_pop_registers(reader, ctx, regs_mask);
        }

        case 2: {
          // 10110010 uleb128: vsp = vsp + 0x204 + (uleb128 << 2)
          u32 offset = 0x204 + unwind_read_uleb128(reader) * 4;
          ctx->registers[REG_SP] += offset;
          reader->modified_registers |= BIT(REG_SP);
          unwind_log("sp += %" PRIu32 "\n", offset);
          return UNWIND_OK;
        }

        case 3: {
          // 10110011 sssscccc: pop D[ssss]-D[ssss+cccc] saved with FSTMFDX
          u8 insn2 = unwind_read_byte(reader);
          u32 first_reg = (insn2 >> 4) & MASK(4), regs_count = (insn2 & MASK(4)) + 1;
          return unwind_pop_fpu_registers(reader, ctx, first_reg, regs_count, FPU_FRAME_FSTMFDX);
        }
      }
      break;
    }

    case 4: {
      if ((insn & BIT(3)) != 0) {
        if ((insn & BITS4(0, 1, 1, 0)) != 0) {
          // 11001yyy: spare (yyy != 000, 001)
          return UNWIND_RESERVED_SPARE_INSTRUCTION;
        }
        // 11001000 sssscccc: pop D[16+ssss]-D[16+ssss+cccc] saved with VPUSH
        // 11001001 sssscccc: pop D[ssss]-D[ssss+cccc] saved with VPUSH
        u8 insn2 = unwind_read_byte(reader);
        u32 first_reg = (insn2 >> 4) & MASK(4), regs_count = (insn2 & MASK(4)) + 1;
        if ((insn & BIT(0)) == 0) first_reg += 16;
        return unwind_pop_fpu_registers(reader, ctx, first_reg, regs_count, FPU_FRAME_VPUSH);
      }
      break;
    }

    case 5: {
      if ((insn & BIT(3)) == 0) {
        // 11010nnn: pop D[8]-D[8+nnn] saved with VPUSH
        u32 regs_count = insn & MASK(3);
        return unwind_pop_fpu_registers(reader, ctx, 8, regs_count, FPU_FRAME_VPUSH);
      }
      break;
    }
  }

  return UNWIND_RESERVED_SPARE_INSTRUCTION;
}

enum UnwindError unwind_frame(struct UnwindContext* ctx, struct UnwindFrame* frame) {
  enum UnwindError err = UNWIND_OK;

  *frame = (struct UnwindFrame){ 0 };

  // Clear the Thumb state bit of the address in PC
  usize pc = clear_bit(ctx->registers[REG_PC], BIT(0));
  frame->instruction_addr = (void*)pc;
  frame->stack_ptr = (void*)ctx->registers[REG_SP];

  // NOTE: This is a bad idea, I am disassembling the wrong instruction.
  // <https://stackoverflow.com/questions/28860250/how-to-determine-if-a-word4-bytes-is-a-16-bit-instruction-or-32-bit-instructio>
  // u8 pc_bits_8_15 = *(const u8*)(pc + 1);
  // pc -= 2;
  // if ((pc_bits_8_15 >> 5) == 0x7) {
  //   u8 op1 = (pc_bits_8_15 >> 3) & MASK(2);
  //   if (op1 == 0x1 || op1 == 0x2 || op1 == 0x3) {
  //     pc -= 2;
  //   }
  // }

  const struct UnwindIndex* index = unwind_search_index(pc, __exidx_start, __exidx_end);
  if (index == NULL || index->instructions == /* EXIDX_CANTUNWIND */ 0x1) {
    return UNWIND_NO_MATCHING_ENTRY;
  }

  usize func_addr = prel31_to_addr(&index->function_addr);
  frame->function_addr = (void*)func_addr;
  frame->function_name = peek_function_name(func_addr, &frame->function_name_len);

  struct UnwindReader reader;
  if ((err = unwind_create_reader(&reader, index))) return err;

  reader.modified_registers = 0;
  while (reader.pos < reader.len) {
    if ((err = unwind_exec_instruction(&reader, ctx))) return err;
  }

  // Prevent infinite loops if none of the important registers were changed.
  if (!test_any_bit(reader.modified_registers, BIT(REG_SP) | BIT(REG_LR) | BIT(REG_PC))) {
    return UNWIND_REFUSED;
  }

  // `mov pc, lr` if the PC hasn't already been touched.
  if (!test_bit(reader.modified_registers, BIT(REG_PC))) {
    ctx->registers[REG_PC] = ctx->registers[REG_LR];
  }

  return err;
}

// Records the register values AT THE CALL SITE into the given context struct,
// so that unwinding starts directly at the caller function. Unfortunately, due
// to the fact that the pointer to the context is passed in r0, the first
// argument register, its original value will be lost, but honestly, who cares?
__NOINLINE __NAKED void unwind_capture_context(__UNUSED struct UnwindContext* ctx) {
  __ASM volatile( //
    // r13 and r15 (SP and PC respectively) can't be present in the register
    // list of the `STM` instruction (the encoding doesn't allow it), so save
    // what we can first. The address in r0 is not incremented, so it can be
    // present in the list.
    "stmia r0, {r0-r12}\n\t"
    // Now, since the task at hand is to create a context as if being located
    // at the call site, write our current return address and stack pointer as
    // the values of PC and SP. The calling convention didn't require changing
    // the SP here in any way, and the LR points to the next instruction in the
    // calling function, so as far as I can tell this is completely legal.
    "str sp, [r0, #(4 * 13)]\n\t" // REG_SP
    "str lr, [r0, #(4 * 14)]\n\t" // REG_LR
    "str lr, [r0, #(4 * 15)]\n\t" // REG_PC
    // Aaand return.
    "bx lr"
  );
}

// A prototype of recording the context of another running task to be able to
// get its backtrace.
void unwind_capture_task_context(struct UnwindContext* ctx, const struct Task* task) {
  struct ExceptionStackedContext {
    u32 r0, r1, r2, r3, r12, lr, pc, xpsr;
  };

  struct TaskStackedContext {
    u32 control, r4, r5, r6, r7, r8, r9, r10, r11, exc_return;
  };

  // We can't trace ourselves like that - the stack must contain a valid
  // context switching frame.
  ASSERT(task != get_current_task());
  const u8* stack_ptr = task->stack_ptr;

  const struct TaskStackedContext* task_ctx = (void*)stack_ptr;
  stack_ptr += sizeof(*task_ctx);
  ASSERT((task_ctx->exc_return & BIT(4)) != 0); // TODO
  for (usize reg = 4; reg <= 11; reg++) {
    ctx->registers[reg] = (&task_ctx->r4)[reg - 4];
  }

  const struct ExceptionStackedContext* exc_ctx = (void*)stack_ptr;
  stack_ptr += sizeof(*exc_ctx);
  for (usize reg = 0; reg <= 3; reg++) {
    ctx->registers[reg] = (&exc_ctx->r0)[reg];
  }
  ctx->registers[12] = exc_ctx->r12;
  ctx->registers[REG_LR] = exc_ctx->lr;
  ctx->registers[REG_PC] = exc_ctx->pc;
  ctx->registers[REG_SP] = (usize)stack_ptr;
}

enum UnwindError backtrace(struct UnwindContext* ctx) {
  struct UnwindFrame frame;
  for (u32 i = 0; true; i++) {
    enum UnwindError err = unwind_frame(ctx, &frame);
    if (err != UNWIND_OK && err != UNWIND_REFUSED) break;
    printf(
      "%2" PRIu32 ": %p %.*s+0x%0" PRIXPTR "\n",
      i + 1,
      frame.instruction_addr,
      frame.function_name_len,
      frame.function_name != NULL ? frame.function_name : "<unknown>",
      (usize)frame.instruction_addr - (usize)frame.function_addr
    );
    if (err == UNWIND_REFUSED) break;
  }
  return UNWIND_OK;
}
