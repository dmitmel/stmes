#pragma once

#include "stmes/kernel/task.h"
#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  REG_SL = 10,
  REG_FP = 11,
  REG_IP = 12,
  REG_SP = 13,
  REG_LR = 14,
  REG_PC = 15,
  REG_XPSR = 16,
  REG_MSP = 17,
  REG_PSP = 18,
  REG_CONTROL = 19,
};

enum UnwindError {
  UNWIND_OK = 0,
  UNWIND_REFUSED,
  UNWIND_RESERVED_SPARE_INSTRUCTION,
  UNWIND_NO_MATCHING_ENTRY,
  UNWIND_UNSUPPORTED_PERSONALITY,
  UNWIND_NOT_COMPACT_ENTRY,
};

struct UnwindContext {
  usize registers[20];
};

struct UnwindFrame {
  void* stack_ptr;
  void* instruction_addr;
  void* function_addr;
  const char* function_name;
  usize function_name_len;
};

__STATIC_FORCEINLINE bool unwind_is_interrupt_frame(const struct UnwindFrame* frame) {
  // The reasoning for using this condition is given in the implementation file
  return EXTRACT_BITS((usize)frame->instruction_addr, 28, 4) == 0xF;
}

const struct UnwindContext* unwind_capture_context(struct UnwindContext* ctx);
void unwind_capture_task_context(struct UnwindContext* ctx, const struct Task* task);
void* unwind_interrupt_frame(struct UnwindContext* ctx, usize exc_return);
enum UnwindError unwind_frame(struct UnwindContext* ctx, struct UnwindFrame* frame);
enum UnwindError backtrace(struct UnwindContext* ctx);

const char* peek_function_name(usize addr, usize* name_len);

#ifdef __cplusplus
}
#endif
