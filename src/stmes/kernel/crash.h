#pragma once

#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

void crash_collect_registers(void);

__NO_RETURN void crash(const char* message, const char* src_file, u32 src_line);

#define CRASH(message) (crash_collect_registers(), crash((message), __FILE__, __LINE__))

#define ASSERT(expr) (unlikely(!(expr)) ? CRASH("Assertion failed") : (void)0)

__NO_RETURN void enter_crash_screen(void);

void crash_init_hard_faults(void);

#ifdef __cplusplus
}
#endif
