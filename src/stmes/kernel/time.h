#pragma once

#include "stmes/utils.h"
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// typedef union Instant {
//   u64 as_u64;
//   struct {
// #if BYTE_ORDER == LITTLE_ENDIAN
//     u32 lo, hi;
// #elif BYTE_ORDER == BIG_ENDIAN
//     u32 hi, lo;
// #else
// #error "Failed to determine endianness on this platform"
// #endif
//   } as_u32;
// } Instant;

typedef u64 Instant;

void hwtimer_init(void);
void hwtimer_deinit(void);
u32 hwtimer_read(void);
void hwtimer_set_alarm(u32 value);

Instant systime_now(void);

#define NO_DEADLINE ((Instant)-1)

__STATIC_FORCEINLINE Instant timeout_to_deadline(u32 timeout) {
  return systime_now() + timeout;
}

#ifdef __cplusplus
}
#endif
