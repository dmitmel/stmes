#pragma once

#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

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
