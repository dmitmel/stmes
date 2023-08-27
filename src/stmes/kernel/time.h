#pragma once

#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef u64 Systime;

#define SYSTIME_TICKS_PER_MS 2

void hwtimer_init(void);
void hwtimer_deinit(void);
u32 hwtimer_read(void);
void hwtimer_set_alarm(u32 value);

Systime systime_now(void);

#define NO_DEADLINE ((Systime)-1)

__STATIC_FORCEINLINE Systime timeout_to_deadline(u64 timeout_ms) {
  return systime_now() + (timeout_ms * SYSTIME_TICKS_PER_MS);
}

__STATIC_FORCEINLINE Systime systime_from_millis(u64 ms) {
  return ms * SYSTIME_TICKS_PER_MS;
}

__STATIC_FORCEINLINE Systime systime_from_secs(u64 secs) {
  return (secs * 1000) * SYSTIME_TICKS_PER_MS;
}

__STATIC_FORCEINLINE u64 systime_as_millis(Systime time) {
  return time / SYSTIME_TICKS_PER_MS;
}

__STATIC_FORCEINLINE u64 systime_as_secs(Systime time) {
  return (time / SYSTIME_TICKS_PER_MS) / 1000;
}

#ifdef __cplusplus
}
#endif
