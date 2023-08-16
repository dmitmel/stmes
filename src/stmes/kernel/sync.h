#pragma once

#include "stmes/kernel/task.h"
#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: deadlines

struct Mutex {
  volatile bool locked;
  volatile TaskId owner;
  u16 lock_count; // Using a 16-bit field here to use up all the padding bytes.
  struct Notification notify;
};

void mutex_init(struct Mutex* self);
void mutex_lock(struct Mutex* self);
bool mutex_try_lock(struct Mutex* self);
void mutex_unlock(struct Mutex* self);

#ifdef __cplusplus
}
#endif
