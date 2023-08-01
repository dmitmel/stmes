#pragma once

#include "stmes/kernel/task.h"
#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

struct Mutex {
  volatile u8 locked;
  struct Notification notify;
};

void mutex_init(struct Mutex* self);
void mutex_lock(struct Mutex* self);
void mutex_unlock(struct Mutex* self);

#ifdef __cplusplus
}
#endif
