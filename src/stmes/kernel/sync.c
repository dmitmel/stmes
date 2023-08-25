#include "stmes/kernel/sync.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"

void mutex_init(struct Mutex* self) {
  __atomic_clear(&self->locked, __ATOMIC_RELAXED);
  __atomic_store_n(&self->owner, DEAD_TASK_ID, __ATOMIC_RELAXED);
  self->lock_count = 0;
  task_notify_init(&self->notify);
}

// The implementation of recursiveness/reentrancy for mutexes has been stolen
// from ^W ^W inspired by this:
// <https://github.com/Amanieu/parking_lot/blob/4adcfdda3cfb4e70cfd876ccbeeb176cb2b88c5b/lock_api/src/remutex.rs#L77-L152>

__STATIC_FORCEINLINE bool mutex_lock_impl(struct Mutex* self, bool just_try) {
  TaskId my_id = get_current_task()->id;
  if (__atomic_load_n(&self->owner, __ATOMIC_RELAXED) == my_id) {
    self->lock_count += 1;
    ASSERT(self->lock_count != 0); // Counter overflow check
    return true;
  }
  while (__atomic_test_and_set(&self->locked, __ATOMIC_ACQUIRE) != 0) {
    if (just_try) {
      return false;
    }
    task_wait(&self->notify, NO_DEADLINE);
  }
  __atomic_store_n(&self->owner, my_id, __ATOMIC_RELAXED);
  self->lock_count = 1;
  return true;
}

void mutex_lock(struct Mutex* self) {
  mutex_lock_impl(self, false);
}

bool mutex_try_lock(struct Mutex* self) {
  return mutex_lock_impl(self, true);
}

void mutex_unlock(struct Mutex* self) {
  ASSERT(self->lock_count != 0); // Counter underflow check
  self->lock_count -= 1;
  if (self->lock_count != 0) {
    return;
  }
  __atomic_store_n(&self->owner, DEAD_TASK_ID, __ATOMIC_RELAXED);
  __atomic_clear(&self->locked, __ATOMIC_RELEASE);
  if (task_notify(&self->notify)) {
    task_yield();
  }
}
