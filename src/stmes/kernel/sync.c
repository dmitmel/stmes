#include "stmes/kernel/sync.h"
#include "stmes/kernel/task.h"

void mutex_init(struct Mutex* self) {
  __atomic_clear(&self->locked, __ATOMIC_RELAXED);
  task_notify_init(&self->notify);
}

void mutex_lock(struct Mutex* self) {
  while (__atomic_test_and_set(&self->locked, __ATOMIC_ACQUIRE) != 0) {
    task_wait(&self->notify, NO_DEADLINE);
  }
}

void mutex_unlock(struct Mutex* self) {
  __atomic_clear(&self->locked, __ATOMIC_RELEASE);
  if (task_notify(&self->notify)) {
    task_yield();
  }
}
