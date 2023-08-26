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

void channel_init(struct Channel* self) {
  __atomic_store_n(&self->state, CHANNEL_IDLE, __ATOMIC_RELAXED);
  task_notify_init(&self->notify);
  self->message = NULL;
  self->message_size = 0;
}

void channel_send(struct Channel* self, void* message, usize size) {
  // Wait for the possibility to claim the channel for ourselves
  while (true) {
    enum ChannelState wanted_state = CHANNEL_IDLE, next_state = CHANNEL_SENDING_MESSAGE;
    bool success = __atomic_compare_exchange_n(
      &self->state, &wanted_state, next_state, /* weak */ false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED
    );
    if (success) break;
    task_wait(&self->notify, NO_DEADLINE);
  }
  // Send the message
  self->message = message;
  self->message_size = size;
  __atomic_store_n(&self->state, CHANNEL_MESSAGE_SENT, __ATOMIC_RELEASE);
  if (task_notify(&self->notify)) {
    task_yield();
  }
  // Wait for the message to be received on the other side
  while (__atomic_load_n(&self->state, __ATOMIC_ACQUIRE) != CHANNEL_MESSAGE_RECEIVED) {
    task_wait(&self->notify, NO_DEADLINE);
  }
  // Conclude the transaction
  __atomic_store_n(&self->state, CHANNEL_IDLE, __ATOMIC_RELEASE);
  if (task_notify(&self->notify)) {
    task_yield();
  }
}

void channel_recv(struct Channel* self, void* message, usize size) {
  // Wait for the possibility to claim a sent message for ourselves
  while (true) {
    u8 wanted_state = CHANNEL_MESSAGE_SENT, next_state = CHANNEL_RECEIVING_MESSAGE;
    bool success = __atomic_compare_exchange_n(
      &self->state, &wanted_state, next_state, /* weak */ false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED
    );
    if (success) break;
    task_wait(&self->notify, NO_DEADLINE);
  }
  // Receive the message
  ASSERT(self->message_size == size);
  __builtin_memcpy(message, self->message, size);
  // Signal that the message has been received
  __atomic_store_n(&self->state, CHANNEL_MESSAGE_RECEIVED, __ATOMIC_RELEASE);
  if (task_notify(&self->notify)) {
    task_yield();
  }
}
