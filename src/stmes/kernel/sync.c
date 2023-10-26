#include "stmes/kernel/sync.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"

void mutex_init(struct Mutex* self) {
  __atomic_store_n(&self->lock_owner, DEAD_TASK_ID, __ATOMIC_RELAXED);
  self->lock_count = 0;
  task_notify_init(&self->notify);
}

// The implementation of recursiveness/reentrancy for mutexes has been stolen
// from ^W ^W inspired by this:
// <https://github.com/Amanieu/parking_lot/blob/4adcfdda3cfb4e70cfd876ccbeeb176cb2b88c5b/lock_api/src/remutex.rs#L77-L152>

__STATIC_FORCEINLINE bool mutex_lock_impl(struct Mutex* self, bool just_try) {
  const TaskId my_id = get_current_task()->id;
  while (true) {
    TaskId expected_owner = DEAD_TASK_ID;
    // Locks can be implemented with a single CAS! Note that the current_owner
    // variable starts out with the value we expect to see, and will be set to
    // the actual value of lock_owner in case they don't match - the signatures
    // of GCC's functions for atomic operations sure are weird.
    bool did_lock = __atomic_compare_exchange_n(
      &self->lock_owner, &expected_owner, my_id, /*weak*/ false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED
    );
    if (likely(did_lock)) {
      // The mutex was in unlocked state and has been successfully acquired.
      self->lock_count = 1;
      return true;
    } else if (likely(expected_owner == my_id)) {
      // lock_owner wasn't equal to DEAD_TASK_ID, meaning that the lock was
      // already locked, but expected_owner has been updated with the lock's
      // current owner, and it seems to be us - the mutex has been acquired
      // recursively from the same task.
      self->lock_count += 1;
      ASSERT(self->lock_count != 0); // Counter overflow check
      return true;
    } else {
      // Couldn't acquire the mutex, try again if necessary.
      if (just_try) {
        return false;
      }
      task_wait(&self->notify, NO_DEADLINE);
    }
  }
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
  __atomic_store_n(&self->lock_owner, DEAD_TASK_ID, __ATOMIC_RELEASE);
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
