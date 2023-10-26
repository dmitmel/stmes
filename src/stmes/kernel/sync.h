#pragma once

#include "stmes/kernel/task.h"
#include "stmes/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: deadlines

struct Mutex {
  volatile TaskId lock_owner;
  u16 lock_count; // Using a 32-bit field here will create more padding
  struct Notification notify;
};

void mutex_init(struct Mutex* self);
void mutex_lock(struct Mutex* self);
bool mutex_try_lock(struct Mutex* self);
void mutex_unlock(struct Mutex* self);

enum __packed ChannelState {
  CHANNEL_IDLE = 0,
  CHANNEL_SENDING_MESSAGE,
  CHANNEL_MESSAGE_SENT,
  CHANNEL_RECEIVING_MESSAGE,
  CHANNEL_MESSAGE_RECEIVED,
};

struct Channel {
  volatile enum ChannelState state;
  struct Notification notify;
  void* message;
  usize message_size;
};

void channel_init(struct Channel* self);
void channel_send(struct Channel* self, void* message, usize size);
void channel_recv(struct Channel* self, void* message, usize size);

#ifdef __cplusplus
}
#endif
