// System call subroutines for tying Newlib with the rest of the system.
// <https://sourceware.org/newlib/libc.html#Syscalls>
// <https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib>

#include "stmes/newlib_support.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/sync.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include <errno.h>
#include <malloc.h>
#include <printf.h>
#include <sys/lock.h>
#include <unistd.h>

__USED void* _sbrk(ptrdiff_t incr) {
  // These three are defined by the linker script
  extern u32 _end, _estack, _Min_Stack_Size;

  const usize stack_limit = (usize)&_estack - (usize)&_Min_Stack_Size;
  const u8* max_heap = (u8*)stack_limit;
  u8* prev_heap_end;

  static u8* heap_end = NULL;
  if (heap_end == NULL) {
    heap_end = (u8*)&_end;
  }

  // Prevent the heap from colliding with the stack
  if (heap_end + incr > max_heap) {
    errno = ENOMEM;
    return (void*)-1;
  }

  prev_heap_end = heap_end;
  heap_end += incr;
  return (void*)prev_heap_end;
}

int _write(int fd, const char* buf, size_t len) {
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    for (const char* end = buf + len; buf != end; buf++) {
      console_putchar(*buf);
    }
    return len;
  } else {
    errno = EBADF;
    return -1;
  }
}

void _putchar(char c) {
  console_putchar(c);
}

int _read(int fd, char* ptr, size_t len) {
  UNUSED(fd), UNUSED(ptr), UNUSED(len);
  errno = EBADF;
  return -1;
}

int _close(int fd) {
  UNUSED(fd);
  errno = EBADF;
  return -1;
}

off_t _lseek(int fd, off_t offset, int whence) {
  UNUSED(fd), UNUSED(offset), UNUSED(whence);
  errno = EBADF;
  return 0;
}

// Newlib has a facility for locking under a multi-threaded system, which
// simply requires providing all locking-related subroutines to override the
// default empty stubs. Note that replacements for every symbol must be
// provided, or else linking will fail.
// <https://sourceware.org/newlib/libc.html#g_t_005f_005fretarget_005flock_005finit>
// <https://gist.github.com/thomask77/65591d78070ace68885d3fd05cdebe3a>
// <https://nadler.com/embedded/newlibAndFreeRTOS.html>
#if _RETARGETABLE_LOCKING

// From Newlib's point of view, this is an opaque type that we can redefine.
struct __lock {
  struct Mutex mutex;
};

struct __lock __lock___sinit_recursive_mutex = { MUTEX_INIT };
struct __lock __lock___sfp_recursive_mutex = { MUTEX_INIT };
struct __lock __lock___atexit_recursive_mutex = { MUTEX_INIT };
struct __lock __lock___at_quick_exit_mutex = { MUTEX_INIT };
struct __lock __lock___malloc_recursive_mutex = { MUTEX_INIT };
struct __lock __lock___env_recursive_mutex = { MUTEX_INIT };
struct __lock __lock___tz_mutex = { MUTEX_INIT };
struct __lock __lock___dd_hash_mutex = { MUTEX_INIT };
struct __lock __lock___arc4random_mutex = { MUTEX_INIT };

// The lock/unlock functions specifically used by malloc are overridden in
// order to catch dynamic memory allocations inside interrupts.
void __malloc_lock(struct _reent* reent) {
  UNUSED(reent);
  ASSERT(!in_interrupt_handler());
  mutex_lock(&__lock___malloc_recursive_mutex.mutex);
}

void __malloc_unlock(struct _reent* reent) {
  UNUSED(reent);
  ASSERT(!in_interrupt_handler());
  mutex_unlock(&__lock___malloc_recursive_mutex.mutex);
}

void __retarget_lock_init(_LOCK_T* lock_ptr) {
  _LOCK_T lock = malloc(sizeof(*lock));
  mutex_init(&lock->mutex);
  *lock_ptr = lock;
}

void __retarget_lock_init_recursive(_LOCK_T* lock_ptr) {
  _LOCK_T lock = malloc(sizeof(*lock));
  mutex_init(&lock->mutex);
  *lock_ptr = lock;
}

void __retarget_lock_close(_LOCK_T lock) {
  free(lock);
}

void __retarget_lock_close_recursive(_LOCK_T lock) {
  free(lock);
}

void __retarget_lock_acquire(_LOCK_T lock) {
  mutex_lock(&lock->mutex);
}

void __retarget_lock_acquire_recursive(_LOCK_T lock) {
  mutex_lock(&lock->mutex);
}

int __retarget_lock_try_acquire(_LOCK_T lock) {
  return mutex_try_lock(&lock->mutex);
}

int __retarget_lock_try_acquire_recursive(_LOCK_T lock) {
  return mutex_try_lock(&lock->mutex);
}

void __retarget_lock_release(_LOCK_T lock) {
  mutex_unlock(&lock->mutex);
}

void __retarget_lock_release_recursive(_LOCK_T lock) {
  mutex_unlock(&lock->mutex);
}

#endif
