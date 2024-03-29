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

#ifndef __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#endif

// Define fallbacks for _init/_fini as weak symbols, so that linking doesn't
// fail in case the crt*.o objects are missing.
__WEAK void _init(void) {}
__WEAK void _fini(void) {}

// Replace exception personality routines with empty stubs to avoid linking the
// unwinding internals from libgcc, which are pretty large.
void __aeabi_unwind_cpp_pr0(void) {
  __builtin_trap();
}
void __aeabi_unwind_cpp_pr1(void) {
  __builtin_trap();
}
void __aeabi_unwind_cpp_pr2(void) {
  __builtin_trap();
}

void* _sbrk(ptrdiff_t incr) {
  extern u32 __heap_start[], __heap_end[]; // These are defined by the linker script
  // Static initializers are allowed to reference addresses of other symbols
  static usize heap_ptr = (usize)__heap_start;
  usize new_heap_ptr = heap_ptr + incr;
  if ((usize)__heap_start <= new_heap_ptr && new_heap_ptr <= (usize)__heap_end) {
    usize prev_heap_ptr = heap_ptr;
    heap_ptr = new_heap_ptr;
    return (void*)prev_heap_ptr;
  } else {
    errno = ENOMEM;
    return (void*)-1;
  }
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

void _exit(int code) {
  UNUSED(code);
  __builtin_trap();
}

#ifndef __clang__
#pragma GCC diagnostic pop
#endif

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
