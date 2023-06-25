#include "syscalls.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include <errno.h>
#include <printf.h>
#include <unistd.h>

__USED void* _sbrk(ptrdiff_t incr) {
  // These three are defined by the linker script
  extern u8 _end, _estack, _Min_Stack_Size;

  const usize stack_limit = (usize)&_estack - (usize)&_Min_Stack_Size;
  const u8* max_heap = (u8*)stack_limit;
  u8* prev_heap_end;

  static u8* heap_end = NULL;
  if (heap_end == NULL) {
    heap_end = &_end;
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
