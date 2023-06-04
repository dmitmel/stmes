#include "syscalls.h"
#include "stmes/utils.h"
#include <errno.h>

__USED void* _sbrk(ptrdiff_t incr) {
  // These three are defined by the linker script
  extern u8 _end;
  extern u8 _estack;
  extern u32 _Min_Stack_Size;

  const u32 stack_limit = (u32)&_estack - (u32)&_Min_Stack_Size;
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
