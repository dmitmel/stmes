#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void* _sbrk(ptrdiff_t incr);

#ifdef __cplusplus
}
#endif
