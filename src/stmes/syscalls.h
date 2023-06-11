#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void* _sbrk(ptrdiff_t incr);
int _write(int fd, const char* buf, size_t len);

#ifdef __cplusplus
}
#endif
