#pragma once

#include <stddef.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

void* _sbrk(ptrdiff_t incr);
int _write(int fd, const char* buf, size_t len);
int _read(int fd, char* buf, size_t len);
off_t _lseek(int fd, off_t offset, int whence);
int _close(int fd);

#ifdef __cplusplus
}
#endif
