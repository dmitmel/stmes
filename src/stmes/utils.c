#include "stmes/utils.h"

void fast_memset_u32(u32* dst, u32 val, usize n) {
  u32* end = dst;
  for (end += n - n % 4; dst != end;) {
    *dst++ = val;
    *dst++ = val;
    *dst++ = val;
    *dst++ = val;
  }
  for (end += n % 4; dst != end;) {
    *dst++ = val;
  }
}

void fast_memcpy_u8(u8* dst, const u8* src, usize n) {
  u8* end = dst;
  for (end += n - n % 4; dst != end;) {
    *dst++ = *src++;
    *dst++ = *src++;
    *dst++ = *src++;
    *dst++ = *src++;
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}
