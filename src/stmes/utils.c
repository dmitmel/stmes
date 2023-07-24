#include "stmes/utils.h"

void fast_memset_u8(u8* dst, u8 val, usize n) {
  u8* end = dst;
  for (end += n - n % 4; dst != end;) {
    UNROLL_4(*dst++ = val);
  }
  for (end += n % 4; dst != end;) {
    *dst++ = val;
  }
}

void fast_memset_u16(u16* dst, u16 val, usize n) {
  u16* end = dst;
  for (end += n - n % 4; dst != end;) {
    UNROLL_4(*dst++ = val);
  }
  for (end += n % 4; dst != end;) {
    *dst++ = val;
  }
}

void fast_memset_u32(u32* dst, u32 val, usize n) {
  u32* end = dst;
  for (end += n - n % 4; dst != end;) {
    UNROLL_4(*dst++ = val);
  }
  for (end += n % 4; dst != end;) {
    *dst++ = val;
  }
}

void fast_memcpy_u8(u8* dst, const u8* src, usize n) {
  u8* end = dst;
  for (end += n - n % 4; dst != end;) {
    UNROLL_4(*dst++ = *src++);
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}

void fast_memcpy_u16(u16* dst, const u16* src, usize n) {
  u16* end = dst;
  for (end += n - n % 4; dst != end;) {
    UNROLL_4(*dst++ = *src++);
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}

void fast_memcpy_u32(u32* dst, const u32* src, usize n) {
  u32* end = dst;
  for (end += n - n % 4; dst != end;) {
    UNROLL_4(*dst++ = *src++);
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}
