#include "stmes/utils.h"
#include <printf.h>
#include <stdlib.h>

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
    u8 a = *src++, b = *src++, c = *src++, d = *src++;
    *dst++ = a, *dst++ = b, *dst++ = c, *dst++ = d;
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}

void fast_memcpy_u16(u16* dst, const u16* src, usize n) {
  u16* end = dst;
  for (end += n - n % 4; dst != end;) {
    u16 a = *src++, b = *src++, c = *src++, d = *src++;
    *dst++ = a, *dst++ = b, *dst++ = c, *dst++ = d;
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}

void fast_memcpy_u32(u32* dst, const u32* src, usize n) {
  u32* end = dst;
  for (end += n - n % 4; dst != end;) {
    u32 a = *src++, b = *src++, c = *src++, d = *src++;
    *dst++ = a, *dst++ = b, *dst++ = c, *dst++ = d;
  }
  for (end += n % 4; dst != end;) {
    *dst++ = *src++;
  }
}

// Somewhat based on <https://stackoverflow.com/a/3758880>.
__STATIC_FORCEINLINE i32 humanize_units_impl(char* buf, usize buf_size, i64 value, u32 unit) {
  static const char UNIT_PREFIXES[] = " kMGTPE";
  const usize DECIMAL_PLACES = 2;
  const u32 unit_close_enough = unit * 99 / 100; // 99% of the unit for nicer formatting
  u64 quotient = value == INT64_MIN ? (u64)INT64_MAX : (u64)ABS(value);
  if (quotient < unit_close_enough) {
    return snprintf(buf, buf_size, "%" PRIi32, (i32)value);
  }
  u32 remainder = 0;
  const char* prefix = UNIT_PREFIXES;
  for (; *prefix != '\0' && quotient >= unit_close_enough; prefix++) {
    remainder = quotient % unit, quotient /= unit;
  }
  u32 decimal_div = 1;
  for (usize i = 0; i < DECIMAL_PLACES; i++) {
    decimal_div *= 10;
  }
  u32 fractional = round_div(remainder * decimal_div, unit);
  quotient += fractional / decimal_div, fractional %= decimal_div;
  return snprintf(
    buf,
    buf_size,
    "%" PRIi32 ".%0*" PRIu32 "%c",
    value < 0 ? -(i32)quotient : (i32)quotient,
    DECIMAL_PLACES,
    fractional,
    *prefix
  );
}

i32 humanize_units(char* buf, usize buf_size, i64 value) {
  return humanize_units_impl(buf, buf_size, value, 1000);
}

i32 humanize_bytes(char* buf, usize buf_size, i64 bytes) {
  return humanize_units_impl(buf, buf_size, bytes, 1024);
}
