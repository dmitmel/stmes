#pragma once

#include <cmsis_compiler.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIZEOF(x) (sizeof((x)) / sizeof(*(x)))

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define floor_div(a, b) ((a) / (b))
#define ceil_div(a, b) ((a) / (b) + ((a) % (b) != 0))
#define round_div(a, b) ((a) / (b) + ((a) % (b) > (b) / 2))

// clang-format off
#define UNROLL_2(x) {x;x;}
#define UNROLL_4(x) {x;x;x;x;}
#define UNROLL_8(x) {x;x;x;x;x;x;x;x;}
#define UNROLL_10(x) {x;x;x;x;x;x;x;x;x;x;}
#define UNROLL_16(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;}
// clang-format on

#ifdef __GNUC__
#define GCC_ATTRIBUTE(expr) __attribute__((expr))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#define STATIC_ASSERT(cond) _Static_assert(cond, #cond)
#else
#define GCC_ATTRIBUTE(expr)
#define likely(x) (x)
#define unlikely(x) (x)
#define STATIC_ASSERT(cond)
#endif

typedef int8_t i8;
typedef uint8_t u8;
typedef int16_t i16;
typedef uint16_t u16;
typedef int32_t i32;
typedef uint32_t u32;
typedef int64_t i64;
typedef uint64_t u64;
typedef intptr_t isize;
typedef uintptr_t usize;
typedef float f32;
typedef double f64;

void fast_memset_u8(u8* dst, u8 val, usize n);
void fast_memset_u32(u32* dst, u32 val, usize n);
void fast_memcpy_u8(u8* dst, const u8* src, usize n);
void fast_memcpy_u32(u32* dst, const u32* src, usize n);

// <https://stackoverflow.com/q/19965076/12005228>
#define __MEMORY_BARRIER() __ASM volatile("" ::: "memory")
#define __WAIT_FOR_EVENT() __WFE()
#define __WAIT_FOR_INTERRUPT() __WFI()

#define __ldmia2(ptr, r1, r2, a, b)                                                              \
  do {                                                                                           \
    register u32 __a __ASM(r1);                                                                  \
    register u32 __b __ASM(r2);                                                                  \
    register u32* __ptr = *(ptr);                                                                \
    __ASM("ldmia   %0!, {%1, %2}" : "+r"(__ptr), "=r"(__a), "=r"(__b) : "m"(*(u32(*)[2])__ptr)); \
    *(ptr) = __ptr, *(a) = __a, *(b) = __b;                                                      \
  } while (0)

#define __ldmia4(ptr, r1, r2, r3, r4, a, b, c, d)                   \
  do {                                                              \
    register u32 __a __ASM(r1);                                     \
    register u32 __b __ASM(r2);                                     \
    register u32 __c __ASM(r3);                                     \
    register u32 __d __ASM(r4);                                     \
    register u32* __ptr = *(ptr);                                   \
    __ASM("ldmia   %0!, {%1, %2, %3, %4}"                           \
          : "+r"(__ptr), "=r"(__a), "=r"(__b), "=r"(__c), "=r"(__d) \
          : "m"(*(u32(*)[4])__ptr));                                \
    *(ptr) = __ptr, *(a) = __a, *(b) = __b, *(c) = __c, *(d) = __d; \
  } while (0)

#define __ldmia8(ptr, r1, r2, r3, r4, r5, r6, r7, r8, a, b, c, d, e, f, g, h) \
  do {                                                                        \
    register u32 __a __ASM(r1);                                               \
    register u32 __b __ASM(r2);                                               \
    register u32 __c __ASM(r3);                                               \
    register u32 __d __ASM(r4);                                               \
    register u32 __e __ASM(r5);                                               \
    register u32 __f __ASM(r6);                                               \
    register u32 __g __ASM(r7);                                               \
    register u32 __h __ASM(r8);                                               \
    register u32* __ptr = *(ptr);                                             \
    __ASM("ldmia   %0!, {%1, %2, %3, %4, %5, %6, %7, %8}"                     \
          : "+r"(__ptr),                                                      \
            "=r"(__a),                                                        \
            "=r"(__b),                                                        \
            "=r"(__c),                                                        \
            "=r"(__d),                                                        \
            "=r"(__e),                                                        \
            "=r"(__f),                                                        \
            "=r"(__g),                                                        \
            "=r"(__h)                                                         \
          : "m"(*(u32(*)[8])__ptr));                                          \
    *(ptr) = __ptr;                                                           \
    *(a) = __a, *(b) = __b, *(c) = __c, *(d) = __d;                           \
    *(e) = __e, *(f) = __f, *(g) = __g, *(h) = __h;                           \
  } while (0)

#define __stmia4(ptr, r1, r2, r3, r4, a, b, c, d)    \
  do {                                               \
    register u32 __a __ASM(r1) = (a);                \
    register u32 __b __ASM(r2) = (b);                \
    register u32 __c __ASM(r3) = (c);                \
    register u32 __d __ASM(r4) = (d);                \
    register u32* __ptr = *(ptr);                    \
    __ASM("stmia   %1!, {%2, %3, %4, %5}"            \
          : "=m"(*(u32(*)[4])__ptr), "+r"(__ptr)     \
          : "r"(__a), "r"(__b), "r"(__c), "r"(__d)); \
    *(ptr) = __ptr;                                  \
  } while (0)

#define BITBAND_ADDR(base, bb_base, addr, bit) \
  ((volatile u32*)(bb_base + 32ul * ((usize)(addr) - (base)) + 4ul * (bit)))
#define SRAM1_BITBAND_ADDR(addr, bit) BITBAND_ADDR(SRAM1_BASE, SRAM1_BB_BASE, addr, bit)
#define PERIPH_BITBAND_ADDR(addr, bit) BITBAND_ADDR(PERIPH_BASE, PERIPH_BB_BASE, addr, bit)

#ifdef __cplusplus
}
#endif
