#pragma once

#include <cmsis_compiler.h>
#include <inttypes.h>
#include <machine/endian.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIZEOF(x) (sizeof((x)) / sizeof((x)[0]))

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define floor_div(a, b) ((a) / (b))
#define ceil_div(a, b) ((a) / (b) + ((a) % (b) != 0))
#define round_div(a, b) ((a) / (b) + ((a) % (b) > (b) / 2))

#define BIT(i) (1u << (i))
#define MASK(i) ((1u << (i)) - 1)
#define EXTRACT_BITS(val, pos, len) (((val) >> (pos)) & ((1 << (len)) - 1))

// clang-format off
#define BITS4(a,b,c,d) ((a)<<3 | (b)<<2 | (c)<<1 | (d)<<0)
#define BITS8(a,b,c,d,e,f,g,h) ((a)<<7 | (b)<<6 | (c)<<5 | (d)<<4 | (e)<<3 | (f)<<2 | (g)<<1 | (h)<<0)
// clang-format on

// Poor man's loop unroller macros for fixed repeat counts that just copy-paste
// the given code. Since the provided statement may contain commas, a variadic
// macro parameter must be used to capture it because otherwise the
// preprocessor will split the statement into multiple "arguments".
// clang-format off
#define UNROLL_2(...) {__VA_ARGS__;__VA_ARGS__;}
#define UNROLL_4(...) {__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;}
#define UNROLL_8(...) {__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;}
#define UNROLL_10(...) {__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;}
#define UNROLL_16(...) {__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;__VA_ARGS__;}
// clang-format on

#ifdef __GNUC__
#define GCC_ATTRIBUTE(expr) __attribute__((expr))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define GCC_ATTRIBUTE(expr)
#define likely(x) (x)
#define unlikely(x) (x)
#endif

#ifdef __clang__
#define CLANG_ATTRIBUTE(expr) __attribute__((expr))
#else
#define CLANG_ATTRIBUTE(expr)
#endif

#define __WEAK_ALIAS(name) __attribute__((weak, alias(name)))
#define __ALIAS(name) __attribute__((alias(name)))
#define __SECTION(name) __attribute__((section(name)))
#define __NAKED __attribute__((naked))
#define __UNUSED __attribute__((unused))

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
void fast_memset_u16(u16* dst, u16 val, usize n);
void fast_memset_u32(u32* dst, u32 val, usize n);
void fast_memcpy_u8(u8* dst, const u8* src, usize n);
void fast_memcpy_u16(u16* dst, const u16* src, usize n);
void fast_memcpy_u32(u32* dst, const u32* src, usize n);

// <https://stackoverflow.com/q/19965076/12005228>
#define COMPILER_MEMORY_BARRIER() __ASM volatile("" ::: "memory")
#define WAIT_FOR_EVENT() __WFE()
#define WAIT_FOR_INTERRUPT() __WFI()

#define BITBAND_ADDR(base, bb_base, addr, bit) \
  ((volatile u32*)(bb_base + 32ul * ((usize)(addr) - (base)) + 4ul * (bit)))
#define SRAM1_BITBAND_ADDR(addr, bit) BITBAND_ADDR(SRAM1_BASE, SRAM1_BB_BASE, addr, bit)
#define PERIPH_BITBAND_ADDR(addr, bit) BITBAND_ADDR(PERIPH_BASE, PERIPH_BB_BASE, addr, bit)

#define interrupt_number() (__get_IPSR())
#define in_interrupt_handler() (__get_IPSR() != 0)

i32 humanize_units(char* buf, usize buf_size, i64 value);
i32 humanize_bytes(char* buf, usize buf_size, i64 bytes);

#define u32_swap_bytes(x) __builtin_bswap32(x)
#if BYTE_ORDER == LITTLE_ENDIAN
#define u32_from_le(x) ((u32)(x))
#define u32_from_be(x) __builtin_bswap32(x)
#define u32_to_le(x) ((u32)(x))
#define u32_to_be(x) __builtin_bswap32(x)
#elif BYTE_ORDER == BIG_ENDIAN
#define u32_from_le(x) __builtin_bswap32(x)
#define u32_from_be(x) ((u32)(x))
#define u32_to_le(x) __builtin_bswap32(x)
#define u32_to_be(x) ((u32)(x))
#endif

#define is_power_of_two(x) (((x) & ((x)-1)) == 0)
#define is_aligned(x, align) (((x) & ((align)-1)) == 0)
#define align_to(x, align) ((x) & ~((align)-1))
#define test_bit(x, bit) (((x) & (bit)) != 0)
#define test_any_bit(x, bit) (((x) & (bit)) != 0)
#define test_all_bits(x, bit) (((x) & (bit)) == bit)
#define set_bit(x, bit) ((x) | (bit))
#define clear_bit(x, bit) ((x) & ~(bit))
#define read_bit(x, bit) ((x) & (reg))
#define toggle_bit(x, bit) ((x) ^ (bit))

#ifdef __cplusplus
}
#endif
