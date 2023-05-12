#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
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

#define check_hal_error(expr) ((expr) != HAL_OK) ? Error_Handler() : ((void)0)

void fast_memset_u32(u32* dst, u32 val, usize n);
void fast_memcpy_u8(u8* dst, const u8* src, usize n);

#ifdef __cplusplus
}
#endif
