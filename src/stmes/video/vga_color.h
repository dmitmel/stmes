#pragma once

#include "stmes/gpio.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef u32 VgaPixel;
#define vga_fast_memset fast_memset_u32
#define vga_fast_memcpy fast_memcpy_u32

// Since the GPIOB pins used for outputting the pixel color for VGA couldn't be
// chosen in a successive order (because PB4 and PB5 are taken up by SDIO data
// lines, and PB11 simply doesn't exist on the MCU chip package), we can't,
// e.g. just write the color's raw RGB value to the port's ODR register.
// Instead, this function establishes a mapping between the RGB colors and the
// pins corresponding to every single bit of the color, and returns a value
// suitable for writing to the BSRR register.
__STATIC_FORCEINLINE VgaPixel rgb12_to_vga_pins(VgaPixel color) {
  // Unfortunately I couldn't find a way to get the compiler to figure out the
  // bit airthmetic for me, so the masks and shifts in this function must be
  // recalculated in case the VGA pixel pins are changed.
  STATIC_ASSERT(VGA_ALL_RED_PINS == 0x7400);
  STATIC_ASSERT(VGA_ALL_GREEN_PINS == 0x03C0);
  STATIC_ASSERT(VGA_ALL_BLUE_PINS == 0x000F);
  VgaPixel pins = 0;
  pins |= (color & 0x00F) << 0; // color[ 3:0] -> pins[ 3:0 ]
  pins |= (color & 0x1F0) << 2; // color[ 8:4] -> pins[10:6 ]
  pins |= (color & 0xE00) << 3; // color[11:9] -> pins[14:12]
  pins |= (pins ^ 0x77CF) << 16;
  return pins;
}

__STATIC_FORCEINLINE u32 rgb24_to_rgb12(u32 rgb) {
  u8 r8 = rgb >> 16, g8 = rgb >> 8, b8 = rgb >> 0;
  u8 r4 = r8 >> 4, g4 = g8 >> 4, b4 = b8 >> 4;
  return (r4 << 8) | (g4 << 4) | (b4 << 0);
}

__STATIC_FORCEINLINE u32 rgb12(u8 r, u8 g, u8 b) {
  return ((r & 0xF) << 8) | ((g & 0xF) << 4) | ((b & 0xF) << 0);
}

__STATIC_FORCEINLINE u32 rgb24(u8 r, u8 g, u8 b) {
  return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF) << 0);
}

#ifdef __cplusplus
}
#endif
