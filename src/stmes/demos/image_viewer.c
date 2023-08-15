#include "stmes/demos.h"
#include "stmes/fatfs.h"
#include "stmes/sdio.h"
#include "stmes/utils.h"
#include "stmes/video/framebuf.h"
#include "stmes/video/vga.h"
#include <ff.h>
#include <stdlib.h>
#include <stm32f4xx_hal.h>

void image_viewer_demo(void) {
  static FATFS SDFatFS;
  static FIL SDFile;

  while (BSP_SD_Init() != HAL_OK) {
    HAL_Delay(500);
  }

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_open(&SDFile, "penguins_palette.bin", FA_READ));

  struct __PACKED image_header {
    u16 width;
    u16 height;
    u16 palette_len;
    u32 pixels_len;
  } image_header = { 0 };
  UINT bytes_read = 0;
  check_fs_error(f_read(&SDFile, &image_header, sizeof(image_header), &bytes_read));
  u16 image_width = image_header.width;
  u16 image_height = image_header.height;

  FSIZE_t file_ptr = 0;
  file_ptr += sizeof(image_header);
  FSIZE_t image_palette_start = file_ptr;
  FSIZE_t image_palette_size = sizeof(u8) * image_header.palette_len;
  file_ptr += image_palette_size;
  FSIZE_t image_pixels_start = file_ptr;
  FSIZE_t image_pixels_size = sizeof(u8) * image_header.pixels_len;
  file_ptr += image_pixels_size;
  ASSERT(file_ptr == f_size(&SDFile));

  u8* image_palette = malloc(image_palette_size);
  check_fs_error(f_lseek(&SDFile, image_palette_start));
  check_fs_error(f_read(&SDFile, image_palette, image_palette_size, &bytes_read));

  u8* image_pixels = malloc(image_pixels_size);
  check_fs_error(f_lseek(&SDFile, image_pixels_start));
  check_fs_error(f_read(&SDFile, image_pixels, image_pixels_size, &bytes_read));

  static u32 video_palette[1 << 8];
  for (usize i = 0; i < MIN(image_header.palette_len, SIZEOF(video_palette) * 3); i += 3) {
    u8 red = image_palette[i], green = image_palette[i + 1], blue = image_palette[i + 2];
    red >>= 4, green >>= 4, blue >>= 4;
    video_palette[i / 3] = rgb12_to_vga_pins((red << 8) | (green << 4) | blue);
  }

  while (true) {
    if (unlikely(vga_control.entering_vblank)) {
      vga_control.entering_vblank = false;
      struct VgaFrameConfig frame = {
        .line_length = FRAME_WIDTH,
        .lines_count = FRAME_HEIGHT * PIXEL_SCALE,
        .pixel_scale = PIXEL_SCALE - 1,
        .line_repeats = PIXEL_SCALE - 1,
      };
      vga_set_frame_config(&frame);
    }

    u16 vga_line = 0;
    if (unlikely(vga_take_scanline_request(&vga_line))) {
      struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
      vga_set_next_scanline(backbuf->data);

      backbuf->data[0] = backbuf->data[FRAME_WIDTH - 1] = VGA_PIXEL_ALL_PINS_RESET;

      u32 video_y = vga_line / PIXEL_SCALE - (FRAME_HEIGHT - image_height) / 2;
      // video_y = (video_y + frame_counter) % video_height;
      if (video_y < image_height) {
        usize pixel_idx = (FRAME_WIDTH - image_width) / 2;
        u8* row = &image_pixels[video_y * image_width];
        VgaPixel* pixel_ptr = &backbuf->data[pixel_idx];
        for (u8* row_end = row + image_width; row != row_end; row += 8, pixel_ptr += 8) {
          u32 quad1 = ((u32*)row)[0], quad2 = ((u32*)row)[1];
          u32 byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8;
          byte1 = quad1 & 0xFF, byte2 = (quad1 >> 8) & 0xFF;
          byte3 = (quad1 >> 16) & 0xFF, byte4 = quad1 >> 24;
          byte5 = quad2 & 0xFF, byte6 = (quad2 >> 8) & 0xFF;
          byte7 = (quad2 >> 16) & 0xFF, byte8 = quad2 >> 24;
          byte1 = row[0], byte2 = row[1], byte3 = row[2], byte4 = row[3];
          byte5 = row[4], byte6 = row[5], byte7 = row[6], byte8 = row[7];
          byte1 = video_palette[byte1], byte2 = video_palette[byte2];
          byte3 = video_palette[byte3], byte4 = video_palette[byte4];
          byte5 = video_palette[byte5], byte6 = video_palette[byte6];
          byte7 = video_palette[byte7], byte8 = video_palette[byte8];
          pixel_ptr[0] = byte1, pixel_ptr[1] = byte2, pixel_ptr[2] = byte3, pixel_ptr[3] = byte4;
          pixel_ptr[4] = byte5, pixel_ptr[5] = byte6, pixel_ptr[6] = byte7, pixel_ptr[7] = byte8;
        }
        pixel_idx = pixel_ptr - &backbuf->data[0];
        if (pixel_idx < FRAME_WIDTH) {
          backbuf->data[pixel_idx] = VGA_PIXEL_ALL_PINS_RESET;
        }
      }
    }

    WAIT_FOR_INTERRUPT();
  }
}
