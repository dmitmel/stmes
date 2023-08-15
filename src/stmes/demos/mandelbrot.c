#include "stmes/demos.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/framebuf.h"
#include "stmes/video/vga.h"
#include <math.h>
#include <printf.h>

static u8 mandelbrot_framebuf[FRAME_HEIGHT][FRAME_WIDTH];
static VgaPixel mandelbrot_palette[1 << 8];
static struct MandelbrotRect {
  u16 x, y, w, h;
} mandelbrot_rect;

static u8 render_task_stack[1024] __ALIGNED(8);
static struct Task render_task;

static u8 mandelbrot_task_stack[1024] __ALIGNED(8);
static struct Task mandelbrot_task;

static u32 hsv2rgb(float hsv[3]) {
  float h = hsv[0], s = hsv[1], v = hsv[2];
  float r = 0, g = 0, b = 0;
  if (s == 0) {
    r = v;
    g = v;
    b = v;
  } else {
    int i = (int)truncf(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - (s * f));
    float t = v * (1 - (s * (1 - f)));
    switch (i % 6) {
      case 0: r = v, g = t, b = p; break;
      case 1: r = q, g = v, b = p; break;
      case 2: r = p, g = v, b = t; break;
      case 3: r = p, g = q, b = v; break;
      case 4: r = t, g = p, b = v; break;
      case 5: r = v, g = p, b = q; break;
    }
  }
  return ((u32)(r * 0xFF) << 16) | ((u32)(g * 0xFF) << 8) | ((u32)(b * 0xFF) << 0);
}

__STATIC_INLINE float lerpf(float a, float b, float t) {
  // <https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/>
  // Complies down to just two instructions: vfms + vfma.
  return fmaf(t, b, fmaf(-t, a, a));
}

__STATIC_INLINE float logerpf(float a, float b, float t) {
  return a * powf(b / a, t);
}

__STATIC_INLINE float mapf(float x, float a_start, float a_end, float b_start, float b_end) {
  return (x - a_start) / (a_end - a_start) * (b_end - b_start) + b_start;
}

__STATIC_INLINE float clampf(float x, float min, float max) {
  return fmaxf(min, fminf(max, x));
}

static u32 lerp_rgb(u32 color1, u32 color2, float t) {
  u8 r = (u8)lerpf((u8)(color1 >> 16), (u8)(color2 >> 16), t);
  u8 g = (u8)lerpf((u8)(color1 >> 8), (u8)(color2 >> 8), t);
  u8 b = (u8)lerpf((u8)(color1 >> 0), (u8)(color2 >> 0), t);
  return (r << 16) | (g << 8) | (b << 0);
}

static void mandelbrot_task_fn(__UNUSED void* user_data) {
  for (u32 y = 0; y < FRAME_HEIGHT; y++) {
    fast_memset_u8(mandelbrot_framebuf[y], UINT8_MAX, FRAME_WIDTH);
  }

  for (usize i = 0; i < SIZEOF(mandelbrot_palette); i++) {
    float x = (float)i / (int)(SIZEOF(mandelbrot_palette) - 1);
    float hsv[3] = {
      mapf(x, 0, 1, 2.0f / 3.0f, 0),
      1,
      clampf(mapf(x, 0, 1, 0.2f, 2.0f), 0, 1),
    };
    mandelbrot_palette[i] = rgb12_to_vga_pins(rgb24_to_rgb12(hsv2rgb(hsv)));
  }
  mandelbrot_palette[UINT8_MAX] = rgb12_to_vga_pins(0x000);

  task_sleep(2000);

  float camera_zoom = 1, camera_x = 0, camera_y = 0;

  const float TARGET_ZOOM = 7.382749e4f;
  const float TARGET_X = -1.2714951f;
  const float TARGET_Y = 0.056114505f;
  const float ZOOM_SPEED = 1;

  while (true) {
    console_clear_screen();
    printf("x=%10.07f y=%10.07f z=%.7e", (double)camera_x, (double)camera_y, (double)camera_zoom);

    const float RADIUS = 2;
    const u32 MAX_ITERATIONS = 400;
    const float EPSILON = 1e-4f;
    const u32 MAX_PERIOD = 20;
    const float UNIT_SIZE = 64.0f;
    const float DEEP_ZOOM_COEFF = 1 / 50.0f;

    const float screen_size_x = (int)FRAME_WIDTH;
    const float screen_size_y = (int)FRAME_HEIGHT;

    for (u32 screen_y = 0; screen_y < FRAME_HEIGHT; screen_y++) {
      for (u32 screen_x = 0; screen_x < FRAME_WIDTH; screen_x++) {
        float world_x = camera_x + (screen_x + 0.5f - screen_size_x / 2) / UNIT_SIZE / camera_zoom;
        float world_y = camera_y + (screen_y + 0.5f - screen_size_y / 2) / UNIT_SIZE / camera_zoom;

        float cx = world_x, cy = world_y;
        float zx = 0, zy = 0;

        float period_x = 0, period_y = 0;
        u32 period = 0;

        u32 iter = 0;
        while (zx * zx + zy * zy < RADIUS * RADIUS && iter < MAX_ITERATIONS) {
          float x = zx * zx - zy * zy + cx, y = 2 * zx * zy + cy;
          zx = x, zy = y;
          iter += 1;

          if (fabsf(period_x - x) < EPSILON && fabsf(period_y - y) < EPSILON) {
            iter = MAX_ITERATIONS;
            break;
          }
          period++;
          if (period > MAX_PERIOD) {
            period = 0, period_x = x, period_y = y;
          }
        }

        u8* pixel = &mandelbrot_framebuf[screen_y][screen_x];
        if (iter >= MAX_ITERATIONS) {
          *pixel = UINT8_MAX;
          continue;
        }

        float nu = log2f(log2f(zx * zx + zy * zy) / 2); /* == log2f(log2f(sqrtf(...))) */
        float smooth = iter + 1 - nu;
        smooth = (1 - cosf(smooth * DEEP_ZOOM_COEFF * (float)M_PI)) / 2;
        *pixel = (u8)roundf(smooth * UINT8_MAX);

        // smooth = (1 - cosf(smooth / (float)MAX_ITERATIONS * 10.0f * (float)M_PI)) / 2;
        // float iter_trunc;
        // float iter_fract = modff(smooth * MAX_ITERATIONS, &iter_trunc);
        // iter = (u32)iter_trunc;
        // iter = MIN(iter, SIZEOF(palette) - 1);
        // // ASSERT((u32)iter_trunc + 1 < SIZEOF(palette));
        // u32 rgb24 = lerp_rgb(palette[(u32)iter_trunc], palette[(u32)iter_trunc + 1], iter_fract);
        // *pixel = rgb12_to_vga_pins(rgb24_to_rgb12(rgb24));
      }
    }

    task_sleep(1500);

    float next_camera_zoom = camera_zoom * (1 + logf(TARGET_ZOOM / camera_zoom) / ZOOM_SPEED);
    float next_camera_x = camera_x - TARGET_X * (1 / next_camera_zoom - 1 / camera_zoom);
    float next_camera_y = camera_y - TARGET_Y * (1 / next_camera_zoom - 1 / camera_zoom);

    struct MandelbrotRect* rect = &mandelbrot_rect;
    for (u32 i = 1, max = 5; i <= max; i++) {
      float factor = i / (float)max;
      float new_camera_zoom = logerpf(camera_zoom, next_camera_zoom, factor);
      float new_camera_x = camera_x - TARGET_X * (1 / new_camera_zoom - 1 / camera_zoom);
      float new_camera_y = camera_y - TARGET_Y * (1 / new_camera_zoom - 1 / camera_zoom);

      float viewport_x = screen_size_x / UNIT_SIZE / new_camera_zoom;
      float viewport_y = screen_size_y / UNIT_SIZE / new_camera_zoom;
      float x1 = new_camera_x - viewport_x / 2, y1 = new_camera_y - viewport_y / 2;
      float x2 = new_camera_x + viewport_x / 2, y2 = new_camera_y + viewport_y / 2;

      i32 screen_x = (i32)((x1 - camera_x) * UNIT_SIZE * camera_zoom + screen_size_x / 2);
      i32 screen_y = (i32)((y1 - camera_y) * UNIT_SIZE * camera_zoom + screen_size_y / 2);
      i32 size_x = (i32)((x2 - x1) * UNIT_SIZE * camera_zoom);
      i32 size_y = (i32)((y2 - y1) * UNIT_SIZE * camera_zoom);

      bool render = (                                      //
        0 <= screen_x && screen_x < FRAME_WIDTH &&         //
        0 <= screen_y && screen_y < FRAME_HEIGHT &&        //
        0 <= size_x && size_x + screen_x <= FRAME_WIDTH && //
        0 <= size_y && size_y + screen_y <= FRAME_HEIGHT   //
      );
      if (render) {
        rect->x = screen_x, rect->y = screen_y, rect->w = size_x, rect->h = size_y;
      } else {
        rect->x = rect->y = rect->w = rect->h = 0;
      }
      task_sleep(200);
    }

    task_sleep(500);
    rect->x = rect->y = rect->w = rect->h = 0;

    camera_zoom = next_camera_zoom, camera_x = next_camera_x, camera_y = next_camera_y;
  }
}

static void render_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_wait(&vga_notification, NO_DEADLINE);

    if (vga_control.next_scanline_requested) {
      u16 vga_line = vga_control.next_scanline_nr;
      vga_control.next_scanline_requested = false;

      u16 y = vga_line / PIXEL_SCALE;
      // ASSERT(y < MANDELBROT_FRAME_HEIGHT);
      if (y < CONSOLE_LINE_HEIGHT) {
        console_render_scanline(vga_line);
      } else if (y < FRAME_HEIGHT) {
        struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
        vga_set_next_scanline(backbuf->data);

        u8* row = mandelbrot_framebuf[y];
        VgaPixel* pixel_ptr = &backbuf->data[0];
        VgaPixel* palette = mandelbrot_palette;
        for (u8* row_end = row + FRAME_WIDTH; row != row_end; row += 8, pixel_ptr += 8) {
          u32 quad1 = ((u32*)row)[0], quad2 = ((u32*)row)[1];
          u32 a = quad1 & 0xFF, b = (quad1 >> 8) & 0xFF, c = (quad1 >> 16) & 0xFF, d = quad1 >> 24;
          a = palette[a], b = palette[b], c = palette[c], d = palette[d];
          pixel_ptr[0] = a, pixel_ptr[1] = b, pixel_ptr[2] = c, pixel_ptr[3] = d;
          u32 e = quad2 & 0xFF, f = (quad2 >> 8) & 0xFF, g = (quad2 >> 16) & 0xFF, h = quad2 >> 24;
          e = palette[e], f = palette[f], g = palette[g], h = palette[h];
          pixel_ptr[4] = e, pixel_ptr[5] = f, pixel_ptr[6] = g, pixel_ptr[7] = h;
        }

        struct MandelbrotRect* rect = &mandelbrot_rect;
        if (rect->w > 0 && rect->h > 0) {
          const u32 red = rgb12_to_vga_pins(0xF00);
          if (y == rect->y || y == rect->y + rect->h - 1) {
            vga_fast_memset(backbuf->data + rect->x, red, rect->w);
          } else if (rect->y < y && y < rect->y + rect->h) {
            backbuf->data[rect->x] = backbuf->data[rect->x + rect->w - 1] = red;
          }
        }
      }
    }

    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      struct VgaFrameConfig frame = {
        .line_length = FRAME_WIDTH,
        .lines_count = FRAME_HEIGHT * PIXEL_SCALE,
        .pixel_scale = PIXEL_SCALE - 1,
        .line_repeats = PIXEL_SCALE - 1,
      };
      vga_set_frame_config(&frame);
    }
  }
}

void mandelbrot_demo(void) {
  struct TaskParams render_task_params = {
    .stack_start = render_task_stack,
    .stack_size = sizeof(render_task_stack),
    .func = &render_task_fn,
  };
  task_spawn(&render_task, &render_task_params);

  struct TaskParams mandelbrot_task_params = {
    .stack_start = mandelbrot_task_stack,
    .stack_size = sizeof(mandelbrot_task_stack),
    .func = &mandelbrot_task_fn,
  };
  task_spawn(&mandelbrot_task, &mandelbrot_task_params);
}
