#include "stmes/demos.h"
#include "stmes/drivers/usb_host.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/sync.h"
#include "stmes/kernel/task.h"
#include "stmes/math.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <math.h>
#include <printf.h>
#include <stm32f4xx_hal.h>
#include <usbh_core.h>
#include <usbh_hid.h>

static u8 game_task_stack[1024] __ALIGNED(8);
static struct Task game_task;

static u8 render_task_stack[1024] __ALIGNED(8);
static struct Task render_task;

static u8 usb_task_stack[1024] __ALIGNED(8);
static struct Task usb_task;

static u8 keyboard_task_stack[1024] __ALIGNED(8);
static struct Task keyboard_task;

#define INPUT_UP BIT(0)
#define INPUT_DOWN BIT(1)
#define INPUT_RIGHT BIT(2)
#define INPUT_LEFT BIT(3)
#define INPUT_ESC BIT(4)

struct Rect {
  float x, y, w, h, vx, vy;
};

static struct GameState {
  struct Mutex lock;
  u32 input_state;
  struct Rect ball, left_racket, right_racket;
} game_state;

static struct Notification frame_notify;

static __attribute__((constructor)) void init_game_state(void) {
  mutex_init(&game_state.lock);
  struct Rect* ball = &game_state.ball;
  struct Rect* left = &game_state.left_racket;
  struct Rect* right = &game_state.right_racket;
  const float frame_w = (u32)FRAME_WIDTH, frame_h = (u32)FRAME_HEIGHT;
  ball->w = ball->h = 8;
  ball->x = (frame_w - ball->w) / 2, ball->y = (frame_h - ball->h) / 2;
  left->w = right->w = 5, left->h = right->h = 50;
  left->y = (frame_h - left->h) / 2, left->x = 10;
  right->y = (frame_h - right->h) / 2, right->x = frame_w - 10 - right->w;
  ball->vy = 1.0f;
  ball->vx = 4.0f;
}

static void game_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_wait(&frame_notify, NO_DEADLINE);
    u32 input_state = __atomic_load_n(&game_state.input_state, __ATOMIC_RELAXED);
    mutex_lock(&game_state.lock);

    struct Rect* ball = &game_state.ball;
    struct Rect* left = &game_state.left_racket;
    struct Rect* right = &game_state.right_racket;
    struct Rect* all_rects[] = { ball, left, right };
    const float frame_w = (u32)FRAME_WIDTH, frame_h = (u32)FRAME_HEIGHT;

    for (usize i = 0; i < SIZEOF(all_rects); i++) {
      struct Rect* rect = all_rects[i];
      rect->x += rect->vx, rect->y += rect->vy;
    }

    const float racket_speed = 150 / 57.0f;
    left->vy = //
      ((input_state & INPUT_DOWN) ? racket_speed : 0) +
      ((input_state & INPUT_UP) ? -racket_speed : 0);
    right->vy = //
      ((input_state & INPUT_RIGHT) ? racket_speed : 0) +
      ((input_state & INPUT_LEFT) ? -racket_speed : 0);
    left->y = clampf(left->y, 0, frame_h - left->h);
    right->y = clampf(right->y, 0, frame_h - right->h);

    if (ball->y <= 0 || ball->y + ball->h > frame_h) {
      ball->vy = -ball->vy;
    }
    if (ball->x <= 0 || ball->x + ball->w > frame_w) {
      ball->vx = -ball->vx;
    }

    if (ball->y >= left->y && ball->y < left->y + left->h && ball->x <= left->x + left->w) {
      ball->vx = -ball->vx;
    }
    if (ball->y >= right->y && ball->y < right->y + right->h && ball->x + ball->w >= right->x) {
      ball->vx = -ball->vx;
    }

    mutex_unlock(&game_state.lock);
  }
}

static float antialias_rect(float top_pos, float bottom_pos, i32 pos) {
  float intensity = 1;
  float pos_floor = floorf(top_pos), pos_ceil = ceilf(bottom_pos);
  if (pos == (i32)pos_floor) {
    intensity = 1 - (top_pos - pos_floor);
  } else if (pos == (i32)pos_ceil) {
    intensity = 1 - (pos_ceil - bottom_pos);
  }
  return intensity;
}

static VgaPixel antialiasing_colors[16];
static __attribute__((constructor)) void init_antialiasing_colors(void) {
  for (usize i = 0; i < SIZEOF(antialiasing_colors); i++) {
    antialiasing_colors[i] = rgb12_to_vga_pins(i * 0x111);
  }
}

static VgaPixel lightness_to_vga_color(float intensity) {
  return antialiasing_colors[(i32)(intensity * 0xF) & 0xF];
}

static void render_rect(VgaPixel* framebuf, struct Rect* rect, i32 frame_y, usize frame_width) {
  float top_edge = rect->y, bottom_edge = rect->y + rect->h - 1;
  float left_edge = rect->x, right_edge = rect->x + rect->w - 1;
  if (!((i32)floorf(top_edge) <= frame_y && frame_y <= (i32)ceilf(bottom_edge))) return;

  i32 start = (i32)floorf(left_edge), end = (i32)ceilf(right_edge);
  start = MAX(0, start), end = MAX(0, end);
  end = MIN(end, (i32)frame_width - 1), start = MIN(start, end + 1);
  i32 length = end - start + 1;
  ASSERT(0 <= start && start <= FRAME_WIDTH);
  ASSERT(0 <= end && end < FRAME_WIDTH);
  ASSERT(0 <= length && length <= FRAME_WIDTH - start);

  if (length < 1) return;
  float vert_color = antialias_rect(top_edge, bottom_edge, frame_y);
  float left_color = antialias_rect(left_edge, right_edge, start);
  float right_color = antialias_rect(left_edge, right_edge, end);
  framebuf[start] = lightness_to_vga_color(fminf(left_color, vert_color));
  if (length < 2) return;
  framebuf[end] = lightness_to_vga_color(fminf(right_color, vert_color));
  if (length < 3) return;
  vga_fast_memset(framebuf + start + 1, lightness_to_vga_color(vert_color), length - 2);
}

static void render_task_fn(__UNUSED void* user_data) {
  while (true) {
    task_wait(&vga_notification, NO_DEADLINE);
    if (vga_control.next_scanline_requested) {
      u16 vga_line = vga_control.next_scanline_nr;
      vga_control.next_scanline_requested = false;
      if ((__atomic_load_n(&game_state.input_state, __ATOMIC_RELAXED) & INPUT_ESC) != 0) {
        console_render_scanline(vga_line);
      } else {
        struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
        u32 y = vga_line / PIXEL_SCALE;
        vga_set_next_scanline(backbuf->data);
        vga_fast_memset(backbuf->data, VGA_PIXEL_ALL_PINS_RESET, FRAME_WIDTH);
        if (y < FRAME_HEIGHT) {
          mutex_lock(&game_state.lock);
          render_rect(backbuf->data, &game_state.ball, y, FRAME_WIDTH);
          render_rect(backbuf->data, &game_state.left_racket, y, FRAME_WIDTH);
          render_rect(backbuf->data, &game_state.right_racket, y, FRAME_WIDTH);
          mutex_unlock(&game_state.lock);
        }
      }
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
      task_notify(&frame_notify);
    }
  }
}

static void usb_task_fn(__UNUSED void* user_data) {
  MX_USB_HOST_Init();
  task_yield();
  while (true) {
    MX_USB_HOST_Process();
    task_wait(&usb_notification, timeout_to_deadline(1));
  }
}

static void keyboard_task_fn(__UNUSED void* user_data) {
  while (true) {
    HID_KEYBD_Info_TypeDef info;
    channel_recv(&usb_keyboard_events, &info, sizeof(info));
    u32 input_state = 0;
    for (usize i = 0; i < SIZEOF(info.keys); i++) {
      switch (info.keys[i]) {
        case KEY_UPARROW: input_state |= INPUT_UP; break;
        case KEY_DOWNARROW: input_state |= INPUT_DOWN; break;
        case KEY_RIGHTARROW: input_state |= INPUT_RIGHT; break;
        case KEY_LEFTARROW: input_state |= INPUT_LEFT; break;
        case KEY_ESCAPE: input_state |= INPUT_ESC;
      }
    }
    __atomic_store_n(&game_state.input_state, input_state, __ATOMIC_RELAXED);
  }
}

void pong_demo(void) {
  task_notify_init(&frame_notify);

  struct TaskParams render_task_params = {
    .stack_start = render_task_stack,
    .stack_size = sizeof(render_task_stack),
    .func = &render_task_fn,
  };
  task_spawn(&render_task, &render_task_params);

  struct TaskParams game_task_params = {
    .stack_start = game_task_stack,
    .stack_size = sizeof(game_task_stack),
    .func = &game_task_fn,
  };
  task_spawn(&game_task, &game_task_params);

  struct TaskParams usb_task_params = {
    .stack_start = usb_task_stack,
    .stack_size = sizeof(usb_task_stack),
    .func = &usb_task_fn,
  };
  task_spawn(&usb_task, &usb_task_params);

  struct TaskParams keyboard_task_params = {
    .stack_start = keyboard_task_stack,
    .stack_size = sizeof(keyboard_task_stack),
    .func = &keyboard_task_fn,
  };
  task_spawn(&keyboard_task, &keyboard_task_params);
}
