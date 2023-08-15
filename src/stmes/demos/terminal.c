#include "stmes/demos.h"
#include "stmes/fatfs.h"
#include "stmes/sdio.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <ff.h>
#include <printf.h>
#include <stm32f4xx_hal.h>

// TODO: rewrite to use tasks
void terminal_demo(void) {
  static FATFS SDFatFS;
  static DIR SDDir;

  while (BSP_SD_Init() != HAL_OK) {
    HAL_Delay(500);
  }

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_opendir(&SDDir, "/"));

  static struct {
    u8 color;
    const char* str;
  } code_tokens[] = {
    // clang-format off
    {5, "#include"}, {7, " "}, {2, "<stdio.h>"}, {7, "\n"},
    {5, "#include"}, {7, " "}, {2, "<stm32f4xx_hal.h>"}, {6, "\n"},
    {3, "int"}, {7, " "}, {4, "main"}, {7, "("}, {3, "int"}, {7, " "}, {1, "argc"}, {7, ", "}, {3, "char"}, {7, "* "}, {1, "argv"}, {7, "[]"}, {7, ") {\n"},
    {7, "  "}, {4, "printf"}, {7, "("}, {2, "\"Hello world from STM32!"}, {3, "\\n"}, {2, "\""}, {7, ");\n"},
    {7, "  "}, {5, "while"}, {7, " ("}, {3, "true"}, {7, ") {\n"},
    {7, "    "}, {4, "HAL_GPIO_TogglePin"}, {7, "("}, {3, "GPIOC"}, {7, ", "}, {3, "GPIO_PIN_13"}, {7, ");\n"},
    {7, "    "}, {4, "HAL_Delay"}, {7, "("}, {3, "100"}, {7, ");\n"},
    {7, "  }\n"},
    {7, "  "}, {5, "return"}, {7, " "}, {3, "0"}, {7, ";\n"},
    {7, "}\n"},
    // clang-format on
  };

  for (usize i = 0; i < SIZEOF(code_tokens); i++) {
    console_set_color(code_tokens[i].color);
    console_print(code_tokens[i].str);
  }

  while (true) {
    u16 vga_line = 0;
    if (vga_take_scanline_request(&vga_line)) {
      console_render_scanline(vga_line);
    }
    if (vga_control.entering_vblank) {
      vga_control.entering_vblank = false;
      console_setup_frame_config();
      static u32 prev_tick;
      u32 tick = HAL_GetTick();
      if (tick >= prev_tick + 500) {
        prev_tick = tick;
        static FILINFO fno;
        check_fs_error(f_readdir(&SDDir, &fno));
        if (fno.fname[0] != '\0') {
          printf(
            "%s%s%s%s%s %04d-%02d-%02d %02d:%02d:%02d %6luk %s\n",
            fno.fattrib & AM_ARC ? "A" : "-",
            fno.fattrib & AM_DIR ? "D" : "-",
            fno.fattrib & AM_SYS ? "S" : "-",
            fno.fattrib & AM_HID ? "H" : "-",
            fno.fattrib & AM_RDO ? "R" : "-",
            ((fno.fdate >> 9) & MASK(7)) + 1980,
            (fno.fdate >> 5) & MASK(4),
            fno.fdate & MASK(5),
            (fno.ftime >> 11) & MASK(5),
            (fno.ftime >> 5) & MASK(6),
            fno.ftime & MASK(5),
            fno.fsize / 1024,
            fno.fname
          );
        }
      }
    }
    WAIT_FOR_INTERRUPT();
  }
}
