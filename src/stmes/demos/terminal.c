#include "stmes/demos.h"
#include "stmes/fatfs.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include <ff.h>
#include <printf.h>

static u8 terminal_task_stack[1024] __ALIGNED(8);
static struct Task terminal_task;

static void terminal_task_fn(__UNUSED void* user_data) {
  static FATFS SDFatFS;
  static DIR SDDir;

  check_fs_error(f_mount(&SDFatFS, "", 1));
  check_fs_error(f_opendir(&SDDir, "/"));

  task_sleep(1000);

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
  task_sleep(1000);

  while (true) {
    static FILINFO fno;
    check_fs_error(f_readdir(&SDDir, &fno));
    if (fno.fname[0] == '\0') {
      break;
    }
    char size_str[16];
    humanize_bytes(size_str, sizeof(size_str), fno.fsize);
    printf(
      "%s%s%s%s%s %04d-%02d-%02d %02d:%02d:%02d %6s %s\n",
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
      size_str,
      fno.fname
    );
    task_sleep(500);
  }
}

void terminal_demo(void) {
  start_console_render_task();

  struct TaskParams terminal_task_params = {
    .stack_start = terminal_task_stack,
    .stack_size = sizeof(terminal_task_stack),
    .func = &terminal_task_fn,
  };
  task_spawn(&terminal_task, &terminal_task_params);
}
