#include "stmes/main.h"
#include "stmes/demos.h"
#include "stmes/dma.h"
#include "stmes/gpio.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/mpu.h"
#include "stmes/kernel/task.h"
#include "stmes/kernel/time.h"
#include "stmes/sdio.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <stm32f4xx_hal.h>

struct Task main_task;
u8 main_task_stack[1024] __ALIGNED(8);

void prestart(void) {
  extern u32 _sdata, _edata, _sidata, _sbss, _ebss;
  fast_memcpy_u32(&_sdata, &_sidata, &_edata - &_sdata);
  fast_memset_u32(&_sbss, 0, &_ebss - &_sbss);

  // Flush the pipeline in case the initialization of the memory sections above
  // has caused any side effects, or placed any functions into RAM.
  __DSB();
  __ISB();

  SystemInit(); // Initializes the FPU among other things

  crash_init_hard_faults();
  mpu_init();

  // Enable the internal CPU cycle counter.
  // <https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit/CYCCNT-cycle-counter-and-related-timers>
  // <https://developer.arm.com/documentation/ka001499/latest> - getting accurate executed instruction counts
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#ifdef ARM_SEMIHOSTING_ENABLE
  // Actually enable the semihosting machinery only when the debugger is attached.
  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
    extern void initialise_monitor_handles(void);
    initialise_monitor_handles();
  }
#endif

  extern void __libc_init_array(void);
  // Runs the initializers of static variables, constructors of C++ classes and
  // functions marked with __attribute__((constructor)).
  __libc_init_array();
}

int main(void) {
  HAL_Init();
  // TODO: this function needs HAL_GetTick to be ready
  SystemClock_Config();

  hwtimer_init();

  console_init();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();

  vga_init();
  vga_apply_timings(&VGA_TIMINGS_640x480_57hz);
  vga_start();

  // console_main_loop();
  // frame_test_demo();
  // image_viewer_demo();
  mandelbrot_demo();
  // print_tasks_demo();
  // sd_card_benchmark();
  // terminal_demo();
  // video_player_demo();
  // video_player_init();
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void SystemClock_Config(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitTypeDef rcc_osc_init = {
    .OscillatorType = RCC_OSCILLATORTYPE_HSE,
    .HSEState = RCC_HSE_ON,
    .PLL = {
      .PLLState = RCC_PLL_ON,
      .PLLSource = RCC_PLLSOURCE_HSE,
      .PLLM = 25,
      .PLLN = 192,
      .PLLP = RCC_PLLP_DIV2,
      .PLLQ = 4,
    },
  };
  check_hal_error(HAL_RCC_OscConfig(&rcc_osc_init));

  RCC_ClkInitTypeDef rcc_clk_init = {
    .ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
    .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
    .AHBCLKDivider = RCC_SYSCLK_DIV1,
    .APB1CLKDivider = RCC_HCLK_DIV2,
    .APB2CLKDivider = RCC_HCLK_DIV1,
  };
  u32 flash_latency = FLASH_LATENCY_3;
  check_hal_error(HAL_RCC_ClockConfig(&rcc_clk_init, flash_latency));
}
