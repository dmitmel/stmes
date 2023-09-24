#include "stmes/demos.h"
#include "stmes/gpio.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/mpu.h"
#include "stmes/kernel/task.h"
#include "stmes/kernel/time.h"
#include "stmes/utils.h"
#include "stmes/video/console.h"
#include "stmes/video/vga.h"
#include <stm32f4xx_hal.h>

static struct Task main_task;
static u8 main_task_stack[1024] __ALIGNED(8);

static void prestart(void* initial_stack_ptr);
static void main_task_launcher(void* initial_stack_ptr);
static void system_clock_config(void);

int main(void) {
  console_init();

  gpio_init();

  vga_init();
  vga_apply_timings(&VGA_TIMINGS_640x480_57hz);
  vga_start();

  // image_viewer_demo();
  // mandelbrot_demo();
  // pong_demo();
  // print_tasks_demo();
  sd_card_benchmark();
  // terminal_demo();
  // text_rendering_benchmark();
  // video_player_demo();
}

// When the power is connected, the CPU first fetches the initial stack pointer
// from address 0x00000000 and sets the MSP to that, then fetches the reset
// handler from the vector table, whose address is stored at 0x00000004, and
// begins code execution from there. All in all, this means that normal C
// functions can be executed on boot right away, however, some setup needs to
// be done first before we can reach main().
__NAKED void Reset_Handler(void) {
  // Pass the initial stack pointer as the first argument (the need to be able
  // to read it is the reason for writing the reset handler in assembly).
  __ASM volatile("mov r0, sp");
  // Call the prestart() function.
  __ASM volatile("bl %0" ::"i"(&prestart));
  // Execution should never return here, trigger an exception if it did.
  __ASM volatile("udf");
}

// Performs the early boot sequence and eventually calls main().
static void prestart(void* initial_stack_ptr) {
  extern u32 _sdata, _edata, _sidata, _sbss, _ebss;
  fast_memcpy_u32(&_sdata, &_sidata, &_edata - &_sdata); // copy the .data section
  fast_memset_u32(&_sbss, 0, &_ebss - &_sbss);           // zero out the .bss section

  // Flush the pipeline in case the initialization of the memory sections above
  // has caused any side effects, or placed any functions into RAM.
  __DSB();
  __ISB();

  SystemInit(); // Initializes the FPU among other things

  // Configure the faults as early as possible to be able to catch errors in
  // the rest of the initialization code.
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
    // <https://github.com/bminor/newlib/blob/048031501043e61ca31713b92ce2190213c7fb21/libgloss/arm/syscalls.c#L141>
    extern void initialise_monitor_handles(void);
    initialise_monitor_handles();
  }
#endif

  HAL_Init();

  // TODO: this function needs HAL_GetTick to be ready
  system_clock_config();

  // Runs the initializers of static variables, constructors of C++ classes and
  // the functions marked with `__attribute__((constructor))`.
  extern void __libc_init_array(void);
  __libc_init_array();

  hwtimer_init();

  start_task_scheduler();

  // The rest of the boot sequence will happen with the task scheduler online,
  // and main() will be executed in its own task.
  struct TaskParams task_params = {
    .stack_start = main_task_stack,
    .stack_size = sizeof(main_task_stack),
    .func = &main_task_launcher,
    .user_data = initial_stack_ptr,
  };
  task_spawn(&main_task, &task_params);

  // The above function will make a syscall that will cause a context switch to
  // the newly created main task. Execution will *never* return here.
}

static __NAKED void main_task_launcher(__UNUSED void* initial_stack_ptr) {
  // This wrapper function will conclude the early boot sequence, in particular
  // reset the MSP to its initial value at boot. From now on, all the Main
  // Stack will ever be used for is processing interrupts, syscalls and running
  // the scheduler. The frames allocated on it prior to entering this function
  // are now useless, so the entirety of the Main Stack may be left to the
  // discretion of interrupts.
  __ASM volatile(     //
    "cpsid i\n\t"     // __disable_irq();
    "msr msp, r0\n\t" // MSP = initial_stack_ptr;
    "isb\n\t"         // apply the changes
    "cpsie i\n\t"     // __enable_irq();
  );
  // And now the main() can be tail-called (this is the reason for writing this
  // wrapper in assembly: to not waste any stack space before entering main()).
  __ASM volatile("b %0" ::"i"(&main));
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

static void system_clock_config(void) {
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
