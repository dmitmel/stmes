// On the general design of the VGA driver:
//
// First of all, I am assuming you are familiar with how the VGA signal is
// generally formed and won't be explaining that here. Instead, here are links
// to the video series titled "The world's worst video card", which are very
// informative and describe the interface and a hardware implementation (using
// discrete logic gates) in great detail:
// <https://www.youtube.com/watch?v=l7rce6IQDWs> - part 1
// <https://www.youtube.com/watch?v=uqY3FMuMuRo> - part 2
// <https://www.youtube.com/watch?v=2iURr3NBprc> - part 3
// <https://www.youtube.com/watch?v=BUTHtNrpwiI> - part 4
//
// This link gives some really useful info on the practical implementation of
// VGA, plus the complete cable pinout: <https://lateblt.tripod.com/bit74.txt>.
// VGA timings and clock frequencies: <http://tinyvga.com/vga-timing>.
//
// Anyway. The main guiding principle here is getting the most out of the
// features of the board's peripherals to ensure that the video signal timings
// are matched as precisely as possible. The key to this has been the (ab)use
// of the hardware timers: the current timer architecture generates the sync
// pulses completely independently of the CPU, thus the board can keep
// outputting a blank frame even when the CPU is halted by the debugger (which
// makes things more convenient because monitors take time to adjust to the
// selected video mode once the sync signals are started, and I don't have to
// wait for that when pressing "continue").
//
// The DMA controller is used for outputting the pixel data onto the color
// lines on the GPIO port (feeding the colors to the its BSRR register) - on
// the F4 series chips it is fast enough to match the VGA pixel clock at
// reasonable video resolutions, freeing up the CPU to do all the rendering
// while the DMA is pushing pixels to the monitor in parallel. Since VGA is an
// analog interface, an external DAC is needed to convert the colors on the
// pixel lines into varying voltages for the RGB components, a simple R-2R
// ladder is more than enough for this.
//
// All in all, this design, by offloading work to hardware and being
// implemented in interrupts, requires minimal intervention from the
// application once all timers are started, demanding only for frame configs
// and rendered scanlines to be prepared in time, by communicating with the
// application through the `vga_control` struct. The only thing to watch out
// for is the pressure on the memory bus during the video output phase: since
// the STM32F411CE has only a single bank of RAM, it is very easy to cause
// contention on the bus matrix and starve the pixel DMA while it is running in
// parallel at really high rates, *which will cause visible artifacts*.
//
// The idea of repurposing PWM for sync pulse generation was stolen from this
// project: <https://github.com/abelykh0/VGA-demo-on-bluepill>. A lot of things
// were also inspired by insights from this series of posts:
// <http://cliffle.com/p/m4vga/> - the author uses an STM32F407 controller and
// manages to do really impressive stuff with it! (highly recommend reading)
// 1. <http://cliffle.com/blog/introducing-glitch/> - a demo of what the
//    project can do.
// 2. <http://cliffle.com/blog/pushing-pixels/> - explains the usage of DMA on
//    STM32F407 and its different RAM banks.
// 3. <http://cliffle.com/blog/glitch-in-the-matrix/> - introduces various
//    hacks to reduce bus matrix contention (NOTE: read this particular article
//    to understand the issues I am having with the memory bus "pressure").
// 4. <http://cliffle.com/blog/racing-the-beam/> - describes the challenges of
//    real-time rasterization and some assembly-level hacks.

#include "stmes/video/vga.h"
#include "stmes/gpio.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include <math.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_tim.h>

// <http://tinyvga.com/vga-timing/640x480@60Hz>
const struct VgaTimings VGA_TIMINGS_640x480_57hz = {
  .pixel_clock_freq = 24000000,
  .active_width = 640,
  .horz_front_porch = 16,
  .hsync_pulse = -96,
  .horz_back_porch = 48,
  .active_height = 480,
  .vert_front_porch = 10,
  .vsync_pulse = -2,
  .vert_back_porch = 33,
};

// <http://tinyvga.com/vga-timing/640x480@60Hz>
const struct VgaTimings VGA_TIMINGS_640x480_60hz = {
  .pixel_clock_freq = 25175000,
  .active_width = 640,
  .horz_front_porch = 16,
  .hsync_pulse = -96,
  .horz_back_porch = 48,
  .active_height = 480,
  .vert_front_porch = 10,
  .vsync_pulse = -2,
  .vert_back_porch = 33,
};

// <http://tinyvga.com/vga-timing/800x600@60Hz>
const struct VgaTimings VGA_TIMINGS_800x600_60hz = {
  .pixel_clock_freq = 40000000,
  .active_width = 800,
  .horz_front_porch = 40,
  .hsync_pulse = 128,
  .horz_back_porch = 88,
  .active_height = 600,
  .vert_front_porch = 1,
  .vsync_pulse = 4,
  .vert_back_porch = 23,
};

// <http://tinyvga.com/vga-timing/1024x768@60Hz>
const struct VgaTimings VGA_TIMINGS_1024x768_60hz = {
  .pixel_clock_freq = 65000000,
  .active_width = 1024,
  .horz_front_porch = 24,
  .hsync_pulse = 136,
  .horz_back_porch = 160,
  .active_height = 768,
  .vert_front_porch = 3,
  .vsync_pulse = 6,
  .vert_back_porch = 29,
};

volatile struct VgaControlBlock vga_control;
struct Notification vga_notification;

// These all are grouped into a common wrapper struct so as to reduce the
// amount of generated machine code: if several internal state variables are
// used within a single function, the compiler will emit only one load of the
// address of the struct itself and then use relative accesses to load or store
// the fields. An additional bonus is that the linker will be forced to put all
// of these into a single memory location, improving data locality.
static struct VgaState {
  bool rendering_current_frame;
  u16 pixel_timer_prescaler;
  u32 active_area_start, active_area_height;
  u32 frame_first_line, frame_last_line;
  u32 current_scanline_repeats;
  const VgaPixel* current_scanline;
  struct VgaFrameConfig current_frame;
} vga_state;

static TIM_HandleTypeDef vga_pixel_timer, vga_hsync_timer, vga_vsync_timer;
static DMA_HandleTypeDef vga_pixel_dma;

void vga_init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  {
    // GPIOB is used for the VGA pixel color pins purely because a lot of pins
    // on this port are unoccupied by the built-in peripherals (which can't be
    // said about GPIOA).
    LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_PIXEL_ALL_PINS,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH, // These pins will be switching at MHz frequencies
    };
    HAL_GPIO_Init(VGA_PIXEL_GPIO_Port, &gpio_init);
  }

  {
    // The usage of the GPIO peripheral to output pixel data onto our parallel
    // color bus mandates the choice of the DMA2 unit and not DMA1, because the
    // GPIO registers live on the AHB1 and there simply is no interconnect in
    // the bus matrix between either port of the DMA1 unit and the peripherals
    // on AHB (see figure 4 of the STM32F411CE datasheet), only DMA2 can read
    // from RAM and write to the GPIO output register. This problem is covered
    // in more detail here: <http://www.efton.sk/STM32/gotcha/g30.html> and
    // here: <https://stackoverflow.com/questions/46613053/pwm-dma-to-a-whole-gpio/46619315>.
    DMA_HandleTypeDef* hdma = &vga_pixel_dma;
    hdma->Instance = DMA2_Stream5;
    // TODO: Dynamically adjust the DMA settings to accommodate higher resolutions.
    hdma->Init = (DMA_InitTypeDef){
      // Corresponds to the TIM1_UP event, for the list of all DMA streams and
      // channels see table 28 "DMA2 request mapping" of RM0383.
      .Channel = DMA_CHANNEL_6,
      // Since all peripherals are memory-mapped and both (memory and
      // peripheral) ports of the DMA2 have more or less full access to the bus
      // matrix, functionally there is no distinction between reading/writing
      // from/to an address in SRAM or an address in the peripheral range,
      // though in practice the DMA controller can prefetch and buffer data
      // into the FIFO when reading from the memory port. This process helps in
      // the presence of contention with the CPU on the bus matrix, however it
      // isn't zero-cost - this gives a very stable video signal, but the
      // stream can't go faster than HCLK/6. Increasing the speed to get to
      // higher resolutions is achievable by "flipping" the direction of the
      // stream to peripheral-to-memory, where the scanline buffer is read from
      // the peripheral port and the writes to the GPIO register occur on the
      // memory port - this schizo-optimization can reliably get us up to
      // HCLK/4 (note that in that case all settings and registers relating to
      // either port need to be swapped with each other).
      .Direction = DMA_MEMORY_TO_PERIPH,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_NORMAL,
      .Priority = DMA_PRIORITY_HIGH,
      // Having FIFO on is very helpful as explained above. The FIFO settings
      // can noticeably affect signal stability (especially when the memory bus
      // is under high load) and the maximum output frequency, but further
      // experimentation is required here.
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL,
      .MemBurst = DMA_MBURST_SINGLE,
      .PeriphBurst = DMA_PBURST_SINGLE,
    };
    check_hal_error(HAL_DMA_Init(hdma));
  }

  // For timer interconnections see tables 49, 53 and 56 "TIMx internal trigger
  // connection" in RM0383.

  // TIM1 <- TIM5/TIM2/TIM3/TIM4
  // TIM2 <- TIM1/----/TIM3/TIM4
  // TIM3 <- TIM1/TIM2/TIM5/TIM4
  // TIM4 <- TIM1/TIM2/TIM3/----
  // TIM5 <- TIM2/TIM3/TIM4/----
  // TIM9 <- TIM2/TIM3/TIM10/TIM11

  {
    TIM_HandleTypeDef* htim = &vga_pixel_timer;
    // Configuration for the pixel timer, which drives the pixel DMA. TIM1 is
    // pretty much the only timer that can generate DMA requests for DMA2, so
    // the choice of the timer was dictated by our pixel DMA unit.
    htim->Instance = TIM1;
    htim->Init = (TIM_Base_InitTypeDef){
      .Prescaler = 0,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 0,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      // The auto-reload preload is absolutely necessary here for the tricks in
      // the implementation of the horizontal scanline offset.
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    check_hal_error(HAL_TIM_Base_Init(htim));

    TIM_ClockConfigTypeDef clock_init = {
      .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
    };
    check_hal_error(HAL_TIM_ConfigClockSource(htim, &clock_init));

    // The pixel timer is programmed to start ticking once a trigger event
    // arrives from the hsync timer, at the start of every scanline. This
    // allows us to VERY accurately meet the timings for scanlines and begin
    // feeding the pixels of every scanline at the exact same time because the
    // hardware is made responsible for priming the pixel timer.
    TIM_SlaveConfigTypeDef slave_init = {
      .SlaveMode = TIM_SLAVEMODE_TRIGGER,
      .InputTrigger = TIM_TS_ITR1, // Slave to TIM2
    };
    check_hal_error(HAL_TIM_SlaveConfigSynchro(htim, &slave_init));

    TIM_MasterConfigTypeDef master_init = {
      .MasterOutputTrigger = TIM_TRGO_RESET,
      .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
    };
    check_hal_error(HAL_TIMEx_MasterConfigSynchronization(htim, &master_init));

    __HAL_LINKDMA(&vga_pixel_timer, hdma[TIM_DMA_ID_UPDATE], vga_pixel_dma);
  }

  {
    // The hsync timer counts the pixels in a single scanline and has its
    // period set to the total duration of a scanline (including the blanking
    // interval). All other timings in the VGA driver are essentially derived
    // from this timer. TIM2 has been chosen because both the pixel and the
    // vsync timers can be connected to it and because one of its channels is
    // connected to a pin on the GPIO port A (to free up the pins on port B for
    // the color pins). Previously TIM3 had been used for this, but it didn't
    // satisfy the 2nd requirement, although its power consumption is lower and
    // I wouldn't have had to sacrifice one of the two available timers with
    // 32-bit counters.
    TIM_HandleTypeDef* htim = &vga_hsync_timer;
    htim->Instance = TIM2;
    htim->Init = (TIM_Base_InitTypeDef){
      .Prescaler = 0,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 0,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    check_hal_error(HAL_TIM_Base_Init(htim));

    TIM_ClockConfigTypeDef clock_init = {
      .ClockSource = TIM_CLOCKSOURCE_INTERNAL,
    };
    check_hal_error(HAL_TIM_ConfigClockSource(htim, &clock_init));

    check_hal_error(HAL_TIM_OC_Init(htim));
    check_hal_error(HAL_TIM_PWM_Init(htim));

    // The hsync timer shall generate trigger events every time the counter
    // matches the value in the output compare register of channel 1, which
    // happens at the start of active area of every line.
    TIM_MasterConfigTypeDef master_init = {
      .MasterOutputTrigger = TIM_TRGO_OC1,
      .MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE,
    };
    check_hal_error(HAL_TIMEx_MasterConfigSynchronization(htim, &master_init));

    // The channel 1 generates output compare events at the start of the line.
    TIM_OC_InitTypeDef channel1_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel1_init, TIM_CHANNEL_1));
    // Preloading the register won't hurt I guess...
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_1);

    // The channel 2 generates output compare events at the end of every line
    // and triggers the corresponding interrupt.
    TIM_OC_InitTypeDef channel2_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel2_init, TIM_CHANNEL_2));
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_2);

    // The horizontal synchronization pulse is generated using the PWM mode of
    // the channel 3. This little trick lets me completely ignore the concerns
    // of sync pulses in the rest of the driver. However, due to the simplistic
    // logic of PWM activation (high if CNT < CCRx, low if CNT >= CCRx) the
    // timer's phase needs to be shifted so that the counter compare value lies
    // on the sync pulse start time and the pulse ends at the auto-reload point
    // (channel 1 is necessary precisely to account for this phase shift).
    TIM_OC_InitTypeDef channel3_init = {
      .OCMode = TIM_OCMODE_PWM1,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_PWM_ConfigChannel(htim, &channel3_init, TIM_CHANNEL_3));
    // The output compare register is configured by HAL to be preloaded by
    // default for PWM channels.

    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_HSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      // Even at the highest resolution video modes supported by VGA this pin
      // won't be switching at a frequency exceeding 2 MHz, so I think even the
      // LOW speed is enough. For the electrical characteristics of different
      // GPIO speed modes consult table 55 "I/O AC characteristics" of the
      // STM32F411CE datasheet.
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF1_TIM2,
    };
    HAL_GPIO_Init(VGA_HSYNC_GPIO_Port, &gpio_init);
  }

  {
    // The vsync timer counts the scanlines and its period is set to the total
    // number of scanlines in a frame, including the vblank interval. It is
    // clocked by the hsync timer. TIM9 has been chosen for this because of its
    // low power consumption and because all of its channels are connected to
    // pins on port A and because it could be connected to the vsync himer. The
    // fact that it is one of the simpler timers and only has two channels,
    // however, means that we have to be creative with our usage of those.
    TIM_HandleTypeDef* htim = &vga_vsync_timer;
    htim->Instance = TIM9;
    htim->Init = (TIM_Base_InitTypeDef){
      .Prescaler = 0,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 0,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    };
    check_hal_error(HAL_TIM_Base_Init(htim));
    check_hal_error(HAL_TIM_OC_Init(htim));
    check_hal_error(HAL_TIM_PWM_Init(htim));

    // This timer is set to use the trigger input events as its clock source,
    // which in practice means that its counter is incremented at the start of
    // each scanline.
    TIM_SlaveConfigTypeDef slave_init = {
      .SlaveMode = TIM_SLAVEMODE_EXTERNAL1,
      .InputTrigger = TIM_TS_ITR0, // Slave to TIM2
    };
    check_hal_error(HAL_TIM_SlaveConfigSynchro(htim, &slave_init));

    // Channel 1 is used for triggering an interrupt at requested vertical
    // positions, which the driver uses for switching its phases.
    TIM_OC_InitTypeDef channel1_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel1_init, TIM_CHANNEL_1));
    // Output compare register preload MUST NOT be enabled here because we will
    // be changing the CCRx while the timer is ticking.

    // The same PWM trick as described above is used for generating the
    // vertical synchronization pulses on channel 2. The caveat regarding the
    // need for phase shift applies here as well.
    TIM_OC_InitTypeDef channel2_init = {
      .OCMode = TIM_OCMODE_PWM1,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_PWM_ConfigChannel(htim, &channel2_init, TIM_CHANNEL_2));

    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_VSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      // The duration of the pulse can be expressed at most in kHz frequencies.
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF3_TIM9,
    };
    HAL_GPIO_Init(VGA_VSYNC_GPIO_Port, &gpio_init);
  }
}

static u32 calc_timer_prescaler(u32 apbclk, u32 frequency, u32 max_prescaler) {
  if (frequency >= apbclk) return 1;
  u32 prescaler = round_div(apbclk, frequency);
  // This will divide the prescaler until it is smaller than the max value.
  return prescaler / ceil_div(prescaler, max_prescaler);
}

void vga_apply_timings(const struct VgaTimings* ts) {
  // APB1 timers: TIM2, TIM3, TIM4, TIM5
  u32 apb1_freq = HAL_RCC_GetPCLK1Freq() * (LL_RCC_GetAPB1Prescaler() == RCC_HCLK_DIV1 ? 1 : 2);
  // APB2 timers: TIM1, TIM9, TIM10, TIM11
  u32 apb2_freq = HAL_RCC_GetPCLK2Freq();

  TIM_TypeDef* pixel_tim = TIM1; // clocked from APB2
  TIM_TypeDef* hsync_tim = TIM2; // clocked from APB1
  TIM_TypeDef* vsync_tim = TIM9; // clocked from TIM2

  u32 hsync_width = ABS(ts->hsync_pulse), vsync_width = ABS(ts->vsync_pulse),
      hsync_polarity = ts->hsync_pulse < 0 ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW,
      vsync_polarity = ts->vsync_pulse < 0 ? LL_TIM_OCPOLARITY_HIGH : LL_TIM_OCPOLARITY_LOW;
  u32 active_width = ts->active_width, active_height = ts->active_height,
      horz_front_porch = ts->horz_front_porch, horz_back_porch = ts->horz_back_porch,
      vert_front_porch = ts->vert_front_porch, vert_back_porch = ts->vert_back_porch;
  u32 whole_line = active_width + horz_front_porch + hsync_width + horz_back_porch,
      whole_frame = active_height + vert_front_porch + vsync_width + vert_back_porch;

  u32 apb1_pixel_prescaler = calc_timer_prescaler(apb1_freq, ts->pixel_clock_freq, UINT16_MAX),
      apb2_pixel_prescaler = calc_timer_prescaler(apb2_freq, ts->pixel_clock_freq, UINT16_MAX);

  // Recalculate the timings if we couldn't get an integer frequency divisor.
  if (apb1_freq / apb1_pixel_prescaler != ts->pixel_clock_freq) {
    u32 pixel_clk_freq = apb1_freq / apb1_pixel_prescaler;
    float pixel_freq_ratio = (float)pixel_clk_freq / (float)ts->pixel_clock_freq;
    hsync_width = (u32)roundf(hsync_width * pixel_freq_ratio);
    active_width = (u32)roundf(active_width * pixel_freq_ratio);
    horz_back_porch = (u32)roundf(horz_back_porch * pixel_freq_ratio);
    horz_front_porch = (u32)roundf(horz_front_porch * pixel_freq_ratio);
    whole_line = (u32)roundf(whole_line * pixel_freq_ratio);
  }

  LL_TIM_DisableIT_CC1(vsync_tim);
  LL_TIM_DisableIT_CC2(hsync_tim);

  vga_state.active_area_start = vert_back_porch;
  vga_state.active_area_height = active_height;
  vga_state.pixel_timer_prescaler = apb2_pixel_prescaler;
  vga_state.rendering_current_frame = false;

  // TODO: Investigate methods of calculating both the prescaler and the
  // autoreload for clock division (the lower the timer frequency the lower the
  // current consumption, however, the timer update events become less precise).
  LL_TIM_SetPrescaler(pixel_tim, 0);
  // NOTE: When ARR=0 the timer is disabled.
  LL_TIM_SetAutoReload(pixel_tim, apb2_pixel_prescaler - 1);

  LL_TIM_SetPrescaler(hsync_tim, apb1_pixel_prescaler - 1);
  LL_TIM_SetAutoReload(hsync_tim, whole_line - 1);
  LL_TIM_OC_SetCompareCH1(hsync_tim, horz_back_porch);
  LL_TIM_OC_SetCompareCH2(hsync_tim, horz_back_porch + active_width);
  LL_TIM_OC_SetCompareCH3(hsync_tim, whole_line - hsync_width);
  LL_TIM_OC_SetPolarity(hsync_tim, LL_TIM_CHANNEL_CH3, hsync_polarity);

  LL_TIM_SetPrescaler(vsync_tim, 0);
  LL_TIM_SetAutoReload(vsync_tim, whole_frame - 1);
  LL_TIM_OC_SetCompareCH1(vsync_tim, 0);
  LL_TIM_OC_SetCompareCH2(vsync_tim, whole_frame - vsync_width);
  LL_TIM_OC_SetPolarity(vsync_tim, LL_TIM_CHANNEL_CH2, vsync_polarity);

  LL_TIM_ClearFlag_CC1(vsync_tim);
  LL_TIM_ClearFlag_CC2(hsync_tim);

  // Apply the prescaler changes immediately (and reset the counters).
  LL_TIM_GenerateEvent_UPDATE(pixel_tim);
  LL_TIM_GenerateEvent_UPDATE(hsync_tim);
  LL_TIM_GenerateEvent_UPDATE(vsync_tim);

  LL_TIM_EnableIT_CC1(vsync_tim);
}

void vga_start(void) {
  // Start the vsync timer and its channels first. It won't actually start
  // ticking yet since it is clocked by the hsync one.
  check_hal_error(HAL_TIM_Base_Start(&vga_vsync_timer));
  check_hal_error(HAL_TIM_OC_Start(&vga_vsync_timer, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_PWM_Start(&vga_vsync_timer, TIM_CHANNEL_2));
  // Start the hsync timer. Nothing happens yet either.
  check_hal_error(HAL_TIM_Base_Start(&vga_hsync_timer));
  // The vsync timer is triggered by the channel 1, so at this point the MCU
  // will begin generating the vertical synchronization pulses.
  check_hal_error(HAL_TIM_OC_Start(&vga_hsync_timer, TIM_CHANNEL_1));
  // Now the MCU will start generating pulses on the horizontal synchronization
  // line, which is enough for the monitor to recognize the VGA resolution.
  check_hal_error(HAL_TIM_PWM_Start(&vga_hsync_timer, TIM_CHANNEL_3));
  // And finally start the channel with the scanline interrupt, at which point
  // we can begin outputting actual video!
  check_hal_error(HAL_TIM_OC_Start(&vga_hsync_timer, TIM_CHANNEL_2));
}

__STATIC_FORCEINLINE void vga_stop_pixel_dma(void) {
  // Using the registers directly is more convenient here than the LL
  // functions and much faster than HAL.
  DMA_TypeDef* pixel_dma_base = DMA2;
  DMA_Stream_TypeDef* pixel_dma = DMA2_Stream5;
  // Disable the common interrupts
  CLEAR_BIT(pixel_dma->CR, DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_TCIE);
  // Disable the FIFO interrupts
  CLEAR_BIT(pixel_dma->FCR, DMA_SxFCR_FEIE);
  // Stop the stream
  CLEAR_BIT(pixel_dma->CR, DMA_SxCR_EN);
  // Confirm that the stream has been completely stopped
  while (READ_BIT(pixel_dma->CR, DMA_SxCR_EN)) {}
  // Additionally, flush the residual data out of the FIFO
  CLEAR_BIT(pixel_dma->CR, DMA_SxCR_EN);
  // Clear the interrupt flags of the stream
  WRITE_REG(
    pixel_dma_base->HIFCR,
    DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5
  );
}

__STATIC_FORCEINLINE void vga_start_pixel_dma(const VgaPixel* buf, u16 len) {
  DMA_Stream_TypeDef* pixel_dma = DMA2_Stream5;
  // Set the number of data
  WRITE_REG(pixel_dma->NDTR, len);
  // Set the peripheral address
  WRITE_REG(pixel_dma->PAR, (usize)&VGA_PIXEL_GPIO_Port->BSRR);
  // Set the memory address
  WRITE_REG(pixel_dma->M0AR, (usize)buf);
  // Enable the stream
  SET_BIT(pixel_dma->CR, DMA_SxCR_EN);
}

// Triggered by the vsync timer only on the important scanlines, returns the
// number of the next line it wants to be triggered on. The logic within this
// function is a convoluted mess, but I tried to simplify it at least somewhat.
__STATIC_FORCEINLINE u32 vga_on_line_start(void) {
  struct VgaState* state = &vga_state;
  volatile struct VgaControlBlock* control = &vga_control;
  TIM_TypeDef* vsync_tim = TIM9;
  TIM_TypeDef* hsync_tim = TIM2;
  u32 line_nr = LL_TIM_GetCounter(vsync_tim);

  // The rendering phase actually begins two lines before the active area of
  // the VGA signal: one line is subtracted because the pixel DMA will start
  // only on the next line anyway since it is reloaded by the end-of-line
  // interrupt, and we need at least one line time to prepare the pixel data.
  u32 active_area_entry_line = state->active_area_start - 2;

  // The outermost condition checks that we haven't just concluded the
  // rendering of a frame, in which case we skip to the fallback section below.
  if (!(state->rendering_current_frame && line_nr == state->frame_last_line + 1)) {
    // Ok, seems that the interrupt has occured somewhere before the frame.
    if (!state->rendering_current_frame) {
      if (line_nr == active_area_entry_line && control->frame_config_ready) {
        // Absorb the frame config if it has been prepared in time for us.
        state->current_frame = control->frame_config;
        control->frame_config_ready = false;
        u32 active_area_end = state->active_area_start + state->active_area_height - 1;
        u32 frame_offset = state->current_frame.offset_top;
        u32 frame_height = state->current_frame.lines_count;
        state->frame_first_line = MIN(state->active_area_start + frame_offset, active_area_end);
        state->frame_last_line = MIN(state->frame_first_line + frame_height - 1, active_area_end);
        // NOTE: frame_last_line and frame_first_line should be considered
        // valid only if rendering_current_frame is set!
        state->rendering_current_frame = true;
        // We are not done yet though: the execution will fall in the if block
        // below to check if we have to wait for the vertical frame offset or
        // can begin outputting video right away.
      }
    }

    if (state->rendering_current_frame) {
      // See the comments above for the justification for the -2.
      u32 frame_entry_line = state->frame_first_line - 2;
      if (line_nr != frame_entry_line) {
        // Have to wait some more (due to the vertical offset)! The execution
        // will eventually re-enter this section after the wait.
        return frame_entry_line;
      }
      // Ok, everything is ready now, we can begin the final preparations for
      // entering the frame. First, insert a dummy scanline for the duration of
      // the wait before the frame active area.
      state->current_scanline = NULL;
      // Minus one because the repeats value starts at zero.
      state->current_scanline_repeats = state->frame_first_line - line_nr - 1;
      // Place a request for the very first scanline.
      control->next_scanline = NULL;
      control->next_scanline_nr = 0;
      control->next_scanline_requested = true;
      // Reset and enable the line-drawing interrupt.
      LL_TIM_ClearFlag_CC2(hsync_tim);
      LL_TIM_EnableIT_CC2(hsync_tim);
      // Finally, notify the rest of the system that we are about to enter the
      // rendering phase.
      control->entering_frame = true;
      // And now this interrupt can peacefully sleep until the end of the frame.
      return state->frame_last_line + 1;
    }
  }

  // This is the fallback section, responsible for entering the vblank state.
  // Normally it is executed at the end of the frame area, but may also be
  // run in an unknown situation, e.g. if the frame config wasn't ready at the
  // start of the frame.
  state->rendering_current_frame = false;
  LL_TIM_DisableIT_CC2(hsync_tim);
  control->entering_vblank = true;
  return active_area_entry_line;
}

__STATIC_FORCEINLINE const VgaPixel* vga_fetch_next_scanline(u32 next_line_nr) {
  struct VgaState* state = &vga_state;
  volatile struct VgaControlBlock* control = &vga_control;

  // This branch is actually unlikely and will occur only once per frame.
  if (unlikely(next_line_nr > state->frame_last_line)) return NULL;

  // Well, the case of repeating scanlines is not exactly unlikely (it will
  // occur half the time), I just want to make the other branch (which has much
  // more code) more attractive to the compiler.
  if (unlikely(state->current_scanline_repeats > 0)) {
    state->current_scanline_repeats -= 1;
    return state->current_scanline;
  }

  const VgaPixel* scanline = control->next_scanline;
  control->next_scanline = NULL;
  state->current_scanline = scanline;
  u32 repeats = state->current_frame.line_repeats;
  state->current_scanline_repeats = repeats;

  // Plus one because the repeats value starts at zero.
  u32 requested_line_nr = next_line_nr + (repeats + 1);
  if (unlikely(requested_line_nr > state->frame_last_line)) return scanline;
  control->next_scanline_nr = requested_line_nr - state->frame_first_line;
  control->next_scanline_requested = true;
  // We can't notify from every end-of-line interrupt because it creates a race
  // condition:
  // 1. When the scanout of the very last line ends, posting a notification
  //    will invoke the task scheduler
  // 2. But if it takes too long to run, end-of-frame interrupt will occur,
  //    preempt it and post another notification
  // 3. But from the former interrupt the scheduler has already concluded that
  //    no work remains to be done (since no new scanlines were requested) and
  //    put the system to sleep with the WFI instruction
  // 4. The result: the rendering task doesn't see the end-of-frame
  //    notification and won't get the next frame configuration ready!
  //
  // In short: notifying from every scanline interrupt creates a situation at
  // the end of the frame where two notifications are posted in a short time
  // span, and the task scheduler currently isn't equipped to handle that. TODO
  if (task_notify(&vga_notification)) {
    task_yield_from_isr();
  }
  return scanline;
}

// Triggered by the hsync timer at the end of every line in the active area.
__STATIC_FORCEINLINE void vga_on_line_end_reached(void) {
  struct VgaState* state = &vga_state;
  TIM_TypeDef* pixel_tim = TIM1;
  // Halt the pixel timer, and thus the DMA.
  LL_TIM_DisableCounter(pixel_tim);
  // Reset all color pins, there must be no output in the blanking interval.
  LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
  vga_stop_pixel_dma();
  // Reset the color pins again if something has been flushed out of the FIFO.
  LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);

  TIM_TypeDef* vsync_tim = TIM9;
  u32 next_line_nr = LL_TIM_GetCounter(vsync_tim) + 1;

  const VgaPixel* next_scanline = vga_fetch_next_scanline(next_line_nr);
  if (unlikely(next_scanline == NULL)) return;

  // Prepare the apply the requested scanline parameters and prepare the pixel
  // timer for restart.
  u32 pixel_period = state->pixel_timer_prescaler * (state->current_frame.pixel_scale + 1);
  u32 left_offset = state->current_frame.offset_left;
  if (likely(left_offset == 0)) {
    // Instead of changing the prescaler (which apparently makes the timer less
    // precise) we change the period.
    LL_TIM_SetAutoReload(pixel_tim, pixel_period - 1);
    // The prescaler and auto-reload (when it is preloaded) changes are applied
    // only on the following UPDATE event, so we must generate one ourselves:
    LL_TIM_GenerateEvent_UPDATE(pixel_tim);
    // Forcing an UPDATE event won't start the counter, though that will reset
    // it. However, we want the pixel DMA to be started without any delay, so
    // set the counter to a value that will immediately cause a reload.
    LL_TIM_SetCounter(pixel_tim, pixel_period - 1);
  } else {
    // HACK: Shifting the scanlines by a horizontal offset is accomplished by
    // exploiting the auto-reload preload setting, which effectively turns the
    // ARR register into a memory cell for the duration of one timer tick.
    LL_TIM_SetAutoReload(pixel_tim, state->pixel_timer_prescaler * left_offset - 1);
    // Force an UPDATE event to place the wait period into the ARR register.
    // The fact that it also resets the counter is beneficial this time.
    LL_TIM_GenerateEvent_UPDATE(pixel_tim);
    // Unsure why, but a short delay is necessary here, otherwise the timer
    // peripheral won't observe the fake auto-reload value set above.
    __NOP();
    // And finally, write the real timer activation period. It won't be applied
    // yet, the first pixel will be outputted only after waiting for the offset
    // period at the start of the next line, but all following pixels will use
    // this auto-reload value.
    LL_TIM_SetAutoReload(pixel_tim, pixel_period - 1);
  }

  // Clear any leftover DMA requests from the timer...
  LL_TIM_DisableDMAReq_UPDATE(pixel_tim);
  LL_TIM_EnableDMAReq_UPDATE(pixel_tim);
  // ..and enable the DMA stream. It will actually be started when the pixel
  // timer starts ticking at the beginning of the next line.
  vga_start_pixel_dma(next_scanline, state->current_frame.line_length);
}

void TIM2_IRQHandler(void) {
  TIM_TypeDef* timer = TIM2;
  if (LL_TIM_IsActiveFlag_CC2(timer)) {
    LL_TIM_ClearFlag_CC2(timer);
    vga_on_line_end_reached();
  }
}

void TIM1_BRK_TIM9_IRQHandler(void) {
  TIM_TypeDef* timer = TIM9;
  if (LL_TIM_IsActiveFlag_CC1(timer)) {
    LL_TIM_ClearFlag_CC1(timer);
    u32 line_nr = vga_on_line_start();
    LL_TIM_OC_SetCompareCH1(timer, line_nr);
    if (task_notify(&vga_notification)) {
      task_yield_from_isr();
    }
  }
}

void vga_deinit(void) {
  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);

  check_hal_error(HAL_TIM_Base_DeInit(&vga_pixel_timer));
  check_hal_error(HAL_TIM_Base_DeInit(&vga_hsync_timer));
  check_hal_error(HAL_TIM_Base_DeInit(&vga_vsync_timer));
  // Since we are using direct register access for starting and stopping the
  // pixel DMA, the HAL code won't know if it was actually running, so force
  // its state, so that cleanup can be performed regardless.
  vga_pixel_dma.State = HAL_DMA_STATE_BUSY;
  check_hal_error(HAL_DMA_Abort(&vga_pixel_dma));
  check_hal_error(HAL_DMA_DeInit(&vga_pixel_dma));

  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM2_CLK_DISABLE();
  __HAL_RCC_TIM9_CLK_DISABLE();
}
