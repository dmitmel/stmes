#include "stmes/video/vga.h"
#include "stmes/gpio.h"
#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <math.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_tim.h>

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

// These all are grouped into a common wrapper struct so as to reduce the
// amount of generated machine code: if several internal state variables are
// used within a single function, the compiler will emit only one load of the
// address of the struct itself and then use relative accesses to load or store
// the fields. An additional bonus is that the linker will be forced to put all
// of these into a single memory location, improving data locality.
static struct VgaState {
  bool rendering_current_frame;
  u32 active_area_start, active_area_height;
  u32 frame_first_line, frame_last_line;
  u32 current_scanline_repeats;
  const u32* current_scanline;
  struct VgaFrameConfig current_frame;
} vga_state;

static TIM_HandleTypeDef vga_pixel_timer;
static TIM_HandleTypeDef vga_hsync_timer;
static TIM_HandleTypeDef vga_vsync_timer;
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
    LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_PIXEL_ALL_PINS,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    HAL_GPIO_Init(VGA_PIXEL_GPIO_Port, &gpio_init);
  }

  {
    DMA_HandleTypeDef* hdma = &vga_pixel_dma;
    hdma->Instance = DMA2_Stream5;
    hdma->Init = (DMA_InitTypeDef){
      .Channel = DMA_CHANNEL_6,
      .Direction = DMA_PERIPH_TO_MEMORY,
      .PeriphInc = DMA_PINC_ENABLE,
      .MemInc = DMA_MINC_DISABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_NORMAL,
      .Priority = DMA_PRIORITY_HIGH,
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL,
      .MemBurst = DMA_MBURST_SINGLE,
      .PeriphBurst = DMA_PBURST_SINGLE,
    };
    check_hal_error(HAL_DMA_Init(hdma));
  }

  {
    TIM_HandleTypeDef* htim = &vga_pixel_timer;
    htim->Instance = TIM1;
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

    TIM_SlaveConfigTypeDef slave_init = {
      .SlaveMode = TIM_SLAVEMODE_TRIGGER,
      .InputTrigger = TIM_TS_ITR1,
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

    TIM_MasterConfigTypeDef master_init = {
      .MasterOutputTrigger = TIM_TRGO_OC1,
      .MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE,
    };
    check_hal_error(HAL_TIMEx_MasterConfigSynchronization(htim, &master_init));

    TIM_OC_InitTypeDef channel1_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel1_init, TIM_CHANNEL_1));
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_1);

    TIM_OC_InitTypeDef channel2_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel2_init, TIM_CHANNEL_2));
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_2);

    TIM_OC_InitTypeDef channel3_init = {
      .OCMode = TIM_OCMODE_PWM1,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_PWM_ConfigChannel(htim, &channel3_init, TIM_CHANNEL_3));

    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_HSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF1_TIM2,
    };
    HAL_GPIO_Init(VGA_HSYNC_GPIO_Port, &gpio_init);
  }

  {
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

    TIM_SlaveConfigTypeDef slave_init = {
      .SlaveMode = TIM_SLAVEMODE_EXTERNAL1,
      .InputTrigger = TIM_TS_ITR0,
    };
    check_hal_error(HAL_TIM_SlaveConfigSynchro(htim, &slave_init));

    TIM_OC_InitTypeDef channel1_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel1_init, TIM_CHANNEL_1));

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

  if (apb1_freq / apb1_pixel_prescaler != ts->pixel_clock_freq) {
    u32 pixel_clk_freq = apb1_freq / apb1_pixel_prescaler;
    float pixel_freq_ratio = (float)pixel_clk_freq / (float)ts->pixel_clock_freq;
    hsync_width = roundf(hsync_width * pixel_freq_ratio);
    active_width = roundf(active_width * pixel_freq_ratio);
    horz_back_porch = roundf(horz_back_porch * pixel_freq_ratio);
    horz_front_porch = roundf(horz_front_porch * pixel_freq_ratio);
    whole_line = roundf(whole_line * pixel_freq_ratio);
  }

  LL_TIM_DisableIT_CC1(vsync_tim);
  LL_TIM_DisableIT_CC2(hsync_tim);

  vga_state.active_area_start = vert_back_porch;
  vga_state.active_area_height = active_height;

  // TODO: Investigate methods of calculating both the prescaler and the
  // autoreload for clock division (the lower the timer frequency the lower the
  // current consumption, however, the timer update events become less precise).
  LL_TIM_SetPrescaler(pixel_tim, 0);
  // NOTE: When ARR=0 the timer is disabled.
  LL_TIM_SetAutoReload(pixel_tim, apb2_pixel_prescaler - 1);

  // TODO: Investigate if 1 needs to subtracted from CCRx values.

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

  LL_TIM_EnableIT_CC1(vsync_tim);

  // Apply the prescaler changes immediately (and reset the counters).
  LL_TIM_GenerateEvent_UPDATE(pixel_tim);
  LL_TIM_GenerateEvent_UPDATE(hsync_tim);
  LL_TIM_GenerateEvent_UPDATE(vsync_tim);
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

__STATIC_FORCEINLINE void vga_start_pixel_dma(const u32* buf, u16 len) {
  DMA_Stream_TypeDef* pixel_dma = DMA2_Stream5;
  // Set the number of data
  WRITE_REG(pixel_dma->NDTR, len);
  // Set the peripheral address
  WRITE_REG(pixel_dma->PAR, (usize)buf);
  // Set the memory address
  WRITE_REG(pixel_dma->M0AR, (usize)&VGA_PIXEL_GPIO_Port->BSRR);
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

  // The rendering mode actually begins two lines before the active area of the
  // VGA signal: one line is subtracted because the pixel DMA will start only
  // on the next line anyway since it is reloaded by the end-of-line interrupt,
  // and we need at least one line time to prepare the pixel data.
  u32 active_area_entry_line = state->active_area_start - 2;

  // The outermost condition checks that we haven't just concluded the
  // rendering of a frame, in which case we skip to the fallback section below.
  if (!(state->rendering_current_frame && line_nr == state->frame_last_line)) {
    // Ok, seems that the interrupt has occured somewhere before the frame.
    if (!state->rendering_current_frame) {
      if (line_nr == active_area_entry_line && control->frame_config_ready) {
        // Absorb the frame config if it has been prepared in time for us.
        state->current_frame = control->frame_config;
        control->frame_config_ready = false;
        u32 active_area_end = state->active_area_start + state->active_area_height;
        u32 frame_offset = state->current_frame.offset_top;
        u32 frame_height = state->current_frame.lines_count;
        state->frame_first_line = MIN(state->active_area_start + frame_offset, active_area_end);
        state->frame_last_line = MIN(state->frame_first_line + frame_height, active_area_end);
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
      // rendering mode.
      control->entering_frame = true;
      // And now this interrupt can peacefully sleep until the end of the frame.
      return state->frame_last_line;
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

__STATIC_FORCEINLINE const u32* vga_fetch_next_scanline(u32 next_line_nr) {
  struct VgaState* state = &vga_state;
  volatile struct VgaControlBlock* control = &vga_control;

  // This branch is actually unlikely and will occur only once per frame.
  if (unlikely(next_line_nr == state->frame_last_line)) {
    return NULL;
  }

  // Well, the case of repeating scanlines is not exactly unlikely (it will
  // occur half the time), I just want to make the other branch (which has much
  // more code) more attractive to the compiler.
  if (unlikely(state->current_scanline_repeats > 0)) {
    state->current_scanline_repeats -= 1;
    return state->current_scanline;
  }

  const u32* scanline = control->next_scanline;
  control->next_scanline = NULL;
  state->current_scanline = scanline;
  u32 repeats = state->current_frame.line_repeats;
  state->current_scanline_repeats = repeats;

  // Plus one because the repeats value starts at zero.
  control->next_scanline_nr = next_line_nr - state->frame_first_line + (repeats + 1);
  control->next_scanline_requested = true;
  return scanline;
}

// Triggered by the hsync timer at the end of every line in the active area.
__STATIC_FORCEINLINE void vga_on_line_end_reached(void) {
  struct VgaState* state = &vga_state;
  TIM_TypeDef* pixel_tim = TIM1;
  LL_TIM_DisableCounter(pixel_tim); // Halt the pixel timer, and thus the DMA
  LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
  vga_stop_pixel_dma();
  LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);

  TIM_TypeDef* vsync_tim = TIM9;
  u32 next_line_nr = LL_TIM_GetCounter(vsync_tim) + 1;

  const u32* next_scanline = vga_fetch_next_scanline(next_line_nr);
  if (unlikely(next_scanline == NULL)) {
    return;
  }

  // Prepare the pixel timer for restart. First, we want to apply the requested
  // scanline parameters:
  // TODO: scanline offset can be accomplished by abusing the autoreload
  // preload setting, which effectively turns the ARR register into a memory
  // cell for one timer tick.
  LL_TIM_SetPrescaler(pixel_tim, state->current_frame.pixel_scale);
  // The prescaler changes are applied only on the following UPDATE event, so
  // we must generate one ourselves:
  LL_TIM_GenerateEvent_UPDATE(pixel_tim);
  // Forcing an UPDATE event won't start the counter, though it will reset it.
  // However, we want the pixel DMA to be started without any delay, so we set
  // the counter to a value that will cause a reload immediately upon restart.
  LL_TIM_SetCounter(pixel_tim, LL_TIM_GetAutoReload(pixel_tim));

  // Clear any leftover DMA requests from the timer...
  LL_TIM_DisableDMAReq_UPDATE(pixel_tim);
  LL_TIM_EnableDMAReq_UPDATE(pixel_tim);
  // ..and enable the DMA stream. It will actually be started when the pixel
  // timer starts ticking at the beginning of the next line.
  vga_start_pixel_dma(next_scanline, state->current_frame.line_length);
}

void vga_hsync_timer_isr(void) {
  TIM_TypeDef* timer = TIM2;
  if (LL_TIM_IsActiveFlag_CC2(timer)) {
    LL_TIM_ClearFlag_CC2(timer);
    vga_on_line_end_reached();
  }
}

void vga_vsync_timer_isr(void) {
  TIM_TypeDef* timer = TIM9;
  if (LL_TIM_IsActiveFlag_CC1(timer)) {
    LL_TIM_ClearFlag_CC1(timer);
    u32 line_nr = vga_on_line_start();
    LL_TIM_OC_SetCompareCH1(timer, line_nr);
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
