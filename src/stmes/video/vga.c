#include "stmes/video/vga.h"
#include "stmes/gpio.h"
#include "stmes/main.h"
#include "stmes/utils.h"
#include <math.h>
#include <stdio.h>
#include <stm32f4xx.h>
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

struct VgaRegisters vga_registers = { 0 };
static u32 vga_active_area_packed = 0;

static TIM_HandleTypeDef vga_pixel_timer;
static TIM_HandleTypeDef vga_hsync_timer;
static TIM_HandleTypeDef vga_vsync_timer;
static DMA_HandleTypeDef vga_pixel_dma;

void vga_init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

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
      .InputTrigger = TIM_TS_ITR2,
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
    htim->Instance = TIM3;
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
      .OCMode = TIM_OCMODE_PWM1,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_PWM_ConfigChannel(htim, &channel2_init, TIM_CHANNEL_2));

    TIM_OC_InitTypeDef channel3_init = {
      .OCMode = TIM_OCMODE_TIMING,
      .Pulse = 0,
      .OCPolarity = TIM_OCPOLARITY_HIGH,
      .OCFastMode = TIM_OCFAST_DISABLE,
    };
    check_hal_error(HAL_TIM_OC_ConfigChannel(htim, &channel3_init, TIM_CHANNEL_3));
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, TIM_CHANNEL_3);

    GPIO_InitTypeDef gpio_init = {
      .Pin = VGA_HSYNC_Pin,
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
      .Alternate = GPIO_AF2_TIM3,
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
      .InputTrigger = TIM_TS_ITR1,
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
  TIM_TypeDef* hsync_tim = TIM3; // clocked from APB1
  TIM_TypeDef* vsync_tim = TIM9; // clocked from TIM3

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

  u32 active_area_start = vert_back_porch, active_area_end = vert_back_porch + active_height;
  vga_active_area_packed = (active_area_start << 16) | (active_area_end & 0xffff);

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
  LL_TIM_OC_SetCompareCH2(hsync_tim, whole_line - hsync_width);
  LL_TIM_OC_SetPolarity(hsync_tim, LL_TIM_CHANNEL_CH2, hsync_polarity);
  LL_TIM_OC_SetCompareCH3(hsync_tim, horz_back_porch + active_width);

  LL_TIM_SetPrescaler(vsync_tim, 0);
  LL_TIM_SetAutoReload(vsync_tim, whole_frame - 1);
  LL_TIM_OC_SetCompareCH1(vsync_tim, vert_back_porch + active_height);
  LL_TIM_OC_SetCompareCH2(vsync_tim, whole_frame - vsync_width);
  LL_TIM_OC_SetPolarity(vsync_tim, LL_TIM_CHANNEL_CH2, vsync_polarity);

  // Apply the prescaler changes immediately
  LL_TIM_GenerateEvent_UPDATE(pixel_tim);
  LL_TIM_GenerateEvent_UPDATE(hsync_tim);
  LL_TIM_GenerateEvent_UPDATE(vsync_tim);
}

void vga_start(void) {
  // Start the vsync timer and its channels first. It won't actually start
  // ticking yet since it is clocked by the hsync one.
  check_hal_error(HAL_TIM_Base_Start(&vga_vsync_timer));
  check_hal_error(HAL_TIM_OC_Start_IT(&vga_vsync_timer, TIM_CHANNEL_1));
  check_hal_error(HAL_TIM_PWM_Start(&vga_vsync_timer, TIM_CHANNEL_2));
  // Start the hsync timer. Nothing happens yet either.
  check_hal_error(HAL_TIM_Base_Start(&vga_hsync_timer));
  // The vsync timer is triggered by the channel 1, so at this point the MCU
  // will begin generating the horizontal synchronization pulses.
  check_hal_error(HAL_TIM_OC_Start(&vga_hsync_timer, TIM_CHANNEL_1));
  // Now the MCU will start generating pulses on the vertical synchronization
  // line, which is enough for the monitor to recognize the VGA resolution.
  check_hal_error(HAL_TIM_PWM_Start(&vga_hsync_timer, TIM_CHANNEL_2));
  // And finally start the channel with the scanline interrupt, at which point
  // we can begin generating actual video!
  check_hal_error(HAL_TIM_OC_Start_IT(&vga_hsync_timer, TIM_CHANNEL_3));
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
  // Additionally, flush the residual data in the FIFO
  CLEAR_BIT(pixel_dma->CR, DMA_SxCR_EN);
  // Clear the interrupt flags of the stream
  WRITE_REG(
    pixel_dma_base->HIFCR,
    DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5
  );
  vga_pixel_dma.State = HAL_DMA_STATE_READY;
}

__STATIC_FORCEINLINE void vga_start_pixel_dma(u32* buf, u16 len) {
  vga_pixel_dma.State = HAL_DMA_STATE_BUSY;
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

__STATIC_FORCEINLINE void vga_on_line_end_reached(void) {
  TIM_TypeDef* pixel_tim = TIM1;
  LL_TIM_DisableCounter(pixel_tim); // Halt the pixel timer, and thus the DMA
  LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);

  if (likely(vga_pixel_dma.State == HAL_DMA_STATE_BUSY)) {
    vga_stop_pixel_dma();
    LL_GPIO_ResetOutputPin(VGA_PIXEL_GPIO_Port, VGA_PIXEL_ALL_PINS);
  }

  TIM_TypeDef* vsync_tim = TIM9;
  u32 line_nr = LL_TIM_GetCounter(vsync_tim);
  u32 active_start = vga_active_area_packed >> 16, active_end = vga_active_area_packed & 0xffff;
  if (likely(!(active_start <= line_nr && line_nr < active_end))) return;

  static struct VgaScanline current_scanline;

  bool should_render = true;
  if (current_scanline.repeats > 0) {
    current_scanline.repeats -= 1;
  } else {
    if (vga_registers.next_scanline_ready) {
      current_scanline = vga_registers.next_scanline;
      vga_registers.next_scanline_ready = false;
    } else {
      should_render = false;
    }
    // +1 because the zero value is used for specifying that there is no
    // pending request.
    vga_registers.scanline_request = line_nr - active_start + 1;
  }
  if (unlikely(!should_render)) return;

  // Prepare the pixel timer for restart. First, we want to apply the requested
  // scanline parameters:
  // TODO: scanline offset can be accomplished by abusing the autoreload
  // preload setting, which effectively turns the ARR register into a memory
  // cell for one timer tick.
  LL_TIM_SetPrescaler(pixel_tim, current_scanline.pixel_scale);
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
  vga_start_pixel_dma(current_scanline.buffer, current_scanline.length);
}

void vga_hsync_timer_isr(void) {
  TIM_TypeDef* timer = TIM3;
  if (LL_TIM_IsActiveFlag_CC3(timer)) {
    LL_TIM_ClearFlag_CC3(timer);
    vga_on_line_end_reached();
  }
}

void vga_vsync_timer_isr(void) {
  TIM_TypeDef* timer = TIM9;
  if (LL_TIM_IsActiveFlag_CC1(timer)) {
    LL_TIM_ClearFlag_CC1(timer);
    vga_registers.next_frame_request = true;
  }
}

void vga_deinit(void) {
  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn);

  check_hal_error(HAL_TIM_Base_DeInit(&vga_pixel_timer));
  check_hal_error(HAL_TIM_Base_DeInit(&vga_hsync_timer));
  check_hal_error(HAL_TIM_Base_DeInit(&vga_vsync_timer));
  check_hal_error(HAL_DMA_Abort(&vga_pixel_dma));
  check_hal_error(HAL_DMA_DeInit(&vga_pixel_dma));

  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM3_CLK_DISABLE();
  __HAL_RCC_TIM9_CLK_DISABLE();
}
