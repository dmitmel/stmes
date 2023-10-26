// The fastest SD card driver in the west. It was developed to improve the
// performance, reduce the size, expand feature support (such as activating the
// High-Speed mode of compatible cards) and to just generally make the
// implementation more straightforward compared to the stock STM32 HAL driver,
// and on top of that being better integrated into the overall system. And also
// because implementing the SD specification turned out to be way more fun than
// sifting through dry technical PDFs should reasonably be.
//
// Other existing projects were referenced while writing code for this driver:
// <https://github.com/embassy-rs/embassy/blob/2a4ebdc150ba7c32d7dacc12aec85bccf05a41ff/embassy-stm32/src/sdmmc/mod.rs>
// <https://github.com/espressif/esp-idf/blob/v5.0.4/components/sdmmc/sdmmc_cmd.c>
// <https://github.com/espressif/esp-idf/blob/v5.0.4/components/sdmmc/sdmmc_sd.c>
// <https://github.com/espressif/esp-idf/blob/v5.0.4/components/sdmmc/sdmmc_common.c>
// <https://github.com/espressif/esp-idf/blob/v5.0.4/components/sdmmc/sdmmc_init.c>
// <https://github.com/espressif/esp-idf/blob/v5.0.4/components/driver/sdmmc/include/driver/sdmmc_defs.h>
// <https://github.com/espressif/esp-idf/blob/v5.0.4/components/driver/sdmmc/include/driver/sdmmc_types.h>
// <https://github.com/zephyrproject-rtos/esp-idf/blob/6835bfc741bf15e98fb7971293913f770df6081f/components/sdmmc/sdmmc_cmd.c>
// <https://github.com/LonelyWolf/stm32/blob/287e216a799fbacf00a8b3e363514c9bb82eafcd/stm32l4-sdio/src/sdcard.c>
// <https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/341cd894516f747f14108de5da593dad99900ae0/STM32F4/cores/maple/libmaple/sdio.c>
// <https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/341cd894516f747f14108de5da593dad99900ae0/STM32F4/libraries/SDIO/SdioF4.cpp>
// <https://github.com/insane-adding-machines/frosted/blob/5c3aa21f146a1b840badfd72d1d8b96e8405a8d8/kernel/drivers/stm32_sdio.c>
// <https://github.com/ARM-software/arm-trusted-firmware/blob/v2.9.0/drivers/mmc/mmc.c>
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c>
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_sd.h>
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c>
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_sdmmc.h>
//
// The SD/MMC/SDIO specs can be obtained at:
// <https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/imx-processors%40tkb/3706/1/Part_1_Physical_Layer_Specification_Ver3.01_Final_100218.pdf>
// <https://users.ece.utexas.edu/~valvano/EE345M/SD_Physical_Layer_Spec.pdf>
// <https://www.sdcard.org/cms/wp-content/themes/sdcard-org/dl.php?f=Part1_Physical_Layer_Simplified_Specification_Ver9.00.pdf>
// <https://www.sdcard.org/cms/wp-content/themes/sdcard-org/dl.php?f=PartE1_SDIO_Simplified_Specification_Ver3.00.pdf>
// <https://elinux.org/images/d/d3/Mmc_spec.pdf>
//
// And also here are some articles on the matter which I found useful:
// <https://blog.frankvh.com/2011/09/04/stm32f2xx-sdio-sd-card-interface/>
// <https://yannik520.github.io/sdio.html>
// <https://habr.com/ru/articles/213803/>
// <http://elm-chan.org/docs/mmc/mmc_e.html>
// <https://www.kingston.com/datasheets/SDCIT-specsheet-8gb-32gb_en.pdf>

// TODO: Support SDUC cards:
// 1. Parse CSD v3.
// 2. Store block addresses and numbers in u64.
// 3. Send the high bits of the block address with CMD22 before reads/writes.

// TODO: Verify checksum of CID and CSD.

// TODO: Add interrupts for card insertion/removal, support hot-plugging.

#include "stmes/drivers/sdmmc.h"
#include "stmes/drivers/dma.h"
#include "stmes/gpio.h"
#include "stmes/interrupts.h"
#include "stmes/kernel/crash.h"
#include "stmes/kernel/sync.h"
#include "stmes/kernel/task.h"
#include <printf.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_sdmmc.h>

#define SDMMC_LOG_COMMANDS 0

// The SDIO peripheral has two DMA streams attached to it, DMA2_Stream3 and
// DMA2_Stream6, however, both of them can be used for transferring data in
// both directions, as such, let's use only one since we will only ever be
// either sending or receiving data at a single time anyway.
#define SDIO_DMA DMA2_Stream3

// Masks for the SDIO_STA, SDIO_ICR and SDIO_MASK registers (they largely have
// the same bit definitions).
// clang-format off
#define SDIO_CMD_INTERRUPT_FLAGS   (SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT)
#define SDIO_DATA_ERROR_FLAGS      (SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR | SDIO_STA_STBITERR)
#define SDIO_DATA_INTERRUPT_FLAGS  (SDIO_DATA_ERROR_FLAGS | SDIO_STA_DATAEND)
// clang-format on

#define rca_arg(card) ((u32)(card)->rca << 16)

static struct SdmmcCard sdmmc_card;

static struct Mutex sdio_lock; // A global lock for the SDIO peripheral.
static struct Notification sdio_notification;

struct SdFuncStatus {
  bool supported : 1, busy : 1, selected : 1;
};

enum SdioTransfer { SDIO_RX, SDIO_TX };

static u32
sd_switch_function(bool do_switch, u32 group, u32 func, struct SdFuncStatus* out_status);
static u32 configure_sdio_bus(u32 freq, u32 bus_width, u32 power_saving);
static void
prepare_sdio_for_transfer(enum SdioTransfer transfer, u8* buffer, u32 blocks, u32 block_size);
static void stop_sdio_transfer(void);
static u32 wait_for_sdio_transfer(Systime deadline);
static u32 sdmmc_command(enum SdmmcCommand cmd, u32 arg, u32 response[4]);

static __attribute__((constructor)) void sdmmc_init_locks(void) {
  mutex_init(&sdio_lock);
}

void sdmmc_init_gpio(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init = {
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = GPIO_AF12_SDIO,
  };
  gpio_init.Pin = SDIO_CLK_PIN;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Pin = SDIO_D1_PIN | SDIO_D2_PIN | SDIO_CMD_PIN;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  gpio_init.Pin = SDIO_D0_PIN | SDIO_D3_PIN;
  HAL_GPIO_Init(GPIOB, &gpio_init);
}

bool sdmmc_is_card_inserted(void) {
  // My card reader slot simply has a "card detect" pin that is pulled low when
  // the card is physically inserted (that is why the result is inverted), but
  // it is also possible to use the built-in pull-up resistor on the DAT3 pin
  // of the card for card detection:
  // <https://community.st.com/t5/stm32-mcus-products/how-to-check-from-firmware-whether-sd-card-inserted-in-slot/m-p/370237>
  return !LL_GPIO_IsInputPinSet(SDIO_CD_GPIO_PORT, SDIO_CD_PIN);
}

const struct SdmmcCard* sdmmc_get_card(void) {
  if (sdmmc_card.type == SDMMC_UNKNOWN_CARD) {
    return NULL; // The card has not been initialized yet
  }
  return &sdmmc_card;
}

__STATIC_INLINE u32 physical2logical(u32 blocks, u32 physical_block_size) {
  // The ctz() function gives us the log2() of the argument, which for the
  // default block size of 512 bytes is 9. The physical block size parameter
  // must be given as a power of 2, which the READ_BL_LEN and WRITE_BL_LEN
  // fields of the CSD already are.
  i32 factor = physical_block_size - __builtin_ctz(SDMMC_BLOCK_SIZE);
  // This expression essentially computes:
  //   blocks * 2^physical / 2^logical = blocks * 2^(physical - logical)
  // After all, bitshifting left or right by N either multiplies the operand by
  // 2 raised to N or divides it by 2 raised to N, respectively.
  return factor >= 0 ? blocks << factor : blocks >> -factor;
}

u32 sdmmc_get_blocks_count(const struct SdmmcCard* card) {
  switch (card->csd.bits.structure_version) {
    case SD_CSD_VERSION_1_0: {
      const struct SdCSDv1* csd_v1 = &card->csd.sd_v1;
      // For SDSC cards the number of blocks is given by:
      //   BLOCKNR = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2)
      // Which can encode at most 4096*512 blocks of size 2^READ_BL_LEN.
      u32 blocks = (csd_v1->card_size + 1u) * (1u << (csd_v1->card_size_multiplier + 2u));
      // The spec tells that to indicate a 2 GB card (the maximum capacity for
      // an SDSC card) the block size will be set to 1024 bytes instead of the
      // normal 512, so the resulting number of logical 512-byte blocks has to
      // be adjusted for that.
      return physical2logical(blocks, card->csd.bits.max_read_block_length);
    }
    case SD_CSD_VERSION_2_0: {
      const struct SdCSDv2* csd_v2 = &card->csd.sd_v2;
      // For SDHC/SDXC cards the number of blocks is defined simply as:
      //   BLOCKNR = (C_SIZE + 1) * 1024
      // With the block size always fixed to 512 bytes and C_SIZE being
      // expanded to 22 bits, the maximum card capacity that can be encoded
      // becomes 2 terabytes.
      u32 blocks = (csd_v2->card_size + 1u) * 1024u;
      // The physical block size is hardcoded as 2^9 = 512 bytes.
      return physical2logical(blocks, 9u);
    }
    default: {
      return 0; // Unsupported CSD version
    }
  }
}

u32 sdmmc_get_eraseable_sector_size(const struct SdmmcCard* card) {
  const struct SdmmcCSDBits* csd = &card->csd.bits;
  if (csd->structure_version != SD_CSD_VERSION_1_0) return 1;
  if (csd->supports_single_block_erase) return 1;
  // Older cards only support erasing (SECTOR_SIZE+1) aligned sectors, in the
  // units of 2^WRITE_BL_LEN (see figures 5-1 and 5-2 in the SD spec).
  // Additionally, block size on such cards may not necessarily be the same as
  // the 512 byte logical blocks.
  return physical2logical(csd->eraseable_sector_size + 1, csd->max_write_block_length);
}

// Returns the maximum supported clock signal frequency reported by the card in
// its CSD. Should almost always be either 25 MHz or 50 MHz.
u32 sdmmc_max_transfer_freq(const struct SdmmcCard* card) {
  const struct SdmmcCSDBits* csd = &card->csd.bits;
  // TODO: These factors are different for MMC cards
  static const u8 transfer_speed_factors[1 << 4] = {
    0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
  };
  static const u32 transfer_speed_units[1 << 2] = { 10000, 100000, 1000000, 10000000 };
  // The transfer speed factor is 4 bits long, the array covers its full range.
  u32 factor = transfer_speed_factors[csd->transfer_speed_factor & MASK(4)];
  // Even though the transfer speed unit is 3 bits long, values over 3 (4..7)
  // are reserved, so the 3rd bit can be simply ignored.
  u32 unit = transfer_speed_units[csd->transfer_speed_unit & MASK(2)];
  return unit * factor;
}

// Determines the supported SD specification version based on the fields in the
// card's SCR.
enum SdSpecVersion sdmmc_sd_spec_version(const struct SdmmcCard* card) {
  if (card->type == MMC_CARD) {
    return SD_SPEC_UNKNOWN_VERSION;
  }
  const struct SdSCRv1* scr = &card->scr.v1;
  // See table 5-19 "Physical Layer Specification Version" for the combinations
  // of the SD_SPEC, SD_SPEC3, SD_SPEC4 and SD_SPECX fields' values.
  switch (scr->sd_spec) {
    case 0: return SD_SPEC_V1_0;
    case 1: return SD_SPEC_V1_1;
    case 2: {
      if (scr->sd_spec3 != 1) return SD_SPEC_V2_0;
      switch (scr->sd_specx) {
        // The value of SD_SPEC4 matters only for differentiating v3.00 and
        // v4.00, for all other versions it is irrelevant.
        case 0: return scr->sd_spec4 != 1 ? SD_SPEC_V3_X : SD_SPEC_V4_X;
        case 1: return SD_SPEC_V5_X;
        case 2: return SD_SPEC_V6_X;
        case 3: return SD_SPEC_V7_X;
        case 4: return SD_SPEC_V8_X;
        case 5: return SD_SPEC_V9_X;
        // Of course, this function can't support every SD card version to ever
        // be released because the spec is gonna keep updating, but the newest
        // versions of the specification promise that future versions are going
        // to be indicated by increasing the value of the SD_SPECX field, so in
        // the case of higher values we can at least be sure that the card
        // implements functionality of v9.00-and-later.
        default: return SD_SPEC_V9_X;
      }
    }
  }
  return SD_SPEC_UNKNOWN_VERSION;
}

u32 sdmmc_init_card(const struct SdmmcHostCapabilities* host_caps) {
  u32 err = SDMMC_ERROR_NONE;
  mutex_lock(&sdio_lock);

  struct SdmmcCard* card = &sdmmc_card;
  // The remaining fields will be automatically zero-initialized.
  *card = (struct SdmmcCard){
    .type = SDMMC_UNKNOWN_CARD,
    .spec_version = SD_SPEC_UNKNOWN_VERSION,
    .rca = 0x0000, // The card starts out with a zero RCA.
  };

  printf("enabling the SDIO peripheral\n");

  __HAL_RCC_SDIO_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(SDIO_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);

  // TODO: There has to be a better way of abstracting this.
  if (!sdmmc_is_card_inserted()) {
    err = SDMMC_ERROR_REQUEST_NOT_APPLICABLE;
    goto exit;
  }

  // TODO: Implement card power control through an external voltage regulator.
  // NOTE: The power up procedure is described in section 6.4 "Power scheme".
  printf("power cycling the card\n");
  // Disable the SDIO clock and reset all of its parameters.
  CLEAR_BIT(SDIO->CLKCR, SDIO_CLKCR_CLKEN | CLKCR_CLEAR_MASK);
  // Cut off the power from the SDIO peripheral (the name of this register is a
  // bit misleading though, it controls only whether the card is clocked, it
  // doesn't actually power off or on the card).
  CLEAR_BIT(SDIO->POWER, SDIO_POWER_PWRCTRL);
  // Wait for the card to completely shut down.
  task_sleep(2);
  // Switch on the power (in actuality switch on the SDIO clocking unit).
  SET_BIT(SDIO->POWER, SDIO_POWER_PWRCTRL);
  // Wait a bit for the supply voltage to reach a stable level of 3.3V.
  task_sleep(2);
  // Supply a 400 kHz clock signal for the whole duration of the identification phase.
  card->clock_freq = configure_sdio_bus(400000, SDIO_BUS_WIDE_1B, SDIO_CLOCK_POWER_SAVE_DISABLE);
  // According to the SD spec, the frequency must be in the 100-400 kHz range.
  // For real, certain initialization-related commands on certain cards will
  // simply refuse to work at higher frequencies:
  // <https://community.st.com/t5/stm32-mcus-products/code-stuck-in-sd-findscr-function/m-p/229445>
  // <https://community.st.com/t5/stm32cubemx-mcus/stm32l4-sd-stm32cubemx-v5-0-0-sd-clock-speed-during-call-to/m-p/371550>
  ASSERT(100000 <= card->clock_freq && card->clock_freq <= 400000);
  // The spec demands that the host waits for the duration of 74 SD clocks
  // after powering the card on, which amounts to 185 microseconds, but
  // currently the kernel APIs don't give us big enough timer resolution for
  // this, and let's generally be safe and wait a little more.
  task_sleep(2);

  u32 response[4];
  Systime deadline = NO_DEADLINE;

  // NOTE: Now begins the card initialization and identification procedure. It
  // is covered in great detail in the section 4.2 "Card Identification Mode"
  // of the SD specification, and an overview is given on figures 4-1 and 4-2.

  printf("starting card initialization\n");
  Systime init_start_time = systime_now();

  // The card here may be in any state if the MCU has been rebooted.

  printf("resetting the card\n");
  for (u32 attempt = 0; attempt < 10; attempt++) {
    if (!(err = sdmmc_command(SDMMC_CMD0_GO_IDLE_STATE, 0, response))) break;
    task_sleep(1);
  }
  if (err) goto exit;

  // After a software reset with CMD0, the card switches to the Idle state.
  // Now, CMD8 must be issued to check support for SD spec v2.00-or-later.

  printf("sending interface conditions... ");
  bool is_v2_x_card;
  union SdmmcIfCond if_cond = { .word = 0 };
  if_cond.bits.check_pattern = 0xAA; // 0b10101010, recommended by the spec
  if_cond.bits.host_voltage_supply = SD_VOLTAGE_RANGE_2_7V_TO_3_6V;
  if (!(err = sdmmc_command(SD_CMD8_SEND_IF_COND, if_cond.word, response))) {
    union SdmmcIfCond resp_if_cond = { .word = response[0] };
    is_v2_x_card = true;
    // The spec recommends using the check pattern (in addition to the CRC
    // built into the protocol) to test signal integrity.
    if (resp_if_cond.bits.check_pattern == if_cond.bits.check_pattern) {
      printf("card is at least v2.x\n");
    } else {
      printf("check pattern error\n");
      err = SDMMC_ERROR_CMD_CRC_FAIL;
      goto exit;
    }
  } else {
    // The card will respond with an error to CMD8 in two cases: either if it
    // is a v1.XX card, and thus doesn't support CMD8, or if it can't operate
    // under the voltage supplied by the host. Problem is, I have no idea how
    // to differentiate the two conditions, since in both cases the card is
    // supposed to not return any response. Though, I guess the cards can at
    // least turn on and respond to commands at a pretty wide range of
    // voltages, so if the execution has reached here, it really must've been
    // an illegal command.
    is_v2_x_card = false;
    printf("card is v1.x\n");
    // Upon receiving an illegal command, the card will set the ILLEGAL_COMMAND
    // error bit returned in the R1 response of the next command. This bit is
    // reset simply by issuing any other valid command, but in the Idle state
    // the choice of legal commands is scarce.
    if ((err = sdmmc_command(SDMMC_CMD0_GO_IDLE_STATE, 0, response))) {
      if (err != SDMMC_ERROR_ILLEGAL_CMD) goto exit;
    }
  }

  // Next, the host shall issue ACMD41 to negotiate the supply voltage and
  // support for High/Extra Capacity cards, and wait until the card reaches
  // the Ready state.

  printf("sending operating conditions...\n");
  // A timeout of 1 second for initialization with ACMD41 is recommended by the spec.
  deadline = timeout_to_deadline(1000);
  while (true) {
    // The parameter and the return value of ACMD41 are so similar that I kinda
    // treat the argument as the host's own OCR and the command as negotiating
    // the two OCRs.
    union SdmmcOCR host_ocr = { .word = 0 };
    // TODO: Deduce the voltage window bits from minimum/maximum voltage
    // parameters of SdmmcHostCapabilities.
    host_ocr.bits.voltage_window_3_2v_to_3_3v = true;
    if (is_v2_x_card) {
      // HCS (Host Capacity Support) bit - SDHC/SDXC cards are supported
      host_ocr.bits.high_capacity_status = true;
      // S18R bit - don't request a switch to the 1.8V signaling level
      host_ocr.bits.switching_to_1_8v_status = false;
      // XPC bit - don't enable the maximum performance mode of SDXC cards
      host_ocr.bits.sdxc_power_control = SDXC_POWER_SAVING;
    }
    if ((err = sdmmc_command(SD_ACMD41_SD_SEND_OP_COND, host_ocr.word, response))) goto exit;
    union SdmmcOCR card_ocr = { .word = response[0] };
    // Check the busy status bit: 1 means that the card is still booting, 0
    // means that the card's internal controller has been initialized.
    if (card_ocr.bits.power_up_status) {
      if (is_v2_x_card) {
        // Check the CCS (Card Capacity Status) bit.
        card->type = card_ocr.bits.high_capacity_status ? SDHC_SDXC_CARD : SDSC_V2_X_CARD;
      } else {
        card->type = SDSC_V1_X_CARD;
      }
      break;
    }
    if (systime_now() >= deadline) {
      // The most frequent cause for a timeout on ACMD41 is precisely that the
      // card can't operate under the supplied voltage. The card will respond
      // to the command, but will refuse to perform further initialization.
      err = SDMMC_ERROR_INVALID_VOLTRANGE;
      goto exit;
    }
    task_sleep(1);
  }

  Systime init_time = systime_now() - init_start_time;
  printf("initialization completed in %" PRIu32 " ms\n", (u32)systime_as_millis(init_time));

  printf("starting card identification\n");
  Systime ident_start_time = systime_now();

  // The card is now in the Ready state. The host now requests its unique CID
  // register by issuing CMD2.

  printf("reading CID data\n");
  if ((err = sdmmc_command(SDMMC_CMD2_ALL_SEND_CID, 0, response))) goto exit;
  // The words in the CID must be flipped. The individual bytes are fine though.
  card->cid.words[0] = response[3];
  card->cid.words[1] = response[2];
  card->cid.words[2] = response[1];
  card->cid.words[3] = response[0];

  // After responding to CMD2, the card transitions to the Identification
  // state. The host then asks the card to publish its RCA which will be later
  // used for addressing it when transferring data.

  printf("requesting RCA... ");
  if ((err = sdmmc_command(SD_CMD3_SEND_RELATIVE_ADDR, 0, response))) goto exit;
  card->rca = (u16)(response[0] >> 16); // Extract the top 16 bits of the response
  printf("0x%04" PRIX16 "\n", card->rca);

  // NOTE: Strictly speaking the identification procedure is complete - the
  // card now enters the Stand-by state in the data transfer mode. However, a
  // few more steps need to be done in order to complete the initialization
  // sequence. The behavior of the SD card's state machine is described in
  // section 4.3 "Data Transfer Mode" of the spec and on figure 4-13 "SD Memory
  // Card State Diagram".

  printf("reading CSD register\n");
  // CMD9 must be issued before selecting the card.
  if ((err = sdmmc_command(SDMMC_CMD9_SEND_CSD, rca_arg(card), response))) goto exit;
  // The words of the CSD must be reversed as well.
  card->csd.words[0] = response[3];
  card->csd.words[1] = response[2];
  card->csd.words[2] = response[1];
  card->csd.words[3] = response[0];

  {
    char bytes_str[16];
    bytes_str[0] = '\0';
    u32 blocks = sdmmc_get_blocks_count(card);
    humanize_bytes(bytes_str, sizeof(bytes_str), (u64)blocks * SDMMC_BLOCK_SIZE);
    printf("blocks = %" PRIu32 ", capacity = %sB\n", blocks, bytes_str);
  }

  printf("selecting card 0x%04" PRIX16 "\n", card->rca);
  // Selecting the card moves it to the Transfer state.
  if ((err = sdmmc_command(SDMMC_CMD7_SELECT_CARD, rca_arg(card), response))) goto exit;

  // TODO: Check the CARD_IS_LOCKED bit at this point

  u32 scr_data[2];
  printf("reading SCR data\n");
  if ((err = sdmmc_command(SDMMC_CMD16_SET_BLOCKLEN, sizeof(scr_data), response))) goto exit;
  // For some reason the SCR is sent via the DAT lines and not, for example,
  // with the long response format R2.
  prepare_sdio_for_transfer(SDIO_RX, (u8*)scr_data, 1, sizeof(scr_data));
  deadline = timeout_to_deadline(100);
  if (!(err = sdmmc_command(SD_ACMD51_SEND_SCR, 0, response))) {
    err = wait_for_sdio_transfer(deadline);
  }
  stop_sdio_transfer();
  if (err) goto exit;
  // The byte order of the whole SCR must be reversed.
  card->scr.words[0] = u32_from_be(scr_data[1]);
  card->scr.words[1] = u32_from_be(scr_data[0]);

  card->spec_version = sdmmc_sd_spec_version(card);
  const char* spec_version_name = "0.00";
  switch (card->spec_version) {
    case SD_SPEC_UNKNOWN_VERSION: spec_version_name = "0.00"; break;
    case SD_SPEC_V1_0: spec_version_name = "1.01"; break;
    case SD_SPEC_V1_1: spec_version_name = "1.10"; break;
    case SD_SPEC_V2_0: spec_version_name = "2.00"; break;
    case SD_SPEC_V3_X: spec_version_name = "3.00"; break;
    case SD_SPEC_V4_X: spec_version_name = "4.00"; break;
    case SD_SPEC_V5_X: spec_version_name = "5.00"; break;
    case SD_SPEC_V6_X: spec_version_name = "6.00"; break;
    case SD_SPEC_V7_X: spec_version_name = "7.00"; break;
    case SD_SPEC_V8_X: spec_version_name = "8.00"; break;
    case SD_SPEC_V9_X: spec_version_name = "9.00"; break;
  }
  printf("card implements specification v%s\n", spec_version_name);

  // Here comes the most interesting part of the card initialization sequence
  // and what actually makes my driver the fastest: activation of the
  // High-Speed mode which allows communication with at frequencies up to 50
  // MHz (25 MB/s transfer rate with a 4 bit bus), instead of the standard 25
  // MHz (12.5 MB/s transfer rate).
  if (host_caps->high_speed_mode) {
    if (card->spec_version >= SD_SPEC_V1_1 && card->csd.bits.supports_switch_commands) {
      const u32 ACCESS_MODE_GROUP = 1;
      const u32 ACCESS_MODE_HIGH_SPEED = 1;

      struct SdFuncStatus status;
      printf("checking support of High-Speed mode... ");
      if ((err = sd_switch_function(false, ACCESS_MODE_GROUP, ACCESS_MODE_HIGH_SPEED, &status))) {
        goto exit;
      }
      if (status.supported && !status.busy && status.selected) {
        printf("ok\n");
        printf("switching to High-Speed mode... ");
        if ((err = sd_switch_function(true, ACCESS_MODE_GROUP, ACCESS_MODE_HIGH_SPEED, &status))) {
          goto exit;
        }
        if (status.supported && status.selected) {
          // The specification requires the host to wait for at least 8 clocks
          // after CMD8 before making use of the new functions.
          task_sleep(1);
          printf("ok\n");
        } else {
          printf("fail\n");
        }

        // Confirm the success of the switch by refreshing the CSD register.
        // Once the card switches to the HS mode, it is supposed to increase
        // the TRAN_SPEED field of the CSD from 25 MHz to 50 MHz. However, CMD9
        // is accepted only in the Stand-by state, so the card must be first
        // deselected, and then re-selected again afterwards to be returned to
        // the Transfer state.
        printf("reading CSD register\n");
        if ((err = sdmmc_command(SDMMC_CMD7_DESELECT_CARD, 0, response))) goto exit;
        if ((err = sdmmc_command(SDMMC_CMD9_SEND_CSD, rca_arg(card), response))) goto exit;
        card->csd.words[0] = response[3];
        card->csd.words[1] = response[2];
        card->csd.words[2] = response[1];
        card->csd.words[3] = response[0];
        if ((err = sdmmc_command(SDMMC_CMD7_SELECT_CARD, rca_arg(card), response))) goto exit;
      } else {
        printf("unsupported\n");
      }
    }
  }

  // This step isn't strictly necessary, but I guess we might want to equalize
  // the electrical characteristics of all data pins.
  printf("disabling pull-up resistor on CD/DAT3 pin\n");
  if ((err = sdmmc_command(SD_ACMD42_SET_CLR_CARD_DETECT, 0, response))) goto exit;

  u32 bus_width = SDIO_BUS_WIDE_1B;
  // Well, technically the SD spec requires all SD cards to support the 4-bit
  // bus, but a check beforehand won't hurt.
  if (host_caps->use_4bit_data_bus && card->scr.v1.supports_4bit_wide_bus) {
    bus_width = SDIO_BUS_WIDE_4B;
    printf("switching data bus to 4-bit mode\n");
    if ((err = sdmmc_command(SD_ACMD6_SET_BUS_WIDTH, 2, response))) goto exit;
  }

  // Engage the warp drive! Crank the SD clock and bus width to the max, plus
  // enable power saving (turns off the clock signal when the bus is idle).
  card->clock_freq =
    configure_sdio_bus(sdmmc_max_transfer_freq(card), bus_width, SDIO_CLOCK_POWER_SAVE_ENABLE);

  // This might be required for compatibility with the ancient v1.00 cards
  // which have variable logical block sizes, with the default not necessarily
  // being 512 bytes. Newer cards simply ignore CMD16.
  const u32 block_len = SDMMC_BLOCK_SIZE;
  printf("setting block length to %" PRIu32 "\n", block_len);
  if ((err = sdmmc_command(SDMMC_CMD16_SET_BLOCKLEN, block_len, response))) goto exit;

  Systime ident_time = systime_now() - ident_start_time;
  printf(
    "indentification sequence completed in %" PRIu32 " ms\n", (u32)systime_as_millis(ident_time)
  );

exit:
  check_sd_error(err);
  mutex_unlock(&sdio_lock);
  return err;
}

// Wrapper around the CMD6 command, which is used to query (do_switch=false)
// and switch (do_switch=true) 15 optional SD card functions in 6 function
// groups. Its usage is covered in the section 4.3.10 "Switch Function Command"
// of the SD specification.
static u32
sd_switch_function(bool do_switch, u32 group, u32 func, struct SdFuncStatus* out_status) {
  ASSERT(1 <= group && group <= 6 && func < 15);
  // The argument to CMD6 consists of six 4-bit values which designate what to
  // do regarding each of the 6 function groups. Each such field may be set to
  // either:
  // 1. 0xF, signifying that the group should be left as-is, untouched
  // 2. 0x0, asking to switch the group to the default function (which is
  //    always available)
  // 3. or any other number, which requests a switch to the respective function
  // We start out with an argument with all fields set to 0xF - meaning no
  // influence to any of the groups.
  u32 func_mask = 0xFFFFFF;
  const u32 group_shift = (group - 1) * 4;
  CLEAR_BIT(func_mask, MASK(4) << group_shift); // Clear out and change just
  SET_BIT(func_mask, func << group_shift);      // the requested group.
  // The 32nd bit of the argument determines whether to query the presence of
  // or actually switch to the function.
  const u32 cmd6_arg = func_mask | (do_switch ? BIT(31) : 0);

  // CMD6 also transmits a long table over the DAT lines with the statuses of
  // all functions and function groups.
  u32 switch_status[512 / (8 * sizeof(u32))];

  u32 err = SDMMC_ERROR_NONE;
  u32 response[4];
  if ((err = sdmmc_command(SDMMC_CMD16_SET_BLOCKLEN, sizeof(switch_status), response))) return err;

  prepare_sdio_for_transfer(SDIO_RX, (u8*)switch_status, 1, sizeof(switch_status));
  Systime deadline = timeout_to_deadline(100);
  if (!(err = sdmmc_command(SD_CMD6_SWITCH_FUNC, cmd6_arg, response))) {
    err = wait_for_sdio_transfer(deadline);
  }
  stop_sdio_transfer();
  if (err) return err;

  // The bytes in the entire returned array must be reversed.
  const usize len = SIZEOF(switch_status);
  u32 *left = &switch_status[0], *right = &switch_status[len - 1], *end = left + len / 2;
  for (; left != end; left++, right--) {
    u32 a = *left, b = *right;
    a = u32_from_be(a), b = u32_from_be(b);
    *left = b, *right = a;
  }

  const u32 int_bits = sizeof(switch_status[0]) * 8;

  // Fortunately, the fields in the status structure are aligned to byte
  // boundaries, so we can use relatively simple bit array code for parsing.
  const u32 version_bit = 368;
  u32 struct_version =
    (switch_status[version_bit / int_bits] >> (version_bit % int_bits)) & MASK(8);

  struct SdFuncStatus tmp_status = { 0 };
  if (struct_version == 0 || struct_version == 1) {
    const u32 bit = 400 + (group - 1) * 16 + func;
    tmp_status.supported = ((switch_status[bit / int_bits] >> (bit % int_bits)) & 1) != 0;
  }
  if (struct_version == 1) {
    const u32 bit = 272 + (group - 1) * 16 + func;
    tmp_status.busy = ((switch_status[bit / int_bits] >> (bit % int_bits)) & 1) != 0;
  }
  if (struct_version == 0 || struct_version == 1) {
    const u32 bit = 376 + (group - 1) * 4;
    tmp_status.selected = ((switch_status[bit / int_bits] >> (bit % int_bits)) & MASK(4)) == func;
  }
  *out_status = tmp_status;

  return err;
}

u32 sdmmc_get_card_status(union SdmmcCSR* out_status) {
  mutex_lock(&sdio_lock);
  const struct SdmmcCard* card = sdmmc_get_card();
  u32 response[4];
  u32 err = sdmmc_command(SDMMC_CMD13_SEND_STATUS, rca_arg(card), response);
  out_status->word = response[0];
  mutex_unlock(&sdio_lock);
  return err;
}

u32 sdmmc_read(u8* buffer, u32 offset, u32 blocks, Systime deadline) {
  u32 err = SDMMC_ERROR_NONE;
  mutex_lock(&sdio_lock);
  const struct SdmmcCard* card = sdmmc_get_card();

  if (card->type != SDHC_SDXC_CARD) {
    // SDSC cards use byte (instead of block) addressing.
    offset *= SDMMC_BLOCK_SIZE;
  }

  // NOTE: For block read operations the DPSM must be configured for data
  // transfer prior to sending the respective command because the card will
  // start streaming data as soon as it has transmitted a response.
  prepare_sdio_for_transfer(SDIO_RX, buffer, blocks, SDMMC_BLOCK_SIZE);
  u32 response[4];
  enum SdmmcCommand command =
    blocks > 1 ? SDMMC_CMD18_READ_MULTIPLE_BLOCK : SDMMC_CMD17_READ_SINGLE_BLOCK;
  if (!(err = sdmmc_command(command, offset, response))) {
    err = wait_for_sdio_transfer(deadline);
  }
  stop_sdio_transfer();

  if (blocks > 1) {
    // TODO: Test the following error conditions:
    // 1. A sticky error flag being set prior to CMD18/CMD17 (e.g. ILLEGAL_CMD)
    // 2. Error while sending CMD18/CMD17, the start command is not sent at all
    u32 stop_err = sdmmc_command(SDMMC_CMD12_STOP_TRANSMISSION, 0, response);
    if (!err) err = stop_err;
  }

  check_sd_error(err);
  mutex_unlock(&sdio_lock);
  return err;
}

u32 sdmmc_write(const u8* buffer, u32 offset, u32 blocks, Systime deadline) {
  u32 err = SDMMC_ERROR_NONE;
  mutex_lock(&sdio_lock);
  const struct SdmmcCard* card = sdmmc_get_card();

  if (card->type != SDHC_SDXC_CARD) {
    // SDSC cards use byte (instead of block) addressing.
    offset *= SDMMC_BLOCK_SIZE;
  }

  u32 response[4];
  enum SdmmcCommand command =
    blocks > 1 ? SDMMC_CMD25_WRITE_MULTIPLE_BLOCK : SDMMC_CMD24_WRITE_BLOCK;
  if (!(err = sdmmc_command(command, offset, response))) {
    // NOTE: For block write operations the DPSM is configured in a different
    // order as opposed block read ones, otherwise it will begin transmission
    // while a command is still being sent.
    prepare_sdio_for_transfer(SDIO_TX, (u8*)buffer, blocks, SDMMC_BLOCK_SIZE);
    err = wait_for_sdio_transfer(deadline);
    stop_sdio_transfer();
  }

  if (blocks > 1) {
    u32 stop_err = sdmmc_command(SDMMC_CMD12_STOP_TRANSMISSION, 0, response);
    if (!err) err = stop_err;
  }

  check_sd_error(err);
  mutex_unlock(&sdio_lock);
  return err;
}

// Erases blocks in the range from `start` to `end` (inclusive).
u32 sdmmc_erase(u32 start, u32 end, Systime deadline) {
  UNUSED(deadline);
  ASSERT(start < end);
  u32 err = SDMMC_ERROR_NONE;
  mutex_lock(&sdio_lock);
  const struct SdmmcCard* card = sdmmc_get_card();

  if (!card->csd.bits.supports_erase_commands) {
    err = SDMMC_ERROR_UNSUPPORTED_FEATURE;
    goto exit;
  }

  u32 align = sdmmc_get_eraseable_sector_size(card);
  ASSERT(start % align == 0);
  ASSERT((end + 1) % align == 0);

  if (card->type != SDHC_SDXC_CARD) {
    // SDSC cards use byte (instead of block) addressing.
    start *= SDMMC_BLOCK_SIZE, end *= SDMMC_BLOCK_SIZE;
  }

  u32 response[4];
  if ((err = sdmmc_command(SD_CMD32_ERASE_WR_BLK_START, start, response))) goto exit;
  if ((err = sdmmc_command(SD_CMD33_ERASE_WR_BLK_END, end, response))) goto exit;
  if ((err = sdmmc_command(SDMMC_CMD38_ERASE, 0, response))) goto exit;

exit:
  check_sd_error(err);
  mutex_unlock(&sdio_lock);
  return err;
}

// Tries to set the SD clock to at most the given frequency.
static u32 configure_sdio_bus(u32 freq, u32 bus_width, u32 power_saving) {
  ASSERT(IS_SDIO_BUS_WIDE(bus_width) && IS_SDIO_CLOCK_POWER_SAVE(power_saving));

  u32 clkcr = READ_REG(SDIO->CLKCR);

  SET_BIT(clkcr, SDIO_CLKCR_CLKEN);
  MODIFY_REG(clkcr, SDIO_CLKCR_WIDBUS, bus_width);
  MODIFY_REG(clkcr, SDIO_CLKCR_PWRSAV, power_saving);
  // NOTE: Both clock dephasing and hardware flow control options may cause
  // data corruption and thus shouldn't be used, see errata ES0287 sections
  // 2.7.1 and 2.7.3.
  CLEAR_BIT(clkcr, SDIO_CLKCR_NEGEDGE | SDIO_CLKCR_HWFC_EN);

  // The reference manual uses two designations for clock signals related to
  // the SDIO peripheral: SDIOCLK, which is the frequency at which the
  // peripheral itself is clocked, and SDIO_CK, which is the frequency on the
  // SDIO_CK pin, used for clocking the card. The formula for calculating
  // SDIOCLK is given in RM0383 section 6.3.2. However, as the SDIO peripheral
  // shares a divider at the main PLL with USB_OTG_FS, thus it always operates
  // at 48 MHz. On the other hand, the formula for determining SDIO_CK based on
  // the parameters in SDIO_CLKCR, is given in RM0383 section 21.9.2.
  u32 sdioclk = LL_RCC_GetSDIOClockFreq(LL_RCC_SDIO_CLKSOURCE);

  // The division factor that will get us to the desired frequency or lower.
  u32 clkdiv = ceil_div(sdioclk, freq);
  u32 sdio_ck; // = the effective card clocking frequency
  if (clkdiv >= 2) {
    clkdiv = MIN(clkdiv - 2, UINT8_MAX);
    sdio_ck = sdioclk / (clkdiv + 2);
    CLEAR_BIT(clkcr, SDIO_CLKCR_BYPASS);
    MODIFY_REG(clkcr, SDIO_CLKCR_CLKDIV, clkdiv);
  } else {
    sdio_ck = sdioclk;
    SET_BIT(clkcr, SDIO_CLKCR_BYPASS);
    MODIFY_REG(clkcr, SDIO_CLKCR_CLKDIV, 0);
  }

  u32 bus_bits = 0;
  switch (bus_width) {
    case SDIO_BUS_WIDE_1B: bus_bits = 1; break;
    case SDIO_BUS_WIDE_4B: bus_bits = 4; break;
    case SDIO_BUS_WIDE_8B: bus_bits = 8; break;
  }

  u32 pclk2 = HAL_RCC_GetPCLK2Freq();
  // RM0383 section 21.3 warns that the clock frequencies must respect the
  // following condition:
  ASSERT(pclk2 >= 3 * sdio_ck / 8);

  // A check for errata ES0287 section 2.7.5: underrun conditions may occur due
  // to limited APB throughput if hardware flow control is disabled (which it
  // must be due to a different errata) and clock periods don't respect a
  // certain relationship.
  if (READ_BIT(clkcr, SDIO_CLKCR_HWFC_EN) == 0) {
    // We are dealing with periods here which are inverse of frequencies. I
    // don't want to use the FPU for a simple safeguard check, and rewriting
    // the equation to not use division operations makes it perform integer
    // multiplications which are way outside the range of u32, so instead I use
    // fixed-point math by calculating the periods in terms of nanoseconds.
    // That doesn't change the meaning of the relationship, is enough to fit
    // well within the range of 32-bit ints and can meaningfully represent
    // periods at megahertz frequencies.
    const u32 ns = 1000000000;
    ASSERT(3 * (ns / pclk2) + 3 * (ns / sdioclk) < 32 / bus_bits * (ns / sdio_ck));
  }

  char str[10];
  humanize_units(str, sizeof(str), sdio_ck);
  printf("setting SDIO_CK to %sHz\n", str);
  WRITE_REG(SDIO->CLKCR, clkcr);
  return sdio_ck;
}

// The error bits in the mapping tables are stored not as 32-bit masks (as they
// are defined in the HAL headers), but as bit positions, thereby giving a 4x
// reduction in size.
struct SdioErrorDef {
  u8 response_bit, error_bit;
};

// The functions in the STM32 HAL (and some other implementations) use a large
// sequence of else-ifs to check for every error bit in the response and return
// an appropriate error code:
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c#L1239-L1318>.
// Needless to say, this generates a lot of machine code for code that is
// normally never even executed. I replace that with a generic function that
// uses a much more compact mapping table between the specific bits in the
// responses and the HAL error codes.
static u32 sdio_check_response_error(
  u32 response, const struct SdioErrorDef* error_defs, usize error_defs_len
) {
  u32 error_bits = SDMMC_ERROR_NONE;
  for (usize i = 0; i < error_defs_len; i++) {
    if (response & BIT(error_defs[i].response_bit)) {
      error_bits |= BIT(error_defs[i].error_bit);
    }
  }
  return error_bits;
}

// Extracts the position of a set bit from a bit mask, otherwise (if the mask
// is not a power of 2, i.e. more than one bit is set) triggers a compile-time
// error.
#define ERROR_BIT(mask) \
  (__builtin_choose_expr(__builtin_popcount(mask) == 1, __builtin_ctz(mask), (void)0))

static const struct SdioErrorDef SDIO_ERRORS_R1[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_OCR_ADDR_OUT_OF_RANGE),     ERROR_BIT(SDMMC_ERROR_ADDR_OUT_OF_RANGE)    },
  { ERROR_BIT(SDMMC_OCR_ADDR_MISALIGNED),       ERROR_BIT(SDMMC_ERROR_ADDR_MISALIGNED)      },
  { ERROR_BIT(SDMMC_OCR_BLOCK_LEN_ERR),         ERROR_BIT(SDMMC_ERROR_BLOCK_LEN_ERR)        },
  { ERROR_BIT(SDMMC_OCR_ERASE_SEQ_ERR),         ERROR_BIT(SDMMC_ERROR_ERASE_SEQ_ERR)        },
  { ERROR_BIT(SDMMC_OCR_BAD_ERASE_PARAM),       ERROR_BIT(SDMMC_ERROR_BAD_ERASE_PARAM)      },
  { ERROR_BIT(SDMMC_OCR_WRITE_PROT_VIOLATION),  ERROR_BIT(SDMMC_ERROR_WRITE_PROT_VIOLATION) },
  { ERROR_BIT(SDMMC_OCR_LOCK_UNLOCK_FAILED),    ERROR_BIT(SDMMC_ERROR_LOCK_UNLOCK_FAILED)   },
  { ERROR_BIT(SDMMC_OCR_COM_CRC_FAILED),        ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)       },
  { ERROR_BIT(SDMMC_OCR_ILLEGAL_CMD),           ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)          },
  { ERROR_BIT(SDMMC_OCR_CARD_ECC_FAILED),       ERROR_BIT(SDMMC_ERROR_CARD_ECC_FAILED)      },
  { ERROR_BIT(SDMMC_OCR_CC_ERROR),              ERROR_BIT(SDMMC_ERROR_CC_ERR)               },
  { ERROR_BIT(SDMMC_OCR_GENERAL_UNKNOWN_ERROR), ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)  },
  { ERROR_BIT(SDMMC_OCR_STREAM_READ_UNDERRUN),  ERROR_BIT(SDMMC_ERROR_STREAM_READ_UNDERRUN) },
  { ERROR_BIT(SDMMC_OCR_STREAM_WRITE_OVERRUN),  ERROR_BIT(SDMMC_ERROR_STREAM_WRITE_OVERRUN) },
  { ERROR_BIT(SDMMC_OCR_CID_CSD_OVERWRITE),     ERROR_BIT(SDMMC_ERROR_CID_CSD_OVERWRITE)    },
  { ERROR_BIT(SDMMC_OCR_WP_ERASE_SKIP),         ERROR_BIT(SDMMC_ERROR_WP_ERASE_SKIP)        },
  { ERROR_BIT(SDMMC_OCR_CARD_ECC_DISABLED),     ERROR_BIT(SDMMC_ERROR_CARD_ECC_DISABLED)    },
  { ERROR_BIT(SDMMC_OCR_ERASE_RESET),           ERROR_BIT(SDMMC_ERROR_ERASE_RESET)          },
  { ERROR_BIT(SDMMC_OCR_AKE_SEQ_ERROR),         ERROR_BIT(SDMMC_ERROR_AKE_SEQ_ERR)          },
  // clang-format on
};

static const struct SdioErrorDef SDIO_ERRORS_R4[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_R4_ILLEGAL_CMD),            ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)            },
  { ERROR_BIT(SDMMC_R4_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)         },
  { ERROR_BIT(SDMMC_R4_INVALID_FUNCTION_NUM),   ERROR_BIT(SDMMC_ERROR_REQUEST_NOT_APPLICABLE) },
  { ERROR_BIT(SDMMC_R4_INVALID_PARAMETER),      ERROR_BIT(SDMMC_ERROR_INVALID_PARAMETER)      },
  { ERROR_BIT(SDMMC_R4_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)         },
  // clang-format on
};

static const struct SdioErrorDef SDIO_ERRORS_R5[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_R5_ARG_OUT_OF_RANGE),       ERROR_BIT(SDMMC_ERROR_ADDR_OUT_OF_RANGE)      },
  { ERROR_BIT(SDMMC_R5_INVALID_FUNCTION_NUM),   ERROR_BIT(SDMMC_ERROR_REQUEST_NOT_APPLICABLE) },
  { ERROR_BIT(SDMMC_R5_GENERAL_UNKNOWN_ERROR),  ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)    },
  { ERROR_BIT(SDMMC_R5_ILLEGAL_CMD),            ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)            },
  { ERROR_BIT(SDMMC_R5_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)         },
  // clang-format on
};

static const struct SdioErrorDef SDIO_ERRORS_R6[] = {
  // clang-format off
  { ERROR_BIT(SDMMC_R6_ILLEGAL_CMD),            ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)          },
  { ERROR_BIT(SDMMC_R6_COM_CRC_FAILED),         ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)       },
  { ERROR_BIT(SDMMC_R6_GENERAL_UNKNOWN_ERROR),  ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)  },
  // clang-format on
};

// Sends an SD/MMC command with a specified argument, waits for a response and
// appropriately handles the errors according to the response format parameter.
// TODO: A deadline parameter.
static u32 sdmmc_command(enum SdmmcCommand cmd, u32 arg, u32 response[4]) {
  if (unlikely(cmd & SD_ACMD)) {
    // Application-specific commands must be preceded by CMD55.
    u32 err = sdmmc_command(SDMMC_CMD55_APP_CMD, rca_arg(&sdmmc_card), response);
    if (err != SDMMC_ERROR_NONE) return err;
    union SdmmcCSR status = { .word = response[0] };
    // CMD55 must've set the bit that tells that the card now expects an ACMD.
    if (!status.bits.application_command) return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
  }

  response[3] = response[2] = response[1] = response[0] = 0;

#if SDMMC_LOG_COMMANDS
  printf("CMD%" PRIu8 "(%08" PRIX32 ") ", (u8)(cmd & SDIO_CMD_CMDINDEX_Msk), arg);
#endif

  // This clears all kinds of completion flags:
  WRITE_REG(SDIO->ICR, SDIO_CMD_INTERRUPT_FLAGS);

  // The command constants are defined in such a way that the fields CMDINDEX
  // and WAITRESP of SDIO_CMD may simply be extracted out of the constant, this
  // trick is explained in the header file.
  u32 cmd_params = cmd & (SDIO_CMD_CMDINDEX_Msk | SDIO_CMD_WAITRESP_Msk);
  // The argument register must be programmed before writing to SDIO_CMD.
  WRITE_REG(SDIO->ARG, arg);
  // This will start the CPSM and initiate the transmission of the command:
  MODIFY_REG(SDIO->CMD, CMD_CLEAR_MASK, cmd_params | SDIO_CMD_CPSMEN);

  u32 completion_flags = SDIO_STA_CMDSENT;
  if (likely(cmd & SDMMC_RESPONSE_IS_PRESENT)) {
    // NOTE: All of these are mutually exclusive and need to be checked, only
    // one of these flags will be set upon receiving the response.
    completion_flags = SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT;
  }

  Systime deadline = timeout_to_deadline(100);
  u32 sdio_sta;
  while (true) {
    sdio_sta = READ_REG(SDIO->STA);
    // Stop spinning if the command transfer is not active and any of the
    // completion flags have been set.
    if (!(sdio_sta & SDIO_STA_CMDACT) && (sdio_sta & completion_flags)) break;
    if (unlikely(systime_now() >= deadline)) return SDMMC_ERROR_TIMEOUT;
    task_yield();
  }

  if (unlikely(!(cmd & SDMMC_RESPONSE_IS_PRESENT))) {
    return SDMMC_ERROR_NONE;
  }

  if (unlikely(sdio_sta & SDIO_STA_CTIMEOUT)) {
    return SDMMC_ERROR_CMD_RSP_TIMEOUT;
  } else if (unlikely(sdio_sta & SDIO_STA_CCRCFAIL)) {
    // The check is a workaround for errata ES0287 2.7.2: the SDIO peripheral
    // calculates the CRC of the response even when the SD protocol doesn't
    // specify a field in it for the CRC, which makes all commands with such
    // response formats invariably fail with a CRC error. Since a CRC error
    // can't happen with a response without a CRC by definition, this situation
    // is treated as a success condition.
    if (likely(cmd & SDMMC_RESPONSE_HAS_CRC)) {
      return SDMMC_ERROR_CMD_CRC_FAIL;
    }
  }

  // Not every response format has a command index field.
  if (likely(cmd & SDMMC_RESPONSE_HAS_CMD_INDEX)) {
    u8 res_cmd = READ_REG(SDIO->RESPCMD) & SDIO_RESPCMD_RESPCMD_Msk;
    if (unlikely(res_cmd != (cmd & SDIO_CMD_CMDINDEX_Msk))) {
      return SDMMC_ERROR_CMD_CRC_FAIL;
    }
  }

  u32 res_r1 = READ_REG(SDIO->RESP1);
  response[0] = res_r1;
  if (unlikely(cmd & SDMMC_RESPONSE_IS_LONG)) {
    response[1] = READ_REG(SDIO->RESP2);
    response[2] = READ_REG(SDIO->RESP3);
    response[3] = READ_REG(SDIO->RESP4);
  }
  u32 resp_fmt = (cmd & SDMMC_RESPONSE_FORMAT_Msk) >> SDMMC_RESPONSE_FORMAT_Pos;

#if SDMMC_LOG_COMMANDS
  printf("=> R%" PRIu32 "(", resp_fmt);
  if (cmd & SDMMC_RESPONSE_IS_PRESENT) {
    if (cmd & SDMMC_RESPONSE_IS_LONG) {
      u32* r = response;
      printf("%08" PRIX32 ", %08" PRIX32 ", %08" PRIX32 ", %08" PRIX32, r[0], r[1], r[2], r[3]);
    } else {
      printf("%08" PRIX32, response[0]);
    }
  }
  printf(")\n");
#endif

  // All responses are first checked against a mask to see if any error bit is
  // set and whether the expensive error checking procedure should be entered.
  if (likely(resp_fmt == 1)) {
    if ((res_r1 & SDMMC_OCR_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R1, SIZEOF(SDIO_ERRORS_R1));
    }
  } else if (unlikely(resp_fmt == 4)) {
    if ((res_r1 & SDMMC_R4_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R4, SIZEOF(SDIO_ERRORS_R4));
    }
  } else if (unlikely(resp_fmt == 5)) {
    if ((res_r1 & SDMMC_R5_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R5, SIZEOF(SDIO_ERRORS_R5));
    }
  } else if (unlikely(resp_fmt == 6)) {
    if ((res_r1 & SDMMC_R6_ERRORBITS) != 0) {
      return sdio_check_response_error(res_r1, SDIO_ERRORS_R6, SIZEOF(SDIO_ERRORS_R6));
    }
  }
  return SDMMC_ERROR_NONE;
}

static void
prepare_sdio_for_transfer(enum SdioTransfer transfer, u8* buffer, u32 blocks, u32 block_size) {
  ASSERT(is_power_of_two(block_size));
  block_size = __builtin_ctz(block_size); // Basically, a cheap log2() for ints
  // The peripheral supports transferring blocks of size of at most 2^14 bytes.
  ASSERT(block_size <= 14);

  usize length = blocks << block_size; // = blocks * 2^block_size
  ASSERT(buffer != NULL);
  ASSERT((usize)buffer % 4 == 0); // Check the alignment
  ASSERT(length % 4 == 0);
  ASSERT(length > 0);
  // DMA can perform at most 65535 transfers in a single transaction, and we
  // are transferring 4 bytes at a time, so check for that.
  ASSERT(length / 4 <= UINT16_MAX);

  // Ensure that the stream is inactive.
  ASSERT(!dma_is_stream_enabled(SDIO_DMA));
  // Clear any leftover pending interrupt flags (a requirement for restarting the stream).
  dma_clear_interrupt_flags(SDIO_DMA, DMA_ALL_INTERRUPT_FLAGS);
  // Clear the SDIO interrupt flags.
  WRITE_REG(SDIO->ICR, SDIO_DATA_INTERRUPT_FLAGS);

  const struct DmaConfig config = {
    .channel = 4,
    .direction = transfer == SDIO_RX ? DMA_PERIPH_TO_MEMORY_DIR : DMA_MEMORY_TO_PERIPH_DIR,
    .mode = DMA_PERIPHERAL_FLOW_CONTROL,
    .periph_addr_increment = false,
    .memory_addr_increment = true,
    .periph_data_size = DMA_WORD_DATA,
    .memory_data_size = DMA_WORD_DATA,
    .priority = DMA_MEDIUM_PRIORITY,
    .enable_fifo = true,
    .fifo_threshold = DMA_FULL_FIFO_THRESHOLD,
    .periph_burst = DMA_BURST_INCR4,
    .memory_burst = DMA_BURST_INCR4,
    .double_buffer_mode = false,
  };
  dma_configure_stream(SDIO_DMA, &config);

  // Disable all DMA interrupts.
  CLEAR_BIT(SDIO_DMA->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  CLEAR_BIT(SDIO_DMA->FCR, DMA_SxFCR_FEIE);

  // The length parameter here is apparently ignored - the termination of the
  // transfer is managed by the peripheral connected to the flow controller.
  dma_configure_transfer(SDIO_DMA, (usize)&SDIO->FIFO, (usize)buffer, length / 4);
  dma_enable_stream(SDIO_DMA);

  WRITE_REG(SDIO->DTIMER, UINT32_MAX); // TODO: Calculate read/write timeouts.
  WRITE_REG(SDIO->DLEN, length);

  u32 dctrl = READ_REG(SDIO->DCTRL);
  CLEAR_BIT(dctrl, DCTRL_CLEAR_MASK);
  // Transfer direction: 0 - controller to card, 1 - card to controller.
  MODIFY_REG(dctrl, SDIO_DCTRL_DTDIR, (transfer == SDIO_RX ? 1 : 0) << SDIO_DCTRL_DTDIR_Pos);
  CLEAR_BIT(dctrl, SDIO_DCTRL_DTMODE); // Block transfer mode
  MODIFY_REG(dctrl, SDIO_DCTRL_DBLOCKSIZE, block_size << SDIO_DCTRL_DBLOCKSIZE_Pos);
  SET_BIT(dctrl, SDIO_DCTRL_DMAEN | SDIO_DCTRL_DTEN);
  WRITE_REG(SDIO->DCTRL, dctrl);
}

static void stop_sdio_transfer(void) {
  dma_disable_stream(SDIO_DMA);
  // Confirm that the stream has been completely halted (encouraged by the
  // reference manual).
  while (dma_is_stream_enabled(SDIO_DMA)) {
    task_yield();
  }
  CLEAR_BIT(SDIO->DCTRL, SDIO_DCTRL_DMAEN | SDIO_DCTRL_DTEN);
}

static u32 wait_for_sdio_transfer(Systime deadline) {
  while (true) {
    SET_BIT(SDIO->MASK, SDIO_DATA_INTERRUPT_FLAGS);
    // TODO: Race condition here. What if the interrupt fires before the task goes to sleep?
    task_wait(&sdio_notification, deadline);

    u32 sdio_sta = READ_REG(SDIO->STA);
    u32 dma_isr = dma_get_interrupt_flags(SDIO_DMA);
    u32 error = 0;
    if (unlikely(sdio_sta & SDIO_DATA_ERROR_FLAGS)) {
      if (sdio_sta & SDIO_STA_DCRCFAIL) error |= SDMMC_ERROR_DATA_CRC_FAIL;
      if (sdio_sta & SDIO_STA_DTIMEOUT) error |= SDMMC_ERROR_DATA_TIMEOUT;
      if (sdio_sta & SDIO_STA_TXUNDERR) error |= SDMMC_ERROR_TX_UNDERRUN;
      if (sdio_sta & SDIO_STA_RXOVERR) error |= SDMMC_ERROR_RX_OVERRUN;
      if (sdio_sta & SDIO_STA_STBITERR) error |= SDMMC_ERROR_DATA_TIMEOUT;
    }
    // NOTE: The FEIF (FIFO error) flag is purposefully ignored here. For some
    // reason it is always generated when the flow controller is on (this is
    // not documented in the errata).
    if (unlikely(dma_isr & (DMA_FLAG_TEIF | DMA_FLAG_DMEIF))) {
      error |= SDMMC_ERROR_DMA;
    }
    if (unlikely(error != 0)) {
      return error;
    }

    // As far as I understand:
    // 1. the DBCKEND flag is set after every single 512 byte block is transferred,
    // 2. and the DATAEND flag is set at the very end of the whole transfer.
    if (likely(sdio_sta & SDIO_STA_DATAEND)) {
      // Confirm that everything has been written to memory by observing the
      // transfer-complete flag on the DMA stream.
      while (!(dma_isr & DMA_FLAG_TCIF)) {
        task_yield();
        dma_isr = dma_get_interrupt_flags(SDIO_DMA);
      }
      return SDMMC_ERROR_NONE;
    }
    if (unlikely(systime_now() >= deadline)) {
      return SDMMC_ERROR_TIMEOUT;
    }
  }
}

void SDIO_IRQHandler(void) {
  // A brilliant trick for simplifying the interrupt logic is used here:
  // instead of clearing the interrupt flags and backing them up in some
  // variable for checking in the normal driver code later, we simply disable
  // the interrupt! It was taken from
  // <https://github.com/embassy-rs/embassy/blob/2a4ebdc150ba7c32d7dacc12aec85bccf05a41ff/embassy-stm32/src/sdmmc/mod.rs#L28-L40>
  CLEAR_BIT(SDIO->MASK, SDIO_DATA_INTERRUPT_FLAGS | SDIO_CMD_INTERRUPT_FLAGS);
  if (task_notify(&sdio_notification)) {
    task_yield_from_isr();
  }
}

__NO_RETURN void crash_on_sd_error(u32 code, const char* file, u32 line) {
  static const char* const ERROR_NAMES[] = {
    [ERROR_BIT(SDMMC_ERROR_CMD_CRC_FAIL)] = "CMD_CRC_FAIL",
    [ERROR_BIT(SDMMC_ERROR_DATA_CRC_FAIL)] = "DATA_CRC_FAIL",
    [ERROR_BIT(SDMMC_ERROR_CMD_RSP_TIMEOUT)] = "CMD_RSP_TIMEOUT",
    [ERROR_BIT(SDMMC_ERROR_DATA_TIMEOUT)] = "DATA_TIMEOUT",
    [ERROR_BIT(SDMMC_ERROR_TX_UNDERRUN)] = "TX_UNDERRUN",
    [ERROR_BIT(SDMMC_ERROR_RX_OVERRUN)] = "RX_OVERRUN",
    [ERROR_BIT(SDMMC_ERROR_ADDR_MISALIGNED)] = "ADDR_MISALIGNED",
    [ERROR_BIT(SDMMC_ERROR_BLOCK_LEN_ERR)] = "BLOCK_LEN_ERR",
    [ERROR_BIT(SDMMC_ERROR_ERASE_SEQ_ERR)] = "ERASE_SEQ_ERR",
    [ERROR_BIT(SDMMC_ERROR_BAD_ERASE_PARAM)] = "BAD_ERASE_PARAM",
    [ERROR_BIT(SDMMC_ERROR_WRITE_PROT_VIOLATION)] = "WRITE_PROT_VIOLATION",
    [ERROR_BIT(SDMMC_ERROR_LOCK_UNLOCK_FAILED)] = "LOCK_UNLOCK_FAILED",
    [ERROR_BIT(SDMMC_ERROR_COM_CRC_FAILED)] = "COM_CRC_FAILED",
    [ERROR_BIT(SDMMC_ERROR_ILLEGAL_CMD)] = "ILLEGAL_CMD",
    [ERROR_BIT(SDMMC_ERROR_CARD_ECC_FAILED)] = "CARD_ECC_FAILED",
    [ERROR_BIT(SDMMC_ERROR_CC_ERR)] = "CC_ERR",
    [ERROR_BIT(SDMMC_ERROR_GENERAL_UNKNOWN_ERR)] = "GENERAL_UNKNOWN_ERR",
    [ERROR_BIT(SDMMC_ERROR_STREAM_READ_UNDERRUN)] = "STREAM_READ_UNDERRUN",
    [ERROR_BIT(SDMMC_ERROR_STREAM_WRITE_OVERRUN)] = "STREAM_WRITE_OVERRUN",
    [ERROR_BIT(SDMMC_ERROR_CID_CSD_OVERWRITE)] = "CID_CSD_OVERWRITE",
    [ERROR_BIT(SDMMC_ERROR_WP_ERASE_SKIP)] = "WP_ERASE_SKIP",
    [ERROR_BIT(SDMMC_ERROR_CARD_ECC_DISABLED)] = "CARD_ECC_DISABLED",
    [ERROR_BIT(SDMMC_ERROR_ERASE_RESET)] = "ERASE_RESET",
    [ERROR_BIT(SDMMC_ERROR_AKE_SEQ_ERR)] = "AKE_SEQ_ERR",
    [ERROR_BIT(SDMMC_ERROR_INVALID_VOLTRANGE)] = "INVALID_VOLTRANGE",
    [ERROR_BIT(SDMMC_ERROR_ADDR_OUT_OF_RANGE)] = "ADDR_OUT_OF_RANGE",
    [ERROR_BIT(SDMMC_ERROR_REQUEST_NOT_APPLICABLE)] = "REQUEST_NOT_APPLICABLE",
    [ERROR_BIT(SDMMC_ERROR_INVALID_PARAMETER)] = "INVALID_PARAMETER",
    [ERROR_BIT(SDMMC_ERROR_UNSUPPORTED_FEATURE)] = "UNSUPPORTED_FEATURE",
    [ERROR_BIT(SDMMC_ERROR_BUSY)] = "BUSY",
    [ERROR_BIT(SDMMC_ERROR_DMA)] = "DMA",
    [ERROR_BIT(SDMMC_ERROR_TIMEOUT)] = "TIMEOUT",
  };

  char msg[64];
  usize msg_pos = 0;
  i32 msg_written = snprintf(msg, sizeof(msg), "0x%08" PRIX32, code);
  msg_pos += MAX(0, msg_written);

  if (code == 0) {
    code = SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
  }
  while (code != 0) {
    u32 bit = 31 - __CLZ(code);
    code ^= BIT(bit);

    const char* name = bit < SIZEOF(ERROR_NAMES) ? ERROR_NAMES[bit] : "UNKNOWN";
    msg_written = snprintf(msg + msg_pos, sizeof(msg) - msg_pos, "/SD_%s", name);
    msg_pos += MAX(0, msg_written);
  }

  crash(msg, file, line);
}
