#pragma once

#include "stmes/kernel/crash.h"
#include "stmes/kernel/time.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_sdmmc.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SDMMC_BLOCK_SIZE 512u

// Some constants that aren't defined in the HAL headers:
// clang-format off
#define SDMMC_R4_ILLEGAL_CMD            0x00000004u
#define SDMMC_R4_COM_CRC_FAILED         0x00000008u
#define SDMMC_R4_INVALID_FUNCTION_NUM   0x00000010u
#define SDMMC_R4_INVALID_PARAMETER      0x00000040u
#define SDMMC_R4_ERRORBITS              0x0000005Cu

#define SDMMC_R5_ARG_OUT_OF_RANGE       0x00000001u
#define SDMMC_R5_INVALID_FUNCTION_NUM   0x00000002u
#define SDMMC_R5_GENERAL_UNKNOWN_ERROR  0x00000008u
#define SDMMC_R5_ILLEGAL_CMD            0x00000040u
#define SDMMC_R5_COM_CRC_FAILED         0x00000080u
#define SDMMC_R5_ERRORBITS              0x000000CBu

#define SDMMC_R6_ERRORBITS              0x0000E000u
// clang-format on

// The bit layout of the command definitions is as follows:
//
// 1. [5:0] - the command index, 0-63
// 2. [6] - set if the command returns a response
// 3. [7] - 1 if the response format is long, 0 if it is short
// 4. [8] - the response has a CRC field
// 5. [9] - the response has a command index field
// 6. [12:10] - the response format, 0-7
//
// Notice how the first 8 bits align with the bit definitions of the SDIO_CMD
// register, which are:
//
// 1. [5:0] - CMDINDEX (is exactly the same)
// 2. [7:6] - WAITRESP, with the following variants:
//    1. 00 - no response
//    2. 01 - short response
//    3. 10 - no response
//    4. 11 - long response
//    This field may be re-interpreted as a combination of two flags, which can
//    be checked separately.
//
#define SDMMC_RESPONSE_HAS_CRC BIT(8)
#define SDMMC_RESPONSE_HAS_CMD_INDEX BIT(9)
#define SDMMC_RESPONSE_FORMAT_Pos 10
#define SDMMC_RESPONSE_FORMAT_Msk (MASK(3) << SDMMC_RESPONSE_FORMAT_Pos)
#define SDMMC_RESPONSE_FORMAT(n) ((n) << SDMMC_RESPONSE_FORMAT_Pos)
#define SDMMC_SHORT_RESPONSE_FMT(n) (SDMMC_RESPONSE_FORMAT(n) | (1u << SDIO_CMD_WAITRESP_Pos))
#define SDMMC_LONG_RESPONSE_FMT(n) (SDMMC_RESPONSE_FORMAT(n) | (3u << SDIO_CMD_WAITRESP_Pos))
#define SDMMC_NO_RESPONSE_FMT(n) (SDMMC_RESPONSE_FORMAT(n) | (0u << SDIO_CMD_WAITRESP_Pos))
#define SDMMC_RESPONSE_IS_PRESENT SDIO_CMD_WAITRESP_0
#define SDMMC_RESPONSE_IS_LONG SDIO_CMD_WAITRESP_1

enum SdmmcResponse {
  SD_CMD_NO_RESP = SDMMC_NO_RESPONSE_FMT(0),
  SD_CMD_R1 = SDMMC_SHORT_RESPONSE_FMT(1) | SDMMC_RESPONSE_HAS_CRC | SDMMC_RESPONSE_HAS_CMD_INDEX,
  SD_CMD_R1B = SD_CMD_R1,
  SD_CMD_R2 = SDMMC_LONG_RESPONSE_FMT(2) | SDMMC_RESPONSE_HAS_CRC,
  SD_CMD_R3 = SDMMC_SHORT_RESPONSE_FMT(3),
  SD_CMD_R4 = SDMMC_SHORT_RESPONSE_FMT(4),
  SD_CMD_R5 = SDMMC_SHORT_RESPONSE_FMT(5) | SDMMC_RESPONSE_HAS_CRC | SDMMC_RESPONSE_HAS_CMD_INDEX,
  SD_CMD_R6 = SDMMC_SHORT_RESPONSE_FMT(6) | SDMMC_RESPONSE_HAS_CRC | SDMMC_RESPONSE_HAS_CMD_INDEX,
  SD_CMD_R7 = SDMMC_SHORT_RESPONSE_FMT(7) | SDMMC_RESPONSE_HAS_CRC | SDMMC_RESPONSE_HAS_CMD_INDEX,
};

// The command definitions were written by combining information from:
// <https://asf.microchip.com/docs/latest/samd21/html/group__sd__mmc__protocol.html>
// <https://elinux.org/images/d/d3/Mmc_spec.pdf>
// <https://www.sdcard.org/cms/wp-content/themes/sdcard-org/dl.php?f=Part1_Physical_Layer_Simplified_Specification_Ver9.00.pdf>
// <https://www.sdcard.org/cms/wp-content/themes/sdcard-org/dl.php?f=PartE1_SDIO_Simplified_Specification_Ver3.00.pdf>
// <https://www.kingston.com/datasheets/SDCIT-specsheet-8gb-32gb_en.pdf>
// <https://nz.apexelex.com/specs/misc_items/KLMxGxxETx-B041(eMMC5.1%20e.MMC)1.1.pdf>
// <https://linux.codingbelief.com/zh/storage/flash_memory/emmc/emmc_commands.html>
// <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_sdmmc.h#L168-L238>
// Command names are prefixed with `SDMMC` if they are supported by both SD and
// MMC cards, with just `SD` or `MMC` if they are implemented by only SD or MMC
// cards (respectively), and with `SDIO` if they are implemented by SDIO cards.
enum SdmmcCommand {
  SDMMC_CMD0_GO_IDLE_STATE = 0 | SD_CMD_NO_RESP,
  MMC_CMD1_SEND_OP_COND = 1 | SD_CMD_R3,
  SDMMC_CMD2_ALL_SEND_CID = 2 | SD_CMD_R2,
  SD_CMD3_SEND_RELATIVE_ADDR = 3 | SD_CMD_R6,
  MMC_CMD3_SET_RELATIVE_ADDR = 3 | SD_CMD_R1,
  SDMMC_CMD4_SET_DSR = 4 | SD_CMD_NO_RESP,
  MMC_CMD5_SLEEP_AWAKE = 5 | SD_CMD_R1B,
  SDIO_CMD5_IO_SEND_OP_COND = 5 | SD_CMD_R4,
  SD_CMD6_SWITCH_FUNC = 6 | SD_CMD_R1,
  SD_ACMD6_SET_BUS_WIDTH = 6 | SD_CMD_R1,
  MMC_CMD6_SWITCH = 6 | SD_CMD_R1B,
  SDMMC_CMD7_SELECT_CARD = 7 | SD_CMD_R1B,
  SDMMC_CMD7_DESELECT_CARD = 7 | SD_CMD_NO_RESP,
  SD_CMD8_SEND_IF_COND = 8 | SD_CMD_R7,
  MMC_CMD8_SEND_EXT_CSD = 8 | SD_CMD_R1,
  SDMMC_CMD9_SEND_CSD = 9 | SD_CMD_R2,
  SDMMC_CMD10_SEND_CID = 10 | SD_CMD_R2,
  SD_CMD11_VOLTAGE_SWITCH = 11 | SD_CMD_R1,
  MMC_CMD11_READ_DAT_UNTIL_STOP = 11 | SD_CMD_R1,
  SDMMC_CMD12_STOP_TRANSMISSION = 12 | SD_CMD_R1B,
  SDMMC_CMD13_SEND_STATUS = 13 | SD_CMD_R1,
  SD_ACMD13_SD_STATUS = 13 | SD_CMD_R1,
  MMC_CMD14_BUSTEST_R = 14 | SD_CMD_R1,
  SDMMC_CMD15_GO_INACTIVE_STATE = 15 | SD_CMD_NO_RESP,
  SDMMC_CMD16_SET_BLOCKLEN = 16 | SD_CMD_R1,
  SDMMC_CMD17_READ_SINGLE_BLOCK = 17 | SD_CMD_R1,
  SDMMC_CMD18_READ_MULTIPLE_BLOCK = 18 | SD_CMD_R1,
  SD_CMD19_SEND_TUNING_BLOCK = 19 | SD_CMD_R1,
  MMC_CMD19_BUSTEST_W = 19 | SD_CMD_R1,
  SD_CMD20_SPEED_CLASS_CONTROL = 20 | SD_CMD_R1B,
  MMC_CMD20_WRITE_DAT_UNTIL_STOP = 20 | SD_CMD_R1,
  SD_CMD22_ADDRESS_EXTENSION = 22 | SD_CMD_R1,
  SD_ACMD22_SEND_NUM_WR_BLOCKS = 22 | SD_CMD_R1,
  SDMMC_CMD23_SET_BLOCK_COUNT = 23 | SD_CMD_R1,
  SD_ACMD23_SET_WR_BLK_ERASE_COUNT = 23 | SD_CMD_R1,
  SDMMC_CMD24_WRITE_BLOCK = 24 | SD_CMD_R1,
  SDMMC_CMD25_WRITE_MULTIPLE_BLOCK = 25 | SD_CMD_R1,
  SDMMC_CMD26_PROGRAM_CID = 26 | SD_CMD_R1,
  SDMMC_CMD27_PROGRAM_CSD = 27 | SD_CMD_R1,
  SDMMC_CMD28_SET_WRITE_PROT = 28 | SD_CMD_R1B,
  SDMMC_CMD29_CLR_WRITE_PROT = 29 | SD_CMD_R1B,
  SDMMC_CMD30_SEND_WRITE_PROT = 30 | SD_CMD_R1,
  SD_CMD32_ERASE_WR_BLK_START = 32 | SD_CMD_R1,
  MMC_CMD32_TAG_SECTOR_START = 32 | SD_CMD_R1,
  SD_CMD33_ERASE_WR_BLK_END = 33 | SD_CMD_R1,
  MMC_CMD33_TAG_SECTOR_END = 33 | SD_CMD_R1,
  MMC_CMD34_UNTAG_SECTOR = 34 | SD_CMD_R1,
  MMC_CMD35_TAG_ERASE_GROUP_START = 35 | SD_CMD_R1,
  MMC_CMD36_TAG_ERASE_GROUP_END = 36 | SD_CMD_R1,
  MMC_CMD37_UNTAG_ERASE_GROUP = 37 | SD_CMD_R1,
  SDMMC_CMD38_ERASE = 38 | SD_CMD_R1B,
  SD_CMD39_SELECT_CARD_PARTITION = 39 | SD_CMD_R1B,
  MMC_CMD39_FAST_IO = 39 | SD_CMD_R4,
  MMC_CMD40_GO_IRQ_STATE = 40 | SD_CMD_R5,
  SD_ACMD41_SD_SEND_OP_COND = 41 | SD_CMD_R3,
  SD_ACMD42_SET_CLR_CARD_DETECT = 42 | SD_CMD_R1,
  SD_CMD42_LOCK_UNLOCK = 42 | SD_CMD_R1,
  MMC_CMD42_LOCK_UNLOCK = 42 | SD_CMD_R1B,
  SD_CMD48_READ_EXTR_SINGLE = 48 | SD_CMD_R1,
  SD_CMD49_WRITE_EXTR_SINGLE = 49 | SD_CMD_R1,
  SD_ACMD51_SEND_SCR = 51 | SD_CMD_R1,
  SDIO_CMD52_IO_RW_DIRECT = 52 | SD_CMD_R5,
  SDIO_CMD53_IO_RW_EXTENDED = 53 | SD_CMD_R5,
  SDMMC_CMD55_APP_CMD = 55 | SD_CMD_R1,
  SDMMC_CMD56_GEN_CMD = 56 | SD_CMD_R1,
  SD_CMD58_READ_EXTR_MULTI = 58 | SD_CMD_R1,
  SD_CMD59_WRITE_EXTR_MULTI = 59 | SD_CMD_R1,
};

// <https://stackoverflow.com/q/41253759>
// <https://gcc.gnu.org/gcc-4.4/changes.html>

// Card Identification register.
union SdmmcCID {
  u32 words[4];
  struct __packed SdCID {
    u32 : 1;
    u32 crc7_checksum : 7;
    u32 manufacturing_month : 4;
    u32 manufacturing_year : 8;
    u32 : 4;
    u32 serial_number : 32;
    u32 product_revision : 8;
    char product_name[5];
    char oem_application_id[2];
    u32 manufacturer_id : 8;
  } sd;
};

// Card-Specific Data register.
union SdmmcCSD {
  u32 words[4];

  struct __packed SdmmcCSDBits {
    u32 : 1;
    u32 crc7 : 7;
    u32 : 1;

    bool write_protect_until_power_cycle : 1; // not in MMC

    enum {
      SD_CSD_HARD_DISK_FS_FORMAT = 0,
      SD_CSD_DOS_FAT_FORMAT = 1,
      SD_CSD_UNIVERSAL_FILE_FORMAT = 2,
      SD_CSD_UNKNOWN_FILE_FORMAT = 3,
    } file_format : 2;

    bool temporary_write_protect : 1;
    bool permanent_write_protect : 1;
    bool contents_were_copied : 1;

    u32 file_format_group : 1;

    u32 : 5;

    bool allows_partial_block_write : 1;
    u32 max_write_block_length : 4;

    u32 r2w_factor : 3;

    u32 : 2;

    bool supports_write_protect_groups : 1;
    u32 write_protect_group_size : 7; // 5 in MMC

    u32 eraseable_sector_size : 7;        // 5 in MMC
    bool supports_single_block_erase : 1; // not in MMC

    u32 : 29;

    bool implements_dsr : 1;

    bool allows_misaligned_block_read : 1;
    bool allows_misaligned_block_write : 1;
    bool allows_partial_block_read : 1;
    u32 max_read_block_length : 4;

    bool supports_basic_commands : 1;
    bool supports_cmd_queue_commands : 1;
    bool supports_block_read_commands : 1;
    bool : 1;
    bool supports_block_write_commands : 1;
    bool supports_erase_commands : 1;
    bool supports_write_protect_commands : 1;
    bool supports_card_lock_commands : 1;
    bool supports_application_commands : 1;
    bool supports_io_mode_commands : 1;
    bool supports_switch_commands : 1;
    bool : 1;

    u32 transfer_speed_unit : 3;
    u32 transfer_speed_factor : 4;
    u32 : 1;

    u32 nsac : 8;
    u32 taac : 8;

    u32 : 6;

    enum {
      SD_CSD_VERSION_1_0 = 0,
      SD_CSD_VERSION_2_0 = 1,
      SD_CSD_VERSION_3_0 = 2,
    } structure_version : 2;
  } bits;

  struct __packed MmcCSD {
    u32 : 8;
    u32 ecc : 2;
    u32 : 19;
    u32 default_ecc : 2;
    u32 : 1;
    u32 wp_grp_size : 5;
    u32 erase_grp_size : 5;
    u32 sector_size : 5;
    u32 c_size_mult : 3;
    u32 vdd_w_curr_max : 3;
    u32 vdd_w_curr_min : 3;
    u32 vdd_r_curr_max : 3;
    u32 vdd_r_curr_min : 3;
    u32 c_size : 12;
    u32 : 16, : 32;
    u32 spec_vers : 4;
    u32 : 2;
  } mmc;

  struct __packed SdCSDv1 {
    u32 : 32, : 15;
    u32 card_size_multiplier : 3;
    u32 max_write_current_at_max_voltage : 3;
    u32 max_write_current_at_min_voltage : 3;
    u32 max_read_current_at_max_voltage : 3;
    u32 max_read_current_at_min_voltage : 3;
    u32 card_size : 12;
    u32 : 22, : 32;
  } sd_v1;

  struct __packed SdCSDv2 {
    u32 : 32, : 16;
    u32 card_size : 22;
    u32 : 26, : 32;
  } sd_v2;

  struct __packed SdCSDv3 {
    u32 : 32, : 16;
    u32 card_size : 28;
    u32 : 20, : 32;
  } sd_v3;
};

// SD Configuration Register.
union SdmmcSCR {
  u32 words[2];
  struct __packed SdSCRv1 {
    u32 manufacturer_reserved : 32;

    bool supports_speed_class_control_cmd : 1;
    bool supports_set_block_count_cmd : 1;
    bool supports_ext_register_single_block_cmds : 1;
    bool supports_ext_register_multi_block_cmds : 1;
    bool supports_secure_receive_send_cmds : 1;
    u32 : 1;

    u32 sd_specx : 4;
    u32 sd_spec4 : 1;

    // These are defined in a spec I can't get my hands on, and not that I care
    // about DRM functionality anyway.
    u32 extended_security_support_bits : 4;

    u32 sd_spec3 : 1;

    bool supports_1bit_wide_bus : 1;
    bool : 1;
    bool supports_4bit_wide_bus : 1;
    bool : 1;

    enum {
      SD_NO_SECURITY_SPEC_SUPPORT = 0,
      SD_SECURITY_SPEC_NOT_USED = 1,
      SD_SECURITY_SPEC_V1 = 2,
      SD_SECURITY_SPEC_V2 = 3,
      SD_SECURITY_SPEC_V3 = 4,
    } security_spec_version : 3;

    // Determines the data stored on the card after an erase operation: 0 if
    // the erased blocks will be zeroed out, 1 if they will be filled with ones.
    u32 erased_data_state : 1;

    u32 sd_spec : 4;

    enum {
      SD_SCR_VERSION_1_0 = 0,
    } structure_version : 4;
  } v1;
};

// SD Status Register.
union SdmmcSSR {
  u32 words[16];
  struct __packed SdSSRBits {
    u8 manufacturer_reserved[39];
    u32 : 32;
    u32 : 32;
    u32 : 16;
    u32 uhs_au_size : 4;
    enum {
      SD_UHS_SPEED_LESS_THAN_10MB = 0,
      SD_UHS_SPEED_MORE_THAN_10MB = 1,
    } uhs_speed_grade : 4;
    u32 erase_offset : 2;
    u32 erase_timeout : 6;
    u32 erase_size : 16;
    u32 : 4;
    u32 au_size : 4;
    u32 performance_move : 8;
    enum {
      SD_SPEED_CLASS_0 = 0,
      SD_SPEED_CLASS_2 = 1,
      SD_SPEED_CLASS_4 = 2,
      SD_SPEED_CLASS_6 = 3,
      SD_SPEED_CLASS_10 = 4,
    } speed_class : 8;
    u32 size_of_protected_area : 32;
    u32 sd_card_type : 16;
    u32 : 6;
    u32 : 7;
    bool secured_mode : 1;
    enum {
      SD_SSR_BUS_WIDTH_1BIT = 0,
      SD_SSR_BUS_WIDTH_4BIT = 2,
    } data_bus_width : 2;
  } bits;
};

enum __packed SdSpecVersion {
  SD_SPEC_UNKNOWN_VERSION = 0,
  SD_SPEC_V1_0,
  SD_SPEC_V1_1,
  SD_SPEC_V2_0,
  SD_SPEC_V3_X,
  SD_SPEC_V4_X,
  SD_SPEC_V5_X,
  SD_SPEC_V6_X,
  SD_SPEC_V7_X,
  SD_SPEC_V8_X,
  SD_SPEC_V9_X,
};

enum __packed SdmmcCardState {
  SDMMC_STATE_IDLE = 0,
  SDMMC_STATE_READY = 1,
  SDMMC_STATE_IDENTIFICATION = 2,
  SDMMC_STATE_STANDBY = 3,
  SDMMC_STATE_TRANSFER = 4,
  SDMMC_STATE_SENDING = 5,
  SDMMC_STATE_RECEIVING = 6,
  SDMMC_STATE_PROGRAMMING = 7,
  SDMMC_STATE_DISCONNECTED = 8,
  SDMMC_STATE_ONLY_SDIO = 15,
};

// Card Status Register.
union SdmmcCSR {
  u32 word;
  struct SdmmcCSRBits {
    bool : 1; // reserved for a manufacturer test mode
    bool : 1; // reserved for a manufacturer test mode as well
    bool : 1; // reserved for application-specific commands
    bool authentication_sequence_error : 1;
    bool : 1; // reserved for SDIO, appears unused
    bool application_command : 1;
    bool extension_function_event : 1;
    bool mmc_switch_error : 1;
    bool ready_for_data : 1;
    enum SdmmcCardState current_state : 4;
    bool erase_sequence_cancelled : 1;
    bool internal_ecc_is_disabled : 1;
    bool tried_erasing_write_protected : 1;
    bool csd_overwrite_error : 1;
    bool mmc_stream_write_overrun : 1;
    bool mmc_stream_read_underrun : 1;
    bool general_unknown_error : 1;
    bool internal_controller_error : 1;
    bool internal_ecc_failure : 1;
    bool prev_command_was_illegal : 1;
    bool prev_command_crc_check_failed : 1;
    bool lock_unlock_failed : 1;
    bool card_is_locked : 1;
    bool write_protection_violation : 1;
    bool invalid_erase_parameters : 1;
    bool incorrect_erase_sequence : 1;
    bool block_transfer_length_error : 1;
    bool misaligned_block_address_error : 1;
    bool out_of_range_block_error : 1;
  } bits;
};

// Argument of CMD8.
union SdmmcIfCond {
  u32 word;
  struct SdmmcIfCondBits {
    u32 check_pattern : 8;
    enum {
      SD_VOLTAGE_RANGE_2_7V_TO_3_6V = BIT(0),
      SD_LOW_VOLTAGE_RANGE = BIT(1),
    } host_voltage_supply : 4;
    u32 : 20;
  } bits;
};

// Operation Conditions Register.
union SdmmcOCR {
  u32 word;
  struct SdmmcOCRBits {
    u32 : 15;
    bool voltage_window_2_7v_to_2_8v : 1;
    bool voltage_window_2_8v_to_2_9v : 1;
    bool voltage_window_2_9v_to_3_0v : 1;
    bool voltage_window_3_0v_to_3_1v : 1;
    bool voltage_window_3_1v_to_3_2v : 1;
    bool voltage_window_3_2v_to_3_3v : 1;
    bool voltage_window_3_3v_to_3_4v : 1;
    bool voltage_window_3_4v_to_3_5v : 1;
    bool voltage_window_3_5v_to_3_6v : 1;
    bool switching_to_1_8v_status : 1;
    bool : 1;
    bool : 1;
    bool over_2tb_capacity_status : 1;
    enum {
      SDXC_POWER_SAVING = 0,
      SDXC_MAXIMUM_PERFORMANCE = 1,
    } sdxc_power_control : 1;
    bool uhs2_status : 1;
    bool high_capacity_status : 1;
    bool power_up_status : 1;
  } bits;
};

enum __packed SdmmcCardType {
  SDMMC_UNKNOWN_CARD = 0,
  SDSC_V1_X_CARD,
  SDSC_V2_X_CARD,
  SDHC_SDXC_CARD,
  MMC_CARD,
  SDIO_CARD,
};

struct SdmmcCard {
  enum SdmmcCardType type;
  enum SdSpecVersion spec_version;
  u16 rca;
  u32 clock_freq;
  union SdmmcCID cid;
  union SdmmcCSD csd;
  union SdmmcSCR scr;
};

// Information about the physical and electrical capabilities of the host.
struct SdmmcHostCapabilities {
  bool high_speed_mode;
  bool use_4bit_data_bus;
};

void sdmmc_init_gpio(void);
bool sdmmc_is_card_inserted(void);

const struct SdmmcCard* sdmmc_get_card(void);
u32 sdmmc_get_blocks_count(const struct SdmmcCard* card);
u32 sdmmc_get_eraseable_sector_size(const struct SdmmcCard* card);
u32 sdmmc_max_transfer_freq(const struct SdmmcCard* card);
enum SdSpecVersion sdmmc_sd_spec_version(const struct SdmmcCard* card);

u32 sdmmc_init_card(const struct SdmmcHostCapabilities* host_caps);
u32 sdmmc_get_card_status(union SdmmcCSR* out_status);
u32 sdmmc_read(u8* buffer, u32 offset, u32 blocks, Systime deadline);
u32 sdmmc_write(const u8* buffer, u32 offset, u32 blocks, Systime deadline);
u32 sdmmc_erase(u32 start, u32 end, Systime deadline);

__NO_RETURN void crash_on_sd_error(u32 code, const char* file, u32 line);

#define check_sd_error(expr)                           \
  do {                                                 \
    u32 __code__ = (expr);                             \
    if (unlikely(__code__ != SDMMC_ERROR_NONE)) {      \
      crash_collect_registers();                       \
      crash_on_sd_error(__code__, __FILE__, __LINE__); \
    }                                                  \
  } while (0)

#ifdef __cplusplus
}
#endif
