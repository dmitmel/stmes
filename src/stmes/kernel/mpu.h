#pragma once

#include "stmes/utils.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// Values for Cortex-M4:
#define MPU_MAX_REGIONS_NUMBER 8
#define MPU_MIN_REGION_SIZE 32

#define MPU_REGION_FLASH_NUM 1
#define MPU_REGION_FLASH_BASE 0x08000000ul
#define MPU_REGION_FLASH_END 0x0807fffful

#define MPU_REGION_SRAM1_NUM 2
#define MPU_REGION_SRAM1_BASE 0x20000000ul
#define MPU_REGION_SRAM1_END 0x2001fffful

#define MPU_REGION_PERIPH_NUM 3
#define MPU_REGION_PERIPH_BASE 0x40000000ul
#define MPU_REGION_PERIPH_END 0x5ffffffful

#define MPU_REGION_STACK_BARRIER 4
#define MPU_REGION_STACK_BARRIER_SIZE 32

void mpu_init(void);
void mpu_enable_region(u8 region_nr);
void mpu_disable_region(u8 region_nr);
bool mpu_match_region(usize address, u32* out_attrs);

enum __packed MpuFaultDiagnosis {
  MPU_FAULT_UNKNOWN,
  MPU_FAULT_NULL_POINTER,
  MPU_FAULT_STACK_OVERFLOW,
  MPU_FAULT_NOT_MAPPED,
  MPU_FAULT_ACCESS_BLOCKED,
  MPU_FAULT_WRITE_TO_READONLY,
  MPU_FAULT_NOT_EXECUTABLE,
  MPU_FAULT_UNPRIV_ACCESS_BLOCKED,
  MPU_FAULT_UNPRIV_WRITE_TO_READONLY,
  MPU_FAULT_UNPRIV_NOT_EXECUTABLE,
};

enum MpuFaultDiagnosis
mpu_diagnose_memfault(usize address, bool instruction_access, bool privileged_access);

#ifdef __cplusplus
}
#endif
