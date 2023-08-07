// NOTE: The MPU is used ONLY for catching trivial bugs such as dereferencing a
// NULL pointer or writing to constant data, and not for stuff like protecting
// the address spaces of different processes/threads, so, despite the fact that
// formally we *are* using a memory protection scheme, the original TempleOS
// spirit of everything having access to everything else is preserved.
//
// NOTE: The MPU settings are ignored for vector table accesses and by the DMA
// controller. That is of no significance to us though, since the vector table
// is the business of the hardware, and because we don't bother implementing
// *proper* memory protection (as stated above), ensuring that DMA operations
// are safe is the job of the programmer (if we were to we'd have to lock the
// DMA registers for unprivileged mode or something and provide special kernel
// functions for starting the DMA streams which would reject illegal accesses).
//
// Useful resoucres:
// <https://interrupt.memfault.com/blog/fix-bugs-and-secure-firmware-with-the-mpu>
// <https://github.com/zephyrproject-rtos/zephyr/blob/10ef3b46d870f9672bb6407d78bf94a6e3fb9675/include/zephyr/arch/arm/aarch32/mpu/arm_mpu_v7m.h>
// <https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1159114/faq-tms570lc4357-what-are-the-differences-among-the-memory-attributes-memory-types-and-cache-policy-in-mpu-settings>
// <https://forum.huawei.com/enterprise/en/differences-between-disk-cache-write-through-and-write-back/thread/667215004455288832-667213859733254144>
// <https://www.youtube.com/watch?v=6IUfxSAFhlw&list=PLnMKNibPkDnEQXu4S6QUUHuSKj81MeqCz&index=1>
// <https://av.tib.eu/media/50199>
// DDI0403E:
//  B3.1 "The system address map"
//  Table B3-1 "ARMv7-M address map"
//  B3.5 "Protected Memory System Architecture, PMSAv7"
//  A3.5 "Memory types and attributes and the memory order model"
// PM0214:
//  2.2 "Memory model"
//  2.2.3 "Behavior of memory accesses"
//  4.2 "Memory protection unit (MPU)"
//  4.2.1 "MPU access permission attributes"
//  4.2.4 "MPU design hints and tips"
// RM0383:
//  2.4 table 3
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Address-Map/Protected-Memory-System-Architecture--PMSAv7/PMSAv7-compliant-MPU-operation>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Memory-Model/Pseudocode-details-of-general-memory-system-operations/Access-permission-checking>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Memory-Model/Pseudocode-details-of-general-memory-system-operations/Default-memory-access-decode>
// <https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Memory-Model/Pseudocode-details-of-general-memory-system-operations/MPU-access-control-decode>

#include "stmes/kernel/mpu.h"
#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

struct MpuRegionConfig {
  usize base_addr;
  usize end_addr;
  u8 number;
  u8 disabled_subregions;
  bool executable : 1;
  u8 permissions : 3;
  u8 type_ext : 3;
  bool shareable : 1;
  bool cacheable : 1;
  bool bufferable : 1;
};

// A helper for setting up the MPU registers.
__STATIC_FORCEINLINE void mpu_setup_region(const struct MpuRegionConfig* cfg) {
  // All assertions here are essentially eliminated after inlining.
  ASSERT(cfg->number < MPU_MAX_REGIONS_NUMBER);
  usize size = cfg->end_addr - cfg->base_addr + 1;
  ASSERT(size >= MPU_MIN_REGION_SIZE);
  ASSERT((size & (size - 1)) == 0);           // Check that the size is a power of 2
  ASSERT((cfg->base_addr & (size - 1)) == 0); // Check the alignment of the base address
  u32 size_log2 = 31 - __CLZ(size);
  u32 attrs = MPU_RASR_ENABLE_Msk;
  attrs |= (size_log2 - 1) << MPU_RASR_SIZE_Pos;
  attrs |= cfg->disabled_subregions << MPU_RASR_SRD_Pos;
  attrs |= !cfg->executable << MPU_RASR_XN_Pos;
  attrs |= cfg->permissions << MPU_RASR_AP_Pos;
  attrs |= cfg->type_ext << MPU_RASR_TEX_Pos;
  attrs |= cfg->shareable << MPU_RASR_S_Pos;
  attrs |= cfg->cacheable << MPU_RASR_C_Pos;
  attrs |= cfg->bufferable << MPU_RASR_B_Pos;
  // Region Number Register
  WRITE_REG(MPU->RNR, cfg->number);
  // Region Base Address Register
  WRITE_REG(MPU->RBAR, cfg->base_addr & MPU_RBAR_ADDR_Msk);
  // Region Attribute and Size Register
  WRITE_REG(MPU->RASR, attrs);
}

// Returns the lowest address the stack is allowed to descend to.
__STATIC_FORCEINLINE usize mpu_get_stack_limit(void) {
  extern u32 _estack, _Min_Stack_Size;
  return (usize)&_estack - (usize)&_Min_Stack_Size;
}

void mpu_init(void) {
  // PM0214 section 4.2.4 recommends disabling the interrupts while programming the MPU.
  u32 primask = __get_PRIMASK();
  __disable_irq();
  // Drain any leftover data transactions before entering the setup.
  __DSB();

  // The MPU region configuration is mostly based on the recommendations from
  // PM0214 section 4.2.4 "MPU design hints and tips". As stated there, the
  // attributes "shareable" and "cacheable" don't affect the behavior of STM32
  // implementations, but the suggested values for those were nonetheless copied.

  // FLASH: Normal memory, non-shareable, write-through cache
  mpu_setup_region(&(struct MpuRegionConfig){
    .number = MPU_REGION_FLASH_NUM,
    .base_addr = MPU_REGION_FLASH_BASE,
    .end_addr = MPU_REGION_FLASH_END,
    .executable = 1,
    .permissions = MPU_REGION_PRIV_RO_URO,
    .type_ext = 0,
    .cacheable = 1,
    .bufferable = 0,
    .shareable = 0,
  });

  // SRAM1: Normal memory, shareable, write-through cache
  mpu_setup_region(&(struct MpuRegionConfig){
    .number = MPU_REGION_SRAM1_NUM,
    .base_addr = MPU_REGION_SRAM1_BASE,
    .end_addr = MPU_REGION_SRAM1_END,
    .executable = 0,
    .permissions = MPU_REGION_FULL_ACCESS,
    .type_ext = 0,
    .cacheable = 1,
    .bufferable = 0,
    .shareable = 1,
  });

  // SRAM1_BB: Normal memory, shareable, write-through cache
  mpu_setup_region(&(struct MpuRegionConfig){
    .number = MPU_REGION_SRAM1_BB_NUM,
    .base_addr = MPU_REGION_SRAM1_BB_BASE,
    .end_addr = MPU_REGION_SRAM1_BB_END,
    .executable = 0,
    .permissions = MPU_REGION_FULL_ACCESS,
    .type_ext = 0,
    .cacheable = 1,
    .bufferable = 0,
    .shareable = 1,
  });

  // PERIPH: Device memory, shareable
  mpu_setup_region(&(struct MpuRegionConfig){
    .number = MPU_REGION_PERIPH_NUM,
    .base_addr = MPU_REGION_PERIPH_BASE,
    .end_addr = MPU_REGION_PERIPH_END,
    .executable = 0,
    .permissions = MPU_REGION_FULL_ACCESS,
    .type_ext = 0,
    .cacheable = 0,
    .bufferable = 1,
    .shareable = 1,
  });

  usize stack_limit = mpu_get_stack_limit();
  // Stack barrier: Strongly-ordered?
  mpu_setup_region(&(struct MpuRegionConfig){
    .number = MPU_REGION_STACK_BARRIER,
    .base_addr = stack_limit,
    .end_addr = stack_limit + MPU_REGION_STACK_BARRIER_SIZE - 1,
    .executable = 0,
    .permissions = MPU_REGION_NO_ACCESS,
    // Unsure about these attributes. Well, this region definitely has to be
    // non-bufferable, so that accesses trip the alarm immediately.
    .type_ext = 0,
    .cacheable = 0,
    .bufferable = 0,
    .shareable = 0,
  });

  // The MPU is enabled with the following options:
  // 1. Our memory map fully replaces the default one instead of being overlaid
  //    on top of it.
  // 2. The MPU is temporarily disabled when running NMI and HardFault handlers
  //    (this is necessary to allow recovery from memfaults).
  WRITE_REG(MPU->CTRL, MPU_CTRL_ENABLE_Msk & ~MPU_CTRL_HFNMIENA_Msk & ~MPU_CTRL_PRIVDEFENA_Msk);
  // Flush the queued memory transactions to ensure that our MPU settings take
  // effect immediately.
  __DSB();
  __ISB();
  // Restore the interrupts.
  __set_PRIMASK(primask);
}

void mpu_enable_region(u8 region_nr) {
  WRITE_REG(MPU->RNR, region_nr);
  SET_BIT(MPU->RASR, MPU_RASR_ENABLE_Msk);
}

void mpu_disable_region(u8 region_nr) {
  WRITE_REG(MPU->RNR, region_nr);
  CLEAR_BIT(MPU->RASR, MPU_RASR_ENABLE_Msk);
}

// Figures out which the region contains the given address (falling back to the
// default map if necessary) and returns its attributes, returns false if no
// regions are mapped at that location.
bool mpu_match_region(usize address, u32* out_attrs) {
  // This function essentially replicates the operation of the hardware MPU as
  // described in the form of pseudocode functions `ValidateAddress` and
  // `CheckPermission` in the sections B3.5.3 "PMSAv7-compliant MPU operation"
  // and B2.3.8 "Access permission checking" of the ARMv7m spec (DDI0403E).
  // Additionally, in the interest of saving RAM, it actually downloads the
  // current configuration from the MPU registers.
  u32 mpu_ctrl = READ_REG(MPU->CTRL);
  // NOTE: Accesses into the PPB (Private Peripheral Bus) space in the range
  // 0xE0000000-0xE00FFFFF always use the default map.
  if ((mpu_ctrl & MPU_CTRL_ENABLE_Msk) && (address >> 20) != 0xE00) {
    u8 regions_count = (READ_REG(MPU->TYPE) & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;

    // The region with a higher number takes priority.
    for (u8 region_nr = regions_count; region_nr > 0; region_nr--) {
      WRITE_REG(MPU->RNR, region_nr - 1); // Select the region
      u32 attrs = READ_REG(MPU->RASR);
      if (!(attrs & MPU_RASR_ENABLE_Msk)) {
        continue;
      }

      usize base_addr = READ_REG(MPU->RBAR) & MPU_RBAR_ADDR_Msk;
      u32 size_log2 = ((attrs & MPU_RASR_SIZE_Msk) >> MPU_RASR_SIZE_Pos) + 1;
      // This value masks out the bits of an offset inside the region.
      u32 addr_mask = size_log2 < 32 ? ~MASK(size_log2) : 0;
      if ((address & addr_mask) != (base_addr & addr_mask)) {
        continue;
      }

      u8 disabled_subregions = (attrs & MPU_RASR_SRD_Msk) >> MPU_RASR_SRD_Pos;
      // Subregions are supported only for regions of 256 bytes or larger.
      if (size_log2 >= 8) {
        // This expression selects the 3 most significant bits of the offset
        // within the region, which give exactly the subregion index.
        u8 subregion_idx = (address >> (size_log2 - 3)) & MASK(3);
        if ((disabled_subregions & BIT(subregion_idx)) != 0) {
          continue;
        }
      }

      // Enforce the execute-never attribute for addresses in the System space
      // (0xE0000000-0xFFFFFFFF).
      if ((address >> 29) == 7) {
        attrs |= MPU_RASR_XN_Msk;
      }
      *out_attrs = attrs;
      return true;
    }

    if (!(mpu_ctrl & MPU_CTRL_PRIVDEFENA_Msk)) {
      return false; // No mapped region found.
    }
    // Fall back to the default map (if configured to do so).
  }

  // The default memory map is partitioned into eight half-GB regions. Their
  // attributes are defined in tables B3-1 and B3-2 in the section B3.1 "The
  // system address map" and pseudocode functions `DefaultMemoryAttributes` in
  // the section B2.3.10 "Default memory access decode" and `DefaultPermissions`
  // in the section B3.5.3 "PMSAv7-compliant MPU operation" of DDI0403E.
  u8 region_nr = address >> 29;
  // I wish C had a language construct for bitsets.
  const u8 EXECUTE_NEVER_REGIONS = BITS8(1, 1, 1, 0, 0, 1, 0, 0);
  bool not_executable = (EXECUTE_NEVER_REGIONS >> region_nr) & 1;

  // Not every attribute is returned for the default map regions because we
  // don't require them all.
  u32 attrs = 0;
  attrs |= MPU_RASR_ENABLE_Msk;
  attrs |= MPU_REGION_SIZE_512MB << MPU_RASR_SIZE_Pos;
  attrs |= MPU_REGION_FULL_ACCESS << MPU_RASR_AP_Pos;
  attrs |= not_executable << MPU_RASR_XN_Pos;
  *out_attrs = attrs;
  return true;
}

// Attempts to establish the cause of a MemFault given the offending address
// and known access circumstances.
enum MpuFaultDiagnosis
mpu_diagnose_memfault(usize address, bool instruction_access, bool privileged_access) {
  if (address <= 1024 * 1024) {
    return MPU_FAULT_NULL_POINTER;
  }

  usize stack_limit = mpu_get_stack_limit();
  if (stack_limit <= address && address < stack_limit + MPU_REGION_STACK_BARRIER_SIZE) {
    return MPU_FAULT_STACK_OVERFLOW;
  }

  u32 region_attrs = 0;
  if (!mpu_match_region(address, &region_attrs)) {
    return MPU_FAULT_NOT_MAPPED;
  }

  bool access_blocked = false, write_blocked = false;
  bool unpriv_access_blocked = false, unpriv_write_blocked = false;
  bool execute_never = (region_attrs & MPU_RASR_XN_Msk) != 0;
  // Decode the access permission bits.
  switch ((region_attrs & MPU_RASR_AP_Msk) >> MPU_RASR_AP_Pos) {
    case MPU_REGION_NO_ACCESS: access_blocked = unpriv_access_blocked = true; break;
    case MPU_REGION_PRIV_RW: unpriv_access_blocked = true; break;
    case MPU_REGION_PRIV_RW_URO: unpriv_write_blocked = true; break;
    case MPU_REGION_PRIV_RO: write_blocked = unpriv_access_blocked = true; break;
    case MPU_REGION_PRIV_RO_URO | 1: // This access mode has two encodings: 0b110 and 0b111.
    case MPU_REGION_PRIV_RO_URO & ~1: write_blocked = unpriv_write_blocked = true; break;
  }

  if (instruction_access && execute_never) {
    return MPU_FAULT_NOT_EXECUTABLE;
  } else if (instruction_access && access_blocked) {
    return MPU_FAULT_NOT_EXECUTABLE;
  } else if (instruction_access && unpriv_access_blocked && !privileged_access) {
    return MPU_FAULT_UNPRIV_NOT_EXECUTABLE;
  } else if (access_blocked) {
    return MPU_FAULT_ACCESS_BLOCKED;
  } else if (!privileged_access && unpriv_access_blocked) {
    return MPU_FAULT_UNPRIV_ACCESS_BLOCKED;
  } else if (write_blocked) {
    return MPU_FAULT_WRITE_TO_READONLY;
  } else if (!privileged_access && unpriv_write_blocked) {
    return MPU_FAULT_UNPRIV_WRITE_TO_READONLY;
  }
  return MPU_FAULT_UNKNOWN;
}
