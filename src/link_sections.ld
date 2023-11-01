/*
I won't bother explaining the ins and outs of linker scripts here, that is
explained very well by the following articles:

<https://mcyoung.xyz/2021/06/01/linker-script/>
<https://interrupt.memfault.com/blog/how-to-write-linker-scripts-for-firmware>
<https://blog.thea.codes/the-most-thoroughly-commented-linker-script/>

Plus, by the ld linker documentation:

<https://sourceware.org/binutils/docs/ld/>
<https://www.acrc.bris.ac.uk/acrc/RedHat/rhel-ld-en-4/sections.html>
<https://ftp.gnu.org/old-gnu/Manuals/ld-2.9.1/html_chapter/ld_3.html>
<https://lld.llvm.org/ELF/linker_script.html>

This script largely mixes ideas found in the following linker scripts:

<https://github.com/stm32duino/Arduino_Core_STM32/blob/2.6.0/variants/STM32F4xx/F411C%28C-E%29%28U-Y%29/ldscript.ld>
<https://github.com/RIOT-OS/RIOT/blob/2023.07/cpu/cortexm_common/ldscripts/cortexm_base.ld>
<https://github.com/RT-Thread/rt-thread/blob/v5.0.2/bsp/stm32/stm32f411-weact-blackpill/board/linker_scripts/link.lds>
<https://github.com/tinygo-org/tinygo/blob/v0.30.0/targets/arm.ld>
<https://github.com/ARM-software/CMSIS_5/blob/5.9.0/Device/ARM/ARMCM4/Source/GCC/gcc_arm.ld>
<https://github.com/raspberrypi/pico-sdk/blob/1.5.1/src/rp2_common/pico_standard_link/memmap_default.ld>
<https://github.com/wntrblm/Castor_and_Pollux/blob/2023.9.11/firmware/scripts/samd21g18a.ld>
<https://github.com/tock/tock/blob/25ce626ec170b42f908aa33fcdc5e7f4cfcbff3c/boards/kernel_layout.ld>

TODO: REGION_ALIAS can be used to allow customization of the memory sections
<https://sourceware.org/binutils/docs/ld/REGION_005fALIAS.html>

*/

SECTIONS {

  /* The vector table is pinned to the start of ROM */
  .vector_table ORIGIN(flash) : ALIGN(4) {
    LONG(__stack_end)
    KEEP(*(.vector_table))
  } > flash

  /* Executable machine code */
  .text : {
    PROVIDE(__text_start = .);
    /* AFAIK, .gnu.linkonce are sections for so-called "vague linking" */
    *(.text .text.* .gnu.linkonce.t.*)
    KEEP(*(.init)) KEEP(*(.fini)) /* these two must be placed in separate pattern expressions */
    *(.glue_7) /* glue ARM to Thumb code */
    *(.glue_7t) /* glue Thumb to ARM code */
    PROVIDE(__text_end = .);
  } > flash

  /* Read-only global variables, constants, strings */
  .rodata : {
    *(.rodata .rodata.* .gnu.linkonce.r.*)
  } > flash

  /* Initialized global variables, will be relocated from ROM to RAM. */
  /* This segment is 4-byte aligned so that the copying can be done 4 bytes at a time. */
  .data : ALIGN(4) {
    *(.RamFunc .RamFunc.*)
    *(.data .data.* .gnu.linkonce.d.*)
    . = ALIGN(4); /* pad the segment at the end as well */
  } > ram AT> flash

  PROVIDE(__data_source = LOADADDR(.data));
  PROVIDE(__data_start = ADDR(.data));
  PROVIDE(__data_end = ADDR(.data) + SIZEOF(.data));

  /* Uninitialized global variables, will be placed in RAM and zeroed out. */
  /* This segment must also be 4-byte aligned to use 4-byte-at-a-time writes. */
  /* The NOLOAD attribute ensures that this segment doesn't contribute to binary size and ROM usage. */
  .bss (NOLOAD) : ALIGN(4) {
    *(.bss .bss.* .gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4); /* segment padding at the end */
  } > ram

  PROVIDE(__bss_start = ADDR(.bss));
  PROVIDE(__bss_end = ADDR(.bss) + SIZEOF(.bss));

  /* The stack is reserved using the linker to ensure that nothing overalps it. */
  /* As for Why this segment is placed in its own memory region: */
  /* <https://stackoverflow.com/a/75917320> */
  .stack (NOLOAD) : ALIGN(8) {
    PROVIDE(__stack_start = .);
    . += DEFINED(__stack_size) ? __stack_size : 0;
    . = ALIGN(8);
    PROVIDE(__stack_end = .);
  } > stack

  /* This segment exists purely to ensure space availability for heap. */
  .heap (NOLOAD) : {
    PROVIDE(__heap_start = .);
    . += DEFINED(__heap_size_min) ? __heap_size_min : 0;
    PROVIDE(__heap_end = ORIGIN(ram) + LENGTH(ram));
  } > ram

  /* Functions to be run before static variable constructors. */
  .preinit_array : {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > flash

  /* Static variable constructors and functions marked with __attribute__((constructor)). */
  .init_array : {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
    KEEP(*(.init_array))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > flash

  /* Static variable destructors and functions marked with __attribute__((destructor)). */
  .fini_array : {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT_BY_INIT_PRIORITY(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > flash

  /* Segments with metadata for C++ exception support */

  .eh_frame : {
    PROVIDE_HIDDEN(__eh_frame_start = .);
    KEEP(*(.eh_frame .eh_frame.*))
  } > flash

  .eh_frame_hdr : {
    *(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*)
  } > flash

  .gcc_except_table : {
    *(.gcc_except_table .gcc_except_table.*)
  } > flash

  /* ARM-specific stack unwind tables */

  .ARM.extab : {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > flash

  .ARM.exidx : {
    PROVIDE_HIDDEN(__exidx_start = .);
    *(.ARM.exidx*)
    PROVIDE_HIDDEN(__exidx_end = .);
  } > flash

}
