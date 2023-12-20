cmake_minimum_required(VERSION 3.17)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm_gcc_toolchain.cmake")
endif()

if(NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

project(stmes LANGUAGES C)

if(POLICY CMP0135)
  # Affects the behavior of timestamps on files downloaded by FetchContent and
  # ExternalProject: <https://cmake.org/cmake/help/latest/policy/CMP0135.html>
  cmake_policy(SET CMP0135 NEW)
endif()

# These must be set before including FetchContent.
option(FETCHCONTENT_QUIET "" OFF)  # Output download progress and other logs
option(FETCHCONTENT_UPDATES_DISCONNECTED "" ON)  # Don't check for updates every time

include(FetchContent)
include(ExternalProject)

set(DEPENDENCIES_DIR "" CACHE FILEPATH "The directory in which the dependencies will be downloaded and unpacked")
if(DEPENDENCIES_DIR STREQUAL "")
  message(FATAL_ERROR "DEPENDENCIES_DIR must be specified")
endif()

# Technically, all STM32Cube and HAL libraries are distributed in a single
# package which can be downloaded from this repo:
# <https://github.com/STMicroelectronics/STM32CubeF4>, but it contains a lot of
# redundant stuff (like, the example projects directory is about 500 megs), so
# to save disk space and network bandwidth only the necessary packages are
# downloaded.

# CMSIS - a common (lowest level) interface for programming microcontrollers
# based on ARM Cortex-M processors and working with their built-in peripherals.
FetchContent_Declare(cmsis_core_cm4
  URL "https://github.com/STMicroelectronics/cmsis_core/archive/refs/tags/v5.4.0_cm4.tar.gz"
  URL_HASH SHA256=f711074a546bce04426c35e681446d69bc177435cd8f2f1395a52db64f52d100
  DOWNLOAD_DIR "${DEPENDENCIES_DIR}"
  SOURCE_DIR "${DEPENDENCIES_DIR}/cmsis_core_cm4"
  DOWNLOAD_NAME "cmsis_core_cm4_v5.4.0_cm4.tar.gz"
  # These directories are pretty large and we won't be needing them anyway.
  PATCH_COMMAND "${CMAKE_COMMAND}" -E rm -rf Lib docs DSP NN
)

# CMSIS device package for STM32F4xx microcontrollers - includes definitions
# for the peripherals available on the F4 series and system startup files.
FetchContent_Declare(cmsis_device_f4
  URL "https://github.com/STMicroelectronics/cmsis_device_f4/archive/refs/tags/v2.6.8.tar.gz"
  URL_HASH SHA256=6390baf3ea44aff09d0327a3c112c6ca44418806bfdfe1c5c2803941c391fdce
  DOWNLOAD_DIR "${DEPENDENCIES_DIR}"
  SOURCE_DIR "${DEPENDENCIES_DIR}/cmsis_device_f4"
  DOWNLOAD_NAME "cmsis_device_f4_v2.6.8.tar.gz"
)

# Portable HAL (Hardware Abstraction Library) for STM32F4xx microcontrollers.
FetchContent_Declare(stm32f4xx_hal_driver
  URL "https://github.com/STMicroelectronics/stm32f4xx_hal_driver/archive/refs/tags/v1.8.1.tar.gz"
  URL_HASH SHA256=d3ffce7d0ecef8a02f893b3341336f54dad0f7d7047328c353f3dcaa1db36312
  DOWNLOAD_DIR "${DEPENDENCIES_DIR}"
  SOURCE_DIR "${DEPENDENCIES_DIR}/stm32f4xx_hal_driver"
  DOWNLOAD_NAME "stm32f4xx_hal_driver_v1.8.1.tar.gz"
)

# USB host middleware for STM32 microcontrollers based around the STM32 HAL.
FetchContent_Declare(stm32_mw_usb_host
  URL "https://github.com/STMicroelectronics/stm32_mw_usb_host/archive/refs/tags/v3.3.4.tar.gz"
  URL_HASH SHA256=454d3e27c0245c4545a400b7d583c964a00bdca35d9a9f60e3190e05524b0c5e
  DOWNLOAD_DIR "${DEPENDENCIES_DIR}"
  SOURCE_DIR "${DEPENDENCIES_DIR}/stm32_mw_usb_host"
  DOWNLOAD_NAME "stm32_mw_usb_host_v3.3.4.tar.gz"
)

# TODO comments
ExternalProject_Add(tools
  PREFIX "${CMAKE_BINARY_DIR}/_deps/tools-prefix"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/tools"
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools"
  DOWNLOAD_COMMAND ""
  INSTALL_COMMAND ""  # An empty string suppresses this step since at least CMake 3.10
  BUILD_ALWAYS YES    # TODO
)

get_property(project_languages GLOBAL PROPERTY ENABLED_LANGUAGES)
foreach(lang IN LISTS project_languages)
  foreach(config IN ITEMS "DEBUG" "MINSIZEREL" "RELEASE" "RELWITHDEBINFO")
    # Clear out the flags variables for all languages for all configs, we'll be
    # using generator expressions for language- and config-dependent options.
    set(CMAKE_${lang}_FLAGS_${config} "")
  endforeach()
endforeach()

# Architecture-specific flags need to be passed to both the compiler and
# linker, so that they both agree on the ABI.
set(arch_flags
  # CPU configuration
  -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16
  # Generate FPU instructions
  -mfloat-abi=hard
  # ARMv7-M processors support unaligned reads/writes by breaking them up into
  # multiple (non-atomic!) memory bus transactions, but can also be configured
  # to trap on such accesses to detect bugs. Either this flag or
  # -mno-unaligned-access needs to be passed because different compilers make
  # different default choices about generating instructions which perform
  # unaligned loads/stores (GCC does, Clang doesn't).
  -munaligned-access
  # TODO: Try out -mslow-flash-data
)
add_compile_options(${arch_flags})
add_link_options(${arch_flags})

# Clang doesn't come with a standard library for every target out there, so we
# piggyback off the ARM GCC toolchain and its libraries.
# <https://interrupt.memfault.com/blog/arm-cortexm-with-llvm-clang>
# <https://github.com/vpetrigo/arm-cmake-toolchains/blob/b0479f74099e933d8bedb303cb7a4863e5cd9909/clang_utils.cmake>
# <https://github.com/vpetrigo/arm-cmake-toolchains/blob/b0479f74099e933d8bedb303cb7a4863e5cd9909/clang-arm-gcc-toolchain.cmake>
if(CMAKE_C_COMPILER_ID STREQUAL "Clang" AND DEFINED TOOLCHAIN_TARGET)
  find_program(TOOLCHAIN_GCC "${TOOLCHAIN_TARGET}-gcc")
  if(NOT TOOLCHAIN_GCC)
    message(FATAL_ERROR "${TOOLCHAIN_TARGET}-gcc could not be found")
  endif()

  # NOTE: The library paths depend on the exact target architecture flags.
  function(toolchain_gcc_query_path out_var print_flag)
    if(NOT DEFINED "${out_var}" OR NOT "${${out_var}_ARCH_FLAGS}" STREQUAL "${arch_flags}")
      execute_process(
        COMMAND "${TOOLCHAIN_GCC}" ${arch_flags} ${print_flag}
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error_output
        RESULT_VARIABLE exit_code
      )
      if(NOT exit_code EQUAL 0)
        message(FATAL_ERROR "${error_output}")
      endif()
      file(TO_CMAKE_PATH "${output}" path)
      set("${out_var}" "${path}" CACHE INTERNAL "" FORCE)
      set("${out_var}_ARCH_FLAGS" "${arch_flags}" CACHE INTERNAL "" FORCE)
      message(STATUS "${out_var}: ${path}")
    endif()
  endfunction()

  toolchain_gcc_query_path(TOOLCHAIN_GCC_SYSROOT -print-sysroot)
  toolchain_gcc_query_path(TOOLCHAIN_GCC_MULTI_DIR -print-multi-directory)
  toolchain_gcc_query_path(TOOLCHAIN_GCC_LIBGCC_FILE -print-libgcc-file-name)

  # Strictly speaking, you are allowed to set the sysroot only in the toolchain
  # script, but eh, whatever.
  set(CMAKE_SYSROOT "${TOOLCHAIN_GCC_SYSROOT}")
  get_filename_component(TOOLCHAIN_GCC_LIBGCC_DIR "${TOOLCHAIN_GCC_LIBGCC_FILE}" DIRECTORY)
  link_directories(
    "${TOOLCHAIN_GCC_LIBGCC_DIR}" "${TOOLCHAIN_GCC_SYSROOT}/lib/${TOOLCHAIN_GCC_MULTI_DIR}"
  )
  add_link_options(-rtlib=libgcc)  # Use libgcc instead of Clang's compiler-rt
endif()

# Configure the compiler to emit a separate section for every function or
# global variable in the compiled object file. This will enable the linker to
# construct a dependency graph of symbols which are actually used by the
# resulting binary, starting at the entry point, and throw out the bulk of
# unused library stuff, at the cost of maybe increasing the size of individual
# functions (because each one now gotta have its own constant pool).
add_compile_options(-ffunction-sections -fdata-sections)
# Request the linker to perform garbage collection of unused sections, which
# 1-to-1 correspond to symbols thanks to the flags above.
add_link_options(LINKER:--gc-sections)

# LTO (link-time optimization) doesn't seem to help right now (making the
# executable larger, probably due to aggressive inlining).
#include(CheckIPOSupported)
#check_ipo_supported(RESULT CMAKE_INTERPROCEDURAL_OPTIMIZATION)

add_compile_options(
  $<$<CONFIG:Debug>:-Og>
  $<$<CONFIG:MinSizeRel>:-Os>
  $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O2>
  # Enable level 3 debugging information for GDB, which among other things
  # includes macros. This, however, incrases the object file sizes by 50 times.
  # Oh no. By default only level 2 is enabled.
  $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:-ggdb3>
  # Getting rid of the frame pointer gives the compiler one more
  # general-purpose register to work with and makes the function
  # prologues/epilogues much shorter. The only downside is that a more
  # complicated stack unwinding strategy must be employed, but we exploit the
  # exception handling metadata for that. For some reason this option is not
  # enabled by default on Clang.
  -fomit-frame-pointer
)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  # GCC optimization options list:
  # <https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html>
  # <https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.1.0/gcc/opts.cc#L566>
  add_compile_options(
    # Throw in some flags that are not enabled by -Og
    $<$<CONFIG:Debug>:-fif-conversion>  # Convert ifs into branchless equivalents
    $<$<CONFIG:Debug>:-fif-conversion2>  # Convert ifs into instructions with condition flags
    $<$<CONFIG:Debug>:-fmove-loop-invariants>  # Moves and reuses common loads out of if branches and loops
    $<$<CONFIG:Debug>:-ftree-switch-conversion>  # Converts simple switch statements into lookup tables
  )
endif()

add_compile_definitions(
  $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
  $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:CFI_DIRECTIVES>
)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS YES)  # GNU extensions must be requested, at least for inline assembly
set(CMAKE_C_STANDARD_REQUIRED YES)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS YES)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>  # Don't generate C++ runtime type information
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>  # Don't generate extra code for exception handling
)

# However, generate the metadata for stack unwinding for both C++ and C.
add_compile_options(-funwind-tables)
add_compile_definitions(ARM_UNWIND_DIRECTIVES)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  # <https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html#index-mpoke-function-name>
  add_compile_options(-mpoke-function-name)
  add_compile_definitions(ARM_POKE_FUNCTION_NAME)
endif()

# Substitute file paths with a shorter string in expansions of the `__FILE__`
# preprocessor variable (most notably used by the assertion macros), which has
# non-negligible impact on the executable size.
add_compile_options(
  -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=
  -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/src/=
  -fmacro-prefix-map=${DEPENDENCIES_DIR}/stm32f4xx_hal_driver/Src/=
)

# We are now done with project-wide compilation flags! External dependencies,
# libraries and other subprojects may be included starting from here, so that
# the global options and flags are propagated to the whole dependency tree.
FetchContent_MakeAvailable(
  cmsis_core_cm4
  cmsis_device_f4
  stm32f4xx_hal_driver
  stm32_mw_usb_host
)

add_subdirectory(lib)

add_executable(firmware)
# This is the correct way of setting the file extension for a binary:
set_target_properties(firmware PROPERTIES PREFIX "" SUFFIX ".elf")

# <https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html>
# TODO: Enable more warnings:
# <https://github.com/embeddedartistry/meson-buildsystem/blob/d4de8c83f97cb6389d9895548b555220f9c56e53/compiler/meson.build#L8-L49>
# <https://github.com/embeddedartistry/meson-buildsystem/blob/d4de8c83f97cb6389d9895548b555220f9c56e53/compiler/cpp/meson.build#L11-L21>
# <https://github.com/embeddedartistry/cmake-buildsystem/blob/f442d5ddbdd87f7590cb06cdbf6c319fa119578e/compiler/DefaultCompilerSettings.cmake#L9-L69>
target_compile_options(firmware PRIVATE
  -Wall -Wextra -Wpedantic -pedantic-errors
  -Werror=return-type
  -Wdouble-promotion
  -Wmissing-declarations
  -Werror=float-conversion
  $<$<COMPILE_LANGUAGE:C>:-Werror=strict-prototypes>
)

target_link_libraries(firmware PUBLIC
  stm32f4xx_hal stm32_usb_host FatFs printf m
)

# NOTE: On disabling the standard library. GCC by default also links programs
# with a bunch of additional support objects, startup files and standard
# libraries. There exist several flags for disabling those to varying degrees:
# 1. -nostdlib - disables everything, only the libraries and objects specified
#    on the command line are linked.
# 2. -nostartfiles - disables the inclusion of CRT (C runtime) objects. Their
#    purpose is described here: <https://gcc.gnu.org/onlinedocs/gccint/Initialization.html>,
#    they are compiled from this file on most targets: <https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.1.0/libgcc/crtstuff.c>.
# 3. -nodefaultlibs - disables standard and system support libraries. Notably,
#    this option doesn't exclude the startup files (CRT objects).
# 4. -nolibc - disables just the libc, all other internal libraries such as
#    libm, libstdc++, libgcc are linked. libgcc in particular provides
#    polyfills for emulating floating point or 64-bit integer arithmetic, see
#    <https://gcc.gnu.org/onlinedocs/gccint/Libgcc.html>.
#
# What options are actually passed to the linker can be checked with the `-v`
# flag to GCC, to debug the library resolution process pass `-Wl,--verbose`.
# See also <https://gcc.gnu.org/onlinedocs/gcc/Link-Options.html>.
#target_link_options(firmware PRIVATE -nostdlib)
#target_link_libraries(firmware PRIVATE $<$<LINK_LANGUAGE:CXX>:stdc++> m gcc c nosys)

foreach(linker_script IN ITEMS stm32f411ceux.ld link_sections.ld)
  set(linker_script "${CMAKE_CURRENT_SOURCE_DIR}/src/${linker_script}")
  target_link_options(firmware PRIVATE "-T${linker_script}")
  set_property(TARGET firmware APPEND PROPERTY LINK_DEPENDS "${linker_script}")
endforeach()

set(linker_map_file $<TARGET_FILE:firmware>.map)
target_link_options(firmware PRIVATE LINKER:--cref,-Map=${linker_map_file})
set_property(TARGET firmware APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${linker_map_file}")

add_dependencies(firmware tools)
set(linker_plugin_file "${CMAKE_CURRENT_BINARY_DIR}/tools/libtest_linker_plugin.so")
target_link_options(firmware PRIVATE LINKER:-plugin,${linker_plugin_file})
set_property(TARGET firmware APPEND PROPERTY LINK_DEPENDS "${linker_plugin_file}")

include(CheckCCompilerFlag)
# A replacement for the CheckLinkerFlag module, added in CMake v3.18, which
# only needs CMake v3.14.
function(check_c_linker_flag flag result_var)
  # Even though I am setting a global variable here, CMake will take care of
  # restoring its value after leaving the scope of this function.
  set(CMAKE_REQUIRED_LINK_OPTIONS "${flag}")
  # (The first argument is the compiler flag being tested - none in our case.)
  check_c_compiler_flag("" "${result_var}")
endfunction()

# <https://github.com/raspberrypi/pico-sdk/issues/1029>
# <https://www.redhat.com/en/blog/linkers-warnings-about-executable-stacks-and-segments>
# Added in recent versions of GNU ld, needs a feature check.
# TODO: Can we get rid of this warning?
check_c_linker_flag(LINKER:--no-warn-rwx-segments HAVE_LINK_FLAG_NO_WARN_RWX_SEGMENTS)
if(HAVE_LINK_FLAG_NO_WARN_RWX_SEGMENTS)
  target_link_options(firmware PRIVATE LINKER:--no-warn-rwx-segments)
endif()

# GNU ld can print a pretty helpful memory usage summary intended precisely for
# embedded projects, but this flag is not implemented in LLVM lld, so it also
# needs a check.
check_c_linker_flag(LINKER:--print-memory-usage HAVE_LINK_FLAG_PRINT_MEMORY_USAGE)
if(HAVE_LINK_FLAG_PRINT_MEMORY_USAGE)
  # TODO: Our own utility for displaying memory usage summary
  # <https://github.com/platformio/platformio-core/blob/v6.1.11/platformio/builder/tools/pioupload.py>
  target_link_options(firmware PRIVATE LINKER:--print-memory-usage)
else()
  # Use a fallback for linkers that don't support it:
  set(CMAKE_SIZE "size")
  if(DEFINED TOOLCHAIN_TARGET)
    set(CMAKE_SIZE "${TOOLCHAIN_TARGET}-size")
  endif()
  find_program(CMAKE_SIZE "${CMAKE_SIZE}")
  add_custom_command(
    TARGET firmware POST_BUILD
    COMMAND "${CMAKE_SIZE}" $<TARGET_FILE:firmware>
    VERBATIM
  )
endif()

# Not all upload tools support flashing an ELF file directly, construct a raw
# binary file for that.
add_custom_command(
  TARGET firmware POST_BUILD
  COMMAND "${CMAKE_OBJCOPY}" -O binary $<TARGET_FILE:firmware> firmware.bin
  BYPRODUCTS firmware.bin
  VERBATIM
)

set(UPLOAD_TOOL "openocd" CACHE STRING "The program to use for flashing the firmware")
set_property(CACHE UPLOAD_TOOL PROPERTY STRINGS "openocd" "pyocd" "stlink")

set(upload_command)
if(UPLOAD_TOOL STREQUAL "openocd")
  find_program(OPENOCD "openocd")
  if(OPENOCD)
    set(upload_command "${OPENOCD}"
      -f interface/stlink.cfg
      -c "transport select hla_swd"
      -f target/stm32f4x.cfg
      # Braces in Tcl (the language OpenOCD uses for its scripts) act like
      # single quotes in shell, i.e. disable interpolation and symbols escapes.
      -c "program {$<TARGET_FILE:firmware>} verify reset exit"
    )
  endif()
elseif(UPLOAD_TOOL STREQUAL "pyocd")
  find_program(PYOCD "pyocd")
  if(PYOCD)
    # IIRC, pyocd sometimes fails to reboot the MCU after flashing is complete.
    set(upload_command "${PYOCD}" flash -t stm32f411ceux $<TARGET_FILE:firmware>)
  endif()
elseif(UPLOAD_TOOL STREQUAL "stlink")
  find_program(STFLASH "st-flash")
  if(STFLASH)
    # In my experience, at least with my specific board, st-flash is really
    # unreliable (it often crashes and can't complete the flashing process).
    set(upload_command "${STFLASH}" --reset write firmware.bin 0x08000000)
  endif()
elseif(UPLOAD_TOOL)
  message(FATAL_ERROR "Unknown upload tool ${UPLOAD_TOOL}")
endif()

if(upload_command)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(firmware_target_comment $<PATH:RELATIVE_PATH,$<TARGET_FILE:firmware>,${CMAKE_BINARY_DIR}>)
  else()
    set(firmware_target_comment $<TARGET_FILE_NAME:firmware>)
  endif()
  add_custom_target(upload
    COMMAND ${upload_command}
    COMMENT "Uploading ${firmware_target_comment}"
    DEPENDS firmware
    VERBATIM
  )
elseif(UPLOAD_TOOL)
  message(FATAL_ERROR "Upload tool ${UPLOAD_TOOL} is not installed")
endif()

target_compile_definitions(stm32f4xx_hal PUBLIC
  STM32F4xx STM32F411xE USE_HAL_DRIVER USE_FULL_LL_DRIVER
)

target_include_directories(stm32f4xx_hal PRIVATE src)   # stm32f4xx_hal_conf.h, stm32_assert.h
target_include_directories(stm32_usb_host PRIVATE src)  # usbh_conf.h
target_include_directories(FatFs PRIVATE src)           # ffconf.h

target_include_directories(firmware PUBLIC src)

add_subdirectory(src)
