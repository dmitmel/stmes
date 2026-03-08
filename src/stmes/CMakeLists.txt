target_sources(firmware PRIVATE
  fatfs.c
  gpio.c
  interrupts.c
  main.c
  newlib_support.c
  utils.c
  stm32_hal_utils.c
)

add_subdirectory(demos)
add_subdirectory(drivers)
add_subdirectory(kernel)
add_subdirectory(video)

set(ENABLE_ARM_SEMIHOSTING "${ARM_SEMIHOSTING}")
set(ENABLE_SEGGER_RTT_LOGGING "${SEGGER_RTT}")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY
)
