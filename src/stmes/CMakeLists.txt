target_sources(firmware PRIVATE
  fatfs.c
  gpio.c
  interrupts.c
  main.c
  newlib_support.c
  utils.c
)

add_subdirectory(demos)
add_subdirectory(drivers)
add_subdirectory(kernel)
add_subdirectory(video)
