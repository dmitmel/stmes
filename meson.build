# NOTE: This is just an unfinished prototype, the script doesn't include all
# the intricacies of the CMake one and would need changes in the Makefile
# driver. I am only adding it to the repository because I've tried out Meson
# prior to implementing a build system based on CMake (which I am more familiar
# with), and it pains me to delete the code.
# <https://github.com/embeddedartistry/meson-buildsystem>
# <https://github.com/embeddedartistry/project-skeleton/blob/master/Makefile>

project(
  'stmes', 'c',
  meson_version: '>=0.55.0',
  subproject_dir: 'lib',
  default_options: [
    'c_std=gnu99',
    'cpp_std=c++11',
    'optimization=g',
  ],
)

stm32_hal = subproject('stm32f4xx_hal_driver')
cmsis_core = subproject('cmsis_core_cm4')
cmsis_device = subproject('cmsis_device_f4')
stm32_usb_host = subproject('stm32_mw_usb_host')
printf = subproject('printf')
fatfs = subproject('FatFs')

firmware_includes = include_directories('src')

firmware_sources = files()
firmware_link_args = []
firmware_link_depends = []
firmware_c_args = []

firmware_c_args += ['-ffunction-sections', '-fdata-sections']
firmware_link_args += ['-Wl,--gc-sections']

foreach linker_script : ['stm32f411ceux.ld', 'link_sections.ld']
  linker_script = meson.current_source_dir() / 'src' / linker_script
  firmware_link_args += ['-T' + linker_script]
  firmware_link_depends += [linker_script]
endforeach

firmware_c_args += ['-DSTM32F4xx', '-DSTM32F411xE', '-DUSE_FULL_LL_DRIVER', '-DUSE_HAL_DRIVER']

subdir('src')

firmware = executable(
  'firmware',
  firmware_sources,
  name_suffix: 'elf',
  include_directories: firmware_includes,
  dependencies: [
    stm32_hal.get_variable('stm32f4xx_hal_driver_dep'),
    cmsis_core.get_variable('cmsis_core_cm4_dep'),
    cmsis_device.get_variable('cmsis_device_f4_dep'),
    stm32_usb_host.get_variable('stm32_usb_host_core_dep'),
    stm32_usb_host.get_variable('stm32_usb_host_hid_class_dep'),
    printf.get_variable('printf_dep'),
    fatfs.get_variable('fatfs_dep'),
  ],
  c_args: firmware_c_args,
  link_args: firmware_link_args,
  link_depends: firmware_link_depends,
  implicit_include_directories: false,
)

custom_target(
  'checkprogsize',
  input: firmware,
  output: ['checkprogsize.txt'],
  command: ['size', '@INPUT@'],
  build_always: true,
)
