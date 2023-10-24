add_subdirectory(FatFs)
add_subdirectory(printf)

#[[
file(READ "${stm32f4xx_hal_driver_SOURCE_DIR}/Src/stm32f4xx_hal.c" version_file_text)
foreach(version_component IN ITEMS MAIN SUB1 SUB2)
  string(REGEX MATCH
    "#define[ \t]+__STM32F4xx_HAL_VERSION_${version_component}[ \t]+\\((0[xX][0-9A-Fa-f]+)[uU]?\\)"
    HAL_VERSION_${version_component} "${version_file_text}")
  math(EXPR HAL_VERSION_${version_component} "${CMAKE_MATCH_1}")
endforeach()
set(HAL_VERSION "${HAL_VERSION_MAIN}.${HAL_VERSION_SUB1}.${HAL_VERSION_SUB2}")
#]]

# The way STM32 libraries are built is largely based on:
# <https://github.com/platformio/platform-ststm32/blob/v16.1.0/builder/frameworks/stm32cube.py>

file(GLOB_RECURSE hal_files RELATIVE "${stm32f4xx_hal_driver_SOURCE_DIR}"
  "${stm32f4xx_hal_driver_SOURCE_DIR}/Src/*.c")
list(FILTER hal_files EXCLUDE REGEX "^Src/Legacy/")
list(FILTER hal_files EXCLUDE REGEX "^Src/.*_template\\.c$")
list(TRANSFORM hal_files PREPEND "${stm32f4xx_hal_driver_SOURCE_DIR}/")

add_library(stm32f4xx_hal STATIC
  ${hal_files} "${cmsis_device_f4_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c"
)
target_include_directories(stm32f4xx_hal PUBLIC
  "${cmsis_core_cm4_SOURCE_DIR}/Include"
  "${cmsis_device_f4_SOURCE_DIR}/Include"
  "${stm32f4xx_hal_driver_SOURCE_DIR}/Inc"
)

file(GLOB_RECURSE usb_sources "${stm32_mw_usb_host_SOURCE_DIR}/Core/Src/*.c")
list(FILTER usb_sources EXCLUDE REGEX "_template\\.c$")
set(usb_includes "${stm32_mw_usb_host_SOURCE_DIR}/Core/Inc")

file(GLOB usb_classes "${stm32_mw_usb_host_SOURCE_DIR}/Class/*")
foreach(usb_class IN LISTS usb_classes)
  if(IS_DIRECTORY "${usb_class}" AND NOT usb_class MATCHES "/Template$")
    file(GLOB_RECURSE usb_class_files "${usb_class}/Src/*.c")
    list(APPEND usb_sources ${usb_class_files})
    list(APPEND usb_includes "${usb_class}/Inc")
  endif()
endforeach()

add_library(stm32_usb_host STATIC ${usb_sources})
target_include_directories(stm32_usb_host PUBLIC ${usb_includes})

target_link_libraries(stm32_usb_host PUBLIC stm32f4xx_hal)
