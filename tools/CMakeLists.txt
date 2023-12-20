cmake_minimum_required(VERSION 3.5)
project(stmes_tools C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS YES)
set(CMAKE_C_STANDARD_REQUIRED YES)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS YES)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

add_library(test_linker_plugin SHARED test_linker_plugin.cc)

# TODO: Share this with the top-level CMakeLists.txt somehow
target_compile_options(test_linker_plugin PRIVATE
  -Wall -Wextra -Wpedantic -pedantic-errors
  -Werror=return-type
  -Wdouble-promotion
  -Wmissing-declarations
  -Werror=float-conversion
  $<$<COMPILE_LANGUAGE:C>:-Werror=strict-prototypes>
)
