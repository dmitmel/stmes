cmake_minimum_required(VERSION 3.5)
project(stmes_tools C CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/common_flags.cmake")

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS YES)
set(CMAKE_C_STANDARD_REQUIRED YES)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS YES)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

add_library(test_linker_plugin SHARED test_linker_plugin.cc)
add_common_flags_to_target(test_linker_plugin)

# This file will be included by the top-level CMakeLists.txt to avoid having to
# predict the paths to the tool binaries there. Using the export(TARGETS)
# command won't do because targets, even imported ones, assume that they are
# created for the same platform, which is clearly not the case with the tools
# and the cross-compiled firmware.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/targets_info.cmake" CONTENT [=[
set(tools__test_linker_plugin__path "$<TARGET_FILE:test_linker_plugin>")
]=])
