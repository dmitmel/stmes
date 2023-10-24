project(printf C)

add_library(printf STATIC printf.c)
target_include_directories(printf INTERFACE "")
