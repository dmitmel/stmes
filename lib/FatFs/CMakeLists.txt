project(FatFs C)

add_library(FatFs STATIC src/ff.c src/ffunicode.c)
target_include_directories(FatFs PUBLIC src)
