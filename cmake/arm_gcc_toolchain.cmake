# Very indirectly based on <https://github.com/vpetrigo/arm-cmake-toolchains/blob/master/arm-gcc-toolchain.cmake>

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING YES)
set(TOOLCHAIN_TARGET arm-none-eabi)

set(CMAKE_C_COMPILER   "${TOOLCHAIN_TARGET}-gcc" CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_TARGET}-g++" CACHE STRING "" FORCE)
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_TARGET}-gcc" CACHE STRING "" FORCE)

# NOTE: Under the hood, the main GCC executable acts as a driver for a bunch of
# lower-level helper programs, invoking them to do the actual work of
# compiling, assembling and linking. What commands are executed as part of the
# compilation process and which options are passed to those programs depending
# on the flags given to the compiler frontend, under GCC is controlled by the
# so-called Spec files: <https://gcc.gnu.org/onlinedocs/gcc/Spec-Files.html>.
# These files are basically a list of rules that the frontend pulls and
# combines together to form the necessary command lines. Newlib (the libc for
# embedded targets) ships with its set of additional spec files, which override
# the flags used when linking the standard library, in order to choose between
# different variants of Newlib. In particular, this will switch to the "nano"
# variant of libc, which obviously is smaller, consumes less memory, but
# implements less features and may perform worse (due to size-speed trade-off).
# The standard spec file may be viewed with `arm-none-eabi-gcc -dumpspecs`.
# <https://github.com/bminor/newlib/blob/newlib-4.3.0/libgloss/arm/elf-nano.specs>
add_link_options(-specs=nano.specs)

option(ARM_SEMIHOSTING "" OFF)
# NOTE: For portability Newlib requires a layer of functions to be defined for
# interfacing with the underlying OS or a bare-metal system, the whole list of
# which can be found here: <https://sourceware.org/newlib/libc.html#Syscalls>.
# Problem is, without it being present even the simplest programs with just an
# empty main() function will refuse to link with libc, so CMake won't be able
# to pass the compiler identification step. Fortunately, Newlib also comes with
# a bunch of spec files to select built-in stubs for implementing system calls,
# which must be enabled as early as possible, so that compiler identification
# succeeds after processing the toolchain script, and try_compile() tests work.
# The binaries produced for compiler feature tests are never run, all the
# required data is extracted by dissecting resulting binaries, they only need
# to link successfully.
if(ARM_SEMIHOSTING)
  # Forward the syscalls to the host machine connected by a debugger probe with
  # semihosting facility: <https://interrupt.memfault.com/blog/arm-semihosting>
  add_link_options(-specs=rdimon.specs)
  add_compile_definitions(ARM_SEMIHOSTING_ENABLE)
else()
  # Provide empty stub functions that do nothing but set `errno` to something
  # along the lines of "unsupported". See their source code here if you don't
  # belive me: <https://github.com/bminor/newlib/tree/newlib-4.3.0/libgloss/libnosys>
  add_link_options(-specs=nosys.specs)
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
