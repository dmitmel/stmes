# Configuration of the compilation flags shared between the firmware, the tools
# and other new subprojects potentially required in the future.

function(add_common_flags_to_target target)
  # <https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html>
  # TODO: Enable more warnings:
  # <https://github.com/embeddedartistry/meson-buildsystem/blob/d4de8c83f97cb6389d9895548b555220f9c56e53/compiler/meson.build#L8-L49>
  # <https://github.com/embeddedartistry/meson-buildsystem/blob/d4de8c83f97cb6389d9895548b555220f9c56e53/compiler/cpp/meson.build#L11-L21>
  # <https://github.com/embeddedartistry/cmake-buildsystem/blob/f442d5ddbdd87f7590cb06cdbf6c319fa119578e/compiler/DefaultCompilerSettings.cmake#L9-L69>
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic -pedantic-errors
    -Werror=return-type
    -Wdouble-promotion
    -Wmissing-declarations
    -Werror=float-conversion
    $<$<COMPILE_LANGUAGE:C>:-Werror=strict-prototypes>
  )
endfunction()
