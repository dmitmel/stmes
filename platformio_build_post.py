Import("projenv")

projenv.Append(
  CCFLAGS=[
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Werror=return-type",
    "-Wdouble-promotion",
    "-Wmissing-declarations",
  ],
  CFLAGS=[
    "-Werror=implicit-function-declaration",
    "-Werror=strict-prototypes",
  ]
)
