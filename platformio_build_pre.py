from SCons.Script import COMMAND_LINE_TARGETS

Import("env")


def normalize_bool_option(value):
  return str(value).lower() in ("1", "true", "yes", "enable", "on")


if normalize_bool_option(env.GetProjectOption("arm_semihosting", False)):
  env.Append(
    BUILD_UNFLAGS=["-lnosys", "--specs=nosys.specs"],
    LINKFLAGS=["--specs=rdimon.specs"],
    LIBS=["rdimon"],
    CPPDEFINES=["ARM_SEMIHOSTING_ENABLE"],
  )

fpu_flags = ["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"]
env.Append(CCFLAGS=fpu_flags, ASFLAGS=fpu_flags, LINKFLAGS=fpu_flags)

env.Append(CPPDEFINES=["USE_FULL_LL_DRIVER"])

# Enable level 3 debugging information for GDB, which among other things
# includes macros. By default only level 2 is enabled.
env.Replace(PIODEBUGFLAGS=["-Og", "-ggdb3"])

# clangd gets confused by optimization options clang itself doesn't have, so
# avoid including them in `compile_commands.json`.
if "compiledb" not in COMMAND_LINE_TARGETS and not env.IsIntegrationDump():
  # GCC optimization options:
  # <https://gcc.gnu.org/onlinedocs/gcc-9.5.0/gcc/Optimize-Options.html>
  # <https://github.com/gcc-mirror/gcc/blob/releases/gcc-9.2.0/gcc/opts.c#L443>
  env.Append(
    PIODEBUGFLAGS=[
      # These are not enabled by -Og
      "-fif-conversion",  # Convert ifs into branchless equivalents
      "-fif-conversion2",  # Convert ifs into instructions with condition flags
    ]
  )
