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

if "compiledb" not in COMMAND_LINE_TARGETS and not env.IsIntegrationDump():
  env.Append(
    PIODEBUGFLAGS=[
      # These are not enabled by -Og
      "-fif-conversion",  # Convert ifs into branchless equivalents
      "-fif-conversion2",  # Convert ifs into instructions with condition flags
    ]
  )
