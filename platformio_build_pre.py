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
