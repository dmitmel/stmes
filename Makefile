# Just a wrapper around CMake.
# <https://www.gnu.org/software/make/manual/make.html>

# Start off by declaring the default target as the very first rule in the file.
default: all
.PHONY: default

# Only allow running the targets from this makefile sequentially
# (CMake-generated makefiles don't support parallel invocation).
.NOTPARALLEL:

CMAKE ?= cmake
NINJA ?= ninja

CMAKE_GENERATOR ?= Unix Makefiles
# The `override` keyword here and below makes it so our options are added to
# CMAKE_FLAGS no matter what, even if the variable has been defined on the
# command line by the user.
override CMAKE_FLAGS += -G'$(CMAKE_GENERATOR)'

CMAKE_BUILD_TYPE ?= Debug
override CMAKE_FLAGS += -DCMAKE_BUILD_TYPE='$(CMAKE_BUILD_TYPE)'

CMAKE_TOOLCHAIN_FILE ?= cmake/arm_gcc_toolchain.cmake
override CMAKE_FLAGS += -DCMAKE_TOOLCHAIN_FILE='$(CMAKE_TOOLCHAIN_FILE)'

BUILD_DIR ?= target/$(CMAKE_BUILD_TYPE)
DEPENDENCIES_DIR ?= target/_deps
override CMAKE_FLAGS += -DDEPENDENCIES_DIR='$(DEPENDENCIES_DIR)'

override CMAKE_FLAGS += --warn-uninitialized

# Include the local overrides file in case it exists.
-include local.mk

# Make doesn't offer a way of escaping special characters in shell commands,
# recommending to put them in variables instead, so enjoy this smiley face.
closeparen := )

# Based on <https://github.com/neovim/neovim/blob/v0.9.4/Makefile#L56-L65>.
ifneq (,$(findstring Ninja,$(CMAKE_GENERATOR)))
  BUILD_TOOL = $(NINJA)
  BUILD_DIR_STAMP = $(BUILD_DIR)/build.ninja
  # Propagate certain options (passed by the user to Make) when invoking Ninja,
  # normal Make inherits those automatically.
  # <https://www.gnu.org/software/make/manual/make.html#Options_002fRecursion>
  ifneq ($(VERBOSE),)
    BUILD_TOOL += -v  # verbose (print the commands being executed)
  endif
  ifeq (n,$(findstring n,$(firstword -$(MAKEFLAGS))))
    BUILD_TOOL += -n  # dry run (just print the commands without running them)
  endif
  ifeq (k,$(findstring k,$(firstword -$(MAKEFLAGS))))
    BUILD_TOOL += -k0  # keep going (try to make as many targets as possible, even if one fails)
  endif
  # Carve the -j (number of parallel jobs) and -l (maximum load average) out of
  # MAKEFLAGS. These will be passed as separate shell words in the string, so,
  # naturally, I use the shell for parsing it. The script splits the MAKEFLAGS
  # string into words and loops over them, filtering out the flags we are
  # looking for (the first branch drops instances where the option has a digit
  # and contains some non-digit garbage in the middle; the second branch
  # catches the valid forms; the third branch skips everything after a `--`).
  # Notice how this script doesn't invoke any external commands!
  BUILD_TOOL += $(shell \
    for arg in $(MAKEFLAGS); do case "$$arg" in \
      -[jl][0-9]*[!0-9]*$(closeparen) ;; \
      -[jl][0-9]*$(closeparen) printf '%s\n' "$$arg";; \
      --$(closeparen) break;; \
    esac; done)
else
  BUILD_TOOL = $(MAKE)
  BUILD_DIR_STAMP = $(BUILD_DIR)/Makefile
  # A convenience thingy for running the build in parallel by default when
  # using regular Make (typing -j8 every time gets really irritating). First,
  # check if a jobs number argument has already been specified on the command
  # line using a simplified version of the script above.
  ifeq (,$(shell for arg in $(MAKEFLAGS); do case "$$arg" in \
      -j*$(closeparen) printf 1;; --$(closeparen) break;; esac; done))
    # If the -j flag is not present, pass the number of available CPU cores.
    BUILD_TOOL += -j$(shell nproc)
  endif
endif

.PHONY: cmake freshcmake
# The regular `cmake` target simply runs CMake (the -B signature has been added
# in v3.13), the `freshcmake` runs it with the --fresh option (added in CMake
# v3.24) using a target-specific variable assignment. The $(BUILD_DIR_STAMP) is
# some file in the generated build system directory whose presence signifies
# that the build scaffolding has been successfully generated. Unlike the former
# two rules it is not a phony target, so other rules may use it for checking
# the presence of the build directory.
freshcmake: override CMAKE_FLAGS += --fresh
cmake freshcmake $(BUILD_DIR_STAMP):
	$(CMAKE) $(CMAKE_FLAGS) -B '$(BUILD_DIR)'

.PHONY: ensurecmake
# This rule simply checks that the build system has been generated.
ensurecmake: $(BUILD_DIR_STAMP)

.PHONY: all firmware tools upload clean
# The targets from the actual build system that will be exposed to the user.
# $(BUILD_DIR_STAMP) is an order-only dependency here: Make will ensure that it
# is made the first time, but won't care about file modification times later.
# Afterwards, it is CMake's responsibility to regenerate the files when it
# detects a change in any `CMakeLists.txt`. The plus sign before the command
# tells Make to execute it even in the dry-run mode.
all firmware tools upload clean: | $(BUILD_DIR_STAMP)
	+$(BUILD_TOOL) -C '$(BUILD_DIR)' $@

.PHONY: distclean
# Completely deletes the build directory.
distclean:
	rm -rf 'target'

compile_commands.json: $(BUILD_DIR)/compile_commands.json
tools/compile_commands.json: $(BUILD_DIR)/tools/compile_commands.json
compile_commands.json tools/compile_commands.json: | $(BUILD_DIR_STAMP)
	ln -sr '$<' '$@'
