PLATFORMIO ?= platformio
PIO_ENV ?= debug

-include local.mk

.PHONY: all
all: build

.PHONY: build build_all
build:
	$(PLATFORMIO) run --environment=$(PIO_ENV)
build_all:
	$(PLATFORMIO) run

.PHONY: check check_all
check:
	$(PLATFORMIO) check --environment=$(PIO_ENV)
check_all:
	$(PLATFORMIO) check

.PHONY: clean upload uploadfs
upload uploadfs:
	$(PLATFORMIO) run --environment=$(PIO_ENV) --target=$@
clean:
	$(PLATFORMIO) run --target=$@

.PHONY: compile_commands.json
compile_commands.json:
	$(PLATFORMIO) run --environment=$(PIO_ENV) --target=compiledb
