#!/usr/bin/make
SHELL := /bin/bash

# TODO: use shell name to determine system/give users option to setup their
# compiler env with make
CI_SYSTEM ?= $(shell uname)

GCC_DIR := $(HOME)/gcc
DIRS = mbed src
DIRSCLEAN = $(addsuffix .clean,$(DIRS))

all:
	@ $(MAKE) -C mbed
	@echo Building Smoothie
	@ $(MAKE) -C src

.PHONY: build
# build hex file with correct configs for opentrons
build:
	@ $(MAKE) all AXIS=6 PAXIS=4 CNC=1 DISABLEMSD=1

clean: $(DIRSCLEAN)

$(DIRSCLEAN): %.clean:
	@echo Cleaning $*
	@ $(MAKE) -C $*  clean

.PHONY: debug-store
debug-store:
	@ $(MAKE) -C src debug-store

.PHONY: flash
flash:
	@ $(MAKE) -C src flash

.PHONY: dfu
dfu:
	@ $(MAKE) -C src dfu

.PHONY: upload
# upload to smoothie board
upload:
	@ $(MAKE) -C src upload

.PHONY: debug
# put smoothie board in debug mode
debug:
	@ $(MAKE) -C src debug

.PHONY: console
# open console connected to smoothie board
console:
	@ $(MAKE) -C src console

.PHONY: opentrons
opentrons:
	make all AXIS=6 PAXIS=4 CNC=1 DISABLEMSD=1
	./build/osx64/lpc21isp -wipe -donotstart ./LPC1768/main.hex /dev/tty.usbserial-A9071HLU 115200 12000

.PHONY: all $(DIRS) $(DIRSCLEAN) debug-store flash upload debug console dfu
