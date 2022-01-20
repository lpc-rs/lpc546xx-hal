#!/bin/sh

# Compiles `flash.c` using the GCC toolchain. `build.rs` makes sure that the
# resulting library is linked.
#
# Since requiring all users of this HAL to have the GCC toolchain installed
# would be inconvenient for many, the resulting library is checked into the
# repository. After every change to `flash.c`, you need to call this script
# manually and commit the updated binary.

arm-none-eabi-as -march=armv7e-m startup.s -o startup.o &&
arm-none-eabi-ar crs libstartup.a startup.o

