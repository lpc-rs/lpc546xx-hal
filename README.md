> PR welcome! 

# lpc546xx-hal [![crates.io](https://img.shields.io/crates/v/lpc546xx-hal.svg)](https://crates.io/crates/lpc546xx-hal) [![Documentation](https://docs.rs/lpc546xx-hal/badge.svg)](https://docs.rs/lpc546xx-hal) 

Hardware Abstraction Layer crate for the lpc546xx family.

This crate relies on the [lpc546xx-pac] crate to provide appropriate register definition and implements a partial set of [embedded-hal] traits.  

[lpc546xx-pac]: https://crates.io/crates/lpc546xx-pac
[embedded-hal]: https://github.com/rust-embedded/embedded-hal

## Usage

Add the `lpc546xx-hal` crate to your dependencies in your Cargo.toml and make sure to pick the appropriate `mcu-*` feature to enjoy the full feature set for your MCU.

For example, for the LPCXpresso54608 board that sports a LPC54608J512ET180: 

```toml
lpc546xx-hal = { version = "0.1.0", features = ["mcu-LPC54608J512ET180", "rt"] }
```

This will select the appropriate memory.x linker script depending on both your RAM and FLASH, the correct feature set from the peripheral access crate, and the correct interrupt vector for your device.

If you look into [`Cargo.toml` file](https://github.com/lpc-rs/lpc546xx-hal/blob/master/Cargo.toml), you will see the supported devices. 

If you want to run an example, you can run it by using:

```console
$ cargo run --example gpio --features=mcu-LPC54608J512ET180,rt  
```

This HAL will also include a `__pre_init` stub in `startup-code/libstartup.a` to be called by cortex-m-rt when the "rt" feature is enabled. In this stub all the rams are turned on. you can disable this feature by using the `disable-linker-script` feature. But you should do it yourself, otherwise your program will not boot, as `cortex-m-rt` will try to init memory region that are not enabled.


# Toolchain Setup

In order to use this HAL, you need the following Setup:

1. Install Rustup

    See [rustup.rs](https://rustup.rs/) for details. You may als be able to
    install Rustup directly through your distro.

2. Install the `arm-none-eabi` compiler toolchain

	https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

    If you cannot install the toolchain directly through your OS / distro, we
    recommend installing the precompiled binaries to '/usr/local/opt'.  Add the
    bin folders (/bin & /arm-none-eabi/bin) to your environments variable 'PATH'.

3. Install the `thumbv7em-none-eabi/hf` target for Rust

    Simply run `rustup target add thumbv7em-none-eabi` 
    or `rustup target add thumbv7em-none-eabihf`
    
For more instructions on how to get started with ARM / Cortex-M programming
using Rust, check out the [Embedded Rust
Book](https://rust-embedded.github.io/book/).

# Build Examples

You can build examples through Cargo:

    $  cargo build --example gpio --features="mcu-LPC54608J512ET180,rt"  

Note that not all examples are compatible with all MCUs. You might need to peek
into the example source code.


# License

0-Clause BSD License, see [LICENSE-0BSD.txt](LICENSE-0BSD.txt) for more details.
