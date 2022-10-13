//! lpc546xx-hal crate
//! Implements drivers for the lpc546xx family from NXP
//!
#![cfg_attr(not(test), no_std)]
#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]
#![deny(missing_docs)]

use embedded_hal as hal;

#[cfg(feature = "lpc54605")]
pub use lpc546xx_pac::lpc54605 as pac;
#[cfg(feature = "lpc54606")]
pub use lpc546xx_pac::lpc54606 as pac;
#[cfg(feature = "lpc54607")]
pub use lpc546xx_pac::lpc54607 as pac;
#[cfg(feature = "lpc54608")]
pub use lpc546xx_pac::lpc54608 as pac;
#[cfg(feature = "lpc54616")]
pub use lpc546xx_pac::lpc54616 as pac;
#[cfg(feature = "lpc54618")]
pub use lpc546xx_pac::lpc54618 as pac;
#[cfg(feature = "lpc54628")]
pub use lpc546xx_pac::lpc54628 as pac;

// implement the pre_init as global_asm. This is needed to turn on the ram.
// only if feature disable-pre-init-blob is not set
#[cfg(not(feature = "disable-pre-init-blob"))]
core::arch::global_asm!(
    "
    .text
    .globl __pre_init
    __pre_init:",
    "ldr r0, =0x40000220
    mov r1, #56
    str r1, [r0]
    bx lr"
);

/// CRC engine
pub mod crc;
/// flexcomm module
pub mod flexcomm;
/// gpio module
pub mod gpio;
/// I2C peripheral, implements I2C trough the flexcomm peripherals
pub mod i2c;
/// prelude module
/// this includes declaration for external peripheral for less verbose use
pub mod prelude;
/// rtc module
pub mod rtc;
/// serial peripheral, implements USART trough the flexcomm peripherals
pub mod serial;
/// System confifuration module (clocks & flash)
pub mod syscon;
/// usb module
pub mod usb;
