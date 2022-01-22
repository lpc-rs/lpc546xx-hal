#![cfg_attr(not(test), no_std)]
#![allow(non_camel_case_types)]
#![allow(clippy::upper_case_acronyms)]

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

pub mod flexcomm;
pub mod gpio;
pub mod prelude;
pub mod serial;
pub mod syscon;
