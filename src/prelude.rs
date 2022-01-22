pub use embedded_hal::{digital::v2::*, prelude::*};

pub use crate::{gpio::GpioExt as _, syscon::SysconExt as _};

// Make items like `.Hz()`, `.microseconds()` available from embedded-time
pub use embedded_time::{
    duration::Extensions as DurationExtensions, rate::Extensions as RateExtensions,
};

pub use crate::serial::Serial0Ext as _;
pub use crate::serial::Serial1Ext as _;
pub use crate::serial::Serial2Ext as _;
pub use crate::serial::Serial3Ext as _;
pub use crate::serial::Serial4Ext as _;
pub use crate::serial::Serial5Ext as _;
pub use crate::serial::Serial6Ext as _;
pub use crate::serial::Serial7Ext as _;
pub use crate::serial::Serial8Ext as _;
#[cfg(feature = "flexcomm-10")]
pub use crate::serial::Serial9Ext as _;
