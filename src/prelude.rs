pub use embedded_hal::{digital::v2::*, prelude::*};

pub use crate::{gpio::GpioExt as _, syscon::SysconExt as _};

// Make items like `.Hz()`, `.microseconds()` available from embedded-time
pub use embedded_time::{
    duration::Extensions as DurationExtensions, rate::Extensions as RateExtensions,
};
