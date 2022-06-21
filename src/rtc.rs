//! RTC peripheral implementation
//!
use crate::{pac::RTC, syscon::ClockControl};

use crate::syscon::Syscon;
pub use rtcc::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};

#[derive(Debug, Clone, Copy)]

/// RTC driver error type
pub enum Error {
    /// General purpose register does not exist
    RegisterDoesNotExist,
}

/// Entry point to the RTC API.
pub struct Rtc {
    rtc: RTC,
}

impl Rtc {
    /// Initialize RTC peripheral
    ///
    /// The `init` argument will only be used if the rtc does not hold a valid time value
    pub fn new(rtc: RTC, syscon: &mut Syscon, init: Option<NaiveDateTime>) -> Result<Self, Error>
    where
        RTC: ClockControl,
    {
        rtc.enable_clock(syscon);
        // ensure the reset bit is cleared
        rtc.ctrl.modify(|_, w| w.swreset().not_in_reset());
        // ensure the rtc osc is powered up
        rtc.ctrl.modify(|_, w| w.rtc_osc_pd().power_up());
        let mut rtc = Self { rtc };
        if rtc.get_time() == NaiveDateTime::from_timestamp(0, 0) {
            rtc.set_time(init.unwrap_or_else(|| NaiveDateTime::from_timestamp(0, 0)));
        }
        rtc.enable();
        Ok(rtc)
    }

    /// this function returns the time argument converted to the number of seconds since 1970-01-01T00:00:00 as u32
    fn time_to_seconds(time: &NaiveDateTime) -> u32 {
        time.signed_duration_since(NaiveDate::from_ymd(1970, 1, 1).and_hms(0, 0, 0))
            .num_seconds() as u32
    }

    /// enable the RTC counter
    fn enable(&mut self) {
        self.rtc.ctrl.modify(|_, w| w.rtc_en().enable());
    }

    /// disable the RTC counter
    fn disable(&mut self) {
        self.rtc.ctrl.modify(|_, w| w.rtc_en().disable());
    }

    /// returns the enable/disable status for the RTC counter
    fn is_enabled(&self) -> bool {
        self.rtc.ctrl.read().rtc_en().is_enable()
    }

    /// Sets the date/time
    pub fn set_time(&mut self, time: NaiveDateTime) {
        let seconds = Self::time_to_seconds(&time);
        if self.is_enabled() {
            self.disable();
            self.rtc.count.write(|w| unsafe { w.val().bits(seconds) });
            self.enable();
        } else {
            self.rtc.count.write(|w| unsafe { w.val().bits(seconds) });
        }
    }

    /// Returns the current date/time
    pub fn get_time(&self) -> NaiveDateTime {
        let seconds = self.rtc.count.read().val().bits();
        NaiveDateTime::from_timestamp(seconds as i64, 0)
    }

    /// Write general purpose registers
    ///
    /// These register retain contents even during deep power-down mode as long as device power is maintained. They can be used to preserve application data or configuration that will always be available.
    pub fn write_general_purpose_registers(
        &mut self,
        index: usize,
        value: u32,
    ) -> Result<(), Error> {
        if index >= 8 {
            return Err(Error::RegisterDoesNotExist);
        }
        self.rtc.gpreg[index].write(|w| unsafe { w.gpdata().bits(value) });
        Ok(())
    }

    /// Read general purpose registers
    ///
    /// These register retain contents even during deep power-down mode as long as device power is maintained. They can be used to preserve application data or configuration that will always be available.
    pub fn read_general_purpose_registers(&self, index: usize) -> Result<u32, Error> {
        if index > 7 {
            return Err(Error::RegisterDoesNotExist);
        }
        Ok(self.rtc.gpreg[index].read().gpdata().bits())
    }

    /// Set the RTC alarm from timestamp
    ///
    /// The alarm will trigger the RTC interrupt when the time matches the `alarm_time`.
    pub fn set_alarm_from_timestamp(&mut self, alarm_time: u32) -> Result<(), Error> {
        self.rtc
            .match_
            .write(|w| unsafe { w.matval().bits(alarm_time) });
        Ok(())
    }

    /// Set the RTC alarm from date/time
    ///
    /// The alarm will trigger the RTC interrupt when the time matches the `alarm_time`.
    /// The `alarm_time` is converted to the number of seconds since 1970-01-01T00:00:00 as u32
    pub fn set_alarm_from_datetime(&mut self, alarm_time: NaiveDateTime) -> Result<(), Error> {
        self.set_alarm_from_timestamp(Self::time_to_seconds(&alarm_time))
    }

    /// Wait for the RTC alarm to trigger
    ///
    /// This function will return nb::WouldBlock until the RTC alarm is triggered.
    pub fn wait(&self) -> nb::Result<(), Error> {
        let ctrl = self.rtc.ctrl.read();
        if ctrl.alarm1hz().is_match() {
            self.rtc.ctrl.modify(|_, w| w.alarm1hz().set_bit()); // clear bit
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl rtcc::DateTimeAccess for Rtc {
    type Error = Error;

    fn datetime(&mut self) -> Result<NaiveDateTime, Self::Error> {
        Ok(self.get_time())
    }

    fn set_datetime(&mut self, datetime: &NaiveDateTime) -> Result<(), Self::Error> {
        self.set_time(*datetime);
        Ok(())
    }
}
