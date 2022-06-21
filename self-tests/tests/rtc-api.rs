#![no_std]
#![no_main]

use defmt_rtt as _; // transport layer for defmt logs
use lpc546xx_hal::{self as _, rtc::Rtc};
use panic_probe as _; // panicking behavior
struct State {
    // state shared between `#[test]` functions
    rtc: Option<Rtc>,
}

#[defmt_test::tests]
mod tests {
    use super::State;
    use defmt::assert;
    use lpc546xx_hal::{
        pac,
        prelude::*,
        rtc::{NaiveDateTime, Rtc},
        syscon::Config,
    }; // the HAL we'll test
    #[init]
    fn init() -> State {
        let dp = pac::Peripherals::take().unwrap();
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let rtc = Rtc::new(dp.RTC, &mut syscon, None).unwrap();
        State { rtc: Some(rtc) }
    }

    /// test setting the time and reading it back
    #[test]
    fn test_time_setting(state: &mut State) {
        let mut rtc = state.rtc.take().unwrap();
        rtc.set_time(NaiveDateTime::from_timestamp(0 as i64, 0));
        assert!(rtc.get_time().timestamp() == 0);

        rtc.set_time(NaiveDateTime::from_timestamp(100000 as i64, 0));
        assert!(rtc.get_time().timestamp() == 100000);

        rtc.set_time(NaiveDateTime::from_timestamp(123456789 as i64, 0));
        assert!(rtc.get_time().timestamp() == 123456789);
        state.rtc = Some(rtc)
    }

    /// test a waiting a second using the alarm
    #[test]
    fn test_alarm_setting(state: &mut State) {
        let mut rtc = state.rtc.take().unwrap();
        let timestamp = rtc.get_time().timestamp() as u32;
        rtc.set_time(NaiveDateTime::from_timestamp(timestamp as i64, 0));
        rtc.set_alarm_from_timestamp(timestamp + 1).unwrap();
        nb::block!(rtc.wait()).unwrap();
        assert!(rtc.get_time().timestamp() as u32 == timestamp + 1);
        state.rtc = Some(rtc)
    }

    /// test write+read back general purposes registers of the rtc
    #[test]
    fn test_gp_registers(state: &mut State) {
        let mut rtc = state.rtc.take().unwrap();
        for i in 0..7 {
            rtc.write_general_purpose_registers(i as usize, !i).unwrap();
        }
        for i in 0..7 as u32 {
            assert!(rtc.read_general_purpose_registers(i as usize).unwrap() == !i);
        }
        state.rtc = Some(rtc)
    }
}
