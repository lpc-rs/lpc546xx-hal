#![deny(warnings)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use lpc546xx_hal::{self as _, crc::Crc};
struct State {
    crc: Option<Crc>,
}

#[defmt_test::tests]
mod tests {
    use defmt::assert;
    use lpc546xx_hal::{
        crc::{self},
        pac,
        prelude::*,
        syscon::Config,
    }; // the HAL we'll test

    use crate::State;

    #[init]
    fn init() -> State {
        let dp = pac::Peripherals::take().unwrap();
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());

        let crc = dp
            .CRC_ENGINE
            .crc(crc::Config::default(), &mut syscon)
            .unwrap();

        State { crc: Some(crc) }
    }

    /// testing CRC ccitt, comparing with results from https://crccalc.com/
    #[test]
    fn crc_ccitt(state: &mut State) {
        let mut crc = state.crc.take().unwrap();

        crc.configure(crc::Config::crc_ccitt());

        let test_vector = "123456789";
        let check = 0x29B1;

        crc.feed(test_vector.as_bytes());
        assert!(crc.peek_result() == check);
        state.crc = Some(crc);
    }

    /// testing CRC-16, comparing with results from https://crccalc.com/
    #[test]
    fn crc_16(state: &mut State) {
        let mut crc = state.crc.take().unwrap();

        crc.configure(crc::Config::crc_16());

        let test_vector = "123456789";
        let check = 0xBB3D;

        crc.feed(test_vector.as_bytes());
        assert!(crc.peek_result() == check);
        state.crc = Some(crc);
    }

    /// testing CRC-32, comparing with results from https://crccalc.com/
    #[test]
    fn crc_32(state: &mut State) {
        let mut crc = state.crc.take().unwrap();

        crc.configure(crc::Config::crc_32());

        let test_vector = "123456789";
        let check = 0xCBF43926;
        crc.feed(test_vector.as_bytes());
        assert!(crc.peek_result() == check);
        state.crc = Some(crc);
    }
}
