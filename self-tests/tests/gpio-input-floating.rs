// Required connections:
//
// - P3.01 <-> GND
// - P1.22 <-> VDD

#![no_std]
#![no_main]

use defmt_rtt as _; // transport layer for defmt logs
use lpc546xx_hal::{
    self as _,
    gpio::{
        gpio::{P1_22, P3_1},
        Floating, Input,
    },
};
use panic_probe as _; // panicking behavior

struct State {
    // state shared between `#[test]` functions
    input_ground: P3_1<Input<Floating>>,
    input_vdd: P1_22<Input<Floating>>,
}

#[defmt_test::tests]
mod tests {
    use super::State;
    use defmt::assert;
    use lpc546xx_hal::{pac, prelude::*, syscon::Config}; // the HAL we'll test
    #[init]
    fn init() -> State {
        let dp = pac::Peripherals::take().unwrap();
        let mut iocon = dp.IOCON;
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let input_ground = gpio.pio3_1.into_floating_input();
        let input_vdd = gpio.pio1_22.into_floating_input();
        State {
            input_ground,
            input_vdd,
        }
    }

    #[test]
    fn ground_is_low(state: &mut State) {
        assert!(state.input_ground.is_low().unwrap());
    }

    #[test]
    fn vdd_is_high(state: &mut State) {
        assert!(state.input_vdd.is_high().unwrap());
    }
    #[test]
    fn always_passes() {
        assert!(true);
    }
}
