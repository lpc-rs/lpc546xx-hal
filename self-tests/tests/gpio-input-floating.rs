// Required connections:
//
// - P4.4 <-> GND
// - P4.5 <-> VDD

#![no_std]
#![no_main]

use defmt_rtt as _; // transport layer for defmt logs
use lpc546xx_hal::{
    self as _,
    gpio::{
        gpio::{P4_4, P4_5},
        Floating, Input,
    },
};
use panic_probe as _; // panicking behavior

struct State {
    // state shared between `#[test]` functions
    input_ground: Option<P4_4<Input<Floating>>>,
    input_vdd: Option<P4_5<Input<Floating>>>,
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

        let input_ground = gpio.pio4_4.into_floating_input();
        let input_vdd = gpio.pio4_5.into_floating_input();
        State {
            input_ground: Some(input_ground),
            input_vdd: Some(input_vdd),
        }
    }

    #[test]
    fn ground_pulled_high_is_low(state: &mut State) {
        // take the pin out of the state, put into a pull-up input
        // assert that it is low and put back into the state
        let input_ground = state.input_ground.take().unwrap();
        let input_ground = input_ground.into_pull_up_input();
        assert!(input_ground.is_low().unwrap());
        state.input_ground = Some(input_ground.into_floating_input());
    }

    #[test]
    fn vdd_pulled_low_is_high(state: &mut State) {
        // take the pin out of the state, put into a pull-down input
        // assert that it is high and put back into the state
        let input_vdd = state.input_vdd.take().unwrap();
        let input_vdd = input_vdd.into_pull_down_input();
        assert!(input_vdd.is_high().unwrap());
        state.input_vdd = Some(input_vdd.into_floating_input());
    }

    #[test]
    fn ground_floating_is_low(state: &mut State) {
        // take the pin out of the state, assert that it is low and put back into the state
        let input_ground = state.input_ground.take().unwrap();
        assert!(input_ground.is_low().unwrap());
        state.input_ground = Some(input_ground);
    }

    #[test]
    fn vdd_floating_is_high(state: &mut State) {
        // take the pin out of the state, assert that it is high and put back into the state
        let input_vdd = state.input_vdd.take().unwrap();
        assert!(input_vdd.is_high().unwrap());
        state.input_vdd = Some(input_vdd);
    }
    #[test]
    fn always_passes() {
        assert!(true);
    }
}
