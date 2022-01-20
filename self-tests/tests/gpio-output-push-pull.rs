// Required connections:
//
// - P0.28 <-> P0.29

#![deny(warnings)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use lpc546xx_hal::{
    self as _,
    gpio::{
        gpio::{P1_17, P1_18},
        Floating, Input, Output, PushPull,
    },
};
struct State {
    input_pin: P1_17<Input<Floating>>,
    output_pin: P1_18<Output<PushPull>>,
}

#[defmt_test::tests]
mod tests {
    use cortex_m::asm;
    use defmt::assert;
    use lpc546xx_hal::{pac, prelude::*, syscon::Config}; // the HAL we'll test

    use super::State;

    #[init]
    fn init() -> State {
        let dp = pac::Peripherals::take().unwrap();
        let mut iocon = dp.IOCON;
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let input_pin = gpio.pio1_17.into_floating_input();
        let output_pin = gpio.pio1_18.into_push_pull_output();
        State {
            input_pin,
            output_pin,
        }
    }

    #[test]
    fn set_low_is_low(state: &mut State) {
        state.output_pin.set_low().unwrap();
        // GPIO operations are not instantaneous so a delay is needed
        asm::delay(100);
        assert!(state.input_pin.is_low().unwrap());
    }

    #[test]
    fn set_high_is_high(state: &mut State) {
        state.output_pin.set_high().unwrap();
        // GPIO operations are not instantaneous so a delay is needed
        asm::delay(100);
        assert!(state.input_pin.is_high().unwrap());
    }
}
