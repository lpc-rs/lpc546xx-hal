// Required connections:
//
// - P1.17 <-> P1.18

#![deny(warnings)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use lpc546xx_hal::{
    self as _,
    gpio::{
        gpio::{P1_17, P1_18},
        Floating, Input, OpenDrain, Output,
    },
};
use panic_probe as _;
struct State {
    input_pin: Option<P1_17<Input<Floating>>>,
    output_pin: P1_18<Output<OpenDrain>>,
}

#[defmt_test::tests]
mod tests {
    use cortex_m::asm;
    use defmt::{assert, unwrap};
    use lpc546xx_hal::{pac, prelude::*, syscon::Config}; // the HAL we'll test

    use super::State;

    #[init]
    fn init() -> State {
        let dp = pac::Peripherals::take().unwrap();
        let mut iocon = dp.IOCON;
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let input_pin = Some(gpio.pio1_17.into_floating_input());
        let output_pin = gpio.pio1_18.into_open_drain_output();

        State {
            input_pin,
            output_pin,
        }
    }

    #[test]
    fn set_low_is_low(state: &mut State) {
        state.output_pin.set_low().unwrap();

        let input_pin = unwrap!(state.input_pin.take());

        let floating_input = input_pin.into_floating_input();
        asm::delay(100);
        assert!(floating_input.is_low().unwrap());

        let pullup_input = floating_input.into_pull_up_input();
        asm::delay(100);
        assert!(pullup_input.is_low().unwrap());

        let pulldown_input = pullup_input.into_pull_down_input();
        asm::delay(100);
        assert!(pulldown_input.is_low().unwrap());

        state.input_pin = Some(pulldown_input.into_floating_input());
    }

    // with the current API we cannot test this w/o an _external_ pull-up

    #[test]
    fn set_high_is_high(state: &mut State) {
        state.output_pin.set_high().unwrap();

        let input_pin = unwrap!(state.input_pin.take());

        let floating_input = input_pin.into_floating_input();
        // no point in testing floating connected to floating anyway

        let pullup_input = floating_input.into_pull_up_input();
        asm::delay(100);
        assert!(pullup_input.is_high().unwrap());

        let pulldown_input = pullup_input.into_pull_down_input();
        asm::delay(100);
        assert!(pulldown_input.is_low().unwrap());

        state.input_pin = Some(pulldown_input.into_floating_input());
    }
}
