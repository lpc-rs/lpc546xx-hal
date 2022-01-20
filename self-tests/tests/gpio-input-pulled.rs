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
        Floating, Input,
    },
};
use panic_probe as _;

struct State {
    input_pin: P1_17<Input<Floating>>,
    puller_pin: Option<P1_18<Input<Floating>>>,
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

        let input_pin = gpio.pio1_17.into_floating_input();
        let puller_pin = Some(gpio.pio1_18.into_floating_input());

        State {
            input_pin,
            puller_pin,
        }
    }

    #[test]
    fn pulldown_is_low(state: &mut State) {
        let puller_pin = unwrap!(state.puller_pin.take());

        let pulldown_pin = puller_pin.into_pull_down_input();
        // GPIO re-configuration is not instantaneous so a delay is needed
        asm::delay(100);
        assert!(pulldown_pin.is_low().unwrap());

        state.puller_pin = Some(pulldown_pin.into_floating_input());
    }

    #[test]
    fn pulldown_drives_low(state: &mut State) {
        let puller_pin = unwrap!(state.puller_pin.take());

        let pulldown_pin = puller_pin.into_pull_down_input();
        assert!(state.input_pin.is_low().unwrap());

        state.puller_pin = Some(pulldown_pin.into_floating_input());
    }

    #[test]
    fn pullup_is_high(state: &mut State) {
        let puller_pin = unwrap!(state.puller_pin.take());

        let pullup_pin = puller_pin.into_pull_up_input();
        // GPIO re-configuration is not instantaneous so a delay is needed
        asm::delay(100);
        assert!(pullup_pin.is_high().unwrap());

        state.puller_pin = Some(pullup_pin.into_floating_input());
    }

    #[test]
    fn pullup_drives_high(state: &mut State) {
        let puller_pin = unwrap!(state.puller_pin.take());

        let pullup_pin = puller_pin.into_pull_up_input();
        // GPIO re-configuration is not instantaneous so a delay is needed
        asm::delay(100);
        assert!(state.input_pin.is_high().unwrap());

        state.puller_pin = Some(pullup_pin.into_floating_input());
    }
}
