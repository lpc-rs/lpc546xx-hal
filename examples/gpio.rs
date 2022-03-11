#![no_main]
#![no_std]

use cortex_m_rt::entry;

use defmt_rtt as _;
use lpc546xx_hal::{pac, prelude::*, syscon::Config};
use panic_probe as _;
#[entry]
fn main() -> ! {
    defmt::info!("gpio example");
    let dp = pac::Peripherals::take().unwrap();
    let mut iocon = dp.IOCON;
    let mut syscon = dp.SYSCON.freeze(Config::fro12m());

    let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

    #[cfg(feature = "io-100")]
    let mut led1 = gpio.pio1_19.into_push_pull_output();
    #[cfg(any(feature = "io-180", feature = "io-208"))]
    let mut led1 = gpio.pio3_14.into_push_pull_output();
    // pio3_14 is LED1 on devkit

    loop {
        led1.toggle().unwrap();

        cortex_m::asm::delay(1_000_000);
    }
}
