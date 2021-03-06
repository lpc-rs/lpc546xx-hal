#![no_main]
#![no_std]

use cortex_m_rt::entry;

use defmt_rtt as _;
use lpc546xx_hal::{pac, prelude::*, syscon::Config};

use panic_probe as _;

#[entry]
fn main() -> ! {
    defmt::info!("serial example");
    let dp = pac::Peripherals::take().unwrap();
    let mut iocon = dp.IOCON;
    let mut syscon = dp.SYSCON.freeze(Config::fro12m());

    let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

    #[cfg(feature = "io-100")]
    let sda = gpio.pio0_26.into_open_drain_output();
    #[cfg(feature = "io-100")]
    let scl = gpio.pio0_27.into_open_drain_output();

    #[cfg(any(feature = "io-180", feature = "io-208"))]
    let sda = gpio.pio3_23.into_open_drain_output();
    #[cfg(any(feature = "io-180", feature = "io-208"))]
    let scl = gpio.pio3_24.into_open_drain_output();

    let mut i2c = dp
        .I2C2
        .i2c(dp.FLEXCOMM2, sda, scl, 100_000.Hz(), &mut syscon)
        .unwrap();

    let buffer = [0x0D; 1]; // WHO_AM_I register
    let mut content = [0u8; 1];
    const MMA8652FCR1_ADDR: u8 = 0b0011101;

    for addr in 0x00_u8..0x80 {
        // Write the empty array and check the slave response.
        if i2c.write_read(addr, &buffer, &mut content).is_ok() {
            defmt::println!("device present at {:02x}", addr);
        }
    }

    loop {
        i2c.write_read(MMA8652FCR1_ADDR, &buffer, &mut content)
            .unwrap();
        defmt::println!("WHO_AM_I: {:X}", content[0]);
        cortex_m::asm::delay(1_000_000);
    }
}
