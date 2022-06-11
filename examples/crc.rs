#![no_main]
#![no_std]

use cortex_m_rt::entry;

use defmt_rtt as _;
use lpc546xx_hal::{crc, pac, prelude::*, syscon::Config};
use panic_probe as _;
#[entry]
fn main() -> ! {
    defmt::info!("gpio example");
    let dp = pac::Peripherals::take().unwrap();
    let mut syscon = dp.SYSCON.freeze(Config::fro12m());

    // this examples implements a CRC-CCITT example over a simple buffer of a hundred elements, and asserts its result
    let buffer = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
    let check = 0xC241;

    let mut crc = dp
        .CRC_ENGINE
        .crc(crc::Config::default(), &mut syscon)
        .unwrap();

    crc.feed(&buffer);

    // print crc result
    defmt::println!(
        "computed crc: {:X}, expected crc : {:X}",
        crc.peek_result(),
        check
    );
    loop {
        cortex_m::asm::delay(10000000);
    }
}
