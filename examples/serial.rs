#![no_main]
#![no_std]

use cortex_m_rt::entry;

use core::fmt::Write;
use defmt_rtt as _;
use lpc546xx_hal::{pac, prelude::*, serial, syscon::Config};
use nb::block;
use panic_probe as _;
#[entry]
fn main() -> ! {
    defmt::info!("serial example");
    let dp = pac::Peripherals::take().unwrap();
    let mut iocon = dp.IOCON;
    let mut syscon = dp.SYSCON.freeze(Config::fro12m());

    let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

    let rx_pin = gpio.pio0_29;
    let tx_pin = gpio.pio0_30;

    let conf = serial::Config {
        baudrate: 115200.Bd(),
        wordlength: serial::WordLength::DataBits8,
        parity: serial::Parity::ParityNone,
        stopbits: serial::StopBits::STOP1,
        loopback: serial::Loopback::Normal,
    };
    let serial = dp
        .USART0
        .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
        .unwrap();

    let (mut tx, mut rx) = serial.split();

    writeln!(
        tx,
        "hello from lpc! this example will echo any char received"
    )
    .unwrap();
    loop {
        match block!(rx.read()) {
            Ok(x) => {
                block!(tx.write(x)).ok();
                defmt::info!("received: {}", x);
            }
            Err(_) => defmt::error!("error"),
        }
    }
}
