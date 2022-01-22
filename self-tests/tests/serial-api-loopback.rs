#![deny(warnings)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use lpc546xx_hal::{self as _};
#[defmt_test::tests]
mod tests {
    //use defmt::assert;
    use lpc546xx_hal::{pac, prelude::*, serial, syscon::Config}; // the HAL we'll test
    use nb::block;

    #[test]
    fn config_8n1_115200() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 115200.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART0
            .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_8n1_9600() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 9600.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART0
            .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_8n2_115200() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 115200.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART0
            .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_7n2_256000() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits7,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART0
            .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i & 0x7F, received);
        }
    }

    #[test]
    fn config_7o2_256000() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits7,
            parity: serial::Parity::ParityOdd,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART0
            .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i & 0x7F, received);
        }
    }

    #[test]
    fn config_7e2_256000() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits7,
            parity: serial::Parity::ParityEven,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART0
            .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i & 0x7F, received);
        }
    }
}
