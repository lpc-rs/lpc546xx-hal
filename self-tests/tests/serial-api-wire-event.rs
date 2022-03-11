// Required connections:
//
// - P3.26 <-> P3.27

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

        let rx_pin = gpio.pio3_26;
        let tx_pin = gpio.pio3_27;

        let conf = serial::Config {
            baudrate: 115200.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Normal,
        };
        let mut serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        serial.listen(serial::Event::RxLvl);
        serial.set_rx_threshold(1);

        for i in 0..255 {
            serial.write(i).unwrap();
            let event = block!(serial.wait_for_pending_event()).unwrap();
            defmt::assert!(event == serial::Event::RxLvl);
            let v = serial.read().unwrap();
            defmt::assert_eq!(v, i);
        }
    }
    #[test]
    fn config_8n1_9600() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio3_26;
        let tx_pin = gpio.pio3_27;

        let conf = serial::Config {
            baudrate: 9600.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Normal,
        };
        let mut serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        serial.listen(serial::Event::RxLvl);
        serial.set_rx_threshold(1);

        for i in 0..255 {
            serial.write(i).unwrap();
            let event = block!(serial.wait_for_pending_event()).unwrap();
            defmt::assert!(event == serial::Event::RxLvl);
            let v = serial.read().unwrap();
            defmt::assert_eq!(v, i);
        }
    }
    #[test]
    fn config_8n2_115200() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio3_26;
        let tx_pin = gpio.pio3_27;

        let conf = serial::Config {
            baudrate: 115200.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Normal,
        };
        let mut serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();
        serial.listen(serial::Event::RxLvl);
        serial.set_rx_threshold(1);

        for i in 0..255 {
            serial.write(i).unwrap();
            let event = block!(serial.wait_for_pending_event()).unwrap();
            defmt::assert!(event == serial::Event::RxLvl);
            let v = serial.read().unwrap();
            defmt::assert_eq!(v, i);
        }
    }
    #[test]
    fn config_7n2_256000() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio3_26;
        let tx_pin = gpio.pio3_27;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits7,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Normal,
        };
        let mut serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        serial.listen(serial::Event::RxLvl);
        serial.set_rx_threshold(1);

        for i in 0..255 {
            serial.write(i).unwrap();
            let event = block!(serial.wait_for_pending_event()).unwrap();
            defmt::assert!(event == serial::Event::RxLvl);
            let v = serial.read().unwrap();
            defmt::assert_eq!(v, i & 0x7F);
        }
    }

    #[test]
    fn config_7o2_256000() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio3_26;
        let tx_pin = gpio.pio3_27;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits7,
            parity: serial::Parity::ParityOdd,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Normal,
        };
        let mut serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        serial.listen(serial::Event::RxLvl);
        serial.set_rx_threshold(1);

        for i in 0..255 {
            serial.write(i).unwrap();
            let event = block!(serial.wait_for_pending_event()).unwrap();
            defmt::assert!(event == serial::Event::RxLvl);
            let v = serial.read().unwrap();
            defmt::assert_eq!(v, i & 0x7F);
        }
    }
    #[test]
    fn config_7e2_256000() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio3_26;
        let tx_pin = gpio.pio3_27;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits7,
            parity: serial::Parity::ParityEven,
            stopbits: serial::StopBits::STOP2,
            loopback: serial::Loopback::Normal,
        };
        let mut serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        serial.listen(serial::Event::RxLvl);
        serial.set_rx_threshold(1);

        for i in 0..255 {
            serial.write(i).unwrap();
            let event = block!(serial.wait_for_pending_event()).unwrap();
            defmt::assert!(event == serial::Event::RxLvl);
            let v = serial.read().unwrap();
            defmt::assert_eq!(v, i & 0x7F);
        }
    }
}
