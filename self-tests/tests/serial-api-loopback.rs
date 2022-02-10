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

    #[test]
    fn config_8n1_256000_fc0() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_29;
        let tx_pin = gpio.pio0_30;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
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
    fn config_8n1_256000_fc1() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio1_10;
        let tx_pin = gpio.pio1_11;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART1
            .usart(dp.FLEXCOMM1, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }

    #[test]
    fn config_8n1_256000_fc2() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_26;
        let tx_pin = gpio.pio0_27;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART2
            .usart(dp.FLEXCOMM2, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }

    #[test]
    fn config_8n1_256000_fc3() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_3;
        let tx_pin = gpio.pio0_2;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART3
            .usart(dp.FLEXCOMM3, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_8n1_256000_fc4() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio1_21;
        let tx_pin = gpio.pio1_20;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART4
            .usart(dp.FLEXCOMM4, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_8n1_256000_fc5() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_8;
        let tx_pin = gpio.pio0_9;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART5
            .usart(dp.FLEXCOMM5, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }

    #[test]
    fn config_8n1_256000_fc6() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio1_13;
        let tx_pin = gpio.pio1_16;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART6
            .usart(dp.FLEXCOMM6, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_8n1_256000_fc7() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio0_20;
        let tx_pin = gpio.pio0_19;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART7
            .usart(dp.FLEXCOMM7, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    fn config_8n1_256000_fc8() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio1_17;
        let tx_pin = gpio.pio1_18;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART8
            .usart(dp.FLEXCOMM8, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
    #[test]
    #[cfg(feature = "local-flexcomm-10")]
    fn config_8n1_256000_fc9() {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let mut iocon = dp.IOCON;
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let rx_pin = gpio.pio4_15;
        let tx_pin = gpio.pio4_16;

        let conf = serial::Config {
            baudrate: 256000.Bd(),
            wordlength: serial::WordLength::DataBits8,
            parity: serial::Parity::ParityNone,
            stopbits: serial::StopBits::STOP1,
            loopback: serial::Loopback::Loopback,
        };
        let serial = dp
            .USART9
            .usart(dp.FLEXCOMM9, tx_pin, rx_pin, conf, &mut syscon)
            .unwrap();

        let (mut tx, mut rx) = serial.split();
        for i in 0..255 {
            block!(tx.write(i)).ok();
            let received = block!(rx.read()).unwrap();
            defmt::assert_eq!(i, received);
        }
    }
}
