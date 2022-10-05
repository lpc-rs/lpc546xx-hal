#![no_std]
#![no_main]

use defmt_rtt as _; // transport layer for defmt logs
use lpc546xx_hal::{
    self as _,
    i2c::I2c,
    pac::{FLEXCOMM2, I2C2},
};
use panic_probe as _; // panicking behavior

struct State {
    // state shared between `#[test]` functions
    i2c: Option<I2c<I2C2, FLEXCOMM2>>,
}

#[defmt_test::tests]
mod tests {
    use super::State;
    use defmt::assert;
    use lpc546xx_hal::{pac, prelude::*, syscon::Config}; // the HAL we'll test
    #[init]
    fn init() -> State {
        let dp = pac::Peripherals::take().unwrap();
        let mut iocon = dp.IOCON;
        let mut syscon = dp.SYSCON.freeze(Config::fro12m());
        let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

        let sda = gpio.pio3_23.into_open_drain_output();
        let scl = gpio.pio3_24.into_open_drain_output();

        let i2c = dp
            .I2C2
            .i2c(dp.FLEXCOMM2, sda, scl, 100_000.Hz(), &mut syscon)
            .unwrap();

        State { i2c: Some(i2c) }
    }

    #[test]
    fn who_am_i_write_read_single(state: &mut State) {
        let mut i2c = state.i2c.take().unwrap();
        const MMA8652FCR1_ADDR: u8 = 0b0011101;
        const WHO_AM_I: u8 = 0x0D;
        const DEVICE_ID: u8 = 0x4A;
        let buffer = [WHO_AM_I; 1];
        let mut content = [0u8; 1];
        i2c.write_read(MMA8652FCR1_ADDR, &buffer, &mut content)
            .unwrap();
        assert!(content[0] == DEVICE_ID);
        state.i2c = Some(i2c);
    }

    #[test]
    fn who_am_i_write_read_multiple(state: &mut State) {
        let mut i2c = state.i2c.take().unwrap();
        const MMA8652FCR1_ADDR: u8 = 0b0011101;
        const WHO_AM_I: u8 = 0x0D;
        const DEVICE_ID: u8 = 0x4A;
        let buffer = [0x0A; 1]; // start reading at address 0, until WHO_AM_I reg
        let mut content = [0u8; (WHO_AM_I + 1 - 0x0A) as usize];
        i2c.write_read(MMA8652FCR1_ADDR, &buffer, &mut content)
            .unwrap();
        assert!(content[(WHO_AM_I - 0x0A) as usize] == DEVICE_ID);
        state.i2c = Some(i2c);
    }
    #[test]
    fn who_am_i_read_cirrus_logic(state: &mut State) {
        let id_doc: u16 = 0b1000_1001_0000_0100;
        let mut i2c = state.i2c.take().unwrap();
        const WM8904CGEFL_ADDR: u8 = 0b0011010;
        let buffer = [0x00; 1]; // start reading at address 0, until WHO_AM_I reg
        let mut content = [0u8; 2];
        i2c.write_read(WM8904CGEFL_ADDR, &buffer, &mut content)
            .unwrap();
        let id_read: u16 = content[1] as u16 | (content[0] as u16) << 8;
        assert!(id_read == id_doc);
        state.i2c = Some(i2c);
    }

    #[test]
    fn cirrus_single_write_single_read(state: &mut State) {
        let mut i2c = state.i2c.take().unwrap();
        const WM8904CGEFL_ADDR: u8 = 0b0011010;
        let id_doc: u16 = 0b1000_1001_0000_0100;
        let buffer = [0x00; 1];
        let mut content = [0u8; 2];
        i2c.write(WM8904CGEFL_ADDR, &buffer).unwrap();
        i2c.read(WM8904CGEFL_ADDR, &mut content).unwrap();
        let id_read: u16 = content[1] as u16 | (content[0] as u16) << 8;
        assert!(id_read == id_doc);
        state.i2c = Some(i2c);
    }

    /// this test reads the register WHO_AM_I of the MMA8652FCR1 and compares it to the expected value
    #[test]
    fn mma8652_who_am_i_read(state: &mut State) {
        let mut i2c = state.i2c.take().unwrap();
        const MMA8652FCR1_ADDR: u8 = 0b0011101;
        const WHO_AM_I: u8 = 0x0D;
        const DEVICE_ID: u8 = 0x4A;
        let buffer = [WHO_AM_I; 1];
        let mut content = [0u8; 1];
        i2c.write_read(MMA8652FCR1_ADDR, &buffer, &mut content)
            .unwrap();
        assert!(content[0] == DEVICE_ID);
        state.i2c = Some(i2c);
    }
}
