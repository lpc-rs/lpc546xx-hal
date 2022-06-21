//! I2C Interface based on flexcomm
//!
//!

use crate::pac::{
    FLEXCOMM0, FLEXCOMM1, FLEXCOMM2, FLEXCOMM3, FLEXCOMM4, FLEXCOMM5, FLEXCOMM6, FLEXCOMM7,
    FLEXCOMM8, I2C0, I2C1, I2C2, I2C3, I2C4, I2C5, I2C6, I2C7, I2C8,
};
#[cfg(feature = "flexcomm-10")]
use crate::pac::{FLEXCOMM9, I2C9};

use crate::flexcomm::*;
use crate::gpio::gpio::*;
use crate::gpio::{AltMode::*, PinMode};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::syscon::{ClockControl, Syscon};
use core::marker::PhantomData;
use embedded_time::rate::Hertz;

/// I2C Error typess
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// bus is busy
    Busy,
    /// bus is not busy
    Idle,
    /// Not ACK
    Nak,
    /// Invalid parameter
    InvalidParameter,
    /// Bit Error (bit read was not equal to bit sent)
    BitError,
    /// Arbitration lost on the bus
    ArbitrationLost,
    /// No transfer in progress on the bus
    NoTransferInProgress,
    /// The Dma request failed
    DmaRequestFail,
    /// Start stop error
    StartStop,
    /// The peripheral is not in the correct state
    UnexpectedState,
    /// There was a timeout waiting for a response
    Timeout,
    /// No peripheral answered at this address
    AddrNak,
    /// There was an event timeout
    EventTimeout,
    /// the SCL line was stuck low
    SclLowTimeout,
}

/// Interrupt event
#[derive(Debug, PartialEq)]
pub enum Event {}

/// Transfer data word length
pub enum WordLength {}

/// Transfer Parity
pub enum Parity {}

/// Invalid configuration indicator
#[derive(Debug)]
pub enum InvalidConfig {
    /// This flexcomm in not capable of being used as an I2C peripheral
    NotI2cCapable,
}

/// Trait to mark serial pins with transmit capability.
pub trait SDAPin<I2C, FLEXCOMM> {
    /// setup SDA pin as I2C function
    fn setup(&self);
}

/// Trait to mark serial pins with receive capability.
pub trait SCLPin<I2C, FLEXCOMM> {
    /// setup SCL pin as I2C function
    fn setup(&self);
}

/// Macro to implement `SDAPin` / `SCLPin` for a certain pin, using a certain
/// alternative function and for a certain serial peripheral.
macro_rules! impl_pins {
    ($($pin:ident, $function:ident, $flexcomm_instance:ty, $I2C_instance:ty, $trait:ident;)*) => {
        $(
            impl<MODE: PinMode> $trait<$I2C_instance, $flexcomm_instance> for $pin<MODE> {
                fn setup(&self) {
                    self.set_alt_mode($function);
                }
            }
        )*
    }
}

#[cfg(any(feature = "io-100", feature = "io-180", feature = "io-208"))]
impl_pins!(
    P0_1, FUNC2, FLEXCOMM3, I2C3, SDAPin;
    P0_2, FUNC1, FLEXCOMM3, I2C3, SCLPin;
    P0_3, FUNC1, FLEXCOMM3, I2C3, SDAPin;
    P0_5, FUNC2, FLEXCOMM4, I2C4, SDAPin;
    P0_7, FUNC1, FLEXCOMM3, I2C3, SCLPin;
    P0_8, FUNC3, FLEXCOMM5, I2C5, SDAPin;
    P0_9, FUNC3, FLEXCOMM5, I2C5, SCLPin;
    P0_10, FUNC4, FLEXCOMM1, I2C1, SCLPin;
    P0_11, FUNC1, FLEXCOMM6, I2C6, SDAPin;
    P0_12, FUNC1, FLEXCOMM3, I2C3, SCLPin;
    P0_13, FUNC1, FLEXCOMM1, I2C1, SDAPin;
    P0_14, FUNC1, FLEXCOMM1, I2C1, SCLPin;
    P0_15, FUNC1, FLEXCOMM6, I2C6, SDAPin;
    P0_16, FUNC1, FLEXCOMM4, I2C4, SCLPin;
    P0_18, FUNC1, FLEXCOMM4, I2C4, SDAPin;
    P0_19, FUNC1, FLEXCOMM4, I2C4, SCLPin;
    P0_19, FUNC7, FLEXCOMM7, I2C7, SCLPin;
    P0_20, FUNC1, FLEXCOMM3, I2C3, SDAPin;
    P0_20, FUNC7, FLEXCOMM7, I2C7, SDAPin;
    P0_21, FUNC1, FLEXCOMM3, I2C3, SCLPin;
    P0_22, FUNC1, FLEXCOMM6, I2C6, SCLPin;
    P0_24, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P0_25, FUNC1, FLEXCOMM0, I2C0, SCLPin;
    P0_26, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P0_26, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P0_27, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P0_29, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P0_30, FUNC1, FLEXCOMM0, I2C0, SCLPin;
    P0_31, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P1_0, FUNC1, FLEXCOMM0, I2C0, SCLPin;
    P1_1, FUNC1, FLEXCOMM3, I2C3, SDAPin;
    P1_5, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P1_6, FUNC1, FLEXCOMM0, I2C0, SCLPin;
    P1_7, FUNC1, FLEXCOMM0, I2C0, SCLPin;
    P1_8, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P1_9, FUNC5, FLEXCOMM4, I2C4, SDAPin;
    P1_10, FUNC2, FLEXCOMM1, I2C1, SDAPin;
    P1_11, FUNC2, FLEXCOMM1, I2C1, SCLPin;
    P1_13, FUNC2, FLEXCOMM6, I2C6, SDAPin;
    P1_14, FUNC4, FLEXCOMM5, I2C5, SDAPin;
    P1_15, FUNC4, FLEXCOMM5, I2C5, SCLPin;
    P1_15, FUNC5, FLEXCOMM4, I2C4, SCLPin;
    P1_16, FUNC2, FLEXCOMM6, I2C6, SCLPin;
    P1_17, FUNC2, FLEXCOMM8, I2C8, SDAPin;
    P1_20, FUNC1, FLEXCOMM7, I2C7, SCLPin;
    P1_20, FUNC5, FLEXCOMM4, I2C4, SCLPin;
    P1_21, FUNC1, FLEXCOMM7, I2C7, SDAPin;
    P1_21, FUNC5, FLEXCOMM4, I2C4, SDAPin;
    P1_22, FUNC1, FLEXCOMM8, I2C8, SCLPin;
    P1_24, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P1_25, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P1_26, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P1_27, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P1_29, FUNC1, FLEXCOMM7, I2C7, SDAPin;
    P1_30, FUNC1, FLEXCOMM7, I2C7, SCLPin;
    P1_31, FUNC5, FLEXCOMM8, I2C8, SDAPin;
);
#[cfg(any(feature = "io-180", feature = "io-208"))]
impl_pins!(
    P2_0, FUNC2, FLEXCOMM0, I2C0, SDAPin;
    P2_1, FUNC2, FLEXCOMM0, I2C0, SCLPin;
    P2_3, FUNC3, FLEXCOMM1, I2C1, SDAPin;
    P2_4, FUNC3, FLEXCOMM1, I2C1, SCLPin;
    P2_5, FUNC3, FLEXCOMM1, I2C1, SDAPin;
    P2_6, FUNC3, FLEXCOMM1, I2C1, SCLPin;
    P2_12, FUNC5, FLEXCOMM5, I2C5, SDAPin;
    P2_13, FUNC5, FLEXCOMM5, I2C5, SCLPin;
    P2_14, FUNC5, FLEXCOMM5, I2C5, SDAPin;
    P2_15, FUNC5, FLEXCOMM5, I2C5, SCLPin;
    P2_17, FUNC5, FLEXCOMM8, I2C8, SDAPin;
    P2_18, FUNC2, FLEXCOMM3, I2C3, SDAPin;
    P2_18, FUNC2, FLEXCOMM8, I2C8, SCLPin;
    P2_19, FUNC2, FLEXCOMM3, I2C3, SCLPin;
    P2_19, FUNC3, FLEXCOMM7, I2C7, SDAPin;
    P2_20, FUNC2, FLEXCOMM3, I2C3, SCLPin;
    P2_20, FUNC3, FLEXCOMM7, I2C7, SCLPin;
    P2_21, FUNC2, FLEXCOMM3, I2C3, SDAPin;
    P2_24, FUNC3, FLEXCOMM2, I2C2, SCLPin;
    P2_28, FUNC2, FLEXCOMM7, I2C7, SDAPin;
    P2_29, FUNC2, FLEXCOMM7, I2C7, SCLPin;
    P2_29, FUNC3, FLEXCOMM8, I2C8, SCLPin;
    P3_2, FUNC2, FLEXCOMM9, I2C9, SDAPin;
    P3_3, FUNC2, FLEXCOMM9, I2C9, SCLPin;
    P3_4, FUNC3, FLEXCOMM8, I2C8, SDAPin;
    P3_5, FUNC3, FLEXCOMM8, I2C8, SCLPin;
    P3_13, FUNC2, FLEXCOMM9, I2C9, SDAPin;
    P3_14, FUNC2, FLEXCOMM9, I2C9, SCLPin;
    P3_16, FUNC1, FLEXCOMM8, I2C8, SDAPin;
    P3_17, FUNC1, FLEXCOMM8, I2C8, SCLPin;
    P3_18, FUNC1, FLEXCOMM8, I2C8, SDAPin;
    P3_19, FUNC1, FLEXCOMM8, I2C8, SCLPin;
    P3_21, FUNC1, FLEXCOMM9, I2C9, SDAPin;
    P3_22, FUNC1, FLEXCOMM9, I2C9, SCLPin;
    P3_23, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P3_24, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P3_26, FUNC3, FLEXCOMM4, I2C4, SDAPin;
    P3_27, FUNC3, FLEXCOMM4, I2C4, SCLPin;
    P3_28, FUNC3, FLEXCOMM4, I2C4, SDAPin;
    P3_29, FUNC3, FLEXCOMM4, I2C4, SCLPin;
    P3_30, FUNC1, FLEXCOMM9, I2C9, SDAPin;
    P3_31, FUNC1, FLEXCOMM9, I2C9, SCLPin;
    P4_0, FUNC2, FLEXCOMM6, I2C6, SDAPin;
    P4_2, FUNC2, FLEXCOMM6, I2C6, SDAPin;
    P4_3, FUNC2, FLEXCOMM6, I2C6, SCLPin;
    P4_4, FUNC3, FLEXCOMM0, I2C0, SCLPin;
    P4_5, FUNC2, FLEXCOMM9, I2C9, SDAPin;
    P4_5, FUNC3, FLEXCOMM0, I2C0, SDAPin;
    P4_6, FUNC2, FLEXCOMM9, I2C9, SCLPin;
    P4_9, FUNC2, FLEXCOMM2, I2C2, SDAPin;
    P4_10, FUNC2, FLEXCOMM2, I2C2, SCLPin;
    P4_11, FUNC2, FLEXCOMM2, I2C2, SDAPin;
    P4_12, FUNC2, FLEXCOMM2, I2C2, SCLPin;
    P4_15, FUNC3, FLEXCOMM9, I2C9, SDAPin;
    P4_16, FUNC3, FLEXCOMM9, I2C9, SCLPin;
);

#[cfg(any(feature = "io-208"))]
impl_pins!(
    P4_20, FUNC3, FLEXCOMM2, I2C2, SDAPin;
    P4_21, FUNC3, FLEXCOMM2, I2C2, SCLPin;
    P4_23, FUNC3, FLEXCOMM2, I2C2, SDAPin;
    P4_24, FUNC3, FLEXCOMM7, I2C7, SCLPin;
    P4_25, FUNC3, FLEXCOMM7, I2C7, SDAPin;
    P4_28, FUNC4, FLEXCOMM1, I2C1, SDAPin;
    P4_29, FUNC4, FLEXCOMM1, I2C1, SCLPin;
    P4_30, FUNC4, FLEXCOMM1, I2C1, SCLPin;
    P5_0, FUNC4, FLEXCOMM4, I2C4, SDAPin;
    P5_1, FUNC4, FLEXCOMM4, I2C4, SCLPin;
    P5_2, FUNC4, FLEXCOMM4, I2C4, SDAPin;
    P5_3, FUNC4, FLEXCOMM4, I2C4, SCLPin;
    P5_7, FUNC3, FLEXCOMM5, I2C5, SDAPin;
    P5_8, FUNC3, FLEXCOMM5, I2C5, SCLPin;
    P5_10, FUNC3, FLEXCOMM5, I2C5, SCLPin;
);

/// Data Pin
pub struct Sda<I2C> {
    _i2c: PhantomData<I2C>,
}

/// Clock Pin
pub struct Scl<I2C> {
    _i2c: PhantomData<I2C>,
}

/// I2C abstraction
pub struct I2c<I2C, FLEXCOMM> {
    i2c: I2C,
    flexcomm: FLEXCOMM,
    sda: Sda<I2C>,
    scl: Scl<I2C>,
}

macro_rules! impl_i2c {
    ($(
        $I2CX:ident: ($i2cX: ident, $FLEXCOMMX: ty, $I2cXExt: ident),
    )+) => {
        $(
/// I2C trait to make implicit declaration possible trough prelude
pub trait $I2cXExt<SDA, SCL> {
    /// Create a new I2Cx instance
    fn i2c(
        self,
        fc: $FLEXCOMMX,
        sda: SDA,
        scl: SCL,
        freq: Hertz,
        syscon: &mut Syscon,
    ) -> Result<I2c<$I2CX, $FLEXCOMMX>, InvalidConfig>;
}

impl<SDA, SCL> $I2cXExt<SDA, SCL> for $I2CX
where
    SDA: SDAPin<$I2CX, $FLEXCOMMX>,
    SCL: SCLPin<$I2CX, $FLEXCOMMX>,
{
    /// Configure I2Cx with pins
    fn i2c(
        self,
        fc: $FLEXCOMMX,
        sda: SDA,
        scl: SCL,
        freq: Hertz,
        syscon: &mut Syscon,
    ) -> Result<I2c<$I2CX, $FLEXCOMMX>, InvalidConfig> {
        I2c::$i2cX(self, fc, sda, scl, freq, syscon)
    }
}

impl I2c<$I2CX, $FLEXCOMMX> {
    /// Create new I2c instance
    pub fn $i2cX<SDA, SCL>(
        i2c: $I2CX,
        flexcomm: $FLEXCOMMX,
        sda: SDA,
        scl: SCL,
        freq: Hertz,
        syscon: &mut Syscon,
    ) -> Result<Self, InvalidConfig>
    where
        SDA: SDAPin<$I2CX, $FLEXCOMMX>,
        SCL: SCLPin<$I2CX, $FLEXCOMMX>,
        $FLEXCOMMX: FlexCommAsI2C<$I2CX> + ClockControl + FlexcommClockControl,
    {
        sda.setup();
        scl.setup();
        flexcomm.enable_clock(syscon);
        if !flexcomm.is_i2c_capable() {
            return Err(InvalidConfig::NotI2cCapable);
        }
        flexcomm.as_i2c();

        i2c.cfg.modify(|_, w| w.msten().enabled());

        let fcsource = FlexcommClockSource::fro_12_mhz;
        flexcomm.set_clock(fcsource, syscon);
        let source_clock = flexcomm.get_clock_freq(syscon).unwrap().0;

        let mut best_err = 0;
        let mut best_scl = 0;
        let mut best_div = 0;

        let mut mindivider = ((source_clock * 10) / 2000000 + 5) / 10;
        if (source_clock / mindivider / freq.0) < 4 {
            mindivider = source_clock / 4 / freq.0;
        }

        for divider in mindivider..0x10000 {
            // Calculte ideal scl value, round up the value
            let mut scl_div = ((source_clock * 10) / (divider * freq.0) + 5) / 10;

            /* adjust it if it is out of range */
            if scl_div > 18 {
                scl_div = 18;
            }

            /* calculate error */
            let err = source_clock - (freq.0 * scl_div * divider);
            if (err < best_err) || (best_err == 0) {
                best_div = divider;
                best_scl = scl_div;
                best_err = err;
            }

            if (err == 0) || (scl_div <= 4) {
                /* either exact value was found
                or scl is at its min (it would be even smaller in the next iteration for sure) */
                break;
            }
        }
        i2c.clkdiv
            .write(|w| unsafe { w.divval().bits(best_div as u16) });
        if best_scl % 2 == 0 {
            i2c.msttime.write(|w| {
                w.mstscllow()
                    .bits(((best_scl / 2) - 2) as u8)
                    .mstsclhigh()
                    .bits((((best_scl / 2) - 2) >> 4) as u8)
            });
        } else {
            i2c.msttime.write(|w| {
                w.mstscllow()
                    .bits(((best_scl / 2) - 1) as u8)
                    .mstsclhigh()
                    .bits((((best_scl / 2) - 2) >> 4) as u8)
            });
        }

        // set timeout  // 10ms
        let mut timeout_value = (10 * source_clock / 16 / 100 + 5) / 10;
        if timeout_value > 0x1000 {
            timeout_value = 0x1000;
        }
        timeout_value -= 1;
        i2c.timeout
            .write(|w| unsafe { w.to().bits(timeout_value as u16) });

        Ok(I2c {
            i2c,
            flexcomm,
            sda: Sda { _i2c: PhantomData },
            scl: Scl { _i2c: PhantomData },
        })
    }

    fn return_on_error(&mut self) -> Result<(), self::Error> {
        if self.i2c.stat.read().mststate().is_nack_data() {
            self.stop()?;
            return Err(Error::Nak);
        }
        if self.i2c.stat.read().mststate().is_nack_address() {
            self.stop()?;
            return Err(Error::AddrNak);
        }
        if self.i2c.stat.read().mstarbloss().is_arbitration_loss() {
            return Err(Error::ArbitrationLost);
        }
        if self.i2c.stat.read().mstststperr().is_error() {
            return Err(Error::StartStop);
        }
        Ok(())
    }
    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> Result<(), self::Error> {
        self.return_on_error()?;

        // Write the slave address with the RW bit set to 0 to the master data register MSTDAT.
        self.i2c
            .mstdat
            .modify(|_, w| unsafe { w.data().bits(addr << 1) });
        // Start the transmission by setting the MSTSTART bit to 1 in the master control register.
        self.i2c.mstctl.write(|w| w.mststart().start());
        // Wait for the pending status to be set (MSTPENDING = 1) by polling the STAT register
        // TODO: Consider implementing a timeout (loop at most N times...) :TODO
        while self.i2c.stat.read().mstpending().is_in_progress() {
            continue;
        }

        self.return_on_error()?;
        if !self.i2c.stat.read().mststate().is_transmit_ready() {
            return Err(Error::Busy);
        }

        // Send bytes
        for byte in bytes {
            // write a byte
            self.i2c
                .mstdat
                .modify(|_, w| unsafe { w.data().bits(*byte) });
            // instruct master to continue
            self.i2c.mstctl.write(|w| w.mstcontinue().continue_());
            // Wait until done
            while self.i2c.stat.read().mstpending().is_in_progress() {
                continue;
            }

            // Error handling
            self.return_on_error()?;
            if !self.i2c.stat.read().mststate().is_transmit_ready() {
                return Err(Error::Busy);
            }
        }

        // Fallthrough is success
        Ok(())
    }

    fn stop(&mut self) -> Result<(), self::Error> {
        // Stop the transmission by setting the MSTSTOP bit to 1 in the master control register.
        self.i2c.mstctl.write(|w| w.mststop().stop());
        // Wait for the pending status to be set (MSTPENDING = 1) by polling the STAT register
        while self.i2c.stat.read().mstpending().is_in_progress() {}

        self.return_on_error()?;
        if !self.i2c.stat.read().mststate().is_idle() {
            return Err(Error::Busy);
        }

        // Fallthrough is success
        Ok(())
    }
    /// Release the borrowed I2C and FLEXCOMM peripherals, and both pins
    pub fn release(self) -> ($I2CX, $FLEXCOMMX, Sda<$I2CX>, Scl<$I2CX>) {
        (self.i2c, self.flexcomm, self.sda, self.scl)
    }
}

impl Write for I2c<$I2CX, $FLEXCOMMX> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.stop()
    }
}

impl Read for I2c<$I2CX, $FLEXCOMMX> {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), self::Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // Write the slave address with the RW bit set to 1 to the master data register MSTDAT.
            self.i2c
                .mstdat
                .modify(|_, w| unsafe { w.data().bits((addr << 1) | 1) });
            // Start the transmission by setting the MSTSTART bit to 1 in the master control register.
            self.i2c.mstctl.write(|w| w.mststart().start());

            // Wait for the pending status to be set (MSTPENDING = 1) by polling the STAT register
            while self.i2c.stat.read().mstpending().is_in_progress() {}

            self.return_on_error()?;
            if !self.i2c.stat.read().mststate().is_receive_ready() {
                return Err(Error::Busy);
            }

            for byte in buffer {
                // Read a byte
                *byte = self.i2c.mstdat.read().data().bits();
                // Instruct master to continue
                self.i2c.mstctl.write(|w| w.mstcontinue().continue_());

                // Wait for next byte
                while self.i2c.stat.read().mstpending().is_in_progress() {}

                self.return_on_error()?;
                if !self.i2c.stat.read().mststate().is_receive_ready() {
                    return Err(Error::Busy);
                }
            }
            // Read last byte
            *last = self.i2c.mstdat.read().data().bits();

            self.stop()?;
        }
        // Fallthrough is success
        Ok(())
    }
}
impl WriteRead for I2c<$I2CX, $FLEXCOMMX> {
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.read(addr, buffer)?;
        Ok(())
    }
}
        )*
    }

}

impl_i2c! {
    I2C0: (i2c0, FLEXCOMM0, I2c0Ext),
    I2C1: (i2c1, FLEXCOMM1, I2c1Ext),
    I2C2: (i2c2, FLEXCOMM2, I2c2Ext),
    I2C3: (i2c3, FLEXCOMM3, I2c3Ext),
    I2C4: (i2c4, FLEXCOMM4, I2c4Ext),
    I2C5: (i2c5, FLEXCOMM5, I2c5Ext),
    I2C6: (i2c6, FLEXCOMM6, I2c6Ext),
    I2C7: (i2c7, FLEXCOMM7, I2c7Ext),
    I2C8: (i2c8, FLEXCOMM8, I2c8Ext),
}

#[cfg(feature = "flexcomm-10")]
impl_i2c! {
    I2C9: (i2c9, FLEXCOMM9, I2c9Ext),
}
