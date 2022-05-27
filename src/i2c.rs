//! I2C Interface based on flexcomm
//!
//!

use crate::pac::{
    FLEXCOMM0, FLEXCOMM1, FLEXCOMM2, FLEXCOMM3, FLEXCOMM4, FLEXCOMM5, FLEXCOMM6, FLEXCOMM7,
    FLEXCOMM8, I2C0, I2C1, I2C2, I2C3, I2C4, I2C5, I2C6, I2C7, I2C8,
};
#[cfg(feature = "flexcomm-10")]
use crate::pac::{FLEXCOMM9, I2C9};
use core::fmt;
use core::marker::PhantomData;

use crate::hal;
use crate::hal::prelude::*;

use crate::flexcomm::*;
use crate::gpio::gpio::*;
use crate::gpio::{AltMode::*, PinMode};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::syscon::{ClockControl, Syscon};
use embedded_time::rate::Hertz;
use nb::block;

pub enum Direction {
    Write,
    Read,
}
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Busy,
    Idle,
    Nak,
    InvalidParameter,
    BitError,
    ArbitrationLost,
    NoTransferInProgress,
    DmaRequestFail,
    StartStop,
    UnexpectedState,
    Timeout,
    AddrNak,
    EventTimeout,
    SclLowTimeout,
}

/// Interrupt event
#[derive(Debug, PartialEq)]
pub enum Event {}

pub enum I2CStatusFlags {
    MasterPending,
    MasterArbitrationLost,
    MasterStartStopError,
    MasterIdle,
    MasterRxReady,
    MasterTxReady,
    MasterAddrNack,
    MasterDataNack,
    SlavePendingFlag,
    SlaveNotStreching,
    SlaveSelected,
    SlaveDeselected,
    SlaveAddressed,
    SlaveReceive,
    SlaveTransmit,
}
/// Transfer data word length
pub enum WordLength {}

/// Transfer Parity
pub enum Parity {}

/// Invalid configuration indicator
#[derive(Debug)]
pub enum InvalidConfig{
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

/// Macro to implement `TxPin` / `RxPin` for a certain pin, using a certain
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
    //P0_13, FUNC1, FLEXCOMM1, I2C1, SDAPin;
    P0_18, FUNC1, FLEXCOMM4, I2C4, SDAPin;
    P0_20, FUNC1, FLEXCOMM3, I2C3, SDAPin;
    P0_21, FUNC1, FLEXCOMM3, I2C3, SCLPin;
    P0_24, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P0_26, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P0_26, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P0_29, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P0_3, FUNC1, FLEXCOMM3, I2C3, SDAPin;
    P1_1, FUNC1, FLEXCOMM3, I2C3, SDAPin;
    P1_21, FUNC1, FLEXCOMM7, I2C7, SDAPin;
    P1_24, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P1_26, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P1_29, FUNC1, FLEXCOMM7, I2C7, SDAPin;
    P1_5, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P1_8, FUNC1, FLEXCOMM0, I2C0, SDAPin;
    P3_16, FUNC1, FLEXCOMM8, I2C8, SDAPin;
    P3_18, FUNC1, FLEXCOMM8, I2C8, SDAPin;
    P3_23, FUNC1, FLEXCOMM2, I2C2, SDAPin;
    P3_24, FUNC1, FLEXCOMM2, I2C2, SCLPin;
    P3_30, FUNC1, FLEXCOMM9, I2C9, SDAPin;
    // Missing Type I P0_13, P3_2
);

/// Serial receiver
pub struct Sda<I2C> {
    _i2c: PhantomData<I2C>,
}

/// Serial transmitter
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

macro_rules! impl_I2C {
    (
        $I2CX:ident: ($i2cX: ident, $FLEXCOMMX: ty, $I2CExt: ident),
    ) => {
        }
    }
/// Serial trait to make implicit declaration possible trough prelude
pub trait I2C2Ext<SDA, SCL> {
    fn i2c(
        self,
        fc: FLEXCOMM2,
        sda: SDA,
        scl: SCL,
        freq: Hertz,
        syscon: &mut Syscon,
    ) -> Result<I2c<I2C2, FLEXCOMM2>, InvalidConfig>;
}

impl<SDA, SCL> I2C2Ext<SDA, SCL> for I2C2
where
    SDA: SDAPin<I2C2, FLEXCOMM2>,
    SCL: SCLPin<I2C2, FLEXCOMM2>,
{
    fn i2c(
        self,
        fc: FLEXCOMM2,
        sda: SDA,
        scl: SCL,
        freq: Hertz,
        syscon: &mut Syscon,
    ) -> Result<I2c<I2C2, FLEXCOMM2>, InvalidConfig> {
        I2c::i2c2(self, fc, sda, scl, freq, syscon)
    }
}

impl I2c<I2C2, FLEXCOMM2> {
    pub fn i2c2<SDA, SCL>(
        i2c: I2C2,
        flexcomm: FLEXCOMM2,
        sda: SDA,
        scl: SCL,
        freq: Hertz,
        syscon: &mut Syscon,
    ) -> Result<Self, InvalidConfig>
    where
        SDA: SDAPin<I2C2, FLEXCOMM2>,
        SCL: SCLPin<I2C2, FLEXCOMM2>,
        FLEXCOMM2: FlexCommAsI2C<I2C2> + ClockControl + FlexcommClockControl,
    {
        sda.setup();
        scl.setup();
        flexcomm.enable_clock(syscon);
        if !flexcomm.is_i2c_capable(){
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
        timeout_value = timeout_value - 1;
        i2c.timeout
            .write(|w| unsafe { w.to().bits(timeout_value as u16) });

        Ok(I2c {
            i2c,
            flexcomm,
            sda: Sda { _i2c: PhantomData },
            scl: Scl { _i2c: PhantomData },
        })
    }

    fn return_on_error(&self) -> Result<(), self::Error> {
        if self.i2c.stat.read().mststate().is_nack_data() {
            return Err(Error::Nak);
        }
        if self.i2c.stat.read().mststate().is_nack_address() {
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
            // dbg!(Error::Bus);
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
                // dbg!(Error::Bus);
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
            // dbg!(Error::Bus);
            return Err(Error::Busy);
        }

        // Fallthrough is success
        Ok(())
    }
   
    pub fn release(self) -> (I2C2, FLEXCOMM2) {
        (self.i2c, self.flexcomm)
    }
}

impl Write for I2c<I2C2, FLEXCOMM2> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.stop()
    }
}

impl Read for I2c<I2C2, FLEXCOMM2> {
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
impl WriteRead for I2c<I2C2, FLEXCOMM2> {
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.read(addr, buffer)?;

        Ok(())
    }
}


// /// Macro to implement all I2C
// macro_rules! impl_I2C {
//     ($(
//         $I2CX:ident: ($i2cX: ident, $FLEXCOMMX: ty, $I2CExt: ident),
//     )+) => {
//         $(

// impl<SDA, SCL> $I2CExt<SDA, SCL> for $I2CX
// where
//     SDA: SDAPin<$I2CX, $FLEXCOMMX>,
//     SCL: SCLPin<$I2CX, $FLEXCOMMX>,
// {
//     /// Configure I2C peripheral according to `config`
//     fn i2c(
//         self,
//         fc: $FLEXCOMMX,
//         sda: SDA,
//         scl: SCL,
//         freq: Hertz,
//         syscon: &mut Syscon,
//     ) -> Result<I2c<$I2CX, $FLEXCOMMX>, InvalidConfig> {
//         I2c::$i2cX(self, fc, sda, scl, freq, syscon)
//     }
// }

// impl I2c<$I2CX, $FLEXCOMMX> {
//     /// Configure I2C peripheral according to `config`
//     pub fn $i2cX<SDA, SCL>(
//         i2c: $I2CX,
//         flexcomm: $FLEXCOMMX,
//         sda: SDA,
//         scl: SCL,
//         freq: Hertz,
//         syscon: &mut Syscon,
//     ) -> Result<Self, InvalidConfig>
//     where
//         SDA: SDAPin<$I2CX, $FLEXCOMMX>,
//         SCL: SCLPin<$I2CX, $FLEXCOMMX>,
//         $FLEXCOMMX: FlexCommAsI2C<$I2CX> + ClockControl + FlexcommClockControl,
//     {
//         sda.setup();
//         scl.setup();
//         flexcomm.enable_clock(syscon);
//         flexcomm.as_i2c();

//         i2c.cfg.modify(|_,w| w.msten().enabled() );

//         let fcsource = FlexcommClockSource::fro_12_mhz;
//         flexcomm.set_clock(fcsource, syscon);
//         let source_clock = flexcomm.get_clock_freq(syscon).unwrap().0;

//         let mut best_err = 0;
//         let mut best_scl = 0;
//         let mut best_div = 0;

//         let mut mindivider = ((source_clock * 10) / 2000000 + 5) / 10;
//         if ((source_clock / mindivider / freq.0) < 4)
//         {
//             mindivider = source_clock / 4 / freq.0;
//         }

//         for divider in mindivider..0x10000 {
//             // Calculte ideal scl value, round up the value
//             let mut scl_div = ((source_clock * 10) / (divider * freq.0) + 5) / 10;

//             /* adjust it if it is out of range */
//             if scl_div > 18 {
//                 scl_div = 18;
//             }

//             /* calculate error */
//             let err = source_clock - (freq.0 * scl_div * divider);
//             if (err < best_err) || (best_err == 0)
//             {
//                 best_div = divider;
//                 best_scl = scl_div;
//                 best_err = err;
//             }

//             if ((err == 0) || (scl_div <= 4))
//             {
//                 /* either exact value was found
//                    or scl is at its min (it would be even smaller in the next iteration for sure) */
//                 break;
//             }
//         }
//         i2c.clkdiv.write(|w| unsafe{w.divval().bits(best_div as u16)});
//         if best_scl % 2 == 0 {
//             i2c.msttime.write(|w| w.mstscllow().bits(((best_scl/2) - 2) as u8).mstsclhigh().bits((((best_scl/2) - 2) >> 4) as u8));
//         }
//         else
//         {
//             i2c.msttime.write(|w| w.mstscllow().bits(((best_scl/2) - 1) as u8).mstsclhigh().bits((((best_scl/2) - 2) >> 4) as u8));
//         }

//         // set timeout  // 10ms
//         let mut timeout_value = (10 * source_clock / 16 / 100 + 5) / 10;
//         if timeout_value > 0x1000 {
//             timeout_value = 0x1000;
//         }
//         timeout_value = (timeout_value -1);
//         i2c.timeout.write(|w| unsafe{w.to().bits(timeout_value as u16)});

//         Ok(I2c {
//             i2c,
//             flexcomm,
//             sda: Sda{ _i2c: PhantomData},
//             scl: Scl{ _i2c: PhantomData},
//         })
//     }
//     /// Release the UART for new uses
//     pub fn release(self) -> ($I2CX, $FLEXCOMMX) {
//         (self.i2c, self.flexcomm)
//     }
// })*}}

// impl_I2C! {
//     I2C0: (i2c0, FLEXCOMM0, I2C0Ext),
//     I2C1: (i2c1, FLEXCOMM1, I2C1Ext),
//     I2C2: (i2c2, FLEXCOMM2, I2C2Ext),
//     I2C3: (i2c3, FLEXCOMM3, I2C3Ext),
//     I2C4: (i2c4, FLEXCOMM4, I2C4Ext),
//     I2C5: (i2c5, FLEXCOMM5, I2C5Ext),
//     I2C6: (i2c6, FLEXCOMM6, I2C6Ext),
//     I2C7: (i2c7, FLEXCOMM7, I2C7Ext),
//     I2C8: (i2c8, FLEXCOMM8, I2C8Ext),
// }

// #[cfg(feature = "flexcomm-10")]
// impl_I2C! {
//     I2C9: (i2c9, FLEXCOMM9, I2C9Ext),
// }

// impl<SDA, SCL> WriteRead for I2c<SDA, SCL> {
//     type Error = Error;

//     fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
//         let writing = !bytes.is_empty();
//         let reading = !buffer.is_empty();

//         // wait for i2c device to be available
//         while self.i2c.stat.read().mststate().is_idle() {
//             self.check_errors()?;
//         }
//         /*while self.i2c.isr.read().busy().is_busy() {
//             self.check_errors()?;
//         }*/
//         // if we are writing bytes
//         if writing {
//             // if the previous write has failed, we need to flush the TX
//             // buffer to prevent sending old data
//             self.i2c.isr.write(|w| w.txe().set_bit());

//             if reading {
//                 self.start_transfer(addr, bytes.len(), RD_WRN_A::WRITE, AUTOEND_A::SOFTWARE);
//             } else {
//                 self.start_transfer(addr, bytes.len(), RD_WRN_A::WRITE, AUTOEND_A::AUTOMATIC);
//             }

//             // Send bytes
//             for c in bytes {
//                 self.send_byte(*c)?;
//             }

//             // if we are going to read afterwards, we need to wait for
//             // the tx to complete
//             if reading {
//                 while self.i2c.isr.read().tc().is_not_complete() {
//                     self.check_errors()?;
//                 }
//             }
//         }

//         if reading {
//             // force a read of the rx data register, so that we dont
//             // get stale data from the last transaction (if there is
//             // anything left over)
//             self.i2c.rxdr.read();

//             //send a new start condition and transfer
//             self.start_transfer(addr, buffer.len(), RD_WRN_A::READ, AUTOEND_A::AUTOMATIC);

//             // Receive bytes into buffer
//             for c in buffer {
//                 *c = self.recv_byte()?;
//             }
//         }

//         Ok(())
//     }
// }

// impl<SDA, SCL> Write for I2c<SDA, SCL> {
//     type Error = Error;

//     fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
//         self.write_read(addr, bytes, &mut [])
//     }
// }

// impl<SDA, SCL> Read for I2c<SDA, SCL> {
//     type Error = Error;

//     fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
//         self.write_read(addr, &[], buffer)
//     }
// }
