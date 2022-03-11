//! Serial (UART) Interface based on flexcomm
//!
//!

#![deny(missing_docs)]

use crate::pac::{
    FLEXCOMM0, FLEXCOMM1, FLEXCOMM2, FLEXCOMM3, FLEXCOMM4, FLEXCOMM5, FLEXCOMM6, FLEXCOMM7,
    FLEXCOMM8, USART0, USART1, USART2, USART3, USART4, USART5, USART6, USART7, USART8,
};
#[cfg(feature = "flexcomm-10")]
use crate::pac::{FLEXCOMM9, USART9};
use core::fmt;
use core::marker::PhantomData;

use crate::hal;
use crate::hal::prelude::*;

use crate::flexcomm::{FlexCommAsUart, FlexcommClockControl, FlexcommClockSource};
use crate::gpio::gpio::*;
use crate::gpio::{AltMode::*, PinMode};
use crate::syscon::{ClockControl, Syscon};
use embedded_time::rate::{Baud, Extensions};
use nb::block;
/// Serial error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    /// AutoBaud Error
    AutoBaud,
    /// Tx Fifo Error
    TxFifo,
    /// Rx Fifo Error
    RxFifo,
}

/// Interrupt event
#[derive(Debug, PartialEq)]
pub enum Event {
    /// Determines whether an interrupt occurs when a transmit error occurs,
    /// based on the TXERR flag in the FIFOSTAT register.
    /// This event is cleared by writing a 1 to FIFOSTAT.TXERR
    TxErr,
    /// Determines whether an interrupt occurs when a receive error occurs,
    /// based on the RXERR flag in the FIFOSTAT register.
    /// This Event is cleared by writing a 1 to FIFOSTAT.RXERR
    RxErr,
    /// Determines whether an interrupt occurs when a the transmit FIFO
    /// reaches the level specified by the TXLVL field in the FIFOTRIG register.
    /// Unclear how to clear this flag, I suppose it is cleared when writing a
    /// new word to FIFOWR
    TxLvl,
    /// Determines whether an interrupt occurs when a the receive FIFO reaches
    /// the level specified by the RXLVL field in the FIFOTRIG register.
    /// Unclear how to clear this flag, I suppose it is cleared when reading a
    /// new word from FIFORD
    RxLvl,
}

/// Transfer data word length
pub enum WordLength {
    /// 7 bit word length
    DataBits7,
    /// 8 bit word length
    DataBits8,
    /// 9 bit word length
    DataBits9,
}

/// Transfer Parity
pub enum Parity {
    /// No parity
    ParityNone,
    /// Even parity
    ParityEven,
    /// Odd parity
    ParityOdd,
}

/// Loopback feature
pub enum Loopback {
    /// Normal mode
    Normal,
    /// Local loopback mode
    Loopback,
}

/// Transfer stop bit
pub enum StopBits {
    /// 1 stop bit
    STOP1,
    /// 2 stop bit
    STOP2,
}

/// USART Configuration template
pub struct Config {
    /// USART baudrate
    pub baudrate: Baud,
    /// USART word length
    pub wordlength: WordLength,
    /// USART parity
    pub parity: Parity,
    /// USART stop bits
    pub stopbits: StopBits,
    /// USART loopback
    pub loopback: Loopback,
}

impl Config {
    /// set the baudrate for this config
    pub fn baudrate(mut self, baudrate: impl Into<Baud>) -> Self {
        self.baudrate = baudrate.into();
        self
    }

    /// set the parity to None for this config
    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    /// set the parity to Even for this config
    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    /// set the parity to Odd for this config
    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    /// set the wordlength to 8 bit for this config
    pub fn wordlength_8(mut self) -> Self {
        self.wordlength = WordLength::DataBits8;
        self
    }
    /// set the wordlength to 9 bit for this config
    pub fn wordlength_9(mut self) -> Self {
        self.wordlength = WordLength::DataBits9;
        self
    }
    /// set the stopbit for this config
    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

/// Invalid configuration indicator
#[derive(Debug)]
pub struct InvalidConfig;

impl Default for Config {
    fn default() -> Config {
        let baudrate = 9_600_u32.Bd();
        Config {
            baudrate,
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            loopback: Loopback::Normal,
        }
    }
}

/// Trait to mark serial pins with transmit capability.
pub trait TxPin<USART, FLEXCOMM> {
    /// setup tx pin as USART function
    fn setup(&self);
}

/// Trait to mark serial pins with receive capability.
pub trait RxPin<USART, FLEXCOMM> {
    /// setup rx pin as USART function
    fn setup(&self);
}

/// Macro to implement `TxPin` / `RxPin` for a certain pin, using a certain
/// alternative function and for a certain serial peripheral.
macro_rules! impl_pins {
    ($($pin:ident, $function:ident, $flexcomm_instance:ty, $usart_instance:ty, $trait:ident;)*) => {
        $(
            impl<MODE: PinMode> $trait<$usart_instance, $flexcomm_instance> for $pin<MODE> {
                fn setup(&self) {
                    self.set_alt_mode($function);
                }
            }
        )*
    }
}

#[cfg(any(feature = "io-100", feature = "io-180", feature = "io-208"))]
impl_pins!(
    P0_19, FUNC7, FLEXCOMM7, USART7, TxPin;
    P0_2,  FUNC1, FLEXCOMM3, USART3, TxPin;
    P0_20, FUNC7, FLEXCOMM7, USART7, RxPin;
    P0_22, FUNC1, FLEXCOMM6, USART6, TxPin;
    P0_24, FUNC1, FLEXCOMM0, USART0, RxPin;
    P0_25, FUNC1, FLEXCOMM0, USART0, TxPin;
    P0_26, FUNC1, FLEXCOMM2, USART2, RxPin;
    P0_27, FUNC1, FLEXCOMM2, USART2, TxPin;
    P0_29, FUNC1, FLEXCOMM0, USART0, RxPin;
    P0_3,  FUNC1, FLEXCOMM3, USART3, RxPin;
    P0_30, FUNC1, FLEXCOMM0, USART0, TxPin;
    P0_5,  FUNC2, FLEXCOMM4, USART4, RxPin;
    P0_8,  FUNC3, FLEXCOMM5, USART5, RxPin;
    P0_9,  FUNC3, FLEXCOMM5, USART5, TxPin;
    P1_1,  FUNC1, FLEXCOMM3, USART3, RxPin;
    P1_10, FUNC2, FLEXCOMM1, USART1, RxPin;
    P1_11, FUNC2, FLEXCOMM1, USART1, TxPin;
    P1_13, FUNC2, FLEXCOMM6, USART6, RxPin;
    P1_16, FUNC2, FLEXCOMM6, USART6, TxPin;
    P1_17, FUNC2, FLEXCOMM8, USART8, RxPin;
    P1_18, FUNC2, FLEXCOMM8, USART8, TxPin;
    P1_20, FUNC5, FLEXCOMM4, USART4, TxPin;
    P1_21, FUNC5, FLEXCOMM4, USART4, RxPin;
    P1_24, FUNC1, FLEXCOMM2, USART2, RxPin;
    P1_25, FUNC1, FLEXCOMM2, USART2, TxPin;
    P1_29, FUNC1, FLEXCOMM7, USART7, RxPin;
    P1_30, FUNC1, FLEXCOMM7, USART7, TxPin;
    P1_5,  FUNC1, FLEXCOMM0, USART0, RxPin;
    P1_6,  FUNC1, FLEXCOMM0, USART0, TxPin;
);

#[cfg(any(feature = "io-180", feature = "io-208"))]
impl_pins!(
    P2_12, FUNC5, FLEXCOMM5, USART5, RxPin;
    P2_13, FUNC5, FLEXCOMM5, USART5, TxPin;
    P2_17, FUNC5, FLEXCOMM8, USART8, RxPin;
    P2_18, FUNC2, FLEXCOMM3, USART3, RxPin;
    P2_19, FUNC3, FLEXCOMM7, USART7, RxPin;
    P2_20, FUNC3, FLEXCOMM7, USART7, TxPin;
    P2_29, FUNC3, FLEXCOMM8, USART8, TxPin;
    P2_3,  FUNC3, FLEXCOMM1, USART1, RxPin;
    P2_4,  FUNC3, FLEXCOMM1, USART1, TxPin;
    P3_16, FUNC1, FLEXCOMM8, USART8, RxPin;
    P3_17, FUNC1, FLEXCOMM8, USART0, TxPin;
    P3_2,  FUNC2, FLEXCOMM9, USART9, RxPin;
    P3_26, FUNC3, FLEXCOMM4, USART4, RxPin;
    P3_27, FUNC3, FLEXCOMM4, USART4, TxPin;
    P4_15, FUNC3, FLEXCOMM9, USART9, RxPin;
    P4_16, FUNC3, FLEXCOMM9, USART9, TxPin;
    P4_2,  FUNC2, FLEXCOMM6, USART6, RxPin;
    P4_9,  FUNC2, FLEXCOMM2, USART2, RxPin;
);
#[cfg(any(feature = "io-208"))]
impl_pins!(
    P4_20, FUNC3, FLEXCOMM2, USART2, RxPin;
    P4_21, FUNC3, FLEXCOMM2, USART2, TxPin;
    P4_28, FUNC4, FLEXCOMM1, USART1, RxPin;
    P4_29, FUNC4, FLEXCOMM1, USART1, TxPin;
    P5_0,  FUNC4, FLEXCOMM4, USART4, RxPin;
    P5_1,  FUNC4, FLEXCOMM4, USART4, TxPin;
    P5_7,  FUNC3, FLEXCOMM5, USART5, RxPin;
    P5_8,  FUNC3, FLEXCOMM5, USART5, TxPin;
);
/// Serial abstraction
pub struct Serial<USART, FLEXCOMM> {
    usart: USART,
    flexcomm: FLEXCOMM,
    rx: Rx<USART>,
    tx: Tx<USART>,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}
/// Serial Flexibl Interface
pub struct Fc<FLEXCOMM> {
    _flexcomm: PhantomData<FLEXCOMM>,
}

/// Macro to implement all usart
macro_rules! impl_usart {
    ($(
        $UARTX:ident: ($usartX: ident, $FLEXCOMMX: ty, $SerialXExt: ident),
    )+) => {
        $(
            /// Serial trait to make implicit declaration possible trough prelude
pub trait $SerialXExt<TX, RX> {
    /// Should return the usable UART or an InvalidConfig error
    fn usart(
        self,
        fc: $FLEXCOMMX,
        tx: TX,
        rx: RX,
        config: Config,
        syscon: &mut Syscon,
    ) -> Result<Serial<$UARTX, $FLEXCOMMX>, InvalidConfig>;
}

impl<TX, RX> $SerialXExt<TX, RX> for $UARTX
where
    TX: TxPin<$UARTX, $FLEXCOMMX>,
    RX: RxPin<$UARTX, $FLEXCOMMX>,
    //FC: FlexCommAsUart<$UARTX> + ClockControl + FlexcommClockControl,
{
    /// Configure USART peripheral according to `config`
    fn usart(
        self,
        fc: $FLEXCOMMX,
        tx: TX,
        rx: RX,
        config: Config,
        syscon: &mut Syscon,
    ) -> Result<Serial<$UARTX, $FLEXCOMMX>, InvalidConfig> {
        Serial::$usartX(self, fc, tx, rx, config, syscon)
    }
}

impl Serial<$UARTX, $FLEXCOMMX> {
    /// Configure USART peripheral according to `config`
    pub fn $usartX<TX, RX>(
        usart: $UARTX,
        flexcomm: $FLEXCOMMX,
        tx: TX,
        rx: RX,
        config: Config,
        syscon: &mut Syscon,
    ) -> Result<Self, InvalidConfig>
    where
        TX: TxPin<$UARTX, $FLEXCOMMX>,
        RX: RxPin<$UARTX, $FLEXCOMMX>,
        $FLEXCOMMX: FlexCommAsUart<$UARTX> + ClockControl + FlexcommClockControl,
    {
        tx.setup();
        rx.setup();
        flexcomm.enable_clock(syscon);
        flexcomm.as_usart();

        // Empty and enable tx/rx FIFO
        usart.fifocfg.modify(|_, w| {
            w.emptytx()
                .set_bit()
                .enabletx()
                .set_bit()
                .emptyrx()
                .set_bit()
                .enablerx()
                .set_bit()
        });

        // set tx_fifo level and RX fifo level
        // TODO: implement trough config
        usart
            .fifotrig
            .modify(|_, w| unsafe { w.txlvl().bits(0).rxlvl().bits(0) });

        // enable tx and rx level
        usart
            .fifotrig
            .modify(|_, w| w.txlvlena().enabled().rxlvlena().enabled());

        // safe because none of the possible parity patterns are reserved
        usart.cfg.modify(|_, w| unsafe {
            w.paritysel().bits(match config.parity {
                Parity::ParityNone => 0b00,
                Parity::ParityEven => 0b10,
                Parity::ParityOdd => 0b11,
            })
        });

        usart.cfg.modify(|_, w| {
            w.stoplen().bit(match config.stopbits {
                StopBits::STOP1 => false,
                StopBits::STOP2 => true,
            })
        });

        // safe because none of the possible word length patterns are reserved
        usart.cfg.modify(|_, w| unsafe {
            w.datalen().bits(match config.wordlength {
                WordLength::DataBits8 => 1,
                WordLength::DataBits9 => 2,
                WordLength::DataBits7 => 0,
            })
        });
        usart.cfg.modify(|_, w| {
            w.loop_().bit(match config.loopback {
                Loopback::Normal => false,
                Loopback::Loopback => true,
            })
        });

        usart.cfg.modify(|_, w| {
            w.syncen()
                .asynchronous_mode()
                .syncmst()
                .clear_bit()
                .rxpol()
                .standard()
                .mode32k()
                .disabled()
                .enable()
                .enabled()
        });

        // setup baudrate
        let fcsource = FlexcommClockSource::fro_12_mhz;

        flexcomm.set_clock(fcsource, syscon);
        let source_clock = flexcomm.get_clock_freq(syscon).unwrap().0;
        let mut best_diff: u32 = u32::MAX;
        let mut best_osrval: u32 = 0xF;
        let mut best_brgval: u32 = u32::MAX;
        let mut diff: u32;
        let mut baudrate: u32;
        let mut brgval: u32;
        let target_baud = config.baudrate.0;

        let cfg = usart.cfg.read();

        // If synchronous master mode is enabled, only configure the BRG value.
        if cfg.syncen().bit_is_set() && cfg.syncmst().bit_is_set() {
            brgval = source_clock / target_baud;
            usart.brg.write(|w| unsafe { w.bits(brgval - 1) });
        } else {
            for osrval in (8..best_osrval).rev() {
                brgval = (((source_clock * 10) / ((osrval + 1) * target_baud)) - 5) / 10;
                if brgval > 0xFFFF {
                    continue;
                }
                baudrate = source_clock / ((osrval + 1) * (brgval + 1));
                diff = match target_baud < baudrate {
                    true => baudrate - target_baud,
                    false => (target_baud - baudrate),
                };
                if diff < best_diff {
                    best_diff = diff;
                    best_osrval = osrval;
                    best_brgval = brgval;
                }
            }
            baudrate = source_clock / ((best_osrval + 1) * (best_brgval + 1));
            diff = match config.baudrate.0 < baudrate {
                true => (baudrate - config.baudrate.0),
                false => (config.baudrate.0 - baudrate),
            };
            if diff > ((config.baudrate.0 / 100) * 3) {
                // no support for this baudrate
                return Err(InvalidConfig);
            }

            // value over range
            if best_brgval > 0xFFFF {
                return Err(InvalidConfig);
            }
            if best_osrval > 0xF {
                return Err(InvalidConfig);
            }
            usart
                .osr
                .write(|w| unsafe { w.osrval().bits(best_osrval as u8) });
            usart
                .brg
                .write(|w| unsafe { w.brgval().bits(best_brgval as u16) });
        }

        Ok(Serial {
            usart,
            flexcomm,
            tx: Tx {
                _usart: PhantomData,
            },
            rx: Rx {
                _usart: PhantomData,
            },
        })
    }

    /// Starts listening for an interrupt event
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::TxErr => self.usart.fifointenset.modify(|_,w| w.txerr().set_bit()),
            Event::RxErr => self.usart.fifointenset.modify(|_,w| w.rxerr().set_bit()),
            Event::TxLvl => self.usart.fifointenset.modify(|_,w| w.txlvl().set_bit()),
            Event::RxLvl => self.usart.fifointenset.modify(|_,w| w.rxlvl().set_bit()),
        }
    }

    /// Stops listening for an interrupt event
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::TxErr => self.usart.fifointenclr.modify(|_,w| w.txerr().set_bit()),
            Event::RxErr => self.usart.fifointenclr.modify(|_,w| w.rxerr().set_bit()),
            Event::TxLvl => self.usart.fifointenclr.modify(|_,w| w.txlvl().set_bit()),
            Event::RxLvl => self.usart.fifointenclr.modify(|_,w| w.rxlvl().set_bit()),
        }
    }

    /// Set the Rx fifo level threshold for RxLvl event
    pub fn set_rx_threshold(&mut self, level: u8) {
        self.usart.fifotrig.modify(|_,w| unsafe{w.rxlvl().bits(level-1)});
    }

    /// Set the Tx fifo level threshold for TxLvl event
    pub fn set_tx_threshold(&mut self, level: u8) {
        self.usart.fifotrig.modify(|_,w| unsafe{w.txlvl().bits(level-1)})
    }

    /// Returns a pending and enabled `Event`.
    ///
    /// Multiple `Event`s can be signaled at the same time. In that case, an arbitrary
    /// pending event will be returned. Clearing the event condition will cause this
    /// method to return the other pending event(s).
    ///
    /// For an event to be returned by this method, it must first be enabled by calling
    /// `listen`.
    ///
    /// This method will never clear a pending event. If the event condition is not
    /// resolved by the user, it will be returned again by the next call to
    /// `pending_event`.

    pub fn pending_event(&self) -> Option<Event> {
        let fifointstat = self.usart.fifointstat.read();

        // give higest priority to errors, and then to rxlevel to avoid overruns
        if fifointstat.rxerr().bit_is_set(){
            Some(Event::RxErr)
        } else if fifointstat.txerr().bit_is_set(){
            Some(Event::TxErr)
        } else if fifointstat.rxlvl().bit_is_set(){
            Some(Event::RxLvl)
        } else if fifointstat.txlvl().bit_is_set(){
            Some(Event::TxLvl)
        } else {
            None
        }
    }

    /// Returns nb::Error::WouldBlock if there is no pending event
    /// returns the event if there is.
    pub fn wait_for_pending_event(&self) -> nb::Result<Event, Error>{
        match self.pending_event(){
            None => Err(nb::Error::WouldBlock),
            Some(x) => Ok(x),
        }
    }
    /// Checks for reception errors that may have occurred.
    ///
    /// Note that multiple errors can be signaled at the same time. In that case,
    /// calling this function repeatedly will return the remaining errors.
    pub fn check_errors(&mut self) -> Result<(), Error> {
        self.rx.check_errors()
    }

    /// Clears any signaled errors without returning them.
    pub fn clear_errors(&mut self) {
        self.rx.clear_errors()
    }

    /// Split Usart in tx and rx part
    pub fn split(self) -> (Tx<$UARTX>, Rx<$UARTX>) {
        (self.tx, self.rx)
    }

    /// Release the UART for new uses
    pub fn release(self) -> ($UARTX, $FLEXCOMMX) {
        (self.usart, self.flexcomm)
    }
}

impl hal::serial::Read<u8> for Serial<$UARTX, $FLEXCOMMX> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.rx.read()
    }
}

impl hal::serial::Write<u8> for Serial<$UARTX, $FLEXCOMMX> {
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(byte)
    }
}

impl Rx<$UARTX> {
    /// Returns true if the line idle status is set
    /// This reads the flexcomm's RXIDLE bit in the STAT register.
    /// This bit is set by hardware when an Idle Line is detected.
    pub fn is_idle(&self) -> bool {
        let stat = unsafe { (*$UARTX::ptr()).stat.read() };
        stat.rxidle().bit_is_set()
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        let fifostat = unsafe { (*$UARTX::ptr()).fifostat.read() };
        fifostat.rxnotempty().bit_is_set()
    }

    /// Checks for reception errors that may have occurred.
    ///
    /// Note that multiple errors can be signaled at the same time. In that case,
    /// calling this function repeatedly will return the remaining errors.
    pub fn check_errors(&mut self) -> Result<(), Error> {
        let stat_r = unsafe { (*$UARTX::ptr()).stat.read() };
        let fifostat_r = unsafe { (*$UARTX::ptr()).fifostat.read() };
        let stat = unsafe { &(*$UARTX::ptr()).stat };
        let fifostat = unsafe { &(*$UARTX::ptr()).fifostat };

        // We don't want to drop any errors, so check each error bit in sequence. If
        // any bit is set, clear it and return its error.
        if stat_r.framerrint().bit_is_set() {
            stat.write(|w| w.framerrint().set_bit());
            return Err(Error::Framing.into());
        } else if stat_r.parityerrint().bit_is_set() {
            stat.write(|w| w.parityerrint().set_bit());
            return Err(Error::Parity.into());
        } else if stat_r.rxnoiseint().bit_is_set() {
            stat.write(|w| w.rxnoiseint().set_bit());
            return Err(Error::Noise.into());
        } else if stat_r.aberr().bit_is_set() {
            stat.write(|w| w.aberr().set_bit());
            return Err(Error::AutoBaud.into());
        } else if fifostat_r.txerr().bit_is_set() {
            fifostat.write(|w| w.txerr().set_bit());
            return Err(Error::TxFifo.into());
        } else if fifostat_r.rxerr().bit_is_set() {
            fifostat.write(|w| w.rxerr().set_bit());
            return Err(Error::RxFifo.into());
        }
        Ok(())
    }

    /// Clears any signaled errors without returning them.
    pub fn clear_errors(&mut self) {
        let stat = unsafe { &(*$UARTX::ptr()).stat };
        let fifostat = unsafe { &(*$UARTX::ptr()).fifostat };

        stat.write(|w| {
            w.framerrint()
                .set_bit()
                .parityerrint()
                .set_bit()
                .rxnoiseint()
                .set_bit()
                .aberr()
                .set_bit()
        });
        fifostat.write(|w| w.txerr().set_bit().rxerr().set_bit());
    }
}

impl hal::serial::Read<u8> for Rx<$UARTX> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.check_errors()?;

        // NOTE(unsafe) atomic read with no side effects
        let fifostat = unsafe { (*$UARTX::ptr()).fifostat.read() };

        // Check if a byte is available
        if fifostat.rxnotempty().bit_is_set() {
            Ok(unsafe { (*$UARTX::ptr()).fiford.read().rxdata().bits() as u8 })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl hal::serial::Write<u8> for Tx<$UARTX> {
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let fifostat = unsafe { (*$UARTX::ptr()).fifostat.read() };

        // Check tx empty
        if fifostat.txempty().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let fifostat = unsafe { (*$UARTX::ptr()).fifostat.read() };

        // check if there is space left in the tx fifo
        if fifostat.txnotfull().bits() {
            // NOTE(unsafe) atomic write to stateless register
            unsafe {
                (*$UARTX::ptr())
                    .fifowr
                    .write(|w| w.txdata().bits(byte as u16))
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Tx<$UARTX> {
    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        let fifostat = unsafe { (*$UARTX::ptr()).fifostat.read() };
        fifostat.txempty().bit_is_set()
    }
}

        )*
    }

}

impl_usart! {
    USART0: (usart0, FLEXCOMM0, Serial0Ext),
    USART1: (usart1, FLEXCOMM1, Serial1Ext),
    USART2: (usart2, FLEXCOMM2, Serial2Ext),
    USART3: (usart3, FLEXCOMM3, Serial3Ext),
    USART4: (usart4, FLEXCOMM4, Serial4Ext),
    USART5: (usart5, FLEXCOMM5, Serial5Ext),
    USART6: (usart6, FLEXCOMM6, Serial6Ext),
    USART7: (usart7, FLEXCOMM7, Serial7Ext),
    USART8: (usart8, FLEXCOMM8, Serial8Ext),
}

#[cfg(feature = "flexcomm-10")]
impl_usart! {
    USART9: (usart9, FLEXCOMM9, Serial9Ext),
}

impl<USART, FLEXCOMM> fmt::Write for Serial<USART, FLEXCOMM>
where
    Serial<USART, FLEXCOMM>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();

        //self.flush().map_err(|_| fmt::Error)?;

        Ok(())
    }
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();

        //self.flush().map_err(|_| fmt::Error)?;

        Ok(())
    }
}
