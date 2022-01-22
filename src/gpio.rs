//! GeneralPurpose Input / Output
//! 
//! Some GPIO are missing as of now because IOCON type `A` and type `I` are
//! not yet implemented
#![deny(missing_docs)]

use crate::syscon::Syscon;
use core::marker::PhantomData;
use crate::pac;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, syscon: &mut Syscon, iocon: &mut pac::IOCON) -> Self::Parts;
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;

/// pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Open Drain input or output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

mod sealed {
    pub trait Sealed {}
}

/// Marker trait for valid pin modes (type state)
///
/// It can not be implemented by outside types
pub trait PinMode: sealed::Sealed {
    // these constant are used to implement the pin configuration code.
    // they are not part of the public API.
    #[doc(hidden)]
    const DIR: u8;
    #[doc(hidden)]
    const MODE: u8;
    #[doc(hidden)]
    const DIGIMODE: u8;
    #[doc(hidden)]
    const OD: u8;

    // ????
}

impl sealed::Sealed for Input<Floating> {}
impl PinMode for Input<Floating> {
    const DIR: u8 = 0b0;
    const MODE: u8 = 0x0;
    const DIGIMODE: u8 = 0b1;
    const OD: u8 = 0b1;
}

impl sealed::Sealed for Input<PullDown> {}
impl PinMode for Input<PullDown> {
    const DIR: u8 = 0b0;
    const MODE: u8 = 0x1;
    const DIGIMODE: u8 = 0b1;
    const OD: u8 = 0b1;
}

impl sealed::Sealed for Input<PullUp> {}
impl PinMode for Input<PullUp> {
    const DIR: u8 = 0b0;
    const MODE: u8 = 0x2;
    const DIGIMODE: u8 = 0b1;
    const OD: u8 = 0b1;
}

impl sealed::Sealed for Analog {}
impl PinMode for Analog {
    const DIR: u8 = 0b0;
    const MODE: u8 = 0x0;
    const DIGIMODE: u8 = 0b0;
    const OD: u8 = 0b1;
}

impl sealed::Sealed for Output<OpenDrain> {}
impl PinMode for Output<OpenDrain> {
    const DIR: u8 = 0b1;
    const MODE: u8 = 0x0;
    const DIGIMODE: u8 = 0b1;
    const OD: u8 = 0b1;
}
impl sealed::Sealed for Output<PushPull> {}
impl PinMode for Output<PushPull> {
    const DIR: u8 = 0b1;
    const MODE: u8 = 0x0;
    const DIGIMODE: u8 = 0b1;
    const OD: u8 = 0b0;
}

/// GPIO Pin speed selection
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Speed {
    /// Standard Speed
    StandardSlew = 0,
    /// Fast Speed (usefull for I2C)
    FastSlew = 1,
}


/// Pin Alternate function for peripheral IO
pub enum AltMode {
    /// Alternate function 0
    FUNC0 = 0,
    /// Alternate function 1
    FUNC1 = 1,
    /// Alternate function 2
    FUNC2 = 2,
    /// Alternate function 3
    FUNC3 = 3,
    /// Alternate function 4
    FUNC4 = 4,
    /// Alternate function 5
    FUNC5 = 5,
    /// Alternate function 6
    FUNC6 = 6,
    /// Alternate function 7
    FUNC7 = 7,
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Port {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
    P4 = 4,
    P5 = 5,
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum IoType {
    TYPED = 0,
    TYPEI = 1,
    TYPEA = 2,
}

macro_rules! gpio {
    ($GPIO:ident, 
        [
            $(
                $PX:ident, $Portn:expr,
                    [ 
                        $(
                            $PX_X:ident: ($pioX_X:ident, $ioconpioX_Xreg:ident, $Pinn:expr, $MODE:ty),
                        )+
                    ],
            )+
        ])  => {
        /// GPIO
        pub mod gpio {
            use core::marker::PhantomData;
            use crate::hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};
            use crate::pac::GPIO;
            use crate::pac::IOCON;
            use crate::pac;
            use crate::syscon::{Syscon, ClockControl};
            use super::{
                Floating, GpioExt, Input, OpenDrain, Output, Speed,
                PullDown, PullUp, PushPull, AltMode, Analog, Port,
                PinMode
            };
            /// GPIO parts
            pub struct Parts {
                $(
                    $(
                        /// Pin
                        pub $pioX_X: $PX_X<$MODE>,
                    )+
                )+
            }
            impl GpioExt for GPIO {
                type Parts = Parts;
                fn split(self, syscon: &mut Syscon, iocon: &mut pac::IOCON) -> Parts {
                    self.enable_clock(syscon);
                    iocon.enable_clock(syscon);
                    Parts {
                        $(
                            $(
                                $pioX_X: $PX_X {
                                    _mode: PhantomData,
                                },
                            )+
                        )+
                    }
                }
            }
            /// Partially erased pin
            pub struct Pin<MODE> {
                pin_number: u8,
                port_number: usize,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> Pin<MODE> {
                /// Returns the port this pin is part of.
                pub fn port(&self) -> usize {
                    self.port_number
                }

                /// Returns this pin's number inside its port.
                pub fn pin(&self) -> u8 {
                    self.pin_number
                }
            }

            impl<MODE> OutputPin for Pin<Output<MODE>> {
                type Error = void::Void;

                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*GPIO::ptr()).set[self.port()].write(|w| w.setp().bits(1 << self.pin()))};
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*GPIO::ptr()).clr[self.port()].write(|w| w.clrp().bits(1 << self.pin()))};
                    Ok(())
                }
            }

            impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    let is_high = !self.is_set_low()?;
                    Ok(is_high)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*GPIO::ptr()).pin[self.port()].read().bits() & (1 << self.pin()) == 0};
                    Ok(is_low)
                }
            }

            impl<MODE> toggleable::Default for Pin<Output<MODE>> {}

            impl<MODE> InputPin for Pin<Output<MODE>> {
                type Error = void::Void;

                fn is_high(&self) -> Result<bool, Self::Error> {
                    let is_high = !self.is_low()?;
                    Ok(is_high)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*GPIO::ptr()).pin[self.port()].read().bits() & (1 << self.pin()) == 0};
                    Ok(is_low)
                }
            }

            impl<MODE> InputPin for Pin<Input<MODE>> {
                type Error = void::Void;

                fn is_high(&self) -> Result<bool, Self::Error> {
                    let is_high = !self.is_low()?;
                    Ok(is_high)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*GPIO::ptr()).pin[self.port()].read().bits() & (1 << self.pin()) == 0};
                    Ok(is_low)
                }
            }
            $(
                $(
                    /// Pin
                    pub struct $PX_X<MODE> {
                        _mode: PhantomData<MODE>,
                    }

                    impl<MODE> $PX_X<MODE> {

                        /// Returns the port this pin is part of.
                        pub fn port(&self) -> u8 {
                            Port::$PX as u8
                        }

                        /// Returns this pin's number inside its port.
                        pub fn pin_number(&self) -> u8 {
                            $Pinn
                        }
                    }

                impl<MODE: PinMode> $PX_X<MODE> {
                        /// Puts `self` into mode `M`.
                        ///
                        /// This violates the type state constraints from `MODE`, so callers must
                        /// ensure they use this properly.
                        fn mode<M: PinMode>(&mut self) {
                            let offset = $Pinn;

                            unsafe{
                                (*IOCON::ptr()).$ioconpioX_Xreg.modify(|_,w| {
                                    w.mode().bits(u8::from(M::MODE)).digimode().bit(M::DIGIMODE != 0).od().bit(M::OD != 0)
                                });
                                (*GPIO::ptr()).dir[$Portn].modify(|r,w| {
                                    w.dirp().bits(r.dirp().bits() & !(0b1 << offset) | ((M::DIR as u32) << offset))
                                });

                            }
                        
                        }

                        fn with_mode<M, F, R>(
                            &mut self,
                            f: F
                        ) -> R
                        where
                            M: PinMode,
                            F: FnOnce(&mut $PX_X<M>) -> R,
                        {
                            struct ResetMode<'a, ORIG: PinMode> {
                                pin: &'a mut $PX_X<ORIG>,
                            }

                            impl<'a, ORIG: PinMode> Drop for ResetMode<'a, ORIG> {
                                fn drop(&mut self) {
                                    self.pin.mode::<ORIG>();
                                }
                            }

                            self.mode::<M>();

                            // This will reset the pin back to the original mode when dropped.
                            // (so either when `with_mode` returns or when `f` unwinds)
                            let _resetti = ResetMode { pin: self };

                            let mut witness = $PX_X {
                                _mode: PhantomData
                            };

                            f(&mut witness)
                        }

                        /// Configures the pin to operate as a floating input pin.
                        pub fn into_floating_input(
                            mut self,
                        ) -> $PX_X<Input<Floating>> {
                            self.mode::<Input<Floating>>();
                            $PX_X {
                                _mode: PhantomData
                            }
                        }

                        /// Temporarily configures this pin as a floating input.
                        ///
                        /// The closure `f` is called with the reconfigured pin. After it returns,
                        /// the pin will be configured back.
                        pub fn with_floating_input<R>(
                            &mut self,
                            f: impl FnOnce(&mut $PX_X<Input<Floating>>) -> R,
                        ) -> R {
                            self.with_mode(f)
                        }

                        /// Configures the pin to operate as a pulled-down input pin.
                        pub fn into_pull_down_input(
                            mut self,
                        ) -> $PX_X<Input<PullDown>> {
                            self.mode::<Input<PullDown>>();
                            $PX_X {
                                _mode: PhantomData
                            }
                        }

                        /// Temporarily configures this pin as a pulled-down input.
                        ///
                        /// The closure `f` is called with the reconfigured pin. After it returns,
                        /// the pin will be configured back.
                        pub fn with_pull_down_input<R>(
                            &mut self,
                            f: impl FnOnce(&mut $PX_X<Input<PullDown>>) -> R,
                        ) -> R {
                            self.with_mode(f)
                        }

                        /// Configures the pin to operate as a pulled-up input pin.
                        pub fn into_pull_up_input(
                            mut self,
                        ) -> $PX_X<Input<PullUp>> {
                            self.mode::<Input<PullUp>>();
                            $PX_X {
                                _mode: PhantomData
                            }
                        }

                        /// Temporarily configures this pin as a pulled-up input.
                        ///
                        /// The closure `f` is called with the reconfigured pin. After it returns,
                        /// the pin will be configured back.
                        pub fn with_pull_up_input<R>(
                            &mut self,
                            f: impl FnOnce(&mut $PX_X<Input<PullUp>>) -> R,
                        ) -> R {
                            self.with_mode(f)
                        }

                        /// Configures the pin to operate as an analog pin.
                        pub fn into_analog(
                            mut self,
                        ) -> $PX_X<Analog> {
                            self.mode::<Analog>();
                            $PX_X {
                                _mode: PhantomData
                            }
                        }

                        /// Temporarily configures this pin as an analog pin.
                        ///
                        /// The closure `f` is called with the reconfigured pin. After it returns,
                        /// the pin will be configured back.
                        pub fn with_analog<R>(
                            &mut self,
                            f: impl FnOnce(&mut $PX_X<Analog>) -> R,
                        ) -> R {
                            self.with_mode(f)
                        }

                        /// Configures the pin to operate as an open drain output pin.
                        pub fn into_open_drain_output(
                            mut self,
                        ) -> $PX_X<Output<OpenDrain>> {
                            self.mode::<Output<OpenDrain>>();
                            $PX_X {
                                _mode: PhantomData
                            }
                        }

                        /// Temporarily configures this pin as an open drain output.
                        ///
                        /// The closure `f` is called with the reconfigured pin. After it returns,
                        /// the pin will be configured back.
                        pub fn with_open_drain_output<R>(
                            &mut self,
                            f: impl FnOnce(&mut $PX_X<Output<OpenDrain>>) -> R,
                        ) -> R {
                            self.with_mode(f)
                        }

                        /// Configures the pin to operate as an push-pull output pin.
                        pub fn into_push_pull_output(
                            mut self,
                        ) -> $PX_X<Output<PushPull>> {
                            self.mode::<Output<PushPull>>();
                            $PX_X {
                                _mode: PhantomData
                            }
                        }

                        /// Temporarily configures this pin as a push-pull output.
                        ///
                        /// The closure `f` is called with the reconfigured pin. After it returns,
                        /// the pin will be configured back.
                        pub fn with_push_pull_output<R>(
                            &mut self,
                            f: impl FnOnce(&mut $PX_X<Output<PushPull>>) -> R,
                        ) -> R {
                            self.with_mode(f)
                        }

                        /// Set pin speed.
                        pub fn set_speed(self, speed: Speed) -> Self {
                            unsafe {
                                &(*IOCON::ptr()).$ioconpioX_Xreg.modify(|_,w| w.slew().bit(speed != Speed::StandardSlew))
                            };
                            /*unsafe {
                                &(*$GPIOX::ptr()).ospeedr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
                                })
                            };*/
                            self
                        }

                        #[allow(dead_code)]
                        pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                            unsafe{
                                &(*IOCON::ptr()).$ioconpioX_Xreg.modify(|_,w| w.func().bits(mode as u8))
                            };
                            /*unsafe {
                                if offset2 < 32 {
                                    &(*$GPIOX::ptr()).afrl.modify(|r, w| {
                                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                                    });
                                } else {
                                    let offset2 = offset2 - 32;
                                    &(*$GPIOX::ptr()).afrh.modify(|r, w| {
                                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                                    });
                                }
                                &(*$GPIOX::ptr()).moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                                });
                            }*/
                        }
                    }

                    impl<MODE> $PX_X<Output<MODE>> {
                        /// Erases the pin number from the type
                        ///
                        /// This is useful when you want to collect the pins into an array where you
                        /// need all the elements to have the same type
                        pub fn downgrade(self) -> Pin<Output<MODE>> {
                            Pin{
                                pin_number: $Pinn,
                                port_number : $Portn,
                                _mode: self._mode,
                            }
                        }
                    }

                    impl<MODE> OutputPin for $PX_X<Output<MODE>> {
                        type Error = void::Void;

                        fn set_high(&mut self) -> Result<(), Self::Error> {
                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe { (*GPIO::ptr()).set[$Portn].write(|w| w.setp().bits(1 << $Pinn))};
                            //unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) };
                            Ok(())
                        }

                        fn set_low(&mut self) -> Result<(), Self::Error> {
                            // NOTE(unsafe) atomic write to a stateless register
                            unsafe { (*GPIO::ptr()).clr[$Portn].write(|w| w.clrp().bits(1 << $Pinn))};
                            //unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << ($i + 16))) };
                            Ok(())
                        }
                    }
                    impl<MODE> StatefulOutputPin for $PX_X<Output<MODE>> {

                        fn is_set_high(&self) -> Result<bool, Self::Error> {
                            let is_set_high = !self.is_set_low()?;
                            Ok(is_set_high)
                        }

                        fn is_set_low(&self) -> Result<bool, Self::Error> {
                            // NOTE(unsafe) atomic read with no side effects
                            let is_set_low = unsafe { (*GPIO::ptr()).pin[$Portn].read().bits() & (1 << $Pinn) == 0};
                            Ok(is_set_low)
                        }
                    }
                    impl<MODE> toggleable::Default for $PX_X<Output<MODE>> {}

                    impl<MODE> InputPin for $PX_X<Output<MODE>> {
                        type Error = void::Void;

                        fn is_high(&self) -> Result<bool, Self::Error> {
                            let is_high = !self.is_low()?;
                            Ok(is_high)
                        }

                        fn is_low(&self) -> Result<bool, Self::Error> {
                            // NOTE(unsafe) atomic read with no side effects
                            let is_low = unsafe { (*GPIO::ptr()).pin[$Portn].read().bits() & (1 << $Pinn) == 0};
                            Ok(is_low)
                        }
                    }

                    impl<MODE> $PX_X<Input<MODE>> {
                        /// Erases the pin number from the type
                        ///
                        /// This is useful when you want to collect the pins into an array where you
                        /// need all the elements to have the same type
                        pub fn downgrade(self) -> Pin<Input<MODE>> {
                            Pin {
                                pin_number: $Pinn,
                                port_number: $Portn,
                                _mode: self._mode,
                            }
                        }
                    }
                    impl<MODE> InputPin for $PX_X<Input<MODE>> {
                        type Error = void::Void;

                        fn is_high(&self) -> Result<bool, Self::Error> {
                            let is_high = !self.is_low()?;
                            Ok(is_high)
                        }

                        fn is_low(&self) -> Result<bool, Self::Error> {
                            // NOTE(unsafe) atomic read with no side effects
                            let is_low = unsafe { (*GPIO::ptr()).pin[$Portn].read().bits() & (1 << $Pinn) == 0};
                            Ok(is_low)
                        }
                    }
                )+
            )+
        }
    }
}


#[cfg(feature = "io-100")]
gpio!(GPIO, 
    [  
        P0, 0, [
            P0_0: (pio0_0, pio00, 0, Analog),
            P0_1: (pio0_1, pio01, 1, Analog),
            P0_2: (pio0_2, pio02, 2, Analog),
            P0_3: (pio0_3, pio03, 3, Analog),
            P0_4: (pio0_4, pio04, 4, Analog),
            P0_5: (pio0_5, pio05, 5, Analog),
            P0_6: (pio0_6, pio06, 6, Analog),
            P0_7: (pio0_7, pio07, 7, Analog),
            P0_8: (pio0_8, pio08, 8, Analog),
            P0_9: (pio0_9, pio09, 9, Analog),
            // P0_10: (pio0_10, pio010, 10, Analog), TYPE A
            // P0_11: (pio0_11, pio011, 11, Analog), TYPE A
            // P0_12: (pio0_12, pio012, 12, Analog), TYPE A
            // P0_13: (pio0_13, pio013, 13, Analog), TYPE I
            // P0_14: (pio0_14, pio014, 14, Analog), TYPE I
            // P0_15: (pio0_15, pio015, 15, Analog), TYPE A
            // P0_16: (pio0_16, pio016, 16, Analog), TYPE A
            P0_17: (pio0_17, pio017, 17, Analog),
            P0_18: (pio0_18, pio018, 18, Analog),
            P0_19: (pio0_19, pio019, 19, Analog),
            P0_20: (pio0_20, pio020, 20, Analog),
            P0_21: (pio0_21, pio021, 21, Analog),
            P0_22: (pio0_22, pio022, 22, Analog),
            // P0_23: (pio0_23, pio023, 23, Analog), TYPE A
            P0_24: (pio0_24, pio024, 24, Analog),
            P0_25: (pio0_25, pio025, 25, Analog),
            P0_26: (pio0_26, pio026, 26, Analog),
            P0_27: (pio0_27, pio027, 27, Analog),
            P0_28: (pio0_28, pio028, 28, Analog),
            P0_29: (pio0_29, pio029, 29, Analog),
            P0_30: (pio0_30, pio030, 30, Analog),
            // P0_31: (pio0_31, pio031, 31, Analog), TYPE A
        ],
        P1, 1, [
            // P1_0: (pio1_0, pio10, 0, Analog), TYPE A
            P1_1: (pio1_1, pio11, 1, Analog),
            P1_2: (pio1_2, pio12, 2, Analog),
            P1_3: (pio1_3, pio13, 3, Analog),
            P1_4: (pio1_4, pio14, 4, Analog),
            P1_5: (pio1_5, pio15, 5, Analog),
            P1_6: (pio1_6, pio16, 6, Analog),
            P1_7: (pio1_7, pio17, 7, Analog),
            P1_8: (pio1_8, pio18, 8, Analog),
            P1_9: (pio1_9, pio19, 9, Analog),
            P1_10: (pio1_10, pio110, 10, Analog),
            P1_11: (pio1_11, pio111, 11, Analog),
            P1_12: (pio1_12, pio112, 12, Analog),
            P1_13: (pio1_13, pio113, 13, Analog),
            P1_14: (pio1_14, pio114, 14, Analog),
            P1_15: (pio1_15, pio115, 15, Analog),
            P1_16: (pio1_16, pio116, 16, Analog),
            P1_17: (pio1_17, pio117, 17, Analog),
            P1_18: (pio1_18, pio118, 18, Analog),
            P1_19: (pio1_19, pio119, 19, Analog),
            P1_20: (pio1_20, pio120, 20, Analog),
            P1_21: (pio1_21, pio121, 21, Analog),
            P1_22: (pio1_22, pio122, 22, Analog),
            P1_23: (pio1_23, pio123, 23, Analog),
            P1_24: (pio1_24, pio124, 24, Analog),
            P1_25: (pio1_25, pio125, 25, Analog),
            P1_26: (pio1_26, pio126, 26, Analog),
            P1_27: (pio1_27, pio127, 27, Analog),
            P1_28: (pio1_28, pio128, 28, Analog),
            P1_29: (pio1_29, pio129, 29, Analog),
            P1_30: (pio1_30, pio130, 30, Analog),
            P1_31: (pio1_31, pio131, 31, Analog),
        ],
    ]
);

#[cfg(feature = "io-180")]
gpio!(GPIO, 
    [  
        P0, 0, [
            P0_0: (pio0_0, pio00, 0, Analog),
            P0_1: (pio0_1, pio01, 1, Analog),
            P0_2: (pio0_2, pio02, 2, Analog),
            P0_3: (pio0_3, pio03, 3, Analog),
            P0_4: (pio0_4, pio04, 4, Analog),
            P0_5: (pio0_5, pio05, 5, Analog),
            P0_6: (pio0_6, pio06, 6, Analog),
            P0_7: (pio0_7, pio07, 7, Analog),
            P0_8: (pio0_8, pio08, 8, Analog),
            P0_9: (pio0_9, pio09, 9, Analog),
            // P0_10: (pio0_10, pio010, 10, Analog), TYPE A
            // P0_11: (pio0_11, pio011, 11, Analog), TYPE A
            // P0_12: (pio0_12, pio012, 12, Analog), TYPE A
            // P0_13: (pio0_13, pio013, 13, Analog), TYPE I
            // P0_14: (pio0_14, pio014, 14, Analog), TYPE I
            // P0_15: (pio0_15, pio015, 15, Analog), TYPE A
            // P0_16: (pio0_16, pio016, 16, Analog), TYPE A
            P0_17: (pio0_17, pio017, 17, Analog),
            P0_18: (pio0_18, pio018, 18, Analog),
            P0_19: (pio0_19, pio019, 19, Analog),
            P0_20: (pio0_20, pio020, 20, Analog),
            P0_21: (pio0_21, pio021, 21, Analog),
            P0_22: (pio0_22, pio022, 22, Analog),
            // P0_23: (pio0_23, pio023, 23, Analog), TYPE A
            P0_24: (pio0_24, pio024, 24, Analog),
            P0_25: (pio0_25, pio025, 25, Analog),
            P0_26: (pio0_26, pio026, 26, Analog),
            P0_27: (pio0_27, pio027, 27, Analog),
            P0_28: (pio0_28, pio028, 28, Analog),
            P0_29: (pio0_29, pio029, 29, Analog),
            P0_30: (pio0_30, pio030, 30, Analog),
            // P0_31: (pio0_31, pio031, 31, Analog), TYPE A
        ],
        P1, 1, [
            // P1_0: (pio1_0, pio10, 0, Analog), TYPE A
            P1_1: (pio1_1, pio11, 1, Analog),
            P1_2: (pio1_2, pio12, 2, Analog),
            P1_3: (pio1_3, pio13, 3, Analog),
            P1_4: (pio1_4, pio14, 4, Analog),
            P1_5: (pio1_5, pio15, 5, Analog),
            P1_6: (pio1_6, pio16, 6, Analog),
            P1_7: (pio1_7, pio17, 7, Analog),
            P1_8: (pio1_8, pio18, 8, Analog),
            P1_9: (pio1_9, pio19, 9, Analog),
            P1_10: (pio1_10, pio110, 10, Analog),
            P1_11: (pio1_11, pio111, 11, Analog),
            P1_12: (pio1_12, pio112, 12, Analog),
            P1_13: (pio1_13, pio113, 13, Analog),
            P1_14: (pio1_14, pio114, 14, Analog),
            P1_15: (pio1_15, pio115, 15, Analog),
            P1_16: (pio1_16, pio116, 16, Analog),
            P1_17: (pio1_17, pio117, 17, Analog),
            P1_18: (pio1_18, pio118, 18, Analog),
            P1_19: (pio1_19, pio119, 19, Analog),
            P1_20: (pio1_20, pio120, 20, Analog),
            P1_21: (pio1_21, pio121, 21, Analog),
            P1_22: (pio1_22, pio122, 22, Analog),
            P1_23: (pio1_23, pio123, 23, Analog),
            P1_24: (pio1_24, pio124, 24, Analog),
            P1_25: (pio1_25, pio125, 25, Analog),
            P1_26: (pio1_26, pio126, 26, Analog),
            P1_27: (pio1_27, pio127, 27, Analog),
            P1_28: (pio1_28, pio128, 28, Analog),
            P1_29: (pio1_29, pio129, 29, Analog),
            P1_30: (pio1_30, pio130, 30, Analog),
            P1_31: (pio1_31, pio131, 31, Analog),
        ],
        P2, 2, [
            // P2_0: (pio2_0, pio20, 0, Analog), TYPE A
            // P2_1: (pio2_1, pio21, 1, Analog), TYPE A
            P2_2: (pio2_2, pio22, 2, Analog),
            P2_3: (pio2_3, pio23, 3, Analog),
            P2_4: (pio2_4, pio24, 4, Analog),
            P2_5: (pio2_5, pio25, 5, Analog),
            P2_6: (pio2_6, pio26, 6, Analog),
            P2_7: (pio2_7, pio27, 7, Analog),
            P2_8: (pio2_8, pio28, 8, Analog),
            P2_9: (pio2_9, pio29, 9, Analog),
            P2_10: (pio2_10, pio210, 10, Analog),
            P2_11: (pio2_11, pio211, 11, Analog),
            P2_12: (pio2_12, pio212, 12, Analog),
            P2_13: (pio2_13, pio213, 13, Analog),
            P2_14: (pio2_14, pio214, 14, Analog),
            P2_15: (pio2_15, pio215, 15, Analog),
            P2_16: (pio2_16, pio216, 16, Analog),
            P2_17: (pio2_17, pio217, 17, Analog),
            P2_18: (pio2_18, pio218, 18, Analog),
            P2_19: (pio2_19, pio219, 19, Analog),
            P2_20: (pio2_20, pio220, 20, Analog),
            P2_21: (pio2_21, pio221, 21, Analog),
            P2_22: (pio2_22, pio222, 22, Analog),
            P2_23: (pio2_23, pio223, 23, Analog),
            P2_24: (pio2_24, pio224, 24, Analog),
            P2_25: (pio2_25, pio225, 25, Analog),
            P2_26: (pio2_26, pio226, 26, Analog),
            P2_27: (pio2_27, pio227, 27, Analog),
            P2_28: (pio2_28, pio228, 28, Analog),
            P2_29: (pio2_29, pio229, 29, Analog),
            P2_30: (pio2_30, pio230, 30, Analog),
            P2_31: (pio2_31, pio231, 31, Analog),
        ],
        P3, 3, [
            P3_0: (pio3_0, pio30, 0, Analog),
            P3_1: (pio3_1, pio31, 1, Analog),
            P3_2: (pio3_2, pio32, 2, Analog),
            P3_3: (pio3_3, pio33, 3, Analog),
            P3_4: (pio3_4, pio34, 4, Analog),
            P3_5: (pio3_5, pio35, 5, Analog),
            P3_6: (pio3_6, pio36, 6, Analog),
            P3_7: (pio3_7, pio37, 7, Analog),
            P3_8: (pio3_8, pio38, 8, Analog),
            P3_9: (pio3_9, pio39, 9, Analog),
            P3_10: (pio3_10, pio310, 10, Analog),
            P3_11: (pio3_11, pio311, 11, Analog),
            P3_12: (pio3_12, pio312, 12, Analog),
            P3_13: (pio3_13, pio313, 13, Analog),
            P3_14: (pio3_14, pio314, 14, Analog),
            P3_15: (pio3_15, pio315, 15, Analog),
            P3_16: (pio3_16, pio316, 16, Analog),
            P3_17: (pio3_17, pio317, 17, Analog),
            P3_18: (pio3_18, pio318, 18, Analog),
            P3_19: (pio3_19, pio319, 19, Analog),
            P3_20: (pio3_20, pio320, 20, Analog),
            // P3_21: (pio3_21, pio321, 21, Analog), TYPE A
            // P3_22: (pio3_22, pio322, 22, Analog), TYPE A
            // P3_23: (pio3_23, pio323, 23, Analog), TYPE I
            // P3_24: (pio3_24, pio324, 24, Analog), TYPE I
            P3_25: (pio3_25, pio325, 25, Analog),
            P3_26: (pio3_26, pio326, 26, Analog),
            P3_27: (pio3_27, pio327, 27, Analog),
            P3_28: (pio3_28, pio328, 28, Analog),
            P3_29: (pio3_29, pio329, 29, Analog),
            P3_30: (pio3_30, pio330, 30, Analog),
            P3_31: (pio3_31, pio331, 31, Analog),
        ],
        P4, 4, [
            P4_0: (pio4_0, pio40, 0, Analog),
            P4_1: (pio4_1, pio41, 1, Analog),
            P4_2: (pio4_2, pio42, 2, Analog),
            P4_3: (pio4_3, pio43, 3, Analog),
            P4_4: (pio4_4, pio44, 4, Analog),
            P4_5: (pio4_5, pio45, 5, Analog),
            P4_6: (pio4_6, pio46, 6, Analog),
            P4_7: (pio4_7, pio47, 7, Analog),
            P4_8: (pio4_8, pio48, 8, Analog),
            P4_9: (pio4_9, pio49, 9, Analog),
            P4_10: (pio4_10, pio410, 10, Analog),
            P4_11: (pio4_11, pio411, 11, Analog),
            P4_12: (pio4_12, pio412, 12, Analog),
            P4_13: (pio4_13, pio413, 13, Analog),
            P4_14: (pio4_14, pio414, 14, Analog),
            P4_15: (pio4_15, pio415, 15, Analog),
            P4_16: (pio4_16, pio416, 16, Analog),
        ],
    ]
);

#[cfg(feature = "io-208")]
gpio!(GPIO, 
    [  
        P0, 0, [
            P0_0: (pio0_0, pio00, 0, Analog),
            P0_1: (pio0_1, pio01, 1, Analog),
            P0_2: (pio0_2, pio02, 2, Analog),
            P0_3: (pio0_3, pio03, 3, Analog),
            P0_4: (pio0_4, pio04, 4, Analog),
            P0_5: (pio0_5, pio05, 5, Analog),
            P0_6: (pio0_6, pio06, 6, Analog),
            P0_7: (pio0_7, pio07, 7, Analog),
            P0_8: (pio0_8, pio08, 8, Analog),
            P0_9: (pio0_9, pio09, 9, Analog),
            // P0_10: (pio0_10, pio010, 10, Analog), TYPE A
            // P0_11: (pio0_11, pio011, 11, Analog), TYPE A
            // P0_12: (pio0_12, pio012, 12, Analog), TYPE A
            // P0_13: (pio0_13, pio013, 13, Analog), TYPE I
            // P0_14: (pio0_14, pio014, 14, Analog), TYPE I
            // P0_15: (pio0_15, pio015, 15, Analog), TYPE A
            // P0_16: (pio0_16, pio016, 16, Analog), TYPE A
            P0_17: (pio0_17, pio017, 17, Analog),
            P0_18: (pio0_18, pio018, 18, Analog),
            P0_19: (pio0_19, pio019, 19, Analog),
            P0_20: (pio0_20, pio020, 20, Analog),
            P0_21: (pio0_21, pio021, 21, Analog),
            P0_22: (pio0_22, pio022, 22, Analog),
            // P0_23: (pio0_23, pio023, 23, Analog), TYPE A
            P0_24: (pio0_24, pio024, 24, Analog),
            P0_25: (pio0_25, pio025, 25, Analog),
            P0_26: (pio0_26, pio026, 26, Analog),
            P0_27: (pio0_27, pio027, 27, Analog),
            P0_28: (pio0_28, pio028, 28, Analog),
            P0_29: (pio0_29, pio029, 29, Analog),
            P0_30: (pio0_30, pio030, 30, Analog),
            // P0_31: (pio0_31, pio031, 31, Analog), TYPE A
        ],
        P1, 1, [
            // P1_0: (pio1_0, pio10, 0, Analog), TYPE A
            P1_1: (pio1_1, pio11, 1, Analog),
            P1_2: (pio1_2, pio12, 2, Analog),
            P1_3: (pio1_3, pio13, 3, Analog),
            P1_4: (pio1_4, pio14, 4, Analog),
            P1_5: (pio1_5, pio15, 5, Analog),
            P1_6: (pio1_6, pio16, 6, Analog),
            P1_7: (pio1_7, pio17, 7, Analog),
            P1_8: (pio1_8, pio18, 8, Analog),
            P1_9: (pio1_9, pio19, 9, Analog),
            P1_10: (pio1_10, pio110, 10, Analog),
            P1_11: (pio1_11, pio111, 11, Analog),
            P1_12: (pio1_12, pio112, 12, Analog),
            P1_13: (pio1_13, pio113, 13, Analog),
            P1_14: (pio1_14, pio114, 14, Analog),
            P1_15: (pio1_15, pio115, 15, Analog),
            P1_16: (pio1_16, pio116, 16, Analog),
            P1_17: (pio1_17, pio117, 17, Analog),
            P1_18: (pio1_18, pio118, 18, Analog),
            P1_19: (pio1_19, pio119, 19, Analog),
            P1_20: (pio1_20, pio120, 20, Analog),
            P1_21: (pio1_21, pio121, 21, Analog),
            P1_22: (pio1_22, pio122, 22, Analog),
            P1_23: (pio1_23, pio123, 23, Analog),
            P1_24: (pio1_24, pio124, 24, Analog),
            P1_25: (pio1_25, pio125, 25, Analog),
            P1_26: (pio1_26, pio126, 26, Analog),
            P1_27: (pio1_27, pio127, 27, Analog),
            P1_28: (pio1_28, pio128, 28, Analog),
            P1_29: (pio1_29, pio129, 29, Analog),
            P1_30: (pio1_30, pio130, 30, Analog),
            P1_31: (pio1_31, pio131, 31, Analog),
        ],
        P2, 2, [
            // P2_0: (pio2_0, pio20, 0, Analog), TYPE A
            // P2_1: (pio2_1, pio21, 1, Analog), TYPE A
            P2_2: (pio2_2, pio22, 2, Analog),
            P2_3: (pio2_3, pio23, 3, Analog),
            P2_4: (pio2_4, pio24, 4, Analog),
            P2_5: (pio2_5, pio25, 5, Analog),
            P2_6: (pio2_6, pio26, 6, Analog),
            P2_7: (pio2_7, pio27, 7, Analog),
            P2_8: (pio2_8, pio28, 8, Analog),
            P2_9: (pio2_9, pio29, 9, Analog),
            P2_10: (pio2_10, pio210, 10, Analog),
            P2_11: (pio2_11, pio211, 11, Analog),
            P2_12: (pio2_12, pio212, 12, Analog),
            P2_13: (pio2_13, pio213, 13, Analog),
            P2_14: (pio2_14, pio214, 14, Analog),
            P2_15: (pio2_15, pio215, 15, Analog),
            P2_16: (pio2_16, pio216, 16, Analog),
            P2_17: (pio2_17, pio217, 17, Analog),
            P2_18: (pio2_18, pio218, 18, Analog),
            P2_19: (pio2_19, pio219, 19, Analog),
            P2_20: (pio2_20, pio220, 20, Analog),
            P2_21: (pio2_21, pio221, 21, Analog),
            P2_22: (pio2_22, pio222, 22, Analog),
            P2_23: (pio2_23, pio223, 23, Analog),
            P2_24: (pio2_24, pio224, 24, Analog),
            P2_25: (pio2_25, pio225, 25, Analog),
            P2_26: (pio2_26, pio226, 26, Analog),
            P2_27: (pio2_27, pio227, 27, Analog),
            P2_28: (pio2_28, pio228, 28, Analog),
            P2_29: (pio2_29, pio229, 29, Analog),
            P2_30: (pio2_30, pio230, 30, Analog),
            P2_31: (pio2_31, pio231, 31, Analog),
        ],
        P3, 3, [
            P3_0: (pio3_0, pio30, 0, Analog),
            P3_1: (pio3_1, pio31, 1, Analog),
            P3_2: (pio3_2, pio32, 2, Analog),
            P3_3: (pio3_3, pio33, 3, Analog),
            P3_4: (pio3_4, pio34, 4, Analog),
            P3_5: (pio3_5, pio35, 5, Analog),
            P3_6: (pio3_6, pio36, 6, Analog),
            P3_7: (pio3_7, pio37, 7, Analog),
            P3_8: (pio3_8, pio38, 8, Analog),
            P3_9: (pio3_9, pio39, 9, Analog),
            P3_10: (pio3_10, pio310, 10, Analog),
            P3_11: (pio3_11, pio311, 11, Analog),
            P3_12: (pio3_12, pio312, 12, Analog),
            P3_13: (pio3_13, pio313, 13, Analog),
            P3_14: (pio3_14, pio314, 14, Analog),
            P3_15: (pio3_15, pio315, 15, Analog),
            P3_16: (pio3_16, pio316, 16, Analog),
            P3_17: (pio3_17, pio317, 17, Analog),
            P3_18: (pio3_18, pio318, 18, Analog),
            P3_19: (pio3_19, pio319, 19, Analog),
            P3_20: (pio3_20, pio320, 20, Analog),
            // P3_21: (pio3_21, pio321, 21, Analog), TYPE A
            // P3_22: (pio3_22, pio322, 22, Analog), TYPE A
            // P3_23: (pio3_23, pio323, 23, Analog), TYPE I
            // P3_24: (pio3_24, pio324, 24, Analog), TYPE I
            P3_25: (pio3_25, pio325, 25, Analog),
            P3_26: (pio3_26, pio326, 26, Analog),
            P3_27: (pio3_27, pio327, 27, Analog),
            P3_28: (pio3_28, pio328, 28, Analog),
            P3_29: (pio3_29, pio329, 29, Analog),
            P3_30: (pio3_30, pio330, 30, Analog),
            P3_31: (pio3_31, pio331, 31, Analog),
        ],
        P4, 4, [
            P4_0: (pio4_0, pio40, 0, Analog),
            P4_1: (pio4_1, pio41, 1, Analog),
            P4_2: (pio4_2, pio42, 2, Analog),
            P4_3: (pio4_3, pio43, 3, Analog),
            P4_4: (pio4_4, pio44, 4, Analog),
            P4_5: (pio4_5, pio45, 5, Analog),
            P4_6: (pio4_6, pio46, 6, Analog),
            P4_7: (pio4_7, pio47, 7, Analog),
            P4_8: (pio4_8, pio48, 8, Analog),
            P4_9: (pio4_9, pio49, 9, Analog),
            P4_10: (pio4_10, pio410, 10, Analog),
            P4_11: (pio4_11, pio411, 11, Analog),
            P4_12: (pio4_12, pio412, 12, Analog),
            P4_13: (pio4_13, pio413, 13, Analog),
            P4_14: (pio4_14, pio414, 14, Analog),
            P4_15: (pio4_15, pio415, 15, Analog),
            P4_16: (pio4_16, pio416, 16, Analog),
            P4_17: (pio4_17, pio417, 17, Analog),
            P4_18: (pio4_18, pio418, 18, Analog),
            P4_19: (pio4_19, pio419, 19, Analog),
            P4_20: (pio4_20, pio420, 20, Analog),
            P4_21: (pio4_21, pio421, 21, Analog),
            P4_22: (pio4_22, pio422, 22, Analog),
            P4_23: (pio4_23, pio423, 23, Analog),
            P4_24: (pio4_24, pio424, 24, Analog),
            P4_25: (pio4_25, pio425, 25, Analog),
            P4_26: (pio4_26, pio426, 26, Analog),
            P4_27: (pio4_27, pio427, 27, Analog),
            P4_28: (pio4_28, pio428, 28, Analog),
            P4_29: (pio4_29, pio429, 29, Analog),
            P4_30: (pio4_30, pio430, 30, Analog),
            P4_31: (pio4_31, pio431, 31, Analog),
        ],
        P5, 5, [
            P5_0: (pio5_0, pio50, 0, Analog),
            P5_1: (pio5_1, pio51, 1, Analog),
            P5_2: (pio5_2, pio52, 2, Analog),
            P5_3: (pio5_3, pio53, 3, Analog),
            P5_4: (pio5_4, pio54, 4, Analog),
            P5_5: (pio5_5, pio55, 5, Analog),
            P5_6: (pio5_6, pio56, 6, Analog),
            P5_7: (pio5_7, pio57, 7, Analog),
            P5_8: (pio5_8, pio58, 8, Analog),
            P5_9: (pio5_9, pio59, 9, Analog),
            P5_10: (pio5_10, pio510, 10, Analog),
        ],
    ]
);

