//! Flexible Communication Interface
//!

use crate::pac::syscon::fclksel::SEL_A;

use crate::pac::{
    FLEXCOMM0, FLEXCOMM1, FLEXCOMM2, FLEXCOMM3, FLEXCOMM4, FLEXCOMM5, FLEXCOMM6, FLEXCOMM7,
    FLEXCOMM8, I2C0, I2C1, I2C2, I2C3, I2C4, I2C5, I2C6, I2C7, I2C8, USART0, USART1, USART2,
    USART3, USART4, USART5, USART6, USART7, USART8,
};
#[cfg(feature = "flexcomm-10")]
use crate::pac::{FLEXCOMM9, I2C9, USART9};

use crate::syscon::Syscon;

/// Flexcomm peripheral source clock
pub enum FlexcommClockSource {
    /// Free Running Osc 12 Mhz
    fro_12_mhz,
    /// Free Running Osc High Frequency Divided
    fro_hf_div,
    /// Audio PLL
    audio_pll_clk,
    /// Master clock
    mck,
    /// Fractional Rate Generator
    frg,
    /// No input clock
    none,
}

/// Trait for Flexcomm used as an USART
pub trait FlexCommAsUart<USART> {
    /// Configure this Flexcomm as an USART
    fn as_usart(&self);

    /// checks if this Flexcomm is capable of being used as an USART
    fn is_usart_capable(&self) -> bool;
}

macro_rules! impl_flexcomm_as_usart {
    ($($FLEXCOMMX:ident, $USARTX:ident;)*) => {
        $(
            impl FlexCommAsUart<$USARTX> for $FLEXCOMMX {
                fn as_usart(&self) {
                    self.pselid.modify(|_, w| w.persel().usart());
                }
                fn is_usart_capable(&self) -> bool {
                    self.pselid.read().usartpresent().is_present()
                }
            }
        )*
    }
}

impl_flexcomm_as_usart!(
    FLEXCOMM0, USART0;
    FLEXCOMM1, USART1;
    FLEXCOMM2, USART2;
    FLEXCOMM3, USART3;
    FLEXCOMM4, USART4;
    FLEXCOMM5, USART5;
    FLEXCOMM6, USART6;
    FLEXCOMM7, USART7;
    FLEXCOMM8, USART8;
);

#[cfg(feature = "flexcomm-10")]
impl_flexcomm_as_usart!(
    FLEXCOMM9, USART9;
);

/// Trait for Flexcomm used as a I2C interface
pub trait FlexCommAsI2C<I2C> {
    /// Configure this Flexcomm as a I2C Interface
    fn as_i2c(&self);

    /// checks if this Flexcomm is capable of being used as an I2C interface
    fn is_i2c_capable(&self) -> bool;
}

macro_rules! impl_flexcomm_as_i2c {
    ($($FLEXCOMMX:ident, $I2CX:ident;)*) => {
        $(
            impl FlexCommAsI2C<$I2CX> for $FLEXCOMMX {
                fn as_i2c(&self) {
                    self.pselid.modify(|_, w| w.persel().i2c());
                }
                fn is_i2c_capable(&self) -> bool {
                    self.pselid.read().i2cpresent().is_present()
                }
            }
        )*
    }
}

impl_flexcomm_as_i2c!(
    FLEXCOMM0, I2C0;
    FLEXCOMM1, I2C1;
    FLEXCOMM2, I2C2;
    FLEXCOMM3, I2C3;
    FLEXCOMM4, I2C4;
    FLEXCOMM5, I2C5;
    FLEXCOMM6, I2C6;
    FLEXCOMM7, I2C7;
    FLEXCOMM8, I2C8;
);

#[cfg(feature = "flexcomm-10")]
impl_flexcomm_as_i2c!(
    FLEXCOMM9, I2C9;
);

/// trait usefull for Specific Flexcomm implementation (USART, I2C, etc.)
pub trait FlexcommClockControl {
    /// configures the input clock for this Flexcomm peripheral in SYSCON
    fn set_clock(&self, clock: FlexcommClockSource, s: &mut Syscon);
}

/// Macro to implement the `set_clock` function for a Flexcomm interface
macro_rules! impl_flexcomm_clock_control {
    ($($FLEXCOMMX:ident, $fclkselid:expr;)*) => {
        $(
            impl FlexcommClockControl for $FLEXCOMMX {
                fn set_clock(&self, clock: FlexcommClockSource, s: &mut Syscon) {
                    s.rb.fclksel[$fclkselid].write(|w| {
                        w.sel().variant(match clock {
                            FlexcommClockSource::fro_12_mhz => SEL_A::FRO_12_MHZ,
                            FlexcommClockSource::fro_hf_div => SEL_A::FRO_HF_DIV,
                            FlexcommClockSource::audio_pll_clk => SEL_A::AUDIO_PLL_OUTPUT,
                            FlexcommClockSource::mck => SEL_A::MCLK_INPUT,
                            FlexcommClockSource::frg => SEL_A::FRG_CLOCK_OUTPUT,
                            FlexcommClockSource::none => SEL_A::NONE,
                        })
                    });
                }
            }
        )*
    }
}

impl_flexcomm_clock_control!(
    FLEXCOMM0, 0;
    FLEXCOMM1, 1;
    FLEXCOMM2, 2;
    FLEXCOMM3, 3;
    FLEXCOMM4, 4;
    FLEXCOMM5, 5;
    FLEXCOMM6, 6;
    FLEXCOMM7, 7;
    FLEXCOMM8, 8;
);

#[cfg(feature = "flexcomm-10")]
impl_flexcomm_clock_control!(
    FLEXCOMM9, 9;
);
