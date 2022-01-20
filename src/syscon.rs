//! System configuration
//!
//!
//!
use crate::pac::*;
use embedded_time::rate::{Extensions, Hertz};

// System clock mux source
#[derive(Clone, Copy)]
pub enum MainClkSelA {
    fro_12m,
    clk_in(Hertz),
    wdt_clk(WdtClkDivSel, WdtClkFreqSel),
    fro_hf(FroOsc),
}

#[derive(Clone, Copy)]
pub struct WdtClkDivSel {
    divsel: u8, // output is divided by (divsel+1)*2
}

#[derive(Clone, Copy)]
pub struct WdtClkFreqSel {
    freqsel: u8, // cant math it
}

#[derive(Clone, Copy)]
pub enum FroOsc {
    Fro48Mhz,
    Fro96Mhz,
}
#[derive(Clone, Copy)]
pub enum MainClkSelB {
    mainclka,
    pll_clk,
    clk_32k,
}

#[derive(Clone, Copy)]
pub enum AHBClkDiv {
    NotDivided,
    DividedBy(u8),
}

pub struct Syscon {
    pub(crate) rb: SYSCON,
    pub clocks: Clocks,
}
impl Syscon {
    // TODO: relocate
    pub fn rev_id(&self) -> u32 {
        self.rb.device_id1.read().revid().bits()
    }
    pub fn part_id(&self) -> u32 {
        self.rb.device_id0.read().partid().bits()
    }
    pub fn get_main_clock_freq(self) -> Hertz {
        self.clocks.main_clk
    }
}
/// The main API for the SYSCON peripheral
impl Syscon {
    /// Enables the clock for a peripheral or other hardware component
    pub fn enable_clock<P: ClockControl>(&mut self, peripheral: &mut P) {
        peripheral.enable_clock(self);
    }

    /// Disable peripheral clock
    pub fn disable_clock<P: ClockControl>(&mut self, peripheral: &mut P) {
        peripheral.disable_clock(self);
    }

    /// Check if peripheral clock is enabled
    pub fn is_clock_enabled<P: ClockControl>(&self, peripheral: &P) -> bool {
        peripheral.is_clock_enabled(&self)
    }

    /// Reset a peripheral
    pub fn reset<P: ResetControl>(&mut self, peripheral: &mut P) {
        peripheral.assert_reset(self);
        peripheral.clear_reset(self);
    }
}

/// Internal trait for controlling peripheral clocks
///
/// This trait is an internal implementation detail and should neither be
/// implemented nor used outside of LPC82x HAL. Any changes to this trait won't
/// be considered breaking changes.
///
/// Compared to <https://git.io/fjpf9> (in lpc-rs/lpc8xx-hal/lpc8xx-hal-common)
/// we use a less minimal API in order to hide the fact that there are three
/// different AHLBCKLCTRL?, which a HAL user shouldn't really need to know about.
pub trait ClockControl {
    /// Internal method to enable a peripheral clock
    fn enable_clock(&self, s: &mut Syscon);

    /// Internal method to disable a peripheral clock
    fn disable_clock(&self, s: &mut Syscon);

    /// Check if peripheral clock is enabled
    fn is_clock_enabled(&self, s: &Syscon) -> bool;
}

macro_rules! impl_clock_control {
    ($clock_control:ty, $clock:ident, $register:ident) => {
        impl ClockControl for $clock_control {
            fn enable_clock(&self, s: &mut Syscon) {
                s.rb.$register.modify(|_, w| w.$clock().set_bit());
                while !s.rb.$register.read().$clock().bit() {}
            }

            fn disable_clock(&self, s: &mut Syscon) {
                s.rb.$register.modify(|_, w| w.$clock().clear_bit());
                while s.rb.$register.read().$clock().bit() {}
            }

            fn is_clock_enabled(&self, s: &Syscon) -> bool {
                s.rb.$register.read().$clock().bit()
            }
        }
    };

    ($clock_control:ty, $clock1:ident, $clock2:ident, $register:ident) => {
        impl ClockControl for $clock_control {
            fn enable_clock(&self, s: &mut Syscon) {
                s.rb.$register.modify(|_, w| w.$clock1().enable());
                s.rb.$register.modify(|_, w| w.$clock2().enable());
                while s.rb.$register.read().$clock1().is_disable() {}
                while s.rb.$register.read().$clock2().is_disable() {}
            }

            fn disable_clock(&self, s: &mut Syscon) {
                s.rb.$register.modify(|_, w| w.$clock1().disable());
                s.raw.$register.modify(|_, w| w.$clock2().disable());
            }

            fn is_clock_enabled(&self, s: &Syscon) -> bool {
                s.rb.$register.read().$clock1().is_enable()
                    && s.raw.$register.read().$clock2().is_enable()
            }
        }
    };
}

// impl_clock_control!(ADC0, adc, ahbclkctrl0);
// impl_clock_control!(CTIMER0, timer0, ahbclkctrl1);
// impl_clock_control!(CTIMER1, timer1, ahbclkctrl1);
// impl_clock_control!(CTIMER2, timer2, ahbclkctrl1);
// impl_clock_control!(CTIMER3, timer3, ahbclkctrl2);
// impl_clock_control!(CTIMER4, timer4, ahbclkctrl2);
// impl_clock_control!(DMA0, dma0, ahbclkctrl0);
// impl_clock_control!(FLEXCOMM0, fc0, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM1, fc1, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM2, fc2, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM3, fc3, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM4, fc4, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM5, fc5, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM6, fc6, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM7, fc7, ahbclkctrl1);
// impl_clock_control!(FLEXCOMM8, hs_lspi, ahbclkctrl2);
// impl_clock_control!(INPUTMUX, mux, ahbclkctrl0);
impl_clock_control!(IOCON, iocon, ahbclkctrl0);
// impl_clock_control!((&mut GINT0, &mut GINT1), gint, ahbclkctrl0);
// impl_clock_control!(PINT, pint, ahbclkctrl0);

// impl_clock_control!(USB0, usb0_dev, ahbclkctrl1);
// impl_clock_control!(USBFSH, usb0_hosts, ahbclkctrl2); // well what about usb0_hostm?
// impl_clock_control!(USBHSH, usb1_host, ahbclkctrl2);
// impl_clock_control!(UTICK0, utick, ahbclkctrl1);

// impl_clock_control!(RTC, rtc, ahbclkctrl0);

impl ClockControl for GPIO {
    fn enable_clock(&self, s: &mut Syscon) {
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio0().set_bit());
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio1().set_bit());
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio2().set_bit());
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio3().set_bit());
        s.rb.ahbclkctrl2.modify(|_, w| w.gpio4().set_bit());
        s.rb.ahbclkctrl2.modify(|_, w| w.gpio5().set_bit());
    }

    fn disable_clock(&self, s: &mut Syscon) {
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio0().clear_bit());
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio1().clear_bit());
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio2().clear_bit());
        s.rb.ahbclkctrl0.modify(|_, w| w.gpio3().clear_bit());
        s.rb.ahbclkctrl2.modify(|_, w| w.gpio4().clear_bit());
        s.rb.ahbclkctrl2.modify(|_, w| w.gpio5().clear_bit());
    }

    #[allow(clippy::nonminimal_bool)]
    fn is_clock_enabled(&self, s: &Syscon) -> bool {
        s.rb.ahbclkctrl0.read().gpio0().bit()
            && s.rb.ahbclkctrl0.read().gpio1().bit()
            && s.rb.ahbclkctrl0.read().gpio2().bit()
            && s.rb.ahbclkctrl0.read().gpio3().bit()
            && s.rb.ahbclkctrl2.read().gpio4().bit()
            && s.rb.ahbclkctrl2.read().gpio5().bit()
    }
}

pub trait ResetControl {
    /// Internal method to assert peripheral reset
    fn assert_reset(&self, syscon: &mut Syscon);

    /// Internal method to clear peripheral reset
    fn clear_reset(&self, syscon: &mut Syscon);
}

impl ResetControl for GPIO {
    fn assert_reset(&self, syscon: &mut Syscon) {
        syscon.rb.presetctrl0.modify(|_, w| w.gpio0_rst().set_bit())
    }

    fn clear_reset(&self, syscon: &mut Syscon) {
        syscon
            .rb
            .presetctrl0
            .modify(|_, w| w.gpio0_rst().clear_bit())
    }
}

/// Extension trait that freezes the `RCC` peripheral with provided clocks configuration
pub trait SysconExt {
    fn freeze(self, cfgr: Config) -> Syscon;
}

impl SysconExt for SYSCON {
    fn freeze(self, cfgr: Config) -> Syscon {
        let sys_clk = match cfgr.mainclksela {
            MainClkSelA::fro_12m => {
                // select free running oscillator 12mhz as clock source a
                self.mainclksela.modify(|_, w| w.sel().fro_12_mhz());
                12_000_000.Hz()
            }
            MainClkSelA::clk_in(freq) => {
                // select clock input
                self.mainclksela.modify(|_, w| w.sel().clkin());
                freq
            }
            MainClkSelA::wdt_clk(divsel, freqsel) => {
                self.mainclksela
                    .modify(|_, w| w.sel().watchdog_oscillator());
                unsafe {
                    self.wdtoscctrl
                        .modify(|_, w| w.divsel().bits(divsel.divsel as u8));
                    self.wdtoscctrl
                        .modify(|_, w| w.freqsel().bits(freqsel.freqsel as u8));
                }
                let freq_in = match freqsel.freqsel {
                    // ref p. 137 UM10912 rev 2.4
                    0x01 => 400_000,
                    0x02 => 600_000,
                    0x03 => 750_000,
                    0x04 => 900_000,
                    0x05 => 1_000_000,
                    0x06 => 1_200_000,
                    0x07 => 1_300_000,
                    0x08 => 1_400_000,
                    0x09 => 1_500_000,
                    0x0A => 1_600_000,
                    0x0B => 1_700_000,
                    0x0C => 1_800_000,
                    0x0D => 1_900_000,
                    0x0E => 2_000_000,
                    0x0F => 2_050_000,
                    0x10 => 2_100_000,
                    0x11 => 2_200_000,
                    0x12 => 2_250_000,
                    0x13 => 2_300_000,
                    0x14 => 2_400_000,
                    0x15 => 2_450_000,
                    0x16 => 2_500_000,
                    0x17 => 2_600_000,
                    0x18 => 2_650_000,
                    0x19 => 2_700_000,
                    0x1A => 2_800_000,
                    0x1B => 2_850_000,
                    0x1C => 2_900_000,
                    0x1D => 2_950_000,
                    0x1E => 3_000_000,
                    0x1F => 3_050_000,
                    _ => unreachable!(),
                };
                let freq_out = freq_in / ((divsel.divsel as u32 + 1) * 2);
                freq_out.Hz()
            }
            MainClkSelA::fro_hf(fro) => {
                self.mainclksela.modify(|_, w| w.sel().fro_hf());
                match fro {
                    FroOsc::Fro48Mhz => 48_000_000.Hz(),
                    FroOsc::Fro96Mhz => 96_000_000.Hz(),
                }
            }
        };

        let clocks = Clocks {
            source: cfgr.mainclksela,
            main_clk: sys_clk,
        };
        Syscon { rb: self, clocks }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct Clocks {
    source: MainClkSelA,
    main_clk: Hertz,
}

impl Clocks {
    /// return clock source
    pub fn source(&self) -> &MainClkSelA {
        &self.source
    }
}

#[allow(dead_code)]
pub struct Config {
    mainclksela: MainClkSelA,
    mainclkselb: MainClkSelB,
    ahbclkdiv: AHBClkDiv,
}

impl Default for Config {
    #[inline]
    fn default() -> Config {
        Config {
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
        }
    }
}

impl Config {
    #[inline]
    pub fn fro12m() -> Config {
        Config {
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
        }
    }
}
