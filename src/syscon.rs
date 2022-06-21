//! System configuration
//!
//! This module deals with clocks and resets.
//! Implementation status:
//! [x] fro12m
//! [~] external
//! [~] PLL
//!
//! All readings are done trough option, with None propagating trough
//! the clock tree. If the return of a get_xxx_freq returns a None,
//! it means that the corresponding clock is not configured correctly.
//!
use crate::pac::*;
use embedded_time::rate::{Extensions, Hertz};

/// System clock mux source
#[derive(Clone, Copy)]
pub enum MainClkSelA {
    /// free running oscillator 12 MHz
    fro_12m,
    /// external clock input
    clk_in,
    /// watchdog timer
    wdt_clk(WdtClkDivSel, WdtClkFreqSel),
    /// free running oscillator high frequency (48/96 MHz)
    fro_hf(FroHfOsc),
}

/// Watchdog clock divider select
#[derive(Clone, Copy)]
pub struct WdtClkDivSel {
    /// Divider selector. output is divided by (divsel+1)*2
    divsel: u8,
}

/// Watchdog clock freq select
#[derive(Clone, Copy)]
pub struct WdtClkFreqSel {
    /// Frequency select. cannot math it out
    freqsel: u8,
}

/// Free runing oscillator High Frequency
#[derive(Clone, Copy)]
pub enum FroHfOsc {
    /// Free running oscillator at 48MHz
    Fro48Mhz,
    /// Free running oscillator at 96MHz
    Fro96Mhz,
}

/// Main clock selector B
#[derive(Clone, Copy)]
pub enum MainClkSelB {
    /// source is main clock A
    mainclka,
    /// source is PLL Clock
    pll_clk,
    /// source is 32768 Hz (rtc) clock
    clk_32k,
}

/// AHB clock divider
#[derive(Clone, Copy)]
pub enum AHBClkDiv {
    /// Clock is not divided
    NotDivided,
    /// Clock is divided by u8
    DividedBy(u8),
}

/// Audio Pll Configuration
pub struct AudioPllConfig {
    /// clock source selection
    pub clksel: AudioPllClkSel,
    /// ndec value, generated with NXP Config Tools
    pub ndec: u16,
    /// mdec value, generated with NXP Config Tools
    pub mdec: u32,
    /// bypass the pdiv divider
    pub bypass_pdiv: bool,
    /// pdec value, generated with NXP Config Tools
    pub pdec: u8,
    /// pll rate, from NXP Config Tools
    pub pllrate: Hertz,
}
/// Audio PLL clock selection
pub enum AudioPllClkSel {
    /// clock input
    clk_in,
    /// free running oscillator 12 MHz
    fro_12m,
    /// no input
    none,
}

/// module singleton.
pub struct Syscon {
    /// SYSCON register block
    pub(crate) rb: SYSCON,
    /// Clocks
    pub clocks: Clocks,
}
impl Syscon {
    // TODO: relocate
    /// Get revision ID
    pub fn rev_id(&self) -> u32 {
        self.rb.device_id1.read().revid().bits()
    }

    /// get Part ID
    pub fn part_id(&self) -> u32 {
        self.rb.device_id0.read().partid().bits()
    }

    /// Get main clock frequency depending on syscon configuration.
    pub fn get_main_clock_freq(&self) -> Option<Hertz> {
        match self.rb.mainclkselb.read().sel().variant().unwrap() {
            syscon::mainclkselb::SEL_A::MAINCLKSELA => {
                match self.rb.mainclksela.read().sel().variant() {
                    syscon::mainclksela::SEL_A::FRO_12_MHZ => self.get_fro_12m_clock_freq(),
                    syscon::mainclksela::SEL_A::CLKIN => self.get_clock_xtal_in_freq(),
                    syscon::mainclksela::SEL_A::WATCHDOG_OSCILLATOR => {
                        self.get_wdt_clk_clock_freq()
                    }
                    syscon::mainclksela::SEL_A::FRO_HF => self.get_fro_hf_clock_freq(),
                }
            }
            syscon::mainclkselb::SEL_A::SYSTEM_PLL_OUTPUT => self.get_syspll_clock_clock_freq(),
            syscon::mainclkselb::SEL_A::RTC_OSC_OUTPUT => self.get_clk_32k_clock_freq(),
        }
    }

    /// Get crystal input frequency (user supplied)
    pub fn get_clock_xtal_in_freq(&self) -> Option<Hertz> {
        self.clocks.clock_in
    }

    /// Get 32768 clock frequency
    pub fn get_clk_32k_clock_freq(&self) -> Option<Hertz> {
        self.clocks.rtc_in.map(|_| (32 * 1024).Hz())
    }

    /// Get System pll clock frequency
    pub fn get_syspll_clock_clock_freq(&self) -> Option<Hertz> {
        let _input_freq = match self.rb.syspllclksel.read().sel().variant().unwrap() {
            syscon::syspllclksel::SEL_A::FRO_12_MHZ => self.get_fro_12m_clock_freq(),
            syscon::syspllclksel::SEL_A::CLKIN => self.get_clock_xtal_in_freq(),
            syscon::syspllclksel::SEL_A::WATCHDOG_OSCILLATOR => self.get_wdt_clk_clock_freq(),
            syscon::syspllclksel::SEL_A::RTC_OSC_OUTPUT => self.get_clk_32k_clock_freq(),
            syscon::syspllclksel::SEL_A::NONE => return None,
        };
        // TODO:
        None
    }

    /// Get usb PLL clock clock freq.
    pub fn get_usbpll_clock_clock_freq(&self) -> Option<Hertz> {
        todo!()
    }

    /// Get Free running oscillator high frequency frequency
    pub fn get_fro_hf_clock_freq(&self) -> Option<Hertz> {
        match self.rb.froctrl.read().sel().bit() {
            false => Some(48_000_000.Hz()),
            true => Some(96_000_000.Hz()),
        }
    }

    /// Get Free running oscillator high frequency divided frequency
    pub fn get_fro_hf_div_clock_freq(&self) -> Option<Hertz> {
        let div = self.rb.frohfclkdiv.read().div().bits() as u32;
        self.get_fro_hf_clock_freq().map(|freq| freq / (div + 1))
    }

    /// Get audio pll clock frequency
    pub fn get_audio_pll_clk_clock_freq(&self) -> Option<Hertz> {
        self.clocks.audio_pll
    }

    /// Get Mclk frequency (user supplied)
    pub fn get_mclk_in_clock_freq(&self) -> Option<Hertz> {
        self.clocks.mclkin
    }

    /// Get Fractional Rate Generator clock frequency
    pub fn get_frg_clk_clock_freq(&self) -> Option<Hertz> {
        match self.rb.frgclksel.read().sel().variant() {
            Some(x) => match x {
                syscon::frgclksel::SEL_A::MAIN_CLOCK => self.get_main_clock_freq(),
                syscon::frgclksel::SEL_A::SYSTEM_PLL_OUTPUT => self.get_syspll_clock_clock_freq(),
                syscon::frgclksel::SEL_A::FRO_12_MHZ => self.get_fro_12m_clock_freq(),
                syscon::frgclksel::SEL_A::FRO_HF => self.get_fro_hf_clock_freq(),
                syscon::frgclksel::SEL_A::NONE => None,
            },
            None => None,
        }
    }

    /// Get Free running oscillator 12MHz frequency (constant)
    pub fn get_fro_12m_clock_freq(&self) -> Option<Hertz> {
        Some(12_000_000.Hz())
    }

    /// Get Watchdog timer clock frequency
    pub fn get_wdt_clk_clock_freq(&self) -> Option<Hertz> {
        match self.rb.pdruncfg0.read().pden_wdt_osc().bit() {
            true => {
                let wdt_freq: u32 = match self.rb.wdtoscctrl.read().freqsel().bits() {
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
                let wdt_div = ((self.rb.wdtoscctrl.read().divsel().bits() as u32) + 1) * 2;
                Some((wdt_freq / wdt_div).Hz())
            }
            false => None,
        }
    }

    /// Get system clock frequency
    pub fn get_system_clock_clock_freq(&self) -> Option<Hertz> {
        let ahbclkdiv = self.rb.ahbclkdiv.read().div().bits() as u32;
        self.get_main_clock_freq()
            .map(|freq| freq / (ahbclkdiv + 1))
    }

    /// Set the Free Running Oscillator frequency.
    ///
    /// Calls fro API that is in the BOOTROM.
    fn set_fro_frequency(&self, ifreq: u32) {
        // cannot do this any other way than unsafe
        let code: extern "C" fn(u32) = unsafe { core::mem::transmute(0x0300_91DF as *const ()) };
        (code)(ifreq);
    }

    /// Setup Free running oscillator clocking.
    fn clock_setup_froclocking(&mut self, ifreq: u32) {
        // the only allowed fro freq are 12MHz | 48MHz | 96MHz
        match ifreq {
            12_000_000 | 48_000_000 | 96_000_000 => (),
            _ => panic!(),
        }

        // Power up the FRO and set this as the base clock
        self.rb.pdruncfgclr0.modify(|_, w| w.pden_fro().set_bit());

        let usb_adj = self.rb.froctrl.read().usbclkadj().bit();

        match ifreq {
            12_000_000 => self.rb.froctrl.modify(|_, w| w.hspdclk().clear_bit()),
            x => {
                self.set_fro_frequency(x);
                self.rb.froctrl.modify(|_, w| {
                    w.sel()
                        .bit(match x {
                            48_000_000 => false,
                            96_000_000 => true,
                            _ => unreachable!(),
                        })
                        .wrtrim()
                        .set_bit()
                        .usbclkadj()
                        .bit(usb_adj)
                        .hspdclk()
                        .set_bit()
                })
            }
        }
    }
    fn clock_set_flashaccess_cycles_for_freq(&mut self, ifreq: u32) {
        let flash_cycles = match ifreq {
            0..=12_000_000 => 1,
            12_000_001..=24_000_000 => 2,
            24_000_001..=36_000_000 => 3,
            36_000_001..=60_000_000 => 4,
            60_000_001..=96_000_000 => 5,
            96_000_001..=120_000_000 => 6,
            120_000_001..=144_000_000 => 7,
            144_000_001..=168_000_000 => 8,
            _ => 9,
        };
        // safe because we keep in the same field, but not all possibles values are enumerated in SVD
        self.rb
            .flashcfg
            .modify(|_, w| unsafe { w.flashtim().bits(flash_cycles - 1) });
    }

    /// Set Voltage for frequency
    ///
    /// Power API, reverse engineering from libpower.a
    fn set_voltage_for_freq(&mut self, freq: u32) {
        let val = match freq {
            0..=100_000_000 => 0xb,
            100_000_001..=180_000_000 => 0xd,
            _ => 0xf,
        };
        unsafe {
            core::ptr::write_volatile(0x4002_000C as *mut u32, val);
            core::ptr::write_volatile(
                0x4002_0000 as *mut u32,
                core::ptr::read_volatile(0x4002_000C as *mut u32),
            );
            core::ptr::write_volatile(0x4002_0004 as *mut u32, 0xc);
            core::ptr::write_volatile(0x4002_0008 as *mut u32, 0xb);
            core::ptr::write_volatile(0x4002_0010 as *mut u32, 0xb);
            core::ptr::write_volatile(0x4002_0014 as *mut u32, 0xb);
        }
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
        peripheral.is_clock_enabled(self)
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

    /// get the peripheral's clock frequency
    fn get_clock_freq(&self, s: &Syscon) -> Option<Hertz>;
}

macro_rules! impl_clock_control {
    ($clock_control:ty, $clock:ident, $register:ident, $clock_getter: ident) => {
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
            fn get_clock_freq(&self, s: &Syscon) -> Option<Hertz> {
                $clock_getter(s)
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

            fn get_clock_freq(self, s: &Syscon) -> Option<Hertz> {
                0.Hz() // TODO
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
impl_clock_control!(CRC_ENGINE, crc, ahbclkctrl0, crc_get_clock_freq);

// the CRC engine runs at AHB bus speed (system clock)
fn crc_get_clock_freq(s: &Syscon) -> Option<Hertz> {
    s.get_system_clock_clock_freq()
}

impl_clock_control!(
    FLEXCOMM0,
    flexcomm0,
    ahbclkctrl1,
    flexcomm0_get_source_clock
);
impl_clock_control!(
    FLEXCOMM1,
    flexcomm1,
    ahbclkctrl1,
    flexcomm1_get_source_clock
);
impl_clock_control!(
    FLEXCOMM2,
    flexcomm2,
    ahbclkctrl1,
    flexcomm2_get_source_clock
);
impl_clock_control!(
    FLEXCOMM3,
    flexcomm3,
    ahbclkctrl1,
    flexcomm3_get_source_clock
);
impl_clock_control!(
    FLEXCOMM4,
    flexcomm4,
    ahbclkctrl1,
    flexcomm4_get_source_clock
);
impl_clock_control!(
    FLEXCOMM5,
    flexcomm5,
    ahbclkctrl1,
    flexcomm5_get_source_clock
);
impl_clock_control!(
    FLEXCOMM6,
    flexcomm6,
    ahbclkctrl1,
    flexcomm6_get_source_clock
);
impl_clock_control!(
    FLEXCOMM7,
    flexcomm7,
    ahbclkctrl1,
    flexcomm7_get_source_clock
);
impl_clock_control!(
    FLEXCOMM8,
    flexcomm8,
    ahbclkctrl2,
    flexcomm8_get_source_clock
);
#[cfg(feature = "flexcomm-10")]
impl_clock_control!(
    FLEXCOMM9,
    flexcomm9,
    ahbclkctrl2,
    flexcomm9_get_source_clock
);

// impl_clock_control!(INPUTMUX, mux, ahbclkctrl0);
impl_clock_control!(IOCON, iocon, ahbclkctrl0, main_clock_get_source_clock);
// impl_clock_control!((&mut GINT0, &mut GINT1), gint, ahbclkctrl0);
// impl_clock_control!(PINT, pint, ahbclkctrl0);

// impl_clock_control!(USBFSH, usb0_hosts, ahbclkctrl2); // well what about usb0_hostm?
// impl_clock_control!(USBHSH, usb1_host, ahbclkctrl2);
// impl_clock_control!(UTICK0, utick, ahbclkctrl1);

impl_clock_control!(RTC, rtc, ahbclkctrl0, rtc_get_source_clock);

fn rtc_get_source_clock(s: &Syscon) -> Option<Hertz> {
    s.get_clk_32k_clock_freq()
}

/// General system clock
fn main_clock_get_source_clock(s: &Syscon) -> Option<Hertz> {
    s.get_main_clock_freq()
}

macro_rules! impl_flexcomm_get_source_clock {
    ($($function_name:ident, $fclkselid:expr ;)*) => {
        $(
            fn $function_name(s: &Syscon) -> Option<Hertz> {
                let source = match s.rb.fclksel[$fclkselid].read().sel().variant().unwrap() {
                    syscon::fclksel::SEL_A::FRO_12_MHZ => s.get_fro_12m_clock_freq(),
                    syscon::fclksel::SEL_A::FRO_HF_DIV => s.get_fro_hf_div_clock_freq(),
                    syscon::fclksel::SEL_A::AUDIO_PLL_OUTPUT => s.get_audio_pll_clk_clock_freq(),
                    syscon::fclksel::SEL_A::MCLK_INPUT => s.get_mclk_in_clock_freq(),
                    syscon::fclksel::SEL_A::FRG_CLOCK_OUTPUT => s.get_frg_clk_clock_freq(),
                    syscon::fclksel::SEL_A::NONE => None,
                };
                source
            }
        )*
    }
}

impl_flexcomm_get_source_clock!(
    flexcomm0_get_source_clock, 0;
    flexcomm1_get_source_clock, 1;
    flexcomm2_get_source_clock, 2;
    flexcomm3_get_source_clock, 3;
    flexcomm4_get_source_clock, 4;
    flexcomm5_get_source_clock, 5;
    flexcomm6_get_source_clock, 6;
    flexcomm7_get_source_clock, 7;
    flexcomm8_get_source_clock, 8;
);
#[cfg(feature = "flexcomm-10")]
impl_flexcomm_get_source_clock!(
    flexcomm9_get_source_clock, 9;
);

/// Clock control for USB0 peripheral
impl ClockControl for USB0 {
    fn enable_clock(&self, s: &mut Syscon) {
        s.rb.ahbclkctrl1.modify(|_, w| w.usb0d().set_bit());
        s.rb.pdruncfg0.modify(|_, w| w.pden_usb0_phy().clear_bit())
    }

    fn disable_clock(&self, s: &mut Syscon) {
        s.rb.ahbclkctrl1.modify(|_, w| w.usb0d().clear_bit());
        s.rb.pdruncfg0.modify(|_, w| w.pden_usb0_phy().set_bit());
    }

    fn is_clock_enabled(&self, s: &Syscon) -> bool {
        s.rb.ahbclkctrl1.read().usb0d().bit()
    }

    fn get_clock_freq(&self, s: &Syscon) -> Option<Hertz> {
        let div = s.rb.usb0clkdiv.read().div().bits();
        let clk = match s.rb.usb0clksel.read().sel().variant() {
            Some(x) => match x {
                syscon::usb0clksel::SEL_A::FRO_HF => s.get_fro_hf_clock_freq(),
                syscon::usb0clksel::SEL_A::SYSTEM_PLL_OUTPUT => s.get_syspll_clock_clock_freq(),
                syscon::usb0clksel::SEL_A::USB_PLL_CLOCK => s.get_usbpll_clock_clock_freq(),
                syscon::usb0clksel::SEL_A::NONE => None,
            },
            None => None,
        };
        clk.map(|x| x / (div as u32 + 1))
    }
}

/// Clock Control for GPIO peripherals
///
/// ATM the  clock control is implemend for GPIO0/1/2/3/4/5 as a single peripheral.
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

    fn get_clock_freq(&self, s: &Syscon) -> Option<Hertz> {
        s.get_main_clock_freq()
    }
}

/// Reset Control trait for peripherals
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

/// Extension trait that freezes the `SYSCON` peripheral with provided clocks configuration
pub trait SysconExt {
    /// Freeze device clock tree according to config
    fn freeze(self, cfgr: Config) -> Syscon;
}

/// Implementation of the external trait for SYSCON
impl SysconExt for SYSCON {
    fn freeze(self, cfgr: Config) -> Syscon {
        let clocks = Clocks {
            clock_in: cfgr.xtal_freq,
            rtc_in: cfgr.rtc_32k_present,
            audio_pll: None,
            mclkin: None,
        };
        let mut syscon = Syscon { rb: self, clocks };
        // we need to watch out for loosing clock. rely on fro_12m for setting up and freezing.
        // ensure FRO is on
        syscon.rb.pdruncfgclr0.modify(|_, w| w.pden_fro().set_bit());
        // Switch to FRO 12MHz first to ensure we can change voltage without accidentally
        // being below the voltage for current speed
        syscon.rb.mainclksela.write(|w| w.sel().fro_12_mhz());
        syscon.rb.mainclkselb.write(|w| w.sel().mainclksela());

        // configure the correct main clock selector A
        match cfgr.mainclksela {
            MainClkSelA::fro_12m => {
                // select free running oscillator 12mhz as clock source a
                syscon.rb.mainclksela.modify(|_, w| w.sel().fro_12_mhz());
            }
            MainClkSelA::clk_in => {
                syscon
                    .rb
                    .pdruncfgclr1
                    .modify(|_, w| w.pden_sysosc().set_bit());
                match cfgr.xtal_freq {
                    Some(x) => match x.0 {
                        1_000_000..=20_000_000 => syscon
                            .rb
                            .sysoscctrl
                            .modify(|_, w| w.freqrange().clear_bit()),
                        20_000_001..=50_000_000 => {
                            syscon.rb.sysoscctrl.modify(|_, w| w.freqrange().set_bit())
                        }
                        _ => unreachable!(), // wrong configuration
                    },
                    None => unreachable!(), // if you want to use the external oscillator, you need to give a valid xtal frequency
                }
                syscon.rb.froctrl.modify(|_, w| w.hspdclk().clear_bit());
                // set voltage for freq
                let f = cfgr.xtal_freq.unwrap().0;
                syscon.clock_setup_froclocking(f);
                syscon.set_voltage_for_freq(f);
                syscon.clock_set_flashaccess_cycles_for_freq(f);
                // select clock input
                syscon.rb.mainclksela.modify(|_, w| w.sel().clkin());
            }
            MainClkSelA::wdt_clk(divsel, freqsel) => {
                syscon
                    .rb
                    .mainclksela
                    .modify(|_, w| w.sel().watchdog_oscillator());
                unsafe {
                    syscon
                        .rb
                        .wdtoscctrl
                        .modify(|_, w| w.divsel().bits(divsel.divsel as u8));
                    syscon
                        .rb
                        .wdtoscctrl
                        .modify(|_, w| w.freqsel().bits(freqsel.freqsel as u8));
                }
            }
            MainClkSelA::fro_hf(fro) => {
                let t_freq = match fro {
                    FroHfOsc::Fro48Mhz => 48_000_000,
                    FroHfOsc::Fro96Mhz => 96_000_000,
                };
                match fro {
                    FroHfOsc::Fro48Mhz => syscon.rb.froctrl.modify(|_, w| w.sel().clear_bit()),
                    FroHfOsc::Fro96Mhz => syscon.rb.froctrl.modify(|_, w| w.sel().set_bit()),
                }
                syscon.clock_setup_froclocking(t_freq);
                let f = syscon.get_fro_hf_clock_freq().unwrap().0;
                syscon.set_voltage_for_freq(f);
                syscon.clock_set_flashaccess_cycles_for_freq(f);
                syscon.rb.mainclksela.modify(|_, w| w.sel().fro_hf());
            }
        };
        match cfgr.mainclkselb {
            MainClkSelB::mainclka => syscon.rb.mainclkselb.modify(|_, w| w.sel().mainclksela()),
            MainClkSelB::pll_clk => syscon
                .rb
                .mainclkselb
                .modify(|_, w| w.sel().system_pll_output()),
            MainClkSelB::clk_32k => syscon
                .rb
                .mainclkselb
                .modify(|_, w| w.sel().rtc_osc_output()),
        }

        if let Some(config) = cfgr.audiopll {
            match config.clksel {
                AudioPllClkSel::clk_in => syscon.rb.audpllclksel.modify(|_, w| w.sel().clkin()),
                AudioPllClkSel::fro_12m => {
                    syscon.rb.audpllclksel.modify(|_, w| w.sel().fro_12_mhz())
                }
                AudioPllClkSel::none => syscon.rb.audpllclksel.modify(|_, w| w.sel().none()),
            }
            syscon
                .rb
                .audpllndec
                .write(|w| unsafe { w.ndec().bits(config.ndec) });
            syscon.rb.audpllndec.modify(|_, w| w.nreq().set_bit());

            syscon
                .rb
                .audpllpdec
                .write(|w| unsafe { w.pdec().bits(config.pdec) });
            syscon.rb.audpllpdec.modify(|_, w| w.preq().set_bit());

            syscon
                .rb
                .audpllmdec
                .write(|w| unsafe { w.mdec().bits(config.mdec) });
            syscon.rb.audpllmdec.modify(|_, w| w.mreq().set_bit());
            syscon.clocks.audio_pll = Some(config.pllrate)
        }

        let ahbdivider = match cfgr.ahbclkdiv {
            AHBClkDiv::NotDivided => 1,
            AHBClkDiv::DividedBy(x) => x,
        };

        // safe because other fields are read-only?
        syscon
            .rb
            .ahbclkdiv
            .write(|w| unsafe { w.div().bits(ahbdivider - 1) });
        syscon
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct Clocks {
    /// External clock input
    clock_in: Option<Hertz>,
    /// RTC clock input (present or not, can only be 32768 Hz)
    rtc_in: Option<()>,
    /// Audio PLL clock
    audio_pll: Option<Hertz>,
    /// Mclk clock
    mclkin: Option<Hertz>,
}

/// Syscon Config structure
#[allow(dead_code)]
pub struct Config {
    /// crystal freq
    pub xtal_freq: Option<Hertz>,
    /// RTC xtal present
    pub rtc_32k_present: Option<()>,
    /// main clock A selection
    pub mainclksela: MainClkSelA,
    /// main clock B selection
    pub mainclkselb: MainClkSelB,
    /// ahb Clock divider
    pub ahbclkdiv: AHBClkDiv,
    /// audio pll configuration
    pub audiopll: Option<AudioPllConfig>,
    /// mclk input freq
    pub mclkin: Option<Hertz>,
}

/// Default clock configuration
impl Default for Config {
    /// Default config
    /// Uses the 12MHz Free running oscillator
    #[inline]
    fn default() -> Config {
        Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            audiopll: None,
            mclkin: None,
        }
    }
}

impl Config {
    /// Uses FRO 12MHz.
    #[inline]
    pub fn fro12m() -> Config {
        Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            audiopll: None,
            mclkin: None,
        }
    }

    /// Uses the external xtal.
    pub fn external(xtal_freq: Hertz) -> Config {
        Config {
            xtal_freq: Some(xtal_freq),
            rtc_32k_present: None,
            mainclksela: MainClkSelA::clk_in,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            audiopll: None,
            mclkin: None,
        }
    }

    /// Uses the FRO HF at 48MHz.
    pub fn frohf_48mhz() -> Config {
        Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_hf(FroHfOsc::Fro48Mhz),
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            audiopll: None,
            mclkin: None,
        }
    }

    /// Uses the FRO HF at 96MHz.
    pub fn frohf_96mhz() -> Config {
        Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_hf(FroHfOsc::Fro96Mhz),
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            audiopll: None,
            mclkin: None,
        }
    }
}
