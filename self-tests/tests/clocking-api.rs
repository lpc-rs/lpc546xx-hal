#![deny(warnings)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use lpc546xx_hal::{self as _};
#[defmt_test::tests]
mod tests {
    use defmt::unwrap;
    //use defmt::assert;
    use lpc546xx_hal::{
        pac,
        prelude::*,
        syscon::{AHBClkDiv, Config, MainClkSelA, MainClkSelB},
    }; // the HAL we'll test

    #[test]
    fn fro12m_is_12mhz() {
        let dp = unsafe { pac::Peripherals::steal() };
        let syscon = dp.SYSCON.freeze(Config::fro12m());
        assert_eq!(unwrap!(syscon.get_fro_12m_clock_freq()), 12_000_000.Hz());
    }

    // I was not able to make this config work on the dev board with nxp's config tool either
    /*#[test]
    fn clk_external() {
        let dp = unsafe { pac::Peripherals::steal() };
        let syscon = dp.SYSCON.freeze(Config::external(12_000_000.Hz()));
        assert_eq!(unwrap!(syscon.get_main_clock_freq()), 12_000_000.Hz());
    }*/

    #[test]
    fn clk_fro_hf_48mhz() {
        let dp = unsafe { pac::Peripherals::steal() };
        let syscon = dp.SYSCON.freeze(Config::frohf_48mhz());
        assert_eq!(unwrap!(syscon.get_main_clock_freq()), 48_000_000.Hz());
    }

    #[test]
    fn clk_fro_hf_96mhz() {
        let dp = unsafe { pac::Peripherals::steal() };
        let syscon = dp.SYSCON.freeze(Config::frohf_96mhz());
        assert_eq!(unwrap!(syscon.get_main_clock_freq()), 96_000_000.Hz());
    }

    #[test]
    fn clk_audio_pll_512mhz() {
        let dp = unsafe { pac::Peripherals::steal() };
        let config = Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            mclkin: None,
        };
        let mut syscon = dp.SYSCON.freeze(config);
        let pll_freq = syscon
            .try_audio_pll_config(512_000_000.Hz())
            .unwrap()
            .unwrap();
        assert_eq!(pll_freq, 512.MHz());
    }

    #[test]
    fn clk_audio_pll_18mhz() {
        let dp = unsafe { pac::Peripherals::steal() };
        let config = Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            mclkin: None,
        };
        let mut syscon = dp.SYSCON.freeze(config);
        let pll_freq = syscon
            .try_audio_pll_config(18_000_000.Hz())
            .unwrap()
            .unwrap();
        assert_eq!(pll_freq, 18.MHz());
    }

    #[test]
    fn clk_audio_pll_43mhz() {
        let dp = unsafe { pac::Peripherals::steal() };
        let config = Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            mclkin: None,
        };
        let mut syscon = dp.SYSCON.freeze(config);
        let pll_freq = syscon
            .try_audio_pll_config(43_000_000.Hz())
            .unwrap()
            .unwrap();
        assert_eq!(pll_freq, 43.MHz());
    }
}
