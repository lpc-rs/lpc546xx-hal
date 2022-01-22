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
        syscon::{AHBClkDiv, AudioPllClkSel, AudioPllConfig, Config, MainClkSelA, MainClkSelB},
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

    /// this test does not fully test the audio pll, because the ndec/pdec/mdec have
    /// not been inverted to provide calculation, and their config values still come
    /// from NXP Config tool, because their pseudo code did not produce the same
    /// values as their tool. TODO: fix the api and implement a better test.
    #[test]
    fn clk_audio_pll_partial() {
        let dp = unsafe { pac::Peripherals::steal() };
        let audiopll_config = AudioPllConfig {
            clksel: AudioPllClkSel::fro_12m,
            ndec: 1,
            mdec: 30580,
            bypass_pdiv: false,
            pdec: 66,
            pllrate: 512000000.Hz(),
        };
        let config = Config {
            xtal_freq: None,
            rtc_32k_present: None,
            mainclksela: MainClkSelA::fro_12m,
            mainclkselb: MainClkSelB::mainclka,
            ahbclkdiv: AHBClkDiv::NotDivided,
            audiopll: Some(audiopll_config),
        };
        let syscon = dp.SYSCON.freeze(config);
        let audio_clk = syscon.get_audio_pll_clk_clock_freq().unwrap();
        assert_eq!(audio_clk, 512.MHz());
    }
}
