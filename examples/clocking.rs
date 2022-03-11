#![no_main]
#![no_std]

use cortex_m_rt::entry;

use defmt_rtt as _;
use lpc546xx_hal::{
    pac,
    prelude::*,
    syscon::{AHBClkDiv, AudioPllClkSel, AudioPllConfig, Config, MainClkSelA, MainClkSelB},
};
use panic_probe as _;
#[entry]
fn main() -> ! {
    defmt::info!("clocking example");

    let dp = pac::Peripherals::take().unwrap();
    let audio_pll_config = AudioPllConfig {
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
        audiopll: Some(audio_pll_config),
        mclkin: None,
    };
    let syscon = dp.SYSCON.freeze(config);
    let audio_clk = syscon.get_audio_pll_clk_clock_freq().unwrap();
    assert_eq!(audio_clk, 512.MHz());
    loop {
        cortex_m::asm::delay(1_000_000);
    }
}
