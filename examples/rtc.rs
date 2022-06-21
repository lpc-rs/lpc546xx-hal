#![no_main]
#![no_std]

use cortex_m_rt::entry;

use defmt_rtt as _;
use lpc546xx_hal::{pac, prelude::*, rtc::Rtc, syscon::Config};
use nb::block;
use panic_probe as _;
#[entry]
fn main() -> ! {
    defmt::info!("gpio example");
    let dp = pac::Peripherals::take().unwrap();
    //let mut iocon = dp.IOCON;
    let mut syscon = dp.SYSCON.freeze(Config::fro12m());

    let mut rtc = Rtc::new(dp.RTC, &mut syscon, None).unwrap();

    for i in 0..7 {
        rtc.write_general_purpose_registers(i as usize, i).unwrap();
    }
    for i in 0..7 {
        assert!(rtc.read_general_purpose_registers(i as usize).unwrap() == i);
    }
    rtc.read_general_purpose_registers(0).unwrap();
    defmt::println!("Waiting for 5 seconds");
    let timestamp = rtc.get_time().timestamp();
    rtc.set_alarm_from_timestamp((timestamp + 5) as u32)
        .unwrap();
    block!(rtc.wait()).unwrap();
    defmt::println!("Alarm triggered");

    loop {
        defmt::println!("{:?}", defmt::Display2Format(&rtc.get_time()));
        let timestamp = rtc.get_time().timestamp();
        rtc.set_alarm_from_timestamp((timestamp + 1) as u32)
            .unwrap();
        block!(rtc.wait()).unwrap();
    }
}
