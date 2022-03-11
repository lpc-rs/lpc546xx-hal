#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use usb_device::{
    device::{UsbDeviceBuilder, UsbVidPid},
    test_class::TestClass,
};

use lpc546xx_hal::{prelude::*, syscon::Config, usb::USBFS};
use lpc_usbd::{self, bus::UsbBus};

#[entry]
fn main() -> ! {
    defmt::info!("Usb Test class example");
    defmt::info!("verify the enumeration using `lsusb` on your computer");
    let dp = lpc546xx_hal::pac::Peripherals::take().unwrap();
    // you need to check if your clock speed works with USB.
    let mut syscon = dp.SYSCON.freeze(Config::frohf_96mhz());
    let mut iocon = dp.IOCON;
    let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

    // this is optional
    let vbus_pin = gpio.pio0_22.into_floating_input();
    vbus_pin.set_alt_mode(lpc546xx_hal::gpio::AltMode::FUNC7);

    // create and init hardware
    let usb = USBFS::new(dp.USB0, dp.USBFSH, &mut syscon);
    // init usb stack
    let usb_bus = UsbBus::new(usb);

    const VID: u16 = 0x1FC9; // NXP VID for example
    const PID: u16 = 0x0094;
    const MANUFACTURER: &'static str = "TestClass lpc546xx-hal";
    const PRODUCT: &'static str = "TestClass Example";
    const SERIAL_NUMBER: &'static str = "TestClass Serial";

    // define test class
    let mut test = TestClass::new(&usb_bus);
    // start the enumeration
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(VID, PID))
        .manufacturer(MANUFACTURER)
        .product(PRODUCT)
        .serial_number(SERIAL_NUMBER)
        .max_packet_size_0(64)
        .build();

    loop {
        if usb_dev.poll(&mut [&mut test]) {
            test.poll();
        }
    }
}
