#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};

use lpc546xx_hal::{prelude::*, syscon::Config, usb::USBFS};
use lpc_usbd::{self, bus::UsbBus};

use usbd_serial::SerialPort;

#[entry]
fn main() -> ! {
    defmt::info!("Usb ACM CDC class example");
    defmt::info!("verify the enumeration using `lsusb` on your computer");
    defmt::info!("if you open this port with any settings, it should echo");
    defmt::info!("back letters uppercased");
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
    const MANUFACTURER: &'static str = "lpc546xx-hal";
    const PRODUCT: &'static str = "Usb Serial Example for lpc546xx-hal";
    const SERIAL_NUMBER: &'static str = "ACM CDC Serial";

    // define serial enum
    let mut serial = SerialPort::new(&usb_bus);
    // start the enumeration
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(VID, PID))
        .manufacturer(MANUFACTURER)
        .product(PRODUCT)
        .serial_number(SERIAL_NUMBER)
        .max_packet_size_0(64)
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}
