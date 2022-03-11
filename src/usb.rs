use core::ops::Deref;

use crate::{
    pac::{self},
    syscon::{ClockControl, Syscon},
};
use lpc_usbd::{self, UsbPeripheral};
pub struct USBFS {
    pub usb_dev: pac::USB0,
    pub usb_host: pac::USBFSH,
}

impl USBFS {
    /// Construct a USB peripheral wrapper.
    ///
    /// Call `UsbBus::new` to construct and initialize the USB peripheral driver.
    pub fn new(usb_dev: pac::USB0, usb_host: pac::USBFSH, syscon: &mut Syscon) -> Self
    where
        pac::USB0: ClockControl,
    {
        let syscon_raw = unsafe { &*pac::SYSCON::ptr() };
        // disable clocks for clock setup
        usb_dev.disable_clock(syscon);
        // usb clock = mainclk / 2 (because mainclock == 96MHz)
        syscon_raw
            .usb0clkdiv
            .modify(|_, w| unsafe { w.div().bits(1) });
        // run clock
        syscon_raw.usb0clkdiv.modify(|_, w| w.halt().clear_bit());
        // select FRO HF as source (96 MHz)
        syscon_raw.usb0clksel.modify(|_, w| w.sel().fro_hf());
        // wait for request done
        while syscon_raw.usb0clkdiv.read().reqflag().bit_is_set() {}
        // enable clocks
        usb_dev.enable_clock(syscon);
        // enable USB1RAM
        syscon_raw.ahbclkctrl2.modify(|_, w| w.usb1ram().set_bit());
        // check clk source
        //defmt::info!("clock: {:?}",usb_dev.get_clock_freq(syscon).unwrap().0);
        let usbh = unsafe { &*pac::USBFSH::ptr() };
        // enable device mode in host controller (?)
        syscon_raw.ahbclkctrl2.modify(|_, w| w.usb0hsl().set_bit());
        usbh.portmode.modify(|_, w| w.dev_enable().set_bit());
        syscon_raw
            .ahbclkctrl2
            .modify(|_, w| w.usb0hsl().clear_bit());
        Self { usb_dev, usb_host }
    }
}

unsafe impl Sync for USBFS {}

impl Deref for USBFS {
    type Target = lpc_usbd::pac::usb::RegisterBlock;
    fn deref(&self) -> &Self::Target {
        let ptr = USBFS::REGISTERS as *const Self::Target;
        unsafe { &*ptr }
    }
}

impl UsbPeripheral for USBFS {
    const REGISTERS: *const () = pac::USB0::ptr() as *const ();
    const SPEED: lpc_usbd::UsbSpeed = lpc_usbd::UsbSpeed::FullSpeed;
}
