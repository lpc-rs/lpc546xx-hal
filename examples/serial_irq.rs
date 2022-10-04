#![no_main]
#![no_std]

use core::cell::RefCell;

use cortex_m::peripheral::NVIC;
use cortex_m::{asm, interrupt::Mutex};
use cortex_m_rt::entry;
use defmt_rtt as _;
use lpc546xx_hal::{
    pac::{self, interrupt, Interrupt, FLEXCOMM0, USART0},
    prelude::*,
    serial::{self, Serial},
    syscon::Config,
};
use panic_probe as _;

static G_SERIAL: Mutex<RefCell<Option<Serial<USART0, FLEXCOMM0>>>> = Mutex::new(RefCell::new(None));
#[entry]
fn main() -> ! {
    defmt::info!("Serial IRQ example, will echo using flexcomm IRQ");
    let dp = pac::Peripherals::take().unwrap();
    let mut iocon = dp.IOCON;
    let mut syscon = dp.SYSCON.freeze(Config::fro12m());

    let gpio = dp.GPIO.split(&mut syscon, &mut iocon);

    let rx_pin = gpio.pio0_29;
    let tx_pin = gpio.pio0_30;

    let conf = serial::Config {
        baudrate: 115200.Bd(),
        wordlength: serial::WordLength::DataBits8,
        parity: serial::Parity::ParityNone,
        stopbits: serial::StopBits::STOP1,
        loopback: serial::Loopback::Normal,
    };
    let mut serial = dp
        .USART0
        .usart(dp.FLEXCOMM0, tx_pin, rx_pin, conf, &mut syscon)
        .unwrap();

    serial.listen(serial::Event::TxErr);
    serial.listen(serial::Event::RxErr);
    serial.listen(serial::Event::RxLvl);
    serial.set_rx_threshold(1); // generate interrupt as soon as 1 word has been received

    //serial.listen(serial::Event::TxLvl);

    cortex_m::interrupt::free(|cs| *G_SERIAL.borrow(cs).borrow_mut() = Some(serial));
    unsafe {
        NVIC::unmask(Interrupt::FLEXCOMM0);
    }

    loop {
        asm::wfi();
        defmt::info!("out of wfi");
    }
}

#[interrupt]
fn FLEXCOMM0() {
    static mut SERIAL: Option<Serial<USART0, FLEXCOMM0>> = None;
    let serial = SERIAL.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move serial  here, leaving a None in its place
            G_SERIAL.borrow(cs).replace(None).unwrap()
        })
    });
    if let Some(x) = serial.pending_event() {
        defmt::info!("event: {:?}", defmt::Debug2Format(&x));

        if x == serial::Event::RxLvl {
            let z = serial.read();
            match z {
                Ok(v) => {
                    defmt::info!("received: {:a}", v); // -> INFO 97
                    serial.write(v).unwrap();
                }
                Err(e) => {
                    defmt::info!("error reading received char: {:?}", defmt::Debug2Format(&e))
                }
            }
        }
    }
}
