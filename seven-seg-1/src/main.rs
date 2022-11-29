#![deny(unsafe_op_in_unsafe_fn)]
#![feature(default_alloc_error_handler)]
#![no_std]
#![no_main]
extern crate alloc;

use async_utils::{Runtime, Sleep};
use core::{arch::asm, cell::RefCell, mem::MaybeUninit, panic::PanicInfo};
use critical_section::Mutex;
use embedded_hal::digital::v2::OutputPin;
use fugit::{MillisDurationU32, RateExtU32};
use hal_exts::OutputPinExt;
use rp_pico as board_support;
use rp_pico::{
    entry,
    hal::{
        self,
        clocks::init_clocks_and_plls,
        gpio::{bank0::*, Output, Pin, PushPull},
        pac,
        sio::Sio,
        watchdog::Watchdog,
        Clock, Timer,
    },
    pac::interrupt,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

pub mod alloc_utils;
// pub mod async_utils;
// pub mod hal_exts;
// pub mod lcd;
pub mod max7219;
// mod midi_notes;
pub mod sevenseg_5611bs;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<board_support::hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<board_support::hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<board_support::hal::usb::UsbBus>> = None;

static PANIC_LED: Mutex<RefCell<Option<Pin<Gpio25, Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));

fn noploop(count: usize) {
    for _ in 0..count {
        unsafe {
            asm!("nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",);
        }
    }
}

#[panic_handler]
fn panic_led(info: &PanicInfo) -> ! {
    critical_section::with(|cs| {
        let mut led = PANIC_LED.borrow_ref_mut(cs);
        let (_file, _line) = info
            .location()
            .map(|loc| (loc.file(), loc.line()))
            .unwrap_or(("", 0));
        if let Some(ref mut led) = *led {
            loop {
                let _ = led.set_high();
                noploop(1048576);
                let _ = led.set_low();
                noploop(1048576);
            }
        } else {
            loop {}
        }
    })
}

#[entry]
fn main() -> ! {
    static mut HEAP_MEM: [MaybeUninit<u8>; 131072] = [MaybeUninit::new(0); 131072];
    ALLOC.init_from_slice(HEAP_MEM);
    real_main() // rust-analyzer macro shenanigans
}

#[global_allocator]
static ALLOC: alloc_utils::Allocator = alloc_utils::Allocator::empty();

fn real_main() -> ! {
    // defmt::info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = board_support::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // https://github.com/rp-rs/rp-hal/blob/HEAD/boards/rp-pico/examples/pico_usb_serial.rs
    let usb_bus = UsbBusAllocator::new(board_support::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let bus_ref = unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
        // SAFETY: we drop this ref before starting interrupts.
        USB_BUS.as_ref().unwrap()
    };

    // Set up the USB Communications Class Device driver
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_SERIAL = Some(SerialPort::new(bus_ref));
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    drop(bus_ref);
    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(board_support::hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let main_led_pin = pins.led.into_push_pull_output();
    critical_section::with(move |cs| {
        PANIC_LED.borrow(cs).replace(Some(main_led_pin));
    });

    let mut a_pin = pins.gpio17.into_push_pull_output(); // 7
    let mut b_pin = pins.gpio16.into_push_pull_output(); // 6
    let mut c_pin = pins.gpio14.into_push_pull_output(); // 4
    let mut d_pin = pins.gpio13.into_push_pull_output(); // 2
    let mut e_pin = pins.gpio12.into_push_pull_output(); // 1
    let mut f_pin = pins.gpio18.into_push_pull_output(); // 9
    let mut g_pin = pins.gpio19.into_push_pull_output(); // 10
    let mut dot_pin = pins.gpio15.into_push_pull_output(); // 5

    let mut sevenseg = sevenseg_5611bs::SevenSeg::new_common_anode(
        &mut a_pin,
        &mut b_pin,
        &mut c_pin,
        &mut d_pin,
        &mut e_pin,
        &mut f_pin,
        &mut g_pin,
        Some(&mut dot_pin),
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    // let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let mut spi_cs = pins.gpio5.into_push_pull_output(); // TODO: SPI CS supposed to be pulldown (assert low, not asserted high)?
    let spi = hal::Spi::<_, _, 16>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10_u32.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut runtime = Runtime::new(/* core.SYST, clocks.system_clock.freq().to_Hz() */);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let mut alarm1 = timer.alarm_1().unwrap();
    runtime.spawn({
        async move {
            loop {
                critical_section::with(|cs| {
                    PANIC_LED
                        .borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .unwrap()
                        .set_high()
                        .unwrap();
                });

                Sleep::new(&mut alarm1, MillisDurationU32::from_ticks(500))
                    .unwrap()
                    .await;

                critical_section::with(|cs| {
                    PANIC_LED
                        .borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .unwrap()
                        .set_low()
                        .unwrap();
                });

                Sleep::new(&mut alarm1, MillisDurationU32::from_ticks(500))
                    .unwrap()
                    .await;
            }
        }
    });

    let mut alarm2 = timer.alarm_2().unwrap();
    runtime.spawn(async move {
        let mut max7219 = max7219::Max7219::<_, _, 1>::new(&mut spi, Some(&mut spi_cs)).unwrap();
        let mut counter: u64 = 0;
        loop {
            max7219.show(bytemuck::cast_ref(&counter)).unwrap();
            counter += 1;
            Sleep::new(&mut alarm2, MillisDurationU32::from_ticks(10))
                .unwrap()
                .await;
        }
    });

    let mut led = pins.gpio11.into_push_pull_output();

    let mut alarm0 = timer.alarm_0().unwrap();
    // runtime.block_on(async {
    //     let mut counter = 0;
    //     loop {
    //         Sleep::new(&mut alarm0, MillisDurationU32::from_ticks(500))
    //             .unwrap()
    //             .await;
    //         sevenseg.write_digit(counter).unwrap();
    //         counter += 1;
    //         counter &= 0b1111;
    //     }
    // })
    runtime.block_on(async {
        loop {
            for digit in 0..16 {
                sevenseg.write_digit(digit).unwrap();
                led.set(digit & 1 != 0).unwrap();

                Sleep::new(&mut alarm0, MillisDurationU32::from_ticks(300))
                    .unwrap()
                    .await;
            }
            sevenseg.clear().unwrap();

            Sleep::new(&mut alarm0, MillisDurationU32::from_ticks(300))
                .unwrap()
                .await;
        }
    })
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Note whether we've already printed the "hello" message.
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

    // Say hello exactly once on start-up
    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = serial.write(b"Hello, World!\r\n");
    }

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                // Convert to upper case
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });
                // led1_pin.set(buf[0] & 1 != 0).unwrap();
                // led2_pin.set(buf[0] & 2 != 0).unwrap();
                // led3_pin.set(buf[0] & 4 != 0).unwrap();
                // led4_pin.set(buf[0] & 8 != 0).unwrap();

                // Send back to the host
                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                    });
                }
            }
        }
    }
}
