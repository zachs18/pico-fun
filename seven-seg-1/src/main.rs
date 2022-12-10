#![deny(unsafe_op_in_unsafe_fn)]
#![feature(default_alloc_error_handler)]
#![no_std]
#![no_main]
extern crate alloc;

use async_utils::{Interval, Runtime, Sleep};
use core::{cell::RefCell, mem::MaybeUninit, panic::PanicInfo};
use critical_section::Mutex;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
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
use utils::{noploop, MaybeOwned};

pub mod alloc_utils;
// pub mod lcd;
// mod midi_notes;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<board_support::hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<board_support::hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<board_support::hal::usb::UsbBus>> = None;

static PANIC_LED: Mutex<RefCell<Option<Pin<Gpio25, Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));

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
        MaybeOwned::Borrowed(&mut a_pin),
        MaybeOwned::Borrowed(&mut b_pin),
        MaybeOwned::Borrowed(&mut c_pin),
        MaybeOwned::Borrowed(&mut d_pin),
        MaybeOwned::Borrowed(&mut e_pin),
        MaybeOwned::Borrowed(&mut f_pin),
        MaybeOwned::Borrowed(&mut g_pin),
        Some(MaybeOwned::Borrowed(&mut dot_pin)),
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

    let alarm2 = timer.alarm_2().unwrap();
    let interval2 = Interval::new(alarm2, MillisDurationU32::from_ticks(200)).unwrap();
    #[allow(non_snake_case)]
    runtime.spawn(async move {
        let mut max7219 = max7219::Max7219::<_, _, 1>::new(&mut spi, Some(&mut spi_cs)).unwrap();
        let mut hcounter: u64 = 255;
        let mut vcounter: [u8; 8] = [1; 8];
        let interval2 = &interval2;
        #[cfg(any())]
        loop {
            max7219
                .show(bytemuck::cast_ref(
                    &(hcounter | bytemuck::cast::<_, u64>(vcounter)),
                ))
                .unwrap();
            hcounter = hcounter.rotate_right(8);
            interval2.await;

            max7219
                .show(bytemuck::cast_ref(
                    &(hcounter | bytemuck::cast::<_, u64>(vcounter)),
                ))
                .unwrap();
            vcounter = vcounter.map(|i| i.rotate_right(1));
            interval2.await;
        }

        fn rotate_bits(value: u64) -> u64 {
            let mut result = 0;
            for y in 0..8 {
                for x in 0..8 {
                    if value & (1u64 << (8 * y + x)) != 0 {
                        result |= 1u64 << (8 * x + 7 - y);
                    }
                }
            }

            result
        }

        let L: [u8; 8] = bytemuck::cast(rotate_bits(bytemuck::cast::<[u8; 8], u64>([
            0,
            0b0100_0000,
            0b0100_0000,
            0b0100_0000,
            0b0100_0000,
            0b0100_0000,
            0b0111_1110,
            0,
        ])));
        let E: [u8; 8] = bytemuck::cast(rotate_bits(bytemuck::cast::<[u8; 8], u64>([
            0,
            0b0111_1110,
            0b0100_0000,
            0b0111_1000,
            0b0100_0000,
            0b0100_0000,
            0b0111_1110,
            0,
        ])));
        let D: [u8; 8] = bytemuck::cast(rotate_bits(bytemuck::cast::<[u8; 8], u64>([
            0,
            0b0111_1100,
            0b0100_0010,
            0b0100_0010,
            0b0100_0010,
            0b0100_0010,
            0b0111_1100,
            0,
        ])));

        let smile: [u8; 8] = bytemuck::cast(rotate_bits(bytemuck::cast::<[u8; 8], u64>([
            0b0011_1100,
            0b0100_0010,
            0b1010_0101,
            0b1000_0001,
            0b1010_0101,
            0b1001_1001,
            0b0100_0010,
            0b0011_1100,
        ])));

        let blank = bytemuck::cast(0u64);

        #[cfg(all())]
        loop {
            for disp in [L, E, D, blank, smile, smile, blank] {
                max7219.show(bytemuck::cast_ref(&disp)).unwrap();
                interval2.await;
            }
        }
    });

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut led_pwm = pwm_slices.pwm5;
    led_pwm.set_ph_correct();
    led_pwm.enable();
    led_pwm.set_div_int(1);
    led_pwm.set_div_frac(0);
    led_pwm.set_top(u16::MAX);

    let _headphones = led_pwm.channel_a.output_to(pins.gpio10);
    let _led = led_pwm.channel_b.output_to(pins.gpio11);

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
                led_pwm.channel_b.set_duty((digit as u16) << 12);

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
