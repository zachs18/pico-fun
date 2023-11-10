//! Using https://shop.pimoroni.com/products/pico-display-pack

#![deny(unsafe_op_in_unsafe_fn)]
#![no_std]
#![no_main]
extern crate alloc;

use async_utils::{Interval, Runtime, Sleep};
use board_support::hal::gpio::{FunctionPwm, FunctionSio, PullDown, SioOutput};
use board_support::hal::pwm::Slices;
use core::{cell::RefCell, mem::MaybeUninit, panic::PanicInfo};
use critical_section::Mutex;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use fugit::MicrosDurationU32;
use rp_pico as board_support;
use rp_pico::{
    entry,
    hal::{
        clocks::init_clocks_and_plls,
        gpio::{bank0::*, Pin},
        pac,
        sio::Sio,
        watchdog::Watchdog,
        Timer,
    },
};
// use usb_device::{
//     class_prelude::UsbBusAllocator,
//     prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
// };
// use usbd_serial::SerialPort;
use utils::{noploop, PinExt};

pub mod alloc_utils;
pub mod lcd;

static PANIC_LED: Mutex<RefCell<Option<Pin<Gpio25, FunctionSio<SioOutput>, PullDown>>>> =
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

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let main_led_pin = pins.led.into_push_pull_output();
    critical_section::with(move |cs| {
        PANIC_LED.borrow(cs).replace(Some(main_led_pin));
    });

    let led_red = pins.gpio6.into_function::<FunctionPwm>();
    let mut led_green = pins
        .gpio7
        .into_function::<FunctionSio<SioOutput>>()
        .into_pull_type::<PullDown>();
    let mut led_blue = pins
        .gpio8
        .into_function::<FunctionSio<SioOutput>>()
        .into_pull_type::<PullDown>();

    // low = on
    let _ = led_green.set_high();
    let _ = led_blue.set_high();

    let mut led_pwm = pwm_slices.pwm3;
    led_pwm.set_ph_correct();
    led_pwm.enable();
    led_pwm.set_div_int(1);
    led_pwm.set_div_frac(0);

    let _channel_pin_a = led_pwm.channel_a.output_to(led_red);

    led_pwm.channel_a.set_duty(!0x0080);
    led_pwm.channel_a.enable();

    let mut backlight_pin = pins.gpio20;
    let mut backlight_pwm = pwm_slices.pwm2;

    let button_a = pins.gpio12.into_pull_up_input().invert_pin();
    let button_b = pins.gpio13.into_pull_up_input().invert_pin();
    let button_x = pins.gpio14.into_pull_up_input().invert_pin();
    let button_y = pins.gpio15.into_pull_up_input().invert_pin();

    let mut runtime = Runtime::new(/* core.SYST, clocks.system_clock.freq().to_Hz() */);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut alarm1 = timer.alarm_1().unwrap();
    runtime.spawn({
        async move {
            let turn_on_led = || {
                critical_section::with(|cs| {
                    PANIC_LED
                        .borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .unwrap()
                        .set_high()
                        .unwrap();
                })
            };
            let turn_off_led = || {
                critical_section::with(|cs| {
                    PANIC_LED
                        .borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .unwrap()
                        .set_low()
                        .unwrap();
                })
            };

            let interval = Interval::new(alarm1, MicrosDurationU32::millis(500)).unwrap();
            let interval = &interval;
            loop {
                turn_on_led();

                interval.await;

                turn_off_led();

                interval.await;
            }
        }
    });

    let mut alarm2 = timer.alarm_2().unwrap();

    runtime.block_on(async {
        loop {
            let mut inv_duty = 0;
            if button_a.is_high().unwrap_or_default() {
                inv_duty |= 0x1000;
            }
            if button_b.is_high().unwrap_or_default() {
                inv_duty |= 0x0400;
            }
            if button_x.is_high().unwrap_or_default() {
                inv_duty |= 0x0100;
            }
            if button_y.is_high().unwrap_or_default() {
                inv_duty |= 0x0040;
            }
            led_pwm.channel_a.set_duty(!inv_duty);

            if let Ok(sleep) = Sleep::new(&mut alarm2, MicrosDurationU32::millis(500)) {
                sleep.await;
            } else {
                noploop(1048576);
            }
            led_pwm.channel_a.set_duty(!0);

            if let Ok(sleep) = Sleep::new(&mut alarm2, MicrosDurationU32::millis(500)) {
                sleep.await;
            } else {
                noploop(1048576);
            }
            led_pwm.channel_a.set_duty(!0x0080);

            if let Ok(sleep) = Sleep::new(&mut alarm2, MicrosDurationU32::millis(500)) {
                sleep.await;
            } else {
                noploop(1048576);
            }
        }
    })
}
