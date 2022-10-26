#![deny(unsafe_op_in_unsafe_fn)]
#![feature(default_alloc_error_handler)]
#![no_std]
#![no_main]
extern crate alloc;

use core::{arch::asm, cell::RefCell, mem::MaybeUninit, panic::PanicInfo};

use alloc::{boxed::Box, format};
use async_utils::{Runtime, StdPin};
use critical_section::Mutex;
use fugit::MillisDurationU32;
use futures::Future;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{bank0::*, AnyPin, FunctionPwm, Input, Output, Pin, PullUp, PushPull},
        pac,
        prelude::_rphal_pio_PIOExt,
        pwm::{FreeRunning, Pwm1, Slice, Slices},
        sio::Sio,
        watchdog::Watchdog,
        Timer,
    },
    pac::interrupt,
};

use embedded_hal::{digital::v2::OutputPin, PwmPin};

pub mod alloc_utils;
pub mod async_utils;
pub mod hal_exts;

/// I fully understand why `set_state` takes a `PinState` and not a `bool`, but it's annoying so I did this.
trait OutputPinExt: OutputPin {
    fn set(&mut self, high: bool) -> Result<(), Self::Error>;
}

impl<P: OutputPin + ?Sized> OutputPinExt for P {
    fn set(&mut self, high: bool) -> Result<(), Self::Error> {
        match high {
            false => self.set_low(),
            true => self.set_high(),
        }
    }
}

const CLOCK_FREQ: u32 = 125_000_000u32;

const fn clkdiv_and_top_for_freq(freq_int: u32, _freq_frac: u32) -> (u8, u8, u16) {
    // if freq_int < 7 || freq_int == 7 && freq_frac <=
    if freq_int <= 7 {
        return (255, 15, 65535);
    } else if freq_int >= CLOCK_FREQ {
        return (0, 1, 1);
    }

    let count = CLOCK_FREQ / freq_int;
    if count < 65535 {
        return (1, 0, count as u16);
    }
    let (count, _rem) = (count / 16, count % 16);
    if count < 65535 {
        return (16, 0, count as u16);
    }

    (1, 0, 65535)
}

static mut MIDI_NOTES: [(u8, u8, u16); 128] = [(0, 0, 0); 128];

#[derive(Clone, Copy)]
enum Note {
    Play(i8, u16),
    Rest(u16),
}

use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use Note::*;
// USB Communications Class Device support
use usbd_serial::SerialPort;

use crate::{alloc_utils::Arc, async_utils::Sleep, hal_exts::AlarmExt};

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<bsp::hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<bsp::hal::usb::UsbBus>> = None;

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
fn panic_led(_: &PanicInfo) -> ! {
    critical_section::with(|cs| {
        let mut led = PANIC_LED.borrow(cs).borrow_mut();
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

#[rustfmt::skip]
#[allow(unused)]
const SONG_1: &'static [(Note, &'static str)] = &[
    (Play(0, 250), "Take"),
    (Rest(250), ""),
    (Play(12, 250), "me"),
    (Play(9, 250), "out"),
    (Play(7, 250), "to"),
    (Play(4, 250), "the"),
    (Play(7, 750), "ball"),
    (Play(2, 750), "game\n"),

    (Play(0, 250), "Take"),
    (Rest(250), ""),
    (Play(12, 250), "me"),
    (Play(9, 250), "out"),
    (Play(7, 250), "with"),
    (Play(4, 250), "the"),
    (Play(7, 750), "crowd\n"),
    (Rest(750), ""),

    (Play(9, 250), "Buy"),
    (Play(7, 250), "me"),
    (Play(9, 250), "some"),
    (Play(4, 250), "pea-"),
    (Play(5, 250), "nuts"),
    (Play(7, 250), "and"),
    (Play(9, 250), "Crack-"),
    (Rest(250), ""),
    (Play(5, 250), "er"),
    (Play(2, 250), "Jack"),
    (Rest(500), ""),

    (Play(9, 250), "I"),
    (Rest(250), ""),
    (Play(9, 250), "don't"),
    (Play(9, 250), "care"),
    (Play(11, 250), "if"),
    (Play(12, 250), "I"),
    (Play(14, 250), "ev-"),
    (Play(11, 250), "er"),
    (Play(9, 250), "get"),
    (Play(7, 250), "back\n"),
    (Play(4, 250), "Let"),
    (Play(2, 250), "me"),

    (Play(0, 250), "root"),
    (Rest(250), ""),
    (Play(12, 250), "root"),
    (Play(9, 250), "root"),
    (Play(7, 250), "for"),
    (Play(4, 250), "the"),
    (Play(7, 750), "home"),
    (Play(2, 750), "team\n"),

    (Play(0, 250), "If"),
    (Play(0, 250), "they"),
    (Play(2, 250), "don't"),
    (Play(4, 250), "win"),
    (Play(5, 250), "it's"),
    (Play(7, 250), "a"),
    (Play(9, 1000), "shame\n"),
    (Play(9, 250), "For"),
    (Play(11, 250), "it's"),

    (Play(12, 250), "one"),
    (Rest(500), ""),
    (Play(12, 250), "two"),
    (Rest(500), ""),
    (Play(12, 250), "three"),
    (Play(11, 250), "strikes"),
    (Play(9, 250), "you're"),
    (Play(7, 250), "out"),
    (Play(6, 250), "at"),
    (Play(7, 250), "the"),

    (Play(9, 750), "old"),
    (Play(11, 750), "ball"),
    (Play(12, 750), "game"),
    (Rest(750), ""),
];

#[rustfmt::skip]
#[allow(unused)]
const SONG_2: &'static [Note] = &[
    Play(0, 250),

    Play(0, 250),
    Play(4, 250),
    Play(7, 250),
    Play(7, 250),
    Rest(250),
    Play(7 + 12, 250),
    Play(7 + 12, 250),
    Rest(250),
    Play(4 + 12, 250),
    Play(4 + 12, 250),
    Rest(250),
    Play(0, 250),

    Play(0, 250),
    Play(4, 250),
    Play(7, 250),
    Play(7, 250),
    Rest(250),
    Play(7 + 12, 250),
    Play(7 + 12, 250),
    Rest(250),
    Play(5 + 12, 250),
    Play(5 + 12, 250),
    Rest(250),
    Play(-1, 250),

    Play(-1, 250),
    Play(2, 250),
    Play(9, 250),
    Play(9, 250),
    Rest(250),
    Play(9 + 12, 250),
    Play(9 + 12, 250),
    Rest(250),
    Play(5 + 12, 250),
    Play(5 + 12, 250),
    Rest(250),
    Play(-1, 250),

    Play(-1, 250),
    Play(2, 250),
    Play(9, 250),
    Play(9, 250),
    Rest(250),
    Play(9 + 12, 250),
    Play(9 + 12, 250),
    Rest(250),
    Play(4 + 12, 250),
    Play(4 + 12, 250),
    Rest(250),
    Play(0, 250),

    Play(0, 250),
    Play(4, 250),
    Play(7, 250),
    Play(12, 250),
    Rest(250),
    Play(12 + 12, 250),
    Play(12 + 12, 250),
    Rest(250),
    Play(7 + 12, 250),
    Play(7 + 12, 250),
    Rest(250),
    Play(0, 250),

    Play(0, 250),
    Play(4, 250),
    Play(7, 250),
    Play(12, 250),
    Rest(250),
    Play(12 + 12, 250),
    Play(12 + 12, 250),
    Rest(250),
    Play(9 + 12, 250),
    Play(9 + 12, 250),
    Rest(250),
    Play(2, 250),

    Play(2, 250),
    Play(5, 250),
    Play(9, 250),
    Play(9, 1000),
    Play(6, 250),
    Play(7, 250),
    Play(4 + 12, 1000),
    Play(12, 250),
    Play(4, 250),

    Play(4, 500),
    Play(2, 250),
    Play(9, 500),
    Play(7, 250),

    Play(0, 250),
    Rest(250),
    Play(0, 250),
    Play(0, 250),
    Rest(500),
];

#[entry]
fn main() -> ! {
    static mut HEAP_MEM: [MaybeUninit<u8>; 131072] = unsafe { MaybeUninit::uninit().assume_init() };
    ALLOC.init_from_slice(HEAP_MEM);
    real_main() // rust-analyzer macro shenanigans
}

#[global_allocator]
static ALLOC: alloc_utils::Allocator = alloc_utils::Allocator::empty();

type ButtonsAndLeds = (
    Pin<Gpio10, Output<PushPull>>,
    Pin<Gpio11, Output<PushPull>>,
    Pin<Gpio12, Output<PushPull>>,
    Pin<Gpio13, Output<PushPull>>,
    Pin<Gpio2, Input<PullUp>>,
    Pin<Gpio3, Input<PullUp>>,
    Pin<Gpio4, Input<PullUp>>,
    Pin<Gpio5, Input<PullUp>>,
);

static PINS: Mutex<RefCell<Option<ButtonsAndLeds>>> = Mutex::new(RefCell::new(None));

fn real_main() -> ! {
    // defmt::info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    for i in 0..128 {
        let half_steps_from_a4 = i as i32 - 69;
        let freq = 444.0 * libm::powf(2.0_f32, half_steps_from_a4 as f32 / 12.0);
        let freq_int = freq as u32;
        // let freq_frac = (freq - freq_int as f32)
        // SAFETY: single-core program currently.
        unsafe {
            MIDI_NOTES[i] = clkdiv_and_top_for_freq(freq_int, 0);
        }
    }

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // https://github.com/rp-rs/rp-hal/blob/HEAD/boards/rp-pico/examples/pico_usb_serial.rs
    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
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
        pac::NVIC::unmask(bsp::hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let main_led_pin = pins.led.into_push_pull_output();
    critical_section::with(move |cs| {
        PANIC_LED.borrow(cs).replace(Some(main_led_pin));
    });
    let led1_pin = pins.gpio10.into_push_pull_output();
    let led2_pin = pins.gpio11.into_push_pull_output();
    let led3_pin = pins.gpio12.into_push_pull_output();
    let led4_pin = pins.gpio13.into_push_pull_output();

    let beep_pin = pins.gpio18.into_mode::<FunctionPwm>();

    let mut beep_pwm = pwm_slices.pwm1;
    beep_pwm.set_ph_correct();
    beep_pwm.enable();
    beep_pwm.set_div_int(1);
    beep_pwm.set_div_frac(0);

    let _channel_pin_a = beep_pwm.channel_a.output_to(beep_pin);

    beep_pwm.channel_a.set_duty(0x0050);
    beep_pwm.channel_a.enable();

    // let set_freq = |freq| {
    //     let (div_int, div_frac, top) = clkdiv_and_top_for_freq(freq, 0);
    //     beep_pwm.set_div_int(div_int);
    //     beep_pwm.set_div_frac(div_frac);
    //     beep_pwm.set_top(top);
    // };

    fn set_midi_note(
        note: isize,
        beep_pwm: &mut Slice<Pwm1, FreeRunning>,
        write: impl Fn(&str, u32),
    ) {
        // let mut set_midi_note = |note: isize| {
        write(&file!()[9..], line!());
        let (div_int, div_frac, top) = unsafe { MIDI_NOTES[note as usize] };
        write(&file!()[9..], line!());
        beep_pwm.disable(); // This may fix the occasional screeching?
        write(&file!()[9..], line!());
        beep_pwm.set_div_int(div_int);
        beep_pwm.set_div_frac(div_frac);
        beep_pwm.set_top(top);
        beep_pwm.enable();
        write(&file!()[9..], line!());
        // };
    }

    // let mut play_note = {
    //     let delay: &mut dyn DelayMs<u16> = &mut delay;
    //     move |note: &Note| match note {
    //         &Play(note, time) => {
    //             set_midi_note(note as isize + 69, &mut beep_pwm);
    //             delay.delay_ms(time);
    //         }
    //         &Rest(time) => {
    //             beep_pwm.disable();
    //             delay.delay_ms(time);
    //             beep_pwm.enable();
    //         }
    //     }
    // };

    let (mut pio, sm0, _a, _b, _c) = pac.PIO0.split(&mut pac.RESETS);

    let mut i2c = i2c_pio::I2C::new(
        &mut pio,
        pins.gpio8,
        pins.gpio9,
        sm0,
        fugit::RateExtU32::kHz(100),
        clocks.system_clock.freq(),
    );

    let lcd = lcd_lcm1602_i2c::Lcd::new(&mut i2c, &mut delay)
        .rows(2)
        .cursor_on(true)
        .address(0x27)
        .init()
        .unwrap();

    let lcd = Mutex::new(RefCell::new(lcd));

    #[cfg(any())]
    let write = Arc::new(move |file: &str, line: u32| {
        critical_section::with(|cs| {
            let mut lcd = lcd.borrow(cs).borrow_mut();
            // ASCII only
            lcd.clear().unwrap();
            for i in 0..file.len() {
                lcd.set_cursor(0, i as _).unwrap();
                lcd.write_str(&file[i..][..1]).unwrap();
            }
            let line = format!("{}", line);
            for i in 0..line.len() {
                lcd.set_cursor(1, i as _).unwrap();
                lcd.write_str(&line[i..][..1]).unwrap();
            }
            lcd.set_cursor(1, 10).unwrap();
            lcd.write_str("buf").unwrap();
        })
    });
    let write = Arc::new(|_: &str, _| ());

    write("Hello, world!", 666);

    // let mut set_leds = |value: u8| -> Result<(), _> {
    //     led1_pin.set(value & 8 != 0)?;
    //     led2_pin.set(value & 4 != 0)?;
    //     led3_pin.set(value & 2 != 0)?;
    //     led4_pin.set(value & 1 != 0)
    // };

    let button1_pin = pins.gpio2.into_pull_up_input();
    let button2_pin = pins.gpio3.into_pull_up_input();
    let button3_pin = pins.gpio4.into_pull_up_input();
    let button4_pin = pins.gpio5.into_pull_up_input();
    button1_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);
    button2_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);
    button3_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);
    button4_pin.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);

    critical_section::with(|cs| {
        PINS.borrow(cs).replace(Some((
            led1_pin,
            led2_pin,
            led3_pin,
            led4_pin,
            button1_pin,
            button2_pin,
            button3_pin,
            button4_pin,
        )));
    });

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    #[cfg(any())]
    {
        let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
        let mut count_down = timer.count_down();
        // Create a count_down timer for 500 milliseconds
        main_led_pin.set_high();
        count_down.start(500_u32.millis());
        // Block until timer has elapsed
        let _ = nb::block!(count_down.wait());
        main_led_pin.set_low();
        // Restart the count_down timer with a period of 100 milliseconds
        count_down.start(100_u32.millis());
        // Cancel it immediately
        count_down.cancel();

        let mut beep_pin = _channel_pin_a.into_mode::<Output<PushPull>>();
        beep_pin.set_high();
        count_down.start(500_u32.millis());
        // Block until timer has elapsed
        let _ = nb::block!(count_down.wait());
        beep_pin.set_low();

        loop {
            cortex_m::asm::wfi();
        }
    }
    let mut runtime = Runtime::new(/* core.SYST, clocks.system_clock.freq().to_Hz() */);
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm0 = Some(timer.alarm_0().unwrap());
    loop {
        runtime.block_on(async {
            // let mut play_note = {
            //     |note: &Note| -> StdPin<Box<dyn Future<Output = ()>>> {
            //         match note {
            //             &Play(note, time) => {
            //                 Box::pin(async {
            //                     beep_pwm.disable();
            //                     set_midi_note(note as isize + 69, &mut beep_pwm);
            //                     beep_pwm.enable();
            //                     // delay.delay_ms(time);
            //                 })
            //             }
            //             &Rest(time) => {
            //                 Box::pin(async {
            //                     beep_pwm.disable();
            //                     // delay.delay_ms(time);
            //                     beep_pwm.enable();
            //                 })
            //             }
            //         }
            //     }
            // };

            fn play_note<'a, Alarm: AlarmExt + 'static>(
                note: Note,
                beep_pwm: &'a mut Slice<Pwm1, FreeRunning>,
                alarm: Alarm,
                write: Arc<impl Fn(&str, u32) + 'a>,
            ) -> StdPin<Box<dyn Future<Output = Alarm> + 'a>> {
                match note {
                    Play(note, time) => Box::pin(async move {
                        write(&file!()[9..], line!());
                        beep_pwm.disable();
                        write(&file!()[9..], line!());
                        set_midi_note(note as isize + 69, beep_pwm, &*write);
                        write(&file!()[9..], line!());
                        beep_pwm.enable();
                        write(&file!()[9..], line!());
                        let alarm =
                            Sleep::new(&*write, alarm, MillisDurationU32::from_ticks(time.into()))
                                .ok()
                                .unwrap()
                                .await;
                        write(&file!()[9..], line!());
                        alarm
                    }),
                    Rest(time) => Box::pin(async move {
                        write(&file!()[9..], line!());
                        beep_pwm.disable();
                        write(&file!()[9..], line!());
                        let alarm =
                            Sleep::new(&*write, alarm, MillisDurationU32::from_ticks(time.into()))
                                .ok()
                                .unwrap()
                                .await;
                        write(&file!()[9..], line!());
                        beep_pwm.enable();
                        write(&file!()[9..], line!());
                        alarm
                    }),
                }
            }
            write(&file!()[9..], line!());
            for (note, _lyric) in SONG_1 {
                write(&file!()[9..], line!());
                alarm0 = Some(
                    play_note(*note, &mut beep_pwm, alarm0.take().unwrap(), write.clone()).await,
                );
                write(&file!()[9..], line!());
            }
            write(&file!()[9..], line!());
        })
    }
    // loop {
    //     for note in SONG_2 {
    //         // match note {
    //         //     Play(note, _) => set_leds(*note as u8 + 1).unwrap(),
    //         //     Rest(_) => set_leds(0).unwrap(),
    //         // }
    //         // lcd.set_cursor(0, 0).unwrap();
    //         // lcd.write_str(lyric).unwrap();
    //         play_note(note);
    //     }

    //     for (note, lyric) in SONG_1 {
    //         // match note {
    //         //     Play(note, _) => set_leds(*note as u8 + 1).unwrap(),
    //         //     Rest(_) => set_leds(0).unwrap(),
    //         // }
    //         // lcd.set_cursor(0, 0).unwrap();
    //         // lcd.write_str(lyric).unwrap();
    //         play_note(note);
    //     }
    // }
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

#[allow(non_snake_case)]
#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndButton>`
    static mut LEDS_AND_BUTTONS: Option<ButtonsAndLeds> = None;
    static mut COUNTER: usize = 1;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if LEDS_AND_BUTTONS.is_none() {
        critical_section::with(|cs| {
            *LEDS_AND_BUTTONS = PINS.borrow(cs).take();
        });
    }

    if let Some((led1, led2, led3, led4, button1, button2, button3, button4)) = LEDS_AND_BUTTONS {
        let mut set_leds = |value: u8| -> Result<(), _> {
            led1.set(value & 8 != 0)?;
            led2.set(value & 4 != 0)?;
            led3.set(value & 2 != 0)?;
            led4.set(value & 1 != 0)
        };
        fn handle_button<B: AnyPin>(button: &mut B, idx: usize, counter: &mut usize) {
            let button = button.as_mut();
            if button.interrupt_status(rp_pico::hal::gpio::Interrupt::EdgeLow) {
                *counter = idx;
                button.clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeLow);
            }
        }
        handle_button(button1, 1, COUNTER);
        handle_button(button2, 2, COUNTER);
        handle_button(button3, 3, COUNTER);
        handle_button(button4, 4, COUNTER);
        set_leds(*COUNTER as u8).unwrap();
    }
}
