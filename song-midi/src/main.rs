#![deny(unsafe_op_in_unsafe_fn)]
#![feature(default_alloc_error_handler)]
#![no_std]
#![no_main]
extern crate alloc;

use crate::lcd::LcdWriteError;
use alloc::boxed::Box;
use async_utils::Sleep;
use async_utils::{Runtime, StdPin};
use board_support::{
    entry,
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        gpio::{bank0::*, AnyPin, FunctionPwm, Input, Output, Pin, PullUp, PushPull},
        pac,
        prelude::*,
        pwm::{Slice, SliceId, SliceMode, Slices, ValidSliceMode},
        sio::Sio,
        watchdog::Watchdog,
        Timer,
    },
    pac::interrupt,
};
use core::{
    cell::RefCell,
    future::Future,
    mem::MaybeUninit,
    panic::PanicInfo,
    sync::atomic::{AtomicU32, Ordering},
};
use critical_section::Mutex;
use embedded_hal::{digital::v2::OutputPin, PwmPin};
use fugit::{MillisDurationU32, RateExtU32};
use hal_exts::{AlarmExt, OutputPinExt};
use i2c_pio::I2C;
use lcd::Lcd;
use midi_notes::MIDI_NOTES;
use rp_pico as board_support;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;
use utils::noploop;

pub mod alloc_utils;
pub mod lcd;
mod midi_notes;

#[derive(Clone, Copy)]
enum Note {
    Play(i8, u16),
    PlayShort(i8, u16),
    Rest(u16),
}
use Note::*;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<board_support::hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<board_support::hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<board_support::hal::usb::UsbBus>> = None;

static PANIC_LED: Mutex<RefCell<Option<Pin<Gpio25, Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));

static MIDI_FILE: &'static [u8] = include_bytes!("../file.mid");

#[panic_handler]
fn panic_led(info: &PanicInfo) -> ! {
    critical_section::with(|cs| {
        let mut led = PANIC_LED.borrow_ref_mut(cs);
        let mut leds = PINS.borrow_ref_mut(cs);
        let (_file, line) = info
            .location()
            .map(|loc| (loc.file(), loc.line()))
            .unwrap_or(("", 0));
        if let Some((led, (led1, led2, led3, led4, ..))) = led.as_mut().zip(leds.as_mut()) {
            let mut set_leds = |value: u8| -> Result<(), _> {
                led1.set(value & 8 != 0)?;
                led2.set(value & 4 != 0)?;
                led3.set(value & 2 != 0)?;
                led4.set(value & 1 != 0)
            };
            loop {
                for chunk in (0..8).rev() {
                    let chunk = line >> (chunk * 4);
                    let _ = set_leds(chunk as u8);
                    noploop(2097152);
                    let _ = set_leds(0);
                    noploop(2097152);
                }
                let _ = led.set_high();
                noploop(1048576);
                let _ = led.set_low();
                noploop(1048576);
            }
        } else if let Some(ref mut led) = *led {
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
    (Play(2, 750), "game"),

    (Play(0, 250), "Take"),
    (Rest(250), ""),
    (Play(12, 250), "me"),
    (Play(9, 250), "out"),
    (Play(7, 250), "with"),
    (Play(4, 250), "the"),
    (Play(7, 750), "crowd"),
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
    (PlayShort(9, 250), "don't"),
    (Play(9, 250), "care"),
    (Play(11, 250), "if"),
    (Play(12, 250), "I"),
    (Play(14, 250), "ev-"),
    (Play(11, 250), "er"),
    (Play(9, 250), "get"),
    (Play(7, 250), "back"),
    (Play(4, 250), "Let"),
    (Play(2, 250), "me"),

    (Play(0, 250), "root"),
    (Rest(250), ""),
    (Play(12, 250), "root"),
    (Play(9, 250), "root"),
    (Play(7, 250), "for"),
    (Play(4, 250), "the"),
    (Play(7, 750), "home"),
    (Play(2, 750), "team"),

    (PlayShort(0, 250), "If"),
    (Play(0, 250), "they"),
    (Play(2, 250), "don't"),
    (Play(4, 250), "win"),
    (Play(5, 250), "it's"),
    (Play(7, 250), "a"),
    (PlayShort(9, 1000), "shame"),
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
    (Play(12, 750), "game!"),
    (Rest(750), ""),
];

#[rustfmt::skip]
#[allow(unused)]
const SONG_2: &'static [Note] = &[
    PlayShort(0, 250),

    Play(0, 250),
    Play(4, 250),
    PlayShort(7, 250),
    Play(7, 250),
    Rest(250),
    PlayShort(7 + 12, 250),
    Play(7 + 12, 250),
    Rest(250),
    PlayShort(4 + 12, 250),
    Play(4 + 12, 250),
    Rest(250),
    PlayShort(0, 250),

    Play(0, 250),
    Play(4, 250),
    PlayShort(7, 250),
    Play(7, 250),
    Rest(250),
    PlayShort(7 + 12, 250),
    Play(7 + 12, 250),
    Rest(250),
    PlayShort(5 + 12, 250),
    Play(5 + 12, 250),
    Rest(250),
    PlayShort(-1, 250),

    Play(-1, 250),
    Play(2, 250),
    PlayShort(9, 250),
    Play(9, 250),
    Rest(250),
    PlayShort(9 + 12, 250),
    Play(9 + 12, 250),
    Rest(250),
    PlayShort(5 + 12, 250),
    Play(5 + 12, 250),
    Rest(250),
    PlayShort(-1, 250),

    Play(-1, 250),
    Play(2, 250),
    PlayShort(9, 250),
    Play(9, 250),
    Rest(250),
    PlayShort(9 + 12, 250),
    Play(9 + 12, 250),
    Rest(250),
    PlayShort(4 + 12, 250),
    Play(4 + 12, 250),
    Rest(250),
    PlayShort(0, 250),

    Play(0, 250),
    Play(4, 250),
    Play(7, 250),
    Play(12, 250),
    Rest(250),
    PlayShort(12 + 12, 250),
    Play(12 + 12, 250),
    Rest(250),
    PlayShort(7 + 12, 250),
    Play(7 + 12, 250),
    Rest(250),
    PlayShort(0, 250),

    Play(0, 250),
    Play(4, 250),
    Play(7, 250),
    Play(12, 250),
    Rest(250),
    PlayShort(12 + 12, 250),
    Play(12 + 12, 250),
    Rest(250),
    PlayShort(9 + 12, 250),
    Play(9 + 12, 250),
    Rest(250),
    PlayShort(2, 250),

    Play(2, 250),
    Play(5, 250),
    PlayShort(9, 250),
    Play(9, 1000),
    Play(6, 250),
    Play(7, 250),
    Play(4 + 12, 1000),
    Play(12, 250),
    PlayShort(4, 250),

    Play(4, 500),
    Play(2, 250),
    Play(9, 500),
    Play(7, 250),

    Play(0, 250),
    Rest(250),
    PlayShort(0, 250),
    Play(0, 250),
    Rest(500),
];

#[entry]
fn main() -> ! {
    static mut HEAP_MEM: [MaybeUninit<u8>; 131072] = [MaybeUninit::new(0); 131072];
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

/// Shift `top` down by this much to get the duty
/// Default: 0xffff -> 0x007f = shift of 9 bits
static DUTY_SHIFT: AtomicU32 = AtomicU32::new(9);

/// Assumes that `beep_pwm` is enabled. Leaves it enabled after this function.
/// Disables it during the function to ensure all values are latched together.
/// `note` should be in `[0, 127]`
fn set_midi_note<I: SliceId, M: SliceMode + ValidSliceMode<I>>(
    note: isize,
    beep_pwm: &mut Slice<I, M>,
) {
    let (div_int, div_frac, top) = MIDI_NOTES[note as usize];
    beep_pwm.disable();
    beep_pwm.set_div_int(div_int);
    beep_pwm.set_div_frac(div_frac);
    beep_pwm.set_top(top);
    beep_pwm
        .channel_a
        .set_duty(top >> DUTY_SHIFT.load(Ordering::Relaxed));
    beep_pwm.enable();
}

fn play_note<'a, Alarm: AlarmExt + 'static, I: SliceId, M: SliceMode + ValidSliceMode<I>>(
    note: Note,
    beep_pwm: &'a mut Slice<I, M>,
    alarm: &'a mut Alarm,
) -> impl Future<Output = ()> + 'a {
    async move {
        match note {
            Play(note, time) => {
                beep_pwm.channel_a.set_duty(0); // This should fix the occasional screeching.
                set_midi_note(note as isize + 69, beep_pwm);
                beep_pwm.enable();

                Sleep::new(alarm, MillisDurationU32::from_ticks(time.into()))
                    .unwrap()
                    .await;
            }
            PlayShort(note, time) => {
                set_midi_note(note as isize + 69, beep_pwm);

                match time.checked_sub(10) {
                    Some(play_time) => {
                        Sleep::new(alarm, MillisDurationU32::from_ticks(play_time.into()))
                            .unwrap()
                            .await;
                        beep_pwm.channel_a.set_duty(0); // This should fix the occasional screeching.
                        Sleep::new(alarm, MillisDurationU32::from_ticks(10))
                            .unwrap()
                            .await;
                    }
                    None => {
                        Sleep::new(alarm, MillisDurationU32::from_ticks(time.into()))
                            .unwrap()
                            .await;
                    }
                };
            }
            Rest(time) => {
                beep_pwm.channel_a.set_duty(0); // This should fix the occasional screeching.
                Sleep::new(alarm, MillisDurationU32::from_ticks(time.into()))
                    .unwrap()
                    .await;
            }
        }
    }
}

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

    beep_pwm.channel_a.set_duty(0x0080);
    beep_pwm.channel_a.enable();

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    // let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let mut spi_cs = pins.gpio1.into_push_pull_output(); // TODO: SPI CS supposed to be pulldown (assert low, not asserted high)?
    let spi = hal::Spi::<_, _, 16>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10_u32.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let (mut pio, sm0, _a, _b, _c) = pac.PIO0.split(&mut pac.RESETS);

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

    // let min_top = unsafe { MIDI_NOTES.iter() }
    //     .fold(0xffff_u16, |curtop, (_int, _frac, top)| curtop.min(*top));

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    let mut runtime = Runtime::new(/* core.SYST, clocks.system_clock.freq().to_Hz() */);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm2 = timer.alarm_2().unwrap();

    // critical_section::with(|cs| {
    //     let spi = unsafe {
    //         static mut TEMP_SPI: Option<Spi<Enabled, pac::SPI0, 8>> = None;
    //         TEMP_SPI = Some(spi);
    //         TEMP_SPI.as_mut().unwrap()
    //     };
    //     let spi_cs = unsafe {
    //         static mut TEMP_SPI_CS: Option<Pin<Gpio1, Output<PushPull>>> = None;
    //         TEMP_SPI_CS = Some(spi_cs);
    //         TEMP_SPI_CS.as_mut().unwrap()
    //     };
    //     let mut max7219 = max7219::Max7219::new(spi, Some(spi_cs), 1).unwrap();
    // });

    #[cfg(any())]
    runtime.spawn(async move {
        // let mut counter = 0_u64;

        let mut max7219 = max7219::Max7219::<_, _, 1>::new(&mut spi, Some(&mut spi_cs)).unwrap();
        // loop {
        //     Sleep::new(&mut alarm2, MillisDurationU32::from_ticks(100))
        //         .unwrap()
        //         .await;

        //     max7219.show(bytemuck::cast_ref(&counter)).unwrap();

        //     counter += 1;
        // }
        let addr: usize = real_main as usize;
        max7219.show(bytemuck::cast_ref(&[addr, 0])).unwrap();
    });

    let mut alarm3 = timer.alarm_3().unwrap();

    let (lyric_sender, lyric_receiver) = async_utils::channel::capacity_1_channel::<&str>();

    runtime.spawn(async move {
        let x: StdPin<Box<dyn Future<Output = Result<(), LcdWriteError<i2c_pio::Error>>> + Send>> =
            Box::pin(async move {
                let mut i2c = I2C::new(
                    &mut pio,
                    pins.gpio8,
                    pins.gpio9,
                    sm0,
                    fugit::RateExtU32::kHz(100),
                    clocks.system_clock.freq(),
                );
                let mut lcd = Lcd::new(&mut i2c)
                    .rows(2)
                    .cursor_on(true)
                    .address(0x27)
                    .init(&mut alarm3)
                    .await?;

                while let Ok(lyric) = lyric_receiver.recv().await {
                    lcd.clear(&mut alarm3).await?;
                    // lcd.set_cursor(0, 0, &mut alarm3).await?;
                    lcd.write_str(lyric, &mut alarm3).await?;
                }

                Ok(())

                // loop {
                //     lcd.clear(&mut alarm3).await?;
                //     lcd.write_str("Hello, world!", &mut alarm3).await?;

                //     let s = format!("min_top: {}", min_top);

                //     lcd.set_cursor(1, 0, &mut alarm3).await?;

                //     lcd.write_str(&s, &mut alarm3).await?;

                //     Sleep::new(&mut alarm3, MillisDurationU32::from_ticks(500000))?.await;
                // }
            });
        let r = x.await;
        critical_section::with(|cs| match r {
            Ok(..) => {}
            Err(LcdWriteError::ScheduleAlarmError(_alarm)) => {
                if let Some((led1, _led2, _led3, _led4, ..)) = &mut *PINS.borrow_ref_mut(cs) {
                    let _ = led1.set_high();
                }
            }
            Err(LcdWriteError::WriteError(_err)) => {
                if let Some((_led1, led2, _led3, _led4, ..)) = &mut *PINS.borrow_ref_mut(cs) {
                    let _ = led2.set_high();
                }
            }
        });
    });

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

    let mut alarm0 = timer.alarm_0().unwrap();
    // #[cfg(any())]
    loop {
        runtime.block_on(async {
            for &note in SONG_2 {
                // let _ = lyric_sender.try_send(lyric);
                play_note(note, &mut beep_pwm, &mut alarm0).await;
            }
            for &(note, lyric) in SONG_1 {
                let _ = lyric_sender.try_send(lyric);
                play_note(note, &mut beep_pwm, &mut alarm0).await;
            }
        })
    }

    let mut max7219 = max7219::Max7219::<_, _, 1>::new(&mut spi, Some(&mut spi_cs)).unwrap();

    #[cfg(any())]
    loop {
        let (midi_header, mut midi_tracks) = midly::parse(MIDI_FILE).unwrap();
        let mut track1 = midi_tracks.next().unwrap().unwrap();

        runtime.block_on(async {
            let mut cur_key = u8::MAX;
            beep_pwm.channel_a.set_duty(0);
            for event in track1 {
                let event = event.unwrap();
                max7219
                    .show(bytemuck::cast_ref(&u64::from(u32::from(event.delta))))
                    .unwrap();
                if let Ok(aaa) = Sleep::new(
                    &mut alarm0,
                    MillisDurationU32::from_ticks(u32::from(event.delta) * 2),
                ) {
                    aaa.await;
                } else {
                }
                if let midly::TrackEventKind::Midi { channel, message } = &event.kind {
                    match message {
                        &midly::MidiMessage::NoteOff { key, vel } => {
                            if key == cur_key {
                                // beep_pwm.channel_a.set_duty(0);
                            }
                        }
                        &midly::MidiMessage::NoteOn { key, vel } => {
                            if u8::from(vel) > 0 {
                                cur_key = key.into();
                                set_midi_note(isize::from(u8::from(key)) + 12isize, &mut beep_pwm);
                            } else if key == cur_key {
                                // beep_pwm.channel_a.set_duty(0);
                            }
                        }
                        midly::MidiMessage::Aftertouch { key, vel } => { /* ignore */ }
                        midly::MidiMessage::Controller { controller, value } => { /* ignore */ }
                        midly::MidiMessage::ProgramChange { program } => { /* ignore */ }
                        midly::MidiMessage::ChannelAftertouch { vel } => { /* ignore */ }
                        midly::MidiMessage::PitchBend { bend } => { /* ignore */ }
                    }
                }
            }
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
    static mut COUNTER: usize = 1;

    critical_section::with(|cs| {
        if let Some((led1, led2, led3, led4, button1, button2, button3, button4)) =
            &mut *PINS.borrow_ref_mut(cs)
        {
            let mut set_leds = |value: u8| -> Result<(), _> {
                led1.set(value & 8 != 0)?;
                led2.set(value & 4 != 0)?;
                led3.set(value & 2 != 0)?;
                led4.set(value & 1 != 0)
            };
            fn handle_button<B: AnyPin>(
                button: &mut B,
                idx: usize,
                counter: &mut usize,
                addl: Option<impl FnOnce()>,
            ) {
                let button = button.as_mut();
                if button.interrupt_status(rp_pico::hal::gpio::Interrupt::EdgeLow) {
                    *counter = idx;
                    button.clear_interrupt(rp_pico::hal::gpio::Interrupt::EdgeLow);
                    if let Some(addl) = addl {
                        addl();
                    }
                }
            }
            handle_button(
                button1,
                1,
                COUNTER,
                Some(|| {
                    let old_value = DUTY_SHIFT.load(Ordering::Relaxed);
                    let new_value = (old_value + 1).min(11);
                    DUTY_SHIFT.store(new_value, Ordering::Relaxed);
                }),
            );
            handle_button(button2, 2, COUNTER, None::<fn()>);
            handle_button(button3, 3, COUNTER, None::<fn()>);
            handle_button(
                button4,
                4,
                COUNTER,
                Some(|| {
                    let old_value = DUTY_SHIFT.load(Ordering::Relaxed);
                    let new_value = (old_value - 1).max(4);
                    DUTY_SHIFT.store(new_value, Ordering::Relaxed);
                }),
            );
            set_leds(*COUNTER as u8).unwrap();
        }
    });
}
