#![deny(unsafe_op_in_unsafe_fn)]
#![no_std]
#![no_main]

use core::cell::RefCell;

use bsp::{
    entry,
    hal::{gpio::FunctionPwm, prelude::_rphal_pio_PIOExt, pwm::Slices},
    pac::interrupt,
};
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin, PwmPin};
use panic_probe as _;

/// I fully understand why `set_state` takes a `PinState` and not a `bool`, but it's annoying so I did this.
trait PinExt: OutputPin {
    fn set(&mut self, high: bool) -> Result<(), Self::Error>;
}

impl<P: OutputPin + ?Sized> PinExt for P {
    fn set(&mut self, high: bool) -> Result<(), Self::Error> {
        match high {
            false => self.set_low(),
            true => self.set_high(),
        }
    }
}

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

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
    let (count, rem) = (count / 16, count % 16);
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

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<bsp::hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<bsp::hal::usb::UsbBus>> = None;

const SONG: &'static [(Note, &'static str)] = &[
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
    (Play(7, 1250), "crowd\n"),
    (Rest(250), ""),
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

#[entry]
fn main() -> ! {
    real_main() // rust-analyzer macro shenanigans
}

fn real_main() -> ! {
    info!("Program start");
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

    struct DelayRef(RefCell<Delay>);

    impl<T> DelayMs<T> for &DelayRef
    where
        Delay: DelayMs<T>,
    {
        fn delay_ms(&mut self, ms: T) {
            DelayMs::delay_ms(&mut *self.0.borrow_mut(), ms)
        }
    }

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let delay = DelayRef(RefCell::new(delay));
    let mut delay = &delay;

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

    let mut main_led_pin = pins.led.into_push_pull_output();
    let mut led1_pin = pins.gpio10.into_push_pull_output();
    let mut led2_pin = pins.gpio11.into_push_pull_output();
    let mut led3_pin = pins.gpio12.into_push_pull_output();
    let mut led4_pin = pins.gpio13.into_push_pull_output();

    let beep_pin = pins.gpio18.into_mode::<FunctionPwm>();

    let mut beep_pwm = pwm_slices.pwm1;
    beep_pwm.set_ph_correct();
    beep_pwm.enable();
    beep_pwm.set_div_int(1);
    beep_pwm.set_div_frac(0);

    let _channel_pin_a = beep_pwm.channel_a.output_to(beep_pin);

    beep_pwm.channel_a.set_duty(0x0070);
    beep_pwm.channel_a.enable();

    // let set_freq = |freq| {
    //     let (div_int, div_frac, top) = clkdiv_and_top_for_freq(freq, 0);
    //     beep_pwm.set_div_int(div_int);
    //     beep_pwm.set_div_frac(div_frac);
    //     beep_pwm.set_top(top);
    // };

    macro_rules! set_midi_note {
        ($note:expr) => {{
            // let mut set_midi_note = |note: isize| {
            let (div_int, div_frac, top) = unsafe { MIDI_NOTES[$note as usize] };
            beep_pwm.disable(); // This may fix the occasional screeching?
            beep_pwm.set_div_int(div_int);
            beep_pwm.set_div_frac(div_frac);
            beep_pwm.set_top(top);
            beep_pwm.enable();
            // };
        }};
    }

    let mut play_note = {
        let mut delay: &DelayRef = delay;
        move |note: &Note| match note {
            &Play(note, time) => {
                set_midi_note!(note as isize + 69);
                delay.delay_ms(time);
            }
            &Rest(time) => {
                beep_pwm.disable();
                delay.delay_ms(time);
                beep_pwm.enable();
            }
        }
    };

    let (mut pio, sm0, _a, _b, _c) = pac.PIO0.split(&mut pac.RESETS);

    let mut i2c = i2c_pio::I2C::new(
        &mut pio,
        pins.gpio8,
        pins.gpio9,
        sm0,
        fugit::RateExtU32::kHz(100),
        clocks.system_clock.freq(),
    );

    let mut lcd = lcd_lcm1602_i2c::Lcd::new(&mut i2c, &mut delay)
        .rows(2)
        .cursor_on(true)
        .address(0x27)
        .init()
        .unwrap();

    lcd.write_str("Hello, world!").unwrap();

    let mut set_leds = |value: u8| -> Result<(), _> {
        led1_pin.set(value & 8 != 0)?;
        led2_pin.set(value & 4 != 0)?;
        led3_pin.set(value & 2 != 0)?;
        led4_pin.set(value & 1 != 0)
    };

    loop {
        for (note, lyric) in SONG {
            match note {
                Play(note, _) => set_leds(*note as u8 + 1).unwrap(),
                Rest(_) => set_leds(0).unwrap(),
            }
            lcd.set_cursor(0, 0).unwrap();
            lcd.write_str(lyric).unwrap();
            play_note(note);
        }
    }
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
