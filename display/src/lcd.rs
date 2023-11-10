//! Modified from [lcd_lcm1602_i2c][https://docs.rs/lcd-lcm1602-i2c/0.1.0/lcd_lcm1602_i2c/]
//!
//! Driver to write characters to LCD displays with a LM1602 connected via i2c like [this one] with
//! 16x2 characters. It requires a I2C instance implementing [`embedded_hal::blocking::i2c::Write`]
//! and an alarm to delay execution using [`crate::async_utils::Sleep`]
//!
//! Usage:
//! ```
//! const LCD_ADDRESS: u8 = 0x27; // Address depends on hardware, see link below
//!
//! // Create a I2C instance, needs to implement embedded_hal::blocking::i2c::Write, this
//! // particular uses the rp_pico crate for the Raspberry Pi Pico.
//! let dp = rp_pico::hal::Peripherals::take().unwrap();
//! let pins = rp_pico::hal::pins!(dp);
//! let (mut pio, sm0, _a, _b, _c) = pac.PIO0.split(&mut pac.RESETS);
//! let mut i2c = i2c_pio::I2C::new(
//!     &mut pio,
//!     pins.gpio8,
//!     pins.gpio9,
//!     sm0,
//!     fugit::RateExtU32::kHz(100),
//!     clocks.system_clock.freq(),
//! );
//! let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
//! let mut alarm0 = timer.alarm_0().unwrap();
//!
//! let mut lcd = Lcd::new(&mut i2c, &mut alarm0)
//!     .address(LCD_ADDRESS)
//!     .cursor_on(false) // no visible cursos
//!     .rows(2) // two rows
//!     .init().unwrap();
//! ```
//!
//! This [site][lcd address] describes how to find the address of your LCD devices.
//!
//! [this one]: https://funduinoshop.com/elektronische-module/displays/lcd/16x02-i2c-lcd-modul-hintergrundbeleuchtung-blau
//! [lcd address]: https://www.ardumotive.com/i2clcden.html

use core::convert::Infallible;

use embedded_hal::blocking::i2c::Write;
use fugit::MicrosDurationU32;
use rp_pico::hal::timer::ScheduleAlarmError;

use {async_utils::Sleep, hal_exts::AlarmExt};

// use ufmt_write::uWrite;

/// API to write to the LCD.
pub struct Lcd<'a, I>
where
    I: Write,
{
    i2c: &'a mut I,
    address: u8,
    rows: u8,
    backlight_state: Backlight,
    cursor_on: bool,
    cursor_blink: bool,
}

pub enum DisplayControl {
    Off = 0x00,
    CursorBlink = 0x01,
    CursosOn = 0x02,
    DisplayOn = 0x04,
}

#[derive(Copy, Clone)]
pub enum Backlight {
    Off = 0x00,
    On = 0x08,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Mode {
    Cmd = 0x00,
    Data = 0x01,
    DisplayControl = 0x08,
    FunctionSet = 0x20,
}

enum Commands {
    Clear = 0x01,
    ReturnHome = 0x02,
    ShiftCursor = 16 | 4,
}

enum BitMode {
    Bit4 = 0x0 << 4,
    Bit8 = 0x1 << 4,
}

#[derive(Clone, Copy, Debug)]
pub enum LcdWriteError<E> {
    ScheduleAlarmError(ScheduleAlarmError),
    WriteError(E),
}

impl<E> From<ScheduleAlarmError> for LcdWriteError<E> {
    fn from(value: ScheduleAlarmError) -> Self {
        LcdWriteError::ScheduleAlarmError(value)
    }
}

impl<E> From<Infallible> for LcdWriteError<E> {
    fn from(value: Infallible) -> Self {
        match value {}
    }
}

impl From<i2c_pio::Error> for LcdWriteError<i2c_pio::Error> {
    fn from(value: i2c_pio::Error) -> Self {
        LcdWriteError::WriteError(value)
    }
}

impl<'a, I> Lcd<'a, I>
where
    I: Write,
    LcdWriteError<<I as Write>::Error>: From<<I as Write>::Error>,
{
    /// Create new instance with only the I2C and delay instance.
    pub fn new(i2c: &'a mut I) -> Self {
        Self {
            i2c,
            backlight_state: Backlight::On,
            address: 0,
            rows: 0,
            cursor_blink: false,
            cursor_on: false,
        }
    }

    /// Zero based number of rows.
    pub fn rows(mut self, rows: u8) -> Self {
        self.rows = rows;
        self
    }

    /// Set I2C address, see [lcd address].
    ///
    /// [lcd address]: https://badboi.dev/rust,/microcontrollers/2020/11/09/i2c-hello-world.html
    pub fn address(mut self, address: u8) -> Self {
        self.address = address;
        self
    }

    pub fn cursor_on(mut self, on: bool) -> Self {
        self.cursor_on = on;
        self
    }

    /// Initializes the hardware.
    ///
    /// Actual procedure is a bit obscure. This one was compiled from this [blog post],
    /// corresponding [code] and the [datasheet].
    ///
    /// [datasheet]: https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf
    /// [code]: https://github.com/jalhadi/i2c-hello-world/blob/main/src/main.rs
    /// [blog post]: https://badboi.dev/rust,/microcontrollers/2020/11/09/i2c-hello-world.html
    pub async fn init(
        mut self,
        alarm: &mut impl AlarmExt,
    ) -> Result<Lcd<'a, I>, LcdWriteError<<I as Write>::Error>> {
        // Initial delay to wait for init after power on.
        Sleep::new(alarm, MicrosDurationU32::millis(80))?.await;

        // Init with 8 bit mode
        let mode_8bit = Mode::FunctionSet as u8 | BitMode::Bit8 as u8;
        self.write4bits(mode_8bit, alarm).await?;
        Sleep::new(alarm, MicrosDurationU32::millis(5))?.await;
        self.write4bits(mode_8bit, alarm).await?;
        Sleep::new(alarm, MicrosDurationU32::millis(5))?.await;
        self.write4bits(mode_8bit, alarm).await?;
        Sleep::new(alarm, MicrosDurationU32::millis(5))?.await;

        // Switch to 4 bit mode
        let mode_4bit = Mode::FunctionSet as u8 | BitMode::Bit4 as u8;
        self.write4bits(mode_4bit, alarm).await?;

        // Function set command
        let lines = if self.rows == 0 { 0x00 } else { 0x08 };
        self.command(
            Mode::FunctionSet as u8 |
            // 5x8 display: 0x00, 5x10: 0x4
            lines, // Two line display
            alarm,
        )
        .await?;

        let display_ctrl = if self.cursor_on {
            DisplayControl::DisplayOn as u8 | DisplayControl::CursosOn as u8
        } else {
            DisplayControl::DisplayOn as u8
        };
        let display_ctrl = if self.cursor_blink {
            display_ctrl | DisplayControl::CursorBlink as u8
        } else {
            display_ctrl
        };
        self.command(Mode::DisplayControl as u8 | display_ctrl, alarm)
            .await?;
        self.command(Mode::Cmd as u8 | Commands::Clear as u8, alarm)
            .await?; // Clear Display

        // Entry right: shifting cursor moves to right
        self.command(0x04, alarm).await?;
        self.backlight(self.backlight_state, alarm).await?;
        Ok(self)
    }

    async fn write4bits(
        &mut self,
        data: u8,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        self.i2c.write(
            self.address,
            &[data | DisplayControl::DisplayOn as u8 | self.backlight_state as u8],
        )?;
        Sleep::new(alarm, MicrosDurationU32::millis(1))?.await;
        self.i2c.write(
            self.address,
            &[DisplayControl::Off as u8 | self.backlight_state as u8],
        )?;
        Sleep::new(alarm, MicrosDurationU32::millis(5))?.await;
        Ok(())
    }

    async fn send(
        &mut self,
        data: u8,
        mode: Mode,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        let high_bits: u8 = data & 0xf0;
        let low_bits: u8 = (data << 4) & 0xf0;
        self.write4bits(high_bits | mode as u8, alarm).await?;
        self.write4bits(low_bits | mode as u8, alarm).await?;
        Ok(())
    }

    async fn command(
        &mut self,
        data: u8,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        self.send(data, Mode::Cmd, alarm).await
    }

    pub async fn backlight(
        &mut self,
        backlight: Backlight,
        _alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        self.backlight_state = backlight;
        self.i2c.write(
            self.address,
            &[DisplayControl::DisplayOn as u8 | backlight as u8],
        )?;
        Ok(())
    }

    /// Write string to display.
    pub async fn write_str(
        &mut self,
        data: &str,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        for c in data.chars() {
            self.send(c as u8, Mode::Data, alarm).await?;
        }
        Ok(())
    }

    /// Clear the display
    pub async fn clear(
        &mut self,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        self.command(Commands::Clear as u8, alarm).await?;
        Sleep::new(alarm, MicrosDurationU32::millis(2))?.await;
        Ok(())
    }

    /// Return cursor to upper left corner, i.e. (0,0).
    pub async fn return_home(
        &mut self,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        self.command(Commands::ReturnHome as u8, alarm).await?;
        Sleep::new(alarm, MicrosDurationU32::millis(2))?.await;
        Ok(())
    }

    /// Set the cursor to (row, col). Coordinates are zero-based.
    pub async fn set_cursor(
        &mut self,
        row: u8,
        col: u8,
        alarm: &mut impl AlarmExt,
    ) -> Result<(), LcdWriteError<<I as Write>::Error>> {
        self.return_home(alarm).await?;
        let shift: u8 = row * 40 + col;
        for _i in 0..shift {
            self.command(Commands::ShiftCursor as u8, alarm).await?;
        }
        Ok(())
    }
}

// impl<'a, I, D> uWrite for Lcd<'a, I, D>
// where
//     I: Write,
//     D: DelayMs<u8>,
// {
//     type Error = <I as Write>::Error;

//     fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
//         self.write_str(s)
//     }
// }
