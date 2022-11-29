//! <https://github.com/mcauser/micropython-max7219/blob/master/max7219.py>

use core::mem::ManuallyDrop;

use cortex_m::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::{
    spi::{Enabled, SpiDevice},
    Spi,
};

use crate::noploop;

pub struct Max7219<'a, SD: SpiDevice, CSP: OutputPin, const COUNT: usize> {
    spi: &'a mut Spi<Enabled, SD, 16>,
    csp: Option<&'a mut CSP>,
}

#[derive(Debug)]
pub enum Error<SDE, CSPE> {
    SpiDeviceError(SDE),
    ChipSelectError(CSPE),
}

struct ChipSelectRAII<'a, CSP: OutputPin> {
    csp: &'a mut CSP,
}

impl<'a, CSP: OutputPin> ChipSelectRAII<'a, CSP> {
    fn new(csp: &'a mut CSP) -> Result<Self, CSP::Error> {
        csp.set_low()?;
        Ok(Self { csp })
    }

    fn free(self) -> Result<(), CSP::Error> {
        let mut this = ManuallyDrop::new(self);
        this.csp.set_high()
    }
}

impl<CSP: OutputPin> Drop for ChipSelectRAII<'_, CSP> {
    fn drop(&mut self) {
        let _ = self.csp.set_high();
    }
}
// TODO: repr(u8) enum
#[repr(u8)]
#[allow(unused)]
#[derive(Debug, Clone, Copy)]
enum Command {
    NOOP = 0,
    DIGIT0 = 1,
    DIGIT1 = 2,
    DIGIT2 = 3,
    DIGIT3 = 4,
    DIGIT4 = 5,
    DIGIT5 = 6,
    DIGIT6 = 7,
    DIGIT7 = 8,
    DECODEMODE = 9,
    INTENSITY = 10,
    SCANLIMIT = 11,
    SHUTDOWN = 12,
    DISPLAYTEST = 15,
}

impl Command {
    const fn digit(digit: u8) -> Option<Self> {
        if digit < 8 {
            let cmd = Self::DIGIT0 as u8 + digit;
            Some(unsafe { core::mem::transmute(cmd) })
        } else {
            None
        }
    }
}

impl<'a, SD: SpiDevice, CSP: OutputPin, const COUNT: usize> Max7219<'a, SD, CSP, COUNT> {
    pub fn new(
        spi: &'a mut Spi<Enabled, SD, 16>,
        mut csp: Option<&'a mut CSP>,
    ) -> Result<
        Self,
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        let mut this = match csp.as_mut().map(|csp| csp.set_high()).transpose() {
            Err(err) => Err(Error::ChipSelectError(err)),
            Ok(..) => Ok(Self { spi, csp }),
        }?;
        this.reinit()?;
        Ok(this)
    }

    pub fn reinit(
        &mut self,
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        use Command::*;
        for (command, data) in [
            (SHUTDOWN, 0),
            (DISPLAYTEST, 0),
            (SCANLIMIT, 7),
            (DECODEMODE, 0),
            (SHUTDOWN, 1),
        ] {
            self._write_all(command, data)?;
        }
        Ok(())
    }

    fn _write_all(
        &mut self,
        command: Command,
        data: u8,
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        noploop(10);
        let csp_raii = self
            .csp
            .as_deref_mut()
            .map(ChipSelectRAII::new)
            .transpose()
            .map_err(Error::ChipSelectError)?;
        for _m in 0..COUNT {
            let msg = (command as u16) << 8 | data as u16;
            self.spi.write(&[msg]).map_err(Error::SpiDeviceError)?;
        }
        csp_raii
            .map(ChipSelectRAII::free)
            .unwrap_or(Ok(()))
            .map_err(Error::ChipSelectError)
    }

    pub fn show(
        &mut self,
        value: &[[u8; 8]; COUNT],
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        for y in 0..8 {
            noploop(10);
            let csp_raii = self
                .csp
                .as_deref_mut()
                .map(ChipSelectRAII::new)
                .transpose()
                .map_err(Error::ChipSelectError)?;
            for m in 0..COUNT {
                let value: [u8; 8] = value[m];
                let msg = (Command::digit(y).unwrap() as u16) << 8 | value[y as usize] as u16;
                self.spi
                    .write(&[msg])
                    .map_err(Error::<_, _>::SpiDeviceError)?;
                // self._write_all(Command::digit(y).unwrap(), value[m][y as usize])?;
            }
            csp_raii
                .map(ChipSelectRAII::free)
                .unwrap_or(Ok(()))
                .map_err(Error::ChipSelectError)?;
        }
        Ok(())
    }

    pub fn display_test(
        &mut self,
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        self._write_all(Command::DISPLAYTEST, 1)
    }

    pub fn enable(
        &mut self,
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        self._write_all(Command::SHUTDOWN, 1)
    }

    pub fn disable(
        &mut self,
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 16> as embedded_hal::blocking::spi::Write<u16>>::Error, CSP::Error>,
    > {
        self._write_all(Command::SHUTDOWN, 0)
    }
}
