//! <https://github.com/mcauser/micropython-max7219/blob/master/max7219.py>

use cortex_m::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::{
    spi::{Enabled, SpiDevice},
    Spi,
};

pub struct Max7219<'a, SD: SpiDevice, CSP: OutputPin, const COUNT: usize> {
    spi: &'a mut Spi<Enabled, SD, 8>,
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
        self.csp.set_high()
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
        spi: &'a mut Spi<Enabled, SD, 8>,
        mut csp: Option<&'a mut CSP>,
    ) -> Result<
        Self,
        Error<<Spi<Enabled, SD, 8> as embedded_hal::blocking::spi::Write<u8>>::Error, CSP::Error>,
    > {
        use Command::*;
        let mut this = match csp.as_mut().map(|csp| csp.set_high()).transpose() {
            Err(err) => Err(Error::ChipSelectError(err)),
            Ok(..) => Ok(Self { spi, csp }),
        }?;
        for (command, data) in [
            (SHUTDOWN, 0),
            (DISPLAYTEST, 0),
            (SCANLIMIT, 7),
            (DECODEMODE, 0),
            (SHUTDOWN, 1),
        ] {
            if let Err(err) = this._write(command, data) {
                return Err(err);
            }
        }
        Ok(this)
    }

    fn _write(
        &mut self,
        command: Command,
        data: u8,
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 8> as embedded_hal::blocking::spi::Write<u8>>::Error, CSP::Error>,
    > {
        let csp_raii = self
            .csp
            .as_deref_mut()
            .map(ChipSelectRAII::new)
            .transpose()
            .map_err(Error::ChipSelectError)?;
        for _m in 0..COUNT {
            self.spi
                .write(&[command as u8, data])
                .map_err(Error::<_, _>::SpiDeviceError)?;
        }
        csp_raii
            .map(ChipSelectRAII::free)
            .unwrap_or(Ok(()))
            .map_err(Error::ChipSelectError)
    }

    pub fn show(
        &mut self,
        value: &[u64; COUNT],
    ) -> Result<
        (),
        Error<<Spi<Enabled, SD, 8> as embedded_hal::blocking::spi::Write<u8>>::Error, CSP::Error>,
    > {
        for y in 0..8 {
            let csp_raii = self
                .csp
                .as_deref_mut()
                .map(ChipSelectRAII::new)
                .transpose()
                .map_err(Error::ChipSelectError)?;
            for m in 0..COUNT {
                let value: [u8; 8] = bytemuck::cast(value[m]);
                self.spi
                    .write(&[Command::digit(y).unwrap() as u8, value[y as usize]])
                    .map_err(Error::<_, _>::SpiDeviceError)?;
            }
            csp_raii
                .map(ChipSelectRAII::free)
                .unwrap_or(Ok(()))
                .map_err(Error::ChipSelectError)?;
        }
        Ok(())
    }
}
