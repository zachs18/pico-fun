//! <https://github.com/mcauser/micropython-max7219/blob/master/max7219.py>

use embedded_hal::digital::v2::OutputPin;

use hal_exts::OutputPinExt;

pub struct SevenSeg<
    'a,
    const COMMON_ANODE: bool,
    PinA: OutputPin,
    PinB: OutputPin<Error = PinA::Error>,
    PinC: OutputPin<Error = PinA::Error>,
    PinD: OutputPin<Error = PinA::Error>,
    PinE: OutputPin<Error = PinA::Error>,
    PinF: OutputPin<Error = PinA::Error>,
    PinG: OutputPin<Error = PinA::Error>,
    PinDot: OutputPin<Error = PinA::Error>,
> {
    a: &'a mut PinA,
    b: &'a mut PinB,
    c: &'a mut PinC,
    d: &'a mut PinD,
    e: &'a mut PinE,
    f: &'a mut PinF,
    g: &'a mut PinG,
    dot: Option<&'a mut PinDot>,
}

impl<
        'a,
        PinA: OutputPin,
        PinB: OutputPin<Error = PinA::Error>,
        PinC: OutputPin<Error = PinA::Error>,
        PinD: OutputPin<Error = PinA::Error>,
        PinE: OutputPin<Error = PinA::Error>,
        PinF: OutputPin<Error = PinA::Error>,
        PinG: OutputPin<Error = PinA::Error>,
        PinDot: OutputPin<Error = PinA::Error>,
    > SevenSeg<'a, true, PinA, PinB, PinC, PinD, PinE, PinF, PinG, PinDot>
{
    pub fn new_common_anode(
        a: &'a mut PinA,
        b: &'a mut PinB,
        c: &'a mut PinC,
        d: &'a mut PinD,
        e: &'a mut PinE,
        f: &'a mut PinF,
        g: &'a mut PinG,
        dot: Option<&'a mut PinDot>,
    ) -> Self {
        Self {
            a,
            b,
            c,
            d,
            e,
            f,
            g,
            dot,
        }
    }
}

impl<
        'a,
        PinA: OutputPin,
        PinB: OutputPin<Error = PinA::Error>,
        PinC: OutputPin<Error = PinA::Error>,
        PinD: OutputPin<Error = PinA::Error>,
        PinE: OutputPin<Error = PinA::Error>,
        PinF: OutputPin<Error = PinA::Error>,
        PinG: OutputPin<Error = PinA::Error>,
        PinDot: OutputPin<Error = PinA::Error>,
    > SevenSeg<'a, false, PinA, PinB, PinC, PinD, PinE, PinF, PinG, PinDot>
{
    pub fn new_common_cathode(
        a: &'a mut PinA,
        b: &'a mut PinB,
        c: &'a mut PinC,
        d: &'a mut PinD,
        e: &'a mut PinE,
        f: &'a mut PinF,
        g: &'a mut PinG,
        dot: Option<&'a mut PinDot>,
    ) -> Self {
        Self {
            a,
            b,
            c,
            d,
            e,
            f,
            g,
            dot,
        }
    }
}

impl<
        'a,
        const COMMON_ANODE: bool,
        PinA: OutputPin,
        PinB: OutputPin<Error = PinA::Error>,
        PinC: OutputPin<Error = PinA::Error>,
        PinD: OutputPin<Error = PinA::Error>,
        PinE: OutputPin<Error = PinA::Error>,
        PinF: OutputPin<Error = PinA::Error>,
        PinG: OutputPin<Error = PinA::Error>,
        PinDot: OutputPin<Error = PinA::Error>,
    > SevenSeg<'a, COMMON_ANODE, PinA, PinB, PinC, PinD, PinE, PinF, PinG, PinDot>
{
    pub fn new(
        a: &'a mut PinA,
        b: &'a mut PinB,
        c: &'a mut PinC,
        d: &'a mut PinD,
        e: &'a mut PinE,
        f: &'a mut PinF,
        g: &'a mut PinG,
        dot: Option<&'a mut PinDot>,
    ) -> Self {
        Self {
            a,
            b,
            c,
            d,
            e,
            f,
            g,
            dot,
        }
    }

    pub fn clear(&mut self) -> Result<(), PinA::Error> {
        self.write_byte(0)
    }

    pub fn write_byte(&mut self, byte: u8) -> Result<(), PinA::Error> {
        self.a.set((byte & 0b1000_0000 != 0) ^ COMMON_ANODE)?;
        self.b.set((byte & 0b0100_0000 != 0) ^ COMMON_ANODE)?;
        self.c.set((byte & 0b0010_0000 != 0) ^ COMMON_ANODE)?;
        self.d.set((byte & 0b0001_0000 != 0) ^ COMMON_ANODE)?;
        self.e.set((byte & 0b0000_1000 != 0) ^ COMMON_ANODE)?;
        self.f.set((byte & 0b0000_0100 != 0) ^ COMMON_ANODE)?;
        self.g.set((byte & 0b0000_0010 != 0) ^ COMMON_ANODE)?;
        if let Some(dot) = self.dot.as_mut() {
            dot.set((byte & 0b0000_0001 != 0) ^ COMMON_ANODE)?;
        }
        Ok(())
    }

    /// `digit` mut be a hexadecimal digit, i.e. `0 <= digit <= 15`.
    pub fn write_digit(&mut self, digit: u8) -> Result<(), PinA::Error> {
        const DIGITS: [u8; 16] = [
            0b1111_1100,
            0b0110_0000,
            0b1101_1010,
            0b1111_0010,
            0b0110_0110,
            0b1011_0110,
            0b1011_1110,
            0b1110_0000,
            0b1111_1110,
            0b1111_0110,
            0b1110_1110,
            0b0011_1110,
            0b1001_1100,
            0b0111_1010,
            0b1001_1110,
            0b1000_1110,
        ];
        self.write_byte(DIGITS[digit as usize & 0b1111])
    }

    // pub fn new(
    //     spi: &'a mut Spi<Enabled, SD, 8>,
    //     mut csp: Option<&'a mut CSP>,
    // ) -> Result<
    //     Self,
    //     Error<<Spi<Enabled, SD, 8> as embedded_hal::blocking::spi::Write<u8>>::Error, CSP::Error>,
    // > {
    //     use Command::*;
    //     let mut this = match csp.as_mut().map(|csp| csp.set_high()).transpose() {
    //         Err(err) => Err(Error::ChipSelectError(err)),
    //         Ok(..) => Ok(Self { spi, csp }),
    //     }?;
    //     for (command, data) in [
    //         (SHUTDOWN, 0),
    //         (DISPLAYTEST, 0),
    //         (SCANLIMIT, 7),
    //         (DECODEMODE, 0),
    //         (SHUTDOWN, 1),
    //     ] {
    //         if let Err(err) = this._write(command, data) {
    //             return Err(err);
    //         }
    //     }
    //     Ok(this)
    // }

    // fn _write(
    //     &mut self,
    //     command: Command,
    //     data: u8,
    // ) -> Result<
    //     (),
    //     Error<<Spi<Enabled, SD, 8> as embedded_hal::blocking::spi::Write<u8>>::Error, CSP::Error>,
    // > {
    //     let csp_raii = self
    //         .csp
    //         .as_deref_mut()
    //         .map(ChipSelectRAII::new)
    //         .transpose()
    //         .map_err(Error::ChipSelectError)?;
    //     for _m in 0..COUNT {
    //         self.spi
    //             .write(&[command as u8, data])
    //             .map_err(Error::<_, _>::SpiDeviceError)?;
    //     }
    //     csp_raii
    //         .map(ChipSelectRAII::free)
    //         .unwrap_or(Ok(()))
    //         .map_err(Error::ChipSelectError)
    // }

    // pub fn show(
    //     &mut self,
    //     value: &[u64; COUNT],
    // ) -> Result<
    //     (),
    //     Error<<Spi<Enabled, SD, 8> as embedded_hal::blocking::spi::Write<u8>>::Error, CSP::Error>,
    // > {
    //     for y in 0..8 {
    //         let csp_raii = self
    //             .csp
    //             .as_deref_mut()
    //             .map(ChipSelectRAII::new)
    //             .transpose()
    //             .map_err(Error::ChipSelectError)?;
    //         for m in 0..COUNT {
    //             let value: [u8; 8] = bytemuck::cast(value[m]);
    //             self.spi
    //                 .write(&[Command::digit(y).unwrap() as u8, value[y as usize]])
    //                 .map_err(Error::<_, _>::SpiDeviceError)?;
    //         }
    //         csp_raii
    //             .map(ChipSelectRAII::free)
    //             .unwrap_or(Ok(()))
    //             .map_err(Error::ChipSelectError)?;
    //     }
    //     Ok(())
    // }
}
