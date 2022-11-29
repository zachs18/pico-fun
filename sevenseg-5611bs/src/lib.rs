#![no_std]
//! <https://github.com/mcauser/micropython-max7219/blob/master/max7219.py>

use embedded_hal::digital::v2::OutputPin;
use hal_exts::OutputPinExt;
use utils::MaybeOwned;

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
    a: MaybeOwned<'a, PinA>,
    b: MaybeOwned<'a, PinB>,
    c: MaybeOwned<'a, PinC>,
    d: MaybeOwned<'a, PinD>,
    e: MaybeOwned<'a, PinE>,
    f: MaybeOwned<'a, PinF>,
    g: MaybeOwned<'a, PinG>,
    dot: Option<MaybeOwned<'a, PinDot>>,
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
        a: MaybeOwned<'a, PinA>,
        b: MaybeOwned<'a, PinB>,
        c: MaybeOwned<'a, PinC>,
        d: MaybeOwned<'a, PinD>,
        e: MaybeOwned<'a, PinE>,
        f: MaybeOwned<'a, PinF>,
        g: MaybeOwned<'a, PinG>,
        dot: Option<MaybeOwned<'a, PinDot>>,
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
        a: MaybeOwned<'a, PinA>,
        b: MaybeOwned<'a, PinB>,
        c: MaybeOwned<'a, PinC>,
        d: MaybeOwned<'a, PinD>,
        e: MaybeOwned<'a, PinE>,
        f: MaybeOwned<'a, PinF>,
        g: MaybeOwned<'a, PinG>,
        dot: Option<MaybeOwned<'a, PinDot>>,
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
        a: MaybeOwned<'a, PinA>,
        b: MaybeOwned<'a, PinB>,
        c: MaybeOwned<'a, PinC>,
        d: MaybeOwned<'a, PinD>,
        e: MaybeOwned<'a, PinE>,
        f: MaybeOwned<'a, PinF>,
        g: MaybeOwned<'a, PinG>,
        dot: Option<MaybeOwned<'a, PinDot>>,
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

    pub fn into_inner(
        self,
    ) -> (
        MaybeOwned<'a, PinA>,
        MaybeOwned<'a, PinB>,
        MaybeOwned<'a, PinC>,
        MaybeOwned<'a, PinD>,
        MaybeOwned<'a, PinE>,
        MaybeOwned<'a, PinF>,
        MaybeOwned<'a, PinG>,
        Option<MaybeOwned<'a, PinDot>>,
    ) {
        (
            self.a, self.b, self.c, self.d, self.e, self.f, self.g, self.dot,
        )
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
}
