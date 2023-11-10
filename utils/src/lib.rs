#![no_std]

use core::{
    arch::asm,
    ops::{Deref, DerefMut},
};

use embedded_hal::digital::v2::{InputPin, OutputPin};

/// Executes at least `10 * count` nops
pub fn noploop(count: usize) {
    for _ in 0..count {
        unsafe {
            asm!("nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",);
        }
    }
}

pub struct AssertSendSync<T: ?Sized>(T);

unsafe impl<T> Send for AssertSendSync<T> {}
unsafe impl<T> Sync for AssertSendSync<T> {}

impl<T> AssertSendSync<T> {
    /// SAFETY: Caller must ensure that this value is actually sound to send and share between threads.
    pub unsafe fn new(value: T) -> Self
    where
        T: Sized,
    {
        AssertSendSync(value)
    }

    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T: ?Sized> Deref for AssertSendSync<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T: ?Sized> DerefMut for AssertSendSync<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub enum MaybeOwned<'a, T> {
    Owned(T),
    Borrowed(&'a mut T),
}

impl<'a, T> MaybeOwned<'a, T> {
    pub fn into_owned(self) -> Result<T, &'a mut T> {
        match self {
            MaybeOwned::Owned(t) => Ok(t),
            MaybeOwned::Borrowed(t) => Err(t),
        }
    }

    pub fn into_borrowed(self) -> Result<&'a mut T, T> {
        match self {
            MaybeOwned::Owned(t) => Err(t),
            MaybeOwned::Borrowed(t) => Ok(t),
        }
    }
}

impl<'a, T> Deref for MaybeOwned<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        match self {
            MaybeOwned::Owned(t) => t,
            MaybeOwned::Borrowed(t) => &**t,
        }
    }
}

impl<'a, T> DerefMut for MaybeOwned<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            MaybeOwned::Owned(t) => t,
            MaybeOwned::Borrowed(t) => &mut **t,
        }
    }
}

impl<'a, T> From<T> for MaybeOwned<'a, T> {
    fn from(value: T) -> Self {
        MaybeOwned::Owned(value)
    }
}

impl<'a, T> From<&'a mut T> for MaybeOwned<'a, T> {
    fn from(mut_ref: &'a mut T) -> Self {
        MaybeOwned::Borrowed(mut_ref)
    }
}

#[repr(transparent)]
pub struct InvertedPin<P: ?Sized> {
    pub pin: P,
}

impl<P: ?Sized> InvertedPin<P> {
    pub fn new(pin: P) -> Self
    where
        P: Sized,
    {
        Self { pin }
    }

    pub fn wrap_ref(pin: &P) -> &Self {
        unsafe { &*(pin as *const P as *const Self) }
    }

    pub fn wrap_mut(pin: &mut P) -> &mut Self {
        unsafe { &mut *(pin as *mut P as *mut Self) }
    }
}

impl<P: InputPin + ?Sized> InputPin for InvertedPin<P> {
    type Error = P::Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }
}

impl<P: OutputPin + ?Sized> OutputPin for InvertedPin<P> {
    type Error = P::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()
    }
}

pub trait PinExt {
    fn invert_pin(self) -> InvertedPin<Self>
    where
        Self: Sized,
    {
        InvertedPin { pin: self }
    }
    fn as_inverted_pin(&self) -> &InvertedPin<Self> {
        InvertedPin::wrap_ref(self)
    }
    fn as_inverted_pin_mut(&mut self) -> &mut InvertedPin<Self> {
        InvertedPin::wrap_mut(self)
    }
}
impl<P: ?Sized> PinExt for P {}
