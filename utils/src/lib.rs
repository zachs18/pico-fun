#![no_std]

use core::{
    arch::asm,
    ops::{Deref, DerefMut},
};

/// Executes at least `10 * count` nops
pub fn noploop(count: usize) {
    for _ in 0..count {
        unsafe {
            asm!("nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",);
        }
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
