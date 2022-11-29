#![no_std]
#![feature(layout_for_ptr)]
#![feature(alloc_layout_extra)]
#![feature(pointer_byte_offsets)]
#![cfg_attr(feature = "unsize", feature(unsize))]
#![cfg_attr(feature = "unsize", feature(coerce_unsized))]
#![deny(unsafe_op_in_unsafe_fn)]
//! Polyfilled/vendored subset of [`alloc::alloc::Arc`] using [`critical_section`].
//! See standard library for docs of relevant functions.

extern crate alloc;

use alloc::alloc::handle_alloc_error;
use core::{
    alloc::Layout,
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
};

#[cfg(feature = "unsize")]
use core::marker::Unsize;
#[cfg(feature = "unsize")]
use core::ops::CoerceUnsized;

// This is repr(C) to future-proof against possible field-reordering, which
// would interfere with otherwise safe [into|from]_raw() of transmutable
// inner types.
#[repr(C)]
struct ArcInner<T: ?Sized> {
    strong: AtomicUsize,
    data: T,
}

#[cfg(feature = "unsize")]
impl<T: ?Sized + Unsize<U>, U: ?Sized> CoerceUnsized<Arc<U>> for Arc<T> {}

/// Thread- and interrupt-safe reference-counted pointer to `T`, using [`critical_section`] whenever
/// modifying anything atomic, to allow for platforms without atomic compare-and-swap.
pub struct Arc<T: ?Sized> {
    data: NonNull<ArcInner<T>>,
    _phantom: PhantomData<ArcInner<T>>,
}

unsafe impl<T: Send + Sync> Send for Arc<T> {}
unsafe impl<T: Send + Sync> Sync for Arc<T> {}

impl<T> Arc<T> {
    pub fn new(value: T) -> Arc<T> {
        let layout = Layout::new::<ArcInner<T>>();
        let ptr = unsafe { alloc::alloc::alloc(layout) };
        if let Some(ptr) = NonNull::new(ptr.cast::<ArcInner<T>>()) {
            unsafe {
                ptr.as_ptr().write(ArcInner {
                    strong: 1.into(),
                    data: value,
                });
            }

            Arc {
                data: ptr.cast(),
                _phantom: PhantomData,
            }
        } else {
            handle_alloc_error(layout)
        }
    }

    pub fn try_unwrap(this: Self) -> Result<T, Self> {
        let ptr = this.data.as_ptr() as *const ArcInner<T>;
        let strong = unsafe { &(*ptr).strong };
        let old_strong = strong.load(Ordering::Acquire);

        let can_unwrap = if old_strong == 1 {
            // SAFETY: This Arc does not support Weak, so if strong was observed to be 1,
            // we know we are the last owner, so we can unwrap it.
            true
        } else {
            critical_section::with(|_cs| {
                let old_strong = strong.load(Ordering::Acquire);
                if old_strong > 1 {
                    // SAFETY: This Arc is aliased, so we cannot unwrap it.
                    false
                } else {
                    // SAFETY: This Arc does not support Weak, so if strong was observed to be 1,
                    // we know we are the last owner, so we can unwrap it.
                    true
                }
            })
        };

        if can_unwrap {
            unsafe {
                // Convert to Arc<MaybeUninit<T>> to deallocate but not drop.
                let this: Arc<MaybeUninit<T>> =
                    Arc::from_raw(Arc::into_raw(this) as *const MaybeUninit<T>);
                let value: T = this.assume_init_read();
                Ok(value)
            }
        } else {
            Err(this)
        }
    }
}

impl<T: ?Sized> Arc<T> {
    /// Gets the number of strong (`Arc`) pointers to this allocation.
    ///
    /// # Safety
    ///
    /// This method by itself is safe, but using it correctly requires extra care.
    /// Another thread can change the strong count,
    /// including potentially between calling this method and acting on the result.
    #[inline]
    #[must_use]
    pub fn strong_count(this: &Self) -> usize {
        unsafe { &*this.data.as_ptr() }
            .strong
            .load(Ordering::Acquire)
    }

    pub fn as_ptr(this: &Self) -> *const T {
        let ptr: *mut ArcInner<T> = this.data.as_ptr();
        unsafe { core::ptr::addr_of_mut!((*ptr).data) }
    }

    pub fn into_raw(self) -> *const T {
        let this = ManuallyDrop::new(self);
        Arc::as_ptr(&this)
    }

    pub unsafe fn from_raw(ptr: *const T) -> Self {
        unsafe {
            let offset = data_offset(ptr);

            let arc_ptr = ptr.byte_sub(offset) as *mut ArcInner<T>;

            Self {
                data: NonNull::new(arc_ptr).unwrap(),
                _phantom: PhantomData,
            }
        }
    }

    pub unsafe fn increment_strong_count(ptr: *const T) {
        let arc: ManuallyDrop<_> = ManuallyDrop::new(unsafe { Self::from_raw(ptr) });
        let _arc_clone: ManuallyDrop<_> = arc.clone();
    }

    pub unsafe fn decrement_strong_count(ptr: *const T) {
        drop(unsafe { Self::from_raw(ptr) })
    }
}

impl<T: ?Sized> Drop for Arc<T> {
    fn drop(&mut self) {
        let ptr = self.data.as_ptr() as *const ArcInner<T>;
        let strong = unsafe { &(*ptr).strong };
        let old_strong = strong.load(Ordering::Acquire);

        let should_drop_inner = if old_strong == 1 {
            // SAFETY: This Arc does not support Weak, so if strong was observed to be 1,
            // we know we are the last owner.
            true
        } else {
            critical_section::with(|_cs| {
                let old_strong = strong.load(Ordering::Acquire);
                if old_strong > 1 {
                    // SAFETY: This Arc is aliased, but we are in a critical section, so no other code can modify strong.
                    // Decrement strong, and do not drop inner;
                    let new_strong = old_strong.checked_sub(1).unwrap();
                    strong.store(new_strong, Ordering::Release);
                    false
                } else {
                    // SAFETY: This Arc does not support Weak, so if strong was observed to be 1,
                    // we know we are the last owner. We drop outside of the critical section,
                    // which is correct because Weak is not supported.
                    true
                }
            })
        };

        if should_drop_inner {
            unsafe {
                let ptr = ptr as *mut ArcInner<T>;
                let layout = Layout::for_value(&*ptr);
                core::ptr::drop_in_place(ptr);
                alloc::alloc::dealloc(ptr.cast(), layout);
            }
        }
    }
}

impl<T: ?Sized> Clone for Arc<T> {
    fn clone(&self) -> Self {
        let ptr = self.data.as_ptr();
        let strong = unsafe { &(*ptr).strong };
        critical_section::with(|_cs| {
            // SAFETY: We are in a critical section, so no other code can modify strong.
            let old_strong = strong.load(Ordering::Acquire);
            let new_strong = old_strong.checked_add(1).unwrap();
            strong.store(new_strong, Ordering::Release);
        });
        Self {
            data: self.data.clone(),
            _phantom: PhantomData,
        }
    }
}

impl<T: ?Sized> core::ops::Deref for Arc<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe {
            let ptr = self.data.as_ptr();
            &(*ptr).data
        }
    }
}

/// Get the offset within an `ArcInner` for the payload behind a pointer.
///
/// # Safety
///
/// The pointer must point to (and have valid metadata for) a previously
/// valid instance of T, but the T is allowed to be dropped.
unsafe fn data_offset<T: ?Sized>(ptr: *const T) -> usize {
    // Align the unsized value to the end of the ArcInner.
    // Because ArcInner is repr(C), it will always be the last field in memory.
    // SAFETY: since the only unsized types possible are slices, trait objects,
    // and extern types, the input safety requirement is currently enough to
    // satisfy the requirements of align_of_val_raw; this is an implementation
    // detail of the language that must not be relied upon outside of std.
    unsafe { data_offset_align(core::mem::align_of_val_raw(ptr)) }
}

#[inline]
fn data_offset_align(align: usize) -> usize {
    let layout = Layout::new::<ArcInner<()>>();
    layout.size() + layout.padding_needed_for(align)
}
