use core::{
    alloc::{GlobalAlloc, Layout},
    cell::RefCell,
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
};

use alloc::alloc::handle_alloc_error;
use critical_section::Mutex;
use linked_list_allocator::Heap;

pub struct Allocator {
    heap: Mutex<RefCell<Heap>>,
}

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        critical_section::with(|cs| {
            let mut heap = self.heap.borrow(cs).borrow_mut();
            let ptr = match heap.allocate_first_fit(layout) {
                Ok(ptr) => ptr.as_ptr(),
                Err(_) => {
                    crate::PANIC_LINE.store(line!(), Ordering::Release);
                    core::ptr::null_mut()
                }
            };
            crate::MEM_FREE.store(heap.free(), Ordering::Release);
            ptr
        })
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: core::alloc::Layout) {
        critical_section::with(|cs| {
            let mut heap = self.heap.borrow(cs).borrow_mut();
            unsafe {
                heap.deallocate(NonNull::new(ptr).unwrap(), layout);
            }
        })
    }
}

impl Allocator {
    pub const fn empty() -> Self {
        Self {
            heap: Mutex::new(RefCell::new(Heap::empty())),
        }
    }

    pub fn init_from_slice(&self, mem: &'static mut [MaybeUninit<u8>]) {
        critical_section::with(|cs| {
            let mut heap = self.heap.borrow(cs).borrow_mut();
            heap.init_from_slice(mem);
        })
    }
}

#[repr(C)] // so we can use Layout::extend, etc.
struct ArcInner<T: ?Sized> {
    count: AtomicUsize,
    data: T,
}

pub struct Arc<T: ?Sized> {
    data: NonNull<ArcInner<T>>,
    _phantom: PhantomData<ArcInner<T>>,
}

unsafe impl<T: Send> Send for Arc<T> {}

impl<T> Arc<T> {
    pub fn new(value: T) -> Arc<T> {
        let layout = Layout::new::<ArcInner<T>>();
        let ptr = unsafe { alloc::alloc::alloc(layout) };
        if let Some(ptr) = NonNull::new(ptr.cast::<ArcInner<T>>()) {
            unsafe {
                ptr.as_ptr().write(ArcInner {
                    count: 1.into(),
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

    pub unsafe fn from_raw(this: *const T) -> Self {
        let header_layout = Layout::new::<AtomicUsize>();
        let data_layout = unsafe { Layout::for_value(&*this) };
        let (_, offset) = header_layout.extend(data_layout).unwrap();
        let ptr = this.cast::<u8>();
        let data = unsafe {
            let data = ptr.sub(offset).cast::<ArcInner<T>>();
            NonNull::new(data as *mut ArcInner<T>).unwrap()
        };
        Self {
            data: data,
            _phantom: PhantomData,
        }
    }
}
impl<T: ?Sized> Arc<T> {
    pub fn as_ptr(this: &Self) -> *const T {
        let ptr: *mut ArcInner<T> = this.data.as_ptr();
        unsafe { core::ptr::addr_of_mut!((*ptr).data) }
    }

    pub fn into_raw(self) -> *const T {
        let this = ManuallyDrop::new(self);
        Arc::as_ptr(&this)
    }

    pub unsafe fn increment_strong_count(this: *const T) {
        let header_layout = Layout::new::<AtomicUsize>();
        let data_layout = unsafe { Layout::for_value(&*this) };
        let (_, offset) = header_layout.extend(data_layout).unwrap();
        let ptr = this.cast::<u8>();
        let count = unsafe { &*ptr.sub(offset).cast::<AtomicUsize>() };
        critical_section::with(|_cs| {
            let old_count = count.load(Ordering::Acquire);
            let new_count = old_count.checked_add(1).unwrap();
            count.store(new_count, Ordering::Release);
        });
    }
}

impl<T: ?Sized> Drop for Arc<T> {
    fn drop(&mut self) {
        let ptr = self.data.as_ptr() as *const ArcInner<T>;
        let count = unsafe { &(*ptr).count };
        let old_count = count.load(Ordering::Acquire);
        if old_count == 1 {
            unsafe {
                let ptr = ptr as *mut ArcInner<T>;
                let layout = Layout::for_value(&*ptr);
                core::ptr::drop_in_place(ptr);
                alloc::alloc::dealloc(ptr.cast(), layout);
            }
        } else {
            critical_section::with(|_cs| {
                let old_count = count.load(Ordering::Acquire);
                if old_count == 1 {
                    // TODO: move this outside of critical section
                    unsafe {
                        let ptr = ptr as *mut ArcInner<T>;
                        let layout = Layout::for_value(&*ptr);
                        core::ptr::drop_in_place(ptr);
                        alloc::alloc::dealloc(ptr.cast(), layout);
                    }
                } else {
                    critical_section::with(|_cs| {
                        let new_count = old_count.checked_add(1).unwrap();
                        count.store(new_count, Ordering::Release);
                    });
                }
            });
        }
    }
}

impl<T: ?Sized> Clone for Arc<T> {
    fn clone(&self) -> Self {
        let ptr = self.data.as_ptr();
        let count = unsafe { &(*ptr).count };
        critical_section::with(|_cs| {
            let old_count = count.load(Ordering::Acquire);
            let new_count = old_count.checked_add(1).unwrap();
            count.store(new_count, Ordering::Release);
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
