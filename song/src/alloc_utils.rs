use core::{alloc::GlobalAlloc, cell::RefCell, mem::MaybeUninit, ptr::NonNull};

pub use arc_polyfill::Arc;
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
                Err(_) => core::ptr::null_mut(),
            };
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
