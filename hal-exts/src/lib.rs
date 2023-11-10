#![no_std]
extern crate alloc;

use alloc::boxed::Box;
use core::cell::RefCell;
use core::mem::ManuallyDrop;
use embedded_hal::digital::v2::OutputPin;

use critical_section::{CriticalSection, Mutex};

use rp_pico::hal::{
    pac::interrupt,
    timer::{Alarm, Alarm0, Alarm1, Alarm2, Alarm3},
};

/// I fully understand why `set_state` takes a `PinState` and not a `bool`, but it's annoying so I did this.
pub trait OutputPinExt: OutputPin {
    fn set(&mut self, high: bool) -> Result<(), Self::Error>;
}

impl<P: OutputPin + ?Sized> OutputPinExt for P {
    fn set(&mut self, high: bool) -> Result<(), Self::Error> {
        match high {
            false => self.set_low(),
            true => self.set_high(),
        }
    }
}
pub trait AlarmExt: Alarm + Unpin {
    fn handler() -> &'static Mutex<RefCell<Option<Box<dyn FnMut() + Send>>>>;
    fn interrupt() -> rp_pico::hal::pac::Interrupt;
    fn register(&mut self, cs: CriticalSection, handler: Box<dyn FnMut() + Send>) {
        unsafe { Self::register_without_ownership(cs, handler) }
    }
    fn unregister(&mut self, cs: CriticalSection) -> Option<Box<dyn FnMut() + Send>> {
        unsafe { Self::unregister_without_ownership(cs) }
    }
    unsafe fn clear_interrupt_without_ownership();
    unsafe fn register_without_ownership(cs: CriticalSection, handler: Box<dyn FnMut() + Send>) {
        Self::handler().borrow_ref_mut(cs).replace(handler);
    }
    unsafe fn unregister_without_ownership(cs: CriticalSection) -> Option<Box<dyn FnMut() + Send>> {
        Self::handler().borrow_ref_mut(cs).take()
    }
}

fn handle_with<Alarm: AlarmExt>(handler: &Mutex<RefCell<Option<Box<dyn FnMut() + Send>>>>) {
    critical_section::with(|cs| {
        if let Some(handler) = handler.borrow_ref_mut(cs).as_deref_mut() {
            handler();
        }

        // TODO: I thought the docs for `clear_interrupt` meant that the irq wouldn't run again, but that appears to not be true.
        unsafe {
            Alarm::clear_interrupt_without_ownership();
        }
    });
}

macro_rules! make_timer_irq {
    ($interrupt:ident : $handler:ident : $alarm:ty) => {
        static $handler: Mutex<RefCell<Option<Box<dyn FnMut() + Send>>>> =
            Mutex::new(RefCell::new(None));

        #[allow(non_snake_case)]
        #[interrupt]
        fn $interrupt() {
            handle_with::<$alarm>(&$handler);
        }

        impl AlarmExt for $alarm {
            fn handler() -> &'static Mutex<RefCell<Option<Box<dyn FnMut() + Send>>>> {
                &$handler
            }
            fn interrupt() -> rp_pico::hal::pac::Interrupt {
                rp_pico::hal::pac::Interrupt::$interrupt
            }
            unsafe fn clear_interrupt_without_ownership() {
                // TODO: I thought the docs for `clear_interrupt` meant that the irq wouldn't run again, but that appears to not be true.
                let mut aaa: ManuallyDrop<Self> = unsafe { core::mem::transmute(()) };
                aaa.clear_interrupt();
            }
        }
    };
}

make_timer_irq!(TIMER_IRQ_0: TIMER_IRQ_0_HANDLER: Alarm0);
make_timer_irq!(TIMER_IRQ_1: TIMER_IRQ_1_HANDLER: Alarm1);
make_timer_irq!(TIMER_IRQ_2: TIMER_IRQ_2_HANDLER: Alarm2);
make_timer_irq!(TIMER_IRQ_3: TIMER_IRQ_3_HANDLER: Alarm3);

// static TIMER_IRQ_0_HANDLER: Mutex<RefCell<Option<Box<dyn FnOnce() + Send>>>> =
//     Mutex::new(RefCell::new(None));

// #[interrupt]
// fn TIMER_IRQ_0() {
//     static mut HANDLER: Option<Box<dyn FnOnce()>> = None;

//     critical_section::with(|cs| {
//         if let Some(new_handler) = TIMER_IRQ_0_HANDLER.borrow(cs).take() {
//             *HANDLER = Some(new_handler)
//         }
//     });

//     if let Some(handler) = HANDLER {
//         handler();
//     }
// }
