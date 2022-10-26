use alloc::boxed::Box;
use core::cell::RefCell;

use crate::bsp::{self, hal::pac::interrupt};
use critical_section::Mutex;
use rp_pico::hal::timer::{Alarm, Alarm0, Alarm1, Alarm2, Alarm3};

pub trait AlarmExt: Alarm + Unpin {
    fn handler() -> &'static Mutex<RefCell<Option<Box<dyn FnOnce() + Send>>>>;
    fn interrupt() -> bsp::hal::pac::Interrupt;
    fn register(&mut self, handler: Box<dyn FnOnce() + Send>) {
        critical_section::with(|cs| Self::handler().borrow(cs).borrow_mut().replace(handler));
    }
    fn unregister(&mut self) -> Option<Box<dyn FnOnce() + Send>> {
        critical_section::with(|cs| Self::handler().borrow(cs).borrow_mut().take())
    }
    unsafe fn clear_interrupt_without_ownership();
}

fn handler<Alarm: AlarmExt>(handler: &Mutex<RefCell<Option<Box<dyn FnOnce() + Send>>>>) {
    critical_section::with(|cs| {
        if let Some(handler) = handler.borrow(cs).take() {
            handler();
        }

        // TODO: I thought the docs for `clear_interrupt` said that the interrupt wouldn't run again, but that appears to not be true.
        unsafe {
            Alarm::clear_interrupt_without_ownership();
        }
    });
}

macro_rules! make_timer_irq {
    ($interrupt:ident : $handler:ident : $alarm:ty) => {
        static $handler: Mutex<RefCell<Option<Box<dyn FnOnce() + Send>>>> =
            Mutex::new(RefCell::new(None));

        #[interrupt]
        fn $interrupt() {
            handler::<$alarm>(&$handler);
        }

        impl AlarmExt for $alarm {
            fn handler() -> &'static Mutex<RefCell<Option<Box<dyn FnOnce() + Send>>>> {
                &$handler
            }
            fn interrupt() -> bsp::hal::pac::Interrupt {
                bsp::hal::pac::Interrupt::$interrupt
            }
            unsafe fn clear_interrupt_without_ownership() {
                // TODO: I thought the docs for `clear_interrupt` said that the interrupt wouldn't run again, but that appears to not be true.
                let mut aaa: Self = unsafe { core::mem::transmute(()) };
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
