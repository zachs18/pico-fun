#![no_std]
extern crate alloc;

use core::cell::{Cell, RefCell, UnsafeCell};
use core::future::Future;
use core::mem::MaybeUninit;
pub use core::pin::Pin as StdPin;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use core::task::{Poll, RawWaker, RawWakerVTable, Waker};

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use arc_polyfill::Arc;
use hal_exts::AlarmExt;

use cortex_m::asm::wfi;
use critical_section::Mutex;
use fugit::MicrosDurationU32;
use rp_pico as board_support;
use rp_pico::hal::timer::ScheduleAlarmError;
use utils::AssertSendSync;

pub mod channel;

type Task = StdPin<Box<dyn Future<Output = ()>>>;

struct BasicWaker(AtomicBool);

pub struct Runtime {
    // syst: SYST,
    // frequency: u32,
    tasks: VecDeque<Task>,
    wake: Arc<BasicWaker>,
}

pub struct JoinHandle<T> {
    /// 0: finished
    /// 1: value
    value: Arc<(AtomicBool, UnsafeCell<MaybeUninit<T>>)>,
}

impl<T> JoinHandle<T> {
    pub fn get(self) -> Result<T, Self> {
        match self.value.0.load(Ordering::Acquire) {
            true => unsafe {
                let ptr = self.value.1.get();
                Ok((*ptr).assume_init_read())
            },
            false => Err(self),
        }
    }
}

impl Runtime {
    pub fn new(/* syst: SYST, frequency: u32 */) -> Self {
        Self {
            // syst,
            // frequency,
            tasks: VecDeque::new(),
            wake: Arc::new(BasicWaker(AtomicBool::new(false))),
        }
    }

    pub fn block_on<F: Future>(&mut self, mut future: F) -> F::Output {
        let mut future: StdPin<&mut F> = unsafe { StdPin::new_unchecked(&mut future) };
        let raw_waker = RawWaker::new(Arc::into_raw(self.wake.clone()).cast(), &VTABLE);
        let waker = unsafe { Waker::from_raw(raw_waker) };
        let mut cx = core::task::Context::from_waker(&waker);
        loop {
            match future.as_mut().poll(&mut cx) {
                core::task::Poll::Ready(val) => return val,
                core::task::Poll::Pending => {
                    if !self.tasks.is_empty() {
                        self.tasks
                            .retain_mut(|task| match task.as_mut().poll(&mut cx) {
                                core::task::Poll::Ready(_) => false,
                                core::task::Poll::Pending => true,
                            });
                        if !self.wake.0.load(Ordering::Acquire) {
                            wfi();
                        }
                    }
                }
            }
        }
    }

    pub fn spawn<F: Future>(&mut self, f: F) -> JoinHandle<F::Output>
    where
        F: 'static,
        F::Output: 'static,
    {
        let handle_value = Arc::new((
            AtomicBool::new(false),
            UnsafeCell::new(MaybeUninit::uninit()),
        ));
        let handle = JoinHandle {
            value: handle_value.clone(),
        };
        let task: StdPin<Box<dyn Future<Output = ()>>> = Box::pin(async move {
            let val = f.await;
            let ptr = handle_value.1.get();
            unsafe {
                (*ptr).write(val);
            }
            handle_value.0.store(true, Ordering::Release);
        });
        // let task: StdPin<Box<dyn Future<Output = ()> + 'static>> = unsafe { std::mem::transmute(task) };
        self.tasks.push_back(task);
        handle
    }

    pub fn run(&mut self) {
        let raw_waker = RawWaker::new(Arc::into_raw(self.wake.clone()).cast(), &VTABLE);
        let waker = unsafe { Waker::from_raw(raw_waker) };
        let mut cx = core::task::Context::from_waker(&waker);

        while !self.tasks.is_empty() {
            self.tasks
                .retain_mut(|task| match task.as_mut().poll(&mut cx) {
                    core::task::Poll::Ready(_) => false,
                    core::task::Poll::Pending => true,
                });
            if self.tasks.is_empty() {
                break;
            }
            if self.wake.0.load(Ordering::Acquire) {
                continue;
            }
            wfi();
        }
    }
}

pub struct Sleep<'a, Alarm: AlarmExt> {
    alarm: &'a mut Alarm,
    /// The boolean is true if the sleep is finished.
    /// The waker is Some if there is a task waiting.
    /// `hal_exts::AlarmExt::register` handles clearing the interrupt flag when it occurs.
    shared: Arc<(AtomicBool, Mutex<RefCell<Option<Waker>>>)>,
}

impl<'a, Alarm: AlarmExt> Sleep<'a, Alarm> {
    pub fn new(
        alarm: &'a mut Alarm,
        countdown: MicrosDurationU32,
    ) -> Result<Self, ScheduleAlarmError> {
        alarm.disable_interrupt();
        alarm.clear_interrupt();

        match alarm.schedule(countdown) {
            Ok(()) => {
                let shared: Arc<(AtomicBool, Mutex<RefCell<Option<Waker>>>)> =
                    Arc::new((AtomicBool::new(false), Mutex::new(RefCell::new(None))));
                let handler = {
                    let shared = shared.clone();
                    Box::new(move || {
                        critical_section::with(|cs| {
                            shared.0.store(true, Ordering::Release);
                            if let Some(waker) = shared.1.borrow(cs).take() {
                                waker.wake();
                            }
                        })
                    })
                };

                critical_section::with(|cs| {
                    alarm.register(cs, handler);
                    alarm.enable_interrupt();
                });

                unsafe {
                    board_support::hal::pac::NVIC::unmask(Alarm::interrupt());
                };

                Ok(Self { alarm, shared })
            }
            Err(err) => Err(err),
        }
    }
}

impl<'a, Alarm: AlarmExt> Future for Sleep<'a, Alarm> {
    type Output = ();

    fn poll(
        self: StdPin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if self.shared.0.load(Ordering::Acquire) {
            return Poll::Ready(());
        }
        critical_section::with(|cs| {
            if self.shared.0.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                self.shared.1.borrow(cs).replace(Some(cx.waker().clone()));

                Poll::Pending
            }
        })
    }
}

impl<'a, Alarm: AlarmExt> Drop for Sleep<'a, Alarm> {
    fn drop(&mut self) {
        critical_section::with(|cs| {
            self.alarm.disable_interrupt();
            self.alarm.clear_interrupt();
            board_support::hal::pac::NVIC::mask(Alarm::interrupt());
            if !self.shared.0.load(Ordering::Acquire) {
                // Remove the waker (Prob not necessary)
                self.shared.1.borrow(cs).replace(None);
            }
            self.alarm.unregister(cs);
        })
    }
}

struct IntervalShared<Alarm> {
    /// The number of intervals that have been passed but not yet awaited.
    /// Awaiting the interval will immediately use one of these if one is available.
    ///
    /// If `usize::MAX` ticks have passed, this will not count more ticks (i.e. it saturates at `usize::MAX`)
    ticks_passed: AtomicUsize,
    waker: Mutex<Cell<Option<Waker>>>,
    alarm: Mutex<RefCell<Option<AssertSendSync<Alarm>>>>,
}

pub struct Interval<Alarm: AlarmExt + 'static> {
    shared: Arc<IntervalShared<Alarm>>,
}

impl<Alarm: AlarmExt + 'static> Interval<Alarm> {
    pub fn new(mut alarm: Alarm, countdown: MicrosDurationU32) -> Result<Self, ScheduleAlarmError> {
        alarm.disable_interrupt();
        alarm.clear_interrupt();

        match alarm.schedule(countdown) {
            Ok(()) => {
                let shared = Arc::new(IntervalShared {
                    ticks_passed: AtomicUsize::new(0),
                    waker: Mutex::new(Cell::new(None)),
                    // SAFETY: alarm will only be accessed on this core.
                    alarm: Mutex::new(RefCell::new(Some(unsafe { AssertSendSync::new(alarm) }))),
                });
                let handler = {
                    let shared = shared.clone();
                    Box::new(move || {
                        critical_section::with(|cs| {
                            let prev = shared.ticks_passed.load(Ordering::Acquire);
                            shared
                                .ticks_passed
                                .store(prev.saturating_add(1), Ordering::Release);
                            if let Some(waker) = shared.waker.borrow(cs).take() {
                                waker.wake();
                            }
                            if let Some(alarm) = &mut *shared.alarm.borrow_ref_mut(cs) {
                                alarm
                                    .schedule(countdown)
                                    .expect("this countdown succeeded once, it should not fail");
                            }
                        })
                    })
                };

                critical_section::with(|cs| {
                    let mut alarm = shared.alarm.borrow_ref_mut(cs);
                    let alarm = alarm.as_mut().unwrap();
                    alarm.register(cs, handler);

                    alarm.enable_interrupt();
                });

                unsafe {
                    board_support::hal::pac::NVIC::unmask(Alarm::interrupt());
                };

                Ok(Self { shared })
            }
            Err(err) => Err(err),
        }
    }

    fn cancel_impl(&mut self) -> Alarm {
        critical_section::with(|cs| {
            let alarm = self.shared.alarm.take(cs);
            let mut alarm = alarm.unwrap().into_inner();
            alarm.disable_interrupt();
            alarm.clear_interrupt();
            board_support::hal::pac::NVIC::mask(Alarm::interrupt());
            alarm.unregister(cs);
            alarm
        })
    }

    pub fn cancel(mut self) -> Alarm {
        self.cancel_impl()
    }
}

impl<Alarm: AlarmExt + 'static> Future for &Interval<Alarm> {
    type Output = ();

    fn poll(self: StdPin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        critical_section::with(|cs| {
            let prev = self.shared.ticks_passed.load(Ordering::Acquire);
            if prev > 0 {
                self.shared.ticks_passed.store(prev - 1, Ordering::Release);
                Poll::Ready(())
            } else {
                let _old_waker = self
                    .shared
                    .waker
                    .borrow(cs)
                    .replace(Some(cx.waker().clone()));

                Poll::Pending
            }
        })
    }
}

impl<Alarm: AlarmExt + 'static> Drop for Interval<Alarm> {
    fn drop(&mut self) {
        self.cancel_impl();
    }
}

fn w_clone(ptr: *const ()) -> RawWaker {
    let ptr: *const BasicWaker = ptr.cast();
    unsafe {
        Arc::increment_strong_count(ptr);
    }
    RawWaker::new(ptr.cast(), &VTABLE)
}

fn w_wake(ptr: *const ()) {
    w_wake_by_ref(ptr);
    w_drop(ptr);
}

fn w_wake_by_ref(ptr: *const ()) {
    let ptr = ptr.cast::<BasicWaker>();
    unsafe {
        (*ptr).0.store(true, core::sync::atomic::Ordering::SeqCst);
    }
}

fn w_drop(ptr: *const ()) {
    let ptr: *const BasicWaker = ptr.cast();
    let arc = unsafe { Arc::from_raw(ptr) };
    drop(arc);
}

static VTABLE: RawWakerVTable = RawWakerVTable::new(w_clone, w_wake, w_wake_by_ref, w_drop);
