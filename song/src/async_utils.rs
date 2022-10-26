use core::cell::{RefCell, UnsafeCell};
use core::future::Future;
use core::mem::MaybeUninit;
pub use core::pin::Pin as StdPin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Poll, RawWaker, RawWakerVTable, Waker};

use crate::alloc_utils::Arc;
use crate::bsp;
use crate::hal_exts::AlarmExt;
use alloc::boxed::Box;
use alloc::collections::VecDeque;

use cortex_m::asm::wfi;
use critical_section::Mutex;
use fugit::Duration;
use rp_pico::hal::timer::ScheduleAlarmError;

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

pub struct Sleep<Alarm: AlarmExt> {
    alarm: Option<Alarm>,
    wake: Arc<(AtomicBool, Mutex<RefCell<Option<Waker>>>)>,
}

impl<Alarm: AlarmExt> Sleep<Alarm> {
    pub fn new<const NUM: u32, const DENOM: u32>(
        mut alarm: Alarm,
        countdown: Duration<u32, NUM, DENOM>,
    ) -> Result<Self, (ScheduleAlarmError, Alarm)> {
        alarm.disable_interrupt();
        alarm.clear_interrupt();
        unsafe {
            bsp::hal::pac::NVIC::unmask(Alarm::interrupt());
        };

        match alarm.schedule(countdown) {
            Ok(()) => {
                let wake: Arc<(AtomicBool, Mutex<RefCell<Option<Waker>>>)> =
                    Arc::new((AtomicBool::new(false), Mutex::new(RefCell::new(None))));
                alarm.register({
                    let wake = wake.clone();
                    Box::new(move || {
                        critical_section::with(|cs| {
                            wake.0.store(true, Ordering::Release);
                            if let Some(waker) = wake.1.borrow(cs).take() {
                                waker.wake();
                            }
                        })
                    })
                });

                alarm.enable_interrupt();

                Ok(Self {
                    alarm: Some(alarm),
                    wake,
                })
            }
            Err(err) => Err((err, alarm)),
        }
    }
}

impl<Alarm: AlarmExt> Future for Sleep<Alarm> {
    type Output = Alarm;

    fn poll(
        mut self: StdPin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        critical_section::with(|cs| {
            if self.wake.0.load(Ordering::Acquire) {
                let ret = Poll::Ready(
                    self.as_mut()
                        .alarm
                        .take()
                        .expect("Should not poll delay after completion"),
                );
                ret
            } else {
                self.wake.1.borrow(cs).replace(Some(cx.waker().clone()));

                Poll::Pending
            }
        })
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
