use core::cell::{RefCell, UnsafeCell};
use core::future::Future;
use core::mem::MaybeUninit;
pub use core::pin::Pin as StdPin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Poll, RawWaker, RawWakerVTable, Waker};

use crate::alloc_utils::Arc;
use crate::board_support;
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

pub struct Sleep<'a, Alarm: AlarmExt> {
    alarm: &'a mut Alarm,
    wake: Arc<(AtomicBool, Mutex<RefCell<Option<Waker>>>)>,
}

impl<'a, Alarm: AlarmExt> Sleep<'a, Alarm> {
    pub fn new<const NUM: u32, const DENOM: u32>(
        alarm: &'a mut Alarm,
        countdown: Duration<u32, NUM, DENOM>,
    ) -> Result<Self, ScheduleAlarmError> {
        alarm.disable_interrupt();
        alarm.clear_interrupt();

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

                unsafe {
                    board_support::hal::pac::NVIC::unmask(Alarm::interrupt());
                };
                alarm.enable_interrupt();

                Ok(Self { alarm, wake })
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
        critical_section::with(|cs| {
            if self.wake.0.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                self.wake.1.borrow(cs).replace(Some(cx.waker().clone()));

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
            if !self.wake.0.load(Ordering::Acquire) {
                // Remove the waker (Prob not necessary)
                self.wake.1.borrow(cs).replace(None);
            }
            self.alarm.unregister();
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

struct ChannelInner<T> {
    value: Option<T>,
    /// Sender's waker if waiting XOR receiver's waker if waiting.
    /// Sender and Receiver are not Clone, so there can never be two things waiting at the same time.
    waker: Option<Waker>,
}

pub struct Sender<T> {
    inner: Arc<Mutex<RefCell<ChannelInner<T>>>>,
}

pub struct Send<'a, T> {
    value: Option<T>,
    inner: &'a Arc<Mutex<RefCell<ChannelInner<T>>>>,
}

impl<T> Unpin for Send<'_, T> {}

pub struct Receiver<T> {
    inner: Arc<Mutex<RefCell<ChannelInner<T>>>>,
}

/// The sender disconnected.
pub struct RecvError;

/// The sender was dropped, or a value was not immediately avaiable.
pub enum TryRecvError {
    Empty,
    Disconnected,
}

/// The receiver disconnected.
pub struct SendError<T>(T);

/// The receiver was dropped, or a the channel was full.
pub enum TrySendError<T> {
    Full(T),
    Disconnected(T),
}

impl<T> Future for &Receiver<T> {
    type Output = Result<T, RecvError>;

    fn poll(self: StdPin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        critical_section::with(|cs| {
            if Arc::strong_count(&self.inner) == 1 {
                return Poll::Ready(Err(RecvError));
            }
            let mut inner = self.inner.borrow_ref_mut(cs);
            match inner.value.take() {
                Some(value) => {
                    // If sender is waiting to send, wake them when the value is free
                    if let Some(waker) = inner.waker.take() {
                        waker.wake();
                    }
                    Poll::Ready(Ok(value))
                }
                None => {
                    // Replace our previous waker with this one.
                    let _oldwaker = inner.waker.replace(cx.waker().clone());
                    Poll::Pending
                }
            }
        })
    }
}

impl<T> Receiver<T> {
    pub fn try_recv(&self) -> Result<T, TryRecvError> {
        critical_section::with(|cs| {
            if Arc::strong_count(&self.inner) == 1 {
                return Err(TryRecvError::Disconnected);
            }
            let mut inner = self.inner.borrow_ref_mut(cs);
            match inner.value.take() {
                Some(value) => {
                    // If sender is waiting to send, wake them when the value is free
                    if let Some(waker) = inner.waker.take() {
                        waker.wake();
                    }
                    Ok(value)
                }
                None => Err(TryRecvError::Empty),
            }
        })
    }

    pub fn recv(&self) -> impl Future<Output = Result<T, RecvError>> + '_ {
        self
    }
}

impl<'a, T> Future for Send<'a, T> {
    type Output = Result<(), SendError<T>>;

    fn poll(mut self: StdPin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        critical_section::with(|cs| {
            if self.value.is_some() {
                if Arc::strong_count(self.inner) == 1 {
                    return Poll::Ready(Err(SendError(self.value.take().unwrap())));
                }
                let mut inner = self.inner.borrow_ref_mut(cs);
                if inner.value.is_some() {
                    // Replace our previous waker with this one.
                    let _oldwaker = inner.waker.replace(cx.waker().clone());
                    Poll::Pending
                } else {
                    inner.value = self.value.take();
                    // If receiver is waiting to recv, wake them when the value is available.
                    if let Some(waker) = inner.waker.take() {
                        waker.wake();
                    }
                    Poll::Ready(Ok(()))
                }
            } else {
                Poll::Ready(Ok(()))
            }
        })
    }
}

impl<T> Sender<T> {
    pub fn send(&self, value: T) -> impl Future<Output = Result<(), SendError<T>>> + '_ {
        Send {
            value: Some(value),
            inner: &self.inner,
        }
    }

    pub fn try_send(&self, value: T) -> Result<(), TrySendError<T>> {
        critical_section::with(|cs| {
            if Arc::strong_count(&self.inner) == 1 {
                return Err(TrySendError::Disconnected(value));
            }
            let mut inner = self.inner.borrow_ref_mut(cs);
            if inner.value.is_some() {
                Err(TrySendError::Full(value))
            } else {
                inner.value = Some(value);
                // If receiver is waiting to recv, wake them when the value is available.
                if let Some(waker) = inner.waker.take() {
                    waker.wake();
                }
                Ok(())
            }
        })
    }
}

pub fn capacity_1_channel<T>() -> (Sender<T>, Receiver<T>) {
    let inner = Arc::new(Mutex::new(RefCell::new(ChannelInner {
        value: None,
        waker: None,
    })));
    (
        Sender {
            inner: inner.clone(),
        },
        Receiver { inner },
    )
}
