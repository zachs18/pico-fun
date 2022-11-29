use core::{
    cell::RefCell,
    future::Future,
    pin::Pin as StdPin,
    task::{Poll, Waker},
};

use arc_polyfill::Arc;
use critical_section::Mutex;

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
