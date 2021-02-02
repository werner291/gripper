use std::marker::Send;
use std::ops::FnMut;
use std::sync::{Arc, Condvar, Mutex};

use std::clone::Clone;

/// A WaitStrategy is a callable that blocks the thread until
/// it is allowed to continue according to some policy.
///
pub trait WaitStrategy: FnMut() + Send + 'static {}
impl<W> WaitStrategy for W where W: FnMut() + Send + 'static {}

pub fn never_wait() {
    // Do nothing, just let the caller continue.
}

/// A simple synchronization strategy where one thread waits for another thread to allow it
/// to continue.
///
/// This function returns two values: a notifier, and a WaitStrategy.
///
/// The WaitStrategy will wait until the notifier has been called at least once since the last wait.
///
/// In between waiting moments, multiple calls to the notifier are equivalent to a single call,
/// so a lag spike in one thread will not cause a burst of activity later on.
///
pub fn continue_once_of_allowed() -> (impl FnMut(), impl WaitStrategy) {
    let wait_mutex = Arc::new((Mutex::new(false), Condvar::new()));
    let wait_mutex2 = wait_mutex.clone();

    let wait_strategy = move || {
        let (lock, cvar) = &*wait_mutex2;

        *cvar.wait_while(lock.lock().unwrap(), |wait| *wait).unwrap() = true;
    };

    let signal_strategy = move || {
        let (lock, cvar) = &*wait_mutex;
        *lock.lock().unwrap() = false;
        cvar.notify_all();
    };

    (signal_strategy, wait_strategy)
}
