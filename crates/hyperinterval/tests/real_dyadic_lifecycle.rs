#![cfg(feature = "hyperreal")]

#[path = "support/real_dyadic_lifecycle.rs"]
mod lifecycle;

use lifecycle::{check_bounds, check_cancelled, translated_root};
use std::sync::{
    Arc, Barrier,
    atomic::{AtomicBool, Ordering},
};

#[test]
fn a_cancelled_dyadic_reader_can_retry_after_an_independent_reader_refines() {
    lifecycle::check_retry();
}

#[test]
fn concurrent_readers_share_the_translated_root_while_a_cancelled_clone_retries() {
    let value = translated_root();
    let mut cancelled = value.clone();
    let stop = Arc::new(AtomicBool::new(true));
    cancelled.abort(stop.clone());
    assert!(value.cached_rational_interval(-320).is_none());
    check_cancelled(&cancelled, -320);
    assert!(value.cached_rational_interval(-320).is_none());

    let start = Barrier::new(4);
    std::thread::scope(|scope| {
        for precision in [-16, -127, -320] {
            let (start, value) = (&start, &value);
            scope.spawn(move || {
                start.wait();
                check_bounds(value, precision);
            });
        }
        start.wait();
        check_cancelled(&cancelled, -127);
    });

    assert!(value.cached_rational_interval(-320).is_some());
    assert!(cancelled.cached_rational_interval(-320).is_none());
    check_cancelled(&cancelled, -16);
    stop.store(false, Ordering::Relaxed);
    for precision in [-16, -320, -384, -16] {
        check_bounds(&cancelled, precision);
    }
    drop(cancelled);
    check_bounds(&value, -384);
}
