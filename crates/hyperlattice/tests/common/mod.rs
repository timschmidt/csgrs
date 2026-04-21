#![allow(dead_code)]

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use realistic_blas::{Real, sin};

pub fn r(value: i32) -> Real {
    value.into()
}

pub fn abort_signal() -> realistic_blas::AbortSignal {
    Arc::new(AtomicBool::new(false))
}

pub fn unknown_zero() -> Real {
    let one = r(1);
    sin(one.clone()) - sin(one)
}
