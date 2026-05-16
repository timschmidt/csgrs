use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use hyperlattice::{Real, sin};

pub fn r(value: i32) -> Real {
    value.into()
}

#[allow(dead_code)]
pub fn frac(numerator: i64, denominator: u64) -> Real {
    hyperlattice::Rational::fraction(numerator, denominator)
        .unwrap()
        .into()
}

#[allow(dead_code)]
pub fn abort_signal() -> hyperlattice::AbortSignal {
    Arc::new(AtomicBool::new(false))
}

#[allow(dead_code)]
pub fn unknown_zero() -> Real {
    let one = r(1);
    sin(one.clone()) - sin(one)
}
