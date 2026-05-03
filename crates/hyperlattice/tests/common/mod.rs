use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use realistic_blas::{Scalar, sin};

pub fn r(value: i32) -> Scalar {
    value.into()
}

#[allow(dead_code)]
#[cfg(feature = "hyperreal-backend")]
pub fn frac(numerator: i64, denominator: u64) -> Scalar {
    realistic_blas::Rational::fraction(numerator, denominator)
        .unwrap()
        .into()
}

#[allow(dead_code)]
#[cfg(not(feature = "hyperreal-backend"))]
pub fn frac(numerator: i64, denominator: u64) -> Scalar {
    Scalar::try_from(numerator as f64 / denominator as f64).unwrap()
}

#[allow(dead_code)]
pub fn abort_signal() -> realistic_blas::AbortSignal {
    Arc::new(AtomicBool::new(false))
}

#[allow(dead_code)]
pub fn unknown_zero() -> Scalar {
    let one = r(1);
    sin(one.clone()) - sin(one)
}
