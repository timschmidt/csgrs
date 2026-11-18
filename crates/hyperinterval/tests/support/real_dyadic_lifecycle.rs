use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperreal::{Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

pub fn budget() -> DyadicBudget {
    DyadicBudget::new(2048, 1_000_000_000)
}

pub fn translated_root() -> Real {
    Real::from(109).sqrt().unwrap() - Real::from(Rational::fraction(7, 5).unwrap())
}

pub fn check_bounds(value: &Real, precision: i32) {
    let range = DyadicInterval::from_real(value, precision, &mut budget())
        .unwrap()
        .to_rational(&mut budget())
        .unwrap();
    let (lower, upper) = range.bounds().unwrap();
    let shift = Rational::fraction(7, 5).unwrap();
    let lower = lower + &shift;
    let upper = upper + shift;
    assert!(lower > Rational::from(10));
    assert!(&lower * &lower < Rational::from(109));
    assert!(&upper * &upper > Rational::from(109));
    assert!(upper < Rational::from(11));
}

pub fn check_cancelled(value: &Real, precision: i32) {
    assert_eq!(
        DyadicInterval::from_real(value, precision, &mut budget()),
        Err(IntervalError::EnclosureUnavailable)
    );
}

pub fn check_retry() {
    let value = translated_root();
    let mut reader = value.clone();
    let stop = Arc::new(AtomicBool::new(true));
    reader.abort(stop.clone());
    // Cancellation on a cold clone must leave the shared result cache cold.
    assert!(value.cached_rational_interval(-320).is_none());
    check_cancelled(&reader, -320);
    assert!(value.cached_rational_interval(-320).is_none());

    check_bounds(&value, -320);
    assert!(value.cached_rational_interval(-320).is_some());
    assert!(reader.cached_rational_interval(-320).is_none());
    check_cancelled(&reader, -16);
    stop.store(false, Ordering::Relaxed);
    assert_eq!(
        DyadicInterval::from_real(&reader, -320, &mut DyadicBudget::new(2048, 1_000_000)),
        Err(IntervalError::WorkExhausted)
    );
    for precision in [-16, -320, -384, -16] {
        check_bounds(&reader, precision);
    }
    drop(reader);
    check_bounds(&value, -384);
}
