//! Independent MPFR checks for finite endpoints and the two open domain limits.
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedIntervalView as View, IntervalError,
};
use rug::{Float, Rational, float::Round};

use crate::q;

fn check_endpoint(input: i64, exponent: i32, bound: Rational, direction: Round) {
    // All source mantissas are i64 and transfer exactly at 512 bits.
    let mut limit = Float::with_val(512, q(&input, exponent));
    limit.atanh_round(direction);
    let limit = limit.to_rational().unwrap();
    match direction {
        Round::Down => assert!(bound <= limit, "lower {bound} exceeds {limit}"),
        Round::Up => assert!(bound >= limit, "upper {bound} below {limit}"),
        _ => unreachable!(),
    }
}

/// Checks one closed source on a grid with 0 to 60 binary fractional places.
pub fn verify(a: i64, b: i64, fraction_bits: u32, precision: i32) {
    assert!(a <= b && fraction_bits <= 60);
    crate::mpfr_cache::cleanup_on_thread_exit();
    let unit = 1_i64 << fraction_bits;
    let exponent = -(fraction_bits as i32);
    let source = DyadicInterval::new(a.into(), b.into(), exponent).unwrap();
    let result = source.atanh(precision, &mut DyadicBudget::new(16_384, 1_000_000_000_000));
    if b <= -unit || a >= unit {
        assert_eq!(result, Err(IntervalError::EmptyDomain));
        return;
    }

    let image = result.unwrap();
    let limits = (a <= -unit, b >= unit);
    let coverage = if limits.0 || limits.1 {
        DomainCoverage::Partial
    } else {
        DomainCoverage::Complete
    };
    assert_eq!(image.domain, coverage);
    let (range, check_lo, check_hi) = match (limits, image.range.view()) {
        ((true, true), View::Entire) => return,
        ((true, false), View::Below(point)) => (point, false, true),
        ((false, true), View::Above(point)) => (point, true, false),
        ((false, false), View::Finite(range)) => (range, true, true),
        _ => panic!("incorrect atanh image for [{a},{b}] * 2^{exponent}: {image:?}"),
    };
    let (lower, upper, e) = range.bounds().unwrap();
    assert!(e >= precision || (lower == &0.into() && upper == &0.into()));
    if check_lo {
        check_endpoint(a, exponent, q(lower, e), Round::Down);
    }
    if check_hi {
        check_endpoint(b, exponent, q(upper, e), Round::Up);
    }
}
