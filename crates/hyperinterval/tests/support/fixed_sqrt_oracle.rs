//! Independent exact GMP grid tightness for fixed square roots.
use hyperinterval::{DomainCoverage, IntervalError};
use rug::Rational;

pub fn fixed(
    a: i64,
    b: i64,
    frac: u32,
    maximum: i64,
    result: Result<((i64, i64), DomainCoverage), IntervalError>,
) {
    if b < 0 {
        assert_eq!(result, Err(IntervalError::EmptyDomain));
        return;
    }
    let scale = 1u64 << frac;
    let lower_source = Rational::from((a.max(0), scale));
    let upper_source = Rational::from((b, scale));
    if Rational::from((maximum, scale)).square() < upper_source {
        assert_eq!(result, Err(IntervalError::Overflow));
        return;
    }
    let ((lower, upper), domain) = result.unwrap();
    assert_eq!(
        domain,
        if a < 0 {
            DomainCoverage::Partial
        } else {
            DomainCoverage::Complete
        }
    );
    let lower = Rational::from((lower, scale));
    let upper = Rational::from((upper, scale));
    let step = Rational::from((1, scale));
    assert!(lower >= 0 && upper >= 0);
    assert!(lower.clone().square() <= lower_source);
    assert!(upper.clone().square() >= upper_source);
    assert!((lower + &step).square() > lower_source);
    if upper > 0 {
        assert!((upper - &step).square() < upper_source);
    }
}
