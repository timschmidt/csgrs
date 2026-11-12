//! Independent exact grid tightness and directed MPFR square-root oracle.
use hyperinterval::DyadicInterval;
use rug::{Float, Rational as Q, float::Round};

#[path = "dyadic_oracle.rs"]
mod dyadic_oracle;
pub use dyadic_oracle::q;

// Certify containment AND the impossibility of moving either grid endpoint
// inward. These exact GMP inequalities do not reuse the implementation's
// division/root algorithm. Directed MPFR separately checks point bounds.
pub fn verify(result: &DyadicInterval, a: Q, b: Q, precision: i32) {
    let (lo, hi, exponent) = result.bounds().unwrap();
    let lo = q(lo, exponent);
    let hi = q(hi, exponent);
    let step = q(&1, precision);
    assert!(lo >= 0 && hi >= 0);
    assert!(lo.clone().square() <= a);
    assert!(hi.clone().square() >= b);
    assert!((lo.clone() + &step).square() > a);
    if hi > 0 {
        assert!((hi.clone() - &step).square() < b);
    }
    assert!((&lo / &step).complete().is_integer());
    assert!((&hi / &step).complete().is_integer());
    // 4096 bits is greater than every exact input and chosen grid in this
    // corpus; perfect grid squares are represented exactly by MPFR.
    crate::mpfr_cache::cleanup_on_thread_exit();
    let (mut down, _) = Float::with_val_round(4096, &a, Round::Down);
    let (mut up, _) = Float::with_val_round(4096, &b, Round::Up);
    down.sqrt_round(Round::Down);
    up.sqrt_round(Round::Up);
    assert!(lo <= down.to_rational().unwrap());
    assert!(hi >= up.to_rational().unwrap());
}
use rug::Complete;
