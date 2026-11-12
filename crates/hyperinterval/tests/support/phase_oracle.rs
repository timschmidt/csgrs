//! Directed MPFR pi bounds and independent exact GMP phase selection.
use rug::{
    Float, Integer, Rational,
    float::{Constant, Round},
};
pub fn half_pi() -> (Rational, Rational) {
    crate::mpfr_cache::cleanup_on_thread_exit();
    let (a, _) = Float::with_val_round(4096, Constant::Pi, Round::Down);
    let (b, _) = Float::with_val_round(4096, Constant::Pi, Round::Up);
    (a.to_rational().unwrap() / 2, b.to_rational().unwrap() / 2)
}
pub fn index(x: &Rational, pi: &(Rational, Rational)) -> Integer {
    let a = (x.clone() / &pi.0).floor().numer().clone();
    let b = (x.clone() / &pi.1).floor().numer().clone();
    assert_eq!(a, b, "independent phase oracle needs more precision");
    a
}
