//! MPFR point bounds plus the global Lipschitz bound and exact GMP extrema.
use hyperinterval::DyadicInterval;
use rug::{
    Float, Integer, Rational,
    float::{Constant, Round},
};
#[path = "grid_oracle.rs"]
mod grid_oracle;
pub use grid_oracle::q;
fn point(x: &Rational, cosine: bool) -> (Rational, Rational) {
    let f = Float::with_val(4096, x);
    let error = (x.clone() - f.to_rational().unwrap()).abs();
    let mut lo = f.clone();
    let mut hi = f;
    if cosine {
        lo.cos_round(Round::Down);
        hi.cos_round(Round::Up);
    } else {
        lo.sin_round(Round::Down);
        hi.sin_round(Round::Up);
    }
    (
        lo.to_rational().unwrap() - &error,
        hi.to_rational().unwrap() + error,
    )
}
pub fn half_pi() -> (Rational, Rational) {
    let (a, _) = Float::with_val_round(4096, Constant::Pi, Round::Down);
    let (b, _) = Float::with_val_round(4096, Constant::Pi, Round::Up);
    (a.to_rational().unwrap() / 2, b.to_rational().unwrap() / 2)
}
fn index(x: &Rational, pi: &(Rational, Rational)) -> Integer {
    let a = (x.clone() / &pi.0).floor().numer().clone();
    let b = (x.clone() / &pi.1).floor().numer().clone();
    assert_eq!(a, b, "independent phase oracle needs more precision");
    a
}
pub fn verify(result: &DyadicInterval, a: &Rational, b: &Rational, precision: i32, cosine: bool) {
    let (a_lo, a_hi) = point(a, cosine);
    let (b_lo, b_hi) = point(b, cosine);
    let mut lo = a_lo.min(b_lo);
    let mut hi = a_hi.max(b_hi);
    let pi = half_pi();
    let left = index(a, &pi);
    let right = index(b, &pi);
    let span = right.clone() - &left;
    for step in 1..=4 {
        if span < step {
            break;
        }
        let residue = ((left.clone() + step) % 4 + 4) % 4;
        if residue == if cosine { 2 } else { 3 } {
            lo = Rational::from(-1);
        }
        if residue == if cosine { 0 } else { 1 } {
            hi = Rational::from(1);
        }
    }
    lo = lo.max(Rational::from(-1));
    hi = hi.min(Rational::from(1));
    let (lo, hi) = grid_oracle::bounds(lo, hi, precision);
    let (ra, rb, e) = result.bounds().unwrap();
    assert_eq!(
        q(ra, e),
        lo,
        "lower: [{a},{b}], p={precision}, cos={cosine}"
    );
    assert_eq!(
        q(rb, e),
        hi,
        "upper: [{a},{b}], p={precision}, cos={cosine}"
    );
}
