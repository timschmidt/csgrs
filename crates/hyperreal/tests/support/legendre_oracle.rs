//! Independent development-only MPFR quadratures, not production algorithms.
use hyperreal::{Rational, Real};
use rug::{Float, float::Constant};

pub fn rational(value: &Rational) -> Float {
    let n: rug::Integer = value.numerator().to_string().parse().unwrap();
    let d: rug::Integer = value.denominator().to_string().parse().unwrap();
    let r = rug::Rational::from((n, d));
    Float::with_val(
        1024,
        if value.sign() == num::bigint::Sign::Minus {
            -r
        } else {
            r
        },
    )
}

pub fn value(value: &Real) -> Float {
    if let Some(exact) = value.exact_rational() {
        return rational(&exact);
    }
    let [lo, hi] = value.certified_rational_interval(-896).unwrap();
    rational(&((lo + hi) / Rational::new(2)))
}

pub fn native(value: Float) -> Real {
    value.to_rational().unwrap().to_string().parse().unwrap()
}

// Binomial expansion of the defining integrand. For |m| <= 3/4 and |phi| < 8,
// 2048 terms leave an analytical tail below 2^-840. MPFR arithmetic uses
// 1024 bits; callers allow a much wider 512-bit (or looser) comparison margin.
// sin-power moments follow integration by parts, independently of Carlson.
pub fn series(amplitude: &Real, parameter: &Real) -> [Float; 2] {
    let phi = value(amplitude);
    let m = value(parameter);
    assert!(phi.clone().abs() < 8 && m.clone().abs() <= 0.75);
    let (sine, cosine) = phi.clone().sin_cos(Float::new(1024));
    let square = Float::with_val(1024, &sine * &sine);
    let mut sine_power = sine;
    let mut moment = phi.clone();
    let mut coefficient = Float::with_val(1024, 1);
    let mut m_power = Float::with_val(1024, 1);
    let mut f = phi.clone();
    let mut e = phi;
    for n in 1..=2048 {
        moment = (moment * (2 * n - 1) - Float::with_val(1024, &sine_power * &cosine)) / (2 * n);
        sine_power *= &square;
        coefficient *= 2 * n - 1;
        coefficient /= 2 * n;
        m_power *= &m;
        let term = Float::with_val(1024, &coefficient * &m_power) * &moment;
        f += &term;
        e -= term / (2 * n - 1);
    }
    [f, e]
}

// Complete integrals from the arithmetic-geometric mean and its square sum.
// Fixed MPFR precision/iterations are oracle choices, never native semantics.
pub fn complete(complement: &Real) -> [Float; 2] {
    let c = value(complement);
    let mut a = Float::with_val(1024, 1);
    let mut b = c.clone().sqrt();
    let mut sum = (Float::with_val(1024, 1) - c) / 2;
    let mut weight = Float::with_val(1024, 1);
    for _ in 0..32 {
        let difference = Float::with_val(1024, &a - &b) / 2;
        sum += Float::with_val(1024, &difference * &difference) * &weight;
        let next = Float::with_val(1024, &a + &b) / 2;
        b = Float::with_val(1024, &a * &b).sqrt();
        a = next;
        weight *= 2;
    }
    let k = Float::with_val(1024, Constant::Pi) / (a * 2);
    let e = Float::with_val(1024, &k * (Float::with_val(1024, 1) - sum));
    [k, e]
}
