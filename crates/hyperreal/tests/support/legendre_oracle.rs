//! Independent development-only Legendre series and AGM references, not production algorithms.
use hyperreal::{Rational, Real};
use rug::{Float, float::Constant};

/// Round an exact Rational to 1024-bit MPFR by transferring numerator, denominator and sign.
/// Avoid mixed-fraction Display parsing. This creates an oracle input, not a
/// directed enclosure; callers register thread-local backend cleanup.
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

/// Convert a Real oracle input through an exact rational or a -896 enclosure midpoint.
/// The exact-rational shortcut avoids refinement; otherwise require an available
/// enclosure and round its midpoint via rational. Neither path is a directed MPFR
/// interval, and the caller owns cleanup and valid fixture assumptions.
pub fn value(value: &Real) -> Float {
    if let Some(exact) = value.exact_rational() {
        return rational(&exact);
    }
    let [lo, hi] = value.certified_rational_interval(-896).unwrap();
    rational(&((lo + hi) / Rational::new(2)))
}

/// Adopt a finite MPFR reference's exact binary rational as a Real test expectation.
/// This preserves the rounded reference value, not the exact integral being
/// approximated. Nonfinite values or unparsable rational text fail the fixture.
pub fn native(value: Float) -> Real {
    value.to_rational().unwrap().to_string().parse().unwrap()
}

/// Compute independent F/E references using binomial coefficients and sine-power moments.
/// Assert the rounded inputs satisfy abs(phi)<8 and abs(m)<=3/4. Integration by
/// parts advances the moments of sin(t)^(2n) from zero to phi; binomial coefficients
/// weight F, with the corresponding negative 1/(2n-1) factor for E. Retain the
/// zeroth term plus 2048 nonconstant terms, independently of Carlson duplication.
///
/// The admitted mathematical range gives an omitted analytic tail below 2^-840.
/// All input conversions and arithmetic use nearest-rounded 1024-bit MPFR; the
/// tail bound alone does not bound their rounding error. These checks concern
/// rounded oracle inputs, not production parameter admission. Callers choose
/// appropriate comparison margins and register MPFR cleanup.
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

/// Compute complete K/E references from 32 AGM steps and the associated weighted square sum.
/// Require a positive finite complement c=1-m. Start a=1, b=sqrt(c), with square
/// sum (1-c)/2 and doubling weights; return pi/(2a) and K*(1-sum) after the fixed
/// schedule. This is a 1024-bit MPFR oracle, not a convergence-tested or directed
/// enclosure algorithm for arbitrary complements. The caller supplies the domain,
/// comparison tolerance and backend cleanup; no production value uses this schedule.
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
