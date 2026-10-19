//! Development-only MPFR inverse, independent of the native Carlson kernel.
use hyperreal::Real;
use rug::Float;

use crate::legendre_oracle as integrals;

// Safeguarded Newton iteration for the integrand series' small-parameter
// domain. Fixed precision and iteration counts belong only to this oracle.
pub fn inverse(value: &Real, parameter: &Real, second: bool) -> Float {
    let target = integrals::value(value);
    let m = integrals::value(parameter);
    assert!(target.clone().abs() < 4 && m.clone().abs() <= 0.75);
    let mut lo = Float::with_val(1024, -8);
    let mut hi = Float::with_val(1024, 8);
    let mut phi = target.clone();
    for _ in 0..24 {
        let evaluated = integrals::series(&integrals::native(phi.clone()), parameter);
        let residual = evaluated[usize::from(second)].clone() - &target;
        if residual < 0 {
            lo = phi.clone();
        } else {
            hi = phi.clone();
        }
        let sine = phi.clone().sin();
        let y = Float::with_val(1024, 1) - Float::with_val(1024, &m * sine.square());
        let derivative = if second { y.sqrt() } else { y.sqrt().recip() };
        let next = &phi - residual / derivative;
        if next == phi {
            break;
        }
        phi = if next > lo && next < hi {
            next
        } else {
            Float::with_val(1024, &lo + &hi) / 2
        };
    }
    // Never publish a merely exhausted Newton iterate as the reference. The
    // defining series has <2^-840 truncation error on this domain; derivatives
    // of both integrals are at least 1/2. The residual margin separates
    // the oracle's fixed-precision arithmetic from callers' 512-bit checks.
    let evaluated = integrals::series(&integrals::native(phi.clone()), parameter);
    let residual = evaluated[usize::from(second)].clone() - target;
    assert!(
        residual.abs() < (Float::with_val(1024, 1) >> 800),
        "MPFR inverse did not converge"
    );
    phi
}
