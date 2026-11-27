//! Development-only residual-checked MPFR inverse, independent of native Carlson root refinement.
use hyperreal::Real;
use rug::Float;

use crate::legendre_oracle as integrals;

/// Compute an independent inverse F/E reference with safeguarded Newton steps and a final residual check.
/// Convert inputs through the shared 1024-bit MPFR adapters and assert rounded
/// abs(target)<4 and abs(m)<=3/4. Both forward derivatives are at least one half
/// on that mathematical parameter range, so the real root is inside (-8,8).
/// Start at the target; update a sign bracket from the forward-series residual,
/// accept a Newton proposal only strictly inside it, and otherwise bisect.
///
/// Try at most 24 steps, stopping early if the rounded proposal cannot move.
/// Regardless of that stopping reason, recompute the forward-series residual and
/// require its magnitude below 2^-800 before returning phi. An exhausted iterate
/// is not silently accepted. This threshold and the series' analytic tail leave
/// room for callers' 512-bit comparisons, but do not provide a directed MPFR
/// rounding proof or native certainty receipt. Domain checks concern rounded
/// oracle inputs; the caller supplies valid fixtures and backend cleanup.
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
