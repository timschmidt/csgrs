//! Independent development-only Jacobi point references over the residual-checked MPFR inverse.
use hyperreal::Real;
use rug::Float;

/// Return MPFR sn/cn/dn references after independently solving the first-kind inverse.
/// Inherit the inverse oracle's rounded abs(u)<4 and abs(m)<=3/4 domain guards,
/// 24-step safeguarded iteration and final 2^-800 residual check. Evaluate sine
/// and cosine of its 1024-bit amplitude, then sqrt(1-m*sn^2) on the positive
/// branch; this mathematical radicand is at least 1/4 in the admitted range.
/// These rounded point operations provide no directed error enclosure or native
/// certainty receipt. Callers choose comparison margins and register backend cleanup.
pub fn values(value: &Real, parameter: &Real) -> [Float; 3] {
    let phi = crate::inverse_oracle::inverse(value, parameter, false);
    let sn = phi.clone().sin();
    let cn = phi.cos();
    let dn = (Float::with_val(1024, 1)
        - crate::legendre_oracle::value(parameter) * sn.clone().square())
    .sqrt();
    [sn, cn, dn]
}
