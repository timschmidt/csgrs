//! Development-only MPFR point references; native code uses no foreign backend.
use hyperreal::Real;
use rug::Float;

pub fn values(value: &Real, parameter: &Real) -> [Float; 3] {
    let phi = crate::inverse_oracle::inverse(value, parameter, false);
    let sn = phi.clone().sin();
    let cn = phi.cos();
    let dn = (Float::with_val(1024, 1)
        - crate::legendre_oracle::value(parameter) * sn.clone().square())
    .sqrt();
    [sn, cn, dn]
}
