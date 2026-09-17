//! Exact native fraction reduction shared with production arithmetic.

#[cfg(verus_keep_ghost)]
use super::division::gcd;
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

/// Divide both parts by their complete GCD, including `0 / denominator`.
#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires denominator > 0,
    ensures
        result.0 == numerator as nat / gcd(numerator as nat, denominator as nat),
        result.1 == denominator as nat / gcd(numerator as nat, denominator as nat),
        result.0 <= numerator,
        0 < result.1 <= denominator,
        gcd(result.0 as nat, result.1 as nat) == 1,
        result.0 as nat * denominator as nat == numerator as nat * result.1 as nat,
        (result.0 == 0 <==> numerator == 0),
        numerator == 0 ==> result.1 == 1,
))]
#[inline]
pub(crate) fn reduce(numerator: u128, denominator: u128) -> (u128, u128) {
    let divisor = super::gcd::gcd_u128(numerator, denominator);
    proof! { super::gcd::gcd_reduction(numerator as nat, denominator as nat); }
    (numerator / divisor, denominator / divisor)
}
