//! Numerical callers through the actual Cargo num re-export.
use num::bigint::{BigInt, Sign};
use vstd::arithmetic::power2::pow2;
use vstd::prelude::*;

verus! {

pub open spec fn absolute(value: int) -> nat {
    if value < 0 { (-value) as nat } else { value as nat }
}

pub fn signed_polarity(value: &BigInt) -> (result: i8)
    requires value.metadata_normalized(),
    ensures result == -1 <==> value.metadata_value() < 0,
        result == 0 <==> value.metadata_value() == 0,
        result == 1 <==> value.metadata_value() > 0,
{
    match value.sign() {
        Sign::Minus => 1,
        Sign::NoSign => 0,
        Sign::Plus => 1,
    }
}

pub fn magnitude_bits(value: &BigInt) -> (bits: u64)
    requires value.metadata_normalized(), value.metadata_storage_bits() <= u64::MAX,
    ensures absolute(value.metadata_value()) == 0 <==> bits == 0,
        bits > 0 ==> pow2((bits - 1) as nat) <= absolute(value.metadata_value())
            && absolute(value.metadata_value()) < pow2(bits as nat),
{
    let magnitude = value.magnitude();
    magnitude.bits()
}

}
