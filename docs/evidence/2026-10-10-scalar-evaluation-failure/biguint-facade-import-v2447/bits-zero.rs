//! Conditional callers of the actual exported dependency type and methods.
use num::bigint::BigUint;
use vstd::arithmetic::power2::pow2;
use vstd::prelude::*;

verus! {

pub fn imported_bits(value: &BigUint) -> (bits: u64)
    requires value.metadata_normalized(), value.metadata_storage_bits() <= u64::MAX,
    ensures value.metadata_value() == 0 <==> bits == 0,
        bits > 0 ==> pow2((bits - 1) as nat) <= value.metadata_value()
            && value.metadata_value() < pow2(bits as nat),
{
    0
}

pub fn imported_trailing_zeros(value: &BigUint) -> (result: Option<u64>)
    requires value.metadata_storage_bits() <= u64::MAX,
    ensures match result {
        Some(count) => {
            let unit = pow2(count as nat);
            let quotient = value.metadata_value() / unit;
            quotient > 0 && quotient % 2 == 1 && value.metadata_value() == quotient * unit
                && count < value.metadata_storage_bits()
        },
        None => value.metadata_value() == 0,
    },
{
    value.trailing_zeros()
}

pub fn imported_trailing_ones(value: &BigUint) -> (count: u64)
    requires value.metadata_storage_bits() <= u64::MAX,
    ensures {
        let unit = pow2(count as nat);
        let quotient = (value.metadata_value() + 1) / unit;
        quotient > 0 && quotient % 2 == 1 && value.metadata_value() + 1 == quotient * unit
            && count <= value.metadata_storage_bits()
    },
{
    value.trailing_ones()
}

}
