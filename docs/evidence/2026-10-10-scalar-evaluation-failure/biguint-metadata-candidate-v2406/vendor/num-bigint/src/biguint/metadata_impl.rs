// These methods are included in the actual biguint module and the proof entry point.
#[cfg(verus_keep_ghost)]
use vstd::{arithmetic::power2::pow2, prelude::*};

#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(verus_keep_ghost)]
verus! {
impl BigUint {
    pub closed spec fn metadata_value(&self) -> nat {
        metadata::value(self.data@)
    }

    pub closed spec fn metadata_normalized(&self) -> bool {
        metadata::normalized(self.data@)
    }

    pub closed spec fn metadata_storage_bits(&self) -> nat {
        metadata::word_width() as nat * self.data@.len()
    }
}
}

impl BigUint {
    /// Determines the fewest bits necessary to express the [`BigUint`].
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires self.metadata_normalized(), self.metadata_storage_bits() <= u64::MAX,
        ensures self.metadata_value() == 0 <==> result == 0,
            result > 0 ==> crate::magnitude_model::has_bit_length(self.metadata_value(), result as nat),
    ))]
    #[inline]
    pub fn bits(&self) -> u64 {
        proof! {
            reveal(BigUint::metadata_normalized);
            reveal(BigUint::metadata_storage_bits);
            reveal(BigUint::metadata_value);
        }
        metadata::bit_count(self.data.as_slice())
    }

    /// Returns the number of least-significant bits that are zero,
    /// or `None` if the entire number is zero.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires self.metadata_storage_bits() <= u64::MAX,
        ensures match result {
            Some(count) => {
                let unit = pow2(count as nat);
                let quotient = self.metadata_value() / unit;
                quotient > 0 && quotient % 2 == 1 && self.metadata_value() == quotient * unit
                    && count < self.metadata_storage_bits()
            },
            None => self.metadata_value() == 0,
        },
    ))]
    #[inline]
    pub fn trailing_zeros(&self) -> Option<u64> {
        proof! {
            reveal(BigUint::metadata_storage_bits);
            reveal(BigUint::metadata_value);
        }
        metadata::zero_count(self.data.as_slice())
    }

    /// Returns the number of least-significant bits that are ones.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires self.metadata_storage_bits() <= u64::MAX,
        ensures {
            let unit = pow2(result as nat);
            let quotient = (self.metadata_value() + 1) / unit;
            quotient > 0 && quotient % 2 == 1 && self.metadata_value() + 1 == quotient * unit
                && result <= self.metadata_storage_bits()
        },
    ))]
    #[inline]
    pub fn trailing_ones(&self) -> u64 {
        proof! {
            reveal(BigUint::metadata_storage_bits);
            reveal(BigUint::metadata_value);
        }
        metadata::one_count(self.data.as_slice())
    }
}
