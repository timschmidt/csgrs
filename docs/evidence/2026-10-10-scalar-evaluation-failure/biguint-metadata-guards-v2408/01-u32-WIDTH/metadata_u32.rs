// Metadata operations shared by the patched dependency and its proof entry point.
use alloc::vec::Vec;

#[cfg(verus_keep_ghost)]
use crate::digit_scan_model::has_odd_factor;
#[cfg(verus_keep_ghost)]
use crate::magnitude_model::has_bit_length;
#[cfg(verus_keep_ghost)]
use crate::word_scan_model::*;
#[cfg(verus_keep_ghost)]
pub(crate) use crate::word_scan_model::{
    normalized_u32 as normalized, word_value_u32 as value,
};
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn word_width() -> u64 { 32 }
}

#[cfg_attr(verus_keep_ghost, verus_spec(ensures WIDTH == word_width(),))]
pub(crate) const WIDTH: u64 = 33;

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != skipped
            && forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == skipped,
    },
))]
#[inline]
pub(crate) fn first_different(words: &[u32], skipped: u32) -> Option<usize> {
    let mut index = 0;
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant index <= words.len(),
            forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        decreases words.len() - index,
    ))]
    while index < words.len() {
        if words[index] != skipped {
            return Some(index);
        }
        index += 1;
    }
    None
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != 0
            && forall|higher: int| index < higher < words.len() ==> #[trigger] words@[higher] == 0,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
))]
#[inline]
pub(crate) fn last_nonzero(words: &[u32]) -> Option<usize> {
    let mut length = words.len();
    #[cfg_attr(verus_keep_ghost, verus_spec(
        invariant length <= words.len(),
            forall|higher: int| length <= higher < words.len() ==> #[trigger] words@[higher] == 0,
        decreases length,
    ))]
    while length > 0 {
        if words[length - 1] != 0 {
            return Some(length - 1);
        }
        length -= 1;
    }
    None
}

#[cfg_attr(verus_keep_ghost, verus_spec(
    ensures word_value_u32(final(words)@) == word_value_u32(old(words)@),
        normalized_u32(final(words)@),
))]
#[inline]
pub(crate) fn trim_zero_suffix(words: &mut Vec<u32>) {
    let length = match last_nonzero(words.as_slice()) {
        Some(index) => index + 1,
        None => 0,
    };
    proof! { typed_zero_suffix_truncation_u32(words@, length as int); }
    words.truncate(length);
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires word_width() * words.len() <= u64::MAX,
    ensures match result {
        Some(count) => has_odd_factor(word_value_u32(words@), count as nat)
            && count < word_width() * words.len(),
        None => word_value_u32(words@) == 0,
    },
))]
#[inline]
pub(crate) fn zero_count(words: &[u32]) -> Option<u64> {
    match first_different(words, 0) {
        Some(index) => {
            let count = words[index].trailing_zeros();
            proof! {
                typed_zero_scan_u32(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, 32, index as nat, count as nat);
            }
            Some(index as u64 * WIDTH + u64::from(count))
        },
        None => {
            proof! { typed_zero_scan_none_u32(words@); }
            None
        },
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires word_width() * words.len() <= u64::MAX,
    ensures has_odd_factor(word_value_u32(words@) + 1, result as nat),
        result <= word_width() * words.len(),
))]
#[inline]
pub(crate) fn one_count(words: &[u32]) -> u64 {
    match first_different(words, u32::MAX) {
        Some(index) => {
            let count = words[index].trailing_ones();
            proof! {
                typed_one_scan_u32(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, 32, index as nat, count as nat);
            }
            index as u64 * WIDTH + u64::from(count)
        },
        None => {
            proof! { typed_one_scan_none_u32(words@); }
            words.len() as u64 * WIDTH
        },
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    requires normalized_u32(words@), word_width() * words.len() <= u64::MAX,
    ensures result == word_bit_length_u32(words@),
        word_value_u32(words@) == 0 <==> result == 0,
        result > 0 ==> has_bit_length(word_value_u32(words@), result as nat),
))]
#[inline]
pub(crate) fn bit_count(words: &[u32]) -> u64 {
    proof! { typed_normalized_bit_length_u32(words@); }
    if words.len() == 0 {
        0
    } else {
        let count = words[words.len() - 1].leading_zeros();
        words.len() as u64 * WIDTH - u64::from(count)
    }
}
