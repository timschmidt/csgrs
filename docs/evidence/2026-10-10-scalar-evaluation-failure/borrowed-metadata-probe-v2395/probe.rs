//! Test pinned dependency call contracts without changing production source.
#![feature(proc_macro_hygiene)]
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/src/verified/mod.rs"]
mod verified;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/magnitude_model.rs"]
mod magnitude_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/bit_length_model.rs"]
mod bit_length_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/digit_scan_model.rs"]
mod digit_scan_model;
#[path = "/home/tim/Documents/GitHub/workspace/csgrs/target/scalar-evaluation-candidate/source/crates/hyperreal/verification/word_scan_model.rs"]
mod word_scan_model;
mod borrowed_metadata {
use crate::word_scan_model::*;
use crate::digit_scan_model::has_odd_factor;
use crate::magnitude_model::has_bit_length;
use vstd::prelude::*;

verus! {

pub(crate) fn first_different_u32(words: &[u32], skipped: u32) -> (result: Option<usize>)
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != skipped
            && forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == skipped,
    },
{
    let mut index = 0;
    while index < words.len()
        invariant index <= words.len(),
            forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        decreases words.len() - index,
    {
        if words[index] != skipped {
            return Some(index);
        }
        index += 1;
    }
    None
}

pub(crate) fn last_nonzero_u32(words: &[u32]) -> (result: Option<usize>)
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != 0
            && forall|higher: int| index < higher < words.len() ==> #[trigger] words@[higher] == 0,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
{
    let mut length = words.len();
    while length > 0
        invariant length <= words.len(),
            forall|higher: int| length <= higher < words.len() ==> #[trigger] words@[higher] == 0,
        decreases length,
    {
        if words[length - 1] != 0 {
            return Some(length - 1);
        }
        length -= 1;
    }
    None
}

pub(crate) fn trim_zero_suffix_u32(words: &mut Vec<u32>)
    ensures word_value_u32(final(words)@) == word_value_u32(old(words)@),
        normalized_u32(final(words)@),
{
    let length = match last_nonzero_u32(words.as_slice()) {
        Some(index) => index + 1,
        None => 0,
    };
    proof { typed_zero_suffix_truncation_u32(words@, length as int); }
    words.truncate(length);
}

pub(crate) fn zero_count_u32(words: &[u32]) -> (result: Option<u64>)
    requires 32 * words.len() <= u64::MAX,
    ensures match result {
        Some(count) => has_odd_factor(word_value_u32(words@), count as nat)
            && count < 32 * words.len(),
        None => word_value_u32(words@) == 0,
    },
{
    match first_different_u32(words, 0) {
        Some(index) => {
            let count = words[index].trailing_zeros();
            proof {
                typed_zero_scan_u32(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, 32, index as nat, count as nat);
            }
            Some(index as u64 * 32 + u64::from(count))
        },
        None => {
            proof { typed_zero_scan_none_u32(words@); }
            None
        },
    }
}

pub(crate) fn one_count_u32(words: &[u32]) -> (result: u64)
    requires 32 * words.len() <= u64::MAX,
    ensures has_odd_factor(word_value_u32(words@) + 1, result as nat),
        result <= 32 * words.len(),
{
    match first_different_u32(words, u32::MAX) {
        Some(index) => {
            let count = words[index].trailing_ones();
            proof {
                typed_one_scan_u32(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, 32, index as nat, count as nat);
            }
            index as u64 * 32 + u64::from(count)
        },
        None => {
            proof { typed_one_scan_none_u32(words@); }
            words.len() as u64 * 32
        },
    }
}

pub(crate) fn bit_count_u32(words: &[u32]) -> (result: u64)
    requires normalized_u32(words@), 32 * words.len() <= u64::MAX,
    ensures result == word_bit_length_u32(words@),
        word_value_u32(words@) == 0 <==> result == 0,
        result > 0 ==> has_bit_length(word_value_u32(words@), result as nat),
{
    proof { typed_normalized_bit_length_u32(words@); }
    if words.len() == 0 {
        0
    } else {
        let count = words[words.len() - 1].leading_zeros();
        words.len() as u64 * 32 - u64::from(count)
    }
}


pub(crate) fn first_different_u64(words: &[u64], skipped: u64) -> (result: Option<usize>)
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != skipped
            && forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == skipped,
    },
{
    let mut index = 0;
    while index < words.len()
        invariant index <= words.len(),
            forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        decreases words.len() - index,
    {
        if words[index] != skipped {
            return Some(index);
        }
        index += 1;
    }
    None
}

pub(crate) fn last_nonzero_u64(words: &[u64]) -> (result: Option<usize>)
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != 0
            && forall|higher: int| index < higher < words.len() ==> #[trigger] words@[higher] == 0,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
{
    let mut length = words.len();
    while length > 0
        invariant length <= words.len(),
            forall|higher: int| length <= higher < words.len() ==> #[trigger] words@[higher] == 0,
        decreases length,
    {
        if words[length - 1] != 0 {
            return Some(length - 1);
        }
        length -= 1;
    }
    None
}

pub(crate) fn trim_zero_suffix_u64(words: &mut Vec<u64>)
    ensures word_value_u64(final(words)@) == word_value_u64(old(words)@),
        normalized_u64(final(words)@),
{
    let length = match last_nonzero_u64(words.as_slice()) {
        Some(index) => index + 1,
        None => 0,
    };
    proof { typed_zero_suffix_truncation_u64(words@, length as int); }
    words.truncate(length);
}

pub(crate) fn zero_count_u64(words: &[u64]) -> (result: Option<u64>)
    requires 64 * words.len() <= u64::MAX,
    ensures match result {
        Some(count) => has_odd_factor(word_value_u64(words@), count as nat)
            && count < 64 * words.len(),
        None => word_value_u64(words@) == 0,
    },
{
    match first_different_u64(words, 0) {
        Some(index) => {
            let count = words[index].trailing_zeros();
            proof {
                typed_zero_scan_u64(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, 64, index as nat, count as nat);
            }
            Some(index as u64 * 64 + u64::from(count))
        },
        None => {
            proof { typed_zero_scan_none_u64(words@); }
            None
        },
    }
}

pub(crate) fn one_count_u64(words: &[u64]) -> (result: u64)
    requires 64 * words.len() <= u64::MAX,
    ensures has_odd_factor(word_value_u64(words@) + 1, result as nat),
        result <= 64 * words.len(),
{
    match first_different_u64(words, u64::MAX) {
        Some(index) => {
            let count = words[index].trailing_ones();
            proof {
                typed_one_scan_u64(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, 64, index as nat, count as nat);
            }
            index as u64 * 64 + u64::from(count)
        },
        None => {
            proof { typed_one_scan_none_u64(words@); }
            words.len() as u64 * 64
        },
    }
}

pub(crate) fn bit_count_u64(words: &[u64]) -> (result: u64)
    requires normalized_u64(words@), 64 * words.len() <= u64::MAX,
    ensures result == word_bit_length_u64(words@),
        word_value_u64(words@) == 0 <==> result == 0,
        result > 0 ==> has_bit_length(word_value_u64(words@), result as nat),
{
    proof { typed_normalized_bit_length_u64(words@); }
    if words.len() == 0 {
        0
    } else {
        let count = words[words.len() - 1].leading_zeros();
        words.len() as u64 * 64 - u64::from(count)
    }
}

}
}
