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
mod storage_probe {
use crate::word_scan_model::*;
use vstd::prelude::*;

verus! {

pub(crate) fn truncate_u32(words: &mut Vec<u32>, length: usize)
    requires length <= old(words).len(),
        length == 0 || old(words)@[length as int - 1] > 0,
        forall|index: int| length <= index < old(words).len() ==> #[trigger] old(words)@[index] == 0,
    ensures word_value_u32(words@) == word_value_u32(old(words)@), normalized_u32(words@),
{
    proof { typed_zero_suffix_truncation_u32(words@, length as int); }
    words.truncate(length);
}


pub(crate) fn truncate_u64(words: &mut Vec<u64>, length: usize)
    requires length <= old(words).len(),
        length == 0 || old(words)@[length as int - 1] > 0,
        forall|index: int| length <= index < old(words).len() ==> #[trigger] old(words)@[index] == 0,
    ensures word_value_u64(words@) == word_value_u64(old(words)@), normalized_u64(words@),
{
    proof { typed_zero_suffix_truncation_u64(words@, length as int); }
    words.truncate(length);
}

}
}
