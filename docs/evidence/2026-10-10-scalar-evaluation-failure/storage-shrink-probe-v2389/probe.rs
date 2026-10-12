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

pub(crate) fn shrink_u32(words: &mut Vec<u32>)
    ensures words@ == old(words)@,
{
    if words.len() < words.capacity() / 4 {
        words.shrink_to_fit();
    }
}


pub(crate) fn shrink_u64(words: &mut Vec<u64>)
    ensures words@ == old(words)@,
{
    if words.len() < words.capacity() / 4 {
        words.shrink_to_fit();
    }
}

}
}
