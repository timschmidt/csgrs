use vstd::prelude::*;

verus! {
fn masked_tag(value: u64) -> (result: u64)
    ensures result == value & 3u64,
{
    assert(value & 3u64 <= 3u64) by (bit_vector);
    match value & 3u64 {
        0 => 0,
        1 => 1,
        2 => 2,
        3 => 3,
        _ => unreachable!(),
    }
}
}

fn main() {}
