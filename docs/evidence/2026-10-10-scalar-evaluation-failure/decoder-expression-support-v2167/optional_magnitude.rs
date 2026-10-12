use vstd::prelude::*;

verus! {
fn original_optional_magnitude(value: u64) -> (result: Option<i32>)
    ensures result == if value & 16u64 != 0 {
        Some(((value >> 32u32) as u32) as i32)
    } else { None },
{
    (value & 16u64 != 0).then_some(((value >> 32u32) as u32) as i32)
}
}

fn main() {}
