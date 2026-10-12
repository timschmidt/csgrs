use vstd::prelude::*;
use vstd::std_specs::bits::*;

verus! {
proof fn complement_zero_count_u64(value: u64)
    ensures u64_trailing_ones(value) == u64_trailing_zeros(!value),
{
    axiom_u64_trailing_ones(value);
}

proof fn complement_zero_count_u32(value: u32)
    ensures u32_trailing_ones(value) == u32_trailing_zeros(!value),
{
    axiom_u32_trailing_ones(value);
}

fn actual_trailing_ones_u64(value: u64) -> (result: u32)
    ensures result == u64_trailing_zeros(!value),
{
    proof! { complement_zero_count_u64(value); }
    value.trailing_ones()
}

fn actual_trailing_ones_u32(value: u32) -> (result: u32)
    ensures result == u32_trailing_zeros(!value),
{
    proof! { complement_zero_count_u32(value); }
    value.trailing_ones()
}
}
