#![no_main]

#[path = "../../tests/support/dyadic_oracle.rs"]
mod dyadic_oracle;
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
#[path = "../../tests/support/atanh_oracle.rs"]
mod oracle;
use dyadic_oracle::q;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, i16, u8, i16, u8, u8)| {
    let (left_anchor, left_offset, right_anchor, right_offset, scale, grid) = data;
    let fraction_bits = u32::from(scale % 61);
    let unit = 1_i64 << fraction_bits;
    let endpoint = |anchor: u8, offset: i16| {
        let center = match anchor % 3 {
            0 => -unit,
            1 => 0,
            _ => unit,
        };
        center + i64::from(offset)
    };
    let a = endpoint(left_anchor, left_offset);
    let b = endpoint(right_anchor, right_offset);
    oracle::verify(a.min(b), a.max(b), fraction_bits, 5 - i32::from(grid % 70));
});
