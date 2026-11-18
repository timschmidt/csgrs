#![no_main]

use libfuzzer_sys::fuzz_target;
#[path = "../../tests/support/integer_affine_oracle.rs"]
mod oracle;

fuzz_target!(|data: [i128; 4]| {
    let [a, b, factor, offset] = data;
    oracle::signed(a, b, factor, offset);
    oracle::signed(a, b, factor % 5, offset);
    // 2*a + (!a) = a-1, even when 2*a cannot fit.
    oracle::signed(a, a, 2, !a);

    let (a, b, f, t) = (a as u128, b as u128, factor as u128, offset as u128);
    oracle::unsigned(a, b, f, t);
    oracle::unsigned(a >> 3, b >> 3, f % 8, t >> 3);
});
