#![no_main]
#[path = "support/jacobian_matrix.rs"]
mod acceptance;
libfuzzer_sys::fuzz_target!(|data: &[u8]| acceptance::check(data));
