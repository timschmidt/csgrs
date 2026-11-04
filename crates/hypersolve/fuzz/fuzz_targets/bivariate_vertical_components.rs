#![no_main]
#[path = "support/vertical_components.rs"]
mod acceptance;
libfuzzer_sys::fuzz_target!(|data: &[u8]| acceptance::check(data));
