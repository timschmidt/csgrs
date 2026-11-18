#![no_main]

#[path = "../../tests/support/schedule_equivalence.rs"]
mod oracle;

libfuzzer_sys::fuzz_target!(|data: &[u8]| oracle::check(data));
