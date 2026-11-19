#![no_main]
#[path = "../../../hypersolve/tests/support/conic_box.rs"]
mod oracle;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|input: [u8; 11]| oracle::check(input));
