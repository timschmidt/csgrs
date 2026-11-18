#![no_main]
#[path = "../../../hypersolve/tests/support/krawczyk_oracle.rs"]
mod oracle;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|input: [u8; 8]| oracle::check(input));
