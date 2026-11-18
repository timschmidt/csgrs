#![no_main]

#[path = "../../tests/support/reachability_policy.rs"]
mod oracle;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|input: [u8; 8]| oracle::check(input));
