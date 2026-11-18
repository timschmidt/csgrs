#![no_main]

#[path = "../../../hyperlattice-interval/tests/support/shape_oracle.rs"]
mod oracle;

libfuzzer_sys::fuzz_target!(|data: &[u8]| oracle::check(data));
