#![no_main]
use libfuzzer_sys::fuzz_target;
#[path = "../../dev/complex_atanh.rs"]
mod complex_atanh;
#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;
fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        complex_atanh::exercise(*bytes).unwrap();
    }
});
