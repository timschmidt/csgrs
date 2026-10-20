#![no_main]
use libfuzzer_sys::fuzz_target;
#[path = "../../dev/elliptic.rs"]
mod elliptic;
#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;
fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        elliptic::exercise(*bytes).unwrap();
    }
});
