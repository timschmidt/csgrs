#![no_main]
use libfuzzer_sys::fuzz_target;
#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;
#[path = "../../dev/transverse_mercator.rs"]
mod transverse;
fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        transverse::exercise(*bytes).unwrap();
    }
});
