#![no_main]

use libfuzzer_sys::fuzz_target;

#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[path = "../../dev/ellipsoidal_mercator.rs"]
mod ellipsoidal_mercator;

fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        ellipsoidal_mercator::exercise(*bytes).unwrap();
    }
});
