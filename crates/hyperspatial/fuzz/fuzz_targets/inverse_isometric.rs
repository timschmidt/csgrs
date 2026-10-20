#![no_main]

use libfuzzer_sys::fuzz_target;

#[path = "../../dev/inverse_isometric.rs"]
mod inverse_isometric;
#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        inverse_isometric::exercise(*bytes).unwrap();
    }
});
