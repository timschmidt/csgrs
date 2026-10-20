#![no_main]
use libfuzzer_sys::fuzz_target;
#[path = "../../dev/parameter_encodings.rs"]
mod parameter_encodings;
#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;
fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        parameter_encodings::exercise(*bytes).unwrap();
    }
});
