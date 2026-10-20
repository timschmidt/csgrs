#![no_main]

use libfuzzer_sys::fuzz_target;

#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[path = "../../dev/normal_offset.rs"]
mod normal_offset;

fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        normal_offset::exercise(*bytes).unwrap();
    }
});
