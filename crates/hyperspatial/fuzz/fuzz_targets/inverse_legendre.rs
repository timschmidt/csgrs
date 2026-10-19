#![no_main]
use libfuzzer_sys::fuzz_target;
#[path = "../../dev/inverse_legendre.rs"]
mod inverse_legendre;
#[path = "../../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;
fuzz_target!(|data: &[u8]| {
    if let Some(bytes) = data.first_chunk::<6>() {
        inverse_legendre::exercise(*bytes).unwrap();
    }
});
