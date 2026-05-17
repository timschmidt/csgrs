#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    support::h_assert_adversarial_polygon_pipeline(&mut reader);
});
