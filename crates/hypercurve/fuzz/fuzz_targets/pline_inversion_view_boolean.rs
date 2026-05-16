#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

// Shape holes are implemented by presenting CW loops through PlineInversionView before calling
// the lower-level polyline boolean. This harness keeps that adapter under direct fuzz pressure.
fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    support::assert_inversion_boolean(&mut reader);
});
