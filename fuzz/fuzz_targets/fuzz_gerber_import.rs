//! Fuzz target for Gerber import parsing.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if let Ok((region, _, _)) = csgrs::io::gerber::import_gerber(bytes) {
        let mesh = csgrs::curve::triangulate(&region);
        for point in mesh.positions.iter() {
            assert!(point.x.is_finite());
            assert!(point.y.is_finite());
            assert!(point.z.is_finite());
        }
    }
});
