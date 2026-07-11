//! Fuzz target for Gerber import parsing.

#![no_main]

use csgrs::io::gerber::FromGerber;
use csgrs::sketch::Profile;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if let Ok(sketch) = Profile::from_gerber(bytes ) {
        for triangle in sketch.triangulate() {
            for point in triangle {
                assert!(point.x.is_finite());
                assert!(point.y.is_finite());
                assert!(point.z.is_finite());
            }
        }
    }
});
