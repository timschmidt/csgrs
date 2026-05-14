#![no_main]

use csgrs::io::gerber::FromGerber;
use csgrs::sketch::Sketch;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if let Ok(sketch) = Sketch::<()>::from_gerber(bytes, None) {
        for triangle in sketch.triangulate() {
            for point in triangle {
                assert!(point.x.is_finite());
                assert!(point.y.is_finite());
                assert!(point.z.is_finite());
            }
        }
    }
});
