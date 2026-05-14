//! Fuzz target for SVG import parsing.

#![no_main]

use csgrs::io::svg::FromSVG;
use csgrs::sketch::Sketch;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    let text = String::from_utf8_lossy(bytes);
    if let Ok(sketch) = Sketch::<()>::from_svg(&text, None) {
        for triangle in sketch.triangulate() {
            for point in triangle {
                assert!(point.x.is_finite());
                assert!(point.y.is_finite());
                assert!(point.z.is_finite());
            }
        }
    }
});
