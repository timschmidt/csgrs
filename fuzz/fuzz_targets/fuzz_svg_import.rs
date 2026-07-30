//! Fuzz target for SVG import parsing.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    let text = String::from_utf8_lossy(bytes);
    if let Ok((region, _, _)) = csgrs::io::svg::import_svg(&text) {
        let Ok(mesh) =
            csgrs::curve::try_triangulate(&region, &csgrs::GeometryContext::STRICT)
        else {
            return;
        };
        let mesh = mesh.into_value();
        for point in mesh.positions.iter() {
            assert!(point.x.is_finite());
            assert!(point.y.is_finite());
            assert!(point.z.is_finite());
        }
    }
});
