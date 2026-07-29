//! Fuzz target for DXF import parsing.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if let Ok(mesh) = csgrs::io::dxf::from_dxf(bytes) {
        for position in mesh.positions.iter() {
            assert!(position.x.is_finite());
            assert!(position.y.is_finite());
            assert!(position.z.is_finite());
        }
    }
});
