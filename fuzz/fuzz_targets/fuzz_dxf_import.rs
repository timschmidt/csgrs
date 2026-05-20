//! Fuzz target for DXF import parsing.

#![no_main]

use csgrs::mesh::Mesh;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if let Ok(mesh) = Mesh::<()>::from_dxf(bytes, ()) {
        for vertex in mesh.vertices() {
            assert!(vertex.position.x.is_finite());
            assert!(vertex.position.y.is_finite());
            assert!(vertex.position.z.is_finite());
        }
    }
});
