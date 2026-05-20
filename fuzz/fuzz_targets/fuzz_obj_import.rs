//! Fuzz target for OBJ import parsing.

#![no_main]

use csgrs::mesh::Mesh;
use libfuzzer_sys::fuzz_target;
use std::io::Cursor;

fuzz_target!(|bytes: &[u8]| {
    let text = String::from_utf8_lossy(bytes);
    if let Ok(mesh) = Mesh::<()>::from_obj(Cursor::new(text.as_bytes()), ()) {
        for vertex in mesh.vertices() {
            assert!(vertex.position.x.is_finite());
            assert!(vertex.position.y.is_finite());
            assert!(vertex.position.z.is_finite());
        }
    }
});
