//! Fuzz target for OBJ import parsing.

#![no_main]

use libfuzzer_sys::fuzz_target;
use std::io::Cursor;

fuzz_target!(|bytes: &[u8]| {
    let text = String::from_utf8_lossy(bytes);
    if let Ok(mesh) = csgrs::io::obj::from_obj(Cursor::new(text.as_bytes())) {
        for position in mesh.positions.iter() {
            assert!(position.x.is_finite());
            assert!(position.y.is_finite());
            assert!(position.z.is_finite());
        }
    }
});
