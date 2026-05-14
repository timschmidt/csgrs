//! Fuzz target for transform matrix inputs.

#![no_main]

use csgrs::csg::CSG;
use csgrs::float_types::Real;
use csgrs::mesh::Mesh;
use libfuzzer_sys::fuzz_target;
use nalgebra::Matrix4;

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-1.0e6, 1.0e6) as Real
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0;
    let mut values = [0.0 as Real; 16];
    for value in &mut values {
        *value = decode_real(bytes, &mut idx);
    }
    let matrix = Matrix4::from_row_slice(&values);
    let mesh: Mesh<()> = Mesh::cube(1.0, None);
    let transformed = mesh.transform(&matrix);

    for vertex in transformed.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.x.is_finite());
        assert!(vertex.normal.y.is_finite());
        assert!(vertex.normal.z.is_finite());
    }
});
