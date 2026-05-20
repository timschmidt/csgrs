//! Fuzz target for paired mesh boolean operations.

#![no_main]

use csgrs::csg::CSG;
use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
use libfuzzer_sys::fuzz_target;

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-100.0, 100.0) as Real
}

fn assert_mesh_finite(mesh: &Mesh<()>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.x.is_finite());
        assert!(vertex.normal.y.is_finite());
        assert!(vertex.normal.z.is_finite());
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let size_a = decode_real(bytes, &mut idx).abs().max(tolerance());
    let size_b = decode_real(bytes, &mut idx).abs().max(tolerance());
    let b = Mesh::cube(size_b, ()).translate(
        decode_real(bytes, &mut idx),
        decode_real(bytes, &mut idx),
        decode_real(bytes, &mut idx),
    );
    let a = Mesh::cube(size_a, ());
    let result = match bytes[idx % bytes.len()] % 4 {
        0 => a.union(&b),
        1 => a.difference(&b),
        2 => a.intersection(&b),
        _ => a.xor(&b),
    };
    assert_mesh_finite(&result);
});
