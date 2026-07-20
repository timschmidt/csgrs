//! Fuzz target for paired mesh boolean operations.

#![no_main]

use csgrs::csg::CSG;
use csgrs::mesh::Mesh;
use hyperlattice::Real;
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    value.max(&tolerance).clone()
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-100.0, 100.0))
}

fn assert_mesh_finite(mesh: &Mesh<()>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.0[0].is_finite());
        assert!(vertex.normal.0[1].is_finite());
        assert!(vertex.normal.0[2].is_finite());
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let size_a = at_least_tolerance(decode_real(bytes, &mut idx).abs());
    let size_b = at_least_tolerance(decode_real(bytes, &mut idx).abs());
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
