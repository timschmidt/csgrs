#![no_main]

use csgrs::float_types::Real;
use csgrs::vertex::Vertex;
use libfuzzer_sys::fuzz_target;
use nalgebra::{Point3, Vector3};

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-1.0e3, 1.0e3) as Real
}

fn assert_vertex_finite(vertex: &Vertex) {
    assert!(vertex.position.x.is_finite());
    assert!(vertex.position.y.is_finite());
    assert!(vertex.position.z.is_finite());
    assert!(vertex.normal.x.is_finite());
    assert!(vertex.normal.y.is_finite());
    assert!(vertex.normal.z.is_finite());
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let a = Vertex::new(
        Point3::new(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        ),
        Vector3::new(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        ),
    );
    let b = Vertex::new(
        Point3::new(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        ),
        Vector3::new(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        ),
    );
    let t = decode_real(bytes, &mut idx).clamp(-2.0, 2.0);
    assert_vertex_finite(&a.interpolate(&b, t));
    assert_vertex_finite(&a.slerp_interpolate(&b, t));
    assert!(a.distance_to(&b).is_finite());
    assert!(a.distance_squared_to(&b).is_finite());
    assert!(a.normal_angle_to(&b).is_finite());
});
