//! Fuzz target for vertex arithmetic operations.

#![no_main]

use csgrs::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn clamp_real(value: Real, min: f64, max: f64) -> Real {
    let min = real(min);
    let max = real(max);
    value.max(&min).min(&max).clone()
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e3, 1.0e3))
}

fn assert_vertex_finite(vertex: &Vertex) {
    assert!(vertex.position.x.is_finite());
    assert!(vertex.position.y.is_finite());
    assert!(vertex.position.z.is_finite());
    assert!(vertex.normal.0[0].is_finite());
    assert!(vertex.normal.0[1].is_finite());
    assert!(vertex.normal.0[2].is_finite());
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
        Vector3::from_xyz(
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
        Vector3::from_xyz(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        ),
    );
    let t = clamp_real(decode_real(bytes, &mut idx), -2.0, 2.0);
    assert_vertex_finite(&a.interpolate(&b, t.clone()));
    assert_vertex_finite(&a.slerp_interpolate(&b, t));
    assert!(a.distance_to(&b).is_finite());
    assert!(a.distance_squared_to(&b).is_finite());
    assert!(a.normal_angle_to(&b).is_finite());
});
