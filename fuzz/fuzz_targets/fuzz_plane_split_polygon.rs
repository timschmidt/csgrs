//! Fuzz target for plane and polygon splitting.

#![no_main]

use csgrs::mesh::plane::Plane;
use csgrs::polygon_mesh::Polygon;
use csgrs::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e4, 1.0e4))
}

fn point(bytes: &[u8], idx: &mut usize) -> Point3 {
    Point3::new(
        decode_real(bytes, idx),
        decode_real(bytes, idx),
        decode_real(bytes, idx),
    )
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 32 {
        return;
    }

    let mut idx = 0usize;
    let mut points = Vec::new();
    let count = (bytes[0] as usize % 12) + 3;
    idx += 1;
    for _ in 0..count {
        points.push(point(bytes, &mut idx));
    }

    let a = points[0].clone();
    let b = points[1].clone();
    let c = points[2].clone();
    let normal = (b - a.clone()).cross(&(c - a));
    if normal.norm() <= tolerance() || !normal.norm().is_finite() {
        return;
    }
    let vertices = points
        .into_iter()
        .map(|p| Vertex::new(p, normal.clone()))
        .collect::<Vec<_>>();
    let polygon: Polygon<()> = Polygon::new(vertices, ());

    let plane_normal = Vector3::from_xyz(
        decode_real(bytes, &mut idx),
        decode_real(bytes, &mut idx),
        decode_real(bytes, &mut idx),
    );
    if plane_normal.norm() <= tolerance() || !plane_normal.norm().is_finite() {
        return;
    }
    let plane = Plane::from_normal(plane_normal, decode_real(bytes, &mut idx));
    let (cf, cb, front, back) = plane.split_polygon(&polygon);

    for poly in cf
        .iter()
        .chain(cb.iter())
        .chain(front.iter())
        .chain(back.iter())
    {
        for vertex in poly.vertices() {
            assert!(vertex.position.x.is_finite());
            assert!(vertex.position.y.is_finite());
            assert!(vertex.position.z.is_finite());
        }
    }
});
