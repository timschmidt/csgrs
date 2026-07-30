//! Fuzz target for reusable native Hypermesh geometry emitted by csgrs.

#![no_main]

use std::collections::BTreeMap;

use csgrs::solid;
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::TriangleMesh;
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    hyperlimit::real_max(&value, &tolerance)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
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

fn decode_mesh(bytes: &[u8], idx: &mut usize) -> TriangleMesh {
    let tag = bytes[*idx % bytes.len()] % 8;
    *idx += 1;
    let a = decode_real(bytes, idx);
    let b = decode_real(bytes, idx);
    let c = decode_real(bytes, idx);
    let segments = (bytes[*idx % bytes.len()] as usize % 16) + 3;
    *idx += 1;

    match tag {
        0 => solid::cube(at_least_tolerance(a.abs())),
        1 => solid::cuboid(
            at_least_tolerance(a.abs()),
            at_least_tolerance(b.abs()),
            at_least_tolerance(c.abs()),
        ),
        2 => solid::sphere(at_least_tolerance(a.abs()), segments, segments),
        3 => solid::cylinder(at_least_tolerance(a.abs()), b, segments),
        4 => solid::frustum(a.abs(), b.abs(), c, segments),
        5 => solid::frustum_between(
            Point3::origin(),
            Point3::new(a.clone(), b.clone(), c.clone()),
            a.abs(),
            b.abs(),
            segments,
        ),
        6 => solid::ellipsoid(
            at_least_tolerance(a.abs()),
            at_least_tolerance(b.abs()),
            at_least_tolerance(c.abs()),
            segments,
            segments,
        ),
        _ => solid::arrow(Point3::origin(), Vector3::from_xyz(a, b, c), segments, false),
    }
}

fn indexed_triangles_are_closed_manifold(
    vertex_count: usize,
    triangles: impl IntoIterator<Item = [usize; 3]>,
) -> bool {
    let mut edge_counts = BTreeMap::<(usize, usize), usize>::new();
    for [a, b, c] in triangles {
        for [u, v] in [[a, b], [b, c], [c, a]] {
            if u >= vertex_count || v >= vertex_count || u == v {
                return false;
            }
            let key = if u < v { (u, v) } else { (v, u) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    !edge_counts.is_empty() && edge_counts.values().all(|count| *count == 2)
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let mesh = decode_mesh(bytes, &mut idx);
    let buffers = mesh
        .exact_gpu_mesh_buffers()
        .expect("csgrs primitives have finite renderer projections");

    assert_eq!(buffers.vertices.len(), mesh.positions.len());
    assert!(buffers.indices.len().is_multiple_of(3));
    let vertex_count = buffers.vertices.len();
    for (position, normal) in &buffers.vertices {
        assert!(position.iter().all(Real::is_finite));
        assert!(normal.iter().all(Real::is_finite));
    }
    for index in &buffers.indices {
        assert!((*index as usize) < vertex_count);
    }
    hypermesh::polygon_soup(&[mesh.as_ref()])
        .expect("csgrs primitives remain valid reusable Hypermesh input");
    assert_eq!(
        mesh.is_closed_manifold(),
        indexed_triangles_are_closed_manifold(
            mesh.positions.len(),
            mesh.triangles.iter().map(|triangle| triangle.indices()),
        )
    );
});
