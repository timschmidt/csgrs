//! Fuzz target for the transitional `csgrs::Mesh` to `hypermesh` adapter.

#![no_main]

use std::collections::BTreeMap;

use csgrs::mesh::Mesh;
use hyperlattice::{Point3, Real, Vector3};
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

fn decode_mesh(bytes: &[u8], idx: &mut usize) -> Mesh<()> {
    let tag = bytes[*idx % bytes.len()] % 8;
    *idx += 1;
    let a = decode_real(bytes, idx);
    let b = decode_real(bytes, idx);
    let c = decode_real(bytes, idx);
    let segments = (bytes[*idx % bytes.len()] as usize % 16) + 3;
    *idx += 1;

    match tag {
        0 => Mesh::cube(at_least_tolerance(a.abs()), ()),
        1 => Mesh::cuboid(
            at_least_tolerance(a.abs()),
            at_least_tolerance(b.abs()),
            at_least_tolerance(c.abs()),
            (),
        ),
        2 => Mesh::sphere(at_least_tolerance(a.abs()), segments, segments, ()),
        3 => Mesh::cylinder(at_least_tolerance(a.abs()), b, segments, ()),
        4 => Mesh::frustum(a.abs(), b.abs(), c, segments, ()),
        5 => Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(a.clone(), b.clone(), c.clone()),
            a.abs(),
            b.abs(),
            segments,
            (),
        ),
        6 => Mesh::ellipsoid(
            at_least_tolerance(a.abs()),
            at_least_tolerance(b.abs()),
            at_least_tolerance(c.abs()),
            segments,
            segments,
            (),
        ),
        _ => Mesh::arrow(
            Point3::origin(),
            Vector3::from_xyz(a, b, c),
            segments,
            false,
            (),
        ),
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
    let buffers = mesh.to_hypermesh_buffers();

    assert!(buffers.positions.len().is_multiple_of(3));
    assert!(buffers.indices.len().is_multiple_of(3));
    let vertex_count = buffers.positions.len() / 3;
    for coordinate in &buffers.positions {
        assert!(coordinate.is_finite());
    }
    for index in &buffers.indices {
        assert!(*index < vertex_count);
    }
    if mesh.to_hypermesh_exact().is_ok() {
        let (vertices, indices) = mesh
            .try_get_vertices_and_indices()
            .expect("exact mesh should lower");
        assert!(!vertices.is_empty());
        assert!(!indices.is_empty());
        let (vertex_map, adjacency) = mesh.build_connectivity();
        assert_eq!(vertex_map.vertex_count(), vertices.len());
        let directed_edge_count = adjacency.values().map(Vec::len).sum::<usize>();
        assert_eq!(directed_edge_count % 2, 0);
    } else {
        let (vertex_map, adjacency) = mesh.build_connectivity();
        assert_eq!(vertex_map.vertex_count(), 0);
        assert!(adjacency.is_empty());
        assert!(mesh.try_get_vertices_and_indices().is_err());
        assert_eq!(mesh.get_vertices_and_indices(), (Vec::new(), Vec::new()));
    }

    if let Ok(exact) = mesh.to_hypermesh_exact() {
        assert!(!exact.positions.is_empty());
        assert!(!exact.triangles.is_empty());
        assert_eq!(
            mesh.is_manifold(),
            indexed_triangles_are_closed_manifold(
                exact.positions.len(),
                exact.triangles.iter().map(|triangle| triangle.indices()),
            )
        );
    } else {
        assert!(!mesh.is_manifold());
    }
});
