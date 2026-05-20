//! Fuzz target for the transitional `csgrs::Mesh` to `hypermesh` adapter.

#![no_main]

use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
use libfuzzer_sys::fuzz_target;
use nalgebra::{Point3, Vector3};

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-100.0, 100.0) as Real
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
        0 => Mesh::cube(a.abs().max(tolerance()), ()),
        1 => Mesh::cuboid(
            a.abs().max(tolerance()),
            b.abs().max(tolerance()),
            c.abs().max(tolerance()),
            (),
        ),
        2 => Mesh::sphere(a.abs().max(tolerance()), segments, segments, ()),
        3 => Mesh::cylinder(a.abs().max(tolerance()), b, segments, ()),
        4 => Mesh::frustum(a.abs(), b.abs(), c, segments, ()),
        5 => Mesh::frustum_ptp(Point3::origin(), Point3::new(a, b, c), a.abs(), b.abs(), segments, ()),
        6 => Mesh::ellipsoid(
            a.abs().max(tolerance()),
            b.abs().max(tolerance()),
            c.abs().max(tolerance()),
            segments,
            segments,
            (),
        ),
        _ => Mesh::arrow(Point3::origin(), Vector3::new(a, b, c), segments, false, ()),
    }
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
    let report = mesh.inspect_hypermesh_input();
    if let Ok(package) = mesh.to_hypermesh_surface_handoff_package() {
        assert!(package.approximate_f64_view.is_some());
        let (vertices, indices) = mesh.try_get_vertices_and_indices().expect("package should lower");
        let view = package.approximate_f64_view.as_ref().unwrap();
        assert_eq!(vertices.len() * 3, view.positions.len());
        assert_eq!(indices.len() * 3, view.indices.len());
        let (vertex_map, adjacency) = mesh.build_connectivity();
        assert_eq!(vertex_map.vertex_count() * 3, view.positions.len());
        let directed_edge_count = adjacency.values().map(Vec::len).sum::<usize>();
        assert_eq!(directed_edge_count, package.audit.edge_count * 2);
        assert!(mesh.to_trimesh().is_ok());
    } else {
        let (vertex_map, adjacency) = mesh.build_connectivity();
        assert_eq!(vertex_map.vertex_count(), 0);
        assert!(adjacency.is_empty());
        assert!(mesh.try_get_vertices_and_indices().is_err());
        assert_eq!(mesh.get_vertices_and_indices(), (Vec::new(), Vec::new()));
        assert!(mesh.to_trimesh().is_err());
    }

    if report.edge_ready() {
        if let Ok(exact) = mesh.to_hypermesh_exact() {
            assert!(exact.validate_retained_state().is_ok());
            assert_eq!(mesh.is_manifold(), exact.facts().mesh.closed_manifold);
            if let Ok(package) = mesh.to_hypermesh_handoff_package() {
                assert!(package.validate_against_mesh(&exact).is_ok());
                assert_eq!(package.readiness.closed_manifold, exact.facts().mesh.closed_manifold);
                assert_eq!(package.readiness.solid_handoff_ready, package.solid.is_some());
                assert_eq!(
                    package.readiness.approximate_f64_view_ready,
                    package.approximate_f64_view.is_some()
                );
            }
            if let Ok(roundtrip) = Mesh::from_hypermesh_exact_lossy(&exact, ()) {
                assert!(roundtrip.is_manifold());
                let (roundtrip_vertices, roundtrip_indices) = roundtrip.get_vertices_and_indices();
                assert_eq!(roundtrip_vertices.len(), exact.vertices().len());
                assert_eq!(roundtrip_indices.len(), exact.triangles().len());
            }
        } else {
            assert!(!mesh.is_manifold());
        }
    } else {
        assert!(!mesh.is_manifold());
    }
});
