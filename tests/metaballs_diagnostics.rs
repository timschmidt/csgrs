use csgrs::solid::{self, MetaBall};
use hyperlattice::{Point3, Real};
use std::collections::HashMap;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn edge_defects(mesh: &hypermesh::TriangleMesh) -> (usize, usize) {
    let mut edges = HashMap::<(usize, usize), (usize, isize)>::new();
    for triangle in mesh.triangles.iter() {
        for [a, b] in [
            [triangle.v0, triangle.v1],
            [triangle.v1, triangle.v2],
            [triangle.v2, triangle.v0],
        ] {
            let key = if a < b { (a, b) } else { (b, a) };
            let entry = edges.entry(key).or_default();
            entry.0 += 1;
            entry.1 += if a < b { 1 } else { -1 };
        }
    }
    (
        edges.values().filter(|(count, _)| *count != 2).count(),
        edges
            .values()
            .filter(|(count, direction)| *count == 2 && *direction != 0)
            .count(),
    )
}

fn signed_volume(mesh: &hypermesh::TriangleMesh) -> f64 {
    mesh.triangles
        .iter()
        .filter_map(|triangle| {
            let a = mesh.positions[triangle.v0].to_vector();
            let b = mesh.positions[triangle.v1].to_vector();
            let c = mesh.positions[triangle.v2].to_vector();
            a.dot(&b.cross(&c)).to_f64_lossy()
        })
        .sum::<f64>()
        / 6.0
}

#[test]
fn metaballs_generate_mesh_from_hyperreal_centers() {
    let balls = [
        MetaBall::new(p3(-0.35, 0.0, 0.0), r(0.6)),
        MetaBall::new(p3(0.35, 0.0, 0.0), r(0.6)),
    ];
    let (mesh, diagnostics) =
        solid::metaballs_with_diagnostics(&balls, (6, 6, 6), r(0.35), r(0.2));

    assert!(diagnostics.crossing_cell_count > 0);
    assert!(diagnostics.surface_nets_vertex_count > 0);
    assert!(
        mesh.is_closed_manifold(),
        "edge defects: {:?}; diagnostics: {diagnostics:?}",
        edge_defects(&mesh),
    );
    assert!(signed_volume(&mesh) > 0.0);
    assert_eq!(mesh.positions.len(), diagnostics.surface_nets_vertex_count);
}

#[test]
fn metaball_influence_at_center_is_finite_hyperreal_value() {
    let ball = MetaBall::new(p3(0.0, 0.0, 0.0), r(1.0));
    let influence = ball
        .influence(&p3(0.0, 0.0, 0.0))
        .expect("valid metaball influence");

    assert_eq!(
        hyperlimit::compare_reals(&influence, &r(0.0)).value(),
        Some(std::cmp::Ordering::Greater)
    );
}

#[test]
fn metaballs_reject_unrepresentable_grid_dimensions() {
    let ball = MetaBall::new(p3(0.0, 0.0, 0.0), r(1.0));
    let (mesh, diagnostics) =
        solid::metaballs_with_diagnostics(&[ball], (usize::MAX, 2, 2), r(1.0), r(0.0));
    assert!(mesh.triangles.is_empty());
    assert_eq!(diagnostics.sample_count, 0);
}
