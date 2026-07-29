use csgrs::solid;
use hyperlattice::{Point3, Real};
use std::collections::HashMap;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn edge_defects(mesh: &hypermesh::TriangleMesh) -> (usize, usize, Vec<(usize, usize)>) {
    let mut edges = HashMap::<(usize, usize), (usize, isize)>::new();
    let mut directed = Vec::new();
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
            directed.push((a, b));
        }
    }
    let boundary = directed
        .into_iter()
        .filter(|(a, b)| edges[&((*a).min(*b), (*a).max(*b))].0 != 2)
        .collect::<Vec<_>>();
    let mut degree = HashMap::<usize, (usize, usize)>::new();
    for &(a, b) in &boundary {
        degree.entry(a).or_default().0 += 1;
        degree.entry(b).or_default().1 += 1;
    }
    (
        edges.values().filter(|(count, _)| *count != 2).count(),
        edges
            .values()
            .filter(|(count, direction)| *count == 2 && *direction != 0)
            .count(),
        degree.into_values().collect(),
    )
}

fn boundary_bounds(mesh: &hypermesh::TriangleMesh) -> ([f64; 3], [f64; 3]) {
    let mut edges = HashMap::<(usize, usize), usize>::new();
    for triangle in mesh.triangles.iter() {
        for [a, b] in [
            [triangle.v0, triangle.v1],
            [triangle.v1, triangle.v2],
            [triangle.v2, triangle.v0],
        ] {
            *edges.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let mut mins = [f64::INFINITY; 3];
    let mut maxs = [f64::NEG_INFINITY; 3];
    for ((a, b), count) in edges {
        if count == 2 {
            continue;
        }
        for index in [a, b] {
            let point = &mesh.positions[index];
            for (axis, value) in [point.x.clone(), point.y.clone(), point.z.clone()]
                .into_iter()
                .enumerate()
            {
                let value = value.to_f64_lossy().expect("finite test mesh");
                mins[axis] = mins[axis].min(value);
                maxs[axis] = maxs[axis].max(value);
            }
        }
    }
    (mins, maxs)
}

fn duplicate_triangles(mesh: &hypermesh::TriangleMesh) -> usize {
    let mut counts = HashMap::new();
    for triangle in mesh.triangles.iter() {
        let mut vertices = triangle.indices();
        vertices.sort_unstable();
        *counts.entry(vertices).or_insert(0usize) += 1;
    }
    counts.values().map(|count| count.saturating_sub(1)).sum()
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
fn sdf_boundary_converts_only_at_sampling_boundary() {
    let mesh = solid::sdf(
        |p| p.to_vector().norm() - r(0.6),
        (8, 8, 8),
        p3(-1.0, -1.0, -1.0),
        p3(1.0, 1.0, 1.0),
        r(0.0),
    );

    assert!(!mesh.triangles.is_empty());
    assert!(mesh.is_closed_manifold());
}

#[test]
fn tpms_solid_accepts_positive_hyperreal_period_and_thickness() {
    let volume = solid::cube(r(2.0));
    let solids = [
        (
            "gyroid",
            solid::gyroid_solid(&volume, 10, r(2.0), r(0.0), r(0.4)),
        ),
        (
            "Schwarz-P",
            solid::schwarz_p_solid(&volume, 10, r(2.0), r(0.0), r(0.4)),
        ),
        (
            "Schwarz-D",
            solid::schwarz_d_solid(&volume, 10, r(2.0), r(0.0), r(0.4)),
        ),
    ];
    for (name, mesh) in solids {
        assert!(!mesh.triangles.is_empty(), "{name}");
        assert!(
            mesh.is_closed_manifold(),
            "{name} edge defects: {:?}; boundary bounds: {:?}; duplicates: {}",
            edge_defects(&mesh),
            boundary_bounds(&mesh),
            duplicate_triangles(&mesh)
        );
        assert!(signed_volume(&mesh) > 0.0, "{name} outward winding");
    }
}

#[test]
fn implicit_shapes_reject_unrepresentable_grid_dimensions() {
    let volume = solid::cube(r(2.0));
    assert!(
        solid::gyroid(&volume, usize::MAX, r(2.0), r(0.0))
            .triangles
            .is_empty()
    );
    assert!(
        solid::sdf(
            |point| point.to_vector().norm(),
            (usize::MAX, 2, 2),
            p3(-1.0, -1.0, -1.0),
            p3(1.0, 1.0, 1.0),
            r(0.0),
        )
        .triangles
        .is_empty()
    );
}
