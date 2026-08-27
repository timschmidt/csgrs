use csgrs::solid::{self, SolidExt};
use hyperlattice::{Point3, Real};
use hyperlimit::{Point3 as HPoint3, PredicatePolicy};
use hypermesh::{MeshCertainty, MeshContext};
use hyperreal::Rational;
use hypersdf::SdfExpr;
use std::collections::HashMap;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).expect("exact test rational"))
}

fn hp3(x: f64, y: f64, z: f64) -> HPoint3 {
    HPoint3::new(r(x), r(y), r(z))
}

fn tiny_exact_real() -> Real {
    let denominator = format!("1{}", "0".repeat(400));
    Real::from(
        format!("1/{denominator}")
            .parse::<Rational>()
            .expect("exact test rational"),
    )
}

fn huge_exact_real() -> Real {
    Real::from(
        format!("1{}", "0".repeat(400))
            .parse::<Rational>()
            .expect("exact test integer"),
    )
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
fn sdf_native_real_surface_nets_produces_closed_mesh() {
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
fn sdf_expr_preserves_exact_signs_below_float_range() {
    let tiny = tiny_exact_real();
    let closure_scale = tiny.clone();
    let expected_max = tiny.clone();
    let surface_x = q(1, 3);
    let iso = surface_x.clone() * tiny.clone();
    let closure_mesh = solid::sdf(
        move |point| &point.x * &closure_scale,
        (9, 9, 9),
        p3(-1.0, -1.0, -1.0),
        p3(1.0, 1.0, 1.0),
        iso.clone(),
    );
    let (expression_mesh, diagnostics) = solid::sdf_expr_with_diagnostics(
        SdfExpr::x().mul_expr(SdfExpr::constant(tiny)),
        (9, 9, 9),
        p3(-1.0, -1.0, -1.0),
        p3(1.0, 1.0, 1.0),
        iso,
    );

    assert!(!closure_mesh.triangles.is_empty());
    assert_eq!(expression_mesh, closure_mesh);
    assert!(
        expression_mesh
            .positions
            .iter()
            .all(|position| position.x == surface_x)
    );
    assert_eq!(diagnostics.negative_sample_count, 6 * 9 * 9);
    assert_eq!(diagnostics.zero_sample_count, 0);
    assert_eq!(diagnostics.positive_sample_count, 3 * 9 * 9);
    assert_eq!(diagnostics.non_finite_sample_count, 0);
    assert_eq!(diagnostics.min_finite_value, Some(-expected_max.clone()));
    assert_eq!(diagnostics.max_finite_value, Some(expected_max));

    let preview = diagnostics
        .hypersdf_preview
        .as_ref()
        .expect("retained expression preview diagnostics");
    assert!(preview.is_self_consistent());
    assert_eq!(preview.grid_samples.samples.negative_count, 6 * 9 * 9);
    assert_eq!(preview.grid_samples.samples.zero_count, 0);
    assert_eq!(preview.grid_samples.samples.positive_count, 3 * 9 * 9);
    assert_eq!(preview.vertex_count, diagnostics.surface_nets_vertex_count);
    assert_eq!(preview.triangle_count, diagnostics.emitted_triangle_count);
}

#[test]
fn sdf_native_real_surface_nets_handles_values_above_float_range() {
    let scale = huge_exact_real();
    let surface_x = q(1, 3);
    let closure_surface_x = surface_x.clone();
    let mesh = solid::sdf(
        move |point| (&point.x - &closure_surface_x) * &scale,
        (3, 3, 3),
        p3(0.0, 0.0, 0.0),
        p3(1.0, 1.0, 1.0),
        Real::zero(),
    );

    assert_eq!(mesh.positions.len(), 4);
    assert_eq!(mesh.triangles.len(), 2);
    assert!(mesh.positions.iter().all(|position| position.x == surface_x));
}

#[test]
fn native_real_sdf_mesh_is_hypermesh_boolean_ready() {
    let radius_squared = q(2, 5);
    let mesh = solid::sdf(
        move |point| {
            point.x.clone() * point.x.clone()
                + point.y.clone() * point.y.clone()
                + point.z.clone() * point.z.clone()
                - radius_squared.clone()
        },
        (9, 9, 9),
        p3(-1.0, -1.0, -1.0),
        p3(1.0, 1.0, 1.0),
        Real::zero(),
    );

    assert!(!mesh.triangles.is_empty());
    assert!(mesh.is_closed_manifold());
    assert!(signed_volume(&mesh) > 0.0);
    let validation =
        hypermesh::polygon_soup(&MeshContext::new(PredicatePolicy::STRICT), &[mesh.as_ref()])
            .expect("native-real Surface Nets output should be valid Hypermesh input");
    assert_eq!(validation.certainty, MeshCertainty::Certified);

    let enclosing = solid::cube(r(2.0)).translated(r(-1.0), r(-1.0), r(-1.0));
    let intersection = mesh
        .try_intersection(&enclosing)
        .expect("native-real SDF mesh should be accepted by Hypermesh Boolean operations");
    assert!(!intersection.triangles.is_empty());
    assert!(intersection.is_closed_manifold());
    assert!(signed_volume(&intersection) > 0.0);
}

#[test]
fn sdf_expr_preview_tracks_nonzero_iso_surface() {
    let expression = SdfExpr::sphere(hp3(0.0, 0.0, 0.0), r(1.0));
    let direct_mesh = solid::sdf_expr(
        expression.clone(),
        (17, 17, 17),
        p3(-2.0, -2.0, -2.0),
        p3(2.0, 2.0, 2.0),
        r(1.0),
    );
    let (diagnostic_mesh, diagnostics) = solid::sdf_expr_with_diagnostics(
        expression,
        (17, 17, 17),
        p3(-2.0, -2.0, -2.0),
        p3(2.0, 2.0, 2.0),
        r(1.0),
    );

    assert_eq!(diagnostic_mesh, direct_mesh);
    assert!(!diagnostic_mesh.triangles.is_empty());
    assert!(diagnostic_mesh.is_closed_manifold());
    assert!(signed_volume(&diagnostic_mesh) > 0.0);

    let preview = diagnostics
        .hypersdf_preview
        .as_ref()
        .expect("retained expression preview diagnostics");
    assert!(preview.is_self_consistent());
    assert_eq!(preview.vertex_count, diagnostics.surface_nets_vertex_count);
    assert_eq!(preview.triangle_count, diagnostics.emitted_triangle_count);
    assert_eq!(
        preview.grid_samples.samples.negative_count,
        diagnostics.negative_sample_count
    );
    assert_eq!(
        preview.grid_samples.samples.zero_count,
        diagnostics.zero_sample_count
    );
    assert_eq!(
        preview.grid_samples.samples.positive_count,
        diagnostics.positive_sample_count
    );
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
