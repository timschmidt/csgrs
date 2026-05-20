#![cfg(feature = "sdf")]

use csgrs::csg::CSG;
use csgrs::float_types::{Real, hreal_from_f64};
use csgrs::mesh::{Mesh, sdf::SdfDiagnostics};
use hyperlimit::Point3 as HPoint3;
use hypersdf::{SdfExpr, SdfSampleTopologyStatus};
use nalgebra::Point3;
use std::collections::HashMap;

fn assert_mesh_vertices_finite(mesh: &Mesh<&'static str>) {
    for (polygon_index, polygon) in mesh.polygons.iter().enumerate() {
        for vertex in &polygon.vertices {
            assert!(
                vertex.position.x.is_finite()
                    && vertex.position.y.is_finite()
                    && vertex.position.z.is_finite(),
                "non-finite SDF vertex in polygon {polygon_index}: {:?}",
                vertex.position
            );
            assert!(
                vertex.normal.x.is_finite()
                    && vertex.normal.y.is_finite()
                    && vertex.normal.z.is_finite(),
                "non-finite SDF normal in polygon {polygon_index}: {:?}",
                vertex.normal
            );
        }
    }
}

fn assert_sdf_mesh_is_triangular(mesh: &Mesh<&'static str>) {
    for (polygon_index, polygon) in mesh.polygons.iter().enumerate() {
        assert_eq!(
            polygon.vertices.len(),
            3,
            "SDF output polygon {polygon_index} is not triangular"
        );
    }
}

fn triangle_area2(poly: &csgrs::polygon::Polygon<&'static str>) -> Real {
    let a = poly.vertices[0].position;
    let b = poly.vertices[1].position;
    let c = poly.vertices[2].position;
    (b - a).cross(&(c - a)).norm()
}

fn quantized_point_key(point: Point3<Real>) -> (i64, i64, i64) {
    let scale = 1_000_000_000.0;
    (
        (point.x * scale).round() as i64,
        (point.y * scale).round() as i64,
        (point.z * scale).round() as i64,
    )
}

fn boundary_edge_count(mesh: &Mesh<&'static str>) -> usize {
    let mut edge_counts = HashMap::<[(i64, i64, i64); 2], usize>::new();
    for poly in &mesh.polygons {
        let keys = [
            quantized_point_key(poly.vertices[0].position),
            quantized_point_key(poly.vertices[1].position),
            quantized_point_key(poly.vertices[2].position),
        ];
        for edge in [[keys[0], keys[1]], [keys[1], keys[2]], [keys[2], keys[0]]] {
            let mut ordered = edge;
            ordered.sort();
            *edge_counts.entry(ordered).or_default() += 1;
        }
    }

    edge_counts.values().filter(|count| **count == 1).count()
}

fn assert_sdf_diagnostics_consistent(mesh: &Mesh<&'static str>, diagnostics: &SdfDiagnostics) {
    assert_eq!(
        diagnostics.sample_count,
        diagnostics.finite_sample_count + diagnostics.non_finite_sample_count
    );
    assert_eq!(
        diagnostics.sample_count,
        diagnostics.negative_sample_count
            + diagnostics.zero_sample_count
            + diagnostics.positive_sample_count
    );
    assert_eq!(diagnostics.surface_nets_index_count % 3, 0);
    assert_eq!(diagnostics.emitted_triangle_count, mesh.polygons.len());
    assert_eq!(
        diagnostics.surface_nets_index_count / 3,
        diagnostics.emitted_triangle_count + diagnostics.skipped_non_finite_triangle_count
    );
}

fn hpoint3(x: Real, y: Real, z: Real) -> HPoint3 {
    HPoint3::new(
        hreal_from_f64(x).unwrap(),
        hreal_from_f64(y).unwrap(),
        hreal_from_f64(z).unwrap(),
    )
}

fn gyroid_value(point: &Point3<Real>, period: Real) -> Real {
    let scale = std::f64::consts::TAU as Real / period;
    let x = point.x * scale;
    let y = point.y * scale;
    let z = point.z * scale;
    x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos()
}

fn schwarz_p_value(point: &Point3<Real>, period: Real) -> Real {
    let scale = std::f64::consts::TAU as Real / period;
    (point.x * scale).cos() + (point.y * scale).cos() + (point.z * scale).cos()
}

fn schwarz_d_value(point: &Point3<Real>, period: Real) -> Real {
    let scale = std::f64::consts::TAU as Real / period;
    let (sx, cx) = (point.x * scale).sin_cos();
    let (sy, cy) = (point.y * scale).sin_cos();
    let (sz, cz) = (point.z * scale).sin_cos();
    sx * sy * sz + sx * cy * cz + cx * sy * cz + cx * cy * sz
}

#[test]
fn sdf_constant_fields_have_no_crossing_cells_or_triangles() {
    for (value, expected_negative, expected_positive) in
        [(-1.0, true, false), (1.0, false, true)]
    {
        let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
            move |_| value,
            (6, 7, 8),
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            0.0,
            "constant",
        );
        assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
        assert!(mesh.polygons.is_empty(), "{diagnostics:#?}");
        assert_eq!(diagnostics.crossing_cell_count, 0, "{diagnostics:#?}");
        assert_eq!(diagnostics.negative_sample_count > 0, expected_negative);
        assert_eq!(diagnostics.positive_sample_count > 0, expected_positive);
    }
}

#[test]
fn sdf_axis_aligned_planes_emit_triangles_for_every_crossing_slab() {
    let bounds = (Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
    let resolution = (9, 10, 11);
    let cases: [fn(&Point3<Real>) -> Real; 3] =
        [|p| p.x - 0.125, |p| p.y + 0.125, |p| p.z - 0.125];

    for sdf in cases {
        let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
            sdf, resolution, bounds.0, bounds.1, 0.0, "plane",
        );
        assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert_sdf_mesh_is_triangular(&mesh);
        assert!(
            diagnostics.crossing_cell_count > 0,
            "plane should cross the sampled volume: {diagnostics:#?}"
        );
        assert!(
            diagnostics.emitted_triangle_count >= diagnostics.crossing_cell_count,
            "a plane should not emit fewer triangles than crossing cells: {diagnostics:#?}"
        );
        assert_eq!(
            diagnostics.skipped_non_finite_triangle_count, 0,
            "{diagnostics:#?}"
        );
    }
}

#[test]
fn sdf_sphere_triangle_counts_do_not_collapse_as_resolution_increases() {
    let mut previous = 0;
    for resolution in [6usize, 8, 10, 12, 16] {
        let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
            |p| p.coords.norm() - 0.72,
            (resolution, resolution, resolution),
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            0.0,
            "sphere",
        );
        assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert_sdf_mesh_is_triangular(&mesh);
        assert!(
            diagnostics.emitted_triangle_count > previous,
            "sphere triangle count collapsed at resolution {resolution}: previous={previous}, diagnostics={diagnostics:#?}"
        );
        assert_eq!(diagnostics.skipped_non_finite_triangle_count, 0);
        previous = diagnostics.emitted_triangle_count;
    }
}

#[test]
fn hypersdf_sphere_expression_is_the_preferred_sdf_mesh_source() {
    let expr = SdfExpr::sphere(hpoint3(0.0, 0.0, 0.0), hreal_from_f64(0.72 * 0.72).unwrap());
    let (mesh, diagnostics) = Mesh::<&'static str>::sdf_expr_with_diagnostics(
        expr,
        (12, 12, 12),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
        0.0,
        "hypersdf_sphere",
    );

    assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
    assert_mesh_vertices_finite(&mesh);
    assert_sdf_mesh_is_triangular(&mesh);
    assert!(
        diagnostics.emitted_triangle_count > 0,
        "retained hypersdf sphere should emit a mesh preview: {diagnostics:#?}"
    );
    let preview = diagnostics
        .hypersdf_preview
        .as_ref()
        .expect("hypersdf expression path should retain a preview report");
    assert_eq!(preview.topology_status, SdfSampleTopologyStatus::PreviewOnly);
    assert_eq!(
        preview.grid_samples.samples.sample_count,
        diagnostics.sample_count
    );
    assert_eq!(
        preview.grid_samples.samples.non_finite_count,
        diagnostics.non_finite_sample_count
    );
}

#[test]
fn sdf_reports_non_finite_samples_without_silently_dropping_generated_triangles() {
    let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
        |p| {
            if p.x > 0.55 {
                Real::NAN
            } else {
                p.coords.norm() - 0.7
            }
        },
        (14, 14, 14),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
        0.0,
        "nonfinite",
    );
    assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
    assert_mesh_vertices_finite(&mesh);
    assert_sdf_mesh_is_triangular(&mesh);
    assert!(
        diagnostics.non_finite_sample_count > 0,
        "diagnostics should expose sanitized non-finite samples: {diagnostics:#?}"
    );
    assert_eq!(
        diagnostics.skipped_non_finite_triangle_count, 0,
        "sanitized samples should not create non-finite mesh triangles: {diagnostics:#?}"
    );
}

#[test]
fn sdf_degenerate_and_reversed_bounds_are_finite_and_diagnosed() {
    for (min, max) in [
        (Point3::new(-1.0, -1.0, -1.0), Point3::new(-1.0, 1.0, 1.0)),
        (Point3::new(1.0, 1.0, 1.0), Point3::new(-1.0, -1.0, -1.0)),
    ] {
        let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
            |p| p.coords.norm() - 0.5,
            (8, 8, 8),
            min,
            max,
            0.0,
            "bounds",
        );
        assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert_sdf_mesh_is_triangular(&mesh);
    }
}

#[test]
fn raw_tpms_sdf_surfaces_emit_triangles_before_boolean_clipping() {
    let min = Point3::new(-2.0, -2.0, -2.0);
    let max = Point3::new(2.0, 2.0, 2.0);
    let resolution = (18, 18, 18);
    let cases: [(&str, fn(&Point3<Real>, Real) -> Real); 3] = [
        ("gyroid", gyroid_value),
        ("schwarz_p", schwarz_p_value),
        ("schwarz_d", schwarz_d_value),
    ];

    for (name, sdf) in cases {
        let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
            |p| sdf(p, 1.0),
            resolution,
            min,
            max,
            0.0,
            name,
        );
        assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert_sdf_mesh_is_triangular(&mesh);
        assert!(
            diagnostics.crossing_cell_count > 0,
            "{name} should cross the sampled cube: {diagnostics:#?}"
        );
        assert!(
            diagnostics.emitted_triangle_count > 0,
            "{name} raw SDF emitted no triangles: {diagnostics:#?}"
        );
        assert_eq!(
            diagnostics.skipped_non_finite_triangle_count, 0,
            "{name} should not require triangle skipping: {diagnostics:#?}"
        );
    }
}

#[test]
fn tpms_helpers_match_raw_sdf_mesh_for_their_bounding_box() {
    let cube = Mesh::<&'static str>::cube(4.0, "clip");
    let aabb = cube.bounding_box();
    let min = aabb.mins;
    let max = aabb.maxs;
    let resolution = (18, 18, 18);
    let cases: [(&str, fn(&Point3<Real>, Real) -> Real, Mesh<&'static str>); 3] = [
        (
            "gyroid",
            gyroid_value,
            cube.gyroid(resolution.0, 1.0, 0.0, "gyroid"),
        ),
        (
            "schwarz_p",
            schwarz_p_value,
            cube.schwarz_p(resolution.0, 1.0, 0.0, "schwarz_p"),
        ),
        (
            "schwarz_d",
            schwarz_d_value,
            cube.schwarz_d(resolution.0, 1.0, 0.0, "schwarz_d"),
        ),
    ];

    for (name, sdf, clipped) in cases {
        let (raw, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
            |p| sdf(p, 1.0),
            resolution,
            min,
            max,
            0.0,
            name,
        );
        assert_sdf_diagnostics_consistent(&raw, &diagnostics);
        assert_mesh_vertices_finite(&clipped);
        assert_sdf_mesh_is_triangular(&clipped);
        assert_eq!(
            clipped.polygons.len(),
            raw.polygons.len(),
            "{name} helper should not boolean-clip away raw TPMS triangles: raw={}, helper={}, diagnostics={diagnostics:#?}",
            raw.polygons.len(),
            clipped.polygons.len()
        );
    }
}

#[test]
fn closed_sdf_sphere_has_no_boundary_edges_or_degenerate_triangles() {
    let (mesh, diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
        |p| p.coords.norm() - 0.75,
        (18, 18, 18),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
        0.0,
        "sphere",
    );
    assert_sdf_diagnostics_consistent(&mesh, &diagnostics);
    assert_mesh_vertices_finite(&mesh);
    assert_sdf_mesh_is_triangular(&mesh);
    assert!(
        !mesh.polygons.is_empty(),
        "sphere should produce triangles: {diagnostics:#?}"
    );
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "closed sphere SDF should not have missing-triangle boundary edges: {diagnostics:#?}"
    );
    assert!(
        mesh.polygons
            .iter()
            .all(|poly| triangle_area2(poly) > Real::EPSILON),
        "sphere SDF emitted degenerate triangles: {diagnostics:#?}"
    );
}

#[test]
fn readme_sdf_examples_mesh_non_empty_surfaces() {
    let (sphere, sphere_diagnostics) = Mesh::<&'static str>::sdf_with_diagnostics(
        |p| p.coords.norm() - 0.75,
        (20, 20, 20),
        Point3::new(-1.1, -1.1, -1.1),
        Point3::new(1.1, 1.1, 1.1),
        0.0,
        "readme_sdf",
    );
    assert_sdf_diagnostics_consistent(&sphere, &sphere_diagnostics);
    assert_mesh_vertices_finite(&sphere);
    assert_sdf_mesh_is_triangular(&sphere);
    assert!(
        sphere_diagnostics.emitted_triangle_count > 0,
        "README SDF sphere emitted no triangles: {sphere_diagnostics:#?}"
    );
    assert_eq!(
        boundary_edge_count(&sphere),
        0,
        "README SDF sphere should be closed: {sphere_diagnostics:#?}"
    );

    let box_mesh = Mesh::<&'static str>::cube(2.0, "tpms");
    for (name, mesh) in [
        ("gyroid", box_mesh.gyroid_solid(24, 2.0, 0.0, 0.18, "gyroid")),
        (
            "schwarz_p",
            box_mesh.schwarz_p_solid(24, 2.0, 0.0, 0.18, "schwarz_p"),
        ),
        (
            "schwarz_d",
            box_mesh.schwarz_d_solid(24, 2.0, 0.0, 0.18, "schwarz_d"),
        ),
    ] {
        assert_mesh_vertices_finite(&mesh);
        assert_sdf_mesh_is_triangular(&mesh);
        assert!(
            mesh.polygons.len() > 100,
            "{name} README TPMS example emitted too few triangles: {}",
            mesh.polygons.len()
        );
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{name} README TPMS solid should be capped and closed"
        );
    }
}

#[test]
fn tpms_solid_helpers_emit_closed_capped_meshes() {
    let cube = Mesh::<&'static str>::cube(2.0, "solid");
    for (name, mesh) in [
        ("gyroid", cube.gyroid_solid(22, 2.0, 0.0, 0.2, "gyroid")),
        (
            "schwarz_p",
            cube.schwarz_p_solid(22, 2.0, 0.0, 0.2, "schwarz_p"),
        ),
        (
            "schwarz_d",
            cube.schwarz_d_solid(22, 2.0, 0.0, 0.2, "schwarz_d"),
        ),
    ] {
        assert_mesh_vertices_finite(&mesh);
        assert_sdf_mesh_is_triangular(&mesh);
        assert!(
            mesh.polygons.len() > 100,
            "{name} solid emitted too few triangles: {}",
            mesh.polygons.len()
        );
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{name} solid should not have open boundary edges"
        );
        assert!(
            mesh.polygons
                .iter()
                .all(|poly| triangle_area2(poly) > Real::EPSILON),
            "{name} solid emitted degenerate triangles"
        );
    }
}

#[test]
fn tpms_solid_helpers_reject_non_positive_thickness() {
    let cube = Mesh::<&'static str>::cube(2.0, "solid");
    for thickness in [0.0, -0.1, Real::NAN, Real::INFINITY] {
        assert!(
            cube.gyroid_solid(12, 2.0, 0.0, thickness, "gyroid")
                .polygons
                .is_empty(),
            "invalid thickness {thickness:?} should return an empty gyroid solid"
        );
    }
}
