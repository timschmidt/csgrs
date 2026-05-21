#![cfg(feature = "metaballs")]

use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::{
    Mesh,
    metaballs::{MetaBall, MetaballDiagnostics},
};
use nalgebra::Point3;
use std::collections::HashMap;

fn assert_mesh_vertices_finite(mesh: &Mesh<&'static str>) {
    for (polygon_index, polygon) in mesh.polygons.iter().enumerate() {
        assert_eq!(
            polygon.vertices.len(),
            3,
            "metaball output polygon {polygon_index} is not triangular"
        );
        for vertex in &polygon.vertices {
            assert!(
                vertex.position.x.is_finite()
                    && vertex.position.y.is_finite()
                    && vertex.position.z.is_finite(),
                "non-finite metaball vertex in polygon {polygon_index}: {:?}",
                vertex.position
            );
            assert!(
                vertex.normal.x.is_finite()
                    && vertex.normal.y.is_finite()
                    && vertex.normal.z.is_finite(),
                "non-finite metaball normal in polygon {polygon_index}: {:?}",
                vertex.normal
            );
        }
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

fn assert_metaball_diagnostics_consistent(
    mesh: &Mesh<&'static str>,
    diagnostics: &MetaballDiagnostics,
) {
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

#[test]
fn empty_metaball_set_returns_empty_mesh_with_zero_diagnostics() {
    let (mesh, diagnostics) =
        Mesh::<&'static str>::metaballs_with_diagnostics(&[], (0, 1, 2), 1.0, 0.25, "empty");
    assert!(mesh.polygons.is_empty());
    assert_eq!(diagnostics.resolution, (2, 2, 2));
    assert_eq!(diagnostics.ball_count, 0);
    assert_eq!(diagnostics.sample_count, 0);
    assert_eq!(diagnostics.emitted_triangle_count, 0);
}

#[test]
fn single_metaball_triangle_counts_grow_with_resolution() {
    let balls = [MetaBall::new(Point3::new(0.0, 0.0, 0.0), 0.75)];
    let mut previous = 0;

    for resolution in [6usize, 8, 10, 12, 16] {
        let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
            &balls,
            (resolution, resolution, resolution),
            1.0,
            0.5,
            "single",
        );
        assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert!(
            diagnostics.crossing_cell_count > 0,
            "single metaball should cross grid at resolution {resolution}: {diagnostics:#?}"
        );
        assert!(
            diagnostics.emitted_triangle_count > previous,
            "single metaball triangle count collapsed at resolution {resolution}: previous={previous}, diagnostics={diagnostics:#?}"
        );
        assert_eq!(diagnostics.skipped_non_finite_triangle_count, 0);
        previous = diagnostics.emitted_triangle_count;
    }
}

#[test]
fn single_metaball_with_padding_is_closed_and_non_degenerate() {
    let balls = [MetaBall::new(Point3::new(0.0, 0.0, 0.0), 0.75)];
    let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
        &balls,
        (18, 18, 18),
        1.0,
        0.5,
        "closed",
    );
    assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
    assert_mesh_vertices_finite(&mesh);
    assert!(
        !mesh.polygons.is_empty(),
        "single metaball should produce triangles: {diagnostics:#?}"
    );
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "closed metaball should not have missing-triangle boundary edges: {diagnostics:#?}"
    );
    assert!(
        mesh.polygons
            .iter()
            .all(|poly| triangle_area2(poly) > tolerance()),
        "metaballs emitted degenerate triangles: {diagnostics:#?}"
    );
}

#[test]
fn too_high_iso_value_reports_no_crossings_or_triangles() {
    let balls = [MetaBall::new(Point3::new(0.0, 0.0, 0.0), 0.75)];
    let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
        &balls,
        (12, 12, 12),
        1.0e12,
        0.5,
        "high_iso",
    );
    assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
    assert!(mesh.polygons.is_empty(), "{diagnostics:#?}");
    assert_eq!(diagnostics.crossing_cell_count, 0, "{diagnostics:#?}");
    assert_eq!(diagnostics.positive_sample_count, 0, "{diagnostics:#?}");
}

#[test]
fn tiny_positive_metaball_samples_keep_hyperreal_sign_before_surface_nets() {
    let balls = [MetaBall::new(Point3::new(0.0, 0.0, 0.0), 1.0e-25)];
    let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
        &balls,
        (2, 2, 2),
        -1.0e-50,
        1.0,
        "tiny_positive",
    );
    assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
    assert!(mesh.polygons.is_empty(), "{diagnostics:#?}");
    assert_eq!(diagnostics.sample_count, 8);
    assert_eq!(diagnostics.finite_sample_count, 8);
    assert_eq!(diagnostics.positive_sample_count, 8);
    assert_eq!(diagnostics.zero_sample_count, 0);
    assert_eq!(diagnostics.negative_sample_count, 0);
    assert_eq!(diagnostics.crossing_cell_count, 0);
}

#[test]
fn separated_metaballs_have_more_triangles_than_one_ball() {
    let one = [MetaBall::new(Point3::new(0.0, 0.0, 0.0), 0.55)];
    let separated = [
        MetaBall::new(Point3::new(-1.2, 0.0, 0.0), 0.55),
        MetaBall::new(Point3::new(1.2, 0.0, 0.0), 0.55),
    ];
    let (one_mesh, one_diagnostics) =
        Mesh::<&'static str>::metaballs_with_diagnostics(&one, (18, 18, 18), 1.0, 0.45, "one");
    let (two_mesh, two_diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
        &separated,
        (18, 18, 18),
        1.0,
        0.45,
        "two",
    );
    assert_metaball_diagnostics_consistent(&one_mesh, &one_diagnostics);
    assert_metaball_diagnostics_consistent(&two_mesh, &two_diagnostics);
    assert_mesh_vertices_finite(&one_mesh);
    assert_mesh_vertices_finite(&two_mesh);
    assert!(
        two_diagnostics.emitted_triangle_count > one_diagnostics.emitted_triangle_count,
        "two separated components should not lose triangles: one={one_diagnostics:#?}, two={two_diagnostics:#?}"
    );
}

#[test]
fn overlapping_metaballs_keep_a_healthy_crossing_and_triangle_count() {
    let balls = [
        MetaBall::new(Point3::new(-0.45, 0.0, 0.0), 0.65),
        MetaBall::new(Point3::new(0.45, 0.0, 0.0), 0.65),
        MetaBall::new(Point3::new(0.0, 0.45, 0.2), 0.55),
    ];
    let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
        &balls,
        (20, 20, 20),
        1.0,
        0.45,
        "overlap",
    );
    assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
    assert_mesh_vertices_finite(&mesh);
    assert!(
        diagnostics.crossing_cell_count > 100,
        "overlapping metaballs should expose many crossing cells: {diagnostics:#?}"
    );
    assert!(
        diagnostics.emitted_triangle_count >= diagnostics.crossing_cell_count,
        "surface nets emitted too few triangles for crossing cells: {diagnostics:#?}"
    );
    assert_eq!(diagnostics.skipped_non_finite_triangle_count, 0);
}

#[test]
fn zero_negative_and_non_finite_radii_are_contained_by_diagnostics() {
    let cases: &[&[MetaBall]] = &[
        &[MetaBall::new(Point3::new(0.0, 0.0, 0.0), 0.0)],
        &[MetaBall::new(Point3::new(0.0, 0.0, 0.0), -0.75)],
        &[MetaBall::new(Point3::new(0.0, 0.0, 0.0), Real::INFINITY)],
        &[MetaBall::new(Point3::new(Real::NAN, 0.0, 0.0), 0.75)],
    ];

    for balls in cases {
        let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
            balls,
            (8, 8, 8),
            1.0,
            0.25,
            "pathological",
        );
        assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert_eq!(
            diagnostics.skipped_non_finite_triangle_count, 0,
            "non-finite inputs should be sanitized before triangle emission: {diagnostics:#?}"
        );
    }
}

#[test]
fn influence_uses_hyperlattice_distance_and_fails_closed() {
    let ball = MetaBall::new(Point3::new(0.0, 0.0, 0.0), 0.75);

    let near = ball.influence(&Point3::new(0.25, 0.0, 0.0));
    let far = ball.influence(&Point3::new(10_000.0, 0.0, 0.0));
    assert!(near.is_finite() && near > 0.0);
    assert_eq!(far, 0.0);

    assert_eq!(ball.influence(&Point3::new(Real::NAN, 0.0, 0.0)), 0.0);
    assert_eq!(
        MetaBall::new(Point3::new(0.0, 0.0, 0.0), Real::INFINITY)
            .influence(&Point3::new(0.0, 0.0, 0.0)),
        0.0
    );
    assert_eq!(
        MetaBall::new(Point3::new(0.0, 0.0, 0.0), -0.75)
            .influence(&Point3::new(0.0, 0.0, 0.0)),
        0.0
    );
}

#[test]
fn anisotropic_resolution_does_not_drop_all_triangles() {
    let balls = [
        MetaBall::new(Point3::new(-0.35, 0.0, 0.0), 0.65),
        MetaBall::new(Point3::new(0.35, 0.0, 0.0), 0.65),
    ];

    for resolution in [(4, 18, 18), (18, 4, 18), (18, 18, 4), (7, 13, 19)] {
        let (mesh, diagnostics) = Mesh::<&'static str>::metaballs_with_diagnostics(
            &balls,
            resolution,
            1.0,
            0.5,
            "anisotropic",
        );
        assert_metaball_diagnostics_consistent(&mesh, &diagnostics);
        assert_mesh_vertices_finite(&mesh);
        assert!(
            diagnostics.emitted_triangle_count > 0,
            "anisotropic resolution {resolution:?} dropped all triangles: {diagnostics:#?}"
        );
    }
}
