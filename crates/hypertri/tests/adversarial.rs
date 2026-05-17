#![cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]

#[cfg(feature = "cdt")]
use hypertri::Constraint;
use hypertri::{ExactPoint, Point2, Rational, Real};
#[cfg(feature = "earcut")]
use proptest::prelude::*;

fn p(x: i32, y: i32) -> ExactPoint {
    Point2::new(Real::from(x), Real::from(y))
}

#[cfg(any(feature = "earcut", feature = "cdt", feature = "f64-interop"))]
fn q(xn: i64, xd: u64, yn: i64, yd: u64) -> ExactPoint {
    Point2::new(
        Real::from(Rational::fraction(xn, xd).unwrap()),
        Real::from(Rational::fraction(yn, yd).unwrap()),
    )
}

#[cfg(feature = "nd")]
fn point_d(values: &[Real]) -> hypertri::nd::PointD {
    hypertri::nd::PointD::new(values.to_vec())
}

#[cfg(feature = "nd")]
fn r(value: i64) -> Real {
    Real::from(value)
}

#[cfg(feature = "nd")]
fn rq(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).unwrap())
}

#[test]
fn polygon_input_retains_ring_structural_facts() {
    let input = hypertri::PolygonInput::new(
        vec![
            p(0, 0),
            p(2, 0),
            p(2, 0),
            p(2, 2),
            p(0, 2),
            p(1, 1),
            p(1, 1),
            p(1, 2),
        ],
        vec![5],
    );

    let facts = input.facts();

    assert_eq!(facts.vertex_count, 8);
    assert_eq!(facts.ring_count, 2);
    assert!(facts.has_holes);
    assert_eq!(facts.exact_rational_coordinates, 16);
    assert_eq!(facts.rings[0].start, 0);
    assert_eq!(facts.rings[0].end, 5);
    assert_eq!(facts.rings[0].known_degenerate_edges, 1);
    assert_eq!(facts.rings[0].known_axis_aligned_edges, 4);
    assert_eq!(facts.rings[0].unknown_edge_zero_status, 0);
    assert_eq!(facts.rings[0].signed_area, Some(hypertri::Sign::Positive));
    assert_eq!(
        facts.rings[0].convexity,
        hypertri::RingConvexity::LocallyConvex
    );
    assert_eq!(facts.rings[1].start, 5);
    assert_eq!(facts.rings[1].end, 8);
    assert_eq!(facts.rings[1].known_degenerate_edges, 1);
    assert_eq!(facts.rings[1].known_axis_aligned_edges, 2);
    assert_eq!(facts.rings[1].unknown_edge_zero_status, 0);
    assert_eq!(facts.rings[1].signed_area, Some(hypertri::Sign::Zero));
    assert_eq!(
        facts.rings[1].convexity,
        hypertri::RingConvexity::Degenerate
    );
    assert_eq!(facts.known_degenerate_edge_count(), 2);
    assert_eq!(facts.unknown_edge_zero_status_count(), 0);
    assert!(facts.all_ring_orientations_certified());
    assert_eq!(facts.unknown_convexity_ring_count(), 0);
}

#[test]
fn polygon_input_retains_exact_ring_orientation_and_turn_facts() {
    let convex = hypertri::PolygonInput::new(vec![p(0, 0), p(4, 0), p(4, 3), p(0, 3)], vec![]);
    assert_eq!(
        convex.facts().rings[0].signed_area,
        Some(hypertri::Sign::Positive)
    );
    assert_eq!(
        convex.facts().rings[0].convexity,
        hypertri::RingConvexity::LocallyConvex
    );

    let concave =
        hypertri::PolygonInput::new(vec![p(0, 0), p(4, 0), p(4, 4), p(2, 1), p(0, 4)], vec![]);
    assert_eq!(
        concave.facts().rings[0].signed_area,
        Some(hypertri::Sign::Positive)
    );
    assert_eq!(
        concave.facts().rings[0].convexity,
        hypertri::RingConvexity::MixedTurns
    );

    let reversed = hypertri::PolygonInput::new(vec![p(0, 3), p(4, 3), p(4, 0), p(0, 0)], vec![]);
    assert_eq!(
        reversed.facts().rings[0].signed_area,
        Some(hypertri::Sign::Negative)
    );
    assert_eq!(
        reversed.facts().rings[0].convexity,
        hypertri::RingConvexity::LocallyConvex
    );
}

#[test]
#[cfg(feature = "nd")]
fn exact_nd_delaunay_stars_3d_simplex_around_rational_interior_point() {
    let points = vec![
        point_d(&[r(0), r(0), r(0)]),
        point_d(&[r(1), r(0), r(0)]),
        point_d(&[r(0), r(1), r(0)]),
        point_d(&[r(0), r(0), r(1)]),
        point_d(&[rq(1, 4), rq(1, 4), rq(1, 4)]),
    ];

    let complex = hypertri::nd::delaunay_complex(&points).unwrap();

    assert_eq!(complex.dimension(), 3);
    assert_eq!(complex.cells().len(), 4);
    assert!(
        complex
            .cells()
            .iter()
            .all(|cell| cell.indices().contains(&4))
    );
    complex.validate().unwrap();
}

#[test]
#[cfg(feature = "nd")]
fn exact_nd_delaunay_preserves_cospherical_cells_as_complex() {
    let points = vec![
        point_d(&[r(0), r(0)]),
        point_d(&[r(1), r(0)]),
        point_d(&[r(1), r(1)]),
        point_d(&[r(0), r(1)]),
    ];

    let complex = hypertri::nd::delaunay_complex(&points).unwrap();

    assert_eq!(complex.dimension(), 2);
    assert_eq!(
        complex.cells().len(),
        4,
        "cospherical square is represented as a Delaunay complex, not an arbitrary float tie-break"
    );
    complex.validate().unwrap();
}

#[test]
#[cfg(feature = "earcut")]
fn exact_earcut_keeps_tiny_rational_spike() {
    let vertices = vec![
        p(0, 0),
        q(1, 1_000_000_000_000, 1, 1_000_000_000_000_000),
        p(3, 0),
        p(3, 2),
        p(0, 2),
    ];

    let triangles = hypertri::earcut(&vertices, &[]).unwrap();

    assert_eq!(triangles.len(), 9);
}

#[test]
#[cfg(all(feature = "f64-interop", feature = "earcut"))]
fn f64_boundary_rejects_infinity_before_exact_lift() {
    let vertices = [[0.0, 0.0], [f64::INFINITY, 0.0], [1.0, 1.0]];

    let error = hypertri::f64::earcut(&vertices, &[]).unwrap_err();

    assert_eq!(
        error,
        hypertri::Error::InvalidInput {
            reason: "f64 coordinates must be finite"
        }
    );
}

#[test]
#[cfg(all(feature = "f64-interop", feature = "cdt"))]
fn f64_delaunay_lifts_larger_point_set_to_exact_path() {
    let points = [
        [0.0, 0.0],
        [3.0, 0.0],
        [5.0, 2.0],
        [4.0, 5.0],
        [1.0, 4.0],
        [2.0, 2.0],
    ];

    let triangulation = hypertri::f64::delaunay(&points).unwrap();

    assert_eq!(triangulation.triangles().len(), 5);
    assert!(
        triangulation
            .triangles()
            .iter()
            .flatten()
            .all(|&index| index < points.len())
    );
}

#[test]
#[cfg(all(feature = "f64-interop", feature = "cdt"))]
fn f64_cdt_returns_inserted_intersection_point() {
    let points = [[0.0, 0.0], [2.0, 2.0], [0.0, 2.0], [2.0, 0.0]];
    let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

    let triangulation = hypertri::f64::constrained_delaunay(&points, &constraints).unwrap();

    assert_eq!(triangulation.constraints(), constraints.as_slice());
    assert_eq!(
        triangulation.constraint_edges(),
        &[
            Constraint::new(0, 4),
            Constraint::new(4, 1),
            Constraint::new(2, 4),
            Constraint::new(4, 3),
        ]
    );
    assert_eq!(triangulation.points().len(), 5);
    assert_eq!(triangulation.points()[4], p(1, 1));
    assert!(
        triangulation
            .triangles()
            .iter()
            .flatten()
            .all(|&index| index < triangulation.points().len())
    );
}

#[test]
#[cfg(all(feature = "cdt", feature = "earcut"))]
fn cdt_closed_ring_accepts_reversed_constraint_edge() {
    let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
    let constraints = vec![
        Constraint::new(0, 1),
        Constraint::new(2, 1),
        Constraint::new(2, 3),
        Constraint::new(3, 0),
    ];

    let triangulation = hypertri::cdt::constrained_delaunay(&points, &constraints).unwrap();

    assert_eq!(triangulation.triangles().len(), 2);
}

#[test]
#[cfg(feature = "cdt")]
fn cdt_splits_tiny_exact_rational_constraint_crossing() {
    let points = vec![
        p(0, 0),
        q(1, 1, 1, 1_000_000_000_000),
        q(0, 1, 1, 1_000_000_000_000),
        p(1, 0),
    ];
    let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

    let triangulation = hypertri::cdt::constrained_delaunay(&points, &constraints).unwrap();

    assert_eq!(triangulation.constraints(), constraints.as_slice());
    assert_eq!(triangulation.points().len(), 5);
    assert_eq!(triangulation.points()[4], q(1, 2, 1, 2_000_000_000_000));
    assert_eq!(
        triangulation.constraint_edges(),
        &[
            Constraint::new(0, 4),
            Constraint::new(4, 1),
            Constraint::new(2, 4),
            Constraint::new(4, 3),
        ]
    );
    for edge in [
        Constraint::new(0, 4),
        Constraint::new(4, 1),
        Constraint::new(2, 4),
        Constraint::new(4, 3),
    ] {
        assert!(
            triangulation
                .triangles()
                .iter()
                .any(|triangle| triangle.contains(&edge.from) && triangle.contains(&edge.to))
        );
    }
}

#[test]
#[cfg(all(feature = "serde", feature = "cdt"))]
fn serde_roundtrips_public_topology_and_rebuilds_polygon_facts() {
    let input = hypertri::PolygonInput::new(vec![p(0, 0), p(4, 0), p(4, 3), p(0, 3)], vec![]);
    let encoded = serde_json::to_string(&input).unwrap();
    let decoded: hypertri::PolygonInput = serde_json::from_str(&encoded).unwrap();

    assert_eq!(decoded.vertices(), input.vertices());
    assert_eq!(decoded.hole_indices(), input.hole_indices());
    assert_eq!(decoded.facts(), input.facts());

    let triangulation =
        hypertri::cdt::constrained_delaunay(decoded.vertices(), &[Constraint::new(1, 3)]).unwrap();
    let encoded = serde_json::to_string(&triangulation).unwrap();
    let decoded: hypertri::cdt::ConstrainedDelaunayTriangulation =
        serde_json::from_str(&encoded).unwrap();

    decoded.validate().unwrap();
    assert_eq!(decoded.constraints(), triangulation.constraints());
    assert_eq!(decoded.constraint_edges(), triangulation.constraint_edges());
    assert_eq!(decoded.triangles(), triangulation.triangles());
}

#[test]
#[cfg(feature = "earcut")]
fn exact_earcut_hole_bridge_uses_exact_visibility() {
    let vertices = vec![
        p(0, 0),
        p(6, 0),
        p(6, 4),
        p(0, 4),
        q(2, 1, 1, 1),
        q(4, 1, 1, 1),
        q(4, 1, 3, 1),
        q(2, 1, 3, 1),
    ];

    let triangles = hypertri::earcut(&vertices, &[4]).unwrap();

    assert_eq!(triangles.len(), 24);
    assert!(triangles.iter().all(|&index| index < vertices.len()));
}

#[cfg(all(feature = "runtime-select", feature = "earcut"))]
#[test]
fn runtime_auto_uses_compiled_boundary_preserving_path() {
    let input = hypertri::PolygonInput::new(vec![p(0, 0), p(1, 0), p(1, 1), p(0, 1)], vec![]);

    let plan =
        hypertri::plan_polygon_triangulation(&input, hypertri::TriangulationOptions::default())
            .unwrap();
    assert_eq!(
        plan.algorithm(),
        hypertri::PolygonTriangulationAlgorithm::Earcut
    );
    assert_eq!(plan.facts(), input.facts());
    assert!(plan.facts().all_coordinates_exact_rational());

    let triangles =
        hypertri::triangulate_polygon(&input, hypertri::TriangulationOptions::default()).unwrap();

    assert_eq!(triangles.len(), 6);
}

#[cfg(all(feature = "runtime-select", feature = "cdt", feature = "earcut"))]
#[test]
fn runtime_can_choose_cdt_polygon_path_explicitly() {
    let input = hypertri::PolygonInput::new(vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)], vec![]);
    let options = hypertri::TriangulationOptions {
        algorithm: hypertri::PolygonTriangulationAlgorithm::ConstrainedDelaunay,
        quality: hypertri::QualityPolicy::PreferDelaunay,
    };

    let plan = hypertri::plan_polygon_triangulation(&input, options).unwrap();
    assert_eq!(
        plan.algorithm(),
        hypertri::PolygonTriangulationAlgorithm::ConstrainedDelaunay
    );
    assert_eq!(plan.quality(), hypertri::QualityPolicy::PreferDelaunay);

    let triangles = hypertri::triangulate_polygon(&input, options).unwrap();

    assert_eq!(triangles.len(), 6);
}

#[cfg(all(feature = "runtime-select", feature = "cdt", feature = "earcut"))]
#[test]
fn runtime_auto_uses_polygon_facts_to_avoid_cdt_on_degenerate_ring_edges() {
    let input =
        hypertri::PolygonInput::new(vec![p(0, 0), p(2, 0), p(2, 0), p(2, 2), p(0, 2)], vec![]);
    let options = hypertri::TriangulationOptions {
        algorithm: hypertri::PolygonTriangulationAlgorithm::Auto,
        quality: hypertri::QualityPolicy::PreferDelaunay,
    };

    let plan = hypertri::plan_polygon_triangulation(&input, options).unwrap();

    assert_eq!(input.facts().known_degenerate_edge_count(), 1);
    assert_eq!(
        plan.algorithm(),
        hypertri::PolygonTriangulationAlgorithm::Earcut
    );
    assert_eq!(plan.facts(), input.facts());
}

#[cfg(all(feature = "runtime-select", feature = "cdt", feature = "earcut"))]
#[test]
fn runtime_cdt_polygon_path_supports_holes() {
    let input = hypertri::PolygonInput::new(
        vec![
            p(0, 0),
            p(6, 0),
            p(6, 6),
            p(0, 6),
            p(2, 2),
            p(4, 2),
            p(4, 4),
            p(2, 4),
        ],
        vec![4],
    );
    let options = hypertri::TriangulationOptions {
        algorithm: hypertri::PolygonTriangulationAlgorithm::ConstrainedDelaunay,
        quality: hypertri::QualityPolicy::PreferDelaunay,
    };

    let triangles = hypertri::triangulate_polygon(&input, options).unwrap();

    assert_eq!(triangles.len(), 24);
    assert!(
        triangles
            .iter()
            .all(|&index| index < input.vertices().len())
    );
}

#[cfg(feature = "earcut")]
proptest! {
    #![proptest_config(ProptestConfig {
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn exact_earcut_rectangles_keep_valid_index_topology(
        x in -1_000i32..1_000,
        y in -1_000i32..1_000,
        width in 1i32..1_000,
        height in 1i32..1_000,
    ) {
        let vertices = vec![
            p(x, y),
            p(x + width, y),
            p(x + width, y + height),
            p(x, y + height),
        ];

        let triangles = hypertri::earcut(&vertices, &[]).unwrap();
        let report = hypertri::earcut_report(&vertices, &[]).unwrap();

        prop_assert_eq!(&report.triangles, &triangles);
        prop_assert_eq!(triangles.len(), 6);
        prop_assert!(
            report.diagnostics.containment_convex_rejects
                + report.diagnostics.containment_bbox_rejects
                + report.diagnostics.containment_tests
                <= report.diagnostics.containment_candidates,
            "containment diagnostic stages must account for no more than the candidates scanned"
        );
        prop_assert_eq!(
            report.diagnostics.containment_prepared_reflex_lookups,
            report.diagnostics.containment_candidates,
            "every scanned containment candidate should use the prepared reflex/convex table"
        );
        for triangle in triangles.chunks_exact(3) {
            prop_assert!(triangle.iter().all(|&index| index < vertices.len()));
            prop_assert_ne!(triangle[0], triangle[1]);
            prop_assert_ne!(triangle[1], triangle[2]);
            prop_assert_ne!(triangle[2], triangle[0]);
        }
    }

    #[test]
    fn exact_earcut_rectangular_holes_keep_valid_index_topology(
        width in 5i32..100,
        height in 5i32..100,
        hole_x in 1i32..3,
        hole_y in 1i32..3,
    ) {
        let vertices = vec![
            p(0, 0),
            p(width, 0),
            p(width, height),
            p(0, height),
            p(hole_x, hole_y),
            p(width - 1, hole_y),
            p(width - 1, height - 1),
            p(hole_x, height - 1),
        ];

        let triangles = hypertri::earcut(&vertices, &[4]).unwrap();
        let report = hypertri::earcut_report(&vertices, &[4]).unwrap();

        prop_assert_eq!(&report.triangles, &triangles);
        prop_assert_eq!(triangles.len(), 24);
        prop_assert!(
            report.diagnostics.containment_convex_rejects
                + report.diagnostics.containment_bbox_rejects
                + report.diagnostics.containment_tests
                <= report.diagnostics.containment_candidates,
            "containment diagnostic stages must account for no more than the candidates scanned"
        );
        prop_assert_eq!(
            report.diagnostics.containment_prepared_reflex_lookups,
            report.diagnostics.containment_candidates,
            "every scanned containment candidate should use the prepared reflex/convex table"
        );
        for triangle in triangles.chunks_exact(3) {
            prop_assert!(triangle.iter().all(|&index| index < vertices.len()));
            prop_assert_ne!(triangle[0], triangle[1]);
            prop_assert_ne!(triangle[1], triangle[2]);
            prop_assert_ne!(triangle[2], triangle[0]);
        }
    }
}
