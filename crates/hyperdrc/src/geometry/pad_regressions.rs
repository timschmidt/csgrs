use super::{chamfered_rect_polygon, polygons_to_profile, rounded_rect_polygon, trapezoid_polygon};
use crate::scalar::{scalar, try_compare};
use crate::{PREDICATE_POLICY, Scalar};
use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};

fn same(left: &Scalar, right: &Scalar) {
    assert_eq!(try_compare(left, right), Ok(std::cmp::Ordering::Equal));
}

fn rounded_area(width: Scalar, height: Scalar, radius: Scalar) -> Scalar {
    width * height - (Scalar::from(4) - Scalar::pi()) * &radius * radius
}

#[test]
fn rounded_rectangle_area_is_independent_of_report_samples() {
    for samples in [4, 5, 8, 17, 128] {
        let polygon = rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, 0.0, samples);
        same(
            &polygon.try_area().unwrap(),
            &rounded_area(2.into(), 1.into(), scalar("0.25")),
        );
    }
}

#[test]
fn rounded_rectangle_classifies_the_native_arc_not_its_chords() {
    for samples in [4, 8, 17] {
        let region = polygons_to_profile(
            vec![rounded_rect_polygon(
                [0.0, 0.0],
                [2.0, 1.0],
                0.25,
                0.0,
                samples,
            )],
            None,
        );
        // Corner center (3/4,1/4), radius 1/4, rational 3-4-5 radius direction.
        assert_eq!(
            region.classify_xy(scalar("0.9"), scalar("0.45")),
            Ok(Boundary)
        );
        assert_eq!(
            region.classify_xy(scalar("0.9") - scalar("1e-30"), scalar("0.45")),
            Ok(Inside)
        );
        assert_eq!(
            region.classify_xy(scalar("0.9") + scalar("1e-30"), scalar("0.45")),
            Ok(Outside)
        );
    }
}

#[test]
fn rounded_rectangle_keeps_the_original_large_coordinate_scale() {
    let polygon = rounded_rect_polygon([1e20, 0.0], [16_384.0, 16_384.0], 4096.0, 0.0, 8);
    same(
        &polygon.try_area().unwrap(),
        &rounded_area(16384.into(), 16384.into(), 4096.into()),
    );
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([
            scalar("1e20") - Scalar::from(8192),
            (-8192).into(),
            scalar("1e20") + Scalar::from(8192),
            8192.into()
        ])
    );
}

#[test]
fn rounded_rectangle_keeps_subnormal_radius_and_dimensions() {
    let tiny = f64::from_bits(1);
    let radius = Scalar::try_from(tiny).unwrap();
    let polygon = rounded_rect_polygon([0.0, 0.0], [2.0 * tiny, 4.0 * tiny], tiny, 0.0, 8);
    same(
        &polygon.try_area().unwrap(),
        &rounded_area(
            &radius * Scalar::from(2),
            &radius * Scalar::from(4),
            radius.clone(),
        ),
    );
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([
            -radius.clone(),
            -(&radius * Scalar::from(2)),
            radius.clone(),
            &radius * Scalar::from(2)
        ])
    );
}

#[test]
fn rounded_rectangle_allows_unrepresentable_report_corners() {
    let width = Scalar::try_from(f64::MAX).unwrap();
    let radius = Scalar::try_from(f64::MAX / 4.0).unwrap();
    let polygon = rounded_rect_polygon(
        [f64::MAX, 0.0],
        [f64::MAX, f64::MAX],
        f64::MAX / 4.0,
        0.0,
        4,
    );
    same(
        &polygon.try_area().unwrap(),
        &rounded_area(width.clone(), width, radius),
    );
}

#[test]
fn rounded_maximum_radius_is_exact_capsule_or_circle() {
    let capsule = rounded_rect_polygon([0.0, 0.0], [4.0, 2.0], 99.0, 0.0, 8);
    same(
        &capsule.try_area().unwrap(),
        &(Scalar::from(4) + Scalar::pi()),
    );
    let circle = rounded_rect_polygon([0.0, 0.0], [2.0, 2.0], 99.0, 45.0, 8);
    same(&circle.try_area().unwrap(), &Scalar::pi());
}

#[test]
fn rounded_quarter_turn_keeps_exact_bounds() {
    let polygon = rounded_rect_polygon([1.0, 2.0], [4.0, 2.0], 0.5, 90.0, 5);
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([0.into(), 0.into(), 2.into(), 4.into()])
    );
    same(
        &polygon.try_area().unwrap(),
        &rounded_area(4.into(), 2.into(), scalar("0.5")),
    );
}

#[test]
fn rounded_negative_radius_retains_rectangle_control() {
    let polygon = rounded_rect_polygon([3.0, 4.0], [-2.0, 1.0], -1.0, 0.0, 8);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
}

#[test]
fn every_chamfer_selection_has_exact_removed_triangle_area() {
    let chamfer = Scalar::try_from(0.2).unwrap();
    for bits in 0_u8..16 {
        let corners = std::array::from_fn(|i| bits & (1 << i) != 0);
        let polygon = chamfered_rect_polygon([0.95, -0.7], [2.0, 1.0], 0.2, corners, 0.0);
        let removed =
            (&chamfer * &chamfer * Scalar::from(bits.count_ones()) / Scalar::from(2)).unwrap();
        assert_eq!(polygon.try_area().unwrap(), Scalar::from(2) - removed);
    }
}

#[test]
fn chamfer_keeps_the_original_large_coordinate_scale() {
    let polygon = chamfered_rect_polygon([1e20, 0.0], [16_384.0, 16_384.0], 4096.0, [true; 4], 0.0);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(234_881_024));
    let region = polygons_to_profile(vec![polygon], None);
    assert_eq!(
        region.classify_xy(scalar("1e20") + Scalar::from(6144), Scalar::from(6144)),
        Ok(Boundary)
    );
}

#[test]
fn chamfer_keeps_subnormal_dimensions_before_clamping() {
    let tiny = f64::from_bits(1);
    let side = Scalar::try_from(tiny).unwrap();
    let polygon = chamfered_rect_polygon([0.0, 0.0], [tiny, tiny], tiny, [true; 4], 0.0);
    assert_eq!(
        polygon.try_area().unwrap(),
        (&side * &side / Scalar::from(2)).unwrap()
    );
}

#[test]
fn maximum_chamfer_removes_only_repeated_vertices() {
    let polygon = chamfered_rect_polygon([0.0, 0.0], [2.0, 2.0], 99.0, [true; 4], 0.0);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
}

#[test]
fn trapezoid_uses_kicad_half_delta_and_corner_signs() {
    // KiCad plot_board_layers.cpp 526-534: half-deltas, opposite X edge widths.
    let polygon = trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [0.5, 0.25], 0.0);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
    let region = polygons_to_profile(vec![polygon], None);
    for (x, y) in [
        ("-1.125", "0.75"),
        ("1.125", "0.25"),
        ("0.875", "-0.25"),
        ("-0.875", "-0.75"),
    ] {
        assert_eq!(region.classify_xy(scalar(x), scalar(y)), Ok(Boundary));
    }
}

#[test]
fn trapezoid_keeps_the_original_large_coordinate_scale() {
    let polygon = trapezoid_polygon([1e20, 0.0], [16_384.0, 16_384.0], [2048.0, 4096.0], 0.0);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(268_435_456));
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([
            scalar("1e20") - Scalar::from(10240),
            (-9216).into(),
            scalar("1e20") + Scalar::from(10240),
            9216.into()
        ])
    );
}

#[test]
fn trapezoid_zero_delta_preserves_minimum_subnormal_sizes() {
    let tiny = f64::from_bits(1);
    let side = Scalar::try_from(tiny).unwrap();
    let polygon = trapezoid_polygon([0.0, 0.0], [tiny, tiny], [0.0, 0.0], 90.0);
    assert_eq!(polygon.try_area().unwrap(), &side * &side);
}

#[test]
fn trapezoid_rejects_delta_beyond_the_opposite_dimension() {
    for delta in [[1.5, 0.0], [-1.5, 0.0], [0.0, 2.5], [0.0, -2.5]] {
        let polygon = trapezoid_polygon([0.0, 0.0], [2.0, 1.0], delta, 0.0);
        assert!(polygon.try_area().is_err());
    }
}

#[test]
fn trapezoid_at_maximum_delta_is_a_valid_triangle() {
    let polygon = trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [1.0, 0.0], 0.0);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
    let region = polygons_to_profile(vec![polygon], None);
    assert_eq!(
        region.classify_xy(Scalar::one(), Scalar::zero()),
        Ok(Boundary)
    );
}

#[test]
fn empty_pad_primitives_are_certified_empty() {
    for size in [[0.0, 1.0], [2.0, 0.0], [0.0, 0.0]] {
        for polygon in [
            rounded_rect_polygon([1e20, 0.0], size, 1.0, 90.0, 8),
            chamfered_rect_polygon([1e20, 0.0], size, 1.0, [true; 4], 90.0),
            trapezoid_polygon([1e20, 0.0], size, [0.0, 0.0], 90.0),
        ] {
            assert!(polygon.try_exact_region().unwrap().is_empty());
            assert_eq!(polygon.try_bounds().unwrap(), None);
        }
    }
}

#[test]
fn non_finite_pad_primitives_remain_failures() {
    for polygon in [
        rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], f64::NAN, 0.0, 8),
        chamfered_rect_polygon([0.0, 0.0], [2.0, 1.0], f64::INFINITY, [true; 4], 0.0),
        trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [f64::NAN, 0.0], 0.0),
    ] {
        assert!(polygon.try_area().is_err());
    }
}

#[test]
fn denied_pad_construction_remains_failed_after_its_scope() {
    let mut escaped = vec![];
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                [-36, 1368, -11034, 31728, -38280, 16620]
                    .into_iter()
                    .map(Scalar::from)
                    .collect(),
            )
            .unwrap();
            assert!(polynomial.isolate_unit_interval_roots().is_err());
            escaped = vec![
                rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, 0.0, 8),
                chamfered_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, [true; 4], 0.0),
                trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [0.0, 0.0], 0.0),
            ];
        });
    assert!(denied.result.is_err());
    assert!(escaped.iter().all(|p| p.try_area().is_err()));
}

#[test]
fn native_rounded_boundary_structure_ignores_report_resolution() {
    let mut expected = None;
    for samples in [4, 5, 17, 128] {
        let polygon = rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, 0.0, samples);
        let native = polygon
            .try_exact_region()
            .unwrap()
            .native_contours_fast_path()
            .unwrap()
            .unwrap();
        assert_eq!(native.material_contours().len(), 1);
        assert!(native.hole_contours().is_empty());
        let segments = native.material_contours()[0].segments();
        let arcs = segments
            .iter()
            .filter(|s| matches!(s, hypercurve::Segment2::Arc(_)))
            .count();
        let lines = segments
            .iter()
            .filter(|s| matches!(s, hypercurve::Segment2::Line(_)))
            .count();
        assert!(arcs > 0 && lines > 0);
        let counts = (arcs, lines, segments.len());
        assert_eq!(*expected.get_or_insert(counts), counts);
    }
}

#[test]
fn native_rounded_boolean_handoff_ignores_report_chords() {
    let region = |samples| {
        polygons_to_profile(
            vec![rounded_rect_polygon(
                [0.0, 0.0],
                [2.0, 1.0],
                0.25,
                0.0,
                samples,
            )],
            None,
        )
    };
    let coarse = region(4);
    let fine = region(17);
    assert!(coarse.try_xor(&fine).unwrap().try_is_empty().unwrap());
    let overlap = coarse.try_intersection(&fine).unwrap();
    same(
        &overlap.try_area().unwrap(),
        &rounded_area(2.into(), 1.into(), scalar("0.25")),
    );
    assert_eq!(
        overlap.classify_xy(scalar("0.9"), scalar("0.45")),
        Ok(Boundary)
    );
}

fn ordinary_primitives() -> [super::Polygon; 3] {
    [
        rounded_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, 0.0, 8),
        chamfered_rect_polygon([0.0, 0.0], [2.0, 1.0], 0.25, [true; 4], 0.0),
        trapezoid_polygon([0.0, 0.0], [2.0, 1.0], [0.5, 0.25], 0.0),
    ]
}

fn exhaust() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Scalar::from)
            .collect(),
    )
    .unwrap();
    assert!(polynomial.isolate_unit_interval_roots().is_err());
}

#[test]
fn warm_pad_measurements_obey_budget_and_retry_from_native_sources() {
    for polygon in ordinary_primitives() {
        let area = polygon.try_area().unwrap();
        let bounds = polygon.try_bounds().unwrap();
        let clone = polygon.clone();
        let denied = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                exhaust();
                assert!(clone.try_area().is_err());
                assert!(clone.try_bounds().is_err());
                assert!(clone.try_exact_region().is_err());
            },
        );
        assert!(denied.result.is_err());
        same(&clone.try_area().unwrap(), &area);
        assert_eq!(clone.try_bounds().unwrap(), bounds);
    }
}

#[test]
fn enclosing_preview_does_not_decide_native_pad_geometry() {
    let preview = hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(|| hypercurve::provisional(ordinary_primitives));
    assert!(preview.is_certified());
    let polygons = preview.certified().unwrap();
    same(
        &polygons[0].try_area().unwrap(),
        &rounded_area(2.into(), 1.into(), scalar("0.25")),
    );
    assert_eq!(polygons[1].try_area().unwrap(), scalar("1.875"));
    assert_eq!(polygons[2].try_area().unwrap(), Scalar::from(2));
}

#[test]
fn signed_dimensions_and_thirty_degree_rotation_keep_native_area() {
    for size in [[-2.0, 1.0], [2.0, -1.0], [-2.0, -1.0]] {
        let rounded = rounded_rect_polygon([3.0, -4.0], size, 0.25, 30.0, 8);
        same(
            &rounded.try_area().unwrap(),
            &rounded_area(2.into(), 1.into(), scalar("0.25")),
        );
        let chamfer = chamfered_rect_polygon([3.0, -4.0], size, 0.25, [true; 4], 30.0);
        same(&chamfer.try_area().unwrap(), &scalar("1.875"));
        let trapezoid = trapezoid_polygon([3.0, -4.0], size, [0.5, 0.25], 30.0);
        same(&trapezoid.try_area().unwrap(), &Scalar::from(2));
    }
}

#[test]
fn general_degree_rounded_rotations_keep_center_membership() {
    for angle in [33.0, 393.0, -327.0, 1e20] {
        let polygon = rounded_rect_polygon([3.0, -4.0], [2.0, 1.0], 0.25, angle, 8);
        assert!(
            try_compare(&polygon.try_area().unwrap(), &Scalar::zero())
                .unwrap()
                .is_gt()
        );
        let region = polygons_to_profile(vec![polygon], None);
        assert_eq!(
            region.classify_xy(Scalar::from(3), Scalar::from(-4)),
            Ok(Inside)
        );
        assert_eq!(
            region.classify_xy(Scalar::from(100), Scalar::from(100)),
            Ok(Outside)
        );
    }
}

#[test]
fn empty_pad_construction_does_not_escape_exhaustion() {
    let mut escaped = Vec::new();
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            exhaust();
            escaped = vec![
                rounded_rect_polygon([1e20, 0.0], [0.0, 1.0], 0.25, 0.0, 8),
                chamfered_rect_polygon([1e20, 0.0], [0.0, 1.0], 0.25, [true; 4], 0.0),
                trapezoid_polygon([1e20, 0.0], [0.0, 1.0], [0.0, 0.0], 0.0),
            ];
        });
    assert!(denied.result.is_err());
    assert!(escaped.iter().all(|p| p.try_area().is_err()));
}

#[test]
fn trapezoid_supports_every_valid_delta_sign_and_limit() {
    for dx in [-1.0, -0.5, 0.0, 0.5, 1.0] {
        for dy in [-2.0, -0.25, 0.0, 0.25, 2.0] {
            let polygon = trapezoid_polygon([3.0, 4.0], [2.0, 1.0], [dx, dy], 90.0);
            assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
        }
    }
}
