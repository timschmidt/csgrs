use super::{polygons_to_profile, rect_polygon};
use crate::scalar::{scalar, try_compare};
use crate::{PREDICATE_POLICY, Scalar};
use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
use std::cmp::Ordering::Equal;

fn same(left: &Scalar, right: &Scalar) {
    assert_eq!(try_compare(left, right), Ok(Equal));
}

#[test]
fn retained_large_rectangle_has_exact_corners_and_area() {
    // Original failing source from the dense-cluster campaign; keep its scale.
    let polygon = rect_polygon([1e20, 0.0], [16_384.0, 16_384.0], 0.0);
    let center = scalar("1e20");
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(268_435_456));
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([
            &center - Scalar::from(8192),
            Scalar::from(-8192),
            &center + Scalar::from(8192),
            Scalar::from(8192),
        ])
    );
    // The historical report ring still collapses; it cannot decide geometry.
    assert!(polygon.exterior().0.iter().all(|point| point.x == 1e20));
    let region = polygons_to_profile(vec![polygon], None);
    for (offset, expected) in [
        (0, Inside),
        (8191, Inside),
        (8192, Boundary),
        (8193, Outside),
    ] {
        assert_eq!(
            region.classify_xy(&center + Scalar::from(offset), Scalar::zero()),
            Ok(expected)
        );
    }
}

#[test]
fn tiny_dimensions_do_not_underflow_before_import() {
    let width = f64::from_bits(1);
    let polygon = rect_polygon([0.0, 0.0], [width, width], 0.0);
    let side = Scalar::try_from(width).unwrap();
    let half = crate::scalar::half(&side);
    assert_eq!(polygon.try_area().unwrap(), &side * &side);
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([-half.clone(), -half.clone(), half.clone(), half])
    );
}

#[test]
fn finite_inputs_can_have_native_corners_beyond_float_range() {
    let polygon = rect_polygon([f64::MAX, 0.0], [f64::MAX, 2.0], 0.0);
    let center = Scalar::try_from(f64::MAX).unwrap();
    let half = crate::scalar::half(&center);
    assert_eq!(polygon.try_area().unwrap(), &center * Scalar::from(2));
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([&center - &half, (-1).into(), &center + half, 1.into()])
    );
}

#[test]
fn quarter_turns_do_not_leave_float_trigonometric_slivers() {
    for angle in [90.0, -90.0, 270.0, 450.0] {
        let polygon = rect_polygon([1.0, 2.0], [4.0, 2.0], angle);
        assert_eq!(polygon.try_area().unwrap(), Scalar::from(8));
        assert_eq!(
            polygon.try_bounds().unwrap(),
            Some([0.into(), 0.into(), 2.into(), 4.into()])
        );
        let region = polygons_to_profile(vec![polygon], None);
        assert_eq!(
            region.classify_xy(Scalar::zero(), Scalar::from(2)),
            Ok(Boundary)
        );
        assert_eq!(
            region.classify_xy(-scalar("1e-30"), Scalar::from(2)),
            Ok(Outside)
        );
    }
}

#[test]
fn exact_eighth_turn_retains_surd_corners_and_area() {
    let polygon = rect_polygon([0.0, 0.0], [2.0, 2.0], 45.0);
    let root = Scalar::from(2).sqrt().unwrap();
    same(&polygon.try_area().unwrap(), &Scalar::from(4));
    let bounds = polygon.try_bounds().unwrap().unwrap();
    for (value, expected) in bounds
        .iter()
        .zip([-root.clone(), -root.clone(), root.clone(), root])
    {
        same(value, &expected);
    }
}

#[test]
fn non_dyadic_rotation_preserves_the_authored_area() {
    let polygon = rect_polygon([3.0, -4.0], [2.0, 6.0], 30.0);
    same(&polygon.try_area().unwrap(), &Scalar::from(12));
}

#[test]
fn zero_rectangle_dimensions_are_certified_empty() {
    for size in [[0.0, 2.0], [2.0, -0.0], [0.0, 0.0]] {
        let polygon = rect_polygon([1e20, 0.0], size, 35.0);
        assert!(polygon.try_exact_region().unwrap().is_empty());
        assert_eq!(polygon.try_area().unwrap(), Scalar::zero());
        assert_eq!(polygon.try_bounds().unwrap(), None);
    }
}

#[test]
fn signed_dimensions_preserve_exact_regularized_rectangle() {
    for size in [[-4.0, 2.0], [4.0, -2.0], [-4.0, -2.0]] {
        let polygon = rect_polygon([1e20, 0.0], size, 90.0);
        assert_eq!(polygon.try_area().unwrap(), Scalar::from(8));
        let x = scalar("1e20");
        assert_eq!(
            polygon.try_bounds().unwrap(),
            Some([
                &x - Scalar::one(),
                (-2).into(),
                &x + Scalar::one(),
                2.into()
            ])
        );
    }
}

#[test]
fn ordinary_axis_aligned_control_stays_exact() {
    let polygon = rect_polygon([1.0, -2.0], [4.0, 6.0], 0.0);
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(24));
    assert_eq!(
        polygon.try_bounds().unwrap(),
        Some([(-1).into(), (-5).into(), 3.into(), 1.into()])
    );
}

#[test]
fn non_finite_inputs_remain_typed_failures() {
    for (center, size, angle) in [
        ([f64::NAN, 0.0], [2.0, 2.0], 0.0),
        ([0.0, 0.0], [f64::INFINITY, 2.0], 0.0),
        ([0.0, 0.0], [0.0, 2.0], f64::INFINITY),
    ] {
        let polygon = rect_polygon(center, size, angle);
        assert!(polygon.try_exact_region().is_err());
        assert!(polygon.try_area().is_err());
        assert!(polygon.try_bounds().is_err());
    }
}

fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Scalar::from)
            .collect(),
    )
    .unwrap();
    assert!(matches!(polynomial.isolate_unit_interval_roots(),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
        if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted));
}

#[test]
fn denied_rectangle_construction_cannot_escape_and_can_retry() {
    let mut escaped = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            escaped = Some(rect_polygon([1.0, 2.0], [4.0, 2.0], 0.0));
        });
    assert!(denied.result.is_err());
    assert!(escaped.unwrap().try_exact_region().is_err());
    assert_eq!(
        rect_polygon([1.0, 2.0], [4.0, 2.0], 0.0)
            .try_area()
            .unwrap(),
        Scalar::from(8)
    );
}

#[test]
fn fresh_zero_budget_keeps_exact_rectangle_fast_paths() {
    let result =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            rect_polygon([1e20, 0.0], [4.0, 2.0], 90.0)
        });
    assert_eq!(result.usage, hypercurve::WorkUsage::default());
    let polygon = result.result.unwrap().certified().unwrap();
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(8));
}

#[test]
fn one_unit_rectangle_survives_a_collapsed_report_view() {
    let polygon = rect_polygon([1e20, -1e20], [1.0, 1.0], 0.0);
    let expected = [
        scalar("1e20") - scalar("0.5"),
        -scalar("1e20") - scalar("0.5"),
        scalar("1e20") + scalar("0.5"),
        -scalar("1e20") + scalar("0.5"),
    ];
    for _ in 0..2 {
        assert_eq!(polygon.try_bounds().unwrap(), Some(expected.clone()));
        assert_eq!(polygon.try_area().unwrap(), Scalar::one());
    }
    assert!(
        polygon
            .exterior()
            .0
            .windows(2)
            .all(|pair| pair[0] == pair[1])
    );
}

#[test]
fn finite_source_dyadics_are_not_rounded_after_corner_arithmetic() {
    let polygon = rect_polygon([0.95, 0.0], [0.1, 0.1], 0.0);
    let x = Scalar::try_from(0.95).unwrap();
    let width = Scalar::try_from(0.1).unwrap();
    let half = crate::scalar::half(&width);
    let right = &x + &half;
    assert!(try_compare(&right, &Scalar::one()).unwrap().is_lt());
    assert_eq!(polygon.try_area().unwrap(), &width * &width);
    let region = polygons_to_profile(vec![polygon], None);
    assert_eq!(region.classify_xy(right, Scalar::zero()), Ok(Boundary));
    assert_eq!(
        region.classify_xy(Scalar::one(), Scalar::zero()),
        Ok(Outside)
    );
}

#[test]
fn retained_large_rectangles_share_the_exact_closed_edge() {
    let left = polygons_to_profile(
        vec![rect_polygon([1e20, 0.0], [16_384.0, 16_384.0], 0.0)],
        None,
    );
    let right = polygons_to_profile(
        vec![rect_polygon(
            [1e20_f64.next_up(), 0.0],
            [16_384.0, 16_384.0],
            0.0,
        )],
        None,
    );
    let edge = scalar("1e20") + Scalar::from(8192);
    assert_eq!(left.classify_xy(edge.clone(), Scalar::zero()), Ok(Boundary));
    assert_eq!(right.classify_xy(edge, Scalar::zero()), Ok(Boundary));
    assert!(
        left.try_intersection(&right)
            .unwrap()
            .try_is_empty()
            .unwrap()
    );
    assert_eq!(
        left.try_union(&right).unwrap().try_area().unwrap(),
        Scalar::from(536_870_912)
    );
}

#[test]
fn general_degree_rotation_keeps_native_center_membership() {
    for angle in [33.0, 393.0, -327.0, 1e20] {
        let polygon = rect_polygon([3.0, -4.0], [2.0, 6.0], angle);
        assert!(
            try_compare(&polygon.try_area().unwrap(), &Scalar::zero())
                .unwrap()
                .is_gt()
        );
        assert!(polygon.try_bounds().unwrap().is_some());
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
fn empty_rectangle_cannot_escape_an_exhausted_construction_scope() {
    let mut escaped = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            escaped = Some(rect_polygon([1e20, 0.0], [0.0, 2.0], 35.0));
        });
    assert!(denied.result.is_err());
    assert!(escaped.unwrap().try_exact_region().is_err());
    assert!(
        rect_polygon([1e20, 0.0], [0.0, 2.0], 35.0)
            .try_exact_region()
            .unwrap()
            .is_empty()
    );
}

#[test]
fn warm_rectangle_measurements_obey_exhaustion_and_retry() {
    let polygon = rect_polygon([1e20, 0.0], [1.0, 2.0], 90.0);
    let bounds = polygon.try_bounds().unwrap();
    assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
    let cloned = polygon.clone();
    let result =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            assert!(cloned.try_area().is_err());
            assert!(cloned.try_bounds().is_err());
            assert!(cloned.try_exact_region().is_err());
        });
    assert!(result.result.is_err());
    assert_eq!(cloned.try_area().unwrap(), Scalar::from(2));
    assert_eq!(cloned.try_bounds().unwrap(), bounds);
}

#[test]
fn rectangle_fallbacks_keep_the_native_source() {
    for polygon in [
        super::rounded_rect_polygon([1e20, 0.0], [1.0, 2.0], 0.0, 90.0, 8),
        super::chamfered_rect_polygon([1e20, 0.0], [1.0, 2.0], 0.1, [false; 4], 90.0),
    ] {
        assert_eq!(polygon.try_area().unwrap(), Scalar::from(2));
        assert_eq!(
            polygon.try_bounds().unwrap(),
            rect_polygon([1e20, 0.0], [1.0, 2.0], 90.0)
                .try_bounds()
                .unwrap()
        );
    }
}

#[test]
fn enclosing_preview_does_not_supply_rectangle_decisions() {
    let preview = hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(|| {
            hypercurve::provisional(rect_polygon([1e20, 0.0], [16_384.0, 16_384.0], 45.0))
        });
    assert!(preview.is_certified());
    let polygon = preview.certified().unwrap();
    same(&polygon.try_area().unwrap(), &Scalar::from(268_435_456));
}
