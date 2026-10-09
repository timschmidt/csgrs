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
