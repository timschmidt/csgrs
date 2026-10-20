#![cfg(feature = "curve")]

#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    GeometryContext, Real,
    adapter::{AdapterError, F32, F64, I128, RawReal, ScalarAdapter, ScalarCurve},
    curve,
    errors::{ValidationError, ValidationErrorKind},
};
use hypercurve::{BooleanOp, CurveRegion2, RegionPointLocation};
use std::error::Error;

const OPERATIONS: [BooleanOp; 4] = [
    BooleanOp::Union,
    BooleanOp::Difference,
    BooleanOp::Intersection,
    BooleanOp::Xor,
];

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

fn boolean<A: ScalarAdapter>(
    a: &ScalarCurve<A>,
    b: &ScalarCurve<A>,
    operation: BooleanOp,
) -> Result<ScalarCurve<A>, AdapterError> {
    match operation {
        BooleanOp::Union => a.union(b),
        BooleanOp::Difference => a.difference(b),
        BooleanOp::Intersection => a.intersection(b),
        BooleanOp::Xor => a.xor(b),
    }
}

fn assert_cause(error: AdapterError, native: ValidationError, kind: ValidationErrorKind) {
    assert_eq!(native.kind(), kind);
    assert_eq!(error.to_string(), native.to_string());
    assert_eq!(error, AdapterError::Construction(native.clone()));
    let cause = error.source().expect("native cause survives adapter");
    assert_eq!(cause.downcast_ref::<ValidationError>(), Some(&native));
    assert_eq!(
        cause.source().map(ToString::to_string),
        native.source().map(ToString::to_string)
    );
}

fn boolean_failures<A: ScalarAdapter>() {
    let square = curve::square(Real::from(2)).unwrap();
    let shifted = curve::translated(&square, unresolved_zero(), Real::zero()).unwrap();
    let rotated =
        curve::try_rotated(&curve::try_rotated(&square, 13.into()).unwrap(), (-13).into())
            .unwrap();
    let a = ScalarCurve::<A>::from_native(square);
    for region in [shifted, rotated] {
        let b = ScalarCurve::<A>::from_native(region);
        for operation in OPERATIONS {
            let native = a.native().boolean_region(b.native(), operation).unwrap_err();
            let error = boolean(&a, &b, operation)
                .err()
                .expect("undecided is an error");
            assert_cause(error, native.into(), ValidationErrorKind::Undecided);
        }
    }
}

#[test]
fn curve_booleans_preserve_native_blockers_in_every_scalar_family() {
    boolean_failures::<RawReal>();
    boolean_failures::<F32>();
    boolean_failures::<F64>();
    boolean_failures::<I128>();
}

fn boundary_failures<A: ScalarAdapter>() {
    let square = curve::square(Real::from(2)).unwrap();
    let raw = ScalarCurve::<A>::from_native(square.clone());
    let big = curve::square(Real::try_from(f64::MAX).unwrap() * Real::from(2)).unwrap();
    let large = ScalarCurve::<A>::from_native(big.clone());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let angle = A::from_real(&Real::from(360)).unwrap();
        let native = curve::revolve(&square, 360.into(), 1, &context).unwrap_err();
        let error = raw
            .revolve(angle, 1, &context)
            .err()
            .expect("invalid resolution fails");
        assert_cause(error, native, ValidationErrorKind::InvalidInput);
        let native = curve::try_finite_profiles(&big, &context).unwrap_err();
        let error = large.region_profiles(&context).unwrap_err();
        assert_cause(error, native, ValidationErrorKind::InvalidInput);
    }
}

#[test]
fn revolve_and_profile_failures_retain_native_causes() {
    boundary_failures::<RawReal>();
    boundary_failures::<F32>();
    boundary_failures::<F64>();
    boundary_failures::<I128>();
}

#[test]
fn unresolved_revolve_angle_is_not_invalid_input_or_empty_geometry() {
    let square = curve::square(2.into()).unwrap();
    let raw = ScalarCurve::<RawReal>::from_native(square.clone());
    let native =
        curve::revolve(&square, unresolved_zero(), 4, &GeometryContext::STRICT).unwrap_err();
    let error = raw
        .revolve(unresolved_zero(), 4, &GeometryContext::STRICT)
        .unwrap_err();
    assert_cause(error, native, ValidationErrorKind::Undecided);
}

// Axis-aligned integer rectangles partition the plane into unit cells. The
// Boolean truth table at each cell center and exact area are independent of
// contour ordering, triangulation, and the adapter implementation.
fn assert_set(region: &CurveRegion2, operation: BooleanOp, area: i32) {
    assert_eq!(region.filled_area().unwrap(), Some(Real::from(area)));
    for x in -1..4 {
        for y in -1..4 {
            let a = (0..2).contains(&x) && (0..2).contains(&y);
            let b = (1..3).contains(&x) && (1..3).contains(&y);
            let inside = match operation {
                BooleanOp::Union => a || b,
                BooleanOp::Difference => a && !b,
                BooleanOp::Intersection => a && b,
                BooleanOp::Xor => a != b,
            };
            let point = hypercurve::Point2::new(
                (Real::from(2 * x + 1) / Real::from(2)).unwrap(),
                (Real::from(2 * y + 1) / Real::from(2)).unwrap(),
            );
            assert_eq!(
                region.classify_point(&point.into()).unwrap(),
                if inside {
                    RegionPointLocation::Inside
                } else {
                    RegionPointLocation::Outside
                }
            );
        }
    }
}

fn composition<A: ScalarAdapter>() {
    let a = ScalarCurve::<A>::from_native(curve::square(2.into()).unwrap());
    let b = ScalarCurve::<A>::from_native(
        curve::translated(a.native(), 1.into(), 1.into()).unwrap(),
    );
    let empty = ScalarCurve::<A>::empty();
    for (operation, area) in OPERATIONS.into_iter().zip([7, 3, 1, 6]) {
        let result = boolean(&a, &b, operation).unwrap();
        assert_set(result.native(), operation, area);
        let continued = result.union(&empty).unwrap();
        assert_set(continued.native(), operation, area);
        assert!(continued.difference(&result).unwrap().native().is_empty());
    }
    let accumulated = empty.union(&a).unwrap().union(&b).unwrap();
    assert_set(accumulated.native(), BooleanOp::Union, 7);
    let removed = accumulated.difference(&a).unwrap().difference(&b).unwrap();
    assert!(removed.native().is_empty());
    assert!(a.intersection(&removed).unwrap().native().is_empty());
    assert!(removed.difference(&a).unwrap().native().is_empty());
    assert_eq!(
        removed.union(&a).unwrap().native().filled_area().unwrap(),
        Some(4.into())
    );
}

#[test]
fn successful_curve_booleans_and_empty_results_remain_composable() {
    composition::<RawReal>();
    composition::<F32>();
    composition::<F64>();
    composition::<I128>();
}
