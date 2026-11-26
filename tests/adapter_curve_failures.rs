//! Curve-adapter error transport and finite composition witnesses across scalar families.
//! Most fixtures wrap native regions directly, avoiding scalar-coordinate demotion.
//! Independent cell/area expectations cover the specified rectangle fixtures only.

#![cfg(feature = "curve")]

#[path = "support/uncertainty.rs"]
/// Shared proved-zero control and deliberately unresolved sine-addition zero.
mod uncertainty;

use csgrs::{
    GeometryContext, Real,
    adapter::{AdapterError, F32, F64, I128, RawReal, ScalarAdapter, ScalarCurve},
    curve,
    errors::{ValidationError, ValidationErrorKind},
};
use hypercurve::{BooleanOp, CurveRegion2, RegionPointLocation};
use std::error::Error;

/// Fixed native/adapter Boolean order, paired with expected areas in the composition fixture.
const OPERATIONS: [BooleanOp; 4] = [
    BooleanOp::Union,
    BooleanOp::Difference,
    BooleanOp::Intersection,
    BooleanOp::Xor,
];

/// Checks the retained Pythagorean zero, then returns the shared strictly unresolved zero.
fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

/// Dispatches a chosen Boolean through the adapter's ordinary strict geometry-only API.
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

/// Requires the expected category, exact structured modeling error and preserved source text.
/// Downcasts the immediate source; deeper sources are compared by display strings.
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

#[test]
/// Wraps an authored incompatible-policy error and checks its undecided classification/cause.
/// Does not execute a failed geometry operation or an actual larger-policy retry.
fn retained_policy_mismatch_remains_retryable_at_the_adapter_boundary() {
    let native = ValidationError::from(hypercurve::CurveError::IncompatiblePredicatePolicy);
    assert_cause(
        AdapterError::Construction(native.clone()),
        native,
        ValidationErrorKind::Undecided,
    );
}

/// Checks native/adapter agreement on unresolved translated-square failures, then
/// area/empty expectations for a 13-degree rotation followed by its inverse.
/// Successful union/intersection evidence here is exact area, not full set reconstruction.
fn boolean_failures<A: ScalarAdapter>() {
    let square = curve::square(Real::from(2)).unwrap();
    let shifted = curve::translated(&square, unresolved_zero(), Real::zero()).unwrap();
    let a = ScalarCurve::<A>::from_native(square.clone());
    let b = ScalarCurve::<A>::from_native(shifted);
    for operation in OPERATIONS {
        let native = a.native().boolean_region(b.native(), operation).unwrap_err();
        let error = boolean(&a, &b, operation)
            .err()
            .expect("undecided is an error");
        assert_cause(error, native.into(), ValidationErrorKind::Undecided);
    }
    // The inverse rotations mathematically recover the square. Check exact
    // area for union/intersection and emptiness for difference/XOR; this block
    // does not independently inspect every resulting boundary segment.
    let rotated = curve::rotated(&curve::rotated(&square, 13).unwrap(), -13).unwrap();
    let r = ScalarCurve::<A>::from_native(rotated);
    let square_area = Some(Real::from(4));
    for operation in OPERATIONS {
        let native = a.native().boolean_region(r.native(), operation).unwrap();
        let result = boolean(&a, &r, operation).unwrap();
        let area = result.native().filled_area().unwrap();
        assert_eq!(area, native.filled_area().unwrap());
        match operation {
            BooleanOp::Union | BooleanOp::Intersection => assert_eq!(area, square_area),
            BooleanOp::Difference | BooleanOp::Xor => assert!(result.native().is_empty()),
        }
    }
}

#[test]
/// Runs the Boolean failure/control matrix through all four scalar adapter families.
fn curve_booleans_preserve_native_blockers_in_every_scalar_family() {
    boolean_failures::<RawReal>();
    boolean_failures::<F32>();
    boolean_failures::<F64>();
    boolean_failures::<I128>();
}

/// Checks invalid revolution resolution and unrepresentable finite-profile export
/// against native errors under both policies, without exporting the wrapped coordinates first.
fn boundary_failures<A: ScalarAdapter>() {
    let square = curve::square(Real::from(2)).unwrap();
    let raw = ScalarCurve::<A>::from_native(square.clone());
    let big = curve::square(Real::try_from(f64::MAX).unwrap() * Real::from(2)).unwrap();
    let large = ScalarCurve::<A>::from_native(big.clone());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let angle = A::from_real(&Real::from(360)).unwrap();
        let native = curve::revolve(&square, 360.into(), 1, context).unwrap_err();
        let error = raw
            .revolve(angle, 1, context)
            .err()
            .expect("invalid resolution fails");
        assert_cause(error, native, ValidationErrorKind::InvalidInput);
        let native = curve::try_finite_profiles(&big, context).unwrap_err();
        let error = large.region_profiles(context).unwrap_err();
        assert_cause(error, native, ValidationErrorKind::InvalidInput);
    }
}

#[test]
/// Requires native modeling causes for revolution/profile errors across all scalar families.
fn revolve_and_profile_failures_retain_native_causes() {
    boundary_failures::<RawReal>();
    boundary_failures::<F32>();
    boundary_failures::<F64>();
    boundary_failures::<I128>();
}

#[test]
/// Requires undecided errors for the unresolved angle through swept-boundary and
/// regularized-solid revolution on the raw-Real adapter.
fn unresolved_revolve_angle_is_not_invalid_input_or_empty_geometry() {
    let square = curve::square(2).unwrap();
    let raw = ScalarCurve::<RawReal>::from_native(square.clone());
    let native =
        curve::revolve(&square, unresolved_zero(), 4, GeometryContext::STRICT).unwrap_err();
    let error = raw
        .revolve(unresolved_zero(), 4, GeometryContext::STRICT)
        .unwrap_err();
    assert_cause(error, native, ValidationErrorKind::Undecided);
    let native = curve::revolve_solid(&square, unresolved_zero(), 4, GeometryContext::STRICT)
        .unwrap_err();
    let error = raw
        .revolve_solid(unresolved_zero(), 4, GeometryContext::STRICT)
        .expect_err("undecided regularized angle");
    assert_cause(error, native, ValidationErrorKind::Undecided);
}

/// Checks exact area and 25 half-grid point locations against two integer squares' Boolean table.
/// Expectations are independent of output contour order and native-wrapper agreement;
/// finite membership samples plus area alone are not a general set-equivalence proof.
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

/// Reuses rectangle Boolean results with empty operands and repeated removals.
/// Checks area/cell expectations before and after continuation, then canonical empty identities.
fn composition<A: ScalarAdapter>() {
    let a = ScalarCurve::<A>::from_native(curve::square(2).unwrap());
    let b = ScalarCurve::<A>::from_native(curve::translated(a.native(), 1, 1).unwrap());
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
/// Runs the finite rectangle/empty composition chain through all scalar wrapper families.
fn successful_curve_booleans_and_empty_results_remain_composable() {
    composition::<RawReal>();
    composition::<F32>();
    composition::<F64>();
    composition::<I128>();
}

/// Checks certified local receipts and exact volume 31 for a three-step, 270-degree
/// axis-crossing rectangle revolution under both policies, then zero-angle error transport.
/// Volume is the asserted geometric invariant, not an exhaustive boundary oracle.
fn regularized_revolution_adapter<A: ScalarAdapter>() {
    let source = curve::translated(&curve::rectangle(5, 2).unwrap(), -2, 0).unwrap();
    let wrapped = ScalarCurve::<A>::from_native(source.clone());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let angle = A::from_real(&Real::from(270)).unwrap();
        let result = wrapped.revolve_solid(angle, 3, context).unwrap();
        assert_eq!(result.certainty, csgrs::GeometryCertainty::Certified);
        let volume = csgrs::solid::exact_mass_properties(result.value.native(), Real::one())
            .unwrap()
            .volume;
        assert_eq!(
            volume.exact_rational_normal_form(),
            Real::from(31).exact_rational_normal_form()
        );
        let angle = A::from_real(&Real::from(0)).unwrap();
        let native = curve::revolve_solid(&source, Real::zero(), 3, context).unwrap_err();
        let error = wrapped
            .revolve_solid(angle, 3, context)
            .err()
            .expect("zero angle fails");
        assert_cause(error, native, ValidationErrorKind::InvalidInput);
    }
}

#[test]
/// Runs the regularized revolution receipt/volume/error fixture through all four scalar families.
fn regularized_revolution_survives_every_scalar_adapter() {
    regularized_revolution_adapter::<RawReal>();
    regularized_revolution_adapter::<F32>();
    regularized_revolution_adapter::<F64>();
    regularized_revolution_adapter::<I128>();
}
