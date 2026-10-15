//! Construction failure is distinct from a successfully represented empty set.
#![cfg(feature = "curve")]

use csgrs::{GeometryContext, Real, adapter::RawCurveRegion, curve};
use std::error::Error;

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn negative_dimensions_fail_through_the_public_adapter() {
    assert!(RawCurveRegion::square(-Real::one()).is_err());
    assert!(RawCurveRegion::rectangle(Real::one(), -Real::one()).is_err());
    assert!(RawCurveRegion::circle(-Real::one(), 4).is_err());
}

#[test]
fn undecided_dimensions_do_not_become_empty_regions() {
    assert!(RawCurveRegion::square(unresolved_zero()).is_err());
}

#[test]
fn degenerate_dimensions_do_not_conceal_invalid_companions() {
    assert!(
        curve::rectangle_with_context(Real::zero(), -Real::one(), &GeometryContext::STRICT)
            .is_err()
    );
    assert!(
        curve::rectangle_with_context(
            Real::zero(),
            unresolved_zero(),
            &GeometryContext::STRICT
        )
        .is_err()
    );
}

#[test]
fn invalid_resolution_fails_even_for_a_zero_radius() {
    for radius in [Real::one(), Real::zero()] {
        assert!(RawCurveRegion::circle(radius, 2).is_err());
    }
}

#[test]
fn malformed_polygon_cannot_enter_a_downstream_pipeline() {
    let points = [[Real::zero(), Real::zero()], [Real::one(), Real::zero()]];
    assert!(RawCurveRegion::polygon(&points).is_err());
    assert!(RawCurveRegion::polygon(&[]).is_err());
}

#[test]
fn native_polygon_cause_survives_the_modeling_wrapper() {
    let error = curve::polygon_with_context(&[], &GeometryContext::STRICT).unwrap_err();
    assert!(
        error.source().is_some(),
        "native cause was flattened: {error:?}"
    );
}

#[test]
fn compound_constructors_reject_invalid_parameters() {
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        assert!(curve::keyhole(Real::one(), -Real::one(), Real::one(), 4, &context).is_err());
        assert!(curve::ring(Real::one(), -Real::one(), 4, &context).is_err());
        assert!(curve::reuleaux(4, Real::one(), 6, &context).is_err());
        assert!(
            curve::crescent(Real::one(), Real::from(2), Real::zero(), 6, &context).is_err()
        );
        assert!(
            curve::circle_with_keyway(Real::one(), 4, Real::from(2), Real::one(), &context)
                .is_err()
        );
        assert!(curve::circle_with_flat(Real::one(), 4, Real::one(), &context).is_err());
        assert!(curve::circle_with_two_flats(Real::one(), 4, Real::one(), &context).is_err());
    }
}

#[test]
fn checked_compound_capacity_failure_is_not_successful_empty_geometry() {
    assert!(
        curve::rounded_rectangle_with_context(
            Real::from(4),
            Real::from(4),
            Real::one(),
            usize::MAX,
            &GeometryContext::STRICT
        )
        .is_err()
    );
}

#[test]
fn undecided_compound_parameter_retains_a_typed_cause() {
    let error = curve::keyhole(
        unresolved_zero(),
        Real::one(),
        Real::one(),
        4,
        &GeometryContext::STRICT,
    )
    .unwrap_err();
    assert!(
        matches!(
            error,
            csgrs::errors::ValidationError::PredicateUndecided { .. }
        ),
        "predicate cause was flattened: {error:?}"
    );
}

#[test]
fn failures_remain_structured_through_scalar_adapters() {
    use csgrs::{
        adapter::AdapterError,
        errors::{ValidationError, ValidationErrorKind},
    };
    for (result, kind) in [
        (
            RawCurveRegion::square(-Real::one()),
            ValidationErrorKind::InvalidInput,
        ),
        (
            RawCurveRegion::square(unresolved_zero()),
            ValidationErrorKind::Undecided,
        ),
        (
            RawCurveRegion::circle(Real::one(), usize::MAX),
            ValidationErrorKind::ResourceExhausted,
        ),
        (
            RawCurveRegion::polygon(&[]),
            ValidationErrorKind::InvalidInput,
        ),
    ] {
        let error = result.unwrap_err();
        let AdapterError::Construction(cause) = &error else {
            panic!("lost cause: {error:?}");
        };
        assert_eq!(cause.kind(), kind);
        assert!(error.source().is_some());
    }
    assert!(matches!(
        curve::polygon(&[]),
        Err(ValidationError::Curve(
            hypercurve::CurveError::InsufficientVertices
        ))
    ));
}

#[test]
fn canonical_degeneracies_compose_and_invalid_companions_still_fail() {
    use hypercurve::BooleanOp;
    let unit = curve::square(Real::one()).unwrap();
    for region in [
        curve::square(Real::zero()),
        curve::rectangle(Real::zero(), Real::one()),
        curve::right_triangle(Real::one(), Real::zero()),
        curve::circle(Real::zero(), 4),
        curve::ellipse(Real::zero(), Real::one(), 4),
        curve::regular_ngon(4, Real::zero()),
        curve::rounded_rectangle(Real::zero(), Real::one(), Real::one(), 2),
        curve::circle_with_two_flats(Real::one(), 4, Real::zero(), &GeometryContext::STRICT)
            .map(csgrs::GeometryOutcome::into_value),
    ] {
        let empty = region.unwrap();
        assert!(empty.is_empty());
        let restored = empty.boolean_region(&unit, BooleanOp::Union).unwrap();
        assert_eq!(restored.filled_area().unwrap(), Some(Real::one()));
        let removed = restored.boolean_region(&unit, BooleanOp::Difference).unwrap();
        let extruded =
            curve::try_extrude(&removed, Real::one(), &GeometryContext::STRICT).unwrap();
        assert!(extruded.value.positions.is_empty() && extruded.value.triangles.is_empty());
    }
    assert!(curve::ellipse(Real::zero(), -Real::one(), 4).is_err());
    assert!(curve::right_triangle(Real::zero(), unresolved_zero()).is_err());
    assert!(curve::rounded_rectangle(Real::zero(), Real::one(), -Real::one(), 2).is_err());
    assert!(curve::circle(Real::zero(), usize::MAX).is_err());
}

#[test]
fn exact_constructor_sets_survive_triangulation_and_extrusion() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    for (region, area) in [
        (curve::rectangle(q(5, 3), q(7, 2)).unwrap(), q(35, 6)),
        (curve::right_triangle(q(5, 3), q(7, 2)).unwrap(), q(35, 12)),
        (curve::circle(Real::one(), 4).unwrap(), Real::from(2)),
        (
            curve::ellipse(Real::from(2), Real::from(4), 4).unwrap(),
            Real::from(4),
        ),
    ] {
        assert_eq!(region.filled_area().unwrap(), Some(area.clone()));
        let flat = curve::try_triangulate(&region, &GeometryContext::STRICT).unwrap();
        let twice_area = flat
            .value
            .triangles
            .iter()
            .fold(Real::zero(), |sum, triangle| {
                let [a, b, c] = triangle.indices().map(|i| &flat.value.positions[i]);
                sum + (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x)
            });
        assert_eq!(twice_area, Real::from(2) * &area);
        let solid =
            curve::try_extrude(&region, Real::from(3), &GeometryContext::STRICT).unwrap();
        assert!(solid.value.is_closed_manifold());
        let six_volume = solid
            .value
            .triangles
            .iter()
            .fold(Real::zero(), |sum, triangle| {
                let [a, b, c] = triangle
                    .indices()
                    .map(|i| solid.value.positions[i].to_vector());
                sum + a.dot(&b.cross(&c))
            });
        assert_eq!(six_volume, Real::from(18) * area);
    }
}

#[test]
fn approximate_selection_and_consumption_are_distinct_and_do_not_poison_strict_calls() {
    use csgrs::GeometryCertainty;
    for _ in 0..2 {
        assert!(
            curve::square_with_context(unresolved_zero(), &GeometryContext::STRICT).is_err()
        );
        let outcome =
            curve::square_with_context(unresolved_zero(), &GeometryContext::APPROXIMATE_512)
                .unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
        assert!(outcome.value.is_empty());
        assert!(
            curve::square_with_context(unresolved_zero(), &GeometryContext::STRICT).is_err()
        );
        let exact = curve::square_with_context(Real::one(), &GeometryContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(exact.certainty, GeometryCertainty::Certified);
        assert_eq!(exact.value.filled_area().unwrap(), Some(Real::one()));
    }
}

#[test]
fn representable_sample_count_with_unrepresentable_bytes_fails_before_allocation() {
    let count = isize::MAX as usize / std::mem::size_of::<[Real; 2]>() + 1;
    for result in [
        curve::circle(Real::one(), count),
        curve::ellipse(Real::one(), Real::one(), count),
    ] {
        assert!(matches!(
            result,
            Err(csgrs::errors::ValidationError::CapacityOverflow)
        ));
    }
}
