//! Composite-policy qualification and retained symbolic-geometry regressions.
//!
//! Terminal controls are paired with the original now-recognized unit-circle
//! identity. Tests distinguish unresolved strict queries, qualified replay and
//! certified completion; individual assertions below determine whether value,
//! relation, receipt or source replay is actually checked. No dispatch counters
//! establish the absence of a coplanarity retest in this file.

use hyperlimit::{
    Certainty, Plane3, Point3, PredicateOutcome, PredicatePolicy, classify_halfspace_feasibility3,
    classify_ray_triangle3_intersection_report,
};
use hyperreal::Real;

/// Construct exact integer x/y coordinates with an already-exact symbolic or rational z.
fn point(x: i64, y: i64, z: Real) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), z)
}

/// Preserve sin(e)^2 + cos(e)^2 - 1 as a mathematical-zero regression for stronger recognition.
fn original_unit_circle_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

/// Construct the angle-addition zero used as the still-unresolved strict-policy control.
fn terminal_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

#[test]
/// Require strict Unknown then an Approximate decided receipt for a ray with symbolic-zero origin height.
/// The test does not inspect the decided relation or point coordinates.
fn ray_report_marks_a_terminal_approximation_from_a_child_predicate() {
    let a = point(0, 0, Real::from(0));
    let b = point(1, 0, Real::from(0));
    let c = point(0, 1, Real::from(0));
    let origin = point(0, 0, terminal_zero());
    let direction = point(0, 0, Real::from(1));

    assert!(matches!(
        classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &a,
            &b,
            &c,
            PredicatePolicy::STRICT,
        ),
        PredicateOutcome::Unknown { .. }
    ));
    assert!(matches!(
        classify_ray_triangle3_intersection_report(
            &origin,
            &direction,
            &a,
            &b,
            &c,
            PredicatePolicy::APPROXIMATE_512,
        ),
        PredicateOutcome::Decided {
            certainty: Certainty::Approximate,
            ..
        }
    ));
}

#[test]
/// Require strict Unknown then Approximate completion for a constant symbolic-zero halfspace constraint.
/// This witness checks qualification, not the returned feasibility status or point.
fn halfspace_report_marks_terminal_approximation_used_to_accept_a_witness() {
    let plane = Plane3::new(point(0, 0, Real::from(0)), terminal_zero());

    assert!(matches!(
        classify_halfspace_feasibility3(core::slice::from_ref(&plane), PredicatePolicy::STRICT),
        PredicateOutcome::Unknown { .. }
    ));
    assert!(matches!(
        classify_halfspace_feasibility3(&[plane], PredicatePolicy::APPROXIMATE_512),
        PredicateOutcome::Decided {
            certainty: Certainty::Approximate,
            ..
        }
    ));
}

#[test]
/// Require certified completion of a rational ray query even when approximation is permitted.
fn composite_reports_remain_certified_for_exact_rational_inputs() {
    let a = point(0, 0, Real::from(0));
    let b = point(1, 0, Real::from(0));
    let c = point(0, 1, Real::from(0));
    let origin = point(0, 0, Real::from(1));
    let direction = point(0, 0, Real::from(-1));
    let report = classify_ray_triangle3_intersection_report(
        &origin,
        &direction,
        &a,
        &b,
        &c,
        PredicatePolicy::APPROXIMATE_512,
    );

    assert!(matches!(
        report,
        PredicateOutcome::Decided {
            certainty: Certainty::Exact | Certainty::Filtered,
            ..
        }
    ));
}

#[test]
/// Require an Exact Proper hit on the retained pi/16 trigonometric triangle under STRICT.
/// The result contract is tested; this file does not instrument the dispatch sequence.
fn ray_hits_on_computable_triangles_are_certified_without_a_coplanarity_retest() {
    // The ray/plane construction supplies incidence with the support plane.
    // Preserve the trigonometric source fixture and exact proper-hit expectation;
    // this assertion does not require a particular coplanarity decision path.
    let angle = (Real::pi() / Real::from(16)).unwrap();
    let (sine, cosine) = (angle.clone().sin(), angle.cos());
    let a = Point3::new(Real::from(10), Real::zero(), Real::zero());
    let b = Point3::new(
        Real::zero(),
        &cosine * Real::from(10),
        &sine * Real::from(10),
    );
    let c = Point3::new(
        Real::zero(),
        &sine * Real::from(-10),
        &cosine * Real::from(10),
    );
    let origin = point(0, 0, Real::from(0));
    let direction = point(1, 1, Real::from(2));

    let report = classify_ray_triangle3_intersection_report(
        &origin,
        &direction,
        &a,
        &b,
        &c,
        PredicatePolicy::STRICT,
    );
    let PredicateOutcome::Decided {
        value, certainty, ..
    } = report
    else {
        panic!("a proper hit on a computable triangle must be certified");
    };
    assert_eq!(certainty, Certainty::Exact);
    assert_eq!(value.relation, hyperlimit::RayTriangleIntersection::Proper);
}

#[test]
/// Require a decided STRICT ray report with BoundaryTouch and OnVertex for the authored (10,0,0) vertex.
/// The explicit assertions inspect relation/location rather than the returned certainty label.
fn ray_through_an_exact_vertex_of_a_computable_triangle_is_a_certified_boundary_touch() {
    // The ray reaches the authored vertex (10,0,0). Preserve that independent
    // contact expectation without assuming a scalar identity is unprovable or
    // asserting which retained-side or reconstruction path established it.
    let angle = (Real::pi() / Real::from(16)).unwrap();
    let (sine, cosine) = (angle.clone().sin(), angle.cos());
    let a = Point3::new(Real::from(10), Real::zero(), Real::zero());
    let b = Point3::new(
        &cosine * Real::from(10),
        &sine * Real::from(10),
        Real::zero(),
    );
    let c = Point3::new(
        &cosine * Real::from(10),
        Real::zero(),
        &sine * Real::from(10),
    );
    let origin = point(-20, 0, Real::from(0));
    let direction = point(1, 0, Real::from(0));

    let report = classify_ray_triangle3_intersection_report(
        &origin,
        &direction,
        &a,
        &b,
        &c,
        PredicatePolicy::STRICT,
    );
    let PredicateOutcome::Decided { value, .. } = report else {
        panic!("a ray through an exact vertex must be certified");
    };
    assert_eq!(
        value.relation,
        hyperlimit::RayTriangleIntersection::BoundaryTouch
    );
    assert_eq!(
        value.triangle_location,
        Some(hyperlimit::Triangle3Location::OnVertex)
    );
}

#[test]
/// Require certified unit-circle ray/halfspace results through strict/approximate/strict calls.
/// Check the zero ray parameter, vertex contact, strict source replay and feasible halfspace replay.
fn original_unit_circle_inputs_now_produce_certified_composite_reports() {
    let a = point(0, 0, Real::zero());
    let b = point(1, 0, Real::zero());
    let c = point(0, 1, Real::zero());
    let origin = point(0, 0, original_unit_circle_zero());
    let direction = point(0, 0, Real::one());
    let plane = Plane3::new(point(0, 0, Real::zero()), original_unit_circle_zero());
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let PredicateOutcome::Decided {
            value: ray,
            certainty: Certainty::Exact | Certainty::Filtered,
            ..
        } = classify_ray_triangle3_intersection_report(&origin, &direction, &a, &b, &c, policy)
        else {
            panic!("the original ray now has an exact unit-circle proof");
        };
        assert_eq!(
            ray.relation,
            hyperlimit::RayTriangleIntersection::BoundaryTouch
        );
        assert_eq!(
            ray.triangle_location,
            Some(hyperlimit::Triangle3Location::OnVertex)
        );
        assert_eq!(ray.parameter, Some(Real::zero()));
        ray.validate_against_sources(&origin, &direction, &a, &b, &c, PredicatePolicy::STRICT)
            .unwrap();
        let PredicateOutcome::Decided {
            value: halfspace,
            certainty: Certainty::Exact | Certainty::Filtered,
            ..
        } = classify_halfspace_feasibility3(core::slice::from_ref(&plane), policy)
        else {
            panic!("the original halfspace now has an exact unit-circle proof");
        };
        assert!(halfspace.is_feasible());
        assert!(matches!(
            halfspace
                .validate_against_planes(core::slice::from_ref(&plane), PredicatePolicy::STRICT),
            PredicateOutcome::Decided {
                value: true,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
    }
}
