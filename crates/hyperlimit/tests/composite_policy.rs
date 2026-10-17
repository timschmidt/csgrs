use hyperlimit::{
    Certainty, Plane3, Point3, PredicateOutcome, PredicatePolicy, classify_halfspace_feasibility3,
    classify_ray_triangle3_intersection_report,
};
use hyperreal::Real;

fn point(x: i64, y: i64, z: Real) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), z)
}

fn original_unit_circle_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

// Angle addition keeps the independently justified terminal-zero cases.
fn terminal_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

#[test]
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
fn ray_hits_on_computable_triangles_are_certified_without_a_coplanarity_retest() {
    // The ray/plane intersection lies on the support plane by construction.
    // Its orient3d value is an exact zero that structural facts cannot prove
    // for trigonometric coordinates, so classifying the constructed point
    // must not re-derive coplanarity.
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
fn ray_through_an_exact_vertex_of_a_computable_triangle_is_a_certified_boundary_touch() {
    // The reconstructed hit equals the vertex only as an unprovable exact
    // identity; the Plucker side products of the input coordinates certify
    // the vertex contact structurally.
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
