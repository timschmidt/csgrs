use hyperlimit::{
    Certainty, Plane3, Point3, PredicateOutcome, PredicatePolicy, classify_halfspace_feasibility3,
    classify_ray_triangle3_intersection_report,
};
use hyperreal::Real;

fn point(x: i64, y: i64, z: Real) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), z)
}

fn terminal_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
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
