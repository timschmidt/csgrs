use hyperlimit::{
    Certainty, Plane3, Point3, PredicateOutcome, PredicatePolicy, classify_halfspace_feasibility3,
    classify_ray_triangle3_intersection_report,
};
use hyperreal::Real;

fn point(x: i64, y: i64, z: Real) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), z)
}

fn terminal_zero() -> Real {
    (Real::pi() + Real::e()) - (Real::e() + Real::pi())
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
