use super::*;
use crate::Vector3;

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn circle(x: Vector3, y: Vector3, start: Real, end: Real) -> Curve3 {
    Curve3::circle_arc(p(0, 0, 0), x, y, 2.into(), start, end).unwrap()
}

#[test]
fn complete_circle_matching_distinguishes_phase_support_and_traversal() {
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let first = circle(Vector3::x(), Vector3::y(), Real::zero(), half_pi.clone());
    let rotated = circle(Vector3::y(), -Vector3::x(), -half_pi, Real::zero());
    assert!(exact_edge_curve_equal(&rotated, &first, false).unwrap());
    assert!(exact_edge_curve_equal(&rotated.reversed().unwrap(), &first, true).unwrap());
    assert!(!exact_edge_curve_equal(&rotated.reversed().unwrap(), &first, false).unwrap());
    let upper = circle(Vector3::x(), Vector3::y(), Real::zero(), Real::pi());
    let lower = circle(Vector3::x(), -Vector3::y(), Real::zero(), Real::pi());
    assert!(points_exactly_equal(&upper.start().unwrap(), &lower.start().unwrap()).unwrap());
    assert!(points_exactly_equal(&upper.end().unwrap(), &lower.end().unwrap()).unwrap());
    assert!(!exact_edge_curve_equal(&upper, &lower, false).unwrap());
    let wrong_support = Curve3::circle_arc(
        p(1, 0, 0),
        Vector3::x(),
        Vector3::y(),
        2.into(),
        Real::zero(),
        Real::pi(),
    )
    .unwrap();
    assert!(!exact_edge_curve_equal(&upper, &wrong_support, false).unwrap());
    let full = circle(Vector3::x(), Vector3::y(), Real::zero(), Real::tau());
    assert!(points_exactly_equal(&full.start().unwrap(), &full.end().unwrap()).unwrap());
    assert!(exact_edge_curve_equal(&full, &full, false).unwrap());
    assert!(!exact_edge_curve_equal(&full, &full, true).unwrap());
    assert!(exact_edge_curve_equal(&full.reversed().unwrap(), &full, true).unwrap());
}

#[test]
fn periodic_iso_lifts_retain_images_across_distant_signed_charts() {
    let (model, _) = crate::construction::torus(5.into(), 2.into()).unwrap();
    let face = FaceId::from_index(0).unwrap();
    let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
    let SurfaceSurfaceIntersection::Curves(traces) = face_surface(&model, face)
        .intersect_surface(&plane)
        .unwrap()
    else {
        panic!("two axial torus circles");
    };
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let trace = traces[0].subcurve(&Real::zero(), &half_pi).unwrap();
    let baseline =
        periodic_face_trace_lifts(&model, face, &trace, SurfaceIntersectionOperand::First).unwrap();
    assert!(!baseline.is_empty());
    for turns in [-1_000_000, -1, 0, 1, 1_000_000] {
        let shifted = trace
            .translated_iso_pcurve(
                SurfaceIntersectionOperand::First,
                &(Real::from(turns) * Real::tau()),
                &Real::zero(),
            )
            .unwrap();
        let lifts =
            periodic_face_trace_lifts(&model, face, &shifted, SurfaceIntersectionOperand::First)
                .unwrap();
        assert_eq!(lifts.len(), baseline.len());
        for (actual, expected) in lifts.iter().zip(&baseline) {
            assert!(actual.curve().shares_parameterization(expected.curve()));
            for parameter in [
                actual.curve().domain().start(),
                actual.curve().domain().end(),
            ] {
                let a = actual.first_pcurve().point_at(parameter).unwrap();
                let b = expected.first_pcurve().point_at(parameter).unwrap();
                assert_eq!(
                    compare_reals(&a.x, &b.x, crate::STRICT_PREDICATES).value(),
                    Some(Ordering::Equal)
                );
                assert_eq!(
                    compare_reals(&a.y, &b.y, crate::STRICT_PREDICATES).value(),
                    Some(Ordering::Equal)
                );
            }
        }
    }
}

#[test]
fn plane_bound_separation_keeps_touching_and_undecided_queries_unrejected() {
    let (model, _) = crate::construction::cuboid(p(0, 0, 0), p(8, 8, 8)).unwrap();
    let face = FaceId::from_index(0).unwrap();
    for (x, separated) in [(9, true), (8, false), (7, false)] {
        let plane = Surface::plane(p(x, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert_eq!(
            plane_separates_finite_face(&plane, &model, face).unwrap(),
            separated
        );
    }
    let enclosure = Real::pi().certified_rational_interval(-2048).unwrap();
    let midpoint = ((Real::new(enclosure[0].clone()) + Real::new(enclosure[1].clone()))
        / Real::from(2))
    .unwrap();
    let x = Real::from(8) + Real::pi() - midpoint;
    let plane = Surface::plane(
        Point3::new(x.clone(), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let bounds = certified_face_bounds(&model, face).unwrap().unwrap();
    assert!(matches!(
        hyperlimit::classify_plane_aabb3(
            &hyperlimit::Plane3::new(p(1, 0, 0), -x),
            &bounds.mins,
            &bounds.maxs,
            crate::STRICT_PREDICATES
        ),
        PredicateOutcome::Unknown { .. }
    ));
    assert!(!plane_separates_finite_face(&plane, &model, face).unwrap());
}
