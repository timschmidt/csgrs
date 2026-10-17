use super::*;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn cone_arc_excludes_other_points_on_its_circle() {
    let angle = q(1, 2) * (q(3, 4).atan().unwrap() - q(1, 2) * Real::pi());
    let arc = crate::CircularArc2::try_from_center(
        Point2::from_values(-4, 3),
        Point2::new(
            Real::from(5) * angle.clone().sin(),
            Real::from(5) * angle.cos(),
        ),
        Point2::from_values(0, 0),
        true,
    )
    .unwrap();
    let query = Point2::from_values(5, 0);
    for span in arc.rational_bezier_decomposition().unwrap().spans() {
        let curve = RationalBezier2::from(span.curve().clone());
        assert!(!curve.contains_point(&query).unwrap());
    }
    let curve = crate::Curve2::from(arc);
    let crate::CurvePointLocations2::Locations(locations) =
        curve.point_locations(&query.into()).unwrap()
    else {
        panic!("finite arc");
    };
    assert!(locations.is_empty());
}

fn circle_source(center: &Point2, weight: i32) -> RationalBezier2 {
    let arc = crate::CircularArc2::try_from_center(
        center.translated(Real::one(), Real::zero()),
        center.translated(Real::zero(), Real::one()),
        center.clone(),
        false,
    )
    .unwrap();
    let (implicit, circle) = crate::arc_bezier::circular_conic_provenance(&arc);
    // Homogeneous coordinates (1-t^2, 2t, 1+t^2).
    RationalBezier2::try_new(
        vec![
            arc.start().clone(),
            center.translated(Real::one(), Real::one()),
            arc.end().clone(),
        ],
        vec![weight.into(), weight.into(), (2 * weight).into()],
    )
    .unwrap()
    .with_implicit_quadratic_conic(implicit, Some(circle))
}

fn oracle_point(center: &Point2, t: &Real) -> Point2 {
    let square = t * t;
    center.translated(
        ((Real::one() - &square) / (Real::one() + &square)).unwrap(),
        ((Real::from(2) * t) / (Real::one() + square)).unwrap(),
    )
}

fn parameters(
    curve: &RationalBezier2,
    point: &Point2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> Vec<BezierParameter2> {
    let Classification::Decided(RationalBezierPointIncidence2::Parameters(parameters)) = curve
        .point_incidence_on_range(point, range, policy)
        .unwrap()
    else {
        panic!("nonconstant circle incidence must decide");
    };
    parameters
}

fn parameter_equals(actual: &BezierParameter2, expected: &Real) {
    let BezierParameter2::Exact(actual) = actual else {
        panic!("represented circle inverse");
    };
    assert_eq!(
        is_zero(&(actual - expected), &CurveContext::STRICT),
        Some(true)
    );
}

#[test]
fn circle_incidence_preserves_subranges_reversal_elevation_and_certainty() {
    for center in [
        Point2::from_values(0, 0),
        Point2::new(Real::pi(), Real::e()),
    ] {
        for weight in [1, -3] {
            for degree in [2, 5] {
                for (sn, en) in [(0, 4), (1, 3)] {
                    for reverse in [false, true] {
                        let source = circle_source(&center, weight)
                            .elevated_to_degree(degree)
                            .unwrap();
                        let mut curve =
                            source.subcurve_between_exact(&q(sn, 4), &q(en, 4)).unwrap();
                        if reverse {
                            curve = curve.reversed();
                        }
                        for (rn, re) in [(0, 4), (1, 3), (3, 1)] {
                            let range = CurveParameterRange2::new_validated(
                                q(rn, 4).into(),
                                q(re, 4).into(),
                            );
                            for n in [-4, 0, 1, 2, 3, 4, 8] {
                                let numerator = if reverse { en - n } else { n - sn };
                                let denominator = en - sn;
                                let inside = 4 * numerator >= rn.min(re) * denominator
                                    && 4 * numerator <= rn.max(re) * denominator;
                                let point = oracle_point(&center, &q(n, 4));
                                for policy in [
                                    CurveContext::STRICT,
                                    CurveContext::APPROXIMATE_512,
                                    CurveContext::STRICT,
                                ] {
                                    let found = parameters(&curve, &point, &range, &policy);
                                    assert_eq!(
                                        found.len(),
                                        usize::from(inside),
                                        "degree={degree}, subrange={sn}..{en}, reverse={reverse}, range={rn}..{re}, t={n}/4"
                                    );
                                    if inside {
                                        parameter_equals(&found[0], &q(numerator, denominator));
                                    }
                                }
                            }
                        }
                        let point = oracle_point(&center, &q(1, 2));
                        let public = crate::Curve2::from(curve);
                        for policy in [
                            CurveContext::STRICT,
                            CurveContext::APPROXIMATE_512,
                            CurveContext::STRICT,
                        ] {
                            let outcome = public
                                .point_locations_with_policy(&point.clone().into(), &policy)
                                .unwrap();
                            assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                            let crate::CurvePointLocations2::Locations(found) = outcome.value
                            else {
                                panic!("nonconstant circle");
                            };
                            assert_eq!(found.len(), 1);
                            let parameter = found[0].parameter().unwrap();
                            assert_eq!(
                                is_zero(
                                    &(parameter.scalar().unwrap() - q(1, 2)),
                                    &CurveContext::STRICT
                                ),
                                Some(true)
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn circle_incidence_keeps_near_points_extrapolation_constants_and_poles() {
    let center = Point2::from_values(0, 0);
    let source = circle_source(&center, 1);
    let unit = CurveParameterRange2::unit();
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    for sign in [-1, 1] {
        let point = Point2::new(Real::one() + Real::from(sign) * &delta, Real::zero());
        assert!(parameters(&source, &point, &unit, &CurveContext::STRICT).is_empty());
    }
    let exterior =
        CurveParameterRange2::new_validated(Real::from(-2).into(), Real::from(-1).into());
    let found = parameters(
        &source,
        &oracle_point(&center, &Real::from(-1)),
        &exterior,
        &CurveContext::STRICT,
    );
    assert_eq!(found.len(), 1);
    parameter_equals(&found[0], &Real::from(-1));

    for degree in [2, 5] {
        let collapsed = source
            .elevated_to_degree(degree)
            .unwrap()
            .subcurve_between_exact(&q(1, 2), &q(1, 2))
            .unwrap();
        assert!(collapsed.retained_circular_conic().is_some());
        assert_eq!(
            collapsed
                .point_incidence_on_range(
                    &oracle_point(&center, &q(1, 2)),
                    &unit,
                    &CurveContext::STRICT
                )
                .unwrap(),
            Classification::Decided(RationalBezierPointIncidence2::EntireCurve)
        );
    }
    let pole = RationalBezier2::try_new(
        vec![Point2::from_values(1, 0); 3],
        vec![Real::one(), -Real::one(), Real::one()],
    )
    .unwrap()
    .with_implicit_quadratic_conic(
        source.retained_implicit_quadratic_conic().unwrap().clone(),
        source.retained_circular_conic().cloned(),
    );
    for point in [Point2::from_values(1, 0), Point2::from_values(2, 0)] {
        assert_eq!(
            pole.point_incidence_on_range(&point, &unit, &CurveContext::STRICT)
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn circular_support_alone_does_not_erase_repeated_visits() {
    let source = circle_source(&Point2::from_values(0, 0), 1);
    // Compose the independent circle oracle with s=4*t*(1-t). Its
    // homogeneous power coordinates are (1-s^2, 2*s, 1+s^2).
    let retracing = RationalBezier2::try_new(
        vec![
            Point2::from_values(1, 0),
            Point2::from_values(1, 2),
            Point2::new(q(-5, 11), q(8, 11)),
            Point2::from_values(1, 2),
            Point2::from_values(1, 0),
        ],
        vec![Real::one(), Real::one(), q(11, 3), Real::one(), Real::one()],
    )
    .unwrap()
    .with_implicit_quadratic_conic(
        source.retained_implicit_quadratic_conic().unwrap().clone(),
        source.retained_circular_conic().cloned(),
    );
    assert!(
        retracing
            .data
            .lineage
            .root
            .quadratic_conic_parameter_frame
            .get()
            .is_none()
    );
    let found = parameters(
        &retracing,
        &Point2::new(q(7, 25), q(24, 25)),
        &CurveParameterRange2::unit(),
        &CurveContext::STRICT,
    );
    assert_eq!(found.len(), 2);
    parameter_equals(&found[0], &q(1, 4));
    parameter_equals(&found[1], &q(3, 4));
}
