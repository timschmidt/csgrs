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
fn similarity_preserves_retained_circle_and_selected_chart() {
    let center = Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap());
    for degree in [2, 5] {
        let source = circle_source(&center, -3)
            .elevated_to_degree(degree)
            .unwrap()
            .subcurve_between_exact(&q(1, 4), &q(3, 4))
            .unwrap();
        for reversed in [false, true] {
            let source = if reversed {
                source.reversed()
            } else {
                source.clone()
            };
            for entries in [[3, -4, 4, 3, 7, -2], [3, 4, 4, -3, 1, 5]] {
                let [a, b, c, d, x, y] = entries.map(Real::from);
                let transform = crate::Similarity2::try_from_real_affine(a, b, c, d, x, y).unwrap();
                let image = source.transform_similarity(&transform);
                let circle = image
                    .retained_circular_conic()
                    .expect("similarity retains the circle definition");
                assert_eq!(circle.center, transform.transform_point(&center));
                assert_eq!(
                    is_zero(
                        &(&circle.radius_squared - Real::from(25)),
                        &CurveContext::STRICT
                    ),
                    Some(true)
                );
                for t in [q(1, 4), q(1, 2), q(3, 4)] {
                    let expected = transform.transform_point(&oracle_point(&center, &t));
                    let local = if reversed {
                        (q(3, 4) - &t) * Real::from(2)
                    } else {
                        (&t - q(1, 4)) * Real::from(2)
                    };
                    let found = parameters(
                        &image,
                        &expected,
                        &CurveParameterRange2::unit(),
                        &CurveContext::STRICT,
                    );
                    assert_eq!(found.len(), 1);
                    parameter_equals(&found[0], &local);
                }
            }
        }
    }
}

#[test]
fn native_arc_reconstruction_preserves_circle_tangent_contacts() {
    use crate::rational_bezier::{
        RationalQuadraticCircleTangentContact2 as Contact, RationalQuadraticParallelCircleContact2,
    };
    let policy = CurveContext::STRICT;
    let center = Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap());
    let point = center.translated(Real::one(), Real::zero());
    let line = LineSeg2::try_new(
        center.translated(Real::one(), -Real::one()),
        center.translated(Real::one(), Real::one()),
    )
    .unwrap();
    let parallel = crate::QuadraticBezier2::from_line_segment(line.clone())
        .parallel_left(Real::zero())
        .unwrap();
    let source = circle_source(&center, -3);
    let mut circle = source.retained_circular_conic().unwrap().as_ref().clone();
    circle.tangent_contacts = Some(Arc::from([
        Contact::Line {
            line,
            point: point.clone(),
        },
        Contact::Parallel(RationalQuadraticParallelCircleContact2 {
            parallel,
            parameter: q(1, 2),
            point: point.clone(),
            eliminant_root_multiplicity: 2,
        }),
    ]));
    let circle = Arc::new(circle);
    let source = RationalBezier2::try_new(
        vec![
            point.clone(),
            center.translated(Real::one(), Real::one()),
            center.translated(Real::zero(), Real::one()),
        ],
        vec![Real::from(-3), Real::from(-3), Real::from(-6)],
    )
    .unwrap()
    .with_implicit_quadratic_conic(circle.implicit_coefficients(), Some(Arc::clone(&circle)));
    assert_eq!(source.retained_circular_conic(), Some(&circle));
    let Classification::Decided(Some(native)) =
        crate::arc_bezier::rational_bezier_circular_arc(&source, &policy).unwrap()
    else {
        panic!("the retained circle has one native arc");
    };
    let (_, promoted_circle) = crate::arc_bezier::circular_conic_provenance(&native);
    assert_eq!(
        promoted_circle, circle,
        "native promotion keeps both contact proofs"
    );
    let Classification::Decided(Some(quadratic)) = source
        .materialized_quadratic_representative(&policy)
        .unwrap()
    else {
        panic!("the rational quadratic has a native representative");
    };
    let Classification::Decided(Some(quadratic_arc)) =
        crate::arc_bezier::rational_quadratic_circular_arc(&quadratic, &policy).unwrap()
    else {
        panic!("the quadratic retains its circle");
    };
    let elevated = source.elevated_to_degree(5).unwrap();
    let Classification::Decided(Some(elevated_arc)) =
        crate::arc_bezier::rational_bezier_circular_arc(&elevated, &policy).unwrap()
    else {
        panic!("degree elevation retains the selected circular chart");
    };
    let midpoint = q(1, 2).sqrt().unwrap();
    let (first, second) = native
        .split_at_retained_sweep_point(&q(1, 2), center.translated(midpoint.clone(), midpoint))
        .unwrap();
    for (arc, contains_contact) in [
        (native.clone(), true),
        (native.reversed(), true),
        (quadratic_arc, true),
        (elevated_arc, true),
        (first, true),
        (second, false),
    ] {
        let (_, support) = crate::arc_bezier::circular_conic_provenance(&arc);
        assert_eq!(support, circle);
        assert_eq!(arc.contains_point(&point).unwrap(), contains_contact);
        for span in arc.rational_bezier_decomposition().unwrap().spans() {
            assert_eq!(span.curve().retained_circular_conic(), Some(&circle));
        }
        let selected = crate::Curve2::from(arc.clone())
            .subcurve(q(1, 4).into(), q(3, 4).into())
            .unwrap();
        let Some(crate::CurveGeometry2::CircularArc(selected)) = selected.geometry() else {
            panic!("a native scalar restriction stays circular");
        };
        assert_eq!(selected.retained_circular_conic(), Some(&circle));
        assert!(!selected.contains_point(&point).unwrap());
        for entries in [[3, -4, 4, 3, 7, -2], [3, 4, 4, -3, 1, 5]] {
            let [a, b, c, d, x, y] = entries.map(Real::from);
            let transform = crate::Similarity2::try_from_real_affine(a, b, c, d, x, y).unwrap();
            let image = arc.transform_similarity(&transform).unwrap();
            assert_eq!(
                image.is_clockwise(),
                arc.is_clockwise() ^ transform.reverses_orientation()
            );
            let expected = circle.transform_similarity(&transform).unwrap();
            let (_, image_circle) = crate::arc_bezier::circular_conic_provenance(&image);
            assert_eq!(image_circle.as_ref(), &expected);
            assert_eq!(
                is_zero(
                    &(image.radius_squared() - &expected.radius_squared),
                    &policy
                ),
                Some(true)
            );
            assert_eq!(
                image
                    .contains_point(&transform.transform_point(&point))
                    .unwrap(),
                contains_contact
            );
        }
    }
}

#[test]
fn similarity_transports_circle_tangent_relations_and_invalidates_affine_metric() {
    use crate::rational_bezier::{
        RationalQuadraticCircle2, RationalQuadraticCircleTangentContact2 as Contact,
        RationalQuadraticParallelCircleContact2,
    };
    let center = Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap());
    let point = center.translated(Real::one(), Real::zero());
    let support = LineSeg2::try_new(
        center.translated(Real::one(), Real::from(-2)),
        center.translated(Real::one(), Real::from(2)),
    )
    .unwrap();
    let line = support.fragment_between_after_distinct_endpoints(
        center.translated(Real::one(), Real::from(-1)),
        center.translated(Real::one(), Real::one()),
        support.fragment_support(),
    );
    let source = crate::QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(
            center.translated(Real::from(2), Real::from(-1)),
            center.translated(Real::from(2), Real::one()),
        )
        .unwrap(),
    );
    let parallel = crate::BezierParallel2::from_source(
        crate::BezierParallelSource2::Quadratic(source),
        Real::one(),
    );
    assert_eq!(parallel.point_at(&q(1, 2)).unwrap(), point);
    let circle = Arc::new(RationalQuadraticCircle2 {
        center: center.clone(),
        radius_squared: Real::one(),
        tangent_contacts: Some(Arc::from([
            Contact::Line {
                line,
                point: point.clone(),
            },
            Contact::Parallel(RationalQuadraticParallelCircleContact2 {
                parallel,
                parameter: q(1, 2),
                point: point.clone(),
                eliminant_root_multiplicity: 2,
            }),
        ])),
    });
    let conic = crate::RationalQuadraticBezier2::try_new(
        point.clone(),
        center.translated(Real::one(), Real::one()),
        center.translated(Real::zero(), Real::one()),
        Real::one(),
        Real::one(),
        Real::from(2),
    )
    .unwrap()
    .with_retained_conic_provenance(Some(circle.implicit_coefficients()), Some(circle));
    let rational = RationalBezier2::from(conic.clone());
    for entries in [[3, -4, 4, 3, 7, -2], [3, 4, 4, -3, 1, 5]] {
        let [a, b, c, d, x, y] = entries.map(Real::from);
        let transform = crate::Similarity2::try_from_real_affine(a, b, c, d, x, y).unwrap();
        let conic_image = conic
            .transform_similarity_with_retained_provenance(&transform)
            .unwrap();
        let rational_image = rational.transform_similarity(&transform);
        for circle in [
            conic_image.retained_circular_conic().unwrap(),
            rational_image.retained_circular_conic().unwrap(),
        ] {
            let contacts = circle.tangent_contacts.as_ref().unwrap();
            assert_eq!(contacts.len(), 2);
            for contact in contacts.iter() {
                let (contact_point, tangent) = match contact {
                    Contact::Concentric { .. } => panic!("this fixture has no concentric image"),
                    Contact::Line { line, point } => {
                        assert!(line.has_retained_support());
                        (point, line.directed_support_delta())
                    }
                    Contact::Parallel(contact) => {
                        assert_eq!(contact.parameter, q(1, 2));
                        assert_eq!(contact.eliminant_root_multiplicity, 2);
                        let expected_distance = Real::from(if transform.reverses_orientation() {
                            -5
                        } else {
                            5
                        });
                        assert_eq!(contact.parallel.distance(), &expected_distance);
                        assert_eq!(
                            contact.parallel.point_at(&contact.parameter).unwrap(),
                            contact.point
                        );
                        (
                            &contact.point,
                            transform.transform_vector_coordinates(&Real::zero(), &Real::one()),
                        )
                    }
                };
                assert_eq!(contact_point, &transform.transform_point(&point));
                let radial = contact_point.delta_from(&circle.center);
                assert_eq!(
                    is_zero(
                        &(radial.0 * tangent.0 + radial.1 * tangent.1),
                        &CurveContext::STRICT
                    ),
                    Some(true)
                );
                assert_eq!(
                    is_zero(
                        &(contact_point.distance_squared(&circle.center) - &circle.radius_squared),
                        &CurveContext::STRICT
                    ),
                    Some(true)
                );
            }
        }
    }
    // A nonuniform affine map preserves the conic but cannot retain a circle's metric.
    let [a, b, c, d, x, y] = [2, 1, 0, 3, 0, 0].map(Real::from);
    let sheared = rational.transformed_affine([&a, &b, &c, &d, &x, &y]);
    assert!(sheared.retained_circular_conic().is_none());
    assert!(sheared.retained_implicit_quadratic_conic().is_none());
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
