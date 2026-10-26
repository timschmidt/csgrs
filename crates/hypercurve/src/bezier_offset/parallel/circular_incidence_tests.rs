use super::*;
use crate::classify::is_zero;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn circular_source(center: &Point2, weight: i32) -> RationalBezier2 {
    let arc = crate::CircularArc2::try_from_center(
        center.translated(Real::one(), Real::zero()),
        center.translated(Real::zero(), Real::one()),
        center.clone(),
        false,
    )
    .unwrap();
    let (implicit, circle) = crate::arc_bezier::circular_conic_provenance(&arc);
    // C + ((1-t^2)/(1+t^2), 2t/(1+t^2)), with the original parameter.
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

#[test]
fn circular_component_does_not_fill_a_stationary_normal_parameter() {
    let policy = CurveContext::STRICT;
    let arc = crate::CircularArc2::try_from_center(
        Point2::from_values(1, 0),
        Point2::from_values(0, 1),
        Point2::from_values(0, 0),
        false,
    )
    .unwrap();
    let (implicit, circle) = crate::arc_bezier::circular_conic_provenance(&arc);
    // C(s)=((1-s²)/(1+s²),2s/(1+s²)), s=4t³-6t²+3t.
    // This chart is injective, but ds/dt=3(2t-1)² vanishes at t=1/2.
    let bernstein = |coefficients: &[i32]| {
        crate::bezier_parameter::power_to_bernstein_coefficients(
            &coefficients
                .iter()
                .copied()
                .map(Real::from)
                .collect::<Vec<_>>(),
            6,
        )
        .unwrap()
    };
    let xs = bernstein(&[1, 0, -9, 36, -60, 48, -16]);
    let ys = bernstein(&[0, 6, -12, 8]);
    let weights = bernstein(&[1, 0, 9, -36, 60, -48, 16]);
    let controls = xs
        .iter()
        .zip(&ys)
        .zip(&weights)
        .map(|((x, y), weight)| Point2::new((x / weight).unwrap(), (y / weight).unwrap()))
        .collect();
    let source = RationalBezier2::try_new(controls, weights)
        .unwrap()
        .with_implicit_quadratic_conic(implicit, Some(circle));
    for source in [source.clone(), source.reversed()] {
        assert_eq!(source.degree(), 6);
        let parallel = source.parallel_left(q(1, 4)).unwrap();
        assert!(matches!(
            parallel.point_at_with_policy(&q(1, 2), &policy),
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        ));
        assert!(matches!(
            parallel.exact_rational_parallel_component(&policy).unwrap(),
            Classification::Decided(None)
        ));
    }
}

#[test]
fn circular_native_components_preserve_normal_side_and_contacts() {
    let policy = CurveContext::STRICT;
    let epsilon = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    for center in [
        Point2::from_values(0, 0),
        Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap()),
    ] {
        let source = circular_source(&center, 1);
        for reversed in [false, true] {
            let source = if reversed {
                source.reversed()
            } else {
                source.clone()
            };
            for distance in [q(-1, 4), q(1, 4), q(3, 2)] {
                let scale = Real::one() - &distance;
                let signed_distance = if reversed {
                    -&distance
                } else {
                    distance.clone()
                };
                let parallel = source.parallel_left(signed_distance.clone()).unwrap();
                let Classification::Decided(Some(component)) =
                    parallel.exact_rational_parallel_component(&policy).unwrap()
                else {
                    panic!("a regular quadratic circular chart has an exact component");
                };
                for parameter in [Real::zero(), q(1, 3), Real::one()] {
                    let point = source.point_at(&parameter).unwrap();
                    let (x, y) = point.delta_from(&center);
                    let expected = center.translated(x * &scale, y * &scale);
                    assert_eq!(
                        CurvePoint2::from(component.point_at(&parameter).unwrap())
                            .same_point(&expected.into(), &policy),
                        Classification::Decided(true)
                    );
                }
                let other = source.parallel_left(&signed_distance + &epsilon).unwrap();
                for (first, second) in [(&parallel, &other), (&other, &parallel)] {
                    let Classification::Decided(Some(result)) = first
                        .rational_unit_parallel_pair_intersections(second, None, &policy)
                        .unwrap()
                    else {
                        panic!("distinct concentric circular images must decide");
                    };
                    assert!(result.is_empty());
                }
                let Classification::Decided(Some(coincident)) = parallel
                    .rational_unit_parallel_pair_intersections(&parallel, None, &policy)
                    .unwrap()
                else {
                    panic!("coincident images must preserve their correspondence");
                };
                assert!(coincident.is_complete() && !coincident.overlaps().is_empty());
            }
        }
        let first = source.parallel_left(q(1, 4)).unwrap();
        for gap in [Real::zero(), epsilon.clone()] {
            let other_center = center.translated(q(3, 2) + &gap, Real::zero());
            let other = circular_source(&Point2::from_values(0, 0), 1)
                .transformed_affine([
                    &-Real::one(),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::one(),
                    other_center.x(),
                    other_center.y(),
                ])
                .parallel_left(q(-1, 4))
                .unwrap();
            for (first, second) in [(&first, &other), (&other, &first)] {
                let Classification::Decided(Some(result)) = first
                    .rational_unit_parallel_pair_intersections(second, None, &policy)
                    .unwrap()
                else {
                    panic!("tangent and disjoint finite circle pairs must decide");
                };
                assert!(result.is_complete() && result.overlaps().is_empty());
                if gap != Real::zero() {
                    assert!(result.contacts().is_empty());
                } else {
                    assert_eq!(result.contacts().len(), 1);
                    let contact = &result.contacts()[0];
                    assert_eq!(contact.first_parameter().scalar(), Some(&Real::zero()));
                    assert_eq!(contact.second_parameter().scalar(), Some(&Real::zero()));
                }
            }
        }
    }
}

#[test]
fn circular_parallel_incidence_keeps_points_order_and_normal_sheet() {
    let policy = CurveContext::STRICT;
    for center in [
        Point2::from_values(0, 0),
        Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap()),
    ] {
        for degree in [2, 5] {
            for weight in [1, -3] {
                let source = circular_source(&center, weight)
                    .elevated_to_degree(degree)
                    .unwrap();
                for reverse in [false, true] {
                    let source = if reverse {
                        source.reversed()
                    } else {
                        source.clone()
                    };
                    for distance in [Real::zero(), q(1, 2), q(3, 2)] {
                        let scale = Real::one() - &distance;
                        let distance = if reverse { -distance } else { distance };
                        let parallel = source.parallel_left(distance).unwrap();
                        let query_center = center.translated(scale.clone(), scale.clone());
                        let radius_squared = &scale * &scale;
                        let contacts = parallel
                            .retained_circular_source_incidence(
                                &query_center,
                                &radius_squared,
                                &CurveParameterRange2::unit(),
                                &policy,
                            )
                            .unwrap()
                            .expect("the circular chart must retain its exact intersection points");
                        assert_eq!(contacts.len(), 2);
                        for (index, contact) in contacts.iter().enumerate() {
                            let expected_parameter = Real::from(u8::try_from(index).unwrap());
                            assert_eq!(
                                contact
                                    .parameter
                                    .cmp_by_refinement_with_policy(
                                        &BezierParameter2::Exact(expected_parameter),
                                        &policy
                                    )
                                    .unwrap(),
                                Classification::Decided(std::cmp::Ordering::Equal)
                            );
                            let at_x_axis = (index == 0) ^ reverse;
                            let expected = if at_x_axis {
                                center.translated(scale.clone(), Real::zero())
                            } else {
                                center.translated(Real::zero(), scale.clone())
                            };
                            assert_eq!(contact.point.as_ref(), Some(&expected));
                            assert_eq!(
                                is_zero(
                                    &(expected.distance_squared(&center) - &radius_squared),
                                    &policy
                                ),
                                Some(true)
                            );
                            assert_eq!(
                                is_zero(
                                    &(expected.distance_squared(&query_center) - &radius_squared),
                                    &policy
                                ),
                                Some(true)
                            );
                            assert_eq!(
                                contact.radial_crossing_sign,
                                Some(if index == 0 {
                                    RealSign::Negative
                                } else {
                                    RealSign::Positive
                                })
                            );
                        }
                        let trim =
                            CurveParameterRange2::new_validated(q(1, 4).into(), q(3, 4).into());
                        assert!(
                            parallel
                                .retained_circular_source_incidence(
                                    &query_center,
                                    &radius_squared,
                                    &trim,
                                    &policy
                                )
                                .unwrap()
                                .unwrap()
                                .is_empty()
                        );
                        let tangent_center =
                            center.translated(Real::from(2) * &scale, Real::zero());
                        let tangent = parallel
                            .retained_circular_source_incidence(
                                &tangent_center,
                                &radius_squared,
                                &CurveParameterRange2::unit(),
                                &policy,
                            )
                            .unwrap()
                            .unwrap();
                        assert_eq!(tangent.len(), 1);
                        assert_eq!(
                            tangent[0].parameter.scalar(),
                            Some(&Real::from(u8::from(reverse)))
                        );
                        assert_eq!(tangent[0].radial_crossing_sign, Some(RealSign::Zero));
                        assert_eq!(
                            tangent[0].point,
                            Some(center.translated(scale.clone(), Real::zero()))
                        );
                        // A nearby but distinct circle is disjoint; no tolerance may merge it.
                        let exterior = center.translated(
                            (Real::from(2) + q(1, 1_000_000_000)) * &scale,
                            Real::zero(),
                        );
                        assert!(
                            parallel
                                .retained_circular_source_incidence(
                                    &exterior,
                                    &radius_squared,
                                    &CurveParameterRange2::unit(),
                                    &policy
                                )
                                .unwrap()
                                .unwrap()
                                .is_empty()
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn circular_parallel_recognition_defers_extensions_collapse_and_overlap() {
    let center = Point2::from_values(0, 0);
    let source = circular_source(&center, 1);
    let parallel = source.parallel_left(q(1, 2)).unwrap();
    let policy = CurveContext::STRICT;
    let extension = CurveParameterRange2::new_validated(Real::from(-1).into(), Real::one().into());
    assert!(
        parallel
            .retained_circular_source_incidence(
                &Point2::from_values(1, 0),
                &q(1, 4),
                &extension,
                &policy
            )
            .unwrap()
            .is_none()
    );
    assert!(
        parallel
            .retained_circular_source_incidence(
                &center,
                &q(1, 4),
                &CurveParameterRange2::unit(),
                &policy
            )
            .unwrap()
            .is_none()
    );
    assert!(
        parallel
            .with_distance(Real::one())
            .retained_circular_source_incidence(
                &Point2::from_values(1, 0),
                &Real::one(),
                &CurveParameterRange2::unit(),
                &policy
            )
            .unwrap()
            .is_none()
    );
}

#[test]
fn circular_parallel_internal_tangency_retains_the_original_parameter() {
    // The center distance is 85/16; after a 1/16 outward offset the
    // clockwise source radius is 94/16 = 85/16 + 9/16. The two circles
    // therefore touch internally at (9/16)*(13/85, -84/85).
    let center = Point2::new(q(-13, 16), q(21, 4));
    let radius = q(93, 16);
    let start = center.translated(radius.clone(), Real::zero());
    let end = center.translated(Real::zero(), -&radius);
    let arc =
        crate::CircularArc2::try_from_center(start.clone(), end.clone(), center.clone(), true)
            .unwrap();
    let (implicit, circle) = crate::arc_bezier::circular_conic_provenance(&arc);
    let source = RationalBezier2::try_new(
        vec![start, center.translated(radius.clone(), -radius), end],
        vec![Real::one(), Real::one(), Real::from(2)],
    )
    .unwrap()
    .with_implicit_quadratic_conic(implicit, Some(circle));
    let parallel = source.parallel_left(q(1, 16)).unwrap();
    let Classification::Decided(contacts) = parallel
        .circle_incidence_with_point_evidence(
            &Point2::from_values(0, 0),
            &q(81, 256),
            &CurveParameterRange2::unit(),
            &[],
            &CurveContext::STRICT,
        )
        .unwrap()
    else {
        panic!("internal tangency must retain its represented point");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].parameter.scalar(), Some(&q(6, 7)));
    assert_eq!(
        contacts[0].point,
        Some(Point2::new(q(117, 1360), q(-189, 340)))
    );
    assert_eq!(contacts[0].radial_crossing_sign, Some(RealSign::Zero));
    assert!(matches!(
        parallel
            .circle_incidence_with_point_evidence(
                &Point2::from_values(0, 0),
                &q(81, 256),
                &CurveParameterRange2::unit(),
                &[(q(6, 7), 0)],
                &CurveContext::STRICT,
            )
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    ));
}

#[test]
fn circular_parallel_incidence_certifies_the_selected_trim_boundary() {
    let parallel = circular_source(&Point2::from_values(0, 0), 1)
        .parallel_left(q(1, 2))
        .unwrap();
    let parameter = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                Real::from(-1),
                Real::zero(),
                Real::from(3),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), q(2, 3)).unwrap(),
        )
        .unwrap(),
    );
    let range = CurveParameterRange2::new_validated(parameter.into(), Real::one().into());
    // Two circles of radius 1/2, with centers (0,0) and (1/2,0), meet
    // at (1/4, +/-sqrt(3)/4). The first-quadrant chart retains only the
    // positive point, at t=1/sqrt(3), exactly the selected trim boundary.
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let query = || {
            parallel
                .circle_incidence_with_point_evidence(
                    &Point2::new(q(1, 2), Real::zero()),
                    &q(1, 4),
                    &range,
                    &[],
                    &policy,
                )
                .unwrap()
        };
        let retry = crate::evaluate_with_budget(
            policy.predicate_policy(),
            crate::WorkBudget {
                refinements: 4096,
                promotions: 128,
            },
            query,
        );
        let Classification::Decided(contacts) = retry.result.unwrap().certified().unwrap() else {
            panic!("the selected trim boundary must retain its exact contact");
        };
        assert_eq!(contacts.len(), 1);
        let point = contacts[0].point.as_ref().unwrap();
        assert_eq!(point.x(), &q(1, 4));
        assert_eq!(
            is_zero(&(point.y() * point.y() - q(3, 16)), &CurveContext::STRICT),
            Some(true)
        );
        assert_eq!(
            real_sign(point.y(), &CurveContext::STRICT),
            Some(RealSign::Positive)
        );
        assert_eq!(contacts[0].radial_crossing_sign, Some(RealSign::Positive));
    }
}

#[test]
fn circular_parallel_incidence_retries_after_exhausted_trim_comparison() {
    let policy = CurveContext::STRICT;
    let parallel = circular_source(&Point2::from_values(0, 0), 1)
        .parallel_left(q(1, 2))
        .unwrap();
    let parameter = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                Real::from(-1),
                Real::zero(),
                Real::from(2),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(2, 3), q(3, 4)).unwrap(),
        )
        .unwrap(),
    );
    let range = CurveParameterRange2::new_validated(parameter.into(), Real::one().into());
    let t = q(353_553, 500_000);
    let square = &t * &t;
    let point = Point2::new(
        ((Real::one() - &square) / (Real::from(2) * (Real::one() + &square))).unwrap(),
        (&t / (Real::one() + &square)).unwrap(),
    );
    let center = Point2::new(Real::from(2) * point.x(), Real::from(2) * point.y());
    // The second radius-1/2 circle touches the parallel at this rational
    // parameter, just before the independently isolated sqrt(1/2) trim.
    assert_eq!(
        real_sign(&(square - q(1, 2)), &policy),
        Some(RealSign::Negative)
    );
    let query = || {
        parallel
            .circle_incidence_with_point_evidence(&center, &q(1, 4), &range, &[], &policy)
            .unwrap()
    };
    let denied =
        crate::evaluate_with_budget(policy.predicate_policy(), crate::WorkBudget::ZERO, query);
    let Err(exhausted) = denied.result else {
        panic!("the unresolved trim comparison requires a refinement");
    };
    assert_eq!(exhausted.kind, crate::WorkKind::RootRefinement);
    let retry = crate::evaluate_with_budget(
        policy.predicate_policy(),
        crate::WorkBudget {
            refinements: 4096,
            promotions: 128,
        },
        query,
    );
    let Classification::Decided(contacts) = retry.result.unwrap().certified().unwrap() else {
        panic!("an exhausted trim comparison must not poison its retry");
    };
    assert!(contacts.is_empty());
}
