//! Kernel tests for authored rational Bezier line and pair contacts.

use crate::{
    BezierLineContactKind, BezierLineContactRelation, BezierParameter2, Classification,
    CubicBezier2, Curve2, CurveContext, CurveIntersectionCandidates2, CurveOverlapOrientation2,
    CurvePoint2, LineSeg2, ParamRange, Point2, QuadraticBezier2, RationalBezier2,
    RationalBezierIntersectionContacts2, RationalQuadraticBezier2, Real,
};
use hyperreal::Rational;
use num::{BigInt, BigUint};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn unresolved_positive() -> Real {
    let tiny = Real::new(
        Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(1_u8) << 5000).unwrap(),
    );
    (Real::pi() + tiny) - Real::pi()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

/// Local parameters locating `point`, or `None` when the whole curve maps to it.
/// `None` entries are selected algebraic parameters without a scalar payload.
fn point_parameters(
    curve: &RationalBezier2,
    point: &Point2,
    policy: &CurveContext,
) -> Option<Vec<Option<Real>>> {
    match Curve2::from(curve.clone())
        .point_locations_with_policy(&CurvePoint2::from(point.clone()), policy)
        .unwrap()
        .value
    {
        crate::CurvePointLocations2::EntireCurve => None,
        crate::CurvePointLocations2::Locations(locations) => Some(
            locations
                .iter()
                .map(|location| location.local_parameter().scalar().cloned())
                .collect(),
        ),
    }
}

fn curve() -> RationalBezier2 {
    RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .unwrap()
}

#[test]
fn independent_quadratic_tail_overlap_retains_one_exact_interval() {
    // Q(t)=(t^2,2t). The unit-distance cut has s^2=sqrt(5)-2;
    // Q(s+(1-s)u) has the independently constructed controls below.
    let s_squared = r(5).sqrt().unwrap() - r(2);
    let s = s_squared.clone().sqrt().unwrap();
    for swap_axes in [false, true] {
        let point = |x: Real, y: Real| {
            if swap_axes {
                Point2::new(y, x)
            } else {
                Point2::new(x, y)
            }
        };
        let source = RationalBezier2::try_new(
            vec![point(r(0), r(0)), point(r(0), r(1)), point(r(1), r(2))],
            vec![r(1); 3],
        )
        .unwrap();
        let tail = RationalBezier2::try_new(
            vec![
                point(s_squared.clone(), r(2) * &s),
                point(s.clone(), r(1) + &s),
                point(r(1), r(2)),
            ],
            vec![r(1); 3],
        )
        .unwrap();
        for reversed in [false, true] {
            let tail = if reversed {
                tail.reversed()
            } else {
                tail.clone()
            };
            for swap_curves in [false, true] {
                let (first, second) = if swap_curves {
                    (&tail, &source)
                } else {
                    (&source, &tail)
                };
                let (first_expected, second_expected) = if swap_curves {
                    (
                        [r(0), r(1)],
                        if reversed {
                            [r(1), s.clone()]
                        } else {
                            [s.clone(), r(1)]
                        },
                    )
                } else {
                    (
                        [s.clone(), r(1)],
                        if reversed { [r(1), r(0)] } else { [r(0), r(1)] },
                    )
                };
                for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                    let RationalBezierIntersectionContacts2::Overlap(overlap) =
                        first.intersection_contacts(second, &policy).unwrap()
                    else {
                        panic!("one shared parabola interval must replay exactly");
                    };
                    assert_eq!(
                        overlap.orientation(),
                        if reversed {
                            CurveOverlapOrientation2::Reversed
                        } else {
                            CurveOverlapOrientation2::Same
                        }
                    );
                    for (range, expected) in [
                        (overlap.first_range(), &first_expected),
                        (overlap.second_range(), &second_expected),
                    ] {
                        for (actual, expected) in
                            [(range.start(), &expected[0]), (range.end(), &expected[1])]
                        {
                            assert_eq!(
                                actual
                                    .cmp_by_refinement(
                                        &BezierParameter2::Exact(expected.clone()),
                                        &CurveContext::STRICT
                                    )
                                    .unwrap(),
                                Classification::Decided(std::cmp::Ordering::Equal)
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn general_rational_line_contact_retains_exact_parameter_and_kind() {
    let curve = curve();
    let policy = CurveContext::STRICT;
    let line =
        LineSeg2::try_new(Point2::new(q(49, 20), r(-1)), Point2::new(q(49, 20), r(1))).unwrap();

    let relation = decided(curve.relation_to_line_with_contacts(&line, &policy));
    let BezierLineContactRelation::Contacts { contacts } = relation else {
        panic!("represented rational line root was not materialized");
    };
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].parameter(), &q(1, 2));
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);
}

#[test]
fn general_rational_line_contact_retains_irrational_crossing_parameter() {
    let curve = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1)],
        vec![r(1); 3],
    )
    .unwrap();
    let line = LineSeg2::try_new(Point2::new(r(-1), q(1, 2)), Point2::new(r(2), q(1, 2))).unwrap();

    let relation = decided(curve.relation_to_line_with_contacts(&line, &CurveContext::STRICT));
    let BezierLineContactRelation::Contacts { contacts } = relation else {
        panic!("irrational rational-Bezier line root was not retained");
    };
    assert_eq!(contacts.len(), 1);
    assert!(matches!(
        contacts[0].parameter(),
        BezierParameter2::Algebraic(_)
    ));
    assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);
}

#[test]
fn every_bezier_family_retains_irrational_line_contacts() {
    let policy = CurveContext::STRICT;
    let horizontal_half =
        LineSeg2::try_new(Point2::new(r(-1), q(1, 2)), Point2::new(r(2), q(1, 2))).unwrap();
    let quadratic = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1));
    let cubic = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), r(0)),
        Point2::new(q(2, 3), r(0)),
        p(1, 1),
    );
    let rational = RationalQuadraticBezier2::try_new(
        p(0, 0),
        Point2::new(q(1, 2), r(0)),
        p(1, 1),
        r(1),
        r(2),
        r(1),
    )
    .unwrap();

    for relation in [
        quadratic.relation_to_line_with_contacts(&horizontal_half, &policy),
        cubic.relation_to_line_with_contacts(&horizontal_half, &policy),
        rational.relation_to_line_with_contacts(&horizontal_half, &policy),
    ] {
        let BezierLineContactRelation::Contacts { contacts } = decided(relation) else {
            panic!("Bezier family did not retain its irrational line root");
        };
        assert_eq!(contacts.len(), 1);
        assert!(matches!(
            contacts[0].parameter(),
            BezierParameter2::Algebraic(_)
        ));
        assert_eq!(contacts[0].kind(), BezierLineContactKind::Crossing);
    }
}

#[test]
fn exact_line_contact_solver_distinguishes_hull_overlap_from_curve_contact() {
    // y(t) = t^2 - t + 1/3 is strictly positive, although its middle
    // Bernstein control lies below y = 0.
    let curve = QuadraticBezier2::new(
        Point2::new(r(0), q(1, 3)),
        Point2::new(q(1, 2), q(-1, 6)),
        Point2::new(r(1), q(1, 3)),
    );
    let axis = LineSeg2::try_new(p(-1, 0), p(2, 0)).unwrap();

    assert_eq!(
        curve.relation_to_line_with_contacts(&axis, &CurveContext::STRICT),
        Classification::Decided(BezierLineContactRelation::NoContact)
    );
}

#[test]
fn general_rational_line_relation_certifies_hull_miss_and_coincidence() {
    let policy = CurveContext::STRICT;
    let below = LineSeg2::try_new(p(0, -1), p(4, -1)).unwrap();
    assert!(matches!(
        curve().relation_to_line_with_contacts(&below, &policy),
        Classification::Decided(BezierLineContactRelation::ControlHullDisjoint { .. })
    ));

    let collinear = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 0), p(3, 0), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .unwrap();
    let axis = LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap();
    assert_eq!(
        collinear.relation_to_line_with_contacts(&axis, &policy),
        Classification::Decided(BezierLineContactRelation::OnSupportingLine)
    );
}

#[test]
fn general_rational_contacts_recognize_projective_scale_and_reversal() {
    let curve = curve();
    let policy = CurveContext::STRICT;
    let scaled = RationalBezier2::try_new(
        curve.affine_control_points().unwrap().to_vec(),
        vec![r(2), r(4), r(6), r(8)],
    )
    .unwrap();

    for (other, orientation, second_range) in [
        (
            curve.clone(),
            CurveOverlapOrientation2::Same,
            ParamRange::new(r(0), r(1)),
        ),
        (
            scaled,
            CurveOverlapOrientation2::Same,
            ParamRange::new(r(0), r(1)),
        ),
        (
            curve.reversed(),
            CurveOverlapOrientation2::Reversed,
            ParamRange::new(r(1), r(0)),
        ),
    ] {
        let RationalBezierIntersectionContacts2::Overlap(overlap) =
            curve.intersection_contacts(&other, &policy).unwrap()
        else {
            panic!("projectively equivalent curve did not retain overlap evidence");
        };
        assert_eq!(overlap.first_range(), &ParamRange::new(r(0), r(1)));
        assert_eq!(overlap.second_range(), &second_range);
        assert_eq!(overlap.orientation(), orientation);
    }
}

#[test]
fn general_rational_contacts_reject_disjoint_control_hulls() {
    let policy = CurveContext::STRICT;
    let shifted = RationalBezier2::try_new(
        vec![p(10, 0), p(11, 3), p(13, 3), p(14, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .unwrap();

    assert_eq!(
        curve().intersection_contacts(&shifted, &policy).unwrap(),
        RationalBezierIntersectionContacts2::NoIntersection
    );
}

#[test]
fn direct_disjoint_conic_cubic_reports_no_candidates_or_contacts() {
    let policy = CurveContext::STRICT;
    let conic =
        RationalBezier2::try_new(vec![p(1, 0), p(1, 1), p(0, 1)], vec![r(1), r(1), r(2)]).unwrap();
    let disjoint_cubic =
        RationalBezier2::try_new(vec![p(10, 0), p(11, 1), p(11, 2), p(10, 3)], vec![r(1); 4])
            .unwrap();

    assert_eq!(
        conic
            .intersection_candidates(&disjoint_cubic, &policy)
            .unwrap(),
        CurveIntersectionCandidates2::NoIntersection
    );
    assert_eq!(
        conic
            .intersection_contacts(&disjoint_cubic, &policy)
            .unwrap(),
        RationalBezierIntersectionContacts2::NoIntersection
    );
}

#[test]
fn implicit_conic_route_replays_degree_elevated_line_contact_in_both_orders() {
    let policy = CurveContext::STRICT;
    let conic =
        RationalBezier2::try_new(vec![p(1, 0), p(1, 1), p(0, 1)], vec![r(1), r(1), r(2)]).unwrap();
    let cubic_line = RationalBezier2::try_new(
        vec![
            Point2::new(q(3, 5), r(-1)),
            Point2::new(q(3, 5), r(0)),
            Point2::new(q(3, 5), r(1)),
            Point2::new(q(3, 5), r(2)),
        ],
        vec![r(1); 4],
    )
    .unwrap();

    let RationalBezierIntersectionContacts2::Contacts(contacts) =
        conic.intersection_contacts(&cubic_line, &policy).unwrap()
    else {
        panic!("implicit conic route did not retain its exact contact");
    };
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].is_certified_transverse());
    assert_eq!(contacts[0].first_parameter().scalar(), Some(&q(1, 2)));
    assert_eq!(contacts[0].second_parameter().scalar(), Some(&q(3, 5)));
    assert!(
        matches!((contacts[0].point()).coordinates(), Some(point) if point == &Point2::new(q(3, 5), q(4, 5)))
    );

    let RationalBezierIntersectionContacts2::Contacts(reversed) =
        cubic_line.intersection_contacts(&conic, &policy).unwrap()
    else {
        panic!("reversed implicit conic route did not retain its exact contact");
    };
    assert_eq!(reversed.len(), 1);
    assert!(reversed[0].is_certified_transverse());
    assert_eq!(
        reversed[0].first_parameter(),
        contacts[0].second_parameter()
    );
    assert_eq!(
        reversed[0].second_parameter(),
        contacts[0].first_parameter()
    );
}

#[test]
fn pi_weight_conic_replays_degree_elevated_horizontal_contact() {
    let policy = CurveContext::STRICT;
    let conic = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1)],
        vec![Real::one(), Real::pi(), Real::one()],
    )
    .unwrap();
    let cubic_line = RationalBezier2::try_new(
        vec![
            Point2::new(r(0), q(1, 2)),
            Point2::new(q(1, 3), q(1, 2)),
            Point2::new(q(2, 3), q(1, 2)),
            Point2::new(r(1), q(1, 2)),
        ],
        vec![Real::one(); 4],
    )
    .unwrap();

    let contacts = conic
        .intersection_contacts(&cubic_line, &policy)
        .expect("pi-weight conic contact should remain exact");

    let RationalBezierIntersectionContacts2::Contacts(contacts) = &contacts else {
        panic!("pi-weight conic contact did not retain its isolated evidence");
    };
    assert_eq!(contacts.len(), 1);
    let point = contacts[0].point();
    assert!(point.coordinates().is_none());
    let expected_height = CurvePoint2::from(Point2::new(Real::zero(), q(1, 2)));
    let height_order = point
        .compare_coordinate(&expected_height, crate::Axis2::Y, &policy)
        .expect("selected contact height remains exactly comparable");
    assert_eq!(height_order.certainty, crate::CurveCertainty::Certified);
    assert_eq!(
        height_order.value,
        Classification::Decided(std::cmp::Ordering::Equal)
    );

    let reversed = cubic_line
        .intersection_contacts(&conic, &policy)
        .expect("reversed pi-weight conic contact should remain exact");
    assert!(matches!(
        reversed,
        RationalBezierIntersectionContacts2::Contacts(ref contacts) if contacts.len() == 1
    ));

    let candidates = conic
        .intersection_candidates(&cubic_line, &policy)
        .expect("pi-weight conic candidates should remain exact");
    assert!(!matches!(
        candidates,
        CurveIntersectionCandidates2::NoIntersection
    ));

    let contacts = |policy: &CurveContext| match conic
        .intersection_contacts(&cubic_line, policy)
        .expect("pi-weight conic contacts should remain exact")
    {
        RationalBezierIntersectionContacts2::Contacts(contacts) => contacts,
        other => panic!("pi-weight conic contacts should be isolated: {other:?}"),
    };
    let topology = contacts(&policy);
    assert_eq!(topology.len(), 1);
    let approximate = contacts(&CurveContext::APPROXIMATE_512);
    assert_eq!(approximate.len(), 1);
    for policy in [policy, CurveContext::APPROXIMATE_512] {
        let pieces = Curve2::from(conic.clone())
            .intersection_topology_with_policy(&Curve2::from(cubic_line.clone()), &policy)
            .expect("the pi-weight conic topology should remain exact")
            .into_value();
        assert_eq!(pieces.first().len() + pieces.second().len(), 4);
    }
    assert_eq!(
        approximate[0]
            .first_parameter()
            .cmp_by_refinement(
                topology[0].first_parameter(),
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        approximate[0]
            .second_parameter()
            .cmp_by_refinement(
                topology[0].second_parameter(),
                &CurveContext::APPROXIMATE_512,
            )
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Equal)
    );
}

#[test]
fn implicit_conic_route_retains_an_interior_rational_quadratic_cubic_contact() {
    let policy = CurveContext::STRICT;
    let conic = RationalBezier2::try_new(vec![p(5, 6), p(14, 5), p(23, 6)], vec![r(1), r(2), r(1)])
        .unwrap();
    let cubic = RationalBezier2::try_new(
        vec![
            p(28, 4),
            Point2::new(q(61, 3), r(8)),
            Point2::new(q(38, 3), r(8)),
            p(5, 4),
        ],
        vec![r(1); 4],
    )
    .unwrap();

    let result = conic.intersection_contacts(&cubic, &policy).unwrap();
    let RationalBezierIntersectionContacts2::Contacts(contacts) = &result else {
        let candidates = conic.intersection_candidates(&cubic, &policy).unwrap();
        panic!(
            "implicit conic route discarded its interior contact: {result:#?}; candidates: {candidates:#?}"
        );
    };
    assert_eq!(contacts.len(), 1);
}

#[test]
fn resultant_replay_retains_an_interior_nonuniform_rational_cubic_contact() {
    let policy = CurveContext::STRICT;
    let first = RationalBezier2::try_new(
        vec![p(5, 6), p(11, 5), p(17, 5), p(23, 6)],
        vec![r(1), r(2), r(2), r(1)],
    )
    .unwrap();
    let second = RationalBezier2::try_new(
        vec![
            p(28, 4),
            Point2::new(q(61, 3), r(8)),
            Point2::new(q(38, 3), r(8)),
            p(5, 4),
        ],
        vec![r(1), r(4), r(4), r(1)],
    )
    .unwrap();

    let result = first.intersection_contacts(&second, &policy).unwrap();
    let RationalBezierIntersectionContacts2::Contacts(contacts) = &result else {
        panic!("resultant replay discarded its interior contact: {result:#?}");
    };
    assert_eq!(contacts.len(), 1);
}

#[test]
fn polynomial_graph_replay_accepts_unequal_resultant_projection_counts() {
    let policy = CurveContext::STRICT;
    let first = RationalBezier2::try_new(
        vec![p(5, 6), p(11, 5), p(17, 5), p(23, 6)],
        vec![r(1), r(2), r(2), r(1)],
    )
    .unwrap();
    let second = RationalBezier2::try_new(
        vec![
            p(28, 4),
            Point2::new(q(61, 3), r(8)),
            Point2::new(q(38, 3), r(8)),
            p(5, 4),
        ],
        vec![r(1); 4],
    )
    .unwrap();

    let result = first.intersection_contacts(&second, &policy).unwrap();
    let RationalBezierIntersectionContacts2::Contacts(contacts) = &result else {
        panic!("polynomial-graph replay discarded its interior contact: {result:#?}");
    };
    assert_eq!(contacts.len(), 1);
}

#[test]
fn implicit_conic_route_does_not_certify_a_tangent_root_as_transverse() {
    let policy = CurveContext::STRICT;
    let conic =
        RationalBezier2::try_new(vec![p(1, 0), p(1, 1), p(0, 1)], vec![r(1), r(1), r(2)]).unwrap();
    let tangent_line =
        RationalBezier2::try_new(vec![p(1, -1), p(1, 0), p(1, 1), p(1, 2)], vec![r(1); 4]).unwrap();

    let RationalBezierIntersectionContacts2::Contacts(contacts) =
        conic.intersection_contacts(&tangent_line, &policy).unwrap()
    else {
        panic!("implicit conic route did not retain its tangent contact");
    };
    assert_eq!(contacts.len(), 1);
    assert!(!contacts[0].is_certified_transverse());
}

#[test]
fn shared_cancellation_resolves_disjoint_rational_contact_blocker() {
    let first = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![unresolved_positive(), r(1)])
        .unwrap();
    let second = RationalBezier2::try_new(vec![p(3, 0), p(4, 1)], vec![r(1), r(1)]).unwrap();

    assert!(matches!(
        first
            .intersection_contacts(&second, &CurveContext::STRICT)
            .unwrap(),
        RationalBezierIntersectionContacts2::NoIntersection
    ));
}

#[test]
fn rational_resultant_certifies_disjoint_and_represented_crossing_parameters() {
    let policy = CurveContext::STRICT;
    let rising = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![r(1), r(1)]).unwrap();
    let falling = RationalBezier2::try_new(vec![p(0, 1), p(1, 0)], vec![r(1), r(1)]).unwrap();
    let crossing = rising.intersection_candidates(&falling, &policy).unwrap();
    let CurveIntersectionCandidates2::Candidates {
        first_parameters,
        second_parameters,
    } = crossing
    else {
        panic!("crossing lines did not retain resultant candidates");
    };
    assert_eq!(first_parameters, vec![BezierParameter2::Exact(q(1, 2))]);
    assert_eq!(second_parameters, vec![BezierParameter2::Exact(q(1, 2))]);
    let contacts = rising.intersection_contacts(&falling, &policy).unwrap();
    let RationalBezierIntersectionContacts2::Contacts(contacts) = contacts else {
        panic!("represented crossing candidates did not replay");
    };
    assert_eq!(contacts.len(), 1);
    assert!(
        matches!((contacts[0].point()).coordinates(), Some(point) if point == &Point2::new(q(1, 2), q(1, 2)))
    );
    let above = RationalBezier2::try_new(vec![p(0, 2), p(1, 2)], vec![r(1), r(1)]).unwrap();
    assert_eq!(
        rising.intersection_candidates(&above, &policy).unwrap(),
        CurveIntersectionCandidates2::NoIntersection
    );
}

#[test]
fn rational_contacts_replay_represented_resultant_candidates() {
    let policy = CurveContext::STRICT;
    let parabola = RationalBezier2::try_new(
        vec![Point2::new(r(0), r(0)), Point2::new(q(1, 2), r(0)), p(1, 1)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();
    let horizontal = RationalBezier2::try_new(
        vec![Point2::new(r(0), q(1, 4)), Point2::new(r(1), q(1, 4))],
        vec![r(1), r(1)],
    )
    .unwrap();
    let contacts = parabola
        .intersection_contacts(&horizontal, &policy)
        .unwrap();
    let RationalBezierIntersectionContacts2::Contacts(contacts) = contacts else {
        panic!("represented resultant candidates were not replayed");
    };
    assert_eq!(contacts.len(), 1);
    assert!(
        matches!((contacts[0].point()).coordinates(), Some(point) if point == &Point2::new(q(1, 2), q(1, 4)))
    );
}

#[test]
fn rational_resultant_replays_identical_and_reversed_full_image_overlap() {
    let policy = CurveContext::STRICT;
    let curve = curve();
    assert_eq!(
        curve
            .intersection_candidates(&curve.clone(), &policy)
            .unwrap(),
        CurveIntersectionCandidates2::DegenerateResultant
    );
    let RationalBezierIntersectionContacts2::Overlap(overlap) = curve
        .intersection_contacts(&curve.clone(), &policy)
        .unwrap()
    else {
        panic!("identical curve did not retain certified overlap");
    };
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
    assert_eq!(
        overlap.first_range(),
        &ParamRange::new(Real::zero(), Real::one())
    );
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::zero(), Real::one())
    );
    let RationalBezierIntersectionContacts2::Overlap(overlap) = curve
        .intersection_contacts(&curve.reversed(), &policy)
        .unwrap()
    else {
        panic!("reversed curve did not retain certified overlap");
    };
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Reversed);
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::one(), Real::zero())
    );
}

#[test]
fn projectively_reparameterized_rational_quadratic_certifies_shared_conic() {
    let weight = (r(2).sqrt().unwrap() / r(2)).unwrap();
    let first = RationalBezier2::try_new(
        vec![p(1, 0), p(1, 1), p(0, 1)],
        vec![r(1), weight.clone(), r(1)],
    )
    .unwrap();
    let second = RationalBezier2::try_new(
        vec![p(1, 0), p(1, 1), p(0, 1)],
        vec![r(1), r(2) * weight, r(4)],
    )
    .unwrap();
    let contacts = first
        .intersection_contacts(&second, &CurveContext::STRICT)
        .unwrap();
    let RationalBezierIntersectionContacts2::Overlap(overlap) = contacts else {
        panic!("projectively reparameterized conic remained unresolved: {contacts:?}");
    };
    assert_eq!(
        overlap.first_range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(
        overlap.second_range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);

    let RationalBezierIntersectionContacts2::Overlap(reversed) = first
        .intersection_contacts(&second.reversed(), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("reversed projective conic did not retain overlap");
    };
    assert_eq!(reversed.orientation(), CurveOverlapOrientation2::Reversed);
    assert_eq!(
        reversed.second_range().scalar_endpoints(),
        Some((&Real::one(), &Real::zero()))
    );
}

#[test]
fn independently_trimmed_projective_conics_retain_partial_overlap() {
    let policy = CurveContext::STRICT;
    let weight = (r(2).sqrt().unwrap() / r(2)).unwrap();
    let controls = vec![p(1, 0), p(1, 1), p(0, 1)];
    let first =
        RationalBezier2::try_new(controls.clone(), vec![r(1), weight.clone(), r(1)]).unwrap();
    let second = RationalBezier2::try_new(controls, vec![r(1), r(2) * weight, r(4)]).unwrap();
    let first = decided(
        first
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    let second = decided(
        second
            .subcurve_between_exact(&q(1, 4), &Real::one(), &policy)
            .unwrap(),
    );

    let RationalBezierIntersectionContacts2::Overlap(overlap) =
        first.intersection_contacts(&second, &policy).unwrap()
    else {
        panic!("independently trimmed projective conics did not retain overlap");
    };
    let (first_start, first_end) = overlap.first_range().scalar_endpoints().unwrap();
    assert!(matches!(
        BezierParameter2::Exact(first_start.clone())
            .cmp_by_interval(&BezierParameter2::Exact(Real::zero()), &policy)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Greater)
    ));
    assert_eq!(first_end, &Real::one());
    let (second_start, second_end) = overlap.second_range().scalar_endpoints().unwrap();
    assert_eq!(second_start, &Real::zero());
    assert!(matches!(
        BezierParameter2::Exact(second_end.clone())
            .cmp_by_interval(&BezierParameter2::Exact(Real::one()), &policy)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Less)
    ));
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
}

#[test]
fn rational_resultant_certifies_exact_partial_nonlinear_overlap_ranges() {
    let policy = CurveContext::STRICT;
    let source = curve();
    let first = decided(
        source
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    let second = decided(
        source
            .subcurve_between_exact(&q(3, 8), &q(7, 8), &policy)
            .unwrap(),
    );
    assert_eq!(
        point_parameters(&first, second.start(), &policy),
        Some(vec![Some(q(1, 2))])
    );
    assert_eq!(
        point_parameters(&second, first.end(), &policy),
        Some(vec![Some(q(3, 4))])
    );
    let first_overlap = decided(
        first
            .subcurve_between_exact(&q(1, 2), &Real::one(), &policy)
            .unwrap(),
    );
    let second_overlap = decided(
        second
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    assert!(matches!(
        first_overlap
            .intersection_contacts(&second_overlap, &policy)
            .unwrap(),
        RationalBezierIntersectionContacts2::Overlap(_)
    ));

    let contacts = first.intersection_contacts(&second, &policy).unwrap();
    let RationalBezierIntersectionContacts2::Overlap(overlap) = contacts else {
        panic!("partial nonlinear shared image did not retain certified overlap: {contacts:?}");
    };
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
    assert_eq!(
        overlap.first_range(),
        &ParamRange::new(q(1, 2), Real::one())
    );
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::zero(), q(3, 4))
    );

    let reversed = second.reversed();
    let RationalBezierIntersectionContacts2::Overlap(overlap) =
        first.intersection_contacts(&reversed, &policy).unwrap()
    else {
        panic!("reversed partial nonlinear shared image did not retain certified overlap");
    };
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Reversed);
    assert_eq!(
        overlap.first_range(),
        &ParamRange::new(q(1, 2), Real::one())
    );
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::one(), q(1, 4))
    );
}
#[test]
fn independently_constructed_partial_overlap_reconstructs_rational_endpoints() {
    let policy = CurveContext::STRICT;
    let source = curve();
    let source_first = decided(
        source
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    let source_second = decided(
        source
            .subcurve_between_exact(&q(1, 4), &Real::one(), &policy)
            .unwrap(),
    );
    let first = decided(
        RationalBezier2::from_homogeneous_controls(
            source_first.homogeneous_controls().to_vec(),
            &policy,
        )
        .unwrap(),
    );
    let second = decided(
        RationalBezier2::from_homogeneous_controls(
            source_second.homogeneous_controls().to_vec(),
            &policy,
        )
        .unwrap(),
    );

    let RationalBezierIntersectionContacts2::Overlap(overlap) =
        first.intersection_contacts(&second, &policy).unwrap()
    else {
        panic!("independently reconstructed shared image did not certify overlap");
    };
    assert_eq!(
        overlap.first_range(),
        &ParamRange::new(q(1, 3), Real::one())
    );
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::zero(), q(2, 3))
    );
}

#[test]
fn line_image_overlap_retains_irrational_algebraic_parameter_boundary() {
    let policy = CurveContext::STRICT;
    let quadratic_parameterization = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 4), r(0)), p(1, 0)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();
    let partial_line =
        RationalBezier2::try_new(vec![Point2::new(q(1, 2), r(0)), p(1, 0)], vec![r(1), r(1)])
            .unwrap();

    let RationalBezierIntersectionContacts2::Overlap(overlap) = quadratic_parameterization
        .intersection_contacts(&partial_line, &policy)
        .unwrap()
    else {
        panic!("certified line images did not retain their algebraic overlap range");
    };
    assert!(matches!(
        overlap.first_range().start(),
        BezierParameter2::Algebraic(_)
    ));
    assert_eq!(overlap.first_range().end().scalar(), Some(&Real::one()));
    assert_eq!(
        overlap.second_range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
}

#[test]
fn line_image_overlap_accepts_monotone_parameterization_with_stationary_point() {
    let policy = CurveContext::STRICT;
    let stationary_monotone =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(0, 0), p(1, 0)], vec![r(1); 4]).unwrap();
    let upper_half =
        RationalBezier2::try_new(vec![Point2::new(q(1, 2), r(0)), p(1, 0)], vec![r(1), r(1)])
            .unwrap();

    let RationalBezierIntersectionContacts2::Overlap(overlap) = stationary_monotone
        .intersection_contacts(&upper_half, &policy)
        .unwrap()
    else {
        panic!("stationary monotone line image did not retain its overlap");
    };
    assert_eq!(
        overlap.first_range().scalar_endpoints(),
        Some((&q(1, 2), &Real::one()))
    );
    assert_eq!(
        overlap.second_range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
}

#[test]
fn polynomial_graph_overlap_retains_irrational_curved_boundary() {
    let policy = CurveContext::STRICT;
    let partial_parabola = RationalBezier2::try_new(
        vec![
            Point2::new(q(1, 2), q(1, 4)),
            Point2::new(q(3, 4), q(1, 2)),
            p(1, 1),
        ],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();
    let nonlinear_parameterization = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 8), r(0)),
            Point2::new(q(1, 3), q(1, 24)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![r(1); 5],
    )
    .unwrap();

    assert_eq!(
        partial_parabola
            .intersection_candidates(&nonlinear_parameterization, &policy)
            .unwrap(),
        CurveIntersectionCandidates2::DegenerateResultant
    );
    let RationalBezierIntersectionContacts2::Overlap(overlap) = partial_parabola
        .intersection_contacts(&nonlinear_parameterization, &policy)
        .unwrap()
    else {
        panic!("certified polynomial graph did not retain its curved overlap");
    };
    assert_eq!(
        overlap.first_range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert!(matches!(
        overlap.second_range().start(),
        BezierParameter2::Algebraic(_)
    ));
    assert_eq!(overlap.second_range().end().scalar(), Some(&Real::one()));
    assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Same);
}
