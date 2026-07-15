#[cfg(feature = "predicates")]
use hypercurve::BezierAlgebraicImageStatus;
use hypercurve::{
    Axis2, BezierLineContactKind, BezierLineContactRelation, BezierParameter2,
    BezierSplitFragment2, BezierSubcurve2, Classification, CubicBezier2, Curve2, CurveFamily2,
    CurveOperation2, CurvePolicy, ExactCurveError, LineSeg2, ParamRange, Point2, QuadraticBezier2,
    RationalBezier2, RationalBezierIntersectionCandidates2, RationalBezierIntersectionContacts2,
    RationalBezierIntersectionPointEvidence2, RationalBezierOverlapOrientation2,
    RationalBezierPointIncidence2, RationalQuadraticBezier2, Real, UncertaintyReason,
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

fn curve() -> RationalBezier2 {
    RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .unwrap()
}

#[test]
fn general_rational_cubic_evaluates_exactly() {
    let curve = curve();
    let policy = CurvePolicy::certified();
    let half = q(1, 2);

    assert_eq!(curve.point_at(&r(0), &policy).unwrap(), p(0, 0));
    assert_eq!(curve.point_at(&r(1), &policy).unwrap(), p(4, 0));
    assert_eq!(
        curve.point_at(&half, &policy).unwrap(),
        Point2::new(q(49, 20), q(9, 4))
    );
}

#[test]
fn general_rational_derivative_is_exact_and_reuses_power_basis() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(4, 0)], vec![r(1), r(3)]).unwrap();
    let clone = curve.clone();
    let policy = CurvePolicy::certified();

    assert!(!curve.is_homogeneous_power_basis_cached());
    let derivative = curve.derivative_at(&q(1, 2), &policy).unwrap();
    assert_eq!(derivative.dx(), &r(3));
    assert_eq!(derivative.dy(), &r(0));
    assert!(clone.is_homogeneous_power_basis_cached());
    assert_eq!(clone.derivative_at(&q(1, 2), &policy).unwrap(), derivative);
}

#[test]
fn general_rational_derivatives_are_not_truncated_at_bezier_degree() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(4, 0)], vec![r(1), r(3)]).unwrap();
    let policy = CurvePolicy::certified();

    let derivatives = curve.derivatives_at(&q(1, 2), 3, &policy).unwrap();

    assert_eq!(derivatives.len(), 3);
    assert_eq!((derivatives[0].dx(), derivatives[0].dy()), (&r(3), &r(0)));
    assert_eq!((derivatives[1].dx(), derivatives[1].dy()), (&r(-6), &r(0)));
    assert_eq!((derivatives[2].dx(), derivatives[2].dy()), (&r(18), &r(0)));
}

#[test]
fn rational_bezier_clones_share_the_retained_homogeneous_control_net() {
    let curve = curve();
    let clone = curve.clone();
    let policy = CurvePolicy::certified();
    assert!(!curve.is_homogeneous_control_net_cached());

    clone.point_at(&q(1, 2), &policy).unwrap();

    assert!(curve.is_homogeneous_control_net_cached());
    assert!(clone.is_homogeneous_control_net_cached());
}

#[test]
fn rational_bezier_clones_share_the_retained_homogeneous_power_basis() {
    let curve = curve();
    let clone = curve.clone();
    let policy = CurvePolicy::certified();
    assert!(!curve.is_homogeneous_power_basis_cached());

    assert!(
        clone
            .contains_point(&Point2::new(q(49, 20), q(9, 4)), &policy)
            .unwrap()
    );

    assert!(curve.is_homogeneous_power_basis_cached());
    assert!(clone.is_homogeneous_power_basis_cached());
}

#[test]
fn general_rational_split_preserves_join_and_degree() {
    let curve = curve();
    let policy = CurvePolicy::certified();
    let half = q(1, 2);
    let expected_join = curve.point_at(&half, &policy).unwrap();
    let (left, right) = decided(curve.split_at_exact(&half, &policy).unwrap());

    assert_eq!(left.degree(), 3);
    assert_eq!(right.degree(), 3);
    assert_eq!(left.end(), &expected_join);
    assert_eq!(right.start(), &expected_join);
    assert_eq!(left.point_at(&r(1), &policy).unwrap(), expected_join);
    assert_eq!(right.point_at(&r(0), &policy).unwrap(), expected_join);
}

#[test]
fn general_rational_cubic_certifies_obvious_axis_monotonicity() {
    let curve = curve();
    let policy = CurvePolicy::certified();

    assert!(curve.axis_is_monotone(Axis2::X, &policy).unwrap());
    assert!(!curve.axis_is_monotone(Axis2::Y, &policy).unwrap());
}

#[test]
fn mixed_derivative_controls_use_exact_root_multiplicity_for_monotonicity() {
    let policy = CurvePolicy::certified();
    let stationary_monotone =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(0, 0), p(1, 0)], vec![r(1); 4]).unwrap();
    let two_extrema =
        RationalBezier2::try_new(vec![p(0, 0), p(3, 0), p(-2, 0), p(1, 0)], vec![r(1); 4]).unwrap();
    let endpoint_sign_reversal =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(1, 0), p(0, 0)], vec![r(1); 4]).unwrap();

    assert!(
        stationary_monotone
            .axis_is_monotone(Axis2::X, &policy)
            .unwrap()
    );
    assert!(!two_extrema.axis_is_monotone(Axis2::X, &policy).unwrap());
    assert!(
        !endpoint_sign_reversal
            .axis_is_monotone(Axis2::X, &policy)
            .unwrap()
    );
}

#[test]
fn high_degree_nonuniform_rational_weights_preserve_axis_monotonicity() {
    let curve = RationalBezier2::try_new(
        (0..=12)
            .map(|index| p(index, (index * index) % 7))
            .collect(),
        (0..=12).map(|index| r(1 + index % 3)).collect(),
    )
    .unwrap();

    assert!(
        curve
            .axis_is_monotone(Axis2::X, &CurvePolicy::certified())
            .unwrap()
    );
}

#[test]
fn degree_40_rational_monotonicity_does_not_depend_on_u64_binomials() {
    let curve = RationalBezier2::try_new(
        (0..=40).map(|index| p(index, index % 3)).collect(),
        vec![r(1); 41],
    )
    .unwrap();

    assert!(
        curve
            .axis_is_monotone(Axis2::X, &CurvePolicy::certified())
            .unwrap()
    );
}

#[test]
fn rational_axis_monotonicity_preserves_typed_sign_blocker() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![unresolved_positive(), r(1)])
        .unwrap();

    let error = curve
        .axis_is_monotone(Axis2::X, &CurvePolicy::certified())
        .unwrap_err();
    assert_eq!(error.operation(), CurveOperation2::Classification);
    assert_eq!(error.family(), CurveFamily2::RationalBezier);
    assert_eq!(error.source(), None);
    assert!(matches!(
        error,
        ExactCurveError::Blocked(blocker)
            if blocker.reason() == UncertaintyReason::RealSign
    ));
}

#[test]
fn rational_evaluation_and_bounds_preserve_typed_sign_blockers() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![unresolved_positive(), r(1)])
        .unwrap();
    let policy = CurvePolicy::certified();

    for error in [
        curve.point_at(&r(0), &policy).unwrap_err(),
        curve.derivative_at(&r(0), &policy).unwrap_err(),
        curve.derivatives_at(&r(0), 3, &policy).unwrap_err(),
    ] {
        assert_eq!(error.operation(), CurveOperation2::Evaluation);
        assert_eq!(error.family(), CurveFamily2::RationalBezier);
        assert_eq!(error.source(), None);
        assert!(matches!(
            error,
            ExactCurveError::Blocked(blocker)
                if blocker.reason() == UncertaintyReason::RealSign
        ));
    }

    let error = curve.certified_bounds(&policy).unwrap_err();
    assert_eq!(error.operation(), CurveOperation2::Classification);
    assert_eq!(error.family(), CurveFamily2::RationalBezier);
    assert!(matches!(
        error,
        ExactCurveError::Blocked(blocker)
            if blocker.reason() == UncertaintyReason::RealSign
    ));
}

#[test]
fn top_level_general_rational_curve_preserves_family_and_native_geometry() {
    let top_level = Curve2::from(curve());

    assert_eq!(top_level.family(), CurveFamily2::RationalBezier);
    let fragments = top_level.native_bezier_fragments().unwrap();
    assert_eq!(fragments.len(), 1);
    assert!(matches!(
        fragments[0].curve(),
        hypercurve::BezierSubcurve2::Rational(_)
    ));
}

#[test]
fn represented_multi_split_materializes_connected_rational_fragments() {
    let curve = curve();
    let policy = CurvePolicy::certified();
    let split = decided(
        curve
            .split_at_parameters(
                &[
                    BezierParameter2::Exact(q(3, 4)),
                    BezierParameter2::Exact(q(1, 4)),
                    BezierParameter2::Exact(q(1, 4)),
                ],
                &policy,
            )
            .unwrap(),
    );

    assert!(split.is_fully_materialized());
    assert_eq!(split.fragments().len(), 3);
    let curves = split
        .fragments()
        .iter()
        .map(|fragment| match fragment {
            BezierSplitFragment2::Materialized {
                curve: BezierSubcurve2::Rational(curve),
                ..
            } => curve,
            _ => panic!("represented rational split did not materialize natively"),
        })
        .collect::<Vec<_>>();
    assert_eq!(curves[0].start(), curve.start());
    assert_eq!(curves[0].end(), curves[1].start());
    assert_eq!(curves[1].end(), curves[2].start());
    assert_eq!(curves[2].end(), curve.end());
}

#[test]
fn general_rational_line_contact_retains_exact_parameter_and_kind() {
    let curve = curve();
    let policy = CurvePolicy::certified();
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

    let relation = decided(curve.relation_to_line_with_contacts(&line, &CurvePolicy::certified()));
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
    let policy = CurvePolicy::certified();
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
        curve.relation_to_line_with_contacts(&axis, &CurvePolicy::certified()),
        Classification::Decided(BezierLineContactRelation::NoContact)
    );
}

#[test]
fn general_rational_line_relation_certifies_hull_miss_and_coincidence() {
    let policy = CurvePolicy::certified();
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
fn general_rational_point_incidence_rechecks_full_homogeneous_image() {
    let curve = curve();
    let policy = CurvePolicy::certified();
    let midpoint = Point2::new(q(49, 20), q(9, 4));

    assert_eq!(
        curve.point_incidence(&midpoint, &policy).unwrap(),
        RationalBezierPointIncidence2::Parameters(vec![BezierParameter2::Exact(q(1, 2))])
    );
    assert!(curve.contains_point(&midpoint, &policy).unwrap());
    assert!(!curve.contains_point(&p(5, 1), &policy).unwrap());
    assert!(!curve.contains_point(&p(2, 1), &policy).unwrap());
}

#[test]
fn general_rational_point_incidence_retains_nonlinear_algebraic_parameter() {
    let curve =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 1)], vec![r(1), r(1), r(1)]).unwrap();
    let policy = CurvePolicy::certified();
    let query = Point2::new(q(1, 2), q(1, 2));
    let incidence = curve.point_incidence(&query, &policy).unwrap();
    let RationalBezierPointIncidence2::Parameters(parameters) = incidence else {
        panic!("nonconstant curve reported whole-curve incidence");
    };

    assert_eq!(parameters.len(), 1);
    assert!(matches!(parameters[0], BezierParameter2::Algebraic(_)));
    assert!(curve.contains_point(&query, &policy).unwrap());
}

#[test]
fn general_rational_point_incidence_retains_endpoint_and_entire_curve_cases() {
    let policy = CurvePolicy::certified();
    let parabola =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 1)], vec![r(1), r(1), r(1)]).unwrap();
    assert_eq!(
        parabola.point_incidence(&p(0, 0), &policy).unwrap(),
        RationalBezierPointIncidence2::Parameters(vec![BezierParameter2::Exact(r(0))])
    );
    assert_eq!(
        parabola.point_incidence(&p(1, 1), &policy).unwrap(),
        RationalBezierPointIncidence2::Parameters(vec![BezierParameter2::Exact(r(1))])
    );

    let constant =
        RationalBezier2::try_new(vec![p(2, 3), p(2, 3), p(2, 3)], vec![r(1), r(2), r(3)]).unwrap();
    assert_eq!(
        constant.point_incidence(&p(2, 3), &policy).unwrap(),
        RationalBezierPointIncidence2::EntireCurve
    );
    assert!(!constant.contains_point(&p(3, 2), &policy).unwrap());
}

#[test]
fn general_rational_contacts_recognize_projective_scale_and_reversal() {
    let curve = curve();
    let policy = CurvePolicy::certified();
    let scaled = RationalBezier2::try_new(
        curve.control_points().to_vec(),
        vec![r(2), r(4), r(6), r(8)],
    )
    .unwrap();

    for (other, orientation, second_range) in [
        (
            curve.clone(),
            RationalBezierOverlapOrientation2::Same,
            ParamRange::new(r(0), r(1)),
        ),
        (
            scaled,
            RationalBezierOverlapOrientation2::Same,
            ParamRange::new(r(0), r(1)),
        ),
        (
            curve.reversed(),
            RationalBezierOverlapOrientation2::Reversed,
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
    let policy = CurvePolicy::certified();
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
fn rational_contacts_preserve_a_typed_sign_blocker_after_bound_fallthrough() {
    let first = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![unresolved_positive(), r(1)])
        .unwrap();
    let second = RationalBezier2::try_new(vec![p(3, 0), p(4, 1)], vec![r(1), r(1)]).unwrap();

    let error = first
        .intersection_contacts(&second, &CurvePolicy::certified())
        .unwrap_err();
    assert_eq!(error.operation(), CurveOperation2::Intersection);
    assert_eq!(error.family(), CurveFamily2::RationalBezier);
    assert_eq!(error.source(), None);
    assert!(matches!(
        error,
        ExactCurveError::Blocked(blocker)
            if blocker.reason() == UncertaintyReason::RealSign
    ));
}

#[test]
fn rational_resultant_certifies_disjoint_and_represented_crossing_parameters() {
    let policy = CurvePolicy::certified();
    let rising = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![r(1), r(1)]).unwrap();
    let falling = RationalBezier2::try_new(vec![p(0, 1), p(1, 0)], vec![r(1), r(1)]).unwrap();
    let crossing = rising.intersection_candidates(&falling, &policy).unwrap();
    let RationalBezierIntersectionCandidates2::Candidates {
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
    assert!(matches!(
        contacts[0].point(),
        RationalBezierIntersectionPointEvidence2::Exact(point)
            if point == &Point2::new(q(1, 2), q(1, 2))
    ));
    let above = RationalBezier2::try_new(vec![p(0, 2), p(1, 2)], vec![r(1), r(1)]).unwrap();
    assert_eq!(
        rising.intersection_candidates(&above, &policy).unwrap(),
        RationalBezierIntersectionCandidates2::NoIntersection
    );
}

#[test]
#[cfg(feature = "predicates")]
fn rational_resultant_retains_algebraic_parameter_projections() {
    let policy = CurvePolicy::certified();
    let parabola = RationalBezier2::try_new(
        vec![Point2::new(r(0), r(0)), Point2::new(q(1, 2), r(0)), p(1, 1)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();
    let horizontal = RationalBezier2::try_new(
        vec![Point2::new(r(0), q(1, 2)), Point2::new(r(1), q(1, 2))],
        vec![r(1), r(1)],
    )
    .unwrap();
    let candidates = parabola
        .intersection_candidates(&horizontal, &policy)
        .unwrap();
    let RationalBezierIntersectionCandidates2::Candidates {
        first_parameters,
        second_parameters,
    } = candidates
    else {
        panic!("parabola crossing did not retain resultant candidates");
    };
    assert!(matches!(
        first_parameters.as_slice(),
        [BezierParameter2::Algebraic(_)]
    ));
    assert!(matches!(
        second_parameters.as_slice(),
        [BezierParameter2::Algebraic(_)]
    ));
    let BezierParameter2::Algebraic(first_parameter) = &first_parameters[0] else {
        unreachable!("asserted algebraic parameter")
    };
    let image = parabola
        .point_at_algebraic_parameter(first_parameter, &policy)
        .unwrap();
    assert_eq!(
        image.status(),
        BezierAlgebraicImageStatus::Transformed,
        "{image:?}"
    );
    assert!(
        image
            .x()
            .and_then(|coordinate| coordinate.representation())
            .is_some()
    );
    assert!(
        image
            .y()
            .and_then(|coordinate| coordinate.representation())
            .is_some()
    );
    let derivatives = parabola
        .derivatives_at_algebraic_parameter(first_parameter, 3, &policy)
        .unwrap();
    assert_eq!(derivatives.len(), 3);
    assert!(
        derivatives
            .iter()
            .all(|derivative| derivative.status() == BezierAlgebraicImageStatus::Transformed)
    );
    let represented_coordinate = |order: usize, x_axis: bool| {
        let coordinate = if x_axis {
            derivatives[order - 1].dx()
        } else {
            derivatives[order - 1].dy()
        };
        coordinate
            .and_then(|coordinate| coordinate.representation())
            .and_then(|coordinate| coordinate.exact_rational_witness())
            .cloned()
    };
    assert_eq!(represented_coordinate(2, true), Some(r(0)));
    assert_eq!(represented_coordinate(2, false), Some(r(2)));
    assert_eq!(represented_coordinate(3, true), Some(r(0)));
    assert_eq!(represented_coordinate(3, false), Some(r(0)));
    let contacts = parabola
        .intersection_contacts(&horizontal, &policy)
        .unwrap();
    let RationalBezierIntersectionContacts2::Contacts(contacts) = contacts else {
        panic!("algebraic resultant candidates did not replay completely");
    };
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].first_parameter().as_exact().is_none());
    assert!(contacts[0].second_parameter().as_exact().is_none());
    assert!(matches!(
        contacts[0].point(),
        RationalBezierIntersectionPointEvidence2::Algebraic(_)
    ));

    let prepared = parabola
        .try_prepare_intersection(&horizontal, &policy)
        .unwrap();
    let prepared_clone = prepared.clone();
    assert!(!prepared.is_contact_replay_cached());
    assert!(matches!(
        prepared.try_contacts().unwrap(),
        RationalBezierIntersectionContacts2::Contacts(ref contacts) if contacts.len() == 1
    ));
    assert!(prepared.is_contact_replay_cached());
    assert!(prepared_clone.is_contact_replay_cached());
    assert!(matches!(
        prepared_clone.try_contact_view().unwrap(),
        RationalBezierIntersectionContacts2::Contacts(contacts) if contacts.len() == 1
    ));
    assert_eq!(
        prepared_clone.try_contacts().unwrap(),
        prepared.try_contacts().unwrap()
    );
    assert!(!prepared.is_topology_cached());
    let topology = prepared.try_topology_view().unwrap();
    assert!(prepared.is_topology_cached());
    assert!(prepared_clone.is_topology_cached());
    assert_eq!(topology.contacts().len(), 1);
    let retained_contacts = match prepared.try_contact_view().unwrap() {
        RationalBezierIntersectionContacts2::Contacts(contacts) => contacts.as_ref(),
        other => panic!("unexpected retained contacts: {other:?}"),
    };
    assert!(std::ptr::eq(topology.contacts(), retained_contacts));
    assert_eq!(topology.first().fragments().len(), 2);
    assert_eq!(topology.second().fragments().len(), 2);
    assert!(
        topology
            .first()
            .fragments()
            .iter()
            .chain(topology.second().fragments())
            .all(|fragment| matches!(
                fragment,
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
            ))
    );
    assert!(!topology.is_arrangement_cached());
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 4);
    assert!(topology.is_arrangement_cached());
    assert_eq!(topology.arrangement_graph().unwrap().len(), 4);

    let split = decided(
        parabola
            .split_at_parameters(&first_parameters, &policy)
            .unwrap(),
    );
    assert_eq!(split.fragments().len(), 2);
    assert!(split.fragments().iter().all(|fragment| matches!(
        fragment,
        BezierSplitFragment2::AlgebraicEndpointImages {
            start_image,
            end_image,
            ..
        } if start_image.as_ref().is_none_or(|image| image.is_transformed())
            && end_image.as_ref().is_none_or(|image| image.is_transformed())
    )));
    for image in split
        .fragments()
        .iter()
        .flat_map(|fragment| match fragment {
            BezierSplitFragment2::AlgebraicEndpointImages {
                start_image,
                end_image,
                ..
            } => [start_image.as_ref(), end_image.as_ref()],
            _ => [None, None],
        })
    {
        let Some(image) = image else { continue };
        assert!(image.second_derivative().is_some());
        assert!(image.third_derivative().is_some());
    }
}

#[test]
fn rational_contacts_replay_represented_resultant_candidates() {
    let policy = CurvePolicy::certified();
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
    assert!(matches!(
        contacts[0].point(),
        RationalBezierIntersectionPointEvidence2::Exact(point)
            if point == &Point2::new(q(1, 2), q(1, 4))
    ));
}

#[test]
fn rational_resultant_replays_identical_and_reversed_full_image_overlap() {
    let policy = CurvePolicy::certified();
    let curve = curve();
    assert_eq!(
        curve
            .intersection_candidates(&curve.clone(), &policy)
            .unwrap(),
        RationalBezierIntersectionCandidates2::DegenerateResultant
    );
    let RationalBezierIntersectionContacts2::Overlap(overlap) = curve
        .intersection_contacts(&curve.clone(), &policy)
        .unwrap()
    else {
        panic!("identical curve did not retain certified overlap");
    };
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
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
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::one(), Real::zero())
    );
    let prepared = curve
        .try_prepare_intersection(&curve.clone(), &policy)
        .unwrap();
    let error = prepared.try_topology().unwrap_err();
    assert_eq!(error.operation(), hypercurve::CurveOperation2::Arrangement);
    assert_eq!(error.family(), CurveFamily2::RationalBezier);
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
        .intersection_contacts(&second, &CurvePolicy::certified())
        .unwrap();
    let RationalBezierIntersectionContacts2::Overlap(overlap) = contacts else {
        panic!("projectively reparameterized conic remained unresolved: {contacts:?}");
    };
    assert_eq!(
        overlap.first_range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(
        overlap.second_range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );

    let RationalBezierIntersectionContacts2::Overlap(reversed) = first
        .intersection_contacts(&second.reversed(), &CurvePolicy::certified())
        .unwrap()
    else {
        panic!("reversed projective conic did not retain overlap");
    };
    assert_eq!(
        reversed.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
    assert_eq!(
        reversed.second_range().exact_endpoints(),
        Some((&Real::one(), &Real::zero()))
    );
}

#[test]
fn independently_trimmed_projective_conics_retain_partial_overlap() {
    let policy = CurvePolicy::certified();
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
    let (first_start, first_end) = overlap.first_range().exact_endpoints().unwrap();
    assert!(matches!(
        BezierParameter2::Exact(first_start.clone())
            .cmp_by_interval(&BezierParameter2::Exact(Real::zero()), &policy)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Greater)
    ));
    assert_eq!(first_end, &Real::one());
    let (second_start, second_end) = overlap.second_range().exact_endpoints().unwrap();
    assert_eq!(second_start, &Real::zero());
    assert!(matches!(
        BezierParameter2::Exact(second_end.clone())
            .cmp_by_interval(&BezierParameter2::Exact(Real::one()), &policy)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Less)
    ));
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
}

#[test]
fn rational_resultant_certifies_exact_partial_nonlinear_overlap_ranges() {
    let policy = CurvePolicy::certified();
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
        first.point_incidence(second.start(), &policy).unwrap(),
        RationalBezierPointIncidence2::Parameters(vec![BezierParameter2::Exact(q(1, 2))])
    );
    assert_eq!(
        second.point_incidence(first.end(), &policy).unwrap(),
        RationalBezierPointIncidence2::Parameters(vec![BezierParameter2::Exact(q(3, 4))])
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
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
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
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
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
fn rational_partial_overlap_reuses_exact_source_parameter_lineage() {
    let policy = CurvePolicy::certified();
    let source = curve();
    let first = decided(
        source
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    let second = decided(
        source
            .subcurve_between_exact(&q(1, 4), &Real::one(), &policy)
            .unwrap(),
    );

    assert!(!first.is_homogeneous_power_basis_cached());
    assert!(!second.is_homogeneous_power_basis_cached());
    let RationalBezierIntersectionContacts2::Overlap(overlap) =
        first.intersection_contacts(&second, &policy).unwrap()
    else {
        panic!("source-related partial overlap was not certified");
    };
    assert_eq!(
        overlap.first_range(),
        &ParamRange::new(q(1, 3), Real::one())
    );
    assert_eq!(
        overlap.second_range(),
        &ParamRange::new(Real::zero(), q(2, 3))
    );
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
    assert!(!first.is_homogeneous_power_basis_cached());
    assert!(!second.is_homogeneous_power_basis_cached());

    let reversed = second.reversed();
    let RationalBezierIntersectionContacts2::Overlap(reversed_overlap) =
        first.intersection_contacts(&reversed, &policy).unwrap()
    else {
        panic!("reversed source-related partial overlap was not certified");
    };
    assert_eq!(
        reversed_overlap.first_range(),
        &ParamRange::new(q(1, 3), Real::one())
    );
    assert_eq!(
        reversed_overlap.second_range(),
        &ParamRange::new(Real::one(), q(1, 3))
    );
    assert_eq!(
        reversed_overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn independently_constructed_partial_overlap_reconstructs_rational_endpoints() {
    let policy = CurvePolicy::certified();
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
    let first = RationalBezier2::try_new(
        source_first.control_points().to_vec(),
        source_first.weights().to_vec(),
    )
    .unwrap();
    let second = RationalBezier2::try_new(
        source_second.control_points().to_vec(),
        source_second.weights().to_vec(),
    )
    .unwrap();

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
    let policy = CurvePolicy::certified();
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
    assert_eq!(overlap.first_range().end().as_exact(), Some(&Real::one()));
    assert_eq!(
        overlap.second_range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
}

#[test]
fn line_image_overlap_accepts_monotone_parameterization_with_stationary_point() {
    let policy = CurvePolicy::certified();
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
        overlap.first_range().exact_endpoints(),
        Some((&q(1, 2), &Real::one()))
    );
    assert_eq!(
        overlap.second_range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
}

#[test]
fn polynomial_graph_overlap_retains_irrational_curved_boundary() {
    let policy = CurvePolicy::certified();
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
        RationalBezierIntersectionCandidates2::DegenerateResultant
    );
    let RationalBezierIntersectionContacts2::Overlap(overlap) = partial_parabola
        .intersection_contacts(&nonlinear_parameterization, &policy)
        .unwrap()
    else {
        panic!("certified polynomial graph did not retain its curved overlap");
    };
    assert_eq!(
        overlap.first_range().exact_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );
    assert!(matches!(
        overlap.second_range().start(),
        BezierParameter2::Algebraic(_)
    ));
    assert_eq!(overlap.second_range().end().as_exact(), Some(&Real::one()));
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );
}

#[test]
fn rational_bezier_degree_elevation_preserves_exact_parameterized_image_and_lineage() {
    let curve = curve();
    let clone = curve.clone();
    assert!(!curve.is_degree_elevation_cached(5));

    let elevated = curve.elevated_to_degree(5).unwrap();
    assert_eq!(elevated.degree(), 5);
    assert_eq!(
        elevated.weights(),
        &[r(1), q(8, 5), q(11, 5), q(14, 5), q(17, 5), r(4)]
    );
    for parameter in [r(0), q(1, 4), q(1, 2), q(3, 4), r(1)] {
        assert_eq!(
            elevated.point_at(&parameter, &CurvePolicy::certified()),
            curve.point_at(&parameter, &CurvePolicy::certified())
        );
    }
    assert!(clone.is_degree_elevation_cached(4));
    assert!(clone.is_degree_elevation_cached(5));
    assert_eq!(clone.elevated_to_degree(5).unwrap(), elevated);
    assert_eq!(
        elevated.source_parameter_range(),
        curve.source_parameter_range()
    );
}

#[test]
fn rational_bezier_degree_elevation_reports_invalid_target_and_zero_projective_weight() {
    let curve = curve();
    let invalid = curve.elevated_to_degree(2).unwrap_err();
    assert_eq!(invalid.operation(), CurveOperation2::DegreeElevation);
    assert_eq!(invalid.family(), CurveFamily2::RationalBezier);

    let singular = RationalBezier2::try_new(vec![p(0, 0), p(2, 0)], vec![r(1), r(-1)]).unwrap();
    assert!(matches!(
        singular.elevated_to_degree(2),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::DegreeElevation
                && blocker.family() == CurveFamily2::RationalBezier
    ));
    assert!(singular.is_degree_elevation_cached(2));
}
