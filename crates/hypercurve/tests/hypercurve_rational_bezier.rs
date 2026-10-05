mod support;
use hypercurve::{
    Axis2, Classification, Curve2, CurveContext, CurveFamily2, CurveOperation2, CurvePoint2,
    Point2, RationalBezier2, RationalQuadraticBezier2, Real,
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
    match crate::support::under(policy, || {
        Curve2::from(curve.clone()).point_locations(&CurvePoint2::from(point.clone()))
    })
    .unwrap()
    .value
    {
        hypercurve::CurvePointLocations2::EntireCurve => None,
        hypercurve::CurvePointLocations2::Locations(locations) => Some(
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
fn general_rational_cubic_evaluates_exactly() {
    let curve = curve();
    let policy = CurveContext::STRICT;
    let half = q(1, 2);

    assert_eq!(
        crate::support::under_value(&policy, || curve.point_at(&r(0))).unwrap(),
        p(0, 0)
    );
    assert_eq!(
        crate::support::under_value(&policy, || curve.point_at(&r(1))).unwrap(),
        p(4, 0)
    );
    assert_eq!(
        crate::support::under_value(&policy, || curve.point_at(&half)).unwrap(),
        Point2::new(q(49, 20), q(9, 4))
    );
}

#[test]
fn rational_quadratic_rational_parameter_uses_exact_power_quotient() {
    let curve =
        RationalQuadraticBezier2::try_new(p(0, 0), p(2, 4), p(6, 0), r(1), r(2), r(3)).unwrap();

    assert_eq!(
        decided(crate::support::under_classified(
            &CurveContext::STRICT,
            || curve.point_at(q(1, 2))
        )),
        Point2::new(q(13, 4), r(2))
    );
}

#[test]
fn rational_quadratic_exact_transcendental_pole_stays_projective() {
    // These weights give W(t) = 1 - pi*t. At t = 1/pi the numerator
    // remains nonzero, so this is a genuine projective pole, not an affine
    // point or a removable singularity.
    let curve = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(2, 4),
        p(6, 0),
        r(1),
        r(1) - (Real::pi() / r(2)).unwrap(),
        r(1) - Real::pi(),
    )
    .unwrap();
    let pole = (r(1) / Real::pi()).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            crate::support::under_classified(&policy, || curve.point_at(pole.clone())),
            Classification::Uncertain(hypercurve::UncertaintyReason::Boundary),
        );
        assert_eq!(
            decided(crate::support::under_classified(&policy, || curve.point_at(r(0)))),
            p(0, 0)
        );
        assert_eq!(
            decided(crate::support::under_classified(&policy, || curve.point_at(r(1)))),
            p(6, 0)
        );
    }
}

#[test]
fn rational_quadratic_monotone_root_preserves_unequal_weight_quotient_derivative() {
    let curve = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(1, 2),
        Point2::new(q(4, 9), r(5)),
        r(1),
        r(2),
        r(3),
    )
    .unwrap();

    assert_eq!(
        decided(crate::support::under_classified(
            &CurveContext::STRICT,
            || curve.axis_monotone_parameters(Axis2::X)
        )),
        vec![q(1, 2)]
    );
}

#[test]
fn rational_quadratic_monotone_root_retains_exact_transcendental_endpoint() {
    // The x coordinate has a horizontal tangent at t=1. Building that root
    // through the generic quadratic formula leaves a cancellative expression
    // involving pi, so endpoint extraction must retain the exact parameter
    // before radical construction.
    let curve = RationalQuadraticBezier2::try_new(
        p(0, 0),
        Point2::new(q(24, 5), q(7, 5)),
        Point2::new(q(24, 5), q(32, 5)),
        r(1),
        Real::pi(),
        r(1),
    )
    .unwrap();

    assert_eq!(
        decided(crate::support::under_classified(
            &CurveContext::STRICT,
            || curve.axis_monotone_parameters(Axis2::X)
        )),
        vec![Real::one()]
    );
}

#[test]
fn general_rational_derivative_is_exact_and_reuses_power_basis() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(4, 0)], vec![r(1), r(3)]).unwrap();
    let clone = curve.clone();
    let policy = CurveContext::STRICT;

    let derivative =
        crate::support::under_value(&policy, || curve.derivative_at(&q(1, 2))).unwrap();
    assert_eq!(derivative.dx(), &r(3));
    assert_eq!(derivative.dy(), &r(0));
    assert_eq!(
        crate::support::under_value(&policy, || clone.derivative_at(&q(1, 2))).unwrap(),
        derivative
    );
}

#[test]
fn general_rational_derivatives_are_not_truncated_at_bezier_degree() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(4, 0)], vec![r(1), r(3)]).unwrap();
    let policy = CurveContext::STRICT;

    let derivatives =
        crate::support::under_value(&policy, || curve.derivatives_at(&q(1, 2), 3)).unwrap();

    assert_eq!(derivatives.len(), 3);
    assert_eq!((derivatives[0].dx(), derivatives[0].dy()), (&r(3), &r(0)));
    assert_eq!((derivatives[1].dx(), derivatives[1].dy()), (&r(-6), &r(0)));
    assert_eq!((derivatives[2].dx(), derivatives[2].dy()), (&r(18), &r(0)));
}

#[test]
fn rational_bezier_clones_evaluate_identically() {
    let curve = curve();
    let clone = curve.clone();
    let policy = CurveContext::STRICT;
    assert_eq!(
        crate::support::under_value(&policy, || clone.point_at(&q(1, 2))).unwrap(),
        crate::support::under_value(&policy, || curve.point_at(&q(1, 2))).unwrap()
    );
}

#[test]
fn rational_bezier_clones_have_identical_point_incidence() {
    let curve = curve();
    let clone = curve.clone();
    let policy = CurveContext::STRICT;
    let point = Point2::new(q(49, 20), q(9, 4));
    assert!(crate::support::under_value(&policy, || clone.contains_point(&point)).unwrap());
    assert_eq!(
        crate::support::under_value(&policy, || clone.contains_point(&point)).unwrap(),
        crate::support::under_value(&policy, || curve.contains_point(&point)).unwrap()
    );
}

#[test]
fn general_rational_split_preserves_join_and_degree() {
    let curve = curve();
    let policy = CurveContext::STRICT;
    let half = q(1, 2);
    let expected_join = crate::support::under_value(&policy, || curve.point_at(&half)).unwrap();
    let (left, right) = decided(
        crate::support::under_classified_result(&policy, || curve.split_at_exact(&half)).unwrap(),
    );

    assert_eq!(left.degree(), 3);
    assert_eq!(right.degree(), 3);
    assert_eq!(left.end(), &expected_join);
    assert_eq!(right.start(), &expected_join);
    assert_eq!(
        crate::support::under_value(&policy, || left.point_at(&r(1))).unwrap(),
        expected_join
    );
    assert_eq!(
        crate::support::under_value(&policy, || right.point_at(&r(0))).unwrap(),
        expected_join
    );
}

#[test]
fn general_rational_cubic_certifies_obvious_axis_monotonicity() {
    let curve = curve();
    let policy = CurveContext::STRICT;

    assert!(crate::support::under_value(&policy, || curve.axis_is_monotone(Axis2::X)).unwrap());
    assert!(!crate::support::under_value(&policy, || curve.axis_is_monotone(Axis2::Y)).unwrap());
}

#[test]
fn mixed_derivative_controls_use_exact_root_multiplicity_for_monotonicity() {
    let policy = CurveContext::STRICT;
    let stationary_monotone =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(0, 0), p(1, 0)], vec![r(1); 4]).unwrap();
    let two_extrema =
        RationalBezier2::try_new(vec![p(0, 0), p(3, 0), p(-2, 0), p(1, 0)], vec![r(1); 4]).unwrap();
    let endpoint_sign_reversal =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(1, 0), p(0, 0)], vec![r(1); 4]).unwrap();

    assert!(
        crate::support::under_value(&policy, || stationary_monotone.axis_is_monotone(Axis2::X))
            .unwrap()
    );
    assert!(
        !crate::support::under_value(&policy, || two_extrema.axis_is_monotone(Axis2::X)).unwrap()
    );
    assert!(
        !crate::support::under_value(&policy, || endpoint_sign_reversal
            .axis_is_monotone(Axis2::X))
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
        crate::support::under_value(&CurveContext::STRICT, || curve.axis_is_monotone(Axis2::X))
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
        crate::support::under_value(&CurveContext::STRICT, || curve.axis_is_monotone(Axis2::X))
            .unwrap()
    );
}

#[test]
fn shared_cancellation_resolves_rational_weight_monotonicity_blocker() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![unresolved_positive(), r(1)])
        .unwrap();

    assert!(
        crate::support::under_value(&CurveContext::STRICT, || curve.axis_is_monotone(Axis2::X))
            .unwrap()
    );
}

#[test]
fn shared_cancellation_resolves_rational_evaluation_and_bounds_blockers() {
    let curve = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![unresolved_positive(), r(1)])
        .unwrap();
    let policy = CurveContext::STRICT;

    assert_eq!(
        crate::support::under_value(&policy, || curve.point_at(&r(0))).unwrap(),
        p(0, 0)
    );
    assert!(crate::support::under_value(&policy, || curve.derivative_at(&r(0))).is_ok());
    assert!(crate::support::under_value(&policy, || curve.derivatives_at(&r(0), 3)).is_ok());
    assert!(curve.certified_bounds().is_ok());
}

#[test]
fn top_level_general_rational_curve_preserves_family_and_native_geometry() {
    let top_level = Curve2::from(curve());

    assert_eq!(top_level.family(), CurveFamily2::RationalBezier);
    let fragments = top_level.native_bezier_fragments().unwrap();
    assert_eq!(fragments.len(), 1);
    assert!(matches!(
        fragments[0].curve(),
        hypercurve::CurveGeometry2::RationalBezier(_)
    ));
}

#[test]
fn general_rational_point_incidence_rechecks_full_homogeneous_image() {
    let curve = curve();
    let policy = CurveContext::STRICT;
    let midpoint = Point2::new(q(49, 20), q(9, 4));

    assert_eq!(
        point_parameters(&curve, &midpoint, &policy),
        Some(vec![Some(q(1, 2))])
    );
    assert!(crate::support::under_value(&policy, || curve.contains_point(&midpoint)).unwrap());
    assert!(!crate::support::under_value(&policy, || curve.contains_point(&p(5, 1))).unwrap());
    assert!(!crate::support::under_value(&policy, || curve.contains_point(&p(2, 1))).unwrap());
}

#[test]
fn general_rational_point_incidence_retains_nonlinear_algebraic_parameter() {
    let curve =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 1)], vec![r(1), r(1), r(1)]).unwrap();
    let policy = CurveContext::STRICT;
    let query = Point2::new(q(1, 2), q(1, 2));
    let Some(parameters) = point_parameters(&curve, &query, &policy) else {
        panic!("nonconstant curve reported whole-curve incidence");
    };

    assert_eq!(parameters.len(), 1);
    assert!(
        parameters[0].is_none(),
        "the root stays a selected algebraic parameter"
    );
    assert!(crate::support::under_value(&policy, || curve.contains_point(&query)).unwrap());
}

#[test]
fn general_rational_point_incidence_retains_endpoint_and_entire_curve_cases() {
    let policy = CurveContext::STRICT;
    let parabola =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 1)], vec![r(1), r(1), r(1)]).unwrap();
    assert_eq!(
        point_parameters(&parabola, &p(0, 0), &policy),
        Some(vec![Some(r(0))])
    );
    assert_eq!(
        point_parameters(&parabola, &p(1, 1), &policy),
        Some(vec![Some(r(1))])
    );

    let constant =
        RationalBezier2::try_new(vec![p(2, 3), p(2, 3), p(2, 3)], vec![r(1), r(2), r(3)]).unwrap();
    assert_eq!(point_parameters(&constant, &p(2, 3), &policy), None);
    assert!(!crate::support::under_value(&policy, || constant.contains_point(&p(3, 2))).unwrap());
}

#[test]
fn rational_bezier_degree_elevation_preserves_exact_parameterized_image_and_lineage() {
    let curve = curve();
    let clone = curve.clone();

    let elevated = curve.elevated_to_degree(5).unwrap();
    assert_eq!(elevated.degree(), 5);
    assert_eq!(
        elevated.weights(),
        &[r(1), q(8, 5), q(11, 5), q(14, 5), q(17, 5), r(4)]
    );
    for parameter in [r(0), q(1, 4), q(1, 2), q(3, 4), r(1)] {
        assert_eq!(
            crate::support::under_value(&CurveContext::STRICT, || elevated.point_at(&parameter)),
            crate::support::under_value(&CurveContext::STRICT, || curve.point_at(&parameter))
        );
    }
    assert_eq!(clone.elevated_to_degree(5).unwrap(), elevated);
    assert_eq!(
        elevated.source_parameter_range(),
        curve.source_parameter_range()
    );
}

#[test]
fn rational_bezier_degree_elevation_preserves_projective_controls_and_poles() {
    let curve = curve();
    let invalid = curve.elevated_to_degree(2).unwrap_err();
    assert_eq!(invalid.operation(), CurveOperation2::DegreeElevation);
    assert_eq!(invalid.family(), Some(CurveFamily2::RationalBezier));

    let singular = RationalBezier2::try_new(vec![p(0, 0), p(2, 0)], vec![r(1), r(-1)]).unwrap();
    let first = singular.elevated_to_degree(2).unwrap();
    assert_eq!(first.degree(), 2);
    assert!(first.affine_control_points().is_none());
    assert_eq!(first.weights(), &[r(1), r(0), r(-1)]);
    assert_eq!(singular.elevated_to_degree(2).unwrap(), first);
    // Representing a zero intermediate weight does not certify a finite curve.
    assert!(
        crate::support::under_value(&CurveContext::STRICT, || first.point_at(&q(1, 2))).is_err()
    );
    assert!(first.certified_bounds().is_err());
}
