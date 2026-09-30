use std::cmp::Ordering;

use hypercurve::{
    Axis2, BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
    Classification, CurveCertainty, CurveContext, CurveOutcome, CurvePoint2, Point2,
    RationalBezier2, Real,
};

mod support;

fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected a certified decision: {reason:?}"),
    }
}

fn certified<T: std::fmt::Debug>(outcome: CurveOutcome<Classification<T>>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    decided(outcome.value)
}

fn selected_point(reversed: bool) -> CurvePoint2 {
    let policy = CurveContext::STRICT;
    // alpha^5 + alpha = 1 has one root in (0, 1). The second chart
    // independently selects u = 1 - alpha from (1 - u)^5 - u = 0.
    let coefficients: &[i32] = if reversed {
        &[1, -6, 10, -10, 5, -1]
    } else {
        &[-1, 1, 0, 0, 0, 1]
    };
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            coefficients.iter().copied().map(Real::from).collect(),
            &policy,
        )
        .unwrap(),
    );
    let interval =
        decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap());
    let parameter =
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap());
    let mut controls = vec![
        Point2::new(Real::zero(), Real::pi()),
        Point2::new(Real::one(), Real::pi()),
    ];
    if reversed {
        controls.reverse();
    }
    let curve = RationalBezier2::try_new(controls, vec![Real::one(); 2]).unwrap();
    hypercurve::Curve2::from(curve)
        .point_at(
            &hypercurve::CurveParameter2::from(hypercurve::BezierParameter2::Algebraic(
                parameter.clone(),
            )),
            &policy,
        )
        .unwrap()
        .into_value()
}

#[test]
fn independent_selected_charts_share_the_general_point_queries() {
    let first = selected_point(false);
    let second = selected_point(true);
    assert!(first.coordinates().is_none());
    assert!(second.coordinates().is_none());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(certified(first.coincides_with(&second, &policy)));
        assert_eq!(
            certified(
                first
                    .compare_coordinate(&second, Axis2::X, &policy)
                    .unwrap()
            ),
            Ordering::Equal,
        );
        let at_height = CurvePoint2::from(Point2::new(Real::zero(), Real::pi()));
        assert_eq!(
            certified(
                first
                    .compare_coordinate(&at_height, Axis2::Y, &policy)
                    .unwrap()
            ),
            Ordering::Equal,
        );
        assert_eq!(
            certified(
                first
                    .compare_coordinate(&at_height, Axis2::X, &policy)
                    .unwrap()
            ),
            Ordering::Greater,
        );
        let bounds = certified(first.bounds(&policy));
        assert_eq!(bounds.min().y(), &Real::pi());
        assert_eq!(bounds.max().y(), &Real::pi());
    }
    // Queries keep the selected parameter representation available for reuse.
    assert!(first.coordinates().is_none());
    assert!(certified(
        first.coincides_with(&first.clone(), &CurveContext::STRICT)
    ));
}

#[test]
fn coordinate_view_accepts_arbitrary_exact_reals_and_preserves_certainty() {
    let coordinates = Point2::new(Real::pi(), Real::e());
    let point = CurvePoint2::from(coordinates.clone());
    assert_eq!(point.coordinates(), Some(&coordinates));
    let bounds = certified(point.bounds(&CurveContext::APPROXIMATE_512));
    assert_eq!(bounds.min(), &coordinates);
    assert_eq!(bounds.max(), &coordinates);
    assert!(certified(
        point.coincides_with(&point.clone(), &CurveContext::APPROXIMATE_512)
    ));

    let origin = CurvePoint2::from(Point2::new(Real::zero(), Real::zero()));
    let unresolved = CurvePoint2::from(Point2::new(
        support::terminally_unresolved_zero(),
        Real::zero(),
    ));
    let strict = origin.coincides_with(&unresolved, &CurveContext::STRICT);
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert!(matches!(strict.value, Classification::Uncertain(_)));
    let approximate = origin.coincides_with(&unresolved, &CurveContext::APPROXIMATE_512);
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(approximate.value, Classification::Decided(true));
    assert!(matches!(
        origin
            .coincides_with(&unresolved, &CurveContext::STRICT)
            .value,
        Classification::Uncertain(_),
    ));
}

mod generated_derivatives {
    use hypercurve::{
        Axis2, BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
        BezierParameterPolynomial, BezierParameterRange2, Classification, Curve2, CurveContext,
        CurveParameter2, ExactCurveError, Point2, QuadraticBezier2, Real, UncertaintyReason,
    };

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact fixture: {reason:?}"),
        }
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    /// The root of `a t^2 - 1` in `[lower, upper]`.
    fn inverse_root(a: i32, lower: Real, upper: Real, policy: &CurveContext) -> CurveParameter2 {
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-1), Real::zero(), Real::from(a)],
                policy,
            )
            .unwrap(),
        );
        let interval = decided(BezierParameterInterval::try_new(lower, upper, policy).unwrap());
        CurveParameter2::from(BezierParameter2::Algebraic(decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap(),
        )))
    }

    fn arch() -> QuadraticBezier2 {
        QuadraticBezier2::new(
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), Real::from(2)),
            Point2::new(Real::from(2), Real::zero()),
        )
    }

    #[test]
    fn zero_distance_generated_parallel_matches_its_source_derivative() {
        let policy = CurveContext::STRICT;
        let unit = decided(
            BezierParameterRange2::try_new(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
                &policy,
            )
            .unwrap(),
        );
        let generated = decided(
            Curve2::try_analytic_parallel(
                arch().parallel_left(Real::zero()).unwrap(),
                unit,
                &policy,
            )
            .unwrap(),
        );
        assert!(generated.geometry().is_none());
        let parameter = CurveParameter2::from(q(1, 3));
        let expected = Curve2::from(arch())
            .derivative_at(&parameter, &policy)
            .unwrap()
            .value;
        let actual = generated.derivative_at(&parameter, &policy).unwrap().value;
        assert_eq!(
            actual.represented_coordinates(),
            expected.represented_coordinates()
        );
    }

    #[test]
    fn retained_bezier_pieces_evaluate_selected_derivatives() {
        let policy = CurveContext::STRICT;
        let curve = Curve2::from(arch());
        // Split at 1/sqrt(2); the left piece keeps the source chart [0, 1/sqrt(2)].
        let cut = inverse_root(2, q(1, 2), q(3, 4), &policy);
        let (left, _) = curve.split_at(cut, &policy).unwrap().into_value();
        assert!(left.geometry().is_none());
        // 1/sqrt(3) lies inside the left piece; y' = 4 - 8t < 0 there, x' = 2.
        let inside = inverse_root(3, q(1, 2), q(2, 3), &policy);
        for derivative in [
            left.derivative_at(&inside, &policy).unwrap().value,
            curve.derivative_at(&inside, &policy).unwrap().value,
        ] {
            assert!(derivative.represented_coordinates().is_none());
            assert_eq!(
                derivative.coordinate_sign(Axis2::X, &policy).unwrap(),
                Classification::Decided(hyperreal::RealSign::Positive)
            );
            assert_eq!(
                derivative.coordinate_sign(Axis2::Y, &policy).unwrap(),
                Classification::Decided(hyperreal::RealSign::Negative)
            );
        }
    }

    fn arch_parallel(
        distance: Real,
        start: BezierParameter2,
        end: BezierParameter2,
        policy: &CurveContext,
    ) -> Curve2 {
        let range = decided(BezierParameterRange2::try_new(start, end, policy).unwrap());
        decided(
            Curve2::try_analytic_parallel(arch().parallel_left(distance).unwrap(), range, policy)
                .unwrap(),
        )
    }

    fn signs(
        derivative: &hypercurve::CurveVector2,
        policy: &CurveContext,
    ) -> (hyperreal::RealSign, hyperreal::RealSign) {
        (
            decided(derivative.coordinate_sign(Axis2::X, policy).unwrap()),
            decided(derivative.coordinate_sign(Axis2::Y, policy).unwrap()),
        )
    }

    #[test]
    fn selected_parallel_derivatives_orient_by_the_exact_speed_ratio() {
        use hyperreal::RealSign::{Negative, Positive};
        // The arch has v = (2, 4 - 8t) and cross(v, a) = -16, so its left
        // parallel at distance d moves along v * (1 + 16 d / |v|^3). At
        // t = 1/sqrt(3), |v|^6 is about 84.2: d = -1 passes the curvature
        // centre and reverses the tangent; d = -1/2 and d = 1 do not.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = inverse_root(3, q(1, 2), q(2, 3), &policy);
            let nearby = CurveParameter2::from(q(4, 7));
            for (distance, expected) in [
                (Real::one(), (Positive, Negative)),
                (q(-1, 2), (Positive, Negative)),
                (Real::from(-1), (Negative, Positive)),
            ] {
                // [11/20, 2/3] avoids every parallel cusp for these distances.
                let parallel = arch_parallel(
                    distance,
                    BezierParameter2::Exact(q(11, 20)),
                    BezierParameter2::Exact(q(2, 3)),
                    &policy,
                );
                let derivative = parallel.derivative_at(&selected, &policy).unwrap().value;
                assert!(derivative.represented_coordinates().is_none());
                assert_eq!(signs(&derivative, &policy), expected);
                // An independent represented evaluation nearby agrees.
                let represented = parallel.derivative_at(&nearby, &policy).unwrap().value;
                assert!(represented.represented_coordinates().is_some());
                assert_eq!(signs(&represented, &policy), expected);
            }
        }
    }

    #[test]
    fn selected_parallel_cusps_have_an_exact_zero_derivative() {
        let policy = CurveContext::STRICT;
        // 64 t^2 - 64 t + 14 = 0 gives (4 - 8t)^2 = 2 and |v|^2 = 6, so the
        // speed ratio 1 + 16 d / 6^(3/2) vanishes at d = -3 sqrt(6) / 8.
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(14), Real::from(-64), Real::from(64)],
                &policy,
            )
            .unwrap(),
        );
        let interval =
            decided(BezierParameterInterval::try_new(q(3, 10), q(7, 20), &policy).unwrap());
        let cusp = BezierParameter2::Algebraic(decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
        ));
        let selected = CurveParameter2::from(cusp.clone());
        let distance =
            Real::zero() - (Real::from(3) * Real::from(6).sqrt().unwrap() / Real::from(8)).unwrap();
        // The parallel may end at its cusp, but not contain it.
        let derivative = arch_parallel(distance, cusp, BezierParameter2::Exact(q(1, 2)), &policy)
            .derivative_at(&selected, &policy)
            .unwrap()
            .value;
        let zero = Real::zero();
        assert_eq!(derivative.represented_coordinates(), Some((&zero, &zero)));
    }

    #[test]
    fn selected_parallel_higher_derivatives_report_the_capability_boundary() {
        let policy = CurveContext::STRICT;
        let selected = inverse_root(3, q(1, 2), q(2, 3), &policy);
        assert!(matches!(
            arch_parallel(
                Real::one(),
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
                &policy
            )
            .derivatives_at(&selected, 2, &policy),
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Unsupported
        ));
    }
}
