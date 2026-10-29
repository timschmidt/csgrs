use hypercurve::{
    Axis2, BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CurveFamily2, CurveParameter2, CurveParameterRange2, CurvePoint2,
    CurvePointDefinition2, Point2, PolynomialCurveSet2, Real, Similarity2,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial, work_budget};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(x: Real, y: Real) -> CurvePoint2 {
    Point2::new(x, y).into()
}
fn set(coefficients: Vec<Vec<Real>>, first: [Real; 2], second: [Real; 2]) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &BivariatePolynomial::new(coefficients),
        &BivariateCurveDomain { first, second },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}
fn circle() -> PolynomialCurveSet2 {
    // The original BRep patch pullback: (u-1/2)^2+(v-1/2)^2-1/16=0.
    set(
        vec![
            vec![q(7, 16), (-1).into(), 1.into()],
            vec![(-1).into()],
            vec![1.into()],
        ],
        [0.into(), 1.into()],
        [0.into(), 1.into()],
    )
}
fn square_root_two() -> CurveParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![(-2).into(), 0.into(), 1.into()])
                .unwrap(),
            BezierParameterInterval::try_new(1.into(), 2.into()).unwrap(),
        )
        .unwrap(),
    )
    .into()
}

pub fn closed_loop_evaluation_split_reverse_and_source_identity() {
    let set = circle();
    assert_eq!(set.curves().len(), 2);
    assert!(set.isolated_points().is_empty());
    let lower = &set.curves()[0];
    let upper = &set.curves()[1];
    assert_eq!(lower.family(), CurveFamily2::ImplicitPolynomial);
    assert!(lower.geometry().is_none());
    assert!(
        lower
            .polynomial_branch()
            .unwrap()
            .shares_source(upper.polynomial_branch().unwrap())
    );
    for curve in set.curves() {
        assert!(curve.start().coincides_with(&p(q(1, 4), q(1, 2))).unwrap());
        assert!(curve.end().coincides_with(&p(q(3, 4), q(1, 2))).unwrap());
        assert!(curve.start().coordinates().is_none());
        assert!(matches!(
            curve.start().definition().unwrap(),
            CurvePointDefinition2::PolynomialFiber { .. }
        ));
        let (left, right) = curve.split_at(q(1, 2).into()).unwrap();
        assert!(left.end().coincides_with(&right.start()).unwrap());
        assert!(
            left.polynomial_branch()
                .unwrap()
                .shares_source(curve.polynomial_branch().unwrap())
        );
        let reverse = curve.reversed().unwrap();
        assert!(reverse.start().coincides_with(&curve.end()).unwrap());
        assert_eq!(
            reverse.parameter_endpoints().0,
            curve.parameter_endpoints().1
        );
        let (high, low) = reverse.split_at(q(1, 2).into()).unwrap();
        assert!(high.end().coincides_with(&low.start()).unwrap());
        assert!(high.start().coincides_with(&curve.end()).unwrap());
        assert!(curve.point_at(&Real::zero().into()).is_err());
        let derivatives = curve.derivatives_at(&q(1, 2).into(), 1).unwrap();
        assert_eq!(derivatives.len(), 1);
        assert_eq!(
            derivatives[0]
                .compare_linear_form(&Real::one(), &Real::zero(), &Real::one())
                .unwrap(),
            Ordering::Equal,
        );
        assert_eq!(
            derivatives[0]
                .compare_linear_form(&Real::zero(), &Real::one(), &Real::zero())
                .unwrap(),
            Ordering::Equal,
        );
        curve.bounds().unwrap();
    }
    assert!(
        lower
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&p(q(1, 2), q(1, 4)))
            .unwrap()
    );
    assert!(
        upper
            .point_at(&q(1, 2).into())
            .unwrap()
            .coincides_with(&p(q(1, 2), q(3, 4)))
            .unwrap()
    );
}

pub fn algebraic_slice_keeps_correlated_noncircular_fiber() {
    // y^3=x, evaluated at x=sqrt(2): x and y are selected, not plain reals.
    let set = set(
        vec![
            vec![0.into(), 0.into(), 0.into(), 1.into()],
            vec![(-1).into()],
        ],
        [1.into(), 2.into()],
        [0.into(), 2.into()],
    );
    assert_eq!(set.curves().len(), 1);
    let curve = &set.curves()[0];
    let parameter = square_root_two();
    let point = curve.point_at(&parameter).unwrap();
    let CurvePointDefinition2::PolynomialFiber {
        parameter: base, ..
    } = point.definition().unwrap()
    else {
        panic!("fiber evidence lost")
    };
    assert!(matches!(
        base.as_bezier_parameter(),
        Some(BezierParameter2::Algebraic(_))
    ));
    assert_eq!(
        point
            .compare_coordinate(&p(1.into(), 1.into()), Axis2::Y)
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        point
            .compare_coordinate(&p(2.into(), q(3, 2)), Axis2::Y)
            .unwrap(),
        Ordering::Less
    );
    let replay = point.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    let (left, right) = curve.split_at(parameter.clone()).unwrap();
    assert!(left.end().coincides_with(&right.start()).unwrap());
    assert!(
        left.point_at(&parameter)
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    assert!(
        point
            .affine_form(1.into(), (-1).into(), 0.into())
            .compare_to_real(&Real::zero())
            .unwrap()
            .is_gt()
    );
    assert_eq!(
        point
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap(),
        parameter
    );
    point.bounds().unwrap();
}

pub fn similarity_and_point_replay_keep_source_chart() {
    let set = circle();
    let transform = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let original = &set.curves()[0];
    let image = original.transform_similarity(&transform).unwrap();
    assert_eq!(original.parameter_domain(), image.parameter_domain());
    assert!(
        original
            .polynomial_branch()
            .unwrap()
            .shares_source(image.polynomial_branch().unwrap())
    );
    let selected = image.point_at(&q(1, 2).into()).unwrap();
    let expected = transform.transform_point(&Point2::new(q(1, 2), q(1, 4)));
    assert!(selected.coincides_with(&expected.into()).unwrap());
    assert!(
        selected
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(&selected)
            .unwrap()
    );
    let (first, second) = image.reversed().unwrap().split_at(q(1, 2).into()).unwrap();
    assert!(first.end().coincides_with(&second.start()).unwrap());
    image.bounds().unwrap();
    for _ in 0..3 {
        let again = image.point_at(&q(1, 2).into()).unwrap();
        assert!(again.coincides_with(&selected).unwrap());
    }
}

pub fn every_similarity_branch_keeps_exact_coordinate_comparisons() {
    let set = circle();
    let transform = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let delta = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            for (side, source) in set.curves().iter().enumerate() {
                let (left, right) = source.split_at(q(1, 2).into()).unwrap();
                for curve in [source.clone(), source.reversed().unwrap(), left, right] {
                    let image = curve.transform_similarity(&transform).unwrap();
                    let point = image.point_at(&q(1, 2).into()).unwrap();
                    // Independently evaluated (3x-4y+2, 4x+3y-1).
                    let (x, y) = if side == 0 {
                        (q(5, 2), q(7, 4))
                    } else {
                        (q(1, 2), q(13, 4))
                    };
                    let expected = p(x.clone(), y.clone());
                    for (a, b) in [(&point, &expected), (&expected, &point)] {
                        assert!(a.coincides_with(b).unwrap());
                        for axis in [Axis2::X, Axis2::Y] {
                            assert_eq!(a.compare_coordinate(b, axis).unwrap(), Ordering::Equal);
                        }
                    }
                    for (nearby, axis) in [
                        (p(&x + &delta, y.clone()), Axis2::X),
                        (p(x, &y + &delta), Axis2::Y),
                    ] {
                        assert!(!point.coincides_with(&nearby).unwrap());
                        assert!(!nearby.coincides_with(&point).unwrap());
                        assert_eq!(
                            point.compare_coordinate(&nearby, axis).unwrap(),
                            Ordering::Less
                        );
                        assert_eq!(
                            nearby.compare_coordinate(&point, axis).unwrap(),
                            Ordering::Greater
                        );
                    }
                }
            }
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
}

pub fn public_source_restrictions_keep_polynomial_bounds_and_traversal() {
    let circle = circle();
    let cubic = set(
        vec![
            vec![0.into(), 0.into(), 0.into(), 1.into()],
            vec![(-1).into()],
        ],
        [1.into(), 2.into()],
        [0.into(), 2.into()],
    );
    let cases: [(&hypercurve::Curve2, CurveParameter2, CurveParameter2); 3] = [
        (&circle.curves()[0], q(3, 8).into(), q(5, 8).into()),
        (&circle.curves()[1], q(3, 8).into(), q(5, 8).into()),
        (&cubic.curves()[0], Real::one().into(), square_root_two()),
    ];
    for (source, start, end) in cases {
        for source in [source.clone(), source.reversed().unwrap()] {
            let range = CurveParameterRange2::try_new(start.clone(), end.clone()).unwrap();
            let restricted = source.restrict_source(range.clone()).unwrap();
            let branch = restricted.polynomial_branch().unwrap();
            assert!(branch.shares_source(source.polynomial_branch().unwrap()));
            assert_eq!(
                branch.is_reversed(),
                source.polynomial_branch().unwrap().is_reversed()
            );
            assert_eq!(restricted.parameter_domain(), &range);
            assert!(restricted.source_restriction().is_none());
            restricted.bounds().unwrap();
            restricted.reversed().unwrap().bounds().unwrap();
            for parameter in [&start, &end] {
                assert!(
                    restricted
                        .point_at(parameter)
                        .unwrap()
                        .coincides_with(&source.point_at(parameter).unwrap())
                        .unwrap()
                );
            }
            let expected_start = if branch.is_reversed() { &end } else { &start };
            assert!(
                restricted
                    .start()
                    .coincides_with(&source.point_at(expected_start).unwrap())
                    .unwrap()
            );
            let outside = CurveParameterRange2::try_new(Real::zero().into(), end.clone()).unwrap();
            assert!(restricted.restrict_source(outside).is_err());
        }
    }
}

pub fn isolated_empty_and_singular_contacts_remain_distinct() {
    assert!(
        set(
            vec![vec![1.into()]],
            [0.into(), 1.into()],
            [0.into(), 1.into()]
        )
        .is_empty()
    );
    let empty = set(
        vec![vec![1.into(), 0.into(), 1.into()], vec![], vec![1.into()]],
        [(-1).into(), 1.into()],
        [(-1).into(), 1.into()],
    );
    assert!(empty.is_empty());
    let isolated = set(
        vec![vec![0.into(), 0.into(), 1.into()], vec![], vec![1.into()]],
        [(-1).into(), 1.into()],
        [(-1).into(), 1.into()],
    );
    assert!(!isolated.is_empty());
    assert!(isolated.curves().is_empty());
    assert_eq!(isolated.isolated_points().len(), 1);
    assert!(
        isolated.isolated_points()[0]
            .coincides_with(&p(0.into(), 0.into()))
            .unwrap()
    );
    for (coefficients, expected_incidence) in [
        (
            vec![
                vec![0.into(), 0.into(), 1.into()],
                vec![],
                vec![(-1).into()],
            ],
            4,
        ), // node
        (
            vec![
                vec![0.into(), 0.into(), 1.into()],
                vec![],
                vec![],
                vec![(-1).into()],
            ],
            2,
        ), // cusp
    ] {
        let set = set(
            coefficients,
            [(-1).into(), 1.into()],
            [(-2).into(), 2.into()],
        );
        assert!(set.isolated_points().is_empty());
        let origin = p(0.into(), 0.into());
        let incident = set
            .curves()
            .iter()
            .flat_map(|c| [c.start(), c.end()])
            .filter(|point| point.coincides_with(&origin).unwrap())
            .count();
        assert_eq!(incident, expected_incidence);
        for curve in set.curves() {
            assert!(
                curve
                    .reversed()
                    .unwrap()
                    .end()
                    .coincides_with(&curve.start())
                    .unwrap()
            );
        }
    }
}

pub fn forged_selector_rejection_and_budget_recovery() {
    let topology = circle().topology().clone();
    let limited = PolynomialCurveSet2::from_topology(
        topology.clone(),
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 0,
            refinement_steps: 0,
        },
    )
    .unwrap();
    let error = limited.curves()[0].point_at(&q(3, 8).into()).unwrap_err();
    assert!(
        matches!(error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
    );
    let recovered =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    recovered.curves()[0].point_at(&q(3, 8).into()).unwrap();
    let set = circle();
    let point = set.curves()[0].point_at(&q(1, 2).into()).unwrap();
    let CurvePointDefinition2::PolynomialFiber {
        equation,
        parameter,
        root,
    } = point.definition().unwrap()
    else {
        panic!("fiber evidence lost")
    };
    let mut both = root.clone();
    both.lower = 0.into();
    both.upper = 1.into();
    both.exact_root = None;
    both.distinct_root_count = 1;
    assert!(
        CurvePointDefinition2::PolynomialFiber {
            equation: equation.clone(),
            parameter: parameter.clone(),
            root: both
        }
        .into_point()
        .is_err()
    );
    let mut forged = root;
    forged.exact_root = Some(0.into());
    assert!(
        CurvePointDefinition2::PolynomialFiber {
            equation,
            parameter,
            root: forged
        }
        .into_point()
        .is_err()
    );
    let denied = work_budget::with_budget(work_budget::WorkBudget::ZERO, || {
        set.curves()[0].point_at(&q(3, 8).into())
    });
    assert!(denied.result.is_err());
    let recovered = set.curves()[0].point_at(&q(3, 8).into()).unwrap();
    assert_eq!(
        recovered
            .compare_coordinate(&p(q(3, 8), q(1, 2)), Axis2::Y)
            .unwrap(),
        Ordering::Less
    );
    assert!(
        recovered
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(&recovered)
            .unwrap()
    );
}

pub fn algebraic_events_and_tiny_loops_preserve_selected_endpoints() {
    let algebraic = set(
        vec![
            vec![(-2).into(), 0.into(), 1.into()],
            vec![],
            vec![1.into()],
        ],
        [(-2).into(), 2.into()],
        [(-2).into(), 2.into()],
    );
    assert_eq!(algebraic.curves().len(), 2);
    let transform = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    for (index, curve) in algebraic.curves().iter().enumerate() {
        assert!(matches!(
            curve.start().definition().unwrap(),
            CurvePointDefinition2::PolynomialFiber { parameter, .. }
                if matches!(parameter.as_bezier_parameter(), Some(BezierParameter2::Algebraic(_)))
        ));
        assert!(
            curve
                .point_at(&Real::one().into())
                .unwrap()
                .coincides_with(&p(
                    1.into(),
                    if index == 0 { (-1).into() } else { 1.into() }
                ))
                .unwrap()
        );
        let image = curve.transform_similarity(&transform).unwrap();
        let form = image.start().affine_form(4.into(), (-3).into(), 0.into());
        for policy in [
            hypercurve::PredicatePolicy::STRICT,
            hypercurve::PredicatePolicy::APPROXIMATE_512,
            hypercurve::PredicatePolicy::STRICT,
        ] {
            let result =
                hypercurve::evaluate_under(policy, || form.compare_to_real(&11.into()).unwrap());
            assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
            assert_eq!(result.certified().unwrap(), Ordering::Equal);
        }
        assert!(
            image
                .start()
                .definition()
                .unwrap()
                .into_point()
                .unwrap()
                .coincides_with(&image.start())
                .unwrap()
        );
    }
    let radius = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    let tiny = set(
        vec![
            vec![q(1, 2) - &radius * &radius, (-1).into(), 1.into()],
            vec![(-1).into()],
            vec![1.into()],
        ],
        [0.into(), 1.into()],
        [0.into(), 1.into()],
    );
    assert_eq!(tiny.curves().len(), 2);
    for (index, curve) in tiny.curves().iter().enumerate() {
        assert!(
            curve
                .start()
                .coincides_with(&p(q(1, 2) - &radius, q(1, 2)))
                .unwrap()
        );
        let y = if index == 0 {
            q(1, 2) - &radius
        } else {
            q(1, 2) + &radius
        };
        assert!(
            curve
                .point_at(&q(1, 2).into())
                .unwrap()
                .coincides_with(&p(q(1, 2), y))
                .unwrap()
        );
    }
    let invalid = CurvePointDefinition2::PolynomialFiber {
        equation: BivariatePolynomial::new(vec![]),
        parameter: BezierParameter2::Exact(0.into()).into(),
        root: hypersolve::IsolatedRootInterval {
            lower: 0.into(),
            upper: 1.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point();
    assert!(matches!(
        invalid,
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
}
