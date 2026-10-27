//! Algebraic similarity images keep the original selected graph parameters.

use hypercurve::{Curve2, CurvePoint2, LineSeg2, Point2, PolynomialCurveSet2, Real, Similarity2};
use hyperreal::RealSign;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i64]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn quintic() -> Curve2 {
    // F = y^5 + 5y + 5 - x, with F_y = 5y^4 + 5 > 0.
    let graph = decompose_bivariate_curve(
        &polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set =
        PolynomialCurveSet2::from_topology(graph, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    assert_eq!(set.curves().len(), 1);
    set.curves()[0].clone()
}

fn on(point: &CurvePoint2, equation: &BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(equation.clone(), polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

fn check_image(transform: Similarity2, equation: BivariatePolynomial, reflected: bool) {
    let source = quintic();
    let native = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, -2), Point2::from_values(0, 2)).unwrap(),
    );
    // The source-coordinate line is an independently admitted control.
    let control = source.intersect_curve(&native).unwrap();
    assert_eq!(control.contacts().len(), 1);
    let image = source.transform_similarity(&transform).unwrap();
    assert_eq!(
        image.polynomial_branch().unwrap().topology().equation(),
        source.polynomial_branch().unwrap().topology().equation(),
    );
    let cut = native.transform_similarity(&transform).unwrap();
    for reverse_branch in [false, true] {
        let curve = if reverse_branch {
            image.reversed().unwrap()
        } else {
            image.clone()
        };
        for reverse_line in [false, true] {
            let line = if reverse_line {
                cut.reversed().unwrap()
            } else {
                cut.clone()
            };
            for (first, second, branch_first) in [(&curve, &line, true), (&line, &curve, false)] {
                let topology = first.intersection_topology(second).unwrap();
                let result = topology.result();
                assert!(result.is_complete());
                assert!(result.overlaps().is_empty());
                assert_eq!(result.contacts().len(), 1);
                assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                let contact = &result.contacts()[0];
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(
                        if reflected ^ reverse_branch ^ reverse_line ^ !branch_first {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        }
                    )
                );
                let parameter = if branch_first {
                    contact.first()
                } else {
                    contact.second()
                }
                .parameter()
                .unwrap();
                assert_eq!(
                    parameter.compare(&Real::zero().into()).unwrap(),
                    Ordering::Equal
                );
                on(contact.point(), &equation);
                for (curve, parameter) in [
                    (first, contact.first().parameter().unwrap()),
                    (second, contact.second().parameter().unwrap()),
                ] {
                    assert!(
                        curve
                            .point_at(&parameter)
                            .unwrap()
                            .coincides_with(contact.point())
                            .unwrap()
                    );
                }
                assert!(
                    contact
                        .point()
                        .definition()
                        .unwrap()
                        .into_point()
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
                let next = topology.first()[0]
                    .intersection_topology(&topology.second()[1])
                    .unwrap();
                assert!(next.result().is_complete());
                assert_eq!(next.result().contacts().len(), 1);
                assert_eq!((next.first().len(), next.second().len()), (1, 1));
            }
        }
    }
}

#[test]
fn polynomial_line_sections_consume_algebraic_scale() {
    let s = Real::from(2).sqrt().unwrap();
    check_image(
        Similarity2::try_from_real_affine(
            s.clone(),
            0.into(),
            0.into(),
            s.clone(),
            0.into(),
            0.into(),
        )
        .unwrap(),
        // X = sqrt(2)x, Y = sqrt(2)y: Y^5 + 20Y + 20sqrt(2) - 4X = 0.
        BivariatePolynomial::new(vec![
            vec![
                Real::from(20) * s,
                20.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ],
            vec![(-4).into()],
        ]),
        false,
    );
}

#[test]
fn polynomial_line_sections_consume_algebraic_translation() {
    let s = Real::from(2).sqrt().unwrap();
    let r = Real::from(3).sqrt().unwrap();
    check_image(
        Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            r.clone(),
            s.clone(),
        )
        .unwrap(),
        // X = x + sqrt(3), Y = y + sqrt(2); expanded independently.
        BivariatePolynomial::new(vec![
            vec![
                Real::from(5) + r - Real::from(9) * &s,
                25.into(),
                -Real::from(20) * &s,
                20.into(),
                -Real::from(5) * s,
                1.into(),
            ],
            vec![(-1).into()],
        ]),
        false,
    );
}

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn scale(value: Real, reflected: bool) -> Similarity2 {
    Similarity2::try_from_real_affine(
        if reflected {
            -value.clone()
        } else {
            value.clone()
        },
        0.into(),
        0.into(),
        value,
        0.into(),
        0.into(),
    )
    .unwrap()
}

#[test]
fn polynomial_line_sections_keep_higher_degree_scale_and_reflection() {
    let s = Real::from(2).root_n(7).unwrap();
    let square = &s * &s;
    let fourth = &square * &square;
    let fifth = &fourth * &s;
    for reflected in [false, true] {
        check_image(
            scale(s.clone(), reflected),
            BivariatePolynomial::new(vec![
                vec![
                    Real::from(5) * &fifth,
                    Real::from(5) * &fourth,
                    0.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                ],
                vec![if reflected {
                    fourth.clone()
                } else {
                    -fourth.clone()
                }],
            ]),
            reflected,
        );
    }
}

fn graphs(equation: BivariatePolynomial) -> PolynomialCurveSet2 {
    let graph = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-3).into(), 3.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(graph, AlgebraicFiberRootIsolationConfig::default()).unwrap()
}

#[test]
fn polynomial_line_sections_keep_oblique_algebraic_images_and_tangency() {
    let source = graphs(polynomial(&[&[0, 1], &[], &[-1]])).curves()[0].clone();
    let a = Real::from(2).sqrt().unwrap();
    let b = Real::from(3).sqrt().unwrap();
    for reflected in [false, true] {
        // X = ax - by, Y = bx + ay; reflection negates X.
        let transform = Similarity2::try_from_real_affine(
            if reflected { -a.clone() } else { a.clone() },
            if reflected { b.clone() } else { -b.clone() },
            b.clone(),
            a.clone(),
            0.into(),
            0.into(),
        )
        .unwrap();
        let image = source.transform_similarity(&transform).unwrap();
        let world = BivariatePolynomial::new(vec![
            vec![0.into(), Real::from(5) * &a, (-3).into()],
            vec![
                if reflected {
                    Real::from(5) * &b
                } else {
                    -Real::from(5) * &b
                },
                (if reflected {
                    Real::from(2)
                } else {
                    Real::from(-2)
                }) * &a
                    * &b,
            ],
            vec![(-2).into()],
        ]);
        for tangent in [false, true] {
            let cut = if tangent {
                LineSeg2::try_new(Point2::from_values(-1, 0), Point2::from_values(1, 0)).unwrap()
            } else {
                LineSeg2::try_new(
                    Point2::new(q(1, 2), (-2).into()),
                    Point2::new(q(1, 2), 2.into()),
                )
                .unwrap()
            };
            let cut = Curve2::from(cut).transform_similarity(&transform).unwrap();
            let (x, y) = if tangent {
                (Real::zero(), Real::zero())
            } else {
                (q(1, 2), q(1, 4))
            };
            let expected = Point2::new(
                (if reflected { -Real::one() } else { Real::one() }) * (&a * &x - &b * &y),
                &b * &x + &a * &y,
            );
            for reversed in [false, true] {
                let curve = if reversed {
                    image.reversed().unwrap()
                } else {
                    image.clone()
                };
                for (first, second, branch_first) in [(&curve, &cut, true), (&cut, &curve, false)] {
                    let topology = first.intersection_topology(second).unwrap();
                    assert!(topology.result().is_complete());
                    assert_eq!(topology.result().contacts().len(), 1);
                    let contact = &topology.result().contacts()[0];
                    let sign = if tangent {
                        RealSign::Zero
                    } else if reflected ^ reversed ^ !branch_first {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    };
                    assert_eq!(contact.tangent_cross_sign(), Some(sign));
                    assert!(
                        contact
                            .point()
                            .coincides_with(&CurvePoint2::from(expected.clone()))
                            .unwrap()
                    );
                    on(contact.point(), &world);
                    let parameter = if branch_first {
                        contact.first()
                    } else {
                        contact.second()
                    }
                    .parameter()
                    .unwrap();
                    assert_eq!(
                        parameter.compare(&x.clone().into()).unwrap(),
                        Ordering::Equal
                    );
                    let next = topology.first()[0]
                        .intersection_topology(&topology.second()[1])
                        .unwrap();
                    assert!(next.result().is_complete());
                    assert_eq!(next.result().contacts().len(), 1);
                    assert_eq!((next.first().len(), next.second().len()), (1, 1));
                }
            }
        }
    }
}

#[test]
fn polynomial_line_sections_preserve_selected_trim_boundaries_in_algebraic_images() {
    let alpha = Real::from(2).sqrt().unwrap() * q(1, 2);
    let source = quintic()
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(alpha.clone().into(), q(4, 5).into())
                .unwrap(),
        )
        .unwrap();
    let transform = scale(Real::from(3).sqrt().unwrap(), true);
    let image = source.transform_similarity(&transform).unwrap();
    let source_cut = Curve2::from(
        LineSeg2::try_new(
            Point2::new(alpha.clone(), (-2).into()),
            Point2::new(alpha.clone(), 2.into()),
        )
        .unwrap(),
    );
    let cut = source_cut.transform_similarity(&transform).unwrap();
    let expected = image.point_at(&alpha.clone().into()).unwrap();
    for curve in [image.clone(), image.reversed().unwrap()] {
        for (first, second, branch_first) in [(&curve, &cut, true), (&cut, &curve, false)] {
            let topology = first.intersection_topology(second).unwrap();
            assert!(topology.result().is_complete());
            assert_eq!(topology.result().contacts().len(), 1);
            assert_eq!(
                (topology.first().len(), topology.second().len()),
                if branch_first { (1, 2) } else { (2, 1) }
            );
            let contact = &topology.result().contacts()[0];
            let parameter = if branch_first {
                contact.first()
            } else {
                contact.second()
            }
            .parameter()
            .unwrap();
            assert_eq!(parameter, curve.parameter_domain().start().clone());
            assert!(contact.point().coincides_with(&expected).unwrap());
            assert!(
                contact
                    .point()
                    .definition()
                    .unwrap()
                    .into_point()
                    .unwrap()
                    .coincides_with(&expected)
                    .unwrap()
            );
        }
    }
    // A transformed but excluded source abscissa is still outside the trim.
    let outside = Curve2::from(
        LineSeg2::try_new(
            Point2::new(q(1, 2), (-2).into()),
            Point2::new(q(1, 2), 2.into()),
        )
        .unwrap(),
    )
    .transform_similarity(&transform)
    .unwrap();
    assert!(image.intersect_curve(&outside).unwrap().is_disjoint());
}

#[test]
fn polynomial_line_components_keep_algebraic_image_overlap_and_distinct_neighbors() {
    let branches = graphs(polynomial(&[&[-2, 0, 1]]));
    assert_eq!(branches.curves().len(), 2);
    let height = -Real::from(2).sqrt().unwrap();
    let transform = scale(Real::from(3).sqrt().unwrap(), true);
    let image = branches.curves()[0]
        .transform_similarity(&transform)
        .unwrap();
    let cut = Curve2::from(
        LineSeg2::try_new(
            Point2::new(q(-1, 2), height.clone()),
            Point2::new(q(1, 2), height.clone()),
        )
        .unwrap(),
    )
    .transform_similarity(&transform)
    .unwrap();
    for curve in [image.clone(), image.reversed().unwrap()] {
        for (first, second, branch_first) in [(&curve, &cut, true), (&cut, &curve, false)] {
            let topology = first.intersection_topology(second).unwrap();
            assert!(topology.result().is_complete());
            assert_eq!(topology.result().overlaps().len(), 1);
            assert_eq!(
                (topology.first().len(), topology.second().len()),
                if branch_first { (3, 1) } else { (1, 3) }
            );
            let (branch, line) = if branch_first {
                (&topology.first()[1], &topology.second()[0])
            } else {
                (&topology.second()[1], &topology.first()[0])
            };
            let next = line.intersection_topology(branch).unwrap();
            assert!(next.result().is_complete());
            assert_eq!(next.result().overlaps().len(), 1);
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
        }
    }
    let tiny = q(1, 1_i64 << 40) * q(1, 1_i64 << 40);
    for shift in [tiny.clone(), -tiny] {
        let nearby = Curve2::from(
            LineSeg2::try_new(
                Point2::new(q(-1, 2), &height + &shift),
                Point2::new(q(1, 2), &height + &shift),
            )
            .unwrap(),
        )
        .transform_similarity(&transform)
        .unwrap();
        assert!(image.intersect_curve(&nearby).unwrap().is_disjoint());
    }
}

#[test]
fn polynomial_algebraic_images_retry_after_reached_budget_and_retain_certainty() {
    let transform = scale(Real::from(2).sqrt().unwrap(), false);
    let (branch_piece, line_piece, point) = {
        let curve = quintic().transform_similarity(&transform).unwrap();
        let native = Curve2::from(
            LineSeg2::try_new(
                Point2::new((-1).into(), -Real::from(2).sqrt().unwrap() * q(1, 2)),
                Point2::from_values(1, -1),
            )
            .unwrap(),
        )
        .transform_similarity(&transform)
        .unwrap();
        let denied = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget::ZERO,
            || {
                assert!(
                    matches!(curve.intersect_curve(&native).unwrap_err(), hypercurve::ExactCurveError::Blocked(blocker)
                if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
                );
            },
        );
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 100_000,
                promotions: 100_000,
            },
            || curve.intersection_topology(&native),
        );
        let topology = retry.result.unwrap().unwrap();
        assert!(topology.result().is_complete());
        assert_eq!(topology.result().contacts().len(), 1);
        (
            topology.first()[0].clone(),
            topology.second()[1].clone(),
            topology.result().contacts()[0].point().clone(),
        )
    };
    let replay = point.definition().unwrap().into_point().unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let next = line_piece.intersection_topology(&branch_piece).unwrap();
            assert!(next.result().is_complete());
            assert_eq!(next.result().contacts().len(), 1);
            assert!(
                next.result().contacts()[0]
                    .point()
                    .coincides_with(&replay)
                    .unwrap()
            );
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}
