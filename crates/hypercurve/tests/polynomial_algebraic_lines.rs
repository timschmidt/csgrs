//! Algebraic-coordinate line sections keep the original selected polynomial.

use hypercurve::{
    Curve2, CurveOverlapOrientation2, CurvePoint2, LineSeg2, Point2, PolynomialCurveSet2, Real,
    Similarity2,
};
use hyperreal::RealSign;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn equation() -> BivariatePolynomial {
    // y^5 + 5y + 5 - x; F_y = 5y^4 + 5 is strictly positive.
    BivariatePolynomial::new(vec![
        vec![5.into(), 5.into(), 0.into(), 0.into(), 0.into(), 1.into()],
        vec![(-1).into()],
    ])
}

fn curves(equation: BivariatePolynomial, y: [Real; 2]) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: y,
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

fn quintic() -> Curve2 {
    let set = curves(equation(), [(-2).into(), 2.into()]);
    assert_eq!(set.curves().len(), 1);
    set.curves()[0].clone()
}

fn line(x0: Real, y0: Real, x1: Real, y1: Real) -> Curve2 {
    LineSeg2::try_new(Point2::new(x0, y0), Point2::new(x1, y1))
        .unwrap()
        .into()
}

fn alpha() -> Real {
    (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap()
}

fn abscissas() -> [Real; 4] {
    [
        alpha(),
        Real::from(2).cbrt().unwrap() * q(1, 2),
        Real::from(2).root_n(7).unwrap() * q(1, 2),
        (Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap()) * q(1, 4),
    ]
}

fn on_quintic(point: &CurvePoint2) {
    assert_eq!(
        point
            .rational_form(
                equation(),
                BivariatePolynomial::new(vec![vec![Real::one()]])
            )
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn algebraic_abscissa_keeps_the_original_quintic_fiber() {
    let curve = quintic();
    for x in abscissas() {
        let point = curve.point_at(&x.clone().into()).unwrap();
        on_quintic(&point);
        assert_eq!(
            point
                .affine_form(1.into(), 0.into(), 0.into())
                .compare_to_real(&x)
                .unwrap(),
            Ordering::Equal
        );
        assert!(
            point
                .definition()
                .unwrap()
                .into_point()
                .unwrap()
                .coincides_with(&point)
                .unwrap()
        );
    }
}

#[test]
fn original_irrational_vertical_line_splits_and_requeries() {
    let source = quintic();
    for x in abscissas() {
        for (lower, upper, descending) in [
            ((-3).into(), 3.into(), false),
            (
                -Real::from(3).sqrt().unwrap(),
                Real::from(3).sqrt().unwrap(),
                false,
            ),
            (
                Real::from(3).sqrt().unwrap(),
                -Real::from(3).sqrt().unwrap(),
                true,
            ),
        ] {
            // The first entry retains the original irrational vertical frontier input.
            let cut = line(x.clone(), lower, x.clone(), upper);
            for reversed in [false, true] {
                let curve = if reversed {
                    source.reversed().unwrap()
                } else {
                    source.clone()
                };
                for (first, second, branch_first) in [(&curve, &cut, true), (&cut, &curve, false)] {
                    let topology = first.intersection_topology(second).unwrap();
                    assert!(topology.result().is_complete());
                    assert_eq!(topology.result().contacts().len(), 1);
                    assert!(topology.result().overlaps().is_empty());
                    assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                    let contact = &topology.result().contacts()[0];
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(if reversed ^ descending ^ !branch_first {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        })
                    );
                    assert!(contact.is_certified_transverse());
                    on_quintic(contact.point());
                    assert_eq!(
                        contact
                            .point()
                            .affine_form(1.into(), 0.into(), 0.into())
                            .compare_to_real(&x)
                            .unwrap(),
                        Ordering::Equal
                    );
                    for (source, parameter) in [
                        (first, contact.first().parameter().unwrap()),
                        (second, contact.second().parameter().unwrap()),
                    ] {
                        assert!(
                            source
                                .point_at(&parameter)
                                .unwrap()
                                .coincides_with(contact.point())
                                .unwrap()
                        );
                    }
                    let next = topology.first()[0]
                        .intersection_topology(&topology.second()[1])
                        .unwrap();
                    assert_eq!((next.first().len(), next.second().len()), (1, 1));
                    assert_eq!(next.result().contacts().len(), 1);
                }
            }
        }
    }
}

#[test]
fn original_irrational_sloped_line_is_certifiably_disjoint() {
    // The original frontier input is entirely above y=0, while the unique
    // quintic root is below y=0 throughout x in [-1,1].
    let curve = quintic();
    let cut = line((-1).into(), alpha(), 1.into(), 3.into());
    for (first, second) in [(&curve, &cut), (&cut, &curve)] {
        assert!(first.intersect_curve(second).unwrap().is_disjoint());
        let topology = first.intersection_topology(second).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        assert!(topology.result().is_disjoint());
    }
}

#[test]
fn algebraic_sloped_line_keeps_its_selected_contact() {
    let curve = quintic();
    for alpha in [
        alpha(),
        Real::from(2).cbrt().unwrap() * q(1, 2),
        Real::from(2).root_n(7).unwrap() * q(1, 2),
        (Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap()) * q(1, 8),
    ] {
        let cut = line((-1).into(), -alpha.clone(), 1.into(), (-1).into());
        // y = (alpha-1)x/2 - (alpha+1)/2; the decreasing line crosses the
        // increasing quintic graph exactly once, at a positive x below 1.
        let slope = (&alpha - Real::one()) * q(1, 2);
        let offset = -(&alpha + Real::one()) * q(1, 2);
        for (first, second) in [(&curve, &cut), (&cut, &curve)] {
            let topology = first.intersection_topology(second).unwrap();
            assert!(topology.result().is_complete());
            assert_eq!(topology.result().contacts().len(), 1);
            assert!(topology.result().overlaps().is_empty());
            let contact = &topology.result().contacts()[0];
            on_quintic(contact.point());
            assert_eq!(
                contact
                    .point()
                    .affine_form(-slope.clone(), 1.into(), -offset.clone())
                    .compare_to_real(&Real::zero())
                    .unwrap(),
                Ordering::Equal
            );
            let x = contact.point().affine_form(1.into(), 0.into(), 0.into());
            assert_eq!(x.compare_to_real(&Real::zero()).unwrap(), Ordering::Greater);
            assert_eq!(x.compare_to_real(&Real::one()).unwrap(), Ordering::Less);
            for (source, parameter) in [
                (first, contact.first().parameter().unwrap()),
                (second, contact.second().parameter().unwrap()),
            ] {
                assert!(
                    source
                        .point_at(&parameter)
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
            }
            for piece in topology.first() {
                assert!(piece.intersect_curve(second).unwrap().is_complete());
            }
        }
    }
}

#[test]
fn algebraic_component_lines_keep_selected_overlaps() {
    let root2 = Real::from(2).sqrt().unwrap();
    let sum = &root2 + Real::from(3).sqrt().unwrap();
    // The second equation also has the two inner roots +- (sqrt(3)-sqrt(2)).
    // Only its outer selected branches can coincide with the tested lines.
    for (polynomial, height, selected, count) in [
        (
            BivariatePolynomial::new(vec![vec![(-2).into(), 0.into(), 1.into()]]),
            root2,
            [0, 1],
            2,
        ),
        (
            BivariatePolynomial::new(vec![vec![
                1.into(),
                0.into(),
                (-10).into(),
                0.into(),
                1.into(),
            ]]),
            sum,
            [0, 3],
            4,
        ),
    ] {
        let set = curves(polynomial, [(-4).into(), 4.into()]);
        assert_eq!(set.curves().len(), count);
        for (height, selected) in [(-height.clone(), selected[0]), (height, selected[1])] {
            for descending in [false, true] {
                let width = Real::from(3).sqrt().unwrap();
                let (a, b) = if descending {
                    (width.clone(), -width)
                } else {
                    (-width.clone(), width)
                };
                let native = line(a, height.clone(), b, height.clone());
                for (index, branch) in set.curves().iter().enumerate() {
                    for (first, second, branch_first) in
                        [(branch, &native, true), (&native, branch, false)]
                    {
                        let topology = first.intersection_topology(second).unwrap();
                        assert!(topology.result().is_complete());
                        if index != selected {
                            assert!(topology.result().is_disjoint());
                            continue;
                        }
                        assert!(topology.result().contacts().is_empty());
                        assert_eq!(
                            topology.result().overlaps().len(),
                            1,
                            "{count} branches, selected {selected}, index {index}, descending {descending}, branch_first {branch_first}"
                        );
                        let overlap = &topology.result().overlaps()[0];
                        assert_eq!(
                            overlap.orientation(),
                            if descending {
                                CurveOverlapOrientation2::Reversed
                            } else {
                                CurveOverlapOrientation2::Same
                            }
                        );
                        assert_eq!(
                            (topology.first().len(), topology.second().len()),
                            if branch_first { (1, 3) } else { (3, 1) }
                        );
                        for (s, t) in [
                            (
                                overlap.first_range().start(),
                                overlap.second_range().start(),
                            ),
                            (overlap.first_range().end(), overlap.second_range().end()),
                        ] {
                            let point = first.point_at(s).unwrap();
                            assert!(point.coincides_with(&second.point_at(t).unwrap()).unwrap());
                            assert_eq!(
                                point
                                    .affine_form(0.into(), 1.into(), 0.into())
                                    .compare_to_real(&height)
                                    .unwrap(),
                                Ordering::Equal
                            );
                        }
                        let (branch_piece, line_piece) = if branch_first {
                            (&topology.first()[0], &topology.second()[1])
                        } else {
                            (&topology.second()[0], &topology.first()[1])
                        };
                        let next = line_piece.intersection_topology(branch_piece).unwrap();
                        assert_eq!((next.first().len(), next.second().len()), (1, 1));
                        assert_eq!(next.result().overlaps().len(), 1);
                    }
                }
                let tiny = q(1, 1_i64 << 40) * q(1, 1_i64 << 40);
                let shifted = line((-2).into(), &height + &tiny, 2.into(), &height + &tiny);
                assert!(
                    set.curves()[selected]
                        .intersect_curve(&shifted)
                        .unwrap()
                        .is_disjoint()
                );
            }
        }
    }
}

#[test]
fn algebraic_line_sections_survive_rational_similarities() {
    let source = quintic();
    let native = line((-1).into(), -alpha(), 1.into(), (-1).into());
    let m = (alpha() - Real::one()) * q(1, 2);
    let b = -(alpha() + Real::one()) * q(1, 2);
    for (transform, polynomial, line_form, reflected) in [
        (
            Similarity2::try_from_real_affine(
                0.into(),
                (-1).into(),
                1.into(),
                0.into(),
                3.into(),
                4.into(),
            )
            .unwrap(),
            // u = y-4, v = 3-x in v^5 + 5v + 5 - u.
            BivariatePolynomial::new(vec![
                vec![267.into(), (-1).into()],
                vec![(-410).into()],
                vec![270.into()],
                vec![(-90).into()],
                vec![15.into()],
                vec![(-1).into()],
            ]),
            [
                -Real::one(),
                -m.clone(),
                Real::from(3) + Real::from(4) * &m - &b,
            ],
            false,
        ),
        (
            Similarity2::try_from_real_affine(
                (-1).into(),
                0.into(),
                0.into(),
                1.into(),
                2.into(),
                (-3).into(),
            )
            .unwrap(),
            // u = 2-x, v = y+3 in the same source equation.
            BivariatePolynomial::new(vec![
                vec![
                    261.into(),
                    410.into(),
                    270.into(),
                    90.into(),
                    15.into(),
                    1.into(),
                ],
                vec![1.into()],
            ]),
            [
                m.clone(),
                Real::one(),
                Real::from(3) - Real::from(2) * &m - &b,
            ],
            true,
        ),
    ] {
        let native = native.transform_similarity(&transform).unwrap();
        let curve = source.transform_similarity(&transform).unwrap();
        for reversed in [false, true] {
            let curve = if reversed {
                curve.reversed().unwrap()
            } else {
                curve.clone()
            };
            for (first, second, branch_first) in [(&curve, &native, true), (&native, &curve, false)]
            {
                let topology = first.intersection_topology(second).unwrap();
                assert!(topology.result().is_complete());
                assert_eq!(topology.result().contacts().len(), 1);
                let contact = &topology.result().contacts()[0];
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if reflected ^ reversed ^ !branch_first {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    })
                );
                assert_eq!(
                    contact
                        .point()
                        .rational_form(
                            polynomial.clone(),
                            BivariatePolynomial::new(vec![vec![Real::one()]])
                        )
                        .unwrap()
                        .compare_to_real(&Real::zero())
                        .unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    contact
                        .point()
                        .affine_form(
                            line_form[0].clone(),
                            line_form[1].clone(),
                            line_form[2].clone()
                        )
                        .compare_to_real(&Real::zero())
                        .unwrap(),
                    Ordering::Equal
                );
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
                assert_eq!((next.first().len(), next.second().len()), (1, 1));
                assert_eq!(next.result().contacts().len(), 1);
            }
        }
    }
}

#[test]
fn algebraic_line_sections_retry_and_retain_certainty() {
    let (branch_piece, line_piece, point) = {
        let curve = quintic();
        let native = line((-1).into(), -alpha(), 1.into(), (-1).into());
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
                let error = curve.intersect_curve(&native).unwrap_err();
                assert!(
                    matches!(error, hypercurve::ExactCurveError::Blocked(blocker)
                    if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
                );
            });
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
    // The original curve, line and topology have been dropped. Retained
    // pieces own the selected source and remain exact under every policy.
    let replay = point.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let next = line_piece.intersection_topology(&branch_piece).unwrap();
            assert!(next.result().is_complete());
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert_eq!(next.result().contacts().len(), 1);
            assert!(
                next.result().contacts()[0]
                    .point()
                    .coincides_with(&replay)
                    .unwrap()
            );
            on_quintic(next.result().contacts()[0].point());
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}
