//! Shared algebraic similarities preserve selected polynomial pair incidence.

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BooleanOp, Curve2, CurveOverlapOrientation2, CurveParameter2,
    CurveParameterRange2, CurvePath2, CurvePoint2, CurveRegion2, FillRule, Point2,
    PolynomialCurveSet2, Real, RegionPointLocation, Similarity2,
};
use hyperreal::RealSign;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn scale(s: Real, reflected: bool) -> Similarity2 {
    Similarity2::try_from_real_affine(
        if reflected { -s.clone() } else { s.clone() },
        0.into(),
        0.into(),
        s,
        0.into(),
        0.into(),
    )
    .unwrap()
}

fn polynomial(rows: &[&[i64]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn curves(equation: BivariatePolynomial) -> Vec<Curve2> {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, Default::default())
        .unwrap()
        .curves()
        .to_vec()
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

fn check_image(transform: Similarity2, world_f: BivariatePolynomial, world_g: BivariatePolynomial) {
    let f = polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]);
    let g = polynomial(&[&[0, 0, 1], &[-1]]);
    let first = curves(f.clone());
    let second = curves(g.clone());
    assert_eq!((first.len(), second.len()), (1, 2));
    let a = &first[0];
    let b = &second[0];
    let control = a.intersection_topology(b).unwrap();
    assert!(control.result().is_complete());
    assert_eq!(control.result().contacts().len(), 1);
    let (aa, ab, ba, bb, tx, ty) = transform.affine_components();
    let independent = Similarity2::try_from_real_affine(
        aa.clone(),
        ab.clone(),
        ba.clone(),
        bb.clone(),
        tx.clone(),
        ty.clone(),
    )
    .unwrap();
    let image_a = a.transform_similarity(&transform).unwrap();
    let image_b = b.transform_similarity(&independent).unwrap();
    for (image, equation) in [(&image_a, &f), (&image_b, &g)] {
        assert_eq!(
            image.polynomial_branch().unwrap().topology().equation(),
            equation
        );
    }
    for reverse_a in [false, true] {
        for reverse_b in [false, true] {
            let a = if reverse_a {
                image_a.reversed().unwrap()
            } else {
                image_a.clone()
            };
            let b = if reverse_b {
                image_b.reversed().unwrap()
            } else {
                image_b.clone()
            };
            for (a, b, swapped) in [(&a, &b, false), (&b, &a, true)] {
                let topology = a.intersection_topology(b).unwrap();
                assert!(topology.result().is_complete());
                assert!(topology.result().overlaps().is_empty());
                assert_eq!(topology.result().contacts().len(), 1);
                assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                let contact = &topology.result().contacts()[0];
                on(contact.point(), &world_f);
                on(contact.point(), &world_g);
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(
                        if transform.reverses_orientation() ^ reverse_a ^ reverse_b ^ swapped {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        }
                    )
                );
                let s = contact.first().parameter().unwrap();
                let t = contact.second().parameter().unwrap();
                assert_eq!(s.compare(&t).unwrap(), Ordering::Equal);
                // Eliminating y gives x^5+10x^3-x^2+35x-25. The retained
                // contact is the root in (3/5,7/10), on the negative y branch.
                let low = (Real::from(3) / Real::from(5)).unwrap();
                let high = (Real::from(7) / Real::from(10)).unwrap();
                assert_eq!(s.compare(&low.into()).unwrap(), Ordering::Greater);
                assert_eq!(s.compare(&high.into()).unwrap(), Ordering::Less);
                assert!(
                    a.point_at(&s)
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
                assert!(
                    b.point_at(&t)
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
                let replay = contact.point().definition().unwrap().into_point().unwrap();
                assert!(replay.coincides_with(contact.point()).unwrap());
                let next = topology.first()[0]
                    .intersection_topology(&topology.second()[1])
                    .unwrap();
                assert!(next.result().is_complete());
                assert_eq!((next.first().len(), next.second().len()), (1, 1));
                assert_eq!(next.result().contacts().len(), 1);
                assert!(
                    next.result().contacts()[0]
                        .point()
                        .coincides_with(&replay)
                        .unwrap()
                );
            }
        }
    }
}

#[test]
fn polynomial_pairs_consume_shared_algebraic_scale() {
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
        // X=sqrt(2)x, Y=sqrt(2)y.
        BivariatePolynomial::new(vec![
            vec![
                Real::from(20) * &s,
                20.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ],
            vec![(-4).into()],
        ]),
        BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![-s]]),
    );
}

#[test]
fn polynomial_pairs_consume_shared_algebraic_translation() {
    let a = Real::from(3).sqrt().unwrap();
    let b = Real::from(2).sqrt().unwrap();
    check_image(
        Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            a.clone(),
            b.clone(),
        )
        .unwrap(),
        // X=x+sqrt(3), Y=y+sqrt(2); expanded independently in world coordinates.
        BivariatePolynomial::new(vec![
            vec![
                Real::from(5) + &a - Real::from(9) * &b,
                25.into(),
                -Real::from(20) * &b,
                20.into(),
                -Real::from(5) * &b,
                1.into(),
            ],
            vec![(-1).into()],
        ]),
        BivariatePolynomial::new(vec![
            vec![Real::from(2) + a, -Real::from(2) * b, 1.into()],
            vec![(-1).into()],
        ]),
    );
}

#[test]
fn polynomial_pairs_keep_higher_degree_images_and_reflections() {
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
            BivariatePolynomial::new(vec![
                vec![0.into(), 0.into(), 1.into()],
                vec![if reflected { s.clone() } else { -s.clone() }],
            ]),
        );
    }
}

#[test]
fn polynomial_pairs_keep_selected_overlaps_under_equal_algebraic_images() {
    let f = curves(polynomial(&[&[0, 0, 1], &[-1]]));
    let g = curves(polynomial(&[&[0, 0, -3, 1], &[3, -1]]));
    let beta = CurveParameter2::from(BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![(-1).into(), 0.into(), 2.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), 1.into()).unwrap(),
        )
        .unwrap(),
    ));
    let a = f[0]
        .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), beta.clone()).unwrap())
        .unwrap();
    let b = g[0]
        .restrict_source(CurveParameterRange2::try_new(q(1, 2).into(), Real::one().into()).unwrap())
        .unwrap();
    let first_scale = Real::from(2).sqrt().unwrap();
    let second_scale = (Real::from(8).sqrt().unwrap() / Real::from(2)).unwrap();
    assert_eq!(
        CurveParameter2::from(first_scale.clone())
            .compare(&second_scale.clone().into())
            .unwrap(),
        Ordering::Equal
    );
    let a = a.transform_similarity(&scale(first_scale, true)).unwrap();
    let b = b.transform_similarity(&scale(second_scale, true)).unwrap();
    for reversed in [false, true] {
        let a = if reversed {
            a.reversed().unwrap()
        } else {
            a.clone()
        };
        for (a, b) in [(&a, &b), (&b, &a)] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().is_complete());
            assert!(topology.result().contacts().is_empty());
            assert_eq!(topology.result().overlaps().len(), 1);
            let overlap = &topology.result().overlaps()[0];
            assert_eq!(
                overlap.orientation(),
                if reversed {
                    CurveOverlapOrientation2::Reversed
                } else {
                    CurveOverlapOrientation2::Same
                }
            );
            let clipped = overlap
                .restrict(
                    [q(3, 5).into(), beta.clone()],
                    [q(1, 2).into(), q(3, 4).into()],
                )
                .unwrap()
                .unwrap();
            assert_eq!(clipped.orientation(), overlap.orientation());
            for endpoint in [clipped.first_range().start(), clipped.first_range().end()] {
                assert!(
                    endpoint.compare(&q(3, 5).into()).unwrap() == Ordering::Equal
                        || endpoint.compare(&beta).unwrap() == Ordering::Equal
                );
            }
            for (s, t) in [
                (
                    clipped.first_range().start(),
                    clipped.second_range().start(),
                ),
                (clipped.first_range().end(), clipped.second_range().end()),
            ] {
                assert_eq!(s.compare(t).unwrap(), Ordering::Equal);
                let point = a.point_at(s).unwrap();
                assert!(point.coincides_with(&b.point_at(t).unwrap()).unwrap());
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
            let range = CurveParameterRange2::try_new(q(3, 5).into(), beta.clone()).unwrap();
            let next = a
                .restrict_source(range.clone())
                .unwrap()
                .intersection_topology(&b.restrict_source(range).unwrap())
                .unwrap();
            assert!(next.result().is_complete());
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert_eq!(next.result().overlaps().len(), 1);
        }
    }
}

#[test]
fn polynomial_pairs_keep_tangency_and_distinct_supports_in_algebraic_images() {
    let f = curves(polynomial(&[&[0, 1], &[0], &[-1]]));
    let tangent = curves(polynomial(&[&[0, 1]]));
    let delta = q(1, 1 << 20) * q(1, 1 << 20) * q(1, 1 << 20) * q(1, 1 << 20);
    let shifted = curves(BivariatePolynomial::new(vec![
        vec![-delta.clone(), 1.into()],
        vec![0.into()],
        vec![(-1).into()],
    ]));
    let s = Real::from(2).root_n(7).unwrap();
    let map = scale(s.clone(), true);
    let a = f[0].transform_similarity(&map).unwrap();
    let b = tangent[0].transform_similarity(&map).unwrap();
    for (a, b) in [(&a, &b), (&b, &a)] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete());
        assert_eq!(topology.result().contacts().len(), 1);
        assert!(topology.result().overlaps().is_empty());
        let contact = &topology.result().contacts()[0];
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Zero));
        assert!(
            contact
                .point()
                .coincides_with(&Point2::from_values(0, 0).into())
                .unwrap()
        );
    }
    let b = shifted[0].transform_similarity(&map).unwrap();
    for (a, b) in [(&a, &b), (&b, &a)] {
        assert!(a.intersection_topology(b).unwrap().result().is_disjoint());
    }
    // A tiny change in the transform is a different chart, even after a
    // successful query has warmed the original source's evidence.
    let changed =
        Similarity2::try_from_real_affine(-s.clone(), 0.into(), 0.into(), s, delta, 0.into())
            .unwrap();
    let different_image = f[0].transform_similarity(&changed).unwrap();
    for (a, b) in [(&a, &different_image), (&different_image, &a)] {
        let error = a.intersect_curve(b).unwrap_err();
        assert!(
            matches!(error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason()==hypercurve::UncertaintyReason::Unsupported)
        );
    }
    assert!(a.intersect_curve(&a).unwrap().is_complete());
}

#[test]
fn polynomial_algebraic_pairs_retry_and_keep_certified_results() {
    let (a, b, point) = {
        let first = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
        let second = curves(polynomial(&[&[0, 0, 1], &[-1]]));
        let map = scale(Real::from(2).root_n(7).unwrap(), true);
        let a = first[0].transform_similarity(&map).unwrap();
        let b = second[0].transform_similarity(&map).unwrap();
        let denied = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget::ZERO,
            || {
                let error = a.intersect_curve(&b).unwrap_err();
                assert!(
                    matches!(error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason()==hypercurve::UncertaintyReason::BudgetExhausted)
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
            || a.intersection_topology(&b),
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
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let replay = point.definition().unwrap().into_point().unwrap();
            let next = a.intersection_topology(&b).unwrap();
            assert!(next.result().is_complete());
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
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

fn band(bottom: Curve2, fill: FillRule) -> CurveRegion2 {
    let top_right = Point2::from_values(1, 1);
    let top_left = Point2::from_values(0, 1);
    let path = CurvePath2::try_new(vec![
        bottom.clone(),
        Curve2::try_line(bottom.end(), top_right.clone().into()).unwrap(),
        Curve2::try_line(top_right.into(), top_left.clone().into()).unwrap(),
        Curve2::try_line(top_left.into(), bottom.start()).unwrap(),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths(&[path], fill).unwrap()
}

#[test]
fn shared_algebraic_polynomial_pairs_feed_region_booleans() {
    let f = polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]);
    let g = polynomial(&[&[0, 0, 1], &[-1]]);
    let first = curves(f.clone());
    let second = curves(g.clone());
    let bottom_a = first[0]
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), Real::one().into()).unwrap(),
        )
        .unwrap();
    let bottom_b = &second[0];
    for fill in [FillRule::NonZero, FillRule::EvenOdd] {
        let a = band(bottom_a.clone(), fill);
        let b = band(bottom_b.clone(), fill);
        // Establish the untransformed filled-set control before consuming
        // the same selected supports through an algebraic similarity.
        let control = a.boolean_region(&b, BooleanOp::Intersection).unwrap();
        assert!(!control.is_empty());
        for reflected in [false, true] {
            let map = scale(Real::from(2).sqrt().unwrap(), reflected);
            let a = a.transform_similarity(&map).unwrap();
            let b = b.transform_similarity(&map).unwrap();
            let selected = a.boolean_region(&b, BooleanOp::Intersection).unwrap();
            assert!(!selected.is_empty());
            for (x, y, expected) in [
                (q(1, 2), Real::zero(), RegionPointLocation::Inside),
                (q(1, 4), q(-3, 4), RegionPointLocation::Outside),
                (q(3, 4), q(-13, 16), RegionPointLocation::Outside),
            ] {
                let point = Point2::new(x, y).transform_similarity(&map);
                assert_eq!(selected.classify_point(&point.into()).unwrap(), expected);
            }
            let paths = selected.boundary_paths().unwrap();
            for equation in [&f, &g] {
                assert!(paths.iter().flat_map(|path| path.curves()).any(|curve| {
                    curve
                        .polynomial_branch()
                        .is_some_and(|branch| branch.topology().equation() == equation)
                }));
            }
            let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, fill).unwrap();
            assert!(
                rebuilt
                    .boolean_region(&selected, BooleanOp::Xor)
                    .unwrap()
                    .is_empty()
            );
            assert!(
                selected
                    .boolean_region(&a, BooleanOp::Difference)
                    .unwrap()
                    .is_empty()
            );
        }
    }
}

#[test]
fn oblique_algebraic_images_consume_chords_with_selected_endpoints() {
    let f = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    let g = curves(polynomial(&[&[0, 0, 1], &[-1]]));
    let beta: CurveParameter2 = q(1, 2).sqrt().unwrap().into();
    let source_chord =
        Curve2::try_line(g[0].point_at(&beta).unwrap(), g[1].point_at(&beta).unwrap()).unwrap();
    let control = f[0].intersect_curve(&source_chord).unwrap();
    assert!(control.is_complete());
    assert_eq!(control.contacts().len(), 1);
    assert_eq!(
        control.contacts()[0]
            .first()
            .parameter()
            .unwrap()
            .compare(&beta)
            .unwrap(),
        Ordering::Equal
    );
    let a = Real::from(2).sqrt().unwrap();
    let b = Real::from(3).sqrt().unwrap();
    for reflected in [false, true] {
        let map = Similarity2::try_from_real_affine(
            if reflected { -a.clone() } else { a.clone() },
            -b.clone(),
            if reflected { -b.clone() } else { b.clone() },
            a.clone(),
            Real::from(2).root_n(7).unwrap(),
            -Real::from(5).sqrt().unwrap(),
        )
        .unwrap();
        let image = f[0].transform_similarity(&map).unwrap();
        let low = g[0]
            .transform_similarity(&map)
            .unwrap()
            .point_at(&beta)
            .unwrap();
        let high = g[1]
            .transform_similarity(&map)
            .unwrap()
            .point_at(&beta)
            .unwrap();
        let chord = Curve2::try_line(low, high).unwrap();
        let expected = image.point_at(&beta).unwrap();
        for reverse in [false, true] {
            let image = if reverse {
                image.reversed().unwrap()
            } else {
                image.clone()
            };
            for (first, second, branch_first) in [(&image, &chord, true), (&chord, &image, false)] {
                let topology = first.intersection_topology(second).unwrap();
                assert!(topology.result().is_complete());
                assert!(topology.result().overlaps().is_empty());
                assert_eq!(topology.result().contacts().len(), 1);
                assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                let contact = &topology.result().contacts()[0];
                let branch_parameter = if branch_first {
                    contact.first()
                } else {
                    contact.second()
                }
                .parameter()
                .unwrap();
                assert_eq!(branch_parameter.compare(&beta).unwrap(), Ordering::Equal);
                assert!(contact.point().coincides_with(&expected).unwrap());
                let replay = contact.point().definition().unwrap().into_point().unwrap();
                assert!(replay.coincides_with(&expected).unwrap());
                let next = topology.first()[0]
                    .intersection_topology(&topology.second()[1])
                    .unwrap();
                assert!(next.result().is_complete());
                assert_eq!(next.result().contacts().len(), 1);
                assert!(
                    next.result().contacts()[0]
                        .point()
                        .coincides_with(&replay)
                        .unwrap()
                );
            }
        }
    }
}
