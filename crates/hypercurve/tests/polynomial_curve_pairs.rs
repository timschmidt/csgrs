//! Selected polynomial pairs retain finite contacts and common components.

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveOverlapOrientation2, CurveParameter2,
    CurveParameterRange2, CurvePoint2, Point2, PolynomialCurveSet2, Real,
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

fn polynomial(rows: &[&[i64]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn curves(polynomial: BivariatePolynomial) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &polynomial,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

fn on(point: &CurvePoint2, polynomial: BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(
                polynomial,
                BivariatePolynomial::new(vec![vec![Real::one()]])
            )
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

fn parameter_is(value: &CurveParameter2, expected: Real) {
    assert_eq!(value.compare(&expected.into()).unwrap(), Ordering::Equal);
}

fn rational_contact(first: &Curve2, second: &Curve2, x: Real, y: Real, sign: RealSign) {
    for (first, second, swapped) in [(first, second, false), (second, first, true)] {
        let topology = first.intersection_topology(second).unwrap();
        assert!(topology.result().is_complete());
        assert!(topology.result().overlaps().is_empty());
        assert_eq!(topology.result().contacts().len(), 1);
        let contact = &topology.result().contacts()[0];
        let expected = Point2::new(x.clone(), y.clone()).into();
        assert!(contact.point().coincides_with(&expected).unwrap());
        assert_eq!(
            contact.tangent_cross_sign(),
            Some(if swapped {
                match sign {
                    RealSign::Positive => RealSign::Negative,
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                }
            } else {
                sign
            })
        );
        for location in [contact.first(), contact.second()] {
            parameter_is(&location.parameter().unwrap(), x.clone());
        }
        for piece in topology.first() {
            assert!(piece.intersect_curve(second).unwrap().is_complete());
        }
    }
}

#[test]
fn original_quintic_parabola_pair_splits_and_requeries() {
    let f = polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]);
    let g = polynomial(&[&[0, 0, 1], &[-1]]);
    let first = curves(f.clone());
    let second = curves(g.clone());
    assert_eq!((first.curves().len(), second.curves().len()), (1, 2));
    let quintic = &first.curves()[0];
    // The unchanged frontier pairs the quintic with the POSITIVE parabola
    // branch. The quintic has y<0 throughout its source interval.
    let positive = &second.curves()[1];
    for (first, second) in [(quintic, positive), (positive, quintic)] {
        assert!(first.intersect_curve(second).unwrap().is_disjoint());
        let topology = first.intersection_topology(second).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        assert!(topology.result().is_disjoint());
    }
    let negative = &second.curves()[0];
    for a_reversed in [false, true] {
        for b_reversed in [false, true] {
            let a = if a_reversed {
                quintic.reversed().unwrap()
            } else {
                quintic.clone()
            };
            let b = if b_reversed {
                negative.reversed().unwrap()
            } else {
                negative.clone()
            };
            for (first, second, swapped) in [(&a, &b, false), (&b, &a, true)] {
                let topology = first.intersection_topology(second).unwrap();
                assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
                assert_eq!(topology.result().contacts().len(), 1);
                assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                let contact = &topology.result().contacts()[0];
                on(contact.point(), f.clone());
                on(contact.point(), g.clone());
                // Eliminating y from x=y^2 and y(x^2+5)=x-5 gives this
                // independently derived x equation and isolating interval.
                on(
                    contact.point(),
                    polynomial(&[&[-25], &[35], &[-1], &[10], &[0], &[1]]),
                );
                let x = contact.point().affine_form(1.into(), 0.into(), 0.into());
                assert_eq!(x.compare_to_real(&q(3, 5)).unwrap(), Ordering::Greater);
                assert_eq!(x.compare_to_real(&q(7, 10)).unwrap(), Ordering::Less);
                assert_eq!(
                    contact
                        .point()
                        .affine_form(0.into(), 1.into(), 0.into())
                        .compare_to_real(&Real::zero())
                        .unwrap(),
                    Ordering::Less
                );
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if swapped ^ a_reversed ^ b_reversed {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    })
                );
                assert!(contact.is_certified_transverse());
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
fn polynomial_pair_components_keep_branch_selection_and_trims() {
    let f = curves(polynomial(&[&[0, 0, 1], &[-1]]));
    // G=(y^2-x)(y-3). The extra line is outside the admitted rectangle;
    // both independently selected parabola branches retain G as authority.
    let g = curves(polynomial(&[&[0, 0, -3, 1], &[3, -1]]));
    assert_eq!((f.curves().len(), g.curves().len()), (2, 2));
    let beta = CurveParameter2::from(BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![(-1).into(), 0.into(), 2.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), 1.into()).unwrap(),
        )
        .unwrap(),
    ));
    let a = f.curves()[0]
        .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), beta.clone()).unwrap())
        .unwrap();
    let b = g.curves()[0]
        .restrict_source(CurveParameterRange2::try_new(q(1, 2).into(), Real::one().into()).unwrap())
        .unwrap();
    for a_reversed in [false, true] {
        for b_reversed in [false, true] {
            let a = if a_reversed {
                a.reversed().unwrap()
            } else {
                a.clone()
            };
            let b = if b_reversed {
                b.reversed().unwrap()
            } else {
                b.clone()
            };
            for (first, second) in [(&a, &b), (&b, &a)] {
                let topology = first.intersection_topology(second).unwrap();
                assert!(topology.result().is_complete() && topology.result().contacts().is_empty());
                assert_eq!(topology.result().overlaps().len(), 1);
                assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                let overlap = &topology.result().overlaps()[0];
                assert_eq!(
                    overlap.orientation(),
                    if a_reversed ^ b_reversed {
                        CurveOverlapOrientation2::Reversed
                    } else {
                        CurveOverlapOrientation2::Same
                    }
                );
                assert!(overlap.includes_start() && overlap.includes_end());
                for (s, t) in [
                    (
                        overlap.first_range().start(),
                        overlap.second_range().start(),
                    ),
                    (overlap.first_range().end(), overlap.second_range().end()),
                ] {
                    assert_eq!(s.compare(t).unwrap(), Ordering::Equal);
                    assert!(
                        s.compare(&q(1, 2).into()).unwrap() == Ordering::Equal
                            || s.compare(&beta).unwrap() == Ordering::Equal
                    );
                    assert!(
                        first
                            .point_at(s)
                            .unwrap()
                            .coincides_with(&second.point_at(t).unwrap())
                            .unwrap()
                    );
                }
                for piece in topology.first() {
                    assert!(piece.intersect_curve(second).unwrap().is_complete());
                }
            }
        }
    }
    assert!(a.intersect_curve(&g.curves()[1]).unwrap().is_disjoint());
    // The full opposite branches meet only at their smooth vertical event.
    // H_x=-1 proves a common tangent line there even though H_y=0.
    rational_contact(
        &f.curves()[0],
        &g.curves()[1],
        0.into(),
        0.into(),
        RealSign::Zero,
    );
}

#[test]
fn shared_polynomial_component_keeps_residual_and_mixed_contacts() {
    let f = curves(polynomial(&[&[0, 0, 1], &[0], &[-1]]));
    let g = curves(polynomial(&[&[0, -1, 2], &[1, -2]]));
    assert_eq!((f.curves().len(), g.curves().len()), (4, 4));
    // F=(y-x)(y+x), G=(y-x)(2y-1). The residual pair intersects at
    // (-1/2,1/2), away from the shared support y=x.
    rational_contact(
        &f.curves()[1],
        &g.curves()[1],
        q(-1, 2),
        q(1, 2),
        RealSign::Positive,
    );
    let overlap = f.curves()[0].intersect_curve(&g.curves()[0]).unwrap();
    assert!(overlap.is_complete() && overlap.contacts().is_empty());
    assert_eq!(overlap.overlaps().len(), 1);
    parameter_is(overlap.overlaps()[0].first_range().start(), (-1).into());
    parameter_is(overlap.overlaps()[0].first_range().end(), 0.into());
    // This mixed component/other-branch contact lies on y=x and therefore
    // is absent from the residual pair. Saturation must not discard it.
    rational_contact(
        &f.curves()[3],
        &g.curves()[1],
        q(1, 2),
        q(1, 2),
        RealSign::Negative,
    );
}

#[test]
fn polynomial_pair_sections_keep_tangencies_and_distinct_supports() {
    let f = polynomial(&[&[0, 1], &[0], &[-1]]);
    let parabola = curves(f.clone());
    let quarter = curves(polynomial(&[&[-1, 4]]));
    assert_eq!((parabola.curves().len(), quarter.curves().len()), (1, 1));
    for (a, b, swapped) in [
        (&parabola.curves()[0], &quarter.curves()[0], false),
        (&quarter.curves()[0], &parabola.curves()[0], true),
    ] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
        assert_eq!(topology.result().contacts().len(), 2);
        assert_eq!((topology.first().len(), topology.second().len()), (3, 3));
        for (index, contact) in topology.result().contacts().iter().enumerate() {
            let x = if index == 0 { q(-1, 2) } else { q(1, 2) };
            assert!(
                contact
                    .point()
                    .coincides_with(&Point2::new(x.clone(), q(1, 4)).into())
                    .unwrap()
            );
            on(contact.point(), f.clone());
            parameter_is(&contact.first().parameter().unwrap(), x.clone());
            parameter_is(&contact.second().parameter().unwrap(), x);
            assert_eq!(
                contact.tangent_cross_sign(),
                Some(if (index == 0) ^ swapped {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                })
            );
            assert!(contact.is_certified_transverse());
        }
        for i in 0..3 {
            let next = topology.first()[i]
                .intersection_topology(&topology.second()[i])
                .unwrap();
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert_eq!(next.result().contacts().len(), if i == 1 { 2 } else { 1 });
        }
    }
    let tangent = curves(polynomial(&[&[0, 1]]));
    rational_contact(
        &parabola.curves()[0],
        &tangent.curves()[0],
        0.into(),
        0.into(),
        RealSign::Zero,
    );
    let delta = q(1, 1_i64 << 50) * q(1, 1_i64 << 50);
    let displaced = curves(BivariatePolynomial::new(vec![
        vec![-delta, 1.into()],
        vec![0.into()],
        vec![(-1).into()],
    ]));
    for (a, b) in [
        (&parabola.curves()[0], &displaced.curves()[0]),
        (&displaced.curves()[0], &parabola.curves()[0]),
    ] {
        assert!(a.intersect_curve(b).unwrap().is_disjoint());
        assert!(a.intersection_topology(b).unwrap().result().is_disjoint());
    }
}

#[test]
fn polynomial_pair_overlaps_keep_nonunit_parameter_maps() {
    let f = curves(polynomial(&[&[0, 1], &[-1]]));
    let g = curves(polynomial(&[&[0, -3, 1], &[3, -1]]));
    assert_eq!((f.curves().len(), g.curves().len()), (1, 1));
    for reverse in [false, true] {
        let a = if reverse {
            f.curves()[0].reversed().unwrap()
        } else {
            f.curves()[0].clone()
        };
        for (a, b) in [(&a, &g.curves()[0]), (&g.curves()[0], &a)] {
            let result = a.intersect_curve(b).unwrap();
            assert!(result.is_complete() && result.contacts().is_empty());
            assert_eq!(result.overlaps().len(), 1);
            let overlap = &result.overlaps()[0];
            let clipped = overlap
                .restrict(
                    [q(-3, 4).into(), q(-1, 4).into()],
                    [q(-1, 2).into(), Real::zero().into()],
                )
                .unwrap()
                .expect("the original negative x charts overlap");
            assert_eq!(clipped.orientation(), overlap.orientation());
            assert!(clipped.includes_start() && clipped.includes_end());
            for (s, t) in [
                (
                    clipped.first_range().start(),
                    clipped.second_range().start(),
                ),
                (clipped.first_range().end(), clipped.second_range().end()),
            ] {
                assert_eq!(s.compare(t).unwrap(), Ordering::Equal);
                assert!(
                    s.compare(&q(-1, 2).into()).unwrap() == Ordering::Equal
                        || s.compare(&q(-1, 4).into()).unwrap() == Ordering::Equal
                );
                assert!(
                    a.point_at(s)
                        .unwrap()
                        .coincides_with(&b.point_at(t).unwrap())
                        .unwrap()
                );
            }
            let widened = clipped
                .restrict(
                    [Real::from(-1).into(), Real::one().into()],
                    [Real::from(-1).into(), Real::one().into()],
                )
                .unwrap()
                .unwrap();
            assert_eq!(widened.first_range(), clipped.first_range());
            assert_eq!(widened.second_range(), clipped.second_range());
            assert!(
                clipped
                    .restrict(
                        [q(-1, 2).into(), q(-3, 8).into()],
                        [q(-3, 8).into(), q(-1, 4).into()]
                    )
                    .unwrap()
                    .is_none()
            );
            // Overlaps retain traversal; restrict_source explicitly accepts
            // ascending source domains and keeps the operand's traversal.
            let ascending = |range: &CurveParameterRange2| {
                let mut ends = [range.start().clone(), range.end().clone()];
                if ends[0].compare(&ends[1]).unwrap() == Ordering::Greater {
                    ends.swap(0, 1);
                }
                CurveParameterRange2::try_new(ends[0].clone(), ends[1].clone()).unwrap()
            };
            let next = a
                .restrict_source(ascending(clipped.first_range()))
                .unwrap()
                .intersection_topology(
                    &b.restrict_source(ascending(clipped.second_range()))
                        .unwrap(),
                )
                .unwrap();
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert_eq!(next.result().overlaps().len(), 1);
        }
    }
}

#[test]
fn singular_polynomial_pairs_do_not_invent_tangency() {
    let f = curves(polynomial(&[&[0, 0, 1], &[0], &[-1]]));
    let independent = curves(polynomial(&[&[0, 0, 1], &[0], &[-1]]));
    let axis = curves(polynomial(&[&[0, 1]]));
    for other in [&independent.curves()[1], &axis.curves()[0]] {
        for (a, b) in [(&f.curves()[0], other), (other, &f.curves()[0])] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
            assert_eq!(topology.result().contacts().len(), 1);
            let contact = &topology.result().contacts()[0];
            assert!(
                contact
                    .point()
                    .coincides_with(&Point2::new(0.into(), 0.into()).into())
                    .unwrap()
            );
            assert_eq!(contact.tangent_cross_sign(), None);
            assert!(!contact.is_certified_transverse());
        }
    }
}

#[test]
fn polynomial_pairs_retry_and_retain_certainty() {
    let (a, b, point) = {
        let first = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
        let second = curves(polynomial(&[&[0, 0, 1], &[-1]]));
        let a = &first.curves()[0];
        let b = &second.curves()[0];
        let denied = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget::ZERO,
            || {
                let error = a.intersect_curve(b).unwrap_err();
                assert!(
                    matches!(error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
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
            || a.intersection_topology(b),
        );
        let topology = retry.result.unwrap().unwrap();
        assert!(topology.result().is_complete());
        (
            topology.first()[0].clone(),
            topology.second()[1].clone(),
            topology.result().contacts()[0].point().clone(),
        )
    };
    let replay = point.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
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
            on(
                next.result().contacts()[0].point(),
                polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]),
            );
            on(
                next.result().contacts()[0].point(),
                polynomial(&[&[0, 0, 1], &[-1]]),
            );
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}

#[test]
fn polynomial_pairs_keep_contacts_and_overlaps_in_shared_similarity_charts() {
    let f = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    let g = curves(polynomial(&[&[0, 0, 1], &[-1]]));
    let line = curves(polynomial(&[&[0, 1], &[-1]]));
    let line_component = curves(polynomial(&[&[0, -3, 1], &[3, -1]]));
    let cases = [
        (
            hypercurve::Similarity2::try_from_real_affine(
                0.into(),
                (-1).into(),
                1.into(),
                0.into(),
                3.into(),
                4.into(),
            )
            .unwrap(),
            polynomial(&[&[267, -1], &[-410], &[270], &[-90], &[15], &[-1]]),
            polynomial(&[&[13, -1], &[-6], &[1]]),
            false,
        ),
        (
            hypercurve::Similarity2::try_from_real_affine(
                (-1).into(),
                0.into(),
                0.into(),
                1.into(),
                2.into(),
                (-3).into(),
            )
            .unwrap(),
            polynomial(&[&[261, 410, 270, 90, 15, 1], &[1]]),
            polynomial(&[&[7, 6, 1], &[1]]),
            true,
        ),
    ];
    for (transform, world_f, world_g, reflected) in cases {
        for reverse in [false, true] {
            let a = f.curves()[0].transform_similarity(&transform).unwrap();
            let a = if reverse { a.reversed().unwrap() } else { a };
            let (m00, m01, m10, m11, tx, ty) = transform.affine_components();
            let independently_constructed = hypercurve::Similarity2::try_from_real_affine(
                m00.clone(),
                m01.clone(),
                m10.clone(),
                m11.clone(),
                tx.clone(),
                ty.clone(),
            )
            .unwrap();
            let b = g.curves()[0]
                .transform_similarity(&independently_constructed)
                .unwrap();
            for (a, b, swapped) in [(&a, &b, false), (&b, &a, true)] {
                let topology = a.intersection_topology(b).unwrap();
                assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
                assert_eq!(topology.result().contacts().len(), 1);
                assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
                let contact = &topology.result().contacts()[0];
                on(contact.point(), world_f.clone());
                on(contact.point(), world_g.clone());
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if reflected ^ reverse ^ swapped {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    })
                );
                let s = contact.first().parameter().unwrap();
                let t = contact.second().parameter().unwrap();
                assert_eq!(s.compare(&t).unwrap(), Ordering::Equal);
                assert_eq!(s.compare(&q(3, 5).into()).unwrap(), Ordering::Greater);
                assert_eq!(s.compare(&q(7, 10).into()).unwrap(), Ordering::Less);
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
                assert_eq!((next.first().len(), next.second().len()), (1, 1));
                assert_eq!(next.result().contacts().len(), 1);
            }
        }
        let a = line.curves()[0].transform_similarity(&transform).unwrap();
        let b = line_component.curves()[0]
            .transform_similarity(&transform)
            .unwrap()
            .reversed()
            .unwrap();
        let topology = a.intersection_topology(&b).unwrap();
        assert_eq!(topology.result().overlaps().len(), 1);
        assert!(topology.result().contacts().is_empty());
        let overlap = &topology.result().overlaps()[0];
        assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Reversed);
        let clipped = overlap
            .restrict(
                [q(-3, 4).into(), q(-1, 4).into()],
                [q(-1, 2).into(), Real::zero().into()],
            )
            .unwrap()
            .unwrap();
        parameter_is(clipped.first_range().start(), q(-1, 2));
        parameter_is(clipped.first_range().end(), q(-1, 4));
        for (s, t) in [
            (
                clipped.first_range().start(),
                clipped.second_range().start(),
            ),
            (clipped.first_range().end(), clipped.second_range().end()),
        ] {
            assert_eq!(s.compare(t).unwrap(), Ordering::Equal);
            assert!(
                a.point_at(s)
                    .unwrap()
                    .coincides_with(&b.point_at(t).unwrap())
                    .unwrap()
            );
        }
    }
    let identity = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
    )
    .unwrap();
    let image = g.curves()[0].transform_similarity(&identity).unwrap();
    for (a, b, swapped) in [
        (&f.curves()[0], &image, false),
        (&image, &f.curves()[0], true),
    ] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete());
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        assert_eq!(topology.result().contacts().len(), 1);
        let contact = &topology.result().contacts()[0];
        assert_eq!(
            contact.tangent_cross_sign(),
            Some(if swapped {
                RealSign::Positive
            } else {
                RealSign::Negative
            })
        );
        on(contact.point(), polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
        on(contact.point(), polynomial(&[&[0, 0, 1], &[-1]]));
    }
    let translation = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
    )
    .unwrap();
    let image = g.curves()[0].transform_similarity(&translation).unwrap();
    for (a, b) in [(&f.curves()[0], &image), (&image, &f.curves()[0])] {
        let error = a.intersect_curve(b).unwrap_err();
        assert!(
            matches!(error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason() == hypercurve::UncertaintyReason::Unsupported)
        );
    }
}

#[test]
fn polynomial_pairs_continue_past_the_hot_resultant_degree() {
    // y=x^37 is a regular graph. Its section by y=1/2 has exactly one
    // selected real point, with 2*x^37-1=0 and 1/2<x<1. The support and
    // parameter degree exceed the ordinary resultant schedule's bound.
    let mut rows = vec![vec![Real::zero()]; 38];
    rows[0].push(Real::one());
    rows[37][0] = Real::from(-1);
    let equation = BivariatePolynomial::new(rows);
    let section = polynomial(&[&[-1, 2]]);
    let bounded = hypersolve::resultant_bivariate_polynomial_system(
        &equation,
        &section,
        hypersolve::CurveResultantParameter::First,
        hypersolve::CurveIntersectionResultantConfig::default(),
    );
    assert_eq!(
        bounded.status,
        hypersolve::CurveIntersectionResultantStatus::DegreeBoundExceeded
    );
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig {
            max_axis_degree: 37,
            ..BivariateTopologyConfig::default()
        },
    )
    .unwrap();
    let first =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    let second = curves(section.clone());
    assert_eq!((first.curves().len(), second.curves().len()), (1, 1));
    for (a, b, swapped) in [
        (&first.curves()[0], &second.curves()[0], false),
        (&second.curves()[0], &first.curves()[0], true),
    ] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
        assert_eq!(topology.result().contacts().len(), 1);
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        let contact = &topology.result().contacts()[0];
        on(contact.point(), equation.clone());
        on(contact.point(), section.clone());
        let x = contact.point().affine_form(1.into(), 0.into(), 0.into());
        assert_eq!(x.compare_to_real(&q(1, 2)).unwrap(), Ordering::Greater);
        assert_eq!(x.compare_to_real(&Real::one()).unwrap(), Ordering::Less);
        assert_eq!(
            contact.tangent_cross_sign(),
            Some(if swapped {
                RealSign::Positive
            } else {
                RealSign::Negative
            })
        );
        assert!(contact.is_certified_transverse());
        for (curve, location) in [(a, contact.first()), (b, contact.second())] {
            assert!(
                curve
                    .point_at(&location.parameter().unwrap())
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
