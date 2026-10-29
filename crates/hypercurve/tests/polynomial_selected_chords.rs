//! Selected-endpoint lines remain exact operands of polynomial incidence.

use hypercurve::{
    Curve2, CurvePoint2, CurvePointDefinition2, CurvePointRationalForm2, PolynomialCurveSet2, Real,
    Similarity2,
};
use hyperreal::RealSign;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn set(equation: BivariatePolynomial, x: [Real; 2], y: [Real; 2]) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: x,
            second: y,
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}
fn form(point: &CurvePoint2, numerator: BivariatePolynomial) -> CurvePointRationalForm2 {
    point.rational_form(numerator, p(&[&[1]])).unwrap()
}
fn shifted(
    point: CurvePoint2,
    numerator: BivariatePolynomial,
    denominator: BivariatePolynomial,
) -> CurvePoint2 {
    CurvePointDefinition2::RationalQueryPair {
        sources: [point.clone(), point],
        numerators: [p(&[&[], &[1]]), numerator],
        denominators: [p(&[&[1]]), denominator],
    }
    .into_point()
    .unwrap()
}
fn parabola() -> Curve2 {
    let set = set(
        p(&[&[0, 0, 1], &[-1]]),
        [0.into(), 1.into()],
        [0.into(), 1.into()],
    );
    assert_eq!(set.curves().len(), 1);
    set.curves()[0].clone()
}
fn selected_line(tangent: bool) -> (Curve2, CurvePoint2) {
    // alpha^3=1/128, 0<alpha<1/4. No coordinate materialization is needed.
    let set = set(
        p(&[&[-1, 0, 0, 128]]),
        [(-1).into(), 1.into()],
        [0.into(), 1.into()],
    );
    let branch = &set.curves()[0];
    let source = branch.point_at(&Real::zero().into()).unwrap();
    let endpoints = [branch.start(), branch.end()].map(|point| {
        assert!(point.coordinates().is_none());
        if tangent {
            // y=(x+alpha^2)/(2alpha), tangent to y^2=x at (alpha^2,alpha).
            shifted(point, p(&[&[0, 0, 1], &[1]]), p(&[&[0, 2]]))
        } else {
            shifted(point, p(&[&[0, 1], &[1]]), p(&[&[1]]))
        }
    });
    (
        Curve2::try_line(endpoints[0].clone(), endpoints[1].clone()).unwrap(),
        source,
    )
}
fn sign(negative: bool) -> RealSign {
    if negative {
        RealSign::Negative
    } else {
        RealSign::Positive
    }
}

#[test]
fn selected_oblique_chord_intersections_preserve_parameters_and_orientation() {
    let reflection = Similarity2::try_from_real_affine(
        0.into(),
        2.into(),
        2.into(),
        0.into(),
        3.into(),
        1.into(),
    )
    .unwrap();
    for reflected in [false, true] {
        for reverse_branch in [false, true] {
            for reverse_line in [false, true] {
                let (line, alpha) = selected_line(false);
                let branch = parabola();
                let (branch, line) = if reflected {
                    (
                        branch.transform_similarity(&reflection).unwrap(),
                        line.transform_similarity(&reflection).unwrap(),
                    )
                } else {
                    (branch, line)
                };
                let branch = if reverse_branch {
                    branch.reversed().unwrap()
                } else {
                    branch
                };
                let line = if reverse_line {
                    line.reversed().unwrap()
                } else {
                    line
                };
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (&line, &branch)
                    } else {
                        (&branch, &line)
                    };
                    let topology = a.intersection_topology(b).unwrap();
                    let result = topology.result();
                    assert!(result.is_complete() && result.overlaps().is_empty());
                    assert_eq!(result.contacts().len(), 2);
                    assert_eq!((topology.first().len(), topology.second().len()), (3, 3));
                    let mut halves = [0, 0];
                    for contact in result.contacts() {
                        let local = if reflected {
                            CurvePointDefinition2::Similarity {
                                source: contact.point().clone(),
                                transform: Similarity2::try_from_real_affine(
                                    0.into(),
                                    q(1, 2),
                                    q(1, 2),
                                    0.into(),
                                    q(-1, 2),
                                    q(-3, 2),
                                )
                                .unwrap(),
                            }
                            .into_point()
                            .unwrap()
                        } else {
                            contact.point().clone()
                        };
                        assert_eq!(
                            form(&local, p(&[&[0, 0, 1], &[-1]]))
                                .compare_to_real(&Real::zero())
                                .unwrap(),
                            Ordering::Equal
                        );
                        let y = form(&local, p(&[&[0, 1]]));
                        // Independent defining relation y^2-y+alpha=0.
                        assert_eq!(
                            y.multiply(&y)
                                .unwrap()
                                .subtract(&y)
                                .unwrap()
                                .add(&form(&alpha, p(&[&[0, 1]])))
                                .unwrap()
                                .compare_to_real(&Real::zero())
                                .unwrap(),
                            Ordering::Equal
                        );
                        let low = y.compare_to_real(&q(1, 2)).unwrap() == Ordering::Less;
                        halves[usize::from(!low)] += 1;
                        assert_eq!(
                            contact.tangent_cross_sign(),
                            Some(sign(
                                low ^ reflected ^ reverse_branch ^ reverse_line ^ swapped
                            ))
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
                    }
                    assert_eq!(halves, [1, 1]);
                }
            }
        }
    }
}

#[test]
fn selected_chord_tangency_keeps_the_even_root_and_joint_point() {
    let (line, alpha) = selected_line(true);
    let branch = parabola();
    for (a, b) in [(&branch, &line), (&line, &branch)] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete());
        assert_eq!(topology.result().contacts().len(), 1);
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        let contact = &topology.result().contacts()[0];
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Zero));
        assert!(!contact.is_certified_transverse());
        assert_eq!(
            form(contact.point(), p(&[&[0, 1]]))
                .compare(&form(&alpha, p(&[&[0, 1]])))
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            form(contact.point(), p(&[&[0, 0, 1], &[-1]]))
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        for (curve, location) in [(a, contact.first()), (b, contact.second())] {
            assert!(
                curve
                    .point_at(&location.parameter().unwrap())
                    .unwrap()
                    .coincides_with(contact.point())
                    .unwrap()
            );
        }
    }
}

#[test]
fn selected_oblique_component_overlap_does_not_merge_nearby_lines() {
    let set = set(
        p(&[&[0, 0, 1], &[], &[-2]]),
        [1.into(), 2.into()],
        [0.into(), 3.into()],
    );
    let branch = &set.curves()[0];
    let line = Curve2::try_line(branch.start(), branch.end()).unwrap();
    for reverse in [false, true] {
        let line = if reverse {
            line.reversed().unwrap()
        } else {
            line.clone()
        };
        for (a, b) in [(branch, &line), (&line, branch)] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().is_complete());
            assert!(topology.result().contacts().is_empty());
            assert_eq!(topology.result().overlaps().len(), 1);
            let overlap = &topology.result().overlaps()[0];
            for (u, v) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                assert!(
                    a.point_at(u)
                        .unwrap()
                        .coincides_with(&b.point_at(v).unwrap())
                        .unwrap()
                );
            }
        }
    }
    let epsilon = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let translated = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        epsilon,
    )
    .unwrap();
    let nearby = line.transform_similarity(&translated).unwrap();
    assert!(branch.intersect_curve(&nearby).unwrap().is_disjoint());
}

#[test]
fn selected_chord_common_component_keeps_other_branch_endpoint_incidence() {
    let set = set(
        p(&[&[0, 0, 1], &[], &[-2]]),
        [(-1).into(), 1.into()],
        [(-2).into(), 2.into()],
    );
    assert_eq!(set.curves().len(), 4);
    let line = Curve2::try_line(set.curves()[0].start(), set.curves()[3].end()).unwrap();
    let other = &set.curves()[1];
    for (a, b) in [(other, &line), (&line, other)] {
        let result = a.intersect_curve(b).unwrap();
        assert!(result.is_complete() && result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 1);
        let contact = &result.contacts()[0];
        assert!(
            contact
                .point()
                .coincides_with(&hypercurve::Point2::from_values(0, 0).into())
                .unwrap()
        );
        // The original reducible equation is singular here. This path does
        // not invent a tangency or a transverse sign without a branch proof.
        if let Some(sign) = contact.tangent_cross_sign() {
            assert_eq!(
                sign,
                if std::ptr::eq(a, other) {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                }
            );
        }
        for (curve, location) in [(a, contact.first()), (b, contact.second())] {
            assert!(
                curve
                    .point_at(&location.parameter().unwrap())
                    .unwrap()
                    .coincides_with(contact.point())
                    .unwrap()
            );
        }
    }
}

#[test]
fn selected_chord_incidence_budget_refusal_allows_a_larger_retry() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let (line, _) = selected_line(false);
    let branch = parabola();
    let denied = with_budget(WorkBudget::ZERO, || {
        assert!(
            matches!(branch.intersect_curve(&line), Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
        );
    });
    assert!(denied.result.is_err());
    let retry = with_budget(
        WorkBudget {
            refinements: 10000,
            promotions: 10000,
        },
        || {
            let result = branch.intersect_curve(&line).unwrap();
            assert!(result.is_complete());
            assert_eq!(result.contacts().len(), 2);
            for contact in result.contacts() {
                assert_eq!(
                    form(contact.point(), p(&[&[0, 0, 1], &[-1]]))
                        .compare_to_real(&Real::zero())
                        .unwrap(),
                    Ordering::Equal
                );
            }
        },
    );
    assert!(retry.result.is_ok());
}
