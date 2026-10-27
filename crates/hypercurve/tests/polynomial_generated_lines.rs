//! Polynomial incidence must consume generated lines with retained endpoints.

use hypercurve::{Curve2, CurvePoint2, LineSeg2, Point2, PolynomialCurveSet2, Real};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn curves(equation: BivariatePolynomial) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
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

fn quintic_equation() -> BivariatePolynomial {
    polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]])
}

fn on(point: &CurvePoint2, equation: BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(equation, polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

fn vertical_contact(source: &Curve2, line: &Curve2) {
    for (a, b) in [(source, line), (line, source)] {
        let result = a.intersect_curve(b).unwrap();
        assert!(result.is_complete() && result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 1);
        on(result.contacts()[0].point(), quintic_equation());
        on(result.contacts()[0].point(), polynomial(&[&[], &[1]]));
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete());
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        assert_eq!(topology.result().contacts().len(), 1);
        let contact = &topology.result().contacts()[0];
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
            .intersection_topology(&topology.second()[0])
            .unwrap();
        assert_eq!((next.first().len(), next.second().len()), (1, 1));
        assert_eq!(next.result().contacts().len(), 1);
    }
}

#[test]
fn generated_rational_line_reuses_polynomial_sections() {
    let set = curves(quintic_equation());
    let start = Point2::from_values(0, -2);
    let end = Point2::from_values(0, 2);
    let native = Curve2::from(LineSeg2::try_new(start.clone(), end.clone()).unwrap());
    vertical_contact(&set.curves()[0], &native);
    let generated = Curve2::try_line(start.into(), end.into()).unwrap();
    vertical_contact(&set.curves()[0], &generated);
}

#[test]
fn generated_selected_endpoint_line_reuses_polynomial_sections() {
    let set = curves(quintic_equation());
    let root = Real::from(2).sqrt().unwrap();
    let native = Curve2::from(
        LineSeg2::try_new(
            Point2::new(Real::zero(), -root.clone()),
            Point2::new(Real::zero(), root),
        )
        .unwrap(),
    );
    vertical_contact(&set.curves()[0], &native);
    let endpoints = curves(polynomial(&[&[-2, 0, 1]]));
    assert_eq!(endpoints.curves().len(), 2);
    let start = endpoints.curves()[0]
        .point_at(&Real::zero().into())
        .unwrap();
    let end = endpoints.curves()[1]
        .point_at(&Real::zero().into())
        .unwrap();
    assert!(start.coordinates().is_none() && end.coordinates().is_none());
    let generated = Curve2::try_line(start, end).unwrap();
    drop(endpoints);
    vertical_contact(&set.curves()[0], &generated);
}

#[test]
fn generated_selected_line_retains_common_component_overlap() {
    let set = curves(polynomial(&[&[-2, 0, 1]]));
    let source = &set.curves()[0];
    let y = -Real::from(2).sqrt().unwrap();
    let native = Curve2::from(
        LineSeg2::try_new(
            Point2::new((-1).into(), y.clone()),
            Point2::new(1.into(), y),
        )
        .unwrap(),
    );
    let generated = Curve2::try_line(source.start(), source.end()).unwrap();
    for line in [native, generated] {
        for (a, b) in [(source, &line), (&line, source)] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().is_complete());
            assert!(topology.result().contacts().is_empty());
            assert_eq!(topology.result().overlaps().len(), 1);
            assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
            let overlap = &topology.result().overlaps()[0];
            for (s, t) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                assert!(
                    a.point_at(s)
                        .unwrap()
                        .coincides_with(&b.point_at(t).unwrap())
                        .unwrap()
                );
            }
        }
    }
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn ordered(mut bounds: [hypercurve::CurveParameter2; 2]) -> [hypercurve::CurveParameter2; 2] {
    if bounds[0].compare(&bounds[1]).unwrap() == Ordering::Greater {
        bounds.swap(0, 1);
    }
    bounds
}

fn located(line: &Curve2, point: &CurvePoint2) -> hypercurve::CurveParameter2 {
    let hypercurve::CurvePointLocations2::Locations(locations) =
        line.point_locations(point).unwrap()
    else {
        panic!("a nonzero chord has one parameter per point");
    };
    assert_eq!(locations.len(), 1);
    locations[0].parameter().unwrap()
}

#[test]
fn generated_constant_line_point_locations_use_selected_coordinate_evidence() {
    let set = curves(polynomial(&[&[-2, 0, 1]]));
    let branch = &set.curves()[0];
    let start = branch.start();
    let end = branch.end();
    let query = branch.point_at(&q(-1, 2).into()).unwrap();
    for endpoint in [&start, &end] {
        assert_eq!(
            endpoint
                .compare_coordinate(&query, hypercurve::Axis2::Y)
                .unwrap(),
            Ordering::Equal
        );
    }
    assert_eq!(
        start
            .compare_coordinate(&query, hypercurve::Axis2::X)
            .unwrap(),
        Ordering::Less
    );
    assert_eq!(
        end.compare_coordinate(&query, hypercurve::Axis2::X)
            .unwrap(),
        Ordering::Greater
    );
    let line = Curve2::try_line(start, end).unwrap();
    let parameter = located(&line, &query);
    assert!(
        line.point_at(&parameter)
            .unwrap()
            .coincides_with(&query)
            .unwrap()
    );
}

#[test]
fn generated_polynomial_contacts_keep_orientation_and_replay() {
    for reverse_branch in [false, true] {
        for reverse_line in [false, true] {
            // Cold selected endpoints: no native-line control has populated
            // alternate coordinate evidence for these objects.
            let set = curves(quintic_equation());
            let endpoints = curves(polynomial(&[&[-2, 0, 1]]));
            let line = Curve2::try_line(
                endpoints.curves()[0]
                    .point_at(&Real::zero().into())
                    .unwrap(),
                endpoints.curves()[1]
                    .point_at(&Real::zero().into())
                    .unwrap(),
            )
            .unwrap();
            let branch = set.curves()[0].clone();
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
            drop(set);
            drop(endpoints);
            for (a, b, swapped) in [(&branch, &line, false), (&line, &branch, true)] {
                let topology = a.intersection_topology(b).unwrap();
                assert!(topology.result().is_complete());
                assert_eq!(topology.result().contacts().len(), 1);
                let contact = &topology.result().contacts()[0];
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if reverse_branch ^ reverse_line ^ swapped {
                        hyperreal::RealSign::Negative
                    } else {
                        hyperreal::RealSign::Positive
                    })
                );
                assert!(contact.is_certified_transverse());
                let replay = contact.point().definition().unwrap().into_point().unwrap();
                on(&replay, quintic_equation());
                on(&replay, polynomial(&[&[], &[1]]));
                assert!(replay.coincides_with(contact.point()).unwrap());
                let parameter = located(&line, &replay);
                assert!(
                    line.point_at(&parameter)
                        .unwrap()
                        .coincides_with(&replay)
                        .unwrap()
                );
                for part in topology.first().iter().chain(topology.second()) {
                    assert!(!part.start().coincides_with(&part.end()).unwrap());
                }
            }
        }
    }
}

#[test]
fn generated_polynomial_overlaps_keep_restrictions_and_reversal() {
    for reverse_branch in [false, true] {
        for reverse_line in [false, true] {
            let set = curves(polynomial(&[&[-2, 0, 1]]));
            let branch = set.curves()[0].clone();
            let line = Curve2::try_line(branch.start(), branch.end()).unwrap();
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
            let half = branch.point_at(&q(-1, 2).into()).unwrap();
            let zero = branch.point_at(&Real::zero().into()).unwrap();
            let chord_bounds = ordered([located(&line, &half), located(&line, &zero)]);
            let source_bounds = [q(-3, 4).into(), q(-1, 4).into()];
            for (a, b, swapped) in [(&branch, &line, false), (&line, &branch, true)] {
                let topology = a.intersection_topology(b).unwrap();
                let overlaps = topology.result().overlaps();
                assert_eq!(overlaps.len(), 1);
                let overlap = &overlaps[0];
                assert_eq!(
                    overlap.orientation(),
                    if reverse_branch ^ reverse_line {
                        hypercurve::CurveOverlapOrientation2::Reversed
                    } else {
                        hypercurve::CurveOverlapOrientation2::Same
                    }
                );
                let (first, second) = if swapped {
                    (chord_bounds.clone(), source_bounds.clone())
                } else {
                    (source_bounds.clone(), chord_bounds.clone())
                };
                let clipped = overlap.restrict(first, second).unwrap().unwrap();
                let first = ordered([
                    clipped.first_range().start().clone(),
                    clipped.first_range().end().clone(),
                ]);
                let second = ordered([
                    clipped.second_range().start().clone(),
                    clipped.second_range().end().clone(),
                ]);
                let mut xs = Vec::new();
                for (s, t) in [
                    (
                        clipped.first_range().start(),
                        clipped.second_range().start(),
                    ),
                    (clipped.first_range().end(), clipped.second_range().end()),
                ] {
                    let point = a.point_at(s).unwrap();
                    assert!(point.coincides_with(&b.point_at(t).unwrap()).unwrap());
                    on(&point, polynomial(&[&[-2, 0, 1]]));
                    assert_eq!(
                        point
                            .affine_form(0.into(), 1.into(), 0.into())
                            .compare_to_real(&Real::zero())
                            .unwrap(),
                        Ordering::Less
                    );
                    xs.push(point);
                }
                assert!(xs.iter().any(|p| {
                    p.affine_form(1.into(), 0.into(), 0.into())
                        .compare_to_real(&q(-1, 2))
                        .unwrap()
                        == Ordering::Equal
                }));
                assert!(xs.iter().any(|p| {
                    p.affine_form(1.into(), 0.into(), 0.into())
                        .compare_to_real(&q(-1, 4))
                        .unwrap()
                        == Ordering::Equal
                }));
                let again = clipped
                    .restrict(
                        [
                            overlap.first_range().start().clone(),
                            overlap.first_range().end().clone(),
                        ],
                        [
                            overlap.second_range().start().clone(),
                            overlap.second_range().end().clone(),
                        ],
                    )
                    .unwrap()
                    .unwrap();
                for (p, q) in [
                    (again.first_range().start(), clipped.first_range().start()),
                    (again.first_range().end(), clipped.first_range().end()),
                ] {
                    assert_eq!(p.compare(q).unwrap(), Ordering::Equal);
                }
                assert!(
                    clipped
                        .restrict([first[0].clone(), first[0].clone()], second.clone())
                        .unwrap()
                        .is_none()
                );
                let restricted_a = a
                    .restrict_source(
                        hypercurve::CurveParameterRange2::try_new(
                            first[0].clone(),
                            first[1].clone(),
                        )
                        .unwrap(),
                    )
                    .unwrap();
                let restricted_b = b
                    .restrict_source(
                        hypercurve::CurveParameterRange2::try_new(
                            second[0].clone(),
                            second[1].clone(),
                        )
                        .unwrap(),
                    )
                    .unwrap();
                let next = restricted_a.intersection_topology(&restricted_b).unwrap();
                assert_eq!(next.result().overlaps().len(), 1);
                assert_eq!((next.first().len(), next.second().len()), (1, 1));
            }
        }
    }
}

#[test]
fn generated_constant_components_remain_distinct_and_clip_endpoints() {
    let set = curves(polynomial(&[&[-2, 0, 1]]));
    let negative = &set.curves()[0];
    let positive = &set.curves()[1];
    let other_component = Curve2::try_line(positive.start(), positive.end()).unwrap();
    for (a, b) in [(negative, &other_component), (&other_component, negative)] {
        assert!(a.intersect_curve(b).unwrap().is_disjoint());
    }
    let restrict = |a: Real, b: Real| {
        negative
            .restrict_source(hypercurve::CurveParameterRange2::try_new(a.into(), b.into()).unwrap())
            .unwrap()
    };
    let left = restrict((-1).into(), q(-1, 2));
    let right =
        Curve2::try_line(negative.point_at(&q(1, 2).into()).unwrap(), negative.end()).unwrap();
    for (a, b) in [(&left, &right), (&right, &left)] {
        assert!(a.intersection_topology(b).unwrap().result().is_disjoint());
    }
    let left = restrict((-1).into(), Real::zero());
    let right = Curve2::try_line(
        negative.point_at(&Real::zero().into()).unwrap(),
        negative.end(),
    )
    .unwrap();
    for (a, b) in [(&left, &right), (&right, &left)] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete());
        assert!(topology.result().overlaps().is_empty());
        assert_eq!(topology.result().contacts().len(), 1);
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        assert_eq!(
            topology.result().contacts()[0].tangent_cross_sign(),
            Some(hyperreal::RealSign::Zero)
        );
    }
    let tiny = q(1, 1 << 20);
    let tiny = &tiny * &tiny;
    let tiny = &tiny * &tiny;
    let shift = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        tiny,
    )
    .unwrap();
    let nearby = Curve2::try_line(negative.start(), negative.end())
        .unwrap()
        .transform_similarity(&shift)
        .unwrap();
    for (a, b) in [(negative, &nearby), (&nearby, negative)] {
        assert!(a.intersect_curve(b).unwrap().is_disjoint());
    }
}

#[test]
fn generated_polynomial_sections_keep_similarity_charts() {
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
            polynomial(&[&[-4, 1]]),
            polynomial(&[&[7], &[-6], &[1]]),
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
            polynomial(&[&[-2], &[1]]),
            polynomial(&[&[7, 6, 1]]),
            true,
        ),
        (
            hypercurve::Similarity2::try_from_real_affine(
                3.into(),
                (-4).into(),
                4.into(),
                3.into(),
                0.into(),
                0.into(),
            )
            .unwrap(),
            // 25^5 F((3x+4y)/25, (-4x+3y)/25).
            polynomial(&[
                &[48_828_125, 4_296_875, 0, 0, 0, 243],
                &[-8_984_375, 0, 0, 0, -1620],
                &[0, 0, 0, 4320],
                &[0, 0, -5760],
                &[0, 3840],
                &[-1024],
            ]),
            polynomial(&[&[0, 4], &[3]]),
            polynomial(&[&[-1250, 0, 9], &[0, -24], &[16]]),
            false,
        ),
    ];
    for (transform, world_f, world_line, world_constant, reflected) in cases {
        let f = curves(quintic_equation());
        let endpoints = curves(polynomial(&[&[-2, 0, 1]]));
        let line = Curve2::try_line(
            endpoints.curves()[0]
                .point_at(&Real::zero().into())
                .unwrap(),
            endpoints.curves()[1]
                .point_at(&Real::zero().into())
                .unwrap(),
        )
        .unwrap()
        .transform_similarity(&transform)
        .unwrap();
        let branch = f.curves()[0].transform_similarity(&transform).unwrap();
        drop(f);
        for (a, b, swapped) in [(&branch, &line, false), (&line, &branch, true)] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
            assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
            assert_eq!(topology.result().contacts().len(), 1);
            let contact = &topology.result().contacts()[0];
            on(contact.point(), world_f.clone());
            on(contact.point(), world_line.clone());
            assert_eq!(
                contact.tangent_cross_sign(),
                Some(if reflected ^ swapped {
                    hyperreal::RealSign::Negative
                } else {
                    hyperreal::RealSign::Positive
                })
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
            let next = topology.first()[0]
                .intersection_topology(&topology.second()[0])
                .unwrap();
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert_eq!(next.result().contacts().len(), 1);
        }
        let constant = endpoints.curves()[0]
            .transform_similarity(&transform)
            .unwrap();
        let line = Curve2::try_line(endpoints.curves()[0].start(), endpoints.curves()[0].end())
            .unwrap()
            .transform_similarity(&transform)
            .unwrap()
            .reversed()
            .unwrap();
        drop(endpoints);
        for (a, b) in [(&constant, &line), (&line, &constant)] {
            let topology = a.intersection_topology(b).unwrap();
            assert_eq!(topology.result().overlaps().len(), 1);
            assert!(topology.result().contacts().is_empty());
            assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
            let overlap = &topology.result().overlaps()[0];
            assert_eq!(
                overlap.orientation(),
                hypercurve::CurveOverlapOrientation2::Reversed
            );
            for (s, t) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                on(&a.point_at(s).unwrap(), world_constant.clone());
                assert!(
                    a.point_at(s)
                        .unwrap()
                        .coincides_with(&b.point_at(t).unwrap())
                        .unwrap()
                );
            }
        }
    }
}

#[test]
fn generated_polynomial_sections_clip_finite_contacts_and_tangency() {
    let equation = polynomial(&[&[0, 1], &[], &[-1]]);
    let set = curves(equation.clone());
    assert_eq!(set.curves().len(), 1);
    let branch = &set.curves()[0];
    for (lo, hi, y, count, pieces, tangent) in [
        ((-1).into(), 1.into(), Real::zero(), 1, 2, true),
        ((-1).into(), 1.into(), q(1, 4), 2, 3, false),
        (Real::zero(), 1.into(), q(1, 4), 1, 2, false),
        (q(3, 4), 1.into(), q(1, 4), 0, 1, false),
        ((-1).into(), 1.into(), -q(1, 1 << 20), 0, 1, false),
    ] {
        let line = Curve2::try_line(
            Point2::new(lo, y.clone()).into(),
            Point2::new(hi, y.clone()).into(),
        )
        .unwrap();
        for (a, b) in [(branch, &line), (&line, branch)] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().is_complete() && topology.result().overlaps().is_empty());
            assert_eq!(topology.result().contacts().len(), count);
            assert_eq!(
                (topology.first().len(), topology.second().len()),
                (pieces, pieces)
            );
            for contact in topology.result().contacts() {
                on(contact.point(), equation.clone());
                assert_eq!(
                    contact
                        .point()
                        .affine_form(0.into(), 1.into(), 0.into())
                        .compare_to_real(&y)
                        .unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    contact.tangent_cross_sign() == Some(hyperreal::RealSign::Zero),
                    tangent
                );
                assert_eq!(contact.is_certified_transverse(), !tangent);
            }
        }
    }
}

#[test]
fn generated_polynomial_sections_retry_and_keep_certainty() {
    let (a, b, point) = {
        let set = curves(quintic_equation());
        let endpoints = curves(polynomial(&[&[-2, 0, 1]]));
        let line = Curve2::try_line(
            endpoints.curves()[0]
                .point_at(&Real::zero().into())
                .unwrap(),
            endpoints.curves()[1]
                .point_at(&Real::zero().into())
                .unwrap(),
        )
        .unwrap();
        let branch = &set.curves()[0];
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
                assert!(matches!(branch.intersect_curve(&line).unwrap_err(),
                hypercurve::ExactCurveError::Blocked(blocker)
                    if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted));
            });
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 100_000,
                promotions: 100_000,
            },
            || branch.intersection_topology(&line),
        );
        let topology = retry.result.unwrap().unwrap();
        assert_eq!(topology.result().contacts().len(), 1);
        (
            topology.first()[0].clone(),
            topology.second()[0].clone(),
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
            let next = a.intersection_topology(&b).unwrap();
            assert!(next.result().is_complete());
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert_eq!(next.result().contacts().len(), 1);
            let contact = &next.result().contacts()[0];
            assert!(contact.point().coincides_with(&replay).unwrap());
            on(contact.point(), quintic_equation());
            on(contact.point(), polynomial(&[&[], &[1]]));
            assert!(
                b.point_at(&located(&b, &replay))
                    .unwrap()
                    .coincides_with(&replay)
                    .unwrap()
            );
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
    assert!(matches!(
        Curve2::try_line(replay.clone(), replay),
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
}
