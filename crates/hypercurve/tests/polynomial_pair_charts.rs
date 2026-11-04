//! Pair incidence preserves independent polynomial source charts.
use hypercurve::{Point2, PolynomialCurveSet2, Real};
use hypersolve::{
    BivariatePolynomial,
    bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve},
};

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn curves(equation: &BivariatePolynomial) -> PolynomialCurveSet2 {
    PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(
            equation,
            &BivariateCurveDomain {
                first: [(-2).into(), 2.into()],
                second: [(-2).into(), 4.into()],
            },
            Default::default(),
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap()
}
#[test]
fn whole_axes_contact_consumes_the_exchanged_native_chart() {
    let set = curves(&polynomial(&[&[], &[0, 1]]));
    assert_eq!(set.curves().len(), 4);
    let first = &set.curves()[0];
    let second = &set.curves()[2];
    let result = first
        .intersect_curve(second)
        .expect("selected branches in different charts must intersect exactly");
    assert!(result.is_complete());
    assert!(result.overlaps().is_empty());
    assert_eq!(result.contacts().len(), 1);
    assert!(
        result.contacts()[0]
            .point()
            .coincides_with(&Point2::from_values(0, 0).into())
            .unwrap()
    );
    assert!(result.contacts()[0].is_certified_transverse());
    let topology = first.intersection_topology(second).unwrap();
    for piece in topology.first() {
        assert!(piece.intersect_curve(second).unwrap().is_complete());
    }
}

fn similarity(values: [i32; 6]) -> hypercurve::Similarity2 {
    let [a, b, c, d, x, y] = values.map(Real::from);
    hypercurve::Similarity2::try_from_real_affine(a, b, c, d, x, y).unwrap()
}
#[test]
fn exchanged_nonlinear_graphs_keep_both_contact_parameters_and_orientation() {
    use hyperreal::RealSign;
    let set = curves(&polynomial(&[&[0, 1], &[], &[-1]]));
    assert_eq!(set.curves().len(), 1);
    let a = &set.curves()[0];
    let b = a
        .transform_similarity(&similarity([0, 1, 1, 0, 0, 0]))
        .unwrap();
    for (first, second, swap) in [(a, &b, false), (&b, a, true)] {
        let result = first.intersect_curve(second).unwrap();
        assert!(result.is_complete());
        assert!(result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 2);
        for (contact, (coordinate, sign)) in result
            .contacts()
            .iter()
            .zip([(0, RealSign::Positive), (1, RealSign::Negative)])
        {
            assert!(
                contact
                    .point()
                    .coincides_with(&Point2::from_values(coordinate, coordinate).into())
                    .unwrap()
            );
            let sign = if swap {
                match sign {
                    RealSign::Positive => RealSign::Negative,
                    _ => RealSign::Positive,
                }
            } else {
                sign
            };
            assert_eq!(contact.tangent_cross_sign(), Some(sign));
            for (curve, location) in [(first, contact.first()), (second, contact.second())] {
                assert!(
                    curve
                        .point_at(&location.parameter().unwrap())
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
            }
        }
        let topology = first.intersection_topology(second).unwrap();
        assert_eq!(topology.first().len(), 3);
        assert_eq!(topology.second().len(), 3);
        for piece in topology.first() {
            assert!(piece.intersect_curve(second).unwrap().is_complete());
        }
    }
}
#[test]
fn translated_graphs_use_independent_domains_and_parameters() {
    let parabola = curves(&polynomial(&[&[0, 1], &[], &[-1]]));
    let horizontal = curves(&polynomial(&[&[-1, 1]]));
    let a = &parabola.curves()[0];
    let b = horizontal.curves()[0]
        .transform_similarity(&similarity([1, 0, 0, 1, 1, 0]))
        .unwrap();
    let result = a.intersect_curve(&b).unwrap();
    assert_eq!(result.contacts().len(), 2);
    for (contact, x) in result.contacts().iter().zip([-1, 1]) {
        assert_eq!(
            contact
                .first()
                .parameter()
                .unwrap()
                .compare(&Real::from(x).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            contact
                .second()
                .parameter()
                .unwrap()
                .compare(&Real::from(x - 1).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
    }
    let translated = a
        .transform_similarity(&similarity([1, 0, 0, 1, 0, 1]))
        .unwrap();
    let empty = a.intersect_curve(&translated).unwrap();
    assert!(empty.is_complete() && empty.contacts().is_empty() && empty.overlaps().is_empty());
}

#[test]
fn nonlinear_overlap_retains_bidirectional_parameter_transport() {
    use hypercurve::{CurveOverlapOrientation2, CurveParameterRange2};
    let source = curves(&polynomial(&[&[0, 1], &[], &[-1]]));
    let a = source.curves()[0]
        .subcurve(Real::zero().into(), Real::one().into())
        .unwrap();
    // The positive branch of x=y², exchanged back, is the same parabola.
    let inverse = curves(&polynomial(&[&[0, 0, 1], &[-1]]));
    let b = inverse.curves()[1]
        .transform_similarity(&similarity([0, 1, 1, 0, 0, 0]))
        .unwrap();
    for reverse in [false, true] {
        let b = if reverse {
            b.reversed().unwrap()
        } else {
            b.clone()
        };
        for (first, second) in [(&a, &b), (&b, &a)] {
            let result = first.intersect_curve(second).unwrap();
            assert!(result.is_complete() && result.contacts().is_empty());
            assert_eq!(result.overlaps().len(), 1);
            let overlap = &result.overlaps()[0];
            assert_eq!(
                overlap.orientation(),
                if reverse {
                    CurveOverlapOrientation2::Reversed
                } else {
                    CurveOverlapOrientation2::Same
                }
            );
            for (s, t) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                assert!(
                    first
                        .point_at(s)
                        .unwrap()
                        .coincides_with(&second.point_at(t).unwrap())
                        .unwrap()
                );
            }
            let quarter = (Real::one() / Real::from(4)).unwrap();
            let half = (Real::one() / Real::from(2)).unwrap();
            let (a_clip, b_clip) = if first.polynomial_branch().unwrap().topology().equation()
                == source.topology().equation()
            {
                (
                    [quarter.clone().into(), half.clone().into()],
                    [Real::zero().into(), Real::one().into()],
                )
            } else {
                (
                    [Real::zero().into(), Real::one().into()],
                    [quarter.clone().into(), half.clone().into()],
                )
            };
            let clipped = overlap.restrict(a_clip, b_clip).unwrap().unwrap();
            for (s, t) in [
                (
                    clipped.first_range().start(),
                    clipped.second_range().start(),
                ),
                (clipped.first_range().end(), clipped.second_range().end()),
            ] {
                assert!(
                    first
                        .point_at(s)
                        .unwrap()
                        .coincides_with(&second.point_at(t).unwrap())
                        .unwrap()
                );
            }
            // The intersection of the two restrictions is maximal, not just
            // an arbitrary pair of corresponding endpoint samples.
            let x = polynomial(&[&[], &[1]]);
            let mut ends = [
                first
                    .point_at(clipped.first_range().start())
                    .unwrap()
                    .rational_form(x.clone(), polynomial(&[&[1]]))
                    .unwrap(),
                first
                    .point_at(clipped.first_range().end())
                    .unwrap()
                    .rational_form(x, polynomial(&[&[1]]))
                    .unwrap(),
            ];
            if ends[0].compare(&ends[1]).unwrap().is_gt() {
                ends.swap(0, 1);
            }
            assert!(ends[0].compare_to_real(&quarter).unwrap().is_eq());
            assert!(ends[1].compare_to_real(&half).unwrap().is_eq());
            let fragment = first
                .restrict_source(
                    CurveParameterRange2::try_new(quarter.into(), half.into()).unwrap(),
                )
                .unwrap();
            assert!(fragment.intersect_curve(second).unwrap().is_complete());
        }
    }
}

#[test]
fn point_incidence_keeps_the_selected_fiber_affine_abscissa() {
    let source = curves(&polynomial(&[&[0, 1], &[], &[-1]]));
    let inverse = curves(&polynomial(&[&[0, 0, 1], &[-1]]));
    let target = inverse.curves()[1]
        .transform_similarity(&similarity([0, 1, 1, 0, 0, 0]))
        .unwrap();
    let half = (Real::one() / Real::from(2)).unwrap();
    let point = source.curves()[0].point_at(&half.into()).unwrap();
    let hypercurve::CurvePointLocations2::Locations(locations) =
        target.point_locations(&point).unwrap()
    else {
        panic!("injective graph has one source parameter")
    };
    assert_eq!(locations.len(), 1);
    let parameter = locations[0].parameter().unwrap();
    let quarter = (Real::one() / Real::from(4)).unwrap();
    assert_eq!(
        parameter.compare(&quarter.into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert!(
        target
            .point_at(&parameter)
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    let (left, right) = target.split_at(parameter).unwrap();
    assert!(left.end().coincides_with(&right.start()).unwrap());
    for piece in [left, right] {
        assert!(
            matches!(piece.point_locations(&point).unwrap(),hypercurve::CurvePointLocations2::Locations(locations) if locations.len()==1)
        );
    }
}

fn curves_in(
    equation: &BivariatePolynomial,
    first: [i32; 2],
    second: [i32; 2],
) -> PolynomialCurveSet2 {
    PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(
            equation,
            &BivariateCurveDomain {
                first: first.map(Real::from),
                second: second.map(Real::from),
            },
            hypersolve::bivariate_topology::BivariateTopologyConfig {
                max_axis_degree: 16,
                ..Default::default()
            },
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap()
}
#[test]
fn rotated_nonlinear_charts_select_the_nonrational_contact_and_requery() {
    use hyperreal::RealSign;
    let p = polynomial(&[&[0, 1], &[], &[-1]]);
    let wide = curves_in(&p, [-4, 4], [-2, 16]);
    let narrow = curves(&p);
    let a = &wide.curves()[0];
    let b = narrow.curves()[0]
        .transform_similarity(&similarity([3, -4, 4, 3, 0, 0]))
        .unwrap();
    let world_b = polynomial(&[&[0, 75, -16], &[-100, -24], &[-9]]);
    // Besides x=0, 8x³+12x²-33x+50 has exactly one real root in (-4,-3).
    // Its positive half-line is excluded by 12x²-33x+50>0; substituting -x
    // gives one sign change. This is an independent full contact census.
    let result = a.intersect_curve(&b).unwrap();
    assert!(result.is_complete() && result.overlaps().is_empty());
    assert_eq!(result.contacts().len(), 2);
    for (hit, negative) in result.contacts().iter().zip([true, false]) {
        let point = hit.point();
        for f in [p.clone(), world_b.clone()] {
            assert!(
                point
                    .rational_form(f, polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
        }
        let x = point.affine_form(1.into(), 0.into(), 0.into());
        if negative {
            assert!(x.compare_to_real(&Real::from(-4)).unwrap().is_gt());
            assert!(x.compare_to_real(&Real::from(-3)).unwrap().is_lt());
            assert!(
                point
                    .rational_form(
                        polynomial(&[&[50], &[-33], &[12], &[8]]),
                        polynomial(&[&[1]])
                    )
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
        } else {
            assert!(
                point
                    .coincides_with(&Point2::from_values(0, 0).into())
                    .unwrap()
            );
        }
        assert_eq!(
            hit.tangent_cross_sign(),
            Some(if negative {
                RealSign::Negative
            } else {
                RealSign::Positive
            })
        );
        for (curve, location) in [(a, hit.first()), (&b, hit.second())] {
            assert!(
                curve
                    .point_at(&location.parameter().unwrap())
                    .unwrap()
                    .coincides_with(point)
                    .unwrap()
            );
        }
    }
    let topology = a.intersection_topology(&b).unwrap();
    assert_eq!(topology.first().len(), 3);
    assert_eq!(topology.second().len(), 3);
    for piece in topology.first() {
        assert!(piece.intersect_curve(&b).unwrap().is_complete());
    }
}
#[test]
fn generated_quintic_column_pair_preserves_defining_power_and_selected_contact() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    let p = polynomial(&[&[5], &[5], &[], &[], &[], &[1]]);
    let cube = bivariate_multiply(&p, &bivariate_multiply(&p, &p));
    let column = curves_in(&cube, [-2, 2], [-2, 4]);
    let line = curves(&polynomial(&[&[-1, 2]]));
    let (a, b) = (&line.curves()[0], &column.curves()[0]);
    let topology = a.intersection_topology(b).unwrap();
    assert_eq!(topology.result().contacts().len(), 1);
    let point = topology.result().contacts()[0].point();
    assert!(
        point
            .rational_form(p.clone(), polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap()
            .is_eq()
    );
    assert!(
        point
            .affine_form(0.into(), 2.into(), (-1).into())
            .compare_to_real(&Real::zero())
            .unwrap()
            .is_eq()
    );
    assert!(topology.result().contacts()[0].is_certified_transverse());
    assert_eq!(topology.second().len(), 2);
    for piece in topology.second() {
        assert_eq!(piece.polynomial_branch().unwrap().defining_exponent(), 3);
        assert!(piece.intersect_curve(a).unwrap().is_complete());
    }
}

#[test]
fn mixed_common_components_keep_other_crossings_and_endpoint_contacts() {
    use hypercurve::{Curve2, LineSeg2};
    let a = curves(&polynomial(&[&[0, 0, 1], &[], &[-1]]));
    let b = curves(&polynomial(&[&[0, -1, 1], &[1, -1]]));
    assert_eq!((a.curves().len(), b.curves().len()), (4, 4));
    let authored = |ends: [i32; 4]| -> Curve2 {
        LineSeg2::try_new(
            Point2::from_values(ends[0], ends[1]),
            Point2::from_values(ends[2], ends[3]),
        )
        .unwrap()
        .into()
    };
    let a_lines = [[-2, -2, 0, 0], [-2, 2, 0, 0], [0, 0, 2, -2], [0, 0, 2, 2]].map(authored);
    let b_lines = [[-2, -2, 1, 1], [1, -2, 1, 1], [1, 1, 1, 2], [1, 1, 2, 2]].map(authored);
    for (first, expected_first) in a.curves().iter().zip(&a_lines) {
        for (second, expected_second) in b.curves().iter().zip(&b_lines) {
            let second = second
                .transform_similarity(&similarity([0, 1, 1, 0, 0, 0]))
                .unwrap();
            for reverse in [false, true] {
                let first = if reverse {
                    first.reversed().unwrap()
                } else {
                    first.clone()
                };
                let expected_first = if reverse {
                    expected_first.reversed().unwrap()
                } else {
                    expected_first.clone()
                };
                let result = first.intersect_curve(&second).unwrap();
                let expected = expected_first.intersect_curve(expected_second).unwrap();
                assert!(result.is_complete());
                // The native line kernel also lists closed overlap endpoints
                // as contacts. Polynomial pairs retain them on the overlap.
                // Audit those point identities instead of requiring duplicates.
                if expected.overlaps().is_empty() {
                    assert_eq!(result.contacts().len(), expected.contacts().len());
                } else {
                    assert!(result.contacts().is_empty());
                    assert_eq!(expected.contacts().len(), 2);
                    let span = &result.overlaps()[0];
                    for reference in expected.contacts() {
                        assert!(
                            [span.first_range().start(), span.first_range().end()]
                                .into_iter()
                                .any(|s| first
                                    .point_at(s)
                                    .unwrap()
                                    .coincides_with(reference.point())
                                    .unwrap())
                        );
                    }
                }
                assert_eq!(result.overlaps().len(), expected.overlaps().len());
                for (hit, reference) in result.contacts().iter().zip(expected.contacts()) {
                    assert!(hit.point().coincides_with(reference.point()).unwrap());
                    assert_eq!(hit.tangent_cross_sign(), reference.tangent_cross_sign());
                }
                for (overlap, reference) in result.overlaps().iter().zip(expected.overlaps()) {
                    assert_eq!(overlap.orientation(), reference.orientation());
                    for (actual, known) in [
                        (
                            overlap.first_range().start(),
                            reference.first_range().start(),
                        ),
                        (overlap.first_range().end(), reference.first_range().end()),
                    ] {
                        assert!(
                            first
                                .point_at(actual)
                                .unwrap()
                                .coincides_with(&expected_first.point_at(known).unwrap())
                                .unwrap()
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn degree_seventeen_overlap_keeps_nonmaterialized_inverse_cuts() {
    let mut rows = vec![vec![]; 18];
    rows[0] = vec![0.into(), 1.into()];
    rows[17] = vec![(-1).into()];
    let p = BivariatePolynomial::new(rows);
    let config = hypersolve::bivariate_topology::BivariateTopologyConfig {
        max_axis_degree: 32,
        ..Default::default()
    };
    let domain = BivariateCurveDomain {
        first: [(-1).into(), 1.into()],
        second: [(-2).into(), 2.into()],
    };
    let a = PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(&p, &domain, config).unwrap(),
        Default::default(),
    )
    .unwrap();
    let inverse = hypersolve::bivariate_arithmetic::bivariate_swap_parameters(&p);
    let b = PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(&inverse, &domain, config).unwrap(),
        Default::default(),
    )
    .unwrap();
    assert_eq!((a.curves().len(), b.curves().len()), (1, 2));
    let first = &a.curves()[0];
    let second = b.curves()[0]
        .transform_similarity(&similarity([0, 1, 1, 0, 0, 0]))
        .unwrap();
    let result = first.intersect_curve(&second).unwrap();
    assert!(result.is_complete() && result.contacts().is_empty());
    assert_eq!(result.overlaps().len(), 1);
    let lower = (Real::from(-1) / Real::from(1_u32 << 16)).unwrap();
    let upper = (Real::from(-1) / Real::from(1_u32 << 18)).unwrap();
    let clipped = result.overlaps()[0]
        .restrict(
            [Real::from(-1).into(), Real::zero().into()],
            [lower.clone().into(), upper.clone().into()],
        )
        .unwrap()
        .unwrap();
    for (s, t, boundary) in [
        (
            clipped.first_range().start(),
            clipped.second_range().start(),
            &lower,
        ),
        (
            clipped.first_range().end(),
            clipped.second_range().end(),
            &upper,
        ),
    ] {
        assert_eq!(
            t.compare(&boundary.clone().into()).unwrap(),
            std::cmp::Ordering::Equal
        );
        let point = first.point_at(s).unwrap();
        assert!(point.coincides_with(&second.point_at(t).unwrap()).unwrap());
        assert!(
            point
                .rational_form(p.clone(), polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap()
                .is_eq()
        );
        assert!(
            matches!(first.point_locations(&point).unwrap(),hypercurve::CurvePointLocations2::Locations(locations) if locations.len()==1)
        );
    }
}

#[test]
fn independent_quintic_and_septic_supports_keep_the_joint_selected_system() {
    let horizontal = curves(&polynomial(&[&[5, 5, 0, 0, 0, 1]]));
    let vertical = curves(&polynomial(&[&[7], &[7], &[], &[], &[], &[], &[], &[1]]));
    assert_eq!((horizontal.curves().len(), vertical.curves().len()), (1, 1));
    let (a, b) = (&horizontal.curves()[0], &vertical.curves()[0]);
    let result = a.intersect_curve(b).unwrap();
    assert!(result.is_complete() && result.overlaps().is_empty());
    assert_eq!(result.contacts().len(), 1);
    let point = result.contacts()[0].point();
    assert!(point.coordinates().is_none());
    // Both Eisenstein polynomials are irreducible and strictly increasing.
    // The two field degrees are coprime, hence their joint degree is 35.
    for equation in [
        polynomial(&[&[5, 5, 0, 0, 0, 1]]),
        polynomial(&[&[7], &[7], &[], &[], &[], &[], &[], &[1]]),
    ] {
        assert!(
            point
                .rational_form(equation, polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap()
                .is_eq()
        );
    }
    assert!(result.contacts()[0].is_certified_transverse());
    let restored = point.definition().unwrap().into_point().unwrap();
    assert!(restored.coincides_with(point).unwrap());
    for curve in [a, b] {
        let hypercurve::CurvePointLocations2::Locations(locations) =
            curve.point_locations(&restored).unwrap()
        else {
            panic!("one source parameter")
        };
        assert_eq!(locations.len(), 1);
        assert!(
            curve
                .point_at(&locations[0].parameter().unwrap())
                .unwrap()
                .coincides_with(&restored)
                .unwrap()
        );
    }
    let topology = a.intersection_topology(b).unwrap();
    assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
    assert_eq!(
        topology.first()[0]
            .intersect_curve(&topology.second()[1])
            .unwrap()
            .contacts()
            .len(),
        1
    );
}

#[test]
fn rotated_circle_overlaps_feed_the_same_filled_set_arrangement() {
    use hypercurve::{BooleanOp, CurvePath2, CurveRegion2, FillRule};
    let source = curves(&polynomial(&[&[-1, 0, 1], &[], &[1]]));
    assert_eq!(source.curves().len(), 2);
    let q = |n: i32| (Real::from(n) / Real::from(5)).unwrap();
    let rotation =
        hypercurve::Similarity2::try_from_real_affine(q(3), q(-4), q(4), q(3), 0.into(), 0.into())
            .unwrap();
    let a = &source.curves()[1];
    let b = a.transform_similarity(&rotation).unwrap();
    let result = a.intersect_curve(&b).unwrap();
    assert!(result.is_complete() && result.contacts().is_empty());
    assert_eq!(result.overlaps().len(), 1);
    let overlap = &result.overlaps()[0];
    assert_eq!(
        overlap
            .first_range()
            .start()
            .compare(&Real::from(-1).into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        overlap.first_range().end().compare(&q(3).into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        overlap
            .second_range()
            .start()
            .compare(&q(-3).into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        overlap
            .second_range()
            .end()
            .compare(&Real::one().into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    let region = |transformed: bool| {
        let pieces = [
            source.curves()[0].clone(),
            source.curves()[1].reversed().unwrap(),
        ]
        .into_iter()
        .map(|c| {
            if transformed {
                c.transform_similarity(&rotation).unwrap()
            } else {
                c
            }
        })
        .collect();
        CurveRegion2::try_from_boundary_paths(
            &[CurvePath2::try_new(pieces).unwrap()],
            FillRule::NonZero,
        )
        .unwrap()
    };
    let first = region(false);
    let second = region(true);
    assert!(
        first
            .boolean_region(&second, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert!(
        first
            .boolean_region(&second, BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );
    for operation in [BooleanOp::Union, BooleanOp::Intersection] {
        let combined = first.boolean_region(&second, operation).unwrap();
        assert!(
            combined
                .boolean_region(&first, BooleanOp::Xor)
                .unwrap()
                .is_empty()
        );
        let rebuilt = CurveRegion2::try_from_boundary_paths(
            &combined.boundary_paths().unwrap(),
            FillRule::EvenOdd,
        )
        .unwrap();
        assert!(
            rebuilt
                .boolean_region(&second, BooleanOp::Xor)
                .unwrap()
                .is_empty()
        );
    }
}

#[test]
fn opposite_reducible_roots_and_tiny_distinct_translations_stay_separate() {
    use hypercurve::CurveOverlapOrientation2;
    let source = curves(&polynomial(&[&[6, 0, -5, 0, 1]]));
    assert_eq!(source.curves().len(), 4);
    for (i, a) in source.curves().iter().enumerate() {
        for (j, b) in source.curves().iter().enumerate() {
            let reflected = b
                .transform_similarity(&similarity([-1, 0, 0, -1, 0, 0]))
                .unwrap();
            let result = a.intersect_curve(&reflected).unwrap();
            assert!(result.is_complete() && result.contacts().is_empty());
            assert_eq!(result.overlaps().len(), usize::from(i + j == 3));
            if let Some(overlap) = result.overlaps().first() {
                assert_eq!(overlap.orientation(), CurveOverlapOrientation2::Reversed);
            }
        }
    }
    let tiny = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let translation = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        tiny,
    )
    .unwrap();
    for a in source.curves() {
        for b in source.curves() {
            let shifted = b.transform_similarity(&translation).unwrap();
            let result = a.intersect_curve(&shifted).unwrap();
            assert!(result.is_complete() && result.is_disjoint());
        }
    }
}

#[test]
fn separate_chart_resource_denial_cancellation_and_policy_retry() {
    use hypercurve::{PredicatePolicy, evaluate_under};
    use hypersolve::work_budget::{self, CancellationToken, WorkBudget, WorkKind};
    let create = || {
        let a = curves(&polynomial(&[&[-1, 37]]));
        let b = curves(&polynomial(&[&[7], &[7], &[], &[], &[], &[], &[], &[1]]));
        let shift = (Real::one() / Real::from(8)).unwrap();
        let map = hypercurve::Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            shift,
            0.into(),
        )
        .unwrap();
        (
            a.curves()[0].clone(),
            b.curves()[0].transform_similarity(&map).unwrap(),
        )
    };
    let (a, b) = create();
    let zero =
        work_budget::with_budget(WorkBudget::ZERO, || assert!(a.intersect_curve(&b).is_err()));
    let exhausted = zero.result.unwrap_err();
    assert_eq!(exhausted.kind, WorkKind::AlgebraicPromotion);
    assert_eq!(
        (exhausted.limit, exhausted.used, exhausted.requested),
        (0, 0, 1)
    );
    let cancel = CancellationToken::new();
    cancel.cancel();
    let cancelled = work_budget::with_cancellable_budget(WorkBudget::UNLIMITED, &cancel, || {
        assert!(a.intersect_curve(&b).is_err())
    });
    assert_eq!(cancelled.result.unwrap_err().kind, WorkKind::Cancellation);
    let retry = work_budget::with_budget(
        WorkBudget {
            refinements: 100_000,
            promotions: 100_000,
        },
        || a.intersection_topology(&b),
    );
    let topology = retry.result.unwrap().unwrap();
    assert!(topology.result().is_complete());
    assert_eq!(topology.result().contacts().len(), 1);
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = evaluate_under(policy, || {
            let (a, b) = create();
            let topology = a.intersection_topology(&b).unwrap();
            assert_eq!(topology.result().contacts().len(), 1);
            assert!(
                topology.first()[0]
                    .intersect_curve(&topology.second()[1])
                    .unwrap()
                    .is_complete()
            );
        });
        result.certified().unwrap();
    }
}

#[cfg(all(feature = "svg", not(target_arch = "wasm32")))]
#[test]
fn selected_chart_pair_cuts_replay_cold_and_feed_another_intersection() {
    use hypercurve::{
        CurvePath2, CurveRegion2, SvgGeometry2, export_svg_document, import_svg_document,
    };
    const INPUT: &str = "HYPERCURVE_PAIR_CHART_REPLAY";
    fn verify(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        assert_eq!(geometry.paths().len(), 2);
        let a = &geometry.paths()[0].curves()[0];
        let b = &geometry.paths()[1].curves()[0];
        let result = a.intersect_curve(b).unwrap();
        assert!(result.is_complete() && result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 1);
        let point = result.contacts()[0].point();
        assert!(result.contacts()[0].is_certified_transverse());
        for equation in [
            polynomial(&[&[5, 5, 0, 0, 0, 1]]),
            polynomial(&[&[7], &[7], &[], &[], &[], &[], &[], &[1]]),
        ] {
            assert!(
                point
                    .rational_form(equation, polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
        }
        assert!(a.start().coincides_with(point).unwrap());
        assert!(b.end().coincides_with(point).unwrap());
        for curve in [a, b] {
            let hypercurve::CurvePointLocations2::Locations(locations) =
                curve.point_locations(point).unwrap()
            else {
                panic!("one selected source parameter");
            };
            assert_eq!(locations.len(), 1);
            assert!(
                curve
                    .point_at(&locations[0].parameter().unwrap())
                    .unwrap()
                    .coincides_with(point)
                    .unwrap()
            );
        }
        assert!(a.intersection_topology(b).unwrap().result().is_complete());
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let horizontal = curves(&polynomial(&[&[5, 5, 0, 0, 0, 1]]));
    let vertical = curves(&polynomial(&[&[7], &[7], &[], &[], &[], &[], &[], &[1]]));
    let topology = horizontal.curves()[0]
        .intersection_topology(&vertical.curves()[0])
        .unwrap();
    let geometry = SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![
            CurvePath2::try_new(vec![topology.first()[0].reversed().unwrap()]).unwrap(),
            CurvePath2::try_new(vec![topology.second()[0].clone()]).unwrap(),
        ],
    );
    let text = export_svg_document(&geometry).unwrap();
    verify(&text);
    let path =
        std::env::temp_dir().join(format!("hypercurve-pair-chart-{}.svg", std::process::id()));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_chart_pair_cuts_replay_cold_and_feed_another_intersection",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
