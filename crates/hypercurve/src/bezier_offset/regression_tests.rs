//! Regression and kernel tests for bezier_offset internals.

use super::*;

mod chord_overlap_transport_tests {
    use super::*;

    fn exact<T: std::fmt::Debug>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("{reason:?}"),
        }
    }
    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn point(x: Real) -> CurvePoint2 {
        Point2::new(x, Real::zero()).into()
    }
    fn source() -> RationalBezier2 {
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
            ],
            vec![Real::one(); 3],
        )
        .unwrap()
    }
    fn overlap(reversed: bool, policy: &CurveContext) -> BezierAlgebraicChordRationalOverlap2 {
        let chord = exact(
            BezierAlgebraicChord2::try_new(point(Real::zero()), point(Real::one()), policy)
                .unwrap(),
        );
        let source = if reversed {
            source().reversed()
        } else {
            source()
        };
        let BezierAlgebraicChordRationalIntersections2::Overlaps(mut overlaps) = exact(
            chord
                .rational_intersections(&source, &CurveParameterRange2::unit(), None, policy)
                .unwrap(),
        ) else {
            panic!("one monotone nonlinear line image")
        };
        assert_eq!(overlaps.len(), 1);
        overlaps.pop().unwrap()
    }
    fn chord_cut(
        overlap: &BezierAlgebraicChordRationalOverlap2,
        a: Real,
        b: Real,
        policy: &CurveContext,
    ) -> CurveParameterRange2 {
        let parameter = |value| {
            CurveParameter2::from_algebraic_chord(
                overlap
                    .chord
                    .parameter_at_certified_support_point(point(value), policy)
                    .unwrap(),
            )
        };
        CurveParameterRange2::new_validated(parameter(a), parameter(b))
    }
    fn replay(
        overlap: &BezierAlgebraicChordRationalOverlap2,
        ranges: &(CurveParameterRange2, CurveParameterRange2),
        policy: &CurveContext,
    ) {
        for (chord, source) in [
            (ranges.0.start(), ranges.1.start()),
            (ranges.0.end(), ranges.1.end()),
        ] {
            let expected = chord.as_algebraic_chord().unwrap().point();
            let actual = exact(
                rational_point_evidence_at_region_parameter(&overlap.source, source, policy)
                    .unwrap(),
            );
            assert_eq!(
                actual.same_point(expected, policy),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn nonlinear_chord_overlap_clips_either_operand_and_keeps_paired_orientation() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let overlap = overlap(reversed, &policy);
                let chord = chord_cut(&overlap, q(1, 3), q(2, 3), &policy);
                let clipped = exact(
                    overlap
                        .clipped_ranges(&chord, &CurveParameterRange2::unit(), &policy)
                        .unwrap(),
                )
                .unwrap();
                replay(&overlap, &clipped, &policy);
                assert_eq!(
                    exact(
                        clipped
                            .1
                            .start()
                            .cmp_by_refinement(clipped.1.end(), &policy)
                            .unwrap()
                    )
                    .is_gt(),
                    reversed
                );
                let excluded = if reversed {
                    CurveParameterRange2::new_validated(q(3, 4).into(), Real::one().into())
                } else {
                    CurveParameterRange2::new_validated(Real::zero().into(), q(1, 4).into())
                };
                assert!(
                    exact(overlap.clipped_ranges(&chord, &excluded, &policy).unwrap()).is_none()
                );
                let source_cut = if reversed {
                    CurveParameterRange2::new_validated(q(1, 4).into(), q(1, 3).into())
                } else {
                    CurveParameterRange2::new_validated(q(2, 3).into(), q(3, 4).into())
                };
                let clipped = exact(
                    overlap
                        .clipped_ranges(&chord, &source_cut, &policy)
                        .unwrap(),
                )
                .unwrap();
                replay(&overlap, &clipped, &policy);
            }
        }
    }

    #[test]
    fn chord_overlap_inverse_uses_its_finite_exterior_source_domain() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // x=t^2 is strictly increasing on [1,2]. These exact endpoints
            // certify the complete monotone correspondence with chord [1,4].
            let chord = exact(
                BezierAlgebraicChord2::try_new(point(Real::one()), point(Real::from(4)), &policy)
                    .unwrap(),
            );
            let overlap = BezierAlgebraicChordRationalOverlap2 {
                chord_range: [chord.start_parameter(), chord.end_parameter()],
                chord,
                source: source(),
                source_range: CurveParameterRange2::new_validated(
                    Real::one().into(),
                    Real::from(2).into(),
                ),
                orientation: CurveOverlapOrientation2::Same,
            };
            let cut = chord_cut(&overlap, Real::from(2), Real::from(3), &policy);
            let clipped = exact(
                overlap
                    .clipped_ranges(&cut, &overlap.source_range, &policy)
                    .unwrap(),
            )
            .unwrap();
            replay(&overlap, &clipped, &policy);
            for parameter in [clipped.1.start(), clipped.1.end()] {
                assert_eq!(
                    exact(
                        parameter
                            .cmp_by_refinement(&CurveParameter2::from(Real::one()), &policy)
                            .unwrap()
                    ),
                    std::cmp::Ordering::Greater
                );
            }
        }
    }

    #[test]
    fn chord_overlap_inverse_reuses_unprojected_selected_parameter_identity() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let overlap = overlap(false, &policy);
            let retained = CurveParameter2::from_selected_fiber(
                degree_nine_selected_fiber_parameter_for_test(q(1, 2), 2, &policy),
            );
            let mapped = exact(
                overlap
                    .chord_parameter_at_source_parameter(&retained, &policy)
                    .unwrap(),
            )
            .unwrap();
            let chord = CurveParameterRange2::new_validated(
                CurveParameter2::from_algebraic_chord(mapped),
                CurveParameter2::from_algebraic_chord(overlap.chord.end_parameter()),
            );
            let mut source = CurveParameterRange2::unit();
            for _ in 0..8 {
                let ranges =
                    exact(overlap.clipped_ranges(&chord, &source, &policy).unwrap()).unwrap();
                assert_eq!(ranges.1.start(), &retained);
                assert!(ranges.1.start().as_selected_fiber().is_some());
                source = ranges.1;
            }
        }
    }
}

mod parallel_normal_source_angle_tests {
    use super::*;
    use std::cmp::Ordering;

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact fixture: {reason:?}"),
        }
    }

    #[test]
    fn selected_chord_parallel_normal_angles_use_source_direction() {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        // P(t)=(t,t²) has source tangent (1,0) at t=0. The distance-one
        // parallel has the opposite tangent there: Q'(0)=(-1,0). Both
        // normal frames still use the source's upward unit normal.
        let source = BezierParallelSource2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            Point2::from_values(1, 1),
        ));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for distance in [0, 1] {
                let parallel = BezierParallel2::from_source(source.clone(), Real::from(distance));
                let selected = BezierParameter2::Exact(Real::zero());
                assert_eq!(
                    decided(
                        parallel
                            .parallel_derivative_scale_sign(&selected.clone().into(), &policy)
                            .unwrap()
                    ),
                    if distance == 0 {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    }
                );
                for clockwise in [false, true] {
                    let tangent = (Real::from(4), Real::from(if clockwise { -3 } else { 3 }));
                    for unit_evidence in [false, true] {
                        let chord = if unit_evidence {
                            let unit = decided(
                                crate::direction::UnitDirection2::from_direction(&tangent).unwrap(),
                            );
                            decided(
                                BezierAlgebraicChord2::from_unit_direction(&unit, &policy).unwrap(),
                            )
                        } else {
                            decided(
                                BezierAlgebraicChord2::try_new(
                                    Point2::from_values(0, 0).into(),
                                    Point2::new(tangent.0.clone(), tangent.1.clone()).into(),
                                    &policy,
                                )
                                .unwrap(),
                            )
                        };
                        for radius in [-1, 1] {
                            let radius = Real::from(radius);
                            let circle = decided(
                                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                                    parallel.clone(),
                                    selected.clone().into(),
                                    radius.clone(),
                                    clockwise,
                                    &policy,
                                )
                                .unwrap(),
                            )
                            .unwrap();
                            let center = CurvePoint2::from(BezierAnalyticParallelPoint2::new(
                                parallel.clone(),
                                selected.clone(),
                                &policy,
                            ));
                            let contact = chord
                                .normal_displaced_point_evidence(center, radius.clone(), &policy)
                                .unwrap();
                            let parameter = decided(
                                circle
                                    .certified_selected_chord_parallel_normal_contact_parameter(
                                        chord.clone(),
                                        contact.clone(),
                                        radius.clone(),
                                        if clockwise {
                                            RealSign::Negative
                                        } else {
                                            RealSign::Positive
                                        },
                                        &policy,
                                    )
                                    .unwrap(),
                            );
                            // At u=1/4 the half-circle chart has cos=4/5,
                            // sin=3/5. This independent 3-4-5 construction
                            // fixes the contact point and its exact angular order.
                            let expected = CurvePoint2::from(Point2::new(
                                -tangent.1.clone() * q(1, 5) * &radius,
                                Real::from(distance) + q(4, 5) * &radius,
                            ));
                            assert_eq!(
                                contact.same_point(&expected, &policy),
                                Classification::Decided(true)
                            );
                            let evaluated =
                                decided(circle.point_evidence_at(&q(1, 4), &policy).unwrap());
                            assert_eq!(
                                evaluated.same_point(&expected, &policy),
                                Classification::Decided(true)
                            );
                            for (cut, expected_order) in [
                                (q(1, 8), Ordering::Greater),
                                (q(1, 4), Ordering::Equal),
                                (q(3, 8), Ordering::Less),
                            ] {
                                assert_eq!(
                                    parameter.order_to_real(&cut, &policy).unwrap(),
                                    Classification::Decided(expected_order),
                                    "distance={distance} clockwise={clockwise} unit_evidence={unit_evidence} radius={radius:?} cut={cut:?} policy={policy:?}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

mod regular_parallel_contact_tests {
    use super::*;
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn p(x: i64, y: i64) -> Point2 {
        Point2::from_values(x, y)
    }
    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("regular-cell query blocked: {reason:?}"),
        }
    }
    #[test]
    fn primitive_source_frames_share_factorization_across_distances_and_branches() {
        // C(t)=(t²,t³), t=2u-1. Its one-sided normals at u=1/2
        // point in opposite vertical directions.
        let source = CubicBezier2::new(
            p(1, -1),
            Point2::new(q(-1, 3), Real::one()),
            Point2::new(q(-1, 3), -Real::one()),
            p(1, 1),
        )
        .parallel_left(Real::one())
        .unwrap();
        let cusp: CurveParameter2 = q(1, 2).into();
        let ranges = [
            CurveParameterRange2::new_validated(Real::zero().into(), cusp.clone()),
            CurveParameterRange2::new_validated(cusp.clone(), Real::one().into()),
        ];
        // An approximate caller still obtains a STRICT factorization proof
        // that a later strict query can reuse.
        let mut previous_frames: Option<[Arc<BezierAnalyticParallelTangentField2>; 2]> = None;
        for policy in [CurveContext::APPROXIMATE_512, CurveContext::STRICT] {
            let shifted = source.with_distance(q(7, 5));
            let frames = ranges.each_ref().map(|range| {
                decided(
                    source
                        .source_oriented_regularized_tangent_field(range, &policy)
                        .unwrap(),
                )
                .expect("a stationary cubic has a nonconstant tangent factor")
            });
            assert!(!Arc::ptr_eq(&frames[0], &frames[1]));
            for axis in 0..2 {
                let reversed = CurveParameterRange2::new_validated(
                    ranges[axis].end().clone(),
                    ranges[axis].start().clone(),
                );
                let reused = decided(
                    shifted
                        .source_oriented_regularized_tangent_field(&reversed, &policy)
                        .unwrap(),
                )
                .unwrap();
                assert!(
                    Arc::ptr_eq(&frames[axis], &reused),
                    "the same source sheet must share its field across distances and range reversal"
                );
                if let Some(prior) = &previous_frames {
                    assert!(Arc::ptr_eq(&prior[axis], &reused));
                }
                let point = decided(
                    shifted
                        .point_evidence_on_regular_range(&cusp, &ranges[axis], &policy)
                        .unwrap(),
                );
                let expected = CurvePoint2::from(Point2::new(
                    Real::zero(),
                    if axis == 0 { q(-7, 5) } else { q(7, 5) },
                ));
                assert_eq!(
                    point.same_point(&expected, &policy),
                    Classification::Decided(true)
                );
            }
            previous_frames = Some(frames);
        }
    }

    #[test]
    fn regular_source_cells_preserve_reversed_and_algebraic_range_boundaries() {
        let source = CubicBezier2::new(
            p(1, -1),
            Point2::new(q(-1, 3), Real::one()),
            Point2::new(q(-1, 3), -Real::one()),
            p(1, 1),
        )
        .parallel_left(Real::zero())
        .unwrap();
        // C(t)=(t²,t³), t=2u-1. Source speed vanishes only at u=1/2.
        // Curvature is 6/(|t|*(4+9t²)^(3/2)), decreasing with |t|.
        // Distance 125/96 therefore contributes exactly two parallel cusps,
        // at t=±1/2, i.e. u=1/4 and 3/4.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let root = |leading, lower, upper| {
                let polynomial = decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        vec![-Real::one(), Real::zero(), Real::from(leading)],
                        &policy,
                    )
                    .unwrap(),
                );
                let interval =
                    decided(BezierParameterInterval::try_new(lower, upper, &policy).unwrap());
                CurveParameter2::from(BezierParameter2::Algebraic(decided(
                    BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
                )))
            };
            let algebraic = [
                root(8_i32, q(1, 4), q(1, 2)),
                root(2_i32, q(1, 2), Real::one()),
            ];
            for distance in [Real::zero(), q(125, 96)] {
                let parallel = source.with_distance(distance.clone());
                for cropped in [false, true] {
                    let endpoints = if cropped {
                        algebraic.clone()
                    } else {
                        [
                            CurveParameter2::from(Real::zero()),
                            CurveParameter2::from(Real::one()),
                        ]
                    };
                    let mut expected = vec![endpoints[0].clone()];
                    if !cropped && distance != Real::zero() {
                        expected.push(q(1, 4).into());
                    }
                    expected.push(q(1, 2).into());
                    if !cropped && distance != Real::zero() {
                        expected.push(q(3, 4).into());
                    }
                    expected.push(endpoints[1].clone());
                    for reversed in [false, true] {
                        let order = if reversed { [1, 0] } else { [0, 1] };
                        let range = CurveParameterRange2::new_validated(
                            endpoints[order[0]].clone(),
                            endpoints[order[1]].clone(),
                        );
                        let analysis =
                            decided(parallel.singularity_analysis(&range, &policy).unwrap());
                        let cells = decided(analysis.regular_subranges(&policy).unwrap());
                        assert_eq!(cells.len(), expected.len() - 1);
                        // The outer endpoints retain their exact parameter
                        // authorities, even when the query traversed them backwards.
                        assert!(cells.first().unwrap().start() == &endpoints[0]);
                        assert!(cells.last().unwrap().end() == &endpoints[1]);
                        for (cell, endpoints) in cells.iter().zip(expected.windows(2)) {
                            for (actual, expected) in
                                [cell.start(), cell.end()].into_iter().zip(endpoints)
                            {
                                assert!(decided(actual.same_value(expected, &policy).unwrap()));
                            }
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn stationary_component_queries_keep_one_sided_endpoints_and_exact_constraints() {
        use CurveParameterComponentSelection2::{Empty, NeedsConstraint, Selected};
        let parallel = CubicBezier2::new(
            p(0, 0),
            p(0, 0),
            Point2::new(q(1, 3), Real::zero()),
            p(1, 1),
        )
        .parallel_left(-Real::one())
        .unwrap();
        // C(t)=(t²,t³) has a right-hand normal at t=0. The same
        // offset on both axes retains u=v, including that limiting endpoint.
        // Its right offset is injective here: the x component of its tangent
        // is positive away from zero and its curvature scale is positive.
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for upper in [q(1, 2), Real::one()] {
                let range =
                    CurveParameterRange2::new_validated(Real::zero().into(), upper.clone().into());
                for reversed in [false, true] {
                    let range = if reversed {
                        CurveParameterRange2::new_validated(
                            range.end().clone(),
                            range.start().clone(),
                        )
                    } else {
                        range.clone()
                    };
                    for inclusion in [[true, true], [true, false], [false, true], [false, false]] {
                        for query in [
                            ParameterComponentQuery2::AllComponents(None),
                            ParameterComponentQuery2::FirstComponent(None),
                        ] {
                            let result = decided(
                                parallel
                                    .parallel_intersections_in_domain(
                                        &parallel,
                                        [CurveParameterDomain2::new(&range, None)
                                            .with_finite_inclusion(inclusion);
                                            2],
                                        query,
                                        &policy,
                                    )
                                    .unwrap(),
                            );
                            assert!(result.intersections.is_complete());
                            assert_eq!(result.components.len(), 1);
                            let component = &result.components[0];
                            assert!(matches!(
                                decided(component.constrain([None, None], &policy).unwrap()),
                                NeedsConstraint
                            ));
                            for (value, included) in [
                                (Real::zero(), inclusion[0]),
                                (q(1, 4), true),
                                (upper.clone(), inclusion[1]),
                            ] {
                                let parameter = CurveParameter2::from(value);
                                for axis in 0..2 {
                                    let constraints =
                                        [0, 1].map(|index| (index == axis).then_some(&parameter));
                                    let selection =
                                        decided(component.constrain(constraints, &policy).unwrap());
                                    if !included {
                                        assert!(matches!(selection, Empty));
                                        continue;
                                    }
                                    let Selected(pair) = selection else {
                                        panic!(
                                            "an owned exact contact must select the stationary family"
                                        );
                                    };
                                    for actual in pair {
                                        assert!(decided(
                                            actual.same_value(&parameter, &policy).unwrap()
                                        ));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn stationary_endpoint_line_chart_reuses_regular_pair_incidence() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let radius = q(15, 16);
            let first = QuadraticBezier2::new(p(0, -2), p(0, -1), p(0, 0))
                .parallel_left(-radius.clone())
                .unwrap();
            let second = RationalBezier2::try_new(
                vec![
                    p(0, 0),
                    p(0, 0),
                    Point2::new(q(1, 6), Real::zero()),
                    Point2::new(q(1, 2), Real::zero()),
                    p(1, 1),
                ],
                vec![Real::one(); 5],
            )
            .unwrap()
            .parallel_left(-radius)
            .unwrap();
            let unit = CurveParameterRange2::unit();
            let result = decided(
                first
                    .parallel_intersections_on_regular_ranges(&second, &unit, &unit, &policy)
                    .unwrap(),
            );
            assert!(result.is_complete());
            assert!(result.overlaps().is_empty());
            assert_eq!(result.contacts().len(), 1);
            let contact = &result.contacts()[0];
            assert_eq!(
                contact
                    .first_parameter()
                    .polynomial_sign(&[Real::from(-89), Real::from(128)], &policy)
                    .unwrap(),
                Classification::Decided(RealSign::Zero)
            );
            assert_eq!(
                contact
                    .second_parameter()
                    .polynomial_sign(&[Real::from(-3), Real::zero(), Real::from(8)], &policy)
                    .unwrap(),
                Classification::Decided(RealSign::Zero)
            );
        }
    }
    #[test]
    fn interior_source_cusp_reuses_owned_left_cell() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let first = CubicBezier2::new(
                p(1, -1),
                Point2::new(q(-1, 3), Real::one()),
                Point2::new(q(-1, 3), -Real::one()),
                p(1, 1),
            )
            .parallel_left(Real::one())
            .unwrap();
            let second =
                QuadraticBezier2::new(p(1, 1), Point2::new(-Real::one(), q(-1, 2)), p(-3, -2))
                    .parallel_left(Real::one())
                    .unwrap();
            // On t in [-1/4,0), curvature is greater than one: its
            // reciprocal squared is at most 73^3 / (36*256^2) < 1.
            // Both this center locus and its source therefore have regular
            // interiors on u in [3/8,1/2], with owned one-sided cusp limits.
            let left = CurveParameterRange2::new_validated(q(3, 8).into(), q(1, 2).into());
            let result = decided(
                first
                    .parallel_intersections_on_regular_ranges(
                        &second,
                        &left,
                        &CurveParameterRange2::unit(),
                        &policy,
                    )
                    .unwrap(),
            );
            assert!(result.is_complete());
            assert!(result.overlaps().is_empty());
            assert_eq!(result.contacts().len(), 1);
            let contact = &result.contacts()[0];
            assert_eq!(
                contact
                    .first_parameter()
                    .polynomial_sign(&[-Real::one(), Real::from(2)], &policy)
                    .unwrap(),
                Classification::Decided(RealSign::Zero)
            );
            assert_eq!(
                contact
                    .second_parameter()
                    .polynomial_sign(&[Real::from(-2), Real::from(5)], &policy)
                    .unwrap(),
                Classification::Decided(RealSign::Zero)
            );
            let center = decided(
                first
                    .point_evidence_on_regular_range(contact.first_parameter(), &left, &policy)
                    .unwrap(),
            );
            assert_eq!(
                center.same_point(&p(0, -1).into(), &policy),
                Classification::Decided(true)
            );
        }
    }
    #[test]
    fn regular_source_frame_keeps_contact_scale_across_a_center_cusp() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let x = q(-3, 16);
            let first = QuadraticBezier2::new(
                Point2::new(x.clone(), Real::from(-2)),
                Point2::new(x.clone(), Real::zero()),
                Point2::new(x, Real::from(2)),
            )
            .parallel_left(Real::zero())
            .unwrap();
            let second = RationalBezier2::try_new(
                vec![
                    p(0, 0),
                    p(0, 0),
                    Point2::new(q(1, 6), Real::zero()),
                    Point2::new(q(1, 2), Real::zero()),
                    p(1, 1),
                ],
                vec![Real::one(); 5],
            )
            .unwrap()
            .parallel_left(q(15, 16))
            .unwrap();
            // At u=sqrt(3/8), the source is (3/8,9/64), its unit
            // normal is (-3/5,4/5), and the offset is (-3/16,57/64).
            // The center/source derivative scale is 1/25 > 0 there.
            // At the unit range midpoint u=1/2 it is 1-3/sqrt(5) < 0.
            let unit = CurveParameterRange2::unit();
            let result = decided(
                first
                    .parallel_intersections_on_regular_ranges(&second, &unit, &unit, &policy)
                    .unwrap(),
            );
            assert!(result.is_complete());
            let mut found = false;
            for contact in result.contacts() {
                if contact
                    .second_parameter()
                    .polynomial_sign(&[Real::from(-3), Real::zero(), Real::from(8)], &policy)
                    .unwrap()
                    == Classification::Decided(RealSign::Zero)
                {
                    found = true;
                    assert_eq!(
                        contact
                            .first_parameter()
                            .polynomial_sign(&[Real::from(-185), Real::from(256)], &policy)
                            .unwrap(),
                        Classification::Decided(RealSign::Zero)
                    );
                    assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Negative));
                    assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Positive));
                }
            }
            assert!(found, "the independently known contact was not enumerated");
        }
    }

    #[test]
    fn owned_endpoint_tangents_replay_both_sides_of_stationary_parameters() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // Q(t)=(t,t²) has curvature 128/125 at t=3/8. Its left
            // parallel of radius 125/128 therefore has a cusp there,
            // with negative orientation before it and positive after it.
            let parallel =
                QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 1))
                    .parallel_left(q(125, 128))
                    .unwrap();
            let cusp: CurveParameter2 = q(3, 8).into();
            let left = CurveParameterRange2::new_validated(Real::zero().into(), cusp.clone());
            let right = CurveParameterRange2::new_validated(cusp.clone(), Real::one().into());
            for (range, expected) in [(&left, RealSign::Negative), (&right, RealSign::Positive)] {
                assert_eq!(
                    parallel
                        .vector_tangent_cross_and_dot_signs_on_regular_range(
                            &cusp,
                            &Real::one(),
                            &Real::zero(),
                            range,
                            &policy
                        )
                        .unwrap(),
                    Classification::Decided((expected, expected))
                );
            }
            assert_eq!(
                parallel
                    .vector_tangent_cross_and_dot_signs_on_regular_range(
                        &cusp,
                        &Real::one(),
                        &Real::zero(),
                        &CurveParameterRange2::unit(),
                        &policy
                    )
                    .unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            );

            // B(u)=(u²,u⁴) approaches Q(0) in opposite source directions
            // from the two sides. For d=15/16, 1-d*kappa tends to 23/8
            // on the left and -7/8 on the right. The cancelled hodograph
            // and local curvature predicates must keep both normal sheets.
            let stationary = RationalBezier2::try_new(
                vec![
                    p(0, 0),
                    p(0, 0),
                    Point2::new(q(1, 6), Real::zero()),
                    Point2::new(q(1, 2), Real::zero()),
                    p(1, 1),
                ],
                vec![Real::one(); 5],
            )
            .unwrap()
            .parallel_left(q(15, 16))
            .unwrap();
            let cusp: CurveParameter2 = Real::zero().into();
            let left = CurveParameterRange2::new_validated((-Real::one()).into(), cusp.clone());
            let right = CurveParameterRange2::unit();
            for (range, expected, y) in [
                (&left, RealSign::Positive, q(-15, 16)),
                (&right, RealSign::Negative, q(15, 16)),
            ] {
                assert_eq!(
                    stationary
                        .parallel_derivative_scale_sign_on_regular_range(&cusp, range, &policy)
                        .unwrap(),
                    Classification::Decided(expected)
                );
                let point = decided(
                    stationary
                        .point_evidence_on_regular_range(&cusp, range, &policy)
                        .unwrap(),
                );
                assert_eq!(
                    point.same_point(&Point2::new(Real::zero(), y).into(), &policy),
                    Classification::Decided(true)
                );
            }
        }
    }
    #[test]
    fn exterior_regular_pair_keeps_contacts_outside_ancestral_bounds() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for stationary_chart in [false, true] {
                let points = if stationary_chart {
                    vec![
                        p(0, 0),
                        p(0, 0),
                        Point2::new(q(1, 6), Real::zero()),
                        Point2::new(q(1, 2), Real::zero()),
                        p(1, 1),
                    ]
                } else {
                    vec![p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 1)]
                };
                let weights = vec![Real::one(); points.len()];
                let other = points
                    .iter()
                    .map(|point| {
                        Point2::new(Real::from(13) - point.y(), Real::from(21) + point.x())
                    })
                    .collect();
                let distance = Real::from(65).sqrt().unwrap();
                let first = RationalBezier2::try_new(points, weights.clone())
                    .unwrap()
                    .parallel_left(distance.clone())
                    .unwrap();
                let second = RationalBezier2::try_new(other, weights)
                    .unwrap()
                    .parallel_left(distance)
                    .unwrap();
                let range = if stationary_chart {
                    CurveParameterRange2::new_validated(q(3, 2).into(), q(5, 2).into())
                } else {
                    CurveParameterRange2::new_validated(Real::from(3).into(), Real::from(5).into())
                };
                let value = Real::from(if stationary_chart { 2 } else { 4 });
                let parameter: CurveParameter2 = value.clone().into();
                let center: CurvePoint2 = Point2::from_values(-4, 17).into();
                // Q(4)=B(2)=(4,16) for Q(t)=(t,t²), B(u)=Q(u²).
                // Its normal times sqrt(65) is (-8,1).
                // The second source is R90(B)+(13,21), so both parallels
                // pass through (-4,17). Their authored unit-span offset boxes
                // are disjoint: 1+sqrt(65) < 21-sqrt(65).
                for parallel in [&first, &second] {
                    let point = decided(
                        parallel
                            .point_evidence_on_regular_range(&parameter, &range, &policy)
                            .unwrap(),
                    );
                    assert_eq!(
                        point.same_point(&center, &policy),
                        Classification::Decided(true)
                    );
                }
                let result = decided(
                    first
                        .parallel_intersections_on_regular_ranges(&second, &range, &range, &policy)
                        .unwrap(),
                );
                assert!(result.is_complete());
                let mut found = false;
                for contact in result.contacts() {
                    found |= contact
                        .first_parameter()
                        .polynomial_sign(&[-value.clone(), Real::one()], &policy)
                        .unwrap()
                        == Classification::Decided(RealSign::Zero)
                        && contact
                            .second_parameter()
                            .polynomial_sign(&[-value.clone(), Real::one()], &policy)
                            .unwrap()
                            == Classification::Decided(RealSign::Zero);
                }
                assert!(
                    found,
                    "the independently known exterior contact was omitted"
                );
            }
        }
    }
    #[test]
    fn general_regular_pair_replays_unequal_contact_scale_changes() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let points = vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 6), Real::zero()),
                Point2::new(q(1, 2), Real::zero()),
                Point2::from_values(1, 1),
            ];
            let other = points
                .iter()
                .map(|point| Point2::new(q(-51, 64) - point.y(), q(-3, 64) + point.x()))
                .collect();
            let first = RationalBezier2::try_new(points, vec![Real::one(); 5])
                .unwrap()
                .parallel_left(q(15, 16))
                .unwrap();
            let second = RationalBezier2::try_new(other, vec![Real::one(); 5])
                .unwrap()
                .parallel_left(q(-15, 16))
                .unwrap();
            let range = CurveParameterRange2::unit();
            let parameter: CurveParameter2 = q(3, 8).sqrt().unwrap().into();
            let center: CurvePoint2 = Point2::new(q(-3, 16), q(57, 64)).into();
            // The first scale is 1/25 at the contact, but negative at the
            // range midpoint. The second scale is 49/25 at the contact
            // and positive everywhere. Neither carrier has a rational
            // parallel image; the general pair replay must keep this crossing.
            for parallel in [&first, &second] {
                let point = decided(
                    parallel
                        .point_evidence_on_regular_range(&parameter, &range, &policy)
                        .unwrap(),
                );
                assert_eq!(
                    point.same_point(&center, &policy),
                    Classification::Decided(true)
                );
            }
            let result = decided(
                first
                    .parallel_intersections_on_regular_ranges(&second, &range, &range, &policy)
                    .unwrap(),
            );
            assert!(result.is_complete());
            let mut found = false;
            for contact in result.contacts() {
                if [contact.first_parameter(), contact.second_parameter()]
                    .into_iter()
                    .all(|parameter| {
                        parameter
                            .polynomial_sign(
                                &[Real::from(-3), Real::zero(), Real::from(8)],
                                &policy,
                            )
                            .unwrap()
                            == Classification::Decided(RealSign::Zero)
                    })
                {
                    found = true;
                    assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Positive));
                    assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Zero));
                }
            }
            assert!(
                found,
                "the independently known general pair contact was omitted"
            );
        }
    }
    #[test]
    fn mixed_regular_pairs_keep_both_exterior_parameter_domains() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for stationary_chart in [false, true] {
                let points = if stationary_chart {
                    vec![
                        p(0, 0),
                        p(0, 0),
                        Point2::new(q(1, 6), Real::zero()),
                        Point2::new(q(1, 2), Real::zero()),
                        p(1, 1),
                    ]
                } else {
                    vec![p(0, 0), Point2::new(q(1, 2), Real::zero()), p(1, 1)]
                };
                let weights = vec![Real::one(); points.len()];
                let first = RationalBezier2::try_new(points, weights)
                    .unwrap()
                    .parallel_left(Real::from(65).sqrt().unwrap())
                    .unwrap();
                let first_range = if stationary_chart {
                    CurveParameterRange2::new_validated(q(3, 2).into(), q(5, 2).into())
                } else {
                    CurveParameterRange2::new_validated(Real::from(3).into(), Real::from(5).into())
                };
                let first_value = Real::from(if stationary_chart { 2 } else { 4 });
                let second_range = CurveParameterRange2::new_validated(
                    Real::from(16).into(),
                    Real::from(18).into(),
                );
                let center: CurvePoint2 = p(-4, 17).into();
                for distance in [0_i64, 1] {
                    let x = Real::from(-4 + distance);
                    let second = QuadraticBezier2::new(
                        Point2::new(x.clone(), Real::zero()),
                        Point2::new(x.clone(), q(1, 2)),
                        Point2::new(x, Real::one()),
                    )
                    .parallel_left(Real::from(distance))
                    .unwrap();
                    for (parallel, range, value) in [
                        (&first, &first_range, first_value.clone()),
                        (&second, &second_range, Real::from(17)),
                    ] {
                        let point = decided(
                            parallel
                                .point_evidence_on_regular_range(&value.into(), range, &policy)
                                .unwrap(),
                        );
                        assert_eq!(
                            point.same_point(&center, &policy),
                            Classification::Decided(true)
                        );
                    }
                    for swapped in [false, true] {
                        let (a, b, ar, br, av, bv) = if swapped {
                            (
                                &second,
                                &first,
                                &second_range,
                                &first_range,
                                Real::from(17),
                                first_value.clone(),
                            )
                        } else {
                            (
                                &first,
                                &second,
                                &first_range,
                                &second_range,
                                first_value.clone(),
                                Real::from(17),
                            )
                        };
                        let result = decided(
                            a.parallel_intersections_on_regular_ranges(b, ar, br, &policy)
                                .unwrap(),
                        );
                        eprintln!(
                            "stationary={stationary_chart} distance={distance} swapped={swapped} complete={} contacts={}",
                            result.is_complete(),
                            result.contacts().len()
                        );
                        assert!(result.is_complete());
                        assert_eq!(result.contacts().len(), 1);
                        let contact = &result.contacts()[0];
                        for (parameter, value) in [
                            (contact.first_parameter(), av),
                            (contact.second_parameter(), bv),
                        ] {
                            assert_eq!(
                                parameter
                                    .polynomial_sign(&[-value, Real::one()], &policy)
                                    .unwrap(),
                                Classification::Decided(RealSign::Zero)
                            );
                        }
                        assert_eq!(
                            contact.tangent_cross_sign(),
                            Some(if swapped {
                                RealSign::Negative
                            } else {
                                RealSign::Positive
                            })
                        );
                        assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Positive));
                    }
                }
            }
        }
    }
    #[test]
    fn rational_regular_pairs_use_the_requested_normal_sheet() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // x=(u-2)^2 has a downward left normal on its authored unit span,
            // but an upward one on [3,4]. Its retained offset meets x=2 at y=1.
            let first = QuadraticBezier2::new(p(4, 0), p(2, 0), p(1, 0))
                .parallel_left(Real::one())
                .unwrap();
            let second = QuadraticBezier2::new(p(2, 0), p(2, 1), p(2, 2))
                .parallel_left(Real::zero())
                .unwrap();
            let first_range =
                CurveParameterRange2::new_validated(Real::from(3).into(), Real::from(4).into());
            let second_range = CurveParameterRange2::unit();
            let first_value = Real::from(2) + Real::from(2).sqrt().unwrap();
            let second_value = q(1, 2);
            let center: CurvePoint2 = p(2, 1).into();
            for (parallel, range, value) in [
                (&first, &first_range, first_value.clone()),
                (&second, &second_range, second_value.clone()),
            ] {
                let point = decided(
                    parallel
                        .point_evidence_on_regular_range(&value.into(), range, &policy)
                        .unwrap(),
                );
                assert_eq!(
                    point.same_point(&center, &policy),
                    Classification::Decided(true)
                );
            }
            for swapped in [false, true] {
                let (a, b, ar, br, av, bv) = if swapped {
                    (
                        &second,
                        &first,
                        &second_range,
                        &first_range,
                        second_value.clone(),
                        first_value.clone(),
                    )
                } else {
                    (
                        &first,
                        &second,
                        &first_range,
                        &second_range,
                        first_value.clone(),
                        second_value.clone(),
                    )
                };
                let result = decided(
                    a.parallel_intersections_on_regular_ranges(b, ar, br, &policy)
                        .unwrap(),
                );
                eprintln!(
                    "rational normal sheet swapped={swapped} complete={} contacts={}",
                    result.is_complete(),
                    result.contacts().len()
                );
                assert!(result.is_complete());
                assert_eq!(result.contacts().len(), 1);
                let contact = &result.contacts()[0];
                for (parameter, value) in [
                    (contact.first_parameter(), av),
                    (contact.second_parameter(), bv),
                ] {
                    assert_eq!(
                        parameter
                            .polynomial_sign(&[-value, Real::one()], &policy)
                            .unwrap(),
                        Classification::Decided(RealSign::Zero)
                    );
                }
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if swapped {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    })
                );
                assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Zero));
            }
        }
    }
    #[test]
    fn native_rational_pair_evidence_clips_both_retained_domains() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for distance in [0_i64, 1] {
                let line = QuadraticBezier2::new(
                    p(0, -distance),
                    Point2::new(q(1, 2), Real::from(-distance)),
                    p(1, -distance),
                )
                .parallel_left(Real::from(distance))
                .unwrap();
                // The offset line is y=0. Q(v)=(v,(v-1/4)(v-3/4))
                // has exactly the two native contacts (u,v)=(1/4,1/4),(3/4,3/4).
                let curve = QuadraticBezier2::new(
                    Point2::new(Real::zero(), q(3, 16)),
                    Point2::new(q(1, 2), q(-5, 16)),
                    Point2::new(Real::one(), q(3, 16)),
                )
                .parallel_left(Real::zero())
                .unwrap();
                let range =
                    |a: Real, b: Real| CurveParameterRange2::new_validated(a.into(), b.into());
                for (first, second, expected) in [
                    (
                        range(Real::zero(), q(1, 2)),
                        CurveParameterRange2::unit(),
                        Some(q(1, 4)),
                    ),
                    (
                        CurveParameterRange2::unit(),
                        range(q(1, 2), Real::one()),
                        Some(q(3, 4)),
                    ),
                    (
                        range(Real::zero(), q(1, 2)),
                        range(q(1, 2), Real::one()),
                        None,
                    ),
                    (
                        range(Real::one(), q(1, 2)),
                        range(Real::one(), Real::zero()),
                        Some(q(3, 4)),
                    ),
                ] {
                    for swapped in [false, true] {
                        let (a, b, ar, br) = if swapped {
                            (&curve, &line, &second, &first)
                        } else {
                            (&line, &curve, &first, &second)
                        };
                        let result = decided(
                            a.parallel_intersections_on_regular_ranges(b, ar, br, &policy)
                                .unwrap(),
                        );
                        assert!(result.is_complete());
                        assert_eq!(result.contacts().len(), usize::from(expected.is_some()));
                        assert!(result.overlaps().is_empty());
                        assert!(result.parameter_components().is_empty());
                        if let Some(value) = &expected {
                            let contact = &result.contacts()[0];
                            for parameter in [contact.first_parameter(), contact.second_parameter()]
                            {
                                assert_eq!(
                                    parameter
                                        .polynomial_sign(&[-value, Real::one()], &policy)
                                        .unwrap(),
                                    Classification::Decided(RealSign::Zero)
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn retained_native_overlap_correspondences_clip_as_exact_curve_results() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // The first source offsets to (u,0); the second is (v^2,0).
            // Correlated clipping must respect u=v^2, not intersect the two
            // parameter intervals as if they used the same chart.
            let first =
                QuadraticBezier2::new(p(0, -1), Point2::new(q(1, 2), (-1).into()), p(1, -1))
                    .parallel_left(Real::one())
                    .unwrap();
            let second = QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))
                .parallel_left(Real::zero())
                .unwrap();
            for (first_end, second_start, expected_overlaps, expected_contacts) in [
                (q(3, 4), q(1, 2), 1, 0),
                (q(1, 4), q(3, 4), 0, 0),
                (q(1, 4), q(1, 2), 0, 1),
            ] {
                for reversed in [false, true] {
                    let retained = |parallel: BezierParallel2, start: Real, end: Real| {
                        let range = if reversed {
                            BezierParameterRange2::from_exact(end, start)
                        } else {
                            BezierParameterRange2::from_exact(start, end)
                        };
                        crate::Curve2::from_retained_fragment(
                            crate::BezierSplitFragment2::AnalyticParallel(decided(
                                crate::BezierParallelFragment2::try_new(parallel, range, &policy)
                                    .unwrap(),
                            )),
                        )
                    };
                    let a = retained(first.clone(), Real::zero(), first_end.clone());
                    let b = retained(second.clone(), second_start.clone(), Real::one());
                    for swapped in [false, true] {
                        let (a, b) = if swapped { (&b, &a) } else { (&a, &b) };
                        let result = a
                            .intersect_curve(b, &policy)
                            .expect("retained native overlap must publish exact clipping");
                        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                        let result = result.value;
                        assert!(result.is_complete());
                        assert_eq!(result.overlaps().len(), expected_overlaps);
                        assert_eq!(result.contacts().len(), expected_contacts);
                        assert!(result.parameter_components().is_empty());
                        for overlap in result.overlaps() {
                            let (first, second) = if swapped {
                                (overlap.second_range(), overlap.first_range())
                            } else {
                                (overlap.first_range(), overlap.second_range())
                            };
                            let first = decided(first.ordered_endpoints(&policy).unwrap());
                            let second = decided(second.ordered_endpoints(&policy).unwrap());
                            for ((u, v), x) in first.into_iter().zip(second).zip([q(1, 4), q(3, 4)])
                            {
                                assert_eq!(
                                    u.polynomial_sign(&[-x.clone(), Real::one()], &policy)
                                        .unwrap(),
                                    Classification::Decided(RealSign::Zero)
                                );
                                assert_eq!(
                                    v.polynomial_sign(&[-x, Real::zero(), Real::one()], &policy)
                                        .unwrap(),
                                    Classification::Decided(RealSign::Zero)
                                );
                            }
                            assert!(overlap.includes_start() && overlap.includes_end());
                        }
                        for contact in result.contacts() {
                            let (u, v) = if swapped {
                                (
                                    contact.second().local_parameter(),
                                    contact.first().local_parameter(),
                                )
                            } else {
                                (
                                    contact.first().local_parameter(),
                                    contact.second().local_parameter(),
                                )
                            };
                            assert_eq!(
                                u.polynomial_sign(&[-q(1, 4), Real::one()], &policy)
                                    .unwrap(),
                                Classification::Decided(RealSign::Zero)
                            );
                            assert_eq!(
                                v.polynomial_sign(&[-q(1, 2), Real::one()], &policy)
                                    .unwrap(),
                                Classification::Decided(RealSign::Zero)
                            );
                        }
                    }
                }
            }
        }
    }
    fn retained_frame_test_parameters(value: Real, policy: &CurveContext) -> [CurveParameter2; 3] {
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![-q(1, 2), Real::zero(), Real::one()],
                policy,
            )
            .unwrap(),
        );
        let interval =
            decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
        let alpha =
            decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
        let selected =
            BezierAlgebraicSelectedFiberAuthority2::exact_parameter(alpha, value.clone(), policy);
        let one = DenseTensorPolynomial::try_new(vec![], vec![Real::one()]).unwrap();
        let field = BezierRecursiveQuadraticField2::base(vec![], one.clone(), one).unwrap();
        let recursive = decided(
            BezierRecursiveProjectiveParameter2::new_with_certified_bounds(
                BezierRecursiveQuadraticProjectiveScalar2 {
                    numerator: field.constant(value.clone()).unwrap(),
                    denominator: field.constant(Real::one()).unwrap(),
                },
                Some((Real::zero(), Real::one())),
                policy,
            )
            .unwrap(),
        );
        [
            value.into(),
            CurveParameter2::from_selected_fiber(selected),
            CurveParameter2::from_recursive_projective(recursive),
        ]
    }

    #[test]
    fn regular_source_frames_accept_every_retained_parameter_authority() {
        let source = CubicBezier2::new(
            p(0, 0),
            p(0, 0),
            Point2::new(q(1, 3), Real::zero()),
            p(1, 1),
        );
        let anchor = source.parallel_left(Real::one()).unwrap();
        let parallel = anchor.with_distance(Real::from(2));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for parameter in retained_frame_test_parameters(Real::zero(), &policy) {
                for (range, normal) in [
                    (CurveParameterRange2::unit(), 1_i64),
                    (
                        CurveParameterRange2::new_validated(
                            (-Real::one()).into(),
                            Real::zero().into(),
                        ),
                        -1,
                    ),
                ] {
                    for direction in [RealSign::Positive, RealSign::Negative] {
                        let (point, tangent) = decided(
                            parallel
                                .regular_source_point_and_tangent_support(
                                    &anchor, &parameter, &range, direction, &policy,
                                )
                                .unwrap(),
                        );
                        let dx = normal
                            * if direction == RealSign::Positive {
                                1
                            } else {
                                -1
                            };
                        for (actual, expected) in [
                            (&point, p(0, 2 * normal)),
                            (tangent.start(), p(0, normal)),
                            (tangent.end(), p(dx, normal)),
                        ] {
                            assert_eq!(
                                actual.same_point(&expected.into(), &policy),
                                Classification::Decided(true)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn regular_source_frames_retain_unprojectable_selected_parameters() {
        let source = CubicBezier2::new(
            p(0, 0),
            p(0, 0),
            Point2::new(q(1, 3), Real::zero()),
            p(1, 1),
        );
        let anchor = source.parallel_left(Real::one()).unwrap();
        let parallel = anchor.with_distance(Real::from(2));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = degree_nine_selected_fiber_parameter_for_test(q(1, 2), 32768, &policy);
            assert!(selected.data.representations.bezier.get().is_none());
            let parameter = CurveParameter2::from_selected_fiber(selected.clone());
            let (point, tangent) = decided(
                parallel
                    .regular_source_point_and_tangent_support(
                        &anchor,
                        &parameter,
                        &CurveParameterRange2::unit(),
                        RealSign::Positive,
                        &policy,
                    )
                    .unwrap(),
            );
            // At positive t, the raw tangent t(2,3t) and cancelled field
            // (2,3t) define the same unit normal and tangent. These independent
            // procedural expressions must compare without a global eliminant.
            for (actual, support, displacement) in [
                (&point, &parallel, Real::zero()),
                (tangent.start(), &anchor, Real::zero()),
                (tangent.end(), &anchor, Real::one()),
            ] {
                let expected =
                    BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                        support.clone(),
                        &parameter,
                        displacement,
                        &policy,
                    )
                    .unwrap();
                assert_eq!(
                    actual.same_point(&CurvePoint2::from(expected), &policy),
                    Classification::Decided(true)
                );
            }
            // Two displaced tangent witnesses share the selected scalar,
            // but have different anchors and opposite traversal. Their
            // relation must reuse the polynomial directions without a global
            // parameter image or separately reconstructed unit vectors.
            let other_anchor = anchor.with_distance(q(3, 7));
            let (_, opposite) = decided(
                other_anchor
                    .regular_source_point_and_tangent_support(
                        &other_anchor,
                        &parameter,
                        &CurveParameterRange2::unit(),
                        RealSign::Negative,
                        &policy,
                    )
                    .unwrap(),
            );
            assert_eq!(
                tangent.tangent_cross_sign(&opposite, &policy).unwrap(),
                Classification::Decided(RealSign::Zero)
            );
            assert_eq!(
                tangent.tangent_dot_sign(&opposite, &policy).unwrap(),
                Classification::Decided(RealSign::Negative)
            );
            assert!(selected.data.representations.bezier.get().is_none());
        }
    }

    #[test]
    fn regular_source_frames_reject_poles_in_every_parameter_authority() {
        let source =
            RationalBezier2::try_new(vec![p(0, 0), p(1, 0)], vec![-Real::one(), Real::one()])
                .unwrap();
        let parallel = source.parallel_left(Real::one()).unwrap();
        let range = CurveParameterRange2::new_validated(q(1, 2).into(), Real::one().into());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for parameter in retained_frame_test_parameters(q(1, 2), &policy) {
                assert!(matches!(
                    parallel
                        .regular_source_point_and_tangent_support(
                            &parallel,
                            &parameter,
                            &range,
                            RealSign::Positive,
                            &policy
                        )
                        .unwrap(),
                    Classification::Uncertain(UncertaintyReason::Boundary)
                ));
            }
        }
    }

    #[test]
    fn rational_pair_endpoint_tangents_preserve_both_operand_roles() {
        let cusp = CubicBezier2::new(
            p(0, 0),
            p(0, 0),
            Point2::new(q(1, 3), Real::zero()),
            p(1, 1),
        )
        .parallel_left(Real::zero())
        .unwrap();
        let line = QuadraticBezier2::new(p(0, 0), Point2::new(Real::zero(), q(1, 2)), p(0, 1))
            .parallel_left(Real::zero())
            .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed_range in [false, true] {
                let range = if reversed_range {
                    CurveParameterRange2::new_validated(Real::one().into(), Real::zero().into())
                } else {
                    CurveParameterRange2::unit()
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&line, &cusp)
                    } else {
                        (&cusp, &line)
                    };
                    let result = decided(
                        first
                            .parallel_intersections_on_regular_ranges(
                                second, &range, &range, &policy,
                            )
                            .unwrap(),
                    );
                    assert!(result.is_complete());
                    assert_eq!(result.contacts().len(), 1);
                    let contact = &result.contacts()[0];
                    for parameter in [contact.first_parameter(), contact.second_parameter()] {
                        assert_eq!(
                            parameter
                                .polynomial_sign(&[Real::zero(), Real::one()], &policy)
                                .unwrap(),
                            Classification::Decided(RealSign::Zero)
                        );
                    }
                    // C(t)=(t^2,t^3) owns tangent +x at t=0; the line
                    // owns +y. Reversing a query range keeps source-parameter
                    // orientation; swapping operands reverses only the cross.
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(if swapped {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        })
                    );
                    assert_eq!(contact.tangent_dot_sign(), Some(RealSign::Zero));
                    assert!(contact.is_certified_transverse());
                }
            }
        }
    }
}

mod empty_incident_component_tests {
    use super::*;

    fn decided<T>(value: CurveResult<Classification<T>>) -> T {
        match value {
            Ok(Classification::Decided(value)) => value,
            Ok(Classification::Uncertain(reason)) => {
                panic!("empty ray query uncertain: {reason:?}")
            }
            Err(error) => panic!("empty ray query failed: {error}"),
        }
    }

    #[test]
    fn empty_incident_charts_preserve_finite_components_and_constraints() {
        use CurveParameterComponentSelection2::{NeedsConstraint, Selected};
        let diagonal =
            BivariatePolynomial::new(vec![vec![Real::zero(), -Real::one()], vec![Real::one()]]);
        let positive = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let selector = ParameterComponentSelector2::Positive(&positive, None);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for shift in [-2_i32, 0, 2] {
                let lower = Real::from(shift);
                let upper = Real::from(shift + 1);
                let finite =
                    CurveParameterRange2::new_validated(lower.clone().into(), upper.clone().into());
                for increasing in [false, true] {
                    let anchor = if increasing { &upper } else { &lower };
                    let direction = if increasing {
                        BezierParameterRayDirection2::Increasing
                    } else {
                        BezierParameterRayDirection2::Decreasing
                    };
                    for blocked_before_anchor in [false, true] {
                        let barrier = BezierParameter2::Exact(if blocked_before_anchor {
                            anchor + Real::from(if increasing { -1_i32 } else { 1 })
                        } else {
                            anchor.clone()
                        });
                        let ray = BezierParameterRay2 {
                            anchor,
                            direction,
                            barrier: Some(&barrier),
                        };
                        for axes in 0..4 {
                            let domains = [0, 1].map(|axis| {
                                CurveParameterDomain2::new(
                                    &finite,
                                    (axes & (1 << axis) != 0).then_some(ray),
                                )
                            });
                            let result = decided(select_parameter_component_in_domain(
                                &diagonal,
                                &selector,
                                domains,
                                ParameterComponentQuery2::AllComponents(None),
                                &policy,
                                config,
                            ));
                            assert_eq!(
                                result.components.len(),
                                1,
                                "shift {shift}, increasing {increasing}, before {blocked_before_anchor}, axes {axes}"
                            );
                            assert!(result.selected_pairs.is_empty());
                            let component = &result.components[0];
                            assert!(matches!(
                                decided(component.constrain([None, None], &policy)),
                                NeedsConstraint
                            ));
                            // Either exact contact fixes the retained diagonal family,
                            // including finite endpoints that an empty ray must not revoke.
                            for value in [
                                lower.clone(),
                                ((&lower + &upper) / Real::from(2_i8)).unwrap(),
                                upper.clone(),
                            ] {
                                let parameter = CurveParameter2::from(value);
                                for constrained_axis in 0..2 {
                                    let constraints = [0, 1].map(|axis| {
                                        (axis == constrained_axis).then_some(&parameter)
                                    });
                                    let Selected(pair) =
                                        decided(component.constrain(constraints, &policy))
                                    else {
                                        panic!("one exact contact must select the finite family");
                                    };
                                    for actual in pair {
                                        assert!(decided(actual.same_value(&parameter, &policy)));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // The barrier and anchor have distinct exact representations of sqrt(2).
    // Empty-ray classification must reuse the isolated root's certificate.
    #[test]
    fn empty_incident_chart_compares_an_algebraic_barrier_exactly() {
        use CurveParameterComponentSelection2::{NeedsConstraint, Selected};
        let diagonal =
            BivariatePolynomial::new(vec![vec![Real::zero(), -Real::one()], vec![Real::one()]]);
        let positive = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-2_i8), Real::zero(), Real::one()],
                &policy,
            ));
            let interval = decided(BezierParameterInterval::try_new(
                Real::one(),
                Real::from(2_i8),
                &policy,
            ));
            let barrier = BezierParameter2::Algebraic(decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy),
            ));
            let anchor = Real::from(2_i8).sqrt().unwrap();
            let parameter = CurveParameter2::from(barrier.clone());
            let finite =
                CurveParameterRange2::new_validated(Real::one().into(), Real::from(2_i8).into());
            for direction in [
                BezierParameterRayDirection2::Increasing,
                BezierParameterRayDirection2::Decreasing,
            ] {
                let ray = BezierParameterRay2 {
                    anchor: &anchor,
                    direction,
                    barrier: Some(&barrier),
                };
                let result = decided(select_parameter_component_in_domain(
                    &diagonal,
                    &ParameterComponentSelector2::Positive(&positive, None),
                    [CurveParameterDomain2::new(&finite, Some(ray)); 2],
                    ParameterComponentQuery2::AllComponents(None),
                    &policy,
                    config,
                ));
                assert_eq!(result.components.len(), 1);
                let component = &result.components[0];
                assert!(matches!(
                    decided(component.constrain([None, None], &policy)),
                    NeedsConstraint
                ));
                let Selected(pair) =
                    decided(component.constrain([Some(&parameter), None], &policy))
                else {
                    panic!("an exact algebraic contact must select the finite component");
                };
                for actual in pair {
                    assert!(decided(actual.same_value(&parameter, &policy)));
                }
            }
        }
    }
}

mod finite_domain_ownership_tests {
    use super::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: CurveResult<Classification<T>>) -> T {
        match value.unwrap() {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("domain ownership was uncertain: {reason:?}")
            }
        }
    }

    #[test]
    fn finite_endpoint_ownership_filters_independent_roots_and_selected_axes() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let root_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-2), Real::zero(), Real::one()],
                &policy,
            ));
            let interval = decided(BezierParameterInterval::try_new(
                Real::one(),
                q(3, 2),
                &policy,
            ));
            let lower = BezierParameter2::Algebraic(decided(
                BezierAlgebraicParameter2::try_isolate(root_polynomial, interval, &policy),
            ));
            // A different degree-four root authority must compare equal to the
            // retained sqrt(2) endpoint without replacing either certificate.
            let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
                polynomial_multiply(
                    &[Real::from(-2), Real::zero(), Real::one()],
                    &polynomial_multiply(
                        &[Real::from(-3), Real::from(2)],
                        &[Real::from(-2), Real::one()],
                    ),
                ),
                &policy,
            ));
            let expected = [
                lower,
                BezierParameter2::Exact(q(3, 2)),
                BezierParameter2::Exact(Real::from(2)),
            ];
            for reversed in [false, true] {
                let ends = if reversed { [2, 0] } else { [0, 2] };
                let range = CurveParameterRange2::new_validated(
                    expected[ends[0]].clone().into(),
                    expected[ends[1]].clone().into(),
                );
                for inclusion in [[true, true], [true, false], [false, true], [false, false]] {
                    let domain =
                        CurveParameterDomain2::new(&range, None).with_finite_inclusion(inclusion);
                    assert_eq!(
                        decided(domain.contains_finite_range(&range, &policy)),
                        inclusion == [true; 2]
                    );
                    let roots = decided(domain.finite_roots(&polynomial, &policy));
                    let selected = decided(selected_axis_parameters_in_domain(
                        domain,
                        &policy,
                        |axis| {
                            assert!(matches!(axis, SelectedThirdAxisDomain2::Finite(_)));
                            Ok(Classification::Decided(
                                BezierAlgebraicFiberProjection2::Parameters(expected.to_vec()),
                            ))
                        },
                    ));
                    let BezierAlgebraicFiberProjection2::Parameters(selected) = selected else {
                        panic!("three certified finite roots have a discrete projection");
                    };
                    for actual in [roots, selected] {
                        let owned = [inclusion[0], true, inclusion[1]];
                        assert_eq!(
                            actual.len(),
                            owned.into_iter().filter(|included| *included).count()
                        );
                        for (expected, included) in expected.iter().zip(owned) {
                            let matches = actual
                                .iter()
                                .filter(|root| {
                                    decided(root.cmp_by_refinement(expected, &policy)).is_eq()
                                })
                                .count();
                            assert_eq!(matches, usize::from(included));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn finite_and_incident_components_partition_open_endpoints_once() {
        let diagonal =
            BivariatePolynomial::new(vec![vec![Real::zero(), -Real::one()], vec![Real::one()]]);
        let positive = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for increasing in [false, true] {
                let sign = if increasing { 1 } else { -1 };
                for anchor_at_end in [false, true] {
                    let finite = CurveParameterRange2::new_validated(
                        Real::from(if anchor_at_end { 0 } else { 2 * sign }).into(),
                        Real::from(4 * sign).into(),
                    );
                    let other = CurveParameterRange2::new_validated(
                        Real::zero().into(),
                        Real::from(6 * sign).into(),
                    );
                    let anchor = Real::from(if anchor_at_end { 4 * sign } else { 0 });
                    let barrier = BezierParameter2::Exact(Real::from(6 * sign));
                    let ray = BezierParameterRay2 {
                        anchor: &anchor,
                        direction: if increasing {
                            BezierParameterRayDirection2::Increasing
                        } else {
                            BezierParameterRayDirection2::Decreasing
                        },
                        barrier: Some(&barrier),
                    };
                    for inclusion in [[true, true], [true, false], [false, true], [false, false]] {
                        let domain = CurveParameterDomain2::new(&finite, Some(ray))
                            .with_finite_inclusion(inclusion);
                        for swapped in [false, true] {
                            let domains = if swapped {
                                [CurveParameterDomain2::new(&other, None), domain]
                            } else {
                                [domain, CurveParameterDomain2::new(&other, None)]
                            };
                            let selected = decided(select_parameter_component_in_domain(
                                &diagonal,
                                &ParameterComponentSelector2::Positive(&positive, None),
                                domains,
                                ParameterComponentQuery2::AllComponents(None),
                                &policy,
                                config,
                            ));
                            for value in [0, 1, 2, 3, 4, 5, 6] {
                                let parameter = CurveParameter2::from(Real::from(value * sign));
                                let count = selected
                                    .components
                                    .iter()
                                    .filter(|component| {
                                        decided(
                                            component
                                                .contains_pair(&parameter, &parameter, &policy),
                                        )
                                    })
                                    .count();
                                let included = if value == 6 {
                                    false
                                } else if anchor_at_end && (value == 0 || value == 4) {
                                    inclusion[usize::from((value == 4) == increasing)]
                                } else {
                                    value != 0
                                };
                                assert_eq!(
                                    count,
                                    usize::from(included),
                                    "value {value}, increasing {increasing}, anchored {anchor_at_end}, inclusion {inclusion:?}, swapped {swapped}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn rational_stationary_components_keep_opposite_source_charts_and_ownership() {
        use CurveParameterComponentSelection2::{Empty, NeedsConstraint, Selected};
        // Offsets of (u²,0) and ((1-v)²,0) agree at (u²,-1) on
        // the selected sides of their stationary ends, with v=1-u.
        let first = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
        )
        .parallel_left(-Real::one())
        .unwrap();
        let second = QuadraticBezier2::new(
            Point2::from_values(1, 0),
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
        )
        .parallel_left(Real::one())
        .unwrap();
        let first_range = CurveParameterRange2::new_validated(Real::zero().into(), q(1, 2).into());
        let second_range = CurveParameterRange2::new_validated(q(1, 2).into(), Real::one().into());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for first_owned in [false, true] {
                for second_owned in [false, true] {
                    let domains = [
                        CurveParameterDomain2::new(&first_range, None)
                            .with_finite_inclusion([first_owned, true]),
                        CurveParameterDomain2::new(&second_range, None)
                            .with_finite_inclusion([true, second_owned]),
                    ];
                    for swapped in [false, true] {
                        let (a, b, domains) = if swapped {
                            (&second, &first, [domains[1], domains[0]])
                        } else {
                            (&first, &second, domains)
                        };
                        for query in [
                            ParameterComponentQuery2::AllComponents(None),
                            ParameterComponentQuery2::FirstComponent(None),
                        ] {
                            let result = decided(
                                a.parallel_intersections_in_domain(b, domains, query, &policy),
                            );
                            assert!(result.intersections.is_complete());
                            assert_eq!(result.components.len(), 1);
                            let component = &result.components[0];
                            assert!(matches!(
                                decided(component.constrain([None, None], &policy)),
                                NeedsConstraint
                            ));
                            for value in [Real::zero(), q(1, 4), q(1, 2)] {
                                let owned = value != Real::zero() || (first_owned && second_owned);
                                let pair = [
                                    CurveParameter2::from(value.clone()),
                                    CurveParameter2::from(Real::one() - value),
                                ];
                                let pair = if swapped {
                                    [pair[1].clone(), pair[0].clone()]
                                } else {
                                    pair
                                };
                                for axis in 0..2 {
                                    let selection = decided(component.constrain(
                                        [0, 1].map(|index| (index == axis).then_some(&pair[index])),
                                        &policy,
                                    ));
                                    if !owned {
                                        assert!(matches!(selection, Empty));
                                        continue;
                                    }
                                    let Selected(actual) = selection else {
                                        panic!(
                                            "an owned PH contact must select the exact correspondence"
                                        );
                                    };
                                    for (actual, expected) in actual.iter().zip(&pair) {
                                        assert!(decided(actual.same_value(expected, &policy)));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

mod retained_structural_pair_domain_regression {
    use super::*;

    #[test]
    fn retained_structural_correspondence_preserves_off_diagonal_contact_domains() {
        let point = |x, y| Point2::from_values(x, y);
        let source = CubicBezier2::new(point(0, 0), point(1, 4), point(3, -4), point(4, 0));
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = source.parallel_left(half.clone()).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(reference) =
                parallel.parallel_intersections(&parallel, &policy).unwrap()
            else {
                panic!("the complete native structural pair must be certified");
            };
            assert!(reference.is_complete());
            assert_eq!(reference.contacts().len(), 2);
            let contact = &reference.contacts()[0];
            let first = contact.first_parameter();
            let second = contact.second_parameter();
            let (lower, upper) = match first.cmp_by_refinement(second, &policy).unwrap() {
                Classification::Decided(std::cmp::Ordering::Less) => (first, second),
                Classification::Decided(std::cmp::Ordering::Greater) => (second, first),
                _ => panic!("the reference contact must be off the diagonal"),
            };
            let Classification::Decided(split) =
                lower.strict_scalar_between_ordered(upper, &policy).unwrap()
            else {
                panic!("distinct crossing parameters must admit a separating cut");
            };
            let ranges = [
                CurveParameterRange2::new_validated(Real::zero().into(), split.clone().into()),
                CurveParameterRange2::new_validated(split.into(), Real::one().into()),
            ];
            for swap in [false, true] {
                let requested = if swap {
                    [&ranges[1], &ranges[0]]
                } else {
                    [&ranges[0], &ranges[1]]
                };
                let domains = requested.map(|range| CurveParameterDomain2::new(range, None));
                let mut expected = Vec::new();
                for contact in reference.contacts() {
                    let parameters = [contact.first_parameter(), contact.second_parameter()];
                    if domains.iter().zip(parameters).all(|(domain, parameter)| {
                        domain
                            .contains_finite_parameter(parameter, &policy)
                            .unwrap()
                            == Classification::Decided(true)
                    }) {
                        expected.push(contact);
                    }
                }
                assert_eq!(
                    expected.len(),
                    1,
                    "the separated domains contain one off-diagonal crossing"
                );
                let Classification::Decided(actual) = parallel
                    .parallel_intersections_on_regular_ranges(
                        &parallel,
                        requested[0],
                        requested[1],
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("retained structural pair replay must classify");
                };
                assert!(actual.is_complete());
                assert_eq!(actual.contacts().len(), expected.len());
                for (got, wanted) in actual.contacts().iter().zip(expected) {
                    for (got, wanted) in [
                        (got.first_parameter(), wanted.first_parameter()),
                        (got.second_parameter(), wanted.second_parameter()),
                    ] {
                        assert_eq!(
                            got.same_value(wanted, &policy).unwrap(),
                            Classification::Decided(true)
                        );
                    }
                }
            }
        }
    }
}

mod incident_offset_cusp_side_regression {
    use super::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn cusp_parallel() -> BezierParallel2 {
        CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(1, 3), Real::zero()),
            Point2::new(q(2, 3), q(1, 3)),
            Point2::new(q(2, 3), Real::one()),
        )
        .parallel_left(Real::from(2))
        .unwrap()
    }

    #[test]
    fn incident_cusp_normal_side_survives_parameter_chart_reversal() {
        let parallel = cusp_parallel();
        let unit = CurveParameterRange2::unit();
        let domain = CurveParameterDomain2::new(&unit, None);
        let range = std::cell::OnceCell::new();
        let identity = ParameterComponentChart2 {
            domain,
            mapping: ParameterComponentMap2::Identity,
            range: &range,
        };
        let negative = ParameterComponentAffineMap2 {
            scale: Real::from(-2),
            offset: Real::from(2),
        };
        let zero = Real::zero();
        let two = Real::from(2);
        let increasing = BezierParameterRay2 {
            anchor: &zero,
            direction: BezierParameterRayDirection2::Increasing,
            barrier: None,
        };
        let decreasing = BezierParameterRay2 {
            anchor: &two,
            direction: BezierParameterRayDirection2::Decreasing,
            barrier: None,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert_eq!(
                parallel
                    .parallel_derivative_scale_sign(&Real::one().into(), &policy)
                    .unwrap(),
                Classification::Decided(RealSign::Zero)
            );
            for (side, expected) in [
                (BezierParameterRayDirection2::Decreasing, RealSign::Negative),
                (BezierParameterRayDirection2::Increasing, RealSign::Positive),
            ] {
                assert_eq!(
                    parallel
                        .parallel_derivative_scale_sign_at_side(&Real::one().into(), side, &policy)
                        .unwrap(),
                    Classification::Decided(expected)
                );
                for axis in [
                    CurveResultantParameter::First,
                    CurveResultantParameter::Second,
                ] {
                    for (mapping, contact) in [
                        (ParameterComponentMap2::Identity, Real::one()),
                        (ParameterComponentMap2::Affine(&negative), q(1, 2)),
                        (ParameterComponentMap2::Incident(increasing), q(1, 2)),
                        (ParameterComponentMap2::Incident(decreasing), q(1, 2)),
                    ] {
                        let chart = ParameterComponentChart2 {
                            mapping,
                            ..identity
                        };
                        let charts = match axis {
                            CurveResultantParameter::First => [chart, identity],
                            CurveResultantParameter::Second => [identity, chart],
                        };
                        let contact = BezierParameter2::Exact(contact);
                        let other = BezierParameter2::Exact(q(1, 3));
                        let parameters = match axis {
                            CurveResultantParameter::First => [&contact, &other],
                            CurveResultantParameter::Second => [&other, &contact],
                        };
                        for selected in [false, true] {
                            let sign = if selected {
                                expected
                            } else {
                                product_sign(expected, RealSign::Negative)
                            };
                            let constraint =
                                parallel.derivative_scale_constraint(axis, sign, Some(side));
                            let transformed = constraint
                                .polynomials()
                                .unwrap()
                                .in_charts(charts[0], charts[1])
                                .unwrap();
                            assert_eq!(
                                transformed
                                    .selected_at(parameters[0], parameters[1], &policy)
                                    .unwrap(),
                                Classification::Decided(selected)
                            );
                        }
                        let pointwise = parallel.derivative_scale_constraint(axis, expected, None);
                        let transformed = pointwise
                            .polynomials()
                            .unwrap()
                            .in_charts(charts[0], charts[1])
                            .unwrap();
                        assert_eq!(
                            transformed
                                .selected_at(parameters[0], parameters[1], &policy)
                                .unwrap(),
                            Classification::Decided(false)
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn incident_cusp_side_does_not_create_undefined_or_collapsed_tangents() {
        let stationary = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
        )
        .parallel_left(Real::one())
        .unwrap();
        let pole = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 1),
                Point2::from_values(2, 0),
            ],
            vec![4.into(), 2.into(), 1.into()],
        )
        .unwrap()
        .parallel_left(Real::one())
        .unwrap();
        // P(t)=((1-t²)/(1+t²),2t/(1+t²)); its unit left parallel
        // is the circle center for every parameter, not a tangent branch.
        let collapsed = RationalBezier2::try_new(
            vec![
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
                Point2::from_values(0, 1),
            ],
            vec![1.into(), 1.into(), 2.into()],
        )
        .unwrap()
        .parallel_left(Real::one())
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for side in [
                BezierParameterRayDirection2::Decreasing,
                BezierParameterRayDirection2::Increasing,
            ] {
                for (curve, parameter) in [(&stationary, Real::zero()), (&pole, Real::from(2))] {
                    assert!(matches!(
                        curve
                            .parallel_derivative_scale_sign_at_side(
                                &parameter.into(),
                                side,
                                &policy
                            )
                            .unwrap(),
                        Classification::Uncertain(UncertaintyReason::Boundary)
                    ));
                }
                assert_eq!(
                    collapsed
                        .parallel_derivative_scale_sign_at_side(&q(1, 2).into(), side, &policy)
                        .unwrap(),
                    Classification::Decided(RealSign::Zero)
                );
            }
        }
    }
}

mod structural_overlap_trace_regression {
    use super::*;

    fn reparameterized_offset(policy: CurveContext, reversed: bool) {
        let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
        let source = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 4),
            Point2::from_values(3, -4),
            Point2::from_values(4, 0),
        );
        let half = q(1, 2);
        let reference = source.parallel_left(half.clone()).unwrap();
        let Classification::Decided(reference_contacts) = reference
            .parallel_intersections(&reference, &policy)
            .unwrap()
        else {
            panic!("the independently qualified S-cubic must classify")
        };
        assert!(reference_contacts.is_complete());
        assert_eq!(reference_contacts.contacts().len(), 2);

        // Exact Bernstein coefficients of C((t+t^2)/2), derived with rational
        // polynomial composition. The chart maps 0 -> 0 and 1 -> 1, and its
        // derivative (1+2t)/2 is strictly positive throughout the unit span.
        // It preserves the source trace, selected left normal, and all offset
        // crossings without introducing stationary source points or poles.
        let controls = vec![
            Point2::from_values(0, 0),
            Point2::new(q(1, 4), Real::one()),
            Point2::new(q(13, 20), q(9, 5)),
            Point2::new(q(101, 80), q(33, 20)),
            Point2::new(q(43, 20), q(-1, 5)),
            Point2::new(q(13, 4), Real::from(-3)),
            Point2::from_values(4, 0),
        ];
        let reparameterized = RationalBezier2::try_new(controls, vec![Real::one(); 7]).unwrap();
        let parallel = reparameterized.parallel_left(half.clone()).unwrap();
        let other = if reversed {
            reparameterized.reversed().parallel_left(-half).unwrap()
        } else {
            parallel.clone()
        };
        let actual = parallel.parallel_intersections(&other, &policy).unwrap();
        let Classification::Decided(actual) = actual else {
            panic!("the same finite offset trace must remain representable and decidable");
        };
        assert_eq!(
            actual.overlaps().len(),
            1,
            "one closed correspondence covers the trace"
        );
        for component in actual.component_overlaps() {
            let overlap = component.overlap();
            let public = crate::CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                first_range: CurveParameterRange2::from_bezier_range(overlap.first_range().clone()),
                second_range: CurveParameterRange2::from_bezier_range(
                    overlap.second_range().clone(),
                ),
                orientation: overlap.orientation(),
                endpoint_inclusion: [overlap.includes_start(), overlap.includes_end()],
                parameter_correspondence:
                    crate::curve_intersection::CurveOverlapCorrespondence2::ParameterComponent {
                        source: component.clone(),
                        swapped: false,
                    },
            };
            let second = if reversed {
                [q(1, 2).into(), q(3, 4).into()]
            } else {
                [q(1, 4).into(), Real::one().into()]
            };
            let clipped = public
                .restrict([Real::zero().into(), q(1, 2).into()], second, &policy)
                .unwrap()
                .value;
            let Classification::Decided(Some(clipped)) = clipped else {
                panic!("the retained correspondence must reenter exact clipping");
            };
            let expected = if reversed {
                [q(1, 4), q(1, 2), q(3, 4), q(1, 2)]
            } else {
                [q(1, 4), q(1, 2), q(1, 4), q(1, 2)]
            };
            for (actual, expected) in [
                clipped.first_range().start(),
                clipped.first_range().end(),
                clipped.second_range().start(),
                clipped.second_range().end(),
            ]
            .into_iter()
            .zip(expected)
            {
                assert!(
                    actual.same_value(&expected.into(), &policy).unwrap()
                        == Classification::Decided(true)
                );
            }
            assert!(clipped.includes_start() && clipped.includes_end());
        }
        eprintln!(
            "REPARAMETERIZED_OVERLAP reversed={reversed} complete={} contacts={} overlaps={}",
            actual.is_complete(),
            actual.contacts().len(),
            actual.overlaps().len()
        );
        assert!(
            actual.is_complete(),
            "the structural correspondence needs complete residual evidence"
        );
        assert_eq!(
            actual.contacts().len(),
            2,
            "monotone reparameterization preserves both off-correspondence contacts"
        );
    }

    #[test]
    fn monotone_reparameterization_preserves_structural_pair_crossings_strict() {
        reparameterized_offset(CurveContext::STRICT, false);
    }

    #[test]
    fn monotone_reparameterization_preserves_structural_pair_crossings_approximate() {
        reparameterized_offset(CurveContext::APPROXIMATE_512, false);
    }

    #[test]
    fn reversed_monotone_reparameterization_preserves_structural_pair_crossings_strict() {
        reparameterized_offset(CurveContext::STRICT, true);
    }

    #[test]
    fn reversed_monotone_reparameterization_preserves_structural_pair_crossings_approximate() {
        reparameterized_offset(CurveContext::APPROXIMATE_512, true);
    }
}
