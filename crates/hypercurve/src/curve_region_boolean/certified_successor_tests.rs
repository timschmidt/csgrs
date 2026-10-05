use super::*;
use crate::bezier_offset::{
    BezierAlgebraicChordAxisDirection2, BezierAlgebraicCuspSemicircle2,
    BezierAlgebraicCuspSemicirclePairIntersections2, BezierAlgebraicCuspSemicircleParameter2,
    exact_selected_fiber_parameter_for_test, nonlinear_parameter_component_overlap_for_test,
};
use crate::{BezierAlgebraicCuspSemicircleFragment2, CubicBezier2};
use crate::{
    BezierAlgebraicParameter2, BezierParallelFragment2, CurvePath2, CurveRegionBoundaryLoop2,
    LineSeg2, Point2, QuadraticBezier2, RationalBezier2, RationalBezierAlgebraicPointImage2, Real,
};
use num::bigint::{BigInt, BigUint};

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("classification unexpectedly uncertain: {reason:?}")
        }
    }
}

fn carrier_parameter(parameter: BezierParameter2) -> CurveParameter2 {
    CurveParameter2::from(parameter)
}

fn carrier_range(range: &BezierParameterRange2) -> CurveParameterRange2 {
    CurveParameterRange2::from_bezier_range(range.clone())
}

fn sqrt_half_parameter(policy: &CurveContext) -> BezierAlgebraicParameter2 {
    let polynomial = decided(
        crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
            vec![(-1).into(), 0.into(), 2.into()],
            policy,
        )
        .expect("valid parameter polynomial"),
    );
    let interval = decided(
        crate::BezierParameterInterval::try_new_with_policy(
            (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
            Real::one(),
            policy,
        )
        .expect("valid parameter interval"),
    );
    decided(
        BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, policy)
            .expect("isolated parameter"),
    )
}

#[test]
fn exterior_circular_split_preserves_major_arc_and_algebraic_endpoints() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let arc = crate::CircularArc2::try_from_center(
            Point2::from_values(1, 0),
            Point2::from_values(0, 1),
            Point2::from_values(0, 0),
            false,
        )
        .unwrap();
        let source = BezierSubcurve2::RationalQuadratic(
            arc.rational_bezier_decomposition_with_policy(&policy)
                .unwrap()
                .value
                .spans()[0]
                .curve()
                .clone(),
        );
        // The quarter-circle source is ((1-t^2)/(1+t^2), 2t/(1+t^2)).
        // [-sqrt(2), sqrt(2)] traverses the major arc through (1, 0).
        let root = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let end = decided(
            root.affine_image_unbounded(&Real::from(2), &Real::zero(), &policy)
                .unwrap(),
        );
        let start = decided(
            end.affine_image_unbounded(&Real::from(-1), &Real::zero(), &policy)
                .unwrap(),
        );
        assert!(matches!(start, BezierParameter2::Algebraic(_)));
        assert!(matches!(end, BezierParameter2::Algebraic(_)));
        let range = CurveParameterRange2::new_validated(start.into(), end.into());
        let fragment = CurveSupport2::Bezier(source)
            .restrict_certified(range.clone(), None, false, &policy)
            .unwrap();
        let y = ((Real::from(2) * Real::from(2).sqrt().unwrap()) / Real::from(3)).unwrap();
        let x = (Real::from(-1) / Real::from(3)).unwrap();
        let points = [Point2::new(x.clone(), -&y), Point2::new(x, y)];
        let events = [range.start(), range.end()]
            .into_iter()
            .enumerate()
            .map(|(index, parameter)| CarrierEvent {
                parameter: parameter.clone(),
                topology_vertex: Some(index),
            })
            .collect::<Vec<_>>();
        let contacts = events
            .iter()
            .zip(points)
            .map(|(event, point)| ContactVertex {
                point: Some(CurvePoint2::from(point)),
                topology_vertex: event.topology_vertex.unwrap(),
                carrier_indices: [0, 1],
                parameters: [event.parameter.clone(), event.parameter.clone()],
            })
            .collect::<Vec<_>>();
        for reversed in [false, true] {
            let mut carrier = build_parameterized_carrier(
                &fragment,
                CurveRegionBooleanOperand2::First,
                0,
                0,
                true,
            );
            carrier.reversed = reversed;
            let splits = split_carrier(&carrier, &events, &contacts, &[0, 1], &policy).unwrap();
            let [split] = splits.as_slice() else {
                panic!("one unsplit major arc");
            };
            assert_eq!(
                split.fragment.curve_region_parameter_range(),
                range,
                "the source chart and selected endpoints must survive compaction",
            );
            let BezierSplitFragment2::RetainedBezier {
                start,
                end,
                source_curve,
                ..
            } = &split.fragment
            else {
                panic!("the compacted major arc retains its source Bezier");
            };
            let parameter = decided(start.strict_scalar_between(end, &policy).unwrap());
            let representative = decided(source_curve.point_at_with_policy(&parameter, &policy));
            assert_eq!(representative, Point2::from_values(1, 0));
            assert_eq!(split.start_topology_vertex, Some(usize::from(reversed)));
            assert_eq!(split.end_topology_vertex, Some(usize::from(!reversed)));
        }
    }
}

#[test]
fn selected_parameter_component_overlap_clips_in_both_boolean_orders() {
    let fraction =
        |numerator: i8, denominator: i8| (Real::from(numerator) / Real::from(denominator)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let retained = sqrt_half_parameter(&policy);
        let selected = |value: Real| {
            CurveParameter2::from_selected_fiber(exact_selected_fiber_parameter_for_test(
                retained.clone(),
                value,
                &policy,
            ))
        };
        let selected_range = |start: Real, end: Real| {
            CurveParameterRange2::new_validated(selected(start), selected(end))
        };
        let ordinary_range = |start: Real, end: Real| {
            CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(start, end))
        };
        let source = nonlinear_parameter_component_overlap_for_test(&policy);
        let first_overlap =
            CurveParameterRange2::from_bezier_range(source.overlap().first_range().clone());
        let second_overlap =
            CurveParameterRange2::from_bezier_range(source.overlap().second_range().clone());
        let line = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                .expect("the test carrier is nondegenerate"),
        ));
        let clip = |first_range: CurveParameterRange2,
                    second_range: CurveParameterRange2,
                    swapped: bool| {
            let carrier = |operand, range: &CurveParameterRange2| {
                let geometry = CurveSupport2::Bezier(line.clone());
                RegionCarrier {
                    operand,
                    loop_index: 0,
                    fragment_index: 0,
                    family: geometry.family(),
                    geometry,
                    start: range.start().clone(),
                    end: range.end().clone(),
                    reversed: false,
                    filled_side_is_left: true,
                    selected_fiber_endpoint_points: None,
                    image_is_injective: OnceLock::new(),
                    bounds: OnceLock::new(),
                    refined_bounds: Default::default(),
                }
            };
            let empty_first = CurveRegion2::empty();
            let empty_second = CurveRegion2::empty();
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        carrier(CurveRegionBooleanOperand2::First, &first_range),
                        carrier(CurveRegionBooleanOperand2::Second, &second_range),
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: Vec::new(),
                    regularization_fill_rule: None,
                    strict_line_image_only: OnceLock::new(),
                    operand_bounds: std::array::from_fn(|_| OnceLock::new()),
                },
            };
            let pair = RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::ParallelPair,
            };
            let (first_range, second_range) = if swapped {
                (second_overlap.clone(), first_overlap.clone())
            } else {
                (first_overlap.clone(), second_overlap.clone())
            };
            let overlap = CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                endpoint_inclusion: [true, true],
                parameter_correspondence: CurveOverlapCorrespondence2::ParameterComponent {
                    source: source.clone(),
                    swapped,
                },
                first_range,
                second_range,
                orientation: source.overlap().orientation(),
            };
            context
                .clipped_overlap_ranges(&pair, &overlap)
                .expect("selected component clipping must decide")
                .expect("the selected carrier ranges overlap")
        };
        let assert_selected = |parameter: &CurveParameter2, expected: Real| {
            assert_eq!(
                parameter
                    .as_selected_fiber()
                    .expect("an unchanged component bound must remain selected")
                    .order_to_real(&expected, &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Equal),
            );
        };

        let (first, second) = clip(
            selected_range(fraction(1, 4), fraction(3, 4)),
            ordinary_range(fraction(1, 16), fraction(1, 4)),
            false,
        );
        assert_selected(first.start(), fraction(1, 4));
        assert_eq!(first.end().scalar(), Some(&fraction(1, 2)));
        assert_eq!(
            second.scalar_endpoints(),
            Some((&fraction(1, 16), &fraction(1, 4))),
        );

        let (first, second) = clip(
            selected_range(fraction(1, 16), fraction(1, 4)),
            ordinary_range(fraction(1, 4), fraction(3, 4)),
            true,
        );
        assert_selected(first.start(), fraction(1, 16));
        assert_selected(first.end(), fraction(1, 4));
        assert_eq!(second.start().scalar(), Some(&fraction(1, 4)));
        assert_eq!(second.end().scalar(), Some(&fraction(1, 2)));
    }
}

#[test]
fn selected_projective_overlap_clips_without_global_parameter_projection() {
    let fraction =
        |numerator: i8, denominator: i8| (Real::from(numerator) / Real::from(denominator)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let retained = sqrt_half_parameter(&policy);
        let selected = |value: Real| {
            CurveParameter2::from_selected_fiber(exact_selected_fiber_parameter_for_test(
                retained.clone(),
                value,
                &policy,
            ))
        };
        let first_carrier_range =
            CurveParameterRange2::new_validated(selected(fraction(1, 2)), selected(fraction(3, 4)));
        let second_carrier_range =
            CurveParameterRange2::new_validated(selected(fraction(1, 3)), selected(fraction(1, 2)));
        let controls = vec![
            Point2::from_values(0, 0),
            Point2::from_values(1, 1),
            Point2::from_values(2, 0),
        ];
        let first = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3])
            .expect("valid polynomial quadratic");
        // Scaling homogeneous Bernstein control i by 2^i composes the
        // first parameter with t=2s/(1+s).
        let second = RationalBezier2::try_new(
            controls,
            vec![Real::one(), Real::from(2_i8), Real::from(4_i8)],
        )
        .expect("valid projectively reparameterized quadratic");
        let first_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
        let second_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
        let overlap = RationalBezierIntersectionOverlap2::from_certified_parameters(
            first_range.start().clone(),
            first_range.end().clone(),
            second_range.start().clone(),
            second_range.end().clone(),
            CurveOverlapOrientation2::Same,
            [true, true],
        );
        let endpoint_correspondence = RationalBezierOverlapParameterCorrespondence2::for_overlap(
            &first, &second, &overlap, &policy,
        );
        let carrier = |operand, curve: RationalBezier2, range: &CurveParameterRange2| {
            let geometry = CurveSupport2::Bezier(BezierSubcurve2::Rational(curve));
            RegionCarrier {
                operand,
                loop_index: 0,
                fragment_index: 0,
                family: geometry.family(),
                geometry,
                start: range.start().clone(),
                end: range.end().clone(),
                reversed: false,
                filled_side_is_left: true,
                selected_fiber_endpoint_points: None,
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
                refined_bounds: Default::default(),
            }
        };
        for correspondence in [
            endpoint_correspondence,
            RationalBezierOverlapParameterCorrespondence2::RangeProjective {
                second_to_first_scale: Real::from(2_i8),
                reversed: false,
            },
        ] {
            assert!(matches!(
                correspondence,
                RationalBezierOverlapParameterCorrespondence2::EndpointProjective { .. }
                    | RationalBezierOverlapParameterCorrespondence2::RangeProjective { .. }
            ));
            let first_carrier = carrier(
                CurveRegionBooleanOperand2::First,
                first.clone(),
                &first_carrier_range,
            );
            let second_carrier = carrier(
                CurveRegionBooleanOperand2::Second,
                second.clone(),
                &second_carrier_range,
            );
            let (first, second) = clip_corresponding_parameter_overlap(
                &first_range,
                &second_range,
                &correspondence,
                &first_carrier,
                &second_carrier,
                &policy,
            )
            .expect("selected projective clipping must decide")
            .expect("the selected projective subranges overlap");
            for (parameter, expected) in [
                (first.start(), fraction(1, 2)),
                (first.end(), fraction(2, 3)),
                (second.start(), fraction(1, 3)),
                (second.end(), fraction(1, 2)),
            ] {
                assert_eq!(
                    parameter
                        .as_selected_fiber()
                        .expect("projective clipping must retain a local selected scalar")
                        .order_to_real(&expected, &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal),
                );
            }
        }
    }
}

#[test]
fn selected_general_overlap_clips_through_exact_projection() {
    let fraction =
        |numerator: i8, denominator: i8| (Real::from(numerator) / Real::from(denominator)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let retained = sqrt_half_parameter(&policy);
        let selected = |value: Real| {
            CurveParameter2::from_selected_fiber(exact_selected_fiber_parameter_for_test(
                retained.clone(),
                value,
                &policy,
            ))
        };
        let first_carrier_range =
            CurveParameterRange2::new_validated(selected(fraction(1, 4)), selected(fraction(3, 4)));
        let second_carrier_range = CurveParameterRange2::new_validated(
            selected(fraction(1, 16)),
            selected(fraction(1, 4)),
        );
        // The first line uses x=t^2 while the second uses x=u. Their
        // positive-dimensional image correspondence u=t^2 is neither
        // affine nor projective.
        let first = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
            ],
            vec![Real::one(); 3],
        )
        .expect("valid quadratically parameterized line");
        let second = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 0)],
            vec![Real::one(); 2],
        )
        .expect("valid affine line");
        let correspondence = RationalBezierOverlapParameterCorrespondence2::General {
            first: first.clone(),
            second: second.clone(),
            unresolved: None,
        };
        let carrier = |operand, curve: RationalBezier2, range: &CurveParameterRange2| {
            let geometry = CurveSupport2::Bezier(BezierSubcurve2::Rational(curve));
            RegionCarrier {
                operand,
                loop_index: 0,
                fragment_index: 0,
                family: geometry.family(),
                geometry,
                start: range.start().clone(),
                end: range.end().clone(),
                reversed: false,
                filled_side_is_left: true,
                selected_fiber_endpoint_points: None,
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
                refined_bounds: Default::default(),
            }
        };
        let first_carrier = carrier(
            CurveRegionBooleanOperand2::First,
            first,
            &first_carrier_range,
        );
        let second_carrier = carrier(
            CurveRegionBooleanOperand2::Second,
            second,
            &second_carrier_range,
        );
        let first_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
        let second_range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
        let (first, second) = clip_corresponding_parameter_overlap(
            &first_range,
            &second_range,
            &correspondence,
            &first_carrier,
            &second_carrier,
            &policy,
        )
        .expect("selected general clipping must decide")
        .expect("the selected nonlinear subranges overlap");
        assert_eq!(
            first
                .start()
                .as_selected_fiber()
                .expect("an unchanged first bound must remain selected")
                .order_to_real(&fraction(1, 4), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Equal),
        );
        assert_eq!(first.end().scalar(), Some(&fraction(1, 2)));
        for (parameter, expected) in [
            (second.start(), fraction(1, 16)),
            (second.end(), fraction(1, 4)),
        ] {
            assert_eq!(
                parameter
                    .as_selected_fiber()
                    .expect("an unchanged second bound must remain selected")
                    .order_to_real(&expected, &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Equal),
            );
        }
    }
}

fn sqrt_third_parameter(policy: &CurveContext) -> BezierAlgebraicParameter2 {
    let polynomial = decided(
        crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
            vec![(-1).into(), 0.into(), 3.into()],
            policy,
        )
        .expect("valid parameter polynomial"),
    );
    let interval = decided(
        crate::BezierParameterInterval::try_new_with_policy(
            (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
            Real::one(),
            policy,
        )
        .expect("valid parameter interval"),
    );
    decided(
        BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, policy)
            .expect("isolated parameter"),
    )
}

fn sqrt_reciprocal_parameter(denominator: i8, policy: &CurveContext) -> BezierAlgebraicParameter2 {
    let polynomial = decided(
        crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
            vec![(-1).into(), 0.into(), denominator.into()],
            policy,
        )
        .expect("valid reciprocal-square-root parameter polynomial"),
    );
    let interval = decided(
        crate::BezierParameterInterval::try_new_with_policy(
            (Real::one() / Real::from(4_i8)).expect("nonzero denominator"),
            (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
            policy,
        )
        .expect("valid reciprocal-square-root parameter interval"),
    );
    decided(
        BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, policy)
            .expect("isolated reciprocal-square-root parameter"),
    )
}

fn rational_line(start_x: i32, end_x: i32) -> RationalBezier2 {
    RationalBezier2::try_new(
        vec![
            Point2::from_values(start_x, 0),
            Point2::from_values(end_x, 0),
        ],
        vec![Real::one(); 2],
    )
    .expect("valid rational line")
}

#[test]
fn boundary_probe_reuses_endpoint_incidence_and_keeps_residual_contacts() {
    let fraction = |n: i8, d: i8| (Real::from(n) / Real::from(d)).unwrap();
    let a = fraction(1, 4);
    let b = fraction(1, 2);
    for r in [fraction(3, 4), fraction(1, 2).sqrt().unwrap()] {
        // x=t, y=(t-a)(t-b)(t-r). A horizontal probe ending at t=r
        // meets two other transverse branches. Its nearer start omits a.
        let c0 = -(&a * &b * &r);
        let c1 = &a * &b + (&a + &b) * &r;
        let c2 = -(&a + &b + &r);
        let source = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), c0.clone()),
                Point2::new(fraction(1, 3), &c0 + &c1 * fraction(1, 3)),
                Point2::new(
                    fraction(2, 3),
                    &c0 + &c1 * fraction(2, 3) + &c2 * fraction(1, 3),
                ),
                Point2::new(Real::one(), &c0 + &c1 + &c2 + Real::one()),
            ],
            vec![Real::one(); 4],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let empty = CurveRegion2::empty();
                let geometry = CurveSupport2::Bezier(BezierSubcurve2::Rational(source.clone()));
                let context = CurveRegionBooleanContext {
                    data: CurveRegionBooleanContextData {
                        first: &empty,
                        second: &empty,
                        policy,
                        carriers: vec![RegionCarrier {
                            operand: CurveRegionBooleanOperand2::First,
                            loop_index: 0,
                            fragment_index: 0,
                            family: geometry.family(),
                            geometry,
                            start: BezierParameter2::Exact(Real::zero()).into(),
                            end: BezierParameter2::Exact(Real::one()).into(),
                            reversed,
                            filled_side_is_left: true,
                            selected_fiber_endpoint_points: None,
                            image_is_injective: OnceLock::new(),
                            bounds: OnceLock::new(),
                            refined_bounds: Default::default(),
                        }],
                        first_carrier_count: 1,
                        authored_carrier_pair_count: 0,
                        pairs: Vec::new(),
                        regularization_fill_rule: None,
                        strict_line_image_only: OnceLock::new(),
                        operand_bounds: std::array::from_fn(|_| OnceLock::new()),
                    },
                };
                for (start, expected) in [
                    (Real::from(-1_i8), vec![a.clone(), b.clone(), r.clone()]),
                    (fraction(1, 3), vec![b.clone(), r.clone()]),
                ] {
                    let representative =
                        CurvePoint2::from(source.point_at_with_policy(&r, &policy).unwrap());
                    let probe = decided(
                        crate::BezierAlgebraicChord2::try_new(
                            Point2::new(start, Real::zero()).into(),
                            representative.clone(),
                            &policy,
                        )
                        .unwrap(),
                    );
                    let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
                    let source_parameter = BezierParameter2::Exact(r.clone()).into();
                    let outcome = crate::policy::resolve_certified_operation(&policy, |_| {
                        context
                            .intersect_algebraic_probe_boundary(probe, Some((0, &source_parameter)))
                    })
                    .unwrap();
                    assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                    let evidence = outcome.value;
                    assert!(evidence.blockers().is_empty());
                    assert!(evidence.overlaps().is_empty());
                    assert_eq!(evidence.contacts().len(), expected.len());
                    for parameter in expected {
                        let parameter = CurveParameter2::from(BezierParameter2::Exact(parameter));
                        let contacts = evidence
                            .contacts()
                            .iter()
                            .filter(|contact| {
                                contact.second_parameter().same_value(&parameter, &policy)
                                    == Ok(Classification::Decided(true))
                            })
                            .collect::<Vec<_>>();
                        assert_eq!(contacts.len(), 1, "each finite branch occurs exactly once");
                        assert!(contacts[0].is_certified_transverse());
                        if parameter == source_parameter {
                            assert_eq!(contacts[0].first_parameter(), &probe_end);
                            assert_eq!(contacts[0].point(), Some(&representative));
                        }
                    }
                }
            }
        }
    }
}

fn algebraic_chord_carrier(
    operand: CurveRegionBooleanOperand2,
    chord: crate::BezierAlgebraicChord2,
) -> RegionCarrier {
    RegionCarrier {
        operand,
        loop_index: 0,
        fragment_index: 0,
        family: CurveFamily2::Line,
        start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
        end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
        geometry: CurveSupport2::Line(chord),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    }
}

fn selected_field_algebraic_chord_rectangle(policy: &CurveContext) -> CurveRegion2 {
    let parameter = sqrt_half_parameter(policy);
    let point = |positive: bool, height: i32| {
        let endpoint_x = if positive { 1 } else { -1 };
        CurvePoint2::from(crate::tests::decided(
            RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, height),
                    Point2::from_values(endpoint_x, height),
                ],
                vec![Real::one(); 2],
            )
            .expect("valid selected-field line")
            .point_at_algebraic_parameter(&parameter, policy)
            .expect("selected-field endpoint"),
        ))
    };
    let bottom_left = point(false, 0);
    let bottom_right = point(true, 0);
    let top_right = point(true, 1);
    let top_left = point(false, 1);
    let chord = |start, end| {
        BezierSplitFragment2::AlgebraicChord(decided(
            crate::BezierAlgebraicChord2::try_new(start, end, policy)
                .expect("valid retained chord"),
        ))
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            chord(bottom_left.clone(), bottom_right.clone()),
            chord(bottom_right, top_right.clone()),
            chord(top_right, top_left.clone()),
            chord(top_left, bottom_left),
        ],
        policy,
    )
    .expect("valid selected-field rectangle");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
    )
    .expect("valid selected-field region")
}

#[test]
fn rational_circle_contact_certificate_keeps_coincident_supports_in_general_replay() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // The selected circle has center (0,0) and radius one. Its center
        // comes from a line, so there is no concentric-parent certificate.
        let circle = decided(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                QuadraticBezier2::new(
                    Point2::from_values(-1, 0),
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 0),
                )
                .parallel_left(Real::zero())
                .unwrap(),
                BezierParameter2::Exact(half.clone()).into(),
                Real::one(),
                true,
                &policy,
            )
            .unwrap(),
        )
        .unwrap();
        let quarter = |points| {
            RationalBezier2::try_new(points, vec![Real::one(), Real::one(), Real::from(2)]).unwrap()
        };
        // This radius-two circle is tangent at (1,0). Radius inequality
        // alone is insufficient: the caller must own the tangency proof.
        let tangent = quarter(vec![
            Point2::from_values(1, 0),
            Point2::from_values(1, 2),
            Point2::from_values(3, 2),
        ]);
        assert!(
            circle
                .certifies_unique_rational_circle_contact(&tangent, true, &policy)
                .unwrap()
        );
        assert!(
            !circle
                .certifies_unique_rational_circle_contact(&tangent, false, &policy)
                .unwrap()
        );
        // Coincident circles have parallel tangents at every common
        // point. Their overlap must not become a singleton contact.
        let coincident = quarter(vec![
            Point2::from_values(1, 0),
            Point2::from_values(1, 1),
            Point2::from_values(0, 1),
        ]);
        assert!(
            !circle
                .certifies_unique_rational_circle_contact(&coincident, true, &policy)
                .unwrap()
        );
    }
}

#[test]
fn sibling_circle_tangency_preserves_half_chart_and_finite_range_ownership() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let three_quarters = &quarter * Real::from(3_i8);
    let parameters = [Real::zero(), half.clone(), Real::one()];
    let points = [
        Point2::from_values(0, 1),
        Point2::from_values(-1, 0),
        Point2::from_values(0, -1),
    ];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let circle = decided(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                QuadraticBezier2::new(
                    Point2::from_values(-1, 0),
                    Point2::from_values(0, 0),
                    Point2::from_values(1, 0),
                )
                .parallel_left(Real::zero())
                .unwrap(),
                BezierParameter2::Exact(half.clone()).into(),
                Real::one(),
                false,
                &policy,
            )
            .unwrap(),
        )
        .unwrap();
        for (lower, upper, reversed) in [(0, 1, false), (1, 2, true), (0, 2, false), (0, 2, true)] {
            let fragment = decided(
                BezierAlgebraicCuspSemicircleFragment2::try_new(
                    circle.clone(),
                    BezierAlgebraicCuspSemicircleParameter2::Exact(parameters[lower].clone()),
                    BezierAlgebraicCuspSemicircleParameter2::Exact(parameters[upper].clone()),
                    reversed,
                    &policy,
                )
                .unwrap(),
            )
            .with_certified_tangent_endpoints();
            let (start_index, end_index) = if reversed {
                (upper, lower)
            } else {
                (lower, upper)
            };
            let start = &points[start_index];
            let end = &points[end_index];
            let tangent = |point: &Point2| {
                if reversed {
                    (point.y().clone(), -point.x())
                } else {
                    (-point.y(), point.x().clone())
                }
            };
            let end_tangent = tangent(end);
            let start_tangent = tangent(start);
            let chord_end = end.translated(end_tangent.0, end_tangent.1);
            let closing_start = start.translated(-start_tangent.0, -start_tangent.1);
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    CurvePoint2::from(end.clone()),
                    CurvePoint2::from(chord_end.clone()),
                    &policy,
                )
                .unwrap(),
            );
            let line = |start, end| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(start, end).unwrap(),
                )),
            };
            let source = CurveRegion2::try_new_with_loop_topology(
                vec![
                    CurveRegionBoundaryLoop2::new(
                        vec![
                            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment.clone()),
                            BezierSplitFragment2::AlgebraicChord(chord.clone()),
                            line(chord_end, closing_start.clone()),
                            line(closing_start, start.clone()),
                        ],
                        &policy,
                    )
                    .unwrap(),
                ],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
            )
            .unwrap();
            for complementary in [false, true] {
                for clipped in [false, true] {
                    for target_reversed in [false, true] {
                        let mut sibling = cusp_test_carrier(
                            circle.clone(),
                            parameters[lower].clone(),
                            parameters[upper].clone(),
                            CurveRegionBooleanOperand2::First,
                            &policy,
                        );
                        sibling.geometry = CurveSupport2::Circle(fragment.clone());
                        sibling.reversed = reversed;
                        let mut chord_carrier = algebraic_chord_carrier(
                            CurveRegionBooleanOperand2::First,
                            chord.clone(),
                        );
                        chord_carrier.fragment_index = 1;
                        let mut target = cusp_test_carrier(
                            if complementary {
                                circle.complementary_half()
                            } else {
                                circle.clone()
                            },
                            if clipped {
                                quarter.clone()
                            } else {
                                Real::zero()
                            },
                            if clipped {
                                three_quarters.clone()
                            } else {
                                Real::one()
                            },
                            CurveRegionBooleanOperand2::Second,
                            &policy,
                        );
                        if target_reversed {
                            target.geometry =
                                CurveSupport2::Circle(target.geometry.circle().reversed());
                            target.reversed = true;
                        }
                        let empty = CurveRegion2::empty();
                        let pair = RegionCarrierPair {
                            first_carrier_index: 2,
                            second_carrier_index: 1,
                            context: RegionCarrierPairContext::CuspChord {
                                cusp_is_first: true,
                            },
                        };
                        let context = CurveRegionBooleanContext {
                            data: CurveRegionBooleanContextData {
                                first: &source,
                                second: &empty,
                                policy,
                                carriers: vec![sibling, chord_carrier, target],
                                first_carrier_count: 2,
                                authored_carrier_pair_count: 1,
                                pairs: vec![pair],
                                regularization_fill_rule: None,
                                strict_line_image_only: OnceLock::new(),
                                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
                            },
                        };
                        let outcome = crate::policy::resolve_certified_value(&policy, |_| {
                            context.pair_result(&context.data.pairs[0]).unwrap()
                        });
                        assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                        let result = outcome.value;
                        assert!(result.blockers.is_empty(), "{result:?}");
                        let expected = if clipped {
                            !complementary && end_index == 1
                        } else {
                            !complementary || end_index != 1
                        };
                        assert_eq!(
                            result.contacts.len(),
                            usize::from(expected),
                            "policy={policy:?} source=({lower},{upper},{reversed}) complementary={complementary} clipped={clipped} reversed={target_reversed}"
                        );
                        if expected {
                            let contact = &result.contacts[0];
                            assert!(!contact.is_certified_transverse());
                            assert_eq!(contact.tangent_cross_sign, Some(RealSign::Zero));
                            assert_eq!(contact.point().unwrap().coordinates(), Some(end));
                            let expected_parameter = if complementary {
                                Real::one() - &parameters[end_index]
                            } else {
                                parameters[end_index].clone()
                            };
                            assert_eq!(
                                contact
                                    .first_parameter()
                                    .as_algebraic_cusp()
                                    .unwrap()
                                    .order_to_real(&expected_parameter, &policy)
                                    .unwrap(),
                                Classification::Decided(Ordering::Equal)
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn isolated_circle_endpoint_contacts_retain_transverse_signs() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = sqrt_half_parameter(&policy);
        let center = CurvePoint2::from(
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter.clone(),
                crate::bezier_algebraic_image::parameter_representation(&parameter, &policy),
                vec![Real::zero(), Real::one()],
                vec![Real::zero()],
                vec![Real::one()],
                "endpoint-only radial contact center",
            ),
        );
        for clockwise in [false, true] {
            let semicircle = decided(
                BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                    &center,
                    (1, 0),
                    Real::from(2_i8),
                    clockwise,
                    &policy,
                )
                .unwrap(),
            )
            .unwrap();
            let start = decided(semicircle.start_point_evidence(&policy).unwrap());
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(start, center.clone(), &policy).unwrap(),
            );
            let fragment = BezierAlgebraicCuspSemicircleFragment2::full(semicircle, &policy);
            for fragment in [fragment.clone(), fragment.reversed()] {
                for reversed_chord in [false, true] {
                    let chord = if reversed_chord {
                        chord.reversed()
                    } else {
                        chord.clone()
                    };
                    let contact = decided(
                        fragment
                            .certified_chord_endpoint_contact(&chord, &policy)
                            .unwrap(),
                    )
                    .expect("the radial chord has only its circle endpoint in common");
                    // T=turn*perp(R), D=-R: T×D=turn*|R|². Reversing
                    // the arc fragment does not reverse its supporting chart.
                    let expected = if clockwise != reversed_chord {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    };
                    assert_eq!(contact.tangent_cross_sign, expected);
                    assert_eq!(
                        contact
                            .tangent_topology(fragment.semicircle(), &chord, &policy)
                            .unwrap(),
                        Classification::Decided(None)
                    );
                }
            }
        }
    }
}

#[test]
fn cusp_chord_pair_retains_an_interior_axis_contact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = sqrt_half_parameter(&policy);
        let point = |x: Vec<Real>, y: Vec<Real>| {
            CurvePoint2::from(
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    parameter.clone(),
                    crate::bezier_algebraic_image::parameter_representation(&parameter, &policy),
                    x,
                    y,
                    vec![Real::one()],
                    "test region cusp/chord point",
                ),
            )
        };
        let center = point(vec![Real::zero(), Real::one()], vec![Real::zero()]);
        let semicircle = decided(
            BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::from(2_i8),
                false,
                &policy,
            )
            .expect("valid selected circle"),
        )
        .expect("nonzero circle radius");
        let chord = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
            point(vec![Real::zero(), Real::one()], vec![Real::from(-3_i8)]),
            point(vec![Real::zero(), Real::one()], vec![Real::from(3_i8)]),
            BezierAlgebraicChordAxisDirection2::PositiveY,
            &policy,
        );
        let cusp = cusp_test_carrier(
            semicircle,
            Real::zero(),
            Real::one(),
            CurveRegionBooleanOperand2::First,
            &policy,
        );
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let pair = RegionCarrierPair {
            first_carrier_index: 0,
            second_carrier_index: 1,
            context: RegionCarrierPairContext::CuspChord {
                cusp_is_first: true,
            },
        };
        let context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers: vec![
                    cusp,
                    algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone()),
                ],
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: vec![pair],
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let result = context
            .pair_result(&context.data.pairs[0])
            .expect("axis cusp/chord pair must complete");
        assert!(result.blockers.is_empty(), "{result:?}");
        let [contact] = result.contacts.as_slice() else {
            panic!("expected one retained cusp/chord contact: {result:?}");
        };
        assert!(contact.is_certified_transverse());
        assert_eq!(contact.tangent_cross_sign, Some(RealSign::Negative));
        assert!(matches!(
            contact.point(),
            Some(CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)))
        ));
        let cusp_parameter = contact
            .first_parameter()
            .as_algebraic_cusp()
            .expect("first carrier must retain the cusp parameter");
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        assert_eq!(
            cusp_parameter.order_to_real(&half, &policy).unwrap(),
            Classification::Decided(Ordering::Equal),
        );
        let chord_parameter = contact
            .second_parameter()
            .as_algebraic_chord()
            .expect("second carrier must retain the chord parameter");
        assert_eq!(
            chord_parameter
                .cmp_by_refinement(&chord.start_parameter(), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Greater),
        );
        assert_eq!(
            chord_parameter
                .cmp_by_refinement(&chord.end_parameter(), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Less),
        );
        let evidence = context
            .build_intersection_evidence()
            .expect("axis cusp/chord contact must enter region evidence");
        assert!(evidence.is_complete(), "{evidence:?}");
        assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
    }
}

#[test]
fn cusp_chord_pair_retains_an_interior_exact_oblique_contact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = sqrt_half_parameter(&policy);
        let center = CurvePoint2::from(
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter.clone(),
                crate::bezier_algebraic_image::parameter_representation(&parameter, &policy),
                vec![Real::zero(), Real::one()],
                vec![Real::zero()],
                vec![Real::one()],
                "test region oblique cusp/chord center",
            ),
        );
        let semicircle = decided(
            BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::from(2_i8),
                false,
                &policy,
            )
            .expect("valid selected circle"),
        )
        .expect("nonzero circle radius");
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(-3, -3)),
                CurvePoint2::from(Point2::from_values(3, 3)),
                &policy,
            )
            .expect("valid exact oblique chord"),
        );
        let cusp = cusp_test_carrier(
            semicircle,
            Real::zero(),
            Real::one(),
            CurveRegionBooleanOperand2::First,
            &policy,
        );
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let pair = RegionCarrierPair {
            first_carrier_index: 0,
            second_carrier_index: 1,
            context: RegionCarrierPairContext::CuspChord {
                cusp_is_first: true,
            },
        };
        let context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers: vec![
                    cusp,
                    algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone()),
                ],
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: vec![pair],
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let result = context
            .pair_result(&context.data.pairs[0])
            .expect("exact oblique cusp/chord pair must complete");
        assert!(result.blockers.is_empty(), "{result:?}");
        let [contact] = result.contacts.as_slice() else {
            panic!("expected one retained oblique cusp/chord contact: {result:?}");
        };
        assert!(contact.is_certified_transverse());
        assert_eq!(contact.tangent_cross_sign, Some(RealSign::Negative));
        let point = contact
            .point()
            .expect("the exact oblique contact must retain point evidence");
        assert!(matches!(
            point,
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        ));
        assert_eq!(
            chord.contains_point(point, &policy).unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            context.data.carriers[0]
                .geometry
                .circle()
                .contains_point(point, &policy)
                .unwrap(),
            Classification::Decided(true),
        );
        let chord_parameter = contact
            .second_parameter()
            .as_algebraic_chord()
            .expect("the second carrier must retain an oblique chord parameter");
        assert_eq!(
            chord_parameter
                .cmp_by_refinement(&chord.start_parameter(), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Greater),
        );
        assert_eq!(
            chord_parameter
                .cmp_by_refinement(&chord.end_parameter(), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Less),
        );
        let evidence = context
            .build_intersection_evidence()
            .expect("the exact oblique contact must enter region evidence");
        assert!(evidence.is_complete(), "{evidence:?}");
        assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
    }
}

#[test]
fn cusp_chord_pair_retains_an_independent_field_oblique_contact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let center_parameter = sqrt_half_parameter(&policy);
        let center = CurvePoint2::from(
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                center_parameter.clone(),
                crate::bezier_algebraic_image::parameter_representation(&center_parameter, &policy),
                vec![Real::zero(), Real::one()],
                vec![Real::zero()],
                vec![Real::one()],
                "test region independent-field oblique circle center",
            ),
        );
        let semicircle = decided(
            BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::from(2_i8),
                false,
                &policy,
            )
            .expect("valid selected circle"),
        )
        .expect("nonzero circle radius");
        let endpoint = |parameter: &BezierAlgebraicParameter2, start: Point2, end: Point2| {
            let curve = RationalBezier2::try_new(vec![start, end], vec![Real::one(), Real::one()])
                .expect("valid endpoint carrier");
            CurvePoint2::from(crate::tests::decided(
                curve
                    .point_at_algebraic_parameter(parameter, &policy)
                    .expect("valid endpoint image"),
            ))
        };
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                endpoint(
                    &sqrt_reciprocal_parameter(5, &policy),
                    Point2::from_values(-3, -3),
                    Point2::from_values(-2, -3),
                ),
                endpoint(
                    &sqrt_reciprocal_parameter(7, &policy),
                    Point2::from_values(3, 3),
                    Point2::from_values(3, 4),
                ),
                &policy,
            )
            .expect("valid independent-field oblique chord"),
        );
        assert!(chord.exact_line().is_none());
        let cusp = cusp_test_carrier(
            semicircle,
            Real::zero(),
            Real::one(),
            CurveRegionBooleanOperand2::First,
            &policy,
        );
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let pair = RegionCarrierPair {
            first_carrier_index: 0,
            second_carrier_index: 1,
            context: RegionCarrierPairContext::CuspChord {
                cusp_is_first: true,
            },
        };
        let context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers: vec![
                    cusp,
                    algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone()),
                ],
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: vec![pair],
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let result = context
            .pair_result(&context.data.pairs[0])
            .expect("independent-field oblique cusp/chord pair must complete");
        assert!(result.blockers.is_empty(), "{result:?}");
        let [contact] = result.contacts.as_slice() else {
            panic!("expected one independent-field oblique contact: {result:?}");
        };
        assert!(contact.is_certified_transverse());
        assert_eq!(contact.tangent_cross_sign, Some(RealSign::Negative));
        assert!(matches!(
            contact.point(),
            Some(CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)))
        ));
        let chord_parameter = contact
            .second_parameter()
            .as_algebraic_chord()
            .expect("the second carrier must retain an oblique chord parameter");
        assert_eq!(
            chord_parameter
                .cmp_by_refinement(&chord.start_parameter(), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Greater),
        );
        assert_eq!(
            chord_parameter
                .cmp_by_refinement(&chord.end_parameter(), &policy)
                .unwrap(),
            Classification::Decided(Ordering::Less),
        );
        let evidence = context
            .build_intersection_evidence()
            .expect("the independent-field oblique contact must enter region evidence");
        assert!(evidence.is_complete(), "{evidence:?}");
        assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
    }
}

#[test]
fn shared_chord_splits_contacts_from_independent_selected_circles_in_order() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first_parameter = sqrt_half_parameter(&policy);
        let second_parameter = sqrt_third_parameter(&policy);
        let center = |parameter: &BezierAlgebraicParameter2, label| {
            CurvePoint2::from(
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    parameter.clone(),
                    crate::bezier_algebraic_image::parameter_representation(parameter, &policy),
                    vec![Real::zero(), Real::one()],
                    vec![Real::zero()],
                    vec![Real::one()],
                    label,
                ),
            )
        };
        let circle = |parameter: &BezierAlgebraicParameter2, label| {
            decided(
                BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                    &center(parameter, label),
                    (1, 0),
                    Real::from(2_i8),
                    false,
                    &policy,
                )
                .expect("valid independent selected circle"),
            )
            .expect("nonzero independent selected circle")
        };
        let chord = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
            CurvePoint2::from(Point2::from_values(-3, 1)),
            CurvePoint2::from(Point2::from_values(3, 1)),
            BezierAlgebraicChordAxisDirection2::PositiveX,
            &policy,
        );
        let contacts = |circle: &BezierAlgebraicCuspSemicircle2| -> [CurveParameter2; 2] {
            let Classification::Decided(contacts) =
                circle.chord_intersections(&chord, &policy).unwrap()
            else {
                panic!("both independent circle contacts must be retained");
            };
            assert_eq!(contacts.len(), 2);
            std::array::from_fn(|index| {
                CurveParameter2::from_algebraic_chord(contacts[index].chord_parameter.clone())
            })
        };
        let first = contacts(&circle(
            &first_parameter,
            "first shared-chord selected circle center",
        ));
        let second = contacts(&circle(
            &second_parameter,
            "second shared-chord selected circle center",
        ));
        let carrier = algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, chord.clone());
        let event = |parameter, topology_vertex| CarrierEvent {
            parameter,
            topology_vertex: Some(topology_vertex),
        };
        let events = [
            event(carrier.end.clone(), 6),
            event(first[1].clone(), 5),
            event(second[0].clone(), 1),
            event(carrier.start.clone(), 0),
            event(second[1].clone(), 4),
            event(first[0].clone(), 2),
        ];
        let fragments = split_algebraic_chord_carrier(&carrier, &chord, &events, &policy)
            .expect("independent selected-circle contacts must split the shared chord");
        assert_eq!(fragments.len(), 5);
        assert_eq!(
            fragments
                .iter()
                .map(|fragment| (fragment.start_topology_vertex, fragment.end_topology_vertex,))
                .collect::<Vec<_>>(),
            vec![
                (Some(0), Some(1)),
                (Some(1), Some(2)),
                (Some(2), Some(4)),
                (Some(4), Some(5)),
                (Some(5), Some(6)),
            ],
        );
    }
}

#[test]
fn algebraic_chord_carrier_retains_ordered_interior_splits() {
    let policy = CurveContext::STRICT;
    let chord = decided(
        crate::BezierAlgebraicChord2::try_new(
            CurvePoint2::from(Point2::from_values(0, 0)),
            CurvePoint2::from(Point2::from_values(4, 0)),
            &policy,
        )
        .expect("valid retained chord"),
    );
    let geometry = CurveSupport2::Line(chord.clone());
    let carrier = RegionCarrier {
        operand: CurveRegionBooleanOperand2::First,
        loop_index: 0,
        fragment_index: 0,
        family: geometry.family(),
        geometry,
        start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
        end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    };
    let cut = |x, vertex| CarrierEvent {
        parameter: CurveParameter2::from_algebraic_chord(
            decided(
                chord
                    .parameter_at_certified_point(
                        CurvePoint2::from(Point2::from_values(x, 0)),
                        &policy,
                    )
                    .expect("certified chord point"),
            )
            .expect("the cut lies on the chord"),
        ),
        topology_vertex: Some(vertex),
    };
    let events = vec![cut(4, 4), cut(1, 1), cut(0, 0), cut(3, 3)];
    let splits = split_algebraic_chord_carrier(&carrier, &chord, &events, &policy)
        .expect("interior chord cuts must remain exact");
    assert_eq!(splits.len(), 3);
    assert_eq!(
        splits
            .iter()
            .map(|split| (split.start_topology_vertex, split.end_topology_vertex))
            .collect::<Vec<_>>(),
        vec![(Some(0), Some(1)), (Some(1), Some(3)), (Some(3), Some(4))]
    );
    for split in splits {
        assert!(matches!(
            split.fragment,
            BezierSplitFragment2::AlgebraicChord(_)
        ));
    }
}

#[test]
fn exact_retained_chord_uses_the_canonical_boolean_carrier() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(0, 0)),
                CurvePoint2::from(Point2::from_values(2, 0)),
                &policy,
            )
            .expect("valid exact retained chord"),
        );
        let curved = CubicBezier2::new(
            Point2::from_values(2, 0),
            Point2::from_values(2, 2),
            Point2::from_values(0, 2),
            Point2::from_values(0, 0),
        );
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(chord),
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Cubic(curved),
                },
            ],
            &policy,
        )
        .expect("the exact chord and curved return must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid exact retained-chord region");
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid unary exact-chord context");
        assert!(matches!(
            context.data.carriers[0].geometry,
            CurveSupport2::Line(_)
        ));
        let regularized = context.build_regularized_region();
        assert!(
            regularized.is_ok(),
            "the exact retained chord must preserve the canonical chord authority: {regularized:?}"
        );
    }
}

#[test]
fn exact_subfragment_of_algebraic_chord_retains_selected_field_witness() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = selected_field_algebraic_chord_rectangle(&policy);
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid unary algebraic-chord context");
        let CurveSupport2::Line(source) = &context.data.carriers[0].geometry else {
            panic!("the selected-field bottom edge must stay algebraic");
        };
        let cut = |x: Real| {
            decided(
                source
                    .parameter_at_certified_point(
                        CurvePoint2::from(Point2::new(x, Real::zero())),
                        &policy,
                    )
                    .expect("exact point on retained support"),
            )
            .expect("the exact point lies strictly on the source chord")
        };
        let exact_subfragment = crate::BezierAlgebraicChord2::from_ordered_parameter_range(
            source,
            &cut(Real::zero()),
            &cut((Real::one() / Real::from(2_u8)).expect("nonzero denominator")),
            &policy,
        )
        .expect("ordered exact subfragment");
        assert!(exact_subfragment.exact_line().is_some());
        let action = context
            .regularized_algebraic_chord_fragment_decision(0, &exact_subfragment, true)
            .map(|decision| decision.action);
        assert!(
            action.is_ok(),
            "an exact subfragment of an algebraic carrier must retain a selected-field side witness: {action:?}"
        );
    }
}

fn assert_trim_endpoint_replay(
    source: &Curve2,
    trimmed: &crate::CurveRegionTrimFragment2,
    expected: [Point2; 2],
    policy: &CurveContext,
) {
    let curve = trimmed.curve();
    let Classification::Decided(range) = trimmed.parameter_range_with_policy(policy).unwrap()
    else {
        panic!("trim parameters must replay exactly");
    };
    for ((endpoint, parameter), expected) in
        [(curve.start(), range.start()), (curve.end(), range.end())]
            .into_iter()
            .zip(expected)
    {
        let expected = CurvePoint2::from(expected);
        assert_eq!(
            endpoint.same_point(&expected, &CurveContext::STRICT),
            Classification::Decided(true)
        );
        let evaluated = source
            .point_at_with_policy(parameter, policy)
            .expect("the trim parameter must reenter source evaluation");
        assert_eq!(evaluated.certainty, crate::CurveCertainty::Certified);
        assert_eq!(
            evaluated.value.same_point(&expected, &CurveContext::STRICT),
            Classification::Decided(true)
        );
    }
}

#[test]
fn curve_trim_retains_selected_field_algebraic_chord_boundaries() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = selected_field_algebraic_chord_rectangle(&policy);
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let source = Curve2::from(
            LineSeg2::try_new(
                Point2::new(Real::from(-1_i8), half.clone()),
                Point2::new(Real::one(), half.clone()),
            )
            .expect("valid horizontal cutter"),
        );

        let trimmed = source
            .trim_inside_region_with_parameters_with_policy(&region, &policy)
            .expect("a rational line must trim against selected-field chords");
        assert_eq!(trimmed.certainty, crate::CurveCertainty::Certified);
        let [trimmed] = trimmed.value.as_slice() else {
            panic!("the algebraic rectangle must retain one exact interval");
        };
        let width = half.clone().sqrt().unwrap();
        assert_trim_endpoint_replay(
            &source,
            trimmed,
            [
                Point2::new(-width.clone(), half.clone()),
                Point2::new(width, half),
            ],
            &policy,
        );
        assert_eq!(trimmed.start_boundary_contacts().len(), 1);
        assert_eq!(trimmed.end_boundary_contacts().len(), 1);
        assert_eq!(
            trimmed.start_boundary_contacts()[0]
                .carrier()
                .fragment_index(),
            3
        );
        assert_eq!(
            trimmed.end_boundary_contacts()[0]
                .carrier()
                .fragment_index(),
            1
        );
        assert!(
            trimmed.start_boundary_contacts()[0]
                .boundary_parameter()
                .is_algebraic_chord()
        );
        assert!(
            trimmed.end_boundary_contacts()[0]
                .boundary_parameter()
                .is_algebraic_chord()
        );
    }
}

#[test]
fn curve_trim_retains_selected_field_algebraic_chord_overlaps() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_field_algebraic_chord_rectangle(&policy);
            let BezierSplitFragment2::AlgebraicChord(bottom) =
                &region.boundary_loops()[0].fragments()[0]
            else {
                unreachable!("the selected-field bottom edge is an algebraic chord");
            };
            let (start, end) = if reversed {
                (Point2::from_values(1, 0), Point2::from_values(-1, 0))
            } else {
                (Point2::from_values(-1, 0), Point2::from_values(1, 0))
            };
            let source = Curve2::from(
                LineSeg2::try_new(start, end).expect("valid horizontal overlap source"),
            );

            let trimmed = source
                .trim_inside_region_with_parameters_with_policy(&region, &policy)
                .expect("selected-field boundary overlap must trim exactly");
            assert_eq!(trimmed.certainty, crate::CurveCertainty::Certified);
            let [trimmed] = trimmed.value.as_slice() else {
                panic!("the selected-field bottom edge must retain one exact interval");
            };
            let width = (Real::one() / Real::from(2_i8)).unwrap().sqrt().unwrap();
            let mut expected = [
                Point2::new(-width.clone(), Real::zero()),
                Point2::new(width, Real::zero()),
            ];
            if reversed {
                expected.reverse();
            }
            assert_trim_endpoint_replay(&source, trimmed, expected, &policy);
            let start_contact = trimmed
                .start_boundary_contacts()
                .iter()
                .find(|contact| contact.carrier().fragment_index() == 0)
                .expect("the overlap start must retain bottom-edge provenance");
            let end_contact = trimmed
                .end_boundary_contacts()
                .iter()
                .find(|contact| contact.carrier().fragment_index() == 0)
                .expect("the overlap end must retain bottom-edge provenance");
            let start_parameter = start_contact
                .boundary_parameter()
                .as_algebraic_chord()
                .expect("bottom-edge contact must retain its chord parameter");
            let end_parameter = end_contact
                .boundary_parameter()
                .as_algebraic_chord()
                .expect("bottom-edge contact must retain its chord parameter");
            let (expected_start, expected_end) = if reversed {
                (bottom.end_parameter(), bottom.start_parameter())
            } else {
                (bottom.start_parameter(), bottom.end_parameter())
            };
            assert_eq!(
                start_parameter
                    .cmp_by_refinement(&expected_start, &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Equal)
            );
            assert_eq!(
                end_parameter
                    .cmp_by_refinement(&expected_end, &policy)
                    .unwrap(),
                Classification::Decided(Ordering::Equal)
            );
        }
    }
}

#[test]
fn source_related_algebraic_chord_contact_enters_split_topology() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
        let source_curve = BezierSubcurve2::Cubic(CubicBezier2::new(
            Point2::from_values(1, 0),
            Point2::new(Real::one() + &third, third.clone()),
            Point2::new(
                Real::one() + Real::from(2_i8) * &third,
                Real::from(2_i8) * &third,
            ),
            Point2::from_values(2, 0),
        ));
        let source_rational =
            RationalBezier2::try_from_subcurve(&source_curve).expect("valid rational source");
        let parameter = sqrt_half_parameter(&policy);
        let source_parameter = BezierParameter2::Algebraic(parameter.clone());
        let materialization = decided(
            source_curve
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&source_parameter),
                    &policy,
                )
                .expect("exact algebraic source split"),
        );
        let source_fragment = materialization.fragments()[0].clone();
        let selected_point = crate::tests::decided(
            exact_contact_point_evidence(&source_rational, &source_parameter, &policy)
                .expect("exact selected point construction"),
        );
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                selected_point,
                CurvePoint2::from(Point2::from_values(0, 0)),
                &policy,
            )
            .expect("valid algebraic chord"),
        );
        let closure = LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
            .expect("valid closure");
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                source_fragment,
                BezierSplitFragment2::AlgebraicChord(chord),
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(closure)),
                },
            ],
            &policy,
        )
        .expect("the correlated self-crossing loop must close exactly");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid retained test region");
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid unary Boolean context");
        let pair = context
            .data
            .pairs
            .iter()
            .find(|pair| {
                matches!(
                    context.data.carriers[pair.first_carrier_index].geometry,
                    CurveSupport2::Line(_)
                ) || matches!(
                    context.data.carriers[pair.second_carrier_index].geometry,
                    CurveSupport2::Line(_)
                )
            })
            .expect("the source/chord pair must be scheduled");
        let pair_result = context
            .pair_result(pair)
            .expect("source-related pair replay must complete");
        assert!(pair_result.blockers.is_empty());
        assert_eq!(pair_result.contacts.len(), 1);
        assert!(pair_result.contacts[0].is_certified_transverse());
        assert!(
            pair_result.contacts[0]
                .first_parameter()
                .is_algebraic_chord()
                || pair_result.contacts[0]
                    .second_parameter()
                    .is_algebraic_chord()
        );

        let topology = context
            .build_split_topology()
            .expect("the correlated contact must enter the common split topology");
        let chord_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| matches!(carrier.geometry, CurveSupport2::Line(_)))
            .expect("retained chord carrier");
        assert_eq!(topology.split_fragments[chord_index].len(), 2);
        assert_eq!(
            topology.split_fragments[pair.first_carrier_index].len()
                + topology.split_fragments[pair.second_carrier_index].len(),
            4
        );
        for split in &topology.split_fragments[chord_index] {
            let BezierSplitFragment2::AlgebraicChord(chord) = &split.fragment else {
                unreachable!();
            };
            let representative = chord.representative_point(&policy);
            assert!(
                matches!(representative, Ok(Classification::Decided(_))),
                "split chord representative: {representative:?}"
            );
            let Classification::Decided(CurvePoint2(CurvePointData2::Algebraic(representative))) =
                representative.expect("representative construction")
            else {
                panic!("the split chord representative must remain algebraic");
            };
            let [tangent_x, tangent_y] = [Axis2::X, Axis2::Y].map(|axis| {
                chord
                    .tangent_axis_sign(axis, &policy)
                    .expect("tangent sign")
            });
            for left in [true, false] {
                let side = context
                    .algebraic_fragment_side_classification(
                        chord_index,
                        &representative,
                        tangent_x,
                        tangent_y,
                        left,
                    )
                    .map(|(_, location)| location);
                assert!(side.is_ok(), "split chord side {left}: {side:?}");
            }
            let action = context
                .regularized_algebraic_chord_fragment_decision(chord_index, chord, true)
                .map(|decision| decision.action);
            assert!(action.is_ok(), "split chord action: {action:?}");
        }
        let regularized = context.build_regularized_region();
        assert!(
            regularized.is_ok(),
            "the split algebraic chord must traverse the arrangement: {regularized:?}"
        );
    }
}

#[test]
fn nonadjacent_source_chord_pair_replays_endpoint_and_residual_contacts() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
        let source_curve = BezierSubcurve2::Cubic(CubicBezier2::new(
            Point2::from_values(1, 0),
            Point2::new(Real::one() + &third, third.clone()),
            Point2::new(
                Real::one() + Real::from(2_i8) * &third,
                Real::from(2_i8) * &third,
            ),
            Point2::from_values(2, 0),
        ));
        let source_rational =
            RationalBezier2::try_from_subcurve(&source_curve).expect("valid rational source");
        let parameter = sqrt_half_parameter(&policy);
        let source_parameter = BezierParameter2::Algebraic(parameter);
        let source_fragment = decided(
            source_curve
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&source_parameter),
                    &policy,
                )
                .expect("exact algebraic source split"),
        )
        .fragments()[0]
            .clone();
        let selected_point = crate::tests::decided(
            exact_contact_point_evidence(&source_rational, &source_parameter, &policy)
                .expect("exact selected point construction"),
        );
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                selected_point,
                CurvePoint2::from(Point2::from_values(0, 0)),
                &policy,
            )
            .expect("valid algebraic chord"),
        );
        let chord_loop = CurveRegionBoundaryLoop2::new(
            vec![
                source_fragment,
                BezierSplitFragment2::AlgebraicChord(chord),
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                            .expect("valid chord-loop closure"),
                    )),
                },
            ],
            &policy,
        )
        .expect("the retained chord loop must close");
        let source_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: source_curve,
                },
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(1, 0))
                            .expect("valid source-loop closure"),
                    )),
                },
            ],
            &policy,
        )
        .expect("the complete source loop must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![chord_loop, source_loop],
            vec![CurveRegionLoopRole::Material; 2],
            vec![FillRule::NonZero; 2],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left; 2],
        )
        .expect("valid multi-loop retained test region");
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid unary Boolean context");
        let chord_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| {
                carrier.loop_index == 0 && matches!(carrier.geometry, CurveSupport2::Line(_))
            })
            .expect("retained chord carrier");
        let source_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| carrier.loop_index == 1 && carrier.fragment_index == 0)
            .expect("nonadjacent source carrier");
        let pair = context
            .data
            .pairs
            .iter()
            .find(|pair| {
                (pair.first_carrier_index == chord_index
                    && pair.second_carrier_index == source_index)
                    || (pair.first_carrier_index == source_index
                        && pair.second_carrier_index == chord_index)
            })
            .expect("the overlapping chord/source bounds must schedule the pair");
        assert!(!context.authored_carriers_are_adjacent(pair));
        let result = context
            .pair_result(pair)
            .expect("nonadjacent general source/chord replay must complete");
        assert!(
            result.blockers.is_empty(),
            "nonadjacent general source/chord result: {result:?}"
        );
        assert_eq!(result.contacts.len(), 2);
        assert!(
            result
                .contacts
                .iter()
                .all(RegionPairContactEvidence::is_certified_transverse)
        );
    }
}

#[test]
fn independent_field_algebraic_chord_uses_general_boolean_pair_engine() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let second_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
        let x_axis = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                .expect("valid x axis"),
        ));
        let y_axis = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(0, 1))
                .expect("valid y axis"),
        ));
        let x_rational =
            RationalBezier2::try_from_subcurve(&x_axis).expect("valid rational x axis");
        let y_rational =
            RationalBezier2::try_from_subcurve(&y_axis).expect("valid rational y axis");
        let start = crate::tests::decided(
            exact_contact_point_evidence(&x_rational, &first_parameter, &policy)
                .expect("exact first endpoint"),
        );
        let end = crate::tests::decided(
            exact_contact_point_evidence(&y_rational, &second_parameter, &policy)
                .expect("exact second endpoint"),
        );
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(start, end, &policy)
                .expect("valid independent-field chord"),
        );
        let x_fragment = decided(
            x_axis
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&first_parameter),
                    &policy,
                )
                .expect("exact x-axis split"),
        )
        .fragments()[0]
            .clone();
        let y_fragment = decided(
            y_axis
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&second_parameter),
                    &policy,
                )
                .expect("exact y-axis split"),
        )
        .fragments()[0]
            .reversed()
            .expect("exact y-axis reversal");
        let chord_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(chord),
                y_fragment,
                x_fragment,
            ],
            &policy,
        )
        .expect("independent-field chord loop must close");

        let diagonal = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 1))
                .expect("valid diagonal"),
        ));
        let materialized = |curve| BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve,
        };
        let source_loop = CurveRegionBoundaryLoop2::new(
            vec![
                materialized(diagonal),
                materialized(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(-1, 1))
                            .expect("valid source-loop top"),
                    ),
                )),
                materialized(BezierSubcurve2::Quadratic(
                    QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(Point2::from_values(-1, 1), Point2::from_values(0, 0))
                            .expect("valid source-loop closure"),
                    ),
                )),
            ],
            &policy,
        )
        .expect("source loop must close");
        let chord_region = CurveRegion2::try_new_with_loop_topology(
            vec![chord_loop],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid independent-field chord region");
        let source_region = CurveRegion2::try_new_with_loop_topology(
            vec![source_loop],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid diagonal source region");
        let context = CurveRegionBooleanContext::try_new(&chord_region, &source_region, &policy)
            .expect("valid independent-field Boolean context");
        let chord_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| {
                carrier.operand == CurveRegionBooleanOperand2::First
                    && matches!(carrier.geometry, CurveSupport2::Line(_))
            })
            .expect("retained independent-field chord carrier");
        let source_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| {
                carrier.operand == CurveRegionBooleanOperand2::Second && carrier.fragment_index == 0
            })
            .expect("nonadjacent diagonal carrier");
        let pair = context
            .data
            .pairs
            .iter()
            .find(|pair| {
                (pair.first_carrier_index == chord_index
                    && pair.second_carrier_index == source_index)
                    || (pair.first_carrier_index == source_index
                        && pair.second_carrier_index == chord_index)
            })
            .expect("overlapping chord/diagonal bounds must schedule the pair");
        assert!(!context.authored_carriers_are_adjacent(pair));
        let result = context
            .pair_result(pair)
            .expect("general independent-field pair replay must complete");
        assert!(
            result.blockers.is_empty(),
            "independent-field pair result: {result:?}"
        );
        assert_eq!(result.contacts.len(), 1);
        assert!(result.contacts[0].is_certified_transverse());
        let topology = context.build_split_topology();
        assert!(
            topology.is_ok(),
            "independent-field contact must enter split topology: {topology:?}"
        );
        let booleans = context.build_boolean_regions();
        assert!(
            booleans.is_ok(),
            "independent-field contact must traverse all Booleans: {booleans:?}"
        );
    }
}

#[test]
fn independent_field_collinear_chord_overlap_enters_all_boolean_topology() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let half_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let third_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
        let apex = Point2::new(Real::zero(), -third.clone());
        let half_source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            apex.clone(),
            Point2::new(half.clone(), -third.clone()),
            Point2::new(Real::one(), third.clone()),
        ));
        let third_source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            apex,
            Point2::new(half, -third.clone()),
            Point2::new(Real::one(), Real::from(2_i8) * &third),
        ));
        let half_rational =
            RationalBezier2::try_from_subcurve(&half_source).expect("valid half source");
        let third_rational =
            RationalBezier2::try_from_subcurve(&third_source).expect("valid third source");
        let half_point = crate::tests::decided(
            exact_contact_point_evidence(&half_rational, &half_parameter, &policy)
                .expect("exact half endpoint"),
        );
        let third_point = crate::tests::decided(
            exact_contact_point_evidence(&third_rational, &third_parameter, &policy)
                .expect("exact third endpoint"),
        );
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(half_point.clone(), third_point.clone(), &policy)
                .expect("valid independent-field horizontal chord"),
        );
        let horizontal_source = rational_line(0, 1);
        let chord_geometry = CurveSupport2::Line(chord.clone());
        let source_geometry =
            CurveSupport2::Bezier(BezierSubcurve2::Rational(horizontal_source.clone()));
        let source_low =
            (Real::from(3_i8) / Real::from(5_i8)).expect("nonzero source-range denominator");
        let source_high =
            (Real::from(2_i8) / Real::from(3_i8)).expect("nonzero source-range denominator");
        let carriers = vec![
            RegionCarrier {
                operand: CurveRegionBooleanOperand2::First,
                loop_index: 0,
                fragment_index: 0,
                family: chord_geometry.family(),
                geometry: chord_geometry,
                start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
                end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
                reversed: false,
                filled_side_is_left: true,
                selected_fiber_endpoint_points: None,
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
                refined_bounds: Default::default(),
            },
            RegionCarrier {
                operand: CurveRegionBooleanOperand2::Second,
                loop_index: 0,
                fragment_index: 0,
                family: source_geometry.family(),
                geometry: source_geometry,
                start: CurveParameter2::from(BezierParameter2::Exact(source_low.clone())),
                end: CurveParameter2::from(BezierParameter2::Exact(source_high.clone())),
                reversed: false,
                filled_side_is_left: true,
                selected_fiber_endpoint_points: None,
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
                refined_bounds: Default::default(),
            },
        ];
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let clipping_context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers,
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: Vec::new(),
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let clipping_pair = RegionCarrierPair {
            first_carrier_index: 0,
            second_carrier_index: 1,
            context: RegionCarrierPairContext::AlgebraicChordPair {
                endpoint_contact: None,
            },
        };
        let clipping_result = clipping_context
            .pair_result(&clipping_pair)
            .expect("full collinear overlap must complete before clipping");
        let [raw_overlap] = clipping_result.overlaps.as_slice() else {
            panic!("expected one raw overlap, got {clipping_result:?}");
        };
        let (clipped_chord_range, clipped_source_range) = clipping_context
            .clipped_overlap_ranges(&clipping_pair, raw_overlap)
            .expect("authored source subrange must clip exactly")
            .expect("the authored source subrange lies inside the chord");
        assert!(clipped_chord_range.start().is_algebraic_chord());
        assert_eq!(
            clipped_source_range
                .start()
                .as_bezier_parameter()
                .expect("Bezier source range")
                .cmp_by_refinement_with_policy(
                    &BezierParameter2::Exact(source_high.clone()),
                    &policy,
                )
                .expect("exact source-range order"),
            Classification::Decided(Ordering::Equal)
        );
        assert_eq!(
            clipped_source_range
                .end()
                .as_bezier_parameter()
                .expect("Bezier source range")
                .cmp_by_refinement_with_policy(
                    &BezierParameter2::Exact(source_low.clone()),
                    &policy
                )
                .expect("exact source-range order"),
            Classification::Decided(Ordering::Equal)
        );
        let half_fragment = decided(
            half_source
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&half_parameter),
                    &policy,
                )
                .expect("exact half-source split"),
        )
        .fragments()[0]
            .clone();
        let third_fragment = decided(
            third_source
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&third_parameter),
                    &policy,
                )
                .expect("exact third-source split"),
        )
        .fragments()[0]
            .reversed()
            .expect("exact third-source reversal");
        let chord_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(chord),
                third_fragment,
                half_fragment,
            ],
            &policy,
        )
        .expect("independent-field chord loop must close");
        let chord_region = CurveRegion2::try_new_with_loop_topology(
            vec![chord_loop],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid independent-field chord region");

        let materialized_line =
            |start_x, start_y, end_x, end_y| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(
                        Point2::from_values(start_x, start_y),
                        Point2::from_values(end_x, end_y),
                    )
                    .expect("valid rectangle edge"),
                )),
            };
        let source_loop = CurveRegionBoundaryLoop2::new(
            vec![
                materialized_line(0, 0, 1, 0),
                materialized_line(1, 0, 1, 1),
                materialized_line(1, 1, 0, 1),
                materialized_line(0, 1, 0, 0),
            ],
            &policy,
        )
        .expect("source rectangle must close");
        let source_region = CurveRegion2::try_new_with_loop_topology(
            vec![source_loop],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid source rectangle");

        let intersections = chord_region
            .intersect_region_with_policy(&source_region, &policy)
            .expect("collinear chord/source intersection must complete")
            .into_value();
        assert!(intersections.is_complete(), "{intersections:?}");
        assert_eq!(intersections.overlaps().len(), 1, "{intersections:?}");
        assert_eq!(
            intersections.overlaps()[0].overlap().orientation(),
            CurveOverlapOrientation2::Reversed
        );
        assert!(
            intersections.overlaps()[0]
                .overlap()
                .first_range()
                .start()
                .is_algebraic_chord()
        );

        let booleans = chord_region.boolean_regions_with_policy(&source_region, &policy);
        assert!(
            booleans.is_ok(),
            "collinear chord overlap must enter all four Booleans: {booleans:?}"
        );
        let booleans = booleans.expect("complete collinear Booleans").into_value();
        assert!(booleans.intersection().is_empty());
        assert!(!booleans.union().is_empty());
        assert!(!booleans.difference().is_empty());
        assert!(!booleans.xor().is_empty());
    }
}

#[test]
fn algebraic_chord_pair_overlap_enters_region_intersection_evidence() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let horizontal = rational_line(0, 1);
        let first_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let second_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
        let first_point = crate::tests::decided(
            exact_contact_point_evidence(&horizontal, &first_parameter, &policy)
                .expect("exact first endpoint"),
        );
        let second_point = crate::tests::decided(
            exact_contact_point_evidence(&horizontal, &second_parameter, &policy)
                .expect("exact second endpoint"),
        );
        let first = decided(
            crate::BezierAlgebraicChord2::try_new(first_point, second_point, &policy)
                .expect("valid independent-field chord"),
        );
        let second = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(1, 0)),
                CurvePoint2::from(Point2::from_values(0, 0)),
                &policy,
            )
            .expect("valid represented containing chord"),
        );
        let carrier = |operand, chord: crate::BezierAlgebraicChord2| RegionCarrier {
            operand,
            loop_index: 0,
            fragment_index: 0,
            family: CurveFamily2::Line,
            start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
            end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
            geometry: CurveSupport2::Line(chord),
            reversed: false,
            filled_side_is_left: true,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
            refined_bounds: Default::default(),
        };
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers: vec![
                    carrier(CurveRegionBooleanOperand2::First, first),
                    carrier(CurveRegionBooleanOperand2::Second, second),
                ],
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: vec![RegionCarrierPair {
                    first_carrier_index: 0,
                    second_carrier_index: 1,
                    context: RegionCarrierPairContext::AlgebraicChordPair {
                        endpoint_contact: None,
                    },
                }],
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let pair_result = context
            .pair_result(&context.data.pairs[0])
            .expect("algebraic chord pair overlap must complete");
        assert!(pair_result.blockers.is_empty(), "{pair_result:?}");
        assert!(pair_result.contacts.is_empty(), "{pair_result:?}");
        let [overlap] = pair_result.overlaps.as_slice() else {
            panic!("expected one algebraic chord overlap: {pair_result:?}");
        };
        assert_eq!(overlap.orientation, CurveOverlapOrientation2::Same);
        assert!(overlap.first_range.start().is_algebraic_chord());
        assert!(overlap.second_range.start().is_algebraic_chord());

        let evidence = context
            .build_intersection_evidence()
            .expect("algebraic chord overlap must enter region evidence");
        assert!(evidence.is_complete(), "{evidence:?}");
        assert!(evidence.contacts().is_empty(), "{evidence:?}");
        assert_eq!(evidence.overlaps().len(), 1, "{evidence:?}");
        let published = evidence.overlaps()[0].clone();
        let CurveSupport2::Line(chord) = &context.data.carriers[0].geometry else {
            unreachable!()
        };
        let interior = CurveParameter2::from_algebraic_chord(
            chord
                .parameter_at_certified_support_point(
                    Point2::new((Real::from(2) / Real::from(3)).unwrap(), Real::zero()).into(),
                    &policy,
                )
                .unwrap(),
        );
        drop(context);
        drop(evidence);
        let limit = CurveParameterRange2::new_validated(
            published.overlap().first_range().start().clone(),
            interior,
        );
        let clipped = published
            .overlap()
            .restrict_with_policy(
                [limit.start().clone(), limit.end().clone()],
                [
                    published.overlap().second_range().start().clone(),
                    published.overlap().second_range().end().clone(),
                ],
                &policy,
            )
            .unwrap();
        assert_eq!(clipped.certainty, crate::CurveCertainty::Certified);
        let clipped = decided(clipped.value).expect("a positive interior chord restriction");
        for (a, b) in [
            (
                clipped.first_range().start(),
                clipped.second_range().start(),
            ),
            (clipped.first_range().end(), clipped.second_range().end()),
        ] {
            let a = published
                .first()
                .curve()
                .point_at_with_policy(a, &policy)
                .unwrap()
                .into_value();
            let b = published
                .second()
                .curve()
                .point_at_with_policy(b, &policy)
                .unwrap()
                .into_value();
            assert_eq!(
                a.coincides_with(&b, &policy).value,
                Classification::Decided(true)
            );
        }
        for _ in 0..8 {
            assert_eq!(
                decided(
                    clipped
                        .restrict_with_policy(
                            [
                                published.overlap().first_range().start().clone(),
                                published.overlap().first_range().end().clone()
                            ],
                            [
                                published.overlap().second_range().start().clone(),
                                published.overlap().second_range().end().clone()
                            ],
                            &policy
                        )
                        .unwrap()
                        .into_value()
                )
                .unwrap(),
                clipped
            );
        }
    }
}

#[test]
fn algebraic_chord_exact_linear_bezier_pair_replays_all_line_relations() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::new(Real::from(2_i8).sqrt().unwrap(), Real::zero())),
                CurvePoint2::from(Point2::from_values(4, 0)),
                &policy,
            )
            .expect("valid exact-field chord"),
        );
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let evaluate = |line: LineSeg2| {
            let chord_geometry = CurveSupport2::Line(chord.clone());
            let curve_geometry = CurveSupport2::Bezier(BezierSubcurve2::Quadratic(
                QuadraticBezier2::from_line_segment(line),
            ));
            let context = CurveRegionBooleanContext {
                data: CurveRegionBooleanContextData {
                    first: &empty_first,
                    second: &empty_second,
                    policy,
                    carriers: vec![
                        RegionCarrier {
                            operand: CurveRegionBooleanOperand2::First,
                            loop_index: 0,
                            fragment_index: 0,
                            family: chord_geometry.family(),
                            geometry: chord_geometry,
                            start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
                            end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
                            reversed: false,
                            filled_side_is_left: true,
                            selected_fiber_endpoint_points: None,
                            image_is_injective: OnceLock::new(),
                            bounds: OnceLock::new(),
                            refined_bounds: Default::default(),
                        },
                        RegionCarrier {
                            operand: CurveRegionBooleanOperand2::Second,
                            loop_index: 0,
                            fragment_index: 0,
                            family: curve_geometry.family(),
                            geometry: curve_geometry,
                            start: CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                            end: CurveParameter2::from(BezierParameter2::Exact(Real::one())),
                            reversed: false,
                            filled_side_is_left: true,
                            selected_fiber_endpoint_points: None,
                            image_is_injective: OnceLock::new(),
                            bounds: OnceLock::new(),
                            refined_bounds: Default::default(),
                        },
                    ],
                    first_carrier_count: 1,
                    authored_carrier_pair_count: 1,
                    pairs: vec![RegionCarrierPair {
                        first_carrier_index: 0,
                        second_carrier_index: 1,
                        context: RegionCarrierPairContext::AlgebraicChordPair {
                            endpoint_contact: None,
                        },
                    }],
                    regularization_fill_rule: None,
                    strict_line_image_only: OnceLock::new(),
                    operand_bounds: std::array::from_fn(|_| OnceLock::new()),
                },
            };
            context
                .pair_result(&context.data.pairs[0])
                .expect("the direct chord/linear-Bezier relation must complete")
        };

        for (line, orientation) in [
            (
                LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(5, 0)).unwrap(),
                CurveOverlapOrientation2::Same,
            ),
            (
                LineSeg2::try_new(Point2::from_values(5, 0), Point2::from_values(2, 0)).unwrap(),
                CurveOverlapOrientation2::Reversed,
            ),
        ] {
            let result = evaluate(line);
            assert!(result.blockers.is_empty(), "{result:?}");
            assert!(result.contacts.is_empty(), "{result:?}");
            let [overlap] = result.overlaps.as_slice() else {
                panic!("expected one exact line overlap: {result:?}");
            };
            assert_eq!(overlap.orientation, orientation);
            assert!(overlap.first_range.start().is_algebraic_chord());
            assert!(overlap.second_range.start().scalar().is_some());
        }

        let crossing = evaluate(
            LineSeg2::try_new(Point2::from_values(3, -1), Point2::from_values(3, 1)).unwrap(),
        );
        assert!(crossing.blockers.is_empty(), "{crossing:?}");
        assert!(crossing.overlaps.is_empty(), "{crossing:?}");
        let [contact] = crossing.contacts.as_slice() else {
            panic!("expected one exact transverse contact: {crossing:?}");
        };
        assert!(contact.certified_transverse);
        assert_eq!(contact.tangent_cross_sign, Some(RealSign::Positive));
        assert!(contact.first_parameter.is_algebraic_chord());
        assert!(contact.second_parameter.scalar().is_some());

        let disjoint = evaluate(
            LineSeg2::try_new(Point2::from_values(2, 1), Point2::from_values(5, 1)).unwrap(),
        );
        assert!(disjoint.contacts.is_empty(), "{disjoint:?}");
        assert!(disjoint.overlaps.is_empty(), "{disjoint:?}");
        assert!(disjoint.blockers.is_empty(), "{disjoint:?}");
    }
}

#[test]
fn parallel_arc_contacts_use_the_requested_exterior_range() {
    let quarter = (Real::one() / Real::from(4)).unwrap();
    let parallel = QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    )
    .parallel_left(quarter.clone())
    .unwrap();
    let arc = crate::CircularArc2::try_from_center(
        Point2::new(Real::from(3), quarter.clone()),
        Point2::new(Real::from(2), Real::one() + &quarter),
        Point2::new(Real::from(2), quarter),
        false,
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let empty = CurveRegion2::empty();
        let context = CurveRegionBooleanContext::try_new_unary(&empty, &policy).unwrap();
        let curve = RationalBezier2::from(
            arc.rational_bezier_decomposition_with_policy(&policy)
                .unwrap()
                .into_value()
                .spans()[0]
                .curve()
                .clone(),
        );
        for reversed in [false, true] {
            let curve = BezierSubcurve2::Rational(if reversed {
                curve.reversed()
            } else {
                curve.clone()
            });
            for (start, end, expected_count) in [(0, 1, 0), (2, 4, 1), (4, 5, 0)] {
                let range = CurveParameterRange2::new_validated(
                    Real::from(start).into(),
                    Real::from(end).into(),
                );
                let result = decided(
                    context
                        .parallel_arc_pair_result(&parallel, &range, &curve, true)
                        .unwrap(),
                )
                .expect("the finite circle incidence must decide");
                assert!(result.blockers.is_empty());
                assert!(result.overlaps.is_empty());
                assert_eq!(result.contacts.len(), expected_count);
                if let Some(contact) = result.contacts.first() {
                    for (actual, expected) in [
                        (&contact.first_parameter, Real::from(3)),
                        (&contact.second_parameter, Real::from(u8::from(reversed))),
                    ] {
                        assert_eq!(
                            actual.cmp_by_refinement(&expected.into(), &policy).unwrap(),
                            Classification::Decided(Ordering::Equal)
                        );
                    }
                    assert!(contact.certified_transverse);
                    assert_eq!(
                        contact.tangent_cross_sign,
                        Some(if reversed {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        })
                    );
                }
            }
        }
    }
}

#[test]
fn represented_parallel_arc_tangencies_retain_certified_crossing_evidence() {
    let parallel = QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    )
    .parallel_left(Real::one())
    .unwrap();
    let arc = crate::CircularArc2::try_from_center(
        Point2::from_values(0, 1),
        Point2::from_values(1, 2),
        Point2::from_values(0, 2),
        false,
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let curve = RationalBezier2::from(
            arc.rational_bezier_decomposition_with_policy(&policy)
                .unwrap()
                .into_value()
                .spans()[0]
                .curve()
                .clone(),
        );
        for reversed in [false, true] {
            for parallel_is_first in [false, true] {
                let curve = BezierSubcurve2::Rational(if reversed {
                    curve.reversed()
                } else {
                    curve.clone()
                });
                let outcome = crate::policy::resolve_certified_value(&policy, |policy| {
                    let empty = CurveRegion2::empty();
                    let context = CurveRegionBooleanContext::try_new_unary(&empty, policy).unwrap();
                    decided(
                        context
                            .parallel_arc_pair_result(
                                &parallel,
                                &CurveParameterRange2::unit(),
                                &curve,
                                parallel_is_first,
                            )
                            .unwrap(),
                    )
                    .expect("the represented tangent contact is exact")
                });
                assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                assert!(outcome.value.overlaps.is_empty());
                assert!(outcome.value.blockers.is_empty());
                let [contact] = outcome.value.contacts.as_slice() else {
                    panic!("the horizontal parallel touches this circle once");
                };
                assert!(!contact.certified_transverse);
                assert_eq!(contact.tangent_cross_sign, Some(RealSign::Zero));
                let (source, circle) = if parallel_is_first {
                    (&contact.first_parameter, &contact.second_parameter)
                } else {
                    (&contact.second_parameter, &contact.first_parameter)
                };
                assert_eq!(
                    source
                        .cmp_by_refinement(&Real::zero().into(), &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal)
                );
                assert_eq!(
                    circle
                        .cmp_by_refinement(&Real::from(u8::from(reversed)).into(), &policy)
                        .unwrap(),
                    Classification::Decided(Ordering::Equal)
                );
            }
        }
    }
}

#[test]
fn parallel_arc_contacts_retain_conic_parameters_across_elevation_and_reversal() {
    // The inner parallel of (2t,t^2) starts inside the unit circle and
    // crosses its first quadrant once, with both coordinates increasing.
    // Its intersection parameter requires a selected algebraic root.
    let parallel = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 0),
        Point2::from_values(2, 1),
    )
    .parallel_left((Real::one() / Real::from(4_i8)).unwrap())
    .unwrap();
    let arc = crate::CircularArc2::try_from_center(
        Point2::from_values(1, 0),
        Point2::from_values(0, 1),
        Point2::from_values(0, 0),
        false,
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let empty = CurveRegion2::empty();
        let context = CurveRegionBooleanContext::try_new_unary(&empty, &policy).unwrap();
        let conic = arc
            .rational_bezier_decomposition_with_policy(&policy)
            .unwrap()
            .into_value()
            .spans()[0]
            .curve()
            .clone();
        // Reverse the retained chart itself: a fresh decomposition of
        // the reversed arc chooses different rational endpoint weights.
        let reversed = decided(
            RationalBezier2::from(conic.clone())
                .reversed()
                .materialized_quadratic_representative(&policy)
                .unwrap(),
        )
        .unwrap();
        let conics = [conic, reversed];
        let mut reference: Option<(CurveParameter2, CurveParameter2, CurvePoint2)> = None;
        for (reversed, parallel_is_first, cross) in [
            (false, true, RealSign::Positive),
            (false, false, RealSign::Negative),
            (true, true, RealSign::Negative),
            (true, false, RealSign::Positive),
        ] {
            let conic = conics[usize::from(reversed)].clone();
            let elevated = RationalBezier2::from(conic.clone())
                .elevated_to_degree(4)
                .unwrap();
            for curve in [
                BezierSubcurve2::RationalQuadratic(conic),
                BezierSubcurve2::Rational(elevated),
            ] {
                let Classification::Decided(Some(result)) = context
                    .parallel_arc_pair_result(
                        &parallel,
                        &CurveParameterRange2::unit(),
                        &curve,
                        parallel_is_first,
                    )
                    .unwrap()
                else {
                    panic!("the exact conic contact must retain its local parameter");
                };
                assert!(result.overlaps.is_empty());
                assert!(result.blockers.is_empty());
                let [contact] = result.contacts.as_slice() else {
                    panic!("the monotone parallel crosses the quarter circle once");
                };
                assert!(contact.certified_transverse);
                assert_eq!(contact.tangent_cross_sign, Some(cross));
                let (source, target) = if parallel_is_first {
                    (&contact.first_parameter, &contact.second_parameter)
                } else {
                    (&contact.second_parameter, &contact.first_parameter)
                };
                assert!(source.scalar().is_none());
                assert!(
                    target.as_bezier_parameter().is_none(),
                    "the conic cut must keep the original point field without global projection"
                );
                for parameter in [source, target] {
                    for (boundary, order) in [
                        (Real::zero(), Ordering::Greater),
                        (Real::one(), Ordering::Less),
                    ] {
                        assert_eq!(
                            parameter
                                .cmp_by_refinement(&boundary.into(), &policy)
                                .unwrap(),
                            Classification::Decided(order)
                        );
                    }
                }
                let point = contact
                    .point
                    .as_ref()
                    .expect("the selected point is retained");
                if let Some((original_source, original_target, original_point)) = &reference {
                    assert_eq!(
                        source.cmp_by_refinement(original_source, &policy).unwrap(),
                        Classification::Decided(Ordering::Equal)
                    );
                    let expected_target = if reversed {
                        original_target.unit_complement().unwrap()
                    } else {
                        original_target.clone()
                    };
                    assert_eq!(
                        target.cmp_by_refinement(&expected_target, &policy).unwrap(),
                        Classification::Decided(Ordering::Equal),
                        "reversed={reversed}, parallel_is_first={parallel_is_first}"
                    );
                    assert_eq!(
                        point.same_point(original_point, &policy),
                        Classification::Decided(true)
                    );
                } else {
                    reference = Some((source.clone(), target.clone(), point.clone()));
                }
            }
        }
    }
}

#[test]
fn extended_conic_retains_certified_tangency_in_its_interior() {
    // x = 4t-2, y = -1+x^2+x^4 touches the unit circle at (0,-1)
    // and crosses it twice above the x axis. The latter contacts keep
    // this query on the selected-root path even though the touch is exact.
    let source = RationalBezier2::try_new(
        vec![
            Point2::from_values(-2, 19),
            Point2::from_values(-1, -17),
            Point2::new(
                Real::zero(),
                (Real::from(41_i8) / Real::from(3_i8)).unwrap(),
            ),
            Point2::from_values(1, -17),
            Point2::from_values(2, 19),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let parallel =
        BezierParallel2::from_source(crate::BezierParallelSource2::Rational(source), Real::zero());
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let point = Point2::from_values(0, -1);
    let center = Point2::from_values(0, 0);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let incidence = decided(
            parallel
                .circle_incidence(
                    &center,
                    &Real::one(),
                    &CurveParameterRange2::unit(),
                    &[(half.clone(), 2)],
                    &policy,
                )
                .unwrap(),
        );
        assert_eq!(incidence.len(), 3);
        assert!(
            incidence
                .iter()
                .any(|(parameter, _)| parameter.scalar().is_none())
        );
        let base = crate::CircularArc2::try_from_center(
            point.clone(),
            Point2::from_values(1, 0),
            center.clone(),
            false,
        )
        .unwrap()
        .rational_bezier_decomposition_with_policy(&policy)
        .unwrap()
        .into_value()
        .spans()[0]
            .curve()
            .clone();
        let circle = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
            center: center.clone(),
            radius_squared: Real::one(),
            tangent_contacts: Some(Arc::from([
                crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(
                    crate::rational_bezier::RationalQuadraticParallelCircleContact2 {
                        parallel: parallel.clone(),
                        parameter: half.clone(),
                        point: point.clone(),
                        eliminant_root_multiplicity: 2,
                    },
                ),
            ])),
        });
        let curve = RationalBezier2::from(base.clone().with_retained_conic_provenance(
            base.retained_implicit_quadratic_conic().cloned(),
            Some(circle),
        ));
        // The old start is now the interior parameter 1/3. The support
        // certificate survives extension, but cannot decide span ownership.
        let extended = decided(
            curve
                .subcurve_between_affine_exact(&(-half.clone()), &Real::one(), &policy)
                .unwrap(),
        );
        assert!(extended.retained_circular_conic().is_some());
        let empty = CurveRegion2::empty();
        let context = CurveRegionBooleanContext::try_new_unary(&empty, &policy).unwrap();
        let result = decided(
            context
                .parallel_arc_pair_result(
                    &parallel,
                    &CurveParameterRange2::unit(),
                    &BezierSubcurve2::Rational(extended),
                    true,
                )
                .unwrap(),
        )
        .expect("the extended quadratic chart must decide its finite contacts");
        assert!(result.overlaps.is_empty());
        assert!(result.blockers.is_empty());
        let [contact] = result.contacts.as_slice() else {
            panic!("only the interior tangency belongs to the extended conic");
        };
        assert!(!contact.certified_transverse);
        assert_eq!(contact.tangent_cross_sign, Some(RealSign::Zero));
        for (actual, expected) in [
            (&contact.first_parameter, half.clone()),
            (
                &contact.second_parameter,
                (Real::one() / Real::from(3_i8)).unwrap(),
            ),
        ] {
            assert_eq!(
                actual.cmp_by_refinement(&expected.into(), &policy).unwrap(),
                Classification::Decided(Ordering::Equal)
            );
        }
        assert_eq!(
            contact
                .point
                .as_ref()
                .unwrap()
                .same_point(&point.clone().into(), &policy),
            Classification::Decided(true)
        );
    }
}

#[test]
fn authored_parallel_support_contact_classifies_perpendicular_and_tangent_joins() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                .expect("valid horizontal source"),
        );
        let parallel = BezierParallel2::from_source(
            crate::BezierParallelSource2::Quadratic(source),
            Real::zero(),
        );
        let analytic =
            BezierSplitFragment2::AnalyticParallel(BezierParallelFragment2::from_certified_range(
                parallel.clone(),
                BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                false,
            ));
        let line = |start_x, start_y, end_x, end_y| BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(
                    Point2::from_values(start_x, start_y),
                    Point2::from_values(end_x, end_y),
                )
                .expect("valid rectangle edge"),
            )),
        };
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                analytic,
                line(1, 0, 1, 1),
                line(1, 1, 0, 1),
                line(0, 1, 0, 0),
            ],
            &policy,
        )
        .expect("analytic rectangle must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid analytic rectangle");
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid unary context");
        let parallel_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| matches!(carrier.geometry, CurveSupport2::Parallel(_)))
            .expect("the analytic carrier must be retained");
        let line_index = context
            .data
            .carriers
            .iter()
            .position(|carrier| carrier.fragment_index == 1)
            .expect("the following line carrier must be retained");
        let pair = RegionCarrierPair {
            first_carrier_index: parallel_index,
            second_carrier_index: line_index,
            context: RegionCarrierPairContext::ParallelPair,
        };
        let parallel_carrier = &context.data.carriers[parallel_index];
        let range = CurveParameterRange2::new_validated(
            parallel_carrier.start.clone(),
            parallel_carrier.end.clone(),
        );
        let perpendicular = context
            .authored_parallel_support_contact(
                &pair,
                &parallel,
                parallel_index,
                &Real::zero(),
                &Real::one(),
                Some(&range),
            )
            .expect("the perpendicular relation must decide")
            .expect("the perpendicular authored contact must be retained");
        assert_eq!(perpendicular, (Real::one(), RealSign::Negative));
        let tangent = context
            .authored_parallel_support_contact(
                &pair,
                &parallel,
                parallel_index,
                &Real::one(),
                &Real::zero(),
                Some(&range),
            )
            .expect("the tangent relation must decide")
            .expect("the tangent authored contact must be retained");
        assert_eq!(tangent, (Real::one(), RealSign::Zero));
    }
}

fn chord_parallel_pair_evidence(
    chord: crate::BezierAlgebraicChord2,
    parallel: BezierParallel2,
    range: CurveParameterRange2,
    policy: CurveContext,
) -> (RegionPairResult, CurveRegionIntersectionResult2) {
    let empty_first = CurveRegion2::empty();
    let empty_second = CurveRegion2::empty();
    // Region carriers store increasing bounds and a separate traversal
    // flag, while the public range passed to this fixture is oriented.
    let parallel_reversed = range
        .start()
        .cmp_by_refinement(range.end(), &policy)
        .unwrap()
        == Classification::Decided(Ordering::Greater);
    let [parallel_start, parallel_end] = decided(range.ordered_endpoints(&policy).unwrap());
    let chord_geometry = CurveSupport2::Line(chord.clone());
    let ordered_range =
        CurveParameterRange2::new_validated(parallel_start.clone(), parallel_end.clone());
    // Construct the same retained fragment accepted by production. A
    // selected range owns its one-sided endpoint evidence; dropping it
    // would manufacture an incomplete private carrier state.
    let fragment = match (
        parallel_start.as_bezier_parameter(),
        parallel_end.as_bezier_parameter(),
    ) {
        (Some(start), Some(end)) => BezierSplitFragment2::AnalyticParallel(
            crate::BezierParallelFragment2::from_certified_range(
                parallel.clone(),
                BezierParameterRange2::new_validated(start.clone(), end.clone()),
                parallel_reversed,
            ),
        ),
        _ => {
            let start = decided(
                parallel
                    .point_evidence_on_regular_range(parallel_start, &ordered_range, &policy)
                    .unwrap(),
            );
            let end = decided(
                parallel
                    .point_evidence_on_regular_range(parallel_end, &ordered_range, &policy)
                    .unwrap(),
            );
            let fragment = BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::AnalyticParallel(parallel),
                    ordered_range,
                    start,
                    end,
                ),
            );
            if parallel_reversed {
                fragment.reversed().unwrap()
            } else {
                fragment
            }
        }
    };
    let context = CurveRegionBooleanContext {
        data: CurveRegionBooleanContextData {
            first: &empty_first,
            second: &empty_second,
            policy,
            carriers: vec![
                RegionCarrier {
                    operand: CurveRegionBooleanOperand2::First,
                    loop_index: 0,
                    fragment_index: 0,
                    family: chord_geometry.family(),
                    geometry: chord_geometry,
                    start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
                    end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
                    reversed: false,
                    filled_side_is_left: true,
                    selected_fiber_endpoint_points: None,
                    image_is_injective: OnceLock::new(),
                    bounds: OnceLock::new(),
                    refined_bounds: Default::default(),
                },
                build_parameterized_carrier(
                    &fragment,
                    CurveRegionBooleanOperand2::Second,
                    0,
                    0,
                    true,
                ),
            ],
            first_carrier_count: 1,
            authored_carrier_pair_count: 1,
            pairs: vec![RegionCarrierPair {
                first_carrier_index: 0,
                second_carrier_index: 1,
                context: RegionCarrierPairContext::AlgebraicChordPair {
                    endpoint_contact: None,
                },
            }],
            regularization_fill_rule: None,
            strict_line_image_only: OnceLock::new(),
            operand_bounds: std::array::from_fn(|_| OnceLock::new()),
        },
    };
    let result = context
        .pair_result(&context.data.pairs[0])
        .expect("the chord/analytic-parallel pair must complete");
    let evidence = context
        .build_intersection_evidence()
        .expect("the chord/analytic-parallel evidence must complete");
    assert_chord_parallel_evidence_replays(&evidence, &policy);
    (result, evidence)
}

fn assert_chord_parallel_evidence_replays(
    evidence: &CurveRegionIntersectionResult2,
    policy: &CurveContext,
) {
    let replay = |first, second| {
        let first = evidence_carrier_point(evidence, true, first, policy);
        let second = evidence_carrier_point(evidence, false, second, policy);
        let equality = first.coincides_with(&second, policy);
        assert_eq!(equality.certainty, crate::CurveCertainty::Certified);
        assert_eq!(equality.value, Classification::Decided(true));
    };
    for contact in evidence.contacts() {
        replay(contact.first_parameter(), contact.second_parameter());
    }
    for overlap in evidence.overlaps() {
        replay(
            overlap.overlap().first_range().start(),
            overlap.overlap().second_range().start(),
        );
        replay(
            overlap.overlap().first_range().end(),
            overlap.overlap().second_range().end(),
        );
    }
}

fn evidence_carrier_point(
    evidence: &CurveRegionIntersectionResult2,
    first: bool,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurvePoint2 {
    let (a, b) = if let Some(contact) = evidence.contacts().first() {
        (contact.first(), contact.second())
    } else {
        let overlap = &evidence.overlaps()[0];
        (overlap.first(), overlap.second())
    };
    let curve = if first { a.curve() } else { b.curve() };
    let point = curve.point_at_with_policy(parameter, policy).unwrap();
    assert_eq!(point.certainty, crate::CurveCertainty::Certified);
    point.value
}

#[test]
fn algebraic_chord_parallel_boolean_keeps_exterior_and_selected_ranges() {
    let quarter = (Real::one() / Real::from(4)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let retained = sqrt_half_parameter(&policy);
        for kind in 0..3 {
            let source = if kind == 0 {
                QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                        .unwrap(),
                )
            } else {
                QuadraticBezier2::new(
                    Point2::from_values(4, 0),
                    Point2::from_values(2, 0),
                    Point2::from_values(1, 0),
                )
            }
            .parallel_left(quarter.clone())
            .unwrap();
            for reversed in [false, true] {
                let parallel = if reversed {
                    source.reversed()
                } else {
                    source.clone()
                };
                assert!(matches!(
                    parallel.exact_rational_parallel_component(&policy).unwrap(),
                    Classification::Decided(Some(_))
                ));
                for selected in [false, true] {
                    let parameter = |value| {
                        let value = if reversed {
                            Real::one() - Real::from(value)
                        } else {
                            Real::from(value)
                        };
                        if selected {
                            CurveParameter2::from_selected_fiber(
                                exact_selected_fiber_parameter_for_test(
                                    retained.clone(),
                                    value,
                                    &policy,
                                ),
                            )
                        } else {
                            CurveParameter2::from(value)
                        }
                    };
                    let (start, end, range) = match kind {
                        0 => (
                            Point2::from_values(3, 0),
                            Point2::from_values(3, 1),
                            CurveParameterRange2::new_validated(parameter(2), parameter(4)),
                        ),
                        1 => (
                            Point2::from_values(1, 0),
                            Point2::from_values(1, 1),
                            CurveParameterRange2::new_validated(parameter(3), parameter(4)),
                        ),
                        _ => (
                            Point2::new(Real::from(-1), quarter.clone()),
                            Point2::new(Real::from(2), quarter.clone()),
                            CurveParameterRange2::new_validated(parameter(2), parameter(3)),
                        ),
                    };
                    let chord = decided(
                        crate::BezierAlgebraicChord2::try_new(start.into(), end.into(), &policy)
                            .unwrap(),
                    );
                    let (result, evidence) = chord_parallel_pair_evidence(
                        chord,
                        parallel.clone(),
                        range.clone(),
                        policy,
                    );
                    assert!(
                        result.blockers.is_empty(),
                        "kind={kind}, selected={selected}, reversed={reversed}: {result:?}"
                    );
                    assert!(evidence.is_complete(), "{evidence:?}");
                    let (first, second) = if let Some(contact) = evidence.contacts().first() {
                        (contact.first().curve(), contact.second().curve())
                    } else {
                        let overlap = &evidence.overlaps()[0];
                        (overlap.first().curve(), overlap.second().curve())
                    };
                    for (a, b) in [(first, second), (second, first)] {
                        let common = a.intersect_curve_with_policy(b, &policy).unwrap();
                        assert_eq!(common.certainty, crate::CurveCertainty::Certified);
                        assert!(common.value.is_complete(), "{common:?}");
                        assert_eq!(common.value.contacts().len(), evidence.contacts().len());
                        assert_eq!(common.value.overlaps().len(), evidence.overlaps().len());
                        for contact in common.value.contacts() {
                            for (curve, location) in [(a, contact.first()), (b, contact.second())] {
                                let parameter =
                                    decided(location.parameter_with_policy(&policy).unwrap());
                                let point =
                                    curve.point_at_with_policy(&parameter, &policy).unwrap();
                                assert_eq!(point.certainty, crate::CurveCertainty::Certified);
                                assert_eq!(
                                    point.value.same_point(contact.point(), &policy),
                                    Classification::Decided(true)
                                );
                            }
                        }
                    }
                    if kind == 2 {
                        assert!(result.contacts.is_empty());
                        let [overlap] = evidence.overlaps() else {
                            panic!("one retained branch must overlap: {evidence:?}")
                        };
                        assert_eq!(overlap.overlap().second_range(), &range);
                    } else {
                        assert!(result.overlaps.is_empty());
                        let [contact] = evidence.contacts() else {
                            panic!(
                                "the exterior contact must be retained: kind={kind}, selected={selected}, reversed={reversed}: {result:?}; {evidence:?}"
                            )
                        };
                        assert_eq!(
                            contact
                                .second_parameter()
                                .cmp_by_refinement(&parameter(3), &policy)
                                .unwrap(),
                            Classification::Decided(Ordering::Equal)
                        );
                        assert!(contact.is_certified_transverse());
                    }
                }
            }
        }
    }
}

#[test]
fn algebraic_chord_analytic_parallel_pair_replays_contacts_and_overlap() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let evaluate =
            |chord, parallel, range| chord_parallel_pair_evidence(chord, parallel, range, policy);

        let crossing_chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(-3, 1)),
                CurvePoint2::from(Point2::from_values(3, 1)),
                &policy,
            )
            .unwrap(),
        );
        let parabola = BezierParallel2::from_source(
            crate::BezierParallelSource2::Quadratic(QuadraticBezier2::new(
                Point2::from_values(-2, 0),
                Point2::from_values(0, 4),
                Point2::from_values(2, 0),
            )),
            Real::zero(),
        );
        let (crossings, _) = evaluate(
            crossing_chord,
            parabola,
            CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                Real::zero(),
                Real::one(),
            )),
        );
        assert!(crossings.blockers.is_empty(), "{crossings:?}");
        assert!(crossings.overlaps.is_empty(), "{crossings:?}");
        assert_eq!(crossings.contacts.len(), 2, "{crossings:?}");
        assert!(crossings.contacts.iter().all(|contact| {
            contact.first_parameter.is_algebraic_chord()
                && contact.second_parameter.as_bezier_parameter().is_some()
                && contact.certified_transverse
        }));

        // Two independently selected roots define this diagonal chord.
        // A foreign conjugate tuple collapses the chord and zeros the
        // global norm, while the authored tuple has the exact local root
        // t=1/2. The CurveRegion pair must consume the same retained
        // chord/parallel kernel result without a second Boolean fallback.
        let selected_parameter = |lower: Real, upper: Real| {
            let polynomial = decided(
                crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
                    vec![Real::one(), Real::from(-8_i8), Real::from(8_i8)],
                    &policy,
                )
                .expect("valid independent endpoint polynomial"),
            );
            let interval = decided(
                crate::BezierParameterInterval::try_new_with_policy(lower, upper, &policy)
                    .expect("valid independent endpoint interval"),
            );
            BezierParameter2::Algebraic(decided(
                BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy)
                    .expect("isolated independent endpoint"),
            ))
        };
        let diagonal = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 1)],
            vec![Real::one(), Real::one()],
        )
        .expect("valid diagonal parameter source");
        let selected_point = |parameter: &BezierParameter2| {
            crate::tests::decided(
                crate::rational_bezier_general::exact_contact_point_evidence(
                    &diagonal, parameter, &policy,
                )
                .expect("exact independent endpoint"),
            )
        };
        let selected_start = selected_parameter(
            (Real::one() / Real::from(8_i8)).expect("nonzero denominator"),
            (Real::one() / Real::from(4_i8)).expect("nonzero denominator"),
        );
        let selected_end = selected_parameter(
            (Real::from(3_i8) / Real::from(4_i8)).expect("nonzero denominator"),
            (Real::from(7_i8) / Real::from(8_i8)).expect("nonzero denominator"),
        );
        let selected_chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                selected_point(&selected_start),
                selected_point(&selected_end),
                &policy,
            )
            .expect("valid independent diagonal chord"),
        );
        let selected_parallel = BezierParallel2::from_source(
            crate::BezierParallelSource2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(
                    Point2::new(
                        Real::zero(),
                        (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                    ),
                    Point2::new(
                        Real::one(),
                        (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                    ),
                )
                .expect("valid horizontal target"),
            )),
            Real::zero(),
        );
        let (selected_contact, selected_evidence) = evaluate(
            selected_chord,
            selected_parallel,
            CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                Real::zero(),
                Real::one(),
            )),
        );
        assert!(selected_contact.blockers.is_empty(), "{selected_contact:?}");
        assert!(selected_contact.overlaps.is_empty(), "{selected_contact:?}");
        let [selected_contact] = selected_contact.contacts.as_slice() else {
            panic!("expected one selected-fiber contact: {selected_contact:?}");
        };
        assert_eq!(
            selected_contact
                .second_parameter
                .cmp_by_refinement(
                    &carrier_parameter(BezierParameter2::Exact(
                        (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                    )),
                    &policy,
                )
                .expect("the retained local root must compare exactly"),
            Classification::Decided(Ordering::Equal),
        );
        assert!(selected_evidence.is_complete(), "{selected_evidence:?}");

        let overlap_chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(-1, 0)),
                CurvePoint2::from(Point2::from_values(1, 0)),
                &policy,
            )
            .unwrap(),
        );
        let line_parallel = BezierParallel2::from_source(
            crate::BezierParallelSource2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(Point2::from_values(-2, 0), Point2::from_values(2, 0)).unwrap(),
            )),
            Real::zero(),
        );
        let (overlap, _) = evaluate(
            overlap_chord,
            line_parallel,
            CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                Real::zero(),
                Real::one(),
            )),
        );
        assert!(overlap.blockers.is_empty(), "{overlap:?}");
        assert!(overlap.contacts.is_empty(), "{overlap:?}");
        let [overlap] = overlap.overlaps.as_slice() else {
            panic!("expected one chord/parallel overlap: {overlap:?}");
        };
        assert_eq!(overlap.orientation, CurveOverlapOrientation2::Same);
        assert!(overlap.first_range.start().is_algebraic_chord());
        assert!(overlap.second_range.start().as_bezier_parameter().is_some());

        // P(t)=(2t-1)^2(-1,1) reverses at t=1/2. Its signed left
        // parallel has opposite line images on the two regular sides, so
        // no global PH component exists. The second branch nevertheless
        // coincides with this independently selected algebraic chord.
        let selected = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let selected_point = |start: Point2, end: Point2| {
            let source = RationalBezier2::try_new(vec![start, end], vec![Real::one(), Real::one()])
                .expect("valid selected line source");
            crate::tests::decided(
                exact_contact_point_evidence(&source, &selected, &policy)
                    .expect("exact selected line point"),
            )
        };
        let retracing_chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                selected_point(Point2::from_values(0, 0), Point2::from_values(-1, -1)),
                selected_point(Point2::from_values(-1, 1), Point2::from_values(-2, 0)),
                &policy,
            )
            .expect("valid independent retracing-line chord"),
        );
        assert!(retracing_chord.exact_line().is_none());
        assert!(
            retracing_chord
                .strict_provenance_support_line(&policy)
                .is_none()
        );
        let retracing_parallel = BezierParallel2::from_source(
            crate::BezierParallelSource2::Quadratic(QuadraticBezier2::new(
                Point2::from_values(-1, 1),
                Point2::from_values(1, -1),
                Point2::from_values(-1, 1),
            )),
            Real::one(),
        );
        assert!(matches!(
            retracing_parallel
                .exact_rational_parallel_component(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided(None),
        ));
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let work = || {
            evaluate(
                retracing_chord,
                retracing_parallel,
                CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                    half,
                    Real::one(),
                )),
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let (component, evidence) = hyperreal::dispatch_trace::with_recording(work);
        #[cfg(not(feature = "dispatch-trace"))]
        let (component, evidence) = work();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        assert!(component.blockers.is_empty(), "{component:?}");
        assert_eq!(component.overlaps.len(), 1, "{component:?}");
        assert!(evidence.is_complete(), "{evidence:?}");
        let [selected_overlap] = evidence.overlaps() else {
            panic!("only the selected regular branch may survive: {evidence:?}");
        };
        assert_eq!(
            selected_overlap.overlap().orientation(),
            CurveOverlapOrientation2::Same,
        );
        #[cfg(feature = "dispatch-trace")]
        {
            assert!(
                trace.path_count(
                    "hypercurve",
                    "analytic-parallel-rational-component",
                    "regularized-pythagorean-hodograph",
                ) > 0,
                "the selected branch must materialize structurally: {trace:?}",
            );
            assert!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "collinear-overlap-complete",
                ) > 0,
                "the rational overlap authority must publish the component: {trace:?}",
            );
        }

        let test_regularized_ph = || {
            // P'(t)=(2t-1)(1-t^2,2t) has no global polynomial speed
            // sheet, while its primitive quotient is the nonconstant PH
            // field (1-t^2,2t). The selected right side must enter the same
            // rational pair publisher and retain its authored parameter.
            let regularized_ph_source = RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::new(-(Real::one() / Real::from(4_i8)).unwrap(), Real::zero()),
                    Point2::new(
                        -(Real::one() / Real::from(3_i8)).unwrap(),
                        -(Real::one() / Real::from(6_i8)).unwrap(),
                    ),
                    Point2::new(
                        -(Real::one() / Real::from(6_i8)).unwrap(),
                        -(Real::one() / Real::from(6_i8)).unwrap(),
                    ),
                    Point2::new(
                        -(Real::one() / Real::from(6_i8)).unwrap(),
                        (Real::one() / Real::from(3_i8)).unwrap(),
                    ),
                ],
                vec![Real::one(); 5],
            )
            .expect("valid regularized PH source");
            let regularized_ph = regularized_ph_source
                .parallel_left(Real::one())
                .expect("valid regularized PH parallel");
            assert!(matches!(
                regularized_ph
                    .exact_rational_parallel_component(&CurveContext::STRICT)
                    .unwrap(),
                Classification::Decided(None),
            ));
            let three_quarters =
                (Real::from(3_i8) / Real::from(4_i8)).expect("nonzero denominator");
            let contact_point =
                match regularized_ph.point_at_with_policy(&three_quarters, &CurveContext::STRICT) {
                    Ok(Classification::Decided(point)) => point,
                    result => panic!("the PH contact point must evaluate exactly: {result:?}"),
                };
            let ph_chord = decided(
                crate::BezierAlgebraicChord2::try_new(
                    CurvePoint2::from(Point2::new(
                        contact_point.x() - Real::one(),
                        contact_point.y().clone(),
                    )),
                    CurvePoint2::from(Point2::new(
                        contact_point.x() + Real::one(),
                        contact_point.y().clone(),
                    )),
                    &policy,
                )
                .expect("valid regularized PH crossing chord"),
            );
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let work = || {
                evaluate(
                    ph_chord,
                    regularized_ph,
                    CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                        (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                        Real::one(),
                    )),
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let (ph_result, ph_evidence) = hyperreal::dispatch_trace::with_recording(work);
            #[cfg(not(feature = "dispatch-trace"))]
            let (ph_result, ph_evidence) = work();
            #[cfg(feature = "dispatch-trace")]
            let ph_trace = hyperreal::dispatch_trace::take_trace();
            assert!(ph_result.blockers.is_empty(), "{ph_result:?}");
            assert!(ph_result.overlaps.is_empty(), "{ph_result:?}");
            #[cfg(feature = "dispatch-trace")]
            assert!(
                !ph_result.contacts.is_empty(),
                "{ph_result:?}; trace: {ph_trace:?}"
            );
            #[cfg(not(feature = "dispatch-trace"))]
            assert!(!ph_result.contacts.is_empty(), "{ph_result:?}");
            assert!(ph_evidence.is_complete(), "{ph_evidence:?}");
            assert!(ph_evidence.contacts().iter().any(|contact| {
                contact
                    .second_parameter()
                    .as_bezier_parameter()
                    .is_some_and(|parameter| {
                        matches!(
                            parameter.same_value(
                                &BezierParameter2::Exact(three_quarters.clone()),
                                &policy,
                            ),
                            Ok(Classification::Decided(true))
                        )
                    })
            }));
            #[cfg(feature = "dispatch-trace")]
            {
                assert!(
                    ph_trace.path_count(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "analytic-parallel-certified-support",
                    ) > 0,
                    "the nonconstant PH branch must use the authoritative retained-support kernel: {ph_trace:?}",
                );
                for (operation, path) in [
                    (
                        "algebraic-chord-parallel-monotonicity",
                        "interior-singularity",
                    ),
                    (
                        "analytic-parallel-rational-component",
                        "regularized-pythagorean-hodograph",
                    ),
                    ("algebraic-chord-pair", "general-rational"),
                    (
                        "algebraic-chord-pair",
                        "analytic-parallel-strict-rational-component",
                    ),
                ] {
                    assert_eq!(
                        ph_trace.path_count("hypercurve", operation, path),
                        0,
                        "the retained-support answer must bypass superseded {operation}/{path} machinery: {ph_trace:?}",
                    );
                }
            }
        };

        // A Real coefficient can have unresolved zero status while the
        // authored hodograph still has exact rank one.  Keep the shared
        // coordinate expression intact through line incidence, restrict
        // the materialized rational image to its certified regular
        // branch, and recover its overlap without assigning a global
        // polynomial degree.
        let epsilon = Real::new(
            hyperreal::Rational::from_bigint_fraction(
                BigInt::from(1_u8),
                BigUint::from(1_u8) << 600,
            )
            .expect("valid dyadic epsilon"),
        );
        let opaque = epsilon.cos() - Real::one();
        assert_eq!(opaque.zero_status(), hyperreal::ZeroKnowledge::Unknown);
        let third = (Real::one() / Real::from(3_i8)).expect("nonzero denominator");
        let quarter = (Real::one() / Real::from(4_i8)).expect("nonzero denominator");
        let shoulder = -((&opaque + Real::one()) * quarter);
        let opaque_source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::new(shoulder.clone(), shoulder.clone()),
                Point2::new(-third.clone(), -third),
                Point2::new(shoulder.clone(), shoulder),
                Point2::from_values(0, 0),
            ],
            vec![Real::one(); 5],
        )
        .expect("valid opaque retracing source");
        let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).expect("nonzero denominator");
        let start_q = opaque_source
            .point_at_with_policy(&three_quarters, &CurveContext::STRICT)
            .expect("exact opaque source point")
            .x()
            .clone();
        let two_thirds = (Real::from(2_i8) / Real::from(3_i8)).expect("nonzero denominator");
        let wide_start_q = opaque_source
            .point_at_with_policy(&two_thirds, &CurveContext::STRICT)
            .expect("exact wide opaque source point")
            .x()
            .clone();
        let end_q = opaque_source.end().x().clone();
        let normal_parameter = sqrt_half_parameter(&policy);
        let normal_representation =
            crate::bezier_algebraic_image::parameter_representation(&normal_parameter, &policy);
        let endpoint = |q: Real, label| {
            CurvePoint2::from(
                RationalBezierAlgebraicPointImage2::from_retained_expression(
                    normal_parameter.clone(),
                    normal_representation.clone(),
                    vec![q.clone(), Real::from(-1_i8)],
                    vec![q, Real::one()],
                    vec![Real::one()],
                    label,
                ),
            )
        };
        let opaque_chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                endpoint(start_q, "opaque Boolean chord start"),
                endpoint(end_q.clone(), "opaque Boolean chord end"),
                &policy,
            )
            .expect("valid opaque algebraic chord"),
        );
        let algebraic_range_chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                endpoint(wide_start_q, "algebraic-range Boolean chord start"),
                endpoint(end_q, "algebraic-range Boolean chord end"),
                &policy,
            )
            .expect("valid algebraic-range chord"),
        );
        assert!(opaque_chord.exact_line().is_none());
        assert!(
            opaque_chord
                .strict_provenance_support_line(&policy)
                .is_none()
        );
        let opaque_parallel = opaque_source
            .parallel_left(Real::one())
            .expect("valid opaque analytic parallel");
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let work = || {
            evaluate(
                opaque_chord.clone(),
                opaque_parallel.clone(),
                CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                    (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                    Real::one(),
                )),
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let (opaque_component, opaque_evidence) = hyperreal::dispatch_trace::with_recording(work);
        #[cfg(not(feature = "dispatch-trace"))]
        let (opaque_component, opaque_evidence) = work();
        #[cfg(feature = "dispatch-trace")]
        let opaque_trace = hyperreal::dispatch_trace::take_trace();
        assert!(opaque_component.blockers.is_empty(), "{opaque_component:?}");
        assert!(opaque_component.contacts.is_empty(), "{opaque_component:?}");
        assert_eq!(opaque_component.overlaps.len(), 1, "{opaque_component:?}");
        assert!(opaque_evidence.is_complete(), "{opaque_evidence:?}");
        let [opaque_overlap] = opaque_evidence.overlaps() else {
            panic!("the selected opaque branch must publish one overlap: {opaque_evidence:?}");
        };
        assert_eq!(
            opaque_overlap.overlap().orientation(),
            CurveOverlapOrientation2::Same,
        );
        for (actual, expected) in [
            (
                opaque_overlap.overlap().second_range().start(),
                three_quarters.clone(),
            ),
            (opaque_overlap.overlap().second_range().end(), Real::one()),
        ] {
            assert_eq!(
                actual
                    .cmp_by_refinement(&CurveParameter2::from(expected), &policy)
                    .unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal),
                "the opaque overlap must retain the complete incident source range",
            );
        }
        #[cfg(feature = "dispatch-trace")]
        {
            // A learned scalar witness may settle the correlated projection
            // before retained-field replay. It must still avoid reconstructing
            // the two coordinates independently.
            assert_eq!(
                opaque_trace.path_count(
                    "hypercurve",
                    "algebraic-chord-point-linear-order",
                    "represented-cold-fallback",
                ),
                0
            );
            for (operation, path) in [
                (
                    "analytic-parallel-regularized-tangent",
                    "constant-direction-rank",
                ),
                (
                    "algebraic-chord-collinear-range",
                    "certified-regular-line-range",
                ),
                (
                    "algebraic-chord-collinear-endpoint",
                    "exact-monotone-inverse",
                ),
                (
                    "algebraic-chord-pair",
                    "certified-rational-support-collinear",
                ),
                ("algebraic-chord-pair", "collinear-overlap-complete"),
            ] {
                assert!(
                    opaque_trace.path_count("hypercurve", operation, path) > 0,
                    "the opaque Boolean must traverse {operation}/{path}: {opaque_trace:?}",
                );
            }
        }

        // The same exact range can arrive with retained root evidence or
        // directly represented irrational endpoints. Internal probes must
        // keep both forms in the collinear component authority.
        for algebraic_range_start in [
            BezierParameter2::Algebraic(normal_parameter.clone()),
            BezierParameter2::Exact((Real::one() / Real::from(2_i8)).unwrap().sqrt().unwrap()),
        ] {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let work = || {
                evaluate(
                    algebraic_range_chord.clone(),
                    opaque_parallel.clone(),
                    CurveParameterRange2::from_bezier_range(BezierParameterRange2::new_validated(
                        algebraic_range_start.clone(),
                        BezierParameter2::Exact(Real::one()),
                    )),
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let (algebraic_component, algebraic_evidence) =
                hyperreal::dispatch_trace::with_recording(work);
            #[cfg(not(feature = "dispatch-trace"))]
            let (algebraic_component, algebraic_evidence) = work();
            #[cfg(feature = "dispatch-trace")]
            let algebraic_trace = hyperreal::dispatch_trace::take_trace();
            assert!(
                algebraic_component.blockers.is_empty(),
                "{algebraic_component:?}",
            );
            assert!(
                algebraic_component.contacts.is_empty(),
                "{algebraic_component:?}",
            );
            assert_eq!(
                algebraic_component.overlaps.len(),
                1,
                "{algebraic_component:?}",
            );
            assert!(algebraic_evidence.is_complete(), "{algebraic_evidence:?}",);
            let [algebraic_overlap] = algebraic_evidence.overlaps() else {
                panic!(
                    "the algebraic regular branch must publish one overlap: {algebraic_evidence:?}"
                );
            };
            assert_eq!(
                algebraic_overlap.overlap().orientation(),
                CurveOverlapOrientation2::Same,
            );
            assert_eq!(
                algebraic_overlap
                    .overlap()
                    .second_range()
                    .start()
                    .as_bezier_parameter(),
                Some(&algebraic_range_start),
            );
            #[cfg(feature = "dispatch-trace")]
            for (operation, path) in [
                (
                    "algebraic-chord-collinear-range",
                    "certified-regular-line-range",
                ),
                (
                    "algebraic-chord-pair",
                    "certified-rational-support-collinear",
                ),
                ("algebraic-chord-pair", "collinear-overlap-complete"),
            ] {
                assert!(
                    algebraic_trace.path_count("hypercurve", operation, path) > 0,
                    "the algebraic range must traverse {operation}/{path}: {algebraic_trace:?}",
                );
            }
        }
        test_regularized_ph();
    }
}

#[test]
fn strict_interior_algebraic_chord_pair_contact_splits_both_carriers() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let fraction = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).expect("nonzero test denominator")
        };
        let sqrt_parameter = |numerator: i8, denominator: i8| {
            let polynomial = decided(
                crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
                    vec![
                        Real::from(-numerator),
                        Real::zero(),
                        Real::from(denominator),
                    ],
                    &policy,
                )
                .expect("valid square-root polynomial"),
            );
            let interval = decided(
                crate::BezierParameterInterval::try_new_with_policy(
                    Real::zero(),
                    Real::one(),
                    &policy,
                )
                .expect("valid unit interval"),
            );
            BezierParameter2::Algebraic(decided(
                BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy)
                    .expect("isolated square-root parameter"),
            ))
        };
        let horizontal = rational_line(0, 1);
        let first_start = crate::tests::decided(
            exact_contact_point_evidence(
                &horizontal,
                &BezierParameter2::Algebraic(sqrt_half_parameter(&policy)),
                &policy,
            )
            .expect("exact first start"),
        );
        let first_end = crate::tests::decided(
            exact_contact_point_evidence(
                &horizontal,
                &BezierParameter2::Algebraic(sqrt_third_parameter(&policy)),
                &policy,
            )
            .expect("exact first end"),
        );
        let vertical = RationalBezier2::try_new(
            vec![
                Point2::new(fraction(5, 8), Real::from(-1_i8)),
                Point2::new(fraction(5, 8), Real::one()),
            ],
            vec![Real::one(); 2],
        )
        .expect("valid vertical source");
        let second_start_parameter = sqrt_parameter(2, 5);
        let second_end_parameter = sqrt_parameter(1, 5);
        let second_start = crate::tests::decided(
            exact_contact_point_evidence(&vertical, &second_start_parameter, &policy)
                .expect("exact second start"),
        );
        let second_end = crate::tests::decided(
            exact_contact_point_evidence(&vertical, &second_end_parameter, &policy)
                .expect("exact second end"),
        );
        let first = decided(
            crate::BezierAlgebraicChord2::try_new(first_start.clone(), first_end.clone(), &policy)
                .expect("valid horizontal chord"),
        );
        let second = decided(
            crate::BezierAlgebraicChord2::try_new(
                second_start.clone(),
                second_end.clone(),
                &policy,
            )
            .expect("valid vertical chord"),
        );
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let pair = RegionCarrierPair {
            first_carrier_index: 0,
            second_carrier_index: 1,
            context: RegionCarrierPairContext::AlgebraicChordPair {
                endpoint_contact: None,
            },
        };
        let context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers: vec![
                    algebraic_chord_carrier(CurveRegionBooleanOperand2::First, first.clone()),
                    algebraic_chord_carrier(CurveRegionBooleanOperand2::Second, second.clone()),
                ],
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: Vec::new(),
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let result = context
            .pair_result(&pair)
            .expect("strict interior chord pair must complete");
        assert!(result.blockers.is_empty(), "{result:?}");
        let [contact] = result.contacts.as_slice() else {
            panic!("expected one strict interior chord contact: {result:?}");
        };
        assert!(contact.is_certified_transverse());
        assert!(matches!(
            contact.point(),
            Some(CurvePoint2(CurvePointData2::AlgebraicChordPair(_)))
        ));
        for (carrier, chord, parameter) in [
            (&context.data.carriers[0], &first, contact.first_parameter()),
            (
                &context.data.carriers[1],
                &second,
                contact.second_parameter(),
            ),
        ] {
            let events = vec![
                CarrierEvent {
                    parameter: carrier.start.clone(),
                    topology_vertex: Some(0),
                },
                CarrierEvent {
                    parameter: parameter.clone(),
                    topology_vertex: Some(2),
                },
                CarrierEvent {
                    parameter: carrier.end.clone(),
                    topology_vertex: Some(1),
                },
            ];
            let splits = split_algebraic_chord_carrier(carrier, chord, &events, &policy)
                .expect("correlated interior contact must split the chord");
            assert_eq!(splits.len(), 2);
            assert_eq!(splits[0].end_topology_vertex, Some(2));
            assert_eq!(splits[1].start_topology_vertex, Some(2));
            assert!(
                splits
                    .iter()
                    .all(|split| matches!(split.fragment, BezierSplitFragment2::AlgebraicChord(_)))
            );
        }

        let first_apex = CurvePoint2::from(Point2::new(fraction(16, 25), Real::from(-1_i8)));
        let second_apex = CurvePoint2::from(Point2::new(Real::one(), fraction(1, 20)));
        let close = |start, end| {
            BezierSplitFragment2::AlgebraicChord(decided(
                crate::BezierAlgebraicChord2::try_new(start, end, &policy)
                    .expect("valid triangle closure chord"),
            ))
        };
        let first_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(first),
                close(first_end, first_apex.clone()),
                close(first_apex, first_start),
            ],
            &policy,
        )
        .expect("first algebraic chord triangle must close");
        let second_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(second),
                close(second_end, second_apex.clone()),
                close(second_apex, second_start),
            ],
            &policy,
        )
        .expect("second algebraic chord triangle must close");
        let region = |boundary| {
            CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
            )
            .expect("valid algebraic chord triangle")
        };
        let first_region = region(first_loop);
        let second_region = region(second_loop);
        let intersections = first_region
            .intersect_region_with_policy(&second_region, &policy)
            .expect("public strict interior chord intersection must complete");
        assert_eq!(intersections.certainty, crate::CurveCertainty::Certified);
        assert!(intersections.value.is_complete(), "{intersections:?}");
        assert!(
            intersections
                .value
                .contacts()
                .iter()
                .any(|contact| matches!(
                    contact.point(),
                    Some(CurvePoint2(CurvePointData2::AlgebraicChordPair(_)))
                )),
            "{intersections:?}"
        );
        let boolean_context =
            CurveRegionBooleanContext::try_new(&first_region, &second_region, &policy)
                .expect("valid strict-interior Boolean context");
        let mut retained_pair_points = Vec::new();
        for pair in &boolean_context.data.pairs {
            let pair_result = boolean_context.pair_result(pair).unwrap();
            retained_pair_points.extend(
                pair_result
                    .contacts
                    .iter()
                    .filter_map(|contact| contact.point().cloned()),
            );
        }
        assert_eq!(retained_pair_points.len(), 2);
        assert_eq!(
            retained_pair_points[0].same_point(&retained_pair_points[1], &policy),
            Classification::Decided(false),
            "distinct correlated contacts need a certified spatial separation"
        );
        let split_topology = boolean_context.build_split_topology();
        assert!(
            split_topology.is_ok(),
            "strict interior chord contact must build split topology: {split_topology:?}"
        );
        let built = boolean_context.build_boolean_regions();
        assert!(
            built.is_ok(),
            "strict interior chord contact must build Boolean regions: {built:?}"
        );
        let booleans = first_region.boolean_regions_with_policy(&second_region, &policy);
        assert!(
            booleans.is_ok(),
            "strict interior algebraic chord crossing must traverse all Booleans: {booleans:?}"
        );
        let booleans = booleans
            .expect("complete strict-interior Boolean batch")
            .into_value();
        for (name, result) in [
            ("union", booleans.union()),
            ("intersection", booleans.intersection()),
            ("difference", booleans.difference()),
            ("xor", booleans.xor()),
        ] {
            let replay_context = CurveRegionBooleanContext::try_new_unary(result, &policy)
                .expect("valid correlated-output arrangement context");
            for pair in &replay_context.data.pairs {
                let pair_result = replay_context
                    .pair_result(pair)
                    .expect("correlated-output carrier pair replay");
                assert!(
                    pair_result.blockers.is_empty(),
                    "{name} retained pair {pair:?} must replay exactly: {pair_result:?}"
                );
            }
            let replay_topology = replay_context.build_split_topology();
            assert!(
                replay_topology.is_ok(),
                "{name} correlated-output split topology must replay: {replay_topology:?}"
            );
            let replay = result.regularized_region_with_policy(&policy);
            assert!(
                replay.is_ok(),
                "{name} must remain an authoritative Boolean input after correlated chord splits: {replay:?}"
            );
        }
        let far_loop = CurveRegionBoundaryLoop2::new(
            vec![
                close(
                    CurvePoint2::from(Point2::from_values(10, 10)),
                    CurvePoint2::from(Point2::from_values(11, 10)),
                ),
                close(
                    CurvePoint2::from(Point2::from_values(11, 10)),
                    CurvePoint2::from(Point2::from_values(10, 11)),
                ),
                close(
                    CurvePoint2::from(Point2::from_values(10, 11)),
                    CurvePoint2::from(Point2::from_values(10, 10)),
                ),
            ],
            &policy,
        )
        .expect("far exact triangle must close");
        let far_region = region(far_loop);
        let replay_boolean = booleans
            .xor()
            .boolean_regions_with_policy(&far_region, &policy);
        assert!(
            replay_boolean.is_ok(),
            "a correlated Boolean output must remain usable against a disjoint exact region: {replay_boolean:?}"
        );
        let enclosing_loop = CurveRegionBoundaryLoop2::new(
            vec![
                close(
                    CurvePoint2::from(Point2::from_values(-3, -3)),
                    CurvePoint2::from(Point2::from_values(5, -3)),
                ),
                close(
                    CurvePoint2::from(Point2::from_values(5, -3)),
                    CurvePoint2::from(Point2::from_values(-3, 5)),
                ),
                close(
                    CurvePoint2::from(Point2::from_values(-3, 5)),
                    CurvePoint2::from(Point2::from_values(-3, -3)),
                ),
            ],
            &policy,
        )
        .expect("enclosing exact triangle must close");
        let enclosing_region = region(enclosing_loop);
        let contained_context =
            CurveRegionBooleanContext::try_new(booleans.xor(), &enclosing_region, &policy)
                .expect("valid contained replay context");
        let contained_topology = contained_context.build_boolean_topology();
        assert!(
            contained_topology.is_ok(),
            "contained correlated topology must classify: {contained_topology:?}"
        );
        let contained_replay = booleans
            .xor()
            .boolean_regions_with_policy(&enclosing_region, &policy);
        assert!(
            contained_replay.is_ok(),
            "a correlated Boolean output must remain classifiable inside an exact region: {contained_replay:?}"
        );
    }
}

#[test]
fn noninjective_preimages_survive_curve_queries_and_cancel_from_regions() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let second_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
        let horizontal = rational_line(0, 1);
        let first_endpoint = crate::tests::decided(
            exact_contact_point_evidence(&horizontal, &first_parameter, &policy)
                .expect("exact first endpoint"),
        );
        let second_endpoint = crate::tests::decided(
            exact_contact_point_evidence(&horizontal, &second_parameter, &policy)
                .expect("exact second endpoint"),
        );
        let bottom = CurvePoint2::from(Point2::from_values(0, -1));
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                first_endpoint.clone(),
                second_endpoint.clone(),
                &policy,
            )
            .expect("valid independent-field chord"),
        );
        let second_closure = decided(
            crate::BezierAlgebraicChord2::try_new(second_endpoint, bottom.clone(), &policy)
                .expect("valid second closure"),
        );
        let first_closure = decided(
            crate::BezierAlgebraicChord2::try_new(bottom, first_endpoint, &policy)
                .expect("valid first closure"),
        );
        let chord_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(chord),
                BezierSplitFragment2::AlgebraicChord(second_closure),
                BezierSplitFragment2::AlgebraicChord(first_closure),
            ],
            &policy,
        )
        .expect("independent-field chord triangle must close");
        let chord_region = CurveRegion2::try_new_with_loop_topology(
            vec![chord_loop],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid independent-field chord triangle");

        let materialized_line = |start: Point2, end: Point2| BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(start, end).expect("valid source closure edge"),
            )),
        };
        let source_loop = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        Point2::from_values(0, 0),
                        Point2::from_values(2, 0),
                        Point2::from_values(0, 0),
                    )),
                },
                materialized_line(Point2::from_values(0, 0), Point2::from_values(-1, 0)),
                materialized_line(Point2::from_values(-1, 0), Point2::from_values(-1, 1)),
                materialized_line(Point2::from_values(-1, 1), Point2::from_values(0, 1)),
                materialized_line(Point2::from_values(0, 1), Point2::from_values(0, 0)),
            ],
            &policy,
        )
        .expect("retraced source loop must close");
        let source_region = CurveRegion2::try_new_with_loop_topology(
            vec![source_loop],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Right],
        )
        .expect("valid retraced source region");

        // The authored curve visits the horizontal chord twice and
        // meets each closing edge at both endpoint preimages. Shared
        // span endpoints belong to the overlap, so query all three
        // authored boundary curves to retain the original point contacts.
        let retraced_curve = Curve2::from_retained_fragment(
            source_region.boundary_loops()[0].fragments()[0].clone(),
        );
        let (mut overlap_count, mut contact_count) = (0, 0);
        for fragment in chord_region.boundary_loops()[0].fragments() {
            let boundary_curve = Curve2::from_retained_fragment(fragment.clone());
            let curve_intersections = boundary_curve
                .intersect_curve_with_policy(&retraced_curve, &policy)
                .unwrap();
            assert_eq!(
                curve_intersections.certainty,
                crate::CurveCertainty::Certified
            );
            assert!(
                curve_intersections.value.is_complete(),
                "{:?}",
                curve_intersections.value.blockers()
            );
            overlap_count += curve_intersections.value.overlaps().len();
            contact_count += curve_intersections.value.contacts().len();
            for contact in curve_intersections.value.contacts() {
                for (curve, location) in [
                    (&boundary_curve, contact.first()),
                    (&retraced_curve, contact.second()),
                ] {
                    let parameter = decided(location.parameter_with_policy(&policy).unwrap());
                    let point = curve.point_at_with_policy(&parameter, &policy).unwrap();
                    assert_eq!(point.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(
                        point.value.same_point(contact.point(), &policy),
                        Classification::Decided(true)
                    );
                }
            }
            for overlap in curve_intersections.value.overlaps() {
                for (first, second) in [
                    (
                        overlap.first_range().start(),
                        overlap.second_range().start(),
                    ),
                    (overlap.first_range().end(), overlap.second_range().end()),
                ] {
                    let first = boundary_curve.point_at_with_policy(first, &policy).unwrap();
                    let second = retraced_curve
                        .point_at_with_policy(second, &policy)
                        .unwrap();
                    assert_eq!(first.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(second.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(
                        first.value.same_point(&second.value, &policy),
                        Classification::Decided(true)
                    );
                }
            }
        }
        assert_eq!(overlap_count, 2);
        assert_eq!(contact_count, 4);

        let intersections = chord_region
            .intersect_region_with_policy(&source_region, &policy)
            .expect("the retraced spur must cancel from the filled boundary");
        assert_eq!(
            intersections.certainty,
            crate::CurveCertainty::Certified,
            "the correlated exact proof must precede the approximate terminal"
        );
        let intersections = intersections.into_value();
        assert!(intersections.is_complete(), "{intersections:?}");
        assert!(intersections.overlaps().is_empty(), "{intersections:?}");
        assert!(intersections.contacts().is_empty(), "{intersections:?}");
        let normalized = source_region
            .regularized_region_with_policy(&policy)
            .unwrap();
        assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
        assert_eq!(normalized.value.boundary_loops().len(), 1);
        assert_eq!(normalized.value.boundary_loops()[0].fragments().len(), 4);
        let intersection = chord_region
            .boolean_region_with_policy(&source_region, BooleanOp::Intersection, &policy)
            .unwrap();
        assert_eq!(intersection.certainty, crate::CurveCertainty::Certified);
        assert!(intersection.value.is_empty());
    }
}

#[test]
fn finite_parallel_self_contacts_retain_active_domain() {
    let p = Point2::from_values;
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let mut cache = CurveIntersectionBatchCache::default();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for finite in [false, true] {
            let source = if finite {
                CubicBezier2::new(
                    p(-1, 0),
                    Point2::new((-1).into(), q(-1, 3)),
                    Point2::new(q(-2, 3), q(-2, 3)),
                    p(0, 0),
                )
            } else {
                CubicBezier2::new(
                    p(3, -6),
                    Point2::new(q(-7, 3), q(26, 3)),
                    Point2::new(q(-7, 3), q(-26, 3)),
                    p(3, 6),
                )
            };
            let parallel = source.parallel_left(Real::zero()).unwrap();
            if finite {
                let native =
                    Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(
                        BezierParallelFragment2::from_certified_range(
                            parallel.clone(),
                            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                            false,
                        ),
                    ));
                let native = CurveIntersectionContext::new_self(&native, &policy, &mut cache);
                assert!(
                    native.result().unwrap().is_disjoint(),
                    "unit evidence cannot exclude the exterior self-contact"
                );
            }
            let fragment = BezierParallelFragment2::from_certified_range(
                parallel,
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(if finite { (-2).into() } else { 0.into() }),
                    BezierParameter2::Exact(if finite { 2.into() } else { 1.into() }),
                ),
                false,
            );
            let curve =
                Curve2::from_retained_fragment(BezierSplitFragment2::AnalyticParallel(fragment));
            let context = CurveIntersectionContext::new_self(&curve, &policy, &mut cache);
            let result = context.result().unwrap();
            assert!(result.is_complete(), "finite={finite}: {result:?}");
            assert_eq!(result.contacts().len(), 1, "finite={finite}: {result:?}");
            let contact = &result.contacts()[0];
            for (actual, expected) in [
                (
                    contact.first().local_parameter(),
                    if finite { Real::from(-1) } else { q(1, 4) },
                ),
                (
                    contact.second().local_parameter(),
                    if finite { Real::one() } else { q(3, 4) },
                ),
            ] {
                assert_eq!(
                    actual.same_value(&expected.into(), &policy).unwrap(),
                    Classification::Decided(true)
                );
            }
            assert!(contact.is_certified_transverse());
        }
    }
}

#[test]
fn finite_self_crossing_regions_retain_boundary_ownership_on_reentry() {
    fn certified<T>(outcome: CurveOutcome<T>) -> T {
        assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
        outcome.value
    }
    let p = Point2::from_values;
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for finite in [false, true] {
            let cubic = if finite {
                CubicBezier2::new(
                    p(-1, 0),
                    Point2::new((-1).into(), q(-1, 3)),
                    Point2::new(q(-2, 3), q(-2, 3)),
                    p(0, 0),
                )
            } else {
                CubicBezier2::new(
                    p(3, -6),
                    Point2::new(q(-7, 3), q(26, 3)),
                    Point2::new(q(-7, 3), q(-26, 3)),
                    p(3, 6),
                )
            };
            for elevated in [false, true] {
                let source = BezierSubcurve2::Cubic(cubic.clone());
                let source = if elevated {
                    BezierSubcurve2::Rational(
                        RationalBezier2::try_from_subcurve(&source)
                            .unwrap()
                            .elevated_to_degree(5)
                            .unwrap(),
                    )
                } else {
                    source
                };
                let source = Curve2::from_retained_fragment(BezierSplitFragment2::RetainedBezier {
                    source_curve: source,
                    start: BezierParameter2::Exact(if finite { (-2).into() } else { 0.into() }),
                    end: BezierParameter2::Exact(if finite { 2.into() } else { 1.into() }),
                    reversed: false,
                    start_image: None,
                    end_image: None,
                });
                for reversed in [false, true] {
                    let mut curves = vec![
                        source.clone(),
                        Curve2::from(LineSeg2::try_new(p(3, 6), p(3, -6)).unwrap()),
                    ];
                    if reversed {
                        curves = curves
                            .into_iter()
                            .rev()
                            .map(|c| certified(c.reversed_with_policy(&policy).unwrap()))
                            .collect();
                    }
                    let path = CurvePath2::try_new(curves).unwrap();
                    let mut region = certified(
                        CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
                            &[path],
                            &[CurveRegionLoopRole::Material],
                            &[FillRule::NonZero],
                            &policy,
                        )
                        .unwrap(),
                    );
                    for generation in 0..2 {
                        region = certified(region.regularized_region_with_policy(&policy).unwrap_or_else(|error| {
                            panic!("finite={finite} elevated={elevated} reversed={reversed} generation={generation}: {error:?}")
                        }));
                        for (point, location) in [
                            (
                                Point2::new(q(-1, 2), Real::zero()),
                                RegionPointLocation::Inside,
                            ),
                            (p(1, 0), RegionPointLocation::Inside),
                            (p(-2, 0), RegionPointLocation::Outside),
                            (p(4, 0), RegionPointLocation::Outside),
                            (p(0, 0), RegionPointLocation::Boundary),
                        ] {
                            assert_eq!(
                                certified(
                                    region
                                        .classify_point_with_policy(&point.clone().into(), &policy)
                                        .unwrap()
                                ),
                                Classification::Decided(location)
                            );
                        }
                        let sides = decided(certified(
                            region.filled_side_is_left_with_policy(&policy).unwrap(),
                        ));
                        let paths = decided(certified(
                            region.boundary_paths_with_policy(&policy).unwrap(),
                        ));
                        let mut checked = 0;
                        // Verify the actual owned side at the leftmost regular point.
                        // Selected cuts need no scalar reconstruction to locate it.
                        for (path, &left) in paths.iter().zip(sides) {
                            for curve in path.curves() {
                                for parameter in [Real::zero(), q(1, 2)] {
                                    let Ok(point) = curve
                                        .point_at_with_policy(&parameter.clone().into(), &policy)
                                    else {
                                        continue;
                                    };
                                    if certified(
                                        certified(point).coincides_with(&p(-1, 0).into(), &policy),
                                    ) != Classification::Decided(true)
                                    {
                                        continue;
                                    }
                                    checked += 1;
                                    let tangent = certified(
                                        curve
                                            .derivative_at_with_policy(
                                                &parameter.clone().into(),
                                                &policy,
                                            )
                                            .unwrap(),
                                    );
                                    for sample_left in [false, true] {
                                        let step = if sample_left { q(1, 128) } else { q(-1, 128) };
                                        let sample = Point2::new(
                                            Real::from(-1)
                                                - &step
                                                    * tangent
                                                        .represented_coordinates()
                                                        .expect("represented derivative")
                                                        .1,
                                            &step
                                                * tangent
                                                    .represented_coordinates()
                                                    .expect("represented derivative")
                                                    .0,
                                        );
                                        let expected = if sample_left == left {
                                            RegionPointLocation::Inside
                                        } else {
                                            RegionPointLocation::Outside
                                        };
                                        assert_eq!(
                                            certified(
                                                region
                                                    .classify_point_with_policy(
                                                        &sample.clone().into(),
                                                        &policy
                                                    )
                                                    .unwrap()
                                            ),
                                            Classification::Decided(expected),
                                            "finite={finite} elevated={elevated} reversed={reversed}"
                                        );
                                    }
                                }
                            }
                        }
                        assert_eq!(checked, 1);
                        region = certified(
                            region
                                .boolean_region_with_policy(&region, BooleanOp::Union, &policy)
                                .unwrap(),
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn regularization_orders_all_branches_at_a_pinched_algebraic_corner() {
    use crate::RegionPointLocation::{Boundary, Inside, Outside};
    use crate::bezier_region::CurveBoundaryInteriorSide2::{Left, Right};

    let q = |numerator: i32, denominator: i32| {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // B(t)=(t-1,(t-1)^2) continues past the origin to t=1+a,
        // where a^2+a^4=1. The return chord and vertical edge meet
        // the interior of B at the same vertex. Its four rays belong
        // to three carriers, so no single pair owns their cyclic order.
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
                [1, -6, 7, -4, 1].into_iter().map(Real::from).collect(),
                &policy,
            )
            .unwrap(),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new_with_policy(q(7, 4), Real::from(2), &policy)
                .unwrap(),
        );
        let cut_root = decided(
            BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, &policy)
                .unwrap(),
        );
        let chord_midpoint = RationalBezierAlgebraicPointImage2::from_retained_expression(
            cut_root.clone(),
            crate::bezier_algebraic_image::parameter_representation(&cut_root, &policy),
            vec![q(-1, 2), q(1, 2)],
            vec![q(1, 2), -Real::one(), q(1, 2)],
            vec![Real::one()],
            "exact midpoint of the return chord",
        );
        let cut = BezierParameter2::Algebraic(cut_root);
        let source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(-1, 1),
            Point2::new(q(-1, 2), Real::zero()),
            Point2::from_values(0, 0),
        ));
        let cut_point = crate::tests::decided(
            exact_contact_point_evidence(
                &RationalBezier2::try_from_subcurve(&source).unwrap(),
                &cut,
                &policy,
            )
            .unwrap(),
        );
        let extended = CurveSupport2::Bezier(source)
            .restrict_certified(
                CurveParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()).into(),
                    cut.into(),
                ),
                Some([Point2::from_values(-1, 1).into(), cut_point.clone()]),
                false,
                &policy,
            )
            .unwrap();
        let chord = BezierSplitFragment2::AlgebraicChord(decided(
            crate::BezierAlgebraicChord2::try_new(
                cut_point,
                Point2::from_values(0, 0).into(),
                &policy,
            )
            .unwrap(),
        ));
        let line = |start, end| BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(start, end).unwrap(),
            )),
        };
        let fragments = vec![
            extended,
            chord,
            line(Point2::from_values(0, 0), Point2::from_values(0, 3)),
            line(Point2::from_values(0, 3), Point2::from_values(-3, 3)),
            line(Point2::from_values(-3, 3), Point2::from_values(-1, 1)),
        ];
        for reversed in [false, true] {
            let fragments = if reversed {
                fragments
                    .iter()
                    .rev()
                    .map(|edge| edge.reversed().unwrap())
                    .collect()
            } else {
                fragments.clone()
            };
            let raw = CurveRegion2::try_new_with_loop_topology(
                vec![CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap()],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed { Right } else { Left }],
            )
            .unwrap();
            // The interior-side ray from (a/2,a²/2) crosses B at
            // x=a/sqrt(2), beyond the source's authored unit interval.
            // This direct geometric seed must agree with face-sector
            // propagation even before the boundary is regularized.
            assert_eq!(
                policy
                    .strict_predicate_pass(|| {
                        raw.algebraic_loop_windings_from_boundary_side_ray(
                            &chord_midpoint,
                            Real::one(),
                            Real::zero(),
                            0,
                            if reversed { 3 } else { 1 },
                            &policy,
                        )
                    })
                    .unwrap()
                    .map(|windings| {
                        let location = raw.region_location_from_loop_windings(&windings).unwrap();
                        (windings, location)
                    }),
                Classification::Decided((vec![if reversed { -1 } else { 1 }], Inside))
            );
            let normalized = raw.regularized_region_with_policy(&policy).unwrap();
            assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
            let normalized = normalized.value;
            for (point, expected) in [
                (Point2::new(q(-1, 2), Real::one()), Inside),
                (Point2::new(q(1, 2), q(3, 10)), Inside),
                (Point2::new(q(1, 2), q(1, 4)), Boundary),
                (Point2::new(q(1, 2), q(1, 2)), Outside),
                (Point2::from_values(0, 0), Boundary),
            ] {
                let actual = normalized
                    .classify_point_with_policy(&point.clone().into(), &policy)
                    .unwrap();
                assert_eq!(actual.certainty, crate::CurveCertainty::Certified);
                assert_eq!(actual.value, Classification::Decided(expected));
            }
            let repeated = normalized
                .boolean_regions_with_policy(&normalized, &policy)
                .unwrap();
            assert_eq!(repeated.certainty, crate::CurveCertainty::Certified);
            assert!(repeated.value.difference().is_empty());
            assert!(repeated.value.xor().is_empty());
            for result in [repeated.value.union(), repeated.value.intersection()] {
                assert_eq!(
                    result
                        .classify_point_with_policy(&Point2::new(q(1, 2), q(3, 10)).into(), &policy)
                        .unwrap()
                        .value,
                    Classification::Decided(Inside)
                );
            }
        }
    }
}

#[test]
fn curved_face_windings_preserve_crossings_tangencies_overlaps_and_nested_holes() {
    use crate::CurveRegionLoopRole::{Hole, Material};

    // All boundaries are curved, so there is no affine seed. Integer
    // circle equations give an independent membership oracle for the
    // propagated winding actions, including four levels of nesting.
    let cases = [
        vec![(-1, 2, Material), (1, 2, Material)],
        vec![(0, 2, Material), (4, 2, Material)],
        vec![(0, 3, Material), (2, 1, Hole)],
        vec![(0, 2, Material), (0, 2, Material)],
        vec![
            (0, 4, Material),
            (0, 3, Hole),
            (0, 2, Material),
            (0, 1, Hole),
        ],
    ];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let weight = (Real::one() / Real::from(2)).unwrap().sqrt().unwrap();
        for reversed in [false, true] {
            for circles in &cases {
                let paths = circles
                    .iter()
                    .map(|&(center, radius, _)| {
                        let point = |x, y| Point2::from_values(center + radius * x, radius * y);
                        let mut curves = [
                            [(1, 0), (1, 1), (0, 1)],
                            [(0, 1), (-1, 1), (-1, 0)],
                            [(-1, 0), (-1, -1), (0, -1)],
                            [(0, -1), (1, -1), (1, 0)],
                        ]
                        .map(|[start, control, end]| {
                            Curve2::from(
                                crate::RationalQuadraticBezier2::try_new(
                                    point(start.0, start.1),
                                    point(control.0, control.1),
                                    point(end.0, end.1),
                                    Real::one(),
                                    weight.clone(),
                                    Real::one(),
                                )
                                .unwrap(),
                            )
                        })
                        .to_vec();
                        if reversed {
                            curves = curves
                                .into_iter()
                                .rev()
                                .map(|curve| curve.reversed_with_policy(&policy).unwrap().value)
                                .collect();
                        }
                        CurvePath2::try_new(curves).unwrap()
                    })
                    .collect::<Vec<_>>();
                let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics_raw(
                    &paths,
                    &circles.iter().map(|circle| circle.2).collect::<Vec<_>>(),
                    &vec![FillRule::NonZero; circles.len()],
                    &policy,
                )
                .unwrap();
                let normalized = raw.regularized_region_raw(&policy).unwrap();
                assert!(normalized.has_regularized_filled_left_topology(&policy));
                for x in [-7, -3, -1, 1, 3, 5, 7, 9] {
                    for y in [-5, -1, 1, 5] {
                        // Half-integer coordinates cannot lie on any of
                        // these integer-center, integer-radius circles.
                        let depth: i32 = circles
                            .iter()
                            .filter(|&&(center, radius, _)| {
                                (x - 2 * center) * (x - 2 * center) + y * y < 4 * radius * radius
                            })
                            .map(|circle| if circle.2 == Material { 1 } else { -1 })
                            .sum();
                        let expected = if depth > 0 {
                            RegionPointLocation::Inside
                        } else {
                            RegionPointLocation::Outside
                        };
                        let point = Point2::new(
                            (Real::from(x) / Real::from(2)).unwrap(),
                            (Real::from(y) / Real::from(2)).unwrap(),
                        );
                        assert_eq!(
                            normalized.classify_point_raw(&point, &policy).unwrap(),
                            Classification::Decided(expected),
                            "circles={circles:?}, reversed={reversed}, policy={policy:?}, twice_point=({x}, {y})"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn regularization_removes_symmetric_polynomial_and_rational_retracing() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let origin = Point2::from_values(0, 0);
        let turning = Point2::from_values(2, 0);
        let curves = [
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                origin.clone(),
                turning.clone(),
                origin.clone(),
            )),
            BezierSubcurve2::Cubic(CubicBezier2::new(
                origin.clone(),
                turning.clone(),
                turning.clone(),
                origin.clone(),
            )),
            BezierSubcurve2::RationalQuadratic(
                crate::RationalQuadraticBezier2::try_new(
                    origin.clone(),
                    turning,
                    origin.clone(),
                    Real::one(),
                    Real::from(2),
                    Real::one(),
                )
                .unwrap(),
            ),
            // This palindrome retraces a genuinely curved image. The
            // proof concerns oriented winding, not collinearity or area.
            BezierSubcurve2::Rational(
                RationalBezier2::try_new(
                    vec![
                        origin.clone(),
                        Point2::from_values(1, 0),
                        Point2::from_values(1, 1),
                        Point2::from_values(1, 0),
                        origin,
                    ],
                    vec![
                        Real::one(),
                        Real::from(2),
                        Real::from(3),
                        Real::from(2),
                        Real::one(),
                    ],
                )
                .unwrap(),
            ),
        ];
        for curve in curves {
            let fragment = BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve,
            };
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![CurveRegionBoundaryLoop2::new(vec![fragment], &policy).unwrap()],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
            )
            .unwrap();
            let normalized = region.regularized_region_with_policy(&policy).unwrap();
            assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
            assert!(normalized.value.is_empty());
        }
    }
}

#[test]
fn noninjective_collinear_chord_dispatch_retains_contacts_and_overlaps() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let endpoint_parameter = sqrt_half_parameter(&policy);
        let horizontal = rational_line(0, 1);
        let endpoint = CurvePoint2::from(crate::tests::decided(
            horizontal
                .point_at_algebraic_parameter(&endpoint_parameter, &policy)
                .expect("exact algebraic endpoint image"),
        ));
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(Point2::from_values(0, 0)),
                endpoint,
                &policy,
            )
            .expect("valid mixed-field chord"),
        );
        let fraction = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
        };
        let cubic_point =
            |numerator, denominator| Point2::new(fraction(numerator, denominator), Real::zero());
        // q(t) has a double zero at 1/4, a second zero at 5/8,
        // and then rises through the complete chord image.
        let source = BezierSubcurve2::Cubic(CubicBezier2::new(
            cubic_point(-5, 27),
            cubic_point(11, 27),
            cubic_point(-7, 9),
            Point2::from_values(1, 0),
        ));
        let chord_geometry = CurveSupport2::Line(chord.clone());
        let source_geometry = CurveSupport2::Bezier(source);
        let empty_first = CurveRegion2::empty();
        let empty_second = CurveRegion2::empty();
        let context = CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: &empty_first,
                second: &empty_second,
                policy,
                carriers: vec![
                    RegionCarrier {
                        operand: CurveRegionBooleanOperand2::First,
                        loop_index: 0,
                        fragment_index: 0,
                        family: chord_geometry.family(),
                        geometry: chord_geometry,
                        start: CurveParameter2::from_algebraic_chord(chord.start_parameter()),
                        end: CurveParameter2::from_algebraic_chord(chord.end_parameter()),
                        reversed: false,
                        filled_side_is_left: true,
                        selected_fiber_endpoint_points: None,
                        image_is_injective: OnceLock::new(),
                        bounds: OnceLock::new(),
                        refined_bounds: Default::default(),
                    },
                    RegionCarrier {
                        operand: CurveRegionBooleanOperand2::Second,
                        loop_index: 0,
                        fragment_index: 0,
                        family: source_geometry.family(),
                        geometry: source_geometry,
                        start: carrier_parameter(BezierParameter2::Exact(Real::zero())),
                        end: carrier_parameter(BezierParameter2::Exact(Real::one())),
                        reversed: false,
                        filled_side_is_left: true,
                        selected_fiber_endpoint_points: None,
                        image_is_injective: OnceLock::new(),
                        bounds: OnceLock::new(),
                        refined_bounds: Default::default(),
                    },
                ],
                first_carrier_count: 1,
                authored_carrier_pair_count: 1,
                pairs: vec![RegionCarrierPair {
                    first_carrier_index: 0,
                    second_carrier_index: 1,
                    context: RegionCarrierPairContext::AlgebraicChordPair {
                        endpoint_contact: None,
                    },
                }],
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        };
        let pair_result = context
            .pair_result(&context.data.pairs[0])
            .expect("mixed collinear evidence must dispatch exactly");
        assert!(pair_result.blockers.is_empty(), "{pair_result:?}");
        assert_eq!(pair_result.contacts.len(), 1, "{pair_result:?}");
        assert_eq!(pair_result.overlaps.len(), 1, "{pair_result:?}");
        assert!(!pair_result.contacts[0].is_certified_transverse());
        assert_eq!(
            pair_result.overlaps[0].orientation,
            CurveOverlapOrientation2::Same
        );

        let evidence = context
            .build_intersection_evidence()
            .expect("mixed evidence must enter CurveRegion intersection output");
        assert!(evidence.is_complete(), "{evidence:?}");
        assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
        assert_eq!(evidence.overlaps().len(), 1, "{evidence:?}");
    }
}

#[test]
fn adjacent_general_chord_fallback_excludes_the_authored_endpoint() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let source_parameter = BezierParameter2::Algebraic(sqrt_half_parameter(&policy));
        let independent_parameter = BezierParameter2::Algebraic(sqrt_third_parameter(&policy));
        let source_curve = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::new(Real::zero(), -half.clone()),
            Point2::new(half.clone(), -half.clone()),
            Point2::new(Real::one(), half.clone()),
        ));
        let source_rational =
            RationalBezier2::try_from_subcurve(&source_curve).expect("valid parabola");
        let shared_endpoint = crate::tests::decided(
            exact_contact_point_evidence(&source_rational, &source_parameter, &policy)
                .expect("exact shared endpoint"),
        );
        let y_axis = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(0, 1))
                .expect("valid y axis"),
        ));
        let y_rational =
            RationalBezier2::try_from_subcurve(&y_axis).expect("valid rational y axis");
        let independent_endpoint = crate::tests::decided(
            exact_contact_point_evidence(&y_rational, &independent_parameter, &policy)
                .expect("exact independent endpoint"),
        );
        let chord = decided(
            crate::BezierAlgebraicChord2::try_new(
                shared_endpoint,
                independent_endpoint.clone(),
                &policy,
            )
            .expect("valid independent-field chord"),
        );
        let closure = decided(
            crate::BezierAlgebraicChord2::try_new(
                independent_endpoint,
                CurvePoint2::from(Point2::new(Real::zero(), -half)),
                &policy,
            )
            .expect("valid algebraic closure"),
        );
        let source_fragment = decided(
            source_curve
                .split_at_parameters_refined(
                    &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    std::slice::from_ref(&source_parameter),
                    &policy,
                )
                .expect("exact source split"),
        )
        .fragments()[0]
            .clone();
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                source_fragment,
                BezierSplitFragment2::AlgebraicChord(chord),
                BezierSplitFragment2::AlgebraicChord(closure),
            ],
            &policy,
        )
        .expect("adjacent independent-field loop must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .expect("valid adjacent independent-field region");
        let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
            .expect("valid adjacent Boolean context");
        let pair = context
            .data
            .pairs
            .iter()
            .find(|pair| {
                [pair.first_carrier_index, pair.second_carrier_index]
                    .into_iter()
                    .map(|index| &context.data.carriers[index])
                    .any(|carrier| carrier.fragment_index == 0)
                    && [pair.first_carrier_index, pair.second_carrier_index]
                        .into_iter()
                        .map(|index| &context.data.carriers[index])
                        .any(|carrier| carrier.fragment_index == 1)
            })
            .expect("adjacent source/chord endpoint must schedule the pair");
        assert!(context.authored_carriers_are_adjacent(pair));
        let result = context
            .pair_result(pair)
            .expect("adjacent independent-field fallback must complete");
        assert!(result.blockers.is_empty(), "adjacent result: {result:?}");
        assert!(
            result.contacts.is_empty(),
            "authored adjacency owns the shared endpoint: {result:?}"
        );
    }
}

fn shifted_sqrt_half_parameter(shift: Real, policy: &CurveContext) -> BezierParameter2 {
    let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
    let polynomial = decided(
        crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
            vec![
                &shift * &shift - half,
                Real::zero() - &shift * Real::from(2_i8),
                Real::one(),
            ],
            policy,
        )
        .expect("valid shifted quadratic"),
    );
    let interval = decided(
        crate::BezierParameterInterval::try_new_with_policy(
            (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
            Real::one(),
            policy,
        )
        .expect("valid positive-root interval"),
    );
    BezierParameter2::Algebraic(decided(
        BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, policy)
            .expect("isolated shifted positive root"),
    ))
}

fn shifted_nested_radical_parameter(shift: Real, policy: &CurveContext) -> BezierParameter2 {
    let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
    let alpha = half.sqrt().expect("positive square root");
    let polynomial = decided(
        crate::BezierParameterPolynomial::try_new_power_basis_with_policy(
            vec![
                &shift * &shift - &shift - alpha,
                Real::one() - &shift * Real::from(2_i8),
                Real::one(),
            ],
            policy,
        )
        .expect("valid translated nested-radical quadratic"),
    );
    let interval = decided(
        crate::BezierParameterInterval::try_new_with_policy(Real::zero(), Real::one(), policy)
            .expect("valid unit parameter interval"),
    );
    BezierParameter2::Algebraic(decided(
        BezierAlgebraicParameter2::try_isolate_with_policy(polynomial, interval, policy)
            .expect("isolated translated nested-radical root"),
    ))
}

fn dyadic_epsilon(exponent: usize) -> Real {
    Real::new(
        crate::Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(1_u8) << exponent)
            .expect("positive dyadic epsilon"),
    )
}

fn injective_test_carrier() -> RegionCarrier {
    RegionCarrier {
        operand: CurveRegionBooleanOperand2::First,
        loop_index: 0,
        fragment_index: 0,
        family: CurveFamily2::RationalBezier,
        geometry: CurveSupport2::Bezier(BezierSubcurve2::Rational(rational_line(0, 1))),
        start: carrier_parameter(BezierParameter2::Exact(Real::zero())),
        end: carrier_parameter(BezierParameter2::Exact(Real::one())),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    }
}

fn noninjective_test_carrier() -> RegionCarrier {
    RegionCarrier {
        operand: CurveRegionBooleanOperand2::First,
        loop_index: 0,
        fragment_index: 0,
        family: CurveFamily2::QuadraticBezier,
        geometry: CurveSupport2::Bezier(BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(0, 0),
        ))),
        start: carrier_parameter(BezierParameter2::Exact(Real::zero())),
        end: carrier_parameter(BezierParameter2::Exact(Real::one())),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    }
}

fn cusp_test_semicircle(policy: &CurveContext) -> BezierAlgebraicCuspSemicircle2 {
    let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
    let parallel = CubicBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(0, 4),
        Point2::from_values(4, -4),
        Point2::from_values(4, 0),
    )
    .parallel_left(half.clone())
    .expect("valid analytic parallel");
    let analysis = decided(
        parallel
            .singularity_analysis_with_policy(&CurveParameterRange2::unit(), policy)
            .expect("certified singularity analysis"),
    );
    let BezierParameter2::Algebraic(parameter) = &analysis.parallel_cusps()[0] else {
        panic!("the selected general cusp must be algebraic");
    };
    decided(
        parallel
            .algebraic_cusp_semicircle(parameter, half, false, policy)
            .expect("certified cusp semicircle"),
    )
    .expect("the nonzero cusp radius must produce a semicircle")
}

fn cusp_test_carrier(
    semicircle: BezierAlgebraicCuspSemicircle2,
    start: Real,
    end: Real,
    operand: CurveRegionBooleanOperand2,
    policy: &CurveContext,
) -> RegionCarrier {
    let fragment = decided(
        BezierAlgebraicCuspSemicircleFragment2::try_new(
            semicircle,
            BezierAlgebraicCuspSemicircleParameter2::Exact(start),
            BezierAlgebraicCuspSemicircleParameter2::Exact(end),
            false,
            policy,
        )
        .expect("valid cusp fragment"),
    );
    let geometry = CurveSupport2::Circle(fragment.clone());
    RegionCarrier {
        operand,
        loop_index: 0,
        fragment_index: 0,
        family: geometry.family(),
        geometry,
        start: CurveParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
        end: CurveParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    }
}

#[test]
fn unary_cusp_regularization_samples_sides_in_the_selected_field() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = cusp_test_semicircle(&policy);
        let second = first.complementary_half();
        let forward = vec![
            BezierSplitFragment2::AlgebraicCuspSemicircle(
                BezierAlgebraicCuspSemicircleFragment2::full(first, &policy),
            ),
            BezierSplitFragment2::AlgebraicCuspSemicircle(
                BezierAlgebraicCuspSemicircleFragment2::full(second, &policy),
            ),
        ];
        let reversed = forward
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect::<Vec<_>>();
        for (fragments, interior_side, expected_action) in [
            (
                forward,
                crate::bezier_region::CurveBoundaryInteriorSide2::Left,
                RegionFragmentAction::Keep,
            ),
            (
                reversed,
                crate::bezier_region::CurveBoundaryInteriorSide2::Right,
                RegionFragmentAction::KeepReversed,
            ),
        ] {
            let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy)
                .expect("complementary selected-field cusp halves must close");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![interior_side],
            )
            .unwrap();
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy).unwrap();
            assert_eq!(context.data.carriers.len(), 2);
            for carrier_index in 0..2 {
                let CurveSupport2::Circle(fragment) =
                    &context.data.carriers[carrier_index].geometry
                else {
                    panic!("the selected-field disk must retain both cusp carriers");
                };
                assert_eq!(
                    context
                        .regularized_algebraic_cusp_fragment_decision(carrier_index, fragment)
                        .expect("selected-field side rays must decide")
                        .action,
                    expected_action,
                );
            }
        }
    }
}

#[test]
fn boolean_topology_seeds_a_general_cusp_run_from_an_adjacent_carrier() {
    let local_start = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let semicircle = cusp_test_semicircle(&policy);
        let source_parameter = BezierParameter2::Algebraic(semicircle.cusp_parameter().clone());
        let regular_span = |parallel: &BezierParallel2| {
            let range = match BezierParameterRange2::try_new_with_policy(
                BezierParameter2::Exact(local_start.clone()),
                source_parameter.clone(),
                &policy,
            )
            .unwrap()
            {
                Classification::Decided(range) => range,
                Classification::Uncertain(reason) => panic!("local cusp range: {reason:?}"),
            };
            match BezierParallelFragment2::try_new(parallel.clone(), range, &policy).unwrap() {
                Classification::Decided(fragment) => fragment,
                Classification::Uncertain(reason) => {
                    panic!("regular local cusp span: {reason:?}")
                }
            }
        };
        let start_parallel = semicircle
            .start_parallel()
            .expect("an analytic cusp retains its starting parallel");
        let end_parallel = semicircle
            .end_parallel()
            .expect("an analytic cusp retains its ending parallel");
        let start_fragment = regular_span(&start_parallel);
        let end_fragment = regular_span(&end_parallel);
        let local_point = |parallel: &BezierParallel2| {
            let Classification::Decided(point) = parallel
                .point_at_with_policy(&local_start, &policy)
                .unwrap()
            else {
                panic!("a rational local parameter must have an exact parallel image");
            };
            point
        };
        let closing_line = BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(local_point(&end_parallel), local_point(&start_parallel))
                    .unwrap(),
            )),
        };
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicCuspSemicircle(
                    BezierAlgebraicCuspSemicircleFragment2::full(semicircle, &policy),
                ),
                BezierSplitFragment2::AnalyticParallel(end_fragment)
                    .reversed()
                    .unwrap(),
                closing_line,
                BezierSplitFragment2::AnalyticParallel(start_fragment),
            ],
            &policy,
        )
        .expect("the algebraic cusp and adjacent carriers form one exact loop");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![crate::bezier_region::CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        let distant_algebraic_point = RationalBezierAlgebraicPointImage2::from_parametric_source(
            rational_line(100, 101),
            sqrt_half_parameter(&policy),
            &policy,
        );
        assert_eq!(
            region
                .classify_algebraic_point_raw(&distant_algebraic_point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );
        let empty = CurveRegion2::default();
        let topology = CurveRegionBooleanContext::try_new(&region, &empty, &policy)
            .unwrap()
            .build_boolean_topology()
            .expect("an adjacent exact carrier must seed the non-rational cusp run");
        assert_eq!(topology.point_classification_count, 1);
        assert!(
            topology
                .split_fragments
                .iter()
                .flatten()
                .all(|fragment| { fragment.location == Some(RegionPointLocation::Outside) })
        );
    }
}

#[test]
fn affine_region_classifies_a_general_algebraic_cusp_point_in_its_source_field() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let fragment =
            BezierAlgebraicCuspSemicircleFragment2::full(cusp_test_semicircle(&policy), &policy);
        let Classification::Decided(point) = fragment.representative_point().unwrap() else {
            panic!("the selected cusp representative must retain an exact point image");
        };
        assert!(point.exact_point(&policy).is_none());
        assert_eq!(
            square_region(-10, -10, 10, 10)
                .classify_algebraic_point_raw(&point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Inside),
        );
        assert_eq!(
            square_region(20, 20, 30, 30)
                .classify_algebraic_point_raw(&point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );

        let curved_cap = |control_y: i8| {
            let left = Point2::from_values(-10, -10);
            let right = Point2::from_values(10, -10);
            CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
                &[CurvePath2::try_new(vec![
                    Curve2::from(LineSeg2::try_new(left.clone(), right.clone()).unwrap()),
                    Curve2::from(QuadraticBezier2::new(
                        right,
                        Point2::from_values(0, control_y),
                        left,
                    )),
                ])
                .unwrap()],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &policy,
            )
            .unwrap()
            .into_value()
        };
        assert_eq!(
            curved_cap(30)
                .classify_algebraic_point_off_boundary_raw(&point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Inside),
        );
        assert_eq!(
            curved_cap(0)
                .classify_algebraic_point_off_boundary_raw(&point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );

        let parameter = sqrt_half_parameter(&policy);
        let boundary = RationalBezierAlgebraicPointImage2::from_parametric_source(
            rational_line(0, 1),
            parameter.clone(),
            &policy,
        );
        assert_eq!(
            square_region(0, 0, 2, 2)
                .classify_algebraic_point_raw(&boundary, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Boundary),
        );
        let parabola = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::new((Real::one() / Real::from(2_i8)).unwrap(), Real::zero()),
                Point2::from_values(1, 1),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        let parabola_boundary = RationalBezierAlgebraicPointImage2::from_parametric_source(
            parabola.clone(),
            parameter.clone(),
            &policy,
        );
        let parabola_region =
            CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
                &[CurvePath2::try_new(vec![
                    Curve2::from(parabola),
                    Curve2::from(
                        LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(0, 1))
                            .unwrap(),
                    ),
                    Curve2::from(
                        LineSeg2::try_new(Point2::from_values(0, 1), Point2::from_values(0, 0))
                            .unwrap(),
                    ),
                ])
                .unwrap()],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &policy,
            )
            .unwrap()
            .into_value();
        let parabola_classification = parabola_region
            .classify_point_with_policy(&parabola_boundary.clone().into(), &policy)
            .unwrap();
        assert_eq!(
            parabola_classification.certainty,
            crate::CurveCertainty::Certified,
        );
        assert_eq!(
            parabola_classification.value,
            Classification::Decided(RegionPointLocation::Boundary),
        );
        let dyadic_ray_point = RationalBezierAlgebraicPointImage2::from_parametric_source(
            rational_line(0, 1),
            parameter.clone(),
            &policy,
        );
        let lower_left = Point2::from_values(-2, -1);
        let lower_right = Point2::from_values(2, -1);
        let upper_right = Point2::from_values(2, 1);
        let upper_left = Point2::from_values(-2, 1);
        let dyadic_crossing_region =
            CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
                &[CurvePath2::try_new(vec![
                    Curve2::from(
                        LineSeg2::try_new(lower_left.clone(), lower_right.clone()).unwrap(),
                    ),
                    Curve2::from(QuadraticBezier2::new(
                        lower_right,
                        Point2::from_values(0, 0),
                        upper_right.clone(),
                    )),
                    Curve2::from(LineSeg2::try_new(upper_right, upper_left.clone()).unwrap()),
                    Curve2::from(LineSeg2::try_new(upper_left, lower_left).unwrap()),
                ])
                .unwrap()],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &policy,
            )
            .unwrap()
            .into_value();
        assert_eq!(
            dyadic_crossing_region
                .classify_algebraic_point_off_boundary_raw(&dyadic_ray_point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Inside),
        );
        let ninth = (Real::one() / Real::from(9_i8)).unwrap();
        let tangent_start = Point2::new(Real::from(2_i8), ninth.clone());
        let tangent_end = Point2::new(Real::from(2_i8), &ninth * Real::from(4_i8));
        let tangent_upper_right = Point2::new(Real::from(3_i8), &ninth * Real::from(4_i8));
        let tangent_lower_right = Point2::new(Real::from(3_i8), ninth.clone());
        let tangent_region = CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
            &[CurvePath2::try_new(vec![
                Curve2::from(QuadraticBezier2::new(
                    tangent_start.clone(),
                    Point2::new(Real::zero(), -(&ninth * Real::from(2_i8))),
                    tangent_end.clone(),
                )),
                Curve2::from(LineSeg2::try_new(tangent_end, tangent_upper_right.clone()).unwrap()),
                Curve2::from(
                    LineSeg2::try_new(tangent_upper_right, tangent_lower_right.clone()).unwrap(),
                ),
                Curve2::from(LineSeg2::try_new(tangent_lower_right, tangent_start).unwrap()),
            ])
            .unwrap()],
            &[CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            &policy,
        )
        .unwrap()
        .into_value();
        assert_eq!(
            tangent_region
                .classify_algebraic_point_off_boundary_raw(&dyadic_ray_point, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );
        let line_extension = RationalBezierAlgebraicPointImage2::from_parametric_source(
            rational_line(2, 4),
            parameter.clone(),
            &policy,
        );
        assert_eq!(
            square_region(0, 0, 2, 2)
                .classify_algebraic_point_raw(&line_extension, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );

        let negative_denominator = RationalBezierAlgebraicPointImage2::from_retained_expression(
            parameter.clone(),
            crate::bezier_algebraic_image::parameter_representation(&parameter, &policy),
            vec![Real::zero(), Real::from(-1_i8)],
            vec![Real::zero(), Real::from(-1_i8)],
            vec![Real::from(-1_i8)],
            "test a correlated point with negative projective weight",
        );
        assert_eq!(
            square_region(0, 0, 2, 2)
                .classify_algebraic_point_raw(&negative_denominator, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Inside),
        );

        let outer = square_region(-2, -2, 2, 2);
        let hole = square_region(0, 0, 1, 1);
        let without_arrangement_provenance = |region: &CurveRegion2| {
            CurveRegionBoundaryLoop2::new(region.boundary_loops()[0].fragments().to_vec(), &policy)
                .unwrap()
        };
        let holed = CurveRegion2::try_new_with_loop_topology(
            vec![
                without_arrangement_provenance(&outer),
                without_arrangement_provenance(&hole),
            ],
            vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
            vec![FillRule::NonZero; 2],
            vec![
                crate::bezier_region::CurveBoundaryInteriorSide2::Left,
                crate::bezier_region::CurveBoundaryInteriorSide2::Right,
            ],
        )
        .unwrap();
        assert_eq!(
            holed
                .classify_algebraic_point_raw(&negative_denominator, &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );
    }
}

#[test]
fn cusp_overlap_clipping_maps_partial_carriers_in_both_orientations() {
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let two_fifths = (Real::from(2_i8) / Real::from(5_i8)).unwrap();
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let three_fifths = (Real::from(3_i8) / Real::from(5_i8)).unwrap();
    let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = cusp_test_semicircle(&policy);
        for (second, second_start, second_end, expected_first, expected_second) in [
            (
                first.clone(),
                half.clone(),
                Real::one(),
                (half.clone(), three_quarters.clone()),
                (half.clone(), three_quarters.clone()),
            ),
            (
                first.reversed(),
                Real::zero(),
                two_fifths.clone(),
                (three_fifths.clone(), three_quarters.clone()),
                (two_fifths.clone(), quarter.clone()),
            ),
        ] {
            let Classification::Decided(BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(
                overlap,
            )) = first.pair_intersections(&second, &policy).unwrap()
            else {
                panic!("coincident selected semicircles must overlap");
            };
            let first_carrier = cusp_test_carrier(
                first.clone(),
                quarter.clone(),
                three_quarters.clone(),
                CurveRegionBooleanOperand2::First,
                &policy,
            );
            let second_carrier = cusp_test_carrier(
                second,
                second_start,
                second_end,
                CurveRegionBooleanOperand2::Second,
                &policy,
            );
            let Classification::Decided(Some(clipped)) = CurveCircleOverlap2::Pair(overlap.clone())
                .clipped_ranges(
                    &CurveParameterRange2::new_validated(
                        first_carrier.start.clone(),
                        first_carrier.end.clone(),
                    ),
                    &CurveParameterRange2::new_validated(
                        second_carrier.start.clone(),
                        second_carrier.end.clone(),
                    ),
                    &policy,
                )
                .unwrap()
            else {
                panic!("the carrier fragments retain a positive shared span");
            };
            assert_eq!(
                clipped.0.scalar_endpoints(),
                Some((&expected_first.0, &expected_first.1)),
            );
            assert_eq!(
                clipped.1.scalar_endpoints(),
                Some((&expected_second.0, &expected_second.1)),
            );
        }
    }
}

#[test]
fn injective_carrier_topology_vertex_canonicalizes_unorderable_parameter_aliases() {
    let policy = CurveContext::STRICT;
    let first = shifted_nested_radical_parameter(Real::zero(), &policy);
    let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &policy);
    assert_eq!(
        first
            .cmp_by_refinement_with_policy(&second, &policy)
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Ordering),
    );

    let carrier = injective_test_carrier();
    let mut events = Vec::new();
    push_carrier_event(
        &mut events,
        carrier_parameter(first.clone()),
        Some(7),
        &carrier,
        &policy,
    )
    .unwrap();
    push_carrier_event(
        &mut events,
        carrier_parameter(second.clone()),
        Some(7),
        &carrier,
        &policy,
    )
    .unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].parameter.as_bezier_parameter(), Some(&first));

    assert!(matches!(
        push_carrier_event(
            &mut events,
            carrier_parameter(second),
            Some(8),
            &carrier,
            &policy
        ),
        Err(ExactCurveError::Blocked(_)),
    ));

    let approximate = CurveContext::APPROXIMATE_512;
    let first = shifted_nested_radical_parameter(Real::zero(), &approximate);
    let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &approximate);
    let carrier = injective_test_carrier();
    let mut events = Vec::new();
    push_carrier_event(
        &mut events,
        carrier_parameter(first),
        Some(7),
        &carrier,
        &approximate,
    )
    .unwrap();
    push_carrier_event(
        &mut events,
        carrier_parameter(second),
        Some(7),
        &carrier,
        &approximate,
    )
    .unwrap();
    assert_eq!(events.len(), 1);
}

#[test]
fn parameter_order_uses_exact_translation_before_approximate_512_terminal() {
    let strict = CurveContext::STRICT;
    let first = shifted_sqrt_half_parameter(Real::zero(), &strict);
    let second = shifted_sqrt_half_parameter(dyadic_epsilon(600), &strict);
    assert_eq!(
        first
            .cmp_by_refinement_with_policy(&second, &strict)
            .unwrap(),
        Classification::Decided(Ordering::Less),
    );

    let approximate = CurveContext::APPROXIMATE_512;
    let first = shifted_sqrt_half_parameter(Real::zero(), &approximate);
    let second = shifted_sqrt_half_parameter(dyadic_epsilon(600), &approximate);
    let outcome = crate::policy::resolve_certified_operation(&approximate, |attempt| {
        first.cmp_by_refinement_with_policy(&second, attempt)
    })
    .unwrap();
    assert_eq!(outcome.value, Classification::Decided(Ordering::Less));
    assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
}

#[test]
fn unsupported_parameter_order_obeys_strict_and_approximate_512_policies() {
    let strict = CurveContext::STRICT;
    let first = shifted_nested_radical_parameter(Real::zero(), &strict);
    let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &strict);
    assert_eq!(
        first
            .cmp_by_refinement_with_policy(&second, &strict)
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Ordering),
    );

    let approximate = CurveContext::APPROXIMATE_512;
    let first = shifted_nested_radical_parameter(Real::zero(), &approximate);
    let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &approximate);
    let outcome = crate::policy::resolve_certified_operation(&approximate, |attempt| {
        first.cmp_by_refinement_with_policy(&second, attempt)
    })
    .unwrap();
    assert_eq!(outcome.value, Classification::Decided(Ordering::Equal));
    assert_eq!(
        outcome.certainty,
        crate::CurveCertainty::Approximate512Consumed
    );
}

#[test]
fn noninjective_carrier_retains_distinct_branches_at_one_topology_vertex() {
    let policy = CurveContext::STRICT;
    let carrier = noninjective_test_carrier();
    let mut events = Vec::new();
    push_carrier_event(
        &mut events,
        carrier_parameter(BezierParameter2::Exact(
            (Real::one() / Real::from(4_i8)).expect("nonzero denominator"),
        )),
        Some(7),
        &carrier,
        &policy,
    )
    .unwrap();
    push_carrier_event(
        &mut events,
        carrier_parameter(BezierParameter2::Exact(
            (Real::from(3_i8) / Real::from(4_i8)).expect("nonzero denominator"),
        )),
        Some(7),
        &carrier,
        &policy,
    )
    .unwrap();
    assert_eq!(events.len(), 2);

    let mut all_events = vec![events];
    canonicalize_injective_topology_events(
        &mut all_events,
        std::slice::from_ref(&carrier),
        &policy,
    );
    assert_eq!(all_events[0].len(), 2);
}

#[test]
fn polynomial_control_polygon_certifies_only_monotone_injective_axis() {
    let monotone = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 1),
        Point2::from_values(2, 0),
    ));
    let retraced = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 0),
        Point2::from_values(0, 0),
    ));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            monotone.certified_injective_axis(&policy),
            Ok(Classification::Decided(true))
        ));
        assert!(!matches!(
            retraced.certified_injective_axis(&policy),
            Ok(Classification::Decided(true))
        ));
    }
}

#[test]
fn transitive_topology_merge_canonicalizes_deferred_injective_aliases() {
    let policy = CurveContext::STRICT;
    let first = shifted_nested_radical_parameter(Real::zero(), &policy);
    let second = shifted_nested_radical_parameter(dyadic_epsilon(600), &policy);
    let carriers = [
        injective_test_carrier(),
        injective_test_carrier(),
        injective_test_carrier(),
    ];
    let quarter =
        BezierParameter2::Exact((Real::one() / Real::from(4_i8)).expect("nonzero denominator"));
    let mut events = vec![Vec::new(), Vec::new(), Vec::new()];
    push_contact_carrier_event(
        &mut events[0],
        carrier_parameter(first.clone()),
        Some(1),
        &carriers[0],
        &policy,
    )
    .unwrap();
    push_contact_carrier_event(
        &mut events[1],
        carrier_parameter(quarter.clone()),
        Some(1),
        &carriers[1],
        &policy,
    )
    .unwrap();
    push_contact_carrier_event(
        &mut events[0],
        carrier_parameter(second.clone()),
        Some(2),
        &carriers[0],
        &policy,
    )
    .unwrap();
    push_contact_carrier_event(
        &mut events[2],
        carrier_parameter(quarter.clone()),
        Some(2),
        &carriers[2],
        &policy,
    )
    .unwrap();
    assert_eq!(events[0].len(), 2);

    let mut contacts = vec![
        ContactVertex {
            point: None,
            topology_vertex: 1,
            carrier_indices: [0, 1],
            parameters: [carrier_parameter(first), carrier_parameter(quarter.clone())],
        },
        ContactVertex {
            point: None,
            topology_vertex: 2,
            carrier_indices: [0, 2],
            parameters: [carrier_parameter(second), carrier_parameter(quarter)],
        },
    ];
    replace_topology_vertex(&mut events, &mut contacts, 2, 1);
    canonicalize_injective_topology_events(&mut events, &carriers, &policy);
    validate_carrier_event_separation(&events, &carriers, &policy).unwrap();
    assert_eq!(events[0].len(), 1);
    assert!(contacts.iter().all(|contact| contact.topology_vertex == 1));
}

fn assert_monotone_parallel_pair_proofs_match_complete_solver(
    context: &CurveRegionBooleanContext,
    filled_side_is_left: bool,
) {
    assert_eq!(context.data.pairs.len(), 6);
    assert!(context.data.pairs.iter().all(|pair| {
        context.parallel_pair_is_coordinate_disjoint(pair)
            || context.adjacent_parallel_pair_is_endpoint_only(pair)
    }));
    assert!(
        context
            .data
            .pairs
            .iter()
            .all(|pair| pair.first_carrier_index != pair.second_carrier_index)
    );
    let topology = context
        .build_split_topology()
        .expect("the monotone loop topology must complete");
    assert_eq!(
        context.certified_simple_single_loop_filled_side(&topology),
        Some(filled_side_is_left)
    );

    // Differentially replay the complete pair solver behind every
    // structural omission. Coordinate separation must remove all retained
    // contacts; an adjacent-range proof may leave only the loop vertex
    // that construction already seeded. This keeps the fast proof a
    // specialization of the same authority rather than an alternate
    // intersection definition.
    for replay_policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for pair in &context.data.pairs {
            let first = &context.data.carriers[pair.first_carrier_index];
            let second = &context.data.carriers[pair.second_carrier_index];
            let coordinate_disjoint = context.parallel_pair_is_coordinate_disjoint(pair);
            let adjacent_endpoint = context.adjacent_parallel_pair_is_endpoint_only(pair);
            assert!(coordinate_disjoint || adjacent_endpoint);
            let intersections = first
                .geometry
                .parallel()
                .parallel_intersections(second.geometry.parallel(), &replay_policy)
                .expect("the complete analytic-parallel replay is valid");
            let Classification::Decided(intersections) = intersections else {
                panic!("a structurally omitted pair must have a complete exact replay");
            };
            assert!(intersections.is_complete(), "{intersections:?}");
            assert!(intersections.parameter_components().is_empty());
            assert!(intersections.overlaps().iter().all(|overlap| {
                !(ranges_intersect(&carrier_range(overlap.first_range()), first, &replay_policy)
                    .expect("the first overlap range comparison is decided")
                    && ranges_intersect(
                        &carrier_range(overlap.second_range()),
                        second,
                        &replay_policy,
                    )
                    .expect("the second overlap range comparison is decided"))
            }));
            let retained_contacts = intersections
                .contacts()
                .iter()
                .filter(|contact| {
                    parameter_in_carrier(contact.first_parameter(), first, &replay_policy)
                        .expect("the first contact range comparison is decided")
                        && parameter_in_carrier(contact.second_parameter(), second, &replay_policy)
                            .expect("the second contact range comparison is decided")
                })
                .collect::<Vec<_>>();
            if coordinate_disjoint {
                assert!(retained_contacts.is_empty(), "{retained_contacts:?}");
                continue;
            }

            let fragment_count = context.data.first.boundary_loops()[first.loop_index]
                .fragments()
                .len();
            let expected = if first.fragment_index.checked_add(1) == Some(second.fragment_index) {
                (
                    carrier_traversal_end(first),
                    carrier_traversal_start(second),
                )
            } else {
                assert_eq!(first.fragment_index, 0);
                assert_eq!(second.fragment_index.checked_add(1), Some(fragment_count));
                (
                    carrier_traversal_start(first),
                    carrier_traversal_end(second),
                )
            };
            assert!(retained_contacts.len() <= 1, "{retained_contacts:?}");
            assert!(retained_contacts.iter().all(|contact| {
                contact.first_parameter() == expected.0 && contact.second_parameter() == expected.1
            }));
        }
    }
}

#[test]
fn selected_parallel_arrangement_splits_interior_cusps() {
    let parallel = QuadraticBezier2::new(
        Point2::from_values(-1, 1),
        Point2::from_values(0, -1),
        Point2::from_values(1, 1),
    )
    .parallel_left(Real::one())
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let range = CurveParameterRange2::unit();
        let analysis = decided(
            parallel
                .singularity_analysis_with_policy(&range, &policy)
                .unwrap(),
        );
        assert!(analysis.source_is_regular());
        assert_eq!(analysis.parallel_cusps().len(), 2);
        let points = [Real::zero(), Real::one()].map(|parameter| {
            CurvePoint2::from(decided(
                parallel.point_at_with_policy(&parameter, &policy).unwrap(),
            ))
        });
        let fragment = CurveSupport2::Parallel(parallel.clone())
            .restrict_certified(range, Some(points.clone()), false, &policy)
            .unwrap();
        let chord = BezierSplitFragment2::AlgebraicChord(decided(
            crate::BezierAlgebraicChord2::try_new(points[1].clone(), points[0].clone(), &policy)
                .unwrap(),
        ));
        for reversed in [false, true] {
            let mut fragments = vec![fragment.clone(), chord.clone()];
            if reversed {
                fragments = fragments
                    .into_iter()
                    .rev()
                    .map(|fragment| fragment.reversed().unwrap())
                    .collect();
            }
            let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed {
                    crate::bezier_region::CurveBoundaryInteriorSide2::Right
                } else {
                    crate::bezier_region::CurveBoundaryInteriorSide2::Left
                }],
            )
            .unwrap();
            let context = CurveRegionBooleanContext::try_new_unary(&region, &policy).unwrap();
            let topology = context.build_split_topology().unwrap();
            let mut cusp_endpoint_visits = 0;
            for piece in topology.split_fragments.iter().flatten() {
                let CurveSupport2::Parallel(source) = CurveSupport2::from_fragment(&piece.fragment)
                else {
                    continue;
                };
                let range = piece.fragment.curve_region_parameter_range();
                let analysis = decided(
                    source
                        .singularity_analysis_with_policy(&range, &policy)
                        .unwrap(),
                );
                for cusp in analysis.parallel_cusps() {
                    let cusp = CurveParameter2::from(cusp.clone());
                    assert!(
                        [range.start(), range.end()].into_iter().any(|endpoint| {
                            cusp.same_value(endpoint, &policy).unwrap()
                                == Classification::Decided(true)
                        }),
                        "an arrangement edge must not hide an interior offset cusp"
                    );
                    cusp_endpoint_visits += 1;
                }
            }
            assert_eq!(
                cusp_endpoint_visits, 4,
                "both cusp branches retain their endpoint evidence"
            );
            let normalized = region.regularized_region_with_policy(&policy).unwrap();
            assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
            assert!(!normalized.value.is_empty());
            let expanded = normalized
                .value
                .offset_with_policy(
                    (Real::one() / Real::from(64)).unwrap(),
                    &crate::OffsetCornerStyle2::Round,
                    &policy,
                )
                .unwrap();
            assert_eq!(expanded.certainty, crate::CurveCertainty::Certified);
            assert!(!expanded.value.is_empty());
        }
    }
}

#[test]
fn cusp_separated_parallel_contacts_are_not_declared_distinct() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    // P(s)=(x,x²), x=4s-2, has a strictly increasing source x.
    // At distance 1, its parallel visits (0,5/4) at
    // s=(2±sqrt(3)/2)/4. Each outer third is regular and injective,
    // but the interval joining those visits crosses two parallel cusps.
    let parallel = QuadraticBezier2::new(
        Point2::from_values(-2, 4),
        Point2::from_values(0, -4),
        Point2::from_values(2, 4),
    )
    .parallel_left(Real::one())
    .unwrap();
    let delta = (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap();
    let left = ((Real::from(2) - &delta) / Real::from(4)).unwrap();
    let right = ((Real::from(2) + delta) / Real::from(4)).unwrap();
    let node = CurvePoint2::from(Point2::new(Real::zero(), q(5, 4)));
    let horizontal = QuadraticBezier2::new(
        Point2::new((-1).into(), q(5, 4)),
        Point2::new(0.into(), q(5, 4)),
        Point2::new(1.into(), q(5, 4)),
    );
    let carrier = |geometry, start: Real, end: Real| RegionCarrier {
        operand: CurveRegionBooleanOperand2::First,
        loop_index: 0,
        fragment_index: 0,
        family: CurveFamily2::QuadraticBezier,
        geometry,
        start: start.into(),
        end: end.into(),
        reversed: false,
        filled_side_is_left: true,
        selected_fiber_endpoint_points: None,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for parameter in [&left, &right] {
            let point = decided(parallel.point_at_with_policy(parameter, &policy).unwrap());
            assert_eq!(
                CurvePoint2::from(point).same_point(&node, &policy),
                Classification::Decided(true)
            );
        }
        let carriers = [
            carrier(
                CurveSupport2::Parallel(parallel.clone()),
                Real::zero(),
                q(1, 3),
            ),
            carrier(
                CurveSupport2::Parallel(parallel.clone()),
                q(2, 3),
                Real::one(),
            ),
            carrier(
                CurveSupport2::Bezier(BezierSubcurve2::Quadratic(horizontal.clone())),
                Real::zero(),
                Real::one(),
            ),
        ];
        for carrier in &carriers[..2] {
            let analysis = decided(
                parallel
                    .singularity_analysis_with_policy(&carrier.range(), &policy)
                    .unwrap(),
            );
            assert!(analysis.source_is_regular() && analysis.parallel_is_cusp_free());
            assert!(carrier_has_certified_injective_image(carrier, &policy));
        }
        let half = CurveParameter2::from(q(1, 2));
        let existing = ContactVertex {
            point: Some(node.clone()),
            topology_vertex: 0,
            carrier_indices: [0, 2],
            parameters: [left.clone().into(), half.clone()],
        };
        assert!(
            !contacts_decided_distinct_from_carriers(
                &existing,
                [1, 2],
                [&right.clone().into(), &half],
                &carriers,
                &policy,
            )
            .unwrap(),
            "two branch visits share the exact same point"
        );
    }
}

#[test]
fn monotone_parallel_ranges_remove_only_proven_unary_pairs() {
    let policy = CurveContext::STRICT;
    let tenth = (Real::one() / Real::from(10_i8)).expect("nonzero denominator");
    let sources = [
        QuadraticBezier2::new(
            Point2::from_values(1, 0),
            Point2::from_values(1, 1),
            Point2::from_values(0, 1),
        ),
        QuadraticBezier2::new(
            Point2::from_values(0, 1),
            Point2::from_values(-1, 1),
            Point2::from_values(-1, 0),
        ),
        QuadraticBezier2::new(
            Point2::from_values(-1, 0),
            Point2::from_values(-1, -1),
            Point2::from_values(0, -1),
        ),
        QuadraticBezier2::new(
            Point2::from_values(0, -1),
            Point2::from_values(1, -1),
            Point2::from_values(1, 0),
        ),
    ];
    let fragments = sources
        .into_iter()
        .map(|source| {
            let parallel = source
                .parallel_left(-tenth.clone())
                .expect("valid exact parallel");
            BezierSplitFragment2::AnalyticParallel(BezierParallelFragment2::from_certified_range(
                parallel,
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                false,
            ))
        })
        .collect();
    let region = CurveRegion2::new(vec![
        CurveRegionBoundaryLoop2::new(fragments, &policy).expect("connected exact parallel loop"),
    ])
    .expect("valid raw region")
    .with_certified_loop_roles(vec![CurveRegionLoopRole::Material])
    .expect("valid material role")
    .with_certified_filled_side_is_left(vec![true])
    .expect("valid filled-side evidence");
    let context = CurveRegionBooleanContext::try_new_unary(&region, &policy)
        .expect("valid unary arrangement");

    assert_monotone_parallel_pair_proofs_match_complete_solver(&context, true);

    let reversed_fragments = region.boundary_loops()[0]
        .fragments()
        .iter()
        .rev()
        .map(|fragment| fragment.reversed().expect("exact traversal reversal"))
        .collect();
    let reversed_region = CurveRegion2::new(vec![
        CurveRegionBoundaryLoop2::new(reversed_fragments, &policy)
            .expect("connected reversed exact parallel loop"),
    ])
    .expect("valid reversed raw region")
    .with_certified_loop_roles(vec![CurveRegionLoopRole::Material])
    .expect("valid reversed material role")
    .with_certified_filled_side_is_left(vec![false])
    .expect("valid reversed filled-side evidence");
    let reversed_context = CurveRegionBooleanContext::try_new_unary(&reversed_region, &policy)
        .expect("valid reversed unary arrangement");
    assert_monotone_parallel_pair_proofs_match_complete_solver(&reversed_context, false);
}

fn square_region(min_x: i8, min_y: i8, max_x: i8, max_y: i8) -> CurveRegion2 {
    let points = [
        Point2::from_values(min_x, min_y),
        Point2::from_values(max_x, min_y),
        Point2::from_values(max_x, max_y),
        Point2::from_values(min_x, max_y),
    ];
    let curves = (0..points.len())
        .map(|index| {
            Curve2::from(
                LineSeg2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                )
                .unwrap(),
            )
        })
        .collect();
    CurveRegion2::try_from_boundary_paths_with_policy(
        &[CurvePath2::try_new(curves).unwrap()],
        crate::FillRule::EvenOdd,
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

fn half_point(x: i8, y: i8) -> Point2 {
    let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
    Point2::new(Real::from(x) * &half, Real::from(y) * &half)
}

#[test]
fn xor_composition_fallback_removes_coincident_seams() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let assert_locations =
            |region: &CurveRegion2, samples: &[(Point2, RegionPointLocation)]| {
                assert!(region.has_regularized_filled_left_topology(&policy));
                assert_eq!(region.regularized_region_raw(&policy).unwrap(), *region);
                for (point, expected) in samples {
                    assert_eq!(
                        region.classify_point_raw(point, &policy).unwrap(),
                        Classification::Decided(*expected),
                        "policy {policy:?} at {point:?}"
                    );
                }
            };

        let left = square_region(0, 0, 2, 2);
        let right = square_region(1, 0, 3, 2);
        let union = left
            .boolean_region_raw(&right, BooleanOp::Union, &policy)
            .unwrap();
        let intersection = left
            .boolean_region_raw(&right, BooleanOp::Intersection, &policy)
            .unwrap();
        let context = CurveRegionBooleanContext::try_new(&left, &right, &policy).unwrap();
        let xor = context
            .compose_xor_from_exact_regions(&union, &intersection)
            .unwrap();
        let public_xor = left
            .boolean_region_raw(&right, BooleanOp::Xor, &policy)
            .unwrap();
        assert_eq!(
            xor.len(),
            2,
            "the shared horizontal seams split into two loops"
        );
        assert_eq!(public_xor.len(), xor.len());
        let overlap_samples = [
            (half_point(1, 2), RegionPointLocation::Inside),
            (half_point(3, 2), RegionPointLocation::Outside),
            (half_point(5, 2), RegionPointLocation::Inside),
            (half_point(3, 0), RegionPointLocation::Outside),
            (Point2::from_values(1, 1), RegionPointLocation::Boundary),
            (Point2::from_values(0, 1), RegionPointLocation::Boundary),
            (Point2::from_values(3, 1), RegionPointLocation::Boundary),
        ];
        assert_locations(&xor, &overlap_samples);
        assert_locations(&public_xor, &overlap_samples);

        let identical = context
            .compose_xor_from_exact_regions(&left, &left)
            .unwrap();
        assert!(identical.is_empty());
        assert_eq!(
            identical
                .classify_point_raw(&Point2::from_values(1, 1), &policy)
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside)
        );

        let outer = square_region(0, 0, 6, 6);
        let inner = square_region(1, 1, 2, 2);
        let nested_union = outer
            .boolean_region_raw(&inner, BooleanOp::Union, &policy)
            .unwrap();
        let nested_intersection = outer
            .boolean_region_raw(&inner, BooleanOp::Intersection, &policy)
            .unwrap();
        let nested = CurveRegionBooleanContext::try_new(&outer, &inner, &policy)
            .unwrap()
            .compose_xor_from_exact_regions(&nested_union, &nested_intersection)
            .unwrap();
        assert_eq!(nested.len(), 2);
        assert_locations(
            &nested,
            &[
                (half_point(1, 6), RegionPointLocation::Inside),
                (half_point(3, 3), RegionPointLocation::Outside),
                (Point2::from_values(0, 3), RegionPointLocation::Boundary),
                (Point2::from_values(1, 1), RegionPointLocation::Boundary),
            ],
        );
    }
}

#[test]
fn operand_bounds_are_lazy_shared_and_policy_neutral() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = square_region(0, 0, 4, 4);
        let second = square_region(10, 10, 14, 14);
        let context = CurveRegionBooleanContext::try_new(&first, &second, &policy).unwrap();
        assert!(
            context
                .data
                .operand_bounds
                .iter()
                .all(|bounds| bounds.get().is_none())
        );
        for index in 0..context.data.carriers.len() {
            let result = resolve_certified_operation(&policy, |_| {
                Ok::<_, ExactCurveError>(context.carrier_bounds_are_outside_other_region(index))
            })
            .unwrap();
            assert!(result.value);
            assert_eq!(result.certainty, crate::CurveCertainty::Certified);
        }
        for bounds in &context.data.operand_bounds {
            let bounds = bounds.get().unwrap();
            assert!(matches!(bounds.refinements[0].get(), Some(Some(_))));
            assert!(
                bounds.refinements[1..]
                    .iter()
                    .all(|bounds| bounds.get().is_none())
            );
        }

        let mut unresolved = CurveRegionBooleanContext::try_new(&first, &second, &policy).unwrap();
        unresolved.data.carriers[unresolved.data.first_carrier_count].bounds =
            OnceLock::from(Classification::Uncertain(UncertaintyReason::Unsupported));
        assert!(unresolved.carrier_bounds_are_outside_other_region(0));
        assert!(unresolved.data.operand_bounds[0].get().is_none());
        let bounds = unresolved.data.operand_bounds[1].get().unwrap();
        assert!(bounds.refinements[0].get().is_none());
        assert!(matches!(bounds.refinements[1].get(), Some(Some(_))));
        assert!(
            bounds.refinements[2..]
                .iter()
                .all(|bounds| bounds.get().is_none())
        );
        unresolved.data.carriers[unresolved.data.first_carrier_count].bounds = OnceLock::new();
        assert!(unresolved.carrier_bounds_are_outside_other_region(0));
        assert!(matches!(
            unresolved.data.operand_bounds[1].get().unwrap().refinements[0].get(),
            Some(Some(_))
        ));

        let intersecting = square_region(2, 2, 6, 6);
        let context = CurveRegionBooleanContext::try_new(&first, &intersecting, &policy).unwrap();
        // The right edge intersects the other operand's exact envelope at
        // every refinement. Replaying it cannot become an absence proof.
        // Both operands are exact straight chords, so refinement cannot
        // tighten either envelope after the decided level-zero overlap.
        for _ in 0..2 {
            assert!(!context.carrier_bounds_are_outside_other_region(1));
        }
        let refinements = &context.data.operand_bounds[1].get().unwrap().refinements;
        assert!(matches!(refinements[0].get(), Some(Some(_))));
        assert!(refinements[1..].iter().all(|bounds| bounds.get().is_none()));

        let empty = CurveRegion2::default();
        let context = CurveRegionBooleanContext::try_new(&first, &empty, &policy).unwrap();
        assert!(context.carrier_bounds_are_outside_other_region(0));
        assert!(matches!(
            context.data.operand_bounds[1].get().unwrap().refinements[0].get(),
            Some(None)
        ));
    }
}

#[test]
fn retained_aabb_candidates_match_cartesian_pairs_in_authored_order() {
    let region = |offset, policy: &CurveContext| {
        let contours = (0..16)
            .map(|index| {
                let x = (index % 4) * 10 + offset;
                let y = (index / 4) * 10;
                crate::Contour2::from_bulge_vertices(
                    &[(x, y), (x + 4, y), (x + 4, y + 4), (x, y + 4)].map(|(x, y)| {
                        crate::BulgeVertex2::new(Point2::from_values(x, y), Real::zero())
                    }),
                )
                .unwrap()
            })
            .collect();
        let result =
            CurveRegion2::try_from_native_contours_with_policy(contours, Vec::new(), policy)
                .unwrap();
        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
        result.into_value()
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = region(0, &policy);
        // Each pair of components shares an edge and its two endpoints.
        let second = region(4, &policy);
        let context = CurveRegionBooleanContext::try_new(&first, &second, &policy).unwrap();
        assert_eq!(context.data.authored_carrier_pair_count, 4_096);
        assert!(!context.data.pairs.is_empty());
        let mut carriers = context.data.carriers;
        let first_count = context.data.first_carrier_count;
        // Exercise both an unknown query and an unknown indexed box. The
        // exact pair kernel may still reject them after local refinement.
        for index in [3, first_count + 7] {
            carriers[index].bounds =
                OnceLock::from(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let indexed = build_cross_operand_carrier_pairs(&carriers, first_count, &policy).unwrap();
        let curves = prepare_bezier_carrier_curves(&carriers, &policy).unwrap();
        let mut cache = CurveIntersectionBatchCache::default();
        let mut cartesian = Vec::new();
        for first_index in 0..first_count {
            for second_index in first_count..carriers.len() {
                if let Some(pair) = build_candidate_carrier_pair(
                    &carriers,
                    &curves,
                    first_index,
                    second_index,
                    &policy,
                    &mut cache,
                )
                .unwrap()
                {
                    cartesian.push((pair.first_carrier_index, pair.second_carrier_index));
                }
            }
        }
        for pairs in [&context.data.pairs, &indexed] {
            assert_eq!(
                pairs
                    .iter()
                    .map(|pair| (pair.first_carrier_index, pair.second_carrier_index))
                    .collect::<Vec<_>>(),
                cartesian
            );
        }
        let unary_carriers = &carriers[..first_count];
        let unary = build_unary_carrier_pairs(unary_carriers, &policy).unwrap();
        let mut complete_unary = Vec::new();
        for first_index in 0..first_count {
            for second_index in first_index + 1..first_count {
                if let Some(pair) = build_candidate_carrier_pair(
                    unary_carriers,
                    &curves[..first_count],
                    first_index,
                    second_index,
                    &policy,
                    &mut cache,
                )
                .unwrap()
                {
                    complete_unary.push((pair.first_carrier_index, pair.second_carrier_index));
                }
            }
            if !carrier_has_certified_injective_image(&unary_carriers[first_index], &policy) {
                complete_unary.push((first_index, first_index));
            }
        }
        assert_eq!(
            unary
                .iter()
                .map(|pair| (pair.first_carrier_index, pair.second_carrier_index))
                .collect::<Vec<_>>(),
            complete_unary
        );
        let evidence = first
            .intersect_region_with_policy(&second, &policy)
            .unwrap();
        assert_eq!(evidence.certainty, crate::CurveCertainty::Certified);
        assert!(evidence.value.is_complete());
        assert_eq!(evidence.value.overlaps().len(), 16);
        assert!(!evidence.value.contacts().is_empty());
    }
}

#[test]
fn native_region_fast_path_matches_forced_general_arrangement() {
    let first = square_region(0, 0, 4, 4);
    let second = square_region(2, 0, 6, 4);
    let policy = CurveContext::STRICT;
    let operations = [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ];
    let fast = operations.map(|operation| {
        first
            .boolean_region_raw(&second, operation, &policy)
            .unwrap()
    });

    let general = CurveRegionBooleanContext::try_new(&first, &second, &policy)
        .unwrap()
        .build_boolean_regions()
        .unwrap();
    assert!(general.candidate_carrier_pair_count() > 0);
    assert!(general.topology_fragment_count() > 0);
    for (operation, fast_region) in operations.into_iter().zip(&fast) {
        let general_region = general.region(operation);
        assert_eq!(
            decided(
                fast_region
                    .signed_area_with_policy(&policy)
                    .unwrap()
                    .into_value()
            ),
            decided(
                general_region
                    .signed_area_with_policy(&policy)
                    .unwrap()
                    .into_value()
            )
        );
        for x_numerator in -2_i8..=14 {
            for y_numerator in -2_i8..=10 {
                let point = Point2::new(
                    (Real::from(x_numerator) / Real::from(2_i8)).unwrap(),
                    (Real::from(y_numerator) / Real::from(2_i8)).unwrap(),
                );
                assert_eq!(
                    fast_region
                        .classify_point_with_policy(&point.clone().into(), &policy)
                        .unwrap()
                        .into_value(),
                    general_region
                        .classify_point_with_policy(&point.clone().into(), &policy)
                        .unwrap()
                        .into_value(),
                    "forced-general {operation:?} differs at {point:?}"
                );
            }
        }
    }
}

fn direction(carrier_index: usize, follows_carrier: bool) -> BooleanArrangementFragmentDirection {
    BooleanArrangementFragmentDirection {
        carrier_index,
        follows_carrier,
        start_contact_branch: None,
        end_contact_branch: None,
    }
}

fn contact_direction(carrier_index: usize, follows_carrier: bool) -> CertifiedContactDirection {
    CertifiedContactDirection {
        branch: if carrier_index == 3 {
            TransitionContactBranch::First
        } else {
            TransitionContactBranch::Second
        },
        follows_carrier,
    }
}

fn vector(direction: BooleanArrangementFragmentDirection, crossing_is_positive: bool) -> (i8, i8) {
    let vector = if direction.carrier_index == 3 {
        (1, 0)
    } else if crossing_is_positive {
        (0, 1)
    } else {
        (0, -1)
    };
    if direction.follows_carrier {
        vector
    } else {
        (-vector.0, -vector.1)
    }
}

fn numerical_turn_preference(
    base: (i8, i8),
    first: (i8, i8),
    second: (i8, i8),
    filled_left_faces: bool,
) -> Option<bool> {
    let half = |candidate: (i8, i8)| {
        let cross = base.0 * candidate.1 - base.1 * candidate.0;
        if cross > 0 {
            0
        } else if cross < 0 {
            1
        } else if base.0 * candidate.0 + base.1 * candidate.1 > 0 {
            0
        } else {
            1
        }
    };
    let first_half = half(first);
    let second_half = half(second);
    if first_half != second_half {
        return Some(first_half < second_half);
    }
    match (first.0 * second.1 - first.1 * second.0).cmp(&0) {
        Ordering::Greater => Some(!filled_left_faces),
        Ordering::Less => Some(filled_left_faces),
        Ordering::Equal => None,
    }
}

#[test]
fn classified_crossing_side_recovers_oriented_tangent_cross() {
    assert_eq!(
        transverse_cross_from_locations(
            RegionPointLocation::Outside,
            RegionPointLocation::Inside,
            true,
        ),
        Some(true)
    );
    assert_eq!(
        transverse_cross_from_locations(
            RegionPointLocation::Inside,
            RegionPointLocation::Outside,
            true,
        ),
        Some(false)
    );
    assert_eq!(
        transverse_cross_from_locations(
            RegionPointLocation::Outside,
            RegionPointLocation::Inside,
            false,
        ),
        Some(false)
    );
    assert_eq!(
        transverse_cross_from_locations(
            RegionPointLocation::Inside,
            RegionPointLocation::Outside,
            false,
        ),
        Some(true)
    );
}

#[test]
fn transverse_contact_certificates_seed_both_operand_faces() {
    fn split(start: usize, end: usize) -> ClassifiedSplitCarrierFragment {
        ClassifiedSplitCarrierFragment {
            split: SplitCarrierFragment {
                fragment: BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                            .unwrap(),
                    )),
                },
                start_topology_vertex: Some(start),
                end_topology_vertex: Some(end),
            },
            location: None,
        }
    }

    let policy = CurveContext::STRICT;
    let first_region = square_region(0, 0, 2, 2);
    let second_region = square_region(1, 1, 3, 3);
    let mut context =
        CurveRegionBooleanContext::try_new(&first_region, &second_region, &policy).unwrap();
    let first_carrier = 0;
    let second_carrier = context.data.first_carrier_count;
    let vertex = 17;
    for source_cross_is_positive in [false, true] {
        for first_reversed in [false, true] {
            for second_reversed in [false, true] {
                for first_filled_left in [false, true] {
                    for second_filled_left in [false, true] {
                        context.data.carriers[first_carrier].reversed = first_reversed;
                        context.data.carriers[second_carrier].reversed = second_reversed;
                        context.data.carriers[first_carrier].filled_side_is_left =
                            first_filled_left;
                        context.data.carriers[second_carrier].filled_side_is_left =
                            second_filled_left;
                        let mut fragments = vec![Vec::new(); context.data.carriers.len()];
                        fragments[first_carrier] = vec![split(1, vertex), split(vertex, 2)];
                        fragments[second_carrier] = vec![split(3, vertex), split(vertex, 4)];
                        let contacts = HashMap::from([(
                            vertex,
                            TransitionContactCandidate {
                                first_carrier,
                                second_carrier,
                                interior_on_both_carriers: true,
                                certified_transverse: true,
                                cross_is_positive: Some(source_cross_is_positive),
                                tangent_dot_is_positive: None,
                                second_side_of_first: None,
                                self_parameters: None,
                            },
                        )]);
                        context
                            .seed_transverse_boolean_locations(&mut fragments, &contacts)
                            .unwrap();

                        let traversal_cross_is_positive =
                            source_cross_is_positive ^ first_reversed ^ second_reversed;
                        let first_before =
                            boolean_location(traversal_cross_is_positive == second_filled_left);
                        let second_before =
                            boolean_location(traversal_cross_is_positive != first_filled_left);
                        assert_eq!(fragments[first_carrier][0].location, Some(first_before));
                        assert_eq!(
                            fragments[first_carrier][1].location,
                            toggled_region_location(first_before),
                        );
                        assert_eq!(fragments[second_carrier][0].location, Some(second_before),);
                        assert_eq!(
                            fragments[second_carrier][1].location,
                            toggled_region_location(second_before),
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn contact_point_bounds_reject_disjoint_lazy_sources() {
    let policy = CurveContext::STRICT;
    let parameter = sqrt_half_parameter(&policy);
    let first_curve = rational_line(0, 1);
    let second_curve = rational_line(2, 3);
    let first = CurvePoint2::from(RationalBezierAlgebraicPointImage2::from_parametric_source(
        first_curve.clone(),
        parameter.clone(),
        &policy,
    ));
    let second = CurvePoint2::from(RationalBezierAlgebraicPointImage2::from_parametric_source(
        second_curve.clone(),
        parameter.clone(),
        &policy,
    ));

    assert!(
        parameter
            .cached_rational_bezier_point_image(&first_curve)
            .is_none()
    );
    assert!(
        parameter
            .cached_rational_bezier_point_image(&second_curve)
            .is_none()
    );
    assert_eq!(
        first.same_point(&second, &policy),
        Classification::Decided(false)
    );
    assert!(
        parameter
            .cached_rational_bezier_point_image(&first_curve)
            .is_none()
    );
    assert!(
        parameter
            .cached_rational_bezier_point_image(&second_curve)
            .is_none()
    );
}

#[test]
fn identical_injective_source_parameters_compare_without_materialization() {
    let policy = CurveContext::STRICT;
    let parameter = sqrt_half_parameter(&policy);
    let curve = rational_line(0, 1);
    let first = CurvePoint2::from(RationalBezierAlgebraicPointImage2::from_parametric_source(
        curve.clone(),
        parameter.clone(),
        &policy,
    ));
    let second = CurvePoint2::from(RationalBezierAlgebraicPointImage2::from_parametric_source(
        curve.clone(),
        parameter.clone(),
        &policy,
    ));

    assert_eq!(
        first.same_point(&second, &policy),
        Classification::Decided(true)
    );
    assert!(
        parameter
            .cached_rational_bezier_point_image(&curve)
            .is_none()
    );
}

#[test]
fn nontransverse_point_touch_certifies_authored_loop_successors() {
    let split = |start_x, end_x, start_vertex, end_vertex| SplitCarrierFragment {
        fragment: BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(
                    Point2::from_values(start_x, 0),
                    Point2::from_values(end_x, 0),
                )
                .unwrap(),
            )),
        },
        start_topology_vertex: Some(start_vertex),
        end_topology_vertex: Some(end_vertex),
    };
    let source_splits = [
        split(-1, 0, 0, 7),
        split(0, -1, 7, 0),
        split(1, 0, 1, 7),
        split(0, 1, 7, 1),
    ];
    let topology = CurveRegionBooleanTopology {
        split_fragments: source_splits
            .iter()
            .cloned()
            .map(|split| {
                vec![ClassifiedSplitCarrierFragment {
                    split,
                    location: Some(RegionPointLocation::Outside),
                }]
            })
            .collect(),
        overlaps: Vec::new(),
        transverse_contacts: HashMap::new(),
        point_classification_count: 0,
    };
    let mut carriers = (0..4).map(|_| injective_test_carrier()).collect::<Vec<_>>();
    for (index, carrier) in carriers.iter_mut().enumerate() {
        carrier.operand = if index < 2 {
            CurveRegionBooleanOperand2::First
        } else {
            CurveRegionBooleanOperand2::Second
        };
        carrier.fragment_index = index % 2;
    }

    for follows_carrier in [false, true] {
        let source_order = if follows_carrier {
            [0, 1, 2, 3]
        } else {
            [1, 0, 3, 2]
        };
        let mut directions = Vec::new();
        let fragments = source_order
            .into_iter()
            .map(|carrier_index| {
                directions.push(direction(carrier_index, follows_carrier));
                let source = &source_splits[carrier_index];
                let fragment = if follows_carrier {
                    source.fragment.clone()
                } else {
                    source.fragment.reversed().unwrap()
                };
                let (start, end) = if follows_carrier {
                    (source.start_topology_vertex, source.end_topology_vertex)
                } else {
                    (source.end_topology_vertex, source.start_topology_vertex)
                };
                BezierArrangementFragment2::new(carrier_index, 0, fragment)
                    .with_topology_vertices(start, end)
            })
            .collect();
        let graph = BezierArrangementGraph2::from_certified_fragments(fragments);
        let starts_by_vertex = arrangement_starts_by_vertex(&graph, None);
        let mut successors = vec![None; graph.len()];
        certify_nontransverse_authored_continuity(
            &mut successors,
            &graph,
            &directions,
            &topology,
            &carriers,
            &starts_by_vertex,
        );
        assert_eq!(successors, [Some(1), None, Some(3), None]);
        let Classification::Decided(traversal) =
            graph.traverse_retained_with_certified_successors(&successors, &CurveContext::STRICT)
        else {
            panic!("the certified point-touch loops must traverse");
        };
        assert_eq!(traversal.chains().len(), 2);
        assert!(traversal.chains().iter().all(|chain| chain.is_closed()));

        let mut changed_topology = topology.clone();
        changed_topology.split_fragments[source_order[1]][0].location =
            Some(RegionPointLocation::Inside);
        let mut rejected = vec![None; graph.len()];
        certify_nontransverse_authored_continuity(
            &mut rejected,
            &graph,
            &directions,
            &changed_topology,
            &carriers,
            &starts_by_vertex,
        );
        assert_eq!(rejected[0], None);
        assert_eq!(rejected[2], Some(3));
    }
}

#[test]
fn certified_branch_order_matches_exact_vector_order() {
    for crossing_is_positive in [false, true] {
        for base_carrier in [3, 7] {
            for base_forward in [false, true] {
                for first_carrier in [3, 7] {
                    for first_forward in [false, true] {
                        for second_carrier in [3, 7] {
                            for second_forward in [false, true] {
                                let base = direction(base_carrier, base_forward);
                                let first = direction(first_carrier, first_forward);
                                let second = direction(second_carrier, second_forward);
                                for filled_left_faces in [false, true] {
                                    assert_eq!(
                                        certified_turn_preference(
                                            contact_direction(base_carrier, base_forward),
                                            contact_direction(first_carrier, first_forward),
                                            contact_direction(second_carrier, second_forward),
                                            crossing_is_positive,
                                            filled_left_faces,
                                        ),
                                        numerical_turn_preference(
                                            vector(base, crossing_is_positive),
                                            vector(first, crossing_is_positive),
                                            vector(second, crossing_is_positive),
                                            filled_left_faces,
                                        ),
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
