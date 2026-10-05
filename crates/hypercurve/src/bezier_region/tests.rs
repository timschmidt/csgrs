use super::*;
use std::cmp::Ordering;

use crate::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, CurveCertainty,
};
use crate::{
    CircularArc2, CubicBezier2, Curve2, CurvePath2, QuadraticBezier2, RationalQuadraticBezier2,
};

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn terminally_unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn algebraic_ray_queries_keep_exterior_roots_and_denominator_orientation() {
    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("exact ray query: {reason:?}"),
        }
    }

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        policy.strict_predicate_pass(|| {
            let root = |coefficients: &[i32], lower, upper| {
                let polynomial = decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        coefficients.iter().copied().map(Real::from).collect(),
                        &policy,
                    )
                    .unwrap(),
                );
                let interval =
                    decided(BezierParameterInterval::try_new(lower, upper, &policy).unwrap());
                decided(
                    BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
                )
            };
            // C(t)=(t/(2-t),t²/(2-t)). All authored control weights
            // are positive, but its denominator is negative after t=2.
            let curve = RationalBezier2::try_new(
                vec![p(0, 0), Point2::new(q(1, 3), Real::zero()), p(1, 1)],
                vec![Real::one(), q(3, 4), q(1, 2)],
            )
            .unwrap();
            let selected_end = root(&[-10, 0, 1], q(31, 10), q(16, 5));
            let boundary = crate::tests::decided(
                curve
                    .point_at_algebraic_parameter(&selected_end, &policy)
                    .unwrap(),
            );
            let boundary = decided(boundary.predicate_evaluator(&policy).unwrap());
            for end in [
                BezierParameter2::Exact(q(16, 5)),
                BezierParameter2::Algebraic(selected_end.clone()),
            ] {
                let range = CurveParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::from(3)).into(),
                    end.into(),
                );
                assert_eq!(
                    curve.denominator_sign(&range),
                    Classification::Decided(RealSign::Negative)
                );
                for reversed in [false, true] {
                    let source = CurveSupport2::Bezier(BezierSubcurve2::Rational(curve.clone()))
                        .restrict_certified(range.clone(), None, reversed, &policy)
                        .unwrap();
                    let fragment =
                        decided(retained_fragment_algebraic_ray_curve(&source, &policy).unwrap());
                    assert_eq!(
                        algebraic_point_on_rational_fragment(&fragment, &boundary, &policy)
                            .unwrap(),
                        Classification::Decided(true),
                        "incidence must retain the exterior selected endpoint"
                    );
                    // The rightward ray from (-4,-44/5) crosses once at
                    // t=(22-2sqrt(11))/5, strictly between 3 and sqrt(10).
                    // The linear owner refines to an Exact scalar; the
                    // quadratic owner exercises selected-fiber replay.
                    for equation in [&[-3, 4][..], &[-1, 0, 2][..]] {
                        let parameter = root(equation, q(1, 2), Real::one());
                        let point = RationalBezierAlgebraicPointImage2::from_retained_expression(
                            parameter.clone(),
                            crate::bezier_algebraic_image::parameter_representation(
                                &parameter, &policy,
                            ),
                            vec![Real::from(-4)],
                            vec![q(-44, 5)],
                            vec![Real::one()],
                            "exact constant query in a selected field",
                        );
                        let predicate = decided(point.predicate_evaluator(&policy).unwrap());
                        assert_eq!(
                            algebraic_point_rational_curve_ray_winding(
                                &fragment,
                                &predicate,
                                &Real::one(),
                                &Real::zero(),
                                &policy,
                            )
                            .unwrap(),
                            Classification::Decided(if reversed { -1 } else { 1 }),
                            "the retained span owns one crossing with its actual denominator sign"
                        );
                    }
                }
            }
        });
    }
}

#[test]
fn certified_boundary_constructors_validate_arrangement_sources() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let fragments = [(p(0, 0), p(1, 0)), (p(1, 0), p(0, 0))]
            .into_iter()
            .map(|(start, end)| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                    start.clone(),
                    start.lerp(&end, q(1, 2)),
                    end,
                )),
            })
            .collect();
        let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
        for sources in [
            Vec::new(),
            vec![CurveRegionFragmentSource2::new(0, 0, 0)],
            vec![
                CurveRegionFragmentSource2::new(0, 0, 0),
                CurveRegionFragmentSource2::new(0, 1, 0),
            ],
        ] {
            assert!(matches!(
                CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                    boundary.fragments.clone(),
                    sources.clone(),
                    &policy
                ),
                Err(CurveError::Topology(_))
            ));
            assert!(matches!(
                CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                    boundary.fragments.clone(),
                    Some(sources),
                    &policy
                ),
                Err(CurveError::Topology(_))
            ));
        }
        let sources = vec![
            CurveRegionFragmentSource2::new(7, 3, 0),
            CurveRegionFragmentSource2::new(8, 3, 1),
        ];
        for result in [
            CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                boundary.fragments.clone(),
                sources.clone(),
                &policy,
            ),
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                boundary.fragments.clone(),
                Some(sources.clone()),
                &policy,
            ),
        ] {
            let result = result.unwrap();
            assert_eq!(result.len(), 2);
            assert_eq!(result.arrangement_sources(), Some(sources.as_slice()));
        }
        assert!(matches!(
            CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
                Vec::new(),
                Vec::new(),
                &policy
            ),
            Err(CurveError::Topology(_))
        ));
        assert!(matches!(
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                Vec::new(),
                Some(Vec::new()),
                &policy
            ),
            Err(CurveError::Topology(_))
        ));
    }
}

#[test]
fn retained_region_constructor_rejects_reused_arrangement_sources_across_loops() {
    let boundary = |vertices: &[Point2], sources| {
        let fragments = (0..vertices.len())
            .map(|i| {
                let start = &vertices[i];
                let end = &vertices[(i + 1) % vertices.len()];
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        start.clone(),
                        start.lerp(end, q(1, 2)),
                        end.clone(),
                    )),
                }
            })
            .collect();
        let boundary = CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT).unwrap();
        CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
            boundary.fragments,
            sources,
            &CurveContext::STRICT,
        )
        .unwrap()
    };
    let outer = boundary(
        &[p(0, 0), p(6, 0), p(6, 6), p(0, 6)],
        vec![
            CurveRegionFragmentSource2::new(0, 0, 0),
            CurveRegionFragmentSource2::new(1, 0, 1),
            CurveRegionFragmentSource2::new(2, 0, 2),
            CurveRegionFragmentSource2::new(3, 0, 3),
        ],
    );
    let inner = boundary(
        &[p(2, 2), p(4, 2), p(4, 4), p(2, 4)],
        vec![
            CurveRegionFragmentSource2::new(0, 1, 0),
            CurveRegionFragmentSource2::new(4, 1, 1),
            CurveRegionFragmentSource2::new(5, 1, 2),
            CurveRegionFragmentSource2::new(6, 1, 3),
        ],
    );
    assert!(matches!(
        CurveRegion2::new(vec![outer, inner]),
        Err(CurveError::Topology(_))
    ));
}

#[test]
fn native_boundary_loops_convert_into_unified_region_validation() {
    let boundary = BezierBoundaryLoop2 {
        fragments: vec![
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(1, -1), p(0, 0))),
        ],
    };
    let boundary: CurveRegionBoundaryLoop2 = boundary.into();
    assert!(matches!(
        CurveRegion2::new(vec![boundary.clone(), boundary]),
        Err(CurveError::Topology(_))
    ));
}

#[test]
fn retained_region_constructor_rejects_duplicate_boundary_loops() {
    let chord = |start: Point2, end: Point2| {
        let Classification::Decided(chord) =
            crate::BezierAlgebraicChord2::try_new(start.into(), end.into(), &CurveContext::STRICT)
                .unwrap()
        else {
            panic!("an exact chord is represented");
        };
        BezierSplitFragment2::AlgebraicChord(chord)
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![chord(p(0, 0), p(1, 0)), chord(p(1, 0), p(0, 0))],
        &CurveContext::STRICT,
    )
    .unwrap();
    assert!(matches!(
        CurveRegion2::new(vec![boundary.clone(), boundary]),
        Err(CurveError::Topology(_))
    ));
}

#[test]
fn selected_fiber_line_images_reuse_exact_real_endpoints_in_both_directions() {
    let half_root_two = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
            vec![-&half_root_two, Real::one()],
            &policy,
        )
        .unwrap() else {
            panic!("exact scalar-tower polynomial");
        };
        let Classification::Decided(interval) =
            BezierParameterInterval::try_new(q(1, 2), Real::one(), &policy).unwrap()
        else {
            panic!("ordered root interval");
        };
        let Classification::Decided(parameter) =
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap()
        else {
            panic!("unique exact scalar root");
        };
        let curve = Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0)))
            .subcurve(
                Real::zero().into(),
                BezierParameter2::Algebraic(parameter).into(),
                &policy,
            )
            .unwrap()
            .into_value();
        let spans = curve
            .source_spans(&policy, CurveOperation2::Subdivision)
            .unwrap();
        assert_eq!(spans.len(), 1);
        assert!(matches!(
            spans[0].fragment,
            BezierSplitFragment2::SelectedFiber(_)
        ));
        let expected = (
            p(0, 0),
            Point2::new(Real::from(2) * &half_root_two, Real::zero()),
        );
        assert_eq!(
            retained_line_fragment_endpoints(&spans[0].fragment, &policy).unwrap(),
            Classification::Decided(expected.clone())
        );
        assert_eq!(
            retained_line_fragment_endpoints(&spans[0].fragment.reversed().unwrap(), &policy)
                .unwrap(),
            Classification::Decided((expected.1, expected.0))
        );
    }
}

#[test]
fn selected_fiber_line_image_fit_rejects_nonmonotone_subrange_excursions() {
    // x(t)=4t(1-t)^3+t^4 stays in [0,1] on the full source chart,
    // but its [1/5,4/5] restriction leaves the interval of its endpoints.
    let source = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 0), p(0, 0), p(0, 0), p(1, 0)],
        vec![Real::one(); 5],
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = q(1, 5);
        let end = q(4, 5);
        let Classification::Decided(first) = source.point_at_classified(&start, &policy) else {
            panic!("finite start");
        };
        let Classification::Decided(last) = source.point_at_classified(&end, &policy) else {
            panic!("finite end");
        };
        let fragment = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::Rational(source.clone()),
                CurveParameterRange2::new_validated(start.into(), end.into()),
                first.into(),
                last.into(),
            ),
        );
        assert!(matches!(
            source.fit_exact_line_image(&policy).unwrap(),
            Classification::Decided(BezierLineImageFitRelation::Fit(_))
        ));
        assert_eq!(
            retained_line_fragment_endpoints(&fragment, &policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        );
    }
}

#[test]
fn material_component_reentry_shares_single_region_evidence() {
    let vertices = [p(0, 0), p(4, 0), p(4, 4), p(0, 4)];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                LineSeg2::try_new(vertices[i].clone(), vertices[(i + 1) % 4].clone())
                    .unwrap()
                    .into()
            })
            .collect(),
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = CurveRegion2::try_from_boundary_paths_with_policy(
            std::slice::from_ref(&path),
            crate::FillRule::EvenOdd,
            &policy,
        )
        .unwrap()
        .into_value();
        let mut current = region.clone();
        for _ in 0..16 {
            let outcome = current.material_components_with_policy(&policy).unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert_eq!(outcome.value.len(), 1);
            current = outcome.into_value().pop().unwrap();
            assert!(Arc::ptr_eq(&region.data, &current.data));
        }
    }
}

#[test]
fn retained_fragment_turn_certificates_own_the_consumed_range() {
    // P(t)=(t,t²-t³/3) has turning polynomial 2-2t and nonzero
    // x derivative. It turns left on [0,1] and right on [2,3].
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), Real::zero()),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(Real::one(), q(2, 3)),
    );
    let rational =
        RationalBezier2::try_from_subcurve(&BezierSubcurve2::Cubic(source.clone())).unwrap();
    let native = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::from(2)),
        end: BezierParameter2::Exact(Real::from(3)),
        curve: BezierSubcurve2::Cubic(source.clone()),
    };
    let range = BezierParameterRange2::new_validated(
        BezierParameter2::Exact(Real::from(2)),
        BezierParameter2::Exact(Real::from(3)),
    );
    let fragments = [
        BezierSplitFragment2::RetainedBezier {
            reversed: false,
            start: range.start().clone(),
            end: range.end().clone(),
            source_curve: BezierSubcurve2::Cubic(source.clone()),
            start_image: None,
            end_image: None,
        },
        BezierSplitFragment2::AnalyticParallel(
            crate::BezierParallelFragment2::from_certified_range(
                source.parallel_left(Real::zero()).unwrap(),
                range.clone(),
                false,
            ),
        ),
        BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::Rational(rational),
                CurveParameterRange2::from_bezier_range(range),
                Point2::new(Real::from(2), q(4, 3)).into(),
                p(3, 0).into(),
            ),
        ),
    ];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // Materialized provenance bounds do not replace its own local chart.
        assert!(fragment_certifies_nonnegative_turn(&native, &policy).unwrap());
        for fragment in &fragments {
            assert!(
                !fragment_certifies_nonnegative_turn(fragment, &policy).unwrap(),
                "the exterior fragment turns strictly right"
            );
            assert!(
                fragment_certifies_nonnegative_turn(&fragment.reversed().unwrap(), &policy)
                    .unwrap(),
                "reversing the exterior range makes its turning positive"
            );
        }
    }
}

#[test]
fn zero_distance_parallel_admission_excludes_source_poles() {
    let source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![Real::one(), -Real::one()]).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parallel = source.parallel_left(Real::zero()).unwrap();
        assert_eq!(
            parallel.point_at(&q(1, 2), &policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        );
        assert!(
            matches!(
                crate::BezierParallelFragment2::try_new(parallel, range, &policy).unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            ),
            "a zero-distance fragment still needs a finite source throughout its range"
        );
    }
}

#[test]
fn finite_parallel_admission_preserves_regular_and_zero_distance_images() {
    let range = |start: Real, end: Real| {
        BezierParameterRange2::new_validated(
            BezierParameter2::Exact(start),
            BezierParameter2::Exact(end),
        )
    };
    let rational =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![Real::one(), -Real::one()]).unwrap();
    let stationary = QuadraticBezier2::new(p(4, 0), p(2, 0), p(1, 0));
    let constant =
        RationalBezier2::try_new(vec![p(3, 7), p(3, 7)], vec![Real::one(), Real::from(2)]).unwrap();
    let parabola = QuadraticBezier2::new(p(0, 4), Point2::new(q(1, 2), Real::from(2)), p(1, 1));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reverse in [false, true] {
            for distance in [Real::zero(), q(1, 10)] {
                let parallel = rational.parallel_left(distance).unwrap();
                let range = if reverse {
                    range(Real::from(3), Real::from(2))
                } else {
                    range(Real::from(2), Real::from(3))
                };
                let Classification::Decided(fragment) =
                    crate::BezierParallelFragment2::try_new(parallel, range, &policy).unwrap()
                else {
                    panic!("a pole outside the finite range cannot block admission")
                };
                assert_eq!(fragment.is_reversed(), reverse);
                assert_eq!(
                    fragment.range().scalar_endpoints(),
                    Some((&Real::from(2), &Real::from(3)))
                );
                assert!(matches!(
                    fragment.representative_point(&policy).unwrap(),
                    Classification::Decided(_)
                ));
            }
        }
        let Classification::Decided(fragment) = crate::BezierParallelFragment2::try_new(
            stationary.parallel_left(Real::zero()).unwrap(),
            range(Real::one(), Real::from(3)),
            &policy,
        )
        .unwrap() else {
            panic!("zero distance preserves a stationary source")
        };
        assert_eq!(
            fragment
                .parallel()
                .point_at(&Real::from(2), &policy)
                .unwrap(),
            Classification::Decided(p(0, 0))
        );
        assert!(
            !fragment_certifies_nonnegative_turn(
                &BezierSplitFragment2::AnalyticParallel(fragment),
                &policy
            )
            .unwrap(),
            "source stationarity in the active range prevents a convexity shortcut"
        );
        assert!(matches!(
            crate::BezierParallelFragment2::try_new(
                stationary.parallel_left(Real::one()).unwrap(),
                range(Real::one(), Real::from(3)),
                &policy,
            ),
            Err(CurveError::Topology(_))
        ));
        let Classification::Decided(fragment) = crate::BezierParallelFragment2::try_new(
            constant.parallel_left(Real::zero()).unwrap(),
            range(Real::from(2), Real::from(3)),
            &policy,
        )
        .unwrap() else {
            panic!("zero distance preserves a finite constant image")
        };
        assert_eq!(
            fragment.representative_point(&policy).unwrap(),
            Classification::Decided(p(3, 7))
        );
        assert!(matches!(
            crate::BezierParallelFragment2::try_new(
                constant.parallel_left(Real::one()).unwrap(),
                range(Real::from(2), Real::from(3)),
                &policy,
            )
            .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        ));

        let parallel = parabola.parallel_left(Real::one()).unwrap();
        let active_range = range(Real::one(), Real::from(3));
        assert!(
            matches!(
                crate::BezierParallelFragment2::try_new(
                    parallel.clone(),
                    active_range.clone(),
                    &policy,
                ),
                Err(CurveError::Topology(_))
            ),
            "interior cusps still require splitting"
        );
        let Classification::Decided(analysis) = parallel
            .singularity_analysis(
                &CurveParameterRange2::from_bezier_range(active_range),
                &policy,
            )
            .unwrap()
        else {
            panic!("finite cusp analysis")
        };
        assert_eq!(analysis.parallel_cusps().len(), 2);
        let parameters = [
            BezierParameter2::Exact(Real::one()),
            analysis.parallel_cusps()[0].clone(),
            analysis.parallel_cusps()[1].clone(),
            BezierParameter2::Exact(Real::from(3)),
        ];
        for endpoints in parameters.windows(2) {
            let Classification::Decided(fragment) = crate::BezierParallelFragment2::try_new(
                parallel.clone(),
                BezierParameterRange2::new_validated(endpoints[0].clone(), endpoints[1].clone()),
                &policy,
            )
            .unwrap() else {
                panic!("cusp endpoints retain their exact boundary authority")
            };
            assert_eq!(fragment.range().start(), &endpoints[0]);
            assert_eq!(fragment.range().end(), &endpoints[1]);
            assert!(matches!(
                fragment.representative_point(&policy).unwrap(),
                Classification::Decided(_)
            ));
        }
    }
}

#[test]
fn boundary_side_rays_preserve_winding_through_reversed_source_charts() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let arc =
            Curve2::from(CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap());
        let spans = arc.native_bezier_fragments(&policy).unwrap().value;
        let source = RationalBezier2::try_from_subcurve(spans[0].native_curve()).unwrap();
        let parameter = q(1, 4);
        let Classification::Decided(point) = source.point_at_classified(&parameter, &policy) else {
            panic!("exact circle representative");
        };
        let Classification::Decided(other) = source.point_at_classified(&q(3, 4), &policy) else {
            panic!("exact second circle point");
        };
        let materialized = |curve| BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve,
        };
        let line = |start, end| {
            materialized(BezierSubcurve2::Quadratic(
                QuadraticBezier2::from_line_segment(LineSeg2::try_new(start, end).unwrap()),
            ))
        };
        for retained in [false, true] {
            for reversed in [false, true] {
                let mut circle = if retained {
                    BezierSplitFragment2::SelectedFiber(
                        crate::bezier_split::BezierSelectedFiberFragment2::new(
                            BezierSelectedFiberSource2::Rational(source.clone()),
                            CurveParameterRange2::unit(),
                            p(1, 0).into(),
                            p(0, 1).into(),
                        ),
                    )
                } else {
                    materialized(BezierSubcurve2::Rational(source.clone()))
                };
                if reversed {
                    circle = circle.reversed().unwrap();
                }
                let (start, end) = if reversed {
                    (p(0, 1), p(1, 0))
                } else {
                    (p(1, 0), p(0, 1))
                };
                let boundary = CurveRegionBoundaryLoop2::new(
                    vec![circle, line(end, p(0, 0)), line(p(0, 0), start)],
                    &policy,
                )
                .unwrap();
                let region = CurveRegion2::try_new_with_loop_topology(
                    vec![boundary],
                    vec![CurveRegionLoopRole::Material],
                    vec![FillRule::NonZero],
                    vec![if reversed {
                        CurveBoundaryInteriorSide2::Right
                    } else {
                        CurveBoundaryInteriorSide2::Left
                    }],
                )
                .unwrap();
                let source_parameter = CurveParameter2::from(if reversed && !retained {
                    Real::one() - &parameter
                } else {
                    parameter.clone()
                });
                for inside in [false, true] {
                    let sign = if inside { Real::one() } else { -Real::one() };
                    let crossing = if inside != reversed {
                        BezierLineCrossingDirection::PositiveToNegative
                    } else {
                        BezierLineCrossingDirection::NegativeToPositive
                    };
                    let result = region
                        .loop_windings_from_boundary_side_ray(
                            &point,
                            (other.x() - point.x()) * &sign,
                            (other.y() - point.y()) * sign,
                            true,
                            crossing,
                            0,
                            0,
                            Some(&source_parameter),
                            &policy,
                        )
                        .unwrap();
                    let result = result.map(|windings| {
                        let location = region
                            .region_location_from_loop_windings(&windings)
                            .unwrap();
                        (windings, location)
                    });
                    assert_eq!(
                        result,
                        Classification::Decided((
                            vec![if inside {
                                if reversed { -1 } else { 1 }
                            } else {
                                0
                            }],
                            if inside {
                                RegionPointLocation::Inside
                            } else {
                                RegionPointLocation::Outside
                            },
                        )),
                        "retained={retained}, reversed={reversed}, inside={inside}, policy={policy:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn retained_and_represented_tangents_share_filled_face_order() {
    // Clockwise order from the reverse of the incoming +X ray. A return
    // along -X is last; equal directions still need higher-order evidence.
    let directions = [
        (-1, 1),
        (0, 1),
        (1, 1),
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
        (-1, 0),
    ];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for turns in 0..4 {
            let rotate = |mut x: i32, mut y: i32| {
                for _ in 0..turns {
                    (x, y) = (-y, x);
                }
                (x, y)
            };
            let (x, y) = rotate(1, 0);
            let base = CurveTangent2::RepresentedDirection((Real::from(x), Real::from(y)));
            let tangents = directions
                .iter()
                .enumerate()
                .map(|(index, &(x, y))| {
                    let (x, y) = rotate(x, y);
                    if index % 2 == 0 {
                        CurveTangent2::RepresentedDirection((Real::from(x * 3), Real::from(y * 3)))
                    } else {
                        let Classification::Decided(chord) = crate::BezierAlgebraicChord2::try_new(
                            CurvePoint2::from(p(7, -4)),
                            CurvePoint2::from(p(7 + 5 * x, -4 + 5 * y)),
                            &policy,
                        )
                        .unwrap() else {
                            panic!("the independently translated tangent chord is nonzero");
                        };
                        let Classification::Decided(tangent) = CurveTangent2::at_boundary_endpoint(
                            &BezierSplitFragment2::AlgebraicChord(chord),
                            true,
                            &policy,
                        )
                        .unwrap() else {
                            panic!("the endpoint retains the chord traversal direction");
                        };
                        tangent
                    }
                })
                .collect::<Vec<_>>();
            for (first_index, first) in tangents.iter().enumerate() {
                for (second_index, second) in tangents.iter().enumerate() {
                    let outcome = crate::policy::resolve_certified_value(&policy, |policy| {
                        base.compare_filled_left_turn(first, second, policy)
                    });
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    assert_eq!(
                        outcome.value,
                        Classification::Decided(first_index.cmp(&second_index))
                    );
                }
            }
        }
    }
}

#[test]
fn boundary_tangent_does_not_assign_a_direction_to_a_stationary_endpoint() {
    let source = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 1))),
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let fragment = if reversed {
                source.reversed().unwrap()
            } else {
                source.clone()
            };
            assert!(matches!(
                CurveTangent2::at_boundary_endpoint(&fragment, !reversed, &policy).unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            ));
            assert!(matches!(
                CurveTangent2::at_boundary_endpoint(&fragment, reversed, &policy).unwrap(),
                Classification::Decided(_)
            ));
        }
    }
}

#[test]
fn even_multiplicity_stationary_source_coalesces_one_offset_span() {
    // With s=2u-1, P(u)=(s^3,s^4) has P'(u)=s^2(6,8s).
    // Cancelling the even common factor leaves the same normal sheet on
    // both sides of u=1/2, so the retained fragments stay split while the
    // stroke composer sees one smooth span.
    let curve = BezierSubcurve2::Rational(
        RationalBezier2::try_new(
            vec![
                p(-1, 1),
                Point2::new(q(1, 2), Real::from(-1_i8)),
                p(0, 1),
                Point2::new(q(-1, 2), Real::from(-1_i8)),
                p(1, 1),
            ],
            vec![Real::one(); 5],
        )
        .unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let spans = exact_offset_spans_from_materialized_curve(&curve, &q(1, 8), &policy).unwrap();
        let Classification::Decided(spans) = spans else {
            panic!("the even stationary source must split exactly");
        };
        let [span] = spans.as_slice() else {
            panic!("the even stationary source must coalesce to one span");
        };
        assert!(span.fragments.len() >= 2);
        assert_eq!(
            span.fragments
                .iter()
                .filter(|fragment| matches!(fragment, BezierSplitFragment2::SelectedFiber(_)))
                .count(),
            2,
        );
    }
}

fn one_fragment_selected_corner_region(reversed: bool, policy: &CurveContext) -> CurveRegion2 {
    let seam = p(0, 0);
    let source = RationalBezier2::try_new(
        vec![seam.clone(), p(4, 0), p(0, 4), seam.clone()],
        vec![Real::one(); 4],
    )
    .expect("the closed cubic selected source is finite");
    let range = CurveParameterRange2::new_validated(
        CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
        CurveParameter2::from(BezierParameter2::Exact(Real::one())),
    );
    let mut fragment = BezierSplitFragment2::SelectedFiber(
        crate::bezier_split::BezierSelectedFiberFragment2::new(
            BezierSelectedFiberSource2::Rational(source),
            range,
            CurvePoint2::from(seam.clone()),
            CurvePoint2::from(seam),
        ),
    );
    if reversed {
        fragment = fragment
            .reversed()
            .expect("the selected closed cubic reverses exactly");
    }
    let boundary = CurveRegionBoundaryLoop2::new(vec![fragment], policy)
        .expect("the selected one-fragment loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .expect("the selected one-fragment loop has authored topology")
}

#[test]
fn retained_selected_corner_trim_keeps_distinct_bounded_fields_local() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let third = (Real::one() / Real::from(3_i8)).unwrap();
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        let cut = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            quarter.clone(),
            1_024,
            &policy,
        );
        let end = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            third.clone(),
            64,
            &policy,
        );
        assert_eq!(
            start.cmp_by_refinement(&cut, &policy).unwrap(),
            Classification::Decided(std::cmp::Ordering::Less),
        );
        assert_eq!(
            cut.cmp_by_refinement(&end, &policy).unwrap(),
            Classification::Decided(std::cmp::Ordering::Less),
        );
        for parameter in [&start, &cut, &end] {
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }

        let source =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let point = |parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                source.clone(),
                parameter,
                &policy,
            ))
        };
        let fragment = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(source.clone()),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_selected_fiber(start.clone()),
                    CurveParameter2::from_selected_fiber(end.clone()),
                ),
                point(start.clone()),
                point(end.clone()),
            ),
        );
        let cut_point = point(cut.clone());
        let trimmed = retained_corner_fragment_trim(
            &fragment,
            CurveParameter2::from_selected_fiber(cut.clone()),
            &cut_point,
            None,
            true,
            CurveOperation2::Chamfer,
            &policy,
        )
        .expect("distinct selected fields must trim without globalizing either endpoint")
        .expect("the strictly interior cut retains a nonempty range");
        let BezierSplitFragment2::SelectedFiber(trimmed) = trimmed else {
            panic!("the compact selected carrier must survive corner reconstruction");
        };
        assert_eq!(trimmed.range().start().as_selected_fiber(), Some(&start));
        assert_eq!(trimmed.range().end().as_selected_fiber(), Some(&cut));
    }
}

#[test]
fn analytic_corner_restrictions_keep_common_scalar_cuts_and_one_source() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 2))
            .parallel_left(q(1, 8))
            .unwrap();
        let Classification::Decided(original) = crate::BezierParallelFragment2::try_new(
            source.clone(),
            BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            ),
            &policy,
        )
        .unwrap() else {
            panic!("the source parallel is regular");
        };
        let parameters =
            [(q(1, 3), 64), (q(1, 4), 1_024), (q(1, 2), 32_768)].map(|(constant, scale)| {
                crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                    constant, scale, &policy,
                )
            });
        for reversed in [false, true] {
            let original = BezierSplitFragment2::AnalyticParallel(if reversed {
                original.reversed()
            } else {
                original.clone()
            });
            let source_curve = Curve2::from_retained_fragment(original.clone());
            let mut fragment = original;
            let mut retained_outer = None;
            for root in &parameters {
                assert!(matches!(
                    root.promoted_bezier_parameter(&policy).unwrap(),
                    Classification::Uncertain(_)
                ));
                let parameter = CurveParameter2::from_selected_fiber(root.clone());
                let point = source_curve.point_at(&parameter, &policy).unwrap();
                assert_eq!(point.certainty, CurveCertainty::Certified);
                fragment = retained_corner_fragment_trim(
                    &fragment,
                    parameter.clone(),
                    &point.value,
                    None,
                    !reversed,
                    CurveOperation2::Chamfer,
                    &policy,
                )
                .expect("common scalar cuts stay in the original analytic chart")
                .expect("the strictly interior cut retains a nonempty range");
                let BezierSplitFragment2::SelectedFiber(selected) = &fragment else {
                    panic!("the common scalar needs its retained point authority");
                };
                assert_eq!(selected.analytic_parallel(), Some(&source));
                assert_eq!(selected.range().end(), &parameter);
                let outer = if reversed {
                    selected.end_point()
                } else {
                    selected.start_point()
                };
                if let Some(retained) = &retained_outer {
                    assert!(
                        outer.shares_storage(retained),
                        "repeated cuts reuse the untouched endpoint image"
                    );
                } else {
                    retained_outer = Some(outer.clone());
                }
                let replay = Curve2::from_retained_fragment(fragment.clone())
                    .point_at(&parameter, &policy)
                    .unwrap();
                let equality = replay.value.coincides_with(&point.value, &policy);
                assert_eq!(equality.certainty, CurveCertainty::Certified);
                assert_eq!(equality.value, Classification::Decided(true));
            }
        }
    }
}

#[test]
fn native_corner_intervals_retain_general_cuts_and_replay_after_trimming() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selected =
            [(q(1, 2), 32_768), (q(1, 4), 1_024), (q(1, 3), 64)].map(|(constant, scale)| {
                let root = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                    constant, scale, &policy,
                );
                assert!(matches!(
                    root.promoted_bezier_parameter(&policy).unwrap(),
                    Classification::Uncertain(_)
                ));
                CurveParameter2::from_selected_fiber(root)
            });
        let sources = [
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 2))),
            BezierSubcurve2::Cubic(CubicBezier2::new(p(0, 0), p(1, 0), p(2, 1), p(3, 3))),
            BezierSubcurve2::RationalQuadratic(
                RationalQuadraticBezier2::try_new(
                    p(0, 0),
                    p(1, 0),
                    p(2, 2),
                    Real::one(),
                    Real::from(2),
                    Real::one(),
                )
                .unwrap(),
            ),
            BezierSubcurve2::Rational(
                RationalBezier2::try_new(
                    vec![p(0, 0), p(1, 0), p(2, 1), p(3, 3), p(4, 4)],
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
        let sqrt_half = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
        for source in sources {
            for reversed in [false, true] {
                let original = BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: source.clone(),
                };
                let original = if reversed {
                    original.reversed().unwrap()
                } else {
                    original
                };
                let source_curve = Curve2::from_retained_fragment(original.clone());
                let BezierSplitFragment2::Materialized { curve, .. } = &original else {
                    unreachable!();
                };
                let rational = RationalBezier2::try_from_subcurve(curve).unwrap();
                for [start, probe, end] in [
                    selected.clone(),
                    [
                        Real::zero().into(),
                        selected[1].clone(),
                        selected[2].clone(),
                    ],
                    [selected[0].clone(), selected[1].clone(), Real::one().into()],
                    [q(1, 4).into(), q(1, 2).into(), sqrt_half.clone().into()],
                ] {
                    let cut = |parameter: CurveParameter2| {
                        let point = source_curve.point_at(&parameter, &policy).unwrap();
                        assert_eq!(point.certainty, CurveCertainty::Certified);
                        CornerTrimCut2 {
                            parameter,
                            point: point.value,
                            placement: CornerPlacement2::Trim,
                            replacement: None,
                        }
                    };
                    let next = cut(start);
                    let previous = cut(end);
                    assert!(
                        retained_single_fragment_corner_cuts_are_separated(
                            &original,
                            &previous,
                            &next,
                            CurveOperation2::Chamfer,
                            &policy,
                        )
                        .unwrap()
                    );
                    let retained = retained_corner_fragment_between_cuts(
                        &original,
                        &previous,
                        &next,
                        CurveOperation2::Chamfer,
                        &policy,
                    )
                    .expect("both native cuts retain their common parameter authority");
                    let BezierSplitFragment2::SelectedFiber(selected) = &retained else {
                        panic!("nonrational cuts keep the original source chart");
                    };
                    assert_eq!(selected.rational_curve(), Some(&rational));
                    assert_eq!(selected.range().start(), &next.parameter);
                    assert_eq!(selected.range().end(), &previous.parameter);
                    assert!(selected.start_point().shares_storage(&next.point));
                    assert!(selected.end_point().shares_storage(&previous.point));
                    let expected = source_curve.point_at(&probe, &policy).unwrap();
                    let actual = Curve2::from_retained_fragment(retained.clone())
                        .point_at(&probe, &policy)
                        .unwrap();
                    let equality = actual.value.coincides_with(&expected.value, &policy);
                    assert_eq!(actual.certainty, CurveCertainty::Certified);
                    assert_eq!(equality.certainty, CurveCertainty::Certified);
                    assert_eq!(equality.value, Classification::Decided(true));
                    let trimmed = retained_corner_fragment_trim(
                        &retained,
                        probe.clone(),
                        &expected.value,
                        None,
                        true,
                        CurveOperation2::Chamfer,
                        &policy,
                    )
                    .expect("the selected interval accepts another exact cut")
                    .expect("the strictly interior cut retains a nonempty range");
                    let BezierSplitFragment2::SelectedFiber(trimmed_source) = &trimmed else {
                        panic!("repeated trimming preserves its selected source");
                    };
                    assert_eq!(trimmed_source.source(), selected.source());
                    assert_eq!(trimmed_source.range().end(), &probe);
                    assert!(trimmed_source.start_point().shares_storage(&next.point));
                    let reversed_trim = Curve2::from_retained_fragment(trimmed.reversed().unwrap());
                    assert!(reversed_trim.start().shares_storage(&expected.value));
                    assert!(reversed_trim.end().shares_storage(&next.point));
                }
            }
        }
    }
}

#[test]
fn corner_trims_drop_consumed_ranges_without_dropping_closed_traces() {
    // P(0) = P(1/2) = (0, 0), but P(1/4) = (21/32, 27/32).
    // The first half is a nonempty closed trace, not a consumed span.
    let polynomial = CubicBezier2::new(p(0, 0), p(2, 3), p(-1, -3), p(-3, 0));
    let rational =
        RationalBezier2::try_from_subcurve(&BezierSubcurve2::Cubic(polynomial.clone())).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for retained in [false, true] {
            for reversed in [false, true] {
                let fragment = if retained {
                    BezierSplitFragment2::SelectedFiber(
                        crate::bezier_split::BezierSelectedFiberFragment2::new(
                            BezierSelectedFiberSource2::Rational(rational.clone()),
                            CurveParameterRange2::unit(),
                            p(0, 0).into(),
                            p(-3, 0).into(),
                        ),
                    )
                } else {
                    BezierSplitFragment2::Materialized {
                        // Source provenance does not change the local unit chart.
                        start: BezierParameter2::Exact(q(1, 5)),
                        end: BezierParameter2::Exact(q(4, 5)),
                        curve: BezierSubcurve2::Cubic(polynomial.clone()),
                    }
                };
                let fragment = if reversed {
                    fragment.reversed().unwrap()
                } else {
                    fragment
                };
                let source = Curve2::from_retained_fragment(fragment.clone());
                for keep_before in [false, true] {
                    let boundary =
                        CurveParameter2::from(if keep_before != fragment.source_is_reversed() {
                            Real::zero()
                        } else {
                            Real::one()
                        });
                    let point = source.point_at(&boundary, &policy).unwrap();
                    assert_eq!(point.certainty, CurveCertainty::Certified);
                    assert!(
                        retained_corner_fragment_trim(
                            &fragment,
                            boundary,
                            &point.value,
                            None,
                            keep_before,
                            CurveOperation2::Chamfer,
                            &policy,
                        )
                        .unwrap()
                        .is_none()
                    );
                }
                let closed = retained_corner_fragment_trim(
                    &fragment,
                    CurveParameter2::from(q(1, 2)),
                    &p(0, 0).into(),
                    None,
                    !reversed,
                    CurveOperation2::Chamfer,
                    &policy,
                )
                .unwrap()
                .expect("a nonempty closed trace survives trimming");
                let closed = Curve2::from_retained_fragment(closed);
                for endpoint in [closed.start(), closed.end()] {
                    let equality = endpoint.coincides_with(&p(0, 0).into(), &policy);
                    assert_eq!(equality.certainty, CurveCertainty::Certified);
                    assert_eq!(equality.value, Classification::Decided(true));
                }
                let probe = CurveParameter2::from(if retained { q(1, 4) } else { q(1, 2) });
                let actual = closed.point_at(&probe, &policy).unwrap();
                let expected = Point2::new(q(21, 32), q(27, 32)).into();
                let equality = actual.value.coincides_with(&expected, &policy);
                assert_eq!(actual.certainty, CurveCertainty::Certified);
                assert_eq!(equality.certainty, CurveCertainty::Certified);
                assert_eq!(equality.value, Classification::Decided(true));
            }
        }
    }
}

#[test]
fn repeated_circle_corner_restrictions_preserve_only_outer_tangency() {
    use crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = crate::BezierAlgebraicCuspSemicircleFragment2::full(
            selected_parallel_normal_circle(&policy),
            &policy,
        )
        .with_certified_tangent_endpoints();
        for reversed in [false, true] {
            for keep_before in [false, true] {
                let mut fragment = BezierSplitFragment2::AlgebraicCuspSemicircle(if reversed {
                    source.reversed()
                } else {
                    source.clone()
                });
                let original = Curve2::from_retained_fragment(fragment.clone());
                let second = if keep_before != reversed {
                    q(1, 4)
                } else {
                    q(3, 4)
                };
                for value in [q(1, 2), second] {
                    let parameter = CurveParameter2::from_algebraic_cusp(
                        BezierAlgebraicCuspSemicircleParameter2::Exact(value),
                    );
                    let point = original.point_at(&parameter, &policy).unwrap();
                    assert_eq!(point.certainty, CurveCertainty::Certified);
                    fragment = retained_corner_fragment_trim(
                        &fragment,
                        parameter,
                        &point.value,
                        None,
                        keep_before,
                        CurveOperation2::Chamfer,
                        &policy,
                    )
                    .unwrap()
                    .expect("the strictly interior cut retains a nonempty circle range");
                    let BezierSplitFragment2::AlgebraicCuspSemicircle(retained) = &fragment else {
                        panic!("circle restriction preserves its exact support");
                    };
                    assert_eq!(retained.certified_tangent_endpoint(true), keep_before);
                    assert_eq!(retained.certified_tangent_endpoint(false), !keep_before);
                    let curve = Curve2::from_retained_fragment(fragment.clone());
                    let (outer, expected) = if keep_before {
                        (curve.start(), original.start())
                    } else {
                        (curve.end(), original.end())
                    };
                    let equality = outer.coincides_with(&expected, &policy);
                    assert_eq!(equality.certainty, CurveCertainty::Certified);
                    assert_eq!(equality.value, Classification::Decided(true));
                }
            }
        }
    }
}

#[test]
fn resource_blocked_selected_range_enters_shared_fillet_kernel_directly() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let third = (Real::one() / Real::from(3_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        let end = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            third.clone(),
            64,
            &policy,
        );
        for parameter in [&start, &end] {
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }

        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(10, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let point = |parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter,
                &policy,
            ))
        };
        let fragment = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_selected_fiber(start.clone()),
                    CurveParameter2::from_selected_fiber(end.clone()),
                ),
                point(start),
                point(end),
            ),
        );
        let mut admitted = CornerCarrierPreparation2::admit(&fragment);
        admitted
            .prepare(CurveOperation2::Fillet, &policy)
            .expect("selected carrier admission must not require global projection");
        assert!(admitted.promoted_parallel().is_none());
        assert!(matches!(
            admitted
                .exact_carrier(true, CurveOperation2::Fillet, &policy)
                .unwrap(),
            crate::curve::ExactCornerCarrier2::SelectedFiber(_)
        ));
        let vertical = LineSeg2::try_new(p(7, 0), p(7, 10)).unwrap();
        let solve = |mode| {
            crate::curve::solve_exact_fillet_corner(
                admitted
                    .exact_carrier(true, CurveOperation2::Fillet, &policy)
                    .unwrap(),
                crate::curve::ExactCornerCarrier2::Line(&vertical),
                &Real::one(),
                RealSign::Positive,
                mode,
                false,
                CurveFamily2::RationalBezier,
                CurveFamily2::Line,
                None,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "resource-blocked selected carrier must fillet in its local chart: policy={policy:?}, mode={mode:?}, error={error:?}"
                )
            })
        };
        let trim = solve(CurveCornerMode2::TrimOnly);
        let extended = solve(CurveCornerMode2::TrimOrExtend);
        assert!({ trim.solutions().len() } > 0);
        assert!({ extended.solutions().len() } > { trim.solutions().len() });
    }
}

#[test]
fn resource_blocked_selected_offset_span_completes_its_cold_projection() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let end = q(3, 4);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        assert!(matches!(
            start.promoted_bezier_parameter(&policy).unwrap(),
            Classification::Uncertain(_)
        ));
        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(10, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let start_point =
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                start.clone(),
                &policy,
            ));
        let fragment = crate::bezier_split::BezierSelectedFiberFragment2::new(
            BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
            CurveParameterRange2::new_validated(
                CurveParameter2::from_selected_fiber(start.clone()),
                CurveParameter2::from(BezierParameter2::Exact(end.clone())),
            ),
            start_point,
            CurvePoint2::from(Point2::new(Real::from(10_i8) * &end, Real::zero())),
        );
        let Classification::Decided(span) = exact_offset_spans_from_retained_parallel_fragment(
            RetainedParallelOffsetFragmentRef2::Selected(&fragment),
            &q(1, 4),
            &policy,
        )
        .unwrap() else {
            panic!("a genuine offset carrier switch must complete selected projection");
        };
        let [span]: [ExactOffsetSpan2; 1] = span
            .try_into()
            .unwrap_or_else(|_| panic!("this regular source range must produce one offset span"));
        assert!(!span.fragments.is_empty());
        let Some(CurveTangent2::AlgebraicChord(retained)) = span.start_tangent else {
            panic!("the offset span must retain its compact local tangent chord");
        };
        assert_eq!(
            retained.certified_axis_direction(),
            Some(BezierAlgebraicChordAxisDirection2::PositiveX),
        );
    }
}

#[test]
fn resource_blocked_selected_corner_chamfers_in_its_affine_fiber() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let third = (Real::one() / Real::from(3_i8)).unwrap();
    let setback = q(1, 4);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        let end = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            third.clone(),
            64,
            &policy,
        );
        for parameter in [&start, &end] {
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }

        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(10, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let point = |parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter,
                &policy,
            ))
        };
        let start_point = point(start.clone());
        let end_point = point(end.clone());
        let selected = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_selected_fiber(start.clone()),
                    CurveParameter2::from_selected_fiber(end.clone()),
                ),
                start_point.clone(),
                end_point.clone(),
            ),
        );
        let apex = match crate::BezierAlgebraicChord2::translated_endpoint(
            &end_point,
            &Real::zero(),
            &Real::from(5_i8),
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("selected loop apex must translate: {reason:?}")
            }
        };
        let vertical = BezierSplitFragment2::AlgebraicChord(
            crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                end_point,
                apex.clone(),
                crate::bezier_offset::BezierAlgebraicChordAxisDirection2::PositiveY,
                &policy,
            ),
        );
        let closing =
            match crate::BezierAlgebraicChord2::try_new(apex, start_point, &policy).unwrap() {
                Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
                Classification::Uncertain(reason) => {
                    panic!("selected loop closing chord must construct: {reason:?}")
                }
            };
        let boundary = CurveRegionBoundaryLoop2::new(vec![selected, vertical, closing], &policy)
            .expect("the selected affine triangle must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        let chamfer = |mode| {
            region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    1,
                    setback.clone(),
                    setback.clone(),
                    mode,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "resource-blocked selected endpoint must chamfer in its local affine fiber: policy={policy:?}, mode={mode:?}, error={error:?}"
                    )
                })
        };
        let chamfers = chamfer(CurveCornerMode2::TrimOnly);
        let extended = chamfer(CurveCornerMode2::TrimOrExtend);
        assert_eq!(chamfers.certainty, CurveCertainty::Certified);
        assert_eq!(extended.certainty, CurveCertainty::Certified);
        assert!(chamfers.value.candidate_count() > 0);
        assert!(extended.value.candidate_count() > chamfers.value.candidate_count());
        let mut retained_local_boundary = false;
        for_each_corner_region(corner_regions(&chamfers.value), |edited| {
            retained_local_boundary |= edited.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(fragment, BezierSplitFragment2::SelectedFiber(_)));
        });
        assert!(retained_local_boundary);

        let projective = region
            .chamfer_loop_vertex_by_setbacks_with_policy(
                0,
                1,
                Real::from(6_i8),
                Real::from(6_i8),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "resource-blocked selected cuts must extend their authored range exactly: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(projective.certainty, CurveCertainty::Certified);
        let authored_source = BezierSelectedFiberSource2::AnalyticParallel(parallel);
        let mut retained_exterior_source = false;
        for_each_corner_region(corner_regions(&projective.value), |edited| {
            retained_exterior_source |=
                edited.boundary_loops()
                    .iter()
                    .flat_map(CurveRegionBoundaryLoop2::fragments)
                    .any(|fragment| {
                        matches!(
                            fragment,
                            BezierSplitFragment2::SelectedFiber(fragment)
                                if fragment.source() == &authored_source
                                    && (fragment.range().start().is_retained_scalar()
                                        || fragment.range().end().is_retained_scalar())
                                    && (matches!(
                                        fragment.range().start().cmp_by_refinement(&CurveParameter2::from_selected_fiber(start.clone()), &policy).unwrap(),
                                        Classification::Decided(std::cmp::Ordering::Less)
                                    ) || matches!(
                                        fragment.range().end().cmp_by_refinement(&CurveParameter2::from_selected_fiber(end.clone()), &policy).unwrap(),
                                        Classification::Decided(std::cmp::Ordering::Greater)
                                    ))
                        )
                    });
        });
        assert!(
            retained_exterior_source,
            "an exterior selected cut must retain its original support and exact source range"
        );
    }
}

#[test]
fn nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image() {
    let half = q(1, 2);
    let start = Real::zero();
    let setback = q(1, 10);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let center = crate::bezier_offset::high_degree_quadratic_selected_fiber_parameter_for_test(
            half.clone(),
            &policy,
        );
        assert!(matches!(
            center.promoted_bezier_parameter(&policy).unwrap(),
            Classification::Uncertain(_)
        ));
        let parallel =
            QuadraticBezier2::new(p(0, 0), Point2::new(Real::zero(), half.clone()), p(2, 1))
                .parallel_left(Real::zero())
                .unwrap();
        let start_point = CurvePoint2::from(p(0, 0));
        let end_point = CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
            parallel.clone(),
            center.clone(),
            &policy,
        ));
        let selected = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from(BezierParameter2::Exact(start.clone())),
                    CurveParameter2::from_selected_fiber(center.clone()),
                ),
                start_point.clone(),
                end_point.clone(),
            ),
        );
        let apex = match crate::BezierAlgebraicChord2::translated_endpoint(
            &end_point,
            &Real::zero(),
            &Real::from(5_i8),
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("the nonlinear selected apex must translate: {reason:?}")
            }
        };
        let vertical = BezierSplitFragment2::AlgebraicChord(
            crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                end_point,
                apex.clone(),
                crate::bezier_offset::BezierAlgebraicChordAxisDirection2::PositiveY,
                &policy,
            ),
        );
        let closing =
            match crate::BezierAlgebraicChord2::try_new(apex, start_point, &policy).unwrap() {
                Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
                Classification::Uncertain(reason) => {
                    panic!("the nonlinear selected closing chord must construct: {reason:?}")
                }
            };
        let boundary = CurveRegionBoundaryLoop2::new(vec![selected, vertical, closing], &policy)
            .expect("the nonlinear selected triangle must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let work = || {
            region.chamfer_loop_vertex_by_setbacks_with_policy(
                0,
                1,
                setback.clone(),
                setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let chamfers = hyperreal::dispatch_trace::with_recording(work);
        #[cfg(not(feature = "dispatch-trace"))]
        let chamfers = work();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        let chamfers = chamfers.unwrap_or_else(|error| {
            panic!(
                "the nonlinear selected endpoint must chamfer in its retained fiber: policy={policy:?}, error={error:?}"
            )
        });
        assert_eq!(chamfers.certainty, CurveCertainty::Certified);
        assert!(chamfers.value.candidate_count() > 0);
        let mut retained_local_cut = false;
        for_each_corner_region(corner_regions(&chamfers.value), |edited| {
            assert!(edited.has_regularized_filled_left_topology(&policy));
            // The closing chord crosses the source again, so exact
            // regularization can place the chamfer on a later loop.
            retained_local_cut |= edited
                .boundary_loops()
                .iter()
                .flat_map(|boundary| boundary.fragments())
                .filter_map(|fragment| match fragment {
                    BezierSplitFragment2::SelectedFiber(fragment) => Some(fragment.range()),
                    _ => None,
                })
                .flat_map(|range| [range.start(), range.end()])
                .filter_map(CurveParameter2::as_selected_fiber)
                .any(|parameter| {
                    parameter.order_to_real(&Real::zero(), &policy).unwrap()
                        == Classification::Decided(std::cmp::Ordering::Greater)
                        && parameter.cmp_by_refinement(&center, &policy).unwrap()
                            == Classification::Decided(std::cmp::Ordering::Less)
                });
            for (point, expected) in [
                // Both lobes survive the contact split. The small lobe
                // is between y=sqrt(x/2) and the original closing chord.
                (
                    Point2::new(q(1, 1000), q(3, 200)),
                    RegionPointLocation::Inside,
                ),
                (
                    Point2::new(q(1, 5), Real::one()),
                    RegionPointLocation::Inside,
                ),
                // This point lies in the corner removed by the chamfer.
                (
                    Point2::new(q(12, 25), half.clone()),
                    RegionPointLocation::Outside,
                ),
                (p(1, 1), RegionPointLocation::Outside),
            ] {
                let location = edited
                    .classify_point_with_policy(&point.clone().into(), &policy)
                    .unwrap();
                assert_eq!(location.certainty, CurveCertainty::Certified);
                assert_eq!(location.value, Classification::Decided(expected));
            }
        });
        assert!(retained_local_cut);
        #[cfg(feature = "dispatch-trace")]
        {
            assert!(
                trace.path_count(
                    "hypercurve",
                    "selected-fiber-fixed-distance",
                    "retained-local-image",
                ) > 0,
                "the nonlinear chamfer must enter the retained image kernel: {trace:?}",
            );
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "selected-fiber-fixed-distance",
                    "global-center-degenerate-fallback",
                ),
                0,
                "the supported nonlinear center must not promote globally: {trace:?}",
            );
        }
    }
}

#[test]
fn one_fragment_selected_native_extensions_keep_the_local_fiber() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let third = (Real::one() / Real::from(3_i8)).unwrap();
    let setback = q(1, 4);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        let end = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            third.clone(),
            64,
            &policy,
        );
        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(10, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let translated = |center| {
            let Classification::Decided(Some(parameters)) = parallel
                .affine_fixed_distance_parameters_from_selected_parameter(center, &setback, &policy)
                .unwrap()
            else {
                panic!("the affine selected parameter must translate exactly")
            };
            parameters
        };
        let [start_extension, _] = translated(&start);
        let [_, end_extension] = translated(&end);
        for parameter in [&start_extension, &end_extension] {
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }
        let point = |parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter,
                &policy,
            ))
        };

        for reversed in [false, true] {
            let mut fragment = BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                    CurveParameterRange2::new_validated(
                        CurveParameter2::from_selected_fiber(start.clone()),
                        CurveParameter2::from_selected_fiber(end.clone()),
                    ),
                    point(start.clone()),
                    point(end.clone()),
                ),
            );
            if reversed {
                fragment = fragment.reversed().unwrap();
            }
            let cut = |parameter: &crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2| {
                CornerTrimCut2 {
                    parameter: CurveParameter2::from_selected_fiber(parameter.clone()),
                    point: point(parameter.clone()),
                    placement: CornerPlacement2::Extension,
                    replacement: None,
                }
            };
            let (mut previous_cut, mut next_cut) = if reversed {
                (cut(&start_extension), cut(&end_extension))
            } else {
                (cut(&end_extension), cut(&start_extension))
            };
            CurveCornerChain2::retain_single_fragment_extension_cuts(
                &fragment,
                &mut previous_cut,
                &mut next_cut,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("native selected cuts must keep their source chart");
            assert!(previous_cut.parameter.is_retained_scalar());
            assert!(next_cut.parameter.is_retained_scalar());
            let retained = retained_corner_fragment_between_cuts(
                &fragment,
                &previous_cut,
                &next_cut,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("one selected extension interval must rebuild in its native chart");
            let BezierSplitFragment2::SelectedFiber(retained) = retained else {
                panic!("one selected extension interval must remain selected")
            };
            assert_eq!(retained.is_reversed(), reversed);
            assert_eq!(
                retained
                    .range()
                    .start()
                    .cmp_by_refinement(
                        &CurveParameter2::from_selected_fiber(start_extension.clone(),),
                        &policy,
                    )
                    .unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal),
            );
            assert_eq!(
                retained
                    .range()
                    .end()
                    .cmp_by_refinement(
                        &CurveParameter2::from_selected_fiber(end_extension.clone()),
                        &policy,
                    )
                    .unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal),
            );
        }
    }
}

#[test]
fn one_fragment_selected_projective_extensions_keep_the_local_fiber() {
    let half = q(1, 2);
    let third = q(1, 3);
    let setback = Real::from(6_i8);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        let end = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            third.clone(),
            64,
            &policy,
        );
        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(10, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let translated = |center| {
            let Classification::Decided(Some(parameters)) = parallel
                .affine_fixed_distance_parameters_from_selected_parameter(center, &setback, &policy)
                .unwrap()
            else {
                panic!("the affine selected parameter must translate exactly")
            };
            parameters
        };
        let [start_extension, _] = translated(&start);
        let [_, end_extension] = translated(&end);
        assert_eq!(
            start_extension
                .order_to_real(&Real::zero(), &policy)
                .unwrap(),
            Classification::Decided(std::cmp::Ordering::Less),
        );
        assert_eq!(
            end_extension.order_to_real(&Real::one(), &policy).unwrap(),
            Classification::Decided(std::cmp::Ordering::Greater),
        );
        for parameter in [&start_extension, &end_extension] {
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }
        let point = |parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter,
                &policy,
            ))
        };

        for reversed in [false, true] {
            let mut fragment = BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                    CurveParameterRange2::new_validated(
                        CurveParameter2::from_selected_fiber(start.clone()),
                        CurveParameter2::from_selected_fiber(end.clone()),
                    ),
                    point(start.clone()),
                    point(end.clone()),
                ),
            );
            if reversed {
                fragment = fragment.reversed().unwrap();
            }
            let cut = |parameter: &crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2| {
                CornerTrimCut2 {
                    parameter: CurveParameter2::from_selected_fiber(parameter.clone()),
                    point: point(parameter.clone()),
                    placement: CornerPlacement2::Extension,
                    replacement: None,
                }
            };
            let (mut previous_cut, mut next_cut) = if reversed {
                (cut(&start_extension), cut(&end_extension))
            } else {
                (cut(&end_extension), cut(&start_extension))
            };
            CurveCornerChain2::retain_single_fragment_extension_cuts(
                &fragment,
                &mut previous_cut,
                &mut next_cut,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("selected cuts beyond the native chart need no global projection");
            let replacement = previous_cut
                .replacement
                .as_deref()
                .expect("an exterior interval must retain one finite source range");
            assert!(next_cut.replacement.as_deref() == Some(replacement));
            let BezierSplitFragment2::SelectedFiber(replacement_fragment) = replacement else {
                panic!("selected source boundaries must retain their local fibers")
            };
            assert_eq!(replacement_fragment.is_reversed(), reversed);
            assert!(
                replacement_fragment
                    .start_point()
                    .shares_storage(&next_cut.point)
            );
            assert!(
                replacement_fragment
                    .end_point()
                    .shares_storage(&previous_cut.point)
            );
            for parameter in [
                replacement_fragment.range().start(),
                replacement_fragment.range().end(),
            ] {
                let selected = parameter
                    .as_selected_fiber()
                    .expect("the resource-blocked cut must remain selected");
                assert!(matches!(
                    selected.promoted_bezier_parameter(&policy).unwrap(),
                    Classification::Uncertain(_)
                ));
            }
            assert!(replacement_fragment.parallel_carrier() == parallel);
            for (retained, original) in [
                (replacement_fragment.range().start(), &start_extension),
                (replacement_fragment.range().end(), &end_extension),
            ] {
                assert!(retained == &CurveParameter2::from_selected_fiber(original.clone()));
            }
            let retained = retained_corner_fragment_between_cuts(
                &fragment,
                &previous_cut,
                &next_cut,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("the selected finite envelope must reconstruct authoritatively");
            let BezierSplitFragment2::SelectedFiber(retained) = retained else {
                panic!("the projective replacement must remain a selected fragment")
            };
            assert!(retained == *replacement_fragment);

            for previous in [false, true] {
                let exterior = match (previous, reversed) {
                    (true, false) | (false, true) => &end_extension,
                    (true, true) | (false, false) => &start_extension,
                };
                let mut one_cut = cut(exterior);
                CurveCornerChain2::retain_corner_cut(
                    &fragment,
                    &mut one_cut,
                    previous,
                    CurveOperation2::Chamfer,
                    &policy,
                )
                .expect("one exterior selected cut must retain a finite envelope");
                assert!(matches!(
                    one_cut.replacement.as_deref(),
                    Some(BezierSplitFragment2::SelectedFiber(_))
                ));
                let rebuilt = retained_corner_fragment_extension(
                    &fragment,
                    one_cut.parameter.clone(),
                    &one_cut.point,
                    one_cut.replacement.as_deref(),
                    previous,
                    CurveOperation2::Chamfer,
                    &policy,
                )
                .expect("one exterior selected extension must reconstruct");
                let [BezierSplitFragment2::SelectedFiber(rebuilt)] = rebuilt.as_slice() else {
                    panic!("one projective extension must remain a selected fragment")
                };
                assert_eq!(rebuilt.is_reversed(), reversed);
                let BezierSplitFragment2::SelectedFiber(original) = &fragment else {
                    unreachable!()
                };
                if previous {
                    assert!(rebuilt.start_point().shares_storage(original.start_point()));
                    assert!(rebuilt.end_point().shares_storage(&one_cut.point));
                } else {
                    assert!(rebuilt.start_point().shares_storage(&one_cut.point));
                    assert!(rebuilt.end_point().shares_storage(original.end_point()));
                }
            }
        }
    }
}

#[test]
fn selected_parallel_companion_fillets_without_range_promotion() {
    let half = q(1, 2);
    let third = q(1, 3);
    let radius = q(1, 4);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let horizontal_start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        let curved_end = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            third.clone(),
            64,
            &policy,
        );
        for parameter in [&horizontal_start, &curved_end] {
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }

        let corner = p(4, 0);
        let horizontal =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let curved = QuadraticBezier2::new(corner.clone(), p(4, 4), p(0, 4))
            .parallel_left(Real::zero())
            .unwrap();
        let selected_point = |parallel: &BezierParallel2, parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter,
                &policy,
            ))
        };
        let horizontal_start_point = selected_point(&horizontal, horizontal_start.clone());
        let curved_end_point = selected_point(&curved, curved_end.clone());
        let previous = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(horizontal),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_selected_fiber(horizontal_start),
                    CurveParameter2::from(BezierParameter2::Exact(Real::one())),
                ),
                horizontal_start_point.clone(),
                CurvePoint2::from(corner.clone()),
            ),
        );
        let next = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(curved),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                    CurveParameter2::from_selected_fiber(curved_end),
                ),
                CurvePoint2::from(corner),
                curved_end_point.clone(),
            ),
        );
        let closing = match crate::BezierAlgebraicChord2::try_new(
            curved_end_point,
            horizontal_start_point,
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
            Classification::Uncertain(reason) => {
                panic!("the selected closing chord must construct: {reason:?}")
            }
        };
        let boundary = CurveRegionBoundaryLoop2::new(vec![previous, next, closing], &policy)
            .expect("the two-selected-carrier triangle must close");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        let fillets = region
            .fillet_loop_vertex_with_policy(
                0,
                1,
                &crate::CurveFillet2::new(radius.clone()),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "a selected parallel companion must fillet without promotion: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(fillets.certainty, CurveCertainty::Certified);
        assert!(!fillets.value.solutions().is_empty());
        for_each_corner_region(fillet_regions(&fillets.value), |edited| {
            assert!(
                edited.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    ))
            );
            assert_eq!(
                edited.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::SelectedFiber(_))
                    })
                    .count(),
                2,
            );
        });
    }
}

#[test]
fn resource_blocked_selected_boundary_fillets_and_reconstructs_in_place() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let radius = (Real::one() / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
            half.clone(),
            32_768,
            &policy,
        );
        assert!(matches!(
            start.promoted_bezier_parameter(&policy).unwrap(),
            Classification::Uncertain(_)
        ));
        let end = q(3, 4);
        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(10, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let point = |parameter| {
            CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                parallel.clone(),
                parameter,
                &policy,
            ))
        };
        let start_point = point(start.clone());
        let end_point = CurvePoint2::from(Point2::new(q(15, 2), Real::zero()));
        let selected = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel),
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_selected_fiber(start),
                    CurveParameter2::from(BezierParameter2::Exact(end)),
                ),
                start_point.clone(),
                end_point.clone(),
            ),
        );
        let apex = match crate::BezierAlgebraicChord2::translated_endpoint(
            &end_point,
            &Real::zero(),
            &Real::from(5_i8),
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("selected loop apex must translate: {reason:?}")
            }
        };
        let chord = |start, end| match crate::BezierAlgebraicChord2::try_new(start, end, &policy)
            .unwrap()
        {
            Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
            Classification::Uncertain(reason) => {
                panic!("selected loop chord must construct: {reason:?}")
            }
        };
        let vertical = BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(
                    end_point
                        .coordinates()
                        .expect("the selected corner endpoint is represented")
                        .clone(),
                    apex.coordinates()
                        .expect("the translated represented apex stays represented")
                        .clone(),
                )
                .unwrap(),
            )),
        };
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![selected, vertical, chord(apex, start_point)],
            &policy,
        )
        .expect("the selected triangle must close with shared endpoint evidence");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        let chamfers = region
            .chamfer_loop_vertex_by_setbacks_with_policy(
                0,
                1,
                radius.clone(),
                radius.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "resource-blocked selected boundary must chamfer and rebuild: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(chamfers.certainty, CurveCertainty::Certified);
        assert!(chamfers.value.candidate_count() > 0);
        let fillets = region
            .fillet_loop_vertex_with_policy(
                0,
                1,
                &crate::CurveFillet2::new(radius.clone()),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "resource-blocked selected boundary must fillet and rebuild: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(fillets.certainty, CurveCertainty::Certified);
        assert!(!fillets.value.solutions().is_empty());
        let extended_fillets = region
            .fillet_loop_vertex_with_policy(
                0,
                1,
                &crate::CurveFillet2::new(radius.clone()),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "resource-blocked selected boundary must extend and rebuild: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(extended_fillets.certainty, CurveCertainty::Certified);
        assert!(
            extended_fillets.value.solutions().len() > fillets.value.solutions().len(),
            "the selected incident chart must contribute exterior fillet candidates"
        );
        let mut retained_local_boundary = false;
        for_each_corner_region(fillet_regions(&fillets.value), |edited| {
            retained_local_boundary |= edited.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(fragment, BezierSplitFragment2::SelectedFiber(_)));
        });
        assert!(retained_local_boundary);
    }
}

fn one_fragment_materialized_corner_region(reversed: bool, policy: &CurveContext) -> CurveRegion2 {
    let seam = p(0, 0);
    let mut fragment = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(seam.clone(), p(4, 0), p(0, 4), seam)),
    };
    if reversed {
        fragment = fragment
            .reversed()
            .expect("the materialized closed cubic reverses exactly");
    }
    let boundary = CurveRegionBoundaryLoop2::new(vec![fragment], policy)
        .expect("the materialized one-fragment loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .expect("the materialized one-fragment loop has authored topology")
}

fn retained_straight_extension_region(reversed: bool, policy: &CurveContext) -> CurveRegion2 {
    let parameter = positive_inverse_sqrt_parameter(2, policy);
    let source = RationalBezier2::try_new(vec![p(0, 0), p(1, 0)], vec![Real::one(), Real::one()])
        .expect("the algebraic endpoint source is finite");
    let lower_left = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(&source, &parameter, policy)
            .expect("the algebraic endpoint has exact evidence"),
    );
    let upper_left = match crate::BezierAlgebraicChord2::translated_endpoint(
        &lower_left,
        &Real::zero(),
        &Real::from(2_i8),
        policy,
    )
    .expect("the retained endpoint translates exactly")
    {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("the retained endpoint translation must be decided: {reason:?}")
        }
    };
    let exact = |point: Point2| CurvePoint2::from(point);
    let chord = |start, end| match crate::BezierAlgebraicChord2::try_new(start, end, policy)
        .expect("the retained straight support is valid")
    {
        Classification::Decided(chord) => BezierSplitFragment2::AlgebraicChord(chord),
        Classification::Uncertain(reason) => {
            panic!("the retained straight support must be decided: {reason:?}")
        }
    };
    let mut fragments = vec![
        chord(lower_left.clone(), exact(p(2, 0))),
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(p(2, 0), p(2, 2)).unwrap(),
            )),
        },
        chord(exact(p(2, 2)), upper_left.clone()),
        chord(upper_left, lower_left),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| {
                fragment
                    .reversed()
                    .expect("the retained rectangle reverses")
            })
            .collect();
    }
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the retained straight loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .expect("the retained straight loop has authored topology")
}

fn retained_nonlinear_extension_region(reversed: bool, policy: &CurveContext) -> CurveRegion2 {
    let materialized = |curve| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve,
    };
    // B(t) = (t - 1, (t - 1)^2). Its endpoint is the edited corner and
    // the increasing exterior ray has both rational and irrational fixed-
    // distance contacts used below.
    let mut fragments = vec![
        materialized(BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            p(-1, 1),
            Point2::new(
                (Real::from(-1_i8) / Real::from(2_i8)).unwrap(),
                Real::zero(),
            ),
            p(0, 0),
        ))),
        materialized(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(0, 3)).unwrap()),
        )),
        materialized(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 3), p(-3, 3)).unwrap()),
        )),
        materialized(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(-3, 3), p(-1, 1)).unwrap()),
        )),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the nonlinear loop reverses"))
            .collect();
    }
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the nonlinear extension loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .expect("the nonlinear extension loop has authored topology")
}

fn retained_rational_extension_region(reversed: bool, policy: &CurveContext) -> CurveRegion2 {
    let materialized = |curve| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve,
    };
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    // R(t) = (t / (2 - t), (t / (2 - t))^2). The edited endpoint is
    // R(1) = (1, 1), and its increasing incident cell ends at the pole
    // t = 2. Setback sqrt(68) has the represented exterior contact
    // R(3/2) = (3, 9); setback one has an algebraic exterior contact whose
    // first coarse isolator reaches the pole and therefore exercises exact
    // finite-envelope refinement.
    let rational = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(half.clone(), Real::zero()), p(1, 1)],
        vec![Real::one(), half, quarter],
    )
    .unwrap();
    let mut fragments = vec![
        materialized(BezierSubcurve2::Rational(rational)),
        materialized(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(1, 1), p(1, 12)).unwrap()),
        )),
        materialized(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(1, 12), p(-3, 12)).unwrap()),
        )),
        materialized(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(-3, 12), p(0, 0)).unwrap()),
        )),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the rational loop reverses"))
            .collect();
    }
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the rational extension loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .expect("the rational extension loop has authored topology")
}

fn retained_analytic_parabola_extension_region(
    selected: bool,
    reversed: bool,
    line_end: Point2,
    policy: &CurveContext,
) -> CurveRegion2 {
    let start = p(0, 0);
    let corner = p(1, 1);
    let source = QuadraticBezier2::new(
        start.clone(),
        Point2::new(q(1, 2), Real::zero()),
        corner.clone(),
    );
    let first = if selected {
        let rational = RationalBezier2::try_new(
            source.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        )
        .expect("the selected parabola is finite");
        BezierSplitFragment2::SelectedFiber(crate::bezier_split::BezierSelectedFiberFragment2::new(
            BezierSelectedFiberSource2::Rational(rational),
            CurveParameterRange2::new_validated(
                CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                CurveParameter2::from(BezierParameter2::Exact(Real::one())),
            ),
            CurvePoint2::from(start.clone()),
            CurvePoint2::from(corner.clone()),
        ))
    } else {
        let Classification::Decided(fragment) = crate::BezierParallelFragment2::try_new(
            source.parallel_left(Real::zero()).unwrap(),
            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            policy,
        )
        .expect("the analytic parabola range is valid") else {
            panic!("the complete analytic parabola range must be decided");
        };
        BezierSplitFragment2::AnalyticParallel(fragment)
    };
    let line = |start, end| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(start, end).unwrap(),
        )),
    };
    let mut fragments = vec![
        first,
        line(corner, line_end.clone()),
        line(line_end, p(-2, 3)),
        line(p(-2, 3), p(-2, -2)),
        line(p(-2, -2), p(0, -2)),
        line(p(0, -2), start),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the analytic loop reverses"))
            .collect();
    }
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the retained analytic parabola loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .expect("the retained analytic parabola loop has authored topology")
}

fn retained_fragment_has_exact_endpoint(
    fragment: &BezierSplitFragment2,
    expected: &Point2,
) -> bool {
    let expected = CurvePoint2::from(expected.clone());
    [true, false].into_iter().any(|start| {
        let Ok(Classification::Decided(Some(point))) =
            curve_fragment_endpoint_point(fragment, start, &CurveContext::STRICT)
        else {
            return false;
        };
        let equality = point.coincides_with(&expected, &CurveContext::STRICT);
        equality.certainty == CurveCertainty::Certified
            && equality.value == Classification::Decided(true)
    })
}

fn retained_rational_fragment_has_algebraic_endpoint(fragment: &BezierSplitFragment2) -> bool {
    let BezierSplitFragment2::SelectedFiber(fragment) = fragment else {
        return false;
    };
    fragment.rational_curve().is_some()
        && [fragment.range().start(), fragment.range().end()]
            .into_iter()
            .any(|parameter| {
                matches!(
                    parameter.as_bezier_parameter(),
                    Some(BezierParameter2::Algebraic(_))
                )
            })
}

fn corner_regions(solutions: &CurveCornerSolutions2<CurveRegion2>) -> &[CurveRegion2] {
    match solutions {
        CurveCornerSolutions2::NoSolution(reason) => panic!("no exact corner edit: {reason:?}"),
        CurveCornerSolutions2::Unique(region) => std::slice::from_ref(region),
        CurveCornerSolutions2::Multiple(regions) => regions,
    }
}

fn fillet_regions(solutions: &CurveCornerSolutions2<CurveRegion2>) -> &[CurveRegion2] {
    assert!(
        !solutions.solutions().is_empty(),
        "no isolated fillets: {:?}",
        solutions.no_solution_reason()
    );
    solutions.solutions()
}

fn for_each_corner_region(solutions: &[CurveRegion2], visit: impl FnMut(&CurveRegion2)) {
    assert!(!solutions.is_empty());
    solutions.iter().for_each(visit);
}

fn assert_one_fragment_edit_shape(
    solutions: &[CurveRegion2],
    selected_fragment_count: usize,
    minimum_inserted_fragment_count: usize,
) {
    for_each_corner_region(solutions, |region| {
        assert_eq!(region.boundary_loops().len(), 1);
        let fragments = region.boundary_loops()[0].fragments();
        assert_eq!(
            fragments
                .iter()
                .filter(|fragment| matches!(fragment, BezierSplitFragment2::SelectedFiber(_)))
                .count(),
            selected_fragment_count,
        );
        assert!(
            fragments.len() >= selected_fragment_count + minimum_inserted_fragment_count,
            "the edit must retain its seam-side trims and inserted carrier: {fragments:?}"
        );
    });
}

#[test]
fn one_fragment_selected_loop_chamfers_from_one_interval() {
    let setback = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = one_fragment_selected_corner_region(reversed, &policy);
            let result = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the one-fragment selected loop must chamfer: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(
                result.value.candidate_count() > 0,
                "the authored seam has an admissible chamfer: policy={policy:?}, reversed={reversed}, result={:?}",
                result.value
            );
            assert_one_fragment_edit_shape(corner_regions(&result.value), 1, 1);
        }
    }
}

#[test]
fn retained_algebraic_straights_trim_or_extend_chamfer_and_fillet() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = retained_straight_extension_region(reversed, &policy);
            let corner = if reversed { 3 } else { 1 };
            let trim_chamfers = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    corner,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the retained straight corner trims exactly");
            assert_eq!(trim_chamfers.certainty, CurveCertainty::Certified);
            let trim_chamfers = trim_chamfers.into_value();
            let extended_chamfers = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    corner,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the retained straight corner extends exactly");
            assert_eq!(extended_chamfers.certainty, CurveCertainty::Certified);
            let extended_chamfers = extended_chamfers.into_value();
            assert!(
                extended_chamfers.candidate_count() > trim_chamfers.candidate_count(),
                "extension must publish the exterior-ray chamfer branches"
            );

            let trim_fillets = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the retained straight corner fillets exactly");
            assert_eq!(trim_fillets.certainty, CurveCertainty::Certified);
            let trim_fillets = trim_fillets.into_value();
            let extended_fillets = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the retained straight fillet extends exactly");
            assert_eq!(extended_fillets.certainty, CurveCertainty::Certified);
            let extended_fillets = extended_fillets.into_value();
            assert!(
                { extended_fillets.solutions().len() } > { trim_fillets.solutions().len() },
                "extension must publish the exterior-ray fillet branch"
            );

            for solutions in [
                corner_regions(&extended_chamfers),
                fillet_regions(&extended_fillets),
            ] {
                let mut found_both_extensions = false;
                for_each_corner_region(solutions, |edited| {
                    assert!(edited.has_regularized_filled_left_topology(&policy));
                    let has_endpoint = |point: &Point2| {
                        edited.boundary_loops().iter().any(|boundary| {
                            boundary.fragments().iter().any(|fragment| {
                                retained_fragment_has_exact_endpoint(fragment, point)
                            })
                        })
                    };
                    let both_extensions = has_endpoint(&p(3, 0)) && has_endpoint(&p(2, -1));
                    found_both_extensions |= both_extensions;
                    for (point, expected) in [
                        (p(1, 1), RegionPointLocation::Inside),
                        (p(5, 5), RegionPointLocation::Outside),
                    ] {
                        let location = edited
                            .classify_point_with_policy(&point.clone().into(), &policy)
                            .unwrap();
                        assert_eq!(location.certainty, CurveCertainty::Certified);
                        assert_eq!(location.value, Classification::Decided(expected));
                    }
                    if both_extensions {
                        // Both exterior cuts enclose material below and to
                        // the right of the old corner, possibly on a second
                        // normalized loop touching the rectangle at (2,0).
                        let exterior = Point2::new(q(17, 8), -q(1, 8));
                        let location = edited
                            .classify_point_with_policy(&exterior.clone().into(), &policy)
                            .unwrap();
                        assert_eq!(location.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            location.value,
                            Classification::Decided(RegionPointLocation::Inside)
                        );
                    }
                });
                assert!(
                    found_both_extensions,
                    "one candidate must extend both incident straight carriers"
                );
            }
        }
    }
}

#[test]
fn retained_polynomial_chamfer_extends_exact_and_algebraic_incident_roots() {
    use RegionPointLocation::{Boundary, Inside, Outside};

    let sqrt_two = Real::from(2_i8).sqrt().unwrap();
    let added_material = Point2::new(q(1, 2), q(3, 10));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = retained_nonlinear_extension_region(reversed, &policy);
            assert_eq!(
                region
                    .classify_point_with_policy(&added_material.clone().into(), &policy)
                    .unwrap()
                    .value,
                Classification::Decided(Outside)
            );
            let corner = if reversed { 3 } else { 1 };
            let setbacks = |setback| {
                if reversed {
                    (Real::zero(), setback)
                } else {
                    (setback, Real::zero())
                }
            };
            for (setback, exact_endpoint) in
                [(sqrt_two.clone(), Some(p(1, 1))), (Real::one(), None)]
            {
                let (previous_setback, next_setback) = setbacks(setback.clone());
                let trim = region
                    .chamfer_loop_vertex_by_setbacks_with_policy(
                        0,
                        corner,
                        previous_setback.clone(),
                        next_setback.clone(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .expect("the nonlinear corner has an interior setback")
                    .into_value();
                let extended = region
                    .chamfer_loop_vertex_by_setbacks_with_policy(
                        0,
                        corner,
                        previous_setback,
                        next_setback,
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the nonlinear incident ray must extend: policy={policy:?}, reversed={reversed}, setback={setback:?}, error={error:?}"
                        )
                    });
                assert_eq!(extended.certainty, CurveCertainty::Certified);
                let extended = extended.into_value();
                assert!(extended.candidate_count() > trim.candidate_count());
                let mut found_extension = false;
                let mut found_added_material = false;
                for_each_corner_region(corner_regions(&extended), |edited| {
                    assert!(matches!(
                        edited
                            .classify_point_with_policy(&p(10, 10).into(), &policy)
                            .expect("the canonicalized extension remains classifiable")
                            .into_value(),
                        Classification::Decided(_)
                    ));
                    found_extension |= edited
                        .boundary_loops()
                        .iter()
                        .flat_map(CurveRegionBoundaryLoop2::fragments)
                        .any(|fragment| match &exact_endpoint {
                            Some(endpoint) => {
                                retained_fragment_has_exact_endpoint(fragment, endpoint)
                            }
                            None => retained_rational_fragment_has_algebraic_endpoint(fragment),
                        });
                    if exact_endpoint.is_none()
                        && edited
                            .classify_point_with_policy(&added_material.clone().into(), &policy)
                            .unwrap()
                            .value
                            == Classification::Decided(Inside)
                    {
                        found_added_material = true;
                        // For setback one, the added lobe lies between
                        // y=x^2 and y=a*x, a=sqrt((sqrt(5)-1)/2). It only
                        // touches the original material at the corner.
                        for (point, location) in [
                            (Point2::new(q(-1, 2), Real::one()), Inside),
                            (Point2::new(q(1, 2), q(1, 4)), Boundary),
                            (Point2::new(q(1, 2), q(1, 2)), Outside),
                            (p(0, 0), Boundary),
                        ] {
                            assert_eq!(
                                edited
                                    .classify_point_with_policy(&point.clone().into(), &policy)
                                    .unwrap()
                                    .value,
                                Classification::Decided(location),
                                "the extended corner must preserve the intended filled sectors"
                            );
                        }
                    }
                });
                assert!(
                    found_extension,
                    "an edited candidate must retain the exterior nonlinear carrier"
                );
                assert!(exact_endpoint.is_some() || found_added_material);
            }
        }
    }
}

#[test]
fn retained_rational_chamfer_extends_exact_and_algebraic_pre_pole_roots() {
    let sqrt_sixty_eight = Real::from(68_i8).sqrt().unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = retained_rational_extension_region(reversed, &policy);
            let corner = if reversed { 3 } else { 1 };
            let setbacks = |setback| {
                if reversed {
                    (Real::zero(), setback)
                } else {
                    (setback, Real::zero())
                }
            };
            for (setback, exact_endpoint) in [
                (sqrt_sixty_eight.clone(), Some(p(3, 9))),
                (Real::one(), None),
            ] {
                let (previous_setback, next_setback) = setbacks(setback.clone());
                let trim = region
                    .chamfer_loop_vertex_by_setbacks_with_policy(
                        0,
                        corner,
                        previous_setback.clone(),
                        next_setback.clone(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .expect("the rational corner has a decided trim result")
                    .into_value();
                let extended = region
                    .chamfer_loop_vertex_by_setbacks_with_policy(
                        0,
                        corner,
                        previous_setback,
                        next_setback,
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the rational incident cell must extend: policy={policy:?}, reversed={reversed}, setback={setback:?}, error={error:?}"
                        )
                    });
                assert_eq!(extended.certainty, CurveCertainty::Certified);
                let extended = extended.into_value();
                assert!(extended.candidate_count() > trim.candidate_count());
                let mut found_extension = false;
                for_each_corner_region(corner_regions(&extended), |edited| {
                    assert!(matches!(
                        edited
                            .classify_point_with_policy(&p(20, 20).into(), &policy)
                            .expect("the pole-free rational extension remains classifiable")
                            .into_value(),
                        Classification::Decided(_)
                    ));
                    found_extension |= edited
                        .boundary_loops()
                        .iter()
                        .flat_map(CurveRegionBoundaryLoop2::fragments)
                        .any(|fragment| match &exact_endpoint {
                            Some(endpoint) => {
                                retained_fragment_has_exact_endpoint(fragment, endpoint)
                            }
                            None => retained_rational_fragment_has_algebraic_endpoint(fragment),
                        });
                });
                assert!(
                    found_extension,
                    "an edited candidate must retain the pre-pole rational extension"
                );
            }
        }
    }
}

#[test]
fn retained_analytic_corners_preserve_normalized_sets() {
    use RegionPointLocation::{Boundary, Inside, Outside};

    let exact_line_end = Point2::new(Real::one() + q(38280, 91901), Real::one() + q(83549, 91901));
    let algebraic_line_end = Point2::new(q(23, 13), q(37, 13));
    let exact_cut = Point2::new(q(6, 5), q(36, 25));
    let chamfer_setback = (Real::from(146_i16).sqrt().unwrap() / Real::from(25_i8)).unwrap();
    // P(t)=(t,t^2) meets the outgoing line again at t=m-1. This
    // crossing precedes the construction cut t=6/5, so normalization
    // consumes that cut and the straight connector inside old material.
    let crossing_parameter = q(83549, 38280) - Real::one();
    let crossing = Point2::new(
        crossing_parameter.clone(),
        &crossing_parameter * &crossing_parameter,
    );
    let chamfer_samples = [
        (
            "exposed parabola",
            Point2::new(q(11, 10), q(121, 100)),
            Boundary,
        ),
        (
            "added material",
            Point2::new(q(11, 10), q(243, 200)),
            Inside,
        ),
        ("source/line crossing", crossing.clone(), Boundary),
        ("consumed construction cut", exact_cut.clone(), Inside),
    ];
    // For r=299/125 the exterior circle has center (-126/125,59/25).
    // Its CCW continuation is the major arc, adding material beyond the
    // original x=-2 wall. The source residual factors as
    // (t-6/5)^2 * (t^2+(12/5)t+3/5), whose other roots are negative.
    let exact_fillet_samples = [
        (
            "exposed parabola",
            Point2::new(q(11, 10), q(121, 100)),
            Boundary,
        ),
        (
            "added source lobe",
            Point2::new(q(11, 10), q(243, 200)),
            Inside,
        ),
        ("source/line crossing", crossing, Boundary),
        ("consumed tangent contact", exact_cut, Inside),
        (
            "circle extreme",
            Point2::new(q(-17, 5), q(59, 25)),
            Boundary,
        ),
        ("added circle interior", p(-3, 2), Inside),
    ];
    // For slope 12/5 and r=1/2 the exterior source contact lies in
    // (7/5,141/100). Its entire tangent disk is inside old material:
    // x>1/5, x<23/13, y>1 and y<37/13, on the material side of the
    // outgoing line. Normalization consumes the algebraic contact and
    // arc, leaving the visible rational crossing P(7/5).
    let algebraic_fillet_samples = [
        (
            "exposed parabola",
            Point2::new(q(13, 10), q(169, 100)),
            Boundary,
        ),
        (
            "added material",
            Point2::new(q(13, 10), q(341, 200)),
            Inside,
        ),
        (
            "source/line crossing",
            Point2::new(q(7, 5), q(49, 25)),
            Boundary,
        ),
    ];
    let common_samples = [
        ("original interior", p(-1, 0), Inside),
        ("far exterior", p(10, 10), Outside),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            for selected in [false, true] {
                let assert_extended_set =
                    |solutions: &[CurveRegion2],
                     operation: &str,
                     samples: &[(&str, Point2, RegionPointLocation)]| {
                        let mut found = false;
                        let mut observations = Vec::new();
                        for_each_corner_region(solutions, |edited| {
                            assert!(edited.has_regularized_filled_left_topology(&policy));
                            let observed: Vec<_> = samples
                                .iter()
                                .chain(&common_samples)
                                .map(|(label, point, _)| {
                                    let location = edited.classify_point_with_policy(&point.clone().into(), &policy).unwrap();
                                    assert_eq!(
                                        location.certainty,
                                        CurveCertainty::Certified,
                                        "{operation}: {label}, policy={policy:?}, reversed={reversed}, selected={selected}",
                                    );
                                    match location.value {
                                        Classification::Decided(location) => location,
                                        Classification::Uncertain(reason) => panic!(
                                            "{operation}: {label} remained {reason:?}, policy={policy:?}, reversed={reversed}, selected={selected}"
                                        ),
                                    }
                                })
                                .collect();
                            found |= observed
                                .iter()
                                .zip(samples.iter().chain(&common_samples))
                                .all(|(actual, (_, _, expected))| actual == expected);
                            observations.push(observed);
                        });
                        assert!(
                            found,
                            "{operation}: the exact extended set was lost, policy={policy:?}, reversed={reversed}, selected={selected}, locations={observations:?}",
                        );
                    };
                let corner = if reversed { 5 } else { 1 };
                let region = retained_analytic_parabola_extension_region(
                    selected,
                    reversed,
                    exact_line_end.clone(),
                    &policy,
                );
                let (previous_setback, next_setback) = if reversed {
                    (Real::zero(), chamfer_setback.clone())
                } else {
                    (chamfer_setback.clone(), Real::zero())
                };
                let chamfers = region
                    .chamfer_loop_vertex_by_setbacks_with_policy(
                        0,
                        corner,
                        previous_setback,
                        next_setback,
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the retained analytic chamfer must extend: policy={policy:?}, reversed={reversed}, selected={selected}, error={error:?}"
                        )
                    });
                assert_eq!(chamfers.certainty, CurveCertainty::Certified);
                assert_extended_set(corner_regions(&chamfers.value), "chamfer", &chamfer_samples);

                let fillets = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(q(299, 125)),
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the retained analytic fillet must extend: policy={policy:?}, reversed={reversed}, selected={selected}, error={error:?}"
                        )
                    });
                assert_eq!(fillets.certainty, CurveCertainty::Certified);
                assert_extended_set(
                    fillet_regions(&fillets.value),
                    "exact fillet",
                    &exact_fillet_samples,
                );

                let algebraic = retained_analytic_parabola_extension_region(
                    selected,
                    reversed,
                    algebraic_line_end.clone(),
                    &policy,
                )
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(q(1, 2)),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the algebraic retained fillet must extend: policy={policy:?}, reversed={reversed}, selected={selected}, error={error:?}"
                    )
                });
                assert_eq!(algebraic.certainty, CurveCertainty::Certified);
                assert_extended_set(
                    fillet_regions(&algebraic.value),
                    "algebraic fillet",
                    &algebraic_fillet_samples,
                );
            }
        }
    }
}

#[test]
fn one_fragment_materialized_loop_chamfers_to_one_middle_interval() {
    let setback = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = one_fragment_materialized_corner_region(reversed, &policy);
            let result = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the materialized one-fragment loop must chamfer: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            for_each_corner_region(corner_regions(&result.value), |edited| {
                assert_eq!(
                    edited.boundary_loops()[0].fragments().len(),
                    2,
                    "two nonzero cuts must retain one middle source interval and one chamfer"
                );
            });
        }
    }
}

#[test]
fn one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once() {
    let setback = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = one_fragment_materialized_corner_region(reversed, &policy);
            let trimmed = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the one-fragment cubic has interior chamfer cuts")
                .into_value();
            let extended = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the algebraic one-fragment chamfer must extend: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(
                extended.value.candidate_count() > trimmed.candidate_count(),
                "the incident rays must add projective chamfer candidates"
            );
            let mut retained_algebraic_interval = false;
            for_each_corner_region(corner_regions(&extended.value), |edited| {
                for fragment in edited.boundary_loops()[0].fragments() {
                    let BezierSplitFragment2::SelectedFiber(selected) = fragment else {
                        continue;
                    };
                    retained_algebraic_interval |=
                        [selected.range().start(), selected.range().end()]
                            .into_iter()
                            .all(corner_parameter_needs_retained_source);
                    let source = selected
                        .rational_curve()
                        .expect("the retained cubic source")
                        .parallel_left(Real::zero())
                        .unwrap();
                    // The range stays ordered in the source chart, while
                    // endpoint evidence follows the fragment's traversal.
                    let (start, end) = if selected.is_reversed() {
                        (selected.range().end(), selected.range().start())
                    } else {
                        (selected.range().start(), selected.range().end())
                    };
                    for (parameter, point) in
                        [(start, selected.start_point()), (end, selected.end_point())]
                    {
                        // Evaluate the support independently of its stored endpoints.
                        // The exact source parameter may lie outside [0, 1].
                        let replay = CurvePoint2::from(
                            crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
                                source.clone(), parameter, Real::zero(), &policy,
                            ).expect("a cubic source accepts each retained scalar")
                        );
                        let equality = replay.coincides_with(point, &policy);
                        assert_eq!(equality.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            equality.value,
                            Classification::Decided(true),
                            "endpoint replay must respect traversal: input_reversed={reversed}, fragment_reversed={}",
                            selected.is_reversed(),
                        );
                    }
                }
            });
            assert!(
                retained_algebraic_interval,
                "one source range must retain its two algebraic cuts"
            );
        }
    }
}

#[test]
fn one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier() {
    let setback = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = one_fragment_selected_corner_region(reversed, &policy);
            let trimmed = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the retained closed cubic has interior chamfer cuts")
                .into_value();
            let extended = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the retained one-fragment chamfer must extend: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(extended.value.candidate_count() > trimmed.candidate_count());
            let original_support =
                CurveSupport2::from_fragment(&region.boundary_loops()[0].fragments()[0]);
            let unit = CurveParameterRange2::unit();
            let mut found_extension = false;
            for_each_corner_region(corner_regions(&extended.value), |edited| {
                found_extension |= edited.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| {
                        let same_support =
                            match (CurveSupport2::from_fragment(fragment), &original_support) {
                                (
                                    CurveSupport2::Bezier(retained),
                                    CurveSupport2::Bezier(original),
                                ) => &retained == original,
                                (
                                    CurveSupport2::Parallel(retained),
                                    CurveSupport2::Parallel(original),
                                ) => &retained == original,
                                _ => false,
                            };
                        same_support
                            && matches!(
                                crate::bezier_split::CurveParameterDomain2::new(&unit, None)
                                    .contains_finite_range(
                                        &fragment.curve_region_parameter_range(),
                                        &policy
                                    )
                                    .unwrap(),
                                Classification::Decided(false),
                            )
                    });
            });
            assert!(
                found_extension,
                "the exterior interval must retain its original support"
            );
        }
    }
}

#[test]
fn one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope() {
    let setback = (Real::one() / Real::from(2_i8)).unwrap();
    let distance = (Real::one() / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let seam = p(0, 0);
            let source = RationalBezier2::try_new(
                vec![seam.clone(), p(3, 0), p(0, 3), p(-3, 0), seam.clone()],
                vec![Real::one(); 5],
            )
            .unwrap();
            let parallel = BezierParallel2::from_source(
                BezierParallelSource2::Rational(source),
                distance.clone(),
            );
            let range = BezierParameterRange2::from_exact(Real::zero(), Real::one());
            let Classification::Decided(fragment) =
                crate::BezierParallelFragment2::try_new(parallel, range, &policy).unwrap()
            else {
                panic!("the quartic parallel must have one regular authored span");
            };
            let mut fragment = BezierSplitFragment2::AnalyticParallel(fragment);
            if reversed {
                fragment = fragment.reversed().unwrap();
            }
            let boundary = CurveRegionBoundaryLoop2::new(vec![fragment], &policy).unwrap();
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed {
                    CurveBoundaryInteriorSide2::Right
                } else {
                    CurveBoundaryInteriorSide2::Left
                }],
            )
            .unwrap();
            let trimmed = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let extended_work = || {
                region.chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let extended = hyperreal::dispatch_trace::with_recording(extended_work);
            #[cfg(not(feature = "dispatch-trace"))]
            let extended = extended_work();
            #[cfg(feature = "dispatch-trace")]
            let trace = hyperreal::dispatch_trace::take_trace();
            let extended = extended.unwrap();
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(extended.value.candidate_count() > trimmed.candidate_count());
            #[cfg(feature = "dispatch-trace")]
            assert!(
                trace.path_count(
                    "hypercurve",
                    "curve-region-retained-chamfer",
                    "certified-cut-chord",
                ) > 0,
                "the finite envelope must retain the original cut-chord certificate: {trace:?}",
            );
        }
    }
}

#[test]
fn one_fragment_selected_loop_one_sided_chamfers_do_not_duplicate_the_source() {
    let setback = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            for (previous_setback, next_setback) in [
                (Real::zero(), setback.clone()),
                (setback.clone(), Real::zero()),
            ] {
                let region = one_fragment_selected_corner_region(reversed, &policy);
                let result = region
                    .chamfer_loop_vertex_by_setbacks_with_policy(
                        0,
                        0,
                        previous_setback,
                        next_setback,
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the one-sided selected seam must chamfer: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                assert_one_fragment_edit_shape(corner_regions(&result.value), 1, 1);
                for_each_corner_region(corner_regions(&result.value), |edited| {
                    assert_eq!(
                        edited.boundary_loops()[0].fragments().len(),
                        2,
                        "a one-sided chamfer must not retain a second copy of its only source fragment"
                    );
                });
            }
        }
    }
}

#[test]
fn one_fragment_selected_loop_fillets_from_one_interval() {
    let radius = (Real::one() / Real::from(4_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = one_fragment_selected_corner_region(reversed, &policy);
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    0,
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the one-fragment selected loop must fillet: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(
                !result.value.solutions().is_empty(),
                "the authored seam has an admissible fillet: policy={policy:?}, reversed={reversed}, result={:?}",
                result.value
            );
            assert_one_fragment_edit_shape(fillet_regions(&result.value), 1, 1);
            for_each_corner_region(fillet_regions(&result.value), |edited| {
                assert!(
                    edited.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .any(|fragment| {
                            matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                        })
                );
            });
        }
    }
}

#[test]
fn one_fragment_ph_loop_fillets_through_rational_self_contact() {
    let root_three = Real::from(3_i8).sqrt().unwrap();
    let control_x = (Real::one() / Real::from(18_i8)).unwrap();
    let control_y = -((Real::one() / (&root_three * Real::from(6_i8))).unwrap());
    let radius = ((Real::from(7_i8) * &root_three) / Real::from(768_i16)).unwrap();
    let seam = p(0, 0);
    let source = RationalBezier2::try_new(
        vec![
            seam.clone(),
            Point2::new(control_x.clone(), control_y.clone()),
            Point2::new(-control_x, control_y),
            seam.clone(),
        ],
        vec![Real::one(); 4],
    )
    .expect("the closed cubic PH source is finite");

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let range = CurveParameterRange2::new_validated(
                CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                CurveParameter2::from(BezierParameter2::Exact(Real::one())),
            );
            let mut fragment = BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    BezierSelectedFiberSource2::Rational(source.clone()),
                    range,
                    CurvePoint2::from(seam.clone()),
                    CurvePoint2::from(seam.clone()),
                ),
            );
            if reversed {
                fragment = fragment
                    .reversed()
                    .expect("the selected closed PH cubic reverses exactly");
            }
            let boundary = CurveRegionBoundaryLoop2::new(vec![fragment], &policy)
                .expect("the selected one-fragment PH loop closes exactly");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed {
                    CurveBoundaryInteriorSide2::Left
                } else {
                    CurveBoundaryInteriorSide2::Right
                }],
            )
            .expect("the selected PH loop has authored topology");
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    0,
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the exact PH self-contact must fillet: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(!result.value.solutions().is_empty());
            assert_one_fragment_edit_shape(fillet_regions(&result.value), 1, 1);
        }
    }
}

#[test]
fn closed_ph_corner_edits_preserve_both_normalized_source_lobes() {
    // This is the regular closed PH cubic used by the direct projective
    // self-contact regression: P(t)=(t(1-t)(1-2t), -sqrt(3)t(1-t))/6.
    // The exterior cuts at -1/2 and 3/2 enclose two source lobes touching
    // at P(0)=P(1). Regularization may split that interval across loops.
    let root_three = Real::from(3_i8).sqrt().unwrap();
    let control_x = (Real::one() / Real::from(18_i8)).unwrap();
    let control_y = -((&root_three / Real::from(18_i8)).unwrap());
    let radius = ((Real::from(13_i8) * &root_three) / Real::from(48_i8)).unwrap();
    let setback = (Real::from(7_i8).sqrt().unwrap() / Real::from(8_i8)).unwrap();
    let previous_cut = Point2::new(
        (Real::one() / Real::from(4_i8)).unwrap(),
        (&root_three / Real::from(8_i8)).unwrap(),
    );
    let next_cut = Point2::new(
        -(Real::one() / Real::from(4_i8)).unwrap(),
        (&root_three / Real::from(8_i8)).unwrap(),
    );
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(control_x.clone(), control_y.clone()),
        Point2::new(-control_x, control_y),
        p(0, 0),
    );
    // Independent samples P(-1/4), P(1/4), P(3/4), P(5/4) require
    // both exterior source arms and the original lower lobe to survive.
    // The selected fillet circle has center (0,17sqrt(3)/48), and
    // |P(t)-center|^2-r^2=(s^2-1)^2(4s^2+9)/36 for s=t-1/2.
    // Thus it has no additional source crossing. Its lowest point is
    // sqrt(3)/12; the chamfer lies at sqrt(3)/8. Both lie above the
    // upper interior sample, while the lower lobe reaches -sqrt(3)/24.
    let boundary_samples = [
        Point2::new(-q(5, 64), &root_three * q(5, 96)),
        Point2::new(q(1, 64), -(&root_three * q(1, 32))),
        Point2::new(-q(1, 64), -(&root_three * q(1, 32))),
        Point2::new(q(5, 64), &root_three * q(5, 96)),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let mut fragment = BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Cubic(source.clone()),
            };
            if reversed {
                fragment = fragment
                    .reversed()
                    .expect("the materialized closed PH cubic reverses exactly");
            }
            let boundary = CurveRegionBoundaryLoop2::new(vec![fragment], &policy)
                .expect("the one-fragment PH loop closes exactly");
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed {
                    CurveBoundaryInteriorSide2::Left
                } else {
                    CurveBoundaryInteriorSide2::Right
                }],
            )
            .expect("the one-fragment PH loop has authored topology");
            let fillets = region
                .fillet_loop_vertex_with_policy(
                    0,
                    0,
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the one-fragment projective fillet must rebuild: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(fillets.certainty, CurveCertainty::Certified);
            let assert_extended_set = |solutions: &[CurveRegion2]| {
                let mut found = false;
                for_each_corner_region(solutions, |edited| {
                    assert!(edited.has_regularized_filled_left_topology(&policy));
                    let has_cut = |cut: &Point2| {
                        edited.boundary_loops().iter().any(|boundary| {
                            boundary
                                .fragments()
                                .iter()
                                .any(|fragment| retained_fragment_has_exact_endpoint(fragment, cut))
                        })
                    };
                    if !has_cut(&next_cut) || !has_cut(&previous_cut) {
                        return;
                    }
                    for (index, point) in boundary_samples.iter().enumerate() {
                        let location = edited
                            .classify_point_with_policy(&point.clone().into(), &policy)
                            .unwrap();
                        assert_eq!(location.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            location.value,
                            Classification::Decided(RegionPointLocation::Boundary),
                            "source sample {index}, reversed={reversed}, policy={policy:?}",
                        );
                    }
                    for (point, expected) in [
                        (
                            Point2::new(Real::zero(), -q(1, 24)),
                            RegionPointLocation::Inside,
                        ),
                        (
                            Point2::new(Real::zero(), q(1, 24)),
                            RegionPointLocation::Inside,
                        ),
                        (p(1, 1), RegionPointLocation::Outside),
                    ] {
                        let location = edited
                            .classify_point_with_policy(&point.clone().into(), &policy)
                            .unwrap();
                        assert_eq!(location.certainty, CurveCertainty::Certified);
                        assert_eq!(location.value, Classification::Decided(expected));
                    }
                    found = true;
                });
                assert!(found, "the exact extended source lobes were lost");
            };
            assert_extended_set(fillet_regions(&fillets.value));

            let chamfers = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    0,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the one-fragment projective chamfer must rebuild: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(chamfers.certainty, CurveCertainty::Certified);
            assert_extended_set(corner_regions(&chamfers.value));
        }
    }
}

#[test]
fn one_fragment_retained_ph_loop_extends_fillet_on_one_analytic_carrier() {
    let root_three = Real::from(3_i8).sqrt().unwrap();
    let control_x = (Real::one() / Real::from(18_i8)).unwrap();
    let control_y = -((&root_three / Real::from(18_i8)).unwrap());
    let radius = ((Real::from(13_i8) * &root_three) / Real::from(48_i8)).unwrap();
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(control_x.clone(), control_y.clone()),
        Point2::new(-control_x, control_y),
        p(0, 0),
    );
    let selected_source = RationalBezier2::try_new(
        source.control_points().into_iter().cloned().collect(),
        vec![Real::one(); 4],
    )
    .expect("the closed cubic PH source is finite");

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            for selected in [false, true] {
                let mut fragment = if selected {
                    let seam = p(0, 0);
                    BezierSplitFragment2::SelectedFiber(
                        crate::bezier_split::BezierSelectedFiberFragment2::new(
                            BezierSelectedFiberSource2::Rational(selected_source.clone()),
                            CurveParameterRange2::new_validated(
                                CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                                CurveParameter2::from(BezierParameter2::Exact(Real::one())),
                            ),
                            CurvePoint2::from(seam.clone()),
                            CurvePoint2::from(seam),
                        ),
                    )
                } else {
                    let parallel = source.parallel_left(Real::zero()).unwrap();
                    let Classification::Decided(fragment) =
                        crate::BezierParallelFragment2::try_new(
                            parallel,
                            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                            &policy,
                        )
                        .unwrap()
                    else {
                        panic!("the complete PH analytic span must be regular");
                    };
                    BezierSplitFragment2::AnalyticParallel(fragment)
                };
                if reversed {
                    fragment = fragment.reversed().unwrap();
                }
                let boundary = CurveRegionBoundaryLoop2::new(vec![fragment], &policy).unwrap();
                let region = CurveRegion2::try_new_with_loop_topology(
                    vec![boundary],
                    vec![CurveRegionLoopRole::Material],
                    vec![FillRule::NonZero],
                    vec![if reversed {
                        CurveBoundaryInteriorSide2::Left
                    } else {
                        CurveBoundaryInteriorSide2::Right
                    }],
                )
                .unwrap();
                let extended = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        0,
                        &crate::CurveFillet2::new(radius.clone()),
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the retained projective fillet must rebuild: policy={policy:?}, reversed={reversed}, selected={selected}, error={error:?}"
                        )
                    });
                assert_eq!(extended.certainty, CurveCertainty::Certified);
                let original_support =
                    CurveSupport2::from_fragment(&region.boundary_loops()[0].fragments()[0]);
                let unit = CurveParameterRange2::unit();
                let mut found_extension = false;
                for_each_corner_region(fillet_regions(&extended.value), |edited| {
                    found_extension |=
                        edited.boundary_loops()[0]
                            .fragments()
                            .iter()
                            .any(|fragment| {
                                let same_support = match (
                                    CurveSupport2::from_fragment(fragment),
                                    &original_support,
                                ) {
                                    (
                                        CurveSupport2::Bezier(retained),
                                        CurveSupport2::Bezier(original),
                                    ) => &retained == original,
                                    (
                                        CurveSupport2::Parallel(retained),
                                        CurveSupport2::Parallel(original),
                                    ) => &retained == original,
                                    _ => false,
                                };
                                same_support
                                    && matches!(
                                        crate::bezier_split::CurveParameterDomain2::new(
                                            &unit, None
                                        )
                                        .contains_finite_range(
                                            &fragment.curve_region_parameter_range(),
                                            &policy
                                        )
                                        .unwrap(),
                                        Classification::Decided(false),
                                    )
                            });
                });
                assert!(
                    found_extension,
                    "the exterior interval must retain its original support"
                );
            }
        }
    }
}

fn parallel_pair_fillet_region(
    previous_retained: bool,
    next_retained: bool,
    reversed: bool,
    policy: &CurveContext,
) -> CurveRegion2 {
    let fragment = |start: Point2, end: Point2, retained: bool| {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let curve = QuadraticBezier2::new(start.clone(), start.lerp(&end, half), end.clone());
        if retained {
            let parallel = curve
                .parallel_left(Real::zero())
                .expect("the exact-line analytic parallel is valid");
            let range = BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            );
            let Classification::Decided(fragment) =
                crate::BezierParallelFragment2::try_new(parallel, range, policy)
                    .expect("the complete analytic range is valid")
            else {
                panic!("the exact analytic range must be decided");
            };
            BezierSplitFragment2::AnalyticParallel(fragment)
        } else {
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(curve),
            }
        }
    };
    let mut fragments = vec![
        fragment(p(0, 0), p(4, 0), previous_retained),
        fragment(p(4, 0), p(4, 4), next_retained),
        fragment(p(4, 4), p(0, 0), false),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the exact fixture reverses"))
            .collect();
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the parallel-pair fixture closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the parallel-pair fixture has authored topology")
}

#[test]
fn direct_mixed_and_retained_parallel_pairs_share_the_fillet_kernel() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (previous_retained, next_retained) in [(false, true), (true, false), (true, true)] {
            for reversed in [false, true] {
                let region = parallel_pair_fillet_region(
                    previous_retained,
                    next_retained,
                    reversed,
                    &policy,
                );
                let corner = if reversed { 2 } else { 1 };
                let result = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(Real::one()),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the unified parallel pair must fillet: policy={policy:?}, previous_retained={previous_retained}, next_retained={next_retained}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let filleted = {
                    let solutions = result.value;
                    let mut candidates = solutions.into_solutions();
                    assert_eq!(candidates.len(), 1, "expected one isolated fillet");
                    candidates.pop().unwrap()
                };
                assert_eq!(
                    filleted
                        .classify_point_with_policy(&p(3, 1).into(), &policy)
                        .expect("the unified parallel-pair fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
            }
        }
    }
}

fn sqrt_half_algebraic_parameter(policy: &CurveContext) -> BezierParameter2 {
    let polynomial = BezierParameterPolynomial::try_new_power_basis(
        vec![Real::from(-1_i8), Real::zero(), Real::from(2_i8)],
        policy,
    )
    .expect("the quadratic parameter polynomial is valid");
    let Classification::Decided(polynomial) = polynomial else {
        panic!("the exact polynomial must be decided");
    };
    let interval = BezierParameterInterval::try_new(
        (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
        (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
        policy,
    )
    .expect("the isolating interval is valid");
    let Classification::Decided(interval) = interval else {
        panic!("the exact interval must be decided");
    };
    let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
        .expect("sqrt(1/2) has one root in the supplied interval");
    let Classification::Decided(parameter) = parameter else {
        panic!("the exact algebraic parameter must be decided");
    };
    BezierParameter2::Algebraic(parameter)
}

fn sqrt_third_algebraic_parameter(policy: &CurveContext) -> BezierParameter2 {
    let polynomial = BezierParameterPolynomial::try_new_power_basis(
        vec![Real::from(-1_i8), Real::zero(), Real::from(3_i8)],
        policy,
    )
    .expect("the quadratic parameter polynomial is valid");
    let Classification::Decided(polynomial) = polynomial else {
        panic!("the exact polynomial must be decided");
    };
    let interval = BezierParameterInterval::try_new(
        (Real::one() / Real::from(2_i8)).unwrap(),
        (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
        policy,
    )
    .expect("the isolating interval is valid");
    let Classification::Decided(interval) = interval else {
        panic!("the exact interval must be decided");
    };
    let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
        .expect("sqrt(1/3) has one root in the supplied interval");
    let Classification::Decided(parameter) = parameter else {
        panic!("the exact algebraic parameter must be decided");
    };
    BezierParameter2::Algebraic(parameter)
}

#[derive(Clone, Copy)]
enum SelectedCircleFilletNeighbor2 {
    RationalArc(i8),
    ElevatedRationalArc(i8),
    MajorRationalArc(i8),
    ElevatedMajorRationalArc(i8),
    ConcentricRationalArc,
    SelectedCircle,
    AnalyticParallel(bool),
    DirectLine,
    DirectBezier,
}

fn selected_circle_fixture_center(policy: &CurveContext) -> (BezierParameter2, CurvePoint2) {
    let center_parameter = sqrt_half_algebraic_parameter(policy);
    let BezierParameter2::Algebraic(parameter) = &center_parameter else {
        panic!("sqrt(1/2) must remain an isolated algebraic parameter");
    };
    let center_source = RationalBezier2::try_new(
        vec![p(0, 0), p(0, 0), p(1, 0)],
        vec![Real::one(), Real::one(), Real::one()],
    )
    .expect("the selected center source is a valid rational quadratic");
    let center = CurvePoint2::from(crate::tests::decided(
        center_source
            .point_at_algebraic_parameter(parameter, policy)
            .expect("the selected center has an exact rational image"),
    ));
    (center_parameter, center)
}

fn selected_circle_neighbor_region(
    policy: &CurveContext,
    neighbor: SelectedCircleFilletNeighbor2,
    reversed: bool,
) -> CurveRegion2 {
    let (center_parameter, center) = selected_circle_fixture_center(policy);
    let BezierParameter2::Algebraic(center_parameter) = &center_parameter else {
        panic!("sqrt(1/2) must remain an isolated algebraic parameter");
    };
    let Classification::Decided(Some(support)) =
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
            &center,
            (1, 0),
            Real::one(),
            true,
            policy,
        )
        .expect("the selected center defines an exact clockwise semicircle")
    else {
        panic!("the nonzero selected semicircle must be decided");
    };

    let alpha = (Real::one() / Real::from(2_i8)).unwrap();
    let half_sqrt_two = alpha.clone().sqrt().unwrap();
    let start = Point2::new(&alpha + Real::one(), Real::zero());
    let join = Point2::new(&alpha - Real::one(), Real::zero());
    let arc_end = Point2::new(alpha.clone(), Real::one());
    let neighbor = match neighbor {
        SelectedCircleFilletNeighbor2::RationalArc(homogeneous_scale)
        | SelectedCircleFilletNeighbor2::ElevatedRationalArc(homogeneous_scale)
        | SelectedCircleFilletNeighbor2::MajorRationalArc(homogeneous_scale)
        | SelectedCircleFilletNeighbor2::ElevatedMajorRationalArc(homogeneous_scale) => {
            let scale = Real::from(homogeneous_scale);
            let major = matches!(
                neighbor,
                SelectedCircleFilletNeighbor2::MajorRationalArc(_)
                    | SelectedCircleFilletNeighbor2::ElevatedMajorRationalArc(_)
            );
            let arc = RationalQuadraticBezier2::try_new(
                join.clone(),
                Point2::new(alpha.clone(), Real::zero()),
                arc_end.clone(),
                scale.clone(),
                if major {
                    -(&scale * half_sqrt_two)
                } else {
                    &scale * half_sqrt_two
                },
                scale,
            )
            .expect("the retained quarter circle has a valid homogeneous gauge");
            assert!(
                matches!(
                    crate::arc_bezier::rational_quadratic_circular_arc(&arc, policy),
                    Ok(Classification::Decided(Some(_)))
                ),
                "the authored exact quarter circle must promote as circular"
            );
            let curve = if matches!(
                neighbor,
                SelectedCircleFilletNeighbor2::ElevatedRationalArc(_)
                    | SelectedCircleFilletNeighbor2::ElevatedMajorRationalArc(_)
            ) {
                BezierSubcurve2::Rational(
                    RationalBezier2::from(arc)
                        .elevated_to_degree(5)
                        .expect("the circular conic elevates exactly"),
                )
            } else {
                BezierSubcurve2::RationalQuadratic(arc)
            };
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve,
            }
        }
        SelectedCircleFilletNeighbor2::ConcentricRationalArc => {
            let arc = RationalQuadraticBezier2::try_new(
                join.clone(),
                Point2::new(&alpha - Real::one(), Real::one()),
                arc_end.clone(),
                Real::one(),
                half_sqrt_two,
                Real::one(),
            )
            .expect("the concentric quarter circle has a valid homogeneous gauge");
            assert!(matches!(
                crate::arc_bezier::rational_quadratic_circular_arc(&arc, policy),
                Ok(Classification::Decided(Some(_)))
            ));
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::RationalQuadratic(arc),
            }
        }
        SelectedCircleFilletNeighbor2::SelectedCircle => {
            let neighbor_center_source = RationalBezier2::try_new(
                vec![p(0, 1), p(0, 1), p(-1, 1)],
                vec![Real::one(), Real::one(), Real::one()],
            )
            .expect("the neighboring selected center source is a valid rational quadratic");
            let neighbor_center = CurvePoint2::from(crate::tests::decided(
                neighbor_center_source
                    .point_at_algebraic_parameter(center_parameter, policy)
                    .expect("the neighboring center has an exact rational image"),
            ));
            let Classification::Decided(Some(neighbor_support)) =
                crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                    &neighbor_center,
                    (0, -1),
                    Real::one(),
                    false,
                    policy,
                )
                .expect("the neighboring selected center defines an exact semicircle")
            else {
                panic!("the neighboring selected semicircle must be decided");
            };
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            let Classification::Decided(neighbor_fragment) =
                crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                    neighbor_support,
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                        Real::zero(),
                    ),
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(half),
                    false,
                    policy,
                )
                .expect("the neighboring selected quarter has a valid range")
            else {
                panic!("the neighboring selected quarter must be decided");
            };
            BezierSplitFragment2::AlgebraicCuspSemicircle(neighbor_fragment)
        }
        SelectedCircleFilletNeighbor2::AnalyticParallel(curved) => {
            let analytic = if curved {
                QuadraticBezier2::new(
                    join.clone(),
                    Point2::new(alpha.clone(), Real::zero()),
                    arc_end.clone(),
                )
                .parallel_left(Real::zero())
                .expect("the neighboring curved analytic parallel is valid")
            } else {
                let half = (Real::one() / Real::from(2_i8)).unwrap();
                QuadraticBezier2::new(join.clone(), join.lerp(&arc_end, half), arc_end.clone())
                    .parallel_left(Real::zero())
                    .expect("the neighboring exact-line parallel is valid")
            };
            let range = BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            );
            let Classification::Decided(analytic) =
                crate::BezierParallelFragment2::try_new(analytic, range, policy)
                    .expect("the neighboring analytic range is valid")
            else {
                panic!("the neighboring analytic fragment must be decided");
            };
            BezierSplitFragment2::AnalyticParallel(analytic)
        }
        SelectedCircleFilletNeighbor2::DirectBezier => {
            let one_quarter = (Real::one() / Real::from(4_i8)).unwrap();
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                    join.clone(),
                    join.lerp(&arc_end, one_quarter),
                    arc_end.clone(),
                )),
            }
        }
        SelectedCircleFilletNeighbor2::DirectLine => BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(join.clone(), arc_end.clone())
                    .expect("the neighboring exact line is nondegenerate"),
            )),
        },
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicCuspSemicircle(
            crate::BezierAlgebraicCuspSemicircleFragment2::full(support, policy),
        ),
        neighbor,
        quadratic_fragment(
            arc_end,
            Point2::new(
                &alpha + (Real::one() / Real::from(2_i8)).unwrap(),
                (Real::one() / Real::from(2_i8)).unwrap(),
            ),
            start,
        ),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the exact fixture reverses"))
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("mixed selected-circle/rational-arc endpoints close exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the mixed exact loop has authored topology")
}

fn split_selected_circle_region(
    policy: &CurveContext,
    region: &CurveRegion2,
    split: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    reversed: bool,
) -> CurveRegion2 {
    let source = region.boundary_loops()[0].fragments();
    let BezierSplitFragment2::AlgebraicCuspSemicircle(circle) = &source[0] else {
        panic!("the mixed fixture starts on its selected circle")
    };
    assert!(!circle.is_reversed());
    let fragment = |start, end| match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
        circle.semicircle().clone(),
        start,
        end,
        false,
        policy,
    )
    .expect("the authored selected-circle split is exact")
    {
        Classification::Decided(fragment) => {
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
        }
        Classification::Uncertain(reason) => {
            panic!("the authored selected-circle split must decide: {reason:?}")
        }
    };
    let mut fragments = vec![
        fragment(circle.start_parameter().clone(), split.clone()),
        fragment(split, circle.end_parameter().clone()),
    ];
    fragments.extend(source[1..].iter().cloned());
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| {
                fragment
                    .reversed()
                    .expect("the split fixture reverses exactly")
            })
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the split selected-circle fixture closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the split selected-circle fixture has authored topology")
}

fn split_selected_circle_neighbor_region(
    policy: &CurveContext,
    neighbor: SelectedCircleFilletNeighbor2,
    split: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    reversed: bool,
) -> CurveRegion2 {
    split_selected_circle_region(
        policy,
        &selected_circle_neighbor_region(policy, neighbor, false),
        split,
        reversed,
    )
}

fn independently_reframed_selected_circle_region(
    policy: &CurveContext,
    region: &CurveRegion2,
    split: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    reversed: bool,
) -> CurveRegion2 {
    let source = region.boundary_loops()[0].fragments();
    let BezierSplitFragment2::AlgebraicCuspSemicircle(circle) = &source[0] else {
        panic!("the reframed fixture starts on its selected circle")
    };
    let Classification::Decided(center) = circle
        .semicircle()
        .center_point_evidence(policy)
        .expect("the selected center remains exact")
    else {
        panic!("the selected center must remain representable")
    };
    let Classification::Decided(Some(reframed)) =
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
            &center,
            (0, -1),
            circle.semicircle().radial_distance().clone(),
            circle.semicircle().is_clockwise(),
            policy,
        )
        .expect("the independent cardinal frame is exact")
    else {
        panic!("the independent selected-circle frame must be decided")
    };
    assert_eq!(
        circle
            .semicircle()
            .shared_frame_chart_relation(&reframed, policy),
        Classification::Decided(None),
        "the fixture must not use shared-frame run evidence"
    );
    let relation = policy
        .strict_predicate_pass(|| circle.semicircle().pair_intersections(&reframed, policy))
        .expect("the equal supporting circles compare exactly");
    let Classification::Decided(
        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap),
    ) = relation
    else {
        panic!("the independently framed half circles must publish their exact overlap")
    };
    let mapped_split = overlap.map_parameter(&split, true);
    let mapped_end = overlap.map_parameter(circle.end_parameter(), true);
    let fragment =
        |support, start, end| match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
            support, start, end, false, policy,
        )
        .expect("the independently framed selected-circle range is exact")
        {
            Classification::Decided(fragment) => {
                BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
            }
            Classification::Uncertain(reason) => {
                panic!("the independently framed range must decide: {reason:?}")
            }
        };
    let mut fragments = vec![
        fragment(
            circle.semicircle().clone(),
            circle.start_parameter().clone(),
            split,
        ),
        fragment(reframed, mapped_split, mapped_end),
    ];
    fragments.extend(source[1..].iter().cloned());
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| {
                fragment
                    .reversed()
                    .expect("the independently framed fixture reverses exactly")
            })
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the independently framed selected-circle fixture closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the independently framed selected-circle fixture has authored topology")
}

fn selected_circle_direct_line_fillet_cut(
    source: &CurveRegion2,
    policy: &CurveContext,
) -> crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2 {
    let BezierSplitFragment2::AlgebraicCuspSemicircle(source_circle) =
        &source.boundary_loops()[0].fragments()[0]
    else {
        panic!("the endpoint fixture starts on its selected circle")
    };
    let baseline = source
        .fillet_loop_vertex_with_policy(
            0,
            1,
            &crate::CurveFillet2::new(q(1, 10)),
            CurveCornerMode2::TrimOnly,
            policy,
        )
        .expect("the unsplit selected-circle/line fillet is exact");
    let baseline = {
        let solutions = baseline.value;
        let mut candidates = solutions.into_solutions();
        assert_eq!(candidates.len(), 1, "expected one isolated fillet");
        candidates.pop().unwrap()
    };
    let retained_source = baseline
        .boundary_loops()
        .iter()
        .flat_map(|boundary| boundary.fragments())
        .filter_map(|fragment| {
            let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment else {
                return None;
            };
            match source_circle
                .semicircle()
                .shared_frame_chart_relation(fragment.semicircle(), policy)
            {
                Classification::Decided(Some(false)) => Some(fragment),
                Classification::Decided(Some(true) | None) => None,
                Classification::Uncertain(reason) => {
                    panic!("the retained source circle relation must decide: {reason:?}")
                }
            }
        })
        .collect::<Vec<_>>();
    let [retained_source] = retained_source.as_slice() else {
        panic!("the baseline fillet must retain one exact source-circle range")
    };
    // Normalization may reverse traversal. The ascending source range
    // still owns the untouched start and the exact interior fillet cut.
    assert_eq!(
        retained_source
            .start_parameter()
            .cmp_by_refinement(source_circle.start_parameter(), policy)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Equal),
    );
    assert_eq!(
        source_circle
            .contains_parameter(retained_source.end_parameter(), false, false, policy)
            .unwrap(),
        Classification::Decided(true),
    );
    retained_source.end_parameter().clone()
}

fn selected_circle_direct_line_region_from_support(
    support: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    policy: &CurveContext,
    reversed: bool,
) -> CurveRegion2 {
    let alpha = q(1, 2);
    let start = Point2::new(&alpha + Real::one(), Real::zero());
    let join = Point2::new(&alpha - Real::one(), Real::zero());
    let end = Point2::new(alpha.clone(), Real::one());
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicCuspSemicircle(
            crate::BezierAlgebraicCuspSemicircleFragment2::full(support, policy),
        ),
        quadratic_fragment(join.clone(), join.lerp(&end, q(1, 2)), end.clone()),
        quadratic_fragment(end, Point2::new(&alpha + q(1, 2), q(1, 2)), start),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the exact fixture reverses"))
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    let boundary =
        CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(fragments, None, policy)
            .expect("the selected-frame circle/line fixture closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the selected-frame circle/line fixture has authored topology")
}

fn selected_chord_normal_circle(
    policy: &CurveContext,
) -> crate::bezier_offset::BezierAlgebraicCuspSemicircle2 {
    let (_, center) = selected_circle_fixture_center(policy);
    let anchor_start = match crate::BezierAlgebraicChord2::translated_endpoint(
        &center,
        &Real::zero(),
        &Real::one(),
        policy,
    )
    .expect("the chord-normal anchor translation is exact")
    {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("the chord-normal anchor must remain exact: {reason:?}")
        }
    };
    let anchor = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
        anchor_start,
        center.clone(),
        crate::bezier_offset::BezierAlgebraicChordAxisDirection2::NegativeY,
        policy,
    );
    match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
        center,
        anchor,
        Real::one(),
        true,
        policy,
    )
    .expect("the retained chord-normal circle is exact")
    {
        Classification::Decided(Some(circle)) => circle,
        result => panic!("the nonzero chord-normal circle must construct: {result:?}"),
    }
}

fn correlated_chord_pair_normal_circle(
    policy: &CurveContext,
) -> crate::bezier_offset::BezierAlgebraicCuspSemicircle2 {
    let selected_point = |start: Point2, end: Point2, denominator| {
        let source = RationalBezier2::try_new(vec![start, end], vec![Real::one(), Real::one()])
            .expect("the selected chord endpoint source is finite");
        crate::tests::decided(
            crate::rational_bezier_general::exact_contact_point_evidence(
                &source,
                &positive_inverse_sqrt_parameter(denominator, policy),
                policy,
            )
            .expect("the selected chord endpoint has exact evidence"),
        )
    };
    let chord = |start, end| match crate::BezierAlgebraicChord2::try_new(start, end, policy)
        .expect("the retained center support is valid")
    {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => {
            panic!("the retained center support must decide: {reason:?}")
        }
    };
    let horizontal = chord(
        selected_point(p(0, -1), p(-1, -1), 2),
        selected_point(p(0, -1), p(1, -1), 3),
    );
    let vertical = chord(
        selected_point(p(1, 0), p(1, -1), 5),
        selected_point(p(1, 0), p(1, 1), 7),
    );
    let first = horizontal
        .parallel_left_retained(Real::one(), policy)
        .expect("the horizontal center support offsets exactly");
    let second = vertical
        .parallel_left_retained(Real::one(), policy)
        .expect("the vertical center support offsets exactly");
    let center = match first
        .supporting_line_intersection(&second, policy)
        .expect("the retained center supports intersect exactly")
    {
        Classification::Decided(Some(
            center @ CurvePoint2(CurvePointData2::AlgebraicChordPair(_)),
        )) => center,
        result => panic!("the selected center must retain its chord pair: {result:?}"),
    };
    match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
        center,
        horizontal,
        Real::one(),
        true,
        policy,
    )
    .expect("the chord-pair-normal circle is exact")
    {
        Classification::Decided(Some(circle)) => circle,
        result => panic!("the chord-pair-normal circle must construct: {result:?}"),
    }
}

fn correlated_chord_pair_collapse_region(policy: &CurveContext, reversed: bool) -> CurveRegion2 {
    let circle = correlated_chord_pair_normal_circle(policy);
    let start = p(0, 1);
    let join = p(0, -1);
    assert_eq!(
        circle
            .start_point_evidence(policy)
            .expect("the selected start point is exact")
            .map(|point| point.same_point(&start.clone().into(), policy)),
        Classification::Decided(Classification::Decided(true)),
    );
    assert_eq!(
        circle
            .end_point_evidence(policy)
            .expect("the selected end point is exact")
            .map(|point| point.same_point(&join.clone().into(), policy)),
        Classification::Decided(Classification::Decided(true)),
    );
    let line_end = p(-2, -1);
    let parallel = QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(join.clone(), line_end.clone()).unwrap(),
    )
    .parallel_left(Real::zero())
    .expect("the incident analytic line is regular");
    let range = BezierParameterRange2::new_validated(
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    );
    let Classification::Decided(parallel) =
        crate::BezierParallelFragment2::try_new(parallel, range, policy)
            .expect("the incident analytic line range is valid")
    else {
        panic!("the incident analytic line must construct");
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicCuspSemicircle(
            crate::BezierAlgebraicCuspSemicircleFragment2::full(circle, policy),
        ),
        BezierSplitFragment2::AnalyticParallel(parallel),
        quadratic_fragment(line_end, p(-2, 1), start),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the retained loop reverses"))
            .collect();
    }
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the chord-pair/parallel loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Left
        } else {
            CurveBoundaryInteriorSide2::Right
        }],
    )
    .expect("the chord-pair/parallel loop has authored topology")
}

fn selected_parallel_normal_circle(
    policy: &CurveContext,
) -> crate::bezier_offset::BezierAlgebraicCuspSemicircle2 {
    let half = q(1, 2);
    let center_support = QuadraticBezier2::new(
        Point2::new(half.clone(), half.clone()),
        Point2::new(half.clone(), half.clone()),
        Point2::new(half.clone(), -half),
    )
    .parallel_left(Real::zero())
    .expect("the selected vertical center support is regular");
    match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
        center_support,
        sqrt_half_algebraic_parameter(policy).into(),
        Real::one(),
        true,
        policy,
    )
    .expect("the retained parallel-normal circle is exact")
    {
        Classification::Decided(Some(circle)) => circle,
        result => panic!("the nonzero parallel-normal circle must construct: {result:?}"),
    }
}

fn selected_fillet_disjoint_square(policy: &CurveContext) -> CurveRegion2 {
    CurveRegion2::new(vec![
        CurveRegionBoundaryLoop2::new(
            vec![
                quadratic_fragment(p(4, 4), p(5, 4), p(6, 4)),
                quadratic_fragment(p(6, 4), p(6, 5), p(6, 6)),
                quadratic_fragment(p(6, 6), p(5, 6), p(4, 6)),
                quadratic_fragment(p(4, 6), p(4, 5), p(4, 4)),
            ],
            policy,
        )
        .expect("the disjoint exact loop closes"),
    ])
    .expect("one disjoint exact loop")
}

fn selected_circle_pair_corner(region: &CurveRegion2) -> usize {
    let fragments = region.boundary_loops()[0].fragments();
    (0..fragments.len())
        .find(|index| {
            matches!(
                (
                    &fragments[(index + fragments.len() - 1) % fragments.len()],
                    &fragments[*index],
                ),
                (
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                )
            )
        })
        .expect("the fixture retains its selected-circle pair corner")
}

fn selected_circle_rational_arc_corner(region: &CurveRegion2) -> usize {
    let fragments = region.boundary_loops()[0].fragments();
    (0..fragments.len())
        .find(|index| {
            let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
            let next = &fragments[*index];
            let is_rational_arc = |fragment: &BezierSplitFragment2| {
                matches!(
                    fragment,
                    BezierSplitFragment2::Materialized {
                        curve: BezierSubcurve2::RationalQuadratic(_) | BezierSubcurve2::Rational(_),
                        ..
                    }
                )
            };
            (matches!(previous, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                && is_rational_arc(next))
                || (is_rational_arc(previous)
                    && matches!(next, BezierSplitFragment2::AlgebraicCuspSemicircle(_)))
        })
        .expect("the fixture retains its mixed circular corner")
}

fn selected_circle_collapsed_arc_offset_region(
    policy: &CurveContext,
    reversed: bool,
) -> CurveRegion2 {
    let center_parameter = sqrt_half_algebraic_parameter(policy);
    let BezierParameter2::Algebraic(center_parameter) = &center_parameter else {
        unreachable!("the selected center parameter is algebraic")
    };
    let center_source =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 0), p(1, 0)], vec![Real::one(); 3]).unwrap();
    let center = CurvePoint2::from(crate::tests::decided(
        center_source
            .point_at_algebraic_parameter(center_parameter, policy)
            .unwrap(),
    ));
    let Classification::Decided(Some(circle)) =
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
            &center,
            (1, 0),
            Real::from(2_i8),
            true,
            policy,
        )
        .unwrap()
    else {
        panic!("the radius-two selected circle must construct");
    };
    let selected_start = Point2::new(q(5, 2), Real::zero());
    let join = Point2::new(q(-3, 2), Real::zero());
    let arc_end = Point2::new(q(-1, 2), Real::one());
    let half_sqrt_two = q(1, 2).sqrt().unwrap();
    let arc = RationalQuadraticBezier2::try_new(
        join.clone(),
        Point2::new(q(-3, 2), Real::one()),
        arc_end.clone(),
        Real::one(),
        half_sqrt_two,
        Real::one(),
    )
    .unwrap();
    assert!(matches!(
        crate::arc_bezier::rational_quadratic_circular_arc(&arc, policy),
        Ok(Classification::Decided(Some(_)))
    ));
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicCuspSemicircle(
            crate::BezierAlgebraicCuspSemicircleFragment2::full(circle, policy),
        ),
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::RationalQuadratic(arc),
        },
        quadratic_fragment(
            arc_end.clone(),
            arc_end.lerp(&selected_start, q(1, 2)),
            selected_start,
        ),
    ];
    if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
    }
    CurveRegion2::try_new_with_loop_topology(
        vec![CurveRegionBoundaryLoop2::new(fragments, policy).unwrap()],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![if reversed {
            CurveBoundaryInteriorSide2::Left
        } else {
            CurveBoundaryInteriorSide2::Right
        }],
    )
    .unwrap()
}

fn assert_disjoint_square_replay_preserves_set(
    source: &CurveRegion2,
    result: &crate::CurveRegionBooleanResults2,
    policy: &CurveContext,
) {
    // Regularization may split an extended self-touching walk into
    // several loops. The distant square contributes one additional loop.
    assert_eq!(
        result.union().boundary_loops().len(),
        result.difference().boundary_loops().len() + 1
    );
    assert!(result.intersection().is_empty());
    for point in [p(0, 0), p(-1, 0), p(1, 0), p(0, 1), p(-10, -10)] {
        let Classification::Decided(expected) = source
            .classify_point_with_policy(&point.clone().into(), policy)
            .unwrap()
            .value
        else {
            panic!("the authored region must classify the independent replay probe");
        };
        for region in [result.union(), result.difference(), result.xor()] {
            let outcome = region
                .classify_point_with_policy(&point.clone().into(), policy)
                .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert_eq!(outcome.value, Classification::Decided(expected));
        }
    }
    assert_eq!(
        result
            .union()
            .classify_point_with_policy(&p(5, 5).into(), policy)
            .unwrap()
            .value,
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn regularization_orders_retained_circle_branches_at_shared_vertices() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = selected_circle_neighbor_region(
            &policy,
            SelectedCircleFilletNeighbor2::RationalArc(1),
            false,
        );
        let candidates = source
            .chamfer_loop_vertex_by_setbacks_with_policy(
                0,
                selected_circle_rational_arc_corner(&source),
                q(1, 10),
                q(1, 10),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .unwrap();
        assert_eq!(candidates.certainty, CurveCertainty::Certified);
        let mut split_walks = 0;
        for_each_corner_region(corner_regions(&candidates.value), |authored| {
            let normalized = authored.regularized_region_with_policy(&policy).unwrap();
            assert_eq!(normalized.certainty, CurveCertainty::Certified);
            if normalized.value.boundary_loops().len() == 2 {
                split_walks += 1;
            }
            for point in [p(0, 0), p(-1, 0), p(1, 0), p(0, 1), p(-10, -10)] {
                let expected = authored
                    .classify_point_with_policy(&point.clone().into(), &policy)
                    .unwrap()
                    .value;
                assert!(matches!(expected, Classification::Decided(_)));
                let actual = normalized
                    .value
                    .classify_point_with_policy(&point.clone().into(), &policy)
                    .unwrap();
                assert_eq!(actual.certainty, CurveCertainty::Certified);
                assert_eq!(actual.value, expected);
            }
        });
        assert!(
            split_walks > 0,
            "a self-touching authored walk separates into filled loops"
        );
    }
}

#[test]
fn selected_circle_and_retained_rational_arc_fillet_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for homogeneous_scale in [-2_i8, -1_i8, 1_i8, 2_i8] {
            for reversed in [false, true] {
                let region = selected_circle_neighbor_region(
                    &policy,
                    SelectedCircleFilletNeighbor2::RationalArc(homogeneous_scale),
                    reversed,
                );
                let corner = selected_circle_rational_arc_corner(&region);
                let result = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new((Real::one() / Real::from(10_i8)).unwrap()),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the mixed circular corner must fillet exactly: policy={policy:?}, scale={homogeneous_scale}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let filleted = {
                    let solutions = result.value;
                    let mut candidates = solutions.into_solutions();
                    assert_eq!(candidates.len(), 1, "expected one isolated fillet");
                    candidates.pop().unwrap()
                };
                assert_eq!(
                    filleted.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                        .count(),
                    2,
                );
                assert_eq!(
                    filleted
                        .classify_point_with_policy(&p(0, 0).into(), &policy)
                        .expect("the retained fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
                let disjoint = selected_fillet_disjoint_square(&policy);
                let replay = filleted
                    .boolean_regions_with_policy(&disjoint, &policy)
                    .expect("the mixed retained fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(replay.value.union().boundary_loops().len(), 2);
                assert!(replay.value.intersection().is_empty());
            }
        }
    }
}

#[test]
fn selected_circle_and_major_retained_rational_arc_fillet_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for elevated in [false, true] {
            for reversed in [false, true] {
                let neighbor = if elevated {
                    SelectedCircleFilletNeighbor2::ElevatedMajorRationalArc(1)
                } else {
                    SelectedCircleFilletNeighbor2::MajorRationalArc(1)
                };
                let region = selected_circle_neighbor_region(&policy, neighbor, reversed);
                let corner = selected_circle_rational_arc_corner(&region);
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let result = region
                        .fillet_loop_vertex_with_policy(0, corner, &crate::CurveFillet2::new(q(1, 10)), mode, &policy)
                        .unwrap_or_else(|error| {
                            panic!(
                                "the selected-circle/major-conic fillet must complete: policy={policy:?}, elevated={elevated}, reversed={reversed}, mode={mode:?}, error={error:?}"
                            )
                        });
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert!(!result.value.solutions().is_empty());
                }
            }
        }
    }
}

#[test]
fn selected_circle_and_retained_rational_arc_chamfer_extend_exactly() {
    let rational_extension = Point2::new(
        q(-1, 2) - (Real::from(399_i16).sqrt().unwrap() / Real::from(200_i16)).unwrap(),
        q(1, 200),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for homogeneous_scale in [1_i8, 2_i8] {
            for major in [false, true] {
                for elevated in [false, true] {
                    for reversed in [false, true] {
                        let neighbor = match (major, elevated) {
                            (false, false) => {
                                SelectedCircleFilletNeighbor2::RationalArc(homogeneous_scale)
                            }
                            (false, true) => SelectedCircleFilletNeighbor2::ElevatedRationalArc(
                                homogeneous_scale,
                            ),
                            (true, false) => {
                                SelectedCircleFilletNeighbor2::MajorRationalArc(homogeneous_scale)
                            }
                            (true, true) => {
                                SelectedCircleFilletNeighbor2::ElevatedMajorRationalArc(
                                    homogeneous_scale,
                                )
                            }
                        };
                        let region = selected_circle_neighbor_region(&policy, neighbor, reversed);
                        if major && homogeneous_scale == 1 && !reversed {
                            let authored_replay = region
                                .boolean_regions_with_policy(
                                    &selected_fillet_disjoint_square(&policy),
                                    &policy,
                                )
                                .unwrap_or_else(|error| {
                                    panic!(
                                        "the authored major circular region must enter the Boolean kernel: policy={policy:?}, elevated={elevated}, error={error:?}"
                                    )
                                });
                            assert_eq!(authored_replay.certainty, CurveCertainty::Certified);
                            assert_eq!(authored_replay.value.union().boundary_loops().len(), 2);
                            assert!(authored_replay.value.intersection().is_empty());
                        }
                        let corner = selected_circle_rational_arc_corner(&region);
                        let trim = region
                            .chamfer_loop_vertex_by_setbacks_with_policy(
                                0,
                                corner,
                                q(1, 10),
                                q(1, 10),
                                CurveCornerMode2::TrimOnly,
                                &policy,
                            )
                            .unwrap_or_else(|error| {
                                panic!(
                                    "the mixed circular corner has a finite chamfer: policy={policy:?}, scale={homogeneous_scale}, major={major}, elevated={elevated}, reversed={reversed}, error={error:?}"
                                )
                            });
                        let extended = region
                        .chamfer_loop_vertex_by_setbacks_with_policy(
                            0,
                            corner,
                            q(1, 10),
                            q(1, 10),
                            CurveCornerMode2::TrimOrExtend,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "the mixed circular chamfer must extend exactly: policy={policy:?}, scale={homogeneous_scale}, major={major}, elevated={elevated}, reversed={reversed}, error={error:?}"
                            )
                        });
                        assert_eq!(extended.certainty, CurveCertainty::Certified);
                        assert!(
                            extended.value.candidate_count() > trim.value.candidate_count(),
                            "both full circular supports must contribute exterior chamfer cuts"
                        );
                        let mut retained_rational_extension = false;
                        let mut candidate_index = 0;
                        for_each_corner_region(corner_regions(&extended.value), |chamfered| {
                            if !major {
                                assert_eq!(
                                    chamfered
                                        .classify_point_with_policy(&p(0, 0).into(), &policy)
                                        .expect("the extended mixed chamfer remains classifiable",)
                                        .into_value(),
                                    Classification::Decided(RegionPointLocation::Inside),
                                    "policy={policy:?}, scale={homogeneous_scale}, major={major}, elevated={elevated}, reversed={reversed}, candidate={candidate_index}",
                                );
                            }
                            assert!(
                                chamfered.boundary_loops()[0]
                                    .fragments()
                                    .iter()
                                    .any(|fragment| match fragment {
                                        BezierSplitFragment2::Materialized {
                                            curve: BezierSubcurve2::RationalQuadratic(curve),
                                            ..
                                        } => curve.retained_circular_conic().is_some(),
                                        BezierSplitFragment2::SelectedFiber(fragment) => {
                                            fragment.rational_curve().is_some_and(|curve| {
                                                curve.retained_circular_conic().is_some()
                                            })
                                        }
                                        _ => false,
                                    }),
                                "the canonical chamfer must retain exact circle provenance: policy={policy:?}, scale={homogeneous_scale}, major={major}, elevated={elevated}, reversed={reversed}, candidate={candidate_index}",
                            );
                            retained_rational_extension |= chamfered.boundary_loops()[0]
                                .fragments()
                                .iter()
                                .any(|fragment| {
                                    let endpoints = match fragment {
                                        BezierSplitFragment2::Materialized { curve, .. } => {
                                            [Some(curve.start()), Some(curve.end())]
                                        }
                                        BezierSplitFragment2::SelectedFiber(fragment) => [
                                            fragment.start_point().coordinates(),
                                            fragment.end_point().coordinates(),
                                        ],
                                        _ => return false,
                                    };
                                    endpoints.into_iter().flatten().any(|point| {
                                        point
                                            .distance_squared(&rational_extension)
                                            .certified_eq_until(&Real::zero(), -4096)
                                            .as_bool()
                                            == Some(true)
                                    })
                                });
                            if homogeneous_scale == 1 && !reversed {
                                if major {
                                    for probe in [p(5, 4), p(6, 5), p(5, 6), p(4, 5)] {
                                        assert_eq!(
                                            chamfered
                                                .classify_point_with_policy(
                                                    &probe.clone().into(),
                                                    &policy
                                                )
                                                .expect("the distant Boolean probe is finite")
                                                .into_value(),
                                            Classification::Decided(RegionPointLocation::Outside,),
                                            "policy={policy:?}, elevated={elevated}, candidate={candidate_index}, probe={probe:?}",
                                        );
                                    }
                                }
                                let replay = chamfered
                                    .boolean_regions_with_policy(
                                        &selected_fillet_disjoint_square(&policy),
                                    &policy,
                                )
                                .unwrap_or_else(|error| {
                                    panic!(
                                        "the retained circular chamfer must re-enter the Boolean kernel: policy={policy:?}, major={major}, elevated={elevated}, candidate={candidate_index}, error={error:?}"
                                    )
                                });
                                assert_eq!(
                                    replay.certainty,
                                    CurveCertainty::Certified,
                                    "policy={policy:?}, scale={homogeneous_scale}, major={major}, elevated={elevated}, reversed={reversed}, candidate={candidate_index}",
                                );
                                assert_disjoint_square_replay_preserves_set(
                                    chamfered,
                                    &replay.value,
                                    &policy,
                                );
                                assert!(replay.value.intersection().is_empty());
                            }
                            candidate_index += 1;
                        });
                        assert!(
                            retained_rational_extension,
                            "an exact candidate must retain the rational-circle extension"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn nonlinear_retained_rational_circle_corner_edits_keep_algebraic_source_cuts() {
    // Compose the unit quarter circle's tan-half-angle parameter with the
    // regular nonlinear bijection t=(s+s^2)/2.  Its homogeneous power
    // basis is (1-t^2, 2t, 1+t^2), so this degree-four carrier is neither
    // an affine/Mobius reparameterization nor a homogeneous degree
    // elevation of the canonical conic.  The point (3/5, 4/5) has t=1/2
    // and native parameter s^2+s-1=0, which must remain an isolated exact
    // parameter until the retained circle sweep is rebuilt.
    let support = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false)
        .expect("the unit quarter circle is valid");
    let (implicit, circular) = crate::arc_bezier::circular_conic_provenance(&support);
    let nonlinear = RationalBezier2::try_new(
        vec![
            p(1, 0),
            Point2::new(Real::one(), q(1, 4)),
            Point2::new(q(23, 25), q(16, 25)),
            Point2::new(q(3, 5), Real::one()),
            p(0, 1),
        ],
        vec![
            Real::one(),
            Real::one(),
            q(25, 24),
            q(5, 4),
            Real::from(2_i8),
        ],
    )
    .map(|curve| curve.with_implicit_quadratic_conic(implicit, Some(circular)))
    .expect("the nonlinear quarter-circle chart is finite");
    let nonlinear = Curve2::from(nonlinear);
    let next_line = Curve2::from(LineSeg2::try_new(p(0, 1), p(0, 0)).unwrap());
    let path = CurvePath2::try_new(vec![
        nonlinear.clone(),
        next_line.clone(),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap()),
    ])
    .expect("the nonlinear quarter-disk boundary closes");
    let arc_setback = (Real::from(10_i8).sqrt().unwrap() / Real::from(5_i8)).unwrap();
    let line_setback = q(1, 5);
    let fillet_radius = q(3, 8);
    let arc_cut = Point2::new(q(3, 5), q(4, 5));
    let line_cut = Point2::new(Real::zero(), q(4, 5));
    let line_fillet_cut = Point2::new(Real::zero(), q(1, 2));

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let previous_carrier =
            crate::curve::exact_corner_carrier(&nonlinear, true, CurveOperation2::Chamfer, &policy)
                .unwrap()
                .unwrap();
        assert!(matches!(
            &previous_carrier,
            crate::curve::ExactCornerCarrier2::RetainedRationalArc(_)
        ));
        let direct = crate::curve::solve_exact_chamfer_corner(
            previous_carrier,
            crate::curve::exact_corner_carrier(
                &next_line,
                false,
                CurveOperation2::Chamfer,
                &policy,
            )
            .unwrap()
            .unwrap(),
            &arc_setback,
            &line_setback,
            RealSign::Positive,
            RealSign::Positive,
            CurveCornerMode2::TrimOnly,
            false,
            false,
            CurveFamily2::RationalBezier,
            CurveFamily2::Line,
            &policy,
        )
        .unwrap_or_else(|error| panic!("the shared carrier solve failed: {error:?}"));
        let CurveCornerSolutions2::Unique(direct) = direct else {
            panic!("the direct retained-circle chamfer solve must be unique");
        };
        let (direct_arc_cut, direct_line_cut) = direct
            .into_retained_cut_evidence()
            .expect("both direct chamfer cuts retain their parameters");
        assert!(matches!(
            direct_arc_cut.parameter.as_bezier_parameter(),
            Some(BezierParameter2::Algebraic(_))
        ));
        assert!(matches!(
            direct_line_cut.parameter.as_bezier_parameter(),
            Some(BezierParameter2::Exact(_))
        ));
        for reversed in [false, true] {
            let source_path = if reversed {
                path.reversed(&policy)
                    .expect("the nonlinear circle boundary reverses")
                    .into_value()
            } else {
                path.clone()
            };
            let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
                std::slice::from_ref(&source_path),
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &policy,
            )
            .expect("the nonlinear retained circle enters CurveRegion2")
            .into_value();
            assert!(
                region.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::Rational(curve),
                            ..
                        } if curve.degree() == 4 && curve.retained_circular_conic().is_some()
                    ))
            );

            let corner = region.boundary_loops()[0]
                .fragments()
                .iter()
                .position(|fragment| {
                    Curve2::from_retained_fragment(fragment.clone())
                        .start()
                        .same_point(&CurvePoint2::from(p(0, 1)), &policy)
                        == Classification::Decided(true)
                })
                .expect("the normalized quarter disk retains its arc-line corner");
            let (previous_setback, next_setback) = (arc_setback.clone(), line_setback.clone());
            let edited = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    corner,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the nonlinear retained-circle chamfer must retain its algebraic source cut: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(edited.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(edited) = edited.value else {
                panic!(
                    "the nonlinear retained-circle corner must have one chamfer: policy={policy:?}, reversed={reversed}"
                );
            };
            let has_endpoint = |edited: &CurveRegion2, expected: &Point2| {
                let expected = CurvePoint2::from(expected.clone());
                edited.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| {
                        [true, false].into_iter().any(|start_endpoint| {
                            matches!(
                                curve_fragment_endpoint_point(fragment, start_endpoint, &policy),
                                Ok(Classification::Decided(Some(point)))
                                    if point.same_point(&expected, &policy)
                                        == Classification::Decided(true)
                            )
                        })
                    })
            };
            assert!(
                has_endpoint(&edited, &arc_cut),
                "the rebuilt circular sweep must end at the algebraic source cut"
            );
            assert!(
                has_endpoint(&edited, &line_cut),
                "the chamfer must meet the exact line setback"
            );
            assert_eq!(
                edited
                    .classify_point_with_policy(&Point2::new(q(1, 10), q(1, 10)).into(), &policy)
                    .expect("the rebuilt nonlinear-circle chamfer remains classifiable")
                    .into_value(),
                Classification::Decided(RegionPointLocation::Inside),
            );

            let filleted = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(fillet_radius.clone()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the nonlinear retained-circle fillet must retain its algebraic source cut: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(filleted.certainty, CurveCertainty::Certified);
            let mut found_expected_fillet = false;
            for_each_corner_region(fillet_regions(&filleted.value), |candidate| {
                found_expected_fillet |=
                    has_endpoint(candidate, &arc_cut) && has_endpoint(candidate, &line_fillet_cut);
                assert_eq!(
                    candidate
                        .classify_point_with_policy(
                            &Point2::new(q(1, 10), q(1, 10)).into(),
                            &policy
                        )
                        .expect("the rebuilt nonlinear-circle fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
            });
            assert!(
                found_expected_fillet,
                "one retained fillet must meet both exact algebraic-source contacts"
            );
        }
    }
}

#[test]
fn collapsed_rational_arc_offset_tests_selected_circle_incidence_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_circle_collapsed_arc_offset_region(&policy, reversed);
            let corner = selected_circle_rational_arc_corner(&region);
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the collapsed arc offset must classify against the selected circle: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
            });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            // The collapsed circle leaves its contact free; the other
            // contact meets the known center only at the excluded corner
            // endpoint, which does not prove that the inserted arc collapses.
            assert_eq!(
                (result.value).no_solution_reason(),
                Some(crate::CurveCornerNoSolution2::OutsideTrimDomain)
            );
        }
    }
}

#[test]
fn collapsed_selected_circle_offset_retains_its_exact_center() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::RationalArc(1),
                reversed,
            );
            let corner = selected_circle_rational_arc_corner(&region);
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the collapsed selected circle must retain its center: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(
                (result.value).no_solution_reason(),
                Some(crate::CurveCornerNoSolution2::NoTangentCircle,)
            );
        }
    }
}

#[test]
fn collapsed_concentric_offsets_compare_retained_and_represented_centers() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::ConcentricRationalArc,
                reversed,
            );
            let corner = selected_circle_rational_arc_corner(&region);
            // Both offsets collapse to the common center, so every contact
            // on the shared circle is admissible: the family needs an
            // exact contact constraint rather than an arbitrary choice.
            let result = region.fillet_loop_vertex_with_policy(
                0,
                corner,
                &crate::CurveFillet2::new(Real::one()),
                CurveCornerMode2::TrimOnly,
                &policy,
            );
            assert!(
                matches!(
                    result,
                    Err(crate::ExactCurveError::Invalid {
                        cause: CurveError::FilletConstraintRequired,
                        ..
                    })
                ),
                "policy={policy:?}, reversed={reversed}, result={result:?}"
            );
        }
    }
}

#[test]
fn collapsed_selected_circle_center_classifies_every_neighbor_carrier() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (name, neighbor) in [
            (
                "selected-circle",
                SelectedCircleFilletNeighbor2::SelectedCircle,
            ),
            (
                "retained-parallel",
                SelectedCircleFilletNeighbor2::AnalyticParallel(true),
            ),
            ("line", SelectedCircleFilletNeighbor2::DirectLine),
            ("direct-bezier", SelectedCircleFilletNeighbor2::DirectBezier),
        ] {
            for reversed in [false, true] {
                let region = selected_circle_neighbor_region(&policy, neighbor, reversed);
                let corner = if reversed { 2 } else { 1 };
                let result = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(Real::one()),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the collapsed selected-circle center must classify against {name}: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(
                    result.certainty,
                    CurveCertainty::Certified,
                    "policy={policy:?}, reversed={reversed}, neighbor={name}"
                );
                assert_eq!(
                    (result.value).no_solution_reason(),
                    Some(crate::CurveCornerNoSolution2::NoTangentCircle,),
                    "policy={policy:?}, reversed={reversed}, neighbor={name}"
                );
            }
        }
    }
}

#[test]
fn collapsed_general_selected_frames_retain_exact_centers() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let chord_region = selected_circle_direct_line_region_from_support(
                selected_chord_normal_circle(&policy),
                &policy,
                reversed,
            );
            let parallel_region = selected_circle_direct_line_region_from_support(
                selected_parallel_normal_circle(&policy),
                &policy,
                reversed,
            );
            let transform = crate::Similarity2::try_from_real_affine(
                Real::zero(),
                Real::from(-2_i8),
                Real::from(2_i8),
                Real::zero(),
                Real::from(5_i8),
                Real::from(-7_i8),
            )
            .expect("the scaled quarter turn is a similarity");
            let transformed = chord_region
                .transform_similarity_with_policy(&transform, &policy)
                .expect("the chord-normal fixture transforms exactly");
            assert_eq!(transformed.certainty, CurveCertainty::Certified);

            for (name, region) in [
                ("chord-normal", chord_region),
                ("parallel-normal", parallel_region),
                ("similarity", transformed.value),
            ] {
                let circle = region.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .find_map(|fragment| match fragment {
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                            Some(fragment.semicircle())
                        }
                        _ => None,
                    })
                    .expect("the selected-frame fixture retains its circle");
                match name {
                    "chord-normal" => {
                        assert!(circle.uses_selected_chord_normal_frame());
                        assert!(matches!(
                            circle.center_point_evidence(&policy).unwrap(),
                            Classification::Decided(CurvePoint2(CurvePointData2::Algebraic(_)))
                        ));
                    }
                    "parallel-normal" => {
                        assert!(circle.uses_selected_parallel_normal_frame());
                        assert!(matches!(
                            circle.center_point_evidence(&policy).unwrap(),
                            Classification::Decided(CurvePoint2(
                                CurvePointData2::AnalyticParallel(_)
                            ))
                        ));
                    }
                    "similarity" => {
                        assert!(circle.uses_selected_chord_normal_frame());
                        assert!(matches!(
                            circle.center_point_evidence(&policy).unwrap(),
                            Classification::Decided(CurvePoint2(CurvePointData2::Similarity(_)))
                        ));
                    }
                    _ => unreachable!(),
                }
                let result = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        if reversed { 2 } else { 1 },
                        &crate::CurveFillet2::new(circle.radial_distance().abs()),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the collapsed {name} center must remain exact: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                assert_eq!(
                    (result.value).no_solution_reason(),
                    Some(crate::CurveCornerNoSolution2::NoTangentCircle)
                );
            }
        }
    }
}

#[test]
fn collapsed_chord_pair_center_classifies_an_analytic_parallel() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = correlated_chord_pair_collapse_region(&policy, reversed);
            let fragments = region.boundary_loops()[0].fragments();
            let corner = (0..fragments.len())
                .find(|index| {
                    matches!(
                        (
                            &fragments[(index + fragments.len() - 1) % fragments.len()],
                            &fragments[*index],
                        ),
                        (
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                            BezierSplitFragment2::AnalyticParallel(_),
                        ) | (
                            BezierSplitFragment2::AnalyticParallel(_),
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                        )
                    )
                })
                .expect("the retained circle/parallel corner is present");
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the collapsed chord-pair center must classify against its analytic parallel: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            // The collapsed circle leaves its contact free; the other
            // contact meets the known center only at the excluded corner
            // endpoint, which does not prove that the inserted arc collapses.
            assert_eq!(
                (result.value).no_solution_reason(),
                Some(crate::CurveCornerNoSolution2::OutsideTrimDomain)
            );
        }
    }
}

#[test]
fn selected_parallel_normal_circle_and_line_fillet_retains_recursive_contact() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let three_halves = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
    let upper = Point2::new(Real::zero(), half.clone());
    let lower = Point2::new(Real::zero(), -half.clone());

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let center_support = QuadraticBezier2::from_line_segment(
            crate::LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap(),
        )
        .parallel_left(Real::zero())
        .unwrap();
        let Classification::Decided(Some(circle)) =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                center_support,
                sqrt_half_algebraic_parameter(&policy).into(),
                three_halves.clone(),
                false,
                &policy,
            )
            .unwrap()
        else {
            panic!("the selected source circle must construct");
        };
        let vertical =
            RationalBezier2::try_new(vec![p(0, -1), p(0, 1)], vec![Real::one(), Real::one()])
                .unwrap();
        let Classification::Decided((intersections, parameter_map)) = circle
            .rational_intersections_with_parameter_map(
                &vertical,
                &crate::CurveParameterRange2::unit(),
                &policy,
            )
            .unwrap()
        else {
            panic!("the source-circle endpoint cuts must be exact");
        };
        let contacts = match intersections {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps: unexpected_overlaps } if unexpected_overlaps.is_empty() => {
                let parameter_map = parameter_map
                    .as_ref()
                    .expect("ordinary interior contacts retain their circle map");
                contacts
                    .iter()
                    .map(|contact| {
                        (
                            parameter_map.contact_parameter(contact),
                            contact.point.clone(),
                        )
                    })
                    .collect::<Vec<_>>()
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { contacts, overlaps: unexpected_overlaps } if unexpected_overlaps.is_empty() => contacts
                .iter()
                .map(|contact| (contact.cusp_parameter(), contact.point_evidence()))
                .collect::<Vec<_>>(),
            other => panic!("the transverse source line must have isolated contacts: {other:?}"),
        };
        let parameter_at = |point: &Point2| {
            contacts
                .iter()
                .find(|(_, evidence)| {
                    evidence.same_point(&CurvePoint2::from(point.clone()), &policy)
                        == Classification::Decided(true)
                })
                .map(|(parameter, _)| parameter.clone())
                .expect("the represented source-circle endpoint must be retained")
        };
        let upper_parameter = parameter_at(&upper);
        let lower_parameter = parameter_at(&lower);
        let order = match upper_parameter
            .cmp_by_refinement(&lower_parameter, &policy)
            .unwrap()
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                panic!("the source-circle endpoint order must decide: {reason:?}")
            }
        };
        let (start, end, reversed) = if order.is_lt() {
            (upper_parameter, lower_parameter, false)
        } else {
            (lower_parameter, upper_parameter, true)
        };
        let Classification::Decided(circle_fragment) =
            crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle, start, end, reversed, &policy,
            )
            .unwrap()
        else {
            panic!("the selected source-circle interval must construct");
        };
        let right_lower = Point2::new(Real::from(4_i8), -half.clone());
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicCuspSemicircle(circle_fragment),
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                        crate::LineSeg2::try_new(lower.clone(), right_lower.clone()).unwrap(),
                    )),
                },
                quadratic_fragment(right_lower, p(2, 0), upper.clone()),
            ],
            &policy,
        )
        .expect("the selected-circle/line fixture closes exactly");
        let region = CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let fillet_work = || {
            region.fillet_loop_vertex_with_policy(
                0,
                1,
                &crate::CurveFillet2::new(half.clone()),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let result = hyperreal::dispatch_trace::with_recording(fillet_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let result = fillet_work();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        let result = result
            .unwrap_or_else(|error| {
                panic!(
                    "the selected-center circle/line contact must fillet: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(result.certainty, CurveCertainty::Certified);
        let mut retained_recursive_cut = false;
        for_each_corner_region(fillet_regions(&result.value), |filleted| {
            retained_recursive_cut |=
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| {
                        let BezierSplitFragment2::SelectedFiber(fragment) = fragment else {
                            return false;
                        };
                        fragment.range().start().as_recursive_projective().is_some()
                            || fragment.range().end().as_recursive_projective().is_some()
                    });
        });
        assert!(retained_recursive_cut);
        #[cfg(feature = "dispatch-trace")]
        {
            assert!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "selected-parallel-normal-recursive-line",
                ) > 0,
                "an authored exact line must keep the compact recursive quadratic kernel: {trace:?}",
            );
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "recursive-projective-retained-chord",
                ),
                0,
                "the general recursive bridge must not preempt the direct selected-center line solve: {trace:?}",
            );
        }
    }
}

#[test]
fn selected_circle_and_retained_rational_arc_extend_on_full_supports() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for homogeneous_scale in [1_i8, 2_i8] {
            for elevated in [false, true] {
                for reversed in [false, true] {
                    let neighbor = if elevated {
                        SelectedCircleFilletNeighbor2::ElevatedRationalArc(homogeneous_scale)
                    } else {
                        SelectedCircleFilletNeighbor2::RationalArc(homogeneous_scale)
                    };
                    let region = selected_circle_neighbor_region(&policy, neighbor, reversed);
                    let corner = selected_circle_rational_arc_corner(&region);
                    let trim = region
                        .fillet_loop_vertex_with_policy(
                            0,
                            corner,
                            &crate::CurveFillet2::new(q(1, 10)),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .expect("the finite mixed circular corner remains supported");
                    let extended = region
                        .fillet_loop_vertex_with_policy(
                            0,
                            corner,
                            &crate::CurveFillet2::new(q(1, 10)),
                            CurveCornerMode2::TrimOrExtend,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "the mixed circular supports must extend exactly: policy={policy:?}, scale={homogeneous_scale}, elevated={elevated}, reversed={reversed}, error={error:?}"
                            )
                        });
                    assert_eq!(extended.certainty, CurveCertainty::Certified);
                    assert!(
                        extended.value.solutions().len() > trim.value.solutions().len(),
                        "both full circular supports must contribute exterior centers"
                    );
                    for_each_corner_region(fillet_regions(&extended.value), |filleted| {
                        assert!(
                            filleted.boundary_loops()[0].fragments().iter().any(
                                |fragment| matches!(
                                    fragment,
                                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                                )
                            ),
                            "the inserted fillet remains an exact selected circle"
                        );
                        assert_eq!(
                            filleted
                                .classify_point_with_policy(&p(0, 0).into(), &policy)
                                .expect("the extended circular fillet remains classifiable")
                                .into_value(),
                            Classification::Decided(RegionPointLocation::Inside),
                        );
                        if homogeneous_scale == 1 && !elevated && !reversed {
                            let replay = filleted
                                .boolean_regions_with_policy(
                                    &selected_fillet_disjoint_square(&policy),
                                    &policy,
                                )
                                .expect(
                                    "the extended circular fillet re-enters the Boolean kernel",
                                );
                            assert_eq!(replay.certainty, CurveCertainty::Certified);
                            assert_disjoint_square_replay_preserves_set(
                                filleted,
                                &replay.value,
                                &policy,
                            );
                            assert!(replay.value.intersection().is_empty());
                        }
                    });
                }
            }
        }
    }
}

#[test]
fn selected_circle_and_promoted_line_extend_through_the_chord_support_cell() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let promoted_line = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::DirectLine,
                reversed,
            );
            let retained_line = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::AnalyticParallel(false),
                reversed,
            );
            let corner = if reversed { 2 } else { 1 };
            let solve = |region: &CurveRegion2, mode| {
                region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(q(1, 10)),
                        mode,
                        &policy,
                    )
                    .expect("the selected-circle/line support must extend exactly")
            };
            let promoted_extension = solve(&promoted_line, CurveCornerMode2::TrimOrExtend);
            let retained_extension = solve(&retained_line, CurveCornerMode2::TrimOrExtend);
            assert_eq!(promoted_extension.certainty, CurveCertainty::Certified);
            assert_eq!(retained_extension.certainty, CurveCertainty::Certified);
            assert_eq!(
                promoted_extension.value.solutions().len(),
                retained_extension.value.solutions().len(),
                "the represented fast path must enumerate both circle charts and both affine rays",
            );
            assert!(promoted_extension.value.solutions().len() > 1);
            for_each_corner_region(fillet_regions(&promoted_extension.value), |filleted| {
                assert!(
                    filleted.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .any(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                );
            });
        }
    }
}

#[test]
fn selected_circle_mixed_fillet_crosses_one_sided_smooth_run_seam() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (name, neighbor) in [
            ("line", SelectedCircleFilletNeighbor2::DirectLine),
            ("arc", SelectedCircleFilletNeighbor2::RationalArc(1)),
            (
                "analytic-parallel",
                SelectedCircleFilletNeighbor2::AnalyticParallel(true),
            ),
            ("bezier", SelectedCircleFilletNeighbor2::DirectBezier),
        ] {
            for reversed in [false, true] {
                let region = split_selected_circle_neighbor_region(
                    &policy,
                    neighbor,
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(q(
                        99, 100,
                    )),
                    reversed,
                );
                let source_fragments = region.boundary_loops()[0].fragments();
                let source_circle_fragments = source_fragments
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                    })
                    .collect::<Vec<_>>();
                assert_eq!(source_circle_fragments.len(), 2);
                let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    2,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                            "the selected-circle/{name} fillet must cross its authored smooth seam: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let consumes_seam = |filleted: &CurveRegion2| {
                    !source_circle_fragments.iter().any(|source| {
                        filleted.boundary_loops()[0]
                            .fragments()
                            .iter()
                            .any(|fragment| fragment == *source)
                    })
                };
                assert!(
                    { result.value.solutions().iter().any(consumes_seam) },
                    "at least one exact mixed-family candidate must consume the selected-circle seam"
                );
                for_each_corner_region(fillet_regions(&result.value), |filleted| {
                    assert_eq!(
                        filleted
                            .classify_point_with_policy(&p(0, 0).into(), &policy)
                            .expect("the one-sided smooth-run fillet remains classifiable")
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                    );
                });
            }
        }
    }
}

#[test]
fn selected_circle_chamfer_crosses_one_sided_smooth_run_seam() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = selected_circle_neighbor_region(
            &policy,
            SelectedCircleFilletNeighbor2::DirectLine,
            false,
        );
        let split =
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(q(99, 100));
        for independently_reframed in [false, true] {
            for reversed in [false, true] {
                let region = if independently_reframed {
                    independently_reframed_selected_circle_region(
                        &policy,
                        &source,
                        split.clone(),
                        reversed,
                    )
                } else {
                    split_selected_circle_region(&policy, &source, split.clone(), reversed)
                };
                let source_circle_fragments = region.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                    })
                    .collect::<Vec<_>>();
                assert_eq!(source_circle_fragments.len(), 2);
                let (previous_setback, next_setback) = if reversed {
                    (q(1, 10), q(1, 2))
                } else {
                    (q(1, 2), q(1, 10))
                };
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let result = region
                        .chamfer_loop_vertex_by_setbacks_with_policy(
                            0,
                            2,
                            previous_setback.clone(),
                            next_setback.clone(),
                            mode,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "the selected-circle chamfer must cross its authored smooth seam: policy={policy:?}, independently_reframed={independently_reframed}, reversed={reversed}, mode={mode:?}, error={error:?}"
                            )
                        });
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let retained_count = |chamfered: &CurveRegion2| {
                        source_circle_fragments
                            .iter()
                            .filter(|source| {
                                chamfered.boundary_loops()[0]
                                    .fragments()
                                    .iter()
                                    .any(|fragment| fragment == **source)
                            })
                            .count()
                    };
                    assert!(
                        match &result.value {
                            CurveCornerSolutions2::Unique(chamfered) => {
                                retained_count(chamfered) == 0
                            }
                            CurveCornerSolutions2::Multiple(chamfered) => chamfered
                                .iter()
                                .any(|candidate| retained_count(candidate) == 0),
                            CurveCornerSolutions2::NoSolution(_) => false,
                        },
                        "the exact chamfer must consume the selected-circle seam"
                    );
                    for_each_corner_region(corner_regions(&result.value), |chamfered| {
                        assert_eq!(
                            chamfered
                                .classify_point_with_policy(&p(0, 0).into(), &policy)
                                .expect("the smooth-run chamfer remains classifiable")
                                .into_value(),
                            Classification::Decided(RegionPointLocation::Inside),
                            "policy={policy:?}, independently_reframed={independently_reframed}, reversed={reversed}, mode={mode:?}",
                        );
                    });
                }
            }
        }
    }
}

#[test]
fn selected_circle_fillet_owns_exact_smooth_run_seam_endpoint() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = selected_circle_neighbor_region(
            &policy,
            SelectedCircleFilletNeighbor2::DirectLine,
            false,
        );
        let BezierSplitFragment2::AlgebraicCuspSemicircle(source_circle) =
            &source.boundary_loops()[0].fragments()[0]
        else {
            panic!("the endpoint fixture starts on its selected circle")
        };
        let split = selected_circle_direct_line_fillet_cut(&source, &policy);
        assert_eq!(
            source_circle
                .contains_parameter(&split, false, false, &policy)
                .expect("the baseline cut compares to its source range"),
            Classification::Decided(true),
        );

        for reversed in [false, true] {
            let region = split_selected_circle_region(&policy, &source, split.clone(), reversed);
            let source_circle_fragments = region.boundary_loops()[0]
                .fragments()
                .iter()
                .filter(|fragment| {
                    matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                })
                // Compare retained support/range evidence independently of
                // the orientation chosen for the normalized boundary.
                .map(|fragment| {
                    [
                        fragment.clone(),
                        fragment.reversed().expect("the exact range reverses"),
                    ]
                })
                .collect::<Vec<_>>();
            assert_eq!(source_circle_fragments.len(), 2);
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    2,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the exact selected-circle seam endpoint must remain admissible: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            // The recursive selected-field authority now proves this seam
            // exactly under either policy; APPROXIMATE_512 need not consume
            // its terminal equality allowance when an exact proof succeeds.
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let owns_seam_endpoint = |filleted: &CurveRegion2| {
                source_circle_fragments
                    .iter()
                    .filter(|source| {
                        filleted
                            .boundary_loops()
                            .iter()
                            .flat_map(|boundary| boundary.fragments())
                            .any(|fragment| source.contains(fragment))
                    })
                    .count()
                    == 1
            };
            assert!(
                { result.value.solutions().iter().any(owns_seam_endpoint) },
                "one retained-side fragment must own the exact seam endpoint"
            );
            for_each_corner_region(fillet_regions(&result.value), |filleted| {
                assert_eq!(
                    filleted
                        .classify_point_with_policy(&p(0, 0).into(), &policy)
                        .expect("the seam-endpoint fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
            });
        }
    }
}

#[test]
fn selected_circle_fillet_crosses_an_independently_reframed_run() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = selected_circle_neighbor_region(
            &policy,
            SelectedCircleFilletNeighbor2::DirectLine,
            false,
        );
        let seam_split = selected_circle_direct_line_fillet_cut(&source, &policy);
        for (cut_location, split) in [
            ("seam", seam_split),
            (
                "reframed interior",
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(q(1, 2)),
            ),
        ] {
            for reversed in [false, true] {
                let region = independently_reframed_selected_circle_region(
                    &policy,
                    &source,
                    split.clone(),
                    reversed,
                );
                let source_circle_fragments = region.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                    })
                    // Compare retained support/range evidence independently of
                    // the orientation chosen for the normalized boundary.
                    .map(|fragment| {
                        [
                            fragment.clone(),
                            fragment.reversed().expect("the exact range reverses"),
                        ]
                    })
                    .collect::<Vec<_>>();
                assert_eq!(source_circle_fragments.len(), 2);
                let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    2,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the independently reframed circle run must fillet: policy={policy:?}, cut_location={cut_location}, reversed={reversed}, error={error:?}"
                    )
                });
                if policy == CurveContext::STRICT {
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                }
                let owns_one_run_fragment = |filleted: &CurveRegion2| {
                    source_circle_fragments
                        .iter()
                        .filter(|source| {
                            filleted
                                .boundary_loops()
                                .iter()
                                .flat_map(|boundary| boundary.fragments())
                                .any(|fragment| source.contains(fragment))
                        })
                        .count()
                        == 1
                };
                assert!(
                    { result.value.solutions().iter().any(owns_one_run_fragment) },
                    "one independently framed run fragment must own the exact {cut_location} cut"
                );
                for_each_corner_region(fillet_regions(&result.value), |filleted| {
                    assert_eq!(
                        filleted
                            .classify_point_with_policy(&p(0, 0).into(), &policy)
                            .expect("the independently reframed fillet remains classifiable")
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                    );
                });
            }
        }
    }
}

#[test]
fn selected_circle_pair_with_rationalizable_support_fillet_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::SelectedCircle,
                reversed,
            );
            let corner = selected_circle_pair_corner(&region);
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new((Real::one() / Real::from(10_i8)).unwrap()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the selected-circle pair must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let filleted = {
                let solutions = result.value;
                let mut candidates = solutions.into_solutions();
                assert_eq!(candidates.len(), 1, "expected one isolated fillet");
                candidates.pop().unwrap()
            };
            assert_eq!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    ))
                    .count(),
                3,
                "policy={policy:?}, reversed={reversed}",
            );
            assert_eq!(
                filleted
                    .classify_point_with_policy(&p(0, 0).into(), &policy)
                    .expect("the selected-circle pair fillet remains classifiable")
                    .into_value(),
                Classification::Decided(RegionPointLocation::Inside),
            );
            if !reversed {
                let replay = filleted
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the selected-circle pair fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(replay.value.union().boundary_loops().len(), 2);
                assert!(replay.value.intersection().is_empty());
            }
        }
    }
}

#[test]
fn selected_circle_pair_fillets_extend_over_both_full_supports() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::SelectedCircle,
                reversed,
            );
            let corner = selected_circle_pair_corner(&region);
            let trim = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the finite selected-circle pair remains supported");
            let extended = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the selected-circle pair must extend exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(
                extended.value.solutions().len() > trim.value.solutions().len(),
                "full circular supports must retain an exterior center"
            );
            for_each_corner_region(fillet_regions(&extended.value), |filleted| {
                assert!(filleted.has_regularized_filled_left_topology(&policy));
                assert!(
                    filleted
                        .boundary_loops()
                        .iter()
                        .flat_map(|boundary| boundary.fragments())
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                        .count()
                        >= 3,
                    "both extended sources and the fillet remain exact selected circles"
                );
            });
        }
    }
}

fn independent_selected_circle_pair_region(policy: &CurveContext, reversed: bool) -> CurveRegion2 {
    let first_parameter = sqrt_half_algebraic_parameter(policy);
    let second_parameter = sqrt_third_algebraic_parameter(policy);
    let BezierParameter2::Algebraic(first_parameter) = first_parameter else {
        panic!("sqrt(1/2) must remain algebraic");
    };
    let BezierParameter2::Algebraic(second_parameter) = second_parameter else {
        panic!("sqrt(1/3) must remain algebraic");
    };
    let first_source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0)], vec![Real::one(), Real::one()])
            .expect("the first selected center source is valid");
    let second_source =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 1)], vec![Real::one(), Real::one()])
            .expect("the second selected center source is valid");
    let first_center = CurvePoint2::from(crate::tests::decided(
        first_source
            .point_at_algebraic_parameter(&first_parameter, policy)
            .expect("the first selected center image is exact"),
    ));
    let second_center = CurvePoint2::from(crate::tests::decided(
        second_source
            .point_at_algebraic_parameter(&second_parameter, policy)
            .expect("the second selected center image is exact"),
    ));
    let Classification::Decided(Some(first_circle)) =
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
            &first_center,
            (1, 0),
            Real::one(),
            false,
            policy,
        )
        .expect("the first selected circle is valid")
    else {
        panic!("the first nonzero selected circle must be decided");
    };
    let Classification::Decided(Some(second_circle)) =
        crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
            &second_center,
            (0, -1),
            Real::one(),
            false,
            policy,
        )
        .expect("the second selected circle is valid")
    else {
        panic!("the second nonzero selected circle must be decided");
    };
    assert!(
        first_circle
            .center_point_image(policy)
            .expect("the first center image remains exact")
            .exact_point(&CurveContext::STRICT)
            .is_none(),
        "the first support center must retain its selected field"
    );
    assert!(
        second_circle
            .center_point_image(policy)
            .expect("the second center image remains exact")
            .exact_point(&CurveContext::STRICT)
            .is_none(),
        "the second support center must retain its independent selected field"
    );
    let intersections = match first_circle
        .pair_intersections(&second_circle, policy)
        .expect("the independent selected circles have an exact pair relation")
    {
        Classification::Decided(intersections) => intersections,
        Classification::Uncertain(reason) => {
            panic!("the independent selected-circle contact must be decided: {reason:?}")
        }
    };
    let crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
        mut contacts,
        parameter_map,
    } = intersections
    else {
        panic!("the independent selected semicircles must meet transversely");
    };
    assert_eq!(contacts.len(), 1, "the selected halves retain one contact");
    let contact = contacts.pop().expect("one pair contact was certified");
    let first_contact = parameter_map.first_contact_parameter(&contact);
    let second_contact = parameter_map.second_contact_parameter(&contact);
    let Classification::Decided(first_fragment) =
        crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
            first_circle.clone(),
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
            first_contact,
            false,
            policy,
        )
        .expect("the first selected contact range is valid")
    else {
        panic!("the first selected contact range must be decided");
    };
    let Classification::Decided(second_fragment) =
        crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
            second_circle.clone(),
            second_contact,
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
            false,
            policy,
        )
        .expect("the second selected contact range is valid")
    else {
        panic!("the second selected contact range must be decided");
    };
    let Classification::Decided(first_endpoint) = first_circle
        .start_point_evidence(policy)
        .expect("the first selected endpoint is exact")
    else {
        panic!("the first selected endpoint must be decided");
    };
    let Classification::Decided(second_endpoint) = second_circle
        .end_point_evidence(policy)
        .expect("the second selected endpoint is exact")
    else {
        panic!("the second selected endpoint must be decided");
    };
    let Classification::Decided(closing_chord) =
        crate::BezierAlgebraicChord2::try_new(second_endpoint, first_endpoint, policy)
            .expect("the independent selected endpoints define a chord")
    else {
        panic!("the independent selected endpoint chord must be decided");
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicCuspSemicircle(first_fragment),
        BezierSplitFragment2::AlgebraicCuspSemicircle(second_fragment),
        BezierSplitFragment2::AlgebraicChord(closing_chord),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the exact fixture reverses"))
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the independent selected-circle loop closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the independent selected-circle loop has authored topology")
}

#[test]
fn independent_selected_circle_pair_fillets_extend_over_both_full_supports() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = independent_selected_circle_pair_region(&policy, reversed);
            let corner = selected_circle_pair_corner(&region);
            let trim = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the finite independent selected-circle pair remains supported");
            let extended = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(q(1, 10)),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the independent selected-circle pair must extend exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(
                extended.value.solutions().len() > trim.value.solutions().len(),
                "both full selected supports must contribute exterior centers"
            );
            for_each_corner_region(fillet_regions(&extended.value), |filleted| {
                assert!(filleted.has_regularized_filled_left_topology(&policy));
                let selected_circles = filleted
                    .boundary_loops()
                    .iter()
                    .flat_map(|boundary| boundary.fragments())
                    .filter_map(|fragment| match fragment {
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                            Some(fragment.semicircle())
                        }
                        _ => None,
                    })
                    .collect::<Vec<_>>();
                assert!(
                    selected_circles.len() >= 3,
                    "both extended sources and the fillet remain exact selected circles"
                );
                assert!(
                    selected_circles
                        .iter()
                        .any(|circle| circle.uses_selected_radial_frame()),
                    "the independent pair correlation remains the fillet radial authority"
                );
            });
        }
    }
}

fn independent_pair_native_fillet(policy: &CurveContext, reversed: bool) -> CurveRegion2 {
    let region = independent_selected_circle_pair_region(policy, reversed);
    let corner = selected_circle_pair_corner(&region);
    let result = region
        .fillet_loop_vertex_with_policy(
            0,
            corner,
            &crate::CurveFillet2::new(q(1, 10)),
            CurveCornerMode2::TrimOnly,
            policy,
        )
        .unwrap_or_else(|error| {
            panic!(
                "the independent selected-circle pair must fillet: policy={policy:?}, reversed={reversed}, error={error:?}"
            )
        });
    assert_eq!(result.certainty, CurveCertainty::Certified);
    {
        let solutions = result.value;
        let mut candidates = solutions.into_solutions();
        assert_eq!(candidates.len(), 1, "expected one isolated fillet");
        candidates.pop().unwrap()
    }
}

#[test]
fn pair_native_offset_join_tangents_remain_exact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = independent_pair_native_fillet(&policy, reversed);
            let distance = (pair_radial_corner(&region).1 / Real::from(200_i16)).unwrap();
            let signed_left_distance = if reversed { -distance } else { distance };
            let spans = match exact_offset_span_runs_from_boundary_loop(
                &region.boundary_loops()[0],
                &signed_left_distance,
                &policy,
            )
            .expect("the pair-native offset spans are valid")
            {
                Classification::Decided(spans) => {
                    spans.into_iter().map(|(span, _)| span).collect::<Vec<_>>()
                }
                Classification::Uncertain(reason) => panic!(
                    "the pair-native offset spans must decide: policy={policy:?}, reversed={reversed}, reason={reason:?}"
                ),
            };
            let mut selected_circle_pairs = 0;
            for span_index in 0..spans.len() {
                let next_index = (span_index + 1) % spans.len();
                let Some((previous, next)) = spans[span_index]
                    .end_tangent
                    .as_ref()
                    .zip(spans[next_index].start_tangent.as_ref())
                else {
                    continue;
                };
                let selected_circle_pair = matches!(
                    (previous, next),
                    (
                        CurveTangent2::SelectedCircularEndpoint { .. },
                        CurveTangent2::SelectedCircularEndpoint { .. }
                    )
                );
                if selected_circle_pair {
                    selected_circle_pairs += 1;
                }
                let cross = curve_tangent_cross_sign(previous, next, &policy);
                let Classification::Decided(cross) = cross else {
                    panic!(
                        "every pair-native offset join cross sign must decide: policy={policy:?}, reversed={reversed}, span={span_index}, selected_circle_pair={selected_circle_pair}, result={cross:?}"
                    );
                };
                if cross == RealSign::Zero {
                    let opposite = curve_tangents_are_opposite(previous, next, &policy);
                    assert!(
                        matches!(opposite, Classification::Decided(_)),
                        "every smooth pair-native offset join dot sign must decide: policy={policy:?}, reversed={reversed}, span={span_index}, selected_circle_pair={selected_circle_pair}, result={opposite:?}"
                    );
                }
            }
            assert!(
                selected_circle_pairs > 0,
                "the recursive fillet must exercise a selected-circle pair join"
            );
        }
    }
}

fn pair_radial_corner(region: &CurveRegion2) -> (usize, Real) {
    let fragments = region.boundary_loops()[0].fragments();
    (0..fragments.len())
        .find_map(|index| {
            let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
            let next = &fragments[index];
            let radial = |fragment: &BezierSplitFragment2| match fragment {
                BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                    if fragment.semicircle().uses_selected_radial_frame() =>
                {
                    Some(fragment.semicircle().radial_distance().abs())
                }
                _ => None,
            };
            radial(previous)
                .or_else(|| radial(next))
                .map(|radius| (index, radius))
        })
        .expect("the first fillet retains one pair-radial circle")
}

fn pair_radial_crossing_corner(region: &CurveRegion2, policy: &CurveContext) -> (usize, usize) {
    for (loop_index, boundary) in region.boundary_loops().iter().enumerate() {
        let fragments = boundary.fragments();
        for corner in 0..fragments.len() {
            let (
                BezierSplitFragment2::AlgebraicCuspSemicircle(previous),
                BezierSplitFragment2::AlgebraicCuspSemicircle(next),
            ) = (
                &fragments[(corner + fragments.len() - 1) % fragments.len()],
                &fragments[corner],
            )
            else {
                continue;
            };
            if !previous.semicircle().uses_selected_radial_frame()
                || !next.semicircle().uses_selected_radial_frame()
            {
                continue;
            }
            let crossing = match previous
                .semicircle()
                .pair_intersections(next.semicircle(), policy)
                .expect("adjacent pair-native supports compare exactly")
            {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                        contacts,
                        ..
                    },
                )
                | Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts),
                ) => contacts
                    .iter()
                    .any(|contact| contact.tangent_cross_sign != RealSign::Zero),
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts
                    | crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(_),
                )
                | Classification::Uncertain(_) => false,
            };
            if crossing {
                return (loop_index, corner);
            }
        }
    }
    panic!("the lens retains one pair-native/pair-native corner");
}

#[test]
fn collapsed_pair_radial_fillet_rejects_the_excluded_contact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let filleted = independent_pair_native_fillet(&policy, reversed);
            let (corner, radius) = pair_radial_corner(&filleted);
            let result = filleted
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(radius),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the collapsed pair-radial fillet must retain its recursive center: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            // The collapsed source leaves a free contact, but the other
            // offset meets this center only outside its strict trim domain.
            // That exclusion does not prove that the inserted arc collapses.
            assert_eq!(
                (result.value).no_solution_reason(),
                Some(crate::CurveCornerNoSolution2::OutsideTrimDomain,)
            );
        }
    }
}

#[test]
fn noncollapsed_pair_radial_fillet_reenters_the_corner_kernel() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let filleted = independent_pair_native_fillet(&policy, reversed);
            let transform = crate::Similarity2::try_from_real_affine(
                Real::zero(),
                Real::from(-2_i8),
                Real::from(2_i8),
                Real::zero(),
                Real::from(5_i8),
                Real::from(-7_i8),
            )
            .expect("the scaled quarter turn is a similarity");
            let reflected = crate::Similarity2::try_from_real_affine(
                Real::from(-3_i8),
                Real::zero(),
                Real::zero(),
                Real::from(3_i8),
                Real::from(2_i8),
                Real::from(3_i8),
            )
            .expect("the scaled reflection is a similarity");
            let transformed = filleted
                .transform_similarity_with_policy(&transform, &policy)
                .expect("the pair-radial fixture transforms exactly");
            assert_eq!(transformed.certainty, CurveCertainty::Certified);
            let reflected = filleted
                .transform_similarity_with_policy(&reflected, &policy)
                .expect("the pair-radial fixture reflects exactly");
            assert_eq!(reflected.certainty, CurveCertainty::Certified);

            for (name, region) in [
                ("direct", filleted),
                ("rotated", transformed.value),
                ("reflected", reflected.value),
            ] {
                let (corner, parent_radius) = pair_radial_corner(&region);
                let radius = (parent_radius / Real::from(2_i8)).unwrap();
                let result = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(radius),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the noncollapsed {name} pair-radial fillet must remain in the exact kernel: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(
                    result.certainty,
                    CurveCertainty::Certified,
                    "the noncollapsed {name} result must not consume the approximate terminal: policy={policy:?}, reversed={reversed}"
                );
                assert_eq!(
                    (result.value).no_solution_reason(),
                    Some(crate::CurveCornerNoSolution2::OutsideTrimDomain,),
                    "the noncollapsed {name} contact lies outside its trim domain: policy={policy:?}, reversed={reversed}"
                );
            }
        }
    }
}

#[test]
fn independent_selected_circle_pair_fillet_retains_pair_native_circle() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let filleted = independent_pair_native_fillet(&policy, reversed);
            let selected_radial = filleted.boundary_loops()[0]
                .fragments()
                .iter()
                .find_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                        if fragment.semicircle().uses_selected_radial_frame() =>
                    {
                        Some(fragment)
                    }
                    _ => None,
                })
                .expect("the fillet retains its pair-radial carrier");
            let center = match selected_radial
                .semicircle()
                .center_point_evidence(&policy)
                .expect("the pair-radial center evidence is exact")
            {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    panic!("the pair-radial center must be decided: {reason:?}")
                }
            };
            let center_bounds = match crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                &center, 8, &policy,
            ) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    panic!("the pair-radial center must refine: {reason:?}")
                }
            };
            let two = Real::from(2_i8);
            let y = ((center_bounds.min().y() + center_bounds.max().y()) / &two)
                .expect("the center bracket midpoint is rational");
            let margin = selected_radial.semicircle().radial_distance().abs() * &two + Real::one();
            let line = RationalBezier2::try_new(
                vec![
                    Point2::new(center_bounds.min().x() - &margin, y.clone()),
                    Point2::new(center_bounds.max().x() + &margin, y),
                ],
                vec![Real::one(), Real::one()],
            )
            .expect("the pair-radial probe is a finite rational line");
            let (contacts, parameter_map) = match selected_radial
                .semicircle()
                .rational_intersections_with_parameter_map(&line, &crate::CurveParameterRange2::unit(), &policy)
                .expect("the pair-radial/rational kernel is exact")
            {
                Classification::Decided((
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps: unexpected_overlaps },
                    parameter_map,
                )) if unexpected_overlaps.is_empty() => (contacts, parameter_map),
                Classification::Decided((intersections, _)) => {
                    panic!("the finite probe must produce contacts, got {intersections:?}")
                }
                Classification::Uncertain(reason) => {
                    panic!("the pair-radial/rational kernel must decide: {reason:?}")
                }
            };
            assert!(
                !contacts.is_empty(),
                "a center-straddling line meets the selected half circle"
            );
            assert!(
                parameter_map.is_some(),
                "an interior pair-radial contact retains one shared parameter map"
            );
            assert!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                            if fragment.semicircle().uses_selected_radial_frame()
                    ))
            );
            for transform in [
                crate::Similarity2::try_from_real_affine(
                    Real::zero(),
                    Real::from(-2_i8),
                    Real::from(2_i8),
                    Real::zero(),
                    Real::from(5_i8),
                    Real::from(-7_i8),
                )
                .expect("the scaled quarter turn is a similarity"),
                crate::Similarity2::try_from_real_affine(
                    Real::from(-3_i8),
                    Real::zero(),
                    Real::zero(),
                    Real::from(3_i8),
                    Real::from(2_i8),
                    Real::from(3_i8),
                )
                .expect("the scaled reflection is a similarity"),
            ] {
                let transformed = filleted
                    .transform_similarity_with_policy(&transform, &policy)
                    .unwrap_or_else(|error| {
                        panic!(
                            "the pair-native fillet must retain its exact similarity: policy={policy:?}, reversed={reversed}, error={error:?}"
                        )
                });
                assert_eq!(transformed.certainty, CurveCertainty::Certified);
                let transformed_selected_radial = transformed.value.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .find_map(|fragment| match fragment {
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                            if fragment.semicircle().uses_selected_radial_frame() =>
                        {
                            Some(fragment)
                        }
                        _ => None,
                    })
                    .expect("the transformed fillet retains its pair-radial carrier");
                let transformed_line = line.transform_similarity(&transform);
                let (transformed_contacts, transformed_parameter_map) =
                    match transformed_selected_radial
                        .semicircle()
                        .rational_intersections_with_parameter_map(
                            &transformed_line, &crate::CurveParameterRange2::unit(),
                            &policy,
                        )
                        .expect("the transformed pair-radial/rational kernel is exact")
                    {
                        Classification::Decided((
                            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps: unexpected_overlaps },
                            parameter_map,
                        )) if unexpected_overlaps.is_empty() => (contacts, parameter_map),
                        Classification::Decided((intersections, _)) => panic!(
                            "the transformed finite probe must produce contacts, got {intersections:?}"
                        ),
                        Classification::Uncertain(reason) => panic!(
                            "the transformed pair-radial/rational kernel must decide: {reason:?}"
                        ),
                    };
                assert_eq!(transformed_contacts.len(), contacts.len());
                assert!(
                    transformed_parameter_map.is_some(),
                    "the transported interior contact retains one shared parameter map"
                );
                for (source, transported) in contacts.iter().zip(&transformed_contacts) {
                    assert_eq!(source.location, transported.location);
                    assert!(
                        source.other_parameter.as_bezier_parameter().is_none()
                            && transported.other_parameter.as_bezier_parameter().is_none(),
                        "the line contacts must retain the compact recursive scalar instead of promoting a global root"
                    );
                    assert_eq!(
                        source
                            .other_parameter
                            .cmp_by_refinement(&transported.other_parameter, &policy)
                            .expect("transported target parameters remain comparable"),
                        Classification::Decided(std::cmp::Ordering::Equal),
                    );
                    let expected_cross = if transform.reverses_orientation() {
                        match source.tangent_cross_sign {
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => RealSign::Zero,
                            RealSign::Positive => RealSign::Negative,
                        }
                    } else {
                        source.tangent_cross_sign
                    };
                    assert_eq!(transported.tangent_cross_sign, expected_cross);
                }
                if policy == CurveContext::STRICT && !reversed && !transform.reverses_orientation()
                {
                    let nested_reflection = crate::Similarity2::try_from_real_affine(
                        Real::from(-1_i8),
                        Real::zero(),
                        Real::zero(),
                        Real::one(),
                        Real::from(11_i8),
                        Real::from(-13_i8),
                    )
                    .expect("the nested reflection is a similarity");
                    let nested_circle = transformed_selected_radial
                        .semicircle()
                        .transform_similarity(&nested_reflection)
                        .expect("a second exact similarity retains pair provenance");
                    let nested_line = transformed_line.transform_similarity(&nested_reflection);
                    let nested_contacts = match nested_circle
                        .rational_intersections_with_parameter_map(&nested_line, &crate::CurveParameterRange2::unit(), &policy)
                        .expect("the nested pair-radial/rational system remains exact")
                    {
                        Classification::Decided((
                            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps: unexpected_overlaps },
                            Some(_),
                        )) if unexpected_overlaps.is_empty() => contacts,
                        result => panic!(
                            "the nested transformed probe must retain contacts and a map, got {result:?}"
                        ),
                    };
                    assert_eq!(nested_contacts.len(), contacts.len());
                }
                if !reversed {
                    let transformed_square = selected_fillet_disjoint_square(&policy)
                        .transform_similarity_with_policy(&transform, &policy)
                        .expect("the disjoint Boolean fixture retains the same similarity");
                    let replay = transformed
                        .value
                        .boolean_regions_with_policy(&transformed_square.value, &policy)
                        .expect("the transformed pair-native fillet re-enters the Boolean kernel");
                    assert_eq!(replay.certainty, CurveCertainty::Certified);
                    assert_eq!(replay.value.union().boundary_loops().len(), 2);
                    assert!(replay.value.intersection().is_empty());
                }
            }
            if !reversed {
                let replay = filleted
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the pair-native fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(replay.value.union().boundary_loops().len(), 2);
                assert!(replay.value.intersection().is_empty());
            }
        }
    }
}

fn pair_native_crossing_cutter(
    filleted: &CurveRegion2,
    analytic_bottom: bool,
    policy: &CurveContext,
) -> CurveRegion2 {
    selected_radial_crossing_cutter(filleted, None, analytic_bottom, false, policy)
}

fn selected_radial_crossing_cutter(
    filleted: &CurveRegion2,
    selected_radius: Option<&Real>,
    analytic_bottom: bool,
    curved_crossing: bool,
    policy: &CurveContext,
) -> CurveRegion2 {
    let pair_fragment = filleted.boundary_loops()[0]
        .fragments()
        .iter()
        .find_map(|fragment| match fragment {
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                if fragment.semicircle().uses_selected_radial_frame() =>
            {
                selected_radius
                    .is_none_or(|radius| fragment.semicircle().radial_distance().abs() == *radius)
                    .then_some(fragment)
            }
            _ => None,
        })
        .expect("the fillet retains the requested selected-radial circle");
    let pair_circle = pair_fragment.semicircle();
    let parameter_bounds =
        |parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
         refinement_steps| {
            let bracket = retained_corner_decision(
                parameter
                    .parameter_bracket(refinement_steps, policy)
                    .expect("the pair-native parameter refines exactly"),
                CurveOperation2::Boolean,
            )
            .unwrap();
            match bracket {
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameterBracket2::Exact(
                    parameter,
                ) => (parameter.clone(), parameter),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameterBracket2::Interval(
                    interval,
                ) => (interval.start().clone(), interval.end().clone()),
            }
        };
    let interior_window = [8_usize, 16, 32, 64, 128]
        .into_iter()
        .find_map(|refinement_steps| {
            let (_, start_upper) =
                parameter_bounds(pair_fragment.start_parameter(), refinement_steps);
            let (end_lower, _) = parameter_bounds(pair_fragment.end_parameter(), refinement_steps);
            (compare_reals(&start_upper, &end_lower, &CurveContext::STRICT) == Some(Ordering::Less))
                .then_some((start_upper, end_lower))
        })
        .expect("the pair-native fragment has a separated exact interior");
    let third = Real::from(3_i8);
    let first_parameter =
        ((Real::from(2_i8) * &interior_window.0 + &interior_window.1) / &third).unwrap();
    let second_parameter =
        ((&interior_window.0 + Real::from(2_i8) * &interior_window.1) / third).unwrap();
    let point_at = |parameter: &Real| {
        retained_corner_decision(
            pair_circle
                .point_evidence_at(parameter, policy)
                .expect("an interior pair-native point remains exact"),
            CurveOperation2::Boolean,
        )
        .unwrap()
    };
    let first_point = point_at(&first_parameter);
    let second_point = point_at(&second_parameter);
    let crossing_axis = [8_usize, 16, 32, 64, 128]
        .into_iter()
        .find_map(|refinement_steps| {
            let first = retained_corner_decision(
                crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                    &first_point,
                    refinement_steps,
                    policy,
                ),
                CurveOperation2::Boolean,
            )
            .unwrap();
            let second = retained_corner_decision(
                crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                    &second_point,
                    refinement_steps,
                    policy,
                ),
                CurveOperation2::Boolean,
            )
            .unwrap();
            let separated_coordinate =
                |first_upper: &Real,
                 first_lower: &Real,
                 second_upper: &Real,
                 second_lower: &Real| {
                    if compare_reals(first_upper, second_lower, &CurveContext::STRICT)
                        == Some(Ordering::Less)
                    {
                        Some(((first_upper + second_lower) / Real::from(2_i8)).unwrap())
                    } else if compare_reals(second_upper, first_lower, &CurveContext::STRICT)
                        == Some(Ordering::Less)
                    {
                        Some(((second_upper + first_lower) / Real::from(2_i8)).unwrap())
                    } else {
                        None
                    }
                };
            separated_coordinate(
                first.max().x(),
                first.min().x(),
                second.max().x(),
                second.min().x(),
            )
            .map(|coordinate| (true, coordinate))
            .or_else(|| {
                separated_coordinate(
                    first.max().y(),
                    first.min().y(),
                    second.max().y(),
                    second.min().y(),
                )
                .map(|coordinate| (false, coordinate))
            })
        })
        .expect("two distinct interior circle points separate on one exact axis");
    let center = retained_corner_decision(
        pair_circle
            .center_point_evidence(policy)
            .expect("the pair-native center remains exact"),
        CurveOperation2::Boolean,
    )
    .unwrap();
    let bounds = retained_corner_decision(
        crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(&center, 8, policy),
        CurveOperation2::Boolean,
    )
    .unwrap();
    let two = Real::from(2_i8);
    let margin = pair_circle.radial_distance().abs() * &two + Real::one();
    let left = bounds.min().x() - &margin;
    let right = bounds.max().x() + &margin;
    let bottom = bounds.min().y() - &margin;
    let top = bounds.max().y() + &margin;
    let line = |start, end| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(start, end).unwrap(),
        )),
    };
    let vertices = if crossing_axis.0 {
        [
            Point2::new(crossing_axis.1.clone(), top.clone()),
            Point2::new(crossing_axis.1, bottom.clone()),
            Point2::new(right.clone(), bottom),
            Point2::new(right, top),
        ]
    } else {
        [
            Point2::new(left.clone(), crossing_axis.1.clone()),
            Point2::new(right.clone(), crossing_axis.1),
            Point2::new(right, top.clone()),
            Point2::new(left, top),
        ]
    };
    let crossing_source = if curved_crossing {
        let half = q(1, 2);
        let bend = (pair_circle.radial_distance().abs() / Real::from(1000_i16)).unwrap();
        let midpoint = Point2::new(
            (vertices[0].x() + vertices[1].x()) * &half,
            (vertices[0].y() + vertices[1].y()) * half,
        );
        let control = if crossing_axis.0 {
            midpoint.translated(bend, Real::zero())
        } else {
            midpoint.translated(Real::zero(), bend)
        };
        QuadraticBezier2::new(vertices[0].clone(), control, vertices[1].clone())
    } else {
        QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(vertices[0].clone(), vertices[1].clone()).unwrap(),
        )
    };
    let crossing = if analytic_bottom {
        let Classification::Decided(crossing) = crate::BezierParallelFragment2::try_new(
            crossing_source.parallel_left(Real::zero()).unwrap(),
            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            policy,
        )
        .unwrap() else {
            panic!("the retained line cutter has a decided range");
        };
        BezierSplitFragment2::AnalyticParallel(crossing)
    } else {
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(crossing_source),
        }
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::new(
                vec![
                    crossing,
                    line(vertices[1].clone(), vertices[2].clone()),
                    line(vertices[2].clone(), vertices[3].clone()),
                    line(vertices[3].clone(), vertices[0].clone()),
                ],
                policy,
            )
            .expect("the crossing cutter closes exactly"),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .expect("the crossing cutter has authored topology")
}

fn selected_radial_disk(
    semicircle: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    policy: &CurveContext,
) -> CurveRegion2 {
    assert!(semicircle.uses_selected_radial_frame());
    let interior_side = if semicircle.is_clockwise() {
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            BezierSplitFragment2::AlgebraicCuspSemicircle(
                crate::BezierAlgebraicCuspSemicircleFragment2::full(semicircle.clone(), policy),
            ),
            BezierSplitFragment2::AlgebraicCuspSemicircle(
                crate::BezierAlgebraicCuspSemicircleFragment2::full(
                    semicircle.complementary_half(),
                    policy,
                ),
            ),
        ],
        policy,
    )
    .expect("the selected-radial disk closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .expect("the selected-radial disk has authored topology")
}

fn selected_radial_cap(
    semicircle: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    policy: &CurveContext,
) -> CurveRegion2 {
    assert!(semicircle.uses_selected_radial_frame());
    let arc = crate::BezierAlgebraicCuspSemicircleFragment2::full(semicircle, policy);
    let Classification::Decided(Some(start)) = arc.endpoint_point_evidence(true, policy).unwrap()
    else {
        panic!("the selected-radial cap start must retain exact point evidence");
    };
    let Classification::Decided(Some(end)) = arc.endpoint_point_evidence(false, policy).unwrap()
    else {
        panic!("the selected-radial cap end must retain exact point evidence");
    };
    let Classification::Decided(chord) =
        crate::bezier_offset::BezierAlgebraicChord2::try_new(end, start, policy).unwrap()
    else {
        panic!("the selected-radial cap diameter must retain an exact chord");
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            BezierSplitFragment2::AlgebraicCuspSemicircle(arc),
            BezierSplitFragment2::AlgebraicChord(chord),
        ],
        policy,
    )
    .expect("the selected-radial cap closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .expect("the selected-radial cap has authored topology")
}

fn recursive_selected_radial_nonlinear_cutter(policy: &CurveContext) -> CurveRegion2 {
    let start = Point2::from_values(-1, -1);
    let end = Point2::from_values(2, -1);
    let bottom_right = Point2::from_values(2, -2);
    let bottom_left = Point2::from_values(-1, -2);
    let fragment = |curve| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(curve),
    };
    let line = |start, end| {
        fragment(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(start, end).unwrap(),
        ))
    };
    let crossing = fragment(QuadraticBezier2::new(
        start.clone(),
        Point2::new(q(1, 2), -q(9, 10)),
        end.clone(),
    ));
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            crossing,
            line(end, bottom_right.clone()),
            line(bottom_right, bottom_left.clone()),
            line(bottom_left, start),
        ],
        policy,
    )
    .expect("the rational nonlinear cutter closes exactly");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Right],
    )
    .expect("the rational nonlinear cutter has authored topology")
}

fn selected_radial_linear_corner(region: &CurveRegion2, selected_radius: &Real) -> (usize, usize) {
    let selected_circle = |fragment: &BezierSplitFragment2| {
        matches!(
            fragment,
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                if fragment.semicircle().uses_selected_radial_frame()
                    && fragment.semicircle().radial_distance().abs() == *selected_radius
        )
    };
    let retained_line = |fragment: &BezierSplitFragment2| {
        matches!(
            fragment,
            BezierSplitFragment2::AlgebraicChord(_) | BezierSplitFragment2::Materialized { .. }
        )
    };
    region
        .boundary_loops()
        .iter()
        .enumerate()
        .find_map(|(loop_index, boundary)| {
            let fragments = boundary.fragments();
            (0..fragments.len()).find_map(|corner| {
                let previous = &fragments[(corner + fragments.len() - 1) % fragments.len()];
                let next = &fragments[corner];
                ((selected_circle(previous) && retained_line(next))
                    || (retained_line(previous) && selected_circle(next)))
                .then_some((loop_index, corner))
            })
        })
        .expect("the clipped region retains the requested selected-radial/line corner")
}

fn next_selected_radial_boolean_fillet_generation(
    source: &CurveRegion2,
    source_radius: &Real,
    policy: &CurveContext,
) -> (CurveRegion2, Real) {
    let cutter = selected_radial_crossing_cutter(source, Some(source_radius), false, false, policy);
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::reset();
    let boolean_work = || source.boolean_regions_with_policy(&cutter, policy);
    #[cfg(feature = "dispatch-trace")]
    let booleans = hyperreal::dispatch_trace::with_recording(boolean_work);
    #[cfg(not(feature = "dispatch-trace"))]
    let booleans = boolean_work();
    let clipped = booleans
        .unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            panic!(
                "the recursive selected-radial cutter must publish exact topology: {error:?}; trace={:?}",
                hyperreal::dispatch_trace::take_trace()
            );
            #[cfg(not(feature = "dispatch-trace"))]
            panic!("the recursive selected-radial cutter must publish exact topology: {error:?}");
        })
        .into_value()
        .intersection()
        .clone();
    let (loop_index, corner) = selected_radial_linear_corner(&clipped, source_radius);
    let radius = (source_radius / Real::from(100_i16)).unwrap();
    let result = clipped
        .fillet_loop_vertex_with_policy(
            loop_index,
            corner,
            &crate::CurveFillet2::new(radius.clone()),
            CurveCornerMode2::TrimOnly,
            policy,
        )
        .expect("the recursive selected-radial/line corner must fillet exactly");
    assert_eq!(result.certainty, CurveCertainty::Certified);
    let candidates = {
        let solutions = result.value;
        let candidates = solutions.into_solutions();
        assert!(
            !candidates.is_empty(),
            "expected at least one isolated fillet"
        );
        candidates
    };
    let candidate = candidates
        .into_iter()
        .find(|candidate| {
            candidate
                .boundary_loops()
                .iter()
                .flat_map(|boundary| boundary.fragments())
                .any(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                        if fragment.semicircle().uses_selected_radial_frame() =>
                    {
                        fragment.semicircle().radial_distance().abs() == radius
                    }
                    _ => false,
                })
        })
        .expect("the recursive fillet retains its selected-radial circle");
    (candidate, radius)
}

fn fourth_selected_radial_boolean_fillet_generation(policy: &CurveContext) -> (CurveRegion2, Real) {
    let source = independent_pair_native_fillet(policy, false);
    let source_radius = pair_radial_corner(&source).1;
    let (third_generation, third_radius) =
        next_selected_radial_boolean_fillet_generation(&source, &source_radius, policy);
    next_selected_radial_boolean_fillet_generation(&third_generation, &third_radius, policy)
}

fn exact_raw_bevel_offset_loops(
    source: &CurveRegion2,
    distance: &Real,
    policy: &CurveContext,
) -> (Vec<CurveRegionBoundaryLoop2>, usize) {
    let filled_sides = match source
        .filled_side_is_left_raw(policy)
        .expect("the recursive filled sides remain exact")
    {
        Classification::Decided(sides) => sides,
        Classification::Uncertain(reason) => {
            panic!("the recursive filled sides must decide: {reason:?}")
        }
    };
    let mut offset_loops = Vec::with_capacity(source.boundary_loops().len());
    let mut mixed_tangent_joins = 0;
    for (loop_index, boundary) in source.boundary_loops().iter().enumerate() {
        let signed_left_distance = if filled_sides[loop_index] {
            -distance.clone()
        } else {
            distance.clone()
        };
        let spans = match exact_offset_span_runs_from_boundary_loop(
            boundary,
            &signed_left_distance,
            policy,
        )
        .expect("the recursive offset spans remain valid")
        {
            Classification::Decided(spans) => {
                spans.into_iter().map(|(span, _)| span).collect::<Vec<_>>()
            }
            Classification::Uncertain(reason) => panic!(
                "the recursive offset spans must decide: loop={loop_index}, reason={reason:?}"
            ),
        };
        let mut fragments = Vec::new();
        for span_index in 0..spans.len() {
            fragments.extend(spans[span_index].fragments.iter().cloned());
            let next_index = (span_index + 1) % spans.len();
            if let Some((first, second)) = spans[span_index]
                .end_tangent
                .as_ref()
                .zip(spans[next_index].start_tangent.as_ref())
                && matches!(
                    (first, second),
                    (
                        CurveTangent2::SelectedCircularEndpoint { .. },
                        CurveTangent2::ChordContact { .. }
                    ) | (
                        CurveTangent2::ChordContact { .. },
                        CurveTangent2::SelectedCircularEndpoint { .. }
                    )
                )
            {
                mixed_tangent_joins += 1;
                let Classification::Decided(forward) =
                    curve_tangent_cross_sign(first, second, policy)
                else {
                    panic!(
                        "the mixed circular tangent cross must decide: loop={loop_index}, span={span_index}"
                    );
                };
                let Classification::Decided(reverse) =
                    curve_tangent_cross_sign(second, first, policy)
                else {
                    panic!(
                        "the reversed mixed circular tangent cross must decide: loop={loop_index}, span={span_index}"
                    );
                };
                assert_eq!(reverse, exact_sign_reverse(forward));
                if forward == RealSign::Zero {
                    let opposite = curve_tangents_are_opposite(first, second, policy);
                    assert!(matches!(opposite, Classification::Decided(_)));
                    assert_eq!(opposite, curve_tangents_are_opposite(second, first, policy),);
                }
            }
            match append_exact_offset_join(
                &mut fragments,
                &spans[span_index],
                &spans[next_index],
                &signed_left_distance,
                &OffsetCornerStyle2::Bevel,
                policy,
            )
            .expect("the recursive offset join remains valid")
            {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => panic!(
                    "the recursive offset join must decide: loop={loop_index}, span={span_index}, reason={reason:?}"
                ),
            }
        }
        offset_loops.push(
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .expect("the recursive offset chain closes exactly"),
        );
    }
    (offset_loops, mixed_tangent_joins)
}

#[test]
fn recursively_nested_selected_radial_operations_remain_exact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (fourth_generation, fourth_radius) =
            fourth_selected_radial_boolean_fillet_generation(&policy);
        let replay = fourth_generation
            .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
            .expect("the fourth-generation fillet re-enters the Boolean kernel");
        assert_eq!(replay.certainty, CurveCertainty::Certified);
        assert!(replay.value.intersection().is_empty());
        assert_eq!(replay.value.union().boundary_loops().len(), 2);

        let offset_distance = (fourth_radius.clone() / Real::from(20_i8)).unwrap();
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let assembly_work =
            || exact_raw_bevel_offset_loops(&fourth_generation, &offset_distance, &policy);
        #[cfg(feature = "dispatch-trace")]
        let (offset_loops, mixed_tangent_joins) =
            hyperreal::dispatch_trace::with_recording(assembly_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let (offset_loops, mixed_tangent_joins) = assembly_work();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(offset_loops.len(), fourth_generation.boundary_loops().len());
        assert!(mixed_tangent_joins > 0);
        #[cfg(feature = "dispatch-trace")]
        assert!(
            trace.path_count(
                "hypercurve",
                "curve-region-exact-offset-tangent-cross",
                "selected-circle-chord-contact",
            ) > 0,
            "the recursive offset must use its retained circular tangent chords: {trace:?}",
        );
        #[cfg(feature = "dispatch-trace")]
        assert!(
            trace.path_count(
                "hypercurve",
                "curve-region-exact-offset-selected-circle-pair-tangent",
                "retained-pair",
            ) > 0,
            "the recursive offset must replay retained pair provenance before chord refinement: {trace:?}",
        );

        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let adjacency_work = || {
            let mut adjacent_cusp_pairs = 0_usize;
            let mut retained_tangent_pairs = 0_usize;
            for boundary in &offset_loops {
                let fragments = boundary.fragments();
                for index in 0..fragments.len() {
                    let next = (index + 1) % fragments.len();
                    let (
                        BezierSplitFragment2::AlgebraicCuspSemicircle(first),
                        BezierSplitFragment2::AlgebraicCuspSemicircle(second),
                    ) = (&fragments[index], &fragments[next])
                    else {
                        continue;
                    };
                    adjacent_cusp_pairs += 1;
                    if matches!(
                        first
                            .unique_shared_tangent_endpoint_contact(second, &policy)
                            .expect("adjacent recursive endpoint tangency remains valid"),
                        Classification::Decided(Some(_)),
                    ) {
                        retained_tangent_pairs += 1;
                        continue;
                    }
                    let pair = first
                        .semicircle()
                        .pair_intersections(second.semicircle(), &policy)
                        .expect("adjacent recursive offset circles remain valid");
                    assert!(
                        matches!(pair, Classification::Decided(_)),
                        "policy {policy:?}, boundary pair {index}->{next}: {pair:?}",
                    );
                }
            }
            (adjacent_cusp_pairs, retained_tangent_pairs)
        };
        #[cfg(feature = "dispatch-trace")]
        let (adjacent_cusp_pairs, retained_tangent_pairs) =
            hyperreal::dispatch_trace::with_recording(adjacency_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let (adjacent_cusp_pairs, retained_tangent_pairs) = adjacency_work();
        assert!(adjacent_cusp_pairs > 0);
        assert!(retained_tangent_pairs > 0);
        #[cfg(feature = "dispatch-trace")]
        let adjacent_pair_trace = hyperreal::dispatch_trace::take_trace();
        #[cfg(feature = "dispatch-trace")]
        assert!(
            adjacent_pair_trace.path_count(
                "hypercurve",
                "algebraic-circle-pair-kernel",
                "retained-pair-endpoint-tangent",
            ) > 0,
            "the fourth-generation offset must replay its retained adjacent circle tangency: {adjacent_pair_trace:?}",
        );

        let (loop_index, corner) =
            selected_radial_linear_corner(&fourth_generation, &fourth_radius);
        let setback = (fourth_radius / Real::from(10_i8)).unwrap();
        assert_eq!(real_sign(&setback, &policy), Some(RealSign::Positive));
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let chamfer_work = || {
            fourth_generation.chamfer_loop_vertex_by_setbacks_with_policy(
                loop_index,
                corner,
                setback.clone(),
                setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let chamfer = hyperreal::dispatch_trace::with_recording(chamfer_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let chamfer = chamfer_work();
        #[cfg(feature = "dispatch-trace")]
        let chamfer_trace = hyperreal::dispatch_trace::take_trace();
        let chamfer = chamfer.unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            panic!(
                "the fourth-generation retained corner chamfers exactly: {error:?}; {chamfer_trace:?}"
            );
            #[cfg(not(feature = "dispatch-trace"))]
            panic!("the fourth-generation retained corner chamfers exactly: {error:?}");
        });
        #[cfg(feature = "dispatch-trace")]
        assert!(
            chamfer_trace.path_count(
                "hypercurve",
                "selected-circle-chamfer-chart",
                "inward-fragment",
            ) > 0,
            "the recursive chamfer must retain its current semicircle chart: {chamfer_trace:?}",
        );
        #[cfg(feature = "dispatch-trace")]
        assert!(
            chamfer_trace.path_count(
                "hypercurve",
                "selected-circle-chamfer-trim-domain",
                "endpoint-chord-distance",
            ) > 0,
            "the recursive chamfer must certify its trim domain by endpoint chord distance: {chamfer_trace:?}",
        );
        assert_eq!(chamfer.certainty, CurveCertainty::Certified);
        for_each_corner_region(corner_regions(&chamfer.value), |chamfered| {
            let replay = chamfered
                .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                .expect("the fourth-generation chamfer re-enters the Boolean kernel");
            assert_eq!(replay.certainty, CurveCertainty::Certified);
            assert!(replay.value.intersection().is_empty());
        });
    }
}

#[test]
fn recursive_selected_radial_crosses_a_nonlinear_quadratic() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let recursive = crate::bezier_offset::recursively_line_contact_radial_half(&policy);
        let recursive_radius = recursive.radial_distance().abs();
        let recursive_disk = selected_radial_disk(recursive, &policy);
        assert_eq!(recursive_radius, q(1, 4));
        let cutter = recursive_selected_radial_nonlinear_cutter(&policy);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let intersection_work = || recursive_disk.intersect_region_with_policy(&cutter, &policy);
        #[cfg(feature = "dispatch-trace")]
        let intersections = hyperreal::dispatch_trace::with_recording(intersection_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let intersections = intersection_work();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        let intersections = intersections.unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            panic!(
                "the recursive selected-radial/nonlinear crossing must decide: policy={policy:?}, error={error:?}, trace={trace:?}"
            );
            #[cfg(not(feature = "dispatch-trace"))]
            panic!(
                "the recursive selected-radial/nonlinear crossing must decide: policy={policy:?}, error={error:?}"
            );
        });
        #[cfg(feature = "dispatch-trace")]
        assert!(
            trace.path_count(
                "hypercurve",
                "algebraic-circle-rational-kernel",
                "recursive-quadratic",
            ) > 0,
            "the deep selected-radial crossing must stay in the recursive projective rational kernel: {trace:?}",
        );
        #[cfg(feature = "dispatch-trace")]
        assert!(
            trace.path_count(
                "hypercurve",
                "recursive-polynomial-roots",
                "local-bernstein",
            ) > 0,
            "the deep selected-radial crossing must isolate its target roots in the retained coefficient field: {trace:?}",
        );
        #[cfg(feature = "dispatch-trace")]
        assert_eq!(
            trace.path_count(
                "hypercurve",
                "recursive-polynomial-roots",
                "projected-replay",
            ),
            0,
            "the transverse recursive quartic must not construct a dense global norm: {trace:?}",
        );
        #[cfg(feature = "dispatch-trace")]
        assert_eq!(
            trace.path_count(
                "hypercurve",
                "algebraic-circle-rational-kernel",
                "represented",
            ),
            0,
            "the deep selected-radial crossing must not materialize a represented Cartesian frame: {trace:?}",
        );
        assert!(
            intersections.value.is_complete(),
            "the recursive nonlinear crossing must retain complete intersection evidence: policy={policy:?}, blockers={:?}",
            intersections.value.blockers(),
        );
        assert!(!intersections.value.contacts().is_empty());
        let booleans = recursive_disk
            .boolean_regions_with_policy(&cutter, &policy)
            .unwrap_or_else(|error| {
                panic!(
                    "the recursive selected-radial/nonlinear crossing must publish Boolean topology: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(booleans.certainty, CurveCertainty::Certified);
        assert!(!booleans.value.intersection().is_empty());
        assert!(!booleans.value.difference().is_empty());
    }
}

#[test]
fn recursive_selected_radial_cap_crosses_a_nonlinear_quadratic() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let recursive = crate::bezier_offset::recursively_line_contact_radial_half(&policy);
        let recursive_cap = selected_radial_cap(recursive, &policy);
        let cutter = recursive_selected_radial_nonlinear_cutter(&policy);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let intersection_work = || recursive_cap.intersect_region_with_policy(&cutter, &policy);
        #[cfg(feature = "dispatch-trace")]
        let intersections = hyperreal::dispatch_trace::with_recording(intersection_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let intersections = intersection_work();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        let intersections = intersections.unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            panic!(
                "the recursive selected-radial cap/nonlinear crossing must decide: policy={policy:?}, error={error:?}, trace={trace:?}"
            );
            #[cfg(not(feature = "dispatch-trace"))]
            panic!(
                "the recursive selected-radial cap/nonlinear crossing must decide: policy={policy:?}, error={error:?}"
            );
        });
        #[cfg(feature = "dispatch-trace")]
        assert!(
            trace.path_count(
                "hypercurve",
                "algebraic-chord-rational-kernel",
                "recursive-projective",
            ) > 0,
            "the recursive cap diameter must use the shared projective rational kernel: {trace:?}",
        );
        assert!(
            intersections.value.is_complete(),
            "the recursive cap crossing must retain complete intersection evidence: policy={policy:?}, blockers={:?}",
            intersections.value.blockers(),
        );
        assert!(!intersections.value.contacts().is_empty());
        let booleans = recursive_cap
            .boolean_regions_with_policy(&cutter, &policy)
            .unwrap_or_else(|error| {
                panic!(
                    "the recursive selected-radial cap/nonlinear crossing must publish Boolean topology: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(booleans.certainty, CurveCertainty::Certified);
        assert!(!booleans.value.intersection().is_empty());
        assert!(!booleans.value.difference().is_empty());
    }
}

#[test]
fn recursive_selected_radial_projective_chamfer_reenters_corner_kernel() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = independent_pair_native_fillet(&policy, false);
        let source_radius = pair_radial_corner(&source).1;
        let (third_generation, third_radius) =
            next_selected_radial_boolean_fillet_generation(&source, &source_radius, &policy);
        let (loop_index, corner) = selected_radial_linear_corner(&third_generation, &third_radius);
        let edit_radius = (third_radius / Real::from(10_i8)).unwrap();

        let trim_chamfer = third_generation
            .chamfer_loop_vertex_by_setbacks_with_policy(
                loop_index,
                corner,
                edit_radius.clone(),
                edit_radius.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("the recursive selected-radial/line trim chamfer remains exact");
        let extended_chamfer = third_generation
            .chamfer_loop_vertex_by_setbacks_with_policy(
                loop_index,
                corner,
                edit_radius.clone(),
                edit_radius.clone(),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .unwrap_or_else(|error| {
                panic!(
                    "the recursive selected-radial/line chamfer must extend exactly: policy={policy:?}, error={error:?}"
                )
            });
        assert_eq!(trim_chamfer.certainty, CurveCertainty::Certified);
        assert_eq!(extended_chamfer.certainty, CurveCertainty::Certified);
        assert!(
            extended_chamfer.value.candidate_count() > trim_chamfer.value.candidate_count(),
            "the incident carrier rays must contribute an exterior chamfer: trim={}, extended={}",
            trim_chamfer.value.candidate_count(),
            extended_chamfer.value.candidate_count(),
        );

        let candidates = match extended_chamfer.value {
            CurveCornerSolutions2::Unique(candidate) => vec![candidate],
            CurveCornerSolutions2::Multiple(candidates) => candidates,
            CurveCornerSolutions2::NoSolution(reason) => {
                panic!("the projective chamfer must have a solution: {reason:?}")
            }
        };
        let chamfer_parameter =
            |parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2| {
                matches!(
                    parameter,
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Mapped(data)
                        if matches!(
                            data.as_ref(),
                            crate::bezier_offset::BezierAlgebraicCuspSemicircleMappedParameterData2::Chamfer { .. }
                        )
                )
            };
        let mut reentered = false;
        for candidate in candidates {
            let nested_corner =
                candidate
                    .boundary_loops()
                    .iter()
                    .enumerate()
                    .find_map(|(loop_index, boundary)| {
                        let fragments = boundary.fragments();
                        (0..fragments.len()).find_map(|corner| {
                            let previous =
                                &fragments[(corner + fragments.len() - 1) % fragments.len()];
                            let next = &fragments[corner];
                            let mapped_circle = match previous {
                                BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                                    chamfer_parameter(fragment.endpoint_parameter(false))
                                }
                                _ => false,
                            } || match next {
                                BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                                    chamfer_parameter(fragment.endpoint_parameter(true))
                                }
                                _ => false,
                            };
                            mapped_circle.then_some((loop_index, corner))
                        })
                    });
            let Some((nested_loop, nested_corner)) = nested_corner else {
                continue;
            };
            let nested_setback = (&edit_radius / Real::from(4_i8)).unwrap();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let nested_chamfer_work = || {
                candidate.chamfer_loop_vertex_by_setbacks_with_policy(
                    nested_loop,
                    nested_corner,
                    nested_setback.clone(),
                    nested_setback,
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let nested = hyperreal::dispatch_trace::with_recording(nested_chamfer_work);
            #[cfg(not(feature = "dispatch-trace"))]
            let nested = nested_chamfer_work();
            #[cfg(feature = "dispatch-trace")]
            let nested_chamfer_trace = hyperreal::dispatch_trace::take_trace();
            let nested = nested.unwrap_or_else(|error| {
                panic!(
                    "the recursively mapped projective chamfer endpoint must re-enter the corner kernel: policy={policy:?}, error={error:?}"
                )
            });
            assert_eq!(nested.certainty, CurveCertainty::Certified);
            assert!(nested.value.candidate_count() > 0);
            #[cfg(feature = "dispatch-trace")]
            assert!(
                nested_chamfer_trace.path_count(
                    "hypercurve",
                    "recursive-projective-axis-order",
                    "interval-separated",
                ) > 0
                    || nested_chamfer_trace.path_count(
                        "hypercurve",
                        "algebraic-chord-point-axis-order",
                        "interval-separated",
                    ) > 0,
                "the nested chamfer must order its transported endpoint in a retained recursive chart: {nested_chamfer_trace:?}",
            );
            let nested_fillet_radius = (&edit_radius / Real::from(16_i8)).unwrap();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let nested_fillet_work = || {
                candidate.fillet_loop_vertex_with_policy(
                    nested_loop,
                    nested_corner,
                    &crate::CurveFillet2::new(nested_fillet_radius),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let nested_fillet = hyperreal::dispatch_trace::with_recording(nested_fillet_work);
            #[cfg(not(feature = "dispatch-trace"))]
            let nested_fillet = nested_fillet_work();
            #[cfg(feature = "dispatch-trace")]
            let nested_fillet_trace = hyperreal::dispatch_trace::take_trace();
            let nested_fillet = nested_fillet.unwrap_or_else(|error| {
                panic!(
                    "the recursively mapped projective chamfer endpoint must fillet exactly: policy={policy:?}, error={error:?}"
                )
            });
            assert_eq!(nested_fillet.certainty, CurveCertainty::Certified);
            assert!(!nested_fillet.value.solutions().is_empty());
            #[cfg(feature = "dispatch-trace")]
            assert!(
                nested_fillet_trace.path_count(
                    "hypercurve",
                    "algebraic-circle-chord-tangent-dot",
                    "retained-contact-map",
                ) > 0,
                "the nested fillet must replay the retained circle/chord tangent dot: {nested_fillet_trace:?}",
            );
            #[cfg(feature = "dispatch-trace")]
            assert!(
                nested_fillet_trace.path_count(
                    "hypercurve",
                    "recursive-projective-parameter",
                    "certified-unit-bounds",
                ) > 0,
                "the finite recursive contact must reuse its constructed unit bounds: {nested_fillet_trace:?}",
            );
            reentered = true;
            break;
        }
        assert!(
            reentered,
            "one exterior recursive chamfer must retain its mapped circle endpoint"
        );
    }
}

#[test]
fn fourth_selected_radial_approximate_nonadjacent_circle_pairs_decide() {
    let policy = CurveContext::APPROXIMATE_512;
    let (fourth_generation, fourth_radius) =
        fourth_selected_radial_boolean_fillet_generation(&policy);
    let distance = (fourth_radius / Real::from(20_i8)).unwrap();
    let (offset_loops, _) = exact_raw_bevel_offset_loops(&fourth_generation, &distance, &policy);
    let mut circles = Vec::new();
    for (loop_index, boundary) in offset_loops.iter().enumerate() {
        for (fragment_index, fragment) in boundary.fragments().iter().enumerate() {
            if let BezierSplitFragment2::AlgebraicCuspSemicircle(circle) = fragment {
                circles.push((
                    loop_index,
                    fragment_index,
                    boundary.fragments().len(),
                    circle,
                ));
            }
        }
    }
    let mut nonadjacent_pairs = 0_usize;
    for first_index in 0..circles.len() {
        for second_index in (first_index + 1)..circles.len() {
            let (first_loop, first_fragment, first_count, first) = circles[first_index];
            let (second_loop, second_fragment, second_count, second) = circles[second_index];
            let adjacent = first_loop == second_loop
                && first_count == second_count
                && ((first_fragment + 1) % first_count == second_fragment
                    || (second_fragment + 1) % second_count == first_fragment);
            if adjacent {
                continue;
            }
            nonadjacent_pairs += 1;
            if matches!(
                first
                    .unique_shared_tangent_endpoint_contact(second, &policy)
                    .expect("a retained nonadjacent tangent remains valid"),
                Classification::Decided(Some(_)),
            ) {
                continue;
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let pair_work = || {
                first
                    .semicircle()
                    .pair_intersections(second.semicircle(), &policy)
                    .expect("a nonadjacent recursive circle pair remains valid")
            };
            #[cfg(feature = "dispatch-trace")]
            let result = hyperreal::dispatch_trace::with_recording(pair_work);
            #[cfg(not(feature = "dispatch-trace"))]
            let result = pair_work();
            #[cfg(feature = "dispatch-trace")]
            let pair_trace = hyperreal::dispatch_trace::take_trace()
                .dispatch
                .into_iter()
                .filter(|entry| entry.layer == "hypercurve")
                .collect::<Vec<_>>();
            #[cfg(not(feature = "dispatch-trace"))]
            let pair_trace: Vec<()> = Vec::new();
            assert!(
                matches!(result, Classification::Decided(_)),
                "nonadjacent recursive circle pair ({first_loop},{first_fragment})/({second_loop},{second_fragment}) must decide: {result:?}; frames selected=({}, {}), chord=({}, {}), parallel=({}, {}); trace={pair_trace:?}",
                first.semicircle().uses_selected_radial_frame(),
                second.semicircle().uses_selected_radial_frame(),
                first.semicircle().uses_selected_chord_normal_frame(),
                second.semicircle().uses_selected_chord_normal_frame(),
                first.semicircle().uses_selected_parallel_normal_frame(),
                second.semicircle().uses_selected_parallel_normal_frame(),
            );
        }
    }
    assert!(nonadjacent_pairs > 0);
}

fn assert_fourth_selected_radial_public_offset_regularizes(policy: CurveContext) {
    let (fourth_generation, fourth_radius) =
        fourth_selected_radial_boolean_fillet_generation(&policy);
    let distance = (fourth_radius / Real::from(20_i8)).unwrap();
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::reset();
    let offset_work =
        || fourth_generation.offset_with_policy(distance, &OffsetCornerStyle2::Bevel, &policy);
    #[cfg(feature = "dispatch-trace")]
    let offset = hyperreal::dispatch_trace::with_recording(offset_work);
    #[cfg(not(feature = "dispatch-trace"))]
    let offset = offset_work();
    #[cfg(feature = "dispatch-trace")]
    let trace = hyperreal::dispatch_trace::take_trace();
    let offset = offset.unwrap_or_else(|error| {
        #[cfg(feature = "dispatch-trace")]
        {
            let hypercurve_trace = trace
                .dispatch
                .iter()
                .filter(|entry| entry.layer == "hypercurve")
                .collect::<Vec<_>>();
            panic!(
                "the fourth-generation public offset must regularize exactly: {error:?}; hypercurve trace: {hypercurve_trace:?}"
            );
        }
        #[cfg(not(feature = "dispatch-trace"))]
        panic!("the fourth-generation public offset must regularize exactly: {error:?}");
    });
    assert_eq!(offset.certainty, CurveCertainty::Certified);
    assert!(!offset.value.is_empty());
}

#[test]
fn recursively_nested_selected_radial_public_offset_regularizes_strict() {
    assert_fourth_selected_radial_public_offset_regularizes(CurveContext::STRICT);
}

#[test]
fn recursively_nested_selected_radial_public_offset_regularizes_approximate_512() {
    assert_fourth_selected_radial_public_offset_regularizes(CurveContext::APPROXIMATE_512);
}

#[test]
fn independent_pair_native_fillet_crosses_a_boolean_cutter() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let filleted = independent_pair_native_fillet(&policy, reversed);
            for analytic_bottom in [false, true] {
                let cutter = pair_native_crossing_cutter(&filleted, analytic_bottom, &policy);
                let evidence = filleted
                    .intersect_region_with_policy(&cutter, &policy)
                    .expect("the crossing carrier pairs return retained evidence");
                assert!(
                    evidence.value.is_complete(),
                    "the crossing carrier pairs must complete: policy={policy:?}, reversed={reversed}, analytic_bottom={analytic_bottom}, blockers={:?}",
                    evidence.value.blockers()
                );
                let booleans = filleted
                    .boolean_regions_with_policy(&cutter, &policy)
                    .unwrap_or_else(|error| {
                        panic!(
                            "the pair-native fillet must cross the Boolean cutter: policy={policy:?}, reversed={reversed}, analytic_bottom={analytic_bottom}, error={error:?}"
                        )
                    });
                assert_eq!(booleans.certainty, CurveCertainty::Certified);
                assert!(!booleans.value.intersection().is_empty());
                assert!(!booleans.value.difference().is_empty());
            }
        }
    }
}

fn assert_pair_native_boolean_boundary_offsets_exactly(policy: CurveContext, reversed: bool) {
    let pair_native = independent_pair_native_fillet(&policy, reversed);
    let cutter = pair_native_crossing_cutter(&pair_native, false, &policy);
    let booleans = pair_native
        .boolean_regions_with_policy(&cutter, &policy)
        .expect("the pair-native cutter publishes one exact intersection");
    let clipped = booleans.value.intersection();
    let parent_radius = clipped
        .boundary_loops()
        .iter()
        .flat_map(|boundary| boundary.fragments())
        .find_map(|fragment| match fragment {
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                if fragment.semicircle().uses_selected_radial_frame() =>
            {
                Some(fragment.semicircle().radial_distance().abs())
            }
            _ => None,
        })
        .expect("the Boolean boundary retains its pair-native circle");
    let distance = (parent_radius / Real::from(200_i16)).unwrap();
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::reset();
    let offset_work = || clipped.offset_with_policy(distance, &OffsetCornerStyle2::Bevel, &policy);
    #[cfg(feature = "dispatch-trace")]
    let offset = hyperreal::dispatch_trace::with_recording(offset_work);
    #[cfg(not(feature = "dispatch-trace"))]
    let offset = offset_work();
    #[cfg(feature = "dispatch-trace")]
    let trace = hyperreal::dispatch_trace::take_trace();
    let offset = offset.unwrap_or_else(|error| {
        #[cfg(feature = "dispatch-trace")]
        {
            let hypercurve_trace = trace
                .dispatch
                .iter()
                .filter(|entry| entry.layer == "hypercurve")
                .collect::<Vec<_>>();
            panic!(
                "the Boolean-fragmented pair-native boundary must offset: {error:?}; hypercurve trace: {hypercurve_trace:?}"
            );
        }
        #[cfg(not(feature = "dispatch-trace"))]
        panic!("the Boolean-fragmented pair-native boundary must offset: {error:?}");
    });
    #[cfg(feature = "dispatch-trace")]
    if !reversed {
        assert!(
            trace.path_count("hypercurve", "regularization-successor", "face-sector")
                + trace.path_count("hypercurve", "regularization-successor", "forced-bijection")
                > 0,
            "the endpoint crossing must reuse certified sector or connectivity topology: {trace:?}",
        );
        assert_eq!(
            trace.path_count(
                "hypercurve",
                "retained-endpoint-scope",
                "tangent-order-rebuild",
            ),
            0,
            "topology-only algebraic chords must not require coordinate tangent reconstruction: {trace:?}",
        );
    }
    assert_eq!(
        offset.certainty,
        CurveCertainty::Certified,
        "strict-compatible retained evidence must not consume the approximate terminal"
    );
    assert!(!offset.value.is_empty());
}

#[test]
fn pair_native_boolean_boundary_offsets_exactly_strict_forward() {
    assert_pair_native_boolean_boundary_offsets_exactly(CurveContext::STRICT, false);
}

#[test]
fn pair_native_boolean_boundary_offsets_exactly_strict_reversed() {
    assert_pair_native_boolean_boundary_offsets_exactly(CurveContext::STRICT, true);
}

#[test]
fn pair_native_boolean_boundary_offset_remains_certified_under_approximate_policy_forward() {
    assert_pair_native_boolean_boundary_offsets_exactly(CurveContext::APPROXIMATE_512, false);
}

#[test]
fn pair_native_boolean_boundary_offset_remains_certified_under_approximate_policy_reversed() {
    assert_pair_native_boolean_boundary_offsets_exactly(CurveContext::APPROXIMATE_512, true);
}

#[test]
fn pair_native_boolean_corner_publishes_a_third_generation_fillet() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let pair_native = independent_pair_native_fillet(&policy, reversed);
            let cutter = pair_native_crossing_cutter(&pair_native, false, &policy);
            let booleans = pair_native
                .boolean_regions_with_policy(&cutter, &policy)
                .unwrap_or_else(|error| {
                    panic!(
                        "the pair-native cutter must publish its retained corner: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(booleans.certainty, CurveCertainty::Certified);
            let clipped = booleans.value.intersection();
            let fragment_kinds = clipped
                .boundary_loops()
                .iter()
                .map(|boundary| {
                    boundary
                        .fragments()
                        .iter()
                        .map(|fragment| match fragment {
                            BezierSplitFragment2::Materialized { .. } => "materialized",
                            BezierSplitFragment2::RetainedBezier { .. } => "endpoint-images",
                            BezierSplitFragment2::AnalyticParallel(_) => "parallel",
                            BezierSplitFragment2::AlgebraicChord(_) => "chord",
                            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                                if fragment.semicircle().uses_selected_radial_frame() =>
                            {
                                "pair-circle"
                            }
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_) => "circle",
                            BezierSplitFragment2::SelectedFiber(_) => "selected-fiber",
                        })
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>();
            let (loop_index, corner, parent_radius) = clipped
                .boundary_loops()
                .iter()
                .enumerate()
                .find_map(|(loop_index, boundary)| {
                    let fragments = boundary.fragments();
                    (0..fragments.len()).find_map(|corner| {
                        let previous = &fragments[(corner + fragments.len() - 1) % fragments.len()];
                        let next = &fragments[corner];
                        let pair_radius = |fragment: &BezierSplitFragment2| match fragment {
                            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                                if fragment.semicircle().uses_selected_radial_frame() =>
                            {
                                Some(fragment.semicircle().radial_distance().abs())
                            }
                            _ => None,
                        };
                        let retained_line = |fragment: &BezierSplitFragment2| {
                            matches!(
                                fragment,
                                BezierSplitFragment2::AlgebraicChord(_)
                                    | BezierSplitFragment2::Materialized { .. }
                            )
                        };
                        if retained_line(previous) {
                            pair_radius(next)
                        } else if retained_line(next) {
                            pair_radius(previous)
                        } else {
                            None
                        }
                        .map(|radius| (loop_index, corner, radius))
                    })
                })
                .unwrap_or_else(|| {
                    panic!(
                        "the clipped region retains a pair-radial/line corner: {fragment_kinds:?}"
                    )
                });
            let radius = (parent_radius / Real::from(100_i16)).unwrap();
            let chamfer = clipped
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    loop_index,
                    corner,
                    radius.clone(),
                    radius.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the retained pair-radial/line corner must chamfer exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(chamfer.certainty, CurveCertainty::Certified);
            for_each_corner_region(corner_regions(&chamfer.value), |chamfered| {
                assert!(
                    chamfered
                        .boundary_loops()
                        .iter()
                        .flat_map(|boundary| boundary.fragments())
                        .any(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                                if fragment.semicircle().uses_selected_radial_frame()
                        )),
                    "the chamfer must retain its pair-native circular parent"
                );
                let replay = chamfered
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the third-generation chamfer re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert!(replay.value.intersection().is_empty());
                assert_eq!(replay.value.union().boundary_loops().len(), 2);
            });
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let offset_distance = (radius.clone() / Real::from(2_i8)).unwrap();
            #[cfg(feature = "dispatch-trace")]
            let offset_result = hyperreal::dispatch_trace::with_recording(|| {
                clipped.offset_with_policy(offset_distance, &OffsetCornerStyle2::Bevel, &policy)
            });
            #[cfg(not(feature = "dispatch-trace"))]
            let offset_result =
                clipped.offset(offset_distance, &OffsetCornerStyle2::Bevel, &policy);
            #[cfg(feature = "dispatch-trace")]
            if let Err(error) = &offset_result {
                panic!(
                    "the retained pair-native Boolean boundary must offset exactly: policy={policy:?}, reversed={reversed}, error={error:?}, trace={:?}",
                    hyperreal::dispatch_trace::take_trace(),
                );
            }
            let offset = offset_result.unwrap_or_else(|error| {
                    panic!(
                        "the retained pair-native Boolean boundary must offset exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(
                offset.certainty,
                CurveCertainty::Certified,
                "strict-compatible retained evidence must not consume the approximate terminal"
            );
            assert!(!offset.value.is_empty());
            assert!(
                offset
                    .value
                    .boundary_loops()
                    .iter()
                    .flat_map(|boundary| boundary.fragments())
                    .any(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                            if fragment.semicircle().uses_selected_radial_frame()
                    )),
                "the offset must retain its pair-native circular authority"
            );
            let result = clipped
                .fillet_loop_vertex_with_policy(
                    loop_index,
                    corner,
                    &crate::CurveFillet2::new(radius),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the retained pair-radial/line corner must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            for_each_corner_region(fillet_regions(&result.value), |filleted| {
                let fragments = filleted
                    .boundary_loops()
                    .iter()
                    .flat_map(|boundary| boundary.fragments());
                assert!(
                    fragments
                        .clone()
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        ))
                        .count()
                        >= 2,
                    "the recursively authored fillet and its circular parent must both remain exact"
                );
                assert!(
                    fragments
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                                if fragment.semicircle().uses_selected_radial_frame()
                        ))
                        .count()
                        >= 1,
                    "the original pair-native carrier must retain its pair-contact authority"
                );
                let replay = filleted
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the third-generation fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert!(replay.value.intersection().is_empty());
                assert_eq!(replay.value.union().boundary_loops().len(), 2);
            });
        }
    }
}

#[test]
fn pair_native_boolean_analytic_corner_publishes_a_third_generation_fillet() {
    for reversed in [false, true] {
        let construction_policy = CurveContext::STRICT;
        let pair_native = independent_pair_native_fillet(&construction_policy, reversed);
        let cutter = pair_native_crossing_cutter(&pair_native, true, &construction_policy);
        let clipped = pair_native
            .boolean_regions_with_policy(&cutter, &construction_policy)
            .expect("the analytic cutter must publish its retained corner")
            .into_value()
            .intersection()
            .clone();
        let (loop_index, corner, parent_radius) = clipped
            .boundary_loops()
            .iter()
            .enumerate()
            .find_map(|(loop_index, boundary)| {
                let fragments = boundary.fragments();
                (0..fragments.len()).find_map(|corner| {
                    let previous = &fragments[(corner + fragments.len() - 1) % fragments.len()];
                    let next = &fragments[corner];
                    let pair_radius = |fragment: &BezierSplitFragment2| match fragment {
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                            if fragment.semicircle().uses_selected_radial_frame() =>
                        {
                            Some(fragment.semicircle().radial_distance().abs())
                        }
                        _ => None,
                    };
                    let analytic = |fragment: &BezierSplitFragment2| {
                        matches!(fragment, BezierSplitFragment2::AnalyticParallel(_))
                            || matches!(
                                fragment,
                                BezierSplitFragment2::SelectedFiber(fragment)
                                    if fragment.analytic_parallel().is_some()
                            )
                    };
                    if analytic(previous) {
                        pair_radius(next)
                    } else if analytic(next) {
                        pair_radius(previous)
                    } else {
                        None
                    }
                    .map(|radius| (loop_index, corner, radius))
                })
            })
            .expect("the clipped region retains a pair-radial/analytic corner");
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let result = clipped
                .fillet_loop_vertex_with_policy(
                    loop_index,
                    corner,
                    &crate::CurveFillet2::new(
                        (parent_radius.clone() / Real::from(100_i16)).unwrap(),
                    ),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the pair-radial/analytic corner must fillet exactly");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(!result.value.solutions().is_empty());
            for_each_corner_region(fillet_regions(&result.value), |filleted| {
                let replay = filleted
                    .boolean_regions_with_policy(
                        &selected_fillet_disjoint_square(&CurveContext::STRICT),
                        &CurveContext::STRICT,
                    )
                    .expect("a certified pair-radial/analytic fillet re-enters the strict Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(
                    replay.value.union().boundary_loops().len(),
                    filleted.boundary_loops().len() + 1
                );
                assert!(replay.value.intersection().is_empty());
            });
        }
    }
}

#[test]
fn pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet() {
    for reversed in [false, true] {
        let construction_policy = CurveContext::STRICT;
        let pair_native = independent_pair_native_fillet(&construction_policy, reversed);
        let exact_cutter = pair_native_crossing_cutter(&pair_native, false, &construction_policy);
        let exact_fragments = exact_cutter.boundary_loops()[0].fragments();
        let vertices = exact_fragments
            .iter()
            .map(|fragment| {
                let Classification::Decided(Some(point)) =
                    curve_fragment_endpoint_point(fragment, true, &construction_policy)
                        .expect("the reference cutter endpoint is exact")
                else {
                    panic!("the reference cutter endpoint has exact evidence");
                };
                let Classification::Decided(point) =
                    retained_native_line_point(&point, &construction_policy)
                        .expect("the reference cutter endpoint projects exactly")
                else {
                    panic!("the reference cutter endpoint is represented");
                };
                point
            })
            .collect::<Vec<_>>();
        let parameter = positive_inverse_sqrt_parameter(2, &construction_policy);
        let BezierParameter2::Algebraic(parameter) = parameter else {
            panic!("the translated cutter parameter remains algebraic");
        };
        let source = RationalBezier2::try_new(
            vec![
                vertices[0].clone(),
                vertices[0].translated(Real::zero(), Real::one()),
            ],
            vec![Real::one(); 2],
        )
        .expect("the translated cutter point source is rational");
        let selected = CurvePoint2::from(crate::tests::decided(
            source
                .point_at_algebraic_parameter(&parameter, &construction_policy)
                .expect("the translated cutter point is exact"),
        ));
        let selected_vertices = vertices
            .iter()
            .map(|vertex| {
                retained_corner_decision(
                    crate::BezierAlgebraicChord2::translated_endpoint(
                        &selected,
                        &(vertex.x() - vertices[0].x()),
                        &(vertex.y() - vertices[0].y()),
                        &construction_policy,
                    )
                    .expect("the selected cutter vertex translates exactly"),
                    CurveOperation2::Boolean,
                )
                .expect("the selected cutter vertex is decided")
            })
            .collect::<Vec<_>>();
        let fragments = (0..selected_vertices.len())
            .map(|index| {
                let chord = retained_corner_decision(
                    crate::BezierAlgebraicChord2::try_new(
                        selected_vertices[index].clone(),
                        selected_vertices[(index + 1) % selected_vertices.len()].clone(),
                        &construction_policy,
                    )
                    .expect("the selected cutter chord is valid"),
                    CurveOperation2::Boolean,
                )
                .expect("the selected cutter chord is decided");
                BezierSplitFragment2::AlgebraicChord(chord)
            })
            .collect();
        let cutter = CurveRegion2::try_new_with_loop_topology(
            vec![
                CurveRegionBoundaryLoop2::new(fragments, &construction_policy)
                    .expect("the selected cutter closes exactly"),
            ],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .expect("the selected cutter has authored topology");
        let clipped = pair_native
            .boolean_regions_with_policy(&cutter, &construction_policy)
            .expect("the selected cutter must publish its retained corner")
            .into_value()
            .intersection()
            .clone();
        let (loop_index, corner, parent_radius) = clipped
            .boundary_loops()
            .iter()
            .enumerate()
            .find_map(|(loop_index, boundary)| {
                let fragments = boundary.fragments();
                (0..fragments.len()).find_map(|corner| {
                    let previous = &fragments[(corner + fragments.len() - 1) % fragments.len()];
                    let next = &fragments[corner];
                    let pair_radius = |fragment: &BezierSplitFragment2| match fragment {
                        BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                            if fragment.semicircle().uses_selected_radial_frame() =>
                        {
                            Some(fragment.semicircle().radial_distance().abs())
                        }
                        _ => None,
                    };
                    let chord = |fragment: &BezierSplitFragment2| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicChord(_))
                    };
                    if chord(previous) {
                        pair_radius(next)
                    } else if chord(next) {
                        pair_radius(previous)
                    } else {
                        None
                    }
                    .map(|radius| (loop_index, corner, radius))
                })
            })
            .expect("the clipped region retains a pair-radial/algebraic-chord corner");
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let result = clipped
                .fillet_loop_vertex_with_policy(
                    loop_index,
                    corner,
                    &crate::CurveFillet2::new(
                        (parent_radius.clone() / Real::from(100_i16)).unwrap(),
                    ),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the pair-radial/algebraic-chord corner must fillet exactly");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(!result.value.solutions().is_empty());
            for_each_corner_region(fillet_regions(&result.value), |filleted| {
                let replay = filleted
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the pair-radial/algebraic-chord fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(
                    replay.value.union().boundary_loops().len(),
                    filleted.boundary_loops().len() + 1
                );
                assert!(replay.value.intersection().is_empty());
            });
        }
    }
}

#[test]
fn translated_pair_native_circles_fillet_after_boolean_crossing() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let filleted = independent_pair_native_fillet(&policy, reversed);
            let circle = filleted.boundary_loops()[0]
                .fragments()
                .iter()
                .find_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                        if fragment.semicircle().uses_selected_radial_frame() =>
                    {
                        Some(fragment.semicircle().clone())
                    }
                    _ => None,
                })
                .expect("the source fillet retains its pair-native circle");
            let radius = circle.radial_distance().abs();
            let boundary = CurveRegionBoundaryLoop2::new(
                vec![
                    BezierSplitFragment2::AlgebraicCuspSemicircle(
                        crate::BezierAlgebraicCuspSemicircleFragment2::full(
                            circle.clone(),
                            &policy,
                        ),
                    ),
                    BezierSplitFragment2::AlgebraicCuspSemicircle(
                        crate::BezierAlgebraicCuspSemicircleFragment2::full(
                            circle.complementary_half(),
                            &policy,
                        ),
                    ),
                ],
                &policy,
            )
            .expect("the pair-native disk closes exactly");
            let disk = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if circle.is_clockwise() {
                    CurveBoundaryInteriorSide2::Right
                } else {
                    CurveBoundaryInteriorSide2::Left
                }],
            )
            .expect("the pair-native disk has authored topology");
            let transform = crate::Similarity2::try_from_real_affine(
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                (&radius / Real::from(2_i8)).unwrap(),
                Real::zero(),
            )
            .expect("the pair-native disk translation is a similarity");
            let shifted = disk
                .transform_similarity_with_policy(&transform, &policy)
                .expect("the second pair-native disk translates exactly")
                .into_value();
            let lens = disk
                .boolean_region_with_policy(&shifted, BooleanOp::Intersection, &policy)
                .expect("translated pair-native disks must intersect exactly")
                .into_value();
            let (loop_index, corner) = pair_radial_crossing_corner(&lens, &policy);
            let result = lens
                .fillet_loop_vertex_with_policy(
                    loop_index,
                    corner,
                    &crate::CurveFillet2::new((radius / Real::from(10_i8)).unwrap()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the translated pair-native circle corner must fillet exactly");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(
                !result.value.solutions().is_empty(),
                "the crossing pair-native circles must fillet across their smooth chart seam: {:?}",
                result.value
            );
            let source_fragments = lens.boundary_loops()[loop_index].fragments();
            let consumes_smooth_seam = |filleted: &CurveRegion2| {
                let retained = source_fragments
                    .iter()
                    .filter(|source| {
                        filleted.boundary_loops().iter().any(|boundary| {
                            boundary
                                .fragments()
                                .iter()
                                .any(|fragment| fragment == *source)
                        })
                    })
                    .count();
                retained < source_fragments.len() - 2
            };
            assert!(
                { result.value.solutions().iter().any(consumes_smooth_seam) },
                "at least one exact candidate must consume more than the two incident fragments"
            );
            for_each_corner_region(fillet_regions(&result.value), |filleted| {
                let replay = filleted
                    .boolean_regions_with_policy(
                        &selected_fillet_disjoint_square(&CurveContext::STRICT),
                        &CurveContext::STRICT,
                    )
                    .expect("a certified translated pair-radial fillet re-enters the strict Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(
                    replay.value.union().boundary_loops().len(),
                    filleted.boundary_loops().len() + 1
                );
                assert!(replay.value.intersection().is_empty());
            });
        }
    }
}

#[test]
fn selected_circle_and_analytic_parallel_fillet_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (curved, reversed) in [false, true]
            .into_iter()
            .flat_map(|curved| [false, true].map(|reversed| (curved, reversed)))
        {
            let region = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::AnalyticParallel(curved),
                reversed,
            );
            let fragments = region.boundary_loops()[0].fragments();
            let corner = (0..fragments.len())
                .find(|index| {
                    let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
                    let next = &fragments[*index];
                    matches!(
                        (previous, next),
                        (
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                            BezierSplitFragment2::AnalyticParallel(_)
                        ) | (
                            BezierSplitFragment2::AnalyticParallel(_),
                            BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        )
                    )
                })
                .expect("the fixture retains its selected-circle/analytic corner");
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new((Real::one() / Real::from(10_i8)).unwrap()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the selected-circle/analytic corner must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let filleted = {
                let solutions = result.value;
                let mut candidates = solutions.into_solutions();
                assert_eq!(candidates.len(), 1, "expected one isolated fillet");
                candidates.pop().unwrap()
            };
            assert_eq!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    ))
                    .count(),
                2,
            );
            assert_eq!(
                filleted
                    .classify_point_with_policy(&p(0, 0).into(), &policy)
                    .expect("the selected-circle/analytic fillet remains classifiable")
                    .into_value(),
                Classification::Decided(RegionPointLocation::Inside),
            );
            assert_eq!(
                filleted
                    .classify_point_with_policy(&p(-1, 0).into(), &policy)
                    .expect("the selected-circle/analytic exterior remains classifiable")
                    .into_value(),
                Classification::Decided(RegionPointLocation::Outside),
            );
            if !curved {
                let replay = filleted
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the selected-circle/analytic fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert_eq!(replay.value.union().boundary_loops().len(), 2);
                assert!(replay.value.intersection().is_empty());
            }
        }
    }
}

#[test]
fn selected_parallel_ray_vertices_use_spatial_endpoint_ownership() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let center_support =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(-1, 0), p(-1, -1)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let Classification::Decided(Some(circle)) =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                center_support,
                BezierParameter2::Exact(Real::zero()).into(),
                Real::one(),
                false,
                &policy,
            )
            .unwrap()
        else {
            panic!("the regular exact circle frame must construct")
        };
        let Classification::Decided(quarter) =
            crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle.complementary_half(),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(q(1, 2)),
                false,
                &policy,
            )
            .unwrap()
        else {
            panic!("the lower-left quarter must construct")
        };
        let parallel =
            QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(-1, -1), p(0, 0)).unwrap())
                .parallel_left(Real::zero())
                .unwrap();
        let selected = BezierSplitFragment2::SelectedFiber(
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel),
                CurveParameterRange2::unit(),
                p(-1, -1).into(),
                p(0, 0).into(),
            ),
        );
        // Three quarters of the unit circle centered at (-1,0), closed
        // by its diagonal chord. The entire boundary lies at x <= 0.
        let fragments = vec![
            BezierSplitFragment2::AlgebraicCuspSemicircle(
                crate::BezierAlgebraicCuspSemicircleFragment2::full(circle, &policy),
            ),
            BezierSplitFragment2::AlgebraicCuspSemicircle(quarter),
            selected,
        ];
        for reversed in [false, true] {
            let fragments = if reversed {
                fragments
                    .iter()
                    .rev()
                    .map(|fragment| fragment.reversed().unwrap())
                    .collect()
            } else {
                fragments.clone()
            };
            let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
            let origin = p(1, 0);
            let ray = ray_candidates(&origin).remove(0);
            assert_eq!(
                classify_point_with_retained_ray_skipping_origin(
                    &boundary, &origin, &ray, None, &policy
                )
                .unwrap(),
                Classification::Decided(RetainedRayWinding::Winding(0)),
                "the left ray crosses both the arc and closing chord: reversed={reversed}, policy={policy:?}"
            );
            for (point, expected) in [
                (origin, ContourPointLocation::Outside),
                (p(-1, 0), ContourPointLocation::Inside),
                (p(0, 0), ContourPointLocation::Boundary),
            ] {
                assert_eq!(
                    boundary.classify_point_raw(&point, &policy).unwrap(),
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn selected_circle_and_analytic_parallel_extend_on_full_supports() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for curved in [false, true] {
            for reversed in [false, true] {
                let region = selected_circle_neighbor_region(
                    &policy,
                    SelectedCircleFilletNeighbor2::AnalyticParallel(curved),
                    reversed,
                );
                let fragments = region.boundary_loops()[0].fragments();
                let corner = (0..fragments.len())
                    .find(|index| {
                        let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
                        let next = &fragments[*index];
                        matches!(
                            (previous, next),
                            (
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                                BezierSplitFragment2::AnalyticParallel(_)
                            ) | (
                                BezierSplitFragment2::AnalyticParallel(_),
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            )
                        )
                    })
                    .expect("the fixture retains its selected-circle/analytic corner");
                let trim = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(q(1, 10)),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .expect("the finite selected-circle/analytic corner remains supported");
                let extended = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        corner,
                        &crate::CurveFillet2::new(q(1, 10)),
                        CurveCornerMode2::TrimOrExtend,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "the selected-circle/analytic supports must extend exactly: policy={policy:?}, curved={curved}, reversed={reversed}, error={error:?}"
                        )
                    });
                assert_eq!(extended.certainty, CurveCertainty::Certified);
                assert!(
                    extended.value.solutions().len() > trim.value.solutions().len(),
                    "the full circle and analytic incident ray must add exterior centers"
                );
                for_each_corner_region(fillet_regions(&extended.value), |filleted| {
                    assert!(filleted.boundary_loops().iter().any(|boundary| {
                        boundary.fragments().iter().any(|fragment| {
                            matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                        })
                    }));
                    assert_eq!(
                        filleted
                            .classify_point_with_policy(&p(0, 0).into(), &policy)
                            .expect("the extended analytic fillet remains classifiable")
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                    );
                    if !curved && !reversed {
                        let replay = filleted
                            .boolean_regions_with_policy(
                                &selected_fillet_disjoint_square(&policy),
                                &policy,
                            )
                            .expect("the extended analytic fillet re-enters the Boolean kernel");
                        assert_eq!(replay.certainty, CurveCertainty::Certified);
                        assert_disjoint_square_replay_preserves_set(
                            filleted,
                            &replay.value,
                            &policy,
                        );
                    }
                });
            }
        }
    }
}

#[test]
fn selected_circle_and_direct_bezier_share_the_parallel_fillet_kernel() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = selected_circle_neighbor_region(
                &policy,
                SelectedCircleFilletNeighbor2::DirectBezier,
                reversed,
            );
            let fragments = region.boundary_loops()[0].fragments();
            let corner = if reversed { 2 } else { 1 };
            let previous = &fragments[(corner + fragments.len() - 1) % fragments.len()];
            let next = &fragments[corner];
            assert!(matches!(
                (previous, next),
                (
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                    BezierSplitFragment2::Materialized {
                        curve: BezierSubcurve2::Quadratic(_),
                        ..
                    }
                ) | (
                    BezierSplitFragment2::Materialized {
                        curve: BezierSubcurve2::Quadratic(_),
                        ..
                    },
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                )
            ));
            let result = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new((Real::one() / Real::from(10_i8)).unwrap()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the selected-circle/direct-Bezier corner must fillet exactly: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let filleted = {
                let solutions = result.value;
                let mut candidates = solutions.into_solutions();
                assert_eq!(candidates.len(), 1, "expected one isolated fillet");
                candidates.pop().unwrap()
            };
            assert_eq!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    ))
                    .count(),
                2,
            );
            assert!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(retained_rational_fragment_has_algebraic_endpoint)
            );
            if policy == CurveContext::STRICT && !reversed {
                assert_eq!(
                    filleted
                        .classify_point_with_policy(&p(0, 0).into(), &policy)
                        .expect("the retained direct-Bezier fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
            }
        }
    }
}

fn positive_inverse_sqrt_parameter(denominator: i8, policy: &CurveContext) -> BezierParameter2 {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        vec![-Real::one(), Real::zero(), Real::from(denominator)],
        policy,
    )
    .unwrap()
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => panic!("inverse-square polynomial: {reason:?}"),
    };
    let roots = match polynomial.isolate_unit_interval_roots(policy).unwrap() {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => panic!("inverse-square root: {reason:?}"),
    };
    let [parameter] = roots.as_slice() else {
        panic!("one positive inverse-square root must lie in the unit interval");
    };
    parameter.clone()
}

#[test]
fn algebraic_query_winding_mixes_genuine_parallel_and_native_fragments() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(half, Real::zero()), p(1, 1));
    let parallel = source.parallel_left(quarter).unwrap();
    let query_curve =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 1)], vec![Real::one(), Real::one()]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            parallel.exact_rational_parallel_component(&policy).unwrap(),
            Classification::Decided(None)
        ));
        let parameter = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(parameter) = parameter else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let query = crate::tests::decided(
            query_curve
                .point_at_algebraic_parameter(&parameter, &policy)
                .unwrap(),
        );
        let Classification::Decided(query) = query.predicate_evaluator(&policy).unwrap() else {
            panic!("the algebraic query predicate must construct");
        };
        let zero = Real::zero();
        let one = Real::one();
        let Classification::Decided(start) = parallel.point_at(&zero, &policy).unwrap() else {
            panic!("the genuine parallel start must evaluate");
        };
        let Classification::Decided(end) = parallel.point_at(&one, &policy).unwrap() else {
            panic!("the genuine parallel end must evaluate");
        };
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(zero.clone()),
            BezierParameter2::Exact(one.clone()),
        );
        let Classification::Decided(analytic) =
            crate::BezierParallelFragment2::try_new(parallel.clone(), range, &policy).unwrap()
        else {
            panic!("the genuine parallel fragment must construct");
        };
        let corner = p(2, 0);
        let closure = |start: Point2, end: Point2| BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(zero.clone()),
            end: BezierParameter2::Exact(one.clone()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(start, end).unwrap(),
            )),
        };
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AnalyticParallel(analytic),
                closure(end, corner.clone()),
                closure(corner, start),
            ],
            &policy,
        )
        .unwrap();
        let Classification::Decided(fragments) =
            prepare_algebraic_ray_retained_fragments(&boundary, &policy).unwrap()
        else {
            panic!("the mixed algebraic ray fragments must prepare");
        };
        let [
            AlgebraicRayRetainedFragment2::AnalyticParallel(analytic),
            AlgebraicRayRetainedFragment2::Rational(first_closure),
            AlgebraicRayRetainedFragment2::Rational(second_closure),
        ] = fragments.as_slice()
        else {
            panic!("the mixed loop must retain analytic and rational evaluators");
        };
        assert_eq!(
            analytic.contains_point(&query, None, &policy).unwrap(),
            Classification::Decided(false),
        );
        assert_eq!(
            algebraic_point_on_rational_fragment(first_closure, &query, &policy).unwrap(),
            Classification::Decided(false),
        );
        assert_eq!(
            algebraic_point_on_rational_fragment(second_closure, &query, &policy).unwrap(),
            Classification::Decided(false),
        );
        assert_eq!(
            algebraic_ray_retained_fragments_admit_direction(
                &fragments,
                &query,
                &Real::zero(),
                &Real::one(),
                &policy,
            )
            .unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            algebraic_ray_retained_fragments_winding(
                &fragments,
                &query,
                &Real::one(),
                &Real::zero(),
                None,
                false,
                &policy,
            )
            .unwrap(),
            Classification::Decided(0),
        );
        assert_eq!(
            classify_algebraic_point_against_retained_loop(
                &boundary,
                &query,
                FillRule::NonZero,
                true,
                &policy,
            )
            .unwrap(),
            Classification::Decided(ContourPointLocation::Outside),
        );
    }
}

#[test]
fn algebraic_side_ray_skips_coincident_chords_across_loops() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let quarter = q(1, 4);
        let image = |start_x: Real, end_x: Real| {
            crate::tests::decided(
                RationalBezier2::try_new(
                    vec![
                        Point2::new(start_x, Real::zero()),
                        Point2::new(end_x, Real::zero()),
                    ],
                    vec![Real::one(); 2],
                )
                .expect("the affine algebraic point carrier is finite")
                .point_at_algebraic_parameter(alpha_root, &policy)
                .expect("the affine algebraic point image is exact"),
            )
        };
        let start = CurvePoint2::from(image(-quarter.clone(), Real::from(3_i8) * &quarter));
        let query_image = image(Real::zero(), Real::one());
        let end = CurvePoint2::from(image(quarter.clone(), Real::from(5_i8) * &quarter));
        let Classification::Decided(query) = query_image.predicate_evaluator(&policy).unwrap()
        else {
            panic!("the algebraic side-ray origin predicate must construct");
        };
        let Classification::Decided(chord) =
            crate::BezierAlgebraicChord2::try_new(start, end, &policy).unwrap()
        else {
            panic!("the straddling algebraic chord must construct");
        };
        assert_eq!(
            chord.contains_algebraic_point(&query, &policy).unwrap(),
            Classification::Decided(true),
        );
        let fragments = vec![
            AlgebraicRayRetainedFragment2::AlgebraicChord(chord.clone()),
            AlgebraicRayRetainedFragment2::AlgebraicChord(chord),
        ];
        assert_eq!(
            algebraic_ray_retained_fragments_admit_direction(
                &fragments,
                &query,
                &-Real::one(),
                &Real::zero(),
                &policy,
            )
            .unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            algebraic_ray_retained_fragments_winding(
                &fragments,
                &query,
                &Real::zero(),
                &Real::one(),
                Some(0),
                false,
                &policy,
            )
            .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary),
        );
        assert_eq!(
            algebraic_ray_retained_fragments_winding(
                &fragments,
                &query,
                &Real::zero(),
                &Real::one(),
                Some(0),
                true,
                &policy,
            )
            .unwrap(),
            Classification::Decided(0),
        );
    }
}

#[test]
fn algebraic_side_ray_skips_retained_rational_contacts_across_loops() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let line = RationalBezier2::try_new(vec![p(0, 0), p(1, 0)], vec![Real::one(), Real::one()])
            .unwrap();
        let query_image = crate::tests::decided(
            line.point_at_algebraic_parameter(alpha_root, &policy)
                .unwrap(),
        );
        let Classification::Decided(query) = query_image.predicate_evaluator(&policy).unwrap()
        else {
            panic!("the algebraic side-ray origin predicate must construct");
        };
        let fragment = AlgebraicRayRationalFragment2 {
            curve: line,
            retained_range: Some(CurveParameterRange2::from_bezier_range(
                BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            )),
            reversed: false,
        };
        assert_eq!(
            algebraic_point_on_rational_fragment(&fragment, &query, &policy).unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            algebraic_point_rational_curve_ray_winding(
                &fragment,
                &query,
                &Real::zero(),
                &Real::one(),
                &policy,
            )
            .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary),
        );
        assert_eq!(
            algebraic_point_rational_curve_ray_winding_skipping_incident_origin(
                &fragment,
                &query,
                &Real::zero(),
                &Real::one(),
                &policy,
            )
            .unwrap(),
            Classification::Decided(Some(0)),
        );
    }
}

#[test]
fn algebraic_side_ray_skips_genuine_parallel_contacts_across_loops() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(half.clone(), Real::zero()), p(1, 1));
    let distance = Real::from(3_i8).sqrt().unwrap();
    let parallel = source.parallel_left(distance).unwrap();
    let query_curve = RationalBezier2::try_new(
        vec![p(0, 1), Point2::new(-half, Real::one()), p(-1, 2)],
        vec![Real::one(); 3],
    )
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            parallel.exact_rational_parallel_component(&policy).unwrap(),
            Classification::Decided(None),
        );
        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let query_image = crate::tests::decided(
            query_curve
                .point_at_algebraic_parameter(alpha_root, &policy)
                .unwrap(),
        );
        let Classification::Decided(query) = query_image.predicate_evaluator(&policy).unwrap()
        else {
            panic!("the genuine-parallel side-ray predicate must construct");
        };
        let endpoint = |parameter: Real| {
            let Classification::Decided(point) = parallel.point_at(&parameter, &policy).unwrap()
            else {
                panic!("the genuine parallel endpoint must evaluate");
            };
            CurvePoint2::from(point)
        };
        let Classification::Decided(evaluator) =
            crate::bezier_offset::BezierParallelAlgebraicRay2::try_new(
                parallel.clone(),
                CurveParameterRange2::from_bezier_range(BezierParameterRange2::from_exact(
                    Real::zero(),
                    Real::one(),
                )),
                false,
                [endpoint(Real::zero()), endpoint(Real::one())],
                &policy,
            )
            .unwrap()
        else {
            panic!("the genuine-parallel algebraic ray must construct");
        };
        assert_eq!(
            evaluator.contains_point(&query, None, &policy).unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta(&query, &Real::zero(), &Real::one(), &policy)
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(0)),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::from(-1_i8),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(-1)),
        );
        assert_eq!(
            algebraic_ray_retained_fragments_winding(
                &[AlgebraicRayRetainedFragment2::AnalyticParallel(evaluator)],
                &query,
                &Real::zero(),
                &Real::from(-1_i8),
                None,
                true,
                &policy,
            )
            .unwrap(),
            Classification::Decided(-1),
        );
    }
}

#[test]
fn boundary_side_rays_skip_retained_cusp_contacts_across_loops() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (_, represented_center) = selected_circle_fixture_center(&policy);
        let Classification::Decided(Some(represented_support)) =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &represented_center,
                (1, 0),
                Real::one(),
                true,
                &policy,
            )
            .unwrap()
        else {
            panic!("the represented selected semicircle must construct");
        };
        let represented = crate::BezierAlgebraicCuspSemicircleFragment2::full(
            represented_support.clone(),
            &policy,
        );
        let query = Point2::new((Real::one() / Real::from(2_i8)).unwrap(), Real::from(-1_i8));
        assert_eq!(
            represented
                .contains_point(&CurvePoint2::from(query.clone()), &policy)
                .unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            represented
                .forward_ray_winding_delta(&query, &Real::zero(), &Real::one(), &policy,)
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary),
        );
        assert_eq!(
            represented
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(0)),
        );
        assert_eq!(
            represented
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::from(2_i8),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(-1)),
        );
        assert_eq!(
            represented
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::from(-1_i8),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(0)),
        );
        let represented_complement = crate::BezierAlgebraicCuspSemicircleFragment2::full(
            represented_support.complementary_half(),
            &policy,
        );
        assert_eq!(
            represented_complement
                .contains_point(&CurvePoint2::from(query.clone()), &policy)
                .unwrap(),
            Classification::Decided(false),
        );
        assert_eq!(
            represented_complement
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(None),
        );
        let boundary = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicCuspSemicircle(represented),
                BezierSplitFragment2::AlgebraicCuspSemicircle(represented_complement),
            ],
            &policy,
        )
        .unwrap();
        let ray = BezierRay2 {
            line: LineSeg2::try_new(query.clone(), Point2::new(query.x().clone(), Real::zero()))
                .unwrap(),
            direction_x: Real::zero(),
            direction_y: Real::one(),
        };
        assert_eq!(
            classify_point_with_retained_ray_skipping_origin(
                &boundary,
                &query,
                &ray,
                Some(RetainedRayOriginContact {
                    fragment_index: None,
                    parameter: None,
                    crossing_direction: BezierLineCrossingDirection::PositiveToNegative,
                    tangent_contacts: None,
                }),
                &policy,
            )
            .unwrap(),
            Classification::Decided(RetainedRayWinding::Winding(-1)),
        );
        let source_parameter = CurveParameter2::from_algebraic_cusp(
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                (Real::one() / Real::from(2_i8)).unwrap(),
            ),
        );
        let oblique_ray = BezierRay2 {
            line: LineSeg2::try_new(
                query.clone(),
                Point2::new(query.x() + Real::from(2_i8), Real::zero()),
            )
            .unwrap(),
            direction_x: Real::from(2_i8),
            direction_y: Real::one(),
        };
        assert_eq!(
            classify_point_with_retained_ray_skipping_origin(
                &boundary,
                &query,
                &oblique_ray,
                Some(RetainedRayOriginContact {
                    fragment_index: Some(0),
                    parameter: Some(&source_parameter),
                    crossing_direction: BezierLineCrossingDirection::NegativeToPositive,
                    tangent_contacts: None,
                }),
                &policy,
            )
            .unwrap(),
            Classification::Decided(RetainedRayWinding::Winding(-1)),
        );

        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let image = |height: i8| {
            crate::tests::decided(
                RationalBezier2::try_new(
                    vec![p(0, i32::from(height)), p(1, i32::from(height))],
                    vec![Real::one(), Real::one()],
                )
                .unwrap()
                .point_at_algebraic_parameter(alpha_root, &policy)
                .unwrap(),
            )
        };
        let center = CurvePoint2::from(image(0));
        let query_image = image(-1);
        let Classification::Decided(query) = query_image.predicate_evaluator(&policy).unwrap()
        else {
            panic!("the algebraic cusp query predicate must construct");
        };
        let Classification::Decided(Some(support)) =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::one(),
                true,
                &policy,
            )
            .unwrap()
        else {
            panic!("the algebraic selected semicircle must construct");
        };
        let fragment = crate::BezierAlgebraicCuspSemicircleFragment2::full(support, &policy);
        let Classification::Decided(evaluator) = fragment.algebraic_ray_evaluator(&policy).unwrap()
        else {
            panic!("the algebraic selected semicircle ray must construct");
        };
        assert_eq!(
            evaluator.contains_point(&query, &policy).unwrap(),
            Classification::Decided(true),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(0)),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::from(2_i8),
                    &Real::one(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(-1)),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::zero(),
                    &Real::from(-1_i8),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(Some(0)),
        );
        assert_eq!(
            evaluator
                .forward_ray_winding_delta_skipping_incident_origin(
                    &query,
                    &Real::one(),
                    &Real::zero(),
                    &policy,
                )
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary),
        );
        assert_eq!(
            algebraic_ray_retained_fragments_winding(
                &[AlgebraicRayRetainedFragment2::AlgebraicCusp(evaluator)],
                &query,
                &Real::from(2_i8),
                &Real::one(),
                Some(0),
                true,
                &policy,
            )
            .unwrap(),
            Classification::Decided(-1),
        );
    }
}

fn assert_algebraic_ray_spatial_endpoint_ownership(retained: bool) {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let image = |end: Point2| {
            crate::tests::decided(
                RationalBezier2::try_new(vec![p(0, 0), end], vec![Real::one(); 2])
                    .unwrap()
                    .point_at_algebraic_parameter(alpha_root, &policy)
                    .unwrap(),
            )
        };
        let outside_image = image(p(1, 0));
        let Classification::Decided(query) = outside_image.predicate_evaluator(&policy).unwrap()
        else {
            panic!("the query must keep its selected algebraic root");
        };
        // Every control is at x <= 0, so the left ray from (alpha,0)
        // meets every y=0 contact ahead of its origin. The arch crosses
        // downwards once; its start has no positive (lower) interior side.
        // The tangent start and closing chord own opposite endpoint sides.
        let arch = vec![p(0, 0), p(-4, 2), p(-1, -1)];
        let tangent = vec![p(0, 0), p(-1, 0), p(-2, -1)];
        let chord = vec![p(-1, -1), p(0, 0)];
        // y=(t-1/4)(t-1/2)(t-3/4), x=2-4t. Only the
        // final two roots lie ahead; their opposite crossings cancel.
        // Subdivision cuts exactly through the middle contact.
        let subdivided = vec![
            Point2::new(Real::from(2_i8), q(-3, 32)),
            Point2::new(q(2, 3), q(13, 96)),
            Point2::new(q(-2, 3), q(-13, 96)),
            Point2::new(Real::from(-2_i8), q(3, 32)),
        ];
        for gauge in [Real::one(), -Real::one()] {
            for (controls, expected) in [(&arch, 1), (&tangent, 1), (&chord, -1), (&subdivided, 0)]
            {
                let curve =
                    RationalBezier2::try_new(controls.clone(), vec![gauge.clone(); controls.len()])
                        .unwrap();
                for reversed in [false, true] {
                    let fragment = AlgebraicRayRationalFragment2 {
                        curve: curve.clone(),
                        retained_range: retained.then(CurveParameterRange2::unit),
                        reversed,
                    };
                    assert_eq!(
                        algebraic_point_rational_curve_ray_winding(
                            &fragment,
                            &query,
                            &-Real::one(),
                            &Real::zero(),
                            &policy,
                        )
                        .unwrap(),
                        Classification::Decided(if reversed { -expected } else { expected }),
                        "spatial endpoint ownership: retained={retained}, reversed={reversed}, degree={}, policy={policy:?}",
                        curve.degree(),
                    );
                }
            }
        }
        for reversed in [false, true] {
            let fragments = [arch.clone(), chord.clone()]
                .into_iter()
                .map(|controls| {
                    let curve = BezierSubcurve2::Rational(
                        RationalBezier2::try_new(
                            controls.clone(),
                            vec![Real::one(); controls.len()],
                        )
                        .unwrap(),
                    );
                    if retained {
                        BezierSplitFragment2::RetainedBezier {
                            source_curve: curve,
                            start: BezierParameter2::Exact(Real::zero()),
                            end: BezierParameter2::Exact(Real::one()),
                            start_image: None,
                            end_image: None,
                            reversed: false,
                        }
                    } else {
                        BezierSplitFragment2::Materialized {
                            curve,
                            start: BezierParameter2::Exact(Real::zero()),
                            end: BezierParameter2::Exact(Real::one()),
                        }
                    }
                })
                .collect::<Vec<_>>();
            let fragments = if reversed {
                fragments
                    .into_iter()
                    .rev()
                    .map(|fragment| fragment.reversed().unwrap())
                    .collect()
            } else {
                fragments
            };
            let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
            let Classification::Decided(prepared) =
                prepare_algebraic_ray_retained_fragments(&boundary, &policy).unwrap()
            else {
                panic!("the polynomial loop must prepare")
            };
            assert_eq!(
                algebraic_ray_retained_fragments_winding(
                    &prepared,
                    &query,
                    &-Real::one(),
                    &Real::zero(),
                    None,
                    false,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(0),
                "the left-ray endpoint and interior crossings cancel",
            );
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![boundary],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed {
                    CurveBoundaryInteriorSide2::Right
                } else {
                    CurveBoundaryInteriorSide2::Left
                }],
            )
            .unwrap();
            let value = q(1, 2).sqrt().unwrap();
            for (image, exact, expected) in [
                (
                    outside_image.clone(),
                    Point2::new(value.clone(), Real::zero()),
                    RegionPointLocation::Outside,
                ),
                (
                    image(p(-1, 0)),
                    Point2::new(-value.clone(), Real::zero()),
                    RegionPointLocation::Inside,
                ),
                (
                    image(p(-1, -1)),
                    Point2::new(-value.clone(), -value),
                    RegionPointLocation::Boundary,
                ),
            ] {
                assert_eq!(
                    region
                        .classify_point_with_policy(&image.clone().into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(expected)
                );
                assert_eq!(
                    region
                        .classify_point_with_policy(&exact.clone().into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn algebraic_native_ray_vertices_use_spatial_endpoint_ownership() {
    assert_algebraic_ray_spatial_endpoint_ownership(false);
}

#[test]
fn algebraic_retained_ray_vertices_use_spatial_endpoint_ownership() {
    assert_algebraic_ray_spatial_endpoint_ownership(true);
}

#[test]
fn algebraic_ray_retains_selected_endpoint_ownership() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        for y_sign in [-1, 1] {
            let query_image = crate::tests::decided(
                RationalBezier2::try_new(vec![p(-1, 0), p(-1, y_sign)], vec![Real::one(); 2])
                    .unwrap()
                    .point_at_algebraic_parameter(alpha_root, &policy)
                    .unwrap(),
            );
            let Classification::Decided(query) = query_image.predicate_evaluator(&policy).unwrap()
            else {
                panic!("the query keeps its selected height")
            };
            for gauge in [Real::one(), -Real::one()] {
                let curve =
                    RationalBezier2::try_new(vec![p(0, 0), p(0, y_sign)], vec![gauge; 2]).unwrap();
                for at_start in [false, true] {
                    let range = if at_start {
                        CurveParameterRange2::new_validated(
                            alpha.clone().into(),
                            Real::one().into(),
                        )
                    } else {
                        CurveParameterRange2::new_validated(
                            Real::zero().into(),
                            alpha.clone().into(),
                        )
                    };
                    // The upward segment owns the positive side after
                    // alpha; the downward segment owns it before alpha.
                    let expected = match (y_sign, at_start) {
                        (1, true) => 1,
                        (-1, false) => -1,
                        _ => 0,
                    };
                    for reversed in [false, true] {
                        let fragment = AlgebraicRayRationalFragment2 {
                            curve: curve.clone(),
                            retained_range: Some(range.clone()),
                            reversed,
                        };
                        assert_eq!(
                            algebraic_point_rational_curve_ray_winding(
                                &fragment,
                                &query,
                                &Real::one(),
                                &Real::zero(),
                                &policy,
                            )
                            .unwrap(),
                            Classification::Decided(if reversed { -expected } else { expected }),
                            "selected endpoint: at_start={at_start}, y_sign={y_sign}, reversed={reversed}, policy={policy:?}",
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn algebraic_retained_range_ray_winding_handles_crossing_multiplicity() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let alpha = sqrt_half_algebraic_parameter(&policy);
        let BezierParameter2::Algebraic(alpha_root) = &alpha else {
            panic!("sqrt(1/2) must remain algebraic");
        };
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let eighth = (Real::one() / Real::from(8_i8)).unwrap();
        let query_curve = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), half.clone()),
                Point2::new(Real::one(), half.clone()),
            ],
            vec![Real::one(); 2],
        )
        .expect("valid query carrier");
        let query = crate::tests::decided(
            query_curve
                .point_at_algebraic_parameter(alpha_root, &policy)
                .expect("selected query point"),
        );
        let query = match query.predicate_evaluator(&policy).unwrap() {
            Classification::Decided(query) => query,
            Classification::Uncertain(reason) => {
                panic!("selected query predicate: {reason:?}")
            }
        };
        let range =
            BezierParameterRange2::new_validated(alpha.clone().unit_complement(), alpha.clone());
        let rational = |points: Vec<Point2>| {
            RationalBezier2::try_new(points.clone(), vec![Real::one(); points.len()])
                .expect("valid polynomial rational carrier")
        };
        let x = Real::from(2_i8);
        let double = rational(vec![
            Point2::new(x.clone(), Real::from(6_i8) * &eighth),
            Point2::new(x.clone(), Real::from(2_i8) * &eighth),
            Point2::new(x.clone(), Real::from(6_i8) * &eighth),
        ]);
        let triple = rational(vec![
            Point2::new(x.clone(), Real::from(3_i8) * &eighth),
            Point2::new(x.clone(), Real::from(5_i8) * &eighth),
            Point2::new(x.clone(), Real::from(3_i8) * &eighth),
            Point2::new(x.clone(), Real::from(5_i8) * &eighth),
        ]);
        let outside = rational(vec![
            Point2::new(x.clone(), Real::from(2_i8) * &eighth),
            Point2::new(x, Real::from(10_i8) * &eighth),
        ]);
        let winding = |curve: RationalBezier2, reversed: bool| {
            let fragment = AlgebraicRayRationalFragment2 {
                curve,
                retained_range: Some(CurveParameterRange2::from_bezier_range(range.clone())),
                reversed,
            };
            algebraic_point_rational_curve_ray_winding(
                &fragment,
                &query,
                &Real::one(),
                &Real::zero(),
                &policy,
            )
            .expect("exact retained-range winding")
        };
        assert_eq!(winding(double, false), Classification::Decided(0));
        assert_eq!(winding(triple.clone(), false), Classification::Decided(1));
        assert_eq!(winding(triple, true), Classification::Decided(-1));
        assert_eq!(winding(outside, false), Classification::Decided(0));
    }
}

fn independent_field_algebraic_chord_region(policy: &CurveContext, reversed: bool) -> CurveRegion2 {
    let x_parameter = positive_inverse_sqrt_parameter(2, policy);
    let y_parameter = positive_inverse_sqrt_parameter(3, policy);
    let x_source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap(),
    ));
    let y_source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(p(0, 0), p(0, 1)).unwrap(),
    ));
    let endpoint_image = |source: &BezierSubcurve2, parameter: &BezierParameter2| {
        let BezierParameter2::Algebraic(parameter) = parameter else {
            panic!("the selected endpoint must remain algebraic");
        };
        crate::tests::decided(
            BezierAlgebraicEndpointImage2::from_source_curve(source, parameter, policy).unwrap(),
        )
    };
    let x_fragment = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(Real::zero()),
        end: x_parameter.clone(),
        source_curve: x_source.clone(),
        start_image: None,
        end_image: Some(endpoint_image(&x_source, &x_parameter)),
    };
    let y_fragment = BezierSplitFragment2::RetainedBezier {
        reversed: true,
        start: BezierParameter2::Exact(Real::zero()),
        end: y_parameter.clone(),
        source_curve: y_source.clone(),
        start_image: None,
        end_image: Some(endpoint_image(&y_source, &y_parameter)),
    };
    let point_evidence = |source: &BezierSubcurve2, parameter: &BezierParameter2| {
        let source = RationalBezier2::try_from_subcurve(source).unwrap();
        crate::tests::decided(
            crate::rational_bezier_general::exact_contact_point_evidence(
                &source, parameter, policy,
            )
            .unwrap(),
        )
    };
    let chord = match crate::BezierAlgebraicChord2::try_new(
        point_evidence(&x_source, &x_parameter),
        point_evidence(&y_source, &y_parameter),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => {
            panic!("independent algebraic chord: {reason:?}")
        }
    };
    let mut fragments = vec![
        x_fragment,
        BezierSplitFragment2::AlgebraicChord(chord),
        y_fragment,
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the exact triangle reverses"))
            .collect();
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the retained triangle must close by exact endpoint evidence");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

#[test]
fn independent_field_corner_edits_preserve_normalized_sets() {
    let x_leg = q(1, 2).sqrt().unwrap();
    let y_leg = q(1, 3).sqrt().unwrap();
    let hypotenuse = q(5, 6).sqrt().unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = independent_field_algebraic_chord_region(&policy, reversed);
            // Vertex one is A=(sqrt(1/2),0) in forward order, and
            // B=(0,sqrt(1/3)) after reversal. Exchange axes so the
            // independent construction below uses V=(a,0) in both cases.
            let (a, b) = if reversed {
                (&y_leg, &x_leg)
            } else {
                (&x_leg, &y_leg)
            };
            let point = |x, y| {
                if reversed {
                    Point2::new(y, x)
                } else {
                    Point2::new(x, y)
                }
            };
            let direction_x = (a / &hypotenuse).unwrap();
            let direction_y = (b / &hypotenuse).unwrap();
            let assert_extended_set =
                |solutions: &[CurveRegion2],
                 operation: &str,
                 setback: &Real,
                 connector_sample: &Point2,
                 added_interior: &Point2| {
                    // The independent exterior contacts are V+L*e and V-L*d,
                    // where e=(1,0) and d=(-a,b)/sqrt(a^2+b^2).
                    let first_cut = point(a + setback, Real::zero());
                    let second_cut = point(a + setback * &direction_x, -(setback * &direction_y));
                    let mut found = false;
                    for_each_corner_region(solutions, |edited| {
                        assert!(edited.has_regularized_filled_left_topology(&policy));
                        let assert_location = |sample: &Point2, expected, label| {
                            let location = edited
                                .classify_point_with_policy(&sample.clone().into(), &policy)
                                .unwrap();
                            assert_eq!(location.certainty, CurveCertainty::Certified);
                            assert_eq!(
                                location.value,
                                Classification::Decided(expected),
                                "{operation}: {label}, reversed={reversed}, policy={policy:?}",
                            );
                        };
                        // Every candidate keeps these portions of the original
                        // triangle, regardless of how its boundary is split.
                        assert_location(
                            &Point2::new(q(1, 8), q(1, 8)),
                            RegionPointLocation::Inside,
                            "original interior",
                        );
                        assert_location(&p(2, 2), RegionPointLocation::Outside, "exterior");
                        assert_location(
                            &Point2::new(q(1, 8), Real::zero()),
                            RegionPointLocation::Boundary,
                            "original x axis",
                        );
                        assert_location(
                            &Point2::new(Real::zero(), q(1, 8)),
                            RegionPointLocation::Boundary,
                            "original y axis",
                        );
                        let has_cut = |cut: &Point2| {
                            edited.boundary_loops().iter().any(|boundary| {
                                boundary.fragments().iter().any(|fragment| {
                                    retained_fragment_has_exact_endpoint(fragment, cut)
                                })
                            })
                        };
                        if !has_cut(&first_cut) || !has_cut(&second_cut) {
                            return;
                        }
                        assert_location(&first_cut, RegionPointLocation::Boundary, "first cut");
                        assert_location(&second_cut, RegionPointLocation::Boundary, "second cut");
                        assert_location(
                            connector_sample,
                            RegionPointLocation::Boundary,
                            "connector",
                        );
                        assert_location(
                            added_interior,
                            RegionPointLocation::Inside,
                            "added interior",
                        );
                        found = true;
                    });
                    assert!(found, "the exact exterior corner lobe was lost");
                };
            let trim_chamfers = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    1,
                    q(1, 10),
                    q(1, 10),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the endpoint-image/chord trim chamfer must decide")
                .into_value();
            let extended_chamfers = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    1,
                    q(1, 10),
                    q(1, 10),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the endpoint-image/chord extension chamfer must decide");
            assert_eq!(extended_chamfers.certainty, CurveCertainty::Certified);
            assert!(
                extended_chamfers.value.candidate_count() > trim_chamfers.candidate_count(),
                "the promoted endpoint carrier must retain its incident ray"
            );
            // Midpoint of the two contacts, and centroid of their triangle
            // with V, certify the new straight connector and filled lobe.
            assert_extended_set(
                corner_regions(&extended_chamfers.value),
                "chamfer",
                &q(1, 10),
                &point(
                    a + (Real::one() + &direction_x) * q(1, 20),
                    -&direction_y * q(1, 20),
                ),
                &point(
                    a + (Real::one() + &direction_x) * q(1, 30),
                    -&direction_y * q(1, 30),
                ),
            );

            let radius = q(1, 100);
            let fillets = region
                .fillet_loop_vertex_with_policy(
                    0,
                    1,
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the endpoint-image/chord fillet must decide");
            assert_eq!(fillets.certainty, CurveCertainty::Certified);
            // The exterior circle has center (a+L,-r), with
            // L=r*(sqrt(a^2+b^2)+a)/b. Traversal requires its major arc,
            // which contains the far axis point (a+L+r,-r). One quarter
            // of the way from V to its center lies inside the new lobe.
            let tangent_setback = (&radius * (&hypotenuse + a) / b).unwrap();
            assert_extended_set(
                fillet_regions(&fillets.value),
                "fillet",
                &tangent_setback,
                &point(a + &tangent_setback + &radius, -&radius),
                &point(a + &tangent_setback * q(1, 4), -&radius * q(1, 4)),
            );
        }
    }
}

fn nonlinear_algebraic_endpoint_region(policy: &CurveContext, reversed: bool) -> CurveRegion2 {
    let alpha = positive_inverse_sqrt_parameter(2, policy);
    let BezierParameter2::Algebraic(alpha_root) = &alpha else {
        panic!("sqrt(1/2) must remain algebraic");
    };
    // P(t)=(t^2,t), represented as a quadratic Bezier. The retained span
    // starts at alpha, while its regular decreasing support contains
    // -alpha at exact squared distance 2 from the corner.
    let source = BezierSubcurve2::Quadratic(QuadraticBezier2::new(
        p(0, 0),
        Point2::new(Real::zero(), q(1, 2)),
        p(1, 1),
    ));
    let rational = RationalBezier2::try_from_subcurve(&source).unwrap();
    let corner = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(&rational, &alpha, policy)
            .unwrap(),
    );
    let chord = match crate::BezierAlgebraicChord2::try_new(
        CurvePoint2::from(p(0, 0)),
        corner.clone(),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => panic!("endpoint chord: {reason:?}"),
    };
    let nonlinear = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: alpha.clone(),
        end: BezierParameter2::Exact(Real::one()),
        source_curve: source.clone(),
        start_image: Some(crate::tests::decided(
            BezierAlgebraicEndpointImage2::from_source_curve(&source, alpha_root, policy).unwrap(),
        )),
        end_image: None,
    };
    let closure = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(p(1, 1), p(0, 0)).unwrap(),
        )),
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicChord(chord),
        nonlinear,
        closure,
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().expect("the endpoint loop reverses"))
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy)
        .expect("the nonlinear algebraic endpoint loop closes");
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

#[test]
fn nonlinear_algebraic_endpoint_chamfer_uses_complete_incident_ray() {
    let setback = Real::from(2_i8).sqrt().unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonlinear_algebraic_endpoint_region(&policy, reversed);
            let corner = if reversed { 2 } else { 1 };
            let (previous_setback, next_setback) = if reversed {
                (setback.clone(), Real::zero())
            } else {
                (Real::zero(), setback.clone())
            };
            let trim = region
                .chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    corner,
                    previous_setback.clone(),
                    next_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("the finite nonlinear endpoint search must decide")
                .into_value();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let extended_work = || {
                region.chamfer_loop_vertex_by_setbacks_with_policy(
                    0,
                    corner,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let extended = hyperreal::dispatch_trace::with_recording(extended_work);
            #[cfg(not(feature = "dispatch-trace"))]
            let extended = extended_work();
            #[cfg(feature = "dispatch-trace")]
            let trace = hyperreal::dispatch_trace::take_trace();
            let extended = extended
                .unwrap_or_else(|error| {
                    panic!(
                        "the nonlinear algebraic endpoint ray must extend: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(extended.value.candidate_count() > trim.candidate_count());
            #[cfg(feature = "dispatch-trace")]
            assert!(
                trace.path_count(
                    "hypercurve",
                    "curve-region-retained-chamfer",
                    "certified-cut-chord",
                ) > 0,
                "the chamfer must reuse its original correlated cut evidence: {trace:?}",
            );
            for_each_corner_region(corner_regions(&extended.value), |edited| {
                assert!(
                    edited.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .any(|fragment| {
                            matches!(
                                fragment,
                                BezierSplitFragment2::RetainedBezier { .. }
                                    | BezierSplitFragment2::AnalyticParallel(_)
                                    | BezierSplitFragment2::SelectedFiber(_)
                            )
                        })
                );
            });
        }
    }
}

#[test]
fn nonlinear_algebraic_endpoint_fillet_uses_complete_incident_domain() {
    let radius = q(1, 10);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonlinear_algebraic_endpoint_region(&policy, reversed);
            let corner = if reversed { 2 } else { 1 };
            let trim_work = || {
                region.fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let trim = hyperreal::dispatch_trace::with_recording(trim_work);
            #[cfg(not(feature = "dispatch-trace"))]
            let trim = trim_work();
            let trim = trim
                .unwrap_or_else(|error| {
                    #[cfg(feature = "dispatch-trace")]
                    eprintln!(
                        "nonlinear endpoint fillet dispatch: {:?}",
                        hyperreal::dispatch_trace::take_trace()
                    );
                    panic!("the finite nonlinear endpoint fillet must decide: {error:?}");
                })
                .into_value();
            let extended = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the nonlinear algebraic endpoint fillet domain must decide: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(extended.value.solutions().len() > { trim.solutions().len() });
            let mut found_exterior_lobe = false;
            for_each_corner_region(fillet_regions(&extended.value), |edited| {
                assert!(edited.has_regularized_filled_left_topology(&policy));
                let fragments = edited
                    .boundary_loops()
                    .iter()
                    .flat_map(|boundary| boundary.fragments())
                    .collect::<Vec<_>>();
                // The incident parabola may now retain local selected
                // cuts. Its carrier spelling is immaterial: verify the
                // homogeneous identity X*W = Y^2 on the surviving source.
                assert!(
                    fragments.iter().any(|fragment| {
                        let parallel = match fragment {
                            BezierSplitFragment2::RetainedBezier { source_curve, .. } => {
                                BezierParallel2::from_source(
                                    BezierParallelSource2::Rational(
                                        RationalBezier2::try_from_subcurve(source_curve).unwrap(),
                                    ),
                                    Real::zero(),
                                )
                            }
                            BezierSplitFragment2::AnalyticParallel(fragment) => {
                                fragment.parallel().clone()
                            }
                            BezierSplitFragment2::SelectedFiber(fragment) => {
                                fragment.parallel_carrier()
                            }
                            _ => return false,
                        };
                        if is_zero(parallel.distance(), &CurveContext::STRICT) != Some(true) {
                            return false;
                        }
                        let source = match parallel.source() {
                            BezierParallelSource2::Quadratic(source) => {
                                RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                                    source.clone(),
                                ))
                                .unwrap()
                            }
                            BezierParallelSource2::Rational(source) => source.clone(),
                            BezierParallelSource2::Cubic(_) => return false,
                        };
                        let power = source.homogeneous_power_basis().unwrap();
                        if source.degree() != 2
                            || power
                                .weight
                                .iter()
                                .skip(1)
                                .any(|weight| is_zero(weight, &CurveContext::STRICT) != Some(true))
                            || !matches!(
                                power.y_numerator.get(1).and_then(|coefficient| {
                                    real_sign(coefficient, &CurveContext::STRICT)
                                }),
                                Some(RealSign::Positive | RealSign::Negative)
                            )
                        {
                            return false;
                        }
                        let mut equation = vec![Real::zero(); 2 * source.degree() + 1];
                        for (i, x) in power.x_numerator.iter().enumerate() {
                            for (j, weight) in power.weight.iter().enumerate() {
                                equation[i + j] += x * weight;
                            }
                        }
                        for (i, y) in power.y_numerator.iter().enumerate() {
                            for (j, other_y) in power.y_numerator.iter().enumerate() {
                                equation[i + j] -= y * other_y;
                            }
                        }
                        equation
                            .iter()
                            .all(|value| is_zero(value, &CurveContext::STRICT) == Some(true))
                    }),
                    "the exact incident parabola must survive the edit"
                );
                assert!(
                    fragments.iter().any(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            | BezierSplitFragment2::Materialized {
                                curve: BezierSubcurve2::RationalQuadratic(_),
                                ..
                            }
                    )),
                    "the normalized candidate must retain its fillet: policy={policy:?}, reversed={reversed}, loops={}",
                    edited.boundary_loops().len(),
                );
                let classify = |point: Point2| {
                    let outcome = edited
                        .classify_point_with_policy(&point.clone().into(), &policy)
                        .unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    outcome.into_value()
                };
                // The radius-1/10 circle on the left of both supports
                // has a unique contact 69/100 < t < 7/10 < alpha. Its
                // line contact lies beyond alpha, so both incident ends
                // extend. The CCW exterior lobe meets the original CW
                // region at the corner; nonzero fill must retain both.
                // The first fixed witness lies strictly inside this disk.
                match classify(Point2::new(q(9, 20), q(3, 4))) {
                    Classification::Decided(RegionPointLocation::Inside) => {
                        found_exterior_lobe = true;
                        for (point, expected) in [
                            (
                                Point2::new(q(49, 100), q(7, 10)),
                                RegionPointLocation::Boundary,
                            ),
                            (
                                Point2::new(q(9, 16), q(3, 4)),
                                RegionPointLocation::Boundary,
                            ),
                            (Point2::new(q(1, 2), q(3, 5)), RegionPointLocation::Inside),
                            (
                                Point2::new(q(197, 400), q(7, 10)),
                                RegionPointLocation::Outside,
                            ),
                        ] {
                            assert_eq!(classify(point), Classification::Decided(expected));
                        }
                    }
                    Classification::Decided(
                        RegionPointLocation::Boundary | RegionPointLocation::Outside,
                    ) => {}
                    Classification::Uncertain(reason) => {
                        panic!("the exact exterior-lobe witness must classify: {reason:?}")
                    }
                }
            });
            assert!(
                found_exterior_lobe,
                "the admissible exterior fillet lobe must survive"
            );
        }
    }
}

#[test]
fn nonlinear_algebraic_endpoint_images_reenter_exact_offset_kernel() {
    let distance = q(1, 100);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonlinear_algebraic_endpoint_region(&policy, reversed);
            let offset =
                region.offset_with_policy(distance.clone(), &OffsetCornerStyle2::Bevel, &policy);
            let offset = offset.unwrap_or_else(|error| {
                panic!(
                    "the nonlinear algebraic-endpoint offset must decide: policy={policy:?}, reversed={reversed}, error={error:?}"
                )
            });
            assert_eq!(offset.certainty, CurveCertainty::Certified);
            assert!(!offset.value.is_empty());
            assert_eq!(
                offset
                    .value
                    .classify_point_with_policy(&p(2, 2).into(), &policy)
                    .unwrap()
                    .value,
                Classification::Decided(RegionPointLocation::Outside),
            );
            assert!(offset.value.boundary_loops().iter().any(|boundary| {
                boundary
                    .fragments()
                    .iter()
                    .any(|fragment| matches!(fragment, BezierSplitFragment2::AnalyticParallel(_)))
            }));
        }
    }
}

#[test]
fn independent_field_algebraic_chord_closes_and_classifies_a_region() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = independent_field_algebraic_chord_region(&policy, false);
        let tenth = (Real::one() / Real::from(10_i8)).unwrap();
        assert_eq!(
            region
                .classify_point_with_policy(&Point2::new(tenth.clone(), tenth).into(), &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            region
                .classify_point_with_policy(&p(1, 1).into(), &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            region
                .classify_point_with_policy(&p(0, 0).into(), &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Boundary)
        );
    }
}

#[test]
fn independent_field_chord_publishes_nonzero_parallel_line_overlap() {
    let tenth = (Real::one() / Real::from(10_i8)).unwrap();
    let high = (Real::from(2_i8) / Real::from(3_i8)).unwrap();
    let low = (Real::from(3_i8) / Real::from(5_i8)).unwrap();
    let independent_parallel = QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(
            Point2::new(
                -tenth.clone(),
                (Real::one() / Real::from(2_i8)).unwrap().sqrt().unwrap(),
            ),
            Point2::new(
                -tenth.clone(),
                (Real::one() / Real::from(3_i8)).unwrap().sqrt().unwrap(),
            ),
        )
        .unwrap(),
    )
    .parallel_left(tenth.clone())
    .unwrap();
    let parallel = QuadraticBezier2::from_line_segment(
        LineSeg2::try_new(
            Point2::new(-tenth.clone(), high.clone()),
            Point2::new(-tenth.clone(), low.clone()),
        )
        .unwrap(),
    )
    .parallel_left(tenth.clone())
    .unwrap();
    let Classification::Decided(parallel_start) = parallel
        .point_at(&Real::zero(), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("the retained source line must have an exact parallel start");
    };
    let Classification::Decided(parallel_end) = parallel
        .point_at(&Real::one(), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("the retained source line must have an exact parallel end");
    };
    let outer_start = Point2::new(tenth.clone(), high);
    let outer_end = Point2::new(tenth.clone(), low);
    let line = |start, end| BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(start, end).unwrap(),
        )),
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            independent_parallel
                .exact_rational_parallel_component(&policy)
                .unwrap(),
            Classification::Decided(Some(_))
        ));
        let Classification::Decided(analysis) = independent_parallel
            .singularity_analysis(&CurveParameterRange2::unit(), &policy)
            .unwrap()
        else {
            panic!("retained line provenance must certify constant nonzero speed");
        };
        assert!(analysis.source_singularities().is_empty());
        assert!(analysis.parallel_cusps().is_empty());
        let y_source = BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(p(0, 0), p(0, 1)).unwrap(),
        ));
        let y_rational = RationalBezier2::try_from_subcurve(&y_source).unwrap();
        let lower_parameter = positive_inverse_sqrt_parameter(3, &policy);
        let upper_parameter = positive_inverse_sqrt_parameter(2, &policy);
        let algebraic_point = |parameter: &BezierParameter2| {
            crate::tests::decided(
                crate::rational_bezier_general::exact_contact_point_evidence(
                    &y_rational,
                    parameter,
                    &policy,
                )
                .unwrap(),
            )
        };
        let vertices = [
            algebraic_point(&lower_parameter),
            CurvePoint2::from(p(1, 0)),
            CurvePoint2::from(p(1, 1)),
            algebraic_point(&upper_parameter),
        ];
        let region_fragments = (0..vertices.len())
            .map(|index| {
                let next = (index + 1) % vertices.len();
                let Classification::Decided(chord) = crate::BezierAlgebraicChord2::try_new(
                    vertices[index].clone(),
                    vertices[next].clone(),
                    &policy,
                )
                .unwrap() else {
                    panic!("the independent-field polygon chord must construct");
                };
                BezierSplitFragment2::AlgebraicChord(chord)
            })
            .collect::<Vec<_>>();
        assert!(matches!(
            parallel
                .exact_rational_parallel_component(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided(Some(_))
        ));
        let Classification::Decided(parallel_fragment) = crate::BezierParallelFragment2::try_new(
            parallel.clone(),
            BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            &policy,
        )
        .unwrap() else {
            panic!("the exact PH parallel span must remain regular");
        };
        let cutter = CurveRegion2::try_new_with_loop_topology(
            vec![
                CurveRegionBoundaryLoop2::new(
                    vec![
                        BezierSplitFragment2::AnalyticParallel(parallel_fragment),
                        line(parallel_end.clone(), outer_end.clone()),
                        line(outer_end.clone(), outer_start.clone()),
                        line(outer_start.clone(), parallel_start.clone()),
                    ],
                    &policy,
                )
                .expect("the exact parallel strip closes"),
            ],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .expect("the exact parallel strip has authored topology");
        for reversed in [false, true] {
            let (fragments, interior_side) = if reversed {
                (
                    region_fragments
                        .clone()
                        .into_iter()
                        .rev()
                        .map(|fragment| fragment.reversed().unwrap())
                        .collect(),
                    CurveBoundaryInteriorSide2::Right,
                )
            } else {
                (region_fragments.clone(), CurveBoundaryInteriorSide2::Left)
            };
            let region = CurveRegion2::try_new_with_loop_topology(
                vec![CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap()],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![interior_side],
            )
            .unwrap();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let work = || region.intersect_region_with_policy(&cutter, &policy);
            #[cfg(feature = "dispatch-trace")]
            let evidence = hyperreal::dispatch_trace::with_recording(work);
            #[cfg(not(feature = "dispatch-trace"))]
            let evidence = work();
            #[cfg(feature = "dispatch-trace")]
            let trace = hyperreal::dispatch_trace::take_trace();
            #[cfg(feature = "dispatch-trace")]
            let evidence = evidence.unwrap_or_else(|error| {
                panic!("the coincident chord/parallel relation is exact: {error:?}; {trace:?}")
            });
            #[cfg(not(feature = "dispatch-trace"))]
            let evidence = evidence.expect("the coincident chord/parallel relation is exact");
            #[cfg(feature = "dispatch-trace")]
            assert!(
                evidence.value.is_complete(),
                "{:?}; {trace:?}",
                evidence.value.blockers()
            );
            #[cfg(not(feature = "dispatch-trace"))]
            assert!(
                evidence.value.is_complete(),
                "{:?}",
                evidence.value.blockers()
            );
            assert!(!evidence.value.overlaps().is_empty());
            #[cfg(feature = "dispatch-trace")]
            assert!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-strict-rational-component",
                ) > 0,
                "the exact analytic component must reuse rational overlap authority: {trace:?}",
            );
            #[cfg(feature = "dispatch-trace")]
            assert!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "collinear-overlap-complete",
                ) > 0,
                "the positive-dimensional support must be certified before overlap mapping: {trace:?}",
            );
            let booleans = region
                .boolean_regions_with_policy(&cutter, &policy)
                .expect("the shared chord/parallel boundary must regularize");
            assert_eq!(booleans.certainty, CurveCertainty::Certified);
        }
    }
}

#[test]
fn independent_field_chord_crosses_a_genuine_analytic_parallel_boolean() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let triangle = independent_field_algebraic_chord_region(&policy, reversed);
            let chord = triangle.boundary_loops()[0]
                .fragments()
                .iter()
                .find_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicChord(chord) => Some(chord),
                    _ => None,
                })
                .expect("the retained triangle has one algebraic chord");
            assert!(chord.exact_line().is_none());
            assert!(chord.strict_provenance_support_line(&policy).is_none());

            let quarter = (Real::one() / Real::from(4_i8)).unwrap();
            let twentieth = (Real::one() / Real::from(20_i8)).unwrap();
            let source = QuadraticBezier2::new(
                p(-1, 0),
                Point2::new(Real::zero(), -quarter),
                Point2::new(Real::one(), (Real::one() / Real::from(2_i8)).unwrap()),
            );
            let parallel = source
                .parallel_left(twentieth)
                .expect("the regular non-PH source has an exact analytic parallel");
            assert!(matches!(
                parallel
                    .exact_rational_parallel_component(&CurveContext::STRICT)
                    .unwrap(),
                Classification::Decided(None)
            ));
            let contacts = match chord.parallel_intersections(&parallel, &policy).unwrap() {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                        contacts,
                    ),
                ) => contacts,
                result => panic!("the retained chord/parallel solve must complete: {result:?}"),
            };
            assert!(!contacts.is_empty());
            assert!(
                contacts
                    .iter()
                    .any(|candidate| candidate.tangent_cross_sign() != RealSign::Zero)
            );

            let endpoint = |parameter: Real| match parallel.point_at(&parameter, &policy).unwrap() {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    panic!("an exact analytic endpoint must evaluate: {reason:?}")
                }
            };
            let lower_left = endpoint(Real::zero());
            let lower_right = endpoint(Real::one());
            let top = if compare_reals(lower_left.y(), lower_right.y(), &policy)
                == Some(std::cmp::Ordering::Greater)
            {
                lower_left.y() + Real::from(2_i8)
            } else {
                lower_right.y() + Real::from(2_i8)
            };
            let upper_left = Point2::new(lower_left.x().clone(), top.clone());
            let upper_right = Point2::new(lower_right.x().clone(), top);
            let Classification::Decided(bottom) = crate::BezierParallelFragment2::try_new(
                parallel,
                BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                &policy,
            )
            .unwrap() else {
                panic!("the complete analytic cutter span must be regular");
            };
            let line = |start, end| BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(
                    LineSeg2::try_new(start, end).unwrap(),
                )),
            };
            let cutter = CurveRegion2::try_new_with_loop_topology(
                vec![
                    CurveRegionBoundaryLoop2::new(
                        vec![
                            BezierSplitFragment2::AnalyticParallel(bottom),
                            line(lower_right.clone(), upper_right.clone()),
                            line(upper_right, upper_left.clone()),
                            line(upper_left, lower_left),
                        ],
                        &policy,
                    )
                    .expect("the analytic cutter closes exactly"),
                ],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![CurveBoundaryInteriorSide2::Left],
            )
            .expect("the analytic cutter has authored topology");
            let evidence = triangle
                .intersect_region_with_policy(&cutter, &policy)
                .expect("the retained chord/analytic carrier pair remains exact");
            assert!(
                evidence.value.is_complete(),
                "the retained chord/analytic intersection must complete: {:?}",
                evidence.value.blockers()
            );
            let booleans = triangle
                .boolean_regions_with_policy(&cutter, &policy)
                .expect("the retained chord/analytic Boolean must complete");
            assert_eq!(booleans.certainty, CurveCertainty::Certified);
            assert!(!booleans.value.intersection().is_empty());
            assert!(!booleans.value.difference().is_empty());
        }
    }
}

#[test]
fn independent_field_chord_replays_a_genuine_parallel_endpoint_contact() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parallel_parameter = positive_inverse_sqrt_parameter(2, &policy);
        let independent_parameter = positive_inverse_sqrt_parameter(3, &policy);
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        let endpoint_source = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), half.clone()),
                Point2::new(three_quarters, half.clone()),
            ],
            vec![Real::one(); 2],
        )
        .expect("the selected parallel endpoint source is valid");
        let independent_source =
            RationalBezier2::try_new(vec![p(0, 0), p(0, 1)], vec![Real::one(); 2])
                .expect("the independent selected endpoint source is valid");
        let selected_point = |source: &RationalBezier2, parameter: &BezierParameter2| {
            let BezierParameter2::Algebraic(parameter) = parameter else {
                panic!("the selected endpoint parameter must remain algebraic");
            };
            CurvePoint2::from(crate::tests::decided(
                source
                    .point_at_algebraic_parameter(parameter, &policy)
                    .expect("the selected endpoint image is exact"),
            ))
        };
        let chord = match crate::BezierAlgebraicChord2::try_new(
            selected_point(&endpoint_source, &parallel_parameter),
            selected_point(&independent_source, &independent_parameter),
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                panic!("the independent endpoint chord must construct: {reason:?}")
            }
        };

        let three_halves = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
        let distance = (three_halves.sqrt().unwrap() / Real::from(4_i8)).unwrap();
        let source = QuadraticBezier2::new(
            p(0, 0),
            Point2::new(half.clone(), Real::zero()),
            Point2::new(Real::one(), half),
        );
        let parallel = source
            .parallel_left(distance)
            .expect("the regular quadratic has an exact analytic parallel");
        assert!(matches!(
            parallel
                .exact_rational_parallel_component(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided(None)
        ));

        for reversed in [false, true] {
            let chord = if reversed {
                chord.reversed()
            } else {
                chord.clone()
            };
            assert!(chord.exact_line().is_none());
            assert!(chord.strict_provenance_support_line(&policy).is_none());
            let contacts = match chord.parallel_intersections(&parallel, &policy).unwrap() {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                        contacts,
                    ),
                ) => contacts,
                result => panic!("the endpoint chord/parallel solve must complete: {result:?}"),
            };
            let contact = contacts
                .iter()
                .find(|contact| {
                    contact
                        .parallel_parameter()
                        .cmp_by_refinement(&parallel_parameter.clone().into(), &policy)
                        .unwrap()
                        == Classification::Decided(std::cmp::Ordering::Equal)
                })
                .expect("the exact parallel parameter reaches the chord endpoint");
            let endpoint = if reversed {
                chord.end_parameter()
            } else {
                chord.start_parameter()
            };
            assert_eq!(
                contact
                    .chord_parameter()
                    .cmp_by_refinement(&endpoint, &policy)
                    .unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal),
            );
            assert_ne!(contact.tangent_cross_sign(), RealSign::Zero);
        }
    }
}

fn nonrepresented_chord_parallel_corner_region(
    policy: &CurveContext,
    reversed: bool,
) -> CurveRegion2 {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let parallel_parameter = BezierParameter2::Exact(half.clone());
    let independent_parameter = positive_inverse_sqrt_parameter(3, policy);
    let source = QuadraticBezier2::new(
        p(0, 0),
        Point2::new(half.clone(), Real::zero()),
        Point2::new(Real::one(), half.clone()),
    );
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let parallel_endpoint = match parallel.point_at(&half, policy).unwrap() {
        Classification::Decided(point) => CurvePoint2::from(point),
        Classification::Uncertain(reason) => panic!("parallel endpoint: {reason:?}"),
    };
    let independent_source =
        RationalBezier2::try_new(vec![p(0, 0), p(0, 1)], vec![Real::one(); 2]).unwrap();
    let independent_point = {
        let BezierParameter2::Algebraic(parameter) = &independent_parameter else {
            panic!("the independent endpoint parameter must remain algebraic");
        };
        CurvePoint2::from(crate::tests::decided(
            independent_source
                .point_at_algebraic_parameter(parameter, policy)
                .unwrap(),
        ))
    };
    let chord =
        match crate::BezierAlgebraicChord2::try_new(parallel_endpoint, independent_point, policy)
            .unwrap()
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => panic!("independent chord: {reason:?}"),
        };
    assert!(chord.exact_line().is_none());
    assert!(chord.strict_provenance_support_line(policy).is_none());
    let range = match BezierParameterRange2::try_new(
        BezierParameter2::Exact(Real::zero()),
        parallel_parameter.clone(),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(range) => range,
        Classification::Uncertain(reason) => panic!("parallel range: {reason:?}"),
    };
    let parallel_fragment =
        match crate::BezierParallelFragment2::try_new(parallel.clone(), range, policy).unwrap() {
            Classification::Decided(fragment) => fragment,
            Classification::Uncertain(reason) => panic!("parallel fragment: {reason:?}"),
        };
    let parallel_start =
        CurvePoint2::from(crate::bezier_offset::BezierAnalyticParallelPoint2::new(
            parallel,
            BezierParameter2::Exact(Real::zero()),
            policy,
        ));
    let closing =
        match crate::BezierAlgebraicChord2::try_new(chord.end().clone(), parallel_start, policy)
            .unwrap()
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => panic!("closing chord: {reason:?}"),
        };
    let mut fragments = vec![
        BezierSplitFragment2::AnalyticParallel(parallel_fragment),
        BezierSplitFragment2::AlgebraicChord(chord),
        BezierSplitFragment2::AlgebraicChord(closing),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .unwrap(),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

fn nonrepresented_chord_line_corner_region(policy: &CurveContext, reversed: bool) -> CurveRegion2 {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let selected_parameter = positive_inverse_sqrt_parameter(2, policy);
    let source = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(half, Real::zero()), p(1, 1)],
        vec![Real::one(); 3],
    )
    .unwrap();
    let selected_point = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(
            &source,
            &selected_parameter,
            policy,
        )
        .unwrap(),
    );
    let corner = CurvePoint2::from(p(0, 0));
    let chord =
        match crate::BezierAlgebraicChord2::try_new(corner.clone(), selected_point.clone(), policy)
            .unwrap()
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => panic!("independent corner chord: {reason:?}"),
        };
    assert!(chord.exact_line().is_none());
    assert!(chord.strict_provenance_support_line(policy).is_none());
    let line = LineSeg2::try_new(p(-1, 0), p(0, 0)).unwrap();
    let closing = match crate::BezierAlgebraicChord2::try_new(
        selected_point,
        CurvePoint2::from(p(-1, 0)),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => panic!("independent closing chord: {reason:?}"),
    };
    let mut fragments = vec![
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line)),
        },
        BezierSplitFragment2::AlgebraicChord(chord),
        BezierSplitFragment2::AlgebraicChord(closing),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .unwrap(),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

fn nonrepresented_cardinal_chord_pair_corner_region(
    policy: &CurveContext,
    reversed: bool,
) -> CurveRegion2 {
    let selected_parameter = positive_inverse_sqrt_parameter(2, policy);
    let diagonal = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![Real::one(); 2]).unwrap();
    let corner = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(
            &diagonal,
            &selected_parameter,
            policy,
        )
        .unwrap(),
    );
    let translated =
        |point: &CurvePoint2, x, y| match crate::BezierAlgebraicChord2::translated_endpoint(
            point,
            &Real::from(x),
            &Real::from(y),
            policy,
        )
        .unwrap()
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("the selected corner translation must remain exact: {reason:?}")
            }
        };
    let chord = |start, end, direction| {
        let chord = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
            start, end, direction, policy,
        );
        assert!(chord.exact_line().is_none());
        assert!(chord.strict_provenance_support_line(policy).is_none());
        assert!(chord.certified_unit_tangent().is_some());
        chord
    };
    let previous = chord(
        translated(&corner, 0, -1),
        corner.clone(),
        crate::bezier_offset::BezierAlgebraicChordAxisDirection2::PositiveY,
    );
    let next = chord(
        corner,
        translated(previous.end(), 1, 0),
        crate::bezier_offset::BezierAlgebraicChordAxisDirection2::PositiveX,
    );
    let Classification::Decided(closing) =
        crate::BezierAlgebraicChord2::try_new(next.end().clone(), previous.start().clone(), policy)
            .unwrap()
    else {
        panic!("the closing chord must construct");
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicChord(previous),
        BezierSplitFragment2::AlgebraicChord(next),
        BezierSplitFragment2::AlgebraicChord(closing),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .unwrap(),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

fn independent_oblique_chord_pair_corner_region(
    policy: &CurveContext,
    reversed: bool,
) -> CurveRegion2 {
    let selected = |start: Point2, end: Point2, radicand| {
        let source = RationalBezier2::try_new(vec![start, end], vec![Real::one(); 2]).unwrap();
        crate::tests::decided(
            crate::rational_bezier_general::exact_contact_point_evidence(
                &source,
                &positive_inverse_sqrt_parameter(radicand, policy),
                policy,
            )
            .unwrap(),
        )
    };
    let previous_start = selected(p(0, 0), p(0, 1), 3);
    let corner = selected(p(0, 0), p(1, 0), 2);
    let next_end = selected(p(1, 0), p(1, 1), 5);
    let chord =
        |start, end| match crate::BezierAlgebraicChord2::try_new(start, end, policy).unwrap() {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => {
                panic!("the independent oblique chord must construct: {reason:?}")
            }
        };
    let previous = chord(previous_start.clone(), corner.clone());
    let next = chord(corner, next_end.clone());
    for incident in [&previous, &next] {
        assert!(incident.exact_line().is_none());
        assert!(incident.strict_provenance_support_line(policy).is_none());
        assert!(incident.certified_unit_tangent().is_none());
    }
    let closing = chord(next_end, previous_start);
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicChord(previous),
        BezierSplitFragment2::AlgebraicChord(next),
        BezierSplitFragment2::AlgebraicChord(closing),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .unwrap(),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

fn nonrepresented_chord_rational_arc_corner_region(
    policy: &CurveContext,
    reversed: bool,
    major_arc: bool,
    elevated: bool,
) -> CurveRegion2 {
    let selected_parameter = positive_inverse_sqrt_parameter(2, policy);
    let diagonal = RationalBezier2::try_new(vec![p(0, 0), p(1, 1)], vec![Real::one(); 2]).unwrap();
    let corner = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(
            &diagonal,
            &selected_parameter,
            policy,
        )
        .unwrap(),
    );
    let translated =
        |point: &CurvePoint2, x, y| match crate::BezierAlgebraicChord2::translated_endpoint(
            point,
            &Real::from(x),
            &Real::from(y),
            policy,
        )
        .unwrap()
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                panic!("the selected chord/arc translation must remain exact: {reason:?}")
            }
        };
    let previous = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
        translated(&corner, 0, -4),
        corner,
        crate::bezier_offset::BezierAlgebraicChordAxisDirection2::PositiveY,
        policy,
    );
    assert!(previous.exact_line().is_none());
    assert!(previous.strict_provenance_support_line(policy).is_none());

    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let selected_coordinate = half.sqrt().unwrap();
    let arc_start = Point2::new(selected_coordinate.clone(), selected_coordinate.clone());
    let arc_control = Point2::new(
        &selected_coordinate + Real::one(),
        selected_coordinate.clone(),
    );
    let arc_end = Point2::new(
        &selected_coordinate + Real::one(),
        &selected_coordinate + Real::one(),
    );
    let arc = RationalQuadraticBezier2::try_new(
        arc_start,
        arc_control,
        arc_end.clone(),
        Real::one(),
        if major_arc {
            -selected_coordinate
        } else {
            selected_coordinate
        },
        Real::one(),
    )
    .unwrap();
    let Classification::Decided(Some(recognized)) =
        crate::arc_bezier::rational_quadratic_circular_arc(&arc, policy).unwrap()
    else {
        panic!("the pole-free authored circular conic must retain arc support")
    };
    if major_arc {
        assert_eq!(
            recognized
                .rational_bezier_decomposition(policy)
                .unwrap()
                .into_value()
                .spans()
                .len(),
            3,
        );
    }
    let Classification::Decided(closing) = crate::BezierAlgebraicChord2::try_new(
        CurvePoint2::from(arc_end),
        previous.start().clone(),
        policy,
    )
    .unwrap() else {
        panic!("the selected chord/arc closing chord must construct");
    };
    let arc = if elevated {
        BezierSubcurve2::Rational(
            RationalBezier2::from(arc)
                .elevated_to_degree(5)
                .expect("the authored circular conic elevates exactly"),
        )
    } else {
        BezierSubcurve2::RationalQuadratic(arc)
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicChord(previous),
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: arc,
        },
        BezierSplitFragment2::AlgebraicChord(closing),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        CurveBoundaryInteriorSide2::Left
    } else {
        CurveBoundaryInteriorSide2::Right
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .unwrap(),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

fn nonrepresented_chord_selected_circle_corner_region(
    policy: &CurveContext,
    reversed: bool,
) -> CurveRegion2 {
    let center_source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0)], vec![Real::one(); 2]).unwrap();
    let center_parameter = positive_inverse_sqrt_parameter(2, policy);
    let center = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(
            &center_source,
            &center_parameter,
            policy,
        )
        .unwrap(),
    );
    let circle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
        &center,
        (1, 0),
        Real::one(),
        false,
        policy,
    )
    .unwrap()
    {
        Classification::Decided(Some(circle)) => circle,
        result => panic!("the selected circle must construct: {result:?}"),
    };
    let start = match circle.start_point_evidence(policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("selected circle start: {reason:?}"),
    };
    let end = match circle.end_point_evidence(policy).unwrap() {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => panic!("selected circle end: {reason:?}"),
    };

    // Keep the incident chord's second endpoint in an independent exact
    // field. Its support cannot collapse to a represented line or retain
    // a represented unit tangent, so the fillet must exercise the general
    // selected-circle/chord kernel and selected-concentric frame.
    let independent_source =
        RationalBezier2::try_new(vec![p(0, 0), p(1, -1), p(2, -3)], vec![Real::one(); 3]).unwrap();
    let independent_parameter = positive_inverse_sqrt_parameter(3, policy);
    let independent = crate::tests::decided(
        crate::rational_bezier_general::exact_contact_point_evidence(
            &independent_source,
            &independent_parameter,
            policy,
        )
        .unwrap(),
    );
    let source_chord =
        match crate::BezierAlgebraicChord2::try_new(independent.clone(), start.clone(), policy)
            .unwrap()
        {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => panic!("the source chord: {reason:?}"),
        };
    let (parallel_start, parallel_end) =
        crate::bezier_offset::BezierAlgebraicChordParallelPoint2::new_pair(
            source_chord,
            Real::zero(),
            Real::zero(),
            Real::zero(),
            policy,
        );
    let previous = match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        CurvePoint2::from(parallel_start),
        CurvePoint2::from(parallel_end),
        policy,
    )
    .unwrap()
    {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => panic!("the incident chord: {reason:?}"),
    };
    assert!(previous.exact_line().is_none());
    assert!(previous.strict_provenance_support_line(policy).is_none());
    assert!(previous.certified_unit_tangent().is_none());
    let closing = match crate::BezierAlgebraicChord2::try_new(end, independent, policy).unwrap() {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => panic!("the closing chord: {reason:?}"),
    };
    let mut fragments = vec![
        BezierSplitFragment2::AlgebraicChord(previous),
        BezierSplitFragment2::AlgebraicCuspSemicircle(
            crate::BezierAlgebraicCuspSemicircleFragment2::full(circle, policy),
        ),
        BezierSplitFragment2::AlgebraicChord(closing),
    ];
    let interior_side = if reversed {
        fragments = fragments
            .into_iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        CurveBoundaryInteriorSide2::Right
    } else {
        CurveBoundaryInteriorSide2::Left
    };
    CurveRegion2::try_new_with_loop_topology(
        vec![
            CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                fragments, None, policy,
            )
            .unwrap(),
        ],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![interior_side],
    )
    .unwrap()
}

#[test]
fn nonrepresented_cardinal_chord_pair_fillets_through_shared_carriers() {
    let radius = (Real::one() / Real::from(100_i16)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonrepresented_cardinal_chord_pair_corner_region(&policy, reversed);
            let outcome = region
                .fillet_loop_vertex_with_policy(
                    0,
                    if reversed { 2 } else { 1 },
                    &crate::CurveFillet2::new(radius.clone()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "nonrepresented chord/chord fillet: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert!(!outcome.value.solutions().is_empty());
            for_each_corner_region(fillet_regions(&outcome.value), |filleted| {
                let fragments = filleted.boundary_loops()[0].fragments();
                let mut chord_adjacencies = 0;
                for (index, fragment) in fragments.iter().enumerate() {
                    let BezierSplitFragment2::AlgebraicCuspSemicircle(fillet) = fragment else {
                        continue;
                    };
                    for (adjacent, shared_circle_start) in [
                        (
                            &fragments[(index + fragments.len() - 1) % fragments.len()],
                            true,
                        ),
                        (&fragments[(index + 1) % fragments.len()], false),
                    ] {
                        let BezierSplitFragment2::AlgebraicChord(chord) = adjacent else {
                            continue;
                        };
                        assert_eq!(
                            fillet.certified_adjacent_chord_is_endpoint_only(
                                chord,
                                shared_circle_start,
                                &policy,
                            ),
                            Ok(Classification::Decided(true))
                        );
                        chord_adjacencies += 1;
                    }
                }
                assert_eq!(chord_adjacencies, 2);
                assert!(
                    fragments
                        .iter()
                        .filter(|fragment| matches!(
                            fragment,
                            BezierSplitFragment2::AlgebraicChord(_)
                        ))
                        .count()
                        >= 2
                );
            });
        }
    }
}

#[test]
fn independent_oblique_chord_pair_fillets_extend_on_infinite_supports() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = independent_oblique_chord_pair_corner_region(&policy, reversed);
            let corner = if reversed { 2 } else { 1 };
            let incident_supports = [corner - 1, corner].map(|index| {
                let BezierSplitFragment2::AlgebraicChord(chord) =
                    &region.boundary_loops()[0].fragments()[index]
                else {
                    panic!("both incident supports are retained chords")
                };
                chord.retained_support()
            });
            let trim = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "independent chord/chord trim: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(trim.certainty, CurveCertainty::Certified);
            let extended = region
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "independent chord/chord extension: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(extended.certainty, CurveCertainty::Certified);
            assert!(
                extended.value.solutions().len() > trim.value.solutions().len(),
                "extension must publish the supporting-line contact: policy={policy:?}, reversed={reversed}, trim={:?}, extended={:?}",
                trim.value,
                extended.value,
            );
            for_each_corner_region(fillet_regions(&extended.value), |filleted| {
                assert!(filleted.has_regularized_filled_left_topology(&policy));
                let mut retained_incident_tangencies = [false; 2];
                let mut chord_adjacencies = 0;
                for boundary in filleted.boundary_loops() {
                    let fragments = boundary.fragments();
                    for (index, fragment) in fragments.iter().enumerate() {
                        let BezierSplitFragment2::AlgebraicCuspSemicircle(fillet) = fragment else {
                            continue;
                        };
                        for endpoint in [true, false] {
                            if let Ok(Classification::Decided(Some((
                                tangent,
                                RealSign::Zero,
                                Some(RealSign::Positive | RealSign::Negative),
                            )))) = fillet.endpoint_chord_tangent_relation(endpoint, &policy)
                            {
                                let tangent = tangent.retained_support();
                                for (index, source) in incident_supports.iter().enumerate() {
                                    retained_incident_tangencies[index] |= tangent.start()
                                        == source.start()
                                        && tangent.end() == source.end()
                                        || tangent.start() == source.end()
                                            && tangent.end() == source.start();
                                }
                            }
                        }
                        for (adjacent, shared_circle_start) in [
                            (
                                &fragments[(index + fragments.len() - 1) % fragments.len()],
                                true,
                            ),
                            (&fragments[(index + 1) % fragments.len()], false),
                        ] {
                            let BezierSplitFragment2::AlgebraicChord(chord) = adjacent else {
                                continue;
                            };
                            assert_eq!(
                                fillet.certified_adjacent_chord_is_endpoint_only(
                                    chord,
                                    shared_circle_start,
                                    &policy,
                                ),
                                Ok(Classification::Decided(true))
                            );
                            chord_adjacencies += 1;
                        }
                    }
                }
                assert!(chord_adjacencies >= 2);
                assert!(
                    retained_incident_tangencies
                        .into_iter()
                        .all(|retained| retained),
                    "the exact tangent evidence must still identify both incident supports",
                );
                assert_eq!(
                    filleted
                        .classify_point_with_policy(&p(10, 10).into(), &policy)
                        .expect("the chord-normal fillet remains classifiable")
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Outside),
                );
                let replay = filleted
                    .boolean_regions_with_policy(&selected_fillet_disjoint_square(&policy), &policy)
                    .expect("the chord-normal fillet re-enters the Boolean kernel");
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert!(replay.value.intersection().is_empty());
                assert_eq!(
                    replay.value.union().boundary_loops().len(),
                    filleted.boundary_loops().len() + 1,
                );
                assert_eq!(
                    replay
                        .value
                        .union()
                        .classify_point_with_policy(&p(5, 5).into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Inside),
                );
            });
        }
    }
}

#[test]
fn independent_oblique_chord_pair_fillet_crosses_a_rational_line_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = independent_oblique_chord_pair_corner_region(&policy, false);
        let extended = region
            .fillet_loop_vertex_with_policy(
                0,
                1,
                &crate::CurveFillet2::new(Real::one()),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .expect("the independent chord pair has exact extended fillets");
        let mut exercised = 0;
        for_each_corner_region(fillet_regions(&extended.value), |filleted| {
            assert!(filleted.has_regularized_filled_left_topology(&policy));
            let Some(circle) = filleted
                .boundary_loops()
                .iter()
                .flat_map(|boundary| boundary.fragments())
                .find_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                        if fragment.semicircle().uses_selected_chord_normal_frame() =>
                    {
                        Some(fragment.semicircle())
                    }
                    _ => None,
                })
            else {
                return;
            };
            exercised += 1;
            let center = match circle.center_point_evidence(&policy).unwrap() {
                Classification::Decided(center) => center,
                Classification::Uncertain(reason) => {
                    panic!("the chord-normal center must be exact: {reason:?}")
                }
            };
            let bounds = match crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                &center, 8, &policy,
            ) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    panic!("the chord-pair center must refine: {reason:?}")
                }
            };
            let interior_dyadic = |lower: &Real, upper: &Real| {
                let approximate = ((lower.to_f64_lossy().unwrap() + upper.to_f64_lossy().unwrap())
                    * 0.5)
                    .clamp(-1.0e6, 1.0e6);
                (0..=32)
                    .find_map(|power| {
                        let denominator = 1_i64 << power;
                        let numerator = (approximate * denominator as f64).round() as i64;
                        let candidate = (Real::from(numerator) / Real::from(denominator)).ok()?;
                        (crate::classify::compare_reals(lower, &candidate, &CurveContext::STRICT)
                            == Some(std::cmp::Ordering::Less)
                            && crate::classify::compare_reals(
                                &candidate,
                                upper,
                                &CurveContext::STRICT,
                            ) == Some(std::cmp::Ordering::Less))
                        .then_some(candidate)
                    })
                    .expect("a refined center interval contains a rational dyadic")
            };
            let x = interior_dyadic(bounds.min().x(), bounds.max().x());
            let y = interior_dyadic(bounds.min().y(), bounds.max().y());
            let margin = circle.radial_distance().abs() * Real::from(2_i8) + Real::one();
            let line = RationalBezier2::try_new(
                vec![
                    Point2::new(&x - &margin, y.clone()),
                    Point2::new(&x + &margin, y),
                ],
                vec![Real::one(), Real::one()],
            )
            .unwrap();
            let (contacts, map) = match circle
                .rational_intersections_with_parameter_map(&line, &crate::CurveParameterRange2::unit(), &policy)
                .unwrap()
            {
                Classification::Decided((
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps: unexpected_overlaps },
                    map,
                )) if unexpected_overlaps.is_empty() => (contacts, map),
                Classification::Decided((intersections, _)) => {
                    panic!("the chord-normal probe must cross: {intersections:?}")
                }
                Classification::Uncertain(reason) => {
                    panic!("the chord-normal rational kernel must decide: {reason:?}")
                }
            };
            assert!(!contacts.is_empty());
            let map = map.expect("an interior chord-normal contact retains its angular map");
            for contact in &contacts {
                assert_ne!(contact.tangent_cross_sign, RealSign::Zero);
                assert!(matches!(
                    map.contact_parameter(contact)
                        .parameter_bracket(16, &policy)
                        .unwrap(),
                    Classification::Decided(_)
                ));
            }
        });
        assert!(exercised > 0);
    }
}

#[test]
fn independent_oblique_chord_pair_fillet_crosses_algebraic_chords_exactly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = independent_oblique_chord_pair_corner_region(&policy, false);
        let extended = region
            .fillet_loop_vertex_with_policy(
                0,
                1,
                &crate::CurveFillet2::new(Real::one()),
                CurveCornerMode2::TrimOrExtend,
                &policy,
            )
            .expect("the independent chord pair has exact extended fillets");
        let mut exercised = 0;
        let mut crossings = 0;
        for_each_corner_region(fillet_regions(&extended.value), |filleted| {
            assert!(filleted.has_regularized_filled_left_topology(&policy));
            let Some(circle) = filleted
                .boundary_loops()
                .iter()
                .flat_map(|boundary| boundary.fragments())
                .find_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)
                        if fragment.semicircle().uses_selected_chord_normal_frame() =>
                    {
                        Some(fragment.semicircle())
                    }
                    _ => None,
                })
            else {
                return;
            };
            exercised += 1;
            let bounds = |point| match crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                &point, 16, &policy,
            ) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    panic!("the projective chord fixture must refine: {reason:?}")
                }
            };
            let center = match circle.center_point_evidence(&policy).unwrap() {
                Classification::Decided(center) => bounds(center),
                Classification::Uncertain(reason) => {
                    panic!("the chord-normal center must be exact: {reason:?}")
                }
            };
            let selected_midpoint = match circle
                .point_evidence_at(&(Real::one() / Real::from(2_i8)).unwrap(), &policy)
                .unwrap()
            {
                Classification::Decided(point) => bounds(point),
                Classification::Uncertain(reason) => {
                    panic!("the selected-half midpoint must be exact: {reason:?}")
                }
            };
            let midpoint = |lower: &Real, upper: &Real| {
                (lower.to_f64_lossy().unwrap() + upper.to_f64_lossy().unwrap()) * 0.5
            };
            let center_x = midpoint(center.min().x(), center.max().x());
            let center_y = midpoint(center.min().y(), center.max().y());
            let selected_y = midpoint(selected_midpoint.min().y(), selected_midpoint.max().y());
            for target_radicand in [3_i8, 7_i8] {
                let parameter = positive_inverse_sqrt_parameter(target_radicand, &policy);
                let endpoint = |x: Real, y: Real| {
                    let source = RationalBezier2::try_new(
                        vec![
                            Point2::new(x.clone(), y.clone()),
                            Point2::new(x, y + Real::one()),
                        ],
                        vec![Real::one(); 2],
                    )
                    .unwrap();
                    crate::tests::decided(
                        crate::rational_bezier_general::exact_contact_point_evidence(
                            &source, &parameter, &policy,
                        )
                        .unwrap(),
                    )
                };
                let target_root = 1.0 / f64::from(target_radicand).sqrt();
                let quarter_grid = (center_y - target_root) * 4.0;
                let y_numerator = if selected_y >= center_y {
                    quarter_grid.ceil() as i64
                } else {
                    quarter_grid.floor() as i64
                };
                let y = (Real::from(y_numerator) / Real::from(4_i8)).unwrap();
                let x = center_x.round() as i64;
                let chord = match crate::BezierAlgebraicChord2::try_new(
                    endpoint(Real::from(x - 2), y.clone()),
                    endpoint(Real::from(x + 2), y),
                    &policy,
                )
                .unwrap()
                {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        panic!("the horizontal algebraic chord must construct: {reason:?}")
                    }
                };
                assert!(chord.exact_line().is_none());
                assert!(chord.strict_provenance_support_line(&policy).is_none());
                let carrier = if policy == CurveContext::APPROXIMATE_512 {
                    chord.reversed()
                } else {
                    chord
                };
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::reset();
                let intersection_work = || circle.chord_intersections(&carrier, &policy);
                #[cfg(feature = "dispatch-trace")]
                let intersections = hyperreal::dispatch_trace::with_recording(intersection_work);
                #[cfg(not(feature = "dispatch-trace"))]
                let intersections = intersection_work();
                #[cfg(feature = "dispatch-trace")]
                let trace = hyperreal::dispatch_trace::take_trace();
                let contacts = match intersections.unwrap() {
                    Classification::Decided(contacts) if contacts.is_empty() => {
                        panic!("the selected-side chord must cross the chord-normal circle")
                    }
                    Classification::Decided(contacts) => contacts,
                    result => {
                        panic!("the chord-normal circle must meet its algebraic chord: {result:?}")
                    }
                };
                #[cfg(feature = "dispatch-trace")]
                {
                    assert!(
                        trace.path_count(
                            "hypercurve",
                            "algebraic-circle-chord-kernel",
                            "chord-normal-recursive-quadratic",
                        ) > 0,
                        "the algebraic line must remain quadratic over the retained chord-normal field: {trace:?}",
                    );
                    assert_eq!(
                        trace.path_count(
                            "hypercurve",
                            "algebraic-circle-chord-kernel",
                            "recursive-projective-retained-chord",
                        ),
                        0,
                        "the chord-normal authority must precede the generic recursive bridge: {trace:?}",
                    );
                }
                crossings += 1;
                assert!(!contacts.is_empty());
                for contact in contacts {
                    assert_ne!(contact.tangent_cross_sign, RealSign::Zero);
                    assert_eq!(
                        contact
                            .chord_parameter
                            .cmp_by_refinement(&carrier.start_parameter(), &policy)
                            .unwrap(),
                        Classification::Decided(std::cmp::Ordering::Greater)
                    );
                    assert_eq!(
                        contact
                            .chord_parameter
                            .cmp_by_refinement(&carrier.end_parameter(), &policy)
                            .unwrap(),
                        Classification::Decided(std::cmp::Ordering::Less)
                    );
                    let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = &contact.point
                    else {
                        panic!("the contact must retain its correlated chord map")
                    };
                    assert!(matches!(
                        point.conservative_bounds_refined(16, &policy),
                        Classification::Decided(_)
                    ));
                    assert_eq!(
                        carrier.contains_point(&contact.point, &policy).unwrap(),
                        Classification::Decided(true),
                    );
                    assert_eq!(
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleFragment2::full(
                            circle.clone(),
                            &policy,
                        )
                        .contains_point(&contact.point, &policy)
                        .unwrap(),
                        Classification::Decided(true),
                    );
                }
            }
        });
        assert!(exercised > 0);
        assert_eq!(crossings, exercised * 2);
    }
}

#[test]
fn nonrepresented_chord_and_retained_rational_arc_share_the_fillet_kernel() {
    for radius in [
        (Real::one() / Real::from(100_i16)).unwrap(),
        (Real::from(3_i8) / Real::from(2_i8)).unwrap(),
    ] {
        let radius_squared = &radius * &radius;
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let region = nonrepresented_chord_rational_arc_corner_region(
                    &policy, reversed, false, false,
                );
                let mut trim_count = None;
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let outcome = region
                        .fillet_loop_vertex_with_policy(
                            0,
                            if reversed { 2 } else { 1 },
                            &crate::CurveFillet2::new(radius.clone()),
                            mode,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "nonrepresented chord/rational-arc fillet: policy={policy:?}, reversed={reversed}, mode={mode:?}, error={error:?}"
                            )
                        });
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    assert!(
                        !outcome.value.solutions().is_empty(),
                        "policy={policy:?}, reversed={reversed}, radius={radius:?}, mode={mode:?}, outcome={:?}",
                        outcome.value,
                    );
                    if mode == CurveCornerMode2::TrimOnly {
                        trim_count = Some(outcome.value.solutions().len());
                    } else {
                        assert!(
                            outcome.value.solutions().len()
                                >= trim_count.expect("the trim result runs first"),
                            "extension cannot discard a finite-support candidate"
                        );
                    }
                    for_each_corner_region(fillet_regions(&outcome.value), |filleted| {
                        assert!(filleted.has_regularized_filled_left_topology(&policy));
                        let mut fillet_spans = 0;
                        let mut chord_adjacencies = 0;
                        // Regularization can move a fillet to another boundary loop.
                        for boundary in filleted.boundary_loops() {
                            let fragments = boundary.fragments();
                            for (index, fragment) in fragments.iter().enumerate() {
                                let BezierSplitFragment2::Materialized {
                                    curve: BezierSubcurve2::RationalQuadratic(curve),
                                    ..
                                } = fragment
                                else {
                                    continue;
                                };
                                let Ok(Classification::Decided(Some(arc))) =
                                    crate::arc_bezier::rational_quadratic_circular_arc(
                                        curve, &policy,
                                    )
                                else {
                                    continue;
                                };
                                if crate::classify::is_zero(
                                    &(arc.radius_squared_ref() - &radius_squared),
                                    &CurveContext::STRICT,
                                ) != Some(true)
                                {
                                    continue;
                                }
                                fillet_spans += 1;
                                // Check the actual curve, independently of the
                                // retained circle provenance and radius tag.
                                for parameter in [
                                    Real::zero(),
                                    (Real::one() / Real::from(2_i8)).unwrap(),
                                    Real::one(),
                                ] {
                                    let Classification::Decided(point) =
                                        curve.point_at(parameter, &policy)
                                    else {
                                        panic!("the exact fillet chart evaluates");
                                    };
                                    assert_eq!(
                                        crate::classify::real_sign(
                                            &(point.distance_squared(arc.center())
                                                - &radius_squared),
                                            &CurveContext::STRICT
                                        ),
                                        Some(RealSign::Zero)
                                    );
                                }
                                for (adjacent_index, contact, chord_at_end) in [
                                    (
                                        (index + fragments.len() - 1) % fragments.len(),
                                        arc.start(),
                                        true,
                                    ),
                                    ((index + 1) % fragments.len(), arc.end(), false),
                                ] {
                                    let BezierSplitFragment2::AlgebraicChord(chord) =
                                        &fragments[adjacent_index]
                                    else {
                                        continue;
                                    };
                                    let chord_point = if chord_at_end {
                                        chord.end()
                                    } else {
                                        chord.start()
                                    };
                                    assert_eq!(
                                        chord_point.same_point(
                                            &CurvePoint2::from(contact.clone(),),
                                            &CurveContext::STRICT,
                                        ),
                                        Classification::Decided(true)
                                    );
                                    let (tangent_x, tangent_y) =
                                        chord.certified_unit_tangent().expect(
                                            "the retained cardinal chord keeps its unit tangent",
                                        );
                                    let radial = contact.delta_from(arc.center());
                                    assert_eq!(
                                        crate::classify::real_sign(
                                            &(&tangent_x * &radial.0 + &tangent_y * &radial.1),
                                            &CurveContext::STRICT,
                                        ),
                                        Some(RealSign::Zero)
                                    );
                                    chord_adjacencies += 1;
                                }
                            }
                        }
                        assert!(fillet_spans >= 1);
                        assert!(chord_adjacencies >= 1);
                    });
                }
            }
        }
    }
}

#[test]
fn boundary_curve_views_share_retained_domains_and_endpoint_evidence() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = nonrepresented_chord_rational_arc_corner_region(&policy, false, false, false);
        let paths = region.boundary_paths_with_policy(&policy).unwrap();
        assert_eq!(paths.certainty, CurveCertainty::Certified);
        let Classification::Decided(paths) = paths.value else {
            panic!("retained boundary curves must remain exact connected paths");
        };
        assert_eq!(paths.len(), region.boundary_loops().len());
        for (boundary, path) in region.boundary_loops().iter().zip(&paths) {
            let curves = boundary.curves();
            assert!(curves.iter().any(|curve| curve.geometry().is_none()));
            assert!(std::ptr::eq(curves, boundary.curves()));
            let cloned_boundary = boundary.clone();
            assert!(std::ptr::eq(curves, cloned_boundary.curves()));
            assert_eq!(curves.len(), path.curves().len());
            for (curve, replay) in curves.iter().zip(path.curves()) {
                assert!(std::ptr::eq(
                    curve.parameter_domain(),
                    replay.parameter_domain()
                ));
                assert_eq!(curve.start(), replay.start());
                assert_eq!(curve.end(), replay.end());
            }
        }
    }
}

#[test]
fn selected_corner_candidates_reenter_normalization_with_retained_contacts() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let cases = [
            (
                nonrepresented_chord_rational_arc_corner_region(&policy, false, false, false),
                2,
                q(1, 100),
            ),
            (
                nonrepresented_chord_rational_arc_corner_region(&policy, true, false, false),
                1,
                q(1, 100),
            ),
            (
                independent_oblique_chord_pair_corner_region(&policy, false),
                1,
                Real::one(),
            ),
        ];
        for (source, corner, radius) in cases {
            let outcome = source
                .fillet_loop_vertex_with_policy(
                    0,
                    corner,
                    &crate::CurveFillet2::new(radius),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert!(!outcome.value.solutions().is_empty());
            for_each_corner_region(fillet_regions(&outcome.value), |candidate| {
                let normalized = candidate
                    .regularized_region_raw(&policy)
                    .expect("retained corner contacts must replay without a new coordinate field");
                assert!(normalized.has_regularized_filled_left_topology(&policy));
                assert!(!normalized.is_empty());
                assert_eq!(
                    normalized.regularized_region_raw(&policy).unwrap(),
                    normalized
                );
            });
        }
    }
}

#[test]
fn general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel() {
    let radius = (Real::one() / Real::from(100_i16)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region =
                nonrepresented_chord_rational_arc_corner_region(&policy, reversed, false, false);
            let mut trim_count = None;
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                let outcome = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        if reversed { 1 } else { 2 },
                        &crate::CurveFillet2::new(radius.clone()),
                        mode,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "general chord/rational-arc fillet: policy={policy:?}, reversed={reversed}, mode={mode:?}, error={error:?}"
                        )
                    });
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                assert!(!outcome.value.solutions().is_empty());
                if mode == CurveCornerMode2::TrimOnly {
                    trim_count = Some(outcome.value.solutions().len());
                } else {
                    assert!(
                        outcome.value.solutions().len()
                            > trim_count.expect("the trim result runs first"),
                        "extension must retain an exterior rational-arc/chord-support branch"
                    );
                }
                for_each_corner_region(fillet_regions(&outcome.value), |filleted| {
                    assert!(filleted.has_regularized_filled_left_topology(&policy));
                    let mut chord_adjacencies = 0;
                    let mut retained_tangent_relations = 0;
                    for boundary in filleted.boundary_loops() {
                        let fragments = boundary.fragments();
                        for (index, fragment) in fragments.iter().enumerate() {
                            let BezierSplitFragment2::AlgebraicCuspSemicircle(fillet) = fragment
                            else {
                                continue;
                            };
                            for endpoint in [true, false] {
                                if matches!(
                                    fillet.endpoint_chord_tangent_relation(endpoint, &policy),
                                    Ok(Classification::Decided(Some((
                                        _,
                                        RealSign::Zero,
                                        Some(RealSign::Positive | RealSign::Negative)
                                    ))))
                                ) {
                                    retained_tangent_relations += 1;
                                }
                            }
                            for (adjacent, shared_circle_start) in [
                                (
                                    &fragments[(index + fragments.len() - 1) % fragments.len()],
                                    true,
                                ),
                                (&fragments[(index + 1) % fragments.len()], false),
                            ] {
                                let BezierSplitFragment2::AlgebraicChord(chord) = adjacent else {
                                    continue;
                                };
                                assert_eq!(
                                    fillet.certified_adjacent_chord_is_endpoint_only(
                                        chord,
                                        shared_circle_start,
                                        &policy,
                                    ),
                                    Ok(Classification::Decided(true)),
                                    "policy={policy:?}, reversed={reversed}, mode={mode:?}"
                                );
                                chord_adjacencies += 1;
                            }
                        }
                    }
                    assert!(chord_adjacencies > 0);
                    assert!(retained_tangent_relations > 0);
                });
            }
        }
    }
}

#[test]
fn major_retained_rational_arc_and_general_chord_share_the_fillet_kernel() {
    let radius = (Real::one() / Real::from(100_i16)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            for elevated in [false, true] {
                let region = nonrepresented_chord_rational_arc_corner_region(
                    &policy, reversed, true, elevated,
                );
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let outcome = region
                        .fillet_loop_vertex_with_policy(
                            0,
                            if reversed { 1 } else { 2 },
                            &crate::CurveFillet2::new(radius.clone()),
                            mode,
                            &policy,
                        )
                        .unwrap_or_else(|error| {
                            panic!(
                                "major rational-arc/chord fillet: policy={policy:?}, reversed={reversed}, elevated={elevated}, mode={mode:?}, error={error:?}"
                            )
                        });
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    assert!(
                        !outcome.value.solutions().is_empty(),
                        "policy={policy:?}, reversed={reversed}, elevated={elevated}, mode={mode:?}, outcome={:?}",
                        outcome.value,
                    );
                }
            }
        }
    }
}

#[test]
fn nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel() {
    let radius = (Real::one() / Real::from(10_i8)).unwrap();
    let radius_squared = &radius * &radius;
    let interior = Point2::new(q(1, 2).sqrt().unwrap(), q(1, 2));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonrepresented_chord_selected_circle_corner_region(&policy, reversed);
            let mut trim_count = None;
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                let outcome = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        if reversed { 2 } else { 1 },
                        &crate::CurveFillet2::new(radius.clone()),
                        mode,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "nonrepresented chord/selected-circle fillet: policy={policy:?}, reversed={reversed}, mode={mode:?}, error={error:?}"
                        )
                    });
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                assert!(
                    !outcome.value.solutions().is_empty(),
                    "policy={policy:?}, reversed={reversed}, mode={mode:?}, outcome={:?}",
                    outcome.value
                );
                if mode == CurveCornerMode2::TrimOnly {
                    trim_count = Some(outcome.value.solutions().len());
                } else {
                    assert!(
                        outcome.value.solutions().len()
                            > trim_count.expect("the trim result runs first"),
                        "extension must retain the complementary selected-circle branch"
                    );
                }
                let mut retained_fillet_spans = 0;
                for_each_corner_region(fillet_regions(&outcome.value), |filleted| {
                    assert!(filleted.has_regularized_filled_left_topology(&policy));
                    // Extension can split the walk into separate material
                    // loops, or regularization can consume the fillet.
                    // Inspect every surviving boundary in the exact set.
                    let mut source_spans = 0;
                    let mut chord_adjacencies = 0;
                    for boundary in filleted.boundary_loops() {
                        let fragments = boundary.fragments();
                        for (index, fragment) in fragments.iter().enumerate() {
                            let BezierSplitFragment2::AlgebraicCuspSemicircle(circle) = fragment
                            else {
                                continue;
                            };
                            let radial = circle.semicircle().radial_distance();
                            if crate::classify::is_zero(
                                &(radial * radial - &radius_squared),
                                &CurveContext::STRICT,
                            ) == Some(true)
                            {
                                retained_fillet_spans += 1;
                            } else {
                                assert_eq!(
                                    crate::classify::is_zero(
                                        &(radial * radial - Real::one()),
                                        &CurveContext::STRICT,
                                    ),
                                    Some(true),
                                );
                                source_spans += 1;
                            }
                            for (adjacent, shared_circle_start) in [
                                (
                                    &fragments[(index + fragments.len() - 1) % fragments.len()],
                                    true,
                                ),
                                (&fragments[(index + 1) % fragments.len()], false),
                            ] {
                                let BezierSplitFragment2::AlgebraicChord(chord) = adjacent else {
                                    continue;
                                };
                                if circle.certified_adjacent_chord_is_endpoint_only(
                                    chord,
                                    shared_circle_start,
                                    &policy,
                                ) == Ok(Classification::Decided(true))
                                {
                                    chord_adjacencies += 1;
                                }
                            }
                        }
                    }
                    assert!(source_spans > 0);
                    assert!(chord_adjacencies > 0);
                    for (point, expected) in [
                        (&interior, RegionPointLocation::Inside),
                        (&p(4, 4), RegionPointLocation::Outside),
                    ] {
                        let location = filleted
                            .classify_point_with_policy(&point.clone().into(), &policy)
                            .unwrap();
                        assert_eq!(location.certainty, CurveCertainty::Certified);
                        assert_eq!(location.value, Classification::Decided(expected));
                    }
                });
                assert!(retained_fillet_spans > 0);
            }
        }
    }
}

#[test]
fn collapsed_selected_circle_center_classifies_nonrepresented_chord() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonrepresented_chord_selected_circle_corner_region(&policy, reversed);
            let outcome = region
                .fillet_loop_vertex_with_policy(
                    0,
                    if reversed { 2 } else { 1 },
                    &crate::CurveFillet2::new(Real::one()),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "the collapsed selected center must classify on the retained chord: policy={policy:?}, reversed={reversed}, error={error:?}"
                    )
                });
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert_eq!(
                (outcome.value).no_solution_reason(),
                Some(crate::CurveCornerNoSolution2::NoTangentCircle,)
            );
        }
    }
}

#[test]
fn selected_circle_extension_reconstructs_every_half_chart_path() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let center_source =
            RationalBezier2::try_new(vec![p(0, 0), p(1, 0)], vec![Real::one(); 2]).unwrap();
        let center_parameter = positive_inverse_sqrt_parameter(2, &policy);
        let center = crate::tests::decided(
            crate::rational_bezier_general::exact_contact_point_evidence(
                &center_source,
                &center_parameter,
                &policy,
            )
            .unwrap(),
        );
        let Classification::Decided(Some(circle)) =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::one(),
                false,
                &policy,
            )
            .unwrap()
        else {
            panic!("the selected reconstruction circle must construct");
        };
        let parameter = |numerator, denominator| {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                (Real::from(numerator) / Real::from(denominator)).unwrap(),
            )
        };
        let endpoint = |fragment: &BezierSplitFragment2, start| {
            let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment else {
                panic!("a selected-circle extension must remain on selected circles");
            };
            match fragment.endpoint_point_evidence(start, &policy).unwrap() {
                Classification::Decided(Some(point)) => point,
                result => panic!("selected-circle endpoint evidence: {result:?}"),
            }
        };
        for reversed in [false, true] {
            let Classification::Decided(source) =
                crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                    circle.clone(),
                    parameter(1, 4),
                    parameter(3, 4),
                    reversed,
                    &policy,
                )
                .unwrap()
            else {
                panic!("the selected source span must construct");
            };
            for keep_before_cut in [false, true] {
                let mut base_counts = Vec::new();
                for cut in [parameter(1, 8), parameter(7, 8)] {
                    let expected_cut =
                        match cut.coincident_point_evidence(&circle, &policy).unwrap() {
                            Classification::Decided(Some(point)) => point,
                            result => panic!("base cut evidence: {result:?}"),
                        };
                    let fragments = retained_cusp_fragment_extension(
                        &source,
                        CurveParameter2::from_algebraic_cusp(cut),
                        keep_before_cut,
                        CurveOperation2::Fillet,
                        &policy,
                    )
                    .unwrap();
                    base_counts.push(fragments.len());
                    for pair in fragments.windows(2) {
                        assert_eq!(
                            endpoint(&pair[0], false)
                                .same_point(&endpoint(&pair[1], true), &policy),
                            Classification::Decided(true),
                        );
                    }
                    let terminal = if keep_before_cut {
                        endpoint(fragments.last().unwrap(), false)
                    } else {
                        endpoint(fragments.first().unwrap(), true)
                    };
                    assert_eq!(
                        terminal.same_point(&expected_cut, &policy),
                        Classification::Decided(true),
                    );
                }
                base_counts.sort_unstable();
                assert_eq!(base_counts, vec![1, 3]);

                let complement = circle.complementary_half();
                let cut = parameter(1, 2);
                let expected_cut =
                    match cut.coincident_point_evidence(&complement, &policy).unwrap() {
                        Classification::Decided(Some(point)) => point,
                        result => panic!("complement cut evidence: {result:?}"),
                    };
                let fragments = retained_cusp_fragment_extension(
                    &source,
                    CurveParameter2::from_algebraic_cusp_complement(cut),
                    keep_before_cut,
                    CurveOperation2::Fillet,
                    &policy,
                )
                .unwrap();
                assert_eq!(fragments.len(), 2);
                assert_eq!(
                    endpoint(&fragments[0], false)
                        .same_point(&endpoint(&fragments[1], true), &policy),
                    Classification::Decided(true),
                );
                let terminal = if keep_before_cut {
                    endpoint(fragments.last().unwrap(), false)
                } else {
                    endpoint(fragments.first().unwrap(), true)
                };
                assert_eq!(
                    terminal.same_point(&expected_cut, &policy),
                    Classification::Decided(true),
                );
            }

            // Generate the same extension cuts through the corner
            // solver's compact angular transport. Small setbacks remain
            // on the authored half, larger setbacks cross the chart
            // boundary, and a diameter setback lands at the antipode.
            for start_endpoint in [false, true] {
                let trim = source
                    .endpoint_chord_setback_cut(
                        start_endpoint,
                        &(Real::one() / Real::from(10_i8)).unwrap(),
                        false,
                        &policy,
                    )
                    .unwrap();
                let Classification::Decided(Some((_, _, false))) = trim else {
                    panic!(
                        "the inward selected-circle setback must be a base-chart trim: policy={policy:?}, reversed={reversed}, start={start_endpoint}, result={trim:?}"
                    );
                };
                for (setback, expected_complement) in [
                    ((Real::one() / Real::from(10_i8)).unwrap(), false),
                    (Real::one(), true),
                    (Real::from(2_i8), true),
                ] {
                    let result = source
                        .endpoint_chord_setback_cut(start_endpoint, &setback, true, &policy)
                        .unwrap();
                    let Classification::Decided(Some((cut, point, complementary))) = result else {
                        panic!(
                            "the selected-circle extension cut must construct: policy={policy:?}, reversed={reversed}, start={start_endpoint}, setback={setback:?}, result={result:?}"
                        );
                    };
                    assert_eq!(complementary, expected_complement);
                    let target = if complementary {
                        circle.complementary_half()
                    } else {
                        circle.clone()
                    };
                    let replay = match cut.coincident_point_evidence(&target, &policy).unwrap() {
                        Classification::Decided(Some(point)) => point,
                        result => panic!("extension point replay: {result:?}"),
                    };
                    assert_eq!(
                        replay.same_point(&point, &policy),
                        Classification::Decided(true),
                    );
                    let parameter = if complementary {
                        CurveParameter2::from_algebraic_cusp_complement(cut)
                    } else {
                        CurveParameter2::from_algebraic_cusp(cut)
                    };
                    let keep_before_cut = !start_endpoint;
                    let fragments = retained_cusp_fragment_extension(
                        &source,
                        parameter,
                        keep_before_cut,
                        CurveOperation2::Chamfer,
                        &policy,
                    )
                    .unwrap();
                    let terminal = if keep_before_cut {
                        endpoint(fragments.last().unwrap(), false)
                    } else {
                        endpoint(fragments.first().unwrap(), true)
                    };
                    assert_eq!(
                        terminal.same_point(&point, &policy),
                        Classification::Decided(true),
                    );
                }
            }
        }
    }
}

#[test]
fn nonrepresented_chord_line_corner_fillets_through_shared_carriers() {
    let radius = (Real::one() / Real::from(100_i16)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonrepresented_chord_line_corner_region(&policy, reversed);
            let vertex = if reversed { 2 } else { 1 };
            let mut trim_count = None;
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                let outcome = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        vertex,
                        &crate::CurveFillet2::new(radius.clone()),
                        mode,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "nonrepresented chord/line fillet: policy={policy:?}, reversed={reversed}, mode={mode:?}, error={error:?}"
                        )
                    });
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                assert!(
                    !outcome.value.solutions().is_empty(),
                    "policy={policy:?}, reversed={reversed}, mode={mode:?}, outcome={:?}",
                    outcome.value
                );
                if mode == CurveCornerMode2::TrimOnly {
                    trim_count = Some(outcome.value.solutions().len());
                } else {
                    assert!(
                        outcome.value.solutions().len()
                            > trim_count.expect("the trim result runs first"),
                        "extension must retain the second infinite-support fillet branch"
                    );
                }
                for_each_corner_region(fillet_regions(&outcome.value), |filleted| {
                    assert!(filleted.has_regularized_filled_left_topology(&policy));
                    let mut chord_adjacencies = 0;
                    let mut retained_tangent_relations = 0;
                    for boundary in filleted.boundary_loops() {
                        let fragments = boundary.fragments();
                        for (index, fragment) in fragments.iter().enumerate() {
                            let BezierSplitFragment2::AlgebraicCuspSemicircle(fillet) = fragment
                            else {
                                continue;
                            };
                            for endpoint in [true, false] {
                                if matches!(
                                    fillet.endpoint_chord_tangent_relation(endpoint, &policy),
                                    Ok(Classification::Decided(Some((
                                        _,
                                        RealSign::Zero,
                                        Some(RealSign::Positive | RealSign::Negative)
                                    ))))
                                ) {
                                    retained_tangent_relations += 1;
                                }
                            }
                            for (adjacent_index, shared_circle_start) in [
                                ((index + fragments.len() - 1) % fragments.len(), true),
                                ((index + 1) % fragments.len(), false),
                            ] {
                                let adjacent = &fragments[adjacent_index];
                                let BezierSplitFragment2::AlgebraicChord(chord) = adjacent else {
                                    continue;
                                };
                                assert_eq!(
                                    fillet.certified_adjacent_chord_is_endpoint_only(
                                        chord,
                                        shared_circle_start,
                                        &policy,
                                    ),
                                    Ok(Classification::Decided(true)),
                                    "policy={policy:?}, reversed={reversed}, mode={mode:?}, fillet={index}, adjacent={adjacent_index}"
                                );
                                chord_adjacencies += 1;
                            }
                        }
                    }
                    assert!(chord_adjacencies > 0);
                    assert!(retained_tangent_relations > 0);
                });
            }
        }
    }
}

#[test]
fn nonrepresented_chord_parallel_corner_fillets_without_reintersection() {
    let radius = (Real::one() / Real::from(100_i16)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let region = nonrepresented_chord_parallel_corner_region(&policy, reversed);
            let vertex = if reversed { 2 } else { 1 };
            let mut trim_count = None;
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                let outcome = region
                    .fillet_loop_vertex_with_policy(
                        0,
                        vertex,
                        &crate::CurveFillet2::new(radius.clone()),
                        mode,
                        &policy,
                    )
                    .unwrap_or_else(|error| {
                        panic!(
                            "nonrepresented chord/parallel fillet: policy={policy:?}, reversed={reversed}, mode={mode:?}, error={error:?}"
                        )
                    });
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                assert!(!outcome.value.solutions().is_empty());
                if mode == CurveCornerMode2::TrimOnly {
                    trim_count = Some(outcome.value.solutions().len());
                } else {
                    assert!(
                        outcome.value.solutions().len()
                            > trim_count.expect("the trim result runs first"),
                        "extension must retain an exterior analytic-support fillet branch"
                    );
                }
                for_each_corner_region(fillet_regions(&outcome.value), |filleted| {
                    assert!(filleted.has_regularized_filled_left_topology(&policy));
                    assert!(
                        filleted
                            .boundary_loops()
                            .iter()
                            .flat_map(|boundary| boundary.fragments())
                            .any(|fragment| matches!(
                                fragment,
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            ))
                    );
                    if policy != CurveContext::STRICT || reversed {
                        return;
                    }
                    let mut chord_adjacencies = 0;
                    let mut retained_tangent_relations = 0;
                    for boundary in filleted.boundary_loops() {
                        let fragments = boundary.fragments();
                        for (index, fragment) in fragments.iter().enumerate() {
                            let BezierSplitFragment2::AlgebraicCuspSemicircle(fillet) = fragment
                            else {
                                continue;
                            };
                            for endpoint in [true, false] {
                                if matches!(
                                    fillet.endpoint_chord_tangent_relation(endpoint, &policy),
                                    Ok(Classification::Decided(Some((
                                        _,
                                        RealSign::Zero,
                                        Some(RealSign::Positive | RealSign::Negative)
                                    ))))
                                ) {
                                    retained_tangent_relations += 1;
                                }
                            }
                            for (adjacent, shared_circle_start) in [
                                (
                                    &fragments[(index + fragments.len() - 1) % fragments.len()],
                                    true,
                                ),
                                (&fragments[(index + 1) % fragments.len()], false),
                            ] {
                                let BezierSplitFragment2::AlgebraicChord(chord) = adjacent else {
                                    continue;
                                };
                                assert_eq!(
                                    fillet.certified_adjacent_chord_is_endpoint_only(
                                        chord,
                                        shared_circle_start,
                                        &policy,
                                    ),
                                    Ok(Classification::Decided(true)),
                                    "policy={policy:?}, reversed={reversed}, mode={mode:?}"
                                );
                                chord_adjacencies += 1;
                            }
                        }
                    }
                    assert!(chord_adjacencies > 0);
                    assert!(retained_tangent_relations > 0);
                });
            }
        }
    }
}

#[test]
fn retained_offset_independent_chord_crosses_a_genuine_analytic_parallel() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let triangle = independent_field_algebraic_chord_region(&policy, false);
        let chord = triangle.boundary_loops()[0]
            .fragments()
            .iter()
            .find_map(|fragment| match fragment {
                BezierSplitFragment2::AlgebraicChord(chord) => Some(chord.clone()),
                _ => None,
            })
            .expect("the retained triangle has one algebraic chord");
        let twentieth = (Real::one() / Real::from(20_i8)).unwrap();
        let offset = chord
            .parallel_left_retained(twentieth.clone(), &policy)
            .expect("the independent chord has an exact retained parallel");
        let translation_x = Real::from(2_i8);
        let translation_y = -Real::one();
        let offset = match offset
            .translated(&translation_x, &translation_y, &policy)
            .expect("the retained parallel chord translation is exact")
        {
            Classification::Decided(offset) => offset,
            Classification::Uncertain(reason) => {
                panic!("the retained parallel chord must translate: {reason:?}")
            }
        };
        assert!(matches!(
            (offset.start(), offset.end()),
            (
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)),
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)),
            )
        ));
        assert!(offset.exact_line().is_none());
        assert!(offset.strict_provenance_support_line(&policy).is_none());

        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let translated =
            |point: Point2| point.translated(translation_x.clone(), translation_y.clone());
        let source = QuadraticBezier2::new(
            translated(p(-1, 0)),
            translated(Point2::new(Real::zero(), -quarter)),
            translated(Point2::new(
                Real::one(),
                (Real::one() / Real::from(2_i8)).unwrap(),
            )),
        );
        let parallel = source
            .parallel_left(twentieth)
            .expect("the regular non-PH source has an exact analytic parallel");
        assert!(matches!(
            parallel
                .exact_rational_parallel_component(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided(None)
        ));

        for offset in [offset.clone(), offset.reversed()] {
            let contacts = match offset.parallel_intersections(&parallel, &policy).unwrap() {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                        contacts,
                    ),
                ) => contacts,
                result => {
                    panic!("the retained-offset chord/parallel solve must complete: {result:?}")
                }
            };
            assert!(!contacts.is_empty());
            assert!(
                contacts
                    .iter()
                    .any(|candidate| candidate.tangent_cross_sign() != RealSign::Zero)
            );
        }
    }
}

#[test]
fn retained_stationary_endpoint_composition_keeps_source_and_tangent_anchor() {
    let source = RationalBezier2::try_new(
        vec![
            p(0, 0),
            p(0, 0),
            p(0, 0),
            Point2::new(q(1, 30), Real::zero()),
            Point2::new(q(2, 15), q(1, 10)),
            Point2::new(q(2, 15), q(1, 2)),
        ],
        vec![Real::one(); 6],
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let initial_distance = q(-1, 20);
        assert!(matches!(
            source
                .parallel_left(initial_distance.clone())
                .unwrap()
                .point_at(&Real::zero(), &policy)
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        ));
        for reversed in [false, true] {
            let Classification::Decided(spans) = exact_offset_spans_from_materialized_curve(
                &BezierSubcurve2::Rational(source.clone()),
                &initial_distance,
                &policy,
            )
            .unwrap() else {
                panic!("the stationary PH source has an exact one-sided offset")
            };
            let mut fragment = spans[0].fragments[0].clone();
            if reversed {
                fragment = fragment.reversed().unwrap();
            }
            let mut total_distance = initial_distance.clone();
            for increment in [q(-1, 40), q(-1, 80), q(7, 80)] {
                let old_distance = total_distance.clone();
                total_distance = &total_distance + &increment;
                let distance = if reversed { -increment } else { increment };
                let Classification::Decided(span) =
                    exact_offset_spans_from_retained_parallel_fragment(
                        RetainedParallelOffsetFragmentRef2::from_fragment(&fragment).unwrap(),
                        &distance,
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("the one-sided endpoint must compose and cancel exactly")
                };
                let [span]: [ExactOffsetSpan2; 1] = span.try_into().unwrap_or_else(|_| {
                    panic!("this regular source range must produce one offset span")
                });
                let (point, tangent) = if reversed {
                    (&span.offset_end, span.end_tangent.as_ref().unwrap())
                } else {
                    (&span.offset_start, span.start_tangent.as_ref().unwrap())
                };
                assert_eq!(
                    point.same_point(
                        &CurvePoint2::from(Point2::new(Real::zero(), total_distance.clone())),
                        &policy
                    ),
                    Classification::Decided(true)
                );
                let CurveTangent2::AlgebraicChord(tangent) = tangent else {
                    panic!("the stationary endpoint must retain its limiting tangent")
                };
                assert_eq!(
                    tangent.start().same_point(
                        &CurvePoint2::from(Point2::new(Real::zero(), old_distance.clone())),
                        &policy
                    ),
                    Classification::Decided(true)
                );
                assert_eq!(
                    tangent.end().same_point(
                        &CurvePoint2::from(Point2::new(
                            if reversed { -Real::one() } else { Real::one() },
                            old_distance.clone()
                        )),
                        &policy
                    ),
                    Classification::Decided(true)
                );
                let source_end = if reversed {
                    Point2::new(Real::zero(), old_distance)
                } else {
                    Point2::new(q(2, 15) - old_distance, q(1, 2))
                };
                assert_eq!(
                    span.source_end.same_point(&source_end.into(), &policy),
                    Classification::Decided(true)
                );
                assert_eq!(span.fragments.len(), 1);
                fragment = span.fragments[0].clone();
                let carrier = RetainedParallelOffsetFragmentRef2::from_fragment(&fragment)
                    .unwrap()
                    .parallel();
                assert_eq!(carrier.source_degree(), 5);
                assert_eq!(
                    carrier.source(),
                    &BezierParallelSource2::Rational(source.clone())
                );
                assert_eq!(carrier.distance(), &total_distance);
            }
            assert_eq!(total_distance, Real::zero());
        }
    }
}

fn selected_cusp_parabola_range(
    range: CurveParameterRange2,
    policy: &CurveContext,
) -> (BezierParallel2, BezierSplitFragment2) {
    // P(u)=(2u-1,(2u-1)^2), d=1. Its derivative scale is
    // positive at both endpoints and negative at u=1/2, with two cusps.
    let parallel = QuadraticBezier2::new(p(-1, 1), p(0, -1), p(1, 1))
        .parallel_left(Real::one())
        .unwrap();
    let point = |parameter: &CurveParameter2| {
        let Classification::Decided(point) =
            exact_parallel_region_point_evidence(&parallel, parameter, policy).unwrap()
        else {
            panic!("the selected range has exact endpoints")
        };
        point
    };
    let points = [point(range.start()), point(range.end())];
    let fragment = CurveSupport2::Parallel(parallel.clone())
        .restrict_certified(range, Some(points), false, policy)
        .unwrap();
    assert!(matches!(fragment, BezierSplitFragment2::SelectedFiber(_)));
    (parallel, fragment)
}

#[test]
fn selected_parallel_offset_partitions_existing_cusps_before_composition() {
    let distance = (Real::one() / Real::from(16_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (parallel, fragment) =
            selected_cusp_parabola_range(CurveParameterRange2::unit(), &policy);
        let Classification::Decided(analysis) = parallel
            .singularity_analysis(&CurveParameterRange2::unit(), &policy)
            .unwrap()
        else {
            panic!("the parabola has an exact cusp partition");
        };
        assert_eq!(analysis.parallel_cusps().len(), 2);
        let mut boundaries = vec![CurveParameter2::from(Real::zero())];
        boundaries.extend(
            analysis
                .parallel_cusps()
                .iter()
                .cloned()
                .map(CurveParameter2::from),
        );
        boundaries.push(Real::one().into());
        for reversed in [false, true] {
            let fragment = if reversed {
                fragment.reversed().unwrap()
            } else {
                fragment.clone()
            };
            let Classification::Decided((spans, consumed)) =
                exact_offset_span_from_source_run(&[fragment], 0, 1, &distance, &policy).unwrap()
            else {
                panic!("each regular source branch must offset exactly")
            };
            assert_eq!(
                consumed, 1,
                "branch partitioning preserves authored indexing"
            );
            assert_eq!(
                spans.len(),
                3,
                "the two old cusps separate three oriented spans"
            );
            let delta = if reversed {
                -distance.clone()
            } else {
                distance.clone()
            };
            for (index, span) in spans.iter().enumerate() {
                let expected = if index == 1 {
                    Real::one() - &delta
                } else {
                    Real::one() + &delta
                };
                assert!(!span.fragments.is_empty());
                for part in &span.fragments {
                    let CurveSupport2::Parallel(composed) = CurveSupport2::from_fragment(part)
                    else {
                        panic!("a parabola offset retains its analytic support");
                    };
                    assert_eq!(composed.distance(), &expected);
                }
                let source_index = if reversed { 2 - index } else { index };
                let first = span
                    .fragments
                    .first()
                    .unwrap()
                    .curve_region_parameter_range();
                let last = span
                    .fragments
                    .last()
                    .unwrap()
                    .curve_region_parameter_range();
                let endpoints = if reversed {
                    [last.start(), first.end()]
                } else {
                    [first.start(), last.end()]
                };
                for (actual, expected) in endpoints
                    .into_iter()
                    .zip(&boundaries[source_index..=source_index + 1])
                {
                    assert_eq!(
                        actual.same_value(expected, &policy).unwrap(),
                        Classification::Decided(true)
                    );
                }
            }
        }
    }
}

#[test]
fn selected_parallel_coalescing_requires_regular_cells() {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let (_, first) = selected_cusp_parabola_range(
            CurveParameterRange2::new_validated(Real::zero().into(), half.clone().into()),
            &policy,
        );
        let (_, second) = selected_cusp_parabola_range(
            CurveParameterRange2::new_validated(half.clone().into(), Real::one().into()),
            &policy,
        );
        for reversed in [false, true] {
            let fragments = if reversed {
                vec![second.reversed().unwrap(), first.reversed().unwrap()]
            } else {
                vec![first.clone(), second.clone()]
            };
            let Classification::Decided(coalesced) =
                coalesced_retained_parallel_offset_run(&fragments, 0, 2, &policy).unwrap()
            else {
                panic!("the exact cusp exclusion must decide")
            };
            assert!(
                coalesced.is_none(),
                "equal midpoint signs do not prove regularity across either cusp"
            );
        }
    }
}

#[test]
fn selected_parallel_endpoint_tangents_use_the_incident_branch() {
    let vertical = CurveTangent2::RepresentedDirection((Real::zero(), Real::one()));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (start, end, signs) in [
            (
                Real::zero(),
                Real::one(),
                [RealSign::Positive, RealSign::Positive],
            ),
            (
                Real::zero(),
                q(1, 2),
                [RealSign::Positive, RealSign::Negative],
            ),
            (
                q(1, 2),
                Real::one(),
                [RealSign::Negative, RealSign::Positive],
            ),
        ] {
            let (_, fragment) = selected_cusp_parabola_range(
                CurveParameterRange2::new_validated(start.into(), end.into()),
                &policy,
            );
            for reversed in [false, true] {
                let fragment = if reversed {
                    fragment.reversed().unwrap()
                } else {
                    fragment.clone()
                };
                for at_start in [false, true] {
                    let Classification::Decided(tangent) =
                        CurveTangent2::at_boundary_endpoint(&fragment, at_start, &policy).unwrap()
                    else {
                        panic!("each endpoint has a regular one-sided tangent");
                    };
                    // P'_x=2. Its offset scale is positive at 0 and 1,
                    // negative at 1/2, independently of an interior sample.
                    let sign = signs[usize::from(at_start == reversed)];
                    let expected = if reversed {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => unreachable!(),
                        }
                    } else {
                        sign
                    };
                    assert_eq!(
                        curve_tangent_cross_sign(&tangent, &vertical, &policy),
                        Classification::Decided(expected)
                    );
                }
            }
        }
    }
}

#[test]
fn selected_parallel_offset_preserves_stationary_branch_frames() {
    // P(u)=((2u-1)^3,0). The derivative vanishes at 1/2 without
    // reversing its direction. A regularity proof must retain that cut.
    let parallel = CubicBezier2::new(p(-1, 0), p(1, 0), p(-1, 0), p(1, 0))
        .parallel_left(Real::zero())
        .unwrap();
    let vertical = CurveTangent2::RepresentedDirection((Real::zero(), Real::one()));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selected = |start: Real, end: Real| {
            let range = CurveParameterRange2::new_validated(start.into(), end.into());
            let point = |parameter| {
                let Classification::Decided(point) =
                    exact_parallel_region_point_evidence(&parallel, parameter, &policy).unwrap()
                else {
                    panic!("zero displacement retains the stationary source point")
                };
                point
            };
            let points = [point(range.start()), point(range.end())];
            CurveSupport2::Parallel(parallel.clone())
                .restrict_certified(range, Some(points), false, &policy)
                .unwrap()
        };
        let first = selected(Real::zero(), q(1, 2));
        let second = selected(q(1, 2), Real::one());
        let whole = selected(Real::zero(), Real::one());
        for reversed in [false, true] {
            let split = if reversed {
                vec![second.reversed().unwrap(), first.reversed().unwrap()]
            } else {
                vec![first.clone(), second.clone()]
            };
            let Classification::Decided(coalesced) =
                coalesced_retained_parallel_offset_run(&split, 0, 2, &policy).unwrap()
            else {
                panic!("stationary-source admission must decide")
            };
            assert!(
                coalesced.is_none(),
                "a stationary point is not a regular interior"
            );
            let whole = if reversed {
                whole.reversed().unwrap()
            } else {
                whole.clone()
            };
            let Classification::Decided((spans, consumed)) =
                exact_offset_span_from_source_run(&[whole], 0, 1, &q(1, 16), &policy).unwrap()
            else {
                panic!("both stationary branches have exact offset frames")
            };
            assert_eq!(consumed, 1);
            assert_eq!(spans.len(), 2);
            let height = if reversed { q(-1, 16) } else { q(1, 16) };
            assert_eq!(
                spans[0]
                    .offset_end
                    .same_point(&Point2::new(Real::zero(), height.clone()).into(), &policy),
                Classification::Decided(true)
            );
            assert_eq!(
                spans[1]
                    .offset_start
                    .same_point(&Point2::new(Real::zero(), height).into(), &policy),
                Classification::Decided(true)
            );
            assert_eq!(
                spans[0].source_end.same_point(&p(0, 0).into(), &policy),
                Classification::Decided(true)
            );
            let expected = if reversed {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            for span in spans {
                for tangent in [span.start_tangent, span.end_tangent] {
                    assert_eq!(
                        curve_tangent_cross_sign(&tangent.unwrap(), &vertical, &policy),
                        Classification::Decided(expected)
                    );
                }
            }
        }
    }
}

#[test]
fn retained_parallel_offset_composition_respects_traversal_orientation() {
    let policy = CurveContext::STRICT;
    let tenth = (Real::one() / Real::from(10_i8)).unwrap();
    let fifth = (Real::one() / Real::from(5_i8)).unwrap();
    let source = QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1));
    let parallel = source.parallel_left(-tenth.clone()).unwrap();
    let range = BezierParameterRange2::new_validated(
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    );

    for (reversed, next_distance, expected_distance) in [
        (false, -fifth.clone(), -Real::from(3_i8) * tenth.clone()),
        (true, fifth, -Real::from(3_i8) * tenth),
    ] {
        let fragment = crate::BezierParallelFragment2::from_certified_range(
            parallel.clone(),
            range.clone(),
            reversed,
        );
        let Classification::Decided(span) = exact_offset_spans_from_retained_parallel_fragment(
            RetainedParallelOffsetFragmentRef2::Analytic(&fragment),
            &next_distance,
            &policy,
        )
        .unwrap() else {
            panic!("regular retained parallel composition must be decided");
        };
        let [span]: [ExactOffsetSpan2; 1] = span
            .try_into()
            .unwrap_or_else(|_| panic!("this regular source range must produce one offset span"));
        assert_eq!(span.fragments.len(), 1);
        let BezierSplitFragment2::AnalyticParallel(composed) = &span.fragments[0] else {
            panic!("a non-PH quadratic composition remains analytic");
        };
        assert_eq!(composed.is_reversed(), reversed);
        assert_eq!(composed.parallel().distance(), &expected_distance);
    }
}

#[test]
fn retained_parallel_offset_composition_splits_new_cusps_in_traversal_order() {
    let policy = CurveContext::STRICT;
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(half.clone(), Real::zero()), p(1, 1));
    let parallel = source.parallel_left(Real::zero()).unwrap();
    let range = BezierParameterRange2::new_validated(
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    );
    let cusp_distance = Real::from(2_i8).sqrt().unwrap();

    for (reversed, distance) in [(false, cusp_distance.clone()), (true, -cusp_distance)] {
        let fragment = crate::BezierParallelFragment2::from_certified_range(
            parallel.clone(),
            range.clone(),
            reversed,
        );
        let Classification::Decided(span) = exact_offset_spans_from_retained_parallel_fragment(
            RetainedParallelOffsetFragmentRef2::Analytic(&fragment),
            &distance,
            &policy,
        )
        .unwrap() else {
            panic!("the represented composed cusp must split exactly");
        };
        let [span]: [ExactOffsetSpan2; 1] = span
            .try_into()
            .unwrap_or_else(|_| panic!("this regular source range must produce one offset span"));
        assert_eq!(span.fragments.len(), 2);
        let composed = span
            .fragments
            .iter()
            .map(|fragment| {
                let BezierSplitFragment2::AnalyticParallel(fragment) = fragment else {
                    panic!("a general quadratic cusp split remains analytic");
                };
                assert_eq!(fragment.is_reversed(), reversed);
                fragment.range()
            })
            .collect::<Vec<_>>();
        if reversed {
            assert_eq!(composed[0].start(), &BezierParameter2::Exact(half.clone()));
            assert_eq!(composed[0].end(), &BezierParameter2::Exact(Real::one()));
            assert_eq!(composed[1].start(), &BezierParameter2::Exact(Real::zero()));
            assert_eq!(composed[1].end(), &BezierParameter2::Exact(half.clone()));
        } else {
            assert_eq!(composed[0].start(), &BezierParameter2::Exact(Real::zero()));
            assert_eq!(composed[0].end(), &BezierParameter2::Exact(half.clone()));
            assert_eq!(composed[1].start(), &BezierParameter2::Exact(half.clone()));
            assert_eq!(composed[1].end(), &BezierParameter2::Exact(Real::one()));
        }
    }
}

#[test]
fn retained_parallel_offset_coalesces_non_cusp_algebraic_arrangement_partitions() {
    let construction_policy = CurveContext::STRICT;
    let algebraic = sqrt_half_algebraic_parameter(&construction_policy);
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let range = |start: BezierParameter2, end: BezierParameter2| {
        let range = BezierParameterRange2::try_new(start, end, &construction_policy)
            .expect("the retained parameter range is valid");
        let Classification::Decided(range) = range else {
            panic!("the isolated range ordering must be decided");
        };
        range
    };
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0))
        .parallel_left(Real::zero())
        .expect("the source has an exact analytic parallel");
    let fragment = |range| {
        let fragment =
            crate::BezierParallelFragment2::try_new(parallel.clone(), range, &construction_policy)
                .expect("the regular parallel range is valid");
        let Classification::Decided(fragment) = fragment else {
            panic!("the regular parallel range must be decided");
        };
        fragment
    };
    let first = fragment(range(zero.clone(), algebraic.clone()));
    let second = fragment(range(algebraic, one.clone()));
    assert!(matches!(
        exact_offset_spans_from_retained_parallel_fragment(
            RetainedParallelOffsetFragmentRef2::Analytic(&first),
            &(Real::one() / Real::from(10_i8)).unwrap(),
            &construction_policy,
        ),
        Ok(Classification::Decided(_))
    ));

    let split_fragments = vec![
        BezierSplitFragment2::AnalyticParallel(first.clone()),
        BezierSplitFragment2::AnalyticParallel(second.clone()),
    ];
    let Classification::Decided(Some((coalesced, consumed))) =
        coalesced_retained_parallel_offset_run(
            &split_fragments,
            0,
            split_fragments.len(),
            &construction_policy,
        )
        .expect("the arrangement-only partition is exactly coalescible")
    else {
        panic!("the arrangement-only partition must coalesce");
    };
    assert_eq!(consumed, 2);
    assert!(!coalesced.is_reversed());
    assert_eq!(
        coalesced.range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );

    let reversed_split_fragments = vec![
        BezierSplitFragment2::AnalyticParallel(second.reversed()),
        BezierSplitFragment2::AnalyticParallel(first.reversed()),
    ];
    let Classification::Decided(Some((coalesced_reversed, consumed))) =
        coalesced_retained_parallel_offset_run(
            &reversed_split_fragments,
            0,
            reversed_split_fragments.len(),
            &construction_policy,
        )
        .expect("the reversed arrangement-only partition is exactly coalescible")
    else {
        panic!("the reversed arrangement-only partition must coalesce");
    };
    assert_eq!(consumed, 2);
    assert!(coalesced_reversed.is_reversed());
    assert_eq!(
        coalesced_reversed.range().scalar_endpoints(),
        Some((&Real::zero(), &Real::one()))
    );

    let closed_loop =
        |mut fragments: Vec<BezierSplitFragment2>, reversed: bool, cyclic_seam: bool| {
            fragments.push(if reversed {
                quadratic_fragment(p(0, 0), p(1, 0), p(2, 0))
            } else {
                quadratic_fragment(p(2, 0), p(1, 0), p(0, 0))
            });
            if cyclic_seam {
                fragments.rotate_left(1);
                assert!(matches!(
                    fragments.first(),
                    Some(BezierSplitFragment2::AnalyticParallel(first))
                        if analytic_parallel_traversal_start(first).scalar().is_none()
                ));
            }
            CurveRegion2::try_new_with_loop_topology(
                vec![
                    CurveRegionBoundaryLoop2::new(fragments, &construction_policy)
                        .expect("the algebraic partition retains exact connectivity"),
                ],
                vec![CurveRegionLoopRole::Material],
                vec![FillRule::NonZero],
                vec![if reversed {
                    CurveBoundaryInteriorSide2::Left
                } else {
                    CurveBoundaryInteriorSide2::Right
                }],
            )
            .expect("the exact cap topology is authored")
        };
    let unsplit = fragment(range(zero, one));
    let region_pairs = [
        (
            closed_loop(split_fragments.clone(), false, false),
            closed_loop(
                vec![BezierSplitFragment2::AnalyticParallel(unsplit.clone())],
                false,
                false,
            ),
        ),
        (
            closed_loop(split_fragments, false, true),
            closed_loop(
                vec![BezierSplitFragment2::AnalyticParallel(unsplit.clone())],
                false,
                false,
            ),
        ),
        (
            closed_loop(reversed_split_fragments.clone(), true, false),
            closed_loop(
                vec![BezierSplitFragment2::AnalyticParallel(unsplit.reversed())],
                true,
                false,
            ),
        ),
        (
            closed_loop(reversed_split_fragments, true, true),
            closed_loop(
                vec![BezierSplitFragment2::AnalyticParallel(unsplit.reversed())],
                true,
                false,
            ),
        ),
    ];
    let distance = (Real::one() / Real::from(10_i8)).unwrap();
    for (split_region, unsplit_region) in region_pairs {
        let mut reference = None;
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let split_offset = split_region
                .offset_with_policy(distance.clone(), &OffsetCornerStyle2::Round, &policy)
                .expect("the algebraically partitioned exact offset must complete");
            let unsplit_offset = unsplit_region
                .offset_with_policy(distance.clone(), &OffsetCornerStyle2::Round, &policy)
                .expect("the equivalent unsplit exact offset must complete");
            assert_eq!(split_offset.certainty, unsplit_offset.certainty);
            if policy == CurveContext::STRICT {
                assert_eq!(split_offset.certainty, CurveCertainty::Certified);
            }
            assert_eq!(split_offset.value, unsplit_offset.value);
            if let Some(reference) = &reference {
                assert_eq!(&split_offset.value, reference);
            } else {
                reference = Some(split_offset.value);
            }
        }
    }
}

#[test]
fn retained_parallel_offset_preserves_algebraic_cusp_partition() {
    let construction_policy = CurveContext::STRICT;
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    let parallel = CubicBezier2::new(p(0, 0), p(0, 4), p(4, -4), p(4, 0))
        .parallel_left(half)
        .expect("the source has an exact analytic parallel");
    let analysis = parallel
        .singularity_analysis(&CurveParameterRange2::unit(), &construction_policy)
        .expect("the parallel cusp analysis is valid");
    let Classification::Decided(analysis) = analysis else {
        panic!("the exact cusp analysis must be decided");
    };
    let [cusp, next_cusp] = analysis.parallel_cusps() else {
        panic!("the selected parallel must have two cusps");
    };
    assert!(cusp.scalar().is_none());
    let make_fragment = |start: BezierParameter2, end: BezierParameter2| {
        let range = BezierParameterRange2::try_new(start, end, &construction_policy)
            .expect("the cusp range is valid");
        let Classification::Decided(range) = range else {
            panic!("the cusp range ordering must be decided");
        };
        let fragment =
            crate::BezierParallelFragment2::try_new(parallel.clone(), range, &construction_policy)
                .expect("a cusp is permitted at a retained range endpoint");
        let Classification::Decided(fragment) = fragment else {
            panic!("the cusp-bounded regular fragment must be decided");
        };
        fragment
    };
    let first = make_fragment(BezierParameter2::Exact(Real::zero()), cusp.clone());
    let second = make_fragment(cusp.clone(), next_cusp.clone());
    let fragments = vec![
        BezierSplitFragment2::AnalyticParallel(first.clone()),
        BezierSplitFragment2::AnalyticParallel(second.clone()),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first_scale = first
            .parallel()
            .regular_fragment_derivative_scale_sign(first.range(), &policy)
            .expect("the first limiting branch scale is valid");
        let second_scale = second
            .parallel()
            .regular_fragment_derivative_scale_sign(second.range(), &policy)
            .expect("the second limiting branch scale is valid");
        let (Classification::Decided(first_scale), Classification::Decided(second_scale)) =
            (first_scale, second_scale)
        else {
            panic!("both cusp-side derivative scales must be decided");
        };
        assert_ne!(first_scale, second_scale);
        assert_eq!(
            coalesced_retained_parallel_offset_run(&fragments, 0, fragments.len(), &policy)
                .expect("the cusp partition decision is exact"),
            Classification::Decided(None),
        );
    }
}

#[test]
fn regularized_analytic_loop_roles_follow_exact_nesting() {
    fn analytic_loop(
        radius: i32,
        center_x: i32,
        source_base: usize,
        policy: &CurveContext,
    ) -> CurveRegionBoundaryLoop2 {
        let sources = [
            QuadraticBezier2::new(
                p(center_x + radius, 0),
                p(center_x + radius, radius),
                p(center_x, radius),
            ),
            QuadraticBezier2::new(
                p(center_x, radius),
                p(center_x - radius, radius),
                p(center_x - radius, 0),
            ),
            QuadraticBezier2::new(
                p(center_x - radius, 0),
                p(center_x - radius, -radius),
                p(center_x, -radius),
            ),
            QuadraticBezier2::new(
                p(center_x, -radius),
                p(center_x + radius, -radius),
                p(center_x + radius, 0),
            ),
        ];
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        );
        let fragments = sources
            .into_iter()
            .map(|source| {
                BezierSplitFragment2::AnalyticParallel(
                    crate::BezierParallelFragment2::from_certified_range(
                        source.parallel_left(Real::zero()).unwrap(),
                        range.clone(),
                        false,
                    ),
                )
            })
            .collect::<Vec<_>>();
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
        CurveRegionBoundaryLoop2::try_new_from_certified_arrangement_chain(
            boundary.fragments,
            (0..4)
                .map(|index| {
                    CurveRegionFragmentSource2::new(source_base + index, source_base + index, 0)
                })
                .collect(),
            policy,
        )
        .unwrap()
    }

    let policy = CurveContext::STRICT;
    let nested = CurveRegion2::new(vec![
        analytic_loop(4, 0, 0, &policy),
        analytic_loop(2, 0, 4, &policy),
    ])
    .unwrap()
    .with_regularized_filled_left_topology(&policy)
    .unwrap();
    assert_eq!(
        nested.loop_roles_raw(&policy),
        Ok(Classification::Decided(vec![
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Hole,
        ]))
    );

    let disjoint = CurveRegion2::new(vec![
        analytic_loop(4, 0, 0, &policy),
        analytic_loop(2, 10, 4, &policy),
    ])
    .unwrap()
    .with_regularized_filled_left_topology(&policy)
    .unwrap();
    assert_eq!(
        disjoint.loop_roles_raw(&policy),
        Ok(Classification::Decided(vec![
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Material,
        ]))
    );
}

#[test]
fn regularized_composite_chord_roles_preserve_nested_islands_and_profiles() {
    for (policy, reversed) in [
        (CurveContext::STRICT, false),
        (CurveContext::STRICT, true),
        (CurveContext::APPROXIMATE_512, false),
        (CurveContext::APPROXIMATE_512, true),
    ] {
        let base = independent_oblique_chord_pair_corner_region(&policy, reversed);
        let center_x = ((&q(1, 2).sqrt().unwrap() + Real::one()) / Real::from(3_i8)).unwrap();
        let center_y = (&q(1, 3).sqrt().unwrap() + q(1, 5).sqrt().unwrap()) / Real::from(3_i8);
        let center_y = center_y.unwrap();
        let nested_loop = |scale: i32| {
            let scale = Real::from(scale);
            let transformed = base
                .transform_affine_with_policy(
                    &scale,
                    &Real::zero(),
                    &Real::zero(),
                    &scale,
                    &-(&scale * &center_x),
                    &-(&scale * &center_y),
                    &policy,
                )
                .expect("a homothetic retained chord loop must transform exactly");
            assert_eq!(transformed.certainty, CurveCertainty::Certified);
            let [boundary] = transformed
                .value
                .into_boundary_loops()
                .try_into()
                .expect("the transformed triangle retains one loop");
            let sample = retained_loop_sample_point_evidence(&boundary, &policy)
                .expect("the transformed loop has exact sample evidence");
            assert!(matches!(
                sample,
                Classification::Decided(
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                        | CurvePoint2(CurvePointData2::Similarity(_))
                )
            ));
            boundary
        };
        let region = CurveRegion2::new(vec![nested_loop(3), nested_loop(2), nested_loop(1)])
            .expect("homothetic triangles are valid retained loops");
        let expected = vec![
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Hole,
            CurveRegionLoopRole::Material,
        ];
        assert_eq!(
            region.regularized_retained_loop_roles_raw(&policy),
            Ok(Classification::Decided(expected.clone()))
        );
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let regularize = || region.regularized_region_with_policy(&policy);
        #[cfg(feature = "dispatch-trace")]
        let regularized = hyperreal::dispatch_trace::with_recording(regularize);
        #[cfg(not(feature = "dispatch-trace"))]
        let regularized = regularize();
        #[cfg(feature = "dispatch-trace")]
        let trace = hyperreal::dispatch_trace::take_trace();
        let regularized = regularized
            .expect("nested composite chord loops must regularize without materialization");
        assert_eq!(regularized.certainty, CurveCertainty::Certified);
        #[cfg(feature = "dispatch-trace")]
        assert!(
            trace.path_count(
                "hypercurve",
                "curve-region-regularization-chord-side",
                "retained-endpoint-winding-probe",
            ) > 0,
            "composite chord regularization must retain the endpoint winding theorem: {trace:?}",
        );
        assert_eq!(regularized.value.boundary_loops().len(), 3);
        assert!(regularized.value.boundary_loops().iter().all(|boundary| {
            boundary
                .fragments()
                .iter()
                .all(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
        }));
        assert_eq!(
            regularized.value.loop_roles_raw(&policy),
            Ok(Classification::Decided(expected.clone()))
        );
        let profiled = region
            .with_certified_loop_roles(expected)
            .expect("the exact nesting roles match the retained loops");
        let profiles = profiled
            .boundary_profiles_with_policy(&policy)
            .expect("composite point evidence must assign hole ownership");
        assert_eq!(profiles.certainty, CurveCertainty::Certified);
        let Classification::Decided(profiles) = profiles.value else {
            panic!("composite hole ownership must be decided");
        };
        assert_eq!(profiles.len(), 2);
        assert_eq!(profiles[0].material_loop_index(), 0);
        assert_eq!(profiles[0].hole_loop_indices(), &[1]);
        assert_eq!(profiles[1].material_loop_index(), 2);
        assert!(profiles[1].hole_loop_indices().is_empty());
    }
}

#[test]
fn exact_line_fragment_lowering_rejects_nonlinear_algebraic_source() {
    let policy = CurveContext::STRICT;
    let polynomial = BezierParameterPolynomial::try_new_power_basis(
        vec![Real::from(-1_i8), Real::zero(), Real::from(2_i8)],
        &policy,
    )
    .expect("the quadratic parameter polynomial is valid");
    let Classification::Decided(polynomial) = polynomial else {
        panic!("the exact polynomial must be decided");
    };
    let interval = BezierParameterInterval::try_new(
        (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
        (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
        &policy,
    )
    .expect("the isolating interval is valid");
    let Classification::Decided(interval) = interval else {
        panic!("the exact interval must be decided");
    };
    let parameter = BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
        .expect("sqrt(1/2) has one root in the supplied interval");
    let Classification::Decided(parameter) = parameter else {
        panic!("the exact algebraic parameter must be decided");
    };

    // In power form this is `(x, y) = (2t^2 - 1, 2t^3 - t)`.
    // Both coordinates are exactly zero at the irrational split while the
    // source image itself is not a line.
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(Real::from(-1_i8), Real::zero()),
            Point2::new(
                Real::from(-1_i8),
                (Real::from(-1_i8) / Real::from(3_i8)).unwrap(),
            ),
            Point2::new(
                (Real::from(-1_i8) / Real::from(3_i8)).unwrap(),
                (Real::from(-2_i8) / Real::from(3_i8)).unwrap(),
            ),
            Point2::new(Real::one(), Real::one()),
        ],
        vec![Real::one(); 4],
    )
    .expect("the polynomial cubic has a rational Bezier representation");
    let split = source
        .split_at_parameters(&[BezierParameter2::Algebraic(parameter)], &policy)
        .expect("the exact algebraic split is constructible");
    let Classification::Decided(split) = split else {
        panic!("the exact algebraic split must be decided");
    };
    assert!(matches!(
        retained_line_fragment_segment(&split.fragments()[0], &policy),
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    ));
}

#[test]
fn retained_boundary_identity_requires_finite_affine_endpoints() {
    // The line has W(t)=1-2t; the conic has W(t)=(1-2t)^2.
    // Their homogeneous numerators are nonzero at t=1/2, so that
    // parameter is a true pole, even when both fragments share it.
    let sources = [
        BezierSubcurve2::Rational(
            RationalBezier2::try_new(vec![p(0, 0), p(2, 0)], vec![Real::one(), -Real::one()])
                .unwrap(),
        ),
        BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(
                p(0, 0),
                p(1, 1),
                p(2, 0),
                Real::one(),
                -Real::one(),
                Real::one(),
            )
            .unwrap(),
        ),
    ];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (family, source) in sources.iter().enumerate() {
            for (range, (start, end, finite)) in [
                (Real::zero(), q(1, 4), true),
                (q(3, 4), Real::one(), true),
                (Real::zero(), q(1, 2), false),
                (q(1, 2), Real::one(), false),
            ]
            .into_iter()
            .enumerate()
            {
                let fragments =
                    [false, true].map(|reversed| BezierSplitFragment2::RetainedBezier {
                        source_curve: source.clone(),
                        start: BezierParameter2::Exact(start.clone()),
                        end: BezierParameter2::Exact(end.clone()),
                        reversed,
                        start_image: None,
                        end_image: None,
                    });
                assert_eq!(
                    CurveRegionBoundaryLoop2::new(fragments.into(), &policy).is_ok(),
                    finite,
                    "shared parameter identity needs affine endpoints: policy={policy:?}, family={family}, range={range}",
                );
            }
        }
    }
}

#[test]
fn curve_region_is_one_word_and_empty_data_is_process_shared() {
    assert!(
        core::mem::size_of::<PolicyEvaluationCache<Option<Real>>>()
            <= core::mem::size_of::<OnceLock<CurveResult<Option<Real>>>>()
                + core::mem::size_of::<usize>(),
        "policy-aware signed-area caching must add at most one alignment word"
    );
    let first = CurveRegion2::empty();
    let second = CurveRegion2::default();

    assert_eq!(
        core::mem::size_of::<CurveRegion2>(),
        core::mem::size_of::<usize>()
    );
    assert!(Arc::ptr_eq(&first.data, &second.data));
    assert!(first.clone().into_boundary_loops().is_empty());
}

#[test]
fn authored_region_clones_share_certified_normalization_without_retaining_the_input() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0))),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(0, 0)).unwrap()),
    ])
    .unwrap();
    for first_policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let authored = CurveRegion2::try_from_boundary_paths_raw(
            std::slice::from_ref(&path),
            &CurveContext::STRICT,
        )
        .unwrap();
        let source = Arc::downgrade(&authored.data);
        let cloned = authored.clone();
        let normalized = authored
            .regularized_region_with_policy(&first_policy)
            .unwrap();
        assert_eq!(normalized.certainty, CurveCertainty::Certified);
        assert!(!Arc::ptr_eq(&normalized.value.data, &authored.data));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let replay = cloned.regularized_region_with_policy(&policy).unwrap();
            assert_eq!(replay.certainty, CurveCertainty::Certified);
            assert!(Arc::ptr_eq(&normalized.value.data, &replay.value.data));
        }
        drop(cloned);
        drop(authored);
        assert!(
            source.upgrade().is_none(),
            "normalization must not retain its input region"
        );
        assert_eq!(
            normalized
                .value
                .classify_point_with_policy(
                    &Point2::new(Real::one(), (Real::one() / Real::from(2)).unwrap()).into(),
                    &CurveContext::STRICT,
                )
                .unwrap()
                .value,
            Classification::Decided(RegionPointLocation::Inside),
        );
    }
}

#[test]
fn raw_crossing_loops_need_arrangement_before_nesting() {
    let curved = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![p(-2, 0), p(0, 4), p(2, 0)],
                vec![Real::one(), Real::from(2), Real::one()],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(-2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-2, -2), p(-2, 0)).unwrap()),
    ])
    .unwrap();
    let corners = [p(-1, 2), p(1, 2), p(1, 5), p(-1, 5), p(-1, 2)];
    let cutter = CurvePath2::try_new(
        corners
            .windows(2)
            .map(|edge| Curve2::from(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap()))
            .collect(),
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let raw =
            CurveRegion2::try_from_boundary_paths_raw(&[curved.clone(), cutter.clone()], &policy)
                .unwrap();
        assert!(matches!(
            raw.native_loop_nesting_raw(&policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        ));
        assert_eq!(
            raw.loop_roles_raw(&policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }
}

#[test]
fn boundary_path_construction_obeys_selected_terminal_policy() {
    let start = Point2::new(Real::e().sin(), Real::zero());
    let end = Point2::new(Real::e().sin() + terminally_unresolved_zero(), Real::zero());
    let path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
        start,
        p(0, 1),
        end,
    ))])
    .expect("one-curve path construction has no adjacency decision");

    let strict = CurveRegion2::try_from_boundary_paths_with_policy(
        std::slice::from_ref(&path),
        crate::FillRule::EvenOdd,
        &CurveContext::STRICT,
    )
    .expect_err("strict construction must preserve an undecidable closure");
    assert!(matches!(
        strict,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Construction
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let approximate = CurveRegion2::try_from_boundary_paths_with_policy(
        std::slice::from_ref(&path),
        crate::FillRule::EvenOdd,
        &CurveContext::APPROXIMATE_512,
    )
    .expect("the authorized terminal equality must close the path");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(approximate.value.is_empty());

    let exact_start = p(0, 0);
    let exact_path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
        exact_start.clone(),
        p(1, 1),
        exact_start,
    ))])
    .unwrap();
    let exact = CurveRegion2::try_from_boundary_paths_with_policy(
        &[exact_path],
        crate::FillRule::EvenOdd,
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    assert!(exact.is_empty());
    assert!(exact.data.strict_materialized_connectivity_certified);
}

#[test]
fn rational_line_measurements_obey_policy_and_isolate_cached_certainty() {
    let undecidable_zero = terminally_unresolved_zero();
    let line_y = || Real::one() + &undecidable_zero;
    let rational = RationalBezier2::try_new(
        vec![
            p(1, 1),
            Point2::new(Real::from(2_i8), line_y()),
            Point2::new(Real::from(3_i8), line_y()),
            Point2::new(Real::from(4_i8), line_y()),
            p(5, 1),
        ],
        vec![
            Real::one(),
            Real::from(2_i8),
            Real::from(3_i8),
            Real::from(5_i8),
            Real::from(10_i8),
        ],
    )
    .expect("positive weights define a finite rational curve");
    assert_eq!(rational.signed_area_contribution().unwrap(), None);
    assert_eq!(rational.area_moments_contribution().unwrap(), None);

    let curve = BezierSubcurve2::Rational(rational);
    let strict_area = curve
        .signed_area_contribution(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(strict_area.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict_area.value,
        Classification::Uncertain(UncertaintyReason::RealSign)
    );
    let approximate_area = curve
        .signed_area_contribution(&CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate_area.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate_area.value,
        Classification::Decided(Some(Real::from(-2_i8)))
    );

    let strict_moments = curve
        .area_moments_contribution(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(strict_moments.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict_moments.value,
        Classification::Uncertain(UncertaintyReason::RealSign)
    );
    let expected_moments = BezierAreaMoments2::line_contribution(&p(1, 1), &p(5, 1)).unwrap();
    let approximate_moments = curve
        .area_moments_contribution(&CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate_moments.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate_moments.value,
        Classification::Decided(Some(expected_moments))
    );

    let loop_ = CurveRegionBoundaryLoop2::new(
        vec![
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve,
            },
            quadratic_fragment(p(5, 1), p(5, 2), p(5, 3)),
            quadratic_fragment(p(5, 3), p(3, 3), p(1, 3)),
            quadratic_fragment(p(1, 3), p(1, 2), p(1, 1)),
        ],
        &CurveContext::STRICT,
    )
    .expect("exact endpoints close the retained loop");
    let region = CurveRegion2::new(vec![loop_]).expect("one retained loop");

    let approximate = region
        .signed_area_with_policy(&CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        Classification::Decided(Some(Real::from(8_i8)))
    );
    let strict = region
        .signed_area_with_policy(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict.value,
        Classification::Uncertain(UncertaintyReason::RealSign)
    );
    assert_eq!(
        region
            .signed_area_with_policy(&CurveContext::APPROXIMATE_512)
            .unwrap()
            .certainty,
        CurveCertainty::Approximate512Consumed
    );
}

#[test]
fn curve_region_mutations_report_selected_terminal_policy() {
    let undecidable_zero = terminally_unresolved_zero();
    let region = single_quadratic_loop_region(false);

    let scale = Real::one() + &undecidable_zero;
    let strict_transform = region
        .transform_affine_with_policy(
            &scale,
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &CurveContext::STRICT,
        )
        .expect("the determinant is certified positive without deciding the symbolic zero");
    assert_eq!(strict_transform.certainty, CurveCertainty::Certified);
    let approximate_transform = region
        .transform_affine_with_policy(
            &scale,
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &CurveContext::APPROXIMATE_512,
        )
        .expect("the same certified determinant is valid under the broader policy");
    assert_eq!(approximate_transform.certainty, CurveCertainty::Certified);
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(first),
        ..
    } = &approximate_transform.value.boundary_loops()[0].fragments()[0]
    else {
        panic!("the transformed quadratic boundary must remain materialized");
    };
    let expected_control = affine_region_point(
        &p(1, 0),
        &scale,
        &Real::zero(),
        &Real::zero(),
        &Real::one(),
        &Real::zero(),
        &Real::zero(),
    );
    assert_eq!(
        first.control(),
        &expected_control,
        "the terminal policy must not replace transformed coordinates"
    );

    let identity = crate::Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::zero(),
    )
    .unwrap();
    assert_eq!(
        region
            .transform_similarity_with_policy(&identity, &CurveContext::APPROXIMATE_512)
            .unwrap()
            .certainty,
        CurveCertainty::Certified
    );

    let bent = CurveRegion2::new(vec![
        CurveRegionBoundaryLoop2::new(
            vec![
                quadratic_fragment(
                    p(0, 0),
                    Point2::new(Real::one(), Real::one() + &undecidable_zero),
                    p(2, 0),
                ),
                quadratic_fragment(p(2, 0), p(2, 1), p(2, 2)),
                quadratic_fragment(p(2, 2), p(1, 2), p(0, 2)),
                quadratic_fragment(p(0, 2), p(0, 1), p(0, 0)),
            ],
            &CurveContext::STRICT,
        )
        .unwrap(),
    ])
    .unwrap();
    let flattening =
        BezierFlatteningOptions::try_new(Real::one(), 4, &CurveContext::STRICT).unwrap();
    let strict_segmentation = bent
        .segment_certified_with_policy(&flattening, &CurveContext::STRICT)
        .unwrap();
    assert_eq!(strict_segmentation.certainty, CurveCertainty::Certified);
    assert!(matches!(
        strict_segmentation.value,
        Classification::Uncertain(UncertaintyReason::Ordering)
    ));
    let approximate_segmentation = bent
        .segment_certified_with_policy(&flattening, &CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate_segmentation.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(matches!(
        approximate_segmentation.value,
        Classification::Decided(_)
    ));

    let collapse_distance = -Real::one() + &undecidable_zero;
    let strict_offset = region.offset_with_policy(
        collapse_distance.clone(),
        &OffsetCornerStyle2::Round,
        &CurveContext::STRICT,
    );
    assert!(matches!(strict_offset, Err(ExactCurveError::Blocked(_))));
    let approximate_offset = region
        .offset_with_policy(
            collapse_distance,
            &OffsetCornerStyle2::Round,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate_offset.certainty,
        CurveCertainty::Approximate512Consumed
    );
}

fn quadratic_fragment(start: Point2, control: Point2, end: Point2) -> BezierSplitFragment2 {
    BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, control, end)),
    }
}

fn rational_quadratic_fragment(
    start: Point2,
    control: Point2,
    end: Point2,
) -> BezierSplitFragment2 {
    let half = (Real::one() / Real::from(2_i8)).unwrap();
    BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(start, control, end, Real::one(), half, Real::one())
                .unwrap(),
        ),
    }
}

fn single_rational_quadratic_loop_region() -> CurveRegion2 {
    let fragments = vec![
        rational_quadratic_fragment(p(0, 0), p(1, 0), p(2, 0)),
        rational_quadratic_fragment(p(2, 0), p(2, 1), p(2, 2)),
        rational_quadratic_fragment(p(2, 2), p(1, 2), p(0, 2)),
        rational_quadratic_fragment(p(0, 2), p(0, 1), p(0, 0)),
    ];
    CurveRegion2::new(vec![
        CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT)
            .expect("closed retained rational-quadratic loop"),
    ])
    .expect("one retained rational-quadratic loop")
}

fn single_quadratic_loop_region(clockwise: bool) -> CurveRegion2 {
    let fragments = if clockwise {
        vec![
            quadratic_fragment(p(0, 0), p(0, 1), p(0, 2)),
            quadratic_fragment(p(0, 2), p(1, 2), p(2, 2)),
            quadratic_fragment(p(2, 2), p(2, 1), p(2, 0)),
            quadratic_fragment(p(2, 0), p(1, 0), p(0, 0)),
        ]
    } else {
        vec![
            quadratic_fragment(p(0, 0), p(1, 0), p(2, 0)),
            quadratic_fragment(p(2, 0), p(2, 1), p(2, 2)),
            quadratic_fragment(p(2, 2), p(1, 2), p(0, 2)),
            quadratic_fragment(p(0, 2), p(0, 1), p(0, 0)),
        ]
    };
    CurveRegion2::new(vec![
        CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT)
            .expect("closed retained quadratic loop"),
    ])
    .expect("one retained loop")
}

#[test]
fn curve_region_clones_share_geometry_and_lazy_caches() {
    let region = single_quadratic_loop_region(false);
    let clone = region.clone();
    let policy = CurveContext::STRICT;

    assert!(Arc::ptr_eq(&region.data, &clone.data));
    assert!(region.data.signed_area_cache.is_empty());
    let clone_area = clone
        .signed_area_with_policy(&policy)
        .expect("clone area")
        .into_value();
    assert!(matches!(clone_area, Classification::Decided(Some(_))));
    assert!(!region.data.signed_area_cache.is_empty());
    assert_eq!(
        clone_area,
        region
            .signed_area_with_policy(&policy)
            .expect("source area")
            .into_value()
    );

    let cloned_loops = clone.into_boundary_loops();
    assert_eq!(cloned_loops, region.boundary_loops());
    assert_eq!(region.len(), 1);
}

#[test]
fn single_loop_filled_side_uses_area_without_constructing_nesting_bounds() {
    let policy = CurveContext::STRICT;
    for (clockwise, expected) in [(false, true), (true, false)] {
        let region = single_quadratic_loop_region(clockwise);
        assert!(region.data.native_boundary_bounds.is_empty());
        assert!(matches!(
            region.filled_side_is_left_with_policy(&policy),
            Ok(CurveOutcome {
                value: Classification::Decided(sides),
                ..
            }) if sides == [expected]
        ));
        assert!(region.data.native_boundary_bounds.is_empty());
    }
}

#[test]
fn native_query_bounds_use_exact_conservative_control_hulls() {
    let policy = CurveContext::STRICT;
    let cubic = CubicBezier2::new(p(0, 0), p(0, 6), p(4, 6), p(4, 0));
    let curve = BezierSubcurve2::Cubic(cubic.clone());
    let query_bounds = match subcurve_query_bounds(&curve, &policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => {
            panic!("polynomial control hull unexpectedly uncertain: {reason:?}")
        }
    };
    let control_hull = match Aabb2::from_points(cubic.control_points()) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => {
            panic!("polynomial control hull unexpectedly uncertain: {reason:?}")
        }
    };
    let tight_bounds = match cubic.certified_bounds() {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => {
            panic!("cubic tight bounds unexpectedly uncertain: {reason:?}")
        }
    };

    assert_eq!(query_bounds, control_hull);
    assert_eq!(
        compare_reals(query_bounds.max().y(), tight_bounds.max().y(), &policy),
        Some(Ordering::Greater)
    );
    for numerator in 0_i32..=8 {
        let parameter = (Real::from(numerator) / Real::from(8_i32)).unwrap();
        assert_eq!(
            query_bounds.contains_point(&cubic.point_at(parameter), &policy),
            Classification::Decided(true)
        );
    }
}

#[test]
fn independent_region_orientations_share_equal_conic_area_kernels() {
    let policy = CurveContext::STRICT;
    let first = single_rational_quadratic_loop_region();
    let second = single_rational_quadratic_loop_region();
    let mut cache = RationalQuadraticAreaIntegralCache::default();

    assert!(matches!(
        first.filled_side_is_left_with_area_cache(&policy, &mut cache),
        Ok(Classification::Decided([true]))
    ));
    assert_eq!(cache.retained_integral_count(), 1);
    assert!(matches!(
        second.filled_side_is_left_with_area_cache(&policy, &mut cache),
        Ok(Classification::Decided([true]))
    ));
    assert_eq!(cache.retained_integral_count(), 1);
}

#[test]
fn retained_subcurve_point_query_preserves_projective_denominator_uncertainty() {
    let conic = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(1, 0),
        p(2, 0),
        1.into(),
        (-1).into(),
        1.into(),
    )
    .unwrap();
    let subcurve = BezierSubcurve2::RationalQuadratic(conic);

    assert_eq!(
        subcurve_contains_point(&subcurve, &p(100, 0), &CurveContext::STRICT),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn irrational_weight_semicircle_region_recovers_exact_native_accelerator() {
    let policy = CurveContext::STRICT;
    let arcs = [
        CircularArc2::try_from_center(p(0, 0), p(2, 0), p(1, 0), true).unwrap(),
        CircularArc2::try_from_center(p(2, 0), p(0, 0), p(1, 0), true).unwrap(),
    ];
    let mut curves = Vec::with_capacity(4);
    for arc in arcs {
        for span in arc
            .rational_bezier_decomposition(&policy)
            .unwrap()
            .into_value()
            .spans()
        {
            let curve = span.curve();
            let controls = curve.control_points();
            let weights = curve.weights();
            curves.push(Curve2::from(
                RationalQuadraticBezier2::try_new(
                    controls[0].clone(),
                    controls[1].clone(),
                    controls[2].clone(),
                    weights[0].clone(),
                    weights[1].clone(),
                    weights[2].clone(),
                )
                .unwrap(),
            ));
        }
    }
    let region = CurveRegion2::try_from_boundary_paths_with_policy(
        &[CurvePath2::try_new(curves).unwrap()],
        crate::FillRule::EvenOdd,
        &policy,
    )
    .unwrap()
    .into_value();
    let point = Point2::new(Real::one(), (Real::one() / Real::from(2_u8)).unwrap());
    assert_eq!(
        region
            .classify_point_with_policy(&point.clone().into(), &policy)
            .map(CurveOutcome::into_value),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert_eq!(
        region
            .classify_point_with_policy(&p(1, 1).into(), &policy)
            .map(CurveOutcome::into_value),
        Ok(Classification::Decided(RegionPointLocation::Boundary))
    );
    assert_eq!(
        region
            .classify_point_with_policy(&p(1, 2).into(), &policy)
            .map(CurveOutcome::into_value),
        Ok(Classification::Decided(RegionPointLocation::Outside))
    );
    assert!(matches!(
        region.data.line_image_region.certified(),
        Some(Some(_))
    ));
}

#[test]
fn nonuniform_rational_line_images_use_exact_geometric_moments() {
    let expected = BezierAreaMoments2::line_contribution(&p(2, 0), &p(4, 2)).unwrap();
    let quadratic = BezierSubcurve2::RationalQuadratic(
        RationalQuadraticBezier2::try_new(
            p(2, 0),
            p(3, 1),
            p(4, 2),
            Real::one(),
            Real::from(2),
            Real::from(3),
        )
        .unwrap(),
    );
    assert_eq!(
        quadratic
            .area_moments_contribution(&CurveContext::STRICT)
            .unwrap()
            .into_value(),
        Classification::Decided(Some(expected.clone()))
    );

    let rational = BezierSubcurve2::Rational(
        RationalBezier2::try_new(
            vec![p(2, 0), p(3, 1), p(4, 2)],
            vec![Real::one(), Real::from(3), Real::from(5)],
        )
        .unwrap(),
    );
    assert_eq!(
        rational
            .area_moments_contribution(&CurveContext::STRICT)
            .unwrap()
            .into_value(),
        Classification::Decided(Some(expected))
    );
}

#[test]
fn explicit_signed_loops_classify_after_regularization() {
    fn rectangle(min_x: i32, max_x: i32) -> CurvePath2 {
        let corners = [p(min_x, -3), p(max_x, -3), p(max_x, 3), p(min_x, 3)];
        CurvePath2::try_new(
            (0..4)
                .map(|index| {
                    Curve2::from(
                        LineSeg2::try_new(corners[index].clone(), corners[(index + 1) % 4].clone())
                            .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap()
    }

    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics_with_policy(
        &[rectangle(-3, 3), rectangle(1, 7)],
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
        &[FillRule::NonZero, FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();
    assert_eq!(
        region
            .classify_point_with_policy(&p(-2, 0).into(), &policy)
            .map(CurveOutcome::into_value),
        Ok(Classification::Decided(RegionPointLocation::Inside))
    );
    assert_eq!(
        region
            .classify_point_with_policy(&p(2, 0).into(), &policy)
            .map(CurveOutcome::into_value),
        Ok(Classification::Decided(RegionPointLocation::Outside))
    );
}
