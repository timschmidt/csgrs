mod support;

use hypercurve::{
    BezierParameter2, CurveCertainty, CurveFamily2, CurveOperation2, ExactCurveError,
    QuadraticBezier2, RationalQuadraticBezier2, RegionPointLocation, UncertaintyReason,
};
use hypercurve::{
    BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurveBoundaryInteriorSide2,
    CurveContext, CurveGeometry2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule,
    LineSeg2, Point2, RationalBezier2, RationalBezierOverlapOrientation2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn symbolic_rectangle_path(width: Real) -> CurvePath2 {
    let points = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(width.clone(), Real::zero()),
        Point2::new(width, Real::one()),
        Point2::new(Real::zero(), Real::one()),
    ];
    CurvePath2::try_new(
        (0..points.len())
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

#[test]
fn curve_parameter_comparison_reports_terminal_certainty() {
    let zero = hypercurve::CurveParameter2::from(Real::zero());
    let unresolved = hypercurve::CurveParameter2::from(support::terminally_unresolved_zero());
    let approximate = zero
        .compare(&unresolved, &CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        Classification::Decided(std::cmp::Ordering::Equal)
    );
    let strict = zero.compare(&unresolved, &CurveContext::STRICT).unwrap();
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert!(matches!(strict.value, Classification::Uncertain(_)));
    let identity = unresolved
        .compare(&unresolved, &CurveContext::STRICT)
        .unwrap();
    assert_eq!(identity.certainty, CurveCertainty::Certified);
    assert_eq!(
        identity.value,
        Classification::Decided(std::cmp::Ordering::Equal)
    );
}

fn path_region(
    path: &CurvePath2,
    interior_side: CurveBoundaryInteriorSide2,
    policy: &CurveContext,
) -> CurveRegion2 {
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        std::slice::from_ref(path),
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &[interior_side],
        policy,
    )
    .expect("test path must define an exact region")
    .into_value()
}

fn boolean_paths(
    first: &CurvePath2,
    second: &CurvePath2,
    operation: BooleanOp,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
    policy: &CurveContext,
) -> CurveRegion2 {
    path_region(first, first_interior_side, policy)
        .boolean_region(
            &path_region(second, second_interior_side, policy),
            operation,
            policy,
        )
        .expect("test region Boolean must complete exactly")
        .into_value()
}

fn assert_real_close(left: &Real, right: &Real, tolerance: f64) {
    let left = left.to_f64_lossy().expect("left Real is approximable");
    let right = right.to_f64_lossy().expect("right Real is approximable");
    assert!(
        (left - right).abs() <= tolerance,
        "expected {left} to be within {tolerance} of {right}"
    );
}

#[test]
fn top_level_rational_intersection_immediately_returns_sources_and_topology() {
    let first = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![Point2::new(r(0), r(0)), Point2::new(q(1, 2), r(0)), p(1, 1)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let second = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![Point2::new(r(0), q(1, 4)), Point2::new(r(1), q(1, 4))],
            vec![r(1), r(1)],
        )
        .unwrap(),
    ));

    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 1);
    assert!(evidence.is_complete());
    assert!(!evidence.is_disjoint());
    assert_eq!(evidence.contacts().len(), 1);
    assert!(evidence.blockers().is_empty());
    let contact = &evidence.contacts()[0];
    assert_eq!(
        decided(contact.first().parameter(&CurveContext::STRICT).unwrap())
            .scalar()
            .cloned(),
        Some(q(1, 2))
    );
    assert_eq!(
        decided(contact.second().parameter(&CurveContext::STRICT).unwrap())
            .scalar()
            .cloned(),
        Some(q(1, 2))
    );
    assert!(
        matches!((contact.point()).coordinates(), Some(point) if point == &Point2::new(q(1, 2), q(1, 4)))
    );
    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 2);
    assert_eq!(topology.arrangement_graph().len(), 4);
    assert_eq!(topology.arrangement_graph().len(), 4);
}

#[test]
fn top_level_retained_noninjective_overlap_keeps_isolated_branch_contacts() {
    let controls = vec![p(9, 0), p(-7, 3), p(-7, -10), p(9, 9)];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let curve = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 4]).unwrap();
        let middle = decided(
            curve
                .subcurve_between_exact(&q(1, 10), &q(9, 10), &policy)
                .unwrap(),
        );
        let result = Curve2::from(curve)
            .intersect_curve(&Curve2::from(middle), &policy)
            .unwrap()
            .into_value();
        assert!(result.is_complete(), "{:#?}", result.blockers());
        assert_eq!(result.contacts().len(), 2);
        assert_eq!(result.overlaps().len(), 1);
        assert!(
            result
                .contacts()
                .iter()
                .all(|contact| contact.is_certified_transverse())
        );
    }
}

#[test]
fn top_level_intersection_retains_implicit_conic_transversality() {
    let conic = Curve2::from(
        RationalBezier2::try_new(vec![p(1, 0), p(1, 1), p(0, 1)], vec![r(1), r(1), r(2)]).unwrap(),
    );
    let cubic_line = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::new(q(3, 5), r(-1)),
                Point2::new(q(3, 5), r(0)),
                Point2::new(q(3, 5), r(1)),
                Point2::new(q(3, 5), r(2)),
            ],
            vec![r(1); 4],
        )
        .unwrap(),
    );

    let result = conic
        .intersect_curve(&cubic_line, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(result.contacts().len(), 1);
    assert!(result.contacts()[0].is_certified_transverse());
}

#[test]
fn top_level_nurbs_intersection_deduplicates_a_shared_knot_contact() {
    let spline = Curve2::try_nurbs(
        1,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(1), r(2), r(2)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let line = Curve2::from(LineSeg2::try_new(p(0, 1), p(2, 1)).unwrap());

    let topology = spline
        .intersection_topology(&line, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 2);
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
    let contact = &evidence.contacts()[0];
    assert_eq!(
        decided(contact.first().parameter(&CurveContext::STRICT).unwrap())
            .scalar()
            .cloned(),
        Some(r(1))
    );
    assert_eq!(
        decided(contact.second().parameter(&CurveContext::STRICT).unwrap())
            .scalar()
            .cloned(),
        Some(q(1, 2))
    );
    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 2);
    assert_eq!(topology.arrangement_graph().len(), 4);
}

#[test]
fn selected_intersection_locations_reenter_evaluation_and_subdivision() {
    // x(t) = t^2 meets x = 1/2 at the positive root sqrt(1/2).
    // Spline charts map that same root into the authored interval [2, 5].
    let controls = vec![p(0, 0), p(0, 0), p(1, 0)];
    let root = q(1, 2).sqrt().unwrap();
    let point = Point2::new(q(1, 2), Real::zero());
    let crossing = Curve2::from(
        LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), Point2::new(q(1, 2), r(1))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let rational = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3]).unwrap();
        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let curves = [
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))),
            Curve2::from(rational.elevated_to_degree(5).unwrap()),
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .into_value(),
            Curve2::try_nurbs(2, controls.clone(), vec![Real::one(); 3], knots, &policy)
                .unwrap()
                .into_value(),
        ];
        for (index, original) in curves.into_iter().enumerate() {
            for reversed in [false, true] {
                let curve = if reversed {
                    original.reversed(&policy).unwrap().into_value()
                } else {
                    original.clone()
                };
                let local = if reversed {
                    Real::one() - &root
                } else {
                    root.clone()
                };
                let expected_parameter = if index < 2 {
                    local
                } else {
                    r(2) + r(3) * local
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&crossing, &curve)
                    } else {
                        (&curve, &crossing)
                    };
                    let outcome = first.intersect_curve(second, &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let result = outcome.value;
                    assert!(result.is_complete(), "{:?}", result.blockers());
                    assert!(result.overlaps().is_empty());
                    assert_eq!(result.contacts().len(), 1);
                    let first_path = CurvePath2::try_new(vec![first.clone()]).unwrap();
                    let second_path = CurvePath2::try_new(vec![second.clone()]).unwrap();
                    let path_outcome = first_path.intersect_path(&second_path, &policy).unwrap();
                    assert_eq!(path_outcome.certainty, CurveCertainty::Certified);
                    let path_result = path_outcome.value;
                    assert!(path_result.is_complete(), "{:?}", path_result.blockers());
                    assert!(path_result.overlaps().is_empty());
                    assert_eq!(path_result.contacts().len(), 1);
                    assert_eq!(path_result.contacts()[0].first_curve_index(), 0);
                    assert_eq!(path_result.contacts()[0].second_curve_index(), 0);
                    for contact in [&result.contacts()[0], path_result.contacts()[0].contact()] {
                        let location = if swapped {
                            contact.second()
                        } else {
                            contact.first()
                        };
                        let parameter = decided(location.parameter(&policy).unwrap());
                        let comparison = parameter
                            .compare(&expected_parameter.clone().into(), &CurveContext::STRICT)
                            .unwrap();
                        assert_eq!(comparison.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            comparison.value,
                            Classification::Decided(std::cmp::Ordering::Equal)
                        );
                        if index < 2 {
                            assert_eq!(&parameter, location.local_parameter());
                        }
                        let evaluated = curve.point_at(&parameter, &policy).unwrap();
                        assert_eq!(evaluated.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            evaluated
                                .value
                                .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                .value,
                            Classification::Decided(true),
                        );
                        let split = curve.split_at(parameter, &policy).unwrap();
                        assert_eq!(split.certainty, CurveCertainty::Certified);
                        for endpoint in [split.value.0.end(), split.value.1.start()] {
                            assert_eq!(
                                endpoint
                                    .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                    .value,
                                Classification::Decided(true),
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn retained_source_intersections_clip_contacts_and_reuse_selected_locations() {
    // Q(t) = (t, t^2). Keep the selected root of y = 1/2 as the
    // lower boundary, rather than rebuilding a control net at sqrt(1/2).
    let controls = vec![p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1)];
    let selecting = Curve2::from(
        LineSeg2::try_new(Point2::new(r(-1), q(1, 2)), Point2::new(r(2), q(1, 2))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let sources = [
            Curve2::from(QuadraticBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
            )),
            Curve2::from(
                RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3])
                    .unwrap()
                    .elevated_to_degree(5)
                    .unwrap(),
            ),
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .into_value(),
            Curve2::try_nurbs(2, controls.clone(), vec![Real::one(); 3], knots, &policy)
                .unwrap()
                .into_value(),
        ];
        for (index, source) in sources.into_iter().enumerate() {
            let selection = source.intersect_curve(&selecting, &policy).unwrap();
            assert_eq!(selection.certainty, CurveCertainty::Certified);
            assert!(selection.value.is_complete());
            assert_eq!(selection.value.contacts().len(), 1);
            let cut = decided(
                selection.value.contacts()[0]
                    .first()
                    .parameter(&policy)
                    .unwrap(),
            );
            assert!(cut.scalar().is_none());
            let tail = source
                .split_at(cut.clone(), &policy)
                .unwrap()
                .into_value()
                .1;
            for reversed in [false, true] {
                let curve = if reversed {
                    tail.reversed(&policy).unwrap().into_value()
                } else {
                    tail.clone()
                };
                for (other, expected) in [
                    (
                        Curve2::from(
                            LineSeg2::try_new(
                                Point2::new(q(3, 4), r(-1)),
                                Point2::new(q(3, 4), r(2)),
                            )
                            .unwrap(),
                        ),
                        Some((
                            Point2::new(q(3, 4), q(9, 16)),
                            if index < 2 {
                                q(3, 4).into()
                            } else {
                                q(17, 4).into()
                            },
                            true,
                        )),
                    ),
                    (
                        Curve2::from(
                            LineSeg2::try_new(
                                Point2::new(q(1, 2), r(-1)),
                                Point2::new(q(1, 2), r(2)),
                            )
                            .unwrap(),
                        ),
                        None,
                    ),
                    (
                        selecting.clone(),
                        Some((
                            Point2::new(q(1, 2).sqrt().unwrap(), q(1, 2)),
                            cut.clone(),
                            false,
                        )),
                    ),
                ] {
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&other, &curve)
                        } else {
                            (&curve, &other)
                        };
                        let outcome = first.intersect_curve(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let result = outcome.value;
                        assert!(
                            result.is_complete(),
                            "{index} reversed={reversed} swapped={swapped}: {:?}",
                            result.blockers()
                        );
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), usize::from(expected.is_some()));
                        let paths = CurvePath2::try_new(vec![first.clone()])
                            .unwrap()
                            .intersect_path(
                                &CurvePath2::try_new(vec![second.clone()]).unwrap(),
                                &policy,
                            )
                            .unwrap();
                        assert_eq!(paths.certainty, CurveCertainty::Certified);
                        assert!(paths.value.is_complete(), "{:?}", paths.value.blockers());
                        assert_eq!(paths.value.contacts().len(), result.contacts().len());
                        assert!(paths.value.overlaps().is_empty());
                        let Some((point, expected_parameter, interior)) = &expected else {
                            continue;
                        };
                        for contact in [
                            result.contacts().first().unwrap(),
                            paths.value.contacts()[0].contact(),
                        ] {
                            assert!(decided(
                                contact
                                    .point()
                                    .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                    .value
                            ));
                            let location = if swapped {
                                contact.second()
                            } else {
                                contact.first()
                            };
                            let parameter = decided(location.parameter(&policy).unwrap());
                            let compared = parameter
                                .compare(expected_parameter, &CurveContext::STRICT)
                                .unwrap();
                            assert_eq!(compared.certainty, CurveCertainty::Certified);
                            assert_eq!(
                                compared.value,
                                Classification::Decided(std::cmp::Ordering::Equal)
                            );
                            let evaluated = curve.point_at(&parameter, &policy).unwrap();
                            assert_eq!(evaluated.certainty, CurveCertainty::Certified);
                            assert!(decided(
                                evaluated
                                    .value
                                    .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                    .value
                            ));
                            if *interior {
                                let split = curve.split_at(parameter, &policy).unwrap();
                                assert_eq!(split.certainty, CurveCertainty::Certified);
                                for endpoint in [split.value.0.end(), split.value.1.start()] {
                                    assert!(decided(
                                        endpoint
                                            .coincides_with(
                                                &point.clone().into(),
                                                &CurveContext::STRICT
                                            )
                                            .value
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
fn retained_source_overlaps_preserve_independent_ranges_and_singleton_contacts() {
    let root = q(1, 2).sqrt().unwrap();
    let source = Curve2::from(QuadraticBezier2::new(
        p(0, 0),
        Point2::new(q(1, 2), r(0)),
        p(1, 1),
    ));
    let selecting = Curve2::from(
        LineSeg2::try_new(Point2::new(r(-1), q(1, 2)), Point2::new(r(2), q(1, 2))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selection = source
            .intersect_curve(&selecting, &policy)
            .unwrap()
            .into_value();
        let cut = decided(selection.contacts()[0].first().parameter(&policy).unwrap());
        let (prefix, tail) = source.split_at(cut, &policy).unwrap().into_value();
        for reversed in [false, true] {
            let tail = if reversed {
                tail.reversed(&policy).unwrap().into_value()
            } else {
                tail.clone()
            };
            for (other, expected_endpoints) in [
                (
                    Curve2::from(QuadraticBezier2::new(
                        Point2::new(root.clone(), q(1, 2)),
                        Point2::new((Real::one() + &root) * q(1, 2), root.clone()),
                        p(1, 1),
                    )),
                    Some([Point2::new(root.clone(), q(1, 2)), p(1, 1)]),
                ),
                (
                    Curve2::from(QuadraticBezier2::new(
                        Point2::new(q(3, 4), q(9, 16)),
                        Point2::new(q(7, 8), q(3, 4)),
                        p(1, 1),
                    )),
                    Some([Point2::new(q(3, 4), q(9, 16)), p(1, 1)]),
                ),
                (prefix.clone(), None),
            ] {
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&other, &tail)
                    } else {
                        (&tail, &other)
                    };
                    let outcome = first.intersect_curve(second, &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let result = outcome.value;
                    assert!(
                        result.is_complete(),
                        "reversed={reversed} swapped={swapped}: {:?}",
                        result.blockers()
                    );
                    if let Some(expected) = &expected_endpoints {
                        assert_eq!(result.overlaps().len(), 1);
                        assert!(result.contacts().is_empty());
                        let overlap = &result.overlaps()[0];
                        assert_eq!(
                            overlap.orientation(),
                            if reversed {
                                RationalBezierOverlapOrientation2::Reversed
                            } else {
                                RationalBezierOverlapOrientation2::Same
                            }
                        );
                        assert!(overlap.includes_start());
                        assert!(overlap.includes_end());
                        for (curve, range) in [
                            (first, overlap.first_range()),
                            (second, overlap.second_range()),
                        ] {
                            let points = [
                                curve.point_at(range.start(), &policy).unwrap().into_value(),
                                curve.point_at(range.end(), &policy).unwrap().into_value(),
                            ];
                            for point in &points {
                                assert!(expected.iter().any(|expected| {
                                    decided(
                                        point
                                            .coincides_with(
                                                &expected.clone().into(),
                                                &CurveContext::STRICT,
                                            )
                                            .value,
                                    )
                                }));
                            }
                            assert!(!decided(
                                points[0]
                                    .coincides_with(&points[1], &CurveContext::STRICT)
                                    .value
                            ));
                        }
                    } else {
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), 1);
                        assert!(!result.contacts()[0].is_certified_transverse());
                        assert!(decided(
                            result.contacts()[0]
                                .point()
                                .coincides_with(
                                    &Point2::new(root.clone(), q(1, 2)).into(),
                                    &CurveContext::STRICT,
                                )
                                .value
                        ));
                    }
                }
            }
        }
    }
}

#[test]
fn generated_chamfer_tails_reuse_paired_overlap_boundaries() {
    use hypercurve::{CurveCornerMode2, CurveCornerSolutions2};
    // The setback selects s^4 + 4s^2 = 1 on Q(t) = (t^2, 2t).
    // The independent tail has exact radical controls, not shared lineage.
    let squared = r(5).sqrt().unwrap() - r(2);
    let root = squared.clone().sqrt().unwrap();
    let independent = Curve2::from(QuadraticBezier2::new(
        Point2::new(squared, r(2) * &root),
        Point2::new(root.clone(), Real::one() + &root),
        p(1, 2),
    ));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        ])
        .unwrap();
        let chamfer = path
            .chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap();
        assert_eq!(chamfer.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(chamfer) = chamfer.value else {
            panic!("one unit setback on each incident curve");
        };
        let tail = chamfer.curves().last().unwrap();
        assert!(tail.geometry().is_none());
        for reversed in [false, true] {
            let tail = if reversed {
                tail.reversed(&policy).unwrap().into_value()
            } else {
                tail.clone()
            };
            for swapped in [false, true] {
                let (first, second) = if swapped {
                    (&independent, &tail)
                } else {
                    (&tail, &independent)
                };
                let outcome = first.intersect_curve(second, &policy).unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                assert!(
                    outcome.value.is_complete(),
                    "{:?}",
                    outcome.value.blockers()
                );
                assert!(outcome.value.contacts().is_empty());
                assert_eq!(outcome.value.overlaps().len(), 1);
                let overlap = &outcome.value.overlaps()[0];
                assert_eq!(
                    overlap.orientation(),
                    if reversed {
                        RationalBezierOverlapOrientation2::Reversed
                    } else {
                        RationalBezierOverlapOrientation2::Same
                    }
                );
                assert!(overlap.includes_start() && overlap.includes_end());
                for (curve, range) in [
                    (first, overlap.first_range()),
                    (second, overlap.second_range()),
                ] {
                    for parameter in [range.start(), range.end()] {
                        let point = curve.point_at(parameter, &policy).unwrap();
                        assert_eq!(point.certainty, CurveCertainty::Certified);
                        assert!(
                            [independent.start(), independent.end()]
                                .iter()
                                .any(|endpoint| {
                                    decided(
                                        point
                                            .value
                                            .coincides_with(endpoint, &CurveContext::STRICT)
                                            .value,
                                    )
                                })
                        );
                    }
                }
            }
        }
    }
}

fn generated_parabola_chord(policy: &CurveContext) -> Curve2 {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
    ])
    .unwrap();
    let outcome = path
        .chamfer_vertex_by_setbacks(
            1,
            Real::one(),
            Real::one(),
            hypercurve::CurveCornerMode2::TrimOnly,
            policy,
        )
        .unwrap();
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    let hypercurve::CurveCornerSolutions2::Unique(path) = outcome.value else {
        panic!("one chamfer");
    };
    let chord = path.curves()[1].clone();
    assert!(chord.geometry().is_none());
    assert_eq!(chord.family(), CurveFamily2::Line);
    chord
}

fn assert_single_contact_curve_pieces(
    source: &Curve2,
    pieces: &[Curve2],
    contact: &Point2,
    policy: &CurveContext,
) {
    assert_eq!(pieces.len(), 2);
    for (actual, expected) in [
        (pieces[0].start(), source.start()),
        (pieces[1].end(), source.end()),
        (pieces[0].end(), contact.clone().into()),
        (pieces[1].start(), contact.clone().into()),
    ] {
        assert!(decided(
            actual
                .coincides_with(&expected, &CurveContext::STRICT)
                .value
        ));
    }
    for piece in pieces {
        let domain = piece.parameter_domain();
        for parameter in [domain.start(), domain.end()] {
            let point = piece.point_at(parameter, policy).unwrap();
            assert_eq!(point.certainty, CurveCertainty::Certified);
            assert!([piece.start(), piece.end()].iter().any(|endpoint| {
                decided(
                    point
                        .value
                        .coincides_with(endpoint, &CurveContext::STRICT)
                        .value,
                )
            }));
        }
    }
    CurvePath2::try_new(pieces.to_vec())
        .expect("the returned pieces preserve the continuous source");
}

#[test]
fn generated_chord_topology_publishes_reusable_curve_pieces() {
    let squared = r(5).sqrt().unwrap() - r(2);
    let root = squared.clone().sqrt().unwrap();
    let point = Point2::new(-q(1, 2), (&root / (Real::one() + squared)).unwrap());
    let crossing = Curve2::from(
        LineSeg2::try_new(Point2::new(-q(1, 2), r(-1)), Point2::new(-q(1, 2), r(2))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let original = generated_parabola_chord(&policy);
        for reverse_first in [false, true] {
            let chord = if reverse_first {
                original.reversed(&policy).unwrap().into_value()
            } else {
                original.clone()
            };
            for reverse_second in [false, true] {
                let crossing = if reverse_second {
                    crossing.reversed(&policy).unwrap().into_value()
                } else {
                    crossing.clone()
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&crossing, &chord)
                    } else {
                        (&chord, &crossing)
                    };
                    let outcome = first.intersection_topology(second, &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let topology = outcome.value;
                    assert_eq!(topology.result().contacts().len(), 1);
                    assert!(topology.result().is_complete());
                    assert_eq!(topology.arrangement_graph().len(), 4);
                    assert!(std::ptr::eq(
                        topology.arrangement_graph(),
                        topology.clone().arrangement_graph()
                    ));
                    for (source, pieces, other) in [
                        (first, topology.first(), second),
                        (second, topology.second(), first),
                    ] {
                        assert_single_contact_curve_pieces(source, pieces, &point, &policy);
                        for piece in pieces {
                            let replay = piece.intersect_curve(other, &policy).unwrap();
                            assert_eq!(replay.certainty, CurveCertainty::Certified);
                            assert!(replay.value.is_complete(), "{:?}", replay.value.blockers());
                            assert_eq!(replay.value.contacts().len(), 1);
                            assert!(replay.value.overlaps().is_empty());
                            assert!(decided(
                                replay.value.contacts()[0]
                                    .point()
                                    .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                    .value
                            ));
                        }
                    }
                    let paths = [
                        CurvePath2::try_new(vec![first.clone()]).unwrap(),
                        CurvePath2::try_new(vec![second.clone()]).unwrap(),
                    ];
                    let outcome = paths[0].intersection_topology(&paths[1], &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    assert_eq!(outcome.value.arrangement_graph().len(), 4);
                    assert_single_contact_curve_pieces(
                        first,
                        outcome.value.first()[0].curves(),
                        &point,
                        &policy,
                    );
                    assert_single_contact_curve_pieces(
                        second,
                        outcome.value.second()[0].curves(),
                        &point,
                        &policy,
                    );
                }
            }
        }
    }
}

#[test]
fn selected_tail_topology_keeps_reversed_and_nonunit_source_charts() {
    let controls = vec![p(0, 0), p(0, 0), p(1, 0)];
    let vertical = |x: Real| {
        Curve2::from(
            LineSeg2::try_new(Point2::new(x.clone(), r(-1)), Point2::new(x, r(1))).unwrap(),
        )
    };
    let selecting = vertical(q(1, 2));
    let crossing = vertical(q(3, 4));
    let point = Point2::new(q(3, 4), r(0));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let curves = [
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))),
            Curve2::from(
                RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3])
                    .unwrap()
                    .elevated_to_degree(5)
                    .unwrap(),
            ),
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .into_value(),
            Curve2::try_nurbs(2, controls.clone(), vec![Real::one(); 3], knots, &policy)
                .unwrap()
                .into_value(),
        ];
        for original in curves {
            let selected = original.intersect_curve(&selecting, &policy).unwrap();
            assert_eq!(selected.certainty, CurveCertainty::Certified);
            let parameter = decided(
                selected.value.contacts()[0]
                    .first()
                    .parameter(&policy)
                    .unwrap(),
            );
            let tail = original
                .split_at(parameter, &policy)
                .unwrap()
                .into_value()
                .1;
            for reversed in [false, true] {
                let tail = if reversed {
                    tail.reversed(&policy).unwrap().into_value()
                } else {
                    tail.clone()
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&crossing, &tail)
                    } else {
                        (&tail, &crossing)
                    };
                    let outcome = first.intersection_topology(second, &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let topology = outcome.value;
                    assert_eq!(topology.result().contacts().len(), 1);
                    assert_single_contact_curve_pieces(first, topology.first(), &point, &policy);
                    assert_single_contact_curve_pieces(second, topology.second(), &point, &policy);
                    let paths = [
                        CurvePath2::try_new(vec![first.clone()]).unwrap(),
                        CurvePath2::try_new(vec![second.clone()]).unwrap(),
                    ];
                    let outcome = paths[0].intersection_topology(&paths[1], &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    assert_single_contact_curve_pieces(
                        first,
                        outcome.value.first()[0].curves(),
                        &point,
                        &policy,
                    );
                    assert_single_contact_curve_pieces(
                        second,
                        outcome.value.second()[0].curves(),
                        &point,
                        &policy,
                    );
                }
            }
        }
    }
}

#[test]
fn split_topology_preserves_both_sides_of_a_discontinuous_spline_knot() {
    use hypercurve::{NurbsCurve2, PolynomialSplineCurve2};

    let crossing = Curve2::from(LineSeg2::try_new(p(-1, 0), p(13, 0)).unwrap());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let controls = vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)];
        let knots = [-2, -1, 0, 1, 1, 1, 2, 3, 4]
            .into_iter()
            .map(r)
            .collect::<Vec<_>>();
        let polynomial =
            PolynomialSplineCurve2::try_new(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .value;
        let rational = NurbsCurve2::try_new(
            2,
            controls,
            vec![r(1), r(2), r(3), r(5), r(7), r(11)],
            knots,
            &policy,
        )
        .unwrap()
        .value;
        for curve in [Curve2::from(polynomial), Curve2::from(rational)] {
            for reversed in [false, true] {
                let source = if reversed {
                    curve.reversed(&policy).unwrap().value
                } else {
                    curve.clone()
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&crossing, &source)
                    } else {
                        (&source, &crossing)
                    };
                    let outcome = first.intersection_topology(second, &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let topology = outcome.value;
                    assert!(topology.result().is_complete());
                    assert_eq!(topology.result().contacts().len(), 2);
                    assert!(topology.result().overlaps().is_empty());
                    let (pieces, line_pieces) = if swapped {
                        (topology.second(), topology.first())
                    } else {
                        (topology.first(), topology.second())
                    };
                    assert_eq!(pieces.len(), 2);
                    assert_eq!(line_pieces.len(), 3);
                    let sides = if reversed {
                        [p(10, 0), p(2, 0)]
                    } else {
                        [p(2, 0), p(10, 0)]
                    };
                    for (actual, expected) in [
                        (pieces[0].start(), source.start()),
                        (pieces[0].end(), sides[0].clone().into()),
                        (pieces[1].start(), sides[1].clone().into()),
                        (pieces[1].end(), source.end()),
                        (line_pieces[0].end(), p(2, 0).into()),
                        (line_pieces[1].start(), p(2, 0).into()),
                        (line_pieces[1].end(), p(10, 0).into()),
                        (line_pieces[2].start(), p(10, 0).into()),
                    ] {
                        assert!(decided(
                            actual
                                .coincides_with(&expected, &CurveContext::STRICT)
                                .value
                        ));
                    }
                    for (piece, expected) in pieces.iter().zip(&sides) {
                        let replay = piece.intersect_curve(&crossing, &policy).unwrap();
                        assert_eq!(replay.certainty, CurveCertainty::Certified);
                        assert!(replay.value.is_complete());
                        assert_eq!(replay.value.contacts().len(), 1);
                        assert!(decided(
                            replay.value.contacts()[0]
                                .point()
                                .coincides_with(&expected.clone().into(), &CurveContext::STRICT)
                                .value
                        ));
                    }
                    let graph = topology.arrangement_graph();
                    assert_eq!(graph.len(), 5);
                    for source_index in 0..2 {
                        let count = if (source_index == 0) == swapped { 3 } else { 2 };
                        let indices = graph
                            .fragments()
                            .iter()
                            .filter(|fragment| fragment.source_curve_index() == source_index)
                            .map(|fragment| fragment.source_fragment_index())
                            .collect::<Vec<_>>();
                        assert_eq!(indices, (0..count).collect::<Vec<_>>());
                    }
                }
            }
        }
    }
}

#[test]
fn generated_chords_keep_open_contacts_and_general_locations() {
    let squared = r(5).sqrt().unwrap() - r(2);
    let root = squared.clone().sqrt().unwrap();
    let middle = Point2::new(-q(1, 2), (&root / (Real::one() + &squared)).unwrap());
    let end = Point2::new(squared, r(2) * root);
    let line = |x: Real| {
        Curve2::from(
            LineSeg2::try_new(Point2::new(x.clone(), r(-1)), Point2::new(x, r(2))).unwrap(),
        )
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let original = generated_parabola_chord(&policy);
        for reverse_chord in [false, true] {
            let chord = if reverse_chord {
                original.reversed(&policy).unwrap().into_value()
            } else {
                original.clone()
            };
            for (other, expected, interior) in [
                (line(-q(1, 2)), Some(middle.clone()), true),
                (line(r(-1)), Some(p(-1, 0)), false),
                (
                    Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
                    Some(end.clone()),
                    false,
                ),
                (line(r(1)), None, false),
            ] {
                for reverse_other in [false, true] {
                    let other = if reverse_other {
                        other.reversed(&policy).unwrap().into_value()
                    } else {
                        other.clone()
                    };
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&other, &chord)
                        } else {
                            (&chord, &other)
                        };
                        let outcome = first.intersect_curve(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let result = outcome.value;
                        assert!(
                            result.is_complete(),
                            "reversed={reverse_chord}/{reverse_other} swapped={swapped}: {:?}",
                            result.blockers()
                        );
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), usize::from(expected.is_some()));
                        let paths = CurvePath2::try_new(vec![first.clone()])
                            .unwrap()
                            .intersect_path(
                                &CurvePath2::try_new(vec![second.clone()]).unwrap(),
                                &policy,
                            )
                            .unwrap();
                        assert_eq!(paths.certainty, CurveCertainty::Certified);
                        assert!(paths.value.is_complete(), "{:?}", paths.value.blockers());
                        assert_eq!(paths.value.contacts().len(), result.contacts().len());
                        let Some(point) = &expected else {
                            continue;
                        };
                        for contact in [
                            result.contacts().first().unwrap(),
                            paths.value.contacts()[0].contact(),
                        ] {
                            assert!(decided(
                                contact
                                    .point()
                                    .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                    .value
                            ));
                            for (curve, location) in
                                [(first, contact.first()), (second, contact.second())]
                            {
                                let parameter = decided(location.parameter(&policy).unwrap());
                                let evaluated = curve.point_at(&parameter, &policy).unwrap();
                                assert_eq!(evaluated.certainty, CurveCertainty::Certified);
                                assert!(decided(
                                    evaluated
                                        .value
                                        .coincides_with(
                                            &point.clone().into(),
                                            &CurveContext::STRICT
                                        )
                                        .value
                                ));
                                if interior {
                                    let split = curve.split_at(parameter, &policy).unwrap();
                                    assert_eq!(split.certainty, CurveCertainty::Certified);
                                    for endpoint in [split.value.0.end(), split.value.1.start()] {
                                        assert!(decided(
                                            endpoint
                                                .coincides_with(
                                                    &point.clone().into(),
                                                    &CurveContext::STRICT
                                                )
                                                .value
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
}

#[test]
fn generated_chord_overlaps_retain_independent_and_selected_boundaries() {
    let squared = r(5).sqrt().unwrap() - r(2);
    let root = squared.clone().sqrt().unwrap();
    let end = Point2::new(squared.clone(), r(2) * &root);
    let selecting = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0)));
    let selecting_line = Curve2::from(
        LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), Point2::new(q(1, 2), r(1))).unwrap(),
    );
    let alpha = q(1, 2).sqrt().unwrap();
    let selected_point = Point2::new(
        -Real::one() + &alpha * (Real::one() + squared),
        r(2) * &alpha * root,
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let original = generated_parabola_chord(&policy);
        let independent = Curve2::from(LineSeg2::try_new(p(-1, 0), end.clone()).unwrap());
        let selected = selecting
            .intersect_curve(&selecting_line, &policy)
            .unwrap()
            .into_value();
        let cut = decided(selected.contacts()[0].first().parameter(&policy).unwrap());
        let tail = independent.split_at(cut, &policy).unwrap().into_value().1;
        for (case, (other, expected)) in [
            (independent, [p(-1, 0), end.clone()]),
            (generated_parabola_chord(&policy), [p(-1, 0), end.clone()]),
            (tail, [selected_point.clone(), end.clone()]),
        ]
        .into_iter()
        .enumerate()
        {
            for reverse_chord in [false, true] {
                let chord = if reverse_chord {
                    original.reversed(&policy).unwrap().into_value()
                } else {
                    original.clone()
                };
                for reverse_other in [false, true] {
                    let other = if reverse_other {
                        other.reversed(&policy).unwrap().into_value()
                    } else {
                        other.clone()
                    };
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&other, &chord)
                        } else {
                            (&chord, &other)
                        };
                        let outcome = first.intersect_curve(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        assert!(
                            outcome.value.is_complete(),
                            "case={case} reverse_chord={reverse_chord} reverse_other={reverse_other} swapped={swapped}: {:?}",
                            outcome.value.blockers()
                        );
                        assert!(outcome.value.contacts().is_empty());
                        assert_eq!(outcome.value.overlaps().len(), 1);
                        let overlap = &outcome.value.overlaps()[0];
                        assert_eq!(
                            overlap.orientation(),
                            if reverse_chord != reverse_other {
                                RationalBezierOverlapOrientation2::Reversed
                            } else {
                                RationalBezierOverlapOrientation2::Same
                            }
                        );
                        assert!(overlap.includes_start() && overlap.includes_end());
                        for (first_parameter, second_parameter) in [
                            (
                                overlap.first_range().start(),
                                overlap.second_range().start(),
                            ),
                            (overlap.first_range().end(), overlap.second_range().end()),
                        ] {
                            let first_point = first.point_at(first_parameter, &policy).unwrap();
                            let second_point = second.point_at(second_parameter, &policy).unwrap();
                            assert_eq!(first_point.certainty, CurveCertainty::Certified);
                            assert_eq!(second_point.certainty, CurveCertainty::Certified);
                            assert!(decided(
                                first_point
                                    .value
                                    .coincides_with(&second_point.value, &CurveContext::STRICT)
                                    .value
                            ));
                        }
                        for (curve, range) in [
                            (first, overlap.first_range()),
                            (second, overlap.second_range()),
                        ] {
                            let points = [
                                curve.point_at(range.start(), &policy).unwrap().into_value(),
                                curve.point_at(range.end(), &policy).unwrap().into_value(),
                            ];
                            for (boundary, point) in points.iter().enumerate() {
                                assert!(
                                    expected.iter().any(|expected| {
                                        decided(
                                            point
                                                .coincides_with(
                                                    &expected.clone().into(),
                                                    &CurveContext::STRICT,
                                                )
                                                .value,
                                        )
                                    }),
                                    "case={case} reverse_chord={reverse_chord} reverse_other={reverse_other} swapped={swapped} boundary={boundary} point bounds={:?}",
                                    point.bounds(&policy).value.map(|b| [
                                        b.min_x().to_f64_lossy(),
                                        b.max_x().to_f64_lossy(),
                                        b.min_y().to_f64_lossy(),
                                        b.max_y().to_f64_lossy()
                                    ])
                                );
                            }
                            assert!(!decided(
                                points[0]
                                    .coincides_with(&points[1], &CurveContext::STRICT)
                                    .value
                            ));
                        }
                        let assert_pieces = |first_pieces: &[Curve2], second_pieces: &[Curve2]| {
                            for (source, pieces, has_cut) in [
                                (first, first_pieces, case == 2 && !swapped),
                                (second, second_pieces, case == 2 && swapped),
                            ] {
                                if has_cut {
                                    assert_single_contact_curve_pieces(
                                        source,
                                        pieces,
                                        &selected_point,
                                        &policy,
                                    );
                                } else {
                                    assert_eq!(pieces.len(), 1);
                                    for (actual, expected) in [
                                        (pieces[0].start(), source.start()),
                                        (pieces[0].end(), source.end()),
                                    ] {
                                        assert!(decided(
                                            actual
                                                .coincides_with(&expected, &CurveContext::STRICT)
                                                .value
                                        ));
                                    }
                                }
                            }
                        };
                        let outcome = first.intersection_topology(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        assert_pieces(outcome.value.first(), outcome.value.second());
                        assert_eq!(
                            outcome.value.arrangement_graph().len(),
                            if case == 2 { 3 } else { 2 }
                        );
                        let paths = [
                            CurvePath2::try_new(vec![first.clone()]).unwrap(),
                            CurvePath2::try_new(vec![second.clone()]).unwrap(),
                        ];
                        let outcome = paths[0].intersection_topology(&paths[1], &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        assert_pieces(
                            outcome.value.first()[0].curves(),
                            outcome.value.second()[0].curves(),
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn generated_chord_cuts_reenter_collinear_endpoint_intersections() {
    let squared = r(5).sqrt().unwrap() - r(2);
    let root = squared.clone().sqrt().unwrap();
    let end = Point2::new(squared.clone(), r(2) * &root);
    let expected = Point2::new(-q(1, 2), (&root / (Real::one() + squared)).unwrap());
    let selecting = Curve2::from(
        LineSeg2::try_new(Point2::new(-q(1, 2), r(-1)), Point2::new(-q(1, 2), r(2))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let original = generated_parabola_chord(&policy);
        let independent = Curve2::from(LineSeg2::try_new(p(-1, 0), end.clone()).unwrap());
        let selected = original.intersect_curve(&selecting, &policy).unwrap();
        assert_eq!(selected.certainty, CurveCertainty::Certified);
        assert!(selected.value.is_complete());
        assert_eq!(selected.value.contacts().len(), 1);
        let cut = decided(
            selected.value.contacts()[0]
                .first()
                .parameter(&policy)
                .unwrap(),
        );
        let split = original.split_at(cut, &policy).unwrap();
        assert_eq!(split.certainty, CurveCertainty::Certified);
        let (left, right) = split.value;

        let overlap = left.intersect_curve(&independent, &policy).unwrap();
        assert_eq!(overlap.certainty, CurveCertainty::Certified);
        assert!(
            overlap.value.is_complete(),
            "{:?}",
            overlap.value.blockers()
        );
        assert!(overlap.value.contacts().is_empty());
        assert_eq!(overlap.value.overlaps().len(), 1);
        let cut = overlap.value.overlaps()[0].second_range().end().clone();
        let split = independent.split_at(cut, &policy).unwrap();
        assert_eq!(split.certainty, CurveCertainty::Certified);
        let (prefix, tail) = split.value;

        for (case, (first, second)) in [(&left, &right), (&left, &tail), (&right, &prefix)]
            .into_iter()
            .enumerate()
        {
            for reverse_first in [false, true] {
                let first = if reverse_first {
                    first.reversed(&policy).unwrap().into_value()
                } else {
                    first.clone()
                };
                for reverse_second in [false, true] {
                    let second = if reverse_second {
                        second.reversed(&policy).unwrap().into_value()
                    } else {
                        second.clone()
                    };
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&second, &first)
                        } else {
                            (&first, &second)
                        };
                        let outcome = first.intersect_curve(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let result = outcome.value;
                        assert!(
                            result.is_complete(),
                            "case={case} reverse_first={reverse_first} reverse_second={reverse_second} swapped={swapped}: {:?}",
                            result.blockers()
                        );
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), 1);
                        let contact = &result.contacts()[0];
                        assert!(!contact.is_certified_transverse());
                        assert_eq!(
                            contact.tangent_cross_sign(),
                            Some(hyperreal::RealSign::Zero)
                        );
                        let topology = first.intersection_topology(second, &policy).unwrap();
                        assert_eq!(topology.certainty, CurveCertainty::Certified);
                        assert_eq!(topology.value.arrangement_graph().len(), 2);
                        for (source, pieces) in [
                            (first, topology.value.first()),
                            (second, topology.value.second()),
                        ] {
                            assert_eq!(
                                pieces.len(),
                                1,
                                "endpoint-only contacts do not split a curve"
                            );
                            for (actual, expected) in [
                                (pieces[0].start(), source.start()),
                                (pieces[0].end(), source.end()),
                            ] {
                                assert!(decided(
                                    actual
                                        .coincides_with(&expected, &CurveContext::STRICT)
                                        .value
                                ));
                            }
                        }
                        assert!(
                            decided(
                                contact
                                    .point()
                                    .coincides_with(&expected.clone().into(), &CurveContext::STRICT)
                                    .value
                            ),
                            "case={case} reverse_first={reverse_first} reverse_second={reverse_second} swapped={swapped} point bounds={:?}",
                            contact.point().bounds(&policy).value.map(|b| [
                                b.min_x().to_f64_lossy(),
                                b.max_x().to_f64_lossy(),
                                b.min_y().to_f64_lossy(),
                                b.max_y().to_f64_lossy()
                            ])
                        );
                        for (curve, location) in
                            [(first, contact.first()), (second, contact.second())]
                        {
                            let parameter = decided(location.parameter(&policy).unwrap());
                            let point = curve.point_at(&parameter, &policy).unwrap();
                            assert_eq!(point.certainty, CurveCertainty::Certified);
                            assert!(decided(
                                point
                                    .value
                                    .coincides_with(&expected.clone().into(), &CurveContext::STRICT)
                                    .value
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn reversed_retained_spline_charts_deduplicate_seams_and_map_interior_contacts() {
    let selecting = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0)));
    let crossing = Curve2::from(
        LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), Point2::new(q(1, 2), r(1))).unwrap(),
    );
    let controls = vec![p(0, 0), p(1, 1), p(2, 0), p(3, 1)];
    let knots = vec![r(0), r(0), r(1), r(2), r(3), r(3)];
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selection = selecting
            .intersect_curve(&crossing, &policy)
            .unwrap()
            .into_value();
        let cut = decided(selection.contacts()[0].first().parameter(&policy).unwrap());
        for source in [
            Curve2::try_polynomial_bspline(1, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .into_value(),
            Curve2::try_nurbs(
                1,
                controls.clone(),
                vec![Real::one(); 4],
                knots.clone(),
                &policy,
            )
            .unwrap()
            .into_value(),
        ] {
            let tail = source
                .split_at(cut.clone(), &policy)
                .unwrap()
                .into_value()
                .1;
            for reversed in [false, true] {
                let curve = if reversed {
                    tail.reversed(&policy).unwrap().into_value()
                } else {
                    tail.clone()
                };
                for (height, expected) in [(r(0), vec![r(2)]), (q(1, 2), vec![q(3, 2), q(5, 2)])] {
                    let crossing = Curve2::from(
                        LineSeg2::try_new(
                            Point2::new(r(-1), height.clone()),
                            Point2::new(r(4), height.clone()),
                        )
                        .unwrap(),
                    );
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&crossing, &curve)
                        } else {
                            (&curve, &crossing)
                        };
                        let outcome = first.intersect_curve(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        assert!(
                            outcome.value.is_complete(),
                            "{:?}",
                            outcome.value.blockers()
                        );
                        assert!(outcome.value.overlaps().is_empty());
                        assert_eq!(outcome.value.contacts().len(), expected.len());
                        for parameter in &expected {
                            let point = Point2::new(parameter.clone(), height.clone());
                            let contact = outcome
                                .value
                                .contacts()
                                .iter()
                                .find(|contact| {
                                    decided(
                                        contact
                                            .point()
                                            .coincides_with(
                                                &point.clone().into(),
                                                &CurveContext::STRICT,
                                            )
                                            .value,
                                    )
                                })
                                .expect("every independent intersection is retained once");
                            let location = if swapped {
                                contact.second()
                            } else {
                                contact.first()
                            };
                            let mapped = decided(location.parameter(&policy).unwrap());
                            assert_eq!(
                                mapped
                                    .compare(&parameter.clone().into(), &CurveContext::STRICT)
                                    .unwrap()
                                    .value,
                                Classification::Decided(std::cmp::Ordering::Equal)
                            );
                            let evaluated = curve.point_at(&mapped, &policy).unwrap();
                            assert_eq!(evaluated.certainty, CurveCertainty::Certified);
                            assert!(decided(
                                evaluated
                                    .value
                                    .coincides_with(&point.into(), &CurveContext::STRICT)
                                    .value
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn retained_noninjective_domains_keep_off_diagonal_contacts_and_traversal_signs() {
    let controls = vec![p(9, 0), p(-7, 3), p(-7, -10), p(9, 9)];
    // This cubic visits (0, 0) at 1/4 and 3/4. A selected cut at sqrt(1/2)
    // separates those two parameters, while its own boundary is also shared.
    let selecting = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0)));
    let crossing = Curve2::from(
        LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), Point2::new(q(1, 2), r(1))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selected = selecting
            .intersect_curve(&crossing, &policy)
            .unwrap()
            .into_value();
        let cut = decided(selected.contacts()[0].first().parameter(&policy).unwrap());
        let source =
            Curve2::from(RationalBezier2::try_new(controls.clone(), vec![Real::one(); 4]).unwrap());
        for independent in [false, true] {
            let other = if independent {
                Curve2::from(
                    RationalBezier2::try_new(controls.clone(), vec![Real::one(); 4]).unwrap(),
                )
            } else {
                source.clone()
            };
            let prefix = source
                .subcurve(r(0).into(), cut.clone(), &policy)
                .unwrap()
                .into_value();
            let tail = other
                .subcurve(cut.clone(), r(1).into(), &policy)
                .unwrap()
                .into_value();
            for reverse_first in [false, true] {
                let prefix = if reverse_first {
                    prefix.reversed(&policy).unwrap().into_value()
                } else {
                    prefix.clone()
                };
                for reverse_second in [false, true] {
                    let tail = if reverse_second {
                        tail.reversed(&policy).unwrap().into_value()
                    } else {
                        tail.clone()
                    };
                    for swapped in [false, true] {
                        let (first, second) = if swapped {
                            (&tail, &prefix)
                        } else {
                            (&prefix, &tail)
                        };
                        let outcome = first.intersect_curve(second, &policy).unwrap();
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let result = outcome.value;
                        assert!(result.is_complete(), "{:?}", result.blockers());
                        assert!(result.overlaps().is_empty());
                        assert_eq!(result.contacts().len(), 2);
                        let origin = result
                            .contacts()
                            .iter()
                            .find(|contact| {
                                decided(
                                    contact
                                        .point()
                                        .coincides_with(&p(0, 0).into(), &CurveContext::STRICT)
                                        .value,
                                )
                            })
                            .expect(
                                "off-diagonal self contact survives the disjoint source ranges",
                            );
                        assert!(origin.is_certified_transverse());
                        assert_eq!(
                            origin.tangent_cross_sign(),
                            Some(if reverse_first ^ reverse_second ^ swapped {
                                hyperreal::RealSign::Positive
                            } else {
                                hyperreal::RealSign::Negative
                            })
                        );
                        let parameters = if swapped {
                            [q(3, 4), q(1, 4)]
                        } else {
                            [q(1, 4), q(3, 4)]
                        };
                        for ((curve, location), expected) in
                            [(first, origin.first()), (second, origin.second())]
                                .into_iter()
                                .zip(parameters)
                        {
                            let parameter = decided(location.parameter(&policy).unwrap());
                            assert_eq!(
                                parameter
                                    .compare(&expected.into(), &CurveContext::STRICT)
                                    .unwrap()
                                    .value,
                                Classification::Decided(std::cmp::Ordering::Equal)
                            );
                            let point = curve.point_at(&parameter, &policy).unwrap();
                            assert_eq!(point.certainty, CurveCertainty::Certified);
                            assert!(decided(
                                point
                                    .value
                                    .coincides_with(&p(0, 0).into(), &CurveContext::STRICT)
                                    .value
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn native_retraced_overlaps_survive_independent_restriction() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 0), p(0, 0)));
        let independent = Curve2::from(
            RationalBezier2::try_new(vec![p(0, 0), p(2, 0), p(0, 0)], vec![r(1); 3])
                .unwrap()
                .elevated_to_degree(5)
                .unwrap(),
        );
        for (first, second) in [
            (&source, &source),
            (&source, &independent),
            (&independent, &source),
        ] {
            let result = first.intersect_curve(second, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(result.value.is_complete(), "{:?}", result.value.blockers());
            assert!(result.value.contacts().is_empty());
            // x=4t(1-t) has both t=u and t=1-u parameter components.
            // A single identity correspondence cannot be reused on opposite
            // restrictions even though their geometric images coincide.
            let mut restrictions = Vec::new();
            for overlap in result.value.overlaps() {
                if let Some(overlap) = decided(
                    overlap
                        .restrict(
                            [r(0).into(), q(1, 4).into()],
                            [q(3, 4).into(), r(1).into()],
                            &policy,
                        )
                        .unwrap()
                        .into_value(),
                ) {
                    restrictions.push(overlap);
                }
            }
            assert_eq!(restrictions.len(), 1);
            let mut overlap = restrictions.pop().unwrap();
            assert_eq!(
                overlap.orientation(),
                RationalBezierOverlapOrientation2::Reversed
            );
            for _ in 0..8 {
                overlap = decided(
                    overlap
                        .restrict(
                            [q(1, 8).into(), q(1, 4).into()],
                            [q(3, 4).into(), q(7, 8).into()],
                            &policy,
                        )
                        .unwrap()
                        .into_value(),
                )
                .unwrap();
                for (a, b) in [
                    (
                        overlap.first_range().start(),
                        overlap.second_range().start(),
                    ),
                    (overlap.first_range().end(), overlap.second_range().end()),
                ] {
                    let a = first.point_at(a, &policy).unwrap().into_value();
                    let b = second.point_at(b, &policy).unwrap().into_value();
                    assert!(decided(a.coincides_with(&b, &policy).value));
                }
            }
            let a = first
                .subcurve(q(1, 8).into(), q(1, 4).into(), &policy)
                .unwrap()
                .into_value();
            let b = second
                .subcurve(q(3, 4).into(), q(7, 8).into(), &policy)
                .unwrap()
                .into_value();
            let fresh = a.intersect_curve(&b, &policy).unwrap();
            assert_eq!(fresh.certainty, CurveCertainty::Certified);
            assert!(fresh.value.is_complete(), "{:?}", fresh.value.blockers());
            assert_eq!(fresh.value.overlaps().len(), 1);
        }
    }
}

#[test]
fn native_nodal_overlap_keeps_transverse_parameter_pairs_and_topology() {
    // x=12(2t-1)^2, y=12((2t-1)^3-(2t-1)/4).
    // The diagonal overlap coexists with the ordered visits (1/4,3/4)
    // and (3/4,1/4) to the transverse double point (3,0).
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = Curve2::from(CubicBezier2::new(
            p(12, -9),
            p(-4, 13),
            p(-4, -13),
            p(12, 9),
        ));
        for reversed in [false, true] {
            let second = if reversed {
                source.reversed(&policy).unwrap().into_value()
            } else {
                source.clone()
            };
            let result = source.intersect_curve(&second, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(result.value.is_complete(), "{:?}", result.value.blockers());
            assert_eq!(result.value.contacts().len(), 2);
            assert_eq!(result.value.overlaps().len(), 1);
            for (a, b) in [(q(1, 4), q(3, 4)), (q(3, 4), q(1, 4))] {
                let b = if reversed { r(1) - b } else { b };
                let contact = result
                    .value
                    .contacts()
                    .iter()
                    .find(|contact| {
                        decided(contact.first().parameter(&policy).unwrap())
                            .compare(&a.clone().into(), &policy)
                            .unwrap()
                            .into_value()
                            == Classification::Decided(std::cmp::Ordering::Equal)
                    })
                    .unwrap();
                assert_eq!(
                    decided(contact.second().parameter(&policy).unwrap())
                        .compare(&b.into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(std::cmp::Ordering::Equal)
                );
                assert!(contact.is_certified_transverse());
                assert!(decided(
                    contact
                        .point()
                        .coincides_with(&p(3, 0).into(), &policy)
                        .value
                ));
            }
            let topology = source.intersection_topology(&second, &policy).unwrap();
            assert_eq!(topology.certainty, CurveCertainty::Certified);
            assert!(topology.value.result().is_complete());
            assert_eq!(topology.value.first().len(), 3);
            assert_eq!(topology.value.second().len(), 3);
            for piece in topology.value.first() {
                let replay = piece.intersect_curve(&second, &policy).unwrap();
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert!(replay.value.is_complete(), "{:?}", replay.value.blockers());
            }
        }
    }
}

#[test]
fn native_nodal_spline_contacts_retain_authored_charts() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let controls = vec![p(12, -9), p(-4, 13), p(-4, -13), p(12, 9)];
        let knots = [2, 2, 2, 2, 6, 6, 6, 6].map(r).to_vec();
        let polynomial =
            Curve2::try_polynomial_bspline(3, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .into_value();
        let rational = Curve2::try_nurbs(3, controls, vec![r(1); 4], knots, &policy)
            .unwrap()
            .into_value();
        for (a, b) in [(&polynomial, &rational), (&rational, &polynomial)] {
            let result = a.intersect_curve(b, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(result.value.is_complete(), "{:?}", result.value.blockers());
            assert_eq!(result.value.contacts().len(), 2);
            for expected in [(3, 5), (5, 3)] {
                let contact = result
                    .value
                    .contacts()
                    .iter()
                    .find(|contact| {
                        decided(contact.first().parameter(&policy).unwrap())
                            .compare(&r(expected.0).into(), &policy)
                            .unwrap()
                            .into_value()
                            == Classification::Decided(std::cmp::Ordering::Equal)
                    })
                    .unwrap();
                let first = decided(contact.first().parameter(&policy).unwrap());
                let second = decided(contact.second().parameter(&policy).unwrap());
                assert_eq!(
                    second
                        .compare(&r(expected.1).into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(std::cmp::Ordering::Equal)
                );
                for (curve, parameter) in [(a, first), (b, second)] {
                    let point = curve.point_at(&parameter, &policy).unwrap();
                    assert_eq!(point.certainty, CurveCertainty::Certified);
                    assert!(decided(
                        point.value.coincides_with(contact.point(), &policy).value
                    ));
                }
            }
        }
    }
}

#[test]
fn retained_retraced_domains_retain_every_parameter_component() {
    let selecting = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0)));
    let crossing = Curve2::from(
        LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), Point2::new(q(1, 2), r(1))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let selected = selecting
            .intersect_curve(&crossing, &policy)
            .unwrap()
            .into_value();
        let cut = decided(selected.contacts()[0].first().parameter(&policy).unwrap());
        let source = Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 0), p(0, 0)));
        let (first, second) = source.split_at(cut, &policy).unwrap().into_value();
        let result = first
            .intersect_curve(&second, &policy)
            .unwrap()
            .into_value();
        // Both domains contain the segment from zero to 4s(1-s). The
        // anti-diagonal gives its correspondence; the diagonal also retains
        // the selected common endpoint as a distinct parameter pair.
        assert!(result.is_complete(), "{:?}", result.blockers());
        assert_eq!(result.overlaps().len(), 1);
        assert_eq!(result.contacts().len(), 1);
        for overlap in result.overlaps() {
            for (a, b) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                let a = first.point_at(a, &policy).unwrap();
                let b = second.point_at(b, &policy).unwrap();
                assert_eq!(a.certainty, CurveCertainty::Certified);
                assert_eq!(b.certainty, CurveCertainty::Certified);
                assert_eq!(
                    a.value.coincides_with(&b.value, &policy).value,
                    Classification::Decided(true)
                );
            }
        }
        for (a, b) in [(&first, &second), (&second, &first)] {
            let topology = a.intersection_topology(b, &policy).unwrap();
            assert_eq!(topology.certainty, CurveCertainty::Certified);
            assert!(topology.value.result().is_complete());
            for piece in topology.value.first() {
                let replay = piece.intersect_curve(b, &policy).unwrap();
                assert_eq!(replay.certainty, CurveCertainty::Certified);
                assert!(replay.value.is_complete(), "{:?}", replay.value.blockers());
            }
        }
    }
}

#[test]
fn top_level_shared_component_retains_certified_overlap() {
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let second = first.clone();
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();

    assert!(evidence.is_complete());
    assert!(!evidence.is_disjoint());
    assert!(evidence.contacts().is_empty());
    assert!(evidence.blockers().is_empty());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(
        evidence.overlaps()[0].orientation(),
        RationalBezierOverlapOrientation2::Same
    );
    let graph = topology.arrangement_graph();
    assert_eq!(graph.len(), 2);
    let traversal =
        decided(graph.traverse_retained_deduplicating_materialized_overlaps(&CurveContext::STRICT));
    assert_eq!(traversal.shadowed_fragment_indices(), &[1]);
    assert_eq!(traversal.traversal().len(), 1);
}

#[test]
fn independently_rebuilt_degree_elevated_rational_image_is_a_complete_overlap() {
    let base =
        RationalBezier2::try_new(vec![p(0, 0), p(2, 3), p(4, 0)], vec![r(1), r(2), r(1)]).unwrap();
    let elevated = base.elevated_to_degree(5).unwrap();
    let independent = decided(
        RationalBezier2::from_homogeneous_controls(
            elevated.homogeneous_controls().to_vec(),
            &CurveContext::STRICT,
        )
        .unwrap(),
    );
    let first = Curve2::from(base);
    let second = Curve2::from(independent);

    let evidence = first
        .intersect_curve(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(
        evidence.overlaps()[0]
            .first_range()
            .start()
            .scalar()
            .unwrap(),
        &Real::zero()
    );
    assert_eq!(
        evidence.overlaps()[0].first_range().end().scalar().unwrap(),
        &Real::one()
    );
    assert_eq!(
        evidence.overlaps()[0]
            .second_range()
            .start()
            .scalar()
            .unwrap(),
        &Real::zero()
    );
    assert_eq!(
        evidence.overlaps()[0]
            .second_range()
            .end()
            .scalar()
            .unwrap(),
        &Real::one()
    );
    assert_eq!(
        evidence.overlaps()[0].orientation(),
        RationalBezierOverlapOrientation2::Same
    );
}

#[test]
fn top_level_partial_nonlinear_overlap_splits_at_retained_ranges() {
    let policy = CurveContext::STRICT;
    let source = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .unwrap();
    let first_curve = decided(
        source
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    let second_curve = decided(
        source
            .subcurve_between_exact(&q(1, 4), &Real::one(), &policy)
            .unwrap(),
    );
    let first = Curve2::new(CurveGeometry2::RationalBezier(first_curve));
    let second = Curve2::new(CurveGeometry2::RationalBezier(second_curve));

    let topology = first
        .intersection_topology(&second, &policy)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert!(evidence.contacts().is_empty());
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = &evidence.overlaps()[0];
    assert_eq!(overlap.first_span_index(), 0);
    assert_eq!(overlap.second_span_index(), 0);
    assert_eq!(overlap.first_range().start().scalar().unwrap(), &q(1, 3));
    assert_eq!(overlap.first_range().end().scalar().unwrap(), &Real::one());
    assert_eq!(
        overlap.second_range().start().scalar().unwrap(),
        &Real::zero()
    );
    assert_eq!(overlap.second_range().end().scalar().unwrap(), &q(2, 3));

    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 2);
    let graph = topology.arrangement_graph();
    assert_eq!(graph.len(), 4);
    let traversal = decided(graph.traverse_retained_deduplicating_materialized_overlaps(&policy));
    assert_eq!(traversal.shadowed_fragment_indices().len(), 1);
}

#[test]
fn top_level_line_image_overlap_preserves_algebraic_split_boundary() {
    let first = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![p(0, 0), Point2::new(q(1, 4), r(0)), p(1, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let second = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(vec![Point2::new(q(1, 2), r(0)), p(1, 0)], vec![r(1), r(1)])
            .unwrap(),
    ));

    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert!(matches!(
        evidence.overlaps()[0]
            .first_range()
            .start()
            .as_bezier_parameter(),
        Some(BezierParameter2::Algebraic(_))
    ));

    assert_eq!(topology.first().len(), 2);
    assert!(decided(
        topology.first()[1]
            .start()
            .coincides_with(&Point2::new(q(1, 2), r(0)).into(), &CurveContext::STRICT)
            .value
    ));
    assert_eq!(topology.second().len(), 1);
    assert_eq!(topology.arrangement_graph().len(), 3);

    let first_path = CurvePath2::try_new(vec![first]).unwrap();
    let second_path = CurvePath2::try_new(vec![second]).unwrap();
    let path_topology = first_path
        .intersection_topology(&second_path, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(path_topology.first()[0].curves().len(), 2);
    assert_eq!(path_topology.second()[0].curves().len(), 1);
}

#[test]
fn promoted_region_boolean_consumes_algebraic_line_image_overlap_boundary() {
    let parameterized_bottom = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![p(0, 0), Point2::new(q(1, 4), r(0)), p(1, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let first = CurvePath2::try_new(vec![
        parameterized_bottom,
        Curve2::from(LineSeg2::try_new(p(1, 0), p(1, 1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, 1), p(0, 1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 1), p(0, 0)).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::from(
            LineSeg2::try_new(Point2::new(q(1, 2), r(0)), Point2::new(q(1, 2), r(-1))).unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), p(1, -1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, -1), p(1, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, 0), Point2::new(q(1, 2), r(0))).unwrap()),
    ])
    .unwrap();

    let evidence = first
        .intersect_path(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert!(matches!(
        evidence.overlaps()[0]
            .overlap()
            .first_range()
            .start()
            .as_bezier_parameter(),
        Some(BezierParameter2::Algebraic(_))
    ));

    let region = boolean_paths(
        &first,
        &second,
        BooleanOp::Union,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &CurveContext::STRICT,
    );
    assert_eq!(region.boundary_loops().len(), 1);
}

#[test]
fn promoted_region_boolean_consumes_irrational_polynomial_graph_overlap() {
    let partial_parabola = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![
                Point2::new(q(1, 2), q(1, 4)),
                Point2::new(q(3, 4), q(1, 2)),
                p(1, 1),
            ],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let nonlinear_parabola = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 8), r(0)),
            Point2::new(q(1, 3), q(1, 24)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![r(1); 5],
    )
    .unwrap();
    let first = CurvePath2::try_new(vec![
        partial_parabola,
        Curve2::from(LineSeg2::try_new(p(1, 1), Point2::new(q(1, 2), q(1, 4))).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::new(CurveGeometry2::RationalBezier(
            nonlinear_parabola.reversed(),
        )),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(0, -1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, -1), p(1, -1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, -1), p(1, 1)).unwrap()),
    ])
    .unwrap();

    let evidence = first
        .intersect_path(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert!(matches!(
        evidence.overlaps()[0]
            .overlap()
            .second_range()
            .start()
            .as_bezier_parameter(),
        Some(BezierParameter2::Algebraic(_))
    ));

    let region = boolean_paths(
        &first,
        &second,
        BooleanOp::Union,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &CurveContext::STRICT,
    );
    let exported = region.boundary_paths(&CurveContext::STRICT).unwrap();
    assert_eq!(exported.certainty, hypercurve::CurveCertainty::Certified);
    let Classification::Decided(paths) = exported.value else {
        panic!("lossless exact boundaries")
    };
    assert!(
        paths
            .iter()
            .flat_map(|path| path.curves())
            .any(|curve| curve.geometry().is_none())
    );
}

#[test]
fn region_boolean_reports_terminal_use_after_explicit_path_promotion() {
    let (first_x, second_x) = support::terminally_equal_pair(Real::pi() + Real::e());
    let first = symbolic_rectangle_path(first_x);
    let second = symbolic_rectangle_path(second_x);
    let approximate = CurveContext::APPROXIMATE_512;

    let strict_first = path_region(
        &first,
        CurveBoundaryInteriorSide2::Left,
        &CurveContext::STRICT,
    );
    let strict_second = path_region(
        &second,
        CurveBoundaryInteriorSide2::Left,
        &CurveContext::STRICT,
    );
    let strict = strict_first
        .boolean_region(&strict_second, BooleanOp::Union, &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Boolean
    ));

    let approximate_first = path_region(&first, CurveBoundaryInteriorSide2::Left, &approximate);
    let approximate_second = path_region(&second, CurveBoundaryInteriorSide2::Left, &approximate);
    let region = approximate_first
        .boolean_region(&approximate_second, BooleanOp::Union, &approximate)
        .expect("region Boolean must report the shared terminal");
    assert_eq!(region.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(region.value.boundary_loops().len(), 1);
}

#[test]
fn top_level_polynomial_trims_reuse_certified_source_lineage() {
    let source = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
        p(0, 0),
        p(1, 3),
        p(3, 3),
        p(4, 0),
    )));
    let first = source
        .subcurve(Real::zero().into(), q(3, 4).into(), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let second = source
        .subcurve(q(1, 4).into(), Real::one().into(), &CurveContext::STRICT)
        .unwrap()
        .into_value();

    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(
        evidence.overlaps()[0]
            .first_range()
            .start()
            .scalar()
            .unwrap(),
        &q(1, 3)
    );
    assert_eq!(
        evidence.overlaps()[0].first_range().end().scalar().unwrap(),
        &Real::one()
    );
    assert_eq!(
        evidence.overlaps()[0]
            .second_range()
            .start()
            .scalar()
            .unwrap(),
        &Real::zero()
    );
    assert_eq!(
        evidence.overlaps()[0]
            .second_range()
            .end()
            .scalar()
            .unwrap(),
        &q(2, 3)
    );
    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 2);

    let reversed = second.reversed(&CurveContext::STRICT).unwrap().into_value();
    let reversed_evidence = first
        .intersect_curve(&reversed, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(reversed_evidence.is_complete());
    assert_eq!(reversed_evidence.overlaps().len(), 1);
    assert_eq!(
        reversed_evidence.overlaps()[0]
            .second_range()
            .scalar_endpoints(),
        Some((&Real::one(), &q(1, 3)))
    );
    assert_eq!(
        reversed_evidence.overlaps()[0].orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn top_level_disjoint_curves_produce_a_complete_empty_evidence() {
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap());
    let second = Curve2::from(LineSeg2::try_new(p(0, 2), p(1, 2)).unwrap());
    let evidence = first
        .intersect_curve(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert!(evidence.is_complete());
    assert!(evidence.is_disjoint());
    assert!(evidence.contacts().is_empty());
    assert!(evidence.blockers().is_empty());
}

#[test]
fn top_level_arc_dispatch_filters_circle_witnesses_and_retains_exact_parameters() {
    let first =
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap());
    let second =
        Curve2::from(CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap());
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 4);

    assert!(evidence.is_complete());
    assert_eq!(evidence.contacts().len(), 1);
    let contact = &evidence.contacts()[0];
    assert!(contact.first().local_parameter().scalar().is_some());
    assert!(contact.second().local_parameter().scalar().is_some());
    assert!(matches!((contact.point()).coordinates(), Some(point) if point == &p(4, 3)));
    assert_eq!(topology.result().contacts().len(), 1);
}

#[test]
fn curve_and_path_intersections_report_terminal_use_without_upgrading_arc_caches() {
    let undecidable_zero = support::terminally_unresolved_zero();
    let arc = Curve2::from(
        CircularArc2::try_from_center(
            p(3, 0),
            p(3, 2),
            Point2::new(Real::from(3_i8) + undecidable_zero, Real::one()),
            false,
        )
        .unwrap(),
    );
    let line = Curve2::from(LineSeg2::try_new(p(2, 1), p(5, 1)).unwrap());

    let approximate = arc
        .intersect_curve(&line, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal must resolve the ambiguous semicircle");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(approximate.value.is_complete());
    assert_eq!(approximate.value.contacts().len(), 1);
    assert!(matches!(
        (approximate.value.contacts()[0].point()).coordinates(),
        Some(_)
    ));

    let strict = arc
        .intersect_curve(&line, &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Intersection
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let topology = arc
        .intersection_topology(&line, &CurveContext::APPROXIMATE_512)
        .expect("topology must replay the authorized terminal from retained arc facts");
    assert_eq!(topology.certainty, CurveCertainty::Approximate512Consumed);
    assert!(topology.value.result().is_complete());
    assert_eq!(topology.value.result().contacts().len(), 1);

    let arc_path = CurvePath2::try_new(vec![arc]).unwrap();
    let line_path = CurvePath2::try_new(vec![line]).unwrap();
    let path_result = arc_path
        .intersect_path(&line_path, &CurveContext::APPROXIMATE_512)
        .expect("path intersection must preserve terminal certainty");
    assert_eq!(
        path_result.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(path_result.value.is_complete());
    assert_eq!(path_result.value.contacts().len(), 1);

    let strict_path = arc_path
        .intersection_topology(&line_path, &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict_path,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Intersection
                && blocker.reason() == UncertaintyReason::RealSign
    ));
}

#[test]
fn native_line_arc_dispatch_preserves_operand_order_and_exact_parameters() {
    let line = Curve2::from(LineSeg2::try_new(p(4, -4), p(4, 4)).unwrap());
    let arc =
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap());
    let policy = CurveContext::STRICT;

    let topology = line
        .intersection_topology(&arc, &policy)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 2);
    assert!(evidence.is_complete());
    assert_eq!(evidence.contacts().len(), 1);
    assert_eq!(
        decided(
            evidence.contacts()[0]
                .first()
                .parameter(&CurveContext::STRICT)
                .unwrap()
        )
        .scalar()
        .cloned(),
        Some(q(7, 8))
    );
    assert!(
        evidence.contacts()[0]
            .second()
            .local_parameter()
            .scalar()
            .is_some()
    );
    assert!(
        matches!((evidence.contacts()[0].point()).coordinates(), Some(point) if point == &p(4, 3))
    );
    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 2);

    let reversed_evidence = arc.intersect_curve(&line, &policy).unwrap().into_value();
    assert_eq!(reversed_evidence.contacts().len(), 1);
    assert!(
        reversed_evidence.contacts()[0]
            .first()
            .local_parameter()
            .scalar()
            .is_some()
    );
    assert_eq!(
        decided(
            reversed_evidence.contacts()[0]
                .second()
                .parameter(&CurveContext::STRICT)
                .unwrap()
        )
        .scalar()
        .cloned(),
        Some(q(7, 8))
    );
}

#[test]
fn native_arc_dispatch_retains_partial_same_circle_overlap_ranges() {
    let first =
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap());
    let second =
        Curve2::from(CircularArc2::try_from_center(p(4, 3), p(0, 5), p(0, 0), false).unwrap());
    let policy = CurveContext::STRICT;
    let topology = first
        .intersection_topology(&second, &policy)
        .unwrap()
        .into_value();
    let evidence = topology.result();

    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = &evidence.overlaps()[0];
    assert_ne!(
        overlap.first_range().start().scalar().unwrap(),
        &Real::zero()
    );
    assert_eq!(overlap.first_range().end().scalar().unwrap(), &Real::one());
    assert_eq!(
        overlap.second_range().start().scalar().unwrap(),
        &Real::zero()
    );
    assert_eq!(overlap.second_range().end().scalar().unwrap(), &Real::one());
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );

    assert_eq!(topology.first().len(), 3);
    assert_eq!(topology.second().len(), 1);

    let reversed =
        Curve2::from(CircularArc2::try_from_center(p(0, 5), p(4, 3), p(0, 0), true).unwrap());
    let reversed_evidence = first
        .intersect_curve(&reversed, &policy)
        .unwrap()
        .into_value();
    assert_eq!(reversed_evidence.overlaps().len(), 1);
    let reversed_overlap = &reversed_evidence.overlaps()[0];
    assert_eq!(
        reversed_overlap.second_range().start().scalar().unwrap(),
        &Real::one()
    );
    assert_eq!(
        reversed_overlap.second_range().end().scalar().unwrap(),
        &Real::zero()
    );
    assert_eq!(
        reversed_overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn promoted_region_boolean_resolves_partial_same_circle_arc_boundaries() {
    let first = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-5, 0), p(5, 0)).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(p(4, 3), p(0, 5), p(0, 0), false).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 5), p(4, 3)).unwrap()),
    ])
    .unwrap();
    let first_area = first
        .boundary_loop(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .signed_area(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let first_area = decided(first_area).unwrap();
    let second_area = second
        .boundary_loop(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .signed_area(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let second_area = decided(second_area).unwrap();
    let evidence = first
        .intersect_path(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);

    let cases = [
        (BooleanOp::Union, first_area.clone()),
        (BooleanOp::Intersection, second_area.clone()),
        (BooleanOp::Difference, &first_area - &second_area),
        (BooleanOp::Xor, &first_area - &second_area),
    ];
    for (operation, expected_area) in cases {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        );
        assert!(
            region
                .boundary_loops()
                .iter()
                .all(|loop_| !loop_.is_empty())
        );
        let actual_area = region
            .signed_area(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let actual_area = decided(actual_area)
            .unwrap_or_else(|| panic!("{operation:?} did not retain an exact area"));
        assert_real_close(&actual_area, &expected_area, 1.0e-10);
    }
}

fn rectangle(x0: i32, y0: i32, x1: i32, y1: i32) -> CurvePath2 {
    let points = [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
    CurvePath2::try_new(
        (0..4)
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(points[index].clone(), points[(index + 1) % 4].clone())
                        .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn closed_under_curve(curve: Curve2, lower_y: i32) -> CurvePath2 {
    let start = curve.start().clone();
    let end = curve.end().clone();
    let lower_end = Point2::new(
        end.coordinates().expect("native endpoint").x().clone(),
        r(lower_y),
    );
    let lower_start = Point2::new(
        start.coordinates().expect("native endpoint").x().clone(),
        r(lower_y),
    );
    CurvePath2::try_new(vec![
        curve,
        Curve2::from(
            LineSeg2::try_new(
                (end).coordinates().expect("native endpoint").clone(),
                lower_end.clone(),
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(lower_end, lower_start.clone()).unwrap()),
        Curve2::from(
            LineSeg2::try_new(
                lower_start,
                (start).coordinates().expect("native endpoint").clone(),
            )
            .unwrap(),
        ),
    ])
    .unwrap()
}

#[test]
fn promoted_region_boolean_consumes_partial_nonlinear_shared_boundary() {
    let policy = CurveContext::STRICT;
    let source = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
        p(0, 0),
        p(1, 3),
        p(3, 3),
        p(4, 0),
    )));
    let first_curve = source
        .subcurve(Real::zero().into(), q(3, 4).into(), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let second_curve = source
        .subcurve(q(1, 4).into(), Real::one().into(), &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let first = closed_under_curve(first_curve, -5);
    let second = closed_under_curve(second_curve, -6);
    let evidence = first.intersect_path(&second, &policy).unwrap().into_value();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(
        evidence.overlaps()[0]
            .overlap()
            .first_range()
            .start()
            .scalar()
            .unwrap(),
        &q(1, 3)
    );
    assert_eq!(
        evidence.overlaps()[0]
            .overlap()
            .second_range()
            .end()
            .scalar()
            .unwrap(),
        &q(2, 3)
    );

    for operation in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Right,
            CurveBoundaryInteriorSide2::Right,
            &policy,
        );
        assert!(
            region
                .boundary_loops()
                .iter()
                .all(|loop_| !loop_.is_empty())
        );
        assert!(!region.boundary_loops().is_empty());
        assert!(
            decided(
                region
                    .signed_area(&CurveContext::STRICT)
                    .unwrap()
                    .into_value()
            )
            .is_some()
        );
    }
}

#[test]
fn path_pair_immediate_topology_splits_each_authored_curve_once() {
    let first = rectangle(0, 0, 2, 2);
    let second = rectangle(1, -1, 3, 1);
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.authored_curve_pair_count(), 16);
    assert_eq!(evidence.candidate_curve_pair_count(), 2);
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert!(evidence.overlaps().is_empty());

    assert_eq!(topology.first().len(), 4);
    assert_eq!(topology.second().len(), 4);
    assert_eq!(
        topology
            .first()
            .iter()
            .map(|split| split.curves().len())
            .sum::<usize>(),
        6
    );
    assert_eq!(
        topology
            .second()
            .iter()
            .map(|split| split.curves().len())
            .sum::<usize>(),
        6
    );
    assert_eq!(topology.arrangement_graph().len(), 12);
    assert_eq!(topology.arrangement_graph().len(), 12);
}

#[test]
fn path_overlap_orientation_feeds_canonical_region_boolean_side_logic() {
    let first = rectangle(0, 0, 2, 2);
    let same = first.clone();
    let reversed = first.reversed(&CurveContext::STRICT).unwrap().into_value();
    let policy = CurveContext::STRICT;
    let same_evidence = first.intersect_path(&same, &policy).unwrap().into_value();
    let reversed_evidence = first
        .intersect_path(&reversed, &policy)
        .unwrap()
        .into_value();

    assert_eq!(same_evidence.overlaps().len(), 4);
    assert!(same_evidence.overlaps().iter().all(|overlap| {
        overlap.overlap().orientation() == RationalBezierOverlapOrientation2::Same
    }));
    assert_eq!(reversed_evidence.overlaps().len(), 4);
    assert!(reversed_evidence.overlaps().iter().all(|overlap| {
        overlap.overlap().orientation() == RationalBezierOverlapOrientation2::Reversed
    }));

    for (second, second_side) in [
        (&same, CurveBoundaryInteriorSide2::Left),
        (&reversed, CurveBoundaryInteriorSide2::Right),
    ] {
        for (operation, expected_area) in [
            (BooleanOp::Union, r(4)),
            (BooleanOp::Intersection, r(4)),
            (BooleanOp::Difference, r(0)),
            (BooleanOp::Xor, r(0)),
        ] {
            let region = boolean_paths(
                &first,
                second,
                operation,
                CurveBoundaryInteriorSide2::Left,
                second_side,
                &policy,
            );
            assert_eq!(
                decided(region.signed_area(&policy).unwrap().into_value()),
                Some(expected_area),
                "{operation:?}, second side {second_side:?}"
            );
        }
    }
}

#[test]
fn native_line_dispatch_retains_partial_overlap_ranges_and_split_endpoints() {
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap());
    let second = Curve2::from(LineSeg2::try_new(p(2, 0), p(6, 0)).unwrap());
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();

    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = &evidence.overlaps()[0];
    assert_eq!(overlap.first_range().start().scalar().unwrap(), &q(1, 2));
    assert_eq!(overlap.first_range().end().scalar().unwrap(), &r(1));
    assert_eq!(overlap.second_range().start().scalar().unwrap(), &r(0));
    assert_eq!(overlap.second_range().end().scalar().unwrap(), &q(1, 2));
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );

    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 2);

    let reversed = Curve2::from(LineSeg2::try_new(p(6, 0), p(2, 0)).unwrap());
    let reversed_evidence = first
        .intersect_curve(&reversed, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let reversed_overlap = &reversed_evidence.overlaps()[0];
    assert_eq!(
        reversed_overlap.second_range().start().scalar().unwrap(),
        &r(1)
    );
    assert_eq!(
        reversed_overlap.second_range().end().scalar().unwrap(),
        &q(1, 2)
    );
    assert_eq!(
        reversed_overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn promoted_region_boolean_resolves_partial_reversed_shared_line_boundaries() {
    let first = rectangle(0, 0, 2, 4);
    let second = rectangle(2, 1, 4, 3);
    let evidence = first
        .intersect_path(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = evidence.overlaps()[0].overlap();
    assert_eq!(overlap.first_range().start().scalar().unwrap(), &q(1, 4));
    assert_eq!(overlap.first_range().end().scalar().unwrap(), &q(3, 4));
    assert_eq!(overlap.second_range().start().scalar().unwrap(), &r(1));
    assert_eq!(overlap.second_range().end().scalar().unwrap(), &r(0));
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );

    let cases = [
        (BooleanOp::Union, r(12)),
        (BooleanOp::Intersection, r(0)),
        (BooleanOp::Difference, r(8)),
        (BooleanOp::Xor, r(12)),
    ];
    for (operation, expected_area) in cases {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        );
        assert_eq!(
            decided(
                region
                    .signed_area(&CurveContext::STRICT)
                    .unwrap()
                    .into_value()
            ),
            Some(expected_area)
        );
    }
}

#[test]
fn promoted_region_boolean_materializes_exact_regularized_operation_matrix() {
    let first = rectangle(0, 0, 2, 2);
    let second = rectangle(1, -1, 3, 1);
    let policy = CurveContext::STRICT;
    let cases = [
        (BooleanOp::Union, r(7)),
        (BooleanOp::Intersection, r(1)),
        (BooleanOp::Difference, r(3)),
        (BooleanOp::Xor, r(6)),
    ];

    for (operation, expected_area) in cases {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        );
        assert!(
            region
                .boundary_loops()
                .iter()
                .all(|loop_| !loop_.is_empty())
        );
        assert_eq!(
            decided(region.signed_area(&policy).unwrap().into_value()),
            Some(expected_area)
        );
    }

    let direct = boolean_paths(
        &first,
        &second,
        BooleanOp::Union,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &policy,
    );
    assert_eq!(
        decided(direct.signed_area(&policy).unwrap().into_value()),
        Some(r(7))
    );
}

#[test]
fn promoted_region_boolean_consumes_complete_shared_boundaries() {
    let first = rectangle(0, 0, 2, 2);
    let second = first.clone();
    let evidence = first
        .intersect_path(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(evidence.overlaps().len(), 4);
    let cases = [
        (BooleanOp::Union, r(4)),
        (BooleanOp::Intersection, r(4)),
        (BooleanOp::Difference, r(0)),
        (BooleanOp::Xor, r(0)),
    ];

    for (operation, expected_area) in cases {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        );
        assert_eq!(
            decided(
                region
                    .signed_area(&CurveContext::STRICT)
                    .unwrap()
                    .into_value()
            ),
            Some(expected_area)
        );
    }
}

#[test]
fn promoted_region_boolean_preserves_disjoint_exact_conic_boundaries() {
    let circle = |center_x: i32| {
        CurvePath2::try_new(vec![Curve2::from(
            CircularArc2::try_from_center(
                p(center_x + 1, 0),
                p(center_x + 1, 0),
                p(center_x, 0),
                false,
            )
            .unwrap(),
        )])
        .unwrap()
    };
    let first = circle(0);
    let second = circle(4);
    let union = boolean_paths(
        &first,
        &second,
        BooleanOp::Union,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &CurveContext::STRICT,
    );
    assert_eq!(union.boundary_loops().len(), 2);

    let intersection = boolean_paths(
        &first,
        &second,
        BooleanOp::Intersection,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &CurveContext::STRICT,
    );
    assert!(intersection.is_empty());
}

#[test]
fn promoted_region_boolean_traverses_overlapping_circles_with_exact_radical_splits() {
    let circle = |center_x: i32| {
        CurvePath2::try_new(vec![Curve2::from(
            CircularArc2::try_from_center(
                p(center_x + 1, 0),
                p(center_x + 1, 0),
                p(center_x, 0),
                false,
            )
            .unwrap(),
        )])
        .unwrap()
    };
    let first = circle(0);
    let second = circle(1);
    let evidence = first
        .intersect_path(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert!(evidence.contacts().iter().all(|contact| {
        contact
            .contact()
            .first()
            .local_parameter()
            .scalar()
            .is_some()
            && contact
                .contact()
                .second()
                .local_parameter()
                .scalar()
                .is_some()
    }));

    for operation in [BooleanOp::Union, BooleanOp::Intersection] {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        );
        assert_eq!(region.boundary_loops().len(), 1);
        assert!(!region.boundary_loops()[0].has_algebraic_fragments());
    }
}

#[test]
fn path_difference_and_xor_reverse_algebraic_parabola_contacts_exactly() {
    let first = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4))),
        Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap()),
    ])
    .unwrap();
    let second = rectangle(-3, 2, 3, 5);
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    let pieces = topology.first()[0].curves();
    assert_eq!(pieces.len(), 3);
    let root = r(2).sqrt().unwrap();
    for (piece, x) in pieces.iter().zip([-root.clone(), root]) {
        assert!(decided(
            piece
                .end()
                .coincides_with(&Point2::new(x, r(2)).into(), &CurveContext::STRICT)
                .value
        ));
    }

    for operation in [BooleanOp::Difference, BooleanOp::Xor] {
        let region = boolean_paths(
            &first,
            &second,
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        );
        assert!(region.has_algebraic_fragments());
        assert_eq!(
            region
                .classify_point(&p(0, 1), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside),
            "{operation:?} retained algebraic interior"
        );
        assert_eq!(
            region
                .classify_point(&p(0, 3), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Outside),
            "{operation:?} retained algebraic overlap interior"
        );
        assert_eq!(
            region
                .classify_point(&p(0, 0), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Boundary),
            "{operation:?} retained algebraic boundary"
        );
        let transformed = region
            .transform_affine(
                &r(-2),
                &r(0),
                &r(0),
                &r(3),
                &r(7),
                &r(-1),
                &CurveContext::STRICT,
            )
            .unwrap_or_else(|error| panic!("{operation:?} affine transform: {error:?}"))
            .into_value();
        assert!(transformed.has_algebraic_fragments());
        for (point, expected) in [
            (p(7, 2), RegionPointLocation::Inside),
            (p(7, 8), RegionPointLocation::Outside),
            (p(7, -1), RegionPointLocation::Boundary),
        ] {
            assert_eq!(
                transformed
                    .classify_point(&point, &CurveContext::STRICT)
                    .unwrap()
                    .into_value(),
                Classification::Decided(expected),
                "{operation:?} transformed algebraic classification"
            );
        }
    }
}

fn equivalent_parabola_curves() -> Vec<(CurveFamily2, Curve2)> {
    let controls = [p(-2, 4), p(0, -4), p(2, 4)];
    let elevated_controls = [
        controls[0].clone(),
        Point2::new(q(-2, 3), q(-4, 3)),
        Point2::new(q(2, 3), q(-4, 3)),
        controls[2].clone(),
    ];
    vec![
        (
            CurveFamily2::QuadraticBezier,
            Curve2::from(QuadraticBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
            )),
        ),
        (
            CurveFamily2::CubicBezier,
            Curve2::from(CubicBezier2::new(
                elevated_controls[0].clone(),
                elevated_controls[1].clone(),
                elevated_controls[2].clone(),
                elevated_controls[3].clone(),
            )),
        ),
        (
            CurveFamily2::RationalQuadraticBezier,
            Curve2::from(
                RationalQuadraticBezier2::try_new(
                    controls[0].clone(),
                    controls[1].clone(),
                    controls[2].clone(),
                    r(1),
                    r(1),
                    r(1),
                )
                .unwrap(),
            ),
        ),
        (
            CurveFamily2::RationalBezier,
            Curve2::from(RationalBezier2::try_new(controls.to_vec(), vec![r(1); 3]).unwrap()),
        ),
        (
            CurveFamily2::PolynomialBSpline,
            Curve2::try_polynomial_bspline(
                2,
                controls.to_vec(),
                vec![r(0), r(0), r(0), r(1), r(1), r(1)],
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        ),
        (
            CurveFamily2::Nurbs,
            Curve2::try_nurbs(
                2,
                controls.to_vec(),
                vec![r(1); 3],
                vec![r(0), r(0), r(0), r(1), r(1), r(1)],
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        ),
    ]
}

#[test]
fn equivalent_top_level_families_complete_independent_region_booleans() {
    let cutter = rectangle(-3, 2, 3, 5);
    let policy = CurveContext::STRICT;
    for (family, curve) in equivalent_parabola_curves() {
        let source = CurvePath2::try_new(vec![
            curve,
            Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap()),
        ])
        .unwrap();
        let evidence = source
            .intersect_path(&cutter, &policy)
            .unwrap()
            .into_value();
        assert!(
            evidence.is_complete(),
            "{family:?}: {:#?}",
            evidence.blockers()
        );
        assert_eq!(evidence.contacts().len(), 2, "{family:?}");

        for operation in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            let _region = boolean_paths(
                &source,
                &cutter,
                operation,
                CurveBoundaryInteriorSide2::Left,
                CurveBoundaryInteriorSide2::Left,
                &policy,
            );
        }
    }
}

#[test]
fn generated_fillet_arcs_intersect_themselves_after_restriction_and_reversal() {
    use hypercurve::{CurveCornerMode2, CurveCornerSolutions2};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
            QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
        ])
        .unwrap();
        let fillet = source
            .fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, &policy)
            .unwrap();
        assert_eq!(fillet.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(path) = fillet.value else {
            panic!("unique exact fillet")
        };
        let circle = &path.curves()[1];
        assert_eq!(circle.family(), CurveFamily2::CircularArc);
        assert!(
            circle.geometry().is_none(),
            "fixture must retain its selected construction"
        );
        let domain = circle.parameter_domain();
        let interior = (1..16)
            .map(|n| hypercurve::CurveParameter2::from(q(n, 16)))
            .filter(|parameter| {
                let lower = domain.start().compare(parameter, &policy).unwrap();
                let upper = parameter.compare(domain.end(), &policy).unwrap();
                assert_eq!(lower.certainty, CurveCertainty::Certified);
                assert_eq!(upper.certainty, CurveCertainty::Certified);
                decided(lower.value).is_lt() && decided(upper.value).is_lt()
            })
            .collect::<Vec<_>>();
        assert!(interior.len() >= 2);
        let restricted = circle
            .subcurve(
                interior[0].clone(),
                interior.last().unwrap().clone(),
                &policy,
            )
            .unwrap();
        assert_eq!(restricted.certainty, CurveCertainty::Certified);
        for source in [circle.clone(), restricted.value] {
            for first_reversed in [false, true] {
                for second_reversed in [false, true] {
                    let first = if first_reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let second = if second_reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let result = first.intersect_curve(&second, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert!(result.value.is_complete(), "{:?}", result.value.blockers());
                    assert!(result.value.contacts().is_empty());
                    assert_eq!(result.value.overlaps().len(), 1);
                    let overlap = &result.value.overlaps()[0];
                    assert_eq!(
                        overlap.orientation(),
                        if first_reversed ^ second_reversed {
                            RationalBezierOverlapOrientation2::Reversed
                        } else {
                            RationalBezierOverlapOrientation2::Same
                        }
                    );
                    for (a, b) in [
                        (
                            overlap.first_range().start(),
                            overlap.second_range().start(),
                        ),
                        (overlap.first_range().end(), overlap.second_range().end()),
                    ] {
                        let a = first.point_at(a, &policy).unwrap();
                        let b = second.point_at(b, &policy).unwrap();
                        assert_eq!(a.certainty, CurveCertainty::Certified);
                        assert_eq!(b.certainty, CurveCertainty::Certified);
                        let same = a.value.coincides_with(&b.value, &policy);
                        assert_eq!(same.certainty, CurveCertainty::Certified);
                        assert_eq!(same.value, Classification::Decided(true));
                    }
                }
            }
        }
    }
}

#[test]
fn generated_fillet_arcs_keep_tangent_contacts_with_their_trimmed_neighbors() {
    use hypercurve::{CurveCornerMode2, CurveCornerSolutions2};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
            QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
        ])
        .unwrap();
        let result = path
            .fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, &policy)
            .unwrap();
        assert_eq!(result.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(path) = result.value else {
            panic!("unique fillet")
        };
        for index in [0, 2] {
            for (reversed, other_reversed) in
                [(false, false), (false, true), (true, false), (true, true)]
            {
                let circle = if reversed {
                    path.curves()[1].reversed(&policy).unwrap().value
                } else {
                    path.curves()[1].clone()
                };
                let other = if other_reversed {
                    path.curves()[index].reversed(&policy).unwrap().value
                } else {
                    path.curves()[index].clone()
                };
                let other = &other;
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (other, &circle)
                    } else {
                        (&circle, other)
                    };
                    let result = first.intersect_curve(second, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert!(
                        result.value.is_complete(),
                        "neighbor {index}: {:?}",
                        result.value.blockers()
                    );
                    assert!(result.value.overlaps().is_empty());
                    assert_eq!(result.value.contacts().len(), 1);
                    let contact = &result.value.contacts()[0];
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(hyperreal::RealSign::Zero)
                    );
                    for (curve, location) in [(first, contact.first()), (second, contact.second())]
                    {
                        let parameter = decided(location.parameter(&policy).unwrap());
                        let point = curve.point_at(&parameter, &policy).unwrap();
                        assert_eq!(point.certainty, CurveCertainty::Certified);
                        let same = point.value.coincides_with(contact.point(), &policy);
                        assert_eq!(same.certainty, CurveCertainty::Certified);
                        assert_eq!(
                            same.value,
                            Classification::Decided(true),
                            "neighbor {index}, reversed {reversed}/{other_reversed}, swapped {swapped}, replay {:?}",
                            curve.family()
                        );
                    }
                }
            }
        }
    }
}
