use hypercurve::{
    CircularArc2, Classification, ContourPointLocation, Curve2, CurveCertainty, CurveContext,
    CurveCornerMode2, CurveCornerSolutions2, CurveFamily2, CurvePath2, CurvePoint2, CurveRegion2,
    LineSeg2, Point2, QuadraticBezier2, Real, RegionPointLocation,
};

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn assert_same_point(actual: &CurvePoint2, expected: &CurvePoint2, policy: &CurveContext) {
    let equal = actual.coincides_with(expected, policy);
    assert_eq!(equal.certainty, CurveCertainty::Certified);
    assert_eq!(equal.value, Classification::Decided(true));
}

fn assert_open_path(path: &CurvePath2, start: &Point2, end: &Point2, policy: &CurveContext) {
    assert_same_point(&path.curves()[0].start(), &start.clone().into(), policy);
    assert_same_point(
        &path.curves().last().unwrap().end(),
        &end.clone().into(),
        policy,
    );
    for pair in path.curves().windows(2) {
        assert_same_point(&pair[0].end(), &pair[1].start(), policy);
    }
}

#[test]
fn boundary_admission_rejects_disconnected_spline_spans() {
    use hypercurve::{CurveError, ExactCurveError, NurbsCurve2, PolynomialSplineCurve2};

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // The outer endpoints coincide, but the two linear spans jump from
        // (1,0) to (2,0) at the fully repeated interior knot.
        let controls = vec![p(0, 0), p(1, 0), p(2, 0), p(0, 0)];
        let knots = vec![0, 0, 1, 1, 2, 2]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>();
        let spline = PolynomialSplineCurve2::try_new(1, controls.clone(), knots.clone(), &policy)
            .unwrap()
            .value;
        let nurbs = NurbsCurve2::try_new(1, controls, vec![Real::one(); 4], knots, &policy)
            .unwrap()
            .value;
        for curve in [Curve2::from(spline), Curve2::from(nurbs)] {
            let path = CurvePath2::try_new(vec![curve]).unwrap();
            assert_same_point(&path.start(), &path.end(), &policy);
            for error in [
                path.boundary_loop(&policy).unwrap_err(),
                CurveRegion2::try_from_boundary_paths(&[path.clone()], &policy).unwrap_err(),
            ] {
                assert!(matches!(
                    error,
                    ExactCurveError::Invalid {
                        cause: CurveError::DisconnectedCurvePath,
                        ..
                    }
                ));
            }
        }
    }
}

#[test]
fn selected_open_chamfers_reenter_the_public_path_api() {
    let start = p(-4, 0);
    let end = p(1, 2);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(start.clone(), p(0, 0)).unwrap()),
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), end.clone())),
        ])
        .unwrap();
        let first = path
            .chamfer_vertex_by_setbacks(
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap();
        assert_eq!(first.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(mut edited) = first.value else {
            panic!("the exact setback has one solution")
        };
        assert!(
            edited
                .curves()
                .iter()
                .any(|curve| curve.geometry().is_none())
        );
        assert_open_path(&edited, &start, &end, &policy);
        for denominator in [4, 16, 64] {
            let next = edited
                .chamfer_vertex_by_setbacks(
                    1,
                    q(1, denominator),
                    q(1, denominator),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(next.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(next) = next.value else {
                panic!("a selected open path accepts another exact setback")
            };
            assert_open_path(&next, &start, &end, &policy);
            edited = next;
        }
    }
}

#[test]
fn selected_spline_chamfers_keep_every_untrimmed_span() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let controls = vec![p(0, 0), p(0, 1), p(1, 2), p(3, 3)];
        let knots = vec![3, 3, 3, 5, 7, 7, 7]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>();
        let sources = [
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .value,
            Curve2::try_nurbs(2, controls, vec![Real::one(); 4], knots, &policy)
                .unwrap()
                .value,
        ];
        for source in sources {
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
                source.clone(),
            ])
            .unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let outcome = path
                    .chamfer_vertex_by_setbacks(
                        1,
                        Real::one(),
                        Real::one(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Unique(edited) = outcome.value else {
                    panic!("the incident spline span has one selected setback")
                };
                let (start, end) = if reversed {
                    (p(3, 3), p(-4, 0))
                } else {
                    (p(-4, 0), p(3, 3))
                };
                assert_open_path(&edited, &start, &end, &policy);
                let untouched_index = if reversed {
                    0
                } else {
                    edited.curves().len() - 1
                };
                let midpoint = edited.curves()[untouched_index]
                    .point_at(&q(1, 2).into(), &policy)
                    .unwrap();
                let authored = source.point_at(&Real::from(6).into(), &policy).unwrap();
                assert_same_point(&midpoint.value, &authored.value, &policy);
            }
        }
    }
}

#[test]
fn selected_chamfers_preserve_all_major_arc_contacts() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let end = Point2::new(q(3, 5), q(4, 5));
        let path = CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(p(0, -2), p(1, -1), p(1, 0))),
            Curve2::from(
                CircularArc2::try_from_center(p(1, 0), end.clone(), p(0, 0), true).unwrap(),
            ),
        ])
        .unwrap();
        for setback in [1, 2] {
            let outcome = path
                .chamfer_vertex_by_setbacks(
                    1,
                    Real::one(),
                    Real::from(setback),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            let candidates = match outcome.value {
                CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                CurveCornerSolutions2::Multiple(candidates) => candidates,
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("lost major-arc contacts: {reason:?}")
                }
            };
            assert_eq!(candidates.len(), if setback == 1 { 2 } else { 1 });
            for candidate in &candidates {
                assert_open_path(candidate, &p(0, -2), &end, &policy);
                let contact = candidate.curves()[1].end();
                let expected = if setback == 2 {
                    vec![p(-1, 0)]
                } else {
                    let y = (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap();
                    vec![Point2::new(q(1, 2), y.clone()), Point2::new(q(1, 2), -y)]
                };
                assert!(expected.into_iter().any(|point| {
                    contact.coincides_with(&point.into(), &policy).value
                        == Classification::Decided(true)
                }));
            }
        }
    }
}

#[test]
fn selected_path_chamfers_close_through_all_region_booleans() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
            Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap()),
        ])
        .unwrap();
        // Rotating to the edited closing vertex also exercises publication of
        // the untouched middle of a closed path.
        for vertex in [1, 0] {
            let path = if vertex == 0 {
                let mut curves = path.curves().to_vec();
                curves.rotate_left(1);
                CurvePath2::try_new(curves).unwrap()
            } else {
                path.clone()
            };
            let edited = path
                .chamfer_vertex_by_setbacks(
                    vertex,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(edited.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(edited) = edited.value else {
                panic!("the closed path has one selected chamfer")
            };
            assert_same_point(
                &edited.curves()[0].start(),
                &edited.curves().last().unwrap().end(),
                &policy,
            );
            let source = CurveRegion2::try_from_boundary_paths(&[edited], &policy).unwrap();
            assert_eq!(source.certainty, CurveCertainty::Certified);
            let corners = [
                Point2::new(q(-1, 2), Real::from(-1)),
                p(2, -1),
                p(2, 3),
                Point2::new(q(-1, 2), Real::from(3)),
            ];
            let cutter = CurvePath2::try_new(
                (0..4)
                    .map(|i| {
                        Curve2::from(
                            LineSeg2::try_new(corners[i].clone(), corners[(i + 1) % 4].clone())
                                .unwrap(),
                        )
                    })
                    .collect(),
            )
            .unwrap();
            let cutter = CurveRegion2::try_from_boundary_paths(&[cutter], &policy).unwrap();
            assert_eq!(cutter.certainty, CurveCertainty::Certified);
            let booleans = source
                .value
                .boolean_regions(&cutter.value, &policy)
                .unwrap();
            assert_eq!(booleans.certainty, CurveCertainty::Certified);
            let booleans = booleans.value;
            for (region, expected) in [
                (booleans.union(), [true, true, true, false]),
                (booleans.intersection(), [false, false, true, false]),
                (booleans.difference(), [true, false, false, false]),
                (booleans.xor(), [true, true, false, false]),
            ] {
                for (query, inside) in [p(-2, 1), p(1, 0), p(0, 1), p(3, 1)]
                    .into_iter()
                    .zip(expected)
                {
                    let location = region.classify_point(&query, &policy).unwrap();
                    assert_eq!(location.certainty, CurveCertainty::Certified);
                    assert_eq!(
                        location.value,
                        Classification::Decided(if inside {
                            RegionPointLocation::Inside
                        } else {
                            RegionPointLocation::Outside
                        })
                    );
                }
            }
        }
    }
}

#[test]
fn selected_open_fillets_accept_a_subsequent_chamfer() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let start = p(-4, 0);
        let end = p(1, 2);
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(start.clone(), p(0, 0)).unwrap()),
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), end.clone())),
        ])
        .unwrap();
        let filleted = path
            .fillet_vertex_by_radius(1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
            .unwrap();
        assert_eq!(filleted.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(filleted) = filleted.value else {
            panic!("the incident line/parabola has one selected fillet")
        };
        assert_open_path(&filleted, &start, &end, &policy);
        assert!(filleted.curves().iter().any(|curve| {
            curve.family() == CurveFamily2::CircularArc && curve.geometry().is_none()
        }));
        let chamfered = filleted
            .chamfer_vertex_by_setbacks(1, q(1, 16), q(1, 16), CurveCornerMode2::TrimOnly, &policy)
            .unwrap();
        assert_eq!(chamfered.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(chamfered) = chamfered.value else {
            panic!("the selected fillet accepts an exact setback")
        };
        assert_open_path(&chamfered, &start, &end, &policy);
    }
}

#[test]
fn selected_spline_fillets_preserve_knot_charts_and_other_spans() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let controls = vec![p(0, 0), p(0, 1), p(1, 2), p(3, 3)];
        let knots = vec![3, 3, 3, 5, 7, 7, 7]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>();
        let sources = [
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .value,
            Curve2::try_nurbs(2, controls, vec![Real::one(); 4], knots, &policy)
                .unwrap()
                .value,
        ];
        for source in sources {
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
                source.clone(),
            ])
            .unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let outcome = path
                    .fillet_vertex_by_radius(1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Unique(edited) = outcome.value else {
                    panic!("the incident spline span has one selected fillet")
                };
                let (start, end) = if reversed {
                    (p(3, 3), p(-4, 0))
                } else {
                    (p(-4, 0), p(3, 3))
                };
                assert_open_path(&edited, &start, &end, &policy);
                let untouched_index = if reversed {
                    0
                } else {
                    edited.curves().len() - 1
                };
                let midpoint = edited.curves()[untouched_index]
                    .point_at(&q(1, 2).into(), &policy)
                    .unwrap();
                let authored = source.point_at(&Real::from(6).into(), &policy).unwrap();
                assert_same_point(&midpoint.value, &authored.value, &policy);
            }
        }
    }
}

#[test]
fn selected_major_arc_fillets_keep_the_complete_authored_sweep() {
    for clockwise in [false, true] {
        check_major_arc_fillet(clockwise);
    }
}

fn check_major_arc_fillet(clockwise: bool) {
    let corner = p(1, 1);
    let center = Point2::new(Real::one(), q(3923, 2150));
    let arc_end = Point2::new(
        if clockwise {
            q(3923, 2150)
        } else {
            q(377, 2150)
        },
        q(3923, 2150),
    );
    let arc = Curve2::from(
        CircularArc2::try_from_center(corner.clone(), arc_end.clone(), center, clockwise).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(
                p(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                corner.clone(),
            )),
            arc.clone(),
        ])
        .unwrap();
        for reversed in [false, true] {
            let path = if reversed {
                path.reversed(&policy).unwrap().value
            } else {
                path.clone()
            };
            let source = &path.curves()[usize::from(!reversed)];
            let spans = source.native_bezier_fragments(&policy).unwrap().value;
            assert!(spans.len() > 1, "the fixture crosses projective charts");
            let preserved = if !clockwise {
                0..spans.len()
            } else if reversed {
                0..spans.len() - 1
            } else {
                1..spans.len()
            };
            let untouched = spans[preserved]
                .iter()
                .map(|span| {
                    Curve2::from(span.curve().clone())
                        .point_at(&q(1, 2).into(), &policy)
                        .unwrap()
                        .value
                        .coordinates()
                        .expect("a native chart sample has exact coordinates")
                        .clone()
                })
                .collect::<Vec<_>>();
            let outcome = path
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
                .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            let candidates = match outcome.value {
                CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                CurveCornerSolutions2::Multiple(candidates) => candidates,
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the major-arc fillet was lost: {reason:?}")
                }
            };
            // The counterclockwise support is tangent at the parabola's
            // exact extension parameter 6/5. The opposite source orientation
            // selects different fillets; retaining that contact would reverse
            // the tangent at the arc join.
            let exact_contact = CurvePoint2::from(Point2::new(q(6, 5), q(36, 25)));
            let (start, end) = if reversed {
                (arc_end.clone(), p(0, 0))
            } else {
                (p(0, 0), arc_end.clone())
            };
            for candidate in &candidates {
                assert_open_path(candidate, &start, &end, &policy);
            }
            let closing = Curve2::from(LineSeg2::try_new(end, start).unwrap());
            let selected = candidates
                .iter()
                .filter(|candidate| {
                    clockwise
                        || candidate.curves().iter().any(|curve| {
                            let endpoint = if reversed { curve.start() } else { curve.end() };
                            endpoint.coincides_with(&exact_contact, &policy).value
                                == Classification::Decided(true)
                        })
                })
                .find_map(|candidate| {
                    let mut curves = candidate.curves().to_vec();
                    curves.push(closing.clone());
                    let closed = CurvePath2::try_new_with_policy(curves, &policy).unwrap();
                    assert_eq!(closed.certainty, CurveCertainty::Certified);
                    untouched
                        .iter()
                        .all(|point| {
                            let location = closed.value.classify_point(point, &policy).unwrap();
                            assert_eq!(location.certainty, CurveCertainty::Certified);

                            location.value
                                == Classification::Decided(ContourPointLocation::Boundary)
                        })
                        .then_some(closed.value)
                });
            assert!(
                selected.is_some(),
                "the fillet preserves the complete source sweep: clockwise={clockwise}, reversed={reversed}, policy={policy:?}"
            );
        }
    }
}

#[test]
fn homogeneous_boundary_closes_through_boolean_corners_and_offset() {
    use hypercurve::{FillRule, HomogeneousControl2, OffsetCornerStyle2, RationalBezier2};
    let admit = |path, policy: &CurveContext| {
        let admitted = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[hypercurve::CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            policy,
        )
        .unwrap();
        assert_eq!(admitted.certainty, CurveCertainty::Certified);
        admitted.into_value()
    };
    let check = |region: &CurveRegion2, policy: &CurveContext| {
        assert_eq!(region.boundary_loops().len(), 1);
        for (point, expected) in [
            (Point2::new(q(1, 4), q(1, 4)), RegionPointLocation::Inside),
            (Point2::new(q(-1, 2), q(1, 2)), RegionPointLocation::Outside),
        ] {
            let result = region.classify_point(&point, policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value, Classification::Decided(expected));
        }
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // The middle homogeneous control is at infinity; the exact upper
        // semicircle and its complete authored denominator remain finite.
        let Classification::Decided(curve) = RationalBezier2::from_homogeneous_controls(
            vec![
                HomogeneousControl2::new(Real::one(), Real::zero(), Real::one()),
                HomogeneousControl2::new(Real::zero(), Real::one(), Real::zero()),
                HomogeneousControl2::new(-Real::one(), Real::zero(), Real::one()),
            ],
            &policy,
        )
        .unwrap() else {
            panic!("the homogeneous semicircle must construct");
        };
        assert!(curve.affine_control_points().is_none());
        // Independently authored elevated controls must recover their small
        // exact source even though that source has no affine control net.
        let Classification::Decided(elevated) = RationalBezier2::from_homogeneous_controls(
            curve
                .elevated_to_degree(12)
                .unwrap()
                .homogeneous_controls()
                .to_vec(),
            &policy,
        )
        .unwrap() else {
            panic!("the elevated homogeneous semicircle must construct");
        };
        let nurbs = hypercurve::NurbsCurve2::from_homogeneous_controls(
            2,
            curve.homogeneous_controls().to_vec(),
            vec![
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::one(),
                Real::one(),
            ],
            hypercurve::SplinePeriodicity2::NonPeriodic,
            &policy,
        )
        .unwrap()
        .into_value();
        for curve in [
            Curve2::from(curve),
            Curve2::from(elevated),
            Curve2::from(nurbs.elevated_to_degree(12, &policy).unwrap().into_value()),
            Curve2::from(nurbs),
        ] {
            let material = admit(
                CurvePath2::try_new(vec![
                    curve,
                    Curve2::from(LineSeg2::try_new(p(-1, 0), p(1, 0)).unwrap()),
                ])
                .unwrap(),
                &policy,
            );
            let rectangle = admit(
                CurvePath2::try_new(
                    [p(0, -1), p(2, -1), p(2, 2), p(0, 2), p(0, -1)]
                        .windows(2)
                        .map(|points| {
                            Curve2::from(
                                LineSeg2::try_new(points[0].clone(), points[1].clone()).unwrap(),
                            )
                        })
                        .collect(),
                )
                .unwrap(),
                &policy,
            );
            let results = material.boolean_regions(&rectangle, &policy).unwrap();
            assert_eq!(results.certainty, CurveCertainty::Certified);
            let clipped = results.value.intersection();
            check(clipped, &policy);
            for fillet in [false, true] {
                let solutions = if fillet {
                    clipped.fillet_loop_vertex_by_radius(
                        0,
                        1,
                        q(1, 8),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                } else {
                    clipped.chamfer_loop_vertex_by_setbacks(
                        0,
                        1,
                        q(1, 8),
                        q(1, 8),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                }
                .unwrap();
                assert_eq!(solutions.certainty, CurveCertainty::Certified);
                let regions = match solutions.value {
                    CurveCornerSolutions2::Unique(region) => vec![region],
                    CurveCornerSolutions2::Multiple(regions) => regions,
                    CurveCornerSolutions2::NoSolution(reason) => {
                        panic!("the clipped corner must admit an edit: {reason:?}")
                    }
                };
                assert!(!regions.is_empty());
                for edited in regions {
                    check(&edited, &policy);
                    let displaced = edited
                        .offset(q(1, 32), &OffsetCornerStyle2::Round, &policy)
                        .unwrap();
                    assert_eq!(displaced.certainty, CurveCertainty::Certified);
                    let replay = displaced
                        .value
                        .boolean_regions(&rectangle, &policy)
                        .unwrap();
                    assert_eq!(replay.certainty, CurveCertainty::Certified);
                    check(replay.value.intersection(), &policy);
                }
            }
        }
    }
}

mod finite_fixed_distance_domains {
    use hypercurve::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn exact<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("{reason:?}"),
        }
    }
    fn certified<T>(value: CurveOutcome<T>) -> T {
        assert_eq!(value.certainty, CurveCertainty::Certified);
        value.value
    }
    fn same(first: &CurvePoint2, second: &CurvePoint2, policy: &CurveContext) {
        assert_eq!(
            certified(first.coincides_with(second, policy)),
            Classification::Decided(true)
        );
    }
    fn cap(chart: usize, policy: &CurveContext) -> Curve2 {
        // All four charts cover P(t)=(-1/8+t^2,t), 0<=t<=1/8.
        // The rational exterior chart has t=(s-2)/(2s-1), s in [2,5/2].
        // Its genuine pole at s=1/2 is outside the requested interval.
        let exterior = chart & 1 != 0;
        let rational = chart & 2 != 0;
        let (parallel, start, end) = if rational {
            let (points, weights, start, end) = if exterior {
                (
                    vec![
                        Point2::new(q(31, 8), 2.into()),
                        Point2::new(q(-17, 8), q(1, 2)),
                        Point2::new(q(7, 8), (-1).into()),
                    ],
                    vec![1.into(), (-1).into(), 1.into()],
                    Real::from(2),
                    q(5, 2),
                )
            } else {
                (
                    vec![
                        Point2::new(q(-1, 8), Real::zero()),
                        Point2::new(q(-1, 8), q(1, 16)),
                        Point2::new(q(-7, 64), q(1, 8)),
                    ],
                    vec![9.into(), 12.into(), 16.into()],
                    Real::zero(),
                    Real::one(),
                )
            };
            (
                RationalBezier2::try_new(points, weights)
                    .unwrap()
                    .parallel_left(q(1, 64))
                    .unwrap(),
                start,
                end,
            )
        } else {
            let (source, start, end) = if exterior {
                (
                    QuadraticBezier2::new(
                        Point2::new(q(31, 8), (-2).into()),
                        Point2::new(q(15, 8), q(-3, 2)),
                        Point2::new(q(7, 8), (-1).into()),
                    ),
                    Real::from(2),
                    q(17, 8),
                )
            } else {
                (
                    QuadraticBezier2::new(
                        Point2::new(q(-1, 8), Real::zero()),
                        Point2::new(q(-1, 8), q(1, 16)),
                        Point2::new(q(-7, 64), q(1, 8)),
                    ),
                    Real::zero(),
                    Real::one(),
                )
            };
            (source.parallel_left(q(1, 64)).unwrap(), start, end)
        };
        let range = exact(
            BezierParameterRange2::try_new(
                BezierParameter2::Exact(start),
                BezierParameter2::Exact(end),
                policy,
            )
            .unwrap(),
        );
        Curve2::from(exact(
            BezierParallelFragment2::try_new(parallel, range, policy).unwrap(),
        ))
    }
    fn run(chart: usize, repeat: bool) {
        let (mut cases, mut successes, mut replays, mut failures) = (0, 0, 0, 0);
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let curved = cap(chart, &policy);
            let corner = Point2::new(q(-9, 64), Real::zero());
            same(&curved.start(), &corner.clone().into(), &policy);
            let line = LineSeg2::try_new(
                Point2::new(corner.x() - Real::one(), Real::zero()),
                corner.clone(),
            )
            .unwrap();
            let original = CurvePath2::try_new(vec![line.into(), curved.clone()]).unwrap();
            for reversed in [false, true] {
                cases += 1;
                let path = if reversed {
                    certified(original.reversed(&policy).unwrap())
                } else {
                    original.clone()
                };
                println!(
                    "begin chart={chart} reversed={reversed} repeat={repeat} policy={policy:?}"
                );
                let result = path.chamfer_vertex_by_setbacks(
                    1,
                    q(1, 128),
                    q(1, 128),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                );
                let modified = match result {
                    Ok(CurveOutcome {
                        certainty: CurveCertainty::Certified,
                        value: CurveCornerSolutions2::Unique(path),
                    }) => path,
                    other => {
                        failures += 1;
                        println!("first chamfer: {other:?}");
                        continue;
                    }
                };
                println!("first chamfer returned");
                assert_eq!(modified.curves().len(), 3);
                same(&modified.start(), &path.start(), &policy);
                same(&modified.end(), &path.end(), &policy);
                let chord = &modified.curves()[1];
                assert_eq!(chord.family(), CurveFamily2::Line);
                let expected_line_cut: CurvePoint2 = Point2::new(q(-19, 128), Real::zero()).into();
                same(
                    &if reversed { chord.end() } else { chord.start() },
                    &expected_line_cut,
                    &policy,
                );
                for adjacent in modified.curves().windows(2) {
                    same(&adjacent[0].end(), &adjacent[1].start(), &policy);
                    replays += 1;
                }
                let survivor = &modified.curves()[if reversed { 0 } else { 2 }];
                for endpoint in [
                    survivor.parameter_domain().start(),
                    survivor.parameter_domain().end(),
                ] {
                    let first = certified(survivor.point_at(endpoint, &policy).unwrap());
                    let original_curve = &path.curves()[if reversed { 0 } else { 1 }];
                    same(
                        &first,
                        &certified(original_curve.point_at(endpoint, &policy).unwrap()),
                        &policy,
                    );
                    replays += 1;
                }
                if repeat {
                    println!("begin repeated chamfer");
                    let index = if reversed { 1 } else { 2 };
                    match modified.chamfer_vertex_by_setbacks(
                        index,
                        q(1, 256),
                        q(1, 256),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    ) {
                        Ok(CurveOutcome {
                            certainty: CurveCertainty::Certified,
                            value: CurveCornerSolutions2::Unique(next),
                        }) => {
                            assert_eq!(next.curves().len(), 4);
                            same(&next.start(), &path.start(), &policy);
                            same(&next.end(), &path.end(), &policy);
                            for adjacent in next.curves().windows(2) {
                                same(&adjacent[0].end(), &adjacent[1].start(), &policy);
                                replays += 1;
                            }
                        }
                        other => {
                            failures += 1;
                            println!("repeated chamfer: {other:?}");
                            continue;
                        }
                    }
                }
                successes += 1;
                println!("complete chart={chart} reversed={reversed} policy={policy:?}");
            }
        }
        println!(
            "{{\"cases\":{cases},\"successes\":{successes},\"point_replays\":{replays},\"failures\":{failures}}}"
        );
        assert_eq!(failures, 0);
    }

    #[test]
    fn finite_chamfers_preserve_equivalent_source_charts() {
        for chart in 0..4 {
            run(chart, false);
        }
    }

    #[test]
    fn repeated_finite_chamfers_preserve_selected_cuts() {
        for chart in 0..4 {
            run(chart, true);
        }
    }
}

mod finite_selected_point_domains {
    use hypercurve::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn exact<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("{reason:?}"),
        }
    }
    fn certified<T>(value: CurveOutcome<T>) -> T {
        assert_eq!(value.certainty, CurveCertainty::Certified);
        value.value
    }
    fn same(first: &CurvePoint2, second: &CurvePoint2, policy: &CurveContext) {
        assert_eq!(
            certified(first.coincides_with(second, policy)),
            Classification::Decided(true)
        );
    }
    fn cap(chart: usize, policy: &CurveContext) -> Curve2 {
        // All four charts cover P(t)=(-1/8+t^2,t), 0<=t<=1/8.
        // The rational exterior chart has t=(s-2)/(2s-1), s in [2,5/2].
        // Its genuine pole at s=1/2 is outside the requested interval.
        let exterior = chart & 1 != 0;
        let rational = chart & 2 != 0;
        let (parallel, start, end) = if rational {
            let (points, weights, start, end) = if exterior {
                (
                    vec![
                        Point2::new(q(31, 8), 2.into()),
                        Point2::new(q(-17, 8), q(1, 2)),
                        Point2::new(q(7, 8), (-1).into()),
                    ],
                    vec![1.into(), (-1).into(), 1.into()],
                    Real::from(2),
                    q(5, 2),
                )
            } else {
                (
                    vec![
                        Point2::new(q(-1, 8), Real::zero()),
                        Point2::new(q(-1, 8), q(1, 16)),
                        Point2::new(q(-7, 64), q(1, 8)),
                    ],
                    vec![9.into(), 12.into(), 16.into()],
                    Real::zero(),
                    Real::one(),
                )
            };
            (
                RationalBezier2::try_new(points, weights)
                    .unwrap()
                    .parallel_left(q(1, 64))
                    .unwrap(),
                start,
                end,
            )
        } else {
            let (source, start, end) = if exterior {
                (
                    QuadraticBezier2::new(
                        Point2::new(q(31, 8), (-2).into()),
                        Point2::new(q(15, 8), q(-3, 2)),
                        Point2::new(q(7, 8), (-1).into()),
                    ),
                    Real::from(2),
                    q(17, 8),
                )
            } else {
                (
                    QuadraticBezier2::new(
                        Point2::new(q(-1, 8), Real::zero()),
                        Point2::new(q(-1, 8), q(1, 16)),
                        Point2::new(q(-7, 64), q(1, 8)),
                    ),
                    Real::zero(),
                    Real::one(),
                )
            };
            (source.parallel_left(q(1, 64)).unwrap(), start, end)
        };
        let range = exact(
            BezierParameterRange2::try_new(
                BezierParameter2::Exact(start),
                BezierParameter2::Exact(end),
                policy,
            )
            .unwrap(),
        );
        Curve2::from(exact(
            BezierParallelFragment2::try_new(parallel, range, policy).unwrap(),
        ))
    }
    fn run(chart: usize) {
        let (mut cases, mut successes, mut replays, mut failures) = (0, 0, 0, 0);
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let curved = cap(chart, &policy);
            let corner = Point2::new(q(-9, 64), Real::zero());
            let center = Point2::new(q(-9, 64), q(-1, 32));
            let arc = CircularArc2::try_from_center(
                Point2::new(q(-11, 64), q(-1, 32)),
                corner.clone(),
                center,
                true,
            )
            .unwrap();
            let original = CurvePath2::try_new(vec![arc.into(), curved]).unwrap();
            for reversed in [false, true] {
                cases += 1;
                let path = if reversed {
                    certified(original.reversed(&policy).unwrap())
                } else {
                    original.clone()
                };
                println!("begin chart={chart} reversed={reversed} policy={policy:?}");
                let edited = match path.fillet_vertex_by_radius(
                    1,
                    q(1, 32),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                ) {
                    Ok(CurveOutcome {
                        certainty: CurveCertainty::Certified,
                        value: CurveCornerSolutions2::Unique(path),
                    }) => path,
                    other => {
                        failures += 1;
                        println!("fillet: {other:?}");
                        continue;
                    }
                };
                assert_eq!(edited.curves().len(), 3);
                same(&edited.start(), &path.start(), &policy);
                same(&edited.end(), &path.end(), &policy);
                for pair in edited.curves().windows(2) {
                    same(&pair[0].end(), &pair[1].start(), &policy);
                    replays += 1;
                }
                let retained = &edited.curves()[if reversed { 0 } else { 2 }];
                let source = &path.curves()[if reversed { 0 } else { 1 }];
                for endpoint in [
                    retained.parameter_domain().start(),
                    retained.parameter_domain().end(),
                ] {
                    same(
                        &certified(retained.point_at(endpoint, &policy).unwrap()),
                        &certified(source.point_at(endpoint, &policy).unwrap()),
                        &policy,
                    );
                    replays += 1;
                }
                successes += 1;
                println!("complete chart={chart} reversed={reversed} policy={policy:?}");
            }
        }
        println!(
            "{{\"cases\":{cases},\"successes\":{successes},\"point_replays\":{replays},\"failures\":{failures}}}"
        );
        assert_eq!(failures, 0);
    }

    #[test]
    fn collapsed_circle_fillet_preserves_finite_parallel_domains() {
        for chart in 0..4 {
            run(chart);
        }
    }
}
