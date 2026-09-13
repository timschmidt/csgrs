use hypercurve::{
    CircularArc2, Classification, Curve2, CurveCertainty, CurveContext, CurveCornerMode2,
    CurveCornerSolutions2, CurvePath2, CurvePoint2, CurveRegion2, LineSeg2, Point2,
    QuadraticBezier2, Real, RegionPointLocation,
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
