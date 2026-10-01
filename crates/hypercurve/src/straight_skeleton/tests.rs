use super::*;
use crate::{
    CircularArc2, CubicBezier2, Curve2, LineSeg2, QuadraticBezier2, RationalBezier2,
    RationalQuadraticBezier2, Segment2,
};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn contour(points: &[(i32, i32)]) -> Contour2 {
    let points = points
        .iter()
        .map(|(x, y)| Point2::new(r(*x), r(*y)))
        .collect::<Vec<_>>();
    let segments = (0..points.len())
        .map(|index| {
            Segment2::Line(
                LineSeg2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                )
                .unwrap(),
            )
        })
        .collect();
    Contour2::try_new(segments).unwrap()
}

fn initial_shape_preserving_state(
    contour: &Contour2,
    orientation: RealSign,
) -> (
    Vec<ShapePreservingSupportRecord2>,
    Vec<StraightSkeletonNode2>,
    ActiveShapePreservingCycle2,
) {
    let records = contour
        .segments()
        .iter()
        .enumerate()
        .map(|(source_edge, segment)| ShapePreservingSupportRecord2 {
            geometry: shape_preserving_support(segment, orientation).unwrap(),
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge },
        })
        .collect::<Vec<_>>();
    let nodes = contour
        .segments()
        .iter()
        .enumerate()
        .map(|(source_vertex, segment)| StraightSkeletonNode2 {
            point: segment.start().clone(),
            time: Real::zero(),
            kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
        })
        .collect::<Vec<_>>();
    let count = records.len();
    let active = ActiveShapePreservingCycle2 {
        supports: (0..count).collect(),
        pair_start: (0..count)
            .map(|right| (((right + count - 1) % count, right), right))
            .collect(),
        pair_branch: BTreeMap::new(),
    };
    (records, nodes, active)
}

#[test]
fn curve_path_dispatch_preserves_native_families_and_evidence_capabilities() {
    for family in [CurveFamily2::Line, CurveFamily2::CircularArc] {
        assert_eq!(
            family.straight_skeleton_support(),
            StraightSkeletonCurveFamilySupport2::NativeExact
        );
    }
    for family in [
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::PolynomialBSpline,
    ] {
        assert_eq!(
            family.straight_skeleton_support(),
            StraightSkeletonCurveFamilySupport2::CertifiedLineImage
        );
    }
    for family in [
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::Nurbs,
    ] {
        assert_eq!(
            family.straight_skeleton_support(),
            StraightSkeletonCurveFamilySupport2::CertifiedLineOrCircularArc
        );
    }
    let square = contour(&[(0, 0), (4, 0), (4, 4), (0, 4)]);
    let path = CurvePath2::try_new(
        square
            .segments()
            .iter()
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::from(line.clone()),
                Segment2::Arc(arc) => Curve2::from(arc.clone()),
            })
            .collect(),
    )
    .unwrap();
    let evidence = path.straight_skeleton(&CurveContext::STRICT).unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);

    let line_start = Point2::new(r(0), r(0));
    let line_end = Point2::new(r(4), r(0));
    let line_controls = vec![
        line_start.clone(),
        Point2::new(r(2), r(0)),
        line_end.clone(),
    ];
    for line_image_edge in [
        Curve2::from(QuadraticBezier2::new(
            line_start.clone(),
            Point2::new(r(2), r(0)),
            line_end.clone(),
        )),
        Curve2::from(CubicBezier2::new(
            line_start.clone(),
            Point2::new(r(1), r(0)),
            Point2::new(r(3), r(0)),
            line_end.clone(),
        )),
        Curve2::from(
            RationalQuadraticBezier2::try_new(
                line_start.clone(),
                Point2::new(r(2), r(0)),
                line_end.clone(),
                r(1),
                r(2),
                r(1),
            )
            .unwrap(),
        ),
        Curve2::from(
            RationalBezier2::try_new(
                vec![
                    line_start.clone(),
                    Point2::new(r(1), r(0)),
                    Point2::new(r(3), r(0)),
                    line_end.clone(),
                ],
                vec![r(1), r(2), r(3), r(1)],
            )
            .unwrap(),
        ),
        Curve2::try_polynomial_bspline(
            2,
            line_controls.clone(),
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
        Curve2::try_nurbs(
            2,
            line_controls,
            vec![r(1), r(2), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value(),
    ] {
        let family = line_image_edge.family();
        let path = CurvePath2::try_new(vec![
            line_image_edge,
            Curve2::from(
                LineSeg2::try_new(Point2::new(r(4), r(0)), Point2::new(r(4), r(4))).unwrap(),
            ),
            Curve2::from(
                LineSeg2::try_new(Point2::new(r(4), r(4)), Point2::new(r(0), r(4))).unwrap(),
            ),
            Curve2::from(
                LineSeg2::try_new(Point2::new(r(0), r(4)), Point2::new(r(0), r(0))).unwrap(),
            ),
        ])
        .unwrap();
        let evidence = path.straight_skeleton(&CurveContext::STRICT).unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{family:?}: {evidence:?}"
        );
    }

    let right = Point2::new(r(1), r(0));
    let top = Point2::new(r(0), r(1));
    let center = Point2::new(r(0), r(0));
    let arc =
        CircularArc2::try_from_center(right.clone(), top.clone(), center.clone(), false).unwrap();
    let rational_arc = arc
        .rational_bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .spans()[0]
        .curve()
        .clone();
    let rational_arc_controls = rational_arc
        .control_points()
        .into_iter()
        .cloned()
        .collect::<Vec<_>>();
    let rational_arc_weights = rational_arc
        .weights()
        .into_iter()
        .cloned()
        .collect::<Vec<_>>();
    let general_rational_arc =
        RationalBezier2::try_new(rational_arc_controls.clone(), rational_arc_weights.clone())
            .unwrap();
    let elevated_rational_arc = general_rational_arc.elevated_to_degree(4).unwrap();
    let nurbs_arc = Curve2::try_nurbs(
        2,
        rational_arc_controls.clone(),
        rational_arc_weights.clone(),
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    for curve in [
        Curve2::from(rational_arc),
        Curve2::from(general_rational_arc),
        Curve2::from(elevated_rational_arc),
        nurbs_arc,
    ] {
        let family = curve.family();
        let degree = match curve.geometry() {
            Some(CurveGeometry2::RationalQuadraticBezier(_)) => 2,
            Some(CurveGeometry2::RationalBezier(curve)) => curve.degree(),
            Some(CurveGeometry2::Nurbs(curve)) => curve.degree(),
            _ => unreachable!(),
        };
        let rational_sector = CurvePath2::try_new(vec![
            curve,
            Curve2::from(LineSeg2::try_new(top.clone(), center.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(center.clone(), right.clone()).unwrap()),
        ])
        .unwrap();
        let evidence = rational_sector
            .straight_skeleton(&CurveContext::STRICT)
            .unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{family:?} degree {degree}: {evidence:?}"
        );
    }
    let noncircular_rational = RationalBezier2::try_new(
        vec![
            Point2::new(r(0), r(0)),
            Point2::new(r(1), r(1)),
            Point2::new(r(2), r(0)),
        ],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();
    assert_eq!(
        rational_bezier_circular_arc(&noncircular_rational, &CurveContext::STRICT).unwrap(),
        Classification::Decided(None)
    );

    let first = Point2::new(r(0), r(0));
    let second = Point2::new(r(2), r(0));
    let unsupported = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            first.clone(),
            Point2::new(r(1), r(1)),
            second.clone(),
        )),
        Curve2::from(QuadraticBezier2::new(
            second,
            Point2::new(r(1), r(-1)),
            first,
        )),
    ])
    .unwrap();
    let evidence = unsupported
        .straight_skeleton(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(
        evidence.blocker(),
        Some(&StraightSkeletonBlocker2::UnsupportedCurveFamily {
            curve_index: 0,
            family: CurveFamily2::QuadraticBezier,
        })
    );
}

#[test]
fn mixed_line_arc_vertices_have_exact_parabolic_trajectories() {
    let left = Point2::new(r(-1), r(0));
    let right = Point2::new(r(1), r(0));
    let source = Contour2::try_new(vec![
        Segment2::Arc(
            CircularArc2::try_from_center(
                left.clone(),
                right.clone(),
                Point2::new(r(0), r(0)),
                false,
            )
            .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(right, left).unwrap()),
    ])
    .unwrap();
    let Classification::Decided(trajectories) = source
        .straight_skeleton_vertex_trajectories(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("mixed line/arc trajectories must be decided");
    };
    assert_eq!(trajectories.len(), 2);
    for trajectory in trajectories {
        assert_eq!(
            trajectory.kind(),
            StraightSkeletonTrajectoryKind2::Parabolic
        );
        let StraightSkeletonTrajectoryGeometry2::Conic(conic) = trajectory.geometry() else {
            panic!("line/circle vertex must retain a conic");
        };
        assert_eq!(
            real_sign(&conic.evaluate(trajectory.start()), &CurveContext::STRICT),
            Some(RealSign::Zero)
        );
        assert_eq!(
            trajectory
                .affine_time()
                .unwrap()
                .evaluate(trajectory.start()),
            r(0)
        );
    }
}

#[test]
fn circular_segment_materializes_exact_parabolic_terminal_branches() {
    for clockwise in [false, true] {
        let left = Point2::new(r(-1), r(0));
        let right = Point2::new(r(1), r(0));
        let (arc_start, arc_end) = if clockwise {
            (right.clone(), left.clone())
        } else {
            (left.clone(), right.clone())
        };
        let source = Contour2::try_new(vec![
            Segment2::Arc(
                CircularArc2::try_from_center(
                    arc_start,
                    arc_end,
                    Point2::new(r(0), r(0)),
                    clockwise,
                )
                .unwrap(),
            ),
            Segment2::Line(
                LineSeg2::try_new(
                    if clockwise {
                        left.clone()
                    } else {
                        right.clone()
                    },
                    if clockwise { right } else { left },
                )
                .unwrap(),
            ),
        ])
        .unwrap();
        let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{evidence:?}"
        );
        assert_eq!(evidence.event_count(), 1);
        assert_eq!(evidence.simultaneous_event_count(), 1);
        let skeleton = evidence.skeleton().unwrap();
        let half = (r(1) / r(2)).unwrap();
        assert_eq!(skeleton.maximum_time(), &half);
        assert_eq!(skeleton.nodes().len(), 3);
        assert_eq!(skeleton.arcs().len(), 2);
        let terminal = &skeleton.nodes()[2];
        assert_eq!(terminal.point(), &Point2::new(r(0), -half.clone()));
        assert_eq!(terminal.time(), &half);
        assert!(matches!(
            terminal.kind(),
            StraightSkeletonNodeKind2::EdgeEvent {
                collapsed_source_edges
            } if collapsed_source_edges == &[0, 1]
        ));
        for arc in skeleton.arcs() {
            let StraightSkeletonArcGeometry2::ConicBranch(branch) = arc.geometry() else {
                panic!("circular-segment skeleton branch must retain its timed conic");
            };
            let affine_time = branch
                .affine_time()
                .expect("circular-segment branch must retain affine time");
            assert_eq!(branch.kind(), StraightSkeletonTrajectoryKind2::Parabolic);
            assert_eq!(
                real_sign(
                    &branch.equation().evaluate(terminal.point()),
                    &CurveContext::STRICT
                ),
                Some(RealSign::Zero)
            );
            assert_eq!(affine_time.evaluate(terminal.point()), half);
        }
    }
}

#[test]
fn circular_sector_materializes_exact_three_support_vanish_event() {
    for clockwise in [false, true] {
        let origin = Point2::new(r(0), r(0));
        let right = Point2::new(r(1), r(0));
        let top = Point2::new(r(0), r(1));
        let (arc_start, arc_end, first_line_end, second_line_end) = if clockwise {
            (top.clone(), right.clone(), origin.clone(), top.clone())
        } else {
            (right.clone(), top.clone(), origin.clone(), right.clone())
        };
        let source = Contour2::try_new(vec![
            Segment2::Arc(
                CircularArc2::try_from_center(
                    arc_start,
                    arc_end.clone(),
                    origin.clone(),
                    clockwise,
                )
                .unwrap(),
            ),
            Segment2::Line(LineSeg2::try_new(arc_end, first_line_end.clone()).unwrap()),
            Segment2::Line(LineSeg2::try_new(first_line_end, second_line_end).unwrap()),
        ])
        .unwrap();
        let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{evidence:?}"
        );
        assert_eq!(evidence.event_count(), 1);
        assert_eq!(evidence.simultaneous_event_count(), 1);
        let skeleton = evidence.skeleton().unwrap();
        let event_time = ((-r(2) + r(8).sqrt().unwrap()) / r(2)).unwrap();
        assert_eq!(skeleton.maximum_time(), &event_time);
        assert_eq!(skeleton.nodes().len(), 4);
        assert_eq!(skeleton.arcs().len(), 3);
        let terminal = &skeleton.nodes()[3];
        assert_eq!(terminal.point().x(), &event_time);
        assert_eq!(terminal.point().y(), &event_time);
        assert_eq!(terminal.time(), skeleton.maximum_time());
        let mut line_count = 0;
        let mut conic_count = 0;
        for arc in skeleton.arcs() {
            match arc.geometry() {
                StraightSkeletonArcGeometry2::LineSegment => line_count += 1,
                StraightSkeletonArcGeometry2::ConicBranch(branch) => {
                    let affine_time = branch
                        .affine_time()
                        .expect("sector parabola must retain its affine time coordinate");
                    conic_count += 1;
                    assert_eq!(branch.kind(), StraightSkeletonTrajectoryKind2::Parabolic);
                    let source_node = &skeleton.nodes()[arc.start_node()];
                    assert_eq!(
                        real_sign(
                            &branch.equation().evaluate(source_node.point()),
                            &CurveContext::STRICT
                        ),
                        Some(RealSign::Zero)
                    );
                    assert_eq!(affine_time.evaluate(source_node.point()), r(0));
                    assert_eq!(affine_time.evaluate(terminal.point()), event_time);
                }
            }
        }
        assert_eq!(line_count, 1);
        assert_eq!(conic_count, 2);
    }
}

#[test]
fn local_arc_queue_distinguishes_vanish_and_bubble_candidates() {
    let origin = Point2::new(r(0), r(0));
    let right = Point2::new(r(1), r(0));
    let top = Point2::new(r(0), r(1));
    let sector = Contour2::try_new(vec![
        Segment2::Arc(
            CircularArc2::try_from_center(right.clone(), top.clone(), origin.clone(), false)
                .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(top, origin.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(origin, right).unwrap()),
    ])
    .unwrap();
    let Classification::Decided(events) = sector
        .straight_skeleton_local_arc_events(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("sector local event must be decided");
    };
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind(), StraightSkeletonLocalArcEventKind2::Vanish);

    let left = Point2::new(r(-1), r(0));
    let right = Point2::new(r(1), r(0));
    let upper_left = Point2::new(r(-20), r(2));
    let upper_right = Point2::new(r(20), r(2));
    let bubble_source = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(upper_left.clone(), left.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(left, right.clone(), Point2::new(r(0), r(0)), false)
                .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(right, upper_right.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(upper_right, upper_left).unwrap()),
    ])
    .unwrap();
    let bubble_events = bubble_source
        .straight_skeleton_local_arc_events(&CurveContext::STRICT)
        .unwrap();
    let Classification::Decided(events) = bubble_events else {
        panic!("bubble local event must be decided: {bubble_events:?}");
    };
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind(), StraightSkeletonLocalArcEventKind2::Bubble);
    let Classification::Decided(splices) = bubble_source
        .straight_skeleton_splice_events(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("bubble fixture splice candidates must be decided");
    };
    assert_eq!(splices.len(), 2);
    assert!(
        splices
            .iter()
            .all(|event| [1, 2].contains(&event.source_vertex()))
    );
    let evidence = bubble_source
        .straight_skeleton(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(
        evidence.stage(),
        StraightSkeletonStage2::Complete,
        "{evidence:?}"
    );
    assert!(evidence.event_count() >= 3);
    let skeleton = evidence.skeleton().unwrap();
    assert!(skeleton.nodes().len() > bubble_source.segments().len());
    assert!(matches!(
        compare_reals(skeleton.maximum_time(), &r(1), &CurveContext::STRICT),
        Some(Ordering::Equal | Ordering::Greater)
    ));
    let bubble_node = skeleton
        .nodes()
        .iter()
        .position(|node| node.kind() == &StraightSkeletonNodeKind2::BubbleEvent { source_edge: 1 })
        .expect("bubble must materialize its local edge event");
    assert_eq!(
        skeleton
            .arcs()
            .iter()
            .filter(|arc| arc.end_node() == bubble_node)
            .count(),
        2
    );

    let left = Point2::new(r(-1), r(0));
    let right = Point2::new(r(1), r(0));
    let upper_left = Point2::new(r(-20), r(2));
    let upper_right = Point2::new(r(20), r(2));
    let clockwise = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(left.clone(), upper_left.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(upper_left, upper_right.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(upper_right, right.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(right, left, Point2::new(r(0), r(0)), true).unwrap(),
        ),
    ])
    .unwrap();
    let evidence = clockwise.straight_skeleton(&CurveContext::STRICT).unwrap();
    assert_eq!(
        evidence.stage(),
        StraightSkeletonStage2::Complete,
        "{evidence:?}"
    );
    assert!(
        evidence.skeleton().unwrap().nodes().iter().any(|node| {
            node.kind() == &StraightSkeletonNodeKind2::BubbleEvent { source_edge: 3 }
        })
    );
}

#[test]
fn local_arc_queue_rejects_an_extraneous_cone_sheet_root() {
    let left = Point2::new(r(-1), r(0));
    let right = Point2::new(r(1), r(0));
    let upper_left = Point2::new(r(-3), r(1));
    let upper_right = Point2::new(r(3), r(2));
    let contour = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(upper_left.clone(), left.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(left, right.clone(), Point2::new(r(0), r(0)), false)
                .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(right, upper_right.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(upper_right, upper_left).unwrap()),
    ])
    .unwrap();
    let Classification::Decided(events) = contour
        .straight_skeleton_local_arc_events(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("branch validation must be decided");
    };
    assert!(events.is_empty());
}

#[test]
fn reflex_line_arc_fixture_has_an_exact_first_splice() {
    let p = Point2::new(r(3), r(0));
    let a = Point2::new(r(1), r(0));
    let b = Point2::new(r(0), r(1));
    let q = Point2::new(r(-4), r(0));
    let lower_left = Point2::new(r(-4), r(-3));
    let lower_right = Point2::new(r(3), r(-3));
    let source = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p.clone(), a.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(a, b.clone(), Point2::new(r(0), r(0)), false).unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(b, q.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(q, lower_left.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(lower_left, lower_right.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(lower_right, p).unwrap()),
    ])
    .unwrap();
    assert_eq!(
        real_sign(
            &source.signed_area().unwrap().unwrap(),
            &CurveContext::STRICT
        ),
        Some(RealSign::Positive)
    );
    assert_eq!(
        source.has_self_contacts(&CurveContext::STRICT).unwrap(),
        Classification::Decided(false)
    );
    let Classification::Decided(splices) = source
        .straight_skeleton_splice_events(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("splice fixture must be decided");
    };
    assert_eq!(splices.len(), 1, "{splices:?}");
    assert_eq!(splices[0].source_vertex(), 1);
    assert_eq!(splices[0].left_source_edge(), 0);
    assert_eq!(splices[0].right_source_edge(), 1);
    assert_eq!(splices[0].time(), &(r(1) / r(2)).unwrap());
    assert_eq!(
        splices[0].point(),
        &Point2::new(r(0), -(r(1) / r(2)).unwrap())
    );
    let (mut records, mut nodes, active) =
        initial_shape_preserving_state(&source, RealSign::Positive);
    let mut arcs = Vec::new();
    let mut cycles = vec![active];
    let completion = complete_shape_preserving_edge_cycles(
        &mut nodes,
        &mut arcs,
        &mut records,
        &mut cycles,
        &Real::zero(),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert!(cycles.iter().all(|cycle| cycle.supports.is_empty()));
    assert_eq!(completion.events.len(), 7);
    assert!(nodes.iter().any(|node| matches!(
        node.kind(),
        StraightSkeletonNodeKind2::SpliceEvent {
            source_vertex: 1,
            left_source_edge: 0,
            right_source_edge: 1,
        }
    )));
    assert!(records.iter().any(|record| matches!(
        record.provenance,
        StraightSkeletonSupportProvenance2::SpliceArc {
            left_source_edge: 0,
            right_source_edge: 1,
            ..
        }
    )));
    assert!(nodes.iter().any(|node| matches!(
        node.kind(),
        StraightSkeletonNodeKind2::SqueezeEvent {
            first: StraightSkeletonSupportProvenance2::SpliceArc {
                left_source_edge: 0,
                right_source_edge: 1,
                ..
            },
            second: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 2 },
        }
    )));
    assert!(nodes.iter().any(|node| matches!(
        node.kind(),
        StraightSkeletonNodeKind2::SupportEvent { collapsed_supports }
            if collapsed_supports.len() == 3
    )));
    let overlap_node = nodes
        .iter()
        .position(|node| {
            node.time() == &(r(3) / r(2)).unwrap()
                && node.kind()
                    == &StraightSkeletonNodeKind2::EdgeEvent {
                        collapsed_source_edges: vec![0],
                    }
        })
        .expect("coincident anti-parallel overlap must terminate exactly");
    assert!(arcs.iter().any(|arc| arc.end_node() == overlap_node
        && arc.kind() == &StraightSkeletonArcKind2::TerminalRidge));
    let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
    assert_eq!(
        evidence.stage(),
        StraightSkeletonStage2::Complete,
        "{evidence:?}"
    );
    assert!(evidence.skeleton().is_some());
}

#[test]
fn splice_topology_inserts_an_exact_expanding_semicircle_support() {
    for orientation in [RealSign::Positive, RealSign::Negative] {
        let geometries = [
            ShapePreservingSupport2::Line {
                normal_x: r(0),
                normal_y: r(1),
                constant: r(0),
            },
            ShapePreservingSupport2::Line {
                normal_x: r(-1),
                normal_y: r(0),
                constant: r(-2),
            },
            ShapePreservingSupport2::Line {
                normal_x: r(0),
                normal_y: r(-1),
                constant: r(-2),
            },
            ShapePreservingSupport2::Line {
                normal_x: r(1),
                normal_y: r(0),
                constant: r(0),
            },
        ];
        let mut records = geometries
            .into_iter()
            .enumerate()
            .map(|(source_edge, geometry)| ShapePreservingSupportRecord2 {
                geometry,
                provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge },
            })
            .collect::<Vec<_>>();
        let mut active = ActiveShapePreservingCycle2 {
            supports: vec![0, 1, 2, 3],
            pair_start: BTreeMap::from([((3, 0), 0), ((0, 1), 1), ((1, 2), 2), ((2, 3), 3)]),
            pair_branch: BTreeMap::new(),
        };
        let mut nodes = [
            Point2::new(r(0), r(0)),
            Point2::new(r(2), r(0)),
            Point2::new(r(2), r(2)),
            Point2::new(r(0), r(2)),
        ]
        .into_iter()
        .enumerate()
        .map(|(source_vertex, point)| StraightSkeletonNode2 {
            point,
            time: Real::zero(),
            kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
        })
        .collect::<Vec<_>>();
        let mut arcs = Vec::new();
        let event = StraightSkeletonSpliceEvent2 {
            source_vertex: 0,
            left_source_edge: 3,
            right_source_edge: 0,
            time: r(2),
            point: Point2::new(r(2), r(2)),
        };
        let (splice_node, generated) = materialize_splice_topology_transition(
            &mut nodes,
            &mut arcs,
            &mut records,
            &mut active,
            &event,
            orientation,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap();
        assert_eq!(splice_node, 4);
        assert_eq!(generated, 4);
        assert_eq!(active.supports, vec![4, 0, 1, 2, 3]);
        assert!(!active.pair_start.contains_key(&(3, 0)));
        assert_eq!(active.pair_start[&(3, 4)], 4);
        assert_eq!(active.pair_start[&(4, 0)], 4);
        assert_eq!(
            records[generated].provenance,
            StraightSkeletonSupportProvenance2::SpliceArc {
                splice_node: 4,
                left_source_edge: 3,
                right_source_edge: 0,
            }
        );
        let ShapePreservingSupport2::Circle {
            center,
            signed_radius,
        } = &records[generated].geometry
        else {
            panic!("splice support must be circular");
        };
        assert_eq!(center, event.point());
        let orientation_scalar = if orientation == RealSign::Positive {
            r(1)
        } else {
            r(-1)
        };
        assert_eq!(signed_radius - &orientation_scalar * event.time(), r(0));
        let later_radius = signed_radius - &orientation_scalar * r(3);
        assert_eq!(later_radius, -orientation_scalar);
        assert_eq!(
            nodes[splice_node].kind(),
            &StraightSkeletonNodeKind2::SpliceEvent {
                source_vertex: 0,
                left_source_edge: 3,
                right_source_edge: 0,
            }
        );
        assert_eq!(arcs.len(), 1);
        assert_eq!(
            arcs[0].kind(),
            &StraightSkeletonArcKind2::VertexBisector {
                left_source_edge: 3,
                right_source_edge: 0,
            }
        );

        let left_end = nodes.len();
        nodes.push(StraightSkeletonNode2 {
            point: Point2::new(r(3), r(2)),
            time: r(3),
            kind: StraightSkeletonNodeKind2::EdgeEvent {
                collapsed_source_edges: vec![3],
            },
        });
        add_recorded_shape_preserving_arc(
            &mut arcs,
            &nodes,
            splice_node,
            left_end,
            (3, generated),
            &records,
            orientation,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap();
        let right_end = nodes.len();
        nodes.push(StraightSkeletonNode2 {
            point: Point2::new(r(2), r(3)),
            time: r(3),
            kind: StraightSkeletonNodeKind2::EdgeEvent {
                collapsed_source_edges: vec![0],
            },
        });
        add_recorded_shape_preserving_arc(
            &mut arcs,
            &nodes,
            splice_node,
            right_end,
            (generated, 0),
            &records,
            orientation,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap();
        assert_eq!(arcs.len(), 3);
        assert!(arcs[1..].iter().all(|arc| matches!(
            arc.kind(),
            StraightSkeletonArcKind2::GeneratedVertexBisector(_)
        )));
    }
}

#[test]
fn generated_support_can_schedule_and_materialize_a_later_splice() {
    let prior_provenance = StraightSkeletonSupportProvenance2::SpliceArc {
        splice_node: 0,
        left_source_edge: 0,
        right_source_edge: 1,
    };
    let mut records = vec![
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(0), r(0)),
                signed_radius: r(1),
            },
            provenance: prior_provenance.clone(),
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(5), r(0)),
                signed_radius: r(8),
            },
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 2 },
        },
    ];
    let mut nodes = vec![
        StraightSkeletonNode2 {
            point: Point2::new(r(0), r(0)),
            time: r(1),
            kind: StraightSkeletonNodeKind2::SpliceEvent {
                source_vertex: 1,
                left_source_edge: 0,
                right_source_edge: 1,
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new((r(9) / r(5)).unwrap(), (r(12) / r(5)).unwrap()),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new((r(9) / r(5)).unwrap(), -(r(12) / r(5)).unwrap()),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
    ];
    let mut active = ActiveShapePreservingCycle2 {
        supports: vec![0, 1],
        pair_start: BTreeMap::from([((0, 1), 1), ((1, 0), 2)]),
        pair_branch: BTreeMap::new(),
    };
    let event = next_shape_preserving_cycle_splice_before_or_at(
        &active,
        &records,
        &nodes,
        &r(4),
        &r(8),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap()
    .into_iter()
    .next()
    .expect("the generated/source reflex branch must reach its exact tangency");
    assert_eq!(event.time, r(7));
    assert!(event.left_support == 0 || event.right_support == 0);

    let left = records[event.left_support].provenance.clone();
    let right = records[event.right_support].provenance.clone();
    let mut arcs = Vec::new();
    let (event_node, generated) = materialize_recorded_splice_topology_transition(
        &mut nodes,
        &mut arcs,
        &mut records,
        &mut active,
        &event,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(
        nodes[event_node].kind(),
        &StraightSkeletonNodeKind2::SupportSpliceEvent {
            left: left.clone(),
            right: right.clone(),
        }
    );
    assert_eq!(
        records[generated].provenance,
        StraightSkeletonSupportProvenance2::SupportSpliceArc {
            splice_node: event_node,
            left: Box::new(left),
            right: Box::new(right),
        }
    );
    assert!(active.supports.contains(&generated));
    assert_eq!(arcs.len(), 1);
    assert!(matches!(
        arcs[0].kind(),
        StraightSkeletonArcKind2::GeneratedVertexBisector(_)
    ));
}

#[test]
fn coincident_independent_splice_and_edge_collapse_share_one_graph_node() {
    let fifth = r(5);
    let records = vec![
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(0), r(0)),
                signed_radius: r(1),
            },
            provenance: StraightSkeletonSupportProvenance2::SpliceArc {
                splice_node: 0,
                left_source_edge: 0,
                right_source_edge: 1,
            },
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(5), r(0)),
                signed_radius: r(8),
            },
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 2 },
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Line {
                normal_x: r(1),
                normal_y: r(0),
                constant: r(-1),
            },
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 3 },
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Line {
                normal_x: r(0),
                normal_y: r(1),
                constant: r(-7),
            },
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 4 },
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Line {
                normal_x: -(r(3) / &fifth).unwrap(),
                normal_y: -(r(4) / fifth).unwrap(),
                constant: (r(-53) / r(5)).unwrap(),
            },
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 5 },
        },
    ];
    let mut records = records;
    let mut nodes = vec![
        StraightSkeletonNode2 {
            point: Point2::new(r(0), r(0)),
            time: r(1),
            kind: StraightSkeletonNodeKind2::SpliceEvent {
                source_vertex: 1,
                left_source_edge: 0,
                right_source_edge: 1,
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new((r(9) / r(5)).unwrap(), (r(12) / r(5)).unwrap()),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new(r(10), r(10)),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new(r(3), r(-3)),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new(r(15), r(-3)),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new(r(-10), r(-10)),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
    ];
    let cycle = ActiveShapePreservingCycle2 {
        supports: vec![0, 1, 2, 3, 4],
        pair_start: BTreeMap::from([
            ((4, 0), 5),
            ((0, 1), 1),
            ((1, 2), 2),
            ((2, 3), 3),
            ((3, 4), 4),
        ]),
        pair_branch: BTreeMap::new(),
    };
    let topology = vec![RecordedTopologyEvent2::Splice(RecordedSpliceEvent2 {
        left_support: 0,
        right_support: 1,
        time: r(7),
        point: Point2::new(r(6), r(0)),
    })];
    let collapsing = vec![EdgeEventCandidate2 {
        active_index: 3,
        time: r(7),
        point: Point2::new(r(6), r(0)),
    }];
    let mut arcs = Vec::new();
    let cycles = apply_independent_shape_preserving_events(
        &mut nodes,
        &mut arcs,
        &mut records,
        cycle,
        &topology,
        &collapsing,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(cycles.len(), 1);
    assert_eq!(cycles[0].supports.len(), 5);
    assert!(!cycles[0].supports.contains(&3));
    let clustered = nodes
        .iter()
        .filter(|node| node.point() == &Point2::new(r(6), r(0)) && node.time() == &r(7))
        .collect::<Vec<_>>();
    assert_eq!(clustered.len(), 1);
    let StraightSkeletonNodeKind2::EventCluster { events } = clustered[0].kind() else {
        panic!("coincident events must share an explicit cluster node");
    };
    assert!(
        events
            .iter()
            .any(|event| matches!(event, StraightSkeletonNodeKind2::SupportSpliceEvent { .. }))
    );
    assert!(events.iter().any(|event| {
        event
            == &StraightSkeletonNodeKind2::EdgeEvent {
                collapsed_source_edges: vec![4],
            }
    }));
    assert_eq!(arcs.len(), 3);
}

#[test]
fn distinct_same_time_splices_can_share_a_surviving_support() {
    let mut records = vec![
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(0), r(0)),
                signed_radius: r(1),
            },
            provenance: StraightSkeletonSupportProvenance2::SpliceArc {
                splice_node: 0,
                left_source_edge: 0,
                right_source_edge: 1,
            },
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(5), r(0)),
                signed_radius: r(8),
            },
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 2 },
        },
        ShapePreservingSupportRecord2 {
            geometry: ShapePreservingSupport2::Circle {
                center: Point2::new(r(10), r(0)),
                signed_radius: r(1),
            },
            provenance: StraightSkeletonSupportProvenance2::SpliceArc {
                splice_node: 1,
                left_source_edge: 3,
                right_source_edge: 4,
            },
        },
    ];
    let mut nodes = vec![
        StraightSkeletonNode2 {
            point: Point2::new(r(0), r(0)),
            time: r(1),
            kind: StraightSkeletonNodeKind2::SpliceEvent {
                source_vertex: 1,
                left_source_edge: 0,
                right_source_edge: 1,
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new(r(10), r(0)),
            time: r(1),
            kind: StraightSkeletonNodeKind2::SpliceEvent {
                source_vertex: 4,
                left_source_edge: 3,
                right_source_edge: 4,
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new((r(9) / r(5)).unwrap(), (r(12) / r(5)).unwrap()),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new((r(41) / r(5)).unwrap(), (r(12) / r(5)).unwrap()),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
        StraightSkeletonNode2 {
            point: Point2::new(r(5), r(-10)),
            time: r(4),
            kind: StraightSkeletonNodeKind2::SupportEvent {
                collapsed_supports: Vec::new(),
            },
        },
    ];
    let cycle = ActiveShapePreservingCycle2 {
        supports: vec![0, 1, 2],
        pair_start: BTreeMap::from([((2, 0), 4), ((0, 1), 2), ((1, 2), 3)]),
        pair_branch: BTreeMap::new(),
    };
    let topology = vec![
        RecordedTopologyEvent2::Splice(RecordedSpliceEvent2 {
            left_support: 0,
            right_support: 1,
            time: r(7),
            point: Point2::new(r(6), r(0)),
        }),
        RecordedTopologyEvent2::Splice(RecordedSpliceEvent2 {
            left_support: 1,
            right_support: 2,
            time: r(7),
            point: Point2::new(r(4), r(0)),
        }),
    ];
    let mut arcs = Vec::new();
    let cycles = apply_independent_shape_preserving_events(
        &mut nodes,
        &mut arcs,
        &mut records,
        cycle,
        &topology,
        &[],
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(cycles.len(), 1);
    assert_eq!(cycles[0].supports.len(), 5);
    assert_eq!(arcs.len(), 2);
    assert_eq!(
        nodes
            .iter()
            .filter(|node| matches!(
                node.kind(),
                StraightSkeletonNodeKind2::SupportSpliceEvent { .. }
            ))
            .count(),
        2
    );
}

#[test]
fn generated_support_global_topology_retains_exact_provenance() {
    let geometries = [
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(1),
            constant: r(0),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(-1),
            normal_y: r(0),
            constant: r(-2),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(-1),
            constant: r(-2),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(1),
            normal_y: r(0),
            constant: r(0),
        },
    ];
    let mut records = geometries
        .into_iter()
        .enumerate()
        .map(|(source_edge, geometry)| ShapePreservingSupportRecord2 {
            geometry,
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge },
        })
        .collect::<Vec<_>>();
    let mut active = ActiveShapePreservingCycle2 {
        supports: vec![0, 1, 2, 3],
        pair_start: BTreeMap::from([((3, 0), 0), ((0, 1), 1), ((1, 2), 2), ((2, 3), 3)]),
        pair_branch: BTreeMap::new(),
    };
    let mut nodes = [
        Point2::new(r(0), r(0)),
        Point2::new(r(2), r(0)),
        Point2::new(r(2), r(2)),
        Point2::new(r(0), r(2)),
    ]
    .into_iter()
    .enumerate()
    .map(|(source_vertex, point)| StraightSkeletonNode2 {
        point,
        time: Real::zero(),
        kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
    })
    .collect::<Vec<_>>();
    let mut arcs = Vec::new();
    let splice = StraightSkeletonSpliceEvent2 {
        source_vertex: 0,
        left_source_edge: 3,
        right_source_edge: 0,
        time: r(2),
        point: Point2::new(r(2), r(2)),
    };
    let (splice_node, generated) = materialize_splice_topology_transition(
        &mut nodes,
        &mut arcs,
        &mut records,
        &mut active,
        &splice,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    let generated_provenance = StraightSkeletonSupportProvenance2::SpliceArc {
        splice_node,
        left_source_edge: 3,
        right_source_edge: 0,
    };

    let mut split_nodes = nodes.clone();
    let mut split_arcs = arcs.clone();
    let split = RecordedSplitEvent2 {
        left_support: 0,
        right_support: 1,
        hit_support: generated,
        time: r(3),
        point: Point2::new(r(1), r(1)),
    };
    let (split_node, split_cycles) = materialize_recorded_shape_preserving_split_transition(
        &mut split_nodes,
        &mut split_arcs,
        &records,
        &active,
        &split,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(
        split_nodes[split_node].kind(),
        &StraightSkeletonNodeKind2::SupportSplitEvent {
            left: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 0 },
            right: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 1 },
            hit: generated_provenance.clone(),
        }
    );
    assert!(
        split_cycles
            .iter()
            .all(|cycle| cycle.supports.contains(&generated))
    );
}

#[test]
fn generated_splice_support_survives_and_materializes_a_later_edge_collapse() {
    let geometries = [
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(1),
            constant: r(0),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(-1),
            normal_y: r(0),
            constant: r(-6),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(-1),
            constant: r(-10),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(1),
            normal_y: r(0),
            constant: r(0),
        },
    ];
    let mut records = geometries
        .into_iter()
        .enumerate()
        .map(|(source_edge, geometry)| ShapePreservingSupportRecord2 {
            geometry,
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge },
        })
        .collect::<Vec<_>>();
    let source_points = [
        Point2::new(r(0), r(0)),
        Point2::new(r(6), r(0)),
        Point2::new(r(6), r(10)),
        Point2::new(r(0), r(10)),
    ];
    let mut nodes = source_points
        .into_iter()
        .enumerate()
        .map(|(source_vertex, point)| StraightSkeletonNode2 {
            point,
            time: Real::zero(),
            kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
        })
        .collect::<Vec<_>>();
    let mut arcs = Vec::new();
    let mut active = ActiveShapePreservingCycle2 {
        supports: vec![0, 1, 2, 3],
        pair_start: BTreeMap::from([((3, 0), 0), ((0, 1), 1), ((1, 2), 2), ((2, 3), 3)]),
        pair_branch: BTreeMap::new(),
    };
    let splice = StraightSkeletonSpliceEvent2 {
        source_vertex: 0,
        left_source_edge: 3,
        right_source_edge: 0,
        time: r(2),
        point: Point2::new(r(2), r(2)),
    };
    let (splice_node, generated) = materialize_splice_topology_transition(
        &mut nodes,
        &mut arcs,
        &mut records,
        &mut active,
        &splice,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    let bottom_position = active
        .supports
        .iter()
        .position(|support| *support == 0)
        .unwrap();
    let candidate = shape_preserving_edge_event_candidate(
        &active,
        &records,
        &nodes,
        bottom_position,
        splice.time(),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap()
    .unwrap();
    assert_eq!(candidate.time, r(4));
    assert_eq!(candidate.point, Point2::new(r(2), r(4)));

    let advance = apply_shape_preserving_edge_collapses(
        &mut nodes,
        &mut arcs,
        &records,
        &mut active,
        &[candidate],
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(advance.time, r(4));
    assert_eq!(advance.collapsed_supports, vec![0]);
    assert_eq!(advance.event_nodes.len(), 1);
    let event_node = advance.event_nodes[0];
    assert_eq!(
        nodes[event_node].kind(),
        &StraightSkeletonNodeKind2::EdgeEvent {
            collapsed_source_edges: vec![0],
        }
    );
    assert_eq!(active.supports, vec![generated, 1, 2, 3]);
    assert_eq!(active.pair_start[&(generated, 1)], event_node);
    assert_eq!(arcs.len(), 3);
    assert!(arcs.iter().any(|arc| {
        arc.start_node() == splice_node
            && arc.end_node() == event_node
            && matches!(
                arc.kind(),
                StraightSkeletonArcKind2::GeneratedVertexBisector(_)
            )
    }));
    assert!(matches!(
        arcs.iter()
            .find(|arc| arc.start_node() == splice_node && arc.end_node() == event_node)
            .unwrap()
            .geometry(),
        StraightSkeletonArcGeometry2::ConicBranch(branch)
            if branch.kind() == StraightSkeletonTrajectoryKind2::Parabolic
    ));
}

#[test]
fn recorded_support_scheduler_groups_the_exact_next_event() {
    let geometries = [
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(1),
            constant: r(0),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(-1),
            normal_y: r(0),
            constant: r(-2),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(-1),
            constant: r(-2),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(1),
            normal_y: r(0),
            constant: r(0),
        },
    ];
    let records = geometries
        .into_iter()
        .enumerate()
        .map(|(source_edge, geometry)| ShapePreservingSupportRecord2 {
            geometry,
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge },
        })
        .collect::<Vec<_>>();
    let mut nodes = [
        Point2::new(r(0), r(0)),
        Point2::new(r(2), r(0)),
        Point2::new(r(2), r(2)),
        Point2::new(r(0), r(2)),
    ]
    .into_iter()
    .enumerate()
    .map(|(source_vertex, point)| StraightSkeletonNode2 {
        point,
        time: Real::zero(),
        kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
    })
    .collect::<Vec<_>>();
    let mut arcs = Vec::new();
    let mut active = ActiveShapePreservingCycle2 {
        supports: vec![0, 1, 2, 3],
        pair_start: BTreeMap::from([((3, 0), 0), ((0, 1), 1), ((1, 2), 2), ((2, 3), 3)]),
        pair_branch: BTreeMap::new(),
    };
    let half = (r(1) / r(2)).unwrap();
    let endpoints = active_shape_preserving_edge_endpoints_at_time(
        &active,
        &records,
        &nodes,
        0,
        &half,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap()
    .unwrap();
    assert_eq!(endpoints.0, Point2::new(half.clone(), half.clone()));
    assert_eq!(endpoints.1, Point2::new(r(2) - &half, half.clone()));
    assert!(
        active_shape_preserving_edge_contains_point_at_time(
            &active,
            &records,
            &nodes,
            0,
            &Point2::new(r(1), half.clone()),
            &half,
            RealSign::Positive,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap()
    );
    assert!(
        !active_shape_preserving_edge_contains_point_at_time(
            &active,
            &records,
            &nodes,
            0,
            &Point2::new(r(3), half.clone()),
            &half,
            RealSign::Positive,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap()
    );
    let advance = advance_shape_preserving_cycle_to_next_edge_event(
        &mut nodes,
        &mut arcs,
        &records,
        &mut active,
        &Real::zero(),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(advance.time, r(1));
    assert_eq!(advance.collapsed_supports, vec![0, 1, 2, 3]);
    assert_eq!(advance.event_nodes.len(), 1);
    assert!(active.supports.is_empty());
    assert!(active.pair_start.is_empty());
    assert_eq!(
        nodes[advance.event_nodes[0]].point(),
        &Point2::new(r(1), r(1))
    );
    assert_eq!(arcs.len(), 4);
}

#[test]
fn convex_mixed_wavefront_processes_successive_native_vanish_events() {
    for clockwise in [false, true] {
        let right = Point2::new(r(2), r(0));
        let top = Point2::new(r(0), r(2));
        let left = Point2::new(r(-1), r(0));
        let bottom = Point2::new(r(0), r(-1));
        let points = if clockwise {
            vec![top.clone(), right.clone(), bottom, left, top.clone()]
        } else {
            vec![right.clone(), top.clone(), left, bottom, right.clone()]
        };
        let mut segments = vec![Segment2::Arc(
            CircularArc2::try_from_center(
                points[0].clone(),
                points[1].clone(),
                Point2::new(r(0), r(0)),
                clockwise,
            )
            .unwrap(),
        )];
        segments.extend((1..4).map(|index| {
            Segment2::Line(
                LineSeg2::try_new(points[index].clone(), points[index + 1].clone()).unwrap(),
            )
        }));
        let source = Contour2::try_new(segments).unwrap();
        let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{evidence:?}"
        );
        assert_eq!(evidence.event_count(), 2);
        let skeleton = evidence.skeleton().unwrap();
        assert_eq!(skeleton.source_edge_count(), 4);
        assert_eq!(skeleton.nodes().len(), 6);
        assert_eq!(skeleton.arcs().len(), 5);
        assert!(
            skeleton
                .arcs()
                .iter()
                .filter(|arc| matches!(
                    arc.geometry(),
                    StraightSkeletonArcGeometry2::ConicBranch(_)
                ))
                .count()
                >= 2
        );
    }
}

#[test]
fn cocircular_smooth_contour_has_exact_empty_shape_preserving_skeleton() {
    for clockwise in [false, true] {
        let right = Point2::new(r(1), r(0));
        let left = Point2::new(r(-1), r(0));
        let center = Point2::new(r(0), r(0));
        let (first_start, first_end) = if clockwise {
            (left.clone(), right.clone())
        } else {
            (right.clone(), left.clone())
        };
        let source = Contour2::try_new(vec![
            Segment2::Arc(
                CircularArc2::try_from_center(
                    first_start.clone(),
                    first_end.clone(),
                    center.clone(),
                    clockwise,
                )
                .unwrap(),
            ),
            Segment2::Arc(
                CircularArc2::try_from_center(first_end, first_start, center.clone(), clockwise)
                    .unwrap(),
            ),
        ])
        .unwrap();
        let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{evidence:?}"
        );
        assert_eq!(evidence.event_count(), 1);
        let skeleton = evidence.skeleton().unwrap();
        assert!(skeleton.nodes().is_empty());
        assert!(skeleton.arcs().is_empty());
        assert_eq!(skeleton.maximum_time(), &r(1));
    }
}

#[test]
fn circle_support_pairs_classify_exact_conic_families() {
    let policy = CurveContext::STRICT;
    let root_two = r(2).sqrt().unwrap();
    let opposite = shape_preserving_vertex_trajectory(
        0,
        0,
        1,
        Point2::new(r(0), r(1)),
        &ShapePreservingSupport2::Circle {
            center: Point2::new(r(-1), r(0)),
            signed_radius: root_two.clone(),
        },
        &ShapePreservingSupport2::Circle {
            center: Point2::new(r(1), r(0)),
            signed_radius: -root_two.clone(),
        },
        RealSign::Positive,
        &policy,
    )
    .unwrap();
    let Classification::Decided(opposite) = opposite else {
        panic!("opposite circle trajectory must be decided");
    };
    assert_eq!(opposite.kind(), StraightSkeletonTrajectoryKind2::Elliptic);
    let StraightSkeletonTrajectoryGeometry2::Conic(conic) = opposite.geometry() else {
        panic!("opposite circles must retain a conic");
    };
    assert_eq!(
        real_sign(&conic.evaluate(opposite.start()), &policy),
        Some(RealSign::Zero)
    );

    let equal = shape_preserving_vertex_trajectory(
        0,
        0,
        1,
        Point2::new(r(0), r(1)),
        &ShapePreservingSupport2::Circle {
            center: Point2::new(r(-1), r(0)),
            signed_radius: root_two.clone(),
        },
        &ShapePreservingSupport2::Circle {
            center: Point2::new(r(1), r(0)),
            signed_radius: root_two,
        },
        RealSign::Positive,
        &policy,
    )
    .unwrap();
    let Classification::Decided(equal) = equal else {
        panic!("equal circle trajectory must be decided");
    };
    assert_eq!(equal.kind(), StraightSkeletonTrajectoryKind2::Linear);
    assert!(equal.affine_time().is_none());

    let unequal = shape_preserving_vertex_trajectory(
        0,
        0,
        1,
        Point2::new(r(2), r(0)),
        &ShapePreservingSupport2::Circle {
            center: Point2::new(r(0), r(0)),
            signed_radius: r(2),
        },
        &ShapePreservingSupport2::Circle {
            center: Point2::new(r(1), r(0)),
            signed_radius: r(1),
        },
        RealSign::Positive,
        &policy,
    )
    .unwrap();
    let Classification::Decided(unequal) = unequal else {
        panic!("unequal circle trajectory must be decided");
    };
    assert_eq!(unequal.kind(), StraightSkeletonTrajectoryKind2::Hyperbolic);
}

#[test]
fn exact_three_support_solver_handles_two_and_three_circle_events() {
    let policy = CurveContext::STRICT;
    let first_circle = ShapePreservingSupport2::Circle {
        center: Point2::new(r(-2), r(0)),
        signed_radius: r(3),
    };
    let second_circle = ShapePreservingSupport2::Circle {
        center: Point2::new(r(0), r(-3)),
        signed_radius: r(4),
    };
    let line = ShapePreservingSupport2::Line {
        normal_x: r(1),
        normal_y: r(0),
        constant: r(-1),
    };
    let third_circle = ShapePreservingSupport2::Circle {
        center: Point2::new(r(2), r(0)),
        signed_radius: r(3),
    };
    for supports in [
        vec![first_circle.clone(), second_circle.clone(), line],
        vec![first_circle, second_circle, third_circle],
    ] {
        let Ok(events) =
            three_support_events(&supports, RealSign::Positive, &Real::zero(), &policy).unwrap()
        else {
            panic!("support triple must have one certified future event");
        };
        assert!(!events.is_empty());
        assert_eq!(events[0], (r(1), Point2::new(r(0), r(0))));
    }
    let parallel_lines = vec![
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(-1),
            constant: r(0),
        },
        ShapePreservingSupport2::Circle {
            center: Point2::new(r(0), r(0)),
            signed_radius: r(4),
        },
        ShapePreservingSupport2::Line {
            normal_x: r(0),
            normal_y: r(1),
            constant: r(-3),
        },
    ];
    let events = three_support_events(&parallel_lines, RealSign::Positive, &Real::zero(), &policy)
        .unwrap()
        .unwrap();
    let time = (r(3) / r(2)).unwrap();
    assert_eq!(
        events,
        vec![
            (time.clone(), Point2::new(r(2), -(r(3) / r(2)).unwrap()),),
            (time, Point2::new(r(-2), -(r(3) / r(2)).unwrap())),
        ]
    );
}

#[test]
fn exact_curved_pair_solver_handles_line_and_circle_tangencies() {
    let policy = CurveContext::STRICT;
    let line = ShapePreservingSupport2::Line {
        normal_x: r(0),
        normal_y: r(-1),
        constant: r(0),
    };
    let circle = ShapePreservingSupport2::Circle {
        center: Point2::new(r(0), r(0)),
        signed_radius: r(1),
    };
    let Ok(Some((time, point))) = shape_preserving_pair_terminal_event(
        0,
        1,
        &line,
        &circle,
        RealSign::Positive,
        &Real::zero(),
        &policy,
    )
    .unwrap() else {
        panic!("line/circle pair must reach exact tangency");
    };
    let half = (r(1) / r(2)).unwrap();
    assert_eq!(time, half);
    assert_eq!(point, Point2::new(r(0), -half));

    for orientation in [RealSign::Positive, RealSign::Negative] {
        let signed_radius = if orientation == RealSign::Positive {
            r(2)
        } else {
            r(-2)
        };
        let first = ShapePreservingSupport2::Circle {
            center: Point2::new(r(-1), r(0)),
            signed_radius: signed_radius.clone(),
        };
        let second = ShapePreservingSupport2::Circle {
            center: Point2::new(r(1), r(0)),
            signed_radius,
        };
        let Ok(Some((time, point))) = shape_preserving_pair_terminal_event(
            0,
            1,
            &first,
            &second,
            orientation,
            &Real::zero(),
            &policy,
        )
        .unwrap() else {
            panic!("circle pair must reach exact external tangency");
        };
        assert_eq!(time, r(1));
        assert_eq!(point, Point2::new(r(0), r(0)));
        let tangencies =
            support_pair_future_tangencies(&first, &second, orientation, &Real::zero(), &policy)
                .unwrap()
                .unwrap();
        assert_eq!(tangencies, vec![(r(1), Point2::new(r(0), r(0)))]);
    }
}

#[test]
fn tracked_support_pairs_evaluate_exact_points_during_local_evolution() {
    let policy = CurveContext::STRICT;
    let horizontal = ShapePreservingSupport2::Line {
        normal_x: r(0),
        normal_y: r(1),
        constant: r(0),
    };
    let vertical = ShapePreservingSupport2::Line {
        normal_x: r(-1),
        normal_y: r(0),
        constant: r(-10),
    };
    let Ok(Some(line_point)) = tracked_support_pair_point_at_time(
        &horizontal,
        &vertical,
        &Point2::new(r(10), r(0)),
        &Real::zero(),
        &r(2),
        RealSign::Positive,
        None,
        &policy,
    )
    .unwrap() else {
        panic!("line pair must retain its unique moving intersection");
    };
    assert_eq!(line_point, Point2::new(r(8), r(2)));

    let circle = ShapePreservingSupport2::Circle {
        center: Point2::new(r(0), r(0)),
        signed_radius: r(5),
    };
    let Ok(Some(line_circle_point)) = tracked_support_pair_point_at_time(
        &horizontal,
        &circle,
        &Point2::new(r(5), r(0)),
        &Real::zero(),
        &r(1),
        RealSign::Positive,
        None,
        &policy,
    )
    .unwrap() else {
        panic!("line/circle pair must retain the selected parabola branch");
    };
    assert_eq!(line_circle_point.x(), &r(15).sqrt().unwrap());
    assert_eq!(line_circle_point.y(), &r(1));

    let second_circle = ShapePreservingSupport2::Circle {
        center: Point2::new(r(6), r(0)),
        signed_radius: r(5),
    };
    let Ok(Some(circle_circle_point)) = tracked_support_pair_point_at_time(
        &circle,
        &second_circle,
        &Point2::new(r(3), r(4)),
        &Real::zero(),
        &r(1),
        RealSign::Positive,
        None,
        &policy,
    )
    .unwrap() else {
        panic!("circle pair must retain the selected conic branch");
    };
    assert_eq!(circle_circle_point.x(), &r(3));
    assert_eq!(circle_circle_point.y(), &r(7).sqrt().unwrap());
}

#[test]
fn active_circular_edge_validation_uses_its_finite_evolved_sweep() {
    let origin = Point2::new(r(0), r(0));
    let right = Point2::new(r(1), r(0));
    let top = Point2::new(r(0), r(1));
    let source = Contour2::try_new(vec![
        Segment2::Arc(
            CircularArc2::try_from_center(right.clone(), top.clone(), origin.clone(), false)
                .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(top, origin.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(origin, right).unwrap()),
    ])
    .unwrap();
    let records = source
        .segments()
        .iter()
        .enumerate()
        .map(|(source_edge, segment)| ShapePreservingSupportRecord2 {
            geometry: shape_preserving_support(segment, RealSign::Positive).unwrap(),
            provenance: StraightSkeletonSupportProvenance2::SourceEdge { source_edge },
        })
        .collect::<Vec<_>>();
    let nodes = source
        .segments()
        .iter()
        .enumerate()
        .map(|(source_vertex, segment)| StraightSkeletonNode2 {
            point: segment.start().clone(),
            time: Real::zero(),
            kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
        })
        .collect::<Vec<_>>();
    let active = ActiveShapePreservingCycle2 {
        supports: vec![0, 1, 2],
        pair_start: BTreeMap::from([((2, 0), 0), ((0, 1), 1), ((1, 2), 2)]),
        pair_branch: BTreeMap::new(),
    };
    let time = (r(1) / r(4)).unwrap();
    let radial = ((r(3) * r(2).sqrt().unwrap()) / r(8)).unwrap();
    assert!(
        active_shape_preserving_edge_contains_point_at_time(
            &active,
            &records,
            &nodes,
            0,
            &Point2::new(radial.clone(), radial),
            &time,
            RealSign::Positive,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap()
    );
    assert!(
        !active_shape_preserving_edge_contains_point_at_time(
            &active,
            &records,
            &nodes,
            0,
            &Point2::new(-(r(3) / r(4)).unwrap(), r(0)),
            &time,
            RealSign::Positive,
            &CurveContext::STRICT,
        )
        .unwrap()
        .unwrap()
    );
}

#[test]
fn square_collapses_to_one_exact_center_event() {
    let evidence = contour(&[(0, 0), (2, 0), (2, 2), (0, 2)])
        .straight_skeleton(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.event_count(), 1);
    assert_eq!(evidence.simultaneous_event_count(), 1);
    let skeleton = evidence.skeleton().unwrap();
    assert_eq!(skeleton.nodes().len(), 5);
    assert_eq!(skeleton.arcs().len(), 4);
    let center = skeleton.nodes().last().unwrap();
    assert_eq!(center.point(), &Point2::new(r(1), r(1)));
    assert_eq!(center.time(), &r(1));
}

#[test]
fn rectangle_retains_the_terminal_ridge() {
    let evidence = contour(&[(0, 0), (4, 0), (4, 2), (0, 2)])
        .straight_skeleton(&CurveContext::STRICT)
        .unwrap();
    let skeleton = evidence.skeleton().unwrap();
    assert_eq!(skeleton.nodes().len(), 6);
    assert_eq!(skeleton.arcs().len(), 5);
    assert!(
        skeleton
            .arcs()
            .iter()
            .any(|arc| arc.kind() == &StraightSkeletonArcKind2::TerminalRidge)
    );
    assert_eq!(skeleton.maximum_time(), &r(1));
}

#[test]
fn clockwise_square_has_the_same_exact_collapse() {
    let evidence = contour(&[(0, 0), (0, 2), (2, 2), (2, 0)])
        .straight_skeleton(&CurveContext::STRICT)
        .unwrap();
    let skeleton = evidence.skeleton().unwrap();
    assert_eq!(
        skeleton.nodes().last().unwrap().point(),
        &Point2::new(r(1), r(1))
    );
}

#[test]
fn codirected_collinear_source_edges_are_normalized_exactly() {
    for points in [
        &[(0, 0), (1, 0), (2, 0), (2, 2), (0, 2)][..],
        &[(0, 2), (2, 2), (2, 0), (1, 0), (0, 0)][..],
    ] {
        let evidence = contour(points)
            .straight_skeleton(&CurveContext::STRICT)
            .unwrap();
        assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
        assert_eq!(evidence.source_edge_count(), 5);
        assert_eq!(evidence.skeleton().unwrap().nodes().len(), 5);
    }
}

#[test]
fn non_general_position_l_shape_materializes_terminal_vertex_event() {
    let evidence = contour(&[(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)])
        .straight_skeleton(&CurveContext::STRICT)
        .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.vertex_event_count(), 1);
    let skeleton = evidence.skeleton().unwrap();
    let two = r(2);
    let half = (r(1) / two).unwrap();
    let event = skeleton
        .nodes()
        .iter()
        .find(|node| matches!(node.kind(), StraightSkeletonNodeKind2::VertexEvent { .. }))
        .unwrap();
    assert_eq!(event.point(), &Point2::new(half.clone(), half.clone()));
    assert_eq!(event.time(), &half);
    assert_eq!(
        skeleton
            .arcs()
            .iter()
            .filter(|arc| arc.kind() == &StraightSkeletonArcKind2::TerminalRidge)
            .count(),
        2
    );
}

#[test]
fn general_position_concave_polygon_materializes_exact_split_topology() {
    let source = contour(&[
        (0, 0),
        (30, 0),
        (30, 24),
        (20, 24),
        (20, 7),
        (17, 11),
        (17, 24),
        (0, 24),
    ]);
    let global_contacts = source
        .straight_skeleton_global_contact_events(&CurveContext::STRICT)
        .unwrap();
    let Classification::Decided(global_contacts) = global_contacts else {
        panic!("line split candidates must be decided: {global_contacts:?}");
    };
    assert!(global_contacts.iter().any(|event| matches!(
        event.kind(),
        StraightSkeletonGlobalContactKind2::Split {
            left_source_edge: 3,
            right_source_edge: 4,
            hit_source_edge: 0,
            ..
        }
    )));
    let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.split_event_count(), 1);
    let skeleton = evidence.skeleton().unwrap();
    let four = r(4);
    let split_time = (r(7) / &four).unwrap();
    let split_x = (r(87) / four).unwrap();
    assert!(skeleton.nodes().iter().any(|node| {
        node.point() == &Point2::new(split_x.clone(), split_time.clone())
            && node.time() == &split_time
            && matches!(
                node.kind(),
                StraightSkeletonNodeKind2::SplitEvent {
                    left_source_edge: 3,
                    right_source_edge: 4,
                    hit_source_edge: 0,
                }
            )
    }));
    assert!(skeleton.arcs().iter().all(|arc| {
        arc.start_node() < skeleton.nodes().len()
            && arc.end_node() < skeleton.nodes().len()
            && arc.start_node() != arc.end_node()
    }));
}

#[test]
fn mixed_arc_polygon_split_is_validated_on_the_finite_evolved_edge() {
    let points = [
        Point2::new(r(0), r(0)),
        Point2::new(r(30), r(0)),
        Point2::new(r(30), r(24)),
        Point2::new(r(20), r(24)),
        Point2::new(r(20), r(7)),
        Point2::new(r(17), r(11)),
        Point2::new(r(17), r(24)),
        Point2::new(r(4), r(24)),
        Point2::new(r(0), r(20)),
    ];
    let mut segments = (0..7)
        .map(|index| {
            Segment2::Line(
                LineSeg2::try_new(points[index].clone(), points[index + 1].clone()).unwrap(),
            )
        })
        .collect::<Vec<_>>();
    segments.push(Segment2::Arc(
        CircularArc2::try_from_center(
            points[7].clone(),
            points[8].clone(),
            Point2::new(r(4), r(20)),
            false,
        )
        .unwrap(),
    ));
    segments.push(Segment2::Line(
        LineSeg2::try_new(points[8].clone(), points[0].clone()).unwrap(),
    ));
    let source = Contour2::try_new(segments).unwrap();
    let contacts = source
        .straight_skeleton_global_contact_events(&CurveContext::STRICT)
        .unwrap();
    let Classification::Decided(contacts) = contacts else {
        panic!("mixed split queue must be decided: {contacts:?}");
    };
    assert!(
        matches!(
            contacts
                .first()
                .map(StraightSkeletonGlobalContactEvent2::kind),
            Some(StraightSkeletonGlobalContactKind2::Split { .. })
        ),
        "{contacts:?}"
    );
    let split = contacts
        .iter()
        .find(|event| {
            event.kind()
                == &StraightSkeletonGlobalContactKind2::Split {
                    source_vertex: 4,
                    left_source_edge: 3,
                    right_source_edge: 4,
                    hit_source_edge: 0,
                }
        })
        .expect("the reflex vertex must hit the finite bottom edge");
    assert_eq!(split.time(), &(r(7) / r(4)).unwrap());
    assert_eq!(
        split.point(),
        &Point2::new((r(87) / r(4)).unwrap(), (r(7) / r(4)).unwrap())
    );
    let (mut records, mut nodes, active) =
        initial_shape_preserving_state(&source, RealSign::Positive);
    let mut arcs = Vec::new();
    let (event_node, cycles) = materialize_shape_preserving_split_transition(
        &mut nodes,
        &mut arcs,
        &records,
        &active,
        split,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(event_node, 9);
    assert_eq!(cycles[0].supports, vec![4, 5, 6, 7, 8, 0]);
    assert_eq!(cycles[1].supports, vec![0, 1, 2, 3]);
    assert_eq!(cycles[0].pair_start[&(0, 4)], event_node);
    assert_eq!(cycles[1].pair_start[&(3, 0)], event_node);
    assert_eq!(arcs.len(), 1);
    assert_eq!(arcs[0].start_node(), 4);
    assert_eq!(arcs[0].end_node(), event_node);
    let mut cycles = Vec::from(cycles);
    let completion = complete_shape_preserving_edge_cycles(
        &mut nodes,
        &mut arcs,
        &mut records,
        &mut cycles,
        split.time(),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap();
    let completion = completion.unwrap();
    assert!(!completion.events.is_empty());
    assert_eq!(completion.maximum_time, (r(17) / r(2)).unwrap());
    assert!(completion.events.iter().any(|(time, _)| time == &r(4)));
    let (mut scheduled_records, mut scheduled_nodes, active) =
        initial_shape_preserving_state(&source, RealSign::Positive);
    let mut scheduled_arcs = Vec::new();
    let mut scheduled_cycles = vec![active];
    let _scheduled = complete_shape_preserving_edge_cycles(
        &mut scheduled_nodes,
        &mut scheduled_arcs,
        &mut scheduled_records,
        &mut scheduled_cycles,
        &Real::zero(),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap();
    assert!(scheduled_nodes.iter().any(|node| matches!(
        node.kind(),
        StraightSkeletonNodeKind2::SplitEvent {
            left_source_edge: 3,
            right_source_edge: 4,
            hit_source_edge: 0,
        }
    )));
    let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
    assert_eq!(
        evidence.stage(),
        StraightSkeletonStage2::Complete,
        "{evidence:?}"
    );
    assert!(evidence.split_event_count() >= 1);
    assert!(evidence.skeleton().is_some());
}

#[test]
fn nonadjacent_line_and_arc_interiors_schedule_an_exact_squeeze() {
    let lower_left = Point2::new(r(0), r(0));
    let lower_right = Point2::new(r(30), r(0));
    let upper_right = Point2::new(r(30), r(10));
    let notch_right = Point2::new(r(16), r(10));
    let notch_left = Point2::new(r(14), r(10));
    let upper_left = Point2::new(r(0), r(10));
    let source = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(lower_left.clone(), lower_right.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(lower_right, upper_right.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(upper_right, notch_right.clone()).unwrap()),
        Segment2::Arc(
            CircularArc2::try_from_center(
                notch_right,
                notch_left.clone(),
                Point2::new(r(15), r(10)),
                true,
            )
            .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(notch_left, upper_left.clone()).unwrap()),
        Segment2::Line(LineSeg2::try_new(upper_left, lower_left).unwrap()),
    ])
    .unwrap();
    assert_eq!(
        source.has_self_contacts(&CurveContext::STRICT).unwrap(),
        Classification::Decided(false)
    );
    let contacts = source
        .straight_skeleton_global_contact_events(&CurveContext::STRICT)
        .unwrap();
    let Classification::Decided(contacts) = contacts else {
        panic!("squeeze queue must be decided: {contacts:?}");
    };
    let squeeze = contacts
        .iter()
        .find(|event| {
            event.kind()
                == &StraightSkeletonGlobalContactKind2::Squeeze {
                    first_source_edge: 0,
                    second_source_edge: 3,
                }
        })
        .expect("bottom line and expanding notch must squeeze");
    assert_eq!(squeeze.time(), &(r(9) / r(2)).unwrap());
    assert_eq!(squeeze.point(), &Point2::new(r(15), (r(9) / r(2)).unwrap()));
    let (records, mut nodes, active) = initial_shape_preserving_state(&source, RealSign::Positive);
    let (event_node, cycles) = materialize_shape_preserving_squeeze_transition(
        &mut nodes,
        &records,
        &active,
        squeeze,
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap()
    .unwrap();
    assert_eq!(event_node, 6);
    assert_eq!(cycles[0].supports, vec![0, 1, 2, 3]);
    assert_eq!(cycles[1].supports, vec![3, 4, 5, 0]);
    assert_eq!(cycles[0].pair_start[&(3, 0)], event_node);
    assert_eq!(cycles[1].pair_start[&(0, 3)], event_node);
    let forward_branch = cycles[0].pair_branch[&(3, 0)];
    let reverse_branch = cycles[1].pair_branch[&(0, 3)];
    assert_ne!(forward_branch, RealSign::Zero);
    assert_ne!(reverse_branch, RealSign::Zero);
    assert_ne!(forward_branch, reverse_branch);
    assert_eq!(
        nodes[event_node].kind(),
        &StraightSkeletonNodeKind2::SqueezeEvent {
            first: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 0 },
            second: StraightSkeletonSupportProvenance2::SourceEdge { source_edge: 3 },
        }
    );
    let (mut records, mut scheduled_nodes, active) =
        initial_shape_preserving_state(&source, RealSign::Positive);
    let mut scheduled_arcs = Vec::new();
    let mut scheduled_cycles = vec![active];
    let scheduled = complete_shape_preserving_edge_cycles(
        &mut scheduled_nodes,
        &mut scheduled_arcs,
        &mut records,
        &mut scheduled_cycles,
        &Real::zero(),
        RealSign::Positive,
        &CurveContext::STRICT,
    )
    .unwrap();
    assert!(scheduled.is_ok(), "{scheduled:?}");
    assert!(
        scheduled_nodes
            .iter()
            .any(|node| matches!(node.kind(), StraightSkeletonNodeKind2::SqueezeEvent { .. }))
    );
    let evidence = source.straight_skeleton(&CurveContext::STRICT).unwrap();
    assert_eq!(
        evidence.stage(),
        StraightSkeletonStage2::Complete,
        "{evidence:?}"
    );
}

#[test]
fn clockwise_general_position_concave_polygon_completes() {
    let evidence = contour(&[
        (0, 24),
        (17, 24),
        (17, 11),
        (20, 7),
        (20, 24),
        (30, 24),
        (30, 0),
        (0, 0),
    ])
    .straight_skeleton(&CurveContext::STRICT)
    .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.split_event_count(), 1);
    assert!(evidence.skeleton().is_some());
}

#[test]
fn non_general_position_line_fixtures_complete_exactly() {
    let fixtures: &[(&str, &[(i32, i32)])] = &[
        (
            "u",
            &[
                (0, 0),
                (6, 0),
                (6, 6),
                (4, 6),
                (4, 2),
                (2, 2),
                (2, 6),
                (0, 6),
            ],
        ),
        (
            "t",
            &[
                (0, 0),
                (6, 0),
                (6, 2),
                (4, 2),
                (4, 6),
                (2, 6),
                (2, 2),
                (0, 2),
            ],
        ),
        (
            "cross",
            &[
                (2, 0),
                (4, 0),
                (4, 2),
                (6, 2),
                (6, 4),
                (4, 4),
                (4, 6),
                (2, 6),
                (2, 4),
                (0, 4),
                (0, 2),
                (2, 2),
            ],
        ),
        (
            "asymmetric_u",
            &[
                (0, 0),
                (9, 0),
                (9, 7),
                (6, 7),
                (6, 2),
                (2, 2),
                (2, 5),
                (0, 5),
            ],
        ),
    ];
    for (name, points) in fixtures {
        let evidence = contour(points)
            .straight_skeleton(&CurveContext::STRICT)
            .unwrap();
        assert_eq!(
            evidence.stage(),
            StraightSkeletonStage2::Complete,
            "{name}: {:?}",
            evidence.blocker()
        );
        let skeleton = evidence.skeleton().unwrap();
        assert!(skeleton.arcs().iter().all(|arc| {
            arc.start_node() < skeleton.nodes().len()
                && arc.end_node() < skeleton.nodes().len()
                && arc.start_node() != arc.end_node()
        }));
    }
}

#[test]
fn nonterminal_bridge_collapse_splits_into_two_live_cycles() {
    let evidence = contour(&[
        (0, 0),
        (4, 0),
        (4, 1),
        (8, 1),
        (8, 0),
        (12, 0),
        (12, 4),
        (8, 4),
        (8, 3),
        (4, 3),
        (4, 4),
        (0, 4),
    ])
    .straight_skeleton(&CurveContext::STRICT)
    .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.vertex_event_count(), 2);
    assert_eq!(evidence.event_count(), 2);
    let skeleton = evidence.skeleton().unwrap();
    assert!(skeleton.arcs().iter().any(|arc| {
        arc.kind() == &StraightSkeletonArcKind2::TerminalRidge
            && skeleton.nodes()[arc.start_node()].time() == &r(1)
            && skeleton.nodes()[arc.end_node()].time() == &r(1)
    }));
}

#[test]
fn clockwise_nonterminal_bridge_collapse_completes() {
    let evidence = contour(&[
        (0, 4),
        (4, 4),
        (4, 3),
        (8, 3),
        (8, 4),
        (12, 4),
        (12, 0),
        (8, 0),
        (8, 1),
        (4, 1),
        (4, 0),
        (0, 0),
    ])
    .straight_skeleton(&CurveContext::STRICT)
    .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.vertex_event_count(), 2);
}

#[test]
fn same_point_nonterminal_multi_vertex_event_splits_four_live_cycles() {
    let evidence = contour(&[
        (4, 0),
        (8, 0),
        (8, 4),
        (7, 4),
        (7, 6),
        (9, 6),
        (9, 5),
        (13, 5),
        (13, 9),
        (9, 9),
        (9, 8),
        (7, 8),
        (7, 10),
        (8, 10),
        (8, 14),
        (4, 14),
        (4, 10),
        (5, 10),
        (5, 8),
        (3, 8),
        (3, 9),
        (-1, 9),
        (-1, 5),
        (3, 5),
        (3, 6),
        (5, 6),
        (5, 4),
        (4, 4),
    ])
    .straight_skeleton(&CurveContext::STRICT)
    .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
    assert_eq!(evidence.vertex_event_count(), 5);
    assert_eq!(evidence.event_count(), 2);
    let center = evidence
        .skeleton()
        .unwrap()
        .nodes()
        .iter()
        .find(|node| node.point() == &Point2::new(r(6), r(7)) && node.time() == &r(1))
        .unwrap();
    assert!(matches!(
        center.kind(),
        StraightSkeletonNodeKind2::VertexEvent {
            incident_source_edges,
            ..
        } if incident_source_edges.len() == 8
    ));
}
