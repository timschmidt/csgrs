use crate::CurveCertainty;
use crate::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveContext, CurveError, CurveRegion2,
    CurveString2, ExactCurveError, ExactCurveResult, FillRule, FiniteProjectionOptions, Real,
    RegionPointLocation, Segment2, SegmentKindCounts, UncertaintyReason,
    finite_polyline_vertex_centroid, finite_ring_signed_area, try_finite_polyline_vertex_centroid,
    try_finite_ring_signed_area,
};
use proptest::prelude::*;

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> crate::Point2 {
    crate::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(0))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmax, ymin),
        vertex(xmax, ymax),
        vertex(xmin, ymax),
    ])
    .unwrap()
}

fn reversed_rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmin, ymax),
        vertex(xmax, ymax),
        vertex(xmax, ymin),
    ])
    .unwrap()
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> crate::LineSeg2 {
    crate::LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap()
}

fn arc_bulge(start_x: i32, start_y: i32, end_x: i32, end_y: i32, bulge: i32) -> CircularArc2 {
    CircularArc2::from_bulge(p(start_x, start_y), p(end_x, end_y), s(bulge)).unwrap()
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn region(material: Vec<Contour2>, holes: Vec<Contour2>) -> CurveRegion2 {
    CurveRegion2::try_from_native_contours_with_policy(material, holes, &policy())
        .unwrap()
        .into_value()
}

fn classify(region: &CurveRegion2, point: &crate::Point2) -> Classification<RegionPointLocation> {
    region
        .classify_point_with_policy(&point.clone().into(), &policy())
        .unwrap()
        .into_value()
}

fn filled_area(region: &CurveRegion2) -> Classification<Option<Real>> {
    region
        .filled_area_with_policy(&policy())
        .unwrap()
        .into_value()
}

fn arrange_lines(
    segments: Vec<crate::LineSeg2>,
    fill_rule: FillRule,
) -> ExactCurveResult<CurveRegion2> {
    arrange_segments(
        segments.into_iter().map(Segment2::Line).collect(),
        fill_rule,
    )
}

fn arrange_segments(
    segments: Vec<Segment2>,
    fill_rule: FillRule,
) -> ExactCurveResult<CurveRegion2> {
    CurveRegion2::arrange_unordered_segments_with_policy(&segments, fill_rule, &policy())
        .map(|outcome| outcome.into_value())
}

fn assert_boundary_blocked(result: ExactCurveResult<CurveRegion2>) {
    let Err(ExactCurveError::Blocked(blocker)) = result else {
        panic!("an open endpoint graph must retain its boundary blocker");
    };
    assert_eq!(blocker.operation(), crate::CurveOperation2::Construction);
    assert_eq!(blocker.reason(), UncertaintyReason::Boundary);
}

#[test]
fn empty_region_classifies_everything_outside() {
    let region = CurveRegion2::empty();
    assert!(region.is_empty());
    assert_eq!(
        classify(&region, &p(0, 0)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn empty_native_boundary_input_constructs_an_exact_empty_region() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let outcome = CurveRegion2::try_from_native_boundary_contours_with_policy(
            &[],
            FillRule::EvenOdd,
            &policy,
        )
        .expect("empty boundary input represents the empty set");
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        let region = outcome.into_value();
        assert!(region.is_empty());
        assert_eq!(
            region
                .classify_point_with_policy(&p(0, 0).into(), &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Outside)
        );
    }
}

#[test]
fn material_contour_classifies_inside_outside_and_boundary() {
    let region = region(vec![rectangle(0, 0, 10, 10)], Vec::new());
    assert_eq!(
        classify(&region, &p(1, 1)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(&region, &p(11, 1)),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        classify(&region, &p(10, 5)),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn sparse_region_and_hole_classification_are_exact() {
    let region = region(
        vec![
            rectangle(0, 0, 10, 10),
            rectangle(20, 20, 24, 24),
            rectangle(40, 40, 44, 44),
        ],
        vec![rectangle(3, 3, 7, 7)],
    );
    assert_eq!(
        classify(&region, &p(21, 21)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(&region, &p(100, 100)),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        classify(&region, &p(20, 22)),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        classify(&region, &p(5, 5)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn material_island_inside_hole_restores_membership() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    for (point, expected_location) in [
        (p(1, 1), RegionPointLocation::Inside),
        (p(3, 3), RegionPointLocation::Outside),
        (p(5, 5), RegionPointLocation::Inside),
    ] {
        assert_eq!(
            classify(&region, &point),
            Classification::Decided(expected_location)
        );
    }
    assert_eq!(
        classify(&region, &p(2, 5)),
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn boundary_contour_fill_assigns_disjoint_nested_roles() {
    let region = CurveRegion2::try_from_native_boundary_contours_with_policy(
        &[rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7)],
        FillRule::EvenOdd,
        &policy(),
    )
    .unwrap()
    .into_value();
    assert_eq!(
        region
            .loop_role_counts_with_policy(&policy())
            .unwrap()
            .into_value(),
        Classification::Decided((1, 1))
    );
    assert_eq!(
        classify(&region, &p(1, 1)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(&region, &p(5, 5)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn boundary_contour_fill_regularizes_crossings_and_touches() {
    use crate::{BooleanOp, OffsetCornerStyle2};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill in [FillRule::EvenOdd, FillRule::NonZero] {
            for (contours, probes) in [
                (
                    vec![rectangle(0, 0, 4, 4), rectangle(2, -1, 6, 3)],
                    vec![
                        (p(1, 1), RegionPointLocation::Inside),
                        (
                            p(3, 1),
                            if fill == FillRule::EvenOdd {
                                RegionPointLocation::Outside
                            } else {
                                RegionPointLocation::Inside
                            },
                        ),
                        (p(5, 1), RegionPointLocation::Inside),
                        (p(7, 1), RegionPointLocation::Outside),
                    ],
                ),
                (
                    vec![rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
                    vec![
                        (p(4, 2), RegionPointLocation::Inside),
                        (p(8, 2), RegionPointLocation::Boundary),
                    ],
                ),
                (
                    vec![rectangle(0, 0, 4, 4), rectangle(4, 4, 8, 8)],
                    vec![
                        (p(4, 4), RegionPointLocation::Boundary),
                        (p(6, 6), RegionPointLocation::Inside),
                        (p(6, 2), RegionPointLocation::Outside),
                    ],
                ),
            ] {
                let outcome = CurveRegion2::try_from_native_boundary_contours_with_policy(
                    &contours, fill, &policy,
                )
                .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let region = outcome.into_value();
                let Classification::Decided(native) = region
                    .native_contours_fast_path_with_policy(&policy)
                    .unwrap()
                    .into_value()
                else {
                    panic!("line boundaries retain a native view");
                };
                let boundaries = native
                    .material_contours()
                    .iter()
                    .chain(native.hole_contours())
                    .cloned()
                    .collect::<Vec<_>>();
                let restored = CurveRegion2::try_from_native_boundary_contours_with_policy(
                    &boundaries,
                    fill,
                    &policy,
                )
                .unwrap()
                .into_value();
                for (point, expected) in probes {
                    for result in [&region, &restored] {
                        assert_eq!(
                            result
                                .classify_point_with_policy(&point.clone().into(), &policy)
                                .unwrap()
                                .into_value(),
                            Classification::Decided(expected)
                        );
                    }
                }
                assert!(
                    region
                        .boolean_region_with_policy(&restored, BooleanOp::Xor, &policy)
                        .unwrap()
                        .into_value()
                        .is_empty()
                );
            }
            let joined = CurveRegion2::try_from_native_boundary_contours_with_policy(
                &[rectangle(0, 0, 4, 4), rectangle(4, 0, 8, 4)],
                fill,
                &policy,
            )
            .unwrap()
            .into_value();
            let grown = joined
                .offset_with_policy(Real::one(), &OffsetCornerStyle2::Round, &policy)
                .unwrap()
                .into_value();
            for (point, expected) in [
                (p(4, 0), RegionPointLocation::Inside),
                (p(4, -1), RegionPointLocation::Boundary),
                (p(4, -2), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    grown
                        .classify_point_with_policy(&point.into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn native_boundary_global_fill_retains_winding_and_recursive_islands() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for contour_fill in [FillRule::EvenOdd, FillRule::NonZero] {
            let outer = rectangle(0, 0, 10, 10);
            let doubled = Contour2::try_new_with_fill_rule(
                outer
                    .segments()
                    .iter()
                    .chain(outer.segments())
                    .cloned()
                    .collect(),
                contour_fill,
            )
            .unwrap();
            for fill in [FillRule::EvenOdd, FillRule::NonZero] {
                let contours = [doubled.clone(), reversed_rectangle(2, 2, 8, 8)];
                let region = CurveRegion2::try_from_native_boundary_contours_with_policy(
                    &contours, fill, &policy,
                )
                .unwrap()
                .into_value();
                for (point, expected) in [
                    (
                        p(1, 1),
                        if fill == FillRule::EvenOdd {
                            RegionPointLocation::Outside
                        } else {
                            RegionPointLocation::Inside
                        },
                    ),
                    (p(5, 5), RegionPointLocation::Inside),
                    (p(11, 5), RegionPointLocation::Outside),
                ] {
                    assert_eq!(
                        region
                            .classify_point_with_policy(&point.into(), &policy)
                            .unwrap()
                            .into_value(),
                        Classification::Decided(expected)
                    );
                }
                let opposite = Contour2::try_new(
                    outer
                        .segments()
                        .iter()
                        .rev()
                        .map(Segment2::reversed)
                        .collect(),
                )
                .unwrap();
                assert!(
                    CurveRegion2::try_from_native_boundary_contours_with_policy(
                        &[outer.clone(), opposite],
                        fill,
                        &policy
                    )
                    .unwrap()
                    .into_value()
                    .is_empty()
                );
            }
        }
        let mut nested = (0..5)
            .map(|i| rectangle(i * 2, i * 2, 20 - i * 2, 20 - i * 2))
            .collect::<Vec<_>>();
        for reverse_order in [false, true] {
            if reverse_order {
                nested.reverse();
            }
            let region = CurveRegion2::try_from_native_boundary_contours_with_policy(
                &nested,
                FillRule::EvenOdd,
                &policy,
            )
            .unwrap()
            .into_value();
            assert_eq!(
                region
                    .loop_role_counts_with_policy(&policy)
                    .unwrap()
                    .into_value(),
                Classification::Decided((3, 2))
            );
            for i in 0..5 {
                let expected = if i % 2 == 0 {
                    RegionPointLocation::Inside
                } else {
                    RegionPointLocation::Outside
                };
                assert_eq!(
                    region
                        .classify_point_with_policy(&p(2 * i + 1, 10).into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn native_boundary_circle_fills_reenter_exact_boolean_operations() {
    use crate::BooleanOp;
    let circle = |x| {
        Contour2::try_new(vec![
            Segment2::Arc(arc_bulge(x - 2, 0, x + 2, 0, 1)),
            Segment2::Arc(arc_bulge(x + 2, 0, x - 2, 0, 1)),
        ])
        .unwrap()
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill in [FillRule::EvenOdd, FillRule::NonZero] {
            for separation in [2, 4] {
                let region = CurveRegion2::try_from_native_boundary_contours_with_policy(
                    &[circle(0), circle(separation)],
                    fill,
                    &policy,
                )
                .unwrap()
                .into_value();
                let expected = if separation == 4 {
                    RegionPointLocation::Boundary
                } else if fill == FillRule::NonZero {
                    RegionPointLocation::Inside
                } else {
                    RegionPointLocation::Outside
                };
                assert_eq!(
                    region
                        .classify_point_with_policy(&p(separation / 2, 0).into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(expected)
                );
                let restored = region
                    .boolean_region_with_policy(&region, BooleanOp::Intersection, &policy)
                    .unwrap()
                    .into_value();
                assert_eq!(
                    restored
                        .classify_point_with_policy(&p(separation / 2, 0).into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(expected)
                );
                assert!(
                    restored
                        .boolean_region_with_policy(&region, BooleanOp::Xor, &policy)
                        .unwrap()
                        .into_value()
                        .is_empty()
                );
            }
        }
    }
}

#[test]
fn unordered_lines_materialize_one_authoritative_region() {
    let built = arrange_lines(
        vec![
            line(0, 0, 4, 0),
            line(0, 4, 4, 4),
            line(0, 0, 0, 4),
            line(4, 0, 4, 4),
        ],
        FillRule::NonZero,
    )
    .unwrap();
    assert_eq!(built.len(), 1);
    assert_eq!(
        built
            .boundary_loops()
            .iter()
            .map(|loop_| loop_.len())
            .sum::<usize>(),
        4
    );
    let region = &built;
    assert_eq!(
        classify(region, &p(2, 2)),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unordered_open_lines_retain_a_boundary_blocker() {
    assert_boundary_blocked(arrange_lines(
        vec![line(0, 0, 1, 0), line(3, 0, 4, 0)],
        FillRule::NonZero,
    ));
}

#[test]
fn unordered_crossing_and_overlapping_lines_remain_explicit_blockers() {
    for lines in [
        vec![line(0, 0, 4, 4), line(0, 4, 4, 0)],
        vec![line(0, 0, 4, 0), line(2, 0, 6, 0)],
    ] {
        assert_boundary_blocked(arrange_lines(lines, FillRule::NonZero));
    }
}

#[test]
fn unordered_self_crossing_walk_uses_the_authoritative_curve_arrangement() {
    let source = vec![
        line(4, 4, 0, 4),
        line(4, 0, 0, 0),
        line(0, 4, 4, 0),
        line(0, 0, 4, 4),
    ];
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let built = arrange_lines(source.clone(), fill_rule).unwrap();
        assert_eq!(built.len(), 2);
        let region = &built;
        for (point, expected) in [
            (p(2, 3), RegionPointLocation::Inside),
            (p(2, 1), RegionPointLocation::Inside),
            (p(0, 2), RegionPointLocation::Outside),
        ] {
            assert_eq!(classify(region, &point), Classification::Decided(expected));
        }
    }
}

#[test]
fn unordered_crossing_walks_are_regularized_by_global_parity() {
    let source = vec![
        line(4, 4, 0, 4),
        line(0, 0, 4, 0),
        line(0, 4, 0, 0),
        line(4, 0, 4, 4),
        line(6, 3, 2, 3),
        line(2, -1, 6, -1),
        line(2, 3, 2, -1),
        line(6, -1, 6, 3),
    ];
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let built = arrange_lines(source.clone(), fill_rule).unwrap();
        let region = &built;
        for (point, expected) in [
            (p(1, 1), RegionPointLocation::Inside),
            (p(3, 1), RegionPointLocation::Outside),
            (p(5, 1), RegionPointLocation::Inside),
            (p(8, 1), RegionPointLocation::Outside),
        ] {
            assert_eq!(classify(region, &point), Classification::Decided(expected));
        }
    }
}

#[test]
fn unordered_single_full_circle_is_a_closed_walk() {
    let start = p(2, 0);
    let circle = CircularArc2::try_from_center(start.clone(), start, p(0, 0), false).unwrap();
    let built = arrange_segments(vec![Segment2::Arc(circle)], FillRule::NonZero).unwrap();
    let region = &built;
    assert_eq!(built.len(), 1);
    assert_eq!(
        classify(region, &p(0, 0)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        classify(region, &p(3, 0)),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unordered_line_arc_segments_recover_the_exact_native_view() {
    let built = arrange_segments(
        vec![
            Segment2::Line(line(4, 0, 0, 0)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
    )
    .unwrap();
    let region = &built;
    assert_eq!(
        classify(region, &p(2, -1)),
        Classification::Decided(RegionPointLocation::Inside)
    );
    let Classification::Decided(facts) = region
        .structural_facts_with_policy(&policy())
        .unwrap()
        .into_value()
    else {
        panic!("the exact line and circular spans expose native facts");
    };
    assert_eq!(facts.material_contour_count, 1);
    assert_eq!(facts.hole_contour_count, 0);
    assert_eq!(facts.segment_kinds, SegmentKindCounts { lines: 1, arcs: 2 });
}

#[test]
fn native_overlap_regularizes_empty_and_open_crossings_remain_blocked() {
    let coincident = arrange_segments(
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
        ],
        FillRule::NonZero,
    )
    .unwrap();
    assert!(coincident.is_empty());

    let cases = [
        vec![
            Segment2::Arc(arc_bulge(0, 0, 4, 0, 1)),
            Segment2::Line(line(2, -3, 2, 1)),
        ],
        vec![
            Segment2::Arc(
                CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap(),
            ),
            Segment2::Arc(CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap()),
        ],
    ];
    for segments in cases {
        assert_boundary_blocked(arrange_segments(segments, FillRule::NonZero));
    }
}

#[test]
fn contour_profiles_group_holes_with_their_exact_material_owner() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(20, 0, 30, 10)],
        vec![rectangle(2, 2, 4, 4), rectangle(22, 2, 24, 4)],
    );
    let profiles = region
        .boundary_profiles_with_policy(&policy())
        .unwrap()
        .into_value();
    let Classification::Decided(profiles) = profiles else {
        panic!("profile ownership should be decided: {profiles:?}");
    };
    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().len() == 1));
    assert_eq!(profiles[0].material_loop_index(), 0);
    assert_eq!(profiles[0].hole_loop_indices(), &[2]);
    assert_eq!(profiles[1].material_loop_index(), 1);
    assert_eq!(profiles[1].hole_loop_indices(), &[3]);
}

#[test]
fn contour_profiles_remove_holes_without_a_material_owner() {
    let region = region(Vec::new(), vec![rectangle(2, 2, 4, 4)]);
    assert_eq!(
        region
            .boundary_profiles_with_policy(&policy())
            .unwrap()
            .into_value(),
        Classification::Decided(Vec::new())
    );
}

#[test]
fn contour_projection_closes_finite_ring_without_owning_topology() {
    let ring = rectangle(0, 0, 10, 10)
        .project_to_finite_ring(&FiniteProjectionOptions::try_new(0.01).unwrap())
        .unwrap();
    assert!(ring.is_closed());
    assert_eq!(ring.points().first(), ring.points().last());
    assert_eq!(ring.points().len(), 5);
    assert_eq!(ring.try_signed_ring_area().unwrap(), 100.0);
    assert_eq!(finite_ring_signed_area(ring.points()), 100.0);
    assert_eq!(ring.try_vertex_centroid().unwrap(), Some([5.0, 5.0]));
}

#[test]
fn finite_projection_checked_measurements_reject_nonfinite_or_overflow() {
    assert_eq!(
        try_finite_ring_signed_area(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
    assert_eq!(
        try_finite_polyline_vertex_centroid(&[[0.0, 0.0], [f64::INFINITY, 1.0]]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
    assert_eq!(
        try_finite_ring_signed_area(&[[1.0e308, 0.0], [0.0, 1.0e308], [0.0, 0.0]]).unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
    assert!(finite_ring_signed_area(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]).is_nan());
    assert!(finite_polyline_vertex_centroid(&[[0.0, 0.0], [f64::INFINITY, 1.0]]).is_some());
}

#[test]
fn curve_string_projection_subdivides_arcs_and_keeps_exact_endpoints() {
    let start = crate::Point2::new(Real::one(), Real::zero());
    let end = crate::Point2::new(-Real::one(), Real::zero());
    let center = crate::Point2::new(Real::zero(), Real::zero());
    let arc = CircularArc2::try_from_center(start, end.clone(), center, false).unwrap();
    let tail = crate::LineSeg2::try_new(end, p(-2, 0)).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc), Segment2::Line(tail)]).unwrap();
    let polyline = curve
        .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.05).unwrap())
        .unwrap();
    assert!(!polyline.is_closed());
    assert!(polyline.points().len() > 3);
    assert_eq!(polyline.points().first(), Some(&[1.0, 0.0]));
    assert_eq!(polyline.points().last(), Some(&[-2.0, 0.0]));
}

#[test]
fn curve_string_projection_rejects_nonfinite_arc_samples() {
    let huge = Real::try_from(1.1e308).unwrap();
    let arc = CircularArc2::try_from_center(
        crate::Point2::new(Real::zero(), Real::zero()),
        crate::Point2::new(huge.clone(), huge.clone()),
        crate::Point2::new(huge, Real::zero()),
        false,
    )
    .unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc)]).unwrap();
    assert_eq!(
        curve
            .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.01).unwrap())
            .unwrap_err(),
        CurveError::NonFiniteProjectionPoint
    );
}

#[test]
fn unified_finite_profiles_preserve_material_hole_bins_and_ownership() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(20, 0, 30, 10)],
        vec![rectangle(2, 2, 4, 4), rectangle(22, 2, 24, 4)],
    );
    let profiles = region
        .project_to_finite_profiles_exact_with_policy(
            &FiniteProjectionOptions::try_new(0.01).unwrap(),
            &policy(),
        )
        .unwrap()
        .into_value();
    let Classification::Decided(profiles) = profiles else {
        panic!("finite profile ownership should be decided: {profiles:?}");
    };
    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().len() == 1));
    assert_eq!(profiles[0].material().points()[0], [0.0, 0.0]);
    assert!(profiles[0].holes()[0].points().contains(&[2.0, 2.0]));
    assert_eq!(profiles[0].holes()[0].try_signed_ring_area().unwrap(), -4.0);
    assert_eq!(profiles[1].material().points()[0], [20.0, 0.0]);
    assert!(profiles[1].holes()[0].points().contains(&[22.0, 2.0]));
    assert_eq!(profiles[1].holes()[0].try_signed_ring_area().unwrap(), -4.0);
    assert_eq!(profiles[0].try_projected_filled_area().unwrap(), 96.0);
    assert_eq!(profiles[1].try_projected_filled_area().unwrap(), 96.0);
}

#[test]
fn similarity_transform_preserves_arcs_and_reflection_flips_orientation() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(arc)]).unwrap();
    let transform =
        crate::Similarity2::try_from_f64_affine(0.0, -1.0, 1.0, 0.0, 3.0, -2.0, 1e-9).unwrap();
    let transformed = curve.transform_similarity(&transform).unwrap();
    let [Segment2::Arc(transformed_arc)] = transformed.segments() else {
        panic!("similarity should preserve an arc");
    };
    assert_eq!(transformed_arc.start(), &crate::Point2::from_values(3, -1));
    assert_eq!(transformed_arc.end(), &crate::Point2::from_values(2, -2));
    assert!(!transformed_arc.is_clockwise());

    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(1, 0), Real::one()),
        BulgeVertex2::new(p(-1, 0), Real::zero()),
    ])
    .unwrap();
    let reflection =
        crate::Similarity2::try_from_f64_affine(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1e-9).unwrap();
    let transformed = contour.transform_similarity(&reflection).unwrap();
    let Segment2::Arc(arc) = &transformed.segments()[0] else {
        panic!("reflection should retain an arc");
    };
    assert!(arc.is_clockwise());
    assert!(reflection.reverses_orientation());
    assert_eq!(
        crate::Similarity2::try_from_f64_affine(1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1e-9),
        Err(CurveError::InvalidSimilarityTransform)
    );
}

#[test]
fn filled_area_uses_roles_not_orientation_and_counts_nested_islands() {
    let simple = region(
        vec![reversed_rectangle(0, 0, 10, 10)],
        vec![rectangle(3, 3, 7, 7)],
    );
    assert_eq!(
        filled_area(&simple),
        Classification::Decided(Some(Real::from(84_i8)))
    );
    let nested = region(
        vec![rectangle(0, 0, 10, 10), reversed_rectangle(4, 4, 6, 6)],
        vec![reversed_rectangle(2, 2, 8, 8)],
    );
    assert_eq!(
        filled_area(&nested),
        Classification::Decided(Some(Real::from(68_i8)))
    );
}

#[test]
fn filled_area_is_exact_for_center_defined_circle() {
    let top = CircularArc2::try_from_center(p(1, 0), p(-1, 0), p(0, 0), false).unwrap();
    let bottom = CircularArc2::try_from_center(p(-1, 0), p(1, 0), p(0, 0), false).unwrap();
    let contour = Contour2::try_new(vec![Segment2::Arc(top), Segment2::Arc(bottom)]).unwrap();
    assert_eq!(
        filled_area(&region(vec![contour], Vec::new())),
        Classification::Decided(Some(Real::pi()))
    );
}

#[test]
fn strict_and_approximate_512_share_the_unified_policy_terminal() {
    let region = region(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    for context in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_eq!(
            region
                .classify_point_with_policy(&p(1, 1).into(), &context)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            region
                .filled_area_with_policy(&context)
                .unwrap()
                .into_value(),
            Classification::Decided(Some(Real::from(84_i8)))
        );
    }
}

#[test]
fn unordered_native_arrangement_obeys_the_approximate_512_terminal() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
    let first_sum = sine;
    let second_sum = first_sum.clone() + unresolved_zero;
    let lines = vec![
        crate::LineSeg2::try_new(
            crate::Point2::new(Real::zero(), Real::zero()),
            crate::Point2::new(first_sum.clone(), Real::zero()),
        )
        .unwrap(),
        crate::LineSeg2::try_new(
            crate::Point2::new(second_sum.clone(), Real::zero()),
            crate::Point2::new(first_sum.clone(), Real::one()),
        )
        .unwrap(),
        crate::LineSeg2::try_new(
            crate::Point2::new(second_sum, Real::one()),
            crate::Point2::new(Real::zero(), Real::one()),
        )
        .unwrap(),
        crate::LineSeg2::try_new(
            crate::Point2::new(Real::zero(), Real::one()),
            crate::Point2::new(Real::zero(), Real::zero()),
        )
        .unwrap(),
    ];

    let segments = lines.into_iter().map(Segment2::Line).collect::<Vec<_>>();
    let strict = CurveRegion2::arrange_unordered_segments_with_policy(
        &segments,
        FillRule::NonZero,
        &CurveContext::STRICT,
    );
    assert!(matches!(strict, Err(ExactCurveError::Blocked(_))));

    let approximate = CurveRegion2::arrange_unordered_segments_with_policy(
        &segments,
        FillRule::NonZero,
        &CurveContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(!approximate.value.is_empty());
}

proptest! {
    #[test]
    fn generated_unordered_line_rectangles_build_unified_regions(
        xmin in -50_i32..50,
        ymin in -50_i32..50,
        width in 2_i32..80,
        height in 2_i32..80,
        order_variant in 0_usize..4,
        reverse_mask in 0_u8..16,
    ) {
        let xmax = xmin + width;
        let ymax = ymin + height;
        let mut lines = vec![
            line(xmin, ymin, xmax, ymin),
            line(xmax, ymin, xmax, ymax),
            line(xmax, ymax, xmin, ymax),
            line(xmin, ymax, xmin, ymin),
        ];
        for (index, segment) in lines.iter_mut().enumerate() {
            if reverse_mask & (1 << index) != 0 {
                *segment = segment.reversed();
            }
        }
        match order_variant {
            0 => {}
            1 => lines.swap(0, 2),
            2 => lines.rotate_left(1),
            _ => lines.reverse(),
        }
        let built = arrange_lines(lines, FillRule::NonZero).unwrap();
                prop_assert_eq!(built.len(), 1);
        prop_assert_eq!(built.boundary_loops().iter().map(|loop_| loop_.len()).sum::<usize>(), 4);
        prop_assert_eq!(
            classify(&built, &p(xmin + 1, ymin + 1)),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }

    #[test]
    fn generated_unordered_line_arc_semicircles_build_unified_regions(
        xmin in -50_i32..50,
        ymin in -50_i32..50,
        width in 4_i32..80,
        bulge_sign in any::<bool>(),
        order_variant in 0_usize..2,
        reverse_mask in 0_u8..4,
    ) {
        let xmax = xmin + width;
        let bulge = if bulge_sign { 1 } else { -1 };
        let inside_y = if bulge_sign { ymin - 1 } else { ymin + 1 };
        let mut segments = vec![
            Segment2::Line(line(xmax, ymin, xmin, ymin)),
            Segment2::Arc(arc_bulge(xmin, ymin, xmax, ymin, bulge)),
        ];
        for (index, segment) in segments.iter_mut().enumerate() {
            if reverse_mask & (1 << index) != 0 {
                *segment = segment.reversed();
            }
        }
        if order_variant == 1 {
            segments.swap(0, 1);
        }
        let built = arrange_segments(segments, FillRule::NonZero).unwrap();
                prop_assert_eq!(
            built.structural_facts_with_policy(&policy()).unwrap().into_value().map(|facts| facts.segment_kinds),
            Classification::Decided(SegmentKindCounts { lines: 1, arcs: 2 })
        );
        prop_assert_eq!(
            classify(
                &built,
                &p(xmin + width / 2, inside_y),
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }

    #[test]
    fn generated_rectangle_hole_area_uses_roles_not_orientation(
        width in 3_i32..80,
        height in 3_i32..80,
        hole_width in 1_i32..20,
        hole_height in 1_i32..20,
    ) {
        let hole_width = hole_width.min(width - 2);
        let hole_height = hole_height.min(height - 2);
        let region = region(
            vec![reversed_rectangle(0, 0, width, height)],
            vec![reversed_rectangle(1, 1, 1 + hole_width, 1 + hole_height)],
        );
        prop_assert_eq!(
            filled_area(&region),
            Classification::Decided(Some(Real::from(
                width * height - hole_width * hole_height,
            )))
        );
    }
}

#[test]
fn batched_classifier_and_structural_facts_use_the_unified_surface() {
    let region = region(
        vec![rectangle(0, 0, 10, 10), rectangle(4, 4, 6, 6)],
        vec![rectangle(2, 2, 8, 8)],
    );
    let Classification::Decided(facts) = region
        .structural_facts_with_policy(&policy())
        .unwrap()
        .into_value()
    else {
        panic!("native specialization should expose structural facts");
    };
    assert!(facts.has_decided_region_box);
    assert_eq!(facts.material_contour_count, 2);
    assert_eq!(facts.hole_contour_count, 1);
    assert_eq!(
        facts.segment_kinds,
        SegmentKindCounts { lines: 12, arcs: 0 }
    );

    let points = [p(1, 1), p(3, 3), p(5, 5), p(11, 1), p(100, 100), p(2, 5)];
    let queries = points
        .each_ref()
        .map(|point| crate::CurvePoint2::from(point.clone()));
    let batched = region
        .classify_points_with_policy(&queries, &policy())
        .unwrap()
        .into_value();
    assert_eq!(
        batched,
        points
            .iter()
            .map(|point| classify(&region, point))
            .collect::<Vec<_>>()
    );
}

#[test]
fn empty_unordered_arrangement_reenters_exact_set_operations() {
    use crate::BooleanOp;
    let material = region(vec![rectangle(0, 0, 4, 4)], Vec::new());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill_rule in [FillRule::EvenOdd, FillRule::NonZero] {
            let outcome =
                CurveRegion2::arrange_unordered_segments_with_policy(&[], fill_rule, &policy)
                    .expect("an empty arrangement represents the empty set");
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            let empty = outcome.into_value();
            assert!(empty.is_empty());
            for (operation, empty_first_filled, empty_second_filled) in [
                (BooleanOp::Union, true, true),
                (BooleanOp::Intersection, false, false),
                (BooleanOp::Difference, false, true),
                (BooleanOp::Xor, true, true),
            ] {
                for (first, second, filled) in [
                    (&empty, &material, empty_first_filled),
                    (&material, &empty, empty_second_filled),
                ] {
                    let result = first
                        .boolean_region_with_policy(second, operation, &policy)
                        .unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let result = result.into_value();
                    assert_eq!(result.is_empty(), !filled);
                    for (point, expected) in [
                        (
                            p(2, 2),
                            if filled {
                                RegionPointLocation::Inside
                            } else {
                                RegionPointLocation::Outside
                            },
                        ),
                        (
                            p(4, 2),
                            if filled {
                                RegionPointLocation::Boundary
                            } else {
                                RegionPointLocation::Outside
                            },
                        ),
                        (p(5, 2), RegionPointLocation::Outside),
                    ] {
                        assert_eq!(
                            result
                                .classify_point_with_policy(&point.into(), &policy)
                                .unwrap()
                                .into_value(),
                            Classification::Decided(expected)
                        );
                    }
                }
            }
            let offset = empty
                .offset_with_policy(Real::from(-1), &crate::OffsetCornerStyle2::Round, &policy)
                .unwrap();
            assert_eq!(offset.certainty, CurveCertainty::Certified);
            assert!(offset.value.is_empty());
        }
    }
}

#[test]
fn empty_region_offsets_preserve_set_and_policy_identity() {
    use crate::{BooleanOp, ExactCurveError, OffsetCornerStyle2};
    let material = region(vec![rectangle(0, 0, 4, 4)], Vec::new());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let difference = material
            .boolean_region_with_policy(&material, BooleanOp::Difference, &policy)
            .unwrap()
            .into_value();
        let collapsed = material
            .offset_with_policy(Real::from(-3), &OffsetCornerStyle2::Bevel, &policy)
            .unwrap()
            .into_value();
        assert!(difference.is_empty());
        assert!(collapsed.is_empty());
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let symbolic_zero = &sine * &sine + &cosine * &cosine - Real::one();
        for empty in [CurveRegion2::empty(), difference, collapsed] {
            for style in [
                OffsetCornerStyle2::Round,
                OffsetCornerStyle2::Bevel,
                OffsetCornerStyle2::Miter {
                    limit: Real::from(4),
                },
            ] {
                for distance in [
                    Real::from(-1),
                    Real::zero(),
                    Real::one(),
                    symbolic_zero.clone(),
                ] {
                    let result = empty.offset_with_policy(distance, &style, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert!(result.value.is_empty());
                }
            }
            assert!(matches!(
                empty.offset_with_policy(
                    Real::one(),
                    &OffsetCornerStyle2::Miter {
                        limit: Real::from(-1)
                    },
                    &policy
                ),
                Err(ExactCurveError::Invalid {
                    cause: CurveError::InvalidOffsetOptions,
                    ..
                })
            ));
        }
    }
}

#[test]
fn unordered_native_regions_reenter_operations_without_summary_queries() {
    use crate::{BooleanOp, OffsetCornerStyle2};
    let full_circle = CircularArc2::try_from_center(p(2, 0), p(2, 0), p(0, 0), false).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill in [FillRule::NonZero, FillRule::EvenOdd] {
            for (segments, boundary, outside) in [
                (
                    rectangle(0, 0, 4, 4).segments().to_vec(),
                    p(-1, 2),
                    p(-2, 2),
                ),
                (vec![Segment2::Arc(full_circle.clone())], p(3, 0), p(4, 0)),
            ] {
                let outcome =
                    CurveRegion2::arrange_unordered_segments_with_policy(&segments, fill, &policy)
                        .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let region = outcome.into_value();
                // The next operation is deliberately the first consumer.
                let grown = region
                    .offset_with_policy(Real::one(), &OffsetCornerStyle2::Round, &policy)
                    .unwrap()
                    .into_value();
                assert_eq!(
                    grown
                        .classify_point_with_policy(&boundary.into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Boundary)
                );
                assert_eq!(
                    grown
                        .classify_point_with_policy(&outside.into(), &policy)
                        .unwrap()
                        .into_value(),
                    Classification::Decided(RegionPointLocation::Outside)
                );
                let original = grown
                    .boolean_region_with_policy(&region, BooleanOp::Intersection, &policy)
                    .unwrap()
                    .into_value();
                assert!(
                    original
                        .boolean_region_with_policy(&region, BooleanOp::Xor, &policy)
                        .unwrap()
                        .into_value()
                        .is_empty()
                );
            }
        }
    }
}

#[test]
fn round_erosion_of_regions_narrower_than_its_diameter_is_empty() {
    use crate::OffsetCornerStyle2;
    // A diagonal strip of width sqrt(2): a disk of radius 1 cannot fit, while
    // one of radius 1/2 can, so only the smaller erosion keeps material.
    let strip = region(
        vec![
            Contour2::from_bulge_vertices(&[
                vertex(0, 0),
                vertex(10, 10),
                vertex(9, 11),
                vertex(-1, 1),
            ])
            .unwrap(),
        ],
        Vec::new(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let collapsed = strip
            .offset_with_policy(Real::from(-1), &OffsetCornerStyle2::Round, &policy)
            .unwrap();
        assert_eq!(collapsed.certainty, CurveCertainty::Certified);
        assert!(collapsed.value.is_empty());
        let retained = strip
            .offset_with_policy(
                (Real::from(-1) / Real::from(2)).unwrap(),
                &OffsetCornerStyle2::Round,
                &policy,
            )
            .unwrap()
            .into_value();
        assert!(!retained.is_empty());
    }
}

#[test]
fn approximate_region_bounds_order_approximately_coincident_representations() {
    use crate::BooleanOp;
    // The clip's sides pass through the diamond's exact vertices (±1, 0), and
    // its bottom lies at an exact zero that refinement cannot certify. The
    // approximate Boolean joins the cut edge's own endpoint expressions to
    // those exact vertices.
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let height = &sine * &sine + &cosine * &cosine - Real::one();
    let diamond = region(
        vec![
            Contour2::from_bulge_vertices(&[
                vertex(1, 0),
                vertex(0, 1),
                vertex(-1, 0),
                vertex(0, -1),
            ])
            .unwrap(),
        ],
        Vec::new(),
    );
    let upper = region(
        vec![
            Contour2::from_bulge_vertices(&[
                BulgeVertex2::new(crate::Point2::new(s(-1), Real::zero() - &height), s(0)),
                BulgeVertex2::new(crate::Point2::new(s(1), Real::zero() - &height), s(0)),
                vertex(1, 2),
                vertex(-1, 2),
            ])
            .unwrap(),
        ],
        Vec::new(),
    );
    let policy = CurveContext::APPROXIMATE_512;
    let clipped = diamond
        .boolean_region_with_policy(&upper, BooleanOp::Intersection, &policy)
        .unwrap()
        .into_value();
    let bounds = clipped.bounds_with_policy(&policy).unwrap();
    assert_eq!(bounds.certainty, CurveCertainty::Approximate512Consumed);
    let Classification::Decided(bounds) = bounds.value else {
        panic!(
            "approximate bounds must decide the ordering: {:?}",
            bounds.value
        );
    };
    assert_eq!(bounds.max_y(), &Real::one());
    assert_eq!(bounds.min_x().to_f64_lossy(), Some(-1.0));
    assert_eq!(bounds.max_x().to_f64_lossy(), Some(1.0));
    assert_eq!(bounds.min_y().to_f64_lossy().map(f64::abs), Some(0.0));
}
