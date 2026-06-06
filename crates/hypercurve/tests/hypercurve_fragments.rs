#![allow(clippy::too_many_arguments)]

use hypercurve::{
    ArcArcIntersection, BulgeVertex2, CircularArc2, Classification, Contour2, ContourFragmentSet,
    ContourIntersection, ContourOperand, ContourSplitMarkers, CurveError, CurvePolicy,
    LineArcIntersection, LineLineIntersection, LineSeg2, Point2, Real, Segment2,
    SegmentIntersection, SegmentSplitMarker, Tolerance,
};
use proptest::prelude::*;

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (s(numerator) / s(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2]) -> Contour2 {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
}

fn arc_overlap_cutter() -> Contour2 {
    Contour2::try_new(vec![
        Segment2::Arc(CircularArc2::try_from_center(p(1, -1), p(2, 0), p(1, 0), false).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(2, -2), p(1, -1)).unwrap()),
    ])
    .unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn approx_policy() -> CurvePolicy {
    CurvePolicy::edge_preview(Tolerance::new(1e-7, 1e-7))
}

#[test]
fn contour_fragments_split_line_segments_at_point_events() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(3, -1, 0),
        vertex(3, 1, 0),
        vertex(1, 1, 0),
        vertex(1, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let fragments = a
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap();
    let Classification::Decided(fragments) = fragments else {
        panic!("expected decided fragments");
    };

    assert_eq!(fragments.len(), 6);
    assert_eq!(fragments.fragments()[0].source_segment_index, 0);
    assert_eq!(fragments.fragments()[0].source_range.start(), &s(0));
    assert_eq!(fragments.fragments()[0].source_range.end(), &q(1, 4));
    assert_line(&fragments.fragments()[0].segment, p(0, 0), p(1, 0));
    assert_line(&fragments.fragments()[1].segment, p(1, 0), p(3, 0));
    assert_line(&fragments.fragments()[2].segment, p(3, 0), p(4, 0));
}

#[test]
fn contour_fragments_split_line_segments_at_overlap_endpoints() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(2, 0, 0),
        vertex(6, 0, 0),
        vertex(6, -2, 0),
        vertex(2, -2, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let Classification::Decided(fragments) = a
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap()
    else {
        panic!("expected decided fragments");
    };

    assert_eq!(fragments.len(), 5);
    assert_line(&fragments.fragments()[0].segment, p(0, 0), p(2, 0));
    assert_line(&fragments.fragments()[1].segment, p(2, 0), p(4, 0));
}

#[test]
fn contour_fragments_split_arc_segments_at_event_points() {
    let circle = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let cutter = contour(&[
        vertex(1, -2, 0),
        vertex(1, 2, 0),
        vertex(3, 2, 0),
        vertex(3, -2, 0),
    ]);

    let events = circle.intersect_contour(&cutter, &policy()).unwrap();
    let Classification::Decided(fragments) = circle
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap()
    else {
        panic!("expected decided fragments");
    };

    assert_eq!(fragments.len(), 4);
    assert_arc(&fragments.fragments()[0].segment, p(0, 0), p(1, -1));
    assert_arc(&fragments.fragments()[1].segment, p(1, -1), p(2, 0));
    assert_arc(&fragments.fragments()[2].segment, p(2, 0), p(1, 1));
    assert_arc(&fragments.fragments()[3].segment, p(1, 1), p(0, 0));
}

#[test]
fn contour_fragments_split_arc_segments_at_overlap_endpoints() {
    let circle = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let cutter = arc_overlap_cutter();

    let events = circle.intersect_contour(&cutter, &policy()).unwrap();
    let Classification::Decided(fragments) = circle
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap()
    else {
        panic!("expected decided arc overlap fragments");
    };

    assert_eq!(fragments.len(), 3);
    assert_arc(&fragments.fragments()[0].segment, p(0, 0), p(1, -1));
    assert_arc(&fragments.fragments()[1].segment, p(1, -1), p(2, 0));
    assert_arc(&fragments.fragments()[2].segment, p(2, 0), p(0, 0));
}

#[test]
fn approximate_arc_fragment_uses_source_radius_for_policy_on_circle_split_points() {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(Point2::new(af(1.0), af(0.0)), af(1.0)),
        BulgeVertex2::new(Point2::new(af(-1.0), af(0.0)), af(1.0)),
    ])
    .unwrap();
    let almost_top = Point2::new(af(0.0), af(1.0 + 1e-12));
    let markers = ContourSplitMarkers::new(vec![
        vec![
            SegmentSplitMarker {
                segment_index: 0,
                param: af(0.0),
                point: Point2::new(af(1.0), af(0.0)),
            },
            SegmentSplitMarker {
                segment_index: 0,
                param: af(0.5),
                point: almost_top.clone(),
            },
            SegmentSplitMarker {
                segment_index: 0,
                param: af(1.0),
                point: Point2::new(af(-1.0), af(0.0)),
            },
        ],
        vec![
            SegmentSplitMarker {
                segment_index: 1,
                param: af(0.0),
                point: Point2::new(af(-1.0), af(0.0)),
            },
            SegmentSplitMarker {
                segment_index: 1,
                param: af(1.0),
                point: Point2::new(af(1.0), af(0.0)),
            },
        ],
    ]);

    let Classification::Decided(fragments) =
        ContourFragmentSet::from_split_markers(&contour, &markers, &approx_policy()).unwrap()
    else {
        panic!("policy-on-circle split points should produce decided fragments");
    };

    assert_eq!(fragments.len(), 3);
    assert_arc_approx(
        &fragments.fragments()[0].segment,
        (1.0, 0.0),
        (0.0, 1.0 + 1e-12),
    );
    assert_arc_approx(
        &fragments.fragments()[1].segment,
        (0.0, 1.0 + 1e-12),
        (-1.0, 0.0),
    );
}

#[test]
fn contour_self_fragments_split_nonadjacent_line_arc_crossing() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(3, 2, 0),
        vertex(1, 2, 0),
        vertex(1, -2, 0),
        vertex(3, -3, 0),
        vertex(-1, -3, 0),
    ]);

    let events = contour.intersect_self(&policy()).unwrap();
    let Classification::Decided(fragments) = contour
        .split_at_self_intersections(&events, &policy())
        .unwrap()
    else {
        panic!("expected decided self-intersection fragments");
    };

    assert_eq!(fragments.len(), 9);
    assert_eq!(count_source_fragments(&fragments, 0), 2);
    assert_eq!(count_source_fragments(&fragments, 3), 2);
    assert_arc(&fragments.fragments()[0].segment, p(0, 0), p(1, -1));
    assert_arc(&fragments.fragments()[1].segment, p(1, -1), p(2, 0));
    assert_line(&fragments.fragments()[4].segment, p(1, 2), p(1, -1));
    assert_line(&fragments.fragments()[5].segment, p(1, -1), p(1, -2));
}

#[test]
fn contour_self_fragments_split_adjacent_line_arc_extra_crossing() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(0, -2, 0),
        vertex(-1, 0, 0),
    ]);

    let events = contour.intersect_self(&policy()).unwrap();
    let Classification::Decided(fragments) = contour
        .split_at_self_intersections(&events, &policy())
        .unwrap()
    else {
        panic!("expected decided self-intersection fragments");
    };

    assert_eq!(fragments.len(), 6);
    assert_eq!(count_source_fragments(&fragments, 0), 2);
    assert_eq!(count_source_fragments(&fragments, 1), 2);
    assert_arc(&fragments.fragments()[0].segment, p(0, 0), p(1, -1));
    assert_arc(&fragments.fragments()[1].segment, p(1, -1), p(2, 0));
    assert_line(&fragments.fragments()[2].segment, p(2, 0), p(1, -1));
    assert_line(&fragments.fragments()[3].segment, p(1, -1), p(0, -2));
}

#[test]
fn edge_preview_retains_rotated_arc_arc_event_regression() {
    let (first, second) = approx_mixed_pair_contours(
        2,
        49.85461434726879,
        0.9197730594020808,
        true,
        0.0,
        0.2,
        2.25,
        0.7165237049786178,
        2.861460986580769,
        0.45,
        false,
        2.9892002795974872,
        0.0,
        0.0,
    );
    let policy = approx_policy();
    let direct = first.segments()[0]
        .intersect_segment(&second.segments()[0], &policy)
        .unwrap();
    assert!(
        relation_has_reportable_intersection(&direct),
        "direct arc-arc relation should retain the preview hit: {direct:?}"
    );

    let events = first.intersect_contour(&second, &policy).unwrap();
    assert!(
        events
            .events()
            .iter()
            .any(|event| event_on_pair(event, 0, 0)),
        "contour event pipeline should retain the preview arc-arc hit"
    );
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        max_shrink_iters: 512,
        ..ProptestConfig::default()
    })]

    #[test]
    fn approximate_arc_heavy_slices_do_not_surface_radius_mismatch(
        vertex_count in 3_usize..9,
        first_radii in proptest::collection::vec(0.7_f64..1.35, 8),
        second_radii in proptest::collection::vec(0.7_f64..1.35, 8),
        first_bulges in proptest::collection::vec(-0.85_f64..0.85, 8),
        second_bulges in proptest::collection::vec(-0.85_f64..0.85, 8),
        dx in -3.5_f64..3.5,
        dy in -3.5_f64..3.5,
        angle_shift in 0.0_f64..0.8,
    ) {
        let first = approx_radial_contour(vertex_count, &first_radii, &first_bulges, 0.0, 0.0, 0.0);
        let second = approx_radial_contour(vertex_count, &second_radii, &second_bulges, dx, dy, angle_shift);
        let policy = approx_policy();
        let events = match first.intersect_contour(&second, &policy) {
            Ok(events) => events,
            Err(error) => {
                prop_assert!(!matches!(error, CurveError::RadiusMismatch));
                return Ok(());
            }
        };

        for (contour, operand) in [
            (&first, ContourOperand::First),
            (&second, ContourOperand::Second),
        ] {
            let split = contour.split_at_intersections(&events, operand, &policy);
            prop_assert!(
                !matches!(split, Err(CurveError::RadiusMismatch)),
                "split returned RadiusMismatch for {operand:?}"
            );
        }
    }

    #[test]
    fn self_line_arc_slices_fuzz_nonadjacent_crossings(
        radius in 0.75_f64..30.0,
        x_fraction in 0.15_f64..0.85,
        top_scale in 1.1_f64..3.0,
        bottom_scale in 1.1_f64..3.0,
    ) {
        let x = 2.0 * radius * x_fraction;
        let contour = approx_self_line_arc_contour(
            radius,
            x,
            radius * top_scale,
            -radius * bottom_scale,
        );
        let policy = approx_policy();
        let events = contour.intersect_self(&policy).unwrap();

        prop_assert!(
            events.events().iter().any(|event| point_event_on_pair(event, 0, 3)),
            "expected a retained self line-arc point between arc segment 0 and line segment 3"
        );

        let split = contour.split_at_self_intersections(&events, &policy);
        prop_assert!(
            !matches!(split, Err(CurveError::RadiusMismatch)),
            "self split returned RadiusMismatch"
        );
        let Classification::Decided(fragments) = split.unwrap() else {
            return Ok(());
        };

        prop_assert!(count_source_fragments(&fragments, 0) >= 2);
        prop_assert!(count_source_fragments(&fragments, 3) >= 2);
    }

    #[test]
    fn self_line_arc_slices_fuzz_adjacent_crossings(radius in 0.75_f64..30.0) {
        let contour = approx_adjacent_self_line_arc_contour(radius);
        let policy = approx_policy();
        let events = contour.intersect_self(&policy).unwrap();

        prop_assert!(
            events.events().iter().any(|event| point_event_on_pair(event, 0, 1)),
            "expected the interior adjacent line-arc crossing to remain after endpoint filtering"
        );

        let split = contour.split_at_self_intersections(&events, &policy);
        prop_assert!(
            !matches!(split, Err(CurveError::RadiusMismatch)),
            "adjacent self split returned RadiusMismatch"
        );
        let Classification::Decided(fragments) = split.unwrap() else {
            return Ok(());
        };

        prop_assert!(count_source_fragments(&fragments, 0) >= 2);
        prop_assert!(count_source_fragments(&fragments, 1) >= 2);
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 1024,
        max_shrink_iters: 1024,
        ..ProptestConfig::default()
    })]

    #[test]
    fn self_line_arc_slices_fuzz_rotated_secants(
        radius in 0.5_f64..60.0,
        arc_fraction in 0.01_f64..0.99,
        upper_arc in any::<bool>(),
        secant_offset in -0.85_f64..0.85,
        line_len_scale in 2.25_f64..16.0,
        rotation in 0.0_f64..std::f64::consts::TAU,
        tx in -75.0_f64..75.0,
        ty in -75.0_f64..75.0,
    ) {
        let contour = approx_rotated_secant_self_contour(
            radius,
            arc_fraction,
            upper_arc,
            secant_offset,
            line_len_scale,
            rotation,
            tx,
            ty,
        );
        let policy = approx_policy();
        let events = contour.intersect_self(&policy).unwrap();

        prop_assert!(
            events.events().iter().any(|event| point_event_on_pair(event, 0, 3)),
            "expected a retained rotated line-arc self-slice event on arc 0 and cutter 3"
        );

        let split = contour.split_at_self_intersections(&events, &policy);
        prop_assert!(
            !matches!(split, Err(CurveError::RadiusMismatch)),
            "rotated self split returned RadiusMismatch"
        );
        let Classification::Decided(fragments) = split.unwrap() else {
            return Ok(());
        };

        prop_assert!(count_source_fragments(&fragments, 0) >= 2);
        prop_assert!(count_source_fragments(&fragments, 3) >= 2);
    }

    #[test]
    fn pair_line_arc_slices_fuzz_rotated_secants(
        radius in 0.5_f64..60.0,
        arc_fraction in 0.01_f64..0.99,
        upper_arc in any::<bool>(),
        secant_offset in -0.85_f64..0.85,
        line_len_scale in 2.25_f64..16.0,
        rotation in 0.0_f64..std::f64::consts::TAU,
        tx in -75.0_f64..75.0,
        ty in -75.0_f64..75.0,
    ) {
        let (arc_contour, cutter_contour) = approx_rotated_secant_pair_contours(
            radius,
            arc_fraction,
            upper_arc,
            secant_offset,
            line_len_scale,
            rotation,
            tx,
            ty,
        );
        let policy = approx_policy();
        let events = arc_contour.intersect_contour(&cutter_contour, &policy).unwrap();

        prop_assert!(
            events.events().iter().any(|event| point_event_on_pair(event, 0, 0)),
            "expected a retained rotated line-arc pair-slice event on arc 0 and cutter 0"
        );

        let first_split = arc_contour.split_at_intersections(&events, ContourOperand::First, &policy);
        prop_assert!(
            !matches!(first_split, Err(CurveError::RadiusMismatch)),
            "rotated pair split returned RadiusMismatch for first contour"
        );
        let second_split = cutter_contour.split_at_intersections(&events, ContourOperand::Second, &policy);
        prop_assert!(
            !matches!(second_split, Err(CurveError::RadiusMismatch)),
            "rotated pair split returned RadiusMismatch for second contour"
        );

        let Classification::Decided(first_fragments) = first_split.unwrap() else {
            return Ok(());
        };
        let Classification::Decided(second_fragments) = second_split.unwrap() else {
            return Ok(());
        };

        prop_assert!(count_source_fragments(&first_fragments, 0) >= 2);
        prop_assert!(count_source_fragments(&second_fragments, 0) >= 2);
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 1024,
        max_shrink_iters: 1024,
        ..ProptestConfig::default()
    })]

    #[test]
    fn contour_event_fuzz_retains_direct_mixed_segment_hits(
        segment_kind in 0_u8..3,
        radius in 0.75_f64..75.0,
        arc_fraction in 0.02_f64..0.98,
        upper_arc in any::<bool>(),
        secant_offset in -0.85_f64..0.85,
        line_angle in 0.2_f64..(std::f64::consts::PI - 0.2),
        line_len_scale in 2.25_f64..18.0,
        second_radius_scale in 0.45_f64..1.75,
        second_center_angle in 0.1_f64..(std::f64::consts::TAU - 0.1),
        second_sweep in 0.45_f64..2.7,
        second_clockwise in any::<bool>(),
        rotation in 0.0_f64..std::f64::consts::TAU,
        tx in -95.0_f64..95.0,
        ty in -95.0_f64..95.0,
    ) {
        let (first, second) = approx_mixed_pair_contours(
            segment_kind,
            radius,
            arc_fraction,
            upper_arc,
            secant_offset,
            line_angle,
            line_len_scale,
            second_radius_scale,
            second_center_angle,
            second_sweep,
            second_clockwise,
            rotation,
            tx,
            ty,
        );
        let policy = approx_policy();
        let direct = first.segments()[0]
            .intersect_segment(&second.segments()[0], &policy)
            .unwrap();

        prop_assert!(
            relation_has_reportable_intersection(&direct),
            "mixed pair generator should place a direct segment hit: {direct:?}"
        );

        let events = first.intersect_contour(&second, &policy).unwrap();
        prop_assert!(
            events.events().iter().any(|event| event_on_pair(event, 0, 0)),
            "contour event pipeline lost a direct mixed segment hit: {direct:?}"
        );
    }

    #[test]
    fn self_event_fuzz_retains_direct_mixed_segment_hits(
        segment_kind in 0_u8..3,
        radius in 0.75_f64..75.0,
        arc_fraction in 0.02_f64..0.98,
        upper_arc in any::<bool>(),
        secant_offset in -0.85_f64..0.85,
        line_angle in 0.2_f64..(std::f64::consts::PI - 0.2),
        line_len_scale in 2.25_f64..18.0,
        second_radius_scale in 0.45_f64..1.75,
        second_center_angle in 0.1_f64..(std::f64::consts::TAU - 0.1),
        second_sweep in 0.45_f64..2.7,
        second_clockwise in any::<bool>(),
        rotation in 0.0_f64..std::f64::consts::TAU,
        tx in -95.0_f64..95.0,
        ty in -95.0_f64..95.0,
    ) {
        let contour = approx_mixed_self_contour(
            segment_kind,
            radius,
            arc_fraction,
            upper_arc,
            secant_offset,
            line_angle,
            line_len_scale,
            second_radius_scale,
            second_center_angle,
            second_sweep,
            second_clockwise,
            rotation,
            tx,
            ty,
        );
        let policy = approx_policy();
        let direct = contour.segments()[0]
            .intersect_segment(&contour.segments()[3], &policy)
            .unwrap();

        prop_assert!(
            relation_has_reportable_intersection(&direct),
            "mixed self generator should place a direct segment hit: {direct:?}"
        );

        let events = contour.intersect_self(&policy).unwrap();
        prop_assert!(
            events.events().iter().any(|event| event_on_pair(event, 0, 3)),
            "self event pipeline lost a direct mixed segment hit: {direct:?}"
        );

        let split = contour.split_at_self_intersections(&events, &policy);
        prop_assert!(
            !matches!(split, Err(CurveError::RadiusMismatch)),
            "mixed self split returned RadiusMismatch"
        );
        let Classification::Decided(fragments) = split.unwrap() else {
            return Ok(());
        };

        prop_assert!(count_source_fragments(&fragments, 0) >= 2);
        prop_assert!(count_source_fragments(&fragments, 3) >= 2);
    }
}

fn assert_line(segment: &Segment2, start: hypercurve::Point2, end: hypercurve::Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected line fragment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn assert_arc(segment: &Segment2, start: hypercurve::Point2, end: hypercurve::Point2) {
    let Segment2::Arc(arc) = segment else {
        panic!("expected arc fragment");
    };
    assert_eq!(arc.start(), &start);
    assert_eq!(arc.end(), &end);
    assert_eq!(arc.center(), &p(1, 0));
}

fn count_source_fragments(fragments: &ContourFragmentSet, source_segment_index: usize) -> usize {
    fragments
        .fragments()
        .iter()
        .filter(|fragment| fragment.source_segment_index == source_segment_index)
        .count()
}

fn af(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

fn ap(x: f64, y: f64) -> Point2 {
    Point2::new(af(x), af(y))
}

fn av(x: f64, y: f64, bulge: f64) -> BulgeVertex2 {
    BulgeVertex2::new(ap(x, y), af(bulge))
}

fn approx_self_line_arc_contour(radius: f64, x: f64, top: f64, bottom: f64) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        av(0.0, 0.0, 1.0),
        av(2.0 * radius, 0.0, 0.0),
        av(3.0 * radius, top, 0.0),
        av(x, top, 0.0),
        av(x, bottom, 0.0),
        av(3.0 * radius, bottom - radius, 0.0),
        av(-radius, bottom - radius, 0.0),
    ])
    .unwrap()
}

fn approx_adjacent_self_line_arc_contour(radius: f64) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        av(0.0, 0.0, 1.0),
        av(2.0 * radius, 0.0, 0.0),
        av(0.0, -2.0 * radius, 0.0),
        av(-radius, 0.0, 0.0),
    ])
    .unwrap()
}

#[derive(Clone, Copy, Debug)]
struct FPoint {
    x: f64,
    y: f64,
}

fn approx_rotated_secant_self_contour(
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    secant_offset: f64,
    line_len_scale: f64,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> Contour2 {
    let data = rotated_secant_geometry(
        radius,
        arc_fraction,
        upper_arc,
        secant_offset,
        line_len_scale,
        rotation,
        tx,
        ty,
    );
    let route = route_scale(radius, line_len_scale);
    let c1 = transform_point(FPoint { x: route, y: route }, rotation, tx, ty);
    let c2 = transform_point(
        FPoint {
            x: -route,
            y: route,
        },
        rotation,
        tx,
        ty,
    );

    Contour2::from_bulge_vertices(&[
        av(data.arc_start.x, data.arc_start.y, data.bulge),
        av(data.arc_end.x, data.arc_end.y, 0.0),
        av(c1.x, c1.y, 0.0),
        av(data.line_start.x, data.line_start.y, 0.0),
        av(data.line_end.x, data.line_end.y, 0.0),
        av(c2.x, c2.y, 0.0),
    ])
    .unwrap()
}

fn approx_rotated_secant_pair_contours(
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    secant_offset: f64,
    line_len_scale: f64,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> (Contour2, Contour2) {
    let data = rotated_secant_geometry(
        radius,
        arc_fraction,
        upper_arc,
        secant_offset,
        line_len_scale,
        rotation,
        tx,
        ty,
    );
    let width = radius * 0.2 + 0.25;
    let normal = FPoint {
        x: -data.line_dir.y * width,
        y: data.line_dir.x * width,
    };
    let cutter_offset_end = FPoint {
        x: data.line_end.x + normal.x,
        y: data.line_end.y + normal.y,
    };
    let cutter_offset_start = FPoint {
        x: data.line_start.x + normal.x,
        y: data.line_start.y + normal.y,
    };

    let arc_contour = Contour2::from_bulge_vertices(&[
        av(data.arc_start.x, data.arc_start.y, data.bulge),
        av(data.arc_end.x, data.arc_end.y, data.bulge),
    ])
    .unwrap();
    let cutter_contour = Contour2::from_bulge_vertices(&[
        av(data.line_start.x, data.line_start.y, 0.0),
        av(data.line_end.x, data.line_end.y, 0.0),
        av(cutter_offset_end.x, cutter_offset_end.y, 0.0),
        av(cutter_offset_start.x, cutter_offset_start.y, 0.0),
    ])
    .unwrap();

    (arc_contour, cutter_contour)
}

fn approx_mixed_pair_contours(
    segment_kind: u8,
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    secant_offset: f64,
    line_angle: f64,
    line_len_scale: f64,
    second_radius_scale: f64,
    second_center_angle: f64,
    second_sweep: f64,
    second_clockwise: bool,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> (Contour2, Contour2) {
    match segment_kind % 3 {
        0 => approx_line_line_pair_contours(
            radius,
            arc_fraction,
            line_angle,
            line_len_scale,
            rotation,
            tx,
            ty,
        ),
        1 => approx_rotated_secant_pair_contours(
            radius,
            arc_fraction,
            upper_arc,
            secant_offset,
            line_len_scale,
            rotation,
            tx,
            ty,
        ),
        _ => approx_arc_arc_pair_contours(
            radius,
            arc_fraction,
            upper_arc,
            line_len_scale,
            second_radius_scale,
            second_center_angle,
            second_sweep,
            second_clockwise,
            rotation,
            tx,
            ty,
        ),
    }
}

fn approx_mixed_self_contour(
    segment_kind: u8,
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    secant_offset: f64,
    line_angle: f64,
    line_len_scale: f64,
    second_radius_scale: f64,
    second_center_angle: f64,
    second_sweep: f64,
    second_clockwise: bool,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> Contour2 {
    match segment_kind % 3 {
        0 => approx_line_line_self_contour(
            radius,
            arc_fraction,
            line_angle,
            line_len_scale,
            rotation,
            tx,
            ty,
        ),
        1 => approx_rotated_secant_self_contour(
            radius,
            arc_fraction,
            upper_arc,
            secant_offset,
            line_len_scale,
            rotation,
            tx,
            ty,
        ),
        _ => approx_arc_arc_self_contour(
            radius,
            arc_fraction,
            upper_arc,
            line_len_scale,
            second_radius_scale,
            second_center_angle,
            second_sweep,
            second_clockwise,
            rotation,
            tx,
            ty,
        ),
    }
}

fn approx_line_line_pair_contours(
    radius: f64,
    crossing_fraction: f64,
    line_angle: f64,
    line_len_scale: f64,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> (Contour2, Contour2) {
    let (a_start, a_end, b_start, b_end) =
        line_line_targets(radius, crossing_fraction, line_angle, line_len_scale);
    let route = route_scale(radius, line_len_scale);
    (
        transformed_target_contour(a_start, a_end, 0.0, route, rotation, tx, ty),
        transformed_target_contour(b_start, b_end, 0.0, route * 0.75, rotation, tx, ty),
    )
}

fn approx_line_line_self_contour(
    radius: f64,
    crossing_fraction: f64,
    line_angle: f64,
    line_len_scale: f64,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> Contour2 {
    let (a_start, a_end, b_start, b_end) =
        line_line_targets(radius, crossing_fraction, line_angle, line_len_scale);
    let route = route_scale(radius, line_len_scale);
    Contour2::from_bulge_vertices(&[
        transformed_vertex(a_start, 0.0, rotation, tx, ty),
        transformed_vertex(a_end, 0.0, rotation, tx, ty),
        transformed_vertex(FPoint { x: route, y: route }, 0.0, rotation, tx, ty),
        transformed_vertex(b_start, 0.0, rotation, tx, ty),
        transformed_vertex(b_end, 0.0, rotation, tx, ty),
        transformed_vertex(
            FPoint {
                x: -route,
                y: -route,
            },
            0.0,
            rotation,
            tx,
            ty,
        ),
    ])
    .unwrap()
}

fn line_line_targets(
    radius: f64,
    crossing_fraction: f64,
    line_angle: f64,
    line_len_scale: f64,
) -> (FPoint, FPoint, FPoint, FPoint) {
    let a_start = FPoint { x: -radius, y: 0.0 };
    let a_end = FPoint { x: radius, y: 0.0 };
    let crossing = FPoint {
        x: -radius + 2.0 * radius * crossing_fraction,
        y: 0.0,
    };
    let len = radius * line_len_scale + 1.0;
    let dir = FPoint {
        x: line_angle.cos(),
        y: line_angle.sin(),
    };
    let b_start = FPoint {
        x: crossing.x - len * dir.x,
        y: crossing.y - len * dir.y,
    };
    let b_end = FPoint {
        x: crossing.x + len * dir.x,
        y: crossing.y + len * dir.y,
    };
    (a_start, a_end, b_start, b_end)
}

fn approx_arc_arc_pair_contours(
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    line_len_scale: f64,
    second_radius_scale: f64,
    second_center_angle: f64,
    second_sweep: f64,
    second_clockwise: bool,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> (Contour2, Contour2) {
    let data = arc_arc_targets(
        radius,
        arc_fraction,
        upper_arc,
        second_radius_scale,
        second_center_angle,
        second_sweep,
        second_clockwise,
    );
    let route = route_scale(radius, line_len_scale);
    let first_width = if upper_arc { -route } else { route };
    let second_width = if second_clockwise {
        -route * 0.75
    } else {
        route * 0.75
    };
    (
        transformed_target_contour(
            data.first_start,
            data.first_end,
            data.first_bulge,
            first_width,
            rotation,
            tx,
            ty,
        ),
        transformed_target_contour(
            data.second_start,
            data.second_end,
            data.second_bulge,
            second_width,
            rotation,
            tx,
            ty,
        ),
    )
}

fn approx_arc_arc_self_contour(
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    line_len_scale: f64,
    second_radius_scale: f64,
    second_center_angle: f64,
    second_sweep: f64,
    second_clockwise: bool,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> Contour2 {
    let data = arc_arc_targets(
        radius,
        arc_fraction,
        upper_arc,
        second_radius_scale,
        second_center_angle,
        second_sweep,
        second_clockwise,
    );
    let route = route_scale(radius, line_len_scale);
    Contour2::from_bulge_vertices(&[
        transformed_vertex(data.first_start, data.first_bulge, rotation, tx, ty),
        transformed_vertex(data.first_end, 0.0, rotation, tx, ty),
        transformed_vertex(FPoint { x: route, y: route }, 0.0, rotation, tx, ty),
        transformed_vertex(data.second_start, data.second_bulge, rotation, tx, ty),
        transformed_vertex(data.second_end, 0.0, rotation, tx, ty),
        transformed_vertex(
            FPoint {
                x: -route,
                y: -route,
            },
            0.0,
            rotation,
            tx,
            ty,
        ),
    ])
    .unwrap()
}

#[derive(Clone, Copy, Debug)]
struct ArcArcTargets {
    first_start: FPoint,
    first_end: FPoint,
    first_bulge: f64,
    second_start: FPoint,
    second_end: FPoint,
    second_bulge: f64,
}

fn arc_arc_targets(
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    second_radius_scale: f64,
    second_center_angle: f64,
    second_sweep: f64,
    second_clockwise: bool,
) -> ArcArcTargets {
    let target_x = -radius + 2.0 * radius * arc_fraction;
    let target_y = if upper_arc { 1.0 } else { -1.0 }
        * (radius.mul_add(radius, -(target_x * target_x)))
            .max(0.0)
            .sqrt();
    let target_angle = target_y.atan2(target_x);
    let second_radius = radius * second_radius_scale + 0.25;
    let second_target_angle = target_angle + second_center_angle;
    let second_center = FPoint {
        x: target_x - second_radius * second_target_angle.cos(),
        y: target_y - second_radius * second_target_angle.sin(),
    };
    let signed_sweep = if second_clockwise {
        -second_sweep
    } else {
        second_sweep
    };
    let second_start_angle = second_target_angle - signed_sweep * 0.5;
    let second_end_angle = second_target_angle + signed_sweep * 0.5;

    ArcArcTargets {
        first_start: FPoint { x: -radius, y: 0.0 },
        first_end: FPoint { x: radius, y: 0.0 },
        first_bulge: if upper_arc { -1.0 } else { 1.0 },
        second_start: FPoint {
            x: second_center.x + second_radius * second_start_angle.cos(),
            y: second_center.y + second_radius * second_start_angle.sin(),
        },
        second_end: FPoint {
            x: second_center.x + second_radius * second_end_angle.cos(),
            y: second_center.y + second_radius * second_end_angle.sin(),
        },
        second_bulge: (signed_sweep * 0.25).tan(),
    }
}

fn transformed_target_contour(
    start: FPoint,
    end: FPoint,
    bulge: f64,
    signed_width: f64,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> Contour2 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let len = dx.hypot(dy);
    let normal = FPoint {
        x: -dy / len,
        y: dx / len,
    };
    let route_end = FPoint {
        x: end.x + normal.x * signed_width,
        y: end.y + normal.y * signed_width,
    };
    let route_start = FPoint {
        x: start.x + normal.x * signed_width,
        y: start.y + normal.y * signed_width,
    };

    Contour2::from_bulge_vertices(&[
        transformed_vertex(start, bulge, rotation, tx, ty),
        transformed_vertex(end, 0.0, rotation, tx, ty),
        transformed_vertex(route_end, 0.0, rotation, tx, ty),
        transformed_vertex(route_start, 0.0, rotation, tx, ty),
    ])
    .unwrap()
}

fn transformed_vertex(point: FPoint, bulge: f64, rotation: f64, tx: f64, ty: f64) -> BulgeVertex2 {
    let point = transform_point(point, rotation, tx, ty);
    av(point.x, point.y, bulge)
}

#[derive(Clone, Copy, Debug)]
struct SecantGeometry {
    arc_start: FPoint,
    arc_end: FPoint,
    line_start: FPoint,
    line_end: FPoint,
    line_dir: FPoint,
    bulge: f64,
}

fn rotated_secant_geometry(
    radius: f64,
    arc_fraction: f64,
    upper_arc: bool,
    secant_offset: f64,
    line_len_scale: f64,
    rotation: f64,
    tx: f64,
    ty: f64,
) -> SecantGeometry {
    let local_arc_start = FPoint { x: -radius, y: 0.0 };
    let local_arc_end = FPoint { x: radius, y: 0.0 };
    let target_x = -radius + (2.0 * radius * arc_fraction);
    let target_y = if upper_arc { 1.0 } else { -1.0 }
        * (radius.mul_add(radius, -(target_x * target_x)))
            .max(0.0)
            .sqrt();
    let radial_angle = target_y.atan2(target_x);
    let line_angle = radial_angle + secant_offset;
    let line_len = radius * line_len_scale + 1.0;
    let dir = FPoint {
        x: line_angle.cos(),
        y: line_angle.sin(),
    };
    let local_line_start = FPoint {
        x: target_x - line_len * dir.x,
        y: target_y - line_len * dir.y,
    };
    let local_line_end = FPoint {
        x: target_x + line_len * dir.x,
        y: target_y + line_len * dir.y,
    };

    let rotated_dir = FPoint {
        x: dir.x * rotation.cos() - dir.y * rotation.sin(),
        y: dir.x * rotation.sin() + dir.y * rotation.cos(),
    };

    SecantGeometry {
        arc_start: transform_point(local_arc_start, rotation, tx, ty),
        arc_end: transform_point(local_arc_end, rotation, tx, ty),
        line_start: transform_point(local_line_start, rotation, tx, ty),
        line_end: transform_point(local_line_end, rotation, tx, ty),
        line_dir: rotated_dir,
        bulge: if upper_arc { -1.0 } else { 1.0 },
    }
}

fn route_scale(radius: f64, line_len_scale: f64) -> f64 {
    radius * (line_len_scale + 4.0) + 10.0
}

fn transform_point(point: FPoint, rotation: f64, tx: f64, ty: f64) -> FPoint {
    FPoint {
        x: point.x * rotation.cos() - point.y * rotation.sin() + tx,
        y: point.x * rotation.sin() + point.y * rotation.cos() + ty,
    }
}

fn point_event_on_pair(event: &ContourIntersection, first: usize, second: usize) -> bool {
    matches!(
        event,
        ContourIntersection::Point(point)
            if point.a_segment_index == first && point.b_segment_index == second
    )
}

fn event_on_pair(event: &ContourIntersection, first: usize, second: usize) -> bool {
    match event {
        ContourIntersection::Point(point) => {
            point.a_segment_index == first && point.b_segment_index == second
        }
        ContourIntersection::Overlap(overlap) => {
            overlap.a_segment_index == first && overlap.b_segment_index == second
        }
        ContourIntersection::Uncertain(uncertain) => {
            uncertain.a_segment_index == first && uncertain.b_segment_index == second
        }
    }
}

fn relation_has_reportable_intersection(relation: &SegmentIntersection) -> bool {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None)
        | SegmentIntersection::LineLine(LineLineIntersection::Uncertain { .. }) => false,
        SegmentIntersection::LineLine(
            LineLineIntersection::Point { .. } | LineLineIntersection::Overlap { .. },
        ) => true,
        SegmentIntersection::LineArc { result, .. } => matches!(
            result,
            LineArcIntersection::Point(_) | LineArcIntersection::TwoPoints { .. }
        ),
        SegmentIntersection::ArcArc(ArcArcIntersection::None)
        | SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { .. }) => false,
        SegmentIntersection::ArcArc(
            ArcArcIntersection::Point(_)
            | ArcArcIntersection::TwoPoints { .. }
            | ArcArcIntersection::Overlap { .. },
        ) => true,
    }
}

fn approx_radial_contour(
    vertex_count: usize,
    radii: &[f64],
    bulges: &[f64],
    dx: f64,
    dy: f64,
    angle_shift: f64,
) -> Contour2 {
    let vertices: Vec<_> = (0..vertex_count)
        .map(|index| {
            let angle = angle_shift + index as f64 * std::f64::consts::TAU / vertex_count as f64;
            let radius = 10.0 * radii[index];
            let raw_bulge = bulges[index];
            let bulge = if raw_bulge.abs() < 0.05 {
                if index % 3 == 0 { 0.0 } else { 0.2 }
            } else {
                raw_bulge
            };
            BulgeVertex2::new(
                Point2::new(af(dx + radius * angle.cos()), af(dy + radius * angle.sin())),
                af(bulge),
            )
        })
        .collect();

    Contour2::from_bulge_vertices(&vertices).unwrap()
}

fn assert_arc_approx(segment: &Segment2, start: (f64, f64), end: (f64, f64)) {
    let Segment2::Arc(arc) = segment else {
        panic!("expected approximate arc fragment");
    };
    assert_approx_point(arc.start(), start);
    assert_approx_point(arc.end(), end);
}

fn assert_approx_point(point: &Point2, expected: (f64, f64)) {
    let x = point.x().to_f64_lossy().unwrap();
    let y = point.y().to_f64_lossy().unwrap();
    assert!((x - expected.0).abs() <= 1e-9, "x mismatch: {x}");
    assert!((y - expected.1).abs() <= 1e-9, "y mismatch: {y}");
}
