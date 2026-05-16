use hypercurve::{
    Aabb2, BooleanFragmentAction, BooleanOp, BulgeVertex2, CircularArc2, Classification, Contour2,
    ContourPointLocation, CurvePolicy, CurveString2, FillRule, LineSeg2, OffsetCap, Point2, Real,
    Region2, RegionPointLocation, Segment2,
};
use proptest::prelude::*;

#[derive(Clone, Copy, Debug)]
struct Rect {
    xmin: i32,
    ymin: i32,
    xmax: i32,
    ymax: i32,
}

fn s(value: i32) -> Real {
    value.into()
}

fn sf(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn pf(x: f64, y: f64) -> Point2 {
    Point2::new(sf(x), sf(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn line_segment(x0: i32, y0: i32, x1: i32, y1: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(x0, y0), p(x1, y1)).unwrap())
}

fn rect_contour(rect: Rect) -> Contour2 {
    Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(rect.xmin, rect.ymin, 0),
            vertex(rect.xmax, rect.ymin, 0),
            vertex(rect.xmax, rect.ymax, 0),
            vertex(rect.xmin, rect.ymax, 0),
        ],
        FillRule::NonZero,
    )
    .unwrap()
}

fn rect_region(rect: Rect) -> Region2 {
    Region2::from_material_contours(vec![rect_contour(rect)])
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn rect_strategy() -> impl Strategy<Value = Rect> {
    (-64_i32..64, -64_i32..64, 2_i32..48, 2_i32..48).prop_map(|(xmin, ymin, width, height)| Rect {
        xmin,
        ymin,
        xmax: xmin + width,
        ymax: ymin + height,
    })
}

fn nested_stack_strategy() -> impl Strategy<Value = (i32, i32, i32, i32, usize)> {
    (-32_i32..32, -32_i32..32, 2_i32..9, 4_i32..28, 3_usize..7)
}

fn inside_rect(rect: Rect, x: f64, y: f64) -> bool {
    x > f64::from(rect.xmin)
        && x < f64::from(rect.xmax)
        && y > f64::from(rect.ymin)
        && y < f64::from(rect.ymax)
}

fn expected_bool(in_a: bool, in_b: bool, op: BooleanOp) -> bool {
    match op {
        BooleanOp::Union => in_a || in_b,
        BooleanOp::Intersection => in_a && in_b,
        BooleanOp::Difference => in_a && !in_b,
        BooleanOp::Xor => in_a != in_b,
    }
}

fn assert_point_finite(point: &Point2) {
    assert!(point.x().to_f64_approx().is_some_and(f64::is_finite));
    assert!(point.y().to_f64_approx().is_some_and(f64::is_finite));
}

fn assert_segment_finite(segment: &Segment2) {
    match segment {
        Segment2::Line(line) => {
            assert_point_finite(line.start());
            assert_point_finite(line.end());
        }
        Segment2::Arc(arc) => {
            assert_point_finite(arc.start());
            assert_point_finite(arc.end());
            assert_point_finite(arc.center());
            assert!(
                arc.radius_squared()
                    .to_f64_approx()
                    .is_some_and(f64::is_finite)
            );
        }
    }
}

fn assert_contour_finite(contour: &Contour2) {
    for segment in contour.segments() {
        assert_segment_finite(segment);
    }
}

fn assert_contour_boundary_sets_match(left: &[Contour2], right: &[Contour2]) {
    assert_eq!(left.len(), right.len());
    let mut matched = vec![false; right.len()];
    for contour in left {
        let Some((index, _)) = right.iter().enumerate().find(|(index, candidate)| {
            !matched[*index] && contour.has_same_exact_boundary(candidate)
        }) else {
            panic!("boundary contour set is missing {contour:?}");
        };
        matched[index] = true;
    }
}

fn assert_segment_containment(segment: &Segment2, policy: &CurvePolicy) {
    assert_eq!(
        segment.contains_point(segment.start(), policy),
        Classification::Decided(true)
    );
    assert_eq!(
        segment.contains_point(segment.end(), policy),
        Classification::Decided(true)
    );

    if let Classification::Decided(representative) = segment.representative_point(policy).unwrap() {
        assert_eq!(
            segment.contains_point(&representative, policy),
            Classification::Decided(true)
        );
        if let Classification::Decided(bbox) = Aabb2::from_segment(segment, policy).unwrap() {
            assert_eq!(
                bbox.contains_point(&representative, policy),
                Classification::Decided(true)
            );
        }
    }
}

fn assert_rectangle_oracle(a: Rect, b: Rect, result: &Region2, op: BooleanOp) {
    let mut xs = vec![
        f64::from(a.xmin) - 0.5,
        f64::from(a.xmin) + 0.5,
        f64::from(a.xmax) - 0.5,
        f64::from(a.xmax) + 0.5,
        f64::from(b.xmin) - 0.5,
        f64::from(b.xmin) + 0.5,
        f64::from(b.xmax) - 0.5,
        f64::from(b.xmax) + 0.5,
    ];
    let mut ys = vec![
        f64::from(a.ymin) - 0.5,
        f64::from(a.ymin) + 0.5,
        f64::from(a.ymax) - 0.5,
        f64::from(a.ymax) + 0.5,
        f64::from(b.ymin) - 0.5,
        f64::from(b.ymin) + 0.5,
        f64::from(b.ymax) - 0.5,
        f64::from(b.ymax) + 0.5,
    ];
    xs.sort_by(f64::total_cmp);
    xs.dedup();
    ys.sort_by(f64::total_cmp);
    ys.dedup();

    for x in xs {
        for y in &ys {
            let expected = expected_bool(inside_rect(a, x, *y), inside_rect(b, x, *y), op);
            let actual = result.classify_point(&pf(x, *y), &policy());
            if let Classification::Decided(location) = actual {
                assert_eq!(
                    matches!(location, RegionPointLocation::Inside),
                    expected,
                    "rectangle boolean oracle mismatch at ({x}, {y}) for {op:?}"
                );
            }
        }
    }
}

fn bool_ops() -> [BooleanOp; 4] {
    [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ]
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        max_shrink_iters: 512,
        ..ProptestConfig::default()
    })]

    #[test]
    fn prepared_line_segment_classifier_matches_plain_for_integer_grid(
        x0 in -64_i32..64,
        y0 in -64_i32..64,
        dx in -16_i32..16,
        dy in -16_i32..16,
        qx in -96_i32..96,
        qy in -96_i32..96,
    ) {
        prop_assume!(dx != 0 || dy != 0);
        let line = LineSeg2::try_new(p(x0, y0), p(x0 + dx, y0 + dy)).unwrap();
        let query = p(qx, qy);
        let prepared = line.prepare_topology_queries();

        // Fuzz the curve/predicate boundary rather than the determinant
        // itself: Yap's EGC model requires prepared objects to preserve the
        // same exact branch decisions as unprepared objects while carrying
        // extra structure for cheaper dispatch.
        prop_assert_eq!(
            prepared.classify_point(&query, &policy()),
            line.classify_point(&query, &policy())
        );
    }

    #[test]
    fn prepared_circular_arc_classifiers_match_plain_for_integer_grid(
        x in -48_i32..48,
        y in -48_i32..48,
        radius in 1_i32..32,
        clockwise in any::<bool>(),
        qx in -96_i32..96,
        qy in -96_i32..96,
    ) {
        let arc = CircularArc2::from_bulge(
            p(x - radius, y),
            p(x + radius, y),
            s(if clockwise { -1 } else { 1 }),
        ).unwrap();
        let query = p(qx, qy);
        let prepared = arc.prepare_topology_queries();

        // The prepared arc caches radial line predicates and scalar facts, but
        // the radius and sweep decisions must stay identical to the ordinary
        // exact classifiers across adversarial integer-grid queries.
        prop_assert_eq!(
            prepared.contains_sweep_point(&query, &policy()),
            arc.contains_sweep_point(&query, &policy())
        );
        prop_assert_eq!(
            prepared.contains_point(&query, &policy()),
            arc.contains_point(&query, &policy())
        );
    }

    #[test]
    fn primitive_segments_reversal_containment_and_bbox(
        x in -128_i32..128,
        y in -128_i32..128,
        dx in -64_i32..64,
        dy in -64_i32..64,
        radius in 1_i32..96,
        clockwise in any::<bool>(),
    ) {
        let dx = if dx == 0 && dy == 0 { 1 } else { dx };
        let policy = policy();
        let line = Segment2::Line(LineSeg2::try_new(p(x, y), p(x + dx, y + dy)).unwrap());
        let arc = Segment2::Arc(CircularArc2::from_bulge(
            p(x - radius, y),
            p(x + radius, y),
            s(if clockwise { -1 } else { 1 }),
        ).unwrap());

        for segment in [line, arc] {
            assert_segment_finite(&segment);
            assert_segment_containment(&segment, &policy);
            let reversed = segment.reversed();
            assert_segment_containment(&reversed, &policy);
            prop_assert_eq!(reversed.reversed(), segment);
        }
    }

    #[test]
    fn prepared_plain_boolean_fragments_and_oracles(a_rect in rect_strategy(), b_rect in rect_strategy()) {
        let policy = policy();
        let fill_rule = FillRule::NonZero;
        let a_contour = rect_contour(a_rect);
        let b_contour = rect_contour(b_rect);
        let a_region = rect_region(a_rect);
        let b_region = rect_region(b_rect);
        let prepared_a_contour = a_contour.prepare_topology_queries(&policy);
        let prepared_b_contour = b_contour.prepare_topology_queries(&policy);
        let prepared_a_region = a_region.prepare_topology_queries(&policy);
        let prepared_b_region = b_region.prepare_topology_queries(&policy);

        let contour_events = a_contour.intersect_contour(&b_contour, &policy).unwrap();
        prop_assert_eq!(
            prepared_a_contour.intersect_prepared_contour(&prepared_b_contour, &policy).unwrap(),
            contour_events.clone()
        );
        prop_assert_eq!(
            prepared_a_contour.intersect_contour(&b_contour, &policy).unwrap(),
            contour_events
        );
        prop_assert_eq!(prepared_a_contour.intersect_self(&policy).unwrap(), a_contour.intersect_self(&policy).unwrap());

        for sample in [
            pf(f64::from(a_rect.xmin) + 0.5, f64::from(a_rect.ymin) + 0.5),
            pf(f64::from(a_rect.xmax) + 0.5, f64::from(a_rect.ymax) + 0.5),
            p(a_rect.xmin, a_rect.ymin),
        ] {
            prop_assert_eq!(
                prepared_a_contour.classify_point(&sample, &policy),
                a_contour.classify_point(&sample, &policy)
            );
            prop_assert_eq!(
                prepared_a_contour.point_on_boundary(&sample, &policy),
                a_contour.point_on_boundary(&sample, &policy)
            );
            prop_assert_eq!(
                prepared_a_contour.winding_number(&sample, &policy),
                a_contour.winding_number(&sample, &policy)
            );
            prop_assert_eq!(
                prepared_a_region.classify_point(&sample, &policy),
                a_region.classify_point(&sample, &policy)
            );
            prop_assert_eq!(
                prepared_a_region.signed_depth(&sample, &policy),
                a_region.signed_depth(&sample, &policy)
            );
        }

        let region_events = a_region.intersect_region(&b_region, &policy).unwrap();
        prop_assert_eq!(
            prepared_a_region.intersect_prepared_region(&prepared_b_region, &policy).unwrap(),
            region_events.clone()
        );
        prop_assert_eq!(
            prepared_a_region.intersect_region(&b_region.as_view(), &policy).unwrap(),
            region_events.clone()
        );
        prop_assert_eq!(
            a_region.as_view().intersect_prepared_region(&prepared_b_region, &policy).unwrap(),
            region_events.clone()
        );

        if let Classification::Decided(fragments) =
            region_events.split_regions(&a_region.as_view(), &b_region.as_view(), &policy).unwrap()
        {
            for contour_fragments in fragments.contours() {
                for fragment in contour_fragments.fragments.fragments() {
                    assert_segment_finite(&fragment.segment);
                }
            }

            for op in bool_ops() {
                if let Classification::Decided(selection) = fragments
                    .classify_for_boolean(&a_region.as_view(), &b_region.as_view(), op, &policy)
                    .unwrap()
                {
                    let emitted = selection.emit_boundary_fragments(&fragments).unwrap();
                    let selected = selection
                        .classifications()
                        .iter()
                        .filter(|classification| {
                            classification.action.emits_fragment()
                                || classification.action == BooleanFragmentAction::BoundaryNeedsResolution
                        })
                        .count();
                    prop_assert_eq!(emitted.directed_len() + emitted.unresolved_len(), selected);
                }
            }
        }

        for op in bool_ops() {
            let plain_loops = a_region.boolean_boundary_loops(&b_region, op, &policy).unwrap();
            prop_assert_eq!(
                prepared_a_region.boolean_boundary_loops(&prepared_b_region, op, &policy).unwrap(),
                plain_loops.clone()
            );
            prop_assert_eq!(
                prepared_a_region.boolean_boundary_loops_against_region(&b_region.as_view(), op, &policy).unwrap(),
                plain_loops.clone()
            );
            prop_assert_eq!(
                a_region.as_view().boolean_boundary_loops_against_prepared_region(&prepared_b_region, op, &policy).unwrap(),
                plain_loops.clone()
            );

            let plain_contours = a_region.boolean_boundary_contours(&b_region, op, fill_rule, &policy).unwrap();
            prop_assert_eq!(
                prepared_a_region.boolean_boundary_contours(&prepared_b_region, op, fill_rule, &policy).unwrap(),
                plain_contours.clone()
            );
            prop_assert_eq!(
                prepared_a_region.boolean_boundary_contours_against_region(&b_region.as_view(), op, fill_rule, &policy).unwrap(),
                plain_contours.clone()
            );
            prop_assert_eq!(
                a_region.as_view().boolean_boundary_contours_against_prepared_region(&prepared_b_region, op, fill_rule, &policy).unwrap(),
                plain_contours.clone()
            );

            if let (Classification::Decided(loops), Classification::Decided(contours)) =
                (&plain_loops, &plain_contours)
            {
                assert_contour_boundary_sets_match(&loops.to_contours(fill_rule).unwrap(), contours);
            }

            let plain_region = a_region.boolean_region(&b_region, op, fill_rule, &policy).unwrap();
            prop_assert_eq!(
                prepared_a_region.boolean_region(&prepared_b_region, op, fill_rule, &policy).unwrap(),
                plain_region.clone()
            );
            prop_assert_eq!(
                prepared_a_region.boolean_region_against_region(&b_region.as_view(), op, fill_rule, &policy).unwrap(),
                plain_region.clone()
            );
            prop_assert_eq!(
                a_region.as_view().boolean_region_against_prepared_region(&prepared_b_region, op, fill_rule, &policy).unwrap(),
                plain_region.clone()
            );

            if let Classification::Decided(result) = &plain_region {
                assert_rectangle_oracle(a_rect, b_rect, result, op);
            }
            if let (Classification::Decided(contours), Classification::Decided(region)) =
                (&plain_contours, &plain_region)
            {
                if let Classification::Decided(rebuilt) =
                    Region2::from_boundary_contours(contours.clone(), &policy).unwrap()
                {
                    prop_assert_eq!(rebuilt, region.clone());
                }
            }
        }
    }

    #[test]
    fn nested_boundary_contours_assign_material_and_hole_bins(
        (x, y, step, inner, depth) in nested_stack_strategy()
    ) {
        let policy = policy();
        let outer_size = inner + 2 * step * (depth as i32 - 1);
        let rects: Vec<_> = (0..depth)
            .map(|index| {
                let inset = step * index as i32;
                Rect {
                    xmin: x + inset,
                    ymin: y + inset,
                    xmax: x + outer_size - inset,
                    ymax: y + outer_size - inset}
            })
            .collect();
        let contours: Vec<_> = rects.iter().copied().map(rect_contour).collect();
        let Classification::Decided(region) = Region2::from_boundary_contours(contours, &policy).unwrap() else {
            panic!("strict nested rectangles should classify nesting");
        };
        let prepared = region.prepare_topology_queries(&policy);

        prop_assert_eq!(region.material_contours().len(), (depth + 1) / 2);
        prop_assert_eq!(region.hole_contours().len(), depth / 2);

        for (index, rect) in rects.iter().enumerate() {
            let sample = pf(f64::from(rect.xmin) + 0.5, f64::from(rect.ymin) + 0.5);
            let expected = if index % 2 == 0 {
                RegionPointLocation::Inside
            } else {
                RegionPointLocation::Outside
            };
            prop_assert_eq!(region.classify_point(&sample, &policy), Classification::Decided(expected));
            prop_assert_eq!(prepared.classify_point(&sample, &policy), Classification::Decided(expected));
        }
    }

    #[test]
    fn offset_outlines_and_closed_offsets_cover_cap_styles(
        horizontal in 6_i32..96,
        vertical in 6_i32..96,
        raw_distance in 1_i32..32,
        radius in 4_i32..96,
    ) {
        let policy = policy();
        let distance = raw_distance.min(horizontal.min(vertical) / 3).max(1);
        let curve = CurveString2::try_new(vec![
            line_segment(0, 0, horizontal, 0),
            line_segment(horizontal, 0, horizontal, vertical),
        ]).unwrap();

        for cap in [OffsetCap::Round, OffsetCap::Butt, OffsetCap::Square] {
            let dispatched = curve.offset_outline(s(distance), cap, &policy).unwrap();
            let direct = match cap {
                OffsetCap::Round => curve.offset_outline_round_caps(s(distance), &policy),
                OffsetCap::Butt => curve.offset_outline_butt_caps(s(distance), &policy),
                OffsetCap::Square => curve.offset_outline_square_caps(s(distance), &policy)}.unwrap();
            prop_assert_eq!(&dispatched, &direct);
            let Classification::Decided(outline) = dispatched else {
                panic!("simple L-path outline should decide for {cap:?}");
            };
            assert_contour_finite(&outline);
            prop_assert_eq!(
                outline.has_self_contacts(&policy).unwrap(),
                Classification::Decided(false)
            );
        }

        let arc_distance = raw_distance.min(radius - 1).max(1);
        let arc_curve = CurveString2::try_new(vec![Segment2::Arc(
            CircularArc2::from_bulge(p(-radius, 0), p(radius, 0), s(-1)).unwrap()
        )]).unwrap();
        for cap in [OffsetCap::Round, OffsetCap::Butt, OffsetCap::Square] {
            if let Classification::Decided(outline) = arc_curve.offset_outline(s(arc_distance), cap, &policy).unwrap() {
                assert_contour_finite(&outline);
            }
        }

        let rect = rect_contour(Rect {
            xmin: 0,
            ymin: 0,
            xmax: horizontal,
            ymax: vertical});
        let Classification::Decided(offset) = rect.offset_left_checked(s(distance), &policy).unwrap() else {
            panic!("simple rectangle inward offset should decide");
        };
        assert_contour_finite(&offset);
        prop_assert_eq!(
            offset.has_self_contacts(&policy).unwrap(),
            Classification::Decided(false)
        );
        prop_assert_eq!(
            offset.classify_point(&pf(
                f64::from(horizontal) * 0.5,
                f64::from(vertical) * 0.5,
            ), &policy),
            Classification::Decided(ContourPointLocation::Inside)
        );
    }
}
