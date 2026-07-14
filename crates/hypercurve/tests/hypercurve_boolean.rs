use hypercurve::{
    Aabb2, BooleanBoundaryChain, BooleanBoundaryChainAssemblyStage2, BooleanBoundaryChainSet,
    BooleanBoundaryContourTransferStage2, BooleanBoundaryFragmentEmissionStage2,
    BooleanBoundaryFragmentSet, BooleanBoundaryLoop, BooleanBoundaryLoopConstructionStage2,
    BooleanBoundaryLoopExtractionStage2, BooleanBoundaryLoopSet, BooleanFragmentAction,
    BooleanFragmentClassification, BooleanFragmentSelection, BooleanFragmentSelectionStage2,
    BooleanOp, BulgeVertex2, Classification, Contour2, ContourFragmentSet, ContourOperand,
    ContourSplitMarkers, CurveError, CurvePolicy, DirectedBooleanFragment, FillRule,
    LineLineIntersection, LineSeg2, ParamRange, Real, Region2, RegionContourKey, RegionContourRole,
    RegionPointLocation, RegionSide, Segment2, SegmentIntersection, SegmentKind, SegmentKindCounts,
    UncertaintyReason,
};

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

fn center_defined_circle(radius: i32, clockwise: bool) -> Contour2 {
    Contour2::try_new(vec![
        Segment2::Arc(
            hypercurve::CircularArc2::try_from_center(
                p(radius, 0),
                p(-radius, 0),
                p(0, 0),
                clockwise,
            )
            .unwrap(),
        ),
        Segment2::Arc(
            hypercurve::CircularArc2::try_from_center(
                p(-radius, 0),
                p(radius, 0),
                p(0, 0),
                clockwise,
            )
            .unwrap(),
        ),
    ])
    .unwrap()
}

fn major_arc_segment_contour(radius: i32) -> Contour2 {
    let start = p(radius, 0);
    let end = p(0, radius);
    Contour2::try_new(vec![
        Segment2::Arc(
            hypercurve::CircularArc2::try_from_center(start.clone(), end.clone(), p(0, 0), true)
                .unwrap(),
        ),
        Segment2::Line(LineSeg2::try_new(end, start).unwrap()),
    ])
    .unwrap()
}

fn triangle(vertices: [(i32, i32); 3]) -> Contour2 {
    contour(
        &vertices
            .map(|(x, y)| vertex(x, y, 0))
            .into_iter()
            .collect::<Vec<_>>(),
    )
}

fn boolean_truth(op: BooleanOp, first: bool, second: bool) -> bool {
    match op {
        BooleanOp::Union => first || second,
        BooleanOp::Intersection => first && second,
        BooleanOp::Difference => first && !second,
        BooleanOp::Xor => first ^ second,
    }
}

fn assert_exact_boolean_matrix(
    first: &Region2,
    second: &Region2,
    samples: &[(hypercurve::Point2, bool, bool)],
) {
    let policy = policy();
    let prepared_first = first.prepare_topology_queries(&policy);
    let prepared_second = second.prepare_topology_queries(&policy);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let direct = first
            .boolean_region_with_report(second, op, FillRule::NonZero, &policy)
            .unwrap();
        let Classification::Decided(direct_region) = direct.region_classification() else {
            panic!("direct {op:?} was {direct:#?}");
        };
        let prepared = prepared_first
            .boolean_region_with_report(&prepared_second, op, FillRule::NonZero, &policy)
            .unwrap();
        let Classification::Decided(prepared_region) = prepared.region_classification() else {
            panic!("prepared {op:?} was {prepared:#?}");
        };

        assert_eq!(prepared_region, direct_region, "prepared {op:?}");
        for (point, first_inside, second_inside) in samples {
            let expected = if boolean_truth(op, *first_inside, *second_inside) {
                RegionPointLocation::Inside
            } else {
                RegionPointLocation::Outside
            };
            assert_eq!(
                direct_region.classify_point(point, &policy),
                Classification::Decided(expected),
                "direct {op:?} at {point:?}"
            );
            assert_eq!(
                prepared_region.classify_point(point, &policy),
                Classification::Decided(expected),
                "prepared {op:?} at {point:?}"
            );
        }
    }
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn symbolic_regular_polygon(radius: Real, sides: usize) -> Contour2 {
    symbolic_regular_polygon_at(radius, sides, Real::zero(), Real::zero())
}

fn symbolic_regular_polygon_at(
    radius: Real,
    sides: usize,
    center_x: Real,
    center_y: Real,
) -> Contour2 {
    let points = (0..sides)
        .map(|index| {
            let fraction = (Real::from(index as u64) / Real::from(sides as u64)).unwrap();
            let angle = Real::tau() * fraction;
            [
                center_x.clone() + radius.clone() * angle.clone().cos(),
                center_y.clone() + radius.clone() * angle.sin(),
            ]
        })
        .collect::<Vec<_>>();
    Contour2::from_real_ring(&points).unwrap()
}

#[test]
fn transcendental_point_equality_requires_exact_coordinate_evidence() {
    let seventh = (Real::pi() / Real::from(7_u8)).unwrap();
    let fifth = (Real::pi() / Real::from(5_u8)).unwrap();
    let point = hypercurve::Point2::new(seventh.clone().sin(), seventh.cos());
    let clone = point.clone();
    let reconstructed = hypercurve::Point2::new(
        (Real::pi() / Real::from(7_u8)).unwrap().sin(),
        (Real::pi() / Real::from(7_u8)).unwrap().cos(),
    );
    let distinct = hypercurve::Point2::new(fifth.clone().sin(), fifth.cos());

    assert_eq!(point, clone);
    assert_eq!(point, reconstructed);
    assert_ne!(point, distinct);
}

#[test]
fn symbolic_regular_polygon_booleans_with_rectangle_are_decided() {
    let circle =
        Region2::from_material_contours(vec![symbolic_regular_polygon(Real::from(2_u8), 7)]);
    let rectangle = Region2::from_material_contours(vec![rectangle(0, -1, 3, 1)]);

    let intersections = circle.intersect_region(&rectangle, &policy()).unwrap();
    let pair = &intersections.pairs()[0];
    let source = circle.material_contours()[0].clone();
    let markers = ContourSplitMarkers::from_intersections(
        &source,
        pair.intersections(),
        ContourOperand::First,
        &policy(),
    );
    assert!(
        matches!(markers, Classification::Decided(_)),
        "symbolic markers were {markers:#?}"
    );
    let Classification::Decided(markers) = markers else {
        unreachable!()
    };
    let split = ContourFragmentSet::from_split_markers(&source, &markers, &policy()).unwrap();
    assert!(
        matches!(split, Classification::Decided(_)),
        "symbolic fragment split was {split:#?}"
    );
    let fragments = intersections
        .split_regions_with_report(&circle.as_view(), &rectangle.as_view(), &policy())
        .unwrap();
    assert!(
        matches!(
            fragments.fragments_classification(),
            Classification::Decided(_)
        ),
        "symbolic split was {fragments:#?}"
    );
    let Classification::Decided(fragment_set) = fragments.fragments_classification() else {
        unreachable!()
    };
    let selection = fragment_set
        .classify_for_boolean_with_report(
            &circle.as_view(),
            &rectangle.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap();
    assert!(
        matches!(
            selection.selection_classification(),
            Classification::Decided(_)
        ),
        "symbolic selection was {selection:#?}"
    );
    let selection_value = selection.selection().unwrap();
    let emission = selection_value
        .emit_boundary_fragments_with_report(fragment_set)
        .unwrap();
    assert!(
        emission.fragments().is_some(),
        "symbolic emission was {emission:#?}"
    );
    let chains = emission
        .fragments()
        .unwrap()
        .assemble_chains_with_report(&policy());
    assert!(
        chains.chains().is_some(),
        "symbolic chains were {chains:#?}"
    );
    let loops = chains.chains().unwrap().closed_loops_with_report();
    assert!(loops.loops().is_some(), "symbolic loops were {loops:#?}");
    let contours = loops
        .loops()
        .unwrap()
        .to_contours_with_report(FillRule::NonZero);
    assert!(
        contours.contours().is_some(),
        "symbolic contours were {contours:#?}"
    );

    let report = circle
        .boolean_region_with_report(&rectangle, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap();
    let result = report.region_classification();
    assert!(
        matches!(result, Classification::Decided(region) if !region.is_empty()),
        "symbolic union was {report:#?}"
    );

    for op in [
        BooleanOp::Difference,
        BooleanOp::Intersection,
        BooleanOp::Xor,
    ] {
        let result = circle
            .boolean_region(&rectangle, op, FillRule::NonZero, &policy())
            .unwrap();
        assert!(
            matches!(result, Classification::Decided(ref region) if !region.is_empty()),
            "symbolic {op:?} was {result:#?}"
        );
    }
}

#[test]
fn concentric_symbolic_polygon_offsets_form_decided_ring() {
    let source = symbolic_regular_polygon(Real::from(15_u8), 40);
    let Classification::Decided(outer) =
        source.offset_left_checked(-Real::one(), &policy()).unwrap()
    else {
        panic!("outward exact offset was uncertain");
    };
    let Classification::Decided(inner) =
        source.offset_left_checked(Real::one(), &policy()).unwrap()
    else {
        panic!("inward exact offset was uncertain");
    };
    let outer = Region2::from_material_contours(vec![outer]);
    let inner = Region2::from_material_contours(vec![inner]);

    let result = outer
        .boolean_region_with_report(&inner, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap();
    assert!(
        matches!(result.region_classification(), Classification::Decided(region) if region.material_contours().len() == 1 && region.hole_contours().len() == 1),
        "concentric exact offset difference was {result:#?}"
    );

    let prepared_outer = outer.prepare_topology_queries(&policy());
    let prepared_inner = inner.prepare_topology_queries(&policy());
    let prepared_result = prepared_outer
        .boolean_region_with_report(
            &prepared_inner,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert!(
        matches!(prepared_result.region_classification(), Classification::Decided(region) if region.material_contours().len() == 1 && region.hole_contours().len() == 1),
        "prepared concentric exact offset difference was {prepared_result:#?}"
    );
}

#[test]
fn retained_offset_containment_stops_at_line_direction_reversal() {
    let source = rectangle(0, 0, 2, 2);
    let near_distance = (Real::one() / Real::from(4_u8)).unwrap();
    let beyond_collapse_distance = (Real::from(7_u8) / Real::from(4_u8)).unwrap();
    let Classification::Decided(near) = source
        .offset_left_checked(near_distance, &policy())
        .unwrap()
    else {
        panic!("near square inset was uncertain");
    };
    let Classification::Decided(beyond_collapse) = source
        .offset_left_checked(beyond_collapse_distance, &policy())
        .unwrap()
    else {
        panic!("over-inset square was uncertain");
    };
    let near = Region2::from_material_contours(vec![near]);
    let beyond_collapse = Region2::from_material_contours(vec![beyond_collapse]);

    let result = near
        .boolean_region_with_report(
            &beyond_collapse,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert!(
        matches!(result.region_classification(), Classification::Decided(region) if region.is_empty()),
        "symmetric pre/post-collapse insets were not recognized as equal: {result:#?}"
    );
}

#[test]
fn nested_symbolic_regular_polygon_difference_is_decided() {
    let outer =
        Region2::from_material_contours(vec![symbolic_regular_polygon(Real::from(2_u8), 24)]);
    let inner = Region2::from_material_contours(vec![symbolic_regular_polygon(
        (Real::from(3_u8) / Real::from(2_u8)).unwrap(),
        24,
    )]);

    let report = outer
        .boolean_region_with_report(&inner, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap();
    assert!(
        matches!(report.region_classification(), Classification::Decided(region) if !region.is_empty()),
        "nested symbolic difference was {report:#?}"
    );
}

#[test]
fn symbolic_regular_polygon_keyway_difference_is_decided() {
    let circle =
        Region2::from_material_contours(vec![symbolic_regular_polygon(Real::from(3_u8), 24)]);
    let keyway = Region2::from_material_contours(vec![
        Contour2::from_real_ring(&[
            [
                Real::from(2_u8),
                (Real::from(-1_i8) / Real::from(2_u8)).unwrap(),
            ],
            [
                Real::from(3_u8),
                (Real::from(-1_i8) / Real::from(2_u8)).unwrap(),
            ],
            [Real::from(3_u8), (Real::one() / Real::from(2_u8)).unwrap()],
            [Real::from(2_u8), (Real::one() / Real::from(2_u8)).unwrap()],
        ])
        .unwrap(),
    ]);

    let report = circle
        .boolean_region_with_report(&keyway, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap();
    assert!(
        matches!(report.region_classification(), Classification::Decided(region) if !region.is_empty()),
        "symbolic keyway difference was {report:#?}"
    );
}

#[test]
fn symbolic_regular_polygon_two_cut_differences_are_decided() {
    let circle =
        Region2::from_material_contours(vec![symbolic_regular_polygon(Real::from(3_u8), 24)]);
    let top = Region2::from_material_contours(vec![rectangle(-3, 1, 3, 3)]);
    let bottom = Region2::from_material_contours(vec![rectangle(-3, -3, 3, -1)]);

    let Classification::Decided(first) = circle
        .boolean_region(&top, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("first symbolic cut was uncertain");
    };
    let report = first
        .boolean_region_with_report(&bottom, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap();
    assert!(
        matches!(report.region_classification(), Classification::Decided(region) if !region.is_empty()),
        "second symbolic cut was {report:#?}"
    );
}

#[test]
fn translated_symbolic_regular_polygon_intersections_are_decided() {
    let center_radius = Real::from(3_u8).sqrt().unwrap();
    let centers = (0..3)
        .map(|index| {
            let angle = Real::tau() * (Real::from(index as u64) / Real::from(3_u8)).unwrap();
            (
                center_radius.clone() * angle.clone().cos(),
                center_radius.clone() * angle.sin(),
            )
        })
        .collect::<Vec<_>>();
    let regions = centers
        .into_iter()
        .map(|(x, y)| {
            Region2::from_material_contours(vec![symbolic_regular_polygon_at(
                Real::from(3_u8),
                32,
                x,
                y,
            )])
        })
        .collect::<Vec<_>>();

    let intersections = regions[0].intersect_region(&regions[1], &policy()).unwrap();
    assert!(
        intersections.pairs().iter().all(|pair| {
            pair.intersections()
                .events()
                .iter()
                .all(|event| !matches!(event, hypercurve::ContourIntersection::Uncertain(_)))
        }),
        "translated symbolic intersections were {intersections:#?}"
    );

    let first_report = regions[0]
        .boolean_region_with_report(
            &regions[1],
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    let Classification::Decided(first) = first_report.region_classification() else {
        panic!("first translated symbolic intersection was {first_report:#?}");
    };
    assert!(!first.is_empty());
    let report = first
        .boolean_region_with_report(
            &regions[2],
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert!(
        matches!(report.region_classification(), Classification::Decided(region) if !region.is_empty()),
        "second translated symbolic intersection was {report:#?}"
    );
}

#[test]
fn translated_symbolic_regular_polygon_boolean_matrix_is_decided() {
    let shift_angle = (Real::pi() / Real::from(5_u8)).unwrap();
    let first =
        Region2::from_material_contours(vec![symbolic_regular_polygon(Real::from(2_u8), 12)]);
    let second = Region2::from_material_contours(vec![symbolic_regular_polygon_at(
        Real::from(2_u8),
        12,
        shift_angle.clone().sin(),
        shift_angle.cos(),
    )]);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let report = first
            .boolean_region_with_report(&second, op, FillRule::NonZero, &policy())
            .unwrap();
        assert!(
            matches!(report.region_classification(), Classification::Decided(region) if !region.is_empty()),
            "translated symbolic {op:?} was {report:#?}"
        );
    }
}

#[test]
fn translated_symbolic_circle_rectangle_union_is_decided() {
    let rectangle = Region2::from_material_contours(vec![rectangle(0, 0, 2, 1)]);
    let circle = Region2::from_material_contours(vec![symbolic_regular_polygon_at(
        (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
        32,
        Real::one(),
        (Real::one() / Real::from(2_u8)).unwrap(),
    )]);
    let crossing = rectangle.material_contours()[0].segments()[0]
        .intersect_segment(&circle.material_contours()[0].segments()[19], &policy())
        .unwrap();
    assert!(
        matches!(
            crossing,
            SegmentIntersection::LineLine(LineLineIntersection::Point { .. })
        ),
        "symbolic crossing was {crossing:#?}"
    );
    let Classification::Decided(rectangle_box) = Aabb2::from_region(&rectangle, &policy()).unwrap()
    else {
        panic!("rectangle bounds were uncertain")
    };
    let Classification::Decided(circle_box) = Aabb2::from_region(&circle, &policy()).unwrap()
    else {
        panic!("circle bounds were uncertain")
    };
    assert_eq!(
        rectangle_box.overlaps(&circle_box, &policy()),
        Classification::Decided(true)
    );
    let Classification::Decided(rectangle_edge_box) =
        Aabb2::from_segment(&rectangle.material_contours()[0].segments()[0], &policy()).unwrap()
    else {
        panic!("rectangle edge bounds were uncertain")
    };
    let Classification::Decided(circle_edge_box) =
        Aabb2::from_segment(&circle.material_contours()[0].segments()[19], &policy()).unwrap()
    else {
        panic!("circle edge bounds were uncertain")
    };
    assert_eq!(
        circle_edge_box.has_valid_ordering(&policy()),
        Classification::Decided(true),
        "circle edge box was {circle_edge_box:#?}"
    );
    assert_eq!(
        rectangle_edge_box.overlaps(&circle_edge_box, &policy()),
        Classification::Decided(true),
        "rectangle={:?}, circle={:?}",
        [
            rectangle_edge_box.min_x().to_f64_lossy(),
            rectangle_edge_box.min_y().to_f64_lossy(),
            rectangle_edge_box.max_x().to_f64_lossy(),
            rectangle_edge_box.max_y().to_f64_lossy(),
        ],
        [
            circle_edge_box.min_x().to_f64_lossy(),
            circle_edge_box.min_y().to_f64_lossy(),
            circle_edge_box.max_x().to_f64_lossy(),
            circle_edge_box.max_y().to_f64_lossy(),
        ]
    );
    let events = rectangle.intersect_region(&circle, &policy()).unwrap();
    assert_eq!(events.pairs().len(), 1);
    assert_eq!(events.pairs()[0].intersections().events().len(), 4);

    let report = rectangle
        .boolean_region_with_report(&circle, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap();
    assert!(
        matches!(report.region_classification(), Classification::Decided(region) if !region.is_empty()),
        "translated symbolic circle/rectangle union was {report:#?}"
    );
}

#[test]
fn five_translated_symbolic_regular_polygon_intersections_are_decided() {
    let center_radius = (Real::from(3_u8)
        / (Real::from(2_u8) * (Real::pi() / Real::from(10_u8)).unwrap().cos()))
    .unwrap();
    let regions = (0..5)
        .map(|index| {
            let angle = Real::tau() * (Real::from(index as u64) / Real::from(5_u8)).unwrap();
            Region2::from_material_contours(vec![symbolic_regular_polygon_at(
                Real::from(3_u8),
                32,
                center_radius.clone() * angle.clone().cos(),
                center_radius.clone() * angle.sin(),
            )])
        })
        .collect::<Vec<_>>();

    let mut intersection = regions[0].clone();
    for (index, region) in regions.iter().enumerate().skip(1) {
        let report = intersection
            .boolean_region_with_report(
                region,
                BooleanOp::Intersection,
                FillRule::NonZero,
                &policy(),
            )
            .unwrap();
        let Classification::Decided(next) = report.region_classification() else {
            let events = intersection.intersect_region(region, &policy()).unwrap();
            let split = events
                .split_regions_with_report(&intersection.as_view(), &region.as_view(), &policy())
                .unwrap();
            let selection = split
                .fragments()
                .unwrap()
                .classify_for_boolean_with_report(
                    &intersection.as_view(),
                    &region.as_view(),
                    BooleanOp::Intersection,
                    &policy(),
                )
                .unwrap();
            let emission = selection
                .selection()
                .unwrap()
                .emit_boundary_fragments_with_report(split.fragments().unwrap())
                .unwrap();
            let Some(emitted) = emission.fragments() else {
                panic!("symbolic emission failed: {:#?}", emission.report());
            };
            let chains = emitted.assemble_chains_with_report(&policy());
            let Some(chain_set) = chains.chains() else {
                panic!("symbolic chain assembly failed: {:#?}", chains.report());
            };
            let loops = chain_set.closed_loops_with_report();
            let Some(loop_set) = loops.loops() else {
                panic!("symbolic loop extraction failed: {:#?}", loops.report());
            };
            let contours = loop_set.to_contours_with_report(FillRule::NonZero);
            panic!(
                "translated symbolic intersection {index} was {report:#?}\ncontours: {:#?}",
                contours.report(),
            );
        };
        assert!(
            !next.is_empty(),
            "translated symbolic intersection {index} was empty"
        );
        intersection = next.clone();
    }
}

fn line_segment(x0: i32, y0: i32, x1: i32, y1: i32) -> Segment2 {
    Segment2::Line(hypercurve::LineSeg2::try_new(p(x0, y0), p(x1, y1)).unwrap())
}

fn assert_topology_error<T>(result: hypercurve::CurveResult<T>) {
    match result {
        Err(CurveError::Topology(_)) => {}
        Ok(_) => panic!("expected topology error"),
        Err(error) => panic!("expected topology error, got {error:?}"),
    }
}

fn directed_fragment(
    fragment_index: usize,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
) -> DirectedBooleanFragment {
    let segment = line_segment(x0, y0, x1, y1);
    DirectedBooleanFragment {
        key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
        fragment_index,
        source_segment_index: fragment_index,
        source_segment_start_point: segment.start().clone(),
        source_segment_end_point: segment.end().clone(),
        source_range: ParamRange::new(s(0), s(1)),
        reversed: false,
        segment,
    }
}

fn open_chain_fragments() -> Vec<DirectedBooleanFragment> {
    vec![
        directed_fragment(0, 0, 0, 1, 0),
        directed_fragment(1, 1, 0, 2, 0),
    ]
}

fn triangle_loop_fragments(
    fragment_indices: [usize; 3],
    x: i32,
    y: i32,
) -> Vec<DirectedBooleanFragment> {
    vec![
        directed_fragment(fragment_indices[0], x, y, x + 1, y),
        directed_fragment(fragment_indices[1], x + 1, y, x, y + 1),
        directed_fragment(fragment_indices[2], x, y + 1, x, y),
    ]
}

fn fragment_classification(
    fragment_index: usize,
    action: BooleanFragmentAction,
) -> BooleanFragmentClassification {
    fragment_classification_with_location(fragment_index, RegionPointLocation::Outside, action)
}

fn fragment_classification_with_location(
    fragment_index: usize,
    opposite_location: RegionPointLocation,
    action: BooleanFragmentAction,
) -> BooleanFragmentClassification {
    BooleanFragmentClassification {
        key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
        fragment_index,
        opposite_location,
        source_filled_side_is_left: true,
        action,
    }
}

fn unresolved_boundary_classification(fragment_index: usize) -> BooleanFragmentClassification {
    fragment_classification_with_location(
        fragment_index,
        RegionPointLocation::Boundary,
        BooleanFragmentAction::BoundaryNeedsResolution,
    )
}

fn overlapping_fragments() -> (Region2, Region2, hypercurve::RegionFragmentSet) {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
    let fragment_result = intersections
        .split_regions_with_report(&first.as_view(), &second.as_view(), &policy())
        .unwrap();
    let Classification::Decided(fragments) = fragment_result.fragments_classification() else {
        panic!("expected decided fragments");
    };
    let fragments = fragments.clone();
    let fragment_report = fragment_result.clone().into_report();
    assert_eq!(&fragment_report, fragment_result.report());
    let (owned_fragments, owned_fragment_report) = fragment_result.clone().into_parts();
    assert_eq!(owned_fragments.as_ref(), fragment_result.fragments());
    assert_eq!(&owned_fragment_report, fragment_result.report());
    assert!(matches!(
        fragment_result.into_fragments_classification(),
        Classification::Decided(owned) if owned == fragments
    ));

    (first, second, fragments)
}

#[test]
fn boolean_fragment_selection_classifies_union_and_intersection() {
    let (first, second, fragments) = overlapping_fragments();

    let Classification::Decided(union) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided union selection");
    };
    let Classification::Decided(intersection) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Intersection,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided intersection selection");
    };

    assert!(union.count_action(BooleanFragmentAction::KeepSourceDirection) > 0);
    assert!(intersection.count_action(BooleanFragmentAction::KeepSourceDirection) > 0);
    assert_eq!(
        union.count_action(BooleanFragmentAction::BoundaryNeedsResolution),
        0
    );
    assert_eq!(
        intersection.count_action(BooleanFragmentAction::BoundaryNeedsResolution),
        0
    );
    assert_ne!(
        union.count_action(BooleanFragmentAction::KeepSourceDirection),
        intersection.count_action(BooleanFragmentAction::KeepSourceDirection)
    );
}

#[test]
fn boolean_fragment_selection_reverses_second_operand_for_difference() {
    let (first, second, fragments) = overlapping_fragments();

    let Classification::Decided(difference) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Difference,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided difference selection");
    };

    assert!(difference.count_action(BooleanFragmentAction::KeepSourceDirection) > 0);
    assert!(difference.count_action(BooleanFragmentAction::KeepReversed) > 0);
}

#[test]
fn boolean_fragment_selection_emits_directed_boundary_fragments() {
    let (first, second, fragments) = overlapping_fragments();
    let union_result = fragments
        .classify_for_boolean_with_report(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap();
    assert!(union_result.report().status().is_native_exact());
    assert_eq!(union_result.report().op(), BooleanOp::Union);
    assert_eq!(
        union_result.report().stage(),
        BooleanFragmentSelectionStage2::ActionAssignment
    );
    assert_eq!(
        union_result.report().source_contour_count(),
        fragments.len()
    );
    assert_eq!(union_result.report().source_fragment_count(), 12);
    assert_eq!(
        union_result.report().source_fragment_kind_counts(),
        SegmentKindCounts { lines: 12, arcs: 0 }
    );
    assert_eq!(union_result.report().classified_fragment_count(), Some(12));
    assert_eq!(
        union_result.report().boundary_needs_resolution_count(),
        Some(0)
    );
    assert_eq!(union_result.report().blocker(), None);
    let union = union_result
        .selection()
        .expect("reported union selection should materialize");
    assert_eq!(
        union_result.report().discard_count(),
        Some(union.count_action(BooleanFragmentAction::Discard))
    );
    assert_eq!(
        union_result.report().keep_source_direction_count(),
        Some(union.count_action(BooleanFragmentAction::KeepSourceDirection))
    );
    assert_eq!(
        union_result.report().keep_reversed_count(),
        Some(union.count_action(BooleanFragmentAction::KeepReversed))
    );

    let emitted_result = union
        .emit_boundary_fragments_with_report(&fragments)
        .unwrap();
    assert!(emitted_result.report().status().is_native_exact());
    assert_eq!(
        emitted_result.report().stage(),
        BooleanBoundaryFragmentEmissionStage2::FragmentEmission
    );
    assert_eq!(
        emitted_result.report().source_classification_count(),
        union.len()
    );
    assert_eq!(
        emitted_result.report().discard_count(),
        union.count_action(BooleanFragmentAction::Discard)
    );
    assert_eq!(
        emitted_result.report().keep_source_direction_count(),
        union.count_action(BooleanFragmentAction::KeepSourceDirection)
    );
    assert_eq!(
        emitted_result.report().keep_reversed_count(),
        union.count_action(BooleanFragmentAction::KeepReversed)
    );
    assert_eq!(
        emitted_result.report().boundary_needs_resolution_count(),
        union.count_action(BooleanFragmentAction::BoundaryNeedsResolution)
    );
    assert_eq!(
        emitted_result.report().directed_fragment_count(),
        Some(
            union.count_action(BooleanFragmentAction::KeepSourceDirection)
                + union.count_action(BooleanFragmentAction::KeepReversed)
        )
    );
    assert_eq!(
        emitted_result
            .report()
            .directed_source_segment_kind_counts(),
        Some(SegmentKindCounts {
            lines: union.count_action(BooleanFragmentAction::KeepSourceDirection)
                + union.count_action(BooleanFragmentAction::KeepReversed),
            arcs: 0,
        })
    );
    assert_eq!(
        emitted_result.report().directed_fragment_kind_counts(),
        Some(SegmentKindCounts {
            lines: union.count_action(BooleanFragmentAction::KeepSourceDirection)
                + union.count_action(BooleanFragmentAction::KeepReversed),
            arcs: 0,
        })
    );
    assert_eq!(
        emitted_result.report().directed_fragments().len(),
        union.count_action(BooleanFragmentAction::KeepSourceDirection)
            + union.count_action(BooleanFragmentAction::KeepReversed)
    );
    assert_eq!(emitted_result.report().unresolved_boundary_count(), Some(0));
    assert_eq!(emitted_result.report().blocker(), None);
    let emitted = emitted_result
        .fragments()
        .expect("reported emission should materialize");

    assert_eq!(
        emitted.directed_len(),
        union.count_action(BooleanFragmentAction::KeepSourceDirection)
            + union.count_action(BooleanFragmentAction::KeepReversed)
    );
    assert_eq!(emitted.unresolved_len(), 0);
    assert!(emitted.is_ready_for_traversal());
    let emitted_first = &emitted.directed_fragments()[0];
    let emitted_first_source = fragments
        .fragments_for_contour(emitted_first.key)
        .unwrap()
        .fragments
        .fragments()
        .get(emitted_first.fragment_index)
        .unwrap();
    assert_eq!(
        emitted_first.source_segment_index,
        emitted_first_source.source_segment_index
    );
    assert_eq!(
        emitted_first.source_segment_start_point,
        emitted_first_source.source_segment_start_point
    );
    assert_eq!(
        emitted_first.source_segment_end_point,
        emitted_first_source.source_segment_end_point
    );
    assert_eq!(
        emitted_first.source_range,
        emitted_first_source.source_range
    );
    assert!(!emitted_first.reversed);
    let emitted_first_report = &emitted_result.report().directed_fragments()[0];
    assert_eq!(emitted_first_report.key(), emitted_first.key);
    assert_eq!(
        emitted_first_report.fragment_index(),
        emitted_first.fragment_index
    );
    assert_eq!(
        emitted_first_report.source_segment_index(),
        emitted_first.source_segment_index
    );
    assert_eq!(
        emitted_first_report.source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        emitted_first_report.source_segment_start_point(),
        &emitted_first.source_segment_start_point
    );
    assert_eq!(
        emitted_first_report.source_segment_end_point(),
        &emitted_first.source_segment_end_point
    );
    assert_eq!(
        emitted_first_report.source_range(),
        &emitted_first.source_range
    );
    assert_eq!(emitted_first_report.reversed(), emitted_first.reversed);
    assert_eq!(emitted_first_report.output_fragment_index(), 0);

    let assembled = emitted.assemble_chains_with_report(&policy());
    assert!(assembled.report().status().is_native_exact());
    assert_eq!(
        assembled.report().stage(),
        BooleanBoundaryChainAssemblyStage2::ChainMaterialization
    );
    assert_eq!(
        assembled.report().directed_fragment_count(),
        emitted.directed_len()
    );
    assert_eq!(
        assembled.report().directed_source_segment_kind_counts(),
        SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        }
    );
    assert_eq!(
        assembled.report().directed_fragment_kind_counts(),
        SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        }
    );
    assert_eq!(assembled.report().unresolved_boundary_count(), 0);
    assert_eq!(assembled.report().chain_count(), Some(1));
    assert_eq!(assembled.report().closed_chain_count(), Some(1));
    assert_eq!(assembled.report().open_chain_count(), Some(0));
    assert_eq!(
        assembled.report().output_fragment_count(),
        Some(emitted.directed_len())
    );
    assert_eq!(
        assembled.report().output_fragment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
    assert_eq!(
        assembled.report().output_fragments().len(),
        emitted.directed_len()
    );
    let assembled_first_report = &assembled.report().output_fragments()[0];
    assert_eq!(assembled_first_report.key(), emitted_first.key);
    assert_eq!(
        assembled_first_report.fragment_index(),
        emitted_first.fragment_index
    );
    assert_eq!(
        assembled_first_report.source_segment_index(),
        emitted_first.source_segment_index
    );
    assert_eq!(
        assembled_first_report.source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        assembled_first_report.source_segment_start_point(),
        &emitted_first.source_segment_start_point
    );
    assert_eq!(
        assembled_first_report.source_segment_end_point(),
        &emitted_first.source_segment_end_point
    );
    assert_eq!(
        assembled_first_report.source_range(),
        &emitted_first.source_range
    );
    assert_eq!(assembled_first_report.reversed(), emitted_first.reversed);
    assert_eq!(assembled_first_report.output_fragment_index(), 0);
    assert_eq!(assembled.report().blocker(), None);
    let chains = assembled
        .chains()
        .expect("reported boundary chain assembly should materialize");
    assert_eq!(chains.len(), 1);
    assert_eq!(chains.closed_count(), 1);
    assert_eq!(chains.chains()[0].len(), emitted.directed_len());

    let extracted = chains.closed_loops_with_report();
    assert!(extracted.report().status().is_native_exact());
    assert_eq!(
        extracted.report().stage(),
        BooleanBoundaryLoopExtractionStage2::LoopMaterialization
    );
    assert_eq!(extracted.report().source_chain_count(), 1);
    assert_eq!(
        extracted.report().source_fragment_count(),
        emitted.directed_len()
    );
    assert_eq!(
        extracted.report().source_fragment_kind_counts(),
        SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        }
    );
    assert_eq!(extracted.report().closed_chain_count(), 1);
    assert_eq!(extracted.report().open_chain_count(), 0);
    assert_eq!(extracted.report().loop_count(), Some(1));
    assert_eq!(
        extracted.report().output_fragment_count(),
        Some(emitted.directed_len())
    );
    assert_eq!(
        extracted.report().output_source_segment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
    assert_eq!(
        extracted.report().output_fragment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
    assert_eq!(
        extracted.report().output_fragments().len(),
        emitted.directed_len()
    );
    let extracted_first_report = &extracted.report().output_fragments()[0];
    assert_eq!(extracted_first_report.key(), assembled_first_report.key());
    assert_eq!(
        extracted_first_report.fragment_index(),
        assembled_first_report.fragment_index()
    );
    assert_eq!(
        extracted_first_report.source_segment_index(),
        assembled_first_report.source_segment_index()
    );
    assert_eq!(
        extracted_first_report.source_segment_kind(),
        assembled_first_report.source_segment_kind()
    );
    assert_eq!(
        extracted_first_report.source_segment_start_point(),
        assembled_first_report.source_segment_start_point()
    );
    assert_eq!(
        extracted_first_report.source_segment_end_point(),
        assembled_first_report.source_segment_end_point()
    );
    assert_eq!(
        extracted_first_report.source_range(),
        assembled_first_report.source_range()
    );
    assert_eq!(
        extracted_first_report.reversed(),
        assembled_first_report.reversed()
    );
    assert_eq!(extracted_first_report.output_fragment_index(), 0);
    assert_eq!(extracted.report().blocker(), None);
    let loops = extracted
        .loops()
        .expect("reported closed loop extraction should materialize");
    assert_eq!(loops.len(), 1);

    let transferred = loops.to_contours_with_report(FillRule::NonZero);
    assert!(transferred.report().status().is_native_exact());
    assert_eq!(
        transferred.report().stage(),
        BooleanBoundaryContourTransferStage2::ContourMaterialization
    );
    assert_eq!(transferred.report().fill_rule(), FillRule::NonZero);
    assert_eq!(transferred.report().source_loop_count(), 1);
    assert_eq!(
        transferred.report().source_fragment_count(),
        emitted.directed_len()
    );
    assert_eq!(
        transferred.report().source_fragment_kind_counts(),
        SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        }
    );
    assert_eq!(transferred.report().contour_count(), Some(1));
    assert_eq!(
        transferred.report().output_segment_count(),
        Some(emitted.directed_len())
    );
    assert_eq!(
        transferred.report().output_source_segment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
    assert_eq!(
        transferred.report().output_segment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
    assert_eq!(
        transferred.report().output_segments().len(),
        emitted.directed_len()
    );
    let transferred_first_report = &transferred.report().output_segments()[0];
    assert_eq!(transferred_first_report.key(), extracted_first_report.key());
    assert_eq!(
        transferred_first_report.fragment_index(),
        extracted_first_report.fragment_index()
    );
    assert_eq!(
        transferred_first_report.source_segment_index(),
        extracted_first_report.source_segment_index()
    );
    assert_eq!(
        transferred_first_report.source_segment_kind(),
        extracted_first_report.source_segment_kind()
    );
    assert_eq!(
        transferred_first_report.source_segment_start_point(),
        extracted_first_report.source_segment_start_point()
    );
    assert_eq!(
        transferred_first_report.source_segment_end_point(),
        extracted_first_report.source_segment_end_point()
    );
    assert_eq!(
        transferred_first_report.source_range(),
        extracted_first_report.source_range()
    );
    assert_eq!(
        transferred_first_report.reversed(),
        extracted_first_report.reversed()
    );
    assert_eq!(transferred_first_report.output_fragment_index(), 0);
    assert_eq!(transferred.report().blocker(), None);
    let contours = transferred
        .contours()
        .expect("reported contour transfer should materialize");
    assert_eq!(contours.len(), 1);
    assert_eq!(contours[0].len(), emitted.directed_len());
}

#[test]
fn boolean_fragment_selection_emit_rejects_incomplete_or_foreign_inventory() {
    let (first, second, fragments) = overlapping_fragments();
    let Classification::Decided(union) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided union selection");
    };

    let mut incomplete = union.classifications().to_vec();
    incomplete.pop();
    let incomplete = BooleanFragmentSelection::new(incomplete).unwrap();
    assert_topology_error(incomplete.emit_boundary_fragments(&fragments));

    let mut foreign = union.classifications().to_vec();
    foreign.push(BooleanFragmentClassification {
        key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 99),
        fragment_index: 0,
        opposite_location: RegionPointLocation::Outside,
        source_filled_side_is_left: true,
        action: BooleanFragmentAction::Discard,
    });
    let foreign = BooleanFragmentSelection::new(foreign).unwrap();
    assert_topology_error(foreign.emit_boundary_fragments(&fragments));
}

#[test]
fn boolean_boundary_chain_assembly_keeps_disjoint_loops_separate() {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 2, 2)]);
    let second = Region2::from_material_contours(vec![rectangle(4, 4, 6, 6)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
    let Classification::Decided(fragments) = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap()
    else {
        panic!("expected decided fragments");
    };
    let Classification::Decided(union) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided union selection");
    };
    let emitted = union.emit_boundary_fragments(&fragments).unwrap();

    let Classification::Decided(chains) = emitted.assemble_chains(&policy()) else {
        panic!("expected disjoint closed chains");
    };

    assert_eq!(chains.len(), 2);
    assert_eq!(chains.closed_count(), 2);
    assert!(chains.chains().iter().all(|chain| chain.len() == 4));

    let extracted = chains.into_closed_loops_with_report();
    assert!(extracted.report().status().is_native_exact());
    assert_eq!(
        extracted.report().stage(),
        BooleanBoundaryLoopExtractionStage2::LoopMaterialization
    );
    assert_eq!(extracted.report().source_chain_count(), 2);
    assert_eq!(extracted.report().source_fragment_count(), 8);
    assert_eq!(
        extracted.report().source_fragment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(extracted.report().closed_chain_count(), 2);
    assert_eq!(extracted.report().open_chain_count(), 0);
    assert_eq!(extracted.report().loop_count(), Some(2));
    assert_eq!(extracted.report().output_fragment_count(), Some(8));
    assert_eq!(
        extracted.report().output_fragment_kind_counts(),
        Some(SegmentKindCounts { lines: 8, arcs: 0 })
    );
    assert!(
        extracted.loops().is_some(),
        "reported disjoint loop extraction should materialize"
    );
    assert!(matches!(
        extracted.loops_classification(),
        Classification::Decided(loops) if loops.len() == 2
    ));
    let extracted_report = extracted.clone().into_report();
    assert_eq!(&extracted_report, extracted.report());
    let (owned_loops, owned_extracted_report) = extracted.clone().into_parts();
    assert_eq!(owned_loops.as_ref(), extracted.loops());
    assert_eq!(&owned_extracted_report, extracted.report());
    assert!(matches!(
        extracted.clone().into_loops_classification(),
        Classification::Decided(loops) if loops.len() == 2
    ));
    let loops = extracted.into_loops().unwrap();
    let transferred = loops.into_contours_with_report(FillRule::NonZero);
    assert!(transferred.report().status().is_native_exact());
    assert_eq!(
        transferred.report().stage(),
        BooleanBoundaryContourTransferStage2::ContourMaterialization
    );
    assert_eq!(transferred.report().fill_rule(), FillRule::NonZero);
    assert_eq!(transferred.report().source_loop_count(), 2);
    assert_eq!(transferred.report().source_fragment_count(), 8);
    assert_eq!(
        transferred.report().source_fragment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(transferred.report().contour_count(), Some(2));
    assert_eq!(transferred.report().output_segment_count(), Some(8));
    assert_eq!(
        transferred.report().output_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 8, arcs: 0 })
    );
    assert_eq!(transferred.report().output_segments().len(), 8);
    assert_eq!(transferred.report().blocker(), None);
    assert!(matches!(
        transferred.contours_classification(),
        Classification::Decided(contours) if contours.len() == 2
    ));
    let transferred_report = transferred.clone().into_report();
    assert_eq!(&transferred_report, transferred.report());
    let (owned_contours, owned_transferred_report) = transferred.clone().into_parts();
    assert_eq!(owned_contours.as_deref(), transferred.contours());
    assert_eq!(&owned_transferred_report, transferred.report());
    assert!(matches!(
        transferred.clone().into_contours_classification(),
        Classification::Decided(contours) if contours.len() == 2
    ));
    let contours = transferred.into_contours().unwrap();
    assert_eq!(contours.len(), 2);
    assert!(contours.iter().all(|contour| contour.len() == 4));
}

#[test]
fn boolean_fragment_selection_reverses_emitted_second_difference_fragments() {
    let (first, second, fragments) = overlapping_fragments();
    let Classification::Decided(difference) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Difference,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided difference selection");
    };

    let emitted = difference.emit_boundary_fragments(&fragments).unwrap();
    let second_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);
    let reversed = difference
        .classifications()
        .iter()
        .find(|classification| {
            classification.key == second_key
                && classification.action == BooleanFragmentAction::KeepReversed
        })
        .expect("expected a reversed second-operand fragment");
    let source = fragments
        .fragments_for_contour(second_key)
        .unwrap()
        .fragments
        .fragments()
        .get(reversed.fragment_index)
        .unwrap();
    let directed = emitted
        .directed_fragments()
        .iter()
        .find(|fragment| {
            fragment.key == second_key && fragment.fragment_index == reversed.fragment_index
        })
        .expect("expected emitted reversed fragment");

    assert_eq!(directed.segment.start(), source.segment.end());
    assert_eq!(directed.segment.end(), source.segment.start());
    assert_eq!(directed.source_segment_index, source.source_segment_index);
    assert_eq!(
        directed.source_segment_start_point,
        source.source_segment_start_point
    );
    assert_eq!(
        directed.source_segment_end_point,
        source.source_segment_end_point
    );
    assert_eq!(directed.source_range, source.source_range);
    assert!(directed.reversed);
    let emitted_report = difference
        .emit_boundary_fragments_with_report(&fragments)
        .unwrap();
    let directed_report = emitted_report
        .report()
        .directed_fragments()
        .iter()
        .find(|fragment| {
            fragment.key() == second_key && fragment.fragment_index() == reversed.fragment_index
        })
        .expect("expected emitted reversed fragment report");
    assert_eq!(
        directed_report.source_segment_index(),
        directed.source_segment_index
    );
    assert_eq!(
        directed_report.source_segment_start_point(),
        &directed.source_segment_start_point
    );
    assert_eq!(
        directed_report.source_segment_end_point(),
        &directed.source_segment_end_point
    );
    assert_eq!(directed_report.source_range(), &directed.source_range);
    assert!(directed_report.reversed());
}

#[test]
fn boolean_fragment_selection_defers_shared_boundary_fragments() {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -2, 6, 0)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
    assert_eq!(intersections.event_count(), 3);
    assert_eq!(
        intersections.event_count(),
        intersections.point_event_count()
            + intersections.overlap_event_count()
            + intersections.uncertain_event_count()
    );
    assert_eq!(intersections.overlap_event_count(), 1);
    assert_eq!(intersections.uncertain_event_count(), 0);
    let Classification::Decided(fragments) = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap()
    else {
        panic!("expected decided fragments");
    };

    let Classification::Decided(selection) = fragments
        .classify_for_boolean(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided selection");
    };

    assert!(selection.count_action(BooleanFragmentAction::BoundaryNeedsResolution) > 0);
    let reported = fragments
        .classify_for_boolean_with_report(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap();
    assert_eq!(
        reported.report().stage(),
        BooleanFragmentSelectionStage2::ActionAssignment
    );
    assert_eq!(
        reported.report().boundary_needs_resolution_count(),
        Some(selection.count_action(BooleanFragmentAction::BoundaryNeedsResolution))
    );
    assert_eq!(
        reported.report().source_fragment_kind_counts(),
        SegmentKindCounts {
            lines: reported.report().source_fragment_count(),
            arcs: 0,
        }
    );
    assert_eq!(reported.report().blocker(), None);
    assert_eq!(
        reported.selection_classification(),
        Classification::Decided(&selection)
    );
    let reported_selection_report = reported.clone().into_report();
    assert_eq!(&reported_selection_report, reported.report());
    let (owned_selection, owned_selection_report) = reported.clone().into_parts();
    assert_eq!(owned_selection.as_ref(), reported.selection());
    assert_eq!(&owned_selection_report, reported.report());
    assert_eq!(
        reported.into_selection_classification(),
        Classification::Decided(selection.clone())
    );
    let emitted_result = selection
        .emit_boundary_fragments_with_report(&fragments)
        .unwrap();
    assert!(emitted_result.report().status().is_native_exact());
    assert_eq!(
        emitted_result.report().stage(),
        BooleanBoundaryFragmentEmissionStage2::FragmentEmission
    );
    assert_eq!(
        emitted_result.report().boundary_needs_resolution_count(),
        selection.count_action(BooleanFragmentAction::BoundaryNeedsResolution)
    );
    assert_eq!(
        emitted_result.report().unresolved_boundary_count(),
        Some(selection.count_action(BooleanFragmentAction::BoundaryNeedsResolution))
    );
    assert_eq!(
        emitted_result.report().directed_fragment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted_result.report().directed_fragment_count().unwrap(),
            arcs: 0,
        })
    );
    assert_eq!(emitted_result.report().blocker(), None);
    let emitted = emitted_result
        .fragments()
        .expect("reported unresolved emission should materialize");
    assert_eq!(
        emitted_result.fragments_classification(),
        Classification::Decided(emitted)
    );
    let emitted_report = emitted_result.clone().into_report();
    assert_eq!(&emitted_report, emitted_result.report());
    let (owned_emitted, owned_emitted_report) = emitted_result.clone().into_parts();
    assert_eq!(owned_emitted.as_ref(), emitted_result.fragments());
    assert_eq!(&owned_emitted_report, emitted_result.report());
    assert_eq!(
        emitted_result.clone().into_fragments_classification(),
        Classification::Decided(emitted.clone())
    );
    assert!(!emitted.is_ready_for_traversal());
    assert_eq!(
        emitted.unresolved_len(),
        selection.count_action(BooleanFragmentAction::BoundaryNeedsResolution)
    );
    let assembled = emitted.assemble_chains_with_report(&policy());
    assert!(assembled.chains().is_none());
    assert!(assembled.report().status().is_retained_evidence());
    assert_eq!(
        assembled.report().stage(),
        BooleanBoundaryChainAssemblyStage2::BoundaryResolution
    );
    assert_eq!(
        assembled.report().unresolved_boundary_count(),
        emitted.unresolved_len()
    );
    assert_eq!(assembled.report().chain_count(), None);
    assert_eq!(
        assembled.report().blocker(),
        Some(UncertaintyReason::Boundary)
    );
    assert_eq!(
        emitted.assemble_chains(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn shared_boundary_same_direction_boolean_matrix_is_exact() {
    let first = Region2::from_material_contours(vec![triangle([(0, 0), (6, 0), (0, 6)])]);
    let second = Region2::from_material_contours(vec![triangle([(0, 0), (6, 0), (6, 6)])]);
    let samples = [
        (p(1, 4), true, false),
        (p(5, 4), false, true),
        (p(3, 1), true, true),
        (p(3, -1), false, false),
    ];

    assert_exact_boolean_matrix(&first, &second, &samples);

    let clockwise_first = Region2::from_material_contours(vec![triangle([(0, 0), (0, 6), (6, 0)])]);
    let clockwise_second =
        Region2::from_material_contours(vec![triangle([(0, 0), (6, 6), (6, 0)])]);
    assert_exact_boolean_matrix(&clockwise_first, &clockwise_second, &samples);

    let clockwise_union = clockwise_first
        .boolean_region_with_report(
            &clockwise_second,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    let pipeline = clockwise_union
        .report()
        .pipeline_report()
        .expect("clockwise shared-edge union retained its arrangement pipeline");
    assert_eq!(pipeline.shared_boundary_resolution_count(), 1);
    let resolution = &pipeline.shared_boundary_resolutions()[0];
    assert!(resolution.same_direction());
    assert!(!resolution.first_filled_side_is_left());
    assert!(!resolution.second_filled_side_is_left());
    assert_eq!(
        resolution.first_action(),
        BooleanFragmentAction::KeepReversed
    );
    assert_eq!(resolution.second_action(), BooleanFragmentAction::Discard);
}

#[test]
fn shared_boundary_opposite_direction_boolean_matrix_is_exact() {
    let first = Region2::from_material_contours(vec![triangle([(0, 0), (6, 0), (0, 6)])]);
    let second = Region2::from_material_contours(vec![triangle([(6, 0), (0, 0), (6, -6)])]);

    assert_exact_boolean_matrix(
        &first,
        &second,
        &[
            (p(1, 1), true, false),
            (p(5, -1), false, true),
            (p(3, 4), false, false),
        ],
    );
}

#[test]
fn shared_material_hole_boundary_boolean_matrix_is_exact() {
    let first = Region2::new(
        vec![rectangle(-10, -10, 10, 10)],
        vec![rectangle(0, 0, 6, 6)],
    );
    let second = Region2::from_material_contours(vec![triangle([(0, 0), (6, 0), (3, 3)])]);

    assert_exact_boolean_matrix(
        &first,
        &second,
        &[
            (p(-5, 0), true, false),
            (p(3, 1), false, true),
            (p(1, 5), false, false),
            (p(20, 0), false, false),
        ],
    );
}

#[test]
fn center_defined_circle_boolean_matrix_is_orientation_independent() {
    let rectangle = Region2::from_material_contours(vec![rectangle(0, -2, 6, 2)]);
    let samples = [
        (p(-3, 0), true, false),
        (p(1, 0), true, true),
        (p(5, 0), false, true),
        (p(5, 3), false, false),
    ];

    for clockwise in [false, true] {
        let circle = Region2::from_material_contours(vec![center_defined_circle(4, clockwise)]);
        let intersections = circle.intersect_region(&rectangle, &policy()).unwrap();
        let split = intersections
            .split_regions_with_report(&circle.as_view(), &rectangle.as_view(), &policy())
            .unwrap();
        let fragments = split
            .fragments()
            .unwrap_or_else(|| panic!("circle split was unresolved: {split:#?}"));
        let selection = fragments
            .classify_for_boolean_with_report(
                &circle.as_view(),
                &rectangle.as_view(),
                BooleanOp::Union,
                &policy(),
            )
            .unwrap();
        selection
            .selection()
            .unwrap_or_else(|| panic!("circle selection was unresolved: {selection:#?}"));
        assert_exact_boolean_matrix(&circle, &rectangle, &samples);
    }
}

#[test]
fn major_arc_multiple_hits_split_in_sweep_order_and_boolean_exactly() {
    let major = Region2::from_material_contours(vec![major_arc_segment_contour(4)]);
    let strip = Region2::from_material_contours(vec![rectangle(-3, -5, -2, 5)]);
    let intersections = major.intersect_region(&strip, &policy()).unwrap();
    assert_eq!(intersections.point_event_count(), 4);

    let split = intersections
        .split_regions_with_report(&major.as_view(), &strip.as_view(), &policy())
        .unwrap();
    let fragments = split
        .fragments()
        .unwrap_or_else(|| panic!("major-arc split was unresolved: {split:#?}"));
    let major_fragments = fragments
        .fragments_for_contour(RegionContourKey::new(
            RegionSide::First,
            RegionContourRole::Material,
            0,
        ))
        .unwrap();
    assert_eq!(
        major_fragments
            .fragments
            .fragments()
            .iter()
            .filter(|fragment| fragment.source_segment_index == 0)
            .count(),
        5
    );
    let selection = fragments
        .classify_for_boolean_with_report(
            &major.as_view(),
            &strip.as_view(),
            BooleanOp::Union,
            &policy(),
        )
        .unwrap();
    let selection_value = selection
        .selection()
        .unwrap_or_else(|| panic!("major-arc selection was unresolved: {selection:#?}"));
    let emitted = selection_value
        .emit_boundary_fragments_with_report(fragments)
        .unwrap();
    let emitted_value = emitted
        .fragments()
        .unwrap_or_else(|| panic!("major-arc emission was unresolved: {emitted:#?}"));
    let chains = emitted_value.assemble_chains_with_report(&policy());
    chains
        .chains()
        .unwrap_or_else(|| panic!("major-arc chain assembly was unresolved: {chains:#?}"));
    let Classification::Decided(difference_contours) = major
        .boolean_boundary_contours(&strip, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("major-arc difference boundary did not materialize");
    };
    assert_eq!(difference_contours.len(), 2);
    let difference_contacts = difference_contours[0]
        .intersect_contour(&difference_contours[1], &policy())
        .unwrap();
    assert!(difference_contacts.is_empty());
    let reverse_difference = strip
        .boolean_region_with_report(&major, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap();
    assert!(
        matches!(
            reverse_difference.region_classification(),
            Classification::Decided(_)
        ),
        "reverse major-arc difference was {reverse_difference:#?}"
    );

    assert_exact_boolean_matrix(
        &major,
        &strip,
        &[
            (p(0, 0), true, false),
            (hypercurve::Point2::new(q(-5, 2), s(0)), true, true),
            (hypercurve::Point2::new(q(-5, 2), s(4)), false, true),
            (p(5, 0), false, false),
        ],
    );
}

#[test]
fn segment_representative_point_samples_arc_geometry() {
    let circle = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let first_midpoint = circle.segments()[0]
        .representative_point(&policy())
        .unwrap();

    assert_eq!(first_midpoint, Classification::Decided(p(1, -1)));
}

#[test]
fn reversing_segments_swaps_endpoints_and_arc_orientation() {
    let line = Segment2::Line(hypercurve::LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let Segment2::Line(reversed_line) = line.reversed() else {
        panic!("expected reversed line");
    };
    assert_eq!(reversed_line.start(), &p(2, 0));
    assert_eq!(reversed_line.end(), &p(0, 0));

    let arc = Segment2::Arc(hypercurve::CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
    let Segment2::Arc(reversed_arc) = arc.reversed() else {
        panic!("expected reversed arc");
    };
    assert_eq!(reversed_arc.start(), &p(2, 0));
    assert_eq!(reversed_arc.end(), &p(0, 0));
    assert!(reversed_arc.is_clockwise());
    assert_eq!(reversed_arc.bulge(), Some(&s(-1)));
}

#[test]
fn boolean_fragment_selection_constructor_validates_source_ownership() {
    BooleanFragmentSelection::new(Vec::new()).unwrap();
    BooleanFragmentSelection::new(vec![
        fragment_classification(0, BooleanFragmentAction::KeepSourceDirection),
        fragment_classification(1, BooleanFragmentAction::Discard),
    ])
    .unwrap();

    assert_topology_error(BooleanFragmentSelection::new(vec![
        fragment_classification(0, BooleanFragmentAction::KeepSourceDirection),
        unresolved_boundary_classification(0),
    ]));
    assert_topology_error(BooleanFragmentSelection::new(vec![
        fragment_classification_with_location(
            2,
            RegionPointLocation::Boundary,
            BooleanFragmentAction::KeepSourceDirection,
        ),
    ]));
    assert_topology_error(BooleanFragmentSelection::new(vec![
        fragment_classification(3, BooleanFragmentAction::BoundaryNeedsResolution),
    ]));
}

#[test]
fn boolean_boundary_fragment_set_constructor_validates_source_ownership() {
    BooleanBoundaryFragmentSet::new(Vec::new(), Vec::new()).unwrap();
    BooleanBoundaryFragmentSet::new(
        vec![directed_fragment(0, 0, 0, 1, 0)],
        vec![unresolved_boundary_classification(1)],
    )
    .unwrap();

    assert_topology_error(BooleanBoundaryFragmentSet::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(0, 1, 0, 2, 0),
        ],
        Vec::new(),
    ));
    assert_topology_error(BooleanBoundaryFragmentSet::new(
        vec![directed_fragment(0, 0, 0, 1, 0)],
        vec![unresolved_boundary_classification(0)],
    ));
    assert_topology_error(BooleanBoundaryFragmentSet::new(
        Vec::new(),
        vec![fragment_classification(
            2,
            BooleanFragmentAction::BoundaryNeedsResolution,
        )],
    ));
    assert_topology_error(BooleanBoundaryFragmentSet::new(
        Vec::new(),
        vec![fragment_classification_with_location(
            3,
            RegionPointLocation::Boundary,
            BooleanFragmentAction::KeepSourceDirection,
        )],
    ));
}

#[test]
fn boolean_boundary_constructors_reject_zero_length_directed_fragments() {
    let zero = DirectedBooleanFragment {
        key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
        fragment_index: 0,
        source_segment_index: 0,
        source_segment_start_point: p(0, 0),
        source_segment_end_point: p(0, 0),
        source_range: ParamRange::new(s(0), s(1)),
        reversed: false,
        segment: Segment2::Line(LineSeg2::new_unchecked(p(0, 0), p(0, 0))),
    };

    assert_topology_error(BooleanBoundaryFragmentSet::new(
        vec![zero.clone()],
        Vec::new(),
    ));
    assert_topology_error(BooleanBoundaryChain::new(vec![zero.clone()], true));
    assert_topology_error(BooleanBoundaryLoop::new(vec![zero]));
}

#[test]
fn boolean_boundary_chain_constructors_validate_fragment_ownership() {
    assert_topology_error(BooleanBoundaryChain::new(Vec::new(), false));
    assert_topology_error(BooleanBoundaryChain::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(0, 1, 0, 0, 0),
        ],
        true,
    ));

    BooleanBoundaryChain::new(open_chain_fragments(), false).unwrap();
    BooleanBoundaryChain::new(triangle_loop_fragments([0, 1, 2], 0, 0), true).unwrap();
    assert_topology_error(BooleanBoundaryChain::new(open_chain_fragments(), true));
    assert_topology_error(BooleanBoundaryChain::new(
        triangle_loop_fragments([0, 1, 2], 0, 0),
        false,
    ));
    assert_topology_error(BooleanBoundaryChain::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(1, 2, 0, 3, 0),
        ],
        false,
    ));

    let first = BooleanBoundaryChain::new(vec![directed_fragment(0, 0, 0, 1, 0)], false).unwrap();
    let second = BooleanBoundaryChain::new(vec![directed_fragment(1, 1, 0, 2, 0)], false).unwrap();
    BooleanBoundaryChainSet::new(vec![first.clone(), second]).unwrap();

    let duplicate =
        BooleanBoundaryChain::new(vec![directed_fragment(0, 2, 0, 3, 0)], false).unwrap();
    assert_topology_error(BooleanBoundaryChainSet::new(vec![first, duplicate]));
}

#[test]
fn boolean_boundary_loop_constructors_validate_fragment_ownership() {
    assert_topology_error(BooleanBoundaryLoop::new(Vec::new()));
    assert_topology_error(BooleanBoundaryLoop::new(vec![
        directed_fragment(0, 0, 0, 1, 0),
        directed_fragment(0, 1, 0, 0, 0),
    ]));

    assert_topology_error(BooleanBoundaryLoop::new(open_chain_fragments()));
    assert_topology_error(BooleanBoundaryLoop::new(vec![
        directed_fragment(0, 0, 0, 1, 0),
        directed_fragment(1, 2, 0, 3, 0),
    ]));

    let first = BooleanBoundaryLoop::new(triangle_loop_fragments([0, 1, 2], 0, 0)).unwrap();
    let second = BooleanBoundaryLoop::new(triangle_loop_fragments([3, 4, 5], 2, 0)).unwrap();
    BooleanBoundaryLoopSet::new(vec![first.clone(), second]).unwrap();

    let duplicate = BooleanBoundaryLoop::new(triangle_loop_fragments([0, 6, 7], 4, 0)).unwrap();
    assert_topology_error(BooleanBoundaryLoopSet::new(vec![first, duplicate]));
}

#[test]
fn boolean_boundary_loop_set_checks_contour_transfer() {
    let loops = BooleanBoundaryLoopSet::from_contours(vec![rectangle(0, 0, 2, 2)]).unwrap();
    assert_eq!(loops.len(), 1);

    let empty = BooleanBoundaryLoopSet::from_contours(Vec::new()).unwrap();
    assert!(empty.is_empty());

    assert_eq!(
        BooleanBoundaryLoopSet::from_contour_classification(Classification::Uncertain(
            UncertaintyReason::Boundary,
        ))
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn boolean_boundary_loop_set_from_contours_retains_construction_report() {
    let contours = vec![rectangle(0, 0, 2, 2), rectangle(3, 3, 5, 5)];
    let built = BooleanBoundaryLoopSet::from_contours_with_report(contours).unwrap();
    let report = built.report();

    assert!(report.status().is_native_exact());
    assert_eq!(
        report.stage(),
        BooleanBoundaryLoopConstructionStage2::LoopMaterialization
    );
    assert_eq!(report.source_contour_count(), 2);
    assert_eq!(report.source_segment_count(), 8);
    assert_eq!(
        report.source_segment_kind_counts(),
        SegmentKindCounts { lines: 8, arcs: 0 }
    );
    assert_eq!(report.loop_count(), Some(2));
    assert_eq!(report.output_fragment_count(), Some(8));
    assert_eq!(
        report.output_source_segment_kind_counts(),
        Some(SegmentKindCounts { lines: 8, arcs: 0 })
    );
    assert_eq!(
        report.output_fragment_kind_counts(),
        Some(SegmentKindCounts { lines: 8, arcs: 0 })
    );
    assert_eq!(report.output_fragments().len(), 8);
    assert_eq!(report.output_fragments()[0].fragment_index(), 0);
    assert_eq!(report.output_fragments()[0].source_segment_index(), 0);
    assert_eq!(
        report.output_fragments()[0].source_segment_kind(),
        SegmentKind::Line
    );
    assert_eq!(
        report.output_fragments()[0].source_segment_start_point(),
        &p(0, 0)
    );
    assert_eq!(
        report.output_fragments()[0].source_segment_end_point(),
        &p(2, 0)
    );
    assert_eq!(report.output_fragments()[0].source_range().start(), &s(0));
    assert_eq!(report.output_fragments()[0].source_range().end(), &s(1));
    assert!(!report.output_fragments()[0].reversed());
    assert_eq!(report.output_fragments()[0].output_fragment_index(), 0);
    assert_eq!(report.blocker(), None);

    let loops = built.loops().unwrap();
    assert_eq!(loops.len(), 2);
    assert_eq!(loops.loops()[0].fragments()[0].fragment_index, 0);
    assert_eq!(loops.loops()[1].fragments()[3].fragment_index, 3);
}

#[test]
fn boolean_boundary_loop_set_from_borrowed_contours_keeps_inputs_available() {
    let contours = vec![rectangle(0, 0, 2, 2)];
    let built = BooleanBoundaryLoopSet::from_contours_borrowed_with_report(&contours).unwrap();

    assert_eq!(contours.len(), 1);
    assert!(built.report().status().is_native_exact());
    assert_eq!(built.report().source_contour_count(), 1);
    assert_eq!(built.report().source_segment_count(), 4);
    assert_eq!(built.report().output_fragments().len(), 4);
    assert_eq!(built.loops().unwrap().len(), 1);

    let loops = BooleanBoundaryLoopSet::from_contours_borrowed(&contours).unwrap();
    assert_eq!(contours.len(), 1);
    assert_eq!(loops.len(), 1);
}

#[test]
fn boundary_chain_assembly_rejects_branch_points() {
    let fragments = BooleanBoundaryFragmentSet::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(1, 1, 0, 2, 0),
            directed_fragment(2, 1, 0, 1, 1),
        ],
        Vec::new(),
    )
    .unwrap();

    let assembled = fragments.assemble_chains_with_report(&policy());
    assert!(assembled.chains().is_none());
    assert!(assembled.report().status().is_retained_evidence());
    assert_eq!(
        assembled.report().stage(),
        BooleanBoundaryChainAssemblyStage2::EndpointAdjacency
    );
    assert_eq!(assembled.report().directed_fragment_count(), 3);
    assert_eq!(
        assembled.report().directed_fragment_kind_counts(),
        SegmentKindCounts { lines: 3, arcs: 0 }
    );
    assert_eq!(assembled.report().output_fragment_kind_counts(), None);
    assert_eq!(assembled.report().output_fragments().len(), 0);
    assert_eq!(assembled.report().unresolved_boundary_count(), 0);
    assert_eq!(
        assembled.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        assembled.chains_classification(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    let assembled_report = assembled.clone().into_report();
    assert_eq!(&assembled_report, assembled.report());
    let (owned_chains, owned_assembled_report) = assembled.clone().into_parts();
    assert_eq!(owned_chains.as_ref(), assembled.chains());
    assert_eq!(&owned_assembled_report, assembled.report());
    assert_eq!(
        assembled.into_chains_classification(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        fragments.assemble_chains(&policy()),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn boundary_loop_extraction_rejects_open_chains() {
    let fragments = BooleanBoundaryFragmentSet::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(1, 1, 0, 2, 0),
        ],
        Vec::new(),
    )
    .unwrap();

    let assembled = fragments.assemble_chains_with_report(&policy());
    assert!(assembled.report().status().is_native_exact());
    assert_eq!(
        assembled.report().stage(),
        BooleanBoundaryChainAssemblyStage2::ChainMaterialization
    );
    assert_eq!(assembled.report().directed_fragment_count(), 2);
    assert_eq!(
        assembled.report().directed_fragment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(assembled.report().chain_count(), Some(1));
    assert_eq!(assembled.report().closed_chain_count(), Some(0));
    assert_eq!(assembled.report().open_chain_count(), Some(1));
    assert_eq!(assembled.report().output_fragment_count(), Some(2));
    assert_eq!(
        assembled.report().output_fragment_kind_counts(),
        Some(SegmentKindCounts { lines: 2, arcs: 0 })
    );
    assert_eq!(assembled.report().blocker(), None);
    let chains = assembled
        .chains()
        .expect("reported open chain assembly should materialize");
    assert!(matches!(
        assembled.chains_classification(),
        Classification::Decided(chains) if chains.len() == 1
    ));
    assert_eq!(chains.len(), 1);
    assert_eq!(chains.closed_count(), 0);
    let extracted = chains.closed_loops_with_report();
    assert!(extracted.loops().is_none());
    assert!(extracted.report().status().is_retained_evidence());
    assert_eq!(
        extracted.report().stage(),
        BooleanBoundaryLoopExtractionStage2::ChainClosureValidation
    );
    assert_eq!(extracted.report().source_chain_count(), 1);
    assert_eq!(extracted.report().source_fragment_count(), 2);
    assert_eq!(
        extracted.report().source_fragment_kind_counts(),
        SegmentKindCounts { lines: 2, arcs: 0 }
    );
    assert_eq!(extracted.report().closed_chain_count(), 0);
    assert_eq!(extracted.report().open_chain_count(), 1);
    assert_eq!(extracted.report().loop_count(), None);
    assert_eq!(extracted.report().output_fragment_kind_counts(), None);
    assert_eq!(extracted.report().output_fragments().len(), 0);
    assert_eq!(
        extracted.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        extracted.loops_classification(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        extracted.into_loops_classification(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        chains.closed_loops(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}
