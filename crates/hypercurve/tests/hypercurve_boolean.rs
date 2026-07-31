use hypercurve::{
    BooleanBoundaryChain, BooleanBoundaryChainSet, BooleanBoundaryFragmentSet, BooleanBoundaryLoop,
    BooleanBoundaryLoopSet, BooleanFragmentAction, BooleanFragmentClassification,
    BooleanFragmentSelection, BooleanOp, BulgeVertex2, Classification, Contour2, ContourFragment,
    ContourFragmentSet, CurveContext, CurveError, DirectedBooleanFragment, FillRule,
    LineArcRegion2, LineSeg2, ParamRange, Real, RegionContourFragments, RegionContourKey,
    RegionContourRole, RegionFragmentSet, RegionPointLocation, RegionSide, Segment2,
    UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
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
    first: &LineArcRegion2,
    second: &LineArcRegion2,
    samples: &[(hypercurve::Point2, bool, bool)],
) {
    let policy = policy();

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let direct = first
            .boolean_region(second, op, FillRule::NonZero, &policy)
            .unwrap();
        let Classification::Decided(direct_region) = direct else {
            panic!("direct {op:?} was not decided");
        };
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
        }
    }
}

fn policy() -> CurveContext {
    CurveContext::STRICT
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

fn overlapping_fragments() -> (
    LineArcRegion2,
    LineArcRegion2,
    hypercurve::RegionFragmentSet,
) {
    let first = LineArcRegion2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = LineArcRegion2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
    let fragment_result = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap();
    let Classification::Decided(fragments) = fragment_result else {
        panic!("expected decided fragments");
    };

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
fn partial_shared_boundary_containment_boolean_matrix_is_exact() {
    let outer = LineArcRegion2::from_material_contours(vec![rectangle(-3, -3, 13, 13)]);
    let touching_inset = LineArcRegion2::from_material_contours(vec![rectangle(4, 6, 6, 13)]);

    assert_exact_boolean_matrix(
        &outer,
        &touching_inset,
        &[
            (p(0, 0), true, false),
            (p(5, 8), true, true),
            (p(14, 5), false, false),
        ],
    );

    let difference = outer
        .boolean_region(
            &touching_inset,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    let Classification::Decided(difference) = difference else {
        panic!("partial shared-boundary containment difference was unresolved");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert!(difference.hole_contours().is_empty());
}

#[test]
fn shared_boundary_opposite_direction_boolean_matrix_is_exact() {
    let first = LineArcRegion2::from_material_contours(vec![triangle([(0, 0), (6, 0), (0, 6)])]);
    let second = LineArcRegion2::from_material_contours(vec![triangle([(6, 0), (0, 0), (6, -6)])]);

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
    let first = LineArcRegion2::new(
        vec![rectangle(-10, -10, 10, 10)],
        vec![rectangle(0, 0, 6, 6)],
    );
    let second = LineArcRegion2::from_material_contours(vec![triangle([(0, 0), (6, 0), (3, 3)])]);

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
fn boundary_chain_assembly_orders_branch_points_by_tangent() {
    let fragments = BooleanBoundaryFragmentSet::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(1, 1, 0, 2, 0),
            directed_fragment(2, 1, 0, 1, 1),
        ],
        Vec::new(),
    )
    .unwrap();

    let Classification::Decided(chains) = fragments.assemble_chains(&policy()) else {
        panic!("distinct branch tangents should select an exact successor");
    };
    assert_eq!(chains.len(), 2);
    assert_eq!(chains.closed_count(), 0);
    assert_eq!(chains.chains()[0].len(), 2);
    assert_eq!(chains.chains()[0].fragments()[0].fragment_index, 0);
    assert_eq!(chains.chains()[0].fragments()[1].fragment_index, 1);
    assert_eq!(chains.chains()[1].len(), 1);
    assert_eq!(chains.chains()[1].fragments()[0].fragment_index, 2);
}

#[test]
fn boundary_chain_assembly_rejects_equal_tangent_branch_points() {
    let fragments = BooleanBoundaryFragmentSet::new(
        vec![
            directed_fragment(0, 0, 0, 1, 0),
            directed_fragment(1, 1, 0, 2, 0),
            directed_fragment(2, 1, 0, 3, 0),
        ],
        Vec::new(),
    )
    .unwrap();

    assert_eq!(
        fragments.assemble_chains(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
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

    let key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let source_point = p(0, 0);
    let fragments = RegionFragmentSet::new(vec![RegionContourFragments {
        key,
        fragments: ContourFragmentSet::new(vec![ContourFragment {
            source_segment_index: 0,
            source_segment_start_point: source_point.clone(),
            source_segment_end_point: source_point.clone(),
            source_range: ParamRange::new(s(0), s(1)),
            segment: Segment2::Line(LineSeg2::new_unchecked(source_point.clone(), source_point)),
        }])
        .unwrap(),
    }])
    .unwrap();
    let selection = BooleanFragmentSelection::new(vec![fragment_classification(
        0,
        BooleanFragmentAction::KeepSourceDirection,
    )])
    .unwrap();
    assert_topology_error(selection.emit_boundary_fragments(&fragments));
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
