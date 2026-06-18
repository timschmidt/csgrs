use hypercurve::{
    BooleanBoundaryChain, BooleanBoundaryChainAssemblyStage2, BooleanBoundaryChainSet,
    BooleanBoundaryContourTransferStage2, BooleanBoundaryFragmentEmissionStage2,
    BooleanBoundaryFragmentSet, BooleanBoundaryLoop, BooleanBoundaryLoopExtractionStage2,
    BooleanBoundaryLoopSet, BooleanFragmentAction, BooleanFragmentClassification,
    BooleanFragmentSelection, BooleanFragmentSelectionStage2, BooleanOp, BulgeVertex2,
    Classification, Contour2, CurveError, CurvePolicy, DirectedBooleanFragment, FillRule, LineSeg2,
    Real, Region2, RegionContourKey, RegionContourRole, RegionPointLocation, RegionSide, Segment2,
    SegmentKindCounts, UncertaintyReason,
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

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
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
    DirectedBooleanFragment {
        key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
        fragment_index,
        segment: line_segment(x0, y0, x1, y1),
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
    let Classification::Decided(fragments) = intersections
        .split_regions(&first.as_view(), &second.as_view(), &policy())
        .unwrap()
    else {
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
        emitted_result.report().directed_fragment_kind_counts(),
        Some(SegmentKindCounts {
            lines: union.count_action(BooleanFragmentAction::KeepSourceDirection)
                + union.count_action(BooleanFragmentAction::KeepReversed),
            arcs: 0,
        })
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
        extracted.report().output_fragment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
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
        transferred.report().output_segment_kind_counts(),
        Some(SegmentKindCounts {
            lines: emitted.directed_len(),
            arcs: 0,
        })
    );
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
    assert_eq!(transferred.report().blocker(), None);
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
}

#[test]
fn boolean_fragment_selection_defers_shared_boundary_fragments() {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -2, 6, 0)]);
    let intersections = first.intersect_region(&second, &policy()).unwrap();
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
fn boundary_chain_assembly_rejects_branch_points() {
    let key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let fragments = BooleanBoundaryFragmentSet::new(
        vec![
            DirectedBooleanFragment {
                key,
                fragment_index: 0,
                segment: line_segment(0, 0, 1, 0),
            },
            DirectedBooleanFragment {
                key,
                fragment_index: 1,
                segment: line_segment(1, 0, 2, 0),
            },
            DirectedBooleanFragment {
                key,
                fragment_index: 2,
                segment: line_segment(1, 0, 1, 1),
            },
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
    assert_eq!(assembled.report().unresolved_boundary_count(), 0);
    assert_eq!(
        assembled.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        fragments.assemble_chains(&policy()),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn boundary_loop_extraction_rejects_open_chains() {
    let key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
    let fragments = BooleanBoundaryFragmentSet::new(
        vec![
            DirectedBooleanFragment {
                key,
                fragment_index: 0,
                segment: line_segment(0, 0, 1, 0),
            },
            DirectedBooleanFragment {
                key,
                fragment_index: 1,
                segment: line_segment(1, 0, 2, 0),
            },
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
    assert_eq!(
        extracted.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        chains.closed_loops(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}
