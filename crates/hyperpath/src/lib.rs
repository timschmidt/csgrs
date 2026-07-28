//! Exact path planning primitives for the hyper geometry stack.
//!
//! `hyperpath` owns path-domain carriers and scheduling facts for CAM and PCB
//! routing. It deliberately delegates scalar arithmetic to `hyperreal` and
//! topology predicates to `hyperlimit`. Path search may generate candidates, but exact
//! predicates certify the topology before the candidate becomes output.

pub mod arc;
pub mod arrangement;
pub mod bezier;
pub mod bezier_arrangement;
pub mod cam;
pub mod curve_cell;
pub mod mixed_bezier_arrangement;
pub mod mixed_conic_arrangement;
pub mod mixed_cubic_arrangement;
pub mod mixed_curve_arrangement;
pub mod offset;
pub mod pcb;
pub mod pcb_circular_board;
pub mod pcb_convex_pad;
pub mod pcb_obround_board;
pub mod pcb_obround_pad;
pub mod pcb_oriented;
pub mod pcb_orthogonal_pad;
pub mod ph;
pub mod ph_smoothing;
pub mod routing;
pub mod segment;
pub mod solve;
pub mod specctra;
mod specctra_syntax;
pub mod swept;
pub mod tangent;

pub use arc::{
    ArcDirection, CardinalPoint, CircularArc, CircularArcError, CircularArcFacts,
    ExplicitArcArrangementClass, ExplicitArcArrangementReport, ExplicitArcIntersectionClass,
    ExplicitArcIntersectionReport, ExplicitArcOverlapClass, ExplicitArcOverlapReport,
    ExplicitArcPointClassification, ExplicitArcSweepClass, ExplicitArcTangentClass,
    ExplicitArcTangentReport, ExplicitCircleRelationClass, ExplicitCircleRelationReport,
    ExplicitCircularArc, ExplicitCircularArcFacts, LineExplicitArcIntersectionClass,
    LineExplicitArcIntersectionReport,
};
pub use arrangement::{
    ExplicitArcArrangementBreakpoint, ExplicitArcArrangementError, ExplicitArcArrangementFragment,
    ExplicitArcSetArrangementEvent, ExplicitArcSetArrangementReport, LineArcArrangementEvent,
    LineArcArrangementEventClass, LineArcArrangementReport, LineArrangementBreakpoint,
    LineArrangementCellEdge, LineArrangementCellFace, LineArrangementCellFaceClass,
    LineArrangementCellGraph, LineArrangementCellVertex, LineArrangementError,
    LineArrangementEvent, LineArrangementEventClass, LineArrangementFacts, LineArrangementFragment,
    LineArrangementHalfEdge, LineArrangementReport, arrange_explicit_arcs, arrange_line_segments,
    arrange_line_segments_with_explicit_arcs,
};
pub use bezier::{
    BezierParameter, BezierParameterError, CubicBezier, CubicBezierFacts, HigherOrderBezier,
    HigherOrderBezierError, HigherOrderBezierFacts, QuadraticBezier, QuadraticBezierFacts,
    RationalQuadraticBezier, RationalQuadraticBezierError, RationalQuadraticBezierFacts,
};
pub use bezier_arrangement::{
    BezierArrangementBreakpoint, BezierArrangementError, CubicBezierArrangementFragment,
    CubicBezierArrangementReport, HomogeneousPoint2, LineCubicAlgebraicPointDomain,
    LineCubicAlgebraicRootDomain, LineCubicBezierAlgebraicInverseRoot,
    LineCubicBezierAlgebraicPointImage, LineCubicBezierAlgebraicSupportRoot,
    LineCubicBezierIntersection, LineCubicBezierIntersectionClass,
    LineCubicBezierIntersectionReport, LineCubicBezierInverseBoundaryRoots,
    LineCubicBezierInverseBoundarySource, LineCubicBezierSupportOverlap,
    LineCubicBezierSupportOverlapMonotonicity, LineQuadraticBezierIntersection,
    LineQuadraticBezierIntersectionClass, LineQuadraticBezierIntersectionReport,
    LineRationalQuadraticBezierAlgebraicInverseRoot, LineRationalQuadraticBezierIntersection,
    LineRationalQuadraticBezierIntersectionClass, LineRationalQuadraticBezierIntersectionReport,
    LineRationalQuadraticBezierInverseBoundaryRoots,
    LineRationalQuadraticBezierInverseBoundarySource, LineRationalQuadraticBezierInverseRootDomain,
    LineRationalQuadraticBezierSupportOverlap,
    LineRationalQuadraticBezierSupportOverlapMonotonicity, QuadraticBezierArrangementFragment,
    QuadraticBezierArrangementReport, RationalQuadraticBezierArrangementFragment,
    RationalQuadraticBezierArrangementReport, arrange_cubic_beziers, arrange_quadratic_beziers,
    arrange_rational_quadratic_beziers, intersect_axis_aligned_line_cubic_bezier,
    intersect_axis_aligned_line_quadratic_bezier,
    intersect_axis_aligned_line_rational_quadratic_bezier, intersect_line_cubic_bezier,
    intersect_line_quadratic_bezier, intersect_line_rational_quadratic_bezier,
};
pub use cam::{
    AdditiveBeadLine, AdditiveInfillLink, BeadFillAxis, BeadPlanError, InfillGraphError,
    PocketLinkGraphError, PocketLinkSegment, PocketOffsetRing, PocketPlanError,
    PocketPlanStopReason, PocketRingSegment, PocketRingSide, RectangularBeadPlan,
    RectangularInfillGraph, RectangularPocket, RectangularPocketLinkGraph, RectangularPocketPlan,
    RectangularRegionDifference, RectangularRegionIntersection, RectangularRegionRelation,
    RectangularRestCutRecord, RectangularRestMaterialError, RectangularRestMaterialGraph,
    RectangularRestMaterialStage, RectangularSupportPlan, RegionBooleanError,
    SupportFootprintStatus, SupportPlanError, intersect_rectangular_regions, rectangular_bead_plan,
    rectangular_pocket_link_graph, rectangular_pocket_plan, rectangular_rest_material_graph,
    rectangular_serpentine_infill_graph, rectangular_support_plan, subtract_rectangular_region,
};
pub use curve_cell::{
    CurveArrangementCellEdge, CurveArrangementCellEdgeKind, CurveArrangementCellError,
    CurveArrangementCellFace, CurveArrangementCellFaceClass, CurveArrangementCellGraph,
    CurveArrangementCellVertex, CurveArrangementHalfEdge, CurveArrangementLoopRoleBlocker,
    CurveArrangementLoopRoleClass, CurveArrangementLoopRoleReport,
};
pub use mixed_bezier_arrangement::{
    LineQuadraticBezierArrangementError, LineQuadraticBezierArrangementEvent,
    LineQuadraticBezierArrangementFacts, LineQuadraticBezierArrangementReport,
    MixedLineArrangementBreakpoint, MixedLineArrangementFragment, QuadraticBezierRealBreakpoint,
    QuadraticBezierRealFragment, arrange_line_segments_with_quadratic_beziers,
};
pub use mixed_conic_arrangement::{
    LineRationalQuadraticBezierAlgebraicBreakpoint,
    LineRationalQuadraticBezierAlgebraicBreakpointDomain,
    LineRationalQuadraticBezierAlgebraicBreakpointOrder,
    LineRationalQuadraticBezierAlgebraicBreakpointOrderClass,
    LineRationalQuadraticBezierAlgebraicBreakpointSequence,
    LineRationalQuadraticBezierAlgebraicBreakpointSequenceBlocker,
    LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass,
    LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    LineRationalQuadraticBezierAlgebraicEndpointEnvelope,
    LineRationalQuadraticBezierAlgebraicSourceSpan,
    LineRationalQuadraticBezierAlgebraicSourceSpanBoundary,
    LineRationalQuadraticBezierArrangementError, LineRationalQuadraticBezierArrangementEvent,
    LineRationalQuadraticBezierArrangementFacts, LineRationalQuadraticBezierArrangementReport,
    LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion,
    LineRationalQuadraticBezierSupportOverlapCandidate, MixedConicLineArrangementBreakpoint,
    MixedConicLineArrangementFragment, RationalQuadraticBezierRealBreakpoint,
    RationalQuadraticBezierRealFragment, arrange_line_segments_with_rational_quadratic_beziers,
};
pub use mixed_cubic_arrangement::{
    CubicBezierRealBreakpoint, CubicBezierRealFragment, LineCubicBezierAlgebraicBreakpoint,
    LineCubicBezierAlgebraicBreakpointDomain, LineCubicBezierAlgebraicBreakpointOrder,
    LineCubicBezierAlgebraicBreakpointOrderClass, LineCubicBezierAlgebraicBreakpointSequence,
    LineCubicBezierAlgebraicBreakpointSequenceBlocker,
    LineCubicBezierAlgebraicBreakpointSequenceClass,
    LineCubicBezierAlgebraicBreakpointSequenceSource, LineCubicBezierAlgebraicEndpointEnvelope,
    LineCubicBezierAlgebraicOverlapBreakpoint, LineCubicBezierAlgebraicOverlapBreakpointDomain,
    LineCubicBezierAlgebraicOverlapBreakpointOrder,
    LineCubicBezierAlgebraicOverlapBreakpointOrderClass,
    LineCubicBezierAlgebraicOverlapBreakpointSequence,
    LineCubicBezierAlgebraicOverlapBreakpointSequenceBlocker,
    LineCubicBezierAlgebraicOverlapBreakpointSequenceClass,
    LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    LineCubicBezierAlgebraicOverlapEndpointEnvelope, LineCubicBezierAlgebraicOverlapSourceSpan,
    LineCubicBezierAlgebraicOverlapSourceSpanBoundary, LineCubicBezierAlgebraicSourceSpan,
    LineCubicBezierAlgebraicSourceSpanBoundary, LineCubicBezierArrangementError,
    LineCubicBezierArrangementEvent, LineCubicBezierArrangementFacts,
    LineCubicBezierArrangementReport, LineCubicBezierExactAlgebraicBreakpointPromotion,
    LineCubicBezierExactAlgebraicOverlapBreakpointPromotion,
    LineCubicBezierSupportOverlapCandidate, MixedCubicLineArrangementBreakpoint,
    MixedCubicLineArrangementFragment, arrange_line_segments_with_cubic_beziers,
};
pub use mixed_curve_arrangement::{
    LineMixedBezierArrangementError, LineMixedBezierArrangementFacts,
    LineMixedBezierArrangementReport, LineMixedCubicAlgebraicEvidence,
    LineMixedRationalQuadraticAlgebraicEvidence, MixedCurveEndpointTangentClass,
    MixedCurveFragmentEndpoint, MixedCurveFragmentRef, MixedCurveFragmentSeparation,
    MixedCurveFragmentSeparationClass, MixedCurveSourceRef,
    arrange_line_segments_with_mixed_beziers, arrange_line_segments_with_mixed_curves,
};
pub use offset::{
    ArcOffsetCandidate, ArcOffsetError, BezierOffsetError, BezierOffsetSampleCandidate,
    ExplicitArcOffsetCandidate, LineOffsetCandidate, LineOffsetError, OffsetSide,
    offset_axis_aligned_segment, offset_cardinal_arc, offset_cubic_bezier_sample,
    offset_explicit_arc, offset_higher_order_bezier_sample, offset_quadratic_bezier_sample,
};
pub use pcb::{
    BoardContourError, BoardContourOrientation, CardinalRotation, ClearanceStatus,
    DrillBoardClearanceReport, NetId, PadBoardClearanceReport, PcbBoardOutline, PcbCardinalRectPad,
    PcbCircularPad, PcbConvexBoardOutline, PcbOrthogonalBoardOutline, PcbPadFacts, PcbRectPad,
    PcbRoundedRectPad, PcbRoundedRectPadFacts, PcbTrace, PcbTraceFacts, PcbViaStack,
    TraceClearanceReport, TraceLayer, TraceWidthClass, ViaAnnularRingReport, ViaAspectRatioReport,
    ViaDrillIntent, ViaDrillPolicyClass, ViaDrillPolicyReport, ViaFabricationAcceptance,
    ViaFabricationError, ViaFabricationPolicy, ViaFabricationReport, ViaLayerSpanRelation,
    ViaLayerSpanReport, ViaLayerTransitionClass, ViaLayerTransitionReport,
    ViaTransitionPolicyReport, certify_via_fabrication_policy,
    check_cardinal_rect_pad_board_clearance, check_circular_pad_board_clearance,
    check_rect_pad_board_clearance, check_rounded_rect_pad_board_clearance,
    check_trace_board_clearance, check_trace_cardinal_rect_pad_clearance, check_trace_clearance,
    check_trace_convex_board_clearance, check_trace_orthogonal_board_clearance,
    check_trace_pad_clearance, check_trace_rect_pad_clearance,
    check_trace_rounded_rect_pad_clearance, check_trace_via_clearance,
    check_trace_via_drill_clearance, check_via_drill_board_clearance,
};
pub use pcb_circular_board::{
    PcbCircularBoardOutline, PcbCircularBoardOutlineFacts,
    check_circular_pad_circular_board_clearance, check_trace_circular_board_clearance,
};
pub use pcb_convex_pad::{
    PcbConvexPad, PcbConvexPadFacts, check_convex_pad_board_clearance,
    check_trace_convex_pad_clearance,
};
pub use pcb_obround_board::{
    PcbObroundBoardOutline, PcbObroundBoardOutlineFacts,
    check_circular_pad_obround_board_clearance, check_trace_obround_board_clearance,
};
pub use pcb_obround_pad::{
    PcbObroundPad, PcbObroundPadFacts, check_obround_pad_board_clearance,
    check_trace_obround_pad_clearance,
};
pub use pcb_oriented::{
    PcbOrientedRectPad, PcbOrientedRectPadFacts, check_oriented_rect_pad_board_clearance,
    check_trace_oriented_rect_pad_clearance,
};
pub use pcb_orthogonal_pad::{
    PcbOrthogonalPad, PcbOrthogonalPadFacts, check_orthogonal_pad_board_clearance,
    check_trace_orthogonal_pad_clearance,
};
pub use ph::{
    CubicPhFacts, CubicPhInverseLengthReport, CubicPythagoreanHodograph, PhCurveError,
    QuinticPhFacts, QuinticPhInverseLengthReport, QuinticPythagoreanHodograph,
    certify_cubic_ph_inverse_length, certify_quintic_ph_inverse_length,
};
pub use ph_smoothing::{
    QuinticPhG1SmoothingReport, certify_quintic_ph_g1_smoothing,
    certify_quintic_ph_g1_smoothing_between,
};
pub use routing::{
    AccelerationLimitedFeedProfileClass, AccelerationLimitedFeedTimeReport, ConstantFeedTimeReport,
    CornerLookaheadJoinClass, CornerLookaheadJoinReport, CornerLookaheadLimitReport,
    DifferentialPairSkewReport, FeedPathElement, JerkLimitedFeedTimeReport,
    JerkRampElementPhaseReport, JerkRampFeedScheduleReport, JerkRampPhaseProposal,
    JerkRampPhaseReport, JerkRampSpanProposal, JerkRampSpanReport, KeepoutAwareDetourMeander,
    LengthMatchProblem, LookaheadFeedSchedule, LookaheadFeedScheduleReport,
    LookaheadSpanTransitionReport, MeanderCandidatePlacementReport, MeanderError, MeanderKeepout,
    MeanderKeepoutCandidatePlacementReport, MeanderKeepoutPlacementReport, MeanderObstacle,
    MeanderPlacementCandidate, MeanderPlacementReport, MeanderPlacementSlot, MultiDetourMeander,
    MultiPhaseJerkRampFeedScheduleReport, NonUniformDetourMeander, ObstacleAwareDetourMeander,
    RouteCertificationError, SingleDetourMeander, alternating_detour_meander,
    certify_acceleration_limited_feed_time, certify_acceleration_limited_feed_time_for_path,
    certify_constant_feed_time, certify_constant_feed_time_for_path,
    certify_corner_lookahead_limits, certify_differential_pair_skew,
    certify_jerk_ramp_feed_schedule, certify_length_extension, certify_lookahead_feed_schedule,
    certify_multi_phase_jerk_ramp_feed_schedule, certify_symmetric_jerk_limited_feed_time,
    certify_symmetric_jerk_limited_feed_time_for_path, classify_meander_candidate_slots,
    classify_meander_candidate_slots_with_keepouts, classify_meander_placement_slots,
    classify_meander_placement_slots_with_keepouts, keepout_aware_detour_meander,
    length_match_problem, multi_detour_meander, nonuniform_detour_meander,
    obstacle_aware_detour_meander, single_detour_meander,
};
pub use segment::{Axis, LinePathSegment, LinePathSegmentFacts, SegmentParameterOrder};
pub use solve::{
    PcbConstraintSet, RectangularRegion, ToolpathConstraintSet, bezier_offset_sample_constraints,
    center_clearance_squared_constraint, constant_feed_time_equation,
    differential_pair_skew_equation, length_match_equation, rectangular_difference_area_equation,
    rectangular_region_area_equation, rectangular_region_containment_constraints,
    symmetric_jerk_limited_feed_time_equation,
};
pub use specctra::{
    SpecctraArcWireRecord, SpecctraGridArcWireRecord, SpecctraGridKeepoutRecord,
    SpecctraGridKeepoutShape, SpecctraGridRouteRecords, SpecctraGridRouteRuleRecord,
    SpecctraGridTraceRecord, SpecctraGridViaRecord, SpecctraImportError, SpecctraKeepoutRecord,
    SpecctraLayerAlias, SpecctraNetAlias, SpecctraParseError, SpecctraRoute, SpecctraRouteArc,
    SpecctraRouteBezier, SpecctraRouteRuleAudit, SpecctraRouteRuleAuditError,
    SpecctraRouteRuleItemAudit, SpecctraRouteRuleItemKind, SpecctraRouteRuleRecord,
    SpecctraRouteRuleScopeClass, SpecctraRouteRuleTraceClearanceAudit,
    SpecctraRouteRuleTraceClearancePairAudit, SpecctraRouteRuleTraceClearanceStatus,
    SpecctraRouteRuleWidthStatus, SpecctraTraceRecord, SpecctraViaRecord,
    audit_specctra_route_rule_widths, audit_specctra_trace_rule_clearances,
    export_specctra_trace_record, export_specctra_via_record, import_specctra_arc_wire_record,
    import_specctra_keepout_record, import_specctra_text_route, import_specctra_trace_record,
    import_specctra_via_record, parse_specctra_grid_route_records,
    parse_specctra_grid_trace_records, serialize_specctra_grid_arc_wire_records,
    serialize_specctra_grid_keepout_records, serialize_specctra_grid_route_records,
    serialize_specctra_grid_route_rule_records, serialize_specctra_grid_trace_records,
    serialize_specctra_grid_via_records, specctra_grid_arc_wire_record,
    specctra_grid_keepout_record, specctra_grid_route_rule_record, specctra_grid_trace_record,
    specctra_grid_via_record,
};
pub use swept::{SweptLineSegment, SweptLineSegmentFacts};
pub use tangent::{
    G1ChainCertificationReport, G1JoinProblem, TangentAlignment, TangentAlignmentProblem,
    TangentChainReport, TangentJoinClass, TangentJoinReport, TangentSpan, certify_g1_chain,
    certify_g1_join_candidate, certify_tangent_alignment_candidate, classify_tangent_alignment,
    classify_tangent_chain, classify_tangent_join, g1_join_problem,
    oriented_tangent_alignment_problem, tangent_alignment_problem, tangent_cross, tangent_dot,
    tangent_norm_squared,
};
