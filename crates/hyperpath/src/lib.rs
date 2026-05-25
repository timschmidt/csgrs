//! Exact path planning primitives for the hyper geometry stack.
//!
//! `hyperpath` owns path-domain carriers and scheduling facts for CAM and PCB
//! routing. It deliberately delegates scalar arithmetic to `hyperreal` and
//! topology predicates to `hyperlimit`. This is the object-layer split
//! advocated by Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997): path search may generate candidates, but exact
//! predicates certify the topology before the candidate becomes output.

pub mod arc;
pub mod bezier;
pub mod cam;
pub mod offset;
pub mod pcb;
pub mod provenance;
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
pub use bezier::{
    BezierParameter, BezierParameterError, CubicBezier, CubicBezierFacts, HigherOrderBezier,
    HigherOrderBezierError, HigherOrderBezierFacts, QuadraticBezier, QuadraticBezierFacts,
    RationalQuadraticBezier, RationalQuadraticBezierError, RationalQuadraticBezierFacts,
};
pub use cam::{
    AdditiveBeadLine, AdditiveInfillLink, BeadFillAxis, BeadPlanError, InfillGraphError,
    PocketOffsetRing, PocketPlanError, PocketPlanStopReason, RectangularBeadPlan,
    RectangularInfillGraph, RectangularPocket, RectangularPocketPlan, RectangularRegionDifference,
    RectangularRegionIntersection, RectangularRegionRelation, RectangularSupportPlan,
    RegionBooleanError, SupportFootprintStatus, SupportPlanError, build_rectangular_bead_plan,
    build_rectangular_pocket_plan, build_rectangular_serpentine_infill_graph,
    build_rectangular_support_plan, intersect_rectangular_regions, subtract_rectangular_region,
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
    PcbTrace, PcbTraceFacts, PcbViaStack, TraceClearanceReport, TraceLayer, TraceWidthClass,
    ViaAnnularRingReport, ViaDrillIntent, ViaDrillPolicyClass, ViaDrillPolicyReport,
    ViaLayerSpanRelation, ViaLayerSpanReport, ViaLayerTransitionClass, ViaLayerTransitionReport,
    check_cardinal_rect_pad_board_clearance, check_circular_pad_board_clearance,
    check_rect_pad_board_clearance, check_trace_board_clearance,
    check_trace_cardinal_rect_pad_clearance, check_trace_clearance,
    check_trace_convex_board_clearance, check_trace_orthogonal_board_clearance,
    check_trace_pad_clearance, check_trace_rect_pad_clearance, check_trace_via_clearance,
    check_trace_via_drill_clearance, check_via_drill_board_clearance,
};
pub use provenance::{
    ConstructionStamp, PathProvenance, PathSourceFormat, SourceGrid, SourceLengthUnit,
};
pub use routing::{
    ConstantFeedTimeReport, DifferentialPairSkewReport, LengthMatchProblem,
    MeanderCandidatePlacementReport, MeanderError, MeanderObstacle, MeanderPlacementCandidate,
    MeanderPlacementReport, MeanderPlacementSlot, MultiDetourMeander, NonUniformDetourMeander,
    ObstacleAwareDetourMeander, RouteCertificationError, SingleDetourMeander,
    build_alternating_detour_meander, build_length_match_problem, build_multi_detour_meander,
    build_nonuniform_detour_meander, build_obstacle_aware_detour_meander,
    build_single_detour_meander, certify_constant_feed_time, certify_differential_pair_skew,
    certify_length_extension, classify_meander_candidate_slots, classify_meander_placement_slots,
};
pub use segment::{Axis, LinePathSegment, LinePathSegmentFacts, SegmentParameterOrder};
pub use solve::{
    PcbConstraintSet, RectangularRegion, ToolpathConstraintSet, bezier_offset_sample_constraints,
    center_clearance_squared_constraint, constant_feed_time_equation,
    differential_pair_skew_equation, length_match_equation, rectangular_difference_area_equation,
    rectangular_region_area_equation, rectangular_region_containment_constraints,
};
pub use specctra::{
    SpecctraGridRouteRecords, SpecctraGridTraceRecord, SpecctraGridViaRecord, SpecctraImportError,
    SpecctraLayerAlias, SpecctraNetAlias, SpecctraParseError, SpecctraRoute, SpecctraTraceRecord,
    SpecctraViaRecord, export_specctra_trace_record, export_specctra_via_record,
    import_specctra_text_route, import_specctra_trace_record, import_specctra_via_record,
    parse_specctra_grid_route_records, parse_specctra_grid_trace_records,
    serialize_specctra_grid_route_records, serialize_specctra_grid_trace_records,
    serialize_specctra_grid_via_records, specctra_grid_trace_record, specctra_grid_via_record,
};
pub use swept::{SweptLineSegment, SweptLineSegmentFacts};
pub use tangent::{
    G1ChainCertificationReport, G1JoinProblem, TangentAlignment, TangentAlignmentProblem,
    TangentChainReport, TangentJoinClass, TangentJoinReport, TangentSpan, build_g1_join_problem,
    build_oriented_tangent_alignment_problem, build_tangent_alignment_problem, certify_g1_chain,
    certify_g1_join_candidate, certify_tangent_alignment_candidate, classify_tangent_alignment,
    classify_tangent_chain, classify_tangent_join, tangent_cross, tangent_dot,
    tangent_norm_squared,
};
