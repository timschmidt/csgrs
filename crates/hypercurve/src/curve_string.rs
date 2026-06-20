//! Ordered open curve strings.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::{compare_reals, in_closed_unit_interval, is_zero, real_sign};
use crate::{
    ArcArcIntersection, BulgeVertex2, CircularArc2, Classification, CurveError, CurvePolicy,
    CurveResult, IntersectionKind, LineArcIntersection, LineArcOrder, LineLineIntersection,
    LineSeg2, LineSide, ParamRange, Point2, PreparedRegionView2, Region2, RegionContourRole,
    RegionPointLocation, RetainedTopologyStatus, Segment2, SegmentIntersection, SegmentKind,
    SegmentKindCounts, UncertaintyReason,
};

/// One segment-pair event between two curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersection {
    /// Segment index in the first curve string.
    pub a_segment_index: usize,
    /// Segment index in the second curve string.
    pub b_segment_index: usize,
    /// Primitive family of the first source segment.
    pub a_segment_kind: SegmentKind,
    /// Primitive family of the second source segment.
    pub b_segment_kind: SegmentKind,
    /// Exact start point of the first source segment.
    pub a_segment_start_point: Point2,
    /// Exact end point of the first source segment.
    pub a_segment_end_point: Point2,
    /// Exact start point of the second source segment.
    pub b_segment_start_point: Point2,
    /// Exact end point of the second source segment.
    pub b_segment_end_point: Point2,
    /// Segment relation for this pair.
    pub relation: SegmentIntersection,
}

/// Query path used to collect curve-string intersections.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringIntersectionQueryPath2 {
    /// Intersections were collected with transient broad-phase boxes.
    Direct,
    /// Intersections were collected through caller-supplied prepared views.
    Prepared,
}

/// Predicate/filter stage reached by a curve-string intersection query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringIntersectionPredicatePath2 {
    /// No segment-pair candidates existed.
    NoCandidates,
    /// All candidates were eliminated by decided disjoint broad-phase boxes.
    AabbOnly,
    /// At least one candidate reached exact segment-pair intersection predicates.
    ExactSegmentPredicates,
}

/// Prepared-cache evidence consumed by a curve-string intersection query.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersectionPreparedCacheReport2 {
    first: CurveStringPreparedCacheAudit2,
    second: CurveStringPreparedCacheAudit2,
}

/// Per-operand prepared curve-string cache inventory.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurveStringPreparedCacheAudit2 {
    freshness: CurveStringPreparedCacheFreshness2,
    prepared_segment_count: usize,
    prepared_segment_kind_counts: SegmentKindCounts,
    decided_segment_box_count: usize,
    undecided_segment_box_count: usize,
    curve_box_decided: bool,
}

/// Freshness claim for prepared curve-string cache evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringPreparedCacheFreshness2 {
    /// Prepared cache borrows the current source segments for this query.
    BorrowedCurrentSource,
}

/// Report for a curve-string intersection query.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersectionReport2 {
    first_segment_count: usize,
    second_segment_count: usize,
    first_segment_kind_counts: SegmentKindCounts,
    second_segment_kind_counts: SegmentKindCounts,
    first_decided_segment_box_count: usize,
    second_decided_segment_box_count: usize,
    first_undecided_segment_box_count: usize,
    second_undecided_segment_box_count: usize,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    intersection_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    query_path: CurveStringIntersectionQueryPath2,
    predicate_path: CurveStringIntersectionPredicatePath2,
    prepared_cache_report: Option<CurveStringIntersectionPreparedCacheReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing curve-string intersection collection.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersectionResult2 {
    intersections: Vec<CurveStringIntersection>,
    report: CurveStringIntersectionReport2,
}

/// Endpoint selector for open curve-string editing reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringEndpoint2 {
    /// First point of the curve string.
    Start,
    /// Final point of the curve string.
    End,
}

/// Exact endpoint-connectivity status for two curve strings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringEndpointConnectionStatus2 {
    /// Endpoints are certified identical and can be consumed as native topology.
    NativeExact,
    /// Endpoints are certified distinct.
    Disconnected,
    /// The active policy could not decide whether the endpoints are identical.
    Unresolved(UncertaintyReason),
}

/// Exact predicate path used to classify one endpoint pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringEndpointConnectionPredicatePath2 {
    /// Squared endpoint distance was proven exactly zero.
    ExactSquaredDistanceZero,
    /// Squared endpoint distance was proven exactly nonzero.
    ExactSquaredDistanceNonzero,
    /// The active policy could not decide the squared endpoint distance sign.
    UnresolvedSquaredDistanceSign,
}

/// Report for one tested endpoint pair.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringEndpointConnectionReport2 {
    first_endpoint: CurveStringEndpoint2,
    second_endpoint: CurveStringEndpoint2,
    first_point: Point2,
    second_point: Point2,
    distance_squared: crate::Real,
    predicate_path: CurveStringEndpointConnectionPredicatePath2,
    status: CurveStringEndpointConnectionStatus2,
}

/// Orientation selected when two open curve strings are linked.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLinkKind2 {
    /// `first.end == second.start`; output is `first + second`.
    FirstEndToSecondStart,
    /// `first.end == second.end`; output is `first + reverse(second)`.
    FirstEndToSecondEnd,
    /// `first.start == second.start`; output is `reverse(first) + second`.
    FirstStartToSecondStart,
    /// `first.start == second.end`; output is `second + first`.
    FirstStartToSecondEnd,
}

/// Input curve string that contributed one linked output segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLinkSourceInput2 {
    /// Segment came from the first input curve string.
    First,
    /// Segment came from the second input curve string.
    Second,
}

/// Source provenance for one segment emitted by a link operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLinkOutputSegmentReport2 {
    output_segment_index: usize,
    source_input: CurveStringLinkSourceInput2,
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    output_segment_kind: SegmentKind,
    reversed: bool,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    output_start_point: Point2,
    output_end_point: Point2,
}

/// Report for an auto-link attempt between two open curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLinkReport2 {
    stage: CurveStringLinkStage2,
    predicate_path: CurveStringLinkPredicatePath2,
    kind: CurveStringLinkKind2,
    endpoint_report: CurveStringEndpointConnectionReport2,
    first_segment_count: usize,
    first_segment_kind_counts: SegmentKindCounts,
    second_segment_count: usize,
    second_segment_kind_counts: SegmentKindCounts,
    endpoint_pair_count: usize,
    exact_endpoint_pair_count: usize,
    disconnected_endpoint_pair_count: usize,
    unresolved_endpoint_pair_count: usize,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    output_segments: Vec<CurveStringLinkOutputSegmentReport2>,
    status: RetainedTopologyStatus,
}

/// Furthest exact stage reached by an endpoint-link attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLinkStage2 {
    /// Endpoint pairs were classified to choose a unique exact link.
    EndpointSelection,
    /// Linked output segments were materialized with retained source ownership.
    SegmentMaterialization,
}

/// Exact predicate path used while selecting a pairwise endpoint link.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLinkPredicatePath2 {
    /// All four endpoint pairs were classified by exact endpoint-connection predicates.
    ExhaustiveEndpointPairClassification,
}

/// A linked open curve string with retained endpoint provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct LinkedCurveString2 {
    curve_string: CurveString2,
    report: CurveStringLinkReport2,
}

/// Report for a pairwise endpoint-link attempt, including non-materialized cases.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLinkAttemptReport2 {
    stage: CurveStringLinkStage2,
    predicate_path: CurveStringLinkPredicatePath2,
    selected_kind: Option<CurveStringLinkKind2>,
    selected_endpoint_report: Option<CurveStringEndpointConnectionReport2>,
    first_segment_count: usize,
    first_segment_kind_counts: SegmentKindCounts,
    second_segment_count: usize,
    second_segment_kind_counts: SegmentKindCounts,
    endpoint_pair_count: usize,
    exact_endpoint_pair_count: usize,
    disconnected_endpoint_pair_count: usize,
    unresolved_endpoint_pair_count: usize,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    output_segments: Vec<CurveStringLinkOutputSegmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing pairwise endpoint linking.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLinkAttemptResult2 {
    linked: Option<LinkedCurveString2>,
    report: CurveStringLinkAttemptReport2,
}

/// One ordered-chain link step in a batch open-path link operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOrderedLinkStepReport2 {
    accumulated_source_indices: Vec<usize>,
    next_source_index: usize,
    link_attempt_report: Option<CurveStringLinkAttemptReport2>,
    link_report: Option<CurveStringLinkReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Report for linking an ordered sequence of open curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOrderedLinkReport2 {
    stage: CurveStringOrderedLinkStage2,
    predicate_path: CurveStringOrderedLinkPredicatePath2,
    source_curve_string_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    attempted_link_step_count: usize,
    materialized_link_step_count: usize,
    blocked_link_step_count: usize,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    output_source_indices: Vec<usize>,
    steps: Vec<CurveStringOrderedLinkStepReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by ordered open-chain linking.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringOrderedLinkStage2 {
    /// Pairwise link steps were being selected and validated.
    StepLinking,
    /// The full ordered chain was materialized.
    ChainMaterialization,
}

/// Exact predicate path used while linking an ordered open-chain sequence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringOrderedLinkPredicatePath2 {
    /// Each ordered step used the exhaustive exact pairwise endpoint-link predicate path.
    RepeatedExhaustiveEndpointPairClassification,
}

/// Result of report-bearing ordered open-chain linking.
#[derive(Clone, Debug, PartialEq)]
pub struct OrderedLinkedCurveString2 {
    curve_string: Option<CurveString2>,
    report: CurveStringOrderedLinkReport2,
}

/// Report for connecting `first.end` to `second.start` with an exact line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringConnectReport2 {
    stage: CurveStringConnectStage2,
    predicate_path: CurveStringConnectPredicatePath2,
    kind: Option<CurveStringLinkKind2>,
    endpoint_report: CurveStringEndpointConnectionReport2,
    endpoint_reports: Vec<CurveStringEndpointConnectionReport2>,
    first_segment_count: usize,
    first_segment_kind_counts: SegmentKindCounts,
    second_segment_count: usize,
    second_segment_kind_counts: SegmentKindCounts,
    endpoint_pair_count: usize,
    exact_endpoint_pair_count: usize,
    disconnected_endpoint_pair_count: usize,
    unresolved_endpoint_pair_count: usize,
    connector_segment_index: Option<usize>,
    connector_start_point: Option<Point2>,
    connector_end_point: Option<Point2>,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    output_segments: Vec<CurveStringConnectOutputSegmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by explicit endpoint connection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringConnectStage2 {
    /// Endpoint pair evidence was classified to decide whether a connector is valid.
    EndpointSelection,
    /// Connector and output segment provenance were materialized.
    ConnectorMaterialization,
}

/// Exact predicate path used while selecting an explicit connector.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringConnectPredicatePath2 {
    /// The caller-selected endpoint pair was classified by exact endpoint predicates.
    SelectedEndpointPairClassification,
    /// All four endpoint pairs were classified before choosing the unique nearest disconnected pair.
    ExhaustiveEndpointPairDistanceSelection,
}

/// Source kind for one segment emitted by a connector operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringConnectSource2 {
    /// Segment came from the first input curve string.
    First,
    /// Segment came from the second input curve string.
    Second,
    /// Segment is the inserted exact connector line.
    Connector,
}

/// Source provenance for one segment emitted by a connector operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringConnectOutputSegmentReport2 {
    output_segment_index: usize,
    source: CurveStringConnectSource2,
    source_segment_index: Option<usize>,
    source_segment_kind: Option<SegmentKind>,
    output_segment_kind: SegmentKind,
    reversed: bool,
    source_segment_start_point: Option<Point2>,
    source_segment_end_point: Option<Point2>,
    output_start_point: Point2,
    output_end_point: Point2,
}

/// A connected open curve string with retained connector provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct ConnectedCurveString2 {
    curve_string: Option<CurveString2>,
    report: CurveStringConnectReport2,
}

/// One retained source run emitted by a line-merge operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLineMergeSpanReport2 {
    source_start_segment_index: usize,
    source_end_segment_index: usize,
    source_segment_indices: Vec<usize>,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_index: usize,
    output_segment_kind: SegmentKind,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Report for exact adjacent-line merging on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLineMergeReport2 {
    stage: CurveStringLineMergeStage2,
    predicate_path: CurveStringLineMergePredicatePath2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    adjacent_pair_count: usize,
    merged_pair_count: usize,
    preserved_pair_count: usize,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    spans: Vec<CurveStringLineMergeSpanReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Exact predicate path used while classifying adjacent line-merge pairs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLineMergePredicatePath2 {
    /// Adjacent line candidates were classified by exact support and direction predicates.
    ExactLineSupportAndDirection,
}

/// Furthest exact stage reached by adjacent line merging.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLineMergeStage2 {
    /// Adjacent segment families, collinearity, and direction predicates were being classified.
    PairClassification,
    /// Output segments and retained source runs were materialized.
    SegmentMaterialization,
}

/// Result of report-bearing adjacent-line merging.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLineMergeResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringLineMergeReport2,
}

/// One exact reversed duplicate pair removed from an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringReversedDuplicatePairReport2 {
    first_source_segment_index: usize,
    second_source_segment_index: usize,
    predicate_path: CurveStringDeduplicatePredicatePath2,
    first_source_segment_kind: SegmentKind,
    second_source_segment_kind: SegmentKind,
    first_start_point: Point2,
    first_end_point: Point2,
    second_start_point: Point2,
    second_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// One retained source segment emitted by adjacent reversed-duplicate removal.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringDeduplicateRetainedSegmentReport2 {
    output_segment_index: usize,
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    output_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Report for exact adjacent reversed-duplicate removal on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringDeduplicateReport2 {
    stage: CurveStringDeduplicateStage2,
    predicate_path: CurveStringDeduplicatePredicatePath2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    retained_source_segment_indices: Vec<usize>,
    retained_segments: Vec<CurveStringDeduplicateRetainedSegmentReport2>,
    removed_pairs: Vec<CurveStringReversedDuplicatePairReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Exact predicate path used while removing adjacent reversed duplicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringDeduplicatePredicatePath2 {
    /// Adjacent candidates were cancelled only when one exact segment equaled the other's reversal.
    ExactReversedSegmentEquality,
}

/// Furthest exact stage reached by adjacent reversed-duplicate removal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringDeduplicateStage2 {
    /// Adjacent reversed duplicate pairs were being detected and cancelled.
    PairCancellation,
    /// Retained output segments were materialized.
    SegmentMaterialization,
}

/// Result of report-bearing exact adjacent reversed-duplicate removal.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringDeduplicateResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringDeduplicateReport2,
}

/// Report for extending one open curve-string endpoint to an exact target point.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringExtendReport2 {
    stage: CurveStringExtendStage2,
    predicate_path: CurveStringExtendPredicatePath2,
    endpoint: CurveStringEndpoint2,
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    output_segment_index: Option<usize>,
    output_segment_kind: Option<SegmentKind>,
    output_segment_start_point: Option<Point2>,
    output_segment_end_point: Option<Point2>,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_endpoint_point: Point2,
    target_point: Point2,
    source_param: Option<Real>,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Exact predicate path used to validate one endpoint extension target.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringExtendPredicatePath2 {
    /// Line target was certified on support and outside the selected endpoint.
    LineSupportOutsideEndpoint,
    /// Line target was certified off the source line support.
    LineTargetOffSupport,
    /// Line target was on support but not outside the selected endpoint.
    LineTargetNotOutsideEndpoint,
    /// Line support or outside-parameter predicates were unresolved.
    LineTargetUnresolved,
    /// Arc target was certified on the same circle and the extended sweep replay kept source points.
    ArcSameCircleSweepReplay,
    /// Arc target was certified off the source circle.
    ArcTargetOffCircle,
    /// Arc target was already on the finite source arc sweep.
    ArcTargetAlreadyOnSweep,
    /// Arc target passed circle/sweep checks but replay did not preserve source arc evidence.
    ArcSweepReplayRejected,
    /// Arc circle, sweep, or replay predicates were unresolved.
    ArcTargetUnresolved,
}

/// Furthest exact stage reached by endpoint extension.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringExtendStage2 {
    /// Target support, parameter, and retained sweep evidence were being validated.
    TargetValidation,
    /// The endpoint segment was materialized with exact target geometry.
    SegmentMaterialization,
}

/// Result of a report-bearing open curve-string extension.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringExtendResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringExtendReport2,
}

/// Report for a line-line chamfer at one open curve-string vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringChamferReport2 {
    input_path: CurveStringChamferInputPath2,
    stage: CurveStringChamferStage2,
    predicate_path: CurveStringChamferPredicatePath2,
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_segment_start_point: Point2,
    previous_segment_end_point: Point2,
    next_segment_start_point: Point2,
    next_segment_end_point: Point2,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    previous_cut_point: Option<Point2>,
    next_cut_point: Option<Point2>,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    chamfer_segment_index: Option<usize>,
    chamfer_segment_kind: Option<SegmentKind>,
    chamfer_segment_start_point: Option<Point2>,
    chamfer_segment_end_point: Option<Point2>,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Exact predicate path used to validate one line-line chamfer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringChamferPredicatePath2 {
    /// Both adjacent segments were lines and both cut parameters were strict interior values.
    LineLineStrictInteriorParameters,
    /// At least one adjacent segment is not a line, so this chamfer slice cannot materialize.
    UnsupportedSegmentFamily,
    /// A point-supplied cut could not be certified on its selected source line segment.
    PointCutNotOnSourceSegment,
    /// A cut parameter was on or outside the valid strict interior range.
    CutParameterNotStrictInterior,
    /// Cut-parameter ordering or point-support predicates were unresolved.
    CutValidationUnresolved,
}

/// Input path used by a report-bearing line-line chamfer operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringChamferInputPath2 {
    /// Cut points were supplied by exact segment parameters.
    Parameters,
    /// Cut points were supplied directly as exact points.
    Points,
}

/// Furthest exact stage reached by a line-line chamfer attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringChamferStage2 {
    /// Input segment family and cut-parameter evidence were being validated.
    InputValidation,
    /// Adjacent source ranges and the inserted chamfer segment were materialized.
    SegmentMaterialization,
}

/// Result of a report-bearing line-line chamfer operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringChamferResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringChamferReport2,
}

/// Report for a line-line fillet at one open curve-string vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringFilletReport2 {
    input_path: CurveStringFilletInputPath2,
    stage: CurveStringFilletStage2,
    predicate_path: CurveStringFilletPredicatePath2,
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_segment_start_point: Point2,
    previous_segment_end_point: Point2,
    next_segment_start_point: Point2,
    next_segment_end_point: Point2,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    previous_tangent_point: Option<Point2>,
    next_tangent_point: Option<Point2>,
    center: Option<Point2>,
    radius_squared: Option<Real>,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    fillet_segment_index: Option<usize>,
    fillet_segment_kind: Option<SegmentKind>,
    fillet_segment_start_point: Option<Point2>,
    fillet_segment_end_point: Option<Point2>,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Exact predicate path used to validate one line-line fillet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringFilletPredicatePath2 {
    /// Line-line tangent, nonzero/equal-radius, and orientation predicates all passed.
    LineLineTangentArc,
    /// At least one adjacent segment is not a line, so this fillet slice cannot materialize.
    UnsupportedSegmentFamily,
    /// A point-supplied tangent could not be certified on its selected source line segment.
    TangentPointNotOnSourceSegment,
    /// A tangent parameter was on or outside the valid strict interior range.
    TangentParameterNotStrictInterior,
    /// Tangent-parameter ordering or point-support predicates were unresolved.
    TangentValidationUnresolved,
    /// The radius was certified zero.
    ZeroRadius,
    /// The two tangent radii were certified unequal.
    RadiusMismatch,
    /// Radius sign or equality predicates were unresolved.
    RadiusValidationUnresolved,
    /// Tangency or traversal orientation predicates rejected the fillet.
    TangencyOrOrientationRejected,
    /// Tangency or traversal orientation predicates were unresolved.
    TangencyOrOrientationUnresolved,
}

/// Input path used by a report-bearing line-line fillet operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringFilletInputPath2 {
    /// Tangent points were supplied by exact segment parameters.
    Parameters,
    /// Tangent points were supplied directly as exact points.
    Points,
}

/// Furthest exact stage reached by a line-line fillet attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringFilletStage2 {
    /// Input segment family and tangent-parameter evidence were being validated.
    InputValidation,
    /// Radius, tangency, and orientation predicates were being validated.
    RadiusAndTangencyValidation,
    /// Adjacent source ranges and the inserted fillet arc were materialized.
    ArcMaterialization,
}

/// Result of a report-bearing line-line fillet operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringFilletResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringFilletReport2,
}

/// Segment-local retained trim point on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimPoint2 {
    segment_index: usize,
    param: Real,
}

/// Report for one source segment range retained by a trim attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimSegmentReport2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    output_segment_index: Option<usize>,
    output_segment_kind: Option<SegmentKind>,
    output_segment_start_point: Option<Point2>,
    output_segment_end_point: Option<Point2>,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    range_start_point: Option<Point2>,
    range_end_point: Option<Point2>,
    status: RetainedTopologyStatus,
}

/// Report for an open curve-string trim attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimReport2 {
    input_path: CurveStringTrimInputPath2,
    predicate_path: CurveStringTrimPredicatePath2,
    start: CurveStringTrimPoint2,
    end: CurveStringTrimPoint2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Input path used by a report-bearing open curve-string trim.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringTrimInputPath2 {
    /// Trim boundaries were supplied by exact segment parameters.
    Parameters,
    /// Trim boundaries were supplied directly as exact points.
    Points,
}

/// Exact predicate path used while trimming an open curve string.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringTrimPredicatePath2 {
    /// Segment ranges were ordered and materialized from retained exact segment parameters.
    ExactParameterRange,
    /// Segment ranges were ordered and materialized from exact located path-point witnesses.
    ExactLocatedPointRange,
}

/// Result of a report-bearing open curve-string trim.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringTrimReport2,
}

/// One point witness where a cutter intersects the trimmed curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimHit2 {
    source_segment_index: usize,
    cutter_segment_index: usize,
    source_segment_kind: SegmentKind,
    cutter_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    cutter_segment_start_point: Point2,
    cutter_segment_end_point: Point2,
    source_param: Real,
    cutter_param: Real,
    point: Point2,
    kind: IntersectionKind,
}

/// Intersection query path used to collect curve-trim split evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringCurveTrimQueryPath2 {
    /// Intersections were collected by preparing transient broad-phase data.
    Direct,
    /// Intersections were collected through caller-supplied prepared views.
    Prepared,
}

/// Furthest exact stage reached by a trim-by-cutter-curve attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringCurveTrimStage2 {
    /// Cutter intersections were collected, but unique trim hits were not selected.
    HitSelection,
    /// Unique cutter hits were selected and point/range trim materialization was attempted.
    RangeMaterialization,
}

/// Report for a trim whose boundaries come from cutter curve intersections.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimReport2 {
    start_hits: Vec<CurveStringCurveTrimHit2>,
    end_hits: Vec<CurveStringCurveTrimHit2>,
    start_intersection_report: CurveStringIntersectionReport2,
    end_intersection_report: CurveStringIntersectionReport2,
    trim_report: Option<CurveStringTrimReport2>,
    query_path: CurveStringCurveTrimQueryPath2,
    stage: CurveStringCurveTrimStage2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing trim by two cutter curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringCurveTrimReport2,
}

/// One exact boundary hit used while trimming an open curve string by a region.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimHit2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    region_contour_role: RegionContourRole,
    region_contour_index: usize,
    region_segment_index: usize,
    region_segment_kind: SegmentKind,
    region_segment_start_point: Point2,
    region_segment_end_point: Point2,
    point: Point2,
    source_param: Real,
    region_param: Real,
    kind: IntersectionKind,
}

/// One retained source interval classified during trim-by-region.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimIntervalReport2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    range_start_point: Point2,
    range_end_point: Point2,
    representative_point: Option<Point2>,
    location: Option<RegionPointLocation>,
    output_curve_string_index: Option<usize>,
    output_segment_index: Option<usize>,
    output_segment_kind: Option<SegmentKind>,
    output_segment_start_point: Option<Point2>,
    output_segment_end_point: Option<Point2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Query path used to trim an open curve string by a region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringRegionTrimQueryPath2 {
    /// Region boundary hits and classifications used transient broad-phase data.
    Direct,
    /// Region boundary hits and classifications reused prepared region caches.
    Prepared,
}

/// Exact predicate family used while collecting trim-by-region boundary hits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringRegionTrimBoundaryPredicatePath2 {
    /// Source/region segment pairs were filtered by AABB before exact segment predicates.
    AabbFilteredExactSegmentIntersections,
}

/// Prepared-cache evidence consumed by a trim-by-region query.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimPreparedCacheReport2 {
    source: CurveStringPreparedCacheAudit2,
    region: RegionTrimPreparedCacheAudit2,
}

/// Per-region prepared cache inventory for trim-by-region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RegionTrimPreparedCacheAudit2 {
    freshness: CurveStringPreparedCacheFreshness2,
    prepared_contour_count: usize,
    prepared_material_segment_count: usize,
    prepared_material_segment_kind_counts: SegmentKindCounts,
    prepared_hole_segment_count: usize,
    prepared_hole_segment_kind_counts: SegmentKindCounts,
    prepared_segment_count: usize,
    prepared_segment_kind_counts: SegmentKindCounts,
    decided_segment_box_count: usize,
    undecided_segment_box_count: usize,
    region_box_decided: bool,
}

/// Furthest exact stage reached by a trim-by-region attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringRegionTrimStage2 {
    /// Region boundary intersections were being collected.
    BoundaryCollection,
    /// Source intervals were split and classified against the region.
    IntervalClassification,
    /// Classified intervals were materialized into output curve strings.
    OutputMaterialization,
}

/// Report for retaining portions of an open curve string inside a region.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimReport2 {
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    region_material_contour_count: usize,
    region_hole_contour_count: usize,
    region_material_segment_count: usize,
    region_hole_segment_count: usize,
    region_material_segment_kind_counts: SegmentKindCounts,
    region_hole_segment_kind_counts: SegmentKindCounts,
    boundary_predicate_path: CurveStringRegionTrimBoundaryPredicatePath2,
    boundary_candidate_pair_count: usize,
    boundary_skipped_aabb_pair_count: usize,
    boundary_tested_pair_count: usize,
    boundary_hit_count: usize,
    boundary_point_relation_count: usize,
    boundary_overlap_relation_count: usize,
    boundary_uncertain_relation_count: usize,
    interval_candidate_count: usize,
    interval_classification_count: usize,
    boundary_hits: Vec<CurveStringRegionTrimHit2>,
    interval_reports: Vec<CurveStringRegionTrimIntervalReport2>,
    output_curve_string_count: Option<usize>,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    query_path: CurveStringRegionTrimQueryPath2,
    prepared_cache_report: Option<CurveStringRegionTrimPreparedCacheReport2>,
    stage: CurveStringRegionTrimStage2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing trim-by-region operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimResult2 {
    curve_strings: Vec<CurveString2>,
    report: CurveStringRegionTrimReport2,
}

/// An ordered sequence of connected native segments.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveString2 {
    segments: Vec<Segment2>,
}

impl CurveString2 {
    /// Constructs a curve string from validated connected segments.
    pub fn try_new(segments: Vec<Segment2>) -> CurveResult<Self> {
        if segments.is_empty() {
            return Err(CurveError::EmptyCurveString);
        }

        for segment in &segments {
            match segment
                .start()
                .distance_squared(segment.end())
                .zero_status()
            {
                hyperreal::ZeroKnowledge::Zero => return Err(CurveError::ZeroLengthLine),
                hyperreal::ZeroKnowledge::NonZero | hyperreal::ZeroKnowledge::Unknown => {}
            }
        }

        for adjacent in segments.windows(2) {
            let distance = adjacent[0].end().distance_squared(adjacent[1].start());
            match distance.zero_status() {
                hyperreal::ZeroKnowledge::Zero => {}
                hyperreal::ZeroKnowledge::NonZero => {
                    return Err(CurveError::DisconnectedCurveString);
                }
                hyperreal::ZeroKnowledge::Unknown => {
                    return Err(CurveError::AmbiguousCurveStringConnection);
                }
            }
        }

        Ok(Self { segments })
    }

    /// Constructs a curve string without checking connectivity.
    pub const fn new_unchecked(segments: Vec<Segment2>) -> Self {
        Self { segments }
    }

    /// Constructs an open curve string from exact bulge vertices.
    pub fn from_bulge_vertices(vertices: &[BulgeVertex2]) -> CurveResult<Self> {
        if vertices.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(vertices.len() - 1);
        for adjacent in vertices.windows(2) {
            segments.push(adjacent[0].segment_to(&adjacent[1])?);
        }
        Self::try_new(segments)
    }

    /// Returns the segments in order.
    pub fn segments(&self) -> &[Segment2] {
        &self.segments
    }

    /// Consumes the curve string and returns its segments.
    pub fn into_segments(self) -> Vec<Segment2> {
        self.segments
    }

    /// Returns the segment count.
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// Returns true when there are no segments.
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Returns the first point of the curve string.
    pub fn start(&self) -> Option<&Point2> {
        self.segments.first().map(Segment2::start)
    }

    /// Returns the final point of the curve string.
    pub fn end(&self) -> Option<&Point2> {
        self.segments.last().map(Segment2::end)
    }

    /// Reports whether one endpoint pair is exactly connected.
    ///
    /// This is the small provenance unit used by open-path merge/connect/link
    /// editing. It compares endpoint squared distance through the active exact
    /// policy and records the result instead of letting callers introduce a
    /// local snapping tolerance. A disconnected report is still useful evidence:
    /// it says the topology branch was decided exactly, but no connection was
    /// available at this endpoint pair.
    pub fn endpoint_connection_report(
        &self,
        other: &Self,
        first_endpoint: CurveStringEndpoint2,
        second_endpoint: CurveStringEndpoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringEndpointConnectionReport2> {
        let first_point = self.endpoint(first_endpoint)?;
        let second_point = other.endpoint(second_endpoint)?;
        let distance_squared = first_point.distance_squared(second_point);
        let (predicate_path, status) = match is_zero(&distance_squared, policy) {
            Some(true) => (
                CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceZero,
                CurveStringEndpointConnectionStatus2::NativeExact,
            ),
            Some(false) => (
                CurveStringEndpointConnectionPredicatePath2::ExactSquaredDistanceNonzero,
                CurveStringEndpointConnectionStatus2::Disconnected,
            ),
            None => (
                CurveStringEndpointConnectionPredicatePath2::UnresolvedSquaredDistanceSign,
                CurveStringEndpointConnectionStatus2::Unresolved(UncertaintyReason::RealSign),
            ),
        };

        Ok(CurveStringEndpointConnectionReport2 {
            first_endpoint,
            second_endpoint,
            first_point: first_point.clone(),
            second_point: second_point.clone(),
            distance_squared,
            predicate_path,
            status,
        })
    }

    /// Links two open curve strings when exactly one endpoint pair is certified.
    ///
    /// The four endpoint pairings are tested exactly. A result is materialized
    /// only when one and only one pairing is
    /// [`RetainedTopologyStatus::NativeExact`]; multiple exact pairings are
    /// ambiguous open-chain topology, and any unresolved pairing prevents
    /// choosing a unique link. Certified disconnected inputs return
    /// `Decided(None)` so higher-level tools can decide whether to create an
    /// explicit connector segment rather than silently snapping.
    pub fn link_connected_endpoints(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::Classification<Option<LinkedCurveString2>>> {
        Ok(self
            .link_connected_endpoints_with_report(other, policy)?
            .into_linked_curve_string_classification())
    }

    /// Links two open curve strings by certified endpoints and retains a report.
    ///
    /// This report-bearing variant preserves endpoint evidence and exact
    /// blockers for disconnected, ambiguous, or unresolved attempts instead of
    /// returning only `None` or `Uncertain`.
    pub fn link_connected_endpoints_with_report(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringLinkAttemptResult2> {
        let reports = self.endpoint_link_reports(other, policy)?;
        let mut endpoint_summary = EndpointPairSummary {
            pair_count: reports.len(),
            ..EndpointPairSummary::default()
        };
        let mut exact = Vec::new();
        let mut unresolved = None;
        for (kind, report) in reports {
            endpoint_summary.add_status(report.status);
            match report.status {
                CurveStringEndpointConnectionStatus2::NativeExact => exact.push((kind, report)),
                CurveStringEndpointConnectionStatus2::Disconnected => {}
                CurveStringEndpointConnectionStatus2::Unresolved(reason) => {
                    unresolved = Some(reason);
                }
            }
        }

        if exact.len() > 1 {
            return Ok(CurveStringLinkAttemptResult2 {
                linked: None,
                report: CurveStringLinkAttemptReport2 {
                    stage: CurveStringLinkStage2::EndpointSelection,
                    predicate_path:
                        CurveStringLinkPredicatePath2::ExhaustiveEndpointPairClassification,
                    selected_kind: None,
                    selected_endpoint_report: None,
                    first_segment_count: self.len(),
                    first_segment_kind_counts: curve_string_segment_kind_counts(self),
                    second_segment_count: other.len(),
                    second_segment_kind_counts: curve_string_segment_kind_counts(other),
                    endpoint_pair_count: endpoint_summary.pair_count,
                    exact_endpoint_pair_count: endpoint_summary.exact_count,
                    disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
                    unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
                    output_segment_count: None,
                    output_segment_kind_counts: None,
                    output_segments: Vec::new(),
                    status: RetainedTopologyStatus::Unsupported,
                    blocker: Some(UncertaintyReason::Boundary),
                },
            });
        }
        if let Some(reason) = unresolved {
            return Ok(CurveStringLinkAttemptResult2 {
                linked: None,
                report: CurveStringLinkAttemptReport2 {
                    stage: CurveStringLinkStage2::EndpointSelection,
                    predicate_path:
                        CurveStringLinkPredicatePath2::ExhaustiveEndpointPairClassification,
                    selected_kind: None,
                    selected_endpoint_report: None,
                    first_segment_count: self.len(),
                    first_segment_kind_counts: curve_string_segment_kind_counts(self),
                    second_segment_count: other.len(),
                    second_segment_kind_counts: curve_string_segment_kind_counts(other),
                    endpoint_pair_count: endpoint_summary.pair_count,
                    exact_endpoint_pair_count: endpoint_summary.exact_count,
                    disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
                    unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
                    output_segment_count: None,
                    output_segment_kind_counts: None,
                    output_segments: Vec::new(),
                    status: retained_status_for_uncertainty(reason),
                    blocker: Some(reason),
                },
            });
        }
        let Some((kind, endpoint_report)) = exact.pop() else {
            return Ok(CurveStringLinkAttemptResult2 {
                linked: None,
                report: CurveStringLinkAttemptReport2 {
                    stage: CurveStringLinkStage2::EndpointSelection,
                    predicate_path:
                        CurveStringLinkPredicatePath2::ExhaustiveEndpointPairClassification,
                    selected_kind: None,
                    selected_endpoint_report: None,
                    first_segment_count: self.len(),
                    first_segment_kind_counts: curve_string_segment_kind_counts(self),
                    second_segment_count: other.len(),
                    second_segment_kind_counts: curve_string_segment_kind_counts(other),
                    endpoint_pair_count: endpoint_summary.pair_count,
                    exact_endpoint_pair_count: endpoint_summary.exact_count,
                    disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
                    unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
                    output_segment_count: None,
                    output_segment_kind_counts: None,
                    output_segments: Vec::new(),
                    status: RetainedTopologyStatus::Unsupported,
                    blocker: Some(UncertaintyReason::Boundary),
                },
            });
        };

        let curve_string = linked_curve_string(self, other, kind)?;
        let output_segment_count = Some(curve_string.len());
        let output_segment_kind_counts = Some(curve_string_segment_kind_counts(&curve_string));
        let output_segments = link_output_segment_reports(self, other, kind);
        let report = CurveStringLinkReport2 {
            stage: CurveStringLinkStage2::SegmentMaterialization,
            predicate_path: CurveStringLinkPredicatePath2::ExhaustiveEndpointPairClassification,
            kind,
            endpoint_report: endpoint_report.clone(),
            first_segment_count: self.len(),
            first_segment_kind_counts: curve_string_segment_kind_counts(self),
            second_segment_count: other.len(),
            second_segment_kind_counts: curve_string_segment_kind_counts(other),
            endpoint_pair_count: endpoint_summary.pair_count,
            exact_endpoint_pair_count: endpoint_summary.exact_count,
            disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
            unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
            output_segment_count,
            output_segment_kind_counts,
            output_segments: output_segments.clone(),
            status: RetainedTopologyStatus::NativeExact,
        };
        Ok(CurveStringLinkAttemptResult2 {
            linked: Some(LinkedCurveString2 {
                curve_string,
                report,
            }),
            report: CurveStringLinkAttemptReport2 {
                stage: CurveStringLinkStage2::SegmentMaterialization,
                predicate_path: CurveStringLinkPredicatePath2::ExhaustiveEndpointPairClassification,
                selected_kind: Some(kind),
                selected_endpoint_report: Some(endpoint_report),
                first_segment_count: self.len(),
                first_segment_kind_counts: curve_string_segment_kind_counts(self),
                second_segment_count: other.len(),
                second_segment_kind_counts: curve_string_segment_kind_counts(other),
                endpoint_pair_count: endpoint_summary.pair_count,
                exact_endpoint_pair_count: endpoint_summary.exact_count,
                disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
                unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
                output_segment_count,
                output_segment_kind_counts,
                output_segments,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Links an ordered sequence of open curve strings by certified endpoints.
    ///
    /// The caller supplies the intended chain order. Each next curve string is
    /// linked to the accumulated output through
    /// [`CurveString2::link_connected_endpoints`], so every step still requires
    /// one unique exact endpoint pair and keeps the pairwise link report. A
    /// disconnected, ambiguous, or unresolved step stops the batch and returns
    /// an explicit report instead of creating connectors or snapping endpoints.
    pub fn link_ordered_connected_endpoints(
        curve_strings: Vec<Self>,
        policy: &CurvePolicy,
    ) -> CurveResult<OrderedLinkedCurveString2> {
        let source_curve_string_count = curve_strings.len();
        let source_segment_kind_counts = curve_strings_segment_kind_counts(&curve_strings);
        let mut iter = curve_strings.into_iter().enumerate();
        let Some((first_index, mut accumulated)) = iter.next() else {
            return Err(CurveError::EmptyCurveString);
        };
        let mut accumulated_source_indices = vec![first_index];
        let mut steps = Vec::new();

        for (next_source_index, next_curve_string) in iter {
            let attempt =
                accumulated.link_connected_endpoints_with_report(&next_curve_string, policy)?;
            let link_attempt_report = attempt.report().clone();
            match attempt.into_linked_curve_string() {
                Some(linked) => {
                    let link_report = linked.report().clone();
                    let next_accumulated_source_indices = ordered_link_source_indices(
                        &accumulated_source_indices,
                        next_source_index,
                        link_report.kind(),
                    );
                    steps.push(CurveStringOrderedLinkStepReport2 {
                        accumulated_source_indices,
                        next_source_index,
                        link_attempt_report: Some(link_attempt_report),
                        link_report: Some(link_report),
                        status: RetainedTopologyStatus::NativeExact,
                        blocker: None,
                    });
                    accumulated = linked.into_curve_string();
                    accumulated_source_indices = next_accumulated_source_indices;
                }
                None => {
                    let status = link_attempt_report.status();
                    let blocker = link_attempt_report
                        .blocker()
                        .unwrap_or(UncertaintyReason::Unsupported);
                    let blocked_output_source_indices = accumulated_source_indices.clone();
                    steps.push(CurveStringOrderedLinkStepReport2 {
                        accumulated_source_indices,
                        next_source_index,
                        link_attempt_report: Some(link_attempt_report),
                        link_report: None,
                        status,
                        blocker: Some(blocker),
                    });
                    let attempted_link_step_count = steps.len();
                    let materialized_link_step_count = steps
                        .iter()
                        .filter(|step| step.status.is_native_exact())
                        .count();
                    let blocked_link_step_count =
                        attempted_link_step_count - materialized_link_step_count;
                    return Ok(OrderedLinkedCurveString2 {
                        curve_string: None,
                        report: CurveStringOrderedLinkReport2 {
                            stage: CurveStringOrderedLinkStage2::StepLinking,
                            predicate_path: CurveStringOrderedLinkPredicatePath2::
                                RepeatedExhaustiveEndpointPairClassification,
                            source_curve_string_count,
                            attempted_link_step_count,
                            materialized_link_step_count,
                            blocked_link_step_count,
                            output_segment_count: None,
                            source_segment_kind_counts,
                            output_segment_kind_counts: None,
                            output_source_indices: blocked_output_source_indices,
                            steps,
                            status,
                            blocker: Some(blocker),
                        },
                    });
                }
            }
        }

        Ok(OrderedLinkedCurveString2 {
            report: CurveStringOrderedLinkReport2 {
                stage: CurveStringOrderedLinkStage2::ChainMaterialization,
                predicate_path: CurveStringOrderedLinkPredicatePath2::
                    RepeatedExhaustiveEndpointPairClassification,
                source_curve_string_count,
                source_segment_kind_counts,
                attempted_link_step_count: steps.len(),
                materialized_link_step_count: steps.len(),
                blocked_link_step_count: 0,
                output_segment_count: Some(accumulated.len()),
                output_segment_kind_counts: Some(curve_string_segment_kind_counts(&accumulated)),
                output_source_indices: accumulated_source_indices,
                steps,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            curve_string: Some(accumulated),
        })
    }

    /// Links a borrowed ordered sequence of open curve strings by certified endpoints.
    ///
    /// This is the borrowed counterpart to
    /// [`CurveString2::link_ordered_connected_endpoints`]. It uses the same
    /// exact pairwise endpoint-linking pipeline and report types, while letting
    /// editing callers retain ownership of their source chains.
    pub fn link_ordered_connected_endpoints_borrowed(
        curve_strings: &[Self],
        policy: &CurvePolicy,
    ) -> CurveResult<OrderedLinkedCurveString2> {
        Self::link_ordered_connected_endpoints(curve_strings.to_vec(), policy)
    }

    /// Connects `self.end` to `other.start` with an exact line segment.
    ///
    /// This operation is the explicit-connector counterpart to
    /// [`CurveString2::link_connected_endpoints`]. It materializes only when
    /// the endpoints are certified distinct; already-equal endpoints should be
    /// linked instead, and unresolved endpoint equality remains an explicit
    /// blocker. No tolerance snap or hidden zero-length connector is introduced.
    pub fn connect_end_to_start_with_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        self.connect_end_to_start_with_line_with_report(other, policy)
    }

    /// Connects `self.end` to `other.start` with an exact line segment and retains evidence.
    ///
    /// This is the report-bearing entry point for the explicit-connector
    /// operation. It materializes only when the selected endpoints are
    /// certified distinct and records the endpoint predicate, inserted
    /// connector index, and output segment provenance.
    pub fn connect_end_to_start_with_line_with_report(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        self.connect_endpoints_with_line_with_report(
            other,
            CurveStringLinkKind2::FirstEndToSecondStart,
            policy,
        )
    }

    /// Connects a selected endpoint pair with an exact line segment.
    ///
    /// The selected endpoints are compared exactly first. Equal endpoints are
    /// blocked as a link case, unresolved equality stays unresolved, and only a
    /// certified positive endpoint gap materializes a connector.
    pub fn connect_endpoints_with_line(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        self.connect_endpoints_with_line_with_report(other, kind, policy)
    }

    /// Connects a selected endpoint pair with an exact line segment and retains evidence.
    ///
    /// The report records the selected endpoint relation, exact/disconnected/
    /// unresolved endpoint counts, connector placement, and full output segment
    /// provenance. Equal endpoints are blocked as a link case rather than
    /// producing a zero-length connector.
    pub fn connect_endpoints_with_line_with_report(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        let endpoint_report = self.endpoint_connection_report_for_kind(other, kind, policy)?;
        let endpoint_summary = EndpointPairSummary::from_report(&endpoint_report);
        match endpoint_report.status {
            CurveStringEndpointConnectionStatus2::NativeExact => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    endpoint_report.clone(),
                    vec![endpoint_report],
                    endpoint_summary,
                    CurveStringConnectPredicatePath2::SelectedEndpointPairClassification,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            CurveStringEndpointConnectionStatus2::Disconnected => {}
            CurveStringEndpointConnectionStatus2::Unresolved(reason) => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    endpoint_report.clone(),
                    vec![endpoint_report],
                    endpoint_summary,
                    CurveStringConnectPredicatePath2::SelectedEndpointPairClassification,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let (curve_string, connector_segment_index) = connected_curve_string(self, other, kind)?;
        let output_segment_count = Some(curve_string.len());
        let output_segment_kind_counts = Some(curve_string_segment_kind_counts(&curve_string));
        let output_segments = connect_output_segment_reports(self, other, kind)?;
        let (connector_start_point, connector_end_point) =
            connector_endpoint_points(&output_segments)?;
        let report = CurveStringConnectReport2 {
            stage: CurveStringConnectStage2::ConnectorMaterialization,
            predicate_path: CurveStringConnectPredicatePath2::SelectedEndpointPairClassification,
            kind: Some(kind),
            endpoint_report: endpoint_report.clone(),
            endpoint_reports: vec![endpoint_report],
            first_segment_count: self.len(),
            first_segment_kind_counts: curve_string_segment_kind_counts(self),
            second_segment_count: other.len(),
            second_segment_kind_counts: curve_string_segment_kind_counts(other),
            endpoint_pair_count: endpoint_summary.pair_count,
            exact_endpoint_pair_count: endpoint_summary.exact_count,
            disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
            unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
            connector_segment_index: Some(connector_segment_index),
            connector_start_point: Some(connector_start_point),
            connector_end_point: Some(connector_end_point),
            output_segment_count,
            output_segment_kind_counts,
            output_segments,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(ConnectedCurveString2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Connects the uniquely nearest certified-disconnected endpoint pair.
    ///
    /// All four endpoint pairs are inspected with exact squared-distance
    /// evidence. Existing exact endpoint equality is blocked as a link case;
    /// unresolved endpoint equality or unresolved distance ordering blocks the
    /// auto-choice; equal nearest distances are reported as boundary ambiguity.
    pub fn connect_nearest_endpoints_with_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        self.connect_nearest_endpoints_with_line_with_report(other, policy)
    }

    /// Connects the uniquely nearest certified-disconnected endpoint pair and retains evidence.
    ///
    /// All four endpoint pairs are inspected and recorded. Existing exact
    /// endpoint equality, unresolved equality, unresolved distance ordering, or
    /// tied nearest distances block materialization with explicit report status.
    pub fn connect_nearest_endpoints_with_line_with_report(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        let reports = self.endpoint_link_reports(other, policy)?;
        let mut candidates = Vec::new();
        let mut endpoint_summary = EndpointPairSummary {
            pair_count: reports.len(),
            ..EndpointPairSummary::default()
        };
        let mut endpoint_reports = Vec::with_capacity(reports.len());
        let mut exact_blocker = None;
        let mut unresolved_blocker = None;
        for (kind, report) in reports {
            endpoint_reports.push(report.clone());
            endpoint_summary.add_status(report.status);
            match report.status {
                CurveStringEndpointConnectionStatus2::NativeExact => {
                    if exact_blocker.is_none() {
                        exact_blocker = Some((kind, report));
                    }
                }
                CurveStringEndpointConnectionStatus2::Disconnected => {
                    candidates.push((kind, report));
                }
                CurveStringEndpointConnectionStatus2::Unresolved(reason) => {
                    if unresolved_blocker.is_none() {
                        unresolved_blocker = Some((kind, report, reason));
                    }
                }
            }
        }
        if let Some((kind, report)) = exact_blocker {
            return Ok(blocked_connected_curve_string(
                self,
                other,
                Some(kind),
                report,
                endpoint_reports.clone(),
                endpoint_summary,
                CurveStringConnectPredicatePath2::ExhaustiveEndpointPairDistanceSelection,
                RetainedTopologyStatus::Unsupported,
                Some(UncertaintyReason::Boundary),
            ));
        }
        if let Some((kind, report, reason)) = unresolved_blocker {
            return Ok(blocked_connected_curve_string(
                self,
                other,
                Some(kind),
                report,
                endpoint_reports.clone(),
                endpoint_summary,
                CurveStringConnectPredicatePath2::ExhaustiveEndpointPairDistanceSelection,
                RetainedTopologyStatus::Unresolved,
                Some(reason),
            ));
        }

        let (kind, endpoint_report) = match unique_nearest_endpoint_report(candidates, policy) {
            NearestEndpointChoice::Selected(kind, report) => (kind, report),
            NearestEndpointChoice::Ambiguous(kind, report) => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    report,
                    endpoint_reports.clone(),
                    endpoint_summary,
                    CurveStringConnectPredicatePath2::ExhaustiveEndpointPairDistanceSelection,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            NearestEndpointChoice::Unresolved(kind, report, reason) => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    report,
                    endpoint_reports.clone(),
                    endpoint_summary,
                    CurveStringConnectPredicatePath2::ExhaustiveEndpointPairDistanceSelection,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
            NearestEndpointChoice::Empty => return Err(CurveError::EmptyCurveString),
        };
        let (curve_string, connector_segment_index) = connected_curve_string(self, other, kind)?;
        let output_segment_count = Some(curve_string.len());
        let output_segment_kind_counts = Some(curve_string_segment_kind_counts(&curve_string));
        let output_segments = connect_output_segment_reports(self, other, kind)?;
        let (connector_start_point, connector_end_point) =
            connector_endpoint_points(&output_segments)?;
        let report = CurveStringConnectReport2 {
            stage: CurveStringConnectStage2::ConnectorMaterialization,
            predicate_path:
                CurveStringConnectPredicatePath2::ExhaustiveEndpointPairDistanceSelection,
            kind: Some(kind),
            endpoint_report: endpoint_report.clone(),
            endpoint_reports,
            first_segment_count: self.len(),
            first_segment_kind_counts: curve_string_segment_kind_counts(self),
            second_segment_count: other.len(),
            second_segment_kind_counts: curve_string_segment_kind_counts(other),
            endpoint_pair_count: endpoint_summary.pair_count,
            exact_endpoint_pair_count: endpoint_summary.exact_count,
            disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
            unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
            connector_segment_index: Some(connector_segment_index),
            connector_start_point: Some(connector_start_point),
            connector_end_point: Some(connector_end_point),
            output_segment_count,
            output_segment_kind_counts,
            output_segments,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(ConnectedCurveString2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Merges adjacent same-direction line segments when collinearity is certified.
    ///
    /// This is an explicit editing utility, not constructor normalization:
    /// source segment runs are retained in the report, mixed line/arc topology
    /// is preserved, and collinear reversals are not collapsed because they are
    /// real authored backtracking topology. If a line-line pair cannot be
    /// classified under the active policy, the operation returns an unresolved
    /// report instead of guessing a merge boundary.
    pub fn merge_adjacent_collinear_lines(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringLineMergeResult2> {
        let mut merged_segments = Vec::with_capacity(self.len());
        let mut spans = Vec::new();
        let mut adjacent_pair_count = 0_usize;
        let mut merged_pair_count = 0_usize;
        let mut preserved_pair_count = 0_usize;
        let mut current_segment = self
            .segments
            .first()
            .cloned()
            .ok_or(CurveError::EmptyCurveString)?;
        let mut current_start_index = 0_usize;

        for (next_index, next_segment) in self.segments.iter().enumerate().skip(1) {
            adjacent_pair_count += 1;
            match merge_adjacent_line_segments(&current_segment, next_segment, policy)? {
                Classification::Decided(Some(merged)) => {
                    merged_pair_count += 1;
                    current_segment = Segment2::Line(merged);
                }
                Classification::Decided(None) => {
                    preserved_pair_count += 1;
                    let output_segment_index = merged_segments.len();
                    let output_start_point = current_segment.start().clone();
                    let output_end_point = current_segment.end().clone();
                    merged_segments.push(current_segment);
                    spans.push(CurveStringLineMergeSpanReport2 {
                        source_start_segment_index: current_start_index,
                        source_end_segment_index: next_index - 1,
                        source_segment_indices: (current_start_index..next_index).collect(),
                        source_segment_kind_counts: segment_kind_counts_for_range(
                            &self.segments,
                            current_start_index..next_index,
                        ),
                        output_segment_index,
                        output_segment_kind: merged_segments[output_segment_index]
                            .structural_facts()
                            .kind,
                        output_start_point,
                        output_end_point,
                        status: RetainedTopologyStatus::NativeExact,
                    });
                    current_segment = next_segment.clone();
                    current_start_index = next_index;
                }
                Classification::Uncertain(reason) => {
                    return Ok(CurveStringLineMergeResult2 {
                        curve_string: None,
                        report: CurveStringLineMergeReport2 {
                            stage: CurveStringLineMergeStage2::PairClassification,
                            predicate_path:
                                CurveStringLineMergePredicatePath2::ExactLineSupportAndDirection,
                            source_segment_count: self.len(),
                            source_segment_kind_counts: curve_string_segment_kind_counts(self),
                            adjacent_pair_count,
                            merged_pair_count,
                            preserved_pair_count,
                            output_segment_count: None,
                            output_segment_kind_counts: None,
                            spans,
                            status: RetainedTopologyStatus::Unresolved,
                            blocker: Some(reason),
                        },
                    });
                }
            }
        }

        let output_segment_index = merged_segments.len();
        let output_start_point = current_segment.start().clone();
        let output_end_point = current_segment.end().clone();
        merged_segments.push(current_segment);
        spans.push(CurveStringLineMergeSpanReport2 {
            source_start_segment_index: current_start_index,
            source_end_segment_index: self.len() - 1,
            source_segment_indices: (current_start_index..self.len()).collect(),
            source_segment_kind_counts: segment_kind_counts_for_range(
                &self.segments,
                current_start_index..self.len(),
            ),
            output_segment_index,
            output_segment_kind: merged_segments[output_segment_index]
                .structural_facts()
                .kind,
            output_start_point,
            output_end_point,
            status: RetainedTopologyStatus::NativeExact,
        });

        let curve_string = CurveString2::try_new(merged_segments)?;
        let output_segment_kind_counts = curve_string_segment_kind_counts(&curve_string);
        Ok(CurveStringLineMergeResult2 {
            report: CurveStringLineMergeReport2 {
                stage: CurveStringLineMergeStage2::SegmentMaterialization,
                predicate_path: CurveStringLineMergePredicatePath2::ExactLineSupportAndDirection,
                source_segment_count: self.len(),
                source_segment_kind_counts: curve_string_segment_kind_counts(self),
                adjacent_pair_count,
                merged_pair_count,
                preserved_pair_count,
                output_segment_count: Some(curve_string.len()),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                spans,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            curve_string: Some(curve_string),
        })
    }

    /// Removes adjacent exact reversed duplicate segment pairs.
    ///
    /// This is a structural de-duplication utility for authored backtracking,
    /// not an overlap resolver: only `segment == next.reversed()` is removed.
    /// Same-support partial overlaps, same-direction repeats, and geometric
    /// coincidences with different segmentation remain intact for the
    /// arrangement pipeline. If every segment cancels, no empty `CurveString2`
    /// is materialized and the report carries an explicit boundary blocker.
    pub fn remove_adjacent_reversed_duplicates(
        &self,
    ) -> CurveResult<CurveStringDeduplicateResult2> {
        let mut retained: Vec<(usize, Segment2)> = Vec::with_capacity(self.len());
        let mut removed_pairs = Vec::new();

        for (source_index, segment) in self.segments.iter().cloned().enumerate() {
            if retained
                .last()
                .is_some_and(|(_, previous)| previous == &segment.reversed())
            {
                let (first_source_segment_index, first_segment) = retained
                    .pop()
                    .expect("retained stack should have a previous segment");
                removed_pairs.push(CurveStringReversedDuplicatePairReport2 {
                    first_source_segment_index,
                    second_source_segment_index: source_index,
                    predicate_path:
                        CurveStringDeduplicatePredicatePath2::ExactReversedSegmentEquality,
                    first_source_segment_kind: first_segment.structural_facts().kind,
                    second_source_segment_kind: segment.structural_facts().kind,
                    first_start_point: first_segment.start().clone(),
                    first_end_point: first_segment.end().clone(),
                    second_start_point: segment.start().clone(),
                    second_end_point: segment.end().clone(),
                    status: RetainedTopologyStatus::NativeExact,
                });
            } else {
                retained.push((source_index, segment));
            }
        }

        if retained.is_empty() {
            return Ok(CurveStringDeduplicateResult2 {
                curve_string: None,
                report: CurveStringDeduplicateReport2 {
                    stage: CurveStringDeduplicateStage2::PairCancellation,
                    predicate_path:
                        CurveStringDeduplicatePredicatePath2::ExactReversedSegmentEquality,
                    source_segment_count: self.len(),
                    source_segment_kind_counts: curve_string_segment_kind_counts(self),
                    output_segment_count: None,
                    output_segment_kind_counts: None,
                    retained_source_segment_indices: Vec::new(),
                    retained_segments: Vec::new(),
                    removed_pairs,
                    status: RetainedTopologyStatus::Unsupported,
                    blocker: Some(UncertaintyReason::Boundary),
                },
            });
        }

        let retained_source_segment_indices = retained
            .iter()
            .map(|(source_index, _)| *source_index)
            .collect::<Vec<_>>();
        let retained_segments = retained
            .iter()
            .enumerate()
            .map(|(output_segment_index, (source_segment_index, segment))| {
                CurveStringDeduplicateRetainedSegmentReport2 {
                    output_segment_index,
                    source_segment_index: *source_segment_index,
                    source_segment_kind: segment.structural_facts().kind,
                    output_segment_kind: segment.structural_facts().kind,
                    source_segment_start_point: segment.start().clone(),
                    source_segment_end_point: segment.end().clone(),
                    output_start_point: segment.start().clone(),
                    output_end_point: segment.end().clone(),
                    status: RetainedTopologyStatus::NativeExact,
                }
            })
            .collect::<Vec<_>>();
        let segments = retained
            .into_iter()
            .map(|(_, segment)| segment)
            .collect::<Vec<_>>();
        let curve_string = CurveString2::try_new(segments)?;
        let output_segment_kind_counts = curve_string_segment_kind_counts(&curve_string);
        Ok(CurveStringDeduplicateResult2 {
            report: CurveStringDeduplicateReport2 {
                stage: CurveStringDeduplicateStage2::SegmentMaterialization,
                predicate_path: CurveStringDeduplicatePredicatePath2::ExactReversedSegmentEquality,
                source_segment_count: self.len(),
                source_segment_kind_counts: curve_string_segment_kind_counts(self),
                output_segment_count: Some(curve_string.len()),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                retained_source_segment_indices,
                retained_segments,
                removed_pairs,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            curve_string: Some(curve_string),
        })
    }

    /// Trims this open curve string between two segment-local parameters.
    ///
    /// The result always carries a report. Materialization currently supports
    /// exact line subsegments and whole native segments. Partial arc ranges are
    /// retained as unsupported blockers because the existing arc split
    /// parameter is chord-projection evidence; without a point-bearing split
    /// marker or stronger arc parameter, constructing a new arc endpoint would
    /// cross the exactness boundary. This keeps future point/curve/region trim
    /// paths free to supply certified split points without letting this API
    /// guess.
    pub fn trim_between_parameters(
        &self,
        start: CurveStringTrimPoint2,
        end: CurveStringTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        self.trim_between_parameters_with_report(start, end, policy)
    }

    /// Trims this open curve string between two segment-local parameters and retains evidence.
    pub fn trim_between_parameters_with_report(
        &self,
        start: CurveStringTrimPoint2,
        end: CurveStringTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        validate_trim_point(self, &start, policy)?;
        validate_trim_point(self, &end, policy)?;
        match compare_trim_points(&start, &end, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => return Err(CurveError::InvalidCurveRange),
            None => {
                return Ok(blocked_trim_result(
                    self,
                    start,
                    end,
                    Vec::new(),
                    CurveStringTrimInputPath2::Parameters,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let mut segment_reports = Vec::new();
        let mut trimmed_segments = Vec::new();
        for source_segment_index in start.segment_index..=end.segment_index {
            let range_start = if source_segment_index == start.segment_index {
                start.param.clone()
            } else {
                Real::zero()
            };
            let range_end = if source_segment_index == end.segment_index {
                end.param.clone()
            } else {
                Real::one()
            };
            let source_range = ParamRange::new(range_start, range_end);
            let source_segment = &self.segments[source_segment_index];
            let segment_report = CurveStringTrimSegmentReport2 {
                source_segment_index,
                source_segment_kind: source_segment.structural_facts().kind,
                output_segment_index: None,
                output_segment_kind: None,
                output_segment_start_point: None,
                output_segment_end_point: None,
                source_segment_start_point: source_segment.start().clone(),
                source_segment_end_point: source_segment.end().clone(),
                source_range: source_range.clone(),
                range_start_point: None,
                range_end_point: None,
                status: RetainedTopologyStatus::NativeExact,
            };
            match trim_segment_by_range(
                &self.segments[source_segment_index],
                &source_range,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(segment) => {
                    let mut segment_report = segment_report;
                    segment_report.output_segment_index = Some(trimmed_segments.len());
                    segment_report.output_segment_kind = Some(segment.structural_facts().kind);
                    segment_report.output_segment_start_point = Some(segment.start().clone());
                    segment_report.output_segment_end_point = Some(segment.end().clone());
                    segment_reports.push(segment_report);
                    trimmed_segments.push(segment);
                }
                SegmentTrimMaterialization::SkippedEmpty => {}
                SegmentTrimMaterialization::Unsupported(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unsupported;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start,
                        end,
                        segment_reports,
                        CurveStringTrimInputPath2::Parameters,
                        RetainedTopologyStatus::Unsupported,
                        Some(reason),
                    ));
                }
                SegmentTrimMaterialization::Unresolved(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unresolved;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start,
                        end,
                        segment_reports,
                        CurveStringTrimInputPath2::Parameters,
                        RetainedTopologyStatus::Unresolved,
                        Some(reason),
                    ));
                }
            }
        }

        if trimmed_segments.is_empty() {
            return Err(CurveError::InvalidCurveRange);
        }
        let curve_string = CurveString2::try_new(trimmed_segments)?;
        let report = CurveStringTrimReport2 {
            input_path: CurveStringTrimInputPath2::Parameters,
            predicate_path: CurveStringTrimPredicatePath2::ExactParameterRange,
            start,
            end,
            source_segment_count: self.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(self),
            segment_reports,
            output_segment_count: Some(curve_string.len()),
            output_segment_kind_counts: Some(curve_string_segment_kind_counts(&curve_string)),
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(CurveStringTrimResult2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Trims this open curve string between two exact points on the path.
    ///
    /// Unlike [`CurveString2::trim_between_parameters`], this point-bearing
    /// path can materialize partial circular arcs because the endpoints are
    /// explicit exact geometry. The source arc's center, radius, and direction
    /// are retained and replayed against point-on-arc predicates before any
    /// fragment is emitted. Repeated non-adjacent point occurrences remain an
    /// explicit boundary blocker instead of choosing a path branch silently.
    pub fn trim_between_points(
        &self,
        start_point: &Point2,
        end_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        self.trim_between_points_with_report(start_point, end_point, policy)
    }

    /// Trims this open curve string between two exact path points and retains evidence.
    pub fn trim_between_points_with_report(
        &self,
        start_point: &Point2,
        end_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        let start = match locate_trim_point(self, start_point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(blocked_trim_result(
                    self,
                    CurveStringTrimPoint2::new(0, Real::zero()),
                    CurveStringTrimPoint2::new(0, Real::zero()),
                    Vec::new(),
                    CurveStringTrimInputPath2::Points,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };
        let end = match locate_trim_point(self, end_point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(blocked_trim_result(
                    self,
                    start.trim_point.clone(),
                    start.trim_point.clone(),
                    Vec::new(),
                    CurveStringTrimInputPath2::Points,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };

        match compare_trim_points(&start.trim_point, &end.trim_point, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => return Err(CurveError::InvalidCurveRange),
            None => {
                return Ok(blocked_trim_result(
                    self,
                    start.trim_point,
                    end.trim_point,
                    Vec::new(),
                    CurveStringTrimInputPath2::Points,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        self.trim_between_located_points(start, end, policy)
    }

    /// Trims this open curve string between exact point intersections with two cutters.
    ///
    /// Each cutter must contribute exactly one point witness on this curve
    /// string. No-hit, multiple-hit, overlap, and uncertain cutter relations are
    /// retained in the report as blockers. Successful materialization delegates
    /// to [`CurveString2::trim_between_points`], so line and arc fragments are
    /// emitted only through the same point-bearing exactness checks.
    pub fn trim_between_curve_intersections(
        &self,
        start_cutter: &Self,
        end_cutter: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringCurveTrimResult2> {
        self.trim_between_curve_intersections_with_report(start_cutter, end_cutter, policy)
    }

    /// Trims this open curve string between exact point intersections with two cutters.
    ///
    /// This report-bearing entry point records both cutter intersection
    /// reports, hit selection, blocker causes, and the final trim report.
    pub fn trim_between_curve_intersections_with_report(
        &self,
        start_cutter: &Self,
        end_cutter: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringCurveTrimResult2> {
        let start_events = self.intersect_curve_string_with_report(start_cutter, policy)?;
        let end_events = self.intersect_curve_string_with_report(end_cutter, policy)?;
        self.trim_between_curve_intersection_events(
            start_cutter,
            start_events,
            end_cutter,
            end_events,
            CurveStringCurveTrimQueryPath2::Direct,
            policy,
        )
    }

    /// Retains the portions of this open curve string inside a region.
    ///
    /// This is the first arrangement-style trim-by-region slice. Boundary
    /// intersections against all material and hole contours are collected with
    /// exact segment relations, source segments are split at retained
    /// parameters, and each retained interval is classified by an exact native
    /// representative. Point hits split intervals; overlaps and undecidable
    /// segment relations remain explicit blockers because they require a
    /// higher-order boundary traversal rather than a local interval decision.
    pub fn trim_inside_region(
        &self,
        region: &Region2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        self.trim_inside_region_with_report(region, policy)
    }

    /// Retains the portions of this open curve string inside a region and retains evidence.
    pub fn trim_inside_region_with_report(
        &self,
        region: &Region2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        trim_curve_string_inside_region(self, region, policy)
    }

    pub(crate) fn trim_inside_prepared_region(
        &self,
        region: &PreparedRegionView2<'_>,
        prepared_cache_report: Option<CurveStringRegionTrimPreparedCacheReport2>,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        trim_curve_string_inside_prepared_region(self, region, prepared_cache_report, policy)
    }

    /// Chamfers one interior line-line vertex by exact segment parameters.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`. The previous
    /// line is cut at `previous_param` and the next line is cut at
    /// `next_param`; the two cut points are connected by an exact line segment.
    /// This first chamfer slice supports only strict interior line parameters
    /// so it never deletes a neighboring segment silently.
    pub fn chamfer_line_line_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringChamferResult2> {
        self.chamfer_line_line_vertex_by_parameters_with_report(
            vertex_index,
            previous_param,
            next_param,
            policy,
        )
    }

    /// Chamfers one interior line-line vertex by exact segment parameters and retains evidence.
    pub fn chamfer_line_line_vertex_by_parameters_with_report(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringChamferResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_trim = CurveStringTrimPoint2::new(previous_segment_index, previous_param);
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Vec::new(),
                    CurveStringChamferPredicatePath2::UnsupportedSegmentFamily,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

        match (
            compare_reals(previous_trim.param(), &Real::zero(), policy),
            compare_reals(previous_trim.param(), &Real::one(), policy),
            compare_reals(next_trim.param(), &Real::zero(), policy),
            compare_reals(next_trim.param(), &Real::one(), policy),
        ) {
            (
                Some(Ordering::Greater),
                Some(Ordering::Less),
                Some(Ordering::Greater),
                Some(Ordering::Less),
            ) => {}
            (Some(_), Some(_), Some(_), Some(_)) => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Vec::new(),
                    CurveStringChamferPredicatePath2::CutParameterNotStrictInterior,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            _ => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Vec::new(),
                    CurveStringChamferPredicatePath2::CutValidationUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let previous_cut = previous_line.point_at(previous_trim.param().clone());
        let next_cut = next_line.point_at(next_trim.param().clone());
        let previous_cut_point = previous_cut.clone();
        let next_cut_point = next_cut.clone();
        let previous_range = ParamRange::new(Real::zero(), previous_trim.param().clone());
        let next_range = ParamRange::new(next_trim.param().clone(), Real::one());
        let previous_segment =
            LineSeg2::try_new(previous_line.start().clone(), previous_cut.clone())?;
        let chamfer_segment = LineSeg2::try_new(previous_cut, next_cut.clone())?;
        let next_segment = LineSeg2::try_new(next_cut, next_line.end().clone())?;

        let mut segments = Vec::with_capacity(self.len() + 1);
        segments.extend(self.segments[..previous_segment_index].iter().cloned());
        segments.push(Segment2::Line(previous_segment));
        let chamfer_segment_index = segments.len();
        segments.push(Segment2::Line(chamfer_segment));
        segments.push(Segment2::Line(next_segment));
        segments.extend(self.segments[next_segment_index + 1..].iter().cloned());
        let curve_string = CurveString2::try_new(segments)?;
        let output_segment_count = curve_string.len();
        let output_segment_kind_counts = curve_string_segment_kind_counts(&curve_string);
        let segment_reports = vec![
            CurveStringTrimSegmentReport2 {
                source_segment_index: previous_segment_index,
                source_segment_kind: SegmentKind::Line,
                output_segment_index: Some(previous_segment_index),
                output_segment_kind: Some(SegmentKind::Line),
                output_segment_start_point: Some(previous_line.start().clone()),
                output_segment_end_point: Some(previous_cut_point.clone()),
                source_segment_start_point: previous_line.start().clone(),
                source_segment_end_point: previous_line.end().clone(),
                source_range: previous_range,
                range_start_point: Some(previous_line.start().clone()),
                range_end_point: Some(previous_cut_point.clone()),
                status: RetainedTopologyStatus::NativeExact,
            },
            CurveStringTrimSegmentReport2 {
                source_segment_index: next_segment_index,
                source_segment_kind: SegmentKind::Line,
                output_segment_index: Some(next_segment_index + 1),
                output_segment_kind: Some(SegmentKind::Line),
                output_segment_start_point: Some(next_cut_point.clone()),
                output_segment_end_point: Some(next_line.end().clone()),
                source_segment_start_point: next_line.start().clone(),
                source_segment_end_point: next_line.end().clone(),
                source_range: next_range,
                range_start_point: Some(next_cut_point.clone()),
                range_end_point: Some(next_line.end().clone()),
                status: RetainedTopologyStatus::NativeExact,
            },
        ];
        Ok(CurveStringChamferResult2 {
            curve_string: Some(curve_string),
            report: CurveStringChamferReport2 {
                input_path: CurveStringChamferInputPath2::Parameters,
                stage: CurveStringChamferStage2::SegmentMaterialization,
                predicate_path: CurveStringChamferPredicatePath2::LineLineStrictInteriorParameters,
                previous_segment_index,
                next_segment_index,
                previous_segment_start_point: previous_line.start().clone(),
                previous_segment_end_point: previous_line.end().clone(),
                next_segment_start_point: next_line.start().clone(),
                next_segment_end_point: next_line.end().clone(),
                previous_trim,
                next_trim,
                previous_cut_point: Some(previous_cut_point.clone()),
                next_cut_point: Some(next_cut_point.clone()),
                segment_reports,
                chamfer_segment_index: Some(chamfer_segment_index),
                chamfer_segment_kind: Some(SegmentKind::Line),
                chamfer_segment_start_point: Some(previous_cut_point.clone()),
                chamfer_segment_end_point: Some(next_cut_point.clone()),
                source_segment_count: self.len(),
                source_segment_kind_counts: curve_string_segment_kind_counts(self),
                output_segment_count: Some(output_segment_count),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Chamfers one interior line-line vertex by exact cut points.
    ///
    /// The supplied points are validated against the specific previous and next
    /// finite line segments adjacent to `vertex_index`. Certified point
    /// parameters are then passed to
    /// [`CurveString2::chamfer_line_line_vertex_by_parameters`], so the same
    /// strict interior-parameter and retained-range rules decide whether native
    /// topology may be materialized.
    pub fn chamfer_line_line_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringChamferResult2> {
        self.chamfer_line_line_vertex_by_points_with_report(
            vertex_index,
            previous_point,
            next_point,
            policy,
        )
    }

    /// Chamfers one interior line-line vertex by exact cut points and retains evidence.
    pub fn chamfer_line_line_vertex_by_points_with_report(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringChamferResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_zero = CurveStringTrimPoint2::new(previous_segment_index, Real::zero());
        let next_zero = CurveStringTrimPoint2::new(next_segment_index, Real::zero());
        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_zero,
                    next_zero,
                    Vec::new(),
                    CurveStringChamferPredicatePath2::UnsupportedSegmentFamily,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

        let previous_param =
            match line_chamfer_point_parameter(previous_line, previous_point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => {
                    let mut result = blocked_chamfer_result(
                        self,
                        previous_segment_index,
                        next_segment_index,
                        previous_zero,
                        next_zero,
                        Vec::new(),
                        if reason == UncertaintyReason::Boundary {
                            CurveStringChamferPredicatePath2::PointCutNotOnSourceSegment
                        } else {
                            CurveStringChamferPredicatePath2::CutValidationUnresolved
                        },
                        retained_status_for_uncertainty(reason),
                        Some(reason),
                    );
                    result.report_mut().input_path = CurveStringChamferInputPath2::Points;
                    return Ok(result);
                }
            };
        let previous_trim =
            CurveStringTrimPoint2::new(previous_segment_index, previous_param.clone());
        let next_param = match line_chamfer_point_parameter(next_line, next_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => {
                let mut result = blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_zero,
                    Vec::new(),
                    if reason == UncertaintyReason::Boundary {
                        CurveStringChamferPredicatePath2::PointCutNotOnSourceSegment
                    } else {
                        CurveStringChamferPredicatePath2::CutValidationUnresolved
                    },
                    retained_status_for_uncertainty(reason),
                    Some(reason),
                );
                result.report_mut().input_path = CurveStringChamferInputPath2::Points;
                return Ok(result);
            }
        };

        let mut result = self.chamfer_line_line_vertex_by_parameters_with_report(
            vertex_index,
            previous_param,
            next_param,
            policy,
        )?;
        result.report_mut().input_path = CurveStringChamferInputPath2::Points;
        Ok(result)
    }

    /// Fillets one interior line-line vertex from exact parameters and center.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`. The
    /// parameters identify the tangent points on the previous and next line
    /// segments. The final materialization delegates to
    /// [`CurveString2::fillet_line_line_vertex_by_points`], so the same
    /// nonzero-radius, equal-radius, tangency, orientation, and retained-range
    /// checks decide whether native topology may be emitted.
    pub fn fillet_line_line_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringFilletResult2> {
        self.fillet_line_line_vertex_by_parameters_with_report(
            vertex_index,
            previous_param,
            next_param,
            center,
            clockwise,
            policy,
        )
    }

    /// Fillets one interior line-line vertex from exact parameters and center, retaining evidence.
    pub fn fillet_line_line_vertex_by_parameters_with_report(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringFilletResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_trim = CurveStringTrimPoint2::new(previous_segment_index, previous_param);
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                let mut result = blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    CurveStringFilletPredicatePath2::UnsupportedSegmentFamily,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                );
                result.report_mut().input_path = CurveStringFilletInputPath2::Parameters;
                return Ok(result);
            }
        };

        let previous_point = previous_line.point_at(previous_trim.param().clone());
        let next_point = next_line.point_at(next_trim.param().clone());
        let mut result = self.fillet_line_line_vertex_by_points_with_report(
            vertex_index,
            &previous_point,
            &next_point,
            center,
            clockwise,
            policy,
        )?;
        result.report_mut().input_path = CurveStringFilletInputPath2::Parameters;
        Ok(result)
    }

    /// Fillets one interior line-line vertex from exact tangent points and center.
    ///
    /// The tangent points must lie at strict interior parameters of the two
    /// adjacent finite line segments. The center must be equidistant from both
    /// tangent points, nonzero-radius, perpendicular to both line directions,
    /// and oriented so the inserted arc is tangent to the incoming and outgoing
    /// line traversal. Failed predicates are retained as explicit blockers.
    pub fn fillet_line_line_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringFilletResult2> {
        self.fillet_line_line_vertex_by_points_with_report(
            vertex_index,
            previous_point,
            next_point,
            center,
            clockwise,
            policy,
        )
    }

    /// Fillets one interior line-line vertex by exact tangent points and center, retaining evidence.
    pub fn fillet_line_line_vertex_by_points_with_report(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringFilletResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_zero = CurveStringTrimPoint2::new(previous_segment_index, Real::zero());
        let next_zero = CurveStringTrimPoint2::new(next_segment_index, Real::zero());
        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_zero,
                    next_zero,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    CurveStringFilletPredicatePath2::UnsupportedSegmentFamily,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

        let previous_param =
            match line_chamfer_point_parameter(previous_line, previous_point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => {
                    return Ok(blocked_fillet_result(
                        self,
                        previous_segment_index,
                        next_segment_index,
                        previous_zero,
                        next_zero,
                        Some(center.clone()),
                        None,
                        Vec::new(),
                        if reason == UncertaintyReason::Boundary {
                            CurveStringFilletPredicatePath2::TangentPointNotOnSourceSegment
                        } else {
                            CurveStringFilletPredicatePath2::TangentValidationUnresolved
                        },
                        retained_status_for_uncertainty(reason),
                        Some(reason),
                    ));
                }
            };
        let previous_trim =
            CurveStringTrimPoint2::new(previous_segment_index, previous_param.clone());
        let next_param = match line_chamfer_point_parameter(next_line, next_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_zero,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    if reason == UncertaintyReason::Boundary {
                        CurveStringFilletPredicatePath2::TangentPointNotOnSourceSegment
                    } else {
                        CurveStringFilletPredicatePath2::TangentValidationUnresolved
                    },
                    retained_status_for_uncertainty(reason),
                    Some(reason),
                ));
            }
        };
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        match (
            compare_reals(previous_trim.param(), &Real::zero(), policy),
            compare_reals(previous_trim.param(), &Real::one(), policy),
            compare_reals(next_trim.param(), &Real::zero(), policy),
            compare_reals(next_trim.param(), &Real::one(), policy),
        ) {
            (
                Some(Ordering::Greater),
                Some(Ordering::Less),
                Some(Ordering::Greater),
                Some(Ordering::Less),
            ) => {}
            (Some(_), Some(_), Some(_), Some(_)) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    CurveStringFilletPredicatePath2::TangentParameterNotStrictInterior,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            _ => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    CurveStringFilletPredicatePath2::TangentValidationUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let radius_squared = previous_point.distance_squared(center);
        match is_zero(&radius_squared, policy) {
            Some(false) => {}
            Some(true) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    CurveStringFilletPredicatePath2::ZeroRadius,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            None => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    CurveStringFilletPredicatePath2::RadiusValidationUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::RealSign),
                ));
            }
        }

        let next_radius_squared = next_point.distance_squared(center);
        let radius_delta = &radius_squared - &next_radius_squared;
        match is_zero(&radius_delta, policy) {
            Some(true) => {}
            Some(false) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    CurveStringFilletPredicatePath2::RadiusMismatch,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            None => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    CurveStringFilletPredicatePath2::RadiusValidationUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::RealSign),
                ));
            }
        }

        if let Some(reason) = line_line_fillet_validation_blocker(
            previous_line,
            previous_point,
            center,
            clockwise,
            policy,
        ) {
            return Ok(blocked_fillet_result(
                self,
                previous_segment_index,
                next_segment_index,
                previous_trim,
                next_trim,
                Some(center.clone()),
                Some(radius_squared),
                Vec::new(),
                if reason == UncertaintyReason::Boundary {
                    CurveStringFilletPredicatePath2::TangencyOrOrientationRejected
                } else {
                    CurveStringFilletPredicatePath2::TangencyOrOrientationUnresolved
                },
                retained_status_for_uncertainty(reason),
                Some(reason),
            ));
        }
        if let Some(reason) =
            line_line_fillet_validation_blocker(next_line, next_point, center, clockwise, policy)
        {
            return Ok(blocked_fillet_result(
                self,
                previous_segment_index,
                next_segment_index,
                previous_trim,
                next_trim,
                Some(center.clone()),
                Some(radius_squared),
                Vec::new(),
                if reason == UncertaintyReason::Boundary {
                    CurveStringFilletPredicatePath2::TangencyOrOrientationRejected
                } else {
                    CurveStringFilletPredicatePath2::TangencyOrOrientationUnresolved
                },
                retained_status_for_uncertainty(reason),
                Some(reason),
            ));
        }

        let previous_range = ParamRange::new(Real::zero(), previous_trim.param().clone());
        let next_range = ParamRange::new(next_trim.param().clone(), Real::one());
        let previous_segment =
            LineSeg2::try_new(previous_line.start().clone(), previous_point.clone())?;
        let fillet_segment = CircularArc2::try_from_center(
            previous_point.clone(),
            next_point.clone(),
            center.clone(),
            clockwise,
        )?;
        let next_segment = LineSeg2::try_new(next_point.clone(), next_line.end().clone())?;

        let mut segments = Vec::with_capacity(self.len() + 1);
        segments.extend(self.segments[..previous_segment_index].iter().cloned());
        segments.push(Segment2::Line(previous_segment));
        let fillet_segment_index = segments.len();
        segments.push(Segment2::Arc(fillet_segment));
        segments.push(Segment2::Line(next_segment));
        segments.extend(self.segments[next_segment_index + 1..].iter().cloned());
        let curve_string = CurveString2::try_new(segments)?;
        let output_segment_count = curve_string.len();
        let output_segment_kind_counts = curve_string_segment_kind_counts(&curve_string);
        let segment_reports = vec![
            CurveStringTrimSegmentReport2 {
                source_segment_index: previous_segment_index,
                source_segment_kind: SegmentKind::Line,
                output_segment_index: Some(previous_segment_index),
                output_segment_kind: Some(SegmentKind::Line),
                output_segment_start_point: Some(previous_line.start().clone()),
                output_segment_end_point: Some((*previous_point).clone()),
                source_segment_start_point: previous_line.start().clone(),
                source_segment_end_point: previous_line.end().clone(),
                source_range: previous_range,
                range_start_point: Some(previous_line.start().clone()),
                range_end_point: Some((*previous_point).clone()),
                status: RetainedTopologyStatus::NativeExact,
            },
            CurveStringTrimSegmentReport2 {
                source_segment_index: next_segment_index,
                source_segment_kind: SegmentKind::Line,
                output_segment_index: Some(next_segment_index + 1),
                output_segment_kind: Some(SegmentKind::Line),
                output_segment_start_point: Some((*next_point).clone()),
                output_segment_end_point: Some(next_line.end().clone()),
                source_segment_start_point: next_line.start().clone(),
                source_segment_end_point: next_line.end().clone(),
                source_range: next_range,
                range_start_point: Some((*next_point).clone()),
                range_end_point: Some(next_line.end().clone()),
                status: RetainedTopologyStatus::NativeExact,
            },
        ];
        Ok(CurveStringFilletResult2 {
            curve_string: Some(curve_string),
            report: CurveStringFilletReport2 {
                input_path: CurveStringFilletInputPath2::Points,
                stage: CurveStringFilletStage2::ArcMaterialization,
                predicate_path: CurveStringFilletPredicatePath2::LineLineTangentArc,
                previous_segment_index,
                next_segment_index,
                previous_segment_start_point: previous_line.start().clone(),
                previous_segment_end_point: previous_line.end().clone(),
                next_segment_start_point: next_line.start().clone(),
                next_segment_end_point: next_line.end().clone(),
                previous_trim,
                next_trim,
                previous_tangent_point: Some((*previous_point).clone()),
                next_tangent_point: Some((*next_point).clone()),
                center: Some(center.clone()),
                radius_squared: Some(radius_squared),
                segment_reports,
                fillet_segment_index: Some(fillet_segment_index),
                fillet_segment_kind: Some(SegmentKind::Arc),
                fillet_segment_start_point: Some((*previous_point).clone()),
                fillet_segment_end_point: Some((*next_point).clone()),
                source_segment_count: self.len(),
                source_segment_kind_counts: curve_string_segment_kind_counts(self),
                output_segment_count: Some(output_segment_count),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Extends one endpoint line segment to an exact point on its supporting line.
    ///
    /// This first extension slice deliberately supports only line endpoint
    /// segments. The target must be certified on the selected line support and
    /// outside the finite segment in the selected endpoint direction. Targets
    /// inside the existing segment, off the support, on arcs, or behind
    /// undecided predicates are retained as explicit blockers.
    pub fn extend_line_endpoint_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        self.extend_line_endpoint_to_point_with_report(endpoint, target_point, policy)
    }

    /// Extends one endpoint line segment to an exact point and retains evidence.
    pub fn extend_line_endpoint_to_point_with_report(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        self.extend_endpoint_to_point_with_report(endpoint, target_point, policy)
    }

    /// Extends one endpoint segment to an exact target point.
    ///
    /// Lines extend along their supporting line. Circular arcs extend only when
    /// the target is certified on the same circle and outside the current
    /// finite arc. The resulting same-orientation native arc must replay both
    /// the old endpoint and an interior witness from the source arc, so the
    /// operation extends the existing topology instead of replacing it with an
    /// unrelated chord or sampled sweep.
    pub fn extend_endpoint_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        self.extend_endpoint_to_point_with_report(endpoint, target_point, policy)
    }

    /// Extends one endpoint segment to an exact target point and retains evidence.
    pub fn extend_endpoint_to_point_with_report(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        let source_segment_index = match endpoint {
            CurveStringEndpoint2::Start => 0,
            CurveStringEndpoint2::End => self
                .len()
                .checked_sub(1)
                .ok_or(CurveError::EmptyCurveString)?,
        };
        let segment = self
            .segments
            .get(source_segment_index)
            .ok_or(CurveError::EmptyCurveString)?;
        match segment {
            Segment2::Line(line) => self.extend_line_endpoint_segment_to_point(
                endpoint,
                source_segment_index,
                line,
                target_point,
                policy,
            ),
            Segment2::Arc(arc) => self.extend_arc_endpoint_segment_to_point(
                endpoint,
                source_segment_index,
                arc,
                target_point,
                policy,
            ),
        }
    }

    fn extend_line_endpoint_segment_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        source_segment_index: usize,
        line: &LineSeg2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        match line.classify_point(&target_point, policy) {
            Classification::Decided(crate::LineSide::On) => {}
            Classification::Decided(_) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::LineTargetOffSupport,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::LineTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let source_param = match line_point_parameter(line, &target_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::LineTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };
        let outside = match endpoint {
            CurveStringEndpoint2::Start => compare_reals(&source_param, &Real::zero(), policy),
            CurveStringEndpoint2::End => compare_reals(&source_param, &Real::one(), policy),
        };
        match (endpoint, outside) {
            (CurveStringEndpoint2::Start, Some(Ordering::Less))
            | (CurveStringEndpoint2::End, Some(Ordering::Greater)) => {}
            (_, Some(_)) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    Some(source_param),
                    CurveStringExtendPredicatePath2::LineTargetNotOutsideEndpoint,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            (_, None) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    Some(source_param),
                    CurveStringExtendPredicatePath2::LineTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let mut segments = self.segments.clone();
        let output_segment = match endpoint {
            CurveStringEndpoint2::Start => {
                LineSeg2::try_new(target_point.clone(), line.end().clone())?
            }
            CurveStringEndpoint2::End => {
                LineSeg2::try_new(line.start().clone(), target_point.clone())?
            }
        };
        let output_segment_start_point = output_segment.start().clone();
        let output_segment_end_point = output_segment.end().clone();
        segments[source_segment_index] = Segment2::Line(output_segment);
        let curve_string = CurveString2::try_new(segments)?;
        let output_segment_count = curve_string.len();
        let output_segment_kind_counts = curve_string_segment_kind_counts(&curve_string);
        Ok(CurveStringExtendResult2 {
            curve_string: Some(curve_string),
            report: CurveStringExtendReport2 {
                stage: CurveStringExtendStage2::SegmentMaterialization,
                predicate_path: CurveStringExtendPredicatePath2::LineSupportOutsideEndpoint,
                endpoint,
                source_segment_index,
                source_segment_kind: SegmentKind::Line,
                output_segment_index: Some(source_segment_index),
                output_segment_kind: Some(SegmentKind::Line),
                output_segment_start_point: Some(output_segment_start_point),
                output_segment_end_point: Some(output_segment_end_point),
                source_segment_start_point: line.start().clone(),
                source_segment_end_point: line.end().clone(),
                source_endpoint_point: line_endpoint_point(line, endpoint).clone(),
                target_point,
                source_param: Some(source_param),
                source_segment_count: self.len(),
                source_segment_kind_counts: curve_string_segment_kind_counts(self),
                output_segment_count: Some(output_segment_count),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    fn extend_arc_endpoint_segment_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        source_segment_index: usize,
        arc: &CircularArc2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        let radius_delta = target_point.distance_squared(arc.center()) - arc.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(true) => {}
            Some(false) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetOffCircle,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            None => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::RealSign),
                ));
            }
        }

        match arc.contains_point(&target_point, policy) {
            Classification::Decided(false) => {}
            Classification::Decided(true) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetAlreadyOnSweep,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let extended_arc = match endpoint {
            CurveStringEndpoint2::Start => CircularArc2::new_unchecked_with_radius(
                target_point.clone(),
                arc.end().clone(),
                arc.center().clone(),
                arc.radius_squared(),
                arc.is_clockwise(),
                None,
            ),
            CurveStringEndpoint2::End => CircularArc2::new_unchecked_with_radius(
                arc.start().clone(),
                target_point.clone(),
                arc.center().clone(),
                arc.radius_squared(),
                arc.is_clockwise(),
                None,
            ),
        };

        let retained_endpoint = match endpoint {
            CurveStringEndpoint2::Start => arc.start(),
            CurveStringEndpoint2::End => arc.end(),
        };
        match extended_arc.contains_point(retained_endpoint, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcSweepReplayRejected,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let representative = match arc.representative_point(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };
        match extended_arc.contains_point(&representative, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcSweepReplayRejected,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    CurveStringExtendPredicatePath2::ArcTargetUnresolved,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let output_segment_start_point = extended_arc.start().clone();
        let output_segment_end_point = extended_arc.end().clone();
        let mut segments = self.segments.clone();
        segments[source_segment_index] = Segment2::Arc(extended_arc);
        let curve_string = CurveString2::try_new(segments)?;
        let output_segment_count = curve_string.len();
        let output_segment_kind_counts = curve_string_segment_kind_counts(&curve_string);
        Ok(CurveStringExtendResult2 {
            curve_string: Some(curve_string),
            report: CurveStringExtendReport2 {
                stage: CurveStringExtendStage2::SegmentMaterialization,
                predicate_path: CurveStringExtendPredicatePath2::ArcSameCircleSweepReplay,
                endpoint,
                source_segment_index,
                source_segment_kind: SegmentKind::Arc,
                output_segment_index: Some(source_segment_index),
                output_segment_kind: Some(SegmentKind::Arc),
                output_segment_start_point: Some(output_segment_start_point),
                output_segment_end_point: Some(output_segment_end_point),
                source_segment_start_point: arc.start().clone(),
                source_segment_end_point: arc.end().clone(),
                source_endpoint_point: arc_endpoint_point(arc, endpoint).clone(),
                target_point,
                source_param: None,
                source_segment_count: self.len(),
                source_segment_kind_counts: curve_string_segment_kind_counts(self),
                output_segment_count: Some(output_segment_count),
                output_segment_kind_counts: Some(output_segment_kind_counts),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    pub(crate) fn trim_between_curve_intersection_events(
        &self,
        start_cutter: &Self,
        start_events: CurveStringIntersectionResult2,
        end_cutter: &Self,
        end_events: CurveStringIntersectionResult2,
        query_path: CurveStringCurveTrimQueryPath2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringCurveTrimResult2> {
        let start_intersection_report = start_events.report().clone();
        let end_intersection_report = end_events.report().clone();
        let start_extraction =
            extract_curve_trim_hits(self, start_cutter, start_events.intersections(), policy)?;
        let end_extraction =
            extract_curve_trim_hits(self, end_cutter, end_events.intersections(), policy)?;

        let start_hit = match single_curve_trim_hit(&start_extraction) {
            Ok(hit) => hit,
            Err((status, blocker)) => {
                return Ok(blocked_curve_trim_result(
                    start_extraction.hits,
                    end_extraction.hits,
                    start_intersection_report,
                    end_intersection_report,
                    None,
                    query_path,
                    status,
                    Some(blocker),
                ));
            }
        };
        let end_hit = match single_curve_trim_hit(&end_extraction) {
            Ok(hit) => hit,
            Err((status, blocker)) => {
                return Ok(blocked_curve_trim_result(
                    start_extraction.hits,
                    end_extraction.hits,
                    start_intersection_report,
                    end_intersection_report,
                    None,
                    query_path,
                    status,
                    Some(blocker),
                ));
            }
        };

        let trim = self.trim_between_points(&start_hit.point, &end_hit.point, policy)?;
        let status = trim.report().status();
        let blocker = trim.report().blocker();
        let curve_string = trim.curve_string().cloned();
        Ok(CurveStringCurveTrimResult2 {
            curve_string,
            report: CurveStringCurveTrimReport2 {
                start_hits: vec![start_hit],
                end_hits: vec![end_hit],
                start_intersection_report,
                end_intersection_report,
                trim_report: Some(trim.report().clone()),
                query_path,
                stage: CurveStringCurveTrimStage2::RangeMaterialization,
                status,
                blocker,
            },
        })
    }

    fn trim_between_located_points(
        &self,
        start: LocatedTrimPoint2,
        end: LocatedTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        let mut segment_reports = Vec::new();
        let mut trimmed_segments = Vec::new();
        for source_segment_index in start.trim_point.segment_index..=end.trim_point.segment_index {
            let range_start = if source_segment_index == start.trim_point.segment_index {
                start.trim_point.param.clone()
            } else {
                Real::zero()
            };
            let range_end = if source_segment_index == end.trim_point.segment_index {
                end.trim_point.param.clone()
            } else {
                Real::one()
            };
            let range_start_point = if source_segment_index == start.trim_point.segment_index {
                start.point.clone()
            } else {
                self.segments[source_segment_index].start().clone()
            };
            let range_end_point = if source_segment_index == end.trim_point.segment_index {
                end.point.clone()
            } else {
                self.segments[source_segment_index].end().clone()
            };
            let source_range = ParamRange::new(range_start, range_end);
            let source_segment = &self.segments[source_segment_index];
            let segment_report = CurveStringTrimSegmentReport2 {
                source_segment_index,
                source_segment_kind: source_segment.structural_facts().kind,
                output_segment_index: None,
                output_segment_kind: None,
                output_segment_start_point: None,
                output_segment_end_point: None,
                source_segment_start_point: source_segment.start().clone(),
                source_segment_end_point: source_segment.end().clone(),
                source_range: source_range.clone(),
                range_start_point: Some(range_start_point.clone()),
                range_end_point: Some(range_end_point.clone()),
                status: RetainedTopologyStatus::NativeExact,
            };
            match trim_segment_by_point_range(
                &self.segments[source_segment_index],
                &source_range,
                &range_start_point,
                &range_end_point,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(segment) => {
                    let mut segment_report = segment_report;
                    segment_report.output_segment_index = Some(trimmed_segments.len());
                    segment_report.output_segment_kind = Some(segment.structural_facts().kind);
                    segment_report.output_segment_start_point = Some(segment.start().clone());
                    segment_report.output_segment_end_point = Some(segment.end().clone());
                    segment_reports.push(segment_report);
                    trimmed_segments.push(segment);
                }
                SegmentTrimMaterialization::SkippedEmpty => {}
                SegmentTrimMaterialization::Unsupported(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unsupported;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start.trim_point,
                        end.trim_point,
                        segment_reports,
                        CurveStringTrimInputPath2::Points,
                        RetainedTopologyStatus::Unsupported,
                        Some(reason),
                    ));
                }
                SegmentTrimMaterialization::Unresolved(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unresolved;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start.trim_point,
                        end.trim_point,
                        segment_reports,
                        CurveStringTrimInputPath2::Points,
                        RetainedTopologyStatus::Unresolved,
                        Some(reason),
                    ));
                }
            }
        }

        if trimmed_segments.is_empty() {
            return Err(CurveError::InvalidCurveRange);
        }
        let curve_string = CurveString2::try_new(trimmed_segments)?;
        let report = CurveStringTrimReport2 {
            input_path: CurveStringTrimInputPath2::Points,
            predicate_path: CurveStringTrimPredicatePath2::ExactLocatedPointRange,
            start: start.trim_point,
            end: end.trim_point,
            source_segment_count: self.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(self),
            segment_reports,
            output_segment_count: Some(curve_string.len()),
            output_segment_kind_counts: Some(curve_string_segment_kind_counts(&curve_string)),
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(CurveStringTrimResult2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Collects all nonempty segment-pair intersections against another curve string.
    ///
    /// Segment axis-aligned bounding boxes are used as a conservative broad
    /// phase before exact segment intersection. A decided box non-overlap skips
    /// the pair; any box uncertainty falls back to exact topology. This keeps
    /// the exact segment relation authoritative while following the
    /// candidate-pruning role used by sweep-line intersection methods such as
    /// Bentley and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections" (1979).
    pub fn intersect_curve_string(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        self.intersect_curve_string_with_report(other, policy)
            .map(|result| result.into_intersections())
    }

    /// Collects all nonempty segment-pair intersections with scan evidence.
    ///
    /// The returned report records how many segment pairs were considered,
    /// skipped by decided AABB disjointness, and tested with exact segment
    /// topology. The broad phase remains advisory: every non-skipped pair is
    /// resolved by the same exact segment intersection dispatch as
    /// [`CurveString2::intersect_curve_string`].
    pub fn intersect_curve_string_with_report(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringIntersectionResult2> {
        let self_boxes: Vec<_> = self
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        let other_boxes: Vec<_> = other
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();

        intersect_curve_strings_with_cached_aabbs_with_report(
            self,
            other,
            &self_boxes,
            &other_boxes,
            CurveStringIntersectionQueryPath2::Direct,
            policy,
        )
    }

    fn endpoint(&self, endpoint: CurveStringEndpoint2) -> CurveResult<&Point2> {
        match endpoint {
            CurveStringEndpoint2::Start => self.start().ok_or(CurveError::EmptyCurveString),
            CurveStringEndpoint2::End => self.end().ok_or(CurveError::EmptyCurveString),
        }
    }

    fn endpoint_link_reports(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<[(CurveStringLinkKind2, CurveStringEndpointConnectionReport2); 4]> {
        Ok([
            (
                CurveStringLinkKind2::FirstEndToSecondStart,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::End,
                    CurveStringEndpoint2::Start,
                    policy,
                )?,
            ),
            (
                CurveStringLinkKind2::FirstEndToSecondEnd,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::End,
                    CurveStringEndpoint2::End,
                    policy,
                )?,
            ),
            (
                CurveStringLinkKind2::FirstStartToSecondStart,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::Start,
                    CurveStringEndpoint2::Start,
                    policy,
                )?,
            ),
            (
                CurveStringLinkKind2::FirstStartToSecondEnd,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::Start,
                    CurveStringEndpoint2::End,
                    policy,
                )?,
            ),
        ])
    }

    fn endpoint_connection_report_for_kind(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringEndpointConnectionReport2> {
        match kind {
            CurveStringLinkKind2::FirstEndToSecondStart => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::Start,
                policy,
            ),
            CurveStringLinkKind2::FirstEndToSecondEnd => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::End,
                policy,
            ),
            CurveStringLinkKind2::FirstStartToSecondStart => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::Start,
                CurveStringEndpoint2::Start,
                policy,
            ),
            CurveStringLinkKind2::FirstStartToSecondEnd => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::Start,
                CurveStringEndpoint2::End,
                policy,
            ),
        }
    }
}

impl CurveStringTrimPoint2 {
    /// Constructs a segment-local trim point.
    pub const fn new(segment_index: usize, param: Real) -> Self {
        Self {
            segment_index,
            param,
        }
    }

    /// Returns the source segment index.
    pub const fn segment_index(&self) -> usize {
        self.segment_index
    }

    /// Returns the local segment parameter.
    pub const fn param(&self) -> &Real {
        &self.param
    }
}

impl CurveStringTrimSegmentReport2 {
    /// Returns the retained source segment index.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the retained source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the emitted output segment index, when this range materialized.
    pub const fn output_segment_index(&self) -> Option<usize> {
        self.output_segment_index
    }

    /// Returns the primitive family of the emitted segment, when materialized.
    pub const fn output_segment_kind(&self) -> Option<SegmentKind> {
        self.output_segment_kind
    }

    /// Returns the exact start point of the emitted segment, when materialized.
    pub const fn output_segment_start_point(&self) -> Option<&Point2> {
        self.output_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the emitted segment, when materialized.
    pub const fn output_segment_end_point(&self) -> Option<&Point2> {
        self.output_segment_end_point.as_ref()
    }

    /// Returns the exact start point of the retained source segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the retained source segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the retained source parameter range.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the exact start point witness for this range, when certified.
    pub const fn range_start_point(&self) -> Option<&Point2> {
        self.range_start_point.as_ref()
    }

    /// Returns the exact end point witness for this range, when certified.
    pub const fn range_end_point(&self) -> Option<&Point2> {
        self.range_end_point.as_ref()
    }

    /// Returns topology status for this retained source range.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringTrimReport2 {
    /// Returns how trim boundary evidence was supplied.
    pub const fn input_path(&self) -> CurveStringTrimInputPath2 {
        self.input_path
    }

    /// Returns the exact predicate path used while retaining trim ranges.
    pub const fn predicate_path(&self) -> CurveStringTrimPredicatePath2 {
        self.predicate_path
    }

    /// Returns the requested trim start.
    pub const fn start(&self) -> &CurveStringTrimPoint2 {
        &self.start
    }

    /// Returns the requested trim end.
    pub const fn end(&self) -> &CurveStringTrimPoint2 {
        &self.end
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns retained source segment ranges considered by this trim.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        &self.segment_reports
    }

    /// Returns output segment count when trim materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized trim output.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns the trim materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized trims.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringTrimResult2 {
    /// Returns the materialized native trim, if this trim was supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized native trim, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Consumes this result and returns retained trim evidence.
    pub fn into_report(self) -> CurveStringTrimReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized trim with its report.
    pub fn into_parts(self) -> (Option<CurveString2>, CurveStringTrimReport2) {
        (self.curve_string, self.report)
    }

    /// Returns the retained trim report.
    pub const fn report(&self) -> &CurveStringTrimReport2 {
        &self.report
    }

    /// Returns the trim output as a convenience classification while retaining this result.
    pub fn curve_string_classification(&self) -> Classification<&CurveString2> {
        match self.curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the trim output as a convenience classification.
    pub fn into_curve_string_classification(self) -> Classification<CurveString2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(blocker),
        }
    }
}

impl CurveStringChamferReport2 {
    /// Returns how the cut-point evidence was supplied to the chamfer.
    pub const fn input_path(&self) -> CurveStringChamferInputPath2 {
        self.input_path
    }

    /// Returns the furthest exact chamfer stage reached.
    pub const fn stage(&self) -> CurveStringChamferStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to validate this chamfer.
    pub const fn predicate_path(&self) -> CurveStringChamferPredicatePath2 {
        self.predicate_path
    }

    /// Returns the previous source segment index at the chamfered vertex.
    pub const fn previous_segment_index(&self) -> usize {
        self.previous_segment_index
    }

    /// Returns the next source segment index at the chamfered vertex.
    pub const fn next_segment_index(&self) -> usize {
        self.next_segment_index
    }

    /// Returns the exact start point of the previous source segment.
    pub const fn previous_segment_start_point(&self) -> &Point2 {
        &self.previous_segment_start_point
    }

    /// Returns the exact end point of the previous source segment.
    pub const fn previous_segment_end_point(&self) -> &Point2 {
        &self.previous_segment_end_point
    }

    /// Returns the exact start point of the next source segment.
    pub const fn next_segment_start_point(&self) -> &Point2 {
        &self.next_segment_start_point
    }

    /// Returns the exact end point of the next source segment.
    pub const fn next_segment_end_point(&self) -> &Point2 {
        &self.next_segment_end_point
    }

    /// Returns the previous line trim point.
    pub const fn previous_trim(&self) -> &CurveStringTrimPoint2 {
        &self.previous_trim
    }

    /// Returns the next line trim point.
    pub const fn next_trim(&self) -> &CurveStringTrimPoint2 {
        &self.next_trim
    }

    /// Returns the exact previous-line cut point when the chamfer materialized.
    pub const fn previous_cut_point(&self) -> Option<&Point2> {
        self.previous_cut_point.as_ref()
    }

    /// Returns the exact next-line cut point when the chamfer materialized.
    pub const fn next_cut_point(&self) -> Option<&Point2> {
        self.next_cut_point.as_ref()
    }

    /// Returns retained source ranges for the shortened adjacent line segments.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        &self.segment_reports
    }

    /// Returns retained adjacent-source trim range count.
    pub const fn trim_segment_report_count(&self) -> usize {
        self.segment_reports.len()
    }

    /// Returns the inserted chamfer segment index in the output curve string.
    pub const fn chamfer_segment_index(&self) -> Option<usize> {
        self.chamfer_segment_index
    }

    /// Returns the primitive family of the inserted chamfer segment, when materialized.
    pub const fn chamfer_segment_kind(&self) -> Option<SegmentKind> {
        self.chamfer_segment_kind
    }

    /// Returns the exact start point of the inserted chamfer segment.
    pub const fn chamfer_segment_start_point(&self) -> Option<&Point2> {
        self.chamfer_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the inserted chamfer segment.
    pub const fn chamfer_segment_end_point(&self) -> Option<&Point2> {
        self.chamfer_segment_end_point.as_ref()
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns output segment count when the chamfer materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized chamfer output.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns chamfer materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized chamfers.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }

    pub(crate) fn remap_source_segment_indices(&mut self, mut remap: impl FnMut(usize) -> usize) {
        self.previous_segment_index = remap(self.previous_segment_index);
        self.next_segment_index = remap(self.next_segment_index);
        self.previous_trim.segment_index = remap(self.previous_trim.segment_index);
        self.next_trim.segment_index = remap(self.next_trim.segment_index);
        for segment_report in &mut self.segment_reports {
            segment_report.source_segment_index = remap(segment_report.source_segment_index);
        }
    }
}

impl CurveStringChamferResult2 {
    /// Returns the materialized chamfered curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized chamfered curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained chamfer report.
    pub const fn report(&self) -> &CurveStringChamferReport2 {
        &self.report
    }

    pub(crate) const fn report_mut(&mut self) -> &mut CurveStringChamferReport2 {
        &mut self.report
    }
}

impl CurveStringFilletReport2 {
    /// Returns how the tangent-point evidence was supplied to the fillet.
    pub const fn input_path(&self) -> CurveStringFilletInputPath2 {
        self.input_path
    }

    /// Returns the furthest exact fillet stage reached.
    pub const fn stage(&self) -> CurveStringFilletStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to validate this fillet.
    pub const fn predicate_path(&self) -> CurveStringFilletPredicatePath2 {
        self.predicate_path
    }

    /// Returns the previous source segment index at the filleted vertex.
    pub const fn previous_segment_index(&self) -> usize {
        self.previous_segment_index
    }

    /// Returns the next source segment index at the filleted vertex.
    pub const fn next_segment_index(&self) -> usize {
        self.next_segment_index
    }

    /// Returns the exact start point of the previous source segment.
    pub const fn previous_segment_start_point(&self) -> &Point2 {
        &self.previous_segment_start_point
    }

    /// Returns the exact end point of the previous source segment.
    pub const fn previous_segment_end_point(&self) -> &Point2 {
        &self.previous_segment_end_point
    }

    /// Returns the exact start point of the next source segment.
    pub const fn next_segment_start_point(&self) -> &Point2 {
        &self.next_segment_start_point
    }

    /// Returns the exact end point of the next source segment.
    pub const fn next_segment_end_point(&self) -> &Point2 {
        &self.next_segment_end_point
    }

    /// Returns the previous line trim point.
    pub const fn previous_trim(&self) -> &CurveStringTrimPoint2 {
        &self.previous_trim
    }

    /// Returns the next line trim point.
    pub const fn next_trim(&self) -> &CurveStringTrimPoint2 {
        &self.next_trim
    }

    /// Returns the exact previous-line tangent point when the fillet materialized.
    pub const fn previous_tangent_point(&self) -> Option<&Point2> {
        self.previous_tangent_point.as_ref()
    }

    /// Returns the exact next-line tangent point when the fillet materialized.
    pub const fn next_tangent_point(&self) -> Option<&Point2> {
        self.next_tangent_point.as_ref()
    }

    /// Returns the certified fillet center when the attempt reached center validation.
    pub const fn center(&self) -> Option<&Point2> {
        self.center.as_ref()
    }

    /// Returns the certified squared radius when the attempt reached radius validation.
    pub const fn radius_squared(&self) -> Option<&Real> {
        self.radius_squared.as_ref()
    }

    /// Returns retained source ranges for the shortened adjacent line segments.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        &self.segment_reports
    }

    /// Returns retained adjacent-source trim range count.
    pub const fn trim_segment_report_count(&self) -> usize {
        self.segment_reports.len()
    }

    /// Returns the inserted fillet arc segment index in the output curve string.
    pub const fn fillet_segment_index(&self) -> Option<usize> {
        self.fillet_segment_index
    }

    /// Returns the primitive family of the inserted fillet segment, when materialized.
    pub const fn fillet_segment_kind(&self) -> Option<SegmentKind> {
        self.fillet_segment_kind
    }

    /// Returns the exact start point of the inserted fillet arc.
    pub const fn fillet_segment_start_point(&self) -> Option<&Point2> {
        self.fillet_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the inserted fillet arc.
    pub const fn fillet_segment_end_point(&self) -> Option<&Point2> {
        self.fillet_segment_end_point.as_ref()
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns output segment count when the fillet materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized fillet output.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns fillet materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized fillets.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }

    pub(crate) fn remap_source_segment_indices(&mut self, mut remap: impl FnMut(usize) -> usize) {
        self.previous_segment_index = remap(self.previous_segment_index);
        self.next_segment_index = remap(self.next_segment_index);
        self.previous_trim.segment_index = remap(self.previous_trim.segment_index);
        self.next_trim.segment_index = remap(self.next_trim.segment_index);
        for segment_report in &mut self.segment_reports {
            segment_report.source_segment_index = remap(segment_report.source_segment_index);
        }
    }
}

impl CurveStringFilletResult2 {
    /// Returns the materialized filleted curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized filleted curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained fillet report.
    pub const fn report(&self) -> &CurveStringFilletReport2 {
        &self.report
    }

    pub(crate) const fn report_mut(&mut self) -> &mut CurveStringFilletReport2 {
        &mut self.report
    }
}

impl CurveStringCurveTrimHit2 {
    /// Returns the source segment index on the trimmed curve string.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the cutter segment index that produced this hit.
    pub const fn cutter_segment_index(&self) -> usize {
        self.cutter_segment_index
    }

    /// Returns the primitive family of the source segment that produced this hit.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the primitive family of the cutter segment that produced this hit.
    pub const fn cutter_segment_kind(&self) -> SegmentKind {
        self.cutter_segment_kind
    }

    /// Returns the exact start point of the source segment that produced this hit.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the source segment that produced this hit.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the exact start point of the cutter segment that produced this hit.
    pub const fn cutter_segment_start_point(&self) -> &Point2 {
        &self.cutter_segment_start_point
    }

    /// Returns the exact end point of the cutter segment that produced this hit.
    pub const fn cutter_segment_end_point(&self) -> &Point2 {
        &self.cutter_segment_end_point
    }

    /// Returns the exact affine parameter on the source segment.
    pub const fn source_param(&self) -> &Real {
        &self.source_param
    }

    /// Returns the exact affine parameter on the cutter segment.
    pub const fn cutter_param(&self) -> &Real {
        &self.cutter_param
    }

    /// Returns the exact intersection point witness.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the local intersection kind.
    pub const fn kind(&self) -> IntersectionKind {
        self.kind
    }
}

impl CurveStringCurveTrimReport2 {
    /// Returns retained start-cutter point witnesses.
    pub fn start_hits(&self) -> &[CurveStringCurveTrimHit2] {
        &self.start_hits
    }

    /// Returns retained end-cutter point witnesses.
    pub fn end_hits(&self) -> &[CurveStringCurveTrimHit2] {
        &self.end_hits
    }

    /// Returns the downstream point-bearing trim report, when attempted.
    pub const fn trim_report(&self) -> Option<&CurveStringTrimReport2> {
        self.trim_report.as_ref()
    }

    /// Returns scan evidence for the start-cutter intersection query.
    pub const fn start_intersection_report(&self) -> &CurveStringIntersectionReport2 {
        &self.start_intersection_report
    }

    /// Returns prepared-cache evidence for the start-cutter intersection query, when used.
    pub const fn start_prepared_cache_report(
        &self,
    ) -> Option<&CurveStringIntersectionPreparedCacheReport2> {
        self.start_intersection_report.prepared_cache_report()
    }

    /// Returns scan evidence for the end-cutter intersection query.
    pub const fn end_intersection_report(&self) -> &CurveStringIntersectionReport2 {
        &self.end_intersection_report
    }

    /// Returns prepared-cache evidence for the end-cutter intersection query, when used.
    pub const fn end_prepared_cache_report(
        &self,
    ) -> Option<&CurveStringIntersectionPreparedCacheReport2> {
        self.end_intersection_report.prepared_cache_report()
    }

    /// Returns the intersection query path used to collect split evidence.
    pub const fn query_path(&self) -> CurveStringCurveTrimQueryPath2 {
        self.query_path
    }

    /// Returns the furthest exact trim stage reached.
    pub const fn stage(&self) -> CurveStringCurveTrimStage2 {
        self.stage
    }

    /// Returns trim-by-curve materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized trims.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringCurveTrimResult2 {
    /// Returns the materialized native trim, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized native trim, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained curve-intersection trim report.
    pub const fn report(&self) -> &CurveStringCurveTrimReport2 {
        &self.report
    }
}

impl CurveStringRegionTrimHit2 {
    /// Returns the source curve-string segment index.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the source curve-string segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the exact start point of the source curve-string segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the source curve-string segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns whether the hit came from a material or hole contour.
    pub const fn region_contour_role(&self) -> RegionContourRole {
        self.region_contour_role
    }

    /// Returns the contour index within its region role bin.
    pub const fn region_contour_index(&self) -> usize {
        self.region_contour_index
    }

    /// Returns the region boundary segment index.
    pub const fn region_segment_index(&self) -> usize {
        self.region_segment_index
    }

    /// Returns the primitive family of the region boundary segment.
    pub const fn region_segment_kind(&self) -> SegmentKind {
        self.region_segment_kind
    }

    /// Returns the exact start point of the region boundary segment.
    pub const fn region_segment_start_point(&self) -> &Point2 {
        &self.region_segment_start_point
    }

    /// Returns the exact end point of the region boundary segment.
    pub const fn region_segment_end_point(&self) -> &Point2 {
        &self.region_segment_end_point
    }

    /// Returns the exact boundary point witness.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the retained source segment parameter.
    pub const fn source_param(&self) -> &Real {
        &self.source_param
    }

    /// Returns the retained region boundary segment parameter.
    pub const fn region_param(&self) -> &Real {
        &self.region_param
    }

    /// Returns the local intersection kind.
    pub const fn kind(&self) -> IntersectionKind {
        self.kind
    }
}

impl CurveStringRegionTrimIntervalReport2 {
    /// Returns the source segment index for this retained interval.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the source curve-string segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the exact start point of the source curve-string segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the source curve-string segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the retained source parameter range.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the exact start point witness for this classified interval.
    pub const fn range_start_point(&self) -> &Point2 {
        &self.range_start_point
    }

    /// Returns the exact end point witness for this classified interval.
    pub const fn range_end_point(&self) -> &Point2 {
        &self.range_end_point
    }

    /// Returns the representative point used for region classification.
    pub const fn representative_point(&self) -> Option<&Point2> {
        self.representative_point.as_ref()
    }

    /// Returns the region location of the representative, when decided.
    pub const fn location(&self) -> Option<RegionPointLocation> {
        self.location
    }

    /// Returns the output curve-string index that consumed this interval.
    pub const fn output_curve_string_index(&self) -> Option<usize> {
        self.output_curve_string_index
    }

    /// Returns the output segment index within the output curve string.
    pub const fn output_segment_index(&self) -> Option<usize> {
        self.output_segment_index
    }

    /// Returns the primitive family of the emitted output segment, when retained.
    pub const fn output_segment_kind(&self) -> Option<SegmentKind> {
        self.output_segment_kind
    }

    /// Returns the exact start point of the emitted output segment, when retained.
    pub const fn output_segment_start_point(&self) -> Option<&Point2> {
        self.output_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the emitted output segment, when retained.
    pub const fn output_segment_end_point(&self) -> Option<&Point2> {
        self.output_segment_end_point.as_ref()
    }

    /// Returns retained topology status for this interval.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for this interval, if any.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringRegionTrimReport2 {
    /// Returns the source segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the number of material contours in the clipping region.
    pub const fn region_material_contour_count(&self) -> usize {
        self.region_material_contour_count
    }

    /// Returns the number of hole contours in the clipping region.
    pub const fn region_hole_contour_count(&self) -> usize {
        self.region_hole_contour_count
    }

    /// Returns the material boundary segment count in the clipping region.
    pub const fn region_material_segment_count(&self) -> usize {
        self.region_material_segment_count
    }

    /// Returns the hole boundary segment count in the clipping region.
    pub const fn region_hole_segment_count(&self) -> usize {
        self.region_hole_segment_count
    }

    /// Returns primitive-family counts for material boundary segments.
    pub const fn region_material_segment_kind_counts(&self) -> SegmentKindCounts {
        self.region_material_segment_kind_counts
    }

    /// Returns primitive-family counts for hole boundary segments.
    pub const fn region_hole_segment_kind_counts(&self) -> SegmentKindCounts {
        self.region_hole_segment_kind_counts
    }

    /// Returns the exact predicate/filter path used for region-boundary hits.
    pub const fn boundary_predicate_path(&self) -> CurveStringRegionTrimBoundaryPredicatePath2 {
        self.boundary_predicate_path
    }

    /// Returns boundary segment-pair candidates considered while collecting trim hits.
    pub const fn boundary_candidate_pair_count(&self) -> usize {
        self.boundary_candidate_pair_count
    }

    /// Returns boundary segment-pair candidates skipped by decided disjoint AABBs.
    pub const fn boundary_skipped_aabb_pair_count(&self) -> usize {
        self.boundary_skipped_aabb_pair_count
    }

    /// Returns boundary segment pairs tested by exact segment topology.
    pub const fn boundary_tested_pair_count(&self) -> usize {
        self.boundary_tested_pair_count
    }

    /// Returns exact region-boundary hits retained as split evidence.
    pub const fn boundary_hit_count(&self) -> usize {
        self.boundary_hit_count
    }

    /// Returns tested boundary relations that produced one or more exact split hits.
    pub const fn boundary_point_relation_count(&self) -> usize {
        self.boundary_point_relation_count
    }

    /// Returns tested boundary relations blocked by exact overlap topology.
    pub const fn boundary_overlap_relation_count(&self) -> usize {
        self.boundary_overlap_relation_count
    }

    /// Returns tested boundary relations left unresolved by the active policy.
    pub const fn boundary_uncertain_relation_count(&self) -> usize {
        self.boundary_uncertain_relation_count
    }

    /// Returns source intervals considered after splitting at retained boundary hits.
    pub const fn interval_candidate_count(&self) -> usize {
        self.interval_candidate_count
    }

    /// Returns retained representative-point classifications attempted for intervals.
    pub const fn interval_classification_count(&self) -> usize {
        self.interval_classification_count
    }

    /// Returns exact boundary hits used as split evidence.
    pub fn boundary_hits(&self) -> &[CurveStringRegionTrimHit2] {
        &self.boundary_hits
    }

    /// Returns retained interval classifications.
    pub fn interval_reports(&self) -> &[CurveStringRegionTrimIntervalReport2] {
        &self.interval_reports
    }

    /// Returns output curve-string count when trim-by-region materialized.
    pub const fn output_curve_string_count(&self) -> Option<usize> {
        self.output_curve_string_count
    }

    /// Returns total emitted segment count when trim-by-region materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for emitted inside segments.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns the query path used to collect boundary and classification evidence.
    pub const fn query_path(&self) -> CurveStringRegionTrimQueryPath2 {
        self.query_path
    }

    /// Returns prepared-cache inventory and freshness evidence, when used.
    pub const fn prepared_cache_report(
        &self,
    ) -> Option<&CurveStringRegionTrimPreparedCacheReport2> {
        self.prepared_cache_report.as_ref()
    }

    /// Returns the furthest exact trim-by-region stage reached.
    pub const fn stage(&self) -> CurveStringRegionTrimStage2 {
        self.stage
    }

    /// Returns trim-by-region materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized trim-by-region attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringRegionTrimPreparedCacheReport2 {
    /// Builds trim-by-region prepared-cache evidence.
    pub(crate) const fn new(
        source: CurveStringPreparedCacheAudit2,
        region: RegionTrimPreparedCacheAudit2,
    ) -> Self {
        Self { source, region }
    }

    /// Returns prepared-cache evidence for the source curve string.
    pub const fn source(&self) -> &CurveStringPreparedCacheAudit2 {
        &self.source
    }

    /// Returns prepared-cache evidence for the clipping region.
    pub const fn region(&self) -> &RegionTrimPreparedCacheAudit2 {
        &self.region
    }
}

impl RegionTrimPreparedCacheAudit2 {
    /// Builds per-region prepared cache evidence for trim-by-region.
    pub(crate) const fn new(
        prepared_contour_count: usize,
        prepared_material_segment_count: usize,
        prepared_material_segment_kind_counts: SegmentKindCounts,
        prepared_hole_segment_count: usize,
        prepared_hole_segment_kind_counts: SegmentKindCounts,
        prepared_segment_count: usize,
        prepared_segment_kind_counts: SegmentKindCounts,
        decided_segment_box_count: usize,
        undecided_segment_box_count: usize,
        region_box_decided: bool,
    ) -> Self {
        Self {
            freshness: CurveStringPreparedCacheFreshness2::BorrowedCurrentSource,
            prepared_contour_count,
            prepared_material_segment_count,
            prepared_material_segment_kind_counts,
            prepared_hole_segment_count,
            prepared_hole_segment_kind_counts,
            prepared_segment_count,
            prepared_segment_kind_counts,
            decided_segment_box_count,
            undecided_segment_box_count,
            region_box_decided,
        }
    }

    /// Returns the cache freshness claim for this borrowed prepared view.
    pub const fn freshness(&self) -> CurveStringPreparedCacheFreshness2 {
        self.freshness
    }

    /// Returns prepared material and hole contour count.
    pub const fn prepared_contour_count(&self) -> usize {
        self.prepared_contour_count
    }

    /// Returns prepared material segment count.
    pub const fn prepared_material_segment_count(&self) -> usize {
        self.prepared_material_segment_count
    }

    /// Returns primitive-family counts for prepared material segments.
    pub const fn prepared_material_segment_kind_counts(&self) -> SegmentKindCounts {
        self.prepared_material_segment_kind_counts
    }

    /// Returns prepared hole segment count.
    pub const fn prepared_hole_segment_count(&self) -> usize {
        self.prepared_hole_segment_count
    }

    /// Returns primitive-family counts for prepared hole segments.
    pub const fn prepared_hole_segment_kind_counts(&self) -> SegmentKindCounts {
        self.prepared_hole_segment_kind_counts
    }

    /// Returns prepared material and hole segment count.
    pub const fn prepared_segment_count(&self) -> usize {
        self.prepared_segment_count
    }

    /// Returns primitive-family counts for all prepared region segments.
    pub const fn prepared_segment_kind_counts(&self) -> SegmentKindCounts {
        self.prepared_segment_kind_counts
    }

    /// Returns decided segment AABB count retained by preparation.
    pub const fn decided_segment_box_count(&self) -> usize {
        self.decided_segment_box_count
    }

    /// Returns segment AABB count that remained undecided.
    pub const fn undecided_segment_box_count(&self) -> usize {
        self.undecided_segment_box_count
    }

    /// Returns whether preparation retained a decided whole-region AABB.
    pub const fn region_box_decided(&self) -> bool {
        self.region_box_decided
    }
}

impl CurveStringRegionTrimResult2 {
    /// Returns materialized inside curve-string chains.
    pub fn curve_strings(&self) -> &[CurveString2] {
        &self.curve_strings
    }

    /// Consumes this result and returns materialized inside curve-string chains.
    pub fn into_curve_strings(self) -> Vec<CurveString2> {
        self.curve_strings
    }

    /// Returns the retained trim-by-region report.
    pub const fn report(&self) -> &CurveStringRegionTrimReport2 {
        &self.report
    }
}

impl CurveStringExtendReport2 {
    /// Returns the furthest exact extension stage reached.
    pub const fn stage(&self) -> CurveStringExtendStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to validate the extension target.
    pub const fn predicate_path(&self) -> CurveStringExtendPredicatePath2 {
        self.predicate_path
    }

    /// Returns which endpoint was extended.
    pub const fn endpoint(&self) -> CurveStringEndpoint2 {
        self.endpoint
    }

    /// Returns the endpoint segment index in the source curve string.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the endpoint segment before extension.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the output segment index carrying the extended geometry, when materialized.
    pub const fn output_segment_index(&self) -> Option<usize> {
        self.output_segment_index
    }

    /// Returns the primitive family of the extended output segment, when materialized.
    pub const fn output_segment_kind(&self) -> Option<SegmentKind> {
        self.output_segment_kind
    }

    /// Returns the exact start point of the extended output segment.
    pub const fn output_segment_start_point(&self) -> Option<&Point2> {
        self.output_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the extended output segment.
    pub const fn output_segment_end_point(&self) -> Option<&Point2> {
        self.output_segment_end_point.as_ref()
    }

    /// Returns the exact start point of the source endpoint segment before extension.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the source endpoint segment before extension.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the exact source endpoint point before extension.
    pub const fn source_endpoint_point(&self) -> &Point2 {
        &self.source_endpoint_point
    }

    /// Returns the requested exact target point.
    pub const fn target_point(&self) -> &Point2 {
        &self.target_point
    }

    /// Returns the affine parameter on the source endpoint line, when certified.
    pub const fn source_param(&self) -> Option<&Real> {
        self.source_param.as_ref()
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for the source curve string.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the output curve-string segment count after materialization.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the materialized extension output.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns extension materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized extensions.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringExtendResult2 {
    /// Returns the materialized extended curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized extended curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Consumes this result and returns retained extension evidence.
    pub fn into_report(self) -> CurveStringExtendReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized extension with its report.
    pub fn into_parts(self) -> (Option<CurveString2>, CurveStringExtendReport2) {
        (self.curve_string, self.report)
    }

    /// Returns the retained extension report.
    pub const fn report(&self) -> &CurveStringExtendReport2 {
        &self.report
    }

    /// Returns the extension output as a convenience classification while retaining this result.
    pub fn curve_string_classification(&self) -> Classification<&CurveString2> {
        match self.curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the extension output as a convenience classification.
    pub fn into_curve_string_classification(self) -> Classification<CurveString2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(blocker),
        }
    }
}

enum SegmentTrimMaterialization {
    Materialized(Segment2),
    SkippedEmpty,
    Unsupported(UncertaintyReason),
    Unresolved(UncertaintyReason),
}

#[derive(Clone, Debug, PartialEq)]
struct LocatedTrimPoint2 {
    trim_point: CurveStringTrimPoint2,
    point: Point2,
}

struct CurveTrimHitExtraction {
    hits: Vec<CurveStringCurveTrimHit2>,
    blocker: Option<(RetainedTopologyStatus, UncertaintyReason)>,
}

#[derive(Clone, Debug, PartialEq)]
struct RegionTrimSplitPoint2 {
    trim_point: CurveStringTrimPoint2,
    point: Point2,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct RegionTrimBoundaryWorkload {
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct EndpointPairSummary {
    pair_count: usize,
    exact_count: usize,
    disconnected_count: usize,
    unresolved_count: usize,
}

impl EndpointPairSummary {
    fn from_report(report: &CurveStringEndpointConnectionReport2) -> Self {
        let mut summary = Self {
            pair_count: 1,
            ..Self::default()
        };
        summary.add_status(report.status);
        summary
    }

    fn add_status(&mut self, status: CurveStringEndpointConnectionStatus2) {
        match status {
            CurveStringEndpointConnectionStatus2::NativeExact => self.exact_count += 1,
            CurveStringEndpointConnectionStatus2::Disconnected => self.disconnected_count += 1,
            CurveStringEndpointConnectionStatus2::Unresolved(_) => self.unresolved_count += 1,
        }
    }
}

enum NearestEndpointChoice {
    Selected(CurveStringLinkKind2, CurveStringEndpointConnectionReport2),
    Ambiguous(CurveStringLinkKind2, CurveStringEndpointConnectionReport2),
    Unresolved(
        CurveStringLinkKind2,
        CurveStringEndpointConnectionReport2,
        UncertaintyReason,
    ),
    Empty,
}

fn trim_curve_string_inside_region(
    curve_string: &CurveString2,
    region: &Region2,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringRegionTrimResult2> {
    let mut boundary_hits = Vec::new();
    let mut boundary_workload = RegionTrimBoundaryWorkload::default();
    if let Some((status, blocker)) = collect_region_trim_boundary_hits(
        curve_string,
        region,
        policy,
        &mut boundary_hits,
        &mut boundary_workload,
    )? {
        return Ok(blocked_region_trim_result(
            curve_string,
            region.material_contours().len(),
            region.hole_contours().len(),
            region_material_segment_count(region),
            region_hole_segment_count(region),
            contours_segment_kind_counts(region.material_contours()),
            contours_segment_kind_counts(region.hole_contours()),
            boundary_workload,
            0,
            0,
            boundary_hits,
            Vec::new(),
            CurveStringRegionTrimQueryPath2::Direct,
            None,
            CurveStringRegionTrimStage2::BoundaryCollection,
            status,
            blocker,
        ));
    }

    trim_curve_string_inside_region_with_hits(
        curve_string,
        region.material_contours().len(),
        region.hole_contours().len(),
        region_material_segment_count(region),
        region_hole_segment_count(region),
        contours_segment_kind_counts(region.material_contours()),
        contours_segment_kind_counts(region.hole_contours()),
        boundary_workload,
        boundary_hits,
        CurveStringRegionTrimQueryPath2::Direct,
        None,
        policy,
        |point| region.classify_point(point, policy),
    )
}

fn region_material_segment_count(region: &Region2) -> usize {
    region
        .material_contours()
        .iter()
        .map(|contour| contour.len())
        .sum()
}

fn region_hole_segment_count(region: &Region2) -> usize {
    region
        .hole_contours()
        .iter()
        .map(|contour| contour.len())
        .sum()
}

fn trim_curve_string_inside_prepared_region(
    curve_string: &CurveString2,
    region: &PreparedRegionView2<'_>,
    prepared_cache_report: Option<CurveStringRegionTrimPreparedCacheReport2>,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringRegionTrimResult2> {
    let mut boundary_hits = Vec::new();
    let mut boundary_workload = RegionTrimBoundaryWorkload::default();
    if let Some((status, blocker)) = collect_prepared_region_trim_boundary_hits(
        curve_string,
        region,
        policy,
        &mut boundary_hits,
        &mut boundary_workload,
    )? {
        return Ok(blocked_region_trim_result(
            curve_string,
            region.material_contours().len(),
            region.hole_contours().len(),
            region.prepared_material_segment_count(),
            region.prepared_hole_segment_count(),
            contour_refs_segment_kind_counts(region.material_contours()),
            contour_refs_segment_kind_counts(region.hole_contours()),
            boundary_workload,
            0,
            0,
            boundary_hits,
            Vec::new(),
            CurveStringRegionTrimQueryPath2::Prepared,
            prepared_cache_report,
            CurveStringRegionTrimStage2::BoundaryCollection,
            status,
            blocker,
        ));
    }

    trim_curve_string_inside_region_with_hits(
        curve_string,
        region.material_contours().len(),
        region.hole_contours().len(),
        region.prepared_material_segment_count(),
        region.prepared_hole_segment_count(),
        contour_refs_segment_kind_counts(region.material_contours()),
        contour_refs_segment_kind_counts(region.hole_contours()),
        boundary_workload,
        boundary_hits,
        CurveStringRegionTrimQueryPath2::Prepared,
        prepared_cache_report,
        policy,
        |point| region.classify_point(point, policy),
    )
}

fn trim_curve_string_inside_region_with_hits(
    curve_string: &CurveString2,
    region_material_contour_count: usize,
    region_hole_contour_count: usize,
    region_material_segment_count: usize,
    region_hole_segment_count: usize,
    region_material_segment_kind_counts: SegmentKindCounts,
    region_hole_segment_kind_counts: SegmentKindCounts,
    boundary_workload: RegionTrimBoundaryWorkload,
    boundary_hits: Vec<CurveStringRegionTrimHit2>,
    query_path: CurveStringRegionTrimQueryPath2,
    prepared_cache_report: Option<CurveStringRegionTrimPreparedCacheReport2>,
    policy: &CurvePolicy,
    mut classify_point: impl FnMut(&Point2) -> Classification<RegionPointLocation>,
) -> CurveResult<CurveStringRegionTrimResult2> {
    let mut output_segments: Vec<Vec<Segment2>> = Vec::new();
    let mut current_segments = Vec::new();
    let mut interval_reports = Vec::new();
    let mut interval_candidate_count = 0_usize;
    let mut interval_classification_count = 0_usize;

    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        let split_points = match region_trim_split_points_for_segment(
            source_segment_index,
            source_segment,
            &boundary_hits,
            policy,
        )? {
            Classification::Decided(split_points) => split_points,
            Classification::Uncertain(reason) => {
                return Ok(blocked_region_trim_result(
                    curve_string,
                    region_material_contour_count,
                    region_hole_contour_count,
                    region_material_segment_count,
                    region_hole_segment_count,
                    region_material_segment_kind_counts,
                    region_hole_segment_kind_counts,
                    boundary_workload,
                    interval_candidate_count,
                    interval_classification_count,
                    boundary_hits,
                    interval_reports,
                    query_path,
                    prepared_cache_report.clone(),
                    CurveStringRegionTrimStage2::IntervalClassification,
                    retained_status_for_uncertainty(reason),
                    reason,
                ));
            }
        };

        for window in split_points.windows(2) {
            let start = &window[0];
            let end = &window[1];
            interval_candidate_count += 1;
            let source_range =
                ParamRange::new(start.trim_point.param.clone(), end.trim_point.param.clone());
            let fragment = match trim_segment_by_point_range(
                source_segment,
                &source_range,
                &start.point,
                &end.point,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(fragment) => fragment,
                SegmentTrimMaterialization::SkippedEmpty => continue,
                SegmentTrimMaterialization::Unsupported(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: None,
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        output_segment_kind: None,
                        output_segment_start_point: None,
                        output_segment_end_point: None,
                        status: RetainedTopologyStatus::Unsupported,
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        region_material_segment_count,
                        region_hole_segment_count,
                        region_material_segment_kind_counts,
                        region_hole_segment_kind_counts,
                        boundary_workload,
                        interval_candidate_count,
                        interval_classification_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        prepared_cache_report.clone(),
                        CurveStringRegionTrimStage2::IntervalClassification,
                        RetainedTopologyStatus::Unsupported,
                        reason,
                    ));
                }
                SegmentTrimMaterialization::Unresolved(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: None,
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        output_segment_kind: None,
                        output_segment_start_point: None,
                        output_segment_end_point: None,
                        status: RetainedTopologyStatus::Unresolved,
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        region_material_segment_count,
                        region_hole_segment_count,
                        region_material_segment_kind_counts,
                        region_hole_segment_kind_counts,
                        boundary_workload,
                        interval_candidate_count,
                        interval_classification_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        prepared_cache_report.clone(),
                        CurveStringRegionTrimStage2::IntervalClassification,
                        RetainedTopologyStatus::Unresolved,
                        reason,
                    ));
                }
            };

            let representative = match fragment.representative_point(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: None,
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        output_segment_kind: None,
                        output_segment_start_point: None,
                        output_segment_end_point: None,
                        status: retained_status_for_uncertainty(reason),
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        region_material_segment_count,
                        region_hole_segment_count,
                        region_material_segment_kind_counts,
                        region_hole_segment_kind_counts,
                        boundary_workload,
                        interval_candidate_count,
                        interval_classification_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        prepared_cache_report.clone(),
                        CurveStringRegionTrimStage2::IntervalClassification,
                        retained_status_for_uncertainty(reason),
                        reason,
                    ));
                }
            };

            interval_classification_count += 1;
            let location = match classify_point(&representative) {
                Classification::Decided(location) => location,
                Classification::Uncertain(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: Some(representative),
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        output_segment_kind: None,
                        output_segment_start_point: None,
                        output_segment_end_point: None,
                        status: retained_status_for_uncertainty(reason),
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        region_material_segment_count,
                        region_hole_segment_count,
                        region_material_segment_kind_counts,
                        region_hole_segment_kind_counts,
                        boundary_workload,
                        interval_candidate_count,
                        interval_classification_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        prepared_cache_report.clone(),
                        CurveStringRegionTrimStage2::IntervalClassification,
                        retained_status_for_uncertainty(reason),
                        reason,
                    ));
                }
            };

            match location {
                RegionPointLocation::Inside => {
                    let output_curve_string_index = output_segments.len();
                    let output_segment_index = current_segments.len();
                    let output_segment_kind = fragment.structural_facts().kind;
                    let output_segment_start_point = fragment.start().clone();
                    let output_segment_end_point = fragment.end().clone();
                    current_segments.push(fragment);
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: Some(representative),
                        location: Some(location),
                        output_curve_string_index: Some(output_curve_string_index),
                        output_segment_index: Some(output_segment_index),
                        output_segment_kind: Some(output_segment_kind),
                        output_segment_start_point: Some(output_segment_start_point),
                        output_segment_end_point: Some(output_segment_end_point),
                        status: RetainedTopologyStatus::NativeExact,
                        blocker: None,
                    });
                }
                RegionPointLocation::Outside => {
                    flush_region_trim_chain(&mut output_segments, &mut current_segments);
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: Some(representative),
                        location: Some(location),
                        output_curve_string_index: None,
                        output_segment_index: None,
                        output_segment_kind: None,
                        output_segment_start_point: None,
                        output_segment_end_point: None,
                        status: RetainedTopologyStatus::NativeExact,
                        blocker: None,
                    });
                }
                RegionPointLocation::Boundary => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_segment_kind: source_segment.structural_facts().kind,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        range_start_point: start.point.clone(),
                        range_end_point: end.point.clone(),
                        representative_point: Some(representative),
                        location: Some(location),
                        output_curve_string_index: None,
                        output_segment_index: None,
                        output_segment_kind: None,
                        output_segment_start_point: None,
                        output_segment_end_point: None,
                        status: RetainedTopologyStatus::Unsupported,
                        blocker: Some(UncertaintyReason::Boundary),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        region_material_segment_count,
                        region_hole_segment_count,
                        region_material_segment_kind_counts,
                        region_hole_segment_kind_counts,
                        boundary_workload,
                        interval_candidate_count,
                        interval_classification_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        prepared_cache_report.clone(),
                        CurveStringRegionTrimStage2::IntervalClassification,
                        RetainedTopologyStatus::Unsupported,
                        UncertaintyReason::Boundary,
                    ));
                }
            }
        }
    }

    flush_region_trim_chain(&mut output_segments, &mut current_segments);
    let output_segment_count = output_segments.iter().map(Vec::len).sum();
    let mut curve_strings = Vec::with_capacity(output_segments.len());
    for segments in output_segments {
        curve_strings.push(CurveString2::try_new(segments)?);
    }
    let output_segment_kind_counts = curve_strings_segment_kind_counts(&curve_strings);

    Ok(CurveStringRegionTrimResult2 {
        report: CurveStringRegionTrimReport2 {
            source_segment_count: curve_string.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(curve_string),
            region_material_contour_count,
            region_hole_contour_count,
            region_material_segment_count,
            region_hole_segment_count,
            region_material_segment_kind_counts,
            region_hole_segment_kind_counts,
            boundary_predicate_path:
                CurveStringRegionTrimBoundaryPredicatePath2::AabbFilteredExactSegmentIntersections,
            boundary_candidate_pair_count: boundary_workload.candidate_pair_count,
            boundary_skipped_aabb_pair_count: boundary_workload.skipped_aabb_pair_count,
            boundary_tested_pair_count: boundary_workload.tested_pair_count,
            boundary_hit_count: boundary_hits.len(),
            boundary_point_relation_count: boundary_workload.point_relation_count,
            boundary_overlap_relation_count: boundary_workload.overlap_relation_count,
            boundary_uncertain_relation_count: boundary_workload.uncertain_relation_count,
            interval_candidate_count,
            interval_classification_count,
            boundary_hits,
            interval_reports,
            output_curve_string_count: Some(curve_strings.len()),
            output_segment_count: Some(output_segment_count),
            output_segment_kind_counts: Some(output_segment_kind_counts),
            query_path,
            prepared_cache_report,
            stage: CurveStringRegionTrimStage2::OutputMaterialization,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
        curve_strings,
    })
}

fn collect_region_trim_boundary_hits(
    curve_string: &CurveString2,
    region: &Region2,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    workload: &mut RegionTrimBoundaryWorkload,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    for (contour_index, contour) in region.material_contours().iter().enumerate() {
        if let Some(blocker) = collect_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Material,
            contour_index,
            policy,
            hits,
            workload,
        )? {
            return Ok(Some(blocker));
        }
    }
    for (contour_index, contour) in region.hole_contours().iter().enumerate() {
        if let Some(blocker) = collect_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Hole,
            contour_index,
            policy,
            hits,
            workload,
        )? {
            return Ok(Some(blocker));
        }
    }
    Ok(None)
}

fn collect_prepared_region_trim_boundary_hits(
    curve_string: &CurveString2,
    region: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    workload: &mut RegionTrimBoundaryWorkload,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    for (contour_index, contour) in region.prepared_material_contours().iter().enumerate() {
        if let Some(blocker) = collect_prepared_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Material,
            contour_index,
            policy,
            hits,
            workload,
        )? {
            return Ok(Some(blocker));
        }
    }
    for (contour_index, contour) in region.prepared_hole_contours().iter().enumerate() {
        if let Some(blocker) = collect_prepared_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Hole,
            contour_index,
            policy,
            hits,
            workload,
        )? {
            return Ok(Some(blocker));
        }
    }
    Ok(None)
}

fn collect_region_trim_contour_hits(
    curve_string: &CurveString2,
    contour: &crate::Contour2,
    role: RegionContourRole,
    contour_index: usize,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    workload: &mut RegionTrimBoundaryWorkload,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    let source_segment_boxes: Vec<_> = curve_string
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
    let region_segment_boxes: Vec<_> = contour
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();

    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        for (region_segment_index, region_segment) in contour.segments().iter().enumerate() {
            workload.candidate_pair_count += 1;
            if let (Some(Some(source_box)), Some(Some(region_box))) = (
                source_segment_boxes.get(source_segment_index),
                region_segment_boxes.get(region_segment_index),
            ) && aabbs_decided_disjoint(source_box, region_box, policy)
            {
                workload.skipped_aabb_pair_count += 1;
                continue;
            }

            workload.tested_pair_count += 1;
            let relation = source_segment.intersect_segment(region_segment, policy)?;
            if let Some(blocker) = append_region_trim_hits_from_relation(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                relation,
                workload,
                policy,
            )? {
                return Ok(Some(blocker));
            }
        }
    }
    Ok(None)
}

fn collect_prepared_region_trim_contour_hits(
    curve_string: &CurveString2,
    contour: &crate::PreparedContourView2<'_>,
    role: RegionContourRole,
    contour_index: usize,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    workload: &mut RegionTrimBoundaryWorkload,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    let source_segment_boxes: Vec<_> = curve_string
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        for (region_segment_index, region_segment) in
            contour.contour().segments().iter().enumerate()
        {
            workload.candidate_pair_count += 1;
            if let (Some(Some(source_box)), Some(Some(region_box))) = (
                source_segment_boxes.get(source_segment_index),
                contour.segment_boxes().get(region_segment_index),
            ) && aabbs_decided_disjoint(source_box, region_box, policy)
            {
                workload.skipped_aabb_pair_count += 1;
                continue;
            }

            workload.tested_pair_count += 1;
            let relation = source_segment.intersect_segment(region_segment, policy)?;
            if let Some(blocker) = append_region_trim_hits_from_relation(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                relation,
                workload,
                policy,
            )? {
                return Ok(Some(blocker));
            }
        }
    }
    Ok(None)
}

#[allow(clippy::too_many_arguments)]
fn append_region_trim_hits_from_relation(
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    source_segment_index: usize,
    source_segment: &Segment2,
    role: RegionContourRole,
    contour_index: usize,
    region_segment_index: usize,
    region_segment: &Segment2,
    relation: SegmentIntersection,
    workload: &mut RegionTrimBoundaryWorkload,
    policy: &CurvePolicy,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None)
        | SegmentIntersection::LineArc {
            result: LineArcIntersection::None,
            ..
        }
        | SegmentIntersection::ArcArc(ArcArcIntersection::None) => Ok(None),
        SegmentIntersection::LineLine(LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        }) => {
            workload.point_relation_count += 1;
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                point,
                a_param,
                b_param,
                kind,
            )
        }
        SegmentIntersection::LineArc {
            result: LineArcIntersection::Point(hit),
            order,
        } => {
            workload.point_relation_count += 1;
            match line_arc_region_trim_params(source_segment, region_segment, order, &hit, policy)?
            {
                Ok((source_param, region_param)) => push_region_trim_hit(
                    hits,
                    source_segment_index,
                    source_segment,
                    role,
                    contour_index,
                    region_segment_index,
                    region_segment,
                    hit.point,
                    source_param,
                    region_param,
                    hit.kind,
                ),
                Err(reason) => Ok(Some((retained_status_for_uncertainty(reason), reason))),
            }
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
            workload.point_relation_count += 1;
            match point_region_trim_params(source_segment, region_segment, &hit.point, policy)? {
                Ok((source_param, region_param)) => push_region_trim_hit(
                    hits,
                    source_segment_index,
                    source_segment,
                    role,
                    contour_index,
                    region_segment_index,
                    region_segment,
                    hit.point,
                    source_param,
                    region_param,
                    hit.kind,
                ),
                Err(reason) => Ok(Some((retained_status_for_uncertainty(reason), reason))),
            }
        }
        SegmentIntersection::LineArc {
            result: LineArcIntersection::TwoPoints { first, second },
            order,
        } => {
            workload.point_relation_count += 1;
            let (source_param, region_param) = match line_arc_region_trim_params(
                source_segment,
                region_segment,
                order,
                &first,
                policy,
            )? {
                Ok(params) => params,
                Err(reason) => return Ok(Some((retained_status_for_uncertainty(reason), reason))),
            };
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                first.point,
                source_param,
                region_param,
                first.kind,
            )?;
            let (source_param, region_param) = match line_arc_region_trim_params(
                source_segment,
                region_segment,
                order,
                &second,
                policy,
            )? {
                Ok(params) => params,
                Err(reason) => return Ok(Some((retained_status_for_uncertainty(reason), reason))),
            };
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                second.point,
                source_param,
                region_param,
                second.kind,
            )
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
            workload.point_relation_count += 1;
            let (source_param, region_param) = match point_region_trim_params(
                source_segment,
                region_segment,
                &first.point,
                policy,
            )? {
                Ok(params) => params,
                Err(reason) => {
                    return Ok(Some((retained_status_for_uncertainty(reason), reason)));
                }
            };
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                first.point,
                source_param,
                region_param,
                first.kind,
            )?;
            let (source_param, region_param) = match point_region_trim_params(
                source_segment,
                region_segment,
                &second.point,
                policy,
            )? {
                Ok(params) => params,
                Err(reason) => return Ok(Some((retained_status_for_uncertainty(reason), reason))),
            };
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                region_segment,
                second.point,
                source_param,
                region_param,
                second.kind,
            )
        }
        SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. })
        | SegmentIntersection::ArcArc(ArcArcIntersection::Overlap { .. }) => {
            workload.overlap_relation_count += 1;
            Ok(Some((
                RetainedTopologyStatus::Unsupported,
                UncertaintyReason::Unsupported,
            )))
        }
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason })
        | SegmentIntersection::LineArc {
            result: LineArcIntersection::Uncertain { reason },
            ..
        }
        | SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
            workload.uncertain_relation_count += 1;
            Ok(Some((RetainedTopologyStatus::Unresolved, reason)))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn push_region_trim_hit(
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    source_segment_index: usize,
    source_segment: &Segment2,
    role: RegionContourRole,
    contour_index: usize,
    region_segment_index: usize,
    region_segment: &Segment2,
    point: Point2,
    source_param: Real,
    region_param: Real,
    kind: IntersectionKind,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    hits.push(CurveStringRegionTrimHit2 {
        source_segment_index,
        source_segment_kind: source_segment.structural_facts().kind,
        source_segment_start_point: source_segment.start().clone(),
        source_segment_end_point: source_segment.end().clone(),
        region_contour_role: role,
        region_contour_index: contour_index,
        region_segment_index,
        region_segment_kind: region_segment.structural_facts().kind,
        region_segment_start_point: region_segment.start().clone(),
        region_segment_end_point: region_segment.end().clone(),
        point,
        source_param,
        region_param,
        kind,
    });
    Ok(None)
}

fn line_arc_region_trim_params(
    source_segment: &Segment2,
    region_segment: &Segment2,
    order: LineArcOrder,
    hit: &crate::LineArcIntersectionPoint,
    policy: &CurvePolicy,
) -> CurveResult<Result<(Real, Real), UncertaintyReason>> {
    match order {
        LineArcOrder::LineThenArc => {
            let region_param = match segment_point_parameter(region_segment, &hit.point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => return Ok(Err(reason)),
            };
            Ok(Ok((hit.line_param.clone(), region_param)))
        }
        LineArcOrder::ArcThenLine => {
            let source_param = match segment_point_parameter(source_segment, &hit.point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => return Ok(Err(reason)),
            };
            Ok(Ok((source_param, hit.line_param.clone())))
        }
    }
}

fn point_region_trim_params(
    source_segment: &Segment2,
    region_segment: &Segment2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Result<(Real, Real), UncertaintyReason>> {
    let source_param = match segment_point_parameter(source_segment, point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(Err(reason)),
    };
    let region_param = match segment_point_parameter(region_segment, point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => return Ok(Err(reason)),
    };
    Ok(Ok((source_param, region_param)))
}

fn region_trim_split_points_for_segment(
    source_segment_index: usize,
    source_segment: &Segment2,
    hits: &[CurveStringRegionTrimHit2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<RegionTrimSplitPoint2>>> {
    let mut split_points = vec![RegionTrimSplitPoint2 {
        trim_point: CurveStringTrimPoint2::new(source_segment_index, Real::zero()),
        point: source_segment.start().clone(),
    }];

    for hit in hits
        .iter()
        .filter(|hit| hit.source_segment_index == source_segment_index)
    {
        match insert_region_trim_split_point(
            &mut split_points,
            RegionTrimSplitPoint2 {
                trim_point: CurveStringTrimPoint2::new(
                    source_segment_index,
                    hit.source_param.clone(),
                ),
                point: hit.point.clone(),
            },
            policy,
        ) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    match insert_region_trim_split_point(
        &mut split_points,
        RegionTrimSplitPoint2 {
            trim_point: CurveStringTrimPoint2::new(source_segment_index, Real::one()),
            point: source_segment.end().clone(),
        },
        policy,
    ) {
        Classification::Decided(()) => Ok(Classification::Decided(split_points)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn insert_region_trim_split_point(
    split_points: &mut Vec<RegionTrimSplitPoint2>,
    point: RegionTrimSplitPoint2,
    policy: &CurvePolicy,
) -> Classification<()> {
    for index in 0..split_points.len() {
        let ordering = match compare_reals(
            point.trim_point.param(),
            split_points[index].trim_point.param(),
            policy,
        ) {
            Some(ordering) => ordering,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        match ordering {
            Ordering::Less => {
                split_points.insert(index, point);
                return Classification::Decided(());
            }
            Ordering::Equal => {
                return match is_zero(
                    &point.point.distance_squared(&split_points[index].point),
                    policy,
                ) {
                    Some(true) => Classification::Decided(()),
                    Some(false) => Classification::Uncertain(UncertaintyReason::Boundary),
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                };
            }
            Ordering::Greater => {}
        }
    }
    split_points.push(point);
    Classification::Decided(())
}

fn flush_region_trim_chain(
    output_segments: &mut Vec<Vec<Segment2>>,
    current_segments: &mut Vec<Segment2>,
) {
    if !current_segments.is_empty() {
        output_segments.push(std::mem::take(current_segments));
    }
}

fn blocked_region_trim_result(
    curve_string: &CurveString2,
    region_material_contour_count: usize,
    region_hole_contour_count: usize,
    region_material_segment_count: usize,
    region_hole_segment_count: usize,
    region_material_segment_kind_counts: SegmentKindCounts,
    region_hole_segment_kind_counts: SegmentKindCounts,
    boundary_workload: RegionTrimBoundaryWorkload,
    interval_candidate_count: usize,
    interval_classification_count: usize,
    boundary_hits: Vec<CurveStringRegionTrimHit2>,
    interval_reports: Vec<CurveStringRegionTrimIntervalReport2>,
    query_path: CurveStringRegionTrimQueryPath2,
    prepared_cache_report: Option<CurveStringRegionTrimPreparedCacheReport2>,
    stage: CurveStringRegionTrimStage2,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> CurveStringRegionTrimResult2 {
    CurveStringRegionTrimResult2 {
        curve_strings: Vec::new(),
        report: CurveStringRegionTrimReport2 {
            source_segment_count: curve_string.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(curve_string),
            region_material_contour_count,
            region_hole_contour_count,
            region_material_segment_count,
            region_hole_segment_count,
            region_material_segment_kind_counts,
            region_hole_segment_kind_counts,
            boundary_predicate_path:
                CurveStringRegionTrimBoundaryPredicatePath2::AabbFilteredExactSegmentIntersections,
            boundary_candidate_pair_count: boundary_workload.candidate_pair_count,
            boundary_skipped_aabb_pair_count: boundary_workload.skipped_aabb_pair_count,
            boundary_tested_pair_count: boundary_workload.tested_pair_count,
            boundary_hit_count: boundary_hits.len(),
            boundary_point_relation_count: boundary_workload.point_relation_count,
            boundary_overlap_relation_count: boundary_workload.overlap_relation_count,
            boundary_uncertain_relation_count: boundary_workload.uncertain_relation_count,
            interval_candidate_count,
            interval_classification_count,
            boundary_hits,
            interval_reports,
            output_curve_string_count: None,
            output_segment_count: None,
            output_segment_kind_counts: None,
            query_path,
            prepared_cache_report,
            stage,
            status,
            blocker: Some(blocker),
        },
    }
}

fn extract_curve_trim_hits(
    source: &CurveString2,
    cutter: &CurveString2,
    events: &[CurveStringIntersection],
    policy: &CurvePolicy,
) -> CurveResult<CurveTrimHitExtraction> {
    let mut hits = Vec::new();
    let mut blocker = None;
    for event in events {
        match &event.relation {
            SegmentIntersection::LineLine(LineLineIntersection::None) => {}
            SegmentIntersection::LineLine(LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            }) => hits.push(curve_trim_hit(
                source,
                cutter,
                event,
                a_param.clone(),
                b_param.clone(),
                point.clone(),
                *kind,
            )),
            SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. }) => {
                blocker = Some((
                    RetainedTopologyStatus::Unsupported,
                    UncertaintyReason::Unsupported,
                ));
            }
            SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
                blocker = Some((RetainedTopologyStatus::Unresolved, *reason));
            }
            SegmentIntersection::LineArc { order, result } => match result {
                LineArcIntersection::None => {}
                LineArcIntersection::Point(hit) => {
                    match line_arc_curve_trim_params(source, cutter, event, *order, hit, policy)? {
                        Ok((source_param, cutter_param)) => hits.push(curve_trim_hit(
                            source,
                            cutter,
                            event,
                            source_param,
                            cutter_param,
                            hit.point.clone(),
                            hit.kind,
                        )),
                        Err(reason) => {
                            blocker = Some((retained_status_for_uncertainty(reason), reason));
                        }
                    }
                }
                LineArcIntersection::TwoPoints { first, second } => {
                    for hit in [first, second] {
                        match line_arc_curve_trim_params(
                            source, cutter, event, *order, hit, policy,
                        )? {
                            Ok((source_param, cutter_param)) => hits.push(curve_trim_hit(
                                source,
                                cutter,
                                event,
                                source_param,
                                cutter_param,
                                hit.point.clone(),
                                hit.kind,
                            )),
                            Err(reason) => {
                                blocker = Some((retained_status_for_uncertainty(reason), reason));
                            }
                        }
                    }
                }
                LineArcIntersection::Uncertain { reason } => {
                    blocker = Some((RetainedTopologyStatus::Unresolved, *reason));
                }
            },
            SegmentIntersection::ArcArc(ArcArcIntersection::None) => {}
            SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
                match point_curve_trim_params(source, cutter, event, &hit.point, policy)? {
                    Ok((source_param, cutter_param)) => hits.push(curve_trim_hit(
                        source,
                        cutter,
                        event,
                        source_param,
                        cutter_param,
                        hit.point.clone(),
                        hit.kind,
                    )),
                    Err(reason) => {
                        blocker = Some((retained_status_for_uncertainty(reason), reason));
                    }
                }
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
                for hit in [first, second] {
                    match point_curve_trim_params(source, cutter, event, &hit.point, policy)? {
                        Ok((source_param, cutter_param)) => hits.push(curve_trim_hit(
                            source,
                            cutter,
                            event,
                            source_param,
                            cutter_param,
                            hit.point.clone(),
                            hit.kind,
                        )),
                        Err(reason) => {
                            blocker = Some((retained_status_for_uncertainty(reason), reason));
                        }
                    }
                }
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::Overlap { .. }) => {
                blocker = Some((
                    RetainedTopologyStatus::Unsupported,
                    UncertaintyReason::Unsupported,
                ));
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
                blocker = Some((RetainedTopologyStatus::Unresolved, *reason));
            }
        }
    }

    Ok(CurveTrimHitExtraction { hits, blocker })
}

fn line_arc_curve_trim_params(
    source: &CurveString2,
    cutter: &CurveString2,
    event: &CurveStringIntersection,
    order: LineArcOrder,
    hit: &crate::LineArcIntersectionPoint,
    policy: &CurvePolicy,
) -> CurveResult<Result<(Real, Real), UncertaintyReason>> {
    match order {
        LineArcOrder::LineThenArc => {
            let cutter_segment = &cutter.segments[event.b_segment_index];
            let cutter_param = match segment_point_parameter(cutter_segment, &hit.point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => return Ok(Err(reason)),
            };
            Ok(Ok((hit.line_param.clone(), cutter_param)))
        }
        LineArcOrder::ArcThenLine => {
            let source_segment = &source.segments[event.a_segment_index];
            let source_param = match segment_point_parameter(source_segment, &hit.point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => return Ok(Err(reason)),
            };
            Ok(Ok((source_param, hit.line_param.clone())))
        }
    }
}

fn point_curve_trim_params(
    source: &CurveString2,
    cutter: &CurveString2,
    event: &CurveStringIntersection,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Result<(Real, Real), UncertaintyReason>> {
    let source_param =
        match segment_point_parameter(&source.segments[event.a_segment_index], point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Err(reason)),
        };
    let cutter_param =
        match segment_point_parameter(&cutter.segments[event.b_segment_index], point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Err(reason)),
        };
    Ok(Ok((source_param, cutter_param)))
}

fn curve_trim_hit(
    source: &CurveString2,
    cutter: &CurveString2,
    event: &CurveStringIntersection,
    source_param: Real,
    cutter_param: Real,
    point: Point2,
    kind: IntersectionKind,
) -> CurveStringCurveTrimHit2 {
    let source_segment = &source.segments[event.a_segment_index];
    let cutter_segment = &cutter.segments[event.b_segment_index];
    CurveStringCurveTrimHit2 {
        source_segment_index: event.a_segment_index,
        cutter_segment_index: event.b_segment_index,
        source_segment_kind: source_segment.structural_facts().kind,
        cutter_segment_kind: cutter_segment.structural_facts().kind,
        source_segment_start_point: source_segment.start().clone(),
        source_segment_end_point: source_segment.end().clone(),
        cutter_segment_start_point: cutter_segment.start().clone(),
        cutter_segment_end_point: cutter_segment.end().clone(),
        source_param,
        cutter_param,
        point,
        kind,
    }
}

fn single_curve_trim_hit(
    extraction: &CurveTrimHitExtraction,
) -> Result<CurveStringCurveTrimHit2, (RetainedTopologyStatus, UncertaintyReason)> {
    if let Some(blocker) = extraction.blocker {
        return Err(blocker);
    }
    match extraction.hits.len() {
        1 => Ok(extraction.hits[0].clone()),
        _ => Err((
            RetainedTopologyStatus::Unsupported,
            UncertaintyReason::Boundary,
        )),
    }
}

fn blocked_curve_trim_result(
    start_hits: Vec<CurveStringCurveTrimHit2>,
    end_hits: Vec<CurveStringCurveTrimHit2>,
    start_intersection_report: CurveStringIntersectionReport2,
    end_intersection_report: CurveStringIntersectionReport2,
    trim_report: Option<CurveStringTrimReport2>,
    query_path: CurveStringCurveTrimQueryPath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringCurveTrimResult2 {
    let stage = if trim_report.is_some() {
        CurveStringCurveTrimStage2::RangeMaterialization
    } else {
        CurveStringCurveTrimStage2::HitSelection
    };
    CurveStringCurveTrimResult2 {
        curve_string: None,
        report: CurveStringCurveTrimReport2 {
            start_hits,
            end_hits,
            start_intersection_report,
            end_intersection_report,
            trim_report,
            query_path,
            stage,
            status,
            blocker,
        },
    }
}

fn blocked_extend_result(
    curve_string: &CurveString2,
    endpoint: CurveStringEndpoint2,
    source_segment_index: usize,
    target_point: Point2,
    source_param: Option<Real>,
    predicate_path: CurveStringExtendPredicatePath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringExtendResult2 {
    let source_segment = &curve_string.segments()[source_segment_index];
    CurveStringExtendResult2 {
        curve_string: None,
        report: CurveStringExtendReport2 {
            stage: CurveStringExtendStage2::TargetValidation,
            predicate_path,
            endpoint,
            source_segment_index,
            source_segment_kind: source_segment.structural_facts().kind,
            output_segment_index: None,
            output_segment_kind: None,
            output_segment_start_point: None,
            output_segment_end_point: None,
            source_segment_start_point: source_segment.start().clone(),
            source_segment_end_point: source_segment.end().clone(),
            source_endpoint_point: curve_string
                .endpoint(endpoint)
                .expect("blocked extension source endpoint should exist")
                .clone(),
            target_point,
            source_param,
            source_segment_count: curve_string.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(curve_string),
            output_segment_count: None,
            output_segment_kind_counts: None,
            status,
            blocker,
        },
    }
}

fn line_endpoint_point(line: &LineSeg2, endpoint: CurveStringEndpoint2) -> &Point2 {
    match endpoint {
        CurveStringEndpoint2::Start => line.start(),
        CurveStringEndpoint2::End => line.end(),
    }
}

fn arc_endpoint_point(arc: &CircularArc2, endpoint: CurveStringEndpoint2) -> &Point2 {
    match endpoint {
        CurveStringEndpoint2::Start => arc.start(),
        CurveStringEndpoint2::End => arc.end(),
    }
}

fn blocked_chamfer_result(
    curve_string: &CurveString2,
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    predicate_path: CurveStringChamferPredicatePath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringChamferResult2 {
    CurveStringChamferResult2 {
        curve_string: None,
        report: CurveStringChamferReport2 {
            input_path: CurveStringChamferInputPath2::Parameters,
            stage: if segment_reports.is_empty() {
                CurveStringChamferStage2::InputValidation
            } else {
                CurveStringChamferStage2::SegmentMaterialization
            },
            predicate_path,
            previous_segment_index,
            next_segment_index,
            previous_segment_start_point: curve_string.segments[previous_segment_index]
                .start()
                .clone(),
            previous_segment_end_point: curve_string.segments[previous_segment_index].end().clone(),
            next_segment_start_point: curve_string.segments[next_segment_index].start().clone(),
            next_segment_end_point: curve_string.segments[next_segment_index].end().clone(),
            previous_trim,
            next_trim,
            previous_cut_point: None,
            next_cut_point: None,
            segment_reports,
            chamfer_segment_index: None,
            chamfer_segment_kind: None,
            chamfer_segment_start_point: None,
            chamfer_segment_end_point: None,
            source_segment_count: curve_string.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(curve_string),
            output_segment_count: None,
            output_segment_kind_counts: None,
            status,
            blocker,
        },
    }
}

fn blocked_fillet_result(
    curve_string: &CurveString2,
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    center: Option<Point2>,
    radius_squared: Option<Real>,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    predicate_path: CurveStringFilletPredicatePath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringFilletResult2 {
    CurveStringFilletResult2 {
        curve_string: None,
        report: CurveStringFilletReport2 {
            input_path: CurveStringFilletInputPath2::Points,
            stage: if radius_squared.is_some() {
                CurveStringFilletStage2::RadiusAndTangencyValidation
            } else {
                CurveStringFilletStage2::InputValidation
            },
            predicate_path,
            previous_segment_index,
            next_segment_index,
            previous_segment_start_point: curve_string.segments[previous_segment_index]
                .start()
                .clone(),
            previous_segment_end_point: curve_string.segments[previous_segment_index].end().clone(),
            next_segment_start_point: curve_string.segments[next_segment_index].start().clone(),
            next_segment_end_point: curve_string.segments[next_segment_index].end().clone(),
            previous_trim,
            next_trim,
            previous_tangent_point: None,
            next_tangent_point: None,
            center,
            radius_squared,
            segment_reports,
            fillet_segment_index: None,
            fillet_segment_kind: None,
            fillet_segment_start_point: None,
            fillet_segment_end_point: None,
            source_segment_count: curve_string.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(curve_string),
            output_segment_count: None,
            output_segment_kind_counts: None,
            status,
            blocker,
        },
    }
}

pub(crate) fn merge_adjacent_line_segments(
    current: &Segment2,
    next: &Segment2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<LineSeg2>>> {
    let (Segment2::Line(current), Segment2::Line(next)) = (current, next) else {
        return Ok(Classification::Decided(None));
    };

    match current.classify_point(next.end(), policy) {
        Classification::Decided(LineSide::On) => {}
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let (current_dx, current_dy) = current.delta();
    let (next_dx, next_dy) = next.delta();
    let dot = (&current_dx * &next_dx) + (&current_dy * &next_dy);
    match real_sign(&dot, policy) {
        Some(RealSign::Positive) => Ok(Classification::Decided(Some(LineSeg2::try_new(
            current.start().clone(),
            next.end().clone(),
        )?))),
        Some(RealSign::Zero | RealSign::Negative) => Ok(Classification::Decided(None)),
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn validate_trim_point(
    curve_string: &CurveString2,
    point: &CurveStringTrimPoint2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    if point.segment_index >= curve_string.len() {
        return Err(CurveError::InvalidCurveRange);
    }
    match in_closed_unit_interval(&point.param, policy) {
        Some(true) => Ok(()),
        Some(false) => Err(CurveError::InvalidCurveParameter),
        None => Ok(()),
    }
}

fn compare_trim_points(
    start: &CurveStringTrimPoint2,
    end: &CurveStringTrimPoint2,
    policy: &CurvePolicy,
) -> Option<Ordering> {
    match start.segment_index.cmp(&end.segment_index) {
        Ordering::Less => Some(Ordering::Less),
        Ordering::Greater => Some(Ordering::Greater),
        Ordering::Equal => compare_reals(&start.param, &end.param, policy),
    }
}

fn trim_segment_by_range(
    source_segment: &Segment2,
    source_range: &ParamRange,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    let ordering = match compare_reals(source_range.start(), source_range.end(), policy) {
        Some(ordering) => ordering,
        None => {
            return Ok(SegmentTrimMaterialization::Unresolved(
                UncertaintyReason::Ordering,
            ));
        }
    };
    match ordering {
        Ordering::Greater => return Err(CurveError::InvalidCurveRange),
        Ordering::Equal => return Ok(SegmentTrimMaterialization::SkippedEmpty),
        Ordering::Less => {}
    }

    let is_full_range = trim_range_is_full(source_range, policy);
    match is_full_range {
        Some(true) => Ok(SegmentTrimMaterialization::Materialized(
            source_segment.clone(),
        )),
        Some(false) => match source_segment {
            Segment2::Line(line) => trim_line_segment_by_range(line, source_range),
            Segment2::Arc(_) => Ok(SegmentTrimMaterialization::Unsupported(
                UncertaintyReason::Unsupported,
            )),
        },
        None => Ok(SegmentTrimMaterialization::Unresolved(
            UncertaintyReason::Ordering,
        )),
    }
}

fn trim_line_segment_by_range(
    line: &LineSeg2,
    source_range: &ParamRange,
) -> CurveResult<SegmentTrimMaterialization> {
    let start = line.point_at(source_range.start().clone());
    let end = line.point_at(source_range.end().clone());
    LineSeg2::try_new(start, end)
        .map(Segment2::Line)
        .map(SegmentTrimMaterialization::Materialized)
}

fn trim_segment_by_point_range(
    source_segment: &Segment2,
    source_range: &ParamRange,
    start_point: &Point2,
    end_point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    let ordering = match compare_reals(source_range.start(), source_range.end(), policy) {
        Some(ordering) => ordering,
        None => {
            return Ok(SegmentTrimMaterialization::Unresolved(
                UncertaintyReason::Ordering,
            ));
        }
    };
    match ordering {
        Ordering::Greater => return Err(CurveError::InvalidCurveRange),
        Ordering::Equal => return Ok(SegmentTrimMaterialization::SkippedEmpty),
        Ordering::Less => {}
    }

    match source_segment {
        Segment2::Line(_) => LineSeg2::try_new(start_point.clone(), end_point.clone())
            .map(Segment2::Line)
            .map(SegmentTrimMaterialization::Materialized),
        Segment2::Arc(arc) => trim_arc_segment_by_point_range(arc, start_point, end_point, policy),
    }
}

fn trim_arc_segment_by_point_range(
    source_arc: &CircularArc2,
    start_point: &Point2,
    end_point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    match (
        source_arc.contains_point(start_point, policy),
        source_arc.contains_point(end_point, policy),
    ) {
        (Classification::Decided(true), Classification::Decided(true)) => {}
        (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {
            return Err(CurveError::InvalidCurveRange);
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            return Ok(SegmentTrimMaterialization::Unresolved(reason));
        }
    }

    let distance = start_point.distance_squared(end_point);
    match is_zero(&distance, policy) {
        Some(true) => Ok(SegmentTrimMaterialization::SkippedEmpty),
        Some(false) => Ok(SegmentTrimMaterialization::Materialized(Segment2::Arc(
            CircularArc2::new_unchecked_with_radius(
                start_point.clone(),
                end_point.clone(),
                source_arc.center().clone(),
                source_arc.radius_squared(),
                source_arc.is_clockwise(),
                None,
            ),
        ))),
        None => Ok(SegmentTrimMaterialization::Unresolved(
            UncertaintyReason::RealSign,
        )),
    }
}

fn trim_range_is_full(range: &ParamRange, policy: &CurvePolicy) -> Option<bool> {
    let start_is_zero = compare_reals(range.start(), &Real::zero(), policy)? == Ordering::Equal;
    let end_is_one = compare_reals(range.end(), &Real::one(), policy)? == Ordering::Equal;
    Some(start_is_zero && end_is_one)
}

fn curve_string_segment_kind_counts(curve_string: &CurveString2) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in curve_string.segments() {
        add_segment_kind_count(&mut counts, segment);
    }
    counts
}

fn segment_kind_counts_for_range(
    segments: &[Segment2],
    range: std::ops::Range<usize>,
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in &segments[range] {
        add_segment_kind_count(&mut counts, segment);
    }
    counts
}

fn curve_strings_segment_kind_counts(curve_strings: &[CurveString2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for curve_string in curve_strings {
        add_segment_kind_counts(&mut counts, curve_string_segment_kind_counts(curve_string));
    }
    counts
}

fn contours_segment_kind_counts(contours: &[crate::Contour2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for contour in contours {
        for segment in contour.segments() {
            add_segment_kind_count(&mut counts, segment);
        }
    }
    counts
}

fn contour_refs_segment_kind_counts(contours: &[&crate::Contour2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for contour in contours {
        for segment in contour.segments() {
            add_segment_kind_count(&mut counts, segment);
        }
    }
    counts
}

fn add_segment_kind_counts(counts: &mut SegmentKindCounts, addend: SegmentKindCounts) {
    counts.lines += addend.lines;
    counts.arcs += addend.arcs;
}

fn add_segment_kind_count(counts: &mut SegmentKindCounts, segment: &Segment2) {
    match segment {
        Segment2::Line(_) => counts.lines += 1,
        Segment2::Arc(_) => counts.arcs += 1,
    }
}

fn blocked_trim_result(
    curve_string: &CurveString2,
    start: CurveStringTrimPoint2,
    end: CurveStringTrimPoint2,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    input_path: CurveStringTrimInputPath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringTrimResult2 {
    CurveStringTrimResult2 {
        curve_string: None,
        report: CurveStringTrimReport2 {
            input_path,
            predicate_path: trim_predicate_path(input_path),
            start,
            end,
            source_segment_count: curve_string.len(),
            source_segment_kind_counts: curve_string_segment_kind_counts(curve_string),
            segment_reports,
            output_segment_count: None,
            output_segment_kind_counts: None,
            status,
            blocker,
        },
    }
}

const fn trim_predicate_path(
    input_path: CurveStringTrimInputPath2,
) -> CurveStringTrimPredicatePath2 {
    match input_path {
        CurveStringTrimInputPath2::Parameters => CurveStringTrimPredicatePath2::ExactParameterRange,
        CurveStringTrimInputPath2::Points => CurveStringTrimPredicatePath2::ExactLocatedPointRange,
    }
}

fn locate_trim_point(
    curve_string: &CurveString2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LocatedTrimPoint2>> {
    let mut located = Vec::new();
    for (segment_index, segment) in curve_string.segments().iter().enumerate() {
        match segment.contains_point(point, policy) {
            Classification::Decided(true) => {
                let param = match segment_point_parameter(segment, point, policy)? {
                    Classification::Decided(param) => param,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                located.push(LocatedTrimPoint2 {
                    trim_point: CurveStringTrimPoint2::new(segment_index, param),
                    point: point.clone(),
                });
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    match canonical_located_trim_point(located, policy) {
        Some(point) => Ok(Classification::Decided(point)),
        None => Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
    }
}

fn canonical_located_trim_point(
    mut located: Vec<LocatedTrimPoint2>,
    policy: &CurvePolicy,
) -> Option<LocatedTrimPoint2> {
    match located.len() {
        0 => None,
        1 => located.pop(),
        _ => {
            located.sort_by(|left, right| {
                left.trim_point
                    .segment_index
                    .cmp(&right.trim_point.segment_index)
                    .then_with(|| {
                        compare_reals(&left.trim_point.param, &right.trim_point.param, policy)
                            .unwrap_or(Ordering::Equal)
                    })
            });
            if located
                .windows(2)
                .all(|window| adjacent_vertex_duplicate(&window[0], &window[1], policy))
            {
                located.pop()
            } else {
                None
            }
        }
    }
}

fn adjacent_vertex_duplicate(
    left: &LocatedTrimPoint2,
    right: &LocatedTrimPoint2,
    policy: &CurvePolicy,
) -> bool {
    if left.trim_point.segment_index + 1 != right.trim_point.segment_index {
        return false;
    }
    let left_at_end = compare_reals(&left.trim_point.param, &Real::one(), policy);
    let right_at_start = compare_reals(&right.trim_point.param, &Real::zero(), policy);
    left_at_end == Some(Ordering::Equal) && right_at_start == Some(Ordering::Equal)
}

fn segment_point_parameter(
    segment: &Segment2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    match segment {
        Segment2::Line(line) => line_point_parameter(line, point, policy),
        Segment2::Arc(arc) => arc_chord_parameter(arc, point),
    }
}

fn line_point_parameter(
    line: &LineSeg2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());
    match is_zero(&dx, policy) {
        Some(false) => (delta.0 / dx)
            .map(Classification::Decided)
            .map_err(Into::into),
        Some(true) => (delta.1 / dy)
            .map(Classification::Decided)
            .map_err(Into::into),
        None => match is_zero(&dy, policy) {
            Some(false) => (delta.1 / dy)
                .map(Classification::Decided)
                .map_err(Into::into),
            Some(true) => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        },
    }
}

fn line_chamfer_point_parameter(
    line: &LineSeg2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    match line.contains_point(point, policy) {
        Classification::Decided(true) => line_point_parameter(line, point, policy),
        Classification::Decided(false) => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn line_line_fillet_validation_blocker(
    line: &LineSeg2,
    tangent_point: &Point2,
    center: &Point2,
    clockwise: bool,
    policy: &CurvePolicy,
) -> Option<UncertaintyReason> {
    let (line_dx, line_dy) = line.delta();
    let (radius_dx, radius_dy) = tangent_point.delta_from(center);
    let perpendicular = (&line_dx * &radius_dx) + (&line_dy * &radius_dy);
    match is_zero(&perpendicular, policy) {
        Some(true) => {}
        Some(false) => return Some(UncertaintyReason::Boundary),
        None => return Some(UncertaintyReason::RealSign),
    }

    let (tangent_dx, tangent_dy) = if clockwise {
        (radius_dy, -radius_dx)
    } else {
        (-radius_dy, radius_dx)
    };
    let direction_dot = (&line_dx * &tangent_dx) + (&line_dy * &tangent_dy);
    match real_sign(&direction_dot, policy) {
        Some(RealSign::Positive) => None,
        Some(RealSign::Zero | RealSign::Negative) => Some(UncertaintyReason::Boundary),
        None => Some(UncertaintyReason::RealSign),
    }
}

fn retained_status_for_uncertainty(reason: UncertaintyReason) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn arc_chord_parameter(arc: &CircularArc2, point: &Point2) -> CurveResult<Classification<Real>> {
    let (dx, dy) = arc.end().delta_from(arc.start());
    let (px, py) = point.delta_from(arc.start());
    let numerator = (&px * &dx) + (&py * &dy);
    let denominator = (&dx * &dx) + (&dy * &dy);
    (numerator / denominator)
        .map(Classification::Decided)
        .map_err(Into::into)
}

impl CurveStringIntersection {
    /// Returns the segment index in the first curve string.
    pub const fn a_segment_index(&self) -> usize {
        self.a_segment_index
    }

    /// Returns the segment index in the second curve string.
    pub const fn b_segment_index(&self) -> usize {
        self.b_segment_index
    }

    /// Returns the primitive family of the first source segment.
    pub const fn a_segment_kind(&self) -> SegmentKind {
        self.a_segment_kind
    }

    /// Returns the primitive family of the second source segment.
    pub const fn b_segment_kind(&self) -> SegmentKind {
        self.b_segment_kind
    }

    /// Returns the exact start point of the first source segment.
    pub const fn a_segment_start_point(&self) -> &Point2 {
        &self.a_segment_start_point
    }

    /// Returns the exact end point of the first source segment.
    pub const fn a_segment_end_point(&self) -> &Point2 {
        &self.a_segment_end_point
    }

    /// Returns the exact start point of the second source segment.
    pub const fn b_segment_start_point(&self) -> &Point2 {
        &self.b_segment_start_point
    }

    /// Returns the exact end point of the second source segment.
    pub const fn b_segment_end_point(&self) -> &Point2 {
        &self.b_segment_end_point
    }

    /// Returns the exact segment relation retained for this pair.
    pub const fn relation(&self) -> &SegmentIntersection {
        &self.relation
    }
}

impl CurveStringEndpointConnectionReport2 {
    /// Returns the tested endpoint on the first curve string.
    pub const fn first_endpoint(&self) -> CurveStringEndpoint2 {
        self.first_endpoint
    }

    /// Returns the tested endpoint on the second curve string.
    pub const fn second_endpoint(&self) -> CurveStringEndpoint2 {
        self.second_endpoint
    }

    /// Returns the exact point witness on the first curve string endpoint.
    pub const fn first_point(&self) -> &Point2 {
        &self.first_point
    }

    /// Returns the exact point witness on the second curve string endpoint.
    pub const fn second_point(&self) -> &Point2 {
        &self.second_point
    }

    /// Returns exact squared endpoint distance evidence.
    pub const fn distance_squared(&self) -> &crate::Real {
        &self.distance_squared
    }

    /// Returns the exact predicate path used for this endpoint decision.
    pub const fn predicate_path(&self) -> CurveStringEndpointConnectionPredicatePath2 {
        self.predicate_path
    }

    /// Returns the exact connectivity status for this endpoint pair.
    pub const fn status(&self) -> CurveStringEndpointConnectionStatus2 {
        self.status
    }

    /// Returns the retained-topology status corresponding to this endpoint test.
    pub const fn topology_status(&self) -> RetainedTopologyStatus {
        match self.status {
            CurveStringEndpointConnectionStatus2::NativeExact => {
                RetainedTopologyStatus::NativeExact
            }
            CurveStringEndpointConnectionStatus2::Disconnected => {
                RetainedTopologyStatus::Unsupported
            }
            CurveStringEndpointConnectionStatus2::Unresolved(_) => {
                RetainedTopologyStatus::Unresolved
            }
        }
    }
}

impl CurveStringLinkReport2 {
    /// Returns the furthest exact link stage reached.
    pub const fn stage(&self) -> CurveStringLinkStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to select this endpoint link.
    pub const fn predicate_path(&self) -> CurveStringLinkPredicatePath2 {
        self.predicate_path
    }

    /// Returns the selected link orientation.
    pub const fn kind(&self) -> CurveStringLinkKind2 {
        self.kind
    }

    /// Returns endpoint-pair evidence for the selected link.
    pub const fn endpoint_report(&self) -> &CurveStringEndpointConnectionReport2 {
        &self.endpoint_report
    }

    /// Returns the first input segment count captured by this report.
    pub const fn first_segment_count(&self) -> usize {
        self.first_segment_count
    }

    /// Returns primitive-family counts for the first input curve string.
    pub const fn first_segment_kind_counts(&self) -> SegmentKindCounts {
        self.first_segment_kind_counts
    }

    /// Returns the second input segment count captured by this report.
    pub const fn second_segment_count(&self) -> usize {
        self.second_segment_count
    }

    /// Returns primitive-family counts for the second input curve string.
    pub const fn second_segment_kind_counts(&self) -> SegmentKindCounts {
        self.second_segment_kind_counts
    }

    /// Returns endpoint pairs inspected before choosing this link.
    pub const fn endpoint_pair_count(&self) -> usize {
        self.endpoint_pair_count
    }

    /// Returns inspected endpoint pairs certified already connected.
    pub const fn exact_endpoint_pair_count(&self) -> usize {
        self.exact_endpoint_pair_count
    }

    /// Returns inspected endpoint pairs certified disconnected.
    pub const fn disconnected_endpoint_pair_count(&self) -> usize {
        self.disconnected_endpoint_pair_count
    }

    /// Returns inspected endpoint pairs that remained unresolved.
    pub const fn unresolved_endpoint_pair_count(&self) -> usize {
        self.unresolved_endpoint_pair_count
    }

    /// Returns the output segment count for the materialized link.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the linked output, when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns output segment source ownership in output order.
    pub fn output_segments(&self) -> &[CurveStringLinkOutputSegmentReport2] {
        &self.output_segments
    }

    /// Returns the topology status of the materialized link.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringLinkOutputSegmentReport2 {
    /// Returns the linked output segment index described by this report.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns which input curve string contributed this output segment.
    pub const fn source_input(&self) -> CurveStringLinkSourceInput2 {
        self.source_input
    }

    /// Returns the contributing segment index within its source input.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the contributing source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the primitive family of the emitted output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns whether the source segment was reversed for output.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the exact start point of the contributing source segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the contributing source segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the exact start point of this emitted output segment.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact end point of this emitted output segment.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }
}

impl LinkedCurveString2 {
    /// Returns the linked curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve_string
    }

    /// Consumes this result and returns the linked curve string.
    pub fn into_curve_string(self) -> CurveString2 {
        self.curve_string
    }

    /// Returns the provenance report for this link.
    pub const fn report(&self) -> &CurveStringLinkReport2 {
        &self.report
    }
}

impl CurveStringLinkAttemptReport2 {
    /// Returns the furthest exact link-attempt stage reached.
    pub const fn stage(&self) -> CurveStringLinkStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to select or block this endpoint link attempt.
    pub const fn predicate_path(&self) -> CurveStringLinkPredicatePath2 {
        self.predicate_path
    }

    /// Returns the selected link orientation, when exactly one endpoint pair matched.
    pub const fn selected_kind(&self) -> Option<CurveStringLinkKind2> {
        self.selected_kind
    }

    /// Returns selected endpoint evidence, when exactly one endpoint pair matched.
    pub const fn selected_endpoint_report(&self) -> Option<&CurveStringEndpointConnectionReport2> {
        self.selected_endpoint_report.as_ref()
    }

    /// Returns the first input segment count captured by this report.
    pub const fn first_segment_count(&self) -> usize {
        self.first_segment_count
    }

    /// Returns primitive-family counts for the first input curve string.
    pub const fn first_segment_kind_counts(&self) -> SegmentKindCounts {
        self.first_segment_kind_counts
    }

    /// Returns the second input segment count captured by this report.
    pub const fn second_segment_count(&self) -> usize {
        self.second_segment_count
    }

    /// Returns primitive-family counts for the second input curve string.
    pub const fn second_segment_kind_counts(&self) -> SegmentKindCounts {
        self.second_segment_kind_counts
    }

    /// Returns endpoint pairs inspected before choosing or blocking a link.
    pub const fn endpoint_pair_count(&self) -> usize {
        self.endpoint_pair_count
    }

    /// Returns inspected endpoint pairs certified already connected.
    pub const fn exact_endpoint_pair_count(&self) -> usize {
        self.exact_endpoint_pair_count
    }

    /// Returns inspected endpoint pairs certified disconnected.
    pub const fn disconnected_endpoint_pair_count(&self) -> usize {
        self.disconnected_endpoint_pair_count
    }

    /// Returns inspected endpoint pairs that remained unresolved.
    pub const fn unresolved_endpoint_pair_count(&self) -> usize {
        self.unresolved_endpoint_pair_count
    }

    /// Returns the output segment count when the link attempt materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the linked output, when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns output segment source ownership when a link materialized.
    pub fn output_segments(&self) -> &[CurveStringLinkOutputSegmentReport2] {
        &self.output_segments
    }

    /// Returns topology status for this link attempt.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for a non-materialized link attempt.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringLinkAttemptResult2 {
    /// Returns the materialized linked curve string, if exactly one endpoint pair matched.
    pub const fn linked_curve_string(&self) -> Option<&LinkedCurveString2> {
        self.linked.as_ref()
    }

    /// Consumes this result and returns the materialized linked curve string, if any.
    pub fn into_linked_curve_string(self) -> Option<LinkedCurveString2> {
        self.linked
    }

    /// Consumes this result and returns retained evidence for the pairwise link attempt.
    pub fn into_report(self) -> CurveStringLinkAttemptReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized link with its report.
    pub fn into_parts(self) -> (Option<LinkedCurveString2>, CurveStringLinkAttemptReport2) {
        (self.linked, self.report)
    }

    /// Returns retained evidence for the pairwise link attempt.
    pub const fn report(&self) -> &CurveStringLinkAttemptReport2 {
        &self.report
    }

    /// Returns the link attempt as a convenience classification while retaining this result.
    pub fn linked_curve_string_classification(
        &self,
    ) -> Classification<Option<&LinkedCurveString2>> {
        if let Some(linked) = self.linked_curve_string() {
            return Classification::Decided(Some(linked));
        }
        if self.report().exact_endpoint_pair_count() == 0
            && self.report().unresolved_endpoint_pair_count() == 0
        {
            return Classification::Decided(None);
        }
        Classification::Uncertain(
            self.report()
                .blocker()
                .unwrap_or(UncertaintyReason::Unsupported),
        )
    }

    /// Consumes this result and returns the link attempt as a convenience classification.
    pub fn into_linked_curve_string_classification(
        self,
    ) -> Classification<Option<LinkedCurveString2>> {
        let disconnected = self.report().exact_endpoint_pair_count() == 0
            && self.report().unresolved_endpoint_pair_count() == 0;
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_linked_curve_string() {
            Some(linked) => Classification::Decided(Some(linked)),
            None if disconnected => Classification::Decided(None),
            None => Classification::Uncertain(blocker),
        }
    }
}

impl CurveStringIntersectionReport2 {
    pub(crate) const fn new_native_exact(
        first_segment_count: usize,
        second_segment_count: usize,
        first_segment_kind_counts: SegmentKindCounts,
        second_segment_kind_counts: SegmentKindCounts,
        first_decided_segment_box_count: usize,
        second_decided_segment_box_count: usize,
        first_undecided_segment_box_count: usize,
        second_undecided_segment_box_count: usize,
        candidate_pair_count: usize,
        skipped_aabb_pair_count: usize,
        tested_pair_count: usize,
        intersection_count: usize,
        point_relation_count: usize,
        overlap_relation_count: usize,
        uncertain_relation_count: usize,
        query_path: CurveStringIntersectionQueryPath2,
        prepared_cache_report: Option<CurveStringIntersectionPreparedCacheReport2>,
    ) -> Self {
        Self {
            first_segment_count,
            second_segment_count,
            first_segment_kind_counts,
            second_segment_kind_counts,
            first_decided_segment_box_count,
            second_decided_segment_box_count,
            first_undecided_segment_box_count,
            second_undecided_segment_box_count,
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            intersection_count,
            point_relation_count,
            overlap_relation_count,
            uncertain_relation_count,
            query_path,
            predicate_path: intersection_predicate_path(candidate_pair_count, tested_pair_count),
            prepared_cache_report,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        }
    }

    /// Returns the first curve-string segment count.
    pub const fn first_segment_count(&self) -> usize {
        self.first_segment_count
    }

    /// Returns the second curve-string segment count.
    pub const fn second_segment_count(&self) -> usize {
        self.second_segment_count
    }

    /// Returns primitive-family counts for the first curve string.
    pub const fn first_segment_kind_counts(&self) -> SegmentKindCounts {
        self.first_segment_kind_counts
    }

    /// Returns primitive-family counts for the second curve string.
    pub const fn second_segment_kind_counts(&self) -> SegmentKindCounts {
        self.second_segment_kind_counts
    }

    /// Returns decided segment boxes available for the first curve string.
    pub const fn first_decided_segment_box_count(&self) -> usize {
        self.first_decided_segment_box_count
    }

    /// Returns decided segment boxes available for the second curve string.
    pub const fn second_decided_segment_box_count(&self) -> usize {
        self.second_decided_segment_box_count
    }

    /// Returns first-curve segments whose boxes stayed undecided.
    pub const fn first_undecided_segment_box_count(&self) -> usize {
        self.first_undecided_segment_box_count
    }

    /// Returns second-curve segments whose boxes stayed undecided.
    pub const fn second_undecided_segment_box_count(&self) -> usize {
        self.second_undecided_segment_box_count
    }

    /// Returns the total flat segment-pair candidates before broad-phase skips.
    pub const fn candidate_pair_count(&self) -> usize {
        self.candidate_pair_count
    }

    /// Returns segment pairs skipped by decided AABB disjointness.
    pub const fn skipped_aabb_pair_count(&self) -> usize {
        self.skipped_aabb_pair_count
    }

    /// Returns segment pairs tested by exact segment topology.
    pub const fn tested_pair_count(&self) -> usize {
        self.tested_pair_count
    }

    /// Returns nonempty segment-pair intersections collected.
    pub const fn intersection_count(&self) -> usize {
        self.intersection_count
    }

    /// Returns nonempty segment-pair relations whose exact topology is point-like.
    pub const fn point_relation_count(&self) -> usize {
        self.point_relation_count
    }

    /// Returns nonempty segment-pair relations whose exact topology is overlapping.
    pub const fn overlap_relation_count(&self) -> usize {
        self.overlap_relation_count
    }

    /// Returns nonempty segment-pair relations left unresolved by the active policy.
    pub const fn uncertain_relation_count(&self) -> usize {
        self.uncertain_relation_count
    }

    /// Returns the query path used to collect intersections.
    pub const fn query_path(&self) -> CurveStringIntersectionQueryPath2 {
        self.query_path
    }

    /// Returns the predicate/filter path reached by this intersection query.
    pub const fn predicate_path(&self) -> CurveStringIntersectionPredicatePath2 {
        self.predicate_path
    }

    /// Returns prepared-cache inventory and freshness evidence, when used.
    pub const fn prepared_cache_report(
        &self,
    ) -> Option<&CurveStringIntersectionPreparedCacheReport2> {
        self.prepared_cache_report.as_ref()
    }

    /// Returns intersection collection status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized intersection collection.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringIntersectionPreparedCacheReport2 {
    /// Builds prepared-cache evidence from per-operand audits.
    pub(crate) const fn new(
        first: CurveStringPreparedCacheAudit2,
        second: CurveStringPreparedCacheAudit2,
    ) -> Self {
        Self { first, second }
    }

    /// Returns prepared-cache evidence for the first curve string.
    pub const fn first(&self) -> &CurveStringPreparedCacheAudit2 {
        &self.first
    }

    /// Returns prepared-cache evidence for the second curve string.
    pub const fn second(&self) -> &CurveStringPreparedCacheAudit2 {
        &self.second
    }
}

impl CurveStringPreparedCacheAudit2 {
    /// Builds per-curve-string prepared cache evidence.
    pub(crate) const fn new(
        prepared_segment_count: usize,
        prepared_segment_kind_counts: SegmentKindCounts,
        decided_segment_box_count: usize,
        undecided_segment_box_count: usize,
        curve_box_decided: bool,
    ) -> Self {
        Self {
            freshness: CurveStringPreparedCacheFreshness2::BorrowedCurrentSource,
            prepared_segment_count,
            prepared_segment_kind_counts,
            decided_segment_box_count,
            undecided_segment_box_count,
            curve_box_decided,
        }
    }

    /// Returns the cache freshness claim for this borrowed prepared view.
    pub const fn freshness(&self) -> CurveStringPreparedCacheFreshness2 {
        self.freshness
    }

    /// Returns the number of prepared source segments.
    pub const fn prepared_segment_count(&self) -> usize {
        self.prepared_segment_count
    }

    /// Returns primitive-family counts for prepared source segments.
    pub const fn prepared_segment_kind_counts(&self) -> SegmentKindCounts {
        self.prepared_segment_kind_counts
    }

    /// Returns the number of decided segment AABBs retained by preparation.
    pub const fn decided_segment_box_count(&self) -> usize {
        self.decided_segment_box_count
    }

    /// Returns the number of source segment AABBs that remained undecided.
    pub const fn undecided_segment_box_count(&self) -> usize {
        self.undecided_segment_box_count
    }

    /// Returns whether preparation retained a decided whole-curve AABB.
    pub const fn curve_box_decided(&self) -> bool {
        self.curve_box_decided
    }
}

impl CurveStringIntersectionResult2 {
    pub(crate) fn from_parts(
        intersections: Vec<CurveStringIntersection>,
        report: CurveStringIntersectionReport2,
    ) -> Self {
        Self {
            intersections,
            report,
        }
    }

    /// Returns collected segment-pair intersections.
    pub fn intersections(&self) -> &[CurveStringIntersection] {
        &self.intersections
    }

    /// Consumes this result and returns collected intersections.
    pub fn into_intersections(self) -> Vec<CurveStringIntersection> {
        self.intersections
    }

    /// Returns retained scan evidence for this intersection query.
    pub const fn report(&self) -> &CurveStringIntersectionReport2 {
        &self.report
    }
}

impl CurveStringOrderedLinkStepReport2 {
    /// Returns source curve-string indices already accumulated before this step.
    pub fn accumulated_source_indices(&self) -> &[usize] {
        &self.accumulated_source_indices
    }

    /// Returns the next source curve-string index consumed by this step.
    pub const fn next_source_index(&self) -> usize {
        self.next_source_index
    }

    /// Returns pairwise endpoint-link attempt evidence for this step.
    pub const fn link_attempt_report(&self) -> Option<&CurveStringLinkAttemptReport2> {
        self.link_attempt_report.as_ref()
    }

    /// Returns the pairwise link report when this step materialized.
    pub const fn link_report(&self) -> Option<&CurveStringLinkReport2> {
        self.link_report.as_ref()
    }

    /// Returns topology status for this ordered link step.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized link steps.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringOrderedLinkReport2 {
    /// Returns the furthest exact ordered-link stage reached.
    pub const fn stage(&self) -> CurveStringOrderedLinkStage2 {
        self.stage
    }

    /// Returns the exact predicate path used while linking ordered steps.
    pub const fn predicate_path(&self) -> CurveStringOrderedLinkPredicatePath2 {
        self.predicate_path
    }

    /// Returns the source curve-string count captured by this report.
    pub const fn source_curve_string_count(&self) -> usize {
        self.source_curve_string_count
    }

    /// Returns primitive-family counts across all source curve strings.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns ordered pairwise link steps attempted.
    pub const fn attempted_link_step_count(&self) -> usize {
        self.attempted_link_step_count
    }

    /// Returns pairwise link steps that materialized native exact topology.
    pub const fn materialized_link_step_count(&self) -> usize {
        self.materialized_link_step_count
    }

    /// Returns pairwise link steps that blocked ordered-chain materialization.
    pub const fn blocked_link_step_count(&self) -> usize {
        self.blocked_link_step_count
    }

    /// Returns the output segment count when ordered linking materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the ordered linked output, when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns source curve-string indices in final output order.
    ///
    /// For blocked reports this is the accumulated output order before the
    /// failing step.
    pub fn output_source_indices(&self) -> &[usize] {
        &self.output_source_indices
    }

    /// Returns ordered link steps and their pairwise evidence.
    pub fn steps(&self) -> &[CurveStringOrderedLinkStepReport2] {
        &self.steps
    }

    /// Returns ordered-link materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for a non-materialized ordered link.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl OrderedLinkedCurveString2 {
    /// Returns the materialized ordered linked curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the linked curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained ordered-link report.
    pub const fn report(&self) -> &CurveStringOrderedLinkReport2 {
        &self.report
    }
}

impl CurveStringConnectReport2 {
    /// Returns the furthest exact connector stage reached.
    pub const fn stage(&self) -> CurveStringConnectStage2 {
        self.stage
    }

    /// Returns the exact predicate path used to select this connector.
    pub const fn predicate_path(&self) -> CurveStringConnectPredicatePath2 {
        self.predicate_path
    }

    /// Returns the selected connector orientation, when one was selected.
    pub const fn kind(&self) -> Option<CurveStringLinkKind2> {
        self.kind
    }

    /// Returns endpoint equality evidence for the connector endpoints.
    pub const fn endpoint_report(&self) -> &CurveStringEndpointConnectionReport2 {
        &self.endpoint_report
    }

    /// Returns every endpoint-pair report inspected for this connector decision.
    pub fn endpoint_reports(&self) -> &[CurveStringEndpointConnectionReport2] {
        &self.endpoint_reports
    }

    /// Returns the first input segment count captured by this report.
    pub const fn first_segment_count(&self) -> usize {
        self.first_segment_count
    }

    /// Returns primitive-family counts for the first input curve string.
    pub const fn first_segment_kind_counts(&self) -> SegmentKindCounts {
        self.first_segment_kind_counts
    }

    /// Returns the second input segment count captured by this report.
    pub const fn second_segment_count(&self) -> usize {
        self.second_segment_count
    }

    /// Returns primitive-family counts for the second input curve string.
    pub const fn second_segment_kind_counts(&self) -> SegmentKindCounts {
        self.second_segment_kind_counts
    }

    /// Returns endpoint pairs inspected before connector selection or blocking.
    pub const fn endpoint_pair_count(&self) -> usize {
        self.endpoint_pair_count
    }

    /// Returns inspected endpoint pairs certified already connected.
    pub const fn exact_endpoint_pair_count(&self) -> usize {
        self.exact_endpoint_pair_count
    }

    /// Returns inspected endpoint pairs certified disconnected.
    pub const fn disconnected_endpoint_pair_count(&self) -> usize {
        self.disconnected_endpoint_pair_count
    }

    /// Returns inspected endpoint pairs that remained unresolved.
    pub const fn unresolved_endpoint_pair_count(&self) -> usize {
        self.unresolved_endpoint_pair_count
    }

    /// Returns the inserted connector segment index in the output curve string.
    pub const fn connector_segment_index(&self) -> Option<usize> {
        self.connector_segment_index
    }

    /// Returns the exact start point of the inserted connector segment.
    pub const fn connector_start_point(&self) -> Option<&Point2> {
        self.connector_start_point.as_ref()
    }

    /// Returns the exact end point of the inserted connector segment.
    pub const fn connector_end_point(&self) -> Option<&Point2> {
        self.connector_end_point.as_ref()
    }

    /// Returns the output segment count when a connector was materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for the connected output, when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns output segment source ownership in output order.
    pub fn output_segments(&self) -> &[CurveStringConnectOutputSegmentReport2] {
        &self.output_segments
    }

    /// Returns connector materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized connections.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringConnectOutputSegmentReport2 {
    /// Returns the connected output segment index described by this report.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns which source produced this output segment.
    pub const fn source(&self) -> CurveStringConnectSource2 {
        self.source
    }

    /// Returns the source segment index for input-derived segments.
    pub const fn source_segment_index(&self) -> Option<usize> {
        self.source_segment_index
    }

    /// Returns the primitive family of the contributing source segment, if any.
    pub const fn source_segment_kind(&self) -> Option<SegmentKind> {
        self.source_segment_kind
    }

    /// Returns the primitive family of the emitted output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns whether an input segment was reversed for output.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the exact start point of the contributing source segment, if any.
    pub const fn source_segment_start_point(&self) -> Option<&Point2> {
        self.source_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the contributing source segment, if any.
    pub const fn source_segment_end_point(&self) -> Option<&Point2> {
        self.source_segment_end_point.as_ref()
    }

    /// Returns the exact start point of this emitted output segment.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact end point of this emitted output segment.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }
}

impl ConnectedCurveString2 {
    /// Returns the connected curve string, if a connector was materialized.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the connected curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained connector report.
    pub const fn report(&self) -> &CurveStringConnectReport2 {
        &self.report
    }
}

impl CurveStringLineMergeSpanReport2 {
    /// Returns the first source segment index included in this output segment.
    pub const fn source_start_segment_index(&self) -> usize {
        self.source_start_segment_index
    }

    /// Returns the final source segment index included in this output segment.
    pub const fn source_end_segment_index(&self) -> usize {
        self.source_end_segment_index
    }

    /// Returns all source segment indices included in this output segment.
    pub fn source_segment_indices(&self) -> &[usize] {
        &self.source_segment_indices
    }

    /// Returns primitive-family counts for the retained source segment run.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the output segment index produced for this source run.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the primitive family of the emitted output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns the exact start point of this emitted output segment.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact end point of this emitted output segment.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained topology status for this source run.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringLineMergeReport2 {
    /// Returns the furthest exact line-merge stage reached.
    pub const fn stage(&self) -> CurveStringLineMergeStage2 {
        self.stage
    }

    /// Returns the exact predicate path used while classifying adjacent pairs.
    pub const fn predicate_path(&self) -> CurveStringLineMergePredicatePath2 {
        self.predicate_path
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the primitive-family inventory captured before merging.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns adjacent source segment pairs classified during this merge.
    pub const fn adjacent_pair_count(&self) -> usize {
        self.adjacent_pair_count
    }

    /// Returns adjacent pairs certified as one same-direction line run.
    pub const fn merged_pair_count(&self) -> usize {
        self.merged_pair_count
    }

    /// Returns adjacent pairs certified to remain distinct topology.
    pub const fn preserved_pair_count(&self) -> usize {
        self.preserved_pair_count
    }

    /// Returns the output segment count when the merge materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns the primitive-family inventory after merging, when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns retained source runs for materialized output segments.
    pub fn spans(&self) -> &[CurveStringLineMergeSpanReport2] {
        &self.spans
    }

    /// Returns merge materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized merge attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringLineMergeResult2 {
    /// Returns the materialized merged curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized merged curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Consumes this result and returns retained line-merge evidence.
    pub fn into_report(self) -> CurveStringLineMergeReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized curve string with its report.
    pub fn into_parts(self) -> (Option<CurveString2>, CurveStringLineMergeReport2) {
        (self.curve_string, self.report)
    }

    /// Returns the retained line-merge report.
    pub const fn report(&self) -> &CurveStringLineMergeReport2 {
        &self.report
    }

    /// Returns the merge output as a convenience classification while retaining this result.
    pub fn curve_string_classification(&self) -> Classification<&CurveString2> {
        match self.curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the merge output as a convenience classification.
    pub fn into_curve_string_classification(self) -> Classification<CurveString2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(blocker),
        }
    }
}

impl CurveStringReversedDuplicatePairReport2 {
    /// Returns the first source segment index removed by this cancellation.
    pub const fn first_source_segment_index(&self) -> usize {
        self.first_source_segment_index
    }

    /// Returns the second source segment index removed by this cancellation.
    pub const fn second_source_segment_index(&self) -> usize {
        self.second_source_segment_index
    }

    /// Returns the exact predicate path used to remove this duplicate pair.
    pub const fn predicate_path(&self) -> CurveStringDeduplicatePredicatePath2 {
        self.predicate_path
    }

    /// Returns the primitive family of the first removed source segment.
    pub const fn first_source_segment_kind(&self) -> SegmentKind {
        self.first_source_segment_kind
    }

    /// Returns the primitive family of the second removed source segment.
    pub const fn second_source_segment_kind(&self) -> SegmentKind {
        self.second_source_segment_kind
    }

    /// Returns the exact start point of the first removed segment.
    pub const fn first_start_point(&self) -> &Point2 {
        &self.first_start_point
    }

    /// Returns the exact end point of the first removed segment.
    pub const fn first_end_point(&self) -> &Point2 {
        &self.first_end_point
    }

    /// Returns the exact start point of the second removed segment.
    pub const fn second_start_point(&self) -> &Point2 {
        &self.second_start_point
    }

    /// Returns the exact end point of the second removed segment.
    pub const fn second_end_point(&self) -> &Point2 {
        &self.second_end_point
    }

    /// Returns retained topology status for this duplicate-pair removal.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringDeduplicateRetainedSegmentReport2 {
    /// Returns the output segment index emitted for this retained source segment.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the retained source segment index.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the retained source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the primitive family of the emitted output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns the exact start point of the retained source segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the retained source segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the exact output start point for this retained segment.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact output end point for this retained segment.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained topology status for this emitted segment.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringDeduplicateReport2 {
    /// Returns the furthest exact de-duplication stage reached.
    pub const fn stage(&self) -> CurveStringDeduplicateStage2 {
        self.stage
    }

    /// Returns the exact predicate path used while cancelling adjacent pairs.
    pub const fn predicate_path(&self) -> CurveStringDeduplicatePredicatePath2 {
        self.predicate_path
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the primitive-family inventory captured before de-duplication.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the output segment count when de-duplication materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns the primitive-family inventory after de-duplication, when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns source segment indices retained in output order.
    pub fn retained_source_segment_indices(&self) -> &[usize] {
        &self.retained_source_segment_indices
    }

    /// Returns retained output segment evidence in output order.
    pub fn retained_segments(&self) -> &[CurveStringDeduplicateRetainedSegmentReport2] {
        &self.retained_segments
    }

    /// Returns exact reversed duplicate pairs removed by this operation.
    pub fn removed_pairs(&self) -> &[CurveStringReversedDuplicatePairReport2] {
        &self.removed_pairs
    }

    /// Returns de-duplication materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized de-duplication attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringDeduplicateResult2 {
    /// Returns the materialized de-duplicated curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Consumes this result and returns retained de-duplication evidence.
    pub fn into_report(self) -> CurveStringDeduplicateReport2 {
        self.report
    }

    /// Consumes this result and returns the materialized curve string with its report.
    pub fn into_parts(self) -> (Option<CurveString2>, CurveStringDeduplicateReport2) {
        (self.curve_string, self.report)
    }

    /// Returns retained de-duplication evidence.
    pub const fn report(&self) -> &CurveStringDeduplicateReport2 {
        &self.report
    }

    /// Returns the de-duplicated output as a convenience classification while retaining this result.
    pub fn curve_string_classification(&self) -> Classification<&CurveString2> {
        match self.curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns the de-duplicated output as a convenience classification.
    pub fn into_curve_string_classification(self) -> Classification<CurveString2> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_curve_string() {
            Some(curve_string) => Classification::Decided(curve_string),
            None => Classification::Uncertain(blocker),
        }
    }
}

fn linked_curve_string(
    first: &CurveString2,
    second: &CurveString2,
    kind: CurveStringLinkKind2,
) -> CurveResult<CurveString2> {
    let mut segments = Vec::with_capacity(first.len() + second.len());
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            segments.extend(first.segments().iter().cloned());
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            segments.extend(first.segments().iter().cloned());
            segments.extend(reversed_segments(second.segments()));
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            segments.extend(reversed_segments(first.segments()));
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            segments.extend(second.segments().iter().cloned());
            segments.extend(first.segments().iter().cloned());
        }
    }

    CurveString2::try_new(segments)
}

fn link_output_segment_reports(
    first: &CurveString2,
    second: &CurveString2,
    kind: CurveStringLinkKind2,
) -> Vec<CurveStringLinkOutputSegmentReport2> {
    let first_segment_count = first.len();
    let second_segment_count = second.len();
    let mut output_segments = Vec::with_capacity(first_segment_count + second_segment_count);
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            push_link_output_segment_reports(
                &mut output_segments,
                first,
                CurveStringLinkSourceInput2::First,
                0..first_segment_count,
                false,
            );
            push_link_output_segment_reports(
                &mut output_segments,
                second,
                CurveStringLinkSourceInput2::Second,
                0..second_segment_count,
                false,
            );
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            push_link_output_segment_reports(
                &mut output_segments,
                first,
                CurveStringLinkSourceInput2::First,
                0..first_segment_count,
                false,
            );
            push_link_output_segment_reports(
                &mut output_segments,
                second,
                CurveStringLinkSourceInput2::Second,
                (0..second_segment_count).rev(),
                true,
            );
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            push_link_output_segment_reports(
                &mut output_segments,
                first,
                CurveStringLinkSourceInput2::First,
                (0..first_segment_count).rev(),
                true,
            );
            push_link_output_segment_reports(
                &mut output_segments,
                second,
                CurveStringLinkSourceInput2::Second,
                0..second_segment_count,
                false,
            );
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            push_link_output_segment_reports(
                &mut output_segments,
                second,
                CurveStringLinkSourceInput2::Second,
                0..second_segment_count,
                false,
            );
            push_link_output_segment_reports(
                &mut output_segments,
                first,
                CurveStringLinkSourceInput2::First,
                0..first_segment_count,
                false,
            );
        }
    }
    output_segments
}

fn push_link_output_segment_reports<I>(
    output_segments: &mut Vec<CurveStringLinkOutputSegmentReport2>,
    source_curve_string: &CurveString2,
    source_input: CurveStringLinkSourceInput2,
    source_segment_indices: I,
    reversed: bool,
) where
    I: IntoIterator<Item = usize>,
{
    for source_segment_index in source_segment_indices {
        let source_segment = &source_curve_string.segments()[source_segment_index];
        let (output_start_point, output_end_point) = if reversed {
            (source_segment.end().clone(), source_segment.start().clone())
        } else {
            (source_segment.start().clone(), source_segment.end().clone())
        };
        output_segments.push(CurveStringLinkOutputSegmentReport2 {
            output_segment_index: output_segments.len(),
            source_input,
            source_segment_index,
            source_segment_kind: source_segment.structural_facts().kind,
            output_segment_kind: source_segment.structural_facts().kind,
            reversed,
            source_segment_start_point: source_segment.start().clone(),
            source_segment_end_point: source_segment.end().clone(),
            output_start_point,
            output_end_point,
        });
    }
}

fn connect_output_segment_reports(
    first: &CurveString2,
    second: &CurveString2,
    kind: CurveStringLinkKind2,
) -> CurveResult<Vec<CurveStringConnectOutputSegmentReport2>> {
    let first_segment_count = first.len();
    let second_segment_count = second.len();
    let mut output_segments = Vec::with_capacity(first_segment_count + second_segment_count + 1);
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            push_connect_input_segment_reports(
                &mut output_segments,
                first,
                CurveStringConnectSource2::First,
                0..first_segment_count,
                false,
            );
            push_connect_connector_report(
                &mut output_segments,
                first.end().ok_or(CurveError::EmptyCurveString)?,
                second.start().ok_or(CurveError::EmptyCurveString)?,
            );
            push_connect_input_segment_reports(
                &mut output_segments,
                second,
                CurveStringConnectSource2::Second,
                0..second_segment_count,
                false,
            );
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            push_connect_input_segment_reports(
                &mut output_segments,
                first,
                CurveStringConnectSource2::First,
                0..first_segment_count,
                false,
            );
            push_connect_connector_report(
                &mut output_segments,
                first.end().ok_or(CurveError::EmptyCurveString)?,
                second.end().ok_or(CurveError::EmptyCurveString)?,
            );
            push_connect_input_segment_reports(
                &mut output_segments,
                second,
                CurveStringConnectSource2::Second,
                (0..second_segment_count).rev(),
                true,
            );
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            push_connect_input_segment_reports(
                &mut output_segments,
                first,
                CurveStringConnectSource2::First,
                (0..first_segment_count).rev(),
                true,
            );
            push_connect_connector_report(
                &mut output_segments,
                first.start().ok_or(CurveError::EmptyCurveString)?,
                second.start().ok_or(CurveError::EmptyCurveString)?,
            );
            push_connect_input_segment_reports(
                &mut output_segments,
                second,
                CurveStringConnectSource2::Second,
                0..second_segment_count,
                false,
            );
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            push_connect_input_segment_reports(
                &mut output_segments,
                second,
                CurveStringConnectSource2::Second,
                0..second_segment_count,
                false,
            );
            push_connect_connector_report(
                &mut output_segments,
                second.end().ok_or(CurveError::EmptyCurveString)?,
                first.start().ok_or(CurveError::EmptyCurveString)?,
            );
            push_connect_input_segment_reports(
                &mut output_segments,
                first,
                CurveStringConnectSource2::First,
                0..first_segment_count,
                false,
            );
        }
    }
    Ok(output_segments)
}

fn push_connect_input_segment_reports<I>(
    output_segments: &mut Vec<CurveStringConnectOutputSegmentReport2>,
    source_curve_string: &CurveString2,
    source: CurveStringConnectSource2,
    source_segment_indices: I,
    reversed: bool,
) where
    I: IntoIterator<Item = usize>,
{
    for source_segment_index in source_segment_indices {
        let source_segment = &source_curve_string.segments()[source_segment_index];
        let (output_start_point, output_end_point) = if reversed {
            (source_segment.end().clone(), source_segment.start().clone())
        } else {
            (source_segment.start().clone(), source_segment.end().clone())
        };
        output_segments.push(CurveStringConnectOutputSegmentReport2 {
            output_segment_index: output_segments.len(),
            source,
            source_segment_index: Some(source_segment_index),
            source_segment_kind: Some(source_segment.structural_facts().kind),
            output_segment_kind: source_segment.structural_facts().kind,
            reversed,
            source_segment_start_point: Some(source_segment.start().clone()),
            source_segment_end_point: Some(source_segment.end().clone()),
            output_start_point,
            output_end_point,
        });
    }
}

fn push_connect_connector_report(
    output_segments: &mut Vec<CurveStringConnectOutputSegmentReport2>,
    output_start_point: &Point2,
    output_end_point: &Point2,
) {
    output_segments.push(CurveStringConnectOutputSegmentReport2 {
        output_segment_index: output_segments.len(),
        source: CurveStringConnectSource2::Connector,
        source_segment_index: None,
        source_segment_kind: None,
        output_segment_kind: SegmentKind::Line,
        reversed: false,
        source_segment_start_point: None,
        source_segment_end_point: None,
        output_start_point: output_start_point.clone(),
        output_end_point: output_end_point.clone(),
    });
}

fn connector_endpoint_points(
    output_segments: &[CurveStringConnectOutputSegmentReport2],
) -> CurveResult<(Point2, Point2)> {
    let connector = output_segments
        .iter()
        .find(|segment| segment.source == CurveStringConnectSource2::Connector)
        .ok_or(CurveError::InvalidCurveRange)?;
    Ok((
        connector.output_start_point.clone(),
        connector.output_end_point.clone(),
    ))
}

fn ordered_link_source_indices(
    accumulated_source_indices: &[usize],
    next_source_index: usize,
    kind: CurveStringLinkKind2,
) -> Vec<usize> {
    let mut source_indices = Vec::with_capacity(accumulated_source_indices.len() + 1);
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart | CurveStringLinkKind2::FirstEndToSecondEnd => {
            source_indices.extend_from_slice(accumulated_source_indices);
            source_indices.push(next_source_index);
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            source_indices.extend(accumulated_source_indices.iter().rev().copied());
            source_indices.push(next_source_index);
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            source_indices.push(next_source_index);
            source_indices.extend_from_slice(accumulated_source_indices);
        }
    }
    source_indices
}

fn blocked_connected_curve_string(
    first: &CurveString2,
    second: &CurveString2,
    kind: Option<CurveStringLinkKind2>,
    endpoint_report: CurveStringEndpointConnectionReport2,
    endpoint_reports: Vec<CurveStringEndpointConnectionReport2>,
    endpoint_summary: EndpointPairSummary,
    predicate_path: CurveStringConnectPredicatePath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> ConnectedCurveString2 {
    ConnectedCurveString2 {
        curve_string: None,
        report: CurveStringConnectReport2 {
            stage: CurveStringConnectStage2::EndpointSelection,
            predicate_path,
            kind,
            endpoint_report,
            endpoint_reports,
            first_segment_count: first.len(),
            first_segment_kind_counts: curve_string_segment_kind_counts(first),
            second_segment_count: second.len(),
            second_segment_kind_counts: curve_string_segment_kind_counts(second),
            endpoint_pair_count: endpoint_summary.pair_count,
            exact_endpoint_pair_count: endpoint_summary.exact_count,
            disconnected_endpoint_pair_count: endpoint_summary.disconnected_count,
            unresolved_endpoint_pair_count: endpoint_summary.unresolved_count,
            connector_segment_index: None,
            connector_start_point: None,
            connector_end_point: None,
            output_segment_count: None,
            output_segment_kind_counts: None,
            output_segments: Vec::new(),
            status,
            blocker,
        },
    }
}

fn connected_curve_string(
    first: &CurveString2,
    second: &CurveString2,
    kind: CurveStringLinkKind2,
) -> CurveResult<(CurveString2, usize)> {
    let mut segments = Vec::with_capacity(first.len() + 1 + second.len());
    let (connector_start, connector_end, connector_segment_index) = match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            segments.extend(first.segments().iter().cloned());
            let connector_segment_index = segments.len();
            let connector_start = first.end().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = second.start().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            segments.extend(first.segments().iter().cloned());
            let connector_segment_index = segments.len();
            let connector_start = first.end().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = second.end().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            segments.extend(reversed_segments(first.segments()));
            let connector_segment_index = segments.len();
            let connector_start = first.start().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = second.start().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            segments.extend(second.segments().iter().cloned());
            let connector_segment_index = segments.len();
            let connector_start = second.end().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = first.start().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
    };

    segments.push(Segment2::Line(LineSeg2::try_new(
        connector_start,
        connector_end,
    )?));
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            segments.extend(reversed_segments(second.segments()));
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            segments.extend(first.segments().iter().cloned());
        }
    }

    CurveString2::try_new(segments).map(|curve_string| (curve_string, connector_segment_index))
}

fn unique_nearest_endpoint_report(
    candidates: Vec<(CurveStringLinkKind2, CurveStringEndpointConnectionReport2)>,
    policy: &CurvePolicy,
) -> NearestEndpointChoice {
    let mut best: Option<(CurveStringLinkKind2, CurveStringEndpointConnectionReport2)> = None;
    for candidate in candidates {
        let Some((_, best_report)) = best.as_ref() else {
            best = Some(candidate);
            continue;
        };
        match compare_reals(
            candidate.1.distance_squared(),
            best_report.distance_squared(),
            policy,
        ) {
            Some(Ordering::Less) => best = Some(candidate),
            Some(Ordering::Greater) => {}
            Some(Ordering::Equal) => {
                return NearestEndpointChoice::Ambiguous(candidate.0, candidate.1);
            }
            None => {
                return NearestEndpointChoice::Unresolved(
                    candidate.0,
                    candidate.1,
                    UncertaintyReason::Ordering,
                );
            }
        }
    }

    match best {
        Some((kind, report)) => NearestEndpointChoice::Selected(kind, report),
        None => NearestEndpointChoice::Empty,
    }
}

fn reversed_segments(segments: &[Segment2]) -> Vec<Segment2> {
    segments
        .iter()
        .rev()
        .map(Segment2::reversed)
        .collect::<Vec<_>>()
}

pub(crate) fn intersect_curve_strings_with_cached_aabbs_with_report(
    first: &CurveString2,
    second: &CurveString2,
    first_segment_boxes: &[Option<Aabb2>],
    second_segment_boxes: &[Option<Aabb2>],
    query_path: CurveStringIntersectionQueryPath2,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringIntersectionResult2> {
    let mut intersections = Vec::new();
    let candidate_pair_count = first.segments.len() * second.segments.len();
    let first_decided_segment_box_count = decided_segment_box_count(first_segment_boxes);
    let second_decided_segment_box_count = decided_segment_box_count(second_segment_boxes);
    let first_undecided_segment_box_count =
        first.len().saturating_sub(first_decided_segment_box_count);
    let second_undecided_segment_box_count = second
        .len()
        .saturating_sub(second_decided_segment_box_count);
    let mut skipped_aabb_pair_count = 0_usize;
    let mut tested_pair_count = 0_usize;

    for (a_segment_index, a_segment) in first.segments.iter().enumerate() {
        for (b_segment_index, b_segment) in second.segments.iter().enumerate() {
            // This is the same conservative broad-phase used by the public
            // curve-string query. Bentley and Ottmann, "Algorithms for
            // Reporting and Counting Geometric Intersections" (1979), use
            // ordered sweep candidates for asymptotically better pruning; this
            // helper keeps the flat pair scan but lets prepared callers reuse
            // segment boxes across repeated queries.
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                first_segment_boxes.get(a_segment_index),
                second_segment_boxes.get(b_segment_index),
            ) && aabbs_decided_disjoint(a_box, b_box, policy)
            {
                skipped_aabb_pair_count += 1;
                continue;
            }

            tested_pair_count += 1;
            let relation = a_segment.intersect_segment(b_segment, policy)?;
            if !relation.is_none() {
                intersections.push(CurveStringIntersection {
                    a_segment_index,
                    b_segment_index,
                    a_segment_kind: a_segment.structural_facts().kind,
                    b_segment_kind: b_segment.structural_facts().kind,
                    a_segment_start_point: a_segment.start().clone(),
                    a_segment_end_point: a_segment.end().clone(),
                    b_segment_start_point: b_segment.start().clone(),
                    b_segment_end_point: b_segment.end().clone(),
                    relation,
                });
            }
        }
    }

    let intersection_count = intersections.len();
    let relation_counts = curve_string_intersection_relation_counts(&intersections);
    Ok(CurveStringIntersectionResult2 {
        intersections,
        report: CurveStringIntersectionReport2 {
            first_segment_count: first.len(),
            second_segment_count: second.len(),
            first_segment_kind_counts: curve_string_segment_kind_counts(first),
            second_segment_kind_counts: curve_string_segment_kind_counts(second),
            first_decided_segment_box_count,
            second_decided_segment_box_count,
            first_undecided_segment_box_count,
            second_undecided_segment_box_count,
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            intersection_count,
            point_relation_count: relation_counts.point,
            overlap_relation_count: relation_counts.overlap,
            uncertain_relation_count: relation_counts.uncertain,
            query_path,
            predicate_path: intersection_predicate_path(candidate_pair_count, tested_pair_count),
            prepared_cache_report: None,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    })
}

pub(crate) fn decided_segment_box_count(segment_boxes: &[Option<Aabb2>]) -> usize {
    segment_boxes.iter().filter(|bbox| bbox.is_some()).count()
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct CurveStringIntersectionRelationCounts {
    pub(crate) point: usize,
    pub(crate) overlap: usize,
    pub(crate) uncertain: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CurveStringIntersectionRelationClass {
    None,
    Point,
    Overlap,
    Uncertain,
}

pub(crate) fn curve_string_intersection_relation_counts(
    intersections: &[CurveStringIntersection],
) -> CurveStringIntersectionRelationCounts {
    let mut counts = CurveStringIntersectionRelationCounts::default();
    for intersection in intersections {
        match curve_string_intersection_relation_class(&intersection.relation) {
            CurveStringIntersectionRelationClass::None => {}
            CurveStringIntersectionRelationClass::Point => counts.point += 1,
            CurveStringIntersectionRelationClass::Overlap => counts.overlap += 1,
            CurveStringIntersectionRelationClass::Uncertain => counts.uncertain += 1,
        }
    }
    counts
}

fn curve_string_intersection_relation_class(
    relation: &SegmentIntersection,
) -> CurveStringIntersectionRelationClass {
    match relation {
        SegmentIntersection::LineLine(result) => match result {
            LineLineIntersection::None => CurveStringIntersectionRelationClass::None,
            LineLineIntersection::Point { .. } => CurveStringIntersectionRelationClass::Point,
            LineLineIntersection::Overlap { .. } => CurveStringIntersectionRelationClass::Overlap,
            LineLineIntersection::Uncertain { .. } => {
                CurveStringIntersectionRelationClass::Uncertain
            }
        },
        SegmentIntersection::LineArc { result, .. } => match result {
            LineArcIntersection::None => CurveStringIntersectionRelationClass::None,
            LineArcIntersection::Point(_) | LineArcIntersection::TwoPoints { .. } => {
                CurveStringIntersectionRelationClass::Point
            }
            LineArcIntersection::Uncertain { .. } => {
                CurveStringIntersectionRelationClass::Uncertain
            }
        },
        SegmentIntersection::ArcArc(result) => match result {
            ArcArcIntersection::None => CurveStringIntersectionRelationClass::None,
            ArcArcIntersection::Point(_) | ArcArcIntersection::TwoPoints { .. } => {
                CurveStringIntersectionRelationClass::Point
            }
            ArcArcIntersection::Overlap { .. } => CurveStringIntersectionRelationClass::Overlap,
            ArcArcIntersection::Uncertain { .. } => CurveStringIntersectionRelationClass::Uncertain,
        },
    }
}

const fn intersection_predicate_path(
    candidate_pair_count: usize,
    tested_pair_count: usize,
) -> CurveStringIntersectionPredicatePath2 {
    if candidate_pair_count == 0 {
        CurveStringIntersectionPredicatePath2::NoCandidates
    } else if tested_pair_count == 0 {
        CurveStringIntersectionPredicatePath2::AabbOnly
    } else {
        CurveStringIntersectionPredicatePath2::ExactSegmentPredicates
    }
}
