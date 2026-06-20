//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed boundary contours into the signed contour
//! bins used by [`crate::Region2`]. It assumes intersections and overlaps have
//! already been resolved by earlier topology stages.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::bbox::{Aabb2, aabbs_decided_disjoint};
use crate::classify::compare_reals;
use crate::{
    ArcArcIntersection, CircularArc2, Classification, Contour2, ContourPointLocation, CurveError,
    CurvePolicy, CurveResult, FillRule, LineArcIntersection, LineArcOrder, LineLineIntersection,
    LineSeg2, ParamRange, Point2, Region2, RetainedTopologyStatus, Segment2, SegmentIntersection,
    SegmentKind, SegmentKindCounts, UncertaintyReason,
};

/// Canonical retained arrangement request for exact curve topology.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementRequest2 {
    source_segments: Vec<Segment2>,
    source_line_segments: Option<Vec<LineSeg2>>,
    fill_rule: FillRule,
}

/// Retained workspace for a single exact curve arrangement attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveWorkspace2 {
    request: ExactCurveArrangementRequest2,
    source_segment_kind_counts: SegmentKindCounts,
    source_segment_aabbs: Vec<Option<Aabb2>>,
    source_aabb: Option<Aabb2>,
    source_segment_cache: ExactCurveArrangementSourceSegmentCache2,
    source_endpoint_bucket_cache: ExactCurveArrangementSourceEndpointBucketCache2,
    split_schedule_cache: ExactCurveArrangementSplitScheduleCache2,
    split_cache: Option<ExactCurveArrangementSplitCache2>,
    endpoint_graph_cache: Option<ExactCurveArrangementEndpointGraphCache2>,
    ring_assembly_cache: Option<ExactCurveArrangementRingAssemblyCache2>,
    output_cache: Option<ExactCurveArrangementOutputCache2>,
}

/// Source segment fact retained during workspace preparation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceSegmentFact2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_start_point: Point2,
    source_end_point: Point2,
    source_aabb: Option<Aabb2>,
}

/// AABB certification status retained for one source segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactCurveArrangementSourceAabbStatus2 {
    /// The source segment box was certified during workspace preparation.
    Decided,
    /// The source segment box stayed uncertain during workspace preparation.
    Undecided,
}

/// Reference to a retained source segment AABB fact.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceAabbRef2 {
    source_segment_index: usize,
}

/// Source segment bucket grouped by retained AABB certification status.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceAabbBucket2 {
    aabb_status: ExactCurveArrangementSourceAabbStatus2,
    source_refs: Vec<ExactCurveArrangementSourceAabbRef2>,
}

/// Source segment AABB buckets retained during workspace preparation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceAabbBucketCache2 {
    bucket_count: usize,
    source_ref_count: usize,
    decided_source_ref_count: usize,
    undecided_source_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSourceAabbBucket2>,
}

/// Reference to a retained source segment fact inside a primitive-family bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceSegmentKindRef2 {
    source_segment_index: usize,
}

/// Source segment bucket grouped by retained primitive family.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceSegmentKindBucket2 {
    source_segment_kind: SegmentKind,
    source_refs: Vec<ExactCurveArrangementSourceSegmentKindRef2>,
}

/// Source segment primitive-family buckets retained during workspace preparation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceSegmentKindBucketCache2 {
    bucket_count: usize,
    source_segment_ref_count: usize,
    line_segment_ref_count: usize,
    arc_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSourceSegmentKindBucket2>,
}

/// Source segment fact cache retained during workspace preparation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceSegmentCache2 {
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    decided_source_segment_aabb_count: usize,
    undecided_source_segment_aabb_count: usize,
    source_aabb: Option<Aabb2>,
    source_aabb_bucket_cache: ExactCurveArrangementSourceAabbBucketCache2,
    source_segment_kind_bucket_cache: ExactCurveArrangementSourceSegmentKindBucketCache2,
    segments: Vec<ExactCurveArrangementSourceSegmentFact2>,
}

/// Source endpoint of a retained exact arrangement input segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactCurveArrangementSourceEndpoint2 {
    /// Start point of the source segment.
    Start,
    /// End point of the source segment.
    End,
}

/// Source-segment endpoint reference retained in a source endpoint bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceEndpointRef2 {
    source_segment_index: usize,
    endpoint: ExactCurveArrangementSourceEndpoint2,
}

/// Exact structural source endpoint bucket retained during workspace preparation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceEndpointBucket2 {
    point: Point2,
    endpoints: Vec<ExactCurveArrangementSourceEndpointRef2>,
}

/// Exact structural source endpoint buckets retained during workspace preparation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSourceEndpointBucketCache2 {
    endpoint_count: usize,
    bucket_count: usize,
    singleton_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSourceEndpointBucket2>,
}

/// AABB pruning status retained for one scheduled source split candidate pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactCurveArrangementSplitCandidateAabbStatus2 {
    /// The source boxes were both decided and certified disjoint.
    DecidedDisjoint,
    /// The source boxes were both decided and not certified disjoint.
    NotDecidedDisjoint,
    /// One or both source boxes were not certified during workspace preparation.
    Undecided,
}

/// Source segment pair scheduled for exact split predicate evaluation or AABB pruning.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitCandidatePair2 {
    first_source_segment_index: usize,
    second_source_segment_index: usize,
    aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2,
}

/// Reference to a retained scheduled split candidate pair.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitScheduleRef2 {
    candidate_pair_index: usize,
}

/// Scheduled split candidate bucket grouped by retained AABB pruning status.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitScheduleBucket2 {
    aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2,
    candidate_refs: Vec<ExactCurveArrangementSplitScheduleRef2>,
}

/// Scheduled split candidate buckets grouped by retained AABB pruning status.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitScheduleBucketCache2 {
    bucket_count: usize,
    candidate_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSplitScheduleBucket2>,
}

/// Retained exact source-pair schedule used before split predicate evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitScheduleCache2 {
    candidate_pair_count: usize,
    decided_disjoint_pair_count: usize,
    predicate_candidate_pair_count: usize,
    undecided_aabb_pair_count: usize,
    bucket_cache: ExactCurveArrangementSplitScheduleBucketCache2,
    candidate_pairs: Vec<ExactCurveArrangementSplitCandidatePair2>,
}

/// Retained exact split evidence cached by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitCache2 {
    predicate_path: Option<RegionLineSegmentSplitPredicatePath2>,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    intersection_event_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    intersection_points: Vec<Point2>,
    intersection_reports: Vec<RegionLineSegmentSplitIntersectionReport2>,
    relation_bucket_cache: ExactCurveArrangementSplitRelationBucketCache2,
    intersection_bucket_cache: ExactCurveArrangementSplitIntersectionBucketCache2,
    intersection_parameter_cache: ExactCurveArrangementSplitIntersectionParameterCache2,
    blocker_cache: Option<ExactCurveArrangementSplitBlockerCache2>,
    output_segment_count: Option<usize>,
}

/// Retained source-pair blocker evidence from exact split arrangement.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitBlockerCache2 {
    first_source_segment_index: usize,
    first_source_segment_kind: SegmentKind,
    first_source_start_point: Point2,
    first_source_end_point: Point2,
    second_source_segment_index: usize,
    second_source_segment_kind: SegmentKind,
    second_source_start_point: Point2,
    second_source_end_point: Point2,
    blocker: Option<UncertaintyReason>,
}

/// Retained split-stage relation class.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactCurveArrangementSplitRelationClass2 {
    /// Source pair relation produced exact point-intersection evidence.
    Point,
    /// Source pair relation produced exact overlap evidence.
    Overlap,
    /// Source pair relation could not be decided by the configured exact predicates.
    Uncertain,
}

/// Retained split-stage relation bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitRelationBucket2 {
    relation: ExactCurveArrangementSplitRelationClass2,
    relation_count: usize,
}

/// Retained split-stage relation buckets.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitRelationBucketCache2 {
    bucket_count: usize,
    relation_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSplitRelationBucket2>,
}

/// Reference to a retained split-intersection report inside an exact point bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitIntersectionRef2 {
    intersection_report_index: usize,
}

/// Exact structural split-intersection bucket retained by an evaluated workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitIntersectionBucket2 {
    point: Point2,
    intersections: Vec<ExactCurveArrangementSplitIntersectionRef2>,
}

/// Exact structural split-intersection buckets retained by an evaluated workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitIntersectionBucketCache2 {
    intersection_event_count: usize,
    bucket_count: usize,
    singleton_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSplitIntersectionBucket2>,
}

/// Exact source-parameter evidence retained for one split intersection.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitIntersectionParameterRef2 {
    intersection_report_index: usize,
    first_source_segment_index: usize,
    first_source_param: Real,
    second_source_segment_index: usize,
    second_source_param: Real,
    point: Point2,
}

/// Exact source-parameter evidence retained for split intersections.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementSplitIntersectionParameterCache2 {
    intersection_event_count: usize,
    source_parameter_ref_count: usize,
    parameters: Vec<ExactCurveArrangementSplitIntersectionParameterRef2>,
}

/// Retained exact endpoint-bucket evidence cached by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementEndpointGraphCache2 {
    predicate_path: RegionLineSegmentEndpointGraphPredicatePath2,
    endpoint_count: usize,
    structural_bucket_count: usize,
    structural_singleton_bucket_count: usize,
    max_structural_bucket_size: usize,
    endpoint_bucket_cache: ExactCurveArrangementArrangedEndpointBucketCache2,
    endpoint_side_bucket_cache: ExactCurveArrangementArrangedEndpointSideBucketCache2,
    endpoint_point_cache: ExactCurveArrangementArrangedEndpointPointCache2,
    endpoint_degree_bucket_cache: ExactCurveArrangementArrangedEndpointDegreeBucketCache2,
    dangling_endpoint_count: usize,
    branch_endpoint_count: usize,
    blocker_arranged_segment_index: Option<usize>,
    blocker_endpoint: Option<RegionLineSegmentArrangedEndpoint2>,
    blocker_point: Option<Point2>,
}

/// Retained arranged endpoint structural degree.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactCurveArrangementArrangedEndpointDegree2 {
    /// One arranged endpoint occupies the structural point.
    Dangling,
    /// Two arranged endpoints occupy the structural point and form a chain connection.
    Chain,
    /// More than two arranged endpoints occupy the structural point.
    Branch,
}

/// Reference to a structural arranged endpoint bucket classified by degree.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointDegreeRef2 {
    structural_bucket_index: usize,
    endpoint_ref_count: usize,
    point: Point2,
}

/// Structural arranged endpoint buckets grouped by retained degree.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointDegreeBucket2 {
    degree: ExactCurveArrangementArrangedEndpointDegree2,
    endpoint_buckets: Vec<ExactCurveArrangementArrangedEndpointDegreeRef2>,
}

/// Arranged endpoint structural degree buckets retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointDegreeBucketCache2 {
    bucket_count: usize,
    structural_bucket_ref_count: usize,
    dangling_structural_bucket_count: usize,
    chain_structural_bucket_count: usize,
    branch_structural_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedEndpointDegreeBucket2>,
}

/// Arranged fragment endpoint reference retained in an exact endpoint bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointRef2 {
    arranged_segment_index: usize,
    endpoint: RegionLineSegmentArrangedEndpoint2,
}

/// Arranged endpoint bucket grouped by retained endpoint side.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointSideBucket2 {
    endpoint: RegionLineSegmentArrangedEndpoint2,
    endpoints: Vec<ExactCurveArrangementArrangedEndpointRef2>,
}

/// Arranged endpoint side buckets retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointSideBucketCache2 {
    bucket_count: usize,
    endpoint_ref_count: usize,
    start_endpoint_ref_count: usize,
    end_endpoint_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedEndpointSideBucket2>,
}

/// Exact structural arranged endpoint bucket retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointBucket2 {
    point: Point2,
    endpoints: Vec<ExactCurveArrangementArrangedEndpointRef2>,
}

/// Exact structural arranged endpoint buckets retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointBucketCache2 {
    endpoint_count: usize,
    bucket_count: usize,
    singleton_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedEndpointBucket2>,
}

/// Exact endpoints retained for one arranged fragment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointPointRef2 {
    arranged_segment_index: usize,
    output_start_point: Point2,
    output_end_point: Point2,
}

/// Exact arranged endpoint records retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedEndpointPointCache2 {
    arranged_fragment_ref_count: usize,
    endpoint_ref_count: usize,
    endpoints: Vec<ExactCurveArrangementArrangedEndpointPointRef2>,
}

/// Source provenance retained for one arranged fragment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentSourceRef2 {
    arranged_source_report_index: usize,
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_range: ParamRange,
    status: RetainedTopologyStatus,
}

/// Arranged fragment provenance retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragment2 {
    arranged_segment_index: usize,
    arranged_segment_kind: SegmentKind,
    output_start_point: Point2,
    output_end_point: Point2,
    source_refs: Vec<ExactCurveArrangementArrangedFragmentSourceRef2>,
}

/// Reference to a retained arranged fragment fact.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentRef2 {
    arranged_fragment_index: usize,
}

/// Reference to retained arranged fragment source evidence inside a status bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentStatusRef2 {
    arranged_fragment_index: usize,
    source_ref_index: usize,
    arranged_source_report_index: usize,
}

/// Arranged fragment bucket grouped by retained primitive family.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentKindBucket2 {
    arranged_segment_kind: SegmentKind,
    fragment_refs: Vec<ExactCurveArrangementArrangedFragmentRef2>,
}

/// Arranged fragment source-provenance bucket grouped by retained topology status.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentStatusBucket2 {
    status: RetainedTopologyStatus,
    source_refs: Vec<ExactCurveArrangementArrangedFragmentStatusRef2>,
}

/// Arranged fragment primitive-family buckets retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentKindBucketCache2 {
    bucket_count: usize,
    arranged_fragment_ref_count: usize,
    line_fragment_ref_count: usize,
    arc_fragment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedFragmentKindBucket2>,
}

/// Arranged fragment topology-status buckets retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentStatusBucketCache2 {
    bucket_count: usize,
    source_ref_count: usize,
    native_exact_ref_count: usize,
    certified_approximation_ref_count: usize,
    display_or_export_ref_count: usize,
    imported_lossy_ref_count: usize,
    unsupported_ref_count: usize,
    unresolved_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedFragmentStatusBucket2>,
}

/// Arranged fragment source-parameter range retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentSourceRangeRef2 {
    arranged_source_report_index: usize,
    source_segment_index: usize,
    source_range: ParamRange,
    arranged_segment_index: usize,
}

/// Arranged fragment source-parameter ranges retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentSourceRangeCache2 {
    source_ref_count: usize,
    full_source_range_ref_count: usize,
    partial_source_range_ref_count: usize,
    ranges: Vec<ExactCurveArrangementArrangedFragmentSourceRangeRef2>,
}

/// Arranged fragment provenance cache retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementArrangedFragmentCache2 {
    arranged_fragment_count: usize,
    source_ref_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    arranged_segment_kind_counts: SegmentKindCounts,
    arranged_fragment_kind_bucket_cache: ExactCurveArrangementArrangedFragmentKindBucketCache2,
    arranged_fragment_status_bucket_cache: ExactCurveArrangementArrangedFragmentStatusBucketCache2,
    arranged_fragment_source_range_cache: ExactCurveArrangementArrangedFragmentSourceRangeCache2,
    max_source_ref_count: usize,
    fragments: Vec<ExactCurveArrangementArrangedFragment2>,
}

/// Output segment provenance retained for one assembled ring bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRingSegmentRef2 {
    source_report_index: usize,
    output_segment_index: usize,
    reversed: bool,
}

/// Output ring bucket retained by exact ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRingBucket2 {
    output_ring_index: usize,
    segments: Vec<ExactCurveArrangementOutputRingSegmentRef2>,
}

/// Output ring buckets retained by exact ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRingBucketCache2 {
    ring_count: usize,
    segment_ref_count: usize,
    max_ring_segment_count: usize,
    rings: Vec<ExactCurveArrangementOutputRingBucket2>,
}

/// Output segment reference retained in a primitive-family bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentKindRef2 {
    source_report_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by retained primitive family.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentKindBucket2 {
    output_segment_kind: SegmentKind,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentKindRef2>,
}

/// Output segment primitive-family buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentKindBucketCache2 {
    bucket_count: usize,
    output_segment_ref_count: usize,
    line_segment_ref_count: usize,
    arc_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentKindBucket2>,
}

/// Output segment reference retained in a source-segment bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentSourceRef2 {
    source_report_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by retained source segment index.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentSourceBucket2 {
    source_segment_index: usize,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentSourceRef2>,
}

/// Output segment source-segment buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentSourceBucketCache2 {
    source_segment_bucket_count: usize,
    output_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentSourceBucket2>,
}

/// Output segment source-parameter range retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentSourceRangeRef2 {
    source_report_index: usize,
    source_segment_index: usize,
    source_range: ParamRange,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment source-parameter ranges retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentSourceRangeCache2 {
    output_segment_ref_count: usize,
    full_source_range_ref_count: usize,
    partial_source_range_ref_count: usize,
    ranges: Vec<ExactCurveArrangementOutputSegmentSourceRangeRef2>,
}

/// Output segment exact endpoints retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentEndpointRef2 {
    source_report_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
    output_start_point: Point2,
    output_end_point: Point2,
}

/// Output segment endpoint cache retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentEndpointCache2 {
    output_segment_ref_count: usize,
    output_endpoint_ref_count: usize,
    segments: Vec<ExactCurveArrangementOutputSegmentEndpointRef2>,
}

/// Exact endpoint continuity retained between adjacent output ring segments.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRingContinuityRef2 {
    source_report_index: usize,
    next_source_report_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
    next_output_segment_index: usize,
    output_end_point: Point2,
    next_output_start_point: Point2,
}

/// Output ring continuity cache retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRingContinuityCache2 {
    output_ring_ref_count: usize,
    output_connection_ref_count: usize,
    max_ring_connection_count: usize,
    connections: Vec<ExactCurveArrangementOutputRingContinuityRef2>,
}

/// Output segment reference retained in a topology-status bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentStatusRef2 {
    source_report_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by retained topology status.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentStatusBucket2 {
    status: RetainedTopologyStatus,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentStatusRef2>,
}

/// Output segment topology-status buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentStatusBucketCache2 {
    bucket_count: usize,
    output_segment_ref_count: usize,
    native_exact_ref_count: usize,
    certified_approximation_ref_count: usize,
    display_or_export_ref_count: usize,
    imported_lossy_ref_count: usize,
    unsupported_ref_count: usize,
    unresolved_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentStatusBucket2>,
}

/// Output segment reference retained in a traversal-direction bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentDirectionRef2 {
    source_report_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by whether ring traversal reversed the source segment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentDirectionBucket2 {
    reversed: bool,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentDirectionRef2>,
}

/// Output segment traversal-direction buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputSegmentDirectionBucketCache2 {
    bucket_count: usize,
    output_segment_ref_count: usize,
    forward_segment_ref_count: usize,
    reversed_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentDirectionBucket2>,
}

/// Output role assignment evidence retained for one boundary contour.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleAssignment2 {
    role_report_index: usize,
    source_contour_index: usize,
    source_segment_count: usize,
    source_fill_rule: FillRule,
    nesting_sample_point: Point2,
    containing_contour_indices: Vec<usize>,
    nesting_depth: usize,
    output_role_index: usize,
    status: RetainedTopologyStatus,
}

/// Reference to a retained output role assignment inside a status bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleStatusRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_report_index: usize,
}

/// Output role bucket retained after boundary contour role assignment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleBucket2 {
    role: RegionBoundaryContourRole2,
    assignments: Vec<ExactCurveArrangementOutputRoleAssignment2>,
}

/// Output role assignment bucket grouped by retained topology status.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleStatusBucket2 {
    status: RetainedTopologyStatus,
    assignments: Vec<ExactCurveArrangementOutputRoleStatusRef2>,
}

/// Output role assignment topology-status buckets retained after role assignment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleStatusBucketCache2 {
    bucket_count: usize,
    assignment_ref_count: usize,
    native_exact_ref_count: usize,
    certified_approximation_ref_count: usize,
    display_or_export_ref_count: usize,
    imported_lossy_ref_count: usize,
    unsupported_ref_count: usize,
    unresolved_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleStatusBucket2>,
}

/// Reference to a retained output role assignment inside a source-contour bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleSourceContourRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_report_index: usize,
    output_role_index: usize,
}

/// Output role assignment bucket grouped by retained source contour identity.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleSourceContourBucket2 {
    source_contour_index: usize,
    assignments: Vec<ExactCurveArrangementOutputRoleSourceContourRef2>,
}

/// Output role assignments grouped by retained source contour identity.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleSourceContourBucketCache2 {
    source_contour_bucket_count: usize,
    assignment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleSourceContourBucket2>,
}

/// Reference to a retained output role assignment inside a nesting-depth bucket.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleNestingDepthRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_report_index: usize,
    source_contour_index: usize,
    output_role_index: usize,
}

/// Output role assignment bucket grouped by exact nesting depth.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleNestingDepthBucket2 {
    nesting_depth: usize,
    assignments: Vec<ExactCurveArrangementOutputRoleNestingDepthRef2>,
}

/// Output role assignments grouped by retained exact nesting depth.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleNestingDepthBucketCache2 {
    nesting_depth_bucket_count: usize,
    assignment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleNestingDepthBucket2>,
}

/// Reference to retained containment evidence for one output role assignment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleContainmentRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_report_index: usize,
    source_contour_index: usize,
    containing_contour_index: usize,
    containing_contour_ref_index: usize,
    output_role_index: usize,
}

/// Output role containment bucket grouped by exact containing source contour.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleContainmentBucket2 {
    containing_contour_index: usize,
    containments: Vec<ExactCurveArrangementOutputRoleContainmentRef2>,
}

/// Output role containment evidence grouped by exact containing source contour.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleContainmentBucketCache2 {
    containing_contour_bucket_count: usize,
    containment_ref_count: usize,
    uncontained_assignment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleContainmentBucket2>,
}

/// Output material/hole role buckets retained after boundary contour role assignment.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputRoleCache2 {
    role_report_count: usize,
    material_contour_count: usize,
    hole_contour_count: usize,
    material_segment_count: usize,
    hole_segment_count: usize,
    role_status_bucket_cache: ExactCurveArrangementOutputRoleStatusBucketCache2,
    role_source_contour_bucket_cache: ExactCurveArrangementOutputRoleSourceContourBucketCache2,
    role_nesting_depth_bucket_cache: ExactCurveArrangementOutputRoleNestingDepthBucketCache2,
    role_containment_bucket_cache: ExactCurveArrangementOutputRoleContainmentBucketCache2,
    buckets: Vec<ExactCurveArrangementOutputRoleBucket2>,
}

/// Final boundary output counts for one material/hole role.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputBoundaryRoleBucket2 {
    role: RegionBoundaryContourRole2,
    output_contour_count: usize,
    output_segment_count: usize,
}

/// Final boundary output counts grouped by material/hole role.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputBoundaryRoleBucketCache2 {
    bucket_count: usize,
    output_contour_count: usize,
    output_segment_count: usize,
    max_segment_count: usize,
    buckets: Vec<ExactCurveArrangementOutputBoundaryRoleBucket2>,
}

/// Final boundary output summary retained by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputBoundaryCache2 {
    output_contour_count: usize,
    output_segment_count: usize,
    output_segment_kind_counts: SegmentKindCounts,
    material_contour_count: usize,
    hole_contour_count: usize,
    material_segment_count: usize,
    hole_segment_count: usize,
    role_bucket_cache: ExactCurveArrangementOutputBoundaryRoleBucketCache2,
}

/// Retained exact ring-traversal evidence cached by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementRingAssemblyCache2 {
    predicate_path: RegionLineSegmentRingAssemblyPredicatePath2,
    attempted_endpoint_connection_count: usize,
    exact_endpoint_connection_count: usize,
    disconnected_endpoint_connection_count: usize,
    unresolved_endpoint_connection_count: usize,
    reversed_source_segment_count: usize,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
    arranged_source_reports: Vec<RegionLineSegmentArrangedSourceReport2>,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
    arranged_fragment_cache: ExactCurveArrangementArrangedFragmentCache2,
    output_ring_bucket_cache: ExactCurveArrangementOutputRingBucketCache2,
    output_segment_kind_bucket_cache: ExactCurveArrangementOutputSegmentKindBucketCache2,
    output_segment_source_bucket_cache: ExactCurveArrangementOutputSegmentSourceBucketCache2,
    output_segment_source_range_cache: ExactCurveArrangementOutputSegmentSourceRangeCache2,
    output_segment_endpoint_cache: ExactCurveArrangementOutputSegmentEndpointCache2,
    output_ring_continuity_cache: ExactCurveArrangementOutputRingContinuityCache2,
    output_segment_status_bucket_cache: ExactCurveArrangementOutputSegmentStatusBucketCache2,
    output_segment_direction_bucket_cache: ExactCurveArrangementOutputSegmentDirectionBucketCache2,
}

/// Retained final output evidence cached by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementOutputCache2 {
    materialized_region: bool,
    boundary_build_report: Option<RegionBoundaryContourBuildReport2>,
    boundary_output_cache: Option<ExactCurveArrangementOutputBoundaryCache2>,
    role_cache: Option<ExactCurveArrangementOutputRoleCache2>,
    stage: RegionLineSegmentRegionBuildStage2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Final retained evaluation facts derived from workspace caches.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementEvaluationSummaryCache2 {
    evaluated_output: bool,
    materialized_region: Option<bool>,
    stage: Option<RegionLineSegmentRegionBuildStage2>,
    status: Option<RetainedTopologyStatus>,
    blocker: Option<UncertaintyReason>,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
    output_contour_count: Option<usize>,
    output_segment_count: Option<usize>,
}

/// Evaluation record for a retained exact curve arrangement attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementEvaluation2 {
    workspace: ExactCurveWorkspace2,
    summary_cache: ExactCurveArrangementEvaluationSummaryCache2,
}

/// Canonical exact curve arrangement attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementAttempt2 {
    request: ExactCurveArrangementRequest2,
}

/// Result of a retained exact curve arrangement attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCurveArrangementResult2 {
    evaluation: ExactCurveArrangementEvaluation2,
    region_result: RegionLineSegmentRegionBuildResult2,
}

/// Material/hole role assigned to one closed boundary contour.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBoundaryContourRole2 {
    /// The contour contributes filled material.
    Material,
    /// The contour contributes a subtractive hole.
    Hole,
}

/// Role assignment for one source boundary contour.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourRoleReport2 {
    source_contour_index: usize,
    source_segment_count: usize,
    source_fill_rule: FillRule,
    nesting_sample_point: Point2,
    containing_contour_indices: Vec<usize>,
    nesting_depth: usize,
    role: RegionBoundaryContourRole2,
    output_role_index: usize,
    status: RetainedTopologyStatus,
}

/// Report for building a region from already-closed boundary contours.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourBuildReport2 {
    stage: RegionBoundaryContourBuildStage2,
    predicate_path: RegionBoundaryContourBuildPredicatePath2,
    source_contour_count: usize,
    source_segment_count: usize,
    validation_candidate_pair_count: usize,
    validation_tested_pair_count: usize,
    validation_intersection_event_count: usize,
    nesting_classification_count: usize,
    blocker_first_contour_index: Option<usize>,
    blocker_second_contour_index: Option<usize>,
    output_contour_count: Option<usize>,
    output_segment_count: Option<usize>,
    material_contour_count: Option<usize>,
    hole_contour_count: Option<usize>,
    material_segment_count: Option<usize>,
    hole_segment_count: Option<usize>,
    role_reports: Vec<RegionBoundaryContourRoleReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by boundary-contour region construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBoundaryContourBuildStage2 {
    /// Contour intersections and containment nesting were being validated.
    NestingValidation,
    /// Material and hole role bins were assigned and materialized.
    RoleAssignment,
}

/// Exact predicate path used while nesting closed boundary contours.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBoundaryContourBuildPredicatePath2 {
    /// Boundary validation used contour intersections and exact point-containment nesting tests.
    ExactContourIntersectionAndPointContainment,
}

/// Result of report-bearing boundary contour region construction.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourBuildResult2 {
    region: Option<Region2>,
    report: RegionBoundaryContourBuildReport2,
}

/// Source line-segment provenance for one assembled boundary ring segment.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRingSourceReport2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    output_ring_index: usize,
    output_segment_index: usize,
    output_segment_kind: SegmentKind,
    reversed: bool,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Source provenance for one arranged fragment before ring traversal.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentArrangedSourceReport2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    arranged_segment_index: usize,
    arranged_segment_kind: SegmentKind,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Arranged segment endpoint reported by unordered region endpoint-graph checks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionLineSegmentArrangedEndpoint2 {
    /// Start point of the arranged fragment.
    Start,
    /// End point of the arranged fragment.
    End,
}

/// Retained point-intersection evidence collected before unordered region assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentSplitIntersectionReport2 {
    first_source_segment_index: usize,
    first_source_segment_kind: SegmentKind,
    first_source_segment_start_point: Point2,
    first_source_segment_end_point: Point2,
    first_source_param: Real,
    second_source_segment_index: usize,
    second_source_segment_kind: SegmentKind,
    second_source_segment_start_point: Point2,
    second_source_segment_end_point: Point2,
    second_source_param: Real,
    point: Point2,
}

/// Exact predicate family used while arranging unordered segments at split points.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionLineSegmentSplitPredicatePath2 {
    /// Line-only construction used exact line-line intersection predicates after AABB filtering.
    AabbFilteredExactLineLine,
    /// Native line/arc construction used exact native segment intersection predicates after AABB filtering.
    AabbFilteredNativeSegment,
}

/// Exact predicate family used while validating arranged segment endpoint topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionLineSegmentEndpointGraphPredicatePath2 {
    /// Arranged endpoints were bucketed by exact structural point equality.
    ExactStructuralEndpointBuckets,
}

/// Exact predicate family used while traversing validated endpoint topology into rings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionLineSegmentRingAssemblyPredicatePath2 {
    /// Ring traversal followed exact structural endpoint buckets.
    ExactEndpointBucketTraversal,
}

/// Report for constructing a region from unordered exact line segments.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRegionBuildReport2 {
    stage: RegionLineSegmentRegionBuildStage2,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    arranged_segment_count: Option<usize>,
    arranged_segment_kind_counts: Option<SegmentKindCounts>,
    split_predicate_path: Option<RegionLineSegmentSplitPredicatePath2>,
    endpoint_graph_predicate_path: Option<RegionLineSegmentEndpointGraphPredicatePath2>,
    ring_assembly_predicate_path: Option<RegionLineSegmentRingAssemblyPredicatePath2>,
    split_candidate_pair_count: usize,
    split_skipped_aabb_pair_count: usize,
    split_tested_pair_count: usize,
    split_intersection_event_count: usize,
    split_point_relation_count: usize,
    split_overlap_relation_count: usize,
    split_uncertain_relation_count: usize,
    split_intersection_points: Vec<Point2>,
    split_intersection_reports: Vec<RegionLineSegmentSplitIntersectionReport2>,
    split_output_segment_count: Option<usize>,
    split_blocker_first_source_segment_index: Option<usize>,
    split_blocker_first_source_segment_kind: Option<SegmentKind>,
    split_blocker_first_source_start_point: Option<Point2>,
    split_blocker_first_source_end_point: Option<Point2>,
    split_blocker_second_source_segment_index: Option<usize>,
    split_blocker_second_source_segment_kind: Option<SegmentKind>,
    split_blocker_second_source_start_point: Option<Point2>,
    split_blocker_second_source_end_point: Option<Point2>,
    endpoint_graph_endpoint_count: Option<usize>,
    endpoint_graph_structural_bucket_count: Option<usize>,
    endpoint_graph_structural_singleton_bucket_count: Option<usize>,
    endpoint_graph_max_structural_bucket_size: Option<usize>,
    endpoint_graph_dangling_endpoint_count: Option<usize>,
    endpoint_graph_branch_endpoint_count: Option<usize>,
    endpoint_graph_blocker_arranged_segment_index: Option<usize>,
    endpoint_graph_blocker_endpoint: Option<RegionLineSegmentArrangedEndpoint2>,
    endpoint_graph_blocker_point: Option<Point2>,
    attempted_endpoint_connection_count: usize,
    exact_endpoint_connection_count: usize,
    disconnected_endpoint_connection_count: usize,
    unresolved_endpoint_connection_count: usize,
    reversed_source_segment_count: usize,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
    arranged_source_reports: Vec<RegionLineSegmentArrangedSourceReport2>,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
    boundary_build_report: Option<RegionBoundaryContourBuildReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached while assembling unordered line segments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionLineSegmentRegionBuildStage2 {
    /// The unordered endpoint graph was being assembled into closed rings.
    RingAssembly,
    /// Assembled line rings were being replayed as checked contours.
    ContourMaterialization,
    /// Checked contours were being assigned material/hole roles.
    RegionRoleAssignment,
}

/// Result of report-bearing region construction from unordered exact line segments.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRegionBuildResult2 {
    region: Option<Region2>,
    report: RegionLineSegmentRegionBuildReport2,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingDepths {
    entries: Vec<BoundaryContourNestingEntry>,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingEntry {
    sample_point: Point2,
    containing_contour_indices: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq)]
struct BoundaryContourNestingBlocker {
    reason: UncertaintyReason,
    first_contour_index: usize,
    second_contour_index: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BoundaryContourValidationCounts {
    candidate_pair_count: usize,
    tested_pair_count: usize,
    intersection_event_count: usize,
    nesting_classification_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
enum BoundaryContourNestingOutcome {
    Decided {
        nesting: BoundaryContourNestingDepths,
        counts: BoundaryContourValidationCounts,
    },
    Blocked {
        blocker: BoundaryContourNestingBlocker,
        counts: BoundaryContourValidationCounts,
    },
}

impl Region2 {
    /// Builds a region from unordered exact line segments that form closed rings.
    ///
    /// This is a narrow first utility for "make region from lines" workflows:
    /// it accepts already-authored finite line segments, splits certified point
    /// intersections, chooses connections only from exact endpoint equality,
    /// reorients source segments as needed, materializes checked contours, and
    /// then delegates material/hole role assignment to
    /// [`Region2::from_boundary_contours_with_report`]. It does not snap
    /// endpoints or resolve overlaps; disconnected, ambiguous, unresolved, or
    /// branching endpoint graphs are returned as explicit blockers.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_unordered_line_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_line_segments(
        segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let request =
            ExactCurveArrangementRequest2::from_unordered_line_segments(segments, fill_rule);
        let result = ExactCurveArrangementAttempt2::new(request).evaluate(policy)?;
        let blocker = result.blocker().unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = result.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region from borrowed unordered exact line segments.
    ///
    /// This is the borrowed counterpart to
    /// [`Region2::from_unordered_line_segments`]. The exact segment carriers
    /// are cloned into the existing arrangement-first implementation so source
    /// ordering, split parameters, endpoint graph evidence, and blockers remain
    /// identical to the owned path.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_line_segments_borrowed(
        segments: &[LineSeg2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let request = ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments(
            segments, fill_rule,
        );
        let result = ExactCurveArrangementAttempt2::new(request).evaluate(policy)?;
        let blocker = result.blocker().unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = result.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region from borrowed unordered exact line segments and retains evidence.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_line_segments_borrowed_with_report(
        segments: &[LineSeg2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
        let request = ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments(
            segments, fill_rule,
        );
        Ok(ExactCurveArrangementAttempt2::new(request)
            .evaluate(policy)?
            .region_result)
    }

    /// Builds a region from unordered exact line segments and retains assembly evidence.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_unordered_line_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_line_segments_with_report(
        segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
        let request =
            ExactCurveArrangementRequest2::from_unordered_line_segments(segments, fill_rule);
        Ok(ExactCurveArrangementAttempt2::new(request)
            .evaluate(policy)?
            .region_result)
    }
}

fn evaluate_unordered_line_segments_region_result(
    segments: Vec<LineSeg2>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let arranged = match arrange_line_segments_at_point_intersections(&segments, policy)? {
        Ok(arranged) => arranged,
        Err((split_report, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                report: blocked_line_segment_region_report(
                    segments.len(),
                    line_segment_kind_counts(segments.len()),
                    Some(split_report),
                    None,
                    Vec::new(),
                    LineSegmentRingAssemblyReportParts::default(),
                    RegionLineSegmentRegionBuildStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    let (endpoint_graph, endpoint_counts) =
        match validate_arranged_line_endpoint_graph(&arranged.segments, policy) {
            Ok(endpoint_graph) => endpoint_graph,
            Err((endpoint_graph, counts, blocker)) => {
                return Ok(RegionLineSegmentRegionBuildResult2 {
                    region: None,
                    report: blocked_line_segment_region_report(
                        segments.len(),
                        line_segment_kind_counts(segments.len()),
                        Some(arranged.report),
                        Some(endpoint_graph),
                        line_arranged_source_reports(&arranged.segments),
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            ..LineSegmentRingAssemblyReportParts::default()
                        },
                        RegionLineSegmentRegionBuildStage2::RingAssembly,
                        retained_status_for_line_segment_region_blocker(blocker),
                        blocker,
                    ),
                });
            }
        };

    let assembled = match assemble_unordered_line_segment_rings(&arranged.segments, policy)? {
        Ok(assembled) => assembled,
        Err((report, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                report: blocked_line_segment_region_report(
                    segments.len(),
                    line_segment_kind_counts(segments.len()),
                    Some(arranged.report),
                    Some(endpoint_graph),
                    line_arranged_source_reports(&arranged.segments),
                    report,
                    RegionLineSegmentRegionBuildStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    let mut contours = Vec::with_capacity(assembled.rings.len());
    for ring in assembled.rings {
        let contour = Contour2::try_new_with_fill_rule(
            ring.into_iter().map(Segment2::Line).collect(),
            fill_rule,
        )?;
        contours.push(contour);
    }

    let built = Region2::from_boundary_contours_with_report(contours, policy)?;
    let status = built.report().status();
    let blocker = built.report().blocker();
    let boundary_build_report = built.report().clone();
    let output_ring_count = boundary_build_report.output_contour_count();
    let output_boundary_segment_count = boundary_build_report.output_segment_count();
    let output_boundary_segment_kind_counts = built.region().map(region_segment_kind_counts);
    Ok(RegionLineSegmentRegionBuildResult2 {
        region: built.into_region(),
        report: RegionLineSegmentRegionBuildReport2 {
            stage: RegionLineSegmentRegionBuildStage2::RegionRoleAssignment,
            source_segment_count: segments.len(),
            source_segment_kind_counts: line_segment_kind_counts(segments.len()),
            arranged_segment_count: Some(arranged.segments.len()),
            arranged_segment_kind_counts: Some(line_segment_kind_counts(arranged.segments.len())),
            split_predicate_path: arranged.report.predicate_path,
            endpoint_graph_predicate_path: Some(
                RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets,
            ),
            ring_assembly_predicate_path: Some(
                RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal,
            ),
            split_candidate_pair_count: arranged.report.candidate_pair_count,
            split_skipped_aabb_pair_count: arranged.report.skipped_aabb_pair_count,
            split_tested_pair_count: arranged.report.tested_pair_count,
            split_intersection_event_count: arranged.report.intersection_event_count,
            split_point_relation_count: arranged.report.point_relation_count,
            split_overlap_relation_count: arranged.report.overlap_relation_count,
            split_uncertain_relation_count: arranged.report.uncertain_relation_count,
            split_intersection_points: arranged.report.intersection_points,
            split_intersection_reports: arranged.report.intersection_reports,
            split_output_segment_count: Some(arranged.segments.len()),
            split_blocker_first_source_segment_index: arranged
                .report
                .blocker_first_source_segment_index,
            split_blocker_first_source_segment_kind: arranged
                .report
                .blocker_first_source_segment_kind,
            split_blocker_first_source_start_point: arranged
                .report
                .blocker_first_source_start_point,
            split_blocker_first_source_end_point: arranged.report.blocker_first_source_end_point,
            split_blocker_second_source_segment_index: arranged
                .report
                .blocker_second_source_segment_index,
            split_blocker_second_source_segment_kind: arranged
                .report
                .blocker_second_source_segment_kind,
            split_blocker_second_source_start_point: arranged
                .report
                .blocker_second_source_start_point,
            split_blocker_second_source_end_point: arranged.report.blocker_second_source_end_point,
            endpoint_graph_endpoint_count: Some(endpoint_graph.endpoint_count),
            endpoint_graph_structural_bucket_count: Some(endpoint_graph.structural_bucket_count),
            endpoint_graph_structural_singleton_bucket_count: Some(
                endpoint_graph.structural_singleton_bucket_count,
            ),
            endpoint_graph_max_structural_bucket_size: Some(
                endpoint_graph.max_structural_bucket_size,
            ),
            endpoint_graph_dangling_endpoint_count: Some(endpoint_graph.dangling_endpoint_count),
            endpoint_graph_branch_endpoint_count: Some(endpoint_graph.branch_endpoint_count),
            endpoint_graph_blocker_arranged_segment_index: endpoint_graph
                .blocker_arranged_segment_index,
            endpoint_graph_blocker_endpoint: endpoint_graph.blocker_endpoint,
            endpoint_graph_blocker_point: endpoint_graph.blocker_point,
            attempted_endpoint_connection_count: assembled
                .counts
                .attempted_endpoint_connection_count
                + endpoint_counts.attempted_endpoint_connection_count,
            exact_endpoint_connection_count: assembled.counts.exact_endpoint_connection_count
                + endpoint_counts.exact_endpoint_connection_count,
            disconnected_endpoint_connection_count: assembled
                .counts
                .disconnected_endpoint_connection_count
                + endpoint_counts.disconnected_endpoint_connection_count,
            unresolved_endpoint_connection_count: assembled
                .counts
                .unresolved_endpoint_connection_count
                + endpoint_counts.unresolved_endpoint_connection_count,
            reversed_source_segment_count: assembled.reversed_source_segment_count,
            output_ring_count,
            output_boundary_segment_count,
            output_boundary_segment_kind_counts,
            arranged_source_reports: line_arranged_source_reports(&arranged.segments),
            source_reports: assembled.source_reports,
            boundary_build_report: Some(boundary_build_report),
            status,
            blocker,
        },
    })
}

impl Region2 {
    /// Builds a region from unordered exact native line/arc segments that form closed rings.
    ///
    /// This is the convenience wrapper for
    /// [`Region2::from_unordered_segments_with_report`]. It returns decided
    /// regions only when the report-bearing path materializes native exact
    /// topology; otherwise it preserves the reported blocker as uncertainty.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_unordered_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_segments(
        segments: Vec<Segment2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let request = ExactCurveArrangementRequest2::from_unordered_segments(segments, fill_rule);
        let result = ExactCurveArrangementAttempt2::new(request).evaluate(policy)?;
        let blocker = result.blocker().unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = result.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region from borrowed unordered exact native line/arc segments.
    ///
    /// This is the borrowed counterpart to [`Region2::from_unordered_segments`].
    /// It delegates to the same exact arrangement, splitting, endpoint graph,
    /// ring traversal, and boundary role-assignment pipeline as the owned API.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_borrowed_unordered_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_segments_borrowed(
        segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let request =
            ExactCurveArrangementRequest2::from_borrowed_unordered_segments(segments, fill_rule);
        let result = ExactCurveArrangementAttempt2::new(request).evaluate(policy)?;
        let blocker = result.blocker().unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = result.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region from borrowed unordered native line/arc segments and retains evidence.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_borrowed_unordered_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_segments_borrowed_with_report(
        segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
        let request =
            ExactCurveArrangementRequest2::from_borrowed_unordered_segments(segments, fill_rule);
        Ok(ExactCurveArrangementAttempt2::new(request)
            .evaluate(policy)?
            .region_result)
    }

    /// Builds a region from unordered exact native line/arc segments.
    ///
    /// This is the native-segment counterpart to
    /// [`Region2::from_unordered_line_segments_with_report`]. It retains exact
    /// point-intersection split parameters for lines and arcs, materializes
    /// line/arc fragments from exact split points, and then assembles closed
    /// rings by exact endpoint equality. Overlaps, ambiguous ordering, and
    /// branching/dangling endpoint graphs remain explicit blockers.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementRequest2::from_unordered_segments with ExactCurveArrangementAttempt2::evaluate"
    )]
    pub fn from_unordered_segments_with_report(
        segments: Vec<Segment2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
        let request = ExactCurveArrangementRequest2::from_unordered_segments(segments, fill_rule);
        Ok(ExactCurveArrangementAttempt2::new(request)
            .evaluate(policy)?
            .region_result)
    }
}

fn evaluate_unordered_segments_region_result(
    segments: Vec<Segment2>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let arranged = match arrange_native_segments_at_point_intersections(&segments, policy)? {
        Ok(arranged) => arranged,
        Err((split_report, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                report: blocked_line_segment_region_report(
                    segments.len(),
                    segment_kind_counts(&segments),
                    Some(split_report),
                    None,
                    Vec::new(),
                    LineSegmentRingAssemblyReportParts::default(),
                    RegionLineSegmentRegionBuildStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    let (endpoint_graph, endpoint_counts) =
        match validate_arranged_native_endpoint_graph(&arranged.segments, policy) {
            Ok(endpoint_graph) => endpoint_graph,
            Err((endpoint_graph, counts, blocker)) => {
                return Ok(RegionLineSegmentRegionBuildResult2 {
                    region: None,
                    report: blocked_line_segment_region_report(
                        segments.len(),
                        segment_kind_counts(&segments),
                        Some(arranged.report),
                        Some(endpoint_graph),
                        native_arranged_source_reports(&segments, &arranged.segments),
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            ..LineSegmentRingAssemblyReportParts::default()
                        },
                        RegionLineSegmentRegionBuildStage2::RingAssembly,
                        retained_status_for_line_segment_region_blocker(blocker),
                        blocker,
                    ),
                });
            }
        };

    let assembled = match assemble_unordered_native_segment_rings(&arranged.segments, policy)? {
        Ok(assembled) => assembled,
        Err((report, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                report: blocked_line_segment_region_report(
                    segments.len(),
                    segment_kind_counts(&segments),
                    Some(arranged.report),
                    Some(endpoint_graph),
                    native_arranged_source_reports(&segments, &arranged.segments),
                    report,
                    RegionLineSegmentRegionBuildStage2::RingAssembly,
                    retained_status_for_line_segment_region_blocker(blocker),
                    blocker,
                ),
            });
        }
    };

    let mut contours = Vec::with_capacity(assembled.rings.len());
    for ring in assembled.rings {
        contours.push(Contour2::try_new_with_fill_rule(ring, fill_rule)?);
    }

    let built = Region2::from_boundary_contours_with_report(contours, policy)?;
    let status = built.report().status();
    let blocker = built.report().blocker();
    let boundary_build_report = built.report().clone();
    let output_ring_count = boundary_build_report.output_contour_count();
    let output_boundary_segment_count = boundary_build_report.output_segment_count();
    let output_boundary_segment_kind_counts = built.region().map(region_segment_kind_counts);
    Ok(RegionLineSegmentRegionBuildResult2 {
        region: built.into_region(),
        report: RegionLineSegmentRegionBuildReport2 {
            stage: RegionLineSegmentRegionBuildStage2::RegionRoleAssignment,
            source_segment_count: segments.len(),
            source_segment_kind_counts: segment_kind_counts(&segments),
            arranged_segment_count: Some(arranged.segments.len()),
            arranged_segment_kind_counts: Some(native_arranged_segment_kind_counts(
                &arranged.segments,
            )),
            split_predicate_path: arranged.report.predicate_path,
            endpoint_graph_predicate_path: Some(
                RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets,
            ),
            ring_assembly_predicate_path: Some(
                RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal,
            ),
            split_candidate_pair_count: arranged.report.candidate_pair_count,
            split_skipped_aabb_pair_count: arranged.report.skipped_aabb_pair_count,
            split_tested_pair_count: arranged.report.tested_pair_count,
            split_intersection_event_count: arranged.report.intersection_event_count,
            split_point_relation_count: arranged.report.point_relation_count,
            split_overlap_relation_count: arranged.report.overlap_relation_count,
            split_uncertain_relation_count: arranged.report.uncertain_relation_count,
            split_intersection_points: arranged.report.intersection_points,
            split_intersection_reports: arranged.report.intersection_reports,
            split_output_segment_count: Some(arranged.segments.len()),
            split_blocker_first_source_segment_index: arranged
                .report
                .blocker_first_source_segment_index,
            split_blocker_first_source_segment_kind: arranged
                .report
                .blocker_first_source_segment_kind,
            split_blocker_first_source_start_point: arranged
                .report
                .blocker_first_source_start_point,
            split_blocker_first_source_end_point: arranged.report.blocker_first_source_end_point,
            split_blocker_second_source_segment_index: arranged
                .report
                .blocker_second_source_segment_index,
            split_blocker_second_source_segment_kind: arranged
                .report
                .blocker_second_source_segment_kind,
            split_blocker_second_source_start_point: arranged
                .report
                .blocker_second_source_start_point,
            split_blocker_second_source_end_point: arranged.report.blocker_second_source_end_point,
            endpoint_graph_endpoint_count: Some(endpoint_graph.endpoint_count),
            endpoint_graph_structural_bucket_count: Some(endpoint_graph.structural_bucket_count),
            endpoint_graph_structural_singleton_bucket_count: Some(
                endpoint_graph.structural_singleton_bucket_count,
            ),
            endpoint_graph_max_structural_bucket_size: Some(
                endpoint_graph.max_structural_bucket_size,
            ),
            endpoint_graph_dangling_endpoint_count: Some(endpoint_graph.dangling_endpoint_count),
            endpoint_graph_branch_endpoint_count: Some(endpoint_graph.branch_endpoint_count),
            endpoint_graph_blocker_arranged_segment_index: endpoint_graph
                .blocker_arranged_segment_index,
            endpoint_graph_blocker_endpoint: endpoint_graph.blocker_endpoint,
            endpoint_graph_blocker_point: endpoint_graph.blocker_point,
            attempted_endpoint_connection_count: assembled
                .counts
                .attempted_endpoint_connection_count
                + endpoint_counts.attempted_endpoint_connection_count,
            exact_endpoint_connection_count: assembled.counts.exact_endpoint_connection_count
                + endpoint_counts.exact_endpoint_connection_count,
            disconnected_endpoint_connection_count: assembled
                .counts
                .disconnected_endpoint_connection_count
                + endpoint_counts.disconnected_endpoint_connection_count,
            unresolved_endpoint_connection_count: assembled
                .counts
                .unresolved_endpoint_connection_count
                + endpoint_counts.unresolved_endpoint_connection_count,
            reversed_source_segment_count: assembled.reversed_source_segment_count,
            output_ring_count,
            output_boundary_segment_count,
            output_boundary_segment_kind_counts,
            arranged_source_reports: native_arranged_source_reports(&segments, &arranged.segments),
            source_reports: assembled.source_reports,
            boundary_build_report: Some(boundary_build_report),
            status,
            blocker,
        },
    })
}

impl Region2 {
    /// Builds a region by nesting closed boundary contours into material/hole bins.
    ///
    /// Contours at even containment depth become material. Contours at odd
    /// depth become holes. This matches the even-odd nesting interpretation
    /// commonly used after boolean traversal has produced disjoint closed
    /// output loops.
    pub fn from_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let built = Self::from_boundary_contours_with_report(contours, policy)?;
        let blocker = built
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = built.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region by nesting borrowed closed boundary contours.
    ///
    /// This clones the exact contour carriers at the API boundary, then uses
    /// the same exact nesting and role-assignment pipeline as
    /// [`Region2::from_boundary_contours`].
    pub fn from_boundary_contours_borrowed(
        contours: &[Contour2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let built = Self::from_boundary_contours_borrowed_with_report(contours, policy)?;
        let blocker = built
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(region) = built.into_region() {
            Ok(Classification::Decided(region))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    /// Builds a region by nesting closed boundary contours and retaining role evidence.
    ///
    /// This is the report-bearing counterpart to
    /// [`Region2::from_boundary_contours`]. Contours at even containment depth
    /// become material and odd-depth contours become holes. If intersections,
    /// touches, or undecided containment predicates prevent role assignment, no
    /// region is materialized and the report carries the blocker.
    pub fn from_boundary_contours_with_report(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBoundaryContourBuildResult2> {
        let source_contour_count = contours.len();
        let source_segment_count = contours
            .iter()
            .map(|contour| contour.segments().len())
            .sum();
        let (nesting, counts) = match contour_nesting_depths(&contours, policy)? {
            BoundaryContourNestingOutcome::Decided { nesting, counts } => (nesting, counts),
            BoundaryContourNestingOutcome::Blocked { blocker, counts } => {
                return Ok(blocked_boundary_contour_region_result(
                    source_contour_count,
                    source_segment_count,
                    counts,
                    Some((blocker.first_contour_index, blocker.second_contour_index)),
                    retained_status_for_boundary_contour_blocker(blocker.reason),
                    blocker.reason,
                ));
            }
        };
        let mut material_contours = Vec::new();
        let mut hole_contours = Vec::new();
        let mut role_reports = Vec::with_capacity(source_contour_count);

        for (source_contour_index, (contour, entry)) in
            contours.into_iter().zip(nesting.entries.iter()).enumerate()
        {
            let source_segment_count = contour.segments().len();
            let source_fill_rule = contour.fill_rule();
            let depth = entry.containing_contour_indices.len();
            if depth % 2 == 0 {
                let output_role_index = material_contours.len();
                material_contours.push(contour);
                role_reports.push(RegionBoundaryContourRoleReport2 {
                    source_contour_index,
                    source_segment_count,
                    source_fill_rule,
                    nesting_sample_point: entry.sample_point.clone(),
                    containing_contour_indices: entry.containing_contour_indices.clone(),
                    nesting_depth: depth,
                    role: RegionBoundaryContourRole2::Material,
                    output_role_index,
                    status: RetainedTopologyStatus::NativeExact,
                });
            } else {
                let output_role_index = hole_contours.len();
                hole_contours.push(contour);
                role_reports.push(RegionBoundaryContourRoleReport2 {
                    source_contour_index,
                    source_segment_count,
                    source_fill_rule,
                    nesting_sample_point: entry.sample_point.clone(),
                    containing_contour_indices: entry.containing_contour_indices.clone(),
                    nesting_depth: depth,
                    role: RegionBoundaryContourRole2::Hole,
                    output_role_index,
                    status: RetainedTopologyStatus::NativeExact,
                });
            }
        }

        let material_contour_count = material_contours.len();
        let hole_contour_count = hole_contours.len();
        let output_contour_count = material_contour_count + hole_contour_count;
        let material_segment_count = role_reports
            .iter()
            .filter(|report| report.role == RegionBoundaryContourRole2::Material)
            .map(|report| report.source_segment_count)
            .sum();
        let hole_segment_count = role_reports
            .iter()
            .filter(|report| report.role == RegionBoundaryContourRole2::Hole)
            .map(|report| report.source_segment_count)
            .sum();
        let output_segment_count = material_segment_count + hole_segment_count;
        Ok(RegionBoundaryContourBuildResult2 {
            region: Some(Region2::new(material_contours, hole_contours)),
            report: RegionBoundaryContourBuildReport2 {
                stage: RegionBoundaryContourBuildStage2::RoleAssignment,
                predicate_path:
                    RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment,
                source_contour_count,
                source_segment_count,
                validation_candidate_pair_count: counts.candidate_pair_count,
                validation_tested_pair_count: counts.tested_pair_count,
                validation_intersection_event_count: counts.intersection_event_count,
                nesting_classification_count: counts.nesting_classification_count,
                blocker_first_contour_index: None,
                blocker_second_contour_index: None,
                output_contour_count: Some(output_contour_count),
                output_segment_count: Some(output_segment_count),
                material_contour_count: Some(material_contour_count),
                hole_contour_count: Some(hole_contour_count),
                material_segment_count: Some(material_segment_count),
                hole_segment_count: Some(hole_segment_count),
                role_reports,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Builds a region from borrowed closed boundary contours and retains role evidence.
    ///
    /// This clones the exact contour carriers at the API boundary, then
    /// delegates to [`Region2::from_boundary_contours_with_report`] so the
    /// retained nesting, validation, and material/hole role reports are
    /// identical to the owned constructor.
    pub fn from_boundary_contours_borrowed_with_report(
        contours: &[Contour2],
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBoundaryContourBuildResult2> {
        Self::from_boundary_contours_with_report(contours.to_vec(), policy)
    }
}

impl ExactCurveArrangementRequest2 {
    /// Builds a canonical arrangement request from unordered exact native segments.
    pub fn from_unordered_segments(source_segments: Vec<Segment2>, fill_rule: FillRule) -> Self {
        Self {
            source_segments,
            source_line_segments: None,
            fill_rule,
        }
    }

    /// Builds a canonical arrangement request from unordered exact line segments.
    pub fn from_unordered_line_segments(
        source_line_segments: Vec<LineSeg2>,
        fill_rule: FillRule,
    ) -> Self {
        let source_segments = source_line_segments
            .iter()
            .cloned()
            .map(Segment2::Line)
            .collect();
        Self {
            source_segments,
            source_line_segments: Some(source_line_segments),
            fill_rule,
        }
    }

    /// Builds a canonical arrangement request by cloning borrowed exact native segments.
    pub fn from_borrowed_unordered_segments(
        source_segments: &[Segment2],
        fill_rule: FillRule,
    ) -> Self {
        Self::from_unordered_segments(source_segments.to_vec(), fill_rule)
    }

    /// Builds a canonical arrangement request by cloning borrowed exact line segments.
    pub fn from_borrowed_unordered_line_segments(
        source_line_segments: &[LineSeg2],
        fill_rule: FillRule,
    ) -> Self {
        Self::from_unordered_line_segments(source_line_segments.to_vec(), fill_rule)
    }

    /// Returns the source segments supplied to the arrangement attempt.
    pub fn source_segments(&self) -> &[Segment2] {
        &self.source_segments
    }

    /// Returns line-only source carriers when the request came from the line-specific API.
    pub fn source_line_segments(&self) -> Option<&[LineSeg2]> {
        self.source_line_segments.as_deref()
    }

    /// Returns the fill rule used when closed loops become contours.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns the number of source segments supplied to the attempt.
    pub fn source_segment_count(&self) -> usize {
        self.source_segments.len()
    }
}

impl ExactCurveArrangementSourceSegmentFact2 {
    /// Returns the source segment index in request order.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the exact source segment start point.
    pub const fn source_start_point(&self) -> &Point2 {
        &self.source_start_point
    }

    /// Returns the exact source segment end point.
    pub const fn source_end_point(&self) -> &Point2 {
        &self.source_end_point
    }

    /// Returns the certified source segment AABB when available.
    pub const fn source_aabb(&self) -> Option<&Aabb2> {
        self.source_aabb.as_ref()
    }
}

impl ExactCurveArrangementSourceAabbRef2 {
    /// Returns the index into [`ExactCurveArrangementSourceSegmentCache2::segments`].
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }
}

impl ExactCurveArrangementSourceAabbBucket2 {
    /// Returns the retained AABB certification status represented by this bucket.
    pub const fn aabb_status(&self) -> ExactCurveArrangementSourceAabbStatus2 {
        self.aabb_status
    }

    /// Returns source segment references with this retained AABB status.
    pub fn source_refs(&self) -> &[ExactCurveArrangementSourceAabbRef2] {
        &self.source_refs
    }
}

impl ExactCurveArrangementSourceAabbBucketCache2 {
    fn from_source_aabbs(source_segment_aabbs: &[Option<Aabb2>]) -> Self {
        let mut decided_refs = Vec::new();
        let mut undecided_refs = Vec::new();

        for (source_segment_index, source_aabb) in source_segment_aabbs.iter().enumerate() {
            let source_ref = ExactCurveArrangementSourceAabbRef2 {
                source_segment_index,
            };
            if source_aabb.is_some() {
                decided_refs.push(source_ref);
            } else {
                undecided_refs.push(source_ref);
            }
        }

        let decided_source_ref_count = decided_refs.len();
        let undecided_source_ref_count = undecided_refs.len();
        let buckets = vec![
            ExactCurveArrangementSourceAabbBucket2 {
                aabb_status: ExactCurveArrangementSourceAabbStatus2::Decided,
                source_refs: decided_refs,
            },
            ExactCurveArrangementSourceAabbBucket2 {
                aabb_status: ExactCurveArrangementSourceAabbStatus2::Undecided,
                source_refs: undecided_refs,
            },
        ];
        let source_ref_count = source_segment_aabbs.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.source_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            source_ref_count,
            decided_source_ref_count,
            undecided_source_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of AABB-status buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained source AABB references.
    pub const fn source_ref_count(&self) -> usize {
        self.source_ref_count
    }

    /// Returns the number of source segment boxes certified during workspace preparation.
    pub const fn decided_source_ref_count(&self) -> usize {
        self.decided_source_ref_count
    }

    /// Returns the number of source segment boxes that stayed uncertain.
    pub const fn undecided_source_ref_count(&self) -> usize {
        self.undecided_source_ref_count
    }

    /// Returns the largest AABB-status bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns source AABB buckets in stable status order.
    pub fn buckets(&self) -> &[ExactCurveArrangementSourceAabbBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementSourceSegmentKindRef2 {
    /// Returns the index into [`ExactCurveArrangementSourceSegmentCache2::segments`].
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }
}

impl ExactCurveArrangementSourceSegmentKindBucket2 {
    /// Returns the retained primitive family represented by this bucket.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns source segment references with this retained primitive family.
    pub fn source_refs(&self) -> &[ExactCurveArrangementSourceSegmentKindRef2] {
        &self.source_refs
    }
}

impl ExactCurveArrangementSourceSegmentKindBucketCache2 {
    fn from_segments(segments: &[ExactCurveArrangementSourceSegmentFact2]) -> Self {
        let mut line_refs = Vec::new();
        let mut arc_refs = Vec::new();

        for segment in segments {
            let source_ref = ExactCurveArrangementSourceSegmentKindRef2 {
                source_segment_index: segment.source_segment_index(),
            };
            match segment.source_segment_kind() {
                SegmentKind::Line => line_refs.push(source_ref),
                SegmentKind::Arc => arc_refs.push(source_ref),
            }
        }

        let line_segment_ref_count = line_refs.len();
        let arc_segment_ref_count = arc_refs.len();
        let buckets = vec![
            ExactCurveArrangementSourceSegmentKindBucket2 {
                source_segment_kind: SegmentKind::Line,
                source_refs: line_refs,
            },
            ExactCurveArrangementSourceSegmentKindBucket2 {
                source_segment_kind: SegmentKind::Arc,
                source_refs: arc_refs,
            },
        ];
        let source_segment_ref_count = segments.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.source_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            source_segment_ref_count,
            line_segment_ref_count,
            arc_segment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of primitive-family buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained source segment references.
    pub const fn source_segment_ref_count(&self) -> usize {
        self.source_segment_ref_count
    }

    /// Returns the number of retained line source segment references.
    pub const fn line_segment_ref_count(&self) -> usize {
        self.line_segment_ref_count
    }

    /// Returns the number of retained arc source segment references.
    pub const fn arc_segment_ref_count(&self) -> usize {
        self.arc_segment_ref_count
    }

    /// Returns the largest primitive-family bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns source segment primitive-family buckets in stable kind order.
    pub fn buckets(&self) -> &[ExactCurveArrangementSourceSegmentKindBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementSourceSegmentCache2 {
    fn from_sources(
        source_segments: &[Segment2],
        source_segment_aabbs: &[Option<Aabb2>],
        source_segment_kind_counts: SegmentKindCounts,
        source_aabb: Option<Aabb2>,
    ) -> Self {
        let segments = source_segments
            .iter()
            .zip(source_segment_aabbs.iter())
            .enumerate()
            .map(|(source_segment_index, (source_segment, source_aabb))| {
                ExactCurveArrangementSourceSegmentFact2 {
                    source_segment_index,
                    source_segment_kind: source_segment.structural_facts().kind,
                    source_start_point: source_segment.start().clone(),
                    source_end_point: source_segment.end().clone(),
                    source_aabb: source_aabb.clone(),
                }
            })
            .collect::<Vec<_>>();
        let decided_source_segment_aabb_count = source_segment_aabbs
            .iter()
            .filter(|source_aabb| source_aabb.is_some())
            .count();
        let source_aabb_bucket_cache =
            ExactCurveArrangementSourceAabbBucketCache2::from_source_aabbs(source_segment_aabbs);
        let source_segment_kind_bucket_cache =
            ExactCurveArrangementSourceSegmentKindBucketCache2::from_segments(&segments);
        Self {
            source_segment_count: source_segments.len(),
            source_segment_kind_counts,
            decided_source_segment_aabb_count,
            undecided_source_segment_aabb_count: source_segments
                .len()
                .saturating_sub(decided_source_segment_aabb_count),
            source_aabb,
            source_aabb_bucket_cache,
            source_segment_kind_bucket_cache,
            segments,
        }
    }

    /// Returns the number of retained source segments.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns source segment primitive-family counts.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns the number of source segment boxes certified during workspace preparation.
    pub const fn decided_source_segment_aabb_count(&self) -> usize {
        self.decided_source_segment_aabb_count
    }

    /// Returns the number of source segment boxes that stayed uncertain.
    pub const fn undecided_source_segment_aabb_count(&self) -> usize {
        self.undecided_source_segment_aabb_count
    }

    /// Returns the retained aggregate source box when every source box was decided.
    pub const fn source_aabb(&self) -> Option<&Aabb2> {
        self.source_aabb.as_ref()
    }

    /// Returns retained source AABB buckets grouped by certification status.
    pub const fn source_aabb_bucket_cache(&self) -> &ExactCurveArrangementSourceAabbBucketCache2 {
        &self.source_aabb_bucket_cache
    }

    /// Returns retained source segment buckets grouped by primitive family.
    pub const fn source_segment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceSegmentKindBucketCache2 {
        &self.source_segment_kind_bucket_cache
    }

    /// Returns source segment facts in request order.
    pub fn segments(&self) -> &[ExactCurveArrangementSourceSegmentFact2] {
        &self.segments
    }
}

impl ExactCurveWorkspace2 {
    /// Builds retained workspace facts for a canonical arrangement request.
    pub fn from_request(
        request: ExactCurveArrangementRequest2,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        let source_segment_kind_counts = segment_kind_counts(&request.source_segments);
        let source_segment_aabbs = source_segment_aabbs(&request.source_segments, policy)?;
        let source_aabb = union_decided_aabbs(&source_segment_aabbs, policy);
        let source_segment_cache = ExactCurveArrangementSourceSegmentCache2::from_sources(
            &request.source_segments,
            &source_segment_aabbs,
            source_segment_kind_counts,
            source_aabb.clone(),
        );
        let source_endpoint_bucket_cache = source_endpoint_bucket_cache(&request.source_segments);
        let split_schedule_cache = split_schedule_cache(&source_segment_aabbs, policy);
        Ok(Self {
            request,
            source_segment_kind_counts,
            source_segment_aabbs,
            source_aabb,
            source_segment_cache,
            source_endpoint_bucket_cache,
            split_schedule_cache,
            split_cache: None,
            endpoint_graph_cache: None,
            ring_assembly_cache: None,
            output_cache: None,
        })
    }

    fn with_arrangement_result_facts(
        mut self,
        region_result: &RegionLineSegmentRegionBuildResult2,
    ) -> Self {
        self.split_cache = Some(ExactCurveArrangementSplitCache2::from_arrangement_report(
            region_result.report(),
        ));
        self.endpoint_graph_cache =
            ExactCurveArrangementEndpointGraphCache2::from_arrangement_report(
                region_result.report(),
            );
        self.ring_assembly_cache = ExactCurveArrangementRingAssemblyCache2::from_arrangement_report(
            region_result.report(),
        );
        self.output_cache = Some(ExactCurveArrangementOutputCache2::from_arrangement_result(
            region_result,
        ));
        self
    }

    /// Returns the retained request.
    pub const fn request(&self) -> &ExactCurveArrangementRequest2 {
        &self.request
    }

    /// Returns retained source segment primitive-family counts.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns retained source segment boxes in request order.
    pub fn source_segment_aabbs(&self) -> &[Option<Aabb2>] {
        &self.source_segment_aabbs
    }

    /// Returns a retained aggregate source box when every source box was decided.
    pub const fn source_aabb(&self) -> Option<&Aabb2> {
        self.source_aabb.as_ref()
    }

    /// Returns the number of source segment boxes certified during workspace preparation.
    pub fn decided_source_segment_aabb_count(&self) -> usize {
        self.source_segment_cache
            .decided_source_segment_aabb_count()
    }

    /// Returns the number of source segment boxes that stayed uncertain.
    pub fn undecided_source_segment_aabb_count(&self) -> usize {
        self.source_segment_cache
            .undecided_source_segment_aabb_count()
    }

    /// Returns retained source segment facts prepared before split scheduling.
    pub const fn source_segment_cache(&self) -> &ExactCurveArrangementSourceSegmentCache2 {
        &self.source_segment_cache
    }

    /// Returns exact source endpoint buckets retained during workspace preparation.
    pub const fn source_endpoint_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceEndpointBucketCache2 {
        &self.source_endpoint_bucket_cache
    }

    /// Returns the retained source-pair schedule prepared before split predicates run.
    pub const fn split_schedule_cache(&self) -> &ExactCurveArrangementSplitScheduleCache2 {
        &self.split_schedule_cache
    }

    /// Returns exact split evidence retained from the evaluated arrangement.
    pub const fn split_cache(&self) -> Option<&ExactCurveArrangementSplitCache2> {
        self.split_cache.as_ref()
    }

    /// Returns exact endpoint-bucket evidence retained from the evaluated arrangement.
    pub const fn endpoint_graph_cache(&self) -> Option<&ExactCurveArrangementEndpointGraphCache2> {
        self.endpoint_graph_cache.as_ref()
    }

    /// Returns exact ring-traversal evidence retained from the evaluated arrangement.
    pub const fn ring_assembly_cache(&self) -> Option<&ExactCurveArrangementRingAssemblyCache2> {
        self.ring_assembly_cache.as_ref()
    }

    /// Returns final output evidence retained from the evaluated arrangement.
    pub const fn output_cache(&self) -> Option<&ExactCurveArrangementOutputCache2> {
        self.output_cache.as_ref()
    }
}

impl ExactCurveArrangementSourceEndpointRef2 {
    /// Returns the source segment index that owns this endpoint.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns which source endpoint this reference points at.
    pub const fn endpoint(&self) -> ExactCurveArrangementSourceEndpoint2 {
        self.endpoint
    }
}

impl ExactCurveArrangementSourceEndpointBucket2 {
    /// Returns the exact structural point shared by this source endpoint bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns source endpoints in request encounter order.
    pub fn endpoints(&self) -> &[ExactCurveArrangementSourceEndpointRef2] {
        &self.endpoints
    }
}

impl ExactCurveArrangementSourceEndpointBucketCache2 {
    /// Returns the number of source endpoints bucketed.
    pub const fn endpoint_count(&self) -> usize {
        self.endpoint_count
    }

    /// Returns the number of exact structural source endpoint buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns buckets containing one source endpoint.
    pub const fn singleton_bucket_count(&self) -> usize {
        self.singleton_bucket_count
    }

    /// Returns the largest source endpoint bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns exact structural source endpoint buckets in encounter order.
    pub fn buckets(&self) -> &[ExactCurveArrangementSourceEndpointBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementSplitCandidatePair2 {
    /// Returns the first source segment index in this scheduled pair.
    pub const fn first_source_segment_index(&self) -> usize {
        self.first_source_segment_index
    }

    /// Returns the second source segment index in this scheduled pair.
    pub const fn second_source_segment_index(&self) -> usize {
        self.second_source_segment_index
    }

    /// Returns the retained AABB pruning status for this scheduled pair.
    pub const fn aabb_status(&self) -> ExactCurveArrangementSplitCandidateAabbStatus2 {
        self.aabb_status
    }
}

impl ExactCurveArrangementSplitScheduleRef2 {
    /// Returns the index into [`ExactCurveArrangementSplitScheduleCache2::candidate_pairs`].
    pub const fn candidate_pair_index(&self) -> usize {
        self.candidate_pair_index
    }
}

impl ExactCurveArrangementSplitScheduleBucket2 {
    /// Returns the retained AABB pruning status represented by this bucket.
    pub const fn aabb_status(&self) -> ExactCurveArrangementSplitCandidateAabbStatus2 {
        self.aabb_status
    }

    /// Returns scheduled candidate pair references with this AABB status.
    pub fn candidate_refs(&self) -> &[ExactCurveArrangementSplitScheduleRef2] {
        &self.candidate_refs
    }
}

impl ExactCurveArrangementSplitScheduleBucketCache2 {
    fn from_candidate_pairs(candidate_pairs: &[ExactCurveArrangementSplitCandidatePair2]) -> Self {
        let mut decided_disjoint_refs = Vec::new();
        let mut not_decided_disjoint_refs = Vec::new();
        let mut undecided_refs = Vec::new();

        for (candidate_pair_index, candidate_pair) in candidate_pairs.iter().enumerate() {
            let candidate_ref = ExactCurveArrangementSplitScheduleRef2 {
                candidate_pair_index,
            };
            match candidate_pair.aabb_status() {
                ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint => {
                    decided_disjoint_refs.push(candidate_ref)
                }
                ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint => {
                    not_decided_disjoint_refs.push(candidate_ref)
                }
                ExactCurveArrangementSplitCandidateAabbStatus2::Undecided => {
                    undecided_refs.push(candidate_ref)
                }
            }
        }

        let buckets = vec![
            ExactCurveArrangementSplitScheduleBucket2 {
                aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint,
                candidate_refs: decided_disjoint_refs,
            },
            ExactCurveArrangementSplitScheduleBucket2 {
                aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint,
                candidate_refs: not_decided_disjoint_refs,
            },
            ExactCurveArrangementSplitScheduleBucket2 {
                aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2::Undecided,
                candidate_refs: undecided_refs,
            },
        ];
        let candidate_ref_count = candidate_pairs.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.candidate_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            candidate_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of AABB-status buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of scheduled candidate references retained.
    pub const fn candidate_ref_count(&self) -> usize {
        self.candidate_ref_count
    }

    /// Returns the largest AABB-status bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns AABB-status buckets in stable status order.
    pub fn buckets(&self) -> &[ExactCurveArrangementSplitScheduleBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementSplitScheduleCache2 {
    /// Returns the total number of scheduled source segment pairs.
    pub const fn candidate_pair_count(&self) -> usize {
        self.candidate_pair_count
    }

    /// Returns scheduled pairs certified disjoint by retained source AABBs.
    pub const fn decided_disjoint_pair_count(&self) -> usize {
        self.decided_disjoint_pair_count
    }

    /// Returns scheduled pairs that require split predicate evaluation.
    pub const fn predicate_candidate_pair_count(&self) -> usize {
        self.predicate_candidate_pair_count
    }

    /// Returns scheduled pairs whose AABB pruning status stayed undecided.
    pub const fn undecided_aabb_pair_count(&self) -> usize {
        self.undecided_aabb_pair_count
    }

    /// Returns scheduled split candidate buckets grouped by retained AABB pruning status.
    pub const fn bucket_cache(&self) -> &ExactCurveArrangementSplitScheduleBucketCache2 {
        &self.bucket_cache
    }

    /// Returns scheduled source segment pairs in canonical `i < j` order.
    pub fn candidate_pairs(&self) -> &[ExactCurveArrangementSplitCandidatePair2] {
        &self.candidate_pairs
    }
}

impl ExactCurveArrangementSplitCache2 {
    fn from_arrangement_report(report: &RegionLineSegmentRegionBuildReport2) -> Self {
        let intersection_bucket_cache =
            split_intersection_bucket_cache(&report.split_intersection_reports);
        let intersection_parameter_cache =
            ExactCurveArrangementSplitIntersectionParameterCache2::from_intersection_reports(
                &report.split_intersection_reports,
            );
        let relation_bucket_cache = ExactCurveArrangementSplitRelationBucketCache2::from_counts(
            report.split_point_relation_count,
            report.split_overlap_relation_count,
            report.split_uncertain_relation_count,
        );
        let blocker_cache =
            ExactCurveArrangementSplitBlockerCache2::from_arrangement_report(report);
        Self {
            predicate_path: report.split_predicate_path,
            candidate_pair_count: report.split_candidate_pair_count,
            skipped_aabb_pair_count: report.split_skipped_aabb_pair_count,
            tested_pair_count: report.split_tested_pair_count,
            intersection_event_count: report.split_intersection_event_count,
            point_relation_count: report.split_point_relation_count,
            overlap_relation_count: report.split_overlap_relation_count,
            uncertain_relation_count: report.split_uncertain_relation_count,
            intersection_points: report.split_intersection_points.clone(),
            intersection_reports: report.split_intersection_reports.clone(),
            relation_bucket_cache,
            intersection_bucket_cache,
            intersection_parameter_cache,
            blocker_cache,
            output_segment_count: report.split_output_segment_count,
        }
    }

    /// Returns the exact predicate family used for source splitting.
    pub const fn predicate_path(&self) -> Option<RegionLineSegmentSplitPredicatePath2> {
        self.predicate_path
    }

    /// Returns source segment pairs considered by the split stage.
    pub const fn candidate_pair_count(&self) -> usize {
        self.candidate_pair_count
    }

    /// Returns source segment pairs skipped by certified AABB disjointness.
    pub const fn skipped_aabb_pair_count(&self) -> usize {
        self.skipped_aabb_pair_count
    }

    /// Returns source segment pairs tested by exact segment predicates.
    pub const fn tested_pair_count(&self) -> usize {
        self.tested_pair_count
    }

    /// Returns exact point-intersection event count found during splitting.
    pub const fn intersection_event_count(&self) -> usize {
        self.intersection_event_count
    }

    /// Returns source-pair relations classified as point intersections.
    pub const fn point_relation_count(&self) -> usize {
        self.point_relation_count
    }

    /// Returns source-pair relations classified as overlaps.
    pub const fn overlap_relation_count(&self) -> usize {
        self.overlap_relation_count
    }

    /// Returns source-pair relations that remained uncertain.
    pub const fn uncertain_relation_count(&self) -> usize {
        self.uncertain_relation_count
    }

    /// Returns exact intersection points retained by the split stage.
    pub fn intersection_points(&self) -> &[Point2] {
        &self.intersection_points
    }

    /// Returns exact per-event source and parameter evidence retained by the split stage.
    pub fn intersection_reports(&self) -> &[RegionLineSegmentSplitIntersectionReport2] {
        &self.intersection_reports
    }

    /// Returns retained split-stage relation buckets.
    pub const fn relation_bucket_cache(&self) -> &ExactCurveArrangementSplitRelationBucketCache2 {
        &self.relation_bucket_cache
    }

    /// Returns exact split-intersection point buckets derived from retained split reports.
    pub const fn intersection_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSplitIntersectionBucketCache2 {
        &self.intersection_bucket_cache
    }

    /// Returns exact source-parameter evidence for retained split intersections.
    pub const fn intersection_parameter_cache(
        &self,
    ) -> &ExactCurveArrangementSplitIntersectionParameterCache2 {
        &self.intersection_parameter_cache
    }

    /// Returns split-stage blocker source-pair evidence, when split arrangement blocked.
    pub const fn blocker_cache(&self) -> Option<&ExactCurveArrangementSplitBlockerCache2> {
        self.blocker_cache.as_ref()
    }

    /// Returns arranged output segment count when splitting completed.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }
}

impl ExactCurveArrangementSplitBlockerCache2 {
    fn from_arrangement_report(report: &RegionLineSegmentRegionBuildReport2) -> Option<Self> {
        Some(Self {
            first_source_segment_index: report.split_blocker_first_source_segment_index?,
            first_source_segment_kind: report.split_blocker_first_source_segment_kind?,
            first_source_start_point: report.split_blocker_first_source_start_point.clone()?,
            first_source_end_point: report.split_blocker_first_source_end_point.clone()?,
            second_source_segment_index: report.split_blocker_second_source_segment_index?,
            second_source_segment_kind: report.split_blocker_second_source_segment_kind?,
            second_source_start_point: report.split_blocker_second_source_start_point.clone()?,
            second_source_end_point: report.split_blocker_second_source_end_point.clone()?,
            blocker: report.blocker,
        })
    }

    /// Returns the first source segment index in the split blocker pair.
    pub const fn first_source_segment_index(&self) -> usize {
        self.first_source_segment_index
    }

    /// Returns the primitive family of the first blocked source segment.
    pub const fn first_source_segment_kind(&self) -> SegmentKind {
        self.first_source_segment_kind
    }

    /// Returns the exact start point of the first blocked source segment.
    pub const fn first_source_start_point(&self) -> &Point2 {
        &self.first_source_start_point
    }

    /// Returns the exact end point of the first blocked source segment.
    pub const fn first_source_end_point(&self) -> &Point2 {
        &self.first_source_end_point
    }

    /// Returns the second source segment index in the split blocker pair.
    pub const fn second_source_segment_index(&self) -> usize {
        self.second_source_segment_index
    }

    /// Returns the primitive family of the second blocked source segment.
    pub const fn second_source_segment_kind(&self) -> SegmentKind {
        self.second_source_segment_kind
    }

    /// Returns the exact start point of the second blocked source segment.
    pub const fn second_source_start_point(&self) -> &Point2 {
        &self.second_source_start_point
    }

    /// Returns the exact end point of the second blocked source segment.
    pub const fn second_source_end_point(&self) -> &Point2 {
        &self.second_source_end_point
    }

    /// Returns the retained blocker reason reported for this split blocker pair.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ExactCurveArrangementSplitRelationBucket2 {
    /// Returns the retained split-stage relation represented by this bucket.
    pub const fn relation(&self) -> ExactCurveArrangementSplitRelationClass2 {
        self.relation
    }

    /// Returns the number of split-stage relations in this bucket.
    pub const fn relation_count(&self) -> usize {
        self.relation_count
    }
}

impl ExactCurveArrangementSplitRelationBucketCache2 {
    fn from_counts(
        point_relation_count: usize,
        overlap_relation_count: usize,
        uncertain_relation_count: usize,
    ) -> Self {
        let buckets = vec![
            ExactCurveArrangementSplitRelationBucket2 {
                relation: ExactCurveArrangementSplitRelationClass2::Point,
                relation_count: point_relation_count,
            },
            ExactCurveArrangementSplitRelationBucket2 {
                relation: ExactCurveArrangementSplitRelationClass2::Overlap,
                relation_count: overlap_relation_count,
            },
            ExactCurveArrangementSplitRelationBucket2 {
                relation: ExactCurveArrangementSplitRelationClass2::Uncertain,
                relation_count: uncertain_relation_count,
            },
        ];
        let relation_count = point_relation_count
            .saturating_add(overlap_relation_count)
            .saturating_add(uncertain_relation_count);
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.relation_count)
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            relation_count,
            point_relation_count,
            overlap_relation_count,
            uncertain_relation_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained relation buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the total number of classified split-stage relations.
    pub const fn relation_count(&self) -> usize {
        self.relation_count
    }

    /// Returns source-pair relations classified as point intersections.
    pub const fn point_relation_count(&self) -> usize {
        self.point_relation_count
    }

    /// Returns source-pair relations classified as overlaps.
    pub const fn overlap_relation_count(&self) -> usize {
        self.overlap_relation_count
    }

    /// Returns source-pair relations that remained uncertain.
    pub const fn uncertain_relation_count(&self) -> usize {
        self.uncertain_relation_count
    }

    /// Returns the largest relation bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns split-stage relation buckets in stable relation order.
    pub fn buckets(&self) -> &[ExactCurveArrangementSplitRelationBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementSplitIntersectionRef2 {
    /// Returns the index into [`ExactCurveArrangementSplitCache2::intersection_reports`].
    pub const fn intersection_report_index(&self) -> usize {
        self.intersection_report_index
    }
}

impl ExactCurveArrangementSplitIntersectionBucket2 {
    /// Returns the exact structural point shared by this split-intersection bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns retained split-intersection report references in report order.
    pub fn intersections(&self) -> &[ExactCurveArrangementSplitIntersectionRef2] {
        &self.intersections
    }
}

impl ExactCurveArrangementSplitIntersectionBucketCache2 {
    /// Returns the number of retained split-intersection events bucketed.
    pub const fn intersection_event_count(&self) -> usize {
        self.intersection_event_count
    }

    /// Returns the number of exact structural split-intersection buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns split-intersection buckets containing one event.
    pub const fn singleton_bucket_count(&self) -> usize {
        self.singleton_bucket_count
    }

    /// Returns the largest split-intersection bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns exact structural split-intersection buckets in report encounter order.
    pub fn buckets(&self) -> &[ExactCurveArrangementSplitIntersectionBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementSplitIntersectionParameterRef2 {
    /// Returns the retained split-intersection report index.
    pub const fn intersection_report_index(&self) -> usize {
        self.intersection_report_index
    }

    /// Returns the first source segment index for this split event.
    pub const fn first_source_segment_index(&self) -> usize {
        self.first_source_segment_index
    }

    /// Returns the retained local parameter on the first source segment.
    pub const fn first_source_param(&self) -> &Real {
        &self.first_source_param
    }

    /// Returns the second source segment index for this split event.
    pub const fn second_source_segment_index(&self) -> usize {
        self.second_source_segment_index
    }

    /// Returns the retained local parameter on the second source segment.
    pub const fn second_source_param(&self) -> &Real {
        &self.second_source_param
    }

    /// Returns the exact point shared by both source parameters.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }
}

impl ExactCurveArrangementSplitIntersectionParameterCache2 {
    fn from_intersection_reports(
        intersection_reports: &[RegionLineSegmentSplitIntersectionReport2],
    ) -> Self {
        let mut parameters = Vec::new();

        for (intersection_report_index, report) in intersection_reports.iter().enumerate() {
            parameters.push(ExactCurveArrangementSplitIntersectionParameterRef2 {
                intersection_report_index,
                first_source_segment_index: report.first_source_segment_index(),
                first_source_param: report.first_source_param().clone(),
                second_source_segment_index: report.second_source_segment_index(),
                second_source_param: report.second_source_param().clone(),
                point: report.point().clone(),
            });
        }

        Self {
            intersection_event_count: parameters.len(),
            source_parameter_ref_count: parameters.len().saturating_mul(2),
            parameters,
        }
    }

    /// Returns the number of retained split-intersection events.
    pub const fn intersection_event_count(&self) -> usize {
        self.intersection_event_count
    }

    /// Returns the number of retained source parameter references.
    pub const fn source_parameter_ref_count(&self) -> usize {
        self.source_parameter_ref_count
    }

    /// Returns retained split-intersection source parameters in report order.
    pub fn parameters(&self) -> &[ExactCurveArrangementSplitIntersectionParameterRef2] {
        &self.parameters
    }
}

impl ExactCurveArrangementEndpointGraphCache2 {
    fn from_arrangement_report(report: &RegionLineSegmentRegionBuildReport2) -> Option<Self> {
        let endpoint_bucket_cache = arranged_endpoint_bucket_cache(&report.arranged_source_reports);
        let endpoint_side_bucket_cache =
            ExactCurveArrangementArrangedEndpointSideBucketCache2::from_arranged_source_reports(
                &report.arranged_source_reports,
            );
        let endpoint_point_cache =
            ExactCurveArrangementArrangedEndpointPointCache2::from_arranged_source_reports(
                &report.arranged_source_reports,
            );
        let endpoint_degree_bucket_cache =
            ExactCurveArrangementArrangedEndpointDegreeBucketCache2::from_endpoint_bucket_cache(
                &endpoint_bucket_cache,
            );
        Some(Self {
            predicate_path: report.endpoint_graph_predicate_path?,
            endpoint_count: report.endpoint_graph_endpoint_count?,
            structural_bucket_count: report.endpoint_graph_structural_bucket_count?,
            structural_singleton_bucket_count: report
                .endpoint_graph_structural_singleton_bucket_count?,
            max_structural_bucket_size: report.endpoint_graph_max_structural_bucket_size?,
            endpoint_bucket_cache,
            endpoint_side_bucket_cache,
            endpoint_point_cache,
            endpoint_degree_bucket_cache,
            dangling_endpoint_count: report.endpoint_graph_dangling_endpoint_count?,
            branch_endpoint_count: report.endpoint_graph_branch_endpoint_count?,
            blocker_arranged_segment_index: report.endpoint_graph_blocker_arranged_segment_index,
            blocker_endpoint: report.endpoint_graph_blocker_endpoint,
            blocker_point: report.endpoint_graph_blocker_point.clone(),
        })
    }

    /// Returns the exact predicate family used for endpoint graph validation.
    pub const fn predicate_path(&self) -> RegionLineSegmentEndpointGraphPredicatePath2 {
        self.predicate_path
    }

    /// Returns the number of arranged endpoints validated.
    pub const fn endpoint_count(&self) -> usize {
        self.endpoint_count
    }

    /// Returns exact structural endpoint bucket count.
    pub const fn structural_bucket_count(&self) -> usize {
        self.structural_bucket_count
    }

    /// Returns structural buckets containing one endpoint.
    pub const fn structural_singleton_bucket_count(&self) -> usize {
        self.structural_singleton_bucket_count
    }

    /// Returns the largest structural endpoint bucket size.
    pub const fn max_structural_bucket_size(&self) -> usize {
        self.max_structural_bucket_size
    }

    /// Returns exact arranged endpoint buckets derived from retained arranged source reports.
    pub const fn endpoint_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedEndpointBucketCache2 {
        &self.endpoint_bucket_cache
    }

    /// Returns arranged endpoints grouped by retained endpoint side.
    pub const fn endpoint_side_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedEndpointSideBucketCache2 {
        &self.endpoint_side_bucket_cache
    }

    /// Returns exact endpoint records for arranged fragments.
    pub const fn endpoint_point_cache(&self) -> &ExactCurveArrangementArrangedEndpointPointCache2 {
        &self.endpoint_point_cache
    }

    /// Returns structural arranged endpoints grouped by retained degree.
    pub const fn endpoint_degree_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedEndpointDegreeBucketCache2 {
        &self.endpoint_degree_bucket_cache
    }

    /// Returns dangling endpoint count found during validation.
    pub const fn dangling_endpoint_count(&self) -> usize {
        self.dangling_endpoint_count
    }

    /// Returns branch endpoint count found during validation.
    pub const fn branch_endpoint_count(&self) -> usize {
        self.branch_endpoint_count
    }

    /// Returns the blocker arranged segment index, when validation blocked.
    pub const fn blocker_arranged_segment_index(&self) -> Option<usize> {
        self.blocker_arranged_segment_index
    }

    /// Returns the blocker endpoint, when validation blocked.
    pub const fn blocker_endpoint(&self) -> Option<RegionLineSegmentArrangedEndpoint2> {
        self.blocker_endpoint
    }

    /// Returns the blocker point, when validation blocked.
    pub const fn blocker_point(&self) -> Option<&Point2> {
        self.blocker_point.as_ref()
    }
}

impl ExactCurveArrangementArrangedEndpointDegreeRef2 {
    /// Returns the index into [`ExactCurveArrangementArrangedEndpointBucketCache2::buckets`].
    pub const fn structural_bucket_index(&self) -> usize {
        self.structural_bucket_index
    }

    /// Returns the number of arranged endpoint references in the structural bucket.
    pub const fn endpoint_ref_count(&self) -> usize {
        self.endpoint_ref_count
    }

    /// Returns the exact structural endpoint point for this bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }
}

impl ExactCurveArrangementArrangedEndpointDegreeBucket2 {
    /// Returns the structural endpoint degree represented by this bucket.
    pub const fn degree(&self) -> ExactCurveArrangementArrangedEndpointDegree2 {
        self.degree
    }

    /// Returns structural endpoint buckets with this retained degree.
    pub fn endpoint_buckets(&self) -> &[ExactCurveArrangementArrangedEndpointDegreeRef2] {
        &self.endpoint_buckets
    }
}

impl ExactCurveArrangementArrangedEndpointDegreeBucketCache2 {
    fn from_endpoint_bucket_cache(
        endpoint_bucket_cache: &ExactCurveArrangementArrangedEndpointBucketCache2,
    ) -> Self {
        let mut dangling_refs = Vec::new();
        let mut chain_refs = Vec::new();
        let mut branch_refs = Vec::new();

        for (structural_bucket_index, bucket) in endpoint_bucket_cache.buckets().iter().enumerate()
        {
            let degree_ref = ExactCurveArrangementArrangedEndpointDegreeRef2 {
                structural_bucket_index,
                endpoint_ref_count: bucket.endpoints().len(),
                point: bucket.point().clone(),
            };
            match degree_ref.endpoint_ref_count {
                0 | 1 => dangling_refs.push(degree_ref),
                2 => chain_refs.push(degree_ref),
                _ => branch_refs.push(degree_ref),
            }
        }

        let dangling_structural_bucket_count = dangling_refs.len();
        let chain_structural_bucket_count = chain_refs.len();
        let branch_structural_bucket_count = branch_refs.len();
        let buckets = vec![
            ExactCurveArrangementArrangedEndpointDegreeBucket2 {
                degree: ExactCurveArrangementArrangedEndpointDegree2::Dangling,
                endpoint_buckets: dangling_refs,
            },
            ExactCurveArrangementArrangedEndpointDegreeBucket2 {
                degree: ExactCurveArrangementArrangedEndpointDegree2::Chain,
                endpoint_buckets: chain_refs,
            },
            ExactCurveArrangementArrangedEndpointDegreeBucket2 {
                degree: ExactCurveArrangementArrangedEndpointDegree2::Branch,
                endpoint_buckets: branch_refs,
            },
        ];
        let structural_bucket_ref_count = endpoint_bucket_cache.bucket_count();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.endpoint_buckets.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            structural_bucket_ref_count,
            dangling_structural_bucket_count,
            chain_structural_bucket_count,
            branch_structural_bucket_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained degree buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of structural endpoint buckets classified by degree.
    pub const fn structural_bucket_ref_count(&self) -> usize {
        self.structural_bucket_ref_count
    }

    /// Returns structural endpoint buckets with dangling degree.
    pub const fn dangling_structural_bucket_count(&self) -> usize {
        self.dangling_structural_bucket_count
    }

    /// Returns structural endpoint buckets with chain degree.
    pub const fn chain_structural_bucket_count(&self) -> usize {
        self.chain_structural_bucket_count
    }

    /// Returns structural endpoint buckets with branch degree.
    pub const fn branch_structural_bucket_count(&self) -> usize {
        self.branch_structural_bucket_count
    }

    /// Returns the largest structural-bucket count inside one degree bucket.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns endpoint degree buckets in stable dangling/chain/branch order.
    pub fn buckets(&self) -> &[ExactCurveArrangementArrangedEndpointDegreeBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementArrangedEndpointRef2 {
    /// Returns the arranged fragment index that owns this endpoint.
    pub const fn arranged_segment_index(&self) -> usize {
        self.arranged_segment_index
    }

    /// Returns which arranged fragment endpoint this reference points at.
    pub const fn endpoint(&self) -> RegionLineSegmentArrangedEndpoint2 {
        self.endpoint
    }
}

impl ExactCurveArrangementArrangedEndpointSideBucket2 {
    /// Returns the arranged endpoint side represented by this bucket.
    pub const fn endpoint(&self) -> RegionLineSegmentArrangedEndpoint2 {
        self.endpoint
    }

    /// Returns arranged endpoint references with this retained endpoint side.
    pub fn endpoints(&self) -> &[ExactCurveArrangementArrangedEndpointRef2] {
        &self.endpoints
    }
}

impl ExactCurveArrangementArrangedEndpointSideBucketCache2 {
    fn from_arranged_source_reports(
        arranged_source_reports: &[RegionLineSegmentArrangedSourceReport2],
    ) -> Self {
        let mut start_refs = Vec::new();
        let mut end_refs = Vec::new();

        for report in arranged_source_reports {
            let arranged_segment_index = report.arranged_segment_index();
            start_refs.push(ExactCurveArrangementArrangedEndpointRef2 {
                arranged_segment_index,
                endpoint: RegionLineSegmentArrangedEndpoint2::Start,
            });
            end_refs.push(ExactCurveArrangementArrangedEndpointRef2 {
                arranged_segment_index,
                endpoint: RegionLineSegmentArrangedEndpoint2::End,
            });
        }

        let start_endpoint_ref_count = start_refs.len();
        let end_endpoint_ref_count = end_refs.len();
        let buckets = vec![
            ExactCurveArrangementArrangedEndpointSideBucket2 {
                endpoint: RegionLineSegmentArrangedEndpoint2::Start,
                endpoints: start_refs,
            },
            ExactCurveArrangementArrangedEndpointSideBucket2 {
                endpoint: RegionLineSegmentArrangedEndpoint2::End,
                endpoints: end_refs,
            },
        ];
        let endpoint_ref_count = arranged_source_reports.len().saturating_mul(2);
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.endpoints.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            endpoint_ref_count,
            start_endpoint_ref_count,
            end_endpoint_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of endpoint-side buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained arranged endpoint references.
    pub const fn endpoint_ref_count(&self) -> usize {
        self.endpoint_ref_count
    }

    /// Returns the number of retained start endpoint references.
    pub const fn start_endpoint_ref_count(&self) -> usize {
        self.start_endpoint_ref_count
    }

    /// Returns the number of retained end endpoint references.
    pub const fn end_endpoint_ref_count(&self) -> usize {
        self.end_endpoint_ref_count
    }

    /// Returns the largest endpoint-side bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns endpoint-side buckets in stable start/end order.
    pub fn buckets(&self) -> &[ExactCurveArrangementArrangedEndpointSideBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementArrangedEndpointBucket2 {
    /// Returns the exact structural point shared by this arranged endpoint bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns arranged endpoints in retained report encounter order.
    pub fn endpoints(&self) -> &[ExactCurveArrangementArrangedEndpointRef2] {
        &self.endpoints
    }
}

impl ExactCurveArrangementArrangedEndpointBucketCache2 {
    /// Returns the number of arranged endpoints bucketed.
    pub const fn endpoint_count(&self) -> usize {
        self.endpoint_count
    }

    /// Returns the number of exact structural arranged endpoint buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns buckets containing one arranged endpoint.
    pub const fn singleton_bucket_count(&self) -> usize {
        self.singleton_bucket_count
    }

    /// Returns the largest arranged endpoint bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns exact structural arranged endpoint buckets in encounter order.
    pub fn buckets(&self) -> &[ExactCurveArrangementArrangedEndpointBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementArrangedEndpointPointRef2 {
    /// Returns the arranged fragment index after exact splitting.
    pub const fn arranged_segment_index(&self) -> usize {
        self.arranged_segment_index
    }

    /// Returns the exact arranged fragment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact arranged fragment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }
}

impl ExactCurveArrangementArrangedEndpointPointCache2 {
    fn from_arranged_source_reports(
        arranged_source_reports: &[RegionLineSegmentArrangedSourceReport2],
    ) -> Self {
        let mut endpoints: Vec<ExactCurveArrangementArrangedEndpointPointRef2> = Vec::new();

        for source_report in arranged_source_reports {
            let arranged_segment_index = source_report.arranged_segment_index();
            if endpoints
                .iter()
                .any(|endpoint| endpoint.arranged_segment_index == arranged_segment_index)
            {
                continue;
            }

            endpoints.push(ExactCurveArrangementArrangedEndpointPointRef2 {
                arranged_segment_index,
                output_start_point: source_report.output_start_point().clone(),
                output_end_point: source_report.output_end_point().clone(),
            });
        }

        endpoints.sort_by_key(|endpoint| endpoint.arranged_segment_index);

        Self {
            arranged_fragment_ref_count: endpoints.len(),
            endpoint_ref_count: endpoints.len().saturating_mul(2),
            endpoints,
        }
    }

    /// Returns the number of retained arranged fragment endpoint records.
    pub const fn arranged_fragment_ref_count(&self) -> usize {
        self.arranged_fragment_ref_count
    }

    /// Returns the number of retained arranged endpoint references.
    pub const fn endpoint_ref_count(&self) -> usize {
        self.endpoint_ref_count
    }

    /// Returns exact arranged endpoint records in arranged segment order.
    pub fn endpoints(&self) -> &[ExactCurveArrangementArrangedEndpointPointRef2] {
        &self.endpoints
    }
}

impl ExactCurveArrangementArrangedFragmentSourceRef2 {
    /// Returns the retained arranged source report index.
    pub const fn arranged_source_report_index(&self) -> usize {
        self.arranged_source_report_index
    }

    /// Returns the source segment index used by this arranged fragment.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the retained parameter range on the source segment.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns retained topology status for this source-to-fragment mapping.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl ExactCurveArrangementArrangedFragment2 {
    /// Returns the arranged fragment index after exact splitting.
    pub const fn arranged_segment_index(&self) -> usize {
        self.arranged_segment_index
    }

    /// Returns the primitive family of the arranged fragment.
    pub const fn arranged_segment_kind(&self) -> SegmentKind {
        self.arranged_segment_kind
    }

    /// Returns the arranged fragment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the arranged fragment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained source provenance references for this arranged fragment.
    pub fn source_refs(&self) -> &[ExactCurveArrangementArrangedFragmentSourceRef2] {
        &self.source_refs
    }
}

impl ExactCurveArrangementArrangedFragmentRef2 {
    /// Returns the index into [`ExactCurveArrangementArrangedFragmentCache2::fragments`].
    pub const fn arranged_fragment_index(&self) -> usize {
        self.arranged_fragment_index
    }
}

impl ExactCurveArrangementArrangedFragmentStatusRef2 {
    /// Returns the index into [`ExactCurveArrangementArrangedFragmentCache2::fragments`].
    pub const fn arranged_fragment_index(&self) -> usize {
        self.arranged_fragment_index
    }

    /// Returns the source reference index inside the retained arranged fragment.
    pub const fn source_ref_index(&self) -> usize {
        self.source_ref_index
    }

    /// Returns the retained arranged source report index.
    pub const fn arranged_source_report_index(&self) -> usize {
        self.arranged_source_report_index
    }
}

impl ExactCurveArrangementArrangedFragmentKindBucket2 {
    /// Returns the retained primitive family represented by this bucket.
    pub const fn arranged_segment_kind(&self) -> SegmentKind {
        self.arranged_segment_kind
    }

    /// Returns arranged fragment references with this retained primitive family.
    pub fn fragment_refs(&self) -> &[ExactCurveArrangementArrangedFragmentRef2] {
        &self.fragment_refs
    }
}

impl ExactCurveArrangementArrangedFragmentStatusBucket2 {
    /// Returns the retained topology status represented by this bucket.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns arranged fragment source references with this retained topology status.
    pub fn source_refs(&self) -> &[ExactCurveArrangementArrangedFragmentStatusRef2] {
        &self.source_refs
    }
}

impl ExactCurveArrangementArrangedFragmentKindBucketCache2 {
    fn from_fragments(fragments: &[ExactCurveArrangementArrangedFragment2]) -> Self {
        let mut line_refs = Vec::new();
        let mut arc_refs = Vec::new();

        for (arranged_fragment_index, fragment) in fragments.iter().enumerate() {
            let fragment_ref = ExactCurveArrangementArrangedFragmentRef2 {
                arranged_fragment_index,
            };
            match fragment.arranged_segment_kind() {
                SegmentKind::Line => line_refs.push(fragment_ref),
                SegmentKind::Arc => arc_refs.push(fragment_ref),
            }
        }

        let line_fragment_ref_count = line_refs.len();
        let arc_fragment_ref_count = arc_refs.len();
        let buckets = vec![
            ExactCurveArrangementArrangedFragmentKindBucket2 {
                arranged_segment_kind: SegmentKind::Line,
                fragment_refs: line_refs,
            },
            ExactCurveArrangementArrangedFragmentKindBucket2 {
                arranged_segment_kind: SegmentKind::Arc,
                fragment_refs: arc_refs,
            },
        ];
        let arranged_fragment_ref_count = fragments.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.fragment_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            arranged_fragment_ref_count,
            line_fragment_ref_count,
            arc_fragment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of primitive-family buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained arranged fragment references.
    pub const fn arranged_fragment_ref_count(&self) -> usize {
        self.arranged_fragment_ref_count
    }

    /// Returns the number of retained line fragment references.
    pub const fn line_fragment_ref_count(&self) -> usize {
        self.line_fragment_ref_count
    }

    /// Returns the number of retained arc fragment references.
    pub const fn arc_fragment_ref_count(&self) -> usize {
        self.arc_fragment_ref_count
    }

    /// Returns the largest primitive-family bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns arranged fragment primitive-family buckets in stable kind order.
    pub fn buckets(&self) -> &[ExactCurveArrangementArrangedFragmentKindBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementArrangedFragmentStatusBucketCache2 {
    fn from_fragments(fragments: &[ExactCurveArrangementArrangedFragment2]) -> Self {
        let mut native_exact_refs = Vec::new();
        let mut certified_approximation_refs = Vec::new();
        let mut display_or_export_refs = Vec::new();
        let mut imported_lossy_refs = Vec::new();
        let mut unsupported_refs = Vec::new();
        let mut unresolved_refs = Vec::new();

        for (arranged_fragment_index, fragment) in fragments.iter().enumerate() {
            for (source_ref_index, source_ref) in fragment.source_refs().iter().enumerate() {
                let status_ref = ExactCurveArrangementArrangedFragmentStatusRef2 {
                    arranged_fragment_index,
                    source_ref_index,
                    arranged_source_report_index: source_ref.arranged_source_report_index(),
                };
                match source_ref.status() {
                    RetainedTopologyStatus::NativeExact => native_exact_refs.push(status_ref),
                    RetainedTopologyStatus::CertifiedApproximation => {
                        certified_approximation_refs.push(status_ref)
                    }
                    RetainedTopologyStatus::DisplayOrExport => {
                        display_or_export_refs.push(status_ref)
                    }
                    RetainedTopologyStatus::ImportedLossy => imported_lossy_refs.push(status_ref),
                    RetainedTopologyStatus::Unsupported => unsupported_refs.push(status_ref),
                    RetainedTopologyStatus::Unresolved => unresolved_refs.push(status_ref),
                }
            }
        }

        let native_exact_ref_count = native_exact_refs.len();
        let certified_approximation_ref_count = certified_approximation_refs.len();
        let display_or_export_ref_count = display_or_export_refs.len();
        let imported_lossy_ref_count = imported_lossy_refs.len();
        let unsupported_ref_count = unsupported_refs.len();
        let unresolved_ref_count = unresolved_refs.len();
        let buckets = vec![
            ExactCurveArrangementArrangedFragmentStatusBucket2 {
                status: RetainedTopologyStatus::NativeExact,
                source_refs: native_exact_refs,
            },
            ExactCurveArrangementArrangedFragmentStatusBucket2 {
                status: RetainedTopologyStatus::CertifiedApproximation,
                source_refs: certified_approximation_refs,
            },
            ExactCurveArrangementArrangedFragmentStatusBucket2 {
                status: RetainedTopologyStatus::DisplayOrExport,
                source_refs: display_or_export_refs,
            },
            ExactCurveArrangementArrangedFragmentStatusBucket2 {
                status: RetainedTopologyStatus::ImportedLossy,
                source_refs: imported_lossy_refs,
            },
            ExactCurveArrangementArrangedFragmentStatusBucket2 {
                status: RetainedTopologyStatus::Unsupported,
                source_refs: unsupported_refs,
            },
            ExactCurveArrangementArrangedFragmentStatusBucket2 {
                status: RetainedTopologyStatus::Unresolved,
                source_refs: unresolved_refs,
            },
        ];
        let source_ref_count = fragments
            .iter()
            .map(|fragment| fragment.source_refs().len())
            .sum();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.source_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            source_ref_count,
            native_exact_ref_count,
            certified_approximation_ref_count,
            display_or_export_ref_count,
            imported_lossy_ref_count,
            unsupported_ref_count,
            unresolved_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained topology-status buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained arranged fragment source references.
    pub const fn source_ref_count(&self) -> usize {
        self.source_ref_count
    }

    /// Returns the number of native-exact source references.
    pub const fn native_exact_ref_count(&self) -> usize {
        self.native_exact_ref_count
    }

    /// Returns the number of certified-approximation source references.
    pub const fn certified_approximation_ref_count(&self) -> usize {
        self.certified_approximation_ref_count
    }

    /// Returns the number of display/export-only source references.
    pub const fn display_or_export_ref_count(&self) -> usize {
        self.display_or_export_ref_count
    }

    /// Returns the number of lossy-import source references.
    pub const fn imported_lossy_ref_count(&self) -> usize {
        self.imported_lossy_ref_count
    }

    /// Returns the number of unsupported source references.
    pub const fn unsupported_ref_count(&self) -> usize {
        self.unsupported_ref_count
    }

    /// Returns the number of unresolved source references.
    pub const fn unresolved_ref_count(&self) -> usize {
        self.unresolved_ref_count
    }

    /// Returns the largest topology-status bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns arranged fragment status buckets in stable status order.
    pub fn buckets(&self) -> &[ExactCurveArrangementArrangedFragmentStatusBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementArrangedFragmentSourceRangeRef2 {
    /// Returns the retained arranged source report index.
    pub const fn arranged_source_report_index(&self) -> usize {
        self.arranged_source_report_index
    }

    /// Returns the source segment index used by this arranged fragment.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the exact retained source parameter range.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the arranged fragment index after exact splitting.
    pub const fn arranged_segment_index(&self) -> usize {
        self.arranged_segment_index
    }

    /// Returns whether this arranged fragment covers the complete source parameter range.
    pub fn covers_full_source_range(&self) -> bool {
        source_range_is_full(&self.source_range)
    }
}

impl ExactCurveArrangementArrangedFragmentSourceRangeCache2 {
    fn from_arranged_source_reports(
        arranged_source_reports: &[RegionLineSegmentArrangedSourceReport2],
    ) -> Self {
        let mut full_source_range_ref_count = 0_usize;
        let mut partial_source_range_ref_count = 0_usize;
        let mut ranges = Vec::new();

        for (arranged_source_report_index, source_report) in
            arranged_source_reports.iter().enumerate()
        {
            if source_range_is_full(source_report.source_range()) {
                full_source_range_ref_count += 1;
            } else {
                partial_source_range_ref_count += 1;
            }

            ranges.push(ExactCurveArrangementArrangedFragmentSourceRangeRef2 {
                arranged_source_report_index,
                source_segment_index: source_report.source_segment_index(),
                source_range: source_report.source_range().clone(),
                arranged_segment_index: source_report.arranged_segment_index(),
            });
        }

        ranges.sort_by_key(|range_ref| {
            (
                range_ref.arranged_segment_index,
                range_ref.arranged_source_report_index,
            )
        });

        Self {
            source_ref_count: ranges.len(),
            full_source_range_ref_count,
            partial_source_range_ref_count,
            ranges,
        }
    }

    /// Returns the number of retained arranged fragment source range references.
    pub const fn source_ref_count(&self) -> usize {
        self.source_ref_count
    }

    /// Returns the number of arranged fragments covering a complete source segment.
    pub const fn full_source_range_ref_count(&self) -> usize {
        self.full_source_range_ref_count
    }

    /// Returns the number of arranged fragments covering a proper source subrange.
    pub const fn partial_source_range_ref_count(&self) -> usize {
        self.partial_source_range_ref_count
    }

    /// Returns arranged fragment source ranges in arranged fragment order.
    pub fn ranges(&self) -> &[ExactCurveArrangementArrangedFragmentSourceRangeRef2] {
        &self.ranges
    }
}

impl ExactCurveArrangementArrangedFragmentCache2 {
    fn from_arranged_source_reports(
        arranged_source_reports: &[RegionLineSegmentArrangedSourceReport2],
    ) -> Self {
        let mut fragments: Vec<ExactCurveArrangementArrangedFragment2> = Vec::new();
        let mut source_segment_kind_counts = SegmentKindCounts::default();

        for (arranged_source_report_index, report) in arranged_source_reports.iter().enumerate() {
            match report.source_segment_kind() {
                SegmentKind::Line => source_segment_kind_counts.lines += 1,
                SegmentKind::Arc => source_segment_kind_counts.arcs += 1,
            }

            let arranged_segment_index = report.arranged_segment_index();
            let fragment_index = fragments
                .iter()
                .position(|fragment| fragment.arranged_segment_index == arranged_segment_index)
                .unwrap_or_else(|| {
                    fragments.push(ExactCurveArrangementArrangedFragment2 {
                        arranged_segment_index,
                        arranged_segment_kind: report.arranged_segment_kind(),
                        output_start_point: report.output_start_point().clone(),
                        output_end_point: report.output_end_point().clone(),
                        source_refs: Vec::new(),
                    });
                    fragments.len() - 1
                });
            fragments[fragment_index].source_refs.push(
                ExactCurveArrangementArrangedFragmentSourceRef2 {
                    arranged_source_report_index,
                    source_segment_index: report.source_segment_index(),
                    source_segment_kind: report.source_segment_kind(),
                    source_range: report.source_range().clone(),
                    status: report.status(),
                },
            );
        }

        fragments.sort_by_key(|fragment| fragment.arranged_segment_index);
        for fragment in &mut fragments {
            fragment
                .source_refs
                .sort_by_key(|source_ref| source_ref.arranged_source_report_index);
        }

        let source_ref_count = arranged_source_reports.len();
        let arranged_segment_kind_counts =
            arranged_report_segment_kind_counts(arranged_source_reports);
        let arranged_fragment_kind_bucket_cache =
            ExactCurveArrangementArrangedFragmentKindBucketCache2::from_fragments(&fragments);
        let arranged_fragment_status_bucket_cache =
            ExactCurveArrangementArrangedFragmentStatusBucketCache2::from_fragments(&fragments);
        let arranged_fragment_source_range_cache =
            ExactCurveArrangementArrangedFragmentSourceRangeCache2::from_arranged_source_reports(
                arranged_source_reports,
            );
        let max_source_ref_count = fragments
            .iter()
            .map(|fragment| fragment.source_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            arranged_fragment_count: fragments.len(),
            source_ref_count,
            source_segment_kind_counts,
            arranged_segment_kind_counts,
            arranged_fragment_kind_bucket_cache,
            arranged_fragment_status_bucket_cache,
            arranged_fragment_source_range_cache,
            max_source_ref_count,
            fragments,
        }
    }

    /// Returns the number of arranged fragments retained.
    pub const fn arranged_fragment_count(&self) -> usize {
        self.arranged_fragment_count
    }

    /// Returns the number of retained source provenance references.
    pub const fn source_ref_count(&self) -> usize {
        self.source_ref_count
    }

    /// Returns source primitive-family counts for retained source references.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns arranged fragment primitive-family counts after exact splitting.
    pub const fn arranged_segment_kind_counts(&self) -> SegmentKindCounts {
        self.arranged_segment_kind_counts
    }

    /// Returns retained arranged fragment buckets grouped by primitive family.
    pub const fn arranged_fragment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentKindBucketCache2 {
        &self.arranged_fragment_kind_bucket_cache
    }

    /// Returns retained arranged fragment source buckets grouped by topology status.
    pub const fn arranged_fragment_status_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentStatusBucketCache2 {
        &self.arranged_fragment_status_bucket_cache
    }

    /// Returns retained arranged fragment source-parameter range records.
    pub const fn arranged_fragment_source_range_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentSourceRangeCache2 {
        &self.arranged_fragment_source_range_cache
    }

    /// Returns the largest source reference count for one arranged fragment.
    pub const fn max_source_ref_count(&self) -> usize {
        self.max_source_ref_count
    }

    /// Returns arranged fragments in arranged segment index order.
    pub fn fragments(&self) -> &[ExactCurveArrangementArrangedFragment2] {
        &self.fragments
    }
}

impl ExactCurveArrangementOutputRingSegmentRef2 {
    /// Returns the retained source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns whether the source segment was reversed for ring traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }
}

impl ExactCurveArrangementOutputRingBucket2 {
    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns output segment references in ring traversal order.
    pub fn segments(&self) -> &[ExactCurveArrangementOutputRingSegmentRef2] {
        &self.segments
    }
}

impl ExactCurveArrangementOutputRingBucketCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut rings: Vec<ExactCurveArrangementOutputRingBucket2> = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            let output_ring_index = source_report.output_ring_index();
            let ring_index = rings
                .iter()
                .position(|ring| ring.output_ring_index == output_ring_index)
                .unwrap_or_else(|| {
                    rings.push(ExactCurveArrangementOutputRingBucket2 {
                        output_ring_index,
                        segments: Vec::new(),
                    });
                    rings.len() - 1
                });
            rings[ring_index]
                .segments
                .push(ExactCurveArrangementOutputRingSegmentRef2 {
                    source_report_index,
                    output_segment_index: source_report.output_segment_index(),
                    reversed: source_report.reversed(),
                });
        }

        rings.sort_by_key(|ring| ring.output_ring_index);
        for ring in &mut rings {
            ring.segments
                .sort_by_key(|segment| segment.output_segment_index);
        }

        let segment_ref_count = source_reports.len();
        let max_ring_segment_count = rings
            .iter()
            .map(|ring| ring.segments.len())
            .max()
            .unwrap_or(0);

        Self {
            ring_count: rings.len(),
            segment_ref_count,
            max_ring_segment_count,
            rings,
        }
    }

    /// Returns the number of output rings retained.
    pub const fn ring_count(&self) -> usize {
        self.ring_count
    }

    /// Returns the number of output segment provenance references retained.
    pub const fn segment_ref_count(&self) -> usize {
        self.segment_ref_count
    }

    /// Returns the largest output ring segment count.
    pub const fn max_ring_segment_count(&self) -> usize {
        self.max_ring_segment_count
    }

    /// Returns output ring buckets in output ring index order.
    pub fn rings(&self) -> &[ExactCurveArrangementOutputRingBucket2] {
        &self.rings
    }
}

impl ExactCurveArrangementOutputSegmentKindRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }
}

impl ExactCurveArrangementOutputSegmentKindBucket2 {
    /// Returns the retained primitive family represented by this bucket.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns output segment references with this retained primitive family.
    pub fn segment_refs(&self) -> &[ExactCurveArrangementOutputSegmentKindRef2] {
        &self.segment_refs
    }
}

impl ExactCurveArrangementOutputSegmentKindBucketCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut line_refs = Vec::new();
        let mut arc_refs = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            let segment_ref = ExactCurveArrangementOutputSegmentKindRef2 {
                source_report_index,
                output_ring_index: source_report.output_ring_index(),
                output_segment_index: source_report.output_segment_index(),
            };
            match source_report.output_segment_kind() {
                SegmentKind::Line => line_refs.push(segment_ref),
                SegmentKind::Arc => arc_refs.push(segment_ref),
            }
        }

        let line_segment_ref_count = line_refs.len();
        let arc_segment_ref_count = arc_refs.len();
        let buckets = vec![
            ExactCurveArrangementOutputSegmentKindBucket2 {
                output_segment_kind: SegmentKind::Line,
                segment_refs: line_refs,
            },
            ExactCurveArrangementOutputSegmentKindBucket2 {
                output_segment_kind: SegmentKind::Arc,
                segment_refs: arc_refs,
            },
        ];
        let output_segment_ref_count = source_reports.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.segment_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            output_segment_ref_count,
            line_segment_ref_count,
            arc_segment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of primitive-family buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained output segment references.
    pub const fn output_segment_ref_count(&self) -> usize {
        self.output_segment_ref_count
    }

    /// Returns the number of retained line output segment references.
    pub const fn line_segment_ref_count(&self) -> usize {
        self.line_segment_ref_count
    }

    /// Returns the number of retained arc output segment references.
    pub const fn arc_segment_ref_count(&self) -> usize {
        self.arc_segment_ref_count
    }

    /// Returns the largest primitive-family bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns output segment primitive-family buckets in stable kind order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputSegmentKindBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputSegmentSourceRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }
}

impl ExactCurveArrangementOutputSegmentSourceBucket2 {
    /// Returns the retained source segment index represented by this bucket.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns output segment references emitted from this source segment.
    pub fn segment_refs(&self) -> &[ExactCurveArrangementOutputSegmentSourceRef2] {
        &self.segment_refs
    }
}

impl ExactCurveArrangementOutputSegmentSourceBucketCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut buckets: Vec<ExactCurveArrangementOutputSegmentSourceBucket2> = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            let source_segment_index = source_report.source_segment_index();
            let bucket_index = buckets
                .iter()
                .position(|bucket| bucket.source_segment_index == source_segment_index)
                .unwrap_or_else(|| {
                    buckets.push(ExactCurveArrangementOutputSegmentSourceBucket2 {
                        source_segment_index,
                        segment_refs: Vec::new(),
                    });
                    buckets.len() - 1
                });
            buckets[bucket_index]
                .segment_refs
                .push(ExactCurveArrangementOutputSegmentSourceRef2 {
                    source_report_index,
                    output_ring_index: source_report.output_ring_index(),
                    output_segment_index: source_report.output_segment_index(),
                });
        }

        buckets.sort_by_key(|bucket| bucket.source_segment_index);
        for bucket in &mut buckets {
            bucket.segment_refs.sort_by_key(|segment_ref| {
                (
                    segment_ref.output_ring_index,
                    segment_ref.output_segment_index,
                    segment_ref.source_report_index,
                )
            });
        }

        let output_segment_ref_count = source_reports.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.segment_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            source_segment_bucket_count: buckets.len(),
            output_segment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of source-segment buckets retained.
    pub const fn source_segment_bucket_count(&self) -> usize {
        self.source_segment_bucket_count
    }

    /// Returns the number of retained output segment references.
    pub const fn output_segment_ref_count(&self) -> usize {
        self.output_segment_ref_count
    }

    /// Returns the largest output segment count for one source segment.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns output segment source buckets in source segment index order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputSegmentSourceBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputSegmentSourceRangeRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the source segment index used by this output segment.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the exact retained source parameter range.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns whether this output segment covers the complete source parameter range.
    pub fn covers_full_source_range(&self) -> bool {
        source_range_is_full(&self.source_range)
    }
}

impl ExactCurveArrangementOutputSegmentSourceRangeCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut full_source_range_ref_count = 0_usize;
        let mut partial_source_range_ref_count = 0_usize;
        let mut ranges = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            if source_range_is_full(source_report.source_range()) {
                full_source_range_ref_count += 1;
            } else {
                partial_source_range_ref_count += 1;
            }

            ranges.push(ExactCurveArrangementOutputSegmentSourceRangeRef2 {
                source_report_index,
                source_segment_index: source_report.source_segment_index(),
                source_range: source_report.source_range().clone(),
                output_ring_index: source_report.output_ring_index(),
                output_segment_index: source_report.output_segment_index(),
            });
        }

        ranges.sort_by_key(|range_ref| {
            (
                range_ref.output_ring_index,
                range_ref.output_segment_index,
                range_ref.source_report_index,
            )
        });

        Self {
            output_segment_ref_count: ranges.len(),
            full_source_range_ref_count,
            partial_source_range_ref_count,
            ranges,
        }
    }

    /// Returns the number of retained output segment source range references.
    pub const fn output_segment_ref_count(&self) -> usize {
        self.output_segment_ref_count
    }

    /// Returns the number of output segments covering a complete source segment.
    pub const fn full_source_range_ref_count(&self) -> usize {
        self.full_source_range_ref_count
    }

    /// Returns the number of output segments covering a proper source subrange.
    pub const fn partial_source_range_ref_count(&self) -> usize {
        self.partial_source_range_ref_count
    }

    /// Returns output segment source ranges in output traversal order.
    pub fn ranges(&self) -> &[ExactCurveArrangementOutputSegmentSourceRangeRef2] {
        &self.ranges
    }
}

impl ExactCurveArrangementOutputSegmentEndpointRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the exact emitted segment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact emitted segment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }
}

impl ExactCurveArrangementOutputSegmentEndpointCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut segments = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            segments.push(ExactCurveArrangementOutputSegmentEndpointRef2 {
                source_report_index,
                output_ring_index: source_report.output_ring_index(),
                output_segment_index: source_report.output_segment_index(),
                output_start_point: source_report.output_start_point().clone(),
                output_end_point: source_report.output_end_point().clone(),
            });
        }

        segments.sort_by_key(|segment| {
            (
                segment.output_ring_index,
                segment.output_segment_index,
                segment.source_report_index,
            )
        });

        Self {
            output_segment_ref_count: segments.len(),
            output_endpoint_ref_count: segments.len().saturating_mul(2),
            segments,
        }
    }

    /// Returns the number of retained output segment endpoint records.
    pub const fn output_segment_ref_count(&self) -> usize {
        self.output_segment_ref_count
    }

    /// Returns the number of retained output endpoint references.
    pub const fn output_endpoint_ref_count(&self) -> usize {
        self.output_endpoint_ref_count
    }

    /// Returns output segment endpoint records in output traversal order.
    pub fn segments(&self) -> &[ExactCurveArrangementOutputSegmentEndpointRef2] {
        &self.segments
    }
}

impl ExactCurveArrangementOutputRingContinuityRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the retained ring source report index for the next output segment.
    pub const fn next_source_report_index(&self) -> usize {
        self.next_source_report_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the following output segment index inside the same ring.
    pub const fn next_output_segment_index(&self) -> usize {
        self.next_output_segment_index
    }

    /// Returns the exact end point of the current output segment.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns the exact start point of the following output segment.
    pub const fn next_output_start_point(&self) -> &Point2 {
        &self.next_output_start_point
    }
}

impl ExactCurveArrangementOutputRingContinuityCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut rings: Vec<Vec<(usize, &RegionLineSegmentRingSourceReport2)>> = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            let output_ring_index = source_report.output_ring_index();
            let ring_index = rings
                .iter()
                .position(|ring| {
                    ring.first()
                        .is_some_and(|(_, first)| first.output_ring_index() == output_ring_index)
                })
                .unwrap_or_else(|| {
                    rings.push(Vec::new());
                    rings.len() - 1
                });
            rings[ring_index].push((source_report_index, source_report));
        }

        for ring in &mut rings {
            ring.sort_by_key(|(_, source_report)| source_report.output_segment_index());
        }
        rings.sort_by_key(|ring| {
            ring.first().map_or(usize::MAX, |(_, source_report)| {
                source_report.output_ring_index()
            })
        });

        let output_ring_ref_count = rings.len();
        let max_ring_connection_count = rings.iter().map(Vec::len).max().unwrap_or(0);
        let mut connections = Vec::new();

        for ring in rings {
            for (segment_index, (source_report_index, source_report)) in ring.iter().enumerate() {
                let (next_source_report_index, next_source_report) =
                    &ring[(segment_index + 1) % ring.len()];
                connections.push(ExactCurveArrangementOutputRingContinuityRef2 {
                    source_report_index: *source_report_index,
                    next_source_report_index: *next_source_report_index,
                    output_ring_index: source_report.output_ring_index(),
                    output_segment_index: source_report.output_segment_index(),
                    next_output_segment_index: next_source_report.output_segment_index(),
                    output_end_point: source_report.output_end_point().clone(),
                    next_output_start_point: next_source_report.output_start_point().clone(),
                });
            }
        }

        Self {
            output_ring_ref_count,
            output_connection_ref_count: connections.len(),
            max_ring_connection_count,
            connections,
        }
    }

    /// Returns the number of output rings with retained continuity evidence.
    pub const fn output_ring_ref_count(&self) -> usize {
        self.output_ring_ref_count
    }

    /// Returns the number of retained segment-to-next-segment connections.
    pub const fn output_connection_ref_count(&self) -> usize {
        self.output_connection_ref_count
    }

    /// Returns the largest retained connection count for one output ring.
    pub const fn max_ring_connection_count(&self) -> usize {
        self.max_ring_connection_count
    }

    /// Returns output ring continuity records in output traversal order.
    pub fn connections(&self) -> &[ExactCurveArrangementOutputRingContinuityRef2] {
        &self.connections
    }
}

impl ExactCurveArrangementOutputSegmentStatusRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }
}

impl ExactCurveArrangementOutputSegmentStatusBucket2 {
    /// Returns the retained topology status represented by this bucket.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns output segment references with this retained topology status.
    pub fn segment_refs(&self) -> &[ExactCurveArrangementOutputSegmentStatusRef2] {
        &self.segment_refs
    }
}

impl ExactCurveArrangementOutputSegmentStatusBucketCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut native_exact_refs = Vec::new();
        let mut certified_approximation_refs = Vec::new();
        let mut display_or_export_refs = Vec::new();
        let mut imported_lossy_refs = Vec::new();
        let mut unsupported_refs = Vec::new();
        let mut unresolved_refs = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            let segment_ref = ExactCurveArrangementOutputSegmentStatusRef2 {
                source_report_index,
                output_ring_index: source_report.output_ring_index(),
                output_segment_index: source_report.output_segment_index(),
            };
            match source_report.status() {
                RetainedTopologyStatus::NativeExact => native_exact_refs.push(segment_ref),
                RetainedTopologyStatus::CertifiedApproximation => {
                    certified_approximation_refs.push(segment_ref)
                }
                RetainedTopologyStatus::DisplayOrExport => display_or_export_refs.push(segment_ref),
                RetainedTopologyStatus::ImportedLossy => imported_lossy_refs.push(segment_ref),
                RetainedTopologyStatus::Unsupported => unsupported_refs.push(segment_ref),
                RetainedTopologyStatus::Unresolved => unresolved_refs.push(segment_ref),
            }
        }

        let native_exact_ref_count = native_exact_refs.len();
        let certified_approximation_ref_count = certified_approximation_refs.len();
        let display_or_export_ref_count = display_or_export_refs.len();
        let imported_lossy_ref_count = imported_lossy_refs.len();
        let unsupported_ref_count = unsupported_refs.len();
        let unresolved_ref_count = unresolved_refs.len();
        let buckets = vec![
            ExactCurveArrangementOutputSegmentStatusBucket2 {
                status: RetainedTopologyStatus::NativeExact,
                segment_refs: native_exact_refs,
            },
            ExactCurveArrangementOutputSegmentStatusBucket2 {
                status: RetainedTopologyStatus::CertifiedApproximation,
                segment_refs: certified_approximation_refs,
            },
            ExactCurveArrangementOutputSegmentStatusBucket2 {
                status: RetainedTopologyStatus::DisplayOrExport,
                segment_refs: display_or_export_refs,
            },
            ExactCurveArrangementOutputSegmentStatusBucket2 {
                status: RetainedTopologyStatus::ImportedLossy,
                segment_refs: imported_lossy_refs,
            },
            ExactCurveArrangementOutputSegmentStatusBucket2 {
                status: RetainedTopologyStatus::Unsupported,
                segment_refs: unsupported_refs,
            },
            ExactCurveArrangementOutputSegmentStatusBucket2 {
                status: RetainedTopologyStatus::Unresolved,
                segment_refs: unresolved_refs,
            },
        ];
        let output_segment_ref_count = source_reports.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.segment_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            output_segment_ref_count,
            native_exact_ref_count,
            certified_approximation_ref_count,
            display_or_export_ref_count,
            imported_lossy_ref_count,
            unsupported_ref_count,
            unresolved_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained topology-status buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained output segment references.
    pub const fn output_segment_ref_count(&self) -> usize {
        self.output_segment_ref_count
    }

    /// Returns the number of native-exact output segment references.
    pub const fn native_exact_ref_count(&self) -> usize {
        self.native_exact_ref_count
    }

    /// Returns the number of certified-approximation output segment references.
    pub const fn certified_approximation_ref_count(&self) -> usize {
        self.certified_approximation_ref_count
    }

    /// Returns the number of display/export-only output segment references.
    pub const fn display_or_export_ref_count(&self) -> usize {
        self.display_or_export_ref_count
    }

    /// Returns the number of lossy-import output segment references.
    pub const fn imported_lossy_ref_count(&self) -> usize {
        self.imported_lossy_ref_count
    }

    /// Returns the number of unsupported output segment references.
    pub const fn unsupported_ref_count(&self) -> usize {
        self.unsupported_ref_count
    }

    /// Returns the number of unresolved output segment references.
    pub const fn unresolved_ref_count(&self) -> usize {
        self.unresolved_ref_count
    }

    /// Returns the largest topology-status bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns output segment status buckets in stable status order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputSegmentStatusBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputSegmentDirectionRef2 {
    /// Returns the retained ring source report index for this output segment.
    pub const fn source_report_index(&self) -> usize {
        self.source_report_index
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside its ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }
}

impl ExactCurveArrangementOutputSegmentDirectionBucket2 {
    /// Returns whether this bucket contains source segments reversed for ring traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns output segment references with this retained traversal direction.
    pub fn segment_refs(&self) -> &[ExactCurveArrangementOutputSegmentDirectionRef2] {
        &self.segment_refs
    }
}

impl ExactCurveArrangementOutputSegmentDirectionBucketCache2 {
    fn from_source_reports(source_reports: &[RegionLineSegmentRingSourceReport2]) -> Self {
        let mut forward_refs = Vec::new();
        let mut reversed_refs = Vec::new();

        for (source_report_index, source_report) in source_reports.iter().enumerate() {
            let segment_ref = ExactCurveArrangementOutputSegmentDirectionRef2 {
                source_report_index,
                output_ring_index: source_report.output_ring_index(),
                output_segment_index: source_report.output_segment_index(),
            };
            if source_report.reversed() {
                reversed_refs.push(segment_ref);
            } else {
                forward_refs.push(segment_ref);
            }
        }

        let forward_segment_ref_count = forward_refs.len();
        let reversed_segment_ref_count = reversed_refs.len();
        let buckets = vec![
            ExactCurveArrangementOutputSegmentDirectionBucket2 {
                reversed: false,
                segment_refs: forward_refs,
            },
            ExactCurveArrangementOutputSegmentDirectionBucket2 {
                reversed: true,
                segment_refs: reversed_refs,
            },
        ];
        let output_segment_ref_count = source_reports.len();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.segment_refs.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            output_segment_ref_count,
            forward_segment_ref_count,
            reversed_segment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of traversal-direction buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained output segment references.
    pub const fn output_segment_ref_count(&self) -> usize {
        self.output_segment_ref_count
    }

    /// Returns the number of output segment references emitted in source direction.
    pub const fn forward_segment_ref_count(&self) -> usize {
        self.forward_segment_ref_count
    }

    /// Returns the number of output segment references emitted in reversed source direction.
    pub const fn reversed_segment_ref_count(&self) -> usize {
        self.reversed_segment_ref_count
    }

    /// Returns the largest traversal-direction bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns output segment direction buckets in stable forward/reversed order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputSegmentDirectionBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputRoleAssignment2 {
    /// Returns the retained boundary role report index.
    pub const fn role_report_index(&self) -> usize {
        self.role_report_index
    }

    /// Returns the source contour index assigned by this report.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns the source contour segment count captured before role binning.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the source contour fill rule captured before role binning.
    pub const fn source_fill_rule(&self) -> FillRule {
        self.source_fill_rule
    }

    /// Returns the exact source point used for containment classification.
    pub const fn nesting_sample_point(&self) -> &Point2 {
        &self.nesting_sample_point
    }

    /// Returns source contour indices that exactly contained the sample point.
    pub fn containing_contour_indices(&self) -> &[usize] {
        &self.containing_contour_indices
    }

    /// Returns exact containment depth used for material/hole parity.
    pub const fn nesting_depth(&self) -> usize {
        self.nesting_depth
    }

    /// Returns this contour's index inside its output role bin.
    pub const fn output_role_index(&self) -> usize {
        self.output_role_index
    }

    /// Returns retained topology status for this role assignment.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl ExactCurveArrangementOutputRoleStatusRef2 {
    /// Returns the role bucket containing this assignment.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns the assignment index inside the retained role bucket.
    pub const fn assignment_index(&self) -> usize {
        self.assignment_index
    }

    /// Returns the retained boundary role report index.
    pub const fn role_report_index(&self) -> usize {
        self.role_report_index
    }
}

impl ExactCurveArrangementOutputRoleBucket2 {
    /// Returns the material/hole role represented by this bucket.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns role assignments in output role index order.
    pub fn assignments(&self) -> &[ExactCurveArrangementOutputRoleAssignment2] {
        &self.assignments
    }
}

impl ExactCurveArrangementOutputRoleStatusBucket2 {
    /// Returns the retained topology status represented by this bucket.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns output role assignment references with this retained topology status.
    pub fn assignments(&self) -> &[ExactCurveArrangementOutputRoleStatusRef2] {
        &self.assignments
    }
}

impl ExactCurveArrangementOutputRoleStatusBucketCache2 {
    fn from_role_assignments(
        material_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
        hole_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
    ) -> Self {
        let mut native_exact_refs = Vec::new();
        let mut certified_approximation_refs = Vec::new();
        let mut display_or_export_refs = Vec::new();
        let mut imported_lossy_refs = Vec::new();
        let mut unsupported_refs = Vec::new();
        let mut unresolved_refs = Vec::new();

        for (role, assignments) in [
            (RegionBoundaryContourRole2::Material, material_assignments),
            (RegionBoundaryContourRole2::Hole, hole_assignments),
        ] {
            for (assignment_index, assignment) in assignments.iter().enumerate() {
                let status_ref = ExactCurveArrangementOutputRoleStatusRef2 {
                    role,
                    assignment_index,
                    role_report_index: assignment.role_report_index(),
                };
                match assignment.status() {
                    RetainedTopologyStatus::NativeExact => native_exact_refs.push(status_ref),
                    RetainedTopologyStatus::CertifiedApproximation => {
                        certified_approximation_refs.push(status_ref)
                    }
                    RetainedTopologyStatus::DisplayOrExport => {
                        display_or_export_refs.push(status_ref)
                    }
                    RetainedTopologyStatus::ImportedLossy => imported_lossy_refs.push(status_ref),
                    RetainedTopologyStatus::Unsupported => unsupported_refs.push(status_ref),
                    RetainedTopologyStatus::Unresolved => unresolved_refs.push(status_ref),
                }
            }
        }

        let native_exact_ref_count = native_exact_refs.len();
        let certified_approximation_ref_count = certified_approximation_refs.len();
        let display_or_export_ref_count = display_or_export_refs.len();
        let imported_lossy_ref_count = imported_lossy_refs.len();
        let unsupported_ref_count = unsupported_refs.len();
        let unresolved_ref_count = unresolved_refs.len();
        let buckets = vec![
            ExactCurveArrangementOutputRoleStatusBucket2 {
                status: RetainedTopologyStatus::NativeExact,
                assignments: native_exact_refs,
            },
            ExactCurveArrangementOutputRoleStatusBucket2 {
                status: RetainedTopologyStatus::CertifiedApproximation,
                assignments: certified_approximation_refs,
            },
            ExactCurveArrangementOutputRoleStatusBucket2 {
                status: RetainedTopologyStatus::DisplayOrExport,
                assignments: display_or_export_refs,
            },
            ExactCurveArrangementOutputRoleStatusBucket2 {
                status: RetainedTopologyStatus::ImportedLossy,
                assignments: imported_lossy_refs,
            },
            ExactCurveArrangementOutputRoleStatusBucket2 {
                status: RetainedTopologyStatus::Unsupported,
                assignments: unsupported_refs,
            },
            ExactCurveArrangementOutputRoleStatusBucket2 {
                status: RetainedTopologyStatus::Unresolved,
                assignments: unresolved_refs,
            },
        ];
        let assignment_ref_count = material_assignments
            .len()
            .saturating_add(hole_assignments.len());
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.assignments.len())
            .max()
            .unwrap_or(0);

        Self {
            bucket_count: buckets.len(),
            assignment_ref_count,
            native_exact_ref_count,
            certified_approximation_ref_count,
            display_or_export_ref_count,
            imported_lossy_ref_count,
            unsupported_ref_count,
            unresolved_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained topology-status buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns the number of retained output role assignment references.
    pub const fn assignment_ref_count(&self) -> usize {
        self.assignment_ref_count
    }

    /// Returns the number of native-exact role assignment references.
    pub const fn native_exact_ref_count(&self) -> usize {
        self.native_exact_ref_count
    }

    /// Returns the number of certified-approximation role assignment references.
    pub const fn certified_approximation_ref_count(&self) -> usize {
        self.certified_approximation_ref_count
    }

    /// Returns the number of display/export-only role assignment references.
    pub const fn display_or_export_ref_count(&self) -> usize {
        self.display_or_export_ref_count
    }

    /// Returns the number of lossy-import role assignment references.
    pub const fn imported_lossy_ref_count(&self) -> usize {
        self.imported_lossy_ref_count
    }

    /// Returns the number of unsupported role assignment references.
    pub const fn unsupported_ref_count(&self) -> usize {
        self.unsupported_ref_count
    }

    /// Returns the number of unresolved role assignment references.
    pub const fn unresolved_ref_count(&self) -> usize {
        self.unresolved_ref_count
    }

    /// Returns the largest topology-status bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns output role assignment status buckets in stable status order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputRoleStatusBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputRoleSourceContourRef2 {
    /// Returns the role bucket containing this assignment.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns the assignment index inside the retained role bucket.
    pub const fn assignment_index(&self) -> usize {
        self.assignment_index
    }

    /// Returns the retained boundary role report index.
    pub const fn role_report_index(&self) -> usize {
        self.role_report_index
    }

    /// Returns this contour's index inside its output role bin.
    pub const fn output_role_index(&self) -> usize {
        self.output_role_index
    }
}

impl ExactCurveArrangementOutputRoleSourceContourBucket2 {
    /// Returns the retained source contour index represented by this bucket.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns retained role assignments emitted from this source contour.
    pub fn assignments(&self) -> &[ExactCurveArrangementOutputRoleSourceContourRef2] {
        &self.assignments
    }
}

impl ExactCurveArrangementOutputRoleSourceContourBucketCache2 {
    fn from_role_assignments(
        material_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
        hole_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
    ) -> Self {
        let mut buckets: Vec<ExactCurveArrangementOutputRoleSourceContourBucket2> = Vec::new();

        for (role, assignments) in [
            (RegionBoundaryContourRole2::Material, material_assignments),
            (RegionBoundaryContourRole2::Hole, hole_assignments),
        ] {
            for (assignment_index, assignment) in assignments.iter().enumerate() {
                let source_contour_index = assignment.source_contour_index();
                let bucket_index = buckets
                    .iter()
                    .position(|bucket| bucket.source_contour_index == source_contour_index)
                    .unwrap_or_else(|| {
                        buckets.push(ExactCurveArrangementOutputRoleSourceContourBucket2 {
                            source_contour_index,
                            assignments: Vec::new(),
                        });
                        buckets.len() - 1
                    });
                buckets[bucket_index].assignments.push(
                    ExactCurveArrangementOutputRoleSourceContourRef2 {
                        role,
                        assignment_index,
                        role_report_index: assignment.role_report_index(),
                        output_role_index: assignment.output_role_index(),
                    },
                );
            }
        }

        buckets.sort_by_key(|bucket| bucket.source_contour_index);
        for bucket in &mut buckets {
            bucket.assignments.sort_by_key(|assignment| {
                (
                    assignment.role_report_index,
                    assignment.output_role_index,
                    assignment.assignment_index,
                )
            });
        }

        let assignment_ref_count = material_assignments
            .len()
            .saturating_add(hole_assignments.len());
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.assignments.len())
            .max()
            .unwrap_or(0);

        Self {
            source_contour_bucket_count: buckets.len(),
            assignment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained source-contour buckets.
    pub const fn source_contour_bucket_count(&self) -> usize {
        self.source_contour_bucket_count
    }

    /// Returns the number of retained output role assignment references.
    pub const fn assignment_ref_count(&self) -> usize {
        self.assignment_ref_count
    }

    /// Returns the largest assignment count for one source contour.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns source-contour buckets in source contour index order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputRoleSourceContourBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputRoleNestingDepthRef2 {
    /// Returns the role bucket containing this assignment.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns the assignment index inside the retained role bucket.
    pub const fn assignment_index(&self) -> usize {
        self.assignment_index
    }

    /// Returns the retained boundary role report index.
    pub const fn role_report_index(&self) -> usize {
        self.role_report_index
    }

    /// Returns the source contour index assigned by this report.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns this contour's index inside its output role bin.
    pub const fn output_role_index(&self) -> usize {
        self.output_role_index
    }
}

impl ExactCurveArrangementOutputRoleNestingDepthBucket2 {
    /// Returns the exact containment depth represented by this bucket.
    pub const fn nesting_depth(&self) -> usize {
        self.nesting_depth
    }

    /// Returns retained role assignments with this exact nesting depth.
    pub fn assignments(&self) -> &[ExactCurveArrangementOutputRoleNestingDepthRef2] {
        &self.assignments
    }
}

impl ExactCurveArrangementOutputRoleNestingDepthBucketCache2 {
    fn from_role_assignments(
        material_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
        hole_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
    ) -> Self {
        let mut buckets: Vec<ExactCurveArrangementOutputRoleNestingDepthBucket2> = Vec::new();

        for (role, assignments) in [
            (RegionBoundaryContourRole2::Material, material_assignments),
            (RegionBoundaryContourRole2::Hole, hole_assignments),
        ] {
            for (assignment_index, assignment) in assignments.iter().enumerate() {
                let nesting_depth = assignment.nesting_depth();
                let bucket_index = buckets
                    .iter()
                    .position(|bucket| bucket.nesting_depth == nesting_depth)
                    .unwrap_or_else(|| {
                        buckets.push(ExactCurveArrangementOutputRoleNestingDepthBucket2 {
                            nesting_depth,
                            assignments: Vec::new(),
                        });
                        buckets.len() - 1
                    });
                buckets[bucket_index].assignments.push(
                    ExactCurveArrangementOutputRoleNestingDepthRef2 {
                        role,
                        assignment_index,
                        role_report_index: assignment.role_report_index(),
                        source_contour_index: assignment.source_contour_index(),
                        output_role_index: assignment.output_role_index(),
                    },
                );
            }
        }

        buckets.sort_by_key(|bucket| bucket.nesting_depth);
        for bucket in &mut buckets {
            bucket.assignments.sort_by_key(|assignment| {
                (
                    assignment.role_report_index,
                    assignment.source_contour_index,
                    assignment.output_role_index,
                    assignment.assignment_index,
                )
            });
        }

        let assignment_ref_count = material_assignments
            .len()
            .saturating_add(hole_assignments.len());
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.assignments.len())
            .max()
            .unwrap_or(0);

        Self {
            nesting_depth_bucket_count: buckets.len(),
            assignment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained nesting-depth buckets.
    pub const fn nesting_depth_bucket_count(&self) -> usize {
        self.nesting_depth_bucket_count
    }

    /// Returns the number of retained output role assignment references.
    pub const fn assignment_ref_count(&self) -> usize {
        self.assignment_ref_count
    }

    /// Returns the largest assignment count for one nesting depth.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns nesting-depth buckets in increasing depth order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputRoleNestingDepthBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputRoleContainmentRef2 {
    /// Returns the role bucket containing this assignment.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns the assignment index inside the retained role bucket.
    pub const fn assignment_index(&self) -> usize {
        self.assignment_index
    }

    /// Returns the retained boundary role report index.
    pub const fn role_report_index(&self) -> usize {
        self.role_report_index
    }

    /// Returns the source contour whose nesting evidence is retained.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns the exact source contour that contained the assignment sample point.
    pub const fn containing_contour_index(&self) -> usize {
        self.containing_contour_index
    }

    /// Returns the index inside the assignment's retained containing-contour list.
    pub const fn containing_contour_ref_index(&self) -> usize {
        self.containing_contour_ref_index
    }

    /// Returns this contour's index inside its output role bin.
    pub const fn output_role_index(&self) -> usize {
        self.output_role_index
    }
}

impl ExactCurveArrangementOutputRoleContainmentBucket2 {
    /// Returns the retained source contour that contains these assignments.
    pub const fn containing_contour_index(&self) -> usize {
        self.containing_contour_index
    }

    /// Returns retained containment references for this containing contour.
    pub fn containments(&self) -> &[ExactCurveArrangementOutputRoleContainmentRef2] {
        &self.containments
    }
}

impl ExactCurveArrangementOutputRoleContainmentBucketCache2 {
    fn from_role_assignments(
        material_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
        hole_assignments: &[ExactCurveArrangementOutputRoleAssignment2],
    ) -> Self {
        let mut buckets: Vec<ExactCurveArrangementOutputRoleContainmentBucket2> = Vec::new();
        let mut uncontained_assignment_ref_count = 0_usize;

        for (role, assignments) in [
            (RegionBoundaryContourRole2::Material, material_assignments),
            (RegionBoundaryContourRole2::Hole, hole_assignments),
        ] {
            for (assignment_index, assignment) in assignments.iter().enumerate() {
                if assignment.containing_contour_indices().is_empty() {
                    uncontained_assignment_ref_count += 1;
                    continue;
                }

                for (containing_contour_ref_index, containing_contour_index) in
                    assignment.containing_contour_indices().iter().enumerate()
                {
                    let bucket_index = buckets
                        .iter()
                        .position(|bucket| {
                            bucket.containing_contour_index == *containing_contour_index
                        })
                        .unwrap_or_else(|| {
                            buckets.push(ExactCurveArrangementOutputRoleContainmentBucket2 {
                                containing_contour_index: *containing_contour_index,
                                containments: Vec::new(),
                            });
                            buckets.len() - 1
                        });
                    buckets[bucket_index].containments.push(
                        ExactCurveArrangementOutputRoleContainmentRef2 {
                            role,
                            assignment_index,
                            role_report_index: assignment.role_report_index(),
                            source_contour_index: assignment.source_contour_index(),
                            containing_contour_index: *containing_contour_index,
                            containing_contour_ref_index,
                            output_role_index: assignment.output_role_index(),
                        },
                    );
                }
            }
        }

        buckets.sort_by_key(|bucket| bucket.containing_contour_index);
        for bucket in &mut buckets {
            bucket.containments.sort_by_key(|containment| {
                (
                    containment.role_report_index,
                    containment.source_contour_index,
                    containment.containing_contour_ref_index,
                    containment.output_role_index,
                    containment.assignment_index,
                )
            });
        }

        let containment_ref_count = buckets.iter().map(|bucket| bucket.containments.len()).sum();
        let max_bucket_size = buckets
            .iter()
            .map(|bucket| bucket.containments.len())
            .max()
            .unwrap_or(0);

        Self {
            containing_contour_bucket_count: buckets.len(),
            containment_ref_count,
            uncontained_assignment_ref_count,
            max_bucket_size,
            buckets,
        }
    }

    /// Returns the number of retained containing-contour buckets.
    pub const fn containing_contour_bucket_count(&self) -> usize {
        self.containing_contour_bucket_count
    }

    /// Returns the number of retained containment references.
    pub const fn containment_ref_count(&self) -> usize {
        self.containment_ref_count
    }

    /// Returns assignments whose retained containing-contour list is empty.
    pub const fn uncontained_assignment_ref_count(&self) -> usize {
        self.uncontained_assignment_ref_count
    }

    /// Returns the largest containment count for one containing contour.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }

    /// Returns containment buckets in containing source contour order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputRoleContainmentBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputRoleCache2 {
    fn from_boundary_build_report(report: &RegionBoundaryContourBuildReport2) -> Option<Self> {
        let material_contour_count = report.material_contour_count()?;
        let hole_contour_count = report.hole_contour_count()?;
        let material_segment_count = report.material_segment_count()?;
        let hole_segment_count = report.hole_segment_count()?;
        let mut material_assignments = Vec::new();
        let mut hole_assignments = Vec::new();

        for (role_report_index, role_report) in report.role_reports().iter().enumerate() {
            let assignment = ExactCurveArrangementOutputRoleAssignment2 {
                role_report_index,
                source_contour_index: role_report.source_contour_index(),
                source_segment_count: role_report.source_segment_count(),
                source_fill_rule: role_report.source_fill_rule(),
                nesting_sample_point: role_report.nesting_sample_point().clone(),
                containing_contour_indices: role_report.containing_contour_indices().to_vec(),
                nesting_depth: role_report.nesting_depth(),
                output_role_index: role_report.output_role_index(),
                status: role_report.status(),
            };
            match role_report.role() {
                RegionBoundaryContourRole2::Material => material_assignments.push(assignment),
                RegionBoundaryContourRole2::Hole => hole_assignments.push(assignment),
            }
        }

        material_assignments.sort_by_key(|assignment| assignment.output_role_index);
        hole_assignments.sort_by_key(|assignment| assignment.output_role_index);
        let role_status_bucket_cache =
            ExactCurveArrangementOutputRoleStatusBucketCache2::from_role_assignments(
                &material_assignments,
                &hole_assignments,
            );
        let role_source_contour_bucket_cache =
            ExactCurveArrangementOutputRoleSourceContourBucketCache2::from_role_assignments(
                &material_assignments,
                &hole_assignments,
            );
        let role_nesting_depth_bucket_cache =
            ExactCurveArrangementOutputRoleNestingDepthBucketCache2::from_role_assignments(
                &material_assignments,
                &hole_assignments,
            );
        let role_containment_bucket_cache =
            ExactCurveArrangementOutputRoleContainmentBucketCache2::from_role_assignments(
                &material_assignments,
                &hole_assignments,
            );

        Some(Self {
            role_report_count: report.role_reports().len(),
            material_contour_count,
            hole_contour_count,
            material_segment_count,
            hole_segment_count,
            role_status_bucket_cache,
            role_source_contour_bucket_cache,
            role_nesting_depth_bucket_cache,
            role_containment_bucket_cache,
            buckets: vec![
                ExactCurveArrangementOutputRoleBucket2 {
                    role: RegionBoundaryContourRole2::Material,
                    assignments: material_assignments,
                },
                ExactCurveArrangementOutputRoleBucket2 {
                    role: RegionBoundaryContourRole2::Hole,
                    assignments: hole_assignments,
                },
            ],
        })
    }

    /// Returns the number of retained role reports.
    pub const fn role_report_count(&self) -> usize {
        self.role_report_count
    }

    /// Returns the number of material contours.
    pub const fn material_contour_count(&self) -> usize {
        self.material_contour_count
    }

    /// Returns the number of hole contours.
    pub const fn hole_contour_count(&self) -> usize {
        self.hole_contour_count
    }

    /// Returns the number of material boundary segments.
    pub const fn material_segment_count(&self) -> usize {
        self.material_segment_count
    }

    /// Returns the number of hole boundary segments.
    pub const fn hole_segment_count(&self) -> usize {
        self.hole_segment_count
    }

    /// Returns output role assignment buckets grouped by topology status.
    pub const fn role_status_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleStatusBucketCache2 {
        &self.role_status_bucket_cache
    }

    /// Returns output role assignment buckets grouped by source contour identity.
    pub const fn role_source_contour_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleSourceContourBucketCache2 {
        &self.role_source_contour_bucket_cache
    }

    /// Returns output role assignment buckets grouped by exact nesting depth.
    pub const fn role_nesting_depth_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleNestingDepthBucketCache2 {
        &self.role_nesting_depth_bucket_cache
    }

    /// Returns output role containment evidence grouped by containing source contour.
    pub const fn role_containment_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleContainmentBucketCache2 {
        &self.role_containment_bucket_cache
    }

    /// Returns material and hole role buckets in stable order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputRoleBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputBoundaryCache2 {
    fn from_arrangement_report(report: &RegionLineSegmentRegionBuildReport2) -> Option<Self> {
        let boundary_build_report = report.boundary_build_report.as_ref()?;
        let material_contour_count = boundary_build_report.material_contour_count()?;
        let hole_contour_count = boundary_build_report.hole_contour_count()?;
        let material_segment_count = boundary_build_report.material_segment_count()?;
        let hole_segment_count = boundary_build_report.hole_segment_count()?;
        Some(Self {
            output_contour_count: boundary_build_report.output_contour_count()?,
            output_segment_count: boundary_build_report.output_segment_count()?,
            output_segment_kind_counts: report.output_boundary_segment_kind_counts?,
            material_contour_count,
            hole_contour_count,
            material_segment_count,
            hole_segment_count,
            role_bucket_cache: ExactCurveArrangementOutputBoundaryRoleBucketCache2::new(
                material_contour_count,
                hole_contour_count,
                material_segment_count,
                hole_segment_count,
            ),
        })
    }

    /// Returns total output contour count.
    pub const fn output_contour_count(&self) -> usize {
        self.output_contour_count
    }

    /// Returns total output boundary segment count.
    pub const fn output_segment_count(&self) -> usize {
        self.output_segment_count
    }

    /// Returns output boundary primitive-family counts.
    pub const fn output_segment_kind_counts(&self) -> SegmentKindCounts {
        self.output_segment_kind_counts
    }

    /// Returns material contour count.
    pub const fn material_contour_count(&self) -> usize {
        self.material_contour_count
    }

    /// Returns hole contour count.
    pub const fn hole_contour_count(&self) -> usize {
        self.hole_contour_count
    }

    /// Returns material boundary segment count.
    pub const fn material_segment_count(&self) -> usize {
        self.material_segment_count
    }

    /// Returns hole boundary segment count.
    pub const fn hole_segment_count(&self) -> usize {
        self.hole_segment_count
    }

    /// Returns final boundary output counts grouped by material/hole role.
    pub const fn role_bucket_cache(&self) -> &ExactCurveArrangementOutputBoundaryRoleBucketCache2 {
        &self.role_bucket_cache
    }
}

impl ExactCurveArrangementOutputBoundaryRoleBucketCache2 {
    fn new(
        material_contour_count: usize,
        hole_contour_count: usize,
        material_segment_count: usize,
        hole_segment_count: usize,
    ) -> Self {
        Self {
            bucket_count: 2,
            output_contour_count: material_contour_count + hole_contour_count,
            output_segment_count: material_segment_count + hole_segment_count,
            max_segment_count: if material_segment_count > hole_segment_count {
                material_segment_count
            } else {
                hole_segment_count
            },
            buckets: vec![
                ExactCurveArrangementOutputBoundaryRoleBucket2 {
                    role: RegionBoundaryContourRole2::Material,
                    output_contour_count: material_contour_count,
                    output_segment_count: material_segment_count,
                },
                ExactCurveArrangementOutputBoundaryRoleBucket2 {
                    role: RegionBoundaryContourRole2::Hole,
                    output_contour_count: hole_contour_count,
                    output_segment_count: hole_segment_count,
                },
            ],
        }
    }

    /// Returns the number of retained role buckets.
    pub const fn bucket_count(&self) -> usize {
        self.bucket_count
    }

    /// Returns total output contour count across role buckets.
    pub const fn output_contour_count(&self) -> usize {
        self.output_contour_count
    }

    /// Returns total output segment count across role buckets.
    pub const fn output_segment_count(&self) -> usize {
        self.output_segment_count
    }

    /// Returns the largest segment count for one output role bucket.
    pub const fn max_segment_count(&self) -> usize {
        self.max_segment_count
    }

    /// Returns role buckets in stable material, then hole order.
    pub fn buckets(&self) -> &[ExactCurveArrangementOutputBoundaryRoleBucket2] {
        &self.buckets
    }
}

impl ExactCurveArrangementOutputBoundaryRoleBucket2 {
    /// Returns the boundary contour role represented by this bucket.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns output contour count for this role.
    pub const fn output_contour_count(&self) -> usize {
        self.output_contour_count
    }

    /// Returns output segment count for this role.
    pub const fn output_segment_count(&self) -> usize {
        self.output_segment_count
    }
}

impl ExactCurveArrangementRingAssemblyCache2 {
    fn from_arrangement_report(report: &RegionLineSegmentRegionBuildReport2) -> Option<Self> {
        Some(Self {
            predicate_path: report.ring_assembly_predicate_path?,
            attempted_endpoint_connection_count: report.attempted_endpoint_connection_count,
            exact_endpoint_connection_count: report.exact_endpoint_connection_count,
            disconnected_endpoint_connection_count: report.disconnected_endpoint_connection_count,
            unresolved_endpoint_connection_count: report.unresolved_endpoint_connection_count,
            reversed_source_segment_count: report.reversed_source_segment_count,
            output_ring_count: report.output_ring_count,
            output_boundary_segment_count: report.output_boundary_segment_count,
            output_boundary_segment_kind_counts: report.output_boundary_segment_kind_counts,
            arranged_source_reports: report.arranged_source_reports.clone(),
            source_reports: report.source_reports.clone(),
            arranged_fragment_cache:
                ExactCurveArrangementArrangedFragmentCache2::from_arranged_source_reports(
                    &report.arranged_source_reports,
                ),
            output_ring_bucket_cache:
                ExactCurveArrangementOutputRingBucketCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_segment_kind_bucket_cache:
                ExactCurveArrangementOutputSegmentKindBucketCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_segment_source_bucket_cache:
                ExactCurveArrangementOutputSegmentSourceBucketCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_segment_source_range_cache:
                ExactCurveArrangementOutputSegmentSourceRangeCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_segment_endpoint_cache:
                ExactCurveArrangementOutputSegmentEndpointCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_ring_continuity_cache:
                ExactCurveArrangementOutputRingContinuityCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_segment_status_bucket_cache:
                ExactCurveArrangementOutputSegmentStatusBucketCache2::from_source_reports(
                    &report.source_reports,
                ),
            output_segment_direction_bucket_cache:
                ExactCurveArrangementOutputSegmentDirectionBucketCache2::from_source_reports(
                    &report.source_reports,
                ),
        })
    }

    /// Returns the exact predicate family used for ring traversal.
    pub const fn predicate_path(&self) -> RegionLineSegmentRingAssemblyPredicatePath2 {
        self.predicate_path
    }

    /// Returns endpoint pair comparisons attempted during ring assembly.
    pub const fn attempted_endpoint_connection_count(&self) -> usize {
        self.attempted_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons certified as equal.
    pub const fn exact_endpoint_connection_count(&self) -> usize {
        self.exact_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons certified as disconnected.
    pub const fn disconnected_endpoint_connection_count(&self) -> usize {
        self.disconnected_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons whose equality could not be certified.
    pub const fn unresolved_endpoint_connection_count(&self) -> usize {
        self.unresolved_endpoint_connection_count
    }

    /// Returns source segments reversed while materializing ring traversal.
    pub const fn reversed_source_segment_count(&self) -> usize {
        self.reversed_source_segment_count
    }

    /// Returns output ring count when available.
    pub const fn output_ring_count(&self) -> Option<usize> {
        self.output_ring_count
    }

    /// Returns output boundary segment count when available.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        self.output_boundary_segment_count
    }

    /// Returns output boundary segment primitive-family counts when available.
    pub const fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_boundary_segment_kind_counts
    }

    /// Returns per-arranged-fragment source provenance after exact splitting.
    pub fn arranged_source_reports(&self) -> &[RegionLineSegmentArrangedSourceReport2] {
        &self.arranged_source_reports
    }

    /// Returns per-output segment source provenance.
    pub fn source_reports(&self) -> &[RegionLineSegmentRingSourceReport2] {
        &self.source_reports
    }

    /// Returns per-arranged-fragment source provenance buckets.
    pub const fn arranged_fragment_cache(&self) -> &ExactCurveArrangementArrangedFragmentCache2 {
        &self.arranged_fragment_cache
    }

    /// Returns per-output-ring source provenance buckets.
    pub const fn output_ring_bucket_cache(&self) -> &ExactCurveArrangementOutputRingBucketCache2 {
        &self.output_ring_bucket_cache
    }

    /// Returns retained output segment buckets grouped by primitive family.
    pub const fn output_segment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentKindBucketCache2 {
        &self.output_segment_kind_bucket_cache
    }

    /// Returns retained output segment buckets grouped by source segment.
    pub const fn output_segment_source_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentSourceBucketCache2 {
        &self.output_segment_source_bucket_cache
    }

    /// Returns retained output segment source parameter ranges.
    pub const fn output_segment_source_range_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentSourceRangeCache2 {
        &self.output_segment_source_range_cache
    }

    /// Returns retained output segment exact endpoint records.
    pub const fn output_segment_endpoint_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentEndpointCache2 {
        &self.output_segment_endpoint_cache
    }

    /// Returns retained exact continuity records between adjacent output segments.
    pub const fn output_ring_continuity_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRingContinuityCache2 {
        &self.output_ring_continuity_cache
    }

    /// Returns retained output segment buckets grouped by topology status.
    pub const fn output_segment_status_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentStatusBucketCache2 {
        &self.output_segment_status_bucket_cache
    }

    /// Returns retained output segment buckets grouped by traversal direction.
    pub const fn output_segment_direction_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentDirectionBucketCache2 {
        &self.output_segment_direction_bucket_cache
    }
}

impl ExactCurveArrangementOutputCache2 {
    fn from_arrangement_result(region_result: &RegionLineSegmentRegionBuildResult2) -> Self {
        let boundary_build_report = region_result.report().boundary_build_report.clone();
        let boundary_output_cache =
            ExactCurveArrangementOutputBoundaryCache2::from_arrangement_report(
                region_result.report(),
            );
        let role_cache = boundary_build_report
            .as_ref()
            .and_then(ExactCurveArrangementOutputRoleCache2::from_boundary_build_report);
        Self {
            materialized_region: region_result.region().is_some(),
            boundary_build_report,
            boundary_output_cache,
            role_cache,
            stage: region_result.report().stage,
            status: region_result.report().status,
            blocker: region_result.report().blocker,
        }
    }

    /// Returns whether the arrangement produced an owned region.
    pub const fn materialized_region(&self) -> bool {
        self.materialized_region
    }

    /// Returns delegated boundary-contour role assignment evidence, when reached.
    pub const fn boundary_build_report(&self) -> Option<&RegionBoundaryContourBuildReport2> {
        self.boundary_build_report.as_ref()
    }

    /// Returns delegated boundary-role assignment stage, if reached.
    pub const fn boundary_build_stage(&self) -> Option<RegionBoundaryContourBuildStage2> {
        match self.boundary_build_report() {
            Some(report) => Some(report.stage()),
            None => None,
        }
    }

    /// Returns delegated boundary-role assignment predicate path, if reached.
    pub const fn boundary_build_predicate_path(
        &self,
    ) -> Option<RegionBoundaryContourBuildPredicatePath2> {
        match self.boundary_build_report() {
            Some(report) => Some(report.predicate_path()),
            None => None,
        }
    }

    /// Returns delegated boundary-role assignment retained status, if reached.
    pub const fn boundary_build_status(&self) -> Option<RetainedTopologyStatus> {
        match self.boundary_build_report() {
            Some(report) => Some(report.status()),
            None => None,
        }
    }

    /// Returns delegated boundary-role assignment blocker, if present.
    pub const fn boundary_build_blocker(&self) -> Option<UncertaintyReason> {
        match self.boundary_build_report() {
            Some(report) => report.blocker(),
            None => None,
        }
    }

    /// Returns source contour count from delegated boundary-role assignment, if reached.
    pub const fn boundary_build_source_contour_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.source_contour_count()),
            None => None,
        }
    }

    /// Returns source boundary segment count from delegated boundary-role assignment, if reached.
    pub const fn boundary_build_source_segment_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.source_segment_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation schedule size from delegated role assignment, if reached.
    pub const fn boundary_build_validation_candidate_pair_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_candidate_pair_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation test count from delegated role assignment, if reached.
    pub const fn boundary_build_validation_tested_pair_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_tested_pair_count()),
            None => None,
        }
    }

    /// Returns exact validation intersection event count from delegated role assignment, if reached.
    pub const fn boundary_build_validation_intersection_event_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_intersection_event_count()),
            None => None,
        }
    }

    /// Returns containment classification count from delegated role assignment, if reached.
    pub const fn boundary_build_nesting_classification_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.nesting_classification_count()),
            None => None,
        }
    }

    /// Returns first blocking contour index from delegated role assignment, if present.
    pub const fn boundary_build_blocker_first_contour_index(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.blocker_first_contour_index(),
            None => None,
        }
    }

    /// Returns second blocking contour index from delegated role assignment, if present.
    pub const fn boundary_build_blocker_second_contour_index(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.blocker_second_contour_index(),
            None => None,
        }
    }

    /// Returns retained final boundary output summary when available.
    pub const fn boundary_output_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputBoundaryCache2> {
        self.boundary_output_cache.as_ref()
    }

    /// Returns retained material/hole role buckets when role assignment was reached.
    pub const fn role_cache(&self) -> Option<&ExactCurveArrangementOutputRoleCache2> {
        self.role_cache.as_ref()
    }

    /// Returns the final retained build stage reached by the arrangement.
    pub const fn stage(&self) -> RegionLineSegmentRegionBuildStage2 {
        self.stage
    }

    /// Returns final retained topology status for the arrangement.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the final blocker when arrangement did not materialize a region.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ExactCurveArrangementEvaluationSummaryCache2 {
    fn from_workspace(workspace: &ExactCurveWorkspace2) -> Self {
        let output_cache = workspace.output_cache();
        let ring_cache = workspace.ring_assembly_cache();
        let boundary_output_cache = output_cache.and_then(|cache| cache.boundary_output_cache());

        Self {
            evaluated_output: output_cache.is_some(),
            materialized_region: output_cache.map(|cache| cache.materialized_region()),
            stage: output_cache.map(|cache| cache.stage()),
            status: output_cache.map(|cache| cache.status()),
            blocker: output_cache.and_then(|cache| cache.blocker()),
            output_ring_count: ring_cache.and_then(|cache| cache.output_ring_count()),
            output_boundary_segment_count: ring_cache
                .and_then(|cache| cache.output_boundary_segment_count()),
            output_boundary_segment_kind_counts: ring_cache
                .and_then(|cache| cache.output_boundary_segment_kind_counts()),
            output_contour_count: boundary_output_cache.map(|cache| cache.output_contour_count()),
            output_segment_count: boundary_output_cache.map(|cache| cache.output_segment_count()),
        }
    }

    /// Returns whether final output evaluation facts were retained.
    pub const fn evaluated_output(&self) -> bool {
        self.evaluated_output
    }

    /// Returns whether the evaluation materialized a region, when evaluated.
    pub const fn materialized_region(&self) -> Option<bool> {
        self.materialized_region
    }

    /// Returns the final retained build stage, when evaluated.
    pub const fn stage(&self) -> Option<RegionLineSegmentRegionBuildStage2> {
        self.stage
    }

    /// Returns the final retained topology status, when evaluated.
    pub const fn status(&self) -> Option<RetainedTopologyStatus> {
        self.status
    }

    /// Returns the final blocker, when the evaluated arrangement blocked.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }

    /// Returns output ring count retained by ring assembly, when available.
    pub const fn output_ring_count(&self) -> Option<usize> {
        self.output_ring_count
    }

    /// Returns output boundary segment count retained by ring assembly, when available.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        self.output_boundary_segment_count
    }

    /// Returns output boundary primitive-family counts retained by ring assembly, when available.
    pub const fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_boundary_segment_kind_counts
    }

    /// Returns final output contour count retained after boundary role assignment.
    pub const fn output_contour_count(&self) -> Option<usize> {
        self.output_contour_count
    }

    /// Returns final output boundary segment count retained after boundary role assignment.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }
}

impl ExactCurveArrangementEvaluation2 {
    /// Builds an evaluation record from retained workspace facts.
    pub fn new(workspace: ExactCurveWorkspace2) -> Self {
        let summary_cache =
            ExactCurveArrangementEvaluationSummaryCache2::from_workspace(&workspace);
        Self {
            workspace,
            summary_cache,
        }
    }

    /// Returns the retained workspace consumed by this evaluation.
    pub const fn workspace(&self) -> &ExactCurveWorkspace2 {
        &self.workspace
    }

    /// Returns final retained evaluation facts derived from workspace caches.
    pub const fn summary_cache(&self) -> &ExactCurveArrangementEvaluationSummaryCache2 {
        &self.summary_cache
    }
}

impl ExactCurveArrangementAttempt2 {
    /// Creates a canonical exact arrangement attempt from a retained request.
    pub const fn new(request: ExactCurveArrangementRequest2) -> Self {
        Self { request }
    }

    /// Returns the retained request.
    pub const fn request(&self) -> &ExactCurveArrangementRequest2 {
        &self.request
    }

    /// Evaluates the request through the retained exact arrangement pipeline.
    pub fn evaluate(&self, policy: &CurvePolicy) -> CurveResult<ExactCurveArrangementResult2> {
        let region_result =
            if let Some(source_line_segments) = self.request.source_line_segments.as_ref() {
                evaluate_unordered_line_segments_region_result(
                    source_line_segments.clone(),
                    self.request.fill_rule,
                    policy,
                )?
            } else {
                evaluate_unordered_segments_region_result(
                    self.request.source_segments.clone(),
                    self.request.fill_rule,
                    policy,
                )?
            };
        let workspace = ExactCurveWorkspace2::from_request(self.request.clone(), policy)?
            .with_arrangement_result_facts(&region_result);
        Ok(ExactCurveArrangementResult2 {
            evaluation: ExactCurveArrangementEvaluation2::new(workspace),
            region_result,
        })
    }
}

impl ExactCurveArrangementResult2 {
    /// Returns the retained evaluation record.
    pub const fn evaluation(&self) -> &ExactCurveArrangementEvaluation2 {
        &self.evaluation
    }

    /// Returns the retained workspace consumed by the evaluation.
    pub const fn workspace(&self) -> &ExactCurveWorkspace2 {
        self.evaluation.workspace()
    }

    /// Returns retained source segment facts prepared before split scheduling.
    pub const fn source_segment_cache(&self) -> &ExactCurveArrangementSourceSegmentCache2 {
        self.workspace().source_segment_cache()
    }

    /// Returns exact source endpoint buckets retained during workspace preparation.
    pub const fn source_endpoint_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceEndpointBucketCache2 {
        self.workspace().source_endpoint_bucket_cache()
    }

    /// Returns the retained source-pair schedule prepared before split predicates run.
    pub const fn split_schedule_cache(&self) -> &ExactCurveArrangementSplitScheduleCache2 {
        self.workspace().split_schedule_cache()
    }

    /// Returns exact split evidence retained from the evaluated arrangement.
    pub const fn split_cache(&self) -> Option<&ExactCurveArrangementSplitCache2> {
        self.workspace().split_cache()
    }

    /// Returns the exact predicate family used by retained split evaluation.
    pub const fn split_predicate_path(&self) -> Option<RegionLineSegmentSplitPredicatePath2> {
        match self.split_cache() {
            Some(split_cache) => split_cache.predicate_path(),
            None => None,
        }
    }

    /// Returns source segment pairs considered by retained split evaluation.
    pub const fn split_candidate_pair_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.candidate_pair_count()),
            None => None,
        }
    }

    /// Returns source segment pairs skipped by certified AABB disjointness.
    pub const fn split_skipped_aabb_pair_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.skipped_aabb_pair_count()),
            None => None,
        }
    }

    /// Returns source segment pairs tested by exact split predicates.
    pub const fn split_tested_pair_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.tested_pair_count()),
            None => None,
        }
    }

    /// Returns exact point-intersection event count found during splitting.
    pub const fn split_intersection_event_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.intersection_event_count()),
            None => None,
        }
    }

    /// Returns source-pair relations classified as point intersections.
    pub const fn split_point_relation_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.point_relation_count()),
            None => None,
        }
    }

    /// Returns source-pair relations classified as overlaps.
    pub const fn split_overlap_relation_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.overlap_relation_count()),
            None => None,
        }
    }

    /// Returns source-pair relations that remained uncertain.
    pub const fn split_uncertain_relation_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.uncertain_relation_count()),
            None => None,
        }
    }

    /// Returns exact intersection points retained by split evaluation.
    pub fn split_intersection_points(&self) -> Option<&[Point2]> {
        self.split_cache()
            .map(ExactCurveArrangementSplitCache2::intersection_points)
    }

    /// Returns exact per-event source and parameter evidence retained by split evaluation.
    pub fn split_intersection_reports(
        &self,
    ) -> Option<&[RegionLineSegmentSplitIntersectionReport2]> {
        self.split_cache()
            .map(ExactCurveArrangementSplitCache2::intersection_reports)
    }

    /// Returns retained split-stage relation buckets.
    pub const fn split_relation_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitRelationBucketCache2> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.relation_bucket_cache()),
            None => None,
        }
    }

    /// Returns exact split-intersection point buckets.
    pub const fn split_intersection_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitIntersectionBucketCache2> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.intersection_bucket_cache()),
            None => None,
        }
    }

    /// Returns exact source-parameter evidence for retained split intersections.
    pub const fn split_intersection_parameter_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitIntersectionParameterCache2> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.intersection_parameter_cache()),
            None => None,
        }
    }

    /// Returns split-stage blocker source-pair evidence, when split evaluation blocked.
    pub const fn split_blocker_cache(&self) -> Option<&ExactCurveArrangementSplitBlockerCache2> {
        match self.split_cache() {
            Some(split_cache) => split_cache.blocker_cache(),
            None => None,
        }
    }

    /// Returns arranged output segment count when retained splitting completed.
    pub const fn split_output_segment_count(&self) -> Option<usize> {
        match self.split_cache() {
            Some(split_cache) => split_cache.output_segment_count(),
            None => None,
        }
    }

    /// Returns exact endpoint-bucket evidence retained from the evaluated arrangement.
    pub const fn endpoint_graph_cache(&self) -> Option<&ExactCurveArrangementEndpointGraphCache2> {
        self.workspace().endpoint_graph_cache()
    }

    /// Returns the exact predicate family used by retained endpoint-graph validation.
    pub const fn endpoint_graph_predicate_path(
        &self,
    ) -> Option<RegionLineSegmentEndpointGraphPredicatePath2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.predicate_path()),
            None => None,
        }
    }

    /// Returns arranged endpoint count validated by retained endpoint-graph evidence.
    pub const fn endpoint_graph_endpoint_count(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.endpoint_count()),
            None => None,
        }
    }

    /// Returns exact structural endpoint bucket count.
    pub const fn endpoint_graph_structural_bucket_count(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.structural_bucket_count()),
            None => None,
        }
    }

    /// Returns structural endpoint singleton bucket count.
    pub const fn endpoint_graph_structural_singleton_bucket_count(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.structural_singleton_bucket_count()),
            None => None,
        }
    }

    /// Returns the largest retained structural endpoint bucket size.
    pub const fn endpoint_graph_max_structural_bucket_size(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.max_structural_bucket_size()),
            None => None,
        }
    }

    /// Returns dangling endpoint count found during endpoint-graph validation.
    pub const fn endpoint_graph_dangling_endpoint_count(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.dangling_endpoint_count()),
            None => None,
        }
    }

    /// Returns branch endpoint count found during endpoint-graph validation.
    pub const fn endpoint_graph_branch_endpoint_count(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.branch_endpoint_count()),
            None => None,
        }
    }

    /// Returns the blocker arranged segment index from endpoint validation, when blocked.
    pub const fn endpoint_graph_blocker_arranged_segment_index(&self) -> Option<usize> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => endpoint_cache.blocker_arranged_segment_index(),
            None => None,
        }
    }

    /// Returns the blocker endpoint from endpoint validation, when blocked.
    pub const fn endpoint_graph_blocker_endpoint(
        &self,
    ) -> Option<RegionLineSegmentArrangedEndpoint2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => endpoint_cache.blocker_endpoint(),
            None => None,
        }
    }

    /// Returns the exact blocker point from endpoint validation, when blocked.
    pub const fn endpoint_graph_blocker_point(&self) -> Option<&Point2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => endpoint_cache.blocker_point(),
            None => None,
        }
    }

    /// Returns exact ring-traversal evidence retained from the evaluated arrangement.
    pub const fn ring_assembly_cache(&self) -> Option<&ExactCurveArrangementRingAssemblyCache2> {
        self.workspace().ring_assembly_cache()
    }

    /// Returns the exact predicate family used by retained ring traversal.
    pub const fn ring_assembly_predicate_path(
        &self,
    ) -> Option<RegionLineSegmentRingAssemblyPredicatePath2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.predicate_path()),
            None => None,
        }
    }

    /// Returns endpoint pair comparisons attempted during retained ring traversal.
    pub const fn attempted_endpoint_connection_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.attempted_endpoint_connection_count()),
            None => None,
        }
    }

    /// Returns endpoint pair comparisons certified as equal during ring traversal.
    pub const fn exact_endpoint_connection_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.exact_endpoint_connection_count()),
            None => None,
        }
    }

    /// Returns endpoint pair comparisons certified as disconnected during ring traversal.
    pub const fn disconnected_endpoint_connection_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.disconnected_endpoint_connection_count()),
            None => None,
        }
    }

    /// Returns endpoint pair comparisons unresolved during ring traversal.
    pub const fn unresolved_endpoint_connection_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.unresolved_endpoint_connection_count()),
            None => None,
        }
    }

    /// Returns source segments reversed while materializing retained ring traversal.
    pub const fn reversed_source_segment_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.reversed_source_segment_count()),
            None => None,
        }
    }

    /// Returns per-arranged-fragment source provenance retained after exact splitting.
    pub fn arranged_source_reports(&self) -> Option<&[RegionLineSegmentArrangedSourceReport2]> {
        self.ring_assembly_cache()
            .map(ExactCurveArrangementRingAssemblyCache2::arranged_source_reports)
    }

    /// Returns per-output segment source provenance retained by ring traversal.
    pub fn source_reports(&self) -> Option<&[RegionLineSegmentRingSourceReport2]> {
        self.ring_assembly_cache()
            .map(ExactCurveArrangementRingAssemblyCache2::source_reports)
    }

    /// Returns final output evidence retained from the evaluated arrangement.
    pub const fn output_cache(&self) -> Option<&ExactCurveArrangementOutputCache2> {
        self.workspace().output_cache()
    }

    /// Returns delegated boundary-contour role assignment evidence, when output reached it.
    pub const fn boundary_build_report(&self) -> Option<&RegionBoundaryContourBuildReport2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_report(),
            None => None,
        }
    }

    /// Returns final boundary-role assignment stage, if reached.
    pub const fn boundary_build_stage(&self) -> Option<RegionBoundaryContourBuildStage2> {
        match self.boundary_build_report() {
            Some(report) => Some(report.stage()),
            None => None,
        }
    }

    /// Returns final boundary-role assignment predicate path, if reached.
    pub const fn boundary_build_predicate_path(
        &self,
    ) -> Option<RegionBoundaryContourBuildPredicatePath2> {
        match self.boundary_build_report() {
            Some(report) => Some(report.predicate_path()),
            None => None,
        }
    }

    /// Returns final boundary-role assignment retained status, if reached.
    pub const fn boundary_build_status(&self) -> Option<RetainedTopologyStatus> {
        match self.boundary_build_report() {
            Some(report) => Some(report.status()),
            None => None,
        }
    }

    /// Returns final boundary-role assignment blocker, if present.
    pub const fn boundary_build_blocker(&self) -> Option<UncertaintyReason> {
        match self.boundary_build_report() {
            Some(report) => report.blocker(),
            None => None,
        }
    }

    /// Returns source contour count from final boundary-role assignment, if reached.
    pub const fn boundary_build_source_contour_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.source_contour_count()),
            None => None,
        }
    }

    /// Returns source boundary segment count from final boundary-role assignment, if reached.
    pub const fn boundary_build_source_segment_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.source_segment_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation schedule size from final role assignment, if reached.
    pub const fn boundary_build_validation_candidate_pair_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_candidate_pair_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation test count from final role assignment, if reached.
    pub const fn boundary_build_validation_tested_pair_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_tested_pair_count()),
            None => None,
        }
    }

    /// Returns exact validation intersection event count from final role assignment, if reached.
    pub const fn boundary_build_validation_intersection_event_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_intersection_event_count()),
            None => None,
        }
    }

    /// Returns containment classification count from final role assignment, if reached.
    pub const fn boundary_build_nesting_classification_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.nesting_classification_count()),
            None => None,
        }
    }

    /// Returns first blocking contour index from final role assignment, if present.
    pub const fn boundary_build_blocker_first_contour_index(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.blocker_first_contour_index(),
            None => None,
        }
    }

    /// Returns second blocking contour index from final role assignment, if present.
    pub const fn boundary_build_blocker_second_contour_index(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.blocker_second_contour_index(),
            None => None,
        }
    }

    /// Returns retained final boundary output summary when role assignment materialized.
    pub const fn boundary_output_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputBoundaryCache2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_output_cache(),
            None => None,
        }
    }

    /// Returns retained material/hole role buckets when role assignment was reached.
    pub const fn role_cache(&self) -> Option<&ExactCurveArrangementOutputRoleCache2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.role_cache(),
            None => None,
        }
    }

    /// Returns output boundary primitive-family counts after role assignment.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.output_segment_kind_counts()),
            None => None,
        }
    }

    /// Returns material contour count after output role assignment.
    pub const fn material_contour_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.material_contour_count()),
            None => None,
        }
    }

    /// Returns hole contour count after output role assignment.
    pub const fn hole_contour_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.hole_contour_count()),
            None => None,
        }
    }

    /// Returns material boundary segment count after output role assignment.
    pub const fn material_segment_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.material_segment_count()),
            None => None,
        }
    }

    /// Returns hole boundary segment count after output role assignment.
    pub const fn hole_segment_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.hole_segment_count()),
            None => None,
        }
    }

    /// Returns retained output role report count when role assignment was reached.
    pub const fn role_report_count(&self) -> Option<usize> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_report_count()),
            None => None,
        }
    }

    /// Returns output role assignment buckets grouped by topology status.
    pub const fn role_status_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleStatusBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_status_bucket_cache()),
            None => None,
        }
    }

    /// Returns output role assignment buckets grouped by source contour identity.
    pub const fn role_source_contour_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleSourceContourBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_source_contour_bucket_cache()),
            None => None,
        }
    }

    /// Returns output role assignment buckets grouped by exact nesting depth.
    pub const fn role_nesting_depth_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleNestingDepthBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_nesting_depth_bucket_cache()),
            None => None,
        }
    }

    /// Returns output role containment evidence grouped by containing source contour.
    pub const fn role_containment_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleContainmentBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_containment_bucket_cache()),
            None => None,
        }
    }

    /// Returns material and hole role buckets in stable order.
    pub fn role_buckets(&self) -> Option<&[ExactCurveArrangementOutputRoleBucket2]> {
        self.role_cache()
            .map(ExactCurveArrangementOutputRoleCache2::buckets)
    }

    /// Returns retained output role reports when role assignment was reached.
    pub fn role_reports(&self) -> Option<&[RegionBoundaryContourRoleReport2]> {
        self.boundary_build_report()
            .map(RegionBoundaryContourBuildReport2::role_reports)
    }

    /// Returns final retained evaluation facts derived from workspace caches.
    pub const fn summary_cache(&self) -> &ExactCurveArrangementEvaluationSummaryCache2 {
        self.evaluation.summary_cache()
    }

    /// Returns whether final output evaluation facts were retained.
    pub const fn evaluated_output(&self) -> bool {
        self.summary_cache().evaluated_output()
    }

    /// Returns whether the retained evaluation materialized a region, when evaluated.
    pub const fn materialized_region(&self) -> Option<bool> {
        self.summary_cache().materialized_region()
    }

    /// Returns the final retained build stage, when evaluated.
    pub const fn stage(&self) -> Option<RegionLineSegmentRegionBuildStage2> {
        self.summary_cache().stage()
    }

    /// Returns the final retained topology status, when evaluated.
    pub const fn status(&self) -> Option<RetainedTopologyStatus> {
        self.summary_cache().status()
    }

    /// Returns the final retained blocker, when the evaluated arrangement blocked.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.summary_cache().blocker()
    }

    /// Returns output ring count retained by ring assembly, when available.
    pub const fn output_ring_count(&self) -> Option<usize> {
        self.summary_cache().output_ring_count()
    }

    /// Returns output boundary segment count retained by ring assembly, when available.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        self.summary_cache().output_boundary_segment_count()
    }

    /// Returns output boundary primitive-family counts retained by ring assembly, when available.
    pub const fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.summary_cache().output_boundary_segment_kind_counts()
    }

    /// Returns final output contour count retained after boundary role assignment.
    pub const fn output_contour_count(&self) -> Option<usize> {
        self.summary_cache().output_contour_count()
    }

    /// Returns final output boundary segment count retained after boundary role assignment.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.summary_cache().output_segment_count()
    }

    /// Returns the underlying legacy region build result.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementResult2 retained accessors, or report() when a derived compatibility report is required"
    )]
    pub const fn region_build_result(&self) -> &RegionLineSegmentRegionBuildResult2 {
        &self.region_result
    }

    /// Returns the materialized region, if the arrangement succeeded.
    pub const fn region(&self) -> Option<&Region2> {
        self.region_result.region()
    }

    /// Returns the derived retained arrangement report.
    pub const fn report(&self) -> &RegionLineSegmentRegionBuildReport2 {
        self.region_result.report()
    }

    /// Consumes this result and returns the underlying legacy region build result.
    #[deprecated(
        since = "0.3.0",
        note = "use ExactCurveArrangementResult2 retained accessors, into_region(), or report() when a derived compatibility report is required"
    )]
    pub fn into_region_build_result(self) -> RegionLineSegmentRegionBuildResult2 {
        self.region_result
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<Region2> {
        self.region_result.into_region()
    }
}

impl RegionBoundaryContourRoleReport2 {
    /// Returns the source contour index assigned by this report.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns the source contour segment count captured before role binning.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the source contour fill rule captured before role binning.
    pub const fn source_fill_rule(&self) -> FillRule {
        self.source_fill_rule
    }

    /// Returns the exact source point used for containment classification.
    pub const fn nesting_sample_point(&self) -> &Point2 {
        &self.nesting_sample_point
    }

    /// Returns source contour indices that exactly contained the sample point.
    pub fn containing_contour_indices(&self) -> &[usize] {
        &self.containing_contour_indices
    }

    /// Returns exact containment depth used for material/hole parity.
    pub const fn nesting_depth(&self) -> usize {
        self.nesting_depth
    }

    /// Returns the assigned material/hole role.
    pub const fn role(&self) -> RegionBoundaryContourRole2 {
        self.role
    }

    /// Returns this contour's index inside its output role bin.
    pub const fn output_role_index(&self) -> usize {
        self.output_role_index
    }

    /// Returns retained topology status for this role assignment.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl RegionBoundaryContourBuildReport2 {
    /// Returns the furthest exact region-construction stage reached.
    pub const fn stage(&self) -> RegionBoundaryContourBuildStage2 {
        self.stage
    }

    /// Returns the exact predicate path used for boundary validation and nesting.
    pub const fn predicate_path(&self) -> RegionBoundaryContourBuildPredicatePath2 {
        self.predicate_path
    }

    /// Returns the number of source boundary contours considered.
    pub const fn source_contour_count(&self) -> usize {
        self.source_contour_count
    }

    /// Returns the total number of source contour segments considered.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the number of contour pairs scheduled for intersection validation.
    pub const fn validation_candidate_pair_count(&self) -> usize {
        self.validation_candidate_pair_count
    }

    /// Returns the number of contour pairs tested before success or a blocker.
    pub const fn validation_tested_pair_count(&self) -> usize {
        self.validation_tested_pair_count
    }

    /// Returns exact contour-intersection events found during nesting validation.
    pub const fn validation_intersection_event_count(&self) -> usize {
        self.validation_intersection_event_count
    }

    /// Returns point-containment classifications used to assign nesting roles.
    pub const fn nesting_classification_count(&self) -> usize {
        self.nesting_classification_count
    }

    /// Returns the first source contour index involved in a blocking relation.
    pub const fn blocker_first_contour_index(&self) -> Option<usize> {
        self.blocker_first_contour_index
    }

    /// Returns the second source contour index involved in a blocking relation.
    pub const fn blocker_second_contour_index(&self) -> Option<usize> {
        self.blocker_second_contour_index
    }

    /// Returns total output contour count when role assignment materialized.
    pub const fn output_contour_count(&self) -> Option<usize> {
        self.output_contour_count
    }

    /// Returns total output boundary segment count when role assignment materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns material contour count when role assignment materialized.
    pub const fn material_contour_count(&self) -> Option<usize> {
        self.material_contour_count
    }

    /// Returns hole contour count when role assignment materialized.
    pub const fn hole_contour_count(&self) -> Option<usize> {
        self.hole_contour_count
    }

    /// Returns material boundary segment count when role assignment materialized.
    pub const fn material_segment_count(&self) -> Option<usize> {
        self.material_segment_count
    }

    /// Returns hole boundary segment count when role assignment materialized.
    pub const fn hole_segment_count(&self) -> Option<usize> {
        self.hole_segment_count
    }

    /// Returns per-contour exact role reports.
    pub fn role_reports(&self) -> &[RegionBoundaryContourRoleReport2] {
        &self.role_reports
    }

    /// Returns region construction status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized construction attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl RegionBoundaryContourBuildResult2 {
    /// Returns the materialized region, if role assignment succeeded.
    pub const fn region(&self) -> Option<&Region2> {
        self.region.as_ref()
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<Region2> {
        self.region
    }

    /// Returns the retained region-construction report.
    pub const fn report(&self) -> &RegionBoundaryContourBuildReport2 {
        &self.report
    }
}

impl RegionLineSegmentRingSourceReport2 {
    /// Returns the source segment index used by this output segment.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the exact start point of the original source segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the original source segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the retained parameter range on the source segment.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the output ring index.
    pub const fn output_ring_index(&self) -> usize {
        self.output_ring_index
    }

    /// Returns the output segment index inside the ring.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the primitive family of the emitted output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns whether the source segment was reversed for ring traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the emitted segment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the emitted segment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained topology status for this source-to-ring mapping.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl RegionLineSegmentArrangedSourceReport2 {
    /// Returns the source segment index used by this arranged fragment.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the primitive family of the source segment.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the exact start point of the original source segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the original source segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the retained parameter range on the source segment.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the arranged fragment index after exact splitting.
    pub const fn arranged_segment_index(&self) -> usize {
        self.arranged_segment_index
    }

    /// Returns the primitive family of the arranged fragment.
    pub const fn arranged_segment_kind(&self) -> SegmentKind {
        self.arranged_segment_kind
    }

    /// Returns the arranged fragment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the arranged fragment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained topology status for this source-to-fragment mapping.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl RegionLineSegmentRegionBuildReport2 {
    /// Returns the furthest exact line-region construction stage reached.
    pub const fn stage(&self) -> RegionLineSegmentRegionBuildStage2 {
        self.stage
    }

    /// Returns the number of source segments considered.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns source segment primitive-family counts.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns arranged segment count after exact point-intersection splitting.
    pub const fn arranged_segment_count(&self) -> Option<usize> {
        self.arranged_segment_count
    }

    /// Returns arranged fragment primitive-family counts after exact splitting.
    pub const fn arranged_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.arranged_segment_kind_counts
    }

    /// Returns the exact predicate family used for split arrangement, when reached.
    pub const fn split_predicate_path(&self) -> Option<RegionLineSegmentSplitPredicatePath2> {
        self.split_predicate_path
    }

    /// Returns the exact predicate family used for endpoint-graph validation, when reached.
    pub const fn endpoint_graph_predicate_path(
        &self,
    ) -> Option<RegionLineSegmentEndpointGraphPredicatePath2> {
        self.endpoint_graph_predicate_path
    }

    /// Returns the exact predicate family used for ring traversal, when reached.
    pub const fn ring_assembly_predicate_path(
        &self,
    ) -> Option<RegionLineSegmentRingAssemblyPredicatePath2> {
        self.ring_assembly_predicate_path
    }

    /// Returns source line pairs considered for splitting.
    pub const fn split_candidate_pair_count(&self) -> usize {
        self.split_candidate_pair_count
    }

    /// Returns source line pairs skipped by decided disjoint AABBs.
    pub const fn split_skipped_aabb_pair_count(&self) -> usize {
        self.split_skipped_aabb_pair_count
    }

    /// Returns source line pairs tested by exact line-line predicates.
    pub const fn split_tested_pair_count(&self) -> usize {
        self.split_tested_pair_count
    }

    /// Returns certified point-intersection split events collected.
    pub const fn split_intersection_event_count(&self) -> usize {
        self.split_intersection_event_count
    }

    /// Returns source segment-pair relations that produced one or more exact split points.
    pub const fn split_point_relation_count(&self) -> usize {
        self.split_point_relation_count
    }

    /// Returns source segment-pair relations blocked by exact overlap topology.
    pub const fn split_overlap_relation_count(&self) -> usize {
        self.split_overlap_relation_count
    }

    /// Returns source segment-pair relations left unresolved by the active policy.
    pub const fn split_uncertain_relation_count(&self) -> usize {
        self.split_uncertain_relation_count
    }

    /// Returns exact point-intersection events retained during split arrangement.
    pub fn split_intersection_points(&self) -> &[Point2] {
        &self.split_intersection_points
    }

    /// Returns source/parameter evidence for retained point-intersection split events.
    pub fn split_intersection_reports(&self) -> &[RegionLineSegmentSplitIntersectionReport2] {
        &self.split_intersection_reports
    }

    /// Returns arranged output segment count after splitting, when available.
    pub const fn split_output_segment_count(&self) -> Option<usize> {
        self.split_output_segment_count
    }

    /// Returns the first source segment in a split-stage blocker, when known.
    pub const fn split_blocker_first_source_segment_index(&self) -> Option<usize> {
        self.split_blocker_first_source_segment_index
    }

    /// Returns the primitive family of the first source segment in a split-stage blocker.
    pub const fn split_blocker_first_source_segment_kind(&self) -> Option<SegmentKind> {
        self.split_blocker_first_source_segment_kind
    }

    /// Returns the exact start point of the first source segment in a split-stage blocker.
    pub const fn split_blocker_first_source_start_point(&self) -> Option<&Point2> {
        self.split_blocker_first_source_start_point.as_ref()
    }

    /// Returns the exact end point of the first source segment in a split-stage blocker.
    pub const fn split_blocker_first_source_end_point(&self) -> Option<&Point2> {
        self.split_blocker_first_source_end_point.as_ref()
    }

    /// Returns the second source segment in a split-stage blocker, when known.
    pub const fn split_blocker_second_source_segment_index(&self) -> Option<usize> {
        self.split_blocker_second_source_segment_index
    }

    /// Returns the primitive family of the second source segment in a split-stage blocker.
    pub const fn split_blocker_second_source_segment_kind(&self) -> Option<SegmentKind> {
        self.split_blocker_second_source_segment_kind
    }

    /// Returns the exact start point of the second source segment in a split-stage blocker.
    pub const fn split_blocker_second_source_start_point(&self) -> Option<&Point2> {
        self.split_blocker_second_source_start_point.as_ref()
    }

    /// Returns the exact end point of the second source segment in a split-stage blocker.
    pub const fn split_blocker_second_source_end_point(&self) -> Option<&Point2> {
        self.split_blocker_second_source_end_point.as_ref()
    }

    /// Returns arranged endpoint count validated before ring traversal.
    pub const fn endpoint_graph_endpoint_count(&self) -> Option<usize> {
        self.endpoint_graph_endpoint_count
    }

    /// Returns structurally identical endpoint bucket count.
    pub const fn endpoint_graph_structural_bucket_count(&self) -> Option<usize> {
        self.endpoint_graph_structural_bucket_count
    }

    /// Returns structurally identical endpoint buckets containing exactly one endpoint.
    pub const fn endpoint_graph_structural_singleton_bucket_count(&self) -> Option<usize> {
        self.endpoint_graph_structural_singleton_bucket_count
    }

    /// Returns the largest structurally identical endpoint bucket size.
    pub const fn endpoint_graph_max_structural_bucket_size(&self) -> Option<usize> {
        self.endpoint_graph_max_structural_bucket_size
    }

    /// Returns arranged endpoints with no exact mate.
    pub const fn endpoint_graph_dangling_endpoint_count(&self) -> Option<usize> {
        self.endpoint_graph_dangling_endpoint_count
    }

    /// Returns arranged endpoints with more than one exact mate.
    pub const fn endpoint_graph_branch_endpoint_count(&self) -> Option<usize> {
        self.endpoint_graph_branch_endpoint_count
    }

    /// Returns the arranged segment index of the first endpoint-graph blocker.
    pub const fn endpoint_graph_blocker_arranged_segment_index(&self) -> Option<usize> {
        self.endpoint_graph_blocker_arranged_segment_index
    }

    /// Returns the arranged endpoint of the first endpoint-graph blocker.
    pub const fn endpoint_graph_blocker_endpoint(
        &self,
    ) -> Option<RegionLineSegmentArrangedEndpoint2> {
        self.endpoint_graph_blocker_endpoint
    }

    /// Returns the exact arranged endpoint point of the first endpoint-graph blocker.
    pub const fn endpoint_graph_blocker_point(&self) -> Option<&Point2> {
        self.endpoint_graph_blocker_point.as_ref()
    }

    /// Returns endpoint pair comparisons attempted during ring assembly.
    pub const fn attempted_endpoint_connection_count(&self) -> usize {
        self.attempted_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons certified as equal.
    pub const fn exact_endpoint_connection_count(&self) -> usize {
        self.exact_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons certified as disconnected.
    pub const fn disconnected_endpoint_connection_count(&self) -> usize {
        self.disconnected_endpoint_connection_count
    }

    /// Returns endpoint pair comparisons whose equality could not be certified.
    pub const fn unresolved_endpoint_connection_count(&self) -> usize {
        self.unresolved_endpoint_connection_count
    }

    /// Returns source segments reversed while materializing ring traversal.
    pub const fn reversed_source_segment_count(&self) -> usize {
        self.reversed_source_segment_count
    }

    /// Returns output ring count when available.
    pub const fn output_ring_count(&self) -> Option<usize> {
        self.output_ring_count
    }

    /// Returns output boundary segment count when available.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        self.output_boundary_segment_count
    }

    /// Returns output boundary segment primitive-family counts when available.
    pub const fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_boundary_segment_kind_counts
    }

    /// Returns per-arranged-fragment source provenance after exact splitting.
    pub fn arranged_source_reports(&self) -> &[RegionLineSegmentArrangedSourceReport2] {
        &self.arranged_source_reports
    }

    /// Returns per-output segment source provenance.
    pub fn source_reports(&self) -> &[RegionLineSegmentRingSourceReport2] {
        &self.source_reports
    }

    /// Returns delegated boundary-contour role assignment evidence, when reached.
    pub const fn boundary_build_report(&self) -> Option<&RegionBoundaryContourBuildReport2> {
        self.boundary_build_report.as_ref()
    }

    /// Returns final boundary-role assignment stage, if reached.
    pub const fn boundary_build_stage(&self) -> Option<RegionBoundaryContourBuildStage2> {
        match self.boundary_build_report() {
            Some(report) => Some(report.stage()),
            None => None,
        }
    }

    /// Returns final boundary-role assignment predicate path, if reached.
    pub const fn boundary_build_predicate_path(
        &self,
    ) -> Option<RegionBoundaryContourBuildPredicatePath2> {
        match self.boundary_build_report() {
            Some(report) => Some(report.predicate_path()),
            None => None,
        }
    }

    /// Returns final boundary-role assignment retained status, if reached.
    pub const fn boundary_build_status(&self) -> Option<RetainedTopologyStatus> {
        match self.boundary_build_report() {
            Some(report) => Some(report.status()),
            None => None,
        }
    }

    /// Returns final boundary-role assignment blocker, if present.
    pub const fn boundary_build_blocker(&self) -> Option<UncertaintyReason> {
        match self.boundary_build_report() {
            Some(report) => report.blocker(),
            None => None,
        }
    }

    /// Returns source contour count from final boundary-role assignment, if reached.
    pub const fn boundary_build_source_contour_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.source_contour_count()),
            None => None,
        }
    }

    /// Returns source boundary segment count from final boundary-role assignment, if reached.
    pub const fn boundary_build_source_segment_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.source_segment_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation schedule size from final role assignment, if reached.
    pub const fn boundary_build_validation_candidate_pair_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_candidate_pair_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation test count from final role assignment, if reached.
    pub const fn boundary_build_validation_tested_pair_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_tested_pair_count()),
            None => None,
        }
    }

    /// Returns exact validation intersection event count from final role assignment, if reached.
    pub const fn boundary_build_validation_intersection_event_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.validation_intersection_event_count()),
            None => None,
        }
    }

    /// Returns containment classification count from final role assignment, if reached.
    pub const fn boundary_build_nesting_classification_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.nesting_classification_count()),
            None => None,
        }
    }

    /// Returns first blocking contour index from final role assignment, if present.
    pub const fn boundary_build_blocker_first_contour_index(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.blocker_first_contour_index(),
            None => None,
        }
    }

    /// Returns second blocking contour index from final role assignment, if present.
    pub const fn boundary_build_blocker_second_contour_index(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.blocker_second_contour_index(),
            None => None,
        }
    }

    /// Returns final output contour count when role assignment was reached.
    pub const fn output_contour_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.output_contour_count(),
            None => None,
        }
    }

    /// Returns final output boundary segment count when role assignment was reached.
    pub const fn output_segment_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.output_segment_count(),
            None => None,
        }
    }

    /// Returns material contour count when role assignment was reached.
    pub const fn material_contour_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.material_contour_count(),
            None => None,
        }
    }

    /// Returns hole contour count when role assignment was reached.
    pub const fn hole_contour_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.hole_contour_count(),
            None => None,
        }
    }

    /// Returns material boundary segment count when role assignment was reached.
    pub const fn material_segment_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.material_segment_count(),
            None => None,
        }
    }

    /// Returns hole boundary segment count when role assignment was reached.
    pub const fn hole_segment_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => report.hole_segment_count(),
            None => None,
        }
    }

    /// Returns retained output role report count when role assignment was reached.
    pub fn role_report_count(&self) -> Option<usize> {
        match self.boundary_build_report() {
            Some(report) => Some(report.role_reports().len()),
            None => None,
        }
    }

    /// Returns retained output role reports when role assignment was reached.
    pub fn role_reports(&self) -> Option<&[RegionBoundaryContourRoleReport2]> {
        self.boundary_build_report()
            .map(RegionBoundaryContourBuildReport2::role_reports)
    }

    /// Returns line-region construction status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized construction attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl RegionLineSegmentSplitIntersectionReport2 {
    /// Returns the first source segment index for this split event.
    pub const fn first_source_segment_index(&self) -> usize {
        self.first_source_segment_index
    }

    /// Returns the first source segment primitive family.
    pub const fn first_source_segment_kind(&self) -> SegmentKind {
        self.first_source_segment_kind
    }

    /// Returns the exact start point of the first source segment.
    pub const fn first_source_segment_start_point(&self) -> &Point2 {
        &self.first_source_segment_start_point
    }

    /// Returns the exact end point of the first source segment.
    pub const fn first_source_segment_end_point(&self) -> &Point2 {
        &self.first_source_segment_end_point
    }

    /// Returns the retained local parameter on the first source segment.
    pub const fn first_source_param(&self) -> &Real {
        &self.first_source_param
    }

    /// Returns the second source segment index for this split event.
    pub const fn second_source_segment_index(&self) -> usize {
        self.second_source_segment_index
    }

    /// Returns the second source segment primitive family.
    pub const fn second_source_segment_kind(&self) -> SegmentKind {
        self.second_source_segment_kind
    }

    /// Returns the exact start point of the second source segment.
    pub const fn second_source_segment_start_point(&self) -> &Point2 {
        &self.second_source_segment_start_point
    }

    /// Returns the exact end point of the second source segment.
    pub const fn second_source_segment_end_point(&self) -> &Point2 {
        &self.second_source_segment_end_point
    }

    /// Returns the retained local parameter on the second source segment.
    pub const fn second_source_param(&self) -> &Real {
        &self.second_source_param
    }

    /// Returns the exact point shared by both source parameters.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }
}

impl RegionLineSegmentRegionBuildResult2 {
    /// Returns the materialized region, if construction succeeded.
    pub const fn region(&self) -> Option<&Region2> {
        self.region.as_ref()
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<Region2> {
        self.region
    }

    /// Returns the retained line-region construction report.
    pub const fn report(&self) -> &RegionLineSegmentRegionBuildReport2 {
        &self.report
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
struct LineSegmentRingAssemblyCounts {
    attempted_endpoint_connection_count: usize,
    exact_endpoint_connection_count: usize,
    disconnected_endpoint_connection_count: usize,
    unresolved_endpoint_connection_count: usize,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct LineSegmentEndpointGraphReportParts {
    endpoint_count: usize,
    structural_bucket_count: usize,
    structural_singleton_bucket_count: usize,
    max_structural_bucket_size: usize,
    dangling_endpoint_count: usize,
    branch_endpoint_count: usize,
    blocker_arranged_segment_index: Option<usize>,
    blocker_endpoint: Option<RegionLineSegmentArrangedEndpoint2>,
    blocker_point: Option<Point2>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct LineSegmentRingAssemblyReportParts {
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
}

#[derive(Clone, Debug, PartialEq)]
struct LineSegmentRingAssembly {
    rings: Vec<Vec<LineSeg2>>,
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct LineSegmentSplitReportParts {
    predicate_path: Option<RegionLineSegmentSplitPredicatePath2>,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    intersection_event_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    intersection_points: Vec<Point2>,
    intersection_reports: Vec<RegionLineSegmentSplitIntersectionReport2>,
    output_segment_count: Option<usize>,
    blocker_first_source_segment_index: Option<usize>,
    blocker_first_source_segment_kind: Option<SegmentKind>,
    blocker_first_source_start_point: Option<Point2>,
    blocker_first_source_end_point: Option<Point2>,
    blocker_second_source_segment_index: Option<usize>,
    blocker_second_source_segment_kind: Option<SegmentKind>,
    blocker_second_source_start_point: Option<Point2>,
    blocker_second_source_end_point: Option<Point2>,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedLineSegment {
    source_segment_index: usize,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    line: LineSeg2,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedLineSegments {
    segments: Vec<ArrangedLineSegment>,
    report: LineSegmentSplitReportParts,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedNativeSegment {
    source_segment_index: usize,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    segment: Segment2,
}

#[derive(Clone, Debug, PartialEq)]
struct ArrangedNativeSegments {
    segments: Vec<ArrangedNativeSegment>,
    report: LineSegmentSplitReportParts,
}

impl ArrangedLineSegment {
    fn reversed(&self) -> Self {
        Self {
            source_segment_index: self.source_segment_index,
            source_segment_start_point: self.source_segment_start_point.clone(),
            source_segment_end_point: self.source_segment_end_point.clone(),
            source_range: ParamRange::new(
                self.source_range.end().clone(),
                self.source_range.start().clone(),
            ),
            line: self.line.reversed(),
        }
    }
}

impl ArrangedNativeSegment {
    fn reversed(&self) -> Self {
        Self {
            source_segment_index: self.source_segment_index,
            source_segment_start_point: self.source_segment_start_point.clone(),
            source_segment_end_point: self.source_segment_end_point.clone(),
            source_range: ParamRange::new(
                self.source_range.end().clone(),
                self.source_range.start().clone(),
            ),
            segment: self.segment.reversed(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum EndpointCandidate {
    Start,
    End,
}

#[derive(Clone, Debug, PartialEq)]
struct LineSegmentSplitMarker {
    param: Real,
}

#[derive(Clone, Debug, PartialEq)]
struct NativeSegmentSplitMarker {
    param: Real,
    point: Point2,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ArrangedLineEndpoint {
    segment_index: usize,
    endpoint: EndpointCandidate,
}

fn validate_arranged_line_endpoint_graph(
    segments: &[ArrangedLineSegment],
    policy: &CurvePolicy,
) -> Result<
    (
        LineSegmentEndpointGraphReportParts,
        LineSegmentRingAssemblyCounts,
    ),
    (
        LineSegmentEndpointGraphReportParts,
        LineSegmentRingAssemblyCounts,
        UncertaintyReason,
    ),
> {
    let endpoints = arranged_line_endpoints(segments);
    let mut graph = structural_endpoint_bucket_report(segments, &endpoints);
    let mut counts = LineSegmentRingAssemblyCounts::default();

    for (endpoint_index, endpoint) in endpoints.iter().enumerate() {
        let point = arranged_line_endpoint_point(segments, *endpoint);
        let mut exact_match_count = 0_usize;
        for (candidate_index, candidate) in endpoints.iter().enumerate() {
            if endpoint_index == candidate_index
                || endpoint.segment_index == candidate.segment_index
            {
                continue;
            }
            match exact_points_match(
                point,
                arranged_line_endpoint_point(segments, *candidate),
                policy,
                &mut counts,
            ) {
                Classification::Decided(true) => exact_match_count += 1,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    set_endpoint_graph_blocker(&mut graph, *endpoint, point);
                    return Err((graph, counts, reason));
                }
            }
        }
        match exact_match_count {
            1 => {}
            0 => {
                graph.dangling_endpoint_count += 1;
                set_endpoint_graph_blocker(&mut graph, *endpoint, point);
            }
            _ => {
                graph.branch_endpoint_count += 1;
                set_endpoint_graph_blocker(&mut graph, *endpoint, point);
            }
        }
    }

    if graph.dangling_endpoint_count > 0 || graph.branch_endpoint_count > 0 {
        Err((graph, counts, UncertaintyReason::Boundary))
    } else {
        Ok((graph, counts))
    }
}

fn set_endpoint_graph_blocker(
    graph: &mut LineSegmentEndpointGraphReportParts,
    endpoint: ArrangedLineEndpoint,
    point: &Point2,
) {
    if graph.blocker_arranged_segment_index.is_none() {
        graph.blocker_arranged_segment_index = Some(endpoint.segment_index);
        graph.blocker_endpoint = Some(region_arranged_endpoint(endpoint.endpoint));
        graph.blocker_point = Some(point.clone());
    }
}

fn region_arranged_endpoint(endpoint: EndpointCandidate) -> RegionLineSegmentArrangedEndpoint2 {
    match endpoint {
        EndpointCandidate::Start => RegionLineSegmentArrangedEndpoint2::Start,
        EndpointCandidate::End => RegionLineSegmentArrangedEndpoint2::End,
    }
}

fn structural_endpoint_bucket_report(
    segments: &[ArrangedLineSegment],
    endpoints: &[ArrangedLineEndpoint],
) -> LineSegmentEndpointGraphReportParts {
    let mut buckets: Vec<(Point2, usize)> = Vec::new();
    for endpoint in endpoints {
        let point = arranged_line_endpoint_point(segments, *endpoint);
        if let Some((_, count)) = buckets
            .iter_mut()
            .find(|(bucket_point, _)| bucket_point == point)
        {
            *count += 1;
        } else {
            buckets.push((point.clone(), 1));
        }
    }

    LineSegmentEndpointGraphReportParts {
        endpoint_count: endpoints.len(),
        structural_bucket_count: buckets.len(),
        structural_singleton_bucket_count: buckets.iter().filter(|(_, count)| *count == 1).count(),
        max_structural_bucket_size: buckets.iter().map(|(_, count)| *count).max().unwrap_or(0),
        ..LineSegmentEndpointGraphReportParts::default()
    }
}

fn arranged_line_endpoints(segments: &[ArrangedLineSegment]) -> Vec<ArrangedLineEndpoint> {
    let mut endpoints = Vec::with_capacity(segments.len() * 2);
    for segment_index in 0..segments.len() {
        endpoints.push(ArrangedLineEndpoint {
            segment_index,
            endpoint: EndpointCandidate::Start,
        });
        endpoints.push(ArrangedLineEndpoint {
            segment_index,
            endpoint: EndpointCandidate::End,
        });
    }
    endpoints
}

fn arranged_line_endpoint_point(
    segments: &[ArrangedLineSegment],
    endpoint: ArrangedLineEndpoint,
) -> &Point2 {
    match endpoint.endpoint {
        EndpointCandidate::Start => segments[endpoint.segment_index].line.start(),
        EndpointCandidate::End => segments[endpoint.segment_index].line.end(),
    }
}

fn validate_arranged_native_endpoint_graph(
    segments: &[ArrangedNativeSegment],
    policy: &CurvePolicy,
) -> Result<
    (
        LineSegmentEndpointGraphReportParts,
        LineSegmentRingAssemblyCounts,
    ),
    (
        LineSegmentEndpointGraphReportParts,
        LineSegmentRingAssemblyCounts,
        UncertaintyReason,
    ),
> {
    let endpoints = arranged_native_endpoints(segments);
    let mut graph = structural_native_endpoint_bucket_report(segments, &endpoints);
    let mut counts = LineSegmentRingAssemblyCounts::default();

    for (endpoint_index, endpoint) in endpoints.iter().enumerate() {
        let point = arranged_native_endpoint_point(segments, *endpoint);
        let mut exact_match_count = 0_usize;
        for (candidate_index, candidate) in endpoints.iter().enumerate() {
            if endpoint_index == candidate_index
                || endpoint.segment_index == candidate.segment_index
            {
                continue;
            }
            match exact_points_match(
                point,
                arranged_native_endpoint_point(segments, *candidate),
                policy,
                &mut counts,
            ) {
                Classification::Decided(true) => exact_match_count += 1,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    set_endpoint_graph_blocker(&mut graph, *endpoint, point);
                    return Err((graph, counts, reason));
                }
            }
        }
        match exact_match_count {
            1 => {}
            0 => {
                graph.dangling_endpoint_count += 1;
                set_endpoint_graph_blocker(&mut graph, *endpoint, point);
            }
            _ => {
                graph.branch_endpoint_count += 1;
                set_endpoint_graph_blocker(&mut graph, *endpoint, point);
            }
        }
    }

    if graph.dangling_endpoint_count > 0 || graph.branch_endpoint_count > 0 {
        Err((graph, counts, UncertaintyReason::Boundary))
    } else {
        Ok((graph, counts))
    }
}

fn structural_native_endpoint_bucket_report(
    segments: &[ArrangedNativeSegment],
    endpoints: &[ArrangedLineEndpoint],
) -> LineSegmentEndpointGraphReportParts {
    let mut buckets: Vec<(Point2, usize)> = Vec::new();
    for endpoint in endpoints {
        let point = arranged_native_endpoint_point(segments, *endpoint);
        if let Some((_, count)) = buckets
            .iter_mut()
            .find(|(bucket_point, _)| bucket_point == point)
        {
            *count += 1;
        } else {
            buckets.push((point.clone(), 1));
        }
    }

    LineSegmentEndpointGraphReportParts {
        endpoint_count: endpoints.len(),
        structural_bucket_count: buckets.len(),
        structural_singleton_bucket_count: buckets.iter().filter(|(_, count)| *count == 1).count(),
        max_structural_bucket_size: buckets.iter().map(|(_, count)| *count).max().unwrap_or(0),
        ..LineSegmentEndpointGraphReportParts::default()
    }
}

fn arranged_native_endpoints(segments: &[ArrangedNativeSegment]) -> Vec<ArrangedLineEndpoint> {
    let mut endpoints = Vec::with_capacity(segments.len() * 2);
    for segment_index in 0..segments.len() {
        endpoints.push(ArrangedLineEndpoint {
            segment_index,
            endpoint: EndpointCandidate::Start,
        });
        endpoints.push(ArrangedLineEndpoint {
            segment_index,
            endpoint: EndpointCandidate::End,
        });
    }
    endpoints
}

fn arranged_native_endpoint_point(
    segments: &[ArrangedNativeSegment],
    endpoint: ArrangedLineEndpoint,
) -> &Point2 {
    match endpoint.endpoint {
        EndpointCandidate::Start => segments[endpoint.segment_index].segment.start(),
        EndpointCandidate::End => segments[endpoint.segment_index].segment.end(),
    }
}

fn arrange_line_segments_at_point_intersections(
    segments: &[LineSeg2],
    policy: &CurvePolicy,
) -> CurveResult<Result<ArrangedLineSegments, (LineSegmentSplitReportParts, UncertaintyReason)>> {
    let mut report = LineSegmentSplitReportParts {
        predicate_path: Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine),
        candidate_pair_count: segments
            .len()
            .saturating_mul(segments.len().saturating_sub(1))
            / 2,
        ..LineSegmentSplitReportParts::default()
    };
    let mut markers = segments
        .iter()
        .map(|_| {
            vec![
                LineSegmentSplitMarker {
                    param: Real::zero(),
                },
                LineSegmentSplitMarker { param: Real::one() },
            ]
        })
        .collect::<Vec<_>>();
    let segment_boxes = segments
        .iter()
        .map(|line| match Aabb2::from_line(line, policy) {
            Classification::Decided(bbox) => Some(bbox),
            Classification::Uncertain(_) => None,
        })
        .collect::<Vec<_>>();

    for (first_index, first) in segments.iter().enumerate() {
        for (second_offset, second) in segments[first_index + 1..].iter().enumerate() {
            let second_index = first_index + 1 + second_offset;
            if let (Some(first_box), Some(second_box)) =
                (&segment_boxes[first_index], &segment_boxes[second_index])
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                report.skipped_aabb_pair_count += 1;
                continue;
            }
            report.tested_pair_count += 1;
            match first.intersect_line(second, policy)? {
                LineLineIntersection::None => {}
                LineLineIntersection::Point {
                    point,
                    a_param,
                    b_param,
                    ..
                } => {
                    report.point_relation_count += 1;
                    report.intersection_event_count += 1;
                    report.intersection_points.push(point.clone());
                    report
                        .intersection_reports
                        .push(RegionLineSegmentSplitIntersectionReport2 {
                            first_source_segment_index: first_index,
                            first_source_segment_kind: SegmentKind::Line,
                            first_source_segment_start_point: first.start().clone(),
                            first_source_segment_end_point: first.end().clone(),
                            first_source_param: a_param.clone(),
                            second_source_segment_index: second_index,
                            second_source_segment_kind: SegmentKind::Line,
                            second_source_segment_start_point: second.start().clone(),
                            second_source_segment_end_point: second.end().clone(),
                            second_source_param: b_param.clone(),
                            point,
                        });
                    if insert_line_split_marker(&mut markers[first_index], a_param, policy)
                        .is_none()
                        || insert_line_split_marker(&mut markers[second_index], b_param, policy)
                            .is_none()
                    {
                        set_split_blocker_pair(
                            &mut report,
                            first_index,
                            SegmentKind::Line,
                            first.start(),
                            first.end(),
                            second_index,
                            SegmentKind::Line,
                            second.start(),
                            second.end(),
                        );
                        return Ok(Err((report, UncertaintyReason::Ordering)));
                    }
                }
                LineLineIntersection::Overlap { .. } => {
                    report.overlap_relation_count += 1;
                    set_split_blocker_pair(
                        &mut report,
                        first_index,
                        SegmentKind::Line,
                        first.start(),
                        first.end(),
                        second_index,
                        SegmentKind::Line,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((report, UncertaintyReason::Boundary)));
                }
                LineLineIntersection::Uncertain { reason } => {
                    report.uncertain_relation_count += 1;
                    set_split_blocker_pair(
                        &mut report,
                        first_index,
                        SegmentKind::Line,
                        first.start(),
                        first.end(),
                        second_index,
                        SegmentKind::Line,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((report, reason)));
                }
            }
        }
    }

    let mut arranged = Vec::new();
    for (source_segment_index, (line, source_markers)) in
        segments.iter().zip(markers.iter_mut()).enumerate()
    {
        sort_line_split_markers(source_markers, policy).ok_or(CurveError::Topology(
            "line split markers could not be sorted".into(),
        ))?;
        for pair in source_markers.windows(2) {
            let start_param = pair[0].param.clone();
            let end_param = pair[1].param.clone();
            match compare_reals(&start_param, &end_param, policy) {
                Some(Ordering::Less) => {
                    arranged.push(ArrangedLineSegment {
                        source_segment_index,
                        source_segment_start_point: line.start().clone(),
                        source_segment_end_point: line.end().clone(),
                        source_range: ParamRange::new(start_param.clone(), end_param.clone()),
                        line: LineSeg2::try_new(
                            line.point_at(start_param),
                            line.point_at(end_param),
                        )?,
                    });
                }
                Some(Ordering::Equal) => {}
                Some(Ordering::Greater) => return Ok(Err((report, UncertaintyReason::Ordering))),
                None => return Ok(Err((report, UncertaintyReason::Ordering))),
            }
        }
    }

    report.output_segment_count = Some(arranged.len());
    Ok(Ok(ArrangedLineSegments {
        segments: arranged,
        report,
    }))
}

fn insert_line_split_marker(
    markers: &mut Vec<LineSegmentSplitMarker>,
    param: Real,
    policy: &CurvePolicy,
) -> Option<()> {
    for marker in markers.iter() {
        if compare_reals(&marker.param, &param, policy)? == Ordering::Equal {
            return Some(());
        }
    }
    markers.push(LineSegmentSplitMarker { param });
    Some(())
}

fn sort_line_split_markers(
    markers: &mut [LineSegmentSplitMarker],
    policy: &CurvePolicy,
) -> Option<()> {
    let mut failed = false;
    markers.sort_by(|left, right| {
        compare_reals(&left.param, &right.param, policy).unwrap_or_else(|| {
            failed = true;
            Ordering::Equal
        })
    });
    (!failed).then_some(())
}

fn set_split_blocker_pair(
    report: &mut LineSegmentSplitReportParts,
    first_source_segment_index: usize,
    first_source_segment_kind: SegmentKind,
    first_source_start_point: &Point2,
    first_source_end_point: &Point2,
    second_source_segment_index: usize,
    second_source_segment_kind: SegmentKind,
    second_source_start_point: &Point2,
    second_source_end_point: &Point2,
) {
    if report.blocker_first_source_segment_index.is_none() {
        report.blocker_first_source_segment_index = Some(first_source_segment_index);
        report.blocker_first_source_segment_kind = Some(first_source_segment_kind);
        report.blocker_first_source_start_point = Some(first_source_start_point.clone());
        report.blocker_first_source_end_point = Some(first_source_end_point.clone());
        report.blocker_second_source_segment_index = Some(second_source_segment_index);
        report.blocker_second_source_segment_kind = Some(second_source_segment_kind);
        report.blocker_second_source_start_point = Some(second_source_start_point.clone());
        report.blocker_second_source_end_point = Some(second_source_end_point.clone());
    }
}

fn arrange_native_segments_at_point_intersections(
    segments: &[Segment2],
    policy: &CurvePolicy,
) -> CurveResult<Result<ArrangedNativeSegments, (LineSegmentSplitReportParts, UncertaintyReason)>> {
    let mut report = LineSegmentSplitReportParts {
        predicate_path: Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment),
        candidate_pair_count: segments
            .len()
            .saturating_mul(segments.len().saturating_sub(1))
            / 2,
        ..LineSegmentSplitReportParts::default()
    };
    let mut markers = segments
        .iter()
        .map(|segment| {
            vec![
                NativeSegmentSplitMarker {
                    param: Real::zero(),
                    point: segment.start().clone(),
                },
                NativeSegmentSplitMarker {
                    param: Real::one(),
                    point: segment.end().clone(),
                },
            ]
        })
        .collect::<Vec<_>>();
    let segment_boxes = segments
        .iter()
        .map(|segment| match Aabb2::from_segment(segment, policy) {
            Ok(Classification::Decided(bbox)) => Some(bbox),
            Ok(Classification::Uncertain(_)) | Err(_) => None,
        })
        .collect::<Vec<_>>();

    for (first_index, first) in segments.iter().enumerate() {
        for (second_offset, second) in segments[first_index + 1..].iter().enumerate() {
            let second_index = first_index + 1 + second_offset;
            if let (Some(first_box), Some(second_box)) =
                (&segment_boxes[first_index], &segment_boxes[second_index])
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                report.skipped_aabb_pair_count += 1;
                continue;
            }
            report.tested_pair_count += 1;
            match native_segment_intersection_split_markers(first, second, policy)? {
                NativeSegmentIntersectionMarkers::None => {}
                NativeSegmentIntersectionMarkers::Points(points) => {
                    report.point_relation_count += 1;
                    report.intersection_event_count += points.len();
                    for point in points {
                        report.intersection_points.push(point.point.clone());
                        report.intersection_reports.push(
                            RegionLineSegmentSplitIntersectionReport2 {
                                first_source_segment_index: first_index,
                                first_source_segment_kind: first.structural_facts().kind,
                                first_source_segment_start_point: first.start().clone(),
                                first_source_segment_end_point: first.end().clone(),
                                first_source_param: point.first_param.clone(),
                                second_source_segment_index: second_index,
                                second_source_segment_kind: second.structural_facts().kind,
                                second_source_segment_start_point: second.start().clone(),
                                second_source_segment_end_point: second.end().clone(),
                                second_source_param: point.second_param.clone(),
                                point: point.point.clone(),
                            },
                        );
                        if insert_native_split_marker(
                            &mut markers[first_index],
                            NativeSegmentSplitMarker {
                                param: point.first_param,
                                point: point.point.clone(),
                            },
                            policy,
                        )
                        .is_none()
                            || insert_native_split_marker(
                                &mut markers[second_index],
                                NativeSegmentSplitMarker {
                                    param: point.second_param,
                                    point: point.point,
                                },
                                policy,
                            )
                            .is_none()
                        {
                            set_split_blocker_pair(
                                &mut report,
                                first_index,
                                first.structural_facts().kind,
                                first.start(),
                                first.end(),
                                second_index,
                                second.structural_facts().kind,
                                second.start(),
                                second.end(),
                            );
                            return Ok(Err((report, UncertaintyReason::Ordering)));
                        }
                    }
                }
                NativeSegmentIntersectionMarkers::Overlap => {
                    report.overlap_relation_count += 1;
                    set_split_blocker_pair(
                        &mut report,
                        first_index,
                        first.structural_facts().kind,
                        first.start(),
                        first.end(),
                        second_index,
                        second.structural_facts().kind,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((report, UncertaintyReason::Boundary)));
                }
                NativeSegmentIntersectionMarkers::Uncertain(reason) => {
                    report.uncertain_relation_count += 1;
                    set_split_blocker_pair(
                        &mut report,
                        first_index,
                        first.structural_facts().kind,
                        first.start(),
                        first.end(),
                        second_index,
                        second.structural_facts().kind,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((report, reason)));
                }
            }
        }
    }

    let mut arranged = Vec::new();
    for (source_segment_index, (segment, source_markers)) in
        segments.iter().zip(markers.iter_mut()).enumerate()
    {
        sort_native_split_markers(source_markers, policy).ok_or(CurveError::Topology(
            "native split markers could not be sorted".into(),
        ))?;
        for pair in source_markers.windows(2) {
            let start_param = pair[0].param.clone();
            let end_param = pair[1].param.clone();
            match compare_reals(&start_param, &end_param, policy) {
                Some(Ordering::Less) => {
                    match materialize_native_segment_between_markers(
                        segment, &pair[0], &pair[1], policy,
                    )? {
                        NativeSegmentMaterialization::Materialized(fragment) => {
                            arranged.push(ArrangedNativeSegment {
                                source_segment_index,
                                source_segment_start_point: segment.start().clone(),
                                source_segment_end_point: segment.end().clone(),
                                source_range: ParamRange::new(start_param, end_param),
                                segment: fragment,
                            });
                        }
                        NativeSegmentMaterialization::SkippedEmpty => {}
                        NativeSegmentMaterialization::Unresolved(reason) => {
                            return Ok(Err((report, reason)));
                        }
                    }
                }
                Some(Ordering::Equal) => {}
                Some(Ordering::Greater) | None => {
                    return Ok(Err((report, UncertaintyReason::Ordering)));
                }
            }
        }
    }

    report.output_segment_count = Some(arranged.len());
    Ok(Ok(ArrangedNativeSegments {
        segments: arranged,
        report,
    }))
}

#[derive(Clone, Debug, PartialEq)]
struct NativeSegmentIntersectionPoint {
    point: Point2,
    first_param: Real,
    second_param: Real,
}

#[derive(Clone, Debug, PartialEq)]
enum NativeSegmentIntersectionMarkers {
    None,
    Points(Vec<NativeSegmentIntersectionPoint>),
    Overlap,
    Uncertain(UncertaintyReason),
}

fn native_segment_intersection_split_markers(
    first: &Segment2,
    second: &Segment2,
    policy: &CurvePolicy,
) -> CurveResult<NativeSegmentIntersectionMarkers> {
    match first.intersect_segment(second, policy)? {
        SegmentIntersection::LineLine(LineLineIntersection::None) => {
            Ok(NativeSegmentIntersectionMarkers::None)
        }
        SegmentIntersection::LineLine(LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            ..
        }) => Ok(NativeSegmentIntersectionMarkers::Points(vec![
            NativeSegmentIntersectionPoint {
                point,
                first_param: a_param,
                second_param: b_param,
            },
        ])),
        SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. }) => {
            Ok(NativeSegmentIntersectionMarkers::Overlap)
        }
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
            Ok(NativeSegmentIntersectionMarkers::Uncertain(reason))
        }
        SegmentIntersection::LineArc { order, result } => {
            native_line_arc_intersection_split_markers(order, result)
        }
        SegmentIntersection::ArcArc(result) => native_arc_arc_intersection_split_markers(result),
    }
}

fn native_line_arc_intersection_split_markers(
    order: LineArcOrder,
    result: LineArcIntersection,
) -> CurveResult<NativeSegmentIntersectionMarkers> {
    let map_point = |hit: crate::LineArcIntersectionPoint| {
        let (first_param, second_param) = match order {
            LineArcOrder::LineThenArc => (hit.line_param, hit.arc_param),
            LineArcOrder::ArcThenLine => (hit.arc_param, hit.line_param),
        };
        NativeSegmentIntersectionPoint {
            point: hit.point,
            first_param,
            second_param,
        }
    };

    Ok(match result {
        LineArcIntersection::None => NativeSegmentIntersectionMarkers::None,
        LineArcIntersection::Point(hit) => {
            NativeSegmentIntersectionMarkers::Points(vec![map_point(hit)])
        }
        LineArcIntersection::TwoPoints { first, second } => {
            NativeSegmentIntersectionMarkers::Points(vec![map_point(first), map_point(second)])
        }
        LineArcIntersection::Uncertain { reason } => {
            NativeSegmentIntersectionMarkers::Uncertain(reason)
        }
    })
}

fn native_arc_arc_intersection_split_markers(
    result: ArcArcIntersection,
) -> CurveResult<NativeSegmentIntersectionMarkers> {
    Ok(match result {
        ArcArcIntersection::None => NativeSegmentIntersectionMarkers::None,
        ArcArcIntersection::Point(hit) => {
            NativeSegmentIntersectionMarkers::Points(vec![NativeSegmentIntersectionPoint {
                point: hit.point,
                first_param: hit.a_param,
                second_param: hit.b_param,
            }])
        }
        ArcArcIntersection::TwoPoints { first, second } => {
            NativeSegmentIntersectionMarkers::Points(vec![
                NativeSegmentIntersectionPoint {
                    point: first.point,
                    first_param: first.a_param,
                    second_param: first.b_param,
                },
                NativeSegmentIntersectionPoint {
                    point: second.point,
                    first_param: second.a_param,
                    second_param: second.b_param,
                },
            ])
        }
        ArcArcIntersection::Overlap { .. } => NativeSegmentIntersectionMarkers::Overlap,
        ArcArcIntersection::Uncertain { reason } => {
            NativeSegmentIntersectionMarkers::Uncertain(reason)
        }
    })
}

fn insert_native_split_marker(
    markers: &mut Vec<NativeSegmentSplitMarker>,
    marker: NativeSegmentSplitMarker,
    policy: &CurvePolicy,
) -> Option<()> {
    for existing in markers.iter() {
        if compare_reals(&existing.param, &marker.param, policy)? == Ordering::Equal {
            return match crate::classify::is_zero(
                &existing.point.distance_squared(&marker.point),
                policy,
            ) {
                Some(true) => Some(()),
                Some(false) | None => None,
            };
        }
    }
    markers.push(marker);
    Some(())
}

fn sort_native_split_markers(
    markers: &mut [NativeSegmentSplitMarker],
    policy: &CurvePolicy,
) -> Option<()> {
    let mut failed = false;
    markers.sort_by(|left, right| {
        compare_reals(&left.param, &right.param, policy).unwrap_or_else(|| {
            failed = true;
            Ordering::Equal
        })
    });
    (!failed).then_some(())
}

enum NativeSegmentMaterialization {
    Materialized(Segment2),
    SkippedEmpty,
    Unresolved(UncertaintyReason),
}

fn materialize_native_segment_between_markers(
    source_segment: &Segment2,
    start: &NativeSegmentSplitMarker,
    end: &NativeSegmentSplitMarker,
    policy: &CurvePolicy,
) -> CurveResult<NativeSegmentMaterialization> {
    match crate::classify::is_zero(&start.point.distance_squared(&end.point), policy) {
        Some(true) => return Ok(NativeSegmentMaterialization::SkippedEmpty),
        Some(false) => {}
        None => {
            return Ok(NativeSegmentMaterialization::Unresolved(
                UncertaintyReason::RealSign,
            ));
        }
    }

    match source_segment {
        Segment2::Line(_) => LineSeg2::try_new(start.point.clone(), end.point.clone())
            .map(Segment2::Line)
            .map(NativeSegmentMaterialization::Materialized),
        Segment2::Arc(arc) => {
            materialize_arc_between_markers(arc, &start.point, &end.point, policy)
        }
    }
}

fn materialize_arc_between_markers(
    source_arc: &CircularArc2,
    start: &Point2,
    end: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<NativeSegmentMaterialization> {
    match (
        source_arc.contains_point(start, policy),
        source_arc.contains_point(end, policy),
    ) {
        (Classification::Decided(true), Classification::Decided(true)) => {
            Ok(NativeSegmentMaterialization::Materialized(Segment2::Arc(
                CircularArc2::new_unchecked_with_radius(
                    start.clone(),
                    end.clone(),
                    source_arc.center().clone(),
                    source_arc.radius_squared(),
                    source_arc.is_clockwise(),
                    None,
                ),
            )))
        }
        (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {
            Err(CurveError::InvalidCurveRange)
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Ok(NativeSegmentMaterialization::Unresolved(reason))
        }
    }
}

fn assemble_unordered_line_segment_rings(
    segments: &[ArrangedLineSegment],
    policy: &CurvePolicy,
) -> CurveResult<
    Result<LineSegmentRingAssembly, (LineSegmentRingAssemblyReportParts, UncertaintyReason)>,
> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    let mut counts = LineSegmentRingAssemblyCounts::default();
    let mut reversed_source_segment_count = 0_usize;
    let mut source_reports = Vec::with_capacity(segments.len());

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let output_ring_index = rings.len();
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        append_line_segment_ring_source_report(
            &mut source_reports,
            &current,
            output_ring_index,
            ring.len(),
            false,
        );
        let ring_start = current.line.start().clone();
        ring.push(current.line.clone());

        loop {
            match exact_points_match(current.line.end(), &ring_start, policy, &mut counts) {
                Classification::Decided(true) => break,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        reason,
                    )));
                }
            }

            let next = match unique_next_line_segment(
                current.line.end(),
                segments,
                &used,
                policy,
                &mut counts,
            ) {
                Classification::Decided(Some(next)) => next,
                Classification::Decided(None) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        UncertaintyReason::Boundary,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        reason,
                    )));
                }
            };

            used[next.arranged_segment_index] = true;
            if next.reversed {
                reversed_source_segment_count += 1;
            }
            current = if next.reversed {
                segments[next.arranged_segment_index].reversed()
            } else {
                segments[next.arranged_segment_index].clone()
            };
            append_line_segment_ring_source_report(
                &mut source_reports,
                &current,
                output_ring_index,
                ring.len(),
                next.reversed,
            );
            ring.push(current.line.clone());
        }

        if ring.len() < 3 {
            return Ok(Err((
                LineSegmentRingAssemblyReportParts {
                    counts,
                    reversed_source_segment_count,
                    source_reports,
                },
                UncertaintyReason::Boundary,
            )));
        }
        rings.push(ring);
    }

    Ok(Ok(LineSegmentRingAssembly {
        rings,
        counts,
        reversed_source_segment_count,
        source_reports,
    }))
}

#[derive(Clone, Debug, PartialEq)]
struct NextLineSegment {
    arranged_segment_index: usize,
    reversed: bool,
}

fn unique_next_line_segment(
    target: &Point2,
    segments: &[ArrangedLineSegment],
    used: &[bool],
    policy: &CurvePolicy,
    counts: &mut LineSegmentRingAssemblyCounts,
) -> Classification<Option<NextLineSegment>> {
    let mut selected = None;
    for (arranged_segment_index, segment) in segments.iter().enumerate() {
        if used[arranged_segment_index] {
            continue;
        }
        for candidate in [EndpointCandidate::Start, EndpointCandidate::End] {
            let point = match candidate {
                EndpointCandidate::Start => segment.line.start(),
                EndpointCandidate::End => segment.line.end(),
            };
            match exact_points_match(target, point, policy, counts) {
                Classification::Decided(true) => {
                    if selected.is_some() {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    selected = Some(NextLineSegment {
                        arranged_segment_index,
                        reversed: candidate == EndpointCandidate::End,
                    });
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(selected)
}

#[derive(Clone, Debug, PartialEq)]
struct NativeSegmentRingAssembly {
    rings: Vec<Vec<Segment2>>,
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_reports: Vec<RegionLineSegmentRingSourceReport2>,
}

fn assemble_unordered_native_segment_rings(
    segments: &[ArrangedNativeSegment],
    policy: &CurvePolicy,
) -> CurveResult<
    Result<NativeSegmentRingAssembly, (LineSegmentRingAssemblyReportParts, UncertaintyReason)>,
> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    let mut counts = LineSegmentRingAssemblyCounts::default();
    let mut reversed_source_segment_count = 0_usize;
    let mut source_reports = Vec::with_capacity(segments.len());

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let output_ring_index = rings.len();
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        append_native_segment_ring_source_report(
            &mut source_reports,
            &current,
            output_ring_index,
            ring.len(),
            false,
        );
        let ring_start = current.segment.start().clone();
        ring.push(current.segment.clone());

        loop {
            match exact_points_match(current.segment.end(), &ring_start, policy, &mut counts) {
                Classification::Decided(true) => break,
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        reason,
                    )));
                }
            }

            let next = match unique_next_native_segment(
                current.segment.end(),
                segments,
                &used,
                policy,
                &mut counts,
            ) {
                Classification::Decided(Some(next)) => next,
                Classification::Decided(None) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        UncertaintyReason::Boundary,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyReportParts {
                            counts,
                            reversed_source_segment_count,
                            source_reports,
                        },
                        reason,
                    )));
                }
            };

            used[next.arranged_segment_index] = true;
            if next.reversed {
                reversed_source_segment_count += 1;
            }
            current = if next.reversed {
                segments[next.arranged_segment_index].reversed()
            } else {
                segments[next.arranged_segment_index].clone()
            };
            append_native_segment_ring_source_report(
                &mut source_reports,
                &current,
                output_ring_index,
                ring.len(),
                next.reversed,
            );
            ring.push(current.segment.clone());
        }

        rings.push(ring);
    }

    Ok(Ok(NativeSegmentRingAssembly {
        rings,
        counts,
        reversed_source_segment_count,
        source_reports,
    }))
}

fn unique_next_native_segment(
    target: &Point2,
    segments: &[ArrangedNativeSegment],
    used: &[bool],
    policy: &CurvePolicy,
    counts: &mut LineSegmentRingAssemblyCounts,
) -> Classification<Option<NextLineSegment>> {
    let mut selected = None;
    for (arranged_segment_index, segment) in segments.iter().enumerate() {
        if used[arranged_segment_index] {
            continue;
        }
        for candidate in [EndpointCandidate::Start, EndpointCandidate::End] {
            let point = match candidate {
                EndpointCandidate::Start => segment.segment.start(),
                EndpointCandidate::End => segment.segment.end(),
            };
            match exact_points_match(target, point, policy, counts) {
                Classification::Decided(true) => {
                    if selected.is_some() {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    selected = Some(NextLineSegment {
                        arranged_segment_index,
                        reversed: candidate == EndpointCandidate::End,
                    });
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(selected)
}

fn exact_points_match(
    left: &Point2,
    right: &Point2,
    policy: &CurvePolicy,
    counts: &mut LineSegmentRingAssemblyCounts,
) -> Classification<bool> {
    counts.attempted_endpoint_connection_count += 1;
    match crate::classify::is_zero(&left.distance_squared(right), policy) {
        Some(true) => {
            counts.exact_endpoint_connection_count += 1;
            Classification::Decided(true)
        }
        Some(false) => {
            counts.disconnected_endpoint_connection_count += 1;
            Classification::Decided(false)
        }
        None => {
            counts.unresolved_endpoint_connection_count += 1;
            Classification::Uncertain(UncertaintyReason::RealSign)
        }
    }
}

fn append_line_segment_ring_source_report(
    source_reports: &mut Vec<RegionLineSegmentRingSourceReport2>,
    segment: &ArrangedLineSegment,
    output_ring_index: usize,
    output_segment_index: usize,
    reversed: bool,
) {
    source_reports.push(RegionLineSegmentRingSourceReport2 {
        source_segment_index: segment.source_segment_index,
        source_segment_kind: SegmentKind::Line,
        source_segment_start_point: segment.source_segment_start_point.clone(),
        source_segment_end_point: segment.source_segment_end_point.clone(),
        source_range: segment.source_range.clone(),
        output_ring_index,
        output_segment_index,
        output_segment_kind: SegmentKind::Line,
        reversed,
        output_start_point: segment.line.start().clone(),
        output_end_point: segment.line.end().clone(),
        status: RetainedTopologyStatus::NativeExact,
    });
}

fn append_native_segment_ring_source_report(
    source_reports: &mut Vec<RegionLineSegmentRingSourceReport2>,
    segment: &ArrangedNativeSegment,
    output_ring_index: usize,
    output_segment_index: usize,
    reversed: bool,
) {
    source_reports.push(RegionLineSegmentRingSourceReport2 {
        source_segment_index: segment.source_segment_index,
        source_segment_kind: segment.segment.structural_facts().kind,
        source_segment_start_point: segment.source_segment_start_point.clone(),
        source_segment_end_point: segment.source_segment_end_point.clone(),
        source_range: segment.source_range.clone(),
        output_ring_index,
        output_segment_index,
        output_segment_kind: segment.segment.structural_facts().kind,
        reversed,
        output_start_point: segment.segment.start().clone(),
        output_end_point: segment.segment.end().clone(),
        status: RetainedTopologyStatus::NativeExact,
    });
}

fn line_arranged_source_reports(
    segments: &[ArrangedLineSegment],
) -> Vec<RegionLineSegmentArrangedSourceReport2> {
    segments
        .iter()
        .enumerate()
        .map(
            |(arranged_segment_index, segment)| RegionLineSegmentArrangedSourceReport2 {
                source_segment_index: segment.source_segment_index,
                source_segment_kind: SegmentKind::Line,
                source_segment_start_point: segment.source_segment_start_point.clone(),
                source_segment_end_point: segment.source_segment_end_point.clone(),
                source_range: segment.source_range.clone(),
                arranged_segment_index,
                arranged_segment_kind: SegmentKind::Line,
                output_start_point: segment.line.start().clone(),
                output_end_point: segment.line.end().clone(),
                status: RetainedTopologyStatus::NativeExact,
            },
        )
        .collect()
}

fn native_arranged_source_reports(
    source_segments: &[Segment2],
    segments: &[ArrangedNativeSegment],
) -> Vec<RegionLineSegmentArrangedSourceReport2> {
    segments
        .iter()
        .enumerate()
        .map(
            |(arranged_segment_index, segment)| RegionLineSegmentArrangedSourceReport2 {
                source_segment_index: segment.source_segment_index,
                source_segment_kind: source_segments[segment.source_segment_index]
                    .structural_facts()
                    .kind,
                source_segment_start_point: source_segments[segment.source_segment_index]
                    .start()
                    .clone(),
                source_segment_end_point: source_segments[segment.source_segment_index]
                    .end()
                    .clone(),
                source_range: segment.source_range.clone(),
                arranged_segment_index,
                arranged_segment_kind: segment.segment.structural_facts().kind,
                output_start_point: segment.segment.start().clone(),
                output_end_point: segment.segment.end().clone(),
                status: RetainedTopologyStatus::NativeExact,
            },
        )
        .collect()
}

fn line_segment_kind_counts(segment_count: usize) -> SegmentKindCounts {
    SegmentKindCounts {
        lines: segment_count,
        arcs: 0,
    }
}

fn source_range_is_full(source_range: &ParamRange) -> bool {
    source_range.start() == &Real::zero() && source_range.end() == &Real::one()
}

fn segment_kind_counts(segments: &[Segment2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        add_segment_kind(&mut counts, segment);
    }
    counts
}

fn source_segment_aabbs(
    segments: &[Segment2],
    policy: &CurvePolicy,
) -> CurveResult<Vec<Option<Aabb2>>> {
    segments
        .iter()
        .map(|segment| match Aabb2::from_segment(segment, policy)? {
            Classification::Decided(bbox) => Ok(Some(bbox)),
            Classification::Uncertain(_) => Ok(None),
        })
        .collect()
}

fn source_endpoint_bucket_cache(
    segments: &[Segment2],
) -> ExactCurveArrangementSourceEndpointBucketCache2 {
    let mut buckets: Vec<ExactCurveArrangementSourceEndpointBucket2> = Vec::new();
    for (source_segment_index, segment) in segments.iter().enumerate() {
        add_source_endpoint_bucket_ref(
            &mut buckets,
            segment.start(),
            ExactCurveArrangementSourceEndpointRef2 {
                source_segment_index,
                endpoint: ExactCurveArrangementSourceEndpoint2::Start,
            },
        );
        add_source_endpoint_bucket_ref(
            &mut buckets,
            segment.end(),
            ExactCurveArrangementSourceEndpointRef2 {
                source_segment_index,
                endpoint: ExactCurveArrangementSourceEndpoint2::End,
            },
        );
    }

    let endpoint_count = segments.len() * 2;
    let bucket_count = buckets.len();
    let singleton_bucket_count = buckets
        .iter()
        .filter(|bucket| bucket.endpoints.len() == 1)
        .count();
    let max_bucket_size = buckets
        .iter()
        .map(|bucket| bucket.endpoints.len())
        .max()
        .unwrap_or(0);
    ExactCurveArrangementSourceEndpointBucketCache2 {
        endpoint_count,
        bucket_count,
        singleton_bucket_count,
        max_bucket_size,
        buckets,
    }
}

fn add_source_endpoint_bucket_ref(
    buckets: &mut Vec<ExactCurveArrangementSourceEndpointBucket2>,
    point: &Point2,
    endpoint_ref: ExactCurveArrangementSourceEndpointRef2,
) {
    if let Some(bucket) = buckets.iter_mut().find(|bucket| bucket.point() == point) {
        bucket.endpoints.push(endpoint_ref);
    } else {
        buckets.push(ExactCurveArrangementSourceEndpointBucket2 {
            point: point.clone(),
            endpoints: vec![endpoint_ref],
        });
    }
}

fn split_schedule_cache(
    source_segment_aabbs: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> ExactCurveArrangementSplitScheduleCache2 {
    let candidate_pair_count = source_segment_aabbs
        .len()
        .saturating_mul(source_segment_aabbs.len().saturating_sub(1))
        / 2;
    let mut decided_disjoint_pair_count = 0_usize;
    let mut undecided_aabb_pair_count = 0_usize;
    let mut candidate_pairs = Vec::with_capacity(candidate_pair_count);

    for first_source_segment_index in 0..source_segment_aabbs.len() {
        for second_source_segment_index in
            first_source_segment_index + 1..source_segment_aabbs.len()
        {
            let aabb_status = match (
                &source_segment_aabbs[first_source_segment_index],
                &source_segment_aabbs[second_source_segment_index],
            ) {
                (Some(first), Some(second)) if aabbs_decided_disjoint(first, second, policy) => {
                    decided_disjoint_pair_count += 1;
                    ExactCurveArrangementSplitCandidateAabbStatus2::DecidedDisjoint
                }
                (Some(_), Some(_)) => {
                    ExactCurveArrangementSplitCandidateAabbStatus2::NotDecidedDisjoint
                }
                _ => {
                    undecided_aabb_pair_count += 1;
                    ExactCurveArrangementSplitCandidateAabbStatus2::Undecided
                }
            };
            candidate_pairs.push(ExactCurveArrangementSplitCandidatePair2 {
                first_source_segment_index,
                second_source_segment_index,
                aabb_status,
            });
        }
    }

    let bucket_cache =
        ExactCurveArrangementSplitScheduleBucketCache2::from_candidate_pairs(&candidate_pairs);

    ExactCurveArrangementSplitScheduleCache2 {
        candidate_pair_count,
        decided_disjoint_pair_count,
        predicate_candidate_pair_count: candidate_pair_count
            .saturating_sub(decided_disjoint_pair_count),
        undecided_aabb_pair_count,
        bucket_cache,
        candidate_pairs,
    }
}

fn split_intersection_bucket_cache(
    intersection_reports: &[RegionLineSegmentSplitIntersectionReport2],
) -> ExactCurveArrangementSplitIntersectionBucketCache2 {
    let mut buckets: Vec<ExactCurveArrangementSplitIntersectionBucket2> = Vec::new();
    for (intersection_report_index, report) in intersection_reports.iter().enumerate() {
        add_split_intersection_bucket_ref(
            &mut buckets,
            report.point(),
            ExactCurveArrangementSplitIntersectionRef2 {
                intersection_report_index,
            },
        );
    }

    let intersection_event_count = intersection_reports.len();
    let bucket_count = buckets.len();
    let singleton_bucket_count = buckets
        .iter()
        .filter(|bucket| bucket.intersections.len() == 1)
        .count();
    let max_bucket_size = buckets
        .iter()
        .map(|bucket| bucket.intersections.len())
        .max()
        .unwrap_or(0);
    ExactCurveArrangementSplitIntersectionBucketCache2 {
        intersection_event_count,
        bucket_count,
        singleton_bucket_count,
        max_bucket_size,
        buckets,
    }
}

fn add_split_intersection_bucket_ref(
    buckets: &mut Vec<ExactCurveArrangementSplitIntersectionBucket2>,
    point: &Point2,
    intersection_ref: ExactCurveArrangementSplitIntersectionRef2,
) {
    if let Some(bucket) = buckets.iter_mut().find(|bucket| bucket.point() == point) {
        bucket.intersections.push(intersection_ref);
    } else {
        buckets.push(ExactCurveArrangementSplitIntersectionBucket2 {
            point: point.clone(),
            intersections: vec![intersection_ref],
        });
    }
}

fn arranged_endpoint_bucket_cache(
    arranged_source_reports: &[RegionLineSegmentArrangedSourceReport2],
) -> ExactCurveArrangementArrangedEndpointBucketCache2 {
    let mut buckets: Vec<ExactCurveArrangementArrangedEndpointBucket2> = Vec::new();
    for report in arranged_source_reports {
        add_arranged_endpoint_bucket_ref(
            &mut buckets,
            report.output_start_point(),
            ExactCurveArrangementArrangedEndpointRef2 {
                arranged_segment_index: report.arranged_segment_index(),
                endpoint: RegionLineSegmentArrangedEndpoint2::Start,
            },
        );
        add_arranged_endpoint_bucket_ref(
            &mut buckets,
            report.output_end_point(),
            ExactCurveArrangementArrangedEndpointRef2 {
                arranged_segment_index: report.arranged_segment_index(),
                endpoint: RegionLineSegmentArrangedEndpoint2::End,
            },
        );
    }

    let endpoint_count = arranged_source_reports.len() * 2;
    let bucket_count = buckets.len();
    let singleton_bucket_count = buckets
        .iter()
        .filter(|bucket| bucket.endpoints.len() == 1)
        .count();
    let max_bucket_size = buckets
        .iter()
        .map(|bucket| bucket.endpoints.len())
        .max()
        .unwrap_or(0);
    ExactCurveArrangementArrangedEndpointBucketCache2 {
        endpoint_count,
        bucket_count,
        singleton_bucket_count,
        max_bucket_size,
        buckets,
    }
}

fn add_arranged_endpoint_bucket_ref(
    buckets: &mut Vec<ExactCurveArrangementArrangedEndpointBucket2>,
    point: &Point2,
    endpoint_ref: ExactCurveArrangementArrangedEndpointRef2,
) {
    if let Some(bucket) = buckets.iter_mut().find(|bucket| bucket.point() == point) {
        bucket.endpoints.push(endpoint_ref);
    } else {
        buckets.push(ExactCurveArrangementArrangedEndpointBucket2 {
            point: point.clone(),
            endpoints: vec![endpoint_ref],
        });
    }
}

fn union_decided_aabbs(segment_aabbs: &[Option<Aabb2>], policy: &CurvePolicy) -> Option<Aabb2> {
    if segment_aabbs.iter().any(Option::is_none) {
        return None;
    }
    let mut boxes = segment_aabbs.iter().filter_map(Option::as_ref);
    let mut source_aabb = boxes.next()?.clone();
    for bbox in boxes {
        source_aabb = match source_aabb.union(bbox, policy) {
            Classification::Decided(merged) => merged,
            Classification::Uncertain(_) => return None,
        };
    }
    Some(source_aabb)
}

fn native_arranged_segment_kind_counts(segments: &[ArrangedNativeSegment]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        add_segment_kind(&mut counts, &segment.segment);
    }
    counts
}

fn arranged_report_segment_kind_counts(
    reports: &[RegionLineSegmentArrangedSourceReport2],
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for report in reports {
        match report.arranged_segment_kind {
            SegmentKind::Line => counts.lines += 1,
            SegmentKind::Arc => counts.arcs += 1,
        }
    }
    counts
}

fn region_segment_kind_counts(region: &Region2) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .flat_map(|contour| contour.segments())
    {
        add_segment_kind(&mut counts, segment);
    }
    counts
}

fn add_segment_kind(counts: &mut SegmentKindCounts, segment: &Segment2) {
    match segment {
        Segment2::Line(_) => counts.lines += 1,
        Segment2::Arc(_) => counts.arcs += 1,
    }
}

fn blocked_line_segment_region_report(
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    split_report: Option<LineSegmentSplitReportParts>,
    endpoint_graph_report: Option<LineSegmentEndpointGraphReportParts>,
    arranged_source_reports: Vec<RegionLineSegmentArrangedSourceReport2>,
    report: LineSegmentRingAssemblyReportParts,
    stage: RegionLineSegmentRegionBuildStage2,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionLineSegmentRegionBuildReport2 {
    let split_report = split_report.unwrap_or_default();
    let arranged_segment_kind_counts = split_report
        .output_segment_count
        .map(|_| arranged_report_segment_kind_counts(&arranged_source_reports));
    RegionLineSegmentRegionBuildReport2 {
        stage,
        source_segment_count,
        source_segment_kind_counts,
        arranged_segment_count: split_report.output_segment_count,
        arranged_segment_kind_counts,
        split_predicate_path: split_report.predicate_path,
        endpoint_graph_predicate_path: endpoint_graph_report
            .as_ref()
            .map(|_| RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets),
        ring_assembly_predicate_path: endpoint_graph_report
            .as_ref()
            .map(|_| RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal),
        split_candidate_pair_count: split_report.candidate_pair_count,
        split_skipped_aabb_pair_count: split_report.skipped_aabb_pair_count,
        split_tested_pair_count: split_report.tested_pair_count,
        split_intersection_event_count: split_report.intersection_event_count,
        split_point_relation_count: split_report.point_relation_count,
        split_overlap_relation_count: split_report.overlap_relation_count,
        split_uncertain_relation_count: split_report.uncertain_relation_count,
        split_intersection_points: split_report.intersection_points,
        split_intersection_reports: split_report.intersection_reports,
        split_output_segment_count: split_report.output_segment_count,
        split_blocker_first_source_segment_index: split_report.blocker_first_source_segment_index,
        split_blocker_first_source_segment_kind: split_report.blocker_first_source_segment_kind,
        split_blocker_first_source_start_point: split_report.blocker_first_source_start_point,
        split_blocker_first_source_end_point: split_report.blocker_first_source_end_point,
        split_blocker_second_source_segment_index: split_report.blocker_second_source_segment_index,
        split_blocker_second_source_segment_kind: split_report.blocker_second_source_segment_kind,
        split_blocker_second_source_start_point: split_report.blocker_second_source_start_point,
        split_blocker_second_source_end_point: split_report.blocker_second_source_end_point,
        endpoint_graph_endpoint_count: endpoint_graph_report
            .as_ref()
            .map(|report| report.endpoint_count),
        endpoint_graph_structural_bucket_count: endpoint_graph_report
            .as_ref()
            .map(|report| report.structural_bucket_count),
        endpoint_graph_structural_singleton_bucket_count: endpoint_graph_report
            .as_ref()
            .map(|report| report.structural_singleton_bucket_count),
        endpoint_graph_max_structural_bucket_size: endpoint_graph_report
            .as_ref()
            .map(|report| report.max_structural_bucket_size),
        endpoint_graph_dangling_endpoint_count: endpoint_graph_report
            .as_ref()
            .map(|report| report.dangling_endpoint_count),
        endpoint_graph_branch_endpoint_count: endpoint_graph_report
            .as_ref()
            .map(|report| report.branch_endpoint_count),
        endpoint_graph_blocker_arranged_segment_index: endpoint_graph_report
            .as_ref()
            .and_then(|report| report.blocker_arranged_segment_index),
        endpoint_graph_blocker_endpoint: endpoint_graph_report
            .as_ref()
            .and_then(|report| report.blocker_endpoint),
        endpoint_graph_blocker_point: endpoint_graph_report
            .as_ref()
            .and_then(|report| report.blocker_point.clone()),
        attempted_endpoint_connection_count: report.counts.attempted_endpoint_connection_count,
        exact_endpoint_connection_count: report.counts.exact_endpoint_connection_count,
        disconnected_endpoint_connection_count: report
            .counts
            .disconnected_endpoint_connection_count,
        unresolved_endpoint_connection_count: report.counts.unresolved_endpoint_connection_count,
        reversed_source_segment_count: report.reversed_source_segment_count,
        output_ring_count: None,
        output_boundary_segment_count: None,
        output_boundary_segment_kind_counts: None,
        arranged_source_reports,
        source_reports: report.source_reports,
        boundary_build_report: None,
        status,
        blocker: Some(blocker),
    }
}

fn retained_status_for_line_segment_region_blocker(
    blocker: UncertaintyReason,
) -> RetainedTopologyStatus {
    match blocker {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn blocked_boundary_contour_region_result(
    source_contour_count: usize,
    source_segment_count: usize,
    counts: BoundaryContourValidationCounts,
    blocker_contour_indices: Option<(usize, usize)>,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionBoundaryContourBuildResult2 {
    let (blocker_first_contour_index, blocker_second_contour_index) =
        blocker_contour_indices.map_or((None, None), |(first, second)| (Some(first), Some(second)));
    RegionBoundaryContourBuildResult2 {
        region: None,
        report: RegionBoundaryContourBuildReport2 {
            stage: RegionBoundaryContourBuildStage2::NestingValidation,
            predicate_path:
                RegionBoundaryContourBuildPredicatePath2::ExactContourIntersectionAndPointContainment,
            source_contour_count,
            source_segment_count,
            validation_candidate_pair_count: counts.candidate_pair_count,
            validation_tested_pair_count: counts.tested_pair_count,
            validation_intersection_event_count: counts.intersection_event_count,
            nesting_classification_count: counts.nesting_classification_count,
            blocker_first_contour_index,
            blocker_second_contour_index,
            output_contour_count: None,
            output_segment_count: None,
            material_contour_count: None,
            hole_contour_count: None,
            material_segment_count: None,
            hole_segment_count: None,
            role_reports: Vec::new(),
            status,
            blocker: Some(blocker),
        },
    }
}

fn retained_status_for_boundary_contour_blocker(
    reason: UncertaintyReason,
) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn contour_nesting_depths(
    contours: &[Contour2],
    policy: &CurvePolicy,
) -> CurveResult<BoundaryContourNestingOutcome> {
    let candidate_pair_count = contours
        .len()
        .saturating_mul(contours.len().saturating_sub(1))
        / 2;
    let mut counts = BoundaryContourValidationCounts {
        candidate_pair_count,
        tested_pair_count: 0,
        intersection_event_count: 0,
        nesting_classification_count: 0,
    };

    for (left_index, left) in contours.iter().enumerate() {
        for (right_offset, right) in contours[left_index + 1..].iter().enumerate() {
            counts.tested_pair_count += 1;
            let intersections = left.intersect_contour(right, policy)?;
            counts.intersection_event_count += intersections.len();
            if !intersections.is_empty() {
                return Ok(BoundaryContourNestingOutcome::Blocked {
                    blocker: BoundaryContourNestingBlocker {
                        reason: crate::UncertaintyReason::Boundary,
                        first_contour_index: left_index,
                        second_contour_index: left_index + 1 + right_offset,
                    },
                    counts,
                });
            }
        }
    }

    let mut entries = Vec::with_capacity(contours.len());

    for (candidate_index, candidate) in contours.iter().enumerate() {
        // A point on the candidate boundary is sufficient for nesting against
        // every *other* non-touching contour. This reduces role assignment to
        // repeated point-in-polygon classification, the degeneracy-sensitive
        // problem surveyed by K. Hormann and A. Agathos, "The point in polygon
        // problem for arbitrary polygons," Computational Geometry 20(3),
        // 131-144, 2001. If that sample lies on another contour boundary, we
        // return uncertainty instead of inventing a role.
        let sample = candidate
            .segments()
            .first()
            .ok_or(CurveError::EmptyCurveString)?
            .start();
        let mut containing_contour_indices = Vec::new();

        for (container_index, container) in contours.iter().enumerate() {
            if candidate_index == container_index {
                continue;
            }

            counts.nesting_classification_count += 1;
            match container.classify_point(sample, policy) {
                Classification::Decided(ContourPointLocation::Inside) => {
                    containing_contour_indices.push(container_index);
                }
                Classification::Decided(ContourPointLocation::Outside) => {}
                Classification::Decided(ContourPointLocation::Boundary) => {
                    return Ok(BoundaryContourNestingOutcome::Blocked {
                        blocker: BoundaryContourNestingBlocker {
                            reason: crate::UncertaintyReason::Boundary,
                            first_contour_index: candidate_index,
                            second_contour_index: container_index,
                        },
                        counts,
                    });
                }
                Classification::Uncertain(reason) => {
                    return Ok(BoundaryContourNestingOutcome::Blocked {
                        blocker: BoundaryContourNestingBlocker {
                            reason,
                            first_contour_index: candidate_index,
                            second_contour_index: container_index,
                        },
                        counts,
                    });
                }
            }
        }

        entries.push(BoundaryContourNestingEntry {
            sample_point: sample.clone(),
            containing_contour_indices,
        });
    }

    Ok(BoundaryContourNestingOutcome::Decided {
        nesting: BoundaryContourNestingDepths { entries },
        counts,
    })
}
