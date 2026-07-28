//! Contour nesting and material/hole role assignment.
//!
//! This module turns already-closed boundary contours into the signed contour
//! bins used by [`crate::LineArcRegion2`]. It assumes intersections and overlaps have
//! already been resolved by earlier topology stages.
use std::{cell::OnceCell, cmp::Ordering, rc::Rc};

use hyperreal::{Real, RealSign};

use crate::bbox::{
    Aabb2, aabb_decided_misses_point, aabbs_decided_disjoint, decided_contour_aabb,
    decided_segment_aabb,
};
use crate::classify::{compare_reals, real_sign};
use crate::{
    ArcArcIntersection, CircularArc2, Classification, Contour2, ContourPointLocation, CurveError,
    CurvePolicy, CurveResult, FillRule, LineArcIntersection, LineArcOrder, LineArcRegion2,
    LineLineIntersection, LineSeg2, ParamRange, Point2, RetainedTopologyStatus, Segment2,
    SegmentIntersection, SegmentKind, SegmentKindCounts, UncertaintyReason,
};

/// Internal retained arrangement request for exact curve topology.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementRequest2 {
    source_segments: Vec<Segment2>,
    source_line_segments: Option<Vec<LineSeg2>>,
    fill_rule: FillRule,
}

/// Internal retained facts for a single exact curve arrangement attempt.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveWorkspace2 {
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

/// Source segment fact retained during workspace construction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceSegmentFact2 {
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_start_point: Point2,
    source_end_point: Point2,
    source_aabb: Option<Aabb2>,
}

/// AABB certification status retained for one source segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactCurveArrangementSourceAabbStatus2 {
    /// The source segment box was certified during workspace construction.
    Decided,
    /// The source segment box stayed uncertain during workspace construction.
    Undecided,
}

/// Reference to a retained source segment AABB fact.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceAabbRef2 {
    source_segment_index: usize,
}

/// Source segment bucket grouped by retained AABB certification status.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceAabbBucket2 {
    aabb_status: ExactCurveArrangementSourceAabbStatus2,
    source_refs: Vec<ExactCurveArrangementSourceAabbRef2>,
}

/// Source segment AABB buckets retained during workspace construction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceAabbBucketCache2 {
    bucket_count: usize,
    source_ref_count: usize,
    decided_source_ref_count: usize,
    undecided_source_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSourceAabbBucket2>,
}

/// Reference to a retained source segment fact inside a primitive-family bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceSegmentKindRef2 {
    source_segment_index: usize,
}

/// Source segment bucket grouped by retained primitive family.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceSegmentKindBucket2 {
    source_segment_kind: SegmentKind,
    source_refs: Vec<ExactCurveArrangementSourceSegmentKindRef2>,
}

/// Source segment primitive-family buckets retained during workspace construction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceSegmentKindBucketCache2 {
    bucket_count: usize,
    source_segment_ref_count: usize,
    line_segment_ref_count: usize,
    arc_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSourceSegmentKindBucket2>,
}

/// Source segment fact cache retained during workspace construction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceSegmentCache2 {
    decided_source_segment_aabb_count: usize,
    undecided_source_segment_aabb_count: usize,
    source_aabb_bucket_cache: ExactCurveArrangementSourceAabbBucketCache2,
    source_segment_kind_bucket_cache: ExactCurveArrangementSourceSegmentKindBucketCache2,
    segments: Vec<ExactCurveArrangementSourceSegmentFact2>,
}

/// Source endpoint of a retained exact arrangement input segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactCurveArrangementSourceEndpoint2 {
    /// Start point of the source segment.
    Start,
    /// End point of the source segment.
    End,
}

/// Source-segment endpoint reference retained in a source endpoint bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceEndpointRef2 {
    source_segment_index: usize,
    endpoint: ExactCurveArrangementSourceEndpoint2,
}

/// Exact structural source endpoint bucket retained during workspace construction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceEndpointBucket2 {
    point: Point2,
    endpoints: Vec<ExactCurveArrangementSourceEndpointRef2>,
}

/// Exact structural source endpoint buckets retained during workspace construction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSourceEndpointBucketCache2 {
    endpoint_count: usize,
    bucket_count: usize,
    singleton_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSourceEndpointBucket2>,
}

/// AABB pruning status retained for one scheduled source split candidate pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactCurveArrangementSplitCandidateAabbStatus2 {
    /// The source boxes were both decided and certified disjoint.
    DecidedDisjoint,
    /// The source boxes were both decided and not certified disjoint.
    NotDecidedDisjoint,
    /// One or both source boxes were not certified during workspace construction.
    Undecided,
}

/// Source segment pair scheduled for exact split predicate evaluation or AABB pruning.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitCandidatePair2 {
    first_source_segment_index: usize,
    second_source_segment_index: usize,
    aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2,
}

/// Reference to a retained scheduled split candidate pair.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitScheduleRef2 {
    candidate_pair_index: usize,
}

/// Scheduled split candidate bucket grouped by retained AABB pruning status.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitScheduleBucket2 {
    aabb_status: ExactCurveArrangementSplitCandidateAabbStatus2,
    candidate_refs: Vec<ExactCurveArrangementSplitScheduleRef2>,
}

/// Scheduled split candidate buckets grouped by retained AABB pruning status.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitScheduleBucketCache2 {
    bucket_count: usize,
    candidate_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSplitScheduleBucket2>,
}

/// Retained exact source-pair schedule used before split predicate evaluation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitScheduleCache2 {
    candidate_pair_count: usize,
    decided_disjoint_pair_count: usize,
    predicate_candidate_pair_count: usize,
    undecided_aabb_pair_count: usize,
    bucket_cache: ExactCurveArrangementSplitScheduleBucketCache2,
    candidate_pairs: Vec<ExactCurveArrangementSplitCandidatePair2>,
}

/// Retained exact split evidence cached by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitCache2 {
    predicate_path: Option<RegionLineSegmentSplitPredicatePath2>,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    intersection_event_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    intersection_points: Vec<Point2>,
    intersection_evidence: Vec<RegionLineSegmentSplitIntersectionEvidence2>,
    relation_bucket_cache: ExactCurveArrangementSplitRelationBucketCache2,
    intersection_bucket_cache: ExactCurveArrangementSplitIntersectionBucketCache2,
    intersection_parameter_cache: ExactCurveArrangementSplitIntersectionParameterCache2,
    blocker_cache: Option<ExactCurveArrangementSplitBlockerCache2>,
    output_segment_count: Option<usize>,
}

/// Retained source-pair blocker evidence from exact split arrangement.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitBlockerCache2 {
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
pub(crate) enum ExactCurveArrangementSplitRelationClass2 {
    /// Source pair relation produced exact point-intersection evidence.
    Point,
    /// Source pair relation produced exact overlap evidence.
    Overlap,
    /// Source pair relation could not be decided by the configured exact predicates.
    Uncertain,
}

/// Retained split-stage relation bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitRelationBucket2 {
    relation: ExactCurveArrangementSplitRelationClass2,
    relation_count: usize,
}

/// Retained split-stage relation buckets.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitRelationBucketCache2 {
    bucket_count: usize,
    relation_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSplitRelationBucket2>,
}

/// Reference to a retained split-intersection evidence inside an exact point bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitIntersectionRef2 {
    intersection_evidence_index: usize,
}

/// Exact structural split-intersection bucket retained by an evaluated workspace.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitIntersectionBucket2 {
    point: Point2,
    intersections: Vec<ExactCurveArrangementSplitIntersectionRef2>,
}

/// Exact structural split-intersection buckets retained by an evaluated workspace.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitIntersectionBucketCache2 {
    intersection_event_count: usize,
    bucket_count: usize,
    singleton_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementSplitIntersectionBucket2>,
}

/// Exact source-parameter evidence retained for one split intersection.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitIntersectionParameterRef2 {
    intersection_evidence_index: usize,
    first_source_segment_index: usize,
    first_source_param: Real,
    second_source_segment_index: usize,
    second_source_param: Real,
    point: Point2,
}

/// Exact source-parameter evidence retained for split intersections.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementSplitIntersectionParameterCache2 {
    intersection_event_count: usize,
    source_parameter_ref_count: usize,
    parameters: Vec<ExactCurveArrangementSplitIntersectionParameterRef2>,
}

/// Retained exact endpoint-bucket evidence cached by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementEndpointGraphCache2 {
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
pub(crate) enum ExactCurveArrangementArrangedEndpointDegree2 {
    /// One arranged endpoint occupies the structural point.
    Dangling,
    /// Two arranged endpoints occupy the structural point and form a chain connection.
    Chain,
    /// More than two arranged endpoints occupy the structural point.
    Branch,
}

/// Reference to a structural arranged endpoint bucket classified by degree.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointDegreeRef2 {
    structural_bucket_index: usize,
    endpoint_ref_count: usize,
    point: Point2,
}

/// Structural arranged endpoint buckets grouped by retained degree.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointDegreeBucket2 {
    degree: ExactCurveArrangementArrangedEndpointDegree2,
    endpoint_buckets: Vec<ExactCurveArrangementArrangedEndpointDegreeRef2>,
}

/// Arranged endpoint structural degree buckets retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointDegreeBucketCache2 {
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
pub(crate) struct ExactCurveArrangementArrangedEndpointRef2 {
    arranged_segment_index: usize,
    endpoint: RegionLineSegmentArrangedEndpoint2,
}

/// Arranged endpoint bucket grouped by retained endpoint side.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointSideBucket2 {
    endpoint: RegionLineSegmentArrangedEndpoint2,
    endpoints: Vec<ExactCurveArrangementArrangedEndpointRef2>,
}

/// Arranged endpoint side buckets retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointSideBucketCache2 {
    bucket_count: usize,
    endpoint_ref_count: usize,
    start_endpoint_ref_count: usize,
    end_endpoint_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedEndpointSideBucket2>,
}

/// Exact structural arranged endpoint bucket retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointBucket2 {
    point: Point2,
    endpoints: Vec<ExactCurveArrangementArrangedEndpointRef2>,
}

/// Exact structural arranged endpoint buckets retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointBucketCache2 {
    endpoint_count: usize,
    bucket_count: usize,
    singleton_bucket_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedEndpointBucket2>,
}

/// Exact endpoints retained for one arranged fragment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointPointRef2 {
    arranged_segment_index: usize,
    output_start_point: Point2,
    output_end_point: Point2,
}

/// Exact arranged endpoint records retained by endpoint-graph validation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedEndpointPointCache2 {
    arranged_fragment_ref_count: usize,
    endpoint_ref_count: usize,
    endpoints: Vec<ExactCurveArrangementArrangedEndpointPointRef2>,
}

/// Source provenance retained for one arranged fragment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentSourceRef2 {
    arranged_source_evidence_index: usize,
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_range: ParamRange,
    status: RetainedTopologyStatus,
}

/// Arranged fragment provenance retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragment2 {
    arranged_segment_index: usize,
    arranged_segment_kind: SegmentKind,
    output_start_point: Point2,
    output_end_point: Point2,
    source_refs: Vec<ExactCurveArrangementArrangedFragmentSourceRef2>,
}

/// Reference to a retained arranged fragment fact.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentRef2 {
    arranged_fragment_index: usize,
}

/// Reference to retained arranged fragment source evidence inside a status bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentStatusRef2 {
    arranged_fragment_index: usize,
    source_ref_index: usize,
    arranged_source_evidence_index: usize,
}

/// Arranged fragment bucket grouped by retained primitive family.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentKindBucket2 {
    arranged_segment_kind: SegmentKind,
    fragment_refs: Vec<ExactCurveArrangementArrangedFragmentRef2>,
}

/// Arranged fragment source-provenance bucket grouped by retained topology status.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentStatusBucket2 {
    status: RetainedTopologyStatus,
    source_refs: Vec<ExactCurveArrangementArrangedFragmentStatusRef2>,
}

/// Arranged fragment primitive-family buckets retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentKindBucketCache2 {
    bucket_count: usize,
    arranged_fragment_ref_count: usize,
    line_fragment_ref_count: usize,
    arc_fragment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementArrangedFragmentKindBucket2>,
}

/// Arranged fragment topology-status buckets retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentStatusBucketCache2 {
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
pub(crate) struct ExactCurveArrangementArrangedFragmentSourceRangeRef2 {
    arranged_source_evidence_index: usize,
    source_segment_index: usize,
    source_range: ParamRange,
    arranged_segment_index: usize,
}

/// Arranged fragment source-parameter ranges retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentSourceRangeCache2 {
    source_ref_count: usize,
    full_source_range_ref_count: usize,
    partial_source_range_ref_count: usize,
    ranges: Vec<ExactCurveArrangementArrangedFragmentSourceRangeRef2>,
}

/// Arranged fragment provenance cache retained after exact splitting.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementArrangedFragmentCache2 {
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
pub(crate) struct ExactCurveArrangementOutputRingSegmentRef2 {
    source_evidence_index: usize,
    output_segment_index: usize,
    reversed: bool,
}

/// Output ring bucket retained by exact ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRingBucket2 {
    output_ring_index: usize,
    segments: Vec<ExactCurveArrangementOutputRingSegmentRef2>,
}

/// Output ring buckets retained by exact ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRingBucketCache2 {
    ring_count: usize,
    segment_ref_count: usize,
    max_ring_segment_count: usize,
    rings: Vec<ExactCurveArrangementOutputRingBucket2>,
}

/// Output segment reference retained in a primitive-family bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentKindRef2 {
    source_evidence_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by retained primitive family.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentKindBucket2 {
    output_segment_kind: SegmentKind,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentKindRef2>,
}

/// Output segment primitive-family buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentKindBucketCache2 {
    bucket_count: usize,
    output_segment_ref_count: usize,
    line_segment_ref_count: usize,
    arc_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentKindBucket2>,
}

/// Output segment reference retained in a source-segment bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentSourceRef2 {
    source_evidence_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by retained source segment index.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentSourceBucket2 {
    source_segment_index: usize,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentSourceRef2>,
}

/// Output segment source-segment buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentSourceBucketCache2 {
    source_segment_bucket_count: usize,
    output_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentSourceBucket2>,
}

/// Output segment source-parameter range retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentSourceRangeRef2 {
    source_evidence_index: usize,
    source_segment_index: usize,
    source_range: ParamRange,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment source-parameter ranges retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentSourceRangeCache2 {
    output_segment_ref_count: usize,
    full_source_range_ref_count: usize,
    partial_source_range_ref_count: usize,
    ranges: Vec<ExactCurveArrangementOutputSegmentSourceRangeRef2>,
}

/// Output segment exact endpoints retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentEndpointRef2 {
    source_evidence_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
    output_start_point: Point2,
    output_end_point: Point2,
}

/// Output segment endpoint cache retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentEndpointCache2 {
    output_segment_ref_count: usize,
    output_endpoint_ref_count: usize,
    segments: Vec<ExactCurveArrangementOutputSegmentEndpointRef2>,
}

/// Exact endpoint continuity retained between adjacent output ring segments.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRingContinuityRef2 {
    source_evidence_index: usize,
    next_source_evidence_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
    next_output_segment_index: usize,
    output_end_point: Point2,
    next_output_start_point: Point2,
}

/// Output ring continuity cache retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRingContinuityCache2 {
    output_ring_ref_count: usize,
    output_connection_ref_count: usize,
    max_ring_connection_count: usize,
    connections: Vec<ExactCurveArrangementOutputRingContinuityRef2>,
}

/// Output segment reference retained in a topology-status bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentStatusRef2 {
    source_evidence_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by retained topology status.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentStatusBucket2 {
    status: RetainedTopologyStatus,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentStatusRef2>,
}

/// Output segment topology-status buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentStatusBucketCache2 {
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
pub(crate) struct ExactCurveArrangementOutputSegmentDirectionRef2 {
    source_evidence_index: usize,
    output_ring_index: usize,
    output_segment_index: usize,
}

/// Output segment bucket grouped by whether ring traversal reversed the source segment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentDirectionBucket2 {
    reversed: bool,
    segment_refs: Vec<ExactCurveArrangementOutputSegmentDirectionRef2>,
}

/// Output segment traversal-direction buckets retained after ring assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputSegmentDirectionBucketCache2 {
    bucket_count: usize,
    output_segment_ref_count: usize,
    forward_segment_ref_count: usize,
    reversed_segment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputSegmentDirectionBucket2>,
}

/// Output role assignment evidence retained for one boundary contour.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleAssignment2 {
    role_evidence_index: usize,
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
pub(crate) struct ExactCurveArrangementOutputRoleStatusRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_evidence_index: usize,
}

/// Output role bucket retained after boundary contour role assignment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleBucket2 {
    role: RegionBoundaryContourRole2,
    assignments: Vec<ExactCurveArrangementOutputRoleAssignment2>,
}

/// Output role assignment bucket grouped by retained topology status.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleStatusBucket2 {
    status: RetainedTopologyStatus,
    assignments: Vec<ExactCurveArrangementOutputRoleStatusRef2>,
}

/// Output role assignment topology-status buckets retained after role assignment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleStatusBucketCache2 {
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
pub(crate) struct ExactCurveArrangementOutputRoleSourceContourRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_evidence_index: usize,
    output_role_index: usize,
}

/// Output role assignment bucket grouped by retained source contour identity.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleSourceContourBucket2 {
    source_contour_index: usize,
    assignments: Vec<ExactCurveArrangementOutputRoleSourceContourRef2>,
}

/// Output role assignments grouped by retained source contour identity.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleSourceContourBucketCache2 {
    source_contour_bucket_count: usize,
    assignment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleSourceContourBucket2>,
}

/// Reference to a retained output role assignment inside a nesting-depth bucket.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleNestingDepthRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_evidence_index: usize,
    source_contour_index: usize,
    output_role_index: usize,
}

/// Output role assignment bucket grouped by exact nesting depth.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleNestingDepthBucket2 {
    nesting_depth: usize,
    assignments: Vec<ExactCurveArrangementOutputRoleNestingDepthRef2>,
}

/// Output role assignments grouped by retained exact nesting depth.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleNestingDepthBucketCache2 {
    nesting_depth_bucket_count: usize,
    assignment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleNestingDepthBucket2>,
}

/// Reference to retained containment evidence for one output role assignment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleContainmentRef2 {
    role: RegionBoundaryContourRole2,
    assignment_index: usize,
    role_evidence_index: usize,
    source_contour_index: usize,
    containing_contour_index: usize,
    containing_contour_ref_index: usize,
    output_role_index: usize,
}

/// Output role containment bucket grouped by exact containing source contour.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleContainmentBucket2 {
    containing_contour_index: usize,
    containments: Vec<ExactCurveArrangementOutputRoleContainmentRef2>,
}

/// Output role containment evidence grouped by exact containing source contour.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleContainmentBucketCache2 {
    containing_contour_bucket_count: usize,
    containment_ref_count: usize,
    uncontained_assignment_ref_count: usize,
    max_bucket_size: usize,
    buckets: Vec<ExactCurveArrangementOutputRoleContainmentBucket2>,
}

/// Output material/hole role buckets retained after boundary contour role assignment.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputRoleCache2 {
    role_evidence_count: usize,
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
pub(crate) struct ExactCurveArrangementOutputBoundaryRoleBucket2 {
    role: RegionBoundaryContourRole2,
    output_contour_count: usize,
    output_segment_count: usize,
}

/// Final boundary output counts grouped by material/hole role.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputBoundaryRoleBucketCache2 {
    bucket_count: usize,
    output_contour_count: usize,
    output_segment_count: usize,
    max_segment_count: usize,
    buckets: Vec<ExactCurveArrangementOutputBoundaryRoleBucket2>,
}

/// Final boundary output summary retained by an evaluated arrangement workspace.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactCurveArrangementOutputBoundaryCache2 {
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
pub(crate) struct ExactCurveArrangementRingAssemblyCache2 {
    predicate_path: RegionLineSegmentRingAssemblyPredicatePath2,
    attempted_endpoint_connection_count: usize,
    exact_endpoint_connection_count: usize,
    disconnected_endpoint_connection_count: usize,
    unresolved_endpoint_connection_count: usize,
    reversed_source_segment_count: usize,
    output_ring_count: Option<usize>,
    output_boundary_segment_count: Option<usize>,
    output_boundary_segment_kind_counts: Option<SegmentKindCounts>,
    arranged_source_evidence: Vec<RegionLineSegmentArrangedSourceEvidence2>,
    source_evidence: Vec<RegionLineSegmentRingSourceEvidence2>,
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
pub(crate) struct ExactCurveArrangementOutputCache2 {
    materialized_region: bool,
    boundary_build_evidence: Option<RegionBoundaryContourBuildEvidence2>,
    boundary_output_cache: Option<ExactCurveArrangementOutputBoundaryCache2>,
    role_cache: Option<ExactCurveArrangementOutputRoleCache2>,
    stage: RegionLineSegmentRegionBuildStage2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Final semantic facts from an immediate arrangement evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionArrangementSummary2 {
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

/// Immediate result of arranging unordered exact boundaries into a [`LineArcRegion2`].
///
/// This is the domain-level arrangement carrier returned by the
/// [`LineArcRegion2::arrange_unordered_segments`] family. It retains the certified
/// facts, blocker evidence, derived evidence, and optional materialized region.
/// Callers inspect this completed result directly; no second report lifecycle is
/// required.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionArrangement2 {
    workspace: Rc<ExactCurveWorkspace2>,
    summary: RegionArrangementSummary2,
    region: Option<LineArcRegion2>,
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
pub struct RegionBoundaryContourRoleEvidence2 {
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

/// Evidence for building a region from already-closed boundary contours.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourBuildEvidence2 {
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
    role_evidence: Vec<RegionBoundaryContourRoleEvidence2>,
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

/// Result of evidence-bearing boundary contour region construction.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundaryContourBuildResult2 {
    region: Option<LineArcRegion2>,
    evidence: RegionBoundaryContourBuildEvidence2,
}

/// Source line-segment provenance for one assembled boundary ring segment.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRingSourceEvidence2 {
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
pub struct RegionLineSegmentArrangedSourceEvidence2 {
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
pub struct RegionLineSegmentSplitIntersectionEvidence2 {
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

/// Internal staging evidence for unordered exact segment region construction.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRegionBuildEvidence2 {
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
    split_intersection_evidence: Vec<RegionLineSegmentSplitIntersectionEvidence2>,
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
    arranged_source_evidence: Vec<RegionLineSegmentArrangedSourceEvidence2>,
    source_evidence: Vec<RegionLineSegmentRingSourceEvidence2>,
    boundary_build_evidence: Option<RegionBoundaryContourBuildEvidence2>,
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

/// Internal staging result for unordered exact segment region construction.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionLineSegmentRegionBuildResult2 {
    region: Option<LineArcRegion2>,
    evidence: RegionLineSegmentRegionBuildEvidence2,
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

fn evaluate_unordered_line_segments_region_result(
    segments: &[LineSeg2],
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let arranged = match arrange_line_segments_at_point_intersections(segments, policy)? {
        Ok(arranged) => arranged,
        Err((split_evidence, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    segments.len(),
                    line_segment_kind_counts(segments.len()),
                    Some(split_evidence),
                    None,
                    Vec::new(),
                    LineSegmentRingAssemblyEvidenceParts::default(),
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
                    evidence: blocked_line_segment_region_evidence(
                        segments.len(),
                        line_segment_kind_counts(segments.len()),
                        Some(arranged.evidence),
                        Some(endpoint_graph),
                        line_arranged_source_evidence(&arranged.segments),
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            ..LineSegmentRingAssemblyEvidenceParts::default()
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
        Err((evidence, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    segments.len(),
                    line_segment_kind_counts(segments.len()),
                    Some(arranged.evidence),
                    Some(endpoint_graph),
                    line_arranged_source_evidence(&arranged.segments),
                    evidence,
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

    let built = LineArcRegion2::from_boundary_contours_with_evidence(contours, policy)?;
    let status = built.status();
    let blocker = built.blocker();
    let output_ring_count = built.output_contour_count();
    let output_boundary_segment_count = built.output_segment_count();
    let output_boundary_segment_kind_counts = built.region().map(region_segment_kind_counts);
    let (region, boundary_build_evidence) = built.into_parts();
    Ok(RegionLineSegmentRegionBuildResult2 {
        region,
        evidence: RegionLineSegmentRegionBuildEvidence2 {
            stage: RegionLineSegmentRegionBuildStage2::RegionRoleAssignment,
            source_segment_count: segments.len(),
            source_segment_kind_counts: line_segment_kind_counts(segments.len()),
            arranged_segment_count: Some(arranged.segments.len()),
            arranged_segment_kind_counts: Some(line_segment_kind_counts(arranged.segments.len())),
            split_predicate_path: arranged.evidence.predicate_path,
            endpoint_graph_predicate_path: Some(
                RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets,
            ),
            ring_assembly_predicate_path: Some(
                RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal,
            ),
            split_candidate_pair_count: arranged.evidence.candidate_pair_count,
            split_skipped_aabb_pair_count: arranged.evidence.skipped_aabb_pair_count,
            split_tested_pair_count: arranged.evidence.tested_pair_count,
            split_intersection_event_count: arranged.evidence.intersection_event_count,
            split_point_relation_count: arranged.evidence.point_relation_count,
            split_overlap_relation_count: arranged.evidence.overlap_relation_count,
            split_uncertain_relation_count: arranged.evidence.uncertain_relation_count,
            split_intersection_points: arranged.evidence.intersection_points,
            split_intersection_evidence: arranged.evidence.intersection_evidence,
            split_output_segment_count: Some(arranged.segments.len()),
            split_blocker_first_source_segment_index: arranged
                .evidence
                .blocker_first_source_segment_index,
            split_blocker_first_source_segment_kind: arranged
                .evidence
                .blocker_first_source_segment_kind,
            split_blocker_first_source_start_point: arranged
                .evidence
                .blocker_first_source_start_point,
            split_blocker_first_source_end_point: arranged.evidence.blocker_first_source_end_point,
            split_blocker_second_source_segment_index: arranged
                .evidence
                .blocker_second_source_segment_index,
            split_blocker_second_source_segment_kind: arranged
                .evidence
                .blocker_second_source_segment_kind,
            split_blocker_second_source_start_point: arranged
                .evidence
                .blocker_second_source_start_point,
            split_blocker_second_source_end_point: arranged
                .evidence
                .blocker_second_source_end_point,
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
            arranged_source_evidence: line_arranged_source_evidence(&arranged.segments),
            source_evidence: assembled.source_evidence,
            boundary_build_evidence: Some(boundary_build_evidence),
            status,
            blocker,
        },
    })
}

fn evaluate_unordered_segments_region_result(
    segments: &[Segment2],
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<RegionLineSegmentRegionBuildResult2> {
    if segments.is_empty() {
        return Err(CurveError::EmptyCurveString);
    }

    let arranged = match arrange_native_segments_at_point_intersections(segments, policy)? {
        Ok(arranged) => arranged,
        Err((split_evidence, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    segments.len(),
                    segment_kind_counts(segments),
                    Some(split_evidence),
                    None,
                    Vec::new(),
                    LineSegmentRingAssemblyEvidenceParts::default(),
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
                    evidence: blocked_line_segment_region_evidence(
                        segments.len(),
                        segment_kind_counts(segments),
                        Some(arranged.evidence),
                        Some(endpoint_graph),
                        native_arranged_source_evidence(segments, &arranged.segments),
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            ..LineSegmentRingAssemblyEvidenceParts::default()
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
        Err((evidence, blocker)) => {
            return Ok(RegionLineSegmentRegionBuildResult2 {
                region: None,
                evidence: blocked_line_segment_region_evidence(
                    segments.len(),
                    segment_kind_counts(segments),
                    Some(arranged.evidence),
                    Some(endpoint_graph),
                    native_arranged_source_evidence(segments, &arranged.segments),
                    evidence,
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

    let built = LineArcRegion2::from_boundary_contours_with_evidence(contours, policy)?;
    let status = built.status();
    let blocker = built.blocker();
    let output_ring_count = built.output_contour_count();
    let output_boundary_segment_count = built.output_segment_count();
    let output_boundary_segment_kind_counts = built.region().map(region_segment_kind_counts);
    let (region, boundary_build_evidence) = built.into_parts();
    Ok(RegionLineSegmentRegionBuildResult2 {
        region,
        evidence: RegionLineSegmentRegionBuildEvidence2 {
            stage: RegionLineSegmentRegionBuildStage2::RegionRoleAssignment,
            source_segment_count: segments.len(),
            source_segment_kind_counts: segment_kind_counts(segments),
            arranged_segment_count: Some(arranged.segments.len()),
            arranged_segment_kind_counts: Some(native_arranged_segment_kind_counts(
                &arranged.segments,
            )),
            split_predicate_path: arranged.evidence.predicate_path,
            endpoint_graph_predicate_path: Some(
                RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets,
            ),
            ring_assembly_predicate_path: Some(
                RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal,
            ),
            split_candidate_pair_count: arranged.evidence.candidate_pair_count,
            split_skipped_aabb_pair_count: arranged.evidence.skipped_aabb_pair_count,
            split_tested_pair_count: arranged.evidence.tested_pair_count,
            split_intersection_event_count: arranged.evidence.intersection_event_count,
            split_point_relation_count: arranged.evidence.point_relation_count,
            split_overlap_relation_count: arranged.evidence.overlap_relation_count,
            split_uncertain_relation_count: arranged.evidence.uncertain_relation_count,
            split_intersection_points: arranged.evidence.intersection_points,
            split_intersection_evidence: arranged.evidence.intersection_evidence,
            split_output_segment_count: Some(arranged.segments.len()),
            split_blocker_first_source_segment_index: arranged
                .evidence
                .blocker_first_source_segment_index,
            split_blocker_first_source_segment_kind: arranged
                .evidence
                .blocker_first_source_segment_kind,
            split_blocker_first_source_start_point: arranged
                .evidence
                .blocker_first_source_start_point,
            split_blocker_first_source_end_point: arranged.evidence.blocker_first_source_end_point,
            split_blocker_second_source_segment_index: arranged
                .evidence
                .blocker_second_source_segment_index,
            split_blocker_second_source_segment_kind: arranged
                .evidence
                .blocker_second_source_segment_kind,
            split_blocker_second_source_start_point: arranged
                .evidence
                .blocker_second_source_start_point,
            split_blocker_second_source_end_point: arranged
                .evidence
                .blocker_second_source_end_point,
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
            arranged_source_evidence: native_arranged_source_evidence(segments, &arranged.segments),
            source_evidence: assembled.source_evidence,
            boundary_build_evidence: Some(boundary_build_evidence),
            status,
            blocker,
        },
    })
}

impl LineArcRegion2 {
    /// Arranges unordered exact line/arc segments into a retained region result.
    pub fn arrange_unordered_segments(
        source_segments: Vec<Segment2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionArrangement2> {
        evaluate_exact_curve_arrangement(
            ExactCurveArrangementRequest2::from_unordered_segments(source_segments, fill_rule),
            policy,
        )
    }

    /// Arranges borrowed unordered exact line/arc segments into a retained region result.
    pub fn arrange_unordered_segments_borrowed(
        source_segments: &[Segment2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionArrangement2> {
        evaluate_exact_curve_arrangement(
            ExactCurveArrangementRequest2::from_borrowed_unordered_segments(
                source_segments,
                fill_rule,
            ),
            policy,
        )
    }

    /// Arranges unordered exact line segments using the specialized line pipeline.
    pub fn arrange_unordered_line_segments(
        source_segments: Vec<LineSeg2>,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionArrangement2> {
        evaluate_exact_curve_arrangement(
            ExactCurveArrangementRequest2::from_unordered_line_segments(source_segments, fill_rule),
            policy,
        )
    }

    /// Arranges borrowed unordered exact lines using the specialized line pipeline.
    pub fn arrange_unordered_line_segments_borrowed(
        source_segments: &[LineSeg2],
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionArrangement2> {
        evaluate_exact_curve_arrangement(
            ExactCurveArrangementRequest2::from_borrowed_unordered_line_segments(
                source_segments,
                fill_rule,
            ),
            policy,
        )
    }

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
        Ok(match contour_nesting_depths(&contours, policy)? {
            BoundaryContourNestingOutcome::Decided { nesting, .. } => {
                Classification::Decided(assign_boundary_contour_roles(contours, &nesting, false).0)
            }
            BoundaryContourNestingOutcome::Blocked { blocker, .. } => {
                Classification::Uncertain(blocker.reason)
            }
        })
    }

    pub(crate) fn from_validated_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if contours.len() <= 1 {
            return Ok(Classification::Decided(Self::from_material_contours(
                contours,
            )));
        }
        Ok(
            match contour_nesting_depths_impl(&contours, policy, false)? {
                BoundaryContourNestingOutcome::Decided { nesting, .. } => Classification::Decided(
                    assign_boundary_contour_roles(contours, &nesting, false).0,
                ),
                BoundaryContourNestingOutcome::Blocked { blocker, .. } => {
                    Classification::Uncertain(blocker.reason)
                }
            },
        )
    }

    pub(crate) fn from_directed_boolean_boundary_contours(
        contours: Vec<Contour2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if contours.len() <= 1 {
            return Ok(Classification::Decided(Self::from_material_contours(
                contours,
            )));
        }
        let Some(material_roles) = contours
            .iter()
            .map(|contour| {
                line_contour_directed_orientation(contour, policy)
                    .map(|orientation| orientation == RealSign::Positive)
            })
            .collect::<Option<Vec<_>>>()
        else {
            return Self::from_validated_boundary_contours(contours, policy);
        };
        let mut material_contours = Vec::new();
        let mut hole_contours = Vec::new();
        for (contour, material) in contours.into_iter().zip(material_roles) {
            if material {
                material_contours.push(contour);
            } else {
                hole_contours.push(contour);
            }
        }
        Ok(Classification::Decided(Self::new(
            material_contours,
            hole_contours,
        )))
    }

    /// Builds a region by nesting borrowed closed boundary contours.
    ///
    /// This clones the exact contour carriers at the API boundary, then uses
    /// the same exact nesting and role-assignment pipeline as
    /// [`LineArcRegion2::from_boundary_contours`].
    pub fn from_boundary_contours_borrowed(
        contours: &[Contour2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        Self::from_boundary_contours(contours.to_vec(), policy)
    }

    /// Builds a region by nesting closed boundary contours and retaining role evidence.
    ///
    /// This is the evidence-bearing counterpart to
    /// [`LineArcRegion2::from_boundary_contours`]. Contours at even containment depth
    /// become material and odd-depth contours become holes. If intersections,
    /// touches, or undecided containment predicates prevent role assignment, no
    /// region is materialized and the evidence carries the blocker.
    pub(crate) fn from_boundary_contours_with_evidence(
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
        let (region, role_evidence) = assign_boundary_contour_roles(contours, &nesting, true);
        let material_contour_count = region.material_contours().len();
        let hole_contour_count = region.hole_contours().len();
        let output_contour_count = material_contour_count + hole_contour_count;
        let material_segment_count = role_evidence
            .iter()
            .filter(|evidence| evidence.role == RegionBoundaryContourRole2::Material)
            .map(|evidence| evidence.source_segment_count)
            .sum();
        let hole_segment_count = role_evidence
            .iter()
            .filter(|evidence| evidence.role == RegionBoundaryContourRole2::Hole)
            .map(|evidence| evidence.source_segment_count)
            .sum();
        let output_segment_count = material_segment_count + hole_segment_count;
        Ok(RegionBoundaryContourBuildResult2 {
            region: Some(region),
            evidence: RegionBoundaryContourBuildEvidence2 {
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
                role_evidence,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }
}

fn assign_boundary_contour_roles(
    contours: Vec<Contour2>,
    nesting: &BoundaryContourNestingDepths,
    retain_evidence: bool,
) -> (LineArcRegion2, Vec<RegionBoundaryContourRoleEvidence2>) {
    let mut material_contours = Vec::new();
    let mut hole_contours = Vec::new();
    let mut role_evidence = Vec::with_capacity(usize::from(retain_evidence) * contours.len());
    for (source_contour_index, (contour, entry)) in
        contours.into_iter().zip(&nesting.entries).enumerate()
    {
        let depth = entry.containing_contour_indices.len();
        let source_segment_count = contour.segments().len();
        let source_fill_rule = contour.fill_rule();
        let (role, output_role_index) = if depth % 2 == 0 {
            let index = material_contours.len();
            material_contours.push(contour);
            (RegionBoundaryContourRole2::Material, index)
        } else {
            let index = hole_contours.len();
            hole_contours.push(contour);
            (RegionBoundaryContourRole2::Hole, index)
        };
        if retain_evidence {
            role_evidence.push(RegionBoundaryContourRoleEvidence2 {
                source_contour_index,
                source_segment_count,
                source_fill_rule,
                nesting_sample_point: entry.sample_point.clone(),
                containing_contour_indices: entry.containing_contour_indices.clone(),
                nesting_depth: depth,
                role,
                output_role_index,
                status: RetainedTopologyStatus::NativeExact,
            });
        }
    }
    (
        LineArcRegion2::new(material_contours, hole_contours),
        role_evidence,
    )
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
}

impl ExactCurveArrangementSourceAabbRef2 {}

impl ExactCurveArrangementSourceAabbBucket2 {}

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

    /// Returns the number of source segment boxes certified during workspace construction.
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
}

impl ExactCurveArrangementSourceSegmentKindRef2 {}

impl ExactCurveArrangementSourceSegmentKindBucket2 {}

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
}

impl ExactCurveArrangementSourceSegmentCache2 {
    fn from_sources(source_segments: &[Segment2], source_segment_aabbs: &[Option<Aabb2>]) -> Self {
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
            decided_source_segment_aabb_count,
            undecided_source_segment_aabb_count: source_segments
                .len()
                .saturating_sub(decided_source_segment_aabb_count),
            source_aabb_bucket_cache,
            source_segment_kind_bucket_cache,
            segments,
        }
    }

    /// Returns the number of source segment boxes certified during workspace construction.
    pub const fn decided_source_segment_aabb_count(&self) -> usize {
        self.decided_source_segment_aabb_count
    }

    /// Returns the number of source segment boxes that stayed uncertain.
    pub const fn undecided_source_segment_aabb_count(&self) -> usize {
        self.undecided_source_segment_aabb_count
    }

    /// Returns retained source AABB buckets grouped by certification status.
    pub(crate) const fn source_aabb_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceAabbBucketCache2 {
        &self.source_aabb_bucket_cache
    }

    /// Returns retained source segment buckets grouped by primitive family.
    pub(crate) const fn source_segment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceSegmentKindBucketCache2 {
        &self.source_segment_kind_bucket_cache
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

    fn with_region_build_evidence(
        mut self,
        evidence: &RegionLineSegmentRegionBuildEvidence2,
        materialized_region: bool,
    ) -> Self {
        self.split_cache =
            Some(ExactCurveArrangementSplitCache2::from_region_build_evidence(evidence));
        self.endpoint_graph_cache =
            ExactCurveArrangementEndpointGraphCache2::from_region_build_evidence(evidence);
        self.ring_assembly_cache =
            ExactCurveArrangementRingAssemblyCache2::from_region_build_evidence(evidence);
        self.output_cache = Some(
            ExactCurveArrangementOutputCache2::from_region_build_evidence(
                evidence,
                materialized_region,
            ),
        );
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

    /// Returns the number of source segment boxes certified during workspace construction.
    pub fn decided_source_segment_aabb_count(&self) -> usize {
        self.source_segment_cache
            .decided_source_segment_aabb_count()
    }

    /// Returns the number of source segment boxes that stayed uncertain.
    pub fn undecided_source_segment_aabb_count(&self) -> usize {
        self.source_segment_cache
            .undecided_source_segment_aabb_count()
    }

    /// Returns source segment facts retained before split scheduling.
    pub(crate) const fn source_segment_cache(&self) -> &ExactCurveArrangementSourceSegmentCache2 {
        &self.source_segment_cache
    }

    /// Returns retained source AABB buckets grouped by certification status.
    pub(crate) const fn source_aabb_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceAabbBucketCache2 {
        self.source_segment_cache().source_aabb_bucket_cache()
    }

    /// Returns retained source AABB-status bucket count.
    pub const fn source_aabb_bucket_count(&self) -> usize {
        self.source_aabb_bucket_cache().bucket_count()
    }

    /// Returns retained source AABB references.
    pub const fn source_aabb_ref_count(&self) -> usize {
        self.source_aabb_bucket_cache().source_ref_count()
    }

    /// Returns retained source AABB references certified as decided.
    pub const fn source_aabb_decided_ref_count(&self) -> usize {
        self.source_aabb_bucket_cache().decided_source_ref_count()
    }

    /// Returns retained source AABB references that stayed undecided.
    pub const fn source_aabb_undecided_ref_count(&self) -> usize {
        self.source_aabb_bucket_cache().undecided_source_ref_count()
    }

    /// Returns the largest retained source AABB-status bucket size.
    pub const fn source_aabb_max_bucket_size(&self) -> usize {
        self.source_aabb_bucket_cache().max_bucket_size()
    }

    /// Returns retained source segment buckets grouped by primitive family.
    pub(crate) const fn source_segment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceSegmentKindBucketCache2 {
        self.source_segment_cache()
            .source_segment_kind_bucket_cache()
    }

    /// Returns retained source segment primitive-family bucket count.
    pub const fn source_segment_kind_bucket_count(&self) -> usize {
        self.source_segment_kind_bucket_cache().bucket_count()
    }

    /// Returns retained source segment references grouped by primitive family.
    pub const fn source_segment_kind_ref_count(&self) -> usize {
        self.source_segment_kind_bucket_cache()
            .source_segment_ref_count()
    }

    /// Returns retained line source segment references.
    pub const fn source_line_segment_ref_count(&self) -> usize {
        self.source_segment_kind_bucket_cache()
            .line_segment_ref_count()
    }

    /// Returns retained arc source segment references.
    pub const fn source_arc_segment_ref_count(&self) -> usize {
        self.source_segment_kind_bucket_cache()
            .arc_segment_ref_count()
    }

    /// Returns the largest retained source segment primitive-family bucket size.
    pub const fn source_segment_kind_max_bucket_size(&self) -> usize {
        self.source_segment_kind_bucket_cache().max_bucket_size()
    }

    /// Returns exact source endpoint buckets retained during workspace construction.
    pub(crate) const fn source_endpoint_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSourceEndpointBucketCache2 {
        &self.source_endpoint_bucket_cache
    }

    /// Returns source endpoints retained in exact structural endpoint buckets.
    pub const fn source_endpoint_count(&self) -> usize {
        self.source_endpoint_bucket_cache().endpoint_count()
    }

    /// Returns exact structural source endpoint bucket count.
    pub const fn source_endpoint_bucket_count(&self) -> usize {
        self.source_endpoint_bucket_cache().bucket_count()
    }

    /// Returns source endpoint buckets containing one endpoint.
    pub const fn source_endpoint_singleton_bucket_count(&self) -> usize {
        self.source_endpoint_bucket_cache().singleton_bucket_count()
    }

    /// Returns the largest exact structural source endpoint bucket size.
    pub const fn source_endpoint_max_bucket_size(&self) -> usize {
        self.source_endpoint_bucket_cache().max_bucket_size()
    }

    /// Returns the source-pair schedule retained before split predicates run.
    pub(crate) const fn split_schedule_cache(&self) -> &ExactCurveArrangementSplitScheduleCache2 {
        &self.split_schedule_cache
    }

    /// Returns retained split schedule buckets grouped by AABB pruning status.
    pub(crate) const fn split_schedule_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSplitScheduleBucketCache2 {
        self.split_schedule_cache().bucket_cache()
    }

    /// Returns source segment pairs scheduled before retained split predicates run.
    pub const fn split_schedule_candidate_pair_count(&self) -> usize {
        self.split_schedule_cache().candidate_pair_count()
    }

    /// Returns scheduled source segment pairs pruned by retained AABB evidence.
    pub const fn split_schedule_decided_disjoint_pair_count(&self) -> usize {
        self.split_schedule_cache().decided_disjoint_pair_count()
    }

    /// Returns scheduled source segment pairs that require split predicate evaluation.
    pub const fn split_schedule_predicate_candidate_pair_count(&self) -> usize {
        self.split_schedule_cache().predicate_candidate_pair_count()
    }

    /// Returns scheduled source segment pairs whose AABB pruning status stayed undecided.
    pub const fn split_schedule_undecided_aabb_pair_count(&self) -> usize {
        self.split_schedule_cache().undecided_aabb_pair_count()
    }

    /// Returns retained split schedule AABB-status bucket count.
    pub const fn split_schedule_bucket_count(&self) -> usize {
        self.split_schedule_bucket_cache().bucket_count()
    }

    /// Returns retained split schedule source-pair references grouped by AABB status.
    pub const fn split_schedule_candidate_ref_count(&self) -> usize {
        self.split_schedule_bucket_cache().candidate_ref_count()
    }

    /// Returns the largest retained split schedule AABB-status bucket size.
    pub const fn split_schedule_max_bucket_size(&self) -> usize {
        self.split_schedule_bucket_cache().max_bucket_size()
    }

    /// Returns exact split evidence retained from the evaluated arrangement.
    pub(crate) const fn split_cache(&self) -> Option<&ExactCurveArrangementSplitCache2> {
        self.split_cache.as_ref()
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

    /// Returns retained split-stage relation buckets.
    pub(crate) const fn split_relation_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitRelationBucketCache2> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.relation_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained split-stage relation bucket count.
    pub const fn split_relation_bucket_count(&self) -> Option<usize> {
        match self.split_relation_bucket_cache() {
            Some(relation_cache) => Some(relation_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained split-stage classified relation references.
    pub const fn split_relation_ref_count(&self) -> Option<usize> {
        match self.split_relation_bucket_cache() {
            Some(relation_cache) => Some(relation_cache.relation_count()),
            None => None,
        }
    }

    /// Returns the largest retained split-stage relation bucket size.
    pub const fn split_relation_max_bucket_size(&self) -> Option<usize> {
        match self.split_relation_bucket_cache() {
            Some(relation_cache) => Some(relation_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns exact split-intersection point buckets.
    pub(crate) const fn split_intersection_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitIntersectionBucketCache2> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.intersection_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained exact split-intersection point bucket count.
    pub const fn split_intersection_bucket_count(&self) -> Option<usize> {
        match self.split_intersection_bucket_cache() {
            Some(intersection_cache) => Some(intersection_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained split-intersection buckets containing one event.
    pub const fn split_intersection_singleton_bucket_count(&self) -> Option<usize> {
        match self.split_intersection_bucket_cache() {
            Some(intersection_cache) => Some(intersection_cache.singleton_bucket_count()),
            None => None,
        }
    }

    /// Returns the largest retained split-intersection point bucket size.
    pub const fn split_intersection_max_bucket_size(&self) -> Option<usize> {
        match self.split_intersection_bucket_cache() {
            Some(intersection_cache) => Some(intersection_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns exact intersection points retained by split evaluation.
    pub fn split_intersection_points(&self) -> Option<&[Point2]> {
        self.split_cache()
            .map(ExactCurveArrangementSplitCache2::intersection_points)
    }

    /// Returns exact per-event source and parameter evidence retained by split evaluation.
    pub fn split_intersection_evidence(
        &self,
    ) -> Option<&[RegionLineSegmentSplitIntersectionEvidence2]> {
        self.split_cache()
            .map(ExactCurveArrangementSplitCache2::intersection_evidence)
    }

    /// Returns exact source-parameter evidence for retained split intersections.
    pub(crate) const fn split_intersection_parameter_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitIntersectionParameterCache2> {
        match self.split_cache() {
            Some(split_cache) => Some(split_cache.intersection_parameter_cache()),
            None => None,
        }
    }

    /// Returns retained source-parameter references for split intersections.
    pub const fn split_intersection_source_parameter_ref_count(&self) -> Option<usize> {
        match self.split_intersection_parameter_cache() {
            Some(parameter_cache) => Some(parameter_cache.source_parameter_ref_count()),
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

    /// Returns split-stage blocker source-pair evidence, when split evaluation blocked.
    pub(crate) const fn split_blocker_cache(
        &self,
    ) -> Option<&ExactCurveArrangementSplitBlockerCache2> {
        match self.split_cache() {
            Some(split_cache) => split_cache.blocker_cache(),
            None => None,
        }
    }

    /// Returns the first source segment in a split-stage blocker, when known.
    pub const fn split_blocker_first_source_segment_index(&self) -> Option<usize> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.first_source_segment_index()),
            None => None,
        }
    }

    /// Returns the primitive family of the first source segment in a split-stage blocker.
    pub const fn split_blocker_first_source_segment_kind(&self) -> Option<SegmentKind> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.first_source_segment_kind()),
            None => None,
        }
    }

    /// Returns the exact start point of the first source segment in a split-stage blocker.
    pub const fn split_blocker_first_source_start_point(&self) -> Option<&Point2> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.first_source_start_point()),
            None => None,
        }
    }

    /// Returns the exact end point of the first source segment in a split-stage blocker.
    pub const fn split_blocker_first_source_end_point(&self) -> Option<&Point2> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.first_source_end_point()),
            None => None,
        }
    }

    /// Returns the second source segment in a split-stage blocker, when known.
    pub const fn split_blocker_second_source_segment_index(&self) -> Option<usize> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.second_source_segment_index()),
            None => None,
        }
    }

    /// Returns the primitive family of the second source segment in a split-stage blocker.
    pub const fn split_blocker_second_source_segment_kind(&self) -> Option<SegmentKind> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.second_source_segment_kind()),
            None => None,
        }
    }

    /// Returns the exact start point of the second source segment in a split-stage blocker.
    pub const fn split_blocker_second_source_start_point(&self) -> Option<&Point2> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.second_source_start_point()),
            None => None,
        }
    }

    /// Returns the exact end point of the second source segment in a split-stage blocker.
    pub const fn split_blocker_second_source_end_point(&self) -> Option<&Point2> {
        match self.split_blocker_cache() {
            Some(blocker_cache) => Some(blocker_cache.second_source_end_point()),
            None => None,
        }
    }

    /// Returns exact endpoint-bucket evidence retained from the evaluated arrangement.
    pub(crate) const fn endpoint_graph_cache(
        &self,
    ) -> Option<&ExactCurveArrangementEndpointGraphCache2> {
        self.endpoint_graph_cache.as_ref()
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

    /// Returns exact arranged endpoint buckets retained by endpoint-graph validation.
    pub(crate) const fn arranged_endpoint_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedEndpointBucketCache2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.endpoint_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained arranged endpoint references in structural buckets.
    pub const fn arranged_endpoint_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_bucket_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.endpoint_count()),
            None => None,
        }
    }

    /// Returns retained exact structural arranged endpoint bucket count.
    pub const fn arranged_endpoint_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_bucket_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained arranged endpoint buckets containing one endpoint.
    pub const fn arranged_endpoint_singleton_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_bucket_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.singleton_bucket_count()),
            None => None,
        }
    }

    /// Returns the largest retained arranged endpoint structural bucket size.
    pub const fn arranged_endpoint_max_bucket_size(&self) -> Option<usize> {
        match self.arranged_endpoint_bucket_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns arranged endpoints grouped by retained endpoint side.
    pub(crate) const fn arranged_endpoint_side_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedEndpointSideBucketCache2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.endpoint_side_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained arranged endpoint side bucket count.
    pub const fn arranged_endpoint_side_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_side_bucket_cache() {
            Some(side_cache) => Some(side_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained arranged endpoint references grouped by side.
    pub const fn arranged_endpoint_side_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_side_bucket_cache() {
            Some(side_cache) => Some(side_cache.endpoint_ref_count()),
            None => None,
        }
    }

    /// Returns retained arranged start endpoint references.
    pub const fn arranged_endpoint_start_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_side_bucket_cache() {
            Some(side_cache) => Some(side_cache.start_endpoint_ref_count()),
            None => None,
        }
    }

    /// Returns retained arranged end endpoint references.
    pub const fn arranged_endpoint_end_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_side_bucket_cache() {
            Some(side_cache) => Some(side_cache.end_endpoint_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained arranged endpoint side bucket size.
    pub const fn arranged_endpoint_side_max_bucket_size(&self) -> Option<usize> {
        match self.arranged_endpoint_side_bucket_cache() {
            Some(side_cache) => Some(side_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns exact endpoint records for arranged fragments.
    pub(crate) const fn arranged_endpoint_point_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedEndpointPointCache2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.endpoint_point_cache()),
            None => None,
        }
    }

    /// Returns retained arranged fragment endpoint records.
    pub const fn arranged_endpoint_point_fragment_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_point_cache() {
            Some(point_cache) => Some(point_cache.arranged_fragment_ref_count()),
            None => None,
        }
    }

    /// Returns retained arranged endpoint point references.
    pub const fn arranged_endpoint_point_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_point_cache() {
            Some(point_cache) => Some(point_cache.endpoint_ref_count()),
            None => None,
        }
    }

    /// Returns structural arranged endpoints grouped by retained degree.
    pub(crate) const fn arranged_endpoint_degree_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedEndpointDegreeBucketCache2> {
        match self.endpoint_graph_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.endpoint_degree_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained arranged endpoint degree bucket count.
    pub const fn arranged_endpoint_degree_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_degree_bucket_cache() {
            Some(degree_cache) => Some(degree_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns structural endpoint buckets classified by retained degree.
    pub const fn arranged_endpoint_degree_structural_bucket_ref_count(&self) -> Option<usize> {
        match self.arranged_endpoint_degree_bucket_cache() {
            Some(degree_cache) => Some(degree_cache.structural_bucket_ref_count()),
            None => None,
        }
    }

    /// Returns structural endpoint buckets classified as dangling.
    pub const fn arranged_endpoint_dangling_structural_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_degree_bucket_cache() {
            Some(degree_cache) => Some(degree_cache.dangling_structural_bucket_count()),
            None => None,
        }
    }

    /// Returns structural endpoint buckets classified as chain continuations.
    pub const fn arranged_endpoint_chain_structural_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_degree_bucket_cache() {
            Some(degree_cache) => Some(degree_cache.chain_structural_bucket_count()),
            None => None,
        }
    }

    /// Returns structural endpoint buckets classified as branches.
    pub const fn arranged_endpoint_branch_structural_bucket_count(&self) -> Option<usize> {
        match self.arranged_endpoint_degree_bucket_cache() {
            Some(degree_cache) => Some(degree_cache.branch_structural_bucket_count()),
            None => None,
        }
    }

    /// Returns the largest retained arranged endpoint degree bucket size.
    pub const fn arranged_endpoint_degree_max_bucket_size(&self) -> Option<usize> {
        match self.arranged_endpoint_degree_bucket_cache() {
            Some(degree_cache) => Some(degree_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns exact ring-traversal evidence retained from the evaluated arrangement.
    pub(crate) const fn ring_assembly_cache(
        &self,
    ) -> Option<&ExactCurveArrangementRingAssemblyCache2> {
        self.ring_assembly_cache.as_ref()
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

    /// Returns per-arranged-fragment source provenance retained after exact splitting.
    pub fn arranged_source_evidence(&self) -> Option<&[RegionLineSegmentArrangedSourceEvidence2]> {
        self.ring_assembly_cache()
            .map(ExactCurveArrangementRingAssemblyCache2::arranged_source_evidence)
    }

    /// Returns the retained arranged-source provenance record count.
    pub const fn arranged_source_evidence_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.arranged_source_evidence.len()),
            None => None,
        }
    }

    /// Returns per-output segment source provenance retained by ring traversal.
    pub fn source_evidence(&self) -> Option<&[RegionLineSegmentRingSourceEvidence2]> {
        self.ring_assembly_cache()
            .map(ExactCurveArrangementRingAssemblyCache2::source_evidence)
    }

    /// Returns the retained output-source provenance record count.
    pub const fn source_evidence_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.source_evidence.len()),
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

    /// Returns per-arranged-fragment source provenance buckets.
    pub(crate) const fn arranged_fragment_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedFragmentCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.arranged_fragment_cache()),
            None => None,
        }
    }

    /// Returns arranged fragment count retained after exact splitting, when available.
    pub const fn arranged_segment_count(&self) -> Option<usize> {
        match self.arranged_fragment_cache() {
            Some(arranged_cache) => Some(arranged_cache.arranged_fragment_count()),
            None => None,
        }
    }

    /// Returns arranged fragment primitive-family counts retained after exact splitting.
    pub const fn arranged_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        match self.arranged_fragment_cache() {
            Some(arranged_cache) => Some(arranged_cache.arranged_segment_kind_counts()),
            None => None,
        }
    }

    /// Returns arranged fragments grouped by primitive family.
    pub(crate) const fn arranged_fragment_kind_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedFragmentKindBucketCache2> {
        match self.arranged_fragment_cache() {
            Some(arranged_cache) => Some(arranged_cache.arranged_fragment_kind_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained arranged fragment primitive-family bucket count.
    pub const fn arranged_fragment_kind_bucket_count(&self) -> Option<usize> {
        match self.arranged_fragment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained arranged fragment references grouped by primitive family.
    pub const fn arranged_fragment_kind_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.arranged_fragment_ref_count()),
            None => None,
        }
    }

    /// Returns retained line arranged fragment references.
    pub const fn arranged_line_fragment_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.line_fragment_ref_count()),
            None => None,
        }
    }

    /// Returns retained arc arranged fragment references.
    pub const fn arranged_arc_fragment_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.arc_fragment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained arranged fragment primitive-family bucket size.
    pub const fn arranged_fragment_kind_max_bucket_size(&self) -> Option<usize> {
        match self.arranged_fragment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns arranged fragment source records grouped by retained topology status.
    pub(crate) const fn arranged_fragment_status_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedFragmentStatusBucketCache2> {
        match self.arranged_fragment_cache() {
            Some(arranged_cache) => Some(arranged_cache.arranged_fragment_status_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained arranged fragment topology-status bucket count.
    pub const fn arranged_fragment_status_bucket_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained arranged fragment source references grouped by topology status.
    pub const fn arranged_fragment_status_source_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.source_ref_count()),
            None => None,
        }
    }

    /// Returns retained native-exact arranged fragment source references.
    pub const fn arranged_fragment_native_exact_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.native_exact_ref_count()),
            None => None,
        }
    }

    /// Returns retained certified-approximation arranged fragment source references.
    pub const fn arranged_fragment_certified_approximation_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.certified_approximation_ref_count()),
            None => None,
        }
    }

    /// Returns retained display/export-only arranged fragment source references.
    pub const fn arranged_fragment_display_or_export_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.display_or_export_ref_count()),
            None => None,
        }
    }

    /// Returns retained lossy-import arranged fragment source references.
    pub const fn arranged_fragment_imported_lossy_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.imported_lossy_ref_count()),
            None => None,
        }
    }

    /// Returns retained unsupported arranged fragment source references.
    pub const fn arranged_fragment_unsupported_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.unsupported_ref_count()),
            None => None,
        }
    }

    /// Returns retained unresolved arranged fragment source references.
    pub const fn arranged_fragment_unresolved_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.unresolved_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained arranged fragment topology-status bucket size.
    pub const fn arranged_fragment_status_max_bucket_size(&self) -> Option<usize> {
        match self.arranged_fragment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns arranged fragment source parameter ranges.
    pub(crate) const fn arranged_fragment_source_range_cache(
        &self,
    ) -> Option<&ExactCurveArrangementArrangedFragmentSourceRangeCache2> {
        match self.arranged_fragment_cache() {
            Some(arranged_cache) => Some(arranged_cache.arranged_fragment_source_range_cache()),
            None => None,
        }
    }

    /// Returns retained arranged fragment source range references.
    pub const fn arranged_fragment_source_range_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_source_range_cache() {
            Some(range_cache) => Some(range_cache.source_ref_count()),
            None => None,
        }
    }

    /// Returns retained arranged fragment source ranges covering complete source segments.
    pub const fn arranged_fragment_full_source_range_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_source_range_cache() {
            Some(range_cache) => Some(range_cache.full_source_range_ref_count()),
            None => None,
        }
    }

    /// Returns retained arranged fragment source ranges covering proper source subranges.
    pub const fn arranged_fragment_partial_source_range_ref_count(&self) -> Option<usize> {
        match self.arranged_fragment_source_range_cache() {
            Some(range_cache) => Some(range_cache.partial_source_range_ref_count()),
            None => None,
        }
    }

    /// Returns per-output-ring source provenance buckets.
    pub(crate) const fn output_ring_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRingBucketCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_ring_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained output ring provenance bucket count.
    pub const fn output_ring_bucket_count(&self) -> Option<usize> {
        match self.output_ring_bucket_cache() {
            Some(ring_cache) => Some(ring_cache.ring_count()),
            None => None,
        }
    }

    /// Returns retained output ring segment references.
    pub const fn output_ring_segment_ref_count(&self) -> Option<usize> {
        match self.output_ring_bucket_cache() {
            Some(ring_cache) => Some(ring_cache.segment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained output ring segment count.
    pub const fn output_ring_max_segment_count(&self) -> Option<usize> {
        match self.output_ring_bucket_cache() {
            Some(ring_cache) => Some(ring_cache.max_ring_segment_count()),
            None => None,
        }
    }

    /// Returns output ring count when ring traversal completed.
    pub const fn output_ring_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => ring_cache.output_ring_count(),
            None => None,
        }
    }

    /// Returns output boundary segment count when ring traversal completed.
    pub const fn output_boundary_segment_count(&self) -> Option<usize> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => ring_cache.output_boundary_segment_count(),
            None => None,
        }
    }

    /// Returns output boundary primitive-family counts when ring traversal completed.
    pub const fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => ring_cache.output_boundary_segment_kind_counts(),
            None => None,
        }
    }

    /// Returns retained output segment buckets grouped by primitive family.
    pub(crate) const fn output_segment_kind_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputSegmentKindBucketCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_segment_kind_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained output segment primitive-family bucket count.
    pub const fn output_segment_kind_bucket_count(&self) -> Option<usize> {
        match self.output_segment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained output segment references grouped by primitive family.
    pub const fn output_segment_kind_ref_count(&self) -> Option<usize> {
        match self.output_segment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.output_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained line output segment references.
    pub const fn output_line_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.line_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained arc output segment references.
    pub const fn output_arc_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.arc_segment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained output segment primitive-family bucket size.
    pub const fn output_segment_kind_max_bucket_size(&self) -> Option<usize> {
        match self.output_segment_kind_bucket_cache() {
            Some(kind_cache) => Some(kind_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns retained output segment buckets grouped by source segment.
    pub(crate) const fn output_segment_source_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputSegmentSourceBucketCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_segment_source_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained source-segment bucket count for output segments.
    pub const fn output_segment_source_bucket_count(&self) -> Option<usize> {
        match self.output_segment_source_bucket_cache() {
            Some(source_cache) => Some(source_cache.source_segment_bucket_count()),
            None => None,
        }
    }

    /// Returns retained output segment references grouped by source segment.
    pub const fn output_segment_source_ref_count(&self) -> Option<usize> {
        match self.output_segment_source_bucket_cache() {
            Some(source_cache) => Some(source_cache.output_segment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained source-segment output bucket size.
    pub const fn output_segment_source_max_bucket_size(&self) -> Option<usize> {
        match self.output_segment_source_bucket_cache() {
            Some(source_cache) => Some(source_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns retained output segment source parameter ranges.
    pub(crate) const fn output_segment_source_range_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputSegmentSourceRangeCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_segment_source_range_cache()),
            None => None,
        }
    }

    /// Returns retained output segment source-range references.
    pub const fn output_segment_source_range_ref_count(&self) -> Option<usize> {
        match self.output_segment_source_range_cache() {
            Some(range_cache) => Some(range_cache.output_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained output segments covering a complete source range.
    pub const fn output_full_source_range_ref_count(&self) -> Option<usize> {
        match self.output_segment_source_range_cache() {
            Some(range_cache) => Some(range_cache.full_source_range_ref_count()),
            None => None,
        }
    }

    /// Returns retained output segments covering a proper source subrange.
    pub const fn output_partial_source_range_ref_count(&self) -> Option<usize> {
        match self.output_segment_source_range_cache() {
            Some(range_cache) => Some(range_cache.partial_source_range_ref_count()),
            None => None,
        }
    }

    /// Returns retained output segment exact endpoint records.
    pub(crate) const fn output_segment_endpoint_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputSegmentEndpointCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_segment_endpoint_cache()),
            None => None,
        }
    }

    /// Returns retained output segment endpoint records.
    pub const fn output_segment_endpoint_record_count(&self) -> Option<usize> {
        match self.output_segment_endpoint_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.output_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained exact output endpoint references.
    pub const fn output_endpoint_ref_count(&self) -> Option<usize> {
        match self.output_segment_endpoint_cache() {
            Some(endpoint_cache) => Some(endpoint_cache.output_endpoint_ref_count()),
            None => None,
        }
    }

    /// Returns retained exact continuity records between adjacent output segments.
    pub(crate) const fn output_ring_continuity_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRingContinuityCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_ring_continuity_cache()),
            None => None,
        }
    }

    /// Returns output rings with retained continuity evidence.
    pub const fn output_ring_continuity_ring_ref_count(&self) -> Option<usize> {
        match self.output_ring_continuity_cache() {
            Some(continuity_cache) => Some(continuity_cache.output_ring_ref_count()),
            None => None,
        }
    }

    /// Returns retained output segment-to-next-segment continuity references.
    pub const fn output_ring_continuity_connection_ref_count(&self) -> Option<usize> {
        match self.output_ring_continuity_cache() {
            Some(continuity_cache) => Some(continuity_cache.output_connection_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained continuity connection count for one output ring.
    pub const fn output_ring_continuity_max_connection_count(&self) -> Option<usize> {
        match self.output_ring_continuity_cache() {
            Some(continuity_cache) => Some(continuity_cache.max_ring_connection_count()),
            None => None,
        }
    }

    /// Returns retained output segment buckets grouped by topology status.
    pub(crate) const fn output_segment_status_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputSegmentStatusBucketCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_segment_status_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained topology-status bucket count for output segments.
    pub const fn output_segment_status_bucket_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained output segment references grouped by topology status.
    pub const fn output_segment_status_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.output_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained native-exact output segment references.
    pub const fn output_native_exact_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.native_exact_ref_count()),
            None => None,
        }
    }

    /// Returns retained certified-approximation output segment references.
    pub const fn output_certified_approximation_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.certified_approximation_ref_count()),
            None => None,
        }
    }

    /// Returns retained display/export-only output segment references.
    pub const fn output_display_or_export_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.display_or_export_ref_count()),
            None => None,
        }
    }

    /// Returns retained lossy-import output segment references.
    pub const fn output_imported_lossy_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.imported_lossy_ref_count()),
            None => None,
        }
    }

    /// Returns retained unsupported output segment references.
    pub const fn output_unsupported_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.unsupported_ref_count()),
            None => None,
        }
    }

    /// Returns retained unresolved output segment references.
    pub const fn output_unresolved_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.unresolved_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained topology-status output bucket size.
    pub const fn output_segment_status_max_bucket_size(&self) -> Option<usize> {
        match self.output_segment_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns retained output segment buckets grouped by traversal direction.
    pub(crate) const fn output_segment_direction_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputSegmentDirectionBucketCache2> {
        match self.ring_assembly_cache() {
            Some(ring_cache) => Some(ring_cache.output_segment_direction_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained traversal-direction bucket count for output segments.
    pub const fn output_segment_direction_bucket_count(&self) -> Option<usize> {
        match self.output_segment_direction_bucket_cache() {
            Some(direction_cache) => Some(direction_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained output segment references grouped by traversal direction.
    pub const fn output_segment_direction_ref_count(&self) -> Option<usize> {
        match self.output_segment_direction_bucket_cache() {
            Some(direction_cache) => Some(direction_cache.output_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained forward output segment references.
    pub const fn output_forward_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_direction_bucket_cache() {
            Some(direction_cache) => Some(direction_cache.forward_segment_ref_count()),
            None => None,
        }
    }

    /// Returns retained reversed output segment references.
    pub const fn output_reversed_segment_ref_count(&self) -> Option<usize> {
        match self.output_segment_direction_bucket_cache() {
            Some(direction_cache) => Some(direction_cache.reversed_segment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained traversal-direction output bucket size.
    pub const fn output_segment_direction_max_bucket_size(&self) -> Option<usize> {
        match self.output_segment_direction_bucket_cache() {
            Some(direction_cache) => Some(direction_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns final output evidence retained from the evaluated arrangement.
    pub(crate) const fn output_cache(&self) -> Option<&ExactCurveArrangementOutputCache2> {
        self.output_cache.as_ref()
    }

    /// Returns whether final output evaluation facts were retained.
    pub const fn evaluated_output(&self) -> bool {
        self.output_cache().is_some()
    }

    /// Returns whether retained output materialized a region, when output was evaluated.
    pub const fn materialized_region(&self) -> Option<bool> {
        match self.output_cache() {
            Some(output_cache) => Some(output_cache.materialized_region()),
            None => None,
        }
    }

    /// Returns the final retained build stage, when output was evaluated.
    pub const fn stage(&self) -> Option<RegionLineSegmentRegionBuildStage2> {
        match self.output_cache() {
            Some(output_cache) => Some(output_cache.stage()),
            None => None,
        }
    }

    /// Returns final retained topology status, when output was evaluated.
    pub const fn status(&self) -> Option<RetainedTopologyStatus> {
        match self.output_cache() {
            Some(output_cache) => Some(output_cache.status()),
            None => None,
        }
    }

    /// Returns the final retained blocker, when output was evaluated and blocked.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        match self.output_cache() {
            Some(output_cache) => output_cache.blocker(),
            None => None,
        }
    }

    /// Returns delegated boundary-contour role assignment evidence, when output reached it.
    pub const fn boundary_build_evidence(&self) -> Option<&RegionBoundaryContourBuildEvidence2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_evidence(),
            None => None,
        }
    }

    /// Returns final boundary-role assignment stage, if reached.
    pub const fn boundary_build_stage(&self) -> Option<RegionBoundaryContourBuildStage2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_stage(),
            None => None,
        }
    }

    /// Returns final boundary-role assignment predicate path, if reached.
    pub const fn boundary_build_predicate_path(
        &self,
    ) -> Option<RegionBoundaryContourBuildPredicatePath2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_predicate_path(),
            None => None,
        }
    }

    /// Returns final boundary-role assignment retained status, if reached.
    pub const fn boundary_build_status(&self) -> Option<RetainedTopologyStatus> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_status(),
            None => None,
        }
    }

    /// Returns final boundary-role assignment blocker, if present.
    pub const fn boundary_build_blocker(&self) -> Option<UncertaintyReason> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_blocker(),
            None => None,
        }
    }

    /// Returns source contour count from delegated boundary-role assignment, if reached.
    pub const fn boundary_build_source_contour_count(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_source_contour_count(),
            None => None,
        }
    }

    /// Returns source boundary segment count from delegated boundary-role assignment, if reached.
    pub const fn boundary_build_source_segment_count(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_source_segment_count(),
            None => None,
        }
    }

    /// Returns contour-pair validation schedule size from delegated role assignment, if reached.
    pub const fn boundary_build_validation_candidate_pair_count(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_validation_candidate_pair_count(),
            None => None,
        }
    }

    /// Returns contour-pair validation test count from delegated role assignment, if reached.
    pub const fn boundary_build_validation_tested_pair_count(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_validation_tested_pair_count(),
            None => None,
        }
    }

    /// Returns exact validation intersection event count from delegated role assignment.
    pub const fn boundary_build_validation_intersection_event_count(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_validation_intersection_event_count(),
            None => None,
        }
    }

    /// Returns containment classification count from delegated role assignment, if reached.
    pub const fn boundary_build_nesting_classification_count(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_nesting_classification_count(),
            None => None,
        }
    }

    /// Returns first blocking contour index from delegated role assignment, if present.
    pub const fn boundary_build_blocker_first_contour_index(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_blocker_first_contour_index(),
            None => None,
        }
    }

    /// Returns second blocking contour index from delegated role assignment, if present.
    pub const fn boundary_build_blocker_second_contour_index(&self) -> Option<usize> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_build_blocker_second_contour_index(),
            None => None,
        }
    }

    /// Returns final boundary output summary when role assignment materialized output.
    pub(crate) const fn boundary_output_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputBoundaryCache2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.boundary_output_cache(),
            None => None,
        }
    }

    /// Returns final boundary output counts grouped by material/hole role.
    pub(crate) const fn boundary_output_role_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputBoundaryRoleBucketCache2> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.role_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained final boundary role bucket count.
    pub const fn boundary_output_role_bucket_count(&self) -> Option<usize> {
        match self.boundary_output_role_bucket_cache() {
            Some(role_cache) => Some(role_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained final boundary output contour references grouped by role.
    pub const fn boundary_output_role_contour_count(&self) -> Option<usize> {
        match self.boundary_output_role_bucket_cache() {
            Some(role_cache) => Some(role_cache.output_contour_count()),
            None => None,
        }
    }

    /// Returns retained final boundary output segment references grouped by role.
    pub const fn boundary_output_role_segment_count(&self) -> Option<usize> {
        match self.boundary_output_role_bucket_cache() {
            Some(role_cache) => Some(role_cache.output_segment_count()),
            None => None,
        }
    }

    /// Returns the largest retained output segment count for one boundary role.
    pub const fn boundary_output_role_max_segment_count(&self) -> Option<usize> {
        match self.boundary_output_role_bucket_cache() {
            Some(role_cache) => Some(role_cache.max_segment_count()),
            None => None,
        }
    }

    /// Returns final output contour count retained after boundary role assignment.
    pub const fn output_contour_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.output_contour_count()),
            None => None,
        }
    }

    /// Returns final output boundary segment count retained after boundary role assignment.
    pub const fn output_segment_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.output_segment_count()),
            None => None,
        }
    }

    /// Returns final output boundary primitive-family counts after role assignment.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.output_segment_kind_counts()),
            None => None,
        }
    }

    /// Returns material contour count retained by role assignment.
    pub const fn material_contour_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.material_contour_count()),
            None => None,
        }
    }

    /// Returns hole contour count retained by role assignment.
    pub const fn hole_contour_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.hole_contour_count()),
            None => None,
        }
    }

    /// Returns material boundary segment count retained by role assignment.
    pub const fn material_segment_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.material_segment_count()),
            None => None,
        }
    }

    /// Returns hole boundary segment count retained by role assignment.
    pub const fn hole_segment_count(&self) -> Option<usize> {
        match self.boundary_output_cache() {
            Some(boundary_cache) => Some(boundary_cache.hole_segment_count()),
            None => None,
        }
    }

    /// Returns output role assignment buckets retained after boundary role assignment.
    pub(crate) const fn role_cache(&self) -> Option<&ExactCurveArrangementOutputRoleCache2> {
        match self.output_cache() {
            Some(output_cache) => output_cache.role_cache(),
            None => None,
        }
    }

    /// Returns retained output role evidence count when role assignment was reached.
    pub const fn role_evidence_count(&self) -> Option<usize> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_evidence_count()),
            None => None,
        }
    }

    /// Returns retained output role evidence when role assignment was reached.
    pub fn role_evidence(&self) -> Option<&[RegionBoundaryContourRoleEvidence2]> {
        self.boundary_build_evidence()
            .map(RegionBoundaryContourBuildEvidence2::role_evidence)
    }

    /// Returns output role assignment buckets grouped by topology status.
    pub(crate) const fn role_status_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleStatusBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_status_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained output role topology-status bucket count.
    pub const fn role_status_bucket_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.bucket_count()),
            None => None,
        }
    }

    /// Returns retained output role assignment references grouped by topology status.
    pub const fn role_status_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.assignment_ref_count()),
            None => None,
        }
    }

    /// Returns retained native-exact output role assignment references.
    pub const fn role_native_exact_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.native_exact_ref_count()),
            None => None,
        }
    }

    /// Returns retained certified-approximation output role assignment references.
    pub const fn role_certified_approximation_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.certified_approximation_ref_count()),
            None => None,
        }
    }

    /// Returns retained display/export-only output role assignment references.
    pub const fn role_display_or_export_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.display_or_export_ref_count()),
            None => None,
        }
    }

    /// Returns retained lossy-import output role assignment references.
    pub const fn role_imported_lossy_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.imported_lossy_ref_count()),
            None => None,
        }
    }

    /// Returns retained unsupported output role assignment references.
    pub const fn role_unsupported_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.unsupported_ref_count()),
            None => None,
        }
    }

    /// Returns retained unresolved output role assignment references.
    pub const fn role_unresolved_assignment_ref_count(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.unresolved_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained output role topology-status bucket size.
    pub const fn role_status_max_bucket_size(&self) -> Option<usize> {
        match self.role_status_bucket_cache() {
            Some(status_cache) => Some(status_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns output role assignment buckets grouped by source contour identity.
    pub(crate) const fn role_source_contour_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleSourceContourBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_source_contour_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained output role source-contour bucket count.
    pub const fn role_source_contour_bucket_count(&self) -> Option<usize> {
        match self.role_source_contour_bucket_cache() {
            Some(source_cache) => Some(source_cache.source_contour_bucket_count()),
            None => None,
        }
    }

    /// Returns retained output role assignment references grouped by source contour.
    pub const fn role_source_contour_assignment_ref_count(&self) -> Option<usize> {
        match self.role_source_contour_bucket_cache() {
            Some(source_cache) => Some(source_cache.assignment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained source-contour output role bucket size.
    pub const fn role_source_contour_max_bucket_size(&self) -> Option<usize> {
        match self.role_source_contour_bucket_cache() {
            Some(source_cache) => Some(source_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns output role assignment buckets grouped by exact nesting depth.
    pub(crate) const fn role_nesting_depth_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleNestingDepthBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_nesting_depth_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained output role nesting-depth bucket count.
    pub const fn role_nesting_depth_bucket_count(&self) -> Option<usize> {
        match self.role_nesting_depth_bucket_cache() {
            Some(depth_cache) => Some(depth_cache.nesting_depth_bucket_count()),
            None => None,
        }
    }

    /// Returns retained output role assignment references grouped by nesting depth.
    pub const fn role_nesting_depth_assignment_ref_count(&self) -> Option<usize> {
        match self.role_nesting_depth_bucket_cache() {
            Some(depth_cache) => Some(depth_cache.assignment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained nesting-depth output role bucket size.
    pub const fn role_nesting_depth_max_bucket_size(&self) -> Option<usize> {
        match self.role_nesting_depth_bucket_cache() {
            Some(depth_cache) => Some(depth_cache.max_bucket_size()),
            None => None,
        }
    }

    /// Returns output role containment evidence grouped by containing source contour.
    pub(crate) const fn role_containment_bucket_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputRoleContainmentBucketCache2> {
        match self.role_cache() {
            Some(role_cache) => Some(role_cache.role_containment_bucket_cache()),
            None => None,
        }
    }

    /// Returns retained output role containing-contour bucket count.
    pub const fn role_containment_bucket_count(&self) -> Option<usize> {
        match self.role_containment_bucket_cache() {
            Some(containment_cache) => Some(containment_cache.containing_contour_bucket_count()),
            None => None,
        }
    }

    /// Returns retained output role containment references.
    pub const fn role_containment_ref_count(&self) -> Option<usize> {
        match self.role_containment_bucket_cache() {
            Some(containment_cache) => Some(containment_cache.containment_ref_count()),
            None => None,
        }
    }

    /// Returns retained output role assignments with no containing contour.
    pub const fn role_uncontained_assignment_ref_count(&self) -> Option<usize> {
        match self.role_containment_bucket_cache() {
            Some(containment_cache) => Some(containment_cache.uncontained_assignment_ref_count()),
            None => None,
        }
    }

    /// Returns the largest retained containing-contour bucket size.
    pub const fn role_containment_max_bucket_size(&self) -> Option<usize> {
        match self.role_containment_bucket_cache() {
            Some(containment_cache) => Some(containment_cache.max_bucket_size()),
            None => None,
        }
    }
}

impl ExactCurveArrangementSourceEndpointRef2 {}

impl ExactCurveArrangementSourceEndpointBucket2 {
    /// Returns the exact structural point shared by this source endpoint bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
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
}

impl ExactCurveArrangementSplitCandidatePair2 {
    /// Returns the retained AABB pruning status for this scheduled pair.
    pub const fn aabb_status(&self) -> ExactCurveArrangementSplitCandidateAabbStatus2 {
        self.aabb_status
    }
}

impl ExactCurveArrangementSplitScheduleRef2 {}

impl ExactCurveArrangementSplitScheduleBucket2 {}

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
    pub(crate) const fn bucket_cache(&self) -> &ExactCurveArrangementSplitScheduleBucketCache2 {
        &self.bucket_cache
    }
}

impl ExactCurveArrangementSplitCache2 {
    fn from_region_build_evidence(evidence: &RegionLineSegmentRegionBuildEvidence2) -> Self {
        let intersection_bucket_cache =
            split_intersection_bucket_cache(evidence.split_intersection_evidence());
        let intersection_parameter_cache =
            ExactCurveArrangementSplitIntersectionParameterCache2::from_intersection_evidence(
                evidence.split_intersection_evidence(),
            );
        let relation_bucket_cache = ExactCurveArrangementSplitRelationBucketCache2::from_counts(
            evidence.split_point_relation_count(),
            evidence.split_overlap_relation_count(),
            evidence.split_uncertain_relation_count(),
        );
        let blocker_cache =
            ExactCurveArrangementSplitBlockerCache2::from_region_build_evidence(evidence);
        let intersection_evidence = evidence.split_intersection_evidence().to_vec();
        let intersection_points = intersection_evidence
            .iter()
            .map(|evidence| evidence.point().clone())
            .collect::<Vec<_>>();
        Self {
            predicate_path: evidence.split_predicate_path(),
            candidate_pair_count: evidence.split_candidate_pair_count(),
            skipped_aabb_pair_count: evidence.split_skipped_aabb_pair_count(),
            tested_pair_count: evidence.split_tested_pair_count(),
            intersection_event_count: intersection_evidence.len(),
            point_relation_count: evidence.split_point_relation_count(),
            overlap_relation_count: evidence.split_overlap_relation_count(),
            uncertain_relation_count: evidence.split_uncertain_relation_count(),
            intersection_points,
            intersection_evidence,
            relation_bucket_cache,
            intersection_bucket_cache,
            intersection_parameter_cache,
            blocker_cache,
            output_segment_count: evidence.split_output_segment_count(),
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
    pub fn intersection_evidence(&self) -> &[RegionLineSegmentSplitIntersectionEvidence2] {
        &self.intersection_evidence
    }

    /// Returns retained split-stage relation buckets.
    pub(crate) const fn relation_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSplitRelationBucketCache2 {
        &self.relation_bucket_cache
    }

    /// Returns exact split-intersection point buckets derived from retained split evidence.
    pub(crate) const fn intersection_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementSplitIntersectionBucketCache2 {
        &self.intersection_bucket_cache
    }

    /// Returns exact source-parameter evidence for retained split intersections.
    pub(crate) const fn intersection_parameter_cache(
        &self,
    ) -> &ExactCurveArrangementSplitIntersectionParameterCache2 {
        &self.intersection_parameter_cache
    }

    /// Returns split-stage blocker source-pair evidence, when split arrangement blocked.
    pub(crate) const fn blocker_cache(&self) -> Option<&ExactCurveArrangementSplitBlockerCache2> {
        self.blocker_cache.as_ref()
    }

    /// Returns arranged output segment count when splitting completed.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }
}

impl ExactCurveArrangementSplitBlockerCache2 {
    fn from_region_build_evidence(
        evidence: &RegionLineSegmentRegionBuildEvidence2,
    ) -> Option<Self> {
        Some(Self {
            first_source_segment_index: evidence.split_blocker_first_source_segment_index()?,
            first_source_segment_kind: evidence.split_blocker_first_source_segment_kind()?,
            first_source_start_point: evidence.split_blocker_first_source_start_point()?.clone(),
            first_source_end_point: evidence.split_blocker_first_source_end_point()?.clone(),
            second_source_segment_index: evidence.split_blocker_second_source_segment_index()?,
            second_source_segment_kind: evidence.split_blocker_second_source_segment_kind()?,
            second_source_start_point: evidence.split_blocker_second_source_start_point()?.clone(),
            second_source_end_point: evidence.split_blocker_second_source_end_point()?.clone(),
            blocker: evidence.blocker(),
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
}

impl ExactCurveArrangementSplitRelationBucket2 {}

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

    /// Returns the largest relation bucket size.
    pub const fn max_bucket_size(&self) -> usize {
        self.max_bucket_size
    }
}

impl ExactCurveArrangementSplitIntersectionRef2 {}

impl ExactCurveArrangementSplitIntersectionBucket2 {
    /// Returns the exact structural point shared by this split-intersection bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }
}

impl ExactCurveArrangementSplitIntersectionBucketCache2 {
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
}

impl ExactCurveArrangementSplitIntersectionParameterRef2 {}

impl ExactCurveArrangementSplitIntersectionParameterCache2 {
    fn from_intersection_evidence(
        intersection_evidence: &[RegionLineSegmentSplitIntersectionEvidence2],
    ) -> Self {
        let mut parameters = Vec::new();

        for (intersection_evidence_index, evidence) in intersection_evidence.iter().enumerate() {
            parameters.push(ExactCurveArrangementSplitIntersectionParameterRef2 {
                intersection_evidence_index,
                first_source_segment_index: evidence.first_source_segment_index(),
                first_source_param: evidence.first_source_param().clone(),
                second_source_segment_index: evidence.second_source_segment_index(),
                second_source_param: evidence.second_source_param().clone(),
                point: evidence.point().clone(),
            });
        }

        Self {
            intersection_event_count: parameters.len(),
            source_parameter_ref_count: parameters.len().saturating_mul(2),
            parameters,
        }
    }

    /// Returns the number of retained source parameter references.
    pub const fn source_parameter_ref_count(&self) -> usize {
        self.source_parameter_ref_count
    }
}

impl ExactCurveArrangementEndpointGraphCache2 {
    fn from_region_build_evidence(
        evidence: &RegionLineSegmentRegionBuildEvidence2,
    ) -> Option<Self> {
        let endpoint_bucket_cache =
            arranged_endpoint_bucket_cache(evidence.arranged_source_evidence());
        let endpoint_side_bucket_cache =
            ExactCurveArrangementArrangedEndpointSideBucketCache2::from_arranged_source_evidence(
                evidence.arranged_source_evidence(),
            );
        let endpoint_point_cache =
            ExactCurveArrangementArrangedEndpointPointCache2::from_arranged_source_evidence(
                evidence.arranged_source_evidence(),
            );
        let endpoint_degree_bucket_cache =
            ExactCurveArrangementArrangedEndpointDegreeBucketCache2::from_endpoint_bucket_cache(
                &endpoint_bucket_cache,
            );
        let dangling_endpoint_count = endpoint_bucket_cache
            .buckets()
            .iter()
            .filter(|bucket| bucket.endpoints().len() == 1)
            .map(|bucket| bucket.endpoints().len())
            .sum();
        let branch_endpoint_count = endpoint_bucket_cache
            .buckets()
            .iter()
            .filter(|bucket| bucket.endpoints().len() > 2)
            .map(|bucket| bucket.endpoints().len())
            .sum();
        Some(Self {
            predicate_path: evidence.endpoint_graph_predicate_path()?,
            endpoint_count: endpoint_bucket_cache.endpoint_count(),
            structural_bucket_count: endpoint_bucket_cache.bucket_count(),
            structural_singleton_bucket_count: endpoint_bucket_cache.singleton_bucket_count(),
            max_structural_bucket_size: endpoint_bucket_cache.max_bucket_size(),
            endpoint_bucket_cache,
            endpoint_side_bucket_cache,
            endpoint_point_cache,
            endpoint_degree_bucket_cache,
            dangling_endpoint_count,
            branch_endpoint_count,
            blocker_arranged_segment_index: evidence
                .endpoint_graph_blocker_arranged_segment_index(),
            blocker_endpoint: evidence.endpoint_graph_blocker_endpoint(),
            blocker_point: evidence.endpoint_graph_blocker_point().cloned(),
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

    /// Returns exact arranged endpoint buckets derived from retained arranged source evidence.
    pub(crate) const fn endpoint_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedEndpointBucketCache2 {
        &self.endpoint_bucket_cache
    }

    /// Returns arranged endpoints grouped by retained endpoint side.
    pub(crate) const fn endpoint_side_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedEndpointSideBucketCache2 {
        &self.endpoint_side_bucket_cache
    }

    /// Returns exact endpoint records for arranged fragments.
    pub(crate) const fn endpoint_point_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedEndpointPointCache2 {
        &self.endpoint_point_cache
    }

    /// Returns structural arranged endpoints grouped by retained degree.
    pub(crate) const fn endpoint_degree_bucket_cache(
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

impl ExactCurveArrangementArrangedEndpointDegreeRef2 {}

impl ExactCurveArrangementArrangedEndpointDegreeBucket2 {}

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
}

impl ExactCurveArrangementArrangedEndpointRef2 {}

impl ExactCurveArrangementArrangedEndpointSideBucket2 {}

impl ExactCurveArrangementArrangedEndpointSideBucketCache2 {
    fn from_arranged_source_evidence(
        arranged_source_evidence: &[RegionLineSegmentArrangedSourceEvidence2],
    ) -> Self {
        let mut start_refs = Vec::new();
        let mut end_refs = Vec::new();

        for evidence in arranged_source_evidence {
            let arranged_segment_index = evidence.arranged_segment_index();
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
        let endpoint_ref_count = arranged_source_evidence.len().saturating_mul(2);
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
}

impl ExactCurveArrangementArrangedEndpointBucket2 {
    /// Returns the exact structural point shared by this arranged endpoint bucket.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns arranged endpoints in retained evidence encounter order.
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

impl ExactCurveArrangementArrangedEndpointPointRef2 {}

impl ExactCurveArrangementArrangedEndpointPointCache2 {
    fn from_arranged_source_evidence(
        arranged_source_evidence: &[RegionLineSegmentArrangedSourceEvidence2],
    ) -> Self {
        let mut endpoints: Vec<ExactCurveArrangementArrangedEndpointPointRef2> = Vec::new();

        for source_evidence in arranged_source_evidence {
            let arranged_segment_index = source_evidence.arranged_segment_index();
            if endpoints
                .iter()
                .any(|endpoint| endpoint.arranged_segment_index == arranged_segment_index)
            {
                continue;
            }

            endpoints.push(ExactCurveArrangementArrangedEndpointPointRef2 {
                arranged_segment_index,
                output_start_point: source_evidence.output_start_point().clone(),
                output_end_point: source_evidence.output_end_point().clone(),
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
}

impl ExactCurveArrangementArrangedFragmentSourceRef2 {
    /// Returns the retained arranged source evidence index.
    pub const fn arranged_source_evidence_index(&self) -> usize {
        self.arranged_source_evidence_index
    }

    /// Returns retained topology status for this source-to-fragment mapping.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl ExactCurveArrangementArrangedFragment2 {
    /// Returns the primitive family of the arranged fragment.
    pub const fn arranged_segment_kind(&self) -> SegmentKind {
        self.arranged_segment_kind
    }

    /// Returns retained source provenance references for this arranged fragment.
    pub fn source_refs(&self) -> &[ExactCurveArrangementArrangedFragmentSourceRef2] {
        &self.source_refs
    }
}

impl ExactCurveArrangementArrangedFragmentRef2 {}

impl ExactCurveArrangementArrangedFragmentStatusRef2 {}

impl ExactCurveArrangementArrangedFragmentKindBucket2 {}

impl ExactCurveArrangementArrangedFragmentStatusBucket2 {}

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
                    arranged_source_evidence_index: source_ref.arranged_source_evidence_index(),
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
}

impl ExactCurveArrangementArrangedFragmentSourceRangeRef2 {}

impl ExactCurveArrangementArrangedFragmentSourceRangeCache2 {
    fn from_arranged_source_evidence(
        arranged_source_evidence: &[RegionLineSegmentArrangedSourceEvidence2],
    ) -> Self {
        let mut full_source_range_ref_count = 0_usize;
        let mut partial_source_range_ref_count = 0_usize;
        let mut ranges = Vec::new();

        for (arranged_source_evidence_index, source_evidence) in
            arranged_source_evidence.iter().enumerate()
        {
            if source_range_is_full(source_evidence.source_range()) {
                full_source_range_ref_count += 1;
            } else {
                partial_source_range_ref_count += 1;
            }

            ranges.push(ExactCurveArrangementArrangedFragmentSourceRangeRef2 {
                arranged_source_evidence_index,
                source_segment_index: source_evidence.source_segment_index(),
                source_range: source_evidence.source_range().clone(),
                arranged_segment_index: source_evidence.arranged_segment_index(),
            });
        }

        ranges.sort_by_key(|range_ref| {
            (
                range_ref.arranged_segment_index,
                range_ref.arranged_source_evidence_index,
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
}

impl ExactCurveArrangementArrangedFragmentCache2 {
    fn from_arranged_source_evidence(
        arranged_source_evidence: &[RegionLineSegmentArrangedSourceEvidence2],
    ) -> Self {
        let mut fragments: Vec<ExactCurveArrangementArrangedFragment2> = Vec::new();
        let mut source_segment_kind_counts = SegmentKindCounts::default();

        for (arranged_source_evidence_index, evidence) in
            arranged_source_evidence.iter().enumerate()
        {
            match evidence.source_segment_kind() {
                SegmentKind::Line => source_segment_kind_counts.lines += 1,
                SegmentKind::Arc => source_segment_kind_counts.arcs += 1,
            }

            let arranged_segment_index = evidence.arranged_segment_index();
            let fragment_index = fragments
                .iter()
                .position(|fragment| fragment.arranged_segment_index == arranged_segment_index)
                .unwrap_or_else(|| {
                    fragments.push(ExactCurveArrangementArrangedFragment2 {
                        arranged_segment_index,
                        arranged_segment_kind: evidence.arranged_segment_kind(),
                        output_start_point: evidence.output_start_point().clone(),
                        output_end_point: evidence.output_end_point().clone(),
                        source_refs: Vec::new(),
                    });
                    fragments.len() - 1
                });
            fragments[fragment_index].source_refs.push(
                ExactCurveArrangementArrangedFragmentSourceRef2 {
                    arranged_source_evidence_index,
                    source_segment_index: evidence.source_segment_index(),
                    source_segment_kind: evidence.source_segment_kind(),
                    source_range: evidence.source_range().clone(),
                    status: evidence.status(),
                },
            );
        }

        fragments.sort_by_key(|fragment| fragment.arranged_segment_index);
        for fragment in &mut fragments {
            fragment
                .source_refs
                .sort_by_key(|source_ref| source_ref.arranged_source_evidence_index);
        }

        let source_ref_count = arranged_source_evidence.len();
        let arranged_segment_kind_counts =
            arranged_evidence_segment_kind_counts(arranged_source_evidence);
        let arranged_fragment_kind_bucket_cache =
            ExactCurveArrangementArrangedFragmentKindBucketCache2::from_fragments(&fragments);
        let arranged_fragment_status_bucket_cache =
            ExactCurveArrangementArrangedFragmentStatusBucketCache2::from_fragments(&fragments);
        let arranged_fragment_source_range_cache =
            ExactCurveArrangementArrangedFragmentSourceRangeCache2::from_arranged_source_evidence(
                arranged_source_evidence,
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

    /// Returns arranged fragment primitive-family counts after exact splitting.
    pub const fn arranged_segment_kind_counts(&self) -> SegmentKindCounts {
        self.arranged_segment_kind_counts
    }

    /// Returns retained arranged fragment buckets grouped by primitive family.
    pub(crate) const fn arranged_fragment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentKindBucketCache2 {
        &self.arranged_fragment_kind_bucket_cache
    }

    /// Returns retained arranged fragment source buckets grouped by topology status.
    pub(crate) const fn arranged_fragment_status_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentStatusBucketCache2 {
        &self.arranged_fragment_status_bucket_cache
    }

    /// Returns retained arranged fragment source-parameter range records.
    pub(crate) const fn arranged_fragment_source_range_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentSourceRangeCache2 {
        &self.arranged_fragment_source_range_cache
    }
}

impl ExactCurveArrangementOutputRingSegmentRef2 {}

impl ExactCurveArrangementOutputRingBucket2 {}

impl ExactCurveArrangementOutputRingBucketCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut rings: Vec<ExactCurveArrangementOutputRingBucket2> = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            let output_ring_index = source_evidence.output_ring_index();
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
                    source_evidence_index,
                    output_segment_index: source_evidence.output_segment_index(),
                    reversed: source_evidence.reversed(),
                });
        }

        rings.sort_by_key(|ring| ring.output_ring_index);
        for ring in &mut rings {
            ring.segments
                .sort_by_key(|segment| segment.output_segment_index);
        }

        let segment_ref_count = source_evidence.len();
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
}

impl ExactCurveArrangementOutputSegmentKindRef2 {}

impl ExactCurveArrangementOutputSegmentKindBucket2 {}

impl ExactCurveArrangementOutputSegmentKindBucketCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut line_refs = Vec::new();
        let mut arc_refs = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            let segment_ref = ExactCurveArrangementOutputSegmentKindRef2 {
                source_evidence_index,
                output_ring_index: source_evidence.output_ring_index(),
                output_segment_index: source_evidence.output_segment_index(),
            };
            match source_evidence.output_segment_kind() {
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
        let output_segment_ref_count = source_evidence.len();
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
}

impl ExactCurveArrangementOutputSegmentSourceRef2 {}

impl ExactCurveArrangementOutputSegmentSourceBucket2 {}

impl ExactCurveArrangementOutputSegmentSourceBucketCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut buckets: Vec<ExactCurveArrangementOutputSegmentSourceBucket2> = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            let source_segment_index = source_evidence.source_segment_index();
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
                    source_evidence_index,
                    output_ring_index: source_evidence.output_ring_index(),
                    output_segment_index: source_evidence.output_segment_index(),
                });
        }

        buckets.sort_by_key(|bucket| bucket.source_segment_index);
        for bucket in &mut buckets {
            bucket.segment_refs.sort_by_key(|segment_ref| {
                (
                    segment_ref.output_ring_index,
                    segment_ref.output_segment_index,
                    segment_ref.source_evidence_index,
                )
            });
        }

        let output_segment_ref_count = source_evidence.len();
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
}

impl ExactCurveArrangementOutputSegmentSourceRangeRef2 {}

impl ExactCurveArrangementOutputSegmentSourceRangeCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut full_source_range_ref_count = 0_usize;
        let mut partial_source_range_ref_count = 0_usize;
        let mut ranges = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            if source_range_is_full(source_evidence.source_range()) {
                full_source_range_ref_count += 1;
            } else {
                partial_source_range_ref_count += 1;
            }

            ranges.push(ExactCurveArrangementOutputSegmentSourceRangeRef2 {
                source_evidence_index,
                source_segment_index: source_evidence.source_segment_index(),
                source_range: source_evidence.source_range().clone(),
                output_ring_index: source_evidence.output_ring_index(),
                output_segment_index: source_evidence.output_segment_index(),
            });
        }

        ranges.sort_by_key(|range_ref| {
            (
                range_ref.output_ring_index,
                range_ref.output_segment_index,
                range_ref.source_evidence_index,
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
}

impl ExactCurveArrangementOutputSegmentEndpointRef2 {}

impl ExactCurveArrangementOutputSegmentEndpointCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut segments = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            segments.push(ExactCurveArrangementOutputSegmentEndpointRef2 {
                source_evidence_index,
                output_ring_index: source_evidence.output_ring_index(),
                output_segment_index: source_evidence.output_segment_index(),
                output_start_point: source_evidence.output_start_point().clone(),
                output_end_point: source_evidence.output_end_point().clone(),
            });
        }

        segments.sort_by_key(|segment| {
            (
                segment.output_ring_index,
                segment.output_segment_index,
                segment.source_evidence_index,
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
}

impl ExactCurveArrangementOutputRingContinuityRef2 {}

impl ExactCurveArrangementOutputRingContinuityCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut rings: Vec<Vec<(usize, &RegionLineSegmentRingSourceEvidence2)>> = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            let output_ring_index = source_evidence.output_ring_index();
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
            rings[ring_index].push((source_evidence_index, source_evidence));
        }

        for ring in &mut rings {
            ring.sort_by_key(|(_, source_evidence)| source_evidence.output_segment_index());
        }
        rings.sort_by_key(|ring| {
            ring.first().map_or(usize::MAX, |(_, source_evidence)| {
                source_evidence.output_ring_index()
            })
        });

        let output_ring_ref_count = rings.len();
        let max_ring_connection_count = rings.iter().map(Vec::len).max().unwrap_or(0);
        let mut connections = Vec::new();

        for ring in rings {
            for (segment_index, (source_evidence_index, source_evidence)) in ring.iter().enumerate()
            {
                let (next_source_evidence_index, next_source_evidence) =
                    &ring[(segment_index + 1) % ring.len()];
                connections.push(ExactCurveArrangementOutputRingContinuityRef2 {
                    source_evidence_index: *source_evidence_index,
                    next_source_evidence_index: *next_source_evidence_index,
                    output_ring_index: source_evidence.output_ring_index(),
                    output_segment_index: source_evidence.output_segment_index(),
                    next_output_segment_index: next_source_evidence.output_segment_index(),
                    output_end_point: source_evidence.output_end_point().clone(),
                    next_output_start_point: next_source_evidence.output_start_point().clone(),
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
}

impl ExactCurveArrangementOutputSegmentStatusRef2 {}

impl ExactCurveArrangementOutputSegmentStatusBucket2 {}

impl ExactCurveArrangementOutputSegmentStatusBucketCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut native_exact_refs = Vec::new();
        let mut certified_approximation_refs = Vec::new();
        let mut display_or_export_refs = Vec::new();
        let mut imported_lossy_refs = Vec::new();
        let mut unsupported_refs = Vec::new();
        let mut unresolved_refs = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            let segment_ref = ExactCurveArrangementOutputSegmentStatusRef2 {
                source_evidence_index,
                output_ring_index: source_evidence.output_ring_index(),
                output_segment_index: source_evidence.output_segment_index(),
            };
            match source_evidence.status() {
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
        let output_segment_ref_count = source_evidence.len();
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
}

impl ExactCurveArrangementOutputSegmentDirectionRef2 {}

impl ExactCurveArrangementOutputSegmentDirectionBucket2 {}

impl ExactCurveArrangementOutputSegmentDirectionBucketCache2 {
    fn from_source_evidence(source_evidence: &[RegionLineSegmentRingSourceEvidence2]) -> Self {
        let mut forward_refs = Vec::new();
        let mut reversed_refs = Vec::new();

        for (source_evidence_index, source_evidence) in source_evidence.iter().enumerate() {
            let segment_ref = ExactCurveArrangementOutputSegmentDirectionRef2 {
                source_evidence_index,
                output_ring_index: source_evidence.output_ring_index(),
                output_segment_index: source_evidence.output_segment_index(),
            };
            if source_evidence.reversed() {
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
        let output_segment_ref_count = source_evidence.len();
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
}

impl ExactCurveArrangementOutputRoleAssignment2 {
    /// Returns the retained boundary role evidence index.
    pub const fn role_evidence_index(&self) -> usize {
        self.role_evidence_index
    }

    /// Returns the source contour index assigned by this evidence.
    pub const fn source_contour_index(&self) -> usize {
        self.source_contour_index
    }

    /// Returns the source contour segment count captured before role binning.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
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

impl ExactCurveArrangementOutputRoleStatusRef2 {}

impl ExactCurveArrangementOutputRoleBucket2 {}

impl ExactCurveArrangementOutputRoleStatusBucket2 {}

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
                    role_evidence_index: assignment.role_evidence_index(),
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
}

impl ExactCurveArrangementOutputRoleSourceContourRef2 {}

impl ExactCurveArrangementOutputRoleSourceContourBucket2 {}

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
                        role_evidence_index: assignment.role_evidence_index(),
                        output_role_index: assignment.output_role_index(),
                    },
                );
            }
        }

        buckets.sort_by_key(|bucket| bucket.source_contour_index);
        for bucket in &mut buckets {
            bucket.assignments.sort_by_key(|assignment| {
                (
                    assignment.role_evidence_index,
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
}

impl ExactCurveArrangementOutputRoleNestingDepthRef2 {}

impl ExactCurveArrangementOutputRoleNestingDepthBucket2 {}

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
                        role_evidence_index: assignment.role_evidence_index(),
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
                    assignment.role_evidence_index,
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
}

impl ExactCurveArrangementOutputRoleContainmentRef2 {}

impl ExactCurveArrangementOutputRoleContainmentBucket2 {}

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
                            role_evidence_index: assignment.role_evidence_index(),
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
                    containment.role_evidence_index,
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
}

impl ExactCurveArrangementOutputRoleCache2 {
    fn from_boundary_build_evidence(
        evidence: &RegionBoundaryContourBuildEvidence2,
    ) -> Option<Self> {
        if evidence.role_evidence().is_empty() {
            return None;
        }

        let mut material_assignments = Vec::new();
        let mut hole_assignments = Vec::new();

        for (role_evidence_index, role_evidence) in evidence.role_evidence().iter().enumerate() {
            let assignment = ExactCurveArrangementOutputRoleAssignment2 {
                role_evidence_index,
                source_contour_index: role_evidence.source_contour_index(),
                source_segment_count: role_evidence.source_segment_count(),
                source_fill_rule: role_evidence.source_fill_rule(),
                nesting_sample_point: role_evidence.nesting_sample_point().clone(),
                containing_contour_indices: role_evidence.containing_contour_indices().to_vec(),
                nesting_depth: role_evidence.nesting_depth(),
                output_role_index: role_evidence.output_role_index(),
                status: role_evidence.status(),
            };
            match role_evidence.role() {
                RegionBoundaryContourRole2::Material => material_assignments.push(assignment),
                RegionBoundaryContourRole2::Hole => hole_assignments.push(assignment),
            }
        }

        material_assignments.sort_by_key(|assignment| assignment.output_role_index);
        hole_assignments.sort_by_key(|assignment| assignment.output_role_index);
        let material_contour_count = material_assignments.len();
        let hole_contour_count = hole_assignments.len();
        let material_segment_count = material_assignments
            .iter()
            .map(ExactCurveArrangementOutputRoleAssignment2::source_segment_count)
            .sum();
        let hole_segment_count = hole_assignments
            .iter()
            .map(ExactCurveArrangementOutputRoleAssignment2::source_segment_count)
            .sum();
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
            role_evidence_count: evidence.role_evidence().len(),
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

    /// Returns the number of retained role evidence.
    pub const fn role_evidence_count(&self) -> usize {
        self.role_evidence_count
    }

    /// Returns output role assignment buckets grouped by topology status.
    pub(crate) const fn role_status_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleStatusBucketCache2 {
        &self.role_status_bucket_cache
    }

    /// Returns output role assignment buckets grouped by source contour identity.
    pub(crate) const fn role_source_contour_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleSourceContourBucketCache2 {
        &self.role_source_contour_bucket_cache
    }

    /// Returns output role assignment buckets grouped by exact nesting depth.
    pub(crate) const fn role_nesting_depth_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleNestingDepthBucketCache2 {
        &self.role_nesting_depth_bucket_cache
    }

    /// Returns output role containment evidence grouped by containing source contour.
    pub(crate) const fn role_containment_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRoleContainmentBucketCache2 {
        &self.role_containment_bucket_cache
    }
}

impl ExactCurveArrangementOutputBoundaryCache2 {
    fn from_region_build_evidence(
        evidence: &RegionLineSegmentRegionBuildEvidence2,
    ) -> Option<Self> {
        let boundary_build_evidence = evidence.boundary_build_evidence()?;
        if boundary_build_evidence.role_evidence().is_empty() {
            return None;
        }

        let mut material_contour_count = 0_usize;
        let mut hole_contour_count = 0_usize;
        let mut material_segment_count = 0_usize;
        let mut hole_segment_count = 0_usize;

        for role_evidence in boundary_build_evidence.role_evidence() {
            match role_evidence.role() {
                RegionBoundaryContourRole2::Material => {
                    material_contour_count += 1;
                    material_segment_count += role_evidence.source_segment_count();
                }
                RegionBoundaryContourRole2::Hole => {
                    hole_contour_count += 1;
                    hole_segment_count += role_evidence.source_segment_count();
                }
            }
        }

        let mut output_segment_kind_counts = SegmentKindCounts { lines: 0, arcs: 0 };
        for source_evidence in evidence.source_evidence() {
            match source_evidence.output_segment_kind() {
                SegmentKind::Line => output_segment_kind_counts.lines += 1,
                SegmentKind::Arc => output_segment_kind_counts.arcs += 1,
            }
        }
        let role_bucket_cache = ExactCurveArrangementOutputBoundaryRoleBucketCache2::new(
            material_contour_count,
            hole_contour_count,
            material_segment_count,
            hole_segment_count,
        );

        Some(Self {
            output_contour_count: role_bucket_cache.output_contour_count(),
            output_segment_count: role_bucket_cache.output_segment_count(),
            output_segment_kind_counts,
            material_contour_count,
            hole_contour_count,
            material_segment_count,
            hole_segment_count,
            role_bucket_cache,
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
    pub(crate) const fn role_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputBoundaryRoleBucketCache2 {
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
}

impl ExactCurveArrangementOutputBoundaryRoleBucket2 {}

impl ExactCurveArrangementRingAssemblyCache2 {
    fn from_region_build_evidence(
        evidence: &RegionLineSegmentRegionBuildEvidence2,
    ) -> Option<Self> {
        let predicate_path = evidence.ring_assembly_predicate_path()?;
        let arranged_source_evidence = evidence.arranged_source_evidence().to_vec();
        let source_evidence = evidence.source_evidence().to_vec();
        let arranged_fragment_cache =
            ExactCurveArrangementArrangedFragmentCache2::from_arranged_source_evidence(
                &arranged_source_evidence,
            );
        let output_ring_bucket_cache =
            ExactCurveArrangementOutputRingBucketCache2::from_source_evidence(&source_evidence);
        let output_segment_kind_bucket_cache =
            ExactCurveArrangementOutputSegmentKindBucketCache2::from_source_evidence(
                &source_evidence,
            );
        let output_segment_source_bucket_cache =
            ExactCurveArrangementOutputSegmentSourceBucketCache2::from_source_evidence(
                &source_evidence,
            );
        let output_segment_source_range_cache =
            ExactCurveArrangementOutputSegmentSourceRangeCache2::from_source_evidence(
                &source_evidence,
            );
        let output_segment_endpoint_cache =
            ExactCurveArrangementOutputSegmentEndpointCache2::from_source_evidence(
                &source_evidence,
            );
        let output_ring_continuity_cache =
            ExactCurveArrangementOutputRingContinuityCache2::from_source_evidence(&source_evidence);
        let output_segment_status_bucket_cache =
            ExactCurveArrangementOutputSegmentStatusBucketCache2::from_source_evidence(
                &source_evidence,
            );
        let output_segment_direction_bucket_cache =
            ExactCurveArrangementOutputSegmentDirectionBucketCache2::from_source_evidence(
                &source_evidence,
            );
        let has_output_segments = !source_evidence.is_empty();
        let output_boundary_segment_kind_counts =
            has_output_segments.then_some(SegmentKindCounts {
                lines: output_segment_kind_bucket_cache.line_segment_ref_count(),
                arcs: output_segment_kind_bucket_cache.arc_segment_ref_count(),
            });

        Some(Self {
            predicate_path,
            attempted_endpoint_connection_count: evidence.attempted_endpoint_connection_count(),
            exact_endpoint_connection_count: evidence.exact_endpoint_connection_count(),
            disconnected_endpoint_connection_count: evidence
                .disconnected_endpoint_connection_count(),
            unresolved_endpoint_connection_count: evidence.unresolved_endpoint_connection_count(),
            reversed_source_segment_count: output_segment_direction_bucket_cache
                .reversed_segment_ref_count(),
            output_ring_count: has_output_segments.then_some(output_ring_bucket_cache.ring_count()),
            output_boundary_segment_count: has_output_segments
                .then_some(output_segment_endpoint_cache.output_segment_ref_count()),
            output_boundary_segment_kind_counts,
            arranged_source_evidence,
            source_evidence,
            arranged_fragment_cache,
            output_ring_bucket_cache,
            output_segment_kind_bucket_cache,
            output_segment_source_bucket_cache,
            output_segment_source_range_cache,
            output_segment_endpoint_cache,
            output_ring_continuity_cache,
            output_segment_status_bucket_cache,
            output_segment_direction_bucket_cache,
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
    pub fn arranged_source_evidence(&self) -> &[RegionLineSegmentArrangedSourceEvidence2] {
        &self.arranged_source_evidence
    }

    /// Returns per-output segment source provenance.
    pub fn source_evidence(&self) -> &[RegionLineSegmentRingSourceEvidence2] {
        &self.source_evidence
    }

    /// Returns per-arranged-fragment source provenance buckets.
    pub(crate) const fn arranged_fragment_cache(
        &self,
    ) -> &ExactCurveArrangementArrangedFragmentCache2 {
        &self.arranged_fragment_cache
    }

    /// Returns per-output-ring source provenance buckets.
    pub(crate) const fn output_ring_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRingBucketCache2 {
        &self.output_ring_bucket_cache
    }

    /// Returns retained output segment buckets grouped by primitive family.
    pub(crate) const fn output_segment_kind_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentKindBucketCache2 {
        &self.output_segment_kind_bucket_cache
    }

    /// Returns retained output segment buckets grouped by source segment.
    pub(crate) const fn output_segment_source_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentSourceBucketCache2 {
        &self.output_segment_source_bucket_cache
    }

    /// Returns retained output segment source parameter ranges.
    pub(crate) const fn output_segment_source_range_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentSourceRangeCache2 {
        &self.output_segment_source_range_cache
    }

    /// Returns retained output segment exact endpoint records.
    pub(crate) const fn output_segment_endpoint_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentEndpointCache2 {
        &self.output_segment_endpoint_cache
    }

    /// Returns retained exact continuity records between adjacent output segments.
    pub(crate) const fn output_ring_continuity_cache(
        &self,
    ) -> &ExactCurveArrangementOutputRingContinuityCache2 {
        &self.output_ring_continuity_cache
    }

    /// Returns retained output segment buckets grouped by topology status.
    pub(crate) const fn output_segment_status_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentStatusBucketCache2 {
        &self.output_segment_status_bucket_cache
    }

    /// Returns retained output segment buckets grouped by traversal direction.
    pub(crate) const fn output_segment_direction_bucket_cache(
        &self,
    ) -> &ExactCurveArrangementOutputSegmentDirectionBucketCache2 {
        &self.output_segment_direction_bucket_cache
    }
}

impl ExactCurveArrangementOutputCache2 {
    fn from_region_build_evidence(
        evidence: &RegionLineSegmentRegionBuildEvidence2,
        materialized_region: bool,
    ) -> Self {
        let boundary_build_evidence = evidence.boundary_build_evidence().cloned();
        let boundary_output_cache =
            ExactCurveArrangementOutputBoundaryCache2::from_region_build_evidence(evidence);
        let role_cache = boundary_build_evidence
            .as_ref()
            .and_then(ExactCurveArrangementOutputRoleCache2::from_boundary_build_evidence);
        Self {
            materialized_region,
            boundary_build_evidence,
            boundary_output_cache,
            role_cache,
            stage: evidence.stage(),
            status: evidence.status(),
            blocker: evidence.blocker(),
        }
    }

    /// Returns whether the arrangement produced an owned region.
    pub const fn materialized_region(&self) -> bool {
        self.materialized_region
    }

    /// Returns delegated boundary-contour role assignment evidence, when reached.
    pub const fn boundary_build_evidence(&self) -> Option<&RegionBoundaryContourBuildEvidence2> {
        self.boundary_build_evidence.as_ref()
    }

    /// Returns delegated boundary-role assignment stage, if reached.
    pub const fn boundary_build_stage(&self) -> Option<RegionBoundaryContourBuildStage2> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.stage()),
            None => None,
        }
    }

    /// Returns delegated boundary-role assignment predicate path, if reached.
    pub const fn boundary_build_predicate_path(
        &self,
    ) -> Option<RegionBoundaryContourBuildPredicatePath2> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.predicate_path()),
            None => None,
        }
    }

    /// Returns delegated boundary-role assignment retained status, if reached.
    pub const fn boundary_build_status(&self) -> Option<RetainedTopologyStatus> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.status()),
            None => None,
        }
    }

    /// Returns delegated boundary-role assignment blocker, if present.
    pub const fn boundary_build_blocker(&self) -> Option<UncertaintyReason> {
        match self.boundary_build_evidence() {
            Some(evidence) => evidence.blocker(),
            None => None,
        }
    }

    /// Returns source contour count from delegated boundary-role assignment, if reached.
    pub const fn boundary_build_source_contour_count(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.source_contour_count()),
            None => None,
        }
    }

    /// Returns source boundary segment count from delegated boundary-role assignment, if reached.
    pub const fn boundary_build_source_segment_count(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.source_segment_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation schedule size from delegated role assignment, if reached.
    pub const fn boundary_build_validation_candidate_pair_count(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.validation_candidate_pair_count()),
            None => None,
        }
    }

    /// Returns contour-pair validation test count from delegated role assignment, if reached.
    pub const fn boundary_build_validation_tested_pair_count(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.validation_tested_pair_count()),
            None => None,
        }
    }

    /// Returns exact validation intersection event count from delegated role assignment, if reached.
    pub const fn boundary_build_validation_intersection_event_count(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.validation_intersection_event_count()),
            None => None,
        }
    }

    /// Returns containment classification count from delegated role assignment, if reached.
    pub const fn boundary_build_nesting_classification_count(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => Some(evidence.nesting_classification_count()),
            None => None,
        }
    }

    /// Returns first blocking contour index from delegated role assignment, if present.
    pub const fn boundary_build_blocker_first_contour_index(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => evidence.blocker_first_contour_index(),
            None => None,
        }
    }

    /// Returns second blocking contour index from delegated role assignment, if present.
    pub const fn boundary_build_blocker_second_contour_index(&self) -> Option<usize> {
        match self.boundary_build_evidence() {
            Some(evidence) => evidence.blocker_second_contour_index(),
            None => None,
        }
    }

    /// Returns retained final boundary output summary when available.
    pub(crate) const fn boundary_output_cache(
        &self,
    ) -> Option<&ExactCurveArrangementOutputBoundaryCache2> {
        self.boundary_output_cache.as_ref()
    }

    /// Returns retained material/hole role buckets when role assignment was reached.
    pub(crate) const fn role_cache(&self) -> Option<&ExactCurveArrangementOutputRoleCache2> {
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

impl RegionArrangementSummary2 {
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

fn evaluate_exact_curve_arrangement(
    request: ExactCurveArrangementRequest2,
    policy: &CurvePolicy,
) -> CurveResult<RegionArrangement2> {
    let staging_result = if let Some(source_line_segments) = request.source_line_segments.as_ref() {
        evaluate_unordered_line_segments_region_result(
            source_line_segments,
            request.fill_rule,
            policy,
        )?
    } else {
        evaluate_unordered_segments_region_result(
            &request.source_segments,
            request.fill_rule,
            policy,
        )?
    };
    let workspace = ExactCurveWorkspace2::from_request(request, policy)?
        .with_region_build_evidence(staging_result.evidence(), staging_result.region().is_some());
    let region = staging_result.region().cloned();
    let summary = RegionArrangementSummary2::from_workspace(&workspace);
    Ok(RegionArrangement2 {
        workspace: Rc::new(workspace),
        summary,
        region,
    })
}

impl RegionArrangement2 {
    fn facts(&self) -> &ExactCurveWorkspace2 {
        &self.workspace
    }

    /// Returns the source segments supplied to the retained arrangement request.
    pub fn source_segments(&self) -> &[Segment2] {
        self.facts().request().source_segments()
    }

    /// Returns line-only source carriers when the retained request came from a line-specific API.
    pub fn source_line_segments(&self) -> Option<&[LineSeg2]> {
        self.facts().request().source_line_segments()
    }

    /// Returns the fill rule retained by the arrangement request.
    pub fn fill_rule(&self) -> FillRule {
        self.facts().request().fill_rule()
    }

    /// Returns retained source segment count from the arrangement request.
    pub fn source_segment_count(&self) -> usize {
        self.facts().request().source_segment_count()
    }

    /// Returns retained source segment primitive-family counts.
    pub fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.facts().source_segment_kind_counts()
    }

    /// Returns retained source segment boxes in request order.
    pub fn source_segment_aabbs(&self) -> &[Option<Aabb2>] {
        self.facts().source_segment_aabbs()
    }

    /// Returns a retained aggregate source box when every source box was decided.
    pub fn source_aabb(&self) -> Option<&Aabb2> {
        self.facts().source_aabb()
    }

    /// Returns the number of source segment boxes certified during workspace construction.
    pub fn decided_source_segment_aabb_count(&self) -> usize {
        self.facts().decided_source_segment_aabb_count()
    }

    /// Returns the number of source segment boxes that stayed uncertain.
    pub fn undecided_source_segment_aabb_count(&self) -> usize {
        self.facts().undecided_source_segment_aabb_count()
    }

    /// Returns retained source AABB-status bucket count.
    pub fn source_aabb_bucket_count(&self) -> usize {
        self.facts().source_aabb_bucket_count()
    }

    /// Returns retained source AABB references.
    pub fn source_aabb_ref_count(&self) -> usize {
        self.facts().source_aabb_ref_count()
    }

    /// Returns retained source AABB references certified as decided.
    pub fn source_aabb_decided_ref_count(&self) -> usize {
        self.facts().source_aabb_decided_ref_count()
    }

    /// Returns retained source AABB references that stayed undecided.
    pub fn source_aabb_undecided_ref_count(&self) -> usize {
        self.facts().source_aabb_undecided_ref_count()
    }

    /// Returns the largest retained source AABB-status bucket size.
    pub fn source_aabb_max_bucket_size(&self) -> usize {
        self.facts().source_aabb_max_bucket_size()
    }

    /// Returns retained source segment primitive-family bucket count.
    pub fn source_segment_kind_bucket_count(&self) -> usize {
        self.facts().source_segment_kind_bucket_count()
    }

    /// Returns retained source segment references grouped by primitive family.
    pub fn source_segment_kind_ref_count(&self) -> usize {
        self.facts().source_segment_kind_ref_count()
    }

    /// Returns retained line source segment references.
    pub fn source_line_segment_ref_count(&self) -> usize {
        self.facts().source_line_segment_ref_count()
    }

    /// Returns retained arc source segment references.
    pub fn source_arc_segment_ref_count(&self) -> usize {
        self.facts().source_arc_segment_ref_count()
    }

    /// Returns the largest retained source segment primitive-family bucket size.
    pub fn source_segment_kind_max_bucket_size(&self) -> usize {
        self.facts().source_segment_kind_max_bucket_size()
    }

    /// Returns source endpoints retained in exact structural endpoint buckets.
    pub fn source_endpoint_count(&self) -> usize {
        self.facts().source_endpoint_count()
    }

    /// Returns exact structural source endpoint bucket count.
    pub fn source_endpoint_bucket_count(&self) -> usize {
        self.facts().source_endpoint_bucket_count()
    }

    /// Returns source endpoint buckets containing one endpoint.
    pub fn source_endpoint_singleton_bucket_count(&self) -> usize {
        self.facts().source_endpoint_singleton_bucket_count()
    }

    /// Returns the largest exact structural source endpoint bucket size.
    pub fn source_endpoint_max_bucket_size(&self) -> usize {
        self.facts().source_endpoint_max_bucket_size()
    }

    /// Returns source segment pairs scheduled before retained split predicates run.
    pub fn split_schedule_candidate_pair_count(&self) -> usize {
        self.facts().split_schedule_candidate_pair_count()
    }

    /// Returns scheduled source segment pairs pruned by retained AABB evidence.
    pub fn split_schedule_decided_disjoint_pair_count(&self) -> usize {
        self.facts().split_schedule_decided_disjoint_pair_count()
    }

    /// Returns scheduled source segment pairs that require split predicate evaluation.
    pub fn split_schedule_predicate_candidate_pair_count(&self) -> usize {
        self.facts().split_schedule_predicate_candidate_pair_count()
    }

    /// Returns scheduled source segment pairs whose AABB pruning status stayed undecided.
    pub fn split_schedule_undecided_aabb_pair_count(&self) -> usize {
        self.facts().split_schedule_undecided_aabb_pair_count()
    }

    /// Returns retained split schedule AABB-status bucket count.
    pub fn split_schedule_bucket_count(&self) -> usize {
        self.facts().split_schedule_bucket_count()
    }

    /// Returns retained split schedule source-pair references grouped by AABB status.
    pub fn split_schedule_candidate_ref_count(&self) -> usize {
        self.facts().split_schedule_candidate_ref_count()
    }

    /// Returns the largest retained split schedule AABB-status bucket size.
    pub fn split_schedule_max_bucket_size(&self) -> usize {
        self.facts().split_schedule_max_bucket_size()
    }

    /// Returns the exact predicate family used by retained split evaluation.
    pub fn split_predicate_path(&self) -> Option<RegionLineSegmentSplitPredicatePath2> {
        self.facts().split_predicate_path()
    }

    /// Returns source segment pairs considered by retained split evaluation.
    pub fn split_candidate_pair_count(&self) -> Option<usize> {
        self.facts().split_candidate_pair_count()
    }

    /// Returns source segment pairs skipped by certified AABB disjointness.
    pub fn split_skipped_aabb_pair_count(&self) -> Option<usize> {
        self.facts().split_skipped_aabb_pair_count()
    }

    /// Returns source segment pairs tested by exact split predicates.
    pub fn split_tested_pair_count(&self) -> Option<usize> {
        self.facts().split_tested_pair_count()
    }

    /// Returns exact point-intersection event count found during splitting.
    pub fn split_intersection_event_count(&self) -> Option<usize> {
        self.facts().split_intersection_event_count()
    }

    /// Returns source-pair relations classified as point intersections.
    pub fn split_point_relation_count(&self) -> Option<usize> {
        self.facts().split_point_relation_count()
    }

    /// Returns source-pair relations classified as overlaps.
    pub fn split_overlap_relation_count(&self) -> Option<usize> {
        self.facts().split_overlap_relation_count()
    }

    /// Returns source-pair relations that remained uncertain.
    pub fn split_uncertain_relation_count(&self) -> Option<usize> {
        self.facts().split_uncertain_relation_count()
    }

    /// Returns exact intersection points retained by split evaluation.
    pub fn split_intersection_points(&self) -> Option<&[Point2]> {
        self.facts().split_intersection_points()
    }

    /// Returns exact per-event source and parameter evidence retained by split evaluation.
    pub fn split_intersection_evidence(
        &self,
    ) -> Option<&[RegionLineSegmentSplitIntersectionEvidence2]> {
        self.facts().split_intersection_evidence()
    }

    /// Returns retained split-stage relation bucket count.
    pub fn split_relation_bucket_count(&self) -> Option<usize> {
        self.facts().split_relation_bucket_count()
    }

    /// Returns retained split-stage classified relation references.
    pub fn split_relation_ref_count(&self) -> Option<usize> {
        self.facts().split_relation_ref_count()
    }

    /// Returns the largest retained split-stage relation bucket size.
    pub fn split_relation_max_bucket_size(&self) -> Option<usize> {
        self.facts().split_relation_max_bucket_size()
    }

    /// Returns retained exact split-intersection point bucket count.
    pub fn split_intersection_bucket_count(&self) -> Option<usize> {
        self.facts().split_intersection_bucket_count()
    }

    /// Returns retained split-intersection buckets containing one event.
    pub fn split_intersection_singleton_bucket_count(&self) -> Option<usize> {
        self.facts().split_intersection_singleton_bucket_count()
    }

    /// Returns the largest retained split-intersection point bucket size.
    pub fn split_intersection_max_bucket_size(&self) -> Option<usize> {
        self.facts().split_intersection_max_bucket_size()
    }

    /// Returns retained source-parameter references for split intersections.
    pub fn split_intersection_source_parameter_ref_count(&self) -> Option<usize> {
        self.facts().split_intersection_source_parameter_ref_count()
    }

    /// Returns the first source segment in a split-stage blocker, when known.
    pub fn split_blocker_first_source_segment_index(&self) -> Option<usize> {
        self.facts().split_blocker_first_source_segment_index()
    }

    /// Returns the primitive family of the first source segment in a split-stage blocker.
    pub fn split_blocker_first_source_segment_kind(&self) -> Option<SegmentKind> {
        self.facts().split_blocker_first_source_segment_kind()
    }

    /// Returns the exact start point of the first source segment in a split-stage blocker.
    pub fn split_blocker_first_source_start_point(&self) -> Option<&Point2> {
        self.facts().split_blocker_first_source_start_point()
    }

    /// Returns the exact end point of the first source segment in a split-stage blocker.
    pub fn split_blocker_first_source_end_point(&self) -> Option<&Point2> {
        self.facts().split_blocker_first_source_end_point()
    }

    /// Returns the second source segment in a split-stage blocker, when known.
    pub fn split_blocker_second_source_segment_index(&self) -> Option<usize> {
        self.facts().split_blocker_second_source_segment_index()
    }

    /// Returns the primitive family of the second source segment in a split-stage blocker.
    pub fn split_blocker_second_source_segment_kind(&self) -> Option<SegmentKind> {
        self.facts().split_blocker_second_source_segment_kind()
    }

    /// Returns the exact start point of the second source segment in a split-stage blocker.
    pub fn split_blocker_second_source_start_point(&self) -> Option<&Point2> {
        self.facts().split_blocker_second_source_start_point()
    }

    /// Returns the exact end point of the second source segment in a split-stage blocker.
    pub fn split_blocker_second_source_end_point(&self) -> Option<&Point2> {
        self.facts().split_blocker_second_source_end_point()
    }

    /// Returns arranged output segment count when retained splitting completed.
    pub fn split_output_segment_count(&self) -> Option<usize> {
        self.facts().split_output_segment_count()
    }

    /// Returns the exact predicate family used by retained endpoint-graph validation.
    pub fn endpoint_graph_predicate_path(
        &self,
    ) -> Option<RegionLineSegmentEndpointGraphPredicatePath2> {
        self.facts().endpoint_graph_predicate_path()
    }

    /// Returns arranged endpoint count validated by retained endpoint-graph evidence.
    pub fn endpoint_graph_endpoint_count(&self) -> Option<usize> {
        self.facts().endpoint_graph_endpoint_count()
    }

    /// Returns exact structural endpoint bucket count.
    pub fn endpoint_graph_structural_bucket_count(&self) -> Option<usize> {
        self.facts().endpoint_graph_structural_bucket_count()
    }

    /// Returns structural endpoint singleton bucket count.
    pub fn endpoint_graph_structural_singleton_bucket_count(&self) -> Option<usize> {
        self.facts()
            .endpoint_graph_structural_singleton_bucket_count()
    }

    /// Returns the largest retained structural endpoint bucket size.
    pub fn endpoint_graph_max_structural_bucket_size(&self) -> Option<usize> {
        self.facts().endpoint_graph_max_structural_bucket_size()
    }

    /// Returns retained arranged endpoint references in structural buckets.
    pub fn arranged_endpoint_ref_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_ref_count()
    }

    /// Returns retained exact structural arranged endpoint bucket count.
    pub fn arranged_endpoint_bucket_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_bucket_count()
    }

    /// Returns retained arranged endpoint buckets containing one endpoint.
    pub fn arranged_endpoint_singleton_bucket_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_singleton_bucket_count()
    }

    /// Returns the largest retained arranged endpoint structural bucket size.
    pub fn arranged_endpoint_max_bucket_size(&self) -> Option<usize> {
        self.facts().arranged_endpoint_max_bucket_size()
    }

    /// Returns retained arranged endpoint side bucket count.
    pub fn arranged_endpoint_side_bucket_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_side_bucket_count()
    }

    /// Returns retained arranged endpoint references grouped by side.
    pub fn arranged_endpoint_side_ref_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_side_ref_count()
    }

    /// Returns retained arranged start endpoint references.
    pub fn arranged_endpoint_start_ref_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_start_ref_count()
    }

    /// Returns retained arranged end endpoint references.
    pub fn arranged_endpoint_end_ref_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_end_ref_count()
    }

    /// Returns the largest retained arranged endpoint side bucket size.
    pub fn arranged_endpoint_side_max_bucket_size(&self) -> Option<usize> {
        self.facts().arranged_endpoint_side_max_bucket_size()
    }

    /// Returns retained arranged fragment endpoint records.
    pub fn arranged_endpoint_point_fragment_ref_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_point_fragment_ref_count()
    }

    /// Returns retained arranged endpoint point references.
    pub fn arranged_endpoint_point_ref_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_point_ref_count()
    }

    /// Returns retained arranged endpoint degree bucket count.
    pub fn arranged_endpoint_degree_bucket_count(&self) -> Option<usize> {
        self.facts().arranged_endpoint_degree_bucket_count()
    }

    /// Returns structural endpoint buckets classified by retained degree.
    pub fn arranged_endpoint_degree_structural_bucket_ref_count(&self) -> Option<usize> {
        self.facts()
            .arranged_endpoint_degree_structural_bucket_ref_count()
    }

    /// Returns structural endpoint buckets classified as dangling.
    pub fn arranged_endpoint_dangling_structural_bucket_count(&self) -> Option<usize> {
        self.facts()
            .arranged_endpoint_dangling_structural_bucket_count()
    }

    /// Returns structural endpoint buckets classified as chain continuations.
    pub fn arranged_endpoint_chain_structural_bucket_count(&self) -> Option<usize> {
        self.facts()
            .arranged_endpoint_chain_structural_bucket_count()
    }

    /// Returns structural endpoint buckets classified as branches.
    pub fn arranged_endpoint_branch_structural_bucket_count(&self) -> Option<usize> {
        self.facts()
            .arranged_endpoint_branch_structural_bucket_count()
    }

    /// Returns the largest retained arranged endpoint degree bucket size.
    pub fn arranged_endpoint_degree_max_bucket_size(&self) -> Option<usize> {
        self.facts().arranged_endpoint_degree_max_bucket_size()
    }

    /// Returns dangling endpoint count found during endpoint-graph validation.
    pub fn endpoint_graph_dangling_endpoint_count(&self) -> Option<usize> {
        self.facts().endpoint_graph_dangling_endpoint_count()
    }

    /// Returns branch endpoint count found during endpoint-graph validation.
    pub fn endpoint_graph_branch_endpoint_count(&self) -> Option<usize> {
        self.facts().endpoint_graph_branch_endpoint_count()
    }

    /// Returns the blocker arranged segment index from endpoint validation, when blocked.
    pub fn endpoint_graph_blocker_arranged_segment_index(&self) -> Option<usize> {
        self.facts().endpoint_graph_blocker_arranged_segment_index()
    }

    /// Returns the blocker endpoint from endpoint validation, when blocked.
    pub fn endpoint_graph_blocker_endpoint(&self) -> Option<RegionLineSegmentArrangedEndpoint2> {
        self.facts().endpoint_graph_blocker_endpoint()
    }

    /// Returns the exact blocker point from endpoint validation, when blocked.
    pub fn endpoint_graph_blocker_point(&self) -> Option<&Point2> {
        self.facts().endpoint_graph_blocker_point()
    }

    /// Returns the exact predicate family used by retained ring traversal.
    pub fn ring_assembly_predicate_path(
        &self,
    ) -> Option<RegionLineSegmentRingAssemblyPredicatePath2> {
        self.facts().ring_assembly_predicate_path()
    }

    /// Returns endpoint pair comparisons attempted during retained ring traversal.
    pub fn attempted_endpoint_connection_count(&self) -> Option<usize> {
        self.facts().attempted_endpoint_connection_count()
    }

    /// Returns endpoint pair comparisons certified as equal during ring traversal.
    pub fn exact_endpoint_connection_count(&self) -> Option<usize> {
        self.facts().exact_endpoint_connection_count()
    }

    /// Returns endpoint pair comparisons certified as disconnected during ring traversal.
    pub fn disconnected_endpoint_connection_count(&self) -> Option<usize> {
        self.facts().disconnected_endpoint_connection_count()
    }

    /// Returns endpoint pair comparisons unresolved during ring traversal.
    pub fn unresolved_endpoint_connection_count(&self) -> Option<usize> {
        self.facts().unresolved_endpoint_connection_count()
    }

    /// Returns source segments reversed while materializing retained ring traversal.
    pub fn reversed_source_segment_count(&self) -> Option<usize> {
        self.facts().reversed_source_segment_count()
    }

    /// Returns per-arranged-fragment source provenance retained after exact splitting.
    pub fn arranged_source_evidence(&self) -> Option<&[RegionLineSegmentArrangedSourceEvidence2]> {
        self.facts().arranged_source_evidence()
    }

    /// Returns the retained arranged-source provenance record count.
    pub fn arranged_source_evidence_count(&self) -> Option<usize> {
        self.facts().arranged_source_evidence_count()
    }

    /// Returns per-output segment source provenance retained by ring traversal.
    pub fn source_evidence(&self) -> Option<&[RegionLineSegmentRingSourceEvidence2]> {
        self.facts().source_evidence()
    }

    /// Returns the retained output-source provenance record count.
    pub fn source_evidence_count(&self) -> Option<usize> {
        self.facts().source_evidence_count()
    }

    /// Returns arranged fragment count retained after exact splitting, when available.
    pub fn arranged_segment_count(&self) -> Option<usize> {
        self.facts().arranged_segment_count()
    }

    /// Returns arranged fragment primitive-family counts retained after exact splitting.
    pub fn arranged_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.facts().arranged_segment_kind_counts()
    }

    /// Returns retained arranged fragment primitive-family bucket count.
    pub fn arranged_fragment_kind_bucket_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_kind_bucket_count()
    }

    /// Returns retained arranged fragment references grouped by primitive family.
    pub fn arranged_fragment_kind_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_kind_ref_count()
    }

    /// Returns retained line arranged fragment references.
    pub fn arranged_line_fragment_ref_count(&self) -> Option<usize> {
        self.facts().arranged_line_fragment_ref_count()
    }

    /// Returns retained arc arranged fragment references.
    pub fn arranged_arc_fragment_ref_count(&self) -> Option<usize> {
        self.facts().arranged_arc_fragment_ref_count()
    }

    /// Returns the largest retained arranged fragment primitive-family bucket size.
    pub fn arranged_fragment_kind_max_bucket_size(&self) -> Option<usize> {
        self.facts().arranged_fragment_kind_max_bucket_size()
    }

    /// Returns retained arranged fragment topology-status bucket count.
    pub fn arranged_fragment_status_bucket_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_status_bucket_count()
    }

    /// Returns retained arranged fragment source references grouped by topology status.
    pub fn arranged_fragment_status_source_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_status_source_ref_count()
    }

    /// Returns retained native-exact arranged fragment source references.
    pub fn arranged_fragment_native_exact_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_native_exact_ref_count()
    }

    /// Returns retained certified-approximation arranged fragment source references.
    pub fn arranged_fragment_certified_approximation_ref_count(&self) -> Option<usize> {
        self.facts()
            .arranged_fragment_certified_approximation_ref_count()
    }

    /// Returns retained display/export-only arranged fragment source references.
    pub fn arranged_fragment_display_or_export_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_display_or_export_ref_count()
    }

    /// Returns retained lossy-import arranged fragment source references.
    pub fn arranged_fragment_imported_lossy_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_imported_lossy_ref_count()
    }

    /// Returns retained unsupported arranged fragment source references.
    pub fn arranged_fragment_unsupported_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_unsupported_ref_count()
    }

    /// Returns retained unresolved arranged fragment source references.
    pub fn arranged_fragment_unresolved_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_unresolved_ref_count()
    }

    /// Returns the largest retained arranged fragment topology-status bucket size.
    pub fn arranged_fragment_status_max_bucket_size(&self) -> Option<usize> {
        self.facts().arranged_fragment_status_max_bucket_size()
    }

    /// Returns retained arranged fragment source range references.
    pub fn arranged_fragment_source_range_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_source_range_ref_count()
    }

    /// Returns retained arranged fragment source ranges covering complete source segments.
    pub fn arranged_fragment_full_source_range_ref_count(&self) -> Option<usize> {
        self.facts().arranged_fragment_full_source_range_ref_count()
    }

    /// Returns retained arranged fragment source ranges covering proper source subranges.
    pub fn arranged_fragment_partial_source_range_ref_count(&self) -> Option<usize> {
        self.facts()
            .arranged_fragment_partial_source_range_ref_count()
    }

    /// Returns retained output ring provenance bucket count.
    pub fn output_ring_bucket_count(&self) -> Option<usize> {
        self.facts().output_ring_bucket_count()
    }

    /// Returns retained output ring segment references.
    pub fn output_ring_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_ring_segment_ref_count()
    }

    /// Returns the largest retained output ring segment count.
    pub fn output_ring_max_segment_count(&self) -> Option<usize> {
        self.facts().output_ring_max_segment_count()
    }

    /// Returns retained output segment primitive-family bucket count.
    pub fn output_segment_kind_bucket_count(&self) -> Option<usize> {
        self.facts().output_segment_kind_bucket_count()
    }

    /// Returns retained output segment references grouped by primitive family.
    pub fn output_segment_kind_ref_count(&self) -> Option<usize> {
        self.facts().output_segment_kind_ref_count()
    }

    /// Returns retained line output segment references.
    pub fn output_line_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_line_segment_ref_count()
    }

    /// Returns retained arc output segment references.
    pub fn output_arc_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_arc_segment_ref_count()
    }

    /// Returns the largest retained output segment primitive-family bucket size.
    pub fn output_segment_kind_max_bucket_size(&self) -> Option<usize> {
        self.facts().output_segment_kind_max_bucket_size()
    }

    /// Returns retained source-segment bucket count for output segments.
    pub fn output_segment_source_bucket_count(&self) -> Option<usize> {
        self.facts().output_segment_source_bucket_count()
    }

    /// Returns retained output segment references grouped by source segment.
    pub fn output_segment_source_ref_count(&self) -> Option<usize> {
        self.facts().output_segment_source_ref_count()
    }

    /// Returns the largest retained source-segment output bucket size.
    pub fn output_segment_source_max_bucket_size(&self) -> Option<usize> {
        self.facts().output_segment_source_max_bucket_size()
    }

    /// Returns retained output segment source-range references.
    pub fn output_segment_source_range_ref_count(&self) -> Option<usize> {
        self.facts().output_segment_source_range_ref_count()
    }

    /// Returns retained output segments covering a complete source range.
    pub fn output_full_source_range_ref_count(&self) -> Option<usize> {
        self.facts().output_full_source_range_ref_count()
    }

    /// Returns retained output segments covering a proper source subrange.
    pub fn output_partial_source_range_ref_count(&self) -> Option<usize> {
        self.facts().output_partial_source_range_ref_count()
    }

    /// Returns retained output segment endpoint records.
    pub fn output_segment_endpoint_record_count(&self) -> Option<usize> {
        self.facts().output_segment_endpoint_record_count()
    }

    /// Returns retained exact output endpoint references.
    pub fn output_endpoint_ref_count(&self) -> Option<usize> {
        self.facts().output_endpoint_ref_count()
    }

    /// Returns output rings with retained continuity evidence.
    pub fn output_ring_continuity_ring_ref_count(&self) -> Option<usize> {
        self.facts().output_ring_continuity_ring_ref_count()
    }

    /// Returns retained output segment-to-next-segment continuity references.
    pub fn output_ring_continuity_connection_ref_count(&self) -> Option<usize> {
        self.facts().output_ring_continuity_connection_ref_count()
    }

    /// Returns the largest retained continuity connection count for one output ring.
    pub fn output_ring_continuity_max_connection_count(&self) -> Option<usize> {
        self.facts().output_ring_continuity_max_connection_count()
    }

    /// Returns retained topology-status bucket count for output segments.
    pub fn output_segment_status_bucket_count(&self) -> Option<usize> {
        self.facts().output_segment_status_bucket_count()
    }

    /// Returns retained output segment references grouped by topology status.
    pub fn output_segment_status_ref_count(&self) -> Option<usize> {
        self.facts().output_segment_status_ref_count()
    }

    /// Returns retained native-exact output segment references.
    pub fn output_native_exact_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_native_exact_segment_ref_count()
    }

    /// Returns retained certified-approximation output segment references.
    pub fn output_certified_approximation_segment_ref_count(&self) -> Option<usize> {
        self.facts()
            .output_certified_approximation_segment_ref_count()
    }

    /// Returns retained display/export-only output segment references.
    pub fn output_display_or_export_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_display_or_export_segment_ref_count()
    }

    /// Returns retained lossy-import output segment references.
    pub fn output_imported_lossy_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_imported_lossy_segment_ref_count()
    }

    /// Returns retained unsupported output segment references.
    pub fn output_unsupported_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_unsupported_segment_ref_count()
    }

    /// Returns retained unresolved output segment references.
    pub fn output_unresolved_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_unresolved_segment_ref_count()
    }

    /// Returns the largest retained topology-status output bucket size.
    pub fn output_segment_status_max_bucket_size(&self) -> Option<usize> {
        self.facts().output_segment_status_max_bucket_size()
    }

    /// Returns retained traversal-direction bucket count for output segments.
    pub fn output_segment_direction_bucket_count(&self) -> Option<usize> {
        self.facts().output_segment_direction_bucket_count()
    }

    /// Returns retained output segment references grouped by traversal direction.
    pub fn output_segment_direction_ref_count(&self) -> Option<usize> {
        self.facts().output_segment_direction_ref_count()
    }

    /// Returns retained forward output segment references.
    pub fn output_forward_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_forward_segment_ref_count()
    }

    /// Returns retained reversed output segment references.
    pub fn output_reversed_segment_ref_count(&self) -> Option<usize> {
        self.facts().output_reversed_segment_ref_count()
    }

    /// Returns the largest retained traversal-direction output bucket size.
    pub fn output_segment_direction_max_bucket_size(&self) -> Option<usize> {
        self.facts().output_segment_direction_max_bucket_size()
    }

    /// Returns delegated boundary-contour role assignment evidence, when output reached it.
    pub fn boundary_build_evidence(&self) -> Option<&RegionBoundaryContourBuildEvidence2> {
        self.facts().boundary_build_evidence()
    }

    /// Returns final boundary-role assignment stage, if reached.
    pub fn boundary_build_stage(&self) -> Option<RegionBoundaryContourBuildStage2> {
        self.facts().boundary_build_stage()
    }

    /// Returns final boundary-role assignment predicate path, if reached.
    pub fn boundary_build_predicate_path(
        &self,
    ) -> Option<RegionBoundaryContourBuildPredicatePath2> {
        self.facts().boundary_build_predicate_path()
    }

    /// Returns final boundary-role assignment retained status, if reached.
    pub fn boundary_build_status(&self) -> Option<RetainedTopologyStatus> {
        self.facts().boundary_build_status()
    }

    /// Returns final boundary-role assignment blocker, if present.
    pub fn boundary_build_blocker(&self) -> Option<UncertaintyReason> {
        self.facts().boundary_build_blocker()
    }

    /// Returns source contour count from final boundary-role assignment, if reached.
    pub fn boundary_build_source_contour_count(&self) -> Option<usize> {
        self.facts().boundary_build_source_contour_count()
    }

    /// Returns source boundary segment count from final boundary-role assignment, if reached.
    pub fn boundary_build_source_segment_count(&self) -> Option<usize> {
        self.facts().boundary_build_source_segment_count()
    }

    /// Returns contour-pair validation schedule size from final role assignment, if reached.
    pub fn boundary_build_validation_candidate_pair_count(&self) -> Option<usize> {
        self.facts()
            .boundary_build_validation_candidate_pair_count()
    }

    /// Returns contour-pair validation test count from final role assignment, if reached.
    pub fn boundary_build_validation_tested_pair_count(&self) -> Option<usize> {
        self.facts().boundary_build_validation_tested_pair_count()
    }

    /// Returns exact validation intersection event count from final role assignment, if reached.
    pub fn boundary_build_validation_intersection_event_count(&self) -> Option<usize> {
        self.facts()
            .boundary_build_validation_intersection_event_count()
    }

    /// Returns containment classification count from final role assignment, if reached.
    pub fn boundary_build_nesting_classification_count(&self) -> Option<usize> {
        self.facts().boundary_build_nesting_classification_count()
    }

    /// Returns first blocking contour index from final role assignment, if present.
    pub fn boundary_build_blocker_first_contour_index(&self) -> Option<usize> {
        self.facts().boundary_build_blocker_first_contour_index()
    }

    /// Returns second blocking contour index from final role assignment, if present.
    pub fn boundary_build_blocker_second_contour_index(&self) -> Option<usize> {
        self.facts().boundary_build_blocker_second_contour_index()
    }

    /// Returns retained final boundary role bucket count.
    pub fn boundary_output_role_bucket_count(&self) -> Option<usize> {
        self.facts().boundary_output_role_bucket_count()
    }

    /// Returns retained final boundary output contour references grouped by role.
    pub fn boundary_output_role_contour_count(&self) -> Option<usize> {
        self.facts().boundary_output_role_contour_count()
    }

    /// Returns retained final boundary output segment references grouped by role.
    pub fn boundary_output_role_segment_count(&self) -> Option<usize> {
        self.facts().boundary_output_role_segment_count()
    }

    /// Returns the largest retained output segment count for one boundary role.
    pub fn boundary_output_role_max_segment_count(&self) -> Option<usize> {
        self.facts().boundary_output_role_max_segment_count()
    }

    /// Returns output boundary primitive-family counts after role assignment.
    pub fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.facts().output_segment_kind_counts()
    }

    /// Returns material contour count after output role assignment.
    pub fn material_contour_count(&self) -> Option<usize> {
        self.facts().material_contour_count()
    }

    /// Returns hole contour count after output role assignment.
    pub fn hole_contour_count(&self) -> Option<usize> {
        self.facts().hole_contour_count()
    }

    /// Returns material boundary segment count after output role assignment.
    pub fn material_segment_count(&self) -> Option<usize> {
        self.facts().material_segment_count()
    }

    /// Returns hole boundary segment count after output role assignment.
    pub fn hole_segment_count(&self) -> Option<usize> {
        self.facts().hole_segment_count()
    }

    /// Returns retained output role evidence count when role assignment was reached.
    pub fn role_evidence_count(&self) -> Option<usize> {
        self.facts().role_evidence_count()
    }

    /// Returns retained output role topology-status bucket count.
    pub fn role_status_bucket_count(&self) -> Option<usize> {
        self.facts().role_status_bucket_count()
    }

    /// Returns retained output role assignment references grouped by topology status.
    pub fn role_status_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_status_assignment_ref_count()
    }

    /// Returns retained native-exact output role assignment references.
    pub fn role_native_exact_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_native_exact_assignment_ref_count()
    }

    /// Returns retained certified-approximation output role assignment references.
    pub fn role_certified_approximation_assignment_ref_count(&self) -> Option<usize> {
        self.facts()
            .role_certified_approximation_assignment_ref_count()
    }

    /// Returns retained display/export-only output role assignment references.
    pub fn role_display_or_export_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_display_or_export_assignment_ref_count()
    }

    /// Returns retained lossy-import output role assignment references.
    pub fn role_imported_lossy_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_imported_lossy_assignment_ref_count()
    }

    /// Returns retained unsupported output role assignment references.
    pub fn role_unsupported_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_unsupported_assignment_ref_count()
    }

    /// Returns retained unresolved output role assignment references.
    pub fn role_unresolved_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_unresolved_assignment_ref_count()
    }

    /// Returns the largest retained output role topology-status bucket size.
    pub fn role_status_max_bucket_size(&self) -> Option<usize> {
        self.facts().role_status_max_bucket_size()
    }

    /// Returns retained output role source-contour bucket count.
    pub fn role_source_contour_bucket_count(&self) -> Option<usize> {
        self.facts().role_source_contour_bucket_count()
    }

    /// Returns retained output role assignment references grouped by source contour.
    pub fn role_source_contour_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_source_contour_assignment_ref_count()
    }

    /// Returns the largest retained source-contour output role bucket size.
    pub fn role_source_contour_max_bucket_size(&self) -> Option<usize> {
        self.facts().role_source_contour_max_bucket_size()
    }

    /// Returns retained output role nesting-depth bucket count.
    pub fn role_nesting_depth_bucket_count(&self) -> Option<usize> {
        self.facts().role_nesting_depth_bucket_count()
    }

    /// Returns retained output role assignment references grouped by nesting depth.
    pub fn role_nesting_depth_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_nesting_depth_assignment_ref_count()
    }

    /// Returns the largest retained nesting-depth output role bucket size.
    pub fn role_nesting_depth_max_bucket_size(&self) -> Option<usize> {
        self.facts().role_nesting_depth_max_bucket_size()
    }

    /// Returns retained output role containing-contour bucket count.
    pub fn role_containment_bucket_count(&self) -> Option<usize> {
        self.facts().role_containment_bucket_count()
    }

    /// Returns retained output role containment references.
    pub fn role_containment_ref_count(&self) -> Option<usize> {
        self.facts().role_containment_ref_count()
    }

    /// Returns retained output role assignments with no containing contour.
    pub fn role_uncontained_assignment_ref_count(&self) -> Option<usize> {
        self.facts().role_uncontained_assignment_ref_count()
    }

    /// Returns the largest retained containing-contour bucket size.
    pub fn role_containment_max_bucket_size(&self) -> Option<usize> {
        self.facts().role_containment_max_bucket_size()
    }

    /// Returns retained output role evidence when role assignment was reached.
    pub fn role_evidence(&self) -> Option<&[RegionBoundaryContourRoleEvidence2]> {
        self.facts().role_evidence()
    }

    /// Returns final semantic facts derived from the evaluation.
    pub fn summary(&self) -> &RegionArrangementSummary2 {
        &self.summary
    }

    /// Returns whether final output evaluation facts were retained.
    pub fn evaluated_output(&self) -> bool {
        self.facts().evaluated_output()
    }

    /// Returns whether the retained evaluation materialized a region, when evaluated.
    pub fn materialized_region(&self) -> Option<bool> {
        self.facts().materialized_region()
    }

    /// Returns the final retained build stage, when evaluated.
    pub fn stage(&self) -> Option<RegionLineSegmentRegionBuildStage2> {
        self.facts().stage()
    }

    /// Returns the final retained topology status, when evaluated.
    pub fn status(&self) -> Option<RetainedTopologyStatus> {
        self.facts().status()
    }

    /// Returns the final retained blocker, when the evaluated arrangement blocked.
    pub fn blocker(&self) -> Option<UncertaintyReason> {
        self.facts().blocker()
    }

    /// Returns output ring count retained by ring assembly, when available.
    pub fn output_ring_count(&self) -> Option<usize> {
        self.facts().output_ring_count()
    }

    /// Returns output boundary segment count retained by ring assembly, when available.
    pub fn output_boundary_segment_count(&self) -> Option<usize> {
        self.facts().output_boundary_segment_count()
    }

    /// Returns output boundary primitive-family counts retained by ring assembly, when available.
    pub fn output_boundary_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.facts().output_boundary_segment_kind_counts()
    }

    /// Returns final output contour count retained after boundary role assignment.
    pub fn output_contour_count(&self) -> Option<usize> {
        self.facts().output_contour_count()
    }

    /// Returns final output boundary segment count retained after boundary role assignment.
    pub fn output_segment_count(&self) -> Option<usize> {
        self.facts().output_segment_count()
    }

    /// Returns the materialized region, if the arrangement succeeded.
    pub fn region(&self) -> Option<&LineArcRegion2> {
        self.region.as_ref()
    }

    /// Returns the materialized region as the canonical convenience classification.
    ///
    /// Callers keep the arrangement and derived evidence available while
    /// branching on a decided region or explicit blocker.
    pub fn region_classification(&self) -> Classification<&LineArcRegion2> {
        match self.region() {
            Some(region) => Classification::Decided(region),
            None => {
                Classification::Uncertain(self.blocker().unwrap_or(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Consumes this result and returns the materialized region as a classification.
    ///
    /// This preserves the explicit blocker used by the retained evaluation when
    /// no region was materialized.
    pub fn into_region_classification(self) -> Classification<LineArcRegion2> {
        let blocker = self.blocker().unwrap_or(UncertaintyReason::Unsupported);
        match self.into_region() {
            Some(region) => Classification::Decided(region),
            None => Classification::Uncertain(blocker),
        }
    }

    /// Consumes this result into the native region and shared arrangement facts.
    pub(crate) fn into_region_with_facts(
        self,
    ) -> (
        Option<LineArcRegion2>,
        Rc<ExactCurveWorkspace2>,
        RegionArrangementSummary2,
    ) {
        let Self {
            workspace,
            summary,
            region,
        } = self;
        (region, workspace, summary)
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<LineArcRegion2> {
        self.region
    }
}

impl RegionBoundaryContourRoleEvidence2 {
    /// Returns the source contour index assigned by this evidence.
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

impl RegionBoundaryContourBuildEvidence2 {
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

    /// Returns per-contour exact role evidence.
    pub fn role_evidence(&self) -> &[RegionBoundaryContourRoleEvidence2] {
        &self.role_evidence
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
    pub const fn region(&self) -> Option<&LineArcRegion2> {
        self.region.as_ref()
    }

    /// Consumes this result and returns the materialized region, if any.
    pub fn into_region(self) -> Option<LineArcRegion2> {
        self.region
    }

    /// Consumes this result and returns the retained region-construction evidence.
    pub fn into_evidence(self) -> RegionBoundaryContourBuildEvidence2 {
        self.evidence
    }

    /// Consumes this result and returns the materialized region with its evidence.
    pub fn into_parts(self) -> (Option<LineArcRegion2>, RegionBoundaryContourBuildEvidence2) {
        (self.region, self.evidence)
    }

    /// Returns the retained region-construction evidence.
    pub const fn evidence(&self) -> &RegionBoundaryContourBuildEvidence2 {
        &self.evidence
    }

    /// Returns the materialized region as a classification while retaining this result.
    pub fn region_classification(&self) -> Classification<&LineArcRegion2> {
        match self.region() {
            Some(region) => Classification::Decided(region),
            None => {
                Classification::Uncertain(self.blocker().unwrap_or(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Consumes this result and returns the materialized region as a classification.
    pub fn into_region_classification(self) -> Classification<LineArcRegion2> {
        let blocker = self.blocker().unwrap_or(UncertaintyReason::Unsupported);
        match self.into_region() {
            Some(region) => Classification::Decided(region),
            None => Classification::Uncertain(blocker),
        }
    }

    /// Returns the furthest exact region-construction stage reached.
    pub const fn stage(&self) -> RegionBoundaryContourBuildStage2 {
        self.evidence.stage()
    }

    /// Returns the exact predicate path used for boundary validation and nesting.
    pub const fn predicate_path(&self) -> RegionBoundaryContourBuildPredicatePath2 {
        self.evidence.predicate_path()
    }

    /// Returns the number of source boundary contours considered.
    pub const fn source_contour_count(&self) -> usize {
        self.evidence.source_contour_count()
    }

    /// Returns the total number of source contour segments considered.
    pub const fn source_segment_count(&self) -> usize {
        self.evidence.source_segment_count()
    }

    /// Returns the number of contour pairs scheduled for intersection validation.
    pub const fn validation_candidate_pair_count(&self) -> usize {
        self.evidence.validation_candidate_pair_count()
    }

    /// Returns the number of contour pairs tested before success or a blocker.
    pub const fn validation_tested_pair_count(&self) -> usize {
        self.evidence.validation_tested_pair_count()
    }

    /// Returns exact contour-intersection events found during nesting validation.
    pub const fn validation_intersection_event_count(&self) -> usize {
        self.evidence.validation_intersection_event_count()
    }

    /// Returns point-containment classifications used to assign nesting roles.
    pub const fn nesting_classification_count(&self) -> usize {
        self.evidence.nesting_classification_count()
    }

    /// Returns the first source contour index involved in a blocking relation.
    pub const fn blocker_first_contour_index(&self) -> Option<usize> {
        self.evidence.blocker_first_contour_index()
    }

    /// Returns the second source contour index involved in a blocking relation.
    pub const fn blocker_second_contour_index(&self) -> Option<usize> {
        self.evidence.blocker_second_contour_index()
    }

    /// Returns total output contour count when role assignment materialized.
    pub const fn output_contour_count(&self) -> Option<usize> {
        self.evidence.output_contour_count()
    }

    /// Returns total output boundary segment count when role assignment materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.evidence.output_segment_count()
    }

    /// Returns material contour count when role assignment materialized.
    pub const fn material_contour_count(&self) -> Option<usize> {
        self.evidence.material_contour_count()
    }

    /// Returns hole contour count when role assignment materialized.
    pub const fn hole_contour_count(&self) -> Option<usize> {
        self.evidence.hole_contour_count()
    }

    /// Returns material boundary segment count when role assignment materialized.
    pub const fn material_segment_count(&self) -> Option<usize> {
        self.evidence.material_segment_count()
    }

    /// Returns hole boundary segment count when role assignment materialized.
    pub const fn hole_segment_count(&self) -> Option<usize> {
        self.evidence.hole_segment_count()
    }

    /// Returns per-contour exact role evidence.
    pub fn role_evidence(&self) -> &[RegionBoundaryContourRoleEvidence2] {
        self.evidence.role_evidence()
    }

    /// Returns region construction status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.evidence.status()
    }

    /// Returns the exact blocker for non-materialized construction attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.evidence.blocker()
    }
}

impl RegionLineSegmentRingSourceEvidence2 {
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

impl RegionLineSegmentArrangedSourceEvidence2 {
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

impl RegionLineSegmentRegionBuildEvidence2 {
    /// Returns the furthest exact line-region construction stage reached.
    pub const fn stage(&self) -> RegionLineSegmentRegionBuildStage2 {
        self.stage
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

    /// Returns source/parameter evidence for retained point-intersection split events.
    pub fn split_intersection_evidence(&self) -> &[RegionLineSegmentSplitIntersectionEvidence2] {
        &self.split_intersection_evidence
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

    /// Returns per-arranged-fragment source provenance after exact splitting.
    pub fn arranged_source_evidence(&self) -> &[RegionLineSegmentArrangedSourceEvidence2] {
        &self.arranged_source_evidence
    }

    /// Returns per-output segment source provenance.
    pub fn source_evidence(&self) -> &[RegionLineSegmentRingSourceEvidence2] {
        &self.source_evidence
    }

    /// Returns delegated boundary-contour role assignment evidence, when reached.
    pub const fn boundary_build_evidence(&self) -> Option<&RegionBoundaryContourBuildEvidence2> {
        self.boundary_build_evidence.as_ref()
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

impl RegionLineSegmentSplitIntersectionEvidence2 {
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
    pub const fn region(&self) -> Option<&LineArcRegion2> {
        self.region.as_ref()
    }

    /// Returns the retained line-region construction evidence.
    pub const fn evidence(&self) -> &RegionLineSegmentRegionBuildEvidence2 {
        &self.evidence
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
struct LineSegmentEndpointGraphEvidenceParts {
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
struct LineSegmentRingAssemblyEvidenceParts {
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_evidence: Vec<RegionLineSegmentRingSourceEvidence2>,
}

#[derive(Clone, Debug, PartialEq)]
struct LineSegmentRingAssembly {
    rings: Vec<Vec<LineSeg2>>,
    counts: LineSegmentRingAssemblyCounts,
    reversed_source_segment_count: usize,
    source_evidence: Vec<RegionLineSegmentRingSourceEvidence2>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct LineSegmentSplitEvidenceParts {
    predicate_path: Option<RegionLineSegmentSplitPredicatePath2>,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    intersection_event_count: usize,
    point_relation_count: usize,
    overlap_relation_count: usize,
    uncertain_relation_count: usize,
    intersection_points: Vec<Point2>,
    intersection_evidence: Vec<RegionLineSegmentSplitIntersectionEvidence2>,
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
    evidence: LineSegmentSplitEvidenceParts,
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
    evidence: LineSegmentSplitEvidenceParts,
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
        LineSegmentEndpointGraphEvidenceParts,
        LineSegmentRingAssemblyCounts,
    ),
    (
        LineSegmentEndpointGraphEvidenceParts,
        LineSegmentRingAssemblyCounts,
        UncertaintyReason,
    ),
> {
    let endpoints = arranged_line_endpoints(segments);
    let mut graph = structural_endpoint_bucket_evidence(segments, &endpoints);
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
    graph: &mut LineSegmentEndpointGraphEvidenceParts,
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

fn structural_endpoint_bucket_evidence(
    segments: &[ArrangedLineSegment],
    endpoints: &[ArrangedLineEndpoint],
) -> LineSegmentEndpointGraphEvidenceParts {
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

    LineSegmentEndpointGraphEvidenceParts {
        endpoint_count: endpoints.len(),
        structural_bucket_count: buckets.len(),
        structural_singleton_bucket_count: buckets.iter().filter(|(_, count)| *count == 1).count(),
        max_structural_bucket_size: buckets.iter().map(|(_, count)| *count).max().unwrap_or(0),
        ..LineSegmentEndpointGraphEvidenceParts::default()
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
        LineSegmentEndpointGraphEvidenceParts,
        LineSegmentRingAssemblyCounts,
    ),
    (
        LineSegmentEndpointGraphEvidenceParts,
        LineSegmentRingAssemblyCounts,
        UncertaintyReason,
    ),
> {
    let endpoints = arranged_native_endpoints(segments);
    let mut graph = structural_native_endpoint_bucket_evidence(segments, &endpoints);
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

fn structural_native_endpoint_bucket_evidence(
    segments: &[ArrangedNativeSegment],
    endpoints: &[ArrangedLineEndpoint],
) -> LineSegmentEndpointGraphEvidenceParts {
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

    LineSegmentEndpointGraphEvidenceParts {
        endpoint_count: endpoints.len(),
        structural_bucket_count: buckets.len(),
        structural_singleton_bucket_count: buckets.iter().filter(|(_, count)| *count == 1).count(),
        max_structural_bucket_size: buckets.iter().map(|(_, count)| *count).max().unwrap_or(0),
        ..LineSegmentEndpointGraphEvidenceParts::default()
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
) -> CurveResult<Result<ArrangedLineSegments, (LineSegmentSplitEvidenceParts, UncertaintyReason)>> {
    let mut evidence = LineSegmentSplitEvidenceParts {
        predicate_path: Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredExactLineLine),
        candidate_pair_count: segments
            .len()
            .saturating_mul(segments.len().saturating_sub(1))
            / 2,
        ..LineSegmentSplitEvidenceParts::default()
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
                evidence.skipped_aabb_pair_count += 1;
                continue;
            }
            evidence.tested_pair_count += 1;
            match first.intersect_line(second, policy)? {
                LineLineIntersection::None => {}
                LineLineIntersection::Point {
                    point,
                    a_param,
                    b_param,
                    ..
                } => {
                    evidence.point_relation_count += 1;
                    evidence.intersection_event_count += 1;
                    evidence.intersection_points.push(point.clone());
                    evidence.intersection_evidence.push(
                        RegionLineSegmentSplitIntersectionEvidence2 {
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
                        },
                    );
                    if insert_line_split_marker(&mut markers[first_index], a_param, policy)
                        .is_none()
                        || insert_line_split_marker(&mut markers[second_index], b_param, policy)
                            .is_none()
                    {
                        set_split_blocker_pair(
                            &mut evidence,
                            first_index,
                            SegmentKind::Line,
                            first.start(),
                            first.end(),
                            second_index,
                            SegmentKind::Line,
                            second.start(),
                            second.end(),
                        );
                        return Ok(Err((evidence, UncertaintyReason::Ordering)));
                    }
                }
                LineLineIntersection::Overlap { .. } => {
                    evidence.overlap_relation_count += 1;
                    set_split_blocker_pair(
                        &mut evidence,
                        first_index,
                        SegmentKind::Line,
                        first.start(),
                        first.end(),
                        second_index,
                        SegmentKind::Line,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((evidence, UncertaintyReason::Boundary)));
                }
                LineLineIntersection::Uncertain { reason } => {
                    evidence.uncertain_relation_count += 1;
                    set_split_blocker_pair(
                        &mut evidence,
                        first_index,
                        SegmentKind::Line,
                        first.start(),
                        first.end(),
                        second_index,
                        SegmentKind::Line,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((evidence, reason)));
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
                Some(Ordering::Greater) => return Ok(Err((evidence, UncertaintyReason::Ordering))),
                None => return Ok(Err((evidence, UncertaintyReason::Ordering))),
            }
        }
    }

    evidence.output_segment_count = Some(arranged.len());
    Ok(Ok(ArrangedLineSegments {
        segments: arranged,
        evidence,
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
    evidence: &mut LineSegmentSplitEvidenceParts,
    first_source_segment_index: usize,
    first_source_segment_kind: SegmentKind,
    first_source_start_point: &Point2,
    first_source_end_point: &Point2,
    second_source_segment_index: usize,
    second_source_segment_kind: SegmentKind,
    second_source_start_point: &Point2,
    second_source_end_point: &Point2,
) {
    if evidence.blocker_first_source_segment_index.is_none() {
        evidence.blocker_first_source_segment_index = Some(first_source_segment_index);
        evidence.blocker_first_source_segment_kind = Some(first_source_segment_kind);
        evidence.blocker_first_source_start_point = Some(first_source_start_point.clone());
        evidence.blocker_first_source_end_point = Some(first_source_end_point.clone());
        evidence.blocker_second_source_segment_index = Some(second_source_segment_index);
        evidence.blocker_second_source_segment_kind = Some(second_source_segment_kind);
        evidence.blocker_second_source_start_point = Some(second_source_start_point.clone());
        evidence.blocker_second_source_end_point = Some(second_source_end_point.clone());
    }
}

fn arrange_native_segments_at_point_intersections(
    segments: &[Segment2],
    policy: &CurvePolicy,
) -> CurveResult<Result<ArrangedNativeSegments, (LineSegmentSplitEvidenceParts, UncertaintyReason)>>
{
    let mut evidence = LineSegmentSplitEvidenceParts {
        predicate_path: Some(RegionLineSegmentSplitPredicatePath2::AabbFilteredNativeSegment),
        candidate_pair_count: segments
            .len()
            .saturating_mul(segments.len().saturating_sub(1))
            / 2,
        ..LineSegmentSplitEvidenceParts::default()
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
                evidence.skipped_aabb_pair_count += 1;
                continue;
            }
            evidence.tested_pair_count += 1;
            match native_segment_intersection_split_markers(first, second, policy)? {
                NativeSegmentIntersectionMarkers::None => {}
                NativeSegmentIntersectionMarkers::Points(points) => {
                    evidence.point_relation_count += 1;
                    evidence.intersection_event_count += points.len();
                    for point in points {
                        evidence.intersection_points.push(point.point.clone());
                        evidence.intersection_evidence.push(
                            RegionLineSegmentSplitIntersectionEvidence2 {
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
                                &mut evidence,
                                first_index,
                                first.structural_facts().kind,
                                first.start(),
                                first.end(),
                                second_index,
                                second.structural_facts().kind,
                                second.start(),
                                second.end(),
                            );
                            return Ok(Err((evidence, UncertaintyReason::Ordering)));
                        }
                    }
                }
                NativeSegmentIntersectionMarkers::Overlap => {
                    evidence.overlap_relation_count += 1;
                    set_split_blocker_pair(
                        &mut evidence,
                        first_index,
                        first.structural_facts().kind,
                        first.start(),
                        first.end(),
                        second_index,
                        second.structural_facts().kind,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((evidence, UncertaintyReason::Boundary)));
                }
                NativeSegmentIntersectionMarkers::Uncertain(reason) => {
                    evidence.uncertain_relation_count += 1;
                    set_split_blocker_pair(
                        &mut evidence,
                        first_index,
                        first.structural_facts().kind,
                        first.start(),
                        first.end(),
                        second_index,
                        second.structural_facts().kind,
                        second.start(),
                        second.end(),
                    );
                    return Ok(Err((evidence, reason)));
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
                            return Ok(Err((evidence, reason)));
                        }
                    }
                }
                Some(Ordering::Equal) => {}
                Some(Ordering::Greater) | None => {
                    return Ok(Err((evidence, UncertaintyReason::Ordering)));
                }
            }
        }
    }

    evidence.output_segment_count = Some(arranged.len());
    Ok(Ok(ArrangedNativeSegments {
        segments: arranged,
        evidence,
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
    Result<LineSegmentRingAssembly, (LineSegmentRingAssemblyEvidenceParts, UncertaintyReason)>,
> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    let mut counts = LineSegmentRingAssemblyCounts::default();
    let mut reversed_source_segment_count = 0_usize;
    let mut source_evidence = Vec::with_capacity(segments.len());

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let output_ring_index = rings.len();
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        append_line_segment_ring_source_evidence(
            &mut source_evidence,
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
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            reversed_source_segment_count,
                            source_evidence,
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
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            reversed_source_segment_count,
                            source_evidence,
                        },
                        UncertaintyReason::Boundary,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            reversed_source_segment_count,
                            source_evidence,
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
            append_line_segment_ring_source_evidence(
                &mut source_evidence,
                &current,
                output_ring_index,
                ring.len(),
                next.reversed,
            );
            ring.push(current.line.clone());
        }

        if ring.len() < 3 {
            return Ok(Err((
                LineSegmentRingAssemblyEvidenceParts {
                    counts,
                    reversed_source_segment_count,
                    source_evidence,
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
        source_evidence,
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
    source_evidence: Vec<RegionLineSegmentRingSourceEvidence2>,
}

fn assemble_unordered_native_segment_rings(
    segments: &[ArrangedNativeSegment],
    policy: &CurvePolicy,
) -> CurveResult<
    Result<NativeSegmentRingAssembly, (LineSegmentRingAssemblyEvidenceParts, UncertaintyReason)>,
> {
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    let mut counts = LineSegmentRingAssemblyCounts::default();
    let mut reversed_source_segment_count = 0_usize;
    let mut source_evidence = Vec::with_capacity(segments.len());

    while let Some(seed_index) = used.iter().position(|used| !*used) {
        let output_ring_index = rings.len();
        let mut ring = Vec::new();
        let mut current = segments[seed_index].clone();
        used[seed_index] = true;
        append_native_segment_ring_source_evidence(
            &mut source_evidence,
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
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            reversed_source_segment_count,
                            source_evidence,
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
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            reversed_source_segment_count,
                            source_evidence,
                        },
                        UncertaintyReason::Boundary,
                    )));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Err((
                        LineSegmentRingAssemblyEvidenceParts {
                            counts,
                            reversed_source_segment_count,
                            source_evidence,
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
            append_native_segment_ring_source_evidence(
                &mut source_evidence,
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
        source_evidence,
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

fn append_line_segment_ring_source_evidence(
    source_evidence: &mut Vec<RegionLineSegmentRingSourceEvidence2>,
    segment: &ArrangedLineSegment,
    output_ring_index: usize,
    output_segment_index: usize,
    reversed: bool,
) {
    source_evidence.push(RegionLineSegmentRingSourceEvidence2 {
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

fn append_native_segment_ring_source_evidence(
    source_evidence: &mut Vec<RegionLineSegmentRingSourceEvidence2>,
    segment: &ArrangedNativeSegment,
    output_ring_index: usize,
    output_segment_index: usize,
    reversed: bool,
) {
    source_evidence.push(RegionLineSegmentRingSourceEvidence2 {
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

fn line_arranged_source_evidence(
    segments: &[ArrangedLineSegment],
) -> Vec<RegionLineSegmentArrangedSourceEvidence2> {
    segments
        .iter()
        .enumerate()
        .map(
            |(arranged_segment_index, segment)| RegionLineSegmentArrangedSourceEvidence2 {
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

fn native_arranged_source_evidence(
    source_segments: &[Segment2],
    segments: &[ArrangedNativeSegment],
) -> Vec<RegionLineSegmentArrangedSourceEvidence2> {
    segments
        .iter()
        .enumerate()
        .map(
            |(arranged_segment_index, segment)| RegionLineSegmentArrangedSourceEvidence2 {
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
    intersection_evidence: &[RegionLineSegmentSplitIntersectionEvidence2],
) -> ExactCurveArrangementSplitIntersectionBucketCache2 {
    let mut buckets: Vec<ExactCurveArrangementSplitIntersectionBucket2> = Vec::new();
    for (intersection_evidence_index, evidence) in intersection_evidence.iter().enumerate() {
        add_split_intersection_bucket_ref(
            &mut buckets,
            evidence.point(),
            ExactCurveArrangementSplitIntersectionRef2 {
                intersection_evidence_index,
            },
        );
    }

    let intersection_event_count = intersection_evidence.len();
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
    arranged_source_evidence: &[RegionLineSegmentArrangedSourceEvidence2],
) -> ExactCurveArrangementArrangedEndpointBucketCache2 {
    let mut buckets: Vec<ExactCurveArrangementArrangedEndpointBucket2> = Vec::new();
    for evidence in arranged_source_evidence {
        add_arranged_endpoint_bucket_ref(
            &mut buckets,
            evidence.output_start_point(),
            ExactCurveArrangementArrangedEndpointRef2 {
                arranged_segment_index: evidence.arranged_segment_index(),
                endpoint: RegionLineSegmentArrangedEndpoint2::Start,
            },
        );
        add_arranged_endpoint_bucket_ref(
            &mut buckets,
            evidence.output_end_point(),
            ExactCurveArrangementArrangedEndpointRef2 {
                arranged_segment_index: evidence.arranged_segment_index(),
                endpoint: RegionLineSegmentArrangedEndpoint2::End,
            },
        );
    }

    let endpoint_count = arranged_source_evidence.len() * 2;
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

fn arranged_evidence_segment_kind_counts(
    evidence: &[RegionLineSegmentArrangedSourceEvidence2],
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for evidence in evidence {
        match evidence.arranged_segment_kind {
            SegmentKind::Line => counts.lines += 1,
            SegmentKind::Arc => counts.arcs += 1,
        }
    }
    counts
}

fn region_segment_kind_counts(region: &LineArcRegion2) -> SegmentKindCounts {
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

fn blocked_line_segment_region_evidence(
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    split_evidence: Option<LineSegmentSplitEvidenceParts>,
    endpoint_graph_evidence: Option<LineSegmentEndpointGraphEvidenceParts>,
    arranged_source_evidence: Vec<RegionLineSegmentArrangedSourceEvidence2>,
    evidence: LineSegmentRingAssemblyEvidenceParts,
    stage: RegionLineSegmentRegionBuildStage2,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionLineSegmentRegionBuildEvidence2 {
    let split_evidence = split_evidence.unwrap_or_default();
    let arranged_segment_kind_counts = split_evidence
        .output_segment_count
        .map(|_| arranged_evidence_segment_kind_counts(&arranged_source_evidence));
    RegionLineSegmentRegionBuildEvidence2 {
        stage,
        source_segment_count,
        source_segment_kind_counts,
        arranged_segment_count: split_evidence.output_segment_count,
        arranged_segment_kind_counts,
        split_predicate_path: split_evidence.predicate_path,
        endpoint_graph_predicate_path: endpoint_graph_evidence
            .as_ref()
            .map(|_| RegionLineSegmentEndpointGraphPredicatePath2::ExactStructuralEndpointBuckets),
        ring_assembly_predicate_path: endpoint_graph_evidence
            .as_ref()
            .map(|_| RegionLineSegmentRingAssemblyPredicatePath2::ExactEndpointBucketTraversal),
        split_candidate_pair_count: split_evidence.candidate_pair_count,
        split_skipped_aabb_pair_count: split_evidence.skipped_aabb_pair_count,
        split_tested_pair_count: split_evidence.tested_pair_count,
        split_intersection_event_count: split_evidence.intersection_event_count,
        split_point_relation_count: split_evidence.point_relation_count,
        split_overlap_relation_count: split_evidence.overlap_relation_count,
        split_uncertain_relation_count: split_evidence.uncertain_relation_count,
        split_intersection_points: split_evidence.intersection_points,
        split_intersection_evidence: split_evidence.intersection_evidence,
        split_output_segment_count: split_evidence.output_segment_count,
        split_blocker_first_source_segment_index: split_evidence.blocker_first_source_segment_index,
        split_blocker_first_source_segment_kind: split_evidence.blocker_first_source_segment_kind,
        split_blocker_first_source_start_point: split_evidence.blocker_first_source_start_point,
        split_blocker_first_source_end_point: split_evidence.blocker_first_source_end_point,
        split_blocker_second_source_segment_index: split_evidence
            .blocker_second_source_segment_index,
        split_blocker_second_source_segment_kind: split_evidence.blocker_second_source_segment_kind,
        split_blocker_second_source_start_point: split_evidence.blocker_second_source_start_point,
        split_blocker_second_source_end_point: split_evidence.blocker_second_source_end_point,
        endpoint_graph_endpoint_count: endpoint_graph_evidence
            .as_ref()
            .map(|evidence| evidence.endpoint_count),
        endpoint_graph_structural_bucket_count: endpoint_graph_evidence
            .as_ref()
            .map(|evidence| evidence.structural_bucket_count),
        endpoint_graph_structural_singleton_bucket_count: endpoint_graph_evidence
            .as_ref()
            .map(|evidence| evidence.structural_singleton_bucket_count),
        endpoint_graph_max_structural_bucket_size: endpoint_graph_evidence
            .as_ref()
            .map(|evidence| evidence.max_structural_bucket_size),
        endpoint_graph_dangling_endpoint_count: endpoint_graph_evidence
            .as_ref()
            .map(|evidence| evidence.dangling_endpoint_count),
        endpoint_graph_branch_endpoint_count: endpoint_graph_evidence
            .as_ref()
            .map(|evidence| evidence.branch_endpoint_count),
        endpoint_graph_blocker_arranged_segment_index: endpoint_graph_evidence
            .as_ref()
            .and_then(|evidence| evidence.blocker_arranged_segment_index),
        endpoint_graph_blocker_endpoint: endpoint_graph_evidence
            .as_ref()
            .and_then(|evidence| evidence.blocker_endpoint),
        endpoint_graph_blocker_point: endpoint_graph_evidence
            .as_ref()
            .and_then(|evidence| evidence.blocker_point.clone()),
        attempted_endpoint_connection_count: evidence.counts.attempted_endpoint_connection_count,
        exact_endpoint_connection_count: evidence.counts.exact_endpoint_connection_count,
        disconnected_endpoint_connection_count: evidence
            .counts
            .disconnected_endpoint_connection_count,
        unresolved_endpoint_connection_count: evidence.counts.unresolved_endpoint_connection_count,
        reversed_source_segment_count: evidence.reversed_source_segment_count,
        output_ring_count: None,
        output_boundary_segment_count: None,
        output_boundary_segment_kind_counts: None,
        arranged_source_evidence,
        source_evidence: evidence.source_evidence,
        boundary_build_evidence: None,
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
        evidence: RegionBoundaryContourBuildEvidence2 {
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
            role_evidence: Vec::new(),
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

fn line_contour_directed_orientation(contour: &Contour2, policy: &CurvePolicy) -> Option<RealSign> {
    line_contour_direction_winding_orientation(contour, policy)
        .or_else(|| line_contour_extreme_vertex_orientation(contour, policy))
}

fn line_contour_extreme_vertex_orientation(
    contour: &Contour2,
    policy: &CurvePolicy,
) -> Option<RealSign> {
    let segments = contour.segments();
    let Segment2::Line(first) = segments.first()? else {
        return None;
    };
    let mut minimum_index = 0;
    let mut minimum = first.start();
    for (index, segment) in segments.iter().enumerate().skip(1) {
        let Segment2::Line(line) = segment else {
            return None;
        };
        let current = line.start();
        let order = compare_reals(current.x(), minimum.x(), policy)?;
        if order == Ordering::Less
            || order == Ordering::Equal
                && compare_reals(current.y(), minimum.y(), policy)? == Ordering::Less
        {
            minimum_index = index;
            minimum = current;
        }
    }
    let Segment2::Line(incoming) = &segments[(minimum_index + segments.len() - 1) % segments.len()]
    else {
        return None;
    };
    let Segment2::Line(outgoing) = &segments[minimum_index] else {
        return None;
    };
    let (incoming_x, incoming_y) = incoming.delta();
    let (outgoing_x, outgoing_y) = outgoing.delta();
    match real_sign(
        &Real::diff_of_products(&incoming_x, &outgoing_y, &incoming_y, &outgoing_x),
        policy,
    )? {
        RealSign::Positive => Some(RealSign::Positive),
        RealSign::Negative => Some(RealSign::Negative),
        RealSign::Zero => None,
    }
}

fn line_contour_direction_winding_orientation(
    contour: &Contour2,
    policy: &CurvePolicy,
) -> Option<RealSign> {
    // The tangent map of a simple closed polyline has rotation index +1 or -1,
    // matching the contour orientation. Connecting consecutive nonzero edge
    // directions by their shorter angular turn preserves that index, so an
    // exact ray-crossing winding count around the origin recovers the role.
    // Retained line supports are collinear with each fragment; the explicit
    // reversal bit preserves their directed angle while avoiding arithmetic on
    // wide intersection endpoints.
    let mut segments = contour.segments().iter();
    let Segment2::Line(first_line) = segments.next()? else {
        return None;
    };
    let first_direction = first_line.directed_support_delta();
    let mut previous_direction = first_direction.clone();
    let mut winding = 0_i32;
    for segment in segments {
        let Segment2::Line(line) = segment else {
            return None;
        };
        let direction = line.directed_support_delta();
        accumulate_direction_winding(&previous_direction, &direction, &mut winding, policy)?;
        previous_direction = direction;
    }
    accumulate_direction_winding(&previous_direction, &first_direction, &mut winding, policy)?;

    match winding {
        1 => Some(RealSign::Positive),
        -1 => Some(RealSign::Negative),
        _ => None,
    }
}

fn accumulate_direction_winding(
    previous: &(Real, Real),
    next: &(Real, Real),
    winding: &mut i32,
    policy: &CurvePolicy,
) -> Option<()> {
    let previous_y = real_sign(&previous.1, policy)?;
    let next_y = real_sign(&next.1, policy)?;
    let previous_x = if previous_y == RealSign::Zero {
        Some(real_sign(&previous.0, policy)?)
    } else {
        None
    };
    let next_x = if next_y == RealSign::Zero {
        Some(real_sign(&next.0, policy)?)
    } else {
        None
    };
    let previous_is_zero = previous_y == RealSign::Zero && previous_x == Some(RealSign::Zero);
    let next_is_zero = next_y == RealSign::Zero && next_x == Some(RealSign::Zero);
    let horizontal_half_turn =
        previous_y == RealSign::Zero && next_y == RealSign::Zero && previous_x != next_x;
    if previous_is_zero || next_is_zero || horizontal_half_turn {
        return None;
    }
    let upward = previous_y != RealSign::Positive && next_y == RealSign::Positive;
    let downward = previous_y == RealSign::Positive && next_y != RealSign::Positive;
    if !upward && !downward {
        return Some(());
    }

    let turn = real_sign(
        &Real::diff_of_products(&previous.0, &next.1, &previous.1, &next.0),
        policy,
    )?;
    match (upward, turn) {
        (_, RealSign::Zero) => None,
        (true, RealSign::Positive) => {
            *winding = winding.checked_add(1)?;
            Some(())
        }
        (false, RealSign::Negative) => {
            *winding = winding.checked_sub(1)?;
            Some(())
        }
        _ => Some(()),
    }
}

fn contour_aabb_overlap_neighbors(
    contour_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Option<Vec<Vec<usize>>> {
    const MIN_RETAINED_NEIGHBOR_COUNT: usize = 4_096;
    const RETAINED_NEIGHBORS_PER_CONTOUR: usize = 32;

    let retained_neighbor_limit = contour_boxes
        .len()
        .saturating_mul(RETAINED_NEIGHBORS_PER_CONTOUR)
        .max(MIN_RETAINED_NEIGHBOR_COUNT);
    let mut ordered = contour_boxes
        .iter()
        .enumerate()
        .filter_map(|(index, bbox)| bbox.as_ref().map(|bbox| (index, bbox)))
        .collect::<Vec<_>>();
    let mut ordering_failed = false;
    ordered.sort_by(|(_, left), (_, right)| {
        compare_reals(left.min().x(), right.min().x(), policy).unwrap_or_else(|| {
            ordering_failed = true;
            Ordering::Equal
        })
    });
    if ordering_failed {
        return None;
    }

    let mut neighbors = (0..contour_boxes.len())
        .map(|_| Vec::new())
        .collect::<Vec<_>>();
    let mut retained_neighbor_count = 0_usize;
    let mut active = Vec::<(usize, &Aabb2)>::new();
    for (current_index, current_box) in ordered {
        active.retain(|(_, active_box)| {
            compare_reals(active_box.max().x(), current_box.min().x(), policy)
                != Some(Ordering::Less)
        });
        for &(active_index, active_box) in &active {
            if aabbs_decided_disjoint(active_box, current_box, policy) {
                continue;
            }
            retained_neighbor_count += 1;
            if retained_neighbor_count > retained_neighbor_limit {
                return None;
            }
            neighbors[active_index].push(current_index);
            neighbors[current_index].push(active_index);
        }
        active.push((current_index, current_box));
    }

    for undecided_index in contour_boxes
        .iter()
        .enumerate()
        .filter_map(|(index, bbox)| bbox.is_none().then_some(index))
    {
        for other_index in 0..contour_boxes.len() {
            if undecided_index == other_index
                || contour_boxes[other_index].is_none() && other_index < undecided_index
            {
                continue;
            }
            retained_neighbor_count += 1;
            if retained_neighbor_count > retained_neighbor_limit {
                return None;
            }
            neighbors[undecided_index].push(other_index);
            neighbors[other_index].push(undecided_index);
        }
    }

    for contour_neighbors in &mut neighbors {
        contour_neighbors.sort_unstable();
    }
    Some(neighbors)
}

fn aabb_may_contain(outer: &Aabb2, inner: &Aabb2, policy: &CurvePolicy) -> bool {
    let mut predicates = [
        (
            outer
                .min_x()
                .to_f64_lossy()
                .zip(inner.min_x().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(outer, inner)| outer - inner),
            outer.min_x(),
            inner.min_x(),
            Ordering::Greater,
        ),
        (
            outer
                .min_y()
                .to_f64_lossy()
                .zip(inner.min_y().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(outer, inner)| outer - inner),
            outer.min_y(),
            inner.min_y(),
            Ordering::Greater,
        ),
        (
            inner
                .max_x()
                .to_f64_lossy()
                .zip(outer.max_x().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(inner, outer)| inner - outer),
            outer.max_x(),
            inner.max_x(),
            Ordering::Less,
        ),
        (
            inner
                .max_y()
                .to_f64_lossy()
                .zip(outer.max_y().to_f64_lossy())
                .map_or(f64::NEG_INFINITY, |(inner, outer)| inner - outer),
            outer.max_y(),
            inner.max_y(),
            Ordering::Less,
        ),
    ];
    predicates.sort_by(|left, right| right.0.total_cmp(&left.0));
    predicates
        .into_iter()
        .all(|(_, outer, inner, violation)| compare_reals(outer, inner, policy) != Some(violation))
}

fn contour_nesting_depths(
    contours: &[Contour2],
    policy: &CurvePolicy,
) -> CurveResult<BoundaryContourNestingOutcome> {
    contour_nesting_depths_impl(contours, policy, true)
}

fn contour_nesting_depths_impl(
    contours: &[Contour2],
    policy: &CurvePolicy,
    validate_intersections: bool,
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
    // Region construction compares every contour pair, then classifies a
    // boundary sample against every other contour. Retain the broad-phase
    // certificates across both passes instead of rebuilding them for every
    // pair and point query. Segment boxes stay lazy because disjoint contour
    // boxes settle the common case without them.
    let contour_boxes = contours
        .iter()
        .map(|contour| decided_contour_aabb(contour, policy))
        .collect::<Vec<_>>();
    let segment_boxes = (0..contours.len())
        .map(|_| OnceCell::<Vec<Option<Aabb2>>>::new())
        .collect::<Vec<_>>();
    let prepared_contours = (0..contours.len())
        .map(|_| OnceCell::<crate::prepared::ContourQuery2<'_>>::new())
        .collect::<Vec<_>>();
    let aabb_overlap_neighbors = contour_aabb_overlap_neighbors(&contour_boxes, policy);

    if validate_intersections {
        for (left_index, left) in contours.iter().enumerate() {
            let mut neighbor_position = aabb_overlap_neighbors.as_ref().map_or(0, |neighbors| {
                neighbors[left_index].partition_point(|&index| index <= left_index)
            });
            for (right_offset, right) in contours[left_index + 1..].iter().enumerate() {
                counts.tested_pair_count += 1;
                let right_index = left_index + 1 + right_offset;
                if let Some(neighbors) = &aabb_overlap_neighbors {
                    if neighbors[left_index].get(neighbor_position).copied() != Some(right_index) {
                        continue;
                    }
                    neighbor_position += 1;
                }
                if let (Some(left_box), Some(right_box)) = (
                    contour_boxes[left_index].as_ref(),
                    contour_boxes[right_index].as_ref(),
                ) && aabbs_decided_disjoint(left_box, right_box, policy)
                {
                    continue;
                }
                let intersections = if let (Some(left_boxes), Some(right_boxes)) = (
                    left.exact_dyadic_line_aabbs(policy),
                    right.exact_dyadic_line_aabbs(policy),
                ) {
                    crate::events::intersect_contours_with_exact_dyadic_line_aabbs(
                        left,
                        right,
                        &left_boxes,
                        &right_boxes,
                        policy,
                    )?
                } else {
                    let left_segment_boxes = segment_boxes[left_index].get_or_init(|| {
                        left.segments()
                            .iter()
                            .map(|segment| decided_segment_aabb(segment, policy))
                            .collect()
                    });
                    let right_segment_boxes = segment_boxes[right_index].get_or_init(|| {
                        right
                            .segments()
                            .iter()
                            .map(|segment| decided_segment_aabb(segment, policy))
                            .collect()
                    });
                    crate::events::intersect_contours_with_cached_aabbs(
                        left,
                        right,
                        contour_boxes[left_index].as_ref(),
                        contour_boxes[right_index].as_ref(),
                        left_segment_boxes,
                        right_segment_boxes,
                        None,
                        policy,
                    )?
                };
                counts.intersection_event_count += intersections.len();
                if let Some(reason) = contour_intersection_blocker(&intersections) {
                    return Ok(BoundaryContourNestingOutcome::Blocked {
                        blocker: BoundaryContourNestingBlocker {
                            reason,
                            first_contour_index: left_index,
                            second_contour_index: left_index + 1 + right_offset,
                        },
                        counts,
                    });
                }
            }
        }
    }

    let mut entries = Vec::with_capacity(contours.len());

    for (candidate_index, candidate) in contours.iter().enumerate() {
        // A point on the candidate boundary is sufficient for nesting against
        // every *other* non-touching contour. This reduces role assignment to
        // repeated point-in-polygon classification. If that sample lies on
        // another contour boundary, return uncertainty instead of inventing a
        // role.
        let first_sample = candidate
            .segments()
            .first()
            .ok_or(CurveError::EmptyCurveString)?
            .start()
            .clone();
        let fractions = [
            (Real::one() / Real::from(2_i8))?,
            (Real::one() / Real::from(3_i8))?,
            (Real::from(2_i8) / Real::from(3_i8))?,
        ];
        // The exact validation pass above, or the trusted Boolean assembly
        // supplying this internal path, proves that every point on this
        // candidate boundary is off every other contour boundary. Its first
        // endpoint is therefore a sufficient nesting sample. Retain interior
        // samples as exact fallbacks for an undecided containment predicate,
        // but do not eagerly interpolate them when that endpoint decides.
        let fallback_samples = candidate.segments().iter().flat_map(|segment| {
            fractions
                .iter()
                .map(move |fraction| segment.point_at(fraction, policy))
        });
        let samples =
            std::iter::once(Ok(Classification::Decided(first_sample))).chain(fallback_samples);

        let mut last_blocker = None;
        let mut decided_entry = None;
        'sample: for sample in samples {
            let Classification::Decided(sample) = sample? else {
                continue;
            };
            let mut containing_contour_indices = Vec::new();
            let mut neighbor_position = 0;
            for (container_index, container) in contours.iter().enumerate() {
                if candidate_index == container_index {
                    continue;
                }

                counts.nesting_classification_count += 1;
                if let Some(neighbors) = &aabb_overlap_neighbors {
                    if neighbors[candidate_index].get(neighbor_position).copied()
                        != Some(container_index)
                    {
                        continue;
                    }
                    neighbor_position += 1;
                }
                if let (Some(container_box), Some(candidate_box)) = (
                    contour_boxes[container_index].as_ref(),
                    contour_boxes[candidate_index].as_ref(),
                ) && !aabb_may_contain(container_box, candidate_box, policy)
                {
                    continue;
                }
                if contour_boxes[container_index]
                    .as_ref()
                    .is_some_and(|bbox| aabb_decided_misses_point(bbox, &sample, policy))
                {
                    continue;
                }
                let prepared = prepared_contours[container_index].get_or_init(|| {
                    crate::prepared::ContourQuery2::from_contour(container, policy)
                });
                match prepared.classify_point_assuming_off_boundary(&sample, policy) {
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
                        last_blocker = Some((reason, container_index));
                        continue 'sample;
                    }
                }
            }
            decided_entry = Some(BoundaryContourNestingEntry {
                sample_point: sample,
                containing_contour_indices,
            });
            break;
        }

        let Some(entry) = decided_entry else {
            let (reason, container_index) = last_blocker.unwrap_or((
                crate::UncertaintyReason::Unsupported,
                usize::from(candidate_index == 0),
            ));
            return Ok(BoundaryContourNestingOutcome::Blocked {
                blocker: BoundaryContourNestingBlocker {
                    reason,
                    first_contour_index: candidate_index,
                    second_contour_index: container_index,
                },
                counts,
            });
        };
        entries.push(entry);
    }

    Ok(BoundaryContourNestingOutcome::Decided {
        nesting: BoundaryContourNestingDepths { entries },
        counts,
    })
}

fn contour_intersection_blocker(
    intersections: &crate::ContourIntersectionSet,
) -> Option<UncertaintyReason> {
    let mut uncertainty = None;
    for event in intersections.events() {
        match event {
            crate::ContourIntersection::Point(_) | crate::ContourIntersection::Overlap(_) => {
                return Some(UncertaintyReason::Boundary);
            }
            crate::ContourIntersection::Uncertain(event) => {
                uncertainty.get_or_insert(event.reason);
            }
        }
    }
    uncertainty
}

#[cfg(test)]
mod tests {
    use super::{
        accumulate_direction_winding, contour_intersection_blocker,
        line_contour_directed_orientation, line_contour_direction_winding_orientation,
        line_contour_extreme_vertex_orientation,
    };
    use crate::{
        BulgeVertex2, Contour2, ContourIntersection, ContourIntersectionSet,
        ContourPointIntersection, ContourUncertainIntersection, CurvePolicy, IntersectionKind,
        LineSeg2, Point2, Segment2, SegmentKind, UncertaintyReason,
    };
    use hyperreal::{Real, RealSign};

    fn line_contour(points: &[(i64, i64)]) -> Contour2 {
        let vertices = points
            .iter()
            .map(|&(x, y)| {
                BulgeVertex2::new(Point2::new(Real::from(x), Real::from(y)), Real::zero())
            })
            .collect::<Vec<_>>();
        Contour2::from_bulge_vertices(&vertices).unwrap()
    }

    fn retained_split_line_contour(points: &[(i64, i64)], reversed: bool) -> Contour2 {
        let half = (Real::one() / Real::from(2)).unwrap();
        let mut segments = Vec::with_capacity(points.len() * 2);
        for index in 0..points.len() {
            let (start_x, start_y) = points[index];
            let (end_x, end_y) = points[(index + 1) % points.len()];
            let source = LineSeg2::try_new(
                Point2::new(Real::from(start_x), Real::from(start_y)),
                Point2::new(Real::from(end_x), Real::from(end_y)),
            )
            .unwrap();
            let midpoint = source.point_at(half.clone());
            let support = source.fragment_support();
            segments.push(Segment2::Line(
                source.fragment_between_after_distinct_endpoints(
                    source.start().clone(),
                    midpoint.clone(),
                    support.clone(),
                ),
            ));
            segments.push(Segment2::Line(
                source.fragment_between_after_distinct_endpoints(
                    midpoint,
                    source.end().clone(),
                    support,
                ),
            ));
        }
        if reversed {
            segments = segments
                .into_iter()
                .rev()
                .map(|segment| match segment {
                    Segment2::Line(line) => Segment2::Line(line.into_reversed()),
                    Segment2::Arc(_) => unreachable!(),
                })
                .collect();
        }
        Contour2::try_new(segments).unwrap()
    }

    #[test]
    fn extreme_turn_recovers_concave_line_contour_orientation() {
        let points = [(0, 0), (4, 0), (4, 4), (2, 2), (0, 4)];
        let forward = line_contour(&points);
        let reverse = line_contour(&points.into_iter().rev().collect::<Vec<_>>());
        let policy = CurvePolicy::certified();

        assert_eq!(
            line_contour_directed_orientation(&forward, &policy),
            Some(RealSign::Positive)
        );
        assert_eq!(
            line_contour_directed_orientation(&reverse, &policy),
            Some(RealSign::Negative)
        );
    }

    #[test]
    fn direction_winding_matches_extreme_vertex_on_simple_concave_contours() {
        let policy = CurvePolicy::certified();
        for points in [
            vec![(0, 0), (8, 0), (8, 8), (4, 4), (0, 8)],
            vec![
                (0, 0),
                (10, 0),
                (10, 10),
                (8, 10),
                (8, 2),
                (6, 2),
                (6, 8),
                (4, 8),
                (4, 2),
                (2, 2),
                (2, 10),
                (0, 10),
            ],
        ] {
            for ordered in [
                points.clone(),
                points.iter().copied().rev().collect::<Vec<_>>(),
            ] {
                let contour = line_contour(&ordered);
                assert_eq!(
                    line_contour_direction_winding_orientation(&contour, &policy),
                    line_contour_extreme_vertex_orientation(&contour, &policy)
                );
            }
        }
    }

    #[test]
    fn direction_winding_tracks_reversed_retained_fragment_supports() {
        let points = [(0, 0), (8, 0), (8, 8), (4, 4), (0, 8)];
        let policy = CurvePolicy::certified();
        let forward = retained_split_line_contour(&points, false);
        let reverse = retained_split_line_contour(&points, true);

        assert_eq!(
            line_contour_direction_winding_orientation(&forward, &policy),
            Some(RealSign::Positive)
        );
        assert_eq!(
            line_contour_direction_winding_orientation(&reverse, &policy),
            Some(RealSign::Negative)
        );
        assert_eq!(
            line_contour_directed_orientation(&forward, &policy),
            line_contour_extreme_vertex_orientation(&forward, &policy)
        );
        assert_eq!(
            line_contour_directed_orientation(&reverse, &policy),
            line_contour_extreme_vertex_orientation(&reverse, &policy)
        );
    }

    #[test]
    fn direction_winding_rejects_an_exact_half_turn() {
        let mut winding = 0;
        assert_eq!(
            accumulate_direction_winding(
                &(Real::one(), Real::zero()),
                &(-Real::one(), Real::zero()),
                &mut winding,
                &CurvePolicy::certified(),
            ),
            None
        );
    }

    #[test]
    fn extreme_turn_defers_mixed_line_arc_contours_to_nesting() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(Point2::new(Real::from(0), Real::from(0)), Real::zero()),
            BulgeVertex2::new(Point2::new(Real::from(4), Real::from(0)), Real::one()),
            BulgeVertex2::new(Point2::new(Real::from(0), Real::from(4)), Real::zero()),
        ])
        .unwrap();

        assert_eq!(
            line_contour_directed_orientation(&contour, &CurvePolicy::certified()),
            None
        );
    }

    #[test]
    fn contour_nesting_preserves_uncertain_intersection_reason() {
        let intersections = ContourIntersectionSet::new(vec![ContourIntersection::Uncertain(
            ContourUncertainIntersection {
                a_segment_index: 2,
                b_segment_index: 4,
                a_segment_kind: SegmentKind::Arc,
                b_segment_kind: SegmentKind::Line,
                reason: UncertaintyReason::RealSign,
            },
        )])
        .unwrap();

        assert_eq!(
            contour_intersection_blocker(&intersections),
            Some(UncertaintyReason::RealSign)
        );
    }

    #[test]
    fn contour_nesting_prefers_decided_contact_over_uncertainty() {
        let intersections = ContourIntersectionSet::new(vec![
            ContourIntersection::Uncertain(ContourUncertainIntersection {
                a_segment_index: 0,
                b_segment_index: 0,
                a_segment_kind: SegmentKind::Line,
                b_segment_kind: SegmentKind::Line,
                reason: UncertaintyReason::Ordering,
            }),
            ContourIntersection::Point(ContourPointIntersection {
                a_segment_index: 1,
                b_segment_index: 1,
                a_segment_kind: SegmentKind::Line,
                b_segment_kind: SegmentKind::Line,
                point: Point2::new(Real::zero(), Real::zero()),
                a_param: Real::zero(),
                b_param: Real::zero(),
                kind: IntersectionKind::Endpoint,
            }),
        ])
        .unwrap();

        assert_eq!(
            contour_intersection_blocker(&intersections),
            Some(UncertaintyReason::Boundary)
        );
    }
}
