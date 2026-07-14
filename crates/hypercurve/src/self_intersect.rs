//! Self-contact detection for curve strings and closed contours.

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::is_zero;
use crate::{
    ArcArcIntersection, Classification, Contour2, CurvePolicy, CurveResult, CurveString2,
    LineArcIntersection, LineLineIntersection, Point2, Segment2, SegmentIntersection,
    SegmentKindCounts, UncertaintyReason,
};
use crate::{RetainedTopologyStatus, SegmentKind};

/// Report for exact self-contact classification on an open or closed path.
#[derive(Clone, Debug, PartialEq)]
pub struct SelfContactReport2 {
    closed: bool,
    segment_count: usize,
    segment_kind_counts: SegmentKindCounts,
    prepared_cache_report: Option<SelfContactPreparedCacheReport2>,
    predicate_path: SelfContactPredicatePath2,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    first_contact_first_segment_index: Option<usize>,
    first_contact_second_segment_index: Option<usize>,
    first_contact_first_segment_start_point: Option<Point2>,
    first_contact_first_segment_end_point: Option<Point2>,
    first_contact_second_segment_start_point: Option<Point2>,
    first_contact_second_segment_end_point: Option<Point2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing self-contact classification.
#[derive(Clone, Debug, PartialEq)]
pub struct SelfContactResult2 {
    has_self_contacts: Classification<bool>,
    report: SelfContactReport2,
}

/// Exact predicate family used while scanning non-adjacent segment contacts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SelfContactPredicatePath2 {
    /// Segment pairs were filtered by AABB before exact segment-intersection predicates.
    AabbFilteredExactSegmentIntersections,
}

/// Prepared-cache evidence consumed by a self-contact scan.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SelfContactPreparedCacheReport2 {
    freshness: SelfContactPreparedCacheFreshness2,
    prepared_segment_count: usize,
    prepared_segment_kind_counts: SegmentKindCounts,
    decided_segment_box_count: usize,
    undecided_segment_box_count: usize,
    path_box_decided: bool,
}

/// Freshness claim for prepared self-contact cache evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SelfContactPreparedCacheFreshness2 {
    /// Prepared cache borrows the current source path for this query.
    BorrowedCurrentSource,
}

impl CurveString2 {
    /// Classifies whether this open curve string has non-adjacent self contacts.
    ///
    /// Adjacent segment endpoint contacts are expected curve-string
    /// connectivity and are ignored. Unlike closed contours, the first and last
    /// segments are not considered adjacent unless they are consecutive in the
    /// open sequence.
    ///
    /// This is an exactness-aware `O(n^2)` pair enumeration with an
    /// axis-aligned bounding-box broad phase. A sweep-line candidate generator
    /// can replace the flat enumeration when larger inputs warrant it.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        self.has_self_contacts_with_report(policy)
            .map(|result| result.has_self_contacts)
    }

    /// Classifies self contacts and retains broad-phase scan evidence.
    pub fn has_self_contacts_with_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<SelfContactResult2> {
        let boxes: Vec<_> = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        segments_have_self_contacts_with_cached_aabbs_and_report(
            self.segments(),
            &boxes,
            false,
            policy,
        )
    }
}

impl Contour2 {
    /// Classifies whether this contour has non-adjacent self contacts.
    ///
    /// Adjacent segment endpoint contacts, including the closing edge back to
    /// the first segment, are expected contour connectivity and are ignored.
    /// Crossings, tangencies, endpoint contacts, and overlaps between
    /// non-adjacent segments are all reported as self contacts.
    ///
    /// This is an exactness-aware `O(n^2)` pair enumeration with an
    /// axis-aligned bounding-box broad phase. Later arrangement and offset
    /// trimming work can replace it with a sweep-line candidate generator;
    /// a sweep-line candidate generator is the standard replacement for that
    /// asymptotically better reporting pattern.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        self.has_self_contacts_with_report(policy)
            .map(|result| result.has_self_contacts)
    }

    /// Classifies self contacts and retains broad-phase scan evidence.
    pub fn has_self_contacts_with_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<SelfContactResult2> {
        let boxes: Vec<_> = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        segments_have_self_contacts_with_cached_aabbs_and_report(
            self.segments(),
            &boxes,
            true,
            policy,
        )
    }
}

pub(crate) fn segments_have_self_contacts_with_cached_aabbs_and_report(
    segments: &[Segment2],
    boxes: &[Option<Aabb2>],
    closed: bool,
    policy: &CurvePolicy,
) -> CurveResult<SelfContactResult2> {
    let segment_kind_counts = self_contact_segment_kind_counts(segments);
    let mut candidate_pair_count = 0_usize;
    let mut skipped_aabb_pair_count = 0_usize;
    let mut tested_pair_count = 0_usize;
    let x_overlap_schedule = self_contact_x_overlap_schedule(boxes, policy);

    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
            candidate_pair_count += 1;
            if x_overlap_schedule
                .as_ref()
                .is_some_and(|schedule| !schedule.overlaps(first_index, second_index))
            {
                skipped_aabb_pair_count += 1;
                continue;
            }
            // The broad phase is allowed to skip only when non-overlap is
            // decided. If a box or coordinate ordering is uncertain, exact
            // segment topology below remains authoritative.
            if let (Some(Some(first_box)), Some(Some(second_box))) =
                (boxes.get(first_index), boxes.get(second_index))
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                skipped_aabb_pair_count += 1;
                continue;
            }

            tested_pair_count += 1;
            let relation =
                segments[first_index].intersect_segment(&segments[second_index], policy)?;
            let connectivity_point =
                connected_segments_vertex(segments, first_index, second_index, closed);
            match segment_relation_has_contact(&relation, connectivity_point, policy) {
                Classification::Decided(true) => {
                    return Ok(SelfContactResult2 {
                        has_self_contacts: Classification::Decided(true),
                        report: SelfContactReport2 {
                            closed,
                            segment_count: segments.len(),
                            segment_kind_counts,
                            prepared_cache_report: None,
                            predicate_path:
                                SelfContactPredicatePath2::AabbFilteredExactSegmentIntersections,
                            candidate_pair_count,
                            skipped_aabb_pair_count,
                            tested_pair_count,
                            first_contact_first_segment_index: Some(first_index),
                            first_contact_second_segment_index: Some(second_index),
                            first_contact_first_segment_start_point: Some(
                                segments[first_index].start().clone(),
                            ),
                            first_contact_first_segment_end_point: Some(
                                segments[first_index].end().clone(),
                            ),
                            first_contact_second_segment_start_point: Some(
                                segments[second_index].start().clone(),
                            ),
                            first_contact_second_segment_end_point: Some(
                                segments[second_index].end().clone(),
                            ),
                            status: RetainedTopologyStatus::NativeExact,
                            blocker: None,
                        },
                    });
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(SelfContactResult2 {
                        has_self_contacts: Classification::Uncertain(reason),
                        report: SelfContactReport2 {
                            closed,
                            segment_count: segments.len(),
                            segment_kind_counts,
                            prepared_cache_report: None,
                            predicate_path:
                                SelfContactPredicatePath2::AabbFilteredExactSegmentIntersections,
                            candidate_pair_count,
                            skipped_aabb_pair_count,
                            tested_pair_count,
                            first_contact_first_segment_index: None,
                            first_contact_second_segment_index: None,
                            first_contact_first_segment_start_point: None,
                            first_contact_first_segment_end_point: None,
                            first_contact_second_segment_start_point: None,
                            first_contact_second_segment_end_point: None,
                            status: RetainedTopologyStatus::Unresolved,
                            blocker: Some(reason),
                        },
                    });
                }
            }
        }
    }

    Ok(SelfContactResult2 {
        has_self_contacts: Classification::Decided(false),
        report: SelfContactReport2 {
            closed,
            segment_count: segments.len(),
            segment_kind_counts,
            prepared_cache_report: None,
            predicate_path: SelfContactPredicatePath2::AabbFilteredExactSegmentIntersections,
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            first_contact_first_segment_index: None,
            first_contact_second_segment_index: None,
            first_contact_first_segment_start_point: None,
            first_contact_first_segment_end_point: None,
            first_contact_second_segment_start_point: None,
            first_contact_second_segment_end_point: None,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    })
}

fn self_contact_x_overlap_schedule(
    boxes: &[Option<Aabb2>],
    _policy: &CurvePolicy,
) -> Option<SelfContactXSchedule> {
    const ENCLOSURE_PRECISION: i32 = -32;

    let count = boxes.len();
    let decided_boxes = boxes
        .iter()
        .map(Option::as_ref)
        .collect::<Option<Vec<_>>>()?;
    let x_intervals = decided_boxes
        .iter()
        .map(|bbox| {
            Some([
                bbox.min_x()
                    .certified_dyadic_interval(ENCLOSURE_PRECISION)?,
                bbox.max_x()
                    .certified_dyadic_interval(ENCLOSURE_PRECISION)?,
            ])
        })
        .collect::<Option<Vec<_>>>()?;
    let mut order = (0..count).collect::<Vec<_>>();

    // Sort conservative lower endpoints. This need not recover the exact total
    // order of overlapping coordinates: a pair is pruned only when the later
    // lower bound is strictly above the earlier segment's certified upper
    // bound.
    order.sort_by(|left, right| {
        x_intervals[*left][0][0]
            .partial_cmp(&x_intervals[*right][0][0])
            .expect("rational interval endpoints are totally ordered")
            .then_with(|| left.cmp(right))
    });

    let mut ranks = vec![0; count];
    for (rank, &segment_index) in order.iter().enumerate() {
        ranks[segment_index] = rank;
    }
    let mut overlap_ends = Vec::with_capacity(count);
    for (position, &first_index) in order.iter().enumerate() {
        let first_maximum_upper = &x_intervals[first_index][1][1];
        let mut overlap_end = position;
        for (second_position, &second_index) in order[position + 1..].iter().enumerate() {
            let second_minimum_lower = &x_intervals[second_index][0][0];
            if second_minimum_lower > first_maximum_upper {
                break;
            }
            overlap_end = position + second_position + 1;
        }
        overlap_ends.push(overlap_end);
    }
    Some(SelfContactXSchedule {
        ranks,
        overlap_ends,
    })
}

struct SelfContactXSchedule {
    ranks: Vec<usize>,
    overlap_ends: Vec<usize>,
}

impl SelfContactXSchedule {
    #[inline]
    fn overlaps(&self, first_index: usize, second_index: usize) -> bool {
        let first_rank = self.ranks[first_index];
        let second_rank = self.ranks[second_index];
        let (earlier, later) = if first_rank <= second_rank {
            (first_rank, second_rank)
        } else {
            (second_rank, first_rank)
        };
        later <= self.overlap_ends[earlier]
    }
}

impl SelfContactReport2 {
    /// Returns whether closing-endpoint adjacency was treated as ordinary connectivity.
    pub const fn closed(&self) -> bool {
        self.closed
    }

    /// Returns the source segment count scanned.
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// Returns primitive-family counts for scanned segments.
    pub const fn segment_kind_counts(&self) -> SegmentKindCounts {
        self.segment_kind_counts
    }

    /// Returns prepared-cache inventory and freshness evidence, when used.
    pub const fn prepared_cache_report(&self) -> Option<&SelfContactPreparedCacheReport2> {
        self.prepared_cache_report.as_ref()
    }

    /// Returns the exact predicate/filter path used by the self-contact scan.
    pub const fn predicate_path(&self) -> SelfContactPredicatePath2 {
        self.predicate_path
    }

    /// Returns visited segment-pair candidates before early decision.
    pub const fn candidate_pair_count(&self) -> usize {
        self.candidate_pair_count
    }

    /// Returns visited segment pairs skipped by decided disjoint AABBs.
    pub const fn skipped_aabb_pair_count(&self) -> usize {
        self.skipped_aabb_pair_count
    }

    /// Returns visited segment pairs tested by exact segment topology.
    pub const fn tested_pair_count(&self) -> usize {
        self.tested_pair_count
    }

    /// Returns the first segment index of the first certified self contact.
    pub const fn first_contact_first_segment_index(&self) -> Option<usize> {
        self.first_contact_first_segment_index
    }

    /// Returns the second segment index of the first certified self contact.
    pub const fn first_contact_second_segment_index(&self) -> Option<usize> {
        self.first_contact_second_segment_index
    }

    /// Returns the exact start point of the first contacted segment, when certified.
    pub const fn first_contact_first_segment_start_point(&self) -> Option<&Point2> {
        self.first_contact_first_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the first contacted segment, when certified.
    pub const fn first_contact_first_segment_end_point(&self) -> Option<&Point2> {
        self.first_contact_first_segment_end_point.as_ref()
    }

    /// Returns the exact start point of the second contacted segment, when certified.
    pub const fn first_contact_second_segment_start_point(&self) -> Option<&Point2> {
        self.first_contact_second_segment_start_point.as_ref()
    }

    /// Returns the exact end point of the second contacted segment, when certified.
    pub const fn first_contact_second_segment_end_point(&self) -> Option<&Point2> {
        self.first_contact_second_segment_end_point.as_ref()
    }

    /// Returns retained topology status for the self-contact classification.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for unresolved self-contact classification.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl SelfContactResult2 {
    /// Returns the self-contact classification.
    pub const fn has_self_contacts(&self) -> Classification<bool> {
        self.has_self_contacts
    }

    /// Consumes this result and returns the self-contact classification.
    pub fn into_has_self_contacts(self) -> Classification<bool> {
        self.has_self_contacts
    }

    /// Consumes this result and returns retained scan evidence.
    pub fn into_report(self) -> SelfContactReport2 {
        self.report
    }

    /// Consumes this result and returns the self-contact classification with its report.
    pub fn into_parts(self) -> (Classification<bool>, SelfContactReport2) {
        (self.has_self_contacts, self.report)
    }

    /// Returns retained scan evidence for self-contact classification.
    pub const fn report(&self) -> &SelfContactReport2 {
        &self.report
    }

    /// Attaches prepared-cache evidence to a self-contact scan result.
    pub(crate) fn with_prepared_cache(
        mut self,
        prepared_cache_report: SelfContactPreparedCacheReport2,
    ) -> Self {
        self.report.prepared_cache_report = Some(prepared_cache_report);
        self
    }
}

impl SelfContactPreparedCacheReport2 {
    /// Builds prepared-cache evidence for one self-contact scan source.
    pub(crate) const fn new(
        prepared_segment_count: usize,
        prepared_segment_kind_counts: SegmentKindCounts,
        decided_segment_box_count: usize,
        undecided_segment_box_count: usize,
        path_box_decided: bool,
    ) -> Self {
        Self {
            freshness: SelfContactPreparedCacheFreshness2::BorrowedCurrentSource,
            prepared_segment_count,
            prepared_segment_kind_counts,
            decided_segment_box_count,
            undecided_segment_box_count,
            path_box_decided,
        }
    }

    /// Returns the freshness claim for the prepared cache.
    pub const fn freshness(&self) -> SelfContactPreparedCacheFreshness2 {
        self.freshness
    }

    /// Returns prepared source segment count.
    pub const fn prepared_segment_count(&self) -> usize {
        self.prepared_segment_count
    }

    /// Returns primitive-family counts for prepared source segments.
    pub const fn prepared_segment_kind_counts(&self) -> SegmentKindCounts {
        self.prepared_segment_kind_counts
    }

    /// Returns source segment boxes that were decided during preparation.
    pub const fn decided_segment_box_count(&self) -> usize {
        self.decided_segment_box_count
    }

    /// Returns source segment boxes that stayed undecided during preparation.
    pub const fn undecided_segment_box_count(&self) -> usize {
        self.undecided_segment_box_count
    }

    /// Returns whether the whole source path box was decided during preparation.
    pub const fn path_box_decided(&self) -> bool {
        self.path_box_decided
    }
}

fn self_contact_segment_kind_counts(segments: &[Segment2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        match segment.structural_facts().kind {
            SegmentKind::Line => counts.lines += 1,
            SegmentKind::Arc => counts.arcs += 1,
        }
    }
    counts
}

fn connected_segments_vertex(
    segments: &[Segment2],
    first: usize,
    second: usize,
    closed: bool,
) -> Option<&Point2> {
    if first + 1 == second {
        return Some(segments[first].end());
    }

    if closed && first == 0 && second + 1 == segments.len() {
        return Some(segments[first].start());
    }

    None
}

fn segment_relation_has_contact(
    relation: &SegmentIntersection,
    connectivity_point: Option<&Point2>,
    policy: &CurvePolicy,
) -> Classification<bool> {
    match relation {
        SegmentIntersection::LineLine(result) => {
            line_line_has_contact(result, connectivity_point, policy)
        }
        SegmentIntersection::LineArc { result, .. } => {
            line_arc_has_contact(result, connectivity_point, policy)
        }
        SegmentIntersection::ArcArc(result) => {
            arc_arc_has_contact(result, connectivity_point, policy)
        }
    }
}

fn line_line_has_contact(
    result: &LineLineIntersection,
    connectivity_point: Option<&Point2>,
    policy: &CurvePolicy,
) -> Classification<bool> {
    match result {
        LineLineIntersection::None => Classification::Decided(false),
        LineLineIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineLineIntersection::Point { point, .. } => {
            Classification::Decided(!point_is_connectivity(point, connectivity_point, policy))
        }
        LineLineIntersection::Overlap { .. } => Classification::Decided(true),
    }
}

fn line_arc_has_contact(
    result: &LineArcIntersection,
    connectivity_point: Option<&Point2>,
    policy: &CurvePolicy,
) -> Classification<bool> {
    match result {
        LineArcIntersection::None => Classification::Decided(false),
        LineArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineArcIntersection::Point(hit) => Classification::Decided(!point_is_connectivity(
            &hit.point,
            connectivity_point,
            policy,
        )),
        LineArcIntersection::TwoPoints { first, second } => {
            let first_is_connectivity =
                point_is_connectivity(&first.point, connectivity_point, policy);
            let second_is_connectivity =
                point_is_connectivity(&second.point, connectivity_point, policy);
            Classification::Decided(!(first_is_connectivity && second_is_connectivity))
        }
    }
}

fn arc_arc_has_contact(
    result: &ArcArcIntersection,
    connectivity_point: Option<&Point2>,
    policy: &CurvePolicy,
) -> Classification<bool> {
    match result {
        ArcArcIntersection::None => Classification::Decided(false),
        ArcArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        ArcArcIntersection::Point(hit) => Classification::Decided(!point_is_connectivity(
            &hit.point,
            connectivity_point,
            policy,
        )),
        ArcArcIntersection::TwoPoints { first, second } => {
            let first_is_connectivity =
                point_is_connectivity(&first.point, connectivity_point, policy);
            let second_is_connectivity =
                point_is_connectivity(&second.point, connectivity_point, policy);
            Classification::Decided(!(first_is_connectivity && second_is_connectivity))
        }
        ArcArcIntersection::Overlap { .. } => Classification::Decided(true),
    }
}

fn point_is_connectivity(
    point: &Point2,
    connectivity_point: Option<&Point2>,
    policy: &CurvePolicy,
) -> bool {
    let Some(connectivity_point) = connectivity_point else {
        return false;
    };

    let distance = point.distance_squared(connectivity_point);
    if is_zero(&distance, policy) == Some(true) {
        return true;
    }

    if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
        && let (Some(distance), Some(tolerance)) = (distance.to_f64_lossy(), policy.tolerance)
    {
        let tolerance = tolerance.absolute.max(tolerance.relative);
        return distance.is_finite() && distance <= tolerance * tolerance;
    }

    false
}
