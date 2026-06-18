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
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    first_contact_first_segment_index: Option<usize>,
    first_contact_second_segment_index: Option<usize>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing self-contact classification.
#[derive(Clone, Debug, PartialEq)]
pub struct SelfContactResult2 {
    has_self_contacts: Classification<bool>,
    report: SelfContactReport2,
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
    /// axis-aligned bounding-box broad phase. Bentley and Ottmann, "Algorithms
    /// for Reporting and Counting Geometric Intersections" (1979), is the
    /// standard reference for replacing the pair enumeration with a sweep-line
    /// candidate generator once offset trimming needs larger inputs.
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
    /// Bentley and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections" (1979), is the standard reference for that
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

    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
            candidate_pair_count += 1;
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
                            candidate_pair_count,
                            skipped_aabb_pair_count,
                            tested_pair_count,
                            first_contact_first_segment_index: Some(first_index),
                            first_contact_second_segment_index: Some(second_index),
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
                            candidate_pair_count,
                            skipped_aabb_pair_count,
                            tested_pair_count,
                            first_contact_first_segment_index: None,
                            first_contact_second_segment_index: None,
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
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            first_contact_first_segment_index: None,
            first_contact_second_segment_index: None,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    })
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

    /// Returns retained scan evidence for self-contact classification.
    pub const fn report(&self) -> &SelfContactReport2 {
        &self.report
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
