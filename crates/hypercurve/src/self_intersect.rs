//! Self-contact detection for curve strings and closed contours.

use hyperlattice::Backend;

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::{
    ArcArcIntersection, Classification, Contour2, CurvePolicy, CurveResult, CurveString2,
    LineArcIntersection, LineLineIntersection, Segment2, SegmentIntersection,
};

impl<B: Backend> CurveString2<B> {
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
        let boxes: Vec<_> = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        segments_have_self_contacts_with_cached_aabbs(self.segments(), &boxes, false, policy)
    }
}

impl<B: Backend> Contour2<B> {
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
        let boxes: Vec<_> = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        segments_have_self_contacts_with_cached_aabbs(self.segments(), &boxes, true, policy)
    }
}

pub(crate) fn segments_have_self_contacts_with_cached_aabbs<B: Backend>(
    segments: &[Segment2<B>],
    boxes: &[Option<Aabb2<B>>],
    closed: bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
            if segments_are_adjacent(first_index, second_index, segments.len(), closed) {
                continue;
            }

            // The broad phase is allowed to skip only when non-overlap is
            // decided. If a box or coordinate ordering is uncertain, exact
            // segment topology below remains authoritative.
            if let (Some(Some(first_box)), Some(Some(second_box))) =
                (boxes.get(first_index), boxes.get(second_index))
            {
                if aabbs_decided_disjoint(first_box, second_box, policy) {
                    continue;
                }
            }

            let relation =
                segments[first_index].intersect_segment(&segments[second_index], policy)?;
            match segment_relation_has_contact(&relation) {
                Classification::Decided(true) => return Ok(Classification::Decided(true)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    Ok(Classification::Decided(false))
}

fn segments_are_adjacent(first: usize, second: usize, len: usize, closed: bool) -> bool {
    first.abs_diff(second) == 1 || (closed && first == 0 && second + 1 == len)
}

fn segment_relation_has_contact<B: Backend>(
    relation: &SegmentIntersection<B>,
) -> Classification<bool> {
    match relation {
        SegmentIntersection::LineLine(result) => line_line_has_contact(result),
        SegmentIntersection::LineArc { result, .. } => line_arc_has_contact(result),
        SegmentIntersection::ArcArc(result) => arc_arc_has_contact(result),
    }
}

fn line_line_has_contact<B: Backend>(result: &LineLineIntersection<B>) -> Classification<bool> {
    match result {
        LineLineIntersection::None => Classification::Decided(false),
        LineLineIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineLineIntersection::Point { .. } | LineLineIntersection::Overlap { .. } => {
            Classification::Decided(true)
        }
    }
}

fn line_arc_has_contact<B: Backend>(result: &LineArcIntersection<B>) -> Classification<bool> {
    match result {
        LineArcIntersection::None => Classification::Decided(false),
        LineArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineArcIntersection::Point(_) | LineArcIntersection::TwoPoints { .. } => {
            Classification::Decided(true)
        }
    }
}

fn arc_arc_has_contact<B: Backend>(result: &ArcArcIntersection<B>) -> Classification<bool> {
    match result {
        ArcArcIntersection::None => Classification::Decided(false),
        ArcArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        ArcArcIntersection::Point(_)
        | ArcArcIntersection::TwoPoints { .. }
        | ArcArcIntersection::Overlap { .. } => Classification::Decided(true),
    }
}
