//! Self-contact detection for curve strings and closed contours.

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::is_zero;
use crate::{
    ArcArcIntersection, Classification, Contour2, CurvePolicy, CurveResult, CurveString2,
    LineArcIntersection, LineLineIntersection, Point2, Segment2, SegmentIntersection,
};

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
        let boxes: Vec<_> = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        segments_have_self_contacts_with_cached_aabbs(self.segments(), &boxes, false, policy)
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
        let boxes: Vec<_> = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        segments_have_self_contacts_with_cached_aabbs(self.segments(), &boxes, true, policy)
    }
}

pub(crate) fn segments_have_self_contacts_with_cached_aabbs(
    segments: &[Segment2],
    boxes: &[Option<Aabb2>],
    closed: bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
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
            let connectivity_point =
                connected_segments_vertex(segments, first_index, second_index, closed);
            match segment_relation_has_contact(&relation, connectivity_point, policy) {
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
        && let (Some(distance), Some(tolerance)) = (distance.to_f64_approx(), policy.tolerance)
    {
        let tolerance = tolerance.absolute.max(tolerance.relative);
        return distance.is_finite() && distance <= tolerance * tolerance;
    }

    false
}
