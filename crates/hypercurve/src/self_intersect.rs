//! Self-contact detection for curve strings and closed contours.

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::is_zero;
use crate::{
    ArcArcIntersection, Classification, Contour2, CurveContext, CurveResult, CurveString2,
    LineArcIntersection, LineLineIntersection, Point2, Segment2, SegmentIntersection,
    UncertaintyReason,
};

impl CurveString2 {
    /// Classifies whether this open curve string has non-adjacent self contacts.
    pub fn has_self_contacts(&self, policy: &CurveContext) -> CurveResult<Classification<bool>> {
        let boxes = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect::<Vec<_>>();
        segments_have_self_contacts_with_cached_aabbs(self.segments(), &boxes, false, policy)
    }
}

impl Contour2 {
    /// Classifies whether this contour has non-adjacent self contacts.
    pub fn has_self_contacts(&self, policy: &CurveContext) -> CurveResult<Classification<bool>> {
        let boxes = self
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect::<Vec<_>>();
        segments_have_self_contacts_with_cached_aabbs(self.segments(), &boxes, true, policy)
    }
}

pub(crate) fn segments_have_self_contacts_with_cached_aabbs(
    segments: &[Segment2],
    boxes: &[Option<Aabb2>],
    closed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let x_overlap_schedule = self_contact_x_overlap_schedule(boxes, policy);

    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
            if x_overlap_schedule
                .as_ref()
                .is_some_and(|schedule| !schedule.overlaps(first_index, second_index))
            {
                continue;
            }
            if let (Some(Some(first_box)), Some(Some(second_box))) =
                (boxes.get(first_index), boxes.get(second_index))
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                continue;
            }

            let relation =
                segments[first_index].intersect_segment(&segments[second_index], policy)?;
            let connectivity_point =
                connected_segments_vertex(segments, first_index, second_index, closed);
            match segment_relation_has_contact(&relation, connectivity_point, policy) {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(true));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    Ok(Classification::Decided(false))
}

fn self_contact_x_overlap_schedule(
    boxes: &[Option<Aabb2>],
    _policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> Classification<bool> {
    match result {
        LineLineIntersection::None => Classification::Decided(false),
        LineLineIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineLineIntersection::Point { point, .. } => {
            contact_is_non_connectivity(point, connectivity_point, policy)
        }
        LineLineIntersection::Overlap { .. } => Classification::Decided(true),
    }
}

fn line_arc_has_contact(
    result: &LineArcIntersection,
    connectivity_point: Option<&Point2>,
    policy: &CurveContext,
) -> Classification<bool> {
    match result {
        LineArcIntersection::None => Classification::Decided(false),
        LineArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineArcIntersection::Point(hit) => {
            contact_is_non_connectivity(&hit.point, connectivity_point, policy)
        }
        LineArcIntersection::TwoPoints { first, second } => either_contact_is_non_connectivity(
            contact_is_non_connectivity(&first.point, connectivity_point, policy),
            contact_is_non_connectivity(&second.point, connectivity_point, policy),
        ),
    }
}

fn arc_arc_has_contact(
    result: &ArcArcIntersection,
    connectivity_point: Option<&Point2>,
    policy: &CurveContext,
) -> Classification<bool> {
    match result {
        ArcArcIntersection::None => Classification::Decided(false),
        ArcArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        ArcArcIntersection::Point(hit) => {
            contact_is_non_connectivity(&hit.point, connectivity_point, policy)
        }
        ArcArcIntersection::TwoPoints { first, second } => either_contact_is_non_connectivity(
            contact_is_non_connectivity(&first.point, connectivity_point, policy),
            contact_is_non_connectivity(&second.point, connectivity_point, policy),
        ),
        ArcArcIntersection::Overlap { .. } => Classification::Decided(true),
    }
}

fn contact_is_non_connectivity(
    point: &Point2,
    connectivity_point: Option<&Point2>,
    policy: &CurveContext,
) -> Classification<bool> {
    let Some(connectivity_point) = connectivity_point else {
        return Classification::Decided(true);
    };

    let distance = point.distance_squared(connectivity_point);
    match is_zero(&distance, policy) {
        Some(equal) => return Classification::Decided(!equal),
        None if !policy.is_edge_preview() => {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }
        None => {}
    }

    if let (Some(distance), Some(tolerance)) =
        (distance.to_f64_lossy(), crate::policy::preview_tolerance())
    {
        let tolerance = tolerance.absolute.max(tolerance.relative);
        if distance.is_finite() {
            return Classification::Decided(distance > tolerance * tolerance);
        }
    }

    Classification::Uncertain(UncertaintyReason::RealSign)
}

fn either_contact_is_non_connectivity(
    first: Classification<bool>,
    second: Classification<bool>,
) -> Classification<bool> {
    match (first, second) {
        (Classification::Decided(true), _) | (_, Classification::Decided(true)) => {
            Classification::Decided(true)
        }
        (Classification::Decided(false), Classification::Decided(false)) => {
            Classification::Decided(false)
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Classification::Uncertain(reason)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::either_contact_is_non_connectivity;
    use crate::{Classification, UncertaintyReason};

    #[test]
    fn unresolved_connectivity_is_not_relabelled_as_a_decided_contact() {
        assert_eq!(
            either_contact_is_non_connectivity(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided(false),
            ),
            Classification::Uncertain(UncertaintyReason::RealSign),
        );
        assert_eq!(
            either_contact_is_non_connectivity(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided(true),
            ),
            Classification::Decided(true),
        );
    }
}
