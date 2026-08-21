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
            let connectivity_vertices =
                connected_segments_vertices(segments, first_index, second_index, closed);
            match segment_relation_has_contact(&relation, connectivity_vertices, policy) {
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

#[derive(Clone, Copy)]
struct ConnectivityVertices<'a> {
    // A closed two-segment contour has the same pair on both sides of the
    // cyclic seam, so both of its shared endpoints are adjacency vertices.
    first: &'a Point2,
    second: Option<&'a Point2>,
}

fn connected_segments_vertices(
    segments: &[Segment2],
    first: usize,
    second: usize,
    closed: bool,
) -> Option<ConnectivityVertices<'_>> {
    if first + 1 == second {
        return Some(ConnectivityVertices {
            first: segments[first].end(),
            second: (closed && first == 0 && second + 1 == segments.len())
                .then(|| segments[first].start()),
        });
    }

    if closed && first == 0 && second + 1 == segments.len() {
        return Some(ConnectivityVertices {
            first: segments[first].start(),
            second: None,
        });
    }

    None
}

fn segment_relation_has_contact(
    relation: &SegmentIntersection,
    connectivity_vertices: Option<ConnectivityVertices<'_>>,
    policy: &CurveContext,
) -> Classification<bool> {
    match relation {
        SegmentIntersection::LineLine(result) => {
            line_line_has_contact(result, connectivity_vertices, policy)
        }
        SegmentIntersection::LineArc { result, .. } => {
            line_arc_has_contact(result, connectivity_vertices, policy)
        }
        SegmentIntersection::ArcArc(result) => {
            arc_arc_has_contact(result, connectivity_vertices, policy)
        }
    }
}

fn line_line_has_contact(
    result: &LineLineIntersection,
    connectivity_vertices: Option<ConnectivityVertices<'_>>,
    policy: &CurveContext,
) -> Classification<bool> {
    match result {
        LineLineIntersection::None => Classification::Decided(false),
        LineLineIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineLineIntersection::Point { point, .. } => {
            contact_is_non_connectivity(point, connectivity_vertices, policy)
        }
        LineLineIntersection::Overlap { .. } => Classification::Decided(true),
    }
}

fn line_arc_has_contact(
    result: &LineArcIntersection,
    connectivity_vertices: Option<ConnectivityVertices<'_>>,
    policy: &CurveContext,
) -> Classification<bool> {
    match result {
        LineArcIntersection::None => Classification::Decided(false),
        LineArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        LineArcIntersection::Point(hit) => {
            contact_is_non_connectivity(&hit.point, connectivity_vertices, policy)
        }
        LineArcIntersection::TwoPoints { first, second } => either_contact_is_non_connectivity(
            contact_is_non_connectivity(&first.point, connectivity_vertices, policy),
            contact_is_non_connectivity(&second.point, connectivity_vertices, policy),
        ),
    }
}

fn arc_arc_has_contact(
    result: &ArcArcIntersection,
    connectivity_vertices: Option<ConnectivityVertices<'_>>,
    policy: &CurveContext,
) -> Classification<bool> {
    match result {
        ArcArcIntersection::None => Classification::Decided(false),
        ArcArcIntersection::Uncertain { reason } => Classification::Uncertain(*reason),
        ArcArcIntersection::Point(hit) => {
            contact_is_non_connectivity(&hit.point, connectivity_vertices, policy)
        }
        ArcArcIntersection::TwoPoints { first, second } => either_contact_is_non_connectivity(
            contact_is_non_connectivity(&first.point, connectivity_vertices, policy),
            contact_is_non_connectivity(&second.point, connectivity_vertices, policy),
        ),
        ArcArcIntersection::Overlap { .. } => Classification::Decided(true),
    }
}

fn contact_is_non_connectivity(
    point: &Point2,
    connectivity_vertices: Option<ConnectivityVertices<'_>>,
    policy: &CurveContext,
) -> Classification<bool> {
    let Some(connectivity_vertices) = connectivity_vertices else {
        return Classification::Decided(true);
    };

    let mut uncertainty = None;
    for connectivity_point in [
        Some(connectivity_vertices.first),
        connectivity_vertices.second,
    ]
    .into_iter()
    .flatten()
    {
        match point_matches_connectivity(point, connectivity_point, policy) {
            Classification::Decided(true) => return Classification::Decided(false),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => uncertainty = Some(reason),
        }
    }
    uncertainty.map_or(Classification::Decided(true), Classification::Uncertain)
}

fn point_matches_connectivity(
    point: &Point2,
    connectivity_point: &Point2,
    policy: &CurveContext,
) -> Classification<bool> {
    let distance = point.distance_squared(connectivity_point);
    match is_zero(&distance, policy) {
        Some(equal) => return Classification::Decided(equal),
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
            return Classification::Decided(distance <= tolerance * tolerance);
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
    use crate::{
        CircularArc2, Classification, Contour2, CurveContext, Point2, Segment2, UncertaintyReason,
    };
    use hyperreal::Real;

    fn point(x: i8, y: i8) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

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

    #[test]
    fn two_segment_closed_contour_accepts_both_connectivity_vertices() {
        let right = point(1, 0);
        let left = point(-1, 0);
        let center = point(0, 0);
        let contour = Contour2::try_new(vec![
            Segment2::Arc(
                CircularArc2::try_from_center(right.clone(), left.clone(), center.clone(), false)
                    .unwrap(),
            ),
            Segment2::Arc(CircularArc2::try_from_center(left, right, center, false).unwrap()),
        ])
        .unwrap();

        assert_eq!(
            contour.has_self_contacts(&CurveContext::STRICT).unwrap(),
            Classification::Decided(false),
        );
    }
}
