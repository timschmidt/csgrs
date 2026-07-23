//! Retained winding transitions for strict line crossings.
//!
//! A complete, unique set of proper line crossings can carry more topology
//! than split markers alone: crossing an oriented opposite edge changes its
//! exact winding number by the sign of the two traversal directions. This
//! module validates that narrow proof and maps it back onto materialized
//! contour fragments. Any endpoint, tangent, arc, overlap, duplicate,
//! unresolved ordering, or non-closing transition set rejects the proof.

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, real_sign};
use crate::{
    ContourIntersection, CurvePolicy, IntersectionKind, RegionContourKey, RegionContourRole,
    RegionIntersectionSet, RegionSide, RegionView2, Segment2, SegmentKind,
};

#[derive(Clone, Debug)]
struct RegionLineCrossing<'a> {
    segment_index: usize,
    parameter: &'a Real,
    winding_delta: i32,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct RegionLineCrossingWindingIndex<'a> {
    first: Vec<RegionLineCrossing<'a>>,
    second: Vec<RegionLineCrossing<'a>>,
    first_segment_offsets: Vec<usize>,
    second_segment_offsets: Vec<usize>,
}

impl<'a> RegionLineCrossingWindingIndex<'a> {
    pub(crate) fn event_set_may_support_propagation(intersections: &RegionIntersectionSet) -> bool {
        intersections.point_event_count() != 0
    }

    pub(crate) fn from_intersections(
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        intersections: &'a RegionIntersectionSet,
        policy: &CurvePolicy,
    ) -> Option<Self> {
        if first.material_contours().len() != 1
            || second.material_contours().len() != 1
            || !first.hole_contours().is_empty()
            || !second.hole_contours().is_empty()
            || intersections.pairs().len() != 1
        {
            return None;
        }

        let pair = &intersections.pairs()[0];
        if pair.first() != RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0)
            || pair.second()
                != RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0)
            || pair.intersections().is_empty()
        {
            return None;
        }

        let first_contour = first.material_contours()[0];
        let second_contour = second.material_contours()[0];
        let crossing_capacity = pair.intersections().len();
        let mut index = Self {
            first: Vec::with_capacity(crossing_capacity),
            second: Vec::with_capacity(crossing_capacity),
            first_segment_offsets: Vec::new(),
            second_segment_offsets: Vec::new(),
        };
        let mut crossing_count = 0_usize;
        for (event_index, event) in pair.intersections().events().iter().enumerate() {
            let ContourIntersection::Point(point) = event else {
                return None;
            };
            // The normalized crossing kind already certifies strict interior parameters.
            if point.kind != IntersectionKind::Crossing
                || point.a_segment_kind != SegmentKind::Line
                || point.b_segment_kind != SegmentKind::Line
            {
                return None;
            }

            // As a point follows the first contour, crossing from the right of
            // the oriented second edge to its left raises the second contour's
            // winding by one; the reverse crossing lowers it. Swapping source
            // and opposite traversal negates the same determinant.
            let first_delta = match pair
                .intersections()
                .certified_line_crossing_delta(event_index)
            {
                Some(delta) => delta,
                None => {
                    let Some(Segment2::Line(first_line)) =
                        first_contour.segments().get(point.a_segment_index)
                    else {
                        return None;
                    };
                    let Some(Segment2::Line(second_line)) =
                        second_contour.segments().get(point.b_segment_index)
                    else {
                        return None;
                    };
                    if let Some(delta) = crate::intersect::certified_line_crossing_winding_delta(
                        first_line,
                        second_line,
                    ) {
                        delta
                    } else {
                        let (first_dx, first_dy) = first_line.delta();
                        let (second_dx, second_dy) = second_line.delta();
                        let determinant =
                            Real::diff_of_products(&second_dx, &first_dy, &second_dy, &first_dx);
                        match real_sign(&determinant, policy) {
                            Some(RealSign::Positive) => 1,
                            Some(RealSign::Negative) => -1,
                            Some(RealSign::Zero) | None => return None,
                        }
                    }
                }
            };

            index.first.push(RegionLineCrossing {
                segment_index: point.a_segment_index,
                parameter: &point.a_param,
                winding_delta: first_delta,
            });
            index.second.push(RegionLineCrossing {
                segment_index: point.b_segment_index,
                parameter: &point.b_param,
                winding_delta: -first_delta,
            });
            crossing_count += 1;
        }

        if !sort_and_validate_unique(&mut index.first, policy)
            || !sort_and_validate_unique(&mut index.second, policy)
        {
            return None;
        }
        index.first_segment_offsets = segment_crossing_offsets(&index.first, first_contour.len())?;
        index.second_segment_offsets =
            segment_crossing_offsets(&index.second, second_contour.len())?;

        (crossing_count != 0
            && index.crossing_count(pair.first()) == crossing_count
            && index.crossing_count(pair.second()) == crossing_count
            && index.winding_delta_sum(pair.first()) == 0
            && index.winding_delta_sum(pair.second()) == 0)
            .then_some(index)
    }

    pub(crate) fn crossing_count(&self, key: RegionContourKey) -> usize {
        self.crossings_for_key(key)
            .map_or(0, <[RegionLineCrossing]>::len)
    }

    fn winding_delta_sum(&self, key: RegionContourKey) -> i64 {
        self.crossings_for_key(key).map_or(0, |crossings| {
            crossings
                .iter()
                .map(|crossing| i64::from(crossing.winding_delta))
                .sum()
        })
    }

    pub(crate) fn delta_between_fragments(
        &self,
        key: RegionContourKey,
        previous_segment_index: usize,
        previous_end: &Real,
        current_segment_index: usize,
        current_start: &Real,
    ) -> Option<i32> {
        if previous_segment_index != current_segment_index {
            return Some(0);
        }
        if !std::ptr::eq(previous_end, current_start) && previous_end != current_start {
            return None;
        }
        let crossings = self.crossings(key, previous_segment_index)?;
        if let [crossing] = crossings {
            return Some(crossing.winding_delta);
        }
        let mut matched = crossings
            .iter()
            .filter(|crossing| crossing.parameter == previous_end);
        let delta = matched.next()?.winding_delta;
        matched.next().is_none().then_some(delta)
    }

    fn crossings_for_key(&self, key: RegionContourKey) -> Option<&[RegionLineCrossing<'a>]> {
        if key.role != RegionContourRole::Material || key.index != 0 {
            return None;
        }
        Some(match key.side {
            RegionSide::First => &self.first,
            RegionSide::Second => &self.second,
        })
    }

    fn crossings(
        &self,
        key: RegionContourKey,
        segment_index: usize,
    ) -> Option<&[RegionLineCrossing<'a>]> {
        let crossings = self.crossings_for_key(key)?;
        let offsets = match key.side {
            RegionSide::First => &self.first_segment_offsets,
            RegionSide::Second => &self.second_segment_offsets,
        };
        let start = *offsets.get(segment_index)?;
        let end = *offsets.get(segment_index + 1)?;
        Some(&crossings[start..end])
    }
}

fn segment_crossing_offsets(
    crossings: &[RegionLineCrossing<'_>],
    segment_count: usize,
) -> Option<Vec<usize>> {
    let mut offsets = Vec::with_capacity(segment_count + 1);
    let mut crossing_index = 0;
    for segment_index in 0..segment_count {
        offsets.push(crossing_index);
        while crossings
            .get(crossing_index)
            .is_some_and(|crossing| crossing.segment_index == segment_index)
        {
            crossing_index += 1;
        }
    }
    offsets.push(crossing_index);
    (crossing_index == crossings.len()).then_some(offsets)
}

fn sort_and_validate_unique(
    crossings: &mut [RegionLineCrossing<'_>],
    policy: &CurvePolicy,
) -> bool {
    crossings.sort_unstable_by_key(|crossing| crossing.segment_index);
    let mut group_start = 0;
    while group_start < crossings.len() {
        let segment_index = crossings[group_start].segment_index;
        let group_end = crossings[group_start..]
            .partition_point(|crossing| crossing.segment_index == segment_index)
            + group_start;
        for (offset, crossing) in crossings[group_start..group_end].iter().enumerate() {
            if crossings[group_start + offset + 1..group_end]
                .iter()
                .any(|other| {
                    !matches!(
                        compare_reals(crossing.parameter, other.parameter, policy),
                        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater)
                    )
                })
            {
                return false;
            }
        }
        group_start = group_end;
    }
    true
}
