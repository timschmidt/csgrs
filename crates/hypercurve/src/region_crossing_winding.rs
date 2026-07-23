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
    ContourIntersection, CurvePolicy, IntersectionKind, Point2, RegionContourKey,
    RegionContourRole, RegionIntersectionSet, RegionSide, RegionView2, Segment2, SegmentKind,
};

#[derive(Clone, Debug)]
pub(crate) struct RegionLineCrossing<'a> {
    pub(crate) segment_index: usize,
    pub(crate) parameter: &'a Real,
    pub(crate) point: &'a Point2,
    pub(crate) winding_delta: i32,
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
                point: &point.point,
                winding_delta: first_delta,
            });
            index.second.push(RegionLineCrossing {
                segment_index: point.b_segment_index,
                parameter: &point.b_param,
                point: &point.point,
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

    pub(crate) fn delta_for_next_fragment(
        &self,
        key: RegionContourKey,
        previous_segment_index: usize,
        current_segment_index: usize,
        segment_transition_index: &mut usize,
    ) -> Option<i32> {
        if previous_segment_index != current_segment_index {
            *segment_transition_index = 0;
            return Some(0);
        }
        // Compact fragments retain source order, and every same-segment
        // boundary corresponds one-to-one with the next certified crossing.
        let crossing = self
            .crossings_for_segment(key, previous_segment_index)?
            .get(*segment_transition_index)?;
        *segment_transition_index += 1;
        Some(crossing.winding_delta)
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

    pub(crate) fn crossings_for_segment(
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
    // A lossy preview is only an ordering hint. The exact adjacent check below
    // certifies the candidate order; ambiguity falls back to an all-exact sort.
    let preview = crossings
        .iter()
        .map(|crossing| {
            crossing
                .parameter
                .to_f64_lossy()
                .filter(|value| value.is_finite())
                .map(|parameter| (crossing.clone(), parameter))
        })
        .collect::<Option<Vec<_>>>();
    if let Some(mut preview) = preview {
        preview.sort_unstable_by(|(left, left_parameter), (right, right_parameter)| {
            left.segment_index
                .cmp(&right.segment_index)
                .then_with(|| left_parameter.total_cmp(right_parameter))
        });
        for (crossing, (ordered, _)) in crossings.iter_mut().zip(preview) {
            *crossing = ordered;
        }
        if crossing_order_is_certified(crossings, policy) {
            return true;
        }
    }

    let mut order_decided = true;
    crossings.sort_unstable_by(|left, right| {
        left.segment_index.cmp(&right.segment_index).then_with(|| {
            match compare_reals(left.parameter, right.parameter, policy) {
                Some(ordering) => ordering,
                None => {
                    order_decided = false;
                    std::cmp::Ordering::Equal
                }
            }
        })
    });
    order_decided && crossing_order_is_certified(crossings, policy)
}

fn crossing_order_is_certified(crossings: &[RegionLineCrossing<'_>], policy: &CurvePolicy) -> bool {
    crossings.windows(2).all(|window| {
        window[0].segment_index < window[1].segment_index
            || window[0].segment_index == window[1].segment_index
                && compare_reals(window[0].parameter, window[1].parameter, policy)
                    == Some(std::cmp::Ordering::Less)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn crossing<'a>(
        segment_index: usize,
        parameter: &'a Real,
        point: &'a Point2,
    ) -> RegionLineCrossing<'a> {
        RegionLineCrossing {
            segment_index,
            parameter,
            point,
            winding_delta: 1,
        }
    }

    #[test]
    fn lossy_crossing_order_is_exactly_certified_and_rejects_duplicates() {
        let policy = CurvePolicy::certified();
        let point = Point2::new(Real::zero(), Real::zero());
        let large = 1_i128 << 100;
        let lower = Real::from(large);
        let upper = Real::from(large + 1);
        assert_eq!(lower.to_f64_lossy(), upper.to_f64_lossy());

        let mut crossings = vec![
            crossing(1, &upper, &point),
            crossing(0, &upper, &point),
            crossing(1, &lower, &point),
        ];
        assert!(sort_and_validate_unique(&mut crossings, &policy));
        assert_eq!(crossings[0].segment_index, 0);
        assert_eq!(crossings[1].parameter, &lower);
        assert_eq!(crossings[2].parameter, &upper);

        let mut duplicates = vec![crossing(0, &lower, &point), crossing(0, &lower, &point)];
        assert!(!sort_and_validate_unique(&mut duplicates, &policy));
    }
}
