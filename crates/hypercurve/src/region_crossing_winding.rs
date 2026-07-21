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
    ContourFragment, ContourIntersection, CurvePolicy, IntersectionKind, RegionContourKey,
    RegionContourRole, RegionIntersectionSet, RegionSide, RegionView2, Segment2, SegmentKind,
};

#[derive(Clone, Debug)]
struct RegionLineCrossing {
    parameter: Real,
    winding_delta: i32,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct RegionLineCrossingWindingIndex {
    first: Vec<Vec<RegionLineCrossing>>,
    second: Vec<Vec<RegionLineCrossing>>,
}

impl RegionLineCrossingWindingIndex {
    pub(crate) fn event_set_may_support_propagation(intersections: &RegionIntersectionSet) -> bool {
        intersections.point_event_count() != 0
    }

    pub(crate) fn from_intersections(
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        intersections: &RegionIntersectionSet,
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
        let mut index = Self {
            first: (0..first_contour.len()).map(|_| Vec::new()).collect(),
            second: (0..second_contour.len()).map(|_| Vec::new()).collect(),
        };
        let mut crossing_count = 0_usize;
        for event in pair.intersections().events() {
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
            // As a point follows the first contour, crossing from the right of
            // the oriented second edge to its left raises the second contour's
            // winding by one; the reverse crossing lowers it. Swapping source
            // and opposite traversal negates the same determinant.
            let first_delta = match crate::intersect::certified_line_segment_support_relation(
                first_line,
                second_line,
            )
            .crossing_winding_delta()
            {
                Some(delta) => delta,
                None => {
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
            };

            if !index.insert_unique(
                pair.first(),
                point.a_segment_index,
                point.a_param.clone(),
                first_delta,
                policy,
            ) || !index.insert_unique(
                pair.second(),
                point.b_segment_index,
                point.b_param.clone(),
                -first_delta,
                policy,
            ) {
                return None;
            }
            crossing_count += 1;
        }

        (crossing_count != 0
            && index.crossing_count(pair.first()) == crossing_count
            && index.crossing_count(pair.second()) == crossing_count
            && index.winding_delta_sum(pair.first()) == 0
            && index.winding_delta_sum(pair.second()) == 0)
            .then_some(index)
    }

    fn insert_unique(
        &mut self,
        key: RegionContourKey,
        segment_index: usize,
        parameter: Real,
        winding_delta: i32,
        policy: &CurvePolicy,
    ) -> bool {
        let Some(entries) = self.crossings_mut(key, segment_index) else {
            return false;
        };
        if entries.iter().any(|existing| {
            !matches!(
                compare_reals(&existing.parameter, &parameter, policy),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater)
            )
        }) {
            return false;
        }
        entries.push(RegionLineCrossing {
            parameter,
            winding_delta,
        });
        true
    }

    pub(crate) fn crossing_count(&self, key: RegionContourKey) -> usize {
        self.crossings_for_key(key)
            .map_or(0, |crossings| crossings.iter().map(Vec::len).sum())
    }

    fn winding_delta_sum(&self, key: RegionContourKey) -> i64 {
        self.crossings_for_key(key).map_or(0, |crossings| {
            crossings
                .iter()
                .flatten()
                .map(|crossing| i64::from(crossing.winding_delta))
                .sum()
        })
    }

    pub(crate) fn delta_between_fragments(
        &self,
        key: RegionContourKey,
        previous: &ContourFragment,
        current: &ContourFragment,
    ) -> Option<i32> {
        if previous.source_segment_index != current.source_segment_index {
            return Some(0);
        }
        if previous.source_range.end() != current.source_range.start() {
            return None;
        }
        let crossings = self.crossings(key, previous.source_segment_index)?;
        let mut matched = crossings
            .iter()
            .filter(|crossing| &crossing.parameter == previous.source_range.end());
        let delta = matched.next()?.winding_delta;
        matched.next().is_none().then_some(delta)
    }

    fn crossings_for_key(&self, key: RegionContourKey) -> Option<&[Vec<RegionLineCrossing>]> {
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
    ) -> Option<&[RegionLineCrossing]> {
        self.crossings_for_key(key)?
            .get(segment_index)
            .map(Vec::as_slice)
    }

    fn crossings_mut(
        &mut self,
        key: RegionContourKey,
        segment_index: usize,
    ) -> Option<&mut Vec<RegionLineCrossing>> {
        if key.role != RegionContourRole::Material || key.index != 0 {
            return None;
        }
        match key.side {
            RegionSide::First => self.first.get_mut(segment_index),
            RegionSide::Second => self.second.get_mut(segment_index),
        }
    }
}
