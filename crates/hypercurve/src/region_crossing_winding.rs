//! Retained winding transitions for strict line crossings.
//!
//! A complete, unique set of proper line crossings can carry more topology
//! than split markers alone: crossing an oriented opposite edge changes its
//! exact winding number by the sign of the two traversal directions. This
//! module validates that narrow proof and maps it back onto materialized
//! contour fragments. Any endpoint, tangent, arc, overlap, duplicate,
//! unresolved ordering, or non-closing transition set rejects the proof.

use std::collections::BTreeMap;

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, real_sign};
use crate::{
    ContourFragment, ContourIntersection, CurvePolicy, IntersectionKind, RegionContourFragments,
    RegionContourKey, RegionContourRole, RegionIntersectionSet, RegionSide, RegionView2, Segment2,
    SegmentKind,
};

#[derive(Clone, Debug)]
struct RegionLineCrossing {
    parameter: Real,
    winding_delta: i32,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct RegionLineCrossingWindingIndex {
    crossings: BTreeMap<(RegionContourKey, usize), Vec<RegionLineCrossing>>,
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
        let mut index = Self::default();
        let mut crossing_count = 0_usize;
        for event in pair.intersections().events() {
            let ContourIntersection::Point(point) = event else {
                return None;
            };
            if point.kind != IntersectionKind::Crossing
                || point.a_segment_kind != SegmentKind::Line
                || point.b_segment_kind != SegmentKind::Line
                || !strict_unit_parameter(&point.a_param, policy)
                || !strict_unit_parameter(&point.b_param, policy)
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
            let (first_dx, first_dy) = first_line.delta();
            let (second_dx, second_dy) = second_line.delta();
            // As a point follows the first contour, crossing from the right of
            // the oriented second edge to its left raises the second contour's
            // winding by one; the reverse crossing lowers it. Swapping source
            // and opposite traversal negates the same determinant.
            let determinant = Real::diff_of_products(&second_dx, &first_dy, &second_dy, &first_dx);
            let first_delta = match real_sign(&determinant, policy) {
                Some(RealSign::Positive) => 1,
                Some(RealSign::Negative) => -1,
                Some(RealSign::Zero) | None => return None,
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
        let entries = self.crossings.entry((key, segment_index)).or_default();
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

    fn crossing_count(&self, key: RegionContourKey) -> usize {
        self.crossings
            .iter()
            .filter(|((crossing_key, _), _)| *crossing_key == key)
            .map(|(_, crossings)| crossings.len())
            .sum()
    }

    fn winding_delta_sum(&self, key: RegionContourKey) -> i64 {
        self.crossings
            .iter()
            .filter(|((crossing_key, _), _)| *crossing_key == key)
            .flat_map(|(_, crossings)| crossings)
            .map(|crossing| i64::from(crossing.winding_delta))
            .sum()
    }

    pub(crate) fn delta_between_fragments(
        &self,
        key: RegionContourKey,
        previous: &ContourFragment,
        current: &ContourFragment,
        policy: &CurvePolicy,
    ) -> Option<i32> {
        if previous.source_segment_index != current.source_segment_index {
            return Some(0);
        }
        if compare_reals(
            previous.source_range.end(),
            current.source_range.start(),
            policy,
        ) != Some(std::cmp::Ordering::Equal)
        {
            return None;
        }
        let crossings = self.crossings.get(&(key, previous.source_segment_index))?;
        let mut matched = crossings.iter().filter(|crossing| {
            compare_reals(&crossing.parameter, previous.source_range.end(), policy)
                == Some(std::cmp::Ordering::Equal)
        });
        let delta = matched.next()?.winding_delta;
        matched.next().is_none().then_some(delta)
    }

    pub(crate) fn certifies_fragments(
        &self,
        contour_fragments: &RegionContourFragments,
        policy: &CurvePolicy,
    ) -> bool {
        let fragments = contour_fragments.fragments.fragments();
        if fragments.is_empty() {
            return false;
        }
        let mut represented_crossings = 0_usize;
        for pair in fragments.windows(2) {
            if pair[0].source_segment_index == pair[1].source_segment_index {
                if self
                    .delta_between_fragments(contour_fragments.key, &pair[0], &pair[1], policy)
                    .is_none()
                {
                    return false;
                }
                represented_crossings += 1;
            }
        }
        represented_crossings == self.crossing_count(contour_fragments.key)
    }
}

fn strict_unit_parameter(parameter: &Real, policy: &CurvePolicy) -> bool {
    compare_reals(parameter, &Real::zero(), policy) == Some(std::cmp::Ordering::Greater)
        && compare_reals(parameter, &Real::one(), policy) == Some(std::cmp::Ordering::Less)
}
