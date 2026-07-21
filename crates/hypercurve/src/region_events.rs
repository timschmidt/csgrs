//! Region-level intersection event collection.
//!
//! Region event collection lifts contour-pair events into material/hole keyed
//! operands. It keeps broad-phase pruning conservative for the same reason as
//! sweep-line scheduling intersection reporting work: candidate generation may
//! be optimized, but topology still depends on the exact segment relation.

use std::cell::OnceCell;
use std::collections::BTreeMap;

use hyperreal::Real;

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_contour_aabb, decided_segment_aabb};
use crate::classify::compare_reals;
use crate::{
    Classification, ContourIntersection, ContourIntersectionSet, ContourOperand, CurveError,
    CurvePolicy, CurveResult, RegionView2, SegmentKind, SegmentKindCounts,
};

/// Which region side a contour key belongs to.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum RegionSide {
    /// First region passed to the query.
    First,
    /// Second region passed to the query.
    Second,
}

/// Semantic role of a contour inside a region.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum RegionContourRole {
    /// Positive material contour.
    Material,
    /// Negative hole contour.
    Hole,
}

/// Identifies one contour inside a region-pair query.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct RegionContourKey {
    /// Region side.
    pub side: RegionSide,
    /// Contour role in that region.
    pub role: RegionContourRole,
    /// Index within the role bin.
    pub index: usize,
}

impl RegionContourKey {
    /// Constructs a contour key.
    pub const fn new(side: RegionSide, role: RegionContourRole, index: usize) -> Self {
        Self { side, role, index }
    }

    /// Returns the region side.
    pub const fn side(self) -> RegionSide {
        self.side
    }

    /// Returns the contour role in that region.
    pub const fn role(self) -> RegionContourRole {
        self.role
    }

    /// Returns the index within the role bin.
    pub const fn index(self) -> usize {
        self.index
    }
}

impl RegionContourIntersection {
    /// Returns the keyed contour in the first region.
    pub const fn first(&self) -> RegionContourKey {
        self.first
    }

    /// Returns the keyed contour in the second region.
    pub const fn second(&self) -> RegionContourKey {
        self.second
    }

    /// Returns normalized contour-level intersections for this contour pair.
    pub const fn intersections(&self) -> &ContourIntersectionSet {
        &self.intersections
    }
}

/// Intersections between two keyed contours from a region-pair query.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourIntersection {
    /// Contour in the first region.
    pub first: RegionContourKey,
    /// Contour in the second region.
    pub second: RegionContourKey,
    /// Normalized contour-level intersections for this pair.
    pub intersections: ContourIntersectionSet,
}

/// Normalized contour-pair intersections between two regions.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionIntersectionSet {
    pairs: Vec<RegionContourIntersection>,
    first_contour_count: Option<usize>,
    second_contour_count: Option<usize>,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct RegionPointEndpointContactIndex {
    vertex_masks: BTreeMap<(RegionContourKey, usize), u8>,
}

impl RegionPointEndpointContactIndex {
    const START: u8 = 1;
    const END: u8 = 2;

    pub(crate) fn from_intersections(
        intersections: &RegionIntersectionSet,
        policy: &CurvePolicy,
    ) -> Self {
        let mut index = Self::default();
        for pair in intersections.pairs() {
            for event in pair.intersections().events() {
                let ContourIntersection::Point(point) = event else {
                    continue;
                };
                // Normalized crossing and tangent kinds certify interior parameters.
                if point.kind != crate::IntersectionKind::Endpoint {
                    continue;
                }
                index.record(pair.first(), point.a_segment_index, &point.a_param, policy);
                index.record(pair.second(), point.b_segment_index, &point.b_param, policy);
            }
        }
        index
    }

    fn record(
        &mut self,
        key: RegionContourKey,
        segment_index: usize,
        parameter: &Real,
        policy: &CurvePolicy,
    ) {
        let mask = match compare_reals(parameter, &Real::zero(), policy) {
            Some(std::cmp::Ordering::Equal) => Self::START,
            None => Self::START | Self::END,
            Some(_) => match compare_reals(parameter, &Real::one(), policy) {
                Some(std::cmp::Ordering::Equal) => Self::END,
                None => Self::START | Self::END,
                Some(_) => 0,
            },
        };
        if mask != 0 {
            *self.vertex_masks.entry((key, segment_index)).or_default() |= mask;
        }
    }

    pub(crate) fn parameter_is_contact(
        &self,
        key: RegionContourKey,
        segment_index: usize,
        segment_count: usize,
        parameter: &Real,
        policy: &CurvePolicy,
    ) -> bool {
        match compare_reals(parameter, &Real::zero(), policy) {
            Some(std::cmp::Ordering::Equal) => {
                let previous = (segment_index + segment_count - 1) % segment_count;
                self.mask(key, segment_index) & Self::START != 0
                    || self.mask(key, previous) & Self::END != 0
            }
            None => true,
            Some(_) => match compare_reals(parameter, &Real::one(), policy) {
                Some(std::cmp::Ordering::Equal) => {
                    let next = (segment_index + 1) % segment_count;
                    self.mask(key, segment_index) & Self::END != 0
                        || self.mask(key, next) & Self::START != 0
                }
                None => true,
                Some(_) => true,
            },
        }
    }

    fn mask(&self, key: RegionContourKey, segment_index: usize) -> u8 {
        self.vertex_masks
            .get(&(key, segment_index))
            .copied()
            .unwrap_or(0)
    }
}

impl RegionIntersectionSet {
    /// Constructs a set from already-normalized region contour pairs.
    pub fn new(pairs: Vec<RegionContourIntersection>) -> CurveResult<Self> {
        let pair_count = pairs.len();
        Self::from_parts(pairs, None, None, pair_count, 0, pair_count)
    }

    pub(crate) fn from_parts(
        pairs: Vec<RegionContourIntersection>,
        first_contour_count: Option<usize>,
        second_contour_count: Option<usize>,
        candidate_pair_count: usize,
        skipped_aabb_pair_count: usize,
        tested_pair_count: usize,
    ) -> CurveResult<Self> {
        validate_region_intersection_pairs(&pairs)?;
        if candidate_pair_count != skipped_aabb_pair_count + tested_pair_count {
            return Err(CurveError::Topology(
                "region intersection workload counts must balance".into(),
            ));
        }
        if pairs.len() > tested_pair_count {
            return Err(CurveError::Topology(
                "region intersection event pairs cannot exceed tested contour pairs".into(),
            ));
        }
        if let (Some(first_count), Some(second_count)) = (first_contour_count, second_contour_count)
            && candidate_pair_count != first_count * second_count
        {
            return Err(CurveError::Topology(
                "region intersection candidate count must match operand contour counts".into(),
            ));
        }
        Ok(Self {
            pairs,
            first_contour_count,
            second_contour_count,
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
        })
    }

    /// Returns nonempty contour-pair event sets.
    pub fn pairs(&self) -> &[RegionContourIntersection] {
        &self.pairs
    }

    /// Consumes the set and returns contour-pair event sets.
    pub fn into_pairs(self) -> Vec<RegionContourIntersection> {
        self.pairs
    }

    /// Returns true when no contour-pair events were collected.
    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }

    /// Returns the first operand contour count when known for this event set.
    pub const fn first_contour_count(&self) -> Option<usize> {
        self.first_contour_count
    }

    /// Returns the second operand contour count when known for this event set.
    pub const fn second_contour_count(&self) -> Option<usize> {
        self.second_contour_count
    }

    /// Returns the number of contour pairs with events.
    pub fn len(&self) -> usize {
        self.pairs.len()
    }

    /// Returns all contour-pair candidates considered by the region broad phase.
    pub const fn candidate_pair_count(&self) -> usize {
        self.candidate_pair_count
    }

    /// Returns contour-pair candidates skipped by decided disjoint AABBs.
    pub const fn skipped_aabb_pair_count(&self) -> usize {
        self.skipped_aabb_pair_count
    }

    /// Returns contour-pair candidates that reached exact contour intersection.
    pub const fn tested_pair_count(&self) -> usize {
        self.tested_pair_count
    }

    /// Returns contour pairs with nonempty normalized intersection evidence.
    pub fn intersecting_pair_count(&self) -> usize {
        self.pairs.len()
    }

    /// Returns normalized contour-level events retained across all intersecting pairs.
    pub fn event_count(&self) -> usize {
        self.pairs.iter().map(|pair| pair.intersections.len()).sum()
    }

    /// Returns retained point events across all intersecting contour pairs.
    pub fn point_event_count(&self) -> usize {
        self.pairs
            .iter()
            .map(|pair| pair.intersections.point_event_count())
            .sum()
    }

    /// Returns retained overlap events across all intersecting contour pairs.
    pub fn overlap_event_count(&self) -> usize {
        self.pairs
            .iter()
            .map(|pair| pair.intersections.overlap_event_count())
            .sum()
    }

    /// Returns retained unresolved events across all intersecting contour pairs.
    pub fn uncertain_event_count(&self) -> usize {
        self.pairs
            .iter()
            .map(|pair| pair.intersections.uncertain_event_count())
            .sum()
    }

    /// Returns primitive families touched by retained first-region event segments.
    pub fn first_event_segment_kind_counts(&self) -> SegmentKindCounts {
        region_event_segment_kind_counts(self, ContourOperand::First)
    }

    /// Returns primitive families touched by retained second-region event segments.
    pub fn second_event_segment_kind_counts(&self) -> SegmentKindCounts {
        region_event_segment_kind_counts(self, ContourOperand::Second)
    }

    /// Returns true when at least one normalized contour-level event was retained.
    pub fn has_events(&self) -> bool {
        self.event_count() != 0
    }

    /// Returns contour-pair events touching a specific keyed contour.
    pub fn pairs_for_contour(
        &self,
        key: RegionContourKey,
    ) -> impl Iterator<Item = &RegionContourIntersection> {
        self.pairs
            .iter()
            .filter(move |pair| pair.first == key || pair.second == key)
    }

    /// Splits every contour in both region views at this event set.
    pub fn split_regions(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<crate::RegionFragmentSet>> {
        crate::region_fragments::split_region_views_at_intersections(first, second, self, policy)
    }

    /// Splits every contour in both region views at this event set and retains a report.
    pub fn split_regions_with_report(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::RegionFragmentBuildResult2> {
        crate::region_fragments::split_region_views_at_intersections_with_report(
            first, second, self, policy,
        )
    }
}

pub(crate) fn intersect_region_views(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<RegionIntersectionSet> {
    let mut pairs = Vec::new();
    let mut workload = RegionIntersectionWorkload::default();
    let first_material_boxes = contour_intersection_aabbs(first.material_contours(), policy);
    let first_hole_boxes = contour_intersection_aabbs(first.hole_contours(), policy);
    let second_material_boxes = contour_intersection_aabbs(second.material_contours(), policy);
    let second_hole_boxes = contour_intersection_aabbs(second.hole_contours(), policy);

    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.material_contours(),
        &first_material_boxes,
        RegionContourRole::Material,
        second.material_contours(),
        &second_material_boxes,
        RegionContourRole::Material,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.material_contours(),
        &first_material_boxes,
        RegionContourRole::Material,
        second.hole_contours(),
        &second_hole_boxes,
        RegionContourRole::Hole,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.hole_contours(),
        &first_hole_boxes,
        RegionContourRole::Hole,
        second.material_contours(),
        &second_material_boxes,
        RegionContourRole::Material,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.hole_contours(),
        &first_hole_boxes,
        RegionContourRole::Hole,
        second.hole_contours(),
        &second_hole_boxes,
        RegionContourRole::Hole,
        policy,
    )?;

    RegionIntersectionSet::from_parts(
        pairs,
        Some(first.material_contours().len() + first.hole_contours().len()),
        Some(second.material_contours().len() + second.hole_contours().len()),
        workload.candidate_pair_count,
        workload.skipped_aabb_pair_count,
        workload.tested_pair_count,
    )
}

struct ContourIntersectionAabbs {
    exact: Option<crate::contour::ExactDyadicLineAabbs>,
    contour: Option<Aabb2>,
    segments: OnceCell<Vec<Option<Aabb2>>>,
}

impl ContourIntersectionAabbs {
    fn is_disjoint(&self, other: &Self, policy: &CurvePolicy) -> bool {
        match (self.exact.as_ref(), other.exact.as_ref()) {
            (Some(first), Some(second)) => first.contour.is_disjoint(second.contour),
            _ => match (self.contour.as_ref(), other.contour.as_ref()) {
                (Some(first), Some(second)) => aabbs_decided_disjoint(first, second, policy),
                _ => false,
            },
        }
    }

    fn segments<'a>(
        &'a self,
        contour: &crate::Contour2,
        policy: &CurvePolicy,
    ) -> &'a [Option<Aabb2>] {
        self.segments.get_or_init(|| {
            contour
                .segments()
                .iter()
                .map(|segment| decided_segment_aabb(segment, policy))
                .collect()
        })
    }
}

fn contour_intersection_aabbs(
    contours: &[&crate::Contour2],
    policy: &CurvePolicy,
) -> Vec<ContourIntersectionAabbs> {
    contours
        .iter()
        .map(|contour| {
            let exact = contour.exact_dyadic_line_aabbs(policy);
            ContourIntersectionAabbs {
                contour: exact
                    .is_none()
                    .then(|| decided_contour_aabb(contour, policy))
                    .flatten(),
                exact,
                segments: OnceCell::new(),
            }
        })
        .collect()
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct RegionIntersectionWorkload {
    pub(crate) candidate_pair_count: usize,
    pub(crate) skipped_aabb_pair_count: usize,
    pub(crate) tested_pair_count: usize,
}

fn validate_region_intersection_pairs(pairs: &[RegionContourIntersection]) -> CurveResult<()> {
    let mut keys = Vec::with_capacity(pairs.len());
    for pair in pairs {
        if pair.first.side != RegionSide::First || pair.second.side != RegionSide::Second {
            return Err(CurveError::Topology(
                "region intersection pair must be keyed from first region to second region".into(),
            ));
        }
        if pair.intersections.is_empty() {
            return Err(CurveError::Topology(
                "region intersection pair must carry nonempty contour event evidence".into(),
            ));
        }
        keys.push((pair.first, pair.second));
    }

    keys.sort_unstable();
    if keys.windows(2).any(|window| window[0] == window[1]) {
        return Err(CurveError::Topology(
            "region intersection set must not contain duplicate contour pairs".into(),
        ));
    }
    Ok(())
}

fn region_event_segment_kind_counts(
    events: &RegionIntersectionSet,
    operand: ContourOperand,
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for pair in events.pairs() {
        for event in pair.intersections.events() {
            match event.segment_kind(operand) {
                SegmentKind::Line => counts.lines += 1,
                SegmentKind::Arc => counts.arcs += 1,
            }
        }
    }
    counts
}

fn collect_role_pairs(
    pairs: &mut Vec<RegionContourIntersection>,
    workload: &mut RegionIntersectionWorkload,
    first_contours: &[&crate::Contour2],
    first_boxes: &[ContourIntersectionAabbs],
    first_role: RegionContourRole,
    second_contours: &[&crate::Contour2],
    second_boxes: &[ContourIntersectionAabbs],
    second_role: RegionContourRole,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for (first_index, first_contour) in first_contours.iter().enumerate() {
        for (second_index, second_contour) in second_contours.iter().enumerate() {
            workload.candidate_pair_count += 1;
            // Region event collection is still contour-pair based. Bounding
            // intervals are only candidate filters: decided disjoint boxes skip
            // the pair, while uncertain boxes fall through to exact events.
            if first_boxes[first_index].is_disjoint(&second_boxes[second_index], policy) {
                workload.skipped_aabb_pair_count += 1;
                continue;
            }

            workload.tested_pair_count += 1;
            let intersections = match (
                first_boxes[first_index].exact.as_ref(),
                second_boxes[second_index].exact.as_ref(),
            ) {
                (Some(first), Some(second)) => {
                    crate::events::intersect_contours_with_exact_dyadic_line_aabbs(
                        first_contour,
                        second_contour,
                        first,
                        second,
                        policy,
                    )?
                }
                _ => {
                    let first_segment_boxes =
                        first_boxes[first_index].segments(first_contour, policy);
                    let second_segment_boxes =
                        second_boxes[second_index].segments(second_contour, policy);
                    crate::events::intersect_contours_with_cached_aabbs(
                        first_contour,
                        second_contour,
                        first_boxes[first_index].contour.as_ref(),
                        second_boxes[second_index].contour.as_ref(),
                        first_segment_boxes,
                        second_segment_boxes,
                        None,
                        policy,
                    )?
                }
            };
            if intersections.is_empty() {
                continue;
            }

            pairs.push(RegionContourIntersection {
                first: RegionContourKey::new(RegionSide::First, first_role, first_index),
                second: RegionContourKey::new(RegionSide::Second, second_role, second_index),
                intersections,
            });
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoint_contact_index_skips_certified_interior_crossings() {
        let parameter = (Real::one() / Real::from(2_u8)).unwrap();
        let intersections = ContourIntersectionSet::new(vec![ContourIntersection::Point(
            crate::ContourPointIntersection {
                a_segment_index: 0,
                b_segment_index: 0,
                a_segment_kind: SegmentKind::Line,
                b_segment_kind: SegmentKind::Line,
                point: crate::Point2::new(parameter.clone(), parameter.clone()),
                a_param: parameter.clone(),
                b_param: parameter,
                kind: crate::IntersectionKind::Crossing,
            },
        )])
        .unwrap();
        let intersections = RegionIntersectionSet::new(vec![RegionContourIntersection {
            first: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
            second: RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0),
            intersections,
        }])
        .unwrap();

        assert!(
            RegionPointEndpointContactIndex::from_intersections(
                &intersections,
                &CurvePolicy::certified(),
            )
            .vertex_masks
            .is_empty()
        );
    }

    #[test]
    fn endpoint_contact_index_checks_both_incident_segments() {
        let key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
        let policy = CurvePolicy::certified();
        let mut index = RegionPointEndpointContactIndex::default();
        index
            .vertex_masks
            .insert((key, 1), RegionPointEndpointContactIndex::START);
        index
            .vertex_masks
            .insert((key, 2), RegionPointEndpointContactIndex::END);

        assert!(index.parameter_is_contact(key, 0, 4, &Real::one(), &policy));
        assert!(index.parameter_is_contact(key, 3, 4, &Real::zero(), &policy));
        assert!(!index.parameter_is_contact(key, 0, 4, &Real::zero(), &policy));
        assert!(index.parameter_is_contact(
            key,
            0,
            4,
            &(Real::one() / Real::from(2_u8)).unwrap(),
            &policy,
        ));
    }
}
