//! Region-level intersection event collection.
//!
//! Region event collection lifts contour-pair events into material/hole keyed
//! operands. It keeps broad-phase pruning conservative for the same reason as
//! Bentley and Ottmann's intersection reporting work: candidate generation may
//! be optimized, but topology still depends on the exact segment relation.

use crate::bbox::{aabbs_decided_disjoint, decided_contour_aabb};
use crate::{
    Classification, ContourIntersectionSet, CurveError, CurvePolicy, CurveResult, RegionView2,
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

    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.material_contours(),
        RegionContourRole::Material,
        second.material_contours(),
        RegionContourRole::Material,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.material_contours(),
        RegionContourRole::Material,
        second.hole_contours(),
        RegionContourRole::Hole,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.hole_contours(),
        RegionContourRole::Hole,
        second.material_contours(),
        RegionContourRole::Material,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        &mut workload,
        first.hole_contours(),
        RegionContourRole::Hole,
        second.hole_contours(),
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

fn collect_role_pairs(
    pairs: &mut Vec<RegionContourIntersection>,
    workload: &mut RegionIntersectionWorkload,
    first_contours: &[&crate::Contour2],
    first_role: RegionContourRole,
    second_contours: &[&crate::Contour2],
    second_role: RegionContourRole,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let first_boxes: Vec<_> = first_contours
        .iter()
        .map(|contour| decided_contour_aabb(contour, policy))
        .collect();
    let second_boxes: Vec<_> = second_contours
        .iter()
        .map(|contour| decided_contour_aabb(contour, policy))
        .collect();

    for (first_index, first_contour) in first_contours.iter().enumerate() {
        for (second_index, second_contour) in second_contours.iter().enumerate() {
            workload.candidate_pair_count += 1;
            // Region event collection is still contour-pair based. As in
            // Bentley and Ottmann's intersection-reporting work, bounding
            // intervals are only candidate filters here: decided disjoint boxes
            // skip the pair, while uncertain boxes fall through to exact
            // contour events. Reference: Bentley and Ottmann, "Algorithms for
            // Reporting and Counting Geometric Intersections," IEEE
            // Transactions on Computers C-28(9), 643-647, 1979.
            if let (Some(first_box), Some(second_box)) =
                (&first_boxes[first_index], &second_boxes[second_index])
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                workload.skipped_aabb_pair_count += 1;
                continue;
            }

            workload.tested_pair_count += 1;
            let intersections = first_contour.intersect_contour(second_contour, policy)?;
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
