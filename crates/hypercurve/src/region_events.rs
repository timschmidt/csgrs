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
}

impl RegionIntersectionSet {
    /// Constructs a set from already-normalized region contour pairs.
    pub fn new(pairs: Vec<RegionContourIntersection>) -> CurveResult<Self> {
        validate_region_intersection_pairs(&pairs)?;
        Ok(Self { pairs })
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

    /// Returns the number of contour pairs with events.
    pub fn len(&self) -> usize {
        self.pairs.len()
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
}

pub(crate) fn intersect_region_views(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<RegionIntersectionSet> {
    let mut pairs = Vec::new();

    collect_role_pairs(
        &mut pairs,
        first.material_contours(),
        RegionContourRole::Material,
        second.material_contours(),
        RegionContourRole::Material,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        first.material_contours(),
        RegionContourRole::Material,
        second.hole_contours(),
        RegionContourRole::Hole,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        first.hole_contours(),
        RegionContourRole::Hole,
        second.material_contours(),
        RegionContourRole::Material,
        policy,
    )?;
    collect_role_pairs(
        &mut pairs,
        first.hole_contours(),
        RegionContourRole::Hole,
        second.hole_contours(),
        RegionContourRole::Hole,
        policy,
    )?;

    RegionIntersectionSet::new(pairs)
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
                continue;
            }

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
