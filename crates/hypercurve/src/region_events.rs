//! Region-level intersection event collection.

use hyperlattice::{Backend, DefaultBackend};

use crate::bbox::{aabbs_decided_disjoint, decided_contour_aabb};
use crate::{Classification, ContourIntersectionSet, CurvePolicy, CurveResult, RegionView2};

/// Which region side a contour key belongs to.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionSide {
    /// First region passed to the query.
    First,
    /// Second region passed to the query.
    Second,
}

/// Semantic role of a contour inside a region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionContourRole {
    /// Positive material contour.
    Material,
    /// Negative hole contour.
    Hole,
}

/// Identifies one contour inside a region-pair query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
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
pub struct RegionContourIntersection<B: Backend = DefaultBackend> {
    /// Contour in the first region.
    pub first: RegionContourKey,
    /// Contour in the second region.
    pub second: RegionContourKey,
    /// Normalized contour-level intersections for this pair.
    pub intersections: ContourIntersectionSet<B>,
}

/// Normalized contour-pair intersections between two regions.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionIntersectionSet<B: Backend = DefaultBackend> {
    pairs: Vec<RegionContourIntersection<B>>,
}

impl<B: Backend> RegionIntersectionSet<B> {
    /// Constructs a set from already-normalized region contour pairs.
    pub const fn new(pairs: Vec<RegionContourIntersection<B>>) -> Self {
        Self { pairs }
    }

    /// Returns nonempty contour-pair event sets.
    pub fn pairs(&self) -> &[RegionContourIntersection<B>] {
        &self.pairs
    }

    /// Consumes the set and returns contour-pair event sets.
    pub fn into_pairs(self) -> Vec<RegionContourIntersection<B>> {
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
    ) -> impl Iterator<Item = &RegionContourIntersection<B>> {
        self.pairs
            .iter()
            .filter(move |pair| pair.first == key || pair.second == key)
    }

    /// Splits every contour in both region views at this event set.
    pub fn split_regions(
        &self,
        first: &RegionView2<'_, B>,
        second: &RegionView2<'_, B>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<crate::RegionFragmentSet<B>>> {
        crate::region_fragments::split_region_views_at_intersections(first, second, self, policy)
    }
}

pub(crate) fn intersect_region_views<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    policy: &CurvePolicy,
) -> CurveResult<RegionIntersectionSet<B>> {
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

    Ok(RegionIntersectionSet::new(pairs))
}

fn collect_role_pairs<B: Backend>(
    pairs: &mut Vec<RegionContourIntersection<B>>,
    first_contours: &[&crate::Contour2<B>],
    first_role: RegionContourRole,
    second_contours: &[&crate::Contour2<B>],
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
            // contour events.
            if let (Some(first_box), Some(second_box)) =
                (&first_boxes[first_index], &second_boxes[second_index])
            {
                if aabbs_decided_disjoint(first_box, second_box, policy) {
                    continue;
                }
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
