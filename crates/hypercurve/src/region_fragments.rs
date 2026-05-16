//! Region-pair fragments produced from region intersection events.
//!
//! Region booleans operate on all material and hole contours from both
//! operands. This module applies the contour-level intersection-insertion pass
//! to each keyed contour, matching the split-boundary preparation used before
//! entry/exit or fill-state classification in Greiner and Hormann, "Efficient
//! Clipping of Arbitrary Polygons" (*ACM Transactions on Graphics* 17(2),
//! 71-83, 1998), and Martinez, Rueda, and Feito, "A New Algorithm for
//! Computing Boolean Operations on Polygons" (*Computers & Geosciences* 35(6),
//! 1177-1185, 2009).

use crate::{
    Classification, Contour2, ContourFragmentSet, ContourOperand, ContourSplitMarkers, CurvePolicy,
    CurveResult, RegionContourKey, RegionContourRole, RegionIntersectionSet, RegionSide,
    RegionView2,
};

/// Fragments for one keyed contour in a region-pair query.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourFragments {
    /// Source contour key.
    pub key: RegionContourKey,
    /// Source contour split into traversal-order fragments.
    pub fragments: ContourFragmentSet,
}

/// Fragment inventory for both regions in a region-pair query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionFragmentSet {
    contours: Vec<RegionContourFragments>,
}

impl RegionFragmentSet {
    /// Constructs a fragment set from already-built keyed contour fragments.
    pub const fn new(contours: Vec<RegionContourFragments>) -> Self {
        Self { contours }
    }

    /// Returns keyed contour fragments.
    pub fn contours(&self) -> &[RegionContourFragments] {
        &self.contours
    }

    /// Consumes the set and returns keyed contour fragments.
    pub fn into_contours(self) -> Vec<RegionContourFragments> {
        self.contours
    }

    /// Returns true when no contour fragments were built.
    pub fn is_empty(&self) -> bool {
        self.contours.is_empty()
    }

    /// Returns the number of keyed contours represented by this set.
    pub fn len(&self) -> usize {
        self.contours.len()
    }

    /// Returns fragments for a keyed contour.
    pub fn fragments_for_contour(&self, key: RegionContourKey) -> Option<&RegionContourFragments> {
        self.contours.iter().find(|fragments| fragments.key == key)
    }
}

pub(crate) fn split_region_views_at_intersections(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RegionFragmentSet>> {
    let mut contours = Vec::new();

    match append_region_contours(
        &mut contours,
        RegionSide::First,
        first.material_contours(),
        RegionContourRole::Material,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match append_region_contours(
        &mut contours,
        RegionSide::First,
        first.hole_contours(),
        RegionContourRole::Hole,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match append_region_contours(
        &mut contours,
        RegionSide::Second,
        second.material_contours(),
        RegionContourRole::Material,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match append_region_contours(
        &mut contours,
        RegionSide::Second,
        second.hole_contours(),
        RegionContourRole::Hole,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(RegionFragmentSet::new(contours)))
}

fn append_region_contours(
    out: &mut Vec<RegionContourFragments>,
    side: RegionSide,
    contours: &[&Contour2],
    role: RegionContourRole,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    for (index, contour) in contours.iter().enumerate() {
        let key = RegionContourKey::new(side, role, index);
        let fragments = match split_keyed_contour(*contour, key, intersections, policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        out.push(RegionContourFragments { key, fragments });
    }

    Ok(Classification::Decided(()))
}

fn split_keyed_contour(
    contour: &Contour2,
    key: RegionContourKey,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet>> {
    let mut markers = ContourSplitMarkers::with_contour_endpoints(contour);

    for pair in intersections.pairs_for_contour(key) {
        let operand = if pair.first == key {
            ContourOperand::First
        } else {
            ContourOperand::Second
        };

        match markers.merge_intersections(&pair.intersections, operand, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    ContourFragmentSet::from_split_markers(contour, &markers, policy)
}
