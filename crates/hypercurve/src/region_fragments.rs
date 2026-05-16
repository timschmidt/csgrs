//! Region-pair fragments produced from region intersection events.

use hyperlattice::{Backend, DefaultBackend};

use crate::{
    Classification, Contour2, ContourFragmentSet, ContourOperand, ContourSplitMarkers, CurvePolicy,
    CurveResult, RegionContourKey, RegionContourRole, RegionIntersectionSet, RegionSide,
    RegionView2,
};

/// Fragments for one keyed contour in a region-pair query.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourFragments<B: Backend = DefaultBackend> {
    /// Source contour key.
    pub key: RegionContourKey,
    /// Source contour split into traversal-order fragments.
    pub fragments: ContourFragmentSet<B>,
}

/// Fragment inventory for both regions in a region-pair query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionFragmentSet<B: Backend = DefaultBackend> {
    contours: Vec<RegionContourFragments<B>>,
}

impl<B: Backend> RegionFragmentSet<B> {
    /// Constructs a fragment set from already-built keyed contour fragments.
    pub const fn new(contours: Vec<RegionContourFragments<B>>) -> Self {
        Self { contours }
    }

    /// Returns keyed contour fragments.
    pub fn contours(&self) -> &[RegionContourFragments<B>] {
        &self.contours
    }

    /// Consumes the set and returns keyed contour fragments.
    pub fn into_contours(self) -> Vec<RegionContourFragments<B>> {
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
    pub fn fragments_for_contour(
        &self,
        key: RegionContourKey,
    ) -> Option<&RegionContourFragments<B>> {
        self.contours.iter().find(|fragments| fragments.key == key)
    }
}

pub(crate) fn split_region_views_at_intersections<B: Backend>(
    first: &RegionView2<'_, B>,
    second: &RegionView2<'_, B>,
    intersections: &RegionIntersectionSet<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RegionFragmentSet<B>>> {
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

fn append_region_contours<B: Backend>(
    out: &mut Vec<RegionContourFragments<B>>,
    side: RegionSide,
    contours: &[&Contour2<B>],
    role: RegionContourRole,
    intersections: &RegionIntersectionSet<B>,
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

fn split_keyed_contour<B: Backend>(
    contour: &Contour2<B>,
    key: RegionContourKey,
    intersections: &RegionIntersectionSet<B>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet<B>>> {
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
