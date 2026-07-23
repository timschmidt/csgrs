//! Region-pair fragments produced from region intersection events.
//!
//! Region booleans operate on all material and hole contours from both
//! operands. This module applies the contour-level intersection-insertion pass
//! to each keyed contour, matching the split-boundary construction used before
//! entry/exit or fill-state classification in polygon clipping traversal.

use crate::fragment::{
    CompactLineContourFragment, compact_line_contour_fragments_from_crossing_windings,
};
use crate::region_crossing_winding::RegionLineCrossingWindingIndex;
use crate::{
    Classification, Contour2, ContourFragmentSet, ContourOperand, ContourSplitMarkers, CurveError,
    CurvePolicy, CurveResult, RegionContourKey, RegionContourRole, RegionIntersectionSet,
    RegionSide, RegionView2, UncertaintyReason,
};

/// Fragments for one keyed contour in a region-pair query.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourFragments {
    /// Source contour key.
    pub key: RegionContourKey,
    /// Source contour split into traversal-order fragments.
    pub fragments: ContourFragmentSet,
}

#[derive(Debug)]
pub(crate) struct CompactLineRegionContourFragments {
    pub(crate) key: RegionContourKey,
    pub(crate) fragments: Vec<CompactLineContourFragment>,
}

#[derive(Debug)]
pub(crate) struct CompactLineRegionFragmentSet {
    contours: Vec<CompactLineRegionContourFragments>,
}

impl CompactLineRegionFragmentSet {
    pub(crate) fn contours(&self) -> &[CompactLineRegionContourFragments] {
        &self.contours
    }

    pub(crate) fn fragment_count(&self) -> usize {
        self.contours
            .iter()
            .map(|contour| contour.fragments.len())
            .sum()
    }

    pub(crate) fn parameters_are_materialized(
        &self,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
    ) -> bool {
        self.contours.iter().all(|contour| {
            contour
                .fragments
                .iter()
                .all(|fragment| fragment.parameter_is_materialized(contour.key, crossing_windings))
        })
    }
}

/// Fragment inventory for both regions in a region-pair query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionFragmentSet {
    contours: Vec<RegionContourFragments>,
}

impl RegionFragmentSet {
    /// Constructs a fragment set from already-built keyed contour fragments.
    pub fn new(contours: Vec<RegionContourFragments>) -> CurveResult<Self> {
        validate_region_fragment_keys(&contours)?;
        Ok(Self { contours })
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
    validate_region_intersection_evidence_against_views(first, second, intersections)?;

    let mut out = Vec::new();
    if let Classification::Uncertain(reason) =
        append_all_region_contours(&mut out, first, second, intersections, policy)?
    {
        return Ok(Classification::Uncertain(reason));
    }
    Ok(Classification::Decided(RegionFragmentSet::new(out)?))
}

pub(crate) fn split_single_material_line_regions_compact(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &RegionIntersectionSet,
    crossing_windings: &RegionLineCrossingWindingIndex<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<CompactLineRegionFragmentSet>> {
    validate_region_intersection_evidence_against_views(first, second, intersections)?;
    if first.material_contours().len() != 1
        || second.material_contours().len() != 1
        || !first.hole_contours().is_empty()
        || !second.hole_contours().is_empty()
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }

    let mut contours = Vec::with_capacity(2);
    for (key, contour) in [
        (
            RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0),
            first.material_contours()[0],
        ),
        (
            RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0),
            second.material_contours()[0],
        ),
    ] {
        let compact_fragments = match compact_line_contour_fragments_from_crossing_windings(
            contour,
            key,
            crossing_windings,
            policy,
        )? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        contours.push(CompactLineRegionContourFragments {
            key,
            fragments: compact_fragments,
        });
    }
    Ok(Classification::Decided(CompactLineRegionFragmentSet {
        contours,
    }))
}

fn validate_region_fragment_keys(contours: &[RegionContourFragments]) -> CurveResult<()> {
    if contours
        .iter()
        .any(|contour_fragments| contour_fragments.fragments.is_empty())
    {
        return Err(CurveError::Topology(
            "region fragment set keyed contour evidence must carry fragments".into(),
        ));
    }

    let mut keys = contours
        .iter()
        .map(|contour_fragments| contour_fragments.key)
        .collect::<Vec<_>>();
    keys.sort_unstable();
    if keys.windows(2).any(|window| window[0] == window[1]) {
        return Err(CurveError::Topology(
            "region fragment set must not contain duplicate contour keys".into(),
        ));
    }
    Ok(())
}

fn validate_region_intersection_evidence_against_views(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &RegionIntersectionSet,
) -> CurveResult<()> {
    for pair in intersections.pairs() {
        let first_contour = contour_for_key(first, RegionSide::First, pair.first)?;
        let second_contour = contour_for_key(second, RegionSide::Second, pair.second)?;
        if let Some(crossings) = pair.intersections().retained_certified_line_crossings() {
            for crossing in crossings.iter() {
                validate_event_segment_index(
                    Some(usize::from(crossing.a_segment_index)),
                    first_contour.len(),
                )?;
                validate_event_segment_index(
                    Some(usize::from(crossing.b_segment_index)),
                    second_contour.len(),
                )?;
            }
            continue;
        }
        for event in pair.intersections.events() {
            validate_event_segment_index(
                event.segment_index(ContourOperand::First),
                first_contour.len(),
            )?;
            validate_event_segment_index(
                event.segment_index(ContourOperand::Second),
                second_contour.len(),
            )?;
        }
    }
    Ok(())
}

fn contour_for_key<'a>(
    view: &'a RegionView2<'_>,
    expected_side: RegionSide,
    key: RegionContourKey,
) -> CurveResult<&'a Contour2> {
    if key.side != expected_side {
        return Err(CurveError::Topology(
            "region intersection pair references the wrong region side".into(),
        ));
    }
    let contours = match key.role {
        RegionContourRole::Material => view.material_contours(),
        RegionContourRole::Hole => view.hole_contours(),
    };
    contours.get(key.index).copied().ok_or_else(|| {
        CurveError::Topology(
            "region intersection pair references contour outside supplied region view".into(),
        )
    })
}

fn validate_event_segment_index(
    segment_index: Option<usize>,
    segment_count: usize,
) -> CurveResult<()> {
    let Some(segment_index) = segment_index else {
        return Err(CurveError::Topology(
            "region intersection event must carry segment index evidence".into(),
        ));
    };
    if segment_index >= segment_count {
        return Err(CurveError::Topology(
            "region intersection event references segment outside supplied contour".into(),
        ));
    }
    Ok(())
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
        let fragments = match split_keyed_contour(contour, key, intersections, policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        out.push(RegionContourFragments { key, fragments });
    }

    Ok(Classification::Decided(()))
}

fn append_all_region_contours(
    out: &mut Vec<RegionContourFragments>,
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    for (side, contours, role) in [
        (
            RegionSide::First,
            first.material_contours(),
            RegionContourRole::Material,
        ),
        (
            RegionSide::First,
            first.hole_contours(),
            RegionContourRole::Hole,
        ),
        (
            RegionSide::Second,
            second.material_contours(),
            RegionContourRole::Material,
        ),
        (
            RegionSide::Second,
            second.hole_contours(),
            RegionContourRole::Hole,
        ),
    ] {
        match append_region_contours(out, side, contours, role, intersections, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(()))
}

fn split_keyed_contour(
    contour: &Contour2,
    key: RegionContourKey,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet>> {
    let markers = match split_markers_for_keyed_contour(contour, key, intersections, policy)? {
        Classification::Decided(markers) => markers,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    ContourFragmentSet::from_split_markers(contour, &markers, policy)
}

fn split_markers_for_keyed_contour(
    contour: &Contour2,
    key: RegionContourKey,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourSplitMarkers>> {
    let mut markers = ContourSplitMarkers::with_implicit_contour_endpoints(contour);

    for pair in intersections.pairs_for_contour(key) {
        let operand = if pair.first == key {
            ContourOperand::First
        } else {
            ContourOperand::Second
        };

        match markers.merge_intersections_for_contour(contour, &pair.intersections, operand, policy)
        {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    Ok(Classification::Decided(markers))
}
