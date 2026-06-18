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
    Classification, Contour2, ContourFragmentSet, ContourOperand, ContourSplitMarkers, CurveError,
    CurvePolicy, CurveResult, RegionContourKey, RegionContourRole, RegionIntersectionSet,
    RegionSide, RegionView2, RetainedTopologyStatus, UncertaintyReason,
};

/// Fragments for one keyed contour in a region-pair query.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourFragments {
    /// Source contour key.
    pub key: RegionContourKey,
    /// Source contour split into traversal-order fragments.
    pub fragments: ContourFragmentSet,
}

/// Fragment materialization report for one keyed source contour.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionContourFragmentReport2 {
    key: RegionContourKey,
    source_segment_count: usize,
    output_fragment_count: usize,
    status: RetainedTopologyStatus,
}

/// Report for splitting two region views at retained intersection evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionFragmentBuildReport2 {
    stage: RegionFragmentBuildStage2,
    first_source_contour_count: usize,
    second_source_contour_count: usize,
    first_source_segment_count: usize,
    second_source_segment_count: usize,
    intersection_pair_count: usize,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    output_contour_count: Option<usize>,
    output_fragment_count: Option<usize>,
    contour_reports: Vec<RegionContourFragmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by region-fragment construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionFragmentBuildStage2 {
    /// Supplied region/intersection evidence was being validated.
    IntersectionEvidenceValidation,
    /// Keyed contours were being split at retained intersection parameters.
    ContourSplitting,
}

/// Result of report-bearing region-fragment construction.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionFragmentBuildResult2 {
    fragments: Option<RegionFragmentSet>,
    report: RegionFragmentBuildReport2,
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
    let result =
        split_region_views_at_intersections_with_report(first, second, intersections, policy)?;
    let blocker = result
        .report()
        .blocker()
        .unwrap_or(UncertaintyReason::Unsupported);
    if let Some(fragments) = result.into_fragments() {
        Ok(Classification::Decided(fragments))
    } else {
        Ok(Classification::Uncertain(blocker))
    }
}

pub(crate) fn split_region_views_at_intersections_with_report(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<RegionFragmentBuildResult2> {
    validate_region_intersection_evidence_against_views(first, second, intersections)?;

    let first_source_contour_count = first.material_contours().len() + first.hole_contours().len();
    let second_source_contour_count =
        second.material_contours().len() + second.hole_contours().len();
    let first_source_segment_count = source_segment_count(first);
    let second_source_segment_count = source_segment_count(second);
    let mut contours = Vec::new();
    let mut contour_reports = Vec::new();

    match append_region_contours(
        &mut contours,
        &mut contour_reports,
        RegionSide::First,
        first.material_contours(),
        RegionContourRole::Material,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(blocked_region_fragment_build_result(
                first_source_contour_count,
                second_source_contour_count,
                first_source_segment_count,
                second_source_segment_count,
                intersections,
                contour_reports,
                reason,
            ));
        }
    }
    match append_region_contours(
        &mut contours,
        &mut contour_reports,
        RegionSide::First,
        first.hole_contours(),
        RegionContourRole::Hole,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(blocked_region_fragment_build_result(
                first_source_contour_count,
                second_source_contour_count,
                first_source_segment_count,
                second_source_segment_count,
                intersections,
                contour_reports,
                reason,
            ));
        }
    }
    match append_region_contours(
        &mut contours,
        &mut contour_reports,
        RegionSide::Second,
        second.material_contours(),
        RegionContourRole::Material,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(blocked_region_fragment_build_result(
                first_source_contour_count,
                second_source_contour_count,
                first_source_segment_count,
                second_source_segment_count,
                intersections,
                contour_reports,
                reason,
            ));
        }
    }
    match append_region_contours(
        &mut contours,
        &mut contour_reports,
        RegionSide::Second,
        second.hole_contours(),
        RegionContourRole::Hole,
        intersections,
        policy,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(blocked_region_fragment_build_result(
                first_source_contour_count,
                second_source_contour_count,
                first_source_segment_count,
                second_source_segment_count,
                intersections,
                contour_reports,
                reason,
            ));
        }
    }

    let output_contour_count = contours.len();
    let output_fragment_count = contour_reports
        .iter()
        .map(RegionContourFragmentReport2::output_fragment_count)
        .sum();
    Ok(RegionFragmentBuildResult2 {
        fragments: Some(RegionFragmentSet::new(contours)?),
        report: RegionFragmentBuildReport2 {
            stage: RegionFragmentBuildStage2::ContourSplitting,
            first_source_contour_count,
            second_source_contour_count,
            first_source_segment_count,
            second_source_segment_count,
            intersection_pair_count: intersections.intersecting_pair_count(),
            candidate_pair_count: intersections.candidate_pair_count(),
            skipped_aabb_pair_count: intersections.skipped_aabb_pair_count(),
            tested_pair_count: intersections.tested_pair_count(),
            output_contour_count: Some(output_contour_count),
            output_fragment_count: Some(output_fragment_count),
            contour_reports,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    })
}

impl RegionContourFragmentReport2 {
    /// Returns the keyed source contour represented by this report.
    pub const fn key(&self) -> RegionContourKey {
        self.key
    }

    /// Returns the number of source contour segments before splitting.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the number of retained fragments emitted for this contour.
    pub const fn output_fragment_count(&self) -> usize {
        self.output_fragment_count
    }

    /// Returns retained topology status for this contour split.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl RegionFragmentBuildReport2 {
    /// Returns the furthest exact fragment-build stage reached.
    pub const fn stage(&self) -> RegionFragmentBuildStage2 {
        self.stage
    }

    /// Returns the number of source contours in the first region view.
    pub const fn first_source_contour_count(&self) -> usize {
        self.first_source_contour_count
    }

    /// Returns the number of source contours in the second region view.
    pub const fn second_source_contour_count(&self) -> usize {
        self.second_source_contour_count
    }

    /// Returns the number of source contour segments in the first region view.
    pub const fn first_source_segment_count(&self) -> usize {
        self.first_source_segment_count
    }

    /// Returns the number of source contour segments in the second region view.
    pub const fn second_source_segment_count(&self) -> usize {
        self.second_source_segment_count
    }

    /// Returns the number of keyed contour pairs that retained intersections.
    pub const fn intersection_pair_count(&self) -> usize {
        self.intersection_pair_count
    }

    /// Returns all contour-pair candidates considered by the source event set.
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

    /// Returns output keyed contour count when splitting materialized.
    pub const fn output_contour_count(&self) -> Option<usize> {
        self.output_contour_count
    }

    /// Returns output fragment count when splitting materialized.
    pub const fn output_fragment_count(&self) -> Option<usize> {
        self.output_fragment_count
    }

    /// Returns per-contour split provenance.
    pub fn contour_reports(&self) -> &[RegionContourFragmentReport2] {
        &self.contour_reports
    }

    /// Returns retained topology status for fragment construction.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized fragment construction.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl RegionFragmentBuildResult2 {
    /// Returns materialized region fragments, if splitting succeeded.
    pub const fn fragments(&self) -> Option<&RegionFragmentSet> {
        self.fragments.as_ref()
    }

    /// Consumes this result and returns materialized region fragments, if any.
    pub fn into_fragments(self) -> Option<RegionFragmentSet> {
        self.fragments
    }

    /// Returns retained fragment-build evidence.
    pub const fn report(&self) -> &RegionFragmentBuildReport2 {
        &self.report
    }
}

fn blocked_region_fragment_build_result(
    first_source_contour_count: usize,
    second_source_contour_count: usize,
    first_source_segment_count: usize,
    second_source_segment_count: usize,
    intersections: &RegionIntersectionSet,
    contour_reports: Vec<RegionContourFragmentReport2>,
    blocker: UncertaintyReason,
) -> RegionFragmentBuildResult2 {
    RegionFragmentBuildResult2 {
        fragments: None,
        report: RegionFragmentBuildReport2 {
            stage: RegionFragmentBuildStage2::ContourSplitting,
            first_source_contour_count,
            second_source_contour_count,
            first_source_segment_count,
            second_source_segment_count,
            intersection_pair_count: intersections.intersecting_pair_count(),
            candidate_pair_count: intersections.candidate_pair_count(),
            skipped_aabb_pair_count: intersections.skipped_aabb_pair_count(),
            tested_pair_count: intersections.tested_pair_count(),
            output_contour_count: None,
            output_fragment_count: None,
            contour_reports,
            status: RetainedTopologyStatus::Unresolved,
            blocker: Some(blocker),
        },
    }
}

fn source_segment_count(view: &RegionView2<'_>) -> usize {
    view.material_contours()
        .iter()
        .chain(view.hole_contours())
        .map(|contour| contour.len())
        .sum()
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
    reports: &mut Vec<RegionContourFragmentReport2>,
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
        reports.push(RegionContourFragmentReport2 {
            key,
            source_segment_count: contour.len(),
            output_fragment_count: fragments.len(),
            status: RetainedTopologyStatus::NativeExact,
        });
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
