//! Boolean fragment classification.
//!
//! This module is the split/classify/select layer before graph traversal and
//! loop assembly. It deliberately does not resolve shared-boundary fragments:
//! those need overlap-aware traversal, not a midpoint guess.

use crate::boolean_boundary::{BooleanBoundaryFragmentSet, DirectedBooleanFragment};
use crate::classify::real_sign;
use crate::{
    Classification, CurveError, CurvePolicy, CurveResult, ParamRange, Point2, RegionContourKey,
    RegionContourRole, RegionFragmentSet, RegionPointLocation, RegionSide, RegionView2,
    RetainedTopologyStatus, Segment2, SegmentKind, SegmentKindCounts, UncertaintyReason,
};
use hyperreal::{Real, RealSign};

/// Boolean operation requested between two regions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanOp {
    /// Filled area in either operand.
    Union,
    /// Filled area common to both operands.
    Intersection,
    /// Filled area in the first operand but not the second.
    Difference,
    /// Filled area in exactly one operand.
    Xor,
}

/// How a classified source fragment participates in a boolean result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanFragmentAction {
    /// The fragment is not part of this operation's boundary.
    Discard,
    /// Emit the fragment in its source traversal direction.
    KeepSourceDirection,
    /// Emit the fragment in the reverse of its source traversal direction.
    KeepReversed,
    /// The representative point lies on the other region's boundary.
    ///
    /// Shared boundaries need a dedicated overlap resolver. Treating them as
    /// inside or outside would recreate the tolerance-first ambiguity this
    /// crate is avoiding.
    BoundaryNeedsResolution,
}

impl BooleanFragmentAction {
    /// Returns true when this action emits a directed fragment immediately.
    pub const fn emits_fragment(self) -> bool {
        matches!(self, Self::KeepSourceDirection | Self::KeepReversed)
    }
}

/// Boolean classification for one source fragment.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BooleanFragmentClassification {
    /// Which keyed source contour owns this fragment.
    pub key: crate::RegionContourKey,
    /// Index within [`crate::RegionContourFragments::fragments`].
    pub fragment_index: usize,
    /// Location of the fragment representative point in the opposite region.
    pub opposite_location: RegionPointLocation,
    /// Whether the source region is filled left of this contour's traversal.
    pub source_filled_side_is_left: bool,
    /// Selection action for the requested operation.
    pub action: BooleanFragmentAction,
}

/// Boolean classification for all fragments in a region-pair fragment set.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct BooleanFragmentSelection {
    classifications: Vec<BooleanFragmentClassification>,
}

/// Report for boolean fragment classification against the opposite region.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanFragmentSelectionReport2 {
    op: BooleanOp,
    stage: BooleanFragmentSelectionStage2,
    source_contour_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    classified_fragment_count: Option<usize>,
    discard_count: Option<usize>,
    keep_source_direction_count: Option<usize>,
    keep_reversed_count: Option<usize>,
    boundary_needs_resolution_count: Option<usize>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by boolean fragment classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanFragmentSelectionStage2 {
    /// Source-contour orientation and signed fill role were being certified.
    SourceFillSideClassification,
    /// A retained fragment representative point was being materialized.
    RepresentativePoint,
    /// Representative points were being classified against opposite regions.
    OppositeRegionClassification,
    /// Boolean fragment actions were assigned and validated.
    ActionAssignment,
}

/// Result of report-bearing boolean fragment classification.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanFragmentSelectionResult2 {
    selection: Option<BooleanFragmentSelection>,
    report: BooleanFragmentSelectionReport2,
}

enum FragmentInteriorClassification {
    Decided(RegionPointLocation),
    Blocked {
        stage: BooleanFragmentSelectionStage2,
        reason: UncertaintyReason,
    },
}

/// Report for emitting selected boolean classifications as boundary fragments.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryFragmentEmissionReport2 {
    stage: BooleanBoundaryFragmentEmissionStage2,
    source_classification_count: usize,
    discard_count: usize,
    keep_source_direction_count: usize,
    keep_reversed_count: usize,
    boundary_needs_resolution_count: usize,
    directed_fragment_count: Option<usize>,
    directed_source_segment_kind_counts: Option<SegmentKindCounts>,
    directed_fragment_kind_counts: Option<SegmentKindCounts>,
    directed_fragments: Vec<BooleanDirectedFragmentReport2>,
    unresolved_boundary_count: Option<usize>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Source provenance for one directed boundary fragment emitted by a boolean selection.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanDirectedFragmentReport2 {
    key: RegionContourKey,
    fragment_index: usize,
    source_segment_index: usize,
    source_segment_kind: SegmentKind,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    reversed: bool,
    output_fragment_index: usize,
    output_fragment_kind: SegmentKind,
}

/// Furthest exact stage reached by boolean boundary fragment emission.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryFragmentEmissionStage2 {
    /// Selection ownership was being validated against the supplied fragments.
    SourceValidation,
    /// Selected fragments were emitted in traversal direction or deferred.
    FragmentEmission,
}

/// Result of report-bearing boolean boundary fragment emission.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryFragmentEmissionResult2 {
    fragments: Option<BooleanBoundaryFragmentSet>,
    report: BooleanBoundaryFragmentEmissionReport2,
}

impl BooleanFragmentSelection {
    /// Constructs a selection from already-classified fragments.
    pub fn new(classifications: Vec<BooleanFragmentClassification>) -> CurveResult<Self> {
        validate_boolean_fragment_classifications(&classifications)?;
        Ok(Self { classifications })
    }

    /// Returns all fragment classifications in region-fragment order.
    pub fn classifications(&self) -> &[BooleanFragmentClassification] {
        &self.classifications
    }

    /// Consumes the selection and returns the fragment classifications.
    pub fn into_classifications(self) -> Vec<BooleanFragmentClassification> {
        self.classifications
    }

    /// Returns true when no fragments were classified.
    pub fn is_empty(&self) -> bool {
        self.classifications.is_empty()
    }

    /// Returns the number of classified fragments.
    pub fn len(&self) -> usize {
        self.classifications.len()
    }

    /// Counts classifications with the given action.
    pub fn count_action(&self, action: BooleanFragmentAction) -> usize {
        self.classifications
            .iter()
            .filter(|classification| classification.action == action)
            .count()
    }

    pub(crate) fn resolve_boundary_actions(
        &self,
        resolutions: &[(RegionContourKey, usize, BooleanFragmentAction)],
    ) -> CurveResult<Self> {
        let mut classifications = self.classifications.clone();
        let mut used = vec![false; resolutions.len()];
        for classification in &mut classifications {
            if classification.action != BooleanFragmentAction::BoundaryNeedsResolution {
                continue;
            }
            let mut matched = None;
            for (index, (key, fragment_index, action)) in resolutions.iter().enumerate() {
                if *key != classification.key || *fragment_index != classification.fragment_index {
                    continue;
                }
                if used[index]
                    || *action == BooleanFragmentAction::BoundaryNeedsResolution
                    || matched.is_some()
                {
                    return Err(CurveError::Topology(
                        "boolean shared-boundary resolution is duplicated or unresolved".into(),
                    ));
                }
                matched = Some((index, *action));
            }
            let Some((index, action)) = matched else {
                return Err(CurveError::Topology(
                    "boolean shared-boundary resolution is incomplete".into(),
                ));
            };
            used[index] = true;
            classification.action = action;
        }
        if used.iter().any(|used| !used) {
            return Err(CurveError::Topology(
                "boolean shared-boundary resolution references a decided fragment".into(),
            ));
        }
        Ok(Self { classifications })
    }

    /// Converts selected classifications into directed boundary fragments.
    ///
    /// This performs the "emit in source direction or reverse direction" step
    /// after local boolean classification. Polygon-clipping traversal follows
    /// selected directed chains after entry/exit classification. Shared
    /// boundaries remain in `unresolved_boundaries` because coincident edges
    /// require handling distinct from ordinary enter/exit classification.
    pub fn emit_boundary_fragments(
        &self,
        fragments: &RegionFragmentSet,
    ) -> CurveResult<BooleanBoundaryFragmentSet> {
        let result = self.emit_boundary_fragments_with_report(fragments)?;
        result.into_fragments().ok_or_else(|| {
            CurveError::Topology("boolean boundary fragment emission did not materialize".into())
        })
    }

    /// Converts selected classifications into boundary fragments and retains evidence.
    pub fn emit_boundary_fragments_with_report(
        &self,
        fragments: &RegionFragmentSet,
    ) -> CurveResult<BooleanBoundaryFragmentEmissionResult2> {
        validate_boolean_selection_matches_fragments(&self.classifications, fragments)?;

        let mut directed_fragments = Vec::new();
        let mut unresolved_boundaries = Vec::new();

        for classification in &self.classifications {
            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    unresolved_boundaries.push(classification.clone());
                }
                BooleanFragmentAction::KeepSourceDirection
                | BooleanFragmentAction::KeepReversed => {
                    let source = fragment_for_classification(fragments, classification)?;
                    let segment =
                        if classification.action == BooleanFragmentAction::KeepSourceDirection {
                            source.segment.clone()
                        } else {
                            source.segment.reversed()
                        };
                    directed_fragments.push(DirectedBooleanFragment {
                        key: classification.key,
                        fragment_index: classification.fragment_index,
                        source_segment_index: source.source_segment_index,
                        source_segment_start_point: source.source_segment_start_point.clone(),
                        source_segment_end_point: source.source_segment_end_point.clone(),
                        source_range: source.source_range.clone(),
                        reversed: classification.action == BooleanFragmentAction::KeepReversed,
                        segment,
                    });
                }
            }
        }

        let directed_fragment_count = directed_fragments.len();
        let directed_fragment_kind_counts =
            directed_boolean_fragment_kind_counts(&directed_fragments);
        let directed_fragment_reports = boolean_directed_fragment_reports(&directed_fragments);
        let directed_source_segment_kind_counts =
            boolean_directed_fragment_report_source_kind_counts(&directed_fragment_reports);
        let unresolved_boundary_count = unresolved_boundaries.len();
        match BooleanBoundaryFragmentSet::new(directed_fragments, unresolved_boundaries) {
            Ok(fragments) => Ok(BooleanBoundaryFragmentEmissionResult2 {
                fragments: Some(fragments),
                report: BooleanBoundaryFragmentEmissionReport2 {
                    stage: BooleanBoundaryFragmentEmissionStage2::FragmentEmission,
                    source_classification_count: self.len(),
                    discard_count: self.count_action(BooleanFragmentAction::Discard),
                    keep_source_direction_count: self
                        .count_action(BooleanFragmentAction::KeepSourceDirection),
                    keep_reversed_count: self.count_action(BooleanFragmentAction::KeepReversed),
                    boundary_needs_resolution_count: self
                        .count_action(BooleanFragmentAction::BoundaryNeedsResolution),
                    directed_fragment_count: Some(directed_fragment_count),
                    directed_source_segment_kind_counts: Some(directed_source_segment_kind_counts),
                    directed_fragment_kind_counts: Some(directed_fragment_kind_counts),
                    directed_fragments: directed_fragment_reports,
                    unresolved_boundary_count: Some(unresolved_boundary_count),
                    status: RetainedTopologyStatus::NativeExact,
                    blocker: None,
                },
            }),
            Err(_) => Ok(blocked_boolean_boundary_fragment_emission_result(
                self,
                BooleanBoundaryFragmentEmissionStage2::FragmentEmission,
                UncertaintyReason::Unsupported,
            )),
        }
    }
}

impl BooleanBoundaryFragmentEmissionReport2 {
    /// Returns the furthest exact emission stage reached.
    pub const fn stage(&self) -> BooleanBoundaryFragmentEmissionStage2 {
        self.stage
    }

    /// Returns source classification count.
    pub const fn source_classification_count(&self) -> usize {
        self.source_classification_count
    }

    /// Returns discard action count.
    pub const fn discard_count(&self) -> usize {
        self.discard_count
    }

    /// Returns source-direction emission action count.
    pub const fn keep_source_direction_count(&self) -> usize {
        self.keep_source_direction_count
    }

    /// Returns reversed emission action count.
    pub const fn keep_reversed_count(&self) -> usize {
        self.keep_reversed_count
    }

    /// Returns unresolved-boundary action count.
    pub const fn boundary_needs_resolution_count(&self) -> usize {
        self.boundary_needs_resolution_count
    }

    /// Returns emitted directed fragment count when materialized.
    pub const fn directed_fragment_count(&self) -> Option<usize> {
        self.directed_fragment_count
    }

    /// Returns primitive-family counts for emitted source segments when materialized.
    pub const fn directed_source_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.directed_source_segment_kind_counts
    }

    /// Returns primitive-family counts for emitted directed fragments when materialized.
    pub const fn directed_fragment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.directed_fragment_kind_counts
    }

    /// Returns provenance for emitted directed fragments when materialized.
    pub fn directed_fragments(&self) -> &[BooleanDirectedFragmentReport2] {
        &self.directed_fragments
    }

    /// Returns unresolved boundary fragment count when materialized.
    pub const fn unresolved_boundary_count(&self) -> Option<usize> {
        self.unresolved_boundary_count
    }

    /// Returns retained topology status for emission.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized emission.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl BooleanDirectedFragmentReport2 {
    /// Returns the source keyed contour.
    pub const fn key(&self) -> RegionContourKey {
        self.key
    }

    /// Returns the source contour fragment index.
    pub const fn fragment_index(&self) -> usize {
        self.fragment_index
    }

    /// Returns the source segment index in the original contour.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the source segment primitive kind in the original contour.
    pub const fn source_segment_kind(&self) -> SegmentKind {
        self.source_segment_kind
    }

    /// Returns the exact start point of the original source segment.
    pub const fn source_segment_start_point(&self) -> &Point2 {
        &self.source_segment_start_point
    }

    /// Returns the exact end point of the original source segment.
    pub const fn source_segment_end_point(&self) -> &Point2 {
        &self.source_segment_end_point
    }

    /// Returns the retained parameter range on the source segment.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns true when the output fragment was emitted opposite source traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the output directed-fragment index.
    pub const fn output_fragment_index(&self) -> usize {
        self.output_fragment_index
    }

    /// Returns the output directed-fragment primitive kind.
    pub const fn output_fragment_kind(&self) -> SegmentKind {
        self.output_fragment_kind
    }
}

impl BooleanBoundaryFragmentEmissionResult2 {
    /// Returns emitted boundary fragments, if emission succeeded.
    pub const fn fragments(&self) -> Option<&BooleanBoundaryFragmentSet> {
        self.fragments.as_ref()
    }

    /// Consumes this result and returns emitted boundary fragments, if any.
    pub fn into_fragments(self) -> Option<BooleanBoundaryFragmentSet> {
        self.fragments
    }

    /// Consumes this result and returns retained emission evidence.
    pub fn into_report(self) -> BooleanBoundaryFragmentEmissionReport2 {
        self.report
    }

    /// Consumes this result and returns emitted boundary fragments with their report.
    pub fn into_parts(
        self,
    ) -> (
        Option<BooleanBoundaryFragmentSet>,
        BooleanBoundaryFragmentEmissionReport2,
    ) {
        (self.fragments, self.report)
    }

    /// Returns retained emission evidence.
    pub const fn report(&self) -> &BooleanBoundaryFragmentEmissionReport2 {
        &self.report
    }

    /// Returns emitted boundary fragments as a classification while retaining this result.
    pub fn fragments_classification(&self) -> Classification<&BooleanBoundaryFragmentSet> {
        match self.fragments() {
            Some(fragments) => Classification::Decided(fragments),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns emitted boundary fragments as a classification.
    pub fn into_fragments_classification(self) -> Classification<BooleanBoundaryFragmentSet> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_fragments() {
            Some(fragments) => Classification::Decided(fragments),
            None => Classification::Uncertain(blocker),
        }
    }
}

fn blocked_boolean_boundary_fragment_emission_result(
    selection: &BooleanFragmentSelection,
    stage: BooleanBoundaryFragmentEmissionStage2,
    blocker: UncertaintyReason,
) -> BooleanBoundaryFragmentEmissionResult2 {
    BooleanBoundaryFragmentEmissionResult2 {
        fragments: None,
        report: BooleanBoundaryFragmentEmissionReport2 {
            stage,
            source_classification_count: selection.len(),
            discard_count: selection.count_action(BooleanFragmentAction::Discard),
            keep_source_direction_count: selection
                .count_action(BooleanFragmentAction::KeepSourceDirection),
            keep_reversed_count: selection.count_action(BooleanFragmentAction::KeepReversed),
            boundary_needs_resolution_count: selection
                .count_action(BooleanFragmentAction::BoundaryNeedsResolution),
            directed_fragment_count: None,
            directed_source_segment_kind_counts: None,
            directed_fragment_kind_counts: None,
            directed_fragments: Vec::new(),
            unresolved_boundary_count: None,
            status: RetainedTopologyStatus::Unsupported,
            blocker: Some(blocker),
        },
    }
}

fn boolean_directed_fragment_reports(
    fragments: &[DirectedBooleanFragment],
) -> Vec<BooleanDirectedFragmentReport2> {
    fragments
        .iter()
        .enumerate()
        .map(
            |(output_fragment_index, fragment)| BooleanDirectedFragmentReport2 {
                key: fragment.key,
                fragment_index: fragment.fragment_index,
                source_segment_index: fragment.source_segment_index,
                source_segment_kind: fragment.segment.structural_facts().kind,
                source_segment_start_point: fragment.source_segment_start_point.clone(),
                source_segment_end_point: fragment.source_segment_end_point.clone(),
                source_range: fragment.source_range.clone(),
                reversed: fragment.reversed,
                output_fragment_index,
                output_fragment_kind: fragment.segment.structural_facts().kind,
            },
        )
        .collect()
}

fn boolean_directed_fragment_report_source_kind_counts(
    fragments: &[BooleanDirectedFragmentReport2],
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for fragment in fragments {
        match fragment.source_segment_kind {
            SegmentKind::Line => counts.lines += 1,
            SegmentKind::Arc => counts.arcs += 1,
        }
    }
    counts
}

impl BooleanOp {
    pub(crate) const fn apply(self, first: bool, second: bool) -> bool {
        match self {
            Self::Union => first || second,
            Self::Intersection => first && second,
            Self::Difference => first && !second,
            Self::Xor => first != second,
        }
    }

    fn action_for(
        self,
        source_side: RegionSide,
        source_filled_side_is_left: bool,
        opposite_location: RegionPointLocation,
    ) -> BooleanFragmentAction {
        use BooleanFragmentAction::{
            BoundaryNeedsResolution, Discard, KeepReversed, KeepSourceDirection,
        };
        use RegionPointLocation::{Boundary, Inside, Outside};
        use RegionSide::{First, Second};

        let material_action = match opposite_location {
            Boundary => BoundaryNeedsResolution,
            Outside => match self {
                Self::Union | Self::Difference | Self::Xor => {
                    if source_side == Second && self == Self::Difference {
                        Discard
                    } else {
                        KeepSourceDirection
                    }
                }
                Self::Intersection => Discard,
            },
            Inside => match self {
                Self::Intersection => KeepSourceDirection,
                Self::Difference => {
                    if source_side == First {
                        Discard
                    } else {
                        KeepReversed
                    }
                }
                Self::Union => Discard,
                Self::Xor => KeepReversed,
            },
        };

        if source_filled_side_is_left {
            material_action
        } else {
            reverse_emitted_action(material_action)
        }
    }
}

pub(crate) fn source_contour_filled_side_is_left(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    key: RegionContourKey,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let view = match key.side {
        RegionSide::First => first,
        RegionSide::Second => second,
    };
    let contours = match key.role {
        RegionContourRole::Material => view.material_contours(),
        RegionContourRole::Hole => view.hole_contours(),
    };
    let contour = contours.get(key.index).copied().ok_or_else(|| {
        CurveError::Topology("boolean classification references a missing contour".into())
    })?;
    let Some(area) = contour.signed_area()? else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let interior_left = match real_sign(&area, policy) {
        Some(RealSign::Positive) => true,
        Some(RealSign::Negative) => false,
        Some(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    Ok(Classification::Decided(match key.role {
        RegionContourRole::Material => interior_left,
        RegionContourRole::Hole => !interior_left,
    }))
}

fn reverse_emitted_action(action: BooleanFragmentAction) -> BooleanFragmentAction {
    use BooleanFragmentAction::{
        BoundaryNeedsResolution, Discard, KeepReversed, KeepSourceDirection,
    };

    // Region contour bins carry signed fill roles independently of storage
    // direction. Whenever the source region is filled right of traversal, the
    // output direction is the opposite of the canonical filled-left action.
    // In fill-state clipping terms, this is the signed-contour equivalent of
    // flipping the transition direction for a right-filled edge.
    match action {
        KeepSourceDirection => KeepReversed,
        KeepReversed => KeepSourceDirection,
        Discard => Discard,
        BoundaryNeedsResolution => BoundaryNeedsResolution,
    }
}

fn fragment_for_classification<'a>(
    fragments: &'a RegionFragmentSet,
    classification: &BooleanFragmentClassification,
) -> CurveResult<&'a crate::ContourFragment> {
    let contour_fragments = fragments
        .fragments_for_contour(classification.key)
        .ok_or_else(|| {
            CurveError::Topology("boolean classification references a missing contour".into())
        })?;
    contour_fragments
        .fragments
        .fragments()
        .get(classification.fragment_index)
        .ok_or_else(|| {
            CurveError::Topology("boolean classification references a missing fragment".into())
        })
}

fn validate_boolean_fragment_classifications(
    classifications: &[BooleanFragmentClassification],
) -> CurveResult<()> {
    for classification in classifications {
        validate_boolean_fragment_classification_boundary_action(classification)?;
    }

    let mut owners = classifications
        .iter()
        .map(|classification| (classification.key, classification.fragment_index))
        .collect::<Vec<_>>();
    owners.sort_unstable();
    if owners.windows(2).any(|window| window[0] == window[1]) {
        return Err(CurveError::Topology(
            "boolean fragment selection must not classify the same source fragment twice".into(),
        ));
    }
    Ok(())
}

fn validate_boolean_selection_matches_fragments(
    classifications: &[BooleanFragmentClassification],
    fragments: &RegionFragmentSet,
) -> CurveResult<()> {
    let mut classified_owners = Vec::with_capacity(classifications.len());
    for classification in classifications {
        let Some(contour_fragments) = fragments.fragments_for_contour(classification.key) else {
            return Err(CurveError::Topology(
                "boolean classification references a contour outside supplied fragments".into(),
            ));
        };
        if classification.fragment_index >= contour_fragments.fragments.len() {
            return Err(CurveError::Topology(
                "boolean classification references a fragment outside supplied fragments".into(),
            ));
        }
        classified_owners.push((classification.key, classification.fragment_index));
    }

    let mut expected_owners = Vec::new();
    for contour_fragments in fragments.contours() {
        expected_owners.reserve(contour_fragments.fragments.len());
        for fragment_index in 0..contour_fragments.fragments.len() {
            expected_owners.push((contour_fragments.key, fragment_index));
        }
    }

    classified_owners.sort_unstable();
    expected_owners.sort_unstable();
    if classified_owners != expected_owners {
        return Err(CurveError::Topology(
            "boolean fragment selection must classify every supplied source fragment exactly once"
                .into(),
        ));
    }

    Ok(())
}

pub(crate) fn validate_boolean_fragment_classification_boundary_action(
    classification: &BooleanFragmentClassification,
) -> CurveResult<()> {
    match (classification.opposite_location, classification.action) {
        (RegionPointLocation::Boundary, BooleanFragmentAction::BoundaryNeedsResolution) => Ok(()),
        (RegionPointLocation::Boundary, _) => Err(CurveError::Topology(
            "boolean boundary classification must remain unresolved".into(),
        )),
        (_, BooleanFragmentAction::BoundaryNeedsResolution) => Err(CurveError::Topology(
            "boolean unresolved classification must carry boundary evidence".into(),
        )),
        _ => Ok(()),
    }
}

impl RegionFragmentSet {
    /// Classifies fragments against the opposite region for a boolean operation.
    ///
    /// This is the local selection stage used by planar clipping algorithms
    /// after intersection insertion. `hypercurve` keeps the stage explicit and
    /// returns `BoundaryNeedsResolution` instead of folding shared boundaries
    /// into an epsilon-based inside/outside decision.
    pub fn classify_for_boolean(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanFragmentSelection>> {
        Ok(self
            .classify_for_boolean_with_report(first, second, op, policy)?
            .into_selection_classification())
    }

    /// Classifies fragments against the opposite region and retains selection evidence.
    pub fn classify_for_boolean_with_report(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<BooleanFragmentSelectionResult2> {
        self.classify_for_boolean_with_point_classifier_with_report(
            first,
            second,
            op,
            policy,
            |source_side, sample| {
                let opposite = match source_side {
                    RegionSide::First => second,
                    RegionSide::Second => first,
                };
                opposite.classify_point(sample, policy)
            },
        )
    }

    /// Classifies fragments using a caller-supplied opposite-region point
    /// classifier.
    ///
    /// Prepared boolean paths use this hook to keep the exact same fragment
    /// selection rules while reusing cached region classifiers.
    pub(crate) fn classify_for_boolean_with_point_classifier<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
        mut classify_opposite: F,
    ) -> CurveResult<Classification<BooleanFragmentSelection>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<RegionPointLocation>,
    {
        let result = self.classify_for_boolean_with_point_classifier_with_report(
            first,
            second,
            op,
            policy,
            &mut classify_opposite,
        )?;
        Ok(result.into_selection_classification())
    }

    pub(crate) fn classify_for_boolean_with_point_classifier_with_report<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
        mut classify_opposite: F,
    ) -> CurveResult<BooleanFragmentSelectionResult2>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<RegionPointLocation>,
    {
        let mut classifications = Vec::new();
        let source_contour_count = self.len();
        let source_fragment_count = region_fragment_count(self);
        let source_fragment_kind_counts = region_fragment_kind_counts(self);

        for contour_fragments in self.contours() {
            let source_filled_side_is_left = match source_contour_filled_side_is_left(
                first,
                second,
                contour_fragments.key,
                policy,
            )? {
                Classification::Decided(filled_side) => filled_side,
                Classification::Uncertain(reason) => {
                    return Ok(blocked_boolean_fragment_selection_result(
                        op,
                        BooleanFragmentSelectionStage2::SourceFillSideClassification,
                        source_contour_count,
                        source_fragment_count,
                        source_fragment_kind_counts,
                        classifications.len(),
                        reason,
                    ));
                }
            };
            for (fragment_index, fragment) in
                contour_fragments.fragments.fragments().iter().enumerate()
            {
                let source_side = contour_fragments.key.side;
                let opposite_location =
                    match classify_fragment_interior(&fragment.segment, policy, |sample| {
                        classify_opposite(source_side, sample)
                    })? {
                        FragmentInteriorClassification::Decided(location) => location,
                        FragmentInteriorClassification::Blocked { stage, reason } => {
                            return Ok(blocked_boolean_fragment_selection_result(
                                op,
                                stage,
                                source_contour_count,
                                source_fragment_count,
                                source_fragment_kind_counts,
                                classifications.len(),
                                reason,
                            ));
                        }
                    };
                let action =
                    op.action_for(source_side, source_filled_side_is_left, opposite_location);

                classifications.push(BooleanFragmentClassification {
                    key: contour_fragments.key,
                    fragment_index,
                    opposite_location,
                    source_filled_side_is_left,
                    action,
                });
            }
        }

        let selection = BooleanFragmentSelection::new(classifications)?;
        Ok(BooleanFragmentSelectionResult2 {
            report: boolean_fragment_selection_report_from_classifications(
                op,
                BooleanFragmentSelectionStage2::ActionAssignment,
                source_contour_count,
                source_fragment_count,
                source_fragment_kind_counts,
                selection.classifications(),
                RetainedTopologyStatus::NativeExact,
                None,
            ),
            selection: Some(selection),
        })
    }
}

fn classify_fragment_interior<F>(
    segment: &Segment2,
    policy: &CurvePolicy,
    mut classify: F,
) -> CurveResult<FragmentInteriorClassification>
where
    F: FnMut(&Point2) -> Classification<RegionPointLocation>,
{
    let fractions = [
        (Real::one() / Real::from(2_i8))?,
        (Real::one() / Real::from(3_i8))?,
        (Real::from(2_i8) / Real::from(3_i8))?,
    ];
    let mut representative_blocker = None;
    let mut classification_blocker = None;

    for fraction in &fractions {
        let sample = match segment.point_at(fraction, policy)? {
            Classification::Decided(sample) => sample,
            Classification::Uncertain(reason) => {
                representative_blocker.get_or_insert(reason);
                continue;
            }
        };
        match classify(&sample) {
            Classification::Decided(location) => {
                return Ok(FragmentInteriorClassification::Decided(location));
            }
            Classification::Uncertain(reason) => {
                classification_blocker.get_or_insert(reason);
            }
        }
    }

    if let Some(reason) = classification_blocker {
        return Ok(FragmentInteriorClassification::Blocked {
            stage: BooleanFragmentSelectionStage2::OppositeRegionClassification,
            reason,
        });
    }
    Ok(FragmentInteriorClassification::Blocked {
        stage: BooleanFragmentSelectionStage2::RepresentativePoint,
        reason: representative_blocker.unwrap_or(UncertaintyReason::Unsupported),
    })
}

impl BooleanFragmentSelectionReport2 {
    /// Returns the requested boolean operation.
    pub const fn op(&self) -> BooleanOp {
        self.op
    }

    /// Returns the furthest exact classification stage reached.
    pub const fn stage(&self) -> BooleanFragmentSelectionStage2 {
        self.stage
    }

    /// Returns keyed source contour count.
    pub const fn source_contour_count(&self) -> usize {
        self.source_contour_count
    }

    /// Returns total source fragment count.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }

    /// Returns primitive-family counts for all source fragments.
    pub const fn source_fragment_kind_counts(&self) -> SegmentKindCounts {
        self.source_fragment_kind_counts
    }

    /// Returns classified fragment count when available.
    pub const fn classified_fragment_count(&self) -> Option<usize> {
        self.classified_fragment_count
    }

    /// Returns discard action count when classification materialized.
    pub const fn discard_count(&self) -> Option<usize> {
        self.discard_count
    }

    /// Returns source-direction emitted action count when classification materialized.
    pub const fn keep_source_direction_count(&self) -> Option<usize> {
        self.keep_source_direction_count
    }

    /// Returns reversed emitted action count when classification materialized.
    pub const fn keep_reversed_count(&self) -> Option<usize> {
        self.keep_reversed_count
    }

    /// Returns unresolved-boundary action count when classification materialized.
    pub const fn boundary_needs_resolution_count(&self) -> Option<usize> {
        self.boundary_needs_resolution_count
    }

    /// Returns retained topology status for classification.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized classification.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl BooleanFragmentSelectionResult2 {
    /// Returns materialized fragment selection, if classification succeeded.
    pub const fn selection(&self) -> Option<&BooleanFragmentSelection> {
        self.selection.as_ref()
    }

    /// Consumes this result and returns the materialized selection, if any.
    pub fn into_selection(self) -> Option<BooleanFragmentSelection> {
        self.selection
    }

    /// Consumes this result and returns retained selection evidence.
    pub fn into_report(self) -> BooleanFragmentSelectionReport2 {
        self.report
    }

    /// Consumes this result and returns materialized selection with its report.
    pub fn into_parts(
        self,
    ) -> (
        Option<BooleanFragmentSelection>,
        BooleanFragmentSelectionReport2,
    ) {
        (self.selection, self.report)
    }

    /// Returns retained selection evidence.
    pub const fn report(&self) -> &BooleanFragmentSelectionReport2 {
        &self.report
    }

    /// Returns materialized fragment selection as a classification while retaining this result.
    pub fn selection_classification(&self) -> Classification<&BooleanFragmentSelection> {
        match self.selection() {
            Some(selection) => Classification::Decided(selection),
            None => Classification::Uncertain(
                self.report()
                    .blocker()
                    .unwrap_or(UncertaintyReason::Unsupported),
            ),
        }
    }

    /// Consumes this result and returns materialized fragment selection as a classification.
    pub fn into_selection_classification(self) -> Classification<BooleanFragmentSelection> {
        let blocker = self
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match self.into_selection() {
            Some(selection) => Classification::Decided(selection),
            None => Classification::Uncertain(blocker),
        }
    }
}

fn blocked_boolean_fragment_selection_result(
    op: BooleanOp,
    stage: BooleanFragmentSelectionStage2,
    source_contour_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    classified_fragment_count: usize,
    blocker: UncertaintyReason,
) -> BooleanFragmentSelectionResult2 {
    BooleanFragmentSelectionResult2 {
        selection: None,
        report: BooleanFragmentSelectionReport2 {
            op,
            stage,
            source_contour_count,
            source_fragment_count,
            source_fragment_kind_counts,
            classified_fragment_count: Some(classified_fragment_count),
            discard_count: None,
            keep_source_direction_count: None,
            keep_reversed_count: None,
            boundary_needs_resolution_count: None,
            status: RetainedTopologyStatus::Unresolved,
            blocker: Some(blocker),
        },
    }
}

fn boolean_fragment_selection_report_from_classifications(
    op: BooleanOp,
    stage: BooleanFragmentSelectionStage2,
    source_contour_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    classifications: &[BooleanFragmentClassification],
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> BooleanFragmentSelectionReport2 {
    BooleanFragmentSelectionReport2 {
        op,
        stage,
        source_contour_count,
        source_fragment_count,
        source_fragment_kind_counts,
        classified_fragment_count: Some(classifications.len()),
        discard_count: Some(count_boolean_action(
            classifications,
            BooleanFragmentAction::Discard,
        )),
        keep_source_direction_count: Some(count_boolean_action(
            classifications,
            BooleanFragmentAction::KeepSourceDirection,
        )),
        keep_reversed_count: Some(count_boolean_action(
            classifications,
            BooleanFragmentAction::KeepReversed,
        )),
        boundary_needs_resolution_count: Some(count_boolean_action(
            classifications,
            BooleanFragmentAction::BoundaryNeedsResolution,
        )),
        status,
        blocker,
    }
}

fn count_boolean_action(
    classifications: &[BooleanFragmentClassification],
    action: BooleanFragmentAction,
) -> usize {
    classifications
        .iter()
        .filter(|classification| classification.action == action)
        .count()
}

fn region_fragment_count(fragments: &RegionFragmentSet) -> usize {
    fragments
        .contours()
        .iter()
        .map(|contour_fragments| contour_fragments.fragments.len())
        .sum()
}

fn region_fragment_kind_counts(fragments: &RegionFragmentSet) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for contour_fragments in fragments.contours() {
        for fragment in contour_fragments.fragments.fragments() {
            add_segment_kind_count(&mut counts, &fragment.segment);
        }
    }
    counts
}

fn directed_boolean_fragment_kind_counts(
    fragments: &[DirectedBooleanFragment],
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for fragment in fragments {
        add_segment_kind_count(&mut counts, &fragment.segment);
    }
    counts
}

fn add_segment_kind_count(counts: &mut SegmentKindCounts, segment: &Segment2) {
    match segment {
        Segment2::Line(_) => counts.lines += 1,
        Segment2::Arc(_) => counts.arcs += 1,
    }
}
