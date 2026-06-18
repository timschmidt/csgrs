//! Boolean fragment classification.
//!
//! This module is the split/classify/select layer before graph traversal and
//! loop assembly. It deliberately does not resolve shared-boundary fragments:
//! those need overlap-aware traversal, not a midpoint guess.

use crate::boolean_boundary::{BooleanBoundaryFragmentSet, DirectedBooleanFragment};
use crate::{
    Classification, CurveError, CurvePolicy, CurveResult, RegionContourRole, RegionFragmentSet,
    RegionPointLocation, RegionSide, RegionView2, RetainedTopologyStatus, UncertaintyReason,
};

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

    /// Converts selected classifications into directed boundary fragments.
    ///
    /// This performs the "emit in source direction or reverse direction" step
    /// after local boolean classification. Greiner-Hormann style traversal
    /// follows selected directed polygon chains after entry/exit classification
    /// (G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
    /// ACM Transactions on Graphics 17(2), 71-83, 1998). We keep shared
    /// boundaries in `unresolved_boundaries` instead of applying a local
    /// tie-breaker because degenerate polygon clipping papers, including
    /// Foster, Hormann, and Popa, "Clipping simple polygons with degenerate
    /// intersections," Computers & Graphics: X 2, 100007, 2019, show that
    /// boundary coincidences need explicit handling separate from ordinary
    /// enter/exit classification.
    pub fn emit_boundary_fragments(
        &self,
        fragments: &RegionFragmentSet,
    ) -> CurveResult<BooleanBoundaryFragmentSet> {
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
                        segment,
                    });
                }
            }
        }

        BooleanBoundaryFragmentSet::new(directed_fragments, unresolved_boundaries)
    }
}

impl BooleanOp {
    fn action_for(
        self,
        source_side: RegionSide,
        source_role: RegionContourRole,
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

        match source_role {
            RegionContourRole::Material => material_action,
            RegionContourRole::Hole => reverse_emitted_action(material_action),
        }
    }
}

fn reverse_emitted_action(action: BooleanFragmentAction) -> BooleanFragmentAction {
    use BooleanFragmentAction::{
        BoundaryNeedsResolution, Discard, KeepReversed, KeepSourceDirection,
    };

    // Region contour bins carry signed fill roles independently of storage
    // direction. When a selected source fragment belongs to a hole contour, the
    // local boundary orientation is the opposite of an equivalent material
    // contour. Vatti frames clipping output as fill-state transitions
    // (B. R. Vatti, "A generic solution to polygon clipping," Communications
    // of the ACM 35(7), 56-63, 1992); this is the signed-contour equivalent of
    // flipping the transition direction for negative fill edges.
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
    /// Algorithm note: this is the local selection stage used by many planar
    /// clipping algorithms after intersection insertion. Greiner and Hormann
    /// mark split polygon chains as entry/exit after intersections are inserted
    /// (G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
    /// ACM Transactions on Graphics 17(2), 71-83, 1998). Vatti's sweep-line
    /// algorithm also reduces result construction to sorted edge events and
    /// fill-state transitions (B. R. Vatti, "A generic solution to polygon
    /// clipping," Communications of the ACM 35(7), 56-63, 1992). Martinez,
    /// Rueda, and Feito formalize boolean selection from segment classifications
    /// for general polygons (F. Martinez, A. J. Rueda, and F. R. Feito, "A new
    /// algorithm for computing Boolean operations on polygons," Computers &
    /// Geosciences 35(6), 1177-1185, 2009). `hypercurve` keeps this stage
    /// explicit and returns `BoundaryNeedsResolution` instead of folding shared
    /// boundaries into an epsilon-based inside/outside decision.
    pub fn classify_for_boolean(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanFragmentSelection>> {
        let result = self.classify_for_boolean_with_report(first, second, op, policy)?;
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(selection) = result.into_selection() {
            Ok(Classification::Decided(selection))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
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
        op: BooleanOp,
        policy: &CurvePolicy,
        mut classify_opposite: F,
    ) -> CurveResult<Classification<BooleanFragmentSelection>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<RegionPointLocation>,
    {
        let result = self.classify_for_boolean_with_point_classifier_with_report(
            op,
            policy,
            &mut classify_opposite,
        )?;
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(selection) = result.into_selection() {
            Ok(Classification::Decided(selection))
        } else {
            Ok(Classification::Uncertain(blocker))
        }
    }

    pub(crate) fn classify_for_boolean_with_point_classifier_with_report<F>(
        &self,
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

        for contour_fragments in self.contours() {
            for (fragment_index, fragment) in
                contour_fragments.fragments.fragments().iter().enumerate()
            {
                let sample = match fragment.segment.representative_point(policy)? {
                    Classification::Decided(sample) => sample,
                    Classification::Uncertain(reason) => {
                        return Ok(blocked_boolean_fragment_selection_result(
                            op,
                            BooleanFragmentSelectionStage2::RepresentativePoint,
                            source_contour_count,
                            source_fragment_count,
                            classifications.len(),
                            reason,
                        ));
                    }
                };
                let source_side = contour_fragments.key.side;
                let opposite_location = match classify_opposite(source_side, &sample) {
                    Classification::Decided(location) => location,
                    Classification::Uncertain(reason) => {
                        return Ok(blocked_boolean_fragment_selection_result(
                            op,
                            BooleanFragmentSelectionStage2::OppositeRegionClassification,
                            source_contour_count,
                            source_fragment_count,
                            classifications.len(),
                            reason,
                        ));
                    }
                };
                let action =
                    op.action_for(source_side, contour_fragments.key.role, opposite_location);

                classifications.push(BooleanFragmentClassification {
                    key: contour_fragments.key,
                    fragment_index,
                    opposite_location,
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
                selection.classifications(),
                RetainedTopologyStatus::NativeExact,
                None,
            ),
            selection: Some(selection),
        })
    }
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

    /// Returns retained selection evidence.
    pub const fn report(&self) -> &BooleanFragmentSelectionReport2 {
        &self.report
    }
}

fn blocked_boolean_fragment_selection_result(
    op: BooleanOp,
    stage: BooleanFragmentSelectionStage2,
    source_contour_count: usize,
    source_fragment_count: usize,
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
    classifications: &[BooleanFragmentClassification],
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> BooleanFragmentSelectionReport2 {
    BooleanFragmentSelectionReport2 {
        op,
        stage,
        source_contour_count,
        source_fragment_count,
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
