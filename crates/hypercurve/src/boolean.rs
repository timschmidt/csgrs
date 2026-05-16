//! Boolean fragment classification.
//!
//! This module is the split/classify/select layer before graph traversal and
//! loop assembly. It deliberately does not resolve shared-boundary fragments:
//! those need overlap-aware traversal, not a midpoint guess.

use crate::boolean_boundary::{BooleanBoundaryFragmentSet, DirectedBooleanFragment};
use crate::{
    Classification, CurveError, CurvePolicy, CurveResult, RegionContourRole, RegionFragmentSet,
    RegionPointLocation, RegionSide, RegionView2,
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

impl BooleanFragmentSelection {
    /// Constructs a selection from already-classified fragments.
    pub const fn new(classifications: Vec<BooleanFragmentClassification>) -> Self {
        Self { classifications }
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
                    let segment = match classification.action {
                        BooleanFragmentAction::KeepSourceDirection => source.segment.clone(),
                        BooleanFragmentAction::KeepReversed => source.segment.reversed(),
                        BooleanFragmentAction::Discard
                        | BooleanFragmentAction::BoundaryNeedsResolution => unreachable!(),
                    };
                    directed_fragments.push(DirectedBooleanFragment {
                        key: classification.key,
                        fragment_index: classification.fragment_index,
                        segment,
                    });
                }
            }
        }

        Ok(BooleanBoundaryFragmentSet::new(
            directed_fragments,
            unresolved_boundaries,
        ))
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
        self.classify_for_boolean_with_point_classifier(op, policy, |source_side, sample| {
            let opposite = match source_side {
                RegionSide::First => second,
                RegionSide::Second => first,
            };
            opposite.classify_point(sample, policy)
        })
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
        let mut classifications = Vec::new();

        for contour_fragments in self.contours() {
            for (fragment_index, fragment) in
                contour_fragments.fragments.fragments().iter().enumerate()
            {
                let sample = match fragment.segment.representative_point(policy)? {
                    Classification::Decided(sample) => sample,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let source_side = contour_fragments.key.side;
                let opposite_location = match classify_opposite(source_side, &sample) {
                    Classification::Decided(location) => location,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
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

        Ok(Classification::Decided(BooleanFragmentSelection::new(
            classifications,
        )))
    }
}
