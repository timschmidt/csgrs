//! Boolean fragment classification.
//!
//! This module is the split/classify/select layer before graph traversal and
//! loop assembly. It deliberately does not resolve shared-boundary fragments:
//! those need overlap-aware traversal, not a midpoint guess.

use crate::boolean_boundary::{
    BooleanBoundaryChainIndices, BooleanBoundaryFragmentSet, BorrowedBooleanBoundaryEdge,
    DirectedBooleanFragment, endpoint_chain_indices, materialize_segment_contours,
};
use crate::classify::real_sign;
use crate::region_crossing_winding::RegionLineCrossingWindingIndex;
use crate::region_fragments::CompactLineRegionFragmentSet;
use crate::{
    Classification, CurveContext, CurveError, CurveResult, FillRule, Point2, RegionContourKey,
    RegionContourRole, RegionFragmentSet, RegionPointLocation, RegionSide, RegionView2, Segment2,
    UncertaintyReason,
};
use hyperreal::{Real, RealSign};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CertifiedFragmentEndpoint {
    Start,
    End,
}

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

enum FragmentInteriorClassification<T> {
    Decided(T),
    Blocked(UncertaintyReason),
}

impl BooleanFragmentSelection {
    /// Constructs a selection from already-classified fragments.
    pub fn new(classifications: Vec<BooleanFragmentClassification>) -> CurveResult<Self> {
        validate_boolean_fragment_classifications(&classifications)?;
        Ok(Self { classifications })
    }

    fn from_complete_fragment_traversal(
        classifications: Vec<BooleanFragmentClassification>,
        source_fragment_count: usize,
    ) -> Self {
        debug_assert_eq!(classifications.len(), source_fragment_count);
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

    fn emitted_fragment_count(&self) -> usize {
        self.classifications
            .iter()
            .filter(|classification| classification.action.emits_fragment())
            .count()
    }

    pub(crate) fn endpoint_chain_indices_from_certified_split(
        &self,
        fragments: &RegionFragmentSet,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BooleanBoundaryChainIndices>>> {
        let mut sources = fragments.contours().iter().flat_map(|contour| {
            let key = contour.key;
            contour
                .fragments
                .fragments()
                .iter()
                .enumerate()
                .map(move |(index, source)| (key, index, source))
        });
        let mut endpoints = Vec::with_capacity(self.emitted_fragment_count());
        for classification in &self.classifications {
            let Some((key, fragment_index, source)) = sources.next() else {
                return Err(CurveError::Topology(
                    "boolean selection references a fragment outside certified split output".into(),
                ));
            };
            if classification.key != key || classification.fragment_index != fragment_index {
                return Err(CurveError::Topology(
                    "boolean selection order differs from certified split order".into(),
                ));
            }
            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                BooleanFragmentAction::KeepSourceDirection => {
                    endpoints.push(BorrowedBooleanBoundaryEdge::new(&source.segment, false));
                }
                BooleanFragmentAction::KeepReversed => {
                    endpoints.push(BorrowedBooleanBoundaryEdge::new(&source.segment, true));
                }
            }
        }
        if sources.next().is_some() {
            return Err(CurveError::Topology(
                "boolean selection omits a supplied source fragment".into(),
            ));
        }
        Ok(match endpoint_chain_indices(&endpoints, policy) {
            Ok(chain_indices) => Classification::Decided(chain_indices),
            Err(reason) => Classification::Uncertain(reason),
        })
    }

    pub(crate) fn emit_contours_from_owned_certified_split(
        self,
        fragments: RegionFragmentSet,
        chain_indices: BooleanBoundaryChainIndices,
        fill_rule: FillRule,
    ) -> CurveResult<Classification<Vec<crate::Contour2>>> {
        let mut sources = fragments.into_contours().into_iter().flat_map(|contour| {
            let key = contour.key;
            contour
                .fragments
                .into_fragments()
                .into_iter()
                .enumerate()
                .map(move |(index, source)| (key, index, source))
        });
        let mut segments = Vec::with_capacity(self.emitted_fragment_count());
        for classification in self.classifications {
            let Some((key, fragment_index, source)) = sources.next() else {
                return Err(CurveError::Topology(
                    "boolean selection references a fragment outside certified split output".into(),
                ));
            };
            if classification.key != key || classification.fragment_index != fragment_index {
                return Err(CurveError::Topology(
                    "boolean selection order differs from certified split order".into(),
                ));
            }
            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                BooleanFragmentAction::KeepSourceDirection => segments.push(source.segment),
                BooleanFragmentAction::KeepReversed => {
                    segments.push(source.segment.into_reversed());
                }
            }
        }
        if sources.next().is_some() {
            return Err(CurveError::Topology(
                "boolean selection omits a supplied source fragment".into(),
            ));
        }
        Ok(materialize_segment_contours(
            chain_indices,
            segments,
            fill_rule,
        ))
    }

    pub(crate) fn endpoint_chain_indices_from_compact_split(
        &self,
        fragments: &CompactLineRegionFragmentSet,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BooleanBoundaryChainIndices>>> {
        let mut sources = fragments.contours().iter().flat_map(|contour| {
            let key = contour.key;
            contour
                .fragments
                .iter()
                .enumerate()
                .map(move |(index, source)| (key, index, source))
        });
        let mut endpoints = Vec::with_capacity(self.emitted_fragment_count());
        for classification in &self.classifications {
            let Some((key, fragment_index, source)) = sources.next() else {
                return Err(CurveError::Topology(
                    "boolean selection references a fragment outside certified split output".into(),
                ));
            };
            if classification.key != key || classification.fragment_index != fragment_index {
                return Err(CurveError::Topology(
                    "boolean selection order differs from certified split order".into(),
                ));
            }
            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                BooleanFragmentAction::KeepSourceDirection => {
                    let source_segment = compact_source_segment(first, second, key, source)?;
                    let (start, end) = source.endpoints(source_segment, key, crossing_windings)?;
                    endpoints.push(BorrowedBooleanBoundaryEdge::from_endpoints(
                        start, end, false,
                    ));
                }
                BooleanFragmentAction::KeepReversed => {
                    let source_segment = compact_source_segment(first, second, key, source)?;
                    let (start, end) = source.endpoints(source_segment, key, crossing_windings)?;
                    endpoints.push(BorrowedBooleanBoundaryEdge::from_endpoints(
                        start, end, true,
                    ));
                }
            }
        }
        if sources.next().is_some() {
            return Err(CurveError::Topology(
                "boolean selection omits a supplied source fragment".into(),
            ));
        }
        Ok(match endpoint_chain_indices(&endpoints, policy) {
            Ok(chain_indices) => Classification::Decided(chain_indices),
            Err(reason) => Classification::Uncertain(reason),
        })
    }

    pub(crate) fn emit_contours_from_owned_compact_split(
        self,
        fragments: CompactLineRegionFragmentSet,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        chain_indices: BooleanBoundaryChainIndices,
        fill_rule: FillRule,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
    ) -> CurveResult<Classification<Vec<crate::Contour2>>> {
        let mut sources = fragments.contours().iter().flat_map(|contour| {
            let key = contour.key;
            contour
                .fragments
                .iter()
                .enumerate()
                .map(move |(index, source)| (key, index, source))
        });
        let mut selected = Vec::with_capacity(self.emitted_fragment_count());
        for classification in self.classifications {
            let Some((key, fragment_index, source)) = sources.next() else {
                return Err(CurveError::Topology(
                    "boolean selection references a fragment outside certified split output".into(),
                ));
            };
            if classification.key != key || classification.fragment_index != fragment_index {
                return Err(CurveError::Topology(
                    "boolean selection order differs from certified split order".into(),
                ));
            }
            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                BooleanFragmentAction::KeepSourceDirection
                | BooleanFragmentAction::KeepReversed => {
                    selected.push((
                        key,
                        source,
                        classification.action == BooleanFragmentAction::KeepReversed,
                    ));
                }
            }
        }
        if sources.next().is_some() {
            return Err(CurveError::Topology(
                "boolean selection omits a supplied source fragment".into(),
            ));
        }
        let mut contours = Vec::with_capacity(chain_indices.len());
        for (indices, closed) in chain_indices {
            if !closed {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let mut segments = Vec::with_capacity(indices.len());
            for index in indices {
                let Some(&(key, source, reversed)) = selected.get(index) else {
                    return Err(CurveError::Topology(
                        "compact boolean chain references a missing selected fragment".into(),
                    ));
                };
                let source_segment = source_contour_for_key(first, second, key)?
                    .segments()
                    .get(source.source_segment_index as usize)
                    .ok_or_else(|| {
                        CurveError::Topology(
                            "compact boolean fragment references a missing source segment".into(),
                        )
                    })?;
                let segment = source.materialize(source_segment, key, crossing_windings)?;
                segments.push(if reversed {
                    segment.into_reversed()
                } else {
                    segment
                });
            }
            contours.push(crate::Contour2::from_validated_closed_segments(
                segments, fill_rule,
            ));
        }
        Ok(Classification::Decided(contours))
    }

    pub(crate) fn emit_boundary_fragments_from_owned_compact_split(
        self,
        fragments: CompactLineRegionFragmentSet,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
    ) -> CurveResult<BooleanBoundaryFragmentSet> {
        let directed_fragment_capacity = self.emitted_fragment_count();
        let mut sources = fragments.contours().iter().flat_map(|contour| {
            let key = contour.key;
            contour
                .fragments
                .iter()
                .enumerate()
                .map(move |(index, source)| (key, index, source))
        });
        let mut directed_fragments = Vec::with_capacity(directed_fragment_capacity);
        let mut unresolved_boundaries = Vec::new();

        for classification in self.classifications {
            let Some((key, fragment_index, source)) = sources.next() else {
                return Err(CurveError::Topology(
                    "boolean selection references a fragment outside certified split output".into(),
                ));
            };
            if classification.key != key || classification.fragment_index != fragment_index {
                return Err(CurveError::Topology(
                    "boolean selection order differs from certified split order".into(),
                ));
            }

            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    unresolved_boundaries.push(classification);
                }
                BooleanFragmentAction::KeepSourceDirection
                | BooleanFragmentAction::KeepReversed => {
                    let source_segment = source_contour_for_key(first, second, key)?
                        .segments()
                        .get(source.source_segment_index as usize)
                        .ok_or_else(|| {
                            CurveError::Topology(
                                "compact boolean fragment references a missing source segment"
                                    .into(),
                            )
                        })?;
                    let reversed = classification.action == BooleanFragmentAction::KeepReversed;
                    let source_segment_index = source.source_segment_index as usize;
                    let segment = source.materialize(source_segment, key, crossing_windings)?;
                    let source_range = source.source_range(key, crossing_windings)?;
                    directed_fragments.push(DirectedBooleanFragment {
                        key,
                        fragment_index,
                        source_segment_index,
                        source_segment_start_point: source_segment.start().clone(),
                        source_segment_end_point: source_segment.end().clone(),
                        source_range,
                        reversed,
                        segment: if reversed {
                            segment.into_reversed()
                        } else {
                            segment
                        },
                    });
                }
            }
        }
        if sources.next().is_some() {
            return Err(CurveError::Topology(
                "boolean selection omits a supplied source fragment".into(),
            ));
        }

        BooleanBoundaryFragmentSet::from_certified_split_fragments(
            directed_fragments,
            unresolved_boundaries,
        )
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
        self.emit_boundary_fragments_impl(fragments, false)?
            .ok_or_else(|| {
                CurveError::Topology(
                    "boolean boundary fragment emission did not materialize".into(),
                )
            })
    }

    pub(crate) fn emit_boundary_fragments_from_owned_certified_split(
        self,
        fragments: RegionFragmentSet,
    ) -> CurveResult<BooleanBoundaryFragmentSet> {
        let directed_fragment_capacity = self.emitted_fragment_count();
        let mut sources = fragments.into_contours().into_iter().flat_map(|contour| {
            let key = contour.key;
            contour
                .fragments
                .into_fragments()
                .into_iter()
                .enumerate()
                .map(move |(index, source)| (key, index, source))
        });
        let mut directed_fragments = Vec::with_capacity(directed_fragment_capacity);
        let mut unresolved_boundaries = Vec::new();

        for classification in self.classifications {
            let Some((key, fragment_index, source)) = sources.next() else {
                return Err(CurveError::Topology(
                    "boolean selection references a fragment outside certified split output".into(),
                ));
            };
            if classification.key != key || classification.fragment_index != fragment_index {
                return Err(CurveError::Topology(
                    "boolean selection order differs from certified split order".into(),
                ));
            }

            match classification.action {
                BooleanFragmentAction::Discard => {}
                BooleanFragmentAction::BoundaryNeedsResolution => {
                    unresolved_boundaries.push(classification);
                }
                BooleanFragmentAction::KeepSourceDirection
                | BooleanFragmentAction::KeepReversed => {
                    let reversed = classification.action == BooleanFragmentAction::KeepReversed;
                    directed_fragments.push(DirectedBooleanFragment {
                        key,
                        fragment_index,
                        source_segment_index: source.source_segment_index,
                        source_segment_start_point: source.source_segment_start_point,
                        source_segment_end_point: source.source_segment_end_point,
                        source_range: source.source_range,
                        reversed,
                        segment: if reversed {
                            source.segment.into_reversed()
                        } else {
                            source.segment
                        },
                    });
                }
            }
        }
        if sources.next().is_some() {
            return Err(CurveError::Topology(
                "boolean selection omits a supplied source fragment".into(),
            ));
        }

        BooleanBoundaryFragmentSet::from_certified_split_fragments(
            directed_fragments,
            unresolved_boundaries,
        )
    }

    fn emit_boundary_fragments_impl(
        &self,
        fragments: &RegionFragmentSet,
        fragments_are_certified_split_output: bool,
    ) -> CurveResult<Option<BooleanBoundaryFragmentSet>> {
        validate_boolean_selection_matches_fragments(&self.classifications, fragments)?;

        let directed_fragment_capacity = self.emitted_fragment_count();
        let mut directed_fragments = Vec::with_capacity(directed_fragment_capacity);
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

        Ok(if fragments_are_certified_split_output {
            BooleanBoundaryFragmentSet::from_certified_split_fragments(
                directed_fragments,
                unresolved_boundaries,
            )
        } else {
            BooleanBoundaryFragmentSet::new(directed_fragments, unresolved_boundaries)
        }
        .ok())
    }
}

impl CompactLineRegionFragmentSet {
    pub(crate) fn classify_for_boolean_with_line_crossing_winding<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurveContext,
        endpoint_contacts: &crate::region_events::RegionPointEndpointContactIndex,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
        mut classify_opposite_winding: F,
    ) -> CurveResult<Option<Classification<BooleanFragmentSelection>>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<i32>,
    {
        let source_fragment_count = self.fragment_count();
        let mut classifications = Vec::with_capacity(source_fragment_count);
        let interior_sample_fractions = [
            (Real::one() / Real::from(2_i8))?,
            (Real::one() / Real::from(3_i8))?,
            (Real::from(2_i8) / Real::from(3_i8))?,
        ];

        for contour_fragments in self.contours() {
            let source_contour = source_contour_for_key(first, second, contour_fragments.key)?;
            let source_filled_side_is_left = match source_contour_filled_side_is_left(
                first,
                second,
                contour_fragments.key,
                policy,
            )? {
                Classification::Decided(filled_side) => filled_side,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let opposite_fill_rule = match contour_fragments.key.side {
                RegionSide::First => second.material_contours()[0].fill_rule(),
                RegionSide::Second => first.material_contours()[0].fill_rule(),
            };
            let Some(first_fragment) = contour_fragments.fragments.first() else {
                return Ok(None);
            };
            let full_start = Real::zero();
            let full_end = Real::one();
            let first_source_segment =
                compact_source_segment(first, second, contour_fragments.key, first_fragment)?;
            let (first_start, first_end) = first_fragment.endpoints(
                first_source_segment,
                contour_fragments.key,
                crossing_windings,
            )?;
            let certified_endpoint = if endpoint_contacts.is_empty() {
                Some(CertifiedFragmentEndpoint::Start)
            } else {
                let Some((first_param_start, first_param_end)) = first_fragment.source_parameters(
                    &full_start,
                    &full_end,
                    contour_fragments.key,
                    crossing_windings,
                ) else {
                    return Ok(None);
                };
                certified_fragment_endpoint(
                    endpoint_contacts,
                    contour_fragments.key,
                    source_contour,
                    first_fragment.source_segment_index as usize,
                    first_param_start,
                    first_param_end,
                    policy,
                )
            };
            let source_side = contour_fragments.key.side;
            let mut opposite_winding = match classify_fragment_interior_with(
                first_start,
                first_end,
                certified_endpoint,
                &interior_sample_fractions,
                |fraction| {
                    Ok(Classification::Decided(
                        first_start.lerp(first_end, fraction.clone()),
                    ))
                },
                |sample| classify_opposite_winding(source_side, sample),
            )? {
                FragmentInteriorClassification::Decided(winding) => winding,
                FragmentInteriorClassification::Blocked(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };

            let mut represented_crossings = 0_usize;
            let mut segment_transition_index = 0_usize;
            for (fragment_index, fragment) in contour_fragments.fragments.iter().enumerate() {
                if fragment_index != 0 {
                    let previous = &contour_fragments.fragments[fragment_index - 1];
                    let Some(delta) = crossing_windings.delta_for_next_fragment(
                        contour_fragments.key,
                        previous.source_segment_index as usize,
                        fragment.source_segment_index as usize,
                        &mut segment_transition_index,
                    ) else {
                        return Ok(None);
                    };
                    represented_crossings +=
                        usize::from(previous.source_segment_index == fragment.source_segment_index);
                    opposite_winding = opposite_winding.checked_add(delta).ok_or_else(|| {
                        CurveError::Topology("boolean contour winding exceeds i32 range".into())
                    })?;
                }
                let opposite_location =
                    contour_location_from_winding(opposite_winding, opposite_fill_rule);
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
            if represented_crossings != crossing_windings.crossing_count(contour_fragments.key) {
                return Ok(None);
            }
        }

        Ok(Some(Classification::Decided(
            BooleanFragmentSelection::from_complete_fragment_traversal(
                classifications,
                source_fragment_count,
            ),
        )))
    }
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

fn source_contour_for_key<'a>(
    first: &'a RegionView2<'_>,
    second: &'a RegionView2<'_>,
    key: RegionContourKey,
) -> CurveResult<&'a crate::Contour2> {
    let view = match key.side {
        RegionSide::First => first,
        RegionSide::Second => second,
    };
    let contours = match key.role {
        RegionContourRole::Material => view.material_contours(),
        RegionContourRole::Hole => view.hole_contours(),
    };
    contours.get(key.index).copied().ok_or_else(|| {
        CurveError::Topology("boolean classification references a missing contour".into())
    })
}

pub(crate) fn source_contour_filled_side_is_left(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    key: RegionContourKey,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let contour = source_contour_for_key(first, second, key)?;
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<BooleanFragmentSelection>> {
        self.classify_for_boolean_with_point_classifier(
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

    pub(crate) fn classify_for_boolean_with_line_crossing_winding<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurveContext,
        endpoint_contacts: &crate::region_events::RegionPointEndpointContactIndex,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
        classify_opposite_winding: F,
    ) -> CurveResult<Option<Classification<BooleanFragmentSelection>>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<i32>,
    {
        self.classify_for_boolean_with_line_crossing_winding_impl(
            first,
            second,
            op,
            policy,
            endpoint_contacts,
            crossing_windings,
            classify_opposite_winding,
        )
    }

    pub(crate) fn classify_for_boolean_with_contacts_and_point_classifier<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurveContext,
        endpoint_contacts: Option<&crate::region_events::RegionPointEndpointContactIndex>,
        classify_opposite: F,
    ) -> CurveResult<Classification<BooleanFragmentSelection>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<RegionPointLocation>,
    {
        self.classify_for_boolean_with_contacts_and_point_classifier_impl(
            first,
            second,
            op,
            policy,
            endpoint_contacts,
            classify_opposite,
        )
    }

    fn classify_for_boolean_with_point_classifier<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurveContext,
        mut classify_opposite: F,
    ) -> CurveResult<Classification<BooleanFragmentSelection>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<RegionPointLocation>,
    {
        self.classify_for_boolean_with_contacts_and_point_classifier_impl(
            first,
            second,
            op,
            policy,
            None,
            &mut classify_opposite,
        )
    }

    fn classify_for_boolean_with_line_crossing_winding_impl<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurveContext,
        endpoint_contacts: &crate::region_events::RegionPointEndpointContactIndex,
        crossing_windings: &RegionLineCrossingWindingIndex<'_>,
        mut classify_opposite_winding: F,
    ) -> CurveResult<Option<Classification<BooleanFragmentSelection>>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<i32>,
    {
        if first.material_contours().len() != 1
            || second.material_contours().len() != 1
            || !first.hole_contours().is_empty()
            || !second.hole_contours().is_empty()
        {
            return Ok(None);
        }

        let source_fragment_count = region_fragment_count(self);
        let mut classifications = Vec::with_capacity(source_fragment_count);
        let interior_sample_fractions = [
            (Real::one() / Real::from(2_i8))?,
            (Real::one() / Real::from(3_i8))?,
            (Real::from(2_i8) / Real::from(3_i8))?,
        ];

        for contour_fragments in self.contours() {
            let source_contour = source_contour_for_key(first, second, contour_fragments.key)?;
            let source_filled_side_is_left = match source_contour_filled_side_is_left(
                first,
                second,
                contour_fragments.key,
                policy,
            )? {
                Classification::Decided(filled_side) => filled_side,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let opposite_fill_rule = match contour_fragments.key.side {
                RegionSide::First => second.material_contours()[0].fill_rule(),
                RegionSide::Second => first.material_contours()[0].fill_rule(),
            };
            let fragments = contour_fragments.fragments.fragments();
            let first_fragment = &fragments[0];
            let certified_endpoint = certified_fragment_endpoint(
                endpoint_contacts,
                contour_fragments.key,
                source_contour,
                first_fragment.source_segment_index,
                first_fragment.source_range.start(),
                first_fragment.source_range.end(),
                policy,
            );
            let source_side = contour_fragments.key.side;
            let mut opposite_winding = match classify_fragment_interior(
                &first_fragment.segment,
                certified_endpoint,
                &interior_sample_fractions,
                policy,
                |sample| classify_opposite_winding(source_side, sample),
            )? {
                FragmentInteriorClassification::Decided(winding) => winding,
                FragmentInteriorClassification::Blocked(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };

            let mut represented_crossings = 0_usize;
            let mut segment_transition_index = 0_usize;
            for (fragment_index, fragment) in fragments.iter().enumerate() {
                if fragment_index != 0 {
                    let Some(delta) = crossing_windings.delta_for_next_fragment(
                        contour_fragments.key,
                        fragments[fragment_index - 1].source_segment_index,
                        fragment.source_segment_index,
                        &mut segment_transition_index,
                    ) else {
                        return Ok(None);
                    };
                    represented_crossings += usize::from(
                        fragments[fragment_index - 1].source_segment_index
                            == fragment.source_segment_index,
                    );
                    opposite_winding = opposite_winding.checked_add(delta).ok_or_else(|| {
                        CurveError::Topology("boolean contour winding exceeds i32 range".into())
                    })?;
                }
                let opposite_location =
                    contour_location_from_winding(opposite_winding, opposite_fill_rule);
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
            if represented_crossings != crossing_windings.crossing_count(contour_fragments.key) {
                return Ok(None);
            }
        }

        let selection = BooleanFragmentSelection::from_complete_fragment_traversal(
            classifications,
            source_fragment_count,
        );
        Ok(Some(Classification::Decided(selection)))
    }

    fn classify_for_boolean_with_contacts_and_point_classifier_impl<F>(
        &self,
        first: &RegionView2<'_>,
        second: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurveContext,
        endpoint_contacts: Option<&crate::region_events::RegionPointEndpointContactIndex>,
        mut classify_opposite: F,
    ) -> CurveResult<Classification<BooleanFragmentSelection>>
    where
        F: FnMut(RegionSide, &crate::Point2) -> Classification<RegionPointLocation>,
    {
        let source_fragment_count = region_fragment_count(self);
        let mut classifications = Vec::with_capacity(source_fragment_count);
        let interior_sample_fractions = [
            (Real::one() / Real::from(2_i8))?,
            (Real::one() / Real::from(3_i8))?,
            (Real::from(2_i8) / Real::from(3_i8))?,
        ];

        for contour_fragments in self.contours() {
            let source_contour = source_contour_for_key(first, second, contour_fragments.key)?;
            let source_filled_side_is_left = match source_contour_filled_side_is_left(
                first,
                second,
                contour_fragments.key,
                policy,
            )? {
                Classification::Decided(filled_side) => filled_side,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for (fragment_index, fragment) in
                contour_fragments.fragments.fragments().iter().enumerate()
            {
                let source_side = contour_fragments.key.side;
                let certified_endpoint = endpoint_contacts.and_then(|contacts| {
                    certified_fragment_endpoint(
                        contacts,
                        contour_fragments.key,
                        source_contour,
                        fragment.source_segment_index,
                        fragment.source_range.start(),
                        fragment.source_range.end(),
                        policy,
                    )
                });
                let opposite_location = match classify_fragment_interior(
                    &fragment.segment,
                    certified_endpoint,
                    &interior_sample_fractions,
                    policy,
                    |sample| classify_opposite(source_side, sample),
                )? {
                    FragmentInteriorClassification::Decided(location) => location,
                    FragmentInteriorClassification::Blocked(reason) => {
                        return Ok(Classification::Uncertain(reason));
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

        let selection = BooleanFragmentSelection::from_complete_fragment_traversal(
            classifications,
            source_fragment_count,
        );
        Ok(Classification::Decided(selection))
    }
}

fn certified_fragment_endpoint(
    contacts: &crate::region_events::RegionPointEndpointContactIndex,
    key: RegionContourKey,
    source_contour: &crate::Contour2,
    source_segment_index: usize,
    source_range_start: &Real,
    source_range_end: &Real,
    policy: &CurveContext,
) -> Option<CertifiedFragmentEndpoint> {
    if !contacts.parameter_is_contact(
        key,
        source_segment_index,
        source_contour.len(),
        source_range_start,
        policy,
    ) {
        Some(CertifiedFragmentEndpoint::Start)
    } else if !contacts.parameter_is_contact(
        key,
        source_segment_index,
        source_contour.len(),
        source_range_end,
        policy,
    ) {
        Some(CertifiedFragmentEndpoint::End)
    } else {
        None
    }
}

fn compact_source_segment<'a>(
    first: &'a RegionView2<'_>,
    second: &'a RegionView2<'_>,
    key: RegionContourKey,
    fragment: &crate::fragment::CompactLineContourFragment,
) -> CurveResult<&'a Segment2> {
    source_contour_for_key(first, second, key)?
        .segments()
        .get(fragment.source_segment_index as usize)
        .ok_or_else(|| {
            CurveError::Topology(
                "compact boolean fragment references a missing source segment".into(),
            )
        })
}

fn contour_location_from_winding(winding: i32, fill_rule: FillRule) -> RegionPointLocation {
    let inside = match fill_rule {
        FillRule::NonZero => winding != 0,
        FillRule::EvenOdd => winding.rem_euclid(2) != 0,
    };
    if inside {
        RegionPointLocation::Inside
    } else {
        RegionPointLocation::Outside
    }
}

fn classify_fragment_interior<T, F>(
    segment: &Segment2,
    certified_endpoint: Option<CertifiedFragmentEndpoint>,
    fractions: &[Real; 3],
    policy: &CurveContext,
    classify: F,
) -> CurveResult<FragmentInteriorClassification<T>>
where
    F: FnMut(&Point2) -> Classification<T>,
{
    classify_fragment_interior_with(
        segment.start(),
        segment.end(),
        certified_endpoint,
        fractions,
        |fraction| segment.point_at(fraction, policy),
        classify,
    )
}

fn classify_fragment_interior_with<T, F, S>(
    start: &Point2,
    end: &Point2,
    certified_endpoint: Option<CertifiedFragmentEndpoint>,
    fractions: &[Real; 3],
    mut sample_at: S,
    mut classify: F,
) -> CurveResult<FragmentInteriorClassification<T>>
where
    F: FnMut(&Point2) -> Classification<T>,
    S: FnMut(&Real) -> CurveResult<Classification<Point2>>,
{
    let mut representative_blocker = None;
    let mut classification_blocker = None;

    if let Some(endpoint) = certified_endpoint {
        let sample = match endpoint {
            CertifiedFragmentEndpoint::Start => start,
            CertifiedFragmentEndpoint::End => end,
        };
        match classify(sample) {
            Classification::Decided(location) => {
                return Ok(FragmentInteriorClassification::Decided(location));
            }
            Classification::Uncertain(reason) => {
                classification_blocker.get_or_insert(reason);
            }
        }
    }

    for fraction in fractions {
        let sample = match sample_at(fraction)? {
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
        return Ok(FragmentInteriorClassification::Blocked(reason));
    }
    Ok(FragmentInteriorClassification::Blocked(
        representative_blocker.unwrap_or(UncertaintyReason::Unsupported),
    ))
}

fn region_fragment_count(fragments: &RegionFragmentSet) -> usize {
    fragments
        .contours()
        .iter()
        .map(|contour_fragments| contour_fragments.fragments.len())
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prepared::RegionQuery2;
    use crate::{Contour2, LineArcRegion2, LineSeg2};

    fn point(x: i8, y: i8) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn rectangle(min_x: i8, min_y: i8, max_x: i8, max_y: i8) -> LineArcRegion2 {
        let points = [
            point(min_x, min_y),
            point(max_x, min_y),
            point(max_x, max_y),
            point(min_x, max_y),
        ];
        let segments = (0..4)
            .map(|index| {
                Segment2::Line(
                    LineSeg2::try_new(points[index].clone(), points[(index + 1) % 4].clone())
                        .unwrap(),
                )
            })
            .collect();
        LineArcRegion2::from_material_contours(vec![Contour2::try_new(segments).unwrap()])
    }

    #[test]
    fn proper_line_crossing_winding_matches_fragment_classification() {
        let first = rectangle(0, 0, 4, 3);
        let second = rectangle(2, -1, 6, 2);
        let first_view = first.as_view();
        let second_view = second.as_view();
        let policy = CurveContext::STRICT;
        let events = first_view.intersect_region(&second_view, &policy).unwrap();
        let fragments = match events
            .split_regions(&first_view, &second_view, &policy)
            .unwrap()
        {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                panic!("expected exact split, got {reason:?}")
            }
        };
        let contacts = crate::region_events::RegionPointEndpointContactIndex::from_intersections(
            &events, &policy,
        );
        let crossings = RegionLineCrossingWindingIndex::from_intersections(
            &first_view,
            &second_view,
            &events,
            &policy,
        )
        .unwrap();
        let first_prepared = RegionQuery2::from_region_view(&first_view, &policy);
        let second_prepared = RegionQuery2::from_region_view(&second_view, &policy);

        let expected = fragments
            .classify_for_boolean_with_contacts_and_point_classifier(
                &first_view,
                &second_view,
                BooleanOp::Union,
                &policy,
                Some(&contacts),
                |source_side, sample| match source_side {
                    RegionSide::First => {
                        second_prepared.classify_point_assuming_off_boundary(sample, &policy)
                    }
                    RegionSide::Second => {
                        first_prepared.classify_point_assuming_off_boundary(sample, &policy)
                    }
                },
            )
            .unwrap();
        let mut winding_queries = 0_usize;
        let actual = fragments
            .classify_for_boolean_with_line_crossing_winding(
                &first_view,
                &second_view,
                BooleanOp::Union,
                &policy,
                &contacts,
                &crossings,
                |source_side, sample| {
                    winding_queries += 1;
                    match source_side {
                        RegionSide::First => {
                            crate::contour::line_contour_winding_assuming_off_boundary(
                                second_view.material_contours()[0],
                                sample,
                                &policy,
                            )
                        }
                        RegionSide::Second => {
                            crate::contour::line_contour_winding_assuming_off_boundary(
                                first_view.material_contours()[0],
                                sample,
                                &policy,
                            )
                        }
                    }
                },
            )
            .unwrap()
            .unwrap();

        assert_eq!(actual, expected);
        assert_eq!(winding_queries, 2);
    }
}
