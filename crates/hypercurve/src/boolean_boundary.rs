//! Directed boolean boundary traversal and loop reconstruction.
//!
//! This module owns the graph-facing part of boolean construction: selected
//! fragments are already classified, oriented, and ready to be connected into
//! chains. It deliberately stops before material/hole role assignment.

use crate::boolean::{
    BooleanFragmentClassification, validate_boolean_fragment_classification_boundary_action,
};
use hyperreal::RealSign;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use crate::classify::{compare_reals, is_zero, real_sign};
use crate::{
    Classification, Contour2, CurveError, CurvePolicy, CurveResult, FillRule, ParamRange, Point2,
    Real, RegionContourKey, RegionContourRole, RegionSide, Segment2, UncertaintyReason,
};

/// A selected fragment with geometry already oriented for result traversal.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectedBooleanFragment {
    /// Source keyed contour.
    pub key: crate::RegionContourKey,
    /// Index within [`crate::RegionContourFragments::fragments`].
    pub fragment_index: usize,
    /// Source segment index in the original contour.
    pub source_segment_index: usize,
    /// Exact start point of the original source segment.
    pub source_segment_start_point: Point2,
    /// Exact end point of the original source segment.
    pub source_segment_end_point: Point2,
    /// Retained parameter interval on the source segment.
    pub source_range: ParamRange,
    /// True when `segment` is emitted opposite the source fragment traversal direction.
    pub reversed: bool,
    /// Segment geometry in result traversal direction.
    pub segment: Segment2,
}

/// Boundary fragments selected by a boolean operation.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BooleanBoundaryFragmentSet {
    directed_fragments: Vec<DirectedBooleanFragment>,
    unresolved_boundaries: Vec<BooleanFragmentClassification>,
}

impl BooleanBoundaryFragmentSet {
    /// Constructs a boundary-fragment set from preclassified pieces.
    pub fn new(
        directed_fragments: Vec<DirectedBooleanFragment>,
        unresolved_boundaries: Vec<BooleanFragmentClassification>,
    ) -> CurveResult<Self> {
        validate_boolean_boundary_fragment_set(&directed_fragments, &unresolved_boundaries)?;
        Ok(Self {
            directed_fragments,
            unresolved_boundaries,
        })
    }

    pub(crate) fn from_certified_split_fragments(
        directed_fragments: Vec<DirectedBooleanFragment>,
        unresolved_boundaries: Vec<BooleanFragmentClassification>,
    ) -> CurveResult<Self> {
        // The internal Boolean pipeline receives these segments directly from
        // ContourFragmentSet::from_split_markers. Its ordered, distinct marker
        // intervals already certify nonzero fragment geometry, and reversal
        // preserves that property. Keep the inventory checks at this carrier
        // boundary without rediscovering endpoint inequality in exact arithmetic.
        validate_boolean_boundary_fragment_inventory(&directed_fragments, &unresolved_boundaries)?;
        Ok(Self {
            directed_fragments,
            unresolved_boundaries,
        })
    }

    /// Returns fragments that can be passed to graph traversal immediately.
    pub fn directed_fragments(&self) -> &[DirectedBooleanFragment] {
        &self.directed_fragments
    }

    /// Returns shared-boundary fragments that still need overlap resolution.
    pub fn unresolved_boundaries(&self) -> &[BooleanFragmentClassification] {
        &self.unresolved_boundaries
    }

    /// Returns true when no directed fragments or unresolved fragments exist.
    pub fn is_empty(&self) -> bool {
        self.directed_fragments.is_empty() && self.unresolved_boundaries.is_empty()
    }

    /// Returns true when this set contains no unresolved shared-boundary work.
    pub fn is_ready_for_traversal(&self) -> bool {
        self.unresolved_boundaries.is_empty()
    }

    /// Number of immediately directed fragments.
    pub fn directed_len(&self) -> usize {
        self.directed_fragments.len()
    }

    /// Number of unresolved shared-boundary fragments.
    pub fn unresolved_len(&self) -> usize {
        self.unresolved_boundaries.len()
    }

    /// Assembles directed boundary fragments into endpoint-connected chains.
    ///
    /// This is the first graph-traversal scaffold, not final loop extraction.
    /// Regular vertices use direct endpoint adjacency. At branch vertices, the
    /// traversal selects the smallest certified counter-clockwise turn from the
    /// incoming tangent. Unresolved overlaps and indistinguishable tangent
    /// continuations remain uncertainty rather than using an arbitrary successor.
    pub fn assemble_chains(&self, policy: &CurvePolicy) -> Classification<BooleanBoundaryChainSet> {
        match self.assemble_chains_impl(policy) {
            Ok(chains) => Classification::Decided(chains),
            Err(reason) => Classification::Uncertain(reason),
        }
    }

    pub(crate) fn into_assembled_chains(
        self,
        policy: &CurvePolicy,
    ) -> Classification<BooleanBoundaryChainSet> {
        if !self.unresolved_boundaries.is_empty() {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        let chain_indices = match endpoint_chain_indices(&self.directed_fragments, policy) {
            Ok(Some(chain_indices)) => chain_indices,
            Ok(None) => return tangent_ordered_chains(&self.directed_fragments, policy),
            Err(reason) => return Classification::Uncertain(reason),
        };
        let mut fragments = self
            .directed_fragments
            .into_iter()
            .map(Some)
            .collect::<Vec<_>>();
        Classification::Decided(materialize_chain_indices(chain_indices, |index| {
            fragments[index]
                .take()
                .expect("each assembled fragment index is visited once")
        }))
    }

    pub(crate) fn into_assembled_contours(
        self,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        if !self.unresolved_boundaries.is_empty() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let chain_indices = match endpoint_chain_indices(&self.directed_fragments, policy) {
            Ok(Some(chain_indices)) => chain_indices,
            Ok(None) => {
                let chains = match self.into_assembled_chains(policy) {
                    Classification::Decided(chains) => chains,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let loops = match chains.into_closed_loops() {
                    Classification::Decided(loops) => loops,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return loops.into_contours(fill_rule).map(Classification::Decided);
            }
            Err(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let segments = self
            .directed_fragments
            .into_iter()
            .map(|fragment| fragment.segment)
            .collect::<Vec<_>>();
        Ok(materialize_segment_contours(
            chain_indices,
            segments,
            fill_rule,
        ))
    }

    fn assemble_chains_impl(
        &self,
        policy: &CurvePolicy,
    ) -> Result<BooleanBoundaryChainSet, UncertaintyReason> {
        if !self.unresolved_boundaries.is_empty() {
            return Err(UncertaintyReason::Boundary);
        }

        let Some(chain_indices) = endpoint_chain_indices(&self.directed_fragments, policy)? else {
            return match tangent_ordered_chains(&self.directed_fragments, policy) {
                Classification::Decided(chains) => Ok(chains),
                Classification::Uncertain(reason) => Err(reason),
            };
        };
        Ok(materialize_chain_indices(chain_indices, |index| {
            self.directed_fragments[index].clone()
        }))
    }
}

/// One endpoint-connected directed boundary chain.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryChain {
    fragments: Vec<DirectedBooleanFragment>,
    closed: bool,
}

impl BooleanBoundaryChain {
    /// Constructs a boundary chain from already-ordered fragments.
    pub fn new(fragments: Vec<DirectedBooleanFragment>, closed: bool) -> CurveResult<Self> {
        validate_directed_boolean_fragments(&fragments, "boolean boundary chain")?;
        validate_boolean_boundary_chain_geometry(&fragments, closed)?;
        Ok(Self { fragments, closed })
    }

    fn from_assembled(fragments: Vec<DirectedBooleanFragment>, closed: bool) -> Self {
        Self { fragments, closed }
    }

    /// Returns fragments in traversal order.
    pub fn fragments(&self) -> &[DirectedBooleanFragment] {
        &self.fragments
    }

    /// Consumes the chain and returns fragments in traversal order.
    pub fn into_fragments(self) -> Vec<DirectedBooleanFragment> {
        self.fragments
    }

    /// Returns true when the chain starts and ends at the same point.
    pub const fn is_closed(&self) -> bool {
        self.closed
    }

    /// Returns true when this chain contains no fragments.
    pub fn is_empty(&self) -> bool {
        self.fragments.is_empty()
    }

    /// Returns the number of fragments in this chain.
    pub fn len(&self) -> usize {
        self.fragments.len()
    }
}

/// Endpoint-connected boundary chains.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BooleanBoundaryChainSet {
    chains: Vec<BooleanBoundaryChain>,
}

impl BooleanBoundaryChainSet {
    /// Constructs a chain set from already-assembled chains.
    pub fn new(chains: Vec<BooleanBoundaryChain>) -> CurveResult<Self> {
        validate_boolean_boundary_chains(&chains)?;
        Ok(Self { chains })
    }

    fn from_assembled(chains: Vec<BooleanBoundaryChain>) -> Self {
        Self { chains }
    }

    /// Returns chains in assembly order.
    pub fn chains(&self) -> &[BooleanBoundaryChain] {
        &self.chains
    }

    /// Consumes the set and returns the chains.
    pub fn into_chains(self) -> Vec<BooleanBoundaryChain> {
        self.chains
    }

    /// Returns true when no chains were assembled.
    pub fn is_empty(&self) -> bool {
        self.chains.is_empty()
    }

    /// Returns the number of assembled chains.
    pub fn len(&self) -> usize {
        self.chains.len()
    }

    /// Counts closed chains.
    pub fn closed_count(&self) -> usize {
        self.chains.iter().filter(|chain| chain.is_closed()).count()
    }

    /// Extracts closed chains as boolean boundary loops.
    ///
    /// This is intentionally only loop extraction. It does not decide which
    /// loops are material contours or holes; that nesting/role pass needs
    /// signed containment and overlap-aware traversal. Keeping this conversion
    /// separate avoids assigning hole/material roles before the graph is fully
    /// resolved.
    pub fn closed_loops(&self) -> Classification<BooleanBoundaryLoopSet> {
        if self.chains.iter().any(|chain| !chain.is_closed()) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        Classification::Decided(BooleanBoundaryLoopSet::from_extracted(
            self.chains
                .iter()
                .map(|chain| BooleanBoundaryLoop::from_closed_chain(chain.fragments.clone()))
                .collect(),
        ))
    }

    /// Consumes the chain set and extracts closed chains as boundary loops.
    pub fn into_closed_loops(self) -> Classification<BooleanBoundaryLoopSet> {
        if self.chains.iter().any(|chain| !chain.is_closed()) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        Classification::Decided(BooleanBoundaryLoopSet::from_extracted(
            self.chains
                .into_iter()
                .map(|chain| BooleanBoundaryLoop::from_closed_chain(chain.fragments))
                .collect(),
        ))
    }
}

/// One closed boolean result boundary loop.
///
/// A loop is a stronger result than a chain: all fragments are ordered in
/// traversal direction and the final endpoint reconnects to the first start
/// point. The loop may later become either a material contour or a hole after a
/// nesting pass.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryLoop {
    fragments: Vec<DirectedBooleanFragment>,
}

impl BooleanBoundaryLoop {
    /// Constructs a loop from already-ordered directed fragments.
    pub fn new(fragments: Vec<DirectedBooleanFragment>) -> CurveResult<Self> {
        validate_directed_boolean_fragments(&fragments, "boolean boundary loop")?;
        validate_boolean_boundary_loop_geometry(&fragments)?;
        Ok(Self { fragments })
    }

    fn from_closed_chain(fragments: Vec<DirectedBooleanFragment>) -> Self {
        Self { fragments }
    }

    /// Returns directed fragments in traversal order.
    pub fn fragments(&self) -> &[DirectedBooleanFragment] {
        &self.fragments
    }

    /// Consumes the loop and returns its directed fragments.
    pub fn into_fragments(self) -> Vec<DirectedBooleanFragment> {
        self.fragments
    }

    /// Returns true when this loop contains no fragments.
    pub fn is_empty(&self) -> bool {
        self.fragments.is_empty()
    }

    /// Returns the number of directed fragments in the loop.
    pub fn len(&self) -> usize {
        self.fragments.len()
    }

    /// Clones the already-validated loop geometry into a closed contour.
    pub fn to_contour(&self, fill_rule: FillRule) -> CurveResult<Contour2> {
        Ok(Contour2::from_validated_closed_segments(
            self.fragments
                .iter()
                .map(|fragment| fragment.segment.clone())
                .collect(),
            fill_rule,
        ))
    }

    /// Consumes the already-validated loop geometry into a closed contour.
    pub fn into_contour(self, fill_rule: FillRule) -> CurveResult<Contour2> {
        Ok(Contour2::from_validated_closed_segments(
            self.fragments
                .into_iter()
                .map(|fragment| fragment.segment)
                .collect(),
            fill_rule,
        ))
    }
}

/// Closed boolean boundary loops before material/hole role assignment.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BooleanBoundaryLoopSet {
    loops: Vec<BooleanBoundaryLoop>,
}

impl BooleanBoundaryLoopSet {
    /// Constructs a loop set from already-extracted loops.
    pub fn new(loops: Vec<BooleanBoundaryLoop>) -> CurveResult<Self> {
        validate_boolean_boundary_loops(&loops)?;
        Ok(Self { loops })
    }

    fn from_extracted(loops: Vec<BooleanBoundaryLoop>) -> Self {
        Self { loops }
    }

    /// Builds a loop set from already-decided closed contours.
    ///
    /// When higher-level boolean stages have already regularized a degenerate
    /// boundary-contact case to a set of closed contours (for example, when two
    /// boundaries share an edge that is a known full-seam overlap), the
    /// remaining work is structural transfer, not graph reconstruction. This
    /// conversion keeps the topological decision external to contour construction,
    /// matching the graph-extraction model used by polygon clipping while
    /// preserving the contour-only assumptions in `Contour2` as boundary facts
    /// rather than topology claims.
    pub fn from_contours(contours: Vec<Contour2>) -> CurveResult<Self> {
        let mut loops = Vec::with_capacity(contours.len());
        for (index, contour) in contours.into_iter().enumerate() {
            let fragments = contour
                .segments()
                .iter()
                .enumerate()
                .map(|(fragment_index, segment)| DirectedBooleanFragment {
                    key: RegionContourKey::new(
                        RegionSide::First,
                        RegionContourRole::Material,
                        index,
                    ),
                    fragment_index,
                    source_segment_index: fragment_index,
                    source_segment_start_point: segment.start().clone(),
                    source_segment_end_point: segment.end().clone(),
                    source_range: ParamRange::new(0.into(), 1.into()),
                    reversed: false,
                    segment: segment.clone(),
                })
                .collect();
            loops.push(BooleanBoundaryLoop::new(fragments)?);
        }
        Self::new(loops)
    }

    /// Builds a loop set from borrowed already-decided closed contours.
    ///
    /// This clones the exact contour carriers at the API boundary and then uses
    /// the same structural transfer as the owned contour path.
    pub fn from_contours_borrowed(contours: &[Contour2]) -> CurveResult<Self> {
        Self::from_contours(contours.to_vec())
    }

    /// Converts a decided contour set into a checked loop set while preserving
    /// upstream uncertainty.
    pub fn from_contour_classification(
        contours: Classification<Vec<Contour2>>,
    ) -> CurveResult<Classification<Self>> {
        match contours {
            Classification::Decided(contours) => {
                Self::from_contours(contours).map(Classification::Decided)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns loops in extraction order.
    pub fn loops(&self) -> &[BooleanBoundaryLoop] {
        &self.loops
    }

    /// Consumes the set and returns loops in extraction order.
    pub fn into_loops(self) -> Vec<BooleanBoundaryLoop> {
        self.loops
    }

    /// Returns true when no loops were extracted.
    pub fn is_empty(&self) -> bool {
        self.loops.is_empty()
    }

    /// Returns the number of closed boundary loops.
    pub fn len(&self) -> usize {
        self.loops.len()
    }

    /// Clones every validated loop into a closed contour.
    pub fn to_contours(&self, fill_rule: FillRule) -> CurveResult<Vec<Contour2>> {
        self.loops
            .iter()
            .map(|boundary_loop| boundary_loop.to_contour(fill_rule))
            .collect()
    }

    /// Consumes every validated loop into a closed contour.
    pub fn into_contours(self, fill_rule: FillRule) -> CurveResult<Vec<Contour2>> {
        self.loops
            .into_iter()
            .map(|boundary_loop| boundary_loop.into_contour(fill_rule))
            .collect()
    }
}

pub(crate) type BooleanBoundaryChainIndices = Vec<(Vec<usize>, bool)>;

type EndpointAdjacency = (Vec<Option<usize>>, Vec<Option<usize>>);

// Boundary endpoint keys are process-local shared-allocation identities, not
// attacker-controlled geometry. Scramble their aligned addresses without the
// general-purpose keyed-hash overhead on this short-lived adjacency map.
#[derive(Default)]
struct EndpointIdentityHasher(u64);

impl Hasher for EndpointIdentityHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        let mut value = 0xcbf2_9ce4_8422_2325_u64;
        for byte in bytes {
            value ^= u64::from(*byte);
            value = value.wrapping_mul(0x0000_0100_0000_01b3);
        }
        self.0 = value;
    }

    fn write_u64(&mut self, value: u64) {
        let mut mixed = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        self.0 = mixed ^ (mixed >> 31);
    }
}

pub(crate) trait BooleanBoundaryEdge {
    fn boundary_start(&self) -> &Point2;
    fn boundary_end(&self) -> &Point2;
}

impl BooleanBoundaryEdge for DirectedBooleanFragment {
    fn boundary_start(&self) -> &Point2 {
        self.segment.start()
    }

    fn boundary_end(&self) -> &Point2 {
        self.segment.end()
    }
}

pub(crate) struct BorrowedBooleanBoundaryEdge<'a> {
    start: &'a Point2,
    end: &'a Point2,
}

impl<'a> BorrowedBooleanBoundaryEdge<'a> {
    pub(crate) fn new(segment: &'a Segment2, reversed: bool) -> Self {
        if reversed {
            Self {
                start: segment.end(),
                end: segment.start(),
            }
        } else {
            Self {
                start: segment.start(),
                end: segment.end(),
            }
        }
    }

    pub(crate) fn from_endpoints(start: &'a Point2, end: &'a Point2, reversed: bool) -> Self {
        if reversed {
            Self {
                start: end,
                end: start,
            }
        } else {
            Self { start, end }
        }
    }
}

impl BooleanBoundaryEdge for BorrowedBooleanBoundaryEdge<'_> {
    fn boundary_start(&self) -> &Point2 {
        self.start
    }

    fn boundary_end(&self) -> &Point2 {
        self.end
    }
}

fn directed_boolean_fragment_owner(
    fragment: &DirectedBooleanFragment,
) -> (RegionContourKey, usize) {
    (fragment.key, fragment.fragment_index)
}

fn validate_directed_boolean_fragments(
    fragments: &[DirectedBooleanFragment],
    owner: &str,
) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(format!(
            "{owner} must carry at least one directed fragment"
        )));
    }

    let mut fragment_owners = fragments
        .iter()
        .map(directed_boolean_fragment_owner)
        .collect::<Vec<_>>();
    fragment_owners.sort_unstable();
    if fragment_owners
        .windows(2)
        .any(|window| window[0] == window[1])
    {
        return Err(CurveError::Topology(format!(
            "{owner} directed fragment ownership must be unique"
        )));
    }
    validate_directed_boolean_fragment_geometry(fragments, owner)?;
    Ok(())
}

fn validate_directed_boolean_fragment_geometry(
    fragments: &[DirectedBooleanFragment],
    owner: &str,
) -> CurveResult<()> {
    let policy = CurvePolicy::STRICT;
    for fragment in fragments {
        match is_zero(
            &fragment
                .segment
                .start()
                .distance_squared(fragment.segment.end()),
            &policy,
        ) {
            Some(false) => {}
            Some(true) => {
                return Err(CurveError::Topology(format!(
                    "{owner} directed fragment must carry nonzero geometry"
                )));
            }
            None => {
                return Err(CurveError::Topology(format!(
                    "{owner} directed fragment geometry must be certified nonzero"
                )));
            }
        }
    }
    Ok(())
}

fn validate_boolean_boundary_chain_geometry(
    fragments: &[DirectedBooleanFragment],
    closed: bool,
) -> CurveResult<()> {
    validate_directed_boolean_fragment_connectivity(fragments, "boolean boundary chain")?;
    let (first, last) = directed_fragment_endpoints(fragments, "boolean boundary chain")?;

    let endpoints_close = certified_endpoint_match(last, first, "boolean boundary chain")?;
    if endpoints_close != closed {
        return Err(CurveError::Topology(
            "boolean boundary chain closed flag must match endpoint evidence".to_owned(),
        ));
    }
    Ok(())
}

fn validate_boolean_boundary_loop_geometry(
    fragments: &[DirectedBooleanFragment],
) -> CurveResult<()> {
    validate_directed_boolean_fragment_connectivity(fragments, "boolean boundary loop")?;
    let (first, last) = directed_fragment_endpoints(fragments, "boolean boundary loop")?;

    if !certified_endpoint_match(last, first, "boolean boundary loop")? {
        return Err(CurveError::Topology(
            "boolean boundary loop must close back to its first fragment".to_owned(),
        ));
    }
    Ok(())
}

fn validate_directed_boolean_fragment_connectivity(
    fragments: &[DirectedBooleanFragment],
    owner: &str,
) -> CurveResult<()> {
    for window in fragments.windows(2) {
        if !certified_endpoint_match(&window[0], &window[1], owner)? {
            return Err(CurveError::Topology(format!(
                "{owner} fragments must be endpoint-connected"
            )));
        }
    }
    Ok(())
}

fn directed_fragment_endpoints<'a>(
    fragments: &'a [DirectedBooleanFragment],
    owner: &str,
) -> CurveResult<(&'a DirectedBooleanFragment, &'a DirectedBooleanFragment)> {
    let first = fragments.first().ok_or_else(|| {
        CurveError::Topology(format!("{owner} must carry at least one directed fragment"))
    })?;
    let last = fragments.last().ok_or_else(|| {
        CurveError::Topology(format!("{owner} must carry at least one directed fragment"))
    })?;
    Ok((first, last))
}

fn certified_endpoint_match(
    left: &DirectedBooleanFragment,
    right: &DirectedBooleanFragment,
    owner: &str,
) -> CurveResult<bool> {
    let policy = CurvePolicy::STRICT;
    match points_match(left.segment.end(), right.segment.start(), &policy) {
        Classification::Decided(matches) => Ok(matches),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "{owner} endpoint equality could not be certified: {reason:?}"
        ))),
    }
}

fn validate_boolean_boundary_chains(chains: &[BooleanBoundaryChain]) -> CurveResult<()> {
    let mut fragment_owners = Vec::new();
    for chain in chains {
        validate_directed_boolean_fragments(chain.fragments(), "boolean boundary chain")?;
        fragment_owners.extend(
            chain
                .fragments()
                .iter()
                .map(directed_boolean_fragment_owner),
        );
    }
    validate_unique_boolean_fragment_owners(
        fragment_owners,
        "boolean boundary chain set must not reuse directed fragment ownership",
    )
}

fn validate_boolean_boundary_loops(loops: &[BooleanBoundaryLoop]) -> CurveResult<()> {
    let mut fragment_owners = Vec::new();
    for boundary_loop in loops {
        validate_directed_boolean_fragments(boundary_loop.fragments(), "boolean boundary loop")?;
        fragment_owners.extend(
            boundary_loop
                .fragments()
                .iter()
                .map(directed_boolean_fragment_owner),
        );
    }
    validate_unique_boolean_fragment_owners(
        fragment_owners,
        "boolean boundary loop set must not reuse directed fragment ownership",
    )
}

fn validate_unique_boolean_fragment_owners(
    mut fragment_owners: Vec<(RegionContourKey, usize)>,
    message: &str,
) -> CurveResult<()> {
    fragment_owners.sort_unstable();
    if fragment_owners
        .windows(2)
        .any(|window| window[0] == window[1])
    {
        return Err(CurveError::Topology(message.to_owned()));
    }
    Ok(())
}

fn validate_boolean_boundary_fragment_set(
    directed_fragments: &[DirectedBooleanFragment],
    unresolved_boundaries: &[BooleanFragmentClassification],
) -> CurveResult<()> {
    validate_directed_boolean_fragment_geometry(
        directed_fragments,
        "boolean boundary fragment set",
    )?;
    validate_boolean_boundary_fragment_inventory(directed_fragments, unresolved_boundaries)
}

fn validate_boolean_boundary_fragment_inventory(
    directed_fragments: &[DirectedBooleanFragment],
    unresolved_boundaries: &[BooleanFragmentClassification],
) -> CurveResult<()> {
    for unresolved in unresolved_boundaries {
        validate_boolean_fragment_classification_boundary_action(unresolved)?;
    }

    let mut fragment_owners = directed_fragments
        .iter()
        .map(directed_boolean_fragment_owner)
        .collect::<Vec<_>>();
    fragment_owners.extend(
        unresolved_boundaries
            .iter()
            .map(|classification| (classification.key, classification.fragment_index)),
    );
    validate_unique_boolean_fragment_owners(
        fragment_owners,
        "boolean boundary fragment set must not contain duplicate source fragment ownership",
    )
}

fn decided_boolean_boundary_chain(
    fragments: Vec<DirectedBooleanFragment>,
    closed: bool,
) -> Classification<BooleanBoundaryChain> {
    Classification::Decided(BooleanBoundaryChain::from_assembled(fragments, closed))
}

fn endpoint_adjacency(
    fragments: &[impl BooleanBoundaryEdge],
    policy: &CurvePolicy,
) -> Classification<EndpointAdjacency> {
    let mut successors = vec![None; fragments.len()];
    let mut predecessors = vec![None; fragments.len()];
    let mut starts_by_identity = HashMap::with_capacity_and_hasher(
        fragments.len(),
        BuildHasherDefault::<EndpointIdentityHasher>::default(),
    );

    for (index, fragment) in fragments.iter().enumerate() {
        if starts_by_identity
            .insert(fragment.boundary_start().identity(), index)
            .is_some()
        {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        }
    }

    let mut unmatched_ends = Vec::new();
    for (left_index, left) in fragments.iter().enumerate() {
        let Some(&right_index) = starts_by_identity.get(&left.boundary_end().identity()) else {
            unmatched_ends.push(left_index);
            continue;
        };
        if left_index == right_index {
            unmatched_ends.push(left_index);
            continue;
        }
        successors[left_index] = Some(right_index);
        if predecessors[right_index].replace(left_index).is_some() {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        }
    }

    for left_index in unmatched_ends {
        let left = &fragments[left_index];
        for (right_index, right) in fragments.iter().enumerate() {
            if left_index == right_index {
                continue;
            }

            let matches = match points_match(left.boundary_end(), right.boundary_start(), policy) {
                Classification::Decided(matches) => matches,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            if !matches {
                continue;
            }

            if successors[left_index].replace(right_index).is_some() {
                return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
            }
            if predecessors[right_index].replace(left_index).is_some() {
                return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
            }
        }
    }

    Classification::Decided((successors, predecessors))
}

#[derive(Clone, Debug)]
struct BoundaryTangent {
    dx: Real,
    dy: Real,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundaryTurnOrdering {
    FirstBeforeSecond,
    SecondBeforeFirst,
    SameDirection,
}

fn tangent_ordered_chains(
    fragments: &[DirectedBooleanFragment],
    policy: &CurvePolicy,
) -> Classification<BooleanBoundaryChainSet> {
    let outgoing = match boundary_outgoing_adjacency(fragments, policy) {
        Classification::Decided(outgoing) => outgoing,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let mut predecessors = vec![0_usize; fragments.len()];
    for successors in &outgoing {
        for successor in successors {
            predecessors[*successor] += 1;
        }
    }

    let mut used = vec![false; fragments.len()];
    let mut chains = Vec::new();

    for index in 0..fragments.len() {
        if predecessors[index] == 0 && !used[index] {
            let chain = match follow_tangent_ordered_chain(
                index, fragments, &outgoing, &mut used, policy,
            ) {
                Classification::Decided(chain) => chain,
                Classification::Uncertain(reason) => {
                    return Classification::Uncertain(reason);
                }
            };
            chains.push(chain);
        }
    }

    for index in 0..fragments.len() {
        if !used[index] {
            let chain = match follow_tangent_ordered_chain(
                index, fragments, &outgoing, &mut used, policy,
            ) {
                Classification::Decided(chain) => chain,
                Classification::Uncertain(reason) => {
                    return Classification::Uncertain(reason);
                }
            };
            chains.push(chain);
        }
    }

    Classification::Decided(BooleanBoundaryChainSet::from_assembled(chains))
}

fn boundary_outgoing_adjacency(
    fragments: &[DirectedBooleanFragment],
    policy: &CurvePolicy,
) -> Classification<Vec<Vec<usize>>> {
    let mut outgoing = vec![Vec::new(); fragments.len()];
    for (left_index, left) in fragments.iter().enumerate() {
        for (right_index, right) in fragments.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            match points_match(left.segment.end(), right.segment.start(), policy) {
                Classification::Decided(true) => outgoing[left_index].push(right_index),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(outgoing)
}

fn follow_tangent_ordered_chain(
    start: usize,
    fragments: &[DirectedBooleanFragment],
    outgoing: &[Vec<usize>],
    used: &mut [bool],
    policy: &CurvePolicy,
) -> Classification<BooleanBoundaryChain> {
    let first_start = fragments[start].segment.start().clone();
    let mut current = start;
    let mut chain = Vec::new();

    loop {
        if used[current] {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        used[current] = true;
        chain.push(fragments[current].clone());

        let next =
            match choose_boundary_tangent_successor(current, &outgoing[current], fragments, policy)
            {
                Classification::Decided(next) => next,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
        let Some(next) = next else {
            let closed = match points_match(fragments[current].segment.end(), &first_start, policy)
            {
                Classification::Decided(closed) => closed,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            return decided_boolean_boundary_chain(chain, closed);
        };
        if next == start {
            return decided_boolean_boundary_chain(chain, true);
        }
        if used[next] {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        current = next;
    }
}

fn choose_boundary_tangent_successor(
    current: usize,
    candidates: &[usize],
    fragments: &[DirectedBooleanFragment],
    policy: &CurvePolicy,
) -> Classification<Option<usize>> {
    if candidates.is_empty() {
        return Classification::Decided(None);
    }
    if candidates.len() == 1 {
        return Classification::Decided(Some(candidates[0]));
    }

    let base = segment_end_tangent(&fragments[current].segment);
    if !boundary_tangent_is_nonzero(&base, policy) {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    }

    let mut best = candidates[0];
    let mut best_tangent = segment_start_tangent(&fragments[best].segment);
    if !boundary_tangent_is_nonzero(&best_tangent, policy) {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    }
    for candidate in candidates.iter().copied().skip(1) {
        let candidate_tangent = segment_start_tangent(&fragments[candidate].segment);
        if !boundary_tangent_is_nonzero(&candidate_tangent, policy) {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }
        match compare_boundary_turn_from_base(&base, &candidate_tangent, &best_tangent, policy) {
            Classification::Decided(BoundaryTurnOrdering::FirstBeforeSecond) => {
                best = candidate;
                best_tangent = candidate_tangent;
            }
            Classification::Decided(BoundaryTurnOrdering::SecondBeforeFirst) => {}
            Classification::Decided(BoundaryTurnOrdering::SameDirection) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(Some(best))
}

fn segment_start_tangent(segment: &Segment2) -> BoundaryTangent {
    segment_tangent_at(segment, segment.start())
}

fn segment_end_tangent(segment: &Segment2) -> BoundaryTangent {
    segment_tangent_at(segment, segment.end())
}

fn segment_tangent_at(segment: &Segment2, point: &Point2) -> BoundaryTangent {
    match segment {
        Segment2::Line(line) => BoundaryTangent {
            dx: line.end().x() - line.start().x(),
            dy: line.end().y() - line.start().y(),
        },
        Segment2::Arc(arc) => {
            let radial_x = point.x() - arc.center().x();
            let radial_y = point.y() - arc.center().y();
            if arc.is_clockwise() {
                BoundaryTangent {
                    dx: radial_y,
                    dy: -radial_x,
                }
            } else {
                BoundaryTangent {
                    dx: -radial_y,
                    dy: radial_x,
                }
            }
        }
    }
}

fn boundary_tangent_is_nonzero(tangent: &BoundaryTangent, policy: &CurvePolicy) -> bool {
    !matches!(
        is_zero(&boundary_dot(tangent, tangent), policy),
        Some(true) | None
    )
}

fn compare_boundary_turn_from_base(
    base: &BoundaryTangent,
    first: &BoundaryTangent,
    second: &BoundaryTangent,
    policy: &CurvePolicy,
) -> Classification<BoundaryTurnOrdering> {
    let first_half = match boundary_turn_half(base, first, policy) {
        Some(half) => half,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    let second_half = match boundary_turn_half(base, second, policy) {
        Some(half) => half,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    if first_half != second_half {
        return Classification::Decided(if first_half < second_half {
            BoundaryTurnOrdering::FirstBeforeSecond
        } else {
            BoundaryTurnOrdering::SecondBeforeFirst
        });
    }

    match real_sign(&boundary_cross(first, second), policy) {
        Some(RealSign::Positive) => {
            Classification::Decided(BoundaryTurnOrdering::FirstBeforeSecond)
        }
        Some(RealSign::Negative) => {
            Classification::Decided(BoundaryTurnOrdering::SecondBeforeFirst)
        }
        Some(RealSign::Zero) => Classification::Decided(BoundaryTurnOrdering::SameDirection),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn boundary_turn_half(
    base: &BoundaryTangent,
    candidate: &BoundaryTangent,
    policy: &CurvePolicy,
) -> Option<u8> {
    match real_sign(&boundary_cross(base, candidate), policy)? {
        RealSign::Positive => Some(0),
        RealSign::Negative => Some(1),
        RealSign::Zero => match real_sign(&boundary_dot(base, candidate), policy)? {
            RealSign::Positive => Some(0),
            RealSign::Negative => Some(1),
            RealSign::Zero => None,
        },
    }
}

fn boundary_cross(left: &BoundaryTangent, right: &BoundaryTangent) -> Real {
    (&left.dx * &right.dy) - (&left.dy * &right.dx)
}

fn boundary_dot(left: &BoundaryTangent, right: &BoundaryTangent) -> Real {
    (&left.dx * &right.dx) + (&left.dy * &right.dy)
}

pub(crate) fn endpoint_chain_indices(
    fragments: &[impl BooleanBoundaryEdge],
    policy: &CurvePolicy,
) -> Result<Option<BooleanBoundaryChainIndices>, UncertaintyReason> {
    let (successors, predecessors) = match endpoint_adjacency(fragments, policy) {
        Classification::Decided(adjacency) => adjacency,
        Classification::Uncertain(UncertaintyReason::Unsupported) => return Ok(None),
        Classification::Uncertain(reason) => return Err(reason),
    };

    let mut used = vec![false; fragments.len()];
    let mut chains = Vec::new();
    for index in (0..fragments.len())
        .filter(|index| predecessors[*index].is_none())
        .chain(0..fragments.len())
    {
        if !used[index] {
            chains.push(
                follow_chain_indices(index, &successors, &mut used)
                    .ok_or(UncertaintyReason::Unsupported)?,
            );
        }
    }
    Ok(Some(chains))
}

fn materialize_chain_indices<F>(
    chains: Vec<(Vec<usize>, bool)>,
    mut take_fragment: F,
) -> BooleanBoundaryChainSet
where
    F: FnMut(usize) -> DirectedBooleanFragment,
{
    BooleanBoundaryChainSet::from_assembled(
        chains
            .into_iter()
            .map(|(indices, closed)| {
                BooleanBoundaryChain::from_assembled(
                    indices.into_iter().map(&mut take_fragment).collect(),
                    closed,
                )
            })
            .collect(),
    )
}

pub(crate) fn materialize_segment_contours(
    chains: BooleanBoundaryChainIndices,
    segments: Vec<Segment2>,
    fill_rule: FillRule,
) -> Classification<Vec<Contour2>> {
    let mut segments = segments.into_iter().map(Some).collect::<Vec<_>>();
    let mut contours = Vec::with_capacity(chains.len());
    for (indices, closed) in chains {
        if !closed {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        contours.push(Contour2::from_validated_closed_segments(
            indices
                .into_iter()
                .map(|index| {
                    segments[index]
                        .take()
                        .expect("each assembled segment index is visited once")
                })
                .collect(),
            fill_rule,
        ));
    }
    Classification::Decided(contours)
}

fn follow_chain_indices(
    start: usize,
    successors: &[Option<usize>],
    used: &mut [bool],
) -> Option<(Vec<usize>, bool)> {
    let mut chain = Vec::new();
    let mut current = start;
    let mut closed = false;

    loop {
        if used[current] {
            return None;
        }

        used[current] = true;
        chain.push(current);

        let Some(next) = successors[current] else {
            break;
        };

        if next == start {
            closed = true;
            break;
        }
        if used[next] {
            return None;
        }

        current = next;
    }

    Some((chain, closed))
}

fn points_match(
    left: &crate::Point2,
    right: &crate::Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    if left == right {
        return Classification::Decided(true);
    }
    if matches!(
        compare_reals(left.x(), right.x(), policy),
        Some(Ordering::Less | Ordering::Greater)
    ) || matches!(
        compare_reals(left.y(), right.y(), policy),
        Some(Ordering::Less | Ordering::Greater)
    ) {
        return Classification::Decided(false);
    }
    let distance = left.distance_squared(right);
    match is_zero(&distance, policy) {
        Some(matches) => Classification::Decided(matches),
        None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
    }
}
