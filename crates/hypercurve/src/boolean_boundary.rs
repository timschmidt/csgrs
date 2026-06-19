//! Directed boolean boundary traversal and loop reconstruction.
//!
//! This module owns the graph-facing part of boolean construction: selected
//! fragments are already classified, oriented, and ready to be connected into
//! chains. It deliberately stops before material/hole role assignment.

use crate::boolean::{
    BooleanFragmentClassification, validate_boolean_fragment_classification_boundary_action,
};
use crate::classify::is_zero;
use crate::{
    Classification, Contour2, CurveError, CurvePolicy, CurveResult, FillRule, ParamRange, Point2,
    RegionContourKey, RegionContourRole, RegionSide, RetainedTopologyStatus, Segment2, SegmentKind,
    SegmentKindCounts, UncertaintyReason,
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

/// Report for assembling directed boolean boundary fragments into chains.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryChainAssemblyReport2 {
    stage: BooleanBoundaryChainAssemblyStage2,
    directed_fragment_count: usize,
    directed_fragment_kind_counts: SegmentKindCounts,
    unresolved_boundary_count: usize,
    chain_count: Option<usize>,
    closed_chain_count: Option<usize>,
    open_chain_count: Option<usize>,
    output_fragment_count: Option<usize>,
    output_fragment_kind_counts: Option<SegmentKindCounts>,
    output_fragments: Vec<BooleanBoundaryOutputFragmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Source provenance for one output fragment produced by boolean boundary traversal.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryOutputFragmentReport2 {
    key: RegionContourKey,
    fragment_index: usize,
    source_segment_index: usize,
    source_segment_start_point: Point2,
    source_segment_end_point: Point2,
    source_range: ParamRange,
    reversed: bool,
    output_fragment_index: usize,
    output_fragment_kind: SegmentKind,
}

/// Furthest exact stage reached by boolean boundary chain assembly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryChainAssemblyStage2 {
    /// Shared-boundary fragments were checked before graph traversal.
    BoundaryResolution,
    /// Directed fragment endpoint adjacency was being classified.
    EndpointAdjacency,
    /// Endpoint-connected chains were assembled and validated.
    ChainMaterialization,
}

/// Result of report-bearing boolean boundary chain assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryChainAssemblyResult2 {
    chains: Option<BooleanBoundaryChainSet>,
    report: BooleanBoundaryChainAssemblyReport2,
}

/// Report for extracting closed boolean boundary loops from assembled chains.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryLoopExtractionReport2 {
    stage: BooleanBoundaryLoopExtractionStage2,
    source_chain_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    closed_chain_count: usize,
    open_chain_count: usize,
    loop_count: Option<usize>,
    output_fragment_count: Option<usize>,
    output_fragment_kind_counts: Option<SegmentKindCounts>,
    output_fragments: Vec<BooleanBoundaryOutputFragmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by boolean boundary loop extraction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryLoopExtractionStage2 {
    /// Chains were being checked for closure before loop materialization.
    ChainClosureValidation,
    /// Closed chains were converted into checked boundary loops.
    LoopMaterialization,
}

/// Result of report-bearing boolean boundary loop extraction.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryLoopExtractionResult2 {
    loops: Option<BooleanBoundaryLoopSet>,
    report: BooleanBoundaryLoopExtractionReport2,
}

/// Report for transferring already-decided contours into boolean boundary loops.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryLoopConstructionReport2 {
    stage: BooleanBoundaryLoopConstructionStage2,
    source_contour_count: usize,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    loop_count: Option<usize>,
    output_fragment_count: Option<usize>,
    output_fragment_kind_counts: Option<SegmentKindCounts>,
    output_fragments: Vec<BooleanBoundaryOutputFragmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by contour-to-boolean-loop construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryLoopConstructionStage2 {
    /// Source contour segments were replayed as directed boundary fragments.
    ContourGeometryReplay,
    /// Checked closed boundary loops were materialized.
    LoopMaterialization,
}

/// Result of report-bearing boolean boundary loop construction.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryLoopConstructionResult2 {
    loops: Option<BooleanBoundaryLoopSet>,
    report: BooleanBoundaryLoopConstructionReport2,
}

/// Report for converting closed boolean boundary loops into checked contours.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryContourTransferReport2 {
    fill_rule: FillRule,
    stage: BooleanBoundaryContourTransferStage2,
    source_loop_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    contour_count: Option<usize>,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    output_segments: Vec<BooleanBoundaryOutputFragmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by boolean boundary contour transfer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryContourTransferStage2 {
    /// Loop geometry was being replayed into checked contour inputs.
    LoopGeometryReplay,
    /// Checked closed contours were materialized with the requested fill rule.
    ContourMaterialization,
}

/// Result of report-bearing boolean boundary contour transfer.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryContourTransferResult2 {
    contours: Option<Vec<Contour2>>,
    report: BooleanBoundaryContourTransferReport2,
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
    /// It requires every directed fragment endpoint to have at most one outgoing
    /// and one incoming neighbor. That mirrors the regularized traversal graph
    /// assumed after polygon clipping has inserted and classified intersections
    /// (G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
    /// ACM Transactions on Graphics 17(2), 71-83, 1998). Branch points and
    /// unresolved overlaps are intentionally returned as uncertainty here
    /// because Vatti-style scanline algorithms resolve those cases with fill
    /// state and event ordering, not by choosing an arbitrary local successor
    /// (B. R. Vatti, "A generic solution to polygon clipping," Communications
    /// of the ACM 35(7), 56-63, 1992).
    pub fn assemble_chains(&self, policy: &CurvePolicy) -> Classification<BooleanBoundaryChainSet> {
        let result = self.assemble_chains_with_report(policy);
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(chains) = result.into_chains() {
            Classification::Decided(chains)
        } else {
            Classification::Uncertain(blocker)
        }
    }

    /// Assembles directed boundary fragments and retains traversal evidence.
    pub fn assemble_chains_with_report(
        &self,
        policy: &CurvePolicy,
    ) -> BooleanBoundaryChainAssemblyResult2 {
        if !self.unresolved_boundaries.is_empty() {
            return blocked_boolean_boundary_chain_assembly_result(
                self,
                BooleanBoundaryChainAssemblyStage2::BoundaryResolution,
                UncertaintyReason::Boundary,
            );
        }

        let (successors, predecessors) = match endpoint_adjacency(&self.directed_fragments, policy)
        {
            Classification::Decided(adjacency) => adjacency,
            Classification::Uncertain(reason) => {
                return blocked_boolean_boundary_chain_assembly_result(
                    self,
                    BooleanBoundaryChainAssemblyStage2::EndpointAdjacency,
                    reason,
                );
            }
        };

        let mut used = vec![false; self.directed_fragments.len()];
        let mut chains = Vec::new();

        for index in 0..self.directed_fragments.len() {
            if predecessors[index].is_none() && !used[index] {
                let chain =
                    match follow_chain(index, &self.directed_fragments, &successors, &mut used) {
                        Classification::Decided(chain) => chain,
                        Classification::Uncertain(reason) => {
                            return blocked_boolean_boundary_chain_assembly_result(
                                self,
                                BooleanBoundaryChainAssemblyStage2::ChainMaterialization,
                                reason,
                            );
                        }
                    };
                chains.push(chain);
            }
        }

        for index in 0..self.directed_fragments.len() {
            if !used[index] {
                let chain =
                    match follow_chain(index, &self.directed_fragments, &successors, &mut used) {
                        Classification::Decided(chain) => chain,
                        Classification::Uncertain(reason) => {
                            return blocked_boolean_boundary_chain_assembly_result(
                                self,
                                BooleanBoundaryChainAssemblyStage2::ChainMaterialization,
                                reason,
                            );
                        }
                    };
                chains.push(chain);
            }
        }

        match BooleanBoundaryChainSet::new(chains) {
            Ok(chain_set) => decided_boolean_boundary_chain_assembly_result(self, chain_set),
            Err(_) => blocked_boolean_boundary_chain_assembly_result(
                self,
                BooleanBoundaryChainAssemblyStage2::ChainMaterialization,
                UncertaintyReason::Unsupported,
            ),
        }
    }
}

impl BooleanBoundaryChainAssemblyReport2 {
    /// Returns the furthest exact chain-assembly stage reached.
    pub const fn stage(&self) -> BooleanBoundaryChainAssemblyStage2 {
        self.stage
    }

    /// Returns the number of directed fragments supplied for traversal.
    pub const fn directed_fragment_count(&self) -> usize {
        self.directed_fragment_count
    }

    /// Returns primitive-family counts for directed fragments supplied for traversal.
    pub const fn directed_fragment_kind_counts(&self) -> SegmentKindCounts {
        self.directed_fragment_kind_counts
    }

    /// Returns the number of unresolved shared-boundary fragments.
    pub const fn unresolved_boundary_count(&self) -> usize {
        self.unresolved_boundary_count
    }

    /// Returns assembled chain count when materialized.
    pub const fn chain_count(&self) -> Option<usize> {
        self.chain_count
    }

    /// Returns closed chain count when materialized.
    pub const fn closed_chain_count(&self) -> Option<usize> {
        self.closed_chain_count
    }

    /// Returns open chain count when materialized.
    pub const fn open_chain_count(&self) -> Option<usize> {
        self.open_chain_count
    }

    /// Returns output fragment count when materialized.
    pub const fn output_fragment_count(&self) -> Option<usize> {
        self.output_fragment_count
    }

    /// Returns primitive-family counts for output chain fragments when materialized.
    pub const fn output_fragment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_fragment_kind_counts
    }

    /// Returns per-output-fragment source provenance when chains materialized.
    pub fn output_fragments(&self) -> &[BooleanBoundaryOutputFragmentReport2] {
        &self.output_fragments
    }

    /// Returns retained topology status for chain assembly.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized chain assembly.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl BooleanBoundaryOutputFragmentReport2 {
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

    /// Returns true when the output fragment is opposite source traversal.
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the output fragment index after chain assembly.
    pub const fn output_fragment_index(&self) -> usize {
        self.output_fragment_index
    }

    /// Returns the output fragment primitive kind.
    pub const fn output_fragment_kind(&self) -> SegmentKind {
        self.output_fragment_kind
    }
}

impl BooleanBoundaryChainAssemblyResult2 {
    /// Returns materialized chains, if assembly succeeded.
    pub const fn chains(&self) -> Option<&BooleanBoundaryChainSet> {
        self.chains.as_ref()
    }

    /// Consumes this result and returns materialized chains, if any.
    pub fn into_chains(self) -> Option<BooleanBoundaryChainSet> {
        self.chains
    }

    /// Returns retained chain-assembly evidence.
    pub const fn report(&self) -> &BooleanBoundaryChainAssemblyReport2 {
        &self.report
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
    /// signed containment and overlap-aware traversal. Greiner and Hormann
    /// treat closed result polygons as the product of classified traversal
    /// (G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
    /// ACM Transactions on Graphics 17(2), 71-83, 1998), while Vatti's
    /// scanline algorithm determines output contours from fill-state
    /// transitions (B. R. Vatti, "A generic solution to polygon clipping,"
    /// Communications of the ACM 35(7), 56-63, 1992). Keeping this conversion
    /// separate avoids assigning hole/material roles before the graph is fully
    /// resolved.
    pub fn closed_loops(&self) -> Classification<BooleanBoundaryLoopSet> {
        let result = self.closed_loops_with_report();
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(loops) = result.into_loops() {
            Classification::Decided(loops)
        } else {
            Classification::Uncertain(blocker)
        }
    }

    /// Extracts closed chains as boolean boundary loops and retains evidence.
    pub fn closed_loops_with_report(&self) -> BooleanBoundaryLoopExtractionResult2 {
        if self.chains.iter().any(|chain| !chain.is_closed()) {
            return blocked_boolean_boundary_loop_extraction_result(
                self,
                BooleanBoundaryLoopExtractionStage2::ChainClosureValidation,
                UncertaintyReason::Unsupported,
            );
        }

        let loops = match self
            .chains
            .iter()
            .map(|chain| BooleanBoundaryLoop::new(chain.fragments.clone()))
            .collect::<CurveResult<Vec<_>>>()
        {
            Ok(loops) => loops,
            Err(_) => {
                return blocked_boolean_boundary_loop_extraction_result(
                    self,
                    BooleanBoundaryLoopExtractionStage2::LoopMaterialization,
                    UncertaintyReason::Unsupported,
                );
            }
        };
        match BooleanBoundaryLoopSet::new(loops) {
            Ok(loop_set) => decided_boolean_boundary_loop_extraction_result(self, loop_set),
            Err(_) => blocked_boolean_boundary_loop_extraction_result(
                self,
                BooleanBoundaryLoopExtractionStage2::LoopMaterialization,
                UncertaintyReason::Unsupported,
            ),
        }
    }

    /// Consumes the chain set and extracts closed chains as boundary loops.
    pub fn into_closed_loops(self) -> Classification<BooleanBoundaryLoopSet> {
        let result = self.into_closed_loops_with_report();
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        if let Some(loops) = result.into_loops() {
            Classification::Decided(loops)
        } else {
            Classification::Uncertain(blocker)
        }
    }

    /// Consumes the chain set and extracts closed chains with retained evidence.
    pub fn into_closed_loops_with_report(self) -> BooleanBoundaryLoopExtractionResult2 {
        if self.chains.iter().any(|chain| !chain.is_closed()) {
            return blocked_boolean_boundary_loop_extraction_result(
                &self,
                BooleanBoundaryLoopExtractionStage2::ChainClosureValidation,
                UncertaintyReason::Unsupported,
            );
        }

        let source_chain_count = self.len();
        let source_fragment_count = chain_set_fragment_count(&self);
        let source_fragment_kind_counts = chain_set_fragment_kind_counts(&self);
        let closed_chain_count = self.closed_count();
        let loops = match self
            .chains
            .into_iter()
            .map(|chain| BooleanBoundaryLoop::new(chain.fragments))
            .collect::<CurveResult<Vec<_>>>()
        {
            Ok(loops) => loops,
            Err(_) => {
                return blocked_boolean_boundary_loop_extraction_counts_result(
                    BooleanBoundaryLoopExtractionStage2::LoopMaterialization,
                    source_chain_count,
                    source_fragment_count,
                    source_fragment_kind_counts,
                    closed_chain_count,
                    0,
                    UncertaintyReason::Unsupported,
                );
            }
        };
        match BooleanBoundaryLoopSet::new(loops) {
            Ok(loop_set) => decided_boolean_boundary_loop_extraction_counts_result(
                source_chain_count,
                source_fragment_count,
                source_fragment_kind_counts,
                closed_chain_count,
                0,
                loop_set,
            ),
            Err(_) => blocked_boolean_boundary_loop_extraction_counts_result(
                BooleanBoundaryLoopExtractionStage2::LoopMaterialization,
                source_chain_count,
                source_fragment_count,
                source_fragment_kind_counts,
                closed_chain_count,
                0,
                UncertaintyReason::Unsupported,
            ),
        }
    }
}

impl BooleanBoundaryLoopExtractionReport2 {
    /// Returns the furthest exact loop-extraction stage reached.
    pub const fn stage(&self) -> BooleanBoundaryLoopExtractionStage2 {
        self.stage
    }

    /// Returns source boundary chain count.
    pub const fn source_chain_count(&self) -> usize {
        self.source_chain_count
    }

    /// Returns total source fragment count across chains.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }

    /// Returns primitive-family counts for source chain fragments.
    pub const fn source_fragment_kind_counts(&self) -> SegmentKindCounts {
        self.source_fragment_kind_counts
    }

    /// Returns source closed chain count.
    pub const fn closed_chain_count(&self) -> usize {
        self.closed_chain_count
    }

    /// Returns source open chain count.
    pub const fn open_chain_count(&self) -> usize {
        self.open_chain_count
    }

    /// Returns output loop count when materialized.
    pub const fn loop_count(&self) -> Option<usize> {
        self.loop_count
    }

    /// Returns output fragment count when materialized.
    pub const fn output_fragment_count(&self) -> Option<usize> {
        self.output_fragment_count
    }

    /// Returns primitive-family counts for output loop fragments when materialized.
    pub const fn output_fragment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_fragment_kind_counts
    }

    /// Returns per-output-fragment source provenance when loops materialized.
    pub fn output_fragments(&self) -> &[BooleanBoundaryOutputFragmentReport2] {
        &self.output_fragments
    }

    /// Returns retained topology status for loop extraction.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized loop extraction.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl BooleanBoundaryLoopExtractionResult2 {
    /// Returns materialized loops, if extraction succeeded.
    pub const fn loops(&self) -> Option<&BooleanBoundaryLoopSet> {
        self.loops.as_ref()
    }

    /// Consumes this result and returns materialized loops, if any.
    pub fn into_loops(self) -> Option<BooleanBoundaryLoopSet> {
        self.loops
    }

    /// Returns retained loop-extraction evidence.
    pub const fn report(&self) -> &BooleanBoundaryLoopExtractionReport2 {
        &self.report
    }
}

impl BooleanBoundaryLoopConstructionReport2 {
    /// Returns the furthest exact construction stage reached.
    pub const fn stage(&self) -> BooleanBoundaryLoopConstructionStage2 {
        self.stage
    }

    /// Returns the number of source contours replayed.
    pub const fn source_contour_count(&self) -> usize {
        self.source_contour_count
    }

    /// Returns total source segment count across contours.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns primitive-family counts for source contour segments.
    pub const fn source_segment_kind_counts(&self) -> SegmentKindCounts {
        self.source_segment_kind_counts
    }

    /// Returns output loop count when materialized.
    pub const fn loop_count(&self) -> Option<usize> {
        self.loop_count
    }

    /// Returns output directed-fragment count when materialized.
    pub const fn output_fragment_count(&self) -> Option<usize> {
        self.output_fragment_count
    }

    /// Returns primitive-family counts for output fragments when materialized.
    pub const fn output_fragment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_fragment_kind_counts
    }

    /// Returns per-output-fragment source provenance when loops materialized.
    pub fn output_fragments(&self) -> &[BooleanBoundaryOutputFragmentReport2] {
        &self.output_fragments
    }

    /// Returns retained topology status for loop construction.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized loop construction.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl BooleanBoundaryLoopConstructionResult2 {
    /// Returns materialized loops, if construction succeeded.
    pub const fn loops(&self) -> Option<&BooleanBoundaryLoopSet> {
        self.loops.as_ref()
    }

    /// Consumes this result and returns materialized loops, if any.
    pub fn into_loops(self) -> Option<BooleanBoundaryLoopSet> {
        self.loops
    }

    /// Returns retained loop-construction evidence.
    pub const fn report(&self) -> &BooleanBoundaryLoopConstructionReport2 {
        &self.report
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

    /// Clones loop geometry into a checked closed contour.
    ///
    /// The checked constructor validates connectivity again instead of trusting
    /// the boolean graph. Foster, Hormann, and Popa emphasize that degenerate
    /// polygon clipping needs explicit validation around boundary coincidences
    /// (E. L. Foster, K. Hormann, and R. T. Popa, "Clipping simple polygons
    /// with degenerate intersections," Computers & Graphics: X 2, 100007,
    /// 2019), so this API keeps geometric validation visible at the conversion
    /// point.
    pub fn to_contour(&self, fill_rule: FillRule) -> CurveResult<Contour2> {
        Contour2::try_new_with_fill_rule(
            self.fragments
                .iter()
                .map(|fragment| fragment.segment.clone())
                .collect(),
            fill_rule,
        )
    }

    /// Consumes loop geometry into a checked closed contour.
    pub fn into_contour(self, fill_rule: FillRule) -> CurveResult<Contour2> {
        Contour2::try_new_with_fill_rule(
            self.fragments
                .into_iter()
                .map(|fragment| fragment.segment)
                .collect(),
            fill_rule,
        )
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

    /// Builds a loop set from already-decided closed contours.
    ///
    /// When higher-level boolean stages have already regularized a degenerate
    /// boundary-contact case to a set of closed contours (for example, when two
    /// boundaries share an edge that is a known full-seam overlap), the
    /// remaining work is structural transfer, not graph reconstruction. This
    /// conversion keeps the topological decision external to contour construction,
    /// matching the graph extraction model used by G. Greiner and K. Hormann while
    /// preserving the contour-only assumptions in `Contour2` as boundary facts
    /// rather than topology claims.
    ///
    /// Greiner and Hormann, "Efficient clipping of arbitrary polygons," ACM TOG 17(2),
    /// 71-83, 1998.
    pub fn from_contours(contours: Vec<Contour2>) -> CurveResult<Self> {
        Self::from_contours_with_report(contours)?
            .into_loops()
            .ok_or_else(|| {
                CurveError::Topology(
                    "boolean boundary loop construction did not materialize loops".to_owned(),
                )
            })
    }

    /// Builds a loop set from borrowed already-decided closed contours.
    ///
    /// This clones the exact contour carriers at the API boundary and then uses
    /// the same report-bearing structural transfer as
    /// [`BooleanBoundaryLoopSet::from_contours_with_report`].
    pub fn from_contours_borrowed(contours: &[Contour2]) -> CurveResult<Self> {
        Self::from_contours_borrowed_with_report(contours)?
            .into_loops()
            .ok_or_else(|| {
                CurveError::Topology(
                    "boolean boundary loop construction did not materialize loops".to_owned(),
                )
            })
    }

    /// Builds a loop set from already-decided contours and retains transfer evidence.
    ///
    /// Each source contour segment is replayed as a directed boolean fragment
    /// with retained source contour/fragment indices. Validation remains
    /// structural: this constructor does not claim to resolve boolean graph
    /// topology, it preserves a prior exact decision as closed boundary loops.
    pub fn from_contours_with_report(
        contours: Vec<Contour2>,
    ) -> CurveResult<BooleanBoundaryLoopConstructionResult2> {
        let source_contour_count = contours.len();
        let source_segment_count = contours.iter().map(Contour2::len).sum();
        let source_segment_kind_counts = contour_segment_kind_counts(&contours);
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
            match BooleanBoundaryLoop::new(fragments) {
                Ok(boundary_loop) => loops.push(boundary_loop),
                Err(_) => {
                    return Ok(blocked_boolean_boundary_loop_construction_result(
                        source_contour_count,
                        source_segment_count,
                        source_segment_kind_counts,
                        BooleanBoundaryLoopConstructionStage2::LoopMaterialization,
                        UncertaintyReason::Unsupported,
                    ));
                }
            }
        }

        match Self::new(loops) {
            Ok(loop_set) => Ok(decided_boolean_boundary_loop_construction_result(
                source_contour_count,
                source_segment_count,
                source_segment_kind_counts,
                loop_set,
            )),
            Err(_) => Ok(blocked_boolean_boundary_loop_construction_result(
                source_contour_count,
                source_segment_count,
                source_segment_kind_counts,
                BooleanBoundaryLoopConstructionStage2::LoopMaterialization,
                UncertaintyReason::Unsupported,
            )),
        }
    }

    /// Builds a loop set from borrowed already-decided contours and retains transfer evidence.
    ///
    /// This clones exact contour carriers only at the API boundary, preserving
    /// the same retained source contour/fragment reports as the owned
    /// constructor.
    pub fn from_contours_borrowed_with_report(
        contours: &[Contour2],
    ) -> CurveResult<BooleanBoundaryLoopConstructionResult2> {
        Self::from_contours_with_report(contours.to_vec())
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

    /// Clones every loop into a checked closed contour.
    pub fn to_contours(&self, fill_rule: FillRule) -> CurveResult<Vec<Contour2>> {
        self.loops
            .iter()
            .map(|boundary_loop| boundary_loop.to_contour(fill_rule))
            .collect()
    }

    /// Clones every loop into checked closed contours and retains transfer evidence.
    pub fn to_contours_with_report(
        &self,
        fill_rule: FillRule,
    ) -> BooleanBoundaryContourTransferResult2 {
        let mut contours = Vec::with_capacity(self.loops.len());
        for boundary_loop in &self.loops {
            match boundary_loop.to_contour(fill_rule) {
                Ok(contour) => contours.push(contour),
                Err(_) => {
                    return blocked_boolean_boundary_contour_transfer_result(
                        self.len(),
                        loop_set_fragment_count(self),
                        loop_set_fragment_kind_counts(self),
                        fill_rule,
                        BooleanBoundaryContourTransferStage2::ContourMaterialization,
                        UncertaintyReason::Unsupported,
                    );
                }
            }
        }
        decided_boolean_boundary_contour_transfer_result(
            self.len(),
            loop_set_fragment_count(self),
            loop_set_fragment_kind_counts(self),
            loop_set_output_fragment_reports(self),
            fill_rule,
            contours,
        )
    }

    /// Consumes every loop into a checked closed contour.
    pub fn into_contours(self, fill_rule: FillRule) -> CurveResult<Vec<Contour2>> {
        self.loops
            .into_iter()
            .map(|boundary_loop| boundary_loop.into_contour(fill_rule))
            .collect()
    }

    /// Consumes every loop into checked closed contours and retains transfer evidence.
    pub fn into_contours_with_report(
        self,
        fill_rule: FillRule,
    ) -> BooleanBoundaryContourTransferResult2 {
        let source_loop_count = self.len();
        let source_fragment_count = loop_set_fragment_count(&self);
        let source_fragment_kind_counts = loop_set_fragment_kind_counts(&self);
        let output_segments = loop_set_output_fragment_reports(&self);
        let mut contours = Vec::with_capacity(source_loop_count);
        for boundary_loop in self.loops {
            match boundary_loop.into_contour(fill_rule) {
                Ok(contour) => contours.push(contour),
                Err(_) => {
                    return blocked_boolean_boundary_contour_transfer_result(
                        source_loop_count,
                        source_fragment_count,
                        source_fragment_kind_counts,
                        fill_rule,
                        BooleanBoundaryContourTransferStage2::ContourMaterialization,
                        UncertaintyReason::Unsupported,
                    );
                }
            }
        }
        decided_boolean_boundary_contour_transfer_result(
            source_loop_count,
            source_fragment_count,
            source_fragment_kind_counts,
            output_segments,
            fill_rule,
            contours,
        )
    }
}

impl BooleanBoundaryContourTransferReport2 {
    /// Returns the fill rule used for materialized contours.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns the furthest exact contour-transfer stage reached.
    pub const fn stage(&self) -> BooleanBoundaryContourTransferStage2 {
        self.stage
    }

    /// Returns source boundary loop count.
    pub const fn source_loop_count(&self) -> usize {
        self.source_loop_count
    }

    /// Returns total source fragment count across loops.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }

    /// Returns primitive-family counts for source loop fragments.
    pub const fn source_fragment_kind_counts(&self) -> SegmentKindCounts {
        self.source_fragment_kind_counts
    }

    /// Returns contour count when transfer materialized.
    pub const fn contour_count(&self) -> Option<usize> {
        self.contour_count
    }

    /// Returns output contour segment count when transfer materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for output contour segments when materialized.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns per-output-segment source provenance when contours materialized.
    pub fn output_segments(&self) -> &[BooleanBoundaryOutputFragmentReport2] {
        &self.output_segments
    }

    /// Returns retained topology status for contour transfer.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized contour transfer.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl BooleanBoundaryContourTransferResult2 {
    /// Returns materialized contours, if transfer succeeded.
    pub fn contours(&self) -> Option<&[Contour2]> {
        self.contours.as_deref()
    }

    /// Consumes this result and returns materialized contours, if any.
    pub fn into_contours(self) -> Option<Vec<Contour2>> {
        self.contours
    }

    /// Returns retained contour-transfer evidence.
    pub const fn report(&self) -> &BooleanBoundaryContourTransferReport2 {
        &self.report
    }
}

type EndpointAdjacency = (Vec<Option<usize>>, Vec<Option<usize>>);

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
    let policy = CurvePolicy::certified();
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
    let policy = CurvePolicy::certified();
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
    match BooleanBoundaryChain::new(fragments, closed) {
        Ok(chain) => Classification::Decided(chain),
        Err(_) => Classification::Uncertain(crate::UncertaintyReason::Unsupported),
    }
}

fn decided_boolean_boundary_chain_assembly_result(
    source: &BooleanBoundaryFragmentSet,
    chains: BooleanBoundaryChainSet,
) -> BooleanBoundaryChainAssemblyResult2 {
    let chain_count = chains.len();
    let closed_chain_count = chains.closed_count();
    let output_fragment_count = chains.chains().iter().map(BooleanBoundaryChain::len).sum();
    let output_fragment_kind_counts = chain_set_fragment_kind_counts(&chains);
    let output_fragments = chain_set_output_fragment_reports(&chains);
    BooleanBoundaryChainAssemblyResult2 {
        chains: Some(chains),
        report: BooleanBoundaryChainAssemblyReport2 {
            stage: BooleanBoundaryChainAssemblyStage2::ChainMaterialization,
            directed_fragment_count: source.directed_len(),
            directed_fragment_kind_counts: directed_boolean_fragment_kind_counts(
                source.directed_fragments(),
            ),
            unresolved_boundary_count: source.unresolved_len(),
            chain_count: Some(chain_count),
            closed_chain_count: Some(closed_chain_count),
            open_chain_count: Some(chain_count - closed_chain_count),
            output_fragment_count: Some(output_fragment_count),
            output_fragment_kind_counts: Some(output_fragment_kind_counts),
            output_fragments,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    }
}

fn blocked_boolean_boundary_chain_assembly_result(
    source: &BooleanBoundaryFragmentSet,
    stage: BooleanBoundaryChainAssemblyStage2,
    blocker: UncertaintyReason,
) -> BooleanBoundaryChainAssemblyResult2 {
    BooleanBoundaryChainAssemblyResult2 {
        chains: None,
        report: BooleanBoundaryChainAssemblyReport2 {
            stage,
            directed_fragment_count: source.directed_len(),
            directed_fragment_kind_counts: directed_boolean_fragment_kind_counts(
                source.directed_fragments(),
            ),
            unresolved_boundary_count: source.unresolved_len(),
            chain_count: None,
            closed_chain_count: None,
            open_chain_count: None,
            output_fragment_count: None,
            output_fragment_kind_counts: None,
            output_fragments: Vec::new(),
            status: retained_status_for_chain_assembly_blocker(blocker),
            blocker: Some(blocker),
        },
    }
}

fn retained_status_for_chain_assembly_blocker(
    blocker: UncertaintyReason,
) -> RetainedTopologyStatus {
    match blocker {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn decided_boolean_boundary_loop_extraction_result(
    source: &BooleanBoundaryChainSet,
    loops: BooleanBoundaryLoopSet,
) -> BooleanBoundaryLoopExtractionResult2 {
    decided_boolean_boundary_loop_extraction_counts_result(
        source.len(),
        chain_set_fragment_count(source),
        chain_set_fragment_kind_counts(source),
        source.closed_count(),
        source.len() - source.closed_count(),
        loops,
    )
}

fn decided_boolean_boundary_loop_extraction_counts_result(
    source_chain_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    closed_chain_count: usize,
    open_chain_count: usize,
    loops: BooleanBoundaryLoopSet,
) -> BooleanBoundaryLoopExtractionResult2 {
    let output_fragment_count = loops.loops().iter().map(BooleanBoundaryLoop::len).sum();
    let output_fragment_kind_counts = loop_set_fragment_kind_counts(&loops);
    let output_fragments = loop_set_output_fragment_reports(&loops);
    BooleanBoundaryLoopExtractionResult2 {
        loops: Some(loops),
        report: BooleanBoundaryLoopExtractionReport2 {
            stage: BooleanBoundaryLoopExtractionStage2::LoopMaterialization,
            source_chain_count,
            source_fragment_count,
            source_fragment_kind_counts,
            closed_chain_count,
            open_chain_count,
            loop_count: Some(closed_chain_count),
            output_fragment_count: Some(output_fragment_count),
            output_fragment_kind_counts: Some(output_fragment_kind_counts),
            output_fragments,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    }
}

fn blocked_boolean_boundary_loop_extraction_result(
    source: &BooleanBoundaryChainSet,
    stage: BooleanBoundaryLoopExtractionStage2,
    blocker: UncertaintyReason,
) -> BooleanBoundaryLoopExtractionResult2 {
    blocked_boolean_boundary_loop_extraction_counts_result(
        stage,
        source.len(),
        chain_set_fragment_count(source),
        chain_set_fragment_kind_counts(source),
        source.closed_count(),
        source.len() - source.closed_count(),
        blocker,
    )
}

fn blocked_boolean_boundary_loop_extraction_counts_result(
    stage: BooleanBoundaryLoopExtractionStage2,
    source_chain_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    closed_chain_count: usize,
    open_chain_count: usize,
    blocker: UncertaintyReason,
) -> BooleanBoundaryLoopExtractionResult2 {
    BooleanBoundaryLoopExtractionResult2 {
        loops: None,
        report: BooleanBoundaryLoopExtractionReport2 {
            stage,
            source_chain_count,
            source_fragment_count,
            source_fragment_kind_counts,
            closed_chain_count,
            open_chain_count,
            loop_count: None,
            output_fragment_count: None,
            output_fragment_kind_counts: None,
            output_fragments: Vec::new(),
            status: retained_status_for_loop_extraction_blocker(blocker),
            blocker: Some(blocker),
        },
    }
}

fn retained_status_for_loop_extraction_blocker(
    blocker: UncertaintyReason,
) -> RetainedTopologyStatus {
    match blocker {
        UncertaintyReason::Unsupported => RetainedTopologyStatus::Unsupported,
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn decided_boolean_boundary_loop_construction_result(
    source_contour_count: usize,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    loops: BooleanBoundaryLoopSet,
) -> BooleanBoundaryLoopConstructionResult2 {
    let output_fragment_count = loop_set_fragment_count(&loops);
    let output_fragment_kind_counts = loop_set_fragment_kind_counts(&loops);
    let output_fragments = loop_set_output_fragment_reports(&loops);
    BooleanBoundaryLoopConstructionResult2 {
        loops: Some(loops),
        report: BooleanBoundaryLoopConstructionReport2 {
            stage: BooleanBoundaryLoopConstructionStage2::LoopMaterialization,
            source_contour_count,
            source_segment_count,
            source_segment_kind_counts,
            loop_count: Some(source_contour_count),
            output_fragment_count: Some(output_fragment_count),
            output_fragment_kind_counts: Some(output_fragment_kind_counts),
            output_fragments,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    }
}

fn blocked_boolean_boundary_loop_construction_result(
    source_contour_count: usize,
    source_segment_count: usize,
    source_segment_kind_counts: SegmentKindCounts,
    stage: BooleanBoundaryLoopConstructionStage2,
    blocker: UncertaintyReason,
) -> BooleanBoundaryLoopConstructionResult2 {
    BooleanBoundaryLoopConstructionResult2 {
        loops: None,
        report: BooleanBoundaryLoopConstructionReport2 {
            stage,
            source_contour_count,
            source_segment_count,
            source_segment_kind_counts,
            loop_count: None,
            output_fragment_count: None,
            output_fragment_kind_counts: None,
            output_fragments: Vec::new(),
            status: RetainedTopologyStatus::Unsupported,
            blocker: Some(blocker),
        },
    }
}

fn chain_set_fragment_count(chains: &BooleanBoundaryChainSet) -> usize {
    chains.chains().iter().map(BooleanBoundaryChain::len).sum()
}

fn chain_set_fragment_kind_counts(chains: &BooleanBoundaryChainSet) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for chain in chains.chains() {
        add_directed_fragment_kind_counts(&mut counts, chain.fragments());
    }
    counts
}

fn chain_set_output_fragment_reports(
    chains: &BooleanBoundaryChainSet,
) -> Vec<BooleanBoundaryOutputFragmentReport2> {
    chains
        .chains()
        .iter()
        .flat_map(BooleanBoundaryChain::fragments)
        .enumerate()
        .map(
            |(output_fragment_index, fragment)| BooleanBoundaryOutputFragmentReport2 {
                key: fragment.key,
                fragment_index: fragment.fragment_index,
                source_segment_index: fragment.source_segment_index,
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

fn decided_boolean_boundary_contour_transfer_result(
    source_loop_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    output_segments: Vec<BooleanBoundaryOutputFragmentReport2>,
    fill_rule: FillRule,
    contours: Vec<Contour2>,
) -> BooleanBoundaryContourTransferResult2 {
    let output_segment_count = contours.iter().map(Contour2::len).sum();
    let output_segment_kind_counts = contour_segment_kind_counts(&contours);
    BooleanBoundaryContourTransferResult2 {
        report: BooleanBoundaryContourTransferReport2 {
            fill_rule,
            stage: BooleanBoundaryContourTransferStage2::ContourMaterialization,
            source_loop_count,
            source_fragment_count,
            source_fragment_kind_counts,
            contour_count: Some(contours.len()),
            output_segment_count: Some(output_segment_count),
            output_segment_kind_counts: Some(output_segment_kind_counts),
            output_segments,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
        contours: Some(contours),
    }
}

fn blocked_boolean_boundary_contour_transfer_result(
    source_loop_count: usize,
    source_fragment_count: usize,
    source_fragment_kind_counts: SegmentKindCounts,
    fill_rule: FillRule,
    stage: BooleanBoundaryContourTransferStage2,
    blocker: UncertaintyReason,
) -> BooleanBoundaryContourTransferResult2 {
    BooleanBoundaryContourTransferResult2 {
        contours: None,
        report: BooleanBoundaryContourTransferReport2 {
            fill_rule,
            stage,
            source_loop_count,
            source_fragment_count,
            source_fragment_kind_counts,
            contour_count: None,
            output_segment_count: None,
            output_segment_kind_counts: None,
            output_segments: Vec::new(),
            status: RetainedTopologyStatus::Unsupported,
            blocker: Some(blocker),
        },
    }
}

fn loop_set_fragment_count(loops: &BooleanBoundaryLoopSet) -> usize {
    loops.loops().iter().map(BooleanBoundaryLoop::len).sum()
}

fn loop_set_fragment_kind_counts(loops: &BooleanBoundaryLoopSet) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for boundary_loop in loops.loops() {
        add_directed_fragment_kind_counts(&mut counts, boundary_loop.fragments());
    }
    counts
}

fn directed_boolean_fragment_kind_counts(
    fragments: &[DirectedBooleanFragment],
) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    add_directed_fragment_kind_counts(&mut counts, fragments);
    counts
}

fn add_directed_fragment_kind_counts(
    counts: &mut SegmentKindCounts,
    fragments: &[DirectedBooleanFragment],
) {
    for fragment in fragments {
        add_segment_kind_count(counts, &fragment.segment);
    }
}

fn loop_set_output_fragment_reports(
    loops: &BooleanBoundaryLoopSet,
) -> Vec<BooleanBoundaryOutputFragmentReport2> {
    loops
        .loops()
        .iter()
        .flat_map(BooleanBoundaryLoop::fragments)
        .enumerate()
        .map(
            |(output_fragment_index, fragment)| BooleanBoundaryOutputFragmentReport2 {
                key: fragment.key,
                fragment_index: fragment.fragment_index,
                source_segment_index: fragment.source_segment_index,
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

fn contour_segment_kind_counts(contours: &[Contour2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for contour in contours {
        for segment in contour.segments() {
            add_segment_kind_count(&mut counts, segment);
        }
    }
    counts
}

fn add_segment_kind_count(counts: &mut SegmentKindCounts, segment: &Segment2) {
    match segment {
        Segment2::Line(_) => counts.lines += 1,
        Segment2::Arc(_) => counts.arcs += 1,
    }
}

fn endpoint_adjacency(
    fragments: &[DirectedBooleanFragment],
    policy: &CurvePolicy,
) -> Classification<EndpointAdjacency> {
    let mut successors = vec![None; fragments.len()];
    let mut predecessors = vec![None; fragments.len()];

    for (left_index, left) in fragments.iter().enumerate() {
        for (right_index, right) in fragments.iter().enumerate() {
            if left_index == right_index {
                continue;
            }

            let matches = match points_match(left.segment.end(), right.segment.start(), policy) {
                Classification::Decided(matches) => matches,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            if !matches {
                continue;
            }

            if successors[left_index].replace(right_index).is_some()
                || predecessors[right_index].replace(left_index).is_some()
            {
                return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
            }
        }
    }

    Classification::Decided((successors, predecessors))
}

fn follow_chain(
    start: usize,
    fragments: &[DirectedBooleanFragment],
    successors: &[Option<usize>],
    used: &mut [bool],
) -> Classification<BooleanBoundaryChain> {
    let mut chain = Vec::new();
    let mut current = start;
    let mut closed = false;

    loop {
        if used[current] {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        }

        used[current] = true;
        chain.push(fragments[current].clone());

        let Some(next) = successors[current] else {
            break;
        };

        if next == start {
            closed = true;
            break;
        }
        if used[next] {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        }

        current = next;
    }

    decided_boolean_boundary_chain(chain, closed)
}

fn points_match(
    left: &crate::Point2,
    right: &crate::Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let distance = left.distance_squared(right);
    match is_zero(&distance, policy) {
        Some(matches) => Classification::Decided(matches),
        None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
    }
}
