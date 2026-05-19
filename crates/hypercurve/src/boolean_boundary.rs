//! Directed boolean boundary traversal and loop reconstruction.
//!
//! This module owns the graph-facing part of boolean construction: selected
//! fragments are already classified, oriented, and ready to be connected into
//! chains. It deliberately stops before material/hole role assignment.

use crate::boolean::BooleanFragmentClassification;
use crate::classify::is_zero;
use crate::{
    Classification, Contour2, CurvePolicy, CurveResult, FillRule, RegionContourKey,
    RegionContourRole, RegionSide, Segment2,
};

/// A selected fragment with geometry already oriented for result traversal.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectedBooleanFragment {
    /// Source keyed contour.
    pub key: crate::RegionContourKey,
    /// Index within [`crate::RegionContourFragments::fragments`].
    pub fragment_index: usize,
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
    pub const fn new(
        directed_fragments: Vec<DirectedBooleanFragment>,
        unresolved_boundaries: Vec<BooleanFragmentClassification>,
    ) -> Self {
        Self {
            directed_fragments,
            unresolved_boundaries,
        }
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
        if !self.unresolved_boundaries.is_empty() {
            return Classification::Uncertain(crate::UncertaintyReason::Boundary);
        }

        let (successors, predecessors) = match endpoint_adjacency(&self.directed_fragments, policy)
        {
            Classification::Decided(adjacency) => adjacency,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        let mut used = vec![false; self.directed_fragments.len()];
        let mut chains = Vec::new();

        for index in 0..self.directed_fragments.len() {
            if predecessors[index].is_none() && !used[index] {
                let chain =
                    match follow_chain(index, &self.directed_fragments, &successors, &mut used) {
                        Classification::Decided(chain) => chain,
                        Classification::Uncertain(reason) => {
                            return Classification::Uncertain(reason);
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
                            return Classification::Uncertain(reason);
                        }
                    };
                chains.push(chain);
            }
        }

        Classification::Decided(BooleanBoundaryChainSet::new(chains))
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
    pub const fn new(fragments: Vec<DirectedBooleanFragment>, closed: bool) -> Self {
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
    pub const fn new(chains: Vec<BooleanBoundaryChain>) -> Self {
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
        if self.chains.iter().any(|chain| !chain.is_closed()) {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        }

        Classification::Decided(BooleanBoundaryLoopSet::new(
            self.chains
                .iter()
                .map(|chain| BooleanBoundaryLoop::new(chain.fragments.clone()))
                .collect(),
        ))
    }

    /// Consumes the chain set and extracts closed chains as boundary loops.
    pub fn into_closed_loops(self) -> Classification<BooleanBoundaryLoopSet> {
        if self.chains.iter().any(|chain| !chain.is_closed()) {
            return Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        }

        Classification::Decided(BooleanBoundaryLoopSet::new(
            self.chains
                .into_iter()
                .map(|chain| BooleanBoundaryLoop::new(chain.fragments))
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
    pub const fn new(fragments: Vec<DirectedBooleanFragment>) -> Self {
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
    pub const fn new(loops: Vec<BooleanBoundaryLoop>) -> Self {
        Self { loops }
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
    pub fn from_contours(contours: Vec<Contour2>) -> Self {
        let mut loops = Vec::with_capacity(contours.len());

        for (index, contour) in contours.into_iter().enumerate() {
            let fragments = contour
                .segments()
                .iter()
                .enumerate()
                .map(|(fragment_index, segment)| DirectedBooleanFragment {
                    key: RegionContourKey::new(RegionSide::First, RegionContourRole::Material, index),
                    fragment_index,
                    segment: segment.clone(),
                })
                .collect();
            loops.push(BooleanBoundaryLoop::new(fragments));
        }

        Self { loops }
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

    /// Consumes every loop into a checked closed contour.
    pub fn into_contours(self, fill_rule: FillRule) -> CurveResult<Vec<Contour2>> {
        self.loops
            .into_iter()
            .map(|boundary_loop| boundary_loop.into_contour(fill_rule))
            .collect()
    }
}

type EndpointAdjacency = (Vec<Option<usize>>, Vec<Option<usize>>);

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

    Classification::Decided(BooleanBoundaryChain::new(chain, closed))
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
