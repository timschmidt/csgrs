//! Report-bearing boolean boundary traversal.
//!
//! This module exposes the raw split/classify/emit/chain/loop stage as an
//! auditable object.  The legacy loop API deliberately returns
//! [`Classification::Uncertain`] for unresolved shared boundaries and unsupported
//! traversal graphs.  This report keeps those cases explicit as certified
//! blockers, following Yap's exact-geometric-computation requirement that
//! uncertainty and incomplete combinatorial construction remain visible at the
//! object API boundary (C. K. Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997).

use crate::{
    BooleanBoundaryLoopSet, BooleanFragmentAction, BooleanOp, Classification, CurvePolicy,
    CurveResult, Region2, RegionView2, UncertaintyReason, prepared::PreparedRegionView2,
};

/// Status of the boolean boundary traversal stage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanBoundaryTraversalStatus {
    /// The operation emitted no boundary fragments and therefore no loops.
    Empty,
    /// All emitted fragments assembled into closed loops.
    LoopsReady,
    /// At least one shared-boundary fragment requires a degenerate-overlap
    /// resolver before graph traversal is valid.
    UnresolvedBoundaries,
    /// Directed fragments assembled, but at least one chain remained open.
    OpenChains,
    /// Endpoint adjacency or traversal encountered a branch/cycle shape that
    /// the current raw loop extractor intentionally does not resolve.
    UnsupportedTraversal,
}

impl BooleanBoundaryTraversalStatus {
    /// Returns true when the report contains a decided loop set.
    pub const fn is_ready(self) -> bool {
        matches!(self, Self::Empty | Self::LoopsReady)
    }
}

/// Report for the raw boolean boundary traversal stage.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanBoundaryTraversalReport2 {
    /// Requested operation.
    pub operation: BooleanOp,
    /// Final traversal status.
    pub status: BooleanBoundaryTraversalStatus,
    /// Number of classified source fragments.
    pub classified_fragment_count: usize,
    /// Number of classifications discarded by the operation.
    pub discarded_fragment_count: usize,
    /// Number of fragments emitted in source direction.
    pub kept_source_direction_count: usize,
    /// Number of fragments emitted in reversed direction.
    pub kept_reversed_count: usize,
    /// Number of shared-boundary classifications retained for later resolution.
    pub unresolved_boundary_count: usize,
    /// Number of directed fragments passed to endpoint graph traversal.
    pub directed_fragment_count: usize,
    /// Number of chains assembled before loop extraction.
    pub assembled_chain_count: usize,
    /// Number of assembled chains whose endpoints reconnect.
    pub closed_chain_count: usize,
    /// Number of assembled chains whose endpoints do not reconnect.
    pub open_chain_count: usize,
    /// Exact reason for a non-ready traversal blocker.
    ///
    /// Ready reports use `None`. Reports with
    /// [`BooleanBoundaryTraversalStatus::UnresolvedBoundaries`] use
    /// [`UncertaintyReason::Boundary`]; reports with
    /// [`BooleanBoundaryTraversalStatus::OpenChains`] or
    /// [`BooleanBoundaryTraversalStatus::UnsupportedTraversal`] use
    /// [`UncertaintyReason::Unsupported`]. Yap's exact-computation model treats
    /// these as first-class object facts instead of hiding them behind an
    /// implementation failure code.
    pub blocker_reason: Option<UncertaintyReason>,
    /// Closed loops when `status` is [`BooleanBoundaryTraversalStatus::Empty`]
    /// or [`BooleanBoundaryTraversalStatus::LoopsReady`].
    pub loops: Option<BooleanBoundaryLoopSet>,
}

impl BooleanBoundaryTraversalReport2 {
    /// Returns true when the report contains a decided loop set.
    pub const fn is_ready(&self) -> bool {
        self.status.is_ready()
    }
}

impl Region2 {
    /// Computes raw boolean boundary traversal and returns a structured report.
    ///
    /// The report exposes the split/classify/emit/chain/loop stage used by
    /// [`Region2::boolean_boundary_loops`].  Greiner and Hormann model polygon
    /// clipping as traversal of classified polygon chains (G. Greiner and
    /// K. Hormann, "Efficient clipping of arbitrary polygons," *ACM
    /// Transactions on Graphics* 17(2), 1998).  Foster, Hormann, and Popa show
    /// that degenerate shared-boundary cases need explicit handling separate
    /// from ordinary traversal ("Clipping simple polygons with degenerate
    /// intersections," *Computers & Graphics: X* 2, 2019); this report names
    /// that blocker instead of flattening it into an opaque failure.
    pub fn boolean_boundary_traversal_report(
        &self,
        other: &Self,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryTraversalReport2>> {
        self.as_view()
            .boolean_boundary_traversal_report(&other.as_view(), op, policy)
    }
}

impl RegionView2<'_> {
    /// Computes raw boolean boundary traversal and returns a structured report.
    pub fn boolean_boundary_traversal_report(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryTraversalReport2>> {
        let intersections = self.intersect_region(other, policy)?;
        let fragments = match intersections.split_regions(self, other, policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let selection = match fragments.classify_for_boolean(self, other, op, policy)? {
            Classification::Decided(selection) => selection,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        traversal_report_from_selection(op, &selection, &fragments, policy)
    }

    /// Computes a raw traversal report against a prepared right operand.
    pub fn boolean_boundary_traversal_report_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryTraversalReport2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_traversal_report(other, op, policy)
    }
}

impl PreparedRegionView2<'_> {
    /// Computes raw boolean boundary traversal between prepared operands.
    ///
    /// Prepared caches accelerate intersection and representative-point
    /// classification only.  The emitted graph and loop readiness facts in the
    /// returned report are replayed from the constructed fragments, so cached
    /// broad-phase data is not treated as proof.
    pub fn boolean_boundary_traversal_report(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryTraversalReport2>> {
        let first_view = self.as_region_view();
        let second_view = other.as_region_view();
        let intersections = self.intersect_prepared_region(other, policy)?;
        let fragments = match intersections.split_regions(&first_view, &second_view, policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let selection = match crate::prepared_boolean::classify_fragments_with_prepared_regions(
            &fragments, self, other, op, policy,
        )? {
            Classification::Decided(selection) => selection,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        traversal_report_from_selection(op, &selection, &fragments, policy)
    }

    /// Computes a raw traversal report against an ordinary region view.
    pub fn boolean_boundary_traversal_report_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryTraversalReport2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_traversal_report(&other, op, policy)
    }
}

fn traversal_report_from_selection(
    op: BooleanOp,
    selection: &crate::BooleanFragmentSelection,
    fragments: &crate::RegionFragmentSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryTraversalReport2>> {
    let emitted = selection.emit_boundary_fragments(fragments)?;
    let base = BooleanBoundaryTraversalReport2 {
        operation: op,
        status: BooleanBoundaryTraversalStatus::UnresolvedBoundaries,
        classified_fragment_count: selection.len(),
        discarded_fragment_count: selection.count_action(BooleanFragmentAction::Discard),
        kept_source_direction_count: selection
            .count_action(BooleanFragmentAction::KeepSourceDirection),
        kept_reversed_count: selection.count_action(BooleanFragmentAction::KeepReversed),
        unresolved_boundary_count: selection
            .count_action(BooleanFragmentAction::BoundaryNeedsResolution),
        directed_fragment_count: emitted.directed_len(),
        assembled_chain_count: 0,
        closed_chain_count: 0,
        open_chain_count: 0,
        blocker_reason: Some(UncertaintyReason::Boundary),
        loops: None,
    };

    if !emitted.is_ready_for_traversal() {
        return Ok(Classification::Decided(base));
    }

    let chains = match emitted.assemble_chains(policy) {
        Classification::Decided(chains) => chains,
        Classification::Uncertain(UncertaintyReason::Unsupported) => {
            return Ok(Classification::Decided(BooleanBoundaryTraversalReport2 {
                status: BooleanBoundaryTraversalStatus::UnsupportedTraversal,
                blocker_reason: Some(UncertaintyReason::Unsupported),
                ..base
            }));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let closed_chain_count = chains.closed_count();
    let assembled_chain_count = chains.len();
    let open_chain_count = assembled_chain_count - closed_chain_count;
    if open_chain_count > 0 {
        return Ok(Classification::Decided(BooleanBoundaryTraversalReport2 {
            status: BooleanBoundaryTraversalStatus::OpenChains,
            assembled_chain_count,
            closed_chain_count,
            open_chain_count,
            blocker_reason: Some(UncertaintyReason::Unsupported),
            ..base
        }));
    }

    let loops = match chains.closed_loops() {
        Classification::Decided(loops) => loops,
        Classification::Uncertain(UncertaintyReason::Unsupported) => {
            return Ok(Classification::Decided(BooleanBoundaryTraversalReport2 {
                status: BooleanBoundaryTraversalStatus::UnsupportedTraversal,
                assembled_chain_count,
                closed_chain_count,
                open_chain_count,
                blocker_reason: Some(UncertaintyReason::Unsupported),
                ..base
            }));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let status = if loops.is_empty() {
        BooleanBoundaryTraversalStatus::Empty
    } else {
        BooleanBoundaryTraversalStatus::LoopsReady
    };

    Ok(Classification::Decided(BooleanBoundaryTraversalReport2 {
        status,
        assembled_chain_count,
        closed_chain_count,
        open_chain_count,
        blocker_reason: None,
        loops: Some(loops),
        ..base
    }))
}
