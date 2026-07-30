//! Retained traversal graph for split Bezier and conic fragments.
//!
//! Higher-order booleans need an arrangement graph before they can emit
//! concrete regions. This module is a deliberately small, testable traversal
//! substrate for the fragments produced by [`BezierSplitMaterialization2`]: it
//! connects materialized Bezier/conic fragments by exact endpoint equality,
//! follows branch-free chains, and can optionally resolve simple branch
//! vertices by exact tangent angle order. A retained traversal variant also
//! consumes algebraic endpoint-image fragments whose represented point and
//! tangent evidence is present, while still refusing unresolved fragments,
//! overlaps, coincident tangents, and zero tangents.
//!
//! That boundary is exact-computation discipline: topology code may retain unresolved exact objects, but it must
//! not invent a floating successor. The branch-free chain walk mirrors the
//! regularized graph assumption in polygon clipping traversal
//! while multi-successor handling follows the degenerate-intersection clipping
//! model: when local
//! order is not certified, traversal stops instead of guessing.

use std::{cmp::Ordering, collections::HashMap, fmt, sync::OnceLock};

use hyperreal::{Rational, Real, RealSign};
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticStatus, AlgebraicRootRepresentation,
    arithmetic_algebraic_root_representations,
};
#[cfg(feature = "predicates")]
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    compare_algebraic_root_representations_by_difference,
};

use crate::bezier_tangent_order::{
    compare_algebraic_tangent_filled_left_face_sign_only,
    compare_algebraic_tangent_turn_from_base_sign_only,
};
use crate::classify::{compare_reals, is_zero, real_sign};
use crate::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicSameTangentOrderStatus,
    BezierAlgebraicTangentOrderStatus, BezierAlgebraicTangentVector2, BezierEndpoint,
    BezierEndpointPointImage2, BezierEndpointTangentImage2, BezierParameter2,
    BezierRetainedOverlapEvidence2, BezierSplitFragment2, BezierSplitMaterialization2,
    BezierSubcurve2, BezierTangentTurnOrdering2, Classification, CurveError, CurvePolicy,
    CurveResult, Point2, UncertaintyReason, ZeroStatus,
    compare_algebraic_same_tangent_second_order, compare_algebraic_same_tangent_third_order,
};

/// One retained Bezier arrangement fragment with source provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierArrangementFragment2 {
    source_curve_index: usize,
    source_fragment_index: usize,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
    fragment: BezierSplitFragment2,
}

/// Branch-free retained Bezier arrangement graph.
pub struct BezierArrangementGraph2 {
    fragments: Vec<BezierArrangementFragment2>,
    certified_overlap_evidence: OnceLock<Box<Classification<BezierRetainedOverlapEvidence2>>>,
}

/// One endpoint-connected traversal chain through retained Bezier fragments.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierArrangementChain2 {
    fragment_indices: Vec<usize>,
    closed: bool,
}

/// Traversal result for a branch-free retained Bezier arrangement graph.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BezierArrangementTraversal2 {
    chains: Vec<BezierArrangementChain2>,
}

impl BezierArrangementFragment2 {
    /// Constructs a retained fragment from split-materialization provenance.
    pub const fn new(
        source_curve_index: usize,
        source_fragment_index: usize,
        fragment: BezierSplitFragment2,
    ) -> Self {
        Self {
            source_curve_index,
            source_fragment_index,
            start_topology_vertex: None,
            end_topology_vertex: None,
            fragment,
        }
    }

    pub(crate) const fn with_topology_vertices(
        mut self,
        start_topology_vertex: Option<usize>,
        end_topology_vertex: Option<usize>,
    ) -> Self {
        self.start_topology_vertex = start_topology_vertex;
        self.end_topology_vertex = end_topology_vertex;
        self
    }

    pub(crate) const fn start_topology_vertex(&self) -> Option<usize> {
        self.start_topology_vertex
    }

    pub(crate) const fn end_topology_vertex(&self) -> Option<usize> {
        self.end_topology_vertex
    }

    /// Returns the source curve index supplied to the graph builder.
    pub const fn source_curve_index(&self) -> usize {
        self.source_curve_index
    }

    /// Returns the fragment index within the source split materialization.
    pub const fn source_fragment_index(&self) -> usize {
        self.source_fragment_index
    }

    /// Returns the retained split fragment.
    pub const fn fragment(&self) -> &BezierSplitFragment2 {
        &self.fragment
    }
}

impl Clone for BezierArrangementGraph2 {
    fn clone(&self) -> Self {
        let clone = Self {
            fragments: self.fragments.clone(),
            certified_overlap_evidence: OnceLock::new(),
        };
        if let Some(evidence) = self.cached_certified_overlap_evidence() {
            clone.cache_certified_overlap_evidence(evidence);
        }
        clone
    }
}

impl fmt::Debug for BezierArrangementGraph2 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("BezierArrangementGraph2")
            .field("fragments", &self.fragments)
            .finish()
    }
}

impl Default for BezierArrangementGraph2 {
    fn default() -> Self {
        Self {
            fragments: Vec::new(),
            certified_overlap_evidence: OnceLock::new(),
        }
    }
}

impl PartialEq for BezierArrangementGraph2 {
    fn eq(&self, other: &Self) -> bool {
        self.fragments == other.fragments
    }
}

impl BezierArrangementGraph2 {
    /// Constructs a retained graph from split materializations in source order.
    pub fn from_split_materializations(
        materializations: &[BezierSplitMaterialization2],
    ) -> CurveResult<Self> {
        let fragments = materializations
            .iter()
            .enumerate()
            .flat_map(|(source_curve_index, materialization)| {
                materialization.fragments().iter().cloned().enumerate().map(
                    move |(source_fragment_index, fragment)| {
                        BezierArrangementFragment2::new(
                            source_curve_index,
                            source_fragment_index,
                            fragment,
                        )
                    },
                )
            })
            .collect();
        Self::new(fragments)
    }

    /// Constructs a graph from already-retained fragments.
    pub fn new(fragments: Vec<BezierArrangementFragment2>) -> CurveResult<Self> {
        validate_arrangement_fragment_provenance(&fragments)?;
        Ok(Self::from_certified_fragments(fragments))
    }

    pub(crate) fn from_certified_fragments(fragments: Vec<BezierArrangementFragment2>) -> Self {
        Self {
            fragments,
            certified_overlap_evidence: OnceLock::new(),
        }
    }

    pub(crate) fn cached_certified_overlap_evidence(
        &self,
    ) -> Option<Classification<BezierRetainedOverlapEvidence2>> {
        self.certified_overlap_evidence
            .get()
            .map(|evidence| evidence.as_ref().clone())
    }

    pub(crate) fn cache_certified_overlap_evidence(
        &self,
        evidence: Classification<BezierRetainedOverlapEvidence2>,
    ) {
        let _ = self.certified_overlap_evidence.set(Box::new(evidence));
    }

    /// Returns retained fragments.
    pub fn fragments(&self) -> &[BezierArrangementFragment2] {
        &self.fragments
    }

    /// Returns true when no fragments are retained.
    pub fn is_empty(&self) -> bool {
        self.fragments.is_empty()
    }

    /// Returns the number of retained fragments.
    pub fn len(&self) -> usize {
        self.fragments.len()
    }

    /// Traverses branch-free materialized fragments into endpoint-connected chains.
    pub fn traverse_branch_free(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        let mut endpoints = Vec::with_capacity(self.fragments.len());
        for fragment in &self.fragments {
            let endpoints_for_fragment = match materialized_endpoints(fragment.fragment()) {
                Some(endpoints) => endpoints,
                None => return Classification::Uncertain(UncertaintyReason::Boundary),
            };
            endpoints.push(endpoints_for_fragment);
        }

        let (successors, predecessors) = match endpoint_adjacency(&endpoints, policy) {
            Classification::Decided(adjacency) => adjacency,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        let mut used = vec![false; self.fragments.len()];
        let mut chains = Vec::new();
        for index in 0..self.fragments.len() {
            if predecessors[index].is_none() && !used[index] {
                let chain = follow_chain(index, &successors, &endpoints, &mut used, policy);
                match chain {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }
        for index in 0..self.fragments.len() {
            if !used[index] {
                let chain = follow_chain(index, &successors, &endpoints, &mut used, policy);
                match chain {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }

        decided_arrangement_traversal(chains)
    }

    /// Traverses materialized fragments and resolves simple branches by tangent order.
    ///
    /// At a branch vertex, the outgoing fragment with the smallest certified
    /// counter-clockwise turn from the incoming endpoint tangent is selected.
    /// The comparison is exact: it uses signs of cross and dot products, not
    /// finite angles. This is the local-order step needed before full
    /// higher-order arrangement traversal can emit regions. Ties, zero
    /// tangents, unresolved split boundaries, and uncertain signs remain
    /// explicit uncertainty in the exactness model's sense.
    pub fn traverse_with_tangent_order(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        let mut endpoints = Vec::with_capacity(self.fragments.len());
        for fragment in &self.fragments {
            let endpoints_for_fragment =
                match materialized_endpoint_data(fragment.fragment(), policy) {
                    Some(Classification::Decided(endpoints)) => endpoints,
                    Some(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    None => return Classification::Uncertain(UncertaintyReason::Boundary),
                };
            endpoints.push(endpoints_for_fragment);
        }

        let (outgoing, predecessors) = match tangent_adjacency(&endpoints, policy) {
            Classification::Decided(adjacency) => adjacency,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        let mut used = vec![false; self.fragments.len()];
        let mut chains = Vec::new();
        for index in 0..self.fragments.len() {
            if predecessors[index] == 0 && !used[index] {
                match follow_tangent_ordered_chain(index, &outgoing, &endpoints, &mut used, policy)
                {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }
        for index in 0..self.fragments.len() {
            if !used[index] {
                match follow_tangent_ordered_chain(index, &outgoing, &endpoints, &mut used, policy)
                {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }

        decided_arrangement_traversal(chains)
    }

    /// Traverses retained fragments using native and algebraic endpoint evidence.
    ///
    /// This is the first traversal consumer for
    /// [`BezierSplitFragment2::AlgebraicEndpointImages`]. It connects endpoints
    /// only when the retained point evidence is exact and structurally equal
    /// (or when a represented coordinate has an exact rational witness matching
    /// a native point). At a branch vertex it compares outgoing tangents with
    /// either the native exact cross/dot predicate or
    /// [`crate::compare_algebraic_tangent_turn_from_base`].
    ///
    /// The method deliberately does not materialize concrete Bezier regions
    /// from algebraic fragments. It only proves traversal order over retained
    /// evidence, preserving the exactness model's construction/decision boundary from
    /// exact-computation discipline, and matching the arrangement local-order discipline in de
    /// standard arrangement algorithms.
    pub fn traverse_retained_with_tangent_order(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        self.traverse_retained_with_certified_successors(&[], policy)
    }

    pub(crate) fn traverse_retained_with_certified_successors(
        &self,
        certified_successors: &[Option<usize>],
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        self.traverse_retained_with_successor_rule(certified_successors, false, policy)
    }

    pub(crate) fn traverse_retained_filled_left_faces_with_certified_successors(
        &self,
        certified_successors: &[Option<usize>],
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        self.traverse_retained_with_successor_rule(certified_successors, true, policy)
    }

    fn traverse_retained_with_successor_rule(
        &self,
        certified_successors: &[Option<usize>],
        filled_left_faces: bool,
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        let defer_tangent_order_evidence = !certified_successors.is_empty();
        let complete_topology_vertices = defer_tangent_order_evidence
            && self.fragments.iter().all(|fragment| {
                fragment.start_topology_vertex().is_some()
                    && fragment.end_topology_vertex().is_some()
            });
        let initial_endpoint_scope = if defer_tangent_order_evidence {
            RetainedEndpointScope::Connectivity
        } else {
            RetainedEndpointScope::TangentOrder
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "retained-endpoint-scope",
            if complete_topology_vertices {
                "topology-connectivity-first"
            } else if defer_tangent_order_evidence {
                "coordinate-connectivity-first"
            } else {
                "tangent-order-immediate"
            },
        );
        let mut endpoints = Vec::with_capacity(self.fragments.len());
        for fragment in &self.fragments {
            let endpoints_for_fragment = if complete_topology_vertices {
                retained_topology_endpoint_data(fragment)
            } else {
                match retained_endpoint_data(fragment, initial_endpoint_scope, policy) {
                    Some(Classification::Decided(endpoints)) => endpoints,
                    Some(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    None => return Classification::Uncertain(UncertaintyReason::Boundary),
                }
            };
            endpoints.push(endpoints_for_fragment);
        }

        let (outgoing, predecessors) = match retained_tangent_adjacency(&endpoints, policy) {
            Classification::Decided(adjacency) => adjacency,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        if defer_tangent_order_evidence
            && outgoing.iter().enumerate().any(|(index, candidates)| {
                candidates.len() > 1
                    && !certified_successors
                        .get(index)
                        .copied()
                        .flatten()
                        .is_some_and(|successor| candidates.contains(&successor))
            })
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "retained-endpoint-scope",
                "tangent-order-rebuild",
            );
            endpoints.clear();
            for fragment in &self.fragments {
                let endpoints_for_fragment = match retained_endpoint_data(
                    fragment,
                    RetainedEndpointScope::TangentOrder,
                    policy,
                ) {
                    Some(Classification::Decided(endpoints)) => endpoints,
                    Some(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    None => return Classification::Uncertain(UncertaintyReason::Boundary),
                };
                endpoints.push(endpoints_for_fragment);
            }
        }

        let mut used = vec![false; self.fragments.len()];
        let mut chains = Vec::new();
        for index in 0..self.fragments.len() {
            if predecessors[index] == 0 && !used[index] {
                match follow_retained_tangent_ordered_chain(
                    index,
                    &outgoing,
                    &endpoints,
                    certified_successors,
                    filled_left_faces,
                    &mut used,
                    policy,
                ) {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
        }
        for index in 0..self.fragments.len() {
            if !used[index] {
                match follow_retained_tangent_ordered_chain(
                    index,
                    &outgoing,
                    &endpoints,
                    certified_successors,
                    filled_left_faces,
                    &mut used,
                    policy,
                ) {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
        }

        decided_arrangement_traversal(chains)
    }
}

fn validate_arrangement_fragment_provenance(
    fragments: &[BezierArrangementFragment2],
) -> CurveResult<()> {
    let policy = CurvePolicy::STRICT;
    for (index, fragment) in fragments.iter().enumerate() {
        validate_arrangement_fragment_source_range(fragment, &policy)?;
        for other in &fragments[index + 1..] {
            if fragment.source_curve_index() == other.source_curve_index()
                && fragment.source_fragment_index() == other.source_fragment_index()
            {
                validate_reused_source_fragment_ranges(fragment, other, &policy)?;
            }
        }
    }
    Ok(())
}

fn validate_arrangement_fragment_source_range(
    fragment: &BezierArrangementFragment2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match fragment.fragment() {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages {
            start,
            end,
            source_curve: Some(_),
            ..
        } => match start.cmp_by_interval(end, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {}
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "retained Bezier arrangement fragment source range must be certified strictly increasing"
                        .to_owned(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "retained Bezier arrangement fragment source range ordering is uncertain: {reason:?}"
                )));
            }
        },
        BezierSplitFragment2::AlgebraicEndpointImages {
            source_curve: None, ..
        }
        | BezierSplitFragment2::Unresolved { .. } => {}
    }

    let BezierSplitFragment2::AlgebraicEndpointImages {
        start,
        end,
        source_curve,
        start_image,
        end_image,
        ..
    } = fragment.fragment()
    else {
        return Ok(());
    };
    validate_arrangement_algebraic_endpoint_image(
        "start",
        start,
        start_image.as_ref(),
        source_curve.as_ref(),
        policy,
    )?;
    validate_arrangement_algebraic_endpoint_image(
        "end",
        end,
        end_image.as_ref(),
        source_curve.as_ref(),
        policy,
    )
}

fn validate_arrangement_algebraic_endpoint_image(
    name: &str,
    boundary: &BezierParameter2,
    image: Option<&BezierAlgebraicEndpointImage2>,
    source_curve: Option<&BezierSubcurve2>,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match (boundary, image) {
        (BezierParameter2::Exact(_), None) => Ok(()),
        (BezierParameter2::Exact(_), Some(_)) => Err(CurveError::Topology(format!(
            "exact {name} Bezier arrangement boundary must not carry algebraic endpoint image evidence"
        ))),
        (BezierParameter2::Algebraic(parameter), Some(image)) => {
            if image.parameter() != parameter {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier arrangement endpoint image parameter does not match boundary"
                )));
            }
            if !image.is_transformed() {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier arrangement endpoint image must be exact transformed evidence"
                )));
            }
            if let Some(source_curve) = source_curve {
                let expected = BezierAlgebraicEndpointImage2::from_source_curve(
                    source_curve,
                    parameter,
                    policy,
                )?;
                if !image.matches_required_source_evidence(&expected) {
                    return Err(CurveError::Topology(format!(
                        "algebraic {name} Bezier arrangement endpoint image does not match retained source curve"
                    )));
                }
            }
            Ok(())
        }
        (BezierParameter2::Algebraic(_), None) => Err(CurveError::Topology(format!(
            "algebraic {name} Bezier arrangement boundary must carry endpoint image evidence"
        ))),
    }
}

fn validate_reused_source_fragment_ranges(
    first: &BezierArrangementFragment2,
    second: &BezierArrangementFragment2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let (first_start, first_end) = arrangement_fragment_source_range(first.fragment());
    let (second_start, second_end) = arrangement_fragment_source_range(second.fragment());
    match first_end.cmp_by_interval(second_start, policy)? {
        Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {
            return Ok(());
        }
        Classification::Decided(std::cmp::Ordering::Greater) => {}
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "retained Bezier arrangement graph cannot certify reused source fragment ranges are disjoint: {reason:?}"
            )));
        }
    }
    match second_end.cmp_by_interval(first_start, policy)? {
        Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => Ok(()),
        Classification::Decided(std::cmp::Ordering::Greater) => Err(CurveError::Topology(
            "retained Bezier arrangement graph must not overlap reused source fragment evidence"
                .to_owned(),
        )),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "retained Bezier arrangement graph cannot certify reused source fragment ranges are disjoint: {reason:?}"
        ))),
    }
}

fn arrangement_fragment_source_range(
    fragment: &BezierSplitFragment2,
) -> (&BezierParameter2, &BezierParameter2) {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. }
        | BezierSplitFragment2::Unresolved { start, end } => (start, end),
    }
}

impl BezierArrangementChain2 {
    /// Constructs a traversal chain from retained fragment indices.
    pub fn new(fragment_indices: Vec<usize>, closed: bool) -> CurveResult<Self> {
        validate_arrangement_chain_indices(&fragment_indices)?;
        Ok(Self {
            fragment_indices,
            closed,
        })
    }

    /// Returns retained fragment indices in traversal order.
    pub fn fragment_indices(&self) -> &[usize] {
        &self.fragment_indices
    }

    /// Consumes the chain and returns retained fragment indices.
    pub fn into_fragment_indices(self) -> Vec<usize> {
        self.fragment_indices
    }

    /// Returns true when the chain's last endpoint equals its first endpoint.
    pub const fn is_closed(&self) -> bool {
        self.closed
    }

    /// Returns the number of fragments in the chain.
    pub fn len(&self) -> usize {
        self.fragment_indices.len()
    }

    /// Returns true when the chain contains no fragments.
    pub fn is_empty(&self) -> bool {
        self.fragment_indices.is_empty()
    }
}

impl BezierArrangementTraversal2 {
    /// Constructs a traversal result from chains.
    pub fn new(chains: Vec<BezierArrangementChain2>) -> CurveResult<Self> {
        validate_arrangement_traversal_indices(&chains)?;
        Ok(Self { chains })
    }

    /// Returns endpoint-connected chains.
    pub fn chains(&self) -> &[BezierArrangementChain2] {
        &self.chains
    }

    /// Consumes the traversal and returns chains.
    pub fn into_chains(self) -> Vec<BezierArrangementChain2> {
        self.chains
    }

    /// Returns true when no chains were produced.
    pub fn is_empty(&self) -> bool {
        self.chains.is_empty()
    }

    /// Returns the number of chains.
    pub fn len(&self) -> usize {
        self.chains.len()
    }

    /// Counts closed chains.
    pub fn closed_count(&self) -> usize {
        self.chains.iter().filter(|chain| chain.is_closed()).count()
    }
}

fn validate_arrangement_chain_indices(fragment_indices: &[usize]) -> CurveResult<()> {
    if fragment_indices.is_empty() {
        return Err(CurveError::Topology(
            "retained Bezier arrangement chain must carry at least one fragment index".to_owned(),
        ));
    }

    if fragment_indices.iter().enumerate().any(|(index, value)| {
        fragment_indices[index + 1..]
            .iter()
            .any(|candidate| candidate == value)
    }) {
        return Err(CurveError::Topology(
            "retained Bezier arrangement chain fragment indices must be unique".to_owned(),
        ));
    }
    Ok(())
}

fn validate_arrangement_traversal_indices(chains: &[BezierArrangementChain2]) -> CurveResult<()> {
    let mut indices = Vec::new();
    for chain in chains {
        validate_arrangement_chain_indices(chain.fragment_indices())?;
        indices.extend_from_slice(chain.fragment_indices());
    }
    indices.sort_unstable();
    if indices.windows(2).any(|window| window[0] == window[1]) {
        return Err(CurveError::Topology(
            "retained Bezier arrangement traversal chains must not reuse fragment indices"
                .to_owned(),
        ));
    }
    Ok(())
}

fn decided_arrangement_chain(
    fragment_indices: Vec<usize>,
    closed: bool,
) -> Classification<BezierArrangementChain2> {
    match BezierArrangementChain2::new(fragment_indices, closed) {
        Ok(chain) => Classification::Decided(chain),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn decided_arrangement_traversal(
    chains: Vec<BezierArrangementChain2>,
) -> Classification<BezierArrangementTraversal2> {
    match BezierArrangementTraversal2::new(chains) {
        Ok(traversal) => Classification::Decided(traversal),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn materialized_endpoints(fragment: &BezierSplitFragment2) -> Option<(Point2, Point2)> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => Some(curve.endpoints()),
        BezierSplitFragment2::AlgebraicEndpointImages { .. }
        | BezierSplitFragment2::Unresolved { .. } => None,
    }
}

#[derive(Clone, Debug)]
struct EndpointData {
    start: Point2,
    end: Point2,
    start_tangent: TangentVector,
    end_tangent: TangentVector,
    start_second_derivative: Option<TangentVector>,
    start_third_derivative: Option<TangentVector>,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct ExactRationalPointKey([Rational; 2]);

impl ExactRationalPointKey {
    fn from_point(point: &Point2) -> Option<Self> {
        Some(Self([
            point.x().exact_rational_ref()?.clone(),
            point.y().exact_rational_ref()?.clone(),
        ]))
    }
}

#[derive(Debug, Default)]
struct EndpointStartBuckets {
    exact: HashMap<ExactRationalPointKey, Vec<usize>>,
    unkeyed: Vec<usize>,
}

const EXACT_ENDPOINT_BUCKET_MIN_COUNT: usize = 16;

impl EndpointStartBuckets {
    fn from_points<'a>(points: impl IntoIterator<Item = &'a Point2>) -> Self {
        let mut buckets = Self::default();
        for (index, point) in points.into_iter().enumerate() {
            match ExactRationalPointKey::from_point(point) {
                Some(key) => buckets.exact.entry(key).or_default().push(index),
                None => buckets.unkeyed.push(index),
            }
        }
        buckets
    }

    fn try_for_each_candidate<E>(
        &self,
        key: Option<&ExactRationalPointKey>,
        endpoint_count: usize,
        mut visit: impl FnMut(usize) -> Result<(), E>,
    ) -> Result<(), E> {
        let Some(key) = key else {
            return (0..endpoint_count).try_for_each(visit);
        };
        self.exact
            .get(key)
            .into_iter()
            .flatten()
            .chain(&self.unkeyed)
            .copied()
            .try_for_each(&mut visit)
    }
}

fn try_for_each_endpoint_candidate<E>(
    indexed: Option<(&EndpointStartBuckets, Option<&ExactRationalPointKey>)>,
    endpoint_count: usize,
    visit: impl FnMut(usize) -> Result<(), E>,
) -> Result<(), E> {
    match indexed {
        Some((buckets, key)) => buckets.try_for_each_candidate(key, endpoint_count, visit),
        None => (0..endpoint_count).try_for_each(visit),
    }
}

#[derive(Debug, Default)]
struct RetainedEndpointStartIndex {
    by_vertex: HashMap<usize, Vec<usize>>,
    with_vertex_exact: HashMap<ExactRationalPointKey, Vec<usize>>,
    with_vertex_unkeyed: Vec<usize>,
    without_vertex_exact: HashMap<ExactRationalPointKey, Vec<usize>>,
    without_vertex_unkeyed: Vec<usize>,
}

impl RetainedEndpointStartIndex {
    fn new(endpoints: &[RetainedEndpointData]) -> Self {
        let mut index = Self::default();
        for (endpoint_index, endpoint) in endpoints.iter().enumerate() {
            if let Some(vertex) = endpoint.start_topology_vertex {
                index
                    .by_vertex
                    .entry(vertex)
                    .or_default()
                    .push(endpoint_index);
            }
            match endpoint
                .start
                .as_ref()
                .and_then(exact_retained_endpoint_key)
            {
                Some(key) => {
                    if endpoint.start_topology_vertex.is_some() {
                        index
                            .with_vertex_exact
                            .entry(key)
                            .or_default()
                            .push(endpoint_index);
                    } else {
                        index
                            .without_vertex_exact
                            .entry(key)
                            .or_default()
                            .push(endpoint_index);
                    }
                }
                None => {
                    if endpoint.start_topology_vertex.is_some() {
                        index.with_vertex_unkeyed.push(endpoint_index);
                    } else {
                        index.without_vertex_unkeyed.push(endpoint_index);
                    }
                }
            }
        }
        index
    }

    fn try_for_each_candidate<E>(
        &self,
        vertex: Option<usize>,
        key: Option<&ExactRationalPointKey>,
        endpoint_count: usize,
        mut visit: impl FnMut(usize) -> Result<(), E>,
    ) -> Result<(), E> {
        if let Some(vertex) = vertex {
            let Some(key) = key else {
                return self
                    .by_vertex
                    .get(&vertex)
                    .into_iter()
                    .flatten()
                    .copied()
                    .try_for_each(visit);
            };
            self.by_vertex
                .get(&vertex)
                .into_iter()
                .flatten()
                .copied()
                .try_for_each(&mut visit)?;
            return self
                .without_vertex_exact
                .get(key)
                .into_iter()
                .flatten()
                .chain(&self.without_vertex_unkeyed)
                .copied()
                .try_for_each(visit);
        }
        let Some(key) = key else {
            return (0..endpoint_count).try_for_each(visit);
        };
        self.with_vertex_exact
            .get(key)
            .into_iter()
            .flatten()
            .chain(self.without_vertex_exact.get(key).into_iter().flatten())
            .chain(&self.with_vertex_unkeyed)
            .chain(&self.without_vertex_unkeyed)
            .copied()
            .try_for_each(visit)
    }
}

fn exact_retained_endpoint_key(endpoint: &RetainedEndpointKey) -> Option<ExactRationalPointKey> {
    match endpoint {
        RetainedEndpointKey::Exact(point) => ExactRationalPointKey::from_point(point),
        RetainedEndpointKey::Algebraic { .. } => None,
    }
}

#[derive(Clone, Debug)]
struct TangentVector {
    dx: Real,
    dy: Real,
}

#[derive(Clone, Debug)]
struct RetainedEndpointData {
    start: Option<RetainedEndpointKey>,
    end: Option<RetainedEndpointKey>,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
    start_tangent: Option<RetainedTangentVector>,
    end_tangent: Option<RetainedTangentVector>,
    start_second_derivative: Option<RetainedTangentVector>,
    start_third_derivative: Option<RetainedTangentVector>,
    start_derivative_source: Option<RetainedAlgebraicDerivativeSource>,
    end_derivative_source: Option<RetainedAlgebraicDerivativeSource>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RetainedEndpointScope {
    Connectivity,
    TangentOrder,
}

#[derive(Clone, Debug, PartialEq)]
enum RetainedEndpointKey {
    Exact(Box<Point2>),
    Algebraic {
        x: Box<AlgebraicRootRepresentation>,
        y: Box<AlgebraicRootRepresentation>,
    },
}

#[derive(Clone, Debug)]
enum RetainedTangentVector {
    Native(Box<TangentVector>),
    Algebraic(Box<BezierAlgebraicTangentVector2>),
}

#[derive(Clone, Debug)]
struct RetainedEndpointSideData {
    point: Option<RetainedEndpointKey>,
    tangent: Option<RetainedTangentVector>,
    second_derivative: Option<RetainedTangentVector>,
    third_derivative: Option<RetainedTangentVector>,
    derivative_source: Option<RetainedAlgebraicDerivativeSource>,
}

#[derive(Clone, Debug)]
struct RetainedAlgebraicDerivativeSource {
    curve: Box<BezierSubcurve2>,
    parameter: crate::BezierAlgebraicParameter2,
    reversed: bool,
}

fn materialized_endpoint_data(
    fragment: &BezierSplitFragment2,
    policy: &CurvePolicy,
) -> Option<Classification<EndpointData>> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => Some(curve.endpoint_data(policy)),
        BezierSplitFragment2::AlgebraicEndpointImages { .. }
        | BezierSplitFragment2::Unresolved { .. } => None,
    }
}

fn retained_topology_endpoint_data(
    arrangement_fragment: &BezierArrangementFragment2,
) -> RetainedEndpointData {
    RetainedEndpointData {
        start: None,
        end: None,
        start_topology_vertex: arrangement_fragment.start_topology_vertex(),
        end_topology_vertex: arrangement_fragment.end_topology_vertex(),
        start_tangent: None,
        end_tangent: None,
        start_second_derivative: None,
        start_third_derivative: None,
        start_derivative_source: None,
        end_derivative_source: None,
    }
}

fn retained_endpoint_data(
    arrangement_fragment: &BezierArrangementFragment2,
    scope: RetainedEndpointScope,
    policy: &CurvePolicy,
) -> Option<Classification<RetainedEndpointData>> {
    let fragment = arrangement_fragment.fragment();
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. }
            if scope == RetainedEndpointScope::Connectivity =>
        {
            let (start, end) = curve.endpoints();
            Some(Classification::Decided(RetainedEndpointData {
                start: Some(RetainedEndpointKey::Exact(Box::new(start))),
                end: Some(RetainedEndpointKey::Exact(Box::new(end))),
                start_topology_vertex: arrangement_fragment.start_topology_vertex(),
                end_topology_vertex: arrangement_fragment.end_topology_vertex(),
                start_tangent: None,
                end_tangent: None,
                start_second_derivative: None,
                start_third_derivative: None,
                start_derivative_source: None,
                end_derivative_source: None,
            }))
        }
        BezierSplitFragment2::Materialized { curve, .. } => match curve.endpoint_data(policy) {
            Classification::Decided(data) => Some(Classification::Decided(RetainedEndpointData {
                start: Some(RetainedEndpointKey::Exact(Box::new(data.start))),
                end: Some(RetainedEndpointKey::Exact(Box::new(data.end))),
                start_topology_vertex: arrangement_fragment.start_topology_vertex(),
                end_topology_vertex: arrangement_fragment.end_topology_vertex(),
                start_tangent: Some(RetainedTangentVector::Native(Box::new(data.start_tangent))),
                end_tangent: Some(RetainedTangentVector::Native(Box::new(data.end_tangent))),
                start_second_derivative: data
                    .start_second_derivative
                    .map(Box::new)
                    .map(RetainedTangentVector::Native),
                start_third_derivative: data
                    .start_third_derivative
                    .map(Box::new)
                    .map(RetainedTangentVector::Native),
                start_derivative_source: None,
                end_derivative_source: None,
            })),
            Classification::Uncertain(reason) => Some(Classification::Uncertain(reason)),
        },
        BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start,
            end,
            source_curve,
            start_image,
            end_image,
        } => {
            let source_start_topology_vertex = if *reversed {
                arrangement_fragment.end_topology_vertex()
            } else {
                arrangement_fragment.start_topology_vertex()
            };
            let source_end_topology_vertex = if *reversed {
                arrangement_fragment.start_topology_vertex()
            } else {
                arrangement_fragment.end_topology_vertex()
            };
            let source_start = match retained_endpoint_side_data(
                start,
                start_image.as_ref(),
                source_curve.as_ref(),
                source_start_topology_vertex,
                scope,
                policy,
            ) {
                Classification::Decided(data) => data,
                Classification::Uncertain(reason) => {
                    return Some(Classification::Uncertain(reason));
                }
            };
            let source_end = match retained_endpoint_side_data(
                end,
                end_image.as_ref(),
                source_curve.as_ref(),
                source_end_topology_vertex,
                scope,
                policy,
            ) {
                Classification::Decided(data) => data,
                Classification::Uncertain(reason) => {
                    return Some(Classification::Uncertain(reason));
                }
            };
            let (start, end) = if *reversed {
                let start = match reverse_retained_endpoint_side_option(source_end) {
                    Some(data) => data,
                    None => return Some(Classification::Uncertain(UncertaintyReason::Boundary)),
                };
                let end = match reverse_retained_endpoint_side_option(source_start) {
                    Some(data) => data,
                    None => return Some(Classification::Uncertain(UncertaintyReason::Boundary)),
                };
                (start, end)
            } else {
                (source_start, source_end)
            };
            let (
                start,
                start_tangent,
                start_second_derivative,
                start_third_derivative,
                start_derivative_source,
            ) = retained_endpoint_side_parts(start);
            let (end, end_tangent, _, _, end_derivative_source) = retained_endpoint_side_parts(end);
            Some(Classification::Decided(RetainedEndpointData {
                start,
                end,
                start_topology_vertex: arrangement_fragment.start_topology_vertex(),
                end_topology_vertex: arrangement_fragment.end_topology_vertex(),
                start_tangent,
                end_tangent,
                start_second_derivative,
                start_third_derivative,
                start_derivative_source,
                end_derivative_source,
            }))
        }
        BezierSplitFragment2::Unresolved { .. } => None,
    }
}

fn retained_endpoint_side_data(
    parameter: &BezierParameter2,
    image: Option<&BezierAlgebraicEndpointImage2>,
    source_curve: Option<&BezierSubcurve2>,
    topology_vertex: Option<usize>,
    scope: RetainedEndpointScope,
    policy: &CurvePolicy,
) -> Classification<Option<RetainedEndpointSideData>> {
    if let Some(image) = image {
        let derivative_source = (scope == RetainedEndpointScope::TangentOrder)
            .then(|| retained_algebraic_derivative_source(source_curve, image.parameter()))
            .flatten();
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "retained-endpoint-image",
            match (image.is_lazy_first_order(), topology_vertex.is_some()) {
                (true, true) => "lazy-topology-deferred",
                (true, false) => "lazy-unkeyed-resolved",
                (false, true) => "eager-topology-resolved",
                (false, false) => "eager-unkeyed-resolved",
            },
        );
        if topology_vertex.is_some() && image.is_lazy_first_order() && source_curve.is_some() {
            return Classification::Decided(Some(RetainedEndpointSideData {
                point: None,
                tangent: None,
                second_derivative: None,
                third_derivative: None,
                derivative_source,
            }));
        }
        let Ok(point_image) = image.try_point() else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let Some(point) = retained_algebraic_point_key(point_image) else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        if scope == RetainedEndpointScope::Connectivity {
            return Classification::Decided(Some(RetainedEndpointSideData {
                point: Some(point),
                tangent: None,
                second_derivative: None,
                third_derivative: None,
                derivative_source: None,
            }));
        }
        let Ok(tangent_image) = image.try_tangent() else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let Some(tangent) = retained_algebraic_tangent(tangent_image) else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let second_derivative = match image.second_derivative() {
            Some(image) => match retained_algebraic_tangent(image) {
                Some(tangent) => Some(tangent),
                None => return Classification::Uncertain(UncertaintyReason::Boundary),
            },
            None => None,
        };
        let third_derivative = match image.third_derivative() {
            Some(image) => match retained_algebraic_tangent(image) {
                Some(tangent) => Some(tangent),
                None => return Classification::Uncertain(UncertaintyReason::Boundary),
            },
            None => None,
        };
        return Classification::Decided(Some(RetainedEndpointSideData {
            point: Some(point),
            tangent: Some(tangent),
            second_derivative,
            third_derivative,
            derivative_source,
        }));
    }

    let (BezierParameter2::Exact(parameter), Some(source_curve)) = (parameter, source_curve) else {
        return Classification::Decided(None);
    };
    if scope == RetainedEndpointScope::Connectivity {
        return source_curve.point_at(parameter, policy).map(|point| {
            Some(RetainedEndpointSideData {
                point: Some(RetainedEndpointKey::Exact(Box::new(point))),
                tangent: None,
                second_derivative: None,
                third_derivative: None,
                derivative_source: None,
            })
        });
    }
    retained_exact_source_endpoint_side_data(source_curve, parameter, true, policy).map(Some)
}

fn retained_exact_source_endpoint_side_data(
    source_curve: &BezierSubcurve2,
    parameter: &Real,
    include_higher_derivatives: bool,
    policy: &CurvePolicy,
) -> Classification<RetainedEndpointSideData> {
    let at_source_end = match compare_reals(parameter, &Real::one(), policy) {
        Some(ordering) => ordering == std::cmp::Ordering::Equal,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let (data, restore_source_orientation) = if at_source_end {
        match source_curve
            .reversed()
            .endpoint_data_with_higher_derivatives(policy, include_higher_derivatives)
        {
            Classification::Decided(data) => (data, true),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    } else {
        let subcurve = match source_curve.subcurve_between_exact(parameter, &Real::one(), policy) {
            Ok(Classification::Decided(subcurve)) => subcurve,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Boundary),
        };
        match subcurve.endpoint_data_with_higher_derivatives(policy, include_higher_derivatives) {
            Classification::Decided(data) => (data, false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    };
    let side = RetainedEndpointSideData {
        point: Some(RetainedEndpointKey::Exact(Box::new(data.start))),
        tangent: Some(RetainedTangentVector::Native(Box::new(data.start_tangent))),
        second_derivative: data
            .start_second_derivative
            .map(Box::new)
            .map(RetainedTangentVector::Native),
        third_derivative: data
            .start_third_derivative
            .map(Box::new)
            .map(RetainedTangentVector::Native),
        derivative_source: None,
    };
    if restore_source_orientation {
        match reversed_retained_endpoint_side(side) {
            Some(side) => Classification::Decided(side),
            None => Classification::Uncertain(UncertaintyReason::Boundary),
        }
    } else {
        Classification::Decided(side)
    }
}

fn retained_endpoint_side_parts(
    side: Option<RetainedEndpointSideData>,
) -> (
    Option<RetainedEndpointKey>,
    Option<RetainedTangentVector>,
    Option<RetainedTangentVector>,
    Option<RetainedTangentVector>,
    Option<RetainedAlgebraicDerivativeSource>,
) {
    match side {
        Some(side) => (
            side.point,
            side.tangent,
            side.second_derivative,
            side.third_derivative,
            side.derivative_source,
        ),
        None => (None, None, None, None, None),
    }
}

fn reverse_retained_endpoint_side_option(
    side: Option<RetainedEndpointSideData>,
) -> Option<Option<RetainedEndpointSideData>> {
    match side {
        Some(side) => Some(Some(reversed_retained_endpoint_side(side)?)),
        None => Some(None),
    }
}

fn reversed_retained_endpoint_side(
    mut side: RetainedEndpointSideData,
) -> Option<RetainedEndpointSideData> {
    side.tangent = match side.tangent {
        Some(tangent) => Some(negate_retained_tangent(tangent)?),
        None => None,
    };
    side.third_derivative = match side.third_derivative {
        Some(derivative) => Some(negate_retained_tangent(derivative)?),
        None => None,
    };
    if let Some(source) = &mut side.derivative_source {
        source.reversed = !source.reversed;
    }
    Some(side)
}

fn retained_algebraic_derivative_source(
    source_curve: Option<&BezierSubcurve2>,
    parameter: &crate::BezierAlgebraicParameter2,
) -> Option<RetainedAlgebraicDerivativeSource> {
    let source_curve = source_curve?;
    Some(RetainedAlgebraicDerivativeSource {
        curve: Box::new(source_curve.clone()),
        parameter: parameter.clone(),
        reversed: false,
    })
}

fn retained_algebraic_point_key(point: &BezierEndpointPointImage2) -> Option<RetainedEndpointKey> {
    let (x, y) = match point {
        BezierEndpointPointImage2::Polynomial(point) => (
            point.x()?.representation()?.clone(),
            point.y()?.representation()?.clone(),
        ),
        BezierEndpointPointImage2::Rational(point) => (
            point.x()?.representation()?.clone(),
            point.y()?.representation()?.clone(),
        ),
    };
    Some(RetainedEndpointKey::Algebraic {
        x: Box::new(x),
        y: Box::new(y),
    })
}

fn retained_algebraic_tangent(
    tangent: &BezierEndpointTangentImage2,
) -> Option<RetainedTangentVector> {
    BezierAlgebraicTangentVector2::from_endpoint_image(tangent)
        .vector
        .map(Box::new)
        .map(RetainedTangentVector::Algebraic)
}

fn negate_retained_tangent(tangent: RetainedTangentVector) -> Option<RetainedTangentVector> {
    match tangent {
        RetainedTangentVector::Native(tangent) => {
            let TangentVector { dx, dy } = *tangent;
            Some(RetainedTangentVector::Native(Box::new(TangentVector {
                dx: -dx,
                dy: -dy,
            })))
        }
        RetainedTangentVector::Algebraic(tangent) => Some(RetainedTangentVector::Algebraic(
            Box::new(BezierAlgebraicTangentVector2::new(
                negate_algebraic_root(tangent.dx())?,
                negate_algebraic_root(tangent.dy())?,
            )),
        )),
    }
}

fn negate_algebraic_root(
    value: &AlgebraicRootRepresentation,
) -> Option<AlgebraicRootRepresentation> {
    let evidence =
        arithmetic_algebraic_root_representations(value, None, AlgebraicRootArithmeticOp::Negate);
    if !matches!(
        evidence.status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
    ) {
        return None;
    }
    if let Some(result) = evidence.result_representation {
        return Some(result);
    }
    evidence
        .exact_result
        .map(|value| exact_value_representation(&value))
}

fn exact_value_representation(value: &Real) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: hypersolve::SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![-value.clone(), Real::one()],
        interval: hypersolve::IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value.clone()),
            distinct_root_count: 1,
        },
        kind: hypersolve::AlgebraicRootKind::ExactRationalWitness,
        validation: hypersolve::AlgebraicRootValidationReport {
            status: hypersolve::AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

type EndpointAdjacency = (Vec<Option<usize>>, Vec<Option<usize>>);

fn endpoint_adjacency(
    endpoints: &[(Point2, Point2)],
    policy: &CurvePolicy,
) -> Classification<EndpointAdjacency> {
    let mut successors = vec![None; endpoints.len()];
    let mut predecessors = vec![None; endpoints.len()];
    let indexed = (endpoints.len() >= EXACT_ENDPOINT_BUCKET_MIN_COUNT)
        .then(|| EndpointStartBuckets::from_points(endpoints.iter().map(|(start, _)| start)));

    for (left_index, (_, left_end)) in endpoints.iter().enumerate() {
        let left_key = indexed
            .as_ref()
            .and_then(|_| ExactRationalPointKey::from_point(left_end));
        let result = try_for_each_endpoint_candidate(
            indexed.as_ref().map(|buckets| (buckets, left_key.as_ref())),
            endpoints.len(),
            |right_index| {
                if left_index == right_index {
                    return Ok(());
                }
                match points_equal(left_end, &endpoints[right_index].0, policy) {
                    Some(true) => {
                        if successors[left_index].replace(right_index).is_some()
                            || predecessors[right_index].replace(left_index).is_some()
                        {
                            return Err(UncertaintyReason::Boundary);
                        }
                    }
                    Some(false) => {}
                    None => return Err(UncertaintyReason::RealSign),
                }
                Ok(())
            },
        );
        if let Err(reason) = result {
            return Classification::Uncertain(reason);
        }
    }

    Classification::Decided((successors, predecessors))
}

fn follow_chain(
    start: usize,
    successors: &[Option<usize>],
    endpoints: &[(Point2, Point2)],
    used: &mut [bool],
    policy: &CurvePolicy,
) -> Classification<BezierArrangementChain2> {
    let first_start = endpoints[start].0.clone();
    let mut current = start;
    let mut indices = Vec::new();

    loop {
        if used[current] {
            break;
        }
        used[current] = true;
        indices.push(current);

        let Some(next) = successors[current] else {
            let closed = match points_equal(&endpoints[current].1, &first_start, policy) {
                Some(value) => value,
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            };
            return decided_arrangement_chain(indices, closed);
        };
        current = next;
        if current == start {
            return decided_arrangement_chain(indices, true);
        }
    }

    Classification::Uncertain(UncertaintyReason::Boundary)
}

type TangentAdjacency = (Vec<Vec<usize>>, Vec<usize>);

fn tangent_adjacency(
    endpoints: &[EndpointData],
    policy: &CurvePolicy,
) -> Classification<TangentAdjacency> {
    let mut outgoing = vec![Vec::new(); endpoints.len()];
    let mut predecessors = vec![0_usize; endpoints.len()];
    let indexed = (endpoints.len() >= EXACT_ENDPOINT_BUCKET_MIN_COUNT).then(|| {
        EndpointStartBuckets::from_points(endpoints.iter().map(|endpoint| &endpoint.start))
    });

    for (left_index, left) in endpoints.iter().enumerate() {
        let left_key = indexed
            .as_ref()
            .and_then(|_| ExactRationalPointKey::from_point(&left.end));
        let result = try_for_each_endpoint_candidate(
            indexed.as_ref().map(|buckets| (buckets, left_key.as_ref())),
            endpoints.len(),
            |right_index| {
                if left_index == right_index {
                    return Ok(());
                }
                match points_equal(&left.end, &endpoints[right_index].start, policy) {
                    Some(true) => {
                        outgoing[left_index].push(right_index);
                        predecessors[right_index] += 1;
                    }
                    Some(false) => {}
                    None => return Err(UncertaintyReason::RealSign),
                }
                Ok(())
            },
        );
        if let Err(reason) = result {
            return Classification::Uncertain(reason);
        }
    }
    Classification::Decided((outgoing, predecessors))
}

fn retained_tangent_adjacency(
    endpoints: &[RetainedEndpointData],
    policy: &CurvePolicy,
) -> Classification<TangentAdjacency> {
    let mut outgoing = vec![Vec::new(); endpoints.len()];
    let mut predecessors = vec![0_usize; endpoints.len()];
    let start_index = (endpoints.len() >= EXACT_ENDPOINT_BUCKET_MIN_COUNT)
        .then(|| RetainedEndpointStartIndex::new(endpoints));
    for (left_index, left) in endpoints.iter().enumerate() {
        if left.end.is_none() && left.end_topology_vertex.is_none() {
            continue;
        }
        let left_key = left.end.as_ref().and_then(exact_retained_endpoint_key);
        let mut visit = |right_index: usize| {
            if left_index == right_index {
                return Ok(());
            }
            let right = &endpoints[right_index];
            if right.start.is_none() && right.start_topology_vertex.is_none() {
                return Ok(());
            }
            match retained_endpoints_equal(
                left.end_topology_vertex,
                left.end.as_ref(),
                right.start_topology_vertex,
                right.start.as_ref(),
                policy,
            ) {
                Some(true) => {
                    outgoing[left_index].push(right_index);
                    predecessors[right_index] += 1;
                }
                Some(false) => {}
                None => return Err(UncertaintyReason::RealSign),
            }
            Ok(())
        };
        let result = match &start_index {
            Some(index) => index.try_for_each_candidate(
                left.end_topology_vertex,
                left_key.as_ref(),
                endpoints.len(),
                &mut visit,
            ),
            None => (0..endpoints.len()).try_for_each(visit),
        };
        if let Err(reason) = result {
            return Classification::Uncertain(reason);
        }
    }
    Classification::Decided((outgoing, predecessors))
}

#[cfg(test)]
mod endpoint_adjacency_tests {
    use super::*;

    fn point(x: i32) -> Point2 {
        Point2::new(Real::from(x), Real::zero())
    }

    fn endpoint(index: i32) -> RetainedEndpointData {
        RetainedEndpointData {
            start: Some(RetainedEndpointKey::Exact(Box::new(point(
                1_000 + index * 3,
            )))),
            end: Some(RetainedEndpointKey::Exact(Box::new(point(
                1_001 + index * 3,
            )))),
            start_topology_vertex: None,
            end_topology_vertex: None,
            start_tangent: None,
            end_tangent: None,
            start_second_derivative: None,
            start_third_derivative: None,
            start_derivative_source: None,
            end_derivative_source: None,
        }
    }

    #[cfg(feature = "predicates")]
    fn sqrt_half_parameter() -> crate::BezierAlgebraicParameter2 {
        let polynomial = match crate::BezierParameterPolynomial::try_new_power_basis(
            vec![Real::from(-1), Real::zero(), Real::from(2)],
            &CurvePolicy::STRICT,
        )
        .expect("valid parameter polynomial")
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                panic!("parameter polynomial unexpectedly uncertain: {reason:?}")
            }
        };
        let interval = match crate::BezierParameterInterval::try_new(
            Real::from(Rational::fraction(2, 3).expect("nonzero denominator")),
            Real::from(Rational::fraction(3, 4).expect("nonzero denominator")),
            &CurvePolicy::STRICT,
        )
        .expect("valid parameter interval")
        {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                panic!("parameter interval unexpectedly uncertain: {reason:?}")
            }
        };
        match crate::BezierAlgebraicParameter2::try_isolate(
            polynomial,
            interval,
            &CurvePolicy::STRICT,
        )
        .expect("isolated parameter")
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                panic!("parameter isolation unexpectedly uncertain: {reason:?}")
            }
        }
    }

    #[test]
    #[cfg(feature = "predicates")]
    fn lazy_polynomial_endpoint_derivatives_match_eager_images() {
        let policy = CurvePolicy::STRICT;
        let parameter = sqrt_half_parameter();
        let curve = crate::CubicBezier2::new(
            point(0),
            Point2::new(Real::from(1), Real::from(2)),
            Point2::new(Real::from(3), Real::from(-1)),
            point(4),
        );
        let lazy =
            BezierAlgebraicEndpointImage2::cubic_first_order(&curve, &parameter, &policy).unwrap();
        let eager = BezierAlgebraicEndpointImage2::cubic(&curve, &parameter, &policy).unwrap();

        assert!(lazy.is_lazy_first_order());
        assert!(lazy.second_derivative().is_none());
        assert!(lazy.third_derivative().is_none());
        assert_eq!(lazy.try_point(), eager.try_point());
        assert_eq!(lazy.try_tangent(), eager.try_tangent());

        let source =
            retained_algebraic_derivative_source(Some(&BezierSubcurve2::Cubic(curve)), &parameter)
                .expect("cubic derivative source");
        for (order, expected) in [
            (1, eager.try_tangent().ok()),
            (2, eager.second_derivative()),
            (3, eager.third_derivative()),
        ] {
            let Classification::Decided(Some(actual)) =
                retained_algebraic_derivative(None, Some(&source), order, &policy)
            else {
                panic!("lazy derivative {order} should construct exactly");
            };
            let expected = expected.and_then(retained_algebraic_tangent);
            assert_eq!(
                retained_algebraic_vector(Some(&actual)),
                retained_algebraic_vector(expected.as_ref())
            );
        }
    }

    #[test]
    fn retained_index_combines_topology_vertices_and_exact_coordinate_fallback() {
        let mut endpoints = (0..16).map(endpoint).collect::<Vec<_>>();
        endpoints[0].end = Some(RetainedEndpointKey::Exact(Box::new(point(10))));
        endpoints[0].end_topology_vertex = Some(7);
        endpoints[1].start = Some(RetainedEndpointKey::Exact(Box::new(point(999))));
        endpoints[1].start_topology_vertex = Some(7);
        endpoints[2].start = Some(RetainedEndpointKey::Exact(Box::new(point(10))));
        endpoints[3].end = Some(RetainedEndpointKey::Exact(Box::new(point(20))));
        endpoints[4].start = Some(RetainedEndpointKey::Exact(Box::new(point(20))));
        endpoints[4].start_topology_vertex = Some(9);

        let Classification::Decided((outgoing, predecessors)) =
            retained_tangent_adjacency(&endpoints, &CurvePolicy::STRICT)
        else {
            panic!("indexed retained adjacency should remain exact");
        };
        assert_eq!(outgoing[0], [1, 2]);
        assert_eq!(outgoing[3], [4]);
        assert_eq!(predecessors[1], 1);
        assert_eq!(predecessors[2], 1);
        assert_eq!(predecessors[4], 1);
    }

    #[test]
    fn partial_successor_evidence_rebuilds_higher_order_tangents() {
        let point2 = |x, y| Point2::new(Real::from(x), Real::from(y));
        let fragment = |source_curve_index, curve| {
            BezierArrangementFragment2::new(
                source_curve_index,
                0,
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(curve),
                },
            )
        };
        let graph = BezierArrangementGraph2::new(vec![
            fragment(
                0,
                crate::QuadraticBezier2::new(point2(0, 0), point2(1, 0), point2(2, 0)),
            ),
            fragment(
                1,
                crate::QuadraticBezier2::new(point2(2, 0), point2(3, 1), point2(4, 0)),
            ),
            fragment(
                2,
                crate::QuadraticBezier2::new(point2(2, 0), point2(4, 2), point2(5, 0)),
            ),
        ])
        .expect("valid branch graph");
        let policy = CurvePolicy::STRICT;

        assert_eq!(
            graph.traverse_retained_with_certified_successors(&[None], &policy),
            graph.traverse_retained_with_tangent_order(&policy)
        );
    }

    #[test]
    fn certified_branch_free_traversal_defers_unused_zero_tangent() {
        let point2 = |x, y| Point2::new(Real::from(x), Real::from(y));
        let start = point2(0, 0);
        let graph = BezierArrangementGraph2::new(vec![BezierArrangementFragment2::new(
            0,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(crate::QuadraticBezier2::new(
                    start.clone(),
                    start,
                    point2(2, 0),
                )),
            },
        )])
        .expect("valid branch-free graph");
        let policy = CurvePolicy::STRICT;

        assert!(matches!(
            graph.traverse_retained_with_tangent_order(&policy),
            Classification::Uncertain(UncertaintyReason::RealSign)
        ));
        assert!(matches!(
            graph.traverse_retained_with_certified_successors(&[None], &policy),
            Classification::Decided(_)
        ));
    }

    #[test]
    fn complete_topology_traversal_defers_unused_endpoint_coordinates() {
        let point2 = |x, y| Point2::new(Real::from(x), Real::from(y));
        let fragment = |source_curve_index: usize,
                        start_vertex: usize,
                        end_vertex: usize,
                        start: Point2,
                        end: Point2| {
            BezierArrangementFragment2::new(
                source_curve_index,
                0,
                BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(crate::QuadraticBezier2::new(
                        start.clone(),
                        start,
                        end,
                    )),
                },
            )
            .with_topology_vertices(Some(start_vertex), Some(end_vertex))
        };
        let first = fragment(0, 0, 1, point2(0, 0), point2(1, 0));
        let second = fragment(1, 1, 2, point2(100, 0), point2(101, 0));
        let policy = CurvePolicy::STRICT;
        let first_endpoints = retained_topology_endpoint_data(&first);

        assert!(first_endpoints.start.is_none());
        assert!(first_endpoints.end.is_none());

        let graph =
            BezierArrangementGraph2::new(vec![first, second]).expect("valid topology-only graph");
        let Classification::Decided(traversal) =
            graph.traverse_retained_with_certified_successors(&[Some(1), None], &policy)
        else {
            panic!("certified topology-only traversal should be decided");
        };
        assert_eq!(traversal.chains()[0].fragment_indices(), [0, 1]);
    }
}

fn follow_retained_tangent_ordered_chain(
    start: usize,
    outgoing: &[Vec<usize>],
    endpoints: &[RetainedEndpointData],
    certified_successors: &[Option<usize>],
    filled_left_faces: bool,
    used: &mut [bool],
    policy: &CurvePolicy,
) -> Classification<BezierArrangementChain2> {
    let first_start = endpoints[start].start.clone();
    let first_start_topology_vertex = endpoints[start].start_topology_vertex;
    let mut current = start;
    let mut indices = Vec::new();

    loop {
        if used[current] {
            break;
        }
        used[current] = true;
        indices.push(current);

        let next = match choose_retained_tangent_successor(
            current,
            &outgoing[current],
            endpoints,
            certified_successors.get(current).copied().flatten(),
            filled_left_faces,
            policy,
        ) {
            Classification::Decided(next) => next,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let Some(next) = next else {
            let closed = match retained_endpoints_equal(
                endpoints[current].end_topology_vertex,
                endpoints[current].end.as_ref(),
                first_start_topology_vertex,
                first_start.as_ref(),
                policy,
            ) {
                Some(value) => value,
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            };
            return decided_arrangement_chain(indices, closed);
        };

        current = next;
        if current == start {
            return decided_arrangement_chain(indices, true);
        }
    }

    Classification::Uncertain(UncertaintyReason::Boundary)
}

fn choose_retained_tangent_successor(
    current: usize,
    candidates: &[usize],
    endpoints: &[RetainedEndpointData],
    certified_successor: Option<usize>,
    filled_left_faces: bool,
    policy: &CurvePolicy,
) -> Classification<Option<usize>> {
    if candidates.is_empty() {
        return Classification::Decided(None);
    }
    if candidates.len() == 1 {
        return Classification::Decided(Some(candidates[0]));
    }
    if let Some(successor) = certified_successor
        && candidates.contains(&successor)
    {
        return Classification::Decided(Some(successor));
    }

    let base = match retained_algebraic_derivative(
        endpoints[current].end_tangent.as_ref(),
        endpoints[current].end_derivative_source.as_ref(),
        1,
        policy,
    ) {
        Classification::Decided(Some(tangent)) => tangent,
        Classification::Decided(None) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let mut best = candidates[0];

    for candidate in candidates.iter().copied().skip(1) {
        let first = match retained_algebraic_derivative(
            endpoints[candidate].start_tangent.as_ref(),
            endpoints[candidate].start_derivative_source.as_ref(),
            1,
            policy,
        ) {
            Classification::Decided(Some(tangent)) => tangent,
            Classification::Decided(None) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let second = match retained_algebraic_derivative(
            endpoints[best].start_tangent.as_ref(),
            endpoints[best].start_derivative_source.as_ref(),
            1,
            policy,
        ) {
            Classification::Decided(Some(tangent)) => tangent,
            Classification::Decided(None) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        match compare_retained_turn_from_base(&base, &first, &second, filled_left_faces, policy) {
            Classification::Decided(TurnOrdering::FirstBeforeSecond) => best = candidate,
            Classification::Decided(TurnOrdering::SecondBeforeFirst) => {}
            Classification::Decided(TurnOrdering::SameDirection) => {
                let ordering = compare_retained_same_tangent_second_order(
                    &endpoints[candidate],
                    &endpoints[best],
                    policy,
                );
                match if filled_left_faces {
                    reverse_turn_ordering(ordering)
                } else {
                    ordering
                } {
                    Classification::Decided(TurnOrdering::FirstBeforeSecond) => best = candidate,
                    Classification::Decided(TurnOrdering::SecondBeforeFirst) => {}
                    Classification::Decided(TurnOrdering::SameDirection) => {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(Some(best))
}

fn follow_tangent_ordered_chain(
    start: usize,
    outgoing: &[Vec<usize>],
    endpoints: &[EndpointData],
    used: &mut [bool],
    policy: &CurvePolicy,
) -> Classification<BezierArrangementChain2> {
    let first_start = endpoints[start].start.clone();
    let mut current = start;
    let mut indices = Vec::new();

    loop {
        if used[current] {
            break;
        }
        used[current] = true;
        indices.push(current);

        let next = match choose_tangent_successor(current, &outgoing[current], endpoints, policy) {
            Classification::Decided(next) => next,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let Some(next) = next else {
            let closed = match points_equal(&endpoints[current].end, &first_start, policy) {
                Some(value) => value,
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            };
            return decided_arrangement_chain(indices, closed);
        };

        current = next;
        if current == start {
            return decided_arrangement_chain(indices, true);
        }
    }

    Classification::Uncertain(UncertaintyReason::Boundary)
}

fn choose_tangent_successor(
    current: usize,
    candidates: &[usize],
    endpoints: &[EndpointData],
    policy: &CurvePolicy,
) -> Classification<Option<usize>> {
    if candidates.is_empty() {
        return Classification::Decided(None);
    }
    if candidates.len() == 1 {
        return Classification::Decided(Some(candidates[0]));
    }

    let base = &endpoints[current].end_tangent;
    if !base.is_nonzero(policy) {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    }

    let mut best = candidates[0];
    for candidate in candidates {
        if !endpoints[*candidate].start_tangent.is_nonzero(policy) {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }
    }

    for candidate in candidates.iter().copied().skip(1) {
        match compare_turn_from_base(
            base,
            &endpoints[candidate].start_tangent,
            &endpoints[best].start_tangent,
            policy,
        ) {
            Classification::Decided(TurnOrdering::FirstBeforeSecond) => best = candidate,
            Classification::Decided(TurnOrdering::SecondBeforeFirst) => {}
            Classification::Decided(TurnOrdering::SameDirection) => {
                match compare_same_tangent_second_order(
                    &endpoints[candidate].start_tangent,
                    endpoints[candidate].start_second_derivative.as_ref(),
                    endpoints[candidate].start_third_derivative.as_ref(),
                    &endpoints[best].start_tangent,
                    endpoints[best].start_second_derivative.as_ref(),
                    endpoints[best].start_third_derivative.as_ref(),
                    policy,
                ) {
                    Classification::Decided(TurnOrdering::FirstBeforeSecond) => best = candidate,
                    Classification::Decided(TurnOrdering::SecondBeforeFirst) => {}
                    Classification::Decided(TurnOrdering::SameDirection) => {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(Some(best))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TurnOrdering {
    FirstBeforeSecond,
    SecondBeforeFirst,
    SameDirection,
}

fn reverse_turn_ordering(ordering: Classification<TurnOrdering>) -> Classification<TurnOrdering> {
    ordering.map(|ordering| match ordering {
        TurnOrdering::FirstBeforeSecond => TurnOrdering::SecondBeforeFirst,
        TurnOrdering::SecondBeforeFirst => TurnOrdering::FirstBeforeSecond,
        TurnOrdering::SameDirection => TurnOrdering::SameDirection,
    })
}

fn compare_filled_left_face_turn_from_base(
    base: &TangentVector,
    first: &TangentVector,
    second: &TangentVector,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let first_half = match turn_half(base, first, policy) {
        Some(half) => half,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    let second_half = match turn_half(base, second, policy) {
        Some(half) => half,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    if first_half != second_half {
        return Classification::Decided(if first_half < second_half {
            TurnOrdering::FirstBeforeSecond
        } else {
            TurnOrdering::SecondBeforeFirst
        });
    }
    match real_sign(&cross_vectors(first, second), policy) {
        Some(RealSign::Positive) => Classification::Decided(TurnOrdering::SecondBeforeFirst),
        Some(RealSign::Negative) => Classification::Decided(TurnOrdering::FirstBeforeSecond),
        Some(RealSign::Zero) => Classification::Decided(TurnOrdering::SameDirection),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn compare_turn_from_base(
    base: &TangentVector,
    first: &TangentVector,
    second: &TangentVector,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let first_half = match turn_half(base, first, policy) {
        Some(half) => half,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    let second_half = match turn_half(base, second, policy) {
        Some(half) => half,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    if first_half != second_half {
        return Classification::Decided(if first_half < second_half {
            TurnOrdering::FirstBeforeSecond
        } else {
            TurnOrdering::SecondBeforeFirst
        });
    }

    match real_sign(&cross_vectors(first, second), policy) {
        Some(RealSign::Positive) => Classification::Decided(TurnOrdering::FirstBeforeSecond),
        Some(RealSign::Negative) => Classification::Decided(TurnOrdering::SecondBeforeFirst),
        Some(RealSign::Zero) => Classification::Decided(TurnOrdering::SameDirection),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn compare_retained_turn_from_base(
    base: &RetainedTangentVector,
    first: &RetainedTangentVector,
    second: &RetainedTangentVector,
    filled_left_faces: bool,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    match (base, first, second) {
        (
            RetainedTangentVector::Native(base),
            RetainedTangentVector::Native(first),
            RetainedTangentVector::Native(second),
        ) => {
            if filled_left_faces {
                compare_filled_left_face_turn_from_base(base, first, second, policy)
            } else {
                compare_turn_from_base(base, first, second, policy)
            }
        }
        _ => {
            let base = retained_tangent_as_algebraic(base);
            let first = retained_tangent_as_algebraic(first);
            let second = retained_tangent_as_algebraic(second);
            let comparison = if filled_left_faces {
                compare_algebraic_tangent_filled_left_face_sign_only(&base, &first, &second, policy)
            } else {
                compare_algebraic_tangent_turn_from_base_sign_only(&base, &first, &second, policy)
            };
            match comparison {
                Classification::Decided(evidence) => match evidence.status {
                    BezierAlgebraicTangentOrderStatus::Ordered => match evidence.ordering {
                        Some(BezierTangentTurnOrdering2::FirstBeforeSecond) => {
                            Classification::Decided(TurnOrdering::FirstBeforeSecond)
                        }
                        Some(BezierTangentTurnOrdering2::SecondBeforeFirst) => {
                            Classification::Decided(TurnOrdering::SecondBeforeFirst)
                        }
                        None => Classification::Uncertain(UncertaintyReason::Boundary),
                    },
                    BezierAlgebraicTangentOrderStatus::SameDirection => {
                        Classification::Decided(TurnOrdering::SameDirection)
                    }
                    BezierAlgebraicTangentOrderStatus::ZeroTangent
                    | BezierAlgebraicTangentOrderStatus::SignUndecided => {
                        Classification::Uncertain(UncertaintyReason::RealSign)
                    }
                    BezierAlgebraicTangentOrderStatus::ArithmeticFailed => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                },
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
    }
}

fn retained_tangent_as_algebraic(tangent: &RetainedTangentVector) -> BezierAlgebraicTangentVector2 {
    match tangent {
        RetainedTangentVector::Native(tangent) => BezierAlgebraicTangentVector2::new(
            exact_value_representation(&tangent.dx),
            exact_value_representation(&tangent.dy),
        ),
        RetainedTangentVector::Algebraic(tangent) => tangent.as_ref().clone(),
    }
}

fn compare_retained_same_tangent_second_order(
    first: &RetainedEndpointData,
    second: &RetainedEndpointData,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let first_tangent = match retained_algebraic_derivative(
        first.start_tangent.as_ref(),
        first.start_derivative_source.as_ref(),
        1,
        policy,
    ) {
        Classification::Decided(Some(tangent)) => tangent,
        Classification::Decided(None) => {
            return Classification::Decided(TurnOrdering::SameDirection);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let second_tangent = match retained_algebraic_derivative(
        second.start_tangent.as_ref(),
        second.start_derivative_source.as_ref(),
        1,
        policy,
    ) {
        Classification::Decided(Some(tangent)) => tangent,
        Classification::Decided(None) => {
            return Classification::Decided(TurnOrdering::SameDirection);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match (&first_tangent, &second_tangent) {
        (
            RetainedTangentVector::Native(first_tangent),
            RetainedTangentVector::Native(second_tangent),
        ) => compare_same_tangent_second_order(
            first_tangent,
            retained_native_vector(first.start_second_derivative.as_ref()),
            retained_native_vector(first.start_third_derivative.as_ref()),
            second_tangent,
            retained_native_vector(second.start_second_derivative.as_ref()),
            retained_native_vector(second.start_third_derivative.as_ref()),
            policy,
        ),
        (
            RetainedTangentVector::Algebraic(first_tangent),
            RetainedTangentVector::Algebraic(second_tangent),
        ) => {
            let first_second_derivative =
                match retained_algebraic_higher_derivative(first, 2, policy) {
                    Classification::Decided(derivative) => derivative,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
            let second_second_derivative =
                match retained_algebraic_higher_derivative(second, 2, policy) {
                    Classification::Decided(derivative) => derivative,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
            match (
                retained_algebraic_vector(first_second_derivative.as_ref()),
                retained_algebraic_vector(second_second_derivative.as_ref()),
            ) {
                (Some(first_second_derivative), Some(second_second_derivative)) => {
                    match compare_algebraic_same_tangent_second_order(
                        first_tangent,
                        first_second_derivative,
                        second_tangent,
                        second_second_derivative,
                        policy,
                    ) {
                        Classification::Decided(evidence) => {
                            if evidence.status
                                == BezierAlgebraicSameTangentOrderStatus::SameDirection
                            {
                                return compare_retained_algebraic_same_tangent_third_order(
                                    first,
                                    second,
                                    first_tangent,
                                    second_tangent,
                                    policy,
                                );
                            }
                            retained_algebraic_same_tangent_evidence_to_turn(
                                evidence.status,
                                evidence.ordering,
                            )
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                }
                _ => Classification::Decided(TurnOrdering::SameDirection),
            }
        }
        _ => Classification::Decided(TurnOrdering::SameDirection),
    }
}

fn compare_retained_algebraic_same_tangent_third_order(
    first: &RetainedEndpointData,
    second: &RetainedEndpointData,
    first_tangent: &BezierAlgebraicTangentVector2,
    second_tangent: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let first_third_derivative = match retained_algebraic_higher_derivative(first, 3, policy) {
        Classification::Decided(derivative) => derivative,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let second_third_derivative = match retained_algebraic_higher_derivative(second, 3, policy) {
        Classification::Decided(derivative) => derivative,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match (
        retained_algebraic_vector(first_third_derivative.as_ref()),
        retained_algebraic_vector(second_third_derivative.as_ref()),
    ) {
        (Some(first_third_derivative), Some(second_third_derivative)) => {
            match compare_algebraic_same_tangent_third_order(
                first_tangent,
                first_third_derivative,
                second_tangent,
                second_third_derivative,
                policy,
            ) {
                Classification::Decided(evidence) => {
                    retained_algebraic_same_tangent_evidence_to_turn(
                        evidence.status,
                        evidence.ordering,
                    )
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        _ => Classification::Decided(TurnOrdering::SameDirection),
    }
}

fn retained_algebraic_higher_derivative(
    endpoint: &RetainedEndpointData,
    order: usize,
    policy: &CurvePolicy,
) -> Classification<Option<RetainedTangentVector>> {
    let retained = match order {
        2 => &endpoint.start_second_derivative,
        3 => &endpoint.start_third_derivative,
        _ => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    retained_algebraic_derivative(
        retained.as_ref(),
        endpoint.start_derivative_source.as_ref(),
        order,
        policy,
    )
}

fn retained_algebraic_derivative(
    retained: Option<&RetainedTangentVector>,
    source: Option<&RetainedAlgebraicDerivativeSource>,
    order: usize,
    policy: &CurvePolicy,
) -> Classification<Option<RetainedTangentVector>> {
    if retained.is_some() {
        return Classification::Decided(retained.cloned());
    }
    let Some(source) = source else {
        return Classification::Decided(None);
    };
    let derivative = match source.curve.as_ref() {
        BezierSubcurve2::Quadratic(curve) => match order {
            1 => curve
                .tangent_at_algebraic_parameter(&source.parameter, policy)
                .map(Some),
            2 => curve
                .second_derivative_at_algebraic_parameter(&source.parameter, policy)
                .map(Some),
            _ => Ok(None),
        }
        .map(|derivative| derivative.map(BezierEndpointTangentImage2::Polynomial)),
        BezierSubcurve2::Cubic(curve) => match order {
            1 => curve
                .tangent_at_algebraic_parameter(&source.parameter, policy)
                .map(Some),
            2 => curve
                .second_derivative_at_algebraic_parameter(&source.parameter, policy)
                .map(Some),
            3 => curve
                .third_derivative_at_algebraic_parameter(&source.parameter, policy)
                .map(Some),
            _ => Ok(None),
        }
        .map(|derivative| derivative.map(BezierEndpointTangentImage2::Polynomial)),
        BezierSubcurve2::RationalQuadratic(curve) => curve
            .derivatives_at_algebraic_parameter(&source.parameter, order, policy)
            .map(|derivatives| {
                derivatives
                    .into_iter()
                    .nth(order - 1)
                    .map(BezierEndpointTangentImage2::Rational)
            }),
        BezierSubcurve2::Rational(curve) => curve
            .derivatives_at_algebraic_parameter(&source.parameter, order, policy)
            .map(|derivatives| {
                derivatives
                    .into_iter()
                    .nth(order - 1)
                    .map(BezierEndpointTangentImage2::Rational)
            }),
    };
    let mut derivative = match derivative {
        Ok(derivative) => derivative.as_ref().and_then(retained_algebraic_tangent),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if source.reversed && order % 2 == 1 {
        derivative = match derivative {
            Some(derivative) => match negate_retained_tangent(derivative) {
                Some(derivative) => Some(derivative),
                None => return Classification::Uncertain(UncertaintyReason::Unsupported),
            },
            None => None,
        };
    }
    Classification::Decided(derivative)
}

fn retained_algebraic_same_tangent_evidence_to_turn(
    status: BezierAlgebraicSameTangentOrderStatus,
    ordering: Option<BezierTangentTurnOrdering2>,
) -> Classification<TurnOrdering> {
    match status {
        BezierAlgebraicSameTangentOrderStatus::Ordered => match ordering {
            Some(BezierTangentTurnOrdering2::FirstBeforeSecond) => {
                Classification::Decided(TurnOrdering::FirstBeforeSecond)
            }
            Some(BezierTangentTurnOrdering2::SecondBeforeFirst) => {
                Classification::Decided(TurnOrdering::SecondBeforeFirst)
            }
            None => Classification::Uncertain(UncertaintyReason::Boundary),
        },
        BezierAlgebraicSameTangentOrderStatus::SameDirection => {
            Classification::Decided(TurnOrdering::SameDirection)
        }
        BezierAlgebraicSameTangentOrderStatus::ZeroTangent
        | BezierAlgebraicSameTangentOrderStatus::SignUndecided => {
            Classification::Uncertain(UncertaintyReason::RealSign)
        }
        BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
    }
}

fn retained_native_vector(vector: Option<&RetainedTangentVector>) -> Option<&TangentVector> {
    match vector {
        Some(RetainedTangentVector::Native(vector)) => Some(vector),
        _ => None,
    }
}

fn retained_algebraic_vector(
    vector: Option<&RetainedTangentVector>,
) -> Option<&BezierAlgebraicTangentVector2> {
    match vector {
        Some(RetainedTangentVector::Algebraic(vector)) => Some(vector),
        _ => None,
    }
}

fn compare_same_tangent_second_order(
    first_tangent: &TangentVector,
    first_second_derivative: Option<&TangentVector>,
    first_third_derivative: Option<&TangentVector>,
    second_tangent: &TangentVector,
    second_second_derivative: Option<&TangentVector>,
    second_third_derivative: Option<&TangentVector>,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let Some(first_second_derivative) = first_second_derivative else {
        return Classification::Decided(TurnOrdering::SameDirection);
    };
    let Some(second_second_derivative) = second_second_derivative else {
        return Classification::Decided(TurnOrdering::SameDirection);
    };

    // Same first-order directions need a higher-order local witness.  For
    // polynomial Bezier arcs we compare signed curvature
    // `cross(B'(0), B''(0)) / |B'(0)|^3` exactly by clearing denominators:
    // the sign gives the side of departure and the squared, speed-scaled
    // magnitude orders arcs departing on the same side.  This is the
    // expression underlying standard parametric curvature, used here only as
    // an exact predicate in the exactness model's EGC sense.
    let first_cross = cross_vectors(first_tangent, first_second_derivative);
    let second_cross = cross_vectors(second_tangent, second_second_derivative);
    let first_sign = match real_sign(&first_cross, policy) {
        Some(sign) => sign,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    let second_sign = match real_sign(&second_cross, policy) {
        Some(sign) => sign,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };

    match (first_sign, second_sign) {
        (RealSign::Zero, RealSign::Zero) => compare_same_tangent_third_order(
            first_tangent,
            first_third_derivative,
            second_tangent,
            second_third_derivative,
            policy,
        ),
        (RealSign::Zero, _) | (_, RealSign::Zero) => {
            Classification::Decided(TurnOrdering::SameDirection)
        }
        (RealSign::Positive, RealSign::Negative) => {
            Classification::Decided(TurnOrdering::FirstBeforeSecond)
        }
        (RealSign::Negative, RealSign::Positive) => {
            Classification::Decided(TurnOrdering::SecondBeforeFirst)
        }
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            compare_same_side_curvature_magnitude(
                first_tangent,
                &first_cross,
                second_tangent,
                &second_cross,
                policy,
            )
        }
    }
}

fn compare_same_tangent_third_order(
    first_tangent: &TangentVector,
    first_third_derivative: Option<&TangentVector>,
    second_tangent: &TangentVector,
    second_third_derivative: Option<&TangentVector>,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let Some(first_third_derivative) = first_third_derivative else {
        return Classification::Decided(TurnOrdering::SameDirection);
    };
    let Some(second_third_derivative) = second_third_derivative else {
        return Classification::Decided(TurnOrdering::SameDirection);
    };

    // If `cross(B'(0), B''(0))` vanishes for both candidates, a cubic Bezier
    // can still peel away at third order.  We compare
    // `cross(B'(0), B'''(0))` as an exact Taylor witness and scale same-side
    // magnitudes by speed to avoid treating a parameter-speed change as a
    // topology decision.  The derivative identities are the polynomial Bezier
    // endpoint formulas from the Bernstein and de Casteljau curve model; using them only after exact sign certification follows exact-computation discipline.
    let first_cross = cross_vectors(first_tangent, first_third_derivative);
    let second_cross = cross_vectors(second_tangent, second_third_derivative);
    let first_sign = match real_sign(&first_cross, policy) {
        Some(sign) => sign,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    let second_sign = match real_sign(&second_cross, policy) {
        Some(sign) => sign,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };

    match (first_sign, second_sign) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => {
            Classification::Decided(TurnOrdering::SameDirection)
        }
        (RealSign::Positive, RealSign::Negative) => {
            Classification::Decided(TurnOrdering::FirstBeforeSecond)
        }
        (RealSign::Negative, RealSign::Positive) => {
            Classification::Decided(TurnOrdering::SecondBeforeFirst)
        }
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            compare_same_side_third_order_magnitude(
                first_tangent,
                &first_cross,
                second_tangent,
                &second_cross,
                policy,
            )
        }
    }
}

fn compare_same_side_curvature_magnitude(
    first_tangent: &TangentVector,
    first_cross: &Real,
    second_tangent: &TangentVector,
    second_cross: &Real,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let first_speed_sq = speed_squared(first_tangent);
    let second_speed_sq = speed_squared(second_tangent);
    if !definitely_nonzero(&first_speed_sq, policy) || !definitely_nonzero(&second_speed_sq, policy)
    {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    }

    let first_scaled = first_cross * first_cross * cube(&second_speed_sq);
    let second_scaled = second_cross * second_cross * cube(&first_speed_sq);
    match real_sign(&(first_scaled - second_scaled), policy) {
        Some(RealSign::Negative) => Classification::Decided(TurnOrdering::FirstBeforeSecond),
        Some(RealSign::Positive) => Classification::Decided(TurnOrdering::SecondBeforeFirst),
        Some(RealSign::Zero) => Classification::Decided(TurnOrdering::SameDirection),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn compare_same_side_third_order_magnitude(
    first_tangent: &TangentVector,
    first_cross: &Real,
    second_tangent: &TangentVector,
    second_cross: &Real,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    let first_speed_sq = speed_squared(first_tangent);
    let second_speed_sq = speed_squared(second_tangent);
    if !definitely_nonzero(&first_speed_sq, policy) || !definitely_nonzero(&second_speed_sq, policy)
    {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    }

    let first_scaled = first_cross * first_cross * square(&second_speed_sq);
    let second_scaled = second_cross * second_cross * square(&first_speed_sq);
    match real_sign(&(first_scaled - second_scaled), policy) {
        Some(RealSign::Negative) => Classification::Decided(TurnOrdering::FirstBeforeSecond),
        Some(RealSign::Positive) => Classification::Decided(TurnOrdering::SecondBeforeFirst),
        Some(RealSign::Zero) => Classification::Decided(TurnOrdering::SameDirection),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn turn_half(base: &TangentVector, candidate: &TangentVector, policy: &CurvePolicy) -> Option<u8> {
    match real_sign(&cross_vectors(base, candidate), policy)? {
        RealSign::Positive => Some(0),
        RealSign::Negative => Some(1),
        RealSign::Zero => match real_sign(&dot_vectors(base, candidate), policy)? {
            RealSign::Positive => Some(0),
            RealSign::Negative => Some(1),
            RealSign::Zero => None,
        },
    }
}

fn points_equal(left: &Point2, right: &Point2, policy: &CurvePolicy) -> Option<bool> {
    if left.identity() == right.identity() {
        return Some(true);
    }
    if compare_reals(left.x(), right.x(), policy)? != Ordering::Equal {
        return Some(false);
    }
    Some(compare_reals(left.y(), right.y(), policy)? == Ordering::Equal)
}

fn retained_endpoints_equal(
    left_topology_vertex: Option<usize>,
    left: Option<&RetainedEndpointKey>,
    right_topology_vertex: Option<usize>,
    right: Option<&RetainedEndpointKey>,
    policy: &CurvePolicy,
) -> Option<bool> {
    if let (Some(left), Some(right)) = (left_topology_vertex, right_topology_vertex) {
        return Some(left == right);
    }
    let (Some(left), Some(right)) = (left, right) else {
        return Some(false);
    };
    match (left, right) {
        (RetainedEndpointKey::Exact(left), RetainedEndpointKey::Exact(right)) => {
            points_equal(left, right, policy)
        }
        (
            RetainedEndpointKey::Algebraic {
                x: left_x,
                y: left_y,
            },
            RetainedEndpointKey::Algebraic {
                x: right_x,
                y: right_y,
            },
        ) => Some(
            represented_roots_equal(left_x, right_x, policy)?
                && represented_roots_equal(left_y, right_y, policy)?,
        ),
        (RetainedEndpointKey::Exact(point), RetainedEndpointKey::Algebraic { x, y })
        | (RetainedEndpointKey::Algebraic { x, y }, RetainedEndpointKey::Exact(point)) => Some(
            represented_roots_equal(x, &exact_value_representation(point.x()), policy)?
                && represented_roots_equal(y, &exact_value_representation(point.y()), policy)?,
        ),
    }
}

fn compare_reals_equal(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(crate::classify::compare_reals(left, right, policy)? == std::cmp::Ordering::Equal)
}

pub(crate) fn represented_roots_equal(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: &CurvePolicy,
) -> Option<bool> {
    if left == right {
        return Some(true);
    }
    if let (Some(left_witness), Some(right_witness)) = (
        left.exact_rational_witness(),
        right.exact_rational_witness(),
    ) {
        return compare_reals_equal(left_witness, right_witness, policy);
    }

    // Algebraic endpoint images produced from different curve expressions can
    // represent the same point without having byte-identical construction
    // payloads. Refine the roots and, when needed, construct their exact
    // difference rather than comparing interval samples.
    compare_represented_roots_by_difference(left, right, policy)
}

#[cfg(feature = "predicates")]
fn compare_represented_roots_by_difference(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: &CurvePolicy,
) -> Option<bool> {
    let comparison = compare_algebraic_root_representations_by_difference(
        left,
        right,
        AlgebraicRootRefinementComparisonConfig {
            policy: policy.predicate_policy,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    (comparison.comparison.status == AlgebraicRootComparisonStatus::Compared)
        .then_some(
            comparison
                .comparison
                .ordering
                .map(|ordering| ordering.is_eq()),
        )
        .flatten()
}

#[cfg(not(feature = "predicates"))]
fn compare_represented_roots_by_difference(
    _left: &AlgebraicRootRepresentation,
    _right: &AlgebraicRootRepresentation,
    _policy: &CurvePolicy,
) -> Option<bool> {
    None
}

impl BezierSubcurve2 {
    /// Returns the exact start and end points of this native subcurve.
    pub fn endpoints(&self) -> (Point2, Point2) {
        match self {
            Self::Quadratic(curve) => (curve.start().clone(), curve.end().clone()),
            Self::Cubic(curve) => (curve.start().clone(), curve.end().clone()),
            Self::RationalQuadratic(curve) => (curve.start().clone(), curve.end().clone()),
            Self::Rational(curve) => (curve.start().clone(), curve.end().clone()),
        }
    }

    /// Returns the exact start point of this native subcurve.
    pub fn start_point(&self) -> Point2 {
        self.endpoints().0
    }

    /// Returns the exact end point of this native subcurve.
    pub fn end_point(&self) -> Point2 {
        self.endpoints().1
    }

    fn endpoint_data(&self, policy: &CurvePolicy) -> Classification<EndpointData> {
        self.endpoint_data_with_higher_derivatives(policy, true)
    }

    fn endpoint_data_with_higher_derivatives(
        &self,
        policy: &CurvePolicy,
        include_higher_derivatives: bool,
    ) -> Classification<EndpointData> {
        let (start, end) = self.endpoints();
        let (
            start_tangent,
            end_tangent,
            start_second_derivative,
            start_third_derivative,
            start_tangent_zero_status,
            end_tangent_zero_status,
        ) = match self {
            Self::Quadratic(curve) => {
                let (start_tangent, start_tangent_zero_status) =
                    TangentVector::from_endpoint_tangent(
                        curve.endpoint_tangent(BezierEndpoint::Start),
                    );
                let (end_tangent, end_tangent_zero_status) = TangentVector::from_endpoint_tangent(
                    curve.endpoint_tangent(BezierEndpoint::End),
                );
                (
                    start_tangent,
                    end_tangent,
                    include_higher_derivatives.then(|| quadratic_second_derivative(curve)),
                    None,
                    Some(start_tangent_zero_status),
                    Some(end_tangent_zero_status),
                )
            }
            Self::Cubic(curve) => {
                let (start_tangent, start_tangent_zero_status) =
                    TangentVector::from_endpoint_tangent(
                        curve.endpoint_tangent(BezierEndpoint::Start),
                    );
                let (end_tangent, end_tangent_zero_status) = TangentVector::from_endpoint_tangent(
                    curve.endpoint_tangent(BezierEndpoint::End),
                );
                (
                    start_tangent,
                    end_tangent,
                    include_higher_derivatives.then(|| cubic_start_second_derivative(curve)),
                    include_higher_derivatives.then(|| cubic_third_derivative(curve)),
                    Some(start_tangent_zero_status),
                    Some(end_tangent_zero_status),
                )
            }
            Self::RationalQuadratic(curve) => {
                let start = match rational_quadratic_endpoint_derivative_jet(
                    curve,
                    false,
                    include_higher_derivatives,
                    policy,
                ) {
                    Classification::Decided(derivatives) => derivatives,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
                let end =
                    match rational_quadratic_endpoint_derivative_jet(curve, true, false, policy) {
                        Classification::Decided(derivatives) => derivatives,
                        Classification::Uncertain(reason) => {
                            return Classification::Uncertain(reason);
                        }
                    };
                (
                    start.first,
                    end.first,
                    start.second,
                    start.third,
                    None,
                    None,
                )
            }
            Self::Rational(curve) => {
                let start = match curve.endpoint_derivatives(
                    false,
                    if include_higher_derivatives { 3 } else { 1 },
                    policy,
                ) {
                    Classification::Decided(derivatives) => derivatives,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
                let end = match curve.endpoint_derivatives(true, 1, policy) {
                    Classification::Decided(derivatives) => derivatives,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
                let vector = |derivative: &(Real, Real)| TangentVector {
                    dx: derivative.0.clone(),
                    dy: derivative.1.clone(),
                };
                (
                    vector(&start[1]),
                    vector(&end[1]),
                    start.get(2).map(vector),
                    start.get(3).map(vector),
                    None,
                    None,
                )
            }
        };

        if !start_tangent.is_nonzero_with_status(start_tangent_zero_status, policy)
            || !end_tangent.is_nonzero_with_status(end_tangent_zero_status, policy)
        {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }

        Classification::Decided(EndpointData {
            start,
            end,
            start_tangent,
            end_tangent,
            start_second_derivative,
            start_third_derivative,
        })
    }
}

struct RationalQuadraticEndpointDerivativeJet {
    first: TangentVector,
    second: Option<TangentVector>,
    third: Option<TangentVector>,
}

fn rational_quadratic_endpoint_derivative_jet(
    curve: &crate::RationalQuadraticBezier2,
    at_end: bool,
    higher_orders: bool,
    policy: &CurvePolicy,
) -> Classification<RationalQuadraticEndpointDerivativeJet> {
    let (point0, point1, point2, weight0, weight1, weight2) = if at_end {
        (
            curve.end(),
            curve.control(),
            curve.start(),
            curve.end_weight(),
            curve.control_weight(),
            curve.start_weight(),
        )
    } else {
        (
            curve.start(),
            curve.control(),
            curve.end(),
            curve.start_weight(),
            curve.control_weight(),
            curve.end_weight(),
        )
    };
    match is_zero(weight0, policy) {
        Some(false) => {}
        Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }
    let Ok(weight_ratio1) = weight1 / weight0 else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let difference1 = TangentVector {
        dx: point1.x() - point0.x(),
        dy: point1.y() - point0.y(),
    };
    let two = Real::from(2_i8);
    let first_scale = &two * &weight_ratio1;
    let first_in_reversed_parameter = TangentVector {
        dx: &first_scale * &difference1.dx,
        dy: &first_scale * &difference1.dy,
    };
    if !higher_orders {
        let first = if at_end {
            TangentVector {
                dx: -first_in_reversed_parameter.dx,
                dy: -first_in_reversed_parameter.dy,
            }
        } else {
            first_in_reversed_parameter
        };
        return Classification::Decided(RationalQuadraticEndpointDerivativeJet {
            first,
            second: None,
            third: None,
        });
    }

    let Ok(weight_ratio2) = weight2 / weight0 else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let difference2 = TangentVector {
        dx: point2.x() - point0.x(),
        dy: point2.y() - point0.y(),
    };
    let second_scale1 = Real::from(4_i8) * &weight_ratio1 * (Real::one() - (&two * &weight_ratio1));
    let second_scale2 = &two * &weight_ratio2;
    let second = TangentVector {
        dx: (&second_scale1 * &difference1.dx) + (&second_scale2 * &difference2.dx),
        dy: (&second_scale1 * &difference1.dy) + (&second_scale2 * &difference2.dy),
    };
    let denominator_first = &two * (&weight_ratio1 - Real::one());
    let denominator_second = &two * ((Real::one() - (&two * &weight_ratio1)) + &weight_ratio2);
    let mut third = TangentVector {
        dx: -(Real::from(3_i8)
            * ((&denominator_first * &second.dx)
                + (&denominator_second * &first_in_reversed_parameter.dx))),
        dy: -(Real::from(3_i8)
            * ((&denominator_first * &second.dy)
                + (&denominator_second * &first_in_reversed_parameter.dy))),
    };
    let first = if at_end {
        third.dx = -third.dx;
        third.dy = -third.dy;
        TangentVector {
            dx: -first_in_reversed_parameter.dx,
            dy: -first_in_reversed_parameter.dy,
        }
    } else {
        first_in_reversed_parameter
    };
    Classification::Decided(RationalQuadraticEndpointDerivativeJet {
        first,
        second: Some(second),
        third: Some(third),
    })
}

#[cfg(test)]
mod rational_quadratic_endpoint_derivative_tests {
    use super::*;

    #[test]
    fn polynomial_endpoint_tangent_keeps_its_structural_zero_evidence() {
        let policy = CurvePolicy::STRICT;
        for (dx, expected_status, expected_nonzero) in [
            (Real::zero(), ZeroStatus::Zero, false),
            (Real::from(7_i8), ZeroStatus::NonZero, true),
        ] {
            let endpoint = crate::EndpointTangent2::new(dx, Real::zero());
            let (tangent, status) = TangentVector::from_endpoint_tangent(endpoint);

            assert_eq!(status, expected_status);
            assert_eq!(
                tangent.is_nonzero_with_status(Some(status), &policy),
                expected_nonzero
            );
        }
    }

    #[test]
    fn specialized_endpoint_jets_match_general_rational_quotient_derivatives() {
        let policy = CurvePolicy::STRICT;

        for (weight_case, (start_weight, control_weight, end_weight)) in [
            (Real::one(), Real::one(), Real::one()),
            (Real::from(2_i8), Real::from(3_i8), Real::from(5_i8)),
            (Real::from(-7_i8), Real::from(-2_i8), Real::from(-3_i8)),
        ]
        .into_iter()
        .enumerate()
        {
            let curve = crate::RationalQuadraticBezier2::try_new(
                Point2::new(Real::from(-3_i8), Real::from(2_i8)),
                Point2::new(Real::from(5_i8), Real::from(11_i8)),
                Point2::new(Real::from(17_i8), Real::from(-7_i8)),
                start_weight,
                control_weight,
                end_weight,
            )
            .expect("nonzero rational weights");
            let general = crate::RationalBezier2::try_new(
                curve.control_points().into_iter().cloned().collect(),
                curve.weights().into_iter().cloned().collect(),
            )
            .expect("valid general rational curve");

            for at_end in [false, true] {
                let Classification::Decided(specialized) =
                    rational_quadratic_endpoint_derivative_jet(&curve, at_end, true, &policy)
                else {
                    panic!("specialized endpoint jet should be exact");
                };
                let Classification::Decided(general) =
                    general.endpoint_derivatives(at_end, 3, &policy)
                else {
                    panic!("general endpoint jet should be exact");
                };
                for (derivative_order, (specialized, general)) in [
                    (&specialized.first, &general[1]),
                    (
                        specialized
                            .second
                            .as_ref()
                            .expect("second derivative requested"),
                        &general[2],
                    ),
                    (
                        specialized
                            .third
                            .as_ref()
                            .expect("third derivative requested"),
                        &general[3],
                    ),
                ]
                .into_iter()
                .enumerate()
                {
                    assert_eq!(
                        compare_reals(&specialized.dx, &general.0, &policy),
                        Some(Ordering::Equal),
                        "weight case {weight_case}, at_end={at_end}, derivative order {} x",
                        derivative_order + 1
                    );
                    assert_eq!(
                        compare_reals(&specialized.dy, &general.1, &policy),
                        Some(Ordering::Equal),
                        "weight case {weight_case}, at_end={at_end}, derivative order {} y",
                        derivative_order + 1
                    );
                }
            }
        }
    }
}

impl TangentVector {
    fn from_endpoint_tangent(tangent: crate::EndpointTangent2) -> (Self, ZeroStatus) {
        let (dx, dy, zero_status) = tangent.into_components();
        (Self { dx, dy }, zero_status)
    }

    fn is_nonzero_with_status(
        &self,
        zero_status: Option<ZeroStatus>,
        policy: &CurvePolicy,
    ) -> bool {
        match zero_status {
            Some(ZeroStatus::NonZero) => true,
            Some(ZeroStatus::Zero) => false,
            Some(ZeroStatus::Unknown) | None => self.is_nonzero(policy),
        }
    }

    fn is_nonzero(&self, policy: &CurvePolicy) -> bool {
        let length_squared = &self.dx * &self.dx + &self.dy * &self.dy;
        match length_squared.zero_status() {
            ZeroStatus::NonZero => true,
            ZeroStatus::Zero => false,
            ZeroStatus::Unknown => is_zero(&length_squared, policy) == Some(false),
        }
    }
}

fn quadratic_second_derivative(curve: &crate::QuadraticBezier2) -> TangentVector {
    let two = Real::from(2_i8);
    let dx = &two * ((curve.start().x() - (&two * curve.control().x())) + curve.end().x());
    let dy = &two * ((curve.start().y() - (&two * curve.control().y())) + curve.end().y());
    TangentVector { dx, dy }
}

fn cubic_start_second_derivative(curve: &crate::CubicBezier2) -> TangentVector {
    let six = Real::from(6_i8);
    let dx = &six
        * ((curve.start().x() - (Real::from(2_i8) * curve.control1().x())) + curve.control2().x());
    let dy = &six
        * ((curve.start().y() - (Real::from(2_i8) * curve.control1().y())) + curve.control2().y());
    TangentVector { dx, dy }
}

fn cubic_third_derivative(curve: &crate::CubicBezier2) -> TangentVector {
    let six = Real::from(6_i8);
    let three = Real::from(3_i8);
    let dx = &six
        * (((curve.end().x() - (&three * curve.control2().x())) + (&three * curve.control1().x()))
            - curve.start().x());
    let dy = &six
        * (((curve.end().y() - (&three * curve.control2().y())) + (&three * curve.control1().y()))
            - curve.start().y());
    TangentVector { dx, dy }
}

fn cross_vectors(left: &TangentVector, right: &TangentVector) -> Real {
    (&left.dx * &right.dy) - (&left.dy * &right.dx)
}

fn dot_vectors(left: &TangentVector, right: &TangentVector) -> Real {
    (&left.dx * &right.dx) + (&left.dy * &right.dy)
}

fn speed_squared(vector: &TangentVector) -> Real {
    (&vector.dx * &vector.dx) + (&vector.dy * &vector.dy)
}

fn cube(value: &Real) -> Real {
    value * value * value
}

fn square(value: &Real) -> Real {
    value * value
}

fn definitely_nonzero(value: &Real, policy: &CurvePolicy) -> bool {
    match value.zero_status() {
        ZeroStatus::NonZero => true,
        ZeroStatus::Zero => false,
        ZeroStatus::Unknown => is_zero(value, policy) == Some(false),
    }
}
