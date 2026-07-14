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
//! overlaps, coincident tangents, zero tangents, and mixed unsupported evidence.
//!
//! That boundary is the exact-computation discipline described by Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 3-23 (1997): topology code may retain unresolved exact objects, but it must
//! not invent a floating successor. The branch-free chain walk mirrors the
//! regularized graph assumption in Greiner and Hormann, "Efficient clipping of
//! arbitrary polygons," *ACM Transactions on Graphics* 17(2), 71-83 (1998),
//! while multi-successor handling follows the degeneracy split emphasized by
//! Foster, Hormann, and Popa, "Clipping simple polygons with degenerate
//! intersections," *Computers & Graphics: X* 2, 100007 (2019): when local
//! order is not certified, traversal stops instead of guessing.

use hyperreal::{Real, RealSign};
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticStatus, AlgebraicRootRepresentation,
    arithmetic_algebraic_root_representations,
};
#[cfg(feature = "predicates")]
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    compare_algebraic_root_representations_by_difference,
};

use crate::classify::{compare_reals, is_zero, real_sign};
use crate::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicSameTangentOrderStatus,
    BezierAlgebraicTangentOrderStatus, BezierAlgebraicTangentVector2, BezierEndpoint,
    BezierEndpointPointImage2, BezierEndpointTangentImage2, BezierParameter2, BezierSplitFragment2,
    BezierSplitMaterialization2, BezierSubcurve2, BezierTangentTurnOrdering2, Classification,
    CurveError, CurvePolicy, CurveResult, Point2, UncertaintyReason, ZeroStatus,
    compare_algebraic_same_tangent_second_order, compare_algebraic_same_tangent_third_order,
    compare_algebraic_tangent_turn_from_base,
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
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BezierArrangementGraph2 {
    fragments: Vec<BezierArrangementFragment2>,
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
        Ok(Self { fragments })
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
    /// explicit uncertainty in Yap's sense.
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

        let outgoing = match outgoing_adjacency(&endpoints, policy) {
            Classification::Decided(outgoing) => outgoing,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let predecessors = match predecessor_counts(&endpoints, policy) {
            Classification::Decided(predecessors) => predecessors,
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
    /// [`compare_algebraic_tangent_turn_from_base`].
    ///
    /// The method deliberately does not materialize concrete Bezier regions
    /// from algebraic fragments. It only proves traversal order over retained
    /// evidence, preserving Yap's construction/decision boundary from
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
    /// 3-23 (1997), and matching the arrangement local-order discipline in de
    /// Berg et al., *Computational Geometry* (3rd ed., 2008).
    pub fn traverse_retained_with_tangent_order(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierArrangementTraversal2> {
        let mut endpoints = Vec::with_capacity(self.fragments.len());
        for fragment in &self.fragments {
            let endpoints_for_fragment = match retained_endpoint_data(fragment, policy) {
                Some(Classification::Decided(endpoints)) => endpoints,
                Some(Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
                None => return Classification::Uncertain(UncertaintyReason::Boundary),
            };
            endpoints.push(endpoints_for_fragment);
        }

        let outgoing = match retained_outgoing_adjacency(&endpoints, policy) {
            Classification::Decided(outgoing) => outgoing,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let predecessors = match retained_predecessor_counts(&endpoints, policy) {
            Classification::Decided(predecessors) => predecessors,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        let mut used = vec![false; self.fragments.len()];
        let mut chains = Vec::new();
        for index in 0..self.fragments.len() {
            if predecessors[index] == 0 && !used[index] {
                match follow_retained_tangent_ordered_chain(
                    index, &outgoing, &endpoints, &mut used, policy,
                ) {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }
        for index in 0..self.fragments.len() {
            if !used[index] {
                match follow_retained_tangent_ordered_chain(
                    index, &outgoing, &endpoints, &mut used, policy,
                ) {
                    Classification::Decided(chain) => chains.push(chain),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }

        decided_arrangement_traversal(chains)
    }
}

fn validate_arrangement_fragment_provenance(
    fragments: &[BezierArrangementFragment2],
) -> CurveResult<()> {
    let policy = CurvePolicy::certified();
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
                if &expected != image {
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
    point: RetainedEndpointKey,
    tangent: RetainedTangentVector,
    second_derivative: Option<RetainedTangentVector>,
    third_derivative: Option<RetainedTangentVector>,
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

fn retained_endpoint_data(
    arrangement_fragment: &BezierArrangementFragment2,
    policy: &CurvePolicy,
) -> Option<Classification<RetainedEndpointData>> {
    let fragment = arrangement_fragment.fragment();
    match fragment {
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
            let source_start = match retained_endpoint_side_data(
                start,
                start_image.as_ref(),
                source_curve.as_ref(),
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
            let (start, start_tangent, start_second_derivative, start_third_derivative) =
                retained_endpoint_side_parts(start);
            let (end, end_tangent, _, _) = retained_endpoint_side_parts(end);
            Some(Classification::Decided(RetainedEndpointData {
                start,
                end,
                start_topology_vertex: arrangement_fragment.start_topology_vertex(),
                end_topology_vertex: arrangement_fragment.end_topology_vertex(),
                start_tangent,
                end_tangent,
                start_second_derivative,
                start_third_derivative,
            }))
        }
        BezierSplitFragment2::Unresolved { .. } => None,
    }
}

fn retained_endpoint_side_data(
    parameter: &BezierParameter2,
    image: Option<&BezierAlgebraicEndpointImage2>,
    source_curve: Option<&BezierSubcurve2>,
    policy: &CurvePolicy,
) -> Classification<Option<RetainedEndpointSideData>> {
    if let Some(image) = image {
        let Some(point) = retained_algebraic_point_key(image.point()) else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let Some(tangent) = retained_algebraic_tangent(image.tangent()) else {
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
            point,
            tangent,
            second_derivative,
            third_derivative,
        }));
    }

    let (BezierParameter2::Exact(parameter), Some(source_curve)) = (parameter, source_curve) else {
        return Classification::Decided(None);
    };
    retained_exact_source_endpoint_side_data(source_curve, parameter, policy).map(Some)
}

fn retained_exact_source_endpoint_side_data(
    source_curve: &BezierSubcurve2,
    parameter: &Real,
    policy: &CurvePolicy,
) -> Classification<RetainedEndpointSideData> {
    let at_source_end = match compare_reals(parameter, &Real::one(), policy) {
        Some(ordering) => ordering == std::cmp::Ordering::Equal,
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let (data, restore_source_orientation) = if at_source_end {
        match source_curve.reversed().endpoint_data(policy) {
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
        match subcurve.endpoint_data(policy) {
            Classification::Decided(data) => (data, false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    };
    let side = RetainedEndpointSideData {
        point: RetainedEndpointKey::Exact(Box::new(data.start)),
        tangent: RetainedTangentVector::Native(Box::new(data.start_tangent)),
        second_derivative: data
            .start_second_derivative
            .map(Box::new)
            .map(RetainedTangentVector::Native),
        third_derivative: data
            .start_third_derivative
            .map(Box::new)
            .map(RetainedTangentVector::Native),
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
) {
    match side {
        Some(side) => (
            Some(side.point),
            Some(side.tangent),
            side.second_derivative,
            side.third_derivative,
        ),
        None => (None, None, None, None),
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
    side.tangent = negate_retained_tangent(side.tangent)?;
    side.third_derivative = match side.third_derivative {
        Some(derivative) => Some(negate_retained_tangent(derivative)?),
        None => None,
    };
    Some(side)
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
    let report =
        arithmetic_algebraic_root_representations(value, None, AlgebraicRootArithmeticOp::Negate);
    if !matches!(
        report.status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
    ) {
        return None;
    }
    if let Some(result) = report.result_representation {
        return Some(result);
    }
    report
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

    for (left_index, (_, left_end)) in endpoints.iter().enumerate() {
        for (right_index, (right_start, _)) in endpoints.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            match points_equal(left_end, right_start, policy) {
                Some(true) => {
                    if successors[left_index].replace(right_index).is_some()
                        || predecessors[right_index].replace(left_index).is_some()
                    {
                        return Classification::Uncertain(UncertaintyReason::Boundary);
                    }
                }
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
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

fn outgoing_adjacency(
    endpoints: &[EndpointData],
    policy: &CurvePolicy,
) -> Classification<Vec<Vec<usize>>> {
    let mut outgoing = vec![Vec::new(); endpoints.len()];
    for (left_index, left) in endpoints.iter().enumerate() {
        for (right_index, right) in endpoints.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            match points_equal(&left.end, &right.start, policy) {
                Some(true) => outgoing[left_index].push(right_index),
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
    }
    Classification::Decided(outgoing)
}

fn predecessor_counts(
    endpoints: &[EndpointData],
    policy: &CurvePolicy,
) -> Classification<Vec<usize>> {
    let mut predecessors = vec![0_usize; endpoints.len()];
    for (left_index, left) in endpoints.iter().enumerate() {
        for (right_index, right) in endpoints.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            match points_equal(&left.end, &right.start, policy) {
                Some(true) => predecessors[right_index] += 1,
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
    }
    Classification::Decided(predecessors)
}

fn retained_outgoing_adjacency(
    endpoints: &[RetainedEndpointData],
    policy: &CurvePolicy,
) -> Classification<Vec<Vec<usize>>> {
    let mut outgoing = vec![Vec::new(); endpoints.len()];
    for (left_index, left) in endpoints.iter().enumerate() {
        let Some(left_end) = left.end.as_ref() else {
            continue;
        };
        for (right_index, right) in endpoints.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            let Some(right_start) = right.start.as_ref() else {
                continue;
            };
            match retained_endpoints_equal(
                left.end_topology_vertex,
                left_end,
                right.start_topology_vertex,
                right_start,
                policy,
            ) {
                Some(true) => outgoing[left_index].push(right_index),
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
    }
    Classification::Decided(outgoing)
}

fn retained_predecessor_counts(
    endpoints: &[RetainedEndpointData],
    policy: &CurvePolicy,
) -> Classification<Vec<usize>> {
    let mut predecessors = vec![0_usize; endpoints.len()];
    for (left_index, left) in endpoints.iter().enumerate() {
        let Some(left_end) = left.end.as_ref() else {
            continue;
        };
        for (right_index, right) in endpoints.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            let Some(right_start) = right.start.as_ref() else {
                continue;
            };
            match retained_endpoints_equal(
                left.end_topology_vertex,
                left_end,
                right.start_topology_vertex,
                right_start,
                policy,
            ) {
                Some(true) => predecessors[right_index] += 1,
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
    }
    Classification::Decided(predecessors)
}

fn follow_retained_tangent_ordered_chain(
    start: usize,
    outgoing: &[Vec<usize>],
    endpoints: &[RetainedEndpointData],
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

        let next =
            match choose_retained_tangent_successor(current, &outgoing[current], endpoints, policy)
            {
                Classification::Decided(next) => next,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
        let Some(next) = next else {
            let closed = match (&endpoints[current].end, &first_start) {
                (Some(end), Some(start)) => match retained_endpoints_equal(
                    endpoints[current].end_topology_vertex,
                    end,
                    first_start_topology_vertex,
                    start,
                    policy,
                ) {
                    Some(value) => value,
                    None => return Classification::Uncertain(UncertaintyReason::RealSign),
                },
                _ => false,
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
    policy: &CurvePolicy,
) -> Classification<Option<usize>> {
    if candidates.is_empty() {
        return Classification::Decided(None);
    }
    if candidates.len() == 1 {
        return Classification::Decided(Some(candidates[0]));
    }

    let Some(base) = endpoints[current].end_tangent.as_ref() else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let mut best = candidates[0];

    for candidate in candidates.iter().copied().skip(1) {
        let Some(first) = endpoints[candidate].start_tangent.as_ref() else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let Some(second) = endpoints[best].start_tangent.as_ref() else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        match compare_retained_turn_from_base(base, first, second, policy) {
            Classification::Decided(TurnOrdering::FirstBeforeSecond) => best = candidate,
            Classification::Decided(TurnOrdering::SecondBeforeFirst) => {}
            Classification::Decided(TurnOrdering::SameDirection) => {
                match compare_retained_same_tangent_second_order(
                    &endpoints[candidate],
                    &endpoints[best],
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
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    match (base, first, second) {
        (
            RetainedTangentVector::Native(base),
            RetainedTangentVector::Native(first),
            RetainedTangentVector::Native(second),
        ) => compare_turn_from_base(base, first, second, policy),
        (
            RetainedTangentVector::Algebraic(base),
            RetainedTangentVector::Algebraic(first),
            RetainedTangentVector::Algebraic(second),
        ) => match compare_algebraic_tangent_turn_from_base(base, first, second, policy) {
            Classification::Decided(report) => match report.status {
                BezierAlgebraicTangentOrderStatus::Ordered => match report.ordering {
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
        },
        _ => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn compare_retained_same_tangent_second_order(
    first: &RetainedEndpointData,
    second: &RetainedEndpointData,
    policy: &CurvePolicy,
) -> Classification<TurnOrdering> {
    match (&first.start_tangent, &second.start_tangent) {
        (
            Some(RetainedTangentVector::Native(first_tangent)),
            Some(RetainedTangentVector::Native(second_tangent)),
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
            Some(RetainedTangentVector::Algebraic(first_tangent)),
            Some(RetainedTangentVector::Algebraic(second_tangent)),
        ) => match (
            retained_algebraic_vector(first.start_second_derivative.as_ref()),
            retained_algebraic_vector(second.start_second_derivative.as_ref()),
        ) {
            (Some(first_second_derivative), Some(second_second_derivative)) => {
                match compare_algebraic_same_tangent_second_order(
                    first_tangent,
                    first_second_derivative,
                    second_tangent,
                    second_second_derivative,
                    policy,
                ) {
                    Classification::Decided(report) => {
                        if report.status == BezierAlgebraicSameTangentOrderStatus::SameDirection {
                            return compare_retained_algebraic_same_tangent_third_order(
                                first,
                                second,
                                first_tangent,
                                second_tangent,
                                policy,
                            );
                        }
                        retained_algebraic_same_tangent_report_to_turn(
                            report.status,
                            report.ordering,
                        )
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
            _ => Classification::Decided(TurnOrdering::SameDirection),
        },
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
    match (
        retained_algebraic_vector(first.start_third_derivative.as_ref()),
        retained_algebraic_vector(second.start_third_derivative.as_ref()),
    ) {
        (Some(first_third_derivative), Some(second_third_derivative)) => {
            match compare_algebraic_same_tangent_third_order(
                first_tangent,
                first_third_derivative,
                second_tangent,
                second_third_derivative,
                policy,
            ) {
                Classification::Decided(report) => {
                    retained_algebraic_same_tangent_report_to_turn(report.status, report.ordering)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        _ => Classification::Decided(TurnOrdering::SameDirection),
    }
}

fn retained_algebraic_same_tangent_report_to_turn(
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
    // an exact predicate in Yap's EGC sense; see Yap, "Towards Exact Geometric
    // Computation," Computational Geometry 7(1-2), 3-23 (1997).
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
    // endpoint formulas from Farin, *Curves and Surfaces for CAGD* (5th ed.,
    // 2002); using them only after exact sign certification follows Yap,
    // "Towards Exact Geometric Computation," Computational Geometry 7(1-2),
    // 3-23 (1997).
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
    is_zero(&left.distance_squared(right), policy)
}

fn retained_endpoints_equal(
    left_topology_vertex: Option<usize>,
    left: &RetainedEndpointKey,
    right_topology_vertex: Option<usize>,
    right: &RetainedEndpointKey,
    policy: &CurvePolicy,
) -> Option<bool> {
    if let (Some(left), Some(right)) = (left_topology_vertex, right_topology_vertex) {
        return Some(left == right);
    }
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
        let (start, end) = self.endpoints();
        let (start_tangent, end_tangent, start_second_derivative, start_third_derivative) =
            match self {
                Self::Quadratic(curve) => {
                    let second_derivative = quadratic_second_derivative(curve);
                    (
                        TangentVector::from_endpoint_tangent(
                            curve.endpoint_tangent(BezierEndpoint::Start),
                        ),
                        TangentVector::from_endpoint_tangent(
                            curve.endpoint_tangent(BezierEndpoint::End),
                        ),
                        Some(second_derivative),
                        None,
                    )
                }
                Self::Cubic(curve) => (
                    TangentVector::from_endpoint_tangent(
                        curve.endpoint_tangent(BezierEndpoint::Start),
                    ),
                    TangentVector::from_endpoint_tangent(
                        curve.endpoint_tangent(BezierEndpoint::End),
                    ),
                    Some(cubic_start_second_derivative(curve)),
                    Some(cubic_third_derivative(curve)),
                ),
                Self::RationalQuadratic(curve) => {
                    let general = match crate::RationalBezier2::try_new(
                        curve.control_points().into_iter().cloned().collect(),
                        curve.weights().into_iter().cloned().collect(),
                    ) {
                        Ok(curve) => curve,
                        Err(_) => {
                            return Classification::Uncertain(UncertaintyReason::Unsupported);
                        }
                    };
                    let start = match general.endpoint_derivatives(false, 3, policy) {
                        Classification::Decided(derivatives) => derivatives,
                        Classification::Uncertain(reason) => {
                            return Classification::Uncertain(reason);
                        }
                    };
                    let end = match general.endpoint_derivatives(true, 1, policy) {
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
                    )
                }
                Self::Rational(curve) => {
                    let start = match curve.endpoint_derivatives(false, 3, policy) {
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
                    )
                }
            };

        if !start_tangent.is_nonzero(policy) || !end_tangent.is_nonzero(policy) {
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

impl TangentVector {
    fn from_endpoint_tangent(tangent: crate::EndpointTangent2) -> Self {
        Self {
            dx: tangent.dx().clone(),
            dy: tangent.dy().clone(),
        }
    }

    fn is_nonzero(&self, policy: &CurvePolicy) -> bool {
        match (&self.dx * &self.dx + &self.dy * &self.dy).zero_status() {
            ZeroStatus::NonZero => true,
            ZeroStatus::Zero => false,
            ZeroStatus::Unknown => {
                is_zero(&(&self.dx * &self.dx + &self.dy * &self.dy), policy) == Some(false)
            }
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
