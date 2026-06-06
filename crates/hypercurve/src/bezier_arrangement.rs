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

use crate::classify::{is_zero, real_sign};
use crate::{
    BezierAlgebraicSameTangentOrderStatus, BezierAlgebraicTangentOrderStatus,
    BezierAlgebraicTangentVector2, BezierEndpoint, BezierEndpointPointImage2,
    BezierEndpointTangentImage2, BezierSplitFragment2, BezierSplitMaterialization2,
    BezierSubcurve2, BezierTangentTurnOrdering2, Classification, CurveError, CurvePolicy,
    CurveResult, Point2, UncertaintyReason, ZeroStatus,
    compare_algebraic_same_tangent_second_order, compare_algebraic_same_tangent_third_order,
    compare_algebraic_tangent_turn_from_base,
};

/// One retained Bezier arrangement fragment with source provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierArrangementFragment2 {
    source_curve_index: usize,
    source_fragment_index: usize,
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
            fragment,
        }
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
    pub fn from_split_materializations(materializations: &[BezierSplitMaterialization2]) -> Self {
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
        Self { fragments }
    }

    /// Constructs a graph from already-retained fragments.
    pub const fn new(fragments: Vec<BezierArrangementFragment2>) -> Self {
        Self { fragments }
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
            let endpoints_for_fragment = match retained_endpoint_data(fragment.fragment(), policy) {
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

    let mut sorted = fragment_indices.to_vec();
    sorted.sort_unstable();
    if sorted.windows(2).any(|window| window[0] == window[1]) {
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
    fragment: &BezierSplitFragment2,
    policy: &CurvePolicy,
) -> Option<Classification<RetainedEndpointData>> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => match curve.endpoint_data(policy) {
            Classification::Decided(data) => Some(Classification::Decided(RetainedEndpointData {
                start: Some(RetainedEndpointKey::Exact(Box::new(data.start))),
                end: Some(RetainedEndpointKey::Exact(Box::new(data.end))),
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
            start_image,
            end_image,
            ..
        } => {
            let start = match start_image {
                Some(image) => match retained_algebraic_point_key(image.point()) {
                    Some(key) => Some(key),
                    None => return Some(Classification::Uncertain(UncertaintyReason::Boundary)),
                },
                None => None,
            };
            let end = match end_image {
                Some(image) => match retained_algebraic_point_key(image.point()) {
                    Some(key) => Some(key),
                    None => return Some(Classification::Uncertain(UncertaintyReason::Boundary)),
                },
                None => None,
            };
            let start_tangent = match start_image {
                Some(image) => match retained_algebraic_tangent(image.tangent()) {
                    Some(tangent) => Some(tangent),
                    None => return Some(Classification::Uncertain(UncertaintyReason::Boundary)),
                },
                None => None,
            };
            let end_tangent = match end_image {
                Some(image) => match retained_algebraic_tangent(image.tangent()) {
                    Some(tangent) => Some(tangent),
                    None => return Some(Classification::Uncertain(UncertaintyReason::Boundary)),
                },
                None => None,
            };
            let start_second_derivative = match start_image
                .as_ref()
                .and_then(|image| image.second_derivative())
            {
                Some(image) => match retained_algebraic_tangent(image) {
                    Some(tangent) => Some(tangent),
                    None => {
                        return Some(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                },
                None => None,
            };
            let start_third_derivative = match start_image
                .as_ref()
                .and_then(|image| image.third_derivative())
            {
                Some(image) => match retained_algebraic_tangent(image) {
                    Some(tangent) => Some(tangent),
                    None => {
                        return Some(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                },
                None => None,
            };
            Some(Classification::Decided(RetainedEndpointData {
                start,
                end,
                start_tangent,
                end_tangent,
                start_second_derivative,
                start_third_derivative,
            }))
        }
        BezierSplitFragment2::Unresolved { .. } => None,
    }
}

fn retained_algebraic_point_key(point: &BezierEndpointPointImage2) -> Option<RetainedEndpointKey> {
    let (x, y) = match point {
        BezierEndpointPointImage2::Polynomial(point) => (
            point.x()?.representation()?.clone(),
            point.y()?.representation()?.clone(),
        ),
        BezierEndpointPointImage2::RationalQuadratic(point) => (
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
            match retained_endpoints_equal(left_end, right_start, policy) {
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
            match retained_endpoints_equal(left_end, right_start, policy) {
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
                (Some(end), Some(start)) => match retained_endpoints_equal(end, start, policy) {
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
    for candidate in candidates {
        if endpoints[*candidate].start_tangent.is_none() {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
    }

    for candidate in candidates.iter().copied().skip(1) {
        let first = endpoints[candidate]
            .start_tangent
            .as_ref()
            .expect("candidate tangent checked above");
        let second = endpoints[best]
            .start_tangent
            .as_ref()
            .expect("candidate tangent checked above");
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
    left: &RetainedEndpointKey,
    right: &RetainedEndpointKey,
    policy: &CurvePolicy,
) -> Option<bool> {
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
        | (RetainedEndpointKey::Algebraic { x, y }, RetainedEndpointKey::Exact(point)) => {
            let x_witness = x.exact_rational_witness()?;
            let y_witness = y.exact_rational_witness()?;
            Some(
                compare_reals_equal(x_witness, point.x(), policy)?
                    && compare_reals_equal(y_witness, point.y(), policy)?,
            )
        }
    }
}

fn compare_reals_equal(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(crate::classify::compare_reals(left, right, policy)? == std::cmp::Ordering::Equal)
}

fn represented_roots_equal(
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
    // payloads. Subtract the represented roots and certify the sign of the
    // exact difference; this keeps endpoint gluing inside Yap's exact
    // construction/decision model instead of comparing interval samples.
    let difference = arithmetic_algebraic_root_representations(
        left,
        Some(right),
        AlgebraicRootArithmeticOp::Subtract,
    );
    if !matches!(
        difference.status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
    ) {
        return None;
    }
    if let Some(exact) = difference.exact_result.as_ref() {
        return compare_reals_equal(exact, &Real::zero(), policy);
    }
    let representation = difference.result_representation.as_ref()?;
    let lower =
        crate::classify::compare_reals(&representation.interval.lower, &Real::zero(), policy)?;
    let upper =
        crate::classify::compare_reals(&representation.interval.upper, &Real::zero(), policy)?;
    if lower == std::cmp::Ordering::Equal && upper == std::cmp::Ordering::Equal {
        Some(true)
    } else if matches!(lower, std::cmp::Ordering::Greater)
        || matches!(upper, std::cmp::Ordering::Less)
    {
        Some(false)
    } else {
        None
    }
}

impl BezierSubcurve2 {
    /// Returns the exact start and end points of this native subcurve.
    pub fn endpoints(&self) -> (Point2, Point2) {
        match self {
            Self::Quadratic(curve) => (curve.start().clone(), curve.end().clone()),
            Self::Cubic(curve) => (curve.start().clone(), curve.end().clone()),
            Self::RationalQuadratic(curve) => (curve.start().clone(), curve.end().clone()),
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
                    let (start_tangent, end_tangent) = rational_endpoint_tangents(curve);
                    let start_second_derivative = match rational_start_second_derivative(curve) {
                        Classification::Decided(derivative) => derivative,
                        Classification::Uncertain(reason) => {
                            return Classification::Uncertain(reason);
                        }
                    };
                    (
                        start_tangent,
                        end_tangent,
                        Some(start_second_derivative),
                        None,
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

/// Returns the affine second derivative at `t = 0` for a rational quadratic.
///
/// A conic Bezier is evaluated as a homogeneous quotient `R(t) = N(t) / W(t)`.
/// For same-tangent branch vertices we need the local Taylor coefficient used
/// by the signed-curvature witness in [`compare_same_tangent_second_order`].
/// The quotient derivative
/// `R'' = (N''W - NW'') / W^2 - 2W'(N'W - NW') / W^3` is evaluated exactly in
/// the scalar model; no floating approximation is introduced. The Bernstein
/// endpoint derivatives are the rational Bezier identities from Farin,
/// *Curves and Surfaces for CAGD* (5th ed., 2002), and refusing unsupported
/// divisions preserves Yap's exact-computation boundary from "Towards Exact
/// Geometric Computation," *Computational Geometry* 7(1-2), 3-23 (1997).
fn rational_start_second_derivative(
    curve: &crate::RationalQuadraticBezier2,
) -> Classification<TangentVector> {
    let dx = match rational_start_second_derivative_coordinate(
        curve.start().x(),
        curve.control().x(),
        curve.end().x(),
        curve.start_weight(),
        curve.control_weight(),
        curve.end_weight(),
    ) {
        Ok(value) => value,
        Err(reason) => return Classification::Uncertain(reason),
    };
    let dy = match rational_start_second_derivative_coordinate(
        curve.start().y(),
        curve.control().y(),
        curve.end().y(),
        curve.start_weight(),
        curve.control_weight(),
        curve.end_weight(),
    ) {
        Ok(value) => value,
        Err(reason) => return Classification::Uncertain(reason),
    };
    Classification::Decided(TangentVector { dx, dy })
}

fn rational_start_second_derivative_coordinate(
    p0: &Real,
    p1: &Real,
    p2: &Real,
    w0: &Real,
    w1: &Real,
    w2: &Real,
) -> Result<Real, UncertaintyReason> {
    let two = Real::from(2_i8);
    let n0 = w0 * p0;
    let n1 = &two * ((w1 * p1) - (w0 * p0));
    let n2 = &two * (((w0 * p0) - (&two * (w1 * p1))) + (w2 * p2));
    let d1 = &two * (w1 - w0);
    let d2 = &two * ((w0 - (&two * w1)) + w2);
    let w0_squared = w0 * w0;
    let w0_cubed = &w0_squared * w0;
    let first_numerator = (&n2 * w0) - (&n0 * &d2);
    let second_inner = (&n1 * w0) - (&n0 * &d1);
    let second_numerator = (&two * &d1) * second_inner;
    let first_term = (first_numerator / w0_squared).map_err(|_| UncertaintyReason::Unsupported)?;
    let second_term = (second_numerator / w0_cubed).map_err(|_| UncertaintyReason::Unsupported)?;
    Ok(first_term - second_term)
}

fn rational_endpoint_tangents(
    curve: &crate::RationalQuadraticBezier2,
) -> (TangentVector, TangentVector) {
    let two = Real::from(2_i8);
    let start_scale = &two * curve.start_weight() * curve.control_weight();
    let end_scale = &two * curve.control_weight() * curve.end_weight();
    let (start_dx, start_dy) = curve.control().delta_from(curve.start());
    let (end_dx, end_dy) = curve.end().delta_from(curve.control());
    (
        TangentVector {
            dx: &start_scale * start_dx,
            dy: &start_scale * start_dy,
        },
        TangentVector {
            dx: &end_scale * end_dx,
            dy: &end_scale * end_dy,
        },
    )
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
