//! Retained traversal graph for split Bezier and conic fragments.
//!
//! Higher-order booleans need an arrangement graph before they can emit
//! concrete regions. This module is a deliberately small, testable traversal
//! substrate for the fragments produced by [`BezierSplitMaterialization2`]: it
//! connects materialized Bezier/conic fragments by exact endpoint equality,
//! follows branch-free chains, and can optionally resolve simple branch
//! vertices by exact tangent angle order. It still refuses overlaps,
//! coincident tangents, zero tangents, and algebraic-boundary uncertainty.
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

use crate::classify::{is_zero, real_sign};
use crate::{
    BezierEndpoint, BezierSplitFragment2, BezierSplitMaterialization2, BezierSubcurve2,
    Classification, CurvePolicy, Point2, UncertaintyReason, ZeroStatus,
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

        Classification::Decided(BezierArrangementTraversal2::new(chains))
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

        Classification::Decided(BezierArrangementTraversal2::new(chains))
    }
}

impl BezierArrangementChain2 {
    /// Constructs a traversal chain from retained fragment indices.
    pub const fn new(fragment_indices: Vec<usize>, closed: bool) -> Self {
        Self {
            fragment_indices,
            closed,
        }
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
    pub const fn new(chains: Vec<BezierArrangementChain2>) -> Self {
        Self { chains }
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

fn materialized_endpoints(fragment: &BezierSplitFragment2) -> Option<(Point2, Point2)> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => Some(curve.endpoints()),
        BezierSplitFragment2::Unresolved { .. } => None,
    }
}

#[derive(Clone, Debug)]
struct EndpointData {
    start: Point2,
    end: Point2,
    start_tangent: TangentVector,
    end_tangent: TangentVector,
}

#[derive(Clone, Debug)]
struct TangentVector {
    dx: Real,
    dy: Real,
}

fn materialized_endpoint_data(
    fragment: &BezierSplitFragment2,
    policy: &CurvePolicy,
) -> Option<Classification<EndpointData>> {
    let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
        return None;
    };
    Some(curve.endpoint_data(policy))
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
            return Classification::Decided(BezierArrangementChain2::new(indices, closed));
        };
        current = next;
        if current == start {
            return Classification::Decided(BezierArrangementChain2::new(indices, true));
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
            return Classification::Decided(BezierArrangementChain2::new(indices, closed));
        };

        current = next;
        if current == start {
            return Classification::Decided(BezierArrangementChain2::new(indices, true));
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
                return Classification::Uncertain(UncertaintyReason::Boundary);
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
        let (start_tangent, end_tangent) = match self {
            Self::Quadratic(curve) => (
                TangentVector::from_endpoint_tangent(curve.endpoint_tangent(BezierEndpoint::Start)),
                TangentVector::from_endpoint_tangent(curve.endpoint_tangent(BezierEndpoint::End)),
            ),
            Self::Cubic(curve) => (
                TangentVector::from_endpoint_tangent(curve.endpoint_tangent(BezierEndpoint::Start)),
                TangentVector::from_endpoint_tangent(curve.endpoint_tangent(BezierEndpoint::End)),
            ),
            Self::RationalQuadratic(curve) => rational_endpoint_tangents(curve),
        };

        if !start_tangent.is_nonzero(policy) || !end_tangent.is_nonzero(policy) {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }

        Classification::Decided(EndpointData {
            start,
            end,
            start_tangent,
            end_tangent,
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
