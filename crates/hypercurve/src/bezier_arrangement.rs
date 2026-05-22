//! Retained traversal graph for split Bezier and conic fragments.
//!
//! Higher-order booleans need an arrangement graph before they can emit
//! concrete regions. This module is a deliberately small, testable traversal
//! substrate for the fragments produced by [`BezierSplitMaterialization2`]: it
//! connects materialized Bezier/conic fragments by exact endpoint equality,
//! follows branch-free chains, and refuses to choose a successor at branch,
//! overlap, or algebraic-boundary uncertainty.
//!
//! That boundary is the exact-computation discipline described by Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 3-23 (1997): topology code may retain unresolved exact objects, but it must
//! not invent a floating successor. The branch-free chain walk mirrors the
//! regularized graph assumption in Greiner and Hormann, "Efficient clipping of
//! arbitrary polygons," *ACM Transactions on Graphics* 17(2), 71-83 (1998),
//! while the refusal at multi-successor vertices follows the degeneracy split
//! emphasized by Foster, Hormann, and Popa, "Clipping simple polygons with
//! degenerate intersections," *Computers & Graphics: X* 2, 100007 (2019).

use crate::classify::is_zero;
use crate::{
    BezierSplitFragment2, BezierSplitMaterialization2, BezierSubcurve2, Classification,
    CurvePolicy, Point2, UncertaintyReason,
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

fn endpoint_adjacency(
    endpoints: &[(Point2, Point2)],
    policy: &CurvePolicy,
) -> Classification<(Vec<Option<usize>>, Vec<Option<usize>>)> {
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
}
