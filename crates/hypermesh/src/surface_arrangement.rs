//! Exact source-face corefinement, radial cells, winding truth, and output.
//!
//! This is the single production Boolean kernel. It consumes canonical
//! `hyperreal::Real` geometry through one operation-wide predicate policy and
//! materializes one or many results from a shared exact arrangement.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlattice::{Point3, Real, Vector3};
use hyperreal::Rational;

use crate::bvh::{ExactBvh, ExactBvhQueryHierarchy};
use crate::context::{DecisionContext, MeshCertainty};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, axis_mut, axis_ref, compare_real_decision};
use crate::intersection::{
    PairwiseIntersectionEventIds, PairwiseIntersectionGraph, pairwise_support_identity,
    source_face_pair_key,
};
use crate::output::{
    BooleanMeshBatch, BooleanMeshClosureEvidence, BooleanMeshResult, TriangleSource,
};
use crate::point_interner::PointInterner;
use crate::polygon::{
    ApproxBounds, ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity,
    ConvexPolygon,
};
use crate::predicate::{
    classify_point_decision, classify_projective_point_decision, classify_real,
    exact_rational_points_contradict,
};
use crate::storage_hash::StorageHashMap;
use crate::winding::{BooleanExpression, BooleanProgram};

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum ArrangementPointIdentity {
    Construction(ConstructionVertexIdentity),
    CoplanarEdges([ConstructionEdgeIdentity; 2]),
}

#[derive(Clone)]
struct RawConstraint {
    endpoints: [u32; 2],
    line: ConstructionEdgeIdentity,
}

#[derive(Default)]
struct FaceWork {
    boundary: [u32; 3],
    // Source-boundary constraints are implied by this triangle and its
    // retained edge identities. Store only additional arrangement work so an
    // untouched source face needs no face-local heap allocation.
    constraints: Vec<RawConstraint>,
    contacts: Vec<u32>,
}

impl FaceWork {
    fn is_changed(&self) -> bool {
        !self.constraints.is_empty() || !self.contacts.is_empty()
    }
}

struct ArrangementPointArena {
    points: Vec<Point3>,
    structural: StorageHashMap<ArrangementPointIdentity, u32>,
    source_edge_points: Vec<(u32, [u32; 2], u32)>,
    numeric: PointInterner<()>,
}

impl ArrangementPointArena {
    fn with_capacity(capacity: usize) -> HypermeshResult<Self> {
        let mut points = Vec::new();
        points
            .try_reserve_exact(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point arena",
            })?;
        let mut structural = StorageHashMap::default();
        structural
            .try_reserve(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity index",
            })?;
        Ok(Self {
            points,
            structural,
            source_edge_points: Vec::new(),
            // Exact-rational points have a complete identity/fingerprint
            // schedule and need no interval grid. The interner promotes all
            // retained points to its policy-aware general schedule on the
            // first non-rational coordinate.
            numeric: PointInterner::try_with_capacity(capacity, true, false)?,
        })
    }

    fn insert(
        &mut self,
        decisions: &DecisionContext,
        identity: ArrangementPointIdentity,
        point: Point3,
    ) -> HypermeshResult<u32> {
        if let Some(&existing) = self.structural.get(&identity) {
            if exact_rational_points_contradict(&self.points[existing as usize], &point) {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "one construction identity materialized at contradictory points",
                });
            }
            return Ok(existing);
        }

        self.structural
            .try_reserve(1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity index",
            })?;
        let index = self
            .numeric
            .intern_owned(decisions, &mut self.points, point, None)?;
        let compact = u32::try_from(index).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement point arena",
        })?;
        self.structural.insert(identity, compact);
        Ok(compact)
    }

    fn retained_point(&self, identity: &ArrangementPointIdentity) -> Option<u32> {
        self.structural.get(identity).copied()
    }

    fn retain_overlay_source_edge_memberships(
        &mut self,
        identity: &ArrangementPointIdentity,
        point: u32,
    ) -> HypermeshResult<()> {
        let source_edge_count = match identity {
            ArrangementPointIdentity::Construction(
                ConstructionVertexIdentity::SourceEdgePlane { .. },
            ) => 1,
            ArrangementPointIdentity::CoplanarEdges(edges) => edges
                .iter()
                .filter(|edge| matches!(edge, ConstructionEdgeIdentity::Source { .. }))
                .count(),
            ArrangementPointIdentity::Construction(
                ConstructionVertexIdentity::Source { .. }
                | ConstructionVertexIdentity::PlaneTriple { .. },
            ) => 0,
        };
        self.source_edge_points
            .try_reserve(source_edge_count)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "retained source-edge point schedule",
            })?;
        match identity {
            ArrangementPointIdentity::Construction(
                ConstructionVertexIdentity::SourceEdgePlane {
                    mesh, endpoints, ..
                },
            ) => self.source_edge_points.push((*mesh, *endpoints, point)),
            ArrangementPointIdentity::CoplanarEdges(edges) => {
                for edge in edges {
                    if let ConstructionEdgeIdentity::Source { mesh, endpoints } = edge {
                        self.source_edge_points.push((*mesh, *endpoints, point));
                    }
                }
            }
            ArrangementPointIdentity::Construction(
                ConstructionVertexIdentity::Source { .. }
                | ConstructionVertexIdentity::PlaneTriple { .. },
            ) => {}
        }
        Ok(())
    }
}

#[derive(Debug)]
struct SurfaceCorefinement {
    points: Vec<Point3>,
    face_offsets: Box<[u32]>,
    triangles: Vec<[u32; 3]>,
    #[cfg(test)]
    constraint_offsets: Box<[u32]>,
    #[cfg(test)]
    constraints: Vec<[u32; 2]>,
    #[cfg(test)]
    contact_offsets: Box<[u32]>,
    #[cfg(test)]
    contacts: Vec<u32>,
}

impl SurfaceCorefinement {
    fn face_triangles(&self, face: usize) -> &[[u32; 3]] {
        let start = self.face_offsets[face] as usize;
        let end = self.face_offsets[face + 1] as usize;
        &self.triangles[start..end]
    }

    #[cfg(test)]
    fn face_constraints(&self, face: usize) -> &[[u32; 2]] {
        let start = self.constraint_offsets[face] as usize;
        let end = self.constraint_offsets[face + 1] as usize;
        &self.constraints[start..end]
    }

    #[cfg(test)]
    fn face_contacts(&self, face: usize) -> &[u32] {
        let start = self.contact_offsets[face] as usize;
        let end = self.contact_offsets[face + 1] as usize;
        &self.contacts[start..end]
    }
}

const FRONT: usize = 0;
const BACK: usize = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct FacetContribution {
    face: u32,
    orientation: i8,
}

#[derive(Clone, Copy, Debug)]
struct SurfaceFacet {
    vertices: [u32; 3],
    cells: [u32; 2],
}

#[derive(Debug)]
struct SurfaceCellComplex {
    facets: Vec<SurfaceFacet>,
    contribution_offsets: Box<[u32]>,
    contributions: Vec<FacetContribution>,
    #[cfg(test)]
    transitions: Box<[i32]>,
    cell_windings: Box<[i32]>,
    operand_count: usize,
    cell_count: u32,
    #[cfg(test)]
    component_count: u32,
    #[cfg(test)]
    radial_edge_count: u32,
    #[cfg(test)]
    max_radial_degree: u32,
}

type CellTruthNode = BooleanExpression;

#[derive(Debug, Eq, PartialEq)]
struct ExpressionClassifications {
    expression_count: usize,
    facet_count: usize,
    classifications: Vec<i8>,
    exterior_inside: Vec<bool>,
}

impl ExpressionClassifications {
    #[cfg(test)]
    fn classification(&self, expression: usize, facet: usize) -> i8 {
        self.classifications[expression * self.facet_count + facet]
    }
}

impl SurfaceCellComplex {
    #[cfg(test)]
    fn facet_contributions(&self, facet: usize) -> &[FacetContribution] {
        let start = self.contribution_offsets[facet] as usize;
        let end = self.contribution_offsets[facet + 1] as usize;
        &self.contributions[start..end]
    }

    fn checked_facet_contributions(&self, facet: usize) -> HypermeshResult<&[FacetContribution]> {
        checked_contribution_row(&self.contribution_offsets, &self.contributions, facet)
    }

    #[cfg(test)]
    fn facet_transition(&self, facet: usize) -> &[i32] {
        let start = facet * self.operand_count;
        &self.transitions[start..start + self.operand_count]
    }

    fn cell_winding(&self, cell: u32) -> &[i32] {
        let start = cell as usize * self.operand_count;
        &self.cell_windings[start..start + self.operand_count]
    }

    fn facet_classification(&self, facet: usize, operation: crate::winding::BooleanOp) -> i8 {
        let cells = self.facets[facet].cells;
        crate::winding::classify_polygon_output(
            self.cell_winding(cells[FRONT]),
            self.cell_winding(cells[BACK]),
            operation,
        )
    }

    fn classify_expressions(
        &self,
        nodes: &[CellTruthNode],
        roots: &[u32],
    ) -> HypermeshResult<ExpressionClassifications> {
        validate_cell_truth_program(nodes, roots, self.operand_count)?;
        let truth_len = (self.cell_count as usize).checked_mul(roots.len()).ok_or(
            HypermeshError::CapacityOverflow {
                operation: "surface cell expression truth table",
            },
        )?;
        let mut root_truth = Vec::new();
        root_truth
            .try_reserve_exact(truth_len)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface cell expression truth table",
            })?;
        root_truth.resize(truth_len, 0_u8);
        let mut node_truth = Vec::new();
        node_truth.try_reserve_exact(nodes.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface cell expression node state",
            }
        })?;
        node_truth.resize(nodes.len(), 0_u8);
        for cell in 0..self.cell_count {
            let winding = self.cell_winding(cell);
            evaluate_cell_truth_nodes(nodes, winding, &mut node_truth);
            let start = cell as usize * roots.len();
            for (expression, root) in roots.iter().copied().enumerate() {
                root_truth[start + expression] = node_truth[root as usize];
            }
        }

        let classification_len =
            roots
                .len()
                .checked_mul(self.facets.len())
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "surface facet expression classifications",
                })?;
        let mut classifications = Vec::new();
        classifications
            .try_reserve_exact(classification_len)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface facet expression classifications",
            })?;
        for expression in 0..roots.len() {
            for facet in &self.facets {
                let front = root_truth[facet.cells[FRONT] as usize * roots.len() + expression] != 0;
                let back = root_truth[facet.cells[BACK] as usize * roots.len() + expression] != 0;
                classifications.push(match (front, back) {
                    (false, true) => 1,
                    (true, false) => -1,
                    _ => 0,
                });
            }
        }
        let mut zero_winding = Vec::new();
        zero_winding
            .try_reserve_exact(self.operand_count)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface exterior winding state",
            })?;
        zero_winding.resize(self.operand_count, 0_i32);
        evaluate_cell_truth_nodes(nodes, &zero_winding, &mut node_truth);
        let mut exterior_inside = Vec::new();
        exterior_inside
            .try_reserve_exact(roots.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface output exterior classifications",
            })?;
        exterior_inside.extend(roots.iter().map(|root| node_truth[*root as usize] != 0));
        Ok(ExpressionClassifications {
            expression_count: roots.len(),
            facet_count: self.facets.len(),
            classifications,
            exterior_inside,
        })
    }
}

fn evaluate_cell_truth_nodes(nodes: &[CellTruthNode], winding: &[i32], truth: &mut [u8]) {
    for (node, instruction) in nodes.iter().enumerate() {
        truth[node] = match *instruction {
            CellTruthNode::False => 0,
            CellTruthNode::True => 1,
            CellTruthNode::Operand(operand) => u8::from(winding[operand as usize] != 0),
            CellTruthNode::Not(input) => 1 - truth[input as usize],
            CellTruthNode::And([left, right]) => truth[left as usize] & truth[right as usize],
            CellTruthNode::Or([left, right]) => truth[left as usize] | truth[right as usize],
            CellTruthNode::Xor([left, right]) => truth[left as usize] ^ truth[right as usize],
            CellTruthNode::Operation(operation) => u8::from(operation.contains(winding)),
        };
    }
}

fn validate_cell_truth_program(
    nodes: &[CellTruthNode],
    roots: &[u32],
    operand_count: usize,
) -> HypermeshResult<()> {
    if operand_count == 0 {
        return Err(HypermeshError::InvalidBooleanProgram {
            reason: "at least one operand is required",
        });
    }
    if roots.is_empty() {
        return Err(HypermeshError::InvalidBooleanProgram {
            reason: "at least one output root is required",
        });
    }
    for (node, instruction) in nodes.iter().enumerate() {
        let dependency_is_valid = |dependency: u32| (dependency as usize) < node;
        let valid = match *instruction {
            CellTruthNode::False | CellTruthNode::True => true,
            CellTruthNode::Operand(operand) => (operand as usize) < operand_count,
            CellTruthNode::Operation(_) => operand_count != 0,
            CellTruthNode::Not(input) => dependency_is_valid(input),
            CellTruthNode::And([left, right])
            | CellTruthNode::Or([left, right])
            | CellTruthNode::Xor([left, right]) => {
                dependency_is_valid(left) && dependency_is_valid(right)
            }
        };
        if !valid {
            return Err(HypermeshError::InvalidBooleanProgram {
                reason: "a node references an absent operand or non-earlier dependency",
            });
        }
    }
    if roots.iter().any(|root| *root as usize >= nodes.len()) {
        return Err(HypermeshError::InvalidBooleanProgram {
            reason: "an output references an absent root",
        });
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct PendingFacet {
    vertices: [u32; 3],
}

#[derive(Clone, Copy)]
struct PendingContribution {
    facet: u32,
    contribution: FacetContribution,
}

#[derive(Clone, Copy)]
struct EdgeUse {
    edge: [u32; 2],
    facet: u32,
    opposite: u32,
}

#[derive(Clone, Copy)]
struct RadialUse {
    facet: u32,
    opposite: u32,
    half: u8,
}

struct CellDisjointSets {
    parents: Vec<u32>,
}

impl CellDisjointSets {
    fn new(node_count: usize) -> HypermeshResult<Self> {
        if node_count > u32::MAX as usize {
            return Err(HypermeshError::CapacityOverflow {
                operation: "surface cell side nodes",
            });
        }
        Ok(Self {
            parents: (0..node_count as u32).collect(),
        })
    }

    fn find(&mut self, node: usize) -> usize {
        let mut root = node;
        while self.parents[root] as usize != root {
            root = self.parents[root] as usize;
        }
        let mut cursor = node;
        while self.parents[cursor] as usize != root {
            let next = self.parents[cursor] as usize;
            self.parents[cursor] = root as u32;
            cursor = next;
        }
        root
    }

    fn union(&mut self, left: usize, right: usize) {
        let left = self.find(left);
        let right = self.find(right);
        if left == right {
            return;
        }
        let (root, child) = if left < right {
            (left, right)
        } else {
            (right, left)
        };
        self.parents[child] = root as u32;
    }

    fn into_cells(mut self) -> HypermeshResult<(Vec<u32>, u32)> {
        for node in 0..self.parents.len() {
            self.find(node);
        }
        let mut root_cells = vec![u32::MAX; self.parents.len()];
        let mut cell_count = 0_u32;
        for node in 0..self.parents.len() {
            let root = self.parents[node] as usize;
            if root_cells[root] == u32::MAX {
                root_cells[root] = cell_count;
                cell_count = cell_count
                    .checked_add(1)
                    .ok_or(HypermeshError::CapacityOverflow {
                        operation: "surface arrangement cells",
                    })?;
            }
            self.parents[node] = root_cells[root];
        }
        Ok((self.parents, cell_count))
    }
}

fn assemble_surface_cells(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    surface: &SurfaceCorefinement,
    radially_separated_face_pair_keys: &[u64],
    source_bvh: &ExactBvhQueryHierarchy,
) -> HypermeshResult<SurfaceCellComplex> {
    if surface.face_offsets.len() != polygons.len().saturating_add(1) {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface corefinement topology is not aligned with its sources",
        });
    }
    let operand_count = polygons.first().map_or(0, |polygon| polygon.delta_w.len());
    if polygons
        .iter()
        .any(|polygon| polygon.delta_w.len() != operand_count)
    {
        return Err(HypermeshError::WindingDimensionMismatch {
            expected: operand_count,
            actual: polygons
                .iter()
                .find(|polygon| polygon.delta_w.len() != operand_count)
                .map_or(operand_count, |polygon| polygon.delta_w.len()),
        });
    }
    if radially_separated_face_pair_keys.iter().any(|&pair| {
        let first = pair >> u32::BITS;
        let second = pair as u32;
        first >= u64::from(second) || second as usize >= polygons.len()
    }) || radially_separated_face_pair_keys
        .windows(2)
        .any(|pairs| pairs[0] >= pairs[1])
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "radially separated source-face pairs are not canonical",
        });
    }

    let (pending, contribution_offsets, contributions, transitions) =
        bundle_surface_facets(polygons, surface, operand_count)?;
    if pending.is_empty() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface corefinement contains no facets",
        });
    }

    let side_count = pending
        .len()
        .checked_mul(2)
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "surface cell side nodes",
        })?;
    let mut sets = CellDisjointSets::new(side_count)?;
    let edge_use_capacity =
        pending
            .len()
            .checked_mul(3)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "surface radial edge uses",
            })?;
    let mut edge_uses = Vec::new();
    edge_uses
        .try_reserve_exact(edge_use_capacity)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface radial edge uses",
        })?;
    for (facet, pending_facet) in pending.iter().enumerate() {
        let facet = compact_len(facet, "surface radial facet IDs")?;
        let [a, b, c] = pending_facet.vertices;
        for (start, end, opposite) in [(a, b, c), (b, c, a), (c, a, b)] {
            edge_uses.push(EdgeUse {
                edge: sorted_edge([start, end]),
                facet,
                opposite,
            });
        }
    }
    edge_uses.sort_unstable_by_key(|edge_use| {
        [
            edge_use.edge[0],
            edge_use.edge[1],
            edge_use.facet,
            edge_use.opposite,
        ]
    });

    let mut edges = Vec::new();
    let mut radial = Vec::<RadialUse>::new();
    let mut ray_starts = Vec::<usize>::new();
    let mut edge_start = 0;
    #[cfg(test)]
    let mut max_radial_degree = 0_u32;
    while edge_start < edge_uses.len() {
        let edge = edge_uses[edge_start].edge;
        let mut edge_end = edge_start + 1;
        while edge_end < edge_uses.len() && edge_uses[edge_end].edge == edge {
            edge_end += 1;
        }
        edges.push(edge);
        #[cfg(test)]
        {
            max_radial_degree =
                max_radial_degree.max(compact_len(edge_end - edge_start, "surface radial degree")?);
        }
        let uses = &edge_uses[edge_start..edge_end];
        if let [first, second] = uses {
            let retained_separation = facets_have_retained_radial_separation(
                first.facet,
                second.facet,
                &contribution_offsets,
                &contributions,
                radially_separated_face_pair_keys,
            )?;
            if !retained_separation
                && same_radial_ray(
                    decisions,
                    &surface.points,
                    edge,
                    first.opposite,
                    second.opposite,
                )?
            {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "two-facet radial edge has one geometric ray",
                });
            }
            let first_after = facet_side_node(&pending, first.facet, edge, true)?;
            let first_before = facet_side_node(&pending, first.facet, edge, false)?;
            let second_after = facet_side_node(&pending, second.facet, edge, true)?;
            let second_before = facet_side_node(&pending, second.facet, edge, false)?;
            sets.union(first_after, second_before);
            sets.union(second_after, first_before);
        } else if !try_assemble_two_face_transverse_ring(
            decisions,
            &surface.points,
            &pending,
            edge,
            uses,
            &contribution_offsets,
            &contributions,
            &mut sets,
        )? {
            assemble_radial_ring(
                decisions,
                &surface.points,
                &pending,
                edge,
                uses,
                &mut radial,
                &mut ray_starts,
                &mut sets,
            )?;
        }
        edge_start = edge_end;
    }

    let (side_cells, cell_count) = sets.into_cells()?;
    let facets = pending
        .into_iter()
        .enumerate()
        .map(|(facet, pending)| SurfaceFacet {
            vertices: pending.vertices,
            cells: [side_cells[facet * 2], side_cells[facet * 2 + 1]],
        })
        .collect::<Vec<_>>();
    let bounds = surface_bounds(decisions, &surface.points)?;
    let (cell_windings, _component_count) = classify_surface_cells(
        decisions,
        polygons,
        surface,
        source_bvh,
        &bounds,
        &surface.points,
        &facets,
        &transitions,
        &contribution_offsets,
        &contributions,
        operand_count,
        cell_count,
        &edges,
    )?;

    Ok(SurfaceCellComplex {
        facets,
        contribution_offsets,
        contributions,
        #[cfg(test)]
        transitions: transitions.into_boxed_slice(),
        cell_windings: cell_windings.into_boxed_slice(),
        operand_count,
        cell_count,
        #[cfg(test)]
        component_count: _component_count,
        #[cfg(test)]
        radial_edge_count: compact_len(edges.len(), "surface radial edge count")?,
        #[cfg(test)]
        max_radial_degree,
    })
}

fn bundle_surface_facets(
    polygons: &[ConvexPolygon],
    surface: &SurfaceCorefinement,
    operand_count: usize,
) -> HypermeshResult<(
    Vec<PendingFacet>,
    Box<[u32]>,
    Vec<FacetContribution>,
    Vec<i32>,
)> {
    let transition_capacity = surface.triangles.len().checked_mul(operand_count).ok_or(
        HypermeshError::CapacityOverflow {
            operation: "surface facet winding transitions",
        },
    )?;
    let mut facet_lookup = StorageHashMap::<[u32; 3], u32>::default();
    facet_lookup
        .try_reserve(surface.triangles.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface facet bundle index",
        })?;
    let mut facets = Vec::<PendingFacet>::new();
    let mut transitions = Vec::<i32>::new();
    transitions
        .try_reserve_exact(transition_capacity)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface facet winding transitions",
        })?;
    let mut pending_contributions = Vec::<PendingContribution>::new();
    pending_contributions
        .try_reserve_exact(surface.triangles.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface facet contributions",
        })?;

    for (face, polygon) in polygons.iter().enumerate() {
        for &triangle in surface.face_triangles(face) {
            if triangle
                .iter()
                .any(|&point| point as usize >= surface.points.len())
            {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface facet references an absent arrangement point",
                });
            }
            let mut canonical = triangle;
            canonical.sort_unstable();
            if canonical[0] == canonical[1] || canonical[1] == canonical[2] {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface corefinement contains a degenerate facet",
                });
            }
            let orientation = triangle_orientation(triangle, canonical)?;
            let facet = if let Some(&facet) = facet_lookup.get(&canonical) {
                facet
            } else {
                let facet = compact_len(facets.len(), "surface facet bundle IDs")?;
                facets.push(PendingFacet {
                    vertices: canonical,
                });
                transitions.extend(std::iter::repeat_n(0, operand_count));
                facet_lookup.insert(canonical, facet);
                facet
            };
            let start = facet as usize * operand_count;
            for (component, delta) in polygon.delta_w.iter().copied().enumerate() {
                let signed = i32::from(orientation)
                    .checked_mul(delta)
                    .ok_or(HypermeshError::WindingOverflow)?;
                transitions[start + component] = transitions[start + component]
                    .checked_add(signed)
                    .ok_or(HypermeshError::WindingOverflow)?;
            }
            pending_contributions.push(PendingContribution {
                facet,
                contribution: FacetContribution {
                    face: compact_len(face, "surface facet source face IDs")?,
                    orientation,
                },
            });
        }
    }

    pending_contributions.sort_unstable_by_key(|pending| {
        (
            pending.facet,
            pending.contribution.face,
            pending.contribution.orientation,
        )
    });
    let mut contribution_offsets = Vec::new();
    contribution_offsets
        .try_reserve_exact(facets.len().saturating_add(1))
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface facet contribution offsets",
        })?;
    let mut contributions = Vec::new();
    contributions
        .try_reserve_exact(pending_contributions.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface facet contributions",
        })?;
    contribution_offsets.push(0_u32);
    let mut next = 0;
    for facet in 0..facets.len() {
        while next < pending_contributions.len()
            && pending_contributions[next].facet as usize == facet
        {
            contributions.push(pending_contributions[next].contribution);
            next += 1;
        }
        contribution_offsets.push(compact_len(
            contributions.len(),
            "surface facet contribution offsets",
        )?);
    }
    if next != pending_contributions.len()
        || contribution_offsets.len() != facets.len().saturating_add(1)
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface facet contributions are incomplete",
        });
    }
    Ok((
        facets,
        contribution_offsets.into_boxed_slice(),
        contributions,
        transitions,
    ))
}

fn triangle_orientation(source: [u32; 3], canonical: [u32; 3]) -> HypermeshResult<i8> {
    if source == canonical
        || source == [canonical[1], canonical[2], canonical[0]]
        || source == [canonical[2], canonical[0], canonical[1]]
    {
        Ok(1)
    } else if source == [canonical[0], canonical[2], canonical[1]]
        || source == [canonical[2], canonical[1], canonical[0]]
        || source == [canonical[1], canonical[0], canonical[2]]
    {
        Ok(-1)
    } else {
        Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface facet orientation does not match its vertex set",
        })
    }
}

fn assemble_radial_ring(
    decisions: &DecisionContext,
    points: &[Point3],
    facets: &[PendingFacet],
    edge: [u32; 2],
    uses: &[EdgeUse],
    radial: &mut Vec<RadialUse>,
    ray_starts: &mut Vec<usize>,
    sets: &mut CellDisjointSets,
) -> HypermeshResult<()> {
    radial.clear();
    radial
        .try_reserve(uses.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface radial ring scratch",
        })?;
    for edge_use in uses {
        radial.push(RadialUse {
            facet: edge_use.facet,
            opposite: edge_use.opposite,
            half: 0,
        });
    }
    let reference = radial[0].opposite;
    for radial_use in radial.iter_mut().skip(1) {
        radial_use.half = radial_half(decisions, points, edge, reference, radial_use.opposite)?;
    }
    let mut failure = None;
    radial.sort_unstable_by(|left, right| {
        if left.half != right.half {
            return left.half.cmp(&right.half);
        }
        if failure.is_some() {
            return Ordering::Equal;
        }
        match compare_radial_rays(decisions, points, edge, left.opposite, right.opposite) {
            Ok(ordering) => ordering,
            Err(error) => {
                failure = Some(error);
                Ordering::Equal
            }
        }
    });
    if let Some(error) = failure {
        return Err(error);
    }

    ray_starts.clear();
    ray_starts.push(0);
    for index in 1..radial.len() {
        if !same_radial_ray(
            decisions,
            points,
            edge,
            radial[index - 1].opposite,
            radial[index].opposite,
        )? {
            ray_starts.push(index);
        }
    }
    if ray_starts.len() < 2 {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface radial edge has fewer than two geometric rays",
        });
    }

    for ray in 0..ray_starts.len() {
        let start = ray_starts[ray];
        let end = ray_starts.get(ray + 1).copied().unwrap_or(radial.len());
        let base_after = facet_side_node(facets, radial[start].facet, edge, true)?;
        let base_before = facet_side_node(facets, radial[start].facet, edge, false)?;
        for radial_use in &radial[start + 1..end] {
            sets.union(
                base_after,
                facet_side_node(facets, radial_use.facet, edge, true)?,
            );
            sets.union(
                base_before,
                facet_side_node(facets, radial_use.facet, edge, false)?,
            );
        }
    }
    for ray in 0..ray_starts.len() {
        let next = (ray + 1) % ray_starts.len();
        let after = facet_side_node(facets, radial[ray_starts[ray]].facet, edge, true)?;
        let before = facet_side_node(facets, radial[ray_starts[next]].facet, edge, false)?;
        sets.union(after, before);
    }
    Ok(())
}

fn try_assemble_two_face_transverse_ring(
    decisions: &DecisionContext,
    points: &[Point3],
    facets: &[PendingFacet],
    edge: [u32; 2],
    uses: &[EdgeUse],
    contribution_offsets: &[u32],
    contributions: &[FacetContribution],
    sets: &mut CellDisjointSets,
) -> HypermeshResult<bool> {
    // A validated planar triangulation has at most two facets from one source
    // face on an edge, and two such facets occupy antipodal radial rays. When
    // four unbundled facets split evenly between two source faces, one
    // nonzero orientation therefore proves the complete alternating cycle:
    // `a, b, -a, -b`. Retain that topology instead of rediscovering the other
    // three relations and sorting four rays. Coplanar, coincident, bundled,
    // or higher-degree incidence declines to the complete exact ring path.
    let [_, _, _, _] = uses else {
        return Ok(false);
    };
    let mut face_ids = [u32::MAX; 2];
    let mut face_uses = [[None; 2]; 2];
    let mut face_counts = [0_usize; 2];
    for &edge_use in uses {
        let [contribution] =
            checked_contribution_row(contribution_offsets, contributions, edge_use.facet as usize)?
        else {
            return Ok(false);
        };
        let face_slot = if face_ids[0] == contribution.face {
            0
        } else if face_ids[0] == u32::MAX {
            face_ids[0] = contribution.face;
            0
        } else if face_ids[1] == contribution.face {
            1
        } else if face_ids[1] == u32::MAX {
            face_ids[1] = contribution.face;
            1
        } else {
            return Ok(false);
        };
        let use_slot = face_counts[face_slot];
        if use_slot >= 2 {
            return Ok(false);
        }
        face_uses[face_slot][use_slot] = Some(edge_use);
        face_counts[face_slot] += 1;
    }
    if face_counts != [2, 2] {
        return Ok(false);
    }
    let [
        [Some(first), Some(first_opposite)],
        [Some(second), Some(second_opposite)],
    ] = face_uses
    else {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "transverse radial ring grouping is incomplete",
        });
    };
    let second_half = match radial_triple_classification(
        decisions,
        points,
        edge,
        first.opposite,
        second.opposite,
    )? {
        Classification::Positive => [second, second_opposite],
        Classification::Negative => [second_opposite, second],
        Classification::On => return Ok(false),
    };
    let ring = [first, second_half[0], first_opposite, second_half[1]];
    for index in 0..ring.len() {
        let next = (index + 1) % ring.len();
        let after = facet_side_node(facets, ring[index].facet, edge, true)?;
        let before = facet_side_node(facets, ring[next].facet, edge, false)?;
        sets.union(after, before);
    }
    Ok(true)
}

fn checked_contribution_row<'a>(
    contribution_offsets: &[u32],
    contributions: &'a [FacetContribution],
    facet: usize,
) -> HypermeshResult<&'a [FacetContribution]> {
    const MALFORMED: HypermeshError = HypermeshError::SurfaceArrangementFailed {
        reason: "surface output facet contribution storage is malformed",
    };
    let start = contribution_offsets.get(facet).copied().ok_or(MALFORMED)? as usize;
    let end = facet
        .checked_add(1)
        .and_then(|terminal| contribution_offsets.get(terminal))
        .copied()
        .ok_or(MALFORMED)? as usize;
    if start > end {
        return Err(MALFORMED);
    }
    contributions.get(start..end).ok_or(MALFORMED)
}

fn facets_have_retained_radial_separation(
    left: u32,
    right: u32,
    contribution_offsets: &[u32],
    contributions: &[FacetContribution],
    radially_separated_face_pair_keys: &[u64],
) -> HypermeshResult<bool> {
    // A bundled arrangement facet may carry several coincident source-face
    // contributions. A shared validated face or retained adjacent source pair
    // proves separation only for the degree-two edge currently being
    // assembled; higher-degree radial rings use their dedicated transverse
    // proof above or continue through the complete exact ordering path.
    let left = checked_contribution_row(contribution_offsets, contributions, left as usize)?;
    let right = checked_contribution_row(contribution_offsets, contributions, right as usize)?;
    for left in left {
        for right in right {
            // Two distinct facets emitted by one validated planar
            // triangulation lie on opposite sides of their shared edge. That
            // source-face incidence is already a stronger radial-separation
            // proof than recomputing equality from materialized coordinates.
            if left.face == right.face {
                return Ok(true);
            }
            let Some(pair) = source_face_pair_key(left.face, right.face) else {
                continue;
            };
            if radially_separated_face_pair_keys
                .binary_search(&pair)
                .is_ok()
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn facet_side_node(
    facets: &[PendingFacet],
    facet: u32,
    edge: [u32; 2],
    after: bool,
) -> HypermeshResult<usize> {
    let vertices = facets
        .get(facet as usize)
        .ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "surface radial ring references an absent facet",
        })?
        .vertices;
    let mut directed_forward = None;
    for index in 0..3 {
        let from = vertices[index];
        let to = vertices[(index + 1) % 3];
        if sorted_edge([from, to]) == edge {
            directed_forward = Some(from == edge[0]);
            break;
        }
    }
    let directed_forward = directed_forward.ok_or(HypermeshError::SurfaceArrangementFailed {
        reason: "surface radial facet does not contain its indexed edge",
    })?;
    let after_side = if directed_forward { FRONT } else { BACK };
    let side = if after { after_side } else { 1 - after_side };
    (facet as usize)
        .checked_mul(2)
        .and_then(|node| node.checked_add(side))
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "surface cell side node IDs",
        })
}

fn radial_half(
    decisions: &DecisionContext,
    points: &[Point3],
    edge: [u32; 2],
    reference: u32,
    candidate: u32,
) -> HypermeshResult<u8> {
    match radial_triple_classification(decisions, points, edge, reference, candidate)? {
        Classification::Positive => Ok(0),
        Classification::Negative => Ok(1),
        Classification::On => {
            match radial_dot_classification(decisions, points, edge, reference, candidate)? {
                Classification::Positive => Ok(0),
                Classification::Negative => Ok(1),
                Classification::On => Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface radial ray is degenerate on its edge",
                }),
            }
        }
    }
}

fn compare_radial_rays(
    decisions: &DecisionContext,
    points: &[Point3],
    edge: [u32; 2],
    left: u32,
    right: u32,
) -> HypermeshResult<Ordering> {
    match radial_triple_classification(decisions, points, edge, left, right)? {
        Classification::Positive => Ok(Ordering::Less),
        Classification::Negative => Ok(Ordering::Greater),
        Classification::On => {
            match radial_dot_classification(decisions, points, edge, left, right)? {
                Classification::Positive => Ok(Ordering::Equal),
                Classification::Negative => Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "antipodal surface rays occupied one angular half",
                }),
                Classification::On => Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface radial ray is degenerate on its edge",
                }),
            }
        }
    }
}

fn same_radial_ray(
    decisions: &DecisionContext,
    points: &[Point3],
    edge: [u32; 2],
    left: u32,
    right: u32,
) -> HypermeshResult<bool> {
    if radial_triple_classification(decisions, points, edge, left, right)? != Classification::On {
        return Ok(false);
    }
    match radial_dot_classification(decisions, points, edge, left, right)? {
        Classification::Positive => Ok(true),
        Classification::Negative => Ok(false),
        Classification::On => Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface radial ray is degenerate on its edge",
        }),
    }
}

fn radial_triple_classification(
    decisions: &DecisionContext,
    points: &[Point3],
    edge: [u32; 2],
    left: u32,
    right: u32,
) -> HypermeshResult<Classification> {
    let origin = point_by_id(points, edge[0])?;
    let endpoint = point_by_id(points, edge[1])?;
    let left = point_by_id(points, left)?;
    let right = point_by_id(points, right)?;
    decisions
        .decide(
            hyperlimit::orient3(origin, endpoint, left, right, decisions.policy()),
            "surface radial orientation",
        )
        .map(|sign| match sign {
            // Hyperlimit's affine orientation is det(a-d, b-d, c-d),
            // the negative of edge dot (left cross right) for this ordering.
            hyperlimit::Sign::Negative => Classification::Positive,
            hyperlimit::Sign::Zero => Classification::On,
            hyperlimit::Sign::Positive => Classification::Negative,
        })
}

fn radial_dot_classification(
    decisions: &DecisionContext,
    points: &[Point3],
    edge: [u32; 2],
    left: u32,
    right: u32,
) -> HypermeshResult<Classification> {
    let origin = point_by_id(points, edge[0])?;
    let direction = point_by_id(points, edge[1])? - origin;
    let left = point_by_id(points, left)? - origin;
    let right = point_by_id(points, right)? - origin;
    if let Some(ordering) = exact_radial_perpendicular_dot_ordering(&direction, &left, &right) {
        return Ok(match ordering {
            Ordering::Less => Classification::Negative,
            Ordering::Equal => Classification::On,
            Ordering::Greater => Classification::Positive,
        });
    }
    let perpendicular_dot =
        direction.dot(&direction) * left.dot(&right) - direction.dot(&left) * direction.dot(&right);
    classify_real(decisions, &perpendicular_dot)
}

fn exact_radial_perpendicular_dot_ordering(
    direction: &Vector3,
    left: &Vector3,
    right: &Vector3,
) -> Option<Ordering> {
    let [dx, dy, dz] = [&direction.0[0], &direction.0[1], &direction.0[2]];
    let [lx, ly, lz] = [&left.0[0], &left.0[1], &left.0[2]];
    let [rx, ry, rz] = [&right.0[0], &right.0[1], &right.0[2]];
    let [dx, dy, dz] = [
        dx.exact_rational_ref()?,
        dy.exact_rational_ref()?,
        dz.exact_rational_ref()?,
    ];
    let [lx, ly, lz] = [
        lx.exact_rational_ref()?,
        ly.exact_rational_ref()?,
        lz.exact_rational_ref()?,
    ];
    let [rx, ry, rz] = [
        rx.exact_rational_ref()?,
        ry.exact_rational_ref()?,
        rz.exact_rational_ref()?,
    ];
    // This predicate needs only the sign of `(direction cross left) dot
    // (direction cross right)`. Retain exact-rational scalar facts long enough
    // for Hyperreal to schedule the whole polynomial and compare its signed
    // magnitudes without materializing or reducing an otherwise dead `Real`.
    Some(Rational::signed_product_sum_ordering(
        [
            true, true, false, false, true, true, false, false, true, true, false, false,
        ],
        [
            [dx, dx, ly, ry],
            [dy, dy, lx, rx],
            [dx, dy, lx, ry],
            [dx, dy, ly, rx],
            [dx, dx, lz, rz],
            [dz, dz, lx, rx],
            [dx, dz, lx, rz],
            [dx, dz, lz, rx],
            [dy, dy, lz, rz],
            [dz, dz, ly, ry],
            [dy, dz, ly, rz],
            [dy, dz, lz, ry],
        ],
    ))
}

fn point_by_id(points: &[Point3], point: u32) -> HypermeshResult<&Point3> {
    points
        .get(point as usize)
        .ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "surface topology references an absent arrangement point",
        })
}

fn surface_bounds(decisions: &DecisionContext, points: &[Point3]) -> HypermeshResult<ApproxBounds> {
    let first = points
        .first()
        .ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "surface cell complex has no arrangement points",
        })?;
    let mut minimum = first.clone();
    let mut maximum = first.clone();
    for point in &points[1..] {
        for axis in 0..3 {
            if compare_real_decision(decisions, axis_ref(point, axis), axis_ref(&minimum, axis))?
                .is_lt()
            {
                *axis_mut(&mut minimum, axis) = axis_ref(point, axis).clone();
            }
            if compare_real_decision(decisions, axis_ref(point, axis), axis_ref(&maximum, axis))?
                .is_gt()
            {
                *axis_mut(&mut maximum, axis) = axis_ref(point, axis).clone();
            }
        }
    }
    Ok(ApproxBounds::new(minimum, maximum))
}

fn classify_surface_cells(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    surface: &SurfaceCorefinement,
    source_bvh: &ExactBvhQueryHierarchy,
    bounds: &ApproxBounds,
    points: &[Point3],
    facets: &[SurfaceFacet],
    transitions: &[i32],
    contribution_offsets: &[u32],
    contributions: &[FacetContribution],
    operand_count: usize,
    cell_count: u32,
    edges: &[[u32; 2]],
) -> HypermeshResult<(Vec<i32>, u32)> {
    if transitions.len() != facets.len().saturating_mul(operand_count) {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface facet transition dimensions are incomplete",
        });
    }
    let incidence_count = facets
        .len()
        .checked_mul(2)
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "surface cell incidences",
        })?;
    if incidence_count > u32::MAX as usize {
        return Err(HypermeshError::CapacityOverflow {
            operation: "surface cell incidences",
        });
    }
    let mut heads = vec![u32::MAX; cell_count as usize];
    let mut next = vec![u32::MAX; incidence_count];
    for (facet, surface_facet) in facets.iter().enumerate() {
        for side in [FRONT, BACK] {
            let cell = surface_facet.cells[side] as usize;
            let head = heads
                .get_mut(cell)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface facet references an absent cell",
                })?;
            let incidence = facet * 2 + side;
            next[incidence] = *head;
            *head = incidence as u32;
        }
    }

    let winding_len = (cell_count as usize).checked_mul(operand_count).ok_or(
        HypermeshError::CapacityOverflow {
            operation: "surface cell winding vectors",
        },
    )?;
    let mut windings = vec![0_i32; winding_len];
    let mut visited = vec![false; cell_count as usize];
    let mut queue = Vec::<u32>::new();
    let mut component_count = 0_u32;
    for cell in 0..cell_count {
        if visited[cell as usize] {
            continue;
        }
        let incidence = heads[cell as usize];
        if incidence == u32::MAX {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface cell has no incident facet",
            });
        }
        let seed_facet = incidence as usize / 2;
        let (seed_cell, seed_winding) = seed_surface_cell_winding(
            decisions,
            polygons,
            surface,
            source_bvh,
            bounds,
            points,
            facets,
            contribution_offsets,
            contributions,
            operand_count,
            edges,
            seed_facet,
        )?;
        if visited[seed_cell as usize] {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface cell component seed aliases an earlier component",
            });
        }
        let seed_start = seed_cell as usize * operand_count;
        windings[seed_start..seed_start + operand_count].copy_from_slice(&seed_winding);
        visited[seed_cell as usize] = true;
        queue.clear();
        queue.push(seed_cell);
        let mut queue_head = 0;
        while queue_head < queue.len() {
            let current = queue[queue_head];
            queue_head += 1;
            let mut incidence = heads[current as usize];
            while incidence != u32::MAX {
                let facet = incidence as usize / 2;
                let side = incidence as usize & 1;
                let neighbor = facets[facet].cells[1 - side];
                let sign = if side == FRONT { 1_i32 } else { -1_i32 };
                let current_start = current as usize * operand_count;
                let neighbor_start = neighbor as usize * operand_count;
                let transition_start = facet * operand_count;
                for component in 0..operand_count {
                    let delta = sign
                        .checked_mul(transitions[transition_start + component])
                        .ok_or(HypermeshError::WindingOverflow)?;
                    let expected = windings[current_start + component]
                        .checked_add(delta)
                        .ok_or(HypermeshError::WindingOverflow)?;
                    if visited[neighbor as usize]
                        && windings[neighbor_start + component] != expected
                    {
                        return Err(HypermeshError::SurfaceArrangementFailed {
                            reason: "surface cell winding propagation is inconsistent",
                        });
                    }
                }
                if !visited[neighbor as usize] {
                    for component in 0..operand_count {
                        let delta = sign
                            .checked_mul(transitions[transition_start + component])
                            .expect("surface winding arithmetic was checked above");
                        windings[neighbor_start + component] = windings[current_start + component]
                            .checked_add(delta)
                            .expect("surface winding arithmetic was checked above");
                    }
                    visited[neighbor as usize] = true;
                    queue.push(neighbor);
                }
                incidence = next[incidence as usize];
            }
        }
        component_count =
            component_count
                .checked_add(1)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "surface cell components",
                })?;
    }
    Ok((windings, component_count))
}

fn seed_surface_cell_winding(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    surface: &SurfaceCorefinement,
    source_bvh: &ExactBvhQueryHierarchy,
    bounds: &ApproxBounds,
    points: &[Point3],
    facets: &[SurfaceFacet],
    contribution_offsets: &[u32],
    contributions: &[FacetContribution],
    operand_count: usize,
    edges: &[[u32; 2]],
    seed_facet: usize,
) -> HypermeshResult<(u32, Vec<i32>)> {
    let triangle = facets
        .get(seed_facet)
        .ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "surface cell seed references an absent facet",
        })?
        .vertices;
    let point = surface_facet_centroid(points, triangle)?;
    let constraint_count = surface.triangles.len().checked_add(edges.len()).ok_or(
        HypermeshError::CapacityOverflow {
            operation: "surface cell seed direction constraints",
        },
    )?;
    let candidate_count = constraint_count
        .checked_mul(2)
        .and_then(|count| count.checked_add(1))
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "surface cell seed direction candidates",
        })?;
    let mut saw_unknown = false;
    let mut try_direction =
        |primary_axis: usize, direction: &Vector3| -> HypermeshResult<Option<(u32, Vec<i32>)>> {
            let local = decisions.isolated();
            match try_seed_surface_cell_winding(
                &local,
                polygons,
                source_bvh,
                bounds,
                primary_axis,
                points,
                facets,
                contribution_offsets,
                contributions,
                operand_count,
                seed_facet,
                &point,
                direction,
            ) {
                Ok(Some(result)) => {
                    decisions.absorb(local.certainty());
                    Ok(Some(result))
                }
                Ok(None) => Ok(None),
                Err(HypermeshError::PredicateUndecided { .. }) => {
                    saw_unknown = true;
                    Ok(None)
                }
                Err(error) => Err(error),
            }
        };
    let axes = seed_ray_axis_order(bounds, &point);
    for primary_axis in axes {
        let mut coordinates = [Real::zero(), Real::zero(), Real::zero()];
        coordinates[primary_axis] = Real::one();
        let direction = Vector3::new(coordinates);
        if let Some(result) = try_direction(primary_axis, &direction)? {
            return Ok(result);
        }
    }

    let primary_axis = axes[0];
    let linear_axis = (primary_axis + 1) % 3;
    let quadratic_axis = (primary_axis + 2) % 3;
    for candidate in 1..=candidate_count {
        let parameter =
            Real::from(
                u64::try_from(candidate).map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "surface cell seed direction parameter",
                })?,
            );
        let mut coordinates = [Real::zero(), Real::zero(), Real::zero()];
        coordinates[primary_axis] = Real::one();
        coordinates[linear_axis] = parameter.clone();
        coordinates[quadratic_axis] = parameter.clone() * parameter;
        let direction = Vector3::new(coordinates);
        if let Some(result) = try_direction(primary_axis, &direction)? {
            return Ok(result);
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "surface cell seed direction separation",
        })
    } else {
        Err(HypermeshError::SurfaceArrangementFailed {
            reason: "finite exact seed direction family did not avoid every facet and edge",
        })
    }
}

fn seed_ray_axis_order(bounds: &ApproxBounds, point: &Point3) -> [usize; 3] {
    let mut axes = [0_usize, 1, 2];
    axes.sort_unstable_by(|&left, &right| {
        approximate_positive_axis_distance(bounds, point, left)
            .total_cmp(&approximate_positive_axis_distance(bounds, point, right))
            .then_with(|| left.cmp(&right))
    });
    axes
}

fn approximate_positive_axis_distance(bounds: &ApproxBounds, point: &Point3, axis: usize) -> f64 {
    let distance = axis_ref(&bounds.max, axis)
        .to_f64_lossy()
        .zip(axis_ref(point, axis).to_f64_lossy())
        .map(|(maximum, coordinate)| maximum - coordinate);
    match distance {
        Some(distance) if distance.is_finite() && distance >= 0.0 => distance,
        _ => f64::INFINITY,
    }
}

fn surface_facet_centroid(points: &[Point3], triangle: [u32; 3]) -> HypermeshResult<Point3> {
    let [a, b, c] = triangle
        .map(|point| point_by_id(points, point))
        .map(|point| point.cloned());
    let [a, b, c] = [a?, b?, c?];
    let denominator = Real::from(3_u8);
    let coordinate = |a: Real, b: Real, c: Real| {
        ((a + b + c) / denominator.clone()).map_err(|_| HypermeshError::SurfaceArrangementFailed {
            reason: "surface facet centroid denominator is not invertible",
        })
    };
    Ok(Point3::new(
        coordinate(a.x, b.x, c.x)?,
        coordinate(a.y, b.y, c.y)?,
        coordinate(a.z, b.z, c.z)?,
    ))
}

fn try_seed_surface_cell_winding(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    source_bvh: &ExactBvhQueryHierarchy,
    bounds: &ApproxBounds,
    primary_axis: usize,
    points: &[Point3],
    facets: &[SurfaceFacet],
    contribution_offsets: &[u32],
    contributions: &[FacetContribution],
    operand_count: usize,
    seed_facet: usize,
    point: &Point3,
    direction: &Vector3,
) -> HypermeshResult<Option<(u32, Vec<i32>)>> {
    let seed = &facets[seed_facet];
    let frontward = match ray_facet_relation(decisions, points, seed.vertices, point, direction)? {
        RayFacetRelation::Origin { frontward } => frontward,
        RayFacetRelation::Degenerate => return Ok(None),
        RayFacetRelation::Miss | RayFacetRelation::Ahead { .. } => {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface seed does not lie inside its source facet",
            });
        }
    };
    let distance = axis_ref(&bounds.max, primary_axis).clone()
        - axis_ref(point, primary_axis).clone()
        + Real::one();
    let endpoint = Point3::new(
        point.x.clone() + distance.clone() * direction.0[0].clone(),
        point.y.clone() + distance.clone() * direction.0[1].clone(),
        point.z.clone() + distance * direction.0[2].clone(),
    );
    let bounds = ApproxBounds::new(point.clone(), endpoint);
    let mut candidate_faces = Vec::new();
    source_bvh.query_bounds_decision(decisions, polygons, &bounds, |face| {
        candidate_faces.push(face)
    })?;

    let mut winding = vec![0_i32; operand_count];
    let mut saw_origin = false;
    let seed_contributions =
        checked_contribution_row(contribution_offsets, contributions, seed_facet)?;
    for face in candidate_faces {
        let polygon = polygons
            .get(face)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "surface seed broad phase returned an absent source face",
            })?;
        // Absolute winding belongs to the original PWN boundary. Corefinement
        // only subdivides that boundary for topology, so an exact ray crosses
        // each convex source triangle at most once and applies its transition
        // once. Testing the subdivisions here would also make their artificial
        // internal edges unnecessary ray degeneracies.
        match ray_source_polygon_relation(decisions, polygon, point, direction)? {
            RayFacetRelation::Miss => {}
            RayFacetRelation::Degenerate => return Ok(None),
            RayFacetRelation::Ahead { frontward } => {
                crate::winding::apply_transition_in_place(
                    &mut winding,
                    if frontward { 1 } else { -1 },
                    &polygon.delta_w,
                )?;
            }
            RayFacetRelation::Origin { .. } => {
                if !seed_contributions
                    .iter()
                    .any(|contribution| contribution.face as usize == face)
                {
                    return Err(HypermeshError::SurfaceArrangementFailed {
                        reason: "surface seed lies inside more than one geometric facet",
                    });
                }
                saw_origin = true;
            }
        }
    }
    if !saw_origin {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface seed broad phase omitted its source facet",
        });
    }
    Ok(Some((
        seed.cells[if frontward { FRONT } else { BACK }],
        winding,
    )))
}

enum RayFacetRelation {
    Miss,
    Degenerate,
    Ahead { frontward: bool },
    Origin { frontward: bool },
}

fn ray_facet_relation(
    decisions: &DecisionContext,
    points: &[Point3],
    triangle: [u32; 3],
    origin: &Point3,
    direction: &Vector3,
) -> HypermeshResult<RayFacetRelation> {
    let [a, b, c] = triangle.map(|vertex| point_by_id(points, vertex));
    ray_triangle_relation(decisions, [a?, b?, c?], origin, direction)
}

fn ray_source_polygon_relation(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    origin: &Point3,
    direction: &Vector3,
) -> HypermeshResult<RayFacetRelation> {
    let vertices =
        polygon
            .known_vertices
            .as_ref()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no retained vertex cycle",
            })?;
    if vertices.len() != 3 {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "source face retained vertex cycle is not triangular",
        });
    }
    let [a, b, c] = [vertices.get(0), vertices.get(1), vertices.get(2)];
    let [a, b, c] = [a, b, c].map(|vertex| {
        vertex.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "source face retained vertex cycle is incomplete",
        })
    });
    ray_triangle_relation(decisions, [a?, b?, c?], origin, direction)
}

fn ray_triangle_relation(
    decisions: &DecisionContext,
    [a, b, c]: [&Point3; 3],
    origin: &Point3,
    direction: &Vector3,
) -> HypermeshResult<RayFacetRelation> {
    let edge_ab = b - a;
    let edge_ac = c - a;
    let cross = direction.cross(&edge_ac);
    let determinant = edge_ab.dot(&cross);
    let determinant_sign = classify_real(decisions, &determinant)?;
    if determinant_sign == Classification::On {
        return Ok(RayFacetRelation::Degenerate);
    }
    let from_a = origin - a;
    let u = from_a.dot(&cross);
    let u_sign = classify_real(decisions, &u)?;
    let cross_from_a = from_a.cross(&edge_ab);
    let v = direction.dot(&cross_from_a);
    let v_sign = classify_real(decisions, &v)?;
    let remainder = determinant.clone() - u - v;
    let remainder_sign = classify_real(decisions, &remainder)?;
    if [u_sign, v_sign, remainder_sign]
        .into_iter()
        .any(|sign| sign != Classification::On && sign != determinant_sign)
    {
        return Ok(RayFacetRelation::Miss);
    }
    if [u_sign, v_sign, remainder_sign]
        .into_iter()
        .any(|sign| sign == Classification::On)
    {
        return Ok(RayFacetRelation::Degenerate);
    }
    let ray_parameter = edge_ac.dot(&cross_from_a);
    let parameter_sign = classify_real(decisions, &ray_parameter)?;
    let frontward = determinant_sign == Classification::Negative;
    if parameter_sign == Classification::On {
        Ok(RayFacetRelation::Origin { frontward })
    } else if parameter_sign == determinant_sign {
        Ok(RayFacetRelation::Ahead { frontward })
    } else {
        Ok(RayFacetRelation::Miss)
    }
}

fn corefine_surface(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    intersections: &PairwiseIntersectionGraph,
) -> HypermeshResult<SurfaceCorefinement> {
    if intersections.len() != polygons.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "intersection graph and source-face counts differ",
        });
    }
    let initial_points = {
        let mut identities = StorageHashMap::<ConstructionVertexIdentity, ()>::default();
        for polygon in polygons {
            if let Some(vertices) = polygon.known_vertex_identities() {
                for identity in vertices {
                    if !identities.contains_key(&identity) {
                        identities.try_reserve(1).map_err(|_| {
                            HypermeshError::CapacityOverflow {
                                operation: "surface arrangement source identity count",
                            }
                        })?;
                        identities.insert(identity, ());
                    }
                }
            }
        }
        identities
            .len()
            .checked_add(intersections.construction_point_count())
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "surface arrangement initial points",
            })?
    };
    let mut arena = ArrangementPointArena::with_capacity(initial_points)?;
    let mut work = Vec::new();
    work.try_reserve_exact(polygons.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face work",
        })?;
    for polygon in polygons {
        work.push(FaceWork {
            boundary: add_source_boundary(decisions, polygon, &mut arena)?,
            ..FaceWork::default()
        });
    }
    // Pairwise point IDs are already operation-wide aliases. Translate each
    // one only after source vertices establish identity precedence, then
    // release this transient table before per-face triangulation.
    let point_count = intersections.construction_point_count();
    let mut point_map = Vec::new();
    point_map
        .try_reserve_exact(point_count)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement graph point remap",
        })?;
    point_map.resize(point_count, None);
    append_intersection_constraints(
        decisions,
        polygons,
        intersections,
        &mut point_map,
        &mut arena,
        &mut work,
    )?;
    drop(point_map);
    propagate_retained_source_edge_points(polygons, &mut arena, &mut work)?;

    let offset_capacity =
        polygons
            .len()
            .checked_add(1)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "surface arrangement face offsets",
            })?;
    let mut face_offsets = Vec::new();
    face_offsets
        .try_reserve_exact(offset_capacity)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face offsets",
        })?;
    face_offsets.push(0_u32);
    #[cfg(test)]
    let mut constraint_offsets = Vec::new();
    #[cfg(test)]
    let mut contact_offsets = Vec::new();
    #[cfg(test)]
    for offsets in [&mut constraint_offsets, &mut contact_offsets] {
        offsets.try_reserve_exact(offset_capacity).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement face offsets",
            }
        })?;
        offsets.push(0_u32);
    }
    let mut triangles = Vec::new();
    #[cfg(test)]
    let mut constraints = Vec::new();
    #[cfg(test)]
    let mut contacts = Vec::new();
    for (face, (polygon, face_work)) in polygons.iter().zip(work).enumerate() {
        let result = corefine_face(decisions, face, polygon, &face_work, &mut arena)?;
        triangles.try_reserve(result.triangles.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement triangles",
            }
        })?;
        #[cfg(test)]
        {
            constraints
                .try_reserve(result.constraints.len())
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "surface arrangement constraints",
                })?;
            contacts.try_reserve(result.contacts.len()).map_err(|_| {
                HypermeshError::CapacityOverflow {
                    operation: "surface arrangement contacts",
                }
            })?;
        }
        triangles.extend(result.triangles);
        #[cfg(test)]
        {
            constraints.extend(result.constraints);
            contacts.extend(result.contacts);
        }
        face_offsets.push(compact_len(
            triangles.len(),
            "surface arrangement triangle offsets",
        )?);
        #[cfg(test)]
        {
            constraint_offsets.push(compact_len(
                constraints.len(),
                "surface arrangement constraint offsets",
            )?);
            contact_offsets.push(compact_len(
                contacts.len(),
                "surface arrangement contact offsets",
            )?);
        }
    }
    Ok(SurfaceCorefinement {
        points: arena.points,
        face_offsets: face_offsets.into_boxed_slice(),
        triangles,
        #[cfg(test)]
        constraint_offsets: constraint_offsets.into_boxed_slice(),
        #[cfg(test)]
        constraints,
        #[cfg(test)]
        contact_offsets: contact_offsets.into_boxed_slice(),
        #[cfg(test)]
        contacts,
    })
}

fn compact_len(len: usize, operation: &'static str) -> HypermeshResult<u32> {
    u32::try_from(len).map_err(|_| HypermeshError::CapacityOverflow { operation })
}

fn add_source_boundary(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    arena: &mut ArrangementPointArena,
) -> HypermeshResult<[u32; 3]> {
    let vertices =
        polygon
            .known_vertices
            .as_ref()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no retained vertex cycle",
            })?;
    let vertex_identities =
        polygon
            .known_vertex_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no canonical vertex identities",
            })?;
    let edge_identities =
        polygon
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no canonical edge identities",
            })?;
    if vertices.len() != 3
        || vertices.len() != vertex_identities.len()
        || vertices.len() != edge_identities.len()
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "source face geometry and identity cycles are not aligned",
        });
    }
    let mut boundary = [0; 3];
    for (slot, (point, identity)) in vertices.iter().zip(vertex_identities).enumerate() {
        boundary[slot] = arena.insert(
            decisions,
            ArrangementPointIdentity::Construction(identity),
            point.clone(),
        )?;
    }
    Ok(boundary)
}

fn append_intersection_constraints(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    intersections: &PairwiseIntersectionGraph,
    point_map: &mut [Option<u32>],
    arena: &mut ArrangementPointArena,
    work: &mut [FaceWork],
) -> HypermeshResult<()> {
    for face in 0..polygons.len() {
        for event in intersections.event_ids(face)? {
            match event? {
                PairwiseIntersectionEventIds::NonCoplanarPoint {
                    point,
                    other_polygon: _,
                }
                | PairwiseIntersectionEventIds::CoplanarPoint {
                    point,
                    other_polygon: _,
                } => {
                    let point = map_graph_point(decisions, intersections, point_map, arena, point)?;
                    work[face].contacts.push(point);
                }
                PairwiseIntersectionEventIds::NonCoplanarSegment {
                    endpoints,
                    other_polygon,
                } => {
                    let endpoints = endpoints.map(|point| {
                        map_graph_point(decisions, intersections, point_map, arena, point)
                    });
                    let endpoints = [endpoints[0].clone()?, endpoints[1].clone()?];
                    work[face].constraints.push(RawConstraint {
                        endpoints,
                        line: pairwise_split_line(face, other_polygon as usize)?,
                    });
                }
                PairwiseIntersectionEventIds::CoplanarSegment {
                    endpoints,
                    other_polygon: _,
                } => {
                    let first = intersections.construction_point(endpoints[0])?.0;
                    let second = intersections.construction_point(endpoints[1])?.0;
                    let line =
                        boundary_line_for_segment(decisions, &polygons[face], first, second)?;
                    let endpoints = [
                        map_graph_point(decisions, intersections, point_map, arena, endpoints[0])?,
                        map_graph_point(decisions, intersections, point_map, arena, endpoints[1])?,
                    ];
                    work[face]
                        .constraints
                        .push(RawConstraint { endpoints, line });
                }
                PairwiseIntersectionEventIds::CoplanarOverlap { other_polygon } => {
                    let other = other_polygon as usize;
                    if face >= other {
                        continue;
                    }
                    let overlay = coplanar_overlay(
                        decisions,
                        face,
                        &polygons[face],
                        other,
                        polygons
                            .get(other)
                            .ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "coplanar overlap references an absent face",
                            })?,
                    )?;
                    let mut point_ids = Vec::new();
                    point_ids.try_reserve_exact(overlay.len()).map_err(|_| {
                        HypermeshError::CapacityOverflow {
                            operation: "coplanar overlay point IDs",
                        }
                    })?;
                    for vertex in &overlay {
                        let point = arena.insert(
                            decisions,
                            vertex.identity.clone(),
                            vertex.point.clone(),
                        )?;
                        // Pairwise-graph points are already attached to every
                        // incident face. Overlay vertices are synthesized at
                        // this later stage, so fan out their retained authored
                        // source-edge membership after all overlays are known.
                        arena.retain_overlay_source_edge_memberships(&vertex.identity, point)?;
                        point_ids.push(point);
                    }
                    for index in 0..overlay.len() {
                        let constraint = RawConstraint {
                            endpoints: [point_ids[index], point_ids[(index + 1) % overlay.len()]],
                            line: overlay[index].outgoing.clone(),
                        };
                        work[face].constraints.push(constraint.clone());
                        work[other].constraints.push(constraint);
                    }
                }
            }
        }
    }
    Ok(())
}

fn propagate_retained_source_edge_points(
    polygons: &[ConvexPolygon],
    arena: &mut ArrangementPointArena,
    work: &mut [FaceWork],
) -> HypermeshResult<()> {
    let mut source_edge_points = std::mem::take(&mut arena.source_edge_points);
    source_edge_points.sort_unstable();
    source_edge_points.dedup();
    if source_edge_points.is_empty() {
        return Ok(());
    }

    if polygons.len() != work.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "source polygons and face work differ",
        });
    }
    for (polygon, face_work) in polygons.iter().zip(work) {
        let boundary = face_work.boundary;
        let edge_identities =
            polygon
                .known_edge_identities()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "source face has no canonical edge identities",
                })?;
        if edge_identities.len() != boundary.len() {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "source face boundary and edge identities are not aligned",
            });
        }
        let mut inserted = false;
        for edge in 0..boundary.len() {
            let line =
                edge_identities
                    .get(edge)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "source edge identity cycle is incomplete",
                    })?;
            let ConstructionEdgeIdentity::Source { mesh, endpoints } = line else {
                continue;
            };
            let key = (mesh, endpoints);
            let edge_boundary = [boundary[edge], boundary[(edge + 1) % boundary.len()]];
            let start =
                source_edge_points.partition_point(|&(candidate_mesh, candidate_endpoints, _)| {
                    (candidate_mesh, candidate_endpoints) < key
                });
            let end =
                source_edge_points.partition_point(|&(candidate_mesh, candidate_endpoints, _)| {
                    (candidate_mesh, candidate_endpoints) <= key
                });
            let retained = &source_edge_points[start..end];
            let additional = retained
                .iter()
                .filter(|(_, _, point)| !edge_boundary.contains(point))
                .count();
            if additional != 0 {
                face_work.contacts.try_reserve(additional).map_err(|_| {
                    HypermeshError::CapacityOverflow {
                        operation: "retained source-edge face schedule",
                    }
                })?;
                face_work.contacts.extend(
                    retained
                        .iter()
                        .map(|&(_, _, point)| point)
                        .filter(|point| !edge_boundary.contains(point)),
                );
                inserted = true;
            }
        }
        if inserted {
            face_work.contacts.sort_unstable();
            face_work.contacts.dedup();
        }
    }
    Ok(())
}

fn map_graph_point(
    decisions: &DecisionContext,
    graph: &PairwiseIntersectionGraph,
    point_map: &mut [Option<u32>],
    arena: &mut ArrangementPointArena,
    point: u32,
) -> HypermeshResult<u32> {
    let remapped =
        point_map
            .get_mut(point as usize)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "intersection event references an absent construction point",
            })?;
    if let Some(point) = *remapped {
        return Ok(point);
    }
    let (materialized, identity) = graph.construction_point(point)?;
    let point = arena.insert(
        decisions,
        ArrangementPointIdentity::Construction(identity.clone()),
        materialized.clone(),
    )?;
    *remapped = Some(point);
    Ok(point)
}

fn pairwise_split_line(face: usize, other: usize) -> HypermeshResult<ConstructionEdgeIdentity> {
    let mut planes = [
        pairwise_support_identity(face)?.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "source face has no operation-local support identity",
        })?,
        pairwise_support_identity(other)?.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "intersecting face has no operation-local support identity",
        })?,
    ];
    planes.sort_unstable();
    if planes[0] == planes[1] {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "non-coplanar cut has one support-plane identity",
        });
    }
    Ok(ConstructionEdgeIdentity::Split { planes })
}

fn boundary_line_for_segment(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    first: &Point3,
    second: &Point3,
) -> HypermeshResult<ConstructionEdgeIdentity> {
    let identities =
        polygon
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "coplanar contact face has no edge identities",
            })?;
    let mut found = None;
    let edge_planes = polygon.edge_planes();
    for edge in 0..edge_planes.len() {
        let plane = &edge_planes[(edge + 1) % edge_planes.len()];
        if classify_point_decision(decisions, first, plane)? == Classification::On
            && classify_point_decision(decisions, second, plane)? == Classification::On
        {
            let identity =
                identities
                    .get(edge)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "coplanar contact edge identity is absent",
                    })?;
            if found.as_ref().is_none_or(|existing| identity < *existing) {
                found = Some(identity);
            }
        }
    }
    found.ok_or(HypermeshError::SurfaceArrangementFailed {
        reason: "coplanar contact segment is not on a source edge",
    })
}

#[derive(Clone)]
struct OverlayVertex {
    point: Point3,
    identity: ArrangementPointIdentity,
    outgoing: ConstructionEdgeIdentity,
}

struct IdentifiedPolygon {
    polygon: ConvexPolygon,
    plane_edges: Vec<ConstructionEdgeIdentity>,
}

impl IdentifiedPolygon {
    fn from_source(polygon: &ConvexPolygon) -> HypermeshResult<Self> {
        let outgoing =
            polygon
                .known_edge_identities()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "coplanar overlap face has no edge identities",
                })?;
        let edge_planes = polygon.edge_planes();
        if edge_planes.len() != outgoing.len() || edge_planes.len() < 3 {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "coplanar overlap edge planes and identities are not aligned",
            });
        }
        let mut plane_edges = Vec::new();
        plane_edges.try_reserve_exact(outgoing.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "coplanar overlay edge identities",
            }
        })?;
        for plane in 0..outgoing.len() {
            // ConvexPolygon stores source edge i and its half-space plane at
            // the same index. Its projective vertex i is the intersection of
            // planes i and i + 1, so retaining that direct alignment is what
            // lets a clipped vertex recover the correct construction recipe.
            plane_edges.push(outgoing.get(plane).ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "coplanar overlap edge identity is absent",
                },
            )?);
        }
        let mut polygon = polygon.clone();
        // Retained source vertices follow the source cycle, whereas the
        // projective representation names vertex i by edge planes i and i+1.
        // A clip clears the retained cycle; clear it up front as well so a
        // no-op (coincident/contained) overlay uses exactly the same indexing.
        polygon.clear_known_vertices();
        polygon.known_identities = None;
        polygon.approx_bounds = None;
        Ok(Self {
            polygon,
            plane_edges,
        })
    }

    fn clip_negative(
        mut self,
        decisions: &DecisionContext,
        plane: &Plane,
        plane_edge: ConstructionEdgeIdentity,
    ) -> HypermeshResult<Option<Self>> {
        let count = self.polygon.vertex_count();
        let mut classifications = Vec::new();
        classifications
            .try_reserve_exact(count)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar overlay classifications",
            })?;
        let mut has_negative = false;
        let mut has_positive = false;
        for vertex in 0..count {
            let classification =
                classify_projective_point_decision(decisions, &self.polygon.vertex(vertex), plane)?;
            has_negative |= classification == Classification::Negative;
            has_positive |= classification == Classification::Positive;
            classifications.push(classification);
        }
        if !has_positive {
            return Ok(Some(self));
        }
        if !has_negative {
            return Ok(None);
        }

        let mut edges = Vec::new();
        let mut identities = Vec::new();
        edges
            .try_reserve(count + 1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar overlay edge planes",
            })?;
        identities
            .try_reserve(count + 1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar overlay edge identities",
            })?;
        let polygon_edges = self.polygon.edge_planes();
        for index in 0..count {
            let next = (index + 1) % count;
            let segment_plane = polygon_edges[next].clone();
            let segment_identity = self.plane_edges[next].clone();
            match (
                classifications[index].is_non_positive(),
                classifications[next].is_non_positive(),
            ) {
                (true, true) | (false, true) => {
                    edges.push(segment_plane);
                    identities.push(segment_identity);
                }
                (true, false) => {
                    edges.push(segment_plane);
                    identities.push(segment_identity);
                    edges.push(plane.clone());
                    identities.push(plane_edge.clone());
                }
                (false, false) => {}
            }
        }
        if edges.len() < 3 || edges.len() != identities.len() {
            return Ok(None);
        }
        self.polygon.replace_edge_planes(edges);
        self.polygon.clear_known_vertices();
        self.polygon.known_identities = None;
        self.polygon.approx_bounds = None;
        self.plane_edges = identities;
        Ok(Some(self))
    }
}

fn coplanar_overlay(
    decisions: &DecisionContext,
    left_face: usize,
    left: &ConvexPolygon,
    _right_face: usize,
    right: &ConvexPolygon,
) -> HypermeshResult<Vec<OverlayVertex>> {
    let right_edges =
        right
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "coplanar overlap face has no edge identities",
            })?;
    let right_planes = right.edge_planes();
    if right_planes.len() != right_edges.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "coplanar overlap clip planes and identities are not aligned",
        });
    }
    let mut overlap = IdentifiedPolygon::from_source(left)?;
    for (edge, right_plane) in right_planes.iter().enumerate() {
        overlap = overlap
            .clip_negative(
                decisions,
                right_plane,
                right_edges
                    .get(edge)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "coplanar overlap clip identity is absent",
                    })?,
            )?
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "positive-area overlap clipped to an empty region",
            })?;
    }
    let points = overlap.polygon.vertices_decision(decisions)?;
    if points.len() < 3 || points.len() != overlap.plane_edges.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "positive-area overlap has no planar boundary cycle",
        });
    }
    let support =
        pairwise_support_identity(left_face)?.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "coplanar overlap face has no support identity",
        })?;
    let mut result = Vec::new();
    result
        .try_reserve_exact(points.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "coplanar overlay vertices",
        })?;
    for (index, point) in points.into_iter().enumerate() {
        let next = (index + 1) % overlap.plane_edges.len();
        result.push(OverlayVertex {
            point,
            identity: intersect_arrangement_lines(
                &overlap.plane_edges[index],
                &overlap.plane_edges[next],
                support,
            )?,
            outgoing: overlap.plane_edges[next].clone(),
        });
    }
    Ok(result)
}

fn intersect_arrangement_lines(
    first: &ConstructionEdgeIdentity,
    second: &ConstructionEdgeIdentity,
    support: ConstructionPlaneIdentity,
) -> HypermeshResult<ArrangementPointIdentity> {
    if first == second {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "adjacent arrangement edges have one construction identity",
        });
    }
    if let (
        ConstructionEdgeIdentity::Source {
            mesh: first_mesh,
            endpoints: first_endpoints,
        },
        ConstructionEdgeIdentity::Source {
            mesh: second_mesh,
            endpoints: second_endpoints,
        },
    ) = (first, second)
        && first_mesh == second_mesh
        && let Some(vertex) = first_endpoints
            .iter()
            .find(|vertex| second_endpoints.contains(vertex))
    {
        return Ok(ArrangementPointIdentity::Construction(
            ConstructionVertexIdentity::Source {
                mesh: *first_mesh,
                vertex: *vertex,
            },
        ));
    }
    let source_split = |source: &ConstructionEdgeIdentity,
                        split: &ConstructionEdgeIdentity|
     -> Option<ConstructionVertexIdentity> {
        let ConstructionEdgeIdentity::Split { planes } = split else {
            return None;
        };
        let other = if planes[0] == support {
            planes[1]
        } else if planes[1] == support {
            planes[0]
        } else {
            return None;
        };
        Some(source.intersection_identity(other))
    };
    if matches!(first, ConstructionEdgeIdentity::Source { .. })
        && let Some(identity) = source_split(first, second)
    {
        return Ok(ArrangementPointIdentity::Construction(identity));
    }
    if matches!(second, ConstructionEdgeIdentity::Source { .. })
        && let Some(identity) = source_split(second, first)
    {
        return Ok(ArrangementPointIdentity::Construction(identity));
    }
    if let (
        ConstructionEdgeIdentity::Split { planes: first },
        ConstructionEdgeIdentity::Split { planes: second },
    ) = (first, second)
        && first.contains(&support)
        && second.contains(&support)
    {
        let first_other = if first[0] == support {
            first[1]
        } else {
            first[0]
        };
        let second_other = if second[0] == support {
            second[1]
        } else {
            second[0]
        };
        if first_other != second_other {
            let mut planes = [support, first_other, second_other];
            planes.sort_unstable();
            return Ok(ArrangementPointIdentity::Construction(
                ConstructionVertexIdentity::PlaneTriple { planes },
            ));
        }
    }
    let mut edges = [first.clone(), second.clone()];
    edges.sort_unstable();
    Ok(ArrangementPointIdentity::CoplanarEdges(edges))
}

struct FaceResult {
    triangles: Vec<[u32; 3]>,
    #[cfg(test)]
    constraints: Vec<[u32; 2]>,
    #[cfg(test)]
    contacts: Vec<u32>,
}

fn corefine_face(
    decisions: &DecisionContext,
    face: usize,
    polygon: &ConvexPolygon,
    work: &FaceWork,
    arena: &mut ArrangementPointArena,
) -> HypermeshResult<FaceResult> {
    if !work.is_changed() {
        return Ok(FaceResult {
            triangles: triangulate_convex_boundary(&work.boundary),
            #[cfg(test)]
            constraints: work
                .boundary
                .iter()
                .copied()
                .zip(work.boundary.iter().copied().cycle().skip(1))
                .take(work.boundary.len())
                .map(|(from, to)| sorted_edge([from, to]))
                .collect(),
            #[cfg(test)]
            contacts: Vec::new(),
        });
    }
    let boundary = &work.boundary;
    let mut constraint_lines = BTreeMap::<[u32; 2], ConstructionEdgeIdentity>::new();
    let edge_identities =
        polygon
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no canonical edge identities",
            })?;
    if edge_identities.len() != boundary.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "source face boundary and edge identities are not aligned",
        });
    }
    for edge in 0..boundary.len() {
        constraint_lines.insert(
            sorted_edge([boundary[edge], boundary[(edge + 1) % boundary.len()]]),
            edge_identities
                .get(edge)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "source edge identity cycle is incomplete",
                })?,
        );
    }
    for constraint in &work.constraints {
        let endpoints = sorted_edge(constraint.endpoints);
        if endpoints[0] == endpoints[1] {
            continue;
        }
        constraint_lines
            .entry(endpoints)
            .and_modify(|line| {
                if constraint.line < *line {
                    *line = constraint.line.clone();
                }
            })
            .or_insert_with(|| constraint.line.clone());
    }
    let mut point_ids = BTreeSet::new();
    point_ids.extend(boundary.iter().copied());
    point_ids.extend(work.contacts.iter().copied());
    for edge in constraint_lines.keys() {
        point_ids.extend(edge);
    }
    let support = polygon.support_plane();
    let edges = polygon.edge_planes();
    let projection_axis = projection_axis(decisions, support)?;
    let axes =
        projection_axes(projection_axis).ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "source face projection axis is invalid",
        })?;
    let mut projected = BTreeMap::new();
    for &point in &point_ids {
        let materialized =
            arena
                .points
                .get(point as usize)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "face constraint references an absent point",
                })?;
        if classify_point_decision(decisions, materialized, support)? != Classification::On
            || edges.iter().try_fold(false, |outside, edge| {
                Ok::<_, HypermeshError>(
                    outside
                        || classify_point_decision(decisions, materialized, edge)?
                            == Classification::Positive,
                )
            })?
        {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "face arrangement point lies outside its source face",
            });
        }
        projected.insert(point, project_point(materialized, axes));
    }

    let authored = constraint_lines
        .iter()
        .map(|(&endpoints, line)| RawConstraint {
            endpoints,
            line: line.clone(),
        })
        .collect::<Vec<_>>();
    for left_index in 0..authored.len() {
        let left = &authored[left_index];
        for right in &authored[(left_index + 1)..] {
            if left
                .endpoints
                .iter()
                .any(|point| right.endpoints.contains(point))
            {
                continue;
            }
            let left_points = left.endpoints.map(|point| {
                projected
                    .get(&point)
                    .expect("constraint endpoint is projected")
            });
            let right_points = right.endpoints.map(|point| {
                projected
                    .get(&point)
                    .expect("constraint endpoint is projected")
            });
            if !segments_properly_cross(decisions, left_points, right_points)? {
                continue;
            }
            let support_identity = pairwise_support_identity(face)?.ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "source face has no operation-local support identity",
                },
            )?;
            let identity = intersect_arrangement_lines(&left.line, &right.line, support_identity)?;
            let point_id = if let Some(point) = arena.retained_point(&identity) {
                point
            } else {
                let intersection = hyperlimit::construct_line_intersection_point(
                    left_points[0],
                    left_points[1],
                    right_points[0],
                    right_points[1],
                )
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "proper face constraints have no intersection point",
                })?;
                let planar = hypertri::ExactPoint::new(intersection.x, intersection.y);
                let point = lift_planar_point(&planar, support, projection_axis, axes)?;
                if classify_point_decision(decisions, &point, support)? != Classification::On {
                    return Err(HypermeshError::SurfaceArrangementFailed {
                        reason: "lifted face crossing does not lie on its source support",
                    });
                }
                arena.insert(decisions, identity, point)?
            };
            projected
                .entry(point_id)
                .or_insert_with(|| project_point(&arena.points[point_id as usize], axes));
        }
    }

    let mut split_lines = BTreeMap::<[u32; 2], ConstructionEdgeIdentity>::new();
    let mut on_segment = Vec::new();
    on_segment.try_reserve_exact(projected.len()).map_err(|_| {
        HypermeshError::CapacityOverflow {
            operation: "face constraint point schedule",
        }
    })?;
    for constraint in &authored {
        on_segment.clear();
        // Authored endpoints define this exact closed segment. Seed them
        // directly and reserve policy-aware incidence tests for other points
        // that may split it through crossings, contacts, or overlaps.
        on_segment.extend_from_slice(&constraint.endpoints);
        for (&point_id, point) in &projected {
            if constraint.endpoints.contains(&point_id) {
                continue;
            }
            if planar_point_on_segment(
                decisions,
                [
                    projected
                        .get(&constraint.endpoints[0])
                        .expect("constraint endpoint is projected"),
                    projected
                        .get(&constraint.endpoints[1])
                        .expect("constraint endpoint is projected"),
                ],
                point,
            )? {
                on_segment.push(point_id);
            }
        }
        sort_point_ids_on_segment(decisions, &projected, constraint.endpoints, &mut on_segment)?;
        for pair in on_segment.windows(2) {
            let edge = sorted_edge([pair[0], pair[1]]);
            if edge[0] != edge[1] {
                split_lines
                    .entry(edge)
                    .and_modify(|line| {
                        if constraint.line < *line {
                            *line = constraint.line.clone();
                        }
                    })
                    .or_insert_with(|| constraint.line.clone());
            }
        }
    }

    let boundary_edges = boundary
        .iter()
        .copied()
        .zip(boundary.iter().copied().cycle().skip(1))
        .take(boundary.len())
        .map(|edge| sorted_edge([edge.0, edge.1]))
        .collect::<BTreeSet<_>>();
    let only_source_boundary = projected.len() == boundary.len()
        && split_lines.keys().copied().collect::<BTreeSet<_>>() == boundary_edges;
    #[cfg(test)]
    let contacts = {
        let mut contacts = work.contacts.clone();
        contacts.sort_unstable();
        contacts.dedup();
        contacts
    };
    if only_source_boundary {
        return Ok(FaceResult {
            triangles: triangulate_convex_boundary(boundary),
            #[cfg(test)]
            constraints: boundary_edges.into_iter().collect(),
            #[cfg(test)]
            contacts,
        });
    }

    let point_ids = projected.keys().copied().collect::<Vec<_>>();
    let local = point_ids
        .iter()
        .enumerate()
        .map(|(local, &global)| (global, local))
        .collect::<BTreeMap<_, _>>();
    let points = point_ids
        .iter()
        .map(|point| projected[point].clone())
        .collect::<Vec<_>>();
    let constraints = split_lines
        .keys()
        .map(|edge| hypertri::Constraint::new(local[&edge[0]], local[&edge[1]]))
        .collect::<Vec<_>>();
    let context = hypertri::TriangulationContext::new(decisions.policy());
    let outcome =
        hypertri::cdt::constrained_triangulation_convex_hull(&context, &points, &constraints)
            .map_err(map_triangulation_error)?;
    decisions.absorb(match outcome.certainty {
        hypertri::TriangulationCertainty::Certified => MeshCertainty::Certified,
        hypertri::TriangulationCertainty::Approximate512Consumed => {
            MeshCertainty::Approximate512Consumed
        }
    });
    if outcome.value.points().len() != points.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "preplanarized face constraints produced an unexpected Steiner point",
        });
    }
    let source_positive = source_projection_is_positive(decisions, &projected, boundary)?;
    let mut triangles = Vec::new();
    triangles
        .try_reserve_exact(outcome.value.triangles().len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face triangles",
        })?;
    for triangle in outcome.value.triangles() {
        let mut triangle = triangle.map(|vertex| point_ids[vertex]);
        // Hypertri's checked topology entry point returns only strictly
        // positive triangles and has already absorbed every predicate into its
        // outcome certainty. Preserve that exact postcondition instead of
        // repeating one orientation decision per output triangle.
        if !source_positive {
            triangle.swap(1, 2);
        }
        triangles.push(triangle);
    }
    #[cfg(test)]
    let constraints = outcome
        .value
        .constraint_edges()
        .iter()
        .map(|constraint| sorted_edge([point_ids[constraint.from], point_ids[constraint.to]]))
        .collect::<Vec<_>>();
    Ok(FaceResult {
        triangles,
        #[cfg(test)]
        constraints,
        #[cfg(test)]
        contacts,
    })
}

fn triangulate_convex_boundary(boundary: &[u32]) -> Vec<[u32; 3]> {
    (1..boundary.len().saturating_sub(1))
        .map(|index| [boundary[0], boundary[index], boundary[index + 1]])
        .collect()
}

fn source_projection_is_positive(
    decisions: &DecisionContext,
    projected: &BTreeMap<u32, hypertri::ExactPoint>,
    boundary: &[u32],
) -> HypermeshResult<bool> {
    for index in 1..boundary.len().saturating_sub(1) {
        match planar_orientation(
            decisions,
            &projected[&boundary[0]],
            &projected[&boundary[index]],
            &projected[&boundary[index + 1]],
        )? {
            Classification::Positive => return Ok(true),
            Classification::Negative => return Ok(false),
            Classification::On => {}
        }
    }
    Err(HypermeshError::SurfaceArrangementFailed {
        reason: "source face projection is degenerate",
    })
}

fn segments_properly_cross(
    decisions: &DecisionContext,
    left: [&hypertri::ExactPoint; 2],
    right: [&hypertri::ExactPoint; 2],
) -> HypermeshResult<bool> {
    let opposite = |first, second| {
        matches!(
            (first, second),
            (Classification::Negative, Classification::Positive)
                | (Classification::Positive, Classification::Negative)
        )
    };
    let right_sides = [
        planar_orientation(decisions, left[0], left[1], right[0])?,
        planar_orientation(decisions, left[0], left[1], right[1])?,
    ];
    if !opposite(right_sides[0], right_sides[1]) {
        return Ok(false);
    }
    let left_sides = [
        planar_orientation(decisions, right[0], right[1], left[0])?,
        planar_orientation(decisions, right[0], right[1], left[1])?,
    ];
    Ok(opposite(left_sides[0], left_sides[1]))
}

fn planar_point_on_segment(
    decisions: &DecisionContext,
    edge: [&hypertri::ExactPoint; 2],
    point: &hypertri::ExactPoint,
) -> HypermeshResult<bool> {
    decisions.decide(
        hyperlimit::point_on_segment(edge[0], edge[1], point, decisions.policy()),
        "surface arrangement point on segment",
    )
}

fn sort_point_ids_on_segment(
    decisions: &DecisionContext,
    points: &BTreeMap<u32, hypertri::ExactPoint>,
    edge: [u32; 2],
    indices: &mut [u32],
) -> HypermeshResult<()> {
    let use_x =
        !compare_real_decision(decisions, &points[&edge[0]].x, &points[&edge[1]].x)?.is_eq();
    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0 {
            let left = &points[&indices[cursor]];
            let right = &points[&indices[cursor - 1]];
            let ordering = if use_x {
                compare_real_decision(decisions, &left.x, &right.x)?
            } else {
                compare_real_decision(decisions, &left.y, &right.y)?
            };
            if !ordering.is_lt() {
                break;
            }
            indices.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Ok(())
}

fn planar_orientation(
    decisions: &DecisionContext,
    a: &hypertri::ExactPoint,
    b: &hypertri::ExactPoint,
    c: &hypertri::ExactPoint,
) -> HypermeshResult<Classification> {
    let sign = decisions.decide(
        hyperlimit::orient2(a, b, c, decisions.policy()),
        "surface arrangement orientation",
    )?;
    Ok(match sign {
        hyperlimit::Sign::Negative => Classification::Negative,
        hyperlimit::Sign::Zero => Classification::On,
        hyperlimit::Sign::Positive => Classification::Positive,
    })
}

fn projection_axis(decisions: &DecisionContext, plane: &Plane) -> HypermeshResult<usize> {
    let normal = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    if let Some(axis) = normal.iter().position(|component| {
        component
            .exact_rational_ref()
            .is_some_and(|value| !value.is_zero())
    }) {
        return Ok(axis);
    }
    for (axis, component) in normal.into_iter().enumerate() {
        if classify_real(decisions, component)? != Classification::On {
            return Ok(axis);
        }
    }
    Err(HypermeshError::SurfaceArrangementFailed {
        reason: "source face has a zero support normal",
    })
}

fn projection_axes(dropped: usize) -> Option<[usize; 2]> {
    match dropped {
        0 => Some([1, 2]),
        1 => Some([0, 2]),
        2 => Some([0, 1]),
        _ => None,
    }
}

fn point_axis(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        _ => &point.z,
    }
}

fn project_point(point: &Point3, [u, v]: [usize; 2]) -> hypertri::ExactPoint {
    hypertri::ExactPoint::new(point_axis(point, u).clone(), point_axis(point, v).clone())
}

fn lift_planar_point(
    point: &hypertri::ExactPoint,
    plane: &Plane,
    dropped: usize,
    [u, v]: [usize; 2],
) -> HypermeshResult<Point3> {
    let normal = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    let one = Real::one();
    let numerator = Real::signed_product_sum(
        [false; 3],
        [
            [normal[u], &point.x],
            [normal[v], &point.y],
            [&plane.offset, &one],
        ],
    );
    let coordinate =
        (numerator / normal[dropped]).map_err(|_| HypermeshError::SurfaceArrangementFailed {
            reason: "source face projection coefficient is zero",
        })?;
    Ok(match dropped {
        0 => Point3::new(coordinate, point.x.clone(), point.y.clone()),
        1 => Point3::new(point.x.clone(), coordinate, point.y.clone()),
        2 => Point3::new(point.x.clone(), point.y.clone(), coordinate),
        _ => {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "source face projection axis is invalid",
            });
        }
    })
}

fn map_triangulation_error(error: hypertri::Error) -> HypermeshError {
    match error {
        hypertri::Error::PredicateUndecided { predicate } => {
            HypermeshError::PredicateUndecided { predicate }
        }
        hypertri::Error::InvalidInput { reason } => {
            HypermeshError::SurfaceArrangementFailed { reason }
        }
        hypertri::Error::NoEarFound => HypermeshError::SurfaceArrangementFailed {
            reason: "bounded face triangulation found no ear",
        },
        hypertri::Error::UnsupportedFeature { feature } => {
            HypermeshError::SurfaceArrangementFailed { reason: feature }
        }
    }
}

fn sorted_edge(mut edge: [u32; 2]) -> [u32; 2] {
    edge.sort_unstable();
    edge
}

pub(crate) struct ExactSurfaceArrangement {
    corefinement: SurfaceCorefinement,
    cells: SurfaceCellComplex,
}

pub(crate) fn build_surface_arrangement(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<ExactSurfaceArrangement> {
    let source_bvh = ExactBvh::build_for_query_hierarchy_decision(decisions, polygons)?;
    let graph = crate::intersection::pairwise_intersections_by_polygon_from_bvh(
        decisions,
        polygons,
        &[],
        &source_bvh,
    )?;
    let source_bvh = source_bvh.into_query_hierarchy(polygons)?;
    let corefinement = corefine_surface(decisions, polygons, &graph)?;
    let radially_separated_face_pair_keys = graph.into_radially_separated_face_pair_keys();
    let cells = assemble_surface_cells(
        decisions,
        polygons,
        &corefinement,
        &radially_separated_face_pair_keys,
        &source_bvh,
    )?;
    Ok(ExactSurfaceArrangement {
        corefinement,
        cells,
    })
}

pub(crate) fn validate_surface_boolean_program(
    program: BooleanProgram<'_>,
    operand_count: usize,
) -> HypermeshResult<()> {
    match program {
        BooleanProgram::Operation(_) if operand_count != 0 => Ok(()),
        BooleanProgram::Operation(_) => Err(HypermeshError::InvalidBooleanProgram {
            reason: "at least one operand is required",
        }),
        BooleanProgram::Expressions { nodes, roots } => {
            validate_cell_truth_program(nodes, roots, operand_count)
        }
    }
}

impl ExactSurfaceArrangement {
    pub(crate) fn materialize_program(
        &self,
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        program: BooleanProgram<'_>,
    ) -> HypermeshResult<BooleanMeshBatch> {
        match program {
            BooleanProgram::Operation(operation) => {
                let mut classifications = Vec::new();
                classifications
                    .try_reserve_exact(self.cells.facets.len())
                    .map_err(|_| HypermeshError::CapacityOverflow {
                        operation: "surface output classifications",
                    })?;
                classifications.extend(
                    (0..self.cells.facets.len())
                        .map(|facet| self.cells.facet_classification(facet, operation)),
                );
                materialize_surface_outputs(
                    decisions,
                    polygons,
                    &self.corefinement,
                    &self.cells,
                    1,
                    &classifications,
                    &[false],
                )
            }
            BooleanProgram::Expressions { nodes, roots } => {
                let classifications = self.cells.classify_expressions(nodes, roots)?;
                self.materialize_expression_classifications(decisions, polygons, &classifications)
            }
        }
    }

    #[cfg(test)]
    fn materialize_operation(
        &self,
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        operation: crate::winding::BooleanOp,
    ) -> HypermeshResult<BooleanMeshBatch> {
        let mut classifications = Vec::new();
        classifications
            .try_reserve_exact(self.cells.facets.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface output classifications",
            })?;
        classifications.extend(
            (0..self.cells.facets.len())
                .map(|facet| self.cells.facet_classification(facet, operation)),
        );
        self.materialize_classifications(decisions, polygons, &classifications)
    }

    #[cfg(test)]
    fn materialize_classifications(
        &self,
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        classifications: &[i8],
    ) -> HypermeshResult<BooleanMeshBatch> {
        materialize_surface_outputs(
            decisions,
            polygons,
            &self.corefinement,
            &self.cells,
            1,
            classifications,
            &[false],
        )
    }

    fn materialize_expression_classifications(
        &self,
        decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        classifications: &ExpressionClassifications,
    ) -> HypermeshResult<BooleanMeshBatch> {
        if classifications.facet_count != self.cells.facets.len() {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface expression classifications and facets differ",
            });
        }
        materialize_surface_outputs(
            decisions,
            polygons,
            &self.corefinement,
            &self.cells,
            classifications.expression_count,
            &classifications.classifications,
            &classifications.exterior_inside,
        )
    }
}

fn materialize_surface_outputs(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    surface: &SurfaceCorefinement,
    cells: &SurfaceCellComplex,
    expression_count: usize,
    classifications: &[i8],
    exterior_inside: &[bool],
) -> HypermeshResult<BooleanMeshBatch> {
    let classification_count = expression_count.checked_mul(cells.facets.len()).ok_or(
        HypermeshError::CapacityOverflow {
            operation: "surface output classification matrix",
        },
    )?;
    if classifications.len() != classification_count {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface output classification matrix has invalid dimensions",
        });
    }
    if exterior_inside.len() != expression_count {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface output exterior classifications have invalid dimensions",
        });
    }
    let mut used = Vec::new();
    used.try_reserve_exact(surface.points.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface output used points",
        })?;
    used.resize(surface.points.len(), false);
    let mut triangle_counts = Vec::new();
    triangle_counts
        .try_reserve_exact(expression_count)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface output triangle counts",
        })?;
    triangle_counts.resize(expression_count, 0_usize);
    for (expression, triangle_count) in triangle_counts.iter_mut().enumerate() {
        let start = expression * cells.facets.len();
        let row = &classifications[start..start + cells.facets.len()];
        for (facet, &classification) in cells.facets.iter().zip(row) {
            if !matches!(classification, -1..=1) {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output classification is invalid",
                });
            }
            if classification == 0 {
                continue;
            }
            *triangle_count =
                triangle_count
                    .checked_add(1)
                    .ok_or(HypermeshError::CapacityOverflow {
                        operation: "surface output triangles",
                    })?;
            for &point in &facet.vertices {
                *used.get_mut(point as usize).ok_or(
                    HypermeshError::SurfaceArrangementFailed {
                        reason: "surface output facet references an absent point",
                    },
                )? = true;
            }
        }
        certify_selected_surface_output(decisions, surface, cells, row)?;
    }

    let vertex_count = used.iter().filter(|is_used| **is_used).count();
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(vertex_count)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface output vertices",
        })?;
    let mut remap = Vec::new();
    remap.try_reserve_exact(surface.points.len()).map_err(|_| {
        HypermeshError::CapacityOverflow {
            operation: "surface output vertex remap",
        }
    })?;
    remap.resize(surface.points.len(), u32::MAX);
    for (point, (source, is_used)) in surface.points.iter().zip(&used).enumerate() {
        if !is_used {
            continue;
        }
        remap[point] = compact_len(vertices.len(), "surface output vertex IDs")?;
        vertices.push(source.clone());
    }

    let mut outputs = Vec::new();
    outputs
        .try_reserve_exact(expression_count)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface outputs",
        })?;
    for &triangle_count in &triangle_counts {
        let mut triangles = Vec::new();
        triangles.try_reserve_exact(triangle_count).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface output triangles",
            }
        })?;
        let mut sources = Vec::new();
        sources.try_reserve_exact(triangle_count).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface output provenance",
            }
        })?;
        outputs.push(BooleanMeshResult {
            triangles,
            sources,
            exterior_inside: exterior_inside[outputs.len()],
        });
    }

    for expression in 0..expression_count {
        let start = expression * cells.facets.len();
        let row = &classifications[start..start + cells.facets.len()];
        let output =
            outputs
                .get_mut(expression)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output row is absent",
                })?;
        for (facet_index, (facet, &classification)) in cells.facets.iter().zip(row).enumerate() {
            if classification == 0 {
                continue;
            }
            let triangle = facet.vertices.map(|point| {
                remap
                    .get(point as usize)
                    .copied()
                    .filter(|&output| output != u32::MAX)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "surface output point has no compact vertex",
                    })
            });
            let [a, b, c] = triangle;
            let mut triangle = [a?, b?, c?];
            if classification == -1 {
                triangle.swap(1, 2);
            }
            let contribution = cells
                .checked_facet_contributions(facet_index)?
                .first()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output facet has no source contribution",
                })?;
            if !matches!(contribution.orientation, -1 | 1) {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output contribution orientation is invalid",
                });
            }
            let polygon = polygons.get(contribution.face as usize).ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output contribution references an absent face",
                },
            )?;
            output.triangles.push(triangle);
            output.sources.push(TriangleSource {
                mesh: polygon.mesh_index,
                triangle: polygon.polygon_index,
                orientation: classification * contribution.orientation,
            });
        }
    }
    Ok(BooleanMeshBatch {
        vertices,
        results: outputs,
    })
}

fn certify_selected_surface_output(
    decisions: &DecisionContext,
    surface: &SurfaceCorefinement,
    cells: &SurfaceCellComplex,
    classifications: &[i8],
) -> HypermeshResult<BooleanMeshClosureEvidence> {
    if classifications.len() != cells.facets.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface output classifications and facets differ",
        });
    }
    let mut selected_count = 0_usize;
    for &classification in classifications {
        if !matches!(classification, -1..=1) {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output classification is invalid",
            });
        }
        selected_count = selected_count
            .checked_add(usize::from(classification != 0))
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "surface output certificate facets",
            })?;
    }
    let mut triangles = StorageHashMap::<[u32; 3], ()>::default();
    triangles
        .try_reserve(selected_count)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface output triangle certificate",
        })?;
    let expected_edges = selected_count
        .checked_add(selected_count / 2)
        .and_then(|edges| edges.checked_add(selected_count & 1))
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "surface output edge certificate",
        })?;
    let mut edges = StorageHashMap::<[u32; 2], [u32; 2]>::default();
    edges
        .try_reserve(expected_edges)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface output edge certificate",
        })?;
    for (facet, &classification) in cells.facets.iter().zip(classifications) {
        if classification == 0 {
            continue;
        }
        let triangle = facet.vertices;
        let [a, b, c] = triangle.map(|point| {
            surface
                .points
                .get(point as usize)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "surface output facet references an absent point",
                })
        });
        let [a, b, c] = [a?, b?, c?];
        if triangle[0] == triangle[1]
            || triangle[1] == triangle[2]
            || triangle[0] == triangle[2]
            || !Plane::decide_points_are_nondegenerate(decisions, a, b, c)?
        {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contains a degenerate triangle",
            });
        }
        let mut triangle_key = triangle;
        triangle_key.sort_unstable();
        if triangles.insert(triangle_key, ()).is_some() {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contains duplicate triangle geometry",
            });
        }
        let mut oriented = triangle;
        if classification == -1 {
            oriented.swap(1, 2);
        }
        for edge in [
            [oriented[0], oriented[1]],
            [oriented[1], oriented[2]],
            [oriented[2], oriented[0]],
        ] {
            let canonical = sorted_edge(edge);
            if edges.len() == edges.capacity() && !edges.contains_key(&canonical) {
                edges
                    .try_reserve(1)
                    .map_err(|_| HypermeshError::CapacityOverflow {
                        operation: "surface output edge certificate",
                    })?;
            }
            let uses = edges.entry(canonical).or_default();
            let direction = usize::from(edge != canonical);
            uses[direction] =
                uses[direction]
                    .checked_add(1)
                    .ok_or(HypermeshError::CapacityOverflow {
                        operation: "surface output edge multiplicity",
                    })?;
        }
    }

    let mut evidence = BooleanMeshClosureEvidence::default();
    for uses in edges.values() {
        let total = u64::from(uses[0]) + u64::from(uses[1]);
        if total == 1 {
            evidence.boundary_edges += 1;
        } else if total > 2 {
            evidence.non_manifold_edges += 1;
        }
        if uses[0] != uses[1] {
            evidence.unbalanced_edges += 1;
        }
    }
    if !evidence.has_no_boundary() {
        return Err(HypermeshError::OpenOutput {
            boundary_edges: evidence.boundary_edges,
            unbalanced_edges: evidence.unbalanced_edges,
            non_manifold_edges: evidence.non_manifold_edges,
        });
    }
    Ok(evidence)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::MeshContext;
    use crate::intersection::pairwise_intersections_by_polygon;
    use crate::test_support::approximate_convex_triangle;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn triangle(
        points: [Point3; 3],
        mesh: usize,
        face: usize,
        vertices: [usize; 3],
    ) -> ConvexPolygon {
        let mut polygon = approximate_convex_triangle(
            &points[0],
            &points[1],
            &points[2],
            mesh as isize,
            face as isize,
        );
        polygon
            .set_source_triangle_edge_identities(mesh, vertices)
            .unwrap();
        polygon
    }

    #[test]
    fn unchanged_face_work_keeps_its_boundary_inline() {
        assert_eq!(std::mem::size_of::<FaceWork>(), 64);
        let work = FaceWork {
            boundary: [0, 1, 2],
            ..FaceWork::default()
        };
        assert!(!work.is_changed());
        assert_eq!(work.constraints.capacity(), 0);
        assert_eq!(work.contacts.capacity(), 0);
    }

    #[test]
    fn directed_graph_events_reuse_one_arrangement_point_transfer() {
        let polygons = [
            triangle([p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)], 0, 0, [0, 1, 2]),
            triangle([p(1, -1, -1), p(1, 3, -1), p(1, -1, 3)], 1, 1, [0, 1, 2]),
        ];
        let decisions = crate::test_support::approximate_decisions();
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let point = match graph.event_ids(0).unwrap().next().unwrap().unwrap() {
            PairwiseIntersectionEventIds::NonCoplanarSegment { endpoints, .. } => endpoints[0],
            _ => panic!("transverse triangles must produce a segment"),
        };
        let mut remap = vec![None; graph.construction_point_count()];
        let mut arena = ArrangementPointArena::with_capacity(remap.len()).unwrap();
        assert_eq!(
            map_graph_point(&decisions, &graph, &mut [], &mut arena, point).unwrap_err(),
            HypermeshError::SurfaceArrangementFailed {
                reason: "intersection event references an absent construction point",
            }
        );
        let first = map_graph_point(&decisions, &graph, &mut remap, &mut arena, point).unwrap();
        let structural_count = arena.structural.len();
        let second = map_graph_point(&decisions, &graph, &mut remap, &mut arena, point).unwrap();

        assert_eq!(first, second);
        assert_eq!(remap[point as usize], Some(first));
        assert_eq!(arena.structural.len(), structural_count);
        assert_eq!(arena.points.len(), 1);
        assert_eq!(decisions.certainty(), MeshCertainty::Certified);
    }

    fn tetrahedron(
        origin: [i64; 3],
        extent: i64,
        mesh: usize,
        face_start: usize,
        vertex_start: usize,
        operand: usize,
        operand_count: usize,
    ) -> Vec<ConvexPolygon> {
        let [x, y, z] = origin;
        let vertices = [
            p(x, y, z),
            p(x + extent, y, z),
            p(x, y + extent, z),
            p(x, y, z + extent),
        ];
        let source_vertices = [
            vertex_start,
            vertex_start + 1,
            vertex_start + 2,
            vertex_start + 3,
        ];
        let faces = [
            [0_usize, 2, 1],
            [0_usize, 1, 3],
            [0_usize, 3, 2],
            [1_usize, 2, 3],
        ];
        faces
            .into_iter()
            .enumerate()
            .map(|(local_face, face)| {
                let mut polygon = triangle(
                    face.map(|vertex| vertices[vertex].clone()),
                    mesh,
                    face_start + local_face,
                    face.map(|vertex| source_vertices[vertex]),
                );
                polygon.delta_w = vec![0; operand_count];
                polygon.delta_w[operand] = 1;
                polygon
            })
            .collect()
    }

    fn tetrahedron_from_vertices(
        vertices: [Point3; 4],
        mesh: usize,
        face_start: usize,
        vertex_start: usize,
        operand: usize,
        operand_count: usize,
    ) -> Vec<ConvexPolygon> {
        let decisions = crate::test_support::approximate_decisions();
        [
            ([1_usize, 2, 3], 0_usize),
            ([0_usize, 3, 2], 1_usize),
            ([0_usize, 1, 3], 2_usize),
            ([0_usize, 2, 1], 3_usize),
        ]
        .into_iter()
        .enumerate()
        .map(|(local_face, (mut face, interior))| {
            let support =
                Plane::from_points(&vertices[face[0]], &vertices[face[1]], &vertices[face[2]]);
            match classify_point_decision(&decisions, &vertices[interior], &support).unwrap() {
                Classification::Positive => face.swap(1, 2),
                Classification::Negative => {}
                Classification::On => panic!("test tetrahedron is degenerate"),
            }
            let mut polygon = triangle(
                face.map(|vertex| vertices[vertex].clone()),
                mesh,
                face_start + local_face,
                face.map(|vertex| vertex_start + vertex),
            );
            polygon.delta_w = vec![0; operand_count];
            polygon.delta_w[operand] = 1;
            polygon
        })
        .collect()
    }

    fn voxel_ring(operand: usize, operand_count: usize) -> Vec<ConvexPolygon> {
        let occupied = (0_i64..3)
            .flat_map(|x| (0_i64..3).map(move |y| (x, y, 0_i64)))
            .filter(|&(x, y, _)| (x, y) != (1, 1))
            .collect::<BTreeSet<_>>();
        let faces = [
            ((-1, 0, 0), [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]]),
            ((1, 0, 0), [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]]),
            ((0, -1, 0), [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]]),
            ((0, 1, 0), [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]]),
            ((0, 0, -1), [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]]),
            ((0, 0, 1), [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]]),
        ];
        let mut vertex_ids = BTreeMap::<(i64, i64, i64), usize>::new();
        let mut polygons = Vec::new();
        for &(x, y, z) in &occupied {
            for &((dx, dy, dz), offsets) in &faces {
                if occupied.contains(&(x + dx, y + dy, z + dz)) {
                    continue;
                }
                let grid = offsets.map(|[ox, oy, oz]| (x + ox, y + oy, z + oz));
                let vertices = grid.map(|vertex| {
                    if let Some(&id) = vertex_ids.get(&vertex) {
                        id
                    } else {
                        let id = vertex_ids.len();
                        vertex_ids.insert(vertex, id);
                        id
                    }
                });
                let points = grid.map(|(px, py, pz)| p(px, py, pz));
                for local in [[0_usize, 1, 2], [0_usize, 2, 3]] {
                    let mut polygon = triangle(
                        local.map(|index| points[index].clone()),
                        0,
                        polygons.len(),
                        local.map(|index| vertices[index]),
                    );
                    polygon.delta_w = vec![0; operand_count];
                    polygon.delta_w[operand] = 1;
                    polygons.push(polygon);
                }
            }
        }
        polygons
    }

    fn arranged_cells(
        polygons: &[ConvexPolygon],
        policy: hyperlimit::PredicatePolicy,
    ) -> (MeshCertainty, SurfaceCorefinement, SurfaceCellComplex) {
        let context = MeshContext::new(policy);
        let decisions = DecisionContext::new(&context);
        let arrangement = build_surface_arrangement(&decisions, polygons).unwrap();
        (
            decisions.certainty(),
            arrangement.corefinement,
            arrangement.cells,
        )
    }

    fn triangle_edges(triangle: [u32; 3]) -> [[u32; 2]; 3] {
        [
            sorted_edge([triangle[0], triangle[1]]),
            sorted_edge([triangle[1], triangle[2]]),
            sorted_edge([triangle[2], triangle[0]]),
        ]
    }

    fn assert_constraints_are_edges(surface: &SurfaceCorefinement, face: usize) {
        let edges = surface
            .face_triangles(face)
            .iter()
            .flat_map(|triangle| triangle_edges(*triangle))
            .collect::<BTreeSet<_>>();
        assert!(
            surface
                .face_constraints(face)
                .iter()
                .all(|constraint| edges.contains(constraint))
        );
    }

    fn assert_face_result_constraints_are_edges(result: &FaceResult) {
        let edges = result
            .triangles
            .iter()
            .flat_map(|triangle| triangle_edges(*triangle))
            .collect::<BTreeSet<_>>();
        assert!(
            result
                .constraints
                .iter()
                .all(|constraint| edges.contains(constraint))
        );
    }

    fn selected_facets_are_closed(
        cells: &SurfaceCellComplex,
        operation: crate::winding::BooleanOp,
    ) -> bool {
        facet_classifications_are_closed(
            cells,
            &(0..cells.facets.len())
                .map(|facet| cells.facet_classification(facet, operation))
                .collect::<Vec<_>>(),
        )
    }

    fn facet_classifications_are_closed(
        cells: &SurfaceCellComplex,
        classifications: &[i8],
    ) -> bool {
        if classifications.len() != cells.facets.len() {
            return false;
        }
        let mut edge_uses = BTreeMap::<[u32; 2], [u32; 2]>::new();
        for (facet, &classification) in classifications.iter().enumerate() {
            if classification == 0 {
                continue;
            }
            let mut triangle = cells.facets[facet].vertices;
            if classification == -1 {
                triangle.swap(1, 2);
            }
            for [start, end] in [
                [triangle[0], triangle[1]],
                [triangle[1], triangle[2]],
                [triangle[2], triangle[0]],
            ] {
                edge_uses.entry(sorted_edge([start, end])).or_default()
                    [usize::from(start > end)] += 1;
            }
        }
        !edge_uses.is_empty() && edge_uses.values().all(|uses| uses[0] == uses[1])
    }

    fn assert_materialized_output(
        _decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        batch: &BooleanMeshBatch,
        selected_facets: usize,
    ) {
        assert_eq!(batch.results.len(), 1);
        let output = &batch.results[0];
        assert_eq!(output.triangles.len(), selected_facets);
        assert_eq!(output.sources.len(), selected_facets);
        let mut used = vec![false; batch.vertices.len()];
        for triangle in &output.triangles {
            for &vertex in triangle {
                used[vertex as usize] = true;
            }
        }
        assert!(used.into_iter().all(|is_used| is_used));
        assert!(output.sources.iter().all(|source| {
            matches!(source.orientation, -1 | 1)
                && polygons.iter().any(|polygon| {
                    polygon.mesh_index == source.mesh && polygon.polygon_index == source.triangle
                })
        }));
        let evidence = crate::output::boolean_mesh_closure_evidence(output);
        assert!(evidence.has_no_boundary());
    }

    fn assert_materialized_batch_output(
        _decisions: &DecisionContext,
        polygons: &[ConvexPolygon],
        vertices: &[Point3],
        output: &BooleanMeshResult,
        selected_facets: usize,
    ) {
        assert_eq!(output.triangles.len(), selected_facets);
        assert_eq!(output.sources.len(), selected_facets);
        assert!(
            output
                .triangles
                .iter()
                .flatten()
                .all(|&vertex| (vertex as usize) < vertices.len())
        );
        assert!(output.sources.iter().all(|source| {
            matches!(source.orientation, -1 | 1)
                && polygons.iter().any(|polygon| {
                    polygon.mesh_index == source.mesh && polygon.polygon_index == source.triangle
                })
        }));
        assert!(crate::output::boolean_mesh_closure_evidence(output).has_no_boundary());
    }

    fn signed_six_volume(vertices: &[Point3], output: &BooleanMeshResult) -> Real {
        let mut volume = Real::zero();
        for triangle in &output.triangles {
            let a = &vertices[triangle[0] as usize];
            let b = &vertices[triangle[1] as usize];
            let c = &vertices[triangle[2] as usize];
            volume += &a.x * &(&b.y * &c.z - &b.z * &c.y)
                + &a.y * &(&b.z * &c.x - &b.x * &c.z)
                + &a.z * &(&b.x * &c.y - &b.y * &c.x);
        }
        volume
    }

    #[test]
    fn source_winding_ray_ignores_artificial_corefinement_edges() {
        let source = triangle([p(0, 0, 0), p(4, 0, 0), p(0, 4, 0)], 0, 0, [0, 1, 2]);
        let points = [p(0, 0, 0), p(4, 0, 0), p(2, 2, 0), p(0, 4, 0)];
        let origin = p(1, 1, -1);
        let direction = Vector3::new([Real::zero(), Real::zero(), Real::one()]);

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            assert!(matches!(
                ray_source_polygon_relation(&decisions, &source, &origin, &direction).unwrap(),
                RayFacetRelation::Ahead { frontward: true }
            ));
            for triangle in [[0, 1, 2], [0, 2, 3]] {
                assert!(matches!(
                    ray_facet_relation(&decisions, &points, triangle, &origin, &direction).unwrap(),
                    RayFacetRelation::Degenerate
                ));
            }
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn one_arrangement_materializes_every_builtin_operation_deterministically() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
        polygons.extend(tetrahedron([1, 1, -1], 4, 1, 4, 0, 1, 2));
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let arrangement = build_surface_arrangement(&decisions, &polygons).unwrap();
            for operation in [
                crate::winding::BooleanOp::Union,
                crate::winding::BooleanOp::Intersection,
                crate::winding::BooleanOp::Difference,
                crate::winding::BooleanOp::SymmetricDifference,
            ] {
                let selected = (0..arrangement.cells.facets.len())
                    .filter(|&facet| arrangement.cells.facet_classification(facet, operation) != 0)
                    .count();
                let first = arrangement
                    .materialize_operation(&decisions, &polygons, operation)
                    .unwrap();
                let second = arrangement
                    .materialize_program(
                        &decisions,
                        &polygons,
                        BooleanProgram::Operation(operation),
                    )
                    .unwrap();
                assert_eq!(first, second);
                assert_materialized_output(&decisions, &polygons, &first, selected);
                assert_eq!(
                    classify_real(
                        &decisions,
                        &signed_six_volume(&first.vertices, &first.results[0]),
                    )
                    .unwrap(),
                    Classification::Positive
                );
            }
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn direct_materialization_accepts_empty_and_balanced_nonmanifold_outputs() {
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);

            let mut disjoint = tetrahedron([0, 0, 0], 1, 0, 0, 0, 0, 2);
            disjoint.extend(tetrahedron([3, 0, 0], 1, 1, 4, 4, 1, 2));
            let arrangement = build_surface_arrangement(&decisions, &disjoint).unwrap();
            let empty = arrangement
                .materialize_operation(
                    &decisions,
                    &disjoint,
                    crate::winding::BooleanOp::Intersection,
                )
                .unwrap();
            assert!(empty.vertices.is_empty());
            assert!(empty.results[0].triangles.is_empty());
            assert_materialized_output(&decisions, &disjoint, &empty, 0);

            let mut tangent = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
            tangent.extend(tetrahedron_from_vertices(
                [p(0, 0, 0), p(4, 0, 0), p(0, -4, 0), p(0, 0, -4)],
                1,
                4,
                0,
                1,
                2,
            ));
            let arrangement = build_surface_arrangement(&decisions, &tangent).unwrap();
            let union = arrangement
                .materialize_operation(&decisions, &tangent, crate::winding::BooleanOp::Union)
                .unwrap();
            assert_materialized_output(&decisions, &tangent, &union, 8);
            let evidence = crate::output::boolean_mesh_closure_evidence(&union.results[0]);
            assert_eq!(evidence.non_manifold_edges, 1);
            assert!(!evidence.is_closed());
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn direct_materialization_rejects_every_malformed_output_path() {
        let polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 1);
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let mut arrangement = build_surface_arrangement(&decisions, &polygons).unwrap();
        let classifications = (0..arrangement.cells.facets.len())
            .map(|facet| {
                arrangement
                    .cells
                    .facet_classification(facet, crate::winding::BooleanOp::Union)
            })
            .collect::<Vec<_>>();

        assert!(matches!(
            arrangement.materialize_classifications(
                &decisions,
                &polygons,
                &classifications[..classifications.len() - 1],
            ),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output classification matrix has invalid dimensions"
            })
        ));
        let malformed_matrix = ExpressionClassifications {
            expression_count: 2,
            facet_count: arrangement.cells.facets.len(),
            classifications: classifications.clone(),
            exterior_inside: vec![false; 2],
        };
        assert!(matches!(
            arrangement.materialize_expression_classifications(
                &decisions,
                &polygons,
                &malformed_matrix,
            ),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output classification matrix has invalid dimensions"
            })
        ));
        let wrong_facet_count = ExpressionClassifications {
            expression_count: 1,
            facet_count: arrangement.cells.facets.len() + 1,
            classifications: classifications.clone(),
            exterior_inside: vec![false],
        };
        assert!(matches!(
            arrangement.materialize_expression_classifications(
                &decisions,
                &polygons,
                &wrong_facet_count,
            ),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface expression classifications and facets differ"
            })
        ));
        let no_outputs = arrangement
            .materialize_expression_classifications(
                &decisions,
                &polygons,
                &ExpressionClassifications {
                    expression_count: 0,
                    facet_count: arrangement.cells.facets.len(),
                    classifications: Vec::new(),
                    exterior_inside: Vec::new(),
                },
            )
            .unwrap();
        assert!(no_outputs.vertices.is_empty());
        assert!(no_outputs.results.is_empty());
        let mut invalid_classification = classifications.clone();
        invalid_classification[0] = 2;
        assert!(matches!(
            arrangement
                .materialize_classifications(&decisions, &polygons, &invalid_classification,),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output classification is invalid"
            })
        ));

        let saved_point = arrangement.cells.facets[0].vertices[0];
        arrangement.cells.facets[0].vertices[0] = u32::MAX;
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output facet references an absent point"
            })
        ));
        arrangement.cells.facets[0].vertices[0] = saved_point;

        let saved_point = arrangement.cells.facets[0].vertices[1];
        arrangement.cells.facets[0].vertices[1] = arrangement.cells.facets[0].vertices[0];
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contains a degenerate triangle"
            })
        ));
        arrangement.cells.facets[0].vertices[1] = saved_point;

        let saved_offset = arrangement.cells.contribution_offsets[1];
        arrangement.cells.contribution_offsets[1] = arrangement.cells.contribution_offsets[0];
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output facet has no source contribution"
            })
        ));
        arrangement.cells.contribution_offsets[1] = saved_offset;

        let saved_start = arrangement.cells.contribution_offsets[0];
        arrangement.cells.contribution_offsets[0] = saved_offset;
        arrangement.cells.contribution_offsets[1] = saved_start;
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output facet contribution storage is malformed"
            })
        ));
        arrangement.cells.contribution_offsets[0] = saved_start;
        arrangement.cells.contribution_offsets[1] = u32::MAX;
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output facet contribution storage is malformed"
            })
        ));
        arrangement.cells.contribution_offsets[1] = saved_offset;

        let saved_orientation = arrangement.cells.contributions[0].orientation;
        arrangement.cells.contributions[0].orientation = 0;
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contribution orientation is invalid"
            })
        ));
        arrangement.cells.contributions[0].orientation = saved_orientation;

        let saved_face = arrangement.cells.contributions[0].face;
        arrangement.cells.contributions[0].face = u32::MAX;
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &classifications),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contribution references an absent face"
            })
        ));
        arrangement.cells.contributions[0].face = saved_face;

        let duplicate_facet = arrangement.cells.facets[0];
        let duplicate_contribution = arrangement.cells.contributions[0];
        let saved_offsets = arrangement.cells.contribution_offsets.clone();
        arrangement.cells.facets.push(duplicate_facet);
        arrangement.cells.contributions.push(duplicate_contribution);
        let mut duplicate_offsets = saved_offsets.to_vec();
        duplicate_offsets.push(arrangement.cells.contributions.len() as u32);
        arrangement.cells.contribution_offsets = duplicate_offsets.into_boxed_slice();
        let mut duplicate_classifications = classifications.clone();
        duplicate_classifications.push(classifications[0]);
        assert!(matches!(
            arrangement.materialize_classifications(
                &decisions,
                &polygons,
                &duplicate_classifications,
            ),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contains duplicate triangle geometry"
            })
        ));
        arrangement.cells.facets.pop();
        arrangement.cells.contributions.pop();
        arrangement.cells.contribution_offsets = saved_offsets;

        let mut open_classifications = vec![0; classifications.len()];
        open_classifications[0] = classifications[0];
        assert!(matches!(
            arrangement.materialize_classifications(&decisions, &polygons, &open_classifications,),
            Err(HypermeshError::OpenOutput { .. })
        ));

        let output = arrangement
            .materialize_classifications(&decisions, &polygons, &classifications)
            .unwrap();
        assert_eq!(
            signed_six_volume(&output.vertices, &output.results[0]),
            Real::from(64)
        );
        assert!(
            output.results[0]
                .sources
                .iter()
                .all(|source| source.orientation == 1)
        );
        assert_eq!(decisions.certainty(), MeshCertainty::Certified);
    }

    #[test]
    fn output_certification_obeys_the_strict_and_approximate_512_terminal_policy() {
        let left = Real::pi() + Real::e();
        let right = Real::e() + Real::pi();
        let surface = SurfaceCorefinement {
            points: vec![
                Point3::new(Real::zero(), Real::zero(), Real::zero()),
                Point3::new(Real::one(), Real::one(), Real::zero()),
                Point3::new(left, right, Real::zero()),
            ],
            face_offsets: Vec::new().into_boxed_slice(),
            triangles: Vec::new(),
            constraint_offsets: Vec::new().into_boxed_slice(),
            constraints: Vec::new(),
            contact_offsets: Vec::new().into_boxed_slice(),
            contacts: Vec::new(),
        };
        let cells = SurfaceCellComplex {
            facets: vec![SurfaceFacet {
                vertices: [0, 1, 2],
                cells: [0, 0],
            }],
            contribution_offsets: vec![0, 0].into_boxed_slice(),
            contributions: Vec::new(),
            transitions: Vec::new().into_boxed_slice(),
            cell_windings: Vec::new().into_boxed_slice(),
            operand_count: 0,
            cell_count: 0,
            component_count: 0,
            radial_edge_count: 0,
            max_radial_degree: 0,
        };

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        assert!(matches!(
            certify_selected_surface_output(&strict, &surface, &cells, &[1]),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        assert_eq!(strict.certainty(), MeshCertainty::Certified);

        let approximate_context = MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        assert_eq!(
            certify_selected_surface_output(&approximate, &surface, &cells, &[1]).unwrap_err(),
            HypermeshError::SurfaceArrangementFailed {
                reason: "surface output contains a degenerate triangle",
            }
        );
        assert_eq!(
            approximate.certainty(),
            MeshCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn exact_radial_tetrahedron_builds_two_reciprocal_cells() {
        let polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 1);
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, surface, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(surface.points.len(), 4);
            assert_eq!(cells.facets.len(), 4);
            assert_eq!(cells.contributions.len(), 4);
            assert_eq!(cells.operand_count, 1);
            assert_eq!(cells.cell_count, 2);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.radial_edge_count, 6);
            assert_eq!(cells.max_radial_degree, 2);

            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0], vec![1]]);
            for facet in 0..cells.facets.len() {
                assert_eq!(cells.facet_contributions(facet).len(), 1);
                assert_eq!(cells.facet_transition(facet).len(), 1);
                assert_eq!(
                    cells.facet_classification(facet, crate::winding::BooleanOp::Union)
                        * cells.facet_contributions(facet)[0].orientation,
                    1
                );
                assert!(
                    cells.facets[facet]
                        .vertices
                        .iter()
                        .all(|vertex| (*vertex as usize) < surface.points.len())
                );
                assert!((cells.facet_contributions(facet)[0].face as usize) < polygons.len());
            }
        }
    }

    #[test]
    fn disconnected_nested_shells_receive_global_absolute_windings() {
        let mut polygons = tetrahedron([0, 0, 0], 10, 0, 0, 0, 0, 2);
        polygons.extend(tetrahedron([2, 2, 2], 1, 1, 4, 0, 1, 2));
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(cells.facets.len(), 8);
            assert_eq!(cells.cell_count, 4);
            assert_eq!(cells.component_count, 2);
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0, 0], vec![1, 0], vec![1, 0], vec![1, 1]]);

            let selected = |operation| {
                (0..cells.facets.len())
                    .filter(|&facet| cells.facet_classification(facet, operation) != 0)
                    .count()
            };
            assert_eq!(selected(crate::winding::BooleanOp::Union), 4);
            assert_eq!(selected(crate::winding::BooleanOp::Intersection), 4);
            assert_eq!(selected(crate::winding::BooleanOp::Difference), 8);
            assert_eq!(selected(crate::winding::BooleanOp::SymmetricDifference), 8);
        }
    }

    #[test]
    fn coincident_shells_bundle_multiplicity_before_cell_assembly() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
        polygons.extend(tetrahedron([0, 0, 0], 4, 1, 4, 0, 1, 2));
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, surface, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(surface.points.len(), 4);
            assert_eq!(cells.facets.len(), 4);
            assert_eq!(cells.contributions.len(), 8);
            assert_eq!(cells.cell_count, 2);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.max_radial_degree, 2);
            assert!(
                (0..cells.facets.len()).all(|facet| cells.facet_contributions(facet).len() == 2)
            );
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0, 0], vec![1, 1]]);

            let selected = |operation| {
                (0..cells.facets.len())
                    .filter(|&facet| cells.facet_classification(facet, operation) != 0)
                    .count()
            };
            assert_eq!(selected(crate::winding::BooleanOp::Union), 4);
            assert_eq!(selected(crate::winding::BooleanOp::Intersection), 4);
            assert_eq!(selected(crate::winding::BooleanOp::Difference), 0);
            assert_eq!(selected(crate::winding::BooleanOp::SymmetricDifference), 0);
        }
    }

    #[test]
    fn transverse_overlapping_shells_form_all_four_winding_cells() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
        polygons.extend(tetrahedron([1, 1, -1], 4, 1, 4, 0, 1, 2));
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.cell_count, 4);
            assert!(cells.max_radial_degree >= 4);
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0, 0], vec![0, 1], vec![1, 0], vec![1, 1]]);
            for operation in [
                crate::winding::BooleanOp::Union,
                crate::winding::BooleanOp::Intersection,
                crate::winding::BooleanOp::Difference,
                crate::winding::BooleanOp::SymmetricDifference,
            ] {
                assert!(selected_facets_are_closed(&cells, operation));
            }
        }
    }

    #[test]
    fn one_cell_table_evaluates_batched_arbitrary_multi_operand_expressions() {
        let mut polygons = tetrahedron([0, 0, 0], 12, 0, 0, 0, 0, 3);
        polygons.extend(tetrahedron([1, 1, 1], 6, 1, 4, 0, 1, 3));
        polygons.extend(tetrahedron([2, 2, 2], 1, 2, 8, 0, 2, 3));
        let nodes = [
            CellTruthNode::False,        // 0
            CellTruthNode::True,         // 1
            CellTruthNode::Operand(0),   // 2: A
            CellTruthNode::Operand(1),   // 3: B
            CellTruthNode::Operand(2),   // 4: C
            CellTruthNode::Not(3),       // 5: !B
            CellTruthNode::And([2, 5]),  // 6: A && !B
            CellTruthNode::Or([6, 4]),   // 7: (A && !B) || C
            CellTruthNode::Or([2, 3]),   // 8
            CellTruthNode::Or([8, 4]),   // 9: union
            CellTruthNode::And([2, 3]),  // 10
            CellTruthNode::And([10, 4]), // 11: intersection
            CellTruthNode::Not(4),       // 12: !C
            CellTruthNode::And([6, 12]), // 13: A - B - C
            CellTruthNode::Xor([2, 3]),  // 14
            CellTruthNode::Xor([14, 4]), // 15: parity
        ];
        let roots = [0_u32, 1, 9, 11, 13, 15, 7];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let arrangement = build_surface_arrangement(&decisions, &polygons).unwrap();
            let cells = &arrangement.cells;
            assert_eq!(cells.component_count, 3);
            assert_eq!(cells.cell_count, 6);
            let classified = cells.classify_expressions(&nodes, &roots).unwrap();
            assert_eq!(classified.expression_count, roots.len());
            assert_eq!(classified.facet_count, cells.facets.len());
            assert_eq!(
                classified.exterior_inside,
                [false, true, false, false, false, false, false]
            );
            let selected = (0..classified.expression_count)
                .map(|expression| {
                    (0..classified.facet_count)
                        .filter(|&facet| classified.classification(expression, facet) != 0)
                        .count()
                })
                .collect::<Vec<_>>();
            assert_eq!(selected, [0, 0, 4, 4, 8, 12, 12]);
            let batch = arrangement
                .materialize_program(
                    &decisions,
                    &polygons,
                    BooleanProgram::Expressions {
                        nodes: &nodes,
                        roots: &roots,
                    },
                )
                .unwrap();
            assert_eq!(batch.results.len(), roots.len());
            assert_eq!(batch.vertices.len(), 12);
            assert_eq!(
                batch
                    .results
                    .iter()
                    .map(|result| result.exterior_inside)
                    .collect::<Vec<_>>(),
                classified.exterior_inside
            );
            let mut used = vec![false; batch.vertices.len()];
            for (expression, &selected_count) in selected.iter().enumerate() {
                let classifications = (0..classified.facet_count)
                    .map(|facet| classified.classification(expression, facet))
                    .collect::<Vec<_>>();
                if selected_count != 0 {
                    assert!(facet_classifications_are_closed(cells, &classifications));
                }
                let output = &batch.results[expression];
                for &vertex in output.triangles.iter().flatten() {
                    used[vertex as usize] = true;
                }
                assert_materialized_batch_output(
                    &decisions,
                    &polygons,
                    &batch.vertices,
                    output,
                    selected_count,
                );
            }
            assert!(used.into_iter().all(|is_used| is_used));

            assert!(matches!(
                cells.classify_expressions(&[CellTruthNode::Operand(3)], &[0]),
                Err(HypermeshError::InvalidBooleanProgram { .. })
            ));
            assert!(matches!(
                cells.classify_expressions(&[CellTruthNode::Not(0)], &[0]),
                Err(HypermeshError::InvalidBooleanProgram { .. })
            ));
            assert!(matches!(
                cells.classify_expressions(&nodes, &[u32::MAX]),
                Err(HypermeshError::InvalidBooleanProgram { .. })
            ));
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn edge_tangent_closed_shells_build_one_nonmanifold_radial_component() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
        polygons.extend(tetrahedron_from_vertices(
            [p(0, 0, 0), p(4, 0, 0), p(0, -4, 0), p(0, 0, -4)],
            1,
            4,
            0,
            1,
            2,
        ));
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.cell_count, 3);
            assert_eq!(cells.max_radial_degree, 4);
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0, 0], vec![0, 1], vec![1, 0]]);
            assert_eq!(
                (0..cells.facets.len())
                    .filter(|&facet| {
                        cells.facet_classification(facet, crate::winding::BooleanOp::Union) != 0
                    })
                    .count(),
                8
            );
            assert!(selected_facets_are_closed(
                &cells,
                crate::winding::BooleanOp::Union
            ));
            assert_eq!(
                (0..cells.facets.len())
                    .filter(|&facet| {
                        cells.facet_classification(facet, crate::winding::BooleanOp::Intersection)
                            != 0
                    })
                    .count(),
                0
            );
        }
    }

    #[test]
    fn opposite_side_face_coincidence_classifies_shared_interface_once() {
        let a = p(4, 0, 0);
        let b = p(0, 4, 0);
        let c = p(0, 0, 4);
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
        polygons.extend(tetrahedron_from_vertices(
            [a, b, c, p(4, 4, 4)],
            1,
            4,
            0,
            1,
            2,
        ));
        let b_minus_a = [
            CellTruthNode::Operand(0),
            CellTruthNode::Operand(1),
            CellTruthNode::Not(0),
            CellTruthNode::And([1, 2]),
        ];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let arrangement = build_surface_arrangement(&decisions, &polygons).unwrap();
            let cells = &arrangement.cells;
            assert_eq!(cells.facets.len(), 7);
            assert_eq!(cells.contributions.len(), 8);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.cell_count, 3);
            let selected = |operation| {
                (0..cells.facets.len())
                    .filter(|&facet| cells.facet_classification(facet, operation) != 0)
                    .count()
            };
            assert_eq!(selected(crate::winding::BooleanOp::Union), 6);
            assert_eq!(selected(crate::winding::BooleanOp::Intersection), 0);
            assert_eq!(selected(crate::winding::BooleanOp::Difference), 4);
            assert_eq!(selected(crate::winding::BooleanOp::SymmetricDifference), 6);
            let reverse = cells.classify_expressions(&b_minus_a, &[3]).unwrap();
            let reverse_classifications = (0..reverse.facet_count)
                .map(|facet| reverse.classification(0, facet))
                .collect::<Vec<_>>();
            assert_eq!(
                reverse_classifications
                    .iter()
                    .filter(|classification| **classification != 0)
                    .count(),
                4
            );
            assert!(facet_classifications_are_closed(
                cells,
                &reverse_classifications
            ));
            let reverse_output = arrangement
                .materialize_classifications(&decisions, &polygons, &reverse_classifications)
                .unwrap();
            assert_materialized_output(&decisions, &polygons, &reverse_output, 4);
            assert!(selected_facets_are_closed(
                cells,
                crate::winding::BooleanOp::Union
            ));
            assert!(selected_facets_are_closed(
                cells,
                crate::winding::BooleanOp::Difference
            ));
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn nested_negative_shell_forms_an_exact_cavity() {
        let mut polygons = tetrahedron([0, 0, 0], 12, 0, 0, 0, 0, 1);
        let mut cavity = tetrahedron([2, 2, 2], 2, 0, 4, 4, 0, 1);
        for polygon in &mut cavity {
            polygon.delta_w[0] = -1;
        }
        polygons.extend(cavity);

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(cells.facets.len(), 8);
            assert_eq!(cells.cell_count, 4);
            assert_eq!(cells.component_count, 2);
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0], vec![0], vec![1], vec![1]]);
            assert_eq!(
                (0..cells.facets.len())
                    .filter(|&facet| {
                        cells.facet_classification(facet, crate::winding::BooleanOp::Union) != 0
                    })
                    .count(),
                8
            );
            assert!(selected_facets_are_closed(
                &cells,
                crate::winding::BooleanOp::Union
            ));
        }
    }

    #[test]
    fn genus_one_voxel_ring_has_one_inside_and_one_outside_cell() {
        let polygons = voxel_ring(0, 1);
        assert_eq!(polygons.len(), 64);
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, surface, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(surface.points.len(), 32);
            assert_eq!(cells.facets.len(), 64);
            assert_eq!(cells.cell_count, 2);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.radial_edge_count, 96);
            assert_eq!(cells.max_radial_degree, 2);
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0], vec![1]]);
            assert_eq!(
                (0..cells.facets.len())
                    .filter(|&facet| {
                        cells.facet_classification(facet, crate::winding::BooleanOp::Union) != 0
                    })
                    .count(),
                64
            );
            assert!(selected_facets_are_closed(
                &cells,
                crate::winding::BooleanOp::Union
            ));
        }
    }

    #[test]
    fn same_operand_transverse_pwn_preserves_winding_multiplicity() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 1);
        polygons.extend(tetrahedron([1, 1, -1], 4, 0, 4, 4, 0, 1));
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(cells.component_count, 1);
            assert_eq!(cells.cell_count, 4);
            assert!(cells.max_radial_degree >= 4);
            let mut windings = (0..cells.cell_count)
                .map(|cell| cells.cell_winding(cell).to_vec())
                .collect::<Vec<_>>();
            windings.sort_unstable();
            assert_eq!(windings, [vec![0], vec![1], vec![1], vec![2]]);
            let selected = (0..cells.facets.len())
                .filter(|&facet| {
                    cells.facet_classification(facet, crate::winding::BooleanOp::Union) != 0
                })
                .count();
            assert!(selected > 0);
            assert!(selected < cells.facets.len());
            assert!(selected_facets_are_closed(
                &cells,
                crate::winding::BooleanOp::Union
            ));
        }
    }

    #[test]
    fn exact_embedding_and_operand_permutation_preserve_cell_truth() {
        let first = [p(0, 0, 0), p(4, 0, 0), p(0, 4, 0), p(0, 0, 4)];
        let second = [p(1, 1, -1), p(5, 1, -1), p(1, 5, -1), p(1, 1, 3)];
        let transform = |[x, y, z]: [i64; 3]| p(100 - 7 * z, -50 + 7 * x, 33 + 7 * y);
        let transformed_first = [
            transform([0, 0, 0]),
            transform([4, 0, 0]),
            transform([0, 4, 0]),
            transform([0, 0, 4]),
        ];
        let transformed_second = [
            transform([1, 1, -1]),
            transform([5, 1, -1]),
            transform([1, 5, -1]),
            transform([1, 1, 3]),
        ];

        let build =
            |left: [Point3; 4], right: [Point3; 4], left_operand: usize, right_operand: usize| {
                let mut polygons = tetrahedron_from_vertices(left, 0, 0, 0, left_operand, 2);
                polygons.extend(tetrahedron_from_vertices(right, 1, 4, 0, right_operand, 2));
                polygons
            };
        let base = build(first.clone(), second.clone(), 0, 1);
        let transformed = build(transformed_first, transformed_second, 0, 1);
        let permuted = build(first, second, 1, 0);
        let mut reordered = base.clone();
        reordered.reverse();

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let (base_certainty, base_surface, base_cells) = arranged_cells(&base, policy);
            let (transformed_certainty, transformed_surface, transformed_cells) =
                arranged_cells(&transformed, policy);
            let (permuted_certainty, _, permuted_cells) = arranged_cells(&permuted, policy);
            let (reordered_certainty, reordered_surface, reordered_cells) =
                arranged_cells(&reordered, policy);
            assert_eq!(base_certainty, MeshCertainty::Certified);
            assert_eq!(transformed_certainty, base_certainty);
            assert_eq!(permuted_certainty, base_certainty);
            assert_eq!(reordered_certainty, base_certainty);
            assert_eq!(transformed_surface.points.len(), base_surface.points.len());
            assert_eq!(reordered_surface.points.len(), base_surface.points.len());
            assert_eq!(transformed_cells.facets.len(), base_cells.facets.len());
            assert_eq!(reordered_cells.facets.len(), base_cells.facets.len());
            assert_eq!(transformed_cells.cell_count, base_cells.cell_count);
            assert_eq!(reordered_cells.cell_count, base_cells.cell_count);
            assert_eq!(
                transformed_cells.component_count,
                base_cells.component_count
            );
            assert_eq!(reordered_cells.component_count, base_cells.component_count);
            assert_eq!(
                transformed_cells.max_radial_degree,
                base_cells.max_radial_degree
            );
            assert_eq!(
                reordered_cells.max_radial_degree,
                base_cells.max_radial_degree
            );

            let signature = |cells: &SurfaceCellComplex| {
                let mut windings = (0..cells.cell_count)
                    .map(|cell| cells.cell_winding(cell).to_vec())
                    .collect::<Vec<_>>();
                windings.sort_unstable();
                windings
            };
            let base_signature = signature(&base_cells);
            assert_eq!(signature(&transformed_cells), base_signature);
            assert_eq!(signature(&reordered_cells), base_signature);
            let mut swapped_signature = signature(&permuted_cells)
                .into_iter()
                .map(|winding| vec![winding[1], winding[0]])
                .collect::<Vec<_>>();
            swapped_signature.sort_unstable();
            assert_eq!(swapped_signature, base_signature);

            for operation in [
                crate::winding::BooleanOp::Union,
                crate::winding::BooleanOp::Intersection,
                crate::winding::BooleanOp::SymmetricDifference,
            ] {
                let selected = |cells: &SurfaceCellComplex| {
                    (0..cells.facets.len())
                        .filter(|&facet| cells.facet_classification(facet, operation) != 0)
                        .count()
                };
                assert_eq!(selected(&transformed_cells), selected(&base_cells));
                assert_eq!(selected(&reordered_cells), selected(&base_cells));
                assert_eq!(selected(&permuted_cells), selected(&base_cells));
                assert!(selected_facets_are_closed(&base_cells, operation));
                assert!(selected_facets_are_closed(&transformed_cells, operation));
                assert!(selected_facets_are_closed(&reordered_cells, operation));
                assert!(selected_facets_are_closed(&permuted_cells, operation));
            }
        }
    }

    #[test]
    fn forty_operands_share_one_arrangement_and_batched_truth_table() {
        const OPERAND_COUNT: usize = 40;
        let mut polygons = Vec::with_capacity(OPERAND_COUNT * 4);
        for operand in 0..OPERAND_COUNT {
            let x = i64::try_from(operand).unwrap() * 3;
            polygons.extend(tetrahedron(
                [x, 0, 0],
                1,
                operand,
                operand * 4,
                0,
                operand,
                OPERAND_COUNT,
            ));
        }
        let mut nodes = (0..OPERAND_COUNT)
            .map(|operand| CellTruthNode::Operand(u32::try_from(operand).unwrap()))
            .collect::<Vec<_>>();
        let mut union = 0_u32;
        for operand in 1..OPERAND_COUNT {
            nodes.push(CellTruthNode::Or([union, u32::try_from(operand).unwrap()]));
            union = u32::try_from(nodes.len() - 1).unwrap();
        }
        let mut intersection = 0_u32;
        for operand in 1..OPERAND_COUNT {
            nodes.push(CellTruthNode::And([
                intersection,
                u32::try_from(operand).unwrap(),
            ]));
            intersection = u32::try_from(nodes.len() - 1).unwrap();
        }
        let mut parity = 0_u32;
        for operand in 1..OPERAND_COUNT {
            nodes.push(CellTruthNode::Xor([
                parity,
                u32::try_from(operand).unwrap(),
            ]));
            parity = u32::try_from(nodes.len() - 1).unwrap();
        }

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let arrangement = build_surface_arrangement(&decisions, &polygons).unwrap();
            let cells = &arrangement.cells;
            assert_eq!(cells.operand_count, OPERAND_COUNT);
            assert_eq!(cells.component_count as usize, OPERAND_COUNT);
            assert_eq!(cells.cell_count as usize, OPERAND_COUNT * 2);
            let classified = cells
                .classify_expressions(&nodes, &[union, intersection, parity])
                .unwrap();
            let selected = (0..classified.expression_count)
                .map(|expression| {
                    (0..classified.facet_count)
                        .filter(|&facet| classified.classification(expression, facet) != 0)
                        .count()
                })
                .collect::<Vec<_>>();
            assert_eq!(selected, [OPERAND_COUNT * 4, 0, OPERAND_COUNT * 4]);
            let batch = arrangement
                .materialize_expression_classifications(&decisions, &polygons, &classified)
                .unwrap();
            assert_eq!(batch.results.len(), 3);
            assert_eq!(batch.vertices.len(), OPERAND_COUNT * 4);
            for (expression, &selected_count) in selected.iter().enumerate() {
                let classifications = (0..classified.facet_count)
                    .map(|facet| classified.classification(expression, facet))
                    .collect::<Vec<_>>();
                if selected_count != 0 {
                    assert!(facet_classifications_are_closed(cells, &classifications));
                }
                assert_materialized_batch_output(
                    &decisions,
                    &polygons,
                    &batch.vertices,
                    &batch.results[expression],
                    selected_count,
                );
            }
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn disconnected_shell_scaling_preserves_every_exact_component() {
        let shell_count = std::env::var("HYPERMESH_TOPOLOGY_SHELLS")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .unwrap_or(32);
        assert!(shell_count > 0);
        let mut polygons = Vec::with_capacity(shell_count.saturating_mul(4));
        for shell in 0..shell_count {
            let x = i64::try_from(shell).unwrap().checked_mul(3).unwrap();
            polygons.extend(tetrahedron([x, 0, 0], 1, 0, shell * 4, shell * 4, 0, 1));
        }
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let arrangement = build_surface_arrangement(&decisions, &polygons).unwrap();
            let surface = &arrangement.corefinement;
            let cells = &arrangement.cells;
            assert_eq!(surface.points.len(), shell_count * 4);
            assert_eq!(cells.facets.len(), shell_count * 4);
            assert_eq!(cells.cell_count as usize, shell_count * 2);
            assert_eq!(cells.component_count as usize, shell_count);
            assert_eq!(cells.radial_edge_count as usize, shell_count * 6);
            assert_eq!(cells.max_radial_degree, 2);
            assert_eq!(
                (0..cells.facets.len())
                    .filter(|&facet| {
                        cells.facet_classification(facet, crate::winding::BooleanOp::Union) != 0
                    })
                    .count(),
                shell_count * 4
            );
            assert!(selected_facets_are_closed(
                cells,
                crate::winding::BooleanOp::Union
            ));
            let output = arrangement
                .materialize_operation(&decisions, &polygons, crate::winding::BooleanOp::Union)
                .unwrap();
            assert_eq!(output.vertices.len(), shell_count * 4);
            assert_eq!(output.results[0].triangles.len(), shell_count * 4);
            assert_eq!(output.results[0].sources.len(), shell_count * 4);
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn open_surface_is_rejected_by_exact_radial_topology() {
        let polygons = vec![triangle(
            [p(0, 0, 0), p(4, 0, 0), p(0, 4, 0)],
            0,
            0,
            [0, 1, 2],
        )];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
            let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();
            let source_bvh = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
                .unwrap()
                .into_query_hierarchy(&polygons)
                .unwrap();
            assert_eq!(
                assemble_surface_cells(&decisions, &polygons, &surface, &[], &source_bvh)
                    .unwrap_err(),
                HypermeshError::SurfaceArrangementFailed {
                    reason: "surface radial edge has fewer than two geometric rays",
                }
            );
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }
    }

    #[test]
    fn surface_topology_rejects_malformed_incidence_and_winding_dimensions() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 2);
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let mut surface = corefine_surface(&decisions, &polygons, &graph).unwrap();
        let source_bvh = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();

        let malformed_pair_sets = [
            vec![(u64::from(1_u32) << u32::BITS) | 1],
            vec![source_face_pair_key(0, polygons.len() as u32).unwrap()],
            vec![
                source_face_pair_key(0, 2).unwrap(),
                source_face_pair_key(0, 1).unwrap(),
            ],
        ];
        for malformed in malformed_pair_sets {
            assert_eq!(
                assemble_surface_cells(&decisions, &polygons, &surface, &malformed, &source_bvh,)
                    .unwrap_err(),
                HypermeshError::SurfaceArrangementFailed {
                    reason: "radially separated source-face pairs are not canonical",
                }
            );
        }

        polygons[1].delta_w.pop();
        assert_eq!(
            assemble_surface_cells(&decisions, &polygons, &surface, &[], &source_bvh).unwrap_err(),
            HypermeshError::WindingDimensionMismatch {
                expected: 2,
                actual: 1,
            }
        );
        polygons[1].delta_w.push(0);

        surface.triangles[0][0] = u32::MAX;
        assert_eq!(
            assemble_surface_cells(&decisions, &polygons, &surface, &[], &source_bvh).unwrap_err(),
            HypermeshError::SurfaceArrangementFailed {
                reason: "surface facet references an absent arrangement point",
            }
        );
    }

    #[test]
    fn coincident_facet_transition_overflow_is_reported_atomically() {
        let mut polygons = tetrahedron([0, 0, 0], 4, 0, 0, 0, 0, 1);
        polygons.extend(tetrahedron([0, 0, 0], 4, 1, 4, 0, 0, 1));
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();
        let source_bvh = ExactBvh::build_for_query_hierarchy_decision(&decisions, &polygons)
            .unwrap()
            .into_query_hierarchy(&polygons)
            .unwrap();
        for polygon in &mut polygons[..4] {
            polygon.delta_w[0] = i32::MAX;
        }
        for polygon in &mut polygons[4..] {
            polygon.delta_w[0] = 1;
        }
        assert_eq!(
            assemble_surface_cells(&decisions, &polygons, &surface, &[], &source_bvh).unwrap_err(),
            HypermeshError::WindingOverflow
        );
    }

    fn test_split_line(face: usize, plane: u32) -> ConstructionEdgeIdentity {
        let mut planes = [
            pairwise_support_identity(face).unwrap().unwrap(),
            ConstructionPlaneIdentity {
                mesh: u32::MAX - 1,
                plane,
            },
        ];
        planes.sort_unstable();
        ConstructionEdgeIdentity::Split { planes }
    }

    #[test]
    fn crossing_split_lines_produce_the_canonical_plane_triple_identity() {
        let support = pairwise_support_identity(0).unwrap().unwrap();
        let identity = intersect_arrangement_lines(
            &pairwise_split_line(0, 1).unwrap(),
            &pairwise_split_line(0, 2).unwrap(),
            support,
        )
        .unwrap();
        let mut planes = [
            pairwise_support_identity(0).unwrap().unwrap(),
            pairwise_support_identity(1).unwrap().unwrap(),
            pairwise_support_identity(2).unwrap().unwrap(),
        ];
        planes.sort_unstable();
        assert_eq!(
            identity,
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::PlaneTriple {
                planes,
            })
        );
    }

    fn insert_test_point(
        decisions: &DecisionContext,
        arena: &mut ArrangementPointArena,
        vertex: &mut u32,
        point: Point3,
    ) -> u32 {
        let identity = ArrangementPointIdentity::Construction(ConstructionVertexIdentity::Source {
            mesh: u32::MAX - 2,
            vertex: *vertex,
        });
        *vertex += 1;
        arena.insert(decisions, identity, point).unwrap()
    }

    #[test]
    fn bounded_face_pslg_keeps_disconnected_inner_cells() {
        let polygons = vec![
            triangle([p(0, 0, 0), p(20, 0, 0), p(0, 20, 0)], 0, 0, [0, 1, 2]),
            triangle([p(2, 2, 0), p(4, 2, 0), p(2, 4, 0)], 1, 1, [0, 1, 2]),
            triangle([p(8, 2, 0), p(10, 2, 0), p(8, 4, 0)], 2, 2, [0, 1, 2]),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
            let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();

            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_eq!(surface.face_triangles(0).len(), 13);
            assert_eq!(surface.face_constraints(0).len(), 9);
            assert!(surface.face_contacts(0).is_empty());
            assert_constraints_are_edges(&surface, 0);
        }
    }

    #[test]
    fn crossing_face_constraints_share_one_triple_point() {
        let polygons = vec![
            triangle([p(0, 0, 0), p(20, 0, 0), p(0, 20, 0)], 0, 0, [0, 1, 2]),
            triangle([p(5, -2, -2), p(5, 20, -2), p(5, -2, 2)], 1, 1, [0, 1, 2]),
            triangle([p(-2, 5, -2), p(20, 5, -2), p(-2, 5, 2)], 2, 2, [0, 1, 2]),
        ];
        let decisions = crate::test_support::approximate_decisions();
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();

        let center = surface
            .points
            .iter()
            .position(|point| point == &p(5, 5, 0))
            .expect("the two cuts have one exact crossing");
        assert!(surface.face_triangles(0).len() >= 8);
        for face in 0..polygons.len() {
            assert!(
                surface
                    .face_triangles(face)
                    .iter()
                    .any(|triangle| triangle.contains(&(center as u32)))
            );
            assert_constraints_are_edges(&surface, face);
        }
    }

    #[test]
    fn opposite_winding_coincident_faces_reuse_one_point_set() {
        let first = triangle([p(0, 0, 0), p(6, 0, 0), p(0, 6, 0)], 0, 0, [0, 1, 2]);
        let second = triangle([p(0, 6, 0), p(6, 0, 0), p(0, 0, 0)], 1, 1, [0, 1, 2]);
        let polygons = [first, second];
        let decisions = crate::test_support::approximate_decisions();
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();

        let canonical = |face: usize| {
            let mut triangles = surface.face_triangles(face).to_vec();
            for triangle in &mut triangles {
                triangle.sort_unstable();
            }
            triangles.sort_unstable();
            triangles
        };
        assert_eq!(surface.points.len(), 3);
        assert_eq!(canonical(0), canonical(1));
        assert_constraints_are_edges(&surface, 0);
        assert_constraints_are_edges(&surface, 1);
    }

    #[test]
    fn partial_coplanar_overlap_reuses_the_same_bounded_cell_triangulation() {
        let polygons = [
            triangle([p(0, 0, 0), p(6, 0, 0), p(0, 6, 0)], 0, 0, [0, 1, 2]),
            triangle([p(5, 5, 0), p(-1, 5, 0), p(5, -1, 0)], 1, 1, [0, 1, 2]),
        ];
        let overlap_points = [
            p(4, 0, 0),
            p(5, 0, 0),
            p(5, 1, 0),
            p(1, 5, 0),
            p(0, 5, 0),
            p(0, 4, 0),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
            let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();
            let overlap = surface
                .points
                .iter()
                .enumerate()
                .filter_map(|(index, point)| {
                    overlap_points
                        .contains(point)
                        .then_some(u32::try_from(index).unwrap())
                })
                .collect::<BTreeSet<_>>();
            assert_eq!(overlap.len(), overlap_points.len());

            let canonical_overlap = |face: usize| {
                let mut triangles = surface
                    .face_triangles(face)
                    .iter()
                    .copied()
                    .filter(|triangle| triangle.iter().all(|point| overlap.contains(point)))
                    .collect::<Vec<_>>();
                for triangle in &mut triangles {
                    triangle.sort_unstable();
                }
                triangles.sort_unstable();
                triangles
            };
            let left = canonical_overlap(0);
            let right = canonical_overlap(1);
            assert_eq!(left.len(), overlap_points.len() - 2);
            assert_eq!(left, right);
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_constraints_are_edges(&surface, 0);
            assert_constraints_are_edges(&surface, 1);
        }
    }

    #[test]
    fn collinear_overlaps_and_t_junctions_split_into_one_edge_set() {
        let polygon = triangle([p(0, 0, 0), p(24, 0, 0), p(0, 24, 0)], 0, 0, [0, 1, 2]);
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let mut arena = ArrangementPointArena::with_capacity(11).unwrap();
            let boundary = add_source_boundary(&decisions, &polygon, &mut arena).unwrap();
            let mut work = FaceWork {
                boundary,
                ..FaceWork::default()
            };
            let mut vertex = 0;
            for (line, endpoints) in [
                (0, [p(2, 5, 0), p(18, 5, 0)]),
                (1, [p(8, 5, 0), p(16, 5, 0)]),
                (2, [p(10, 2, 0), p(10, 5, 0)]),
                (3, [p(12, 5, 0), p(12, 8, 0)]),
            ] {
                let endpoints = endpoints
                    .map(|point| insert_test_point(&decisions, &mut arena, &mut vertex, point));
                work.constraints.push(RawConstraint {
                    endpoints,
                    line: test_split_line(0, line),
                });
            }
            let result = corefine_face(&decisions, 0, &polygon, &work, &mut arena).unwrap();
            assert_eq!(arena.points.len(), 11);
            assert_eq!(result.constraints.len(), 10);
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_face_result_constraints_are_edges(&result);
        }
    }

    #[test]
    fn dense_crossing_discovery_exhausts_the_finite_grid_without_a_pass_limit() {
        const LINE_COUNT: u32 = 17;
        let polygon = triangle([p(0, 0, 0), p(40, 0, 0), p(0, 40, 0)], 0, 0, [0, 1, 2]);
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let mut arena = ArrangementPointArena::with_capacity(3).unwrap();
            let boundary = add_source_boundary(&decisions, &polygon, &mut arena).unwrap();
            let mut work = FaceWork {
                boundary,
                ..FaceWork::default()
            };
            work.constraints
                .try_reserve((LINE_COUNT * 2) as usize)
                .unwrap();
            let mut vertex = 0;
            for coordinate in 1..=LINE_COUNT {
                let endpoints = [p(coordinate.into(), 0, 0), p(coordinate.into(), 18, 0)]
                    .map(|point| insert_test_point(&decisions, &mut arena, &mut vertex, point));
                work.constraints.push(RawConstraint {
                    endpoints,
                    line: test_split_line(0, coordinate - 1),
                });
            }
            for coordinate in 1..=LINE_COUNT {
                let endpoints = [p(0, coordinate.into(), 0), p(18, coordinate.into(), 0)]
                    .map(|point| insert_test_point(&decisions, &mut arena, &mut vertex, point));
                work.constraints.push(RawConstraint {
                    endpoints,
                    line: test_split_line(0, LINE_COUNT + coordinate - 1),
                });
            }
            let result = corefine_face(&decisions, 0, &polygon, &work, &mut arena).unwrap();
            assert_eq!(
                arena.points.len(),
                3 + (LINE_COUNT * LINE_COUNT + 4 * LINE_COUNT) as usize
            );
            assert_eq!(
                result.constraints.len(),
                3 + (2 * LINE_COUNT) as usize + (2 * LINE_COUNT * (LINE_COUNT + 1)) as usize
            );
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_face_result_constraints_are_edges(&result);
        }
    }

    #[test]
    fn isolated_face_contact_is_retained_as_a_triangulation_vertex() {
        let polygon = triangle([p(0, 0, 0), p(8, 0, 0), p(0, 8, 0)], 0, 0, [0, 1, 2]);
        let decisions = crate::test_support::approximate_decisions();
        let mut arena = ArrangementPointArena::with_capacity(4).unwrap();
        let boundary = add_source_boundary(&decisions, &polygon, &mut arena).unwrap();
        let mut work = FaceWork {
            boundary,
            ..FaceWork::default()
        };
        let contact = arena
            .insert(
                &decisions,
                ArrangementPointIdentity::Construction(ConstructionVertexIdentity::PlaneTriple {
                    planes: [
                        ConstructionPlaneIdentity { mesh: 0, plane: 0 },
                        ConstructionPlaneIdentity { mesh: 1, plane: 0 },
                        ConstructionPlaneIdentity { mesh: 2, plane: 0 },
                    ],
                }),
                p(2, 2, 0),
            )
            .unwrap();
        work.contacts.push(contact);
        let result = corefine_face(&decisions, 0, &polygon, &work, &mut arena).unwrap();
        assert_eq!(result.contacts, [contact]);
        assert_eq!(result.triangles.len(), 3);
        assert!(
            result
                .triangles
                .iter()
                .all(|triangle| triangle.contains(&contact))
        );
        assert_face_result_constraints_are_edges(&result);
    }

    #[test]
    fn point_aliasing_obeys_strict_and_approximate_512_terminal_policy() {
        let left = p(0, 0, 0);
        let mut symbolic_left = left.clone();
        symbolic_left.x = Real::pi() + Real::e();
        let mut symbolic_right = left;
        symbolic_right.x = Real::e() + Real::pi();
        let first_identity =
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::Source {
                mesh: 0,
                vertex: 0,
            });
        let second_identity =
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::Source {
                mesh: 1,
                vertex: 0,
            });

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        let mut strict_arena = ArrangementPointArena::with_capacity(2).unwrap();
        strict_arena
            .insert(&strict, first_identity.clone(), symbolic_left.clone())
            .unwrap();
        assert!(matches!(
            strict_arena.insert(&strict, second_identity.clone(), symbolic_right.clone()),
            Err(HypermeshError::PredicateUndecided { .. })
        ));

        let approximate_context = MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        let mut approximate_arena = ArrangementPointArena::with_capacity(2).unwrap();
        let first = approximate_arena
            .insert(&approximate, first_identity, symbolic_left)
            .unwrap();
        let second = approximate_arena
            .insert(&approximate, second_identity, symbolic_right)
            .unwrap();
        assert_eq!(first, second);
        assert_eq!(
            approximate.certainty(),
            MeshCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn radial_equality_obeys_strict_and_approximate_512_terminal_policy() {
        let symbolic_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
        let points = [
            p(0, 0, 0),
            p(1, 0, 0),
            p(0, 1, 0),
            Point3::new(Real::zero(), Real::one(), symbolic_zero),
        ];

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        assert!(matches!(
            same_radial_ray(&strict, &points, [0, 1], 2, 3),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        assert_eq!(strict.certainty(), MeshCertainty::Certified);

        let approximate_context = MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        assert!(same_radial_ray(&approximate, &points, [0, 1], 2, 3).unwrap());
        assert_eq!(
            approximate.certainty(),
            MeshCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn retained_two_face_transverse_ring_matches_complete_radial_order() {
        let point_sets = [
            [
                p(0, 0, 0),
                p(1, 0, 0),
                p(0, 1, 0),
                p(0, 0, 1),
                p(0, -1, 0),
                p(0, 0, -1),
            ],
            [
                p(0, 0, 0),
                p(2, 1, 3),
                p(0, 1, 0),
                p(0, 0, 1),
                p(10, 3, 15),
                p(-4, -2, -9),
            ],
        ];
        let facets = (2_u32..=5)
            .map(|opposite| PendingFacet {
                vertices: [0, 1, opposite],
            })
            .collect::<Vec<_>>();
        let base_uses = [
            EdgeUse {
                edge: [0, 1],
                facet: 0,
                opposite: 2,
            },
            EdgeUse {
                edge: [0, 1],
                facet: 1,
                opposite: 3,
            },
            EdgeUse {
                edge: [0, 1],
                facet: 2,
                opposite: 4,
            },
            EdgeUse {
                edge: [0, 1],
                facet: 3,
                opposite: 5,
            },
        ];
        let contribution_offsets = [0, 1, 2, 3, 4];
        let contributions = [
            FacetContribution {
                face: 7,
                orientation: 1,
            },
            FacetContribution {
                face: 11,
                orientation: -1,
            },
            FacetContribution {
                face: 7,
                orientation: -1,
            },
            FacetContribution {
                face: 11,
                orientation: 1,
            },
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            for points in &point_sets {
                for first in 0..4 {
                    for second in 0..4 {
                        for third in 0..4 {
                            for fourth in 0..4 {
                                if [first, second, third, fourth]
                                    .iter()
                                    .copied()
                                    .collect::<BTreeSet<_>>()
                                    .len()
                                    != 4
                                {
                                    continue;
                                }
                                let uses = [
                                    base_uses[first],
                                    base_uses[second],
                                    base_uses[third],
                                    base_uses[fourth],
                                ];
                                let mut retained = CellDisjointSets::new(8).unwrap();
                                assert!(
                                    try_assemble_two_face_transverse_ring(
                                        &decisions,
                                        points,
                                        &facets,
                                        [0, 1],
                                        &uses,
                                        &contribution_offsets,
                                        &contributions,
                                        &mut retained,
                                    )
                                    .unwrap()
                                );
                                let mut complete = CellDisjointSets::new(8).unwrap();
                                assemble_radial_ring(
                                    &decisions,
                                    points,
                                    &facets,
                                    [0, 1],
                                    &uses,
                                    &mut Vec::new(),
                                    &mut Vec::new(),
                                    &mut complete,
                                )
                                .unwrap();
                                assert_eq!(
                                    retained.into_cells().unwrap(),
                                    complete.into_cells().unwrap()
                                );
                            }
                        }
                    }
                }
            }
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
        }

        let coplanar_points = [
            p(0, 0, 0),
            p(1, 0, 0),
            p(0, 1, 0),
            p(0, 2, 0),
            p(0, -1, 0),
            p(0, -2, 0),
        ];
        let decisions = crate::test_support::approximate_decisions();
        let mut declined = CellDisjointSets::new(8).unwrap();
        assert!(
            !try_assemble_two_face_transverse_ring(
                &decisions,
                &coplanar_points,
                &facets,
                [0, 1],
                &base_uses,
                &contribution_offsets,
                &contributions,
                &mut declined,
            )
            .unwrap()
        );
        assert_eq!(declined.into_cells().unwrap().1, 8);
    }

    #[test]
    fn sign_only_radial_dot_matches_the_factored_exact_polynomial() {
        let mut vectors = Vec::with_capacity(27);
        let zero = Rational::from(0);
        for x in -1_i64..=1 {
            for y in -1_i64..=1 {
                for z in -1_i64..=1 {
                    vectors.push(Vector3::from_xyz(
                        Real::from(x),
                        Real::from(y),
                        Real::from(z),
                    ));
                }
            }
        }
        for direction in &vectors {
            for left in &vectors {
                for right in &vectors {
                    let factored = direction.dot(direction) * left.dot(right)
                        - direction.dot(left) * direction.dot(right);
                    let ordering =
                        exact_radial_perpendicular_dot_ordering(direction, left, right).unwrap();
                    assert_eq!(
                        ordering,
                        factored
                            .exact_rational_ref()
                            .unwrap()
                            .partial_cmp(&zero)
                            .unwrap()
                    );
                }
            }
        }
    }

    #[test]
    fn retained_radial_separation_requires_shared_face_or_proved_pair() {
        let contribution_offsets = [0, 1, 3, 4, 5];
        let contributions = [
            FacetContribution {
                face: 7,
                orientation: 1,
            },
            FacetContribution {
                face: 11,
                orientation: -1,
            },
            FacetContribution {
                face: 2,
                orientation: 1,
            },
            FacetContribution {
                face: 3,
                orientation: 1,
            },
            FacetContribution {
                face: 7,
                orientation: -1,
            },
        ];
        let separated = [
            source_face_pair_key(2, 7).unwrap(),
            source_face_pair_key(3, 9).unwrap(),
        ];

        assert!(
            facets_have_retained_radial_separation(
                0,
                1,
                &contribution_offsets,
                &contributions,
                &separated,
            )
            .unwrap()
        );
        assert!(
            !facets_have_retained_radial_separation(
                0,
                2,
                &contribution_offsets,
                &contributions,
                &separated,
            )
            .unwrap()
        );
        assert!(
            facets_have_retained_radial_separation(
                0,
                3,
                &contribution_offsets,
                &contributions,
                &[],
            )
            .unwrap()
        );
        assert!(matches!(
            facets_have_retained_radial_separation(0, 1, &[1, 0], &contributions, &separated,),
            Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface output facet contribution storage is malformed"
            })
        ));
    }
}
