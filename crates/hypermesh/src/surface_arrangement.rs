//! Exact source-face corefinement prototype.
//!
//! This module is test-only until the new surface-arrangement orchestrator is
//! atomically substituted for EMBER. It deliberately exercises the production
//! scalar, predicate-policy, intersection-graph, and Hypertri paths while
//! adding no shipped second engine.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use hyperlattice::{Point3, Real, Vector3};

use crate::bvh::ExactBvh;
use crate::context::{DecisionContext, MeshCertainty};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, compare_real_decision};
use crate::intersection::{
    PairwiseIntersectionEventIds, PairwiseIntersectionGraph, pairwise_support_identity,
};
use crate::point_interner::PointInterner;
use crate::polygon::{
    ApproxBounds, ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity,
    ConvexPolygon,
};
use crate::predicate::{
    classify_point_decision, classify_projective_point_decision, classify_real,
};
use crate::storage_hash::StorageHashMap;

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
    boundary: Vec<u32>,
    constraints: Vec<RawConstraint>,
    contacts: Vec<u32>,
    changed: bool,
}

struct ArrangementPointArena {
    points: Vec<Point3>,
    identities: Vec<ArrangementPointIdentity>,
    structural: StorageHashMap<ArrangementPointIdentity, u32>,
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
        let mut identities = Vec::new();
        identities
            .try_reserve_exact(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity arena",
            })?;
        let mut structural = StorageHashMap::default();
        structural
            .try_reserve(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity index",
            })?;
        Ok(Self {
            points,
            identities,
            structural,
            numeric: PointInterner::try_with_capacity(capacity, false, false)?,
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
        self.identities
            .try_reserve(1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity arena",
            })?;
        let old_len = self.points.len();
        let index = self
            .numeric
            .intern_owned(decisions, &mut self.points, point, None)?;
        let compact = u32::try_from(index).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement point arena",
        })?;
        if index == old_len {
            self.identities.push(identity.clone());
        } else {
            let canonical =
                self.identities
                    .get_mut(index)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "numeric point alias has no construction identity",
                    })?;
            if identity < *canonical {
                *canonical = identity.clone();
            }
        }
        self.structural.insert(identity, compact);
        Ok(compact)
    }
}

#[derive(Debug)]
struct SurfaceCorefinement {
    points: Vec<Point3>,
    identities: Vec<ArrangementPointIdentity>,
    face_offsets: Box<[u32]>,
    triangles: Vec<[u32; 3]>,
    constraint_offsets: Box<[u32]>,
    constraints: Vec<[u32; 2]>,
    contact_offsets: Box<[u32]>,
    contacts: Vec<u32>,
}

impl SurfaceCorefinement {
    fn face_triangles(&self, face: usize) -> &[[u32; 3]] {
        let start = self.face_offsets[face] as usize;
        let end = self.face_offsets[face + 1] as usize;
        &self.triangles[start..end]
    }

    fn face_constraints(&self, face: usize) -> &[[u32; 2]] {
        let start = self.constraint_offsets[face] as usize;
        let end = self.constraint_offsets[face + 1] as usize;
        &self.constraints[start..end]
    }

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
    transitions: Box<[i32]>,
    cell_windings: Box<[i32]>,
    operand_count: usize,
    cell_count: u32,
    component_count: u32,
    radial_edge_count: u32,
    max_radial_degree: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CellTruthNode {
    False,
    True,
    Operand(u32),
    Not(u32),
    And([u32; 2]),
    Or([u32; 2]),
    Xor([u32; 2]),
}

#[derive(Debug, Eq, PartialEq)]
struct ExpressionClassifications {
    expression_count: usize,
    facet_count: usize,
    classifications: Vec<i8>,
}

impl ExpressionClassifications {
    fn classification(&self, expression: usize, facet: usize) -> i8 {
        self.classifications[expression * self.facet_count + facet]
    }
}

impl SurfaceCellComplex {
    fn facet_contributions(&self, facet: usize) -> &[FacetContribution] {
        let start = self.contribution_offsets[facet] as usize;
        let end = self.contribution_offsets[facet + 1] as usize;
        &self.contributions[start..end]
    }

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
        let mut root_truth = vec![0_u8; truth_len];
        let mut node_truth = vec![0_u8; nodes.len()];
        for cell in 0..self.cell_count {
            let winding = self.cell_winding(cell);
            for (node, instruction) in nodes.iter().enumerate() {
                node_truth[node] = match *instruction {
                    CellTruthNode::False => 0,
                    CellTruthNode::True => 1,
                    CellTruthNode::Operand(operand) => u8::from(winding[operand as usize] != 0),
                    CellTruthNode::Not(input) => 1 - node_truth[input as usize],
                    CellTruthNode::And([left, right]) => {
                        node_truth[left as usize] & node_truth[right as usize]
                    }
                    CellTruthNode::Or([left, right]) => {
                        node_truth[left as usize] | node_truth[right as usize]
                    }
                    CellTruthNode::Xor([left, right]) => {
                        node_truth[left as usize] ^ node_truth[right as usize]
                    }
                };
            }
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
        Ok(ExpressionClassifications {
            expression_count: roots.len(),
            facet_count: self.facets.len(),
            classifications,
        })
    }
}

fn validate_cell_truth_program(
    nodes: &[CellTruthNode],
    roots: &[u32],
    operand_count: usize,
) -> HypermeshResult<()> {
    for (node, instruction) in nodes.iter().enumerate() {
        let dependency_is_valid = |dependency: u32| (dependency as usize) < node;
        let valid = match *instruction {
            CellTruthNode::False | CellTruthNode::True => true,
            CellTruthNode::Operand(operand) => (operand as usize) < operand_count,
            CellTruthNode::Not(input) => dependency_is_valid(input),
            CellTruthNode::And([left, right])
            | CellTruthNode::Or([left, right])
            | CellTruthNode::Xor([left, right]) => {
                dependency_is_valid(left) && dependency_is_valid(right)
            }
        };
        if !valid {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "surface cell truth program is not a valid topological DAG",
            });
        }
    }
    if roots.iter().any(|root| *root as usize >= nodes.len()) {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "surface cell truth program references an absent root",
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
) -> HypermeshResult<SurfaceCellComplex> {
    if surface.face_offsets.len() != polygons.len().saturating_add(1)
        || surface.identities.len() != surface.points.len()
    {
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
    let mut max_radial_degree = 0_u32;
    while edge_start < edge_uses.len() {
        let edge = edge_uses[edge_start].edge;
        let mut edge_end = edge_start + 1;
        while edge_end < edge_uses.len() && edge_uses[edge_end].edge == edge {
            edge_end += 1;
        }
        edges.push(edge);
        max_radial_degree =
            max_radial_degree.max(compact_len(edge_end - edge_start, "surface radial degree")?);
        assemble_radial_ring(
            decisions,
            &surface.points,
            &pending,
            edge,
            &edge_uses[edge_start..edge_end],
            &mut radial,
            &mut ray_starts,
            &mut sets,
        )?;
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
    let source_bvh = ExactBvh::build_decision(decisions, polygons)?;
    let maximum_x = maximum_surface_x(decisions, &surface.points)?;
    let (cell_windings, component_count) = classify_surface_cells(
        decisions,
        polygons,
        surface,
        &source_bvh,
        &maximum_x,
        &surface.points,
        &facets,
        &transitions,
        operand_count,
        cell_count,
        &edges,
    )?;

    Ok(SurfaceCellComplex {
        facets,
        contribution_offsets,
        contributions,
        transitions: transitions.into_boxed_slice(),
        cell_windings: cell_windings.into_boxed_slice(),
        operand_count,
        cell_count,
        component_count,
        radial_edge_count: compact_len(edges.len(), "surface radial edge count")?,
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
    let direction = point_by_id(points, edge[1])? - origin;
    let left = point_by_id(points, left)? - origin;
    let right = point_by_id(points, right)? - origin;
    classify_real(decisions, &direction.dot(&left.cross(&right)))
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
    let perpendicular_dot =
        direction.dot(&direction) * left.dot(&right) - direction.dot(&left) * direction.dot(&right);
    classify_real(decisions, &perpendicular_dot)
}

fn point_by_id(points: &[Point3], point: u32) -> HypermeshResult<&Point3> {
    points
        .get(point as usize)
        .ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "surface topology references an absent arrangement point",
        })
}

fn maximum_surface_x(decisions: &DecisionContext, points: &[Point3]) -> HypermeshResult<Real> {
    let mut maximum = points
        .first()
        .ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "surface cell complex has no arrangement points",
        })?
        .x
        .clone();
    for point in &points[1..] {
        if compare_real_decision(decisions, &point.x, &maximum)?.is_gt() {
            maximum = point.x.clone();
        }
    }
    Ok(maximum)
}

fn classify_surface_cells(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    surface: &SurfaceCorefinement,
    source_bvh: &ExactBvh,
    maximum_x: &Real,
    points: &[Point3],
    facets: &[SurfaceFacet],
    transitions: &[i32],
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
            maximum_x,
            points,
            facets,
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
    source_bvh: &ExactBvh,
    maximum_x: &Real,
    points: &[Point3],
    facets: &[SurfaceFacet],
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
    for candidate in 0..candidate_count {
        let parameter =
            Real::from(
                u64::try_from(candidate).map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "surface cell seed direction parameter",
                })?,
            );
        let direction = Vector3::new([
            Real::one(),
            parameter.clone(),
            parameter.clone() * parameter,
        ]);
        let local = decisions.isolated();
        match try_seed_surface_cell_winding(
            &local,
            polygons,
            surface,
            source_bvh,
            maximum_x,
            points,
            facets,
            operand_count,
            seed_facet,
            &point,
            &direction,
        ) {
            Ok(Some(result)) => {
                decisions.absorb(local.certainty());
                return Ok(result);
            }
            Ok(None) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
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
    surface: &SurfaceCorefinement,
    source_bvh: &ExactBvh,
    maximum_x: &Real,
    points: &[Point3],
    facets: &[SurfaceFacet],
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
    let distance = maximum_x.clone() - point.x.clone() + Real::one();
    let endpoint = Point3::new(
        point.x.clone() + distance.clone() * direction.0[0].clone(),
        point.y.clone() + distance.clone() * direction.0[1].clone(),
        point.z.clone() + distance * direction.0[2].clone(),
    );
    let bounds = ApproxBounds::new(point.clone(), endpoint);
    let mut candidate_faces = Vec::new();
    source_bvh.query_bounds_decision(decisions, &bounds, |face| {
        candidate_faces.push(face);
    })?;

    let mut winding = vec![0_i32; operand_count];
    let mut saw_origin = false;
    for face in candidate_faces {
        let polygon = polygons
            .get(face)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "surface seed broad phase returned an absent source face",
            })?;
        for &triangle in surface.face_triangles(face) {
            match ray_facet_relation(decisions, points, triangle, point, direction)? {
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
                    let mut canonical = triangle;
                    canonical.sort_unstable();
                    if canonical != seed.vertices {
                        return Err(HypermeshError::SurfaceArrangementFailed {
                            reason: "surface seed lies inside more than one geometric facet",
                        });
                    }
                    saw_origin = true;
                }
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
    let [a, b, c] = triangle
        .map(|vertex| point_by_id(points, vertex))
        .map(|vertex| vertex.cloned());
    let [a, b, c] = [a?, b?, c?];
    let edge_ab = &b - &a;
    let edge_ac = &c - &a;
    let cross = direction.cross(&edge_ac);
    let determinant = edge_ab.dot(&cross);
    let determinant_sign = classify_real(decisions, &determinant)?;
    if determinant_sign == Classification::On {
        return Ok(RayFacetRelation::Degenerate);
    }
    let from_a = origin - &a;
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
    let initial_points = polygons.iter().try_fold(0usize, |total, polygon| {
        total.checked_add(polygon.vertex_count())
    });
    let mut arena = ArrangementPointArena::with_capacity(initial_points.ok_or(
        HypermeshError::CapacityOverflow {
            operation: "surface arrangement source vertices",
        },
    )?)?;
    let mut work = Vec::new();
    work.try_reserve_exact(polygons.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face work",
        })?;
    work.resize_with(polygons.len(), FaceWork::default);

    for (face, polygon) in polygons.iter().enumerate() {
        add_source_boundary(decisions, face, polygon, &mut arena, &mut work[face])?;
    }
    append_intersection_constraints(decisions, polygons, intersections, &mut arena, &mut work)?;

    let offset_capacity =
        polygons
            .len()
            .checked_add(1)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "surface arrangement face offsets",
            })?;
    let mut face_offsets = Vec::new();
    let mut constraint_offsets = Vec::new();
    let mut contact_offsets = Vec::new();
    for offsets in [
        &mut face_offsets,
        &mut constraint_offsets,
        &mut contact_offsets,
    ] {
        offsets.try_reserve_exact(offset_capacity).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement face offsets",
            }
        })?;
        offsets.push(0_u32);
    }
    let mut triangles = Vec::new();
    let mut constraints = Vec::new();
    let mut contacts = Vec::new();
    for (face, (polygon, face_work)) in polygons.iter().zip(&work).enumerate() {
        let result = corefine_face(decisions, face, polygon, face_work, &mut arena)?;
        triangles.try_reserve(result.triangles.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement triangles",
            }
        })?;
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
        triangles.extend(result.triangles);
        constraints.extend(result.constraints);
        contacts.extend(result.contacts);
        face_offsets.push(compact_len(
            triangles.len(),
            "surface arrangement triangle offsets",
        )?);
        constraint_offsets.push(compact_len(
            constraints.len(),
            "surface arrangement constraint offsets",
        )?);
        contact_offsets.push(compact_len(
            contacts.len(),
            "surface arrangement contact offsets",
        )?);
    }
    Ok(SurfaceCorefinement {
        points: arena.points,
        identities: arena.identities,
        face_offsets: face_offsets.into_boxed_slice(),
        triangles,
        constraint_offsets: constraint_offsets.into_boxed_slice(),
        constraints,
        contact_offsets: contact_offsets.into_boxed_slice(),
        contacts,
    })
}

fn compact_len(len: usize, operation: &'static str) -> HypermeshResult<u32> {
    u32::try_from(len).map_err(|_| HypermeshError::CapacityOverflow { operation })
}

fn add_source_boundary(
    decisions: &DecisionContext,
    _face: usize,
    polygon: &ConvexPolygon,
    arena: &mut ArrangementPointArena,
    work: &mut FaceWork,
) -> HypermeshResult<()> {
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
    if vertices.len() < 3
        || vertices.len() != vertex_identities.len()
        || vertices.len() != edge_identities.len()
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "source face geometry and identity cycles are not aligned",
        });
    }
    work.boundary
        .try_reserve_exact(vertices.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face boundary",
        })?;
    work.constraints
        .try_reserve(vertices.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face constraints",
        })?;
    for (point, identity) in vertices.iter().zip(vertex_identities) {
        work.boundary.push(arena.insert(
            decisions,
            ArrangementPointIdentity::Construction(identity),
            point.clone(),
        )?);
    }
    for index in 0..work.boundary.len() {
        work.constraints.push(RawConstraint {
            endpoints: [
                work.boundary[index],
                work.boundary[(index + 1) % work.boundary.len()],
            ],
            line: edge_identities
                .get(index)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "source edge identity cycle is incomplete",
                })?,
        });
    }
    Ok(())
}

fn append_intersection_constraints(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    intersections: &PairwiseIntersectionGraph,
    arena: &mut ArrangementPointArena,
    work: &mut [FaceWork],
) -> HypermeshResult<()> {
    for face in 0..polygons.len() {
        for event in intersections.event_ids(face)? {
            match event {
                PairwiseIntersectionEventIds::NonCoplanarPoint {
                    point,
                    other_polygon: _,
                }
                | PairwiseIntersectionEventIds::CoplanarPoint {
                    point,
                    other_polygon: _,
                } => {
                    let point = insert_graph_point(decisions, intersections, arena, point)?;
                    work[face].contacts.push(point);
                    work[face].changed = true;
                }
                PairwiseIntersectionEventIds::NonCoplanarSegment {
                    endpoints,
                    other_polygon,
                } => {
                    let endpoints = endpoints
                        .map(|point| insert_graph_point(decisions, intersections, arena, point));
                    let endpoints = [endpoints[0].clone()?, endpoints[1].clone()?];
                    work[face].constraints.push(RawConstraint {
                        endpoints,
                        line: pairwise_split_line(face, other_polygon as usize)?,
                    });
                    work[face].changed = true;
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
                        insert_graph_point(decisions, intersections, arena, endpoints[0])?,
                        insert_graph_point(decisions, intersections, arena, endpoints[1])?,
                    ];
                    work[face]
                        .constraints
                        .push(RawConstraint { endpoints, line });
                    work[face].changed = true;
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
                        point_ids.push(arena.insert(
                            decisions,
                            vertex.identity.clone(),
                            vertex.point.clone(),
                        )?);
                    }
                    for index in 0..overlay.len() {
                        let constraint = RawConstraint {
                            endpoints: [point_ids[index], point_ids[(index + 1) % overlay.len()]],
                            line: overlay[index].outgoing.clone(),
                        };
                        work[face].constraints.push(constraint.clone());
                        work[other].constraints.push(constraint);
                    }
                    work[face].changed = true;
                    work[other].changed = true;
                }
            }
        }
    }
    Ok(())
}

fn insert_graph_point(
    decisions: &DecisionContext,
    graph: &PairwiseIntersectionGraph,
    arena: &mut ArrangementPointArena,
    point: u32,
) -> HypermeshResult<u32> {
    let (materialized, identity) = graph.construction_point(point)?;
    arena.insert(
        decisions,
        ArrangementPointIdentity::Construction(identity.clone()),
        materialized.clone(),
    )
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
    for edge in 0..polygon.edges.len() {
        let plane = &polygon.edges[(edge + 1) % polygon.edges.len()];
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
        if polygon.edges.len() != outgoing.len() || polygon.edges.len() < 3 {
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
        polygon.known_vertices = None;
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
        for index in 0..count {
            let next = (index + 1) % count;
            let segment_plane = self.polygon.edges[next].clone();
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
        self.polygon.edges = Arc::new(edges);
        self.polygon.known_vertices = None;
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
    if right.edges.len() != right_edges.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "coplanar overlap clip planes and identities are not aligned",
        });
    }
    let mut overlap = IdentifiedPolygon::from_source(left)?;
    for edge in 0..right.edges.len() {
        overlap = overlap
            .clip_negative(
                decisions,
                &right.edges[edge],
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
    constraints: Vec<[u32; 2]>,
    contacts: Vec<u32>,
}

fn corefine_face(
    decisions: &DecisionContext,
    face: usize,
    polygon: &ConvexPolygon,
    work: &FaceWork,
    arena: &mut ArrangementPointArena,
) -> HypermeshResult<FaceResult> {
    let mut constraint_lines = BTreeMap::<[u32; 2], ConstructionEdgeIdentity>::new();
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
    point_ids.extend(work.boundary.iter().copied());
    point_ids.extend(work.contacts.iter().copied());
    for edge in constraint_lines.keys() {
        point_ids.extend(edge);
    }
    let projection_axis = projection_axis(decisions, &polygon.support)?;
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
        if classify_point_decision(decisions, materialized, &polygon.support)? != Classification::On
            || polygon.edges.iter().try_fold(false, |outside, edge| {
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
            let intersection = decisions
                .decide(
                    hyperlimit::proper_segment_intersection_point(
                        &limit_point(left_points[0]),
                        &limit_point(left_points[1]),
                        &limit_point(right_points[0]),
                        &limit_point(right_points[1]),
                        decisions.policy(),
                    ),
                    "surface arrangement proper segment intersection",
                )?
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "proper face constraints have no intersection point",
                })?;
            let planar = hypertri::ExactPoint::new(intersection.x, intersection.y);
            let point = lift_planar_point(&planar, &polygon.support, projection_axis, axes)?;
            if classify_point_decision(decisions, &point, &polygon.support)? != Classification::On {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "lifted face crossing does not lie on its source support",
                });
            }
            let support = pairwise_support_identity(face)?.ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "source face has no operation-local support identity",
                },
            )?;
            let identity = intersect_arrangement_lines(&left.line, &right.line, support)?;
            let point_id = arena.insert(decisions, identity, point)?;
            projected
                .entry(point_id)
                .or_insert_with(|| project_point(&arena.points[point_id as usize], axes));
        }
    }

    let mut split_lines = BTreeMap::<[u32; 2], ConstructionEdgeIdentity>::new();
    for constraint in &authored {
        let mut on_segment = Vec::new();
        for (&point_id, point) in &projected {
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

    let boundary_edges = work
        .boundary
        .iter()
        .copied()
        .zip(work.boundary.iter().copied().cycle().skip(1))
        .take(work.boundary.len())
        .map(|edge| sorted_edge([edge.0, edge.1]))
        .collect::<BTreeSet<_>>();
    let only_source_boundary = projected.len() == work.boundary.len()
        && split_lines.keys().copied().collect::<BTreeSet<_>>() == boundary_edges;
    let mut contacts = work.contacts.clone();
    contacts.sort_unstable();
    contacts.dedup();
    if !work.changed || only_source_boundary {
        return Ok(FaceResult {
            triangles: triangulate_convex_boundary(&work.boundary),
            constraints: boundary_edges.into_iter().collect(),
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
    let outcome = hypertri::cdt::constrained_delaunay_convex_hull(&context, &points, &constraints)
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
    let source_positive = source_projection_is_positive(decisions, &projected, &work.boundary)?;
    let mut triangles = Vec::new();
    triangles
        .try_reserve_exact(outcome.value.triangles().len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face triangles",
        })?;
    for triangle in outcome.value.triangles() {
        let mut triangle = triangle.map(|vertex| point_ids[vertex]);
        let triangle_positive = match planar_orientation(
            decisions,
            &projected[&triangle[0]],
            &projected[&triangle[1]],
            &projected[&triangle[2]],
        )? {
            Classification::Positive => true,
            Classification::Negative => false,
            Classification::On => {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "bounded face triangulation produced a degenerate triangle",
                });
            }
        };
        if triangle_positive != source_positive {
            triangle.swap(1, 2);
        }
        triangles.push(triangle);
    }
    let constraints = outcome
        .value
        .constraint_edges()
        .iter()
        .map(|constraint| sorted_edge([point_ids[constraint.from], point_ids[constraint.to]]))
        .collect::<BTreeSet<_>>();
    let expected_constraints = split_lines.keys().copied().collect::<BTreeSet<_>>();
    let triangulation_edges = triangles
        .iter()
        .flat_map(|triangle| {
            [
                sorted_edge([triangle[0], triangle[1]]),
                sorted_edge([triangle[1], triangle[2]]),
                sorted_edge([triangle[2], triangle[0]]),
            ]
        })
        .collect::<BTreeSet<_>>();
    if constraints != expected_constraints
        || !constraints
            .iter()
            .all(|constraint| triangulation_edges.contains(constraint))
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "bounded face triangulation did not preserve every exact constraint",
        });
    }
    Ok(FaceResult {
        triangles,
        constraints: constraints.into_iter().collect(),
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
        hyperlimit::point_on_segment(
            &limit_point(edge[0]),
            &limit_point(edge[1]),
            &limit_point(point),
            decisions.policy(),
        ),
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
        hyperlimit::orient2(
            &limit_point(a),
            &limit_point(b),
            &limit_point(c),
            decisions.policy(),
        ),
        "surface arrangement orientation",
    )?;
    Ok(match sign {
        hyperlimit::Sign::Negative => Classification::Negative,
        hyperlimit::Sign::Zero => Classification::On,
        hyperlimit::Sign::Positive => Classification::Positive,
    })
}

fn limit_point(point: &hypertri::ExactPoint) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
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

fn exact_rational_points_contradict(left: &Point3, right: &Point3) -> bool {
    let left = [&left.x, &left.y, &left.z].map(Real::exact_rational_ref);
    let right = [&right.x, &right.y, &right.z].map(Real::exact_rational_ref);
    match (left, right) {
        ([Some(lx), Some(ly), Some(lz)], [Some(rx), Some(ry), Some(rz)]) => {
            lx != rx || ly != ry || lz != rz
        }
        _ => false,
    }
}

fn sorted_edge(mut edge: [u32; 2]) -> [u32; 2] {
    edge.sort_unstable();
    edge
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

    fn arranged_cells(
        polygons: &[ConvexPolygon],
        policy: hyperlimit::PredicatePolicy,
    ) -> (MeshCertainty, SurfaceCorefinement, SurfaceCellComplex) {
        let context = MeshContext::new(policy);
        let decisions = DecisionContext::new(&context);
        let graph = pairwise_intersections_by_polygon(&decisions, polygons).unwrap();
        let surface = corefine_surface(&decisions, polygons, &graph).unwrap();
        let cells = assemble_surface_cells(&decisions, polygons, &surface).unwrap();
        (decisions.certainty(), surface, cells)
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
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
            assert_eq!(cells.component_count, 3);
            assert_eq!(cells.cell_count, 6);
            let classified = cells.classify_expressions(&nodes, &roots).unwrap();
            assert_eq!(classified.expression_count, roots.len());
            assert_eq!(classified.facet_count, cells.facets.len());
            let selected = (0..classified.expression_count)
                .map(|expression| {
                    (0..classified.facet_count)
                        .filter(|&facet| classified.classification(expression, facet) != 0)
                        .count()
                })
                .collect::<Vec<_>>();
            assert_eq!(selected, [0, 0, 4, 4, 8, 12, 12]);
            for expression in 2..classified.expression_count {
                let classifications = (0..classified.facet_count)
                    .map(|facet| classified.classification(expression, facet))
                    .collect::<Vec<_>>();
                assert!(facet_classifications_are_closed(&cells, &classifications));
            }

            assert!(matches!(
                cells.classify_expressions(&[CellTruthNode::Operand(3)], &[0]),
                Err(HypermeshError::SurfaceArrangementFailed { .. })
            ));
            assert!(matches!(
                cells.classify_expressions(&[CellTruthNode::Not(0)], &[0]),
                Err(HypermeshError::SurfaceArrangementFailed { .. })
            ));
            assert!(matches!(
                cells.classify_expressions(&nodes, &[u32::MAX]),
                Err(HypermeshError::SurfaceArrangementFailed { .. })
            ));
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
            let (certainty, _, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
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
                &cells,
                &reverse_classifications
            ));
            assert!(selected_facets_are_closed(
                &cells,
                crate::winding::BooleanOp::Union
            ));
            assert!(selected_facets_are_closed(
                &cells,
                crate::winding::BooleanOp::Difference
            ));
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
            let (certainty, surface, cells) = arranged_cells(&polygons, policy);
            assert_eq!(certainty, MeshCertainty::Certified);
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
                &cells,
                crate::winding::BooleanOp::Union
            ));
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
            assert_eq!(
                assemble_surface_cells(&decisions, &polygons, &surface).unwrap_err(),
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

        polygons[1].delta_w.pop();
        assert_eq!(
            assemble_surface_cells(&decisions, &polygons, &surface).unwrap_err(),
            HypermeshError::WindingDimensionMismatch {
                expected: 2,
                actual: 1,
            }
        );
        polygons[1].delta_w.push(0);

        surface.triangles[0][0] = u32::MAX;
        assert_eq!(
            assemble_surface_cells(&decisions, &polygons, &surface).unwrap_err(),
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
        for polygon in &mut polygons[..4] {
            polygon.delta_w[0] = i32::MAX;
        }
        for polygon in &mut polygons[4..] {
            polygon.delta_w[0] = 1;
        }
        assert_eq!(
            assemble_surface_cells(&decisions, &polygons, &surface).unwrap_err(),
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
        assert!(matches!(
            &surface.identities[center],
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::PlaneTriple { .. })
        ));
        assert!(surface.face_triangles(0).len() >= 8);
        assert_constraints_are_edges(&surface, 0);
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
            let mut work = FaceWork::default();
            add_source_boundary(&decisions, 0, &polygon, &mut arena, &mut work).unwrap();
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
            work.changed = true;

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
            let mut work = FaceWork::default();
            add_source_boundary(&decisions, 0, &polygon, &mut arena, &mut work).unwrap();
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
            work.changed = true;

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
        let mut work = FaceWork::default();
        add_source_boundary(&decisions, 0, &polygon, &mut arena, &mut work).unwrap();
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
        work.changed = true;

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
}
