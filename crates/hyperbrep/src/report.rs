//! Validation reports for retained BREP evidence.
//!
//! Reports are the boundary between value carriers and topology decisions. They
//! make blockers explicit instead of silently repairing a shell.

use std::collections::{BTreeMap, BTreeSet};

use crate::surface::{BrepSurface, BrepSurfaceKind, BrepSurfaceSource};
use crate::topology::{BrepEdgeId, BrepEdgeOrientation, BrepShell, BrepVertexId};

/// Count summary for a retained BREP shell.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BrepTopologyCounts {
    /// Number of vertices.
    pub vertex_count: usize,
    /// Number of edges.
    pub edge_count: usize,
    /// Number of surfaces.
    pub surface_count: usize,
    /// Number of faces.
    pub face_count: usize,
    /// Number of loops across all faces.
    pub loop_count: usize,
    /// Number of oriented edge uses across all loops.
    pub coedge_count: usize,
}

/// Surface inventory facts for a BREP shell or surface list.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BrepSurfaceInventoryReport {
    /// Total retained surfaces.
    pub surface_count: usize,
    /// Planar surfaces.
    pub planar_count: usize,
    /// Unsupported/named adapter surfaces.
    pub unsupported_count: usize,
    /// Planar surfaces whose normal is structurally known to be zero.
    pub zero_normal_count: usize,
    /// Surfaces sourced through lossy adapters.
    pub lossy_source_count: usize,
    /// Surfaces with unknown provenance.
    pub unknown_source_count: usize,
    /// Whether every surface is a supported exact plane.
    pub all_exact_planar: bool,
}

impl BrepSurfaceInventoryReport {
    /// Build an inventory report from retained surfaces.
    pub fn from_surfaces(surfaces: &[BrepSurface]) -> Self {
        let mut report = Self {
            surface_count: surfaces.len(),
            all_exact_planar: !surfaces.is_empty(),
            ..Self::default()
        };
        for surface in surfaces {
            match &surface.kind {
                BrepSurfaceKind::Plane(plane) => {
                    report.planar_count += 1;
                    if plane.structural_facts().normal_known_zero() {
                        report.zero_normal_count += 1;
                    }
                }
                BrepSurfaceKind::Unsupported { .. } => report.unsupported_count += 1,
            }
            match surface.source {
                BrepSurfaceSource::LossyImport => report.lossy_source_count += 1,
                BrepSurfaceSource::Unknown => report.unknown_source_count += 1,
                BrepSurfaceSource::ExactConstruction | BrepSurfaceSource::ExactImport => {}
            }
            report.all_exact_planar &= surface.is_supported_exact_plane();
        }
        report
    }
}

/// Explicit blocker for exact shell readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepShellBlocker {
    /// Shell contains no faces.
    EmptyShell,
    /// A vertex id appears more than once.
    DuplicateVertexId,
    /// An edge id appears more than once.
    DuplicateEdgeId,
    /// A surface id appears more than once.
    DuplicateSurfaceId,
    /// A face id appears more than once.
    DuplicateFaceId,
    /// A loop id appears more than once.
    DuplicateLoopId,
    /// An edge references a missing vertex.
    MissingEdgeVertex,
    /// An edge references the same vertex twice.
    DegenerateEdge,
    /// A face references a missing surface.
    MissingFaceSurface,
    /// A loop references a missing edge.
    MissingLoopEdge,
    /// A face has an empty outer or inner loop.
    EmptyLoop,
    /// At least one retained plane has a zero normal.
    ZeroNormalSurface,
    /// At least one surface is unsupported.
    UnsupportedSurface,
    /// At least one surface came from a lossy adapter.
    LossySurfaceSource,
    /// At least one surface has unknown provenance.
    UnknownSurfaceSource,
    /// At least one edge is used fewer than two times.
    BoundaryEdges,
    /// At least one edge is used more than two times.
    NonmanifoldEdges,
    /// A two-use edge is used twice with the same orientation.
    SameOrientationEdgePair,
}

/// Explicit blocker for topology graph validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepTopologyValidationBlocker {
    /// Shell contains no faces.
    EmptyShell,
    /// A vertex id appears more than once.
    DuplicateVertexId,
    /// An edge id appears more than once.
    DuplicateEdgeId,
    /// A surface id appears more than once.
    DuplicateSurfaceId,
    /// A face id appears more than once.
    DuplicateFaceId,
    /// A loop id appears more than once.
    DuplicateLoopId,
    /// An edge references a missing vertex.
    MissingEdgeVertex,
    /// An edge references the same vertex twice.
    DegenerateEdge,
    /// A face references a missing surface.
    MissingFaceSurface,
    /// A loop references a missing edge.
    MissingLoopEdge,
    /// A loop has no edge uses.
    EmptyLoop,
    /// At least one edge is used fewer than two times.
    BoundaryEdges,
    /// At least one edge is used more than two times.
    NonmanifoldEdges,
    /// A two-use edge is used twice with the same orientation.
    SameOrientationEdgePair,
    /// At least one vertex is not attached to any retained edge.
    IsolatedVertices,
}

/// Topology graph validation and compact summaries.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepTopologyValidationReport {
    /// Topology counts.
    pub counts: BrepTopologyCounts,
    /// Euler characteristic `V - E + F` over retained topology counts.
    pub euler_characteristic: isize,
    /// Number of connected components in the valid vertex-edge graph.
    pub connected_component_count: usize,
    /// Number of connected components made by boundary edges.
    pub boundary_component_count: usize,
    /// Number of vertices not referenced by any retained edge.
    pub isolated_vertex_count: usize,
    /// Number of edges used fewer than two times.
    pub boundary_edge_count: usize,
    /// Number of edges used more than two times.
    pub nonmanifold_edge_count: usize,
    /// Number of two-use edges whose uses have the same orientation.
    pub same_orientation_pair_count: usize,
    /// Explicit blockers.
    pub blockers: Vec<BrepTopologyValidationBlocker>,
    /// Whether references, ids, incidence, and basic graph summaries are ready.
    pub topology_ready: bool,
}

/// Shell closure and exact-readiness audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepShellClosureReport {
    /// Topology counts.
    pub counts: BrepTopologyCounts,
    /// Surface inventory facts.
    pub surface_inventory: BrepSurfaceInventoryReport,
    /// Number of edges used fewer than two times.
    pub boundary_edge_count: usize,
    /// Number of edges used more than two times.
    pub nonmanifold_edge_count: usize,
    /// Number of two-use edges whose uses have the same orientation.
    pub same_orientation_pair_count: usize,
    /// Explicit blockers.
    pub blockers: Vec<BrepShellBlocker>,
    /// Whether every edge has exactly two opposite oriented uses.
    pub closed: bool,
    /// Whether this shell is ready to be consumed as exact BREP topology.
    pub exact_shell_ready: bool,
}

impl BrepTopologyValidationReport {
    /// Validate retained topology graph facts without repairing them.
    ///
    /// This report provides the reusable graph-level substrate described in
    /// classical BREP validity models such as Mäntylä, *An Introduction to
    /// Solid Modeling* (1988): vertices, edges, coedges, loops, and faces must
    /// have consistent identity and incidence before geometric validation or
    /// tessellation is meaningful. Following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), missing incidence
    /// and nonmanifold cases are explicit blockers rather than opportunities
    /// for tolerance-based sewing or identity inference.
    pub fn from_shell(shell: &BrepShell) -> Self {
        let counts = count_topology(shell);
        let mut blockers = BTreeSet::new();
        if shell.faces.is_empty() {
            blockers.insert(BrepTopologyValidationBlocker::EmptyShell);
        }
        insert_duplicate_topology_blocker(
            shell.vertices.iter().map(|vertex| vertex.id.0),
            BrepTopologyValidationBlocker::DuplicateVertexId,
            &mut blockers,
        );
        insert_duplicate_topology_blocker(
            shell.edges.iter().map(|edge| edge.id.0),
            BrepTopologyValidationBlocker::DuplicateEdgeId,
            &mut blockers,
        );
        insert_duplicate_topology_blocker(
            shell.surfaces.iter().map(|surface| surface.id.0),
            BrepTopologyValidationBlocker::DuplicateSurfaceId,
            &mut blockers,
        );
        insert_duplicate_topology_blocker(
            shell.faces.iter().map(|face| face.id.0),
            BrepTopologyValidationBlocker::DuplicateFaceId,
            &mut blockers,
        );

        let vertex_ids = shell
            .vertices
            .iter()
            .map(|vertex| vertex.id)
            .collect::<BTreeSet<_>>();
        let edge_ids = shell
            .edges
            .iter()
            .map(|edge| edge.id)
            .collect::<BTreeSet<_>>();
        let surface_ids = shell
            .surfaces
            .iter()
            .map(|surface| surface.id)
            .collect::<BTreeSet<_>>();

        let mut attached_vertices = BTreeSet::new();
        for edge in &shell.edges {
            if !vertex_ids.contains(&edge.start) || !vertex_ids.contains(&edge.end) {
                blockers.insert(BrepTopologyValidationBlocker::MissingEdgeVertex);
            } else {
                attached_vertices.insert(edge.start);
                attached_vertices.insert(edge.end);
            }
            if edge.is_degenerate() {
                blockers.insert(BrepTopologyValidationBlocker::DegenerateEdge);
            }
        }
        let isolated_vertex_count = vertex_ids.difference(&attached_vertices).count();
        if isolated_vertex_count > 0 {
            blockers.insert(BrepTopologyValidationBlocker::IsolatedVertices);
        }

        let mut edge_uses: BTreeMap<BrepEdgeId, (usize, usize)> = BTreeMap::new();
        let mut loop_ids = BTreeSet::new();
        let mut duplicated_loop_id = false;
        for face in &shell.faces {
            if !surface_ids.contains(&face.surface) {
                blockers.insert(BrepTopologyValidationBlocker::MissingFaceSurface);
            }
            for face_loop in face.loops() {
                duplicated_loop_id |= !loop_ids.insert(face_loop.id);
                if face_loop.is_empty() {
                    blockers.insert(BrepTopologyValidationBlocker::EmptyLoop);
                }
                for coedge in &face_loop.coedges {
                    if !edge_ids.contains(&coedge.edge) {
                        blockers.insert(BrepTopologyValidationBlocker::MissingLoopEdge);
                    }
                    let entry = edge_uses.entry(coedge.edge).or_default();
                    match coedge.orientation {
                        BrepEdgeOrientation::Forward => entry.0 += 1,
                        BrepEdgeOrientation::Reversed => entry.1 += 1,
                    }
                }
            }
        }
        if duplicated_loop_id {
            blockers.insert(BrepTopologyValidationBlocker::DuplicateLoopId);
        }

        let mut boundary_edge_count = 0_usize;
        let mut nonmanifold_edge_count = 0_usize;
        let mut same_orientation_pair_count = 0_usize;
        let mut boundary_edges = Vec::new();
        for edge in &shell.edges {
            let (forward, reversed) = edge_uses.get(&edge.id).copied().unwrap_or_default();
            let total = forward + reversed;
            if total < 2 {
                boundary_edge_count += 1;
                boundary_edges.push((edge.start, edge.end));
            } else if total > 2 {
                nonmanifold_edge_count += 1;
            } else if forward != 1 || reversed != 1 {
                same_orientation_pair_count += 1;
            }
        }
        if boundary_edge_count > 0 {
            blockers.insert(BrepTopologyValidationBlocker::BoundaryEdges);
        }
        if nonmanifold_edge_count > 0 {
            blockers.insert(BrepTopologyValidationBlocker::NonmanifoldEdges);
        }
        if same_orientation_pair_count > 0 {
            blockers.insert(BrepTopologyValidationBlocker::SameOrientationEdgePair);
        }

        let connected_component_count = component_count(
            vertex_ids.iter().copied(),
            shell.edges.iter().filter_map(|edge| {
                (vertex_ids.contains(&edge.start) && vertex_ids.contains(&edge.end))
                    .then_some((edge.start, edge.end))
            }),
        );
        let boundary_component_count = component_count(
            boundary_edges.iter().flat_map(|(a, b)| [*a, *b]),
            boundary_edges.iter().copied(),
        );
        let blockers = blockers.into_iter().collect::<Vec<_>>();
        let topology_ready = blockers.is_empty();
        Self {
            counts,
            euler_characteristic: counts.vertex_count as isize - counts.edge_count as isize
                + counts.face_count as isize,
            connected_component_count,
            boundary_component_count,
            isolated_vertex_count,
            boundary_edge_count,
            nonmanifold_edge_count,
            same_orientation_pair_count,
            blockers,
            topology_ready,
        }
    }
}

impl BrepShellClosureReport {
    /// Build a closure report from a shell without mutating or repairing it.
    pub fn from_shell(shell: &BrepShell) -> Self {
        let counts = count_topology(shell);
        let surface_inventory = BrepSurfaceInventoryReport::from_surfaces(&shell.surfaces);
        let mut blockers = BTreeSet::new();
        if shell.faces.is_empty() {
            blockers.insert(BrepShellBlocker::EmptyShell);
        }
        insert_duplicate_id_blocker(
            shell.vertices.iter().map(|vertex| vertex.id.0),
            BrepShellBlocker::DuplicateVertexId,
            &mut blockers,
        );
        insert_duplicate_id_blocker(
            shell.edges.iter().map(|edge| edge.id.0),
            BrepShellBlocker::DuplicateEdgeId,
            &mut blockers,
        );
        insert_duplicate_id_blocker(
            shell.surfaces.iter().map(|surface| surface.id.0),
            BrepShellBlocker::DuplicateSurfaceId,
            &mut blockers,
        );
        insert_duplicate_id_blocker(
            shell.faces.iter().map(|face| face.id.0),
            BrepShellBlocker::DuplicateFaceId,
            &mut blockers,
        );

        let vertex_ids = shell
            .vertices
            .iter()
            .map(|vertex| vertex.id)
            .collect::<BTreeSet<_>>();
        let edge_ids = shell
            .edges
            .iter()
            .map(|edge| edge.id)
            .collect::<BTreeSet<_>>();
        let surface_ids = shell
            .surfaces
            .iter()
            .map(|surface| surface.id)
            .collect::<BTreeSet<_>>();

        for edge in &shell.edges {
            if !vertex_ids.contains(&edge.start) || !vertex_ids.contains(&edge.end) {
                blockers.insert(BrepShellBlocker::MissingEdgeVertex);
            }
            if edge.is_degenerate() {
                blockers.insert(BrepShellBlocker::DegenerateEdge);
            }
        }

        let mut edge_uses: BTreeMap<BrepEdgeId, (usize, usize)> = BTreeMap::new();
        let mut loop_ids = BTreeSet::new();
        let mut duplicated_loop_id = false;
        for face in &shell.faces {
            if !surface_ids.contains(&face.surface) {
                blockers.insert(BrepShellBlocker::MissingFaceSurface);
            }
            for face_loop in face.loops() {
                duplicated_loop_id |= !loop_ids.insert(face_loop.id);
                if face_loop.is_empty() {
                    blockers.insert(BrepShellBlocker::EmptyLoop);
                }
                for coedge in &face_loop.coedges {
                    if !edge_ids.contains(&coedge.edge) {
                        blockers.insert(BrepShellBlocker::MissingLoopEdge);
                    }
                    let entry = edge_uses.entry(coedge.edge).or_default();
                    match coedge.orientation {
                        BrepEdgeOrientation::Forward => entry.0 += 1,
                        BrepEdgeOrientation::Reversed => entry.1 += 1,
                    }
                }
            }
        }
        if duplicated_loop_id {
            blockers.insert(BrepShellBlocker::DuplicateLoopId);
        }

        if surface_inventory.zero_normal_count > 0 {
            blockers.insert(BrepShellBlocker::ZeroNormalSurface);
        }
        if surface_inventory.unsupported_count > 0 {
            blockers.insert(BrepShellBlocker::UnsupportedSurface);
        }
        if surface_inventory.lossy_source_count > 0 {
            blockers.insert(BrepShellBlocker::LossySurfaceSource);
        }
        if surface_inventory.unknown_source_count > 0 {
            blockers.insert(BrepShellBlocker::UnknownSurfaceSource);
        }

        let mut boundary_edge_count = 0_usize;
        let mut nonmanifold_edge_count = 0_usize;
        let mut same_orientation_pair_count = 0_usize;
        for edge in &shell.edges {
            let (forward, reversed) = edge_uses.get(&edge.id).copied().unwrap_or_default();
            let total = forward + reversed;
            if total < 2 {
                boundary_edge_count += 1;
            } else if total > 2 {
                nonmanifold_edge_count += 1;
            } else if forward != 1 || reversed != 1 {
                same_orientation_pair_count += 1;
            }
        }
        if boundary_edge_count > 0 {
            blockers.insert(BrepShellBlocker::BoundaryEdges);
        }
        if nonmanifold_edge_count > 0 {
            blockers.insert(BrepShellBlocker::NonmanifoldEdges);
        }
        if same_orientation_pair_count > 0 {
            blockers.insert(BrepShellBlocker::SameOrientationEdgePair);
        }

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        let closed = !shell.faces.is_empty()
            && boundary_edge_count == 0
            && nonmanifold_edge_count == 0
            && same_orientation_pair_count == 0;
        let exact_shell_ready = closed
            && blockers.is_empty()
            && surface_inventory.all_exact_planar
            && counts.coedge_count > 0;

        Self {
            counts,
            surface_inventory,
            boundary_edge_count,
            nonmanifold_edge_count,
            same_orientation_pair_count,
            blockers,
            closed,
            exact_shell_ready,
        }
    }
}

fn count_topology(shell: &BrepShell) -> BrepTopologyCounts {
    let mut loop_count = 0_usize;
    let mut coedge_count = 0_usize;
    for face in &shell.faces {
        for face_loop in face.loops() {
            loop_count += 1;
            coedge_count += face_loop.coedges.len();
        }
    }
    BrepTopologyCounts {
        vertex_count: shell.vertices.len(),
        edge_count: shell.edges.len(),
        surface_count: shell.surfaces.len(),
        face_count: shell.faces.len(),
        loop_count,
        coedge_count,
    }
}

fn insert_duplicate_id_blocker<I>(
    ids: I,
    blocker: BrepShellBlocker,
    blockers: &mut BTreeSet<BrepShellBlocker>,
) where
    I: IntoIterator<Item = u64>,
{
    let mut seen = BTreeSet::new();
    for id in ids {
        if !seen.insert(id) {
            blockers.insert(blocker);
            return;
        }
    }
}

fn insert_duplicate_topology_blocker<I>(
    ids: I,
    blocker: BrepTopologyValidationBlocker,
    blockers: &mut BTreeSet<BrepTopologyValidationBlocker>,
) where
    I: IntoIterator<Item = u64>,
{
    let mut seen = BTreeSet::new();
    for id in ids {
        if !seen.insert(id) {
            blockers.insert(blocker);
            return;
        }
    }
}

fn component_count<V, E>(vertices: V, edges: E) -> usize
where
    V: IntoIterator<Item = BrepVertexId>,
    E: IntoIterator<Item = (BrepVertexId, BrepVertexId)>,
{
    let vertices = vertices.into_iter().collect::<BTreeSet<_>>();
    if vertices.is_empty() {
        return 0;
    }
    let mut adjacency = vertices
        .iter()
        .map(|vertex| (*vertex, BTreeSet::new()))
        .collect::<BTreeMap<_, _>>();
    for (start, end) in edges {
        if start == end || !vertices.contains(&start) || !vertices.contains(&end) {
            continue;
        }
        adjacency.entry(start).or_default().insert(end);
        adjacency.entry(end).or_default().insert(start);
    }

    let mut visited = BTreeSet::new();
    let mut count = 0;
    for vertex in &vertices {
        if visited.contains(vertex) {
            continue;
        }
        count += 1;
        let mut stack = vec![*vertex];
        while let Some(current) = stack.pop() {
            if !visited.insert(current) {
                continue;
            }
            if let Some(neighbors) = adjacency.get(&current) {
                stack.extend(
                    neighbors
                        .iter()
                        .filter(|next| !visited.contains(next))
                        .copied(),
                );
            }
        }
    }
    count
}
