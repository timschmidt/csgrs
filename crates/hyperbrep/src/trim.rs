//! Trim-loop evidence reports.
//!
//! This module starts the shared curve-on-surface layer with topology-only trim
//! evidence. It validates ordered coedge chains against retained BREP edges and
//! prepared surface facts, but deliberately does not claim UV containment,
//! curve-on-surface image equality, or self-intersection freedom. Those remain
//! separate predicate/report surfaces.

use std::collections::{BTreeMap, BTreeSet};

use crate::surface::BrepSurfaceId;
use crate::topology::{
    BrepEdge, BrepEdgeOrientation, BrepFaceId, BrepLoop, BrepLoopId, BrepShell, BrepVertexId,
};

/// Role of a trim loop on a face.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepTrimLoopRole {
    /// Boundary loop that encloses the primary face region.
    Outer,
    /// Boundary loop that removes a hole or island from the primary region.
    Inner,
}

/// Explicit blocker for topology-only trim-loop readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepTrimLoopBlocker {
    /// The loop has no coedges.
    EmptyLoop,
    /// A planar trim loop needs at least three edge uses before it can bound an
    /// area-bearing face region.
    TooFewCoedges,
    /// The face references a surface id not present in the shell.
    MissingSurface,
    /// The referenced surface exists but is unsupported, lossy, unknown, or has
    /// invalid exact-core facts.
    SurfaceNotReady,
    /// A coedge references an edge id not present in the shell.
    MissingEdge,
    /// A referenced edge starts and ends at the same vertex.
    DegenerateEdge,
    /// A referenced edge points at a vertex id not present in the shell.
    MissingVertex,
    /// Consecutive oriented coedges do not form a closed vertex chain.
    VertexChainBreak,
}

/// Topology-only report for one trim loop.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepTrimLoopReport {
    /// Face that owns the loop.
    pub face: BrepFaceId,
    /// Surface referenced by the owning face.
    pub surface: BrepSurfaceId,
    /// Loop identifier.
    pub trim_loop: BrepLoopId,
    /// Outer/inner role on the face.
    pub role: BrepTrimLoopRole,
    /// Number of ordered coedges in the loop.
    pub coedge_count: usize,
    /// Number of coedges whose edge id is absent from the shell.
    pub missing_edge_count: usize,
    /// Number of referenced degenerate edges.
    pub degenerate_edge_count: usize,
    /// Number of referenced edge uses whose endpoint ids are absent.
    pub missing_vertex_count: usize,
    /// Number of consecutive oriented-edge endpoint mismatches.
    pub vertex_chain_break_count: usize,
    /// Whether the ordered coedges form a closed topological vertex chain.
    pub closed_vertex_chain: bool,
    /// Whether the referenced surface is ready for exact replay.
    pub surface_replay_ready: bool,
    /// Explicit blockers discovered while auditing this loop.
    pub blockers: Vec<BrepTrimLoopBlocker>,
    /// Whether this loop is ready as topology-only trim evidence.
    pub trim_loop_ready: bool,
}

/// Face-level trim-set report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepFaceTrimSetReport {
    /// Face that was requested.
    pub face: BrepFaceId,
    /// Surface referenced by the face, when the face exists.
    pub surface: Option<BrepSurfaceId>,
    /// Whether the requested face exists in the shell.
    pub face_found: bool,
    /// Per-loop topology reports.
    pub loops: Vec<BrepTrimLoopReport>,
    /// Whether the outer loop is ready.
    pub outer_ready: bool,
    /// Number of ready inner loops.
    pub inner_ready_count: usize,
    /// Number of blocked loops.
    pub blocked_loop_count: usize,
    /// Whether all face trim loops are ready.
    pub trim_set_ready: bool,
}

impl BrepTrimLoopReport {
    /// Audit one loop as topology-only trim evidence.
    ///
    /// Ordered oriented edge uses must form a closed loop that can bound a
    /// face. This records certified object facts and explicit blockers instead
    /// of repairing gaps or treating an approximate chain as a valid trim.
    pub fn from_shell_loop(
        shell: &BrepShell,
        face: BrepFaceId,
        surface: BrepSurfaceId,
        role: BrepTrimLoopRole,
        trim_loop: &BrepLoop,
    ) -> Self {
        let edge_by_id = shell
            .edges
            .iter()
            .map(|edge| (edge.id, *edge))
            .collect::<BTreeMap<_, _>>();
        let vertex_ids = shell
            .vertices
            .iter()
            .map(|vertex| vertex.id)
            .collect::<BTreeSet<_>>();
        let surface_replay_ready = shell
            .surfaces
            .iter()
            .find(|candidate| candidate.id == surface)
            .map(|candidate| candidate.facts().exact_replay_ready)
            .unwrap_or(false);
        let surface_missing = !shell
            .surfaces
            .iter()
            .any(|candidate| candidate.id == surface);

        let mut missing_edge_count = 0;
        let mut degenerate_edge_count = 0;
        let mut missing_vertex_count = 0;
        let mut endpoints = Vec::with_capacity(trim_loop.coedges.len());

        for coedge in &trim_loop.coedges {
            let Some(edge) = edge_by_id.get(&coedge.edge) else {
                missing_edge_count += 1;
                endpoints.push(None);
                continue;
            };
            if edge.is_degenerate() {
                degenerate_edge_count += 1;
            }
            let missing_start = !vertex_ids.contains(&edge.start);
            let missing_end = !vertex_ids.contains(&edge.end);
            if missing_start || missing_end {
                missing_vertex_count += 1;
            }
            if missing_start || missing_end {
                endpoints.push(None);
            } else {
                endpoints.push(Some(oriented_endpoints(*edge, coedge.orientation)));
            }
        }

        let mut vertex_chain_break_count = 0;
        let mut closed_vertex_chain =
            !endpoints.is_empty() && endpoints.iter().all(Option::is_some);
        if closed_vertex_chain {
            for index in 0..endpoints.len() {
                let (_, current_end) = endpoints[index].expect("checked Some above");
                let (next_start, _) =
                    endpoints[(index + 1) % endpoints.len()].expect("checked Some above");
                if current_end != next_start {
                    vertex_chain_break_count += 1;
                }
            }
            closed_vertex_chain = vertex_chain_break_count == 0;
        }

        let mut blockers = Vec::new();
        if trim_loop.coedges.is_empty() {
            blockers.push(BrepTrimLoopBlocker::EmptyLoop);
        }
        if trim_loop.coedges.len() < 3 {
            blockers.push(BrepTrimLoopBlocker::TooFewCoedges);
        }
        if surface_missing {
            blockers.push(BrepTrimLoopBlocker::MissingSurface);
        } else if !surface_replay_ready {
            blockers.push(BrepTrimLoopBlocker::SurfaceNotReady);
        }
        if missing_edge_count > 0 {
            blockers.push(BrepTrimLoopBlocker::MissingEdge);
        }
        if degenerate_edge_count > 0 {
            blockers.push(BrepTrimLoopBlocker::DegenerateEdge);
        }
        if missing_vertex_count > 0 {
            blockers.push(BrepTrimLoopBlocker::MissingVertex);
        }
        if !closed_vertex_chain {
            blockers.push(BrepTrimLoopBlocker::VertexChainBreak);
        }

        let trim_loop_ready = blockers.is_empty();
        Self {
            face,
            surface,
            trim_loop: trim_loop.id,
            role,
            coedge_count: trim_loop.coedges.len(),
            missing_edge_count,
            degenerate_edge_count,
            missing_vertex_count,
            vertex_chain_break_count,
            closed_vertex_chain,
            surface_replay_ready,
            blockers,
            trim_loop_ready,
        }
    }
}

impl BrepFaceTrimSetReport {
    /// Audit all loops on one face.
    pub fn from_shell_face(shell: &BrepShell, face: BrepFaceId) -> Self {
        let Some(face_record) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            return Self {
                face,
                surface: None,
                face_found: false,
                loops: Vec::new(),
                outer_ready: false,
                inner_ready_count: 0,
                blocked_loop_count: 0,
                trim_set_ready: false,
            };
        };

        let mut loops = Vec::with_capacity(1 + face_record.inner.len());
        loops.push(BrepTrimLoopReport::from_shell_loop(
            shell,
            face_record.id,
            face_record.surface,
            BrepTrimLoopRole::Outer,
            &face_record.outer,
        ));
        loops.extend(face_record.inner.iter().map(|trim_loop| {
            BrepTrimLoopReport::from_shell_loop(
                shell,
                face_record.id,
                face_record.surface,
                BrepTrimLoopRole::Inner,
                trim_loop,
            )
        }));

        let outer_ready = loops
            .iter()
            .any(|report| report.role == BrepTrimLoopRole::Outer && report.trim_loop_ready);
        let inner_ready_count = loops
            .iter()
            .filter(|report| report.role == BrepTrimLoopRole::Inner && report.trim_loop_ready)
            .count();
        let blocked_loop_count = loops
            .iter()
            .filter(|report| !report.trim_loop_ready)
            .count();
        let trim_set_ready = outer_ready && blocked_loop_count == 0;

        Self {
            face,
            surface: Some(face_record.surface),
            face_found: true,
            loops,
            outer_ready,
            inner_ready_count,
            blocked_loop_count,
            trim_set_ready,
        }
    }
}

impl BrepShell {
    /// Audit trim-loop evidence for one face.
    pub fn trim_set_report(&self, face: BrepFaceId) -> BrepFaceTrimSetReport {
        BrepFaceTrimSetReport::from_shell_face(self, face)
    }
}

fn oriented_endpoints(
    edge: BrepEdge,
    orientation: BrepEdgeOrientation,
) -> (BrepVertexId, BrepVertexId) {
    match orientation {
        BrepEdgeOrientation::Forward => (edge.start, edge.end),
        BrepEdgeOrientation::Reversed => (edge.end, edge.start),
    }
}
