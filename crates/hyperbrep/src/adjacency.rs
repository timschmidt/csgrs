//! Adjacent-face edge agreement reports.
//!
//! These reports package per-edge evidence for downstream boolean, tessellation,
//! and mesh handoff code. They do not sew topology or infer pcurves; they replay
//! retained edge uses and support-surface incidence as explicit facts.

use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{PlaneSide, PredicateOutcome};

use crate::surface::BrepSurfaceKind;
use crate::topology::{
    BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFaceId, BrepLoopId, BrepShell,
};

/// Explicit blocker for adjacent-face edge agreement.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepEdgeAgreementBlocker {
    /// A retained edge id was not present in the shell.
    MissingEdge,
    /// Edge references a vertex id not present in the shell.
    MissingVertex,
    /// Edge starts and ends at the same vertex.
    DegenerateEdge,
    /// Edge is used fewer than two times.
    BoundaryEdge,
    /// Edge is used more than two times.
    NonmanifoldEdge,
    /// A two-use edge is used twice with the same orientation.
    SameOrientationPair,
    /// An edge use references a face surface not present in the shell.
    MissingSurface,
    /// An edge use references an unsupported surface family.
    UnsupportedSurface,
    /// An edge-use support surface is not ready for exact replay.
    SurfaceNotReady,
    /// At least one edge endpoint is certified off an adjacent face surface.
    EndpointOffSurface,
    /// At least one endpoint/surface relation was undecidable.
    UnknownEndpointSurfaceRelation,
}

/// One oriented use of an edge by a face loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BrepEdgeUseReport {
    /// Face using the edge.
    pub face: BrepFaceId,
    /// Loop containing the edge use.
    pub loop_id: BrepLoopId,
    /// Orientation of the coedge in that loop.
    pub orientation: BrepEdgeOrientation,
    /// Whether both edge endpoints were certified on the face support surface.
    pub endpoints_on_surface: bool,
}

/// Agreement report for one retained edge across adjacent face uses.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepEdgeAgreementReport {
    /// Edge that was checked.
    pub edge: BrepEdgeId,
    /// Number of retained uses across face loops.
    pub use_count: usize,
    /// Number of forward uses.
    pub forward_use_count: usize,
    /// Number of reversed uses.
    pub reversed_use_count: usize,
    /// Face-loop uses of this edge.
    pub uses: Vec<BrepEdgeUseReport>,
    /// Explicit blockers.
    pub blockers: Vec<BrepEdgeAgreementBlocker>,
    /// Whether this edge has exactly two opposite oriented face uses.
    pub manifold_pair_ready: bool,
    /// Whether both adjacent face support surfaces contain the retained edge
    /// endpoints exactly.
    pub exact_edge_image_ready: bool,
    /// Whether topology pairing and endpoint/support-surface replay both agree.
    pub edge_agreement_ready: bool,
}

/// Shell-level adjacent-face agreement summary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepShellEdgeAgreementReport {
    /// Per-edge agreement reports.
    pub edges: Vec<BrepEdgeAgreementReport>,
    /// Number of retained edges checked.
    pub edge_count: usize,
    /// Number of edges whose adjacency agreement is ready.
    pub ready_edge_count: usize,
    /// Number of edges blocked from adjacency agreement.
    pub blocked_edge_count: usize,
    /// Number of boundary edges.
    pub boundary_edge_count: usize,
    /// Number of nonmanifold edges.
    pub nonmanifold_edge_count: usize,
    /// Number of two-use same-orientation edge pairs.
    pub same_orientation_pair_count: usize,
    /// Number of edges whose adjacent surface images are exact-ready.
    pub exact_edge_image_count: usize,
    /// Explicit blockers aggregated across all edges.
    pub blockers: Vec<BrepEdgeAgreementBlocker>,
    /// Whether every retained edge has ready adjacent-face agreement.
    pub shell_edge_agreement_ready: bool,
}

impl BrepShellEdgeAgreementReport {
    /// Validate adjacent face agreement for every retained edge.
    ///
    /// Adjacent topology is ready only when retained edge uses and
    /// support-surface predicates replay exactly. Full pcurve image equality
    /// remains future work.
    pub fn from_shell(shell: &BrepShell) -> Self {
        let edge_by_id = shell
            .edges
            .iter()
            .map(|edge| (edge.id, *edge))
            .collect::<BTreeMap<_, _>>();
        let mut uses_by_edge = BTreeMap::<BrepEdgeId, Vec<RawEdgeUse>>::new();
        for face in &shell.faces {
            for face_loop in face.loops() {
                for coedge in &face_loop.coedges {
                    uses_by_edge
                        .entry(coedge.edge)
                        .or_default()
                        .push(RawEdgeUse {
                            face: face.id,
                            loop_id: face_loop.id,
                            orientation: coedge.orientation,
                        });
                }
            }
        }

        let mut edge_ids = shell
            .edges
            .iter()
            .map(|edge| edge.id)
            .collect::<BTreeSet<_>>();
        edge_ids.extend(uses_by_edge.keys().copied());
        let edges = edge_ids
            .into_iter()
            .map(|edge_id| {
                BrepEdgeAgreementReport::from_parts(
                    shell,
                    edge_id,
                    edge_by_id.get(&edge_id).copied(),
                    uses_by_edge.remove(&edge_id).unwrap_or_default(),
                )
            })
            .collect::<Vec<_>>();

        let ready_edge_count = edges
            .iter()
            .filter(|edge| edge.edge_agreement_ready)
            .count();
        let blocked_edge_count = edges.len().saturating_sub(ready_edge_count);
        let boundary_edge_count = edges
            .iter()
            .filter(|edge| {
                edge.blockers
                    .contains(&BrepEdgeAgreementBlocker::BoundaryEdge)
            })
            .count();
        let nonmanifold_edge_count = edges
            .iter()
            .filter(|edge| {
                edge.blockers
                    .contains(&BrepEdgeAgreementBlocker::NonmanifoldEdge)
            })
            .count();
        let same_orientation_pair_count = edges
            .iter()
            .filter(|edge| {
                edge.blockers
                    .contains(&BrepEdgeAgreementBlocker::SameOrientationPair)
            })
            .count();
        let exact_edge_image_count = edges
            .iter()
            .filter(|edge| edge.exact_edge_image_ready)
            .count();
        let blockers = edges
            .iter()
            .flat_map(|edge| edge.blockers.iter().copied())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let shell_edge_agreement_ready = !edges.is_empty() && blocked_edge_count == 0;
        Self {
            edge_count: edges.len(),
            edges,
            ready_edge_count,
            blocked_edge_count,
            boundary_edge_count,
            nonmanifold_edge_count,
            same_orientation_pair_count,
            exact_edge_image_count,
            blockers,
            shell_edge_agreement_ready,
        }
    }
}

impl BrepEdgeAgreementReport {
    fn from_parts(
        shell: &BrepShell,
        edge_id: BrepEdgeId,
        edge: Option<BrepEdge>,
        raw_uses: Vec<RawEdgeUse>,
    ) -> Self {
        let mut blockers = BTreeSet::new();
        let Some(edge) = edge else {
            blockers.insert(BrepEdgeAgreementBlocker::MissingEdge);
            return Self::blocked(edge_id, raw_uses, blockers);
        };
        if edge.is_degenerate() {
            blockers.insert(BrepEdgeAgreementBlocker::DegenerateEdge);
        }
        let vertex_by_id = shell
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex))
            .collect::<BTreeMap<_, _>>();
        let endpoints = match (vertex_by_id.get(&edge.start), vertex_by_id.get(&edge.end)) {
            (Some(start), Some(end)) => Some((start, end)),
            _ => {
                blockers.insert(BrepEdgeAgreementBlocker::MissingVertex);
                None
            }
        };
        let face_by_id = shell
            .faces
            .iter()
            .map(|face| (face.id, face))
            .collect::<BTreeMap<_, _>>();
        let surface_by_id = shell
            .surfaces
            .iter()
            .map(|surface| (surface.id, surface))
            .collect::<BTreeMap<_, _>>();

        let mut uses = Vec::with_capacity(raw_uses.len());
        for raw_use in &raw_uses {
            let mut endpoints_on_surface = false;
            if let (Some((start, end)), Some(face)) = (endpoints, face_by_id.get(&raw_use.face)) {
                match surface_by_id.get(&face.surface) {
                    Some(surface) => {
                        if !surface.facts().exact_replay_ready {
                            blockers.insert(BrepEdgeAgreementBlocker::SurfaceNotReady);
                        }
                        match &surface.kind {
                            BrepSurfaceKind::Plane(plane) if surface.facts().exact_replay_ready => {
                                let prepared = plane.prepare();
                                let start_on =
                                    classify_endpoint(&prepared, &start.point, &mut blockers);
                                let end_on =
                                    classify_endpoint(&prepared, &end.point, &mut blockers);
                                endpoints_on_surface = start_on && end_on;
                            }
                            BrepSurfaceKind::Plane(_) => {}
                            BrepSurfaceKind::Unsupported { .. } => {
                                blockers.insert(BrepEdgeAgreementBlocker::UnsupportedSurface);
                            }
                        }
                    }
                    None => {
                        blockers.insert(BrepEdgeAgreementBlocker::MissingSurface);
                    }
                }
            }
            uses.push(BrepEdgeUseReport {
                face: raw_use.face,
                loop_id: raw_use.loop_id,
                orientation: raw_use.orientation,
                endpoints_on_surface,
            });
        }

        let forward_use_count = raw_uses
            .iter()
            .filter(|edge_use| edge_use.orientation == BrepEdgeOrientation::Forward)
            .count();
        let reversed_use_count = raw_uses.len().saturating_sub(forward_use_count);
        if raw_uses.len() < 2 {
            blockers.insert(BrepEdgeAgreementBlocker::BoundaryEdge);
        } else if raw_uses.len() > 2 {
            blockers.insert(BrepEdgeAgreementBlocker::NonmanifoldEdge);
        } else if forward_use_count != 1 || reversed_use_count != 1 {
            blockers.insert(BrepEdgeAgreementBlocker::SameOrientationPair);
        }
        let manifold_pair_ready =
            raw_uses.len() == 2 && forward_use_count == 1 && reversed_use_count == 1;
        let exact_edge_image_ready =
            !uses.is_empty() && uses.iter().all(|edge_use| edge_use.endpoints_on_surface);
        let edge_agreement_ready =
            manifold_pair_ready && exact_edge_image_ready && blockers.is_empty();
        Self {
            edge: edge_id,
            use_count: raw_uses.len(),
            forward_use_count,
            reversed_use_count,
            uses,
            blockers: blockers.into_iter().collect(),
            manifold_pair_ready,
            exact_edge_image_ready,
            edge_agreement_ready,
        }
    }

    fn blocked(
        edge: BrepEdgeId,
        raw_uses: Vec<RawEdgeUse>,
        blockers: BTreeSet<BrepEdgeAgreementBlocker>,
    ) -> Self {
        let forward_use_count = raw_uses
            .iter()
            .filter(|edge_use| edge_use.orientation == BrepEdgeOrientation::Forward)
            .count();
        let reversed_use_count = raw_uses.len().saturating_sub(forward_use_count);
        Self {
            edge,
            use_count: raw_uses.len(),
            forward_use_count,
            reversed_use_count,
            uses: raw_uses
                .into_iter()
                .map(|edge_use| BrepEdgeUseReport {
                    face: edge_use.face,
                    loop_id: edge_use.loop_id,
                    orientation: edge_use.orientation,
                    endpoints_on_surface: false,
                })
                .collect(),
            blockers: blockers.into_iter().collect(),
            manifold_pair_ready: false,
            exact_edge_image_ready: false,
            edge_agreement_ready: false,
        }
    }
}

impl BrepShell {
    /// Validate adjacent-face agreement for retained edge uses.
    pub fn edge_agreement_report(&self) -> BrepShellEdgeAgreementReport {
        BrepShellEdgeAgreementReport::from_shell(self)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RawEdgeUse {
    face: BrepFaceId,
    loop_id: BrepLoopId,
    orientation: BrepEdgeOrientation,
}

fn classify_endpoint(
    plane: &hyperlimit::PreparedPlane3<'_>,
    point: &hyperlimit::Point3,
    blockers: &mut BTreeSet<BrepEdgeAgreementBlocker>,
) -> bool {
    match plane.classify_point(point) {
        PredicateOutcome::Decided {
            value: PlaneSide::On,
            ..
        } => true,
        PredicateOutcome::Decided { .. } => {
            blockers.insert(BrepEdgeAgreementBlocker::EndpointOffSurface);
            false
        }
        PredicateOutcome::Unknown { .. } => {
            blockers.insert(BrepEdgeAgreementBlocker::UnknownEndpointSurfaceRelation);
            false
        }
    }
}
