//! Exact shell signed-volume evidence.
//!
//! The report in this module is intentionally an algebraic certificate. It
//! computes six times signed volume from retained oriented face loops and keeps
//! topology or validation failures explicit instead of promoting a mesh or a
//! primitive-float accumulator to source BREP truth.

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperreal::Real;

use crate::report::BrepShellClosureReport;
use crate::topology::{BrepEdge, BrepEdgeOrientation, BrepShell, BrepVertexId};
use crate::validation::BrepFaceValidationReport;

/// Orientation inferred from exact signed shell volume.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepShellOrientation {
    /// Face loops enclose positive signed volume.
    Positive,
    /// Face loops enclose negative signed volume.
    Negative,
    /// Face loops enclose zero signed volume.
    Zero,
}

/// Explicit blocker for exact shell signed-volume evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepShellVolumeBlocker {
    /// Shell contains no faces.
    EmptyShell,
    /// Retained shell closure is not exact-ready.
    ShellClosureNotReady,
    /// At least one face validation report is not exact-ready.
    FaceValidationNotReady,
    /// A loop has fewer than three coedges.
    DegenerateLoop,
    /// A coedge references an edge id not present in the shell.
    MissingEdge,
    /// A referenced edge starts and ends at the same vertex.
    DegenerateEdge,
    /// A referenced edge points at a vertex id not present in the shell.
    MissingVertex,
    /// Consecutive oriented coedges do not form a closed vertex chain.
    BrokenLoopChain,
    /// The signed-volume sign could not be certified.
    UnknownVolumeSign,
    /// The shell has exactly zero signed volume.
    ZeroVolume,
}

/// Exact signed-volume report for a retained shell.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepShellVolumeReport {
    /// Number of retained faces scanned.
    pub face_count: usize,
    /// Number of loops scanned.
    pub loop_count: usize,
    /// Number of exact-ready face validation reports.
    pub ready_face_count: usize,
    /// Number of blocked face validation reports.
    pub blocked_face_count: usize,
    /// Six times the signed volume when exact replay succeeds.
    pub signed_six_volume: Option<Real>,
    /// Inferred orientation from the signed volume.
    pub orientation: Option<BrepShellOrientation>,
    /// Whether signed volume is exactly zero.
    pub zero_volume: bool,
    /// Whether signed volume is positive.
    pub positive_volume: bool,
    /// Whether signed volume is negative.
    pub negative_volume: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepShellVolumeBlocker>,
    /// Whether exact signed-volume and orientation evidence is available.
    pub exact_volume_ready: bool,
}

impl BrepShellVolumeReport {
    /// Derive exact signed-volume evidence from retained oriented face loops.
    ///
    /// The computation fans each loop into triangles and accumulates
    /// `det(a, b, c)`, yielding six times the oriented volume over
    /// `hyperreal::Real`. Open shells, broken loop chains, and undecidable signs
    /// remain explicit blockers.
    pub fn from_shell(shell: &BrepShell) -> Self {
        let closure = shell.closure_report();
        let face_reports = shell
            .faces
            .iter()
            .map(|face| shell.face_validation_report(face.id))
            .collect::<Vec<_>>();
        Self::from_shell_with_evidence(shell, &closure, &face_reports)
    }

    pub(crate) fn from_shell_with_evidence(
        shell: &BrepShell,
        closure: &BrepShellClosureReport,
        face_reports: &[BrepFaceValidationReport],
    ) -> Self {
        let ready_face_count = face_reports
            .iter()
            .filter(|face| face.exact_face_ready)
            .count();
        let blocked_face_count = face_reports.len().saturating_sub(ready_face_count);

        let mut blockers = Vec::new();
        if shell.faces.is_empty() {
            blockers.push(BrepShellVolumeBlocker::EmptyShell);
        }
        if !closure.exact_shell_ready {
            blockers.push(BrepShellVolumeBlocker::ShellClosureNotReady);
        }
        if blocked_face_count > 0 || shell.faces.is_empty() {
            blockers.push(BrepShellVolumeBlocker::FaceValidationNotReady);
        }

        let edge_by_id = shell
            .edges
            .iter()
            .map(|edge| (edge.id, *edge))
            .collect::<BTreeMap<_, _>>();
        let vertex_by_id = shell
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex))
            .collect::<BTreeMap<_, _>>();

        let mut signed_six_volume = Real::from(0);
        let mut loop_count = 0;
        for face in &shell.faces {
            for face_loop in face.loops() {
                loop_count += 1;
                if face_loop.coedges.len() < 3 {
                    blockers.push(BrepShellVolumeBlocker::DegenerateLoop);
                    continue;
                }

                let mut loop_vertices = Vec::with_capacity(face_loop.coedges.len());
                let mut loop_ends = Vec::with_capacity(face_loop.coedges.len());
                for coedge in &face_loop.coedges {
                    let Some(edge) = edge_by_id.get(&coedge.edge) else {
                        blockers.push(BrepShellVolumeBlocker::MissingEdge);
                        continue;
                    };
                    if edge.is_degenerate() {
                        blockers.push(BrepShellVolumeBlocker::DegenerateEdge);
                    }
                    let (start, end) = oriented_endpoints(*edge, coedge.orientation);
                    loop_vertices.push(start);
                    loop_ends.push(end);
                }

                for (end, next_start) in loop_ends
                    .iter()
                    .zip(loop_vertices.iter().cycle().skip(1))
                    .take(loop_vertices.len())
                {
                    if end != next_start {
                        blockers.push(BrepShellVolumeBlocker::BrokenLoopChain);
                    }
                }

                let Some(anchor) = loop_vertices.first() else {
                    continue;
                };
                let Some(anchor) = vertex_by_id.get(anchor) else {
                    blockers.push(BrepShellVolumeBlocker::MissingVertex);
                    continue;
                };
                for index in 1..loop_vertices.len().saturating_sub(1) {
                    let Some(second) = vertex_by_id.get(&loop_vertices[index]) else {
                        blockers.push(BrepShellVolumeBlocker::MissingVertex);
                        continue;
                    };
                    let Some(third) = vertex_by_id.get(&loop_vertices[index + 1]) else {
                        blockers.push(BrepShellVolumeBlocker::MissingVertex);
                        continue;
                    };
                    signed_six_volume = &signed_six_volume
                        + determinant3(&anchor.point, &second.point, &third.point);
                }
            }
        }

        let zero = Real::from(0);
        let volume_order = signed_six_volume.partial_cmp(&zero);
        let orientation = match volume_order {
            Some(Ordering::Greater) => Some(BrepShellOrientation::Positive),
            Some(Ordering::Less) => Some(BrepShellOrientation::Negative),
            Some(Ordering::Equal) => {
                blockers.push(BrepShellVolumeBlocker::ZeroVolume);
                Some(BrepShellOrientation::Zero)
            }
            None => {
                blockers.push(BrepShellVolumeBlocker::UnknownVolumeSign);
                None
            }
        };
        let exact_volume_ready = blockers.is_empty();

        Self {
            face_count: shell.faces.len(),
            loop_count,
            ready_face_count,
            blocked_face_count,
            signed_six_volume: exact_volume_ready.then_some(signed_six_volume),
            orientation: exact_volume_ready.then_some(orientation).flatten(),
            zero_volume: volume_order == Some(Ordering::Equal),
            positive_volume: volume_order == Some(Ordering::Greater),
            negative_volume: volume_order == Some(Ordering::Less),
            blockers,
            exact_volume_ready,
        }
    }
}

impl BrepShell {
    /// Derive exact signed-volume and orientation evidence for this shell.
    pub fn shell_volume_report(&self) -> BrepShellVolumeReport {
        BrepShellVolumeReport::from_shell(self)
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

fn determinant3(
    first: &hyperlimit::Point3,
    second: &hyperlimit::Point3,
    third: &hyperlimit::Point3,
) -> Real {
    let yz = &(&second.y * &third.z) - &(&second.z * &third.y);
    let zx = &(&second.z * &third.x) - &(&second.x * &third.z);
    let xy = &(&second.x * &third.y) - &(&second.y * &third.x);
    &(&first.x * &yz) + &(&first.y * &zx) + &(&first.z * &xy)
}
