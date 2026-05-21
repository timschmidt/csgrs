//! Face validation reports.
//!
//! Validation reports package retained object facts for downstream crates. They
//! do not repair topology, run private BREP booleans, or infer missing geometry.

use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{PlaneSide, PredicateOutcome};

use crate::bounds::{BrepFaceBoundsReport, BrepShellBoundsReport};
use crate::report::{BrepShellClosureReport, BrepTopologyValidationReport};
use crate::surface::{BrepSurfaceBlocker, BrepSurfaceFacts, BrepSurfaceKind};
use crate::tessellation::{BrepFaceTessellationManifest, BrepFaceTessellationReport};
use crate::topology::{BrepFaceId, BrepShell};
use crate::trim::BrepFaceTrimSetReport;

/// Explicit blocker for face-level validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFaceValidationBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face surface was missing, unsupported, lossy, unknown, or invalid.
    SurfaceNotReady,
    /// Face trim loops were not ready topology evidence.
    TrimSetNotReady,
    /// Exact face bounds could not be derived.
    BoundsNotReady,
    /// Face geometry consistency could not be certified.
    GeometryNotReady,
    /// Optional tessellation evidence was supplied but was not ready.
    TessellationNotReady,
}

/// Explicit blocker for shell validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepShellValidationBlocker {
    /// Retained topology graph facts are not ready.
    TopologyNotReady,
    /// Shell closure is not ready.
    ShellClosureNotReady,
    /// Exact shell bounds could not be derived.
    ShellBoundsNotReady,
    /// At least one face boundary is not ready.
    FaceBoundaryNotReady,
    /// At least one face validation report is not exact-ready.
    FaceValidationNotReady,
    /// Shell contains no faces.
    EmptyShell,
}

/// Explicit blocker for face geometry validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepGeometryValidationBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a surface id not present in the shell.
    MissingSurface,
    /// Face surface is not a supported exact plane.
    UnsupportedSurface,
    /// Retained surface facts are not ready for exact replay.
    SurfaceNotReady,
    /// A coedge references an edge id not present in the shell.
    MissingEdge,
    /// A referenced edge starts and ends at the same vertex.
    DegenerateEdge,
    /// A referenced edge points at a vertex id not present in the shell.
    MissingVertex,
    /// At least one boundary vertex is certified off the retained support surface.
    BoundaryVertexOffSurface,
    /// `hyperlimit` could not decide a boundary vertex/support-plane relation.
    UnknownVertexSurfaceRelation,
    /// Trim topology is not ready, so geometry consistency is incomplete.
    TrimSetNotReady,
}

/// Face geometry consistency report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepGeometryValidationReport {
    /// Face that was requested.
    pub face: BrepFaceId,
    /// Whether the face exists in the shell.
    pub face_found: bool,
    /// Number of unique boundary vertices referenced by face loops.
    pub boundary_vertex_count: usize,
    /// Number of missing edge references.
    pub missing_edge_count: usize,
    /// Number of degenerate edge references.
    pub degenerate_edge_count: usize,
    /// Number of missing vertex references.
    pub missing_vertex_count: usize,
    /// Number of boundary vertices certified on the retained support surface.
    pub on_surface_vertex_count: usize,
    /// Number of boundary vertices certified off the retained support surface.
    pub off_surface_vertex_count: usize,
    /// Number of boundary vertex/support-surface relations that were unknown.
    pub unknown_vertex_surface_count: usize,
    /// Whether trim topology was ready while validating geometry.
    pub trim_set_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepGeometryValidationBlocker>,
    /// Whether face geometry consistency is exact-ready.
    pub geometry_ready: bool,
}

/// Shared face validation report.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepFaceValidationReport {
    /// Face that was requested.
    pub face: BrepFaceId,
    /// Whether the face exists in the shell.
    pub face_found: bool,
    /// Prepared exact-core surface facts, when the face surface exists.
    pub surface_facts: Option<BrepSurfaceFacts>,
    /// Surface-preparation blockers, when the face surface exists but is not ready.
    pub surface_blockers: Vec<BrepSurfaceBlocker>,
    /// Trim-set report for the face, when the face exists.
    pub trim_set: Option<BrepFaceTrimSetReport>,
    /// Exact AABB/support report for the face, when the face exists.
    pub bounds: Option<BrepFaceBoundsReport>,
    /// Geometry consistency report for the face, when the face exists.
    pub geometry: Option<BrepGeometryValidationReport>,
    /// Optional tessellation report replayed against the same source face.
    pub tessellation: Option<BrepFaceTessellationReport>,
    /// Explicit validation blockers.
    pub blockers: Vec<BrepFaceValidationBlocker>,
    /// Whether retained surface and trim evidence are ready.
    pub exact_face_boundary_ready: bool,
    /// Whether exact face bounds are ready.
    pub exact_bounds_ready: bool,
    /// Whether the optional tessellation evidence is ready, or no tessellation
    /// evidence was requested.
    pub tessellation_ready: bool,
    /// Whether the face is ready for downstream exact/certified consumers.
    pub exact_face_ready: bool,
}

/// Shell validation report that aggregates topology, closure, bounds, and face reports.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepShellValidationReport {
    /// Topology graph validation report.
    pub topology: BrepTopologyValidationReport,
    /// Shell closure report.
    pub closure: BrepShellClosureReport,
    /// Exact shell AABB/support facts.
    pub bounds: BrepShellBoundsReport,
    /// Per-face validation reports.
    pub faces: Vec<BrepFaceValidationReport>,
    /// Number of faces whose retained boundary evidence is exact-ready.
    pub ready_face_boundary_count: usize,
    /// Number of faces whose retained boundary evidence is blocked.
    pub blocked_face_boundary_count: usize,
    /// Number of faces whose full validation report is exact-ready.
    pub ready_face_count: usize,
    /// Number of faces whose full validation report is blocked.
    pub blocked_face_count: usize,
    /// Whether all retained face boundary evidence is exact-ready.
    pub exact_surface_boundary_ready: bool,
    /// Whether topology, closure, bounds, and all faces are exact-ready as a
    /// closed shell.
    pub exact_closed_shell_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepShellValidationBlocker>,
}

impl BrepShellValidationReport {
    /// Validate retained shell evidence without promoting it to a solid.
    ///
    /// This report is the shared shell-level validation envelope for downstream
    /// crates that need exact BREP evidence but do not necessarily need a solid
    /// volume. It follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): topology, closure, bounds, and
    /// face geometry are replayed as explicit evidence, while open boundaries
    /// or invalid faces remain blockers instead of being repaired by sewing or
    /// tolerance merging. Classical BREP graph roles follow Mäntylä, *An
    /// Introduction to Solid Modeling* (1988).
    pub fn from_shell(shell: &BrepShell) -> Self {
        let topology = shell.validate_topology();
        let closure = shell.audit_closure();
        let bounds = shell.shell_bounds_report();
        let faces = shell
            .faces
            .iter()
            .map(|face| shell.face_validation_report(face.id, None))
            .collect::<Vec<_>>();
        let ready_face_boundary_count = faces
            .iter()
            .filter(|face| face.exact_face_boundary_ready)
            .count();
        let blocked_face_boundary_count = faces.len().saturating_sub(ready_face_boundary_count);
        let ready_face_count = faces.iter().filter(|face| face.exact_face_ready).count();
        let blocked_face_count = faces.len().saturating_sub(ready_face_count);
        let exact_surface_boundary_ready =
            !faces.is_empty() && blocked_face_boundary_count == 0 && bounds.exact_bounds_ready;
        let exact_closed_shell_ready = exact_surface_boundary_ready
            && topology.topology_ready
            && closure.exact_shell_ready
            && blocked_face_count == 0;

        let mut blockers = Vec::new();
        if shell.faces.is_empty() {
            blockers.push(BrepShellValidationBlocker::EmptyShell);
        }
        if !topology.topology_ready {
            blockers.push(BrepShellValidationBlocker::TopologyNotReady);
        }
        if !closure.exact_shell_ready {
            blockers.push(BrepShellValidationBlocker::ShellClosureNotReady);
        }
        if !bounds.exact_bounds_ready {
            blockers.push(BrepShellValidationBlocker::ShellBoundsNotReady);
        }
        if blocked_face_boundary_count > 0 || faces.is_empty() {
            blockers.push(BrepShellValidationBlocker::FaceBoundaryNotReady);
        }
        if blocked_face_count > 0 || faces.is_empty() {
            blockers.push(BrepShellValidationBlocker::FaceValidationNotReady);
        }

        Self {
            topology,
            closure,
            bounds,
            faces,
            ready_face_boundary_count,
            blocked_face_boundary_count,
            ready_face_count,
            blocked_face_count,
            exact_surface_boundary_ready,
            exact_closed_shell_ready,
            blockers,
        }
    }
}

impl BrepFaceValidationReport {
    /// Validate one face from retained shell evidence.
    ///
    /// This report is the face-level aggregation point for the shared BREP
    /// layer. Following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), it keeps surface preparation,
    /// trim topology, and optional derived-mesh evidence as explicit replayed
    /// facts. A caller can consume `exact_face_ready` only after the component
    /// reports agree; unsupported or lossy sources remain named blockers.
    pub fn from_shell_face(
        shell: &BrepShell,
        face: BrepFaceId,
        tessellation_manifest: Option<&BrepFaceTessellationManifest>,
    ) -> Self {
        let Some(source_face) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            return Self {
                face,
                face_found: false,
                surface_facts: None,
                surface_blockers: Vec::new(),
                trim_set: None,
                bounds: None,
                geometry: None,
                tessellation: None,
                blockers: vec![BrepFaceValidationBlocker::MissingFace],
                exact_face_boundary_ready: false,
                exact_bounds_ready: false,
                tessellation_ready: tessellation_manifest.is_none(),
                exact_face_ready: false,
            };
        };

        let mut blockers = Vec::new();
        let (surface_facts, surface_blockers, surface_ready) = shell
            .surfaces
            .iter()
            .find(|surface| surface.id == source_face.surface)
            .map(|surface| {
                let facts = surface.facts();
                let prepared = surface.prepare();
                let blockers = match prepared {
                    crate::surface::PreparedBrepSurface::Plane { .. } => Vec::new(),
                    crate::surface::PreparedBrepSurface::Blocked { blockers, .. } => blockers,
                };
                let ready = facts.exact_replay_ready && blockers.is_empty();
                (Some(facts), blockers, ready)
            })
            .unwrap_or((None, Vec::new(), false));
        if !surface_ready {
            blockers.push(BrepFaceValidationBlocker::SurfaceNotReady);
        }

        let trim_set = shell.trim_set_report(face);
        if !trim_set.trim_set_ready {
            blockers.push(BrepFaceValidationBlocker::TrimSetNotReady);
        }
        let bounds = shell.face_bounds_report(face);
        if !bounds.exact_bounds_ready {
            blockers.push(BrepFaceValidationBlocker::BoundsNotReady);
        }
        let geometry = shell.geometry_validation_report(face);
        if !geometry.geometry_ready {
            blockers.push(BrepFaceValidationBlocker::GeometryNotReady);
        }

        let tessellation = tessellation_manifest.map(|manifest| {
            BrepFaceTessellationReport::from_shell_face(shell, face, Some(manifest))
        });
        let tessellation_ready = tessellation
            .as_ref()
            .is_none_or(|report| report.exact_surface_handoff_ready);
        if !tessellation_ready {
            blockers.push(BrepFaceValidationBlocker::TessellationNotReady);
        }

        let exact_bounds_ready = bounds.exact_bounds_ready;
        let exact_face_boundary_ready =
            surface_ready && trim_set.trim_set_ready && geometry.geometry_ready;
        let exact_face_ready =
            exact_face_boundary_ready && exact_bounds_ready && tessellation_ready;
        Self {
            face,
            face_found: true,
            surface_facts,
            surface_blockers,
            trim_set: Some(trim_set),
            bounds: Some(bounds),
            geometry: Some(geometry),
            tessellation,
            blockers,
            exact_face_boundary_ready,
            exact_bounds_ready,
            tessellation_ready,
            exact_face_ready,
        }
    }
}

impl BrepGeometryValidationReport {
    /// Validate face boundary geometry against retained surface evidence.
    ///
    /// This is a first exact planar consistency report. It checks retained edge
    /// and vertex references, reuses trim-loop readiness, and certifies every
    /// boundary vertex against the face's support plane through
    /// `hyperlimit::PreparedPlane3`. Following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), unsupported
    /// surfaces and unknown point/plane relations remain explicit blockers
    /// rather than being hidden behind primitive-float tolerances. Full pcurve
    /// image equality and adjacent-face curve agreement remain future reports.
    pub fn from_shell_face(shell: &BrepShell, face: BrepFaceId) -> Self {
        let Some(face_record) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            return Self::blocked(
                face,
                false,
                vec![BrepGeometryValidationBlocker::MissingFace],
            );
        };
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

        let mut blockers = Vec::new();
        let mut missing_edge_count = 0;
        let mut degenerate_edge_count = 0;
        let mut missing_vertex_count = 0;
        let mut boundary_vertices = BTreeSet::new();
        for face_loop in face_record.loops() {
            for coedge in &face_loop.coedges {
                let Some(edge) = edge_by_id.get(&coedge.edge) else {
                    missing_edge_count += 1;
                    blockers.push(BrepGeometryValidationBlocker::MissingEdge);
                    continue;
                };
                if edge.is_degenerate() {
                    degenerate_edge_count += 1;
                    blockers.push(BrepGeometryValidationBlocker::DegenerateEdge);
                }
                boundary_vertices.insert(edge.start);
                boundary_vertices.insert(edge.end);
            }
        }

        let Some(surface) = shell
            .surfaces
            .iter()
            .find(|surface| surface.id == face_record.surface)
        else {
            blockers.push(BrepGeometryValidationBlocker::MissingSurface);
            return Self {
                face,
                face_found: true,
                boundary_vertex_count: boundary_vertices.len(),
                missing_edge_count,
                degenerate_edge_count,
                missing_vertex_count,
                on_surface_vertex_count: 0,
                off_surface_vertex_count: 0,
                unknown_vertex_surface_count: 0,
                trim_set_ready: false,
                geometry_ready: false,
                blockers,
            };
        };
        if !surface.facts().exact_replay_ready {
            blockers.push(BrepGeometryValidationBlocker::SurfaceNotReady);
        }

        let trim_set = shell.trim_set_report(face);
        if !trim_set.trim_set_ready {
            blockers.push(BrepGeometryValidationBlocker::TrimSetNotReady);
        }

        let mut on_surface_vertex_count = 0;
        let mut off_surface_vertex_count = 0;
        let mut unknown_vertex_surface_count = 0;
        if let BrepSurfaceKind::Plane(plane) = &surface.kind {
            if surface.facts().exact_replay_ready {
                let prepared = plane.prepare();
                for vertex_id in &boundary_vertices {
                    let Some(vertex) = vertex_by_id.get(vertex_id) else {
                        missing_vertex_count += 1;
                        blockers.push(BrepGeometryValidationBlocker::MissingVertex);
                        continue;
                    };
                    match prepared.classify_point(&vertex.point) {
                        PredicateOutcome::Decided { value, .. } => {
                            if value == PlaneSide::On {
                                on_surface_vertex_count += 1;
                            } else {
                                off_surface_vertex_count += 1;
                                blockers
                                    .push(BrepGeometryValidationBlocker::BoundaryVertexOffSurface);
                            }
                        }
                        PredicateOutcome::Unknown { .. } => {
                            unknown_vertex_surface_count += 1;
                            blockers
                                .push(BrepGeometryValidationBlocker::UnknownVertexSurfaceRelation);
                        }
                    }
                }
            }
        } else {
            blockers.push(BrepGeometryValidationBlocker::UnsupportedSurface);
        }

        blockers.sort_unstable();
        blockers.dedup();
        let geometry_ready = blockers.is_empty();
        Self {
            face,
            face_found: true,
            boundary_vertex_count: boundary_vertices.len(),
            missing_edge_count,
            degenerate_edge_count,
            missing_vertex_count,
            on_surface_vertex_count,
            off_surface_vertex_count,
            unknown_vertex_surface_count,
            trim_set_ready: trim_set.trim_set_ready,
            blockers,
            geometry_ready,
        }
    }

    fn blocked(
        face: BrepFaceId,
        face_found: bool,
        blockers: Vec<BrepGeometryValidationBlocker>,
    ) -> Self {
        Self {
            face,
            face_found,
            boundary_vertex_count: 0,
            missing_edge_count: 0,
            degenerate_edge_count: 0,
            missing_vertex_count: 0,
            on_surface_vertex_count: 0,
            off_surface_vertex_count: 0,
            unknown_vertex_surface_count: 0,
            trim_set_ready: false,
            blockers,
            geometry_ready: false,
        }
    }
}

impl BrepShell {
    /// Validate retained shell evidence without promoting it to a solid.
    pub fn shell_validation_report(&self) -> BrepShellValidationReport {
        BrepShellValidationReport::from_shell(self)
    }

    /// Validate retained face geometry against support-surface evidence.
    pub fn geometry_validation_report(&self, face: BrepFaceId) -> BrepGeometryValidationReport {
        BrepGeometryValidationReport::from_shell_face(self, face)
    }

    /// Validate retained face evidence and optional tessellation evidence.
    pub fn face_validation_report(
        &self,
        face: BrepFaceId,
        tessellation_manifest: Option<&BrepFaceTessellationManifest>,
    ) -> BrepFaceValidationReport {
        BrepFaceValidationReport::from_shell_face(self, face, tessellation_manifest)
    }
}
