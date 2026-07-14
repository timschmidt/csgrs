//! Tessellation readiness and derived-mesh handoff reports.
//!
//! Tessellated output is useful for display, `hypermesh`, voxelization, and
//! physics, but it is derived evidence. This module keeps that boundary
//! explicit: a mesh can be ready for downstream handoff only after the retained
//! BREP face/shell evidence and the tessellation manifest agree.

use std::collections::{BTreeMap, BTreeSet};

use crate::provenance::{BrepConstructionManifest, BrepConstructionProvenanceReport};
use crate::report::BrepShellClosureReport;
use crate::surface::{BrepSurfaceKind, BrepSurfaceSource};
use crate::topology::{BrepFaceId, BrepShell, BrepVertexId};
use crate::triangle::{BrepTriangleMeshBlocker, collect_loop_vertices};

/// Tessellation producer declared by a derived-mesh manifest.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepTessellationBackend {
    /// Exact planar lowering through `hypertri` earcut triangulation.
    HypertriPlanar,
    /// External tessellation replayed by exact Hyper predicates.
    ExactExternalReplay,
    /// Primitive-float or tolerance adapter for preview/export only.
    LossyPreviewAdapter,
    /// Producer did not declare a backend.
    Unknown,
}

/// Producer-declared tessellation manifest for one BREP face.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepFaceTessellationManifest {
    /// Source face whose trim region was tessellated.
    pub face: BrepFaceId,
    /// Declared tessellation backend.
    pub backend: BrepTessellationBackend,
    /// Number of generated triangles.
    pub triangle_count: usize,
    /// Number of generated lifted 3D vertices.
    pub lifted_vertex_count: usize,
    /// Number of generated boundary mesh edges linked to BREP coedges.
    pub boundary_edge_count: usize,
    /// Number of inserted Steiner points.
    pub steiner_point_count: usize,
    /// Whether UV triangulation was replayed exactly or certified.
    pub exact_uv_triangulation: bool,
    /// Whether lifted 3D vertices were certified incident to the face surface.
    pub exact_lifted_incidence: bool,
    /// Whether all BREP boundary coedges are represented in the derived mesh.
    pub preserves_boundary_edges: bool,
    /// Whether output coordinates/topology came from a lossy adapter.
    pub lossy_adapter_output: bool,
}

impl BrepFaceTessellationManifest {
    /// Construct an exact planar tessellation manifest.
    pub const fn exact_planar(
        face: BrepFaceId,
        triangle_count: usize,
        lifted_vertex_count: usize,
        boundary_edge_count: usize,
        steiner_point_count: usize,
    ) -> Self {
        Self {
            face,
            backend: BrepTessellationBackend::HypertriPlanar,
            triangle_count,
            lifted_vertex_count,
            boundary_edge_count,
            steiner_point_count,
            exact_uv_triangulation: true,
            exact_lifted_incidence: true,
            preserves_boundary_edges: true,
            lossy_adapter_output: false,
        }
    }

    /// Construct an exact planar manifest by replaying retained face loops
    /// through the BREP surface frame and `hypertri`'s exact earcut route.
    ///
    /// Derived triangle counts are accepted only after source loops project
    /// through the exact retained frame and `hypertri` returns an index stream.
    /// The manifest remains derived mesh evidence; retained BREP topology stays
    /// authoritative.
    pub fn from_exact_planar_shell_face(shell: &BrepShell, face: BrepFaceId) -> Option<Self> {
        let source_face = shell.faces.iter().find(|candidate| candidate.id == face)?;
        let surface = shell
            .surfaces
            .iter()
            .find(|surface| surface.id == source_face.surface)?;
        if !matches!(surface.kind, BrepSurfaceKind::Plane(_)) {
            return None;
        }
        let frame = surface.frame_report();
        if !frame.exact_frame_ready {
            return None;
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
            .collect::<BTreeMap<BrepVertexId, _>>();
        let mut blockers = Vec::<BrepTriangleMeshBlocker>::new();
        let mut points2 = Vec::new();
        let mut hole_indices = Vec::new();

        push_manifest_loop_uvs(
            &source_face.outer.coedges,
            surface,
            &edge_by_id,
            &vertex_by_id,
            &mut blockers,
            &mut points2,
        )?;
        for inner in &source_face.inner {
            hole_indices.push(points2.len());
            push_manifest_loop_uvs(
                &inner.coedges,
                surface,
                &edge_by_id,
                &vertex_by_id,
                &mut blockers,
                &mut points2,
            )?;
        }
        if !blockers.is_empty() || points2.len() < 3 {
            return None;
        }

        let exact = points2
            .iter()
            .map(|point| hypertri::Point2::new(point.x.clone(), point.y.clone()))
            .collect::<Vec<_>>();
        let indices = hypertri::earcut(&exact, &hole_indices).ok()?;
        let triangle_count = indices.len() / 3;
        (triangle_count > 0).then(|| {
            let boundary_edge_count = source_face
                .loops()
                .map(|face_loop| face_loop.coedges.len())
                .sum();
            Self::exact_planar(face, triangle_count, points2.len(), boundary_edge_count, 0)
        })
    }
}

fn push_manifest_loop_uvs(
    coedges: &[crate::topology::BrepCoedge],
    surface: &crate::surface::BrepSurface,
    edge_by_id: &BTreeMap<crate::topology::BrepEdgeId, crate::topology::BrepEdge>,
    vertex_by_id: &BTreeMap<BrepVertexId, &crate::topology::BrepVertex>,
    blockers: &mut Vec<BrepTriangleMeshBlocker>,
    points2: &mut Vec<hyperlimit::Point2>,
) -> Option<()> {
    let vertices = collect_loop_vertices(coedges, edge_by_id, vertex_by_id, blockers)?;
    for point in vertices {
        let projection = surface.project_frame_point(point);
        points2.push(projection.uv?);
    }
    Some(())
}

/// Explicit blocker for derived face-mesh readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepTessellationBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a missing surface.
    MissingSurface,
    /// Face surface is not a supported exact plane.
    UnsupportedSurface,
    /// Surface provenance is lossy or unknown.
    NonExactSurfaceSource,
    /// Face has an empty loop.
    EmptyLoop,
    /// Face boundary has fewer than three coedges.
    TooFewBoundaryCoedges,
    /// Source face trim loops are not ready topology evidence.
    TrimLoopNotReady,
    /// Tessellation manifest was not supplied.
    MissingManifest,
    /// Manifest face id does not match the requested face.
    ManifestFaceMismatch,
    /// Manifest backend was unknown.
    UnknownBackend,
    /// Manifest backend is preview/lossy only.
    LossyBackend,
    /// Manifest has no triangles.
    EmptyTriangleSet,
    /// Manifest has no lifted vertices.
    EmptyLiftedVertices,
    /// Manifest does not replay UV triangulation exactly.
    MissingExactUvReplay,
    /// Manifest does not certify lifted 3D surface incidence.
    MissingLiftedIncidenceReplay,
    /// Manifest does not preserve all BREP boundary coedges.
    MissingBoundaryReplay,
    /// Manifest declares lossy adapter output.
    LossyAdapterOutput,
}

/// Readiness report for one face's derived tessellation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepFaceTessellationReport {
    /// Source face.
    pub face: BrepFaceId,
    /// Number of loops on the source face.
    pub source_loop_count: usize,
    /// Number of BREP coedges across source loops.
    pub source_coedge_count: usize,
    /// Declared backend when a manifest was supplied.
    pub backend: Option<BrepTessellationBackend>,
    /// Declared triangle count.
    pub triangle_count: usize,
    /// Declared lifted vertex count.
    pub lifted_vertex_count: usize,
    /// Declared boundary edge count.
    pub boundary_edge_count: usize,
    /// Declared Steiner point count.
    pub steiner_point_count: usize,
    /// Explicit blockers.
    pub blockers: Vec<BrepTessellationBlocker>,
    /// Whether source face trim loops are ready as retained topology evidence.
    pub trim_set_ready: bool,
    /// Whether the face is ready for exact surface mesh handoff.
    pub exact_surface_handoff_ready: bool,
    /// Whether generated mesh data remains derived, not authoritative BREP topology.
    pub derived_mesh_only: bool,
}

/// Shell-level derived tessellation readiness report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepShellTessellationReport {
    /// Shell closure report replayed with tessellation evidence.
    pub shell_closure: BrepShellClosureReport,
    /// Face tessellation reports.
    pub faces: Vec<BrepFaceTessellationReport>,
    /// Number of source faces in the shell.
    pub source_face_count: usize,
    /// Number of faces ready for exact surface handoff.
    pub ready_face_count: usize,
    /// Number of faces blocked from exact surface handoff.
    pub blocked_face_count: usize,
    /// Total declared triangle count.
    pub triangle_count: usize,
    /// Total declared lifted vertex count.
    pub lifted_vertex_count: usize,
    /// Total declared boundary edge count.
    pub boundary_edge_count: usize,
    /// Total declared Steiner point count.
    pub steiner_point_count: usize,
    /// Whether every source face has exact tessellation evidence.
    pub exact_surface_handoff_ready: bool,
    /// Whether the shell is closed and every source face has exact tessellation
    /// evidence.
    pub exact_solid_handoff_ready: bool,
    /// Whether generated mesh data remains derived, not authoritative BREP topology.
    pub derived_mesh_only: bool,
}

impl BrepFaceTessellationReport {
    /// Build a face tessellation report from retained BREP evidence and an
    /// optional producer manifest.
    ///
    /// The derived mesh is consumable only when source-face, trim-boundary, UV
    /// triangulation, and lifted-surface incidence evidence are retained
    /// together.
    pub fn from_shell_face(
        shell: &BrepShell,
        face: BrepFaceId,
        manifest: Option<&BrepFaceTessellationManifest>,
    ) -> Self {
        let mut blockers = BTreeSet::new();
        let mut source_loop_count = 0_usize;
        let mut source_coedge_count = 0_usize;

        let Some(source_face) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            blockers.insert(BrepTessellationBlocker::MissingFace);
            return Self::blocked(face, manifest, blockers);
        };
        for face_loop in source_face.loops() {
            source_loop_count += 1;
            source_coedge_count += face_loop.coedges.len();
            if face_loop.is_empty() {
                blockers.insert(BrepTessellationBlocker::EmptyLoop);
            }
        }
        if source_coedge_count < 3 {
            blockers.insert(BrepTessellationBlocker::TooFewBoundaryCoedges);
        }
        let trim_set = shell.trim_set_report(face);
        if !trim_set.trim_set_ready {
            blockers.insert(BrepTessellationBlocker::TrimLoopNotReady);
        }

        match shell
            .surfaces
            .iter()
            .find(|surface| surface.id == source_face.surface)
        {
            Some(surface) => {
                if !matches!(surface.kind, BrepSurfaceKind::Plane(_))
                    || !surface.is_supported_exact_plane()
                {
                    blockers.insert(BrepTessellationBlocker::UnsupportedSurface);
                }
                if !matches!(
                    surface.source,
                    BrepSurfaceSource::ExactConstruction | BrepSurfaceSource::ExactImport
                ) {
                    blockers.insert(BrepTessellationBlocker::NonExactSurfaceSource);
                }
            }
            None => {
                blockers.insert(BrepTessellationBlocker::MissingSurface);
            }
        }

        if let Some(manifest) = manifest {
            if manifest.face != face {
                blockers.insert(BrepTessellationBlocker::ManifestFaceMismatch);
            }
            match manifest.backend {
                BrepTessellationBackend::Unknown => {
                    blockers.insert(BrepTessellationBlocker::UnknownBackend);
                }
                BrepTessellationBackend::LossyPreviewAdapter => {
                    blockers.insert(BrepTessellationBlocker::LossyBackend);
                }
                BrepTessellationBackend::HypertriPlanar
                | BrepTessellationBackend::ExactExternalReplay => {}
            }
            if manifest.triangle_count == 0 {
                blockers.insert(BrepTessellationBlocker::EmptyTriangleSet);
            }
            if manifest.lifted_vertex_count == 0 {
                blockers.insert(BrepTessellationBlocker::EmptyLiftedVertices);
            }
            if !manifest.exact_uv_triangulation {
                blockers.insert(BrepTessellationBlocker::MissingExactUvReplay);
            }
            if !manifest.exact_lifted_incidence {
                blockers.insert(BrepTessellationBlocker::MissingLiftedIncidenceReplay);
            }
            if !manifest.preserves_boundary_edges
                || manifest.boundary_edge_count < source_coedge_count
            {
                blockers.insert(BrepTessellationBlocker::MissingBoundaryReplay);
            }
            if manifest.lossy_adapter_output {
                blockers.insert(BrepTessellationBlocker::LossyAdapterOutput);
            }
        } else {
            blockers.insert(BrepTessellationBlocker::MissingManifest);
        }

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        let exact_surface_handoff_ready = blockers.is_empty();
        let (
            backend,
            triangle_count,
            lifted_vertex_count,
            boundary_edge_count,
            steiner_point_count,
        ) = manifest.map_or((None, 0, 0, 0, 0), |manifest| {
            (
                Some(manifest.backend),
                manifest.triangle_count,
                manifest.lifted_vertex_count,
                manifest.boundary_edge_count,
                manifest.steiner_point_count,
            )
        });
        Self {
            face,
            source_loop_count,
            source_coedge_count,
            backend,
            triangle_count,
            lifted_vertex_count,
            boundary_edge_count,
            steiner_point_count,
            blockers,
            trim_set_ready: trim_set.trim_set_ready,
            exact_surface_handoff_ready,
            derived_mesh_only: true,
        }
    }

    fn blocked(
        face: BrepFaceId,
        manifest: Option<&BrepFaceTessellationManifest>,
        blockers: BTreeSet<BrepTessellationBlocker>,
    ) -> Self {
        let (
            backend,
            triangle_count,
            lifted_vertex_count,
            boundary_edge_count,
            steiner_point_count,
        ) = manifest.map_or((None, 0, 0, 0, 0), |manifest| {
            (
                Some(manifest.backend),
                manifest.triangle_count,
                manifest.lifted_vertex_count,
                manifest.boundary_edge_count,
                manifest.steiner_point_count,
            )
        });
        Self {
            face,
            source_loop_count: 0,
            source_coedge_count: 0,
            backend,
            triangle_count,
            lifted_vertex_count,
            boundary_edge_count,
            steiner_point_count,
            blockers: blockers.into_iter().collect(),
            trim_set_ready: false,
            exact_surface_handoff_ready: false,
            derived_mesh_only: true,
        }
    }
}

impl BrepShellTessellationReport {
    /// Build a shell tessellation readiness report from per-face manifests.
    ///
    /// A shell-level mesh proposal is exact-ready only when retained shell
    /// evidence and every source-face tessellation report replay exactly. The
    /// triangles remain derived and never replace retained BREP topology.
    pub fn from_shell_manifests(
        shell: &BrepShell,
        manifests: &[BrepFaceTessellationManifest],
    ) -> Self {
        let shell_closure = shell.audit_closure();
        let faces = shell
            .faces
            .iter()
            .map(|face| {
                let manifest = manifests.iter().find(|manifest| manifest.face == face.id);
                BrepFaceTessellationReport::from_shell_face(shell, face.id, manifest)
            })
            .collect::<Vec<_>>();
        Self::from_parts(shell_closure, faces)
    }

    /// Build a shell tessellation report by deriving exact planar manifests
    /// from retained faces before replaying the normal manifest checks.
    ///
    /// This is the production path for current planar BREP faces: generated
    /// manifests come from exact frame projection plus `hypertri`, then the
    /// existing readiness report checks source topology, exact surface status,
    /// boundary preservation, and shell closure. Failed faces simply have no
    /// manifest and remain blocked by the same report vocabulary.
    pub fn from_exact_planar_shell(shell: &BrepShell) -> Self {
        let manifests = shell.exact_planar_tessellation_manifests();
        Self::from_shell_manifests(shell, &manifests)
    }

    fn from_parts(
        shell_closure: BrepShellClosureReport,
        faces: Vec<BrepFaceTessellationReport>,
    ) -> Self {
        let ready_face_count = faces
            .iter()
            .filter(|face| face.exact_surface_handoff_ready)
            .count();
        let blocked_face_count = faces.len().saturating_sub(ready_face_count);
        let triangle_count = faces.iter().map(|face| face.triangle_count).sum();
        let lifted_vertex_count = faces.iter().map(|face| face.lifted_vertex_count).sum();
        let boundary_edge_count = faces.iter().map(|face| face.boundary_edge_count).sum();
        let steiner_point_count = faces.iter().map(|face| face.steiner_point_count).sum();
        let exact_surface_handoff_ready = !faces.is_empty() && blocked_face_count == 0;
        let exact_solid_handoff_ready =
            exact_surface_handoff_ready && shell_closure.exact_shell_ready;
        Self {
            shell_closure,
            source_face_count: faces.len(),
            faces,
            ready_face_count,
            blocked_face_count,
            triangle_count,
            lifted_vertex_count,
            boundary_edge_count,
            steiner_point_count,
            exact_surface_handoff_ready,
            exact_solid_handoff_ready,
            derived_mesh_only: true,
        }
    }
}

/// Shell-level derived mesh handoff report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepMeshHandoffReport {
    /// Shell closure report replayed at handoff time.
    pub shell_closure: BrepShellClosureReport,
    /// Shell tessellation readiness report replayed at handoff time.
    pub tessellation: BrepShellTessellationReport,
    /// Optional construction provenance freshness report.
    pub construction: Option<BrepConstructionProvenanceReport>,
    /// Face tessellation reports.
    pub faces: Vec<BrepFaceTessellationReport>,
    /// Number of faces ready for exact surface handoff.
    pub ready_face_count: usize,
    /// Number of faces blocked from exact surface handoff.
    pub blocked_face_count: usize,
    /// Total declared triangle count.
    pub triangle_count: usize,
    /// Total declared lifted vertex count.
    pub lifted_vertex_count: usize,
    /// Whether all supplied face meshes are ready as exact derived surfaces.
    pub exact_surface_handoff_ready: bool,
    /// Whether the closed shell and all face meshes are ready for exact solid handoff.
    pub exact_solid_handoff_ready: bool,
    /// Whether generated mesh data remains derived, not authoritative BREP topology.
    pub derived_mesh_only: bool,
}

impl BrepMeshHandoffReport {
    /// Build a shell mesh handoff report from per-face tessellation manifests.
    pub fn from_shell_manifests(
        shell: &BrepShell,
        manifests: &[BrepFaceTessellationManifest],
    ) -> Self {
        Self::from_shell_manifests_with_construction(shell, manifests, None)
    }

    /// Build a mesh handoff by deriving exact planar face manifests from the
    /// retained shell and replaying them through the normal handoff gate.
    pub fn from_exact_planar_shell(shell: &BrepShell) -> Self {
        let manifests = shell.exact_planar_tessellation_manifests();
        Self::from_shell_manifests(shell, &manifests)
    }

    /// Build a shell mesh handoff report with construction provenance replay.
    pub fn from_shell_manifests_with_construction(
        shell: &BrepShell,
        manifests: &[BrepFaceTessellationManifest],
        construction: Option<&BrepConstructionManifest>,
    ) -> Self {
        let tessellation = BrepShellTessellationReport::from_shell_manifests(shell, manifests);
        let shell_closure = tessellation.shell_closure.clone();
        let construction = construction.map(|manifest| manifest.report(shell));
        let construction_fresh = construction
            .as_ref()
            .is_none_or(|report| report.construction_fresh);
        let exact_solid_handoff_ready =
            tessellation.exact_solid_handoff_ready && construction_fresh;
        Self {
            shell_closure,
            faces: tessellation.faces.clone(),
            ready_face_count: tessellation.ready_face_count,
            blocked_face_count: tessellation.blocked_face_count,
            triangle_count: tessellation.triangle_count,
            lifted_vertex_count: tessellation.lifted_vertex_count,
            exact_surface_handoff_ready: tessellation.exact_surface_handoff_ready,
            tessellation,
            construction,
            exact_solid_handoff_ready,
            derived_mesh_only: true,
        }
    }
}

impl BrepShell {
    /// Derive exact planar tessellation manifests for every face whose retained
    /// loops can be projected and triangulated without adapter evidence.
    pub fn exact_planar_tessellation_manifests(&self) -> Vec<BrepFaceTessellationManifest> {
        self.faces
            .iter()
            .filter_map(|face| {
                BrepFaceTessellationManifest::from_exact_planar_shell_face(self, face.id)
            })
            .collect()
    }

    /// Replay retained planar faces through exact tessellation readiness.
    pub fn exact_planar_tessellation_report(&self) -> BrepShellTessellationReport {
        BrepShellTessellationReport::from_exact_planar_shell(self)
    }

    /// Replay retained planar faces as an exact derived mesh handoff.
    pub fn exact_planar_mesh_handoff_report(&self) -> BrepMeshHandoffReport {
        BrepMeshHandoffReport::from_exact_planar_shell(self)
    }
}
