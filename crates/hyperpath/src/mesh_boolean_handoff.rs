//! Opaque exact-mesh handoff operands for path-domain boolean programs.
//!
//! `hyperpath` sometimes needs to compose a path/CAM/EDA boolean program
//! around a mesh source whose topology was already accepted by `hypermesh` or
//! another exact producer. This module is that boundary. It deliberately stores
//! the `hypermesh` object and its checked handoff package as an opaque operand;
//! it does not build, split, triangulate, or reinterpret mesh topology in
//! `hyperpath`.
//!
//! The replay checks follow Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): cached geometric artifacts may cross
//! crate boundaries only while their exact evidence replays from the retained
//! object. Boolean use additionally requires a closed-solid domain, matching
//! Requicha, "Representations for Rigid Solids: Theory, Methods, and Systems,"
//! *ACM Computing Surveys* 12.4 (1980), because the current path boolean API
//! consumes regularized solid operands.

use hypermesh::exact::{
    ExactMesh, ExactMeshConsumerDomain, ExactMeshDomainSummary, ExactMeshHandoffPackage,
    exact_mesh_handoff_package,
};

use crate::mesh_boolean::PathMeshBooleanError;

/// A retained, opaque exact `hypermesh` solid accepted as a boolean operand.
///
/// This is the correct `hyperpath` integration point for richer mesh-producing
/// domains, including future curved PCB pads, curved board outlines, or CAM
/// rest-geometry packages. The mesh topology remains owned by its producer and
/// by `hypermesh`; `hyperpath` only retains the exact mesh plus package facts
/// needed to prove that the operand is still a closed-solid handoff when a
/// boolean program is replayed.
#[derive(Clone, Debug, PartialEq)]
pub struct PathExactMeshHandoffSource {
    mesh: ExactMesh,
    package: ExactMeshHandoffPackage,
    domain_summary: ExactMeshDomainSummary,
}

impl PathExactMeshHandoffSource {
    /// Accept an existing `hypermesh` exact mesh and retained handoff package.
    ///
    /// The package must replay against `mesh`, must carry the closed-solid
    /// domain, and the copied domain summary must replay from the package. This
    /// check is intentionally repeated by [`Self::to_exact_mesh`] so a mutated
    /// retained source cannot silently authorize stale topology later.
    pub fn new(
        mesh: ExactMesh,
        package: ExactMeshHandoffPackage,
    ) -> Result<Self, PathMeshBooleanError> {
        package
            .validate_against_mesh(&mesh)
            .map_err(|error| PathMeshBooleanError::MeshHandoff(format!("{error:?}")))?;
        package
            .require_domain(ExactMeshConsumerDomain::Solid)
            .map_err(|error| PathMeshBooleanError::MeshHandoff(format!("{error:?}")))?;
        let domain_summary = package.domain_summary();
        domain_summary
            .require_domain_against_mesh(&package, &mesh, ExactMeshConsumerDomain::Solid)
            .map_err(|error| PathMeshBooleanError::MeshHandoff(format!("{error:?}")))?;
        Ok(Self {
            mesh,
            package,
            domain_summary,
        })
    }

    /// Package an already-built exact `hypermesh` mesh as a solid operand.
    ///
    /// This convenience constructor still leaves topology construction outside
    /// `hyperpath`: it asks `hypermesh` to produce the handoff package, then
    /// applies the same closed-solid acceptance checks as [`Self::new`].
    pub fn from_exact_mesh(mesh: ExactMesh) -> Result<Self, PathMeshBooleanError> {
        let package = exact_mesh_handoff_package(&mesh)
            .map_err(|error| PathMeshBooleanError::MeshHandoff(format!("{error:?}")))?;
        Self::new(mesh, package)
    }

    /// Return the retained exact mesh.
    pub const fn mesh(&self) -> &ExactMesh {
        &self.mesh
    }

    /// Return the retained `hypermesh` handoff package.
    pub const fn package(&self) -> &ExactMeshHandoffPackage {
        &self.package
    }

    /// Return the copied domain summary checked beside this source.
    pub const fn domain_summary(&self) -> &ExactMeshDomainSummary {
        &self.domain_summary
    }

    /// Replay package and summary evidence, then return the exact mesh operand.
    ///
    /// The returned mesh is a clone because existing boolean program code owns
    /// operands step-by-step. The clone is not topology authority by itself:
    /// the source revalidates the retained package and summary first, preserving
    /// Yap's exact-object/cached-artifact separation at each replay boundary.
    pub fn to_exact_mesh(&self) -> Result<ExactMesh, PathMeshBooleanError> {
        self.package
            .validate_against_mesh(&self.mesh)
            .map_err(|error| PathMeshBooleanError::MeshHandoff(format!("{error:?}")))?;
        self.domain_summary
            .require_domain_against_mesh(&self.package, &self.mesh, ExactMeshConsumerDomain::Solid)
            .map_err(|error| PathMeshBooleanError::MeshHandoff(format!("{error:?}")))?;
        Ok(self.mesh.clone())
    }
}
