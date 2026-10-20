//! Explicit derived triangulations with retained source ownership.

use super::*;
pub use crate::meshing::{
    ChordalApproximationPolicy, ChordalSourceRelation, ExactPlanarFaceTriangulation,
    TessellationError,
};

/// A derived chordal approximation retaining its exact source revision.
#[derive(Clone, Debug)]
pub struct ChordalFaceApproximation {
    source: Model,
    data: crate::meshing::ArenaChordalFaceApproximation,
}

impl ChordalFaceApproximation {
    /// Returns the exact source face in its retained revision.
    pub fn source_face(&self) -> FaceRef<'_> {
        FaceRef {
            model: &self.source,
            id: self.data.source_face(),
        }
    }
    /// Returns the source carrier family.
    pub fn source_surface_kind(&self) -> SurfaceKind {
        self.data.source_surface_kind()
    }
    /// Returns the explicitly requested approximation policy.
    pub fn policy(&self) -> ChordalApproximationPolicy {
        self.data.policy()
    }
    /// Returns the certified relationship to the source.
    pub fn source_relation(&self) -> ChordalSourceRelation {
        self.data.source_relation()
    }
    /// Borrows source parameter samples.
    pub fn parameters(&self) -> &[Point2] {
        self.data.parameters()
    }
    /// Borrows derived model-space samples.
    pub fn points(&self) -> &[Point3] {
        self.data.points()
    }
    /// Borrows triangle indices into the samples.
    pub fn triangles(&self) -> &[[usize; 3]] {
        self.data.triangles()
    }
}

/// Triangulates a supported planar face with exact vertices.
pub fn triangulate_planar_face(
    face: FaceRef<'_>,
) -> Result<ExactPlanarFaceTriangulation, TessellationError> {
    crate::meshing::triangulate_planar_face(&face.model.inner, face.id)
}

/// Produces an explicit derived chordal approximation of a bound face.
pub fn approximate_face_chordally(
    face: FaceRef<'_>,
    policy: ChordalApproximationPolicy,
) -> Result<ChordalFaceApproximation, TessellationError> {
    crate::meshing::approximate_face_chordally(&face.model.inner, face.id, policy).map(|data| {
        ChordalFaceApproximation {
            source: face.model.clone(),
            data,
        }
    })
}
