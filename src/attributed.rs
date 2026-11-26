//! Optional face attributes attached to native triangle geometry.
//!
//! [`AttributedMesh`] is deliberately a boundary carrier, not a second mesh
//! implementation. Geometry remains a [`TriangleMesh`]; metadata and authored
//! normals are parallel arrays whose lengths are checked at construction.
//! Alignment does not validate mesh topology or certify the authored normals.
//! Modeling consumes the native mesh; there is no automatic attribute propagation
//! through a Boolean or other geometry-changing operation.

use hyperlattice::Vector3;
use hypermesh::TriangleMesh;
use std::sync::{Arc, OnceLock};

/// Invalid alignment between native geometry and attached attributes.
#[derive(Clone, Copy, Debug, Eq, PartialEq, thiserror::Error)]
pub enum AttributeAlignmentError {
    /// The number of metadata rows does not match the number of triangles.
    #[error("face metadata has {actual} rows for {expected} triangles")]
    FaceMetadata {
        /// Number of triangles in the native mesh.
        expected: usize,
        /// Number of supplied metadata rows.
        actual: usize,
    },
    /// The number of authored normals does not match the number of positions.
    #[error("authored normals have {actual} rows for {expected} positions")]
    AuthoredNormals {
        /// Number of positions in the native mesh.
        expected: usize,
        /// Number of supplied normal rows.
        actual: usize,
    },
}

/// Native triangle geometry with optional, index-aligned boundary attributes.
///
/// Boolean and modeling operations consume [`TriangleMesh`] directly.
/// Callers construct this type only when an interchange format or renderer
/// needs one metadata value per face or an authored normal per position.
/// Metadata is required even when normals are absent. A clone shares native mesh
/// allocations and the successful export cache, while cloning attribute vectors.
/// Equality compares geometry buffers and attribute rows, not construction
/// qualifications, cached exports, or equality of the represented filled sets.
#[derive(Clone, Debug)]
pub struct AttributedMesh<M> {
    /// Authoritative mesh, including its independent construction qualifications.
    geometry: TriangleMesh,
    /// Exactly one metadata row per triangle, preserving triangle order.
    face_metadata: Vec<M>,
    /// Optional per-position rows, retained without normalization or direction checks.
    authored_normals: Option<Vec<Vector3>>,
    /// Shared first-success export cache, tied to immutable geometry and normal rows.
    /// Metadata mappings reuse it; a newly constructed sidecar starts a fresh cache.
    /// No eviction, reset or retained-byte cap is provided.
    exact_gpu: Arc<OnceLock<hypermesh::ExactGpuMeshBuffers>>,
}

impl<M: PartialEq> PartialEq for AttributedMesh<M> {
    /// Compare native buffer values, metadata and authored normals in that order.
    /// The cache and native construction evidence do not participate; this is
    /// representation equality rather than a geometric set-equivalence predicate.
    fn eq(&self, other: &Self) -> bool {
        self.geometry == other.geometry
            && self.face_metadata == other.face_metadata
            && self.authored_normals == other.authored_normals
    }
}

impl<M> AttributedMesh<M> {
    /// Attach one metadata row per triangle without authored normals.
    ///
    /// Rejects a row-count mismatch; does not validate triangle indices or topology.
    /// Later graphics export derives face normals from the geometry.
    pub fn new(
        geometry: TriangleMesh,
        face_metadata: Vec<M>,
    ) -> Result<Self, AttributeAlignmentError> {
        Self::with_authored_normals(geometry, face_metadata, None)
    }

    /// Attach metadata and optional authored normals after checking row counts.
    ///
    /// Metadata alignment is checked first, then normals against position count.
    /// No topology, index, unit-length or normal-direction test is performed.
    /// Zero and nonunit authored normals remain unchanged for boundary export.
    pub fn with_authored_normals(
        geometry: TriangleMesh,
        face_metadata: Vec<M>,
        authored_normals: Option<Vec<Vector3>>,
    ) -> Result<Self, AttributeAlignmentError> {
        if face_metadata.len() != geometry.triangles.len() {
            return Err(AttributeAlignmentError::FaceMetadata {
                expected: geometry.triangles.len(),
                actual: face_metadata.len(),
            });
        }
        if let Some(normals) = &authored_normals
            && normals.len() != geometry.positions.len()
        {
            return Err(AttributeAlignmentError::AuthoredNormals {
                expected: geometry.positions.len(),
                actual: normals.len(),
            });
        }
        Ok(Self {
            geometry,
            face_metadata,
            authored_normals,
            exact_gpu: Arc::new(OnceLock::new()),
        })
    }

    /// Borrows the authoritative native geometry.
    pub const fn geometry(&self) -> &TriangleMesh {
        &self.geometry
    }

    /// Borrows metadata in native triangle order.
    pub fn face_metadata(&self) -> &[M] {
        &self.face_metadata
    }

    /// Borrow authored normals in position order; `None` requests derived export normals.
    pub fn authored_normals(&self) -> Option<&[Vector3]> {
        self.authored_normals.as_deref()
    }

    /// Consume the sidecar and return geometry, metadata and authored normals.
    ///
    /// Drops this sidecar's export-cache reference. Other clones or metadata
    /// mappings may still retain the cached buffer.
    pub fn into_parts(self) -> (TriangleMesh, Vec<M>, Option<Vec<Vector3>>) {
        (self.geometry, self.face_metadata, self.authored_normals)
    }

    /// Map each metadata row in triangle order while retaining geometry and normals.
    ///
    /// Moves the successful export-cache handle unchanged because metadata does
    /// not affect the exported rows. The callback runs once per metadata row.
    pub fn map_metadata<N>(self, f: impl FnMut(M) -> N) -> AttributedMesh<N> {
        AttributedMesh {
            geometry: self.geometry,
            face_metadata: self.face_metadata.into_iter().map(f).collect(),
            authored_normals: self.authored_normals,
            exact_gpu: self.exact_gpu,
        }
    }

    /// Builds exact GPU rows, using authored per-position normals when present.
    ///
    /// This is an interchange/rendering boundary only. Native geometry does
    /// not absorb authored normals or depend on them for modeling predicates.
    /// Successful buffers are shared by clones and metadata mappings for their
    /// lifetime. Failures are returned by value and retried on the next call,
    /// so stronger scalar evidence can make a formerly undecided export succeed.
    /// Concurrent callers may compute the same exact rows before sharing the
    /// first success. A failing concurrent attempt returns that success if it is
    /// already published when the failure is checked. This cache retains one
    /// buffer until the last sharing owner is dropped, without a heap-byte limit.
    ///
    /// Authored normals are copied as supplied: they bypass derived-normal
    /// nondegeneracy checks and need not be unit length or match the surface.
    /// Success with authored normals therefore does not certify mesh validity.
    ///
    /// Migration: the error type is now owned `GpuMeshError`, replacing
    /// `&GpuMeshError`; callers that previously cloned the error can remove that
    /// conversion. Successful buffers are still borrowed from this sidecar.
    pub fn exact_gpu_mesh_buffers(
        &self,
    ) -> Result<&hypermesh::ExactGpuMeshBuffers, hypermesh::GpuMeshError> {
        if let Some(buffers) = self.exact_gpu.get() {
            return Ok(buffers);
        }
        match self.build_exact_gpu_mesh_buffers() {
            Ok(buffers) => Ok(self.exact_gpu.get_or_init(|| buffers)),
            Err(error) => self.exact_gpu.get().ok_or(error),
        }
    }

    /// Build uncached triangle-corner rows after checking every source index.
    ///
    /// Without authored normals, delegate to native flat-normal construction and
    /// its failure checks. With authored normals, copy aligned position/normal
    /// values directly without geometric normal validation. Native buffer assembly
    /// supplies sequential `u32` indices and can reject index-capacity overflow.
    fn build_exact_gpu_mesh_buffers(
        &self,
    ) -> Result<hypermesh::ExactGpuMeshBuffers, hypermesh::GpuMeshError> {
        let Some(normals) = &self.authored_normals else {
            return self.geometry.to_exact_gpu_mesh_buffers();
        };
        for (triangle_offset, triangle) in self.geometry.triangles.iter().enumerate() {
            for (corner, index) in triangle.indices().into_iter().enumerate() {
                if index >= self.geometry.positions.len() {
                    return Err(hypermesh::GpuMeshError::SourceTriangleIndexOutOfBounds {
                        triangle: triangle_offset,
                        corner,
                        index,
                        vertex_count: self.geometry.positions.len(),
                    });
                }
            }
        }
        let rows = self.geometry.triangles.iter().map(|triangle| {
            triangle.indices().map(|index| {
                let point = &self.geometry.positions[index];
                let normal = &normals[index];
                (
                    [point.x.clone(), point.y.clone(), point.z.clone()],
                    [
                        normal.0[0].clone(),
                        normal.0[1].clone(),
                        normal.0[2].clone(),
                    ],
                )
            })
        });
        hypermesh::ExactGpuMeshBuffers::from_triangles_with_capacity(
            self.geometry.triangles.len(),
            rows,
        )
    }
}

impl<M: Clone> AttributedMesh<M> {
    /// Clone one metadata value across all triangles without authored normals.
    ///
    /// Alignment follows from the triangle count; geometry is not validated and
    /// the new export cache starts empty. Allocation failure is not a typed result.
    pub fn from_uniform(geometry: TriangleMesh, metadata: M) -> Self {
        let face_metadata = vec![metadata; geometry.triangles.len()];
        Self {
            geometry,
            face_metadata,
            authored_normals: None,
            exact_gpu: Arc::new(OnceLock::new()),
        }
    }
}

/// Constructor witnesses for metadata alignment, independent of topology validation.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid;
    use hyperlattice::Real;

    /// Check uniform metadata count and preservation of native geometry buffers.
    #[test]
    fn attributes_are_strictly_aligned_to_native_geometry() {
        let geometry = solid::cube(Real::from(2_u8)).expect("valid primitive fixture");
        let attributed = AttributedMesh::from_uniform(geometry.clone(), "body");
        assert_eq!(attributed.geometry(), &geometry);
        assert_eq!(attributed.face_metadata().len(), geometry.triangles.len());
    }

    /// Check that missing metadata rows produce the alignment-specific error.
    #[test]
    fn rejects_misaligned_metadata() {
        let geometry = solid::cube(Real::from(2_u8)).expect("valid primitive fixture");
        let error = AttributedMesh::<()>::new(geometry, Vec::new()).unwrap_err();
        assert!(matches!(error, AttributeAlignmentError::FaceMetadata { .. }));
    }
}
