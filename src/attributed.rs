//! Optional face attributes attached to native triangle geometry.
//!
//! [`AttributedMesh`] is deliberately a boundary carrier, not a second mesh
//! implementation. Geometry remains a [`TriangleMesh`]; metadata and authored
//! normals are parallel arrays whose alignment is checked at construction.

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
#[derive(Clone, Debug)]
pub struct AttributedMesh<M> {
    geometry: TriangleMesh,
    face_metadata: Vec<M>,
    authored_normals: Option<Vec<Vector3>>,
    exact_gpu: Arc<OnceLock<Result<hypermesh::ExactGpuMeshBuffers, hypermesh::GpuMeshError>>>,
}

impl<M: PartialEq> PartialEq for AttributedMesh<M> {
    fn eq(&self, other: &Self) -> bool {
        self.geometry == other.geometry
            && self.face_metadata == other.face_metadata
            && self.authored_normals == other.authored_normals
    }
}

impl<M> AttributedMesh<M> {
    /// Attaches one metadata row per native triangle.
    pub fn new(
        geometry: TriangleMesh,
        face_metadata: Vec<M>,
    ) -> Result<Self, AttributeAlignmentError> {
        Self::with_authored_normals(geometry, face_metadata, None)
    }

    /// Attaches face metadata and, when supplied, one normal per position.
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

    /// Borrows authored normals in native position order.
    pub fn authored_normals(&self) -> Option<&[Vector3]> {
        self.authored_normals.as_deref()
    }

    /// Consumes the sidecar and returns all aligned components.
    pub fn into_parts(self) -> (TriangleMesh, Vec<M>, Option<Vec<Vector3>>) {
        (self.geometry, self.face_metadata, self.authored_normals)
    }

    /// Replaces each face metadata value without touching native geometry.
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
    pub fn exact_gpu_mesh_buffers(
        &self,
    ) -> Result<&hypermesh::ExactGpuMeshBuffers, &hypermesh::GpuMeshError> {
        self.exact_gpu
            .get_or_init(|| {
                let Some(normals) = &self.authored_normals else {
                    return self
                        .geometry
                        .exact_gpu_mesh_buffers()
                        .cloned()
                        .map_err(Clone::clone);
                };
                for (triangle_offset, triangle) in self.geometry.triangles.iter().enumerate() {
                    for (corner, index) in triangle.indices().into_iter().enumerate() {
                        if index >= self.geometry.positions.len() {
                            return Err(
                                hypermesh::GpuMeshError::SourceTriangleIndexOutOfBounds {
                                    triangle: triangle_offset,
                                    corner,
                                    index,
                                    vertex_count: self.geometry.positions.len(),
                                },
                            );
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
            })
            .as_ref()
    }
}

impl<M: Clone> AttributedMesh<M> {
    /// Attaches the same metadata value to every native triangle.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid;
    use hyperlattice::Real;

    #[test]
    fn attributes_are_strictly_aligned_to_native_geometry() {
        let geometry = solid::cube(Real::from(2_u8));
        let attributed = AttributedMesh::from_uniform(geometry.clone(), "body");
        assert_eq!(attributed.geometry(), &geometry);
        assert_eq!(attributed.face_metadata().len(), geometry.triangles.len());
    }

    #[test]
    fn rejects_misaligned_metadata() {
        let geometry = solid::cube(Real::from(2_u8));
        let error = AttributedMesh::<()>::new(geometry, Vec::new()).unwrap_err();
        assert!(matches!(error, AttributeAlignmentError::FaceMetadata { .. }));
    }
}
