//! Explicit approximation adapters for GPU triangle buffers.
//!
//! Hypermesh keeps topology and native coordinates exact. This module owns the
//! deliberate boundary where exact render rows become finite `f32` or `f64`
//! vertex attributes and `u32` triangle-list indices suitable for graphics APIs.

use std::error::Error;
use std::fmt;

use hyperlattice::{Point3, Real, Vector3};

use crate::output::TriangleSoup;

/// One exact render vertex stored as `(position, normal)` rows.
pub type ExactGpuVertex = ([Real; 3], [Real; 3]);

/// Exact render rows with a GPU-compatible `u32` triangle-list index carrier.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExactGpuMeshBuffers {
    /// Exact position/normal vertex rows.
    pub vertices: Vec<ExactGpuVertex>,
    /// Flat triangle-list indices.
    pub indices: Vec<u32>,
}

impl ExactGpuMeshBuffers {
    /// Builds sequential exact vertex and index buffers from triangle rows.
    ///
    /// Vertices remain duplicated at triangle corners so authored flat-normal
    /// seams are preserved without a second position/normal index stream.
    pub fn from_triangles(
        triangles: impl IntoIterator<Item = [ExactGpuVertex; 3]>,
    ) -> Result<Self, GpuMeshError> {
        let triangles = triangles.into_iter();
        let (lower_bound, upper_bound) = triangles.size_hint();
        Self::from_triangle_iterator(upper_bound.unwrap_or(lower_bound), triangles)
    }

    /// Builds sequential exact buffers while reserving space for the expected
    /// number of triangle rows.
    pub fn from_triangles_with_capacity(
        triangle_capacity: usize,
        triangles: impl IntoIterator<Item = [ExactGpuVertex; 3]>,
    ) -> Result<Self, GpuMeshError> {
        Self::from_triangle_iterator(triangle_capacity, triangles.into_iter())
    }

    fn from_triangle_iterator(
        triangle_capacity: usize,
        triangles: impl Iterator<Item = [ExactGpuVertex; 3]>,
    ) -> Result<Self, GpuMeshError> {
        let vertex_capacity = triangle_capacity.saturating_mul(3);
        let mut vertices = Vec::with_capacity(vertex_capacity);
        let mut indices = Vec::with_capacity(vertex_capacity);

        for triangle in triangles {
            let base =
                u32::try_from(vertices.len()).map_err(|_| GpuMeshError::VertexCountExceededU32)?;
            let second = base
                .checked_add(1)
                .ok_or(GpuMeshError::VertexCountExceededU32)?;
            let third = base
                .checked_add(2)
                .ok_or(GpuMeshError::VertexCountExceededU32)?;
            vertices.extend(triangle);
            indices.extend([base, second, third]);
        }

        Ok(Self { vertices, indices })
    }

    /// Approximates every exact attribute as finite `f32` values.
    pub fn try_approximate_f32(&self) -> Result<GpuMeshBuffersF32, GpuMeshError> {
        approximate_gpu_mesh_f32(&self.vertices, &self.indices)
    }

    /// Approximates attributes as `f32`, replacing an unrepresentable position
    /// row or normal component with zero while still rejecting invalid indices.
    ///
    /// This compatibility policy matches renderers that require a total lossy
    /// export. Exact topology is never changed; only the affected position row
    /// or normal component receives the documented zero fallback.
    pub fn approximate_f32_or_zero(&self) -> Result<GpuMeshBuffersF32, GpuMeshError> {
        approximate_gpu_mesh_f32_or_zero(&self.vertices, &self.indices)
    }

    /// Approximates every exact attribute as finite `f64` values.
    pub fn try_approximate_f64(&self) -> Result<GpuMeshBuffersF64, GpuMeshError> {
        approximate_gpu_mesh_f64(&self.vertices, &self.indices)
    }

    /// Approximates attributes as `f64`, replacing an unrepresentable position
    /// row or normal component with zero while still rejecting invalid indices.
    pub fn approximate_f64_or_zero(&self) -> Result<GpuMeshBuffersF64, GpuMeshError> {
        approximate_gpu_mesh_f64_or_zero(&self.vertices, &self.indices)
    }
}

/// Backend-neutral binary32 GPU buffers using triangle-list topology.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GpuMeshBuffersF32 {
    /// Finite binary32 positions.
    pub positions: Vec<[f32; 3]>,
    /// Finite binary32 normals.
    pub normals: Vec<[f32; 3]>,
    /// Flat `u32` triangle-list indices.
    pub indices: Vec<u32>,
}

/// Backend-neutral binary64 GPU buffers using triangle-list topology.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GpuMeshBuffersF64 {
    /// Finite binary64 positions.
    pub positions: Vec<[f64; 3]>,
    /// Finite binary64 normals.
    pub normals: Vec<[f64; 3]>,
    /// Flat `u32` triangle-list indices.
    pub indices: Vec<u32>,
}

/// Vertex attribute selected while crossing the GPU approximation boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GpuVertexAttribute {
    /// Position coordinate.
    Position,
    /// Normal coordinate.
    Normal,
}

/// Failure while constructing or approximating GPU mesh buffers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GpuMeshError {
    /// The exact vertex stream cannot be represented by `u32` indices.
    VertexCountExceededU32,
    /// The flat index stream does not contain complete triangles.
    IndexCountNotTriangleList {
        /// Number of entries in the flat index stream.
        index_count: usize,
    },
    /// A supplied index does not address the supplied vertex stream.
    IndexOutOfBounds {
        /// Offset in the flat index stream.
        index_offset: usize,
        /// Invalid vertex index.
        index: u32,
        /// Number of available vertices.
        vertex_count: usize,
    },
    /// A triangle soup index does not address its exact vertex stream.
    SourceTriangleIndexOutOfBounds {
        /// Offset of the triangle in the soup.
        triangle: usize,
        /// Corner within the triangle.
        corner: usize,
        /// Invalid source vertex index.
        index: usize,
        /// Number of available source vertices.
        vertex_count: usize,
    },
    /// An exact attribute could not be approximated as a finite floating-point row.
    AttributeApproximationFailed {
        /// Vertex containing the attribute.
        vertex: usize,
        /// Attribute that failed conversion.
        attribute: GpuVertexAttribute,
    },
}

impl fmt::Display for GpuMeshError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::VertexCountExceededU32 => {
                formatter.write_str("GPU vertex count exceeds the u32 index range")
            }
            Self::IndexCountNotTriangleList { index_count } => write!(
                formatter,
                "GPU triangle-list index stream contains {index_count} entries, which is not divisible by three"
            ),
            Self::IndexOutOfBounds {
                index_offset,
                index,
                vertex_count,
            } => write!(
                formatter,
                "GPU index {index} at offset {index_offset} does not address any of {vertex_count} vertices"
            ),
            Self::SourceTriangleIndexOutOfBounds {
                triangle,
                corner,
                index,
                vertex_count,
            } => write!(
                formatter,
                "triangle {triangle} corner {corner} references source vertex {index}, but only {vertex_count} vertices exist"
            ),
            Self::AttributeApproximationFailed { vertex, attribute } => {
                write!(
                    formatter,
                    "GPU {attribute:?} row for vertex {vertex} is not representable as the requested finite floating-point type"
                )
            }
        }
    }
}

impl Error for GpuMeshError {}

/// Approximates exact render rows into backend-neutral GPU buffers.
pub fn approximate_gpu_mesh_f32(
    vertices: &[ExactGpuVertex],
    indices: &[u32],
) -> Result<GpuMeshBuffersF32, GpuMeshError> {
    validate_indices(vertices.len(), indices)?;
    let (positions, normals) = try_approximate_rows(vertices, Real::to_f32_lossy)?;
    Ok(GpuMeshBuffersF32 {
        positions,
        normals,
        indices: indices.to_vec(),
    })
}

/// Approximates exact render rows and substitutes zero for an unrepresentable
/// position row or normal component.
pub fn approximate_gpu_mesh_f32_or_zero(
    vertices: &[ExactGpuVertex],
    indices: &[u32],
) -> Result<GpuMeshBuffersF32, GpuMeshError> {
    validate_indices(vertices.len(), indices)?;
    let (positions, normals) = approximate_rows_or_zero(vertices, 0.0_f32, Real::to_f32_lossy);
    Ok(GpuMeshBuffersF32 {
        positions,
        normals,
        indices: indices.to_vec(),
    })
}

/// Approximates exact render rows into backend-neutral binary64 GPU buffers.
pub fn approximate_gpu_mesh_f64(
    vertices: &[ExactGpuVertex],
    indices: &[u32],
) -> Result<GpuMeshBuffersF64, GpuMeshError> {
    validate_indices(vertices.len(), indices)?;
    let (positions, normals) = try_approximate_rows(vertices, Real::to_f64_lossy)?;
    Ok(GpuMeshBuffersF64 {
        positions,
        normals,
        indices: indices.to_vec(),
    })
}

/// Approximates exact render rows as binary64 values and substitutes zero for
/// an unrepresentable position row or normal component.
pub fn approximate_gpu_mesh_f64_or_zero(
    vertices: &[ExactGpuVertex],
    indices: &[u32],
) -> Result<GpuMeshBuffersF64, GpuMeshError> {
    validate_indices(vertices.len(), indices)?;
    let (positions, normals) = approximate_rows_or_zero(vertices, 0.0_f64, Real::to_f64_lossy);
    Ok(GpuMeshBuffersF64 {
        positions,
        normals,
        indices: indices.to_vec(),
    })
}

impl TriangleSoup {
    /// Builds exact flat-shaded render rows and `u32` indices.
    pub fn to_exact_gpu_mesh_buffers(&self) -> Result<ExactGpuMeshBuffers, GpuMeshError> {
        for (triangle_offset, triangle) in self.triangles.iter().enumerate() {
            for (corner, &index) in triangle.iter().enumerate() {
                if index >= self.vertices.len() {
                    return Err(GpuMeshError::SourceTriangleIndexOutOfBounds {
                        triangle: triangle_offset,
                        corner,
                        index,
                        vertex_count: self.vertices.len(),
                    });
                }
            }
        }

        let triangles = self.triangles.iter().map(|triangle| {
            let [a, b, c] = triangle.map(|index| {
                let vertex = &self.vertices[index];
                Point3::new(vertex.x.clone(), vertex.y.clone(), vertex.z.clone())
            });
            let normal = (&b - &a)
                .unit_cross_checked(&(&c - &a))
                .unwrap_or_else(|_| Vector3::z());
            let normal = [
                normal.0[0].clone(),
                normal.0[1].clone(),
                normal.0[2].clone(),
            ];
            [
                (point_row(a), normal.clone()),
                (point_row(b), normal.clone()),
                (point_row(c), normal),
            ]
        });
        ExactGpuMeshBuffers::from_triangles_with_capacity(self.triangles.len(), triangles)
    }

    /// Produces strict finite-`f32` GPU buffers from this exact triangle soup.
    pub fn try_to_gpu_mesh_f32(&self) -> Result<GpuMeshBuffersF32, GpuMeshError> {
        self.to_exact_gpu_mesh_buffers()?.try_approximate_f32()
    }

    /// Produces finite-`f32` GPU buffers, using zero for an unrepresentable
    /// position row or normal component.
    pub fn to_gpu_mesh_f32_or_zero(&self) -> Result<GpuMeshBuffersF32, GpuMeshError> {
        self.to_exact_gpu_mesh_buffers()?.approximate_f32_or_zero()
    }

    /// Produces strict finite-`f64` GPU buffers from this exact triangle soup.
    pub fn try_to_gpu_mesh_f64(&self) -> Result<GpuMeshBuffersF64, GpuMeshError> {
        self.to_exact_gpu_mesh_buffers()?.try_approximate_f64()
    }

    /// Produces finite-`f64` GPU buffers, using zero for an unrepresentable
    /// position row or normal component.
    pub fn to_gpu_mesh_f64_or_zero(&self) -> Result<GpuMeshBuffersF64, GpuMeshError> {
        self.to_exact_gpu_mesh_buffers()?.approximate_f64_or_zero()
    }
}

fn point_row(point: Point3) -> [Real; 3] {
    [point.x, point.y, point.z]
}

fn try_approximate_row<T: Copy>(
    row: &[Real; 3],
    approximate: impl Fn(&Real) -> Option<T>,
) -> Option<[T; 3]> {
    Some([
        approximate(&row[0])?,
        approximate(&row[1])?,
        approximate(&row[2])?,
    ])
}

fn try_approximate_rows<T: Copy>(
    vertices: &[ExactGpuVertex],
    approximate: impl Fn(&Real) -> Option<T> + Copy,
) -> Result<(Vec<[T; 3]>, Vec<[T; 3]>), GpuMeshError> {
    let mut positions = Vec::with_capacity(vertices.len());
    let mut normals = Vec::with_capacity(vertices.len());

    for (vertex, (position, normal)) in vertices.iter().enumerate() {
        positions.push(try_approximate_row(position, approximate).ok_or(
            GpuMeshError::AttributeApproximationFailed {
                vertex,
                attribute: GpuVertexAttribute::Position,
            },
        )?);
        normals.push(try_approximate_row(normal, approximate).ok_or(
            GpuMeshError::AttributeApproximationFailed {
                vertex,
                attribute: GpuVertexAttribute::Normal,
            },
        )?);
    }

    Ok((positions, normals))
}

fn approximate_rows_or_zero<T: Copy>(
    vertices: &[ExactGpuVertex],
    zero: T,
    approximate: impl Fn(&Real) -> Option<T> + Copy,
) -> (Vec<[T; 3]>, Vec<[T; 3]>) {
    let mut positions = Vec::with_capacity(vertices.len());
    let mut normals = Vec::with_capacity(vertices.len());

    for (position, normal) in vertices {
        positions.push(try_approximate_row(position, approximate).unwrap_or([zero; 3]));
        normals.push(
            normal
                .each_ref()
                .map(|value| approximate(value).unwrap_or(zero)),
        );
    }

    (positions, normals)
}

fn validate_indices(vertex_count: usize, indices: &[u32]) -> Result<(), GpuMeshError> {
    if !indices.len().is_multiple_of(3) {
        return Err(GpuMeshError::IndexCountNotTriangleList {
            index_count: indices.len(),
        });
    }
    for (index_offset, &index) in indices.iter().enumerate() {
        if usize::try_from(index).map_or(true, |index| index >= vertex_count) {
            return Err(GpuMeshError::IndexOutOfBounds {
                index_offset,
                index,
                vertex_count,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::OutputVertex;

    fn exact_vertex(position: [i64; 3], normal: [i64; 3]) -> ExactGpuVertex {
        (position.map(Real::from), normal.map(Real::from))
    }

    #[test]
    fn exact_triangle_rows_approximate_to_gpu_buffers() {
        let exact = ExactGpuMeshBuffers::from_triangles([[
            exact_vertex([0, 0, 0], [0, 0, 1]),
            exact_vertex([2, 0, 0], [0, 0, 1]),
            exact_vertex([0, 2, 0], [0, 0, 1]),
        ]])
        .unwrap();

        let gpu = exact.try_approximate_f32().unwrap();
        let gpu_f64 = exact.try_approximate_f64().unwrap();
        assert_eq!(
            gpu.positions,
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        );
        assert_eq!(gpu.normals, [[0.0, 0.0, 1.0]; 3]);
        assert_eq!(gpu.indices, [0, 1, 2]);
        assert_eq!(
            gpu_f64.positions,
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        );
        assert_eq!(gpu_f64.normals, [[0.0, 0.0, 1.0]; 3]);
        assert_eq!(gpu_f64.indices, gpu.indices);
    }

    #[test]
    fn approximation_rejects_an_invalid_index() {
        let vertices = [exact_vertex([0, 0, 0], [0, 0, 1])];
        assert_eq!(
            approximate_gpu_mesh_f32(&vertices, &[0, 0, 1]),
            Err(GpuMeshError::IndexOutOfBounds {
                index_offset: 2,
                index: 1,
                vertex_count: 1,
            })
        );
    }

    #[test]
    fn approximation_rejects_an_incomplete_triangle_list() {
        let vertices = [exact_vertex([0, 0, 0], [0, 0, 1])];
        assert_eq!(
            approximate_gpu_mesh_f32(&vertices, &[0]),
            Err(GpuMeshError::IndexCountNotTriangleList { index_count: 1 })
        );
    }

    #[test]
    fn strict_and_zero_fallback_policies_are_explicit() {
        let mut huge = Real::from(2);
        for _ in 0..8 {
            huge = huge.clone() * huge;
        }
        let vertices = [
            (
                [huge.clone(), 1.into(), 2.into()],
                [0.into(), 0.into(), 1.into()],
            ),
            ([0.into(), 0.into(), 0.into()], [huge, 1.into(), 0.into()]),
            exact_vertex([0, 0, 0], [0, 0, 1]),
        ];
        let indices = [0, 1, 2];

        assert_eq!(
            approximate_gpu_mesh_f32(&vertices, &indices),
            Err(GpuMeshError::AttributeApproximationFailed {
                vertex: 0,
                attribute: GpuVertexAttribute::Position,
            })
        );
        let fallback = approximate_gpu_mesh_f32_or_zero(&vertices, &indices).unwrap();
        assert_eq!(fallback.positions[0], [0.0; 3]);
        assert_eq!(fallback.normals[1], [0.0, 1.0, 0.0]);
        assert_eq!(fallback.indices, indices);

        let binary64 = approximate_gpu_mesh_f64(&vertices, &indices).unwrap();
        assert!(binary64.positions[0][0] > f64::from(f32::MAX));
        assert_eq!(binary64.positions[0][1..], [1.0, 2.0]);
        assert_eq!(binary64.normals[1][1..], [1.0, 0.0]);

        let mut binary64_overflow = vertices.clone();
        let mut too_huge = binary64_overflow[0].0[0].clone();
        too_huge = too_huge.clone() * too_huge;
        too_huge = too_huge.clone() * too_huge;
        binary64_overflow[0].0[0] = too_huge;
        assert_eq!(
            approximate_gpu_mesh_f64(&binary64_overflow, &indices),
            Err(GpuMeshError::AttributeApproximationFailed {
                vertex: 0,
                attribute: GpuVertexAttribute::Position,
            })
        );
        let fallback64 = approximate_gpu_mesh_f64_or_zero(&binary64_overflow, &indices).unwrap();
        assert_eq!(fallback64.positions[0], [0.0; 3]);
    }

    #[test]
    fn triangle_soup_rejects_an_invalid_source_index() {
        let soup = TriangleSoup {
            vertices: vec![OutputVertex {
                x: 0.into(),
                y: 0.into(),
                z: 0.into(),
            }],
            triangles: vec![[0, 4, 0]],
            sources: Vec::new(),
        };

        assert_eq!(
            soup.to_exact_gpu_mesh_buffers(),
            Err(GpuMeshError::SourceTriangleIndexOutOfBounds {
                triangle: 0,
                corner: 1,
                index: 4,
                vertex_count: 1,
            })
        );
    }

    #[test]
    fn triangle_soup_exports_flat_shaded_gpu_rows() {
        let soup = TriangleSoup {
            vertices: vec![
                OutputVertex {
                    x: 0.into(),
                    y: 0.into(),
                    z: 0.into(),
                },
                OutputVertex {
                    x: 2.into(),
                    y: 0.into(),
                    z: 0.into(),
                },
                OutputVertex {
                    x: 0.into(),
                    y: 2.into(),
                    z: 0.into(),
                },
            ],
            triangles: vec![[0, 1, 2]],
            sources: Vec::new(),
        };

        let gpu = soup.try_to_gpu_mesh_f32().unwrap();
        let gpu_f64 = soup.try_to_gpu_mesh_f64().unwrap();
        assert_eq!(
            gpu.positions,
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        );
        assert_eq!(gpu.normals, [[0.0, 0.0, 1.0]; 3]);
        assert_eq!(gpu.indices, [0, 1, 2]);
        assert_eq!(
            gpu_f64.positions,
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        );
        assert_eq!(gpu_f64.normals, [[0.0, 0.0, 1.0]; 3]);
        assert_eq!(gpu_f64.indices, gpu.indices);
    }
}
