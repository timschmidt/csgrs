//! Exact voxel-surface triangle mesh handoff.
//!
//! This module is the exact mesh-vocabulary counterpart to the lossy OBJ/quad
//! preview adapters in [`crate::mesh`]. It keeps vertices as lattice vertices
//! and triangles as indexed combinatorial records over exact voxel faces. No
//! primitive-float coordinates are introduced here.
//!
//! Topology is replayable only from retained exact objects and certified
//! predicates. The indexed vertex/triangle vocabulary uses exact grid-lattice
//! coordinates rather than display vertices.

use crate::surface_topology::validate_surface_topology;
use crate::{
    ExactSurfaceFaceKey, ExactSurfaceVertex, ExactVoxelFace, HypervoxelResult, VoxelFaceSide,
};
use rustc_hash::FxHashMap;

/// One exact indexed triangle emitted from a voxel face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExactSurfaceTriangle {
    /// Indices into [`ExactVoxelSurfaceTriangleMesh::vertices`].
    pub vertices: [u32; 3],
    /// Exact voxel face whose quad was split.
    pub source_face: ExactSurfaceFaceKey,
    /// Split triangle ordinal for the source quad, either `0` or `1`.
    pub split: u8,
}

/// Exact indexed triangle mesh over voxel-surface lattice vertices.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactVoxelSurfaceTriangleMesh {
    /// Exact lattice vertices in deterministic order.
    pub vertices: Vec<ExactSurfaceVertex>,
    /// Exact indexed triangles, two per accepted voxel face.
    pub triangles: Vec<ExactSurfaceTriangle>,
}

/// Builds an exact indexed triangle mesh from audited voxel faces.
///
/// Each exact voxel quad is split deterministically into two indexed
/// triangles. The function first runs
/// [`audit_exact_voxel_surface_topology`]; if the face set is empty, mixed
/// depth, duplicate, open, degenerate, or nonmanifold, no triangles are
/// emitted and the report records the topology blockers. This keeps exact
/// mesh handoff separate from preview mesh repair or display triangulation.
pub fn exact_voxel_surface_triangle_mesh_from_faces(
    faces: &[ExactVoxelFace],
) -> HypervoxelResult<ExactVoxelSurfaceTriangleMesh> {
    let vertices = validate_surface_topology(faces)?
        .into_iter()
        .collect::<Vec<_>>();
    let vertex_index = vertices
        .iter()
        .enumerate()
        .map(|(index, vertex)| (*vertex, index as u32))
        .collect::<FxHashMap<_, _>>();
    let mut triangles = Vec::with_capacity(faces.len() * 2);
    for face in faces {
        let key = ExactSurfaceFaceKey {
            address: face.address,
            side: face.side,
        };
        let corners = face_lattice_vertices(face);
        let indices = corners.map(|vertex| vertex_index[&vertex]);
        triangles.push(ExactSurfaceTriangle {
            vertices: [indices[0], indices[1], indices[2]],
            source_face: key,
            split: 0,
        });
        triangles.push(ExactSurfaceTriangle {
            vertices: [indices[0], indices[2], indices[3]],
            source_face: key,
            split: 1,
        });
    }

    Ok(ExactVoxelSurfaceTriangleMesh {
        vertices,
        triangles,
    })
}

fn face_lattice_vertices(face: &ExactVoxelFace) -> [ExactSurfaceVertex; 4] {
    let depth = face.address.depth;
    let [x, y, z] = face.address.xyz;
    let vertex = |xyz| ExactSurfaceVertex { depth, xyz };
    match face.side {
        VoxelFaceSide::XNeg => [
            vertex([x, y, z]),
            vertex([x, y + 1, z]),
            vertex([x, y + 1, z + 1]),
            vertex([x, y, z + 1]),
        ],
        VoxelFaceSide::XPos => [
            vertex([x + 1, y, z]),
            vertex([x + 1, y, z + 1]),
            vertex([x + 1, y + 1, z + 1]),
            vertex([x + 1, y + 1, z]),
        ],
        VoxelFaceSide::YNeg => [
            vertex([x, y, z]),
            vertex([x, y, z + 1]),
            vertex([x + 1, y, z + 1]),
            vertex([x + 1, y, z]),
        ],
        VoxelFaceSide::YPos => [
            vertex([x, y + 1, z]),
            vertex([x + 1, y + 1, z]),
            vertex([x + 1, y + 1, z + 1]),
            vertex([x, y + 1, z + 1]),
        ],
        VoxelFaceSide::ZNeg => [
            vertex([x, y, z]),
            vertex([x + 1, y, z]),
            vertex([x + 1, y + 1, z]),
            vertex([x, y + 1, z]),
        ],
        VoxelFaceSide::ZPos => [
            vertex([x, y, z + 1]),
            vertex([x, y + 1, z + 1]),
            vertex([x + 1, y + 1, z + 1]),
            vertex([x + 1, y, z + 1]),
        ],
    }
}
