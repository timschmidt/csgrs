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

use std::collections::BTreeMap;

use crate::{
    ExactSurfaceFaceKey, ExactSurfaceVertex, ExactVoxelFace, ExactVoxelSurfaceTopologyReport,
    VoxelFaceSide, audit_exact_voxel_surface_topology,
};

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
    /// Report tying the mesh back to the audited exact face set.
    pub report: ExactVoxelSurfaceTriangleMeshReport,
}

/// Report for exact voxel-surface triangle handoff.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactVoxelSurfaceTriangleMeshReport {
    /// Topology audit that gates exact mesh emission.
    pub topology: ExactVoxelSurfaceTopologyReport,
    /// Number of input faces offered to the handoff.
    pub input_faces: usize,
    /// Number of exact lattice vertices emitted.
    pub exact_vertices: usize,
    /// Number of exact indexed triangles emitted.
    pub exact_triangles: usize,
    /// Number of face-to-triangle records emitted.
    pub face_triangle_records: usize,
    /// Whether every emitted triangle retains its source voxel-face identity.
    pub exact_face_identity_preserved: bool,
    /// Whether this mesh can be consumed as exact surface-triangle vocabulary.
    pub exact_triangle_surface_mesh_ready: bool,
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
) -> ExactVoxelSurfaceTriangleMesh {
    let topology = audit_exact_voxel_surface_topology(faces);
    if !topology.exact_surface_topology_ready {
        return ExactVoxelSurfaceTriangleMesh {
            vertices: Vec::new(),
            triangles: Vec::new(),
            report: ExactVoxelSurfaceTriangleMeshReport {
                input_faces: faces.len(),
                topology,
                exact_vertices: 0,
                exact_triangles: 0,
                face_triangle_records: 0,
                exact_face_identity_preserved: false,
                exact_triangle_surface_mesh_ready: false,
            },
        };
    }

    let vertices = topology.vertices.iter().copied().collect::<Vec<_>>();
    let vertex_index = vertices
        .iter()
        .enumerate()
        .map(|(index, vertex)| (*vertex, index as u32))
        .collect::<BTreeMap<_, _>>();
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

    let exact_face_identity_preserved = triangles.len() == faces.len() * 2
        && triangles.chunks_exact(2).zip(faces).all(|(pair, face)| {
            let key = ExactSurfaceFaceKey {
                address: face.address,
                side: face.side,
            };
            pair[0].source_face == key
                && pair[1].source_face == key
                && pair[0].split == 0
                && pair[1].split == 1
        });
    let exact_triangle_surface_mesh_ready = topology.exact_surface_topology_ready
        && !vertices.is_empty()
        && triangles.len() == faces.len() * 2
        && exact_face_identity_preserved;
    ExactVoxelSurfaceTriangleMesh {
        report: ExactVoxelSurfaceTriangleMeshReport {
            input_faces: faces.len(),
            exact_vertices: vertices.len(),
            exact_triangles: triangles.len(),
            face_triangle_records: triangles.len(),
            exact_face_identity_preserved,
            exact_triangle_surface_mesh_ready,
            topology,
        },
        vertices,
        triangles,
    }
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
