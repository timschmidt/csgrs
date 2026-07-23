//! Exact voxel-surface topology vocabulary.

use std::collections::{BTreeMap, BTreeSet};

use crate::{ExactVoxelFace, HypervoxelError, HypervoxelResult, VoxelAddress, VoxelFaceSide};

/// Exact lattice vertex of a voxel surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ExactSurfaceVertex {
    /// Voxel-address depth of the lattice coordinate.
    pub depth: u8,
    /// Integer vertex coordinate at `depth`.
    pub xyz: [u64; 3],
}

/// Exact undirected lattice edge of a voxel surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ExactSurfaceEdge {
    /// Lower endpoint in deterministic order.
    pub a: ExactSurfaceVertex,
    /// Upper endpoint in deterministic order.
    pub b: ExactSurfaceVertex,
}

/// Stable identity for an exact voxel face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ExactSurfaceFaceKey {
    /// Source cell address.
    pub address: VoxelAddress,
    /// Exposed side.
    pub side: VoxelFaceSide,
}

pub(crate) fn validate_surface_topology(
    faces: &[ExactVoxelFace],
) -> HypervoxelResult<BTreeSet<ExactSurfaceVertex>> {
    let Some(common_depth) = faces.first().map(|face| face.address.depth) else {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "surface contains no faces",
        });
    };
    let mut seen_faces = BTreeSet::new();
    let mut vertices = BTreeSet::new();
    let mut edge_incidence = BTreeMap::<ExactSurfaceEdge, usize>::new();

    for face in faces {
        if face.address.depth != common_depth {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "surface contains mixed-depth faces",
            });
        }
        let key = ExactSurfaceFaceKey {
            address: face.address,
            side: face.side,
        };
        if !seen_faces.insert(key) {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "surface contains duplicate faces",
            });
        }

        let corners = face_lattice_vertices(face);
        vertices.extend(corners);
        let edges = [
            ExactSurfaceEdge::new(corners[0], corners[1]),
            ExactSurfaceEdge::new(corners[1], corners[2]),
            ExactSurfaceEdge::new(corners[2], corners[3]),
            ExactSurfaceEdge::new(corners[3], corners[0]),
        ];
        if edges.iter().any(|edge| edge.a == edge.b) {
            return Err(HypervoxelError::InvalidSourceGeometry {
                reason: "surface contains a degenerate face",
            });
        }
        for edge in edges {
            *edge_incidence.entry(edge).or_insert(0) += 1;
        }
    }

    if edge_incidence.values().any(|count| *count != 2) {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "surface is open or nonmanifold",
        });
    }
    Ok(vertices)
}

impl ExactSurfaceEdge {
    fn new(a: ExactSurfaceVertex, b: ExactSurfaceVertex) -> Self {
        if a <= b {
            Self { a, b }
        } else {
            Self { a: b, b: a }
        }
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
