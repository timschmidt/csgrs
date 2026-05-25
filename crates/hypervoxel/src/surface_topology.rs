//! Exact voxel-surface topology vocabulary.
//!
//! Exposed voxel faces are already exact grid facts. This module adds the
//! next shared mesh vocabulary layer without lowering to primitive floats:
//! exact lattice vertices, exact lattice edges, and face-incidence reports.
//! That gives downstream mesh/part crates a report-bearing handoff point before
//! any OBJ/glTF/renderer adapter runs.
//!
//! The design follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: topology is accepted from retained
//! combinatorial objects and certified predicates, not from approximate mesh
//! coordinates. The vertex/edge/face incidence vocabulary matches the
//! combinatorial mesh model used in Botsch et al., *Polygon Mesh Processing*,
//! AK Peters, 2010, but every coordinate here is an integer grid coordinate at
//! an explicit voxel depth.

use std::collections::{BTreeMap, BTreeSet};

use crate::{ExactVoxelFace, VoxelAddress, VoxelFaceSide};

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

/// Exact vertex/edge/face incidence audit for voxel-surface faces.
///
/// A non-empty shell is topology-ready only when every audited face is at the
/// same lattice depth, no face identity appears twice, every face has four
/// non-degenerate lattice edges, and every edge is incident to exactly two
/// faces. Boundary and nonmanifold edges remain explicit blockers instead of
/// being patched by a display mesher.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactVoxelSurfaceTopologyReport {
    /// Number of input exact faces.
    pub input_faces: usize,
    /// Number of faces audited at the common lattice depth.
    pub audited_faces: usize,
    /// Common lattice depth used for incidence, when at least one face exists.
    pub common_depth: Option<u8>,
    /// Number of faces skipped because their depth differs from the common
    /// incidence depth.
    pub mixed_depth_faces: usize,
    /// Unique face identities.
    pub unique_faces: usize,
    /// Duplicate face identities encountered after their first occurrence.
    pub duplicate_faces: Vec<ExactSurfaceFaceKey>,
    /// Unique exact lattice vertices.
    pub vertices: BTreeSet<ExactSurfaceVertex>,
    /// Unique exact lattice edges.
    pub edges: BTreeSet<ExactSurfaceEdge>,
    /// Number of face-edge records audited.
    pub face_edge_records: usize,
    /// Faces whose four lattice edges were not all non-degenerate.
    pub degenerate_faces: Vec<ExactSurfaceFaceKey>,
    /// Edges incident to exactly one audited face.
    pub boundary_edges: Vec<ExactSurfaceEdge>,
    /// Edges incident to exactly two audited faces.
    pub manifold_edges: usize,
    /// Edges incident to more than two audited faces, with incidence counts.
    pub nonmanifold_edges: Vec<(ExactSurfaceEdge, usize)>,
    /// Whether this face set is non-empty exact closed surface-topology
    /// evidence.
    pub exact_surface_topology_ready: bool,
}

/// Audits exact voxel faces as a combinatorial vertex/edge/face surface.
///
/// This function intentionally does not merge coplanar faces or lower vertices
/// to metric coordinates. Greedy patches and lossy display meshes can be built
/// later, but this report is the exact mesh-vocabulary boundary: face identity,
/// edge incidence, duplicate faces, mixed-depth blockers, and open/nonmanifold
/// edges are all explicit.
pub fn audit_exact_voxel_surface_topology(
    faces: &[ExactVoxelFace],
) -> ExactVoxelSurfaceTopologyReport {
    let common_depth = faces.first().map(|face| face.address.depth);
    let mut seen_faces = BTreeSet::new();
    let mut duplicate_faces = Vec::new();
    let mut vertices = BTreeSet::new();
    let mut edge_incidence = BTreeMap::<ExactSurfaceEdge, usize>::new();
    let mut face_edge_records = 0_usize;
    let mut degenerate_faces = Vec::new();
    let mut mixed_depth_faces = 0_usize;
    let mut audited_faces = 0_usize;

    for face in faces {
        let key = ExactSurfaceFaceKey {
            address: face.address,
            side: face.side,
        };
        if !seen_faces.insert(key) {
            duplicate_faces.push(key);
        }

        if Some(face.address.depth) != common_depth {
            mixed_depth_faces += 1;
            continue;
        }

        audited_faces += 1;
        let corners = face_lattice_vertices(face);
        for vertex in corners {
            vertices.insert(vertex);
        }

        let edges = [
            ExactSurfaceEdge::new(corners[0], corners[1]),
            ExactSurfaceEdge::new(corners[1], corners[2]),
            ExactSurfaceEdge::new(corners[2], corners[3]),
            ExactSurfaceEdge::new(corners[3], corners[0]),
        ];
        if edges.iter().any(ExactSurfaceEdge::is_degenerate) {
            degenerate_faces.push(key);
            continue;
        }
        for edge in edges {
            face_edge_records += 1;
            *edge_incidence.entry(edge).or_insert(0) += 1;
        }
    }

    let mut boundary_edges = Vec::new();
    let mut manifold_edges = 0_usize;
    let mut nonmanifold_edges = Vec::new();
    for (edge, count) in &edge_incidence {
        match *count {
            0 => unreachable!("edge incidence map never stores zero counts"),
            1 => boundary_edges.push(*edge),
            2 => manifold_edges += 1,
            count => nonmanifold_edges.push((*edge, count)),
        }
    }
    let edges = edge_incidence.keys().copied().collect::<BTreeSet<_>>();
    let exact_surface_topology_ready = !faces.is_empty()
        && mixed_depth_faces == 0
        && duplicate_faces.is_empty()
        && degenerate_faces.is_empty()
        && boundary_edges.is_empty()
        && nonmanifold_edges.is_empty()
        && audited_faces == faces.len();

    ExactVoxelSurfaceTopologyReport {
        input_faces: faces.len(),
        audited_faces,
        common_depth,
        mixed_depth_faces,
        unique_faces: seen_faces.len(),
        duplicate_faces,
        vertices,
        edges,
        face_edge_records,
        degenerate_faces,
        boundary_edges,
        manifold_edges,
        nonmanifold_edges,
        exact_surface_topology_ready,
    }
}

impl ExactSurfaceEdge {
    fn new(a: ExactSurfaceVertex, b: ExactSurfaceVertex) -> Self {
        if a <= b {
            Self { a, b }
        } else {
            Self { a: b, b: a }
        }
    }

    fn is_degenerate(&self) -> bool {
        self.a == self.b
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
