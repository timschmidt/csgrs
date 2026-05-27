//! Exact coordinate recovery for boolmesh `boolean45` output vertices.
//!
//! Legacy boolmesh duplicates primitive `Vec3` coordinates into `ps_r` while
//! sizing and emitting output topology.  The exact port keeps the boolmesh
//! allocation order but recovers coordinates from retained source or
//! `kernel12` construction evidence only at triangulation/export boundaries.
//! This is the Yap boundary from "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): topology ids and exact numeric
//! objects are connected by replayable provenance, not by tolerance welding.

use hyperlimit::Point3;

use crate::exact::mesh::ExactMesh;

use super::super::{
    ExactBoolMeshBoolean03, ExactBoolMeshOutputVertexAllocation, ExactBoolMeshOutputVertexOrigin,
    ExactBoolMeshSide, ExactBoolMeshSourceVertex,
};

/// Recover the exact point for one boolmesh output vertex id.
pub(super) fn output_vertex_point(
    vertex: usize,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    boolean03: &ExactBoolMeshBoolean03,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Point3> {
    match allocation.output_vertex_origins.get(vertex)? {
        ExactBoolMeshOutputVertexOrigin::SourceVertex { source, .. } => {
            source_vertex_point(*source, left, right)
        }
        ExactBoolMeshOutputVertexOrigin::Kernel12LeftEdgeRightFace { event, .. } => {
            boolean03.v12.get(*event).cloned()
        }
        ExactBoolMeshOutputVertexOrigin::Kernel12RightEdgeLeftFace { event, .. } => {
            boolean03.v21.get(*event).cloned()
        }
    }
}

/// Return whether an output-vertex origin can be resolved from retained counts.
pub(super) fn output_vertex_origin_has_coordinate(
    origin: ExactBoolMeshOutputVertexOrigin,
    boolean03: &ExactBoolMeshBoolean03,
    left_vertices: usize,
    right_vertices: usize,
) -> bool {
    match origin {
        ExactBoolMeshOutputVertexOrigin::SourceVertex { source, .. } => match source.side {
            ExactBoolMeshSide::Left => source.vertex < left_vertices,
            ExactBoolMeshSide::Right => source.vertex < right_vertices,
        },
        ExactBoolMeshOutputVertexOrigin::Kernel12LeftEdgeRightFace { event, .. } => {
            event < boolean03.v12.len()
        }
        ExactBoolMeshOutputVertexOrigin::Kernel12RightEdgeLeftFace { event, .. } => {
            event < boolean03.v21.len()
        }
    }
}

fn source_vertex_point(
    source: ExactBoolMeshSourceVertex,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Point3> {
    let mesh = match source.side {
        ExactBoolMeshSide::Left => left,
        ExactBoolMeshSide::Right => right,
    };
    mesh.vertices()
        .get(source.vertex)
        .map(|point| point.to_hyperlimit_point())
}
