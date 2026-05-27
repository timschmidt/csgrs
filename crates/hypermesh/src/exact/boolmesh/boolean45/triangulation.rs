//! Exact triangulation-prep for boolmesh `boolean45` face loops.
//!
//! Legacy boolmesh assembles halfedge loops, then triangulates each output face
//! boundary.  This module ports the first executable part of that handoff:
//! simple single-loop faces are projected with exact source-face evidence and
//! sent to `hypertri` earcut.  Multi-loop faces are retained as explicit
//! blockers for the later constrained-triangulation slice instead of being
//! flattened by a tolerance heuristic.  That separation follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
//! The simple-polygon triangulation step consumes `hypertri`'s exact earcut
//! port, whose algorithmic basis is Meisters, "Polygons Have Ears," *The
//! American Mathematical Monthly* 82.6 (1975).

use std::collections::BTreeMap;

use hyperlimit::Point3;

use crate::exact::mesh::ExactMesh;
use crate::exact::region::{choose_region_projection, project_for_hypertri};

use super::super::{
    ExactBoolMeshBoolean03, ExactBoolMeshFaceLoopAssemblyStage, ExactBoolMeshHalfedgeAssemblyStage,
    ExactBoolMeshLoopTriangulation, ExactBoolMeshLoopTriangulationStage,
    ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshOutputVertexAllocation, ExactBoolMeshSide,
};
use super::geometry::output_vertex_point;

/// Triangulate simple assembled boolmesh output loops.
pub(super) fn triangulate_output_face_loops(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_loops: &ExactBoolMeshFaceLoopAssemblyStage,
) -> ExactBoolMeshLoopTriangulationStage {
    let mut stage = ExactBoolMeshLoopTriangulationStage::default();
    let mut loops_by_face = BTreeMap::<usize, Vec<usize>>::new();
    for (loop_index, face_loop) in face_loops.loops.iter().enumerate() {
        loops_by_face
            .entry(face_loop.output_face)
            .or_default()
            .push(loop_index);
    }

    for loop_indices in loops_by_face.into_values() {
        if loop_indices.len() > 1 {
            stage.multi_loop_faces += 1;
            continue;
        }
        let loop_index = loop_indices[0];
        let face_loop = &face_loops.loops[loop_index];
        if face_loop.vertices.len() < 3 {
            stage.short_loops += 1;
            continue;
        }

        let Some((source_side, source_face)) =
            loop_source_face(face_loop.halfedges.first().copied(), halfedges)
        else {
            stage.missing_source_faces += 1;
            continue;
        };
        let Some(source_mesh) = source_mesh(source_side, source_face, left, right) else {
            stage.missing_source_faces += 1;
            continue;
        };
        let Ok(projection) = choose_region_projection(source_mesh, source_face) else {
            stage.missing_source_faces += 1;
            continue;
        };

        let Some(points) =
            output_loop_points(&face_loop.vertices, allocation, boolean03, left, right)
        else {
            stage.missing_vertex_coordinates += 1;
            continue;
        };
        let projected = points
            .iter()
            .map(|point| project_for_hypertri(point, projection))
            .collect::<Vec<_>>();
        let Ok(triangles) = hypertri::earcut(&projected, &[]) else {
            stage.triangulation_failures += 1;
            continue;
        };
        stage.triangulations.push(ExactBoolMeshLoopTriangulation {
            output_face: face_loop.output_face,
            loop_index,
            source_side,
            source_face,
            projection,
            vertices: face_loop.vertices.clone(),
            triangles,
        });
    }

    stage
}

fn loop_source_face(
    halfedge_slot: Option<usize>,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
) -> Option<(ExactBoolMeshSide, usize)> {
    let source = &halfedges
        .output_halfedges
        .get(halfedge_slot?)?
        .as_ref()?
        .source;
    match source {
        ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
            side, source_face, ..
        }
        | ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
            side, source_face, ..
        }
        | ExactBoolMeshOutputHalfedgeSource::NewFacePair {
            side, source_face, ..
        } => Some((*side, *source_face)),
    }
}

fn source_mesh<'a>(
    side: ExactBoolMeshSide,
    face: usize,
    left: &'a ExactMesh,
    right: &'a ExactMesh,
) -> Option<&'a ExactMesh> {
    let mesh = match side {
        ExactBoolMeshSide::Left => left,
        ExactBoolMeshSide::Right => right,
    };
    (face < mesh.triangles().len()).then_some(mesh)
}

fn output_loop_points(
    vertices: &[usize],
    allocation: &ExactBoolMeshOutputVertexAllocation,
    boolean03: &ExactBoolMeshBoolean03,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Vec<Point3>> {
    vertices
        .iter()
        .map(|vertex| output_vertex_point(*vertex, allocation, boolean03, left, right))
        .collect()
}
