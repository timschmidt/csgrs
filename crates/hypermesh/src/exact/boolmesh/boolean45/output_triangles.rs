//! Exact output-triangle materialization for boolmesh `boolean45`.
//!
//! Legacy boolmesh emits output mesh triangles only after face boundaries have
//! been assembled and triangulated.  This module ports that handoff without
//! exporting the final mesh yet: each local `hypertri` index triple is resolved
//! through the boolmesh output-vertex list into a replayable triangle record.
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997), is the boundary condition here: the exact triangulation
//! certificate and the topology mutation remain separate and source-replayable.
//! The simple-loop local index buffers consumed here come from the earcut
//! stage based on Meisters, "Polygons Have Ears," *The American Mathematical
//! Monthly* 82.6 (1975).

use super::super::{
    ExactBoolMeshLoopTriangulationStage, ExactBoolMeshOutputTriangleStage,
    ExactBoolMeshTriangulatedOutputTriangle,
};

/// Resolve exact loop triangulations into output vertex triplets.
pub(super) fn materialize_output_triangles(
    triangulations: &ExactBoolMeshLoopTriangulationStage,
) -> ExactBoolMeshOutputTriangleStage {
    let mut stage = ExactBoolMeshOutputTriangleStage {
        triangles: Vec::new(),
        missing_loop_triangulations: triangulations.multi_loop_faces
            + triangulations.short_loops
            + triangulations.missing_source_faces
            + triangulations.missing_vertex_coordinates
            + triangulations.triangulation_failures,
        invalid_local_triangles: 0,
    };

    for triangulation in &triangulations.triangulations {
        for local_triangle in triangulation.triangles.chunks_exact(3) {
            let local_triangle = [local_triangle[0], local_triangle[1], local_triangle[2]];
            if local_triangle
                .iter()
                .any(|index| *index >= triangulation.vertices.len())
                || local_triangle[0] == local_triangle[1]
                || local_triangle[1] == local_triangle[2]
                || local_triangle[2] == local_triangle[0]
            {
                stage.invalid_local_triangles += 1;
                continue;
            }
            stage
                .triangles
                .push(ExactBoolMeshTriangulatedOutputTriangle {
                    output_face: triangulation.output_face,
                    loop_index: triangulation.loop_index,
                    source_side: triangulation.source_side,
                    source_face: triangulation.source_face,
                    local_triangle,
                    vertices: [
                        triangulation.vertices[local_triangle[0]],
                        triangulation.vertices[local_triangle[1]],
                        triangulation.vertices[local_triangle[2]],
                    ],
                });
        }
        if !triangulation
            .triangles
            .chunks_exact(3)
            .remainder()
            .is_empty()
        {
            stage.invalid_local_triangles += 1;
        }
    }

    stage
}
