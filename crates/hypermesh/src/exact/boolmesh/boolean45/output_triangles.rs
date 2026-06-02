//! Exact output-triangle materialization for boolmesh `boolean45`.
//!
//! Legacy boolmesh emits output mesh triangles only after face boundaries have
//! been assembled and triangulated.  This module ports that handoff without
//! exporting the final mesh yet: each local `hypertri` index triple is resolved
//! through the boolmesh output-vertex list into a replayable triangle record.
//! certificate and the topology mutation remain separate and source-replayable.
//! The simple-loop local index buffers consumed here come from the earcut

use super::super::{
    ExactBoolMeshLoopTriangulationStage, ExactBoolMeshOutputTriangleStage,
    ExactBoolMeshTriangulatedOutputTriangle,
};

/// Resolve exact loop triangulations into output vertex triplets.
pub(super) fn materialize_output_triangles(
    triangulations: &ExactBoolMeshLoopTriangulationStage,
    allocation_vertex_count: usize,
) -> ExactBoolMeshOutputTriangleStage {
    let mut stage = ExactBoolMeshOutputTriangleStage {
        triangles: Vec::new(),
        steiner_points: Vec::new(),
        missing_loop_triangulations: triangulations.multi_loop_faces
            + triangulations.short_loops
            + triangulations.missing_source_faces
            + triangulations.missing_vertex_coordinates
            + triangulations.triangulation_failures,
        invalid_local_triangles: 0,
    };

    for triangulation in &triangulations.triangulations {
        let local_point_count = triangulation.vertices.len() + triangulation.steiner_points.len();
        let steiner_output_offset = allocation_vertex_count + stage.steiner_points.len();
        stage
            .steiner_points
            .extend(triangulation.steiner_points.iter().cloned());
        for local_triangle in triangulation.triangles.chunks_exact(3) {
            let local_triangle = [local_triangle[0], local_triangle[1], local_triangle[2]];
            if local_triangle
                .iter()
                .any(|index| *index >= local_point_count)
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
                        resolve_local_vertex(
                            local_triangle[0],
                            &triangulation.vertices,
                            steiner_output_offset,
                        ),
                        resolve_local_vertex(
                            local_triangle[1],
                            &triangulation.vertices,
                            steiner_output_offset,
                        ),
                        resolve_local_vertex(
                            local_triangle[2],
                            &triangulation.vertices,
                            steiner_output_offset,
                        ),
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

fn resolve_local_vertex(local: usize, vertices: &[usize], steiner_output_offset: usize) -> usize {
    vertices
        .get(local)
        .copied()
        .unwrap_or_else(|| steiner_output_offset + local - vertices.len())
}
