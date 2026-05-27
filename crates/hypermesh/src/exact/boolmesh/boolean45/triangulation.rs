//! Exact triangulation-prep for boolmesh `boolean45` face loops.
//!
//! Legacy boolmesh assembles halfedge loops, then triangulates each output face
//! boundary.  This module ports the first executable part of that handoff:
//! simple and holed faces are projected with exact source-face evidence and
//! sent to `hypertri` earcut.  That separation follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): assembled
//! boundary loops remain replayable topology, while exact triangulation is a
//! later certified object.  The simple/holed polygon triangulation step
//! consumes `hypertri`'s exact earcut port; its simple-polygon basis is
//! Meisters, "Polygons Have Ears," *The American Mathematical Monthly* 82.6
//! (1975), and its hole-bridging reduction follows de Berg et al.,
//! *Computational Geometry: Algorithms and Applications*.

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::{Point3, compare_reals};

use crate::exact::mesh::ExactMesh;
use crate::exact::region::{choose_region_projection, project_for_hypertri};
use crate::exact::scalar::ExactReal;

use super::super::{
    ExactBoolMeshBoolean03, ExactBoolMeshFaceLoopAssemblyStage, ExactBoolMeshHalfedgeAssemblyStage,
    ExactBoolMeshLoopTriangulation, ExactBoolMeshLoopTriangulationStage,
    ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshOutputVertexAllocation, ExactBoolMeshSide,
};
use super::geometry::output_vertex_point;

/// Triangulate assembled boolmesh output loops.
///
/// Legacy boolmesh's `general_triangulate` passes all loops of one output face
/// to its ear-clipper, with the outer loop and hole loops kept as separate
/// polygon rings.  The exact port mirrors that boundary by feeding a flat
/// earcut-compatible vertex buffer plus hole-start offsets into `hypertri`.
/// A face with multiple loops is therefore no longer a blocker when exact
/// projection, ring area, and hole bridging all replay.
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
        triangulate_output_face_loop_group(
            &loop_indices,
            left,
            right,
            boolean03,
            allocation,
            halfedges,
            face_loops,
            &mut stage,
        );
    }

    stage
}

fn triangulate_output_face_loop_group(
    loop_indices: &[usize],
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_loops: &ExactBoolMeshFaceLoopAssemblyStage,
    stage: &mut ExactBoolMeshLoopTriangulationStage,
) {
    if loop_indices.iter().any(|loop_index| {
        face_loops.loops[*loop_index].vertices.len() < 3
            || face_loops.loops[*loop_index].halfedges.len() < 3
    }) {
        stage.short_loops += 1;
        return;
    }

    let first_loop = &face_loops.loops[loop_indices[0]];
    let Some((source_side, source_face)) =
        loop_source_face(first_loop.halfedges.first().copied(), halfedges)
    else {
        stage.missing_source_faces += 1;
        return;
    };
    let Some(source_mesh) = source_mesh(source_side, source_face, left, right) else {
        stage.missing_source_faces += 1;
        return;
    };
    let Ok(projection) = choose_region_projection(source_mesh, source_face) else {
        stage.missing_source_faces += 1;
        return;
    };

    let mut rings = Vec::with_capacity(loop_indices.len());
    for &loop_index in loop_indices {
        let face_loop = &face_loops.loops[loop_index];
        let Some(points) =
            output_loop_points(&face_loop.vertices, allocation, boolean03, left, right)
        else {
            stage.missing_vertex_coordinates += 1;
            return;
        };
        let projected = points
            .iter()
            .map(|point| project_for_hypertri(point, projection))
            .collect::<Vec<_>>();
        let Some(area_abs) = projected_area2_abs(&projected) else {
            stage.triangulation_failures += 1;
            return;
        };
        if compare_reals(&area_abs, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
            stage.triangulation_failures += 1;
            return;
        }
        rings.push(ProjectedLoop {
            loop_index,
            vertices: face_loop.vertices.clone(),
            projected,
            area_abs,
        });
    }

    let Some(ordered) = order_polygon_rings(rings) else {
        stage.triangulation_failures += 1;
        return;
    };
    let mut vertices = Vec::new();
    let mut projected = Vec::new();
    let mut hole_indices = Vec::new();
    for (ring_index, ring) in ordered.iter().enumerate() {
        if ring_index > 0 {
            hole_indices.push(projected.len());
        }
        vertices.extend(ring.vertices.iter().copied());
        projected.extend(ring.projected.iter().cloned());
    }

    let Ok(triangles) = hypertri::earcut(&projected, &hole_indices) else {
        stage.triangulation_failures += 1;
        return;
    };
    if triangles.is_empty() {
        stage.triangulation_failures += 1;
        return;
    }
    stage.triangulations.push(ExactBoolMeshLoopTriangulation {
        output_face: first_loop.output_face,
        loop_index: ordered[0].loop_index,
        source_side,
        source_face,
        projection,
        vertices,
        triangles,
    });
}

#[derive(Clone)]
struct ProjectedLoop {
    loop_index: usize,
    vertices: Vec<usize>,
    projected: Vec<hypertri::ExactPoint>,
    area_abs: ExactReal,
}

fn order_polygon_rings(rings: Vec<ProjectedLoop>) -> Option<Vec<ProjectedLoop>> {
    let mut exterior = 0;
    for index in 1..rings.len() {
        match compare_reals(&rings[index].area_abs, &rings[exterior].area_abs).value()? {
            Ordering::Greater => exterior = index,
            Ordering::Equal | Ordering::Less => {}
        }
    }
    let mut ordered = Vec::with_capacity(rings.len());
    ordered.push(rings[exterior].clone());
    ordered.extend(
        rings
            .into_iter()
            .enumerate()
            .filter_map(|(index, ring)| (index != exterior).then_some(ring)),
    );
    Some(ordered)
}

fn projected_area2_abs(points: &[hypertri::ExactPoint]) -> Option<ExactReal> {
    let mut signed = ExactReal::from(0);
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        signed = signed + &((current.x.clone() * &next.y) - &(current.y.clone() * &next.x));
    }
    match compare_reals(&signed, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(ExactReal::from(0) - &signed),
        Ordering::Equal | Ordering::Greater => Some(signed),
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::validation::ValidationPolicy;
    use crate::exact::{
        ExactBoolMeshOutputFaceLoop, ExactBoolMeshOutputHalfedge,
        ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshOutputVertexOrigin,
        ExactBoolMeshSourceVertex,
    };

    fn planar_source() -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
                3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn empty_mesh() -> ExactMesh {
        ExactMesh::from_i64_triangles(&[], &[]).unwrap()
    }

    fn source_allocation(vertex_count: usize) -> ExactBoolMeshOutputVertexAllocation {
        ExactBoolMeshOutputVertexAllocation {
            left_vertex_output_starts: (0..vertex_count).map(Some).collect(),
            right_vertex_output_starts: Vec::new(),
            p1q2_output_starts: Vec::new(),
            p2q1_output_starts: Vec::new(),
            output_vertex_origins: (0..vertex_count)
                .map(|vertex| ExactBoolMeshOutputVertexOrigin::SourceVertex {
                    source: ExactBoolMeshSourceVertex {
                        side: ExactBoolMeshSide::Left,
                        vertex,
                    },
                    copy: 0,
                })
                .collect(),
        }
    }

    fn empty_boolean03(left_vertices: usize) -> ExactBoolMeshBoolean03 {
        ExactBoolMeshBoolean03 {
            p1q2: Vec::new(),
            p2q1: Vec::new(),
            x12: Vec::new(),
            x21: Vec::new(),
            v12: Vec::new(),
            v21: Vec::new(),
            w03: vec![0; left_vertices],
            w30: Vec::new(),
        }
    }

    fn face_halfedges(
        vertices: &[usize],
        start: usize,
    ) -> Vec<Option<ExactBoolMeshOutputHalfedge>> {
        vertices
            .iter()
            .enumerate()
            .map(|(local, &tail)| {
                let head = vertices[(local + 1) % vertices.len()];
                Some(ExactBoolMeshOutputHalfedge {
                    tail,
                    head,
                    pair: start + local,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
                        side: ExactBoolMeshSide::Left,
                        source_halfedge: local,
                        source_face: 0,
                        edge: [tail, head],
                        fragment: 0,
                        forward: true,
                    },
                })
            })
            .collect()
    }

    #[test]
    fn triangulates_holed_face_even_when_hole_loop_arrives_first() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[4, 5, 6, 7], 0);
        output_halfedges.extend(face_halfedges(&[0, 1, 2, 3], 4));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3],
                    vertices: vec![4, 5, 6, 7],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![4, 5, 6, 7],
                    vertices: vec![0, 1, 2, 3],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.multi_loop_faces, 0);
        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.loop_index, 1);
        assert_eq!(triangulation.vertices, vec![0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(triangulation.triangles.len(), 24);
        assert!(triangulation.triangles.iter().all(|index| *index < 8));
    }

    #[test]
    fn holed_face_with_short_ring_remains_blocked() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[0, 1, 2, 3], 0);
        output_halfedges.extend(face_halfedges(&[4, 5], 4));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3],
                    vertices: vec![0, 1, 2, 3],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![4, 5],
                    vertices: vec![4, 5],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.triangulations.len(), 0);
        assert_eq!(stage.short_loops, 1);
        assert_eq!(stage.multi_loop_faces, 0);
    }
}
