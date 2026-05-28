//! Exact mesh-export staging for boolmesh `boolean45`.
//!
//! Legacy boolmesh writes final triangle topology after output halfedges have
//! been assembled and face boundaries triangulated.  This module ports that
//! handoff as a replayable staging artifact: output vertex ids are still the
//! boolmesh allocation ids, and triangle ids are the materialized `hypertri`
//! triplets from the previous stage.  Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997), motivates keeping this
//! export candidate separate from final `ExactMesh` construction so stale
//! topology can be rejected before retained mesh facts are built.

use core::cmp::Ordering;

use hyperlimit::{CoplanarProjection, Point3, compare_reals, project_point3 as project_point};

use crate::exact::mesh::ExactPoint3;
use crate::exact::mesh::{ExactMesh, Triangle};
use crate::exact::region::choose_region_projection;
use crate::exact::scalar::ExactReal;

use super::super::{
    ExactBoolMeshBoolean03, ExactBoolMeshMeshExportStage, ExactBoolMeshOutputTriangleStage,
    ExactBoolMeshOutputVertexAllocation, ExactBoolMeshSide,
};
use super::geometry::{output_vertex_origin_has_coordinate, output_vertex_point};

/// Build the exact final-triangle export candidate for a `boolean45` stage.
pub(super) fn stage_mesh_export(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    output_triangles: &ExactBoolMeshOutputTriangleStage,
) -> ExactBoolMeshMeshExportStage {
    let missing_vertex_coordinates = allocation
        .output_vertex_origins
        .iter()
        .filter(|origin| {
            !output_vertex_origin_has_coordinate(
                **origin,
                boolean03,
                left.vertices().len(),
                right.vertices().len(),
            )
        })
        .count();
    let allocation_vertex_count = allocation.output_vertex_origins.len();
    let mut stage = ExactBoolMeshMeshExportStage {
        vertex_count: allocation_vertex_count + output_triangles.steiner_points.len(),
        steiner_points: output_triangles.steiner_points.clone(),
        triangles: Vec::with_capacity(output_triangles.triangles.len()),
        missing_vertex_coordinates,
        blocked_output_triangles: output_triangles.missing_loop_triangulations
            + output_triangles.invalid_local_triangles,
        invalid_output_triangles: 0,
        orientation_failures: 0,
    };

    for triangle in &output_triangles.triangles {
        if triangle
            .vertices
            .iter()
            .any(|vertex| *vertex >= stage.vertex_count)
            || triangle.vertices[0] == triangle.vertices[1]
            || triangle.vertices[1] == triangle.vertices[2]
            || triangle.vertices[2] == triangle.vertices[0]
        {
            stage.invalid_output_triangles += 1;
            continue;
        }
        let Some(oriented) = orient_triangle_to_source(
            triangle.vertices,
            triangle.source_side,
            triangle.source_face,
            allocation,
            boolean03,
            &output_triangles.steiner_points,
            left,
            right,
        ) else {
            stage.orientation_failures += 1;
            continue;
        };
        stage.triangles.push(Triangle(oriented));
    }

    stage
}

fn orient_triangle_to_source(
    vertices: [usize; 3],
    source_side: ExactBoolMeshSide,
    source_face: usize,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    boolean03: &ExactBoolMeshBoolean03,
    steiner_points: &[ExactPoint3],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<[usize; 3]> {
    let source = match source_side {
        ExactBoolMeshSide::Left => left,
        ExactBoolMeshSide::Right => right,
    };
    let projection = choose_region_projection(source, source_face).ok()?;
    let source_triangle = source.triangles().get(source_face)?.0;
    let source_points = [
        source
            .vertices()
            .get(source_triangle[0])?
            .to_hyperlimit_point(),
        source
            .vertices()
            .get(source_triangle[1])?
            .to_hyperlimit_point(),
        source
            .vertices()
            .get(source_triangle[2])?
            .to_hyperlimit_point(),
    ];
    let output_points = [
        export_vertex_point(
            vertices[0],
            allocation,
            boolean03,
            steiner_points,
            left,
            right,
        )?,
        export_vertex_point(
            vertices[1],
            allocation,
            boolean03,
            steiner_points,
            left,
            right,
        )?,
        export_vertex_point(
            vertices[2],
            allocation,
            boolean03,
            steiner_points,
            left,
            right,
        )?,
    ];
    let source_sign = triangle_area_ordering(&source_points, projection)?;
    let output_sign = triangle_area_ordering(&output_points, projection)?;
    if source_sign == output_sign {
        Some(vertices)
    } else {
        Some([vertices[0], vertices[2], vertices[1]])
    }
}

fn export_vertex_point(
    vertex: usize,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    boolean03: &ExactBoolMeshBoolean03,
    steiner_points: &[ExactPoint3],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Point3> {
    if vertex < allocation.output_vertex_origins.len() {
        output_vertex_point(vertex, allocation, boolean03, left, right)
    } else {
        steiner_points
            .get(vertex - allocation.output_vertex_origins.len())
            .map(ExactPoint3::to_hyperlimit_point)
    }
}

fn triangle_area_ordering(
    points: &[Point3; 3],
    projection: CoplanarProjection,
) -> Option<Ordering> {
    let area = projected_area2_signed(points, projection);
    match compare_reals(&area, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(Ordering::Less),
        Ordering::Greater => Some(Ordering::Greater),
        Ordering::Equal => None,
    }
}

fn projected_area2_signed(points: &[Point3; 3], projection: CoplanarProjection) -> ExactReal {
    let mut sum = ExactReal::from(0);
    for index in 0..3 {
        let current = project_point(&points[index], projection);
        let next = project_point(&points[(index + 1) % 3], projection);
        sum = add(
            &sum,
            &sub(&mul(&current.x, &next.y), &mul(&current.y, &next.x)),
        );
    }
    sum
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}
