//! Exact endpoint-shadow boundary classification for boolmesh `kernel12`.
//!
//! Legacy boolmesh splits endpoint contacts between the vertex/face terms in
//! `boolean03::kernel02`/`kernel12` and the edge/edge shadow rule in
//! `boolean03::kernel11`.  This module keeps that same boundary explicit:
//! strict face-interior endpoint shadows may lower directly into `p1q2` or
//! `p2q1`, while edge and vertex boundary shadows are identified but left for
//! the direct `Kernel11` port.  Separating the exact point, projected
//! predicate result, and topological boundary identity follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use std::cmp::Ordering;

use hyperlimit::{
    CoplanarProjection, Point2, Point3, Sign, TriangleLocation, classify_point_triangle,
    compare_reals, point_on_segment, project_point3, projected_polygon_area2_value,
};

use crate::exact::mesh::ExactMesh;

use super::{ExactBoolMeshEdgeFacePair, ExactBoolMeshSide, ExactReal};

/// Exact location of a source endpoint shadow on the opposite triangle.
///
/// The edge and vertex variants carry local vertex indices in the opposite
/// face, not global mesh vertex handles.  That mirrors the boolmesh kernels,
/// which reason about the current face-local triangle before emitting global
/// halfedge rows.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum EndpointShadowLocation {
    /// The endpoint lies strictly inside the opposite triangle.
    StrictInterior,
    /// The endpoint lies on a unique opposite-face boundary edge.
    BoundaryEdge([usize; 2]),
    /// The endpoint coincides with an opposite-face vertex.
    BoundaryVertex(usize),
    /// The endpoint does not shadow the closed opposite triangle.
    Outside,
    /// The opposite triangle has no certified non-zero projected area.
    Degenerate,
}

/// Classify an endpoint shadow against the exact opposite face.
///
/// This ports the boolmesh kernel split without importing its floating-point
/// expansion logic: `TriangleLocation::Inside` is the only case consumed by
/// the current strict vertex/face lowering, while `OnEdge` and `OnVertex` are
/// normalized into face-local boundary identities for the later exact
/// `Kernel11` edge-shadow rule.
pub(super) fn classify_endpoint_shadow(
    point: &Point3,
    edge_face: ExactBoolMeshEdgeFacePair,
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
) -> Option<EndpointShadowLocation> {
    let face_mesh = match edge_face.face_side {
        ExactBoolMeshSide::Left => left_mesh,
        ExactBoolMeshSide::Right => right_mesh,
    };
    let triangle = face_mesh.triangles().get(edge_face.face)?.0;
    let face = [
        face_mesh.vertices().get(triangle[0])?.to_hyperlimit_point(),
        face_mesh.vertices().get(triangle[1])?.to_hyperlimit_point(),
        face_mesh.vertices().get(triangle[2])?.to_hyperlimit_point(),
    ];
    let Some(projection) = choose_triangle_projection(&face) else {
        return Some(EndpointShadowLocation::Degenerate);
    };
    let projected_face = [
        project_point3(&face[0], projection),
        project_point3(&face[1], projection),
        project_point3(&face[2], projection),
    ];
    let projected_point = project_point3(point, projection);
    let location = classify_point_triangle(
        &projected_face[0],
        &projected_face[1],
        &projected_face[2],
        &projected_point,
    )
    .value()?;

    match location {
        TriangleLocation::Inside => Some(EndpointShadowLocation::StrictInterior),
        TriangleLocation::OnEdge => boundary_edge(&projected_face, &projected_point)
            .map(EndpointShadowLocation::BoundaryEdge),
        TriangleLocation::OnVertex => boundary_vertex(&projected_face, &projected_point)
            .map(EndpointShadowLocation::BoundaryVertex),
        TriangleLocation::Outside => Some(EndpointShadowLocation::Outside),
        TriangleLocation::Degenerate => Some(EndpointShadowLocation::Degenerate),
    }
}

fn boundary_vertex(face: &[Point2; 3], point: &Point2) -> Option<usize> {
    (0..3).find(|&index| same_projected_point(&face[index], point))
}

fn boundary_edge(face: &[Point2; 3], point: &Point2) -> Option<[usize; 2]> {
    let mut edge = None;
    for candidate in [[0, 1], [1, 2], [2, 0]] {
        if point_on_segment(&face[candidate[0]], &face[candidate[1]], point).value() == Some(true) {
            if edge.is_some() {
                return None;
            }
            edge = Some(candidate);
        }
    }
    edge
}

fn same_projected_point(left: &Point2, right: &Point2) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
}

fn choose_triangle_projection(points: &[Point3; 3]) -> Option<CoplanarProjection> {
    [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_polygon_area2_value(points, projection);
        !matches!(real_sign(&area), Some(Sign::Zero) | None)
    })
}

fn real_sign(value: &ExactReal) -> Option<Sign> {
    match compare_reals(value, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(Sign::Negative),
        Ordering::Equal => Some(Sign::Zero),
        Ordering::Greater => Some(Sign::Positive),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::mesh::ExactMesh;

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn edge_face(face: usize) -> ExactBoolMeshEdgeFacePair {
        ExactBoolMeshEdgeFacePair {
            face_pair: super::super::ExactBoolMeshFacePair {
                left_face: 0,
                right_face: face,
            },
            edge_side: ExactBoolMeshSide::Left,
            edge: [0, 1],
            face_side: ExactBoolMeshSide::Right,
            face,
        }
    }

    #[test]
    fn classifies_strict_endpoint_face_shadow() {
        let left = tetrahedron_i64([1, 1, 0], [1, 1, 2], [2, 1, 1], [1, 2, 1]);
        let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let location = classify_endpoint_shadow(
            &Point3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(0)),
            edge_face(0),
            &left,
            &right,
        );

        assert_eq!(location, Some(EndpointShadowLocation::StrictInterior));
    }

    #[test]
    fn classifies_endpoint_on_opposite_boundary_edge() {
        let left = tetrahedron_i64([2, 0, 0], [2, 0, 2], [3, 1, 1], [1, 1, 1]);
        let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let location = classify_endpoint_shadow(
            &Point3::new(ExactReal::from(2), ExactReal::from(0), ExactReal::from(0)),
            edge_face(0),
            &left,
            &right,
        );

        assert_eq!(location, Some(EndpointShadowLocation::BoundaryEdge([2, 0])));
    }

    #[test]
    fn classifies_endpoint_on_opposite_boundary_vertex() {
        let left = tetrahedron_i64([0, 0, 0], [0, 0, 2], [1, 1, 1], [1, 0, 1]);
        let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let location = classify_endpoint_shadow(
            &Point3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(0)),
            edge_face(0),
            &left,
            &right,
        );

        assert_eq!(location, Some(EndpointShadowLocation::BoundaryVertex(0)));
    }
}
