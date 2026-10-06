use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::HypermeshResult;
use crate::geometry::{Classification, Plane};
use crate::mesh::{PolygonSoup, TriangleMeshRef};
use crate::polygon::ConvexPolygon;
use crate::{Point3, PredicatePolicy};

pub(crate) const APPROXIMATE_CONTEXT: MeshContext =
    MeshContext::new(PredicatePolicy::APPROXIMATE_512);

pub(crate) fn approximate_decisions() -> DecisionContext {
    DecisionContext::new(&APPROXIMATE_CONTEXT)
}

fn value<T>(result: HypermeshResult<MeshOutcome<T>>) -> HypermeshResult<T> {
    result.map(MeshOutcome::into_value)
}

pub(crate) fn approximate_convex_triangle(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    value(crate::convex_triangle(
        &APPROXIMATE_CONTEXT,
        p0,
        p1,
        p2,
        mesh_index,
        polygon_index,
    ))
    .expect("approximate-policy triangle fixture must be nondegenerate")
}

pub(crate) fn approximate_polygon_soup(
    meshes: &[TriangleMeshRef<'_>],
) -> HypermeshResult<PolygonSoup> {
    value(crate::polygon_soup(&APPROXIMATE_CONTEXT, meshes))
}

pub(crate) fn approximate_classify_point(
    point: &Point3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    value(crate::classify_point(&APPROXIMATE_CONTEXT, point, plane))
}

pub(crate) fn approximate_convex_hull(points: &[Point3]) -> HypermeshResult<crate::TriangleMesh> {
    value(crate::convex_hull(&APPROXIMATE_CONTEXT, points))
}
