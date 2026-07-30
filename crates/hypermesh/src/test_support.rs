#![allow(dead_code)]

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshResult, HypermeshResult as Result};
use crate::geometry::{Classification, Plane};
use crate::intersection::PairwiseIntersection;
use crate::mesh::{PolygonSoup, TriangleMeshRef};
use crate::operations::EmberConfig;
use crate::output::{BooleanMesh, BooleanResult};
use crate::polygon::ConvexPolygon;
use crate::segment_trace::TraceAxisSegmentResult;
use crate::subdivision::{SubdivisionConfig, SubdivisionTask};
use crate::winding::{BooleanOp, WindingNumberVector};
use crate::{Aabb, Point3, PredicatePolicy};

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

pub(crate) fn approximate_convex_quad(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    p3: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    value(crate::convex_quad(
        &APPROXIMATE_CONTEXT,
        p0,
        p1,
        p2,
        p3,
        mesh_index,
        polygon_index,
    ))
    .expect("approximate-policy quad fixture must be nondegenerate")
}

pub(crate) fn approximate_classify_point(
    point: &Point3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    value(crate::classify_point(&APPROXIMATE_CONTEXT, point, plane))
}

pub(crate) fn approximate_intersect_polygons(
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    value(crate::intersect_polygons(
        &APPROXIMATE_CONTEXT,
        polygon,
        other,
        other_polygon_idx,
    ))
}

pub(crate) fn approximate_polygon_soup(
    meshes: &[TriangleMeshRef<'_>],
) -> HypermeshResult<PolygonSoup> {
    value(crate::polygon_soup(&APPROXIMATE_CONTEXT, meshes))
}

pub(crate) fn approximate_convex_hull(points: &[Point3]) -> HypermeshResult<crate::TriangleMesh> {
    value(crate::convex_hull(&APPROXIMATE_CONTEXT, points))
}

pub(crate) fn approximate_boolean_operation(
    meshes: &[TriangleMeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    value(crate::boolean_operation(
        &APPROXIMATE_CONTEXT,
        meshes,
        operation,
        config,
    ))
}

pub(crate) fn approximate_triangulate_and_resolve_certified(
    result: &BooleanResult,
) -> HypermeshResult<BooleanMesh> {
    value(crate::triangulate_and_resolve_certified(
        &APPROXIMATE_CONTEXT,
        result,
    ))
}

pub(crate) fn approximate_trace_axis_segment(
    start: &Point3,
    end: &Point3,
    axis: usize,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    value(crate::trace_axis_segment(
        &APPROXIMATE_CONTEXT,
        start,
        end,
        axis,
        start_wnv,
        polygons,
    ))
}

pub(crate) fn approximate_trace_segment(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    value(crate::trace_segment(
        &APPROXIMATE_CONTEXT,
        start,
        end,
        winding,
        polygons,
    ))
}

pub(crate) fn approximate_classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    value(crate::classify_leaf_polygon(
        &APPROXIMATE_CONTEXT,
        support,
        leaf_edges,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
    ))
}

pub(crate) fn approximate_subdivide(
    task: SubdivisionTask,
    operation: BooleanOp,
    config: SubdivisionConfig,
) -> HypermeshResult<Vec<crate::output::ClassifiedPolygon>> {
    value(crate::subdivide(
        &APPROXIMATE_CONTEXT,
        task,
        operation,
        config,
    ))
}

pub(crate) fn approximate_bounds_contains(bounds: &Aabb, point: &Point3) -> Result<bool> {
    value(bounds.contains_point(&APPROXIMATE_CONTEXT, point))
}
