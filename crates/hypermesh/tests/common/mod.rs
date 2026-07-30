#![allow(dead_code)]

use hypermesh::clip::{ClipResult, clip_polygon};
use hypermesh::output::ClassifiedPolygon;
use hypermesh::{
    Aabb, BooleanMesh, BooleanMeshClosureEvidence, BooleanOp, BooleanResult, Classification,
    ConvexPolygon, EmberConfig, ExactBvh, ExactPointBvh, HypermeshResult, MeshContext, MeshOutcome,
    Plane, Point3, PredicatePolicy, SubdivisionConfig, SubdivisionTask, TraceAxisSegmentResult,
    TriangleMesh, TriangleMeshRef, WindingNumberVector,
};

pub const APPROXIMATE_CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn value<T>(result: HypermeshResult<MeshOutcome<T>>) -> HypermeshResult<T> {
    result.map(MeshOutcome::into_value)
}

pub fn approximate_convex_triangle(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    value(hypermesh::convex_triangle(
        &APPROXIMATE_CONTEXT,
        p0,
        p1,
        p2,
        mesh_index,
        polygon_index,
    ))
    .expect("approximate-policy triangle fixture must be nondegenerate")
}

pub fn approximate_convex_quad(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    p3: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    value(hypermesh::convex_quad(
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

pub fn approximate_polygon_soup(
    meshes: &[TriangleMeshRef<'_>],
) -> HypermeshResult<hypermesh::PolygonSoup> {
    value(hypermesh::polygon_soup(&APPROXIMATE_CONTEXT, meshes))
}

pub fn approximate_boolean_operation(
    meshes: &[TriangleMeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    value(hypermesh::boolean_operation(
        &APPROXIMATE_CONTEXT,
        meshes,
        operation,
        config,
    ))
}

pub fn approximate_boolean_mesh(
    meshes: &[TriangleMeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanMesh> {
    value(hypermesh::boolean_mesh(
        &APPROXIMATE_CONTEXT,
        meshes,
        operation,
        config,
    ))
}

pub fn approximate_boolean_triangle_meshes(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<TriangleMesh> {
    value(hypermesh::boolean_triangle_meshes(
        &APPROXIMATE_CONTEXT,
        left,
        right,
        operation,
        config,
    ))
}

pub fn approximate_triangulate_and_resolve_certified(
    result: &BooleanResult,
) -> HypermeshResult<BooleanMesh> {
    value(hypermesh::triangulate_and_resolve_certified(
        &APPROXIMATE_CONTEXT,
        result,
    ))
}

pub fn approximate_certify_output_polygon_closure(
    result: &BooleanResult,
) -> HypermeshResult<BooleanMeshClosureEvidence> {
    value(hypermesh::certify_output_polygon_closure(
        &APPROXIMATE_CONTEXT,
        result,
    ))
}

pub fn approximate_classify_point(
    point: &Point3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    value(hypermesh::classify_point(
        &APPROXIMATE_CONTEXT,
        point,
        plane,
    ))
}

pub fn approximate_intersect_polygons(
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<hypermesh::PairwiseIntersection> {
    value(hypermesh::intersect_polygons(
        &APPROXIMATE_CONTEXT,
        polygon,
        other,
        other_polygon_idx,
    ))
}

pub fn approximate_clip_polygon(
    polygon: &ConvexPolygon,
    split_plane: &Plane,
) -> HypermeshResult<ClipResult> {
    value(clip_polygon(&APPROXIMATE_CONTEXT, polygon, split_plane))
}

pub fn approximate_bounds_overlap(
    left: &hypermesh::ApproxBounds,
    right: &hypermesh::ApproxBounds,
) -> HypermeshResult<bool> {
    value(hypermesh::bvh::bounds_overlap(
        &APPROXIMATE_CONTEXT,
        left,
        right,
    ))
}

pub fn approximate_aabb_contains_point(bounds: &Aabb, point: &Point3) -> HypermeshResult<bool> {
    value(bounds.contains_point(&APPROXIMATE_CONTEXT, point))
}

pub fn approximate_polygon_is_valid(polygon: &ConvexPolygon) -> HypermeshResult<bool> {
    value(polygon.is_valid(&APPROXIMATE_CONTEXT))
}

pub fn approximate_certify_convex_mesh(mesh: TriangleMeshRef<'_>) -> HypermeshResult<()> {
    value(hypermesh::certify_convex_mesh(&APPROXIMATE_CONTEXT, mesh))
}

pub fn approximate_exact_bvh_build(polygons: &[ConvexPolygon]) -> HypermeshResult<ExactBvh> {
    value(ExactBvh::build(&APPROXIMATE_CONTEXT, polygons))
}

pub fn approximate_exact_bvh_intersect_pairs<F>(
    left: &ExactBvh,
    right: &ExactBvh,
    callback: F,
) -> HypermeshResult<()>
where
    F: FnMut(usize, usize),
{
    value(left.intersect_pairs(&APPROXIMATE_CONTEXT, right, callback))
}

pub fn approximate_exact_point_bvh_build(points: &[Point3]) -> HypermeshResult<ExactPointBvh> {
    value(ExactPointBvh::build(&APPROXIMATE_CONTEXT, points))
}

pub fn approximate_query_positive_halfspace<F>(
    bvh: &ExactPointBvh,
    points: &[Point3],
    plane: &Plane,
    callback: F,
) -> HypermeshResult<()>
where
    F: FnMut(usize),
{
    value(bvh.query_positive_halfspace(&APPROXIMATE_CONTEXT, points, plane, callback))
}

pub fn approximate_query_positive_oriented_plane<F>(
    bvh: &ExactPointBvh,
    points: &[Point3],
    a: &Point3,
    b: &Point3,
    c: &Point3,
    callback: F,
) -> HypermeshResult<()>
where
    F: FnMut(usize),
{
    value(bvh.query_positive_oriented_plane(&APPROXIMATE_CONTEXT, points, a, b, c, callback))
}

pub fn approximate_query_negative_oriented_plane<F>(
    bvh: &ExactPointBvh,
    points: &[Point3],
    a: &Point3,
    b: &Point3,
    c: &Point3,
    callback: F,
) -> HypermeshResult<()>
where
    F: FnMut(usize),
{
    value(bvh.query_negative_oriented_plane(&APPROXIMATE_CONTEXT, points, a, b, c, callback))
}

pub fn approximate_trace_axis_segment(
    start: &Point3,
    end: &Point3,
    axis: usize,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    value(hypermesh::trace_axis_segment(
        &APPROXIMATE_CONTEXT,
        start,
        end,
        axis,
        start_wnv,
        polygons,
    ))
}

pub fn approximate_trace_segment(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    value(hypermesh::trace_segment(
        &APPROXIMATE_CONTEXT,
        start,
        end,
        winding,
        polygons,
    ))
}

#[allow(clippy::too_many_arguments)]
pub fn approximate_classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    value(hypermesh::classify_leaf_polygon(
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

pub fn approximate_process_leaf_into(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    operation: BooleanOp,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<hypermesh::LeafProcessingStats> {
    value(hypermesh::process_leaf_into(
        &APPROXIMATE_CONTEXT,
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        operation,
        output,
    ))
}

pub fn approximate_subdivide(
    task: SubdivisionTask,
    operation: BooleanOp,
    config: SubdivisionConfig,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    value(hypermesh::subdivide(
        &APPROXIMATE_CONTEXT,
        task,
        operation,
        config,
    ))
}
