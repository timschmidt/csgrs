use std::hint::black_box;
use std::time::Instant;

use criterion::{Criterion, criterion_group, criterion_main};
use exactcore_hyper_comparison::{
    COMPARABLE_OPERATION_IDS, PreparedBenchmark, plane3_relation, triangle3_relation,
};
use hyperlattice::HomogeneousPoint3;
use hypermesh::{
    ConvexPolygon, MeshContext, Plane, Point3, PredicatePolicy, Real, convex_triangle,
    intersect_polygons,
};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn pair<Exact, Hyper, ExactOutput, HyperOutput>(
    criterion: &mut Criterion,
    id: &str,
    mut exact: Exact,
    mut hyper: Hyper,
) where
    Exact: FnMut() -> ExactOutput,
    Hyper: FnMut() -> HyperOutput,
{
    assert!(
        COMPARABLE_OPERATION_IDS.contains(&id),
        "{id} is missing from the memory-operation catalog"
    );
    criterion.bench_function(&format!("{id}/exactCorelib"), |bencher| {
        bencher.iter(|| black_box(exact()))
    });
    let mut retained = PreparedBenchmark::new(id).expect("construct retained exactCore fixture");
    criterion.bench_function(&format!("{id}/exactCorelib-retained"), |bencher| {
        bencher.iter_custom(|iterations| {
            let started = Instant::now();
            retained
                .run_iterations(iterations)
                .expect("run retained exactCore fixture");
            started.elapsed()
        })
    });
    criterion.bench_function(&format!("{id}/hypermesh"), |bencher| {
        bencher.iter(|| black_box(hyper()))
    });
}

fn r(value: i64) -> Real {
    value.into()
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn h(point: &Point3) -> HomogeneousPoint3 {
    HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    )
}

fn triangle(points: [[i64; 3]; 3], index: isize) -> ConvexPolygon {
    convex_triangle(
        &CONTEXT,
        &p(points[0][0], points[0][1], points[0][2]),
        &p(points[1][0], points[1][1], points[1][2]),
        &p(points[2][0], points[2][1], points[2][2]),
        0,
        index,
    )
    .unwrap()
    .into_value()
}

fn mesh_benchmarks(criterion: &mut Criterion) {
    let base_points = [[0, 0, 0], [13, 2, 0], [3, 17, 0]];
    let base = triangle(base_points, 0);
    let query = p(4, 4, 0);
    let projective_query = h(&query);
    let point_coordinates = [0, 0, 0, 13, 2, 0, 3, 17, 0, 4, 4, 0, 0, 0, 0, 0, 0, 0];
    let plane = Plane::from_points(&p(0, 0, 0), &p(13, 2, 0), &p(3, 17, 0));

    pair(
        criterion,
        "mesh.plane_point_classification",
        || plane3_relation(1, &point_coordinates),
        || hypermesh::classify_point(&CONTEXT, &query, &plane),
    );
    pair(
        criterion,
        "mesh.triangle_contains_point",
        || triangle3_relation(1, &point_coordinates),
        || base.contains_point(&CONTEXT, &projective_query),
    );
    pair(
        criterion,
        "mesh.triangle_contains_point_strictly",
        || triangle3_relation(3, &point_coordinates),
        || base.contains_point_strictly(&CONTEXT, &projective_query),
    );
    pair(
        criterion,
        "mesh.triangle_boundary_point",
        || {
            triangle3_relation(
                2,
                &[0, 0, 0, 13, 2, 0, 3, 17, 0, 13, 2, 0, 0, 0, 0, 0, 0, 0],
            )
        },
        || {
            let endpoint = h(&p(13, 2, 0));
            let closed = base
                .contains_point(&CONTEXT, &endpoint)
                .unwrap()
                .into_value();
            let strict = base
                .contains_point_strictly(&CONTEXT, &endpoint)
                .unwrap()
                .into_value();
            closed && !strict
        },
    );

    let other_points = [[4, 4, -7], [4, 4, 7], [9, 4, 0]];
    let other = triangle(other_points, 1);
    let intersection_coordinates = [0, 0, 0, 13, 2, 0, 3, 17, 0, 4, 4, -7, 4, 4, 7, 9, 4, 0];
    pair(
        criterion,
        "mesh.triangle_triangle_intersection",
        || triangle3_relation(6, &intersection_coordinates),
        || intersect_polygons(&CONTEXT, &base, &other, 1),
    );
}

criterion_group!(benches, mesh_benchmarks);
criterion_main!(benches);
