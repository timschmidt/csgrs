use std::hint::black_box;
use std::time::Instant;

use criterion::{Criterion, criterion_group, criterion_main};
use exactcore_hyper_comparison::{
    COMPARABLE_OPERATION_IDS, PreparedBenchmark, between2, circle2_distance, circle2_relation,
    line2_relation, point2_distance, segment2_relation,
};
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment};
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

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
    criterion.bench_function(&format!("{id}/hyperpath"), |bencher| {
        bencher.iter(|| black_box(hyper()))
    });
}

fn r(value: i64) -> Real {
    value.into()
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn line(x0: i64, y0: i64, x1: i64, y1: i64) -> LinePathSegment {
    LinePathSegment::new(p(x0, y0), p(x1, y1), POLICY).unwrap()
}

fn circle(center_x: i64, radius: i64) -> ExplicitCircularArc {
    ExplicitCircularArc::new(
        p(center_x, 0),
        r(radius),
        p(center_x + radius, 0),
        p(center_x + radius, 0),
        ArcDirection::Ccw,
        POLICY,
    )
    .unwrap()
}

fn path_benchmarks(criterion: &mut Criterion) {
    let segment = line(-2, 3, 6, -3);
    pair(
        criterion,
        "path.line_length",
        || point2_distance([-2, 3, 6, -3]),
        || segment.euclidean_length().unwrap(),
    );
    pair(
        criterion,
        "path.line_axis_classification",
        || line2_relation(6, &[0, 4, 12, 4, 0, 0, 0, 0]),
        || line(0, 4, 12, 4).axis_length(POLICY),
    );
    let same = line(6, -3, -2, 3);
    pair(
        criterion,
        "path.line_endpoint_equality",
        || segment2_relation(2, &[-2, 3, 6, -3, 6, -3, -2, 3]),
        || segment.exact_endpoint_equal(&same, POLICY),
    );
    let ordered = line(0, 0, 10, 0);
    pair(
        criterion,
        "path.line_parameter_order",
        || between2([0, 0, 2, 0, 8, 0]),
        || ordered.compare_points_along(&p(2, 0), &p(8, 0), POLICY),
    );

    let base = circle(0, 5);
    pair(
        criterion,
        "path.circle_point_membership",
        || circle2_distance(0, 5, 0, [0, 0, 3, 4]),
        || base.classify_point(&p(3, 4), POLICY),
    );
    let crossing = line(-10, 0, 10, 0);
    pair(
        criterion,
        "path.circle_segment_intersection",
        || circle2_relation(1, 5, [0, 0, -10, 0, 10, 0]),
        || base.intersect_segment(&crossing, POLICY),
    );
    pair(
        criterion,
        "path.circle_circle_relation",
        || circle2_distance(1, 5, 5, [0, 0, 12, 0]),
        || base.classify_circle_relation(&circle(12, 5), POLICY),
    );
}

criterion_group!(benches, path_benchmarks);
criterion_main!(benches);
