use std::hint::black_box;
use std::time::Instant;

use criterion::{Criterion, criterion_group, criterion_main};
use exactcore_hyper_comparison::{
    COMPARABLE_OPERATION_IDS, PreparedBenchmark, circle2_distance, circle2_relation,
    line2_intersection, line2_relation, point2_distance, segment2_relation,
};
use hypercurve::{CircularArc2, LineSeg2, Point2, Real, Segment2};

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
    criterion.bench_function(&format!("{id}/hypercurve"), |bencher| {
        bencher.iter(|| black_box(hyper()))
    });
}

fn r(value: i64) -> Real {
    value.into()
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn arc(center_x: i64) -> CircularArc2 {
    CircularArc2::try_from_center(
        p(center_x + 5, 0),
        p(center_x - 5, 0),
        p(center_x, 0),
        false,
    )
    .unwrap()
}

fn curve_benchmarks(criterion: &mut Criterion) {
    let first = LineSeg2::try_new(p(0, 0), p(13, 2)).unwrap();
    let second = LineSeg2::try_new(p(3, 17), p(19, -7)).unwrap();
    let query = p(4, 4);
    let coordinates = [0, 0, 13, 2, 3, 17, 19, -7];

    pair(
        criterion,
        "curve.line_length",
        || point2_distance([0, 0, 13, 2]),
        || first.length_squared().sqrt().unwrap(),
    );
    pair(
        criterion,
        "curve.line_side",
        || line2_relation(0, &[0, 0, 13, 2, 4, 4, 0, 0]),
        || {
            first
                .classify_point(&query)
                .expect("fixed curve fixture must certify")
        },
    );
    pair(
        criterion,
        "curve.line_contains_point",
        || segment2_relation(0, &[0, 0, 13, 2, 4, 4, 0, 0]),
        || {
            first
                .contains_point(&query)
                .expect("fixed curve fixture must certify")
        },
    );
    pair(
        criterion,
        "curve.line_intersection_topology",
        || segment2_relation(1, &coordinates),
        || {
            first
                .intersect_line(&second)
                .expect("fixed curve fixture must certify")
        },
    );
    pair(
        criterion,
        "curve.line_intersection_witness",
        || line2_intersection(coordinates),
        || {
            first
                .intersect_line(&second)
                .expect("fixed curve fixture must certify")
        },
    );
    pair(
        criterion,
        "curve.segment_dispatch",
        || segment2_relation(1, &coordinates),
        || {
            Segment2::Line(first.clone())
                .intersect_segment(&Segment2::Line(second.clone()))
                .expect("fixed curve fixture must certify")
        },
    );

    let first_circle = arc(0);
    let second_circle = arc(12);
    let line = LineSeg2::try_new(p(-10, 7), p(10, 7)).unwrap();
    pair(
        criterion,
        "curve.supporting_line_circle",
        || circle2_relation(0, 5, [0, 0, -10, 7, 10, 7]),
        || {
            line.supporting_line_circle_relation(&first_circle)
                .expect("fixed curve fixture must certify")
        },
    );
    pair(
        criterion,
        "curve.circle_point_distance",
        || circle2_distance(0, 5, 0, [0, 0, 13, 7]),
        || {
            p(13, 7)
                .distance_squared(first_circle.center())
                .sqrt()
                .unwrap()
                - r(5)
        },
    );
    pair(
        criterion,
        "curve.circle_circle_relation",
        || circle2_distance(1, 5, 5, [0, 0, 12, 0]),
        || {
            first_circle
                .circle_relation(&second_circle)
                .expect("fixed curve fixture must certify")
        },
    );
}

criterion_group!(benches, curve_benchmarks);
criterion_main!(benches);
