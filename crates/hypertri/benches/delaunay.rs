use criterion::{Criterion, criterion_group, criterion_main};
use hypertri::{PredicatePolicy, TriangulationContext};

const APPROX: TriangulationContext = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

fn bench_exact_lifted_delaunay(c: &mut Criterion) {
    let points = vec![
        [0.0, 0.0],
        [3.0, 0.0],
        [5.0, 2.0],
        [4.0, 5.0],
        [1.0, 4.0],
        [2.0, 2.0],
        [6.0, 4.0],
        [7.0, 1.0],
        [8.0, 5.0],
        [9.0, 2.0],
    ];

    c.bench_function("f64_exact_lifted_incremental_delaunay", |b| {
        b.iter(|| hypertri::f64::delaunay(&APPROX, &points).unwrap())
    });

    let polygon = vec![
        [0.0, 0.0],
        [12.0, 0.0],
        [12.0, 12.0],
        [0.0, 12.0],
        [3.0, 3.0],
        [9.0, 3.0],
        [9.0, 9.0],
        [3.0, 9.0],
    ];
    let constraints = vec![
        hypertri::Constraint::new(0, 1),
        hypertri::Constraint::new(1, 2),
        hypertri::Constraint::new(2, 3),
        hypertri::Constraint::new(3, 0),
        hypertri::Constraint::new(4, 5),
        hypertri::Constraint::new(5, 6),
        hypertri::Constraint::new(6, 7),
        hypertri::Constraint::new(7, 4),
    ];

    c.bench_function("f64_exact_lifted_closed_ring_cdt_hole", |b| {
        b.iter(|| hypertri::f64::constrained_delaunay(&APPROX, &polygon, &constraints).unwrap())
    });

    let flip_points = vec![
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 2.0],
        [0.0, 2.0],
        [3.0, 1.0],
        [-1.0, 1.0],
    ];
    let flip_constraints = vec![hypertri::Constraint::new(1, 3)];

    c.bench_function("f64_exact_lifted_cdt_edge_flip_recovery", |b| {
        b.iter(|| {
            hypertri::f64::constrained_delaunay(&APPROX, &flip_points, &flip_constraints).unwrap()
        })
    });

    let split_points = vec![[0.0, 0.0], [4.0, 0.0], [2.0, 0.0], [0.0, 3.0]];
    let split_constraints = vec![hypertri::Constraint::new(0, 1)];

    c.bench_function("f64_exact_lifted_cdt_existing_vertex_split", |b| {
        b.iter(|| {
            hypertri::f64::constrained_delaunay(&APPROX, &split_points, &split_constraints).unwrap()
        })
    });

    let crossing_points = vec![[0.0, 0.0], [4.0, 3.0], [0.0, 3.0], [4.0, 0.0]];
    let crossing_constraints = vec![
        hypertri::Constraint::new(0, 1),
        hypertri::Constraint::new(2, 3),
    ];

    c.bench_function("f64_exact_lifted_cdt_crossing_split", |b| {
        b.iter(|| {
            hypertri::f64::constrained_delaunay(&APPROX, &crossing_points, &crossing_constraints)
                .unwrap()
        })
    });

    let exact_points = vec![
        hypertri::Point2::new(hypertri::Real::from(0), hypertri::Real::from(0)),
        hypertri::Point2::new(hypertri::Real::from(4), hypertri::Real::from(3)),
        hypertri::Point2::new(hypertri::Real::from(0), hypertri::Real::from(3)),
        hypertri::Point2::new(hypertri::Real::from(4), hypertri::Real::from(0)),
    ];
    let exact_constraints = crossing_constraints.clone();
    let exact_crossing =
        hypertri::cdt::constrained_delaunay(&APPROX, &exact_points, &exact_constraints)
            .unwrap()
            .value;

    c.bench_function("exact_cdt_validate_crossing_split", |b| {
        b.iter(|| {
            exact_crossing.validate(&APPROX).unwrap();
            exact_crossing
                .validate_unconstrained_edges_are_delaunay(&APPROX)
                .unwrap();
        })
    });
}

criterion_group!(benches, bench_exact_lifted_delaunay);
criterion_main!(benches);
