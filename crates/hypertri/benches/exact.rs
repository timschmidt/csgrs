use criterion::{Criterion, criterion_group, criterion_main};
use hypertri::{Constraint, Point2, Rational, Real};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).unwrap())
}

fn p(x: Real, y: Real) -> Point2 {
    Point2::new(x, y)
}

fn bench_exact_triangulation(c: &mut Criterion) {
    let rational_spike = vec![
        p(r(0), r(0)),
        p(q(1, 1_000_000_000_000), q(1, 1_000_000_000_000_000)),
        p(r(3), r(0)),
        p(r(3), r(2)),
        p(r(0), r(2)),
    ];

    c.bench_function("exact_rational_spike_earcut", |b| {
        b.iter(|| hypertri::earcut(&rational_spike, &[]).unwrap())
    });

    let crossing_points = vec![p(r(0), r(0)), p(r(4), r(3)), p(r(0), r(3)), p(r(4), r(0))];
    let crossing_constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

    c.bench_function("exact_cdt_crossing_constraint_split", |b| {
        b.iter(|| {
            hypertri::cdt::constrained_delaunay(&crossing_points, &crossing_constraints).unwrap()
        })
    });

    let shared_denominator_polygon = hypertri::PolygonInput::new(
        vec![
            p(q(0, 3), q(0, 3)),
            p(q(12, 3), q(0, 3)),
            p(q(12, 3), q(9, 3)),
            p(q(0, 3), q(9, 3)),
        ],
        Vec::new(),
    );

    c.bench_function("exact_polygon_input_shared_denominator_facts", |b| {
        b.iter(|| {
            let facts = shared_denominator_polygon.facts();
            (
                facts.all_coordinates_exact_rational(),
                facts.has_shared_denominator_schedule(),
                facts.rings[0].known_axis_aligned_edges,
                facts.rings[0].signed_area,
                facts.rings[0].convexity,
            )
        })
    });
}

criterion_group!(benches, bench_exact_triangulation);
criterion_main!(benches);
