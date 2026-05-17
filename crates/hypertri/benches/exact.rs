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

    c.bench_function("exact_rational_spike_earcut_diagnostics", |b| {
        b.iter(|| {
            let report = hypertri::earcut_report(&rational_spike, &[]).unwrap();
            (
                report.triangles.len(),
                report.diagnostics.ear_tests,
                report.diagnostics.containment_candidates,
                report.diagnostics.containment_prepared_reflex_lookups,
                report.diagnostics.containment_convex_rejects,
                report.diagnostics.prepared_reflex_rebuilds,
                report.diagnostics.prepared_reflex_updates,
                report.diagnostics.containment_bbox_rejects,
                report.diagnostics.containment_tests,
                report.diagnostics.emitted_triangles,
            )
        })
    });

    let mut sawtooth = Vec::new();
    for i in 0..32_i64 {
        sawtooth.push(p(r(i), if i % 2 == 0 { r(0) } else { q(1, 3) }));
    }
    sawtooth.push(p(r(31), r(4)));
    sawtooth.push(p(r(0), r(4)));

    c.bench_function("exact_sawtooth_earcut_candidate_pressure", |b| {
        b.iter(|| {
            // This row intentionally measures the exact ear loop before adding
            // z-order candidate pruning or unsafe indexing. The report counts
            // predicate-stage pressure while topology still routes through
            // exact `hyperlimit` predicates, following Yap's advice to retain
            // and measure object-level structure first; see Yap, "Towards
            // Exact Geometric Computation," Computational Geometry 7.1-2
            // (1997).
            let report = hypertri::earcut_report(&sawtooth, &[]).unwrap();
            (
                report.triangles.len(),
                report.diagnostics.ear_tests,
                report.diagnostics.containment_candidates,
                report.diagnostics.containment_prepared_reflex_lookups,
                report.diagnostics.containment_convex_rejects,
                report.diagnostics.prepared_reflex_rebuilds,
                report.diagnostics.prepared_reflex_updates,
                report.diagnostics.containment_bbox_rejects,
                report.diagnostics.containment_tests,
                report.diagnostics.split_fallbacks,
            )
        })
    });

    let crossing_points = vec![p(r(0), r(0)), p(r(4), r(3)), p(r(0), r(3)), p(r(4), r(0))];
    let crossing_constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];

    c.bench_function("exact_cdt_crossing_constraint_split", |b| {
        b.iter(|| {
            hypertri::cdt::constrained_delaunay(&crossing_points, &crossing_constraints).unwrap()
        })
    });

    let separated_cycles = vec![
        p(r(0), r(0)),
        p(r(2), r(0)),
        p(r(2), r(2)),
        p(r(0), r(2)),
        p(r(5), r(0)),
        p(r(7), r(0)),
        p(r(7), r(3)),
        p(r(5), r(3)),
    ];
    let separated_constraints = vec![
        Constraint::new(0, 1),
        Constraint::new(1, 2),
        Constraint::new(2, 3),
        Constraint::new(3, 0),
        Constraint::new(4, 5),
        Constraint::new(5, 6),
        Constraint::new(6, 7),
        Constraint::new(7, 4),
    ];

    c.bench_function("exact_cdt_separated_cycles_general_pslg", |b| {
        b.iter(|| {
            // This is not a polygon-with-holes shortcut. It measures the exact
            // PSLG path that starts from an exact Delaunay triangulation,
            // recovers protected cycle edges, then re-legalizes unprotected
            // edges using Lee-Lin constrained-Delaunay local legality under
            // Yap-style exact predicates.
            hypertri::cdt::constrained_delaunay(&separated_cycles, &separated_constraints).unwrap()
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

    let nd_points = vec![
        hypertri::nd::PointD::new(vec![r(0), r(0), r(0), r(0)]),
        hypertri::nd::PointD::new(vec![r(1), r(0), r(0), r(0)]),
        hypertri::nd::PointD::new(vec![r(0), r(1), r(0), r(0)]),
        hypertri::nd::PointD::new(vec![r(0), r(0), r(1), r(0)]),
        hypertri::nd::PointD::new(vec![r(0), r(0), r(0), r(1)]),
        hypertri::nd::PointD::new(vec![q(1, 5), q(1, 5), q(1, 5), q(1, 5)]),
    ];

    c.bench_function("exact_nd_4d_delaunay_complex", |b| {
        b.iter(|| hypertri::nd::delaunay_complex(&nd_points).unwrap())
    });
}

criterion_group!(benches, bench_exact_triangulation);
criterion_main!(benches);
