use criterion::{Criterion, black_box, criterion_group, criterion_main};
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

    let runtime_polygon = hypertri::PolygonInput::new(rational_spike.clone(), Vec::new());
    let runtime_options = hypertri::TriangulationOptions::default();
    c.bench_function("runtime_polygon_triangulation", |b| {
        b.iter(|| {
            hypertri::triangulate_polygon(black_box(&runtime_polygon), black_box(runtime_options))
                .unwrap()
        })
    });
    c.bench_function("runtime_polygon_triangulation_report", |b| {
        b.iter(|| {
            hypertri::triangulate_polygon_with_report(
                black_box(&runtime_polygon),
                black_box(runtime_options),
            )
            .unwrap()
        })
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
            // exact `hyperlimit` predicates.
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
            // edges with exact local Delaunay predicates.
            hypertri::cdt::constrained_delaunay(&separated_cycles, &separated_constraints).unwrap()
        })
    });

    let located_delaunay_points = (0..400_i64)
        .map(|index| {
            p(
                r((index % 20) * 100 + (index * 17) % 31),
                r((index / 20) * 100 + (index * 29) % 37),
            )
        })
        .collect::<Vec<_>>();
    c.bench_function("exact_delaunay_400_located_insertions", |b| {
        b.iter(|| hypertri::cdt::delaunay(&located_delaunay_points).unwrap())
    });

    let scattered_delaunay_points = (0..400_usize)
        .map(|position| located_delaunay_points[(position * 37) % 400].clone())
        .collect::<Vec<_>>();
    c.bench_function("exact_delaunay_400_scattered_insertions", |b| {
        b.iter(|| hypertri::cdt::delaunay(&scattered_delaunay_points).unwrap())
    });
    c.bench_function("exact_delaunay_spatial_400_located_input", |b| {
        b.iter(|| hypertri::cdt::delaunay_spatial(&located_delaunay_points).unwrap())
    });
    c.bench_function("exact_delaunay_spatial_400_scattered_input", |b| {
        b.iter(|| hypertri::cdt::delaunay_spatial(&scattered_delaunay_points).unwrap())
    });
    c.bench_function("exact_delaunay_64_located_insertions", |b| {
        b.iter(|| hypertri::cdt::delaunay(&located_delaunay_points[..64]).unwrap())
    });
    c.bench_function("exact_delaunay_spatial_64_located_input", |b| {
        b.iter(|| hypertri::cdt::delaunay_spatial(&located_delaunay_points[..64]).unwrap())
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

    let insertion_base = hypertri::nd::delaunay_complex(&nd_points[..5]).unwrap();
    let insertion_point = hypertri::nd::PointD::new(vec![q(1, 5), q(1, 5), q(1, 5), q(1, 5)]);
    c.bench_function("exact_nd_4d_oracle_insertion_report", |b| {
        b.iter(|| {
            let report = insertion_base
                .insert_point_oracle(insertion_point.clone())
                .unwrap();
            (
                report.old_cell_count(),
                report.new_cell_count(),
                report.conflict_cells().len(),
                report.boundary_facets().len(),
            )
        })
    });

    let flip_complex = hypertri::DelaunayComplex::from_parts(
        2,
        vec![
            hypertri::PointD::new(vec![r(0), r(0)]),
            hypertri::PointD::new(vec![r(1), r(0)]),
            hypertri::PointD::new(vec![r(1), r(1)]),
            hypertri::PointD::new(vec![r(0), r(1)]),
        ],
        vec![
            hypertri::Simplex::new(vec![0, 1, 2]),
            hypertri::Simplex::new(vec![0, 2, 3]),
        ],
    );
    let flip = hypertri::BistellarFlipD::new(vec![0, 1, 2, 3], vec![1, 3]);
    c.bench_function("exact_nd_bistellar_flip_validate", |b| {
        b.iter(|| {
            let report = flip_complex.validate_bistellar_flip(&flip);
            (
                report.is_valid(),
                report.p(),
                report.q(),
                report.removed_cells().len(),
                report.inserted_cells().len(),
                report.blocks_delaunay(),
            )
        })
    });
    c.bench_function("exact_nd_bistellar_flip_oracle_apply", |b| {
        b.iter(|| {
            let report = flip_complex.flip_oracle(&flip).unwrap();
            (
                report.validation().p(),
                report.validation().q(),
                report.result().cells().len(),
            )
        })
    });

    let mut tds = hypertri::TriangulationDataStructureD::new(3).unwrap();
    let v0 = tds
        .add_finite_vertex(hypertri::PointD::new(vec![r(0), r(0), r(0)]))
        .unwrap();
    let v1 = tds
        .add_finite_vertex(hypertri::PointD::new(vec![r(1), r(0), r(0)]))
        .unwrap();
    let v2 = tds
        .add_finite_vertex(hypertri::PointD::new(vec![r(0), r(1), r(0)]))
        .unwrap();
    let v3 = tds
        .add_finite_vertex(hypertri::PointD::new(vec![r(0), r(0), r(1)]))
        .unwrap();
    tds.add_cell(hypertri::Cell::new(
        vec![v0, v1, v2, v3],
        vec![None, None, None, None],
    ))
    .unwrap();

    c.bench_function("exact_nd_tds_combinatorial_validate", |b| {
        b.iter(|| tds.validate_combinatorial().unwrap())
    });

    c.bench_function("exact_nd_tds_combinatorial_report", |b| {
        b.iter(|| {
            let report = tds.validate_combinatorial_report();
            (
                report.is_valid(),
                report.facet_count(),
                report.boundary_facet_count(),
                report.interior_facet_count(),
                report.violations().len(),
            )
        })
    });

    c.bench_function("exact_nd_tds_manifold_report", |b| {
        b.iter(|| {
            let report = tds.validate_manifold_report(hypertri::TdsBoundaryPolicyD::AllowBoundary);
            (
                report.is_valid(),
                report.finite_facet_count(),
                report.boundary_facet_count(),
                report.interior_facet_count(),
                report.violations().len(),
            )
        })
    });

    c.bench_function("exact_nd_tds_geometric_report", |b| {
        b.iter(|| {
            let report = tds.validate_geometric_report();
            (
                report.is_valid(),
                report.finite_cell_count(),
                report.positive_orientation_count(),
                report.negative_orientation_count(),
                report.cospherical_boundary_count(),
                report.violations().len(),
            )
        })
    });
}

criterion_group!(benches, bench_exact_triangulation);
criterion_main!(benches);
