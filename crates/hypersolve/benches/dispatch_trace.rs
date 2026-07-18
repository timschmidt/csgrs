use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::hint::black_box;

use hyperreal::{Rational, Real};
use hypersolve::{
    BatchPredicateScheduleConfig, BezierPowerBasisSubstitutionConfig, Constraint,
    CurveIntersectionResultantConfig, CurveResultantParameter, Expr,
    IntervalBoxCertificationPackage, PolynomialCurvePoint2, PolynomialParametricCurve2,
    PreparedProblem, Problem, ProposalEngineKind, RootIsolationConfig, SolverConfig, SolverState,
    SparseResidualTerm, SymbolId, VariableBall, analyze_exact_affine_rank, audit_active_set,
    certify_affine_krawczyk_box, certify_candidate, certify_candidate_batch,
    certify_candidate_domains, certify_interval_box_candidate, certify_sketch_construction,
    context_from_problem, determinant_bareiss, diagnose_failed_constraints,
    isolate_univariate_polynomial_roots, preflight_sketch_degeneracies,
    preflight_sketch_entity_domains, preflight_sketch_parameter_domains,
    represent_univariate_algebraic_roots, resultant_parametric_curve_intersection,
    resultant_univariate_polynomials, schedule_candidate_batch_predicates,
    solve_damped_least_squares, solve_dense_linear_system_bareiss,
    solve_dense_linear_system_bareiss_multi_rhs, solve_direct_affine_system,
    solve_sparse_linear_system_bareiss, substitute_bezier_power_basis,
};

const MARKER_LAYER: &str = "hypersolve-benchmark";

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn affine_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("affine trace row {index}"),
            x.clone() * Expr::int(scale) - Expr::int(2 * scale),
        ));
    }
    problem
}

fn affine_krawczyk_problem() -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(3));
    problem.add_variable("y", r(2));
    problem.add_constraint(Constraint::equality(
        "x plus y minus five",
        x.clone() + y.clone() - Expr::int(5),
    ));
    problem.add_constraint(Constraint::equality(
        "x minus y minus one",
        x - y - Expr::int(1),
    ));
    problem
}

fn quadratic_problem() -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
    problem.add_constraint(Constraint::equality(
        "x squared minus two",
        x.clone() * x - Expr::int(2),
    ));
    problem
}

fn trace_case<T>(
    name: &'static str,
    workload: impl FnOnce() -> T,
) -> hyperreal::dispatch_trace::TraceSnapshot {
    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(|| {
        hyperreal::dispatch_trace::record(MARKER_LAYER, name, "recorded-workload");
        workload()
    });
    black_box(result);

    let snapshot = hyperreal::dispatch_trace::take_trace();
    let dependency_dispatch = snapshot
        .dispatch
        .iter()
        .filter(|entry| entry.layer != MARKER_LAYER)
        .map(|entry| entry.count)
        .sum::<u64>();
    assert!(
        dependency_dispatch > 0 || snapshot.rational.temporary_rationals > 0,
        "{name} recorded no dependency dispatch or rational work"
    );
    snapshot
}

fn trace_sketch_lowering() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("sketch_lowering_and_preflight", || {
        let mut sketch = hypersolve::SketchSolveProblem::new();
        let a = sketch.add_point2d("a", r(0), r(0));
        let b = sketch.add_point2d("b", r(3), r(4));
        let distance = sketch.add_distance("five", r(5));
        let line = sketch.add_line_segment2("line", a, b);
        let circle = sketch.add_circle2("circle", a, distance);
        hypersolve::sketch_distance_builders::point_point_distance(
            &mut sketch,
            "distance",
            a,
            b,
            distance,
        );
        hypersolve::sketch_incidence_builders::point_on_circle(&mut sketch, "incidence", b, circle);
        hypersolve::sketch_orientation_builders::horizontal(&mut sketch, "horizontal", line);

        black_box(preflight_sketch_parameter_domains(&sketch));
        black_box(preflight_sketch_entity_domains(&sketch));
        black_box(preflight_sketch_degeneracies(&sketch));
        black_box(certify_sketch_construction(&sketch));
        let lowering = sketch.lower_to_problem();
        let prepared = PreparedProblem::new(&lowering.problem);
        black_box(certify_candidate(
            &prepared,
            &context_from_problem(&lowering.problem),
        ));
    })
}

fn trace_candidate_certification() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("candidate_batch_and_active_set", || {
        let problem = affine_problem(8);
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        black_box(certify_candidate(&prepared, &context));
        black_box(certify_candidate_batch(
            &prepared,
            &[context.clone(), context.clone(), context.clone()],
        ));
        black_box(
            schedule_candidate_batch_predicates(
                &prepared,
                3,
                BatchPredicateScheduleConfig {
                    max_rows_per_work_item: 2,
                },
            )
            .expect("trace batch schedule should be valid"),
        );
        black_box(audit_active_set(
            &prepared,
            &context,
            hypersolve::CandidateCertificationConfig::default(),
        ));
        black_box(diagnose_failed_constraints(&prepared, &context));
    })
}

fn trace_direct_and_fraction_free_linear() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("direct_and_fraction_free_linear", || {
        let problem = affine_krawczyk_problem();
        let prepared = PreparedProblem::new(&problem);
        black_box(solve_direct_affine_system(&prepared));
        black_box(analyze_exact_affine_rank(&prepared, -64));
        black_box(
            determinant_bareiss(&[vec![r(2), r(1)], vec![r(1), r(-1)]], -64)
                .expect("trace matrix should have a determinant"),
        );
        black_box(
            solve_dense_linear_system_bareiss(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[r(5), r(1)],
                -64,
            )
            .expect("trace dense system should be nonsingular"),
        );
        black_box(
            solve_dense_linear_system_bareiss_multi_rhs(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[vec![r(5), r(1)], vec![r(0), r(3)]],
                -64,
            )
            .expect("trace multi-right-hand-side system should be nonsingular"),
        );
        black_box(
            solve_sparse_linear_system_bareiss(
                2,
                2,
                &[
                    SparseResidualTerm {
                        row: 0,
                        column: 0,
                        coefficient: r(2),
                    },
                    SparseResidualTerm {
                        row: 0,
                        column: 1,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 0,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 1,
                        coefficient: r(-1),
                    },
                ],
                &[r(5), r(1)],
                -64,
            )
            .expect("trace sparse system should be nonsingular"),
        );
    })
}

fn trace_resultants_and_curves() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("resultants_and_curve_substitution", || {
        black_box(
            resultant_univariate_polynomials(&[r(-1), r(0), r(1)], &[r(-2), r(1)], -64)
                .expect("trace polynomial pair should have a resultant"),
        );

        let parabola = PolynomialParametricCurve2::new(vec![r(0), r(1)], vec![r(0), r(0), r(1)]);
        let horizontal = PolynomialParametricCurve2::new(vec![r(0), r(1)], vec![r(1)]);
        black_box(resultant_parametric_curve_intersection(
            &parabola,
            &horizontal,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig::default(),
        ));

        black_box(substitute_bezier_power_basis(
            &[
                PolynomialCurvePoint2::new(r(0), r(0)),
                PolynomialCurvePoint2::new(r(1), r(2)),
                PolynomialCurvePoint2::new(r(2), r(0)),
            ],
            BezierPowerBasisSubstitutionConfig::default(),
        ));
    })
}

fn trace_roots_and_interval_certification() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("roots_and_interval_certification", || {
        let problem = quadratic_problem();
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        black_box(isolate_univariate_polynomial_roots(
            &prepared,
            hyperlimit::PredicatePolicy,
        ));
        black_box(represent_univariate_algebraic_roots(
            &prepared,
            RootIsolationConfig::default(),
        ));
        black_box(certify_interval_box_candidate(
            &prepared,
            &context,
            &[VariableBall {
                symbol: SymbolId(0),
                radius: r(1),
            }],
            IntervalBoxCertificationPackage::UnivariateQuadratic,
            hyperlimit::PredicatePolicy,
        ));
    })
}

fn trace_krawczyk_certification() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("affine_krawczyk_certification", || {
        let problem = affine_krawczyk_problem();
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        black_box(certify_affine_krawczyk_box(
            &prepared,
            &context,
            &[
                VariableBall {
                    symbol: SymbolId(0),
                    radius: r(1),
                },
                VariableBall {
                    symbol: SymbolId(1),
                    radius: r(0),
                },
            ],
            hyperlimit::PredicatePolicy,
        ));
    })
}

fn trace_domain_certification() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("domain_certification", || {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", r(4));
        problem.add_constraint(Constraint::equality(
            "sqrt plus logarithm",
            x.clone().sqrt() + x.ln(),
        ));
        black_box(certify_candidate_domains(
            &problem,
            &context_from_problem(&problem),
            hyperlimit::PredicatePolicy,
        ));
    })
}

fn trace_lossy_proposal_boundary() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("lossy_proposal_exact_replay_boundary", || {
        black_box(solve_damped_least_squares(SolverState {
            problem: affine_problem(4),
            config: SolverConfig {
                max_iterations: 2,
                proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                ..SolverConfig::default()
            },
        }));
    })
}

fn write_report(rows: &BTreeMap<&'static str, hyperreal::dispatch_trace::TraceSnapshot>) {
    let mut out = String::from(
        "# Hypersolve Dispatch Trace\n\n\
Generated by `cargo bench --bench dispatch_trace --features dispatch-trace`. Each workload is run once outside Criterion: use `certification` for timings and this report to correlate those families with exact dependency paths. The harness rejects any row that records only its Hypersolve marker.\n\n\
## Correlation Summary\n\n\
| Workload | Dependency Dispatch | Predicate | Linear Algebra | Exact Reducers | Approximation | Refinement | Cache | Fallback/Abort | Rational Temporaries | Rational Reductions | Rational GCDs |\n\
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n",
    );

    for (name, snapshot) in rows {
        let correlation = snapshot.correlation_summary();
        let dependency_dispatch = snapshot
            .dispatch
            .iter()
            .filter(|entry| entry.layer != MARKER_LAYER)
            .map(|entry| entry.count)
            .sum::<u64>();
        writeln!(
            out,
            "| `{name}` | {dependency_dispatch} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
            correlation.predicate_events,
            correlation.linear_algebra_events,
            correlation.exact_reducer_events,
            correlation.approximation_events,
            correlation.refinement_events,
            correlation.cache_events,
            correlation.fallback_or_abort_events,
            correlation.rational_temporaries,
            correlation.rational_reductions,
            correlation.rational_gcds,
        )
        .expect("writing a String cannot fail");
    }

    out.push_str("\n## Dispatch Paths\n\n| Workload | Layer | Operation | Path | Count |\n| --- | --- | --- | --- | ---: |\n");
    for (name, snapshot) in rows {
        for entry in &snapshot.dispatch {
            writeln!(
                out,
                "| `{name}` | `{}` | `{}` | `{}` | {} |",
                entry.layer, entry.operation, entry.path, entry.count,
            )
            .expect("writing a String cannot fail");
        }
    }

    fs::write("dispatch_trace.md", out).expect("dispatch trace report should be writable");
}

fn main() {
    let rows = BTreeMap::from([
        ("sketch_lowering_and_preflight", trace_sketch_lowering()),
        (
            "candidate_batch_and_active_set",
            trace_candidate_certification(),
        ),
        (
            "direct_and_fraction_free_linear",
            trace_direct_and_fraction_free_linear(),
        ),
        (
            "resultants_and_curve_substitution",
            trace_resultants_and_curves(),
        ),
        (
            "roots_and_interval_certification",
            trace_roots_and_interval_certification(),
        ),
        (
            "affine_krawczyk_certification",
            trace_krawczyk_certification(),
        ),
        ("domain_certification", trace_domain_certification()),
        (
            "lossy_proposal_exact_replay_boundary",
            trace_lossy_proposal_boundary(),
        ),
    ]);
    write_report(&rows);

    for (name, snapshot) in rows {
        println!("{name}: {:?}", snapshot.correlation_summary());
    }
}
