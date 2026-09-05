use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::hint::black_box;

use hyperreal::{Rational, Real};
use hypersolve::{
    AlgebraicRootRationalImageStatus, AlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    BatchPredicateScheduleConfig, BezierPowerBasisSubstitutionConfig, Constraint,
    CurveIntersectionResultantConfig, CurveResultantParameter, Expr,
    IntervalBoxCertificationPackage, IsolatedRootInterval, PolynomialCurvePoint2,
    PolynomialParametricCurve2, Problem, ProposalEngineKind, RootIsolationConfig, SolverConfig,
    SolverState, SparseResidualTerm, SymbolId, VariableBall, analyze_exact_affine_rank,
    audit_active_set, certify_affine_krawczyk_box, certify_candidate, certify_candidate_batch,
    certify_candidate_domains, certify_interval_box_candidate, certify_sketch_construction,
    compare_algebraic_root_representations, compare_algebraic_root_representations_by_difference,
    compare_algebraic_root_representations_with_refinement, context_from_problem,
    determinant_bareiss, diagnose_failed_constraints, evaluate_polynomial_at_algebraic_root,
    evaluate_rational_expression_at_algebraic_root, isolate_univariate_polynomial_roots,
    polynomial_has_one_distinct_root_in_open_interval, preflight_sketch_degeneracies,
    preflight_sketch_entity_domains, preflight_sketch_parameter_domains,
    represent_univariate_algebraic_roots, represented_root_sign,
    resultant_parametric_curve_intersection, resultant_univariate_polynomials,
    schedule_candidate_batch_predicates, solve_damped_least_squares,
    solve_dense_linear_system_bareiss, solve_dense_linear_system_bareiss_multi_rhs,
    solve_direct_affine_system, solve_sparse_linear_system_bareiss, substitute_bezier_power_basis,
    transform_algebraic_root_rational_image, transform_algebraic_root_rational_images,
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
        let analysis = lowering.problem.analyze();
        black_box(certify_candidate(
            &analysis,
            &context_from_problem(&lowering.problem),
        ));
    })
}

fn trace_candidate_certification() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("candidate_batch_and_active_set", || {
        let problem = affine_problem(8);
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        black_box(certify_candidate(&analysis, &context));
        black_box(certify_candidate_batch(
            &analysis,
            &[context.clone(), context.clone(), context.clone()],
        ));
        black_box(
            schedule_candidate_batch_predicates(
                &analysis,
                3,
                BatchPredicateScheduleConfig {
                    max_rows_per_work_item: 2,
                },
            )
            .expect("trace batch schedule should be valid"),
        );
        black_box(audit_active_set(
            &analysis,
            &context,
            hypersolve::CandidateCertificationConfig::default(),
        ));
        black_box(diagnose_failed_constraints(&analysis, &context));
    })
}

fn trace_direct_and_fraction_free_linear() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("direct_and_fraction_free_linear", || {
        let problem = affine_krawczyk_problem();
        let analysis = problem.analyze();
        black_box(solve_direct_affine_system(&analysis));
        black_box(analyze_exact_affine_rank(&analysis, -64));
        black_box(
            determinant_bareiss(&[vec![r(2), r(1)], vec![r(1), r(-1)]], -64)
                .expect("trace matrix should have a determinant"),
        );
        black_box(
            solve_dense_linear_system_bareiss(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[r(5), r(1)],
                -64,
                hyperlimit::PredicatePolicy::STRICT,
            )
            .expect("trace dense system should be nonsingular"),
        );
        black_box(
            solve_dense_linear_system_bareiss_multi_rhs(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[vec![r(5), r(1)], vec![r(0), r(3)]],
                -64,
                hyperlimit::PredicatePolicy::STRICT,
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
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        black_box(isolate_univariate_polynomial_roots(
            &analysis,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        black_box(represent_univariate_algebraic_roots(
            &analysis,
            RootIsolationConfig::default(),
        ));
        black_box(certify_interval_box_candidate(
            &analysis,
            &context,
            &[VariableBall {
                symbol: SymbolId(0),
                radius: r(1),
            }],
            IntervalBoxCertificationPackage::UnivariateQuadratic,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        let mut monotone_degree_16 = vec![Real::zero(); 17];
        monotone_degree_16[0] = Real::new(Rational::fraction(-1, 2).unwrap());
        monotone_degree_16[1] = Real::one();
        monotone_degree_16[16] = Real::one();
        assert_eq!(
            black_box(polynomial_has_one_distinct_root_in_open_interval(
                &monotone_degree_16,
                &Real::zero(),
                &Real::one(),
                hyperlimit::PredicatePolicy::STRICT,
            )),
            Some(true)
        );
        assert_eq!(
            black_box(polynomial_has_one_distinct_root_in_open_interval(
                &[r(1), r(-4), r(4)],
                &Real::zero(),
                &Real::one(),
                hyperlimit::PredicatePolicy::STRICT,
            )),
            Some(true)
        );
    })
}

fn trace_algebraic_evaluation() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("algebraic_evaluation", || {
        let root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![r(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::one(),
                upper: r(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let dense_polynomial = vec![Real::one(); 17];
        black_box(evaluate_polynomial_at_algebraic_root(
            &root,
            &dense_polynomial,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        black_box(evaluate_rational_expression_at_algebraic_root(
            &root,
            &[Real::one(), Real::one()],
            &[r(2), Real::one()],
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));

        let exact_value = r(2).sqrt().expect("positive radicand");
        let exact_point = AlgebraicRootRepresentation {
            constraint_index: 1,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-exact_value.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: exact_value.clone(),
                upper: exact_value.clone(),
                exact_root: Some(exact_value),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        black_box(evaluate_polynomial_at_algebraic_root(
            &exact_point,
            &[Real::one(), Real::one()],
            hyperlimit::PredicatePolicy::STRICT,
        ));
    })
}

fn trace_algebraic_rational_images() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("algebraic_rational_images", || {
        let root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![r(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::one(),
                upper: r(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };

        let ordinary = black_box(transform_algebraic_root_rational_image(
            &root,
            &[Real::zero(), Real::one()],
            &[Real::one(), Real::one()],
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        assert_eq!(
            ordinary.status,
            AlgebraicRootRationalImageStatus::Transformed
        );

        let reduced = black_box(transform_algebraic_root_rational_image(
            &root,
            &[Real::zero(), Real::zero(), Real::one()],
            &[Real::one()],
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        assert_eq!(
            reduced
                .representation
                .as_ref()
                .and_then(AlgebraicRootRepresentation::exact_point_witness),
            Some(&r(2))
        );

        let numerators = [
            [Real::zero(), Real::one()],
            [Real::one(), Real::one()],
            [r(-1), r(2)],
            [r(3), r(-1)],
        ];
        let numerator_refs = [
            numerators[0].as_slice(),
            numerators[1].as_slice(),
            numerators[2].as_slice(),
            numerators[3].as_slice(),
        ];
        let batch = black_box(transform_algebraic_root_rational_images(
            &root,
            numerator_refs,
            &[r(2), r(-2), Real::one()],
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        assert!(
            batch
                .iter()
                .all(|report| report.status == AlgebraicRootRationalImageStatus::Transformed)
        );

        let pole = black_box(transform_algebraic_root_rational_image(
            &root,
            &[Real::one()],
            &root.polynomial_coefficients,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ));
        assert_eq!(
            pole.status,
            AlgebraicRootRationalImageStatus::CertifiedZeroDenominator
        );
    })
}

fn trace_algebraic_comparison() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("algebraic_comparison", || {
        let sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![r(-2), Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower: Real::one(),
                upper: r(2),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        let sqrt_three = AlgebraicRootRepresentation {
            constraint_index: 1,
            polynomial_coefficients: vec![r(-3), Real::zero(), Real::one()],
            ..sqrt_two.clone()
        };
        black_box(compare_algebraic_root_representations(
            &sqrt_two,
            &sqrt_three,
            hyperlimit::PredicatePolicy::STRICT,
        ));
        black_box(compare_algebraic_root_representations_with_refinement(
            &sqrt_two,
            &sqrt_three,
            AlgebraicRootRefinementComparisonConfig {
                policy: hyperlimit::PredicatePolicy::STRICT,
                max_refinement_rounds: 4,
                steps_per_round: 1,
            },
        ));
        black_box(compare_algebraic_root_representations_by_difference(
            &sqrt_two,
            &sqrt_three,
            AlgebraicRootRefinementComparisonConfig {
                policy: hyperlimit::PredicatePolicy::STRICT,
                max_refinement_rounds: 0,
                steps_per_round: 1,
            },
        ));
        black_box(represented_root_sign(
            &sqrt_two,
            hyperlimit::PredicatePolicy::STRICT,
        ));
        let exact_sqrt_two_value = r(2).sqrt().expect("positive exact square root");
        let exact_sqrt_two = AlgebraicRootRepresentation {
            constraint_index: 2,
            polynomial_coefficients: vec![-exact_sqrt_two_value.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: exact_sqrt_two_value.clone(),
                upper: exact_sqrt_two_value.clone(),
                exact_root: Some(exact_sqrt_two_value),
                distinct_root_count: 1,
            },
            ..sqrt_two.clone()
        };
        black_box(compare_algebraic_root_representations(
            &exact_sqrt_two,
            &sqrt_three,
            hyperlimit::PredicatePolicy::STRICT,
        ));
        black_box(compare_algebraic_root_representations_by_difference(
            &exact_sqrt_two,
            &sqrt_two,
            AlgebraicRootRefinementComparisonConfig {
                policy: hyperlimit::PredicatePolicy::STRICT,
                max_refinement_rounds: 0,
                steps_per_round: 1,
            },
        ));
        black_box(represented_root_sign(
            &exact_sqrt_two,
            hyperlimit::PredicatePolicy::STRICT,
        ));
    })
}

fn trace_krawczyk_certification() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("affine_krawczyk_certification", || {
        let problem = affine_krawczyk_problem();
        let analysis = problem.analyze();
        let context = context_from_problem(&problem);
        black_box(certify_affine_krawczyk_box(
            &analysis,
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
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
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
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
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

    if std::env::var_os("HYPERSOLVE_SKIP_BENCHMARK_REPORTS").is_none() {
        fs::write("dispatch_trace.md", out).expect("dispatch trace report should be writable");
    }
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
        ("algebraic_evaluation", trace_algebraic_evaluation()),
        (
            "algebraic_rational_images",
            trace_algebraic_rational_images(),
        ),
        ("algebraic_comparison", trace_algebraic_comparison()),
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
