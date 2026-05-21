use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootKind, AlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    Constraint, EqualitySubstitution, Expr, IntervalBoxCertificationPackage, IsolatedRootInterval,
    PreparedProblem, PreparedSolverBlock, Problem, ProposalEngineKind, ProposalEnginePrecision,
    ProposalEngineReport, SolverConfig, SolverPoint2, SolverState, SparseResidualTerm, SymbolId,
    UnivariateResultantPairInput, VariableBall, analyze_sparse_bareiss_elimination_pattern,
    apply_equality_substitution_classes, arithmetic_algebraic_root_representations,
    audit_active_set, build_equality_substitution_classes, certify_affine_krawczyk_box,
    certify_candidate, certify_candidate_domains, certify_direct_univariate_quadratic_roots,
    certify_interval_box_candidate, certify_multivariate_quadratic_interval_candidate,
    certify_multivariate_quadratic_krawczyk_box, certify_quadratic_interval_candidate,
    certify_univariate_quadratic_alpha, certify_univariate_quadratic_krawczyk_box,
    compare_algebraic_root_representations, compare_algebraic_root_representations_with_refinement,
    context_from_problem, count_bernstein_univariate_polynomial_interval_roots,
    count_descartes_univariate_polynomial_roots, determinant_bareiss,
    eliminate_affine_rows_with_substitution_classes, isolate_univariate_polynomial_roots,
    propose_active_set_update, replay_dense_linear_residuals, replay_sparse_linear_residuals,
    report_lossy_adapter_only_candidate, represent_univariate_algebraic_roots,
    resultant_univariate_polynomials, run_active_set_update_loop,
    schedule_univariate_resultant_pairs, solve_damped_least_squares,
    solve_dense_linear_system_bareiss, solve_direct_affine_system,
    solve_direct_univariate_quadratic_equalities, solve_sparse_linear_system_bareiss,
    squared_distance_equation, subdivide_bernstein_univariate_polynomial_interval_roots,
    subresultant_chain_univariate_polynomials,
};

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn affine_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
    for index in 0..row_count {
        problem.add_constraint(Constraint::equality(
            format!("row {index}"),
            x.clone() * Expr::int(index as i64 + 1) - Expr::int((index as i64 + 1) * 2),
        ));
    }
    problem
}

fn affine_krawczyk_problem() -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
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

fn substitution_elimination_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let z = Expr::symbol(SymbolId(2), "z");
    let mut problem = Problem::default();
    problem.add_variable("x", r(0));
    problem.add_variable("y", r(0));
    problem.add_variable("z", r(0));
    problem.add_constraint(Constraint::equality(
        "y - x - 3",
        y.clone() - x.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality(
        "z - x + 2",
        z.clone() - x.clone() + Expr::int(2),
    ));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("reduced row {index}"),
            z.clone() * Expr::int(scale) + y.clone() * Expr::int(scale + 1) - Expr::int(scale),
        ));
    }
    problem
}

fn univariate_quadratic_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", r(3));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("quadratic {index}"),
            x.clone() * x.clone() * Expr::int(scale) - x.clone() * Expr::int(2 * scale)
                + Expr::int(scale),
        ));
    }
    problem
}

fn multivariate_quadratic_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(3));
    problem.add_variable("y", r(5));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("mixed quadratic {index}"),
            x.clone() * y.clone() * Expr::int(scale) + x.clone().powi(2) * Expr::int(scale + 1)
                - y.clone() * Expr::int(scale + 2)
                + Expr::int(scale),
        ));
    }
    problem
}

fn domain_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(9));
    problem.add_variable("y", r(3));
    for index in 0..row_count {
        problem.add_constraint(Constraint::equality(
            format!("domain row {index}"),
            (x.clone().sqrt()
                + (x.clone() / y.clone()).log10()
                + x.clone().ln()
                + y.clone().acos()
                + y.clone().atanh()
                + x.clone().acosh())
                / Expr::int(index as i64 + 1),
        ));
    }
    problem
}

fn sketch_problem(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let a = sketch.add_point2d(format!("a{index}"), r(index as i64), r(0));
        let b = sketch.add_point2d(format!("b{index}"), r(index as i64 + 3), r(4));
        let distance = sketch.add_distance(format!("d{index}"), r(5));
        let line = sketch.add_line_segment2(format!("line{index}"), a, b);
        let circle = sketch.add_circle2(format!("circle{index}"), a, distance);
        hypersolve::sketch_distance_builders::point_point_distance(
            &mut sketch,
            format!("distance {index}"),
            a,
            b,
            distance,
        );
        hypersolve::sketch_incidence_builders::point_on_circle(
            &mut sketch,
            format!("circle incidence {index}"),
            b,
            circle,
        );
        hypersolve::sketch_orientation_builders::horizontal(
            &mut sketch,
            format!("horizontal proposal {index}"),
            line,
        );
    }
    sketch
}

fn unary_endpoint_expression(row_count: usize) -> Expr {
    let mut expression = Expr::zero();
    for index in 1..=row_count {
        expression = expression
            + Expr::zero().sin()
            + Expr::zero().cos()
            + Expr::int(index as i64).sqrt()
            + Expr::int(1).log10()
            + Expr::int(1).ln()
            + Expr::zero().asin()
            + Expr::int(1).acos()
            + Expr::int(1).acosh()
            + Expr::zero().atanh();
    }
    expression
}

fn certification(c: &mut Criterion) {
    let problem = affine_problem(16);
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    c.bench_function("prepared_solver_block_affine_rows", |b| {
        b.iter(|| PreparedSolverBlock::new(&prepared))
    });
    let sketch = sketch_problem(16);
    c.bench_function("sketch_lower_to_problem", |b| {
        b.iter(|| sketch.lower_to_problem())
    });
    let form_handles = sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointPointDistance { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_distance_residual_forms", |b| {
        b.iter(|| {
            for handle in &form_handles {
                let _ = sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let krawczyk_problem = affine_krawczyk_problem();
    let krawczyk_prepared = PreparedProblem::new(&krawczyk_problem);
    let krawczyk_context = context_from_problem(&krawczyk_problem);
    c.bench_function("certify_affine_krawczyk_box", |b| {
        b.iter(|| {
            certify_affine_krawczyk_box(
                &krawczyk_prepared,
                &krawczyk_context,
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
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("solve_direct_affine_system", |b| {
        b.iter(|| solve_direct_affine_system(&krawczyk_prepared))
    });
    c.bench_function("determinant_bareiss", |b| {
        b.iter(|| determinant_bareiss(&[vec![r(2), r(1)], vec![r(1), r(-1)]], -64))
    });
    c.bench_function("solve_dense_linear_system_bareiss", |b| {
        b.iter(|| {
            solve_dense_linear_system_bareiss(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[r(5), r(1)],
                -64,
            )
        })
    });
    c.bench_function("solve_sparse_linear_system_bareiss", |b| {
        b.iter(|| {
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
        })
    });
    c.bench_function("analyze_sparse_bareiss_elimination_pattern", |b| {
        b.iter(|| {
            analyze_sparse_bareiss_elimination_pattern(
                3,
                3,
                &[
                    SparseResidualTerm {
                        row: 0,
                        column: 0,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 0,
                        column: 2,
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
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 2,
                        column: 2,
                        coefficient: r(1),
                    },
                ],
                -64,
            )
        })
    });
    c.bench_function("resultant_univariate_polynomials", |b| {
        b.iter(|| resultant_univariate_polynomials(&[r(-1), r(0), r(1)], &[r(-2), r(1)], -64))
    });
    c.bench_function("subresultant_chain_univariate_polynomials", |b| {
        b.iter(|| {
            subresultant_chain_univariate_polynomials(&[r(2), r(-3), r(1)], &[r(-1), r(1)], -64)
        })
    });
    c.bench_function("schedule_univariate_resultant_pairs", |b| {
        b.iter(|| {
            schedule_univariate_resultant_pairs(
                &[
                    UnivariateResultantPairInput {
                        pair_index: 0,
                        left_coefficients: vec![r(-1), r(1)],
                        right_coefficients: vec![r(-2), r(1)],
                    },
                    UnivariateResultantPairInput {
                        pair_index: 1,
                        left_coefficients: vec![r(-1), r(1)],
                        right_coefficients: vec![r(-1), r(0), r(1)],
                    },
                ],
                -64,
            )
        })
    });
    c.bench_function("replay_dense_linear_residuals", |b| {
        b.iter(|| {
            replay_dense_linear_residuals(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[r(5), r(1)],
                &[r(2), r(1)],
                -64,
            )
        })
    });
    c.bench_function("replay_sparse_linear_residuals", |b| {
        b.iter(|| {
            replay_sparse_linear_residuals(
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
                &[r(2), r(1)],
                -64,
            )
        })
    });
    let elimination_problem = substitution_elimination_problem(16);
    let elimination_prepared = PreparedProblem::new(&elimination_problem);
    let substitutions = vec![
        EqualitySubstitution {
            constraint_index: 0,
            left: SymbolId(1),
            right: SymbolId(0),
            offset: r(3),
        },
        EqualitySubstitution {
            constraint_index: 1,
            left: SymbolId(2),
            right: SymbolId(0),
            offset: r(-2),
        },
    ];
    let substitution_classes = build_equality_substitution_classes(&substitutions).unwrap();
    c.bench_function("eliminate_affine_rows_with_substitution_classes", |b| {
        b.iter(|| {
            eliminate_affine_rows_with_substitution_classes(
                &elimination_prepared,
                &substitution_classes,
            )
        })
    });
    c.bench_function("apply_equality_substitution_classes", |b| {
        b.iter(|| {
            let mut context = hypersolve::EvaluationContext::default();
            context.bind(SymbolId(2), r(8));
            apply_equality_substitution_classes(&mut context, &substitution_classes)
        })
    });
    let quadratic_problem = univariate_quadratic_problem(16);
    c.bench_function("prepared_univariate_quadratic_rows", |b| {
        b.iter(|| PreparedProblem::new(&quadratic_problem))
    });
    let multivariate_quadratic = multivariate_quadratic_problem(16);
    c.bench_function("prepared_multivariate_quadratic_rows", |b| {
        b.iter(|| PreparedProblem::new(&multivariate_quadratic))
    });
    let prepared_multivariate_quadratic = PreparedProblem::new(&multivariate_quadratic);
    let multivariate_quadratic_context = context_from_problem(&multivariate_quadratic);
    let prepared_quadratic = PreparedProblem::new(&quadratic_problem);
    let quadratic_context = context_from_problem(&quadratic_problem);
    c.bench_function("solve_direct_univariate_quadratic_rows", |b| {
        b.iter(|| solve_direct_univariate_quadratic_equalities(&prepared_quadratic))
    });
    c.bench_function("certify_direct_univariate_quadratic_roots", |b| {
        b.iter(|| {
            certify_direct_univariate_quadratic_roots(&prepared_quadratic, &quadratic_context)
        })
    });
    c.bench_function("isolate_univariate_polynomial_roots_sturm", |b| {
        b.iter(|| {
            isolate_univariate_polynomial_roots(
                &prepared_quadratic,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("represent_univariate_algebraic_roots", |b| {
        b.iter(|| {
            represent_univariate_algebraic_roots(
                &prepared_quadratic,
                hypersolve::RootIsolationConfig::default(),
            )
        })
    });
    let represented_roots = represent_univariate_algebraic_roots(
        &prepared_quadratic,
        hypersolve::RootIsolationConfig::default(),
    );
    c.bench_function("compare_algebraic_root_representations", |b| {
        b.iter(|| {
            compare_algebraic_root_representations(
                &represented_roots[0].roots[0],
                &represented_roots[0].roots[1],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    let sqrt_two = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![r(-2), Real::zero(), Real::one()],
        interval: IsolatedRootInterval {
            lower: r(1),
            upper: r(2),
            exact_root: None,
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::IsolatingInterval,
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
    c.bench_function(
        "compare_algebraic_root_representations_with_refinement",
        |b| {
            b.iter(|| {
                compare_algebraic_root_representations_with_refinement(
                    &sqrt_two,
                    &sqrt_three,
                    AlgebraicRootRefinementComparisonConfig {
                        max_refinement_rounds: 8,
                        steps_per_round: 1,
                        ..AlgebraicRootRefinementComparisonConfig::default()
                    },
                )
            })
        },
    );
    let rational_two = AlgebraicRootRepresentation {
        constraint_index: 2,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![r(-2), Real::one()],
        interval: IsolatedRootInterval {
            lower: r(2),
            upper: r(2),
            exact_root: Some(r(2)),
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    let rational_three = AlgebraicRootRepresentation {
        constraint_index: 3,
        polynomial_coefficients: vec![r(-3), Real::one()],
        interval: IsolatedRootInterval {
            lower: r(3),
            upper: r(3),
            exact_root: Some(r(3)),
            distinct_root_count: 1,
        },
        ..rational_two.clone()
    };
    c.bench_function("arithmetic_algebraic_root_representations", |b| {
        b.iter(|| {
            arithmetic_algebraic_root_representations(
                &rational_two,
                Some(&rational_three),
                AlgebraicRootArithmeticOp::Multiply,
            )
        })
    });
    c.bench_function("count_descartes_univariate_polynomial_roots", |b| {
        b.iter(|| {
            count_descartes_univariate_polynomial_roots(
                &prepared_quadratic,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function(
        "count_bernstein_univariate_polynomial_interval_roots",
        |b| {
            b.iter(|| {
                count_bernstein_univariate_polynomial_interval_roots(
                    &prepared_quadratic,
                    r(0),
                    r(4),
                    hyperlimit::PredicatePolicy::default(),
                )
            })
        },
    );
    c.bench_function(
        "subdivide_bernstein_univariate_polynomial_interval_roots",
        |b| {
            b.iter(|| {
                subdivide_bernstein_univariate_polynomial_interval_roots(
                    &prepared_quadratic,
                    r(0),
                    r(4),
                    hypersolve::BernsteinSubdivisionConfig {
                        policy: hyperlimit::PredicatePolicy::default(),
                        max_depth: 8,
                    },
                )
            })
        },
    );
    c.bench_function("certify_quadratic_interval_rows", |b| {
        b.iter(|| {
            certify_quadratic_interval_candidate(
                &prepared_quadratic,
                &quadratic_context,
                &[VariableBall {
                    symbol: SymbolId(0),
                    radius: r(1),
                }],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_interval_box_candidate_report", |b| {
        b.iter(|| {
            certify_interval_box_candidate(
                &prepared_quadratic,
                &quadratic_context,
                &[VariableBall {
                    symbol: SymbolId(0),
                    radius: r(1),
                }],
                IntervalBoxCertificationPackage::UnivariateQuadratic,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_univariate_quadratic_alpha_rows", |b| {
        b.iter(|| {
            certify_univariate_quadratic_alpha(
                &prepared_quadratic,
                &quadratic_context,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_univariate_quadratic_krawczyk_rows", |b| {
        b.iter(|| {
            certify_univariate_quadratic_krawczyk_box(
                &prepared_quadratic,
                &quadratic_context,
                &[VariableBall {
                    symbol: SymbolId(0),
                    radius: r(0),
                }],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_multivariate_quadratic_interval_rows", |b| {
        b.iter(|| {
            certify_multivariate_quadratic_interval_candidate(
                &prepared_multivariate_quadratic,
                &multivariate_quadratic_context,
                &[
                    VariableBall {
                        symbol: SymbolId(0),
                        radius: r(1),
                    },
                    VariableBall {
                        symbol: SymbolId(1),
                        radius: r(1),
                    },
                ],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_multivariate_quadratic_krawczyk_rows", |b| {
        b.iter(|| {
            certify_multivariate_quadratic_krawczyk_box(
                &prepared_multivariate_quadratic,
                &multivariate_quadratic_context,
                &[
                    VariableBall {
                        symbol: SymbolId(0),
                        radius: r(0),
                    },
                    VariableBall {
                        symbol: SymbolId(1),
                        radius: r(0),
                    },
                ],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("prepared_quadratic_candidate_replay", |b| {
        b.iter(|| {
            for row in 0..prepared_multivariate_quadratic.problem().constraints.len() {
                let _ = prepared_multivariate_quadratic
                    .evaluate_constraint_residual(row, &multivariate_quadratic_context);
            }
        })
    });
    c.bench_function("certify_affine_candidate_exact", |b| {
        b.iter(|| certify_candidate(&prepared, &context))
    });
    c.bench_function("audit_active_set", |b| {
        b.iter(|| {
            audit_active_set(
                &prepared,
                &context,
                hypersolve::CandidateCertificationConfig::default(),
            )
        })
    });
    c.bench_function("propose_active_set_update", |b| {
        b.iter(|| {
            propose_active_set_update(
                &prepared,
                &context,
                hypersolve::CandidateCertificationConfig::default(),
            )
        })
    });
    let active_mask = prepared
        .problem()
        .constraints
        .iter()
        .map(|constraint| constraint.active)
        .collect::<Vec<_>>();
    c.bench_function("run_active_set_update_loop", |b| {
        b.iter(|| {
            run_active_set_update_loop(
                &prepared,
                &context,
                &active_mask,
                hypersolve::ActiveSetLoopConfig::default(),
            )
        })
    });
    c.bench_function("report_lossy_adapter_only_candidate", |b| {
        b.iter(|| {
            report_lossy_adapter_only_candidate(
                &prepared,
                ProposalEngineReport {
                    requested: ProposalEngineKind::DampedLeastSquares,
                    used: Some(ProposalEngineKind::DampedLeastSquares),
                    precision: ProposalEnginePrecision::LossyF64,
                    supported: true,
                },
            )
        })
    });
    c.bench_function("solve_levenberg_marquardt_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    proposal_engine: ProposalEngineKind::LevenbergMarquardt,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_least_squares_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_dogleg_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::Dogleg,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_powell_hybrid_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::PowellHybrid,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_bfgs_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::Bfgs,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_sqp_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::Sqp,
                    ..SolverConfig::default()
                },
            })
        })
    });
    let domain_problem = domain_problem(16);
    let domain_context = context_from_problem(&domain_problem);
    c.bench_function("certify_candidate_domains", |b| {
        b.iter(|| {
            certify_candidate_domains(
                &domain_problem,
                &domain_context,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("simplify_unary_endpoint_expression", |b| {
        let expression = unary_endpoint_expression(16);
        b.iter(|| expression.clone().simplify())
    });

    let substitutions = (0..16)
        .map(|index| EqualitySubstitution {
            constraint_index: index,
            left: SymbolId(index as u32 + 1),
            right: SymbolId(index as u32),
            offset: r(index as i64),
        })
        .collect::<Vec<_>>();
    c.bench_function("build_equality_substitution_classes_exact", |b| {
        b.iter(|| build_equality_substitution_classes(&substitutions))
    });

    c.bench_function("domain_geometry_squared_distance_build", |b| {
        b.iter(|| {
            let mut problem = Problem::default();
            let ax = problem.add_variable("ax", r(0));
            let ay = problem.add_variable("ay", r(0));
            let bx = problem.add_variable("bx", r(3));
            let by = problem.add_variable("by", r(4));
            problem.add_constraint(squared_distance_equation(
                "3-4-5",
                SolverPoint2::new(ax, ay),
                SolverPoint2::new(bx, by),
                r(25),
            ));
            problem
        })
    });
}

criterion_group!(benches, certification);
criterion_main!(benches);
