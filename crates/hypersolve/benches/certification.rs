use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, EqualitySubstitution, Expr, PreparedProblem, PreparedSolverBlock, Problem,
    ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport, RectangularRegion,
    SolverPoint2, SymbolId, VariableBall, bezier_offset_sample_constraints,
    build_equality_substitution_classes, center_clearance_squared_constraint,
    certify_affine_krawczyk_box, certify_candidate, certify_candidate_domains,
    certify_multivariate_quadratic_interval_candidate, certify_quadratic_interval_candidate,
    certify_univariate_quadratic_alpha, certify_univariate_quadratic_krawczyk_box,
    context_from_problem, differential_pair_skew_equation,
    eliminate_affine_rows_with_substitution_classes, rectangular_difference_area_equation,
    report_lossy_adapter_only_candidate, solve_direct_univariate_quadratic_equalities,
    squared_distance_equation,
};

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn rect(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> RectangularRegion {
    RectangularRegion::new(
        hyperlimit::Point2::new(r(min_x), r(min_y)),
        hyperlimit::Point2::new(r(max_x), r(max_y)),
    )
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
            (x.clone().sqrt() + (x.clone() / y.clone()).log10()) / Expr::int(index as i64 + 1),
        ));
    }
    problem
}

fn unary_endpoint_expression(row_count: usize) -> Expr {
    let mut expression = Expr::zero();
    for index in 1..=row_count {
        expression = expression
            + Expr::zero().sin()
            + Expr::zero().cos()
            + Expr::int(index as i64).sqrt()
            + Expr::int(1).log10();
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
    c.bench_function("domain_pcb_clearance_certification", |b| {
        let mut problem = Problem::default();
        let ax = problem.add_variable("ax", r(0));
        let ay = problem.add_variable("ay", r(0));
        let bx = problem.add_variable("bx", r(6));
        let by = problem.add_variable("by", r(8));
        problem.add_constraint(center_clearance_squared_constraint(
            "center clearance",
            SolverPoint2::new(ax, ay),
            SolverPoint2::new(bx, by),
            r(10),
        ));
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        b.iter(|| certify_candidate(&prepared, &context))
    });
    c.bench_function("domain_toolpath_bezier_offset_sample_certification", |b| {
        let mut problem = Problem::default();
        let x = problem.add_variable("offset_x", r(5));
        let y = problem.add_variable("offset_y", r(3));
        for constraint in bezier_offset_sample_constraints(
            "bezier offset",
            SolverPoint2::new(x, y),
            hyperlimit::Point2::new(r(5), r(0)),
            hyperlimit::Point2::new(r(10), r(0)),
            hyperlimit::Point2::new(r(0), r(10)),
            r(9),
        )
        .constraints
        {
            problem.add_constraint(constraint);
        }
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        b.iter(|| certify_candidate(&prepared, &context))
    });
    c.bench_function("domain_pcb_differential_pair_skew_certification", |b| {
        let mut problem = Problem::default();
        let first = problem.add_variable("first_length", r(1050));
        let second = problem.add_variable("second_length", r(1000));
        problem.add_constraint(differential_pair_skew_equation(
            "differential pair skew",
            Expr::symbol(SymbolId(first.0), "first_length"),
            Expr::symbol(SymbolId(second.0), "second_length"),
            r(50),
        ));
        let prepared = PreparedProblem::new(&problem);
        let context = context_from_problem(&problem);
        b.iter(|| certify_candidate(&prepared, &context))
    });
    c.bench_function(
        "domain_toolpath_rectangular_difference_certification",
        |b| {
            let mut problem = Problem::default();
            problem.add_constraint(rectangular_difference_area_equation(
                "rectangular difference area",
                rect(0, 0, 10, 10),
                Some(rect(3, 4, 7, 8)),
                vec![
                    rect(0, 0, 3, 10),
                    rect(7, 0, 10, 10),
                    rect(3, 0, 7, 4),
                    rect(3, 8, 7, 10),
                ],
            ));
            let prepared = PreparedProblem::new(&problem);
            let context = context_from_problem(&problem);
            b.iter(|| certify_candidate(&prepared, &context))
        },
    );
}

criterion_group!(benches, certification);
criterion_main!(benches);
