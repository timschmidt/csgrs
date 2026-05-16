use hyperreal::{Rational, Real, SymbolicDependencyMask};
use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_prepared};
use hypersolve::{
    Constraint, ConvergenceReason, DenseLinearBackend, Expr, ExprDegree, LinearAdapterKind,
    LinearAdapterPrecision, LinearBackend, PreparedProblem, Problem, SolverConfig, SolverState,
    SymbolId, context_from_problem, evaluate_residuals, facts_depend_on_symbol,
    solve_damped_least_squares,
};

fn real(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn edge_real(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

#[test]
fn expression_evaluates_with_symbol_binding() {
    let x = Expr::symbol(SymbolId(0), "x");
    let expr = (x.clone() * x) - Expr::int(9);
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_constraint(Constraint::equality("x squared minus nine", expr));

    let context = context_from_problem(&problem);
    let residuals = evaluate_residuals(&problem, &context).unwrap();

    assert_eq!(residuals.len(), 1);
    assert_eq!(residuals[0].value, Real::zero());
}

#[test]
fn symbolic_derivative_solves_one_variable_linear_equation() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("x minus two", x - Expr::int(2)));

    let report = solve_damped_least_squares(SolverState {
        problem,
        config: SolverConfig {
            max_iterations: 8,
            residual_tolerance: edge_real(1.0e-6),
            step_tolerance: edge_real(1.0e-12),
            damping: edge_real(1.0e-9),
        },
    });

    assert!(matches!(
        report.reason,
        ConvergenceReason::Converged | ConvergenceReason::StepTooSmall
    ));
    assert!(
        !report.linear_reports.is_empty(),
        "top-level solve reports should expose dense adapter diagnostics"
    );
    assert!(report.linear_reports.iter().all(|row| row.is_lossy()));
    assert!(
        report
            .linear_reports
            .iter()
            .all(|row| row.precision() == LinearAdapterPrecision::LossyF64)
    );
}

#[test]
fn expression_facts_classify_affine_polynomial_and_transcendental_rows() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");

    let affine = (x.clone() * Expr::int(3)) + Expr::int(7);
    let affine_facts = affine.structural_facts();
    assert_eq!(affine_facts.degree, ExprDegree::Polynomial(1));
    assert!(affine_facts.is_affine());
    assert!(affine_facts.dependencies.contains(&SymbolId(0)));

    let quadratic = x.clone() * y.clone();
    let quadratic_facts = quadratic.structural_facts();
    assert_eq!(quadratic_facts.degree, ExprDegree::Polynomial(2));
    assert!(quadratic_facts.dependencies.contains(&SymbolId(0)));
    assert!(quadratic_facts.dependencies.contains(&SymbolId(1)));

    let non_polynomial = (x.clone() / (y + Expr::int(1))).sin();
    assert_eq!(
        non_polynomial.structural_facts().degree,
        ExprDegree::NonPolynomial
    );
}

#[test]
fn expression_facts_preserve_real_symbolic_dependency_families() {
    let x = Expr::symbol(SymbolId(0), "x");
    let expression = Expr::real(Real::pi()) + x;
    let facts = expression.structural_facts();

    assert_eq!(facts.degree, ExprDegree::Polynomial(1));
    assert!(facts.dependencies.contains(&SymbolId(0)));
    assert!(
        facts
            .real_symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );
}

#[test]
fn prepared_problem_caches_residual_dependency_and_sparsity_facts() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_variable("y", real(0));
    problem.add_constraint(Constraint::equality(
        "affine x row",
        x.clone() * Expr::int(2) + Expr::int(1),
    ));
    problem.add_constraint(Constraint::equality(
        "quadratic coupling",
        x.clone() * y.clone(),
    ));
    problem.add_constraint(Constraint::equality("constant", Expr::int(7)));

    let prepared = PreparedProblem::new(&problem);
    let facts = prepared.facts();

    assert_eq!(facts.variable_count, 2);
    assert_eq!(facts.constraint_count, 3);
    assert_eq!(facts.active_constraint_count, 3);
    assert_eq!(facts.affine_active_rows, 1);
    assert_eq!(facts.prepared_affine_active_rows, 2);
    assert_eq!(facts.polynomial_active_rows, 3);
    assert_eq!(facts.non_polynomial_active_rows, 0);
    assert_eq!(facts.known_zero_constant_active_rows, 0);
    assert_eq!(facts.known_nonzero_constant_active_rows, 1);
    assert_eq!(facts.unknown_sign_constant_active_rows, 0);
    assert!(facts.has_known_nonzero_constant_residual());
    assert_eq!(facts.structural_jacobian_nonzeros, 3);
    assert!(!facts.all_active_rows_prepared_affine());

    assert_eq!(prepared.constraints()[0].dependent_columns, vec![0]);
    assert_eq!(prepared.constraints()[1].dependent_columns, vec![0, 1]);
    assert_eq!(
        prepared.constraints()[2].dependent_columns,
        Vec::<usize>::new()
    );
    assert_eq!(
        prepared.constraints()[2].residual_constant_sign,
        Some(hyperreal::RealSign::Positive)
    );
    assert!(prepared.constraints()[2].is_known_nonzero_constant_row());
    assert_eq!(prepared.jacobian_sparsity()[0], vec![true, false]);
    assert_eq!(prepared.jacobian_sparsity()[1], vec![true, true]);
    assert_eq!(prepared.jacobian_sparsity()[2], vec![false, false]);
    assert!(prepared.affine_residuals()[0].is_some());
    assert!(prepared.affine_residuals()[1].is_none());
    assert!(prepared.affine_residuals()[2].is_some());
    assert!(facts_depend_on_symbol(
        &prepared.constraints()[1].residual,
        SymbolId(1)
    ));
}

#[test]
fn prepared_affine_residual_preserves_coefficients_and_exact_product_sum_eval() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(5));
    problem.add_variable("y", real(7));
    problem.add_constraint(Constraint::equality(
        "affine block",
        x.clone() * Expr::int(2) - y.clone() * Expr::int(3) + Expr::int(11),
    ));
    let context = context_from_problem(&problem);
    let prepared = PreparedProblem::new(&problem);
    let affine = prepared.affine_residuals()[0]
        .as_ref()
        .expect("affine row should be prepared");

    assert!(prepared.facts().all_active_rows_prepared_affine());
    assert_eq!(affine.coefficients(), &[real(2), real(-3)]);
    assert_eq!(affine.constant(), &real(11));
    assert_eq!(affine.nonzero_coefficient_count(), 2);
    assert!(affine.coefficient_exact().all_exact_rational);
    assert!(affine.is_exact_rational());

    let prepared_value = affine
        .eval_real(&problem.variables, context.bindings())
        .expect("prepared affine eval should succeed");
    let ordinary_value = problem.constraints[0]
        .residual
        .eval_real(context.bindings())
        .expect("ordinary expression eval should succeed");

    assert_eq!(prepared_value, ordinary_value);
    assert_eq!(
        prepared.evaluate_residuals(&context).unwrap()[0].value,
        real(0)
    );
}

#[test]
fn prepared_affine_residual_rejects_stale_model_shape() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(5));
    problem.add_constraint(Constraint::equality("affine block", x + Expr::int(1)));
    let prepared = PreparedProblem::new(&problem);
    let affine = prepared.affine_residuals()[0]
        .as_ref()
        .expect("affine row should be prepared");

    let error = affine
        .eval_real(&[], context_from_problem(&problem).bindings())
        .expect_err("stale prepared affine row must not evaluate against a different model shape");

    match error {
        hypersolve::ExprEvalError::PreparedShapeMismatch {
            expected_coefficients,
            actual_variables,
        } => {
            assert_eq!(expected_coefficients, 1);
            assert_eq!(actual_variables, 0);
        }
        other => panic!("unexpected eval error: {other:?}"),
    }
}

#[test]
fn prepared_problem_classifies_constant_residual_signs_without_evaluation() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality("known zero", Expr::zero()));
    problem.add_constraint(Constraint::equality("known negative", Expr::int(-5)));
    problem.add_constraint(Constraint::equality(
        "constant expression tree",
        Expr::int(2) - Expr::int(2),
    ));
    problem.add_constraint(Constraint::equality("variable row", x - Expr::int(1)));

    let prepared = PreparedProblem::new(&problem);
    let facts = prepared.facts();

    assert_eq!(facts.active_constraint_count, 4);
    assert_eq!(facts.known_zero_constant_active_rows, 1);
    assert_eq!(facts.known_nonzero_constant_active_rows, 1);
    assert_eq!(facts.unknown_sign_constant_active_rows, 1);
    assert!(facts.has_known_nonzero_constant_residual());

    assert!(prepared.constraints()[0].is_known_zero_constant_row());
    assert_eq!(
        prepared.constraints()[1].residual_constant_sign,
        Some(hyperreal::RealSign::Negative)
    );
    assert!(prepared.constraints()[1].is_known_nonzero_constant_row());
    assert!(prepared.constraints()[2].is_constant_row());
    assert_eq!(prepared.constraints()[2].residual_constant_sign, None);
    assert!(!prepared.constraints()[3].is_constant_row());
    assert_eq!(prepared.constraints()[3].residual_constant_sign, None);
}

#[test]
fn prepared_problem_evaluates_through_source_problem_without_float_topology() {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(3));
    problem.add_constraint(Constraint::equality("x minus three", x - Expr::int(3)));
    let context = context_from_problem(&problem);
    let prepared = PreparedProblem::new(&problem);

    let residuals = prepared.evaluate_residuals(&context).unwrap();

    assert_eq!(residuals.len(), 1);
    assert_eq!(residuals[0].value, Real::zero());
    assert_eq!(residuals[0].sign, Some(hyperreal::RealSign::Zero));
}

#[test]
fn prepared_symbolic_jacobian_consumes_structural_sparsity() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", real(5));
    problem.add_variable("y", real(11));
    problem.add_constraint(Constraint::equality(
        "x-only affine row",
        x.clone() * Expr::int(3) - Expr::int(4),
    ));
    problem.add_constraint(Constraint::equality("coupled row", x.clone() * y.clone()));
    let context = context_from_problem(&problem);
    let prepared = PreparedProblem::new(&problem);

    let plain = symbolic_jacobian(&problem, &context).unwrap();
    let sparse = symbolic_jacobian_prepared(&prepared, &context).unwrap();

    assert_eq!(sparse, plain);
    assert_eq!(prepared.jacobian_sparsity()[0], vec![true, false]);
    assert_eq!(sparse[0][1], 0.0);
    assert_eq!(sparse[0][0], 3.0);
}

#[test]
fn dense_linear_adapter_reports_lossy_pivot_diagnostics() {
    let backend = DenseLinearBackend;
    let jacobian = vec![vec![0.0, 2.0], vec![3.0, 0.0]];
    let residuals = vec![-4.0, -9.0];

    let (step, report) = backend
        .solve_damped_normal(&jacobian, &residuals, 0.0)
        .expect("nonsingular dense adapter solve should succeed");

    assert_eq!(report.adapter, LinearAdapterKind::DenseF64NormalEquations);
    assert_eq!(report.precision(), LinearAdapterPrecision::LossyF64);
    assert!(report.lossy);
    assert!(report.is_lossy());
    assert_eq!(report.rank_hint, Some(2));
    assert_eq!(report.pivot_count, 2);
    assert_eq!(report.row_swaps, 0);
    assert!(report.min_abs_pivot.unwrap() > 0.0);
    assert!(report.max_abs_pivot.unwrap() >= report.min_abs_pivot.unwrap());
    assert!((step[0] - 3.0).abs() < 1.0e-9);
    assert!((step[1] - 2.0).abs() < 1.0e-9);
}
