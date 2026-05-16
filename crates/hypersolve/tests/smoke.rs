use hyperreal::{Rational, Real, SymbolicDependencyMask};
use hypersolve::{
    Constraint, ConvergenceReason, Expr, ExprDegree, PreparedProblem, Problem, SolverConfig,
    SolverState, SymbolId, context_from_problem, evaluate_residuals, facts_depend_on_symbol,
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
    assert_eq!(facts.polynomial_active_rows, 3);
    assert_eq!(facts.non_polynomial_active_rows, 0);
    assert_eq!(facts.structural_jacobian_nonzeros, 3);

    assert_eq!(prepared.constraints()[0].dependent_columns, vec![0]);
    assert_eq!(prepared.constraints()[1].dependent_columns, vec![0, 1]);
    assert_eq!(
        prepared.constraints()[2].dependent_columns,
        Vec::<usize>::new()
    );
    assert_eq!(prepared.jacobian_sparsity()[0], vec![true, false]);
    assert_eq!(prepared.jacobian_sparsity()[1], vec![true, true]);
    assert_eq!(prepared.jacobian_sparsity()[2], vec![false, false]);
    assert!(facts_depend_on_symbol(
        &prepared.constraints()[1].residual,
        SymbolId(1)
    ));
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
