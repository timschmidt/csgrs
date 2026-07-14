use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, PreparedProblem, Problem, certify_candidate, context_from_problem,
};

fn main() {
    let mut problem = Problem::default();
    let x = problem.add_variable("x", Real::from(2));
    let x_expr = Expr::symbol(problem.variables[x.0 as usize].symbol, "x");
    problem.add_constraint(Constraint::equality(
        "x squared is four",
        x_expr.clone() * x_expr - Expr::real(Real::from(4)),
    ));

    let prepared = PreparedProblem::new(&problem);
    let candidate = context_from_problem(&problem);
    let certification = certify_candidate(&prepared, &candidate);

    assert_eq!(certification.certified_satisfied_rows, 1);
    assert!(!certification.has_certified_violation());
}
