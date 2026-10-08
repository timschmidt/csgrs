use hyperreal::Real;
use hypersolve::{Constraint, Expr, Problem, QuadraticResidual, SymbolId, UnivariateQuadraticResidual};
use std::collections::HashMap;

fn main() {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", Real::from(2));
    problem.add_variable("y", Real::from(3));
    let bindings = HashMap::from([(SymbolId(0), Real::from(2)), (SymbolId(1), Real::from(3))]);
    let cases = [
        ("different degrees", x.clone().powi(2) * (x.clone() - x.clone().powi(2))),
        ("different cubic monomials", x.clone().powi(2) * (x.clone() - y.clone())),
        ("quadratic plus discarded nonquadratic", x.clone().powi(2) + x.clone().powi(2) * (x.clone() - y)),
    ];
    for (name, expression) in cases {
        let exact = expression.eval_real(&bindings).unwrap();
        let uni = UnivariateQuadraticResidual::from_expr(&expression, &problem)
            .map(|p| p.eval_real(&problem.variables, &bindings).unwrap());
        let multi = QuadraticResidual::from_expr(&expression, &problem)
            .map(|p| p.eval_real(&problem.variables, &bindings).unwrap());
        println!("{name}: source={exact:?}; univariate={uni:?}; multivariate={multi:?}");
        assert!(multi.as_ref().is_some_and(|v| v != &exact), "expected baseline defect not reproduced");
        problem.add_constraint(Constraint::equality(name, expression));
    }
    let analysis = problem.analyze();
    assert!(analysis.quadratic_residuals().iter().all(Option::is_none));
    assert!(analysis.univariate_quadratic_residuals().iter().all(Option::is_none));
    println!("ProblemAnalysis degree guard rejects all three nonquadratic rows; no solver certificate defect claimed.");
}
