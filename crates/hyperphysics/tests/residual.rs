use hyperphysics::{DiagnosticStatus, HypersolveResidualReplayReport, Real};
use hypersolve::{Constraint, Expr, Problem, SymbolId};
use proptest::prelude::*;

fn r(value: i64) -> Real {
    value.into()
}

#[test]
fn hypersolve_residual_replay_certifies_zero_candidate() {
    let mut problem = Problem::default();
    problem.add_variable("x", r(3));
    let x = Expr::symbol(SymbolId(0), "x");
    problem.add_constraint(Constraint::equality(
        "x squared minus nine",
        (x.clone() * x) - Expr::int(9),
    ));

    let report = HypersolveResidualReplayReport::replay(&problem).unwrap();

    assert_eq!(report.variable_count, 1);
    assert_eq!(report.residuals.len(), 1);
    assert_eq!(report.residuals[0].value, Real::zero());
    assert_eq!(report.status, DiagnosticStatus::Exact);
    assert!(report.all_residuals_zero());
}

#[test]
fn hypersolve_residual_replay_keeps_nonzero_residual_visible() {
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
    let x = Expr::symbol(SymbolId(0), "x");
    problem.add_constraint(Constraint::equality("x minus three", x - Expr::int(3)));

    let report = HypersolveResidualReplayReport::replay(&problem).unwrap();

    assert_eq!(report.residuals[0].value, r(-1));
    assert_eq!(report.status, DiagnosticStatus::Exact);
    assert!(!report.all_residuals_zero());
    assert!(report.residuals[0].dense_solver_estimate.is_some());
}

proptest! {
    #[test]
    fn generated_linear_candidate_residual_is_exact(value in -1000_i64..1000_i64, target in -1000_i64..1000_i64) {
        let mut problem = Problem::default();
        problem.add_variable("x", r(value));
        let x = Expr::symbol(SymbolId(0), "x");
        problem.add_constraint(Constraint::equality("x minus target", x - Expr::real(r(target))));

        let report = HypersolveResidualReplayReport::replay(&problem).unwrap();

        prop_assert_eq!(report.residuals[0].value.clone(), r(value - target));
        prop_assert_eq!(report.all_residuals_zero(), value == target);
    }
}
