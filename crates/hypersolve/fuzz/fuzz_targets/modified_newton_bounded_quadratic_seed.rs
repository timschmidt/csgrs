#![no_main]

use hyperreal::Real;
use hypersolve::{
    solve_damped_least_squares, Constraint, ConvergenceReason, Expr, Problem, ProposalEngineKind,
    SolverConfig, SolverState, SymbolId,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 3]| {
    let start = i64::from(data[0] % 64);
    let lower_root = i64::from(data[1] % 32);
    let gap = i64::from((data[2] % 16).abs()) + 1;
    let upper_root = lower_root + gap;
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", Real::from(start));
    problem.variables[variable.0 as usize].lower = Some(Real::from(lower_root + 1));
    problem.add_constraint(Constraint::equality(
        "bounded quadratic branch",
        x.clone() * x.clone() - Expr::int(lower_root + upper_root) * x
            + Expr::int(lower_root * upper_root),
    ));

    let report = solve_damped_least_squares(SolverState {
        problem,
        config: SolverConfig {
            max_iterations: 1,
            proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
            ..SolverConfig::default()
        },
    });

    assert_eq!(report.reason, ConvergenceReason::Converged);
    assert_eq!(report.iterations, 0);
    assert!(report.preprocessing.requested);
    assert!(report.preprocessing.completed);
    assert_eq!(report.preprocessing.quadratic_soluble_alone_rows, 1);
    assert_eq!(report.preprocessing.quadratic_seed_assignments, 1);
    assert_eq!(report.preprocessing.rejected_quadratic_seed_assignments, 0);
});
