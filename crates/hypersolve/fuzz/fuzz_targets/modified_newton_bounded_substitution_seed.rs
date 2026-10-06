#![no_main]

use hyperreal::Real;
use hypersolve::{
    solve_damped_least_squares, Constraint, ConvergenceReason, Expr, Problem, ProposalEngineKind,
    SolverConfig, SolverState, SymbolId,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 4]| {
    let start_x = i64::from(data[0] % 64);
    let start_y = i64::from(data[1] % 64);
    let offset = i64::from(data[2] % 32);
    let target_y = i64::from(data[3] % 32);
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    let x_id = problem.add_variable("x", Real::from(start_x));
    problem.variables[x_id.0 as usize].upper = Some(Real::from(target_y + offset));
    let y_id = problem.add_variable("y", Real::from(start_y));
    problem.variables[y_id.0 as usize].lower = Some(Real::from(target_y));
    problem.add_constraint(Constraint::equality(
        "bounded substitution",
        x - y - Expr::int(offset),
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
    assert_eq!(report.preprocessing.equality_substitutions, 1);
    assert_eq!(report.preprocessing.affine_seed_assignments, 0);
    assert_eq!(report.preprocessing.substitution_seed_classes, 1);
    assert_eq!(report.preprocessing.rejected_substitution_seed_classes, 0);
    assert_eq!(report.preprocessing.substitution_seed_assignments, 2);
});
