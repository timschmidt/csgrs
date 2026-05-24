#![no_main]

use hyperreal::Real;
use hypersolve::{
    context_from_problem, search_failed_constraint_minimal_removals, Constraint, Expr,
    FailedConstraintRemovalStatus, PreparedProblem, Problem, SymbolId,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 4]| {
    let value = i64::from(data[0]);
    let target = |index: usize| {
        let mut target = i64::from(data[index]);
        if target == value {
            target += 1;
        }
        target
    };
    let first = target(1);
    let second = target(2);
    let third = target(3);
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", Real::from(value));
    problem.add_constraint(Constraint::equality(
        "first generated miss",
        x.clone() - Expr::int(first),
    ));
    problem.add_constraint(Constraint::equality(
        "second generated miss",
        x.clone() - Expr::int(second),
    ));
    problem.add_constraint(Constraint::equality(
        "third generated miss",
        x - Expr::int(third),
    ));

    let pair_bound = search_failed_constraint_minimal_removals(
        &PreparedProblem::new(&problem),
        &context_from_problem(&problem),
        2,
    );
    assert_eq!(pair_bound.original.blocking_rows, 3);
    assert_eq!(pair_bound.minimal_cardinality, None);
    assert_eq!(pair_bound.probes.len(), 6);
    assert_eq!(pair_bound.clearing_removals, 0);

    let triple_bound = search_failed_constraint_minimal_removals(
        &PreparedProblem::new(&problem),
        &context_from_problem(&problem),
        3,
    );
    assert_eq!(triple_bound.original.blocking_rows, 3);
    assert_eq!(triple_bound.minimal_cardinality, Some(3));
    assert_eq!(triple_bound.probes.len(), 1);
    assert_eq!(triple_bound.clearing_removals, 1);
    assert_eq!(
        triple_bound.probes[0].removal_status,
        FailedConstraintRemovalStatus::ClearsAllBlockingRows
    );
});
