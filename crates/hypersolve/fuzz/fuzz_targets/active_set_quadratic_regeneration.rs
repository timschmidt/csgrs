#![no_main]

use hyperreal::Real;
use hypersolve::{
    ActiveSetQuadraticCandidateStatus, ActiveSetQuadraticRegenerationConfig,
    ActiveSetQuadraticRegenerationStatus, Constraint, EvaluationContext, Expr, Problem, SymbolId,
    regenerate_active_set_quadratic_candidates,
};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fuzz_target!(|data: [i16; 2]| {
    let root = i64::from((data[0] % 32).abs()) + 1;
    let bound = i64::from(data[1] % 32);
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", real(0));
    problem.add_constraint(Constraint::equality(
        "generated quadratic",
        x.clone() * x.clone() - Expr::int(root * root),
    ));
    let mut lower = Constraint::equality("generated lower bound", x - Expr::int(bound));
    lower.kind = hypersolve::ConstraintKind::GreaterOrEqual;
    lower.active = false;
    problem.add_constraint(lower);

    let prepared = hypersolve::PreparedProblem::new(&problem);
    let report = regenerate_active_set_quadratic_candidates(
        &prepared,
        &EvaluationContext::default(),
        &[true, false],
        ActiveSetQuadraticRegenerationConfig::default(),
    );
    assert_eq!(
        report.status,
        // An inactive inequality must be strictly satisfied; equality is a
        // binding-mask mismatch even when the positive root is feasible.
        if root > bound {
            ActiveSetQuadraticRegenerationStatus::Certified
        } else {
            ActiveSetQuadraticRegenerationStatus::RejectedCandidate
        }
    );
    if report.status == ActiveSetQuadraticRegenerationStatus::Certified {
        assert!(report.candidates.iter().any(|candidate| {
            candidate.status == ActiveSetQuadraticCandidateStatus::Certified
                && candidate.candidate.bindings().get(&SymbolId(0)) == Some(&real(root))
        }));
    }
});
