use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, Expr, PreparedProblem, PreparedSolverBlock, Problem, SymbolId, certify_candidate,
    context_from_problem,
};

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
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

fn certification(c: &mut Criterion) {
    let problem = affine_problem(16);
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    c.bench_function("prepared_solver_block_affine_rows", |b| {
        b.iter(|| PreparedSolverBlock::new(&prepared))
    });
    c.bench_function("certify_affine_candidate_exact", |b| {
        b.iter(|| certify_candidate(&prepared, &context))
    });
}

criterion_group!(benches, certification);
criterion_main!(benches);
