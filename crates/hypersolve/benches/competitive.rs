//! Competitive solver benchmarks with explicit exactness boundaries.
//!
//! `roots` and `nalgebra` operate on primitive floats, so their rows are
//! proposal-only controls. Hypersolve's rows retain exact `Real` results and
//! residual proof. The existing CGAL/Gmpq harness remains the
//! correctness-equivalent exact quadratic competitor.

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    Constraint, Expr, Problem, SymbolId, solve_dense_linear_system_bareiss,
    solve_direct_univariate_quadratic_equalities,
};
use nalgebra::{Matrix4, Vector4};
use std::hint::black_box;

#[path = "support/benchmark_report.rs"]
mod benchmark_report;
use std::time::Duration;

const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

fn sqrt_two_problem() -> Problem {
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", Real::one());
    let symbol = SymbolId(variable.0);
    let x = Expr::symbol(symbol, "x");
    problem.add_constraint(Constraint::equality(
        "x squared minus two",
        x.clone() * x - Expr::int(2),
    ));
    problem
}

fn bench_quadratic_roots(c: &mut Criterion) {
    let problem = sqrt_two_problem();
    let analysis = problem.analyze();
    let exact = solve_direct_univariate_quadratic_equalities(&analysis)
        .expect("supported exact quadratic row");
    assert_eq!(exact.len(), 1);
    assert_eq!(exact[0].roots.len(), 2);
    assert!(matches!(
        roots::find_roots_quadratic(1.0_f64, 0.0, -2.0),
        roots::Roots::Two(_)
    ));

    let mut group = c.benchmark_group("competitive/quadratic_roots");
    group
        .sample_size(30)
        .warm_up_time(Duration::from_millis(250))
        .measurement_time(Duration::from_secs(1));
    group.bench_function("hypersolve_exact_candidates", |b| {
        b.iter(|| {
            solve_direct_univariate_quadratic_equalities(black_box(&analysis))
                .expect("supported exact quadratic row")
        })
    });
    group.bench_function("roots_f64_proposal", |b| {
        b.iter(|| roots::find_roots_quadratic(black_box(1.0_f64), 0.0, -2.0))
    });
    group.finish();
}

fn bench_dense_linear(c: &mut Criterion) {
    let exact_matrix = vec![
        vec![Real::from(4), Real::from(1), Real::zero(), Real::zero()],
        vec![Real::from(1), Real::from(4), Real::from(1), Real::zero()],
        vec![Real::zero(), Real::from(1), Real::from(4), Real::from(1)],
        vec![Real::zero(), Real::zero(), Real::from(1), Real::from(3)],
    ];
    let exact_rhs = vec![
        Real::from(6),
        Real::from(12),
        Real::from(18),
        Real::from(15),
    ];
    let exact = solve_dense_linear_system_bareiss(&exact_matrix, &exact_rhs, -256, APPROX)
        .expect("nonsingular exact fixture");
    assert!(exact.residual_replay.accepted);
    assert_eq!(
        exact.solution,
        vec![Real::from(1), Real::from(2), Real::from(3), Real::from(4)]
    );

    let float_matrix = Matrix4::new(
        4.0, 1.0, 0.0, 0.0, //
        1.0, 4.0, 1.0, 0.0, //
        0.0, 1.0, 4.0, 1.0, //
        0.0, 0.0, 1.0, 3.0,
    );
    let float_rhs = Vector4::new(6.0, 12.0, 18.0, 15.0);
    let float_solution = float_matrix
        .lu()
        .solve(&float_rhs)
        .expect("nonsingular float fixture");
    assert!((float_solution - Vector4::new(1.0, 2.0, 3.0, 4.0)).norm() < 1.0e-12);

    let mut group = c.benchmark_group("competitive/dense_linear_4x4");
    group
        .sample_size(30)
        .warm_up_time(Duration::from_millis(250))
        .measurement_time(Duration::from_secs(1));
    group.bench_function("hypersolve_exact_with_replay", |b| {
        b.iter(|| {
            solve_dense_linear_system_bareiss(
                black_box(&exact_matrix),
                black_box(&exact_rhs),
                -256,
                APPROX,
            )
            .expect("nonsingular exact fixture")
        })
    });
    group.bench_function("nalgebra_f64_lu_proposal", |b| {
        b.iter(|| {
            black_box(float_matrix)
                .lu()
                .solve(black_box(&float_rhs))
                .expect("nonsingular float fixture")
        })
    });
    group.finish();
}

criterion_group!(
    benches,
    bench_quadratic_roots,
    bench_dense_linear,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
