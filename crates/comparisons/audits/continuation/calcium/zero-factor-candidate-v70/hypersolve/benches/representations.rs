//! End-to-end solver benchmarks for every optimized finite `Real` class.

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, Expr, Problem, SparseResidualTerm, SymbolId, certify_candidate,
    context_from_problem, replay_sparse_linear_residuals, solve_dense_linear_system_bareiss,
};
use std::hint::black_box;
use std::time::Duration;

#[path = "support/benchmark_report.rs"]
mod benchmark_report;

const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const MIN_PRECISION: i32 = -256;

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn representation_values() -> Vec<(&'static str, Real)> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        ("One", fraction(3, 2)),
        ("Pi", pi.clone()),
        ("PiPow", pi_squared.clone()),
        ("PiInv", pi.clone().inverse().expect("pi is nonzero")),
        ("PiExp", &pi * &e),
        ("PiInvExp", (&e / &pi).expect("pi is nonzero")),
        ("PiSqrt", &pi * &sqrt_two),
        ("ConstProduct", &pi_squared * &e),
        ("ConstOffset", &pi - Real::from(3)),
        ("ConstProductSqrt", &(&pi_squared * &e) * &sqrt_two),
        ("Sqrt", sqrt_two),
        ("Exp", Real::from(2).exp().expect("finite exponential")),
        ("Ln", ln_three.clone()),
        (
            "LnAffine",
            (Real::from(2) * &e).ln().expect("positive logarithm input"),
        ),
        ("LnProduct", &ln_two * &ln_three),
        ("Log10", Real::from(2).log10().expect("positive input")),
        ("Log2", Real::from(3).log2().expect("positive input")),
        (
            "Pow10",
            fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        ),
        (
            "Pow2",
            fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        ),
        ("SinPi", fraction(1, 5).sin_pi()),
        (
            "TanPi",
            fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        ),
        ("Irrational", Real::one().sin()),
    ]
}

fn representation_problem(value: &Real) -> Problem {
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", value.clone());
    let symbol = SymbolId(variable.0);
    let delta = || Expr::symbol(symbol, "x") - Expr::real(value.clone());
    problem.add_constraint(Constraint::equality("affine", delta()));
    problem.add_constraint(Constraint::equality("quadratic", delta() * delta()));
    problem.add_constraint(Constraint::equality("non-polynomial", delta().sin()));
    problem
}

fn bench_real_representations(c: &mut Criterion) {
    let values = representation_values();
    assert_eq!(values.len(), 22, "update the Real representation benchmark");

    let mut group = c.benchmark_group("real_representations/full_solver_boundary");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(150))
        .measurement_time(Duration::from_millis(500));

    for (name, value) in values {
        let problem = representation_problem(&value);
        let context = context_from_problem(&problem);
        let matrix = vec![
            vec![value.clone(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        let rhs = vec![&value * Real::from(2), Real::from(3)];
        let sparse_terms = [
            SparseResidualTerm {
                row: 0,
                column: 0,
                coefficient: value.clone(),
            },
            SparseResidualTerm {
                row: 1,
                column: 1,
                coefficient: Real::one(),
            },
        ];
        let candidate = [Real::from(2), Real::from(3)];

        group.bench_function(name, |b| {
            b.iter(|| {
                let analysis = problem.analyze();
                black_box(
                    analysis
                        .evaluate_residuals(black_box(&context))
                        .expect("representation residuals"),
                );
                black_box(certify_candidate(&analysis, black_box(&context)));
                black_box(
                    solve_dense_linear_system_bareiss(
                        black_box(&matrix),
                        black_box(&rhs),
                        MIN_PRECISION,
                        APPROX,
                    )
                    .expect("bounded exact dense solve"),
                );
                black_box(
                    replay_sparse_linear_residuals(
                        2,
                        2,
                        black_box(&sparse_terms),
                        black_box(&rhs),
                        black_box(&candidate),
                        MIN_PRECISION,
                    )
                    .expect("identity-preserving exact sparse replay"),
                );
            })
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_real_representations,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
