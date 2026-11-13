//! Absolute costs of completed graph operations, not validated-field timings.
//! Scalar inputs are exact small integers. Compile includes payload cloning;
//! replay includes definition cloning; evaluation includes full output/drop.

use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn budget() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 1_000_000)
}

fn benchmarks(c: &mut Criterion) {
    let x = SymbolId(0);
    let variable = Expr::anonymous_symbol(x);
    for degree in [1, 8, 32, 128] {
        let mut source = Expr::int(1);
        for _ in 0..degree {
            // Explicit constructors retain authored tree structure.
            source = Expr::Add(
                Box::new(Expr::Mul(Box::new(source), Box::new(variable.clone()))),
                Box::new(Expr::int(1)),
            );
        }
        let sources = [source];
        let program = ExpressionProgram::compile(&sources, &[x], &mut budget()).unwrap();
        let values = [Real::one()];
        assert_eq!(
            program.eval_real(&values, &mut budget()).unwrap(),
            [Real::from(degree + 1)]
        );
        let mut group = c.benchmark_group(format!("expression_program/degree_{degree}"));
        group.bench_function("compile", |b| {
            b.iter(|| {
                black_box(
                    ExpressionProgram::compile(black_box(&sources), &[x], &mut budget()).unwrap(),
                );
            })
        });
        group.bench_function("replay", |b| {
            b.iter(|| {
                black_box(
                    ExpressionProgram::from_definition(
                        black_box(program.definition()).clone(),
                        &mut budget(),
                    )
                    .unwrap(),
                );
            })
        });
        group.bench_function("evaluate", |b| {
            b.iter(|| {
                let result = program
                    .eval_real(black_box(&values), &mut budget())
                    .unwrap();
                assert_eq!(result, [Real::from(degree + 1)]);
                black_box(result);
            })
        });
        group.finish();
    }
}

criterion_group!(benches, benchmarks);
criterion_main!(benches);
