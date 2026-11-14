//! Complete affine-observation enclosures, with identical exact output bounds.
//! The original Taylor/affine certificate is common input. Cold noise rows include
//! noise preparation and destruction; warm rows reuse that prepared representation.
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorAffineMap, TaylorAffineModel, TaylorAffineRow,
        TaylorNoiseModel, TaylorRequest,
    },
};
use std::{hint::black_box, sync::Arc};
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 100_000_000)
}
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn source() -> Arc<TaylorAffineModel> {
    let x = Expr::anonymous_symbol(SymbolId(0));
    let shared = Expr::int(16) * x.clone() * x.clone();
    let p = ExpressionProgram::compile(&[shared.clone() + x, shared], &[SymbolId(0)], &mut graph())
        .unwrap();
    let original = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0)],
                domain: vec![d(-1, 1)],
                order: 2,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    );
    let identity = TaylorAffineMap {
        input_count: 2,
        rows: vec![
            TaylorAffineRow {
                coefficients: vec![Real::one(), Real::zero()],
                offset: Real::zero(),
            },
            TaylorAffineRow {
                coefficients: vec![Real::zero(), Real::one()],
                offset: Real::zero(),
            },
        ],
    };
    Arc::new(TaylorAffineModel::new(original, identity, &mut graph(), &mut scalar()).unwrap())
}
fn bench(c: &mut Criterion) {
    let source = source();
    let prepared = TaylorNoiseModel::new(source.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    let mut group = c.benchmark_group("taylor_noise_observations");
    for queries in [1, 16] {
        let maps: Vec<_> = (1..=queries)
            .map(|n| TaylorAffineMap {
                input_count: 2,
                rows: vec![TaylorAffineRow {
                    coefficients: vec![Real::from(n), Real::from(-n)],
                    offset: Real::zero(),
                }],
            })
            .collect();
        let expected: Vec<_> = (1..=queries).map(|n| d(-n, n)).collect();
        group.bench_with_input(
            BenchmarkId::new("full_affine", queries),
            &queries,
            |b, _| {
                b.iter(|| {
                    let mut g = graph();
                    let mut a = scalar();
                    for (map, bound) in maps.iter().zip(&expected) {
                        let result = source
                            .transform(black_box(map.clone()), &mut g, &mut a)
                            .unwrap();
                        assert_eq!(result.range().bounds(), std::slice::from_ref(bound));
                        drop(black_box(result));
                    }
                })
            },
        );
        group.bench_with_input(BenchmarkId::new("noise_cold", queries), &queries, |b, _| {
            b.iter(|| {
                let mut g = graph();
                let mut a = scalar();
                let model =
                    TaylorNoiseModel::new(black_box(source.clone()), 1, &mut g, &mut a).unwrap();
                for (map, bound) in maps.iter().zip(&expected) {
                    let result = model.enclose(black_box(map), &mut g, &mut a).unwrap();
                    assert_eq!(result.bounds(), std::slice::from_ref(bound));
                    drop(black_box(result));
                }
                drop(black_box(model));
            })
        });
        group.bench_with_input(BenchmarkId::new("noise_warm", queries), &queries, |b, _| {
            b.iter(|| {
                let mut g = graph();
                let mut a = scalar();
                for (map, bound) in maps.iter().zip(&expected) {
                    let result = prepared.enclose(black_box(map), &mut g, &mut a).unwrap();
                    assert_eq!(result.bounds(), std::slice::from_ref(bound));
                    drop(black_box(result));
                }
            })
        });
    }
    group.finish();
}
criterion_group!(benches, bench);
criterion_main!(benches);
