//! Completed polynomial-map proofs from an existing seed versus fresh differentiation.
//! Both paths retain exact coefficients and certify the same function/domain/order.
//! Polynomial cases have identical ranges. The nonlinear case has an explicit
//! common width limit; its distinct certified widths are printed, not hidden.
use criterion::{Criterion, criterion_group, criterion_main};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorModel, TaylorPolynomialModel, TaylorRequest,
    },
};
use std::{hint::black_box, sync::Arc};
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
struct Case {
    name: &'static str,
    seed: Arc<TaylorModel>,
    map: ExpressionProgram,
    native: ExpressionProgram,
    request: TaylorRequest,
    coefficients: [Real; 3],
    extrema: [Rational; 2],
    exact_range: Option<DyadicInterval>,
}
fn cases() -> Vec<Case> {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let u = Expr::anonymous_symbol(SymbolId(9));
    let root = Real::from(2).sqrt().unwrap();
    let f = Expr::int(1) / (Expr::int(1) - x.clone());
    let definitions = [
        (
            "centered_logistic",
            x.clone(),
            u.clone() * (Expr::int(1) - u.clone()),
            x.clone() * (Expr::int(1) - x.clone()),
            d(1, 1, -1),
            d(0, 1, 0),
            [
                Real::from(Rational::from(1) / Rational::from(4)),
                Real::zero(),
                Real::from(-1),
            ],
            [Rational::from(0), Rational::from(1) / Rational::from(4)],
            Some(d(0, 1, -2)),
        ),
        (
            "algebraic_product",
            x.clone() + Expr::real(root.clone()),
            u.clone() * (u.clone() - Expr::real(Real::from(2) * root)),
            x.clone() * x.clone() - Expr::int(2),
            d(0, 0, 0),
            d(-1, 1, 0),
            [Real::from(-2), Real::zero(), Real::one()],
            [Rational::from(-2), Rational::from(-1)],
            Some(d(-2, -1, 0)),
        ),
        (
            "reciprocal_square",
            f.clone(),
            u.clone() * u,
            f.clone() * f,
            d(0, 0, 0),
            d(-1, 1, -4),
            [Real::one(), Real::from(2), Real::from(3)],
            [
                Rational::from(256) / Rational::from(289),
                Rational::from(256) / Rational::from(225),
            ],
            None,
        ),
    ];
    definitions
        .into_iter()
        .map(
            |(name, seed, map, native, center, domain, coefficients, extrema, exact_range)| {
                let request = TaylorRequest {
                    center: vec![center],
                    domain: vec![domain],
                    order: 2,
                    precision: -64,
                };
                let seed =
                    ExpressionProgram::compile(&[seed], &[SymbolId(7)], &mut graph()).unwrap();
                let seed = Arc::new(
                    seed.taylor_model(request.clone(), &mut graph(), &mut arithmetic())
                        .unwrap(),
                );
                Case {
                    name,
                    seed,
                    map: ExpressionProgram::compile(&[map], &[SymbolId(9)], &mut graph()).unwrap(),
                    native: ExpressionProgram::compile(&[native], &[SymbolId(7)], &mut graph())
                        .unwrap(),
                    request,
                    coefficients,
                    extrema,
                    exact_range,
                }
            },
        )
        .collect()
}
fn validate(case: &Case) {
    let reused = TaylorPolynomialModel::new(
        case.seed.clone(),
        case.map.clone(),
        &mut graph(),
        &mut arithmetic(),
    )
    .unwrap();
    let rebuilt = case
        .native
        .taylor_model(case.request.clone(), &mut graph(), &mut arithmetic())
        .unwrap();
    for (degree, expected) in case.coefficients.iter().enumerate() {
        let powers = if degree == 0 {
            vec![]
        } else {
            vec![(SymbolId(7), degree)]
        };
        assert_eq!(reused.terms(0).unwrap().get(&powers), Some(expected));
        let term = rebuilt
            .terms()
            .iter()
            .find(|t| rebuilt.coefficient_requests()[t.request_index()].powers == powers)
            .unwrap();
        assert_eq!(term.coefficient(), expected);
    }
    for bounds in [reused.range(), rebuilt.image()] {
        assert_eq!(bounds.len(), 1);
        let exact = bounds[0].to_rational(&mut arithmetic()).unwrap();
        for endpoint in &case.extrema {
            assert!(exact.contains_point(endpoint));
        }
        if let Some(expected) = &case.exact_range {
            assert_eq!(&bounds[0], expected);
        } else {
            assert!(
                exact.upper().unwrap() - exact.lower().unwrap()
                    <= Rational::from(1) / Rational::from(2)
            );
        }
    }
    eprintln!(
        "COMPLETED {} reused={:?} rebuilt={:?}",
        case.name,
        reused.range(),
        rebuilt.image()
    );
}
fn benchmarks(c: &mut Criterion) {
    for case in cases() {
        validate(&case);
        let mut group = c.benchmark_group(format!("taylor_polynomial/{}", case.name));
        group.bench_function("reuse", |b| {
            b.iter(|| {
                drop(black_box(
                    TaylorPolynomialModel::new(
                        black_box(case.seed.clone()),
                        black_box(case.map.clone()),
                        &mut graph(),
                        &mut arithmetic(),
                    )
                    .unwrap(),
                ))
            })
        });
        group.bench_function("redifferentiate", |b| {
            b.iter(|| {
                drop(black_box(
                    black_box(&case.native)
                        .taylor_model(
                            black_box(case.request.clone()),
                            &mut graph(),
                            &mut arithmetic(),
                        )
                        .unwrap(),
                ))
            })
        });
        group.finish();
    }
}
criterion_group!(benches, benchmarks);
criterion_main!(benches);
