//! Completed certified model composition versus fresh native Taylor construction.
//! Both paths meet the same explicit range-width contract. Preparation is separate.
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
    inner: Arc<TaylorModel>,
    outer: Arc<TaylorModel>,
    native: ExpressionProgram,
    request: TaylorRequest,
    expected: [Real; 3],
    // Exact extremum values, or their squares for the positive sqrt fixture.
    extrema: [Rational; 2],
    squared: bool,
    width: DyadicInterval,
}
fn cases() -> Vec<Case> {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let u = Expr::anonymous_symbol(SymbolId(20));
    let reciprocal = Expr::int(1) / (Expr::int(1) - x.clone());
    let root = Real::from(2).sqrt().unwrap();
    let definitions = [
        (
            "nested_reciprocal",
            reciprocal.clone(),
            Expr::int(1) / (Expr::int(2) + u.clone()),
            Expr::int(1) / (Expr::int(2) + reciprocal),
            d(1, 1, 0),
            d(7, 9, -3),
            [
                Real::from(Rational::from(1) / Rational::from(3)),
                Real::from(Rational::from(-1) / Rational::from(9)),
                Real::from(Rational::from(-2) / Rational::from(27)),
            ],
            [
                Rational::from(15) / Rational::from(46),
                Rational::from(17) / Rational::from(50),
            ],
            false,
            d(1, 1, -5),
        ),
        (
            "algebraic_sqrt",
            x.clone(),
            (Expr::int(2) + u).sqrt(),
            (Expr::int(2) + x).sqrt(),
            d(0, 0, 0),
            d(-1, 1, -4),
            [
                root.clone(),
                root.clone() * Real::from(Rational::from(1) / Rational::from(4)),
                root * Real::from(Rational::from(-1) / Rational::from(32)),
            ],
            [
                Rational::from(31) / Rational::from(16),
                Rational::from(33) / Rational::from(16),
            ],
            true,
            d(1, 1, -3),
        ),
    ];
    definitions
        .into_iter()
        .map(
            |(name, inner, outer, native, center, domain, expected, extrema, squared, width)| {
                let request = TaylorRequest {
                    center: vec![d(0, 0, 0)],
                    domain: vec![d(-1, 1, -4)],
                    order: 2,
                    precision: -64,
                };
                let inner = Arc::new(
                    ExpressionProgram::compile(&[inner], &[SymbolId(7)], &mut graph())
                        .unwrap()
                        .taylor_model(request.clone(), &mut graph(), &mut arithmetic())
                        .unwrap(),
                );
                let outer = Arc::new(
                    ExpressionProgram::compile(&[outer], &[SymbolId(20)], &mut graph())
                        .unwrap()
                        .taylor_model(
                            TaylorRequest {
                                center: vec![center],
                                domain: vec![domain],
                                order: 2,
                                precision: -64,
                            },
                            &mut graph(),
                            &mut arithmetic(),
                        )
                        .unwrap(),
                );
                let native =
                    ExpressionProgram::compile(&[native], &[SymbolId(7)], &mut graph()).unwrap();
                Case {
                    name,
                    inner,
                    outer,
                    native,
                    request,
                    expected,
                    extrema,
                    squared,
                    width,
                }
            },
        )
        .collect()
}
fn reuse(case: &Case) -> TaylorPolynomialModel {
    case.inner
        .clone()
        .compose(case.outer.clone(), &mut graph(), &mut arithmetic())
        .unwrap()
}
fn fresh(case: &Case) -> TaylorModel {
    case.native
        .taylor_model(case.request.clone(), &mut graph(), &mut arithmetic())
        .unwrap()
}
fn validate_range(case: &Case, range: &[DyadicInterval]) {
    assert_eq!(range.len(), 1);
    assert!(
        range[0]
            .width(&mut arithmetic())
            .unwrap()
            .subset_of(
                &d(0, 1, if case.squared { -3 } else { -5 }),
                &mut arithmetic()
            )
            .unwrap()
    );
    let r = range[0].to_rational(&mut arithmetic()).unwrap();
    let (lo, hi) = (r.lower().unwrap(), r.upper().unwrap());
    if case.squared {
        assert!(lo > &0.into());
        assert!(lo * lo <= case.extrema[0]);
        assert!(hi * hi >= case.extrema[1]);
    } else {
        assert!(lo <= &case.extrema[0]);
        assert!(hi >= &case.extrema[1]);
    }
    let allowed = case.width.to_rational(&mut arithmetic()).unwrap();
    assert!(r.upper().unwrap() - r.lower().unwrap() <= allowed.upper().unwrap().clone());
}
fn validate(case: &Case) {
    let reused = reuse(case);
    let native = fresh(case);
    validate_range(case, reused.range());
    validate_range(case, native.image());
    assert_eq!(reused.range(), native.image());
    for (degree, expected) in case.expected.iter().enumerate() {
        let powers = if degree == 0 {
            vec![]
        } else {
            vec![(SymbolId(7), degree)]
        };
        let got = reused.terms(0).unwrap().get(&powers).unwrap();
        let native_term = native
            .terms()
            .iter()
            .find(|term| {
                native.coefficient_program().requests()[term.request_index()].powers == powers
            })
            .unwrap();
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            assert!(matches!(
                hyperlimit::classify_real_sign(
                    &(native_term.coefficient().clone() - expected.clone()),
                    policy
                ),
                hyperlimit::PredicateOutcome::Decided {
                    value: hyperlimit::Sign::Zero,
                    certainty: hyperlimit::Certainty::Exact | hyperlimit::Certainty::Filtered,
                    ..
                }
            ));
            assert!(matches!(
                hyperlimit::classify_real_sign(&(got.clone() - expected.clone()), policy),
                hyperlimit::PredicateOutcome::Decided {
                    value: hyperlimit::Sign::Zero,
                    certainty: hyperlimit::Certainty::Exact | hyperlimit::Certainty::Filtered,
                    ..
                }
            ));
        }
    }
    eprintln!(
        "COMPLETED {} reuse_range {:?} fresh_range {:?}",
        case.name,
        reused.range(),
        native.image()
    );
}
fn benchmarks(c: &mut Criterion) {
    for case in cases() {
        validate(&case);
        let mut group = c.benchmark_group(format!("taylor_composition/{}", case.name));
        group.bench_function("reuse", |b| {
            b.iter(|| drop(black_box(reuse(black_box(&case)))))
        });
        group.bench_function("redifferentiate", |b| {
            b.iter(|| drop(black_box(fresh(black_box(&case)))))
        });
        group.bench_function("inner_preparation", |b| {
            b.iter(|| {
                drop(black_box(
                    case.inner
                        .source()
                        .taylor_model(
                            black_box(case.inner.request().clone()),
                            &mut graph(),
                            &mut arithmetic(),
                        )
                        .unwrap(),
                ))
            })
        });
        group.bench_function("outer_preparation", |b| {
            b.iter(|| {
                drop(black_box(
                    case.outer
                        .source()
                        .taylor_model(
                            black_box(case.outer.request().clone()),
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
