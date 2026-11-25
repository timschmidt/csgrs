//! Joining an independently built model versus fresh native Taylor construction
//! of the combined expression. Both paths are validated against the same width
//! contract before timing; preparation of the two source models is separate.
use criterion::{Criterion, criterion_group, criterion_main};
use hyperinterval::{DyadicBudget, DyadicInterval};
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
fn model(
    expr: Expr,
    center: DyadicInterval,
    domain: DyadicInterval,
    order: usize,
) -> Arc<TaylorModel> {
    Arc::new(
        ExpressionProgram::compile(&[expr], &[SymbolId(7)], &mut graph())
            .unwrap()
            .taylor_model(
                TaylorRequest {
                    center: vec![center],
                    domain: vec![domain],
                    order,
                    precision: -64,
                },
                &mut graph(),
                &mut arithmetic(),
            )
            .unwrap(),
    )
}
struct Case {
    name: &'static str,
    left: Arc<TaylorModel>,
    right: Arc<TaylorModel>,
    map: ExpressionProgram,
    native: ExpressionProgram,
    request: TaylorRequest,
    width: DyadicInterval,
}
fn cases() -> Vec<Case> {
    let x = || Expr::anonymous_symbol(SymbolId(7));
    let (u, v) = (
        Expr::anonymous_symbol(SymbolId(30)),
        Expr::anonymous_symbol(SymbolId(31)),
    );
    let cubic = || x() - x() * x() * x() / Expr::int(6);
    let domain = d(-1, 1, -3);
    let request = TaylorRequest {
        center: vec![d(0, 0, 0)],
        domain: vec![domain.clone()],
        order: 4,
        precision: -64,
    };
    let program = |e: Expr, vars: &[u32]| {
        let vars: Vec<_> = vars.iter().map(|&v| SymbolId(v)).collect();
        ExpressionProgram::compile(&[e], &vars, &mut graph()).unwrap()
    };
    vec![
        Case {
            name: "sine_minus_cubic",
            left: model(x().sin(), d(0, 0, 0), domain.clone(), 4),
            right: model(cubic(), d(1, 1, -4), d(-1, 1, -2), 3),
            map: program(u.clone() - v.clone(), &[30, 31]),
            native: program(x().sin() - cubic(), &[7]),
            request: request.clone(),
            width: d(1, 1, -20),
        },
        Case {
            name: "sine_times_cosine",
            left: model(x().sin(), d(0, 0, 0), domain.clone(), 4),
            right: model(x().cos(), d(1, 1, -3), d(-1, 2, -2), 4),
            map: program(u * v, &[30, 31]),
            native: program(x().sin() * x().cos(), &[7]),
            request,
            width: d(1, 1, -2),
        },
    ]
}
fn joined(case: &Case) -> TaylorPolynomialModel {
    case.left
        .clone()
        .join(case.right.clone(), &mut graph(), &mut arithmetic())
        .unwrap()
        .transform(case.map.clone(), &mut graph(), &mut arithmetic())
        .unwrap()
}
fn fresh(case: &Case) -> TaylorModel {
    case.native
        .taylor_model(case.request.clone(), &mut graph(), &mut arithmetic())
        .unwrap()
}
fn width(range: &DyadicInterval) -> DyadicInterval {
    range.width(&mut arithmetic()).unwrap()
}
fn bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("taylor_combination");
    for case in cases() {
        let (j, f) = (joined(&case), fresh(&case));
        // Both paths certify a range no wider than the case contract.
        let allowed = case.width.hull(&d(0, 0, 0), &mut arithmetic()).unwrap();
        for range in [&j.range()[0], &f.image()[0]] {
            assert!(width(range).subset_of(&allowed, &mut arithmetic()).unwrap());
        }
        eprintln!(
            "{}: joined width {:?}, fresh width {:?}",
            case.name,
            width(&j.range()[0])
                .to_rational(&mut arithmetic())
                .unwrap()
                .upper(),
            width(&f.image()[0])
                .to_rational(&mut arithmetic())
                .unwrap()
                .upper()
        );
        group.bench_function(format!("{}/join", case.name), |b| {
            b.iter(|| black_box(joined(black_box(&case))))
        });
        group.bench_function(format!("{}/fresh", case.name), |b| {
            b.iter(|| black_box(fresh(black_box(&case))))
        });
    }
    group.finish();
}
criterion_group!(benches, bench);
criterion_main!(benches);
