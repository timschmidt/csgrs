//! Complete `taylor_model` construction across polynomial orders.
//!
//! Each iteration builds the whole certified model: regularity proof, exact
//! centered coefficients, every degree-(n+1) remainder contribution and the
//! final image. Higher orders expose the cost of the coefficient method.
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram, TaylorRequest},
};
use std::hint::black_box;
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(10_000_000, 10_000_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000_000)
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn s(id: u32) -> Expr {
    Expr::anonymous_symbol(SymbolId(id))
}
struct Case {
    name: &'static str,
    program: ExpressionProgram,
    center: Vec<DyadicInterval>,
    domain: Vec<DyadicInterval>,
    orders: &'static [usize],
}
fn cases() -> Vec<Case> {
    let (x, y) = (s(7), s(8));
    let one = || Expr::int(1);
    let two = || Expr::int(2);
    // Coordinate of a rational quadratic Bezier span with weights 1, 1/2, 1.
    let t = || x.clone();
    let b0 = || (one() - t()) * (one() - t());
    let b1 = || two() * t() * (one() - t());
    let b2 = || t() * t();
    let conic = (b0() + b1() / two()) / (b0() + b1() / two() + b2());
    let compile = |expr: Expr, ids: &[u32]| {
        let ids: Vec<_> = ids.iter().map(|&id| SymbolId(id)).collect();
        ExpressionProgram::compile(&[expr], &ids, &mut graph()).unwrap()
    };
    let quarter = || vec![d(-1, 1, -2)];
    vec![
        Case {
            name: "reciprocal",
            program: compile(one() / (one() + x.clone()), &[7]),
            center: vec![d(0, 0, 0)],
            domain: quarter(),
            orders: &[2, 4, 6, 8],
        },
        Case {
            name: "rational_conic",
            program: compile(conic, &[7]),
            center: vec![d(1, 1, -2)],
            domain: vec![d(0, 1, -1)],
            orders: &[2, 4, 6, 8],
        },
        Case {
            name: "sine_cosine",
            program: compile(x.clone().sin() * x.clone().cos(), &[7]),
            center: vec![d(0, 0, 0)],
            domain: quarter(),
            orders: &[2, 4, 6, 8],
        },
        Case {
            name: "square_root",
            program: compile((one() + x.clone()).sqrt(), &[7]),
            center: vec![d(0, 0, 0)],
            domain: quarter(),
            orders: &[2, 4, 6, 8],
        },
        Case {
            name: "bivariate_quotient_sine",
            program: compile(
                x.clone() / (y.clone() + two()) + (x.clone() * y.clone()).sin(),
                &[7, 8],
            ),
            center: vec![d(0, 0, 0); 2],
            domain: vec![d(-1, 1, -2); 2],
            orders: &[2, 4, 6],
        },
    ]
}
fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("taylor_model_orders");
    group.sample_size(10);
    for case in cases() {
        for &order in case.orders {
            let request = TaylorRequest {
                center: case.center.clone(),
                domain: case.domain.clone(),
                order,
                precision: -64,
            };
            let model = case
                .program
                .taylor_model(request.clone(), &mut graph(), &mut arithmetic())
                .unwrap();
            assert!(!model.image()[0].is_empty());
            group.bench_with_input(
                BenchmarkId::new(case.name, order),
                &request,
                |b, request| {
                    b.iter(|| {
                        black_box(
                            case.program
                                .taylor_model(request.clone(), &mut graph(), &mut arithmetic())
                                .unwrap(),
                        )
                    })
                },
            );
        }
    }
    group.finish();
}
criterion_group!(benches, benchmarks);
criterion_main!(benches);
