//! Fresh complete order portfolio, including all preparation, proofs and destruction.
use criterion::{Criterion, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    picard::PicardProposal,
    taylor_flow::{FlowTaylorReport, FlowTaylorStatus},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::hint::black_box;
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn field(case: usize) -> ExpressionField<1> {
    let t = SymbolId(0);
    let x = SymbolId(1);
    let expr = Expr::anonymous_symbol(x);
    let rhs = match case {
        0 => Expr::int(2),
        1 => expr,
        _ => expr.clone() * expr,
    };
    let source = ExpressionProgram::compile(
        &[rhs],
        &[x, t],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: t,
            state: vec![x],
        },
        &mut budget(),
    )
    .unwrap()
}
fn proposal() -> PicardProposal<1> {
    PicardProposal {
        start: d(0, 0, 0),
        end: d(1, 1, -3),
        initial: BoxN::new([d(1, 1, 0)]).unwrap(),
        candidate: BoxN::new([d(1, 2, 0)]).unwrap(),
    }
}
fn complete(
    f: &ExpressionField<1>,
    reuse: bool,
    b: &mut EnclosureBudget,
) -> Vec<FlowTaylorReport<1>> {
    let high = if reuse {
        Some(f.flow_taylor(3, b).unwrap())
    } else {
        None
    };
    (0..=3)
        .map(|order| {
            let p = if let Some(high) = &high {
                high.restrict_order(order, b).unwrap()
            } else {
                f.flow_taylor(order, b).unwrap()
            };
            let r = p.certify_step(proposal(), b);
            assert!(matches!(r.status(), FlowTaylorStatus::Certified));
            assert_eq!(r.evidence().coefficients().len(), order);
            assert_eq!(r.evidence().end_terms().len(), order + 1);
            assert!(r.evidence().remainder().is_some());
            r
        })
        .collect()
}
fn benchmark(c: &mut Criterion) {
    for (case, name) in ["constant", "exponential", "riccati"]
        .into_iter()
        .enumerate()
    {
        let f = field(case);
        let expected = complete(&f, false, &mut budget());
        let shared = complete(&f, true, &mut budget());
        assert_eq!(format!("{expected:?}"), format!("{shared:?}"));
        for reuse in [false, true] {
            let label = if reuse { "shared" } else { "independent" };
            c.bench_function(&format!("taylor_orders/{name}/{label}"), |b| {
                b.iter(|| {
                    let reports = complete(black_box(&f), reuse, &mut budget());
                    for (actual, expected) in reports.iter().zip(&expected) {
                        assert_eq!(actual.end_enclosure(), expected.end_enclosure());
                        assert_eq!(actual.tube(), expected.tube());
                    }
                    drop(black_box(reports));
                })
            });
        }
    }
}
criterion_group!(benches, benchmark);
criterion_main!(benches);
