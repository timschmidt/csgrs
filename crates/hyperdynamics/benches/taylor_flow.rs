//! Completed flow proof for x'=x, x(0)=1, horizon 1/4, endpoint width <= 1/8192.
//! Every method retains all required reports and includes output destruction.
//! Exact field/derivative preparation is outside repeated-query measurements.
use criterion::{Criterion, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    picard::{PicardProposal, PicardReport, PicardStatus},
    taylor_flow::{FlowTaylorProgram, FlowTaylorReport, FlowTaylorStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::Rational;
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
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn field() -> ExpressionField<1> {
    let x = SymbolId(1);
    let t = SymbolId(0);
    let source = ExpressionProgram::compile(
        &[Expr::anonymous_symbol(x)],
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
        end: d(1, 1, -2),
        initial: BoxN::new([d(1, 1, 0)]).unwrap(),
        candidate: BoxN::new([d(1, 2, 0)]).unwrap(),
    }
}
fn taylor(program: &FlowTaylorProgram<1>) -> FlowTaylorReport<1> {
    let result = program.certify_step(proposal(), &mut budget());
    assert!(matches!(result.status(), FlowTaylorStatus::Certified));
    result
}
fn picard(field: &ExpressionField<1>, count: i64, exponent: i32) -> Vec<PicardReport<1>> {
    // For this linear fixture, hull(X0, X0/(1-h)) is a tightly proposed
    // positive self-map box. The generic Picard verifier checks it independently.
    // Outward quotient and transport arithmetic are charged to a shared native
    // budget; the field proof budget is also shared across the whole horizon.
    let mut native = budget();
    let mut arithmetic = scalar();
    let mut initial = BoxN::new([d(1, 1, 0)]).unwrap();
    let mut reports = Vec::new();
    let denominator = d(1, 1, 0).sub(&d(1, 1, exponent), &mut arithmetic).unwrap();
    for i in 0..count {
        let lane = &initial.lanes().unwrap()[0];
        let quotient = lane.div(&denominator, -64, &mut arithmetic).unwrap();
        let widened = lane.hull(&quotient, &mut arithmetic).unwrap();
        let report = field.certify_step(
            PicardProposal {
                start: d(i, i, exponent),
                end: d(i + 1, i + 1, exponent),
                initial,
                candidate: BoxN::new([widened]).unwrap(),
            },
            &mut native,
        );
        assert!(
            matches!(report.status(), PicardStatus::Certified),
            "step {i}: {:?}",
            report.status()
        );
        let endpoint = report.end_enclosure().unwrap();
        // Outward transport bounds denominator growth without altering the
        // represented source. Every carried box contains its certified endpoint,
        // and the next report retains that carried box as its exact initial set.
        initial = endpoint.regrid(-64, &mut arithmetic).unwrap();
        assert!(endpoint.subset_of(&initial, &mut arithmetic).unwrap());
        reports.push(report);
    }
    reports
}
fn validate(name: &str, end: &BoxN<DyadicInterval, 1>) {
    let value = end.lanes().unwrap()[0].to_rational(&mut scalar()).unwrap();
    let mut term = Rational::from(1);
    let mut lower = term.clone();
    for k in 1..=10 {
        term = term / (Rational::from(4) * Rational::from(k));
        lower = lower + &term;
    }
    let upper = &lower
        + (&term / Rational::from(44))
            / (Rational::from(1) - Rational::from(1) / Rational::from(48));
    assert!(value.contains_point(&lower));
    assert!(value.contains_point(&upper));
    let width = value.upper().unwrap() - value.lower().unwrap();
    assert!(
        width <= Rational::from(1) / Rational::from(8192),
        "{name}: {width:?}"
    );
    eprintln!("COMPLETED {name} end={end:?} width={width:?}");
}
fn benchmarks(c: &mut Criterion) {
    let field = field();
    let program = field.flow_taylor(3, &mut budget()).unwrap();
    let report = taylor(&program);
    validate("taylor3", report.end_enclosure().unwrap());
    let coarse = picard(&field, 512, -11);
    let bounds = coarse
        .last()
        .unwrap()
        .end_enclosure()
        .unwrap()
        .lanes()
        .unwrap()[0]
        .to_rational(&mut scalar())
        .unwrap();
    let width = bounds.upper().unwrap() - bounds.lower().unwrap();
    assert!(width > Rational::from(1) / Rational::from(8192));
    eprintln!("UNDERRESOLVED picard512 width={width:?}");
    drop(coarse);
    let reports = picard(&field, 1024, -12);
    validate(
        "picard1024",
        reports.last().unwrap().end_enclosure().unwrap(),
    );
    assert_eq!(reports.last().unwrap().proposal().end, d(1, 1, -2));
    drop((report, reports));
    let mut group = c.benchmark_group("flow_endpoint_width_1_over_8192");
    group.bench_function("taylor3", |b| {
        b.iter(|| drop(black_box(taylor(black_box(&program)))))
    });
    group.bench_function("picard1024", |b| {
        b.iter(|| drop(black_box(picard(black_box(&field), 1024, -12))))
    });
    group.finish();
    c.bench_function("flow_taylor_prepare_order3", |b| {
        b.iter(|| {
            drop(black_box(
                black_box(&field).flow_taylor(3, &mut budget()).unwrap(),
            ))
        })
    });
}
criterion_group!(benches, benchmarks);
criterion_main!(benches);
