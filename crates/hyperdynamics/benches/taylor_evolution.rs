//! Complete source flows with independently checked endpoint quality.
use criterion::{Criterion, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus},
    reachability::{
        ReachabilityPolicy, ReachabilityReport, ReachabilityRequest, ReachabilityStatus,
    },
    taylor_flow::{
        FlowTaylorProgram,
        evolution::{FlowTaylorEvolution, FlowTaylorEvolutionStatus},
    },
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
    EnclosureBudget::new(EnclosurePolicy {
        scalar_bits: 1_048_576,
        scalar_work: 100_000_000_000_000,
        ..EnclosurePolicy::default()
    })
}
struct Fixture {
    name: &'static str,
    field: ExpressionField<1>,
    program: FlowTaylorProgram<1>,
    initial: i64,
    native_exponent: i32,
}
fn fixtures() -> Vec<Fixture> {
    let x = SymbolId(1);
    let t = SymbolId(0);
    [
        ("exponential", Expr::anonymous_symbol(x), 1, -12),
        ("constant", Expr::int(2), 0, -2),
    ]
    .into_iter()
    .map(|(name, expr, initial, native_exponent)| {
        let source = ExpressionProgram::compile(
            &[expr],
            &[x, t],
            &mut ExpressionBudget::new(1_000_000, 100_000_000),
        )
        .unwrap();
        let field = ExpressionField::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time: t,
                state: vec![x],
            },
            &mut budget(),
        )
        .unwrap();
        let program = prepare(&field);
        Fixture {
            name,
            field,
            program,
            initial,
            native_exponent,
        }
    })
    .collect()
}
fn prepare(field: &ExpressionField<1>) -> FlowTaylorProgram<1> {
    field.flow_taylor(3, &mut budget()).unwrap()
}
fn request(f: &Fixture) -> EvolutionRequest<1> {
    EvolutionRequest {
        start: d(0, 0, 0),
        end: d(1, 1, -1),
        initial: BoxN::new([d(f.initial, f.initial, 0)]).unwrap(),
    }
}
fn options(exponent: i32) -> EvolutionOptions {
    EvolutionOptions {
        maximum_step: Some(d(1, 1, exponent)),
        maximum_steps: 16384,
        ..Default::default()
    }
}
fn native(f: &Fixture, work: &mut EnclosureBudget) -> EvolutionReport<1> {
    let r = f
        .field
        .integrate(request(f), options(f.native_exponent), work);
    assert!(
        matches!(r.status(), EvolutionStatus::Completed),
        "{:?}",
        r.status()
    );
    r
}
fn taylor(f: &Fixture, work: &mut EnclosureBudget) -> FlowTaylorEvolution<1> {
    let r = f.program.integrate(request(f), options(-2), work);
    assert!(
        matches!(r.status(), FlowTaylorEvolutionStatus::Completed),
        "{:?}",
        r.status()
    );
    r
}
fn validate(f: &Fixture, end: &BoxN<DyadicInterval, 1>) {
    let mut b = DyadicBudget::new(1_048_576, 100_000_000_000_000);
    let value = end.lanes().unwrap()[0].to_rational(&mut b).unwrap();
    if f.name == "constant" {
        assert_eq!(end, &BoxN::new([d(1, 1, 0)]).unwrap());
    } else {
        let mut term = Rational::from(1);
        let mut lower = term.clone();
        for k in 1..=10 {
            term = term / Rational::from(2 * k);
            lower = lower + &term;
        }
        let upper = &lower
            + (&term / Rational::from(22))
                / (Rational::from(1) - Rational::from(1) / Rational::from(24));
        assert!(value.contains_point(&lower));
        assert!(value.contains_point(&upper));
        let width = value.upper().unwrap() - value.lower().unwrap();
        assert!(
            width <= Rational::from(1) / Rational::from(512),
            "{} width={width:?}",
            f.name
        );
    }
    eprintln!("COMPLETED {} end={end:?}", f.name);
}
// Existing checked regridding plus displacement proposals, including the
// complete single-cell epsilon-cover certificate and every native prefix.
fn reachability(f: &Fixture, work: &mut EnclosureBudget) -> ReachabilityReport<1> {
    let r = f.field.reach(
        ReachabilityRequest {
            evolution: request(f),
            epsilon: if f.name == "constant" {
                d(0, 0, 0)
            } else {
                d(1, 1, -9)
            },
        },
        ReachabilityPolicy {
            // The retained cap probe rejects -10 and admits -11 at width 1/512.
            // Constant flow keeps the same 1/4 cap in every strategy.
            evolution: options(if f.name == "constant" {
                f.native_exponent
            } else {
                -11
            }),
            maximum_cells: 1,
            maximum_evolutions: 1,
            maximum_depth: 0,
            maximum_time_refinements: 0,
        },
        work,
    );
    assert!(
        matches!(r.status(), ReachabilityStatus::Completed),
        "{:?}",
        r.status()
    );
    r
}
fn benchmarks(c: &mut Criterion) {
    for f in fixtures() {
        let a = native(&f, &mut budget());
        let b = taylor(&f, &mut budget());
        validate(&f, a.end_enclosure().unwrap());
        validate(&f, b.end_enclosure().unwrap());
        assert_eq!(a.reached_time(), &request(&f).end);
        assert_eq!(b.reached_time(), &request(&f).end);
        eprintln!(
            "SEGMENTS {} native={} taylor={}",
            f.name,
            a.segments().len(),
            b.segments().len()
        );
        let cover = reachability(&f, &mut budget());
        let end = cover.end_cover().unwrap();
        assert_eq!(end.boxes().len(), 1);
        assert_eq!(end.end_time(), &request(&f).end);
        validate(&f, end.boxes().next().unwrap());
        assert_eq!(cover.cells().len(), 1);
        assert_eq!(cover.attempts().len(), 1);
        assert_eq!(
            cover.attempts()[0].evolution().steps().len(),
            if f.name == "constant" { 2 } else { 1024 }
        );
        drop((a, b, cover));
        let mut group = c.benchmark_group(f.name);
        group.bench_function("picard", |b| {
            b.iter(|| drop(black_box(native(black_box(&f), &mut budget()))))
        });
        group.bench_function("taylor", |b| {
            b.iter(|| drop(black_box(taylor(black_box(&f), &mut budget()))))
        });
        group.bench_function("prepare_taylor3", |b| {
            b.iter(|| drop(black_box(prepare(black_box(&f.field)))))
        });
        group.bench_function("reachability", |b| {
            b.iter(|| drop(black_box(reachability(black_box(&f), &mut budget()))))
        });
        group.finish();
    }
}
criterion_group!(benches, benchmarks);
criterion_main!(benches);
