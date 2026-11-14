//! Completed source-coordinate flow proofs in identity and exact QR frames.
use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{
        CoordinateEvolution, CoordinateEvolutionStatus, CoordinateField, QrCoordinateField,
    },
    evolution::{EvolutionOptions, EvolutionRequest},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);
const T: SymbolId = SymbolId(2);
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn d(l: i64, u: i64) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), 0).unwrap()
}
fn basis() -> Vec<Vec<Real>> {
    vec![vec![3.into(), (-4).into()], vec![4.into(), 3.into()]]
}
fn ratio(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
struct Fixture {
    name: &'static str,
    source: ExpressionField<2>,
    identity: CoordinateField<2>,
    qr: QrCoordinateField<2>,
    horizon: DyadicInterval,
    expected: BoxN<DyadicInterval, 2>,
    segments: [usize; 2],
}
fn fixture(
    name: &'static str,
    outputs: Vec<Expr>,
    horizon: DyadicInterval,
    expected: [i64; 2],
    segments: [usize; 2],
) -> Fixture {
    let p = ExpressionProgram::compile(
        &outputs,
        &[Y, T, X],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    let source = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: p.definition().clone(),
            time: T,
            state: vec![X, Y],
        },
        &mut budget(),
    )
    .unwrap();
    let identity = source
        .in_coordinates(
            CoordinateChange {
                variables: vec![X, Y],
                to_source: vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]],
                from_source: vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]],
                origin: vec![0.into(), 0.into()],
            },
            &mut budget(),
        )
        .unwrap();
    let qr = prepare(&source);
    Fixture {
        name,
        source,
        identity,
        qr,
        horizon,
        expected: BoxN::new(expected.map(|x| d(x, x))).unwrap(),
        segments,
    }
}
fn fixtures() -> Vec<Fixture> {
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    vec![
        fixture(
            "rotated_equilibrium",
            vec![
                Expr::real(ratio(-7, 25)) * x.clone() + Expr::real(ratio(24, 25)) * y.clone(),
                Expr::real(ratio(24, 25)) * x + Expr::real(ratio(7, 25)) * y,
            ],
            DyadicInterval::point(7.into(), -3).unwrap(),
            [0, 0],
            [2, 1],
        ),
        fixture(
            "constant_flow",
            vec![Expr::int(2), Expr::int(1)],
            d(1, 1),
            [2, 1],
            [1, 1],
        ),
    ]
}
fn prepare(source: &ExpressionField<2>) -> QrCoordinateField<2> {
    source
        .in_qr_coordinates(basis(), vec![0.into(), 0.into()], &mut budget())
        .unwrap()
}
fn run(chart: &CoordinateField<2>, horizon: &DyadicInterval) -> CoordinateEvolution<2> {
    let report = chart.integrate(
        EvolutionRequest {
            start: d(0, 0),
            end: horizon.clone(),
            initial: BoxN::new([d(0, 0), d(0, 0)]).unwrap(),
        },
        EvolutionOptions::default(),
        &mut budget(),
    );
    assert!(
        matches!(report.status(), CoordinateEvolutionStatus::Completed),
        "{:?}",
        report.status()
    );
    report
}
fn check(f: &Fixture) {
    for (chart, count) in [(&f.identity, f.segments[0]), (f.qr.chart(), f.segments[1])] {
        let report = run(chart, &f.horizon);
        assert_eq!(report.native().reached_time(), &f.horizon);
        assert_eq!(report.segments().len(), count);
        let output = report.source_end_enclosure().unwrap();
        let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
        // Independent exact source flows: equilibrium at zero, or (2t,t).
        assert!(f.expected.subset_of(output, &mut arithmetic).unwrap());
        for width in output.widths(&mut arithmetic).unwrap().unwrap() {
            assert!(
                width
                    .compare_singleton(
                        &DyadicInterval::point(1.into(), -48).unwrap(),
                        &mut arithmetic
                    )
                    .unwrap()
                    != std::cmp::Ordering::Greater
            );
        }
        eprintln!(
            "COMPLETED {} segments={} attempts={} output={:?}",
            f.name,
            count,
            report.native().attempts().len(),
            output
        );
    }
}
fn benchmarks(c: &mut Criterion) {
    for f in fixtures() {
        check(&f);
        let mut group = c.benchmark_group(f.name);
        group.bench_function("identity", |b| {
            b.iter(|| {
                drop(black_box(run(
                    black_box(&f.identity),
                    black_box(&f.horizon),
                )))
            })
        });
        group.bench_function("qr", |b| {
            b.iter(|| {
                drop(black_box(run(
                    black_box(f.qr.chart()),
                    black_box(&f.horizon),
                )))
            })
        });
        group.bench_function("prepare_qr", |b| {
            b.iter(|| drop(black_box(prepare(black_box(&f.source)))))
        });
        group.finish();
    }
}
criterion_group!(benches, benchmarks);
criterion_main!(benches);
