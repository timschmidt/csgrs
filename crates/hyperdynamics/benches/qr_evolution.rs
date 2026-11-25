//! Complete fixed-chart and automatic QR flows, including preparation and source projections.
use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{
        CoordinateField,
        qr_evolution::{QrEvolution, QrEvolutionOptions, QrEvolutionStatus},
    },
    evolution::{EvolutionOptions, EvolutionRequest},
    taylor_flow::evolution::{FlowTaylorEvolution, FlowTaylorEvolutionStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
#[path = "../tests/support/flow_oracles.rs"]
mod flow_oracles;
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);
const T: SymbolId = SymbolId(2);
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
struct Fixture {
    name: &'static str,
    chart: CoordinateField<2>,
    request: EvolutionRequest<2>,
    options: QrEvolutionOptions,
    expected: [(Rational, Rational); 2],
    width_limit: Rational,
}
fn fixture(
    name: &'static str,
    outputs: Vec<Expr>,
    initial: [DyadicInterval; 2],
    expected: [(Rational, Rational); 2],
    width_limit: Rational,
) -> Fixture {
    let source = ExpressionProgram::compile(
        &outputs,
        &[Y, T, X],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    let field = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state: vec![X, Y],
        },
        &mut budget(),
    )
    .unwrap();
    let a = vec![
        vec![Real::one(), Real::zero()],
        vec![Real::zero(), Real::one()],
    ];
    let chart = field
        .in_coordinates(
            CoordinateChange {
                variables: vec![X, Y],
                to_source: a.clone(),
                from_source: a,
                origin: vec![Real::zero(); 2],
            },
            &mut budget(),
        )
        .unwrap();
    Fixture {
        name,
        chart,
        request: EvolutionRequest {
            start: d(0, 0, 0),
            end: d(3, 3, -1),
            initial: BoxN::new(initial).unwrap(),
        },
        options: QrEvolutionOptions {
            window: d(3, 3, -2),
            maximum_windows: 2,
            taylor_order: Some(3),
            native: EvolutionOptions {
                maximum_step: Some(d(1, 1, -4)),
                ..Default::default()
            },
            ..Default::default()
        },
        expected,
        width_limit,
    }
}
fn fixtures() -> Vec<Fixture> {
    let p = |a: i64, b: i64| (Rational::from(a), Rational::from(b));
    let t = Rational::from(3) / Rational::from(2);
    let cos = flow_oracles::alternating_trig(&t, false);
    let sin = flow_oracles::alternating_trig(&t, true);
    assert!(cos.0.is_positive() && sin.0.is_positive());
    let mut result = vec![
        fixture(
            "constant",
            vec![Expr::int(2), Expr::int(1)],
            [d(-1, 1, 0), d(0, 0, 0)],
            [
                p(2, 4),
                (
                    Rational::from(3) / Rational::from(2),
                    Rational::from(3) / Rational::from(2),
                ),
            ],
            Rational::from(3),
        ),
        // For initial x in [-1,1], y=0, exact solution is (x*cos(t),x*sin(t)).
        // Independent exact alternating-series outer bounds cover both extremal
        // source points and therefore the complete line-segment input image.
        fixture(
            "rotation_uncertainty",
            vec![-Expr::anonymous_symbol(Y), Expr::anonymous_symbol(X)],
            [d(-1, 1, 0), d(0, 0, 0)],
            [(-&cos.1, cos.1), (-&sin.1, sin.1)],
            Rational::from(10),
        ),
    ];
    let q = |n: i64| Expr::real(Real::from(Rational::from(n) / Rational::from(25)));
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    // Zero is the exact equilibrium. The tilted Jacobian has a larger infinity
    // norm in the original frame; selected QR frames certify longer Picard steps.
    let mut tilted = fixture(
        "tilted_equilibrium",
        vec![q(7) * x.clone() + q(24) * y.clone(), q(24) * x + q(-7) * y],
        [d(0, 0, 0), d(0, 0, 0)],
        [p(0, 0), p(0, 0)],
        Rational::from(1) / Rational::from(1_i64 << 48),
    );
    tilted.request.end = d(7, 7, -2);
    tilted.options.window = d(7, 7, -3);
    tilted.options.native.maximum_step = Some(d(7, 7, -3));
    result.push(tilted);
    result
}
struct FixedRun {
    native: FlowTaylorEvolution<2>,
    initial: BoxN<DyadicInterval, 2>,
    source: Vec<(BoxN<DyadicInterval, 2>, BoxN<DyadicInterval, 2>)>,
}
fn fixed(f: &Fixture) -> FixedRun {
    let mut b = budget();
    let initial = f
        .chart
        .source_enclosure(&f.request.initial, &mut b)
        .unwrap();
    let p = f.chart.transformed_field().flow_taylor(3, &mut b).unwrap();
    let native = p.integrate(f.request.clone(), f.options.native.clone(), &mut b);
    assert!(
        matches!(native.status(), FlowTaylorEvolutionStatus::Completed),
        "{:?}",
        native.status()
    );
    let source = native
        .segments()
        .map(|s| {
            (
                f.chart.source_enclosure(s.tube(), &mut b).unwrap(),
                f.chart.source_enclosure(s.end_enclosure(), &mut b).unwrap(),
            )
        })
        .collect();
    FixedRun {
        native,
        initial,
        source,
    }
}
fn selected(f: &Fixture) -> QrEvolution<2> {
    let r = f
        .chart
        .integrate_qr(f.request.clone(), f.options.clone(), &mut budget());
    assert!(
        matches!(r.status(), QrEvolutionStatus::Completed),
        "{:?}",
        r.status()
    );
    r
}
fn check(f: &Fixture, end: &BoxN<DyadicInterval, 2>) -> Vec<Rational> {
    end.lanes()
        .unwrap()
        .iter()
        .zip(&f.expected)
        .map(|(x, (lo, hi))| {
            let q = x.to_rational(&mut arithmetic()).unwrap();
            assert!(q.contains_point(lo));
            assert!(q.contains_point(hi));
            let width = q.width().unwrap();
            assert!(width < f.width_limit);
            width
        })
        .collect()
}
fn run(c: &mut Criterion) {
    for f in fixtures() {
        let a = fixed(&f);
        let b = selected(&f);
        assert_eq!(a.native.reached_time(), &f.request.end);
        assert_eq!(b.reached_time(), &f.request.end);
        assert_eq!(a.initial, f.request.initial);
        assert_eq!(b.windows().len(), 2);
        let aw = check(&f, &a.source.last().unwrap().1);
        let bw = check(&f, b.source_end_enclosure().unwrap());
        eprintln!(
            "COMPLETED {} fixed_widths={:?} qr_widths={:?}",
            f.name,
            aw.iter()
                .map(Rational::to_f64_enclosure)
                .collect::<Vec<_>>(),
            bw.iter()
                .map(Rational::to_f64_enclosure)
                .collect::<Vec<_>>()
        );
        if let Ok(iterations) = std::env::var("QR_EVOLUTION_ITERATIONS") {
            let iterations: usize = iterations.parse().unwrap();
            for mode in ["fixed", "qr"] {
                for _ in 0..4 {
                    if mode == "fixed" {
                        drop(black_box(fixed(&f)));
                    } else {
                        drop(black_box(selected(&f)));
                    }
                }
                let start = std::time::Instant::now();
                for _ in 0..iterations {
                    if mode == "fixed" {
                        let r = fixed(&f);
                        check(&f, &r.source.last().unwrap().1);
                        drop(black_box(r));
                    } else {
                        let r = selected(&f);
                        check(&f, r.source_end_enclosure().unwrap());
                        drop(black_box(r));
                    }
                }
                println!(
                    "TIME {} {mode} iterations={iterations} ns_per_operation={}",
                    f.name,
                    start.elapsed().as_nanos() / iterations as u128
                );
            }
        } else {
            let mut g = c.benchmark_group(f.name);
            g.bench_function("fixed", |b| {
                b.iter(|| {
                    let r = fixed(black_box(&f));
                    check(&f, &r.source.last().unwrap().1);
                    drop(black_box(r));
                })
            });
            g.bench_function("qr", |b| {
                b.iter(|| {
                    let r = selected(black_box(&f));
                    check(&f, r.source_end_enclosure().unwrap());
                    drop(black_box(r));
                })
            });
            g.finish();
        }
    }
}
criterion_group!(benches, run);
criterion_main!(benches);
