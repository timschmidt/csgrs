//! External comparison adapter. No numerical approximation enters the IVP.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
    taylor_flow::{
        FlowTaylorProgram,
        evolution::{FlowTaylorEvolution, FlowTaylorEvolutionStatus},
        noise::{
            FlowNoiseState,
            evolution::{
                FlowNoiseEvolution, FlowNoiseEvolutionOptions, FlowNoiseEvolutionStatus,
            },
        },
    },
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use serde_json::{Value, json};
use std::{hint::black_box, sync::Arc, time::Instant};

fn d(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}
fn exact(value: &DyadicInterval) -> Value {
    let (lo, hi, exponent) = value.bounds().expect("nonempty output");
    json!([lo.to_string(), hi.to_string(), exponent])
}
fn prepare<const N: usize>(case: &str) -> FlowTaylorProgram<N> {
    let t = SymbolId(0);
    let x = SymbolId(1);
    let y = SymbolId(2);
    let z = SymbolId(3);
    let v = Expr::anonymous_symbol(x);
    let outputs = match case {
        "constant" => vec![Expr::int(2)],
        "exponential" => vec![v],
        "riccati" | "riccati_long" => vec![v.clone() * v],
        "oscillator" | "oscillator_long" => vec![Expr::anonymous_symbol(y), -v],
        "time_growth" => vec![Expr::anonymous_symbol(t) * v],
        "turnaround" => vec![Expr::int(1) - Expr::int(8) * Expr::anonymous_symbol(t)],
        "noncommuting" => vec![
            Expr::anonymous_symbol(y),
            Expr::anonymous_symbol(t) * Expr::anonymous_symbol(z),
            Expr::int(0),
        ],
        "two_rate" => vec![-v, Expr::int(-8) * Expr::anonymous_symbol(y)],
        _ => unreachable!(),
    };
    let states = [x, y, z][..N].to_vec();
    let bindings: Vec<_> = std::iter::once(t).chain(states.iter().copied()).collect();
    let source = ExpressionProgram::compile(
        &outputs,
        &bindings,
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<N>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: t,
            state: states,
        },
        &mut budget,
    )
    .unwrap();
    field.flow_taylor(3, &mut budget).unwrap()
}
/// Long-horizon cases use correlated affine-noise evolution with its default
/// options; the original cases keep Cartesian whole-horizon evolution.
fn long(case: &str) -> bool {
    case.ends_with("_long")
}
/// Exact horizon `[0, 2^e]` and step cap `2^s`.
fn horizon(case: &str) -> (i32, i32) {
    match case {
        "oscillator_long" => (3, -3),
        "riccati_long" => (-1, -4),
        "two_rate" => (-2, -6),
        _ => (-2, -4),
    }
}
enum Report<const N: usize> {
    Cartesian(FlowTaylorEvolution<N>),
    Noise(FlowNoiseEvolution<N>),
}
struct Outcome<const N: usize> {
    report: Report<N>,
    initial: [DyadicInterval; N],
    end: [DyadicInterval; N],
    tube: [DyadicInterval; N],
    times: Vec<DyadicInterval>,
    graph_work: usize,
    scalar_work: u64,
}
fn complete<const N: usize>(
    program: &FlowTaylorProgram<N>,
    case: &str,
    uncertain: bool,
) -> Outcome<N> {
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let radius = i64::from(uncertain);
    let initial = BoxN::new(std::array::from_fn(|i| {
        let center = if i == 0 || case == "two_rate" || (case == "noncommuting" && i == 2) {
            1024
        } else {
            0
        };
        d(center - radius, center + radius, -10)
    }))
    .unwrap();
    let (end_exponent, step_exponent) = horizon(case);
    let end_time = d(1, 1, end_exponent);
    let options = EvolutionOptions {
        maximum_step: Some(d(1, 1, step_exponent)),
        ..Default::default()
    };
    let mut times = vec![d(0, 0, 0)];
    let mut pieces: Vec<BoxN<DyadicInterval, N>> = Vec::new();
    let (report, end) = if long(case) {
        let state =
            Arc::new(FlowNoiseState::new(d(0, 0, 0), initial.clone(), &mut budget).unwrap());
        let report = program.integrate_noise(
            state,
            end_time.clone(),
            FlowNoiseEvolutionOptions {
                native: options,
                ..Default::default()
            },
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
            "{:?}",
            report.status()
        );
        for segment in report.segments() {
            times.push(segment.end_state().unwrap().time().clone());
            pieces.push(segment.evidence().flow().unwrap().tube().unwrap().clone());
        }
        let end = report.end_state().unwrap().enclosure().clone();
        (Report::Noise(report), end)
    } else {
        let report = program.integrate(
            EvolutionRequest {
                start: d(0, 0, 0),
                end: end_time.clone(),
                initial: initial.clone(),
            },
            options,
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowTaylorEvolutionStatus::Completed),
            "{:?}",
            report.status()
        );
        assert_eq!(report.reached_time(), &end_time);
        for segment in report.segments() {
            let proposal = segment.picard().proposal();
            assert_eq!(&proposal.start, times.last().unwrap());
            times.push(proposal.end.clone());
            pieces.push(segment.tube().clone());
        }
        let end = report.end_enclosure().unwrap().clone();
        (Report::Cartesian(report), end)
    };
    let mut tube = initial.lanes().unwrap().clone();
    let mut publication = DyadicBudget::new(16384, 1_000_000_000);
    for (window, piece) in times.windows(2).zip(&pieces) {
        assert!(
            window[1]
                .sub(&window[0], &mut publication)
                .unwrap()
                .subset_of(&d(0, 1, step_exponent), &mut publication)
                .unwrap()
        );
        assert_ne!(window[0], window[1]);
        for (total, lane) in tube.iter_mut().zip(piece.lanes().unwrap()) {
            *total = total.hull(lane, &mut publication).unwrap();
        }
    }
    assert_eq!(times.last().unwrap(), &end_time);
    Outcome {
        report,
        initial: initial.lanes().unwrap().clone(),
        end: end.lanes().unwrap().clone(),
        tube,
        times,
        graph_work: budget.policy().graph_work - budget.remaining_graph_work(),
        scalar_work: budget.policy().scalar_work - budget.remaining_scalar_work(),
    }
}
fn record<const N: usize>(case: &str, uncertain: bool, out: &Outcome<N>) -> Value {
    let (method, pieces) = match &out.report {
        Report::Cartesian(report) => ("cartesian", report.segments().len()),
        Report::Noise(report) => ("affine_noise", report.segments().len()),
    };
    assert_eq!(pieces + 1, out.times.len());
    json!({
        "kind": "certificate", "solver": "hyperdynamics", "case": format!("{case}_{}", if uncertain {"box"} else {"point"}),
        "status": "certified", "encoding": "dyadic", "order": 3, "method": method,
        "initial": out.initial.iter().map(exact).collect::<Vec<_>>(),
        "start": exact(&d(0,0,0)), "end_time": exact(out.times.last().unwrap()),
        "end": out.end.iter().map(exact).collect::<Vec<_>>(),
        "tube": out.tube.iter().map(exact).collect::<Vec<_>>(), "pieces": pieces,
        "time_domains": out.times.iter().map(exact).collect::<Vec<_>>(),
        "graph_work": out.graph_work,
        "scalar_work": out.scalar_work,
    })
}
fn benchmark<const N: usize>(case: &str, uncertain: bool, iterations: usize, samples: usize) {
    let prepared = prepare::<N>(case);
    let expected = record(case, uncertain, &complete(&prepared, case, uncertain));
    println!("{expected}");
    for mode in ["reuse", "fresh"] {
        let run = || {
            if mode == "fresh" {
                let program = prepare::<N>(black_box(case));
                drop(black_box(complete(&program, case, uncertain)));
                drop(black_box(program));
            } else {
                drop(black_box(complete(&prepared, case, uncertain)));
            }
        };
        for _ in 0..8 {
            run();
        }
        for sample in 0..samples {
            let start = Instant::now();
            for _ in 0..iterations {
                run();
            }
            let ns = start.elapsed().as_nanos();
            println!(
                "{}",
                json!({"kind":"timing", "solver":"hyperdynamics",
                "case":expected["case"], "mode":mode, "sample":sample,
                "iterations":iterations, "total_ns":ns})
            );
        }
        // Exact outputs must remain stable across warm reuse and fresh preparation.
        assert_eq!(
            record(case, uncertain, &complete(&prepared, case, uncertain)),
            expected
        );
        assert_eq!(
            record(
                case,
                uncertain,
                &complete(&prepare::<N>(case), case, uncertain)
            ),
            expected
        );
    }
}
fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    assert!(
        args.is_empty() || args.len() == 2,
        "usage: [iterations samples]"
    );
    let iterations = args.first().map_or(1, |s| s.parse::<usize>().unwrap());
    let samples = args.get(1).map_or(0, |s| s.parse::<usize>().unwrap());
    assert!((1..=100000).contains(&iterations) && samples <= 100);
    for case in [
        "constant",
        "exponential",
        "riccati",
        "oscillator",
        "time_growth",
        "turnaround",
        "noncommuting",
        "two_rate",
        "oscillator_long",
        "riccati_long",
    ] {
        for uncertain in [false, true] {
            if case == "noncommuting" {
                benchmark::<3>(case, uncertain, iterations, samples);
            } else if case.starts_with("oscillator") || case == "two_rate" {
                benchmark::<2>(case, uncertain, iterations, samples);
            } else {
                benchmark::<1>(case, uncertain, iterations, samples);
            }
        }
    }
}
