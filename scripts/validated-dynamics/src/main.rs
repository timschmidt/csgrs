//! External comparison adapter. No numerical approximation enters the IVP.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
    taylor_flow::{
        FlowTaylorProgram,
        evolution::{FlowTaylorEvolution, FlowTaylorEvolutionStatus},
    },
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use serde_json::{Value, json};
use std::{hint::black_box, time::Instant};

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
    let v = Expr::anonymous_symbol(x);
    let outputs = match case {
        "constant" => vec![Expr::int(2)],
        "exponential" => vec![v],
        "riccati" => vec![v.clone() * v],
        "oscillator" => vec![Expr::anonymous_symbol(y), -v],
        _ => unreachable!(),
    };
    let states = [x, y][..N].to_vec();
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
struct Outcome<const N: usize> {
    report: FlowTaylorEvolution<N>,
    tube: [DyadicInterval; N],
    graph_work: usize,
    scalar_work: u64,
}
fn complete<const N: usize>(program: &FlowTaylorProgram<N>, uncertain: bool) -> Outcome<N> {
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let radius = i64::from(uncertain);
    let initial = BoxN::new(std::array::from_fn(|i| {
        let center = if i == 0 { 1024 } else { 0 };
        d(center - radius, center + radius, -10)
    }))
    .unwrap();
    let report = program.integrate(
        EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, -2),
            initial: initial.clone(),
        },
        EvolutionOptions {
            maximum_step: Some(d(1, 1, -4)),
            ..Default::default()
        },
        &mut budget,
    );
    assert!(
        matches!(report.status(), FlowTaylorEvolutionStatus::Completed),
        "{:?}",
        report.status()
    );
    assert_eq!(report.reached_time(), &d(1, 1, -2));
    let mut tube = initial.lanes().unwrap().clone();
    let mut publication = DyadicBudget::new(16384, 1_000_000_000);
    let mut time = d(0, 0, 0);
    for segment in report.segments() {
        let proposal = segment.picard().proposal();
        assert_eq!(proposal.start, time);
        assert_ne!(proposal.end, time);
        assert!(
            proposal
                .end
                .sub(&time, &mut publication)
                .unwrap()
                .subset_of(&d(0, 1, -4), &mut publication)
                .unwrap()
        );
        time = proposal.end.clone();
        for (total, piece) in tube.iter_mut().zip(segment.tube().lanes().unwrap()) {
            *total = total.hull(piece, &mut publication).unwrap();
        }
    }
    assert_eq!(time, d(1, 1, -2));
    Outcome {
        report,
        tube,
        graph_work: budget.policy().graph_work - budget.remaining_graph_work(),
        scalar_work: budget.policy().scalar_work - budget.remaining_scalar_work(),
    }
}
fn record<const N: usize>(case: &str, uncertain: bool, out: &Outcome<N>) -> Value {
    let report = &out.report;
    json!({
        "kind": "certificate", "solver": "hyperdynamics", "case": format!("{case}_{}", if uncertain {"box"} else {"point"}),
        "status": "certified", "encoding": "dyadic", "order": 3,
        "initial": report.request().initial.lanes().unwrap().iter().map(exact).collect::<Vec<_>>(),
        "start": exact(&d(0,0,0)), "end_time": exact(report.reached_time()),
        "end": report.end_enclosure().unwrap().lanes().unwrap().iter().map(exact).collect::<Vec<_>>(),
        "tube": out.tube.iter().map(exact).collect::<Vec<_>>(), "pieces": report.segments().len(),
        "time_domains": std::iter::once(exact(&d(0,0,0))).chain(report.segments().map(|s|exact(&s.picard().proposal().end))).collect::<Vec<_>>(),
        "graph_work": out.graph_work,
        "scalar_work": out.scalar_work,
    })
}
fn benchmark<const N: usize>(case: &str, uncertain: bool, iterations: usize, samples: usize) {
    let prepared = prepare::<N>(case);
    let expected = record(case, uncertain, &complete(&prepared, uncertain));
    println!("{expected}");
    for mode in ["reuse", "fresh"] {
        let run = || {
            if mode == "fresh" {
                let program = prepare::<N>(black_box(case));
                drop(black_box(complete(&program, uncertain)));
                drop(black_box(program));
            } else {
                drop(black_box(complete(&prepared, uncertain)));
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
            record(case, uncertain, &complete(&prepared, uncertain)),
            expected
        );
        assert_eq!(
            record(case, uncertain, &complete(&prepare::<N>(case), uncertain)),
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
    for case in ["constant", "exponential", "riccati", "oscillator"] {
        for uncertain in [false, true] {
            if case == "oscillator" {
                benchmark::<2>(case, uncertain, iterations, samples);
            } else {
                benchmark::<1>(case, uncertain, iterations, samples);
            }
        }
    }
}
