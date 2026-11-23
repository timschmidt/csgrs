//! Same completed identity-map enclosure contract, including complete proof retention.
use criterion::{Criterion, criterion_group, criterion_main};
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
    taylor_flow::{
        FlowTaylorProgram,
        evolution::{FlowTaylorEvolution, FlowTaylorEvolutionStatus},
        noise::{
            FlowNoiseState,
            evolution::{FlowNoiseEvolution, FlowNoiseEvolutionOptions, FlowNoiseEvolutionStatus},
        },
    },
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::{hint::black_box, sync::Arc};
fn d(lo: i64, hi: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), e).unwrap()
}
fn fields(constant: bool) -> [ExpressionField<2>; 2] {
    let (time, x, y) = (SymbolId(10), SymbolId(20), SymbolId(30));
    [1, -1].map(|sign| {
        let rhs = Expr::int(sign)
            * if constant {
                Expr::int(1)
            } else {
                Expr::anonymous_symbol(y)
            };
        let source = ExpressionProgram::compile(
            &[rhs, Expr::int(0)],
            &[y, time, x],
            &mut ExpressionBudget::new(100000, 1000000),
        )
        .unwrap();
        ExpressionField::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x, y],
            },
            &mut EnclosureBudget::new(EnclosurePolicy::default()),
        )
        .unwrap()
    })
}
struct Completed {
    cover: Vec<BoxN<DyadicInterval, 2>>,
    noise: Vec<FlowNoiseEvolution<2>>,
    boxes: Vec<FlowTaylorEvolution<2>>,
}
fn complete(
    programs: &[FlowTaylorProgram<2>; 2],
    noise: bool,
    constant: bool,
    budget: &mut EnclosureBudget,
) -> Completed {
    let mut output = Completed {
        cover: Vec::new(),
        noise: Vec::new(),
        boxes: Vec::new(),
    };
    let cells: usize = if noise || constant { 1 } else { 8 };
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
    let target = BoxN::new([d(-9, 9, -3), d(-1, 1, 0)]).unwrap();
    for cell in 0..cells {
        // These closed y intervals cover [-1,1] without gaps; x remains [-1,1].
        let initial = BoxN::new([
            d(-1, 1, 0),
            d(
                2 * cell as i64 - cells as i64,
                2 * cell as i64 + 2 - cells as i64,
                -(cells.ilog2() as i32),
            ),
        ])
        .unwrap();
        let options = EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        };
        let end = if noise {
            let mut state =
                Arc::new(FlowNoiseState::new(d(0, 0, 0), initial.clone(), budget).unwrap());
            for (index, program) in programs.iter().enumerate() {
                let report = program.integrate_noise(
                    state,
                    d(3 * (index + 1) as i64, 3 * (index + 1) as i64, -3),
                    FlowNoiseEvolutionOptions {
                        maximum_noise: 2,
                        round_coefficients: false,
                        column_cutoff_bits: None,
                        native: options.clone(),
                    },
                    budget,
                );
                assert!(
                    matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
                    "{:?}",
                    report.status()
                );
                state = report.end_state().unwrap().clone();
                output.noise.push(report);
            }
            assert_eq!(state.enclosure(), &initial);
            state.enclosure().clone()
        } else {
            let mut state = initial.clone();
            for (index, program) in programs.iter().enumerate() {
                let report = program.integrate(
                    EvolutionRequest {
                        start: d(3 * index as i64, 3 * index as i64, -3),
                        end: d(3 * (index + 1) as i64, 3 * (index + 1) as i64, -3),
                        initial: state,
                    },
                    options.clone(),
                    budget,
                );
                assert!(
                    matches!(report.status(), FlowTaylorEvolutionStatus::Completed),
                    "{:?}",
                    report.status()
                );
                state = report.end_enclosure().unwrap().clone();
                output.boxes.push(report);
            }
            state
        };
        // Independent map: forward translation/shear followed by its inverse is identity.
        // Every exact initial cell is covered; the whole output stays within 1/8 of it.
        assert!(initial.subset_of(&end, &mut arithmetic).unwrap());
        assert!(end.subset_of(&target, &mut arithmetic).unwrap());
        assert_eq!(end.lanes().unwrap()[1], initial.lanes().unwrap()[1]);
        output.cover.push(end);
    }
    assert_eq!(output.cover.len(), cells);
    assert_eq!(output.noise.len() + output.boxes.len(), 2 * cells);
    output
}
fn benchmark(c: &mut Criterion) {
    for constant in [true, false] {
        let field = fields(constant);
        let prepare = |budget: &mut EnclosureBudget| {
            field.each_ref().map(|f| f.flow_taylor(1, budget).unwrap())
        };
        let prepared = prepare(&mut EnclosureBudget::new(EnclosurePolicy::default()));
        for cold in [false, true] {
            for noise in [false, true] {
                let label = format!(
                    "flow_noise_evolution/{}/{}/{}",
                    if constant { "constant" } else { "shear" },
                    if cold { "cold" } else { "warm" },
                    if noise { "noise" } else { "boxes" }
                );
                c.bench_function(&label, |b| {
                    b.iter(|| {
                        let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
                        let local = cold.then(|| prepare(&mut budget));
                        let programs = local.as_ref().unwrap_or(&prepared);
                        let output = complete(black_box(programs), noise, constant, &mut budget);
                        drop(black_box(output));
                        drop(local);
                    })
                });
            }
        }
    }
}
criterion_group!(benches, benchmark);
criterion_main!(benches);
