//! Complete source-bound circuit query: preparation and reuse are separate costs.
use hypercircuit::{
    CircuitParameter, ComponentId, MnaUnknown, NetId, Real,
    residual::{CircuitResidualProgram, ResidualDisposition, ResidualInput, ResidualInputKind},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::{hint::black_box, time::Instant};
fn g() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn a() -> DyadicBudget {
    DyadicBudget::new(4096, 100_000_000)
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn main() {
    let iterations: u32 = std::env::var("CIRCUIT_RESIDUAL_ITERATIONS")
        .map(|v| v.parse().unwrap())
        .unwrap_or(2_000);
    assert!(iterations > 0);
    let v = Expr::anonymous_symbol(SymbolId(9));
    let r = Expr::anonymous_symbol(SymbolId(2));
    let source = ExpressionProgram::compile(
        &[v / r - Expr::int(1)],
        &[SymbolId(9), SymbolId(2)],
        &mut g(),
    )
    .unwrap();
    let inputs = vec![
        ResidualInput {
            symbol: SymbolId(9),
            kind: ResidualInputKind::Unknown(MnaUnknown::NetVoltage(NetId::new("v").unwrap())),
        },
        ResidualInput {
            symbol: SymbolId(2),
            kind: ResidualInputKind::Parameter {
                component: ComponentId::new("R1").unwrap(),
                parameter: CircuitParameter {
                    name: "resistance".into(),
                    value: Real::one(),
                    unit: "ohm".into(),
                    source: "supplier tolerance".into(),
                },
            },
        },
    ];
    let equations = vec!["R1 KCL (A)".to_owned()];
    let prepare = || {
        CircuitResidualProgram::new(source.clone(), inputs.clone(), equations.clone(), &mut g())
            .unwrap()
    };
    let program = prepare();
    let arguments = [d(3, 4, 0), d(1, 2, 0)];
    let expected = program
        .enclose(&arguments, -16, &mut g(), &mut a())
        .unwrap();
    assert_eq!(expected.residuals(), &[d(1, 6, -1)]);
    assert_eq!(
        expected.disposition(),
        ResidualDisposition::Excluded { equation: 0 }
    );
    assert_eq!(
        expected
            .evidence()
            .matrix::<1, 2>(&mut g(), &mut a())
            .unwrap()
            .intervals()
            .rows(),
        Some(&[[d(1, 8, -2), d(-16, -3, -2)]])
    );
    for candidate in [prepare(), program.clone()] {
        let report = candidate
            .enclose(&arguments, -16, &mut g(), &mut a())
            .unwrap();
        assert_eq!(report.evidence().values(), expected.evidence().values());
        assert_eq!(
            report.circuit().source().definition(),
            expected.circuit().source().definition()
        );
        assert_eq!(report.circuit().inputs(), expected.circuit().inputs());
        assert_eq!(report.circuit().equations(), expected.circuit().equations());
        assert_eq!(report.evidence().arguments(), &arguments);
        assert_eq!(report.disposition(), expected.disposition());
    }
    for mode in ["prepare", "reuse", "prepare_and_query"] {
        for _ in 0..16 {
            drop(
                prepare()
                    .enclose(&arguments, -16, &mut g(), &mut a())
                    .unwrap(),
            );
        }
        let start = Instant::now();
        for _ in 0..iterations {
            if mode == "prepare" {
                drop(black_box(prepare()));
            } else {
                let prepared;
                let selected = if mode == "reuse" {
                    &program
                } else {
                    prepared = prepare();
                    &prepared
                };
                let result = selected
                    .enclose(black_box(&arguments), -16, &mut g(), &mut a())
                    .unwrap();
                // Completed range, proof and retained circuit metadata are materialized.
                assert_eq!(result.evidence().values(), expected.evidence().values());
                assert_eq!(result.disposition(), expected.disposition());
                drop(black_box(result));
            }
        }
        println!(
            "{mode}: iterations={iterations} total_ns={} ns_per_operation={}",
            start.elapsed().as_nanos(),
            start.elapsed().as_nanos() / u128::from(iterations)
        );
    }
}
