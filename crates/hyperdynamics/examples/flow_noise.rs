//! Piecewise shear and reverse shear, retaining their shared uncertainty.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    taylor_flow::noise::{FlowNoiseRequest, FlowNoiseState, FlowNoiseStatus},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::sync::Arc;

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let time = SymbolId(10);
    let x = SymbolId(20);
    let y = SymbolId(30);
    let interval = |lower: i64, upper: i64, exponent| {
        DyadicInterval::new(lower.into(), upper.into(), exponent)
    };
    let initial = BoxN::new([interval(-1, 1, 0)?, interval(-1, 1, 0)?])?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let mut state = Arc::new(FlowNoiseState::new(
        interval(0, 0, 0)?,
        initial.clone(),
        &mut budget,
    )?);
    for (step, sign) in [(0, 1), (1, -1)] {
        let source = ExpressionProgram::compile(
            &[Expr::int(sign) * Expr::anonymous_symbol(y), Expr::int(0)],
            &[y, time, x],
            &mut ExpressionBudget::new(100_000, 1_000_000),
        )?;
        let field = ExpressionField::<2>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x, y],
            },
            &mut budget,
        )?;
        let report = field.flow_taylor(1, &mut budget)?.certify_noise_step(
            state,
            FlowNoiseRequest {
                end: interval(step + 1, step + 1, -2)?,
                candidate: BoxN::new([interval(-2, 2, 0)?, interval(-1, 1, 0)?])?,
                maximum_noise: 2,
            },
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowNoiseStatus::Certified),
            "{:?}",
            report.status()
        );
        state = report.end_state().expect("complete step").clone();
        println!("time={:?} endpoint={:?}", state.time(), state.enclosure());
    }
    // (x + y/4) - y/4 = x. Independent endpoint boxes lose this cancellation.
    assert_eq!(state.enclosure(), &initial);
    Ok(())
}

#[allow(dead_code)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
