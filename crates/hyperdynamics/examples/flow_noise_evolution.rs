//! Automatic forward and reverse shear horizons preserve shared uncertainty.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::EvolutionOptions,
    taylor_flow::noise::{
        FlowNoiseState,
        evolution::{FlowNoiseEvolutionOptions, FlowNoiseEvolutionStatus},
    },
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::sync::Arc;

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let (time, x, y) = (SymbolId(10), SymbolId(20), SymbolId(30));
    let d = |lo: i64, hi: i64, e| DyadicInterval::new(lo.into(), hi.into(), e);
    let initial = BoxN::new([d(-1, 1, 0)?, d(-1, 1, 0)?])?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let mut state = Arc::new(FlowNoiseState::new(
        d(0, 0, 0)?,
        initial.clone(),
        &mut budget,
    )?);
    for (sign, end) in [(1, 3), (-1, 6)] {
        let source = ExpressionProgram::compile(
            &[Expr::int(sign) * Expr::anonymous_symbol(y), Expr::int(0)],
            &[y, time, x],
            &mut ExpressionBudget::new(100000, 1000000),
        )?;
        let field = ExpressionField::<2>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time,
                state: vec![x, y],
            },
            &mut budget,
        )?;
        let report = field.flow_taylor(1, &mut budget)?.integrate_noise(
            state,
            d(end, end, -3)?,
            FlowNoiseEvolutionOptions {
                maximum_noise: 2,
                round_coefficients: false,
                column_cutoff_bits: None,
                native: EvolutionOptions {
                    maximum_step: Some(d(1, 1, -2)?),
                    ..Default::default()
                },
            },
            &mut budget,
        );
        assert!(
            matches!(report.status(), FlowNoiseEvolutionStatus::Completed),
            "{:?}",
            report.status()
        );
        assert_eq!(report.segments().len(), 2);
        state = report.end_state().expect("complete horizon").clone();
        println!("time={:?} endpoint={:?}", state.time(), state.enclosure());
    }
    // Every pair (x,y) returns to itself; no independent copies of y are introduced.
    assert_eq!(state.enclosure(), &initial);
    Ok(())
}
#[allow(dead_code)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
