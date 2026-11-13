#![cfg(all(feature = "field", feature = "dispatch-trace"))]
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hyperreal::dispatch_trace;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
#[test]
fn native_evolution_retains_fixed_dispatch_counts() {
    let [t, x, v] = [SymbolId(100), SymbolId(0), SymbolId(1)];
    for (case, outputs) in [
        vec![Expr::int(2), Expr::int(0)],
        vec![Expr::anonymous_symbol(v), -Expr::anonymous_symbol(x)],
        vec![Expr::anonymous_symbol(x).powi(2), Expr::int(0)],
    ]
    .into_iter()
    .enumerate()
    {
        let source = ExpressionProgram::compile(
            &outputs,
            &[v, t, x],
            &mut ExpressionBudget::new(100000, 10000000),
        )
        .unwrap();
        let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
        let field = ExpressionField::<2>::from_definition(
            FieldDefinition {
                source: source.definition().clone(),
                time: t,
                state: vec![x, v],
            },
            &mut budget,
        )
        .unwrap();
        let request = EvolutionRequest {
            start: DyadicInterval::point(0.into(), 0).unwrap(),
            end: DyadicInterval::point(1.into(), -2).unwrap(),
            initial: BoxN::new([
                DyadicInterval::point(1.into(), 0).unwrap(),
                DyadicInterval::point(0.into(), 0).unwrap(),
            ])
            .unwrap(),
        };
        dispatch_trace::reset();
        let report = dispatch_trace::with_recording(|| {
            field.integrate(request, EvolutionOptions::default(), &mut budget)
        });
        let trace = dispatch_trace::take_trace();
        assert!(report.end_enclosure().is_some());
        assert!(trace.dispatch.is_empty());
        assert_eq!(
            trace.rational,
            dispatch_trace::RationalTraceStats::default()
        );
        println!("TRACECASE {case} {trace:?}");
    }
}
