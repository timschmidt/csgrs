#![cfg(all(feature = "field", feature = "dispatch-trace", feature = "serde"))]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
};
use hyperreal::dispatch_trace;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

#[test]
fn validated_source_construction_preserves_dispatch_and_work() {
    let [time, x, v] = [SymbolId(100), SymbolId(0), SymbolId(1)];
    for (case, outputs) in [
        vec![Expr::int(1), Expr::int(0)],
        vec![Expr::anonymous_symbol(v), -Expr::anonymous_symbol(x)],
        vec![
            Expr::anonymous_symbol(x).powi(2) + Expr::anonymous_symbol(time),
            Expr::anonymous_symbol(v) / (Expr::anonymous_symbol(x) + Expr::int(2)),
        ],
    ]
    .into_iter()
    .enumerate()
    {
        let source = ExpressionProgram::compile(
            &outputs,
            &[v, time, x],
            &mut ExpressionBudget::new(100000, 10000000),
        )
        .unwrap();
        let definition = FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x, v],
        };
        let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
        dispatch_trace::reset();
        let field = dispatch_trace::with_recording(|| {
            ExpressionField::<2>::from_definition(definition, &mut budget).unwrap()
        });
        let trace = dispatch_trace::take_trace();
        let (one, rational, zero, temporaries, work) =
            [(1, 0, 1, 2, 72), (2, 1, 2, 6, 93), (8, 12, 14, 34, 249)][case];
        let expected: Vec<_> = [("one", one), ("rational", rational), ("zero", zero)]
            .into_iter()
            .filter(|(_, n)| *n != 0)
            .map(|(path, count)| dispatch_trace::DispatchCount {
                layer: "real",
                operation: "constructor",
                path,
                count,
            })
            .collect();
        assert_eq!(trace.dispatch, expected);
        assert_eq!(
            trace.rational,
            dispatch_trace::RationalTraceStats {
                temporary_rationals: temporaries,
                ..Default::default()
            }
        );
        assert_eq!(
            budget.policy().graph_work - budget.remaining_graph_work(),
            work
        );
        assert_eq!(budget.remaining_scalar_work(), budget.policy().scalar_work);
        assert_eq!(field.identity().source(), source.definition());
        println!("TRACE={trace:?}");
        println!(
            "WORK={}:{}",
            budget.policy().graph_work - budget.remaining_graph_work(),
            budget.policy().scalar_work - budget.remaining_scalar_work()
        );
        println!(
            "SOURCE={}",
            serde_json::to_string(&field.identity()).unwrap()
        );
    }
}
