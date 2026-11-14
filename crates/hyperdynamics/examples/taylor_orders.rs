//! Share exact Lie coefficient graphs, then certify two consecutive polynomial steps.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition, picard::PicardProposal,
    taylor_flow::FlowTaylorStatus,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
/// Execute the example with exact endpoint and remainder assertions.
pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let t = SymbolId(0);
    let x = SymbolId(1);
    let source = ExpressionProgram::compile(
        &[Expr::int(2) * Expr::anonymous_symbol(t)],
        &[x, t],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<1>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: t,
            state: vec![x],
        },
        &mut budget,
    )?;
    let prepared = field.flow_taylor(4, &mut budget)?;
    let program = prepared.restrict_order(2, &mut budget)?;
    assert!(std::ptr::eq(
        prepared.stages()[1].program().definition(),
        program.stages()[1].program().definition()
    ));
    drop(prepared);
    let mut initial = BoxN::new([d(3, 4, 0)])?;
    for i in 0..2 {
        let report = program.certify_step(
            PicardProposal {
                start: d(i, i, -3),
                end: d(i + 1, i + 1, -3),
                initial,
                candidate: BoxN::new([d(3, 5, 0)])?,
            },
            &mut budget,
        );
        assert!(matches!(report.status(), FlowTaylorStatus::Certified));
        assert_eq!(
            report.evidence().remainder().unwrap().lanes(),
            Some(&[d(0, 0, 0)])
        );
        initial = report.end_enclosure().unwrap().clone();
    }
    assert_eq!(initial.lanes(), Some(&[d(49, 65, -4)]));
    Ok(())
}
#[cfg(not(test))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
