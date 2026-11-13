use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::EvolutionRequest,
    reachability::{ReachabilityPolicy, ReachabilityRequest},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let time = SymbolId(0);
    let state = SymbolId(1);
    let source = ExpressionProgram::compile(
        &[Expr::int(2)],
        &[time, state],
        &mut ExpressionBudget::new(1000, 10000),
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<1>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![state],
        },
        &mut budget,
    )?;
    let request = ReachabilityRequest {
        evolution: EvolutionRequest {
            start: DyadicInterval::point(0.into(), 0)?,
            end: DyadicInterval::point(1.into(), -1)?,
            initial: BoxN::new([DyadicInterval::new((-1).into(), 1.into(), 0)?])?,
        },
        epsilon: DyadicInterval::point(1.into(), -1)?,
    };
    let report = field.reach(request, ReachabilityPolicy::default(), &mut budget);
    println!("{:?}", report.status());
    if let Some(cover) = report.end_cover() {
        println!(
            "{} certified endpoint boxes at {:?}",
            cover.boxes().len(),
            cover.end_time()
        );
        for state in cover.boxes() {
            println!("{state:?}");
        }
    }
    Ok(())
}
