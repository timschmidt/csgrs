//! Numerical search followed by independent exact verification and composition.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionRequest, EvolutionStatus},
    picard::PicardStatus,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let [time, x] = [SymbolId(0), SymbolId(1)];
    let source = ExpressionProgram::compile(
        &[Expr::anonymous_symbol(x)],
        &[x, time],
        &mut ExpressionBudget::new(100, 1000),
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<1>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x],
        },
        &mut budget,
    )?;
    let quarter = DyadicInterval::point(1.into(), -2)?;
    let candidate = field.rk4_proposer(&mut budget)?.propose(
        EvolutionRequest {
            start: DyadicInterval::point(0.into(), 0)?,
            end: quarter.clone(),
            initial: BoxN::new([DyadicInterval::point(1.into(), 0)?])?,
        },
        Default::default(),
        &mut budget,
    )?;
    println!(
        "Uncertified RK endpoint estimate: {:?}",
        candidate.trace().end_estimate()
    );
    let attempt = candidate.certify(&mut budget);
    assert!(matches!(attempt.proof().status(), PicardStatus::Certified));
    let next = field.integrate(
        EvolutionRequest {
            start: quarter,
            end: DyadicInterval::point(1.into(), -1)?,
            initial: attempt.proof().end_enclosure().unwrap().clone(),
        },
        Default::default(),
        &mut budget,
    );
    assert!(matches!(next.status(), EvolutionStatus::Completed));
    println!(
        "Certified endpoint at exact time 1/2: {:?}",
        next.end_enclosure()
    );
    Ok(())
}
