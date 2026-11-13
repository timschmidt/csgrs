//! Centered logistic field image, retaining exact coefficients and remainder.
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    taylor::FieldTaylorRequest,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (t, x) = (SymbolId(0), SymbolId(1));
    let v = Expr::anonymous_symbol(x);
    let source = ExpressionProgram::compile(
        &[v.clone() * (Expr::int(1) - v)],
        &[t, x],
        &mut ExpressionBudget::new(100_000, 10_000_000),
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
    let unit = DyadicInterval::new(0.into(), 1.into(), 0)?;
    let request = FieldTaylorRequest {
        time: unit.clone(),
        state: BoxN::new([unit])?,
        order: 2,
    };
    let natural = field.image(&request.time, &request.state, &mut budget)?;
    let model = field.taylor_model(request, &mut budget)?;
    assert_eq!(
        model.image().lanes(),
        Some(&[DyadicInterval::new(0.into(), 1.into(), -2)?])
    );
    println!("natural={natural:?}\nTaylor={:?}", model.image());
    let proof = model.model().expect("nonempty domain");
    println!(
        "coefficients={:?}\nremainder={:?}",
        proof.terms(),
        proof.remainder()
    );
    let midpoint = BoxN::new([DyadicInterval::point(1.into(), -1)?])?;
    let reused = model.enclose_on(&DyadicInterval::point(0.into(), 0)?, &midpoint, &mut budget)?;
    assert_eq!(
        reused.lanes(),
        Some(&[DyadicInterval::point(1.into(), -2)?])
    );
    println!("midpoint image={reused:?}");
    Ok(())
}
