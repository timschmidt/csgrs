//! An owned time-dependent field; this example does not certify a trajectory.
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let [time, x, v] = [SymbolId(0), SymbolId(1), SymbolId(2)];
    let source = ExpressionProgram::compile(
        &[
            Expr::anonymous_symbol(v),
            -Expr::anonymous_symbol(x) + Expr::anonymous_symbol(time),
        ],
        &[v, time, x],
        &mut ExpressionBudget::new(100, 1000),
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x, v],
        },
        &mut budget,
    )?;
    let state = BoxN::new([d(1, 2), d(-1, 1)])?;
    let image = field.image(&d(0, 1), &state, &mut budget)?;
    let jacobian = field.jacobian(&d(0, 1), &state, &mut budget)?;
    assert_eq!(image.lanes(), Some(&[d(-1, 1), d(-2, 0)]));
    assert_eq!(
        jacobian.rows(),
        Some(&[[d(0, 0), d(1, 1)], [d(-1, -1), d(0, 0)]])
    );
    assert_eq!(
        jacobian
            .mul_vector(
                &state,
                &mut hyperinterval::DyadicBudget::new(16384, 1_000_000_000)
            )?
            .lanes(),
        Some(&[d(-1, 1), d(-2, -1)])
    );
    println!("field: ([-1,1], [-2,0]); state Jacobian: [[0,1], [-1,0]]");
    Ok(())
}
