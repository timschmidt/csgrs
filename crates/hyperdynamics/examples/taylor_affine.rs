//! Exact linear observables of a nonlinear field, retaining shared Taylor terms.
//! This is a field-image certificate, not an ODE coordinate-change proof.
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    taylor::FieldTaylorRequest,
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorAffineMap, TaylorAffineModel, TaylorAffineRow,
    },
};
use std::sync::Arc;
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (time, x, y) = (SymbolId(1), SymbolId(2), SymbolId(3));
    let (u, v) = (Expr::anonymous_symbol(x), Expr::anonymous_symbol(y));
    let square = u.clone() * u;
    let mut graph = ExpressionBudget::new(100_000, 10_000_000);
    let source = ExpressionProgram::compile(
        &[square.clone() + v.clone(), square - v],
        &[time, x, y],
        &mut graph,
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![x, y],
        },
        &mut budget,
    )?;
    let field_model = field.taylor_model(
        FieldTaylorRequest {
            time: d(0, 0),
            state: BoxN::new([d(-1, 1), d(-1, 1)])?,
            order: 2,
        },
        &mut budget,
    )?;
    // The explicit one-time model clone is caller-owned work. Further output
    // transformations share this model and do not rebuild its derivative graph.
    let model = Arc::new(field_model.model().expect("nonempty domain").clone());
    let half = Real::from(Rational::from(1) / Rational::from(2));
    let map = TaylorAffineMap {
        input_count: 2,
        rows: vec![
            TaylorAffineRow {
                coefficients: vec![half.clone(), half.clone()],
                offset: Real::zero(),
            },
            TaylorAffineRow {
                coefficients: vec![half.clone(), -half],
                offset: Real::zero(),
            },
        ],
    };
    let mut scalar = DyadicBudget::new(16384, 1_000_000_000_000);
    let observables = TaylorAffineModel::new(model, map, &mut graph, &mut scalar)?;
    // (f0+f1)/2=x² and (f0-f1)/2=y, independently known exact identities.
    assert_eq!(observables.range().bounds(), &[d(0, 1), d(-1, 1)]);
    assert_eq!(observables.remainder(), &[d(0, 0), d(0, 0)]);
    // The explicit projected result can enter a later field operation. Keep
    // `observables` itself for further affine transforms without boxing first.
    let next = field.image(
        &d(0, 0),
        &BoxN::new(observables.range().bounds().to_vec().try_into().unwrap())?,
        &mut budget,
    )?;
    println!("original image: {:?}", field_model.image());
    println!("shared-coefficient observables: {:?}", observables.range());
    println!("further field image after explicit projection: {next:?}");
    Ok(())
}
