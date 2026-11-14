//! Sensitivity-retained shared noise for repeated nonlinear field observations.
//! This certifies field images; it does not propagate noise through an ODE flow.
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    taylor::FieldTaylorRequest,
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorAffineMap, TaylorAffineModel, TaylorAffineRow,
        TaylorNoiseModel,
    },
};
use std::sync::Arc;
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
/// Build a field certificate, retain one shared noise column, then consume its
/// explicit Cartesian projection in another certified field-image operation.
pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let (time, x, y) = (SymbolId(1), SymbolId(2), SymbolId(3));
    let v = Expr::anonymous_symbol(x);
    let square = Expr::int(16) * v.clone() * v;
    let mut graph = ExpressionBudget::new(100_000, 10_000_000);
    let source = ExpressionProgram::compile(
        &[square.clone() + Expr::anonymous_symbol(y), square],
        &[y, time, x],
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
    // One explicit caller-owned clone; all later queries retain this certificate.
    let original = Arc::new(field_model.model().expect("nonempty domain").clone());
    let identity = TaylorAffineMap {
        input_count: 2,
        rows: vec![
            TaylorAffineRow {
                coefficients: vec![Real::one(), Real::zero()],
                offset: Real::zero(),
            },
            TaylorAffineRow {
                coefficients: vec![Real::zero(), Real::one()],
                offset: Real::zero(),
            },
        ],
    };
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000_000);
    let affine = Arc::new(TaylorAffineModel::new(
        original,
        identity,
        &mut graph,
        &mut arithmetic,
    )?);
    let noise = TaylorNoiseModel::new(affine, 1, &mut graph, &mut arithmetic)?;
    let observation = TaylorAffineMap {
        input_count: 2,
        rows: vec![
            TaylorAffineRow {
                coefficients: vec![Real::one(), -Real::one()],
                offset: Real::zero(),
            },
            TaylorAffineRow {
                coefficients: vec![Real::zero(), Real::one()],
                offset: Real::zero(),
            },
        ],
    };
    let range = noise.enclose(&observation, &mut graph, &mut arithmetic)?;
    // Independent identities: f0-f1=y, f1=16*x².
    assert_eq!(range.bounds(), &[d(-1, 1), d(0, 16)]);
    assert_eq!(
        noise.source().source().source().definition(),
        field_model.identity().source()
    );
    // Deliberately project to a box for the next enclosure operation. The noise
    // object remains available for future observations without adopting that box.
    let next = field.image(
        &d(0, 0),
        &BoxN::new(range.bounds().to_vec().try_into().unwrap())?,
        &mut budget,
    )?;
    for (exact, outer) in [d(0, 32), d(0, 16)].iter().zip(next.lanes().unwrap()) {
        assert!(exact.subset_of(outer, &mut arithmetic)?);
    }
    assert_eq!(
        noise
            .enclose(&observation, &mut graph, &mut arithmetic)?
            .bounds(),
        range.bounds()
    );
    println!(
        "retained columns: {}; observables: {:?}; next image: {next:?}",
        noise.columns().len(),
        range.bounds()
    );
    Ok(())
}
#[cfg(not(test))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
