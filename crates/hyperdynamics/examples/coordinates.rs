//! Exact state scaling admits a Picard step rejected in the original coordinates.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::CoordinateStatus,
    picard::{PicardFailure, PicardProposal, PicardStatus},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
fn d(a: i64, b: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), exponent).unwrap()
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (time, x, y) = (SymbolId(0), SymbolId(1), SymbolId(2));
    let source = ExpressionProgram::compile(
        &[Expr::int(128) * Expr::anonymous_symbol(y), Expr::int(0)],
        &[time, x, y],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
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
    let original = field.certify_step(
        PicardProposal {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial: BoxN::new([d(0, 0, 0), d(-1, 1, 0)])?,
            candidate: BoxN::new([d(-128, 128, 0), d(-1, 1, 0)])?,
        },
        &mut budget,
    );
    assert!(matches!(
        original.status(),
        PicardStatus::Rejected(PicardFailure::ContractionNotCertified)
    ));
    let chart = field.in_coordinates(
        CoordinateChange {
            variables: vec![x, y],
            to_source: vec![vec![128.into(), 0.into()], vec![0.into(), 1.into()]],
            from_source: vec![
                vec![(Real::one() / Real::from(128))?, 0.into()],
                vec![0.into(), 1.into()],
            ],
            origin: vec![0.into(), 0.into()],
        },
        &mut budget,
    )?;
    let step = chart.certify_step(
        PicardProposal {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial: BoxN::new([d(0, 0, 0), d(-1, 1, 0)])?,
            candidate: BoxN::new([d(-1, 1, 0), d(-1, 1, 0)])?,
        },
        &mut budget,
    );
    assert!(matches!(step.status(), CoordinateStatus::Certified));
    assert_eq!(
        step.source_end_enclosure().unwrap().lanes(),
        Some(&[d(-64, 64, 0), d(-1, 1, 0)])
    );
    println!("Original attempt: {:?}", original.status());
    println!(
        "Coordinate attempt: {:?}; original-coordinate endpoint: {:?}",
        step.status(),
        step.source_end_enclosure()
    );
    Ok(())
}
