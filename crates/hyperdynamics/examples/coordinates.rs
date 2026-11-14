//! Exact state scaling admits a Picard step rejected in the original coordinates.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{CoordinateEvolutionStatus, CoordinateStatus},
    evolution::{EvolutionOptions, EvolutionRequest},
    picard::{PicardFailure, PicardProposal, PicardStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
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
    // Continue in the exact chart. Each endpoint stays in chart coordinates;
    // original-coordinate Cartesian projections never become the next input.
    let evolution = chart.integrate(
        EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, 0),
            initial: BoxN::new([d(0, 0, 0), d(-1, 1, 0)])?,
        },
        EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..EvolutionOptions::default()
        },
        &mut budget,
    );
    assert!(matches!(
        evolution.status(),
        CoordinateEvolutionStatus::Completed
    ));
    assert_eq!(evolution.segments().len(), 4);
    // The native candidate search widens the velocity box to [-3,3] for its
    // self-map proof. The final envelope therefore exceeds the exact reachable
    // set; retaining an exact chart does not remove native interval overestimation.
    assert_eq!(
        evolution.source_end_enclosure().unwrap().lanes(),
        Some(&[d(-384, 384, 0), d(-1, 1, 0)])
    );
    // Independent affine solution: x(t)=128*t*y(0), y(t)=y(0).
    for (index, segment) in evolution.segments().enumerate() {
        let bound = 32 * i64::try_from(index + 1)?;
        let analytic = [d(-bound, bound, 0), d(-1, 1, 0)];
        for enclosure in [segment.source_tube(), segment.source_end_enclosure()] {
            for (exact, outer) in analytic.iter().zip(enclosure.lanes().unwrap()) {
                assert!(exact.subset_of(outer, &mut DyadicBudget::new(16384, 1_000_000_000))?);
            }
        }
    }
    println!(
        "Whole horizon: {:?}; endpoint: {:?}",
        evolution.status(),
        evolution.source_end_enclosure()
    );
    Ok(())
}
