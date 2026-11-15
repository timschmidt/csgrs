//! Native proof-bearing voxel intake; no sampled scalar becomes occupancy.
use hyperinterval::DyadicBudget;
use hyperreal::{Rational, Real};
use hypersdf::{Sdf, SdfExpr, SdfProgram, SdfVoxelEnclosureOptions};
use hypersolve::expression_program::ExpressionBudget;
use hypervoxel::{GridFrame, LengthUnit, MaterialRegionId};
use std::sync::Arc;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut graph = ExpressionBudget::new(100_000, 10_000_000);
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000_000);
    let source = Arc::new(Sdf::new(
        SdfExpr::x().sin().add_expr(SdfExpr::y().cos()).complement(),
    ));
    let program = SdfProgram::compile(source, &mut graph)?;
    let pitch = Real::from(Rational::fraction(1, 8)?);
    let frame = GridFrame::new(
        [0.into(), 0.into(), 0.into()],
        [pitch.clone(), pitch.clone(), pitch],
        1,
        LengthUnit::Millimeter,
    )?;
    let proof = program.classify_voxel_grid(
        frame,
        SdfVoxelEnclosureOptions::default(),
        &mut graph,
        &mut arithmetic,
    )?;
    let intake = proof.continuous_field_batch(MaterialRegionId(1), &mut graph)?;
    let grid = intake.materialize_exact_sparse_grid()?;
    assert_eq!(grid.len(), 8);
    assert!(Arc::ptr_eq(proof.source().source(), program.source()));
    println!("{} cells with retained whole-cell evidence", grid.len());
    Ok(())
}
