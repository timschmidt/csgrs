#![cfg(feature = "enclosure-program")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersdf::{Sdf, SdfCellLocation, SdfExpr, SdfProgram, SdfSubdivisionOptions};
use hypersolve::expression_program::ExpressionBudget;
use std::sync::Arc;
#[test]
fn adaptive_cover_certifies_the_complete_dependency_cell() {
    let mut graph = ExpressionBudget::new(1_000_000, 100_000_000);
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000_000);
    let source = Arc::new(Sdf::new(
        SdfExpr::x()
            .mul_expr(SdfExpr::constant(Real::one()).sub_expr(SdfExpr::x()))
            .offset((Real::from(3) / Real::from(8)).unwrap()),
    ));
    let p = SdfProgram::compile(source, &mut graph).unwrap();
    let cover = p
        .subdivide(
            [
                DyadicInterval::new(0.into(), 1.into(), 0).unwrap(),
                DyadicInterval::point(0.into(), 0).unwrap(),
                DyadicInterval::point(0.into(), 0).unwrap(),
            ],
            SdfSubdivisionOptions {
                precision: -48,
                max_depth: 3,
                max_cells: 31,
            },
            &mut graph,
            &mut arithmetic,
        )
        .unwrap();
    assert_eq!(cover.location(), Some(SdfCellLocation::ConservativeInside));
}
