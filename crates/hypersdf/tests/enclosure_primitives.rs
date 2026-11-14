#![cfg(feature = "enclosure-program")]
use hyperlimit::Point3;
use hyperreal::Real;
use hypersdf::{Sdf, SdfExpr, SdfProgram};
use hypersolve::expression_program::ExpressionBudget;
use std::sync::Arc;
#[test]
fn sphere_is_a_native_program_operand() {
    let source = Arc::new(Sdf::new(SdfExpr::sphere(
        Point3::new(0.into(), 0.into(), 0.into()),
        Real::one(),
    )));
    let program = SdfProgram::compile(source, &mut ExpressionBudget::new(10000, 1000000)).unwrap();
    assert_eq!(
        program
            .program()
            .eval_real(
                &[2.into(), 0.into(), 0.into()],
                &mut ExpressionBudget::new(10000, 1000000)
            )
            .unwrap(),
        vec![Real::from(3)]
    );
}
