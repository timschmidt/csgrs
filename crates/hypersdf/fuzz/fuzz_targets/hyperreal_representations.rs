//! SDF expression and transform dispatch over every Hyperreal representation pair.

#![no_main]

use hyperlimit::Point3;
use hyperreal::{Rational, Real, StructuralKind};
use hypersdf::{Sdf, SdfExpr, SdfSamplingPrecision};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    let origin = Point3::new(Real::zero(), Real::zero(), Real::zero());
    let min = Point3::new(-Real::one(), -Real::one(), -Real::one());
    let max = Point3::new(Real::one(), Real::one(), Real::one());

    for left in &values {
        for right in &values {
            let expressions = [
                SdfExpr::constant(left.clone()).add_expr(SdfExpr::constant(right.clone())),
                SdfExpr::constant(left.clone()).sub_expr(SdfExpr::constant(right.clone())),
                SdfExpr::constant(left.clone()).mul_expr(SdfExpr::constant(right.clone())),
                SdfExpr::constant(left.clone()).union(SdfExpr::constant(right.clone())),
                SdfExpr::constant(left.clone()).intersection(SdfExpr::constant(right.clone())),
                SdfExpr::x().translate(Point3::new(left.clone(), right.clone(), Real::zero())),
            ];
            for expression in expressions {
                let sdf = Sdf::new(expression);
                let scalar = sdf.classify_point(&origin);
                assert!(scalar.scalar_value.is_some());
                assert_eq!(sdf.classify_points([&origin])[0].location, scalar.location);
                let _ = sdf.interval_cell(&min, &max);
                let preview = sdf.sample_points_preview([&origin], SdfSamplingPrecision::F64);
                assert_eq!(preview.samples.len(), 1);
            }
        }
    }
});

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().expect("positive"),
        Real::new(Rational::new(3)).ln().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::ProductConstant,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}
