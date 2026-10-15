//! Scalar construction preserves mathematical values and failure causes.
use hyperlimit::{Plane3, Point3};
use hyperreal::{Problem, Real};
use hypersdf::{Sdf, SdfCoordinate, SdfEvaluationError, SdfExpr};
use std::error::Error;

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn primitive_scalar_values_match_independent_integer_geometry() {
    // At (3, 0, 4): distance²=25, radial²=9, box outside distance²=13.
    let cases = [
        (SdfExpr::plane(Plane3::new(p(0, 0, 1), (-2).into())), 2),
        (SdfExpr::sphere(p(0, 0, 0), 9.into()), 16),
        (SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)), 3),
        (
            SdfExpr::rounded_aabb(p(-1, -1, -1), p(1, 1, 1), 1.into()),
            12,
        ),
        (
            SdfExpr::cylinder(SdfCoordinate::Z, p(0, 0, 0), 4.into(), 2.into()),
            5,
        ),
        (
            SdfExpr::capsule(SdfCoordinate::Z, p(0, 0, 0), 4.into(), 2.into()),
            9,
        ),
        (
            SdfExpr::torus(SdfCoordinate::Z, p(0, 0, 0), 4.into(), 1.into()),
            640,
        ),
        (
            SdfExpr::slab(Plane3::new(p(0, 0, 1), 0.into()), 2.into()),
            2,
        ),
    ];
    for (expr, expected) in cases {
        assert_eq!(
            Sdf::new(expr).evaluate_point(&p(3, 0, 4)).unwrap(),
            Real::from(expected)
        );
    }
    // Boundary and interior branches have different construction paths.
    let rounded = Sdf::new(SdfExpr::rounded_aabb(p(-1, -1, -1), p(1, 1, 1), 1.into()));
    assert_eq!(rounded.evaluate_point(&p(0, 0, 0)).unwrap(), Real::from(-2));
    assert_eq!(rounded.evaluate_point(&p(1, 0, 0)).unwrap(), Real::from(-1));
    let capsule = Sdf::new(SdfExpr::capsule(
        SdfCoordinate::Z,
        p(0, 0, 0),
        4.into(),
        2.into(),
    ));
    assert_eq!(capsule.evaluate_point(&p(3, 0, -4)).unwrap(), Real::from(9));
    assert_eq!(capsule.evaluate_point(&p(0, 0, 1)).unwrap(), Real::from(-4));
}

#[test]
fn expression_composition_preserves_exact_scalar_values() {
    // |(x-1)² + y - z| evaluated at (4, 2, 3) is 8.
    let x = SdfExpr::x().offset(1.into());
    let expr = x
        .clone()
        .mul_expr(x)
        .add_expr(SdfExpr::y())
        .sub_expr(SdfExpr::z())
        .abs();
    assert_eq!(
        Sdf::new(expr).evaluate_point(&p(4, 2, 3)).unwrap(),
        Real::from(8)
    );
    let expr = SdfExpr::x()
        .union(SdfExpr::y())
        .intersection(SdfExpr::z())
        .complement()
        .translate(p(1, 1, 1));
    assert_eq!(
        Sdf::new(expr).evaluate_point(&p(4, 2, 3)).unwrap(),
        Real::from(-2)
    );
    assert_eq!(
        Sdf::new(SdfExpr::constant(9.into()).sqrt())
            .evaluate_point(&p(0, 0, 0))
            .unwrap(),
        Real::from(3)
    );
    for (expr, expected) in [
        (SdfExpr::constant(0.into()).sin(), 0),
        (SdfExpr::constant(0.into()).cos(), 1),
        (SdfExpr::constant(0.into()).tan(), 0),
    ] {
        assert_eq!(
            Sdf::new(expr).evaluate_point(&p(0, 0, 0)).unwrap(),
            Real::from(expected)
        );
    }
}

#[test]
fn primitive_domain_rejection_survives_scalar_evaluation() {
    let cases = [
        SdfExpr::sphere(p(0, 0, 0), (-1).into()),
        SdfExpr::rounded_aabb(p(-1, -1, -1), p(1, 1, 1), (-1).into()),
        SdfExpr::cylinder(SdfCoordinate::Z, p(0, 0, 0), 4.into(), (-2).into()),
        SdfExpr::capsule(SdfCoordinate::Z, p(0, 0, 0), (-4).into(), 2.into()),
        SdfExpr::torus(SdfCoordinate::Z, p(0, 0, 0), 0.into(), 1.into()),
        SdfExpr::slab(Plane3::new(p(0, 0, 1), 0.into()), (-2).into()),
    ];
    for expr in cases {
        let SdfExpr::Primitive(primitive) = expr else {
            unreachable!()
        };
        assert!(matches!(
            primitive.try_scalar_value(&p(0, 0, 0)),
            Err(SdfEvaluationError::InvalidDomain { .. })
        ));
        // The documented compatibility method intentionally erases the cause.
        assert_eq!(primitive.scalar_value(&p(0, 0, 0)), None);
    }
}

#[test]
fn domain_and_branch_uncertainty_are_not_invalid_input() {
    let zero = SdfExpr::constant(unknown_zero());
    let cases = [
        zero.clone().abs(),
        zero.clone().union(SdfExpr::constant(0.into())),
        zero.intersection(SdfExpr::constant(0.into())),
        SdfExpr::sphere(p(0, 0, 0), unknown_zero()),
    ];
    for expr in cases {
        assert!(matches!(
            Sdf::new(expr).evaluate_point(&p(0, 0, 0)),
            Err(SdfEvaluationError::Undecided { .. })
        ));
    }
}

#[test]
fn nested_arithmetic_failure_keeps_its_native_source() {
    let expr = SdfExpr::constant((-1).into())
        .sqrt()
        .translate(p(1, 2, 3))
        .abs();
    let error = Sdf::new(expr).evaluate_point(&p(0, 0, 0)).unwrap_err();
    assert!(matches!(
        error,
        SdfEvaluationError::Scalar {
            source: Problem::SqrtNegative,
            ..
        }
    ));
    assert_eq!(
        error.source().unwrap().downcast_ref::<Problem>(),
        Some(&Problem::SqrtNegative)
    );
}
