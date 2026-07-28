//! BREP curve and surface replay over every pair of Hyperreal representations.

#![no_main]

use hyperbrep::{BrepRationalBezier3, BrepSurface, BrepSurfaceId};
use hyperlimit::{Plane3, Point2, Point3};
use hyperreal::{CertifiedRealEquality, Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    let half = Real::new(Rational::fraction(1, 2).expect("valid rational"));

    for left in &values {
        for right in &values {
            let controls = vec![
                Point3::new(left.clone(), right.clone(), Real::zero()),
                Point3::new(left + Real::one(), right + Real::one(), Real::one()),
            ];
            let curve =
                BrepRationalBezier3::try_new(controls.clone(), vec![left.clone(), right.clone()])
                    .expect("positive projective weights");
            assert_eq!(curve.degree(), 1);
            assert_point_bounded_equal(
                &curve.point_at(&Real::zero()).expect("domain start"),
                &controls[0],
            );
            assert_point_bounded_equal(
                &curve.point_at(&Real::one()).expect("domain end"),
                &controls[1],
            );
            assert!(curve.point_at(&half).is_ok());

            let surface = BrepSurface::plane(
                BrepSurfaceId::new(1),
                Plane3::new(
                    Point3::new(Real::zero(), Real::zero(), Real::one()),
                    -left.clone(),
                ),
            );
            let uv = Point2::new(left.clone(), right.clone());
            let evaluated = surface.evaluate_frame_uv(uv);
            assert!(evaluated.exact_evaluation_ready);
            let point = evaluated.point.expect("supported z-normal frame");
            let projected = surface.project_frame_point(point);
            assert!(projected.exact_projection_ready);
        }
    }
});

fn assert_point_bounded_equal(left: &Point3, right: &Point3) {
    assert_bounded_equal(&left.x, &right.x);
    assert_bounded_equal(&left.y, &right.y);
    assert_bounded_equal(&left.z, &right.z);
}

fn assert_bounded_equal(left: &Real, right: &Real) {
    if matches!(
        left.certified_eq_until(right, -512),
        CertifiedRealEquality::Equal { .. }
    ) {
        return;
    }
    let [left_lower, left_upper] = left
        .certified_dyadic_interval(-512)
        .expect("bounded left value");
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(-512)
        .expect("bounded right value");
    assert!(left_lower <= right_upper && right_lower <= left_upper);
}

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
