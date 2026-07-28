//! Cross every Hyperreal representation through vector, matrix, and scalar algebra.

#![no_main]

use hyperlattice::{Matrix3, Vector2, Vector3, dot2, squared_distance2, wedge2};
use hyperreal::{CertifiedRealEquality, Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for left in &values {
        for right in &values {
            let a2 = Vector2::new([left.clone(), right.clone()]);
            let b2 = Vector2::new([right.clone(), left.clone()]);
            assert_bounded_equal(&a2.dot(&b2), &b2.dot(&a2));
            assert_eq!(dot2([&a2[0], &a2[1]], [&b2[0], &b2[1]]), a2.dot(&b2));
            assert_eq!(wedge2([&a2[0], &a2[1]], [&b2[0], &b2[1]]), a2.wedge(&b2));
            assert_eq!(
                squared_distance2([&a2[0], &a2[1]], [&a2[0], &a2[1]]),
                Real::zero()
            );

            let a3 = Vector3::new([left.clone(), right.clone(), Real::one()]);
            let b3 = Vector3::new([right.clone(), Real::one(), left.clone()]);
            assert_bounded_equal(&a3.dot(&b3), &b3.dot(&a3));
            let forward_cross = a3.cross(&b3);
            let reverse_cross = -b3.cross(&a3);
            for axis in 0..3 {
                assert_bounded_equal(&forward_cross[axis], &reverse_cross[axis]);
            }
            assert_eq!(Matrix3::identity() * a3.clone(), a3);
        }
    }
});

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
