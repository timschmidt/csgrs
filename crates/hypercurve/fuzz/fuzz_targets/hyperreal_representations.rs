//! Native curve transforms over every pair of Hyperreal representations.

#![no_main]

use hypercurve::{LineSeg2, Point2, Similarity2};
use hyperreal::{CertifiedRealEquality, Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for tx in &values {
        for ty in &values {
            let transform = Similarity2::try_from_real_affine(
                Real::zero(),
                -Real::one(),
                Real::one(),
                Real::zero(),
                tx.clone(),
                ty.clone(),
            )
            .expect("unit quarter-turn similarity");
            let source = LineSeg2::try_new(
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
            )
            .expect("nondegenerate line");
            let transformed = source
                .transform_similarity(&transform)
                .expect("similarity preserves lines");
            assert_point_bounded_equal(transformed.start(), &Point2::new(tx.clone(), ty.clone()));
            assert_point_bounded_equal(
                transformed.end(),
                &Point2::new(tx.clone(), ty + Real::one()),
            );
            assert_bounded_equal(&transformed.length_squared(), &Real::one());

            let midpoint =
                transformed.point_at(Real::new(Rational::fraction(1, 2).expect("valid rational")));
            assert_bounded_equal(midpoint.x(), tx);
        }
    }
});

fn assert_point_bounded_equal(left: &Point2, right: &Point2) {
    assert_bounded_equal(left.x(), right.x());
    assert_bounded_equal(left.y(), right.y());
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
