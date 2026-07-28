//! Cross every Hyperreal representation through exact predicate entry points.

#![no_main]

use hyperlimit::{
    Point2, classify_point_line, classify_real_sign, compare_reals, orient2, orient2_batch,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for left in &values {
        for right in &values {
            let a = Point2::new(left.clone(), right.clone());
            let b = Point2::new(left + Real::one(), right.clone());
            let c = Point2::new(left.clone(), right + Real::one());

            let orientation = orient2(&a, &b, &c);
            let reversed = orient2(&a, &c, &b);
            assert_eq!(
                orientation.value().map(|sign| sign.reversed()),
                reversed.value()
            );
            assert_eq!(
                orient2_batch(&[(a.clone(), b.clone(), c.clone())])[0].value(),
                orientation.value()
            );
            assert_eq!(
                classify_point_line(&a, &b, &a).value(),
                Some(hyperlimit::LineSide::On)
            );

            let _ = classify_real_sign(left);
            let _ = compare_reals(left, right);
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
