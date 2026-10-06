//! Exact part assertions and envelopes over every Hyperreal representation pair.

#![no_main]

use hyperparts::{AssertionValue, VoltageEnvelope, VoltageRange};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for left in &values {
        for right in &values {
            let left_end = left + right;
            let right_end = right + left;
            let assertion = AssertionValue::interval(left.clone(), left_end.clone())
                .expect("positive interval width");
            assert!(matches!(assertion, AssertionValue::ExactInterval { .. }));
            assert!(matches!(
                AssertionValue::exact_scalar(right.clone()),
                AssertionValue::ExactScalar(_)
            ));

            let voltage =
                VoltageRange::new(left.clone(), left_end.clone()).expect("positive interval width");
            let other =
                VoltageRange::new(right.clone(), right_end).expect("positive interval width");
            assert_eq!(voltage.overlaps(&other), Some(true));

            let envelope =
                VoltageEnvelope::new(left.clone(), left_end).expect("positive interval width");
            let other_envelope =
                VoltageEnvelope::new(right.clone(), left + right).expect("positive width");
            assert_eq!(envelope.overlaps(&other_envelope), Some(true));
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
