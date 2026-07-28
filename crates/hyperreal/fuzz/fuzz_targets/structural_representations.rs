//! Exhaustive cross-representation fuzzing for every public `Real` structural kind.

#![no_main]

use hyperreal::{
    CertifiedRealEquality, CertifiedRealSign, Rational, Real, RealSign, StructuralKind,
};
use libfuzzer_sys::fuzz_target;

const EXPECTED_KINDS: [StructuralKind; 8] = [
    StructuralKind::ExactRational,
    StructuralKind::PiLike,
    StructuralKind::ExpLike,
    StructuralKind::SqrtLike,
    StructuralKind::LogLike,
    StructuralKind::TrigExact,
    StructuralKind::ProductConstant,
    StructuralKind::ComputableOpaque,
];

fuzz_target!(|data: &[u8]| {
    let values = representative_values();
    assert_eq!(values.len(), EXPECTED_KINDS.len());

    for (value, expected) in values.iter().zip(EXPECTED_KINDS) {
        assert_eq!(value.detailed_facts().symbolic.kind, expected);
        assert!(value.certified_dyadic_interval(-512).is_some());

        let negated = -value;
        assert_bounded_equal(&-negated, value);
        assert_bounded_equal(&(value + Real::zero()), value);
        assert_bounded_equal(&(value * Real::one()), value);

        // Exercise every public representation through the major unary
        // dispatch families without requiring every domain to be valid.
        let _ = value.clone().sqrt();
        let _ = value.clone().exp();
        let _ = value.clone().ln();
        let _ = value.clone().sin();
        let _ = value.clone().cos();
        let _ = value.clone().tan();
        let _ = value.clone().atan();
        let _ = value.clone().sinh();
        let _ = value.clone().cosh();
        let _ = value.clone().tanh();
        let exponent = i64::from(data.first().copied().unwrap_or(0) % 9) - 4;
        let _ = value.clone().powi_i64(exponent);
    }

    // Every representation is paired with every other representation. This
    // catches asymmetric dispatch and ownership-specific arithmetic paths.
    for left in &values {
        for right in &values {
            assert_bounded_equal(&(left + right), &(right + left));
            assert_bounded_equal(&(left * right), &(right * left));
            assert_bounded_equal(&(left - right), &-(right - left));

            let quotient = (left / right).expect("representatives are nonzero");
            assert!(quotient.certified_dyadic_interval(-512).is_some());

            assert!(matches!(
                left.certified_eq_until(left, -512),
                CertifiedRealEquality::Equal { .. }
            ));
            let _ = left.certified_cmp_until(right, -512);
        }
    }
});

fn assert_bounded_equal(left: &Real, right: &Real) {
    if matches!(
        left.certified_eq_until(right, -512),
        CertifiedRealEquality::Equal { .. }
    ) || matches!(
        (left - right).certified_sign_until(-512),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    ) {
        return;
    }

    let [left_lower, left_upper] = left
        .certified_dyadic_interval(-512)
        .expect("representative has bounded approximation");
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(-512)
        .expect("representative has bounded approximation");
    assert!(
        left_lower <= right_upper && right_lower <= left_upper,
        "512-bit intervals must overlap for approximately equal expressions"
    );
}

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    vec![
        Real::new(Rational::fraction(3, 2).expect("nonzero denominator")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2))
            .sqrt()
            .expect("positive rational"),
        Real::new(Rational::new(3)).ln().expect("positive rational"),
        Real::new(Rational::fraction(1, 5).expect("nonzero denominator")).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ]
}
