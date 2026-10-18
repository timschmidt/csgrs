#![cfg(feature = "serde")]

use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};
use serde_json::Value;

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).unwrap())
}

// Every named certificate, constructed through the public API. Bounds follow
// from 3 < pi < 22/7, 8/3 < e < 11/4, 7/5 < sqrt(2) < 3/2,
// monotonicity, and the defining power/logarithm identities.
fn certificates() -> Vec<(&'static str, Real, i64, i64)> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().unwrap();
    let ln_two = Real::from(2).ln().unwrap();
    let ln_three = Real::from(3).ln().unwrap();
    vec![
        ("One", fraction(3, 2), 1, 2),
        ("Pi", pi.clone(), 3, 4),
        ("PiPow", pi_squared.clone(), 9, 10),
        ("PiInv", pi.clone().inverse().unwrap(), 0, 1),
        ("PiExp", &pi * &e, 8, 9),
        ("PiInvExp", (&e / &pi).unwrap(), 0, 1),
        ("PiSqrt", &pi * &sqrt_two, 4, 5),
        ("ConstProduct", &pi_squared * &e, 24, 28),
        ("ConstOffset", &pi - Real::from(3), 0, 1),
        ("ConstProductSqrt", &(&pi_squared * &e) * &sqrt_two, 33, 42),
        ("Sqrt", sqrt_two, 1, 2),
        ("Exp", Real::from(2).exp().unwrap(), 7, 8),
        ("Ln", ln_three.clone(), 1, 2),
        ("LnAffine", (Real::from(2) * &e).ln().unwrap(), 1, 2),
        ("LnProduct", &ln_two * &ln_three, 0, 2),
        ("Log10", Real::from(2).log10().unwrap(), 0, 1),
        ("Log2", Real::from(3).log2().unwrap(), 1, 2),
        ("Pow10", fraction(1, 3).exp10().unwrap(), 2, 3),
        ("Pow2", fraction(1, 3).exp2().unwrap(), 1, 2),
        ("SinPi", fraction(1, 5).sin_pi(), 0, 1),
        ("TanPi", fraction(1, 5).tan_pi().unwrap(), 0, 1),
    ]
}

fn assert_tag(wire: &Value, expected: &str) {
    let tag = &wire["class"];
    assert!(
        tag == expected
            || tag
                .as_object()
                .is_some_and(|map| map.contains_key(expected))
    );
}

fn query_and_compose(value: &Real, scale: i64, lower: i64, upper: i64) {
    let sign = match scale.cmp(&0) {
        std::cmp::Ordering::Less => RealSign::Negative,
        std::cmp::Ordering::Equal => RealSign::Zero,
        std::cmp::Ordering::Greater => RealSign::Positive,
    };
    let json = value.to_json();
    let bytes = value.to_bytes();
    for precision in [0, -128, 0] {
        assert_eq!(value.structural_facts().sign, Some(sign));
        let _ = value.detailed_facts();
        let _ = value.domain_facts();
        assert_eq!(value.certified_sign_until(precision).sign(), Some(sign));
        assert_eq!(
            value.zero_status(),
            if scale == 0 {
                ZeroKnowledge::Zero
            } else {
                ZeroKnowledge::NonZero
            }
        );
    }
    let (lower, upper) = if scale < 0 {
        (upper * scale, lower * scale)
    } else {
        (lower * scale, upper * scale)
    };
    let [lo, hi] = value.certified_rational_interval(-32).unwrap();
    assert!(lo >= Rational::from(lower) && hi <= Rational::from(upper));
    let translated = value + Real::one();
    let [lo, hi] = translated.certified_rational_interval(-32).unwrap();
    assert!(lo >= Rational::from(lower + 1) && hi <= Rational::from(upper + 1));
    if scale != 0 {
        let inverse = value.inverse_ref().unwrap();
        let product = value * inverse;
        // Request the existing exact reduction explicitly. Automatic equality
        // refinement alone does not currently decide every reciprocal product.
        assert_eq!(
            product.exact_rational_normal_form(),
            Some(Rational::one()),
            "{json}"
        );
    }
    if scale > 0 {
        let root = value.clone().sqrt().unwrap();
        let [lo, hi] = root.certified_rational_interval(-32).unwrap();
        assert!(lo >= Rational::zero() && hi <= Rational::from(7));
    }
    assert_eq!(value.to_json(), json);
    assert_eq!(value.to_bytes(), bytes);
}

fn lazy_certificate_matrix(decode: impl Fn(&Value) -> Real) {
    for (name, value, lower, upper) in certificates() {
        let original = serde_json::to_value(&value).unwrap();
        assert_tag(&original, name);
        for scale in [0, 1, -3] {
            for omit in [false, true] {
                let mut wire = original.clone();
                let original_scale: Rational =
                    serde_json::from_value(wire["rational"].clone()).unwrap();
                wire["rational"] =
                    serde_json::to_value(original_scale * Rational::from(scale)).unwrap();
                if omit {
                    wire.as_object_mut().unwrap().remove("computable");
                } else {
                    wire["computable"] = Value::Null;
                }
                let decoded = decode(&wire);
                query_and_compose(&decoded, scale, lower, upper);
                assert!(serde_json::to_value(&decoded).unwrap()["computable"].is_null());
            }
        }
    }
}

#[test]
fn json_lazy_certificates_support_queries_and_composition() {
    lazy_certificate_matrix(|wire| Real::from_json(&wire.to_string()).unwrap());
}

#[test]
fn cbor_lazy_certificates_support_queries_and_composition() {
    lazy_certificate_matrix(|wire| {
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(wire, &mut bytes).unwrap();
        Real::from_bytes(&bytes).unwrap()
    });
}

#[test]
fn materialized_certificates_keep_the_same_semantics() {
    for (name, value, lower, upper) in certificates() {
        assert_tag(&serde_json::to_value(&value).unwrap(), name);
        query_and_compose(&value, 1, lower, upper);
    }
}

#[test]
fn lazy_quadratic_certificate_supports_normal_forms_and_towers() {
    let value = Real::from(-3) * Real::from(2).sqrt().unwrap();
    let mut wire = serde_json::to_value(value).unwrap();
    assert_tag(&wire, "Sqrt");
    wire["computable"] = Value::Null;
    let value = Real::from_json(&wire.to_string()).unwrap();
    let expected = (Rational::zero(), Rational::from(-3), Rational::from(2));
    assert_eq!(value.exact_rational_normal_form(), None);
    assert_eq!(value.exact_quadratic_normal_form(), Some(expected.clone()));
    assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Negative));
    assert_eq!(value.radical_tower_sign(), Some(RealSign::Negative));
    let compact = value.compact_quadratic_tower().unwrap();
    assert_eq!(compact.exact_quadratic_normal_form(), Some(expected));
    let polynomial = value.quadratic_tower_annihilating_polynomial().unwrap();
    // This value is -3 sqrt(2), hence x^2 = 18. Independently reduce every
    // coefficient into its rational and sqrt(2) parts.
    let mut even = Rational::zero();
    let mut odd = Rational::zero();
    let mut power = Rational::one();
    for (index, coefficient) in polynomial.into_iter().enumerate() {
        if index % 2 == 0 {
            even = even + coefficient * &power;
        } else {
            odd = odd + coefficient * &power;
            power *= Rational::from(18);
        }
    }
    assert_eq!((even, odd), (Rational::zero(), Rational::zero()));
    assert!(serde_json::to_value(&value).unwrap()["computable"].is_null());
}
