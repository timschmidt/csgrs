//! Exhaustive representation-contract tests for `Real` and `Computable`.
//!
//! Public structural facts cover a deliberately smaller taxonomy than the
//! private optimized certificates and approximation nodes. This matrix keeps
//! all three inventories synchronized and exercises every finite variant
//! through scalar arithmetic, certification, caching, and serialization.

use core::cmp::Ordering;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering as AtomicOrdering},
};

#[cfg(feature = "serde")]
use hyperreal::ZeroKnowledge;
use hyperreal::{
    CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, PrimitiveFloatStatus,
    Rational, RationalStorageClass, Real, RealSign, StructuralKind,
};

struct RepresentationCase {
    certificate: &'static str,
    public_kind: StructuralKind,
    value: Real,
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn optimized_certificate_representatives() -> Vec<RepresentationCase> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        RepresentationCase {
            certificate: "One",
            public_kind: StructuralKind::ExactRational,
            value: fraction(3, 2),
        },
        RepresentationCase {
            certificate: "Pi",
            public_kind: StructuralKind::PiLike,
            value: pi.clone(),
        },
        RepresentationCase {
            certificate: "PiPow",
            public_kind: StructuralKind::PiLike,
            value: pi_squared.clone(),
        },
        RepresentationCase {
            certificate: "PiInv",
            public_kind: StructuralKind::PiLike,
            value: pi.clone().inverse().expect("pi is nonzero"),
        },
        RepresentationCase {
            certificate: "PiExp",
            public_kind: StructuralKind::ExpLike,
            value: &pi * &e,
        },
        RepresentationCase {
            certificate: "PiInvExp",
            public_kind: StructuralKind::ExpLike,
            value: (&e / &pi).expect("pi is nonzero"),
        },
        RepresentationCase {
            certificate: "PiSqrt",
            public_kind: StructuralKind::SqrtLike,
            value: &pi * &sqrt_two,
        },
        RepresentationCase {
            certificate: "ConstProduct",
            public_kind: StructuralKind::ProductConstant,
            value: &pi_squared * &e,
        },
        RepresentationCase {
            certificate: "ConstOffset",
            public_kind: StructuralKind::ProductConstant,
            value: &pi - Real::from(3),
        },
        RepresentationCase {
            certificate: "ConstProductSqrt",
            public_kind: StructuralKind::ProductConstant,
            value: &(&pi_squared * &e) * &sqrt_two,
        },
        RepresentationCase {
            certificate: "Sqrt",
            public_kind: StructuralKind::SqrtLike,
            value: sqrt_two,
        },
        RepresentationCase {
            certificate: "Exp",
            public_kind: StructuralKind::ExpLike,
            value: Real::from(2).exp().expect("finite exponential"),
        },
        RepresentationCase {
            certificate: "Ln",
            public_kind: StructuralKind::LogLike,
            value: ln_three.clone(),
        },
        RepresentationCase {
            certificate: "LnAffine",
            public_kind: StructuralKind::LogLike,
            value: (Real::from(2) * &e).ln().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "LnProduct",
            public_kind: StructuralKind::LogLike,
            value: &ln_two * &ln_three,
        },
        RepresentationCase {
            certificate: "Log10",
            public_kind: StructuralKind::LogLike,
            value: Real::from(2).log10().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "Log2",
            public_kind: StructuralKind::LogLike,
            value: Real::from(3).log2().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "Pow10",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        },
        RepresentationCase {
            certificate: "Pow2",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        },
        RepresentationCase {
            certificate: "SinPi",
            public_kind: StructuralKind::TrigExact,
            value: fraction(1, 5).sin_pi(),
        },
        RepresentationCase {
            certificate: "TanPi",
            public_kind: StructuralKind::TrigExact,
            value: fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        },
        RepresentationCase {
            certificate: "Irrational",
            public_kind: StructuralKind::ComputableOpaque,
            value: Real::one().sin(),
        },
    ]
}

fn structural_kind_index(kind: StructuralKind) -> usize {
    match kind {
        StructuralKind::ExactRational => 0,
        StructuralKind::PiLike => 1,
        StructuralKind::ExpLike => 2,
        StructuralKind::SqrtLike => 3,
        StructuralKind::LogLike => 4,
        StructuralKind::TrigExact => 5,
        StructuralKind::ProductConstant => 6,
        StructuralKind::ComputableOpaque => 7,
    }
}

fn rational_storage_index(storage: RationalStorageClass) -> usize {
    match storage {
        RationalStorageClass::Zero => 0,
        RationalStorageClass::WordSized => 1,
        RationalStorageClass::MultiLimb => 2,
        RationalStorageClass::VeryLarge => 3,
    }
}

fn primitive_status_index(status: PrimitiveFloatStatus) -> usize {
    match status {
        PrimitiveFloatStatus::Zero => 0,
        PrimitiveFloatStatus::NormalFinite => 1,
        PrimitiveFloatStatus::SubnormalOrUnderflows => 2,
        PrimitiveFloatStatus::Overflows => 3,
        PrimitiveFloatStatus::Unknown => 4,
    }
}

fn assert_same_value(left: &Real, right: &Real, context: &str) {
    if matches!(
        left.certified_eq_until(right, -160),
        CertifiedRealEquality::Equal { .. }
    ) || matches!(
        (left - right).certified_sign_until(-160),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    ) {
        return;
    }

    let [left_lower, left_upper] = left
        .certified_dyadic_interval(-160)
        .unwrap_or_else(|| panic!("{context}: left value must have a certified interval"));
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(-160)
        .unwrap_or_else(|| panic!("{context}: right value must have a certified interval"));
    assert!(
        left_lower <= right_upper && right_lower <= left_upper,
        "{context}: certified intervals for equal expressions do not overlap"
    );
}

#[test]
fn every_public_kind_and_private_optimized_certificate_crosses_scalar_dispatch() {
    let cases = optimized_certificate_representatives();
    assert_eq!(cases.len(), 22, "update the optimized certificate matrix");

    let mut observed_kinds = [false; 8];
    for case in &cases {
        let facts = case.value.detailed_facts();
        assert_eq!(
            facts.symbolic.kind, case.public_kind,
            "{}",
            case.certificate
        );
        observed_kinds[structural_kind_index(facts.symbolic.kind)] = true;
        assert_eq!(case.value.immediate_sign(), Some(RealSign::Positive));
        assert!(case.value.certified_dyadic_interval(-160).is_some());
        assert_eq!(
            case.value.partial_cmp(&Real::zero()),
            Some(Ordering::Greater)
        );
        assert_eq!(case.value.partial_cmp(&0.0), Some(Ordering::Greater));
        assert_eq!(0.0_f64.partial_cmp(&case.value), Some(Ordering::Less));
        assert_ne!(case.value, 0.0);
        assert_ne!(0.0, case.value);
        assert_ne!(case.value, f64::NAN);
        assert_eq!(case.value.partial_cmp(&f64::NAN), None);

        assert_same_value(&(-(-case.value.clone())), &case.value, case.certificate);
        assert_same_value(&(&case.value + Real::zero()), &case.value, case.certificate);
        assert_same_value(&(&case.value * Real::one()), &case.value, case.certificate);

        let owned_inverse = case
            .value
            .clone()
            .inverse()
            .expect("representative is nonzero");
        let borrowed_inverse = case.value.inverse_ref().expect("representative is nonzero");
        assert_same_value(&owned_inverse, &borrowed_inverse, case.certificate);
        assert_same_value(
            &(&case.value * &borrowed_inverse),
            &Real::one(),
            case.certificate,
        );

        let scaled = &case.value * &fraction(3, 5);
        assert_eq!(
            scaled.detailed_facts().symbolic.kind,
            case.public_kind,
            "{} fractional scale",
            case.certificate,
        );
        assert_eq!((-scaled).immediate_sign(), Some(RealSign::Negative));
    }
    assert_eq!(observed_kinds, [true; 8], "missing public Real kind");

    for left in &cases {
        for right in &cases {
            let context = format!("{} with {}", left.certificate, right.certificate);

            let borrowed_add = &left.value + &right.value;
            let owned_add = left.value.clone() + right.value.clone();
            assert_same_value(&borrowed_add, &owned_add, &context);
            assert_same_value(&borrowed_add, &(&right.value + &left.value), &context);

            let borrowed_sub = &left.value - &right.value;
            let owned_sub = left.value.clone() - right.value.clone();
            assert_same_value(&borrowed_sub, &owned_sub, &context);
            assert_same_value(&borrowed_sub, &-(&right.value - &left.value), &context);

            let borrowed_mul = &left.value * &right.value;
            let owned_mul = left.value.clone() * right.value.clone();
            assert_same_value(&borrowed_mul, &owned_mul, &context);
            assert_same_value(&borrowed_mul, &(&right.value * &left.value), &context);

            let borrowed_div = (&left.value / &right.value).expect("representative is nonzero");
            let owned_div =
                (left.value.clone() / right.value.clone()).expect("representative is nonzero");
            assert_same_value(&borrowed_div, &owned_div, &context);
            assert_same_value(&(&borrowed_div * &right.value), &left.value, &context);

            assert!(matches!(
                left.value.certified_eq_until(&left.value, -160),
                CertifiedRealEquality::Equal { .. }
            ));
            assert!(matches!(
                left.value.certified_cmp_until(&left.value, -160),
                CertifiedRealOrdering::Known {
                    ordering: Ordering::Equal,
                    ..
                }
            ));
        }
    }
}

#[test]
fn ownership_float_assignment_and_iterator_forms_match_borrowed_arithmetic() {
    let value = Real::pi();
    let half = fraction(1, 2);

    assert_same_value(&(value.clone() + 0.5), &(&value + &half), "owned + f64");
    assert_same_value(&(&value + 0.5), &(&value + &half), "borrowed + f64");
    assert_same_value(&(0.5 + value.clone()), &(&half + &value), "f64 + owned");
    assert_same_value(&(value.clone() - 0.5), &(&value - &half), "owned - f64");
    assert_same_value(&(&value - 0.5), &(&value - &half), "borrowed - f64");
    assert_same_value(&(0.5 - value.clone()), &(&half - &value), "f64 - owned");
    assert_same_value(&(value.clone() * 0.5), &(&value * &half), "owned * f64");
    assert_same_value(&(&value * 0.5), &(&value * &half), "borrowed * f64");
    assert_same_value(&(0.5 * value.clone()), &(&half * &value), "f64 * owned");
    assert_same_value(
        &(value.clone() / 0.5).expect("nonzero divisor"),
        &(&value / &half).expect("nonzero divisor"),
        "owned / f64",
    );
    assert_same_value(
        &(&value / 0.5).expect("nonzero divisor"),
        &(&value / &half).expect("nonzero divisor"),
        "borrowed / f64",
    );
    assert_same_value(
        &(0.5 / value.clone()).expect("nonzero divisor"),
        &(&half / &value).expect("nonzero divisor"),
        "f64 / owned",
    );

    let mut assigned = value.clone();
    assigned += 0.5;
    assigned -= 0.5;
    assigned *= 0.5;
    assigned /= 0.5;
    assert_same_value(&assigned, &value, "f64 assignment round trip");

    let owned_sum: Real = [value.clone(), half.clone()].into_iter().sum();
    let values = [value.clone(), half.clone()];
    let borrowed_sum: Real = values.iter().sum();
    assert_same_value(&owned_sum, &borrowed_sum, "iterator sums");

    assert_same_value(
        &Real::average_pair(&value, &Real::zero()),
        &(&value * &half),
        "average right zero",
    );
    assert_same_value(
        &(Real::from(3) - Real::pi()),
        &-(Real::pi() - Real::from(3)),
        "three minus pi",
    );

    let exact = Rational::fraction(3, 2).expect("nonzero denominator");
    let exact_real = Real::new(exact.clone());
    assert_eq!(exact_real, exact);
    assert_eq!(exact, exact_real);
    assert_ne!(Real::pi(), Rational::one());
    assert_ne!(Rational::one(), Real::pi());
}

#[test]
fn exact_rational_storage_and_primitive_status_matrix_is_exhaustive() {
    let cases = [
        ("zero", Real::zero(), RationalStorageClass::Zero),
        (
            "binary64 negative zero",
            Real::try_from(-0.0_f64).expect("finite signed zero"),
            RationalStorageClass::Zero,
        ),
        (
            "binary64 dyadic",
            Real::try_from(0.1_f64).expect("finite f64"),
            RationalStorageClass::WordSized,
        ),
        (
            "word non-dyadic",
            fraction(1, 3),
            RationalStorageClass::WordSized,
        ),
        (
            "binary64 subnormal",
            Real::try_from(f64::from_bits(1)).expect("finite subnormal"),
            RationalStorageClass::MultiLimb,
        ),
        (
            "binary32 dyadic",
            Real::try_from(0.1_f32).expect("finite f32"),
            RationalStorageClass::WordSized,
        ),
        (
            "multi-limb",
            "1267650600228229401496703205377"
                .parse::<Real>()
                .expect("101-bit exact integer"),
            RationalStorageClass::MultiLimb,
        ),
        (
            "very-large",
            format!("1{}", "0".repeat(1_300))
                .parse::<Real>()
                .expect("large exact integer"),
            RationalStorageClass::VeryLarge,
        ),
    ];

    let mut observed_storage = [false; 4];
    let mut observed_f32 = [false; 5];
    let mut observed_f64 = [false; 5];
    for (name, value, expected_storage) in cases {
        let facts = value.detailed_facts();
        assert_eq!(facts.symbolic.kind, StructuralKind::ExactRational, "{name}");
        assert_eq!(facts.rational.storage, expected_storage, "{name}");
        observed_storage[rational_storage_index(facts.rational.storage)] = true;
        observed_f32[primitive_status_index(facts.primitive.f32)] = true;
        observed_f64[primitive_status_index(facts.primitive.f64)] = true;
    }

    let opaque = Real::e().sin().detailed_facts();
    observed_f32[primitive_status_index(opaque.primitive.f32)] = true;
    observed_f64[primitive_status_index(opaque.primitive.f64)] = true;
    assert_eq!(
        observed_storage, [true; 4],
        "missing rational storage class"
    );
    assert_eq!(observed_f32, [true; 5], "missing binary32 status");
    assert_eq!(observed_f64, [true; 5], "missing binary64 status");
}

fn assert_cache_contains(value: &Real, expected: &str, context: &str) {
    let debug = format!("{value:?}");
    assert!(
        debug.contains(expected),
        "{context}: expected {expected:?} in {debug}"
    );
}

fn assert_cache_excludes(value: &Real, excluded: &str, context: &str) {
    let debug = format!("{value:?}");
    assert!(
        !debug.contains(excluded),
        "{context}: unexpected {excluded:?} in {debug}"
    );
}

#[test]
fn cache_scale_and_abort_state_space_preserves_certificates() {
    for case in optimized_certificate_representatives() {
        let warmed_f32 = case.value.clone();
        let first_f32 = warmed_f32.to_f32_lossy();
        assert_eq!(warmed_f32.to_f32_lossy(), first_f32);
        match std::env::var("HYPERREAL_EXPECT_F32_CACHE").as_deref() {
            Ok("present") => assert_cache_contains(
                &warmed_f32,
                "primitive_approx_cache: F32(Some(",
                case.certificate,
            ),
            Ok("absent") => assert_cache_excludes(
                &warmed_f32,
                "primitive_approx_cache: F32(",
                case.certificate,
            ),
            Ok(other) => panic!("unknown HYPERREAL_EXPECT_F32_CACHE value {other:?}"),
            Err(_) => {}
        }

        let warmed_f64 = case.value.clone();
        let first_f64 = warmed_f64.to_f64_lossy();
        assert_eq!(warmed_f64.to_f64_lossy(), first_f64);
        assert_cache_contains(
            &warmed_f64,
            "primitive_approx_cache: F64(Some(",
            case.certificate,
        );
        assert_eq!(
            warmed_f64.certified_sign_until(-160),
            case.value.certified_sign_until(-160),
        );
    }

    let warmed_exact = fraction(1, 3);
    assert!(warmed_exact.to_f64_lossy().is_some());
    let exact_clone = warmed_exact.clone();
    match std::env::var("HYPERREAL_EXPECT_EXACT_CLONE_CACHE").as_deref() {
        Ok("present") => assert_cache_contains(
            &exact_clone,
            "primitive_approx_cache: F64(Some(",
            "exact-rational clone",
        ),
        Ok("absent") => assert_cache_contains(
            &exact_clone,
            "primitive_approx_cache: Empty",
            "exact-rational clone",
        ),
        Ok(other) => panic!("unknown HYPERREAL_EXPECT_EXACT_CLONE_CACHE value {other:?}"),
        Err(_) => {}
    }

    let huge: Real = format!("1{}", "0".repeat(1_300)).parse().unwrap();
    let huge_f32 = huge.clone();
    assert_eq!(huge_f32.to_f32_lossy(), None);
    assert_eq!(huge_f32.to_f32_lossy(), None);
    match std::env::var("HYPERREAL_EXPECT_F32_CACHE").as_deref() {
        Ok("present") => assert_cache_contains(
            &huge_f32,
            "primitive_approx_cache: F32(None)",
            "binary32 overflow",
        ),
        Ok("absent") => assert_cache_excludes(
            &huge_f32,
            "primitive_approx_cache: F32(",
            "binary32 overflow",
        ),
        Ok(other) => panic!("unknown HYPERREAL_EXPECT_F32_CACHE value {other:?}"),
        Err(_) => {}
    }
    let huge_f64 = huge.clone();
    assert_eq!(huge_f64.to_f64_lossy(), None);
    assert_eq!(huge_f64.to_f64_lossy(), None);
    assert_cache_contains(
        &huge_f64,
        "primitive_approx_cache: F64(None)",
        "binary64 overflow",
    );

    for mut value in [Real::from(3), Real::pi(), Real::one().sin()] {
        let signal = Arc::new(AtomicBool::new(false));
        value.abort(Arc::clone(&signal));
        assert!(!signal.load(AtomicOrdering::Relaxed));
        assert!(value.certified_dyadic_interval(-160).is_some());
    }

    let signal = Arc::new(AtomicBool::new(true));
    let mut exact = Real::from(3);
    exact.abort(Arc::clone(&signal));
    assert_eq!(exact.immediate_sign(), Some(RealSign::Positive));

    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let mut unresolved = &sine * &sine + &cosine * &cosine - Real::one();
    unresolved.abort(signal);
    assert!(matches!(
        unresolved.certified_sign_until(-160),
        CertifiedRealSign::Unknown { .. }
    ));
}

#[cfg(feature = "serde")]
fn serialized_computable(value: &hyperreal::Computable) -> serde_json::Value {
    serde_json::to_value(value).expect("Computable serializes")
}

#[cfg(feature = "serde")]
fn serialized_rational(numerator: i64, denominator: u64) -> serde_json::Value {
    serde_json::to_value(
        Rational::fraction(numerator, denominator).expect("nonzero serialized denominator"),
    )
    .expect("Rational serializes")
}

#[cfg(feature = "serde")]
fn computable_from_internal(internal: serde_json::Value) -> hyperreal::Computable {
    serde_json::from_value(serde_json::json!({ "internal": internal }))
        .expect("valid serialized Computable node")
}

#[cfg(feature = "serde")]
fn computable_root_tag(value: &hyperreal::Computable) -> String {
    let serialized = serialized_computable(value);
    match serialized
        .get("internal")
        .expect("serialized Computable has an internal node")
    {
        serde_json::Value::String(name) => name.clone(),
        serde_json::Value::Object(fields) if fields.len() == 1 => fields
            .keys()
            .next()
            .expect("single-variant object has one key")
            .clone(),
        internal => panic!("unexpected serialized Computable node: {internal}"),
    }
}

#[cfg(feature = "serde")]
fn opaque_real_from_computable(value: &hyperreal::Computable) -> Real {
    let mut serialized: serde_json::Value =
        serde_json::from_str(&Real::one().sin().to_json()).expect("valid opaque Real template");
    serialized["rational"] = serde_json::to_value(Rational::one()).unwrap();
    serialized["class"] = serde_json::Value::String("Irrational".into());
    serialized["computable"] = serialized_computable(value);
    serde_json::from_value(serialized).expect("valid opaque Real with supplied graph")
}

#[cfg(feature = "serde")]
fn quoted_variant_list(names: &[&str]) -> String {
    names
        .iter()
        .map(|name| format!("`{name}`"))
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(feature = "serde")]
fn serde_reported_variants(error: &str) -> &str {
    error
        .split_once("expected one of ")
        .expect("serde reports the complete private variant set")
        .1
        .split(" at line ")
        .next()
        .expect("variant list precedes source location")
}

#[cfg(feature = "serde")]
fn exhaustive_computable_nodes() -> Vec<(&'static str, hyperreal::Computable)> {
    use hyperreal::Computable;

    let rational = |numerator, denominator| {
        Computable::rational(
            Rational::fraction(numerator, denominator).expect("nonzero node denominator"),
        )
    };
    let child = serialized_computable(&rational(1, 8));
    let half = serialized_computable(&rational(1, 2));
    let one = serialized_computable(&Computable::one());
    let two = serialized_computable(&rational(2, 1));
    let zero = serialized_computable(&Computable::zero());
    let int_two = serialized_computable(&Computable::one().add(Computable::one()))
        .get("internal")
        .and_then(|internal| internal.get("Int"))
        .cloned()
        .expect("one plus one stores an Int payload");
    let int_zero = zero
        .get("internal")
        .and_then(|internal| internal.get("Int"))
        .cloned()
        .expect("zero stores an Int payload");
    let r_one_eighth = serialized_rational(1, 8);
    let r_one_half = serialized_rational(1, 2);
    let r_nine_eighths = serialized_rational(9, 8);
    let r_one = serialized_rational(1, 1);
    let r_two = serialized_rational(2, 1);
    let r_eight = serialized_rational(8, 1);

    vec![
        ("Int", Computable::zero()),
        ("One", Computable::one()),
        ("Constant", Computable::pi()),
        (
            "Inverse",
            computable_from_internal(serde_json::json!({ "Inverse": child.clone() })),
        ),
        (
            "Negate",
            computable_from_internal(serde_json::json!({ "Negate": child.clone() })),
        ),
        (
            "Add",
            computable_from_internal(serde_json::json!({ "Add": [child.clone(), half.clone()] })),
        ),
        (
            "Multiply",
            computable_from_internal(
                serde_json::json!({ "Multiply": [child.clone(), half.clone()] }),
            ),
        ),
        (
            "LinearCombination3",
            computable_from_internal(serde_json::json!({
                "LinearCombination3": {
                    "coefficients": [child.clone(), half.clone(), one.clone()],
                    "values": [r_one.clone(), r_two.clone(), r_one_half.clone()]
                }
            })),
        ),
        (
            "Square",
            computable_from_internal(serde_json::json!({ "Square": half.clone() })),
        ),
        ("Ratio", rational(1, 8)),
        (
            "Offset",
            computable_from_internal(serde_json::json!({ "Offset": [child.clone(), 1] })),
        ),
        (
            "PrescaledExp",
            computable_from_internal(serde_json::json!({ "PrescaledExp": child.clone() })),
        ),
        (
            "Expm1",
            computable_from_internal(serde_json::json!({ "Expm1": child.clone() })),
        ),
        (
            "Sqrt",
            computable_from_internal(serde_json::json!({ "Sqrt": half.clone() })),
        ),
        (
            "PrescaledLn",
            computable_from_internal(
                serde_json::json!({ "PrescaledLn": serialized_computable(&rational(9, 8)) }),
            ),
        ),
        (
            "PrescaledLnRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledLnRational": r_nine_eighths.clone() }),
            ),
        ),
        (
            "BinaryScaledLnRational",
            computable_from_internal(
                serde_json::json!({ "BinaryScaledLnRational": { "residual": r_nine_eighths.clone(), "shift": 0 } }),
            ),
        ),
        (
            "IntegralAtan",
            computable_from_internal(serde_json::json!({ "IntegralAtan": int_two })),
        ),
        (
            "PrescaledAtan",
            computable_from_internal(serde_json::json!({ "PrescaledAtan": child.clone() })),
        ),
        (
            "AtanDeferred",
            computable_from_internal(serde_json::json!({ "AtanDeferred": half.clone() })),
        ),
        (
            "AtanRational",
            computable_from_internal(serde_json::json!({ "AtanRational": r_one_half.clone() })),
        ),
        (
            "AsinRational",
            computable_from_internal(serde_json::json!({ "AsinRational": r_one_half.clone() })),
        ),
        (
            "PrescaledAsin",
            computable_from_internal(serde_json::json!({ "PrescaledAsin": child.clone() })),
        ),
        (
            "AsinDeferred",
            computable_from_internal(serde_json::json!({ "AsinDeferred": half.clone() })),
        ),
        (
            "AcosPositive",
            computable_from_internal(serde_json::json!({ "AcosPositive": half.clone() })),
        ),
        (
            "AcosPositiveRational",
            computable_from_internal(
                serde_json::json!({ "AcosPositiveRational": r_one_half.clone() }),
            ),
        ),
        (
            "AcosNegativeRational",
            computable_from_internal(
                serde_json::json!({ "AcosNegativeRational": r_one_half.clone() }),
            ),
        ),
        (
            "AcoshNearOne",
            computable_from_internal(
                serde_json::json!({ "AcoshNearOne": serialized_computable(&rational(9, 8)) }),
            ),
        ),
        (
            "AcoshDirect",
            computable_from_internal(serde_json::json!({ "AcoshDirect": two.clone() })),
        ),
        (
            "AsinhNearZero",
            computable_from_internal(serde_json::json!({ "AsinhNearZero": half.clone() })),
        ),
        (
            "AsinhDirect",
            computable_from_internal(serde_json::json!({ "AsinhDirect": two.clone() })),
        ),
        (
            "PrescaledAsinh",
            computable_from_internal(serde_json::json!({ "PrescaledAsinh": child.clone() })),
        ),
        (
            "AsinhRational",
            computable_from_internal(serde_json::json!({ "AsinhRational": r_one_eighth.clone() })),
        ),
        (
            "AtanhDirect",
            computable_from_internal(serde_json::json!({ "AtanhDirect": half.clone() })),
        ),
        (
            "PrescaledAtanh",
            computable_from_internal(serde_json::json!({ "PrescaledAtanh": child.clone() })),
        ),
        (
            "AtanhRational",
            computable_from_internal(serde_json::json!({ "AtanhRational": r_one_eighth.clone() })),
        ),
        (
            "PrescaledCos",
            computable_from_internal(serde_json::json!({ "PrescaledCos": child.clone() })),
        ),
        (
            "PrescaledCosRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledCosRational": r_one_eighth.clone() }),
            ),
        ),
        (
            "CosLargeRational",
            computable_from_internal(serde_json::json!({ "CosLargeRational": r_eight.clone() })),
        ),
        (
            "PrescaledCosHalfPiMinusRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledCosHalfPiMinusRational": r_one.clone() }),
            ),
        ),
        (
            "PrescaledSin",
            computable_from_internal(serde_json::json!({ "PrescaledSin": child.clone() })),
        ),
        (
            "PrescaledSinRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledSinRational": r_one_eighth.clone() }),
            ),
        ),
        (
            "SinLargeRational",
            computable_from_internal(serde_json::json!({ "SinLargeRational": r_eight.clone() })),
        ),
        (
            "PrescaledSinHalfPiMinusRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledSinHalfPiMinusRational": r_one.clone() }),
            ),
        ),
        (
            "PrescaledCotHalfPiMinusRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledCotHalfPiMinusRational": r_one.clone() }),
            ),
        ),
        (
            "TanLargeRational",
            computable_from_internal(serde_json::json!({ "TanLargeRational": r_eight.clone() })),
        ),
        (
            "PrescaledTan",
            computable_from_internal(serde_json::json!({ "PrescaledTan": child.clone() })),
        ),
        (
            "PrescaledTanRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledTanRational": r_one_eighth.clone() }),
            ),
        ),
        (
            "PrescaledCot",
            computable_from_internal(serde_json::json!({ "PrescaledCot": half.clone() })),
        ),
        (
            "ErfSeries",
            computable_from_internal(serde_json::json!({ "ErfSeries": child.clone() })),
        ),
        (
            "Erfc",
            computable_from_internal(serde_json::json!({ "Erfc": one.clone() })),
        ),
        (
            "NormalSf",
            computable_from_internal(serde_json::json!({ "NormalSf": one.clone() })),
        ),
        (
            "NormalInterval",
            computable_from_internal(
                serde_json::json!({ "NormalInterval": { "lo": zero.clone(), "hi": one.clone() } }),
            ),
        ),
        (
            "LogPnorm",
            computable_from_internal(serde_json::json!({ "LogPnorm": one.clone() })),
        ),
        (
            "LogNormalSf",
            computable_from_internal(serde_json::json!({ "LogNormalSf": one.clone() })),
        ),
        (
            "LogDnorm",
            computable_from_internal(serde_json::json!({ "LogDnorm": one.clone() })),
        ),
        (
            "NormalQuantile",
            computable_from_internal(
                serde_json::json!({ "NormalQuantile": { "p": half, "seed": int_zero, "seed_prec": -16 } }),
            ),
        ),
        (
            "NthRoot",
            computable_from_internal(serde_json::json!({ "NthRoot": [child.clone(), 3] })),
        ),
        (
            "SincSmall",
            computable_from_internal(serde_json::json!({ "SincSmall": child.clone() })),
        ),
        (
            "CoscSmall",
            computable_from_internal(serde_json::json!({ "CoscSmall": child })),
        ),
    ]
}

#[cfg(feature = "serde")]
#[test]
fn every_private_class_survives_json_and_cbor_without_cache_state() {
    let cases = optimized_certificate_representatives();
    let names = cases
        .iter()
        .map(|case| case.certificate)
        .collect::<Vec<_>>();

    let mut probe: serde_json::Value = serde_json::from_str(&Real::one().to_json()).unwrap();
    probe["class"] = serde_json::Value::String("__hyperreal_variant_probe__".into());
    let error = serde_json::from_value::<Real>(probe)
        .expect_err("unknown private class must be rejected")
        .to_string();
    assert_eq!(
        serde_reported_variants(&error),
        quoted_variant_list(&names),
        "private Real class inventory drifted",
    );

    for case in cases {
        let value = case.value.clone();
        let _ = value.to_f32_lossy();
        let _ = value.to_f64_lossy();
        let json = value.to_json();
        let serialized: serde_json::Value = serde_json::from_str(&json).unwrap();
        let class = serialized.get("class").expect("serialized class");
        let class_name = match class {
            serde_json::Value::String(name) => name.as_str(),
            serde_json::Value::Object(fields) if fields.len() == 1 => {
                fields.keys().next().unwrap().as_str()
            }
            _ => panic!(
                "unexpected serialized class for {}: {class}",
                case.certificate
            ),
        };
        assert_eq!(class_name, case.certificate);

        let from_json = Real::from_json(&json).expect("valid Real JSON");
        let from_cbor = Real::from_bytes(&value.to_bytes()).expect("valid Real CBOR");
        assert_eq!(from_json.detailed_facts().symbolic.kind, case.public_kind);
        assert_eq!(from_cbor.detailed_facts().symbolic.kind, case.public_kind);
        assert_same_value(&from_json, &value, case.certificate);
        assert_same_value(&from_cbor, &value, case.certificate);
        assert_cache_contains(
            &from_json,
            "primitive_approx_cache: Empty",
            case.certificate,
        );
    }
}

#[cfg(feature = "serde")]
#[test]
fn every_computable_node_and_shared_constant_variant_round_trips_and_evaluates() {
    use hyperreal::Computable;

    const NODE_NAMES: [&str; 60] = [
        "Int",
        "One",
        "Constant",
        "Inverse",
        "Negate",
        "Add",
        "Multiply",
        "LinearCombination3",
        "Square",
        "Ratio",
        "Offset",
        "PrescaledExp",
        "Expm1",
        "Sqrt",
        "PrescaledLn",
        "PrescaledLnRational",
        "BinaryScaledLnRational",
        "IntegralAtan",
        "PrescaledAtan",
        "AtanDeferred",
        "AtanRational",
        "AsinRational",
        "PrescaledAsin",
        "AsinDeferred",
        "AcosPositive",
        "AcosPositiveRational",
        "AcosNegativeRational",
        "AcoshNearOne",
        "AcoshDirect",
        "AsinhNearZero",
        "AsinhDirect",
        "PrescaledAsinh",
        "AsinhRational",
        "AtanhDirect",
        "PrescaledAtanh",
        "AtanhRational",
        "PrescaledCos",
        "PrescaledCosRational",
        "CosLargeRational",
        "PrescaledCosHalfPiMinusRational",
        "PrescaledSin",
        "PrescaledSinRational",
        "SinLargeRational",
        "PrescaledSinHalfPiMinusRational",
        "PrescaledCotHalfPiMinusRational",
        "TanLargeRational",
        "PrescaledTan",
        "PrescaledTanRational",
        "PrescaledCot",
        "ErfSeries",
        "Erfc",
        "NormalSf",
        "NormalInterval",
        "LogPnorm",
        "LogNormalSf",
        "LogDnorm",
        "NormalQuantile",
        "NthRoot",
        "SincSmall",
        "CoscSmall",
    ];
    const SHARED_CONSTANT_NAMES: [&str; 18] = [
        "E",
        "Pi",
        "InvPi",
        "Tau",
        "Ln2",
        "Ln3",
        "Ln5",
        "Ln6",
        "Ln7",
        "Ln10",
        "Sqrt2",
        "Sqrt3",
        "Acosh2",
        "Asinh1",
        "AtanInv2",
        "AtanInv5",
        "Atan2",
        "AtanThreeHalves",
    ];

    let node_error = serde_json::from_value::<Computable>(serde_json::json!({
        "internal": { "__hyperreal_node_probe__": null }
    }))
    .expect_err("unknown private node must be rejected")
    .to_string();
    assert_eq!(
        serde_reported_variants(&node_error),
        quoted_variant_list(&NODE_NAMES),
        "private Computable node inventory drifted",
    );

    let nodes = exhaustive_computable_nodes();
    assert_eq!(nodes.len(), NODE_NAMES.len());
    for ((expected_name, value), declared_name) in nodes.into_iter().zip(NODE_NAMES) {
        assert_eq!(expected_name, declared_name);
        assert_eq!(computable_root_tag(&value), expected_name);
        let restored: Computable = serde_json::from_value(serialized_computable(&value)).unwrap();
        assert_eq!(restored.approx(-24), value.approx(-24), "{expected_name}");
        assert_eq!(restored.approx(-48), value.approx(-48), "{expected_name}");
        if matches!(expected_name, "SincSmall" | "CoscSmall") {
            assert_eq!(restored.zero_status(), ZeroKnowledge::NonZero);
        }

        let carrier = opaque_real_from_computable(&restored);
        assert_eq!(
            carrier.detailed_facts().symbolic.kind,
            StructuralKind::ComputableOpaque,
            "{expected_name}",
        );
        assert_same_value(
            &(&carrier + Real::one() - Real::one()),
            &carrier,
            expected_name,
        );
    }

    let constant_error = serde_json::from_value::<Computable>(serde_json::json!({
        "internal": { "Constant": "__hyperreal_constant_probe__" }
    }))
    .expect_err("unknown shared constant must be rejected")
    .to_string();
    assert_eq!(
        serde_reported_variants(&constant_error),
        quoted_variant_list(&SHARED_CONSTANT_NAMES),
        "private shared-constant inventory drifted",
    );

    for name in SHARED_CONSTANT_NAMES {
        let value = computable_from_internal(serde_json::json!({ "Constant": name }));
        assert_eq!(computable_root_tag(&value), "Constant", "{name}");
        let restored: Computable = serde_json::from_value(serialized_computable(&value)).unwrap();
        assert_eq!(restored.approx(-24), value.approx(-24), "{name}");
        assert!(
            opaque_real_from_computable(&restored)
                .certified_dyadic_interval(-48)
                .is_some()
        );
    }
}
