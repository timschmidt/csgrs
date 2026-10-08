use core::cmp::Ordering;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering as AtomicOrdering},
};

use hyperlimit::{
    Plane3, PlaneSide, Point2, Point3, PointSegmentLocation, PredicateOutcome, PredicatePolicy,
    Sign, TriangleLocation, classify_point_plane, classify_point_segment, classify_point_triangle,
    classify_real_sign, compare_point_triangle3_distance_squared, compare_reals, orient2, orient3,
};
use hyperreal::{PrimitiveFloatStatus, Rational, RationalStorageClass, Real, StructuralKind};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;
const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

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
            value: fraction(1, 3)
                .exp10()
                .expect("finite rational base-ten power"),
        },
        RepresentationCase {
            certificate: "Pow2",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 3)
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

fn assert_decided<T>(actual: PredicateOutcome<T>, expected: T, context: &str)
where
    T: Copy + core::fmt::Debug + PartialEq,
{
    assert_eq!(actual.value(), Some(expected), "{context}: {actual:?}");
}

fn assert_private_cache_debug(value: &Real, expected: &str, context: &str) {
    let debug = format!("{value:?}");
    assert!(
        debug.contains(expected),
        "{context}: expected private cache state containing {expected:?}, got {debug}"
    );
}

fn assert_private_cache_debug_excludes(value: &Real, excluded: &str, context: &str) {
    let debug = format!("{value:?}");
    assert!(
        !debug.contains(excluded),
        "{context}: private cache unexpectedly contained {excluded:?}: {debug}"
    );
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

fn assert_translation_predicates(case: &RepresentationCase, policy: PredicatePolicy) {
    let carrier = &case.value;
    let one = Real::one();
    let two = Real::from(2);
    let half = fraction(1, 2);

    let a2 = Point2::new(carrier.clone(), carrier.clone());
    let b2 = Point2::new(carrier + &two, carrier.clone());
    let c2 = Point2::new(carrier.clone(), carrier + &two);
    let interior2 = Point2::new(carrier + &half, carrier + &half);
    let segment_end = Point2::new(carrier + &two, carrier + &two);

    assert_decided(
        compare_reals(carrier, &(carrier + &one), policy),
        Ordering::Less,
        case.certificate,
    );
    assert_decided(
        orient2(&a2, &b2, &c2, policy),
        Sign::Positive,
        case.certificate,
    );
    assert_decided(
        classify_point_segment(&a2, &segment_end, &interior2, policy),
        PointSegmentLocation::OnSegment,
        case.certificate,
    );
    assert_decided(
        classify_point_triangle(&a2, &b2, &c2, &interior2, policy),
        TriangleLocation::Inside,
        case.certificate,
    );

    let a3 = Point3::new(carrier.clone(), carrier.clone(), carrier.clone());
    let b3 = Point3::new(carrier + &one, carrier.clone(), carrier.clone());
    let c3 = Point3::new(carrier.clone(), carrier + &one, carrier.clone());
    let d3 = Point3::new(carrier.clone(), carrier.clone(), carrier + &one);
    assert_decided(
        orient3(&a3, &b3, &c3, &d3, policy),
        Sign::Negative,
        case.certificate,
    );
    assert_decided(
        compare_point_triangle3_distance_squared(&a3, &a3, &a3, &a3, &Real::zero(), policy),
        Ordering::Equal,
        case.certificate,
    );

    let translated_plane = Plane3::new(
        Point3::new(Real::zero(), Real::zero(), Real::one()),
        -carrier.clone(),
    );
    assert_decided(
        classify_point_plane(&d3, &translated_plane, policy),
        PlaneSide::Above,
        case.certificate,
    );
}

#[test]
fn every_public_kind_and_optimized_certificate_crosses_predicate_families() {
    let cases = optimized_certificate_representatives();
    assert_eq!(cases.len(), 22, "update the exhaustive certificate matrix");

    let mut observed_kinds = [false; 8];
    for case in &cases {
        let actual_kind = case.value.detailed_facts().symbolic.kind;
        assert_eq!(actual_kind, case.public_kind, "{} recipe", case.certificate);
        observed_kinds[structural_kind_index(actual_kind)] = true;

        assert_translation_predicates(case, STRICT);
        assert_translation_predicates(case, APPROX);
    }

    assert_eq!(observed_kinds, [true; 8], "missing public Real kind");
}

#[test]
fn exact_rational_storage_and_primitive_origin_matrix_crosses_predicates() {
    let zero = Real::zero();
    let negative_zero = Real::try_from(-0.0_f64).expect("finite signed zero");
    let word_dyadic = Real::try_from(0.1_f64).expect("finite f64");
    let word_non_dyadic = fraction(1, 3);
    let subnormal = Real::try_from(f64::from_bits(1)).expect("finite subnormal");
    let from_f32 = Real::try_from(0.1_f32).expect("finite f32");
    let multi_limb: Real = "1267650600228229401496703205377"
        .parse()
        .expect("101-bit exact integer");
    let very_large: Real = format!("1{}", "0".repeat(1_300))
        .parse()
        .expect("large exact integer");

    let cases = [
        ("zero", zero, RationalStorageClass::Zero),
        (
            "binary64 negative zero",
            negative_zero,
            RationalStorageClass::Zero,
        ),
        (
            "binary64 dyadic",
            word_dyadic,
            RationalStorageClass::WordSized,
        ),
        (
            "word non-dyadic",
            word_non_dyadic,
            RationalStorageClass::WordSized,
        ),
        (
            "binary64 subnormal dyadic",
            subnormal,
            RationalStorageClass::MultiLimb,
        ),
        ("binary32 dyadic", from_f32, RationalStorageClass::WordSized),
        ("multi-limb", multi_limb, RationalStorageClass::MultiLimb),
        ("very-large", very_large, RationalStorageClass::VeryLarge),
    ];

    let mut observed_storage = [false; 4];
    let mut observed_f32_status = [false; 5];
    let mut observed_f64_status = [false; 5];
    for (name, value, storage) in cases {
        let facts = value.detailed_facts();
        assert_eq!(facts.symbolic.kind, StructuralKind::ExactRational, "{name}");
        assert_eq!(facts.rational.storage, storage, "{name}");
        observed_storage[rational_storage_index(facts.rational.storage)] = true;
        observed_f32_status[primitive_status_index(facts.primitive.f32)] = true;
        observed_f64_status[primitive_status_index(facts.primitive.f64)] = true;
        let case = RepresentationCase {
            certificate: name,
            public_kind: StructuralKind::ExactRational,
            value,
        };
        assert_translation_predicates(&case, STRICT);
    }

    let opaque_facts = Real::e().sin().detailed_facts();
    observed_f32_status[primitive_status_index(opaque_facts.primitive.f32)] = true;
    observed_f64_status[primitive_status_index(opaque_facts.primitive.f64)] = true;
    assert_eq!(
        observed_storage, [true; 4],
        "missing rational storage class"
    );
    assert_eq!(
        observed_f32_status, [true; 5],
        "missing binary32 range representation"
    );
    assert_eq!(
        observed_f64_status, [true; 5],
        "missing binary64 range representation"
    );
}

#[test]
fn sign_scale_cache_and_abort_states_preserve_predicate_results() {
    for case in optimized_certificate_representatives() {
        let positive = classify_real_sign(&case.value, STRICT).value();
        assert_eq!(
            positive,
            Some(Sign::Positive),
            "{} positive",
            case.certificate
        );
        assert_eq!(
            classify_real_sign(&(-case.value.clone()), STRICT).value(),
            Some(Sign::Negative),
            "{} negative",
            case.certificate,
        );

        let fractional_scale = &case.value * &fraction(3, 5);
        assert_eq!(
            fractional_scale.detailed_facts().symbolic.kind,
            case.public_kind,
            "{} fractional scale",
            case.certificate,
        );
        assert_translation_predicates(
            &RepresentationCase {
                certificate: case.certificate,
                public_kind: case.public_kind,
                value: fractional_scale,
            },
            STRICT,
        );

        // Separate clones ensure builds with both Hyperreal cache features
        // materialize both private cache variants instead of upgrading one
        // object from F32 to F64 before predicate use.
        let warmed_f32 = case.value.clone();
        let first_f32 = warmed_f32.to_f32_lossy();
        assert_eq!(warmed_f32.to_f32_lossy(), first_f32);
        match std::env::var("HYPERLIMIT_EXPECT_F32_CACHE").as_deref() {
            Ok("present") => assert_private_cache_debug(
                &warmed_f32,
                "primitive_approx_cache: F32(Some(",
                case.certificate,
            ),
            Ok("absent") => assert_private_cache_debug_excludes(
                &warmed_f32,
                "primitive_approx_cache: F32(",
                case.certificate,
            ),
            Ok(other) => panic!("unknown HYPERLIMIT_EXPECT_F32_CACHE value {other:?}"),
            Err(_) => {}
        }
        assert_translation_predicates(
            &RepresentationCase {
                certificate: case.certificate,
                public_kind: case.public_kind,
                value: warmed_f32,
            },
            STRICT,
        );

        let warmed_f64 = case.value.clone();
        let first_f64 = warmed_f64.to_f64_lossy();
        assert_eq!(warmed_f64.to_f64_lossy(), first_f64);
        assert_private_cache_debug(
            &warmed_f64,
            "primitive_approx_cache: F64(Some(",
            case.certificate,
        );
        assert_translation_predicates(
            &RepresentationCase {
                certificate: case.certificate,
                public_kind: case.public_kind,
                value: warmed_f64,
            },
            STRICT,
        );
    }

    let huge: Real = format!("1{}", "0".repeat(1_300)).parse().unwrap();
    let huge_f32 = huge.clone();
    assert_eq!(huge_f32.to_f32_lossy(), None, "cache F32(None)");
    assert_eq!(huge_f32.to_f32_lossy(), None, "reuse F32(None)");
    match std::env::var("HYPERLIMIT_EXPECT_F32_CACHE").as_deref() {
        Ok("present") => assert_private_cache_debug(
            &huge_f32,
            "primitive_approx_cache: F32(None)",
            "binary32 overflow",
        ),
        Ok("absent") => assert_private_cache_debug_excludes(
            &huge_f32,
            "primitive_approx_cache: F32(",
            "binary32 overflow",
        ),
        Ok(other) => panic!("unknown HYPERLIMIT_EXPECT_F32_CACHE value {other:?}"),
        Err(_) => {}
    }
    let huge_f64 = huge.clone();
    assert_eq!(huge_f64.to_f64_lossy(), None, "cache F64(None)");
    assert_eq!(huge_f64.to_f64_lossy(), None, "reuse F64(None)");
    assert_private_cache_debug(
        &huge_f64,
        "primitive_approx_cache: F64(None)",
        "binary64 overflow",
    );
    assert_translation_predicates(
        &RepresentationCase {
            certificate: "One + cached f64 overflow",
            public_kind: StructuralKind::ExactRational,
            value: huge,
        },
        STRICT,
    );

    for (certificate, public_kind, mut abort_attached) in [
        (
            "One + abort signal",
            StructuralKind::ExactRational,
            Real::from(3),
        ),
        ("Pi + abort signal", StructuralKind::PiLike, Real::pi()),
        (
            "Irrational + abort signal",
            StructuralKind::ComputableOpaque,
            Real::one().sin(),
        ),
    ] {
        let signal = Arc::new(AtomicBool::new(false));
        abort_attached.abort(Arc::clone(&signal));
        assert!(!signal.load(AtomicOrdering::Relaxed));
        let case = RepresentationCase {
            certificate,
            public_kind,
            value: abort_attached,
        };
        assert_translation_predicates(&case, STRICT);
    }

    let signal = Arc::new(AtomicBool::new(true));
    let mut aborted_rational = Real::from(3);
    aborted_rational.abort(Arc::clone(&signal));
    assert_eq!(
        classify_real_sign(&aborted_rational, STRICT).value(),
        Some(Sign::Positive),
        "a triggered abort does not erase an exact-rational sign certificate",
    );

    let mut aborted_pi = Real::pi();
    aborted_pi.abort(Arc::clone(&signal));
    assert_eq!(
        classify_real_sign(&aborted_pi, STRICT).value(),
        Some(Sign::Positive),
        "a triggered abort does not erase a pi sign certificate",
    );

    let mut aborted_opaque = Real::one().sin();
    aborted_opaque.abort(Arc::clone(&signal));
    assert_eq!(
        classify_real_sign(&aborted_opaque, STRICT).value(),
        Some(Sign::Positive),
        "a triggered abort does not erase a computable node's exact sign certificate",
    );

    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let mut aborted_unresolved = &sine * &sine + &cosine * &cosine - Real::one();
    aborted_unresolved.abort(signal);
    assert!(matches!(
        classify_real_sign(&aborted_unresolved, STRICT),
        PredicateOutcome::Unknown { .. }
    ));
}

#[test]
fn unresolved_opaque_terminal_form_respects_policy_boundary() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let terminal_zero = &sine * &sine + &cosine * &cosine - Real::one();
    assert_eq!(
        terminal_zero.detailed_facts().symbolic.kind,
        StructuralKind::ComputableOpaque
    );
    assert!(matches!(
        classify_real_sign(&terminal_zero, STRICT),
        PredicateOutcome::Unknown { .. }
    ));
    assert_eq!(
        classify_real_sign(&terminal_zero, APPROX).value(),
        Some(Sign::Zero)
    );
}

#[cfg(feature = "serde")]
#[test]
fn every_optimized_certificate_survives_serialized_representation() {
    let cases = optimized_certificate_representatives();
    let expected_names = cases
        .iter()
        .map(|case| case.certificate)
        .collect::<Vec<_>>();

    // Serde's generated unknown-variant diagnostic is the only public view of
    // Hyperreal's private Class enumeration. Comparing it with this matrix
    // makes a newly added private certificate fail here until it receives a
    // predicate representative.
    let mut probe: serde_json::Value =
        serde_json::from_str(&Real::one().to_json()).expect("valid probe JSON");
    probe["class"] = serde_json::Value::String("__hyperlimit_variant_probe__".into());
    let error = serde_json::from_value::<Real>(probe)
        .expect_err("an unknown private class must be rejected")
        .to_string();
    let reported = error
        .split_once("expected one of ")
        .expect("serde reports the complete private variant set")
        .1
        .split(" at line ")
        .next()
        .expect("variant list precedes source location");
    let expected = expected_names
        .iter()
        .map(|name| format!("`{name}`"))
        .collect::<Vec<_>>()
        .join(", ");
    assert_eq!(reported, expected, "private Real class matrix drifted");

    for case in cases {
        let serialized_value = case.value.clone();
        let _ = serialized_value.to_f32_lossy();
        let _ = serialized_value.to_f64_lossy();
        let json = serialized_value.to_json();
        let serialized: serde_json::Value = serde_json::from_str(&json).expect("valid JSON");
        let class = serialized
            .get("class")
            .expect("serialized Real retains its class certificate");
        let class_name = match class {
            serde_json::Value::String(name) => name.as_str(),
            serde_json::Value::Object(fields) if fields.len() == 1 => fields
                .keys()
                .next()
                .expect("single variant object has one key"),
            _ => panic!(
                "unexpected serialized class for {}: {class}",
                case.certificate
            ),
        };
        assert_eq!(class_name, case.certificate, "certificate recipe drifted");

        let restored = Real::from_json(&json).expect("valid Real JSON");
        assert_eq!(
            restored.detailed_facts().symbolic.kind,
            case.public_kind,
            "{} serialized kind",
            case.certificate,
        );
        let restored_case = RepresentationCase {
            certificate: case.certificate,
            public_kind: case.public_kind,
            value: restored,
        };
        assert_translation_predicates(&restored_case, STRICT);

        let bytes = serialized_value.to_bytes();
        let restored = Real::from_bytes(&bytes).expect("valid Real CBOR");
        assert_eq!(
            restored.detailed_facts().symbolic.kind,
            case.public_kind,
            "{} CBOR kind",
            case.certificate,
        );
        assert_translation_predicates(
            &RepresentationCase {
                certificate: case.certificate,
                public_kind: case.public_kind,
                value: restored,
            },
            STRICT,
        );
    }
}

#[cfg(feature = "serde")]
fn computable_from_internal(internal: serde_json::Value) -> hyperreal::Computable {
    serde_json::from_value(serde_json::json!({ "internal": internal }))
        .expect("valid serialized Computable node")
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
fn computable_root_tag(value: &hyperreal::Computable) -> String {
    let serialized = serialized_computable(value);
    let internal = serialized
        .get("internal")
        .expect("serialized Computable has an internal node");
    match internal {
        serde_json::Value::String(name) => name.clone(),
        serde_json::Value::Object(fields) if fields.len() == 1 => fields
            .keys()
            .next()
            .expect("single-variant object has one key")
            .clone(),
        _ => panic!("unexpected serialized Computable node: {internal}"),
    }
}

#[cfg(feature = "serde")]
fn opaque_real_from_computable(value: &hyperreal::Computable) -> Real {
    let mut serialized: serde_json::Value =
        serde_json::from_str(&Real::one().sin().to_json()).expect("valid opaque Real template");
    serialized["rational"] =
        serde_json::to_value(Rational::one()).expect("unit rational serializes");
    serialized["class"] = serde_json::Value::String("Irrational".into());
    serialized["computable"] = serialized_computable(value);
    serde_json::from_value(serialized).expect("valid opaque Real with supplied Computable graph")
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
            computable_from_internal(serde_json::json!({
                "PrescaledLn": serialized_computable(&rational(9, 8))
            })),
        ),
        (
            "PrescaledLnRational",
            computable_from_internal(
                serde_json::json!({ "PrescaledLnRational": r_nine_eighths.clone() }),
            ),
        ),
        (
            "BinaryScaledLnRational",
            computable_from_internal(serde_json::json!({
                "BinaryScaledLnRational": { "residual": r_nine_eighths.clone(), "shift": 0 }
            })),
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
            computable_from_internal(serde_json::json!({
                "AcoshNearOne": serialized_computable(&rational(9, 8))
            })),
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
            computable_from_internal(serde_json::json!({
                "PrescaledCosHalfPiMinusRational": r_one.clone()
            })),
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
            computable_from_internal(serde_json::json!({
                "PrescaledSinHalfPiMinusRational": r_one.clone()
            })),
        ),
        (
            "PrescaledCotHalfPiMinusRational",
            computable_from_internal(serde_json::json!({
                "PrescaledCotHalfPiMinusRational": r_one.clone()
            })),
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
            computable_from_internal(serde_json::json!({
                "NormalInterval": { "lo": zero.clone(), "hi": one.clone() }
            })),
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
            computable_from_internal(serde_json::json!({
                "NormalQuantile": { "p": half, "seed": int_zero, "seed_prec": -16 }
            })),
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
fn every_computable_node_and_shared_constant_representation_crosses_predicates() {
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
        "internal": { "__hyperlimit_node_probe__": null }
    }))
    .expect_err("an unknown private Computable node must be rejected")
    .to_string();
    assert_eq!(
        serde_reported_variants(&node_error),
        quoted_variant_list(&NODE_NAMES),
        "private Computable node matrix drifted",
    );

    let nodes = exhaustive_computable_nodes();
    assert_eq!(nodes.len(), NODE_NAMES.len());
    for ((expected_name, value), declared_name) in nodes.into_iter().zip(NODE_NAMES) {
        assert_eq!(expected_name, declared_name, "node recipe ordering drifted");
        assert_eq!(
            computable_root_tag(&value),
            expected_name,
            "{expected_name} recipe drifted",
        );
        let restored: Computable =
            serde_json::from_value(serialized_computable(&value)).expect("node round trip");
        assert_eq!(restored.approx(-24), value.approx(-24), "{expected_name}");

        let carrier = opaque_real_from_computable(&restored);
        assert_eq!(
            carrier.detailed_facts().symbolic.kind,
            StructuralKind::ComputableOpaque,
            "{expected_name}",
        );
        assert_translation_predicates(
            &RepresentationCase {
                certificate: expected_name,
                public_kind: StructuralKind::ComputableOpaque,
                value: carrier,
            },
            STRICT,
        );
    }

    let constant_error = serde_json::from_value::<Computable>(serde_json::json!({
        "internal": { "Constant": "__hyperlimit_constant_probe__" }
    }))
    .expect_err("an unknown private shared constant must be rejected")
    .to_string();
    assert_eq!(
        serde_reported_variants(&constant_error),
        quoted_variant_list(&SHARED_CONSTANT_NAMES),
        "private shared-constant matrix drifted",
    );

    for name in SHARED_CONSTANT_NAMES {
        let value = computable_from_internal(serde_json::json!({ "Constant": name }));
        assert_eq!(computable_root_tag(&value), "Constant", "{name}");
        let restored: Computable =
            serde_json::from_value(serialized_computable(&value)).expect("constant round trip");
        assert_eq!(restored.approx(-24), value.approx(-24), "{name}");
        assert_translation_predicates(
            &RepresentationCase {
                certificate: name,
                public_kind: StructuralKind::ComputableOpaque,
                value: opaque_real_from_computable(&restored),
            },
            STRICT,
        );
    }
}
