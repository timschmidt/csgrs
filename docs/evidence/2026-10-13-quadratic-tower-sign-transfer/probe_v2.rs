//! Prepared public-API observation; run each case in a fresh process.
use hyperreal::{Real, RealSign};

fn sign_name(sign: Option<RealSign>) -> &'static str {
    match sign {
        Some(RealSign::Positive) => "positive",
        Some(RealSign::Negative) => "negative",
        Some(RealSign::Zero) => "zero",
        None => "unknown",
    }
}

fn main() {
    let arguments = std::env::args().collect::<Vec<_>>();
    assert_eq!(arguments.len(), 4);
    let family = arguments[1].as_str();
    let shape = arguments[2].as_str();
    let scale = arguments[3].parse::<i32>().unwrap();
    assert!(matches!(scale, -7 | 1));
    let root = match family {
        "quadratic" => Real::from(2).sqrt().unwrap(),
        "nested" => (Real::from(3) + Real::from(5).sqrt().unwrap())
            .sqrt()
            .unwrap(),
        "biquadratic" => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        _ => panic!("unknown family"),
    };
    let lower = root.certified_rational_interval(-1024).unwrap()[0].clone();
    let delta = &root - Real::from(lower);
    let delta_before = delta.certified_sign_until(0).sign();
    let delta_warmed = delta.certified_sign_until(-1100).sign();
    assert_eq!(delta_warmed, Some(RealSign::Positive));
    let value = match shape {
        "linear" => delta.clone(),
        "square" => &delta * &delta,
        _ => panic!("unknown shape"),
    } * Real::from(scale);
    let expected = if scale < 0 {
        RealSign::Negative
    } else {
        RealSign::Positive
    };
    let original_immediate = value.immediate_sign();
    let original_zero = value.certified_sign_until(0).sign();
    let original_clone = value.clone();
    let original_clone_zero = original_clone.certified_sign_until(0).sign();
    let Some(compact) = value.compact_quadratic_tower() else {
        println!(
            "{{\"family\":\"{family}\",\"shape\":\"{shape}\",\"scale\":{scale},\"delta_before\":\"{}\",\"delta_warmed\":\"{}\",\"original_immediate\":\"{}\",\"original_zero\":\"{}\",\"original_clone_zero\":\"{}\",\"compact_covered\":false}}",
            sign_name(delta_before),
            sign_name(delta_warmed),
            sign_name(original_immediate),
            sign_name(original_zero),
            sign_name(original_clone_zero),
        );
        return;
    };
    let clone = compact.clone();
    let compact_immediate = compact.immediate_sign();
    let compact_zero = compact.certified_sign_until(0).sign();
    let compact_512 = compact.certified_sign_until(-512).sign();
    let clone_zero = clone.certified_sign_until(0).sign();
    let repeated = compact.compact_quadratic_tower();
    let repeated_zero = repeated
        .as_ref()
        .and_then(|value| value.certified_sign_until(0).sign());
    // Equality is queried last so it cannot warm earlier observations.
    let equality = (&compact - &value).quadratic_tower_sign();
    let original_after = value.certified_sign_until(0).sign();
    let definite_agreement = [
        original_immediate,
        original_zero,
        original_clone_zero,
        compact_immediate,
        compact_zero,
        compact_512,
        clone_zero,
        repeated_zero,
        original_after,
    ]
    .into_iter()
    .flatten()
    .all(|sign| sign == expected);
    println!(
        "{{\"family\":\"{family}\",\"shape\":\"{shape}\",\"scale\":{scale},\"delta_before\":\"{}\",\"delta_warmed\":\"{}\",\"original_immediate\":\"{}\",\"original_zero\":\"{}\",\"original_clone_zero\":\"{}\",\"compact_covered\":true,\"compact_immediate\":\"{}\",\"compact_zero\":\"{}\",\"compact_512\":\"{}\",\"clone_zero\":\"{}\",\"repeated_covered\":{},\"repeated_zero\":\"{}\",\"difference_tower_sign\":\"{}\",\"original_after\":\"{}\",\"all_definite_signs_agree\":{definite_agreement}}}",
        sign_name(delta_before),
        sign_name(delta_warmed),
        sign_name(original_immediate),
        sign_name(original_zero),
        sign_name(original_clone_zero),
        sign_name(compact_immediate),
        sign_name(compact_zero),
        sign_name(compact_512),
        sign_name(clone_zero),
        repeated.is_some(),
        sign_name(repeated_zero),
        sign_name(equality),
        sign_name(original_after),
    );
}
