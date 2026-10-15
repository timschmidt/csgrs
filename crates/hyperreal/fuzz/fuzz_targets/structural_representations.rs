//! Cross-representation checks using the shared finite `Real` certificate inventory.

#![no_main]

use hyperreal::{
    CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, Computable, Rational, Real,
    RealSign,
};
use libfuzzer_sys::fuzz_target;

#[path = "../../tests/support/representations.rs"]
mod representations;

fuzz_target!(|data: &[u8]| {
    let cases = representations::optimized_certificate_representatives();
    assert_eq!(cases.len(), 22, "update the optimized certificate matrix");

    for case in &cases {
        let value = &case.value;
        assert_eq!(
            value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        check_normal_and_affine_cache_contract(value);

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
        let _ = value.clone().cot();
        let _ = value.clone().atan();
        let _ = value.clone().sinh();
        let _ = value.clone().cosh();
        let _ = value.clone().tanh();
        let exponent = i64::from(data.first().copied().unwrap_or(0) % 9) - 4;
        let _ = value.clone().powi_i64(exponent);
    }

    // Each of the 22 retained stride seeds covers one rotation of the left-hand
    // certificates. Replaying all seeds covers all 484 ordered pairs without
    // forcing every execution to perform the complete matrix.
    let stride = usize::from(data.first().copied().unwrap_or(0)) % cases.len();
    for (index, case) in cases.iter().enumerate() {
        let left = &case.value;
        let right = &cases[(index + stride) % cases.len()].value;
        assert_bounded_equal(&(left + right), &(right + left));
        assert_bounded_equal(&(left * right), &(right * left));
        assert_bounded_equal(&(left - right), &-(right - left));

        let quotient = (left / right).expect("representatives are nonzero");
        assert!(quotient.certified_rational_interval(-512).is_some());

        assert!(matches!(
            left.certified_eq_until(left, -512),
            CertifiedRealEquality::Equal { .. }
        ));
        assert_certificates_match_bounded_evaluation(left, right);
    }

    // Finite node tags are covered deterministically by the serde inventory
    // test. This fuzzer covers the unbounded part of the representation space:
    // variable-depth, shared expression-DAG topology.
    let graph = variable_depth_graph(data);
    let cold_sign = graph.sign_until(0);
    let coarse = graph.approx(-32);
    assert_eq!(graph.approx(-32), coarse);
    let fine = graph.approx(-96);
    assert_eq!(graph.approx(-96), fine);
    let _ = graph.structural_facts();
    let refined_sign = graph.sign_until(-128);
    let warm_sign = graph.sign_until(0);
    if let Some(sign) = cold_sign {
        assert_eq!(refined_sign, Some(sign));
    }
    if let Some(sign) = refined_sign {
        assert_eq!(warm_sign, Some(sign));
    }
});

fn check_normal_and_affine_cache_contract(value: &Real) {
    let zero = Real::zero();
    let one = Real::one();
    let query = || {
        (
            value.exact_rational_normal_form(),
            Real::certified_affine_det3_sign(
                [value, &zero, &zero],
                [&zero, &one, &zero],
                [&zero, &zero, &one],
                [&zero, &zero, &zero],
            ),
        )
    };
    let cold = query();
    let [lower, upper] = value
        .certified_rational_interval(-512)
        .expect("representative has a bounded approximation");
    let warm = query();
    if let Some(normal) = cold.0 {
        assert_eq!(warm.0.as_ref(), Some(&normal));
    }
    if let Some(sign) = cold.1 {
        assert_eq!(warm.1, Some(sign));
    }
    if let Some(normal) = warm.0 {
        assert!(lower <= normal && normal <= upper);
    }
    if let Some(sign) = warm.1 {
        // This determinant equals the positive inventory representative.
        assert_eq!(sign, RealSign::Positive);
    }
}

fn assert_certificates_match_bounded_evaluation(left: &Real, right: &Real) {
    let difference = left - right;
    let [lower, upper] = difference
        .certified_rational_interval(-768)
        .expect("representative difference has a bounded approximation");
    let zero = Rational::zero();

    match left.certified_cmp_until(right, -512) {
        CertifiedRealOrdering::Known { ordering, .. } => match ordering {
            core::cmp::Ordering::Less => assert!(upper < zero),
            core::cmp::Ordering::Equal => assert!(lower <= zero && upper >= zero),
            core::cmp::Ordering::Greater => assert!(lower > zero),
        },
        CertifiedRealOrdering::Unknown { .. } => {}
    }
    match left.certified_eq_until(right, -512) {
        CertifiedRealEquality::Equal { .. } => assert!(lower <= zero && upper >= zero),
        CertifiedRealEquality::NotEqual { .. } => assert!(upper < zero || lower > zero),
        CertifiedRealEquality::Unknown { .. } => {}
    }
}

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
        .certified_rational_interval(-512)
        .expect("representative has bounded approximation");
    let [right_lower, right_upper] = right
        .certified_rational_interval(-512)
        .expect("representative has bounded approximation");
    assert!(
        left_lower <= right_upper && right_lower <= left_upper,
        "512-bit intervals must overlap for approximately equal expressions"
    );
}

fn variable_depth_graph(data: &[u8]) -> Computable {
    let numerator = i64::from(data.get(1).copied().unwrap_or(1) % 7) + 1;
    let denominator = u64::from(data.get(2).copied().unwrap_or(2) % 7) + 1;
    let mut value = Computable::rational(
        Rational::fraction(numerator, denominator).expect("positive graph denominator"),
    );

    for (depth, byte) in data.iter().copied().take(32).enumerate() {
        let small = Computable::rational(
            Rational::fraction(i64::from(byte % 5) + 1, u64::from(byte % 7) + 2)
                .expect("positive graph scale denominator"),
        );
        value = match (usize::from(byte) + depth) % 8 {
            0 => value.add(small),
            1 => value.multiply(small),
            2 => value.negate(),
            3 => value.sin().square(),
            4 => value.sin(),
            5 => value.cos(),
            6 => value.atan(),
            _ => {
                let shared = value.clone();
                value.add(shared)
            }
        };
    }
    value
}
