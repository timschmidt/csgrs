//! Exhaustive cross-representation fuzzing for every public `Real` structural kind.

#![no_main]

use hyperreal::{
    CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, Computable, Rational, Real,
    RealSign, StructuralKind,
};
use libfuzzer_sys::fuzz_target;

const EXPECTED_KINDS: [StructuralKind; 20] = [
    StructuralKind::ExactRational,
    StructuralKind::PiLike,
    StructuralKind::PiLike,
    StructuralKind::PiLike,
    StructuralKind::ExpLike,
    StructuralKind::ExpLike,
    StructuralKind::SqrtLike,
    StructuralKind::ProductConstant,
    StructuralKind::ProductConstant,
    StructuralKind::ProductConstant,
    StructuralKind::SqrtLike,
    StructuralKind::ExpLike,
    StructuralKind::LogLike,
    StructuralKind::LogLike,
    StructuralKind::LogLike,
    StructuralKind::LogLike,
    StructuralKind::LogLike,
    StructuralKind::TrigExact,
    StructuralKind::TrigExact,
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

    // Rotate all twenty left-hand certificates across a fuzzer-selected right
    // stride. A campaign covers the full ordered 20x20 dispatch matrix without
    // forcing every individual execution to perform 400 high-precision pairs.
    let stride = usize::from(data.first().copied().unwrap_or(0)) % values.len();
    for (index, left) in values.iter().enumerate() {
        let right = &values[(index + stride) % values.len()];
        assert_bounded_equal(&(left + right), &(right + left));
        assert_bounded_equal(&(left * right), &(right * left));
        assert_bounded_equal(&(left - right), &-(right - left));

        let quotient = (left / right).expect("representatives are nonzero");
        assert!(quotient.certified_dyadic_interval(-512).is_some());

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
    let coarse = graph.approx(-32);
    assert_eq!(graph.approx(-32), coarse);
    let fine = graph.approx(-96);
    assert_eq!(graph.approx(-96), fine);
    let _ = graph.structural_facts();
    let _ = graph.sign_until(-128);
});

fn assert_certificates_match_bounded_evaluation(left: &Real, right: &Real) {
    let difference = left - right;
    let [lower, upper] = difference
        .certified_dyadic_interval(-768)
        .expect("representative difference has a bounded approximation");
    let zero = Rational::zero();

    match left.certified_cmp_until(right, -512) {
        CertifiedRealOrdering::Known { ordering, .. } => match ordering {
            core::cmp::Ordering::Less => assert!(&upper < &zero),
            core::cmp::Ordering::Equal => assert!(&lower <= &zero && &upper >= &zero),
            core::cmp::Ordering::Greater => assert!(&lower > &zero),
        },
        CertifiedRealOrdering::Unknown { .. } => {}
    }
    match left.certified_eq_until(right, -512) {
        CertifiedRealEquality::Equal { .. } => {
            assert!(&lower <= &zero && &upper >= &zero)
        }
        CertifiedRealEquality::NotEqual { .. } => {
            assert!(&upper < &zero || &lower > &zero)
        }
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
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");
    vec![
        Real::new(Rational::fraction(3, 2).expect("nonzero denominator")),
        pi.clone(),
        pi_squared.clone(),
        pi.clone().inverse().expect("pi is nonzero"),
        &pi * &e,
        (&e / &pi).expect("pi is nonzero"),
        &pi * &sqrt_two,
        &pi_squared * &e,
        &pi - Real::from(3),
        &(&pi_squared * &e) * &sqrt_two,
        sqrt_two,
        Real::from(2).exp().expect("finite exponential"),
        ln_three.clone(),
        (Real::from(2) * &e)
            .ln()
            .expect("positive logarithm input"),
        &ln_two * &ln_three,
        Real::from(2).log10().expect("positive logarithm input"),
        Real::from(3).log2().expect("positive logarithm input"),
        Real::new(Rational::fraction(1, 5).expect("nonzero denominator")).sin_pi(),
        Real::new(Rational::fraction(1, 5).expect("nonzero denominator"))
            .tan_pi()
            .expect("one fifth of a turn is not a tangent pole"),
        Real::new(Rational::one()).sin(),
    ]
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
