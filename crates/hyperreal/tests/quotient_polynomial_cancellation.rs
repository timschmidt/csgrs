//! Quotient identities require evidence that every expanded divisor is nonzero.
use hyperreal::{Problem, Rational, Real, RealSign};

fn equivalent_quotients() -> (Real, Real) {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    let u = Real::from(6) - Real::from(2) * (Real::pi() / Real::from(8)).unwrap().sin();
    let v = Real::from(6)
        - Real::from(2)
            * ((Real::pi() * Real::from(3)) / Real::from(8))
                .unwrap()
                .sin();
    let expanded = Real::from(6) * (&v - &origin) - Real::from(6) * (&u - &origin);
    let reduced = Real::from(6) * (&v - &u);
    // Both denominators equal 6(v-u) < 0. The numerators agree by
    // (u+v)(u-v)=u^2-v^2; no trigonometric equality is required.
    (
        (((&u + &v) * (&u - &v)) / expanded).unwrap(),
        ((&u * &u - &v * &v) / reduced).unwrap(),
    )
}

#[test]
fn differently_expanded_quotients_have_an_exact_equality_proof() {
    let (left, right) = equivalent_quotients();
    let difference = left - right;
    assert_eq!(
        difference.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    assert_eq!(
        difference.certified_sign_until(0).sign(),
        Some(RealSign::Zero)
    );
}

#[test]
fn quotient_proofs_preserve_exact_nonzero_displacements() {
    for numerator in [-1, 1] {
        let (left, right) = equivalent_quotients();
        let displacement = Rational::fraction(numerator, 1_000_000_000_000).unwrap();
        let difference = left - right + Real::from(displacement.clone());
        assert_eq!(difference.exact_rational_normal_form(), Some(displacement));
        assert_eq!(
            difference.certified_sign_until(0).sign(),
            Some(if numerator < 0 {
                RealSign::Negative
            } else {
                RealSign::Positive
            }),
        );
    }
}

#[test]
fn a_proved_zero_quotient_composes_with_high_degree_reciprocals() {
    let (left, right) = equivalent_quotients();
    let denominator = Real::from(4) + Real::e().sin();
    let mut reciprocal = denominator.inverse_ref().unwrap();
    for _ in 0..3 {
        reciprocal = &reciprocal * &reciprocal;
    }
    let value = (left - right) * reciprocal;
    #[cfg(feature = "serde")]
    let before = serde_json::to_string(&value).unwrap();
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
    #[cfg(feature = "serde")]
    assert_eq!(serde_json::to_string(&value).unwrap(), before);
    for numerator in [-1, 1] {
        let displacement = Rational::fraction(numerator, 1_000_000_000_000).unwrap();
        assert_eq!(
            (&value + Real::from(displacement.clone())).exact_rational_normal_form(),
            Some(displacement)
        );
    }
}

#[test]
fn polynomial_factor_division_preserves_small_nonzero_denominators() {
    let a = Real::from(4) + Real::e().sin();
    let b = Real::one() + (Real::pi() / Real::from(8)).unwrap().sin();
    for scale in [
        Rational::one(),
        Rational::fraction(1, 1_000_000_000_000).unwrap(),
    ] {
        let scale = Real::from(scale);
        let denominator = (&a - &b) * &scale;
        let quotient = ((&a * &a - &b * &b) / denominator).unwrap();
        let expected = ((&a + &b) / scale).unwrap();
        // a > 3 and b < 2, so the factor a-b is strictly positive.
        assert_eq!(
            (quotient - expected).exact_rational_normal_form(),
            Some(Rational::zero())
        );
    }
}

#[test]
fn proved_and_unproved_zero_denominators_keep_typed_failures() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let denominator = &sine * &sine + &cosine * &cosine - Real::one();
    // The original denominator has a new proof. Its inverse is still invalid.
    assert_eq!(
        denominator.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    assert_eq!(denominator.inverse_ref(), Err(Problem::DivideByZero));

    // Preserve the unresolved-domain contract with an independent zero whose
    // angle-addition identity is outside the admitted exact normal form.
    let denominator = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert_eq!(denominator.exact_rational_normal_form(), None);
    assert_eq!(denominator.inverse_ref(), Err(Problem::UnknownZero));
}

#[test]
fn quotient_proofs_survive_precision_changes_and_concurrent_queries() {
    let (left, right) = equivalent_quotients();
    let difference = left - right;
    assert_eq!(
        difference.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    for precision in [0, -1100, 0] {
        assert_eq!(
            difference.certified_sign_until(precision).sign(),
            Some(RealSign::Zero)
        );
    }
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                assert_eq!(
                    difference.exact_rational_normal_form(),
                    Some(Rational::zero())
                );
                assert_eq!(
                    difference.certified_sign_until(0).sign(),
                    Some(RealSign::Zero)
                );
            });
        }
    });
}

#[test]
fn distinct_denominators_use_the_exact_common_denominator() {
    let a = Real::from(4) + Real::e().sin();
    let b = Real::from(2) + (Real::pi() / Real::from(8)).unwrap().sin();
    // a > 3 and b > 2. 1/a - 1/b = (b-a)/(ab).
    let difference =
        a.inverse_ref().unwrap() - b.inverse_ref().unwrap() - ((&b - &a) / (&a * &b)).unwrap();
    assert_eq!(
        difference.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    let nonrational = a.inverse_ref().unwrap() + b.inverse_ref().unwrap();
    assert_eq!(nonrational.exact_rational_normal_form(), None);
    assert_eq!(
        nonrational.certified_sign_until(-64).sign(),
        Some(RealSign::Positive)
    );
}

#[cfg(feature = "serde")]
#[test]
fn quotient_proof_preserves_the_authoritative_serialized_expression() {
    let (left, right) = equivalent_quotients();
    let difference = left - right;
    let before = serde_json::to_string(&difference).unwrap();
    assert_eq!(
        difference.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    assert_eq!(serde_json::to_string(&difference).unwrap(), before);
}
