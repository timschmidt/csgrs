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
fn an_unproved_zero_denominator_keeps_its_typed_failure() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let denominator = &sine * &sine + &cosine * &cosine - Real::one();
    assert_eq!(denominator.exact_rational_normal_form(), None);
    assert!(matches!(
        denominator.inverse_ref(),
        Err(Problem::UnknownZero)
    ));
}
