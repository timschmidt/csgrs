mod common;

use common::{abort_signal, frac, r, unknown_zero};

use hyperlattice::{
    Problem, Real, RealSign, RealStructuralFacts, ZeroKnowledge, reciprocal_checked,
    reciprocal_ref_checked,
};

#[test]
fn scalar_functions() {
    assert_eq!(Real::tau(), r(2) * Real::pi());
    assert_eq!(Real::sqrt(9.into()).unwrap(), r(3));
    assert_eq!(Real::sin(Real::pi()), Real::zero());
    assert_eq!(Real::ln(Real::e()).unwrap(), Real::one());
    assert_eq!(Real::log10(r(100)).unwrap(), r(2));
    assert_eq!(Real::powi_i64(frac(7, 5), 5).unwrap(), frac(16_807, 3_125));
    assert_eq!(Real::powi_i64(Real::pi(), -1), Real::inverse(Real::pi()));

    let signal = abort_signal();
    let mut value = r(1_000);
    value.abort(signal);
    assert_eq!(value.log10().unwrap(), r(3));
}

#[test]
fn native_hyperbolic_operations_preserve_exact_log_collapses() {
    let ln_two = Real::ln(r(2)).unwrap();

    assert_eq!(Real::sinh(ln_two.clone()).unwrap(), frac(3, 4));
    assert_eq!(Real::cosh(ln_two.clone()).unwrap(), frac(5, 4));
    assert_eq!(Real::tanh(ln_two).unwrap(), frac(3, 5));
}

#[test]
fn scalar_functions_reject_invalid_domains() {
    assert_eq!(Real::inverse(Real::zero()), Err(Problem::DivideByZero));
    assert_eq!(Real::sqrt(r(-1)), Err(Problem::SqrtNegative));
    assert_eq!(Real::ln(Real::zero()), Err(Problem::NotANumber));
    assert_eq!(Real::ln(r(-1)), Err(Problem::NotANumber));
    assert_eq!(Real::log10(Real::zero()), Err(Problem::NotANumber));
    assert_eq!(Real::log10(r(-1)), Err(Problem::NotANumber));
    assert_eq!(Real::asin(r(2)), Err(Problem::NotANumber));
    assert_eq!(Real::acos(r(2)), Err(Problem::NotANumber));
    assert_eq!(Real::acosh(Real::zero()), Err(Problem::NotANumber));
    assert_eq!(Real::atanh(Real::one()), Err(Problem::Infinity));
    assert_eq!(Real::powi_i64(Real::zero(), 0), Err(Problem::NotANumber));

    let half_pi = (Real::pi() / r(2)).unwrap();
    assert_eq!(Real::tan(half_pi), Err(Problem::NotANumber));
}

#[test]
fn zero_status_classifies_basic_values() {
    assert_eq!(Real::zero_status(&Real::zero()), ZeroKnowledge::Zero);
    assert_eq!(Real::zero_status(&r(7)), ZeroKnowledge::NonZero);
    assert_eq!(Real::zero_status(&Real::pi()), ZeroKnowledge::NonZero);
    assert_eq!(Real::zero_status(&unknown_zero()), ZeroKnowledge::Unknown);

    let signal = abort_signal();
    let mut unknown = unknown_zero();
    unknown.abort(signal);
    assert_eq!(unknown.zero_status(), ZeroKnowledge::Unknown);
}

#[test]
fn scalar_structural_facts_classify_basic_values() {
    let zero_facts = Real::zero().structural_facts();
    assert_eq!(zero_facts.sign, Some(RealSign::Zero));
    assert_eq!(zero_facts.zero, ZeroKnowledge::Zero);
    assert!(Real::zero().definitely_zero());

    let positive_facts = r(7).structural_facts();
    assert_eq!(positive_facts.sign, Some(RealSign::Positive));
    assert_eq!(positive_facts.zero, ZeroKnowledge::NonZero);
    assert!(!r(7).definitely_zero());

    let negative_facts = r(-7).structural_facts();
    assert_eq!(negative_facts.sign, Some(RealSign::Negative));
    assert_eq!(negative_facts.zero, ZeroKnowledge::NonZero);

    assert_eq!(Real::zero().refine_sign_until(-64), Some(RealSign::Zero));
    assert_eq!(r(9).refine_sign_until(-64), Some(RealSign::Positive));
    assert_eq!(r(-9).refine_sign_until(-64), Some(RealSign::Negative));
}

#[test]
fn scalar_to_f64_lossy_is_borrowed() {
    let value = r(7);
    assert_eq!(value.to_f64_lossy(), Some(7.0));
    assert_eq!(value, r(7));
}

#[test]
fn hyperreal_scalar_forwards_symbolic_structural_facts() {
    let pi_facts = Real::pi().structural_facts();
    assert_eq!(pi_facts.sign, Some(RealSign::Positive));
    assert_eq!(pi_facts.zero, ZeroKnowledge::NonZero);
    assert!(!pi_facts.exact_rational);
    assert!(pi_facts.magnitude.is_some());

    let sqrt_two = Real::sqrt(r(2)).unwrap();
    let sqrt_facts = sqrt_two.structural_facts();
    assert_eq!(sqrt_facts.sign, Some(RealSign::Positive));
    assert_eq!(sqrt_facts.zero, ZeroKnowledge::NonZero);
    assert!(!sqrt_facts.exact_rational);

    let rational_facts = frac(1, 2).structural_facts();
    assert_eq!(rational_facts.sign, Some(RealSign::Positive));
    assert_eq!(rational_facts.zero, ZeroKnowledge::NonZero);
    assert!(rational_facts.exact_rational);
}

#[test]
fn hyperreal_scalar_keeps_unknown_structural_facts_unknown() {
    assert_eq!(
        unknown_zero().structural_facts(),
        RealStructuralFacts {
            sign: None,
            zero: ZeroKnowledge::Unknown,
            exact_rational: false,
            magnitude: None,
        }
    );
    assert_eq!(unknown_zero().refine_sign_until(-1), None);
}

#[test]
fn hyperreal_inverse_trig_helpers_preserve_exact_paths() {
    assert_eq!(
        Real::asin(frac(1, 2)).unwrap(),
        (Real::pi() / r(6)).unwrap()
    );
    assert_eq!(
        Real::acos(frac(1, 2)).unwrap(),
        (Real::pi() / r(3)).unwrap()
    );
    assert_eq!(
        Real::atan(Real::one()).unwrap(),
        (Real::pi() / r(4)).unwrap()
    );
}

#[test]
fn checked_scalar_reciprocal_rejects_zero() {
    assert_eq!(reciprocal_checked(Real::zero()), Err(Problem::DivideByZero));
    assert_eq!(reciprocal_checked(r(4)).unwrap(), frac(1, 4));
}

#[test]
fn borrowed_scalar_reciprocal_does_not_consume_input() {
    let value = r(4);
    assert_eq!(Real::inverse_ref(&value).unwrap(), frac(1, 4));
    assert_eq!(reciprocal_ref_checked(&value).unwrap(), frac(1, 4));
    assert_eq!(value, r(4));
}

#[test]
fn checked_scalar_reciprocal_rejects_unknown_zero() {
    assert_eq!(
        reciprocal_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );

    let signal = abort_signal();
    let mut unknown = unknown_zero();
    unknown.abort(signal);
    assert_eq!(reciprocal_checked(unknown), Err(Problem::UnknownZero));
}

#[test]
fn native_inverse_functions_accept_abort_signal() {
    let signal = abort_signal();
    let mut zero = Real::zero();
    zero.abort(signal);
    assert_eq!(zero.clone().asin().unwrap(), Real::zero());
    assert_eq!(zero.acosh(), Err(Problem::NotANumber));
}

#[test]
fn structural_queries_and_checked_guards_do_not_depend_on_abort() {
    use std::sync::atomic::Ordering;

    for active in [false, true] {
        let signal = abort_signal();
        signal.store(active, Ordering::Relaxed);
        for (mut value, expected) in [
            (Real::zero(), ZeroKnowledge::Zero),
            (r(-7), ZeroKnowledge::NonZero),
            (Real::pi(), ZeroKnowledge::NonZero),
            (r(2).sqrt().unwrap(), ZeroKnowledge::NonZero),
            (unknown_zero(), ZeroKnowledge::Unknown),
        ] {
            assert_eq!(value.zero_status(), expected);
            value.abort(signal.clone());
            assert_eq!(value.zero_status(), expected);
            match expected {
                ZeroKnowledge::Zero => {
                    assert_eq!(reciprocal_ref_checked(&value), Err(Problem::DivideByZero));
                    assert_eq!(reciprocal_checked(value), Err(Problem::DivideByZero));
                }
                ZeroKnowledge::Unknown => {
                    assert_eq!(reciprocal_ref_checked(&value), Err(Problem::UnknownZero));
                    assert_eq!(reciprocal_checked(value), Err(Problem::UnknownZero));
                }
                ZeroKnowledge::NonZero => {
                    let expected_inverse = value.inverse_ref().unwrap();
                    assert_eq!(reciprocal_ref_checked(&value).unwrap(), expected_inverse);
                    assert_eq!(reciprocal_checked(value).unwrap(), expected_inverse);
                }
            }
        }
    }
}

#[test]
fn native_domain_errors_preserve_structural_certificates_with_active_abort() {
    use hyperlattice::DomainStatus;
    use std::sync::atomic::Ordering;

    let cases = [
        Real::zero(),
        r(-1),
        r(-2),
        r(2),
        Real::pi(),
        -Real::pi(),
        Real::e(),
        -Real::e(),
        r(2).sqrt().unwrap(),
        -r(2).sqrt().unwrap(),
        Real::pi() + Real::e(),
        -(Real::pi() + Real::e()),
    ];
    for active in [false, true] {
        let signal = abort_signal();
        signal.store(active, Ordering::Relaxed);
        for value in &cases {
            let facts = value.domain_facts();
            let mut aborted = value.clone();
            aborted.abort(signal.clone());
            if facts.log == DomainStatus::Invalid {
                assert_eq!(aborted.clone().ln(), Err(Problem::NotANumber));
                assert_eq!(aborted.clone().log10(), Err(Problem::NotANumber));
            }
            if facts.asin_acos == DomainStatus::Invalid {
                assert_eq!(aborted.clone().asin(), Err(Problem::NotANumber));
                assert_eq!(aborted.acos(), Err(Problem::NotANumber));
            }
        }
    }
}

#[test]
fn structural_checked_reciprocal_does_not_invoke_native_refinement() {
    let value = unknown_zero();
    assert_eq!(value.zero_status(), ZeroKnowledge::Unknown);
    assert_eq!(reciprocal_ref_checked(&value), Err(Problem::UnknownZero));
    assert_eq!(value.zero_status(), ZeroKnowledge::Unknown);
    assert!(
        value.inverse_ref().is_ok(),
        "native inversion may refine this nonzero value"
    );
}
