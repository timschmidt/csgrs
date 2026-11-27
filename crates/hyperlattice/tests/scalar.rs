//! Scalar identity, domain, structural-knowledge, and abort-attachment regressions.
//!
//! Uses exact expected constants/ratios and typed native errors. The near-zero
//! fixture is mathematically nonzero despite unknown cheap zero knowledge. These
//! finite cases do not establish general symbolic decidability or cancellation
//! coverage; successful structural paths may ignore an active flag.

/// Shared exact literals, near-zero expressions, and cooperative abort flags.
mod common;

use common::{abort_signal, frac, r, unknown_zero};

use hyperlattice::{
    Problem, Real, RealSign, RealStructuralFacts, ZeroKnowledge, reciprocal_checked,
    reciprocal_ref_checked,
};

/// Check selected exact constants, elementary identities, an integer power, and a log shortcut after attaching an inactive abort flag.
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

/// Check sinh, cosh, and tanh of ln(2) collapse to the exact ratios 3/4, 5/4, and 3/5.
#[test]
fn native_hyperbolic_operations_preserve_exact_log_collapses() {
    let ln_two = Real::ln(r(2)).unwrap();

    assert_eq!(Real::sinh(ln_two.clone()).unwrap(), frac(3, 4));
    assert_eq!(Real::cosh(ln_two.clone()).unwrap(), frac(5, 4));
    assert_eq!(Real::tanh(ln_two).unwrap(), frac(3, 5));
}

/// Check native error variants for selected invalid inverse, root, logarithm, inverse-trigonometric, hyperbolic, power, and tangent inputs.
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

/// Check cheap zero knowledge for zero, an integer, pi, and the near-zero sine fixture, including after inactive abort attachment.
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

/// Check integer sign/zero metadata and exact sign refinement at precision -64 for zero and signed integers.
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

/// Check lossy export returns 7.0 for exact seven while retaining the input; this is not a general error-bound or round-trip test.
#[test]
fn scalar_to_f64_lossy_is_borrowed() {
    let value = r(7);
    assert_eq!(value.to_f64_lossy(), Some(7.0));
    assert_eq!(value, r(7));
}

/// Check positive sign/nonzero metadata for pi, sqrt(2), and 1/2, their rational flags, and presence of a magnitude bound for pi.
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

/// Check the near-zero sine fixture has no cheap sign/magnitude or rational certificate and remains undecided at refinement precision -1.
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

/// Check asin(1/2), acos(1/2), and atan(1) retain the expected exact rational multiples of pi.
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

/// Check owned structural reciprocal rejects zero as DivideByZero and maps four to 1/4.
#[test]
fn checked_scalar_reciprocal_rejects_zero() {
    assert_eq!(reciprocal_checked(Real::zero()), Err(Problem::DivideByZero));
    assert_eq!(reciprocal_checked(r(4)).unwrap(), frac(1, 4));
}

/// Compare native and checked borrowed reciprocals of four against 1/4, then verify the input remains four.
#[test]
fn borrowed_scalar_reciprocal_does_not_consume_input() {
    let value = r(4);
    assert_eq!(Real::inverse_ref(&value).unwrap(), frac(1, 4));
    assert_eq!(reciprocal_ref_checked(&value).unwrap(), frac(1, 4));
    assert_eq!(value, r(4));
}

/// Check owned structural reciprocal rejects the mathematically nonzero sine fixture as UnknownZero, including after inactive abort attachment.
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

/// Check asin(0) and invalid acosh(0) retain their native outcomes after inactive flag attachment; active cancellation is not tested here.
#[test]
fn native_inverse_functions_accept_abort_signal() {
    let signal = abort_signal();
    let mut zero = Real::zero();
    zero.abort(signal);
    assert_eq!(zero.clone().asin().unwrap(), Real::zero());
    assert_eq!(zero.acosh(), Err(Problem::NotANumber));
}

/// Check five scalar fixtures with inactive and active abort flags.
///
/// Zero knowledge and owned/borrowed checked reciprocal outcomes are compared with
/// the expected structural classification and native inverse. These structural/exact
/// routes do not exercise all refinement or cancellation paths.
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

/// Check invalid logarithm and asin/acos domains retain native NotANumber errors across twelve exact/symbolic fixtures with both abort-flag values.
///
/// Only domains already classified Invalid before attachment are asserted. This
/// does not require all cases to be invalid or certify unknown-domain decisions.
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

/// Check structural reciprocal returns UnknownZero without changing cheap knowledge, while native inversion can resolve the same nonzero fixture.
///
/// This contrasts the two domain contracts; it asserts outcomes and retained
/// knowledge, not dispatch counts or a bound on native refinement work.
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
