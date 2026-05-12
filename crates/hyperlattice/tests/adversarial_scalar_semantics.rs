mod common;

use common::{frac, r, unknown_zero};
use realistic_blas::{
    Problem, Scalar, ScalarFacts, ScalarSign, ZeroStatus, acos, acosh, asin, atanh, cos, e, ln,
    log10, one, pi, reciprocal_checked, reciprocal_ref_checked, sin, sqrt, tan, zero, zero_status,
};

fn assert_stable_facts(value: &Scalar) {
    let facts = value.structural_facts();

    for _ in 0..8 {
        assert_eq!(value.structural_facts(), facts);
        assert_eq!(value.zero_status(), facts.zero);
        assert_eq!(zero_status(value), facts.zero);
        assert_eq!(value.definitely_zero(), facts.zero == ZeroStatus::Zero);
        if facts.zero == ZeroStatus::Zero {
            assert_eq!(facts.sign, Some(ScalarSign::Zero));
            assert!(facts.magnitude.is_none());
        }
        if facts.zero == ZeroStatus::NonZero {
            assert_ne!(facts.sign, Some(ScalarSign::Zero));
        }
    }
}

fn assert_same_semantics(left: Scalar, right: Scalar) {
    assert_eq!(left, right);
    assert_eq!(left.zero_status(), right.zero_status());
    assert_eq!(left.structural_facts(), right.structural_facts());
    assert_eq!(left.refine_sign_until(-64), right.refine_sign_until(-64));
    assert_eq!(left.to_f64_approx(), right.to_f64_approx());
}

#[test]
fn scalar_fact_queries_survive_repeated_cache_warming() {
    let values = [
        zero(),
        one(),
        r(-7),
        frac(1, 1 << 20),
        pi(),
        e(),
        realistic_blas::tau(),
        sqrt(r(2)).unwrap(),
        pi() - r(3),
        ((pi() * e()) / e()).unwrap(),
        unknown_zero(),
    ];

    for value in values {
        assert_stable_facts(&value);
        let _ = value.to_f64_approx();
        let _ = value.refine_sign_until(-128);
        assert_stable_facts(&value);
    }
}

#[test]
fn structural_equivalents_built_by_different_histories_agree() {
    assert_same_semantics((pi() / r(2)).unwrap(), frac(1, 2) * pi());
    assert_same_semantics(sin(pi()), zero());
    assert_same_semantics(cos(pi()), r(-1));
    assert_same_semantics(ln(e()).unwrap(), one());
    assert_same_semantics(log10(r(1_000)).unwrap(), r(3));
    assert_same_semantics(
        (((pi() * e()) * sqrt(r(2)).unwrap()) / e()).unwrap(),
        pi() * sqrt(r(2)).unwrap(),
    );
    assert_same_semantics(ln(r(1024)).unwrap(), r(10) * ln(r(2)).unwrap());
}

#[test]
fn exact_special_forms_and_principal_branches_are_guarded() {
    assert_same_semantics(asin(frac(1, 2)).unwrap(), (pi() / r(6)).unwrap());
    assert_same_semantics(acos(frac(1, 2)).unwrap(), (pi() / r(3)).unwrap());
    assert_same_semantics(realistic_blas::atan(one()).unwrap(), (pi() / r(4)).unwrap());

    let seven_pi_six = (r(7) * pi() / r(6)).unwrap();
    assert_same_semantics(sin(seven_pi_six), frac(-1, 2));

    let five_pi_four = (r(5) * pi() / r(4)).unwrap();
    assert_same_semantics(
        realistic_blas::atan(tan(five_pi_four).unwrap()).unwrap(),
        (pi() / r(4)).unwrap(),
    );
}

#[test]
fn domain_boundary_errors_do_not_stale_cache_valid_neighbors() {
    assert_eq!(sqrt(r(-1)), Err(Problem::SqrtNegative));
    assert_eq!(ln(zero()), Err(Problem::NotANumber));
    assert_eq!(ln(r(-1)), Err(Problem::NotANumber));
    assert_eq!(asin(r(2)), Err(Problem::NotANumber));
    assert_eq!(acos(r(2)), Err(Problem::NotANumber));
    assert_eq!(atanh(one()), Err(Problem::Infinity));
    assert_eq!(acosh(zero()), Err(Problem::NotANumber));

    for value in [
        sqrt(zero()).unwrap(),
        sqrt(frac(1, 1_000_000)).unwrap(),
        ln(one()).unwrap(),
        atanh(frac(999_999, 1_000_000)).unwrap(),
        acosh(frac(1_000_001, 1_000_000)).unwrap(),
    ] {
        assert_stable_facts(&value);
    }
}

#[test]
fn checked_reciprocal_distinguishes_zero_nonzero_and_unknown_zero() {
    assert_eq!(reciprocal_checked(zero()), Err(Problem::DivideByZero));
    assert_eq!(reciprocal_ref_checked(&r(4)).unwrap(), frac(1, 4));
    assert_eq!(
        reciprocal_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );
}

#[test]
fn float_import_regressions_cover_zero_subnormals_decimals_and_large_values() {
    let cases = [
        0.0,
        -0.0,
        0.5,
        -0.25,
        f64::MIN_POSITIVE,
        f64::from_bits(1),
        0.1,
        0.2,
        0.3,
        1.0e-12,
        1.0e6,
        1.0e30,
    ];

    for value in cases {
        let imported = Scalar::try_from(value).unwrap();
        assert_stable_facts(&imported);
        if value == 0.0 {
            assert_eq!(imported.zero_status(), ZeroStatus::Zero);
        } else {
            assert_eq!(imported.zero_status(), ZeroStatus::NonZero);
        }
    }
}

#[test]
fn fact_api_invariants_are_self_consistent_for_adversarial_forms() {
    let cases = [
        (sqrt(r(2)).unwrap() + sqrt(r(2)).unwrap()) - r(2) * sqrt(r(2)).unwrap(),
        ((pi() * e()) / e()).unwrap() - pi(),
        pi() - frac(355, 113),
        sqrt(r(2)).unwrap() - frac(99, 70),
        r(1_000_000) * (one() + frac(1, 1_000_000)) - r(1_000_000),
    ];

    for value in cases {
        let facts = value.structural_facts();
        let expected_facts = ScalarFacts {
            sign: facts.sign,
            zero: facts.zero,
            exact_rational: facts.exact_rational,
            magnitude: facts.magnitude,
        };
        assert_eq!(facts, expected_facts);
        assert_stable_facts(&value);
    }
}
