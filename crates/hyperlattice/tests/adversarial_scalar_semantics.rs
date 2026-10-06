mod common;

use common::{frac, r, unknown_zero};
use hyperlattice::{
    Problem, Real, RealSign, RealStructuralFacts, ZeroKnowledge, reciprocal_checked,
    reciprocal_ref_checked,
};

fn assert_stable_facts(value: &Real) {
    let facts = value.structural_facts();

    for _ in 0..8 {
        assert_eq!(value.structural_facts(), facts);
        assert_eq!(value.zero_status(), facts.zero);
        assert_eq!(value.definitely_zero(), facts.zero == ZeroKnowledge::Zero);
        if facts.zero == ZeroKnowledge::Zero {
            assert_eq!(facts.sign, Some(RealSign::Zero));
            assert!(facts.magnitude.is_none());
        }
        if facts.zero == ZeroKnowledge::NonZero {
            assert_ne!(facts.sign, Some(RealSign::Zero));
        }
    }
}

fn assert_same_semantics(left: Real, right: Real) {
    assert_eq!(left, right);
    assert_eq!(left.zero_status(), right.zero_status());
    assert_eq!(left.structural_facts(), right.structural_facts());
    assert_eq!(left.refine_sign_until(-64), right.refine_sign_until(-64));
    assert_eq!(left.to_f64_lossy(), right.to_f64_lossy());
}

#[test]
fn scalar_fact_queries_survive_repeated_cache_warming() {
    let values = [
        Real::zero(),
        Real::one(),
        r(-7),
        frac(1, 1 << 20),
        Real::pi(),
        Real::e(),
        Real::tau(),
        Real::sqrt(r(2)).unwrap(),
        Real::pi() - r(3),
        ((Real::pi() * Real::e()) / Real::e()).unwrap(),
        unknown_zero(),
    ];

    for value in values {
        assert_stable_facts(&value);
        let _ = value.to_f64_lossy();
        let _ = value.refine_sign_until(-128);
        assert_stable_facts(&value);
    }
}

#[test]
fn structural_equivalents_built_by_different_histories_agree() {
    assert_same_semantics((Real::pi() / r(2)).unwrap(), frac(1, 2) * Real::pi());
    assert_same_semantics(Real::sin(Real::pi()), Real::zero());
    assert_same_semantics(Real::cos(Real::pi()), r(-1));
    assert_same_semantics(Real::ln(Real::e()).unwrap(), Real::one());
    assert_same_semantics(Real::log10(r(1_000)).unwrap(), r(3));
    assert_same_semantics(
        (((Real::pi() * Real::e()) * Real::sqrt(r(2)).unwrap()) / Real::e()).unwrap(),
        Real::pi() * Real::sqrt(r(2)).unwrap(),
    );
    assert_same_semantics(Real::ln(r(1024)).unwrap(), r(10) * Real::ln(r(2)).unwrap());
}

#[test]
fn exact_special_forms_and_principal_branches_are_guarded() {
    assert_same_semantics(
        Real::asin(frac(1, 2)).unwrap(),
        (Real::pi() / r(6)).unwrap(),
    );
    assert_same_semantics(
        Real::acos(frac(1, 2)).unwrap(),
        (Real::pi() / r(3)).unwrap(),
    );
    assert_same_semantics(
        Real::atan(Real::one()).unwrap(),
        (Real::pi() / r(4)).unwrap(),
    );

    let seven_pi_six = (r(7) * Real::pi() / r(6)).unwrap();
    assert_same_semantics(Real::sin(seven_pi_six), frac(-1, 2));

    let five_pi_four = (r(5) * Real::pi() / r(4)).unwrap();
    assert_same_semantics(
        Real::atan(Real::tan(five_pi_four).unwrap()).unwrap(),
        (Real::pi() / r(4)).unwrap(),
    );
}

#[test]
fn domain_boundary_errors_do_not_stale_cache_valid_neighbors() {
    assert_eq!(Real::sqrt(r(-1)), Err(Problem::SqrtNegative));
    assert_eq!(Real::ln(Real::zero()), Err(Problem::NotANumber));
    assert_eq!(Real::ln(r(-1)), Err(Problem::NotANumber));
    assert_eq!(Real::asin(r(2)), Err(Problem::NotANumber));
    assert_eq!(Real::acos(r(2)), Err(Problem::NotANumber));
    assert_eq!(Real::atanh(Real::one()), Err(Problem::Infinity));
    assert_eq!(Real::acosh(Real::zero()), Err(Problem::NotANumber));

    for value in [
        Real::sqrt(Real::zero()).unwrap(),
        Real::sqrt(frac(1, 1_000_000)).unwrap(),
        Real::ln(Real::one()).unwrap(),
        Real::atanh(frac(999_999, 1_000_000)).unwrap(),
        Real::acosh(frac(1_000_001, 1_000_000)).unwrap(),
    ] {
        assert_stable_facts(&value);
    }
}

#[test]
fn checked_reciprocal_distinguishes_zero_nonzero_and_unknown_zero() {
    assert_eq!(reciprocal_checked(Real::zero()), Err(Problem::DivideByZero));
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
        let imported = Real::try_from(value).unwrap();
        assert_stable_facts(&imported);
        if value == 0.0 {
            assert_eq!(imported.zero_status(), ZeroKnowledge::Zero);
        } else {
            assert_eq!(imported.zero_status(), ZeroKnowledge::NonZero);
        }
    }
}

#[test]
fn fact_api_invariants_are_self_consistent_for_adversarial_forms() {
    let cases = [
        (Real::sqrt(r(2)).unwrap() + Real::sqrt(r(2)).unwrap()) - r(2) * Real::sqrt(r(2)).unwrap(),
        ((Real::pi() * Real::e()) / Real::e()).unwrap() - Real::pi(),
        Real::pi() - frac(355, 113),
        Real::sqrt(r(2)).unwrap() - frac(99, 70),
        r(1_000_000) * (Real::one() + frac(1, 1_000_000)) - r(1_000_000),
    ];

    for value in cases {
        let facts = value.structural_facts();
        let expected_facts = RealStructuralFacts {
            sign: facts.sign,
            zero: facts.zero,
            exact_rational: facts.exact_rational,
            magnitude: facts.magnitude,
        };
        assert_eq!(facts, expected_facts);
        assert_stable_facts(&value);
    }
}
