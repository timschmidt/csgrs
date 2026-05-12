use hyperreal::{Problem, Rational, Real, RealSign, ZeroKnowledge};

fn q(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

fn r(numerator: i64, denominator: u64) -> Real {
    Real::new(q(numerator, denominator))
}

fn assert_stable_facts(value: &Real) {
    let facts = value.structural_facts();

    for _ in 0..8 {
        assert_eq!(value.structural_facts(), facts);
        assert_eq!(value.zero_status(), facts.zero);
        assert_eq!(value.definitely_zero(), facts.zero == ZeroKnowledge::Zero);
        if facts.exact_rational {
            assert!(value.exact_rational().is_some());
        }
        if facts.zero == ZeroKnowledge::Zero {
            assert_eq!(facts.sign, Some(RealSign::Zero));
            assert!(facts.magnitude.is_none());
        }
        if facts.zero == ZeroKnowledge::NonZero {
            assert_ne!(facts.sign, Some(RealSign::Zero));
        }
    }
}

fn assert_same_semantics(left: &Real, right: &Real) {
    assert_eq!(left, right);
    assert_eq!(left.structural_facts(), right.structural_facts());
    assert_eq!(left.zero_status(), right.zero_status());
    assert_eq!(left.refine_sign_until(-64), right.refine_sign_until(-64));
    assert_eq!(left.to_f64_approx(), right.to_f64_approx());
}

#[test]
fn scalar_fact_queries_are_stable_across_repeated_and_warmed_access() {
    let values = [
        Real::zero(),
        Real::one(),
        Real::new(Rational::new(-7)),
        r(1, 1 << 20),
        Real::pi(),
        Real::e(),
        Real::tau(),
        r(2, 1).sqrt().unwrap(),
        Real::pi() - Real::new(Rational::new(3)),
        ((Real::pi() * Real::e() * r(2, 1).sqrt().unwrap()) / Real::e()).unwrap(),
    ];

    for value in values {
        assert_stable_facts(&value);
        let _ = value.to_f64_approx();
        let _ = value.refine_sign_until(-128);
        assert_stable_facts(&value);
    }
}

#[test]
fn equivalent_structural_forms_keep_same_public_semantics() {
    let pi_over_two = Real::pi() / Real::new(Rational::new(2));
    let half_pi = r(1, 2) * Real::pi();
    assert_same_semantics(&pi_over_two.unwrap(), &half_pi);

    assert_same_semantics(&Real::pi().sin(), &Real::zero());
    assert_same_semantics(&Real::e().ln().unwrap(), &Real::one());
    assert_same_semantics(
        &Real::new(Rational::new(1024)).ln().unwrap(),
        &(Real::new(Rational::new(10)) * Real::new(Rational::new(2)).ln().unwrap()),
    );
    assert_same_semantics(
        &((Real::pi() * Real::e() * r(2, 1).sqrt().unwrap()) / Real::e()).unwrap(),
        &(Real::pi() * r(2, 1).sqrt().unwrap()),
    );
}

#[test]
fn domain_boundaries_do_not_poison_later_valid_queries() {
    assert_eq!(
        Real::new(Rational::new(-1)).sqrt(),
        Err(Problem::SqrtNegative)
    );
    assert_eq!(Real::zero().ln(), Err(Problem::NotANumber));
    assert_eq!(Real::new(Rational::new(-1)).ln(), Err(Problem::NotANumber));
    assert_eq!(Real::new(Rational::new(2)).asin(), Err(Problem::NotANumber));
    assert_eq!(Real::new(Rational::new(2)).acos(), Err(Problem::NotANumber));
    assert_eq!(Real::one().atanh(), Err(Problem::Infinity));
    assert_eq!(
        Real::new(Rational::new(-1)).acosh(),
        Err(Problem::NotANumber)
    );

    assert_stable_facts(&Real::zero().sqrt().unwrap());
    assert_stable_facts(&r(1, 1_000_000).sqrt().unwrap());
    assert_stable_facts(&Real::one().ln().unwrap());
    assert_stable_facts(&r(999_999, 1_000_000).atanh().unwrap());
    assert_stable_facts(&r(1_000_001, 1_000_000).acosh().unwrap());
}

#[test]
fn float_import_preserves_dyadic_exactness_and_zero_facts() {
    let cases = [
        0.0,
        -0.0,
        0.5,
        -0.25,
        f64::MIN_POSITIVE,
        f64::from_bits(1),
        1.0e-12,
        1.0e6,
        0.1,
        0.2,
        0.3,
    ];

    for value in cases {
        let imported = Real::try_from(value).unwrap();
        assert_stable_facts(&imported);
        assert!(imported.exact_rational().is_some());
        if value == 0.0 {
            assert_eq!(imported.zero_status(), ZeroKnowledge::Zero);
        } else {
            assert_eq!(imported.zero_status(), ZeroKnowledge::NonZero);
        }
    }
}

#[test]
fn refinement_order_is_cache_coherent_for_cancellation_adversaries() {
    let values = [
        Real::pi() - r(355, 113),
        r(2, 1).sqrt().unwrap() - r(99, 70),
        ((Real::pi() * Real::e()) / Real::e()).unwrap() - Real::pi(),
        (r(2, 1).sqrt().unwrap() + r(2, 1).sqrt().unwrap())
            - Real::new(Rational::new(2)) * r(2, 1).sqrt().unwrap(),
    ];

    for value in values {
        let cold = value.structural_facts();
        let signs = [
            value.refine_sign_until(-32),
            value.refine_sign_until(-256),
            value.refine_sign_until(-64),
            value.refine_sign_until(-128),
        ];
        let warmed = value.structural_facts();
        if cold.zero == ZeroKnowledge::Zero {
            assert_eq!(warmed.zero, ZeroKnowledge::Zero);
        }
        if let Some(sign) = cold.sign {
            assert_eq!(warmed.sign, Some(sign));
        }
        assert_eq!(signs[0], signs[2]);
        if signs[1].is_some() {
            assert_eq!(signs[1], signs[3]);
        }
    }
}
