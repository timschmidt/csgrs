use hyperreal::{Problem, Rational, Real, RealSign, ZeroKnowledge};

fn q(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

fn r(numerator: i64, denominator: u64) -> Real {
    Real::new(q(numerator, denominator))
}

fn assert_zero(value: Real) {
    assert_eq!(value, Real::zero());
    assert_eq!(value.zero_status(), ZeroKnowledge::Zero);
    assert_eq!(value.structural_facts().sign, Some(RealSign::Zero));
}

#[test]
fn cancellation_chains_collapse_to_exact_zero_without_forcing_approximation() {
    let sqrt2 = r(2, 1).sqrt().unwrap();
    assert_zero((sqrt2.clone() + sqrt2.clone()) - Real::new(Rational::new(2)) * sqrt2);

    let pi = Real::pi();
    let e = Real::e();
    assert_zero(((pi.clone() * e.clone()) / e).unwrap() - pi);

    let log_chain = r(1024, 1).ln().unwrap() - Real::new(Rational::new(10)) * r(2, 1).ln().unwrap();
    assert_zero(log_chain);
}

#[test]
fn inverse_inverse_and_division_identities_preserve_exact_rationals() {
    for value in [q(1, 3), q(-7, 11), q(1 << 20, 3), q(-99, 70)] {
        let real = Real::new(value);
        let inverse = real.clone().inverse().unwrap();

        assert_eq!(inverse.clone().inverse().unwrap(), real);
        assert_eq!((real.clone() / real.clone()).unwrap(), Real::one());
        assert_eq!(real.clone() * inverse, Real::one());
    }
}

#[test]
fn exact_trig_special_forms_and_neighbors_are_distinguished() {
    assert_eq!(Real::pi().sin(), Real::zero());
    assert_eq!(
        (Real::pi() / Real::new(Rational::new(6))).unwrap().sin(),
        r(1, 2)
    );
    assert_eq!(
        (Real::pi() / Real::new(Rational::new(3))).unwrap().cos(),
        r(1, 2)
    );
    assert_eq!(
        (Real::pi() / Real::new(Rational::new(4)))
            .unwrap()
            .tan()
            .unwrap(),
        Real::one()
    );

    let neighbor =
        ((Real::pi() / Real::new(Rational::new(6))).unwrap()) + Real::new(q(1, 1_000_000));
    assert_ne!(neighbor.sin(), r(1, 2));
}

#[test]
fn inverse_trig_domain_edges_are_exact_and_outside_edges_fail() {
    let half_pi = (Real::pi() / Real::new(Rational::new(2))).unwrap();

    assert_eq!(Real::one().asin().unwrap(), half_pi);
    assert_eq!((-Real::one()).asin().unwrap(), -half_pi);
    assert_eq!(Real::one().acos().unwrap(), Real::zero());
    assert_eq!((-Real::one()).acos().unwrap(), Real::pi());
    assert_eq!(r(1_000_001, 1_000_000).asin(), Err(Problem::NotANumber));
    assert_eq!(r(-1_000_001, 1_000_000).acos(), Err(Problem::NotANumber));
}

#[cfg(feature = "serde")]
#[test]
fn serde_roundtrip_preserves_structural_facts_and_special_forms() {
    let cases = [
        Real::zero(),
        Real::one(),
        Real::pi(),
        Real::e(),
        Real::tau(),
        r(355, 113),
        r(2, 1).sqrt().unwrap(),
        r(17, 1).root_n(5).unwrap(),
        r(1024, 1).ln().unwrap(),
        (Real::pi() / Real::new(Rational::new(6))).unwrap().sin(),
    ];

    for value in cases {
        let json = value.to_json();
        let decoded = Real::from_json(&json).unwrap();
        assert_eq!(decoded, value);
        assert_eq!(decoded.structural_facts(), value.structural_facts());

        let bytes = value.to_bytes();
        let decoded = Real::from_bytes(&bytes).unwrap();
        assert_eq!(decoded, value);
        assert_eq!(decoded.zero_status(), value.zero_status());
    }
}

#[cfg(feature = "serde")]
#[test]
fn serde_rejects_forged_direct_nth_root_degrees() {
    fn replace_degree(value: &mut serde_json::Value, replacement: u32) -> bool {
        match value {
            serde_json::Value::Object(fields) => {
                if let Some(serde_json::Value::Array(arguments)) = fields.get_mut("NthRoot")
                    && let Some(degree) = arguments.get_mut(1)
                {
                    *degree = serde_json::Value::from(replacement);
                    return true;
                }
                fields
                    .values_mut()
                    .any(|child| replace_degree(child, replacement))
            }
            serde_json::Value::Array(values) => values
                .iter_mut()
                .any(|child| replace_degree(child, replacement)),
            _ => false,
        }
    }

    let root = r(17, 1).root_n(5).unwrap();
    for invalid_degree in [0_u32, 1, 2, 10, u32::MAX] {
        let mut json: serde_json::Value = serde_json::from_str(&root.to_json()).unwrap();
        assert!(replace_degree(&mut json, invalid_degree));
        assert_eq!(
            Real::from_json(&json.to_string()),
            Err(Problem::ParseError),
            "degree {invalid_degree} should be rejected"
        );
    }
}

#[test]
fn compare_and_equality_do_not_conflate_nearby_cancellation_values() {
    let pi_error = Real::pi() - r(355, 113);
    let sqrt_error = r(2, 1).sqrt().unwrap() - r(99, 70);

    assert_ne!(pi_error, Real::zero());
    assert_ne!(sqrt_error, Real::zero());
    assert_ne!(pi_error.to_f64_lossy(), Some(0.0));
    assert_ne!(sqrt_error.to_f64_lossy(), Some(0.0));
}

#[test]
fn rump_polynomial_retains_exact_large_integer_cancellation() {
    let a = r(77_617, 1);
    let b = r(33_096, 1);
    let aa = &a * &a;
    let bb = &b * &b;

    let integer_part = Real::from(21) * &bb - Real::from(2) * &aa + Real::from(55) * (&bb * &bb)
        - Real::from(10) * (&aa * &bb);
    assert_eq!(integer_part, Real::from(-2));

    let quotient = (&a / &(Real::from(2) * &b)).unwrap();
    let value = integer_part + quotient;
    assert_eq!(value, r(-54_767, 66_192));
    assert!(value.exact_rational().is_some());
}

#[test]
fn scheinerman_radical_sums_receive_the_correct_certified_order() {
    let root = |value| Real::from(value).sqrt().expect("positive integer radicand");
    let left = root(75_025) + root(121_393) + root(196_418) + root(317_811);
    let right = root(514_229) + root(832_040);

    assert_eq!(
        (left - right).certified_sign_until(-32).sign(),
        Some(RealSign::Positive)
    );
}

#[test]
fn archived_nested_radical_identities_receive_exact_zero_certificates() {
    let root = |numerator: i64, denominator: u64, degree: u32| {
        r(numerator, denominator)
            .root_n(degree)
            .expect("positive rational radicand")
    };

    let ramanujan_one = Real::from(3)
        * (root(5, 1, 3) - root(4, 1, 3))
            .sqrt()
            .expect("positive nested radicand")
        - (root(2, 1, 3) + root(20, 1, 3) - root(25, 1, 3));
    let ramanujan_two = (root(2, 1, 3) - Real::one())
        .root_n(3)
        .expect("positive nested radicand")
        - (root(1, 9, 3) - root(2, 9, 3) + root(4, 9, 3));
    let fifth2 = root(2, 1, 5);
    let many_digits_c10 = (Real::from(7) + fifth2.clone() - Real::from(5) * root(8, 1, 5))
        .root_n(3)
        .expect("positive nested radicand")
        + root(4, 1, 5)
        - fifth2
        - Real::one();

    for (name, value) in [
        ("Ramanujan one", ramanujan_one),
        ("Ramanujan two", ramanujan_two),
        ("Many Digits C10", many_digits_c10),
    ] {
        let tiny_positive = value.clone()
            + Real::new(
                Rational::from_bigint_fraction(
                    num::BigInt::from(1_u8),
                    num::BigUint::from(1_u8) << 512_usize,
                )
                .unwrap(),
            );
        assert_eq!(
            tiny_positive.certified_sign_until(-2_048).sign(),
            Some(RealSign::Positive),
            "{name} control must not be conflated with zero"
        );
        assert_eq!(
            value.certified_sign_until(0).sign(),
            None,
            "{name} should not be accepted by a coarse approximation"
        );
        assert_eq!(
            value.certified_sign_until(-2_048).sign(),
            Some(RealSign::Zero),
            "{name} should receive a bounded exact-zero proof"
        );
    }
}
