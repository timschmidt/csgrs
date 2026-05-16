use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};
use proptest::prelude::*;

fn rational_strategy() -> impl Strategy<Value = Rational> {
    (-1_000_000_i64..=1_000_000, 1_u64..=1_000_000)
        .prop_map(|(numerator, denominator)| Rational::fraction(numerator, denominator).unwrap())
}

fn nonzero_rational_strategy() -> impl Strategy<Value = Rational> {
    rational_strategy().prop_filter("nonzero rational", |value| *value != Rational::zero())
}

fn real_strategy() -> impl Strategy<Value = Real> {
    rational_strategy().prop_map(Real::new)
}

fn finite_f64_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        Just(-0.0),
        Just(f64::from_bits(1)),
        Just(f64::MIN_POSITIVE),
        Just(0.1),
        Just(0.2),
        Just(0.3),
        (-1.0e12_f64..1.0e12).prop_filter("finite", |value| value.is_finite()),
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn rational_ring_identities_preserve_exact_real_facts(a in rational_strategy(), b in rational_strategy(), c in rational_strategy()) {
        let ar = Real::new(a.clone());
        let br = Real::new(b.clone());
        let cr = Real::new(c.clone());

        prop_assert_eq!(&ar + &br, Real::new(&a + &b));
        prop_assert_eq!(&ar - &br, Real::new(&a - &b));
        prop_assert_eq!(&ar * &br, Real::new(&a * &b));
        prop_assert_eq!(&ar * (&br + &cr), (&ar * &br) + (&ar * &cr));

        let facts = ar.structural_facts();
        prop_assert!(facts.exact_rational);
        prop_assert_eq!(facts.zero, if a == Rational::zero() { ZeroKnowledge::Zero } else { ZeroKnowledge::NonZero });
    }

    #[test]
    fn nonzero_rational_division_and_inverse_are_exact(a in nonzero_rational_strategy(), b in nonzero_rational_strategy()) {
        let ar = Real::new(a.clone());
        let br = Real::new(b.clone());

        prop_assert_eq!((&ar / &br).unwrap(), Real::new(&a / &b));
        prop_assert_eq!((ar.clone() / ar.clone()).unwrap(), Real::one());
        prop_assert_eq!(ar.clone().inverse().unwrap() * ar, Real::one());
    }

    #[test]
    fn square_root_of_exact_square_preserves_principal_sign(n in 0_i64..=1_000_000) {
        let square = Real::new(Rational::new(n)) * Real::new(Rational::new(n));
        let root = square.sqrt().unwrap();

        prop_assert_eq!(root.clone(), Real::new(Rational::new(n)));
        prop_assert_eq!(root.structural_facts().sign, Some(if n == 0 { RealSign::Zero } else { RealSign::Positive }));
    }

    #[test]
    fn float_import_exactness_has_stable_zero_sign_and_roundtrip(value in finite_f64_strategy()) {
        let imported = Real::try_from(value).unwrap();
        let facts = imported.structural_facts();

        prop_assert!(facts.exact_rational);
        if value == 0.0 {
            prop_assert_eq!(facts.zero, ZeroKnowledge::Zero);
            prop_assert_eq!(facts.sign, Some(RealSign::Zero));
        } else if value > 0.0 {
            prop_assert_eq!(facts.sign, Some(RealSign::Positive));
        } else {
            prop_assert_eq!(facts.sign, Some(RealSign::Negative));
        }
        prop_assert_eq!(imported.structural_facts(), facts);
    }

    #[test]
    fn rational_serde_roundtrip_preserves_generated_exact_values(value in rational_strategy()) {
        let real = Real::new(value);
        let json_roundtrip = Real::from_json(&real.to_json()).unwrap();
        let bytes = real.to_bytes();
        let bytes_roundtrip = Real::from_bytes(&bytes).unwrap();

        prop_assert_eq!(json_roundtrip.clone(), real.clone());
        prop_assert_eq!(bytes_roundtrip.clone(), real.clone());
        prop_assert_eq!(json_roundtrip.structural_facts(), real.structural_facts());
        prop_assert_eq!(bytes_roundtrip.zero_status(), real.zero_status());
    }

    #[test]
    fn signed_product_sum_matches_expanded_3x3_determinant(
        a0 in rational_strategy(), a1 in rational_strategy(), a2 in rational_strategy(),
        b0 in rational_strategy(), b1 in rational_strategy(), b2 in rational_strategy(),
        c0 in rational_strategy(), c1 in rational_strategy(), c2 in rational_strategy(),
    ) {
        let a = [Real::new(a0), Real::new(a1), Real::new(a2)];
        let b = [Real::new(b0), Real::new(b1), Real::new(b2)];
        let c = [Real::new(c0), Real::new(c1), Real::new(c2)];

        let fused = Real::signed_product_sum(
            [true, false, false, true, true, false],
            [
                [&a[0], &b[1], &c[2]],
                [&a[0], &b[2], &c[1]],
                [&a[1], &b[0], &c[2]],
                [&a[1], &b[2], &c[0]],
                [&a[2], &b[0], &c[1]],
                [&a[2], &b[1], &c[0]],
            ],
        );
        let expanded = &(&a[0] * &b[1] * &c[2])
            - &(&a[0] * &b[2] * &c[1])
            - &(&a[1] * &b[0] * &c[2])
            + &(&a[1] * &b[2] * &c[0])
            + &(&a[2] * &b[0] * &c[1])
            - &(&a[2] * &b[1] * &c[0]);

        prop_assert_eq!(fused, expanded);
    }

    #[test]
    fn cache_warming_does_not_change_refinement_answers(a in real_strategy(), b in real_strategy()) {
        let value = (a.clone() * Real::pi()) - (b.clone() * Real::pi());
        let facts = value.structural_facts();
        let low = value.refine_sign_until(-32);
        let high = value.refine_sign_until(-160);

        prop_assert_eq!(value.structural_facts(), facts);
        prop_assert_eq!(value.refine_sign_until(-32), low);
        prop_assert_eq!(value.refine_sign_until(-160), high);
    }
}
