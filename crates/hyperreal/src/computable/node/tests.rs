#[cfg(test)]
mod tests {
    use super::*;
    use num::Signed;
    use num::bigint::BigUint;
    use std::mem::size_of;

    #[test]
    fn compare() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let six = Computable::integer(six.clone());
        let five = Computable::integer(five.clone());
        let four = Computable::integer(four.clone());

        assert_eq!(six.try_compare_to(&five), Some(Ordering::Greater));
        assert_eq!(five.try_compare_to(&six), Some(Ordering::Less));
        assert_eq!(four.try_compare_to(&six), Some(Ordering::Less));
    }

    #[test]
    fn bigger() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let a = Computable::integer(six.clone());
        let b = Computable::integer(five.clone());
        assert_eq!(a.compare_absolute(&b, 0), Ordering::Greater);
        let c = Computable::integer(four.clone());
        assert_eq!(c.compare_absolute(&a, 0), Ordering::Less);
        assert_eq!(b.compare_absolute(&b, 0), Ordering::Equal);
    }

    #[test]
    fn shifted() {
        let one = BigInt::one();
        let two = &one + &one;
        assert_eq!(one, shift(two, -1));
    }

    #[test]
    fn prec() {
        let nine: BigInt = "9".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let two: BigInt = "2".parse().unwrap();
        let one = BigInt::one();
        let a = Computable::integer(nine.clone());
        assert_eq!(nine, a.approx(0));
        assert_eq!(five, a.approx(1));
        assert_eq!(two, a.approx(2));
        assert_eq!(one, a.approx(3));
        assert_eq!(Cache::Valid((0, nine)), a.cache.into_inner());
    }

    #[test]
    fn prec_pi() {
        let three: BigInt = "3".parse().unwrap();
        let six: BigInt = "6".parse().unwrap();
        let thirteen: BigInt = "13".parse().unwrap();
        let four_zero_two: BigInt = "402".parse().unwrap();
        let a = Computable::pi();
        assert_eq!(four_zero_two, a.approx(-7));
        assert_eq!(three, a.approx(0));
        assert_eq!(six, a.approx(-1));
        assert_eq!(thirteen, a.approx(-2));
        assert_eq!(Some((-7, four_zero_two)), a.cached());
    }

    #[test]
    fn rational_zero_and_one_use_dedicated_nodes() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());

        // These identities are pervasive in higher-level constructors. Keep
        // them on the dedicated nodes so structural facts are available without
        // forcing the generic Ratio approximation path.
        assert!(matches!(*zero.internal, Approximation::Int(ref value) if value.is_zero()));
        assert!(matches!(*one.internal, Approximation::One));
        assert_eq!(zero.zero_status(), ZeroKnowledge::Zero);
        assert_eq!(one.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert_eq!(one.exact_sign(), Some(Sign::Plus));
    }

    #[test]
    fn layout_sizes() {
        assert!(
            size_of::<Computable>() <= 80,
            "Computable grew to {} bytes",
            size_of::<Computable>()
        );
        assert!(
            size_of::<Approximation>() <= 168,
            "Approximation grew to {} bytes",
            size_of::<Approximation>()
        );
        assert!(
            size_of::<Cache>() <= 40,
            "Cache grew to {} bytes",
            size_of::<Cache>()
        );
        assert!(
            size_of::<BoundCache>() <= 12,
            "BoundCache grew to {} bytes",
            size_of::<BoundCache>()
        );
        assert!(
            size_of::<ExactSignCache>() <= 1,
            "ExactSignCache grew to {} bytes",
            size_of::<ExactSignCache>()
        );
    }

    #[test]
    fn prec_atan_5() {
        let five: BigInt = "5".parse().unwrap();
        let atan_5 = Computable::prescaled_atan(five);
        let two_zero_two: BigInt = "202".parse().unwrap();
        assert_eq!(two_zero_two, atan_5.approx(-10));
        let at_twenty: BigInt = "206984".parse().unwrap();
        assert_eq!(at_twenty, atan_5.approx(-20));
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::prescaled_atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
        let at_twenty: BigInt = "4387".parse().unwrap();
        assert_eq!(at_twenty, atan_239.approx(-20));
    }

    #[test]
    fn msd() {
        let one: BigInt = "1".parse().unwrap();
        let a = Computable::integer(one.clone());
        assert_eq!(Some(0), a.msd(-4));
        let three: BigInt = "3".parse().unwrap();
        let d = Computable::integer(three.clone());
        assert_eq!(Some(1), d.msd(-4));
        let five: BigInt = "5".parse().unwrap();
        let e = Computable::integer(five.clone());
        assert_eq!(Some(2), e.msd(-4));
        let seven: BigInt = "7".parse().unwrap();
        let f = Computable::integer(seven.clone());
        assert_eq!(Some(2), f.msd(-4));
        let eight: BigInt = "8".parse().unwrap();
        let g = Computable::integer(eight.clone());
        assert_eq!(Some(3), g.msd(-4));
    }

    #[test]
    fn iter_msd() {
        let one = Computable::one();
        assert_eq!(one.iter_msd(), 0);
        let pi = Computable::pi();
        assert_eq!(pi.iter_msd(), 1);
        let five = Rational::new(5);
        let e = Computable::exp_rational(five);
        assert_eq!(e.iter_msd(), 7);
    }

    #[test]
    fn e_constant_cache_is_shared() {
        let e = Computable::e_constant();
        assert!(e.cached().is_none());
        let _ = e.approx(-32);

        let cached = Computable::e_constant()
            .cached()
            .expect("e cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn exp_one_uses_dedicated_e_constant() {
        let e = Computable::rational(Rational::one()).exp();
        assert!(matches!(
            &*e.internal,
            Approximation::Constant(SharedConstant::E)
        ));
    }

    #[test]
    fn pi_cache_is_shared() {
        let pi = Computable::pi();
        assert!(pi.cached().is_none());
        let _ = pi.approx(-32);

        let cached = Computable::pi()
            .cached()
            .expect("pi cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn tau_cache_is_shared() {
        let tau = Computable::tau();
        assert!(tau.cached().is_none());
        let _ = tau.approx(-32);

        let cached = Computable::tau()
            .cached()
            .expect("tau cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn tau_cache_reuses_warmed_pi_cache() {
        std::thread::spawn(|| {
            let pi = Computable::pi();
            let _ = pi.approx(-64);
            assert!(Computable::tau().cached().is_none());

            let tau_appr = Computable::tau().approx(-32);
            let pi_scaled_as_tau = Computable::pi().approx(-33);
            assert_eq!(tau_appr, pi_scaled_as_tau);

            let cached = Computable::tau()
                .cached()
                .expect("tau cache should be filled from pi cache");
            assert_eq!(cached.0, -32);
            assert_eq!(cached.1, tau_appr);
        })
        .join()
        .expect("tau cache test thread should finish");
    }

    #[test]
    fn pi_cache_reuses_warmed_tau_cache() {
        std::thread::spawn(|| {
            let tau = Computable::tau();
            let _ = tau.approx(-65);
            assert!(Computable::pi().cached().is_none());

            let pi_appr = Computable::pi().approx(-64);
            let tau_scaled_as_pi = Computable::tau().approx(-63);
            assert_eq!(pi_appr, tau_scaled_as_pi);

            let cached = Computable::pi()
                .cached()
                .expect("pi cache should be filled from tau cache");
            assert_eq!(cached.0, -64);
            assert_eq!(cached.1, pi_appr);
        })
        .join()
        .expect("pi cache test thread should finish");
    }

    #[test]
    fn ln_constant_cache_is_shared() {
        let ln2 = Computable::ln_constant(2).unwrap();
        assert!(ln2.cached().is_none());
        let _ = ln2.approx(-32);

        let cached = Computable::ln_constant(2)
            .unwrap()
            .cached()
            .expect("ln constant cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn negate() {
        let fifteen: BigInt = "15".parse().unwrap();
        let a = Computable::integer(fifteen.clone());
        let b = Computable::negate(a);
        let answer: BigInt = "-7".parse().unwrap();
        assert_eq!(answer, b.approx(1));
    }

    #[test]
    fn multiply() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(a, b);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn multiply_opposite() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(b, a);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn rational() {
        let sixth: Rational = "1/6".parse().unwrap();
        let c = Computable::rational(sixth);
        let zero = BigInt::zero();
        let one = BigInt::one();
        let ten: BigInt = "10".parse().unwrap();
        let eighty_five: BigInt = "85".parse().unwrap();
        assert_eq!(zero, c.approx(0));
        assert_eq!(zero, c.approx(-1));
        assert_eq!(zero, c.approx(-2));
        assert_eq!(one, c.approx(-3));
        assert_eq!(ten, c.approx(-6));
        assert_eq!(eighty_five, c.approx(-9));
    }

    #[test]
    fn scaled_ln1() {
        let zero = Computable::integer(BigInt::zero());
        let ln = Computable {
            internal: Box::new(Approximation::PrescaledLn(zero)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        };
        let zero = BigInt::zero();
        assert_eq!(zero, ln.approx(100));
    }

    #[test]
    fn scaled_ln1_4() {
        let zero_4: Rational = "0.4".parse().unwrap();
        let rational = Computable::rational(zero_4);
        let ln = Computable {
            internal: Box::new(Approximation::PrescaledLn(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: Cell::new(BoundCache::Invalid),
            exact_sign: Cell::new(ExactSignCache::Invalid),
            signal: None,
        };
        let five: BigInt = "5".parse().unwrap();
        assert_eq!(five, ln.approx(-4));
    }

    #[test]
    fn ln() {
        let five: BigInt = "5".parse().unwrap();
        let integer = Computable::integer(five);
        let ln = Computable::ln(integer);
        let correct: BigInt = "1769595698905".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn exp_and_ln_round_trip() {
        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_close(seven_fifths.clone().exp().ln(), seven_fifths, -40, 2);
    }

    #[test]
    fn exact_transcendental_identities() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());
        assert_close(zero.clone().exp(), one.clone(), -40, 0);
        assert_close(one.ln(), zero.clone(), -40, 0);
        assert_close(zero.clone().sin(), zero.clone(), -40, 0);
        assert_close(zero.clone().cos(), Computable::one(), -40, 0);
        assert_close(zero.tan(), Computable::rational(Rational::zero()), -40, 0);
    }

    #[test]
    fn compare_to_uses_exact_sign_and_rational_shortcuts() {
        let minus_pi = Computable::pi().negate();
        let pi = Computable::pi();
        assert_eq!(minus_pi.try_compare_to(&pi), Some(Ordering::Less));

        let left = Computable::rational(Rational::fraction(7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.try_compare_to(&right), Some(Ordering::Less));
    }

    #[test]
    fn try_compare_to_handles_identical_symbolic_values() {
        let pi = Computable::pi();
        assert_eq!(pi.try_compare_to(&pi), Some(Ordering::Equal));

        let left = Computable::rational(Rational::fraction(3, 7).unwrap());
        let right = Computable::rational(Rational::fraction(3, 7).unwrap());
        assert_eq!(left.try_compare_to(&right), Some(Ordering::Equal));
    }

    #[test]
    fn compare_to_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(
                BigInt::from(1_u8) << 200,
            )));
        assert_eq!(huge.try_compare_to(&base), Some(Ordering::Greater));
        assert_eq!(base.try_compare_to(&huge), Some(Ordering::Less));

        let minus_base = base.negate();
        let minus_huge = huge.negate();
        assert_eq!(minus_huge.try_compare_to(&minus_base), Some(Ordering::Less));
        assert_eq!(
            minus_base.try_compare_to(&minus_huge),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn compare_absolute_uses_exact_shortcuts() {
        let zero = Computable::rational(Rational::zero());
        let tiny = Computable::rational(Rational::fraction(1, 1024).unwrap());
        assert_eq!(zero.compare_absolute(&tiny, -40), Ordering::Less);

        let left = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.compare_absolute(&right, -40), Ordering::Less);
    }

    #[test]
    fn compare_absolute_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(
                BigInt::from(1_u8) << 200,
            )));
        assert_eq!(huge.compare_absolute(&base, -40), Ordering::Greater);
        assert_eq!(base.compare_absolute(&huge, -40), Ordering::Less);
    }

    #[test]
    fn warmed_zero_sum_product_stays_zero() {
        let zero = Computable::pi().add(Computable::pi().negate());
        zero.approx(-128);
        let product = zero.multiply(Computable::pi());
        assert_eq!(product.approx(-128), BigInt::zero());
    }

    #[test]
    fn exp_negative_is_inverse() {
        let eleven_tenths = Computable::rational(Rational::fraction(11, 10).unwrap());
        let product = eleven_tenths
            .clone()
            .exp()
            .multiply(eleven_tenths.negate().exp());
        assert_close(product, Computable::one(), -40, 2);
    }

    #[test]
    fn exp_near_prescaled_limit_round_trip() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        assert_close(half.clone().exp().ln(), half, -40, 2);
    }

    #[test]
    fn exp_large_argument_reduces_by_ln2() {
        let exponent = BigInt::from(200);
        let offset = Computable::rational(Rational::fraction(7, 20).unwrap());
        let value = Computable::ln2()
            .multiply(Computable::integer(exponent.clone()))
            .add(offset.clone());
        let expected = offset.exp().shift_left(200);

        assert_close(value.exp(), expected, -80, 2);
    }

    #[test]
    fn cos_zero() {
        let zero = Computable::rational(Rational::zero());
        let cos = zero.cos();
        let correct: BigInt = "4294967296".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    #[test]
    fn cos_one() {
        let one = Computable::one();
        let cos = one.cos();
        let correct: BigInt = "2320580734".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    fn assert_approx(c: Computable, p: Precision, expected: &str, max_error: i32) {
        let actual = c.approx(p);
        let expected: BigInt = expected.parse().unwrap();
        let error = (&actual - &expected).abs();
        let max_error = BigInt::from(max_error);
        assert!(
            error <= max_error,
            "actual {actual}, expected {expected}, error {error}"
        );
    }

    fn assert_close(left: Computable, right: Computable, p: Precision, max_error: i32) {
        let left = left.approx(p);
        let right = right.approx(p);
        let error = (&left - &right).abs();
        let max_error = BigInt::from(max_error);
        assert!(
            error <= max_error,
            "left {left}, right {right}, error {error}"
        );
    }

    fn pi_times(r: Rational) -> Computable {
        Computable::pi().multiply(Computable::rational(r))
    }

    fn shifted_cos_sin(c: Computable) -> Computable {
        pi_times(Rational::fraction(1, 2).unwrap())
            .add(c.negate())
            .cos()
    }

    #[test]
    fn sin_small_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.sin(), -32, "853278278", 1);

        let zero = Computable::rational(Rational::zero());
        assert_eq!(BigInt::zero(), zero.sin().approx(-32));
    }

    #[test]
    fn sin_medium_arguments() {
        let three: BigInt = "3".parse().unwrap();
        let three = Computable::integer(three);
        assert_approx(three.sin(), -32, "606105819", 1);
    }

    #[test]
    fn sin_cos_direct_medium_exact_rationals_match_reduced_forms() {
        for rational in [
            Rational::fraction(6, 5).unwrap(),
            Rational::fraction(7, 5).unwrap(),
            Rational::fraction(47, 32).unwrap(),
            Rational::try_from(1.23456789_f64).unwrap(),
        ] {
            let value = Computable::rational(rational);
            let complement =
                pi_times(Rational::fraction(1, 2).unwrap()).add(value.clone().negate());

            assert_close(value.clone().sin(), complement.clone().cos(), -96, 2);
            assert_close(value.clone().cos(), complement.sin(), -96, 2);
            assert_close(
                value.clone().negate().sin(),
                value.clone().sin().negate(),
                -96,
                2,
            );
            assert_close(value.clone().negate().cos(), value.cos(), -96, 2);
        }
    }

    #[test]
    fn owned_rational_trig_helpers_match_generic_paths() {
        for rational in [
            Rational::fraction(-1, 5).unwrap(),
            Rational::fraction(1, 5).unwrap(),
            Rational::fraction(6, 5).unwrap(),
            Rational::fraction(7, 5).unwrap(),
            Rational::new(1_000_000),
        ] {
            let generic = Computable::rational(rational.clone());

            assert_close(
                Computable::sin_rational(rational.clone()),
                generic.clone().sin(),
                -80,
                8,
            );
            assert_close(
                Computable::cos_rational(rational.clone()),
                generic.clone().cos(),
                -80,
                8,
            );
            assert_close(Computable::tan_rational(rational), generic.tan(), -80, 16);
        }
    }

    #[test]
    fn sin_large_arguments() {
        let one_two_three: BigInt = "123".parse().unwrap();
        let one_two_three = Computable::integer(one_two_three);
        assert_approx(one_two_three.sin(), -32, "-1975270452", 1);
    }

    #[test]
    fn sin_negative_arguments() {
        let negative_three_fifths = Computable::rational(Rational::fraction(-3, 5).unwrap());
        assert_approx(negative_three_fifths.sin(), -32, "-2425120957", 1);
    }

    #[test]
    fn sin_near_pi_multiples() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let pi_plus_epsilon = Computable::pi().add(epsilon.clone());
        let two_pi_minus_epsilon = pi_times(Rational::new(2)).add(epsilon.clone().negate());

        assert_approx(pi_plus_epsilon.sin(), -32, "-67106133", 1);
        assert_approx(two_pi_minus_epsilon.sin(), -32, "-67106133", 1);
    }

    #[test]
    fn sin_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let half_pi = pi_times(Rational::fraction(1, 2).unwrap());
        let half_pi_plus_epsilon = half_pi.clone().add(epsilon.clone());
        let half_pi_minus_epsilon = half_pi.add(epsilon.negate());

        assert_approx(half_pi_plus_epsilon.sin(), -32, "4294443019", 1);
        assert_approx(half_pi_minus_epsilon.sin(), -32, "4294443019", 1);
    }

    #[test]
    fn sin_matches_shifted_cos_identity() {
        for r in ["-12", "-3/5", "0", "1/5", "3", "123"] {
            let r: Rational = r.parse().unwrap();
            let c = Computable::rational(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }

        for r in ["-7/3", "-1/2", "1/2", "2", "41/6"] {
            let r: Rational = r.parse().unwrap();
            let c = pi_times(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }
    }

    #[test]
    fn inverse_trig_computable_kernels_approximate_expected_values() {
        let value = Computable::rational(Rational::fraction(7, 10).unwrap());
        let negative_value = Computable::rational(Rational::fraction(-7, 10).unwrap());

        assert_approx(value.clone().asin(), -40, "852558563672", 2);
        assert_approx(negative_value.asin(), -40, "-852558563672", 2);
        assert_approx(value.acos(), -40, "874550262507", 2);
    }

    #[test]
    fn endpoint_inverse_trig_computable_kernels_approximate_expected_values() {
        let tiny = Computable::rational(Rational::fraction(1, 1_000_000_000_000).unwrap());
        let near_one = Computable::rational(Rational::fraction(999_999, 1_000_000).unwrap());

        assert_approx(tiny.clone().asin(), -80, "1208925819615", 2);
        assert_approx(tiny.clone().acos(), -40, "1727108826178", 2);
        assert_approx(tiny.atanh(), -80, "1208925819615", 2);
        assert_approx(near_one.clone().asin(), -40, "1725553881793", 2);
        assert_approx(near_one.clone().acos(), -40, "1554944386", 2);
        assert_approx(near_one.atanh(), -40, "7976218668587", 2);
    }

    #[test]
    fn tiny_non_rational_asin_uses_prescaled_series() {
        let tiny = Computable::rational(Rational::new(2))
            .sqrt()
            .shift_left(-20);
        let result = tiny.clone().asin();

        assert!(matches!(
            result.internal.as_ref(),
            Approximation::PrescaledAsin(_)
        ));
        assert_close(result, Computable::asin_deferred(tiny), -80, 4);
    }

    #[test]
    fn tiny_non_rational_atanh_uses_prescaled_series() {
        let tiny = Computable::rational(Rational::new(2))
            .sqrt()
            .shift_left(-20);
        let result = tiny.clone().atanh();

        assert!(matches!(
            result.internal.as_ref(),
            Approximation::PrescaledAtanh(_)
        ));
        assert_close(result, Computable::atanh_direct_deferred(tiny), -80, 4);
    }

    #[test]
    fn inverse_hyperbolic_computable_kernels_approximate_expected_values() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        let negative_half = Computable::rational(Rational::fraction(-1, 2).unwrap());
        let two = Computable::rational(Rational::new(2));

        assert_approx(half.clone().asinh(), -40, "529097997076", 2);
        assert_approx(negative_half.clone().asinh(), -40, "-529097997076", 2);
        assert_approx(two.acosh(), -40, "1448010520960", 2);
        assert_approx(
            Computable::rational(Rational::new(2)).sqrt().acosh(),
            -40,
            "969080507343",
            2,
        );
        assert_approx(half.atanh(), -40, "603968492904", 2);
        assert_approx(negative_half.atanh(), -40, "-603968492904", 2);
    }

    #[test]
    fn deep_add_chain_approximates_without_recursive_walk() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.add(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(5001));
    }

    #[test]
    fn deep_multiply_chain_of_ones_stays_exact() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(1));
    }

    #[test]
    fn deep_multiply_chain_by_one_preserves_irrational() {
        let mut value = Computable::pi();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_close(value, Computable::pi(), -40, 2);
    }

    #[test]
    fn rational_msd_exact_for_small_fraction() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        assert_eq!(third.msd(-4), Some(-2));
    }

    #[test]
    fn multiply_combines_exact_scales() {
        let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
        let combined = Computable::pi()
            .multiply(scale.clone())
            .multiply(scale.clone())
            .multiply(scale);
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(343, 512).unwrap()));
        assert_close(combined, expected, -60, 2);
    }

    #[test]
    fn square_of_scaled_irrational_reuses_exact_scale() {
        let scaled =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let expected = Computable::pi()
            .square()
            .multiply(Computable::rational(Rational::fraction(49, 64).unwrap()));
        assert_close(scaled.square(), expected, -60, 2);
    }

    #[test]
    fn inverse_of_exact_fraction_has_structural_bound() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        let inverse = third.inverse();
        assert_eq!(inverse.sign(), Sign::Plus);
        assert_eq!(inverse.msd(-4), Some(1));
    }

    #[test]
    fn inverse_of_scaled_irrational_uses_structural_msd() {
        let scale = Rational::fraction(7, 8).unwrap();
        let base = Computable::pi();
        base.approx(-16);
        let value = base.multiply(Computable::rational(scale.clone()));
        assert_eq!(value.planning_msd(), Some(Some(0)));
        assert_eq!(value.msd(-4), Some(1));
        let inverse = value.inverse();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(scale.inverse().unwrap()));
        assert_close(inverse, expected, -60, 2);

        let negative_scale = Rational::fraction(-7, 8).unwrap();
        let negative_value = Computable::pi().multiply(Computable::rational(negative_scale));
        let normalized = negative_value.inverse().negate();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(Rational::fraction(8, 7).unwrap()));
        assert_close(normalized, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_fraction_has_structural_bound() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap()).square();
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(-3));
    }

    #[test]
    fn sqrt_of_scaled_square_tracks_structural_msd() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(1));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_square_of_negative_value_returns_absolute_value() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap())
            .square()
            .sqrt();
        assert_eq!(
            value.approx(-8),
            Computable::rational(Rational::fraction(3, 8).unwrap()).approx(-8)
        );
    }

    #[test]
    fn double_negate_collapses_at_construction() {
        let value = Computable::pi().negate().negate();
        assert_close(value, Computable::pi(), -60, 2);
    }

    #[test]
    fn inverse_of_inverse_of_nonzero_value_collapses_at_construction() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().inverse().inverse();
        assert_close(value, base, -60, 2);
    }

    #[test]
    fn inverse_of_square_of_nonzero_value_collapses_at_construction() {
        let base =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let value = base.clone().square().inverse();
        let expected = base.inverse().square();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn nested_offsets_collapse_at_construction() {
        let value = Computable::pi().shift_left(5).shift_right(3);
        let expected = Computable::pi().shift_left(2);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_value_collapses_to_square_of_positive_value() {
        let value = Computable::pi().negate().square();
        let expected = Computable::pi().square();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_negative_nonzero_value_normalizes_sign() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().negate().inverse();
        let expected = base.inverse().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_one_collapses_to_negate() {
        let minus_one = Computable::rational(Rational::one().neg());
        let value = Computable::pi().multiply(minus_one);
        let expected = Computable::pi().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_power_of_two_fraction_collapses_to_shift() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_power_of_two_fraction_collapses_to_shift_and_negate() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3).negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_power_of_two_scaled_value_collapses_to_shifted_square() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::new(8)))
            .square();
        let expected = Computable::pi().square().shift_left(6);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exactly_scaled_square_collapses_at_construction() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exact_rational_square_is_exact() {
        let value = Computable::rational(Rational::fraction(49, 64).unwrap()).sqrt();
        let expected = Computable::rational(Rational::fraction(7, 8).unwrap());
        assert_close(value, expected, -60, 0);
    }

    #[test]
    fn sqrt_squarefree_two_three_reuses_shared_constants() {
        let sqrt_twelve = Computable::rational(Rational::new(12)).sqrt();
        let expected = Computable::sqrt_constant(3)
            .unwrap()
            .multiply(Computable::rational(Rational::new(2)));
        assert_close(sqrt_twelve, expected, -60, 2);
    }

    #[test]
    fn square_of_sqrt_of_positive_value_collapses_at_construction() {
        let value = Computable::rational(Rational::new(2)).sqrt().square();
        let expected = Computable::rational(Rational::new(2));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_shifted_nonzero_value_collapses_to_shifted_inverse() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().shift_left(5).inverse();
        let expected = base.inverse().shift_right(5);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn structural_facts_for_exact_rationals() {
        let zero = Computable::rational(Rational::zero()).structural_facts();
        assert_eq!(zero.sign, Some(RealSign::Zero));
        assert_eq!(zero.zero, ZeroKnowledge::Zero);
        assert!(zero.exact_rational);
        assert_eq!(zero.magnitude, None);

        let negative = Computable::rational(Rational::fraction(-7, 8).unwrap()).structural_facts();
        assert_eq!(negative.sign, Some(RealSign::Negative));
        assert_eq!(negative.zero, ZeroKnowledge::NonZero);
        assert!(negative.exact_rational);
        assert_eq!(
            negative.magnitude,
            Some(MagnitudeBits {
                msd: -1,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn structural_facts_for_shared_constant() {
        let facts = Computable::pi().structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert!(!facts.exact_rational);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: 1,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn structural_facts_for_constant_rational_offset_certificates() {
        let pi_minus_three = Computable::pi().add(Computable::rational(Rational::new(-3)));
        let facts = pi_minus_three.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -3,
                exact_msd: true,
            })
        );
        assert_eq!(pi_minus_three.sign_until(0), Some(RealSign::Positive));

        let three_minus_pi = Computable::rational(Rational::new(3)).add(Computable::pi().negate());
        let facts = three_minus_pi.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Negative));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -3,
                exact_msd: true,
            })
        );

        let two_pi_minus_six = Computable::pi()
            .shift_left(1)
            .add(Computable::rational(Rational::new(-6)));
        let facts = two_pi_minus_six.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -2,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn zero_status_uses_structural_facts_without_refinement() {
        assert_eq!(
            Computable::rational(Rational::zero()).zero_status(),
            ZeroKnowledge::Zero
        );
        assert_eq!(
            Computable::rational(Rational::fraction(-7, 8).unwrap()).zero_status(),
            ZeroKnowledge::NonZero
        );
        assert_eq!(Computable::pi().zero_status(), ZeroKnowledge::NonZero);

        let near_pi =
            Computable::pi().add(Computable::rational(Rational::fraction(-22, 7).unwrap()));
        assert_eq!(near_pi.zero_status(), ZeroKnowledge::NonZero);
    }

    #[test]
    fn sign_until_respects_precision_floor() {
        let near_pi = Computable::pi().add(Computable::rational(Rational::new(-3)));

        assert_eq!(near_pi.sign_until(0), Some(RealSign::Positive));
        assert_eq!(near_pi.sign_until(-8), Some(RealSign::Positive));
    }

    #[test]
    fn sign_until_uses_structural_bounds_without_refinement() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(-7, 8).unwrap()))
            .inverse()
            .negate();

        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
    }

    #[test]
    fn add_with_dominant_term_has_structural_bound() {
        let value = Computable::integer(BigInt::from(8))
            .add(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(2));
    }

    #[test]
    fn opposite_sign_add_with_inexact_msd_is_not_certified_nonzero() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        let negative_radical = Computable::rational(Rational::new(2))
            .sqrt()
            .multiply_rational(Rational::fraction(-3, 8).unwrap());
        let sum = half.add(negative_radical);

        assert_eq!(sum.zero_status(), ZeroKnowledge::Unknown);
        assert_eq!(sum.sign_until(-64), Some(RealSign::Negative));
    }

    #[test]
    fn add_ignores_tiny_term_at_target_precision() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 200).unwrap(),
        );
        assert_eq!(
            big.clone().add(tiny).compare_absolute(&big, -128),
            Ordering::Equal
        );
    }

    #[test]
    fn add_does_not_ignore_tiny_opposite_sign_term() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
        );
        let sum = big.clone().add(tiny.clone());
        let delta = sum.add(big.negate());
        assert_eq!(delta.compare_absolute(&tiny, -180), Ordering::Equal);
    }

    #[test]
    fn deep_structural_bound_chain() {
        let scale = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let mut value = Computable::pi();
        value.approx(-16);
        for _ in 0..2000 {
            value = value.multiply(scale.clone()).inverse().negate();
        }
        assert_eq!(value.sign(), Sign::Plus);
        assert_close(value, Computable::pi(), -60, 2);
    }

    #[test]
    fn huge_trig_arguments_reduce_correctly() {
        let huge_multiple = BigInt::from(1_u8) << 200;
        let offset = Computable::rational(Rational::fraction(7, 5).unwrap());
        let huge = Computable::pi()
            .multiply(Computable::integer(huge_multiple))
            .add(offset.clone());

        assert_eq!(
            huge.clone()
                .sin()
                .compare_absolute(&offset.clone().sin(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.clone()
                .cos()
                .compare_absolute(&offset.clone().cos(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.tan().compare_absolute(&offset.tan(), -72),
            Ordering::Equal
        );
    }

    #[test]
    fn exact_large_rational_trig_uses_correct_quadrant() {
        let million = Computable::rational(Rational::new(1_000_000));

        assert_approx(million.clone().sin(), -32, "-1503210646", 8);
        assert_approx(million.clone().cos(), -32, "4023319752", 8);
        assert_approx(million.tan(), -32, "-1604704811", 8);
    }

    #[test]
    fn exact_huge_rational_trig_uses_correct_quadrant() {
        let huge = Rational::new(10).powi(BigInt::from(30)).unwrap();
        let direct = Computable::rational(huge.clone());

        assert_approx(direct.clone().sin(), -72, "-425565037129932206620", 8);
        assert_approx(direct.clone().cos(), -72, "-4703152091704373381319", 8);
        assert_approx(direct.tan(), -72, "427303652622316740317", 16);
    }

    #[test]
    fn tan_small_and_medium_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.tan(), -32, "870632973", 2);

        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_approx(seven_fifths.tan(), -32, "24901720944", 2);
    }

    #[test]
    fn tan_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let near_half_pi = pi_times(Rational::fraction(1, 2).unwrap()).add(epsilon.negate());
        assert_approx(near_half_pi.tan(), -32, "274855536959", 8);
    }

    #[test]
    fn ln_sqrt_pi() {
        let pi = Computable::pi();
        let sqrt = Computable::sqrt(pi);
        let ln = Computable::ln(sqrt);
        let correct: BigInt = "629321910077".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn ln_large_power_of_two() {
        let value = Computable::rational(Rational::new(1024));
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()), -40, 2);
    }

    #[test]
    fn ln_tiny_power_of_two() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), denominator).unwrap(),
        );
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()).negate(), -40, 2);
    }

    #[test]
    fn ln_exact_binary_scaled_rational() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(3), denominator).unwrap(),
        );
        let expected = Computable::rational(Rational::new(3))
            .ln()
            .add(Computable::rational(Rational::new(-10)).multiply(Computable::ln2()));
        assert_close(value.ln(), expected, -40, 2);
    }

    #[test]
    fn ln_smooth_rational_reuses_shared_prime_logs() {
        let value = Computable::rational(Rational::fraction(45, 14).unwrap());
        let expected = Computable::ln_constant(3)
            .unwrap()
            .multiply(Computable::rational(Rational::new(2)))
            .add(Computable::ln_constant(5).unwrap())
            .add(Computable::ln_constant(2).unwrap().negate())
            .add(Computable::ln_constant(7).unwrap().negate());
        assert_close(value.ln(), expected, -50, 3);
    }

    #[test]
    fn sqrt_square_round_trip() {
        let two = Computable::rational(Rational::new(2));
        let sqrt_two = two.clone().sqrt();
        assert_close(sqrt_two.square(), two, -40, 2);
    }

    #[test]
    fn ln_near_prescaled_limit_round_trip() {
        let value = Computable::rational(Rational::fraction(47, 32).unwrap());
        assert_close(value.clone().ln().exp(), value, -40, 2);
    }

    #[test]
    fn erf_known_values() {
        assert_close(
            Computable::zero().erf(),
            Computable::zero(),
            -160,
            2,
        );
        assert_close(
            Computable::rational(Rational::fraction(1, 2).unwrap()).erf(),
            Computable::rational("0.5204998778130465376827466538919645287364".parse().unwrap()),
            -90,
            2,
        );
        assert_close(
            Computable::one().erf(),
            Computable::rational("0.8427007929497148693412206350826092592960".parse().unwrap()),
            -90,
            2,
        );
        assert_close(
            Computable::one().erfc(),
            Computable::rational("0.1572992070502851306587793649173907407040".parse().unwrap()),
            -90,
            2,
        );
        assert_close(
            Computable::one().erfcx(),
            Computable::rational("0.4275835761558070044107503444905151808202".parse().unwrap()),
            -90,
            2,
        );
    }

    #[test]
    fn normal_density_and_cdf_known_values() {
        assert_close(
            Computable::zero().dnorm(),
            Computable::rational("0.39894228040143267793994605993438186847585863".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::one().pnorm(),
            Computable::rational("0.8413447460685429485852325456320379224779".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::one().normal_sf(),
            Computable::rational("0.1586552539314570514147674543679620775221".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::normal_interval(Computable::zero(), Computable::one()),
            Computable::rational("0.3413447460685429485852325456320379224779".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::zero().log_pnorm(),
            Computable::rational("-0.6931471805599453094172321214581765680755".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::zero().log_normal_sf(),
            Computable::rational("-0.6931471805599453094172321214581765680755".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::zero().log_dnorm(),
            Computable::rational("-0.9189385332046727417803297364056176398614".parse().unwrap()),
            -120,
            2,
        );
    }

    #[test]
    fn normal_tail_nodes_have_structural_signs() {
        let x = Computable::one();
        assert_eq!(x.clone().erfc().exact_sign(), Some(Sign::Plus));
        assert_eq!(x.clone().erfcx().exact_sign(), Some(Sign::Plus));
        assert_eq!(x.clone().normal_sf().exact_sign(), Some(Sign::Plus));
        assert_eq!(
            Computable::normal_interval(Computable::zero(), Computable::one()).exact_sign(),
            Some(Sign::Plus)
        );
        assert_eq!(
            Computable::normal_interval(Computable::one(), Computable::one()).exact_sign(),
            Some(Sign::NoSign)
        );
        assert_eq!(x.clone().log_pnorm().exact_sign(), Some(Sign::Minus));
        assert_eq!(x.clone().log_normal_sf().exact_sign(), Some(Sign::Minus));
        assert_eq!(x.log_dnorm().exact_sign(), Some(Sign::Minus));
    }

    #[test]
    fn expm1_preserves_small_argument_and_sign() {
        let tiny = Computable::rational(Rational::fraction(1, 1_000_000).unwrap());
        assert_eq!(tiny.clone().expm1().exact_sign(), Some(Sign::Plus));
        assert_eq!(tiny.clone().negate().expm1().exact_sign(), Some(Sign::Minus));
        assert_close(
            tiny.expm1(),
            Computable::rational("0.0000010000005000001666667083333416667".parse().unwrap()),
            -120,
            2,
        );
    }

    #[test]
    fn normal_quantile_inverts_cdf() {
        let two = Computable::rational(Rational::new(2));
        let p = two.clone().pnorm();
        let seed = BigInt::from((1.9999_f64 * f64::from(1_u32 << 13)).round() as i64);
        let q = Computable::normal_quantile(p, seed, -13);
        assert_close(q, two, -120, 2);
    }

    #[test]
    fn add() {
        let three: BigInt = "3".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(three);
        let b = Computable::integer(five);
        let c = Computable::add(a, b);
        let answer: BigInt = "256".parse().unwrap();
        assert_eq!(answer, c.approx(-5));
    }

    #[test]
    fn scale_up() {
        let ten: BigInt = "10".parse().unwrap();
        let three: BigInt = "3".parse().unwrap();
        assert_eq!(ten, scale(ten.clone(), 0));
        let a = scale(ten.clone(), -2);
        assert_eq!(three, a);
        let forty: BigInt = "40".parse().unwrap();
        let b = scale(ten.clone(), 2);
        assert_eq!(forty, b);
    }

    #[test]
    fn msd_refines_ambiguous_unit_approximations_at_binary_boundaries() {
        for value in [
            Computable::rational(Rational::fraction(1, 4).unwrap()).atan(),
            Computable::rational(Rational::fraction(1, 4).unwrap()).asinh(),
        ] {
            assert_eq!(value.msd(-128), Some(-3));
        }
    }
}
