use hyperreal::{Problem, Rational};
use num::{BigInt, BigRational, BigUint, Zero};
use proptest::prelude::*;

fn hyper(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).expect("generated denominator is nonzero")
}

fn oracle(numerator: i64, denominator: u64) -> BigRational {
    BigRational::new(BigInt::from(numerator), BigInt::from(denominator))
}

fn from_oracle(value: &BigRational) -> Rational {
    Rational::from_bigint_fraction(
        value.numer().clone(),
        BigUint::try_from(value.denom().clone()).expect("BigRational denominator is positive"),
    )
    .expect("BigRational denominator is nonzero")
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2_048))]

    #[test]
    fn binary32_import_matches_independent_exact_rational(bits in any::<u32>()) {
        let value = f32::from_bits(bits);
        let actual = Rational::try_from(value);
        if let Some(expected) = BigRational::from_float(value) {
            let actual = actual.expect("finite binary32 value");
            prop_assert_eq!(BigInt::from_biguint(actual.sign(), actual.numerator().clone()), expected.numer().clone());
            prop_assert_eq!(BigInt::from(actual.denominator().clone()), expected.denom().clone());
        } else if value.is_nan() {
            prop_assert_eq!(actual, Err(Problem::NotANumber));
        } else {
            prop_assert_eq!(actual, Err(Problem::Infinity));
        }
    }

    #[test]
    fn binary64_import_matches_independent_exact_rational(bits in any::<u64>()) {
        let value = f64::from_bits(bits);
        let actual = Rational::try_from(value);
        if let Some(expected) = BigRational::from_float(value) {
            let actual = actual.expect("finite binary64 value");
            prop_assert_eq!(BigInt::from_biguint(actual.sign(), actual.numerator().clone()), expected.numer().clone());
            prop_assert_eq!(BigInt::from(actual.denominator().clone()), expected.denom().clone());
        } else if value.is_nan() {
            prop_assert_eq!(actual, Err(Problem::NotANumber));
        } else {
            prop_assert_eq!(actual, Err(Problem::Infinity));
        }
    }

    #[test]
    fn two_limb_reduction_matches_independent_gcd(
        numerator in any::<u128>(),
        denominator in 1_u128..=u128::MAX,
        negative in any::<bool>(),
    ) {
        let numerator = if negative { -BigInt::from(numerator) } else { BigInt::from(numerator) };
        let expected = BigRational::new(numerator.clone(), BigInt::from(denominator));
        let actual = Rational::from_bigint_fraction(numerator, BigUint::from(denominator)).unwrap();
        // Compare canonical parts directly, so a defect in Hyperreal's own
        // equality implementation cannot make this oracle check pass.
        prop_assert_eq!(BigInt::from_biguint(actual.sign(), actual.numerator().clone()), expected.numer().clone());
        prop_assert_eq!(BigInt::from(actual.denominator().clone()), expected.denom().clone());
    }

    #[test]
    fn construction_and_arithmetic_match_big_rational(
        an in -1_000_000_i64..=1_000_000,
        ad in 1_u64..=1_000_000,
        bn in -1_000_000_i64..=1_000_000,
        bd in 1_u64..=1_000_000,
    ) {
        let a = hyper(an, ad);
        let b = hyper(bn, bd);
        let ao = oracle(an, ad);
        let bo = oracle(bn, bd);

        prop_assert_eq!(a.clone(), from_oracle(&ao));
        prop_assert_eq!(&a + &b, from_oracle(&(&ao + &bo)));
        prop_assert_eq!(&a - &b, from_oracle(&(&ao - &bo)));
        prop_assert_eq!(&a * &b, from_oracle(&(&ao * &bo)));
        prop_assert_eq!(a.partial_cmp(&b), ao.partial_cmp(&bo));

        if !bo.is_zero() {
            prop_assert_eq!(&a / &b, from_oracle(&(&ao / &bo)));
        }

        let truncated = BigRational::from_integer(ao.to_integer());
        prop_assert_eq!(a.trunc(), from_oracle(&truncated));
        prop_assert_eq!(a.fract(), from_oracle(&(ao - truncated)));
    }

    #[test]
    fn integer_powers_match_big_rational(
        numerator in -10_000_i64..=10_000,
        denominator in 1_u64..=10_000,
        exponent in -12_i32..=12,
    ) {
        let value = hyper(numerator, denominator);
        let reference = oracle(numerator, denominator);

        if numerator == 0 && exponent < 0 {
            prop_assert!(value.powi(BigInt::from(exponent)).is_err());
        } else {
            let expected = reference.pow(exponent);
            prop_assert_eq!(
                value.powi(BigInt::from(exponent)).expect("defined rational power"),
                from_oracle(&expected),
            );
        }
    }
}

#[test]
fn alternate_decimal_format_honors_zero_precision() {
    let value = hyper(1, 3);
    assert_eq!(format!("{value:#.0}"), "0.");
}

#[test]
fn truncating_proper_fractions_returns_canonical_zero() {
    for numerator in [-1, 1] {
        let truncated = hyper(numerator, 2).trunc();
        assert_eq!(truncated, Rational::zero());
        assert!(truncated.is_zero());
    }
}

#[test]
fn remaining_public_rational_helpers_cover_boundaries() {
    let negative = Rational::from_bigint(BigInt::from(-7));
    assert_eq!(negative, Rational::new(-7));
    assert!(Rational::one().is_one());
    assert!(!negative.is_one());

    let seven_fifths = hyper(7, 5);
    assert_eq!(seven_fifths.shifted_big_integer(3), BigInt::from(11));
    assert_eq!(seven_fifths.shifted_big_integer(-1), BigInt::from(0));
    assert_eq!(hyper(-11, 3).shifted_big_integer(-1), BigInt::from(-1));

    assert!(Rational::new(144).extract_square_will_succeed());
    let oversized = Rational::from_bigint(BigInt::from(1_u8) << 5_000_usize);
    assert!(!oversized.extract_square_will_succeed());

    assert_eq!(
        Rational::zero().powi(BigInt::from(-1)),
        Err(Problem::DivideByZero),
    );
    assert_eq!(Rational::zero().powi(BigInt::from(0)), Ok(Rational::one()),);
}
