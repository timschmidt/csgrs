#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_word_gcd_matches_euclidean_reference() {
        fn reference(mut left: u128, mut right: u128) -> u128 {
            while right != 0 {
                (left, right) = (right, left % right);
            }
            left
        }

        let edge_cases = [
            (0, 0),
            (0, 1),
            (1, 0),
            (1, 1),
            (2, 4),
            (u128::MAX, u128::MAX - 1),
            (1_u128 << 127, 1_u128 << 126),
            ((1_u128 << 127) + (1_u128 << 63), 1_u128 << 64),
        ];
        for (left, right) in edge_cases {
            assert_eq!(Rational::gcd_word(left, right), reference(left, right));
        }

        let mut left = 0x243f_6a88_85a3_08d3_1319_8a2e_0370_7344_u128;
        let mut right = 0xa409_3822_299f_31d0_082e_fa98_ec4e_6c89_u128;
        for _ in 0..20_000 {
            left ^= left << 13;
            left ^= left >> 17;
            left ^= left << 43;
            right ^= right << 29;
            right ^= right >> 31;
            right ^= right << 37;
            assert_eq!(Rational::gcd_word(left, right), reference(left, right));
        }
    }

    #[test]
    fn word_multiplication_cross_cancellation_stays_reduced() {
        let dyadic = Rational::try_from(0.123_456_789_f64).unwrap();
        let scaled = &dyadic * Rational::new(10);
        assert_eq!(
            scaled,
            Rational::from_bigint_fraction(
                BigInt::from(44_479_995_914_940_635_u64),
                BigUint::from(36_028_797_018_963_968_u64),
            )
            .unwrap()
        );

        let left = Rational::fraction(35, 22).unwrap();
        let right = Rational::fraction(121, 14).unwrap();
        assert_eq!(&left * right, Rational::fraction(55, 4).unwrap());
    }

    #[test]
    fn exact_dyadic_f64_view_round_trips_finite_binary64_values() {
        let mut state = 0xbb67_ae85_84ca_a73b_u64;
        let mut recovered_count = 0_u32;
        for _ in 0..20_000 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let value = f64::from_bits(state);
            if !value.is_finite() {
                continue;
            }
            let rational = Rational::try_from(value).unwrap();
            if let Some(recovered) = rational.dyadic_to_f64_exact() {
                assert_eq!(Rational::try_from(recovered).unwrap(), rational);
                recovered_count += 1;
            }
        }
        assert!(recovered_count > 19_000);
    }

    #[test]
    fn exact_dyadic_f64_view_rejects_unrepresentable_values() {
        let too_precise = Rational::new((1_i64 << 54) + 1);
        let exactly_representable = Rational::new((1_i64 << 54) + 4);
        let non_dyadic = Rational::fraction(1, 3).unwrap();
        let too_small = Rational::from_bigint_fraction(
            BigInt::from(1_u8),
            BigUint::from(1_u8) << 1075,
        )
        .unwrap();

        assert_eq!(non_dyadic.dyadic_to_f64_exact(), None);
        assert_eq!(too_precise.dyadic_to_f64_exact(), None);
        assert_eq!(
            exactly_representable.dyadic_to_f64_exact(),
            Some((1_u64 << 54) as f64 + 4.0),
        );
        assert_eq!(too_small.dyadic_to_f64_exact(), None);
    }

    #[test]
    fn display() {
        let many: Rational = "12345".parse().unwrap();
        let s = format!("{many}");
        assert_eq!(s, "12345");
        let five: Rational = "5".parse().unwrap();
        let third: Rational = "1/3".parse().unwrap();
        let s = format!("{}", five * third);
        assert_eq!(s, "1 2/3");
    }

    #[test]
    fn decimals() {
        let first: Rational = "0.0".parse().unwrap();
        assert_eq!(first, Rational::zero());
        let a: Rational = "0.4".parse().unwrap();
        let b: Rational = "2.5".parse().unwrap();
        let answer = a * b;
        assert_eq!(answer, Rational::one());
    }

    #[test]
    /// Large decimal integer parsing and multiplication remain exact.
    fn parse() {
        let big: Rational = "288230376151711743".parse().unwrap();
        let small: Rational = "45".parse().unwrap();
        let expected: Rational = "12970366926827028435".parse().unwrap();
        assert_eq!(big * small, expected);
    }

    #[test]
    fn parse_fractions() {
        let third: Rational = "1/3".parse().unwrap();
        let minus_four: Rational = "-4".parse().unwrap();
        let twelve: Rational = "12/20".parse().unwrap();
        let answer = third + minus_four * twelve;
        let expected: Rational = "-31/15".parse().unwrap();
        assert_eq!(answer, expected);
    }

    #[test]
    fn parse_fraction_rejects_zero_denominator_and_reduces() {
        assert_eq!("1/0".parse::<Rational>(), Err(Problem::DivideByZero));
        assert_eq!("0/0".parse::<Rational>(), Err(Problem::DivideByZero));

        let reduced: Rational = "9/18".parse().unwrap();
        assert_eq!(reduced, Rational::fraction(1, 2).unwrap());
        assert_eq!(format!("{reduced}"), "1/2");
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_rejects_invalid_or_uncanonical_rational_state() {
        let bad = r#"{"sign":1,"numerator":[1],"denominator":[]}"#;
        assert!(serde_json::from_str::<Rational>(bad).is_err());

        let unreduced = r#"{"sign":1,"numerator":[9],"denominator":[18]}"#;
        let decoded: Rational = serde_json::from_str(unreduced).unwrap();
        assert_eq!(decoded, Rational::fraction(1, 2).unwrap());
        assert_eq!(format!("{decoded}"), "1/2");
    }

    #[test]
    fn square_reduced() {
        let thirty_two = Rational::new(32);
        let (square, rest) = thirty_two.extract_square_reduced();
        let four = Rational::new(4);
        assert_eq!(square, four);
        let two = Rational::new(2);
        assert_eq!(rest, two);
        let minus_one = Rational::new(-1);
        let (square, rest) = minus_one.clone().extract_square_reduced();
        assert_eq!(square, Rational::one());
        assert_eq!(rest, minus_one);
    }

    #[test]
    fn signs() {
        let half: Rational = "4/8".parse().unwrap();
        let one = Rational::one();
        let minus_half = half - one;
        let two = Rational::new(2);
        let zero = Rational::zero();
        let minus_two = zero - two;
        let i2 = minus_two.inverse().unwrap();
        assert_eq!(i2, minus_half);
    }

    #[test]
    fn half_plus_one_times_two() {
        let two = Rational::new(2);
        let half = two.inverse().unwrap();
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        let sum = half + one;
        assert_eq!(sum * two, three);
    }

    #[test]
    fn average_pair_matches_expanded_exact_arithmetic() {
        let cases = [
            (
                Rational::fraction(5, 8).unwrap(),
                Rational::fraction(5, 8).unwrap(),
            ),
            (
                Rational::fraction(-7, 12).unwrap(),
                Rational::fraction(11, 18).unwrap(),
            ),
            (
                Rational::zero(),
                Rational::fraction(-13, 32).unwrap(),
            ),
            (
                Rational::from_parts_raw(
                    Plus,
                    BigUint::from(3_u8),
                    BigUint::one() << 200_usize,
                ),
                Rational::from_parts_raw(
                    Minus,
                    BigUint::from(5_u8),
                    BigUint::one() << 201_usize,
                ),
            ),
            (
                Rational::from_unsigned_integer(BigUint::one() << 127_usize),
                Rational::from_unsigned_integer(
                    (BigUint::one() << 127_usize) + BigUint::from(2_u8),
                ),
            ),
        ];
        let half = Rational::fraction(1, 2).unwrap();
        for (left, right) in cases {
            let expanded = (&left + &right) * &half;
            assert_eq!(Rational::average_pair(&left, &right), expanded);
            assert_eq!(Rational::average_pair(&right, &left), expanded);
        }
    }

    #[test]
    fn three_divided_by_six() {
        let three = Rational::new(3);
        let six = Rational::new(6);
        let half: Rational = "1/2".parse().unwrap();
        assert_eq!(three / six, half);
    }

    #[test]
    fn one_plus_two() {
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        assert_eq!(one + two, three);
    }

    #[test]
    fn two_minus_one() {
        let two = Rational::new(2);
        let one = Rational::one();
        assert_eq!(two - one, Rational::one());
    }

    #[test]
    fn two_times_three() {
        let two = Rational::new(2);
        let three = Rational::new(3);
        assert_eq!(two * three, Rational::new(6));
    }

    #[test]
    fn fract() {
        let seventy_ninths = Rational::fraction(70, 9).unwrap();
        assert_eq!(seventy_ninths.fract(), Rational::fraction(7, 9).unwrap());
        assert_eq!(
            seventy_ninths.neg().fract(),
            Rational::fraction(-7, 9).unwrap()
        );
        let six = Rational::new(6);
        assert_eq!(six.fract(), Rational::zero());
    }

    #[test]
    fn trunc() {
        let seventy_ninths = Rational::fraction(70, 9).unwrap();
        let whole = seventy_ninths.trunc();
        let frac = seventy_ninths.fract();
        assert_eq!(whole + frac, seventy_ninths);
        let shrink = Rational::fraction(-405, 11).unwrap();
        let whole = shrink.trunc();
        let frac = shrink.fract();
        assert_eq!(whole + frac, shrink);
        let zero = Rational::zero();
        let whole = zero.trunc();
        let frac = zero.fract();
        assert_eq!(whole, frac);
        assert_eq!(whole + frac, zero);
    }

    #[test]
    fn power() {
        let one_two_five = Rational::new(5).powi(BigInt::from(-3));
        assert_eq!(one_two_five, Rational::fraction(1, 125));
        let more = Rational::new(7).powi(11i32.into()).unwrap();
        assert_eq!(more, Rational::new(1_977_326_743));
    }

    #[test]
    fn sqrt_trouble() {
        for (n, root, rest) in [
            (1, 1, 1),
            (2, 1, 2),
            (3, 1, 3),
            (4, 2, 1),
            (16, 4, 1),
            (400, 20, 1),
            (1323, 21, 3),
            (4761, 69, 1),
            (123456, 8, 1929),
            (715716, 846, 1),
        ] {
            let n = Rational::new(n);
            let reduced = n.extract_square_reduced();
            assert_eq!(reduced, (Rational::new(root), Rational::new(rest)));
        }
    }

    #[test]
    fn word_sized_square_extraction_preserves_exact_product() {
        for value in [
            1_u64,
            2,
            18,
            123_456,
            715_716,
            u32::MAX as u64,
            u64::MAX,
        ] {
            let (root, rest) = Rational::extract_square(BigUint::from(value));
            assert_eq!(&root * &root * rest, BigUint::from(value));
        }
    }

    #[test]
    fn clones_share_storage_without_sharing_arithmetic_results() {
        let value = Rational::fraction(7, 11).unwrap();
        let clone = value.clone();
        assert!(Arc::ptr_eq(&value.0, &clone.0));

        let negated = -clone;
        assert_eq!(value, Rational::fraction(7, 11).unwrap());
        assert_eq!(negated, Rational::fraction(-7, 11).unwrap());
        assert!(!Arc::ptr_eq(&value.0, &negated.0));
    }

    #[test]
    fn identity_constructors_share_canonical_storage() {
        let zero = Rational::zero();
        let another_zero = Rational::zero();
        let one = Rational::one();
        let another_one = Rational::one();

        assert!(Arc::ptr_eq(&zero.0, &another_zero.0));
        assert!(Arc::ptr_eq(&one.0, &another_one.0));
        assert!(!Arc::ptr_eq(&zero.0, &one.0));
    }

    #[test]
    fn perfect_nth_root_detects_exact_rational_roots() {
        assert_eq!(
            Rational::new(27).perfect_nth_root(3),
            Some(Rational::new(3))
        );
        assert_eq!(
            Rational::new(-27).perfect_nth_root(3),
            Some(Rational::new(-3))
        );
        assert_eq!(
            Rational::fraction(8, 27).unwrap().perfect_nth_root(3),
            Some(Rational::fraction(2, 3).unwrap())
        );
        assert_eq!(Rational::new(2).perfect_nth_root(3), None);
        assert_eq!(Rational::new(-16).perfect_nth_root(4), None);
        assert_eq!(Rational::new(16).perfect_nth_root(0), None);
    }

    #[test]
    fn decimal() {
        let decimal: Rational = "7.125".parse().unwrap();
        assert!(!decimal.prefer_fraction());
        let half: Rational = "4/8".parse().unwrap();
        assert!(!half.prefer_fraction());
        let third: Rational = "2/6".parse().unwrap();
        assert!(third.prefer_fraction());
    }

    #[test]
    fn power_of_two_shift_detects_only_power_of_two_ratios() {
        assert_eq!(
            Rational::fraction(8, 1).unwrap().power_of_two_shift(),
            Some((3, Plus))
        );
        assert_eq!(
            Rational::fraction(1, 8).unwrap().power_of_two_shift(),
            Some((-3, Plus))
        );
        assert_eq!(
            Rational::fraction(-4, 32).unwrap().power_of_two_shift(),
            Some((-3, Minus))
        );
        assert_eq!(Rational::fraction(7, 8).unwrap().power_of_two_shift(), None);
        assert_eq!(Rational::fraction(5, 6).unwrap().power_of_two_shift(), None);
        assert_eq!(Rational::zero().power_of_two_shift(), None);
    }

    #[test]
    fn add_and_subtract_one_helpers_match_generic_arithmetic() {
        for value in [
            Rational::zero(),
            Rational::one(),
            Rational::new(-1),
            Rational::fraction(7, 4).unwrap(),
            Rational::fraction(3, 5).unwrap(),
            Rational::fraction(-7, 4).unwrap(),
            Rational::fraction(-3, 5).unwrap(),
        ] {
            assert_eq!(value.add_one(), value.clone() + Rational::one());
            assert_eq!(value.subtract_one(), value.clone() - Rational::one());
        }
    }

    #[test]
    fn word_sized_add_sub_matches_arbitrary_precision_fallback() {
        let left = Rational::fraction(-17, 30).unwrap();
        let right = Rational::fraction(11, 42).unwrap();
        assert_eq!(&left + &right, Rational::fraction(-32, 105).unwrap());
        assert_eq!(&left - &right, Rational::fraction(-29, 35).unwrap());

        let huge = Rational::from_bigint(BigInt::from(1_u8) << 200);
        assert_eq!(&huge + &right - &huge, right);
    }

    #[test]
    fn word_sized_mul_div_matches_arbitrary_precision_fallback() {
        let left = Rational::fraction(-17, 30).unwrap();
        let right = Rational::fraction(11, 42).unwrap();
        assert_eq!(&left * &right, Rational::fraction(-187, 1260).unwrap());
        assert_eq!(&left / &right, Rational::fraction(-119, 55).unwrap());

        let a = BigUint::one() << 80_usize;
        let b = &a - BigUint::one();
        let ratio = Rational::from_bigint_fraction(BigInt::from(a.clone()), b.clone()).unwrap();
        let reciprocal =
            Rational::from_bigint_fraction(BigInt::from(b), a).unwrap();
        assert_eq!(&ratio * &reciprocal, Rational::one());
        assert_eq!(&ratio / &ratio, Rational::one());

        let huge = Rational::from_bigint(BigInt::from(1_u8) << 200);
        assert_eq!(&huge * &right / &huge, right);
        assert_eq!(&huge * Rational::one(), huge);
        assert_eq!(&huge / &huge, Rational::one());

        let huge_ratio: Rational = format!("{}/3", BigInt::from(1_u8) << 200)
            .parse()
            .unwrap();
        let huge_reciprocal: Rational = format!("3/{}", BigInt::from(1_u8) << 200)
            .parse()
            .unwrap();
        assert_eq!(&huge_ratio * &huge_reciprocal, Rational::one());
    }

    #[test]
    fn magnitude_at_least_power_of_two_handles_threshold_boundaries() {
        assert!(
            !Rational::fraction(7, 1)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            Rational::fraction(8, 1)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            Rational::fraction(-9, 1)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            !Rational::fraction(15, 2)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(
            Rational::fraction(16, 2)
                .unwrap()
                .magnitude_at_least_power_of_two(3)
        );
        assert!(!Rational::zero().magnitude_at_least_power_of_two(3));
    }

    #[test]
    fn exact_msd_matches_f64_floor_for_small_rationals() {
        for numerator in 1..=128 {
            for denominator in 1..=128 {
                let rational = Rational::fraction(numerator, denominator).unwrap();
                let expected = ((numerator as f64) / (denominator as f64))
                    .log2()
                    .floor() as i32;
                assert_eq!(rational.msd_exact(), Some(expected));
            }
        }
    }

    #[test]
    fn exact_msd_handles_large_shift_boundaries_without_rounding() {
        let power = BigUint::one() << 4096usize;
        let exact = Rational::from_fraction_parts(Plus, power.clone(), BigUint::one());
        let below = Rational::from_fraction_parts(Plus, &power - BigUint::one(), BigUint::one());
        let reciprocal = Rational::from_fraction_parts(Plus, BigUint::one(), power);

        assert_eq!(exact.msd_exact(), Some(4096));
        assert_eq!(below.msd_exact(), Some(4095));
        assert_eq!(reciprocal.msd_exact(), Some(-4096));
    }

    #[test]
    fn dyadic_add_sub_stay_reduced() {
        let three_eighths = Rational::fraction(3, 8).unwrap();
        let five_sixteenths = Rational::fraction(5, 16).unwrap();

        assert_eq!(
            &three_eighths + &five_sixteenths,
            Rational::fraction(11, 16).unwrap()
        );
        assert_eq!(
            &three_eighths - &five_sixteenths,
            Rational::fraction(1, 16).unwrap()
        );
        assert_eq!(
            &five_sixteenths - &three_eighths,
            Rational::fraction(-1, 16).unwrap()
        );
        assert_eq!(&three_eighths - &three_eighths, Rational::zero());
    }

    #[test]
    fn integer_add_sub_preserves_reduced_fraction_denominator() {
        let three_eighths = Rational::fraction(3, 8).unwrap();
        let five = Rational::from(5_u8);
        let negative_five = Rational::from(-5_i8);

        for (actual, expected) in [
            (&three_eighths + &five, Rational::fraction(43, 8).unwrap()),
            (&five + &three_eighths, Rational::fraction(43, 8).unwrap()),
            (&three_eighths - &five, Rational::fraction(-37, 8).unwrap()),
            (&five - &three_eighths, Rational::fraction(37, 8).unwrap()),
            (
                &three_eighths + &negative_five,
                Rational::fraction(-37, 8).unwrap(),
            ),
            (
                &negative_five - &three_eighths,
                Rational::fraction(-43, 8).unwrap(),
            ),
        ] {
            assert_eq!(actual, expected);
            assert_eq!(actual.denominator(), &BigUint::from(8_u8));
        }
    }

    #[test]
    fn same_denominator_reports_reduced_common_scale() {
        let a = Rational::fraction(3, 10).unwrap();
        let b = Rational::fraction(-7, 10).unwrap();
        let reduced = Rational::fraction(6, 20).unwrap();
        let c = Rational::fraction(1, 3).unwrap();

        assert!(a.same_denominator(&b));
        assert!(a.same_denominator(&reduced));
        assert!(!a.same_denominator(&c));
    }

    #[test]
    fn dot_products_match_pairwise_arithmetic() {
        let left = [
            Rational::fraction(3, 8).unwrap(),
            Rational::fraction(-5, 16).unwrap(),
            Rational::zero(),
            Rational::fraction(7, 10).unwrap(),
        ];
        let right = [
            Rational::fraction(11, 32).unwrap(),
            Rational::fraction(13, 64).unwrap(),
            Rational::fraction(17, 19).unwrap(),
            Rational::fraction(-23, 25).unwrap(),
        ];
        let expected = &(&left[0] * &right[0])
            + &(&left[1] * &right[1])
            + &(&left[2] * &right[2])
            + &(&left[3] * &right[3]);

        assert_eq!(
            Rational::dot_products(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            ),
            expected
        );
    }

    #[test]
    fn dot_products_preserve_dyadic_exactness() {
        let left = [
            Rational::fraction(1, 8).unwrap(),
            Rational::fraction(3, 16).unwrap(),
            Rational::fraction(-5, 32).unwrap(),
        ];
        let right = [
            Rational::fraction(7, 4).unwrap(),
            Rational::fraction(-11, 8).unwrap(),
            Rational::fraction(13, 16).unwrap(),
        ];

        let dot = Rational::dot_products(
            [&left[0], &left[1], &left[2]],
            [&right[0], &right[1], &right[2]],
        );
        assert!(dot.is_dyadic());
        assert_eq!(
            dot,
            &(&left[0] * &right[0]) + &(&left[1] * &right[1]) + &(&left[2] * &right[2])
        );
    }

    #[test]
    fn dot_products_handle_equal_non_dyadic_denominators() {
        let left = [
            Rational::fraction(7, 10).unwrap(),
            Rational::fraction(-9, 10).unwrap(),
            Rational::fraction(11, 10).unwrap(),
        ];
        let right = [
            Rational::fraction(13, 7).unwrap(),
            Rational::fraction(5, 7).unwrap(),
            Rational::fraction(-3, 7).unwrap(),
        ];

        assert_eq!(
            Rational::dot_products(
                [&left[0], &left[1], &left[2]],
                [&right[0], &right[1], &right[2]],
            ),
            &(&left[0] * &right[0]) + &(&left[1] * &right[1]) + &(&left[2] * &right[2])
        );
    }

    #[test]
    fn signed_product_sum_matches_pairwise_arithmetic() {
        let terms = [
            [
                Rational::fraction(3, 8).unwrap(),
                Rational::fraction(-5, 12).unwrap(),
                Rational::fraction(7, 11).unwrap(),
            ],
            [
                Rational::fraction(13, 9).unwrap(),
                Rational::fraction(17, 25).unwrap(),
                Rational::fraction(-19, 6).unwrap(),
            ],
            [
                Rational::fraction(-23, 10).unwrap(),
                Rational::fraction(29, 14).unwrap(),
                Rational::fraction(31, 15).unwrap(),
            ],
        ];
        let expected = &(&terms[0][0] * &terms[0][1] * &terms[0][2])
            - &(&terms[1][0] * &terms[1][1] * &terms[1][2])
            + &(&terms[2][0] * &terms[2][1] * &terms[2][2]);

        assert_eq!(
            Rational::signed_product_sum(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1], &terms[0][2]],
                    [&terms[1][0], &terms[1][1], &terms[1][2]],
                    [&terms[2][0], &terms[2][1], &terms[2][2]],
                ],
            ),
            expected
        );
    }

    #[test]
    fn signed_product_sum_preserves_dyadic_exactness() {
        let terms = [
            [
                Rational::fraction(1, 8).unwrap(),
                Rational::fraction(3, 16).unwrap(),
            ],
            [
                Rational::fraction(5, 32).unwrap(),
                Rational::fraction(7, 64).unwrap(),
            ],
            [
                Rational::fraction(-9, 4).unwrap(),
                Rational::fraction(11, 8).unwrap(),
            ],
        ];
        let sum = Rational::signed_product_sum(
            [true, false, true],
            [
                [&terms[0][0], &terms[0][1]],
                [&terms[1][0], &terms[1][1]],
                [&terms[2][0], &terms[2][1]],
            ],
        );

        assert!(sum.is_dyadic());
        assert_eq!(
            sum,
            &(&terms[0][0] * &terms[0][1]) - &(&terms[1][0] * &terms[1][1])
                + &(&terms[2][0] * &terms[2][1])
        );
    }

    #[test]
    fn signed_product_sum_handles_equal_non_dyadic_denominators() {
        let terms = [
            [
                Rational::fraction(7, 10).unwrap(),
                Rational::fraction(13, 7).unwrap(),
            ],
            [
                Rational::fraction(9, 10).unwrap(),
                Rational::fraction(5, 7).unwrap(),
            ],
            [
                Rational::fraction(11, 10).unwrap(),
                Rational::fraction(3, 7).unwrap(),
            ],
        ];

        assert_eq!(
            Rational::signed_product_sum(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1]],
                    [&terms[1][0], &terms[1][1]],
                    [&terms[2][0], &terms[2][1]],
                ],
            ),
            &(&terms[0][0] * &terms[0][1]) - &(&terms[1][0] * &terms[1][1])
                + &(&terms[2][0] * &terms[2][1])
        );
    }

    #[test]
    fn signed_product_sum_word_overflow_falls_back_to_biguint() {
        let huge = Rational::from_bigint(BigInt::one() << 200_usize);
        let two = Rational::new(2);
        let three = Rational::new(3);
        let expected = &huge * &three - &huge * &two;

        assert_eq!(
            Rational::signed_product_sum(
                [true, false],
                [[&huge, &three], [&huge, &two]],
            ),
            expected
        );
    }

    #[test]
    fn signed_product_sum_cross_cancels_before_word_overflow() {
        let a = BigUint::one() << 80_usize;
        let b = &a - BigUint::one();
        let ratio = Rational::from_bigint_fraction(BigInt::from(a.clone()), b.clone()).unwrap();
        let reciprocal = Rational::from_bigint_fraction(BigInt::from(b), a).unwrap();

        assert_eq!(
            Rational::signed_product_sum([true], [[&ratio, &reciprocal]]),
            Rational::one(),
        );
    }

    #[test]
    fn signed_product_sum_ordering_matches_materialized_result_and_overflow_fallback() {
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        assert_eq!(
            Rational::signed_product_sum_ordering(
                [true, false],
                [[&three, &one], [&two, &one]],
            ),
            Ordering::Greater,
        );

        let huge = Rational::from_bigint(BigInt::from(1_u8) << 200);
        assert_eq!(
            Rational::signed_product_sum_ordering(
                [false, true],
                [[&huge, &three], [&huge, &two]],
            ),
            Ordering::Less,
        );

        let wide_a = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 220_usize) + BigUint::from(7_u8)),
            BigUint::from(15_u8),
        )
        .unwrap();
        let wide_b = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 180_usize) + BigUint::from(11_u8)),
            BigUint::from(77_u8),
        )
        .unwrap();
        let wide_c = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 160_usize) + BigUint::from(13_u8)),
            BigUint::from(143_u16),
        )
        .unwrap();
        let wide_d = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 140_usize) + BigUint::from(17_u8)),
            BigUint::from(221_u16),
        )
        .unwrap();
        let terms = [[&wide_a, &wide_b], [&wide_c, &wide_d], [&wide_b, &wide_d]];
        let signs = [true, false, true];
        let materialized = Rational::signed_product_sum(signs, terms);
        assert_eq!(
            Rational::signed_product_sum_ordering(signs, terms),
            materialized.partial_cmp(&Rational::zero()).unwrap(),
        );

        let dyadic_a = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 200_usize) + BigUint::one()),
            BigUint::one() << 127_usize,
        )
        .unwrap();
        let dyadic_b = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 170_usize) + BigUint::one()),
            BigUint::one() << 93_usize,
        )
        .unwrap();
        let dyadic_terms = [[&dyadic_a, &dyadic_b], [&dyadic_b, &dyadic_b]];
        let dyadic_signs = [false, true];
        let materialized = Rational::signed_product_sum(dyadic_signs, dyadic_terms);
        assert_eq!(
            Rational::signed_product_sum_ordering(dyadic_signs, dyadic_terms),
            materialized.partial_cmp(&Rational::zero()).unwrap(),
        );
    }

    #[test]
    fn signed_product_sum_shared_denominator_consumes_common_scale() {
        let terms = [
            [
                Rational::fraction(7, 15).unwrap(),
                Rational::fraction(13, 15).unwrap(),
            ],
            [
                Rational::fraction(8, 15).unwrap(),
                Rational::fraction(-2, 15).unwrap(),
            ],
            [
                Rational::fraction(11, 15).unwrap(),
                Rational::fraction(14, 15).unwrap(),
            ],
        ];
        let expected = &(&terms[0][0] * &terms[0][1]) - &(&terms[1][0] * &terms[1][1])
            + &(&terms[2][0] * &terms[2][1]);

        assert_eq!(
            Rational::signed_product_sum_shared_denominator(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1]],
                    [&terms[1][0], &terms[1][1]],
                    [&terms[2][0], &terms[2][1]],
                ],
            ),
            Some(expected)
        );

        let mixed = Rational::fraction(1, 7).unwrap();
        assert_eq!(
            Rational::signed_product_sum_shared_denominator(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1]],
                    [&terms[1][0], &mixed],
                    [&terms[2][0], &terms[2][1]],
                ],
            ),
            None
        );
    }

    #[test]
    fn compare() {
        assert!(Rational::one() > Rational::zero());
        assert!(Rational::new(5) > Rational::new(4));
        assert!(Rational::new(-10) < Rational::new(5));
        assert!(Rational::fraction(1, 4).unwrap() < Rational::fraction(1, 3).unwrap());
    }

    #[test]
    fn sign_queries_are_strict() {
        assert!(Rational::new(-1).is_negative());
        assert!(!Rational::new(-1).is_positive());
        assert!(!Rational::zero().is_negative());
        assert!(!Rational::zero().is_positive());
        assert!(!Rational::one().is_negative());
        assert!(Rational::one().is_positive());
        assert_eq!(Rational::one_ref(), &Rational::one());
        assert!(std::ptr::eq(Rational::one_ref(), Rational::one_ref()));
    }

    #[test]
    fn equality_cross_multiplies_unequal_denominators() {
        let half = Rational::fraction(1, 2).unwrap();
        let two_quarters = Rational::from_parts_raw(
            Plus,
            BigUint::from(2_u8),
            BigUint::from(4_u8),
        );
        let three_sixths = Rational::from_parts_raw(
            Plus,
            BigUint::from(3_u8),
            BigUint::from(6_u8),
        );
        assert_eq!(half, two_quarters);
        assert_eq!(two_quarters, three_sixths);
    }

    #[test]
    fn word_comparison_falls_back_exactly_when_cross_products_overflow() {
        use std::cmp::Ordering;

        let left = Rational::from_parts_raw(
            Plus,
            BigUint::from(u128::MAX),
            BigUint::from(3_u8),
        );
        let right = Rational::from_parts_raw(
            Plus,
            BigUint::from(u128::MAX - 1),
            BigUint::from(2_u8),
        );
        assert_ne!(left, right);
        assert_eq!(left.partial_cmp(&right), Some(Ordering::Less));
    }

    #[test]
    fn same() {
        use std::cmp::Ordering;

        assert_eq!(
            Rational::zero().partial_cmp(&Rational::zero()),
            Some(Ordering::Equal)
        );
        assert_eq!(
            Rational::one().partial_cmp(&Rational::one()),
            Some(Ordering::Equal)
        );
        assert_eq!(
            Rational::new(-10).partial_cmp(&Rational::new(-10)),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn dyadic_comparison_handles_unequal_denominators_without_cross_products() {
        use std::cmp::Ordering;

        let values = [
            Rational::fraction(-17, 32).unwrap(),
            Rational::fraction(-1, 2).unwrap(),
            Rational::fraction(3, 16).unwrap(),
            Rational::fraction(1, 2).unwrap(),
            Rational::fraction(17, 32).unwrap(),
        ];
        for left in &values {
            for right in &values {
                let cross_products = match left.sign.cmp(&right.sign) {
                    Ordering::Equal if left.sign == Plus => (&left.numerator
                        * &right.denominator)
                        .cmp(&(&right.numerator * &left.denominator)),
                    Ordering::Equal if left.sign == Minus => (&right.numerator
                        * &left.denominator)
                        .cmp(&(&left.numerator * &right.denominator)),
                    ordering => ordering,
                };
                assert_eq!(left.partial_cmp(right), Some(cross_products));
            }
        }

        let two_quarters = Rational::from_parts_raw(
            Plus,
            BigUint::from(2_u8),
            BigUint::from(4_u8),
        );
        assert_eq!(Rational::fraction(1, 2).unwrap(), two_quarters);
    }

    #[test]
    fn divide_by_zero() {
        let err = Rational::fraction(1, 0).unwrap_err();
        assert_eq!(err, Problem::DivideByZero);
        let zero = Rational::zero();
        let err = zero.inverse().unwrap_err();
        assert_eq!(err, Problem::DivideByZero);
    }

    #[test]
    fn operations_work_on_refs_on_rhs() {
        let a = Rational::new(2);
        let b = Rational::new(3);
        let c = Rational::new(6);
        assert_eq!(a.clone() * &b, c.clone());
        assert_eq!(c.clone() / &b, a.clone());
        assert_eq!(c.clone() - &a, Rational::new(4));
        assert_eq!(-&c, Rational::new(-6));
        assert_eq!(a.clone() + &b, Rational::new(5));
    }

    #[test]
    fn operations_work_on_refs() {
        let a = Rational::new(2);
        let b = Rational::new(3);
        let c = Rational::new(6);
        assert_eq!(&a * &b, c.clone());
        assert_eq!(&c / &b, a.clone());
        assert_eq!(&c - &a, Rational::new(4));
        assert_eq!(-&c, Rational::new(-6));
        assert_eq!(&a + &b, Rational::new(5));
    }
}
