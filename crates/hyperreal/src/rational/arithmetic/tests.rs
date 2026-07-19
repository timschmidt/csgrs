#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::size_of;

    #[test]
    fn rational_data_layout_stays_bounded() {
        assert!(
            size_of::<RationalData>() <= 96,
            "RationalData grew to {} bytes",
            size_of::<RationalData>()
        );
    }

    fn magnitude_with_backend_limbs(limbs: usize) -> BigUint {
        assert!(limbs > 0);
        BigUint::one() << ((limbs - 1) * usize::BITS as usize)
    }

    #[test]
    fn backend_multiplication_classifier_matches_locked_num_bigint_thresholds() {
        let limbs = magnitude_with_backend_limbs;
        assert_eq!(
            Rational::backend_multiplication_algorithm(&limbs(32), &limbs(400)),
            BackendMultiplicationAlgorithm::Basecase
        );
        assert_eq!(
            Rational::backend_multiplication_algorithm(&limbs(33), &limbs(66)),
            BackendMultiplicationAlgorithm::HalfKaratsuba
        );
        assert_eq!(
            Rational::backend_multiplication_algorithm(&limbs(33), &limbs(65)),
            BackendMultiplicationAlgorithm::Karatsuba
        );
        assert_eq!(
            Rational::backend_multiplication_algorithm(&limbs(256), &limbs(256)),
            BackendMultiplicationAlgorithm::Karatsuba
        );
        assert_eq!(
            Rational::backend_multiplication_algorithm(&limbs(257), &limbs(257)),
            BackendMultiplicationAlgorithm::Toom3
        );
    }

    #[test]
    fn rust_native_toom4_matches_backend_products() {
        let threshold = usize::try_from(Rational::TOOM4_MULTIPLICATION_THRESHOLD_BITS).unwrap();
        assert!(!Rational::should_use_toom4_multiplication(
            &(BigUint::one() << (threshold - 2)),
            &(BigUint::one() << (threshold - 2))
        ));
        assert!(Rational::should_use_toom4_multiplication(
            &(BigUint::one() << (threshold - 1)),
            &(BigUint::one() << (threshold - 1))
        ));
        assert!(!Rational::should_use_toom4_multiplication(
            &(BigUint::one() << (threshold + threshold / 2)),
            &(BigUint::one() << (threshold - 1))
        ));

        for left in 0_u32..128 {
            for right in 0_u32..128 {
                let left = BigUint::from(left);
                let right = BigUint::from(right);
                assert_eq!(
                    Rational::multiply_magnitudes_toom4_candidate(&left, &right),
                    &left * &right
                );
            }
        }

        fn generated_magnitude(bits: usize, state: &mut u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                *state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + *state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        let mut state = 0xa409_3822_299f_31d0_u64;
        for bits in [257, 1024, 4096, 16_384, 65_536] {
            for shorter_bits in [bits, bits - 1, bits * 3 / 4] {
                let left = generated_magnitude(bits, &mut state);
                let right = generated_magnitude(shorter_bits, &mut state);
                assert_eq!(
                    Rational::multiply_magnitudes_toom4_candidate(&left, &right),
                    &left * &right,
                    "failed for {bits}-by-{shorter_bits}-bit operands"
                );
            }
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_toom4_candidate_path_is_traced() {
        let left = (BigUint::one() << 4096_usize) + 17_u8;
        let right = (BigUint::one() << 4095_usize) + 19_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes_toom4_candidate(&left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "multiplication-candidate",
                "rust-native-toom4"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_toom4_selected_path_is_traced_at_crossover() {
        let shorter_bits = usize::try_from(Rational::TOOM4_MULTIPLICATION_THRESHOLD_BITS).unwrap();
        let longer_bits = shorter_bits + shorter_bits / 5;
        let left = (BigUint::one() << (longer_bits - 1)) + 17_u8;
        let right = (BigUint::one() << (shorter_bits - 1)) + 19_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes("toom4-selection-test", &left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "toom4-selection-test",
                "rust-native-toom4"
            ),
            1
        );
    }

    #[test]
    fn rust_native_toom6_matches_backend_products() {
        let threshold = usize::try_from(Rational::TOOM6_MULTIPLICATION_THRESHOLD_BITS).unwrap();
        assert!(!Rational::should_use_toom6_multiplication(
            &(BigUint::one() << (threshold - 2)),
            &(BigUint::one() << (threshold - 2))
        ));
        assert!(Rational::should_use_toom6_multiplication(
            &(BigUint::one() << (threshold - 1)),
            &(BigUint::one() << (threshold - 1))
        ));
        assert!(!Rational::should_use_toom6_multiplication(
            &(BigUint::one() << (threshold + threshold / 5)),
            &(BigUint::one() << (threshold - 1))
        ));

        for left in 0_u32..64 {
            for right in 0_u32..64 {
                let left = BigUint::from(left);
                let right = BigUint::from(right);
                assert_eq!(
                    Rational::multiply_magnitudes_toom6_candidate(&left, &right),
                    &left * &right
                );
            }
        }

        fn generated_magnitude(bits: usize, state: &mut u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                *state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + *state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        let mut state = 0x082e_fa98_ec4e_6c89_u64;
        for bits in [257, 1024, 4096, 65_536, 131_072] {
            for shorter_bits in [bits, bits - 1, bits * 5 / 6] {
                let left = generated_magnitude(bits, &mut state);
                let right = generated_magnitude(shorter_bits, &mut state);
                assert_eq!(
                    Rational::multiply_magnitudes_toom6_candidate(&left, &right),
                    &left * &right,
                    "failed for {bits}-by-{shorter_bits}-bit operands"
                );
            }
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_toom6_candidate_path_is_traced() {
        let left = (BigUint::one() << 4096_usize) + 23_u8;
        let right = (BigUint::one() << 4095_usize) + 29_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes_toom6_candidate(&left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "multiplication-candidate",
                "rust-native-toom6"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_toom6_selected_path_is_traced_at_crossover() {
        let shorter_bits = usize::try_from(Rational::TOOM6_MULTIPLICATION_THRESHOLD_BITS).unwrap();
        let longer_bits = shorter_bits + shorter_bits / 7;
        let left = (BigUint::one() << (longer_bits - 1)) + 31_u8;
        let right = (BigUint::one() << (shorter_bits - 1)) + 37_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes("toom6-selection-test", &left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "toom6-selection-test",
                "rust-native-toom6"
            ),
            1
        );
    }

    #[test]
    fn rust_native_toom8_matches_backend_products() {
        let threshold = usize::try_from(Rational::TOOM8_MULTIPLICATION_THRESHOLD_BITS).unwrap();
        assert!(!Rational::should_use_toom8_multiplication(
            &(BigUint::one() << (threshold - 2)),
            &(BigUint::one() << (threshold - 2))
        ));
        assert!(Rational::should_use_toom8_multiplication(
            &(BigUint::one() << (threshold - 1)),
            &(BigUint::one() << (threshold - 1))
        ));
        assert!(!Rational::should_use_toom8_multiplication(
            &(BigUint::one() << (threshold + threshold / 7)),
            &(BigUint::one() << (threshold - 1))
        ));

        for left in 0_u32..32 {
            for right in 0_u32..32 {
                let left = BigUint::from(left);
                let right = BigUint::from(right);
                assert_eq!(
                    Rational::multiply_magnitudes_toom8_candidate(&left, &right),
                    &left * &right
                );
            }
        }

        fn generated_magnitude(bits: usize, state: &mut u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                *state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + *state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        let mut state = 0x4528_21e6_38d0_1377_u64;
        for bits in [257, 1024, 4096, 65_536, 131_072] {
            for shorter_bits in [bits, bits - 1, bits * 7 / 8] {
                let left = generated_magnitude(bits, &mut state);
                let right = generated_magnitude(shorter_bits, &mut state);
                assert_eq!(
                    Rational::multiply_magnitudes_toom8_candidate(&left, &right),
                    &left * &right,
                    "failed for {bits}-by-{shorter_bits}-bit operands"
                );
            }
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_toom8_candidate_path_is_traced() {
        let left = (BigUint::one() << 4096_usize) + 41_u8;
        let right = (BigUint::one() << 4095_usize) + 43_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes_toom8_candidate(&left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "multiplication-candidate",
                "rust-native-toom8"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_toom8_selected_path_is_traced_at_crossover() {
        let bits = usize::try_from(Rational::TOOM8_MULTIPLICATION_THRESHOLD_BITS).unwrap();
        let left = (BigUint::one() << (bits - 1)) + 47_u8;
        let right = (BigUint::one() << (bits - 1)) + 53_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes("toom8-selection-test", &left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "toom8-selection-test",
                "rust-native-toom8"
            ),
            1
        );
    }

    #[test]
    fn rust_native_ntt_matches_backend_products() {
        for left in 0_u32..32 {
            for right in 0_u32..32 {
                let left = BigUint::from(left);
                let right = BigUint::from(right);
                assert_eq!(
                    Rational::multiply_magnitudes_ntt_candidate(&left, &right),
                    &left * &right
                );
            }
        }

        fn generated_magnitude(bits: usize, state: &mut u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                *state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + *state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        let mut state = 0xbe54_66cf_34e9_0c6c_u64;
        for bits in [1, 15, 16, 31, 32, 257, 4096, 65_536, 262_144] {
            let left = generated_magnitude(bits, &mut state);
            let right = generated_magnitude(bits.saturating_sub(1).max(1), &mut state);
            assert_eq!(
                Rational::multiply_magnitudes_ntt_candidate(&left, &right),
                &left * &right,
                "failed for {bits}-bit operands"
            );
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn rust_native_ntt_candidate_path_is_traced() {
        let left = (BigUint::one() << 4096_usize) + 59_u8;
        let right = (BigUint::one() << 4095_usize) + 61_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::multiply_magnitudes_ntt_candidate(&left, &right),
                &left * &right
            );
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "multiplication-candidate",
                "rust-native-ntt-crt"
            ),
            1
        );
    }

    #[test]
    fn backend_division_classifier_matches_locked_num_bigint_thresholds() {
        let limbs = magnitude_with_backend_limbs;
        assert_eq!(
            Rational::backend_division_algorithm(&limbs(2), &limbs(3)),
            BackendDivisionAlgorithm::TrivialOrSmallQuotient
        );
        assert_eq!(
            Rational::backend_division_algorithm(&limbs(10), &BigUint::from(3_u8)),
            BackendDivisionAlgorithm::SingleLimb
        );
        assert_eq!(
            Rational::backend_division_algorithm(&limbs(128), &limbs(64)),
            BackendDivisionAlgorithm::KnuthBasecase
        );
        assert_eq!(
            Rational::backend_division_algorithm(&limbs(129), &limbs(65)),
            BackendDivisionAlgorithm::KnuthBasecase
        );
    }

    #[test]
    fn block_wise_barrett_matches_backend_div_rem() {
        fn generated_magnitude(bits: usize, state: &mut u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                *state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + *state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        use num::Integer as _;
        for divisor in 1_u32..128 {
            let divisor = BigUint::from(divisor);
            for quotient in [0_u32, 1, 2, 7, 31, 127, 1021] {
                for remainder in [0_u32, 1, 3, 17, 63, 126] {
                    let remainder = BigUint::from(remainder) % &divisor;
                    let dividend = &divisor * quotient + remainder;
                    assert_eq!(
                        Rational::div_rem_magnitudes_barrett_candidate(&dividend, &divisor),
                        dividend.div_rem(&divisor)
                    );
                }
            }
        }

        let mut state = 0x243f_6a88_85a3_08d3_u64;
        for divisor_bits in [64, 65, 192, 1024, 4096] {
            for blocks in [1, 2, 3, 8] {
                for case in 0..8 {
                    let divisor = generated_magnitude(divisor_bits, &mut state);
                    let dividend_bits = divisor_bits * blocks + case;
                    let dividend = generated_magnitude(dividend_bits.max(1), &mut state);
                    assert_eq!(
                        Rational::div_rem_magnitudes_barrett_candidate(&dividend, &divisor),
                        dividend.div_rem(&divisor),
                        "failed for {divisor_bits}-bit divisor, {blocks} blocks, case {case}"
                    );
                }
            }
        }
    }

    #[test]
    fn block_wise_barrett_batch_reuses_one_reciprocal() {
        let divisor = (BigUint::one() << 1023_usize) + 0x9e37_79b9_u64;
        let dividends: Vec<_> = (1_u32..=16)
            .map(|index| {
                (&divisor << (usize::try_from(index).unwrap() * 257))
                    + &divisor * index
                    + (index - 1)
            })
            .collect();
        assert_eq!(
            Rational::div_rem_magnitudes_barrett_batch_candidate(&dividends, &divisor),
            Rational::div_rem_magnitudes_backend_batch(&dividends, &divisor)
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn block_wise_barrett_candidate_path_is_traced() {
        let divisor = (BigUint::one() << 1023_usize) + 17_u8;
        let dividend = (&divisor << 4096_usize) + 19_u8;
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            let _ = Rational::div_rem_magnitudes_barrett_candidate(&dividend, &divisor);
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "division-candidate",
                "block-wise-barrett"
            ),
            1
        );
    }

    #[test]
    fn backend_radix_classifier_matches_locked_num_bigint_threshold() {
        assert_eq!(
            Rational::backend_radix_output_algorithm(&magnitude_with_backend_limbs(31)),
            BackendRadixOutputAlgorithm::RepeatedSingleLimbDivision
        );
        assert_eq!(
            Rational::backend_radix_output_algorithm(&magnitude_with_backend_limbs(32)),
            BackendRadixOutputAlgorithm::DivideAndConquer
        );
    }

    #[test]
    fn divide_conquer_decimal_parser_matches_backend_reference() {
        let digits = "1234567890".repeat(1024);
        let expected = Rational::from_bigint(BigInt::from(
            BigUint::parse_bytes(digits.as_bytes(), 10).unwrap(),
        ));
        assert_eq!(digits.parse::<Rational>().unwrap(), expected);
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn retained_dyadic_fact_and_backend_algorithm_paths_are_traced() {
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            let non_dyadic = Rational::fraction(7, 15).unwrap();
            assert!(!non_dyadic.is_dyadic());
            assert!(!non_dyadic.is_dyadic());

            let left = Rational::from_bigint(BigInt::from_biguint(
                Plus,
                magnitude_with_backend_limbs(40) + 1_u8,
            ));
            let right = Rational::from_bigint(BigInt::from_biguint(
                Plus,
                magnitude_with_backend_limbs(40) + 3_u8,
            ));
            let _ = &left * &right;
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("rational", "retained-facts", "dyadic-learned"),
            1
        );
        assert_eq!(
            trace.path_count("rational", "retained-facts", "dyadic-hit"),
            3
        );
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "multiplication-wide-dyadic",
                "backend-karatsuba"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn exact_reduction_traces_single_limb_and_small_and_large_knuth_division() {
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            let single_limb_numerator = magnitude_with_backend_limbs(10) * 3_u8;
            assert_eq!(
                Rational::from_bigint_fraction(
                    BigInt::from(single_limb_numerator),
                    BigUint::from(3_u8),
                )
                .unwrap(),
                Rational::from_bigint(BigInt::from(magnitude_with_backend_limbs(10)))
            );

            let common = magnitude_with_backend_limbs(10) + 1_u8;
            assert_eq!(
                Rational::from_bigint_fraction(
                    BigInt::from(&common * 3_u8),
                    &common * 5_u8,
                )
                .unwrap(),
                Rational::fraction(3, 5).unwrap()
            );

            let wide_common = magnitude_with_backend_limbs(65) + 1_u8;
            let wide_factor = magnitude_with_backend_limbs(65) + 2_u8;
            let _ = Rational::from_bigint_fraction(
                BigInt::from(&wide_common * &wide_factor),
                &wide_common * 3_u8,
            )
            .unwrap();
        });
        let trace = crate::dispatch_trace::take_trace();
        assert!(
            trace.path_count(
                "rational_algorithm",
                "reduction-numerator",
                "backend-single-limb"
            )
                >= 1
        );
        assert!(
            trace.path_count(
                "rational_algorithm",
                "reduction-numerator",
                "backend-knuth-basecase"
            )
                >= 2
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn exact_fractional_remainder_traces_backend_division() {
        let denominator = magnitude_with_backend_limbs(65) + 3_u8;
        let numerator = (&denominator << (64 * usize::BITS as usize)) + 17_u8;
        let value = Rational::from_bigint_fraction(BigInt::from(numerator), denominator).unwrap();
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert!(!value.fract().is_integer());
        });
        assert_eq!(
            crate::dispatch_trace::take_trace().path_count(
                "rational_algorithm",
                "exact-fractional-remainder",
                "backend-knuth-basecase"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn radix_conversion_paths_are_traced_at_backend_crossover() {
        let large = Rational::from_bigint(BigInt::from_biguint(
            Plus,
            magnitude_with_backend_limbs(32) + 17_u8,
        ));
        let large_decimal = large.to_string();

        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(Rational::new(17).to_string(), "17");
            assert_eq!(large.to_string(), large_decimal);
            assert_eq!(large_decimal.parse::<Rational>().unwrap(), large);
            assert_eq!(format!("{:#.4}", Rational::fraction(1, 7).unwrap()), "0.1428");
        });
        let trace = crate::dispatch_trace::take_trace();
        assert!(
            trace.path_count(
                "rational_algorithm",
                "binary-to-radix",
                "backend-repeated-single-limb-division"
            )
                >= 1
        );
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "binary-to-radix",
                "backend-divide-and-conquer"
            ),
            1
        );
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "radix-to-binary",
                "backend-chunked-multiply-add"
            ),
            1
        );
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "binary-to-radix",
                "rational-repeated-digit-division"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn divide_conquer_radix_input_path_is_traced() {
        let digits = "3141592653".repeat(1024);
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            let parsed = digits.parse::<Rational>().unwrap();
            assert!(parsed.is_integer());
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "radix-to-binary",
                "divide-conquer-product-tree"
            ),
            1
        );
    }

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
    fn wide_magnitude_gcd_handles_balanced_and_wide_word_operands() {
        let common = (BigUint::one() << 192_usize) + BigUint::one();
        assert_eq!(
            Rational::gcd_magnitudes(&(&common * 17_u8), &(&common * 19_u8)),
            common
        );

        let word = BigUint::from(15_u8);
        let wide = ((BigUint::one() << 260_usize) + BigUint::from(2_u8)) * &word;
        assert_eq!(Rational::gcd_magnitudes(&wide, &word), word);
        assert_eq!(
            Rational::gcd_magnitudes(&BigUint::ZERO, &wide),
            wide
        );
    }

    #[test]
    fn lehmer_magnitude_gcd_matches_backend_reference() {
        fn generated_magnitude(bits: usize, state: &mut u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                *state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + *state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        let mut state = 0x243f_6a88_85a3_08d3_u64;
        for bits in [192, 193, 256, 1024, 1031, 2048, 4096] {
            for case in 0..24 {
                let common = BigUint::from((case % 7) + 1);
                let left = generated_magnitude(bits, &mut state) * &common;
                let right = generated_magnitude(bits - case % 2, &mut state) * &common;
                assert_eq!(
                    Rational::gcd_magnitudes(&left, &right),
                    num::Integer::gcd(&left, &right),
                    "failed for {bits}-bit case {case}"
                );
            }
        }
    }

    #[test]
    fn recursive_half_gcd_preserves_matrix_and_stop_invariants() {
        fn generated_magnitude(bits: usize, mut state: u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        for bits in [1024, 1025, 2048, 4096] {
            let left = generated_magnitude(bits, 0x243f_6a88_85a3_08d3);
            let right = generated_magnitude(bits, 0xa409_3822_299f_31d0);
            let reduction = Rational::half_gcd_reduce(&left, &right)
                .unwrap_or_else(|| panic!("half-GCD failed for {bits}-bit operands"));
            let matrix = &reduction.matrix;
            assert_eq!(
                &matrix.u00 * &reduction.left + &matrix.u01 * &reduction.right,
                left
            );
            assert_eq!(
                &matrix.u10 * &reduction.left + &matrix.u11 * &reduction.right,
                right
            );
            assert_eq!(
                &matrix.u00 * &matrix.u11 - &matrix.u01 * &matrix.u10,
                BigUint::one()
            );
            let stop_bits = left.bits().max(right.bits()) / 2 + 1;
            assert!(reduction.left.bits().min(reduction.right.bits()) > stop_bits);
            assert!(
                Rational::magnitude_difference_bits(&reduction.left, &reduction.right)
                    <= stop_bits,
                "{bits}-bit reduction stopped at {} difference bits with [{}, {}]-bit remainders (target {stop_bits})",
                Rational::magnitude_difference_bits(&reduction.left, &reduction.right),
                reduction.left.bits(),
                reduction.right.bits()
            );
            assert_eq!(
                num::Integer::gcd(&reduction.left, &reduction.right),
                num::Integer::gcd(&left, &right)
            );
        }
    }

    #[test]
    fn half_gcd_candidate_matches_lehmer_baseline() {
        fn generated_magnitude(bits: usize, mut state: u64) -> BigUint {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + state;
            }
            let mask = (BigUint::one() << bits) - 1_u8;
            (value & mask) | (BigUint::one() << (bits - 1))
        }

        for (bits, seed) in [(16_384, 1_u64), (16_391, 2), (32_768, 3)] {
            let left = generated_magnitude(bits, 0x243f_6a88_85a3_08d3 ^ seed);
            let right = generated_magnitude(bits, 0xa409_3822_299f_31d0 ^ seed);
            assert_eq!(
                Rational::gcd_magnitudes_half_gcd_candidate(&left, &right),
                Rational::gcd_magnitudes_lehmer_baseline(&left, &right),
                "failed for {bits}-bit operands"
            );
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn recursive_half_gcd_path_is_traced() {
        let bits = usize::try_from(Rational::HALF_GCD_THRESHOLD_BITS).unwrap();
        let generated_magnitude = |mut state: u64| {
            let mut value = BigUint::ZERO;
            for _ in 0..bits.div_ceil(64) {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                value = (value << 64_usize) + state;
            }
            value | (BigUint::one() << (bits - 1))
        };
        let left = generated_magnitude(0x243f_6a88_85a3_08d3);
        let right = generated_magnitude(0xa409_3822_299f_31d0);
        let expected = Rational::gcd_magnitudes_lehmer_baseline(&left, &right);

        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::gcd_magnitudes_half_gcd_candidate(&left, &right),
                expected
            );
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "gcd",
                "recursive-half-gcd"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn lehmer_magnitude_gcd_path_is_traced() {
        fn fibonacci_pair_at_least(bits: u64) -> (BigUint, BigUint) {
            let (mut previous, mut current) = (BigUint::one(), BigUint::one());
            while current.bits() < bits {
                (previous, current) = (current.clone(), previous + current);
            }
            (current, previous)
        }

        let (below, below_previous) =
            fibonacci_pair_at_least(Rational::LEHMER_GCD_THRESHOLD_BITS - 1);
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::gcd_magnitudes(&below, &below_previous),
                BigUint::one()
            );
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "gcd",
                "euclidean-wide-remainder"
            ),
            1
        );

        let (current, previous) =
            fibonacci_pair_at_least(Rational::LEHMER_GCD_THRESHOLD_BITS + 128);
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert_eq!(
                Rational::gcd_magnitudes(&current, &previous),
                BigUint::one()
            );
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "gcd",
                "lehmer-leading-limb"
            ),
            1
        );
    }

    #[test]
    fn clear_common_denominator_preserves_one_positive_shared_scale() {
        let values = [
            Rational::fraction(3, 8).unwrap(),
            Rational::fraction(-5, 12).unwrap(),
            Rational::zero(),
        ];
        assert_eq!(
            Rational::clear_common_denominator([&values[0], &values[1], &values[2]]),
            [Rational::new(9), Rational::new(-10), Rational::zero()]
        );
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

        // Decimal parsing may retain an unreduced internal fraction until a
        // later arithmetic operation. The dyadic/general path must reduce
        // those parts as well as cross-cancelling the operands.
        let decimal: Rational = "1.6".parse().unwrap();
        assert_eq!(Rational::fraction(5, 4).unwrap() * decimal, Rational::new(2));
        let odd_common_factor: Rational = "1.5".parse().unwrap();
        assert_eq!(
            Rational::fraction(5, 4).unwrap() * odd_common_factor,
            Rational::fraction(15, 8).unwrap()
        );
    }

    #[test]
    fn wide_dyadic_multiplication_cross_cancels_before_products() {
        let dyadic = Rational::from_bigint_fraction(
            BigInt::from(35_u8),
            BigUint::one() << 180_usize,
        )
        .unwrap();
        let general = Rational::from_bigint_fraction(
            BigInt::from(11_u8) << 150_usize,
            BigUint::from(21_u8),
        )
        .unwrap();
        let expected = Rational::from_bigint_fraction(
            BigInt::from(55_u8),
            BigUint::from(3_u8) << 30_usize,
        )
        .unwrap();
        assert_eq!(&dyadic * &general, expected);
        assert_eq!(&general * &dyadic, expected);
        assert_eq!((-&dyadic) * &general, -&expected);

        let unreduced_general = Rational::from_parts_raw(
            Plus,
            (BigUint::from(11_u8) << 150_usize) * BigUint::from(3_u8),
            BigUint::from(63_u8),
        );
        assert_eq!(&dyadic * unreduced_general, expected);

        let left = Rational::from_bigint_fraction(
            BigInt::from(3_u8),
            BigUint::one() << 200_usize,
        )
        .unwrap();
        let right = Rational::from_bigint_fraction(
            BigInt::from(5_u8) << 140_usize,
            BigUint::one() << 220_usize,
        )
        .unwrap();
        let expected = Rational::from_bigint_fraction(
            BigInt::from(15_u8),
            BigUint::one() << 280_usize,
        )
        .unwrap();
        assert_eq!(left * right, expected);

        let dyadic = Rational::from_bigint_fraction(
            BigInt::from(1_u8),
            BigUint::one() << 120_usize,
        )
        .unwrap();
        let odd_denominator = (BigUint::one() << 20_usize) + BigUint::one();
        let general = Rational::from_bigint_fraction(
            BigInt::from(1_u8),
            odd_denominator.clone(),
        )
        .unwrap();
        let expected = Rational::from_bigint_fraction(
            BigInt::from(1_u8),
            odd_denominator << 120_usize,
        )
        .unwrap();
        assert_eq!(dyadic * general, expected);
    }

    #[test]
    fn wide_dyadic_word_numerator_product_matches_biguint_reference() {
        let tiny = Rational::try_from(1.0e-12_f64).unwrap();
        assert!(tiny.denominator.bits() * 2 > u64::from(u128::BITS));
        assert!(tiny.numerator.to_u128().is_some());
        let negative = -tiny.clone();
        let product = &tiny * &negative;
        let expected = Rational::from_bigint_fraction(
            -BigInt::from_biguint(Plus, &tiny.numerator * &tiny.numerator),
            &tiny.denominator * &tiny.denominator,
        )
        .unwrap();
        assert_eq!(product, expected);
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
    fn perfect_square_residue_filter_never_rejects_a_square() {
        assert_eq!(
            Rational::SMALL_SQUARE_FACTORS
                .iter()
                .map(|(_, square)| square)
                .product::<u64>(),
            Rational::SMALL_SQUARE_PRODUCT
        );
        for root in 0_u64..=4096 {
            let root = BigUint::from(root);
            let square = &root * &root;
            assert!(Rational::could_be_perfect_square(&square));
            assert_eq!(Rational::try_perfect(&square), Some(root));
        }

        for nonsquare in [2_u64, 3, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 19, 23] {
            assert_eq!(Rational::try_perfect(&BigUint::from(nonsquare)), None);
        }
    }

    #[test]
    fn large_square_factor_schedule_preserves_canonical_residuals() {
        let base = (BigUint::one() << 80_usize) + BigUint::from(123_u8);
        for small_factor in [1_u64, 2, 3, 5, 7, 11, 13, 17] {
            let expected_root = &base * small_factor;
            for residual in [1_u64, 2, 3, 5, 6, 7, 10, 11, 13, 15, 17, 19] {
                let value = &expected_root * &expected_root * residual;
                let (root, rest) = Rational::extract_square(value);
                assert_eq!(root, expected_root);
                assert_eq!(rest, BigUint::from(residual));
            }
        }
    }

    #[test]
    fn large_power_of_two_square_extraction_splits_the_exponent() {
        for exponent in [64_usize, 65, 256, 257] {
            let value = BigUint::one() << exponent;
            let (root, rest) = Rational::extract_square(value.clone());
            assert_eq!(&root * &root * &rest, value);
            assert_eq!(rest, BigUint::from(if exponent.is_multiple_of(2) { 1_u8 } else { 2 }));
        }
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

        let dyadic = Rational::try_from(1.0e-12_f64).unwrap();
        let powered = dyadic.clone().powi(5_i32.into()).unwrap();
        let square = &dyadic * &dyadic;
        let fourth = &square * &square;
        assert_eq!(powered, fourth * dyadic);

        let negative = Rational::fraction(-7, 5).unwrap();
        assert_eq!(
            negative.clone().powi(3_i32.into()),
            Rational::fraction(-343, 125)
        );
        assert_eq!(
            negative.clone().powi(4_i32.into()),
            Rational::fraction(2_401, 625)
        );
        assert_eq!(
            negative.powi((-3_i32).into()),
            Rational::fraction(-125, 343)
        );
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
    fn repeated_negation_reuses_exact_storage_without_a_cycle() {
        let value = Rational::fraction(5_000_000_003, 7_000_000_009).unwrap();
        let owner = Arc::downgrade(&value.0);

        let first = -&value;
        let second = -&value;
        assert!(Arc::ptr_eq(&first.0, &second.0));

        drop(value);
        drop(first);
        drop(second);
        assert!(owner.upgrade().is_none());
    }

    #[test]
    fn shared_owned_negation_reuses_retained_storage() {
        let value = Rational::fraction(5_000_000_003, 7_000_000_009).unwrap();
        let shared = value.clone();

        let first = -value;
        let second = -shared;
        assert!(Arc::ptr_eq(&first.0, &second.0));
    }

    #[test]
    fn negation_first_cache_still_retains_inverse_and_two_linear_results() {
        let left = Rational::new(5_000_000_000);
        let _shared_left = left.clone();
        let negation = -&left;
        let inverse = left.clone().inverse().unwrap();
        let first_right = Rational::try_from(11.0e-9_f64).unwrap();
        let second_right = Rational::try_from(13.0e-9_f64).unwrap();

        let first = &left + &first_right;
        let second = &left + &second_right;
        let negation_reused = -&left;
        let inverse_reused = left.clone().inverse().unwrap();
        let first_reused = &left + &first_right;
        let second_reused = &left + &second_right;

        assert!(Arc::ptr_eq(&negation.0, &negation_reused.0));
        assert!(Arc::ptr_eq(&inverse.0, &inverse_reused.0));
        assert!(Arc::ptr_eq(&first.0, &first_reused.0));
        assert!(Arc::ptr_eq(&second.0, &second_reused.0));
    }

    #[test]
    fn unique_owned_negation_discards_results_for_the_old_sign() {
        let left = Rational::new(5_000_000_000);
        let right = Rational::fraction(1, 7).unwrap();

        let _cold = &left + &right;
        let retained = &left + &right;
        let reused = &left + &right;
        assert!(Arc::ptr_eq(&retained.0, &reused.0));

        let negated = -left;
        assert_eq!(&negated + &right, Rational::fraction(-34_999_999_999_i64, 7).unwrap());
    }

    #[test]
    fn repeated_small_power_reuses_the_retained_product_chain() {
        for exponent in 2..=5 {
            let base = Rational::fraction(5_000_000_003, 7_000_000_009).unwrap();

            let cold = base.clone().powi_i64(exponent).unwrap();
            let retained = base.clone().powi_i64(exponent).unwrap();
            let reused = base.powi_i64(exponent).unwrap();

            assert_eq!(cold, retained);
            assert!(Arc::ptr_eq(&retained.0, &reused.0));
        }
    }

    #[test]
    fn repeated_square_reduction_reuses_exact_factors_without_a_cycle() {
        let value = Rational::fraction(90_000_000_054_i64, 49_000_000_063).unwrap();
        let owner = Arc::downgrade(&value.0);

        let cold = value.clone().extract_square_reduced_retained();
        let retained = value.clone().extract_square_reduced_retained();
        let reused = value.clone().extract_square_reduced_retained();

        assert_eq!(cold, retained);
        assert!(Arc::ptr_eq(&retained.0.0, &reused.0.0));
        assert!(Arc::ptr_eq(&retained.1.0, &reused.1.0));

        drop(value);
        drop(cold);
        drop(retained);
        drop(reused);
        assert!(owner.upgrade().is_none());
    }

    #[test]
    fn square_reduction_first_cache_retains_both_unary_and_linear_pairs() {
        let left = Rational::new(5_000_000_000);
        let _shared_left = left.clone();
        let _cold_reduction = left.clone().extract_square_reduced_retained();
        let reduction = left.clone().extract_square_reduced_retained();
        let inverse = left.clone().inverse().unwrap();
        let negation = -&left;
        let first_right = Rational::try_from(11.0e-9_f64).unwrap();
        let second_right = Rational::try_from(13.0e-9_f64).unwrap();

        let first = &left + &first_right;
        let second = &left + &second_right;
        let reduction_reused = left.clone().extract_square_reduced_retained();
        let inverse_reused = left.clone().inverse().unwrap();
        let negation_reused = -&left;
        let first_reused = &left + &first_right;
        let second_reused = &left + &second_right;

        assert!(Arc::ptr_eq(&reduction.0.0, &reduction_reused.0.0));
        assert!(Arc::ptr_eq(&reduction.1.0, &reduction_reused.1.0));
        assert!(Arc::ptr_eq(&inverse.0, &inverse_reused.0));
        assert!(Arc::ptr_eq(&negation.0, &negation_reused.0));
        assert!(Arc::ptr_eq(&first.0, &first_reused.0));
        assert!(Arc::ptr_eq(&second.0, &second_reused.0));
    }

    #[test]
    fn small_dyadic_products_share_canonical_storage() {
        let left = Rational::fraction(5, 4).unwrap();
        let right = Rational::fraction(5, 2).unwrap();
        let first = &left * &right;
        let second = &left * &right;
        assert_eq!(first, Rational::fraction(25, 8).unwrap());
        assert!(Arc::ptr_eq(&first.0, &second.0));

        let negative = -Rational::fraction(11, 4).unwrap();
        let eighth = Rational::fraction(1, 8).unwrap();
        let first = &negative * &eighth;
        let second = &negative * &eighth;
        assert_eq!(first, Rational::fraction(-11, 32).unwrap());
        assert!(Arc::ptr_eq(&first.0, &second.0));
    }

    #[test]
    fn products_retain_exact_results_in_both_operand_orders() {
        let value = Rational::try_from(1.0e-12_f64).unwrap();
        let positive = &value * &value;
        let retained_positive = &value * &value;
        assert!(Arc::ptr_eq(&positive.0, &retained_positive.0));

        let left = Rational::new(1_000_000_000);
        let right = Rational::try_from(1.0e-9_f64).unwrap();
        let product = &left * &right;
        let retained = &left * &right;
        let reversed = &right * &left;
        assert!(Arc::ptr_eq(&product.0, &retained.0));
        assert!(Arc::ptr_eq(&product.0, &reversed.0));
    }

    #[test]
    fn linear_operations_retain_exact_results_without_competing_for_one_slot() {
        let left = Rational::new(1_000_000_000);
        let right = Rational::try_from(1.0e-9_f64).unwrap();
        let _retained_left = left.clone();
        let _retained_right = right.clone();

        let sum = &left + &right;
        let retained_sum = &left + &right;
        let reversed_sum = &right + &left;
        assert!(Arc::ptr_eq(&sum.0, &retained_sum.0));
        assert!(Arc::ptr_eq(&sum.0, &reversed_sum.0));

        let difference = &left - &right;
        let retained_difference = &left - &right;
        assert!(Arc::ptr_eq(&difference.0, &retained_difference.0));

        let product = &left * &right;
        let retained_product = &left * &right;
        assert!(Arc::ptr_eq(&product.0, &retained_product.0));
    }

    #[test]
    fn borrowed_linear_operations_retain_only_after_reuse_evidence() {
        let left = Rational::new(1_000_000_000);
        let right = Rational::try_from(1.0e-9_f64).unwrap();

        let cold_sum = &left + &right;
        let retained_sum = &left + &right;
        let reused_sum = &left + &right;
        assert_eq!(cold_sum, retained_sum);
        assert!(!Arc::ptr_eq(&cold_sum.0, &retained_sum.0));
        assert!(Arc::ptr_eq(&retained_sum.0, &reused_sum.0));

        let difference_left = Rational::new(2_000_000_000);
        let difference_right = Rational::try_from(3.0e-9_f64).unwrap();
        let cold_difference = &difference_left - &difference_right;
        let retained_difference = &difference_left - &difference_right;
        let reused_difference = &difference_left - &difference_right;
        assert_eq!(cold_difference, retained_difference);
        assert!(!Arc::ptr_eq(
            &cold_difference.0,
            &retained_difference.0
        ));
        assert!(Arc::ptr_eq(
            &retained_difference.0,
            &reused_difference.0
        ));
    }

    #[test]
    fn shared_right_operand_can_retain_a_directed_difference() {
        let left = Rational::new(3_000_000_000);
        let right = Rational::try_from(7.0e-9_f64).unwrap();
        let _shared_right = right.clone();

        let difference = &left - &right;
        let reused = &left - &right;
        assert!(Arc::ptr_eq(&difference.0, &reused.0));
    }

    #[test]
    fn shared_operand_retains_two_distinct_linear_results() {
        let left = Rational::new(5_000_000_000);
        let _shared_left = left.clone();
        let first_right = Rational::try_from(11.0e-9_f64).unwrap();
        let second_right = Rational::try_from(13.0e-9_f64).unwrap();

        let first = &left + &first_right;
        let second = &left + &second_right;
        let first_reused = &left + &first_right;
        let second_reused = &left + &second_right;

        assert!(Arc::ptr_eq(&first.0, &first_reused.0));
        assert!(Arc::ptr_eq(&second.0, &second_reused.0));
    }

    #[test]
    fn shared_inverse_is_retained_with_weak_reverse_link() {
        let value = Rational::fraction(5_000_000_003, 7_000_000_009).unwrap();
        let shared = value.clone();

        let inverse = value.clone().inverse().unwrap();
        let reused = shared.clone().inverse().unwrap();
        assert!(Arc::ptr_eq(&inverse.0, &reused.0));

        let reversed = inverse.inverse().unwrap();
        assert!(Arc::ptr_eq(&value.0, &reversed.0));

        let owner = Arc::downgrade(&value.0);
        drop(value);
        drop(shared);
        drop(reversed);
        assert!(owner.upgrade().is_none());
    }

    #[test]
    fn inverse_first_cache_still_retains_two_linear_results() {
        let left = Rational::new(5_000_000_000);
        let _shared_left = left.clone();
        let _inverse = left.clone().inverse().unwrap();
        let first_right = Rational::try_from(11.0e-9_f64).unwrap();
        let second_right = Rational::try_from(13.0e-9_f64).unwrap();

        let first = &left + &first_right;
        let second = &left + &second_right;
        let first_reused = &left + &first_right;
        let second_reused = &left + &second_right;

        assert!(Arc::ptr_eq(&first.0, &first_reused.0));
        assert!(Arc::ptr_eq(&second.0, &second_reused.0));
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
    fn general_perfect_power_detection_matches_integer_definition() {
        for value in [
            Rational::zero(),
            Rational::one(),
            Rational::new(-1),
            Rational::new(64),
            Rational::new(729),
            Rational::new(-27),
            Rational::new(-512),
            Rational::fraction(8, 27).unwrap(),
            Rational::fraction(101_i64.pow(7), 103_u64.pow(7)).unwrap(),
        ] {
            assert!(value.is_perfect_power(), "expected {value} to be a perfect power");
        }
        for value in [
            Rational::new(2),
            Rational::new(12),
            Rational::new(-16),
            Rational::fraction(4, 8).unwrap(),
            Rational::fraction(101_i64.pow(7), 103_u64.pow(5)).unwrap(),
        ] {
            assert!(!value.is_perfect_power(), "expected {value} not to be a perfect power");
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn general_perfect_power_paths_are_traced() {
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            assert!(!Rational::new(12).is_perfect_power());
            assert!(
                Rational::fraction(101_i64.pow(7), 103_u64.pow(7))
                    .unwrap()
                    .is_perfect_power()
            );
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "perfect-power",
                "factor-multiplicity-reject"
            ),
            1
        );
        assert_eq!(
            trace.path_count(
                "rational_algorithm",
                "perfect-power",
                "prime-root-exact"
            ),
            1
        );
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
    fn paired_complex_product_matches_independent_exact_arithmetic() {
        let cases = [
            [
                Rational::fraction(3, 7).unwrap(),
                Rational::fraction(-5, 11).unwrap(),
                Rational::fraction(13, 17).unwrap(),
                Rational::fraction(19, 23).unwrap(),
            ],
            [
                Rational::try_from(3.25_f64).unwrap(),
                Rational::try_from(-2.125_f64).unwrap(),
                Rational::try_from(1.75_f64).unwrap(),
                Rational::try_from(0.625_f64).unwrap(),
            ],
        ];

        for [a, b, c, d] in &cases {
            let (re, im) = Rational::complex_product_components([a, b], [c, d]);
            assert_eq!(re, a * c - b * d);
            assert_eq!(im, a * d + b * c);
        }

        let wide = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 170_usize) + BigUint::from(3_u8)),
            (BigUint::one() << 149_usize) + BigUint::one(),
        )
        .unwrap();
        let (re, im) = Rational::complex_product_components(
            [&wide, &Rational::one()],
            [&wide, &Rational::minus_one()],
        );
        assert_eq!(re, &wide * &wide + Rational::one());
        assert_eq!(im, Rational::zero());
    }

    #[test]
    fn paired_complex_quotient_matches_independent_exact_arithmetic() {
        let cases = [
            [
                Rational::fraction(3, 7).unwrap(),
                Rational::fraction(-5, 11).unwrap(),
                Rational::fraction(13, 17).unwrap(),
                Rational::fraction(19, 23).unwrap(),
            ],
            [
                Rational::try_from(1.0e-9_f64).unwrap(),
                Rational::try_from(-2.0e-9_f64).unwrap(),
                Rational::try_from(-1.0e-9_f64).unwrap(),
                Rational::try_from(2.0e-9_f64).unwrap(),
            ],
        ];
        for [a, b, c, d] in &cases {
            let denominator = c * c + d * d;
            let (re, im) = Rational::complex_quotient_components([a, b], [c, d]).unwrap();
            assert_eq!(re, (a * c + b * d) / &denominator);
            assert_eq!(im, (b * c - a * d) / &denominator);
        }

        let wide = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 170_usize) + BigUint::from(3_u8)),
            (BigUint::one() << 149_usize) + BigUint::one(),
        )
        .unwrap();
        let wide_other = Rational::from_bigint_fraction(
            BigInt::from((BigUint::one() << 163_usize) + BigUint::from(5_u8)),
            (BigUint::one() << 137_usize) + BigUint::from(3_u8),
        )
        .unwrap();
        let denominator = &wide_other * &wide_other + Rational::one();
        let (re, im) = Rational::complex_quotient_components(
            [&wide, &Rational::minus_one()],
            [&wide_other, &Rational::one()],
        )
        .unwrap();
        assert_eq!(
            re,
            (&wide * &wide_other - Rational::one()) / &denominator
        );
        assert_eq!(
            im,
            (-&wide_other - &wide) / &denominator
        );

        let a = &cases[0][0];
        let b = &cases[0][1];

        assert_eq!(
            Rational::complex_quotient_components(
                [a, b],
                [&Rational::zero(), &Rational::zero()],
            ),
            Err(crate::Problem::DivideByZero),
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
