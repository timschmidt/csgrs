impl Rational {
    const NTT_DIGIT_BITS: u32 = 15;
    const NTT_DIGIT_MASK: u64 = (1_u64 << Self::NTT_DIGIT_BITS) - 1;
    const NTT_MODULI: [(u64, u64); 2] = [(2_013_265_921, 31), (998_244_353, 3)];

    fn ntt_mod_pow(mut value: u64, mut exponent: u64, modulus: u64) -> u64 {
        let mut result = 1_u64;
        while exponent != 0 {
            if exponent & 1 == 1 {
                result = result * value % modulus;
            }
            value = value * value % modulus;
            exponent >>= 1;
        }
        result
    }

    fn ntt_transform(values: &mut [u64], inverse: bool, modulus: u64, primitive_root: u64) {
        let mut target = 0_usize;
        for index in 1..values.len() {
            let mut bit = values.len() >> 1;
            while target & bit != 0 {
                target ^= bit;
                bit >>= 1;
            }
            target ^= bit;
            if index < target {
                values.swap(index, target);
            }
        }

        let mut width = 2_usize;
        while width <= values.len() {
            let mut root = Self::ntt_mod_pow(
                primitive_root,
                (modulus - 1) / u64::try_from(width).expect("NTT width fits u64"),
                modulus,
            );
            if inverse {
                root = Self::ntt_mod_pow(root, modulus - 2, modulus);
            }
            for chunk in values.chunks_exact_mut(width) {
                let mut factor = 1_u64;
                for index in 0..width / 2 {
                    let even = chunk[index];
                    let odd = chunk[index + width / 2] * factor % modulus;
                    chunk[index] = if even + odd >= modulus {
                        even + odd - modulus
                    } else {
                        even + odd
                    };
                    chunk[index + width / 2] = if even >= odd {
                        even - odd
                    } else {
                        even + modulus - odd
                    };
                    factor = factor * root % modulus;
                }
            }
            width <<= 1;
        }

        if inverse {
            let inverse_length = Self::ntt_mod_pow(
                u64::try_from(values.len()).expect("NTT length fits u64"),
                modulus - 2,
                modulus,
            );
            for value in values {
                *value = *value * inverse_length % modulus;
            }
        }
    }

    fn ntt_digits(value: &BigUint) -> Vec<u32> {
        let mut digits = Vec::with_capacity(
            usize::try_from(value.bits())
                .expect("BigUint bit width fits usize")
                .div_ceil(Self::NTT_DIGIT_BITS as usize),
        );
        let mut accumulator = 0_u64;
        let mut accumulator_bits = 0_u32;
        for limb in value.to_u32_digits() {
            accumulator |= u64::from(limb) << accumulator_bits;
            accumulator_bits += u32::BITS;
            while accumulator_bits >= Self::NTT_DIGIT_BITS {
                digits.push((accumulator & Self::NTT_DIGIT_MASK) as u32);
                accumulator >>= Self::NTT_DIGIT_BITS;
                accumulator_bits -= Self::NTT_DIGIT_BITS;
            }
        }
        if accumulator_bits != 0 {
            digits.push(accumulator as u32);
        }
        while digits.last() == Some(&0) {
            digits.pop();
        }
        digits
    }

    fn ntt_convolution_mod(
        left: &[u32],
        right: &[u32],
        length: usize,
        modulus: u64,
        primitive_root: u64,
    ) -> Vec<u64> {
        let mut left_values = vec![0_u64; length];
        let mut right_values = vec![0_u64; length];
        left_values[..left.len()]
            .iter_mut()
            .zip(left)
            .for_each(|(target, source)| *target = u64::from(*source));
        right_values[..right.len()]
            .iter_mut()
            .zip(right)
            .for_each(|(target, source)| *target = u64::from(*source));
        Self::ntt_transform(&mut left_values, false, modulus, primitive_root);
        Self::ntt_transform(&mut right_values, false, modulus, primitive_root);
        for (left, right) in left_values.iter_mut().zip(right_values) {
            *left = *left * right % modulus;
        }
        Self::ntt_transform(&mut left_values, true, modulus, primitive_root);
        left_values
    }

    fn ntt_digits_to_biguint(digits: impl IntoIterator<Item = u32>) -> BigUint {
        let mut limbs = Vec::new();
        let mut accumulator = 0_u64;
        let mut accumulator_bits = 0_u32;
        for digit in digits {
            accumulator |= u64::from(digit) << accumulator_bits;
            accumulator_bits += Self::NTT_DIGIT_BITS;
            while accumulator_bits >= u32::BITS {
                limbs.push(accumulator as u32);
                accumulator >>= u32::BITS;
                accumulator_bits -= u32::BITS;
            }
        }
        if accumulator_bits != 0 {
            limbs.push(accumulator as u32);
        }
        BigUint::new(limbs)
    }

    /// Exact Rust-native FFT-family candidate using two NTT primes and CRT.
    #[doc(hidden)]
    pub fn multiply_magnitudes_ntt_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-ntt-crt"
        );
        Self::multiply_magnitudes_ntt(left, right)
    }

    fn multiply_magnitudes_ntt(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() || right.is_zero() {
            return BigUint::ZERO;
        }
        let left = Self::ntt_digits(left);
        let right = Self::ntt_digits(right);
        let coefficient_count = left.len() + right.len() - 1;
        let length = coefficient_count.next_power_of_two();
        assert!(length <= 1_usize << 23, "NTT operand exceeds supported transform length");
        let coefficient_bound = u128::from(Self::NTT_DIGIT_MASK).pow(2)
            * u128::try_from(left.len().min(right.len())).expect("digit count fits u128");
        let modulus_product = u128::from(Self::NTT_MODULI[0].0) * u128::from(Self::NTT_MODULI[1].0);
        assert!(coefficient_bound < modulus_product, "NTT CRT modulus product is insufficient");

        let first = Self::ntt_convolution_mod(
            &left,
            &right,
            length,
            Self::NTT_MODULI[0].0,
            Self::NTT_MODULI[0].1,
        );
        let second = Self::ntt_convolution_mod(
            &left,
            &right,
            length,
            Self::NTT_MODULI[1].0,
            Self::NTT_MODULI[1].1,
        );
        let first_modulus = Self::NTT_MODULI[0].0;
        let second_modulus = Self::NTT_MODULI[1].0;
        let inverse_first = Self::ntt_mod_pow(first_modulus % second_modulus, second_modulus - 2, second_modulus);
        let mut carry = 0_u128;
        let mut digits = Vec::with_capacity(coefficient_count + 8);
        for (&first, &second) in first.iter().zip(&second).take(coefficient_count) {
            let difference = if second >= first % second_modulus {
                second - first % second_modulus
            } else {
                second + second_modulus - first % second_modulus
            };
            let multiplier = u64::try_from(
                u128::from(difference) * u128::from(inverse_first) % u128::from(second_modulus),
            )
            .expect("CRT multiplier fits u64");
            let coefficient = u128::from(first) + u128::from(first_modulus) * u128::from(multiplier);
            let total = coefficient + carry;
            digits.push((total & u128::from(Self::NTT_DIGIT_MASK)) as u32);
            carry = total >> Self::NTT_DIGIT_BITS;
        }
        while carry != 0 {
            digits.push((carry & u128::from(Self::NTT_DIGIT_MASK)) as u32);
            carry >>= Self::NTT_DIGIT_BITS;
        }
        Self::ntt_digits_to_biguint(digits)
    }
}
