/// Exact integer multiplication candidate using modular transforms and CRT.
///
/// Split nonnegative magnitudes into little-endian base-2^15 digits. Zero-padded
/// radix-two transforms compute their convolution under two primes; Chinese
/// remainder reconstruction is unique because every integer coefficient is below
/// the product of those primes. Base-2^15 carry propagation then materializes the
/// exact BigUint product. No floating-point FFT or approximate reconstruction is
/// used, and no GMP representation is introduced.
///
/// This family is invoked directly by benchmarks/tests, not by the production
/// Toom/backend selector. It limits transform length to 2^23 and asserts the CRT
/// coefficient bound. Those checks panic rather than return typed exhaustion;
/// digit conversion precedes them, and there is no total-work/memory budget or
/// cancellation. The helpers rely on explicit transform/digit preconditions.
impl Rational {
    /// Bits per convolution digit: 15, giving base 32768.
    ///
    /// This keeps coefficient growth below the two-prime CRT range at admitted
    /// transform lengths and allows packing through a u64 accumulator.
    const NTT_DIGIT_BITS: u32 = 15;
    /// Low-15-bit mask used for digit extraction and carry reduction modulo 32768.
    const NTT_DIGIT_MASK: u64 = (1_u64 << Self::NTT_DIGIT_BITS) - 1;
    /// Two (prime modulus, primitive root) pairs for exact radix-two transforms.
    ///
    /// 2013265921=15*2^27+1 has root 31; 998244353=119*2^23+1 has root 3.
    /// Their common power-of-two transform capacity is 2^23. Reduced residue products
    /// fit u64, while CRT reconstruction uses u128. The modulus product is
    /// 2009731336725594113; primality/root order are assumptions of the transform,
    /// not checked during each multiplication.
    const NTT_MODULI: [(u64, u64); 2] = [(2_013_265_921, 31), (998_244_353, 3)];

    /// Compute a modular power by binary square-and-multiply using u64 products.
    ///
    /// Callers supply a positive prime modulus and reduced values small enough that
    /// products fit u64. The initial value is not reduced or validated. Exponent zero
    /// returns the literal 1 without applying the modulus; with a positive exponent,
    /// modulus zero panics and oversized values can overflow before reduction. The
    /// NTT constants keep all consumed products in range. No primality test is made.
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

    /// Replace residues in place with a radix-two transform or its normalized inverse.
    ///
    /// For a nonempty slice, require power-of-two length dividing modulus-1, a prime
    /// modulus, a primitive root, and entries reduced below the modulus. Residue
    /// products/sums must fit u64. Bit reversal precedes iterative butterfly stages;
    /// the forward transform uses positive root powers. The inverse uses reciprocal
    /// stage roots and multiplies by length^(modulus-2) modulo the prime.
    ///
    /// These preconditions are not validated. Bad lengths/roots can yield incorrect
    /// results, and arbitrary inputs can panic or overflow; an empty slice is a no-op
    /// under the valid modulus assumptions, not a validated transform. Supplied NTT
    /// primes and caller lengths <=2^23 satisfy the intended arithmetic/width bounds.
    /// No allocation occurs here, but work is O(n log n) without cancellation.
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

    /// Convert a magnitude to canonical little-endian base-32768 convolution digits.
    ///
    /// Pack its little-endian u32 limbs through a u64 accumulator, emit 15-bit chunks,
    /// then discard high zero digits. Zero becomes an empty vector. The pending-bit
    /// count stays below 15 before adding each limb, so the accumulator fits. Both the
    /// limb copy and output vector allocate; bit-width conversion expects the width to fit usize.
    /// This helper runs before the multiplication kernel checks transform capacity.
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

    /// Return a length-sized modular convolution using two forward transforms and one inverse.
    ///
    /// Copy input digits into separate zero-padded u64 vectors, multiply transformed
    /// residues pointwise, and inverse-transform the first buffer. Require the transform
    /// preconditions, digits reduced below the prime and length >= each input length.
    /// Too-short buffers panic while slicing. To obtain a linear rather than cyclic
    /// convolution, nonempty inputs additionally require length >= left.len()+right.len()-1.
    /// Padding is retained in the returned vector; the kernel consumes only real
    /// coefficient positions. Two length-sized residue buffers are allocated per call;
    /// there is no separate total-memory/work limit or validation of the field inputs.
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

    /// Pack little-endian base-32768 digits into u32 limbs and construct a BigUint.
    ///
    /// Each digit must be below 32768; no range check is performed. The accumulator
    /// uses bitwise OR, so oversized digits may overlap rather than propagate carries
    /// and are not a supported alternative representation. Empty/all-zero input gives
    /// zero, and high zero limbs are normalized by BigUint. Packing allocates a limb
    /// vector and supports only finite input iterators; no input-length limit is applied.
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

    /// Benchmark an exact two-prime NTT product of nonnegative integer magnitudes.
    ///
    /// Zero operands return zero. Otherwise the next power of two covering the two
    /// base-32768 digit lengths minus one must be <=2^23; exceeding that capacity or
    /// the CRT coefficient bound panics. Input-digit allocation occurs before these
    /// checks. Integer size conversions and allocation can also fail; there is no
    /// typed exhaustion, total-work/memory budget or cancellation interface.
    ///
    /// With dispatch-trace recording active, the wrapper records
    /// rational_algorithm/multiplication-candidate/rust-native-ntt-crt even for zero.
    /// It bypasses the production multiplication selector and always invokes this
    /// candidate. Successful output is fully materialized and exact; no numerical
    /// precision parameter, floating-point transform or retained recipe is involved.
    pub fn multiply_magnitudes_ntt_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        #[cfg(feature = "dispatch-trace")]
        crate::trace_dispatch!(
            "rational_algorithm",
            "multiplication-candidate",
            "rust-native-ntt-crt"
        );
        Self::multiply_magnitudes_ntt(left, right)
    }

    /// Materialize an exact product by two modular convolutions and bounded CRT lifting.
    ///
    /// Handle zero first, then convert both operands to digits. Let c=a.len()+b.len()-1
    /// and n=next_power_of_two(c). Assert n<=2^23 and
    /// 32767^2*min(a.len(),b.len()) < p1*p2 before allocating transform buffers.
    /// Length arithmetic and conversions are unchecked or expect-based; these are
    /// panic guards after input conversion, not typed resource admission.
    ///
    /// For residues r1,r2, compute t=(r2-r1)*inverse(p1) mod p2 and lift to
    /// r1+p1*t in [0,p1*p2). The asserted bound makes this the unique nonnegative
    /// integer convolution coefficient. Add the carried high bits, emit the low
    /// 15 bits, and propagate the remaining carry; pack the final digits as BigUint.
    /// With admitted lengths, min(a.len(),b.len())<=4194304, giving a coefficient
    /// bound of 4503324753657856, below the CRT product and well within u128.
    ///
    /// The first residue vector remains live during the second convolution; both
    /// full padded results are retained until reconstruction ends. No explicit memory
    /// accounting, cancellation or trace event is provided by this private kernel.
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
