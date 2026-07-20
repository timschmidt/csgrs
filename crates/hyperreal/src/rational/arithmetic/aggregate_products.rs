#[derive(Clone, Debug)]
struct HalfGcdMatrix {
    u00: BigUint,
    u01: BigUint,
    u10: BigUint,
    u11: BigUint,
}

impl HalfGcdMatrix {
    fn identity() -> Self {
        Self {
            u00: BigUint::one(),
            u01: BigUint::ZERO,
            u10: BigUint::ZERO,
            u11: BigUint::one(),
        }
    }

    fn apply_inverse(&self, left: &BigUint, right: &BigUint) -> Option<(BigUint, BigUint)> {
        let first_positive =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u11, left);
        let first_negative =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u01, right);
        let second_positive =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u00, right);
        let second_negative =
            Rational::multiply_magnitudes("half-gcd-matrix-apply", &self.u10, left);
        if first_positive < first_negative || second_positive < second_negative {
            return None;
        }
        Some((
            first_positive - first_negative,
            second_positive - second_negative,
        ))
    }

    fn multiply_right(&mut self, right: &Self) {
        let u00 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u00, &right.u00)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u01,
                &right.u10,
            );
        let u01 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u00, &right.u01)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u01,
                &right.u11,
            );
        let u10 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u10, &right.u00)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u11,
                &right.u10,
            );
        let u11 = Rational::multiply_magnitudes("half-gcd-matrix-compose", &self.u10, &right.u01)
            + Rational::multiply_magnitudes(
                "half-gcd-matrix-compose",
                &self.u11,
                &right.u11,
            );
        (self.u00, self.u01, self.u10, self.u11) = (u00, u01, u10, u11);
    }

    fn update_left_column(&mut self, quotient: &BigUint) {
        self.u00 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u01,
            quotient,
        );
        self.u10 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u11,
            quotient,
        );
    }

    fn update_right_column(&mut self, quotient: &BigUint) {
        self.u01 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u00,
            quotient,
        );
        self.u11 += Rational::multiply_magnitudes(
            "half-gcd-matrix-column-update",
            &self.u10,
            quotient,
        );
    }
}

struct HalfGcdReduction {
    left: BigUint,
    right: BigUint,
    matrix: HalfGcdMatrix,
}

struct DyadicProductSumPlan<const TERMS: usize> {
    denominator_shifts: [u64; TERMS],
    max_shift: u64,
    prefer_wide: bool,
}

impl Rational {
    /// Use one full-width remainder when exactly one operand fits a native word.
    ///
    /// `BigUint`'s binary GCD remains faster for word/word and balanced wide
    /// operands. Mixed-width rational reductions are different: reducing the
    /// wide value modulo the word once avoids a long subtraction/shift chain.
    pub(crate) fn gcd_magnitudes_with_mixed_width_fast_path(
        left: &BigUint,
        right: &BigUint,
    ) -> BigUint {
        match (left.to_u128(), right.to_u128()) {
            (Some(_), Some(_)) => num::Integer::gcd(left, right),
            (Some(0), None) => right.clone(),
            (None, Some(0)) => left.clone(),
            (Some(word), None) => {
                let remainder = (right % left)
                    .to_u128()
                    .expect("remainder is smaller than a u128 divisor");
                BigUint::from(Self::gcd_word(word, remainder))
            }
            (None, Some(word)) => {
                let remainder = (left % right)
                    .to_u128()
                    .expect("remainder is smaller than a u128 divisor");
                BigUint::from(Self::gcd_word(word, remainder))
            }
            (None, None) => num::Integer::gcd(left, right),
        }
    }

    // Tuned below after the Lehmer path is benchmarked against the full-width
    // Euclidean remainder loop. Keeping the boundary explicit lets the trace
    // and crossover tests describe the selected algorithm rather than merely
    // the operand size.
    const LEHMER_GCD_THRESHOLD_BITS: u64 = 192;
    const HALF_GCD_RECURSION_BASE_BITS: u64 = 1024;
    const HALF_GCD_THRESHOLD_BITS: u64 = 16_384;

    const POWERS_OF_FIVE: [u128; 56] = {
        let mut powers = [1_u128; 56];
        let mut index = 1;
        while index < powers.len() {
            powers[index] = powers[index - 1] * 5;
            index += 1;
        }
        powers
    };

    #[inline]
    fn gcd_u64(left: u64, right: u64) -> u64 {
        if left == 0 {
            return right;
        }
        if right == 0 {
            return left;
        }

        let common_shift = left.trailing_zeros().min(right.trailing_zeros());
        let mut left = left >> left.trailing_zeros();
        let mut right = right;
        loop {
            right >>= right.trailing_zeros();
            if left > right {
                std::mem::swap(&mut left, &mut right);
            }
            right -= left;
            if right == 0 {
                return left << common_shift;
            }
        }
    }

    fn gcd_word(left: u128, right: u128) -> u128 {
        if left == 0 {
            return right;
        }
        if right == 0 {
            return left;
        }
        if left <= u128::from(u64::MAX) && right <= u128::from(u64::MAX) {
            return u128::from(Self::gcd_u64(left as u64, right as u64));
        }

        // u128 remainder is a compiler-rt software call on common 64-bit
        // targets. Word-sized exact arithmetic reaches this helper heavily for
        // imported binary floats, so use Stein's binary GCD: it needs only
        // trailing-zero counts, shifts, comparisons, and subtraction.
        let common_shift = left.trailing_zeros().min(right.trailing_zeros());
        let mut left = left >> left.trailing_zeros();
        let mut right = right;
        loop {
            right >>= right.trailing_zeros();
            if left > right {
                std::mem::swap(&mut left, &mut right);
            }
            right -= left;
            if right == 0 {
                return left << common_shift;
            }
        }
    }

    /// Compute an arbitrary-precision GCD without entering `BigUint`'s
    /// subtraction-heavy binary reducer for balanced wide operands.
    ///
    /// Exact binary-float matrix inversion repeatedly cross-cancels a dyadic
    /// cofactor numerator against the same odd determinant. Those operands are
    /// commonly only two or three machine words wide, where Euclidean
    /// remainder steps are substantially cheaper than long runs of shifts and
    /// subtractions. Preserve the tuned binary reducer for values that fit the
    /// scalar word path, and reduce a wide/small pair to that path after one
    /// remainder.
    fn lehmer_gcd_matrix(larger: &BigUint, smaller: &BigUint) -> Option<[i128; 4]> {
        debug_assert!(larger >= smaller);
        let shift = larger.bits().saturating_sub(62);
        let mut high_larger = (larger >> usize::try_from(shift).ok()?).to_i128()?;
        let mut high_smaller = (smaller >> usize::try_from(shift).ok()?).to_i128()?;
        if high_smaller == 0 {
            return None;
        }

        // The matrix maps the original pair to consecutive Euclidean
        // remainders. Two quotient estimates must agree at both interval
        // endpoints before a step is retained; this is Lehmer's guard against
        // a quotient depending on the discarded low limbs.
        let (mut a, mut b, mut c, mut d) = (1_i128, 0_i128, 0_i128, 1_i128);
        let mut steps = 0_u8;
        while let Some(numerator_low) = high_larger.checked_add(a) {
            let Some(numerator_high) = high_larger.checked_add(b) else {
                break;
            };
            let Some(denominator_low) = high_smaller.checked_add(c) else {
                break;
            };
            let Some(denominator_high) = high_smaller.checked_add(d) else {
                break;
            };
            if numerator_low < 0
                || numerator_high < 0
                || denominator_low <= 0
                || denominator_high <= 0
            {
                break;
            }
            let quotient = numerator_low / denominator_low;
            if quotient == 0 || quotient != numerator_high / denominator_high {
                break;
            }

            let Some(next_c) = a.checked_sub(quotient.checked_mul(c)?) else {
                break;
            };
            let Some(next_d) = b.checked_sub(quotient.checked_mul(d)?) else {
                break;
            };
            let Some(next_high_smaller) =
                high_larger.checked_sub(quotient.checked_mul(high_smaller)?)
            else {
                break;
            };
            if next_high_smaller < 0 {
                break;
            }

            // Scalar multiplication by coefficients larger than one machine
            // word loses the property that makes a Lehmer batch cheap.
            if [c, d, next_c, next_d]
                .into_iter()
                .any(|value| value.unsigned_abs() > u128::from(u64::MAX))
            {
                break;
            }

            (a, b, c, d) = (c, d, next_c, next_d);
            (high_larger, high_smaller) = (high_smaller, next_high_smaller);
            steps += 1;
        }

        (steps >= 2).then_some([a, b, c, d])
    }

    fn apply_lehmer_gcd_matrix(
        larger: &BigUint,
        smaller: &BigUint,
        [a, b, c, d]: [i128; 4],
    ) -> Option<(BigUint, BigUint)> {
        let larger_signed = BigInt::from(larger.clone());
        let smaller_signed = BigInt::from(smaller.clone());
        let first = &larger_signed * a + &smaller_signed * b;
        let second = larger_signed * c + smaller_signed * d;
        let first = first.magnitude().clone();
        let second = second.magnitude().clone();
        if &first >= larger || &second >= larger {
            return None;
        }
        Some((first, second))
    }

    fn magnitude_difference_bits(left: &BigUint, right: &BigUint) -> u64 {
        if left >= right {
            (left - right).bits()
        } else {
            (right - left).bits()
        }
    }

    fn half_gcd_sdiv_step(
        left: &mut BigUint,
        right: &mut BigUint,
        stop_bits: u64,
        matrix: &mut HalfGcdMatrix,
    ) -> Option<()> {
        if left == right {
            return None;
        }
        if left > right {
            let mut quotient = &*left / &*right;
            let mut remainder = &*left
                - Self::multiply_magnitudes("half-gcd-sdiv-product", &quotient, right);
            if remainder.bits() <= stop_bits {
                quotient -= 1_u8;
                remainder += &*right;
            }
            if quotient.is_zero() || remainder >= *left {
                return None;
            }
            *left = remainder;
            matrix.update_right_column(&quotient);
        } else {
            let mut quotient = &*right / &*left;
            let mut remainder = &*right
                - Self::multiply_magnitudes("half-gcd-sdiv-product", &quotient, left);
            if remainder.bits() <= stop_bits {
                quotient -= 1_u8;
                remainder += &*left;
            }
            if quotient.is_zero() || remainder >= *right {
                return None;
            }
            *right = remainder;
            matrix.update_left_column(&quotient);
        }
        Some(())
    }

    fn half_gcd_slow(left: &BigUint, right: &BigUint) -> Option<HalfGcdReduction> {
        let stop_bits = left.bits().max(right.bits()) / 2 + 1;
        let mut left = left.clone();
        let mut right = right.clone();
        let mut matrix = HalfGcdMatrix::identity();
        while Self::magnitude_difference_bits(&left, &right) > stop_bits {
            Self::half_gcd_sdiv_step(&mut left, &mut right, stop_bits, &mut matrix)?;
        }
        Some(HalfGcdReduction {
            left,
            right,
            matrix,
        })
    }

    fn half_gcd_reduce(left: &BigUint, right: &BigUint) -> Option<HalfGcdReduction> {
        let input_bits = left.bits().max(right.bits());
        if input_bits <= Self::HALF_GCD_RECURSION_BASE_BITS {
            return Self::half_gcd_slow(left, right);
        }

        let stop_bits = input_bits / 2 + 1;
        let three_quarter_bits = input_bits.saturating_mul(3) / 4;
        let first_shift = input_bits / 2;
        let first_shift_usize = usize::try_from(first_shift).ok()?;
        let high_left = left >> first_shift_usize;
        let high_right = right >> first_shift_usize;
        let first = Self::half_gcd_reduce(&high_left, &high_right)?;
        let mut matrix = first.matrix;
        let (mut reduced_left, mut reduced_right) = matrix.apply_inverse(left, right)?;

        while reduced_left.bits().max(reduced_right.bits()) > three_quarter_bits + 1
            && Self::magnitude_difference_bits(&reduced_left, &reduced_right) > stop_bits
        {
            Self::half_gcd_sdiv_step(
                &mut reduced_left,
                &mut reduced_right,
                stop_bits,
                &mut matrix,
            )?;
        }

        let reduced_bits = reduced_left.bits().max(reduced_right.bits());
        if reduced_bits > stop_bits + 2 {
            let second_shift = stop_bits
                .saturating_mul(2)
                .checked_sub(reduced_bits)?
                .checked_add(1)?;
            if second_shift == 0 {
                return Self::half_gcd_slow(left, right);
            }
            let second_shift_usize = usize::try_from(second_shift).ok()?;
            let high_left = &reduced_left >> second_shift_usize;
            let high_right = &reduced_right >> second_shift_usize;
            if high_left.bits().max(high_right.bits()) >= input_bits {
                return Self::half_gcd_slow(left, right);
            }
            let second = Self::half_gcd_reduce(&high_left, &high_right)?;
            let (next_left, next_right) = second
                .matrix
                .apply_inverse(&reduced_left, &reduced_right)?;
            reduced_left = next_left;
            reduced_right = next_right;
            matrix.multiply_right(&second.matrix);
        }

        while Self::magnitude_difference_bits(&reduced_left, &reduced_right) > stop_bits {
            Self::half_gcd_sdiv_step(
                &mut reduced_left,
                &mut reduced_right,
                stop_bits,
                &mut matrix,
            )?;
        }
        Some(HalfGcdReduction {
            left: reduced_left,
            right: reduced_right,
            matrix,
        })
    }

    /// Exact magnitude GCD used by rational cross-cancellation.
    ///
    /// This is public only so the benchmark harness can compare the selected
    /// implementation with an otherwise identical full-width Euclidean loop.
    #[doc(hidden)]
    pub fn gcd_magnitudes(left: &BigUint, right: &BigUint) -> BigUint {
        let (divisor, algorithm) = if left.is_zero() {
            (right.clone(), "binary-word")
        } else if right.is_zero() {
            (left.clone(), "binary-word")
        } else if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            (
                BigUint::from(Self::gcd_word(left, right)),
                "binary-word",
            )
        } else {
            Self::gcd_wide_magnitudes(left, right, false)
        };

        #[cfg(feature = "dispatch-trace")]
        {
            crate::trace_dispatch!("rational_algorithm", "gcd", algorithm);
            crate::dispatch_trace::record_rational_gcd(left, right, &divisor);
        }
        #[cfg(not(feature = "dispatch-trace"))]
        let _ = algorithm;
        divisor
    }

    /// Quadratic Lehmer baseline retained for paired half-GCD benchmarks.
    #[doc(hidden)]
    pub fn gcd_magnitudes_lehmer_baseline(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() {
            return right.clone();
        }
        if right.is_zero() {
            return left.clone();
        }
        if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            return BigUint::from(Self::gcd_word(left, right));
        }
        Self::gcd_wide_magnitudes(left, right, false).0
    }

    /// Recursive Möller half-GCD candidate retained for benchmark comparison.
    #[doc(hidden)]
    pub fn gcd_magnitudes_half_gcd_candidate(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() {
            return right.clone();
        }
        if right.is_zero() {
            return left.clone();
        }
        if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            return BigUint::from(Self::gcd_word(left, right));
        }
        let (divisor, algorithm) = Self::gcd_wide_magnitudes(left, right, true);
        #[cfg(feature = "dispatch-trace")]
        {
            crate::trace_dispatch!("rational_algorithm", "gcd", algorithm);
            crate::dispatch_trace::record_rational_gcd(left, right, &divisor);
        }
        #[cfg(not(feature = "dispatch-trace"))]
        let _ = algorithm;
        divisor
    }

    fn gcd_wide_magnitudes(
        left: &BigUint,
        right: &BigUint,
        half_gcd_enabled: bool,
    ) -> (BigUint, &'static str) {

        let (larger, smaller) = if left >= right {
            (left, right)
        } else {
            (right, left)
        };
        if let Some(smaller_word) = smaller.to_u128() {
            let remainder = (larger % smaller)
                .to_u128()
                .expect("remainder is smaller than a u128 divisor");
            return (
                BigUint::from(Self::gcd_word(smaller_word, remainder)),
                "euclidean-wide-word",
            );
        }

        let mut larger = larger.clone();
        let mut smaller = smaller.clone();
        let mut used_half_gcd = false;
        while half_gcd_enabled
            && smaller.bits() >= Self::HALF_GCD_THRESHOLD_BITS
            && larger.bits().abs_diff(smaller.bits()) <= 1
        {
            let before_bits = larger.bits().max(smaller.bits());
            let Some(reduction) = Self::half_gcd_reduce(&larger, &smaller) else {
                break;
            };
            let (mut first, mut second) = (reduction.left, reduction.right);
            if first == second {
                return (first, "recursive-half-gcd");
            }
            if first > second {
                first -= &second;
            } else {
                second -= &first;
            }
            (larger, smaller) = if first >= second {
                (first, second)
            } else {
                (second, first)
            };
            if larger.bits().max(smaller.bits()) >= before_bits {
                break;
            }
            used_half_gcd = true;
        }
        let lehmer_selected = smaller.bits() >= Self::LEHMER_GCD_THRESHOLD_BITS
            && larger.bits().abs_diff(smaller.bits()) <= 1;
        let mut used_lehmer = false;
        while !smaller.is_zero() {
            if lehmer_selected
                && smaller.bits() > 128
                && larger.bits().abs_diff(smaller.bits()) <= 1
                && let Some(matrix) = Self::lehmer_gcd_matrix(&larger, &smaller)
                && let Some((first, second)) =
                    Self::apply_lehmer_gcd_matrix(&larger, &smaller, matrix)
            {
                used_lehmer = true;
                (larger, smaller) = if first >= second {
                    (first, second)
                } else {
                    (second, first)
                };
                continue;
            }
            let remainder = &larger % &smaller;
            larger = smaller;
            smaller = remainder;
        }
        (
            larger,
            if used_half_gcd {
                "recursive-half-gcd"
            } else if used_lehmer {
                "lehmer-leading-limb"
            } else {
                "euclidean-wide-remainder"
            },
        )
    }

    /// Multiply a fixed set of rationals by one positive common denominator.
    ///
    /// Vector normalization is invariant under this shared scale. Clearing it
    /// before the self-dot prevents the common denominator from passing through
    /// square extraction, reciprocal construction, and every output lane.
    pub(crate) fn clear_common_denominator<const N: usize>(values: [&Self; N]) -> [Self; N] {
        let common_denominator = if values
            .iter()
            .all(|value| value.denominator == values[0].denominator)
        {
            values[0].denominator.clone()
        } else if values
            .iter()
            .all(|value| Self::is_power_of_two(&value.denominator))
        {
            values
                .iter()
                .max_by_key(|value| value.denominator.bits())
                .expect("fixed rational set is nonempty")
                .denominator
                .clone()
        } else {
            values.iter().skip(1).fold(
                values[0].denominator.clone(),
                |common, value| {
                    let divisor = Self::gcd_magnitudes(&common, &value.denominator);
                    (common / divisor) * &value.denominator
                },
            )
        };

        crate::trace_dispatch!("rational", "common-scale", "clear-denominator");
        core::array::from_fn(|index| {
            let value = values[index];
            if value.sign == NoSign {
                return Self::zero();
            }
            let scale = &common_denominator / &value.denominator;
            Self::from_integer_magnitude(value.sign, &value.numerator * scale)
        })
    }

    fn from_word_magnitude_difference(
        positive: u128,
        negative: u128,
        denominator: u128,
    ) -> Self {
        let (sign, magnitude) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => {
                crate::trace_dispatch!("rational", "word-result", "zero");
                return Self::zero();
            }
        };
        if denominator.is_power_of_two() {
            let common_shift = magnitude.trailing_zeros().min(denominator.trailing_zeros());
            return Self::from_reduced_word_parts(
                sign,
                magnitude >> common_shift,
                denominator >> common_shift,
            );
        }

        let common_shift = magnitude.trailing_zeros().min(denominator.trailing_zeros());
        let mut magnitude = magnitude >> common_shift;
        let mut denominator = denominator >> common_shift;
        let odd_denominator = denominator >> denominator.trailing_zeros();
        if Self::POWERS_OF_FIVE
            .binary_search(&odd_denominator)
            .is_ok()
        {
            while denominator.is_multiple_of(5) && magnitude.is_multiple_of(5) {
                denominator /= 5;
                magnitude /= 5;
            }
        } else {
            let divisor = Self::gcd_word(magnitude, denominator);
            magnitude /= divisor;
            denominator /= divisor;
        }
        Self::from_reduced_word_parts(sign, magnitude, denominator)
    }

    fn from_reduced_word_parts(sign: Sign, magnitude: u128, denominator: u128) -> Self {
        debug_assert_ne!(sign, NoSign);
        debug_assert_ne!(magnitude, 0);
        debug_assert_ne!(denominator, 0);
        if magnitude == denominator {
            crate::trace_dispatch!("rational", "word-result", "unit");
            return if sign == Minus {
                Self::minus_one()
            } else {
                Self::one()
            };
        }
        if denominator == 1
            && let Some(value) = Self::small_integer(sign, magnitude)
        {
            crate::trace_dispatch!("rational", "word-result", "cached-small-integer");
            return value;
        }
        if let Some(value) = Self::small_dyadic(sign, magnitude, denominator) {
            crate::trace_dispatch!("rational", "word-result", "cached-small-dyadic");
            return value;
        }
        if denominator == 1 {
            #[cfg(feature = "dispatch-trace")]
            {
                let path = match magnitude {
                    0..=127 => "uncached-integer-65-127",
                    128..=255 => "uncached-integer-128-255",
                    256..=1023 => "uncached-integer-256-1023",
                    1024..=4095 => "uncached-integer-1024-4095",
                    _ => "uncached-integer-wide",
                };
                crate::trace_dispatch!("rational", "word-result", path);
            }
        } else if denominator.is_power_of_two() {
            crate::trace_dispatch!("rational", "word-result", "dyadic-fraction");
        } else if magnitude <= u128::from(u64::MAX) && denominator <= u128::from(u64::MAX) {
            crate::trace_dispatch!("rational", "word-result", "small-general-fraction");
        } else {
            crate::trace_dispatch!("rational", "word-result", "wide-general-fraction");
        }
        Self::from_parts_raw(
            sign,
            BigUint::from(magnitude),
            BigUint::from(denominator),
        )
    }

    fn product_term_words<const FACTORS: usize>(
        term: [&Self; FACTORS],
    ) -> Option<(u128, u128)> {
        let mut magnitude = 1_u128;
        let mut denominator = 1_u128;
        for factor in term {
            let numerator = factor.numerator.to_u128()?;
            let factor_denominator = factor.denominator.to_u128()?;
            let Some(next_magnitude) = magnitude.checked_mul(numerator) else {
                return Self::product_term_words_cross_cancelled(term);
            };
            let Some(next_denominator) = denominator.checked_mul(factor_denominator) else {
                return Self::product_term_words_cross_cancelled(term);
            };
            magnitude = next_magnitude;
            denominator = next_denominator;
        }
        Some((magnitude, denominator))
    }

    fn product_term_words_cross_cancelled<const FACTORS: usize>(
        term: [&Self; FACTORS],
    ) -> Option<(u128, u128)> {
        let mut numerators = [0_u128; FACTORS];
        let mut denominators = [1_u128; FACTORS];
        for i in 0..FACTORS {
            numerators[i] = term[i].numerator.to_u128()?;
            denominators[i] = term[i].denominator.to_u128()?;
        }

        for numerator in &mut numerators {
            if *numerator == 1 {
                continue;
            }
            for denominator in &mut denominators {
                if *denominator == 1 {
                    continue;
                }
                let divisor = Self::gcd_word(*numerator, *denominator);
                *numerator /= divisor;
                *denominator /= divisor;
                if *numerator == 1 {
                    break;
                }
            }
        }

        let mut magnitude = 1_u128;
        let mut denominator = 1_u128;
        for factor in numerators {
            magnitude = magnitude.checked_mul(factor)?;
        }
        for factor in denominators {
            denominator = denominator.checked_mul(factor)?;
        }
        Some((magnitude, denominator))
    }

    fn signed_product_sum_words<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<Self> {
        let (positive, negative, common_denominator) =
            Self::signed_product_sum_word_totals(terms, signs)?;
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            common_denominator,
        ))
    }

    fn signed_product_sum_word_totals<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<(u128, u128, u128)> {
        let mut magnitudes = [0_u128; TERMS];
        let mut denominators = [1_u128; TERMS];
        let mut common_denominator = None::<u128>;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let (magnitude, denominator) = Self::product_term_words(terms[i])?;
            magnitudes[i] = magnitude;
            denominators[i] = denominator;
            common_denominator = Some(match common_denominator {
                None => denominator,
                Some(common) if common == denominator => common,
                Some(common) => {
                    let divisor = Self::gcd_word(common, denominator);
                    common.checked_mul(denominator / divisor)?
                }
            });
        }

        let common_denominator = common_denominator.unwrap_or(1);

        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = magnitudes[i]
                .checked_mul(common_denominator / denominators[i])?;
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some((positive, negative, common_denominator))
    }

    fn two_product_word_sum(
        first: (Sign, u128, u128),
        second: (Sign, u128, u128),
    ) -> Option<Self> {
        let common_denominator = match (first.0, second.0) {
            (NoSign, NoSign) => return Some(Self::zero()),
            (NoSign, _) => second.2,
            (_, NoSign) => first.2,
            _ if first.2 == second.2 => first.2,
            _ => {
                let divisor = Self::gcd_word(first.2, second.2);
                first.2.checked_mul(second.2 / divisor)?
            }
        };
        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for (sign, magnitude, denominator) in [first, second] {
            if sign == NoSign {
                continue;
            }
            let magnitude = magnitude.checked_mul(common_denominator / denominator)?;
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            common_denominator,
        ))
    }

    fn word_product(
        left: (Sign, u128, u128),
        right: (Sign, u128, u128),
        positive: bool,
    ) -> Option<(Sign, u128, u128)> {
        let sign = (if positive { Plus } else { Minus }) * left.0 * right.0;
        if sign == NoSign {
            return Some((NoSign, 0, 1));
        }
        Some((
            sign,
            left.1.checked_mul(right.1)?,
            left.2.checked_mul(right.2)?,
        ))
    }

    fn signed_word_sum(first: (Sign, u128), second: (Sign, u128)) -> Option<(Sign, u128)> {
        match (first.0, second.0) {
            (NoSign, _) => Some(second),
            (_, NoSign) => Some(first),
            (left, right) if left == right => Some((left, first.1.checked_add(second.1)?)),
            _ if first.1 > second.1 => Some((first.0, first.1 - second.1)),
            _ if second.1 > first.1 => Some((second.0, second.1 - first.1)),
            _ => Some((NoSign, 0)),
        }
    }

    fn scaled_dyadic_word_part(
        part: (Sign, u128, u128),
        common_shift: u32,
    ) -> Option<(Sign, u128, u128)> {
        let shift = common_shift.checked_sub(part.2.trailing_zeros())?;
        Some((part.0, part.1.checked_shl(shift)?, 1))
    }

    fn from_scaled_dyadic_quotient_component(
        sign: Sign,
        mut magnitude: u128,
        mut denominator: u128,
        scale_shift: i32,
    ) -> Option<Self> {
        if sign == NoSign || magnitude == 0 {
            return Some(Self::zero());
        }

        let divisor = Self::gcd_word(magnitude, denominator);
        magnitude /= divisor;
        denominator /= divisor;
        if scale_shift >= 0 {
            let shift = scale_shift as u32;
            let cancel = shift.min(denominator.trailing_zeros());
            denominator >>= cancel;
            magnitude = magnitude.checked_shl(shift - cancel)?;
        } else {
            let shift = scale_shift.unsigned_abs();
            let cancel = shift.min(magnitude.trailing_zeros());
            magnitude >>= cancel;
            denominator = denominator.checked_shl(shift - cancel)?;
        }
        Some(Self::from_reduced_word_parts(
            sign,
            magnitude,
            denominator,
        ))
    }

    fn complex_dyadic_quotient_words(
        parts: [(Sign, u128, u128); 4],
    ) -> Option<Result<(Self, Self), crate::Problem>> {
        if !parts.iter().all(|part| part.2.is_power_of_two()) {
            return None;
        }
        let left_shift = parts[0]
            .2
            .trailing_zeros()
            .max(parts[1].2.trailing_zeros());
        let right_shift = parts[2]
            .2
            .trailing_zeros()
            .max(parts[3].2.trailing_zeros());
        let a = Self::scaled_dyadic_word_part(parts[0], left_shift)?;
        let b = Self::scaled_dyadic_word_part(parts[1], left_shift)?;
        let c = Self::scaled_dyadic_word_part(parts[2], right_shift)?;
        let d = Self::scaled_dyadic_word_part(parts[3], right_shift)?;

        let ac = Self::word_product(a, c, true)?;
        let bd = Self::word_product(b, d, true)?;
        let bc = Self::word_product(b, c, true)?;
        let ad = Self::word_product(a, d, false)?;
        let cc = Self::word_product(c, c, true)?;
        let dd = Self::word_product(d, d, true)?;
        let re = Self::signed_word_sum((ac.0, ac.1), (bd.0, bd.1))?;
        let im = Self::signed_word_sum((bc.0, bc.1), (ad.0, ad.1))?;
        let norm = Self::signed_word_sum((cc.0, cc.1), (dd.0, dd.1))?;
        if norm.0 == NoSign {
            return Some(Err(crate::Problem::DivideByZero));
        }
        let scale_shift = i32::try_from(right_shift).ok()? - i32::try_from(left_shift).ok()?;
        let re = Self::from_scaled_dyadic_quotient_component(
            re.0,
            re.1,
            norm.1,
            scale_shift,
        )?;
        let im = Self::from_scaled_dyadic_quotient_component(
            im.0,
            im.1,
            norm.1,
            scale_shift,
        )?;
        crate::trace_dispatch!("rational", "complex-quotient", "paired-dyadic-word-sized");
        Some(Ok((re, im)))
    }

    fn from_scaled_word_quotient_component(
        sign: Sign,
        mut magnitude: u128,
        mut norm: u128,
        mut scale_numerator: u128,
        mut scale_denominator: u128,
    ) -> Option<Self> {
        if sign == NoSign || magnitude == 0 {
            return Some(Self::zero());
        }

        let divisor = Self::gcd_word(magnitude, norm);
        magnitude /= divisor;
        norm /= divisor;
        if scale_numerator == scale_denominator {
            scale_numerator = 1;
            scale_denominator = 1;
        } else {
            let divisor = Self::gcd_word(scale_numerator, scale_denominator);
            scale_numerator /= divisor;
            scale_denominator /= divisor;
        }
        if scale_denominator != 1 {
            let divisor = Self::gcd_word(magnitude, scale_denominator);
            magnitude /= divisor;
            scale_denominator /= divisor;
        }
        if scale_numerator != 1 {
            let divisor = Self::gcd_word(scale_numerator, norm);
            scale_numerator /= divisor;
            norm /= divisor;
        }

        Some(Self::from_reduced_word_parts(
            sign,
            magnitude.checked_mul(scale_numerator)?,
            norm.checked_mul(scale_denominator)?,
        ))
    }

    fn complex_word_quotient_words(
        parts: [(Sign, u128, u128); 4],
    ) -> Option<Result<(Self, Self), crate::Problem>> {
        let left_denominator = if parts[0].2 == parts[1].2 {
            parts[0].2
        } else {
            let divisor = Self::gcd_word(parts[0].2, parts[1].2);
            parts[0].2.checked_mul(parts[1].2 / divisor)?
        };
        let right_denominator = if parts[2].2 == parts[3].2 {
            parts[2].2
        } else {
            let divisor = Self::gcd_word(parts[2].2, parts[3].2);
            parts[2].2.checked_mul(parts[3].2 / divisor)?
        };
        let a = (
            parts[0].0,
            parts[0].1.checked_mul(left_denominator / parts[0].2)?,
            1,
        );
        let b = (
            parts[1].0,
            parts[1].1.checked_mul(left_denominator / parts[1].2)?,
            1,
        );
        let c = (
            parts[2].0,
            parts[2].1.checked_mul(right_denominator / parts[2].2)?,
            1,
        );
        let d = (
            parts[3].0,
            parts[3].1.checked_mul(right_denominator / parts[3].2)?,
            1,
        );

        let ac = Self::word_product(a, c, true)?;
        let bd = Self::word_product(b, d, true)?;
        let bc = Self::word_product(b, c, true)?;
        let ad = Self::word_product(a, d, false)?;
        let cc = Self::word_product(c, c, true)?;
        let dd = Self::word_product(d, d, true)?;
        let re = Self::signed_word_sum((ac.0, ac.1), (bd.0, bd.1))?;
        let im = Self::signed_word_sum((bc.0, bc.1), (ad.0, ad.1))?;
        let norm = Self::signed_word_sum((cc.0, cc.1), (dd.0, dd.1))?;
        if norm.0 == NoSign {
            return Some(Err(crate::Problem::DivideByZero));
        }
        let re = Self::from_scaled_word_quotient_component(
            re.0,
            re.1,
            norm.1,
            right_denominator,
            left_denominator,
        )?;
        let im = Self::from_scaled_word_quotient_component(
            im.0,
            im.1,
            norm.1,
            right_denominator,
            left_denominator,
        )?;
        crate::trace_dispatch!("rational", "complex-quotient", "paired-general-word-sized");
        Some(Ok((re, im)))
    }

    fn complex_product_components_impl(
        left: [&Self; 2],
        right: [&Self; 2],
        conjugate_right: bool,
    ) -> (Self, Self) {
        let word_parts = [left[0], left[1], right[0], right[1]].map(|value| {
            Some((
                value.sign,
                value.numerator.to_u128()?,
                value.denominator.to_u128()?,
            ))
        });
        if let [Some(a), Some(b), Some(c), Some(d)] = word_parts
            && let (Some(ac), Some(bd), Some(ad), Some(bc)) = (
                Self::word_product(a, c, true),
                Self::word_product(b, d, conjugate_right),
                Self::word_product(a, d, !conjugate_right),
                Self::word_product(b, c, true),
            )
            && let (Some(re), Some(im)) = (
                Self::two_product_word_sum(ac, bd),
                Self::two_product_word_sum(ad, bc),
            )
        {
            crate::trace_dispatch!(
                "rational",
                "complex-product",
                if conjugate_right {
                    "paired-conjugate-word-sized"
                } else {
                    "paired-word-sized"
                }
            );
            return (re, im);
        }

        crate::trace_dispatch!(
            "rational",
            "complex-product",
            if conjugate_right {
                "paired-conjugate-general-fallback"
            } else {
                "paired-general-fallback"
            }
        );
        if conjugate_right {
            (
                Self::signed_product_sum2(
                    [true, true],
                    [[left[0], right[0]], [left[1], right[1]]],
                ),
                Self::signed_product_sum2(
                    [false, true],
                    [[left[0], right[1]], [left[1], right[0]]],
                ),
            )
        } else {
            (
                Self::signed_product_sum2(
                    [true, false],
                    [[left[0], right[0]], [left[1], right[1]]],
                ),
                Self::signed_product_sum2(
                    [true, true],
                    [[left[0], right[1]], [left[1], right[0]]],
                ),
            )
        }
    }

    /// Multiply two exact complex component pairs with one shared word scan.
    ///
    /// Returns `(ac - bd, ad + bc)` for left `(a, b)` and right `(c, d)`.
    /// Word-sized operands convert once; wider products fall back to the
    /// general exact signed-product reducers.
    pub fn complex_product_components(left: [&Self; 2], right: [&Self; 2]) -> (Self, Self) {
        Self::complex_product_components_impl(left, right, false)
    }

    /// Divide two exact complex component pairs with delayed canonicalization.
    ///
    /// The conjugate product `(ac + bd, bc - ad)` is formed with one shared
    /// word scan, while `c² + d²` is reduced once and reused by both output
    /// components. No approximation participates in the zero check or result.
    pub fn complex_quotient_components(
        left: [&Self; 2],
        right: [&Self; 2],
    ) -> Result<(Self, Self), crate::Problem> {
        let word_parts = [left[0], left[1], right[0], right[1]].map(|value| {
            Some((
                value.sign,
                value.numerator.to_u128()?,
                value.denominator.to_u128()?,
            ))
        });
        if let [Some(a), Some(b), Some(c), Some(d)] = word_parts
        {
            if let Some(result) = Self::complex_dyadic_quotient_words([a, b, c, d]) {
                return result;
            }
            if let Some(result) = Self::complex_word_quotient_words([a, b, c, d]) {
                return result;
            }
        }

        let (re_numerator, im_numerator) =
            Self::complex_product_components_impl(left, right, true);
        let denominator =
            Self::signed_product_sum2([true, true], [[right[0], right[0]], [right[1], right[1]]]);
        if denominator.sign == NoSign {
            return Err(crate::Problem::DivideByZero);
        }
        crate::trace_dispatch!("rational", "complex-quotient", "paired-exact-rational");
        Ok((
            &re_numerator / &denominator,
            &im_numerator / &denominator,
        ))
    }

    /// Invert a fixed 3x3 exact-rational matrix as one aggregate operation.
    ///
    /// Keeping the cofactors and shared determinant reciprocal in `Rational`
    /// avoids repeatedly reclassifying and wrapping intermediate `Real`
    /// values. Each signed product is still fused and every returned component
    /// remains canonically exact.
    pub(crate) fn matrix3_inverse_components(
        matrix: [[&Self; 3]; 3],
    ) -> Result<[[Self; 3]; 3], crate::Problem> {
        let m = matrix;
        let c00 = Self::signed_product_sum2(
            [true, false],
            [[m[1][1], m[2][2]], [m[1][2], m[2][1]]],
        );
        let c01 = Self::signed_product_sum2(
            [true, false],
            [[m[0][2], m[2][1]], [m[0][1], m[2][2]]],
        );
        let c02 = Self::signed_product_sum2(
            [true, false],
            [[m[0][1], m[1][2]], [m[0][2], m[1][1]]],
        );
        let c10 = Self::signed_product_sum2(
            [true, false],
            [[m[1][2], m[2][0]], [m[1][0], m[2][2]]],
        );
        let c11 = Self::signed_product_sum2(
            [true, false],
            [[m[0][0], m[2][2]], [m[0][2], m[2][0]]],
        );
        let c12 = Self::signed_product_sum2(
            [true, false],
            [[m[0][2], m[1][0]], [m[0][0], m[1][2]]],
        );
        let c20 = Self::signed_product_sum2(
            [true, false],
            [[m[1][0], m[2][1]], [m[1][1], m[2][0]]],
        );
        let c21 = Self::signed_product_sum2(
            [true, false],
            [[m[0][1], m[2][0]], [m[0][0], m[2][1]]],
        );
        let c22 = Self::signed_product_sum2(
            [true, false],
            [[m[0][0], m[1][1]], [m[0][1], m[1][0]]],
        );
        let determinant = Self::signed_product_sum(
            [true, true, true],
            [
                [m[0][0], &c00],
                [m[0][1], &c10],
                [m[0][2], &c20],
            ],
        );
        let inverse_determinant = determinant.inverse()?;
        crate::trace_dispatch!("rational", "matrix3-inverse", "aggregate-cofactor");
        Ok([
            [
                c00 * &inverse_determinant,
                c01 * &inverse_determinant,
                c02 * &inverse_determinant,
            ],
            [
                c10 * &inverse_determinant,
                c11 * &inverse_determinant,
                c12 * &inverse_determinant,
            ],
            [
                c20 * &inverse_determinant,
                c21 * &inverse_determinant,
                c22 * &inverse_determinant,
            ],
        ])
    }

    fn dot_products_dyadic<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<Self> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; N];
        let mut any_nonzero = false;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let shift =
                left[i].dyadic_denominator_shift()? + right[i].dyadic_denominator_shift()?;
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Self::zero());
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic dot-product scale should fit in usize");
            let mut magnitude = &left[i].numerator * &right[i].numerator;
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        let denominator =
            BigUint::one() << usize::try_from(max_shift).expect("shift should fit in usize");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    fn dot_products_equal_denominator<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<Self> {
        let mut shared_denominator = None::<BigUint>;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            match &shared_denominator {
                None => shared_denominator = Some(denominator),
                Some(shared) if *shared == denominator => {}
                Some(_) => return None,
            }
        }

        let Some(denominator) = shared_denominator else {
            return Some(Self::zero());
        };

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = &left[i].numerator * &right[i].numerator;
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    fn product_term_denominator<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut denominator = BigUint::one();
        for factor in term {
            denominator *= &factor.denominator;
        }
        denominator
    }

    fn product_term_magnitude<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut magnitude = BigUint::one();
        for factor in term {
            magnitude *= &factor.numerator;
        }
        magnitude
    }

    fn product_term_sign<const FACTORS: usize>(positive: bool, term: [&Self; FACTORS]) -> Sign {
        let mut sign = if positive { Plus } else { Minus };
        for factor in term {
            sign = sign * factor.sign;
        }
        sign
    }

    fn product_sum_dyadic_plan<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<DyadicProductSumPlan<TERMS>> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; TERMS];
        let mut numerator_bits = [0_u64; TERMS];
        let mut live_terms = 0_u64;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            live_terms += 1;
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift = shift.checked_add(factor.dyadic_denominator_shift()?)?;
                numerator_bits[i] = numerator_bits[i].saturating_add(factor.numerator.bits());
            }
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
        }
        let total_growth = live_terms.next_power_of_two().trailing_zeros();
        let prefer_wide = (0..TERMS).any(|i| {
            signs[i] != NoSign
                && numerator_bits[i]
                    .saturating_add(max_shift - denominator_shifts[i])
                    .saturating_add(u64::from(total_growth))
                    > u64::from(u128::BITS)
        });
        Some(DyadicProductSumPlan {
            denominator_shifts,
            max_shift,
            prefer_wide,
        })
    }

    fn signed_product_sum_dyadic_with_plan<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Self {
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic product-sum scale should fit in usize");
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        let denominator =
            BigUint::one() << usize::try_from(max_shift).expect("shift should fit in usize");
        Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        )
    }

    fn signed_product_sum_dyadic_ordering_with_plan<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
        denominator_shifts: [u64; TERMS],
        max_shift: u64,
    ) -> Ordering {
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic product-sum scale should fit in usize");
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        positive.cmp(&negative)
    }

    /// Evaluate a signed product sum when every live factor shares one
    /// reduced denominator.
    ///
    /// The caller is expected to have carried an object-level common-scale
    /// certificate, but this method still validates the denominator fact before
    /// using it. Returning `None` means the certificate was too weak for this
    /// particular product shape, so callers should fall back to
    /// [`Self::signed_product_sum`]. Keeping the validation in `Rational`
    /// preserves scalar storage ownership while giving geometric kernels a
    /// denominator-specialized schedule. The final delayed reduction uses the
    /// same fraction-delay strategy as the generic reducer.
    pub fn signed_product_sum_shared_denominator<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Option<Self> {
        debug_assert!(FACTORS > 0);
        let mut signs = [NoSign; TERMS];
        let mut nonzero_count = 0_usize;
        let mut shared_denominator = None::<&BigUint>;
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign == NoSign {
                signs[i] = sign;
                continue;
            }
            nonzero_count += 1;
            signs[i] = sign;
            for factor in terms[i] {
                match shared_denominator {
                    None => shared_denominator = Some(&factor.denominator),
                    Some(shared) if shared == &factor.denominator => {}
                    Some(_) => return None,
                }
            }
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!(
                "rational",
                "product_sum",
                "shared-factor-denominator-all-zero"
            );
            return Some(Self::zero());
        }
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum", "word-sized-shared-scale");
            return Some(word);
        }
        let exponent = u32::try_from(FACTORS).ok()?;
        let denominator = shared_denominator
            .expect("nonzero product sum has a live factor denominator")
            .pow(exponent);
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = Self::product_term_magnitude(terms[i]);
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        crate::trace_dispatch!("rational", "product_sum", "shared-factor-denominator");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Return the exact arithmetic mean of borrowed rationals, reducing once.
    ///
    /// The denominator schedule is built across the complete input before the
    /// final division by the number of values. This avoids reducing every
    /// partial sum and then reducing the quotient again.
    pub fn mean_refs(values: &[&Self]) -> Option<Self> {
        if values.is_empty() {
            return None;
        }
        let count = BigUint::from(values.len());
        Some(Self::sum_refs_with_denominator_factor(values, &count))
    }

    fn sum_refs_with_denominator_factor(values: &[&Self], factor: &BigUint) -> Self {
        let mut live_count = 0_usize;
        let mut common_dyadic_shift = Some(0_u64);
        let mut shared_denominator = None::<&BigUint>;
        let mut equal_denominator = true;
        for &value in values {
            if value.sign == NoSign {
                continue;
            }
            live_count += 1;
            common_dyadic_shift = match (common_dyadic_shift, value.dyadic_denominator_shift()) {
                (Some(current), Some(shift)) => Some(current.max(shift)),
                _ => None,
            };
            match shared_denominator {
                None => shared_denominator = Some(&value.denominator),
                Some(shared) if shared == &value.denominator => {}
                Some(_) => equal_denominator = false,
            }
        }
        if live_count == 0 {
            crate::trace_dispatch!("rational", "mean", "all-zero");
            return Self::zero();
        }

        if let Some(common_shift) = common_dyadic_shift {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for &value in values {
                if value.sign == NoSign {
                    continue;
                }
                let shift = value
                    .dyadic_denominator_shift()
                    .expect("all live mean inputs were certified dyadic");
                let magnitude = &value.numerator << (common_shift - shift);
                match value.sign {
                    Plus => positive += magnitude,
                    Minus => negative += magnitude,
                    NoSign => {}
                }
            }
            let mut denominator = BigUint::one() << common_shift;
            denominator *= factor;
            crate::trace_dispatch!("rational", "mean", "dyadic-shared-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        if equal_denominator {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for &value in values {
                match value.sign {
                    Plus => positive += &value.numerator,
                    Minus => negative += &value.numerator,
                    NoSign => {}
                }
            }
            let mut denominator = shared_denominator
                .expect("nonzero mean has a shared denominator")
                .clone();
            denominator *= factor;
            crate::trace_dispatch!("rational", "mean", "equal-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        let mut common_denominator = BigUint::one();
        for &value in values {
            if value.sign == NoSign {
                continue;
            }
            if value.denominator != *ONE.deref() {
                let divisor = Self::gcd_magnitudes_with_mixed_width_fast_path(
                    &common_denominator,
                    &value.denominator,
                );
                trace_rational_gcd!(&common_denominator, &value.denominator, &divisor);
                common_denominator *= &value.denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for &value in values {
            if value.sign == NoSign {
                continue;
            }
            let mut magnitude = value.numerator.clone();
            if value.denominator != common_denominator {
                magnitude *= &common_denominator / &value.denominator;
            }
            match value.sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        common_denominator *= factor;
        crate::trace_dispatch!("rational", "mean", "lcm-shared-denominator");
        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

    /// Evaluate a fixed-size signed sum of products exactly.
    ///
    /// Each row in `terms` is multiplied, then added or subtracted according to
    /// the matching entry in `positive_terms`. The implementation delays
    /// rational reduction until the final sum and has structural fast paths for
    /// dyadic and equal-denominator products. This is the scalar reducer that
    /// geometry crates use for small determinant schedules while preserving
    /// their abstraction boundary: they pass a known polynomial shape, but do
    /// not inspect `Rational` storage internals.
    ///
    /// The fraction-delay strategy keeps algebraic object shape visible before
    /// scalar expansion.
    pub fn signed_product_sum2(
        positive_terms: [bool; 2],
        terms: [[&Self; 2]; 2],
    ) -> Self {
        let signs = [
            Self::product_term_sign(positive_terms[0], terms[0]),
            Self::product_term_sign(positive_terms[1], terms[1]),
        ];
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum", "fixed-two-by-two-word-sized");
            return word;
        }
        crate::trace_dispatch!("rational", "product_sum", "fixed-two-by-two-fallback");
        Self::signed_product_sum(positive_terms, terms)
    }

    /// Evaluate a fixed-size signed sum of products exactly.
    ///
    /// This general entry point accepts any fixed product shape. Two products
    /// of two factors should use [`Self::signed_product_sum2`] so word-sized
    /// complex and determinant kernels can bypass the generic shape planner.
    pub fn signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Self {
        // Short determinant and cofactor polynomials are exact rational sums of
        // products. As with `dot_products`, build one denominator and reduce
        // only the final row. This targets the trace rows where fixed 3x3/4x4
        // inverse, division, and negative-powi kernels still paid repeated gcd
        // work after dot products had already been fused. The algebraic
        // strategy delays fractions and keeps the public fixed-size cofactor
        // formulas division-free.
        // Structural note: keep this hook scalar-local. Hyperlattice can use it
        // for exact cofactor and determinant kernels, while predicate and
        // triangulation crates should consume only the resulting exact signs or
        // values through their own abstraction boundaries.
        debug_assert!(FACTORS > 0);
        let mut signs = [NoSign; TERMS];
        let mut nonzero_count = 0_usize;
        for i in 0..TERMS {
            // Term signs are pure structural facts from stored rational signs.
            // Compute them once and reuse them across dyadic, equal-denominator,
            // and LCM reducers. This preserves Bareiss-style delayed reduction
            // while removing repeated sign walks from exact cofactor/determinant
            // product sums.
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign != NoSign {
                nonzero_count += 1;
            }
            signs[i] = sign;
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!("rational", "product_sum", "all-zero");
            return Self::zero();
        }
        let dyadic_plan = Self::product_sum_dyadic_plan(terms, signs);
        let prefer_wide_dyadic = dyadic_plan
            .as_ref()
            .is_some_and(|plan| plan.prefer_wide);
        if !prefer_wide_dyadic
            && let Some(word) =
                Self::signed_product_sum_words(terms, signs)
        {
            crate::trace_dispatch!("rational", "product_sum", "word-sized");
            return word;
        }
        if nonzero_count == 1 {
            for i in 0..TERMS {
                match signs[i] {
                    Plus => {
                        crate::trace_dispatch!("rational", "product_sum", "single-term-product");
                        let denominator = Self::product_term_denominator(terms[i]);
                        return Self::from_signed_magnitude_difference(
                            Self::product_term_magnitude(terms[i]),
                            BigUint::ZERO,
                            denominator,
                        );
                    }
                    Minus => {
                        crate::trace_dispatch!("rational", "product_sum", "single-term-product");
                        let denominator = Self::product_term_denominator(terms[i]);
                        return Self::from_signed_magnitude_difference(
                            BigUint::ZERO,
                            Self::product_term_magnitude(terms[i]),
                            denominator,
                        );
                    }
                    NoSign => {}
                }
            }
        }

        if let Some(plan) = dyadic_plan {
            // Structural-dispatch note: callers that know coordinates are
            // lifted from a common binary grid could pass that grid exponent
            // with the terms and jump directly to this reducer, avoiding the
            // exploratory denominator scans used by generic exact rationals.
            crate::trace_dispatch!("rational", "product_sum", "dyadic-shared-denominator");
            return Self::signed_product_sum_dyadic_with_plan(
                terms,
                signs,
                plan.denominator_shifts,
                plan.max_shift,
            );
        }

        let mut denominators: [BigUint; TERMS] = std::array::from_fn(|_| BigUint::ZERO);
        let mut shared_denominator = None::<BigUint>;
        let mut equal_denominator = true;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            match &shared_denominator {
                None => shared_denominator = Some(denominator.clone()),
                Some(shared) if *shared == denominator => {}
                Some(_) => equal_denominator = false,
            }
            denominators[i] = denominator;
        }

        if equal_denominator {
            let denominator = shared_denominator.expect("nonzero product sum has denominator");
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..TERMS {
                let sign = signs[i];
                if sign == NoSign {
                    continue;
                }
                let magnitude = Self::product_term_magnitude(terms[i]);
                match sign {
                    Plus => positive += magnitude,
                    Minus => negative += magnitude,
                    NoSign => {}
                }
            }

            crate::trace_dispatch!("rational", "product_sum", "equal-product-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        crate::trace_dispatch!("rational", "product_sum", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            if denominator != ONE.deref() {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, denominator);
                trace_rational_gcd!(&common_denominator, denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            let denominator = &denominators[i];
            if denominator != &common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

    /// Compare a fixed signed sum of products with zero without materializing
    /// or reducing the resulting rational.
    pub fn signed_product_sum_ordering<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Ordering {
        debug_assert!(FACTORS > 0);
        let signs = std::array::from_fn(|i| Self::product_term_sign(positive_terms[i], terms[i]));
        let dyadic_plan = Self::product_sum_dyadic_plan(terms, signs);
        let prefer_wide_dyadic = dyadic_plan
            .as_ref()
            .is_some_and(|plan| plan.prefer_wide);
        if !prefer_wide_dyadic
            && let Some((positive, negative, _)) =
                Self::signed_product_sum_word_totals(terms, signs)
        {
            crate::trace_dispatch!("rational", "product_sum_ordering", "word-sized");
            return positive.cmp(&negative);
        }

        if let Some(plan) = dyadic_plan {
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering",
                "arbitrary-precision-dyadic"
            );
            return Self::signed_product_sum_dyadic_ordering_with_plan(
                terms,
                signs,
                plan.denominator_shifts,
                plan.max_shift,
            );
        }

        let mut denominators: [BigUint; TERMS] = std::array::from_fn(|_| BigUint::ZERO);
        let mut shared_denominator = None::<BigUint>;
        let mut equal_denominator = true;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            match &shared_denominator {
                None => shared_denominator = Some(denominator.clone()),
                Some(shared) if *shared == denominator => {}
                Some(_) => equal_denominator = false,
            }
            denominators[i] = denominator;
        }

        if equal_denominator {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..TERMS {
                match signs[i] {
                    Plus => positive += Self::product_term_magnitude(terms[i]),
                    Minus => negative += Self::product_term_magnitude(terms[i]),
                    NoSign => {}
                }
            }
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering",
                "arbitrary-precision-equal-denominator"
            );
            return positive.cmp(&negative);
        }

        let mut common_denominator = BigUint::one();
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            if denominator != ONE.deref() {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, denominator);
                trace_rational_gcd!(&common_denominator, denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if denominator != &common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        crate::trace_dispatch!(
            "rational",
            "product_sum_ordering",
            "arbitrary-precision-lcm"
        );
        positive.cmp(&negative)
    }

    pub(crate) fn dot_products<const N: usize>(left: [&Self; N], right: [&Self; N]) -> Self {
        // Dense vector and matrix dot products are exact rational linear
        // forms when all inputs are rational. Build one shared denominator and
        // canonicalize only the final sum instead of reducing every product
        // and partial sum, delaying fractions until the end.
        // Keep exact matrix rows at one rational
        // constructor per output cell. This dropped mat4 powi from-f64 trace
        // activity from 161.75 to 32 reductions/call and from 462.25 to 67.75
        // temporaries/call; keep future changes within noise of those counts.
        let mut signs = [NoSign; N];
        let mut nonzero_count = 0_usize;
        for i in 0..N {
            let sign = left[i].sign * right[i].sign;
            if sign != NoSign {
                nonzero_count += 1;
            }
            signs[i] = sign;
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!("rational", "dot_product", "all-zero");
            return Self::zero();
        }
        let terms = std::array::from_fn(|i| [left[i], right[i]]);
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "dot_product", "word-sized");
            return word;
        }
        if nonzero_count == 1 {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..N {
                match signs[i] {
                    Plus => {
                        let denominator = &left[i].denominator * &right[i].denominator;
                        positive = &left[i].numerator * &right[i].numerator;
                        crate::trace_dispatch!("rational", "dot_product", "single-term-product");
                        return Self::from_signed_magnitude_difference(
                            positive,
                            negative,
                            denominator,
                        );
                    }
                    Minus => {
                        let denominator = &left[i].denominator * &right[i].denominator;
                        negative = &left[i].numerator * &right[i].numerator;
                        crate::trace_dispatch!("rational", "dot_product", "single-term-product");
                        return Self::from_signed_magnitude_difference(
                            positive,
                            negative,
                            denominator,
                        );
                    }
                    NoSign => {}
                }
            }
            return Self::zero();
        }

        if let Some(dyadic) = Self::dot_products_dyadic(left, right, signs) {
            // Dyadic f64 imports are the hottest exact-rational matrix path.
            // A common power-of-two denominator lets us scale numerators with
            // shifts and lets `maybe_reduce` avoid a BigInt gcd.
            // Structural-dispatch note: matrix/vector callers with retained
            // grid-scale metadata can route straight here and reserve the LCM
            // path for genuinely mixed rational inputs.
            crate::trace_dispatch!("rational", "dot_product", "dyadic-shared-denominator");
            return dyadic;
        }
        if let Some(equal_denominator) = Self::dot_products_equal_denominator(left, right, signs) {
            // Decimal rational fixtures often enter with identical product
            // denominators even after exact parsing. The LCM algorithm below is
            // still the right general fallback, but this structural fact means
            // there is no LCM to build and no per-term scale division. 2026-05
            // tracing target: lower non-dyadic rational dot-product gcd counts
            // without perturbing the dyadic fast path above. Targeted
            // Criterion, 200 samples/8s: hyperlattice hyperreal-rational
            // mat3 powi improved 2.83%, mat4 div_matrix improved 3.88%,
            // mat3 inverse_checked and mat4 powi stayed within noise.
            crate::trace_dispatch!("rational", "dot_product", "equal-product-denominator");
            return equal_denominator;
        }

        crate::trace_dispatch!("rational", "dot_product", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        let mut any_nonzero = false;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            if denominator != *ONE.deref() {
                let divisor =
                    Self::gcd_magnitudes_with_mixed_width_fast_path(&common_denominator, &denominator);
                trace_rational_gcd!(&common_denominator, &denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
            any_nonzero = true;
        }
        if !any_nonzero {
            return Self::zero();
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            let mut magnitude = &left[i].numerator * &right[i].numerator;
            if denominator != common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

}
