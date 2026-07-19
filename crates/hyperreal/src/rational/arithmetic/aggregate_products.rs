impl Rational {
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
    fn gcd_magnitudes(left: &BigUint, right: &BigUint) -> BigUint {
        if left.is_zero() {
            return right.clone();
        }
        if right.is_zero() {
            return left.clone();
        }
        if let (Some(left), Some(right)) = (left.to_u128(), right.to_u128()) {
            return BigUint::from(Self::gcd_word(left, right));
        }

        let (larger, smaller) = if left >= right {
            (left, right)
        } else {
            (right, left)
        };
        if let Some(smaller_word) = smaller.to_u128() {
            let remainder = (larger % smaller)
                .to_u128()
                .expect("remainder is smaller than a u128 divisor");
            return BigUint::from(Self::gcd_word(smaller_word, remainder));
        }

        let mut larger = larger.clone();
        let mut smaller = smaller.clone();
        while !smaller.is_zero() {
            let remainder = &larger % &smaller;
            larger = smaller;
            smaller = remainder;
        }
        larger
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

    fn signed_product_sum_dyadic<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<Self> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; TERMS];
        let mut any_nonzero = false;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift += factor.dyadic_denominator_shift()?;
            }
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Self::zero());
        }

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
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    fn signed_product_sum_dyadic_ordering<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<Ordering> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; TERMS];
        let mut any_nonzero = false;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift = shift.checked_add(factor.dyadic_denominator_shift()?)?;
            }
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Ordering::Equal);
        }

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
        Some(positive.cmp(&negative))
    }

    fn product_sum_prefers_wide_dyadic<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> bool {
        let mut denominator_shifts = [0_u64; TERMS];
        let mut numerator_bits = [0_u64; TERMS];
        let mut max_shift = 0_u64;
        let mut live_terms = 0_u64;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            live_terms += 1;
            for factor in terms[i] {
                let Some(shift) = factor.dyadic_denominator_shift() else {
                    return false;
                };
                denominator_shifts[i] =
                    denominator_shifts[i].saturating_add(shift);
                numerator_bits[i] = numerator_bits[i]
                    .saturating_add(factor.numerator.bits());
            }
            max_shift = max_shift.max(denominator_shifts[i]);
        }

        let total_growth = live_terms.next_power_of_two().trailing_zeros();
        (0..TERMS).any(|i| {
            signs[i] != NoSign
                && numerator_bits[i]
                    .saturating_add(max_shift - denominator_shifts[i])
                    .saturating_add(u64::from(total_growth))
                    > u64::from(u128::BITS)
        })
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
        let prefer_wide_dyadic =
            Self::product_sum_prefers_wide_dyadic(terms, signs);
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

        if let Some(dyadic) = Self::signed_product_sum_dyadic(terms, signs) {
            // Structural-dispatch note: callers that know coordinates are
            // lifted from a common binary grid could pass that grid exponent
            // with the terms and jump directly to this reducer, avoiding the
            // exploratory denominator scans used by generic exact rationals.
            crate::trace_dispatch!("rational", "product_sum", "dyadic-shared-denominator");
            return dyadic;
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
                let divisor = num::Integer::gcd(&common_denominator, denominator);
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
        let prefer_wide_dyadic =
            Self::product_sum_prefers_wide_dyadic(terms, signs);
        if !prefer_wide_dyadic
            && let Some((positive, negative, _)) =
                Self::signed_product_sum_word_totals(terms, signs)
        {
            crate::trace_dispatch!("rational", "product_sum_ordering", "word-sized");
            return positive.cmp(&negative);
        }

        if let Some(ordering) = Self::signed_product_sum_dyadic_ordering(terms, signs) {
            crate::trace_dispatch!(
                "rational",
                "product_sum_ordering",
                "arbitrary-precision-dyadic"
            );
            return ordering;
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
                let divisor = num::Integer::gcd(&common_denominator, denominator);
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
                let divisor = num::Integer::gcd(&common_denominator, &denominator);
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
