use core::ops::*;

/// Exact integer shortcuts, specialized rational arithmetic and retained relations.
///
/// Private kernels distinguish schedule ineligibility from arithmetic failure;
/// cache installers trust caller-proved relations. None of these paths supplies
/// a universal work, retained-memory or cancellation bound.
impl Rational {
    /// Evaluate `(self * right - subtract_left * subtract_right) / divisor` exactly.
    ///
    /// Canonicalizes all five operands in argument order before requiring integer
    /// parts and a nonzero divisor. Returns `None` for noninteger input, zero
    /// divisor or nondivisibility, including when a shortcut might otherwise
    /// annihilate an invalid operand. Successful zero is `Some(0)`.
    ///
    /// Signed magnitude products avoid intermediate rationals; exact division
    /// supports fraction-free elimination. Normalization, products and division
    /// can allocate without a total work/memory budget; `None` is not exhaustion.
    pub fn checked_exact_integer_cross_difference_quotient(
        &self,
        right: &Self,
        subtract_left: &Self,
        subtract_right: &Self,
        divisor: &Self,
    ) -> Option<Self> {
        let left = self.canonicalized_ref();
        let right = right.canonicalized_ref();
        let subtract_left = subtract_left.canonicalized_ref();
        let subtract_right = subtract_right.canonicalized_ref();
        let divisor = divisor.canonicalized_ref();
        if left.denominator != *ONE.deref()
            || right.denominator != *ONE.deref()
            || subtract_left.denominator != *ONE.deref()
            || subtract_right.denominator != *ONE.deref()
            || divisor.denominator != *ONE.deref()
            || divisor.sign == NoSign
        {
            return None;
        }

        let first_sign = left.sign * right.sign;
        let second_sign = subtract_left.sign * subtract_right.sign;
        let first = if first_sign == NoSign {
            BigUint::ZERO
        } else {
            Self::multiply_magnitudes(
                "integer-cross-difference-first",
                &left.numerator,
                &right.numerator,
            )
        };
        let second = if second_sign == NoSign {
            BigUint::ZERO
        } else {
            Self::multiply_magnitudes(
                "integer-cross-difference-second",
                &subtract_left.numerator,
                &subtract_right.numerator,
            )
        };
        let (sign, numerator) = if first_sign == NoSign {
            (-second_sign, second)
        } else if second_sign == NoSign {
            (first_sign, first)
        } else if first_sign != second_sign {
            (first_sign, first + second)
        } else {
            match first.cmp(&second) {
                core::cmp::Ordering::Greater => (first_sign, first - second),
                core::cmp::Ordering::Equal => return Some(Self::zero()),
                core::cmp::Ordering::Less => (-first_sign, second - first),
            }
        };
        if sign == NoSign {
            return Some(Self::zero());
        }
        if divisor.numerator.is_one() {
            crate::trace_dispatch!(
                "rational",
                "integer-cross-difference",
                "unit-divisor"
            );
            return Some(Self::from_integer_magnitude(
                sign * divisor.sign,
                numerator,
            ));
        }
        use num::Integer as _;
        let (quotient, remainder) = numerator.div_rem(&divisor.numerator);
        if !remainder.is_zero() {
            return None;
        }
        let sign = sign * divisor.sign;
        crate::trace_dispatch!(
            "rational",
            "integer-cross-difference",
            "exact-divisible"
        );
        Some(Self::from_integer_magnitude(sign, quotient))
    }

    /// Evaluate `self - subtractand * scale` on exact integer magnitudes.
    ///
    /// Both rational operands canonicalize and must be integers, even when scale
    /// is zero. Otherwise return `None`. The unsigned absolute scale handles
    /// `i64::MIN` without overflow. A zero scaled term clones the original self
    /// handle, which can still carry lazy storage despite the validation step.
    /// Other paths construct the signed integer difference without an intermediate
    /// rational product. Allocation/work is not budgeted by this shortcut.
    pub fn checked_exact_integer_scaled_difference(
        &self,
        subtractand: &Self,
        scale: i64,
    ) -> Option<Self> {
        let left = self.canonicalized_ref();
        let subtractand = subtractand.canonicalized_ref();
        if left.denominator != *ONE.deref() || subtractand.denominator != *ONE.deref() {
            return None;
        }

        let scale_sign = match scale.cmp(&0) {
            core::cmp::Ordering::Less => Minus,
            core::cmp::Ordering::Equal => NoSign,
            core::cmp::Ordering::Greater => Plus,
        };
        let right_sign = subtractand.sign * scale_sign;
        if right_sign == NoSign {
            return Some(self.clone());
        }
        let right = &subtractand.numerator * scale.unsigned_abs();
        let (sign, numerator) = if left.sign == NoSign {
            (-right_sign, right)
        } else if left.sign != right_sign {
            (left.sign, left.numerator.clone() + right)
        } else {
            match left.numerator.cmp(&right) {
                core::cmp::Ordering::Greater => {
                    (left.sign, left.numerator.clone() - right)
                }
                core::cmp::Ordering::Equal => return Some(Self::zero()),
                core::cmp::Ordering::Less => (-right_sign, right - &left.numerator),
            }
        };
        crate::trace_dispatch!(
            "rational",
            "integer-scaled-difference",
            "signed-word-scale"
        );
        Some(Self::from_integer_magnitude(sign, numerator))
    }

    /// Divide exact integers only when the result is also an integer.
    ///
    /// Canonicalize dividend then divisor. Fractional operands, a zero divisor
    /// or nonzero remainder return `None`; zero dividend succeeds only after
    /// those operand checks. This shortcut avoids an intermediate rational in
    /// fraction-free elimination. Success is exact, but normalization and division
    /// have no total resource bound or typed exhaustion outcome.
    pub fn checked_exact_integer_quotient(&self, divisor: &Self) -> Option<Self> {
        let dividend = self.canonicalized_ref();
        let divisor = divisor.canonicalized_ref();
        if dividend.denominator != *ONE.deref()
            || divisor.denominator != *ONE.deref()
            || divisor.sign == NoSign
        {
            return None;
        }
        if dividend.sign == NoSign {
            return Some(Self::zero());
        }
        use num::Integer as _;
        let (quotient, remainder) = dividend.numerator.div_rem(&divisor.numerator);
        if !remainder.is_zero() {
            return None;
        }
        let sign = if dividend.sign == divisor.sign {
            Plus
        } else {
            Minus
        };
        crate::trace_dispatch!("rational", "integer-quotient", "exact-divisible");
        Some(Self::from_integer_magnitude(sign, quotient))
    }

    /// Return the exact mean `(left + right) / 2` without an intermediate sum rational.
    ///
    /// Canonicalize both inputs first. Equal values clone the canonical left
    /// handle. Checked word arithmetic handles eligible inputs; overflow falls
    /// back to arbitrary-width signed totals over twice the common denominator.
    ///
    /// For reduced a/b and c/d with g=gcd(b,d), only factors of 2g can cancel
    /// from the numerator over 2*lcm(b,d). The fallback uses that restricted
    /// reduction proof. Zero, negative and lazy inputs are supported; no rounding
    /// or predicate policy participates. Work, allocation and retained operand
    /// caches have no total budget here.
    pub fn average_pair(left: &Self, right: &Self) -> Self {
        // The restricted final GCD below requires coprime input parts.
        let left = left.canonicalized_ref();
        let right = right.canonicalized_ref();
        if left.sign == right.sign
            && left.numerator == right.numerator
            && left.denominator == right.denominator
        {
            crate::trace_dispatch!("rational", "average_pair", "equal");
            return left.clone();
        }

        if let (
            Some(left_numerator),
            Some(left_denominator),
            Some(right_numerator),
            Some(right_denominator),
        ) = (
            left.numerator.to_u128(),
            left.denominator.to_u128(),
            right.numerator.to_u128(),
            right.denominator.to_u128(),
        ) {
            let (left_scale, right_scale) =
                if left_denominator.is_power_of_two()
                    && right_denominator.is_power_of_two()
                {
                    let left_shift = left_denominator.trailing_zeros();
                    let right_shift = right_denominator.trailing_zeros();
                    let common_shift = left_shift.max(right_shift);
                    (
                        1_u128 << (common_shift - left_shift),
                        1_u128 << (common_shift - right_shift),
                    )
                } else {
                    let divisor =
                        Self::gcd_word(left_denominator, right_denominator);
                    (
                        right_denominator / divisor,
                        left_denominator / divisor,
                    )
                };
            if let (Some(left_magnitude), Some(right_magnitude), Some(denominator)) = (
                left_numerator.checked_mul(left_scale),
                right_numerator.checked_mul(right_scale),
                left_denominator
                    .checked_mul(left_scale)
                    .and_then(|denominator| denominator.checked_mul(2)),
            ) {
                let totals = (|| {
                    let mut positive = 0_u128;
                    let mut negative = 0_u128;
                    for (sign, magnitude) in [
                        (left.sign, left_magnitude),
                        (right.sign, right_magnitude),
                    ] {
                        match sign {
                            Plus => {
                                positive = positive.checked_add(magnitude)?;
                            }
                            Minus => {
                                negative = negative.checked_add(magnitude)?;
                            }
                            NoSign => {}
                        }
                    }
                    Some((positive, negative))
                })();
                if let Some((positive, negative)) = totals {
                    crate::trace_dispatch!("rational", "average_pair", "word-sized");
                    return Self::from_word_magnitude_difference(
                        positive,
                        negative,
                        denominator,
                    );
                }
            }
        }

        let common_denominator = Rational::gcd_magnitudes_with_mixed_width_fast_path(
            &left.denominator,
            &right.denominator,
        );
        trace_rational_gcd!(
            &left.denominator,
            &right.denominator,
            &common_denominator
        );
        let left_scale = &right.denominator / &common_denominator;
        let right_scale = &left.denominator / &common_denominator;
        let denominator = (&left.denominator * &left_scale) << 1_usize;
        let left_magnitude = &left.numerator * &left_scale;
        let right_magnitude = &right.numerator * &right_scale;
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for (sign, magnitude) in [
            (left.sign, left_magnitude),
            (right.sign, right_magnitude),
        ] {
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        crate::trace_dispatch!("rational", "average_pair", "arbitrary-precision");
        let (sign, numerator) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => return Self::zero(),
        };

        // With reduced inputs and g = gcd(b, d), the cross-sum numerator is
        // coprime to b/g and d/g. Only factors of 2g can therefore cancel
        // from the mean's denominator 2*lcm(b, d). Restricting reduction to
        // that proven divisor avoids a second GCD over the much wider LCM.
        let possible_divisor = &common_denominator << 1_usize;
        trace_rational_temporary!();
        Self::from_parts_raw(sign, numerator, denominator)
            .reduce_with_possible_divisor(&possible_divisor)
    }

    /// Combine two signed word magnitudes over a caller-proved reduced denominator.
    ///
    /// The denominator must be positive and the nonzero resulting numerator must
    /// be coprime to it. Integer-plus-fraction callers establish that invariant;
    /// this helper does not reduce the result. NoSign entries are ignored and
    /// exact cancellation returns canonical zero. A same-sign total exceeding
    /// u128 returns `None` to request the wider arithmetic path.
    fn from_reduced_word_sum(
        left_sign: Sign,
        left: u128,
        right_sign: Sign,
        right: u128,
        denominator: u128,
    ) -> Option<Self> {
        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for (sign, magnitude) in [(left_sign, left), (right_sign, right)] {
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        match positive.cmp(&negative) {
            core::cmp::Ordering::Greater => Some(Self::from_reduced_word_parts(
                Plus,
                positive - negative,
                denominator,
            )),
            core::cmp::Ordering::Less => Some(Self::from_reduced_word_parts(
                Minus,
                negative - positive,
                denominator,
            )),
            core::cmp::Ordering::Equal => Some(Self::zero()),
        }
    }

    /// Attempt reduced rational addition or subtraction entirely within u128 parts.
    ///
    /// Callers supply canonical valid operands. Integer-side shortcuts preserve
    /// the other denominator; general inputs align by dyadic shifts or denominator
    /// GCD and reduce the signed totals. `subtract` reverses the second sign.
    /// Unrepresentable parts, products or same-sign sums return `None`, not an
    /// arithmetic failure. This helper does not canonicalize or retain a pair.
    fn add_sub_words(&self, other: &Self, subtract: bool) -> Option<Self> {
        let left_denominator = self.denominator.to_u128()?;
        let right_denominator = other.denominator.to_u128()?;
        let right_sign = if subtract { -other.sign } else { other.sign };

        if right_denominator == 1 {
            let right = other
                .numerator
                .to_u128()?
                .checked_mul(left_denominator)?;
            return Self::from_reduced_word_sum(
                self.sign,
                self.numerator.to_u128()?,
                right_sign,
                right,
                left_denominator,
            );
        }
        if left_denominator == 1 {
            let left = self
                .numerator
                .to_u128()?
                .checked_mul(right_denominator)?;
            return Self::from_reduced_word_sum(
                self.sign,
                left,
                right_sign,
                other.numerator.to_u128()?,
                right_denominator,
            );
        }

        let (left_scale, right_scale, denominator) =
            if left_denominator.is_power_of_two()
                && right_denominator.is_power_of_two()
            {
                let left_shift = left_denominator.trailing_zeros();
                let right_shift = right_denominator.trailing_zeros();
                let common_shift = left_shift.max(right_shift);
                (
                    1_u128 << (common_shift - left_shift),
                    1_u128 << (common_shift - right_shift),
                    1_u128 << common_shift,
                )
            } else {
                let divisor =
                    Self::gcd_word(left_denominator, right_denominator);
                (
                    right_denominator / divisor,
                    left_denominator / divisor,
                    left_denominator
                        .checked_mul(right_denominator / divisor)?,
                )
            };
        let left = self.numerator.to_u128()?.checked_mul(left_scale)?;
        let right = other.numerator.to_u128()?.checked_mul(right_scale)?;
        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for (sign, magnitude) in [(self.sign, left), (right_sign, right)] {
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Add or subtract nonzero reduced dyadics using exact power-of-two alignment.
    ///
    /// The caller handles zero operands first. Marked unreduced or nondyadic
    /// inputs and unrepresentable shift metadata return `None`. Equal scales
    /// can expose powers of two to cancel; unequal reduced scales leave an odd
    /// sum numerator. Exact cancellation returns zero, otherwise construct the
    /// reduced result using word storage when possible.
    ///
    /// Magnitude clones/shifts can allocate without a local resource bound.
    /// Calling this helper directly on two zeros violates its nonzero invariant.
    fn add_sub_wide_dyadic(&self, other: &Self, subtract: bool) -> Option<Self> {
        let left_shift = self.dyadic_denominator_shift_if_reduced()?;
        let right_shift = other.dyadic_denominator_shift_if_reduced()?;
        let denominator_shift = left_shift.max(right_shift);
        let mut left = self.numerator.clone();
        let mut right = other.numerator.clone();
        let left_alignment = usize::try_from(denominator_shift - left_shift).ok()?;
        let right_alignment = usize::try_from(denominator_shift - right_shift).ok()?;
        if left_alignment != 0 {
            left <<= left_alignment;
        }
        if right_alignment != 0 {
            right <<= right_alignment;
        }

        let right_sign = if subtract { -other.sign } else { other.sign };
        let (sign, mut numerator) = if self.sign == right_sign {
            (self.sign, left + right)
        } else {
            match left.cmp(&right) {
                Ordering::Greater => (self.sign, left - right),
                Ordering::Less => (right_sign, right - left),
                Ordering::Equal => return Some(Self::zero()),
            }
        };

        // Unequal reduced dyadic scales align an odd numerator with an even
        // one, so their result is already reduced. Equal scales may expose a
        // shared power of two, which is removed with shifts alone.
        let common_shift = if left_shift == right_shift {
            numerator
                .trailing_zeros()
                .expect("nonzero dyadic sum has trailing zeros")
                .min(denominator_shift)
        } else {
            0
        };
        if common_shift != 0 {
            numerator >>= usize::try_from(common_shift).ok()?;
        }
        let result_denominator_shift = denominator_shift - common_shift;
        if result_denominator_shift < u64::from(u128::BITS)
            && let Some(magnitude) = numerator.to_u128()
        {
            return Some(Self::from_reduced_word_parts(
                sign,
                magnitude,
                1_u128 << result_denominator_shift,
            ));
        }

        let mut denominator = if left_shift >= right_shift {
            self.denominator.clone()
        } else {
            other.denominator.clone()
        };
        if common_shift != 0 {
            denominator >>= usize::try_from(common_shift).ok()?;
        }
        trace_rational_temporary!();
        Some(Self::from_parts_raw(sign, numerator, denominator))
    }

    /// Recognize simple coprimality proofs for a word numerator and positive denominator.
    ///
    /// Unit numerator/denominator or a power of two paired with an odd magnitude
    /// proves reducedness. False means this cheap test did not prove it, not
    /// that a common factor exists. The denominator validity precondition is
    /// trusted; this is not a general GCD or ratio validator.
    #[inline]
    fn word_parts_provably_reduced(numerator: u128, denominator: u128) -> bool {
        denominator == 1
            || numerator == 1
            || (denominator.is_power_of_two() && numerator & 1 == 1)
            || (numerator.is_power_of_two() && denominator & 1 == 1)
    }

    /// Attempt an exact nonzero product or quotient using checked u128 magnitudes.
    ///
    /// Both valid operands must be nonzero with positive denominators; division
    /// swaps the right magnitudes. Dyadic paths cancel powers of two, mixed paths
    /// reduce internal parts and cross-cancel, and the general path cross-cancels
    /// before multiplying. A recognized coprimality proof skips final reduction;
    /// otherwise the final word ratio is reduced. Valid unreduced parts can be
    /// handled here, although public operators canonicalize marked storage first.
    ///
    /// None means a part, shift or product does not fit this schedule. It is not
    /// resource exhaustion or a zero-divisor error; callers must reject zero
    /// before entry. Arithmetic dispatch may be traced and constructors may intern
    /// results, but this helper does not retain a binary operand pair.
    fn mul_div_words(&self, other: &Self, divide: bool) -> Option<Self> {
        let mut left_numerator = self.numerator.to_u128()?;
        let mut left_denominator = self.denominator.to_u128()?;
        let (mut right_numerator, mut right_denominator) = if divide {
            (other.denominator.to_u128()?, other.numerator.to_u128()?)
        } else {
            (other.numerator.to_u128()?, other.denominator.to_u128()?)
        };
        let inputs_provably_reduced = Self::word_parts_provably_reduced(
            left_numerator,
            left_denominator,
        ) && Self::word_parts_provably_reduced(right_numerator, right_denominator);

        if !divide
            && left_denominator.is_power_of_two()
            && right_denominator.is_power_of_two()
        {
            // Reduced dyadic operands need only power-of-two cancellation.
            // Avoid even binary-GCD loops for the imported-f64 products that
            // dominate exact mesh construction.
            let mut denominator_shift =
                left_denominator.trailing_zeros() + right_denominator.trailing_zeros();
            let left_cancel = left_numerator.trailing_zeros().min(denominator_shift);
            left_numerator >>= left_cancel;
            denominator_shift -= left_cancel;
            let right_cancel = right_numerator.trailing_zeros().min(denominator_shift);
            right_numerator >>= right_cancel;
            denominator_shift -= right_cancel;
            if denominator_shift >= u128::BITS {
                return None;
            }
            let numerator = left_numerator.checked_mul(right_numerator)?;
            let denominator = 1_u128 << denominator_shift;
            return Some(Self::from_reduced_word_parts(
                self.sign * other.sign,
                numerator,
                denominator,
            ));
        }

        if !divide
            && left_denominator.is_power_of_two() != right_denominator.is_power_of_two()
        {
            let (mut dyadic_numerator, dyadic_denominator, mut general_numerator, mut general_denominator) =
                if left_denominator.is_power_of_two() {
                    (
                        left_numerator,
                        left_denominator,
                        right_numerator,
                        right_denominator,
                    )
                } else {
                    (
                        right_numerator,
                        right_denominator,
                        left_numerator,
                        left_denominator,
                    )
                };
            let mut denominator_shift = dyadic_denominator.trailing_zeros();

            // Raw internal rationals are not required to be canonical. Strip
            // the only possible factor from the dyadic operand and fully
            // reduce the general operand before cross-cancelling them.
            let internal_power_cancel = dyadic_numerator.trailing_zeros().min(denominator_shift);
            dyadic_numerator >>= internal_power_cancel;
            denominator_shift -= internal_power_cancel;
            let internal_general = if general_numerator.is_power_of_two()
                && general_denominator & 1 == 1
            {
                1
            } else {
                Self::gcd_word(general_numerator, general_denominator)
            };
            general_numerator /= internal_general;
            general_denominator /= internal_general;

            let power_cancel = general_numerator.trailing_zeros().min(denominator_shift);
            general_numerator >>= power_cancel;
            denominator_shift -= power_cancel;

            let cross = if dyadic_numerator <= u128::from(u64::MAX) {
                // Binary64-derived dyadic numerators fit one word. Reduce the opposing wide
                // denominator once before the binary GCD so coprime vector
                // scales do not take a long u128 subtraction schedule.
                Self::gcd_word(
                    dyadic_numerator,
                    general_denominator % dyadic_numerator,
                )
            } else {
                Self::gcd_word(dyadic_numerator, general_denominator)
            };
            dyadic_numerator /= cross;
            general_denominator /= cross;
            let numerator = dyadic_numerator.checked_mul(general_numerator)?;
            let denominator_scale = 1_u128.checked_shl(denominator_shift)?;
            let denominator = general_denominator.checked_mul(denominator_scale)?;
            crate::trace_dispatch!("rational", "mul", "word-dyadic-general-cross-cancel");
            return Some(Self::from_reduced_word_parts(
                self.sign * other.sign,
                numerator,
                denominator,
            ));
        }

        let cross = Self::gcd_word(left_numerator, right_denominator);
        left_numerator /= cross;
        right_denominator /= cross;
        let cross = Self::gcd_word(right_numerator, left_denominator);
        right_numerator /= cross;
        left_denominator /= cross;

        let numerator = left_numerator.checked_mul(right_numerator)?;
        let denominator = left_denominator.checked_mul(right_denominator)?;
        let sign = self.sign * other.sign;
        if inputs_provably_reduced {
            crate::trace_dispatch!("rational", "mul-div", "proven-reduced-word-product");
            return Some(Self::from_reduced_word_parts(
                sign,
                numerator,
                denominator,
            ));
        }
        let (positive, negative) = if sign == Minus {
            (0, numerator)
        } else {
            (numerator, 0)
        };
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Multiply nonzero rational magnitudes when at least one denominator is dyadic.
    ///
    /// Marked unreduced inputs decline before shape queries; public multiplication
    /// canonicalizes them first. If neither denominator is dyadic, or shift
    /// metadata is ineligible, return `None`. Two dyadic operands cancel powers of two before
    /// multiplication. A mixed pair reduces internal parts and cross-cancels the
    /// remaining numerator/denominator pair before forming wide results.
    ///
    /// Successful results are exact and reduced. Shape queries can retain dyadic
    /// facts and products are traced. The fallback may allocate large magnitudes;
    /// None is schedule ineligibility, not a total work or memory limit.
    fn mul_wide_with_dyadic_denominator(&self, other: &Self) -> Option<Self> {
        if self.is_internally_unreduced() || other.is_internally_unreduced() {
            return None;
        }
        let left_dyadic = self.is_dyadic();
        let right_dyadic = other.is_dyadic();
        if !left_dyadic && !right_dyadic {
            return None;
        }

        if left_dyadic && right_dyadic {
            let mut denominator_shift = self
                .denominator
                .trailing_zeros()?
                .checked_add(other.denominator.trailing_zeros()?)?;
            let left_cancel = self.numerator.trailing_zeros()?.min(denominator_shift);
            denominator_shift -= left_cancel;
            let right_cancel = other.numerator.trailing_zeros()?.min(denominator_shift);
            denominator_shift -= right_cancel;
            let left_cancel = usize::try_from(left_cancel).ok()?;
            let right_cancel = usize::try_from(right_cancel).ok()?;
            let denominator_shift = usize::try_from(denominator_shift).ok()?;
            let left_numerator = &self.numerator >> left_cancel;
            let right_numerator = &other.numerator >> right_cancel;
            let numerator = Self::multiply_magnitudes(
                "multiplication-wide-dyadic",
                &left_numerator,
                &right_numerator,
            );
            let denominator = BigUint::one() << denominator_shift;
            crate::trace_dispatch!("rational", "mul", "wide-dyadic-cross-cancel");
            trace_rational_temporary!();
            return Some(Self::from_parts_raw(
                self.sign * other.sign,
                numerator,
                denominator,
            ));
        }

        // Reduce raw internal parts, cancel the opposing power of two with
        // shifts, then cancel the remaining cross pair before either wide
        // product is formed.
        let (dyadic, general) = if left_dyadic {
            (self, other)
        } else {
            (other, self)
        };
        let mut denominator_shift = dyadic.denominator.trailing_zeros()?;
        let dyadic_internal_cancel = dyadic.numerator.trailing_zeros()?.min(denominator_shift);
        denominator_shift -= dyadic_internal_cancel;
        let dyadic_internal_cancel = usize::try_from(dyadic_internal_cancel).ok()?;
        let dyadic_numerator = &dyadic.numerator >> dyadic_internal_cancel;

        let internal_general = if Self::is_power_of_two(&general.numerator)
            && general.denominator.bit(0)
        {
            BigUint::one()
        } else {
            let divisor = Self::gcd_magnitudes_with_mixed_width_fast_path(
                &general.numerator,
                &general.denominator,
            );
            trace_rational_gcd!(&general.numerator, &general.denominator, &divisor);
            divisor
        };
        let general_numerator = &general.numerator / &internal_general;
        let general_denominator = &general.denominator / &internal_general;
        let numerator_shift = general_numerator.trailing_zeros()?;
        let power_cancel = denominator_shift.min(numerator_shift);
        let remaining_denominator_shift = denominator_shift - power_cancel;
        let power_cancel = usize::try_from(power_cancel).ok()?;
        let remaining_denominator_shift = usize::try_from(remaining_denominator_shift).ok()?;

        let cross = Self::gcd_magnitudes(&dyadic_numerator, &general_denominator);
        let dyadic_numerator = dyadic_numerator / &cross;
        let general_denominator = general_denominator / &cross;
        let general_numerator = general_numerator >> power_cancel;
        let numerator = Rational::multiply_magnitudes(
            "multiplication-dyadic-general",
            &dyadic_numerator,
            &general_numerator,
        );
        let denominator = general_denominator << remaining_denominator_shift;
        crate::trace_dispatch!("rational", "mul", "dyadic-general-cross-cancel");
        trace_rational_temporary!();
        Some(Self::from_parts_raw(
            self.sign * other.sign,
            numerator,
            denominator,
        ))
    }

    /// Multiply nonzero reduced dyadics whose numerator product fits u128.
    ///
    /// Both denominator exponents must be available without promoting lazy input.
    /// Cancel numerator powers of two against their combined exponent before
    /// checked word multiplication. A small remaining exponent uses word parts;
    /// a wide exponent materializes a power-of-two BigUint denominator.
    ///
    /// Ineligible shape, exponent conversion or word product returns `None` for
    /// fallback. Zero operands must be handled earlier: a zero word's trailing
    /// zero count can make the shift invalid. The wide allocation has no budget.
    fn mul_wide_dyadic_with_word_numerators(&self, other: &Self) -> Option<Self> {
        let mut denominator_shift = self
            .dyadic_denominator_shift_if_reduced()?
            .checked_add(other.dyadic_denominator_shift_if_reduced()?)?;
        let mut left_numerator = self.numerator.to_u128()?;
        let mut right_numerator = other.numerator.to_u128()?;
        let left_cancel = u64::from(left_numerator.trailing_zeros()).min(denominator_shift);
        left_numerator >>= left_cancel;
        denominator_shift -= left_cancel;
        let right_cancel = u64::from(right_numerator.trailing_zeros()).min(denominator_shift);
        right_numerator >>= right_cancel;
        denominator_shift -= right_cancel;
        let numerator = left_numerator.checked_mul(right_numerator)?;
        let sign = self.sign * other.sign;

        if denominator_shift < u64::from(u128::BITS) {
            let denominator = 1_u128 << denominator_shift;
            crate::trace_dispatch!(
                "rational",
                "mul",
                "wide-dyadic-word-numerators-word-result"
            );
            return Some(Self::from_reduced_word_parts(sign, numerator, denominator));
        }

        let denominator_shift = usize::try_from(denominator_shift).ok()?;
        crate::trace_dispatch!("rational", "mul", "wide-dyadic-word-numerators");
        trace_rational_temporary!();
        Some(Self::from_parts_raw(
            sign,
            BigUint::from(numerator),
            BigUint::one() << denominator_shift,
        ))
    }

    /// Look up this owner's exact product with the same other payload.
    ///
    /// The primary product slot is checked first, then eligible linear-cache
    /// product slots. Keys are weak allocation identities rather than numeric
    /// equality, and a hit clones the retained result and emits its path trace.
    /// The installer is trusted to have supplied the exact relation.
    #[inline]
    fn retained_product(&self, other: &Self) -> Option<Self> {
        if let Some(cached) = self.product_cache.get()
            && cached.other.as_ref().is_some_and(|cached_other| {
                std::ptr::eq(cached_other.as_ptr(), Arc::as_ptr(&other.0))
            })
        {
            crate::trace_dispatch!("rational", "mul", "retained-product");
            return Some(cached.result.clone());
        }
        self.retained_secondary_product(other)
    }

    /// Find a product relation in the primary, secondary or tertiary linear slot.
    ///
    /// Each candidate must have Product kind and the other payload's address.
    /// Missing/mismatched entries return `None`; quaternary/quinary slots are
    /// reserved for other roles and are not searched. A hit clones the strong
    /// result without retaining the keyed operand or recomputing multiplication.
    #[cold]
    #[inline(never)]
    fn retained_secondary_product(&self, other: &Self) -> Option<Self> {
        let cached = self.linear_cache.get()?;
        let other_ptr = Arc::as_ptr(&other.0);
        let matches = |entry: &CachedRationalLinearEntry| {
            entry.kind == CachedRationalLinearKind::Product
                && std::ptr::eq(entry.other.as_ptr(), other_ptr)
        };
        let entry = if matches(&cached.primary) {
            Some(&cached.primary)
        } else if cached.secondary.get().is_some_and(matches) {
            cached.secondary.get()
        } else {
            cached.tertiary.get().filter(|entry| matches(entry))
        }?;
        crate::trace_dispatch!("rational", "mul", "retained-secondary-product");
        Some(entry.result.clone())
    }

    /// Classify primary-square availability for self-dot scheduling.
    ///
    /// Both units return `Some(true)` because their square is trivial; that
    /// identity check can canonicalize lazy input. Otherwise no primary product
    /// slot gives `None`, a key matching this owner gives `Some(true)`, and any
    /// other primary entry gives `Some(false)`. Secondary square entries are
    /// not inspected. This is cache state, not a numeric or completeness predicate.
    #[inline]
    pub(crate) fn retained_primary_self_product_state(&self) -> Option<bool> {
        if self.is_one() || self.is_minus_one() {
            return Some(true);
        }
        let primary = self.product_cache.get()?;
        Some(primary.other.as_ref().is_some_and(|other| {
            std::ptr::eq(other.as_ptr(), Arc::as_ptr(&self.0))
        }))
    }

    /// Recognize a trivial unit square or a retained product keyed by this owner.
    ///
    /// After the unit shortcut, a primary product slot must exist. Check its
    /// self-key, then Product entries in the linear primary/secondary/tertiary
    /// slots. False can mean missing/unrecognized evidence and does not deny the
    /// mathematical square. Lazy unit recognition may canonicalize.
    #[inline]
    pub(crate) fn has_retained_self_product(&self) -> bool {
        if self.is_one() || self.is_minus_one() {
            return true;
        }
        let Some(primary) = self.product_cache.get() else {
            return false;
        };
        let self_ptr = Arc::as_ptr(&self.0);
        if primary
            .other
            .as_ref()
            .is_some_and(|other| std::ptr::eq(other.as_ptr(), self_ptr))
        {
            return true;
        }
        let Some(cached) = self.linear_cache.get() else {
            return false;
        };
        let matches = |entry: &CachedRationalLinearEntry| {
            entry.kind == CachedRationalLinearKind::Product
                && std::ptr::eq(entry.other.as_ptr(), self_ptr)
        };
        matches(&cached.primary)
            || cached.secondary.get().is_some_and(matches)
            || cached.tertiary.get().is_some_and(matches)
    }

    /// Admit one self-dot retry while the owner's conflict-attempt bit is unset.
    ///
    /// The atomic observation sets that bit and returns true only for the first
    /// observer of this state. Later calls decline until a valid owner-state
    /// transition clears reuse flags. This schedules work; it proves neither
    /// that a square is cached nor that the attempted construction will fit.
    #[inline]
    pub(crate) fn admit_conflicted_self_dot_once(&self) -> bool {
        !self.observe_retained_fact(RETAINED_SELF_DOT_CONFLICT_ATTEMPTED)
    }

    /// Attempt to retain the caller-proved product on one of its operand owners.
    ///
    /// Try the left primary slot, then the right, skipping primary installation
    /// on marked unreduced payloads because that slot serves canonicalization.
    /// If neither succeeds, try the left then right linear Product slots.
    /// Keys are weak operands and results are strong handles; installation
    /// requires an exact relation and acyclic result ownership from the caller.
    ///
    /// This is best-effort write-once retention with no returned success flag,
    /// replacement or relation validation. Bounded slots do not bound retained
    /// magnitude/graph bytes or allocation failures.
    fn retain_product_pair(&self, other: &Self, result: &Self) {
        if !self.is_internally_unreduced()
            && self
                .product_cache
                .set(CachedRationalProduct {
                    other: Some(Arc::downgrade(&other.0)),
                    result: result.clone(),
                })
                .is_ok()
        {
            return;
        }
        if !other.is_internally_unreduced()
            && other
                .product_cache
                .set(CachedRationalProduct {
                    other: Some(Arc::downgrade(&self.0)),
                    result: result.clone(),
                })
                .is_ok()
        {
            return;
        }
        if !Self::retain_linear(self, other, CachedRationalLinearKind::Product, result)
        {
            let _ = Self::retain_linear(
                other,
                self,
                CachedRationalLinearKind::Product,
                result,
            );
        }
    }

    /// Lookup a directed arithmetic relation by owner, other identity and kind.
    ///
    /// Check linear primary, secondary then tertiary; a missing secondary or
    /// tertiary ends the scan. A matching entry clones the result and emits the
    /// supplied trace-path label. Only those slots are eligible. No numeric
    /// comparison or relation validation replaces the installer's proof.
    #[inline(always)]
    fn retained_linear(
        owner: &Self,
        other: &Self,
        kind: CachedRationalLinearKind,
        _path: &'static str,
    ) -> Option<Self> {
        let cached = owner.linear_cache.get()?;
        let other_ptr = Arc::as_ptr(&other.0);
        if cached.primary.kind == kind
            && std::ptr::eq(cached.primary.other.as_ptr(), other_ptr)
        {
            crate::trace_dispatch!("rational", "linear", _path);
            return Some(cached.primary.result.clone());
        }
        let secondary = cached.secondary.get()?;
        if secondary.kind == kind && std::ptr::eq(secondary.other.as_ptr(), other_ptr) {
            crate::trace_dispatch!("rational", "linear", _path);
            return Some(secondary.result.clone());
        }
        let tertiary = cached.tertiary.get()?;
        (tertiary.kind == kind && std::ptr::eq(tertiary.other.as_ptr(), other_ptr)).then(|| {
            crate::trace_dispatch!("rational", "linear", _path);
            tertiary.result.clone()
        })
    }

    /// Look up a sum first on self, then on the other operand by commutativity.
    ///
    /// Each lookup requires the same operand identities and Sum kind. Cache
    /// misses do not construct the sum or imply mathematical uncertainty.
    #[inline]
    fn retained_sum(&self, other: &Self) -> Option<Self> {
        Self::retained_linear(self, other, CachedRationalLinearKind::Sum, "retained-sum")
            .or_else(|| {
                Self::retained_linear(
                    other,
                    self,
                    CachedRationalLinearKind::Sum,
                    "retained-sum",
                )
            })
    }

    /// Look up self minus other with the correct direction on either owner.
    ///
    /// First try OwnerMinusOther on self, then OtherMinusOwner on other. A hit
    /// preserves the requested sign; matching values alone do not establish a
    /// cache correspondence. Misses leave arithmetic to the caller.
    #[inline]
    fn retained_difference(&self, other: &Self) -> Option<Self> {
        Self::retained_linear(
            self,
            other,
            CachedRationalLinearKind::OwnerMinusOther,
            "retained-difference",
        )
        .or_else(|| {
            Self::retained_linear(
                other,
                self,
                CachedRationalLinearKind::OtherMinusOwner,
                "retained-difference",
            )
        })
    }

    /// Attempt to install a caller-proved binary relation in eligible linear slots.
    ///
    /// With no cache, install a primary entry. An existing ordinary primary admits
    /// only a secondary; a placeholder primary admits secondary then tertiary.
    /// Occupied slots are not replaced or deduplicated. A lost outer-cache race
    /// returns false without retrying inside the winning cache.
    ///
    /// The other operand is weakly keyed and the result strongly retained. The
    /// caller owns kind/direction correctness and acyclic ownership. False reports
    /// slot/publication ineligibility, not invalid arithmetic or exhaustion.
    /// No equation verification or retained-byte limit is implemented here.
    #[inline]
    fn retain_linear(
        owner: &Self,
        other: &Self,
        kind: CachedRationalLinearKind,
        result: &Self,
    ) -> bool {
        if let Some(cached) = owner.linear_cache.get() {
            let primary_placeholder = cached.primary.kind.is_primary_placeholder();
            // Avoid cloning the result and touching both Arc counters when
            // every eligible write-once slot is already occupied.
            if cached.secondary.get().is_some()
                && (!primary_placeholder || cached.tertiary.get().is_some())
            {
                return false;
            }
            let entry = CachedRationalLinearEntry {
                other: Arc::downgrade(&other.0),
                kind,
                result: result.clone(),
            };
            if primary_placeholder {
                return cached
                    .secondary
                    .set(entry)
                    .or_else(|entry| cached.tertiary.set(entry))
                    .is_ok();
            }
            return cached.secondary.set(entry).is_ok();
        }
        owner
            .linear_cache
            .set(Box::new(CachedRationalArithmetic {
                primary: CachedRationalLinearEntry {
                    other: Arc::downgrade(&other.0),
                    kind,
                    result: result.clone(),
                },
                secondary: OnceLock::new(),
                tertiary: OnceLock::new(),
                quaternary: OnceLock::new(),
                quinary: OnceLock::new(),
                square_reduction: OnceLock::new(),
            }))
            .is_ok()
    }

    /// Observe whether an owner has enough arithmetic history to admit retention.
    ///
    /// Any product cache, linear cache or previous linear-observation bit returns
    /// true. Otherwise emit first-observation, set that bit, and return false.
    /// The separate read/set can let concurrent first callers both return false;
    /// it is not a one-winner admission primitive. Existing cache presence is
    /// scheduling evidence, not a numeric theorem or a strong-reference count.
    #[inline]
    pub(crate) fn has_arithmetic_reuse_evidence(&self) -> bool {
        // A shared value may have been cloned only for ownership. Admit a
        // binary linear cache after an arithmetic observation, or immediately
        // when another retained arithmetic result already proves reuse.
        if self.product_cache.get().is_some()
            || self.linear_cache.get().is_some()
            || self.retained_fact(RETAINED_LINEAR_REUSE_SEEN)
        {
            return true;
        }
        crate::trace_dispatch!("rational", "arithmetic-reuse", "first-observation");
        self.retain_fact(RETAINED_LINEAR_REUSE_SEEN);
        false
    }

    /// Forward to the arithmetic-history observation used for linear retention.
    ///
    /// This can record a first observation and return false. It neither checks
    /// sharing counts nor independently proves that the requested result is cached.
    #[inline]
    fn has_linear_reuse_evidence(&self) -> bool {
        self.has_arithmetic_reuse_evidence()
    }

    /// Retain a proved sum after arithmetic reuse has been observed on either operand.
    ///
    /// Observe self then other even if the first already qualifies. Prefer the
    /// eligible self owner, then try other. Cold operands merely learn observation
    /// bits. Installation is best effort and requires the correct exact relation
    /// and ownership discipline of the linear-cache helper.
    #[cold]
    #[inline(never)]
    fn retain_sum_pair(&self, other: &Self, result: &Self) {
        let self_shared = self.has_linear_reuse_evidence();
        let other_shared = other.has_linear_reuse_evidence();
        if !self_shared && !other_shared {
            return;
        }
        if self_shared
            && Self::retain_linear(self, other, CachedRationalLinearKind::Sum, result)
        {
            return;
        }
        if other_shared {
            let _ = Self::retain_linear(other, self, CachedRationalLinearKind::Sum, result);
        }
    }

    /// Retain a proved directed difference after observing both operands in order.
    ///
    /// Observe self then other; try OwnerMinusOther on eligible self before
    /// OtherMinusOwner on eligible other. Cold operands only learn reuse bits.
    /// The supplied result must have exactly that direction; retention does not
    /// recompute it or enforce a total retained-memory bound.
    #[cold]
    #[inline(never)]
    fn retain_difference_pair(&self, other: &Self, result: &Self) {
        let self_shared = self.has_linear_reuse_evidence();
        let other_shared = other.has_linear_reuse_evidence();
        if !self_shared && !other_shared {
            return;
        }
        if self_shared
            && Self::retain_linear(
            self,
            other,
            CachedRationalLinearKind::OwnerMinusOther,
            result,
        )
        {
            return;
        }
        if other_shared {
            let _ = Self::retain_linear(
                other,
                self,
                CachedRationalLinearKind::OtherMinusOwner,
                result,
            );
        }
    }
}

/// Shared exact addition core used by owned and borrowed operator adapters.
impl Rational {
    // Keep generic operator adapters from cloning this exact core into every consumer.
    /// Compute an exact sum through the shared nongeneric borrowed kernel.
    ///
    /// Zero identities precede canonicalization and can return original lazy
    /// handles. Other inputs canonicalize before one shortcuts, retained sums,
    /// checked words, wide dyadics, then general denominator-GCD alignment.
    /// For reduced inputs only the denominator GCD can cancel from the final
    /// cross-sum, permitting restricted reduction. Eligible results are offered
    /// for retention without changing value or operand order.
    ///
    /// No caller-supplied work, allocation or cancellation budget applies; word
    /// ineligibility falls through to wider exact arithmetic.
    #[inline(never)]
    fn add_ref(&self, other: &Self) -> Self {
        use std::cmp::Ordering::*;

        if self.sign == NoSign {
            return other.clone();
        }
        if other.sign == NoSign {
            return self.clone();
        }
        // The word and restricted-GCD kernels publish reduced results, so
        // they must consume the cached canonical view of lazy ratios.
        if self.is_internally_unreduced() || other.is_internally_unreduced() {
            return self.canonicalized_ref().add_ref(other.canonicalized_ref());
        }
        if self.is_one() {
            if other.is_one() {
                return Self::new(2);
            }
            return other.add_one();
        }
        if other.is_one() {
            return self.add_one();
        }
        if let Some(result) = self.retained_sum(other) {
            return result;
        }
        if let Some(result) = self.add_sub_words(other, false) {
            crate::trace_dispatch!("rational", "add", "word-sized");
            self.retain_sum_pair(other, &result);
            return result;
        }
        if let Some(result) = self.add_sub_wide_dyadic(other, false) {
            crate::trace_dispatch!("rational", "add", "wide-dyadic");
            self.retain_sum_pair(other, &result);
            return result;
        }
        let common_denominator = Rational::gcd_magnitudes_with_mixed_width_fast_path(
            &self.denominator,
            &other.denominator,
        );
        trace_rational_gcd!(&self.denominator, &other.denominator, &common_denominator);
        let left_scale = &other.denominator / &common_denominator;
        let right_scale = &self.denominator / &common_denominator;
        let denominator = &self.denominator * &left_scale;
        let a = &self.numerator * &left_scale;
        let b = &other.numerator * &right_scale;
        let (sign, numerator) = match (self.sign, other.sign) {
            (Plus, Plus) => (Plus, a + b),
            (Minus, Minus) => (Minus, a + b),
            (x, y) => match a.cmp(&b) {
                Greater => (x, a - b),
                Equal => {
                    let result = Self::zero();
                    self.retain_sum_pair(other, &result);
                    return result;
                }
                Less => (y, b - a),
            },
        };
        trace_rational_temporary!();
        let result = Self::from_parts_raw(sign, numerator, denominator)
            .reduce_with_possible_divisor(&common_denominator);
        self.retain_sum_pair(other, &result);
        result
    }
}

/// Borrowed exact addition accepting any right operand exposing a rational reference.
impl<T: AsRef<Rational>> Add<T> for &Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Rational;

    /// Borrow the right operand and delegate to the shared exact addition kernel.
    fn add(self, other: T) -> Self::Output {
        self.add_ref(other.as_ref())
    }
}

/// Owned-left exact addition using the same borrowed kernel and retention order.
impl<T: AsRef<Rational>> Add<T> for Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Self;

    /// Borrow the consumed left and right operands for exact addition; return an owned result.
    fn add(self, other: T) -> Self {
        &self + other.as_ref()
    }
}

/// Negation relation lookup and best-effort acyclic unary retention.
impl Rational {
    /// Extract a retained negation from the entry's strong or weak unary variant.
    ///
    /// Strong placeholders clone their result; weak placeholders upgrade their
    /// target and return None after expiry. Other kinds do not match. The caller
    /// trusts that this entry was installed as the owner's exact negative.
    #[inline]
    fn negation_from_entry(entry: &CachedRationalLinearEntry) -> Option<Self> {
        match entry.kind {
            CachedRationalLinearKind::StrongNegationPlaceholder => Some(entry.result.clone()),
            CachedRationalLinearKind::WeakNegationPlaceholder => entry.other.upgrade().map(Self),
            _ => None,
        }
    }

    /// Search negation evidence in primary, tertiary, quaternary then quinary slots.
    ///
    /// Expired weak entries are skipped so later candidates can still succeed;
    /// secondary is not a unary slot. A hit returns a handle without constructing
    /// new magnitudes. This helper does not emit the operator's hit trace itself.
    #[inline]
    fn retained_negation(&self) -> Option<Self> {
        let cached = self.linear_cache.get()?;
        if let Some(result) = Self::negation_from_entry(&cached.primary) {
            return Some(result);
        }
        if let Some(result) = cached
            .tertiary
            .get()
            .and_then(Self::negation_from_entry)
        {
            return Some(result);
        }
        cached
            .quaternary
            .get()
            .and_then(Self::negation_from_entry)
            .or_else(|| cached.quinary.get().and_then(Self::negation_from_entry))
    }

    /// Attempt write-once installation of a caller-proved strong or weak negative.
    ///
    /// An existing negation primary declines immediately. Other primary
    /// placeholders admit quaternary then quinary; ordinary primaries admit
    /// tertiary, quaternary then quinary. A new cache uses its primary slot.
    /// Weak entries carry an inert zero result; strong entries carry the target.
    ///
    /// False reports occupancy/publication failure, without replacement or outer
    /// race retry. The caller must establish the negative relation and prevent
    /// strong ownership cycles; slot count is not a retained-byte budget.
    fn retain_negation_entry(&self, negation: CachedRationalUnary) -> bool {
        if let Some(cached) = self.linear_cache.get() {
            if cached.primary.kind.is_negation_placeholder() {
                return false;
            }
            let entry = match negation {
                CachedRationalUnary::Strong(negation) => CachedRationalLinearEntry {
                    other: std::sync::Weak::new(),
                    kind: CachedRationalLinearKind::StrongNegationPlaceholder,
                    result: negation,
                },
                CachedRationalUnary::Weak(negation) => CachedRationalLinearEntry {
                    other: negation,
                    kind: CachedRationalLinearKind::WeakNegationPlaceholder,
                    result: RATIONAL_ZERO.clone(),
                },
            };
            if cached.primary.kind.is_primary_placeholder() {
                return cached
                    .quaternary
                    .set(entry)
                    .or_else(|entry| cached.quinary.set(entry))
                    .is_ok();
            }
            return cached
                .tertiary
                .set(entry)
                .or_else(|entry| cached.quaternary.set(entry))
                .or_else(|entry| cached.quinary.set(entry))
                .is_ok();
        }

        let (kind, other, placeholder) = match negation {
            CachedRationalUnary::Strong(negation) => (
                CachedRationalLinearKind::StrongNegationPlaceholder,
                std::sync::Weak::new(),
                negation,
            ),
            CachedRationalUnary::Weak(negation) => (
                CachedRationalLinearKind::WeakNegationPlaceholder,
                negation,
                RATIONAL_ZERO.clone(),
            ),
        };
        self.linear_cache
            .set(Box::new(CachedRationalArithmetic {
                primary: CachedRationalLinearEntry {
                    other,
                    kind,
                    result: placeholder,
                },
                secondary: OnceLock::new(),
                tertiary: OnceLock::new(),
                quaternary: OnceLock::new(),
                quinary: OnceLock::new(),
                square_reduction: OnceLock::new(),
            }))
            .is_ok()
    }

    /// Install a weak backlink on a fresh negative before a strong forward link.
    ///
    /// The caller supplies the exact newly constructed negative. This order
    /// permits repeated negation to recover the original owner without making a
    /// two-way strong cycle. Each write is independently best effort; existing
    /// cache layouts can decline either link without changing the value.
    #[cold]
    fn retain_negation_pair(&self, negation: &Self) {
        let _ = negation
            .retain_negation_entry(CachedRationalUnary::Weak(Arc::downgrade(&self.0)));
        let _ = self.retain_negation_entry(CachedRationalUnary::Strong(negation.clone()));
    }
}

/// Exact borrowed negation with shared unary-result retention.
impl Neg for &Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Rational;

    /// Preserve zero identity; clone lazy magnitudes with the sign reversed.
    ///
    /// Canonical units use shared opposite units. Other canonical values reuse
    /// a retained negative or allocate one and attempt a weak-back/strong-forward
    /// pair. Lazy input is not reduced here and does not install a negative pair.
    /// The source remains exact; there is no total allocation/work budget.
    #[inline]
    fn neg(self) -> Self::Output {
        if self.sign == NoSign {
            return self.clone();
        }
        if self.is_internally_unreduced() {
            return Self::Output::from_parts_raw_unreduced(
                -self.sign,
                self.numerator.clone(),
                self.denominator.clone(),
            );
        }
        if self.is_one() {
            return Self::Output::minus_one();
        }
        if self.is_minus_one() {
            return Self::Output::one();
        }
        if let Some(result) = self.retained_negation() {
            crate::trace_dispatch!("rational", "neg", "retained");
            return result;
        }
        trace_rational_temporary!();
        let result = Self::Output::from_parts_raw(
            -self.sign,
            self.numerator.clone(),
            self.denominator.clone(),
        );
        self.retain_negation_pair(&result);
        result
    }
}

/// Exact owned negation, reusing uniquely owned storage only after cache invalidation.
impl Neg for Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Self;

    /// Negate the value, preferring zero/unit and retained-negative shortcuts.
    ///
    /// Unit checks can canonicalize lazy input. If Arc::get_mut succeeds, flip
    /// the sign in place, discard product/linear caches and clear reuse bits;
    /// sign-independent facts and the lazy marker remain. Outstanding strong or
    /// weak aliases prevent that mutation and route through borrowed negation.
    /// No arithmetic evidence for the old sign survives the in-place path.
    #[inline]
    fn neg(mut self) -> Self {
        if self.sign == NoSign {
            return self;
        }
        if self.is_one() {
            return Self::minus_one();
        }
        if self.is_minus_one() {
            return Self::one();
        }
        if let Some(result) = self.retained_negation() {
            crate::trace_dispatch!("rational", "neg", "retained");
            return result;
        }
        if let Some(data) = Arc::get_mut(&mut self.0) {
            data.sign = -data.sign;
            // Unary negation changes the cache key represented by this node.
            // A unique owner can reuse the BigUint allocation, but any locally
            // retained arithmetic results describe the old sign and must go.
            data.product_cache.take();
            data.linear_cache.clear();
            data.retained_facts.fetch_and(
                !RETAINED_REUSE_MASK,
                std::sync::atomic::Ordering::Relaxed,
            );
            return self;
        }
        -&self
    }
}

/// Shared exact subtraction core preserving directed arithmetic and cache order.
impl Rational {
    // Keep generic operator adapters from cloning this exact core into every consumer.
    /// Compute an exact directed difference through the shared borrowed kernel.
    ///
    /// Zero identities precede lazy canonicalization. Unit shortcuts precede
    /// retained differences, word arithmetic, wide dyadics and general alignment.
    /// For canonical inputs, only the denominator GCD can cancel from the final
    /// cross-difference. Cancellation yields exact zero; eligible results retain
    /// their directed relation on an operand owner.
    ///
    /// No rounding, predicate policy or total work/memory limit participates;
    /// an ineligible fast path falls back to wider exact arithmetic.
    #[inline(never)]
    fn subtract_ref(&self, other: &Self) -> Self {
        use std::cmp::Ordering::*;

        if other.sign == NoSign {
            return self.clone();
        }
        if self.sign == NoSign {
            return -other;
        }
        if self.is_internally_unreduced() || other.is_internally_unreduced() {
            return self.canonicalized_ref().subtract_ref(other.canonicalized_ref());
        }
        if other.is_one() {
            return self.subtract_one();
        }
        if self.is_one() {
            return -other.subtract_one();
        }
        if let Some(result) = self.retained_difference(other) {
            return result;
        }
        if let Some(result) = self.add_sub_words(other, true) {
            crate::trace_dispatch!("rational", "sub", "word-sized");
            self.retain_difference_pair(other, &result);
            return result;
        }
        if let Some(result) = self.add_sub_wide_dyadic(other, true) {
            crate::trace_dispatch!("rational", "sub", "wide-dyadic");
            self.retain_difference_pair(other, &result);
            return result;
        }
        let common_denominator = Rational::gcd_magnitudes_with_mixed_width_fast_path(
            &self.denominator,
            &other.denominator,
        );
        trace_rational_gcd!(&self.denominator, &other.denominator, &common_denominator);
        let left_scale = &other.denominator / &common_denominator;
        let right_scale = &self.denominator / &common_denominator;
        let denominator = &self.denominator * &left_scale;
        let a = &self.numerator * &left_scale;
        let b = &other.numerator * &right_scale;
        let (sign, numerator) = match (self.sign, other.sign) {
            (Plus, Minus) => (Plus, a + b),
            (Minus, Plus) => (Minus, a + b),
            (x, y) => match a.cmp(&b) {
                Greater => (x, a - b),
                Equal => {
                    let result = Self::zero();
                    self.retain_difference_pair(other, &result);
                    return result;
                }
                Less => (-y, b - a),
            },
        };
        trace_rational_temporary!();
        let result = Self::from_parts_raw(sign, numerator, denominator)
            .reduce_with_possible_divisor(&common_denominator);
        self.retain_difference_pair(other, &result);
        result
    }
}

/// Borrowed exact subtraction with a directed result and shared arithmetic authority.
impl<T: AsRef<Rational>> Sub<T> for &Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Rational;

    /// Borrow the right operand and delegate to the exact directed subtraction kernel.
    fn sub(self, other: T) -> Self::Output {
        self.subtract_ref(other.as_ref())
    }
}

/// Owned-left exact subtraction using the borrowed directed kernel.
impl<T: AsRef<Rational>> Sub<T> for Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Self;

    /// Borrow both operand views for subtraction without changing their requested order.
    fn sub(self, other: T) -> Self {
        &self - other.as_ref()
    }
}

/// Borrowed exact multiplication with shape-specialized kernels and relation reuse.
impl<T: AsRef<Rational>> Mul<T> for &Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Rational;

    /// Multiply after zero, lazy-canonicalization and signed-unit shortcuts.
    ///
    /// Then try retained products in left/right order, reciprocal identity,
    /// word cross-cancellation, wide dyadic word numerators and wide dyadic/mixed
    /// arithmetic. The general fallback forms magnitude products and reduces.
    /// Eligible results retain the exact product on one owner. No rounding or
    /// total work/memory budget replaces an ineligible specialized path.
    fn mul(self, other: T) -> Self::Output {
        let other = other.as_ref();
        let sign = self.sign * other.sign;
        if sign == NoSign {
            return Self::Output::zero();
        }
        if self.is_internally_unreduced() || other.is_internally_unreduced() {
            return self.canonicalized_ref() * other.canonicalized_ref();
        }
        if self.is_one() {
            return other.clone();
        }
        if other.is_one() {
            return self.clone();
        }
        if self.is_minus_one() {
            return -other;
        }
        if other.is_minus_one() {
            return -self;
        }
        if let Some(result) = self
            .retained_product(other)
            .or_else(|| other.retained_product(self))
        {
            return result;
        }
        if self.numerator == other.denominator && self.denominator == other.numerator {
            return if sign == Minus {
                Self::Output::minus_one()
            } else {
                Self::Output::one()
            };
        }
        if let Some(result) = self.mul_div_words(other, false) {
            crate::trace_dispatch!("rational", "mul", "word-sized");
            self.retain_product_pair(other, &result);
            return result;
        }
        if let Some(result) = self.mul_wide_dyadic_with_word_numerators(other) {
            self.retain_product_pair(other, &result);
            return result;
        }
        if let Some(result) = self.mul_wide_with_dyadic_denominator(other) {
            self.retain_product_pair(other, &result);
            return result;
        }
        let numerator = Rational::multiply_magnitudes(
            "multiplication-numerator",
            &self.numerator,
            &other.numerator,
        );
        let denominator = Rational::multiply_magnitudes(
            "multiplication-denominator",
            &self.denominator,
            &other.denominator,
        );
        trace_rational_temporary!();
        let result = Self::Output::maybe_reduce(Self::Output::from_parts_raw(
            sign,
            numerator,
            denominator,
        ));
        self.retain_product_pair(other, &result);
        result
    }
}

/// Owned-left exact multiplication through the same borrowed arithmetic schedule.
impl<T: AsRef<Rational>> Mul<T> for Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Self;

    /// Borrow both operand views and use exact multiplication, including its retention effects.
    fn mul(self, other: T) -> Self {
        &self * other.as_ref()
    }
}

/// Exact multiplication assignment by replacing the left handle with its product.
impl<T: AsRef<Rational>> MulAssign<T> for Rational {
    /// Compute the borrowed exact product before replacing self; existing aliases keep their old value.
    fn mul_assign(&mut self, other: T) {
        *self = &*self * other.as_ref();
    }
}

/// Maximum number of original nontrivial factors in one pairwise product chunk.
///
/// This bounds vector occupancy, not factor bytes, intermediate products or
/// dependency graphs retained by arithmetic caches. Binary partials add O(log n)
/// slots beyond this chunk; no total resource limit is enforced here.
const BALANCED_PRODUCT_CHUNK: usize = 512;

/// Multiply one pair and emit the aggregate-product trace at most once.
///
/// The caller shares `traced` across a reduction. Its first false value
/// records balanced-pairwise and becomes true; every call still uses the
/// ordinary exact multiplication kernel, including its caches and traces.
#[inline]
fn balanced_product_pair(left: &Rational, right: &Rational, traced: &mut bool) -> Rational {
    if !*traced {
        crate::trace_dispatch!("rational", "product", "balanced-pairwise");
        *traced = true;
    }
    left * right
}

/// Consume a factor vector by in-place adjacent pairwise reduction.
///
/// Each pass writes products to the front, carries an odd last factor and
/// truncates obsolete handles; the final pop leaves the vector empty while
/// retaining its capacity. Empty input returns None, singleton input returns
/// its original handle. Length strictly decreases each nontrivial pass.
/// This helper does not itself enforce the caller's 512-factor chunk size.
fn reduce_rational_product_chunk(
    factors: &mut Vec<Rational>,
    traced: &mut bool,
) -> Option<Rational> {
    while factors.len() > 1 {
        let length = factors.len();
        let mut read = 0;
        let mut write = 0;
        while read + 1 < length {
            let product = balanced_product_pair(&factors[read], &factors[read + 1], traced);
            factors[write] = product;
            read += 2;
            write += 1;
        }
        if read < length {
            if read != write {
                factors.swap(read, write);
            }
            write += 1;
        }
        factors.truncate(write);
    }
    factors.pop()
}

/// Carry one completed chunk product through occupied binary partial levels.
///
/// Slot n represents a product of 2^n full chunks. Take occupied slots and
/// multiply their earlier left block by the incoming later block until an
/// empty slot or a new top slot can retain the carry. The caller establishes
/// that block invariant. Vector/result allocations have no byte/work budget.
fn push_rational_product_partial(
    partials: &mut Vec<Option<Rational>>,
    mut product: Rational,
    traced: &mut bool,
) {
    let mut level = 0;
    loop {
        if level == partials.len() {
            partials.push(Some(product));
            return;
        }
        if let Some(left) = partials[level].take() {
            product = balanced_product_pair(&left, &product, traced);
            level += 1;
        } else {
            partials[level] = Some(product);
            return;
        }
    }
}

/// Consume an iterator into an exact product using bounded-count chunks.
///
/// Skip ones, hold one nontrivial pending factor before allocating a vector,
/// and reduce groups of at most 512 factors pairwise. Binary partial slots
/// combine full chunks; low-to-high final merging places the accumulated
/// trailing block on the left. Multiplication is commutative in value, but
/// this schedule determines cache probes and dispatch traces.
///
/// Empty/all-one input returns one; a lone factor other than one keeps its handle.
/// Zero clears accumulated factors and prevents further arithmetic, but the
/// iterator is still fully consumed, so later producer side effects or panics
/// remain observable and an infinite iterator does not complete.
///
/// The wrapper retains O(log n) partial products and at most 512 original
/// factor slots, not bounded bytes or construction graphs. Arithmetic caches
/// can retain dependencies. No total work, cancellation or typed exhaustion
/// contract is added by this reduction.
fn balanced_rational_product<I>(iter: I) -> Rational
where
    I: Iterator<Item = Rational>,
{
    let initial_chunk_capacity = iter.size_hint().0.min(BALANCED_PRODUCT_CHUNK);
    let mut chunk = Vec::new();
    let mut pending_factor = None;
    // Slot n holds either no value or a product of 2^n full chunks. Chunking
    // keeps the hot inner tree contiguous while retaining only O(log n)
    // partial products and at most 512 original factors.
    let mut partials: Vec<Option<Rational>> = Vec::new();
    let mut has_zero = false;
    let mut traced = false;

    for factor in iter {
        if has_zero {
            continue;
        }
        if factor.is_zero() {
            pending_factor = None;
            chunk.clear();
            partials.clear();
            has_zero = true;
            continue;
        }
        if factor.is_one() {
            continue;
        }
        if chunk.capacity() == 0 {
            let Some(first) = pending_factor.take() else {
                pending_factor = Some(factor);
                continue;
            };
            chunk = Vec::with_capacity(initial_chunk_capacity.max(2));
            chunk.push(first);
        }
        chunk.push(factor);
        if chunk.len() == BALANCED_PRODUCT_CHUNK {
            let product = reduce_rational_product_chunk(&mut chunk, &mut traced)
                .expect("a full product chunk is nonempty");
            push_rational_product_partial(&mut partials, product, &mut traced);
        }
    }

    if has_zero {
        return Rational::zero();
    }
    if let Some(factor) = pending_factor {
        return factor;
    }

    if let Some(product) = reduce_rational_product_chunk(&mut chunk, &mut traced) {
        push_rational_product_partial(&mut partials, product, &mut traced);
    }

    // Low-to-high merging combines the short trailing blocks before the
    // largest leading block, avoiding a final long chain for non-powers of 2.
    let mut result = None;
    for leading in partials.into_iter().flatten() {
        result = Some(match result {
            Some(trailing) => balanced_product_pair(&trailing, &leading, &mut traced),
            None => leading,
        });
    }
    result.unwrap_or_else(Rational::one)
}

/// Exact owned iterator products using pairwise chunks and binary partials.
impl std::iter::Product for Rational {
    /// Consume all factors through the shared balanced reduction.
    ///
    /// Empty input returns one. Zero yields zero after the remaining iterator is
    /// consumed; producer effects and panics still occur. Factor-count chunking
    /// does not bound magnitude sizes or retained arithmetic graphs.
    fn product<I: Iterator<Item = Self>>(iter: I) -> Self {
        balanced_rational_product(iter)
    }
}

/// Exact borrowed iterator products sharing the owned reduction schedule.
impl<'a> std::iter::Product<&'a Rational> for Rational {
    /// Clone each borrowed handle into the shared balanced product reducer.
    ///
    /// Source magnitudes are not cloned merely to iterate. Empty input returns
    /// one, and a zero factor does not stop consumption of later iterator items.
    /// The source and any retained arithmetic evidence keep their exact values.
    fn product<I: Iterator<Item = &'a Rational>>(iter: I) -> Self {
        balanced_rational_product(iter.cloned())
    }
}

/// Exact borrowed rational division; zero divisors panic at this operator boundary.
impl<T: AsRef<Rational>> Div<T> for &Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Rational;

    /// Divide by a nonzero rational exactly, with no typed error result.
    ///
    /// Assert a nonzero divisor numerator before the zero-dividend shortcut,
    /// so both 0/0 and nonzero/0 panic. Then canonicalize lazy operands and try
    /// signed-unit/equal-magnitude identities, checked word cross-cancellation,
    /// reciprocal squaring and general cross-products with reduction.
    ///
    /// Unlike multiplication this operator installs no binary result relation.
    /// It can still canonicalize/cache operand evidence. Wide arithmetic has no
    /// total work, allocation or cancellation budget; use a checked integer
    /// shortcut only when its narrower integer contract is appropriate.
    fn div(self, other: T) -> Self::Output {
        let other = other.as_ref();
        assert_ne!(other.numerator, BigUint::ZERO);
        let sign = self.sign * other.sign;
        if sign == NoSign {
            return Self::Output::zero();
        }
        if self.is_internally_unreduced() || other.is_internally_unreduced() {
            return self.canonicalized_ref() / other.canonicalized_ref();
        }
        if other.is_one() {
            return self.clone();
        }
        if other.is_minus_one() {
            return -self;
        }
        if self.numerator == other.numerator && self.denominator == other.denominator {
            return if sign == Minus {
                Self::Output::minus_one()
            } else {
                Self::Output::one()
            };
        }
        if let Some(result) = self.mul_div_words(other, true) {
            crate::trace_dispatch!("rational", "div", "word-sized");
            return result;
        }
        if self.numerator == other.denominator && self.denominator == other.numerator {
            trace_rational_temporary!();
            return Self::Output::maybe_reduce(Self::Output::from_parts_raw(
                sign,
                Rational::multiply_magnitudes(
                    "division-reciprocal-numerator",
                    &self.numerator,
                    &self.numerator,
                ),
                Rational::multiply_magnitudes(
                    "division-reciprocal-denominator",
                    &self.denominator,
                    &self.denominator,
                ),
            ));
        }
        let numerator = Rational::multiply_magnitudes(
            "division-cross-numerator",
            &self.numerator,
            &other.denominator,
        );
        let denominator = Rational::multiply_magnitudes(
            "division-cross-denominator",
            &self.denominator,
            &other.numerator,
        );
        trace_rational_temporary!();
        Self::Output::maybe_reduce(Self::Output::from_parts_raw(sign, numerator, denominator))
    }
}

/// Owned-left exact division using the borrowed kernel and its zero-divisor panic contract.
impl<T: AsRef<Rational>> Div<T> for Rational {
    /// Owned exact rational result, possibly sharing an existing immutable payload.
    type Output = Self;

    /// Borrow both operands for exact division; a zero right operand panics, including for zero self.
    fn div(self, other: T) -> Self {
        &self / other.as_ref()
    }
}
