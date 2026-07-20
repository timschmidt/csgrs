use core::ops::*;

impl Rational {
    /// Return the exact arithmetic mean of two rationals.
    ///
    /// This delays canonical reduction until after both the addition and the
    /// division by two. It is equivalent to `(left + right) / 2`, but avoids
    /// materializing and reducing the intermediate sum.
    pub fn average_pair(left: &Self, right: &Self) -> Self {
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
        Self::from_signed_magnitude_difference(positive, negative, denominator)
    }

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

    #[inline]
    fn word_parts_provably_reduced(numerator: u128, denominator: u128) -> bool {
        denominator == 1
            || numerator == 1
            || (denominator.is_power_of_two() && numerator & 1 == 1)
            || (numerator.is_power_of_two() && denominator & 1 == 1)
    }

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

    fn mul_wide_with_dyadic_denominator(&self, other: &Self) -> Option<Self> {
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

    fn mul_wide_dyadic_with_word_numerators(&self, other: &Self) -> Option<Self> {
        let mut denominator_shift = self
            .dyadic_denominator_shift()?
            .checked_add(other.dyadic_denominator_shift()?)?;
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

    #[inline]
    fn retained_product(&self, other: &Self) -> Option<Self> {
        let cached = self.product_cache.get()?;
        std::ptr::eq(cached.other.as_ptr(), Arc::as_ptr(&other.0)).then(|| {
            crate::trace_dispatch!("rational", "mul", "retained-product");
            cached.result.clone()
        })
    }

    fn retain_product_pair(&self, other: &Self, result: &Self) {
        let _ = self.product_cache.set(CachedRationalProduct {
            other: Arc::downgrade(&other.0),
            result: result.clone(),
        });
        let _ = other.product_cache.set(CachedRationalProduct {
            other: Arc::downgrade(&self.0),
            result: result.clone(),
        });
    }

    #[inline]
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

    #[inline]
    fn retain_linear(
        owner: &Self,
        other: &Self,
        kind: CachedRationalLinearKind,
        result: &Self,
    ) -> bool {
        if let Some(cached) = owner.linear_cache.get() {
            let entry = CachedRationalLinearEntry {
                other: Arc::downgrade(&other.0),
                kind,
                result: result.clone(),
            };
            if cached.primary.kind.is_primary_placeholder() {
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

    #[inline]
    pub(crate) fn has_arithmetic_reuse_evidence(&self) -> bool {
        if Arc::strong_count(&self.0) > 1
            || self.product_cache.get().is_some()
            || self.linear_cache.get().is_some()
            || self.retained_fact(RETAINED_LINEAR_REUSE_SEEN)
        {
            return true;
        }
        crate::trace_dispatch!("rational", "arithmetic-reuse", "first-observation");
        self.retain_fact(RETAINED_LINEAR_REUSE_SEEN);
        false
    }

    #[inline]
    fn has_linear_reuse_evidence(&self) -> bool {
        self.has_arithmetic_reuse_evidence()
    }

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

impl<T: AsRef<Rational>> Add<T> for &Rational {
    type Output = Rational;

    fn add(self, other: T) -> Self::Output {
        use std::cmp::Ordering::*;

        let other = other.as_ref();
        if self.sign == NoSign {
            return other.clone();
        }
        if other.sign == NoSign {
            return self.clone();
        }
        if self.is_one() {
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
                    let result = Self::Output::zero();
                    self.retain_sum_pair(other, &result);
                    return result;
                }
                Less => (y, b - a),
            },
        };
        trace_rational_temporary!();
        let result = Self::Output::from_parts_raw(sign, numerator, denominator)
            .reduce_with_possible_divisor(&common_denominator);
        self.retain_sum_pair(other, &result);
        result
    }
}

impl<T: AsRef<Rational>> Add<T> for Rational {
    type Output = Self;

    fn add(self, other: T) -> Self {
        &self + other.as_ref()
    }
}

impl Rational {
    #[inline]
    fn negation_from_entry(entry: &CachedRationalLinearEntry) -> Option<Self> {
        match entry.kind {
            CachedRationalLinearKind::StrongNegationPlaceholder => Some(entry.result.clone()),
            CachedRationalLinearKind::WeakNegationPlaceholder => entry.other.upgrade().map(Self),
            _ => None,
        }
    }

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

    #[cold]
    fn retain_negation_pair(&self, negation: &Self) {
        let _ = negation
            .retain_negation_entry(CachedRationalUnary::Weak(Arc::downgrade(&self.0)));
        let _ = self.retain_negation_entry(CachedRationalUnary::Strong(negation.clone()));
    }
}

impl Neg for &Rational {
    type Output = Rational;

    #[inline]
    fn neg(self) -> Self::Output {
        if self.sign == NoSign {
            return self.clone();
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

impl Neg for Rational {
    type Output = Self;

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
            data.linear_cache.take();
            data.retained_facts.fetch_and(
                !RETAINED_REUSE_MASK,
                std::sync::atomic::Ordering::Relaxed,
            );
            return self;
        }
        -&self
    }
}

impl<T: AsRef<Rational>> Sub<T> for &Rational {
    type Output = Rational;

    fn sub(self, other: T) -> Self::Output {
        use std::cmp::Ordering::*;

        let other = other.as_ref();
        if other.sign == NoSign {
            return self.clone();
        }
        if self.sign == NoSign {
            return -other;
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
                    let result = Self::Output::zero();
                    self.retain_difference_pair(other, &result);
                    return result;
                }
                Less => (-y, b - a),
            },
        };
        trace_rational_temporary!();
        let result = Self::Output::from_parts_raw(sign, numerator, denominator)
            .reduce_with_possible_divisor(&common_denominator);
        self.retain_difference_pair(other, &result);
        result
    }
}

impl<T: AsRef<Rational>> Sub<T> for Rational {
    type Output = Self;

    fn sub(self, other: T) -> Self {
        &self - other.as_ref()
    }
}

impl<T: AsRef<Rational>> Mul<T> for &Rational {
    type Output = Rational;

    fn mul(self, other: T) -> Self::Output {
        let other = other.as_ref();
        let sign = self.sign * other.sign;
        if sign == NoSign {
            return Self::Output::zero();
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

impl<T: AsRef<Rational>> Mul<T> for Rational {
    type Output = Self;

    fn mul(self, other: T) -> Self {
        &self * other.as_ref()
    }
}

impl<T: AsRef<Rational>> MulAssign<T> for Rational {
    fn mul_assign(&mut self, other: T) {
        *self = &*self * other.as_ref();
    }
}

impl<T: AsRef<Rational>> Div<T> for &Rational {
    type Output = Rational;

    fn div(self, other: T) -> Self::Output {
        let other = other.as_ref();
        assert_ne!(other.numerator, BigUint::ZERO);
        let sign = self.sign * other.sign;
        if sign == NoSign {
            return Self::Output::zero();
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
            return Self::Output::from_parts_raw(
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
            );
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

impl<T: AsRef<Rational>> Div<T> for Rational {
    type Output = Self;

    fn div(self, other: T) -> Self {
        &self / other.as_ref()
    }
}
