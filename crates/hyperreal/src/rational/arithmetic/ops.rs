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

        let common_denominator =
            num::Integer::gcd(&left.denominator, &right.denominator);
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

    fn mul_div_words(&self, other: &Self, divide: bool) -> Option<Self> {
        let mut left_numerator = self.numerator.to_u128()?;
        let mut left_denominator = self.denominator.to_u128()?;
        let (mut right_numerator, mut right_denominator) = if divide {
            (other.denominator.to_u128()?, other.numerator.to_u128()?)
        } else {
            (other.numerator.to_u128()?, other.denominator.to_u128()?)
        };

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

        let cross = Self::gcd_word(left_numerator, right_denominator);
        left_numerator /= cross;
        right_denominator /= cross;
        let cross = Self::gcd_word(right_numerator, left_denominator);
        right_numerator /= cross;
        left_denominator /= cross;

        let numerator = left_numerator.checked_mul(right_numerator)?;
        let denominator = left_denominator.checked_mul(right_denominator)?;
        let sign = self.sign * other.sign;
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
        if let Some(result) = self.add_sub_words(other, false) {
            crate::trace_dispatch!("rational", "add", "word-sized");
            return result;
        }
        let common_denominator = num::Integer::gcd(&self.denominator, &other.denominator);
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
                    return Self::Output::zero();
                }
                Less => (y, b - a),
            },
        };
        trace_rational_temporary!();
        Self::Output::from_parts_raw(sign, numerator, denominator)
        .reduce_with_possible_divisor(&common_denominator)
    }
}

impl<T: AsRef<Rational>> Add<T> for Rational {
    type Output = Self;

    fn add(self, other: T) -> Self {
        &self + other.as_ref()
    }
}

impl Neg for &Rational {
    type Output = Rational;

    fn neg(self) -> Self::Output {
        if self.is_one() {
            return Self::Output::minus_one();
        }
        if self.is_minus_one() {
            return Self::Output::one();
        }
        trace_rational_temporary!();
        Self::Output::from_parts_raw(
            -self.sign,
            self.numerator.clone(),
            self.denominator.clone(),
        )
    }
}

impl Neg for Rational {
    type Output = Self;

    fn neg(mut self) -> Self {
        if let Some(data) = Arc::get_mut(&mut self.0) {
            data.sign = -data.sign;
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
        if let Some(result) = self.add_sub_words(other, true) {
            crate::trace_dispatch!("rational", "sub", "word-sized");
            return result;
        }
        let common_denominator = num::Integer::gcd(&self.denominator, &other.denominator);
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
                    return Self::Output::zero();
                }
                Less => (-y, b - a),
            },
        };
        trace_rational_temporary!();
        Self::Output::from_parts_raw(sign, numerator, denominator)
        .reduce_with_possible_divisor(&common_denominator)
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
        if self.numerator == other.denominator && self.denominator == other.numerator {
            return if sign == Minus {
                Self::Output::minus_one()
            } else {
                Self::Output::one()
            };
        }
        if let Some(result) = self.mul_div_words(other, false) {
            crate::trace_dispatch!("rational", "mul", "word-sized");
            return result;
        }
        let numerator = &self.numerator * &other.numerator;
        let denominator = &self.denominator * &other.denominator;
        trace_rational_temporary!();
        Self::Output::maybe_reduce(Self::Output::from_parts_raw(sign, numerator, denominator))
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
                &self.numerator * &self.numerator,
                &self.denominator * &self.denominator,
            );
        }
        let numerator = &self.numerator * &other.denominator;
        let denominator = &self.denominator * &other.numerator;
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
