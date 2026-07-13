use core::ops::*;

impl Rational {
    fn add_sub_words(&self, other: &Self, subtract: bool) -> Option<Self> {
        let left_denominator = self.denominator.to_u128()?;
        let right_denominator = other.denominator.to_u128()?;
        let divisor = Self::gcd_word(left_denominator, right_denominator);
        let left_scale = right_denominator / divisor;
        let right_scale = left_denominator / divisor;
        let denominator = left_denominator.checked_mul(left_scale)?;
        let left = self.numerator.to_u128()?.checked_mul(left_scale)?;
        let right = other.numerator.to_u128()?.checked_mul(right_scale)?;
        let right_sign = if subtract { -other.sign } else { other.sign };

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
