use core::ops::*;

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
        Self::Output {
            sign,
            numerator,
            denominator,
        }
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
        trace_rational_temporary!();
        let mut ret = self.clone();
        ret.sign = -ret.sign;
        ret
    }
}

impl Neg for Rational {
    type Output = Self;

    fn neg(mut self) -> Self {
        self.sign = -self.sign;
        self
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
        Self::Output {
            sign,
            numerator,
            denominator,
        }
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
        let numerator = &self.numerator * &other.numerator;
        let denominator = &self.denominator * &other.denominator;
        trace_rational_temporary!();
        Self::Output::maybe_reduce(Self::Output {
            sign,
            numerator,
            denominator,
        })
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
        if self.numerator == other.denominator && self.denominator == other.numerator {
            trace_rational_temporary!();
            return Self::Output {
                sign,
                numerator: &self.numerator * &self.numerator,
                denominator: &self.denominator * &self.denominator,
            };
        }
        let numerator = &self.numerator * &other.denominator;
        let denominator = &self.denominator * &other.numerator;
        trace_rational_temporary!();
        Self::Output::maybe_reduce(Self::Output {
            sign,
            numerator,
            denominator,
        })
    }
}

impl<T: AsRef<Rational>> Div<T> for Rational {
    type Output = Self;

    fn div(self, other: T) -> Self {
        &self / other.as_ref()
    }
}

impl Rational {
    fn definitely_equal(&self, other: &Self) -> bool {
        if self.sign != other.sign {
            return false;
        }
        if self.denominator != other.denominator {
            return false;
        }
        self.numerator == other.numerator
    }
}

