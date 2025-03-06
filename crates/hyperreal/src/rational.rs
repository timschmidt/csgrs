use crate::Problem;
use num::bigint::Sign::{self, *};
use num::{bigint::ToBigInt, bigint::ToBigUint, BigInt, BigUint};
use num::{One, Zero};
use std::sync::LazyLock;

pub(crate) mod convert;

/// Ratio of two integers
///
/// This type is functionally a [`Sign`] with a ratio between two [`BigUint`]
/// (the numerator and denominator). The numerator and denominator are finite.
///
/// The "ordinary" floating point numbers are rationals, but when converted
/// the exact rational may not be what you intuitively expected. It's obvious
/// that one third isn't represented exactly as an f64, but not everybody
/// will realise that 0.3 isn't either.
///
/// # Examples
///
/// Parsing a rational from a simple fraction
/// ```
/// use realistic::Rational;
/// let half: Rational = "9/18".parse().unwrap();
/// ```
///
/// Parsing a decimal fraction
/// ```
/// use realistic::Rational;
/// let point_two_five: Rational = "0.25".parse().unwrap();
/// ```
///
/// Converting a 64-bit floating point number
/// ```
/// use realistic::Rational;
/// let r: Rational = 0.3_f64.try_into().unwrap();
/// assert!(r != Rational::fraction(3, 10).unwrap());
/// ```
///
/// Simple arithmetic
/// ```
/// use realistic::Rational;
/// let quarter = Rational::fraction(1, 4).unwrap();
/// let eighteen = Rational::new(18);
/// let two = Rational::one() + Rational::one();
/// let sixteen = eighteen - two;
/// let four = quarter * sixteen;
/// assert_eq!(four, Rational::new(4));
/// ```

#[derive(Clone, Debug)]
pub struct Rational {
    sign: Sign,
    numerator: BigUint,
    denominator: BigUint,
}

static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
static TWO: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&2).unwrap());
static FIVE: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&5).unwrap());
static TEN: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&10).unwrap());

impl Rational {
    /// Zero, the additive identity
    pub fn zero() -> Self {
        Self {
            sign: NoSign,
            numerator: BigUint::ZERO,
            denominator: BigUint::one(),
        }
    }

    /// One, the multiplicative identity
    pub fn one() -> Self {
        Self {
            sign: Plus,
            numerator: BigUint::one(),
            denominator: BigUint::one(),
        }
    }

    /// The non-negative Rational corresponding to the provided [`i64`]
    pub fn new(n: i64) -> Self {
        Self::from_bigint(ToBigInt::to_bigint(&n).unwrap())
    }

    /// The Rational corresponding to the provided [`BigInt`]
    pub fn from_bigint(n: BigInt) -> Self {
        Self::from_bigint_fraction(n, BigUint::one()).unwrap()
    }

    /// The non-negative Rational corresponding to the provided [`i64`]
    /// numerator and [`u64`] denominator as a fraction
    pub fn fraction(n: i64, d: u64) -> Result<Self, Problem> {
        let numerator = ToBigInt::to_bigint(&n).unwrap();
        let denominator = ToBigUint::to_biguint(&d).unwrap();
        Self::from_bigint_fraction(numerator, denominator)
    }

    /// The Rational corresponding to the provided [`BigInt`]
    /// numerator and [`BigUint`] denominator as a fraction
    pub fn from_bigint_fraction(n: BigInt, denominator: BigUint) -> Result<Self, Problem> {
        if denominator == BigUint::ZERO {
            return Err(Problem::DivideByZero);
        }
        let sign = n.sign();
        let numerator = n.magnitude().clone();
        let answer = Self {
            sign,
            numerator,
            denominator,
        };
        Ok(answer.reduce())
    }

    fn maybe_reduce(self) -> Self {
        /* for now, always */
        self.reduce()
    }

    fn reduce(self) -> Self {
        if self.denominator == *ONE.deref() {
            return self;
        }

        let divisor = num::Integer::gcd(&self.numerator, &self.denominator);
        if divisor == *ONE.deref() {
            self
        } else {
            let numerator = self.numerator / &divisor;
            let denominator = self.denominator / &divisor;
            Self {
                sign: self.sign,
                numerator,
                denominator,
            }
        }
    }

    /// The inverse of this Rational
    ///
    /// # Example
    ///
    /// ```
    /// use realistic::Rational;
    /// let five = Rational::new(5);
    /// let a_fifth = Rational::fraction(1, 5).unwrap();
    /// assert_eq!(five.clone().inverse(), a_fifth);
    /// assert_eq!(a_fifth.clone().inverse(), five);
    /// ```
    pub fn inverse(self) -> Self {
        Self {
            sign: self.sign,
            numerator: self.denominator,
            denominator: self.numerator,
        }
    }

    /// Checks if the value is an integer
    ///
    /// # Example
    ///
    /// ```
    /// use realistic::Rational;
    /// assert!(Rational::new(5).is_integer());
    /// assert!(Rational::fraction(16, 4).unwrap().is_integer());
    /// assert!(!Rational::fraction(5, 4).unwrap().is_integer());
    /// ```
    pub fn is_integer(&self) -> bool {
        self.denominator == *ONE.deref()
    }

    /// The integer part of this Rational
    ///
    /// Non integer rationals will thus be truncated towards zero
    ///
    /// # Examples
    ///
    /// ```
    /// use realistic::Rational;
    /// let approx_pi = Rational::fraction(22, 7).unwrap();
    /// let three = Rational::new(3);
    /// assert_eq!(approx_pi.trunc(), three);
    /// ```
    ///
    /// The integer result can be converted to a primitive integer type
    /// with suitable range
    ///
    /// ```
    /// use realistic::Rational;
    /// let fraction = Rational::new(172) / Rational::new(9);
    /// let int: u8 = fraction.trunc().try_into().unwrap();
    /// assert_eq!(int, 19);
    /// ```
    pub fn trunc(&self) -> Self {
        if self.is_integer() {
            return self.clone();
        }
        let n = &self.numerator / &self.denominator;
        Self {
            sign: self.sign,
            numerator: n,
            denominator: ONE.deref().clone(),
        }
    }

    /// The fractional part of this Rational
    ///
    /// If the rational was negative, this fraction will also be negative
    ///
    /// # Examples
    ///
    /// ```
    /// use realistic::Rational;
    /// let approx_pi = Rational::fraction(22, 7).unwrap();
    /// let a_seventh = Rational::fraction(1, 7).unwrap();
    /// assert_eq!(approx_pi.fract(), a_seventh);
    /// ```
    ///
    /// ```
    /// use realistic::Rational;
    /// let backward = Rational::fraction(-53, 9).unwrap();
    /// let fract = Rational::fraction(-8, 9).unwrap();
    /// assert_eq!(backward.fract(), fract);
    /// ```
    pub fn fract(&self) -> Self {
        if self.is_integer() {
            return Self::zero();
        }
        let n = &self.numerator % &self.denominator;
        Self {
            sign: self.sign,
            numerator: n,
            denominator: self.denominator.clone(),
        }
    }

    pub(crate) fn denominator(&self) -> &BigUint {
        &self.denominator
    }

    /// Is this Rational better understood as a fraction?
    ///
    /// If a decimal expansion of this fraction would never end this is true
    ///
    /// # Example
    ///
    /// ```
    /// use realistic::Rational;
    /// let third = Rational::fraction(1, 3).unwrap();
    /// assert!(third.prefer_fraction());
    /// ```
    pub fn prefer_fraction(&self) -> bool {
        let mut rem = self.denominator.clone();
        while (&rem % &*TEN).is_zero() {
            rem /= &*TEN;
        }
        while (&rem % &*FIVE).is_zero() {
            rem /= &*FIVE;
        }
        while (&rem % &*TWO).is_zero() {
            rem /= &*TWO;
        }
        rem != BigUint::one()
    }

    /// Left shift the value by any amount and return a [`BigInt`]
    /// of the truncated integer value
    ///
    /// # Example
    ///
    /// ```
    /// use realistic::Rational;
    /// use num::bigint::ToBigInt;
    /// let seven_fifths = Rational::fraction(7, 5).unwrap();
    /// let eleven = ToBigInt::to_bigint(&11).unwrap();
    /// assert_eq!(seven_fifths.shifted_big_integer(3), eleven);
    /// ```
    pub fn shifted_big_integer(&self, shift: i32) -> BigInt {
        let whole = (&self.numerator << shift) / &self.denominator;
        BigInt::from_biguint(self.sign, whole)
    }

    /// Either the corresponding [`BigInt`] or None if this value is not an integer
    pub fn to_big_integer(&self) -> Option<BigInt> {
        let whole = &self.numerator / &self.denominator;
        let round = &whole * &self.denominator;
        if self.numerator == round {
            debug_assert!(self.denominator == *ONE.deref());
            Some(BigInt::from_biguint(self.sign, whole))
        } else {
            debug_assert!(self.denominator != *ONE.deref());
            None
        }
    }

    /// The [`Sign`] of this value
    pub fn sign(&self) -> Sign {
        self.sign
    }

    const EXTRACT_SQUARE_MAX_LEN: u64 = 5000;

    fn make_squares() -> Vec<(BigUint, BigUint)> {
        vec![
            (
                ToBigUint::to_biguint(&2).unwrap(),
                ToBigUint::to_biguint(&4).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&3).unwrap(),
                ToBigUint::to_biguint(&9).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&5).unwrap(),
                ToBigUint::to_biguint(&25).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&7).unwrap(),
                ToBigUint::to_biguint(&49).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&11).unwrap(),
                ToBigUint::to_biguint(&121).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&13).unwrap(),
                ToBigUint::to_biguint(&169).unwrap(),
            ),
            (
                ToBigUint::to_biguint(&17).unwrap(),
                ToBigUint::to_biguint(&289).unwrap(),
            ),
        ]
    }

    // Some(root) squared is n, otherwise None
    fn try_perfect(n: BigUint) -> Option<BigUint> {
        use crate::Computable;
        use std::cmp::Ordering::*;

        let r = Self {
            sign: Plus,
            numerator: n.clone(),
            denominator: BigUint::one(),
        };
        let sqrt = Computable::sqrt_rational(r);
        let root = ToBigUint::to_biguint(&sqrt.approx(0)).expect("should be an unsigned integer");
        let square = &root * &root;
        match n.cmp(&square) {
            Equal => Some(root),
            _ => None,
        }
    }

    // (root squared times rest) = n
    fn extract_square(n: BigUint) -> (BigUint, BigUint) {
        static SQUARES: LazyLock<Vec<(BigUint, BigUint)>> = LazyLock::new(Rational::make_squares);

        let one: BigUint = One::one();
        let mut root = one.clone();
        let mut rest = n;
        if rest.bits() > Self::EXTRACT_SQUARE_MAX_LEN {
            return (root, rest);
        }
        for (p, s) in &*SQUARES {
            if rest == one {
                break;
            }
            while (&rest % s).is_zero() {
                rest /= s;
                root *= p;
            }
        }

        let divisors = if rest.bit(0) {
            // Odd number so dividing by an even number won't get a whole result
            [1, 3, 5, 7, 11, 13, 15, 17, 19]
        } else {
            [1, 2, 3, 5, 6, 7, 8, 10, 11]
        };

        for n in divisors {
            let divisor = ToBigUint::to_biguint(&n).unwrap();
            if rest == divisor {
                return (root, rest);
            }
            if (&rest % &divisor).is_zero() {
                let square = &rest / &divisor;
                if let Some(factor) = Self::try_perfect(square) {
                    return (root * factor, divisor);
                }
            }
        }
        (root, rest)
    }

    pub fn extract_square_reduced(self) -> (Self, Self) {
        if self.sign == NoSign {
            return (Self::zero(), Self::zero());
        }
        let (nroot, nrest) = Self::extract_square(self.numerator);
        let (droot, drest) = Self::extract_square(self.denominator);
        (
            Self {
                sign: Plus,
                numerator: nroot,
                denominator: droot,
            },
            Self {
                sign: self.sign,
                numerator: nrest,
                denominator: drest,
            },
        )
    }

    pub fn extract_square_will_succeed(&self) -> bool {
        self.numerator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
            && self.denominator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
    }

    // This could grow unreasonably in terms of object size
    // so only call this for modest exp values
    fn pow_up(&self, exp: &BigUint) -> Self {
        if exp == &BigUint::ZERO {
            return Self::one();
        }
        let mut result = Self::one();
        for b in (0..(exp.bits())).rev() {
            result *= result.clone();
            if exp.bit(b) {
                result *= self;
            }
        }
        result
    }

    /// Integer exponeniation
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        const TOO_MANY_BITS: u64 = 1000;
        // Arguably wrong if self is also zero
        if exp == BigInt::ZERO {
            return Ok(Self::one());
        }
        if self.sign == NoSign {
            return Ok(Self::zero());
        }
        // Plus or minus one exactly
        if self.is_integer() && self.numerator == *ONE.deref() {
            if self.sign == Minus && exp.bit(0) {
                return Ok(Self::new(-1));
            } else {
                return Ok(Self::one());
            }
        }
        if exp.bits() >= TOO_MANY_BITS {
            return Err(Problem::Exhausted);
        }
        match exp.sign() {
            Minus => Ok(self.inverse().pow_up(exp.magnitude())),
            Plus => Ok(self.pow_up(exp.magnitude())),
            NoSign => unreachable!(),
        }
    }
}

use core::fmt;

impl fmt::Display for Rational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.denominator == *ONE.deref() {
            let int = self.numerator.to_string();
            return f.pad_integral(self.sign != Minus, "", &int);
        }

        if self.sign == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            f.write_str("+")?;
        }
        if f.alternate() {
            let whole = &self.numerator / &self.denominator;
            write!(f, "{whole}.")?;
            let round = &whole * &self.denominator;
            let mut left = &self.numerator - &round;
            let mut digits = f.precision().unwrap_or(1000);
            loop {
                left *= &*TEN;
                let digit = &left / &self.denominator;
                write!(f, "{digit}")?;
                left -= digit * &self.denominator;
                if left.is_zero() {
                    break;
                }
                digits -= 1;
                if digits == 0 {
                    break;
                }
            }
            Ok(())
        } else {
            let whole = &self.numerator / &self.denominator;
            let round = &whole * &self.denominator;
            let left = &self.numerator - &round;
            if whole.is_zero() {
                write!(f, "{left}/{}", self.denominator)
            } else {
                write!(f, "{whole} {left}/{}", self.denominator)
            }
        }
    }
}

impl std::str::FromStr for Rational {
    type Err = Problem;

    fn from_str(s: &str) -> Result<Self, Problem> {
        let mut sign: Sign = Plus;
        let s = match s.strip_prefix('-') {
            Some(s) => {
                sign = Minus;
                s
            }
            None => s,
        };
        if let Some((n, d)) = s.split_once('/') {
            let numerator = BigUint::parse_bytes(n.as_bytes(), 10).ok_or(Problem::BadFraction)?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            Ok(Self {
                sign,
                numerator,
                denominator: BigUint::parse_bytes(d.as_bytes(), 10).ok_or(Problem::BadFraction)?,
            })
        } else if let Some((i, d)) = s.split_once('.') {
            let numerator = BigUint::parse_bytes(i.as_bytes(), 10).ok_or(Problem::BadDecimal)?;
            let whole = if numerator.is_zero() {
                Self {
                    sign: NoSign,
                    numerator,
                    denominator: One::one(),
                }
            } else {
                Self {
                    sign,
                    numerator,
                    denominator: One::one(),
                }
            };
            let numerator = BigUint::parse_bytes(d.as_bytes(), 10).ok_or(Problem::BadDecimal)?;
            if numerator.is_zero() {
                return Ok(whole);
            }
            let denominator = TEN.pow(d.len() as u32);
            let fraction = Self {
                sign,
                numerator,
                denominator,
            };
            Ok(whole + fraction)
        } else {
            let numerator = BigUint::parse_bytes(s.as_bytes(), 10).ok_or(Problem::BadInteger)?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            Ok(Self {
                sign,
                numerator,
                denominator: One::one(),
            })
        }
    }
}

use core::ops::*;

impl Add for Rational {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        use std::cmp::Ordering::*;

        let denominator = &self.denominator * &other.denominator;
        let a = self.numerator * other.denominator;
        let b = other.numerator * self.denominator;
        let (sign, numerator) = match (self.sign, other.sign) {
            (any, NoSign) => (any, a),
            (NoSign, any) => (any, b),
            (Plus, Plus) => (Plus, a + b),
            (Minus, Minus) => (Minus, a + b),
            (x, y) => match a.cmp(&b) {
                Greater => (x, a - b),
                Equal => {
                    return Self::zero();
                }
                Less => (y, b - a),
            },
        };
        Self::maybe_reduce(Self {
            sign,
            numerator,
            denominator,
        })
    }
}

impl Neg for Rational {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            sign: -self.sign,
            ..self
        }
    }
}

impl Sub for Rational {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        self + -other
    }
}

impl Mul for Rational {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let sign = self.sign * other.sign;
        let numerator = self.numerator * other.numerator;
        let denominator = self.denominator * other.denominator;
        Self::maybe_reduce(Self {
            sign,
            numerator,
            denominator,
        })
    }
}

impl MulAssign for Rational {
    fn mul_assign(&mut self, other: Self) {
        let sign = self.sign * other.sign;
        self.sign = sign;
        self.numerator *= other.numerator;
        self.denominator *= other.denominator;
    }
}

impl MulAssign<&Rational> for Rational {
    fn mul_assign(&mut self, other: &Rational) {
        let sign = self.sign * other.sign;
        self.sign = sign;
        self.numerator = &self.numerator * &other.numerator;
        self.denominator = &self.denominator * &other.denominator;
    }
}

impl Div for Rational {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        let sign = self.sign * other.sign;
        let numerator = self.numerator * other.denominator;
        let denominator = self.denominator * other.numerator;
        Self::maybe_reduce(Self {
            sign,
            numerator,
            denominator,
        })
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

impl PartialEq for Rational {
    fn eq(&self, other: &Self) -> bool {
        if self.sign != other.sign {
            return false;
        }
        if self.denominator == other.denominator {
            self.numerator == other.numerator
        } else {
            Self::definitely_equal(&self.clone().reduce(), &other.clone().reduce())
        }
    }
}

impl PartialOrd for Rational {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering::*;
        match self.sign.cmp(&other.sign) {
            Less => return Some(Less),
            Greater => return Some(Greater),
            _ => (),
        }
        if self.denominator == other.denominator {
            match self.sign {
                Plus => self.numerator.partial_cmp(&other.numerator),
                Minus => other.numerator.partial_cmp(&self.numerator),
                NoSign => unreachable!(),
            }
        } else {
            let left = &self.numerator * &other.denominator;
            let right = &other.numerator * &self.denominator;
            match self.sign {
                Plus => left.partial_cmp(&right),
                Minus => right.partial_cmp(&left),
                NoSign => unreachable!(),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    /// See e.g. https://discussions.apple.com/thread/252474975
    /// Apple calculator is not trustworthy if you are a programmer
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
    fn signs() {
        let half: Rational = "4/8".parse().unwrap();
        let one = Rational::one();
        let minus_half = half - one;
        let two = Rational::new(2);
        let zero = Rational::zero();
        let minus_two = zero - two;
        let i2 = minus_two.inverse();
        assert_eq!(i2, minus_half);
    }

    #[test]
    fn half_plus_one_times_two() {
        let two = Rational::new(2);
        let half = two.inverse();
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        let sum = half + one;
        assert_eq!(sum * two, three);
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
        let one_two_five = Rational::new(5).powi(ToBigInt::to_bigint(&-3).unwrap());
        assert_eq!(one_two_five, Rational::fraction(1, 125));
        let more = Rational::new(7).powi(11i32.into()).unwrap();
        assert_eq!(more, Rational::new(1_977_326_743));
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
    fn decimal() {
        let decimal: Rational = "7.125".parse().unwrap();
        assert!(!decimal.prefer_fraction());
        let half: Rational = "4/8".parse().unwrap();
        assert!(!half.prefer_fraction());
        let third: Rational = "2/6".parse().unwrap();
        assert!(third.prefer_fraction());
    }

    #[test]
    fn compare() {
        assert!(Rational::one() > Rational::zero());
        assert!(Rational::new(5) > Rational::new(4));
        assert!(Rational::new(-10) < Rational::new(5));
        assert!(Rational::fraction(1, 4).unwrap() < Rational::fraction(1, 3).unwrap());
    }

    #[test]
    fn divide_by_zero() {
        let err = Rational::fraction(1, 0).unwrap_err();
        assert_eq!(err, Problem::DivideByZero);
    }
}
