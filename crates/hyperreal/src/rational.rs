use crate::Computable;
use num::bigint::Sign::{self, *};
use num::{bigint::ToBigInt, bigint::ToBigUint, BigInt, BigUint};
use num::{One, Zero};
use std::sync::LazyLock;

#[derive(Clone, Debug)]
pub struct ParseBRError();

/// Ratio of two integers
///
/// This type is functionally a [`Sign`] with a ratio between two [`BigUint`]
/// (the numerator and denominator). The numerator and denominator are finite.
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
/// Simple arithmetic
/// ```
/// use realistic::Rational;
/// let quarter = Rational::fraction(1, 4);
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
        let sign = n.sign();
        let numerator = n.magnitude().clone();
        Self {
            sign,
            numerator,
            denominator: BigUint::one(),
        }
    }

    /// The non-negative Rational corresponding to the provided [`u64`]
    /// numerator and denominator as a fraction
    pub fn fraction(n: u64, d: u64) -> Self {
        Self {
            sign: Plus,
            numerator: ToBigUint::to_biguint(&n).unwrap(),
            denominator: ToBigUint::to_biguint(&d).unwrap(),
        }
    }

    /// The Rational corresponding to the provided [`BigInt`]
    /// numerator and [`BigUint`] denominator as a fraction
    pub fn from_bigint_fraction(n: BigInt, denominator: BigUint) -> Self {
        let sign = n.sign();
        let numerator = n.magnitude().clone();
        let answer =
        Self {
            sign,
            numerator,
            denominator,
        };
        answer.reduce()
    }

    fn maybe_reduce(self) -> Self {
        /* for now, always */
        self.reduce()
    }

    fn reduce(self) -> Self {
        if self.denominator == *ONE.deref() {
            self
        } else {
            let divisor = num::Integer::gcd(&self.numerator, &self.denominator);
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
    /// let a_fifth = Rational::fraction(1, 5);
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

    pub fn is_whole(&self) -> bool {
        let whole = &self.numerator / &self.denominator;
        let round = &whole * &self.denominator;
        let left = &self.numerator - &round;
        left.is_zero()
    }

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

    pub fn shifted_big_integer(&self, shift: i32) -> BigInt {
        let whole = (&self.numerator << shift) / &self.denominator;
        BigInt::from_biguint(self.sign, whole)
    }

    pub fn to_big_integer(&self) -> Option<BigInt> {
        let whole = &self.numerator / &self.denominator;
        let round = &whole * &self.denominator;
        let left = &self.numerator - &round;
        if left.is_zero() {
            Some(BigInt::from_biguint(self.sign, whole))
        } else {
            None
        }
    }

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
        ]
    }

    fn extract_square(n: BigUint) -> (BigUint, BigUint) {
        static SQUARES: LazyLock<Vec<(BigUint, BigUint)>> = LazyLock::new(Rational::make_squares);

        let mut square = One::one();
        let mut rest = n;
        if rest.bits() > Self::EXTRACT_SQUARE_MAX_LEN {
            return (square, rest);
        }
        for (p, s) in &*SQUARES {
            let one: BigUint = One::one();
            if rest == one {
                break;
            }
            while (&rest % s).is_zero() {
                rest /= s;
                square *= p;
            }
        }

        (square, rest)
    }

    pub fn extract_square_reduced(self) -> (Self, Self) {
        if self.sign == NoSign {
            return (Self::zero(), Self::zero());
        }
        let (nsquare, nrest) = Self::extract_square(self.numerator);
        let (dsquare, drest) = Self::extract_square(self.denominator);
        (
            Self {
                sign: Plus,
                numerator: nsquare,
                denominator: dsquare,
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
}

impl Rational {
    pub(crate) fn fmt_combine(&self, c: &Computable, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let d = &self.denominator.to_bigint().unwrap();
        let n = &self.numerator.to_bigint().unwrap();
        let precision = f.precision().unwrap_or(16);
        let bits = (n / d).bits() + ((precision * 10) as u64 / 3);
        let factor: i32 = bits
            .try_into()
            .expect("The number of bits we care about should fit in a 32-bit integer!");

        let divisor = TWO.pow(factor.try_into().unwrap());

        let r = Self::from_bigint_fraction(c.approx(-factor), divisor);
        let s = r * self.clone();

        f.write_fmt(format_args!("{s:#.*}...", precision))
    }
}

use core::fmt;

impl fmt::Display for Rational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.sign == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            f.write_str("+")?;
        }
        if self.denominator == *ONE.deref() {
            f.write_fmt(format_args!("{}", self.numerator))?;
        } else if f.alternate() {
            let whole = &self.numerator / &self.denominator;
            f.write_fmt(format_args!("{whole}."))?;
            let round = &whole * &self.denominator;
            let mut left = &self.numerator - &round;
            let mut digits = f.precision().unwrap_or(1000);
            loop {
                left *= &*TEN;
                let digit = &left / &self.denominator;
                f.write_fmt(format_args!("{digit}"))?;
                left -= digit * &self.denominator;
                if left.is_zero() {
                    break;
                }
                digits -= 1;
                if digits == 0 {
                    break;
                }
            }
        } else {
            let whole = &self.numerator / &self.denominator;
            let round = &whole * &self.denominator;
            let left = &self.numerator - &round;
            if whole.is_zero() {
                f.write_fmt(format_args!("{left}/{}", self.denominator))?;
            } else {
                f.write_fmt(format_args!("{whole} {left}/{}", self.denominator))?;
            }
        }
        Ok(())
    }
}

impl std::str::FromStr for Rational {
    type Err = ParseBRError;

    fn from_str(s: &str) -> Result<Self, ParseBRError> {
        let mut sign: Sign = Plus;
        let s = match s.strip_prefix('-') {
            Some(s) => {
                sign = Minus;
                s
            }
            None => s,
        };
        if let Some((n, d)) = s.split_once('/') {
            let numerator = BigUint::parse_bytes(n.as_bytes(), 10).ok_or(ParseBRError {})?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            Ok(Self {
                sign,
                numerator,
                denominator: BigUint::parse_bytes(d.as_bytes(), 10).ok_or(ParseBRError {})?,
            })
        } else if let Some((i, d)) = s.split_once('.') {
            let numerator = BigUint::parse_bytes(i.as_bytes(), 10).ok_or(ParseBRError {})?;
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
            let numerator = BigUint::parse_bytes(d.as_bytes(), 10).ok_or(ParseBRError {})?;
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
            let numerator = BigUint::parse_bytes(s.as_bytes(), 10).ok_or(ParseBRError {})?;
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
    fn decimal() {
        let decimal: Rational = "7.125".parse().unwrap();
        assert!(!decimal.prefer_fraction());
        let half: Rational = "4/8".parse().unwrap();
        assert!(!half.prefer_fraction());
        let third: Rational = "2/6".parse().unwrap();
        assert!(third.prefer_fraction());
    }
}
