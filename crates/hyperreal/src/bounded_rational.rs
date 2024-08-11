use crate::Computable;
use lazy_static::lazy_static;
use num_bigint::Sign::{self, *};
use num_bigint::{BigInt, BigUint, ToBigUint};
use num_traits::{One, Zero};

#[derive(Clone, Debug)]
pub struct ParseBRError();

#[derive(Clone, Debug)]
pub struct BoundedRational {
    sign: Sign,
    numerator: BigUint,
    denominator: BigUint,
}

impl BoundedRational {
    pub fn zero() -> Self {
        Self {
            sign: NoSign,
            numerator: BigUint::ZERO,
            denominator: BigUint::one(),
        }
    }

    pub fn one() -> Self {
        Self {
            sign: Plus,
            numerator: BigUint::one(),
            denominator: BigUint::one(),
        }
    }

    pub fn new(n: u64) -> Self {
        Self {
            sign: Plus,
            numerator: ToBigUint::to_biguint(&n).unwrap(),
            denominator: BigUint::one(),
        }
    }

    pub fn from_bigint(n: BigInt) -> Self {
        let sign = n.sign();
        let numerator = n.magnitude().clone();
        Self {
            sign,
            numerator,
            denominator: BigUint::one(),
        }
    }

    pub fn fraction(n: u64, d: u64) -> Self {
        Self {
            sign: Plus,
            numerator: ToBigUint::to_biguint(&n).unwrap(),
            denominator: ToBigUint::to_biguint(&d).unwrap(),
        }
    }

    pub fn from_bigint_fraction(n: BigInt, denominator: BigUint) -> Self {
        let sign = n.sign();
        let numerator = n.magnitude().clone();
        Self {
            sign,
            numerator,
            denominator,
        }
    }

    fn maybe_reduce(self) -> Self {
        /* for now, always */
        self.reduce()
    }

    fn reduce(self) -> Self {
        if self.denominator == One::one() {
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

    pub fn inverse(self) -> Self {
        Self {
            sign: self.sign,
            numerator: self.denominator,
            denominator: self.numerator,
        }
    }
}

use core::fmt;

impl fmt::Display for BoundedRational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.sign == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            f.write_str("+")?;
        }
        if self.denominator == One::one() {
            f.write_fmt(format_args!("{}", self.numerator))?;
        } else if f.alternate() {
            let whole = &self.numerator / &self.denominator;
            f.write_fmt(format_args!("{whole}."))?;
            let round = &whole * &self.denominator;
            let mut left = &self.numerator - &round;
            let mut digits = f.precision().unwrap_or(12);
            loop {
                left *= BigUint::parse_bytes("10".as_bytes(), 10).unwrap();
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

use num_bigint::ToBigInt;

impl BoundedRational {
    pub fn fmt_combine(&self, c: &Computable, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let d = &self.denominator.to_bigint().unwrap();
        let n = &self.numerator.to_bigint().unwrap();
        let precision = f.precision().unwrap_or(16);
        let bits = (n / d).bits() + ((precision * 10) as u64 / 3);
        let factor: i32 = bits
            .try_into()
            .expect("The number of bits we care about should fit in a 32-bit integer!");

        let two = BigUint::parse_bytes("2".as_bytes(), 10).unwrap();
        let divisor = two.pow(factor.try_into().unwrap());

        let r = Self::from_bigint_fraction(c.approx(-factor), divisor);
        let s = r * self.clone();

        f.write_fmt(format_args!("{s:#.*}...", precision))
    }
}

impl BoundedRational {
    pub fn is_whole(&self) -> bool {
        let whole = &self.numerator / &self.denominator;
        let round = &whole * &self.denominator;
        let left = &self.numerator - &round;
        left.is_zero()
    }

    pub fn prefer_decimal(&self) -> bool {
        let ten: BigUint = "10".parse().unwrap();
        let mut rem = self.denominator.clone();
        while (&rem % &ten).is_zero() {
            rem /= &ten;
        }
        let five: BigUint = "5".parse().unwrap();
        while (&rem % &five).is_zero() {
            rem /= &five;
        }
        let two: BigUint = "2".parse().unwrap();
        while (&rem % &two).is_zero() {
            rem /= &two;
        }
        rem == BigUint::one()
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
}

impl BoundedRational {
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
        lazy_static! {
            static ref SQUARES: Vec<(BigUint, BigUint)> = BoundedRational::make_squares();
        }

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
            return (Self::zero(), Self::new(0));
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

use std::str::FromStr;

impl FromStr for BoundedRational {
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
            let ten = BigUint::parse_bytes("10".as_bytes(), 10).unwrap();
            let denominator = ten.pow(d.len() as u32);
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
use std::cmp::Ordering;

impl Add for BoundedRational {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let denominator = &self.denominator * &other.denominator;
        let a = self.numerator * other.denominator;
        let b = other.numerator * self.denominator;
        let (sign, numerator) = match (self.sign, other.sign) {
            (any, NoSign) => (any, a),
            (Plus, Plus) => (Plus, a + b),
            (Minus, Minus) => (Minus, a + b),
            (x, y) => match a.cmp(&b) {
                Ordering::Greater => (x, a - b),
                Ordering::Equal => {
                    return Self::zero();
                }
                Ordering::Less => (y, b - a),
            },
        };
        Self::maybe_reduce(Self {
            sign,
            numerator,
            denominator,
        })
    }
}

impl Neg for BoundedRational {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            sign: -self.sign,
            ..self
        }
    }
}

impl Sub for BoundedRational {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        self + -other
    }
}

impl Mul for BoundedRational {
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

impl Div for BoundedRational {
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

impl BoundedRational {
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

impl PartialEq for BoundedRational {
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
        let many: BoundedRational = "12345".parse().unwrap();
        let s = format!("{many}");
        assert_eq!(s, "12345");
        let five: BoundedRational = "5".parse().unwrap();
        let third: BoundedRational = "1/3".parse().unwrap();
        let s = format!("{}", five * third);
        assert_eq!(s, "1 2/3");
    }

    #[test]
    fn decimals() {
        let first: BoundedRational = "0.0".parse().unwrap();
        assert_eq!(first, BoundedRational::zero());
        let a: BoundedRational = "0.4".parse().unwrap();
        let b: BoundedRational = "2.5".parse().unwrap();
        let answer = a * b;
        assert_eq!(answer, BoundedRational::one());
    }

    #[test]
    /// See e.g. https://discussions.apple.com/thread/252474975
    /// Apple calculator is not trustworthy if you are a programmer
    fn parse() {
        let big: BoundedRational = "288230376151711743".parse().unwrap();
        let small: BoundedRational = "45".parse().unwrap();
        let expected: BoundedRational = "12970366926827028435".parse().unwrap();
        assert_eq!(big * small, expected);
    }

    #[test]
    fn parse_fractions() {
        let third: BoundedRational = "1/3".parse().unwrap();
        let minus_four: BoundedRational = "-4".parse().unwrap();
        let twelve: BoundedRational = "12/20".parse().unwrap();
        let answer = third + minus_four * twelve;
        let expected: BoundedRational = "-31/15".parse().unwrap();
        assert_eq!(answer, expected);
    }

    #[test]
    fn square_reduced() {
        let thirty_two: BoundedRational = "32".parse().unwrap();
        let (square, rest) = thirty_two.extract_square_reduced();
        let four: BoundedRational = "4".parse().unwrap();
        assert_eq!(square, four);
        let two: BoundedRational = "2".parse().unwrap();
        assert_eq!(rest, two);
        let minus_one: BoundedRational = "-1".parse().unwrap();
        let (square, rest) = minus_one.clone().extract_square_reduced();
        assert_eq!(square, BoundedRational::one());
        assert_eq!(rest, minus_one);
    }

    #[test]
    fn signs() {
        let half: BoundedRational = "4/8".parse().unwrap();
        let one = BoundedRational::one();
        let minus_half = half - one;
        let two = BoundedRational::new(2);
        let zero = BoundedRational::zero();
        let minus_two = zero - two;
        let i2 = minus_two.inverse();
        assert_eq!(i2, minus_half);
    }

    #[test]
    fn half_plus_one_times_two() {
        let two = BoundedRational::new(2);
        let half = two.inverse();
        let one = BoundedRational::one();
        let two = BoundedRational::new(2);
        let three = BoundedRational::new(3);
        let sum = half + one;
        assert_eq!(sum * two, three);
    }

    #[test]
    fn three_divided_by_six() {
        let three = BoundedRational::new(3);
        let six = BoundedRational::new(6);
        let half: BoundedRational = "1/2".parse().unwrap();
        assert_eq!(three / six, half);
    }

    #[test]
    fn one_plus_two() {
        let one = BoundedRational::one();
        let two = BoundedRational::new(2);
        let three = BoundedRational::new(3);
        assert_eq!(one + two, three);
    }

    #[test]
    fn two_minus_one() {
        let two = BoundedRational::new(2);
        let one = BoundedRational::one();
        assert_eq!(two - one, BoundedRational::new(1));
    }

    #[test]
    fn two_times_three() {
        let two = BoundedRational::new(2);
        let three = BoundedRational::new(3);
        assert_eq!(two * three, BoundedRational::new(6));
    }

    #[test]
    fn decimal() {
        let half: BoundedRational = "4/8".parse().unwrap();
        assert!(half.prefer_decimal());
        let third: BoundedRational = "2/6".parse().unwrap();
        assert!(!third.prefer_decimal());
    }
}
