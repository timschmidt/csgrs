use num_bigint::{BigInt, ToBigInt};

#[derive(Clone, Debug, PartialEq)]
pub struct BoundedRational {
    numerator: BigInt,
    denominator: BigInt,
}

impl BoundedRational {
    pub fn new(n: impl ToBigInt) -> Self {
        Self {
            numerator: ToBigInt::to_bigint(&n).unwrap(),
            denominator: ToBigInt::to_bigint(&1).unwrap(),
        }
    }

    pub fn fraction(n: impl ToBigInt, d: impl ToBigInt) -> Self {
        Self {
            numerator: ToBigInt::to_bigint(&n).unwrap(),
            denominator: ToBigInt::to_bigint(&d).unwrap(),
        }
    }

    fn maybe_reduce(self) -> Self {
        /* for now, always */
        self.reduce()
    }

    fn reduce(self) -> Self {
        if self.denominator == ToBigInt::to_bigint(&1).unwrap() {
            self
        } else {
            let divisor = num::Integer::gcd(&self.numerator, &self.denominator);
            let numerator = self.numerator / &divisor;
            let denominator = self.denominator / &divisor;
            Self { numerator, denominator }
        }
    }
}

use core::ops::*;

impl Add for BoundedRational {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let denominator = &self.denominator * &other.denominator;
        let numerator = (self.numerator * other.denominator) + (other.numerator * self.denominator);
        Self::maybe_reduce(Self {
            numerator,
            denominator,
        })
    }
}

impl Neg for BoundedRational {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            numerator: -self.numerator,
            denominator: self.denominator,
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
        let numerator = self.numerator * other.numerator;
        let denominator = self.denominator * other.denominator;
        Self::maybe_reduce(Self {
            numerator,
            denominator,
        })
    }
}

impl Div for BoundedRational {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        let numerator = self.numerator * other.denominator;
        let denominator = self.denominator * other.numerator;
        Self::maybe_reduce(Self {
            numerator,
            denominator,
        })
    }
}


#[cfg(test)]
#[test]
fn half_plus_one_times_two() {
    let half = BoundedRational::fraction(1, 2);
    let one = BoundedRational::new(1);
    let two = BoundedRational::new(2);
    let three = BoundedRational::new(3);
    let sum = half + one;
    assert_eq!(sum * two, three);
}

#[test]
fn three_divided_by_six() {
    let three = BoundedRational::new(3);
    let six = BoundedRational::new(6);
    let half = BoundedRational::fraction(1, 2);
    assert_eq!(three / six, half);
}

#[test]
fn one_plus_two() {
    let one = BoundedRational::new(1);
    let two = BoundedRational::new(2);
    let three = BoundedRational::new(3);
    assert_eq!(one + two, three);
}

#[test]
fn two_minus_one() {
    let two = BoundedRational::new(2);
    let one = BoundedRational::new(1);
    assert_eq!(two - one, BoundedRational::new(1));
}

#[test]
fn two_times_three() {
    let two = BoundedRational::new(2);
    let three = BoundedRational::new(3);
    assert_eq!(two * three, BoundedRational::new(6));
}
