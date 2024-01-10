use crate::BoundedRational;

#[derive(Copy, Clone, Debug)]
pub enum RealProblem {
    SqrtNegative,
    DivideZero,
}

#[derive(Clone, Debug)]
struct Guts {/* This is a stand-in */}

#[derive(Clone, Debug)]
enum Class {
    One,
    Pi,
    Sqrt(BoundedRational),
    Exp(BoundedRational),
    Ln(BoundedRational),
    Log10(BoundedRational),
    SinPi(BoundedRational),
    TanPi(BoundedRational),
    Asin(BoundedRational),
    Atan(BoundedRational),
    Irrational,
}

use Class::*;

// Irrational is never judged equal to anything here
impl PartialEq for Class {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (One, One) => true,
            (Pi, Pi) => true,
            (Sqrt(r), Sqrt(s)) => r == s,
            (Exp(r), Exp(s)) => r == s,
            (Ln(r), Ln(s)) => r == s,
            (Log10(r), Log10(s)) => r == s,
            (SinPi(r), SinPi(s)) => r == s,
            (TanPi(r), TanPi(s)) => r == s,
            (Asin(r), Asin(s)) => r == s,
            (Atan(r), Atan(s)) => r == s,
            (_, _) => false,
        }
    }
}

impl Class {
    // Could treat Class::Exp specially for large negative exponents
    fn is_non_zero(&self) -> bool {
        true
    }
}

#[derive(Clone, Debug)]
pub struct Real {
    rational: BoundedRational,
    class: Class,
    computable: Guts,
}

impl Real {
    pub fn zero() -> Self {
        Self {
            rational: BoundedRational::zero(),
            class: Class::One,
            computable: Guts {},
        }
    }

    pub fn new(rational: BoundedRational) -> Self {
        Self {
            rational,
            class: Class::One,
            computable: Guts {},
        }
    }

    pub fn pi() -> Self {
        Self {
            rational: BoundedRational::new(1),
            class: Class::Pi,
            computable: Guts {},
        }
    }
}

use num_bigint::Sign;

impl Real {
    pub fn definitely_zero(&self) -> bool {
        self.rational.sign() == Sign::NoSign
    }

    pub fn definitely_not_equal(&self, other: &Self) -> bool {
        if self.rational.sign() == Sign::NoSign {
            return other.class.is_non_zero() && other.rational.sign() != Sign::NoSign;
        }
        if other.rational.sign() == Sign::NoSign {
            return self.class.is_non_zero() && self.rational.sign() != Sign::NoSign;
        }
        false
        /* ... */
    }

    pub fn best_sign(&self) -> Sign {
        match &self.class {
            Class::One | Class::Pi => self.rational.sign(),
            other => {
                todo!("Sign of {other:?} unimplemented")
            }
        }
    }
}

impl Real {
    pub fn sqrt(self) -> Result<Self, RealProblem> {
        if self.best_sign() == Sign::Minus {
            return Err(RealProblem::SqrtNegative);
        }
        if self.definitely_zero() {
            return Ok(Self::zero());
        }
        match &self.class {
            Class::One => {
                if self.rational.extract_square_will_succeed() {
                    let (square, rest) = self.rational.extract_square_reduced();
                    if rest == BoundedRational::new(1) {
                        return Ok(Self {
                            rational: square,
                            class: Class::One,
                            computable: Guts {},
                        });
                    } else {
                        return Ok(Self {
                            rational: square,
                            class: Class::Sqrt(rest),
                            computable: Guts {},
                        });
                    }
                }
            }
            _ => (),
        }
        todo!("Square root of {self:?} unimplemented")
    }
}
use std::ops::*;

impl Add for Real {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        if self.class == other.class {
            let rational = self.rational + other.rational;
            return Self { rational, ..self };
        }
        if self.definitely_zero() {
            return other;
        }
        if other.definitely_zero() {
            return self;
        }
        todo!(
            "Adding {:?} to {:?} isn't implemented yet",
            self.class,
            other.class
        )
    }
}

impl Neg for Real {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            rational: -self.rational,
            ..self
        }
    }
}

impl Sub for Real {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        self + -other
    }
}

impl Real {
    fn multiply_sqrts(x: BoundedRational, y: BoundedRational) -> Self {
        if x == y {
            Self {
                rational: x,
                class: One,
                computable: Guts {},
            }
        } else {
            todo!("Multiply unlike square roots is unimplemented")
        }
    }
}

impl Mul for Real {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        if self.class == One {
            let rational = self.rational * other.rational;
            return Self { rational, ..other };
        }
        if other.class == One {
            let rational = self.rational * other.rational;
            return Self { rational, ..self };
        }
        if self.definitely_zero() || other.definitely_zero() {
            return Self::zero();
        }
        match (self.class, other.class) {
            (Class::Sqrt(r), Class::Sqrt(s)) => {
                let square = Self::multiply_sqrts(r, s);
                Self {
                    rational: square.rational * self.rational * other.rational,
                    ..square
                }
            }
            (Class::Exp(r), Class::Exp(s)) => {
                let exp = Class::Exp(r + s);
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: exp,
                    computable: Guts {},
                }
            }
            (sc, oc) => {
                todo!("Multiplying {:?} by {:?} isn't implemented yet", sc, oc);
            }
        }
    }
}

// Best efforts only, definitely not adequate for Eq
// Requirements: PartialEq should be transitive and symmetric
// however it needn't be complete or reflexive
impl PartialEq for Real {
    fn eq(&self, other: &Self) -> bool {
        self.rational == other.rational && self.class == other.class
    }
}

#[cfg(test)]
#[test]
fn zero() {
    assert_eq!(Real::zero(), Real::zero());
}

#[test]
fn rational() {
    let two = Real::new(BoundedRational::new(2));
    assert_ne!(two, Real::zero());
    let four = Real::new(BoundedRational::new(4));
    let answer = four - two;
    assert_eq!(answer, Real::new(BoundedRational::new(2)));
    let two = Real::new(BoundedRational::new(2));
    let zero = answer - two;
    assert_eq!(zero, Real::zero());
}

#[test]
fn zero_pi() {
    let pi = Real::pi();
    let z1 = pi - Real::pi();
    let pi2 = Real::pi() + Real::pi();
    let z2 = pi2 * Real::new(BoundedRational::zero());
    assert!(z1.definitely_zero());
    assert!(z2.definitely_zero());
    let two_pi = Real::pi() + Real::pi();
    assert_eq!(two_pi, Real::new(BoundedRational::new(2)) * Real::pi());
}

#[test]
fn sqrt_exact() {
    let big = Real::new(BoundedRational::new(40_000));
    let small = Real::new(BoundedRational::new(200));
    let answer = big.sqrt().expect("Square root of 100 should be 10");
    assert_eq!(small, answer);
}

#[test]
fn square_sqrt() {
    let big = Real::new(BoundedRational::new(3));
    let small = big.sqrt().expect("Should be able to sqrt(n)");
    let a = small.clone() * Real::new(BoundedRational::new(2));
    let b = small.clone() * Real::new(BoundedRational::new(3));
    let answer = a * b;
    assert_eq!(answer, Real::new(BoundedRational::new(18)));
}
