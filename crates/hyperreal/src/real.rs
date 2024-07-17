use crate::BoundedRational;
use crate::Computable;

#[derive(Copy, Clone, Debug)]
pub enum RealProblem {
    ParseError,
    SqrtNegative,
    DivideByZero,
    NotFound,
}

#[derive(Clone, Debug)]
enum Class {
    None,
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

// Neither None nor Irrational are judged equal to anything here
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

    fn make_exp(br: BoundedRational) -> Class {
        if br == BoundedRational::zero() {
            Class::One
        } else {
            Class::Exp(br)
        }
    }
}

#[derive(Clone, Debug)]
pub struct Real {
    rational: BoundedRational,
    class: Class,
    computable: Computable,
}

impl Real {
    pub fn zero() -> Self {
        Self {
            rational: BoundedRational::zero(),
            class: Class::One,
            computable: Computable::one(),
        }
    }

    pub fn new(rational: BoundedRational) -> Self {
        Self {
            rational,
            class: Class::One,
            computable: Computable::one(),
        }
    }

    pub fn pi() -> Self {
        Self {
            rational: BoundedRational::one(),
            class: Class::Pi,
            computable: Computable::pi(),
        }
    }

    pub fn e() -> Self {
        let one = BoundedRational::one();
        Self {
            rational: one.clone(),
            class: Class::Exp(one),
            computable: Computable::placeholder(),
        }
    }

    pub fn ln10() -> Self {
        Self {
            rational: BoundedRational::one(),
            class: Class::Ln(BoundedRational::new(10)),
            computable: Computable::placeholder(),
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
    pub fn inverse(self) -> Result<Self, RealProblem> {
        if self.definitely_zero() {
            return Err(RealProblem::DivideByZero);
        }
        match &self.class {
            Class::One => {
                return Ok(Self {
                    rational: self.rational.inverse(),
                    class: Class::One,
                    computable: Computable::one(),
                });
            }
            Class::Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.to_big_integer() {
                    let rational = (self.rational * BoundedRational::from_bigint(sqrt)).inverse();
                    return Ok(Self {
                        rational,
                        class: self.class,
                        computable: self.computable,
                    });
                }
            }
            Class::Exp(exp) => {
                let exp = Neg::neg(exp.clone());
                return Ok(Self {
                    rational: self.rational.inverse(),
                    class: Class::Exp(exp),
                    computable: Computable::placeholder(),
                });
            }
            _ => (),
        }
        todo!("Inverse of {self:?} unimplemented")
    }

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
                    if rest == BoundedRational::one() {
                        return Ok(Self {
                            rational: square,
                            class: Class::One,
                            computable: Computable::one(),
                        });
                    } else {
                        return Ok(Self {
                            rational: square,
                            class: Class::Sqrt(rest),
                            computable: Computable::placeholder(),
                        });
                    }
                }
            }
            Class::Exp(_exp) => {
                // Implementation originally submitted here doesn't handle the rational component?
                todo!()
            }
            _ => (),
        }
        todo!("Square root of {self:?} unimplemented")
    }

    pub fn exp(self) -> Result<Self, RealProblem> {
        if self.definitely_zero() {
            return Ok(Self::new(BoundedRational::one()));
        }
        todo!("Exp({self:?}) unimplemented")
    }
}

use core::fmt;

impl Real {
    // Should we display this as a decimal ?
    pub fn prefer_decimal(&self) -> bool {
        // For now, prefer to display a decimal when we're not simple enough for Class::One
        // In future also prefer when the rational could be expressed nicely this way
        self.class != Class::One
    }

    // Format this Real as a decimal rather than rational
    pub fn decimal(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("{:#}", self.rational))?;
        if self.class != Class::One {
            f.write_str(" x ...?")?;
        }
        Ok(())
    }
}

impl fmt::Display for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            self.decimal(f)?;
            return Ok(());
        } else {
            f.write_fmt(format_args!("{}", self.rational))?;
        }

        if self.class != Class::One {
            f.write_fmt(format_args!(" x {:?}::{:?}", self.class, self.computable))?;
        }
        Ok(())
    }
}

use std::str::FromStr;

impl FromStr for Real {
    type Err = RealProblem;

    fn from_str(s: &str) -> Result<Self, RealProblem> {
        let rational: BoundedRational = s.parse().map_err(|_| RealProblem::ParseError)?;
        Ok(Self {
            rational,
            class: Class::One,
            computable: Computable::one(),
        })
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
                computable: Computable::one(),
            }
        } else {
            let product = x * y;
            if product == BoundedRational::zero() {
                return Self {
                    rational: product,
                    class: One,
                    computable: Computable::one(),
                };
            }
            let (a, b) = product.extract_square_reduced();
            Self {
                rational: a,
                class: Sqrt(b),
                computable: Computable::placeholder(),
            }
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
                let class = Class::make_exp(r + s);
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class,
                    computable: Computable::placeholder(),
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
mod tests {
    use super::*;

    #[test]
    fn zero() {
        assert_eq!(Real::zero(), Real::zero());
    }

    #[test]
    fn root_divide() {
        let twenty: Real = "20".parse().unwrap();
        let five: Real = "5".parse().unwrap();
        let a = twenty.sqrt().unwrap();
        let b = five.sqrt().unwrap().inverse().unwrap();
        let answer = a * b;
        let two: Real = "2".parse().unwrap();
        assert_eq!(answer, two);
    }

    #[test]
    fn rational() {
        let two: Real = "2".parse().unwrap();
        assert_ne!(two, Real::zero());
        let four: Real = "4".parse().unwrap();
        let answer = four - two.clone();
        assert_eq!(answer, two.clone());
        let zero = answer - two;
        assert_eq!(zero, Real::zero());
        let six_half: Real = "13/2".parse().unwrap();
        let opposite = six_half.inverse().unwrap();
        let expected = "2/13".parse().unwrap();
        assert_eq!(opposite, expected);
    }

    // https://devblogs.microsoft.com/oldnewthing/?p=93765
    // "Why does the Windows calculator generate tiny errors when calculating the square root of a
    // perfect square?" (fixed in 2018)
    #[test]
    fn perfect_square() {
        let four: Real = "4".parse().unwrap();
        let two: Real = "2".parse().unwrap();
        let calc = four.sqrt().unwrap() - two;
        assert_eq!(calc, Real::zero());
    }

    #[test]
    fn one_over_e() {
        let one: Real = "1".parse().unwrap();
        let e = Real::e();
        let e_inverse = Real::e().inverse().unwrap();
        let answer = e * e_inverse;
        assert_eq!(one, answer);
        let again = answer.sqrt().unwrap();
        assert_eq!(one, again);
    }

    #[test]
    fn unlike_sqrts() {
        let thirty: Real = "30".parse().unwrap();
        let ten: Real = "10".parse().unwrap();
        let answer = thirty.sqrt().unwrap() * ten.sqrt().unwrap();
        let ten: Real = "10".parse().unwrap();
        let three: Real = "3".parse().unwrap();
        let or = ten * three.sqrt().unwrap();
        assert_eq!(answer, or);
    }

    #[test]
    fn zero_pi() {
        let pi = Real::pi();
        let z1 = pi - Real::pi();
        let pi2 = Real::pi() + Real::pi();
        let z2 = pi2 * Real::zero();
        assert!(z1.definitely_zero());
        assert!(z2.definitely_zero());
        let two_pi = Real::pi() + Real::pi();
        let two: Real = "2".parse().unwrap();
        assert_eq!(two_pi, two * Real::pi());
    }

    #[test]
    fn sqrt_exact() {
        let big: Real = "40000".parse().unwrap();
        let small: Real = "200".parse().unwrap();
        let answer = big.sqrt().unwrap();
        assert_eq!(small, answer);
    }

    #[test]
    fn square_sqrt() {
        let two: Real = "2".parse().unwrap();
        let three: Real = "3".parse().unwrap();
        let small = three.clone().sqrt().expect("Should be able to sqrt(n)");
        let a = small.clone() * two;
        let b = small.clone() * three;
        let answer = a * b;
        let eighteen: Real = "18".parse().unwrap();
        assert_eq!(answer, eighteen);
    }
}
