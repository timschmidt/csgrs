use crate::Computable;
use crate::Rational;

/// Problems when either parsing or attempting Arithmetic with [`Real`] numbers

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum RealProblem {
    ParseError,
    SqrtNegative,
    DivideByZero,
    NotFound,
    InsufficientParameters,
}

#[derive(Clone, Debug)]
enum Class {
    One,
    Pi,
    Sqrt(Rational),
    Exp(Rational),
    Ln(Rational),
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
            (_, _) => false,
        }
    }
}

impl Class {
    // Could treat Class::Exp specially for large negative exponents
    fn is_non_zero(&self) -> bool {
        true
    }

    fn make_exp(br: Rational) -> (Class, Computable) {
        if br == Rational::zero() {
            (Class::One, Computable::one())
        } else {
            (Class::Exp(br.clone()), Computable::e(br))
        }
    }
}

/// (More) Real numbers
///
/// This type is functionally the product of a [`Computable`] number
/// and a [`Rational`]
///
/// # Examples
///
/// Even a normal rational can be parsed as a Real
/// ```
/// use realistic::Real;
/// let half: Real = "0.5".parse().unwrap();
/// ```
///
/// Simple arithmetic
/// ```
/// use realistic::Real;
/// let two_pi = Real::pi() + Real::pi();
/// let four: Real = "4".parse().unwrap();
/// let four_pi = four * Real::pi();
/// let answer = (four_pi / two_pi).unwrap();
/// let two = realistic::Rational::new(2);
/// assert_eq!(answer, Real::new(two));
/// ```

#[derive(Debug)]
pub struct Real {
    rational: Rational,
    class: Class,
    computable: Computable,
}

impl Real {
    /// Zero, the additive identity
    pub fn zero() -> Real {
        Self {
            rational: Rational::zero(),
            class: Class::One,
            computable: Computable::one(),
        }
    }

    /// The specified [`Rational`] as a Real
    pub fn new(rational: Rational) -> Real {
        Self {
            rational,
            class: Class::One,
            computable: Computable::one(),
        }
    }

    /// π, the ratio of a circle's circumference to its diameter
    pub fn pi() -> Real {
        Self {
            rational: Rational::one(),
            class: Class::Pi,
            computable: Computable::pi(),
        }
    }

    /// e, Euler's number and the base of the natural logarithm function
    pub fn e() -> Real {
        let one = Rational::one();
        Self {
            rational: one.clone(),
            class: Class::Exp(one.clone()),
            computable: Computable::e(one),
        }
    }
}

use num::bigint::Sign;

impl Real {
    /// Is this Real exactly zero?
    pub fn definitely_zero(&self) -> bool {
        self.rational.sign() == Sign::NoSign
    }

    /// Are two Reals definitely unequal?
    pub fn definitely_not_equal(&self, other: &Self) -> bool {
        if self.rational.sign() == Sign::NoSign {
            return other.class.is_non_zero() && other.rational.sign() != Sign::NoSign;
        }
        if other.rational.sign() == Sign::NoSign {
            return self.class.is_non_zero() && self.rational.sign() != Sign::NoSign;
        }
        false
        /* ... TODO add more cases which definitely aren't equal */
    }

    /// Our best attempt to discern the [`Sign`] of this Real
    /// this will be accurate for trivial Rationals and some but not all other cases
    pub fn best_sign(&self) -> Sign {
        match &self.class {
            Class::One | Class::Pi | Class::Exp(_) | Class::Sqrt(_) => self.rational.sign(),
            _ => match (self.rational.sign(), self.computable.sign()) {
                (Sign::NoSign, _) => Sign::NoSign,
                (_, Sign::NoSign) => Sign::NoSign,
                (Sign::Plus, Sign::Plus) => Sign::Plus,
                (Sign::Plus, Sign::Minus) => Sign::Minus,
                (Sign::Minus, Sign::Plus) => Sign::Minus,
                (Sign::Minus, Sign::Minus) => Sign::Plus,
            },
        }
    }

    fn make_computable<F>(self, convert: F) -> Self
    where
        F: FnOnce(Computable) -> Computable,
    {
        let r = Computable::rational(self.rational);
        let prev = Computable::multiply(r, self.computable);
        let computable = convert(prev);

        Self {
            rational: Rational::one(),
            class: Class::Irrational,
            computable,
        }
    }

    /// The inverse of this Real, or a [`RealProblem`] if that's impossible,
    /// in particular RealProblem::DivideByZero if this real is zero
    ///
    /// Example
    /// ```
    /// use realistic::{Rational,Real};
    /// let five = Real::new(Rational::new(5));
    /// let a_fifth = Real::new(Rational::fraction(1, 5));
    /// assert_eq!(five.inverse(), Ok(a_fifth));
    /// ```
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
                    let rational = (self.rational * Rational::from_bigint(sqrt)).inverse();
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
                    class: Class::Exp(exp.clone()),
                    computable: Computable::e(exp),
                });
            }
            _ => (),
        }
        Ok(Self {
            rational: self.rational.inverse(),
            class: Class::Irrational,
            computable: Computable::inverse(self.computable),
        })
    }

    /// The square root of this Real, or a [`RealProblem`] if that's impossible,
    /// in particular RealProblem::SqrtNegative if this Real is negative
    pub fn sqrt(self) -> Result<Real, RealProblem> {
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
                    if rest == Rational::one() {
                        return Ok(Self {
                            rational: square,
                            class: Class::One,
                            computable: Computable::one(),
                        });
                    } else {
                        return Ok(Self {
                            rational: square,
                            class: Class::Sqrt(rest.clone()),
                            computable: Computable::sqrt_rational(rest),
                        });
                    }
                }
            }
            Class::Pi => {
                if self.rational.extract_square_will_succeed() {
                    let (square, rest) = self.rational.clone().extract_square_reduced();
                    if rest == Rational::one() {
                        return Ok(Self {
                            rational: square,
                            class: Class::Irrational,
                            computable: Computable::sqrt(self.computable),
                        });
                    }
                }
            }
            Class::Exp(exp) => {
                if self.rational.extract_square_will_succeed() {
                    let (square, rest) = self.rational.clone().extract_square_reduced();
                    if rest == Rational::one() {
                        let exp = exp.clone() / Rational::new(2);
                        return Ok(Self {
                            rational: square,
                            class: Class::Exp(exp.clone()),
                            computable: Computable::e(exp),
                        });
                    }
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::sqrt))
    }

    /// Apply the exponential function to this Real parameter
    pub fn exp(self) -> Result<Real, RealProblem> {
        if self.definitely_zero() {
            return Ok(Self::new(Rational::one()));
        }
        match &self.class {
            Class::One => {
                return Ok(Self {
                    rational: Rational::one(),
                    class: Class::Exp(self.rational.clone()),
                    computable: Computable::e(self.rational),
                })
            }
            Class::Ln(ln) => {
                if self.rational == Rational::one() {
                    return Ok(Self {
                        rational: ln.clone(),
                        class: Class::One,
                        computable: Computable::one(),
                    });
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::exp))
    }

    /// The natural logarithm of this Real
    pub fn ln(self) -> Result<Real, RealProblem> {
        match &self.class {
            Class::One => {
                if self.rational == Rational::one() {
                    return Ok(Self::zero());
                } else {
                    let new = Computable::rational(self.rational.clone());
                    return Ok(Self {
                        rational: Rational::one(),
                        class: Class::Ln(self.rational),
                        computable: Computable::ln(new),
                    });
                }
            }
            Class::Exp(exp) => {
                if self.rational == Rational::one() {
                    return Ok(Self {
                        rational: exp.clone(),
                        class: Class::One,
                        computable: Computable::one(),
                    });
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::ln))
    }

    /// Is this Real a whole number aka integer ?
    pub fn is_whole(&self) -> bool {
        self.class == Class::One && self.rational.is_whole()
    }

    /// Is this Real known to be rational ?
    pub fn is_rational(&self) -> bool {
        self.class == Class::One
    }

    /// Should we display this Real as a fraction ?
    pub fn prefer_fraction(&self) -> bool {
        self.class == Class::One && self.rational.prefer_fraction()
    }
}

use core::fmt;

impl Real {
    /// Format this Real as a decimal rather than rational
    pub fn decimal(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.class == Class::One {
            if let Some(precision) = f.precision() {
                f.write_fmt(format_args!("{:#.*}", precision, self.rational))
            } else {
                f.write_fmt(format_args!("{:#}", self.rational))
            }
        } else {
            self.rational.fmt_combine(&self.computable, f)
        }
    }
}

impl fmt::Display for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.class {
            Class::One => {
                if f.alternate() {
                    return self.decimal(f);
                } else {
                    f.write_fmt(format_args!("{}", self.rational))?;
                }
            }
            Class::Pi => {
                if f.alternate() {
                    return self.decimal(f);
                } else {
                    f.write_fmt(format_args!("{} Pi", self.rational))?;
                }
            }
            Class::Exp(n) => {
                if f.alternate() {
                    return self.decimal(f);
                } else {
                    f.write_fmt(format_args!("{} x e**({})", self.rational, &n))?;
                }
            }
            Class::Ln(n) => {
                if f.alternate() {
                    return self.decimal(f);
                } else {
                    f.write_fmt(format_args!("{} x ln({})", self.rational, &n))?;
                }
            }
            Class::Sqrt(n) => {
                if f.alternate() {
                    return self.decimal(f);
                } else {
                    f.write_fmt(format_args!("{} √({})", self.rational, &n))?;
                }
            }
            _ => {
                if f.alternate() {
                    return self.decimal(f);
                } else {
                    f.write_fmt(format_args!("{} x {:?}", self.rational, self.class))?;
                }
            }
        }

        Ok(())
    }
}

use std::str::FromStr;

impl FromStr for Real {
    type Err = RealProblem;

    fn from_str(s: &str) -> Result<Self, RealProblem> {
        let rational: Rational = s.parse().map_err(|_| RealProblem::ParseError)?;
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
            if rational.sign() == Sign::NoSign {
                return Self::zero();
            } else {
                return Self { rational, ..self };
            }
        }
        if self.definitely_zero() {
            return other;
        }
        if other.definitely_zero() {
            return self;
        }
        if self.class == Class::One && self.rational == Rational::one() {
            return other;
        }
        if other.class == Class::One && other.rational == Rational::one() {
            return self;
        }
        let left = if self.class == Class::One {
            Computable::rational(self.rational)
        } else if self.rational == Rational::one() {
            self.computable
        } else {
            let lr = Computable::rational(self.rational);
            Computable::multiply(lr, self.computable)
        };
        let right = if other.class == Class::One {
            Computable::rational(other.rational)
        } else if other.rational == Rational::one() {
            other.computable
        } else {
            let rr = Computable::rational(other.rational);
            Computable::multiply(rr, other.computable)
        };
        let computable = Computable::add(left, right);
        Self {
            rational: Rational::one(),
            class: Class::Irrational,
            computable,
        }
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
    fn multiply_sqrts(x: Rational, y: Rational) -> Self {
        if x == y {
            Self {
                rational: x,
                class: One,
                computable: Computable::one(),
            }
        } else {
            let product = x * y;
            if product == Rational::zero() {
                return Self {
                    rational: product,
                    class: One,
                    computable: Computable::one(),
                };
            }
            let (a, b) = product.extract_square_reduced();
            Self {
                rational: a,
                class: Sqrt(b.clone()),
                computable: Computable::sqrt_rational(b),
            }
        }
    }
}

impl Mul for Real {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        if self.definitely_zero() || other.definitely_zero() {
            return Self::zero();
        }
        if self.class == One {
            let rational = self.rational * other.rational;
            return Self { rational, ..other };
        }
        if other.class == One {
            let rational = self.rational * other.rational;
            return Self { rational, ..self };
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
                let (class, computable) = Class::make_exp(r + s);
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class,
                    computable,
                }
            }
            (Class::Pi, Class::Pi) => {
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: Class::Irrational,
                    computable: Computable::square(Computable::pi()),
                }
            }
            _ => {
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: Class::Irrational,
                    computable: Computable::multiply(self.computable, other.computable),
                }
            }
        }
    }
}

impl Div for Real {
    type Output = Result<Self, RealProblem>;

    fn div(self, other: Self) -> Result<Self, RealProblem> {
        if self.class == other.class {
            let rational = self.rational / other.rational;
            return Ok(Self::new(rational));
        }
        if other.class == One {
            let rational = self.rational / other.rational;
            return Ok(Self { rational, ..self });
        }
        if self.definitely_zero() {
            return Ok(Self::zero());
        }
        if other.definitely_zero() {
            return Err(RealProblem::DivideByZero);
        }

        let inverted = other.inverse()?;
        Ok(self * inverted)
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
        let answer = four - two;
        let two: Real = "2".parse().unwrap();
        assert_eq!(answer, two);
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
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let a = small * two;
        let three: Real = "3".parse().unwrap();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let three: Real = "3".parse().unwrap();
        let b = small * three;
        let answer = a * b;
        let eighteen: Real = "18".parse().unwrap();
        assert_eq!(answer, eighteen);
    }
}
