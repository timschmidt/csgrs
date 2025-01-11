use crate::Computable;
use crate::Rational;
use num::bigint::Sign;
use Class::*;

mod convert;

/// Problems when either parsing or attempting Arithmetic with [`Real`] numbers

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum RealProblem {
    ParseError,
    SqrtNegative,
    DivideByZero,
    NotFound,
    InsufficientParameters,
    NotANumber,
    Infinity,
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

// We can't tell whether an Irrational value is ever equal to anything
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
    // Could treat Exp specially for large negative exponents
    fn is_non_zero(&self) -> bool {
        true
    }

    fn make_exp(br: Rational) -> (Class, Computable) {
        if br == Rational::zero() {
            (One, Computable::one())
        } else {
            (Exp(br.clone()), Computable::e(br))
        }
    }
}

mod unsigned {
    use num::{bigint::ToBigUint, BigUint};
    use std::sync::LazyLock;

    pub(super) static TWO: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&2).unwrap());
    pub(super) static THREE: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&3).unwrap());
    pub(super) static FOUR: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&4).unwrap());
    pub(super) static SIX: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&6).unwrap());
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
/// use realistic::{Real, Rational};
/// let half: Real = "0.5".parse().unwrap();
/// assert_eq!(half, Rational::fraction(1, 2));
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
///
/// Conversion
/// ```
/// use realistic::{Real, Rational};
/// let nine: Real = 9.into();
/// let three = Rational::new(3);
/// let answer = nine.sqrt().unwrap();
/// assert_eq!(answer, three);
/// ```

#[derive(Clone, Debug)]
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
            class: One,
            computable: Computable::one(),
        }
    }

    /// The specified [`Rational`] as a Real
    pub fn new(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: Computable::one(),
        }
    }

    /// π, the ratio of a circle's circumference to its diameter
    pub fn pi() -> Real {
        Self {
            rational: Rational::one(),
            class: Pi,
            computable: Computable::pi(),
        }
    }

    /// e, Euler's number and the base of the natural logarithm function
    pub fn e() -> Real {
        let one = Rational::one();
        Self {
            rational: one.clone(),
            class: Exp(one.clone()),
            computable: Computable::e(one),
        }
    }
}

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
            One | Pi | Exp(_) | Sqrt(_) => self.rational.sign(),
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
            class: Irrational,
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
            One => {
                return Ok(Self {
                    rational: self.rational.inverse(),
                    class: One,
                    computable: Computable::one(),
                });
            }
            Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.to_big_integer() {
                    let rational = (self.rational * Rational::from_bigint(sqrt)).inverse();
                    return Ok(Self {
                        rational,
                        class: self.class,
                        computable: self.computable,
                    });
                }
            }
            Exp(exp) => {
                let exp = Neg::neg(exp.clone());
                return Ok(Self {
                    rational: self.rational.inverse(),
                    class: Exp(exp.clone()),
                    computable: Computable::e(exp),
                });
            }
            _ => (),
        }
        Ok(Self {
            rational: self.rational.inverse(),
            class: Irrational,
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
            One => {
                if self.rational.extract_square_will_succeed() {
                    let (square, rest) = self.rational.extract_square_reduced();
                    if rest == Rational::one() {
                        return Ok(Self {
                            rational: square,
                            class: One,
                            computable: Computable::one(),
                        });
                    } else {
                        return Ok(Self {
                            rational: square,
                            class: Sqrt(rest.clone()),
                            computable: Computable::sqrt_rational(rest),
                        });
                    }
                }
            }
            Pi => {
                if self.rational.extract_square_will_succeed() {
                    let (square, rest) = self.rational.clone().extract_square_reduced();
                    if rest == Rational::one() {
                        return Ok(Self {
                            rational: square,
                            class: Irrational,
                            computable: Computable::sqrt(self.computable),
                        });
                    }
                }
            }
            Exp(exp) => {
                if self.rational.extract_square_will_succeed() {
                    let (square, rest) = self.rational.clone().extract_square_reduced();
                    if rest == Rational::one() {
                        let exp = exp.clone() / Rational::new(2);
                        return Ok(Self {
                            rational: square,
                            class: Exp(exp.clone()),
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
            One => {
                return Ok(Self {
                    rational: Rational::one(),
                    class: Exp(self.rational.clone()),
                    computable: Computable::e(self.rational),
                })
            }
            Ln(ln) => {
                if self.rational == Rational::one() {
                    return Ok(Self {
                        rational: ln.clone(),
                        class: One,
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
        if self.definitely_zero() {
            return Err(RealProblem::NotANumber);
        }
        match &self.class {
            One => {
                if self.rational == Rational::one() {
                    return Ok(Self::zero());
                } else {
                    let new = Computable::rational(self.rational.clone());
                    return Ok(Self {
                        rational: Rational::one(),
                        class: Ln(self.rational),
                        computable: Computable::ln(new),
                    });
                }
            }
            Exp(exp) => {
                if self.rational == Rational::one() {
                    return Ok(Self {
                        rational: exp.clone(),
                        class: One,
                        computable: Computable::one(),
                    });
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::ln))
    }

    /// The sine of this Real
    pub fn sin(self) -> Real {
        if self.definitely_zero() {
            return Self::zero();
        }
        match &self.class {
            One => {
                let new = Computable::rational(self.rational.clone());
                return Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Computable::sin(new),
                };
            }
            Pi => {
                if self.rational.is_whole() {
                    return Self::zero();
                }
                let mut r: Option<Real> = None;
                let d = self.rational.denominator();
                if d == unsigned::TWO.deref() {
                    r = Some(Self::new(Rational::one()));
                }
                if d == unsigned::THREE.deref() {
                    r = Some(Self {
                        rational: Rational::fraction(1, 2),
                        class: Sqrt(Rational::new(3)),
                        computable: Computable::sqrt_rational(Rational::new(3)),
                    });
                }
                if d == unsigned::FOUR.deref() {
                    r = Some(Self {
                        rational: Rational::fraction(1, 2),
                        class: Sqrt(Rational::new(2)),
                        computable: Computable::sqrt_rational(Rational::new(2)),
                    });
                }
                if d == unsigned::SIX.deref() {
                    r = Some(Self::new(Rational::fraction(1, 2)));
                }
                if let Some(real) = r {
                    let whole = self.rational.shifted_big_integer(0);
                    if whole.bit(0) {
                        return real.neg();
                    } else {
                        return real;
                    }
                }
            }
            _ => (),
        }

        self.make_computable(Computable::sin)
    }

    /// The cosine of this Real
    pub fn cos(self) -> Real {
        if self.definitely_zero() {
            return Self::new(Rational::one());
        }
        match &self.class {
            One => {
                let new = Computable::rational(self.rational.clone());
                return Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Computable::cos(new),
                };
            }
            Pi => {
                let off = Self {
                    rational: self.rational + Rational::fraction(1, 2),
                    class: Pi,
                    computable: self.computable,
                };
                return off.sin();
            }
            _ => (),
        }

        self.make_computable(Computable::cos)
    }

    /// Is this Real a whole number aka integer ?
    pub fn is_whole(&self) -> bool {
        self.class == One && self.rational.is_whole()
    }

    /// Is this Real known to be rational ?
    pub fn is_rational(&self) -> bool {
        self.class == One
    }

    /// Should we display this Real as a fraction ?
    pub fn prefer_fraction(&self) -> bool {
        self.class == One && self.rational.prefer_fraction()
    }
}

use core::fmt;

impl Real {
    /// Format this Real as a decimal rather than rational
    /// Scientific notation will be used if the value is very large or small
    pub fn decimal(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.clone().fold();
        let msd = folded.iter_msd();
        if msd > -20 && msd < 60 {
            fmt::Display::fmt(&folded, f)
        } else {
            fmt::LowerExp::fmt(&folded, f)
        }
    }
}

impl fmt::UpperExp for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.clone().fold();
        folded.fmt(f)
    }
}

impl fmt::LowerExp for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.clone().fold();
        folded.fmt(f)
    }
}

impl fmt::Display for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            self.decimal(f)
        } else {
            self.rational.fmt(f)?;
            match &self.class {
                One => Ok(()),
                Pi => f.write_str(" Pi"),
                Exp(n) => write!(f, " x e**({})", &n),
                Ln(n) => write!(f, " x ln({})", &n),
                Sqrt(n) => write!(f, " √({})", &n),
                _ => write!(f, " x {:?}", self.class),
            }
        }
    }
}

impl std::str::FromStr for Real {
    type Err = RealProblem;

    fn from_str(s: &str) -> Result<Self, RealProblem> {
        let rational: Rational = s.parse().map_err(|_| RealProblem::ParseError)?;
        Ok(Self {
            rational,
            class: One,
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
        let left = if self.class == One {
            Computable::rational(self.rational)
        } else if self.rational == Rational::one() {
            self.computable
        } else {
            let lr = Computable::rational(self.rational);
            Computable::multiply(lr, self.computable)
        };
        let right = if other.class == One {
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
            class: Irrational,
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
            if b == Rational::one() {
                return Self {
                    rational: a,
                    class: One,
                    computable: Computable::one(),
                };
            }
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
            (Sqrt(r), Sqrt(s)) => {
                let square = Self::multiply_sqrts(r, s);
                Self {
                    rational: square.rational * self.rational * other.rational,
                    ..square
                }
            }
            (Exp(r), Exp(s)) => {
                let (class, computable) = Class::make_exp(r + s);
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class,
                    computable,
                }
            }
            (Pi, Pi) => {
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: Irrational,
                    computable: Computable::square(Computable::pi()),
                }
            }
            _ => {
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: Irrational,
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

// For a rational this definitely works
impl PartialEq<Rational> for Real {
    fn eq(&self, other: &Rational) -> bool {
        self.class == Class::One && self.rational == *other
    }
}

// Symmetry
impl PartialEq<Real> for Rational {
    fn eq(&self, other: &Real) -> bool {
        other.class == Class::One && *self == other.rational
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
    fn parse() {
        let counting: Real = "123456789".parse().unwrap();
        let answer = Real::new(Rational::new(123456789));
        assert_eq!(counting, answer);
    }

    #[test]
    fn parse_large() {
        let input: Real = "378089444731722233953867379643788100".parse().unwrap();
        let root = Rational::new(614889782588491410);
        let answer = Real::new(root.clone() * root);
        assert_eq!(input, answer);
    }

    #[test]
    fn parse_fraction() {
        let input: Real = "98760/123450".parse().unwrap();
        let answer = Real::new(Rational::fraction(9876, 12345));
        assert_eq!(input, answer);
    }

    #[test]
    fn root_divide() {
        let twenty: Real = 20.into();
        let five: Real = 5.into();
        let a = twenty.sqrt().unwrap();
        let b = five.sqrt().unwrap().inverse().unwrap();
        let answer = a * b;
        let two: Real = 2.into();
        assert_eq!(answer, two);
    }

    #[test]
    fn rational() {
        let two: Real = 2.into();
        assert_ne!(two, Real::zero());
        let four: Real = 4.into();
        let answer = four - two;
        let two: Real = 2.into();
        assert_eq!(answer, two);
        let zero = answer - two;
        assert_eq!(zero, Real::zero());
        let six_half: Real = "13/2".parse().unwrap();
        let opposite = six_half.inverse().unwrap();
        let expected: Real = "2/13".parse().unwrap();
        assert_eq!(opposite, expected);
    }

    // https://devblogs.microsoft.com/oldnewthing/?p=93765
    // "Why does the Windows calculator generate tiny errors when calculating the square root of a
    // perfect square?" (fixed in 2018)
    #[test]
    fn perfect_square() {
        let four: Real = 4.into();
        let two: Real = 2.into();
        let calc = four.sqrt().unwrap() - two;
        assert_eq!(calc, Real::zero());
    }

    #[test]
    fn one_over_e() {
        let one: Real = 1.into();
        let e = Real::e();
        let e_inverse = Real::e().inverse().unwrap();
        let answer = e * e_inverse;
        assert_eq!(one, answer);
        let again = answer.sqrt().unwrap();
        assert_eq!(one, again);
    }

    #[test]
    fn unlike_sqrts() {
        let thirty: Real = 30.into();
        let ten: Real = 10.into();
        let answer = thirty.sqrt().unwrap() * ten.sqrt().unwrap();
        let ten: Real = 10.into();
        let three: Real = 3.into();
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
        let two: Real = 2.into();
        assert_eq!(two_pi, two * Real::pi());
        assert_ne!(two_pi, Rational::new(2));
    }

    #[test]
    fn ln_zero() {
        let zero = Real::zero();
        assert_eq!(zero.ln(), Err(RealProblem::NotANumber));
    }

    #[test]
    fn sqrt_exact() {
        let big: Real = 40_000.into();
        let small: Rational = Rational::new(200);
        let answer = big.sqrt().unwrap();
        assert_eq!(answer, small);
    }

    #[test]
    fn square_sqrt() {
        let two: Real = 2.into();
        let three: Real = 3.into();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let a = small * two;
        let three: Real = 3.into();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let three: Real = 3.into();
        let b = small * three;
        let answer = a * b;
        let eighteen: Rational = Rational::new(18);
        assert_eq!(answer, eighteen);
    }

    #[test]
    fn adding_one_works() {
        let pi = Real::pi();
        let one: Real = 1.into();
        let plus_one = pi + one;
        let float: f64 = plus_one.into();
        assert_eq!(float, 4.141592653589793);
    }

    #[test]
    fn sin_easy() {
        let pi = Real::pi();
        let zero = Real::zero();
        let two: Real = 2.into();
        let two_pi = pi.clone() * two;
        assert_eq!(zero.clone().sin(), zero);
        assert_eq!(pi.clone().sin(), zero);
        assert_eq!(two_pi.clone().sin(), zero);
    }

    #[test]
    fn cos_easy() {
        let pi = Real::pi();
        let zero = Real::zero();
        let one: Real = 1.into();
        let two: Real = 2.into();
        let two_pi = pi.clone() * two;
        let minus_one: Real = (-1).into();
        assert_eq!(zero.clone().cos(), one);
        assert_eq!(pi.clone().cos(), minus_one);
        assert_eq!(two_pi.clone().cos(), one);
    }

    #[test]
    fn sqrt_3045512() {
        let n: Real = 3045512.into();
        let sqrt = n.sqrt().unwrap();
        let root = Rational::new(1234);
        assert_eq!(sqrt.rational, root);
        let two = Rational::new(2);
        assert_eq!(sqrt.class, Sqrt(two));
    }

    #[test]
    fn exp_pi() {
        let pi = Real::pi();
        assert_eq!(format!("{pi:.2e}"), "3.14e0");
        assert_eq!(format!("{pi:.4E}"), "3.1416E0");
        assert_eq!(format!("{pi:.8e}"), "3.14159265e0");
        assert_eq!(format!("{pi:.16E}"), "3.1415926535897932E0");
        assert_eq!(format!("{pi:.32e}"), "3.14159265358979323846264338327950e0");
        assert_eq!(format!("{pi:e}"), "3.1415926535897932384626433832795e0");
    }
}
