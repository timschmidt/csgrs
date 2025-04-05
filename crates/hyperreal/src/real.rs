use crate::{Computable, Problem, Rational};
use num::bigint::{BigInt, BigUint, Sign};
use num::One;

mod convert;
mod test;

#[derive(Clone, Debug)]
enum Class {
    One,            // Exactly one
    Pi,             // Exactly pi
    Sqrt(Rational), // Square root of some positive integer without an integer square root
    Exp(Rational),
    Ln(Rational),
    SinPi(Rational), // 0 < Rational < 1/2 also never 1/6 or 1/4 or 1/3
    Irrational,
}

use Class::*;

// We can't tell whether an Irrational value is ever equal to anything
impl PartialEq for Class {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (One, One) => true,
            (Pi, Pi) => true,
            (Sqrt(r), Sqrt(s)) => r == s,
            (Exp(r), Exp(s)) => r == s,
            (Ln(r), Ln(s)) => r == s,
            (SinPi(r), SinPi(s)) => r == s,
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

mod rationals {
    use crate::Rational;
    use std::sync::LazyLock;

    pub(super) static HALF: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 2).unwrap());
    pub(super) static ONE: LazyLock<Rational> = LazyLock::new(|| Rational::new(1));
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

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

pub type Signal = Arc<AtomicBool>;

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
/// assert_eq!(half, Rational::fraction(1, 2).unwrap());
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
    signal: Option<Signal>,
}

impl Real {
    /// Provide an atomic flag to signal early abort of calculations
    /// The provided flag can be used e.g. from another execution thread
    /// Aborted calculations may have incorrect results
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s.clone());
        self.computable.abort(s);
    }

    /// Zero, the additive identity
    pub fn zero() -> Real {
        Self {
            rational: Rational::zero(),
            class: One,
            computable: Computable::one(),
            signal: None,
        }
    }

    /// The specified [`Rational`] as a Real
    pub fn new(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: Computable::one(),
            signal: None,
        }
    }

    /// π, the ratio of a circle's circumference to its diameter
    pub fn pi() -> Real {
        Self {
            rational: Rational::one(),
            class: Pi,
            computable: Computable::pi(),
            signal: None,
        }
    }

    /// e, Euler's number and the base of the natural logarithm function
    pub fn e() -> Real {
        let one = Rational::one();
        Self {
            rational: one.clone(),
            class: Exp(one.clone()),
            computable: Computable::e(one),
            signal: None,
        }
    }
}

// Sin(r) is a single curve, then reflected, then both halves negated
// returns whether to negate, and the (if necessary reflected) fraction
// 0 < r < 0.5
// Never actually used for exact zero or half
fn curve(r: Rational) -> (bool, Rational) {
    let whole = r.shifted_big_integer(0);
    let mut s = r.fract();
    if s.sign() == Sign::Minus {
        s = s.neg();
    }
    if s > *rationals::HALF {
        s = Rational::one() - s;
    }
    if whole.bit(0) {
        (true, s)
    } else {
        (false, s)
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
            One | Pi | Exp(_) | Sqrt(_) | SinPi(_) => self.rational.sign(),
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
        let computable = convert(self.fold());

        Self {
            rational: Rational::one(),
            class: Irrational,
            computable,
            signal: None,
        }
    }

    /// The inverse of this Real, or a [`Problem`] if that's impossible,
    /// in particular Problem::DivideByZero if this real is zero
    ///
    /// Example
    /// ```
    /// use realistic::{Rational,Real};
    /// let five = Real::new(Rational::new(5));
    /// let a_fifth = Real::new(Rational::fraction(1, 5).unwrap());
    /// assert_eq!(five.inverse(), Ok(a_fifth));
    /// ```
    pub fn inverse(self) -> Result<Self, Problem> {
        if self.definitely_zero() {
            return Err(Problem::DivideByZero);
        }
        match &self.class {
            One => {
                return Ok(Self {
                    rational: self.rational.inverse(),
                    class: One,
                    computable: Computable::one(),
                    signal: None,
                });
            }
            Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.to_big_integer() {
                    let rational = (self.rational * Rational::from_bigint(sqrt)).inverse();
                    return Ok(Self {
                        rational,
                        class: self.class,
                        computable: self.computable,
                        signal: None,
                    });
                }
            }
            Exp(exp) => {
                let exp = Neg::neg(exp.clone());
                return Ok(Self {
                    rational: self.rational.inverse(),
                    class: Exp(exp.clone()),
                    computable: Computable::e(exp),
                    signal: None,
                });
            }
            _ => (),
        }
        Ok(Self {
            rational: self.rational.inverse(),
            class: Irrational,
            computable: Computable::inverse(self.computable),
            signal: None,
        })
    }

    /// The square root of this Real, or a [`Problem`] if that's impossible,
    /// in particular Problem::SqrtNegative if this Real is negative
    pub fn sqrt(self) -> Result<Real, Problem> {
        if self.best_sign() == Sign::Minus {
            return Err(Problem::SqrtNegative);
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
                            signal: None,
                        });
                    } else {
                        return Ok(Self {
                            rational: square,
                            class: Sqrt(rest.clone()),
                            computable: Computable::sqrt_rational(rest),
                            signal: None,
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
                            signal: None,
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
                            signal: None,
                        });
                    }
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::sqrt))
    }

    /// Apply the exponential function to this Real parameter
    pub fn exp(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            return Ok(Self::new(Rational::one()));
        }
        match &self.class {
            One => {
                return Ok(Self {
                    rational: Rational::one(),
                    class: Exp(self.rational.clone()),
                    computable: Computable::e(self.rational),
                    signal: None,
                })
            }
            Ln(ln) => {
                if self.rational == Rational::one() {
                    return Ok(Self {
                        rational: ln.clone(),
                        class: One,
                        computable: Computable::one(),
                        signal: None,
                    });
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::exp))
    }

    /// The natural logarithm of this Real or Problem::NotANumber if this Real is negative
    pub fn ln(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            return Err(Problem::NotANumber);
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
                        signal: None,
                    });
                }
            }
            Exp(exp) => {
                if self.rational == Rational::one() {
                    return Ok(Self {
                        rational: exp.clone(),
                        class: One,
                        computable: Computable::one(),
                        signal: None,
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
                    signal: None,
                };
            }
            Pi => {
                if self.rational.is_integer() {
                    return Self::zero();
                }
                let mut r: Option<Real> = None;
                let d = self.rational.denominator();
                if d == unsigned::TWO.deref() {
                    r = Some(Self::new(Rational::one()));
                }
                if d == unsigned::THREE.deref() {
                    r = Some(Self {
                        rational: Rational::fraction(1, 2).unwrap(),
                        class: Sqrt(Rational::new(3)),
                        computable: Computable::sqrt_rational(Rational::new(3)),
                        signal: None,
                    });
                }
                if d == unsigned::FOUR.deref() {
                    r = Some(Self {
                        rational: Rational::fraction(1, 2).unwrap(),
                        class: Sqrt(Rational::new(2)),
                        computable: Computable::sqrt_rational(Rational::new(2)),
                        signal: None,
                    });
                }
                if d == unsigned::SIX.deref() {
                    r = Some(Self::new(Rational::fraction(1, 2).unwrap()));
                }
                if let Some(real) = r {
                    let whole = self.rational.shifted_big_integer(0);
                    if whole.bit(0) {
                        return real.neg();
                    } else {
                        return real;
                    }
                } else {
                    let (neg, r) = curve(self.rational);
                    let new =
                        Computable::multiply(Computable::pi(), Computable::rational(r.clone()));
                    if neg {
                        return Self {
                            rational: Rational::new(-1),
                            class: SinPi(r),
                            computable: Computable::sin(new),
                            signal: None,
                        };
                    } else {
                        return Self {
                            rational: Rational::one(),
                            class: SinPi(r),
                            computable: Computable::sin(new),
                            signal: None,
                        };
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
                    signal: None,
                };
            }
            Pi => {
                let off = Self {
                    rational: self.rational + Rational::fraction(1, 2).unwrap(),
                    class: Pi,
                    computable: self.computable,
                    signal: None,
                };
                return off.sin();
            }
            _ => (),
        }

        self.make_computable(Computable::cos)
    }

    fn recursive_powi(base: &Real, exp: &BigUint) -> Self {
        if exp == &BigUint::one() {
            return base.clone();
        }
        let mut result = Self::new(Rational::one());
        for b in (0..(exp.bits())).rev() {
            result = result.clone() * result;
            if exp.bit(b) {
                result = result * base.clone();
            }
        }
        result
    }

    fn compute_exp_ln_powi(value: Computable, exp: BigInt) -> Option<Computable> {
        match value.sign() {
            Sign::NoSign => None,
            Sign::Plus => Some(value.ln().multiply(Computable::integer(exp)).exp()),
            Sign::Minus => {
                let odd = exp.bit(0);
                let exp = Computable::integer(exp);
                if odd {
                    Some(value.ln().multiply(exp).exp().negate())
                } else {
                    Some(value.ln().multiply(exp).exp())
                }
            }
        }
    }

    fn exp_ln_powi(self, exp: BigInt) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                if exp.sign() == Sign::Minus {
                    Ok(Self::recursive_powi(&self, exp.magnitude()).neg())
                } else {
                    Ok(Self::recursive_powi(&self, exp.magnitude()))
                }
            }
            Sign::Plus => {
                let value = self.fold();
                let exp = Computable::integer(exp);

                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: value.ln().multiply(exp).exp(),
                    signal: None,
                })
            }
            Sign::Minus => {
                let odd = exp.bit(0);
                let value = self.fold();
                let exp = Computable::integer(exp);
                if odd {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: value.ln().multiply(exp).exp().negate(),
                        signal: None,
                    })
                } else {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: value.ln().multiply(exp).exp(),
                        signal: None,
                    })
                }
            }
        }
    }

    /// Raise this Real to some integer exponent
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        if exp == BigInt::one() {
            return Ok(self);
        }
        if exp.sign() == Sign::NoSign {
            if self.definitely_zero() {
                return Err(Problem::NotANumber);
            } else {
                return Ok(Self::new(Rational::one()));
            }
        }
        if exp.sign() == Sign::Minus && self.definitely_zero() {
            return Err(Problem::NotANumber);
        }
        if let Ok(rational) = self.rational.clone().powi(exp.clone()) {
            match &self.class {
                One => {
                    return Ok(Self {
                        rational,
                        class: One,
                        computable: self.computable,
                        signal: None,
                    });
                }
                Sqrt(sqrt) => 'quick: {
                    let odd = exp.bit(0);
                    let Ok(rf2) = sqrt.clone().powi(exp.clone() >> 1) else {
                        break 'quick;
                    };
                    let product = rational * rf2;
                    if odd {
                        let n = Self {
                            rational: product,
                            class: Sqrt(sqrt.clone()),
                            computable: self.computable,
                            signal: None,
                        };
                        return Ok(n);
                    } else {
                        return Ok(Self::new(product));
                    }
                }
                _ => {
                    if let Some(computable) =
                        Self::compute_exp_ln_powi(self.computable.clone(), exp.clone())
                    {
                        return Ok(Self {
                            rational,
                            class: Irrational,
                            computable,
                            signal: None,
                        });
                    }
                }
            }
        }
        self.exp_ln_powi(exp)
    }

    /// Fractional (Non-integer) rational exponent
    fn pow_fraction(self, exponent: Rational) -> Result<Self, Problem> {
        if exponent.denominator() == unsigned::TWO.deref() {
            let n = exponent.shifted_big_integer(1);
            self.powi(n)?.sqrt()
        } else {
            self.pow_arb(Real::new(exponent))
        }
    }

    /// Arbitrary, possibly irrational exponent
    /// NB: Assumed not to be integer
    fn pow_arb(self, exponent: Self) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                if exponent.best_sign() == Sign::Plus {
                    Ok(Real::zero())
                } else {
                    Err(Problem::NotAnInteger)
                }
            }
            Sign::Minus => Err(Problem::NotAnInteger),
            Sign::Plus => {
                let value = self.fold();
                let exp = exponent.fold();

                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: value.ln().multiply(exp).exp(),
                    signal: None,
                })
            }
        }
    }

    /// Raise this Real to some Real exponent
    pub fn pow(self, exponent: Self) -> Result<Self, Problem> {
        if let Exp(ref n) = self.class {
            if n == rationals::ONE.deref() {
                if self.rational == *rationals::ONE.deref() {
                    return exponent.exp();
                } else {
                    let left = Real::new(self.rational).pow(exponent.clone())?;
                    return Ok(left * exponent.exp()?);
                }
            }
        }
        /* could handle self == 10 =>  10 ^ log10(exponent) specially */
        if exponent.class == One {
            let r = exponent.rational;
            match r.to_big_integer() {
                Some(n) => {
                    return self.powi(n);
                }
                None => {
                    return self.pow_fraction(r);
                }
            }
        }
        if exponent.definitely_zero() {
            return self.powi(BigInt::ZERO);
        }
        self.pow_arb(exponent)
    }

    /// Is this Real an integer ?
    pub fn is_integer(&self) -> bool {
        self.class == One && self.rational.is_integer()
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
        match folded.iter_msd_stop(-20) {
            Some(-19..60) => fmt::Display::fmt(&folded, f),
            _ => fmt::LowerExp::fmt(&folded, f),
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
                SinPi(n) => write!(f, " x sin({} x Pi)", &n),
                _ => write!(f, " x {:?}", self.class),
            }
        }
    }
}

impl std::str::FromStr for Real {
    type Err = Problem;

    fn from_str(s: &str) -> Result<Self, Problem> {
        let rational: Rational = s.parse()?;
        Ok(Self {
            rational,
            class: One,
            computable: Computable::one(),
            signal: None,
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
        let left = self.fold();
        let right = other.fold();
        let computable = Computable::add(left, right);
        Self {
            rational: Rational::one(),
            class: Irrational,
            computable,
            signal: None,
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
                signal: None,
            }
        } else {
            let product = x * y;
            if product == Rational::zero() {
                return Self {
                    rational: product,
                    class: One,
                    computable: Computable::one(),
                    signal: None,
                };
            }
            let (a, b) = product.extract_square_reduced();
            if b == Rational::one() {
                return Self {
                    rational: a,
                    class: One,
                    computable: Computable::one(),
                    signal: None,
                };
            }
            Self {
                rational: a,
                class: Sqrt(b.clone()),
                computable: Computable::sqrt_rational(b),
                signal: None,
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
                    signal: None,
                }
            }
            (Pi, Pi) => {
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: Irrational,
                    computable: Computable::square(Computable::pi()),
                    signal: None,
                }
            }
            _ => {
                let rational = self.rational * other.rational;
                Self {
                    rational,
                    class: Irrational,
                    computable: Computable::multiply(self.computable, other.computable),
                    signal: None,
                }
            }
        }
    }
}

impl Div for Real {
    type Output = Result<Self, Problem>;

    fn div(self, other: Self) -> Result<Self, Problem> {
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
            return Err(Problem::DivideByZero);
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
