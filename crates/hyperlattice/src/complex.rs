use std::fmt;
use std::ops::{Add, BitXor, Div, Mul, Neg, Sub};

use crate::scalar::{one, require_known_nonzero, zero};
use crate::{BlasResult, CheckedBlasResult, Problem, Scalar};

#[derive(Clone, Debug, PartialEq)]
pub struct Complex {
    pub re: Scalar,
    pub im: Scalar,
}

impl Complex {
    pub fn new(re: Scalar, im: Scalar) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self::new(zero(), zero())
    }

    pub fn one() -> Self {
        Self::new(one(), zero())
    }

    pub fn i() -> Self {
        Self::new(zero(), one())
    }

    pub fn conjugate(self) -> Self {
        Self::new(self.re, -self.im)
    }

    pub fn norm_squared(&self) -> Scalar {
        self.re.clone() * self.re.clone() + self.im.clone() * self.im.clone()
    }

    pub fn reciprocal(self) -> BlasResult<Self> {
        let denom = self.norm_squared();
        Ok(Self::new((self.re / denom.clone())?, ((-self.im) / denom)?))
    }

    pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
        let denom = self.norm_squared();
        require_known_nonzero(&denom)?;
        Ok(Self::new((self.re / denom.clone())?, ((-self.im) / denom)?))
    }

    pub fn powi(self, exponent: i64) -> BlasResult<Self> {
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
        }

        let mut exp = exponent.unsigned_abs();
        let mut result = Self::one();
        let mut factor = self;
        while exp > 0 {
            if exp & 1 == 1 {
                result = result * factor.clone();
            }
            exp >>= 1;
            if exp > 0 {
                factor = factor.clone() * factor;
            }
        }

        if exponent < 0 {
            result.reciprocal()
        } else {
            Ok(result)
        }
    }

    pub fn powi_checked(self, exponent: i64) -> CheckedBlasResult<Self> {
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
        }

        let mut exp = exponent.unsigned_abs();
        let mut result = Self::one();
        let mut factor = self;
        while exp > 0 {
            if exp & 1 == 1 {
                result = result * factor.clone();
            }
            exp >>= 1;
            if exp > 0 {
                factor = factor.clone() * factor;
            }
        }

        if exponent < 0 {
            result.reciprocal_checked()
        } else {
            Ok(result)
        }
    }

    pub fn div_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
        let denom = rhs.norm_squared();
        require_known_nonzero(&denom)?;
        Ok(Self::new(
            ((self.re.clone() * rhs.re.clone() + self.im.clone() * rhs.im.clone())
                / denom.clone())?,
            ((self.im * rhs.re - self.re * rhs.im) / denom)?,
        ))
    }

    pub fn div_real_checked(self, rhs: Scalar) -> CheckedBlasResult<Self> {
        require_known_nonzero(&rhs)?;
        Ok(Self::new((self.re / rhs.clone())?, (self.im / rhs)?))
    }
}

impl From<Scalar> for Complex {
    fn from(value: Scalar) -> Self {
        Self::new(value, zero())
    }
}

impl fmt::Display for Complex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            write!(f, "({:#} + {:#}i)", self.re, self.im)
        } else {
            write!(f, "({} + {}i)", self.re, self.im)
        }
    }
}

impl Add for Complex {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl Sub for Complex {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl Neg for Complex {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.re, -self.im)
    }
}

impl Mul for Complex {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let re = self.re.clone() * rhs.re.clone() - self.im.clone() * rhs.im.clone();
        let im = self.re * rhs.im + self.im * rhs.re;
        Self::new(re, im)
    }
}

impl Div for Complex {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Self) -> Self::Output {
        let denom = rhs.norm_squared();
        Ok(Self::new(
            ((self.re.clone() * rhs.re.clone() + self.im.clone() * rhs.im.clone())
                / denom.clone())?,
            ((self.im * rhs.re - self.re * rhs.im) / denom)?,
        ))
    }
}

impl Div<Scalar> for Complex {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Scalar) -> Self::Output {
        Ok(Self::new((self.re / rhs.clone())?, (self.im / rhs)?))
    }
}

impl BitXor<i64> for Complex {
    type Output = BlasResult<Self>;

    fn bitxor(self, rhs: i64) -> Self::Output {
        self.powi(rhs)
    }
}
