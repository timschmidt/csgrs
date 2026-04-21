use std::error::Error;
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

#[cfg(feature = "realistic-backend")]
pub use realistic::{Rational, Real};

pub type AbortSignal = Arc<AtomicBool>;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Problem {
    ParseError,
    SqrtNegative,
    DivideByZero,
    NotFound,
    InsufficientParameters,
    NotANumber,
    Infinity,
    BadFraction,
    BadDecimal,
    BadInteger,
    OutOfRange,
    NotAnInteger,
    Exhausted,
    UnknownZero,
}

#[cfg(feature = "realistic-backend")]
impl From<realistic::Problem> for Problem {
    fn from(problem: realistic::Problem) -> Self {
        match problem {
            realistic::Problem::ParseError => Self::ParseError,
            realistic::Problem::SqrtNegative => Self::SqrtNegative,
            realistic::Problem::DivideByZero => Self::DivideByZero,
            realistic::Problem::NotFound => Self::NotFound,
            realistic::Problem::InsufficientParameters => Self::InsufficientParameters,
            realistic::Problem::NotANumber => Self::NotANumber,
            realistic::Problem::Infinity => Self::Infinity,
            realistic::Problem::BadFraction => Self::BadFraction,
            realistic::Problem::BadDecimal => Self::BadDecimal,
            realistic::Problem::BadInteger => Self::BadInteger,
            realistic::Problem::OutOfRange => Self::OutOfRange,
            realistic::Problem::NotAnInteger => Self::NotAnInteger,
            realistic::Problem::Exhausted => Self::Exhausted,
            _ => Self::Exhausted,
        }
    }
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

impl Error for Problem {}

pub type BlasResult<T> = Result<T, Problem>;
pub type CheckedBlasResult<T> = BlasResult<T>;
pub type BlasProblem = Problem;

mod backend;

#[derive(Clone, Debug, PartialEq)]
pub struct Scalar(crate::backend::BackendScalar);

#[cfg(feature = "realistic-backend")]
impl PartialEq<Rational> for Scalar {
    fn eq(&self, rhs: &Rational) -> bool {
        self == &Self::from(rhs.clone())
    }
}

#[cfg(feature = "realistic-backend")]
impl PartialEq<Scalar> for Rational {
    fn eq(&self, rhs: &Scalar) -> bool {
        rhs == self
    }
}

impl Scalar {
    #[cfg(feature = "realistic-backend")]
    pub fn new(rational: Rational) -> Self {
        rational.into()
    }

    pub fn approx(value: f64, epsilon: f64) -> BlasResult<Self> {
        #[cfg(all(not(feature = "realistic-backend"), feature = "approx-backend"))]
        {
            crate::backend::BackendScalar::new(value, epsilon).map(Self)
        }

        #[cfg(feature = "realistic-backend")]
        {
            let _ = epsilon;
            Self::try_from(value)
        }
    }

    pub fn zero() -> Self {
        Self(crate::backend::BackendScalar::zero())
    }

    pub fn one() -> Self {
        Self(crate::backend::BackendScalar::one())
    }

    pub fn e() -> Self {
        Self(crate::backend::BackendScalar::e())
    }

    pub fn pi() -> Self {
        Self(crate::backend::BackendScalar::pi())
    }

    pub fn inverse(self) -> BlasResult<Self> {
        self.0.inverse().map(Self)
    }

    pub fn pow(self, exponent: Self) -> BlasResult<Self> {
        self.0.pow(exponent.0).map(Self)
    }

    pub fn exp(self) -> BlasResult<Self> {
        self.0.exp().map(Self)
    }

    pub fn ln(self) -> BlasResult<Self> {
        self.0.ln().map(Self)
    }

    pub fn log10(self) -> BlasResult<Self> {
        self.0.log10().map(Self)
    }

    pub fn sqrt(self) -> BlasResult<Self> {
        self.0.sqrt().map(Self)
    }

    pub fn sin(self) -> Self {
        Self(self.0.sin())
    }

    pub fn cos(self) -> Self {
        Self(self.0.cos())
    }

    pub fn tan(self) -> BlasResult<Self> {
        self.0.tan().map(Self)
    }

    pub fn definitely_zero(&self) -> bool {
        self.0.definitely_zero()
    }

    pub fn zero_status(&self) -> ZeroStatus {
        self.0.zero_status()
    }

    pub fn abort(&mut self, signal: AbortSignal) {
        self.0.abort(signal);
    }
}

impl fmt::Display for Scalar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(feature = "realistic-backend")]
impl From<Real> for Scalar {
    fn from(value: Real) -> Self {
        Self(value.into())
    }
}

#[cfg(feature = "realistic-backend")]
impl From<Rational> for Scalar {
    fn from(value: Rational) -> Self {
        Self(value.into())
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl From<$ty> for Scalar {
                fn from(value: $ty) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl TryFrom<f32> for Scalar {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        crate::backend::BackendScalar::try_from(value).map(Self)
    }
}

impl TryFrom<f64> for Scalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        crate::backend::BackendScalar::try_from(value).map(Self)
    }
}

impl From<Scalar> for f64 {
    fn from(value: Scalar) -> Self {
        value.0.into_f64()
    }
}

impl Add for Scalar {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Scalar {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Neg for Scalar {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl Mul for Scalar {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl Div for Scalar {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Self) -> Self::Output {
        self.0.div(rhs.0).map(Self)
    }
}

mod complex;
mod matrix;
mod scalar;
mod vector;

pub use complex::Complex;
pub use matrix::{Matrix3, Matrix4};
pub use scalar::*;
pub use vector::{Vector3, Vector4};
