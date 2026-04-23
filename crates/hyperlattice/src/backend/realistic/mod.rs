use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};
use std::sync::{Mutex, OnceLock};

use num::bigint::Sign;

use crate::backend::{Backend, BackendScalar as BackendScalarTrait};
use crate::{AbortSignal, BlasResult, Problem, ZeroStatus};

static ZERO: OnceLock<Mutex<BackendScalar>> = OnceLock::new();
static ONE: OnceLock<Mutex<BackendScalar>> = OnceLock::new();
static E: OnceLock<Mutex<BackendScalar>> = OnceLock::new();
static PI: OnceLock<Mutex<BackendScalar>> = OnceLock::new();

fn cached(
    cell: &'static OnceLock<Mutex<BackendScalar>>,
    init: fn() -> BackendScalar,
) -> BackendScalar {
    cell.get_or_init(|| Mutex::new(init()))
        .lock()
        .expect("realistic constant cache lock poisoned")
        .clone()
}

#[derive(Clone, Debug)]
pub struct BackendScalar(pub(crate) realistic::Real);

/// Backend marker for exact computable reals from the `realistic` crate.
#[derive(Clone, Debug, PartialEq)]
pub struct RealisticBackend;

impl Backend for RealisticBackend {
    const MOVE_ELEMENTWISE: bool = true;
    const SPECIALIZE_SCALAR_POWI: bool = true;

    type Repr = BackendScalar;
}

impl BackendScalarTrait for BackendScalar {
    fn zero() -> Self {
        cached(&ZERO, || Self(realistic::Real::zero()))
    }

    fn one() -> Self {
        cached(&ONE, || Self(1.into()))
    }

    fn e() -> Self {
        cached(&E, || Self(realistic::Real::e()))
    }

    fn pi() -> Self {
        cached(&PI, || Self(realistic::Real::pi()))
    }

    fn inverse(self) -> BlasResult<Self> {
        self.0.inverse().map(Self).map_err(Problem::from)
    }

    fn pow(self, exponent: Self) -> BlasResult<Self> {
        self.0.pow(exponent.0).map(Self).map_err(Problem::from)
    }

    fn add_ref(self, rhs: &Self) -> Self {
        Self(&self.0 + &rhs.0)
    }

    fn sub_ref(self, rhs: &Self) -> Self {
        Self(&self.0 - &rhs.0)
    }

    fn mul_ref(self, rhs: &Self) -> Self {
        Self(&self.0 * &rhs.0)
    }

    fn div_ref(self, rhs: &Self) -> BlasResult<Self> {
        (&self.0 / &rhs.0).map(Self).map_err(Problem::from)
    }

    fn exp(self) -> BlasResult<Self> {
        self.0.exp().map(Self).map_err(Problem::from)
    }

    fn ln(self) -> BlasResult<Self> {
        self.0.ln().map(Self).map_err(Problem::from)
    }

    fn log10(self) -> BlasResult<Self> {
        self.0.log10().map(Self).map_err(Problem::from)
    }

    fn sqrt(self) -> BlasResult<Self> {
        self.0.sqrt().map(Self).map_err(Problem::from)
    }

    fn sin(self) -> Self {
        Self(self.0.sin())
    }

    fn cos(self) -> Self {
        Self(self.0.cos())
    }

    fn tan(self) -> BlasResult<Self> {
        self.0.tan().map(Self).map_err(Problem::from)
    }

    fn div(self, rhs: Self) -> BlasResult<Self> {
        (self.0 / rhs.0).map(Self).map_err(Problem::from)
    }

    fn definitely_zero(&self) -> bool {
        self.0.definitely_zero()
    }

    fn zero_status(&self) -> ZeroStatus {
        if self.0.definitely_zero() {
            ZeroStatus::Zero
        } else {
            match self.0.best_sign() {
                Sign::Plus | Sign::Minus => ZeroStatus::NonZero,
                Sign::NoSign => ZeroStatus::Unknown,
            }
        }
    }

    fn abort(&mut self, signal: AbortSignal) {
        self.0.abort(signal);
    }

    fn into_f64(self) -> f64 {
        f64::from(self.0)
    }
}

impl PartialEq for BackendScalar {
    fn eq(&self, rhs: &Self) -> bool {
        self.0 == rhs.0
    }
}

impl fmt::Display for BackendScalar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            write!(f, "{:#}", self.0)
        } else {
            write!(f, "{}", self.0)
        }
    }
}

impl From<realistic::Real> for BackendScalar {
    fn from(value: realistic::Real) -> Self {
        Self(value)
    }
}

impl From<realistic::Rational> for BackendScalar {
    fn from(value: realistic::Rational) -> Self {
        Self(value.into())
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl From<$ty> for BackendScalar {
                fn from(value: $ty) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl TryFrom<f32> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        realistic::Real::try_from(value)
            .map(Self)
            .map_err(Problem::from)
    }
}

impl TryFrom<f64> for BackendScalar {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        realistic::Real::try_from(value)
            .map(Self)
            .map_err(Problem::from)
    }
}

impl Add for BackendScalar {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for BackendScalar {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Neg for BackendScalar {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl Mul for BackendScalar {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl Add<&BackendScalar> for BackendScalar {
    type Output = Self;

    #[inline]
    fn add(self, rhs: &BackendScalar) -> Self::Output {
        Self(&self.0 + &rhs.0)
    }
}

impl Sub<&BackendScalar> for BackendScalar {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: &BackendScalar) -> Self::Output {
        Self(&self.0 - &rhs.0)
    }
}

impl Mul<&BackendScalar> for BackendScalar {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: &BackendScalar) -> Self::Output {
        Self(&self.0 * &rhs.0)
    }
}

impl Div<&BackendScalar> for BackendScalar {
    type Output = Result<Self, Problem>;

    #[inline]
    fn div(self, rhs: &BackendScalar) -> Self::Output {
        (&self.0 / &rhs.0).map(Self).map_err(Problem::from)
    }
}
