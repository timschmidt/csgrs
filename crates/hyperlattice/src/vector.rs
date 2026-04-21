use std::array::from_fn;
use std::fmt;
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{
    clone_with_abort, reject_definite_zero, require_known_nonzero,
    require_known_nonzero_with_abort, with_abort, zero,
};
use crate::{AbortSignal, BlasProblem, BlasResult, CheckedBlasResult, Real};

#[derive(Clone, Debug, PartialEq)]
pub struct Vector3(pub [Real; 3]);

#[derive(Clone, Debug, PartialEq)]
pub struct Vector4(pub [Real; 4]);

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl $name {
            pub fn new(values: [Real; $n]) -> Self {
                Self(values)
            }

            pub fn zero() -> Self {
                Self(from_fn(|_| zero()))
            }

            pub fn dot(&self, rhs: &Self) -> Real {
                (0..$n).fold(zero(), |acc, i| acc + self.0[i].clone() * rhs.0[i].clone())
            }

            pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Real {
                (0..$n).fold(zero(), |acc, i| {
                    acc + clone_with_abort(&self.0[i], signal) * clone_with_abort(&rhs.0[i], signal)
                })
            }

            pub fn magnitude(&self) -> BlasResult<Real> {
                self.dot(self).sqrt()
            }

            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Real> {
                with_abort(self.dot_with_abort(self, signal), signal).sqrt()
            }

            pub fn normalize(&self) -> BlasResult<Self> {
                let mag = self.magnitude()?;
                reject_definite_zero(&mag)?;
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / mag.clone()).unwrap()
                })))
            }

            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                let mag = self.magnitude().map_err(BlasProblem::from)?;
                require_known_nonzero(&mag)?;
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / mag.clone()).unwrap()
                })))
            }

            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let mag = self
                    .magnitude_with_abort(signal)
                    .map_err(BlasProblem::from)?;
                require_known_nonzero_with_abort(&mag, signal)?;
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / mag.clone()).unwrap()
                })))
            }

            pub fn div_scalar_checked(self, rhs: Real) -> CheckedBlasResult<Self> {
                require_known_nonzero(&rhs)?;
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / rhs.clone()).unwrap()
                })))
            }

            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Real,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let rhs = with_abort(rhs, signal);
                require_known_nonzero_with_abort(&rhs, signal)?;
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / rhs.clone()).unwrap()
                })))
            }
        }

        impl Index<usize> for $name {
            type Output = Real;

            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl IndexMut<usize> for $name {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("[")?;
                for i in 0..$n {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    if f.alternate() {
                        write!(f, "{:#}", self.0[i])?;
                    } else {
                        write!(f, "{}", self.0[i])?;
                    }
                }
                f.write_str("]")
            }
        }

        impl Add for $name {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() + rhs.0[i].clone()))
            }
        }

        impl Add<Real> for $name {
            type Output = Self;

            fn add(self, rhs: Real) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() + rhs.clone()))
            }
        }

        impl Sub for $name {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() - rhs.0[i].clone()))
            }
        }

        impl Sub<Real> for $name {
            type Output = Self;

            fn sub(self, rhs: Real) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() - rhs.clone()))
            }
        }

        impl Neg for $name {
            type Output = Self;

            fn neg(self) -> Self::Output {
                Self(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl Mul<Real> for $name {
            type Output = Self;

            fn mul(self, rhs: Real) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() * rhs.clone()))
            }
        }

        impl Div<Real> for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Real) -> Self::Output {
                reject_definite_zero(&rhs)?;
                Ok(Self(from_fn(|i| {
                    (self.0[i].clone() / rhs.clone()).unwrap()
                })))
            }
        }
    };
}

impl_vector!(Vector3, 3);
impl_vector!(Vector4, 4);
