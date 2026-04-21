use std::array::from_fn;
use std::fmt;
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{
    clone_with_abort, reject_definite_zero, require_known_nonzero,
    require_known_nonzero_with_abort, with_abort, zero,
};
use crate::{AbortSignal, BlasResult, CheckedBlasResult, Scalar};

#[derive(Clone, Debug, PartialEq)]
pub struct Vector3(pub [Scalar; 3]);

#[derive(Clone, Debug, PartialEq)]
pub struct Vector4(pub [Scalar; 4]);

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl $name {
            pub fn new(values: [Scalar; $n]) -> Self {
                Self(values)
            }

            pub fn zero() -> Self {
                Self(from_fn(|_| zero()))
            }

            pub fn dot(&self, rhs: &Self) -> Scalar {
                (0..$n).fold(zero(), |acc, i| acc + self.0[i].clone() * rhs.0[i].clone())
            }

            pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar {
                (0..$n).fold(zero(), |acc, i| {
                    acc + clone_with_abort(&self.0[i], signal) * clone_with_abort(&rhs.0[i], signal)
                })
            }

            pub fn magnitude(&self) -> BlasResult<Scalar> {
                self.dot(self).sqrt()
            }

            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Scalar> {
                with_abort(self.dot_with_abort(self, signal), signal).sqrt()
            }

            pub fn normalize(&self) -> BlasResult<Self> {
                let mag = self.magnitude()?;
                reject_definite_zero(&mag)?;
                let mut values = self.0.clone();
                for value in &mut values {
                    *value = (value.clone() / mag.clone())?;
                }
                Ok(Self(values))
            }

            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                let mag = self.magnitude()?;
                require_known_nonzero(&mag)?;
                let mut values = self.0.clone();
                for value in &mut values {
                    *value = (value.clone() / mag.clone())?;
                }
                Ok(Self(values))
            }

            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let mag = self.magnitude_with_abort(signal)?;
                require_known_nonzero_with_abort(&mag, signal)?;
                let mut values = self.0.clone();
                for value in &mut values {
                    *value = (value.clone() / mag.clone())?;
                }
                Ok(Self(values))
            }

            pub fn div_scalar_checked(self, rhs: Scalar) -> CheckedBlasResult<Self> {
                require_known_nonzero(&rhs)?;
                let mut values = self.0;
                for value in &mut values {
                    *value = (value.clone() / rhs.clone())?;
                }
                Ok(Self(values))
            }

            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Scalar,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let rhs = with_abort(rhs, signal);
                require_known_nonzero_with_abort(&rhs, signal)?;
                let mut values = self.0;
                for value in &mut values {
                    *value = (value.clone() / rhs.clone())?;
                }
                Ok(Self(values))
            }
        }

        impl Index<usize> for $name {
            type Output = Scalar;

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

        impl Add<Scalar> for $name {
            type Output = Self;

            fn add(self, rhs: Scalar) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() + rhs.clone()))
            }
        }

        impl Sub for $name {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() - rhs.0[i].clone()))
            }
        }

        impl Sub<Scalar> for $name {
            type Output = Self;

            fn sub(self, rhs: Scalar) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() - rhs.clone()))
            }
        }

        impl Neg for $name {
            type Output = Self;

            fn neg(self) -> Self::Output {
                Self(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl Mul<Scalar> for $name {
            type Output = Self;

            fn mul(self, rhs: Scalar) -> Self::Output {
                Self(from_fn(|i| self.0[i].clone() * rhs.clone()))
            }
        }

        impl Div<Scalar> for $name {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar) -> Self::Output {
                reject_definite_zero(&rhs)?;
                let mut values = self.0;
                for value in &mut values {
                    *value = (value.clone() / rhs.clone())?;
                }
                Ok(Self(values))
            }
        }
    };
}

impl_vector!(Vector3, 3);
impl_vector!(Vector4, 4);
