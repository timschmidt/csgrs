//! Fixed-size vectors over [`Scalar`](crate::Scalar).

use std::array::from_fn;
use std::fmt;
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{
    clone_with_abort, reject_definite_zero, require_known_nonzero,
    require_known_nonzero_with_abort, with_abort, zero,
};
use crate::{AbortSignal, BlasResult, CheckedBlasResult, Scalar};

/// Three-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector3(
    /// Components stored in `[x, y, z]` order.
    pub [Scalar; 3],
);

/// Four-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector4(
    /// Components stored in `[x, y, z, w]` order.
    pub [Scalar; 4],
);

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl $name {
            /// Constructs a vector from its component array.
            pub fn new(values: [Scalar; $n]) -> Self {
                Self(values)
            }

            /// Returns the zero vector.
            pub fn zero() -> Self {
                Self(from_fn(|_| zero()))
            }

            /// Returns the dot product with `rhs`.
            pub fn dot(&self, rhs: &Self) -> Scalar {
                let mut sum = self.0[0].clone() * rhs.0[0].clone();
                for i in 1..$n {
                    sum = sum + self.0[i].clone() * rhs.0[i].clone();
                }
                sum
            }

            /// Returns the dot product after attaching an abort signal to operands.
            pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar {
                let mut sum =
                    clone_with_abort(&self.0[0], signal) * clone_with_abort(&rhs.0[0], signal);
                for i in 1..$n {
                    sum = sum
                        + clone_with_abort(&self.0[i], signal)
                            * clone_with_abort(&rhs.0[i], signal);
                }
                sum
            }

            /// Returns the Euclidean magnitude.
            pub fn magnitude(&self) -> BlasResult<Scalar> {
                self.dot(self).sqrt()
            }

            /// Returns the Euclidean magnitude after attaching an abort signal.
            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Scalar> {
                with_abort(self.dot_with_abort(self, signal), signal).sqrt()
            }

            /// Returns a unit vector in the same direction.
            ///
            /// This rejects definite zero magnitudes before division. If the
            /// scalar backend rejects a divisor for another reason, that
            /// [`Problem`](crate::Problem) is propagated.
            pub fn normalize(&self) -> BlasResult<Self> {
                let mag = self.magnitude()?;
                reject_definite_zero(&mag)?;
                let inv_mag = mag.inverse()?;
                let mut values = self.0.clone();
                for value in &mut values {
                    *value = value.clone().mul_cached(&inv_mag);
                }
                Ok(Self(values))
            }

            /// Returns a unit vector after rejecting zero and unknown-zero magnitudes.
            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                let mag = self.magnitude()?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                let mut values = self.0.clone();
                for value in &mut values {
                    *value = value.clone().mul_cached(&inv_mag);
                }
                Ok(Self(values))
            }

            /// Returns a checked unit vector after attaching an abort signal.
            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let mag = self.magnitude_with_abort(signal)?;
                require_known_nonzero_with_abort(&mag, signal)?;
                let inv_mag = mag.inverse()?;
                let mut values = self.0.clone();
                for value in &mut values {
                    *value = value.clone().mul_cached(&inv_mag);
                }
                Ok(Self(values))
            }

            /// Divides every component by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar) -> CheckedBlasResult<Self> {
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                let mut values = self.0;
                for value in &mut values {
                    *value = value.clone().mul_cached(&inv_rhs);
                }
                Ok(Self(values))
            }

            /// Divides every component by `rhs` after attaching an abort signal.
            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Scalar,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let rhs = with_abort(rhs, signal);
                require_known_nonzero_with_abort(&rhs, signal)?;
                let inv_rhs = rhs.inverse()?;
                let mut values = self.0;
                for value in &mut values {
                    *value = value.clone().mul_cached(&inv_rhs);
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
                let inv_rhs = rhs.inverse()?;
                let mut values = self.0;
                for value in &mut values {
                    *value = value.clone().mul_cached(&inv_rhs);
                }
                Ok(Self(values))
            }
        }
    };
}

impl_vector!(Vector3, 3);
impl_vector!(Vector4, 4);
