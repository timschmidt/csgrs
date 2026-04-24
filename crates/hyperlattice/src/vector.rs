//! Fixed-size vectors over [`Scalar`](crate::Scalar).

use std::array::from_fn;
use std::fmt;
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub};

use crate::scalar::{clone_with_abort, reject_definite_zero, require_known_nonzero, with_abort};
use crate::{AbortSignal, Backend, BlasResult, CheckedBlasResult, DefaultBackend, Scalar};

/// Three-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector3<B: Backend = DefaultBackend>(
    /// Components stored in `[x, y, z]` order.
    pub [Scalar<B>; 3],
);

/// Four-dimensional vector.
#[derive(Clone, Debug, PartialEq)]
pub struct Vector4<B: Backend = DefaultBackend>(
    /// Components stored in `[x, y, z, w]` order.
    pub [Scalar<B>; 4],
);

fn map_array2<B: Backend, const N: usize, F>(
    left: [Scalar<B>; N],
    right: [Scalar<B>; N],
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, Scalar<B>) -> Scalar<B>,
{
    let mut right = right.into_iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl<B: Backend> $name<B> {
            /// Constructs a vector from its component array.
            pub fn new(values: [Scalar<B>; $n]) -> Self {
                Self(values)
            }

            /// Returns the zero vector.
            pub fn zero() -> Self {
                Self(from_fn(|_| Scalar::zero()))
            }

            /// Returns the Euclidean magnitude.
            pub fn magnitude(&self) -> BlasResult<Scalar<B>> {
                self.dot(self).sqrt()
            }

            /// Returns the Euclidean magnitude after attaching an abort signal.
            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Scalar<B>> {
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
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.clone().map(|value| value.mul_cached(&inv_mag))))
                } else {
                    let mut values = self.0.clone();
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_mag);
                    }
                    Ok(Self(values))
                }
            }

            /// Returns a unit vector after rejecting zero and unknown-zero magnitudes.
            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                let mag = self.magnitude()?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.clone().map(|value| value.mul_cached(&inv_mag))))
                } else {
                    let mut values = self.0.clone();
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_mag);
                    }
                    Ok(Self(values))
                }
            }

            /// Returns a checked unit vector after attaching an abort signal.
            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let mag = self.magnitude_with_abort(signal)?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.clone().map(|value| value.mul_cached(&inv_mag))))
                } else {
                    let mut values = self.0.clone();
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_mag);
                    }
                    Ok(Self(values))
                }
            }

            /// Divides every component by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }

            /// Divides every component by `rhs` after attaching an abort signal.
            pub fn div_scalar_checked_with_abort(
                self,
                rhs: Scalar<B>,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                let rhs = with_abort(rhs, signal);
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }
        }

        impl<B: Backend> Index<usize> for $name<B> {
            type Output = Scalar<B>;

            fn index(&self, index: usize) -> &Self::Output {
                &self.0[index]
            }
        }

        impl<B: Backend> IndexMut<usize> for $name<B> {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                &mut self.0[index]
            }
        }

        impl<B: Backend> fmt::Display for $name<B> {
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

        impl<B: Backend> Add for $name<B> {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_array2(self.0, rhs.0, |lhs, rhs| lhs + rhs))
                } else {
                    Self(from_fn(|i| self.0[i].clone() + rhs.0[i].clone()))
                }
            }
        }

        impl<B: Backend> Add<&$name<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                let mut rhs = rhs.0.iter();
                Self(
                    self.0
                        .map(|lhs| lhs.add_cached(rhs.next().expect("vectors have equal length"))),
                )
            }
        }

        impl<B: Backend> Add<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: $name<B>) -> Self::Output {
                let mut left = self.0.iter();
                $name(rhs.0.map(|rhs| {
                    left.next()
                        .expect("vectors have equal length")
                        .clone()
                        .add_cached(&rhs)
                }))
            }
        }

        impl<B: Backend> Add<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                $name(from_fn(|i| self.0[i].clone().add_cached(&rhs.0[i])))
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| value.add_cached(rhs)))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().add_cached(rhs);
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Add<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: &Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| value.add_cached(rhs)))
                } else {
                    Self(from_fn(|i| self.0[i].clone() + rhs.clone()))
                }
            }
        }

        impl<B: Backend> Sub for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(map_array2(self.0, rhs.0, |lhs, rhs| lhs - rhs))
                } else {
                    Self(from_fn(|i| self.0[i].clone() - rhs.0[i].clone()))
                }
            }
        }

        impl<B: Backend> Sub<&$name<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                let mut rhs = rhs.0.iter();
                Self(
                    self.0
                        .map(|lhs| lhs.sub_cached(rhs.next().expect("vectors have equal length"))),
                )
            }
        }

        impl<B: Backend> Sub<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: $name<B>) -> Self::Output {
                let mut left = self.0.iter();
                $name(
                    rhs.0
                        .map(|rhs| left.next().expect("vectors have equal length").clone() - rhs),
                )
            }
        }

        impl<B: Backend> Sub<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                $name(from_fn(|i| self.0[i].clone().sub_cached(&rhs.0[i])))
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| value.sub_cached(rhs)))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().sub_cached(rhs);
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Sub<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: &Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| value.sub_cached(rhs)))
                } else {
                    Self(from_fn(|i| self.0[i].clone() - rhs.clone()))
                }
            }
        }

        impl<B: Backend> Neg for $name<B> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| -value))
                } else {
                    Self(from_fn(|i| -self.0[i].clone()))
                }
            }
        }

        impl<B: Backend> Neg for &$name<B> {
            type Output = $name<B>;

            fn neg(self) -> Self::Output {
                $name(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                let rhs = &rhs;
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| value.mul_cached(rhs)))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(rhs);
                    }
                    Self(values)
                }
            }
        }

        impl<B: Backend> Mul<&Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: &Scalar<B>) -> Self::Output {
                if B::MOVE_ELEMENTWISE {
                    Self(self.0.map(|value| value.mul_cached(rhs)))
                } else {
                    Self(from_fn(|i| self.0[i].clone() * rhs.clone()))
                }
            }
        }

        impl<B: Backend> Div<Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar<B>) -> Self::Output {
                reject_definite_zero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }
        }

        impl<B: Backend> Div<&Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: &Scalar<B>) -> Self::Output {
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.clone().inverse()?;
                if B::MOVE_ELEMENTWISE {
                    Ok(Self(self.0.map(|value| value.mul_cached(&inv_rhs))))
                } else {
                    let mut values = self.0;
                    for value in &mut values {
                        *value = value.clone().mul_cached(&inv_rhs);
                    }
                    Ok(Self(values))
                }
            }
        }
    };
}

impl_vector!(Vector3, 3);
impl_vector!(Vector4, 4);

impl<B: Backend> Vector3<B> {
    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Scalar<B> {
        Scalar::dot3(
            [&self.0[0], &self.0[1], &self.0[2]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar<B> {
        let p0 = clone_with_abort(&self.0[0], signal) * clone_with_abort(&rhs.0[0], signal);
        let p1 = clone_with_abort(&self.0[1], signal) * clone_with_abort(&rhs.0[1], signal);
        let p2 = clone_with_abort(&self.0[2], signal) * clone_with_abort(&rhs.0[2], signal);
        (p0 + p1) + p2
    }
}

impl<B: Backend> Vector4<B> {
    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Scalar<B> {
        Scalar::dot4(
            [&self.0[0], &self.0[1], &self.0[2], &self.0[3]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2], &rhs.0[3]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar<B> {
        let p0 = clone_with_abort(&self.0[0], signal) * clone_with_abort(&rhs.0[0], signal);
        let p1 = clone_with_abort(&self.0[1], signal) * clone_with_abort(&rhs.0[1], signal);
        let p2 = clone_with_abort(&self.0[2], signal) * clone_with_abort(&rhs.0[2], signal);
        let p3 = clone_with_abort(&self.0[3], signal) * clone_with_abort(&rhs.0[3], signal);
        (p0 + p1) + (p2 + p3)
    }
}
