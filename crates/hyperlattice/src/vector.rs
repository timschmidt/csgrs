//! Fixed-size vectors over [`Scalar`](crate::Scalar).

use std::array::from_fn;
use std::fmt;
use std::ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub};
use std::sync::atomic::Ordering;

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

fn map_array_ref<B: Backend, const N: usize, F>(
    left: [Scalar<B>; N],
    right: &[Scalar<B>; N],
    mut op: F,
) -> [Scalar<B>; N]
where
    F: FnMut(Scalar<B>, &Scalar<B>) -> Scalar<B>,
{
    let mut right = right.iter();
    left.map(|lhs| op(lhs, right.next().expect("arrays have equal length")))
}

macro_rules! impl_vector {
    ($name:ident, $n:expr) => {
        impl<B: Backend> $name<B> {
            /// Constructs a vector from its component array.
            pub fn new(values: [Scalar<B>; $n]) -> Self {
                crate::trace_dispatch!("realistic_blas_vector", "constructor", "new");
                Self(values)
            }

            /// Returns the zero vector.
            pub fn zero() -> Self {
                crate::trace_dispatch!("realistic_blas_vector", "constructor", "zero");
                Self(from_fn(|_| Scalar::zero()))
            }

            /// Returns the Euclidean magnitude.
            pub fn magnitude(&self) -> BlasResult<Scalar<B>> {
                crate::trace_dispatch!("realistic_blas_vector", "method", "magnitude");
                self.dot(self).sqrt()
            }

            /// Returns the Euclidean magnitude after attaching an abort signal.
            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Scalar<B>> {
                crate::trace_dispatch!("realistic_blas_vector", "method", "magnitude-with-abort");
                with_abort(self.dot_with_abort(self, signal), signal).sqrt()
            }

            /// Returns a unit vector in the same direction.
            ///
            /// This rejects definite zero magnitudes before division. If the
            /// scalar backend rejects a divisor for another reason, that
            /// [`Problem`](crate::Problem) is propagated.
            pub fn normalize(&self) -> BlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_vector", "method", "normalize");
                let mag = self.magnitude()?;
                reject_definite_zero(&mag)?;
                let inv_mag = mag.inverse()?;
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Returns a unit vector after rejecting zero and unknown-zero magnitudes.
            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_vector", "method", "normalize-checked");
                let mag = self.magnitude()?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Returns a checked unit vector after attaching an abort signal.
            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "realistic_blas_vector",
                    "method",
                    "normalize-checked-with-abort"
                );
                let mag = self.magnitude_with_abort(signal)?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Divides every component by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("realistic_blas_vector", "method", "div-scalar-checked");
                require_known_nonzero(&rhs)?;
                let inv_rhs = rhs.inverse()?;
                if B::MOVE_ELEMENTWISE {
                    // Some backends, notably hyperreal, are cheaper when owned vector slots
                    // are consumed element-by-element instead of cloned and overwritten.
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
                crate::trace_dispatch!(
                    "realistic_blas_vector",
                    "method",
                    "div-scalar-checked-with-abort"
                );
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "add-owned-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "add-owned-ref");
                Self(map_array_ref(self.0, &rhs.0, Scalar::add_cached))
            }
        }

        impl<B: Backend> Add<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "add-ref-owned");
                let mut left = self.0.iter();
                $name(
                    rhs.0
                        .map(|rhs| left.next().expect("vectors have equal length") + rhs),
                )
            }
        }

        impl<B: Backend> Add<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "add-ref-ref");
                $name(from_fn(|i| &self.0[i] + &rhs.0[i]))
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "add-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "add-scalar-ref");
                Self(self.0.map(|value| value.add_cached(rhs)))
            }
        }

        impl<B: Backend> Sub for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "sub-owned-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "sub-owned-ref");
                Self(map_array_ref(self.0, &rhs.0, Scalar::sub_cached))
            }
        }

        impl<B: Backend> Sub<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "sub-ref-owned");
                let mut left = self.0.iter();
                $name(
                    rhs.0
                        .map(|rhs| left.next().expect("vectors have equal length") - rhs),
                )
            }
        }

        impl<B: Backend> Sub<&$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: &$name<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "sub-ref-ref");
                $name(from_fn(|i| &self.0[i] - &rhs.0[i]))
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "sub-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "sub-scalar-ref");
                Self(self.0.map(|value| value.sub_cached(rhs)))
            }
        }

        impl<B: Backend> Neg for $name<B> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "neg-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "neg-ref");
                $name(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "mul-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "mul-scalar-ref");
                Self(self.0.map(|value| value.mul_cached(rhs)))
            }
        }

        impl<B: Backend> Div<Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("realistic_blas_vector", "op", "div-scalar-owned");
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
                crate::trace_dispatch!("realistic_blas_vector", "op", "div-scalar-ref");
                reject_definite_zero(rhs)?;
                let inv_rhs = rhs.inverse_ref()?;
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
        crate::trace_dispatch!("realistic_blas_vector", "method", "dot3");
        Scalar::dot3(
            [&self.0[0], &self.0[1], &self.0[2]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar<B> {
        crate::trace_dispatch!("realistic_blas_vector", "method", "dot3-with-abort");
        if !signal.load(Ordering::Relaxed) {
            // 2026-05 trace-guided shortcut: inactive abort signals are the
            // common predicate benchmark case, and clone-and-attach bypasses
            // hyperreal's shared-denominator exact-rational dot path. Reuse
            // the ordinary dot unless a cancellation request is already set;
            // the active path below preserves abort-aware operand attachment.
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot3-inactive-signal");
            return self.dot(rhs);
        }
        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();
        let has2 = !self.0[2].definitely_zero() && !rhs.0[2].definitely_zero();

        if !has0 && !has1 && !has2 {
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot3-sparse-all-zero");
            return Scalar::zero();
        }

        let product = |lhs: &Scalar<B>, rhs: &Scalar<B>, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };

        if has0 as u8 + has1 as u8 + has2 as u8 == 1 {
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot3-sparse-single");
            return if has0 {
                product(&self.0[0], &rhs.0[0], signal)
            } else if has1 {
                product(&self.0[1], &rhs.0[1], signal)
            } else {
                product(&self.0[2], &rhs.0[2], signal)
            };
        }

        if has0 as u8 + has1 as u8 + has2 as u8 == 2 {
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot3-sparse-two");
            return if has0 && has1 {
                product(&self.0[0], &rhs.0[0], signal) + product(&self.0[1], &rhs.0[1], signal)
            } else if has0 && has2 {
                product(&self.0[0], &rhs.0[0], signal) + product(&self.0[2], &rhs.0[2], signal)
            } else {
                product(&self.0[1], &rhs.0[1], signal) + product(&self.0[2], &rhs.0[2], signal)
            };
        }

        crate::trace_dispatch!(
            "realistic_blas_vector",
            "abort",
            "dot3-sparse-three-nonzero"
        );
        let p0 = product(&self.0[0], &rhs.0[0], signal);
        let p1 = product(&self.0[1], &rhs.0[1], signal);
        let p2 = product(&self.0[2], &rhs.0[2], signal);
        (p0 + p1) + p2
    }
}

impl<B: Backend> Vector4<B> {
    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Scalar<B> {
        crate::trace_dispatch!("realistic_blas_vector", "method", "dot4");
        Scalar::dot4(
            [&self.0[0], &self.0[1], &self.0[2], &self.0[3]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2], &rhs.0[3]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar<B> {
        crate::trace_dispatch!("realistic_blas_vector", "method", "dot4-with-abort");
        if !signal.load(Ordering::Relaxed) {
            // Same inactive-abort policy as `Vector3`: keep matrix/vector
            // benches on the backend dot specialization unless cancellation is
            // already requested.
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot4-inactive-signal");
            return self.dot(rhs);
        }
        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();
        let has2 = !self.0[2].definitely_zero() && !rhs.0[2].definitely_zero();
        let has3 = !self.0[3].definitely_zero() && !rhs.0[3].definitely_zero();
        let nonzero = has0 as u8 + has1 as u8 + has2 as u8 + has3 as u8;

        if nonzero == 0 {
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot4-sparse-all-zero");
            return Scalar::zero();
        }

        let product = |lhs: &Scalar<B>, rhs: &Scalar<B>, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };

        if nonzero == 1 {
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot4-sparse-single");
            return if has0 {
                product(&self.0[0], &rhs.0[0], signal)
            } else if has1 {
                product(&self.0[1], &rhs.0[1], signal)
            } else if has2 {
                product(&self.0[2], &rhs.0[2], signal)
            } else {
                product(&self.0[3], &rhs.0[3], signal)
            };
        }

        if nonzero == 2 {
            let (p0, p1) = if has0 && has1 {
                (
                    product(&self.0[0], &rhs.0[0], signal),
                    product(&self.0[1], &rhs.0[1], signal),
                )
            } else if has0 && has2 {
                (
                    product(&self.0[0], &rhs.0[0], signal),
                    product(&self.0[2], &rhs.0[2], signal),
                )
            } else if has0 && has3 {
                (
                    product(&self.0[0], &rhs.0[0], signal),
                    product(&self.0[3], &rhs.0[3], signal),
                )
            } else if has1 && has2 {
                (
                    product(&self.0[1], &rhs.0[1], signal),
                    product(&self.0[2], &rhs.0[2], signal),
                )
            } else if has1 && has3 {
                (
                    product(&self.0[1], &rhs.0[1], signal),
                    product(&self.0[3], &rhs.0[3], signal),
                )
            } else {
                (
                    product(&self.0[2], &rhs.0[2], signal),
                    product(&self.0[3], &rhs.0[3], signal),
                )
            };
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot4-sparse-two");
            return p0 + p1;
        }

        if nonzero == 3 {
            let (p0, p1, p2) = if !has0 {
                (
                    product(&self.0[1], &rhs.0[1], signal),
                    product(&self.0[2], &rhs.0[2], signal),
                    product(&self.0[3], &rhs.0[3], signal),
                )
            } else if !has1 {
                (
                    product(&self.0[0], &rhs.0[0], signal),
                    product(&self.0[2], &rhs.0[2], signal),
                    product(&self.0[3], &rhs.0[3], signal),
                )
            } else if !has2 {
                (
                    product(&self.0[0], &rhs.0[0], signal),
                    product(&self.0[1], &rhs.0[1], signal),
                    product(&self.0[3], &rhs.0[3], signal),
                )
            } else {
                (
                    product(&self.0[0], &rhs.0[0], signal),
                    product(&self.0[1], &rhs.0[1], signal),
                    product(&self.0[2], &rhs.0[2], signal),
                )
            };
            crate::trace_dispatch!("realistic_blas_vector", "abort", "dot4-sparse-three");
            return (p0 + p1) + p2;
        }

        crate::trace_dispatch!("realistic_blas_vector", "abort", "dot4-sparse-four");
        let p0 = product(&self.0[0], &rhs.0[0], signal);
        let p1 = product(&self.0[1], &rhs.0[1], signal);
        let p2 = product(&self.0[2], &rhs.0[2], signal);
        let p3 = product(&self.0[3], &rhs.0[3], signal);
        (p0 + p1) + (p2 + p3)
    }
}
