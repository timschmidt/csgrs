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

#[derive(Clone, Copy, Debug)]
pub(crate) enum Vector4HomogeneousKind {
    /// A vector lies on the implicit vector subspace in homogeneous coordinates.
    Direction,
    /// A point has unit homogeneous coordinate in projective form.
    Point,
    /// `w` is neither provably zero nor one from structural facts.
    Unknown,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Vector4GeometricFacts {
    /// Whether this 4-vector is structurally a point, direction, or unknown.
    pub(crate) homogeneous: Vector4HomogeneousKind,
}

#[inline]
fn vector4_geometric_facts<B: Backend>(values: &[Scalar<B>; 4]) -> Vector4GeometricFacts {
    // Homogeneous geometry kernels in exact computation benefit from keeping
    // `w`-classification as retained structure; this is the projective split
    // used throughout 3D affine pipelines, and mirrors the direction/point
    // specialization logic that precedes exact reductions in robust kernels
    // (Yap, “Towards Exact Geometric Computation”, 1997).
    let homogeneous = match values[3].zero_or_one() {
        Some(false) => Vector4HomogeneousKind::Direction,
        Some(true) => Vector4HomogeneousKind::Point,
        None => Vector4HomogeneousKind::Unknown,
    };
    Vector4GeometricFacts { homogeneous }
}

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
                crate::trace_dispatch!("hyperlattice_vector", "constructor", "new");
                Self(values)
            }

            /// Returns the zero vector.
            pub fn zero() -> Self {
                crate::trace_dispatch!("hyperlattice_vector", "constructor", "zero");
                Self(from_fn(|_| Scalar::zero()))
            }

            /// Returns the Euclidean magnitude.
            pub fn magnitude(&self) -> BlasResult<Scalar<B>> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "magnitude");
                self.magnitude_squared_fast().sqrt()
            }

            /// Returns the Euclidean magnitude after attaching an abort signal.
            pub fn magnitude_with_abort(&self, signal: &AbortSignal) -> BlasResult<Scalar<B>> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "magnitude-with-abort");
                with_abort(self.dot_with_abort(self, signal), signal).sqrt()
            }

            #[inline]
            fn magnitude_squared_fast(&self) -> Scalar<B> {
                // Magnitude is a self-dot, so each structural zero fact is
                // shared by both multiplicands. Use the dedicated self-dot
                // kernels to avoid redundant fact probes while keeping dense
                // inputs on the backend dot hook; that preserves hyperreal's
                // deferred exact-rational reduction strategy.
                if $n == 3 {
                    Scalar::dot3_same([&self.0[0], &self.0[1], &self.0[2]])
                } else {
                    Scalar::dot4_same([&self.0[0], &self.0[1], &self.0[2], &self.0[3]])
                }
            }

            /// Returns a unit vector in the same direction.
            ///
            /// This rejects definite zero magnitudes before division. If the
            /// scalar backend rejects a divisor for another reason, that
            /// [`Problem`](crate::Problem) is propagated.
            pub fn normalize(&self) -> BlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "normalize");
                let mag = self.magnitude()?;
                reject_definite_zero(&mag)?;
                let inv_mag = mag.inverse()?;
                // Keep the borrowed `Mul` form here. A `mul_cached` prototype
                // reused the reciprocal magnitude like matrix inverse scaling,
                // but Criterion regressed approx, hyperreal, and
                // hyperreal-rational normalize rows. For 3/4 component vectors,
                // preserving the backend's direct borrowed multiply is cheaper
                // than forcing the matrix shared-scale path.
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Returns a unit vector after rejecting zero and unknown-zero magnitudes.
            pub fn normalize_checked(&self) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "normalize-checked");
                let mag_squared = self.magnitude_squared_fast();
                require_known_nonzero(&mag_squared)?;
                let mag = mag_squared.sqrt()?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                // See `normalize`: direct borrowed multiply benchmarks faster
                // than forcing the matrix shared-scale helper for vectors.
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Returns a checked unit vector after attaching an abort signal.
            pub fn normalize_checked_with_abort(
                &self,
                signal: &AbortSignal,
            ) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!(
                    "hyperlattice_vector",
                    "method",
                    "normalize-checked-with-abort"
                );
                let mag_squared = with_abort(self.dot_with_abort(self, signal), signal);
                require_known_nonzero(&mag_squared)?;
                let mag = mag_squared.sqrt()?;
                require_known_nonzero(&mag)?;
                let inv_mag = mag.inverse()?;
                // See `normalize`: direct borrowed multiply keeps this vector
                // path faster after abort-aware magnitude construction.
                Ok(Self(from_fn(|i| &self.0[i] * &inv_mag)))
            }

            /// Divides every component by `rhs` after rejecting unknown-zero divisors.
            pub fn div_scalar_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
                crate::trace_dispatch!("hyperlattice_vector", "method", "div-scalar-checked");
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
                    "hyperlattice_vector",
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-owned-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-owned-ref");
                Self(map_array_ref(self.0, &rhs.0, Scalar::add_cached))
            }
        }

        impl<B: Backend> Add<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn add(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-ref-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-ref-ref");
                $name(from_fn(|i| &self.0[i] + &rhs.0[i]))
            }
        }

        impl<B: Backend> Add<Scalar<B>> for $name<B> {
            type Output = Self;

            fn add(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-scalar-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "add-scalar-ref");
                Self(self.0.map(|value| value.add_cached(rhs)))
            }
        }

        impl<B: Backend> Sub for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-owned-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-owned-ref");
                Self(map_array_ref(self.0, &rhs.0, Scalar::sub_cached))
            }
        }

        impl<B: Backend> Sub<$name<B>> for &$name<B> {
            type Output = $name<B>;

            fn sub(self, rhs: $name<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-ref-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-ref-ref");
                $name(from_fn(|i| &self.0[i] - &rhs.0[i]))
            }
        }

        impl<B: Backend> Sub<Scalar<B>> for $name<B> {
            type Output = Self;

            fn sub(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-scalar-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "sub-scalar-ref");
                Self(self.0.map(|value| value.sub_cached(rhs)))
            }
        }

        impl<B: Backend> Neg for $name<B> {
            type Output = Self;

            fn neg(self) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "neg-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "neg-ref");
                $name(from_fn(|i| -self.0[i].clone()))
            }
        }

        impl<B: Backend> Mul<Scalar<B>> for $name<B> {
            type Output = Self;

            fn mul(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "mul-scalar-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "mul-scalar-ref");
                Self(self.0.map(|value| value.mul_cached(rhs)))
            }
        }

        impl<B: Backend> Div<Scalar<B>> for $name<B> {
            type Output = BlasResult<Self>;

            fn div(self, rhs: Scalar<B>) -> Self::Output {
                crate::trace_dispatch!("hyperlattice_vector", "op", "div-scalar-owned");
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
                crate::trace_dispatch!("hyperlattice_vector", "op", "div-scalar-ref");
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

impl<B: Backend> Vector4<B> {
    #[inline]
    pub(crate) fn geometric_facts(&self) -> Vector4GeometricFacts {
        // Keep geometric classification on the vector object itself so matrix
        // handles can avoid re-reading `zero_or_one` for mixed batches.
        vector4_geometric_facts(&self.0)
    }
}

impl<B: Backend> Vector3<B> {
    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot3");
        Scalar::dot3(
            [&self.0[0], &self.0[1], &self.0[2]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot3-with-abort");
        if !signal.load(Ordering::Relaxed) {
            // 2026-05 trace-guided shortcut: inactive abort signals are the
            // common predicate benchmark case, and clone-and-attach bypasses
            // hyperreal's shared-denominator exact-rational dot path. Reuse
            // the ordinary dot unless a cancellation request is already set;
            // the active path below preserves abort-aware operand attachment.
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-inactive-signal");
            return self.dot(rhs);
        }
        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();
        let has2 = !self.0[2].definitely_zero() && !rhs.0[2].definitely_zero();

        if !has0 && !has1 && !has2 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-all-zero");
            return Scalar::zero();
        }

        let product = |lhs: &Scalar<B>, rhs: &Scalar<B>, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };
        let product_sum2 = |lhs0: &Scalar<B>,
                            rhs0: &Scalar<B>,
                            lhs1: &Scalar<B>,
                            rhs1: &Scalar<B>,
                            signal: &AbortSignal| {
            // The structural scan above has already reduced this active-abort
            // dot to exactly two contributing lanes. Attach the abort signal
            // once per surviving operand, then keep the pair as a product-sum
            // so exact backends can reuse denominator/canonicalization work
            // rather than materializing two independent products plus an add.
            // This is the same delayed-normalization principle as Bareiss,
            // Math. Comp. 22(103), 1968, <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            Scalar::active_signed_product_sum2([true, true], [[&lhs0, &rhs0], [&lhs1, &rhs1]])
        };
        let product_sum3 = |lhs0: &Scalar<B>,
                            rhs0: &Scalar<B>,
                            lhs1: &Scalar<B>,
                            rhs1: &Scalar<B>,
                            lhs2: &Scalar<B>,
                            rhs2: &Scalar<B>,
                            signal: &AbortSignal| {
            // All three lanes survived cheap structural-zero pruning. Keep the
            // active-abort dot as one exact product polynomial after operand
            // attachment instead of reducing three independent products. This
            // mirrors the vec4 sparse-three fast path and follows Bareiss-style
            // delayed normalization for short exact sums:
            // Bareiss, Math. Comp. 22(103), 1968,
            // <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            let lhs2 = clone_with_abort(lhs2, signal);
            let rhs2 = clone_with_abort(rhs2, signal);
            Scalar::active_signed_product_sum2(
                [true, true, true],
                [[&lhs0, &rhs0], [&lhs1, &rhs1], [&lhs2, &rhs2]],
            )
        };
        if has0 as u8 + has1 as u8 + has2 as u8 == 1 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-single");
            return if has0 {
                product(&self.0[0], &rhs.0[0], signal)
            } else if has1 {
                product(&self.0[1], &rhs.0[1], signal)
            } else {
                product(&self.0[2], &rhs.0[2], signal)
            };
        }

        if has0 as u8 + has1 as u8 + has2 as u8 == 2 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-two");
            return if has0 && has1 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], signal)
            } else if has0 && has2 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[2], &rhs.0[2], signal)
            } else {
                product_sum2(&self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal)
            };
        }

        crate::trace_dispatch!("hyperlattice_vector", "abort", "dot3-sparse-three-nonzero");
        product_sum3(
            &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal,
        )
    }
}

impl<B: Backend> Vector4<B> {
    /// Returns the dot product with `rhs`.
    pub fn dot(&self, rhs: &Self) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot4");
        Scalar::dot4(
            [&self.0[0], &self.0[1], &self.0[2], &self.0[3]],
            [&rhs.0[0], &rhs.0[1], &rhs.0[2], &rhs.0[3]],
        )
    }

    /// Returns the dot product after attaching an abort signal to operands.
    pub fn dot_with_abort(&self, rhs: &Self, signal: &AbortSignal) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_vector", "method", "dot4-with-abort");
        if !signal.load(Ordering::Relaxed) {
            // Same inactive-abort policy as `Vector3`: keep matrix/vector
            // benches on the backend dot specialization unless cancellation is
            // already requested.
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-inactive-signal");
            return self.dot(rhs);
        }
        let has0 = !self.0[0].definitely_zero() && !rhs.0[0].definitely_zero();
        let has1 = !self.0[1].definitely_zero() && !rhs.0[1].definitely_zero();
        let has2 = !self.0[2].definitely_zero() && !rhs.0[2].definitely_zero();
        let has3 = !self.0[3].definitely_zero() && !rhs.0[3].definitely_zero();
        let nonzero = has0 as u8 + has1 as u8 + has2 as u8 + has3 as u8;

        if nonzero == 0 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-all-zero");
            return Scalar::zero();
        }

        let product = |lhs: &Scalar<B>, rhs: &Scalar<B>, signal: &AbortSignal| {
            clone_with_abort(lhs, signal) * clone_with_abort(rhs, signal)
        };
        let product_sum2 = |lhs0: &Scalar<B>,
                            rhs0: &Scalar<B>,
                            lhs1: &Scalar<B>,
                            rhs1: &Scalar<B>,
                            signal: &AbortSignal| {
            // Keep the two active-abort lanes as one product-sum after operand
            // attachment. This mirrors the non-abort sparse-dot fast path and
            // preserves exact-rational sharing where abort wrappers still allow
            // the backend to see through to exact structure, following the
            // delayed-normalization principle in Bareiss, Math. Comp. 22(103),
            // 1968, <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            Scalar::active_signed_product_sum2([true, true], [[&lhs0, &rhs0], [&lhs1, &rhs1]])
        };
        let product_sum3 = |lhs0: &Scalar<B>,
                            rhs0: &Scalar<B>,
                            lhs1: &Scalar<B>,
                            rhs1: &Scalar<B>,
                            lhs2: &Scalar<B>,
                            rhs2: &Scalar<B>,
                            signal: &AbortSignal| {
            // Three active sparse lanes are still a short exact polynomial, so
            // attach abort guards once per operand and keep the sum in the
            // backend's fixed-product reducer instead of materializing three
            // products plus two adds. This follows the same delayed
            // normalization rationale as Bareiss fraction-free elimination:
            // keep exact products grouped until the last responsible moment.
            // Bareiss, Math. Comp. 22(103), 1968,
            // <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            let lhs2 = clone_with_abort(lhs2, signal);
            let rhs2 = clone_with_abort(rhs2, signal);
            Scalar::active_signed_product_sum2(
                [true, true, true],
                [[&lhs0, &rhs0], [&lhs1, &rhs1], [&lhs2, &rhs2]],
            )
        };
        let product_sum4 = |lhs0: &Scalar<B>,
                            rhs0: &Scalar<B>,
                            lhs1: &Scalar<B>,
                            rhs1: &Scalar<B>,
                            lhs2: &Scalar<B>,
                            rhs2: &Scalar<B>,
                            lhs3: &Scalar<B>,
                            rhs3: &Scalar<B>,
                            signal: &AbortSignal| {
            // The dense active-abort vec4 dot is a four-term exact polynomial.
            // Keeping all terms in the backend reducer avoids four separate
            // guarded products and three immediate additions, preserving shared
            // denominator/canonicalization work until the fused sum. This is the
            // same fraction-free/delayed-normalization idea used by Bareiss,
            // Math. Comp. 22(103), 1968,
            // <https://doi.org/10.2307/2004533>.
            let lhs0 = clone_with_abort(lhs0, signal);
            let rhs0 = clone_with_abort(rhs0, signal);
            let lhs1 = clone_with_abort(lhs1, signal);
            let rhs1 = clone_with_abort(rhs1, signal);
            let lhs2 = clone_with_abort(lhs2, signal);
            let rhs2 = clone_with_abort(rhs2, signal);
            let lhs3 = clone_with_abort(lhs3, signal);
            let rhs3 = clone_with_abort(rhs3, signal);
            Scalar::active_signed_product_sum2(
                [true, true, true, true],
                [
                    [&lhs0, &rhs0],
                    [&lhs1, &rhs1],
                    [&lhs2, &rhs2],
                    [&lhs3, &rhs3],
                ],
            )
        };

        if nonzero == 1 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-single");
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
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-two");
            return if has0 && has1 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], signal)
            } else if has0 && has2 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[2], &rhs.0[2], signal)
            } else if has0 && has3 {
                product_sum2(&self.0[0], &rhs.0[0], &self.0[3], &rhs.0[3], signal)
            } else if has1 && has2 {
                product_sum2(&self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal)
            } else if has1 && has3 {
                product_sum2(&self.0[1], &rhs.0[1], &self.0[3], &rhs.0[3], signal)
            } else {
                product_sum2(&self.0[2], &rhs.0[2], &self.0[3], &rhs.0[3], signal)
            };
        }

        if nonzero == 3 {
            crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-three");
            return if !has0 {
                product_sum3(
                    &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], &self.0[3], &rhs.0[3], signal,
                )
            } else if !has1 {
                product_sum3(
                    &self.0[0], &rhs.0[0], &self.0[2], &rhs.0[2], &self.0[3], &rhs.0[3], signal,
                )
            } else if !has2 {
                product_sum3(
                    &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[3], &rhs.0[3], signal,
                )
            } else {
                product_sum3(
                    &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], signal,
                )
            };
        }

        crate::trace_dispatch!("hyperlattice_vector", "abort", "dot4-sparse-four");
        product_sum4(
            &self.0[0], &rhs.0[0], &self.0[1], &rhs.0[1], &self.0[2], &rhs.0[2], &self.0[3],
            &rhs.0[3], signal,
        )
    }
}
