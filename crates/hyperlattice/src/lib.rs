//! Scalar-centered linear algebra primitives with selectable numeric backends.
//!
//! `hyperlattice` exposes a crate-owned [`Scalar`] type, complex numbers,
//! 3D/4D vectors, and 3x3/4x4 matrices. These types are generic over a backend
//! marker and default to [`DefaultBackend`]. The default `hyperreal-backend`
//! stores scalars as `hyperreal::Real` values and re-exports `Real` and
//! `Rational` for explicit interop. The `approx-backend` stores an `f64` center
//! value plus an absolute `f64` error bound, which lets tests and callers
//! exercise unknown-zero paths without depending on computable-real evaluation.
//!
//! Backend features gate backend availability. The default feature set enables
//! `hyperreal-backend`; use `default-features = false, features =
//! ["approx-backend"]` to make the approximate backend the default, or enable
//! both features and use [`Scalar`] with [`HyperrealBackend`] and
//! [`ApproxBackend`] explicitly.
//!
//! Most arithmetic that can fail returns [`BlasResult`]. Checked APIs use
//! [`ZeroStatus`] and reject both definite zero and unknown-zero divisors,
//! returning [`Problem::UnknownZero`] for the latter.
//!
//! # Examples
//!
//! ```
//! use hyperlattice::{Matrix3, Scalar, Vector3, sqrt};
//!
//! fn s(value: i32) -> Scalar {
//!     value.into()
//! }
//!
//! let v = Vector3::new([s(3), s(4), s(0)]);
//! assert_eq!(v.dot(&v), s(25));
//! assert_eq!(sqrt(v.dot(&v)).unwrap(), s(5));
//!
//! let identity = Matrix3::identity();
//! assert_eq!(identity * v.clone(), v);
//! ```

#![warn(missing_docs)]

use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[cfg(feature = "hyperreal-backend")]
pub use hyperreal::{Rational, Real};

mod trace;
pub(crate) use trace::trace_dispatch;

mod error;
pub use error::{AbortSignal, BlasResult, CheckedBlasResult, Problem};

mod backend;

#[cfg(feature = "approx-backend")]
pub use backend::ApproxBackend;
pub use backend::DefaultBackend;
#[cfg(feature = "hyperreal-backend")]
pub use backend::HyperrealBackend;
pub use backend::{Backend, BackendScalar};

/// Crate-owned scalar value used throughout the public API.
///
/// The backend type parameter selects the representation and arithmetic
/// behavior. The default backend is [`DefaultBackend`], which resolves to the
/// hyperreal backend when the default feature set is enabled.
pub struct Scalar<B: Backend = DefaultBackend>(B::Repr);

impl<B: Backend> Clone for Scalar<B> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<B: Backend> fmt::Debug for Scalar<B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl<B: Backend> PartialEq for Scalar<B> {
    fn eq(&self, rhs: &Self) -> bool {
        self.0 == rhs.0
    }
}

#[cfg(feature = "hyperreal-backend")]
impl PartialEq<Rational> for Scalar<HyperrealBackend> {
    fn eq(&self, rhs: &Rational) -> bool {
        self == &Self::from(rhs.clone())
    }
}

#[cfg(feature = "hyperreal-backend")]
impl PartialEq<Scalar<HyperrealBackend>> for Rational {
    fn eq(&self, rhs: &Scalar<HyperrealBackend>) -> bool {
        rhs == self
    }
}

impl<B: Backend> Scalar<B> {
    /// Returns the additive identity.
    pub fn zero() -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "zero");
        Self(B::Repr::zero())
    }

    /// Returns the multiplicative identity.
    pub fn one() -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "one");
        Self(B::Repr::one())
    }

    /// Returns Euler's number.
    pub fn e() -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "e");
        Self(B::Repr::e())
    }

    /// Returns pi.
    pub fn pi() -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "pi");
        Self(B::Repr::pi())
    }

    /// Returns tau, equal to `2 * pi`.
    pub fn tau() -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "tau");
        Self(B::Repr::tau())
    }

    /// Returns the multiplicative inverse of this scalar.
    pub fn inverse(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "inverse-owned");
        self.0.inverse().map(Self)
    }

    /// Returns the multiplicative inverse without consuming this scalar.
    pub fn inverse_ref(&self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "inverse-ref");
        self.0.inverse_ref().map(Self)
    }

    #[inline]
    pub(crate) fn mul_cached(self, factor: &Self) -> Self {
        // Hot elementwise kernels often reuse one scalar factor across an
        // entire vector/matrix. Keeping the factor borrowed avoids cloning
        // hyperreal expression graphs for every lane.
        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "mul-cached");
        Self(self.0.mul_ref(&factor.0))
    }

    #[inline]
    pub(crate) fn add_cached(self, rhs: &Self) -> Self {
        // Same borrowed-factor pattern as `mul_cached`; this is intentionally
        // tiny but centralizes the "owned lhs, borrowed rhs" fast path.
        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "add-cached");
        Self(self.0.add_ref(&rhs.0))
    }

    #[inline]
    pub(crate) fn sub_cached(self, rhs: &Self) -> Self {
        // Avoid cloning the scalar subtrahend in repeated elementwise kernels.
        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "sub-cached");
        Self(self.0.sub_ref(&rhs.0))
    }

    #[inline]
    pub(crate) fn is_exact_dyadic_rational(&self) -> bool {
        // Matrix right-division uses this representation fact to choose between
        // shared-adjugate scaling and Gauss-Jordan solving. Dyadic exact
        // rationals reduce by shifts, so extra products can be cheaper than
        // repeated pivot inverses; non-dyadic rationals usually need BigInt gcds.
        crate::trace_dispatch!("hyperlattice", "scalar_query", "exact-dyadic-rational");
        self.0.is_exact_dyadic_rational()
    }

    #[inline]
    pub(crate) fn is_exact_rational(&self) -> bool {
        // Reuse the backend's existing structural certificate. This is a
        // representation fact, not an approximate scalar query, so matrix
        // dispatch can decide between reduction schedules before entering dense
        // arithmetic lanes.
        crate::trace_dispatch!("hyperlattice", "scalar_query", "exact-rational");
        self.0.is_exact_rational()
    }

    #[inline]
    pub(crate) fn dot3(left: [&Self; 3], right: [&Self; 3]) -> Self {
        // Avoid introducing symbolic products when a lane is definitely zero:
        // this preserves deferred exact-rational constructors and skips both
        // clone and multiplication work on sparse rows/columns.
        let left0_zero = left[0].definitely_zero() || right[0].definitely_zero();
        let left1_zero = left[1].definitely_zero() || right[1].definitely_zero();
        let left2_zero = left[2].definitely_zero() || right[2].definitely_zero();
        let nonzero_lanes =
            usize::from(!left0_zero) + usize::from(!left1_zero) + usize::from(!left2_zero);

        if nonzero_lanes == 0 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-all-zero");
            return Self::zero();
        }

        if nonzero_lanes == 1 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-single-term");
            return if !left0_zero {
                left[0] * right[0]
            } else if !left1_zero {
                left[1] * right[1]
            } else {
                left[2] * right[2]
            };
        }

        if nonzero_lanes == 2 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-sparse");
            return if left0_zero {
                // The two surviving lanes are known nonzero by the structural
                // scan above. Keep them as one product-sum so exact backends can
                // share denominator/canonicalization work instead of expanding
                // two independent products. This mirrors the delayed
                // normalization idea used in Bareiss fraction-free elimination
                // (Math. Comp. 22(103), 1968, https://doi.org/10.2307/2004533).
                Self::active_signed_product_sum2(
                    [true, true],
                    [[left[1], right[1]], [left[2], right[2]]],
                )
            } else if left1_zero {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[left[0], right[0]], [left[2], right[2]]],
                )
            } else if left2_zero {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[left[0], right[0]], [left[1], right[1]]],
                )
            } else {
                unreachable!("nonzero lane count checked")
            };
        }

        if B::FUSE_SIGNED_PRODUCT_SUM {
            // The scan above has already proved every lane active. Exact
            // backends can use the active dot hook without re-entering scalar
            // zero classification inside the dot-product hook.
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-active");
            return Self(B::Repr::active_dot3(
                [&left[0].0, &left[1].0, &left[2].0],
                [&right[0].0, &right[1].0, &right[2].0],
            ));
        }

        // Route full 3-lane approximate dots through the backend so compact
        // representations can choose a better add/mul ordering than the
        // default trait methods.
        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-backend");
        Self(B::Repr::dot3(
            [&left[0].0, &left[1].0, &left[2].0],
            [&right[0].0, &right[1].0, &right[2].0],
        ))
    }

    #[inline]
    pub(crate) fn dot4(left: [&Self; 4], right: [&Self; 4]) -> Self {
        // See `dot3`; matrix and complex kernels hit this path heavily.
        // Keep sparse rows sparse with early product elimination so exact
        // symbolic constructors do not absorb zero operands during reduction.
        let left0_zero = left[0].definitely_zero() || right[0].definitely_zero();
        let left1_zero = left[1].definitely_zero() || right[1].definitely_zero();
        let left2_zero = left[2].definitely_zero() || right[2].definitely_zero();
        let left3_zero = left[3].definitely_zero() || right[3].definitely_zero();
        let nonzero_lanes = usize::from(!left0_zero)
            + usize::from(!left1_zero)
            + usize::from(!left2_zero)
            + usize::from(!left3_zero);

        if nonzero_lanes == 0 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-all-zero");
            return Self::zero();
        }

        if nonzero_lanes == 1 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-single-term");
            return if !left0_zero {
                left[0] * right[0]
            } else if !left1_zero {
                left[1] * right[1]
            } else if !left2_zero {
                left[2] * right[2]
            } else {
                left[3] * right[3]
            };
        }

        if nonzero_lanes == 2 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-sparse-two");
            if left0_zero {
                if left1_zero {
                    return Self::active_signed_product_sum2(
                        [true, true],
                        [[left[2], right[2]], [left[3], right[3]]],
                    );
                }
                if left2_zero {
                    return Self::active_signed_product_sum2(
                        [true, true],
                        [[left[1], right[1]], [left[3], right[3]]],
                    );
                }
                return Self::active_signed_product_sum2(
                    [true, true],
                    [[left[1], right[1]], [left[2], right[2]]],
                );
            }
            if left1_zero {
                return if left2_zero {
                    Self::active_signed_product_sum2(
                        [true, true],
                        [[left[0], right[0]], [left[3], right[3]]],
                    )
                } else {
                    Self::active_signed_product_sum2(
                        [true, true],
                        [[left[0], right[0]], [left[2], right[2]]],
                    )
                };
            }
            if left2_zero {
                return Self::active_signed_product_sum2(
                    [true, true],
                    [[left[0], right[0]], [left[3], right[3]]],
                );
            }
            return Self::active_signed_product_sum2(
                [true, true],
                [[left[0], right[0]], [left[1], right[1]]],
            );
        }

        if nonzero_lanes == 3 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-sparse-three");
            if left0_zero {
                return Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [left[1], right[1]],
                        [left[2], right[2]],
                        [left[3], right[3]],
                    ],
                );
            }
            if left1_zero {
                return Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [left[0], right[0]],
                        [left[2], right[2]],
                        [left[3], right[3]],
                    ],
                );
            }
            if left2_zero {
                return Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [left[0], right[0]],
                        [left[1], right[1]],
                        [left[3], right[3]],
                    ],
                );
            }
            return Self::active_signed_product_sum2(
                [true, true, true],
                [
                    [left[0], right[0]],
                    [left[1], right[1]],
                    [left[2], right[2]],
                ],
            );
        }

        if B::FUSE_SIGNED_PRODUCT_SUM {
            // The zero scan above already established four active lanes. Reuse
            // that fact for exact backends instead of asking the scalar layer to
            // rediscover it in a hot matrix lane.
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-active");
            return Self(B::Repr::active_dot4(
                [&left[0].0, &left[1].0, &left[2].0, &left[3].0],
                [&right[0].0, &right[1].0, &right[2].0, &right[3].0],
            ));
        }

        // Full 4-lane approximate case remains specialized so backends can
        // choose optimal reduction order.
        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-backend");
        Self(B::Repr::dot4(
            [&left[0].0, &left[1].0, &left[2].0, &left[3].0],
            [&right[0].0, &right[1].0, &right[2].0, &right[3].0],
        ))
    }

    #[inline]
    pub(crate) fn dot3_same(values: [&Self; 3]) -> Self {
        // Norm kernels compute dot(v, v). Query each lane's structural zero
        // fact once, then keep the same sparse-product pruning used by `dot3`.
        // This applies Gustavson's sparse product idea to fixed-size vectors
        // (Gustavson, "Two fast algorithms for sparse matrices: multiplication
        // and permuted transposition", ACM TOMS 4(3), 1978) while preserving
        // the backend dense dot hook so hyperreal can delay rational
        // canonicalization as in Boehm et al.'s exact-real model
        // (https://doi.org/10.1145/319838.319860).
        let zero0 = values[0].definitely_zero();
        let zero1 = values[1].definitely_zero();
        let zero2 = values[2].definitely_zero();
        let nonzero_lanes = usize::from(!zero0) + usize::from(!zero1) + usize::from(!zero2);

        if nonzero_lanes == 0 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-same-all-zero");
            return Self::zero();
        }

        if nonzero_lanes == 1 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-same-single-term");
            return if !zero0 {
                values[0] * values[0]
            } else if !zero1 {
                values[1] * values[1]
            } else {
                values[2] * values[2]
            };
        }

        if nonzero_lanes == 2 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-same-sparse");
            return if zero0 {
                // Self-dot sparse-two has the same normalization shape as a
                // two-lane dot. Keep both squares in one product-sum so exact
                // backends can share denominator/canonicalization work instead
                // of expanding two independent squares and an add. This follows
                // Bareiss-style delayed normalization (Math. Comp. 22(103),
                // 1968, <https://doi.org/10.2307/2004533>).
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[1], values[1]], [values[2], values[2]]],
                )
            } else if zero1 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[0], values[0]], [values[2], values[2]]],
                )
            } else {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[0], values[0]], [values[1], values[1]]],
                )
            };
        }

        if B::FUSE_SIGNED_PRODUCT_SUM {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-same-active");
            return Self(B::Repr::active_dot3(
                [&values[0].0, &values[1].0, &values[2].0],
                [&values[0].0, &values[1].0, &values[2].0],
            ));
        }

        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot3-same-backend");
        Self(B::Repr::dot3(
            [&values[0].0, &values[1].0, &values[2].0],
            [&values[0].0, &values[1].0, &values[2].0],
        ))
    }

    #[inline]
    pub(crate) fn dot4_same(values: [&Self; 4]) -> Self {
        // Same self-dot specialization as `dot3_same`, extended to 4 lanes for
        // homogeneous vectors. Keeping dense inputs on `B::Repr::dot4` avoids
        // the earlier normalize regression from forcing a generic shared-scale
        // path while still halving structural zero probes before sparse exits.
        let zero0 = values[0].definitely_zero();
        let zero1 = values[1].definitely_zero();
        let zero2 = values[2].definitely_zero();
        let zero3 = values[3].definitely_zero();
        let nonzero_lanes =
            usize::from(!zero0) + usize::from(!zero1) + usize::from(!zero2) + usize::from(!zero3);

        if nonzero_lanes == 0 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-same-all-zero");
            return Self::zero();
        }

        if nonzero_lanes == 1 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-same-single-term");
            return if !zero0 {
                values[0] * values[0]
            } else if !zero1 {
                values[1] * values[1]
            } else if !zero2 {
                values[2] * values[2]
            } else {
                values[3] * values[3]
            };
        }

        if nonzero_lanes == 2 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-same-sparse-two");
            return if zero0 && zero1 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[2], values[2]], [values[3], values[3]]],
                )
            } else if zero0 && zero2 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[1], values[1]], [values[3], values[3]]],
                )
            } else if zero0 && zero3 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[1], values[1]], [values[2], values[2]]],
                )
            } else if zero1 && zero2 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[0], values[0]], [values[3], values[3]]],
                )
            } else if zero1 && zero3 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[0], values[0]], [values[2], values[2]]],
                )
            } else {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[values[0], values[0]], [values[1], values[1]]],
                )
            };
        }

        if nonzero_lanes == 3 {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-same-sparse-three");
            return if zero0 {
                Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [values[1], values[1]],
                        [values[2], values[2]],
                        [values[3], values[3]],
                    ],
                )
            } else if zero1 {
                Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [values[0], values[0]],
                        [values[2], values[2]],
                        [values[3], values[3]],
                    ],
                )
            } else if zero2 {
                Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [values[0], values[0]],
                        [values[1], values[1]],
                        [values[3], values[3]],
                    ],
                )
            } else {
                Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [values[0], values[0]],
                        [values[1], values[1]],
                        [values[2], values[2]],
                    ],
                )
            };
        }

        if B::FUSE_SIGNED_PRODUCT_SUM {
            crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-same-active");
            return Self(B::Repr::active_dot4(
                [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
                [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
            ));
        }

        crate::trace_dispatch!("hyperlattice", "scalar_fast_path", "dot4-same-backend");
        Self(B::Repr::dot4(
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
        ))
    }

    #[inline]
    pub(crate) fn linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self {
        // Row-local sparse pruning is intentionally retained here instead of
        // hoisting cached RHS zero facts in transform kernels. A cached-RHS
        // prototype reduced repeated structural queries but regressed targeted
        // mat3/mat4 hyperreal transform rows by roughly 2-8%; the per-row
        // checks preserve the inlined Gustavson-style sparse dot-product shape
        // that hyperlattice depends on for small fixed matrices. See
        // Gustavson, "Two fast algorithms for sparse matrices: multiplication
        // and permuted transposition", ACM TOMS 4(3), 1978.
        let zero0 = coefficients[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coefficients[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coefficients[2].definitely_zero() || values[2].definitely_zero();
        let nonzero_lanes = usize::from(!zero0) + usize::from(!zero1) + usize::from(!zero2);

        if nonzero_lanes == 0 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination3-all-zero"
            );
            return Self::zero();
        }

        if nonzero_lanes == 1 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination3-single-term"
            );
            return if !zero0 {
                coefficients[0] * values[0]
            } else if !zero1 {
                coefficients[1] * values[1]
            } else {
                coefficients[2] * values[2]
            };
        }

        if nonzero_lanes == 2 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination3-sparse"
            );
            return if zero0 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[coefficients[1], values[1]], [coefficients[2], values[2]]],
                )
            } else if zero1 {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[coefficients[0], values[0]], [coefficients[2], values[2]]],
                )
            } else {
                Self::active_signed_product_sum2(
                    [true, true],
                    [[coefficients[0], values[0]], [coefficients[1], values[1]]],
                )
            };
        }

        if B::FUSE_SIGNED_PRODUCT_SUM {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination3-active"
            );
            return Self(B::Repr::active_linear_combination3(
                [&coefficients[0].0, &coefficients[1].0, &coefficients[2].0],
                [&values[0].0, &values[1].0, &values[2].0],
            ));
        }

        crate::trace_dispatch!(
            "hyperlattice",
            "scalar_fast_path",
            "linear-combination3-specialized"
        );
        Self(B::Repr::linear_combination3(
            [&coefficients[0].0, &coefficients[1].0, &coefficients[2].0],
            [&values[0].0, &values[1].0, &values[2].0],
        ))
    }

    #[inline]
    pub(crate) fn active_linear_combination3(coefficients: [&Self; 3], values: [&Self; 3]) -> Self {
        crate::trace_dispatch!(
            "hyperlattice",
            "scalar_fast_path",
            "active-linear-combination3"
        );
        Self(B::Repr::active_linear_combination3(
            [&coefficients[0].0, &coefficients[1].0, &coefficients[2].0],
            [&values[0].0, &values[1].0, &values[2].0],
        ))
    }

    #[inline]
    pub(crate) fn linear_combination4(coefficients: [&Self; 4], values: [&Self; 4]) -> Self {
        // Dedicated linear-combination hooks let hyperreal keep shared affine
        // structure when transform kernels can preserve matrix row geometry.
        let zero0 = coefficients[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coefficients[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coefficients[2].definitely_zero() || values[2].definitely_zero();
        let zero3 = coefficients[3].definitely_zero() || values[3].definitely_zero();
        let nonzero_lanes =
            usize::from(!zero0) + usize::from(!zero1) + usize::from(!zero2) + usize::from(!zero3);

        if nonzero_lanes == 0 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination4-all-zero"
            );
            return Self::zero();
        }

        if nonzero_lanes == 1 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination4-single-term"
            );
            return if !zero0 {
                coefficients[0] * values[0]
            } else if !zero1 {
                coefficients[1] * values[1]
            } else if !zero2 {
                coefficients[2] * values[2]
            } else {
                coefficients[3] * values[3]
            };
        }

        if nonzero_lanes == 2 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination4-sparse-two"
            );
            if zero0 {
                if zero1 {
                    return Self::active_signed_product_sum2(
                        [true, true],
                        [[coefficients[2], values[2]], [coefficients[3], values[3]]],
                    );
                }
                if zero2 {
                    return Self::active_signed_product_sum2(
                        [true, true],
                        [[coefficients[1], values[1]], [coefficients[3], values[3]]],
                    );
                }
                return Self::active_signed_product_sum2(
                    [true, true],
                    [[coefficients[1], values[1]], [coefficients[2], values[2]]],
                );
            }
            if zero1 {
                if zero2 {
                    return Self::active_signed_product_sum2(
                        [true, true],
                        [[coefficients[0], values[0]], [coefficients[3], values[3]]],
                    );
                }
                return Self::active_signed_product_sum2(
                    [true, true],
                    [[coefficients[0], values[0]], [coefficients[2], values[2]]],
                );
            }
            if zero2 {
                return Self::active_signed_product_sum2(
                    [true, true],
                    [[coefficients[0], values[0]], [coefficients[3], values[3]]],
                );
            }
            // With `nonzero_lanes == 2` and zero0/zero1/zero2 all false, lane
            // 3 is the zero lane and the active pair is lanes 0 and 1. Keep the
            // sparse structural path explicit instead of falling back to the
            // dense reducer: it avoids constructing zero products in affine
            // matrix/vector kernels, the same sparse-product principle
            // described by Gustavson, ACM TOMS 4(3), 1978,
            // https://doi.org/10.1145/355791.355796.
            return Self::active_signed_product_sum2(
                [true, true],
                [[coefficients[0], values[0]], [coefficients[1], values[1]]],
            );
        }

        if nonzero_lanes == 3 {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination4-sparse-three"
            );
            if zero0 {
                return Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [coefficients[1], values[1]],
                        [coefficients[2], values[2]],
                        [coefficients[3], values[3]],
                    ],
                );
            }
            if zero1 {
                return Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [coefficients[0], values[0]],
                        [coefficients[2], values[2]],
                        [coefficients[3], values[3]],
                    ],
                );
            }
            if zero2 {
                return Self::active_signed_product_sum2(
                    [true, true, true],
                    [
                        [coefficients[0], values[0]],
                        [coefficients[1], values[1]],
                        [coefficients[3], values[3]],
                    ],
                );
            }
            return Self::active_signed_product_sum2(
                [true, true, true],
                [
                    [coefficients[0], values[0]],
                    [coefficients[1], values[1]],
                    [coefficients[2], values[2]],
                ],
            );
        }

        if B::FUSE_SIGNED_PRODUCT_SUM {
            crate::trace_dispatch!(
                "hyperlattice",
                "scalar_fast_path",
                "linear-combination4-active"
            );
            return Self(B::Repr::active_linear_combination4(
                [
                    &coefficients[0].0,
                    &coefficients[1].0,
                    &coefficients[2].0,
                    &coefficients[3].0,
                ],
                [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
            ));
        }

        crate::trace_dispatch!(
            "hyperlattice",
            "scalar_fast_path",
            "linear-combination4-specialized"
        );
        Self(B::Repr::linear_combination4(
            [
                &coefficients[0].0,
                &coefficients[1].0,
                &coefficients[2].0,
                &coefficients[3].0,
            ],
            [&values[0].0, &values[1].0, &values[2].0, &values[3].0],
        ))
    }

    #[inline]
    pub(crate) fn signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        // Fixed determinant and cofactor formulas are short signed sums of
        // products. Prune zero terms before handing dense cases to the backend
        // so exact implementations can delay normalization in the same spirit
        // as fraction-free elimination (Bareiss, Math. Comp. 22(103), 1968,
        // https://doi.org/10.2307/2004533), while sparse cases avoid building
        // a shared-denominator accumulator at all.
        let mut nonzero_count = 0usize;
        let mut first_term: Option<([&Self; 2], bool)> = None;
        let mut second_term: Option<([&Self; 2], bool)> = None;

        for i in 0..TERMS {
            if terms[i][0].definitely_zero() || terms[i][1].definitely_zero() {
                continue;
            }

            let term = (terms[i], positive_terms[i]);
            nonzero_count += 1;
            if nonzero_count == 1 {
                first_term = Some(term);
            } else if nonzero_count == 2 {
                second_term = Some(term);
            }
        }

        match nonzero_count {
            0 => {
                crate::trace_dispatch!(
                    "hyperlattice",
                    "scalar_fast_path",
                    "signed-product-sum2-all-zero"
                );
                return Self::zero();
            }
            1 => {
                let (term, positive) = first_term.expect("first term tracked for nonzero count");
                let product = term[0] * term[1];
                crate::trace_dispatch!(
                    "hyperlattice",
                    "scalar_fast_path",
                    "signed-product-sum2-single-term"
                );
                return if positive { product } else { -product };
            }
            2 => {
                let (left, left_positive) =
                    first_term.expect("first term tracked for nonzero count");
                let (right, right_positive) =
                    second_term.expect("second term tracked for nonzero count");
                if B::FUSE_SIGNED_PRODUCT_SUM {
                    // Keep two surviving terms on the backend reducer for
                    // exact backends. Matrix cofactors often prune one term
                    // out of a 3-term polynomial; routing the remaining pair
                    // through hyperreal preserves the shared-denominator path
                    // instead of immediately materializing two reduced
                    // products and an add/subtract. Compact approximate
                    // backends do not opt in, preserving their direct LLVM
                    // expression shape.
                    crate::trace_dispatch!(
                        "hyperlattice",
                        "scalar_fast_path",
                        "signed-product-sum2-sparse-two-fused"
                    );
                    return Self(B::Repr::active_signed_product_sum2(
                        [left_positive, right_positive],
                        [[&left[0].0, &left[1].0], [&right[0].0, &right[1].0]],
                    ));
                }
                let left_product = left[0] * left[1];
                let right_product = right[0] * right[1];
                crate::trace_dispatch!(
                    "hyperlattice",
                    "scalar_fast_path",
                    "signed-product-sum2-sparse-two"
                );
                return match (left_positive, right_positive) {
                    (true, true) => left_product + right_product,
                    (true, false) => left_product - right_product,
                    (false, true) => -left_product + right_product,
                    (false, false) => -(left_product + right_product),
                };
            }
            _ => {}
        }

        Self(B::Repr::signed_product_sum2(
            positive_terms,
            terms.map(|term| [&term[0].0, &term[1].0]),
        ))
    }

    #[inline]
    pub(crate) fn active_signed_product_sum2<const TERMS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; 2]; TERMS],
    ) -> Self {
        crate::trace_dispatch!(
            "hyperlattice",
            "scalar_fast_path",
            "active-signed-product-sum2"
        );
        Self(B::Repr::active_signed_product_sum2(
            positive_terms,
            terms.map(|term| [&term[0].0, &term[1].0]),
        ))
    }

    /// Raises this scalar to a scalar exponent.
    pub fn pow(self, exponent: Self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "pow");
        self.0.pow(exponent.0).map(Self)
    }

    /// Returns `e` raised to this scalar.
    pub fn exp(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "exp");
        self.0.exp().map(Self)
    }

    /// Returns the natural logarithm.
    pub fn ln(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "ln");
        self.0.ln().map(Self)
    }

    /// Returns the base-10 logarithm.
    pub fn log10(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "log10");
        self.0.log10().map(Self)
    }

    /// Returns the principal square root.
    pub fn sqrt(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "sqrt");
        self.0.sqrt().map(Self)
    }

    /// Returns the sine.
    pub fn sin(self) -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "sin");
        Self(self.0.sin())
    }

    /// Returns the cosine.
    pub fn cos(self) -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "cos");
        Self(self.0.cos())
    }

    /// Returns the tangent.
    pub fn tan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "tan");
        self.0.tan().map(Self)
    }

    /// Returns the inverse sine.
    pub fn asin(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "asin");
        self.0.asin().map(Self)
    }

    /// Returns the inverse cosine.
    pub fn acos(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "acos");
        self.0.acos().map(Self)
    }

    /// Returns the inverse tangent.
    pub fn atan(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "atan");
        self.0.atan().map(Self)
    }

    /// Returns the inverse hyperbolic sine.
    pub fn asinh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "asinh");
        self.0.asinh().map(Self)
    }

    /// Returns the inverse hyperbolic cosine.
    pub fn acosh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "acosh");
        self.0.acosh().map(Self)
    }

    /// Returns the inverse hyperbolic tangent.
    pub fn atanh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice", "scalar_method", "atanh");
        self.0.atanh().map(Self)
    }

    /// Returns whether this scalar is definitely zero.
    ///
    /// This is an optimistic predicate. Use [`Scalar::zero_status`] or
    /// [`zero_status`] when unknown-zero conditions must be distinguished.
    #[inline(always)]
    pub fn definitely_zero(&self) -> bool {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "definitely-zero");
        self.0.definitely_zero()
    }

    /// Classifies this scalar as zero, non-zero, or unknown.
    #[inline(always)]
    pub fn zero_status(&self) -> ZeroStatus {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "zero-status");
        self.0.zero_status()
    }

    /// Returns whether this scalar is definitely one.
    #[inline(always)]
    pub fn definitely_one(&self) -> bool {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "definitely-one");
        self.0.definitely_one()
    }

    /// Classifies this scalar as definitely one/zero when provable.
    ///
    /// Returns `Some(false)` for zero, `Some(true)` for one, and `None` when
    /// neither can be proven without approximation.
    #[inline(always)]
    pub fn zero_or_one(&self) -> Option<bool> {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "zero-or-one");
        self.0.zero_or_one()
    }

    /// Returns conservative structural facts exposed by this scalar's backend.
    #[inline(always)]
    pub fn structural_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "structural-facts");
        self.0.structural_facts()
    }

    /// Tries to prove this scalar's sign without refining beyond `min_precision`.
    ///
    /// Backends without refinement support return only signs already known from
    /// structural facts.
    #[inline(always)]
    pub fn refine_sign_until(&self, min_precision: i32) -> Option<ScalarSign> {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "refine-sign-until");
        self.0.refine_sign_until(min_precision)
    }

    /// Returns a borrowed finite `f64` approximation when one is available.
    #[inline(always)]
    pub fn to_f64_approx(&self) -> Option<f64> {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "to-f64-approx");
        self.0.to_f64_approx()
    }

    /// Attaches a cancellation signal to this scalar where the backend supports it.
    ///
    /// This affects hyperreal backend evaluation. It is a no-op on the approx
    /// backend.
    pub fn abort(&mut self, signal: AbortSignal) {
        crate::trace_dispatch!("hyperlattice", "scalar_query", "attach-abort");
        self.0.abort(signal);
    }
}

#[cfg(feature = "hyperreal-backend")]
impl Scalar<HyperrealBackend> {
    /// Constructs a scalar from a hyperreal rational value.
    pub fn new(rational: Rational) -> Self {
        rational.into()
    }
}

#[cfg(feature = "approx-backend")]
impl Scalar<ApproxBackend> {
    /// Constructs an approximate scalar from a center value and error bound.
    ///
    /// `epsilon` must be finite and non-negative and is stored as the scalar's
    /// absolute error bound.
    pub fn approx(value: f64, epsilon: f64) -> BlasResult<Self> {
        crate::backend::ApproxScalarRepr::new(value, epsilon).map(Self)
    }
}

#[cfg(feature = "hyperreal-backend")]
impl Scalar<HyperrealBackend> {
    /// Constructs a hyperreal scalar from an approximate center value.
    ///
    /// The `epsilon` argument is accepted for API compatibility and ignored
    /// because `Real` values do not store an interval error bound.
    pub fn approx(value: f64, epsilon: f64) -> BlasResult<Self> {
        let _ = epsilon;
        Self::try_from(value)
    }
}

impl<B: Backend> fmt::Display for Scalar<B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(feature = "hyperreal-backend")]
impl From<Real> for Scalar<HyperrealBackend> {
    fn from(value: Real) -> Self {
        Self(value.into())
    }
}

#[cfg(feature = "hyperreal-backend")]
impl From<Rational> for Scalar<HyperrealBackend> {
    fn from(value: Rational) -> Self {
        Self(value.into())
    }
}

macro_rules! impl_integer_conversion {
    ($($ty:ty),* $(,)?) => {
        $(
            impl<B: Backend> From<$ty> for Scalar<B> {
                fn from(value: $ty) -> Self {
                    Self(value.into())
                }
            }
        )*
    };
}

impl_integer_conversion!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl<B: Backend> TryFrom<f32> for Scalar<B> {
    type Error = Problem;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "try-from-f32");
        B::Repr::try_from(value).map(Self)
    }
}

impl<B: Backend> TryFrom<f64> for Scalar<B> {
    type Error = Problem;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        crate::trace_dispatch!("hyperlattice", "scalar_constructor", "try-from-f64");
        B::Repr::try_from(value).map(Self)
    }
}

impl<B: Backend> From<Scalar<B>> for f64 {
    fn from(value: Scalar<B>) -> Self {
        crate::trace_dispatch!("hyperlattice", "scalar_conversion", "into-f64");
        value.0.into_f64()
    }
}

impl<B: Backend> Add for Scalar<B> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "add-owned-owned");
        Self(self.0 + rhs.0)
    }
}

impl<B: Backend> Sub for Scalar<B> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "sub-owned-owned");
        Self(self.0 - rhs.0)
    }
}

impl<B: Backend> Neg for Scalar<B> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "neg-owned");
        Self(-self.0)
    }
}

impl<B: Backend> Mul for Scalar<B> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "mul-owned-owned");
        Self(self.0 * rhs.0)
    }
}

impl<B: Backend> Div for Scalar<B> {
    type Output = BlasResult<Self>;

    #[inline]
    fn div(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "div-owned-owned");
        self.0.div(rhs.0).map(Self)
    }
}

impl<B: Backend> Add<&Scalar<B>> for Scalar<B> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "add-owned-ref");
        Self(B::Repr::add_owned_ref(self.0, &rhs.0))
    }
}

impl<B: Backend> Add<Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn add(self, rhs: Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "add-ref-owned");
        Scalar(B::Repr::add_ref_owned(&self.0, rhs.0))
    }
}

impl<B: Backend> Add<&Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn add(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "add-ref-ref");
        Scalar(B::Repr::add_refs(&self.0, &rhs.0))
    }
}

impl<B: Backend> Sub<&Scalar<B>> for Scalar<B> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "sub-owned-ref");
        Self(B::Repr::sub_owned_ref(self.0, &rhs.0))
    }
}

impl<B: Backend> Sub<Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn sub(self, rhs: Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "sub-ref-owned");
        Scalar(B::Repr::sub_ref_owned(&self.0, rhs.0))
    }
}

impl<B: Backend> Sub<&Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn sub(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "sub-ref-ref");
        Scalar(B::Repr::sub_refs(&self.0, &rhs.0))
    }
}

impl<B: Backend> Mul<&Scalar<B>> for Scalar<B> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "mul-owned-ref");
        Self(B::Repr::mul_owned_ref(self.0, &rhs.0))
    }
}

impl<B: Backend> Mul<Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn mul(self, rhs: Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "mul-ref-owned");
        Scalar(B::Repr::mul_ref_owned(&self.0, rhs.0))
    }
}

impl<B: Backend> Mul<&Scalar<B>> for &Scalar<B> {
    type Output = Scalar<B>;

    #[inline]
    fn mul(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "mul-ref-ref");
        Scalar(B::Repr::mul_refs(&self.0, &rhs.0))
    }
}

impl<B: Backend> Div<&Scalar<B>> for Scalar<B> {
    type Output = BlasResult<Self>;

    #[inline]
    fn div(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "div-owned-ref");
        B::Repr::div_owned_ref(self.0, &rhs.0).map(Self)
    }
}

impl<B: Backend> Div<Scalar<B>> for &Scalar<B> {
    type Output = BlasResult<Scalar<B>>;

    #[inline]
    fn div(self, rhs: Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "div-ref-owned");
        B::Repr::div_ref_owned(&self.0, rhs.0).map(Scalar)
    }
}

impl<B: Backend> Div<&Scalar<B>> for &Scalar<B> {
    type Output = BlasResult<Scalar<B>>;

    #[inline]
    fn div(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice", "scalar_op", "div-ref-ref");
        B::Repr::div_refs(&self.0, &rhs.0).map(Scalar)
    }
}

mod complex;
mod matrix;
mod scalar;
mod vector;

#[cfg(feature = "arbitrary")]
mod arbitrary_impls;

pub use complex::Complex;
pub use matrix::{Matrix3, Matrix4, PreparedRightDivisor3, PreparedRightDivisor4};
pub use scalar::*;
pub use vector::{Vector3, Vector4};
