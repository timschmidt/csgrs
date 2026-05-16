//! Complex numbers backed by [`Scalar`](crate::Scalar).

use std::fmt;
use std::ops::{Add, BitXor, Div, Mul, Neg, Sub};

use crate::backend::ExactRationalKind;
use crate::scalar::require_known_nonzero;
use crate::{Backend, BlasResult, CheckedBlasResult, DefaultBackend, Problem, Scalar};

/// Complex scalar with real and imaginary components.
#[derive(Clone, Debug, PartialEq)]
pub struct Complex<B: Backend = DefaultBackend> {
    /// Real component.
    pub re: Scalar<B>,
    /// Imaginary component.
    pub im: Scalar<B>,
}

impl<B: Backend> Complex<B> {
    /// Constructs a complex value from real and imaginary components.
    pub fn new(re: Scalar<B>, im: Scalar<B>) -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "new");
        Self { re, im }
    }

    /// Returns `0 + 0i`.
    pub fn zero() -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "zero");
        Self::new(Scalar::zero(), Scalar::zero())
    }

    /// Returns `1 + 0i`.
    pub fn one() -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "one");
        Self::new(Scalar::one(), Scalar::zero())
    }

    /// Returns the imaginary unit `0 + 1i`.
    pub fn i() -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "i");
        Self::new(Scalar::zero(), Scalar::one())
    }

    /// Returns the complex conjugate.
    pub fn conjugate(self) -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "method", "conjugate");
        Self::new(self.re, -self.im)
    }

    /// Returns `re^2 + im^2`.
    pub fn norm_squared(&self) -> Scalar<B> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "norm-squared");
        if B::FUSE_SIGNED_PRODUCT_SUM {
            // Isolated exact complex norms are two positive product terms. Fuse
            // only this public norm query so exact-rational hyperreal backends
            // can share one denominator and reduce once, following the same
            // fraction-delaying principle as Bareiss fraction-free elimination
            // (Bareiss, Math. Comp. 22(103), 1968,
            // https://doi.org/10.2307/2004533). Reciprocal/division deliberately
            // use `norm_squared_direct`; targeted Criterion showed their larger
            // arithmetic kernels regress when this fused expression is inlined
            // into denominator construction.
            Scalar::signed_product_sum2([true, true], [[&self.re, &self.re], [&self.im, &self.im]])
        } else {
            &self.re * &self.re + &self.im * &self.im
        }
    }

    /// Returns the multiplicative inverse.
    pub fn reciprocal(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "reciprocal");
        let inv_denom = (&self.re * &self.re + &self.im * &self.im).inverse()?;
        // Apply the shared denominator by borrowed cached multiplication; cloning it for
        // both real and imaginary components is visible with symbolic scalar backends.
        Ok(Self::new(
            self.re.mul_cached(&inv_denom),
            (-self.im).mul_cached(&inv_denom),
        ))
    }

    /// Returns the multiplicative inverse after rejecting unknown-zero norms.
    pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "reciprocal-checked");
        let denom = &self.re * &self.re + &self.im * &self.im;
        require_known_nonzero(&denom)?;
        let inv_denom = denom.inverse()?;
        // Same cached-denominator path as `reciprocal`, after the checked zero gate.
        Ok(Self::new(
            self.re.mul_cached(&inv_denom),
            (-self.im).mul_cached(&inv_denom),
        ))
    }

    /// Raises this complex value to an integer exponent.
    ///
    /// Negative exponents require the result to be invertible. `0^0` returns
    /// [`Problem::NotANumber`].
    pub fn powi(self, exponent: i64) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "powi");
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
        }
        if exponent == -1 {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "negative-one-reciprocal");
            return self.reciprocal();
        }

        let result = complex_powi_positive(self, exponent.unsigned_abs());
        if exponent < 0 {
            result.reciprocal()
        } else {
            Ok(result)
        }
    }

    /// Raises this complex value to an integer exponent with checked inversion.
    pub fn powi_checked(self, exponent: i64) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "powi-checked");
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
        }
        if exponent == -1 {
            crate::trace_dispatch!(
                "hyperlattice_complex",
                "powi",
                "negative-one-reciprocal-checked"
            );
            return self.reciprocal_checked();
        }

        let result = complex_powi_positive(self, exponent.unsigned_abs());
        if exponent < 0 {
            result.reciprocal_checked()
        } else {
            Ok(result)
        }
    }

    /// Divides by another complex value after rejecting unknown-zero norms.
    pub fn div_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "div-checked");
        let denom = &rhs.re * &rhs.re + &rhs.im * &rhs.im;
        require_known_nonzero(&denom)?;
        let inv_denom = denom.inverse()?;
        let (re, im) = complex_division_numerators(&self, &rhs);
        // The inverse norm is reused for both components, so keep it borrowed.
        Ok(Self::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }

    /// Divides by a real scalar after rejecting unknown-zero divisors.
    pub fn div_real_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "div-real-checked");
        require_known_nonzero(&rhs)?;
        let inv_rhs = rhs.inverse()?;
        // Real scalar division has a single inverse shared by both components.
        Ok(Self::new(
            self.re.mul_cached(&inv_rhs),
            self.im.mul_cached(&inv_rhs),
        ))
    }
}

fn complex_powi_positive<B: Backend>(base: Complex<B>, exponent: u64) -> Complex<B> {
    // Small powers are common in BLAS-style polynomial kernels.  Writing them out avoids
    // loop bookkeeping and keeps clone placement tuned for the borrowed scalar backend.
    match exponent {
        1 => {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "exponent-one");
            base
        }
        2 => {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "specialized-square");
            complex_multiply_for_powi(base.clone(), base)
        }
        3 => {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "specialized-cube");
            let square = complex_multiply_for_powi(base.clone(), base.clone());
            complex_multiply_for_powi(square, base)
        }
        4 => {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "specialized-fourth");
            let square = complex_multiply_for_powi(base.clone(), base);
            complex_multiply_for_powi(square.clone(), square)
        }
        5 => {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "specialized-fifth");
            let square = complex_multiply_for_powi(base.clone(), base.clone());
            let fourth = complex_multiply_for_powi(square.clone(), square);
            complex_multiply_for_powi(fourth, base)
        }
        _ => {
            crate::trace_dispatch!("hyperlattice_complex", "powi", "generic-squaring");
            complex_powi_by_squaring(base, exponent)
        }
    }
}

#[inline(always)]
fn complex_multiply_for_powi<B: Backend>(lhs: Complex<B>, rhs: Complex<B>) -> Complex<B> {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!("hyperlattice_complex", "powi", "mul-fused-exact");
        let (re, im) = complex_multiply_components(&lhs, &rhs);
        Complex::new(re, im)
    } else {
        crate::trace_dispatch!("hyperlattice_complex", "powi", "mul-direct");
        lhs * rhs
    }
}

#[inline(always)]
fn complex_division_numerators<B: Backend>(
    lhs: &Complex<B>,
    rhs: &Complex<B>,
) -> (Scalar<B>, Scalar<B>) {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        let known_exact_rational = lhs.re.exact_rational_kind() != ExactRationalKind::NonRational
            && lhs.im.exact_rational_kind() != ExactRationalKind::NonRational
            && rhs.re.exact_rational_kind() != ExactRationalKind::NonRational
            && rhs.im.exact_rational_kind() != ExactRationalKind::NonRational;
        if known_exact_rational {
            crate::trace_dispatch!(
                "hyperlattice_complex",
                "op",
                "div-numerators-fused-known-exact-rational"
            );
            return (
                Scalar::active_signed_product_sum2_known_exact_rational(
                    [true, true],
                    [[&lhs.re, &rhs.re], [&lhs.im, &rhs.im]],
                ),
                Scalar::active_signed_product_sum2_known_exact_rational(
                    [true, false],
                    [[&lhs.im, &rhs.re], [&lhs.re, &rhs.im]],
                ),
            );
        }
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-numerators-fused-exact");
        return (
            Scalar::active_signed_product_sum2(
                [true, true],
                [[&lhs.re, &rhs.re], [&lhs.im, &rhs.im]],
            ),
            Scalar::active_signed_product_sum2(
                [true, false],
                [[&lhs.im, &rhs.re], [&lhs.re, &rhs.im]],
            ),
        );
    }

    crate::trace_dispatch!("hyperlattice_complex", "op", "div-numerators-direct");
    (
        &lhs.re * &rhs.re + &lhs.im * &rhs.im,
        &lhs.im * &rhs.re - &lhs.re * &rhs.im,
    )
}

#[inline]
fn complex_multiply_components<B: Backend>(
    lhs: &Complex<B>,
    rhs: &Complex<B>,
) -> (Scalar<B>, Scalar<B>) {
    if B::FUSE_SIGNED_PRODUCT_SUM {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-components-fused-exact");
        return (
            Scalar::active_signed_product_sum2(
                [true, false],
                [[&lhs.re, &rhs.re], [&lhs.im, &rhs.im]],
            ),
            Scalar::active_signed_product_sum2(
                [true, true],
                [[&lhs.re, &rhs.im], [&lhs.im, &rhs.re]],
            ),
        );
    }

    crate::trace_dispatch!("hyperlattice_complex", "op", "mul-components-direct");
    (
        &lhs.re * &rhs.re - &lhs.im * &rhs.im,
        &lhs.re * &rhs.im + &lhs.im * &rhs.re,
    )
}

fn complex_powi_by_squaring<B: Backend>(base: Complex<B>, exponent: u64) -> Complex<B> {
    let mut exp = exponent;
    let mut result = None;
    let mut factor = base;
    while exp > 0 {
        if exp & 1 == 1 {
            result = Some(match result {
                Some(result) => result * factor.clone(),
                None => factor.clone(),
            });
        }
        exp >>= 1;
        if exp > 0 {
            factor = factor.clone() * factor;
        }
    }

    result.expect("non-zero exponent sets at least one result bit")
}

impl<B: Backend> From<Scalar<B>> for Complex<B> {
    fn from(value: Scalar<B>) -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "from-scalar");
        Self::new(value, Scalar::zero())
    }
}

impl<B: Backend> fmt::Display for Complex<B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            write!(f, "({:#} + {:#}i)", self.re, self.im)
        } else {
            write!(f, "({} + {}i)", self.re, self.im)
        }
    }
}

impl<B: Backend> Add for Complex<B> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-owned-owned");
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl<B: Backend> Add<&Complex<B>> for Complex<B> {
    type Output = Self;

    fn add(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-owned-ref");
        Self::new(self.re.add_cached(&rhs.re), self.im.add_cached(&rhs.im))
    }
}

impl<B: Backend> Add<Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn add(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-ref-owned");
        Complex::new(&self.re + rhs.re, &self.im + rhs.im)
    }
}

impl<B: Backend> Add<&Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn add(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-ref-ref");
        Complex::new(&self.re + &rhs.re, &self.im + &rhs.im)
    }
}

impl<B: Backend> Sub for Complex<B> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-owned-owned");
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl<B: Backend> Sub<&Complex<B>> for Complex<B> {
    type Output = Self;

    fn sub(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-owned-ref");
        Self::new(self.re.sub_cached(&rhs.re), self.im.sub_cached(&rhs.im))
    }
}

impl<B: Backend> Sub<Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn sub(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-ref-owned");
        Complex::new(&self.re - rhs.re, &self.im - rhs.im)
    }
}

impl<B: Backend> Sub<&Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn sub(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-ref-ref");
        Complex::new(&self.re - &rhs.re, &self.im - &rhs.im)
    }
}

impl<B: Backend> Neg for Complex<B> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "neg-owned");
        Self::new(-self.re, -self.im)
    }
}

impl<B: Backend> Neg for &Complex<B> {
    type Output = Complex<B>;

    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "neg-ref");
        Complex::new(-self.re.clone(), -self.im.clone())
    }
}

impl<B: Backend> Mul for Complex<B> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-owned-owned");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Self::new(re, im)
    }
}

impl<B: Backend> Mul<&Complex<B>> for Complex<B> {
    type Output = Self;

    fn mul(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-owned-ref");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Self::new(re, im)
    }
}

impl<B: Backend> Mul<Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn mul(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-ref-owned");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Complex::new(re, im)
    }
}

impl<B: Backend> Mul<&Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn mul(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-ref-ref");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Complex::new(re, im)
    }
}

impl<B: Backend> Div for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-owned-owned");
        let inv_denom = (&rhs.re * &rhs.re + &rhs.im * &rhs.im).inverse()?;
        let (re, im) = complex_division_numerators(&self, &rhs);
        Ok(Self::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<&Complex<B>> for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-owned-ref");
        let inv_denom = (&rhs.re * &rhs.re + &rhs.im * &rhs.im).inverse()?;
        let (re, im) = complex_division_numerators(&self, rhs);
        Ok(Self::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<Complex<B>> for &Complex<B> {
    type Output = BlasResult<Complex<B>>;

    fn div(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-ref-owned");
        let inv_denom = (&rhs.re * &rhs.re + &rhs.im * &rhs.im).inverse()?;
        let (re, im) = complex_division_numerators(self, &rhs);
        Ok(Complex::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<&Complex<B>> for &Complex<B> {
    type Output = BlasResult<Complex<B>>;

    fn div(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-ref-ref");
        let inv_denom = (&rhs.re * &rhs.re + &rhs.im * &rhs.im).inverse()?;
        let (re, im) = complex_division_numerators(self, rhs);
        Ok(Complex::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<Scalar<B>> for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-real-owned");
        let inv_rhs = rhs.inverse()?;
        Ok(Self::new(
            self.re.mul_cached(&inv_rhs),
            self.im.mul_cached(&inv_rhs),
        ))
    }
}

impl<B: Backend> Div<&Scalar<B>> for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: &Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-real-ref");
        let inv_rhs = rhs.inverse_ref()?;
        Ok(Self::new(
            self.re.mul_cached(&inv_rhs),
            self.im.mul_cached(&inv_rhs),
        ))
    }
}

impl<B: Backend> BitXor<i64> for Complex<B> {
    type Output = BlasResult<Self>;

    fn bitxor(self, rhs: i64) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "bitxor-powi");
        self.powi(rhs)
    }
}
