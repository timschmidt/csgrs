//! Complex numbers backed by [`Scalar`](crate::Scalar).

use std::fmt;
use std::ops::{Add, BitXor, Div, Mul, Neg, Sub};

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
        crate::trace_dispatch!("realistic_blas_complex", "constructor", "new");
        Self { re, im }
    }

    /// Returns `0 + 0i`.
    pub fn zero() -> Self {
        crate::trace_dispatch!("realistic_blas_complex", "constructor", "zero");
        Self::new(Scalar::zero(), Scalar::zero())
    }

    /// Returns `1 + 0i`.
    pub fn one() -> Self {
        crate::trace_dispatch!("realistic_blas_complex", "constructor", "one");
        Self::new(Scalar::one(), Scalar::zero())
    }

    /// Returns the imaginary unit `0 + 1i`.
    pub fn i() -> Self {
        crate::trace_dispatch!("realistic_blas_complex", "constructor", "i");
        Self::new(Scalar::zero(), Scalar::one())
    }

    /// Returns the complex conjugate.
    pub fn conjugate(self) -> Self {
        crate::trace_dispatch!("realistic_blas_complex", "method", "conjugate");
        Self::new(self.re, -self.im)
    }

    /// Returns `re^2 + im^2`.
    pub fn norm_squared(&self) -> Scalar<B> {
        crate::trace_dispatch!("realistic_blas_complex", "method", "norm-squared");
        &self.re * &self.re + &self.im * &self.im
    }

    /// Returns the multiplicative inverse.
    pub fn reciprocal(self) -> BlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_complex", "method", "reciprocal");
        let inv_denom = self.norm_squared().inverse()?;
        // Apply the shared denominator by borrowed cached multiplication; cloning it for
        // both real and imaginary components is visible with symbolic scalar backends.
        Ok(Self::new(
            self.re.mul_cached(&inv_denom),
            (-self.im).mul_cached(&inv_denom),
        ))
    }

    /// Returns the multiplicative inverse after rejecting unknown-zero norms.
    pub fn reciprocal_checked(self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_complex", "method", "reciprocal-checked");
        let denom = self.norm_squared();
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
        crate::trace_dispatch!("realistic_blas_complex", "method", "powi");
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
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
        crate::trace_dispatch!("realistic_blas_complex", "method", "powi-checked");
        if exponent == 0 {
            if self.re.definitely_zero() && self.im.definitely_zero() {
                return Err(Problem::NotANumber);
            }
            return Ok(Self::one());
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
        crate::trace_dispatch!("realistic_blas_complex", "method", "div-checked");
        let denom = rhs.norm_squared();
        require_known_nonzero(&denom)?;
        let inv_denom = denom.inverse()?;
        let re = &self.re * &rhs.re + &self.im * &rhs.im;
        let im = &self.im * &rhs.re - &self.re * &rhs.im;
        // The inverse norm is reused for both components, so keep it borrowed.
        Ok(Self::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }

    /// Divides by a real scalar after rejecting unknown-zero divisors.
    pub fn div_real_checked(self, rhs: Scalar<B>) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("realistic_blas_complex", "method", "div-real-checked");
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
            crate::trace_dispatch!("realistic_blas_complex", "powi", "exponent-one");
            base
        }
        2 => {
            crate::trace_dispatch!("realistic_blas_complex", "powi", "specialized-square");
            base.clone() * base
        }
        3 => {
            crate::trace_dispatch!("realistic_blas_complex", "powi", "specialized-cube");
            let square = base.clone() * base.clone();
            square * base
        }
        4 => {
            crate::trace_dispatch!("realistic_blas_complex", "powi", "specialized-fourth");
            let square = base.clone() * base;
            square.clone() * square
        }
        5 => {
            crate::trace_dispatch!("realistic_blas_complex", "powi", "specialized-fifth");
            let square = base.clone() * base.clone();
            let fourth = square.clone() * square;
            fourth * base
        }
        _ => {
            crate::trace_dispatch!("realistic_blas_complex", "powi", "generic-squaring");
            complex_powi_by_squaring(base, exponent)
        }
    }
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
        crate::trace_dispatch!("realistic_blas_complex", "constructor", "from-scalar");
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
        crate::trace_dispatch!("realistic_blas_complex", "op", "add-owned-owned");
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl<B: Backend> Add<&Complex<B>> for Complex<B> {
    type Output = Self;

    fn add(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "add-owned-ref");
        Self::new(self.re.add_cached(&rhs.re), self.im.add_cached(&rhs.im))
    }
}

impl<B: Backend> Add<Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn add(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "add-ref-owned");
        Complex::new(&self.re + rhs.re, &self.im + rhs.im)
    }
}

impl<B: Backend> Add<&Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn add(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "add-ref-ref");
        Complex::new(&self.re + &rhs.re, &self.im + &rhs.im)
    }
}

impl<B: Backend> Sub for Complex<B> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "sub-owned-owned");
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl<B: Backend> Sub<&Complex<B>> for Complex<B> {
    type Output = Self;

    fn sub(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "sub-owned-ref");
        Self::new(self.re.sub_cached(&rhs.re), self.im.sub_cached(&rhs.im))
    }
}

impl<B: Backend> Sub<Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn sub(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "sub-ref-owned");
        Complex::new(&self.re - rhs.re, &self.im - rhs.im)
    }
}

impl<B: Backend> Sub<&Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn sub(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "sub-ref-ref");
        Complex::new(&self.re - &rhs.re, &self.im - &rhs.im)
    }
}

impl<B: Backend> Neg for Complex<B> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "neg-owned");
        Self::new(-self.re, -self.im)
    }
}

impl<B: Backend> Neg for &Complex<B> {
    type Output = Complex<B>;

    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "neg-ref");
        Complex::new(-self.re.clone(), -self.im.clone())
    }
}

impl<B: Backend> Mul for Complex<B> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "mul-owned-owned");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Self::new(re, im)
    }
}

impl<B: Backend> Mul<&Complex<B>> for Complex<B> {
    type Output = Self;

    fn mul(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "mul-owned-ref");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Self::new(re, im)
    }
}

impl<B: Backend> Mul<Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn mul(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "mul-ref-owned");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Complex::new(re, im)
    }
}

impl<B: Backend> Mul<&Complex<B>> for &Complex<B> {
    type Output = Complex<B>;

    fn mul(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "mul-ref-ref");
        let re = &self.re * &rhs.re - &self.im * &rhs.im;
        let im = &self.re * &rhs.im + &self.im * &rhs.re;
        Complex::new(re, im)
    }
}

impl<B: Backend> Div for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "div-owned-owned");
        let inv_denom = rhs.norm_squared().inverse()?;
        let re = &self.re * &rhs.re + &self.im * &rhs.im;
        let im = &self.im * &rhs.re - &self.re * &rhs.im;
        Ok(Self::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<&Complex<B>> for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "div-owned-ref");
        let inv_denom = rhs.norm_squared().inverse()?;
        let re = &self.re * &rhs.re + &self.im * &rhs.im;
        let im = &self.im * &rhs.re - &self.re * &rhs.im;
        Ok(Self::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<Complex<B>> for &Complex<B> {
    type Output = BlasResult<Complex<B>>;

    fn div(self, rhs: Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "div-ref-owned");
        let inv_denom = rhs.norm_squared().inverse()?;
        let re = &self.re * &rhs.re + &self.im * &rhs.im;
        let im = &self.im * &rhs.re - &self.re * &rhs.im;
        Ok(Complex::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<&Complex<B>> for &Complex<B> {
    type Output = BlasResult<Complex<B>>;

    fn div(self, rhs: &Complex<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "div-ref-ref");
        let inv_denom = rhs.norm_squared().inverse()?;
        let re = &self.re * &rhs.re + &self.im * &rhs.im;
        let im = &self.im * &rhs.re - &self.re * &rhs.im;
        Ok(Complex::new(
            re.mul_cached(&inv_denom),
            im.mul_cached(&inv_denom),
        ))
    }
}

impl<B: Backend> Div<Scalar<B>> for Complex<B> {
    type Output = BlasResult<Self>;

    fn div(self, rhs: Scalar<B>) -> Self::Output {
        crate::trace_dispatch!("realistic_blas_complex", "op", "div-real-owned");
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
        crate::trace_dispatch!("realistic_blas_complex", "op", "div-real-ref");
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
        crate::trace_dispatch!("realistic_blas_complex", "op", "bitxor-powi");
        self.powi(rhs)
    }
}
