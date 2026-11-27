//! Complex scalar carriers and exact arithmetic schedules over Hyperreal entries.
//!
//! For z=a+bi, multiplication forms (ac-bd)+(ad+bc)i, and division by c+di
//! uses ((ac+bd)+(bc-ad)i)/(c^2+d^2). Hyperreal owns scalar construction,
//! zero knowledge, refinement, and native failures. Rational fast paths retain
//! exactness; storage reuse evidence only selects an arithmetic schedule.
//!
//! Complex equality is componentwise structural Real equality, not a general
//! proof that two expressions have the same value. Checked division/inversion
//! reject unavailable cheap nonzero knowledge; ordinary paths defer to scalar
//! inverse construction. Result aliases preserve Problem but add no predicate
//! policy, certainty receipt, total-work limit, or independent cancellation budget.
//!
//! Default Display uses symbolic component formatting; alternate Display is a
//! human-readable diagnostic projection and can contain nonnumeric error markers.
//! Neither form is a promised lossless machine interchange. Transcendental and
//! Jacobi continuations are implemented in the child modules.

use std::fmt;
use std::ops::{Add, BitXor, Div, Mul, Neg, Sub};

use crate::scalar::require_known_nonzero;
use crate::{
    BlasResult, CheckedBlasResult, ExactRationalKind, Problem, Real, RealKernelExt, ZeroKnowledge,
};

/// Principal complex elementary-function continuation implemented with native Real expressions and explicit cut handling.
mod elementary;
/// Complex Jacobi triplet and epsilon continuation, delegating real evaluations and parameter decisions to Hyperreal.
mod jacobi;

/// An exact expression pair re + i*im with independently editable Real components.
///
/// Construction does not normalize or refine the pair. Derived PartialEq compares
/// component structure, so unequal structures need not mean unequal mathematical
/// values. Clone retains native scalar sharing/cache semantics rather than promising
/// independent cold expressions. Methods return new carriers; no separate geometric
/// validity, predicate-policy, or inherited-certainty packet is stored.
#[derive(Clone, Debug, PartialEq)]
pub struct Complex {
    /// Real component; public replacement directly changes the represented expression pair.
    pub re: Real,
    /// Imaginary coefficient; public replacement directly changes the represented expression pair.
    pub im: Real,
}

impl Complex {
    /// Move two native Real expressions into a complex carrier without normalization, approximation, or domain checks.
    pub fn new(re: Real, im: Real) -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "new");
        Self { re, im }
    }

    /// Construct exact 0+0i through the native scalar zeros.
    pub fn zero() -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "zero");
        Self::new(Real::zero(), Real::zero())
    }

    /// Construct exact 1+0i, the complex multiplicative identity.
    pub fn one() -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "one");
        Self::new(Real::one(), Real::zero())
    }

    /// Construct exact 0+1i, the imaginary unit.
    pub fn i() -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "i");
        Self::new(Real::zero(), Real::one())
    }

    /// Consume a+bi and return a-bi, moving the real component and negating the imaginary coefficient.
    pub fn conjugate(self) -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "method", "conjugate");
        Self::new(self.re, -self.im)
    }

    /// Construct re^2+im^2 as a fused two-positive-product scalar expression.
    ///
    /// This is the squared modulus, not its square root. It does not refine a sign or
    /// prove invertibility here. Reciprocal/division deliberately use their own inline
    /// denominator schedule; this norm query does not select those paths.
    pub fn norm_squared(&self) -> Real {
        crate::trace_dispatch!("hyperlattice_complex", "method", "norm-squared");
        // Isolated exact complex norms are two positive product terms. Fuse
        // only this public norm query so exact-rational hyperreal Real kernels
        // can share one denominator and reduce once. Reciprocal/division deliberately
        // build their denominator inline; targeted Criterion showed their larger
        // arithmetic kernels regress when this fused expression is substituted.
        Real::signed_product_sum2([true, true], [[&self.re, &self.re], [&self.im, &self.im]])
    }

    /// Consume z and construct conjugate(z)/(re^2+im^2), propagating scalar inverse errors.
    ///
    /// Forms the denominator inline, computes one inverse, and borrows that shared factor
    /// for both components. No explicit structural nonzero guard is inserted here;
    /// Real::inverse owns zero/unknown/domain handling. No total-work bound is added.
    pub fn reciprocal(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "reciprocal");
        let inv_denom = (&self.re * &self.re + &self.im * &self.im).inverse()?;
        // Apply the shared denominator by borrowed cached multiplication; cloning it for
        // both real and imaginary components is visible with symbolic Real arithmetics.
        Ok(Self::new(
            self.re.mul_cached(&inv_denom),
            (-self.im).mul_cached(&inv_denom),
        ))
    }

    /// Consume z and invert only after the squared modulus is cheaply known nonzero.
    ///
    /// Zero knowledge maps to DivideByZero; unknown knowledge maps to UnknownZero without
    /// starting refinement in the guard. Accepted inputs then use one native inverse and
    /// borrow it for both components, preserving any scalar construction error.
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

    /// Raise this value to a signed integer power using exact multiplication and optional inversion.
    ///
    /// At exponent zero, cheap component facts must prove the base nonzero: known zero
    /// returns [`Problem::NotANumber`], unknown returns [`Problem::UnknownZero`], and
    /// known nonzero returns one. Positive exponents accept zero. Exponent -1 calls
    /// reciprocal directly; other negative exponents invert the constructed positive
    /// power. Powers 1..5 use fixed schedules, larger magnitudes use binary squaring.
    /// unsigned_abs handles i64::MIN without signed negation overflow. Logarithmically
    /// many multiplications do not bound coefficient/expression growth or total work.
    pub fn powi(self, exponent: i64) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "powi");
        if exponent == 0 {
            match complex_zero_status(&self) {
                ZeroKnowledge::Zero => return Err(Problem::NotANumber),
                ZeroKnowledge::Unknown => return Err(Problem::UnknownZero),
                ZeroKnowledge::NonZero => {}
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

    /// Use the same integer-power schedules with checked inversion for negative exponents.
    ///
    /// Exponent zero has the same known-zero/unknown rejection as [`Self::powi`]. Positive
    /// exponents add no nonzero test. For -1 check the base reciprocal; for other negative
    /// exponents check the resulting positive power's squared modulus before inversion.
    /// No up-front nonzero proof for every negative base or extra work bound is supplied.
    pub fn powi_checked(self, exponent: i64) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "powi-checked");
        if exponent == 0 {
            match complex_zero_status(&self) {
                ZeroKnowledge::Zero => return Err(Problem::NotANumber),
                ZeroKnowledge::Unknown => return Err(Problem::UnknownZero),
                ZeroKnowledge::NonZero => {}
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

    /// Divide self by rhs with exact-rational specialization or a checked squared-modulus guard.
    ///
    /// When all four components are exact rational, the scalar quotient kernel validates
    /// its exact denominator directly. Otherwise form rhs.re^2+rhs.im^2, reject Zero or
    /// Unknown cheaply, invert once, and scale both conjugate-product numerators. Native
    /// errors remain Problem values; no predicate certainty packet is added.
    pub fn div_checked(self, rhs: Self) -> CheckedBlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "div-checked");
        let (re, im) = complex_divide_components(&self, &rhs, true)?;
        Ok(Self::new(re, im))
    }

    /// Divide both components by one real scalar after a cheap nonzero guard.
    ///
    /// Reject Zero as DivideByZero and Unknown as UnknownZero, then compute one inverse
    /// and borrow it for both products. This differs from ordinary scalar division only
    /// in the explicit guard, not in the returned error/result type.
    pub fn div_real_checked(self, rhs: Real) -> CheckedBlasResult<Self> {
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

/// Combine component zero facts without requiring a general equality proof.
///
/// Inspect the real exact-rational view first: nonzero returns immediately, zero
/// asks only for the imaginary zero status. Otherwise inspect the real zero status;
/// known nonzero short-circuits, known zero delegates to imaginary, and unknown
/// becomes NonZero only if imaginary is known nonzero. Both known zero give Zero;
/// all remaining cases give Unknown. This order preserves scalar fact/cache behavior.
#[inline]
fn complex_zero_status(value: &Complex) -> ZeroKnowledge {
    if let Some(real) = value.re.exact_rational_ref() {
        if !real.is_zero() {
            return ZeroKnowledge::NonZero;
        }
        return value.im.zero_status();
    }
    match value.re.zero_status() {
        ZeroKnowledge::NonZero => ZeroKnowledge::NonZero,
        ZeroKnowledge::Zero => value.im.zero_status(),
        ZeroKnowledge::Unknown => match value.im.zero_status() {
            ZeroKnowledge::NonZero => ZeroKnowledge::NonZero,
            ZeroKnowledge::Zero | ZeroKnowledge::Unknown => ZeroKnowledge::Unknown,
        },
    }
}

/// Consume a base and compute a strictly positive unsigned power.
///
/// Use tuned clone/multiply schedules for exponents 1..5; delegate all others to
/// binary squaring. A zero exponent violates the private precondition and reaches
/// the generic helper's expect panic. Public entry points handle zero beforehand.
/// Clone placement and multiplication order affect reuse evidence and are intentional.
fn complex_powi_positive(base: Complex, exponent: u64) -> Complex {
    // Small powers are common in BLAS-style polynomial kernels.  Writing them out avoids
    // loop bookkeeping and keeps clone placement tuned for the borrowed Real arithmetic.
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

/// Consume two power intermediates, delegate component multiplication to the shared scheduler, and construct a new Complex; records the power-specific path.
#[inline(always)]
fn complex_multiply_for_powi(lhs: Complex, rhs: Complex) -> Complex {
    crate::trace_dispatch!("hyperlattice_complex", "powi", "mul-canonical-components");
    let (re, im) = complex_multiply_components(&lhs, &rhs);
    Complex::new(re, im)
}

/// Form the conjugate-product numerators (ac+bd, bc-ad) for (a+bi)/(c+di).
///
/// Use active two-term scalar reducers in real-then-imaginary order; no denominator
/// or nonzero test occurs here. Active means not cheaply zero, not proved nonzero.
#[inline(always)]
fn complex_division_numerators(lhs: &Complex, rhs: &Complex) -> (Real, Real) {
    crate::trace_dispatch!("hyperlattice_complex", "op", "div-numerators-fused-exact");
    (
        Real::active_signed_product_sum2([true, true], [[&lhs.re, &rhs.re], [&lhs.im, &rhs.im]]),
        Real::active_signed_product_sum2([true, false], [[&lhs.im, &rhs.re], [&lhs.re, &rhs.im]]),
    )
}

/// Return quotient components through exact-rational or general conjugate/norm arithmetic.
///
/// Classify all four components in lhs.re, lhs.im, rhs.re, rhs.im order. If every
/// classification is exact rational, call the known-exact scalar quotient directly;
/// its own zero-denominator check makes the checked flag unnecessary on that path.
/// Otherwise build the rhs squared modulus, optionally require known nonzero, then
/// invert it before building numerators and borrowing the inverse for both products.
/// No approximate division, changed predicate order, or local refinement budget is
/// introduced; native inverse/kernel errors propagate.
#[inline]
fn complex_divide_components(
    lhs: &Complex,
    rhs: &Complex,
    checked: bool,
) -> BlasResult<(Real, Real)> {
    let kinds = [
        lhs.re.exact_rational_kind(),
        lhs.im.exact_rational_kind(),
        rhs.re.exact_rational_kind(),
        rhs.im.exact_rational_kind(),
    ];
    let known_exact_rational = kinds
        .iter()
        .all(|kind| *kind != ExactRationalKind::NonRational);
    if known_exact_rational {
        crate::trace_dispatch!(
            "hyperlattice_complex",
            "op",
            "div-components-fused-exact-rational"
        );
        return Real::exact_rational_complex_quotient_known_exact(
            [&lhs.re, &lhs.im],
            [&rhs.re, &rhs.im],
        );
    }

    let denominator = &rhs.re * &rhs.re + &rhs.im * &rhs.im;
    if checked {
        require_known_nonzero(&denominator)?;
    }
    let inverse_denominator = denominator.inverse()?;
    let (re, im) = complex_division_numerators(lhs, rhs);
    Ok((
        re.mul_cached(&inverse_denominator),
        im.mul_cached(&inverse_denominator),
    ))
}

/// Select an exact component-product schedule using all four scalar reuse observations.
///
/// Observe lhs.re, lhs.im, rhs.re, rhs.im in that order. If all evidence is Some(true),
/// use three products ac, bd, (a+b)(c+d), giving re=ac-bd and im=cross-ac-bd. If all
/// are exact rational but some lack reuse evidence, use the fused rational kernel.
/// Any nonrational component selects the two active signed-product reducers.
/// The observations can remember first rational use; this is advisory scheduling,
/// not approximate value equality or a claim that every cloned input is cold.
#[inline]
fn complex_multiply_components(lhs: &Complex, rhs: &Complex) -> (Real, Real) {
    let reuse_evidence = [
        lhs.re.exact_rational_reuse_evidence(),
        lhs.im.exact_rational_reuse_evidence(),
        rhs.re.exact_rational_reuse_evidence(),
        rhs.im.exact_rational_reuse_evidence(),
    ];
    let known_exact_rational = reuse_evidence.iter().all(Option::is_some);
    if reuse_evidence
        .iter()
        .all(|evidence| *evidence == Some(true))
    {
        crate::trace_dispatch!(
            "hyperlattice_complex",
            "op",
            "mul-components-three-product-exact-rational"
        );
        let real_product = &lhs.re * &rhs.re;
        let imaginary_product = &lhs.im * &rhs.im;
        let cross_product = (&lhs.re + &lhs.im) * (&rhs.re + &rhs.im);
        let re = &real_product - &imaginary_product;
        let im = (cross_product - real_product) - imaginary_product;
        return (re, im);
    }

    if known_exact_rational {
        crate::trace_dispatch!(
            "hyperlattice_complex",
            "op",
            "mul-components-fused-cold-exact-rational"
        );
        return Real::exact_rational_complex_product_known_exact(
            [&lhs.re, &lhs.im],
            [&rhs.re, &rhs.im],
        );
    }

    crate::trace_dispatch!("hyperlattice_complex", "op", "mul-components-fused-exact");
    (
        Real::active_signed_product_sum2([true, false], [[&lhs.re, &rhs.re], [&lhs.im, &rhs.im]]),
        Real::active_signed_product_sum2([true, true], [[&lhs.re, &rhs.im], [&lhs.im, &rhs.re]]),
    )
}

/// Compute a nonzero unsigned power by scanning exponent bits from least significant upward.
///
/// The first set bit clones its factor into the accumulator; later set bits multiply.
/// Shift before squaring and skip the final unused square. Ordinary Complex products
/// select their own native schedule. Exponent zero leaves no accumulator and panics
/// at expect; callers must handle it separately. Work per multiplication is unbounded.
fn complex_powi_by_squaring(base: Complex, exponent: u64) -> Complex {
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

impl From<Real> for Complex {
    /// Embed an owned real expression as value+0i without approximation or normalization.
    fn from(value: Real) -> Self {
        crate::trace_dispatch!("hyperlattice_complex", "constructor", "from-scalar");
        Self::new(value, Real::zero())
    }
}

impl fmt::Display for Complex {
    /// Write parenthesized components as (re + imi), retaining the literal plus even for negative im.
    ///
    /// Ordinary formatting delegates to Real Display. Alternate formatting uses each
    /// component's diagnostic_decimal(-256), which may emit UnknownZero/Exhausted or
    /// other nonnumeric markers. Nested formatting uses default precision; outer width,
    /// precision, and sign flags are not forwarded. This diagnostic projection is not
    /// machine serialization or a total computation-time bound.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            write!(
                f,
                "({} + {}i)",
                self.re.diagnostic_decimal(-256),
                self.im.diagnostic_decimal(-256)
            )
        } else {
            write!(f, "({} + {}i)", self.re, self.im)
        }
    }
}

impl Add for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the left operand and consume the right operand for componentwise addition.
    fn add(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-owned-owned");
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl Add<&Complex> for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the left operand and borrow the right operand for componentwise addition. The cached adapter borrows the reusable rhs scalar; it does not promise a cache hit.
    fn add(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-owned-ref");
        Self::new(self.re.add_cached(&rhs.re), self.im.add_cached(&rhs.im))
    }
}

impl Add<Complex> for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the left operand and consume the right operand for componentwise addition.
    fn add(self, rhs: Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-ref-owned");
        Complex::new(&self.re + rhs.re, &self.im + rhs.im)
    }
}

impl Add<&Complex> for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the left operand and borrow the right operand for componentwise addition.
    fn add(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "add-ref-ref");
        Complex::new(&self.re + &rhs.re, &self.im + &rhs.im)
    }
}

impl Sub for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the left operand and consume the right operand for componentwise subtraction.
    fn sub(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-owned-owned");
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl Sub<&Complex> for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the left operand and borrow the right operand for componentwise subtraction. The cached adapter borrows the reusable rhs scalar; it does not promise a cache hit.
    fn sub(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-owned-ref");
        Self::new(self.re.sub_cached(&rhs.re), self.im.sub_cached(&rhs.im))
    }
}

impl Sub<Complex> for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the left operand and consume the right operand for componentwise subtraction.
    fn sub(self, rhs: Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-ref-owned");
        Complex::new(&self.re - rhs.re, &self.im - rhs.im)
    }
}

impl Sub<&Complex> for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the left operand and borrow the right operand for componentwise subtraction.
    fn sub(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "sub-ref-ref");
        Complex::new(&self.re - &rhs.re, &self.im - &rhs.im)
    }
}

impl Neg for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the pair and negate both components; returns a new complex carrier.
    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "neg-owned");
        Self::new(-self.re, -self.im)
    }
}

impl Neg for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the pair and negate both components; returns a new complex carrier.
    fn neg(self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "neg-ref");
        Complex::new(-&self.re, -&self.im)
    }
}

impl Mul for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the left operand and consume the right operand for exact complex multiplication. All ownership forms share the same component scheduler and preserve native reuse evidence.
    fn mul(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-owned-owned");
        let (re, im) = complex_multiply_components(&self, &rhs);
        Self::new(re, im)
    }
}

impl Mul<&Complex> for Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Self;

    /// Consume the left operand and borrow the right operand for exact complex multiplication. All ownership forms share the same component scheduler and preserve native reuse evidence.
    fn mul(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-owned-ref");
        let (re, im) = complex_multiply_components(&self, rhs);
        Self::new(re, im)
    }
}

impl Mul<Complex> for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the left operand and consume the right operand for exact complex multiplication. All ownership forms share the same component scheduler and preserve native reuse evidence.
    fn mul(self, rhs: Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-ref-owned");
        let (re, im) = complex_multiply_components(self, &rhs);
        Complex::new(re, im)
    }
}

impl Mul<&Complex> for &Complex {
    /// New complex value containing the resulting exact component expressions.
    type Output = Complex;

    /// Borrow the left operand and borrow the right operand for exact complex multiplication. All ownership forms share the same component scheduler and preserve native reuse evidence.
    fn mul(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "mul-ref-ref");
        let (re, im) = complex_multiply_components(self, rhs);
        Complex::new(re, im)
    }
}

impl Div for Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Self>;

    /// Consume the left operand and consume the right operand for fallible complex division. All complex ownership forms share the quotient helper with checked=false and propagate its native errors.
    fn div(self, rhs: Self) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-owned-owned");
        let (re, im) = complex_divide_components(&self, &rhs, false)?;
        Ok(Self::new(re, im))
    }
}

impl Div<&Complex> for Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Self>;

    /// Consume the left operand and borrow the right operand for fallible complex division. All complex ownership forms share the quotient helper with checked=false and propagate its native errors.
    fn div(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-owned-ref");
        let (re, im) = complex_divide_components(&self, rhs, false)?;
        Ok(Self::new(re, im))
    }
}

impl Div<Complex> for &Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Complex>;

    /// Borrow the left operand and consume the right operand for fallible complex division. All complex ownership forms share the quotient helper with checked=false and propagate its native errors.
    fn div(self, rhs: Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-ref-owned");
        let (re, im) = complex_divide_components(self, &rhs, false)?;
        Ok(Complex::new(re, im))
    }
}

impl Div<&Complex> for &Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Complex>;

    /// Borrow the left operand and borrow the right operand for fallible complex division. All complex ownership forms share the quotient helper with checked=false and propagate its native errors.
    fn div(self, rhs: &Complex) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-ref-ref");
        let (re, im) = complex_divide_components(self, rhs, false)?;
        Ok(Complex::new(re, im))
    }
}

impl Div<Real> for Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Self>;

    /// Consume the left operand and consume the right operand; compute one native scalar inverse and borrow it for both components. No explicit checked zero guard is added.
    fn div(self, rhs: Real) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-real-owned");
        let inv_rhs = rhs.inverse()?;
        Ok(Self::new(
            self.re.mul_cached(&inv_rhs),
            self.im.mul_cached(&inv_rhs),
        ))
    }
}

impl Div<&Real> for Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Self>;

    /// Consume the left operand and borrow the right operand; compute one native scalar inverse and borrow it for both components. No explicit checked zero guard is added.
    fn div(self, rhs: &Real) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "div-real-ref");
        let inv_rhs = rhs.inverse_ref()?;
        Ok(Self::new(
            self.re.mul_cached(&inv_rhs),
            self.im.mul_cached(&inv_rhs),
        ))
    }
}

impl BitXor<i64> for Complex {
    /// Native fallible complex result, preserving Problem without adding a certainty receipt.
    type Output = BlasResult<Self>;

    /// Interpret ^ with an i64 as [`Complex::powi`], not bitwise XOR; preserve its zero-power domain and native errors.
    fn bitxor(self, rhs: i64) -> Self::Output {
        crate::trace_dispatch!("hyperlattice_complex", "op", "bitxor-powi");
        self.powi(rhs)
    }
}
