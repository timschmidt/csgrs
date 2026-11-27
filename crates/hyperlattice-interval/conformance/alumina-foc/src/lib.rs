//! Conformance client for shared fixed arithmetic, preserving Alumina semantics.
//!
//! This is an independently built, unpublished client, not a firmware migration.
//! Domain wrappers, rotation validation and errors remain owned by Alumina.
//! The functions intentionally preserve each current rounding/failure boundary.
//!
//! Inputs are ordered, nonempty closed intervals with signed Q2.30 endpoints.
//! Operations use Cartesian interval bounds; rotation components retain their
//! validated wrapper but are evaluated as separate sine/cosine intervals. An
//! inverse transform need not undo prior widening or recover lost correlation.
//! Results preserve Alumina's domain wrappers and Range/UnbalancedPhases errors,
//! not a new predicate certainty receipt or a whole-controller safety proof.
//! All storage is fixed-size and the unit arithmetic context adds no work budget.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

pub mod samples;

use alumina_foc::{AlphaBeta, DqInterval, FocError, Phase3, Q30, Q30Interval, Rotation};
use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix, IntervalVector};

/// Native-width closed intervals on Alumina's signed 2^-30 grid, plus kernel emptiness.
type Fixed = FixedInterval<i32, 30>;
/// Same grid with i64 endpoint storage for Clarke's exact wide linear combinations.
type Wide = FixedInterval<i64, 30>;
/// Raw numerator for exact one; twice this value supplies affine multiplication by two.
const SCALE: i64 = 1 << 30;

/// Import exact endpoint numerators; the Q30Interval wrapper already guarantees order.
/// Scale 30 is valid for i32, so a failed constructor would violate the adapter invariant.
fn input(value: Q30Interval) -> Fixed {
    Fixed::new(value.lower().bits(), value.upper().bits()).expect("validated Q30 bounds")
}

/// Rewrap nonempty ordered fixed bounds without rounding or a change of scale.
/// Private callers only supply nonempty transform results; empty would panic because
/// Alumina's Q30Interval has no empty variant. Constructor failure is an invariant violation.
fn output(value: Fixed) -> Q30Interval {
    let (lo, hi) = value
        .bounds()
        .expect("operations on nonempty finite inputs");
    Q30Interval::new(Q30::from_bits(lo), Q30::from_bits(hi)).expect("checked fixed bounds")
}

/// Translate scalar endpoint overflow to Alumina Range; panic on an unexpected error kind.
/// This is not a general scalar-error adapter: validated nonempty F30 inputs and
/// nonzero constant divisors rule out the other errors on these transform paths.
fn scalar_error(error: IntervalError) -> FocError {
    match error {
        IntervalError::Overflow => FocError::Range,
        // This client uses only fixed F30 operations on validated nonempty
        // inputs, with constant nonzero divisors. Other errors are unreachable.
        _ => unreachable!("validated fixed transform: {error}"),
    }
}

/// Translate the scalar failure of a statically dimensioned product through scalar_error.
/// Other lattice errors are impossible for the constructed one-row/two-column operands
/// and panic rather than being silently collapsed into Range.
fn lattice_error(error: IntervalLatticeError) -> FocError {
    match error {
        IntervalLatticeError::Scalar(error) => scalar_error(error),
        _ => unreachable!("statically dimensioned matrix product: {error}"),
    }
}

/// Check both wide endpoints fit i32, preserve the Q30 grid, and rewrap the result.
/// Overflow becomes Range; no saturation, extra rounding or partial interval is returned.
/// Private callers supply nonempty intervals; an empty value would panic in output.
fn narrow(value: Wide) -> Result<Q30Interval, FocError> {
    value.convert::<i32>().map(output).map_err(scalar_error)
}

/// Enclose a*b+c*d through a one-row matrix action with staged Q30 products and addition.
/// The dot kernel visits a*b before c*d. Its initial exact zero addition changes
/// neither bounds nor failure, but product overflow still precedes possible cancellation.
/// Nonempty operands are a private precondition; only the final scalar lane is returned.
fn sum_products(a: Fixed, b: Fixed, c: Fixed, d: Fixed) -> Result<Fixed, FocError> {
    // The zero accumulator is exact and adds no rounding/overflow boundary.
    let matrix = IntervalMatrix::new([[a, c]]).expect("valid scalar family");
    let vector = IntervalVector::new([b, d]).expect("valid scalar family");
    let result = matrix.mul_vector(&vector, &mut ()).map_err(lattice_error)?;
    Ok(result.into_lanes().expect("nonempty product")[0])
}

/// Enclose a*b-c*d by separately rounded products followed by checked subtraction.
/// Preserve first-product, second-product, then subtraction failure order. Negating
/// a coefficient in a dense matrix can change representability at an intermediate step.
fn difference_products(a: Fixed, b: Fixed, c: Fixed, d: Fixed) -> Result<Fixed, FocError> {
    // Negating a coefficient and using a dense product changes intermediate
    // overflow, e.g. MIN * -1 followed by subtraction. Preserve both products.
    let first = a.checked_mul(&b).map_err(scalar_error)?;
    let second = c.checked_mul(&d).map_err(scalar_error)?;
    first.checked_sub(&second).map_err(scalar_error)
}

/// Clarke transform with the same balance policy, rounding and errors as Alumina.
/// Widened Q30 storage retains the integer numerator before division by three.
///
/// Require a nonnegative maximum_zero_sequence and every value of the independent
/// residual interval a+b+c to have magnitude at most that bound; otherwise return
/// UnbalancedPhases before constructing either output. The bound is on the sum,
/// not its average. Compute alpha=(2a-b-c)/3 in wide storage, rounding outward only
/// at division, then narrow it before computing beta=(b-c)/sqrt(3). Beta uses the
/// authored INV_SQRT_3 bracket with outward multiplication and checked narrowing.
///
/// Intermediate differences may exceed i32 while the result still fits. A
/// nonrepresentable rounded output returns Range, with alpha failure preceding
/// beta work. No partly constructed AlphaBeta escapes and no source is mutated.
pub fn clarke(phases: Phase3, maximum_zero_sequence: Q30) -> Result<AlphaBeta, FocError> {
    if maximum_zero_sequence.bits() < 0 {
        return Err(FocError::UnbalancedPhases);
    }
    let a = input(phases.a).convert::<i64>().map_err(scalar_error)?;
    let b = input(phases.b).convert::<i64>().map_err(scalar_error)?;
    let c = input(phases.c).convert::<i64>().map_err(scalar_error)?;
    let residual = a
        .checked_add(&b)
        .map_err(scalar_error)?
        .checked_add(&c)
        .map_err(scalar_error)?;
    let (lo, hi) = residual.bounds().expect("nonempty residual");
    if lo.unsigned_abs().max(hi.unsigned_abs()) > maximum_zero_sequence.bits() as u64 {
        return Err(FocError::UnbalancedPhases);
    }
    // All i32 source values and these integer linear combinations fit i64.
    let numerator = a
        .affine(2 * SCALE, 0)
        .map_err(scalar_error)?
        .checked_sub(&b)
        .map_err(scalar_error)?
        .checked_sub(&c)
        .map_err(scalar_error)?;
    let alpha = narrow(numerator.div_integer(3).map_err(scalar_error)?)?;
    let difference = b.checked_sub(&c).map_err(scalar_error)?;
    let scale = input(alumina_foc::INV_SQRT_3)
        .convert::<i64>()
        .map_err(scalar_error)?;
    let beta = narrow(difference.checked_mul(&scale).map_err(scalar_error)?)?;
    Ok(AlphaBeta { alpha, beta })
}

/// Inverse Clarke transform, retaining multiplication then negation boundaries.
///
/// Return a=alpha, b=-alpha/2+sqrt(3)*beta/2, c=-alpha/2-sqrt(3)*beta/2 as interval
/// enclosures. First round HALF*alpha and negate it, then round the product of
/// beta with SQRT_3_OVER_2, then compute b before c. Intermediate or final overflow
/// returns Range. The copied alpha lane is exact; the three lanes are independent
/// enclosures and need not have an exactly zero interval sum or undo a prior Clarke.
pub fn inverse_clarke(value: AlphaBeta) -> Result<Phase3, FocError> {
    let negative_half = input(Q30Interval::HALF)
        .checked_mul(&input(value.alpha))
        .map_err(scalar_error)?
        .checked_neg()
        .map_err(scalar_error)?;
    let beta = input(alumina_foc::SQRT_3_OVER_2)
        .checked_mul(&input(value.beta))
        .map_err(scalar_error)?;
    let b = negative_half.checked_add(&beta).map_err(scalar_error)?;
    let c = negative_half.checked_sub(&beta).map_err(scalar_error)?;
    Ok(Phase3 {
        a: value.alpha,
        b: output(b),
        c: output(c),
    })
}

/// Park transform using validated rotation evidence and staged fixed operations.
///
/// Evaluate d=alpha*cosine+beta*sine, then q=beta*cosine-alpha*sine, retaining each
/// product's outward Q30 rounding before addition/subtraction. The Rotation wrapper
/// supplies validated component/norm evidence; this operation does not revalidate
/// it or enforce a joint unit-circle constraint while evaluating its intervals.
/// Intermediate or final overflow returns Range even if an algebraically combined
/// expression would fit. A failed d prevents q work; no partial DqInterval escapes.
pub fn park(value: AlphaBeta, rotation: Rotation) -> Result<DqInterval, FocError> {
    let alpha = input(value.alpha);
    let beta = input(value.beta);
    let sine = input(rotation.sine());
    let cosine = input(rotation.cosine());
    let d = sum_products(alpha, cosine, beta, sine)?;
    let q = difference_products(beta, cosine, alpha, sine)?;
    Ok(DqInterval {
        d: output(d),
        q: output(q),
    })
}

/// Inverse Park transform with the same intermediate failures as Alumina.
///
/// Evaluate alpha=d*cosine-q*sine before beta=d*sine+q*cosine. Products round
/// outward separately; every multiply, subtraction and addition retains its Range
/// boundary. Validated rotation components are consumed without a new norm check.
/// The result encloses the inverse-frame expression, not a correlation-preserving
/// reversal of a prior Park call; widening and intermediate failures can remain.
pub fn inverse_park(value: DqInterval, rotation: Rotation) -> Result<AlphaBeta, FocError> {
    let d = input(value.d);
    let q = input(value.q);
    let sine = input(rotation.sine());
    let cosine = input(rotation.cosine());
    let alpha = difference_products(d, cosine, q, sine)?;
    let beta = sum_products(d, sine, q, cosine)?;
    Ok(AlphaBeta {
        alpha: output(alpha),
        beta: output(beta),
    })
}
