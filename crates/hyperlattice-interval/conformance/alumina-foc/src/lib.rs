//! Conformance client for shared fixed arithmetic, preserving Alumina semantics.
//!
//! This is an independently built, unpublished client, not a firmware migration.
//! Domain wrappers, rotation validation and errors remain owned by Alumina.
//! The functions intentionally preserve each current rounding/failure boundary.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

pub mod samples;

use alumina_foc::{AlphaBeta, DqInterval, FocError, Phase3, Q30, Q30Interval, Rotation};
use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix, IntervalVector};

type Fixed = FixedInterval<i32, 30>;
type Wide = FixedInterval<i64, 30>;
const SCALE: i64 = 1 << 30;

fn input(value: Q30Interval) -> Fixed {
    Fixed::new(value.lower().bits(), value.upper().bits()).expect("validated Q30 bounds")
}

fn output(value: Fixed) -> Q30Interval {
    let (lo, hi) = value
        .bounds()
        .expect("operations on nonempty finite inputs");
    Q30Interval::new(Q30::from_bits(lo), Q30::from_bits(hi)).expect("checked fixed bounds")
}

fn scalar_error(error: IntervalError) -> FocError {
    match error {
        IntervalError::Overflow => FocError::Range,
        // This client uses only fixed F30 operations on validated nonempty
        // inputs, with constant nonzero divisors. Other errors are unreachable.
        _ => unreachable!("validated fixed transform: {error}"),
    }
}

fn lattice_error(error: IntervalLatticeError) -> FocError {
    match error {
        IntervalLatticeError::Scalar(error) => scalar_error(error),
        _ => unreachable!("statically dimensioned matrix product: {error}"),
    }
}

fn narrow(value: Wide) -> Result<Q30Interval, FocError> {
    value.convert::<i32>().map(output).map_err(scalar_error)
}

fn sum_products(a: Fixed, b: Fixed, c: Fixed, d: Fixed) -> Result<Fixed, FocError> {
    // The zero accumulator is exact and adds no rounding/overflow boundary.
    let matrix = IntervalMatrix::new([[a, c]]).expect("valid scalar family");
    let vector = IntervalVector::new([b, d]).expect("valid scalar family");
    let result = matrix.mul_vector(&vector, &mut ()).map_err(lattice_error)?;
    Ok(result.into_lanes().expect("nonempty product")[0])
}

fn difference_products(a: Fixed, b: Fixed, c: Fixed, d: Fixed) -> Result<Fixed, FocError> {
    // Negating a coefficient and using a dense product changes intermediate
    // overflow, e.g. MIN * -1 followed by subtraction. Preserve both products.
    let first = a.checked_mul(&b).map_err(scalar_error)?;
    let second = c.checked_mul(&d).map_err(scalar_error)?;
    first.checked_sub(&second).map_err(scalar_error)
}

/// Clarke transform with the same balance policy, rounding and errors as Alumina.
/// Widened Q30 storage retains the integer numerator before division by three.
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
