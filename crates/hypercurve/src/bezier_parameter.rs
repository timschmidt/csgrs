//! Exact carriers for Bezier split parameters.
//!
//! Bezier arrangements eventually need split points whose parameters are not
//! represented by the scalar `Real` API yet. This module gives those parameters
//! a first-class exact carrier instead of forcing an approximate collapse: an
//! exact parameter is either a represented [`Real`] or an algebraic root
//! described by a power-basis polynomial and an isolating interval in `[0, 1]`.
//! That is the representation boundary Yap prescribes for exact geometric
//! computation: construct exact objects first, then branch only through exact
//! predicates or explicit uncertainty; see Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 3-23 (1997).
//!
//! The root-count validation below uses Sturm sequences. The sign-variation
//! theorem used here is the classical one from Sturm, "Memoire sur la
//! resolution des equations numeriques," *Bulletin des Sciences de Ferussac*
//! 11 (1829). Hypercurve intentionally stores the validated interval with the
//! parameter so later Bezier boolean and offset APIs can carry a certificate
//! rather than re-solving the root from scratch.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, in_closed_unit_interval, is_zero, real_sign};
use crate::{
    BezierMonotoneSpan, Classification, CurveError, CurvePolicy, CurveResult, UncertaintyReason,
};

/// Power-basis polynomial used to define an algebraic Bezier parameter.
///
/// Coefficients are stored from low to high degree, so `coefficients()[0]` is
/// the constant term. Constructors trim certified trailing zero coefficients
/// and reject the structurally zero polynomial. Unknown leading-zero status is
/// reported as [`Classification::Uncertain`] so a topology caller cannot
/// silently choose the wrong degree.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParameterPolynomial {
    coefficients: Vec<Real>,
}

/// Closed isolating interval for a Bezier parameter root.
///
/// The interval is always certified to lie inside `[0, 1]` and to satisfy
/// `start <= end`. `BezierAlgebraicParameter2` additionally requires the
/// defining polynomial to have no endpoint root and exactly one distinct root
/// in this interval under Sturm validation.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParameterInterval {
    start: Real,
    end: Real,
}

/// Algebraic Bezier parameter represented by a polynomial and isolating interval.
///
/// This is the minimum certificate needed by native Bezier boolean/offset
/// materialization: consumers can retain the exact defining equation, carry
/// the bracket through API boundaries, and ask for ordering only when interval
/// separation proves it. The `root_count` is stored explicitly so downstream
/// code can assert that the object was validated as a singleton isolator.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicParameter2 {
    polynomial: BezierParameterPolynomial,
    interval: BezierParameterInterval,
    root_count: usize,
}

/// Exact Bezier parameter carrier.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierParameter2 {
    /// A parameter represented directly by `Real`.
    Exact(Real),
    /// A parameter represented as one isolated algebraic root.
    Algebraic(BezierAlgebraicParameter2),
}

impl BezierParameterPolynomial {
    /// Constructs a nonzero power-basis polynomial.
    pub fn try_new_power_basis(
        coefficients: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match normalize_coefficients(coefficients, policy)? {
            Classification::Decided(Some(coefficients)) => {
                Ok(Classification::Decided(Self { coefficients }))
            }
            Classification::Decided(None) => Err(CurveError::InvalidBezierPolynomial),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns coefficients in low-to-high power-basis order.
    pub fn coefficients(&self) -> &[Real] {
        &self.coefficients
    }

    /// Returns the certified degree.
    pub fn degree(&self) -> usize {
        self.coefficients.len() - 1
    }

    /// Evaluates the polynomial at `parameter` using Horner's rule.
    pub fn evaluate(&self, parameter: &Real) -> Real {
        evaluate_coefficients(&self.coefficients, parameter)
    }

    /// Counts distinct roots in `interval` using a Sturm sequence.
    ///
    /// The interval endpoints must not themselves be roots. Endpoint roots are
    /// legitimate split parameters, but they should be represented with
    /// [`BezierParameter2::Exact`] or isolated by a narrower interval. This
    /// avoids half-open endpoint conventions leaking into arrangement code.
    pub fn root_count_in_interval(
        &self,
        interval: &BezierParameterInterval,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<usize>> {
        let start_value = self.evaluate(interval.start());
        let end_value = self.evaluate(interval.end());
        match (
            real_sign(&start_value, policy),
            real_sign(&end_value, policy),
        ) {
            (Some(RealSign::Zero), _) | (_, Some(RealSign::Zero)) => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            (Some(_), Some(_)) => {}
            _ => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        let sequence = match sturm_sequence(&self.coefficients, policy)? {
            Classification::Decided(sequence) => sequence,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let start_variations = sign_variations_at(&sequence, interval.start(), policy)?;
        let end_variations = sign_variations_at(&sequence, interval.end(), policy)?;
        match (start_variations, end_variations) {
            (Classification::Decided(start), Classification::Decided(end)) => {
                Ok(Classification::Decided(start.saturating_sub(end)))
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Ok(Classification::Uncertain(reason))
            }
        }
    }
}

impl BezierParameterInterval {
    /// Constructs a closed interval in Bezier parameter space.
    pub fn try_new(
        start: Real,
        end: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let in_start = in_closed_unit_interval(&start, policy);
        let in_end = in_closed_unit_interval(&end, policy);
        match (in_start, in_end) {
            (Some(false), _) | (_, Some(false)) => return Err(CurveError::InvalidBezierParameter),
            (Some(true), Some(true)) => {}
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        match compare_reals(&start, &end, policy) {
            Some(Ordering::Greater) => Err(CurveError::InvalidBezierRange),
            Some(_) => Ok(Classification::Decided(Self { start, end })),
            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }

    /// Converts an existing monotone span into a validated parameter interval.
    pub fn from_monotone_span(
        span: &BezierMonotoneSpan,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        Self::try_new(span.start().clone(), span.end().clone(), policy)
    }

    /// Returns the interval start.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the interval end.
    pub const fn end(&self) -> &Real {
        &self.end
    }
}

impl BezierAlgebraicParameter2 {
    /// Validates a singleton algebraic Bezier parameter isolator.
    pub fn try_isolate(
        polynomial: BezierParameterPolynomial,
        interval: BezierParameterInterval,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let count = match polynomial.root_count_in_interval(&interval, policy)? {
            Classification::Decided(count) => count,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if count != 1 {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }

        Ok(Classification::Decided(Self {
            polynomial,
            interval,
            root_count: count,
        }))
    }

    /// Returns the defining polynomial.
    pub const fn polynomial(&self) -> &BezierParameterPolynomial {
        &self.polynomial
    }

    /// Returns the certified isolating interval.
    pub const fn interval(&self) -> &BezierParameterInterval {
        &self.interval
    }

    /// Returns the certified distinct-root count for the interval.
    pub const fn root_count(&self) -> usize {
        self.root_count
    }
}

impl BezierParameter2 {
    /// Constructs a represented exact Bezier parameter.
    pub fn exact(value: Real, policy: &CurvePolicy) -> CurveResult<Classification<Self>> {
        match in_closed_unit_interval(&value, policy) {
            Some(true) => Ok(Classification::Decided(Self::Exact(value))),
            Some(false) => Err(CurveError::InvalidBezierParameter),
            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }

    /// Wraps a validated algebraic Bezier parameter.
    pub const fn algebraic(value: BezierAlgebraicParameter2) -> Self {
        Self::Algebraic(value)
    }

    /// Returns the exact value when represented directly.
    pub const fn as_exact(&self) -> Option<&Real> {
        match self {
            Self::Exact(value) => Some(value),
            Self::Algebraic(_) => None,
        }
    }

    /// Returns true for a directly represented exact parameter.
    pub const fn is_exact(&self) -> bool {
        matches!(self, Self::Exact(_))
    }

    /// Returns the known enclosing interval.
    pub fn known_interval(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierParameterInterval>> {
        match self {
            Self::Exact(value) => {
                BezierParameterInterval::try_new(value.clone(), value.clone(), policy)
            }
            Self::Algebraic(value) => Ok(Classification::Decided(value.interval().clone())),
        }
    }

    /// Compares parameters when exact values or disjoint isolating intervals prove the order.
    pub fn cmp_by_interval(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Ordering>> {
        if let (Self::Exact(left), Self::Exact(right)) = (self, other) {
            return Ok(compare_reals(left, right, policy)
                .map(Classification::Decided)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
        }

        let left = match self.known_interval(policy)? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let right = match other.known_interval(policy)? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        if compare_reals(left.end(), right.start(), policy) == Some(Ordering::Less) {
            return Ok(Classification::Decided(Ordering::Less));
        }
        if compare_reals(right.end(), left.start(), policy) == Some(Ordering::Less) {
            return Ok(Classification::Decided(Ordering::Greater));
        }

        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }
}

fn sturm_sequence(
    coefficients: &[Real],
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Vec<Real>>>> {
    let p0 = coefficients.to_vec();
    let p1 = derivative_coefficients(coefficients);
    let p1 = match normalize_coefficients(p1, policy)? {
        Classification::Decided(Some(coefficients)) => coefficients,
        Classification::Decided(None) => return Ok(Classification::Decided(vec![p0])),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut sequence = vec![p0, p1];
    while sequence.len() < 64 {
        let last = sequence[sequence.len() - 1].clone();
        let previous = sequence[sequence.len() - 2].clone();
        let remainder = match polynomial_remainder(previous, last, policy)? {
            Classification::Decided(Some(remainder)) => remainder,
            Classification::Decided(None) => break,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        sequence.push(negate_coefficients(remainder));
    }

    Ok(Classification::Decided(sequence))
}

fn sign_variations_at(
    sequence: &[Vec<Real>],
    parameter: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<usize>> {
    let mut previous = None;
    let mut variations = 0_usize;

    for polynomial in sequence {
        let value = evaluate_coefficients(polynomial, parameter);
        let sign = match real_sign(&value, policy) {
            Some(RealSign::Zero) => continue,
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if let Some(previous) = previous
            && previous != sign
        {
            variations += 1;
        }
        previous = Some(sign);
    }

    Ok(Classification::Decided(variations))
}

fn polynomial_remainder(
    mut remainder: Vec<Real>,
    divisor: Vec<Real>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let divisor = match normalize_coefficients(divisor, policy)? {
        Classification::Decided(Some(coefficients)) => coefficients,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    loop {
        remainder = match normalize_coefficients(remainder, policy)? {
            Classification::Decided(Some(coefficients)) => coefficients,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if remainder.len() < divisor.len() {
            return Ok(Classification::Decided(Some(remainder)));
        }

        let shift = remainder.len() - divisor.len();
        let factor = (remainder[remainder.len() - 1].clone() / divisor[divisor.len() - 1].clone())?;
        for (index, divisor_coefficient) in divisor.iter().enumerate() {
            let product = &factor * divisor_coefficient;
            remainder[shift + index] = &remainder[shift + index] - &product;
        }
    }
}

fn normalize_coefficients(
    mut coefficients: Vec<Real>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    while let Some(last) = coefficients.last() {
        match is_zero(last, policy) {
            Some(true) => {
                coefficients.pop();
            }
            Some(false) => return Ok(Classification::Decided(Some(coefficients))),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    Ok(Classification::Decided(None))
}

fn derivative_coefficients(coefficients: &[Real]) -> Vec<Real> {
    let mut derivative = Vec::with_capacity(coefficients.len().saturating_sub(1));
    for (degree, coefficient) in coefficients.iter().enumerate().skip(1) {
        let scale = Real::from(degree as i64);
        derivative.push(coefficient * &scale);
    }
    derivative
}

fn evaluate_coefficients(coefficients: &[Real], parameter: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |accumulator, coefficient| {
            (&accumulator * parameter) + coefficient
        })
}

fn negate_coefficients(coefficients: Vec<Real>) -> Vec<Real> {
    coefficients
        .into_iter()
        .map(|coefficient| Real::zero() - coefficient)
        .collect()
}
