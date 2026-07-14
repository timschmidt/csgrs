//! Exact carriers for Bezier split parameters.
//!
//! Bezier arrangements eventually need split points whose parameters are not
//! represented by the scalar `Real` API yet. This module gives those parameters
//! a first-class exact carrier instead of forcing an approximate collapse: an
//! exact parameter is either a represented [`Real`] or an algebraic root
//! described by a power-basis polynomial and an isolating interval in `[0, 1]`.
//! That is the representation boundary the exactness model prescribes for exact geometric
//! computation: construct exact objects first, then branch only through exact
//! predicates or explicit uncertainty.
//!
//! The root-count validation below uses Sturm sequences. Hypercurve stores the
//! validated interval with the
//! parameter so later Bezier boolean and offset APIs can carry a certificate
//! rather than re-solving the root from scratch.
//! Linear defining polynomials are additionally recoverable as represented
//! [`Real`] values when the exact quotient is certified to be the singleton
//! root. That is the first narrow "true algebraic root materialization" bridge:
//! it keeps the exactness model's construction/decision separation, but it avoids retaining an
//! algebraic wrapper when the exact root already lives in the scalar tower.

use std::cmp::Ordering;

use hyperreal::{Rational as HyperRational, Real, RealSign};
use num::{BigInt, BigRational, BigUint, Integer, One, Zero};

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

/// Operation counts for certified root isolation in the Bezier unit interval.
///
/// These counters expose algorithmic work without introducing timing or a
/// primitive-float observation boundary.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct BezierRootIsolationTrace2 {
    sturm_sequence_builds: usize,
    interval_root_counts: usize,
    bisections: usize,
    rational_reconstruction_refinements: usize,
    maximum_depth: usize,
}

/// Certified unit-interval roots together with their isolation trace.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRootIsolationResult2 {
    roots: Vec<BezierParameter2>,
    trace: BezierRootIsolationTrace2,
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
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierParameter2 {
    /// A parameter represented directly by `Real`.
    Exact(Real),
    /// A parameter represented as one isolated algebraic root.
    Algebraic(BezierAlgebraicParameter2),
}

/// Oriented positive-length range in a Bezier segment's `[0, 1]` domain.
///
/// Endpoints retain their exact representation, including isolated algebraic
/// roots. A descending range records reversed traversal; callers do not need
/// to demote algebraic split boundaries merely to express orientation.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParameterRange2 {
    start: BezierParameter2,
    end: BezierParameter2,
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

    /// Constructs a nonzero polynomial from Bernstein-basis coefficients.
    pub fn try_new_bernstein_basis(
        coefficients: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let coefficients = bernstein_to_power_coefficients(coefficients)?;
        Self::try_new_power_basis(coefficients, policy)
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

    /// Reduces a power-basis expression modulo this defining polynomial.
    ///
    /// At any root of `self`, the returned remainder has exactly the same
    /// value as the input expression. Algebraic image construction uses this
    /// to avoid rebuilding values already implied by retained root evidence.
    pub(crate) fn reduce_power_basis(
        &self,
        coefficients: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Real>>> {
        match polynomial_remainder(coefficients, self.coefficients.clone(), policy)? {
            Classification::Decided(Some(remainder)) => Ok(Classification::Decided(remainder)),
            Classification::Decided(None) => Ok(Classification::Decided(vec![Real::zero()])),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
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
        let sequence = match sturm_sequence(&self.coefficients, policy)? {
            Classification::Decided(sequence) => sequence,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.root_count_in_interval_with_sequence(interval, &sequence, policy)
    }

    fn root_count_in_interval_with_sequence(
        &self,
        interval: &BezierParameterInterval,
        sequence: &[Vec<Real>],
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

        let start_variations = sign_variations_at(sequence, interval.start(), policy)?;
        let end_variations = sign_variations_at(sequence, interval.end(), policy)?;
        match (start_variations, end_variations) {
            (Classification::Decided(start), Classification::Decided(end)) => {
                Ok(Classification::Decided(start.saturating_sub(end)))
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Ok(Classification::Uncertain(reason))
            }
        }
    }

    /// Returns the nonconstant monic GCD when the polynomials share roots.
    pub fn greatest_common_divisor(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<Self>>> {
        let mut first = self.coefficients.clone();
        let mut second = other.coefficients.clone();
        while !second.is_empty() {
            let remainder = match polynomial_remainder(first, second.clone(), policy)? {
                Classification::Decided(Some(remainder)) => remainder,
                Classification::Decided(None) => Vec::new(),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            first = second;
            second = remainder;
        }
        let first = match normalize_coefficients(first, policy)? {
            Classification::Decided(Some(first)) => first,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if first.len() == 1 {
            return Ok(Classification::Decided(None));
        }
        let leading = first.last().expect("nonempty normalized polynomial");
        let monic = first
            .iter()
            .map(|coefficient| coefficient / leading)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Classification::Decided(Some(Self {
            coefficients: monic,
        })))
    }

    /// Isolates every distinct root in `[0, 1]` as an exact parameter carrier.
    pub fn isolate_unit_interval_roots(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        Ok(self
            .isolate_unit_interval_roots_with_trace(policy)?
            .map(BezierRootIsolationResult2::into_roots))
    }

    /// Isolates every distinct root in `[0, 1]` and reports exact work counts.
    pub fn isolate_unit_interval_roots_with_trace(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierRootIsolationResult2>> {
        isolate_unit_roots(self.coefficients.clone(), policy)
    }

    /// Returns whether this polynomial changes sign at a certified root.
    ///
    /// A sign change is equivalent to odd root multiplicity. Represented roots
    /// are divided out exactly until the first nonzero residual is reached;
    /// isolated algebraic roots use the certified nonroot signs at the two
    /// isolator boundaries. No approximate root value is introduced.
    pub fn changes_sign_at_root(
        &self,
        parameter: &BezierParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        match parameter {
            BezierParameter2::Exact(root) => {
                let mut coefficients = self.coefficients.clone();
                let mut multiplicity = 0_usize;
                while coefficients.len() > 1 {
                    match real_sign(&evaluate_coefficients(&coefficients, root), policy) {
                        Some(RealSign::Zero) => {
                            multiplicity += 1;
                            coefficients = divide_by_linear_root(&coefficients, root);
                        }
                        Some(RealSign::Positive | RealSign::Negative) => break,
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                        }
                    }
                }
                if multiplicity == 0 {
                    return Err(CurveError::InvalidBezierParameter);
                }
                Ok(Classification::Decided(!multiplicity.is_multiple_of(2)))
            }
            BezierParameter2::Algebraic(parameter) => {
                let count = match self.root_count_in_interval(parameter.interval(), policy)? {
                    Classification::Decided(count) => count,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if count != 1 {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                let start = match real_sign(&self.evaluate(parameter.interval().start()), policy) {
                    Some(RealSign::Positive) => true,
                    Some(RealSign::Negative) => false,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
                let end = match real_sign(&self.evaluate(parameter.interval().end()), policy) {
                    Some(RealSign::Positive) => true,
                    Some(RealSign::Negative) => false,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
                Ok(Classification::Decided(start != end))
            }
        }
    }
}

impl BezierRootIsolationTrace2 {
    /// Number of Sturm sequences constructed during the complete query.
    pub const fn sturm_sequence_builds(&self) -> usize {
        self.sturm_sequence_builds
    }

    /// Number of certified interval root-count queries.
    pub const fn interval_root_counts(&self) -> usize {
        self.interval_root_counts
    }

    /// Number of interval bisections performed by root isolation.
    pub const fn bisections(&self) -> usize {
        self.bisections
    }

    /// Number of refinements used while testing rational reconstruction.
    pub const fn rational_reconstruction_refinements(&self) -> usize {
        self.rational_reconstruction_refinements
    }

    /// Deepest pending unit-interval subdivision visited.
    pub const fn maximum_depth(&self) -> usize {
        self.maximum_depth
    }
}

impl BezierRootIsolationResult2 {
    /// Returns the ordered distinct roots in `[0, 1]`.
    pub fn roots(&self) -> &[BezierParameter2] {
        &self.roots
    }

    /// Consumes the result and returns its ordered roots.
    pub fn into_roots(self) -> Vec<BezierParameter2> {
        self.roots
    }

    /// Returns the algorithmic work trace.
    pub const fn trace(&self) -> &BezierRootIsolationTrace2 {
        &self.trace
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

    fn from_certified_singleton(
        polynomial: BezierParameterPolynomial,
        interval: BezierParameterInterval,
    ) -> Self {
        Self {
            polynomial,
            interval,
            root_count: 1,
        }
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

    /// Returns the represented root when this isolator contains an exact rational root.
    ///
    /// Exact-rational coefficients are cleared to a primitive integer
    /// polynomial. The rational-root theorem bounds the reduced denominator by
    /// the leading coefficient. The retained Sturm isolator is then refined
    /// until rational reconstruction is unique under that bound. Continued-
    /// fraction candidates are accepted only after exact polynomial replay.
    /// Nonrational coefficients and irrational roots return `None` without
    /// demoting the algebraic carrier.
    pub fn represented_rational_root(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<Real>>> {
        if self.polynomial.degree() == 1 {
            return self.represented_linear_root(policy);
        }

        let Some(denominator_bound) = rational_root_denominator_bound(&self.polynomial) else {
            return Ok(Classification::Decided(None));
        };
        let sequence = match sturm_sequence(self.polynomial.coefficients(), policy)? {
            Classification::Decided(sequence) => sequence,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.represented_rational_root_with_sequence(policy, denominator_bound, &sequence, None)
    }

    fn represented_rational_root_with_sequence(
        &self,
        policy: &CurvePolicy,
        denominator_bound: BigUint,
        sequence: &[Vec<Real>],
        mut trace: Option<&mut BezierRootIsolationTrace2>,
    ) -> CurveResult<Classification<Option<Real>>> {
        let two = BigInt::from(2_u8);
        let bound = BigInt::from(denominator_bound);
        let target_width = BigRational::new(BigInt::one(), &two * &bound * &bound);
        let mut interval = self.interval.clone();
        loop {
            let Some(start) = real_as_big_rational(interval.start()) else {
                return Ok(Classification::Decided(None));
            };
            let Some(end) = real_as_big_rational(interval.end()) else {
                return Ok(Classification::Decided(None));
            };
            if &end - &start < target_width {
                return reconstruct_rational_root(
                    &self.polynomial,
                    &interval,
                    (&start + &end) / &two,
                    &bound,
                    policy,
                );
            }

            let midpoint = (&start + &end) / &two;
            let midpoint_real = real_from_big_rational(&midpoint)?;
            match real_sign(&self.polynomial.evaluate(&midpoint_real), policy) {
                Some(RealSign::Zero) => {
                    return Ok(Classification::Decided(Some(midpoint_real)));
                }
                Some(_) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let left = match BezierParameterInterval::try_new(
                interval.start().clone(),
                midpoint_real.clone(),
                policy,
            )? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let left_count = match self
                .polynomial
                .root_count_in_interval_with_sequence(&left, sequence, policy)?
            {
                Classification::Decided(count) => count,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if let Some(trace) = trace.as_deref_mut() {
                trace.interval_root_counts += 1;
                trace.rational_reconstruction_refinements += 1;
            }
            if left_count == 1 {
                interval = left;
                continue;
            }
            if left_count != 0 {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            interval = match BezierParameterInterval::try_new(
                midpoint_real,
                interval.end().clone(),
                policy,
            )? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
    }

    fn represented_rational_root_with_cached_sequence(
        &self,
        policy: &CurvePolicy,
        sequence: &[Vec<Real>],
        trace: Option<&mut BezierRootIsolationTrace2>,
    ) -> CurveResult<Classification<Option<Real>>> {
        if self.polynomial.degree() == 1 {
            return self.represented_linear_root(policy);
        }
        let Some(denominator_bound) = rational_root_denominator_bound(&self.polynomial) else {
            return Ok(Classification::Decided(None));
        };
        self.represented_rational_root_with_sequence(policy, denominator_bound, sequence, trace)
    }

    fn represented_linear_root(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<Real>>> {
        let constant = &self.polynomial.coefficients()[0];
        let slope = &self.polynomial.coefficients()[1];
        if is_zero(slope, policy) != Some(false) {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        let root = ((Real::zero() - constant) / slope.clone())?;
        match in_closed_unit_interval(&root, policy) {
            Some(true) => {}
            Some(false) => return Ok(Classification::Decided(None)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        match (
            compare_reals(self.interval.start(), &root, policy),
            compare_reals(&root, self.interval.end(), policy),
        ) {
            (Some(Ordering::Greater), _) | (_, Some(Ordering::Greater)) => {
                return Ok(Classification::Decided(None));
            }
            (Some(_), Some(_)) => {}
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        match real_sign(&self.polynomial.evaluate(&root), policy) {
            Some(RealSign::Zero) => Ok(Classification::Decided(Some(root))),
            Some(_) => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
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

    /// Promotes a rational algebraic parameter to a represented exact value.
    ///
    /// Irrational and nonrational-coefficient parameters remain algebraic.
    /// Promotion occurs only through exact reconstruction and polynomial replay.
    pub fn promote_represented_rational_root(
        self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Exact(_) => Ok(Classification::Decided(self)),
            Self::Algebraic(parameter) => match parameter.represented_rational_root(policy)? {
                Classification::Decided(Some(root)) => {
                    Ok(Classification::Decided(Self::Exact(root)))
                }
                Classification::Decided(None) => {
                    Ok(Classification::Decided(Self::Algebraic(parameter)))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            },
        }
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

    pub(crate) fn strict_rational_between(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Real>> {
        match self.cmp_by_refinement(other, policy)? {
            Classification::Decided(Ordering::Less) => {}
            Classification::Decided(Ordering::Equal | Ordering::Greater) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
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
            return midpoint_real(left.end(), right.start()).map(Classification::Decided);
        }
        match (self, other) {
            (Self::Algebraic(parameter), _) => {
                refine_algebraic_upper_gap(parameter, right.start(), policy)
            }
            (_, Self::Algebraic(parameter)) => {
                refine_algebraic_lower_gap(parameter, left.end(), policy)
            }
            (Self::Exact(_), Self::Exact(_)) => {
                Ok(Classification::Uncertain(UncertaintyReason::Ordering))
            }
        }
    }

    /// Compares parameters when exact values or nonoverlapping isolating intervals prove the order.
    ///
    /// Algebraic isolators certify that their endpoints are not roots, so two
    /// intervals that only share one endpoint still certify a strict order.
    pub fn cmp_by_interval(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Ordering>> {
        if self == other {
            return Ok(Classification::Decided(Ordering::Equal));
        }
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

        if matches!(
            compare_reals(left.end(), right.start(), policy),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Ok(Classification::Decided(Ordering::Less));
        }
        if matches!(
            compare_reals(right.end(), left.start(), policy),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Ok(Classification::Decided(Ordering::Greater));
        }

        match self.same_value(other, policy)? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(Ordering::Equal));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }

    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Ordering>> {
        match self.cmp_by_interval(other, policy)? {
            Classification::Decided(ordering) => {
                return Ok(Classification::Decided(ordering));
            }
            Classification::Uncertain(reason) if reason != UncertaintyReason::Ordering => {
                return Ok(Classification::Uncertain(reason));
            }
            Classification::Uncertain(_) => {}
        }
        match self.same_value(other, policy)? {
            Classification::Decided(true) => Ok(Classification::Decided(Ordering::Equal)),
            Classification::Decided(false) => compare_distinct_parameters(self, other, policy),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn same_value(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        if self == other {
            return Ok(Classification::Decided(true));
        }
        match (self, other) {
            (Self::Exact(left), Self::Exact(right)) => compare_reals(left, right, policy)
                .map(|ordering| Classification::Decided(ordering.is_eq()))
                .map(Ok)
                .unwrap_or_else(|| Ok(Classification::Uncertain(UncertaintyReason::Ordering))),
            (Self::Exact(exact), Self::Algebraic(algebraic))
            | (Self::Algebraic(algebraic), Self::Exact(exact)) => {
                let interval = algebraic.interval();
                let lower = compare_reals(exact, interval.start(), policy);
                let upper = compare_reals(exact, interval.end(), policy);
                match (lower, upper) {
                    (Some(Ordering::Less), _) | (_, Some(Ordering::Greater)) => {
                        Ok(Classification::Decided(false))
                    }
                    (Some(_), Some(_)) => {
                        match real_sign(&algebraic.polynomial().evaluate(exact), policy) {
                            Some(RealSign::Zero) => Ok(Classification::Decided(true)),
                            Some(_) => Ok(Classification::Decided(false)),
                            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                        }
                    }
                    _ => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
            }
            (Self::Algebraic(left), Self::Algebraic(right)) => {
                let start = match compare_reals(
                    left.interval().start(),
                    right.interval().start(),
                    policy,
                ) {
                    Some(Ordering::Less) => right.interval().start().clone(),
                    Some(_) => left.interval().start().clone(),
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                };
                let end = match compare_reals(left.interval().end(), right.interval().end(), policy)
                {
                    Some(Ordering::Greater) => right.interval().end().clone(),
                    Some(_) => left.interval().end().clone(),
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                };
                match compare_reals(&start, &end, policy) {
                    Some(Ordering::Greater | Ordering::Equal) => {
                        return Ok(Classification::Decided(false));
                    }
                    Some(Ordering::Less) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
                let gcd = match left
                    .polynomial()
                    .greatest_common_divisor(right.polynomial(), policy)?
                {
                    Classification::Decided(Some(gcd)) => gcd,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(false));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let interval = match BezierParameterInterval::try_new(start, end, policy)? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match gcd.root_count_in_interval(&interval, policy)? {
                    Classification::Decided(0) => Ok(Classification::Decided(false)),
                    Classification::Decided(1) => Ok(Classification::Decided(true)),
                    Classification::Decided(_) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
        }
    }
}

enum RefinedParameter<'a> {
    Exact(Real),
    Algebraic {
        parameter: &'a BezierAlgebraicParameter2,
        interval: BezierParameterInterval,
    },
}

impl<'a> RefinedParameter<'a> {
    fn from_parameter(parameter: &'a BezierParameter2) -> Self {
        match parameter {
            BezierParameter2::Exact(value) => Self::Exact(value.clone()),
            BezierParameter2::Algebraic(parameter) => Self::Algebraic {
                parameter,
                interval: parameter.interval().clone(),
            },
        }
    }

    fn bounds(&self) -> (&Real, &Real) {
        match self {
            Self::Exact(value) => (value, value),
            Self::Algebraic { interval, .. } => (interval.start(), interval.end()),
        }
    }

    fn refine_once(self, policy: &CurvePolicy) -> CurveResult<Classification<Self>> {
        let Self::Algebraic {
            parameter,
            interval,
        } = self
        else {
            return Ok(Classification::Decided(self));
        };
        let midpoint = midpoint_real(interval.start(), interval.end())?;
        match real_sign(&parameter.polynomial.evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(Self::Exact(midpoint))),
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let interval = match parameter.polynomial.root_count_in_interval(&left, policy)? {
            Classification::Decided(1) => left,
            Classification::Decided(0) => {
                match BezierParameterInterval::try_new(midpoint, interval.end().clone(), policy)? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Classification::Decided(_) => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Self::Algebraic {
            parameter,
            interval,
        }))
    }
}

fn compare_distinct_parameters(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Ordering>> {
    const MAX_ORDERING_REFINEMENTS: usize = 64;

    let mut first = RefinedParameter::from_parameter(first);
    let mut second = RefinedParameter::from_parameter(second);
    for _ in 0..MAX_ORDERING_REFINEMENTS {
        let (first_start, first_end) = first.bounds();
        let (second_start, second_end) = second.bounds();
        if matches!(
            compare_reals(first_end, second_start, policy),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Ok(Classification::Decided(Ordering::Less));
        }
        if matches!(
            compare_reals(second_end, first_start, policy),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Ok(Classification::Decided(Ordering::Greater));
        }

        match (&first, &second) {
            (RefinedParameter::Exact(first), RefinedParameter::Exact(second)) => {
                return Ok(compare_reals(first, second, policy)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)));
            }
            (RefinedParameter::Exact(_), RefinedParameter::Algebraic { .. }) => {
                second = match second.refine_once(policy)? {
                    Classification::Decided(second) => second,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            (RefinedParameter::Algebraic { .. }, RefinedParameter::Exact(_)) => {
                first = match first.refine_once(policy)? {
                    Classification::Decided(first) => first,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            (RefinedParameter::Algebraic { .. }, RefinedParameter::Algebraic { .. }) => {
                first = match first.refine_once(policy)? {
                    Classification::Decided(first) => first,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                second = match second.refine_once(policy)? {
                    Classification::Decided(second) => second,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
        }
    }
    Ok(Classification::Uncertain(UncertaintyReason::Ordering))
}

fn refine_algebraic_upper_gap(
    parameter: &BezierAlgebraicParameter2,
    upper_bound: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    let mut interval = parameter.interval.clone();
    loop {
        if compare_reals(interval.end(), upper_bound, policy) == Some(Ordering::Less) {
            return midpoint_real(interval.end(), upper_bound).map(Classification::Decided);
        }
        let midpoint = midpoint_real(interval.start(), interval.end())?;
        match real_sign(&parameter.polynomial.evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return midpoint_real(&midpoint, upper_bound).map(Classification::Decided);
            }
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match parameter.polynomial.root_count_in_interval(&left, policy)? {
            Classification::Decided(1) => interval = left,
            Classification::Decided(0) => {
                interval = match BezierParameterInterval::try_new(
                    midpoint,
                    interval.end().clone(),
                    policy,
                )? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            Classification::Decided(_) => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
}

fn refine_algebraic_lower_gap(
    parameter: &BezierAlgebraicParameter2,
    lower_bound: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    let mut interval = parameter.interval.clone();
    loop {
        if compare_reals(lower_bound, interval.start(), policy) == Some(Ordering::Less) {
            return midpoint_real(lower_bound, interval.start()).map(Classification::Decided);
        }
        let midpoint = midpoint_real(interval.start(), interval.end())?;
        match real_sign(&parameter.polynomial.evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return midpoint_real(lower_bound, &midpoint).map(Classification::Decided);
            }
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match parameter.polynomial.root_count_in_interval(&left, policy)? {
            Classification::Decided(1) => interval = left,
            Classification::Decided(0) => {
                interval = match BezierParameterInterval::try_new(
                    midpoint,
                    interval.end().clone(),
                    policy,
                )? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            }
            Classification::Decided(_) => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
}

fn midpoint_real(first: &Real, second: &Real) -> CurveResult<Real> {
    ((first + second) / Real::from(2_u8)).map_err(Into::into)
}

impl BezierParameterRange2 {
    /// Constructs a certified positive-length oriented parameter range.
    pub fn try_new(
        start: BezierParameter2,
        end: BezierParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        for boundary in [&start, &end] {
            if let Classification::Uncertain(reason) = boundary.known_interval(policy)? {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match start.cmp_by_interval(&end, policy)? {
            Classification::Decided(Ordering::Equal) => Err(CurveError::InvalidBezierRange),
            Classification::Decided(Ordering::Less | Ordering::Greater) => {
                Ok(Classification::Decided(Self { start, end }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) const fn new_validated(start: BezierParameter2, end: BezierParameter2) -> Self {
        Self { start, end }
    }

    pub(crate) fn from_exact(start: Real, end: Real) -> Self {
        Self::new_validated(BezierParameter2::Exact(start), BezierParameter2::Exact(end))
    }

    /// Returns the oriented start boundary.
    pub const fn start(&self) -> &BezierParameter2 {
        &self.start
    }

    /// Returns the oriented end boundary.
    pub const fn end(&self) -> &BezierParameter2 {
        &self.end
    }

    /// Returns both represented values when neither endpoint is algebraic.
    pub fn exact_endpoints(&self) -> Option<(&Real, &Real)> {
        Some((self.start.as_exact()?, self.end.as_exact()?))
    }

    /// Promotes exactly reconstructible rational endpoints to represented values.
    ///
    /// Irrational algebraic endpoints remain algebraic. Each successful
    /// reconstruction is replayed against its defining polynomial before the
    /// endpoint representation changes.
    pub fn promote_represented_rational_endpoints(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let start = match self
            .start
            .clone()
            .promote_represented_rational_root(policy)?
        {
            Classification::Decided(start) => start,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end = match self.end.clone().promote_represented_rational_root(policy)? {
            Classification::Decided(end) => end,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(Self::new_validated(start, end)))
    }

    /// Returns the same range with traversal reversed.
    pub fn reversed(&self) -> Self {
        Self {
            start: self.end.clone(),
            end: self.start.clone(),
        }
    }
}

impl PartialEq<Real> for BezierParameter2 {
    fn eq(&self, other: &Real) -> bool {
        matches!(self, Self::Exact(value) if value == other)
    }
}

impl PartialEq<BezierParameter2> for Real {
    fn eq(&self, other: &BezierParameter2) -> bool {
        other == self
    }
}

impl PartialEq<crate::ParamRange> for BezierParameterRange2 {
    fn eq(&self, other: &crate::ParamRange) -> bool {
        self.exact_endpoints()
            .is_some_and(|(start, end)| start == other.start() && end == other.end())
    }
}

fn rational_root_denominator_bound(polynomial: &BezierParameterPolynomial) -> Option<BigUint> {
    let rationals = polynomial
        .coefficients()
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let common_denominator = rationals.iter().fold(BigUint::one(), |common, value| {
        common.lcm(value.denominator())
    });
    let mut content = BigUint::zero();
    let mut leading_magnitude = BigUint::zero();
    for rational in rationals {
        let magnitude = rational.numerator() * (&common_denominator / rational.denominator());
        if !magnitude.is_zero() {
            content = if content.is_zero() {
                magnitude.clone()
            } else {
                content.gcd(&magnitude)
            };
        }
        leading_magnitude = magnitude;
    }
    if content.is_zero() || leading_magnitude.is_zero() {
        return None;
    }
    Some(leading_magnitude / content)
}

fn real_as_big_rational(value: &Real) -> Option<BigRational> {
    let rational = value.exact_rational_ref()?;
    Some(BigRational::new(
        BigInt::from_biguint(rational.sign(), rational.numerator().clone()),
        BigInt::from(rational.denominator().clone()),
    ))
}

fn real_from_big_rational(value: &BigRational) -> CurveResult<Real> {
    HyperRational::from_bigint_fraction(value.numer().clone(), value.denom().magnitude().clone())
        .map(Real::new)
        .map_err(Into::into)
}

fn reconstruct_rational_root(
    polynomial: &BezierParameterPolynomial,
    interval: &BezierParameterInterval,
    approximation: BigRational,
    denominator_bound: &BigInt,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Real>>> {
    let Some(interval_start) = real_as_big_rational(interval.start()) else {
        return Ok(Classification::Decided(None));
    };
    let Some(interval_end) = real_as_big_rational(interval.end()) else {
        return Ok(Classification::Decided(None));
    };
    let mut numerator = approximation.numer().clone();
    let mut denominator = approximation.denom().clone();
    let mut previous_numerator = BigInt::zero();
    let mut current_numerator = BigInt::one();
    let mut previous_denominator = BigInt::one();
    let mut current_denominator = BigInt::zero();
    while !denominator.is_zero() {
        let (quotient, remainder) = numerator.div_rem(&denominator);
        let next_numerator = &quotient * &current_numerator + &previous_numerator;
        let next_denominator = &quotient * &current_denominator + &previous_denominator;
        if next_denominator > *denominator_bound {
            break;
        }
        if !next_denominator.is_zero() {
            let candidate = BigRational::new(next_numerator.clone(), next_denominator.clone());
            if candidate >= interval_start && candidate <= interval_end {
                let candidate = real_from_big_rational(&candidate)?;
                match real_sign(&polynomial.evaluate(&candidate), policy) {
                    Some(RealSign::Zero) => {
                        return Ok(Classification::Decided(Some(candidate)));
                    }
                    Some(_) => {}
                    None => {
                        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                    }
                }
            }
        }
        previous_numerator = current_numerator;
        current_numerator = next_numerator;
        previous_denominator = current_denominator;
        current_denominator = next_denominator;
        numerator = denominator;
        denominator = remainder;
    }
    Ok(Classification::Decided(None))
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

enum UnitRootSearch {
    Isolated(Vec<BezierParameter2>),
    RepresentedRoot(Real),
}

fn isolate_unit_roots(
    mut coefficients: Vec<Real>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierRootIsolationResult2>> {
    let mut trace = BezierRootIsolationTrace2::default();
    let mut represented = Vec::new();
    for endpoint in [Real::zero(), Real::one()] {
        let mut found = false;
        loop {
            if coefficients.len() <= 1 {
                break;
            }
            match real_sign(&evaluate_coefficients(&coefficients, &endpoint), policy) {
                Some(RealSign::Zero) => {
                    coefficients = divide_by_linear_root(&coefficients, &endpoint);
                    found = true;
                }
                Some(_) => break,
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        if found {
            represented.push(BezierParameter2::Exact(endpoint));
        }
    }

    loop {
        let polynomial =
            match BezierParameterPolynomial::try_new_power_basis(coefficients.clone(), policy) {
                Ok(Classification::Decided(polynomial)) => polynomial,
                Err(CurveError::InvalidBezierPolynomial) => break,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(error) => return Err(error),
            };
        let represented_boundaries = represented
            .iter()
            .filter_map(BezierParameter2::as_exact)
            .cloned()
            .collect::<Vec<_>>();
        match search_unit_roots(&polynomial, &represented_boundaries, policy, &mut trace)? {
            Classification::Decided(UnitRootSearch::Isolated(mut algebraic)) => {
                represented.append(&mut algebraic);
                break;
            }
            Classification::Decided(UnitRootSearch::RepresentedRoot(root)) => {
                represented.push(BezierParameter2::Exact(root.clone()));
                loop {
                    if coefficients.len() <= 1
                        || real_sign(&evaluate_coefficients(&coefficients, &root), policy)
                            != Some(RealSign::Zero)
                    {
                        break;
                    }
                    coefficients = divide_by_linear_root(&coefficients, &root);
                }
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    let mut ordered = Vec::with_capacity(represented.len());
    for parameter in represented {
        insert_parameter_ordered(&mut ordered, parameter, policy)?;
    }
    Ok(Classification::Decided(BezierRootIsolationResult2 {
        roots: ordered,
        trace,
    }))
}

fn search_unit_roots(
    polynomial: &BezierParameterPolynomial,
    represented_roots: &[Real],
    policy: &CurvePolicy,
    trace: &mut BezierRootIsolationTrace2,
) -> CurveResult<Classification<UnitRootSearch>> {
    let sequence = match sturm_sequence(polynomial.coefficients(), policy)? {
        Classification::Decided(sequence) => sequence,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    trace.sturm_sequence_builds += 1;
    let mut boundaries = vec![Real::zero()];
    for root in represented_roots {
        if compare_reals(root, &Real::zero(), policy) == Some(Ordering::Greater)
            && compare_reals(root, &Real::one(), policy) == Some(Ordering::Less)
        {
            let insert_at = boundaries
                .iter()
                .position(|boundary| {
                    compare_reals(boundary, root, policy) == Some(Ordering::Greater)
                })
                .unwrap_or(boundaries.len());
            if insert_at == 0
                || compare_reals(&boundaries[insert_at - 1], root, policy) != Some(Ordering::Equal)
            {
                boundaries.insert(insert_at, root.clone());
            }
        }
    }
    boundaries.push(Real::one());
    let mut pending = boundaries
        .windows(2)
        .rev()
        .map(|pair| (pair[0].clone(), pair[1].clone(), 0_usize))
        .collect::<Vec<_>>();
    let mut isolated = Vec::new();
    while let Some((start, end, depth)) = pending.pop() {
        trace.maximum_depth = trace.maximum_depth.max(depth);
        let interval = match BezierParameterInterval::try_new(start.clone(), end.clone(), policy)? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let count =
            match polynomial.root_count_in_interval_with_sequence(&interval, &sequence, policy)? {
                Classification::Decided(count) => count,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        trace.interval_root_counts += 1;
        if count == 0 {
            continue;
        }
        let midpoint = ((&start + &end) / Real::from(2_i8))?;
        match real_sign(&polynomial.evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return Ok(Classification::Decided(UnitRootSearch::RepresentedRoot(
                    midpoint,
                )));
            }
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let touches_represented_root = represented_roots.iter().any(|root| {
            compare_reals(root, &start, policy) == Some(Ordering::Equal)
                || compare_reals(root, &end, policy) == Some(Ordering::Equal)
        });
        let touches_domain_endpoint = compare_reals(&start, &Real::zero(), policy)
            == Some(Ordering::Equal)
            || compare_reals(&end, &Real::one(), policy) == Some(Ordering::Equal);
        if count == 1 && !touches_represented_root && !touches_domain_endpoint {
            // `count == 1` above was proved with the cached Sturm sequence.
            // Reusing that certificate avoids rebuilding the identical
            // sequence solely to construct the carrier.
            let parameter =
                BezierAlgebraicParameter2::from_certified_singleton(polynomial.clone(), interval);
            match parameter.represented_rational_root_with_cached_sequence(
                policy,
                &sequence,
                Some(trace),
            )? {
                Classification::Decided(Some(root)) => {
                    return Ok(Classification::Decided(UnitRootSearch::RepresentedRoot(
                        root,
                    )));
                }
                Classification::Decided(None) => {
                    isolated.push(BezierParameter2::Algebraic(parameter));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            continue;
        }
        if depth >= 256 {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        trace.bisections += 1;
        pending.push((midpoint.clone(), end, depth + 1));
        pending.push((start, midpoint, depth + 1));
    }
    Ok(Classification::Decided(UnitRootSearch::Isolated(isolated)))
}

fn insert_parameter_ordered(
    parameters: &mut Vec<BezierParameter2>,
    parameter: BezierParameter2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let mut insert_at = parameters.len();
    for (index, existing) in parameters.iter().enumerate() {
        match existing.cmp_by_interval(&parameter, policy)? {
            Classification::Decided(Ordering::Equal) => return Ok(()),
            Classification::Decided(Ordering::Greater) => {
                insert_at = index;
                break;
            }
            Classification::Decided(Ordering::Less) => {}
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "isolated parameter ordering remained uncertain: {reason:?}; existing={existing:?}; candidate={parameter:?}"
                )));
            }
        }
    }
    parameters.insert(insert_at, parameter);
    Ok(())
}

fn divide_by_linear_root(coefficients: &[Real], root: &Real) -> Vec<Real> {
    let degree = coefficients.len() - 1;
    let mut quotient = vec![Real::zero(); degree];
    quotient[degree - 1] = coefficients[degree].clone();
    for index in (1..degree).rev() {
        quotient[index - 1] = &coefficients[index] + root * &quotient[index];
    }
    quotient
}

pub(crate) fn bernstein_to_power_coefficients(values: Vec<Real>) -> CurveResult<Vec<Real>> {
    let degree = values
        .len()
        .checked_sub(1)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    let mut coefficients = vec![Real::zero(); values.len()];
    for (index, value) in values.into_iter().enumerate() {
        for (power, coefficient) in coefficients
            .iter_mut()
            .enumerate()
            .take(degree + 1)
            .skip(index)
        {
            let magnitude = checked_binomial_u64(degree, index)?
                .checked_mul(checked_binomial_u64(degree - index, power - index)?)
                .ok_or(CurveError::InvalidBezierPolynomial)?;
            let term = &value * &Real::from(magnitude);
            if (power - index).is_multiple_of(2) {
                *coefficient = &*coefficient + term;
            } else {
                *coefficient = &*coefficient - term;
            }
        }
    }
    Ok(coefficients)
}

fn checked_binomial_u64(n: usize, k: usize) -> CurveResult<u64> {
    let k = k.min(n - k);
    (0..k).try_fold(1_u64, |result, index| {
        let numerator =
            u64::try_from(n - index).map_err(|_| CurveError::InvalidBezierPolynomial)?;
        let denominator =
            u64::try_from(index + 1).map_err(|_| CurveError::InvalidBezierPolynomial)?;
        result
            .checked_mul(numerator)
            .map(|value| value / denominator)
            .ok_or(CurveError::InvalidBezierPolynomial)
    })
}
