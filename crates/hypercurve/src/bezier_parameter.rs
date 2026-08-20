//! Exact carriers for Bezier split parameters.
//!
//! Bezier arrangements eventually need split points whose parameters are not
//! represented by the scalar `Real` API yet. This module gives those parameters
//! a first-class exact carrier instead of forcing an approximate collapse: an
//! exact parameter is either a represented [`Real`] or an algebraic root
//! described by a power-basis polynomial and an isolating interval. Public
//! segment parameters remain in `[0, 1]`; retained corner-extension work may
//! carry a finite root on either incident exterior ray without demoting it.
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
use std::sync::Arc;
use std::sync::{Mutex, OnceLock};

use hyperreal::{CertifiedRealSign, Rational as HyperRational, Real, RealSign};
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRepresentation, compare_algebraic_root_representations_by_difference,
    compose_univariate_polynomial_linear_fractional,
};
use num::{BigInt, BigRational, BigUint, Integer, One, ToPrimitive, Zero};

use crate::classify::{compare_reals, in_closed_unit_interval, is_zero, real_sign};
use crate::{
    BezierMonotoneSpan, Classification, CurveContext, CurveError, CurveResult, UncertaintyReason,
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
/// Public construction certifies `[0, 1]` membership and `start <= end`.
/// Internally, exact corner-extension charts may retain an ordered finite
/// interval outside the segment domain. `BezierAlgebraicParameter2`
/// additionally requires the defining polynomial to have no endpoint root and
/// exactly one distinct root in this interval under Sturm validation.
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
#[derive(Clone, Debug)]
pub struct BezierAlgebraicParameter2 {
    data: Arc<BezierAlgebraicParameterData>,
}

#[derive(Debug)]
struct BezierAlgebraicParameterData {
    polynomial: BezierParameterPolynomial,
    interval: BezierParameterInterval,
    root_count: usize,
    shared: Arc<BezierAlgebraicParameterSharedData>,
}

#[derive(Debug, Default)]
struct BezierAlgebraicParameterSharedData {
    represented_rational_root: OnceLock<Option<Real>>,
    sturm_sequence: OnceLock<Arc<[Vec<Real>]>>,
    simple_root: OnceLock<bool>,
    rational_images: Mutex<Vec<RetainedRationalBezierAlgebraicImages>>,
}

#[derive(Debug)]
struct RetainedRationalBezierAlgebraicImages {
    curve: RetainedRationalBezierAlgebraicImageCurve,
    point: Option<crate::RationalBezierAlgebraicPointImage2>,
    derivatives: Arc<[crate::RationalBezierAlgebraicTangentImage2]>,
}

#[derive(Debug)]
enum RetainedRationalBezierAlgebraicImageCurve {
    Rational(crate::RationalBezier2),
    RationalQuadratic(crate::RationalQuadraticBezier2),
}

impl PartialEq for BezierAlgebraicParameter2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.polynomial == other.data.polynomial
                && self.data.interval == other.data.interval
                && self.data.root_count == other.data.root_count)
    }
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

/// Incremental exact isolator refinement for increasing proof budgets.
///
/// Each target is interpreted relative to the original parameter. Reusing the
/// preceding bracket avoids replaying already-certified bisections when a
/// caller progressively asks for 2, 4, then 8 refinement steps.
pub(crate) struct BezierParameterRefinement2<'a> {
    parameter: BezierParameter2,
    completed_steps: usize,
    policy: &'a CurveContext,
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

/// Direction of an affine parameter ray incident to one finite anchor.
///
/// Roots returned for either direction are ordered away from the anchor. This
/// is the order corner editing consumes and avoids a second algebraic sort for
/// the decreasing ray.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierParameterRayDirection2 {
    /// Parameters strictly below the anchor.
    Decreasing,
    /// Parameters strictly above the anchor.
    Increasing,
}

impl BezierParameterPolynomial {
    /// Constructs a nonzero power-basis polynomial.
    pub fn try_new_power_basis(
        coefficients: Vec<Real>,
        policy: &CurveContext,
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
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<Real>>> {
        match polynomial_remainder(coefficients, &self.coefficients, policy)? {
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
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<usize>> {
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let mut first = self.coefficients.clone();
        let mut second = other.coefficients.clone();
        while !second.is_empty() {
            let remainder = match scale_invariant_polynomial_remainder(first, &second, policy)? {
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        Ok(self
            .isolate_unit_interval_roots_with_trace(policy)?
            .map(BezierRootIsolationResult2::into_roots))
    }

    /// Isolates an exactly square-free rational polynomial on `[0, 1]` with
    /// Bernstein--Descartes subdivision before falling back to Sturm replay.
    ///
    /// Callers own the square-free certificate. One Bernstein sign variation
    /// then certifies one simple root, so recursive exact subdivision has no
    /// historical degree cap. A root on a dyadic split or an undecided sign
    /// uses the general complete isolator instead.
    pub(crate) fn isolate_square_free_unit_interval_roots(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        let mut trace = BezierRootIsolationTrace2::default();
        if let Some(roots) =
            exact_rational_square_free_bernstein_unit_roots(self, policy, &mut trace)?
        {
            return ordered_root_isolation_result(roots, trace, policy)
                .map(|result| result.map(BezierRootIsolationResult2::into_roots));
        }
        self.isolate_unit_interval_roots(policy)
    }

    /// Isolates every distinct root in `[0, 1]` and evidence exact work counts.
    pub fn isolate_unit_interval_roots_with_trace(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierRootIsolationResult2>> {
        isolate_unit_roots(self.coefficients.clone(), policy)
    }

    /// Isolates every root on one open ray incident to `anchor`.
    ///
    /// The compact chart `u in (0, 1)` uses
    /// `t = anchor +/- u/(1-u)`. Hypersolve performs the exact homogeneous
    /// polynomial composition, the established unit-interval isolator owns all
    /// root finding, and each root is transported back to the original affine
    /// parameter with the source polynomial retained as its authority.
    pub fn isolate_incident_ray_roots(
        &self,
        anchor: &Real,
        direction: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        let signed_unit = match direction {
            BezierParameterRayDirection2::Decreasing => -Real::one(),
            BezierParameterRayDirection2::Increasing => Real::one(),
        };
        let numerator_scale = &signed_unit - anchor;
        let transformed = match compose_univariate_polynomial_linear_fractional(
            &self.coefficients,
            &numerator_scale,
            anchor,
            &-Real::one(),
            &Real::one(),
            policy.predicate_policy(),
        ) {
            Some(transformed) => transformed,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let transformed = match Self::try_new_power_basis(transformed, policy)? {
            Classification::Decided(transformed) => transformed,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let compact_roots = match transformed.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let zero = BezierParameter2::Exact(Real::zero());
        let one = BezierParameter2::Exact(Real::one());
        let mut roots = Vec::with_capacity(compact_roots.len());
        for compact in compact_roots {
            let after_zero = match compact.cmp_by_refinement(&zero, policy)? {
                Classification::Decided(ordering) => ordering,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before_one = match compact.cmp_by_refinement(&one, policy)? {
                Classification::Decided(ordering) => ordering,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if after_zero != Ordering::Greater || before_one != Ordering::Less {
                continue;
            }
            roots.push(map_compact_incident_ray_root(
                self, compact, anchor, direction,
            )?);
        }
        Ok(Classification::Decided(roots))
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
        policy: &CurveContext,
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

    /// Classifies root multiplicity while sharing square-free work across roots.
    ///
    /// Algebraic roots from one isolation pass have the same defining
    /// polynomial. Computing `gcd(P, P')` once proves every root simple when
    /// the GCD is constant; when repeated roots exist, one retained Sturm
    /// sequence classifies all of their disjoint isolating intervals.
    pub(crate) fn simple_root_classifications(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Vec<Classification<bool>>> {
        enum RepeatedRootEvidence {
            NoDerivative,
            SquareFree,
            Repeated {
                polynomial: BezierParameterPolynomial,
                sturm_sequence: Vec<Vec<Real>>,
            },
            Uncertain(UncertaintyReason),
        }

        let source_derivative_coefficients = derivative_coefficients(&self.coefficients);
        let algebraic_polynomial = parameters.iter().find_map(|parameter| match parameter {
            BezierParameter2::Algebraic(parameter) => Some(parameter.polynomial()),
            BezierParameter2::Exact(_) => None,
        });
        let algebraic_needing_classification =
            parameters.iter().find_map(|parameter| match parameter {
                BezierParameter2::Algebraic(parameter)
                    if parameter.data.shared.simple_root.get() != Some(&true) =>
                {
                    Some(parameter)
                }
                BezierParameter2::Exact(_) | BezierParameter2::Algebraic(_) => None,
            });
        // Unit-interval isolation exactly deflates represented rational roots,
        // including endpoints, before isolating the remaining algebraic roots.
        // Their retained carrier can therefore differ from `self`, while
        // preserving the multiplicity of every root that remains on it.
        if let Some(polynomial) = algebraic_polynomial
            && polynomial != self
        {
            match polynomial_remainder(
                self.coefficients.clone(),
                polynomial.coefficients(),
                policy,
            )? {
                Classification::Decided(None) => {}
                Classification::Decided(Some(_)) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Uncertain(reason) => {
                    return Ok(vec![Classification::Uncertain(reason); parameters.len()]);
                }
            }
        }
        let algebraic_derivative_coefficients = algebraic_polynomial
            .map(|polynomial| derivative_coefficients(polynomial.coefficients()));
        let repeated_evidence = if let Some(algebraic) = algebraic_needing_classification {
            match algebraic.retained_sturm_sequence(policy)? {
                Classification::Decided(sequence) => {
                    let gcd_coefficients = sequence
                        .last()
                        .expect("a Sturm sequence contains its source polynomial");
                    if gcd_coefficients.len() == 1 {
                        RepeatedRootEvidence::SquareFree
                    } else if sequence.len() < 64 {
                        let polynomial = BezierParameterPolynomial {
                            coefficients: gcd_coefficients.clone(),
                        };
                        match sturm_sequence(polynomial.coefficients(), policy)? {
                            Classification::Decided(sturm_sequence) => {
                                RepeatedRootEvidence::Repeated {
                                    polynomial,
                                    sturm_sequence,
                                }
                            }
                            Classification::Uncertain(reason) => {
                                RepeatedRootEvidence::Uncertain(reason)
                            }
                        }
                    } else {
                        // A nonconstant 64th remainder may be the bounded
                        // Sturm builder's last permitted step rather than the
                        // completed gcd. Retain the unbounded classification
                        // path for that high-degree case.
                        match Self::try_new_power_basis(
                            algebraic_derivative_coefficients
                                .as_ref()
                                .expect("an algebraic carrier has a derivative")
                                .clone(),
                            policy,
                        ) {
                            Ok(Classification::Decided(derivative)) => {
                                match algebraic_polynomial
                                    .expect("an algebraic parameter retains its carrier")
                                    .greatest_common_divisor(&derivative, policy)?
                                {
                                    Classification::Decided(Some(polynomial)) => {
                                        match sturm_sequence(polynomial.coefficients(), policy)? {
                                            Classification::Decided(sturm_sequence) => {
                                                RepeatedRootEvidence::Repeated {
                                                    polynomial,
                                                    sturm_sequence,
                                                }
                                            }
                                            Classification::Uncertain(reason) => {
                                                RepeatedRootEvidence::Uncertain(reason)
                                            }
                                        }
                                    }
                                    Classification::Decided(None) => {
                                        RepeatedRootEvidence::SquareFree
                                    }
                                    Classification::Uncertain(reason) => {
                                        RepeatedRootEvidence::Uncertain(reason)
                                    }
                                }
                            }
                            Err(CurveError::InvalidBezierPolynomial) => {
                                RepeatedRootEvidence::NoDerivative
                            }
                            Ok(Classification::Uncertain(reason)) => {
                                RepeatedRootEvidence::Uncertain(reason)
                            }
                            Err(error) => return Err(error),
                        }
                    }
                }
                Classification::Uncertain(reason) => RepeatedRootEvidence::Uncertain(reason),
            }
        } else {
            RepeatedRootEvidence::SquareFree
        };

        let mut classifications = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            let classification = match parameter {
                BezierParameter2::Exact(root) => {
                    if real_sign(&self.evaluate(root), policy) != Some(RealSign::Zero) {
                        return Err(CurveError::InvalidBezierParameter);
                    }
                    match real_sign(
                        &evaluate_coefficients(&source_derivative_coefficients, root),
                        policy,
                    ) {
                        Some(RealSign::Positive | RealSign::Negative) => {
                            Classification::Decided(true)
                        }
                        Some(RealSign::Zero) => Classification::Decided(false),
                        None => Classification::Uncertain(UncertaintyReason::RealSign),
                    }
                }
                BezierParameter2::Algebraic(parameter) => {
                    if Some(parameter.polynomial()) != algebraic_polynomial {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    if parameter.data.shared.simple_root.get() == Some(&true) {
                        classifications.push(Classification::Decided(true));
                        continue;
                    }
                    match &repeated_evidence {
                        RepeatedRootEvidence::NoDerivative => Classification::Decided(false),
                        RepeatedRootEvidence::SquareFree => Classification::Decided(true),
                        RepeatedRootEvidence::Repeated {
                            polynomial,
                            sturm_sequence,
                        } => polynomial
                            .root_count_in_interval_with_sequence(
                                parameter.interval(),
                                sturm_sequence,
                                policy,
                            )?
                            .map(|count| count == 0),
                        RepeatedRootEvidence::Uncertain(reason) => {
                            Classification::Uncertain(*reason)
                        }
                    }
                }
            };
            classifications.push(classification);
        }
        Ok(classifications)
    }

    /// Returns the sign immediately after an odd-multiplicity root.
    ///
    /// `None` denotes an even-multiplicity non-crossing root. Represented roots
    /// are divided out exactly; algebraic roots use the certified signs at the
    /// isolating interval boundaries.
    pub(crate) fn sign_after_crossing_root(
        &self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RealSign>>> {
        match parameter {
            BezierParameter2::Exact(root) => {
                let mut coefficients = self.coefficients.clone();
                let mut multiplicity = 0_usize;
                let residual_sign = loop {
                    match real_sign(&evaluate_coefficients(&coefficients, root), policy) {
                        Some(RealSign::Zero) if coefficients.len() > 1 => {
                            multiplicity += 1;
                            coefficients = divide_by_linear_root(&coefficients, root);
                        }
                        Some(sign @ (RealSign::Positive | RealSign::Negative)) => break sign,
                        Some(RealSign::Zero) => {
                            return Err(CurveError::InvalidBezierParameter);
                        }
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                        }
                    }
                };
                if multiplicity == 0 {
                    return Err(CurveError::InvalidBezierParameter);
                }
                Ok(Classification::Decided(
                    (!multiplicity.is_multiple_of(2)).then_some(residual_sign),
                ))
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
                    Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
                let end = match real_sign(&self.evaluate(parameter.interval().end()), policy) {
                    Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                    Some(RealSign::Zero) => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
                Ok(Classification::Decided((start != end).then_some(end)))
            }
        }
    }
}

fn map_compact_incident_ray_root(
    source: &BezierParameterPolynomial,
    compact: BezierParameter2,
    anchor: &Real,
    direction: BezierParameterRayDirection2,
) -> CurveResult<BezierParameter2> {
    let map = |parameter: &Real| -> CurveResult<Real> {
        let distance = (parameter / (Real::one() - parameter))?;
        Ok(match direction {
            BezierParameterRayDirection2::Decreasing => anchor - distance,
            BezierParameterRayDirection2::Increasing => anchor + distance,
        })
    };
    match compact {
        BezierParameter2::Exact(parameter) => Ok(BezierParameter2::Exact(map(&parameter)?)),
        BezierParameter2::Algebraic(parameter) => {
            let first = map(parameter.interval().start())?;
            let second = map(parameter.interval().end())?;
            let interval = match direction {
                BezierParameterRayDirection2::Decreasing => BezierParameterInterval {
                    start: second,
                    end: first,
                },
                BezierParameterRayDirection2::Increasing => BezierParameterInterval {
                    start: first,
                    end: second,
                },
            };
            let mapped =
                BezierAlgebraicParameter2::from_certified_singleton(source.clone(), interval);
            if parameter.data.shared.simple_root.get() == Some(&true) {
                let _ = mapped.data.shared.simple_root.set(true);
            }
            Ok(BezierParameter2::Algebraic(mapped))
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let in_start = in_closed_unit_interval(&start, policy);
        let in_end = in_closed_unit_interval(&end, policy);
        match (in_start, in_end) {
            (Some(false), _) | (_, Some(false)) => return Err(CurveError::InvalidBezierParameter),
            (Some(true), Some(true)) => {}
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        Self::try_new_ordered(start, end, policy)
    }

    fn try_new_ordered(
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match compare_reals(&start, &end, policy) {
            Some(Ordering::Greater) => Err(CurveError::InvalidBezierRange),
            Some(_) => Ok(Classification::Decided(Self { start, end })),
            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }

    /// Converts an existing monotone span into a validated parameter interval.
    pub fn from_monotone_span(
        span: &BezierMonotoneSpan,
        policy: &CurveContext,
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

    pub(crate) fn unit_complement(&self) -> Self {
        Self {
            start: Real::one() - &self.end,
            end: Real::one() - &self.start,
        }
    }
}

impl BezierAlgebraicParameter2 {
    /// Validates a singleton algebraic Bezier parameter isolator.
    pub fn try_isolate(
        polynomial: BezierParameterPolynomial,
        interval: BezierParameterInterval,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let count = match polynomial.root_count_in_interval(&interval, policy)? {
            Classification::Decided(count) => count,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if count != 1 {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }

        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicParameterData {
                polynomial,
                interval,
                root_count: count,
                shared: Arc::new(BezierAlgebraicParameterSharedData::default()),
            }),
        }))
    }

    pub(crate) fn from_certified_singleton(
        polynomial: BezierParameterPolynomial,
        interval: BezierParameterInterval,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicParameterData {
                polynomial,
                interval,
                root_count: 1,
                shared: Arc::new(BezierAlgebraicParameterSharedData::default()),
            }),
        }
    }

    fn from_certified_simple_singleton(
        polynomial: BezierParameterPolynomial,
        interval: BezierParameterInterval,
    ) -> Self {
        let parameter = Self::from_certified_singleton(polynomial, interval);
        let _ = parameter.data.shared.simple_root.set(true);
        parameter
    }

    /// Constructs a simple singleton whose local root count was certified by
    /// an equivalent exact eliminant (for example, one Bernstein sign
    /// variation on this interval).
    ///
    /// Only structurally explicit trailing zeros are removed here. The caller's
    /// local certificate proves that the retained leading coefficient is
    /// nonzero and that the polynomial has exactly one simple root, so no
    /// second global degree/sign pass is needed.
    pub(crate) fn from_certified_simple_power_basis(
        mut coefficients: Vec<Real>,
        interval: BezierParameterInterval,
    ) -> Option<Self> {
        while coefficients.len() > 1 && coefficients.last().is_some_and(Real::definitely_zero) {
            coefficients.pop();
        }
        (coefficients.len() > 1).then(|| {
            Self::from_certified_simple_singleton(
                BezierParameterPolynomial { coefficients },
                interval,
            )
        })
    }

    fn from_certified_singleton_with_sturm_sequence(
        polynomial: BezierParameterPolynomial,
        interval: BezierParameterInterval,
        sturm_sequence: Arc<[Vec<Real>]>,
    ) -> Self {
        let parameter = Self::from_certified_singleton(polynomial, interval);
        let _ = parameter.data.shared.sturm_sequence.set(sturm_sequence);
        parameter
    }

    fn with_certified_interval(&self, interval: BezierParameterInterval) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicParameterData {
                polynomial: self.data.polynomial.clone(),
                interval,
                root_count: self.data.root_count,
                shared: Arc::clone(&self.data.shared),
            }),
        }
    }

    /// Returns the defining polynomial.
    pub fn polynomial(&self) -> &BezierParameterPolynomial {
        &self.data.polynomial
    }

    /// Returns the certified isolating interval.
    pub fn interval(&self) -> &BezierParameterInterval {
        &self.data.interval
    }

    /// Returns the certified distinct-root count for the interval.
    pub fn root_count(&self) -> usize {
        self.data.root_count
    }

    fn retained_sturm_sequence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<[Vec<Real>]>>> {
        if let Some(sequence) = self.data.shared.sturm_sequence.get() {
            return Ok(Classification::Decided(Arc::clone(sequence)));
        }
        let sequence = match sturm_sequence(self.polynomial().coefficients(), policy)? {
            Classification::Decided(sequence) => Arc::<[Vec<Real>]>::from(sequence),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let _ = self.data.shared.sturm_sequence.set(Arc::clone(&sequence));
        Ok(Classification::Decided(
            self.data
                .shared
                .sturm_sequence
                .get()
                .map_or(sequence, Arc::clone),
        ))
    }

    pub(crate) fn cached_rational_bezier_point_image(
        &self,
        curve: &crate::RationalBezier2,
    ) -> Option<crate::RationalBezierAlgebraicPointImage2> {
        self.data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned")
            .iter()
            .find(|images| {
                matches!(
                    &images.curve,
                    RetainedRationalBezierAlgebraicImageCurve::Rational(cached)
                        if cached == curve
                )
            })
            .and_then(|images| images.point.clone())
    }

    pub(crate) fn retain_rational_bezier_point_image(
        &self,
        curve: &crate::RationalBezier2,
        image: crate::RationalBezierAlgebraicPointImage2,
    ) {
        let mut cache = self
            .data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned");
        if let Some(images) = cache.iter_mut().find(|images| {
            matches!(
                &images.curve,
                RetainedRationalBezierAlgebraicImageCurve::Rational(cached)
                    if cached == curve
            )
        }) {
            images.point = Some(image);
            return;
        }
        cache.push(RetainedRationalBezierAlgebraicImages {
            curve: RetainedRationalBezierAlgebraicImageCurve::Rational(curve.clone()),
            point: Some(image),
            derivatives: Arc::from([]),
        });
    }

    pub(crate) fn cached_rational_bezier_derivative_images(
        &self,
        curve: &crate::RationalBezier2,
        max_order: usize,
    ) -> Option<Vec<crate::RationalBezierAlgebraicTangentImage2>> {
        self.data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned")
            .iter()
            .find(|images| {
                matches!(
                    &images.curve,
                    RetainedRationalBezierAlgebraicImageCurve::Rational(cached)
                        if cached == curve
                ) && images.derivatives.len() >= max_order
            })
            .map(|images| images.derivatives[..max_order].to_vec())
    }

    pub(crate) fn retain_rational_bezier_derivative_images(
        &self,
        curve: &crate::RationalBezier2,
        images: Vec<crate::RationalBezierAlgebraicTangentImage2>,
    ) {
        let mut cache = self
            .data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned");
        if let Some(cached) = cache.iter_mut().find(|cached| {
            matches!(
                &cached.curve,
                RetainedRationalBezierAlgebraicImageCurve::Rational(cached_curve)
                    if cached_curve == curve
            )
        }) {
            if cached.derivatives.len() < images.len() {
                cached.derivatives = Arc::from(images);
            }
            return;
        }
        cache.push(RetainedRationalBezierAlgebraicImages {
            curve: RetainedRationalBezierAlgebraicImageCurve::Rational(curve.clone()),
            point: None,
            derivatives: Arc::from(images),
        });
    }

    pub(crate) fn cached_rational_quadratic_point_image(
        &self,
        curve: &crate::RationalQuadraticBezier2,
    ) -> Option<crate::RationalBezierAlgebraicPointImage2> {
        self.data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned")
            .iter()
            .find(|images| {
                matches!(
                    &images.curve,
                    RetainedRationalBezierAlgebraicImageCurve::RationalQuadratic(cached)
                        if cached == curve
                )
            })
            .and_then(|images| images.point.clone())
    }

    pub(crate) fn retain_rational_quadratic_point_image(
        &self,
        curve: &crate::RationalQuadraticBezier2,
        image: crate::RationalBezierAlgebraicPointImage2,
    ) {
        let mut cache = self
            .data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned");
        if let Some(images) = cache.iter_mut().find(|images| {
            matches!(
                &images.curve,
                RetainedRationalBezierAlgebraicImageCurve::RationalQuadratic(cached)
                    if cached == curve
            )
        }) {
            images.point = Some(image);
            return;
        }
        cache.push(RetainedRationalBezierAlgebraicImages {
            curve: RetainedRationalBezierAlgebraicImageCurve::RationalQuadratic(curve.clone()),
            point: Some(image),
            derivatives: Arc::from([]),
        });
    }

    pub(crate) fn cached_rational_quadratic_derivative_images(
        &self,
        curve: &crate::RationalQuadraticBezier2,
        max_order: usize,
    ) -> Option<Vec<crate::RationalBezierAlgebraicTangentImage2>> {
        self.data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned")
            .iter()
            .find(|images| {
                matches!(
                    &images.curve,
                    RetainedRationalBezierAlgebraicImageCurve::RationalQuadratic(cached)
                        if cached == curve
                ) && images.derivatives.len() >= max_order
            })
            .map(|images| images.derivatives[..max_order].to_vec())
    }

    pub(crate) fn retain_rational_quadratic_derivative_images(
        &self,
        curve: &crate::RationalQuadraticBezier2,
        images: Vec<crate::RationalBezierAlgebraicTangentImage2>,
    ) {
        let mut cache = self
            .data
            .shared
            .rational_images
            .lock()
            .expect("algebraic parameter image cache mutex poisoned");
        if let Some(cached) = cache.iter_mut().find(|cached| {
            matches!(
                &cached.curve,
                RetainedRationalBezierAlgebraicImageCurve::RationalQuadratic(cached_curve)
                    if cached_curve == curve
            )
        }) {
            if cached.derivatives.len() < images.len() {
                cached.derivatives = Arc::from(images);
            }
            return;
        }
        cache.push(RetainedRationalBezierAlgebraicImages {
            curve: RetainedRationalBezierAlgebraicImageCurve::RationalQuadratic(curve.clone()),
            point: None,
            derivatives: Arc::from(images),
        });
    }

    /// Returns the represented root when this isolator contains an exact rational root.
    ///
    /// Exact-rational coefficients are cleared to a primitive integer
    /// polynomial. Small-prime reductions first reject polynomials that cannot
    /// have any rational root. Otherwise the rational-root theorem bounds the
    /// reduced denominator by the leading coefficient, and the retained Sturm
    /// isolator is refined until rational reconstruction is unique under that
    /// bound. Continued-fraction candidates are accepted only after exact
    /// polynomial replay. Nonrational coefficients and irrational roots return
    /// `None` without demoting the algebraic carrier.
    #[inline]
    pub fn represented_rational_root(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        if let Some(root) = self.data.shared.represented_rational_root.get() {
            return Ok(Classification::Decided(root.clone()));
        }
        self.compute_represented_rational_root(policy)
    }

    fn compute_represented_rational_root(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let result = if self.data.polynomial.degree() == 1 {
            self.represented_linear_root(policy)?
        } else {
            let Some(denominator_bound) = rational_root_denominator_bound(&self.data.polynomial)
            else {
                return self.cache_represented_rational_root(None);
            };
            let sequence = match sturm_sequence(self.data.polynomial.coefficients(), policy)? {
                Classification::Decided(sequence) => sequence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            self.represented_rational_root_with_sequence(
                policy,
                denominator_bound,
                &sequence,
                None,
            )?
        };
        self.cache_decided_represented_rational_root(result)
    }

    fn represented_rational_root_with_sequence(
        &self,
        policy: &CurveContext,
        denominator_bound: BigUint,
        sequence: &[Vec<Real>],
        mut trace: Option<&mut BezierRootIsolationTrace2>,
    ) -> CurveResult<Classification<Option<Real>>> {
        let two = BigInt::from(2_u8);
        let bound = BigInt::from(denominator_bound);
        let target_width = BigRational::new(BigInt::one(), &two * &bound * &bound);
        let mut interval = self.data.interval.clone();
        loop {
            let Some(start) = real_as_big_rational(interval.start()) else {
                return Ok(Classification::Decided(None));
            };
            let Some(end) = real_as_big_rational(interval.end()) else {
                return Ok(Classification::Decided(None));
            };
            if &end - &start < target_width {
                return reconstruct_rational_root(
                    &self.data.polynomial,
                    &interval,
                    (&start + &end) / &two,
                    &bound,
                    policy,
                );
            }

            let midpoint = (&start + &end) / &two;
            let midpoint_real = real_from_big_rational(&midpoint)?;
            match real_sign(&self.data.polynomial.evaluate(&midpoint_real), policy) {
                Some(RealSign::Zero) => {
                    return Ok(Classification::Decided(Some(midpoint_real)));
                }
                Some(_) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let left = match BezierParameterInterval::try_new_ordered(
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
                .polynomial()
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
            interval = match BezierParameterInterval::try_new_ordered(
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
        policy: &CurveContext,
        denominator_bound: Option<&BigUint>,
        sequence: &[Vec<Real>],
        trace: Option<&mut BezierRootIsolationTrace2>,
    ) -> CurveResult<Classification<Option<Real>>> {
        if let Some(root) = self.data.shared.represented_rational_root.get() {
            return Ok(Classification::Decided(root.clone()));
        }
        if self.data.polynomial.degree() == 1 {
            let result = self.represented_linear_root(policy)?;
            return self.cache_decided_represented_rational_root(result);
        }
        let Some(denominator_bound) = denominator_bound else {
            return self.cache_represented_rational_root(None);
        };
        let result = self.represented_rational_root_with_sequence(
            policy,
            denominator_bound.clone(),
            sequence,
            trace,
        )?;
        self.cache_decided_represented_rational_root(result)
    }

    fn cache_decided_represented_rational_root(
        &self,
        result: Classification<Option<Real>>,
    ) -> CurveResult<Classification<Option<Real>>> {
        if let Classification::Decided(root) = &result {
            let _ = self.data.shared.represented_rational_root.set(root.clone());
        }
        Ok(result)
    }

    fn cache_represented_rational_root(
        &self,
        root: Option<Real>,
    ) -> CurveResult<Classification<Option<Real>>> {
        let _ = self.data.shared.represented_rational_root.set(root.clone());
        Ok(Classification::Decided(root))
    }

    fn represented_linear_root(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Real>>> {
        let constant = &self.data.polynomial.coefficients()[0];
        let slope = &self.data.polynomial.coefficients()[1];
        if is_zero(slope, policy) != Some(false) {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        let root = ((Real::zero() - constant) / slope.clone())?;
        match (
            compare_reals(self.data.interval.start(), &root, policy),
            compare_reals(&root, self.data.interval.end(), policy),
        ) {
            (Some(Ordering::Greater), _) | (_, Some(Ordering::Greater)) => {
                return Ok(Classification::Decided(None));
            }
            (Some(_), Some(_)) => {}
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        match real_sign(&self.data.polynomial.evaluate(&root), policy) {
            Some(RealSign::Zero) => Ok(Classification::Decided(Some(root))),
            Some(_) => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
}

impl BezierParameter2 {
    /// Constructs a represented exact Bezier parameter.
    pub fn exact(value: Real, policy: &CurveContext) -> CurveResult<Classification<Self>> {
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

    pub(crate) fn unit_complement(&self) -> Self {
        match self {
            Self::Exact(value) => Self::Exact(Real::one() - value),
            Self::Algebraic(parameter) => {
                // Horner composition with `(1 - t)` preserves the leading
                // coefficient structurally while avoiding a second temporary
                // coefficient basis. The affine bijection preserves the
                // certified singleton root and reverses its isolating interval.
                let polynomial = parameter.polynomial();
                let polynomial = BezierParameterPolynomial {
                    coefficients: unit_complement_power_coefficients(polynomial.coefficients()),
                };
                let interval = BezierParameterInterval {
                    start: Real::one() - parameter.interval().end(),
                    end: Real::one() - parameter.interval().start(),
                };
                let complemented =
                    BezierAlgebraicParameter2::from_certified_singleton(polynomial, interval);
                if parameter.data.shared.simple_root.get() == Some(&true) {
                    let _ = complemented.data.shared.simple_root.set(true);
                }
                Self::Algebraic(complemented)
            }
        }
    }

    /// Applies `scale * parameter + offset` while retaining an exact algebraic
    /// root carrier on any finite affine interval.
    pub(crate) fn affine_image_unbounded(
        &self,
        scale: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let scale_sign = match real_sign(scale, policy) {
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Some(RealSign::Zero) => return Err(CurveError::InvalidBezierRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let parameter = match self {
            Self::Exact(parameter) => {
                return Ok(Classification::Decided(Self::Exact(
                    scale * parameter + offset,
                )));
            }
            Self::Algebraic(parameter) => parameter,
        };
        let coefficients = match compose_univariate_polynomial_linear_fractional(
            parameter.polynomial().coefficients(),
            &Real::one(),
            &-offset.clone(),
            &Real::zero(),
            scale,
            policy.predicate_policy(),
        ) {
            Some(coefficients) => coefficients,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)?
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let first = scale * parameter.interval().start() + offset;
        let second = scale * parameter.interval().end() + offset;
        let interval = match scale_sign {
            RealSign::Positive => BezierParameterInterval {
                start: first,
                end: second,
            },
            RealSign::Negative => BezierParameterInterval {
                start: second,
                end: first,
            },
            RealSign::Zero => unreachable!(),
        };
        let mapped = BezierAlgebraicParameter2::from_certified_singleton(polynomial, interval);
        if parameter.data.shared.simple_root.get() == Some(&true) {
            let _ = mapped.data.shared.simple_root.set(true);
        }
        Ok(Classification::Decided(Self::Algebraic(mapped)))
    }

    /// Maps an exterior parameter on the ray incident to `anchor` back into
    /// the compact coordinate used by [`BezierParameterPolynomial::isolate_incident_ray_roots`].
    ///
    /// For `direction_sign = +/-1`, the exact inverse of
    /// `t = anchor + direction_sign*x/(1-x)` is
    /// `x = (t-anchor)/(t-anchor+direction_sign)`. The input is already
    /// certified to lie on that open ray, so its denominator cannot vanish.
    /// Algebraic inputs retain an exact transformed polynomial and isolator;
    /// no floating approximation becomes construction evidence.
    pub(crate) fn incident_ray_compact_parameter(
        &self,
        anchor: &Real,
        direction: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let direction_sign = match direction {
            BezierParameterRayDirection2::Decreasing => -Real::one(),
            BezierParameterRayDirection2::Increasing => Real::one(),
        };
        let map = |parameter: &Real| -> CurveResult<Real> {
            ((parameter - anchor) / (parameter - anchor + &direction_sign))
                .map_err(CurveError::from)
        };
        let parameter = match self {
            Self::Exact(parameter) => {
                return Self::exact(map(parameter)?, policy);
            }
            Self::Algebraic(parameter) => parameter,
        };

        // Q(x) = (1-x)^degree P(anchor + direction*x/(1-x)).
        let coefficients = match compose_univariate_polynomial_linear_fractional(
            parameter.polynomial().coefficients(),
            &(&direction_sign - anchor),
            anchor,
            &-Real::one(),
            &Real::one(),
            policy.predicate_policy(),
        ) {
            Some(coefficients) => coefficients,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)?
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let first = map(parameter.interval().start())?;
        let second = map(parameter.interval().end())?;
        let (start, end) = match direction {
            BezierParameterRayDirection2::Increasing => (first, second),
            BezierParameterRayDirection2::Decreasing => (second, first),
        };
        let interval = match BezierParameterInterval::try_new(start, end, policy)? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mapped = BezierAlgebraicParameter2::from_certified_singleton(polynomial, interval);
        if parameter.data.shared.simple_root.get() == Some(&true) {
            let _ = mapped.data.shared.simple_root.set(true);
        }
        Ok(Classification::Decided(Self::Algebraic(mapped)))
    }

    /// Maps a compact incident-ray coordinate back to its original affine
    /// parameter while retaining exact algebraic evidence.
    pub(crate) fn incident_ray_parameter(
        &self,
        anchor: &Real,
        direction: BezierParameterRayDirection2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let direction_sign = match direction {
            BezierParameterRayDirection2::Decreasing => -Real::one(),
            BezierParameterRayDirection2::Increasing => Real::one(),
        };
        let map = |parameter: &Real| -> CurveResult<Real> {
            let distance = (parameter / (Real::one() - parameter))?;
            Ok(anchor + &direction_sign * distance)
        };
        let parameter = match self {
            Self::Exact(parameter) => {
                return Ok(Classification::Decided(Self::Exact(map(parameter)?)));
            }
            Self::Algebraic(parameter) => parameter,
        };

        // If x=(t-anchor)/(t-anchor+direction), inverse substitution gives
        // the exact defining polynomial for t.
        let coefficients = match compose_univariate_polynomial_linear_fractional(
            parameter.polynomial().coefficients(),
            &Real::one(),
            &-anchor.clone(),
            &Real::one(),
            &(&direction_sign - anchor),
            policy.predicate_policy(),
        ) {
            Some(coefficients) => coefficients,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)?
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let first = map(parameter.interval().start())?;
        let second = map(parameter.interval().end())?;
        let (start, end) = match direction {
            BezierParameterRayDirection2::Increasing => (first, second),
            BezierParameterRayDirection2::Decreasing => (second, first),
        };
        let interval = match BezierParameterInterval::try_new_ordered(start, end, policy)? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mapped = BezierAlgebraicParameter2::from_certified_singleton(polynomial, interval);
        if parameter.data.shared.simple_root.get() == Some(&true) {
            let _ = mapped.data.shared.simple_root.set(true);
        }
        Ok(Classification::Decided(Self::Algebraic(mapped)))
    }

    /// Promotes a rational algebraic parameter to a represented exact value.
    ///
    /// Irrational and nonrational-coefficient parameters remain algebraic.
    /// Promotion occurs only through exact reconstruction and polynomial replay.
    pub fn promote_represented_rational_root(
        self,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameterInterval>> {
        match self {
            // Public exact-parameter construction validates the authored
            // segment domain. Internal corner-extension and incident charts
            // also use this representation for finite affine parameters
            // outside `[0, 1]`; an enclosure query must preserve those values
            // rather than reapplying the public-domain constraint.
            Self::Exact(value) => {
                BezierParameterInterval::try_new_ordered(value.clone(), value.clone(), policy)
            }
            Self::Algebraic(value) => Ok(Classification::Decided(value.interval().clone())),
        }
    }

    pub(crate) fn strict_rational_between(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
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
        match self.cmp_by_refinement(other, policy)? {
            Classification::Decided(Ordering::Less) => {}
            Classification::Decided(Ordering::Equal | Ordering::Greater) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        strict_rational_between_known_order(self, other, &left, &right, policy)
    }

    pub(crate) fn strict_rational_between_ordered(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
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
        let mut left_refinement = BezierParameterRefinement2::new(self, policy);
        let mut right_refinement = BezierParameterRefinement2::new(other, policy);
        for refinement_steps in [1, 2, 4, 8] {
            let refined_left = left_refinement.refine_to(refinement_steps);
            let refined_right = right_refinement.refine_to(refinement_steps);
            let Classification::Decided(left) = refined_left.known_interval(policy)? else {
                continue;
            };
            let Classification::Decided(right) = refined_right.known_interval(policy)? else {
                continue;
            };
            if compare_reals(left.end(), right.start(), policy) == Some(Ordering::Less) {
                return midpoint_real(left.end(), right.start()).map(Classification::Decided);
            }
        }
        strict_rational_between_known_order(self, other, &left, &right, policy)
    }
}

fn unit_complement_power_coefficients(coefficients: &[Real]) -> Vec<Real> {
    let mut transformed = vec![
        coefficients
            .last()
            .expect("a Bezier parameter polynomial is nonempty")
            .clone(),
    ];
    for coefficient in coefficients[..coefficients.len() - 1].iter().rev() {
        let previous_len = transformed.len();
        transformed.push(-transformed[previous_len - 1].clone());
        for power in (1..previous_len).rev() {
            transformed[power] = &transformed[power] - &transformed[power - 1];
        }
        transformed[0] += coefficient;
    }
    transformed
}

fn strict_rational_between_known_order(
    left_parameter: &BezierParameter2,
    right_parameter: &BezierParameter2,
    left: &BezierParameterInterval,
    right: &BezierParameterInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    match (left_parameter, right_parameter) {
        (BezierParameter2::Algebraic(parameter), _) => {
            refine_algebraic_upper_gap(parameter, right.start(), policy)
        }
        (_, BezierParameter2::Algebraic(parameter)) => {
            refine_algebraic_lower_gap(parameter, left.end(), policy)
        }
        (BezierParameter2::Exact(_), BezierParameter2::Exact(_)) => {
            Ok(Classification::Uncertain(UncertaintyReason::Ordering))
        }
    }
}

impl BezierParameter2 {
    pub(crate) fn from_algebraic_root_representation(
        representation: &AlgebraicRootRepresentation,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Self::from_algebraic_root_representation_with_domain(representation, true, policy)
    }

    /// Retains a finite algebraic parameter without imposing the authored
    /// Bezier segment domain. Incident-cell corner operations use this after a
    /// separate exact pole/regularity certificate has selected the affine
    /// continuation containing the authored endpoint.
    pub(crate) fn from_algebraic_root_representation_unbounded(
        representation: &AlgebraicRootRepresentation,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Self::from_algebraic_root_representation_with_domain(representation, false, policy)
    }

    fn from_algebraic_root_representation_with_domain(
        representation: &AlgebraicRootRepresentation,
        unit_domain: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if !representation.is_valid() || representation.interval.distinct_root_count != 1 {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        if let Some(exact) = representation.exact_rational_witness() {
            return if unit_domain {
                Self::exact(exact.clone(), policy)
            } else {
                Ok(Classification::Decided(Self::Exact(exact.clone())))
            };
        }
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(
            representation.polynomial_coefficients.clone(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let interval = match if unit_domain {
            BezierParameterInterval::try_new(
                representation.interval.lower.clone(),
                representation.interval.upper.clone(),
                policy,
            )
        } else {
            BezierParameterInterval::try_new_ordered(
                representation.interval.lower.clone(),
                representation.interval.upper.clone(),
                policy,
            )
        }? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Self::Algebraic(
            BezierAlgebraicParameter2::from_certified_singleton(polynomial, interval),
        )))
    }

    pub(crate) fn refined_isolating_interval(
        self,
        max_refinement_steps: usize,
        policy: &CurveContext,
    ) -> Self {
        if max_refinement_steps == 0 {
            return self;
        }
        let Self::Algebraic(algebraic) = self else {
            return self;
        };
        if let Some(refined) =
            refine_algebraic_sign_change(&algebraic, max_refinement_steps, policy)
        {
            return refined;
        }
        let sturm_sequence = match algebraic.retained_sturm_sequence(policy) {
            Ok(Classification::Decided(sequence)) => sequence,
            Ok(Classification::Uncertain(_)) | Err(_) => return Self::Algebraic(algebraic),
        };
        let mut refined = RefinedParameter::Algebraic {
            parameter: &algebraic,
            interval: algebraic.interval().clone(),
            sturm_sequence,
        };
        for _ in 0..max_refinement_steps {
            refined = match refined.refine_once(policy) {
                Ok(Classification::Decided(refined)) => refined,
                Ok(Classification::Uncertain(_)) | Err(_) => {
                    return Self::Algebraic(algebraic);
                }
            };
            if matches!(refined, RefinedParameter::Exact(_)) {
                break;
            }
        }
        match refined {
            RefinedParameter::Exact(exact) => Self::Exact(exact),
            RefinedParameter::Algebraic { interval, .. } => {
                Self::Algebraic(algebraic.with_certified_interval(interval))
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
        policy: &CurveContext,
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

        if let Some(ordering) = disjoint_parameter_interval_order(&left, &right, policy) {
            return Ok(Classification::Decided(ordering));
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

    /// Compares parameters by refining overlapping algebraic isolators as needed.
    ///
    /// This first uses the retained intervals and exact equality evidence. When
    /// distinct parameters still have overlapping bounds, it bisects their
    /// singleton isolators until their order is certified.
    pub fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        if policy.permits_approximate_512() {
            match self.cmp_by_refinement_once(other, &policy.strict_counterpart())? {
                Classification::Decided(ordering) => {
                    return Ok(Classification::Decided(ordering));
                }
                Classification::Uncertain(_) => {}
            }
        }
        self.cmp_by_refinement_once(other, policy)
    }

    fn cmp_by_refinement_once(
        &self,
        other: &Self,
        policy: &CurveContext,
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
        if let Some(ordering) = disjoint_parameter_interval_order(&left, &right, policy) {
            return Ok(Classification::Decided(ordering));
        }

        // Distinct construction roots are much more common than equal values
        // represented by different polynomials. First bisect simple
        // sign-changing isolators without constructing either polynomial's
        // Sturm sequence or their GCD; retain the equality machinery below as
        // the authoritative fallback when intervals continue to overlap.
        let mut left_refinement = BezierParameterRefinement2::new(self, policy);
        let mut right_refinement = BezierParameterRefinement2::new(other, policy);
        for steps in [1, 3, 7, 15, 31] {
            let left = match left_refinement.refine_to(steps).known_interval(policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(_) => continue,
            };
            let right = match right_refinement.refine_to(steps).known_interval(policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(_) => continue,
            };
            if let Some(ordering) = disjoint_parameter_interval_order(&left, &right, policy) {
                return Ok(Classification::Decided(ordering));
            }
        }
        match self.same_value(other, policy)? {
            Classification::Decided(true) => Ok(Classification::Decided(Ordering::Equal)),
            Classification::Decided(false) => compare_distinct_parameters(self, other, policy),
            Classification::Uncertain(reason) => {
                if let Some(ordering) =
                    compare_parameters_by_exact_difference(self, other, None, None, policy)
                {
                    Ok(Classification::Decided(ordering))
                } else {
                    Ok(Classification::Uncertain(reason))
                }
            }
        }
    }

    pub(crate) fn same_value(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if matches!(
            (self, other),
            (Self::Algebraic(left), Self::Algebraic(right))
                if Arc::ptr_eq(&left.data.shared, &right.data.shared)
        ) || self == other
        {
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
                let interval = match BezierParameterInterval::try_new_ordered(start, end, policy)? {
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

fn disjoint_parameter_interval_order(
    left: &BezierParameterInterval,
    right: &BezierParameterInterval,
    policy: &CurveContext,
) -> Option<Ordering> {
    if matches!(
        compare_reals(left.end(), right.start(), policy),
        Some(Ordering::Less | Ordering::Equal)
    ) {
        return Some(Ordering::Less);
    }
    matches!(
        compare_reals(right.end(), left.start(), policy),
        Some(Ordering::Less | Ordering::Equal)
    )
    .then_some(Ordering::Greater)
}

impl<'a> BezierParameterRefinement2<'a> {
    pub(crate) fn new(parameter: &BezierParameter2, policy: &'a CurveContext) -> Self {
        Self {
            parameter: parameter.clone(),
            completed_steps: 0,
            policy,
        }
    }

    pub(crate) fn refine_to(&mut self, target_steps: usize) -> &BezierParameter2 {
        debug_assert!(target_steps >= self.completed_steps);
        let additional_steps = target_steps.saturating_sub(self.completed_steps);
        if additional_steps != 0 {
            let refined = self
                .parameter
                .clone()
                .refined_isolating_interval(additional_steps, self.policy);
            let progressed =
                matches!(refined, BezierParameter2::Exact(_)) || refined != self.parameter;
            self.parameter = refined;
            if progressed {
                self.completed_steps = target_steps;
            }
        }
        &self.parameter
    }
}

/// Signs a Real-coefficient power-basis polynomial at one retained parameter.
///
/// The direct algebraic-field path proves equality through a polynomial GCD.
/// When radical coefficients prevent that package from deciding a nonzero
/// sign, strict Bernstein bounds over progressively refined isolators provide
/// an independent exact certificate. Both exact paths run before an
/// `APPROXIMATE_512` terminal is permitted, so selecting the weaker policy
/// cannot bypass an available exact proof.
pub(crate) fn signed_coefficients_at_parameter(
    coefficients: Vec<Real>,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let strict_policy = policy.strict_counterpart();
    let direct =
        direct_signed_coefficients_at_parameter(coefficients.clone(), parameter, &strict_policy)?;
    if direct.is_decided() {
        return Ok(direct);
    }
    if let Some(sign) = strict_polynomial_sign_on_refined_parameter_interval(
        &coefficients,
        parameter,
        &strict_policy,
    )? {
        return Ok(Classification::Decided(sign));
    }
    if policy.permits_approximate_512() {
        let approximate =
            direct_signed_coefficients_at_parameter(coefficients.clone(), parameter, policy)?;
        if approximate.is_decided() {
            return Ok(approximate);
        }
        if let Some(sign) =
            strict_polynomial_sign_on_refined_parameter_interval(&coefficients, parameter, policy)?
        {
            return Ok(Classification::Decided(sign));
        }
        return Ok(approximate);
    }
    Ok(direct)
}

fn direct_signed_coefficients_at_parameter(
    coefficients: Vec<Real>,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    Ok(
        match BezierParameterPolynomial::try_new_power_basis(coefficients, policy) {
            Ok(Classification::Decided(polynomial)) => {
                signed_polynomial_at_root(Some(&polynomial), parameter, policy)?
            }
            Err(CurveError::InvalidBezierPolynomial) => Classification::Decided(RealSign::Zero),
            Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
            Err(error) => return Err(error),
        },
    )
}

/// Signs an optional nonzero parameter polynomial at a retained parameter.
/// `None` is the structurally zero polynomial.
pub(crate) fn signed_polynomial_at_root(
    polynomial: Option<&BezierParameterPolynomial>,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let Some(polynomial) = polynomial else {
        return Ok(Classification::Decided(RealSign::Zero));
    };
    match parameter {
        BezierParameter2::Exact(parameter) => {
            match real_sign(&polynomial.evaluate(parameter), policy) {
                Some(sign) => Ok(Classification::Decided(sign)),
                None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        BezierParameter2::Algebraic(parameter) => signed_polynomial_on_isolating_interval(
            polynomial,
            parameter.polynomial(),
            parameter.interval(),
            policy,
        ),
    }
}

pub(crate) fn signed_polynomial_on_isolating_interval(
    filter: &BezierParameterPolynomial,
    defining: &BezierParameterPolynomial,
    interval: &BezierParameterInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match defining.greatest_common_divisor(filter, policy)? {
        Classification::Decided(Some(common)) => {
            match common.root_count_in_interval(interval, policy)? {
                Classification::Decided(0) => {}
                Classification::Decided(1) => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Decided(_) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let mut interval = interval.clone();
    loop {
        match filter.root_count_in_interval(&interval, policy) {
            Ok(Classification::Decided(0)) => {
                return match real_sign(&filter.evaluate(interval.start()), policy) {
                    Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        Ok(Classification::Decided(sign))
                    }
                    Some(RealSign::Zero) => Err(CurveError::InvalidBezierAlgebraicParameter),
                    None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
            }
            Ok(Classification::Decided(_)) | Err(CurveError::InvalidBezierAlgebraicParameter) => {}
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(error) => return Err(error),
        }

        let midpoint = ((interval.start() + interval.end()) / Real::from(2_i8))?;
        match real_sign(&defining.evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return match real_sign(&filter.evaluate(&midpoint), policy) {
                    Some(sign) => Ok(Classification::Decided(sign)),
                    None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
            }
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new_ordered(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let right = match BezierParameterInterval::try_new_ordered(
            midpoint,
            interval.end().clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let left_count = match defining.root_count_in_interval(&left, policy)? {
            Classification::Decided(count) => count,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        interval = if left_count == 1 {
            left
        } else {
            let right_count = match defining.root_count_in_interval(&right, policy)? {
                Classification::Decided(count) => count,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if right_count != 1 {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            right
        };
    }
}

fn strict_polynomial_sign_on_refined_parameter_interval(
    coefficients: &[Real],
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    // Refinement eventually gives a strict enclosure for every nonzero
    // continuous value. A true zero cannot be misclassified because every
    // Bernstein control would have to acquire the same strict sign.
    let mut refinement = BezierParameterRefinement2::new(parameter, policy);
    for target_steps in [0, 1, 2, 4, 8, 16, 32] {
        if let Some(sign) = strict_coefficients_sign_on_parameter_interval(
            coefficients,
            refinement.refine_to(target_steps),
            policy,
        )? {
            return Ok(Some(sign));
        }
    }
    Ok(None)
}

/// Cold exact sign fallback for algebraic predicates whose Real coefficients
/// share deep computable cancellations.
///
/// Every accepted result is a certified nonzero enclosure: refinement may go
/// beyond the ordinary topology budget, but no finite-precision equality or
/// approximate sign is accepted. This is reserved for residual replay after
/// the cheaper algebraic GCD and ordinary Bernstein paths are exhausted.
pub(crate) fn deep_exact_coefficients_sign_at_parameter(
    coefficients: &[Real],
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    let strict = policy.strict_counterpart();
    let mut refinement = BezierParameterRefinement2::new(parameter, &strict);
    for (target_steps, min_precision) in [
        (8, -64),
        (16, -64),
        (32, -64),
        (32, -128),
        (64, -128),
        (64, -256),
        (128, -512),
        (128, -1024),
        (128, -2048),
        (128, -4096),
    ] {
        let parameter = refinement.refine_to(target_steps);
        let restricted = match parameter {
            BezierParameter2::Exact(parameter) => {
                vec![evaluate_coefficients(coefficients, parameter)]
            }
            BezierParameter2::Algebraic(parameter) => restrict_power_basis_to_interval(
                coefficients,
                parameter.interval().start(),
                parameter.interval().end(),
            ),
        };
        let controls =
            power_to_bernstein_coefficients(&restricted, restricted.len().saturating_sub(1))?;
        let mut common_sign = None;
        let mut certified = true;
        for control in controls {
            let sign = match control.certified_sign_until(min_precision) {
                CertifiedRealSign::Known {
                    sign: sign @ (RealSign::Positive | RealSign::Negative),
                    ..
                } => sign,
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => continue,
                CertifiedRealSign::Unknown { .. } => {
                    certified = false;
                    break;
                }
            };
            if common_sign.is_some_and(|common| common != sign) {
                certified = false;
                break;
            }
            common_sign = Some(sign);
        }
        if certified && common_sign.is_some() {
            return Ok(common_sign);
        }
    }
    Ok(None)
}

/// Attempts a nonzero sign proof over one already-selected parameter bracket.
///
/// This is the allocation-bounded front end for callers that reuse a refined
/// isolator across many related predicates. A mixed Bernstein hull says only
/// that this bracket is insufficient; equality and difficult signs still go
/// through the complete algebraic-root authority.
pub(crate) fn strict_coefficients_sign_on_parameter_interval(
    coefficients: &[Real],
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    match parameter {
        BezierParameter2::Exact(parameter) => Ok(real_sign(
            &evaluate_coefficients(coefficients, parameter),
            policy,
        )),
        BezierParameter2::Algebraic(parameter) => {
            let interval = parameter.interval();
            let restricted =
                restrict_power_basis_to_interval(coefficients, interval.start(), interval.end());
            univariate_unit_interval_strict_bernstein_sign(&restricted, policy)
        }
    }
}

/// Encloses a polynomial value over one retained parameter interval with
/// certified dyadic rational bounds.
///
/// Bernstein convex-hull bounds make this useful for batching many predicates
/// over the same selected root: callers may refine the root and scalar
/// precision together without invoking an algebraic GCD for every value.
pub(crate) fn coefficients_value_interval_on_parameter_interval(
    coefficients: &[Real],
    parameter: &BezierParameter2,
    precision: i32,
) -> CurveResult<Option<[HyperRational; 2]>> {
    let restricted = match parameter {
        BezierParameter2::Exact(parameter) => {
            vec![evaluate_coefficients(coefficients, parameter)]
        }
        BezierParameter2::Algebraic(parameter) => restrict_power_basis_to_interval(
            coefficients,
            parameter.interval().start(),
            parameter.interval().end(),
        ),
    };
    coefficients_dyadic_convex_hull(&restricted, precision)
}

/// Encloses a polynomial over one represented parameter interval.
///
/// Callers that own algebraic range boundaries first pass their certified
/// outer endpoints here.  The resulting Bernstein convex hull is conservative
/// for every selected value in that interval and contains no approximate
/// construction decision.
pub(crate) fn coefficients_value_interval_on_real_interval(
    coefficients: &[Real],
    start: &Real,
    end: &Real,
    precision: i32,
) -> CurveResult<Option<[HyperRational; 2]>> {
    let restricted = restrict_power_basis_to_interval(coefficients, start, end);
    coefficients_dyadic_convex_hull(&restricted, precision)
}

fn coefficients_dyadic_convex_hull(
    coefficients: &[Real],
    precision: i32,
) -> CurveResult<Option<[HyperRational; 2]>> {
    let controls =
        power_to_bernstein_coefficients(coefficients, coefficients.len().saturating_sub(1))?;
    let mut bounds = controls
        .into_iter()
        .map(|control| control.certified_dyadic_interval(precision));
    let Some(Some([mut lower, mut upper])) = bounds.next() else {
        return Ok(None);
    };
    for bound in bounds {
        let Some([next_lower, next_upper]) = bound else {
            return Ok(None);
        };
        if next_lower < lower {
            lower = next_lower;
        }
        if next_upper > upper {
            upper = next_upper;
        }
    }
    Ok(Some([lower, upper]))
}

/// Returns a strict common Bernstein-control sign on `[0, 1]`, if one exists.
pub(crate) fn univariate_unit_interval_strict_bernstein_sign(
    polynomial: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    if polynomial.is_empty() {
        return Ok(None);
    }
    let mut strict_sign = None;
    for control in power_to_bernstein_coefficients(polynomial, polynomial.len() - 1)? {
        let Some(sign @ (RealSign::Positive | RealSign::Negative)) = real_sign(&control, policy)
        else {
            return Ok(None);
        };
        match strict_sign {
            Some(previous) if previous != sign => return Ok(None),
            Some(_) => {}
            None => strict_sign = Some(sign),
        }
    }
    Ok(strict_sign)
}

/// Composes `p(start + u * (end-start))` in power basis. Horner composition
/// keeps only one degree-sized temporary instead of retaining every affine
/// power used by the former offset-local implementation.
fn restrict_power_basis_to_interval(coefficients: &[Real], start: &Real, end: &Real) -> Vec<Real> {
    let Some((leading, remaining)) = coefficients.split_last() else {
        return Vec::new();
    };
    let extent = end - start;
    let mut restricted = vec![leading.clone()];
    for coefficient in remaining.iter().rev() {
        let mut next = vec![Real::zero(); restricted.len() + 1];
        for (degree, value) in restricted.iter().enumerate() {
            next[degree] = &next[degree] + value * start;
            next[degree + 1] = &next[degree + 1] + value * &extent;
        }
        next[0] = &next[0] + coefficient;
        restricted = next;
    }
    restricted
}

fn refine_algebraic_sign_change(
    algebraic: &BezierAlgebraicParameter2,
    max_refinement_steps: usize,
    policy: &CurveContext,
) -> Option<BezierParameter2> {
    let polynomial = algebraic.polynomial();
    let mut start = algebraic.interval().start().clone();
    let mut end = algebraic.interval().end().clone();
    let mut start_sign = real_sign(&polynomial.evaluate(&start), policy)?;
    let end_sign = real_sign(&polynomial.evaluate(&end), policy)?;
    if !matches!(start_sign, RealSign::Positive | RealSign::Negative)
        || !matches!(end_sign, RealSign::Positive | RealSign::Negative)
        || start_sign == end_sign
    {
        return None;
    }
    for _ in 0..max_refinement_steps {
        let midpoint = midpoint_real(&start, &end).ok()?;
        let midpoint_sign = real_sign(&polynomial.evaluate(&midpoint), policy)?;
        if midpoint_sign == RealSign::Zero {
            return Some(BezierParameter2::Exact(midpoint));
        }
        if midpoint_sign == start_sign {
            start = midpoint;
            start_sign = midpoint_sign;
        } else if midpoint_sign == end_sign {
            end = midpoint;
        } else {
            return None;
        }
    }
    let interval = match BezierParameterInterval::try_new_ordered(start, end, policy).ok()? {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(_) => return None,
    };
    Some(BezierParameter2::Algebraic(
        algebraic.with_certified_interval(interval),
    ))
}

enum RefinedParameter<'a> {
    Exact(Real),
    Algebraic {
        parameter: &'a BezierAlgebraicParameter2,
        interval: BezierParameterInterval,
        sturm_sequence: Arc<[Vec<Real>]>,
    },
}

impl<'a> RefinedParameter<'a> {
    fn from_parameter(
        parameter: &'a BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match parameter {
            BezierParameter2::Exact(value) => {
                Ok(Classification::Decided(Self::Exact(value.clone())))
            }
            BezierParameter2::Algebraic(parameter) => {
                let sturm_sequence = match parameter.retained_sturm_sequence(policy)? {
                    Classification::Decided(sequence) => sequence,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(Classification::Decided(Self::Algebraic {
                    parameter,
                    interval: parameter.interval().clone(),
                    sturm_sequence,
                }))
            }
        }
    }

    fn bounds(&self) -> (&Real, &Real) {
        match self {
            Self::Exact(value) => (value, value),
            Self::Algebraic { interval, .. } => (interval.start(), interval.end()),
        }
    }

    fn refine_once(self, policy: &CurveContext) -> CurveResult<Classification<Self>> {
        let Self::Algebraic {
            parameter,
            interval,
            sturm_sequence,
        } = self
        else {
            return Ok(Classification::Decided(self));
        };
        let midpoint = midpoint_real(interval.start(), interval.end())?;
        match real_sign(&parameter.polynomial().evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => return Ok(Classification::Decided(Self::Exact(midpoint))),
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new_ordered(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let interval = match parameter
            .polynomial()
            .root_count_in_interval_with_sequence(&left, &sturm_sequence, policy)?
        {
            Classification::Decided(1) => left,
            Classification::Decided(0) => {
                match BezierParameterInterval::try_new_ordered(
                    midpoint,
                    interval.end().clone(),
                    policy,
                )? {
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
            sturm_sequence,
        }))
    }
}

fn compare_distinct_parameters(
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Ordering>> {
    const EXACT_DIFFERENCE_REFINEMENTS: usize = 64;
    const APPROXIMATE_MAX_ORDERING_REFINEMENTS: usize =
        (-hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION) as usize;
    // STRICT has no finite precision terminal: after the historical 64 exact
    // bisections, transfer the remaining proof obligation to exact algebraic
    // difference construction and otherwise report uncertainty. Approximate
    // policy alone continues to its specified 512-bit equality terminal.
    let max_ordering_refinements = if policy.permits_approximate_512() {
        APPROXIMATE_MAX_ORDERING_REFINEMENTS
    } else {
        EXACT_DIFFERENCE_REFINEMENTS
    };

    let mut first = match RefinedParameter::from_parameter(first_parameter, policy)? {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(exact_difference_or_uncertain(
                first_parameter,
                second_parameter,
                None,
                None,
                policy,
                reason,
            ));
        }
    };
    let mut second = match RefinedParameter::from_parameter(second_parameter, policy)? {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(exact_difference_or_uncertain(
                first_parameter,
                second_parameter,
                Some(&first),
                None,
                policy,
                reason,
            ));
        }
    };
    for refinement_count in 0..=max_ordering_refinements {
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
            _ if refinement_count == EXACT_DIFFERENCE_REFINEMENTS => {
                if let Some(ordering) = compare_parameters_by_exact_difference(
                    first_parameter,
                    second_parameter,
                    Some(&first),
                    Some(&second),
                    policy,
                ) {
                    return Ok(Classification::Decided(ordering));
                }
            }
            _ => {}
        }
        if refinement_count == max_ordering_refinements {
            break;
        }

        match (&first, &second) {
            (RefinedParameter::Exact(_), RefinedParameter::Exact(_)) => break,
            (RefinedParameter::Exact(_), RefinedParameter::Algebraic { .. }) => {
                second = match second.refine_once(policy)? {
                    Classification::Decided(second) => second,
                    Classification::Uncertain(reason) => {
                        return Ok(exact_difference_or_uncertain(
                            first_parameter,
                            second_parameter,
                            Some(&first),
                            None,
                            policy,
                            reason,
                        ));
                    }
                };
            }
            (RefinedParameter::Algebraic { .. }, RefinedParameter::Exact(_)) => {
                first = match first.refine_once(policy)? {
                    Classification::Decided(first) => first,
                    Classification::Uncertain(reason) => {
                        return Ok(exact_difference_or_uncertain(
                            first_parameter,
                            second_parameter,
                            None,
                            Some(&second),
                            policy,
                            reason,
                        ));
                    }
                };
            }
            (RefinedParameter::Algebraic { .. }, RefinedParameter::Algebraic { .. }) => {
                first = match first.refine_once(policy)? {
                    Classification::Decided(first) => first,
                    Classification::Uncertain(reason) => {
                        return Ok(exact_difference_or_uncertain(
                            first_parameter,
                            second_parameter,
                            None,
                            Some(&second),
                            policy,
                            reason,
                        ));
                    }
                };
                second = match second.refine_once(policy)? {
                    Classification::Decided(second) => second,
                    Classification::Uncertain(reason) => {
                        return Ok(exact_difference_or_uncertain(
                            first_parameter,
                            second_parameter,
                            Some(&first),
                            None,
                            policy,
                            reason,
                        ));
                    }
                };
            }
        }
    }
    if policy.permits_approximate_512() {
        policy.observe_approximate_512();
        return Ok(Classification::Decided(Ordering::Equal));
    }
    Ok(Classification::Uncertain(UncertaintyReason::Ordering))
}

#[cold]
#[inline(never)]
fn exact_difference_or_uncertain(
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    first_refined: Option<&RefinedParameter<'_>>,
    second_refined: Option<&RefinedParameter<'_>>,
    policy: &CurveContext,
    reason: UncertaintyReason,
) -> Classification<Ordering> {
    compare_parameters_by_exact_difference(
        first_parameter,
        second_parameter,
        first_refined,
        second_refined,
        policy,
    )
    .map(Classification::Decided)
    .unwrap_or(Classification::Uncertain(reason))
}

#[cold]
#[inline(never)]
fn compare_parameters_by_exact_difference(
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    first_refined: Option<&RefinedParameter<'_>>,
    second_refined: Option<&RefinedParameter<'_>>,
    policy: &CurveContext,
) -> Option<Ordering> {
    if policy.permits_approximate_512() {
        return None;
    }
    let first = parameter_algebraic_representation(first_parameter, first_refined, policy)?;
    let second = parameter_algebraic_representation(second_parameter, second_refined, policy)?;
    let comparison = compare_algebraic_root_representations_by_difference(
        &first,
        &second,
        AlgebraicRootRefinementComparisonConfig {
            policy: hypersolve::PredicatePolicy::STRICT,
            max_refinement_rounds: 0,
            steps_per_round: 1,
        },
    )
    .comparison;
    matches!(
        comparison.status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    )
    .then_some(comparison.ordering)
    .flatten()
}

fn parameter_algebraic_representation(
    parameter: &BezierParameter2,
    refined: Option<&RefinedParameter<'_>>,
    policy: &CurveContext,
) -> Option<AlgebraicRootRepresentation> {
    if let Some(RefinedParameter::Exact(value)) = refined {
        return Some(crate::bezier_algebraic_image::exact_real_algebraic_representation(value));
    }
    match parameter {
        BezierParameter2::Exact(value) => {
            Some(crate::bezier_algebraic_image::exact_real_algebraic_representation(value))
        }
        BezierParameter2::Algebraic(parameter) => {
            let mut representation =
                crate::bezier_algebraic_image::parameter_representation(parameter, policy);
            if let Some(RefinedParameter::Algebraic { interval, .. }) = refined {
                representation.interval.lower = interval.start().clone();
                representation.interval.upper = interval.end().clone();
                representation.interval.exact_root = None;
                representation.interval.distinct_root_count = 1;
            }
            representation.is_valid().then_some(representation)
        }
    }
}

fn refine_algebraic_upper_gap(
    parameter: &BezierAlgebraicParameter2,
    upper_bound: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    let sequence = match parameter.retained_sturm_sequence(policy)? {
        Classification::Decided(sequence) => sequence,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut interval = parameter.interval().clone();
    loop {
        if compare_reals(interval.end(), upper_bound, policy) == Some(Ordering::Less) {
            return midpoint_real(interval.end(), upper_bound).map(Classification::Decided);
        }
        let midpoint = midpoint_real(interval.start(), interval.end())?;
        match real_sign(&parameter.polynomial().evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return midpoint_real(&midpoint, upper_bound).map(Classification::Decided);
            }
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new_ordered(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match parameter
            .polynomial()
            .root_count_in_interval_with_sequence(&left, sequence.as_ref(), policy)?
        {
            Classification::Decided(1) => interval = left,
            Classification::Decided(0) => {
                interval = match BezierParameterInterval::try_new_ordered(
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
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    let sequence = match parameter.retained_sturm_sequence(policy)? {
        Classification::Decided(sequence) => sequence,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut interval = parameter.interval().clone();
    loop {
        if compare_reals(lower_bound, interval.start(), policy) == Some(Ordering::Less) {
            return midpoint_real(lower_bound, interval.start()).map(Classification::Decided);
        }
        let midpoint = midpoint_real(interval.start(), interval.end())?;
        match real_sign(&parameter.polynomial().evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return midpoint_real(lower_bound, &midpoint).map(Classification::Decided);
            }
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new_ordered(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match parameter
            .polynomial()
            .root_count_in_interval_with_sequence(&left, sequence.as_ref(), policy)?
        {
            Classification::Decided(1) => interval = left,
            Classification::Decided(0) => {
                interval = match BezierParameterInterval::try_new_ordered(
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
    Ok(Real::average_pair(first, second))
}

impl BezierParameterRange2 {
    /// Constructs a certified positive-length oriented parameter range.
    pub fn try_new(
        start: BezierParameter2,
        end: BezierParameter2,
        policy: &CurveContext,
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
        policy: &CurveContext,
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
    let mut integer_coefficients = Vec::with_capacity(rationals.len());
    for rational in rationals {
        let magnitude = rational.numerator() * (&common_denominator / rational.denominator());
        if !magnitude.is_zero() {
            content = if content.is_zero() {
                magnitude.clone()
            } else {
                content.gcd(&magnitude)
            };
        }
        integer_coefficients.push(BigInt::from_biguint(rational.sign(), magnitude));
    }
    let leading_magnitude = integer_coefficients.last()?.magnitude().clone();
    if content.is_zero() || leading_magnitude.is_zero() {
        return None;
    }
    let content_integer = BigInt::from(content.clone());
    for coefficient in &mut integer_coefficients {
        *coefficient /= &content_integer;
    }
    if !could_have_rational_root_modulo_small_primes(&integer_coefficients) {
        return None;
    }
    Some(leading_magnitude / content)
}

/// Rejects rational roots by reduction modulo primes not dividing the lead.
///
/// A reduced rational root `a/b` of a primitive integer polynomial has
/// `b` dividing the leading coefficient. For a prime that does not divide
/// that coefficient, `b` is invertible in the finite field, so `a/b` must
/// appear as a root of the reduced polynomial. One rootless reduction is
/// therefore a complete exact rejection; surviving every small prime merely
/// falls through to bounded rational reconstruction.
fn could_have_rational_root_modulo_small_primes(coefficients: &[BigInt]) -> bool {
    const PRIMES: [u32; 11] = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31];
    let leading = coefficients
        .last()
        .expect("nonempty polynomial has a leading coefficient");
    let mut residues = Vec::with_capacity(coefficients.len());
    for prime in PRIMES {
        let leading_residue = bigint_modulo_u32(leading, prime);
        if leading_residue == 0 {
            continue;
        }
        residues.clear();
        residues.extend(
            coefficients
                .iter()
                .map(|coefficient| bigint_modulo_u32(coefficient, prime)),
        );
        let has_root = (0..prime).any(|candidate| {
            residues.iter().rev().fold(0_u64, |value, coefficient| {
                (value * u64::from(candidate) + u64::from(*coefficient)) % u64::from(prime)
            }) == 0
        });
        if !has_root {
            return false;
        }
    }
    true
}

fn bigint_modulo_u32(value: &BigInt, modulus: u32) -> u32 {
    debug_assert_ne!(modulus, 0);
    let modulus = u64::from(modulus);
    let radix = (u64::from(u32::MAX) + 1) % modulus;
    let magnitude_residue = value.iter_u32_digits().rev().fold(0_u64, |residue, digit| {
        (residue * radix + u64::from(digit) % modulus) % modulus
    });
    if value.sign() == num::bigint::Sign::Minus && magnitude_residue != 0 {
        (modulus - magnitude_residue) as u32
    } else {
        magnitude_residue as u32
    }
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
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Vec<Real>>>> {
    if let Some(sequence) = primitive_integer_sturm_sequence(coefficients) {
        return Ok(Classification::Decided(sequence));
    }
    let p0 = coefficients.to_vec();
    let p1 = derivative_coefficients(coefficients);
    let p1 = match normalize_coefficients(p1, policy)? {
        Classification::Decided(Some(coefficients)) => coefficients,
        Classification::Decided(None) => return Ok(Classification::Decided(vec![p0])),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let maximum_sequence_len = p0.len();
    let mut sequence = vec![p0, p1];
    while sequence.len() < maximum_sequence_len {
        let previous = sequence[sequence.len() - 2].clone();
        let divisor_len = sequence[sequence.len() - 1].len();
        let remainder = match scale_invariant_polynomial_remainder(
            previous,
            &sequence[sequence.len() - 1],
            policy,
        )? {
            Classification::Decided(Some(remainder)) => remainder,
            Classification::Decided(None) => break,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if remainder.len() >= divisor_len {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        sequence.push(negate_coefficients(remainder));
    }

    Ok(Classification::Decided(sequence))
}

fn primitive_integer_sturm_sequence(coefficients: &[Real]) -> Option<Vec<Vec<Real>>> {
    let rationals = coefficients
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let p0 = HyperRational::primitive_bigint_ratio(&rationals);
    let p1 = primitive_bigint_coefficients(
        p0.iter()
            .enumerate()
            .skip(1)
            .map(|(degree, coefficient)| coefficient * BigInt::from(degree))
            .collect(),
    );
    let mut integer_sequence = vec![p0];
    if !p1.is_empty() {
        integer_sequence.push(p1);
    }
    let maximum_sequence_len = coefficients.len();
    while integer_sequence.len() >= 2 && integer_sequence.len() < maximum_sequence_len {
        let previous = integer_sequence[integer_sequence.len() - 2].clone();
        let divisor_len = integer_sequence[integer_sequence.len() - 1].len();
        let mut remainder = primitive_integer_pseudo_remainder_bigint(
            previous,
            &integer_sequence[integer_sequence.len() - 1],
        )?;
        if remainder.is_empty() {
            break;
        }
        if remainder.len() >= divisor_len {
            return None;
        }
        for coefficient in &mut remainder {
            *coefficient = -std::mem::take(coefficient);
        }
        integer_sequence.push(remainder);
    }
    Some(
        integer_sequence
            .into_iter()
            .map(|polynomial| {
                polynomial
                    .into_iter()
                    .map(|coefficient| Real::new(HyperRational::from_bigint(coefficient)))
                    .collect()
            })
            .collect(),
    )
}

fn sign_variations_at(
    sequence: &[Vec<Real>],
    parameter: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<usize>> {
    match sturm_point_evidence(sequence, parameter, policy)? {
        Classification::Decided(SturmPointEvidence::Root) => {
            Err(CurveError::InvalidBezierAlgebraicParameter)
        }
        Classification::Decided(SturmPointEvidence::NonRoot(variations)) => {
            Ok(Classification::Decided(variations))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

enum SturmPointEvidence {
    Root,
    NonRoot(usize),
}

fn sturm_point_evidence(
    sequence: &[Vec<Real>],
    parameter: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<SturmPointEvidence>> {
    let mut previous = None;
    let mut variations = 0_usize;

    for (index, polynomial) in sequence.iter().enumerate() {
        let sign = match exact_integer_polynomial_sign(polynomial, parameter)
            .or_else(|| real_sign(&evaluate_coefficients(polynomial, parameter), policy))
        {
            Some(RealSign::Zero) if index == 0 => {
                return Ok(Classification::Decided(SturmPointEvidence::Root));
            }
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

    Ok(Classification::Decided(SturmPointEvidence::NonRoot(
        variations,
    )))
}

fn exact_integer_polynomial_sign(coefficients: &[Real], parameter: &Real) -> Option<RealSign> {
    let parameter = parameter.exact_rational_ref()?;
    let (leading, coefficients) = coefficients.split_last()?;
    let leading = leading.exact_rational_ref()?;
    if !leading.denominator().is_one()
        || !coefficients.iter().all(|coefficient| {
            coefficient
                .exact_rational_ref()
                .is_some_and(|coefficient| coefficient.denominator().is_one())
        })
    {
        return None;
    }
    if let Some(sign) = exact_integer_polynomial_sign_i128(coefficients, leading, parameter) {
        return Some(sign);
    }
    let parameter_numerator = BigInt::from_biguint(parameter.sign(), parameter.numerator().clone());
    let parameter_denominator = BigInt::from(parameter.denominator().clone());
    let mut denominator_power = BigInt::one();
    let mut accumulator = BigInt::from_biguint(leading.sign(), leading.numerator().clone());
    for coefficient in coefficients.iter().rev() {
        let coefficient = coefficient
            .exact_rational_ref()
            .expect("integer coefficients were checked");
        denominator_power *= &parameter_denominator;
        accumulator *= &parameter_numerator;
        accumulator += BigInt::from_biguint(coefficient.sign(), coefficient.numerator().clone())
            * &denominator_power;
    }
    Some(match accumulator.sign() {
        num::bigint::Sign::Minus => RealSign::Negative,
        num::bigint::Sign::NoSign => RealSign::Zero,
        num::bigint::Sign::Plus => RealSign::Positive,
    })
}

fn exact_integer_polynomial_sign_i128(
    coefficients: &[Real],
    leading: &HyperRational,
    parameter: &HyperRational,
) -> Option<RealSign> {
    let parameter_numerator = rational_signed_numerator_i128(parameter)?;
    let parameter_denominator = i128::try_from(parameter.denominator().to_u128()?).ok()?;
    let mut denominator_power = 1_i128;
    let mut accumulator = rational_signed_numerator_i128(leading)?;
    for coefficient in coefficients.iter().rev() {
        denominator_power = denominator_power.checked_mul(parameter_denominator)?;
        accumulator = accumulator.checked_mul(parameter_numerator)?.checked_add(
            rational_signed_numerator_i128(
                coefficient
                    .exact_rational_ref()
                    .expect("integer coefficients were checked"),
            )?
            .checked_mul(denominator_power)?,
        )?;
    }
    Some(match accumulator.cmp(&0) {
        Ordering::Less => RealSign::Negative,
        Ordering::Equal => RealSign::Zero,
        Ordering::Greater => RealSign::Positive,
    })
}

fn rational_signed_numerator_i128(value: &HyperRational) -> Option<i128> {
    let magnitude = value.numerator().to_u128()?;
    match value.sign() {
        num::bigint::Sign::NoSign => Some(0),
        num::bigint::Sign::Plus => i128::try_from(magnitude).ok(),
        num::bigint::Sign::Minus if magnitude == 1_u128 << 127 => Some(i128::MIN),
        num::bigint::Sign::Minus => i128::try_from(magnitude).ok()?.checked_neg(),
    }
}

fn scale_invariant_polynomial_remainder(
    dividend: Vec<Real>,
    divisor: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    if let Some(remainder) = primitive_integer_pseudo_remainder(&dividend, divisor) {
        return Ok(Classification::Decided(
            (!remainder.is_empty()).then_some(remainder),
        ));
    }
    polynomial_remainder(dividend, divisor, policy)
}

/// Returns a positive multiple of the field remainder for rational inputs.
///
/// GCDs and Sturm chains are invariant under positive polynomial scaling, so
/// they can avoid coefficient division by clearing denominators once and
/// pseudo-dividing with integers. A negative divisor leading coefficient
/// contributes one sign per elimination step; correcting that parity keeps
/// every returned Sturm member a positive multiple of the ordinary remainder.
fn primitive_integer_pseudo_remainder(dividend: &[Real], divisor: &[Real]) -> Option<Vec<Real>> {
    let dividend = dividend
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let divisor = divisor
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let remainder = HyperRational::primitive_bigint_ratio(&dividend);
    let divisor = HyperRational::primitive_bigint_ratio(&divisor);
    primitive_integer_pseudo_remainder_bigint(remainder, &divisor).map(|remainder| {
        remainder
            .into_iter()
            .map(|coefficient| Real::new(HyperRational::from_bigint(coefficient)))
            .collect()
    })
}

fn primitive_integer_pseudo_remainder_bigint(
    mut remainder: Vec<BigInt>,
    divisor: &[BigInt],
) -> Option<Vec<BigInt>> {
    let divisor_leading = divisor.last()?;
    if divisor_leading.is_zero() {
        return None;
    }

    let mut steps = 0_usize;
    while remainder.len() >= divisor.len() {
        let remainder_leading = remainder.last()?.clone();
        let shift = remainder.len() - divisor.len();
        for coefficient in &mut remainder[..shift] {
            *coefficient *= divisor_leading;
        }
        for (index, divisor_coefficient) in divisor[..divisor.len() - 1].iter().enumerate() {
            let target = shift + index;
            remainder[target] *= divisor_leading;
            remainder[target] -= &remainder_leading * divisor_coefficient;
        }
        remainder.pop();
        while remainder.last().is_some_and(BigInt::is_zero) {
            remainder.pop();
        }
        steps += 1;
    }

    if divisor_leading < &BigInt::zero() && !steps.is_multiple_of(2) {
        for coefficient in &mut remainder {
            *coefficient = -std::mem::take(coefficient);
        }
    }
    Some(primitive_bigint_coefficients(remainder))
}

fn primitive_bigint_coefficients(mut coefficients: Vec<BigInt>) -> Vec<BigInt> {
    let content = coefficients
        .iter()
        .fold(BigInt::zero(), |content, coefficient| {
            content.gcd(coefficient)
        });
    if !content.is_zero() && !content.is_one() {
        for coefficient in &mut coefficients {
            *coefficient /= &content;
        }
    }
    coefficients
}

fn polynomial_remainder(
    mut remainder: Vec<Real>,
    divisor: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let mut divisor_len = divisor.len();
    while divisor_len != 0 {
        match is_zero(&divisor[divisor_len - 1], policy) {
            Some(true) => divisor_len -= 1,
            Some(false) => break,
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
        }
    }
    if divisor_len == 0 {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let divisor = &divisor[..divisor_len];

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
        let factor = (&remainder[remainder.len() - 1] / &divisor[divisor.len() - 1])?;
        for (index, divisor_coefficient) in divisor[..divisor.len() - 1].iter().enumerate() {
            let product = &factor * divisor_coefficient;
            remainder[shift + index] = &remainder[shift + index] - &product;
        }
        remainder.pop();
    }
}

fn normalize_coefficients(
    mut coefficients: Vec<Real>,
    policy: &CurveContext,
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

pub(crate) fn evaluate_coefficients(coefficients: &[Real], parameter: &Real) -> Real {
    if let Some(parameter) = parameter.exact_rational_ref() {
        if parameter == &HyperRational::zero() {
            return coefficients.first().cloned().unwrap_or_else(Real::zero);
        }
        if coefficients
            .iter()
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
        {
            let Some((leading, coefficients)) = coefficients.split_last() else {
                return Real::zero();
            };
            let mut accumulator = leading
                .exact_rational_ref()
                .expect("all coefficients were checked")
                .clone();
            let one = HyperRational::one();
            for coefficient in coefficients.iter().rev() {
                accumulator = HyperRational::signed_product_sum2(
                    [true, true],
                    [
                        [&accumulator, parameter],
                        [
                            coefficient
                                .exact_rational_ref()
                                .expect("all coefficients were checked"),
                            &one,
                        ],
                    ],
                );
            }
            return Real::new(accumulator);
        }
        if parameter == &HyperRational::one() {
            return Real::sum_refs(coefficients);
        }
    }
    if let [constant, linear, quadratic] = coefficients
        && parameter.exact_rational_ref().is_none()
        && coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        let twice_quadratic = Real::from(2_i8) * quadratic;
        let vertex = (-linear.clone()) / &twice_quadratic;
        let correction = (linear * linear) / (&twice_quadratic * Real::from(2_i8));
        if let (Ok(vertex), Ok(correction)) = (vertex, correction) {
            let vertex_value = constant - correction;
            let offset = parameter - vertex;
            return quadratic * &offset * offset + vertex_value;
        }
    }
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

type RationalBernsteinBasis = Vec<Vec<HyperRational>>;
type CoefficientIntervals = Vec<[HyperRational; 2]>;

// One shared enclosure keeps coefficient approximation work reusable across
// every subdivided control. Inconclusive interval signs fall through to exact
// `Real` refinement without changing the construction certificate.
const BERNSTEIN_COEFFICIENT_INTERVAL_PRECISION: i32 = -14;

fn power_to_rational_bernstein_basis(degree: usize) -> CurveResult<RationalBernsteinBasis> {
    let coefficient_count = degree + 1;
    let mut controls = vec![vec![HyperRational::zero(); coefficient_count]; coefficient_count];
    for power in 0..coefficient_count {
        let mut basis = vec![Real::zero(); coefficient_count];
        basis[power] = Real::one();
        for (control, value) in power_to_bernstein_coefficients(&basis, degree)?
            .into_iter()
            .enumerate()
        {
            controls[control][power] = value
                .exact_rational_ref()
                .expect("a rational basis change remains rational")
                .clone();
        }
    }
    Ok(controls)
}

fn rational_bernstein_basis_value(coefficients: &[Real], weights: &[HyperRational]) -> Real {
    debug_assert_eq!(coefficients.len(), weights.len());
    // Controls remain rational coordinates in the original power-coefficient
    // frame. Materializing only the requested sign value prevents recursive
    // averaging from growing a deep `Real` midpoint DAG.
    Real::sum_owned(
        coefficients
            .iter()
            .zip(weights)
            .filter(|(_, weight)| !weight.is_zero())
            .map(|(coefficient, weight)| coefficient * Real::new(weight.clone())),
    )
}

fn coefficient_intervals(coefficients: &[Real], precision: i32) -> Option<CoefficientIntervals> {
    coefficients
        .iter()
        .map(|coefficient| coefficient.certified_dyadic_interval(precision))
        .collect()
}

fn rational_bernstein_basis_sign(
    coefficients: &[Real],
    coefficient_intervals: &CoefficientIntervals,
    weights: &[HyperRational],
    policy: &CurveContext,
) -> Option<RealSign> {
    debug_assert_eq!(coefficients.len(), weights.len());
    debug_assert_eq!(coefficient_intervals.len(), weights.len());
    let zero = HyperRational::zero();
    if weights.iter().any(|weight| weight < &zero) {
        return real_sign(
            &rational_bernstein_basis_value(coefficients, weights),
            policy,
        );
    }
    let mut lower = HyperRational::zero();
    let mut upper = HyperRational::zero();
    for (weight, interval) in weights.iter().zip(coefficient_intervals) {
        lower = &lower + weight * &interval[0];
        upper = &upper + weight * &interval[1];
    }
    if lower > zero {
        return Some(RealSign::Positive);
    }
    if upper < zero {
        return Some(RealSign::Negative);
    }
    if lower.is_zero() && upper.is_zero() {
        return Some(RealSign::Zero);
    }
    real_sign(
        &rational_bernstein_basis_value(coefficients, weights),
        policy,
    )
}

fn rational_bernstein_sign_variations(
    coefficients: &[Real],
    coefficient_intervals: &CoefficientIntervals,
    controls: &RationalBernsteinBasis,
    start_sign: RealSign,
    end_sign: RealSign,
    policy: &CurveContext,
) -> Option<usize> {
    debug_assert!(start_sign != RealSign::Zero);
    debug_assert!(end_sign != RealSign::Zero);
    let mut previous = start_sign;
    let mut variations = 0_usize;
    for control in &controls[1..controls.len() - 1] {
        let sign =
            rational_bernstein_basis_sign(coefficients, coefficient_intervals, control, policy)?;
        if sign != RealSign::Zero {
            variations += usize::from(previous != sign);
            previous = sign;
        }
    }
    Some(variations + usize::from(previous != end_sign))
}

fn midpoint_rational_bernstein_basis(
    first: &[HyperRational],
    second: &[HyperRational],
) -> Vec<HyperRational> {
    first
        .iter()
        .zip(second)
        .map(|(first, second)| HyperRational::average_pair(first, second))
        .collect()
}

fn subdivide_rational_bernstein_half(
    controls: &RationalBernsteinBasis,
) -> (RationalBernsteinBasis, RationalBernsteinBasis) {
    let mut level = controls.clone();
    let mut left = Vec::with_capacity(controls.len());
    let mut right = Vec::with_capacity(controls.len());
    while !level.is_empty() {
        left.push(level[0].clone());
        right.push(level[level.len() - 1].clone());
        level = level
            .windows(2)
            .map(|pair| midpoint_rational_bernstein_basis(&pair[0], &pair[1]))
            .collect();
    }
    right.reverse();
    (left, right)
}

fn exact_nonrational_bernstein_unit_roots(
    polynomial: &BezierParameterPolynomial,
    policy: &CurveContext,
    trace: &mut BezierRootIsolationTrace2,
) -> CurveResult<Option<Vec<BezierParameter2>>> {
    if polynomial.degree() < 2
        || polynomial
            .coefficients()
            .iter()
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
    {
        return Ok(None);
    }

    let coefficients = polynomial.coefficients();
    let controls = power_to_rational_bernstein_basis(polynomial.degree())?;
    let Some(coefficient_intervals) =
        coefficient_intervals(coefficients, BERNSTEIN_COEFFICIENT_INTERVAL_PRECISION)
    else {
        return Ok(None);
    };
    let start_sign = match rational_bernstein_basis_sign(
        coefficients,
        &coefficient_intervals,
        &controls[0],
        policy,
    ) {
        Some(RealSign::Positive) => RealSign::Positive,
        Some(RealSign::Negative) => RealSign::Negative,
        Some(RealSign::Zero) | None => return Ok(None),
    };
    let end_sign = match rational_bernstein_basis_sign(
        coefficients,
        &coefficient_intervals,
        &controls[controls.len() - 1],
        policy,
    ) {
        Some(RealSign::Positive) => RealSign::Positive,
        Some(RealSign::Negative) => RealSign::Negative,
        Some(RealSign::Zero) | None => return Ok(None),
    };
    let mut pending = vec![(
        controls,
        Real::zero(),
        Real::one(),
        0_usize,
        true,
        true,
        start_sign,
        end_sign,
    )];
    let mut isolated = Vec::new();
    while let Some((
        controls,
        start,
        end,
        depth,
        touches_start,
        touches_end,
        start_sign,
        end_sign,
    )) = pending.pop()
    {
        trace.maximum_depth = trace.maximum_depth.max(depth);
        let variations = match rational_bernstein_sign_variations(
            coefficients,
            &coefficient_intervals,
            &controls,
            start_sign,
            end_sign,
            policy,
        ) {
            Some(variations) => variations,
            None => return Ok(None),
        };
        trace.interval_root_counts += 1;
        if variations == 0 {
            continue;
        }
        if variations == 1 && !touches_start && !touches_end {
            let interval = match BezierParameterInterval::try_new(start, end, policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(_) => return Ok(None),
            };
            isolated.push(BezierParameter2::Algebraic(
                BezierAlgebraicParameter2::from_certified_simple_singleton(
                    polynomial.clone(),
                    interval,
                ),
            ));
            continue;
        }
        if depth >= 64 {
            return Ok(None);
        }
        let midpoint = midpoint_real(&start, &end)?;
        let (left, right) = subdivide_rational_bernstein_half(&controls);
        let midpoint_sign = match rational_bernstein_basis_sign(
            coefficients,
            &coefficient_intervals,
            &left[left.len() - 1],
            policy,
        ) {
            Some(RealSign::Positive) => RealSign::Positive,
            Some(RealSign::Negative) => RealSign::Negative,
            // A zero midpoint is a represented root. The existing Sturm path
            // materializes and deflates it without mixing certificates.
            Some(RealSign::Zero) | None => return Ok(None),
        };
        trace.bisections += 1;
        pending.push((
            right,
            midpoint.clone(),
            end,
            depth + 1,
            false,
            touches_end,
            midpoint_sign,
            end_sign,
        ));
        pending.push((
            left,
            start,
            midpoint,
            depth + 1,
            touches_start,
            false,
            start_sign,
            midpoint_sign,
        ));
    }
    Ok(Some(isolated))
}

fn exact_rational_square_free_bernstein_unit_roots(
    polynomial: &BezierParameterPolynomial,
    policy: &CurveContext,
    trace: &mut BezierRootIsolationTrace2,
) -> CurveResult<Option<Vec<BezierParameter2>>> {
    if polynomial.degree() < 2
        || polynomial
            .coefficients()
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return Ok(None);
    }
    let controls = power_to_bernstein_coefficients(polynomial.coefficients(), polynomial.degree())?;
    let endpoint_sign = |value: &Real| match real_sign(value, policy) {
        Some(RealSign::Positive) => Some(RealSign::Positive),
        Some(RealSign::Negative) => Some(RealSign::Negative),
        Some(RealSign::Zero) | None => None,
    };
    let Some(start_sign) = endpoint_sign(&controls[0]) else {
        return Ok(None);
    };
    let Some(end_sign) = endpoint_sign(&controls[controls.len() - 1]) else {
        return Ok(None);
    };
    let variations = |controls: &[Real], start_sign: RealSign, end_sign: RealSign| {
        let mut previous = start_sign;
        let mut variations = 0_usize;
        for control in &controls[1..controls.len() - 1] {
            let sign = real_sign(control, policy)?;
            if sign != RealSign::Zero {
                variations += usize::from(previous != sign);
                previous = sign;
            }
        }
        Some(variations + usize::from(previous != end_sign))
    };
    let mut pending = vec![(
        controls,
        Real::zero(),
        Real::one(),
        0_usize,
        true,
        true,
        start_sign,
        end_sign,
    )];
    let mut isolated = Vec::new();
    while let Some((
        controls,
        start,
        end,
        depth,
        touches_start,
        touches_end,
        start_sign,
        end_sign,
    )) = pending.pop()
    {
        trace.maximum_depth = trace.maximum_depth.max(depth);
        let Some(variation_count) = variations(&controls, start_sign, end_sign) else {
            return Ok(None);
        };
        trace.interval_root_counts += 1;
        if variation_count == 0 {
            continue;
        }
        if variation_count == 1 && !touches_start && !touches_end {
            let interval = match BezierParameterInterval::try_new(start, end, policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(_) => return Ok(None),
            };
            isolated.push(BezierParameter2::Algebraic(
                BezierAlgebraicParameter2::from_certified_simple_singleton(
                    polynomial.clone(),
                    interval,
                ),
            ));
            continue;
        }
        let midpoint = midpoint_real(&start, &end)?;
        let (left, right) = subdivide_scalar_bernstein_half(&controls)?;
        let Some(midpoint_sign) = endpoint_sign(&left[left.len() - 1]) else {
            return Ok(None);
        };
        trace.bisections += 1;
        pending.push((
            right,
            midpoint.clone(),
            end,
            depth + 1,
            false,
            touches_end,
            midpoint_sign,
            end_sign,
        ));
        pending.push((
            left,
            start,
            midpoint,
            depth + 1,
            touches_start,
            false,
            start_sign,
            midpoint_sign,
        ));
    }
    Ok(Some(isolated))
}

fn isolate_unit_roots(
    mut coefficients: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierRootIsolationResult2>> {
    let mut trace = BezierRootIsolationTrace2::default();
    if coefficients.len() <= 3
        && coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
        && let Some(result) = exact_nonrational_low_degree_unit_roots(&coefficients, policy)?
    {
        return Ok(result);
    }
    if coefficients
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        let polynomial =
            match BezierParameterPolynomial::try_new_power_basis(coefficients.clone(), policy) {
                Ok(Classification::Decided(polynomial)) => Some(polynomial),
                Err(CurveError::InvalidBezierPolynomial) => None,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(error) => return Err(error),
            };
        if let Some(polynomial) = polynomial {
            if let Some(algebraic) =
                exact_nonrational_bernstein_unit_roots(&polynomial, policy, &mut trace)?
            {
                return ordered_root_isolation_result(algebraic, trace, policy);
            }
            coefficients = polynomial.coefficients;
        }
    }
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
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)
        {
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
        let has_interior_represented_root = represented_boundaries.iter().any(|root| {
            compare_reals(root, &Real::zero(), policy) == Some(Ordering::Greater)
                && compare_reals(root, &Real::one(), policy) == Some(Ordering::Less)
        });
        if !has_interior_represented_root
            && let Some(mut algebraic) =
                exact_nonrational_bernstein_unit_roots(&polynomial, policy, &mut trace)?
        {
            represented.append(&mut algebraic);
            break;
        }
        match search_unit_roots(&polynomial, &represented_boundaries, policy, &mut trace)? {
            Classification::Decided(UnitRootSearch::Isolated(mut algebraic)) => {
                represented.append(&mut algebraic);
                break;
            }
            Classification::Decided(UnitRootSearch::RepresentedRoot(root)) => {
                represented.push(BezierParameter2::Exact(root.clone()));
                coefficients = polynomial.coefficients;
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

    ordered_root_isolation_result(represented, trace, policy)
}

/// Materializes roots already representable by the canonical scalar tower.
///
/// Rational-coefficient polynomials retain the established Sturm carrier. For
/// a non-rational linear or quadratic, exact field division and square root
/// can produce roots directly as `Real`. Each candidate must replay its
/// defining polynomial to zero; a scalar tower that cannot certify that
/// identity falls through to the established algebraic carrier. The formula
/// uses no approximate root or inferred coefficient. APPROXIMATE_512 may only
/// terminate the replay equality, as it does elsewhere in Hypercurve.
#[cold]
#[inline(never)]
fn exact_nonrational_low_degree_unit_roots(
    coefficients: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<Classification<BezierRootIsolationResult2>>> {
    const EXACT_REFINEMENT_PRECISION: i32 = -512;

    let mut candidates = match coefficients {
        [_constant] => Vec::new(),
        [constant, linear] => {
            if !matches!(
                linear.certified_sign_until(EXACT_REFINEMENT_PRECISION),
                CertifiedRealSign::Known {
                    sign: RealSign::Positive | RealSign::Negative,
                    ..
                }
            ) {
                return Ok(None);
            }
            vec![((-constant.clone()) / linear)?]
        }
        [constant, linear, quadratic] => {
            match quadratic.certified_sign_until(EXACT_REFINEMENT_PRECISION) {
                CertifiedRealSign::Known {
                    sign: RealSign::Positive | RealSign::Negative,
                    ..
                } => {}
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                }
                | CertifiedRealSign::Unknown { .. } => return Ok(None),
            };
            let discriminant = linear * linear - Real::from(4_i8) * quadratic * constant;
            match discriminant.certified_sign_until(EXACT_REFINEMENT_PRECISION) {
                CertifiedRealSign::Known {
                    sign: RealSign::Negative,
                    ..
                } => Vec::new(),
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                } => vec![((-linear.clone()) / (Real::from(2_i8) * quadratic))?],
                CertifiedRealSign::Known {
                    sign: RealSign::Positive,
                    ..
                } => {
                    let denominator = Real::from(2_i8) * quadratic;
                    let vertex = ((-linear.clone()) / &denominator)?;
                    let delta = (discriminant / (&denominator * &denominator))?.sqrt()?;
                    vec![&vertex - &delta, vertex + delta]
                }
                CertifiedRealSign::Unknown { .. } => return Ok(None),
            }
        }
        _ => return Ok(None),
    };

    let mut roots = Vec::with_capacity(candidates.len());
    for candidate in candidates.drain(..) {
        match real_sign(&evaluate_coefficients(coefficients, &candidate), policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) | None => return Ok(None),
        }
        match in_closed_unit_interval(&candidate, policy) {
            Some(true) => roots.push(BezierParameter2::Exact(candidate)),
            Some(false) => {}
            None => return Ok(None),
        }
    }
    Ok(Some(Classification::Decided(BezierRootIsolationResult2 {
        roots,
        trace: BezierRootIsolationTrace2::default(),
    })))
}

fn ordered_root_isolation_result(
    represented: Vec<BezierParameter2>,
    trace: BezierRootIsolationTrace2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierRootIsolationResult2>> {
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
    policy: &CurveContext,
    trace: &mut BezierRootIsolationTrace2,
) -> CurveResult<Classification<UnitRootSearch>> {
    let sequence = match sturm_sequence(polynomial.coefficients(), policy)? {
        Classification::Decided(sequence) => Arc::<[Vec<Real>]>::from(sequence),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let rational_root_denominator_bound = rational_root_denominator_bound(polynomial);
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
    let mut boundary_variations = Vec::with_capacity(boundaries.len());
    for boundary in &boundaries {
        match sturm_point_evidence(&sequence, boundary, policy)? {
            Classification::Decided(SturmPointEvidence::Root) => {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            Classification::Decided(SturmPointEvidence::NonRoot(variations)) => {
                boundary_variations.push(variations);
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let mut pending = boundaries
        .windows(2)
        .zip(boundary_variations.windows(2))
        .rev()
        .map(|(pair, variations)| {
            (
                pair[0].clone(),
                pair[1].clone(),
                variations[0],
                variations[1],
                0_usize,
            )
        })
        .collect::<Vec<_>>();
    let mut isolated = Vec::new();
    while let Some((start, end, start_variations, end_variations, depth)) = pending.pop() {
        trace.maximum_depth = trace.maximum_depth.max(depth);
        let interval = match BezierParameterInterval::try_new(start.clone(), end.clone(), policy)? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let count = start_variations.saturating_sub(end_variations);
        trace.interval_root_counts += 1;
        if count == 0 {
            continue;
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
            let parameter = BezierAlgebraicParameter2::from_certified_singleton_with_sturm_sequence(
                polynomial.clone(),
                interval,
                Arc::clone(&sequence),
            );
            match parameter.represented_rational_root_with_cached_sequence(
                policy,
                rational_root_denominator_bound.as_ref(),
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
        let midpoint = ((&start + &end) / Real::from(2_i8))?;
        let midpoint_variations = match sturm_point_evidence(&sequence, &midpoint, policy)? {
            Classification::Decided(SturmPointEvidence::Root) => {
                return Ok(Classification::Decided(UnitRootSearch::RepresentedRoot(
                    midpoint,
                )));
            }
            Classification::Decided(SturmPointEvidence::NonRoot(variations)) => variations,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if depth >= 256 {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        trace.bisections += 1;
        pending.push((
            midpoint.clone(),
            end,
            midpoint_variations,
            end_variations,
            depth + 1,
        ));
        pending.push((
            start,
            midpoint,
            start_variations,
            midpoint_variations,
            depth + 1,
        ));
    }
    Ok(Classification::Decided(UnitRootSearch::Isolated(isolated)))
}

fn insert_parameter_ordered(
    parameters: &mut Vec<BezierParameter2>,
    parameter: BezierParameter2,
    policy: &CurveContext,
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

pub(crate) fn divide_by_linear_root(coefficients: &[Real], root: &Real) -> Vec<Real> {
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

    // If `b` stores the Bernstein controls, power coefficient `k` is
    // `binomial(degree, k) * Δ^k b[0]`. Building the forward-difference
    // column in place avoids the old quadratic Pascal triangle and replaces
    // every inner-loop multiplication by one subtraction. `BigUint` keeps
    // the conversion exact beyond the former `u64` binomial ceiling.
    let mut differences = values;
    let mut coefficients = Vec::with_capacity(degree + 1);
    let mut degree_binomial = BigUint::one();
    for power in 0..=degree {
        if power != 0 {
            degree_binomial *= BigUint::from(degree - power + 1);
            degree_binomial /= BigUint::from(power);
            for index in 0..=degree - power {
                differences[index] = &differences[index + 1] - &differences[index];
            }
        }
        coefficients.push(&differences[0] * exact_nonnegative_integer_real(&degree_binomial)?);
    }
    Ok(coefficients)
}

pub(crate) fn power_to_bernstein_coefficients(
    coefficients: &[Real],
    degree: usize,
) -> CurveResult<Vec<Real>> {
    if coefficients.len() > degree + 1 {
        return Err(CurveError::InvalidDegreeElevation);
    }
    // Selected-root predicates overwhelmingly transform affine and quadratic
    // axes.  Keep those conversions out of the arbitrary-degree BigUint
    // machinery: the closed forms are the same exact basis change and retain
    // the general path for every larger (including beyond-u64) binomial.
    let coefficient = |power| coefficients.get(power).cloned().unwrap_or_else(Real::zero);
    if degree == 0 {
        return Ok(vec![coefficient(0)]);
    }
    if degree == 1 {
        let constant = coefficient(0);
        let linear = coefficient(1);
        return Ok(vec![constant.clone(), &constant + linear]);
    }
    if degree == 2 {
        let constant = coefficient(0);
        let linear = coefficient(1);
        let quadratic = coefficient(2);
        let middle = &constant + (linear.clone() / Real::from(2_i8))?;
        let end = &constant + &linear + quadratic;
        return Ok(vec![constant, middle, end]);
    }
    let mut degree_binomials = Vec::with_capacity(degree + 1);
    let mut binomial = BigUint::one();
    for index in 0..=degree {
        degree_binomials.push(exact_nonnegative_integer_real(&binomial)?);
        if index != degree {
            binomial *= BigUint::from(degree - index);
            binomial /= BigUint::from(index + 1);
        }
    }

    let mut bernstein = Vec::with_capacity(degree + 1);
    let mut row = vec![BigUint::one()];
    for index in 0..=degree {
        if index != 0 {
            row.push(BigUint::one());
            for power in (1..index).rev() {
                row[power] = &row[power - 1] + &row[power];
            }
        }
        let mut value = Real::zero();
        for (power, coefficient) in coefficients.iter().enumerate().take(index + 1) {
            let numerator = exact_nonnegative_integer_real(&row[power])?;
            value = &value + coefficient * (numerator / &degree_binomials[power])?;
        }
        bernstein.push(value);
    }
    Ok(bernstein)
}

pub(crate) fn subdivide_scalar_bernstein_half(
    controls: &[Real],
) -> CurveResult<(Vec<Real>, Vec<Real>)> {
    if controls.is_empty() {
        return Err(CurveError::InvalidBezierPolynomial);
    }

    let degree = controls.len() - 1;
    let mut work = controls.to_vec();
    let mut left = Vec::with_capacity(controls.len());
    let mut right = Vec::with_capacity(controls.len());
    left.push(work[0].clone());
    right.push(work[degree].clone());
    for level in 1..=degree {
        for index in 0..=degree - level {
            work[index] = midpoint_real(&work[index], &work[index + 1])?;
        }
        left.push(work[0].clone());
        right.push(work[degree - level].clone());
    }
    right.reverse();
    Ok((left, right))
}

pub(crate) fn exact_nonnegative_integer_real(value: &BigUint) -> CurveResult<Real> {
    if let Some(value) = value.to_u64() {
        return Ok(Real::from(value));
    }
    HyperRational::from_bigint_fraction(BigInt::from(value.clone()), BigUint::one())
        .map(Real::new)
        .map_err(Into::into)
}

#[cfg(test)]
mod conversion_tests {
    use super::*;

    fn rational(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn polynomial(coefficients: &[i32]) -> BezierParameterPolynomial {
        match BezierParameterPolynomial::try_new_power_basis(
            coefficients.iter().copied().map(Real::from).collect(),
            &CurveContext::STRICT,
        )
        .unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                panic!("polynomial unexpectedly uncertain: {reason:?}")
            }
        }
    }

    fn algebraic_parameter(polynomial: &BezierParameterPolynomial) -> BezierParameter2 {
        let policy = CurveContext::STRICT;
        let interval =
            match BezierParameterInterval::try_new(rational(1, 2), Real::one(), &policy).unwrap() {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    panic!("interval unexpectedly uncertain: {reason:?}")
                }
            };
        match BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval, &policy).unwrap()
        {
            Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
            Classification::Uncertain(reason) => {
                panic!("parameter unexpectedly uncertain: {reason:?}")
            }
        }
    }

    fn simple_root_classification(
        polynomial: &BezierParameterPolynomial,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        polynomial
            .simple_root_classifications(std::slice::from_ref(parameter), policy)
            .unwrap()
            .pop()
            .expect("one parameter produces one classification")
    }

    fn primitive_ratio(coefficients: &[Real]) -> Vec<Real> {
        let rationals = coefficients
            .iter()
            .map(Real::exact_rational_ref)
            .collect::<Option<Vec<_>>>()
            .expect("test coefficients are rational");
        HyperRational::primitive_bigint_ratio(&rationals)
            .into_iter()
            .map(HyperRational::from_bigint)
            .map(Real::from)
            .collect()
    }

    fn ordinary_field_sturm_sequence(
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> Vec<Vec<Real>> {
        let p0 = coefficients.to_vec();
        let p1 =
            match normalize_coefficients(derivative_coefficients(coefficients), policy).unwrap() {
                Classification::Decided(Some(coefficients)) => coefficients,
                Classification::Decided(None) => return vec![p0],
                Classification::Uncertain(reason) => {
                    panic!("field Sturm derivative unexpectedly uncertain: {reason:?}")
                }
            };
        let mut sequence = vec![p0, p1];
        while sequence.len() < 64 {
            let previous = sequence[sequence.len() - 2].clone();
            let remainder = match polynomial_remainder(
                previous,
                &sequence[sequence.len() - 1],
                policy,
            )
            .unwrap()
            {
                Classification::Decided(Some(remainder)) => remainder,
                Classification::Decided(None) => break,
                Classification::Uncertain(reason) => {
                    panic!("field Sturm remainder unexpectedly uncertain: {reason:?}")
                }
            };
            sequence.push(negate_coefficients(remainder));
        }
        sequence
    }

    fn sturm_evidence_key(evidence: SturmPointEvidence) -> Option<usize> {
        match evidence {
            SturmPointEvidence::Root => None,
            SturmPointEvidence::NonRoot(variations) => Some(variations),
        }
    }

    fn decided<T>(classification: Classification<T>, context: &str) -> T {
        match classification {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("{context} unexpectedly uncertain: {reason:?}")
            }
        }
    }

    #[test]
    fn incident_ray_isolation_orders_exact_roots_away_from_the_anchor() {
        let policy = CurveContext::STRICT;
        let increasing = decided(
            polynomial(&[6, -5, 1])
                .isolate_incident_ray_roots(
                    &Real::one(),
                    BezierParameterRayDirection2::Increasing,
                    &policy,
                )
                .unwrap(),
            "increasing incident ray",
        );
        assert_eq!(
            increasing,
            vec![
                BezierParameter2::Exact(Real::from(2_i8)),
                BezierParameter2::Exact(Real::from(3_i8)),
            ]
        );

        let decreasing = decided(
            polynomial(&[2, 3, 1])
                .isolate_incident_ray_roots(
                    &Real::zero(),
                    BezierParameterRayDirection2::Decreasing,
                    &policy,
                )
                .unwrap(),
            "decreasing incident ray",
        );
        assert_eq!(
            decreasing,
            vec![
                BezierParameter2::Exact(Real::from(-1_i8)),
                BezierParameter2::Exact(Real::from(-2_i8)),
            ]
        );
    }

    #[test]
    fn incident_ray_isolation_retains_irrational_exterior_root_authority() {
        let source = polynomial(&[-2, 0, 1]);
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let roots = decided(
                source
                    .isolate_incident_ray_roots(
                        &Real::one(),
                        BezierParameterRayDirection2::Increasing,
                        &policy,
                    )
                    .unwrap(),
                "irrational incident ray",
            );
            let [BezierParameter2::Algebraic(root)] = roots.as_slice() else {
                panic!("the exterior irrational root must retain algebraic evidence")
            };
            assert_eq!(root.polynomial(), &source);
            assert_eq!(
                compare_reals(root.interval().start(), &Real::one(), &policy),
                Some(Ordering::Greater)
            );
            assert_eq!(
                source.simple_root_classifications(&roots, &policy).unwrap(),
                vec![Classification::Decided(true)]
            );
        }
    }

    #[test]
    fn algebraic_unit_complement_preserves_the_exact_singleton_root() {
        let policy = CurveContext::STRICT;
        let parameter = algebraic_parameter(&polynomial(&[-1, 0, 2]));
        let complemented = parameter.unit_complement();
        let BezierParameter2::Algebraic(complemented_algebraic) = &complemented else {
            panic!("an irrational parameter must remain algebraic")
        };
        assert_eq!(
            complemented_algebraic.polynomial().coefficients(),
            &[Real::one(), Real::from(-4_i8), Real::from(2_i8)]
        );
        assert_eq!(complemented_algebraic.interval().start(), &Real::zero());
        assert_eq!(complemented_algebraic.interval().end(), &rational(1, 2));
        let restored = complemented.unit_complement();
        assert!(decided(
            restored.same_value(&parameter, &policy).unwrap(),
            "double-complement equality"
        ));
    }

    #[test]
    fn primitive_pseudo_remainder_is_positive_field_remainder_scale() {
        let policy = CurveContext::STRICT;
        for (dividend, divisor) in [
            (
                vec![
                    rational(2, 3),
                    rational(-5, 4),
                    rational(3, 2),
                    rational(-7, 3),
                ],
                vec![rational(1, 5), rational(-2, 3)],
            ),
            (
                vec![
                    rational(-3, 7),
                    rational(4, 9),
                    rational(5, 6),
                    rational(-2, 5),
                    rational(8, 3),
                ],
                vec![rational(2, 11), rational(3, 5), rational(-4, 7)],
            ),
            (
                vec![rational(-2, 3), rational(1, 3), rational(1, 3)],
                vec![rational(-1, 2), rational(1, 2)],
            ),
            (
                vec![rational(1, 3), rational(2, 5), rational(3, 7)],
                vec![rational(2, 9), rational(-1, 4)],
            ),
        ] {
            let expected = match polynomial_remainder(dividend.clone(), &divisor, &policy).unwrap()
            {
                Classification::Decided(Some(remainder)) => primitive_ratio(&remainder),
                Classification::Decided(None) => Vec::new(),
                Classification::Uncertain(reason) => {
                    panic!("field remainder unexpectedly uncertain: {reason:?}")
                }
            };
            assert_eq!(
                primitive_integer_pseudo_remainder(&dividend, &divisor),
                Some(expected)
            );
        }
    }

    #[test]
    fn primitive_integer_sturm_matches_field_sequence_variations() {
        let policy = CurveContext::STRICT;
        let polynomials = [
            vec![rational(-1, 3), rational(0, 1), rational(2, 5)],
            vec![
                rational(1, 2),
                rational(-7, 3),
                rational(4, 5),
                rational(9, 7),
            ],
            vec![
                rational(-5, 11),
                rational(0, 1),
                rational(13, 6),
                rational(0, 1),
                rational(-3, 2),
            ],
            vec![
                rational(1, 4),
                rational(-1, 1),
                rational(3, 2),
                rational(-1, 1),
                rational(1, 4),
            ],
        ];
        let samples = [
            rational(0, 1),
            rational(1, 8),
            rational(1, 3),
            rational(1, 2),
            rational(5, 7),
            rational(1, 1),
        ];
        for coefficients in polynomials {
            let optimized = decided(
                sturm_sequence(&coefficients, &policy).unwrap(),
                "rational Sturm sequence",
            );
            let field = ordinary_field_sturm_sequence(&coefficients, &policy);
            for sample in &samples {
                let optimized = decided(
                    sturm_point_evidence(&optimized, sample, &policy).unwrap(),
                    "integer Sturm evidence",
                );
                let field = decided(
                    sturm_point_evidence(&field, sample, &policy).unwrap(),
                    "field Sturm evidence",
                );
                assert_eq!(
                    sturm_evidence_key(optimized),
                    sturm_evidence_key(field),
                    "different variation evidence for {coefficients:?} at {sample:?}"
                );
            }
        }
    }

    #[test]
    fn primitive_integer_sturm_sequence_has_no_historical_degree_cap() {
        let policy = CurveContext::STRICT;
        let mut previous = vec![Real::one()];
        let mut current = vec![Real::zero(), Real::one()];
        for _ in 2..=70 {
            let mut next = vec![Real::zero(); current.len() + 1];
            for (power, coefficient) in current.iter().enumerate() {
                next[power + 1] += Real::from(2_i8) * coefficient;
            }
            for (power, coefficient) in previous.iter().enumerate() {
                next[power] -= coefficient;
            }
            previous = current;
            current = next;
        }
        let sequence = primitive_integer_sturm_sequence(&current)
            .expect("the degree-seventy Chebyshev polynomial has an integer Sturm chain");
        assert!(sequence.len() > 64);
        assert_eq!(sequence.last().map(Vec::len), Some(1));
        let start = decided(
            sturm_point_evidence(&sequence, &Real::zero(), &policy).unwrap(),
            "Chebyshev Sturm start",
        );
        let end = decided(
            sturm_point_evidence(&sequence, &Real::one(), &policy).unwrap(),
            "Chebyshev Sturm end",
        );
        let (SturmPointEvidence::NonRoot(start), SturmPointEvidence::NonRoot(end)) = (start, end)
        else {
            panic!("the degree-seventy Chebyshev endpoints are not roots")
        };
        assert_eq!(start - end, 35);
    }

    #[test]
    fn carried_sturm_variations_match_partition_root_counts() {
        let policy = CurveContext::STRICT;
        // (2t² - 1)(3t² - 1) has two irrational roots in (1/2, 3/4).
        let defining = polynomial(&[1, 0, -5, 0, 6]);
        let sequence = match sturm_sequence(defining.coefficients(), &policy).unwrap() {
            Classification::Decided(sequence) => sequence,
            Classification::Uncertain(reason) => {
                panic!("Sturm sequence unexpectedly uncertain: {reason:?}")
            }
        };
        let boundaries = [Real::zero(), rational(1, 2), rational(3, 4), Real::one()];
        let variations = boundaries
            .iter()
            .map(
                |boundary| match sturm_point_evidence(&sequence, boundary, &policy).unwrap() {
                    Classification::Decided(SturmPointEvidence::NonRoot(variations)) => variations,
                    Classification::Decided(SturmPointEvidence::Root) => {
                        panic!("partition boundary unexpectedly is a root")
                    }
                    Classification::Uncertain(reason) => {
                        panic!("Sturm evidence unexpectedly uncertain: {reason:?}")
                    }
                },
            )
            .collect::<Vec<_>>();

        for (partition, endpoint_variations) in boundaries.windows(2).zip(variations.windows(2)) {
            let interval = match BezierParameterInterval::try_new(
                partition[0].clone(),
                partition[1].clone(),
                &policy,
            )
            .unwrap()
            {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    panic!("partition interval unexpectedly uncertain: {reason:?}")
                }
            };
            assert_eq!(
                defining.root_count_in_interval(&interval, &policy).unwrap(),
                Classification::Decided(
                    endpoint_variations[0].saturating_sub(endpoint_variations[1])
                )
            );
        }

        let linear = polynomial(&[-1, 2]);
        let linear_sequence = match sturm_sequence(linear.coefficients(), &policy).unwrap() {
            Classification::Decided(sequence) => sequence,
            Classification::Uncertain(reason) => {
                panic!("linear Sturm sequence unexpectedly uncertain: {reason:?}")
            }
        };
        assert!(matches!(
            sturm_point_evidence(&linear_sequence, &rational(1, 2), &policy).unwrap(),
            Classification::Decided(SturmPointEvidence::Root)
        ));
    }

    #[test]
    fn bernstein_simple_root_certificates_avoid_multiplicity_sturm_rebuilds() {
        let policy = CurveContext::STRICT;
        let pi = Real::pi();
        let defining = BezierParameterPolynomial::try_new_power_basis(
            vec![
                pi.clone(),
                Real::zero(),
                &pi * Real::from(-5),
                Real::zero(),
                &pi * Real::from(6),
            ],
            &policy,
        )
        .unwrap();
        let defining = decided(defining, "non-rational quartic");
        let result = decided(
            defining
                .isolate_unit_interval_roots_with_trace(&policy)
                .unwrap(),
            "Bernstein isolation",
        );
        assert_eq!(result.trace().sturm_sequence_builds(), 0);
        assert_eq!(result.roots().len(), 2);
        for root in result.roots() {
            let BezierParameter2::Algebraic(parameter) = root else {
                panic!("Bernstein singleton should remain algebraic");
            };
            assert!(parameter.data.shared.sturm_sequence.get().is_none());
        }

        assert_eq!(
            defining
                .simple_root_classifications(result.roots(), &policy)
                .unwrap(),
            vec![Classification::Decided(true), Classification::Decided(true)]
        );
        for root in result.roots() {
            let BezierParameter2::Algebraic(parameter) = root else {
                unreachable!();
            };
            assert!(
                parameter.data.shared.sturm_sequence.get().is_none(),
                "the Descartes singleton certificate must carry simple-root evidence"
            );
        }
    }

    #[test]
    fn rational_basis_subdivision_matches_the_shared_bernstein_kernel() {
        let coefficients = [
            rational(-23, 17),
            rational(29, 19),
            rational(-31, 23),
            rational(37, 29),
            rational(-41, 31),
        ];
        let basis = power_to_rational_bernstein_basis(4).unwrap();
        let controls = basis
            .iter()
            .map(|control| rational_bernstein_basis_value(&coefficients, control))
            .collect::<Vec<_>>();
        assert_eq!(
            controls.as_slice(),
            power_to_bernstein_coefficients(&coefficients, 4).unwrap()
        );

        let controls = [
            rational(-7, 3),
            rational(11, 5),
            rational(-13, 7),
            rational(17, 11),
            rational(-19, 13),
        ];
        let expected = subdivide_scalar_bernstein_half(&controls).unwrap();
        let coefficients = bernstein_to_power_coefficients(controls.to_vec()).unwrap();
        let basis = power_to_rational_bernstein_basis(4).unwrap();
        let (left_basis, right_basis) = subdivide_rational_bernstein_half(&basis);
        let left = left_basis
            .iter()
            .map(|control| rational_bernstein_basis_value(&coefficients, control))
            .collect::<Vec<_>>();
        let right = right_basis
            .iter()
            .map(|control| rational_bernstein_basis_value(&coefficients, control))
            .collect::<Vec<_>>();

        assert_eq!(left.as_slice(), expected.0);
        assert_eq!(right.as_slice(), expected.1);

        let pi = Real::pi();
        let nonrational_coefficients: [Real; 5] =
            std::array::from_fn(|index| &pi * &coefficients[index]);
        let intervals = coefficient_intervals(&nonrational_coefficients, -32).unwrap();
        for control in basis {
            assert_eq!(
                rational_bernstein_basis_sign(
                    &nonrational_coefficients,
                    &intervals,
                    &control,
                    &CurveContext::STRICT,
                ),
                real_sign(
                    &rational_bernstein_basis_value(&nonrational_coefficients, &control),
                    &CurveContext::STRICT,
                ),
            );
        }
    }

    #[test]
    fn arbitrary_degree_nonrational_bernstein_isolates_every_simple_root() {
        let rational_coefficients = [3_i32, 5, 7, 9, 11, 13].into_iter().fold(
            vec![Real::one()],
            |coefficients, denominator| {
                let mut product = vec![Real::zero(); coefficients.len() + 1];
                for (degree, coefficient) in coefficients.iter().enumerate() {
                    product[degree] = &product[degree] - coefficient;
                    product[degree + 1] =
                        &product[degree + 1] + coefficient * Real::from(denominator);
                }
                product
            },
        );
        let pi = Real::pi();
        let coefficients = rational_coefficients
            .iter()
            .map(|coefficient| &pi * coefficient)
            .collect::<Vec<_>>();
        let expected = [13_i32, 11, 9, 7, 5, 3].map(|denominator| rational(1, denominator));

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = decided(
                BezierParameterPolynomial::try_new_power_basis(coefficients.clone(), &policy)
                    .unwrap(),
                "degree-six non-rational polynomial",
            );
            let result = decided(
                polynomial
                    .isolate_unit_interval_roots_with_trace(&policy)
                    .unwrap(),
                "degree-six Bernstein isolation",
            );
            assert_eq!(result.trace().sturm_sequence_builds(), 0);
            assert_eq!(result.roots().len(), expected.len());
            for (root, expected) in result.roots().iter().zip(&expected) {
                let BezierParameter2::Algebraic(root) = root else {
                    panic!("a Bernstein singleton must retain its algebraic certificate");
                };
                assert_eq!(
                    compare_reals(root.interval().start(), expected, &policy),
                    Some(Ordering::Less)
                );
                assert_eq!(
                    compare_reals(root.interval().end(), expected, &policy),
                    Some(Ordering::Greater)
                );
            }
            assert_eq!(
                polynomial
                    .simple_root_classifications(result.roots(), &policy)
                    .unwrap(),
                vec![Classification::Decided(true); expected.len()]
            );
        }
    }

    #[test]
    fn square_free_degree_128_bernstein_isolates_every_unit_root() {
        // (9t² - 9t + 2)(t^126 + 1) is square-free and has exactly the
        // rational roots 1/3 and 2/3 in the unit interval. Its degree exceeds
        // the former 64-entry Sturm schedule that omitted valid roots from
        // arbitrary-rank curve norms.
        let policy = CurveContext::STRICT;
        let mut coefficients = vec![Real::zero(); 129];
        for (power, coefficient) in [2_i16, -9, 9].into_iter().enumerate() {
            coefficients[power] = Real::from(coefficient);
            coefficients[126 + power] = Real::from(coefficient);
        }
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(coefficients, &policy).unwrap(),
            "degree-128 square-free polynomial",
        );
        let roots = decided(
            polynomial
                .isolate_square_free_unit_interval_roots(&policy)
                .unwrap(),
            "degree-128 Bernstein roots",
        );
        assert_eq!(roots.len(), 2);
        for (root, expected) in roots.iter().zip([rational(1, 3), rational(2, 3)]) {
            let BezierParameter2::Algebraic(root) = root else {
                panic!("a Bernstein singleton retains algebraic evidence")
            };
            assert_eq!(
                compare_reals(root.interval().start(), &expected, &policy),
                Some(Ordering::Less)
            );
            assert_eq!(
                compare_reals(root.interval().end(), &expected, &policy),
                Some(Ordering::Greater)
            );
        }
    }

    #[test]
    fn simple_root_certificate_distinguishes_exact_multiplicity() {
        let policy = CurveContext::STRICT;
        let root = BezierParameter2::Exact(rational(1, 2));

        assert_eq!(
            simple_root_classification(&polynomial(&[-1, 2]), &root, &policy),
            Classification::Decided(true)
        );
        assert_eq!(
            simple_root_classification(&polynomial(&[1, -4, 4]), &root, &policy),
            Classification::Decided(false)
        );
    }

    #[test]
    fn simple_root_certificate_distinguishes_algebraic_multiplicity() {
        let policy = CurveContext::STRICT;
        let simple = polynomial(&[-1, 0, 2]);
        let simple_root = algebraic_parameter(&simple);
        let repeated = polynomial(&[1, 0, -4, 0, 4]);
        let repeated_root = algebraic_parameter(&repeated);

        assert_eq!(
            simple_root_classification(&simple, &simple_root, &policy),
            Classification::Decided(true)
        );
        assert_eq!(
            simple_root_classification(&repeated, &repeated_root, &policy),
            Classification::Decided(false)
        );
    }

    #[test]
    fn simple_root_certificate_accepts_an_endpoint_deflated_algebraic_carrier() {
        let policy = CurveContext::STRICT;
        // t(2t² - 1) has a represented endpoint root and one nonrational
        // interior root. Isolation deflates t before retaining the algebraic
        // carrier, without changing the interior root's multiplicity.
        let source_polynomial = polynomial(&[0, -1, 0, 2]);
        let roots = match source_polynomial
            .isolate_unit_interval_roots(&policy)
            .unwrap()
        {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                panic!("roots unexpectedly uncertain: {reason:?}")
            }
        };

        assert_eq!(roots.len(), 2);
        assert!(matches!(&roots[0], BezierParameter2::Exact(root) if root == &Real::zero()));
        assert!(matches!(&roots[1], BezierParameter2::Algebraic(_)));
        assert_eq!(
            source_polynomial
                .simple_root_classifications(&roots, &policy)
                .unwrap(),
            vec![Classification::Decided(true), Classification::Decided(true)]
        );

        let unrelated = polynomial(&[-1, 0, 3]);
        let unrelated_root = algebraic_parameter(&unrelated);
        assert_eq!(
            source_polynomial.simple_root_classifications(&[unrelated_root], &policy),
            Err(CurveError::InvalidBezierAlgebraicParameter)
        );
    }

    #[test]
    fn retained_sturm_certificate_classifies_mixed_root_multiplicity() {
        let policy = CurveContext::STRICT;
        // (2t² - 1)²(8t² - 1) has one simple and one repeated root in
        // the unit interval, neither representable as a rational scalar.
        let polynomial = polynomial(&[-1, 0, 12, 0, -36, 0, 32]);
        let roots = match polynomial.isolate_unit_interval_roots(&policy).unwrap() {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                panic!("roots unexpectedly uncertain: {reason:?}")
            }
        };

        assert_eq!(roots.len(), 2);
        assert!(
            roots
                .iter()
                .all(|root| matches!(root, BezierParameter2::Algebraic(_)))
        );
        assert!(roots.iter().all(|root| match root {
            BezierParameter2::Algebraic(parameter) =>
                parameter.data.shared.sturm_sequence.get().is_some(),
            BezierParameter2::Exact(_) => false,
        }));
        assert_eq!(
            polynomial
                .simple_root_classifications(&roots, &policy)
                .unwrap(),
            vec![
                Classification::Decided(true),
                Classification::Decided(false)
            ]
        );
    }

    #[test]
    fn retained_refinement_matches_square_free_reference_and_shares_sturm_work() {
        let policy = CurveContext::STRICT;
        for (defining, expects_sturm_fallback) in [
            (polynomial(&[-1, 0, 2]), false),
            (polynomial(&[1, 0, -4, 0, 4]), true),
        ] {
            let source = algebraic_parameter(&defining);
            let BezierParameter2::Algebraic(source_algebraic) = &source else {
                panic!("test helper always constructs an algebraic parameter");
            };
            assert!(source_algebraic.data.shared.sturm_sequence.get().is_none());

            let reference = hypersolve::refine_isolated_univariate_polynomial_interval(
                defining.coefficients(),
                &hypersolve::IsolatedRootInterval {
                    lower: source_algebraic.interval().start().clone(),
                    upper: source_algebraic.interval().end().clone(),
                    exact_root: None,
                    distinct_root_count: 1,
                },
                hypersolve::RootIsolationConfig {
                    policy: policy.predicate_policy(),
                    max_interval_width: None,
                    max_refinement_steps: 3,
                },
            )
            .refined_interval
            .expect("the reference refinement succeeds");
            let refined = source.clone().refined_isolating_interval(3, &policy);
            let BezierParameter2::Algebraic(refined) = refined else {
                panic!("the irrational test roots remain algebraic");
            };
            assert_ne!(source_algebraic.interval(), refined.interval());
            assert_ne!(source, BezierParameter2::Algebraic(refined.clone()));
            assert_eq!(
                source
                    .same_value(&BezierParameter2::Algebraic(refined.clone()), &policy)
                    .unwrap(),
                Classification::Decided(true)
            );
            assert_eq!(refined.interval().start(), &reference.lower);
            assert_eq!(refined.interval().end(), &reference.upper);

            let source_sequence = source_algebraic.data.shared.sturm_sequence.get();
            let refined_sequence = refined.data.shared.sturm_sequence.get();
            if expects_sturm_fallback {
                assert!(Arc::ptr_eq(
                    source_sequence.expect("even root refinement retains one Sturm sequence"),
                    refined_sequence.expect("refined clones share retained Sturm work")
                ));
            } else {
                assert!(source_sequence.is_none());
                assert!(refined_sequence.is_none());
            }
        }
    }

    #[test]
    fn progressive_refinement_matches_one_pass_proof_budget() {
        let policy = CurveContext::STRICT;
        let source = algebraic_parameter(&polynomial(&[-1, 0, 2]));
        let direct = source.clone().refined_isolating_interval(8, &policy);
        let mut progressive = BezierParameterRefinement2::new(&source, &policy);

        assert_eq!(progressive.refine_to(0), &source);
        let _ = progressive.refine_to(2);
        let _ = progressive.refine_to(4);
        assert_eq!(progressive.refine_to(8), &direct);
    }

    #[test]
    fn modular_sieve_rejects_only_rootless_rational_reductions() {
        assert_eq!(
            rational_root_denominator_bound(&polynomial(&[-1, 0, 2])),
            None
        );
        assert_eq!(
            rational_root_denominator_bound(&polynomial(&[1, -4, 4])),
            Some(BigUint::from(4_u8))
        );
        // (2t - 1)(2t² - 1) retains reconstruction because one of its
        // three roots is rational.
        assert_eq!(
            rational_root_denominator_bound(&polynomial(&[1, -2, -2, 4])),
            Some(BigUint::from(4_u8))
        );
    }

    #[test]
    fn native_small_modulus_matches_bigint_floor_remainder() {
        let wide = (BigInt::one() << 200_usize) + BigInt::from(123_456_789_u64);
        let values = [
            BigInt::zero(),
            BigInt::from(1_i8),
            BigInt::from(-1_i8),
            BigInt::from(i128::MAX),
            BigInt::from(i128::MIN),
            wide.clone(),
            -wide,
        ];
        for prime in [2_u32, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31] {
            let modulus = BigInt::from(prime);
            for value in &values {
                assert_eq!(
                    bigint_modulo_u32(value, prime),
                    value
                        .mod_floor(&modulus)
                        .to_u32()
                        .expect("small-prime residue fits u32")
                );
            }
        }
    }

    #[test]
    fn integer_polynomial_sign_matches_exact_rational_evaluation() {
        let policy = CurveContext::STRICT;
        let wide = (BigInt::one() << 200_usize) + BigInt::from(123_456_789_u64);
        let polynomials = [
            vec![rational(-2, 1), rational(0, 1), rational(1, 1)],
            vec![
                Real::new(HyperRational::from_bigint(-wide.clone())),
                rational(7, 1),
                Real::new(HyperRational::from_bigint(wide)),
            ],
            vec![rational(0, 1)],
        ];
        let parameters = [
            rational(-3, 7),
            rational(0, 1),
            rational(1, 2),
            rational(5, 3),
        ];
        for polynomial in &polynomials {
            for parameter in &parameters {
                assert_eq!(
                    exact_integer_polynomial_sign(polynomial, parameter),
                    real_sign(&evaluate_coefficients(polynomial, parameter), &policy)
                );
            }
        }
        assert_eq!(
            exact_integer_polynomial_sign(&[rational(1, 2)], &Real::zero()),
            None
        );
    }

    #[test]
    fn nonrational_power_evaluation_uses_exact_endpoint_identities() {
        let coefficients = [
            Real::pi(),
            Real::e(),
            Real::from(2_i32).sqrt().unwrap(),
            Real::from(3_i32).ln().unwrap(),
            Real::from(5_i32).sqrt().unwrap(),
        ];

        assert_eq!(
            evaluate_coefficients(&coefficients, &Real::zero()),
            coefficients[0]
        );
        assert_eq!(
            evaluate_coefficients(&coefficients, &Real::one()),
            Real::sum_refs(&coefficients)
        );
    }

    #[test]
    fn nonrational_low_degree_roots_materialize_without_sturm_work() {
        let alpha = (Real::one() / Real::from(8_i8)).unwrap().sqrt().unwrap();
        let half = rational(1, 2);
        let quarter = rational(1, 4);
        let cases = [
            (
                vec![-(&alpha * &half), alpha.clone()],
                vec![half.clone()],
                vec![Classification::Decided(true)],
            ),
            (
                vec![rational(1, 16), Real::from(-2_i8) * &alpha, Real::one()],
                vec![&alpha - &quarter, &alpha + &quarter],
                vec![Classification::Decided(true), Classification::Decided(true)],
            ),
            (
                vec![rational(1, 8), Real::from(-2_i8) * &alpha, Real::one()],
                vec![alpha.clone()],
                vec![Classification::Decided(false)],
            ),
            (
                vec![rational(3, 16), Real::from(-2_i8) * &alpha, Real::one()],
                Vec::new(),
                Vec::new(),
            ),
            (
                vec![rational(-7, 8), Real::from(-2_i8) * &alpha, Real::one()],
                Vec::new(),
                Vec::new(),
            ),
        ];

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (coefficients, expected_roots, expected_simple) in &cases {
                let polynomial = decided(
                    BezierParameterPolynomial::try_new_power_basis(coefficients.clone(), &policy)
                        .unwrap(),
                    "non-rational low-degree polynomial",
                );
                let result = decided(
                    polynomial
                        .isolate_unit_interval_roots_with_trace(&policy)
                        .unwrap(),
                    "non-rational low-degree roots",
                );
                assert_eq!(result.trace(), &BezierRootIsolationTrace2::default());
                assert_eq!(result.roots().len(), expected_roots.len());
                for (root, expected) in result.roots().iter().zip(expected_roots) {
                    let BezierParameter2::Exact(root) = root else {
                        panic!("a scalar-tower root retained an algebraic wrapper");
                    };
                    assert_eq!(real_sign(&(root - expected), &policy), Some(RealSign::Zero));
                    assert_eq!(
                        real_sign(&polynomial.evaluate(root), &policy),
                        Some(RealSign::Zero)
                    );
                }
                assert_eq!(
                    polynomial
                        .simple_root_classifications(result.roots(), &policy)
                        .unwrap(),
                    *expected_simple
                );
            }
        }
    }

    #[test]
    fn nested_nonrational_quadratic_root_retains_its_polynomial_certificate() {
        let alpha = (Real::one() / Real::from(2_i8)).unwrap().sqrt().unwrap();
        let coefficients = vec![-alpha, Real::one(), Real::one()];

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = decided(
                BezierParameterPolynomial::try_new_power_basis(coefficients.clone(), &policy)
                    .unwrap(),
                "nested-radical polynomial",
            );
            let roots = decided(
                polynomial.isolate_unit_interval_roots(&policy).unwrap(),
                "nested-radical root",
            );
            let [root] = roots.as_slice() else {
                panic!("the nested-radical polynomial must have one unit root");
            };
            match root {
                BezierParameter2::Exact(root) => assert_eq!(
                    real_sign(&polynomial.evaluate(root), &policy),
                    Some(RealSign::Zero)
                ),
                BezierParameter2::Algebraic(_) => {}
            }
            if policy == CurveContext::STRICT {
                assert!(matches!(root, BezierParameter2::Algebraic(_)));
            }
            assert_eq!(
                polynomial
                    .simple_root_classifications(&roots, &policy)
                    .unwrap(),
                vec![Classification::Decided(true)]
            );
        }
    }

    #[test]
    fn modular_sieve_retains_polynomials_with_known_rational_roots() {
        for denominator in 1_i32..=8 {
            for numerator in 0_i32..=denominator {
                for quotient in [[3, -5, 2].as_slice(), &[-7, 0, 4], &[1]] {
                    let mut coefficients = vec![0_i32; quotient.len() + 1];
                    for (degree, coefficient) in quotient.iter().enumerate() {
                        coefficients[degree] -= numerator * coefficient;
                        coefficients[degree + 1] += denominator * coefficient;
                    }
                    assert!(
                        rational_root_denominator_bound(&polynomial(&coefficients)).is_some(),
                        "known root {numerator}/{denominator} was rejected for {coefficients:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn bernstein_to_power_remains_exact_beyond_u64_binomials() {
        let degree = 80_usize;
        let coefficients = bernstein_to_power_coefficients(
            (0..=degree)
                .map(|index| Real::from(u64::try_from(index).unwrap()))
                .collect(),
        )
        .unwrap();
        let policy = CurveContext::STRICT;

        assert_eq!(coefficients.len(), degree + 1);
        assert_eq!(
            compare_reals(&coefficients[0], &Real::zero(), &policy),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &coefficients[1],
                &Real::from(u64::try_from(degree).unwrap()),
                &policy
            ),
            Some(Ordering::Equal)
        );
        assert!(coefficients[2..].iter().all(|coefficient| {
            compare_reals(coefficient, &Real::zero(), &policy) == Some(Ordering::Equal)
        }));
    }
}
