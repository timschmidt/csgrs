//! Exact real-root counts for polynomial fibers over one represented algebraic root.
//!
//! A bivariate subresultant becomes a univariate fiber polynomial after one
//! parameter is fixed to an algebraic root. Its coefficients then live in the
//! local field at that root, not in `Real`. This module represents those
//! coefficients as reduced rational functions of the retained parameter and
//! runs an ordinary Sturm sequence in the fiber parameter. The construction
//! counts distinct roots, including roots of even multiplicity, without
//! sampling either algebraic value.

use std::cmp::Ordering;

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperreal::{Real, ZeroKnowledge};

use crate::algebraic::{
    AlgebraicRootKind, AlgebraicRootPolynomialEvaluationReport,
    AlgebraicRootPolynomialEvaluationStatus, AlgebraicRootRepresentation,
    evaluate_polynomial_at_algebraic_root, validate_algebraic_root_representation,
};
use crate::curve_resultant::{BivariatePolynomial, CurveResultantParameter};
use crate::root_isolation::{
    IsolatedRootRefinementStatus, RootIsolationConfig, polynomial_div_rem,
    polynomials_share_one_root_in_interval, refine_isolated_univariate_polynomial_interval,
};

/// Final status for an algebraic-parameter fiber root count.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberRootCountStatus {
    /// The number of distinct roots in the open interval was certified.
    Counted,
    /// Every specialized fiber coefficient vanishes at the retained root.
    IdenticallyZeroFiber,
    /// A fiber root lies exactly on an interval endpoint, where this bounded
    /// open-interval contract intentionally does not choose ownership.
    EndpointRoot,
    /// The retained algebraic-root representation is invalid.
    InvalidEvidence,
    /// The requested fiber interval is empty or reversed.
    InvalidInterval,
    /// Exact-rational coefficient arithmetic is required by this local-field
    /// package.
    UnsupportedCoefficient,
    /// Exact arithmetic or a predicate did not complete.
    Undecided,
}

/// Exact report for counting roots of a bivariate polynomial fiber.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AlgebraicFiberRootCountReport {
    /// Final root-count status.
    pub status: AlgebraicFiberRootCountStatus,
    /// Certified number of distinct roots when `status == Counted`.
    pub distinct_root_count: Option<usize>,
    /// Number of local-field Sturm polynomials built.
    pub sturm_sequence_length: usize,
    /// Exact bisection steps used to refine the retained root while signing
    /// local-field coefficients.
    pub retained_refinement_steps: usize,
    /// Weakest predicate certainty consumed by decisions visible to this
    /// package.
    pub certainty: Certainty,
    /// Compact diagnostic reason for a non-counted result.
    pub message: Option<&'static str>,
}

/// Count distinct roots of one exact bivariate fiber over a represented root.
///
/// `retained_parameter` selects the parameter represented by `retained_root`;
/// the opposite parameter is counted in `(fiber_lower, fiber_upper)`. All
/// polynomial and endpoint values must be exact rationals. Coefficients of the
/// specialized fiber are represented as fractions in the local field at the
/// selected root, reduced modulo its defining polynomial after every
/// operation. Sturm's theorem then counts distinct roots, so an isolated root
/// of even multiplicity is retained instead of being missed by an endpoint
/// sign-change test.
pub fn count_bivariate_fiber_roots_at_algebraic_parameter(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    retained_root: &AlgebraicRootRepresentation,
    fiber_lower: &Real,
    fiber_upper: &Real,
    policy: PredicatePolicy,
) -> AlgebraicFiberRootCountReport {
    if fiber_lower.exact_rational_ref().is_none()
        || fiber_upper.exact_rational_ref().is_none()
        || polynomial
            .coefficients
            .iter()
            .flatten()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return fiber_root_count_report(
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient,
            None,
            0,
            0,
            Certainty::Exact,
            Some("fiber root counting requires exact-rational coefficients and endpoints"),
        );
    }

    let mut field = match LocalAlgebraicField::new(retained_root, policy) {
        Ok(field) => field,
        Err(error) => {
            return fiber_root_count_error_report(error, 0, 0, Certainty::Exact);
        }
    };
    let outcome = count_fiber_roots(
        polynomial,
        retained_parameter,
        fiber_lower,
        fiber_upper,
        &mut field,
    );
    match outcome {
        Ok(LocalRootCountOutcome::Counted {
            count,
            sequence_length,
        }) => fiber_root_count_report(
            AlgebraicFiberRootCountStatus::Counted,
            Some(count),
            sequence_length,
            field.refinement_steps,
            field.certainty,
            None,
        ),
        Ok(LocalRootCountOutcome::IdenticallyZeroFiber) => fiber_root_count_report(
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber,
            None,
            0,
            field.refinement_steps,
            field.certainty,
            Some("every specialized fiber coefficient vanishes at the retained root"),
        ),
        Ok(LocalRootCountOutcome::EndpointRoot { sequence_length }) => fiber_root_count_report(
            AlgebraicFiberRootCountStatus::EndpointRoot,
            None,
            sequence_length,
            field.refinement_steps,
            field.certainty,
            Some("the specialized fiber has a root on an interval endpoint"),
        ),
        Err(error) => {
            fiber_root_count_error_report(error, 0, field.refinement_steps, field.certainty)
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LocalRootCountOutcome {
    Counted {
        count: usize,
        sequence_length: usize,
    },
    IdenticallyZeroFiber,
    EndpointRoot {
        sequence_length: usize,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LocalFieldError {
    InvalidEvidence,
    InvalidInterval,
    UnsupportedCoefficient,
    DivisionByZero,
    Undecided,
}

fn local_evaluation_sign(
    evaluation: &AlgebraicRootPolynomialEvaluationReport,
) -> Result<Option<Ordering>, LocalFieldError> {
    if let Some(sign) = evaluation.sign {
        return Ok(Some(sign));
    }
    match evaluation.status {
        AlgebraicRootPolynomialEvaluationStatus::IntervalContainsZero => Ok(None),
        AlgebraicRootPolynomialEvaluationStatus::InvalidEvidence => {
            Err(LocalFieldError::InvalidEvidence)
        }
        AlgebraicRootPolynomialEvaluationStatus::InvalidPolynomial => {
            Err(LocalFieldError::UnsupportedCoefficient)
        }
        AlgebraicRootPolynomialEvaluationStatus::Undecided
        | AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
        | AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive
        | AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedNegative => {
            Err(LocalFieldError::Undecided)
        }
    }
}

fn count_fiber_roots(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    fiber_lower: &Real,
    fiber_upper: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<LocalRootCountOutcome, LocalFieldError> {
    if field.compare(fiber_lower, fiber_upper)? != Ordering::Less {
        return Err(LocalFieldError::InvalidInterval);
    }

    let mut first = fiber_coefficient_polynomials(polynomial, retained_parameter)
        .into_iter()
        .map(|coefficient| LocalFieldElement::from_polynomial(coefficient, field))
        .collect::<Result<Vec<_>, _>>()?;
    trim_local_polynomial(&mut first, field)?;
    if first.len() == 1 && first[0].is_zero(field)? {
        return Ok(LocalRootCountOutcome::IdenticallyZeroFiber);
    }
    if first.len() == 1 {
        return Ok(LocalRootCountOutcome::Counted {
            count: 0,
            sequence_length: 1,
        });
    }

    let second = derivative_local_polynomial(&first, field)?;
    let mut sequence = vec![first, second];
    loop {
        let Some(last) = sequence.last() else {
            return Err(LocalFieldError::Undecided);
        };
        if last.len() == 1 {
            break;
        }
        let previous = sequence
            .get(sequence.len() - 2)
            .cloned()
            .ok_or(LocalFieldError::Undecided)?;
        let mut remainder = local_polynomial_remainder(previous, last, field)?;
        if remainder.len() == 1 && remainder[0].is_zero(field)? {
            break;
        }
        for coefficient in &mut remainder {
            coefficient.negate();
        }
        sequence.push(remainder);
    }

    let sequence_length = sequence.len();
    if local_polynomial_sign_at(&sequence[0], fiber_lower, field)? == Ordering::Equal
        || local_polynomial_sign_at(&sequence[0], fiber_upper, field)? == Ordering::Equal
    {
        return Ok(LocalRootCountOutcome::EndpointRoot { sequence_length });
    }
    let lower_variations = local_sign_variations(&sequence, fiber_lower, field)?;
    let upper_variations = local_sign_variations(&sequence, fiber_upper, field)?;
    let count = lower_variations
        .checked_sub(upper_variations)
        .ok_or(LocalFieldError::Undecided)?;
    Ok(LocalRootCountOutcome::Counted {
        count,
        sequence_length,
    })
}

fn fiber_coefficient_polynomials(
    polynomial: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
) -> Vec<Vec<Real>> {
    match retained_parameter {
        CurveResultantParameter::First => {
            let fiber_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            (0..fiber_count)
                .map(|fiber_power| {
                    polynomial
                        .coefficients
                        .iter()
                        .map(|row| row.get(fiber_power).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect()
        }
        CurveResultantParameter::Second => polynomial.coefficients.clone(),
    }
}

fn derivative_local_polynomial(
    polynomial: &[LocalFieldElement],
    field: &LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    let mut derivative = polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(power, coefficient)| coefficient.scale(&Real::from(power as u64), field))
        .collect::<Result<Vec<_>, _>>()?;
    if derivative.is_empty() {
        derivative.push(LocalFieldElement::zero());
    }
    Ok(derivative)
}

fn local_polynomial_remainder(
    mut dividend: Vec<LocalFieldElement>,
    divisor: &[LocalFieldElement],
    field: &mut LocalAlgebraicField,
) -> Result<Vec<LocalFieldElement>, LocalFieldError> {
    trim_local_polynomial(&mut dividend, field)?;
    if divisor.is_empty() || (divisor.len() == 1 && divisor[0].is_zero(field)?) {
        return Err(LocalFieldError::DivisionByZero);
    }
    while dividend.len() >= divisor.len() && !(dividend.len() == 1 && dividend[0].is_zero(field)?) {
        let degree_delta = dividend.len() - divisor.len();
        let scale = dividend.last().ok_or(LocalFieldError::Undecided)?.divide(
            divisor.last().ok_or(LocalFieldError::DivisionByZero)?,
            field,
        )?;
        for (power, divisor_coefficient) in divisor.iter().enumerate() {
            let product = scale.multiply(divisor_coefficient, field)?;
            dividend[degree_delta + power] =
                dividend[degree_delta + power].subtract(&product, field)?;
        }
        trim_local_polynomial(&mut dividend, field)?;
    }
    Ok(dividend)
}

fn trim_local_polynomial(
    polynomial: &mut Vec<LocalFieldElement>,
    field: &mut LocalAlgebraicField,
) -> Result<(), LocalFieldError> {
    while polynomial.len() > 1
        && polynomial
            .last()
            .ok_or(LocalFieldError::Undecided)?
            .is_zero(field)?
    {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(LocalFieldElement::zero());
    }
    Ok(())
}

fn local_polynomial_sign_at(
    polynomial: &[LocalFieldElement],
    parameter: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<Ordering, LocalFieldError> {
    let mut value = LocalFieldElement::zero();
    for coefficient in polynomial.iter().rev() {
        value = value.scale(parameter, field)?.add(coefficient, field)?;
    }
    value.sign(field)
}

fn local_sign_variations(
    sequence: &[Vec<LocalFieldElement>],
    parameter: &Real,
    field: &mut LocalAlgebraicField,
) -> Result<usize, LocalFieldError> {
    let mut previous = None;
    let mut variations = 0_usize;
    for polynomial in sequence {
        let sign = local_polynomial_sign_at(polynomial, parameter, field)?;
        if sign == Ordering::Equal {
            continue;
        }
        if previous.is_some_and(|previous| previous != sign) {
            variations += 1;
        }
        previous = Some(sign);
    }
    Ok(variations)
}

#[derive(Clone, Debug)]
struct LocalFieldElement {
    numerator: Vec<Real>,
    // `None` is the canonical, allocation-free representation of one.
    denominator: Option<Vec<Real>>,
}

impl LocalFieldElement {
    fn zero() -> Self {
        Self {
            numerator: vec![Real::zero()],
            denominator: None,
        }
    }

    fn from_polynomial(
        numerator: Vec<Real>,
        field: &LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        Ok(Self {
            numerator: field.reduce(numerator)?,
            denominator: None,
        })
    }

    fn add(&self, other: &Self, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        self.combine(other, false, field)
    }

    fn subtract(&self, other: &Self, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        self.combine(other, true, field)
    }

    fn combine(
        &self,
        other: &Self,
        subtract: bool,
        field: &LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        let (numerator, denominator) = match (&self.denominator, &other.denominator) {
            (None, None) => (
                field.add_polynomials(&self.numerator, &other.numerator, subtract)?,
                None,
            ),
            (Some(left_denominator), Some(right_denominator))
                if left_denominator == right_denominator =>
            {
                (
                    field.add_polynomials(&self.numerator, &other.numerator, subtract)?,
                    Some(left_denominator.clone()),
                )
            }
            (Some(left_denominator), None) => {
                let right = field.multiply_polynomials(&other.numerator, left_denominator)?;
                (
                    field.add_polynomials(&self.numerator, &right, subtract)?,
                    Some(left_denominator.clone()),
                )
            }
            (None, Some(right_denominator)) => {
                let left = field.multiply_polynomials(&self.numerator, right_denominator)?;
                (
                    field.add_polynomials(&left, &other.numerator, subtract)?,
                    Some(right_denominator.clone()),
                )
            }
            (Some(left_denominator), Some(right_denominator)) => {
                let left = field.multiply_polynomials(&self.numerator, right_denominator)?;
                let right = field.multiply_polynomials(&other.numerator, left_denominator)?;
                (
                    field.add_polynomials(&left, &right, subtract)?,
                    Some(field.multiply_polynomials(left_denominator, right_denominator)?),
                )
            }
        };
        Ok(Self {
            numerator,
            denominator,
        })
    }

    fn multiply(&self, other: &Self, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        let denominator = match (&self.denominator, &other.denominator) {
            (None, None) => None,
            (Some(denominator), None) | (None, Some(denominator)) => Some(denominator.clone()),
            (Some(left), Some(right)) => Some(field.multiply_polynomials(left, right)?),
        };
        Ok(Self {
            numerator: field.multiply_polynomials(&self.numerator, &other.numerator)?,
            denominator,
        })
    }

    fn divide(
        &self,
        other: &Self,
        field: &mut LocalAlgebraicField,
    ) -> Result<Self, LocalFieldError> {
        if other.is_zero(field)? {
            return Err(LocalFieldError::DivisionByZero);
        }
        let numerator = match &other.denominator {
            Some(denominator) => field.multiply_polynomials(&self.numerator, denominator)?,
            None => self.numerator.clone(),
        };
        let denominator = match &self.denominator {
            Some(denominator) => field.multiply_polynomials(denominator, &other.numerator)?,
            None => other.numerator.clone(),
        };
        Ok(Self {
            numerator,
            denominator: field.canonical_denominator(denominator),
        })
    }

    fn scale(&self, scale: &Real, field: &LocalAlgebraicField) -> Result<Self, LocalFieldError> {
        if scale.exact_rational_ref().is_none() {
            return Err(LocalFieldError::UnsupportedCoefficient);
        }
        Ok(Self {
            numerator: field.reduce(
                self.numerator
                    .iter()
                    .map(|coefficient| coefficient * scale)
                    .collect(),
            )?,
            denominator: self.denominator.clone(),
        })
    }

    fn negate(&mut self) {
        for coefficient in &mut self.numerator {
            *coefficient = -coefficient.clone();
        }
    }

    fn is_zero(&self, field: &mut LocalAlgebraicField) -> Result<bool, LocalFieldError> {
        field.is_zero_polynomial(&self.numerator)
    }

    fn sign(&self, field: &mut LocalAlgebraicField) -> Result<Ordering, LocalFieldError> {
        let numerator = field.sign_polynomial(&self.numerator)?;
        if numerator == Ordering::Equal {
            return Ok(Ordering::Equal);
        }
        let denominator = match &self.denominator {
            Some(denominator) => {
                let sign = field.sign_polynomial(denominator)?;
                if sign == Ordering::Equal {
                    return Err(LocalFieldError::DivisionByZero);
                }
                sign
            }
            None => Ordering::Greater,
        };
        Ok(if numerator == denominator {
            Ordering::Greater
        } else {
            Ordering::Less
        })
    }
}

struct LocalAlgebraicField {
    root: AlgebraicRootRepresentation,
    modulus: Vec<Real>,
    signed_polynomials: Vec<(Vec<Real>, Ordering)>,
    policy: PredicatePolicy,
    certainty: Certainty,
    refinement_steps: usize,
}

impl LocalAlgebraicField {
    fn new(
        root: &AlgebraicRootRepresentation,
        policy: PredicatePolicy,
    ) -> Result<Self, LocalFieldError> {
        if !root.is_valid()
            || root.interval.distinct_root_count != 1
            || root.polynomial_coefficients.len() <= 1
        {
            return Err(LocalFieldError::InvalidEvidence);
        }
        if root
            .polynomial_coefficients
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
        {
            return Err(LocalFieldError::UnsupportedCoefficient);
        }
        Ok(Self {
            root: root.clone(),
            modulus: root.polynomial_coefficients.clone(),
            signed_polynomials: Vec::new(),
            policy,
            certainty: Certainty::Exact,
            refinement_steps: 0,
        })
    }

    fn observe_certainty(&mut self, certainty: Certainty) {
        self.certainty = match (self.certainty, certainty) {
            (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
            (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
            (Certainty::Exact, Certainty::Exact) => Certainty::Exact,
        };
    }

    fn consume<T>(&mut self, outcome: PredicateOutcome<T>) -> Result<T, LocalFieldError> {
        match outcome {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                self.observe_certainty(certainty);
                Ok(value)
            }
            PredicateOutcome::Unknown { .. } => Err(LocalFieldError::Undecided),
        }
    }

    fn compare(&mut self, left: &Real, right: &Real) -> Result<Ordering, LocalFieldError> {
        self.consume(compare_reals(left, right, self.policy))
    }

    fn reduce(&self, mut polynomial: Vec<Real>) -> Result<Vec<Real>, LocalFieldError> {
        if polynomial
            .iter()
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
        {
            return Err(LocalFieldError::UnsupportedCoefficient);
        }
        while polynomial.len() > 1
            && polynomial
                .last()
                .is_some_and(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            polynomial.pop();
        }
        if polynomial.is_empty() {
            polynomial.push(Real::zero());
        }
        if polynomial.len() < self.modulus.len() {
            return Ok(polynomial);
        }
        polynomial_div_rem(polynomial, &self.modulus, self.policy)
            .map(|(_, remainder)| remainder)
            .ok_or(LocalFieldError::Undecided)
    }

    fn canonical_denominator(&self, denominator: Vec<Real>) -> Option<Vec<Real>> {
        if denominator.len() == 1 && denominator[0] == Real::one() {
            None
        } else {
            Some(denominator)
        }
    }

    fn add_polynomials(
        &self,
        left: &[Real],
        right: &[Real],
        subtract: bool,
    ) -> Result<Vec<Real>, LocalFieldError> {
        let mut result = vec![Real::zero(); left.len().max(right.len())];
        for (power, coefficient) in left.iter().enumerate() {
            result[power] = result[power].clone() + coefficient;
        }
        for (power, coefficient) in right.iter().enumerate() {
            result[power] = if subtract {
                result[power].clone() - coefficient
            } else {
                result[power].clone() + coefficient
            };
        }
        self.reduce(result)
    }

    fn multiply_polynomials(
        &self,
        left: &[Real],
        right: &[Real],
    ) -> Result<Vec<Real>, LocalFieldError> {
        let mut result = vec![Real::zero(); left.len() + right.len() - 1];
        for (left_power, left_coefficient) in left.iter().enumerate() {
            for (right_power, right_coefficient) in right.iter().enumerate() {
                result[left_power + right_power] =
                    result[left_power + right_power].clone() + left_coefficient * right_coefficient;
            }
        }
        self.reduce(result)
    }

    fn sign_polynomial(&mut self, polynomial: &[Real]) -> Result<Ordering, LocalFieldError> {
        let polynomial = self.reduce(polynomial.to_vec())?;
        if polynomial
            .iter()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            return Ok(Ordering::Equal);
        }
        if let Some((_, sign)) = self
            .signed_polynomials
            .iter()
            .find(|(signed, _)| signed == &polynomial)
        {
            return Ok(*sign);
        }
        let sign = self.sign_reduced_polynomial(&polynomial)?;
        self.signed_polynomials.push((polynomial, sign));
        Ok(sign)
    }

    fn is_zero_polynomial(&mut self, polynomial: &[Real]) -> Result<bool, LocalFieldError> {
        let polynomial = self.reduce(polynomial.to_vec())?;
        if polynomial
            .iter()
            .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
        {
            return Ok(true);
        }
        if let Some((_, sign)) = self
            .signed_polynomials
            .iter()
            .find(|(signed, _)| signed == &polynomial)
        {
            return Ok(*sign == Ordering::Equal);
        }
        let evaluation =
            evaluate_polynomial_at_algebraic_root(&self.root, &polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            self.signed_polynomials.push((polynomial, sign));
            return Ok(sign == Ordering::Equal);
        }
        let is_zero = polynomials_share_one_root_in_interval(
            &self.modulus,
            &polynomial,
            &self.root.interval.lower,
            &self.root.interval.upper,
            self.policy,
        )
        .ok_or(LocalFieldError::Undecided)?;
        if is_zero {
            self.signed_polynomials.push((polynomial, Ordering::Equal));
        }
        Ok(is_zero)
    }

    fn sign_reduced_polynomial(
        &mut self,
        polynomial: &[Real],
    ) -> Result<Ordering, LocalFieldError> {
        if self.root.exact_rational_witness().is_some() {
            return evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy)
                .sign
                .ok_or(LocalFieldError::Undecided);
        }

        // Most local-field coefficients separate from zero over the existing
        // isolating interval.  Prove that cheap case before constructing a
        // polynomial GCD; a zero coefficient necessarily leaves zero inside
        // the conservative interval enclosure and still reaches the exact
        // common-root test below.
        let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            return Ok(sign);
        }
        // Ordered sign queries usually need only a tighter isolating interval.
        // Refine once before paying for a common-root GCD.  Exact zero tests
        // use `is_zero_polynomial`, which takes the identity path directly.
        self.refine_root()?;
        let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
        if let Some(sign) = local_evaluation_sign(&evaluation)? {
            return Ok(sign);
        }

        match polynomials_share_one_root_in_interval(
            &self.modulus,
            polynomial,
            &self.root.interval.lower,
            &self.root.interval.upper,
            self.policy,
        ) {
            Some(true) => return Ok(Ordering::Equal),
            Some(false) => {}
            None => return Err(LocalFieldError::Undecided),
        }

        loop {
            let evaluation =
                evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
            if let Some(sign) = local_evaluation_sign(&evaluation)? {
                return Ok(sign);
            }
            self.refine_root()?;
        }
    }

    fn refine_root(&mut self) -> Result<(), LocalFieldError> {
        let refinement = refine_isolated_univariate_polynomial_interval(
            &self.modulus,
            &self.root.interval,
            RootIsolationConfig {
                policy: self.policy,
                max_interval_width: None,
                max_refinement_steps: 4,
            },
        );
        let Some(refined_interval) = refinement.refined_interval else {
            return Err(match refinement.status {
                IsolatedRootRefinementStatus::InvalidPolynomial
                | IsolatedRootRefinementStatus::InvalidInterval
                | IsolatedRootRefinementStatus::NonUnitIsolation => {
                    LocalFieldError::InvalidEvidence
                }
                IsolatedRootRefinementStatus::Refined
                | IsolatedRootRefinementStatus::ExactRoot
                | IsolatedRootRefinementStatus::Undecided => LocalFieldError::Undecided,
            });
        };
        if refined_interval == self.root.interval {
            return Err(LocalFieldError::Undecided);
        }
        self.refinement_steps += refinement.refinement_steps;
        self.root.interval = refined_interval;
        self.root.kind = if self.root.interval.exact_root.is_some() {
            AlgebraicRootKind::ExactRationalWitness
        } else {
            AlgebraicRootKind::IsolatingInterval
        };
        self.root.validation = validate_algebraic_root_representation(&self.root, self.policy);
        if !self.root.is_valid() {
            return Err(LocalFieldError::InvalidEvidence);
        }
        Ok(())
    }
}

fn fiber_root_count_error_report(
    error: LocalFieldError,
    sturm_sequence_length: usize,
    retained_refinement_steps: usize,
    certainty: Certainty,
) -> AlgebraicFiberRootCountReport {
    let (status, message) = match error {
        LocalFieldError::InvalidEvidence => (
            AlgebraicFiberRootCountStatus::InvalidEvidence,
            "the retained algebraic-root evidence is invalid",
        ),
        LocalFieldError::InvalidInterval => (
            AlgebraicFiberRootCountStatus::InvalidInterval,
            "the fiber interval must be strictly ordered",
        ),
        LocalFieldError::UnsupportedCoefficient => (
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient,
            "the local-field Sturm package requires exact-rational coefficients",
        ),
        LocalFieldError::DivisionByZero => (
            AlgebraicFiberRootCountStatus::Undecided,
            "a local-field Sturm divisor vanished at the retained root",
        ),
        LocalFieldError::Undecided => (
            AlgebraicFiberRootCountStatus::Undecided,
            "exact local-field arithmetic or coefficient signing did not complete",
        ),
    };
    fiber_root_count_report(
        status,
        None,
        sturm_sequence_length,
        retained_refinement_steps,
        certainty,
        Some(message),
    )
}

fn fiber_root_count_report(
    status: AlgebraicFiberRootCountStatus,
    distinct_root_count: Option<usize>,
    sturm_sequence_length: usize,
    retained_refinement_steps: usize,
    certainty: Certainty,
    message: Option<&'static str>,
) -> AlgebraicFiberRootCountReport {
    AlgebraicFiberRootCountReport {
        status,
        distinct_root_count,
        sturm_sequence_length,
        retained_refinement_steps,
        certainty,
        message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algebraic::{
        AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
        validate_algebraic_root_representation,
    };
    use crate::root_isolation::IsolatedRootInterval;
    use crate::symbolic::SymbolId;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn rational(numerator: i64, denominator: i64) -> Real {
        (real(numerator) / real(denominator)).unwrap()
    }

    fn represented_root(
        polynomial_coefficients: Vec<Real>,
        lower: Real,
        upper: Real,
        policy: PredicatePolicy,
    ) -> AlgebraicRootRepresentation {
        let mut root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients,
            interval: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        root.validation = validate_algebraic_root_representation(&root, policy);
        assert!(root.is_valid());
        root
    }

    fn represented_rational_root(
        value: Real,
        policy: PredicatePolicy,
    ) -> AlgebraicRootRepresentation {
        let mut root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-value.clone(), Real::one()],
            interval: IsolatedRootInterval {
                lower: value.clone(),
                upper: value.clone(),
                exact_root: Some(value),
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::ExactRationalWitness,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };
        root.validation = validate_algebraic_root_representation(&root, policy);
        assert!(root.is_valid());
        root
    }

    #[test]
    fn local_field_sturm_counts_even_multiplicity_in_both_orientations() {
        // alpha = cbrt(1/2), beta = alpha^2 = cbrt(1/4).
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![],
            vec![real(0), real(-2)],
            vec![],
            vec![real(1)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(-1), real(0), real(0), real(2)],
                rational(3, 4),
                rational(4, 5),
                policy,
            );
            let beta = represented_root(
                vec![real(-1), real(0), real(0), real(4)],
                rational(3, 5),
                rational(2, 3),
                policy,
            );

            let retained_first = count_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                &beta.interval.lower,
                &beta.interval.upper,
                policy,
            );
            assert_eq!(
                retained_first.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(retained_first.distinct_root_count, Some(1));
            assert_eq!(retained_first.certainty, Certainty::Exact);

            let retained_second = count_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::Second,
                &beta,
                &alpha.interval.lower,
                &alpha.interval.upper,
                policy,
            );
            assert_eq!(
                retained_second.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(retained_second.distinct_root_count, Some(1));
            assert_eq!(retained_second.certainty, Certainty::Exact);

            let empty_interval = count_bivariate_fiber_roots_at_algebraic_parameter(
                &relation,
                CurveResultantParameter::First,
                &alpha,
                &real(0),
                &rational(1, 2),
                policy,
            );
            assert_eq!(
                empty_interval.status,
                AlgebraicFiberRootCountStatus::Counted
            );
            assert_eq!(empty_interval.distinct_root_count, Some(0));
        }
    }

    #[test]
    fn local_field_sturm_reports_an_identically_zero_specialization() {
        let policy = PredicatePolicy::STRICT;
        let alpha = represented_root(
            vec![real(-1), real(0), real(2)],
            rational(2, 3),
            rational(3, 4),
            policy,
        );
        let defining = BivariatePolynomial::new(vec![vec![real(-1)], vec![real(0)], vec![real(2)]]);
        let report = count_bivariate_fiber_roots_at_algebraic_parameter(
            &defining,
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            report.status,
            AlgebraicFiberRootCountStatus::IdenticallyZeroFiber
        );
        assert_eq!(report.distinct_root_count, None);
    }

    #[test]
    fn local_field_sturm_handles_a_reducible_defining_polynomial() {
        // (a^2 - 2)(a - 3) defines sqrt(2) locally on (7/5, 3/2).
        // The quotient by the whole cubic is not a field, so this exercises
        // denominator signing and exact common-root zero detection at the
        // selected factor rather than assuming global irreducibility.
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![real(0), real(-2)],
            vec![real(1)],
        ]);
        let repeated_two_root_fiber = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(0), real(0), real(1)],
            vec![real(0), real(0), real(-2)],
            vec![real(1)],
        ]);
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let alpha = represented_root(
                vec![real(6), real(-2), real(-3), real(1)],
                rational(7, 5),
                rational(3, 2),
                policy,
            );
            for retained_parameter in [
                CurveResultantParameter::First,
                CurveResultantParameter::Second,
            ] {
                let report = count_bivariate_fiber_roots_at_algebraic_parameter(
                    &relation,
                    retained_parameter,
                    &alpha,
                    &real(1),
                    &real(2),
                    policy,
                );
                assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                assert_eq!(report.distinct_root_count, Some(1));
                assert_eq!(report.certainty, Certainty::Exact);
            }

            let two_roots = count_bivariate_fiber_roots_at_algebraic_parameter(
                &repeated_two_root_fiber,
                CurveResultantParameter::First,
                &alpha,
                &real(-2),
                &real(2),
                policy,
            );
            assert_eq!(two_roots.status, AlgebraicFiberRootCountStatus::Counted);
            assert_eq!(two_roots.distinct_root_count, Some(2));
            assert_eq!(two_roots.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn local_field_sturm_reports_rational_endpoint_roots_and_input_boundaries() {
        let policy = PredicatePolicy::STRICT;
        let half = rational(1, 2);
        let alpha = represented_rational_root(half.clone(), policy);
        let relation = BivariatePolynomial::new(vec![
            vec![real(0), real(0), real(1)],
            vec![real(0), real(-2)],
            vec![real(1)],
        ]);

        let counted = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(counted.status, AlgebraicFiberRootCountStatus::Counted);
        assert_eq!(counted.distinct_root_count, Some(1));

        let endpoint = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &half,
            &real(1),
            policy,
        );
        assert_eq!(endpoint.status, AlgebraicFiberRootCountStatus::EndpointRoot);

        let reversed = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &alpha,
            &real(1),
            &real(0),
            policy,
        );
        assert_eq!(
            reversed.status,
            AlgebraicFiberRootCountStatus::InvalidInterval
        );

        let unsupported = count_bivariate_fiber_roots_at_algebraic_parameter(
            &BivariatePolynomial::new(vec![vec![Real::pi(), real(1)]]),
            CurveResultantParameter::First,
            &alpha,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            unsupported.status,
            AlgebraicFiberRootCountStatus::UnsupportedCoefficient
        );

        let mut nonunit = alpha;
        nonunit.interval.distinct_root_count = 2;
        let invalid = count_bivariate_fiber_roots_at_algebraic_parameter(
            &relation,
            CurveResultantParameter::First,
            &nonunit,
            &real(0),
            &real(1),
            policy,
        );
        assert_eq!(
            invalid.status,
            AlgebraicFiberRootCountStatus::InvalidEvidence
        );
    }
}
