//! Checked square-free factors for a previously repeated finite-fiber source.
use super::*;
use crate::bivariate_arithmetic::{
    bivariate_multiply, bivariate_outer_product, bivariate_storage_bidegree_sum,
};
use crate::bivariate_components::rational_parameter_component_support;
use crate::curve_resultant::{
    BivariatePolynomialAxisFactorStatus, BivariatePolynomialComponentStatus,
    extract_bivariate_polynomial_system_axis_factors,
    parameter_component_bivariate_polynomial_system,
};
use std::sync::Arc;

/// A product of distinct irreducible factors with one defining exponent.
///
/// This exponent is not a local contact or intersection multiplicity. Different
/// branches can belong to the same factor, and singular points need their own
/// incidence analysis. Factors need not be irreducible individually.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateMultiplicityFactor {
    equation: Arc<BivariatePolynomial>,
    exponent: usize,
}

impl BivariateMultiplicityFactor {
    /// Exact supporting polynomial for this multiplicity group.
    pub fn equation(&self) -> &BivariatePolynomial {
        &self.equation
    }

    /// Exponent in the original defining polynomial.
    pub const fn exponent(&self) -> usize {
        self.exponent
    }
}

/// Exact factorization retained when generic repeated fibers need reduction.
///
/// The original polynomial equals `scale * product(factor^exponent)`. The
/// support has one copy of each factor. This checked live value is reconstructed
/// from the original polynomial on replay; serialized flags are not proof.
#[derive(Clone, Debug, PartialEq)]
pub struct BivariateRepeatedFactorization {
    support: Arc<BivariatePolynomial>,
    scale: Real,
    factors: Vec<BivariateMultiplicityFactor>,
}

impl BivariateRepeatedFactorization {
    /// Square-free support of the original zero set.
    pub fn support(&self) -> &BivariatePolynomial {
        &self.support
    }

    /// Nonzero rational scalar in the exact reconstruction identity.
    pub const fn scale(&self) -> &Real {
        &self.scale
    }

    /// Coprime square-free factors, grouped by increasing positive exponent.
    pub fn factors(&self) -> &[BivariateMultiplicityFactor] {
        &self.factors
    }
}

type Result<T> = std::result::Result<T, BivariateTopologyError>;

fn canonical(polynomial: &BivariatePolynomial) -> Result<BivariatePolynomial> {
    let mut rows = polynomial
        .coefficients
        .iter()
        .map(|row| {
            let mut row = row
                .iter()
                .map(|value| {
                    value
                        .exact_rational_normal_form()
                        .map(Real::new)
                        .ok_or(BivariateTopologyError::Undecided)
                })
                .collect::<Result<Vec<_>>>()?;
            while row.last().is_some_and(Real::definitely_zero) {
                row.pop();
            }
            Ok(row)
        })
        .collect::<Result<Vec<_>>>()?;
    while rows.last().is_some_and(Vec::is_empty) {
        rows.pop();
    }
    Ok(BivariatePolynomial::new(rows))
}

fn one() -> BivariatePolynomial {
    BivariatePolynomial::new(vec![vec![Real::one()]])
}

fn constant(polynomial: &BivariatePolynomial) -> bool {
    polynomial.coefficients.len() <= 1
        && polynomial
            .coefficients
            .first()
            .is_none_or(|row| row.len() <= 1)
}

fn monic(polynomial: &BivariatePolynomial) -> Result<BivariatePolynomial> {
    let mut polynomial = canonical(polynomial)?;
    let Some(leading) = polynomial
        .coefficients
        .last()
        .and_then(|row| row.last())
        .cloned()
    else {
        return Ok(polynomial);
    };
    let inverse = (Real::one() / leading).map_err(|_| BivariateTopologyError::Undecided)?;
    for coefficient in polynomial.coefficients.iter_mut().flatten() {
        *coefficient *= &inverse;
    }
    canonical(&polynomial)
}

fn quotient(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> Result<BivariatePolynomial> {
    crate::divide_bivariate_polynomial_exact(first, second)
        .ok_or(BivariateTopologyError::Undecided)
        .and_then(|result| canonical(&result))
}

fn product(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> Result<BivariatePolynomial> {
    canonical(&bivariate_multiply(first, second))
}

fn gcd(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    config: BivariateTopologyConfig,
) -> Result<BivariatePolynomial> {
    work_budget::consume(WorkKind::AlgebraicPromotion, 1)
        .map_err(|_| BivariateTopologyError::BudgetExhausted)?;
    let mut pair = [monic(first)?, monic(second)?];
    if pair[0].coefficients.is_empty() {
        return Ok(pair[1].clone());
    }
    if pair[1].coefficients.is_empty() {
        return Ok(pair[0].clone());
    }
    if constant(&pair[0]) || constant(&pair[1]) {
        return Ok(one());
    }
    for (dividend, divisor) in [(0, 1), (1, 0)] {
        if crate::divide_bivariate_polynomial_exact(&pair[dividend], &pair[divisor]).is_some() {
            return Ok(pair[divisor].clone());
        }
    }
    let mut common = one();
    let bounded = CurveIntersectionResultantConfig {
        max_resultant_degree: config.max_projection_degree,
        ..Default::default()
    };
    loop {
        let axes = extract_bivariate_polynomial_system_axis_factors(&pair[0], &pair[1]);
        match axes.status {
            BivariatePolynomialAxisFactorStatus::Primitive => {}
            BivariatePolynomialAxisFactorStatus::Reduced => {
                let factor = bivariate_outer_product(
                    &axes.first_parameter_factor,
                    &axes.second_parameter_factor,
                );
                // Recheck the reported factor by exact division. Its entire
                // zero set stays in the accumulated gcd, including axis fibers.
                pair = [quotient(&pair[0], &factor)?, quotient(&pair[1], &factor)?];
                common = product(&common, &factor)?;
            }
            BivariatePolynomialAxisFactorStatus::BudgetExhausted => {
                return Err(BivariateTopologyError::BudgetExhausted);
            }
            _ => return Err(BivariateTopologyError::Undecided),
        }
        if pair
            .iter()
            .any(|p| p.coefficients.iter().all(|row| row.len() <= 1))
        {
            // Any common factor of a first-axis polynomial was covered by the
            // exact axis-content gcd. No generic lifted factor can remain.
            return monic(&common);
        }
        let resultant = resultant_bivariate_polynomial_system(&pair[0], &pair[1], FIRST, bounded);
        match resultant.status {
            CurveIntersectionResultantStatus::Constructed => {}
            CurveIntersectionResultantStatus::BudgetExhausted => {
                return Err(BivariateTopologyError::BudgetExhausted);
            }
            CurveIntersectionResultantStatus::DegreeBoundExceeded => {
                return Err(BivariateTopologyError::DegreeLimit);
            }
            _ => return Err(BivariateTopologyError::Undecided),
        }
        if !is_zero(&resultant.resultant_coefficients) {
            // A nonzero resultant proves generic coprimality. Common axis
            // factors were already retained above, so the full gcd is known.
            return monic(&common);
        }
        let report =
            parameter_component_bivariate_polynomial_system(&pair[0], &pair[1], FIRST, bounded);
        let component = match report.status {
            BivariatePolynomialComponentStatus::Rational => rational_parameter_component_support(
                FIRST,
                &report.numerator_coefficients,
                &report.denominator_coefficients,
            ),
            BivariatePolynomialComponentStatus::Implicit => report.implicit_component,
            BivariatePolynomialComponentStatus::BudgetExhausted => {
                return Err(BivariateTopologyError::BudgetExhausted);
            }
            BivariatePolynomialComponentStatus::DegreeBoundExceeded => {
                return Err(BivariateTopologyError::DegreeLimit);
            }
            _ => return Err(BivariateTopologyError::Undecided),
        }
        .ok_or(BivariateTopologyError::Undecided)?;
        let next = [
            quotient(&pair[0], &component)?,
            quotient(&pair[1], &component)?,
        ];
        if next
            .iter()
            .map(bivariate_storage_bidegree_sum)
            .sum::<usize>()
            >= pair
                .iter()
                .map(bivariate_storage_bidegree_sum)
                .sum::<usize>()
        {
            return Err(BivariateTopologyError::Undecided);
        }
        common = product(&common, &component)?;
        pair = next;
    }
}

pub(super) fn factor(
    original: &BivariatePolynomial,
    config: BivariateTopologyConfig,
) -> Result<BivariateRepeatedFactorization> {
    let canonical_original = canonical(original)?;
    let scale = canonical_original
        .coefficients
        .last()
        .and_then(|row| row.last())
        .cloned()
        .ok_or(BivariateTopologyError::ZeroPolynomial)?;
    let normalized = monic(original)?;
    let dx = bivariate_parameter_derivative(&normalized, FIRST);
    let dy = bivariate_parameter_derivative(&normalized, CurveResultantParameter::Second);
    let mut repeated = gcd(&gcd(&normalized, &dy, config)?, &dx, config)?;
    let mut remaining = quotient(&normalized, &repeated)?;
    let mut exponent = 1_usize;
    let mut factors = Vec::new();
    while !constant(&remaining) {
        let common = gcd(&remaining, &repeated, config)?;
        let factor = quotient(&remaining, &common)?;
        if !constant(&factor) {
            factors.push(BivariateMultiplicityFactor {
                equation: Arc::new(factor),
                exponent,
            });
        }
        let next_repeated = quotient(&repeated, &common)?;
        if bivariate_storage_bidegree_sum(&common) + bivariate_storage_bidegree_sum(&next_repeated)
            >= bivariate_storage_bidegree_sum(&remaining)
                + bivariate_storage_bidegree_sum(&repeated)
        {
            return Err(BivariateTopologyError::Undecided);
        }
        remaining = common;
        repeated = next_repeated;
        exponent = exponent
            .checked_add(1)
            .ok_or(BivariateTopologyError::DegreeLimit)?;
    }
    if !constant(&repeated) || factors.is_empty() {
        return Err(BivariateTopologyError::Undecided);
    }
    let mut rebuilt = BivariatePolynomial::new(vec![vec![scale.clone()]]);
    let mut support = one();
    for factor in &factors {
        support = product(&support, factor.equation())?;
        for _ in 0..factor.exponent {
            rebuilt = product(&rebuilt, factor.equation())?;
        }
    }
    if canonical(&rebuilt)? != canonical_original
        || bivariate_storage_bidegree_sum(&support)
            >= bivariate_storage_bidegree_sum(&canonical_original)
    {
        return Err(BivariateTopologyError::Undecided);
    }
    let support = if factors.len() == 1 {
        Arc::clone(&factors[0].equation)
    } else {
        Arc::new(support)
    };
    Ok(BivariateRepeatedFactorization {
        support,
        scale,
        factors,
    })
}

/// Bind each selected root to the retained multiplicity factors. Every factor
/// divides the reduced support, so a root in one of its singleton isolators is
/// the same selected root, including coincident factors at an event.
pub(super) fn root_factors(
    evidence: &BivariateRepeatedFactorization,
    parameter: &AlgebraicRootRepresentation,
    roots: &[IsolatedRootInterval],
) -> Result<Vec<Vec<usize>>> {
    if evidence.factors.len() == 1 {
        return Ok(roots.iter().map(|_| vec![0]).collect());
    }
    roots
        .iter()
        .map(|root| {
            let mut indices = Vec::new();
            for (index, factor) in evidence.factors.iter().enumerate() {
                if let Some(exact) = &root.exact_root {
                    if fiber_sign(factor.equation(), parameter, exact)? == Ordering::Equal {
                        indices.push(index);
                    }
                    continue;
                }
                let report = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
                    factor.equation(),
                    FIRST,
                    parameter,
                    &root.lower,
                    &root.upper,
                    STRICT,
                );
                if report.status != AlgebraicFiberRootCountStatus::Counted
                    || report.certainty != Certainty::Exact
                {
                    return Err(BivariateTopologyError::Undecided);
                }
                match report.distinct_root_count {
                    Some(1) => indices.push(index),
                    Some(0) => {}
                    _ => return Err(BivariateTopologyError::Undecided),
                }
            }
            if indices.is_empty() {
                return Err(BivariateTopologyError::Undecided);
            }
            Ok(indices)
        })
        .collect()
}

pub(super) fn assign(
    topology: &mut BivariateCurveTopology,
    evidence: &BivariateRepeatedFactorization,
) -> Result<()> {
    for event in &mut topology.events {
        event.factor_indices = Some(root_factors(evidence, &event.parameter, &event.roots)?);
    }
    for slab in &mut topology.slabs {
        let parameter = AlgebraicRootRepresentation::from_exact_value(&slab.sample);
        let factors = root_factors(evidence, &parameter, &slab.roots)?;
        for (arc, factors) in slab.arcs.iter_mut().zip(factors) {
            // Distinct square-free factors cannot meet inside a projection
            // slab; their intersections are events of the reduced support.
            let [index] = factors.as_slice() else {
                return Err(BivariateTopologyError::Undecided);
            };
            arc.factor_index = Some(*index);
        }
    }
    Ok(())
}
