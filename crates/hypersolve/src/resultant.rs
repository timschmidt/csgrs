//! Exact univariate resultant reports.
//!
//! Resultants are the elimination step that lets a curve or solver pipeline
//! remove one variable before root isolation. This module starts with the
//! classical Sylvester-matrix determinant: the determinant is computed by
//! [`crate::determinant_bareiss`], so coefficient arithmetic remains exact and
//! pivot decisions remain certified. It also exposes a fraction-free
//! pseudo-remainder chain for the subresultant scheduling boundary. Both
//! constructions remain exact and report uncertain pivot decisions.

use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
#[cfg(test)]
use num::Integer;
use num::{BigInt, One, Zero};

use crate::bareiss::{BareissDeterminantReport, BareissError, determinant_bareiss};

/// Failure mode for exact univariate resultant construction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum UnivariateResultantError {
    /// One of the input coefficient slices was empty.
    EmptyPolynomial,
    /// A coefficient needed for degree trimming could not be certified zero or
    /// nonzero within the requested bound.
    UndecidedCoefficient {
        /// Input side whose coefficient was undecided.
        side: ResultantInputSide,
        /// Coefficient index in ascending power order.
        index: usize,
    },
    /// The Sylvester determinant could not be computed exactly.
    Determinant(BareissError),
}

/// Failure mode for exact univariate pseudo-remainder chain construction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum UnivariateSubresultantChainError {
    /// One of the input coefficient slices was empty.
    EmptyPolynomial,
    /// A coefficient needed for degree trimming could not be certified zero or
    /// nonzero within the requested bound.
    UndecidedCoefficient {
        /// Input side whose coefficient was undecided.
        side: ResultantInputSide,
        /// Coefficient index in ascending power order.
        index: usize,
    },
}

/// Identifies the input polynomial that produced a resultant error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResultantInputSide {
    /// Left-hand polynomial.
    Left,
    /// Right-hand polynomial.
    Right,
}

/// Exact Sylvester-resultant report for two univariate polynomials.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateResultantReport {
    /// Trimmed left polynomial coefficients in ascending power order.
    pub left_coefficients: Vec<Real>,
    /// Trimmed right polynomial coefficients in ascending power order.
    pub right_coefficients: Vec<Real>,
    /// Degree of the left polynomial after certified trimming.
    pub left_degree: usize,
    /// Degree of the right polynomial after certified trimming.
    pub right_degree: usize,
    /// Size of the square Sylvester matrix.
    pub sylvester_dimension: usize,
    /// Exact resultant value.
    pub resultant: Real,
    /// Determinant report for nonconstant/nonconstant inputs.
    pub determinant: Option<BareissDeterminantReport>,
}

/// One step in the exact pseudo-remainder chain.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateSubresultantChainStep {
    /// Dividend degree entering this step.
    pub dividend_degree: usize,
    /// Divisor degree entering this step.
    pub divisor_degree: usize,
    /// Leading coefficient of the divisor used by pseudo-division.
    pub divisor_leading_coefficient: Real,
    /// Signed pseudo-remainder coefficients in ascending power order.
    pub remainder: Vec<Real>,
    /// Degree of the signed pseudo-remainder.
    pub remainder_degree: usize,
    /// True when the chain terminated with a zero pseudo-remainder.
    pub zero_remainder: bool,
}

/// Exact pseudo-remainder chain report for two univariate polynomials.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateSubresultantChainReport {
    /// Trimmed left polynomial coefficients in ascending power order.
    pub left_coefficients: Vec<Real>,
    /// Trimmed right polynomial coefficients in ascending power order.
    pub right_coefficients: Vec<Real>,
    /// Fraction-free signed pseudo-remainder steps.
    pub steps: Vec<UnivariateSubresultantChainStep>,
    /// Last nonzero polynomial in the chain, in ascending power order.
    pub last_nonzero: Vec<Real>,
    /// Degree of the last nonzero polynomial.
    pub last_nonzero_degree: usize,
    /// True when the last nonzero polynomial is nonconstant, proving a common
    /// factor over exact arithmetic.
    pub has_nonconstant_common_factor: bool,
}

/// Input pair for batch resultant scheduling.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateResultantPairInput {
    /// Caller-supplied pair id preserved in reports.
    pub pair_index: usize,
    /// Left polynomial coefficients in ascending power order.
    pub left_coefficients: Vec<Real>,
    /// Right polynomial coefficients in ascending power order.
    pub right_coefficients: Vec<Real>,
}

/// Classification for one resultant-scheduled polynomial pair.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum UnivariateResultantPairStatus {
    /// The resultant was certified nonzero, so the two univariate polynomials
    /// are coprime over exact arithmetic.
    CertifiedCoprime,
    /// The resultant was zero and the pseudo-remainder chain ended with a
    /// nonconstant common factor.
    CertifiedCommonFactor,
    /// The resultant was zero but the chain did not expose a nonconstant
    /// common factor under this narrow report surface.
    ResultantZeroWithoutCommonFactorReport,
    /// Resultant construction failed.
    ResultantError,
    /// Subresultant-chain construction failed after a zero resultant.
    SubresultantChainError,
    /// Resultant sign could not be certified.
    UndecidedResultantSign,
}

/// Batch resultant scheduling report for one polynomial pair.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateResultantPairReport {
    /// Caller-supplied pair id.
    pub pair_index: usize,
    /// Pair classification.
    pub status: UnivariateResultantPairStatus,
    /// Resultant report when construction succeeded.
    pub resultant: Option<UnivariateResultantReport>,
    /// Subresultant-chain report when a zero resultant triggered it.
    pub subresultant_chain: Option<UnivariateSubresultantChainReport>,
    /// Resultant construction error, if any.
    pub resultant_error: Option<UnivariateResultantError>,
    /// Subresultant construction error, if any.
    pub subresultant_error: Option<UnivariateSubresultantChainError>,
}

/// Batch resultant scheduling report.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateResultantScheduleReport {
    /// One report per input pair.
    pub pairs: Vec<UnivariateResultantPairReport>,
    /// Number of pairs certified coprime by a nonzero resultant.
    pub certified_coprime_pairs: usize,
    /// Number of pairs certified to have a nonconstant common factor.
    pub certified_common_factor_pairs: usize,
    /// Number of pairs that ended in explicit error or uncertainty.
    pub unresolved_pairs: usize,
}

impl UnivariateResultantScheduleReport {
    /// Returns true when every pair has a certified resultant classification.
    pub fn all_classified(&self) -> bool {
        self.unresolved_pairs == 0
            && self.pairs.len() == self.certified_coprime_pairs + self.certified_common_factor_pairs
    }
}

/// Computes the exact Sylvester resultant of two univariate polynomials.
///
/// Coefficients are supplied in ascending power order. Constant inputs use the
/// standard resultant convention `Res(f, c) = c^deg(f)` and
/// `Res(c, g) = c^deg(g)`, avoiding an artificial zero-size determinant. For
/// two nonconstant inputs, the Sylvester matrix is built exactly and evaluated
/// through Bareiss fraction-free elimination.
pub fn resultant_univariate_polynomials(
    left: &[Real],
    right: &[Real],
    min_precision: i32,
) -> Result<UnivariateResultantReport, UnivariateResultantError> {
    if left.is_empty() || right.is_empty() {
        return Err(UnivariateResultantError::EmptyPolynomial);
    }

    let left = trim_coefficients(left, ResultantInputSide::Left, min_precision)?;
    let right = trim_coefficients(right, ResultantInputSide::Right, min_precision)?;
    let left_degree = left.len() - 1;
    let right_degree = right.len() - 1;

    if left_degree == 0 && right_degree == 0 {
        return Ok(resultant_report(left, right, Real::one(), None));
    }
    if right_degree == 0 {
        let resultant = real_pow(&right[0], left_degree);
        return Ok(resultant_report(left, right, resultant, None));
    }
    if left_degree == 0 {
        let resultant = real_pow(&left[0], right_degree);
        return Ok(resultant_report(left, right, resultant, None));
    }

    let sylvester = sylvester_matrix(&left, &right);
    let determinant = determinant_bareiss(&sylvester, min_precision)
        .map_err(UnivariateResultantError::Determinant)?;
    Ok(resultant_report(
        left,
        right,
        determinant.determinant.clone(),
        Some(determinant),
    ))
}

/// Constructs `Res_x(P(x), N(x) - y D(x))` up to one shared nonzero scale.
///
/// Multiplication by the relation in `Q[x] / (P)` has dimension `deg(P)`,
/// independent of the relation degree. Its determinant is the relation norm
/// and differs from the Sylvester resultant only by a source-leading-
/// coefficient power that is constant across `y`. Clearing all entries of the
/// two multiplication matrices with one shared scale keeps the determinant
/// polynomial integral and preserves its roots.
pub(crate) fn quotient_ring_resultant_polynomial(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<Vec<Real>> {
    let degree = source.len().checked_sub(1)?;
    if degree == 0 || numerator.is_empty() || denominator.is_empty() {
        return None;
    }
    let matrix_entries = degree.checked_mul(degree)?;
    let relation_degree = numerator.len().max(denominator.len()).checked_sub(1)?;
    let (numerator_entries, denominator_entries) =
        pseudo_quotient_multiplication_matrices(source, numerator, denominator, relation_degree)?;
    if numerator_entries.len() != matrix_entries || denominator_entries.len() != matrix_entries {
        return None;
    }

    determinant_linear_polynomial_matrix(&numerator_entries, &denominator_entries, degree).map(
        |coefficients| {
            coefficients
                .into_iter()
                .map(Rational::from_bigint)
                .map(Real::from)
                .collect()
        },
    )
}

fn determinant_linear_polynomial_matrix(
    constants: &[BigInt],
    negative_linear_coefficients: &[BigInt],
    dimension: usize,
) -> Option<Vec<BigInt>> {
    let matrix_entries = dimension.checked_mul(dimension)?;
    if constants.len() != matrix_entries || negative_linear_coefficients.len() != matrix_entries {
        return None;
    }
    let state_count = 1usize.checked_shl(u32::try_from(dimension).ok()?)?;
    let mut partials = vec![None; state_count];
    partials[0] = Some(vec![BigInt::one()]);
    for mask in 0..state_count {
        let row = usize::try_from(mask.count_ones()).ok()?;
        if row == dimension {
            continue;
        }
        let Some(partial) = partials[mask].clone() else {
            continue;
        };
        for column in 0..dimension {
            let column_bit = 1usize.checked_shl(u32::try_from(column).ok()?)?;
            if mask & column_bit != 0 {
                continue;
            }
            let entry_index = row * dimension + column;
            let sign_is_negative = (mask >> (column + 1)).count_ones() % 2 != 0;
            let next_mask = mask | column_bit;
            let next =
                partials[next_mask].get_or_insert_with(|| vec![BigInt::zero(); partial.len() + 1]);
            for (power, coefficient) in partial.iter().enumerate() {
                if !constants[entry_index].is_zero() {
                    let term = coefficient * &constants[entry_index];
                    if sign_is_negative {
                        next[power] -= term;
                    } else {
                        next[power] += term;
                    }
                }
                if !negative_linear_coefficients[entry_index].is_zero() {
                    let term = coefficient * &negative_linear_coefficients[entry_index];
                    if sign_is_negative {
                        next[power + 1] += term;
                    } else {
                        next[power + 1] -= term;
                    }
                }
            }
        }
    }
    partials.pop()?
}

#[cfg(test)]
fn quotient_ring_resultant_samples(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<Vec<Real>> {
    let degree = source.len().checked_sub(1)?;
    let matrix_entries = degree.checked_mul(degree)?;
    let relation_degree = numerator.len().max(denominator.len()).checked_sub(1)?;
    let (numerator_entries, denominator_entries) =
        pseudo_quotient_multiplication_matrices(source, numerator, denominator, relation_degree)?;
    let mut samples = Vec::with_capacity(degree + 1);
    let mut matrix = vec![BigInt::zero(); matrix_entries];
    for sample in 0..=degree {
        let image_value = i64::try_from(sample).ok()?;
        for ((entry, numerator), denominator) in matrix
            .iter_mut()
            .zip(&numerator_entries)
            .zip(&denominator_entries)
        {
            entry.clone_from(numerator);
            if image_value != 0 {
                *entry -= denominator * image_value;
            }
        }
        samples.push(Real::from(Rational::from_bigint(
            determinant_integer_bareiss_flat(&mut matrix, degree)?,
        )));
    }
    Some(samples)
}

#[cfg(test)]
fn determinant_integer_bareiss_flat(matrix: &mut [BigInt], dimension: usize) -> Option<BigInt> {
    if matrix.len() != dimension.checked_mul(dimension)? {
        return None;
    }
    if dimension == 0 {
        return Some(BigInt::one());
    }
    let mut swaps = 0;
    let mut previous_pivot = BigInt::one();
    for pivot in 0..dimension.saturating_sub(1) {
        let pivot_row = (pivot..dimension).find(|row| !matrix[row * dimension + pivot].is_zero());
        let Some(pivot_row) = pivot_row else {
            return Some(BigInt::zero());
        };
        if pivot_row != pivot {
            for column in 0..dimension {
                matrix.swap(pivot * dimension + column, pivot_row * dimension + column);
            }
            swaps += 1;
        }

        let pivot_value = matrix[pivot * dimension + pivot].clone();
        for row in (pivot + 1)..dimension {
            let eliminand = matrix[row * dimension + pivot].clone();
            for column in (pivot + 1)..dimension {
                let mut numerator = &pivot_value * &matrix[row * dimension + column];
                numerator -= &eliminand * &matrix[pivot * dimension + column];
                matrix[row * dimension + column] = if previous_pivot.is_one() {
                    numerator
                } else {
                    let (quotient, remainder) = numerator.div_rem(&previous_pivot);
                    if !remainder.is_zero() {
                        return None;
                    }
                    quotient
                };
            }
            matrix[row * dimension + pivot] = BigInt::zero();
        }
        previous_pivot = pivot_value;
    }

    let determinant = matrix[dimension * dimension - 1].clone();
    Some(if swaps % 2 == 0 {
        determinant
    } else {
        -determinant
    })
}

fn pseudo_quotient_multiplication_matrices(
    source: &[Real],
    first_relation: &[Real],
    second_relation: &[Real],
    relation_degree: usize,
) -> Option<(Vec<BigInt>, Vec<BigInt>)> {
    let source = source
        .iter()
        .map(|value| value.exact_rational_ref()?.to_big_integer())
        .collect::<Option<Vec<_>>>()?;
    let first_relation = first_relation
        .iter()
        .map(|value| value.exact_rational_ref()?.to_big_integer())
        .collect::<Option<Vec<_>>>()?;
    let second_relation = second_relation
        .iter()
        .map(|value| value.exact_rational_ref()?.to_big_integer())
        .collect::<Option<Vec<_>>>()?;
    Some((
        pseudo_quotient_multiplication_matrix(&source, &first_relation, relation_degree)?,
        pseudo_quotient_multiplication_matrix(&source, &second_relation, relation_degree)?,
    ))
}

fn pseudo_quotient_multiplication_matrix(
    source: &[BigInt],
    relation: &[BigInt],
    relation_degree: usize,
) -> Option<Vec<BigInt>> {
    let degree = source.len().checked_sub(1)?;
    let leading = source.last()?;
    if leading.is_zero() {
        return None;
    }

    let product_len = degree.checked_add(relation_degree)?;
    let mut matrix = vec![BigInt::zero(); degree.checked_mul(degree)?];
    let mut product = vec![BigInt::zero(); product_len];
    for column in 0..degree {
        for (power, coefficient) in relation.iter().enumerate() {
            product[column + power] = coefficient.clone();
        }
        for power in (degree..product_len).rev() {
            let eliminand = product[power].clone();
            let shift = power - degree;
            if !leading.is_one() {
                for coefficient in &mut product[..shift] {
                    *coefficient *= leading;
                }
            }
            for (source_power, coefficient) in source[..degree].iter().enumerate() {
                let index = shift + source_power;
                product[index] = leading * &product[index] - coefficient * &eliminand;
            }
            product[power] = BigInt::zero();
        }
        for row in 0..degree {
            matrix[row * degree + column] = std::mem::take(&mut product[row]);
        }
    }
    Some(matrix)
}

/// Builds an exact fraction-free pseudo-remainder chain.
///
/// The chain uses pseudo-division instead of field division, matching the
/// subresultant polynomial remainder sequence. This is
/// intentionally a report-bearing scheduling API: it gives curve and solver
/// callers exact elimination evidence without claiming to be a full
/// multivariate algebraic solver. The exact-computation boundary is
/// preserved by requiring certified coefficient trimming before any degree
/// decision is made.
pub fn subresultant_chain_univariate_polynomials(
    left: &[Real],
    right: &[Real],
    min_precision: i32,
) -> Result<UnivariateSubresultantChainReport, UnivariateSubresultantChainError> {
    if left.is_empty() || right.is_empty() {
        return Err(UnivariateSubresultantChainError::EmptyPolynomial);
    }

    let left = trim_coefficients(left, ResultantInputSide::Left, min_precision)
        .map_err(map_trim_error_to_chain)?;
    let right = trim_coefficients(right, ResultantInputSide::Right, min_precision)
        .map_err(map_trim_error_to_chain)?;

    let mut previous = left.clone();
    let mut current = right.clone();
    if degree(&previous) < degree(&current) {
        std::mem::swap(&mut previous, &mut current);
    }

    let mut steps = Vec::new();
    while !is_zero_polynomial(&current, min_precision).map_err(|(side, index)| {
        UnivariateSubresultantChainError::UndecidedCoefficient { side, index }
    })? && degree(&current) > 0
    {
        let divisor_leading_coefficient = current[current.len() - 1].clone();
        let remainder = pseudo_remainder(&previous, &current, min_precision)?;
        let zero_remainder =
            is_zero_polynomial(&remainder, min_precision).map_err(|(side, index)| {
                UnivariateSubresultantChainError::UndecidedCoefficient { side, index }
            })?;
        let signed_remainder = if zero_remainder {
            vec![Real::zero()]
        } else {
            remainder
                .into_iter()
                .map(|value| -value)
                .collect::<Vec<_>>()
        };
        let remainder_degree = degree(&signed_remainder);
        steps.push(UnivariateSubresultantChainStep {
            dividend_degree: degree(&previous),
            divisor_degree: degree(&current),
            divisor_leading_coefficient,
            remainder: signed_remainder.clone(),
            remainder_degree,
            zero_remainder,
        });

        if zero_remainder {
            break;
        }
        previous = current;
        current = signed_remainder;
    }

    let last_nonzero = if is_zero_polynomial(&current, min_precision).map_err(|(side, index)| {
        UnivariateSubresultantChainError::UndecidedCoefficient { side, index }
    })? {
        previous
    } else {
        current
    };
    let last_nonzero_degree = degree(&last_nonzero);

    Ok(UnivariateSubresultantChainReport {
        left_coefficients: left,
        right_coefficients: right,
        steps,
        last_nonzero,
        last_nonzero_degree,
        has_nonconstant_common_factor: last_nonzero_degree > 0,
    })
}

/// Schedule exact univariate resultant checks for many polynomial pairs.
///
/// This is the generic hypersolve side of curve-pair elimination scheduling:
/// curve crates own span construction and topology, then pass univariate
/// coefficient pairs here for exact algebraic filtering. Each pair first runs
/// the Sylvester resultant; zero resultants then run the fraction-free
/// pseudo-remainder chain. Exact reports, rather than sampled approximations,
/// drive every branching decision.
pub fn schedule_univariate_resultant_pairs(
    pairs: &[UnivariateResultantPairInput],
    min_precision: i32,
) -> UnivariateResultantScheduleReport {
    let reports = pairs
        .iter()
        .map(|pair| schedule_one_resultant_pair(pair, min_precision))
        .collect::<Vec<_>>();
    let certified_coprime_pairs = reports
        .iter()
        .filter(|pair| pair.status == UnivariateResultantPairStatus::CertifiedCoprime)
        .count();
    let certified_common_factor_pairs = reports
        .iter()
        .filter(|pair| pair.status == UnivariateResultantPairStatus::CertifiedCommonFactor)
        .count();
    let unresolved_pairs = reports.len() - certified_coprime_pairs - certified_common_factor_pairs;

    UnivariateResultantScheduleReport {
        pairs: reports,
        certified_coprime_pairs,
        certified_common_factor_pairs,
        unresolved_pairs,
    }
}

fn resultant_report(
    left_coefficients: Vec<Real>,
    right_coefficients: Vec<Real>,
    resultant: Real,
    determinant: Option<BareissDeterminantReport>,
) -> UnivariateResultantReport {
    let left_degree = left_coefficients.len() - 1;
    let right_degree = right_coefficients.len() - 1;
    UnivariateResultantReport {
        left_coefficients,
        right_coefficients,
        left_degree,
        right_degree,
        sylvester_dimension: left_degree + right_degree,
        resultant,
        determinant,
    }
}

fn schedule_one_resultant_pair(
    pair: &UnivariateResultantPairInput,
    min_precision: i32,
) -> UnivariateResultantPairReport {
    let resultant = match resultant_univariate_polynomials(
        &pair.left_coefficients,
        &pair.right_coefficients,
        min_precision,
    ) {
        Ok(resultant) => resultant,
        Err(error) => {
            return UnivariateResultantPairReport {
                pair_index: pair.pair_index,
                status: UnivariateResultantPairStatus::ResultantError,
                resultant: None,
                subresultant_chain: None,
                resultant_error: Some(error),
                subresultant_error: None,
            };
        }
    };

    let resultant_sign = match resultant.resultant.certified_sign_until(min_precision) {
        CertifiedRealSign::Known { sign, .. } => sign,
        CertifiedRealSign::Unknown { .. } => {
            return UnivariateResultantPairReport {
                pair_index: pair.pair_index,
                status: UnivariateResultantPairStatus::UndecidedResultantSign,
                resultant: Some(resultant),
                subresultant_chain: None,
                resultant_error: None,
                subresultant_error: None,
            };
        }
    };
    if !matches!(resultant_sign, RealSign::Zero) {
        return UnivariateResultantPairReport {
            pair_index: pair.pair_index,
            status: UnivariateResultantPairStatus::CertifiedCoprime,
            resultant: Some(resultant),
            subresultant_chain: None,
            resultant_error: None,
            subresultant_error: None,
        };
    }

    let chain = match subresultant_chain_univariate_polynomials(
        &pair.left_coefficients,
        &pair.right_coefficients,
        min_precision,
    ) {
        Ok(chain) => chain,
        Err(error) => {
            return UnivariateResultantPairReport {
                pair_index: pair.pair_index,
                status: UnivariateResultantPairStatus::SubresultantChainError,
                resultant: Some(resultant),
                subresultant_chain: None,
                resultant_error: None,
                subresultant_error: Some(error),
            };
        }
    };
    let status = if chain.has_nonconstant_common_factor {
        UnivariateResultantPairStatus::CertifiedCommonFactor
    } else {
        UnivariateResultantPairStatus::ResultantZeroWithoutCommonFactorReport
    };

    UnivariateResultantPairReport {
        pair_index: pair.pair_index,
        status,
        resultant: Some(resultant),
        subresultant_chain: Some(chain),
        resultant_error: None,
        subresultant_error: None,
    }
}

fn trim_coefficients(
    coefficients: &[Real],
    side: ResultantInputSide,
    min_precision: i32,
) -> Result<Vec<Real>, UnivariateResultantError> {
    let mut last_nonzero = None;
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => {
                last_nonzero = Some(index);
                break;
            }
            CertifiedRealSign::Unknown { .. } => {
                return Err(UnivariateResultantError::UndecidedCoefficient { side, index });
            }
        }
    }

    let Some(last_nonzero) = last_nonzero else {
        return Ok(vec![Real::zero()]);
    };
    Ok(coefficients[..=last_nonzero].to_vec())
}

fn map_trim_error_to_chain(error: UnivariateResultantError) -> UnivariateSubresultantChainError {
    match error {
        UnivariateResultantError::EmptyPolynomial => {
            UnivariateSubresultantChainError::EmptyPolynomial
        }
        UnivariateResultantError::UndecidedCoefficient { side, index } => {
            UnivariateSubresultantChainError::UndecidedCoefficient { side, index }
        }
        UnivariateResultantError::Determinant(_) => {
            unreachable!("trimming does not compute determinants")
        }
    }
}

fn pseudo_remainder(
    dividend: &[Real],
    divisor: &[Real],
    min_precision: i32,
) -> Result<Vec<Real>, UnivariateSubresultantChainError> {
    let mut remainder = dividend.to_vec();
    let divisor_degree = degree(divisor);
    let divisor_leading = divisor[divisor_degree].clone();

    while degree(&remainder) >= divisor_degree
        && !is_zero_polynomial(&remainder, min_precision).map_err(|(side, index)| {
            UnivariateSubresultantChainError::UndecidedCoefficient { side, index }
        })?
    {
        let remainder_degree = degree(&remainder);
        let shift = remainder_degree - divisor_degree;
        let remainder_leading = remainder[remainder_degree].clone();
        let mut next = remainder
            .iter()
            .map(|coefficient| divisor_leading.clone() * coefficient.clone())
            .collect::<Vec<_>>();
        for (index, coefficient) in divisor.iter().enumerate() {
            let target = index + shift;
            next[target] = next[target].clone() - remainder_leading.clone() * coefficient.clone();
        }
        remainder = trim_coefficients(&next, ResultantInputSide::Left, min_precision)
            .map_err(map_trim_error_to_chain)?;
    }

    Ok(remainder)
}

fn degree(polynomial: &[Real]) -> usize {
    polynomial.len().saturating_sub(1)
}

fn is_zero_polynomial(
    polynomial: &[Real],
    min_precision: i32,
) -> Result<bool, (ResultantInputSide, usize)> {
    for (index, coefficient) in polynomial.iter().enumerate() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(false),
            CertifiedRealSign::Unknown { .. } => return Err((ResultantInputSide::Left, index)),
        }
    }
    Ok(true)
}

fn sylvester_matrix(left: &[Real], right: &[Real]) -> Vec<Vec<Real>> {
    let left_degree = left.len() - 1;
    let right_degree = right.len() - 1;
    let dimension = left_degree + right_degree;
    let mut matrix = vec![vec![Real::zero(); dimension]; dimension];

    for row in 0..right_degree {
        for (degree, coefficient) in left.iter().enumerate() {
            matrix[row][row + left_degree - degree] = coefficient.clone();
        }
    }
    for row in 0..left_degree {
        for (degree, coefficient) in right.iter().enumerate() {
            matrix[right_degree + row][row + right_degree - degree] = coefficient.clone();
        }
    }

    matrix
}

fn real_pow(value: &Real, exponent: usize) -> Real {
    let mut result = Real::one();
    let mut base = value.clone();
    let mut remaining = exponent;
    while remaining > 0 {
        if remaining & 1 == 1 {
            result *= base.clone();
        }
        remaining >>= 1;
        if remaining > 0 {
            base = base.clone() * base;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    #[test]
    fn resultant_reports_common_linear_root_as_zero() {
        let report = resultant_univariate_polynomials(
            &[real(-1), real(1)],
            &[real(-1), real(0), real(1)],
            -64,
        )
        .unwrap();

        assert_eq!(report.left_degree, 1);
        assert_eq!(report.right_degree, 2);
        assert_eq!(report.sylvester_dimension, 3);
        assert_eq!(report.resultant, Real::zero());
        assert!(report.determinant.is_some());
    }

    #[test]
    fn resultant_of_linear_polynomials_is_exact_difference() {
        let report =
            resultant_univariate_polynomials(&[real(-2), real(1)], &[real(-5), real(1)], -64)
                .unwrap();

        assert_eq!(report.resultant, real(-3));
    }

    #[test]
    fn quotient_ring_samples_match_sylvester_resultants_up_to_one_scale() {
        let source = [real(2), real(-3), Real::zero(), real(2)];
        let numerator = [real(1), real(2), real(3)];
        let denominator = [real(2), real(-1)];
        let quotient_samples =
            quotient_ring_resultant_samples(&source, &numerator, &denominator).unwrap();
        let quotient_polynomial =
            quotient_ring_resultant_polynomial(&source, &numerator, &denominator).unwrap();
        let polynomial_samples = (0..source.len())
            .map(|sample| {
                let image = real(i64::try_from(sample).unwrap());
                quotient_polynomial
                    .iter()
                    .rev()
                    .fold(Real::zero(), |value, coefficient| {
                        value * image.clone() + coefficient.clone()
                    })
            })
            .collect::<Vec<_>>();
        assert_eq!(polynomial_samples, quotient_samples);
        let sylvester_samples = (0..source.len())
            .map(|sample| {
                let image = real(i64::try_from(sample).unwrap());
                let mut relation = numerator.to_vec();
                for (coefficient, denominator_coefficient) in relation.iter_mut().zip(&denominator)
                {
                    *coefficient =
                        coefficient.clone() - denominator_coefficient.clone() * image.clone();
                }
                resultant_univariate_polynomials(&source, &relation, -64)
                    .unwrap()
                    .resultant
            })
            .collect::<Vec<_>>();
        let pivot = quotient_samples
            .iter()
            .zip(&sylvester_samples)
            .position(|(quotient, sylvester)| {
                quotient != &Real::zero() && sylvester != &Real::zero()
            })
            .unwrap();

        for (quotient, sylvester) in quotient_samples.iter().zip(&sylvester_samples) {
            assert_eq!(
                quotient.clone() * sylvester_samples[pivot].clone(),
                sylvester.clone() * quotient_samples[pivot].clone()
            );
        }
    }

    #[test]
    fn flat_integer_bareiss_matches_reported_determinants() {
        let cases = [
            (0, vec![]),
            (1, vec![5]),
            (2, vec![0, 2, 3, 4]),
            (2, vec![1, 2, 2, 4]),
            (3, vec![2, -1, 3, 4, 0, 1, -2, 5, 2]),
        ];
        for (dimension, entries) in cases {
            let mut flat = entries
                .iter()
                .copied()
                .map(BigInt::from)
                .collect::<Vec<_>>();
            let matrix = entries
                .chunks(dimension.max(1))
                .map(|row| row.iter().copied().map(real).collect::<Vec<_>>())
                .collect::<Vec<_>>();
            let expected = determinant_bareiss(&matrix, -64).unwrap().determinant;

            assert_eq!(
                determinant_integer_bareiss_flat(&mut flat, dimension)
                    .map(Rational::from_bigint)
                    .map(Real::from),
                Some(expected)
            );
        }
        let mut wrong_size = vec![BigInt::one()];
        assert_eq!(determinant_integer_bareiss_flat(&mut wrong_size, 2), None);
    }

    #[test]
    fn resultant_validates_empty_and_zero_polynomials() {
        assert_eq!(
            resultant_univariate_polynomials(&[], &[real(1)], -64).unwrap_err(),
            UnivariateResultantError::EmptyPolynomial
        );

        let report =
            resultant_univariate_polynomials(&[Real::zero()], &[real(-3), real(1)], -64).unwrap();
        assert_eq!(report.left_degree, 0);
        assert_eq!(report.resultant, Real::zero());
        assert!(report.determinant.is_none());
    }

    #[test]
    fn constant_resultant_uses_exact_binary_power_for_high_degree() {
        let mut polynomial = vec![Real::zero(); 65];
        polynomial[0] = real(-1);
        polynomial[64] = Real::one();
        let report = resultant_univariate_polynomials(&polynomial, &[real(2)], -64).unwrap();
        let expected = (0..64).fold(Real::one(), |value, _| value * real(2));

        assert_eq!(report.resultant, expected);
        assert_eq!(report.left_degree, 64);
        assert!(report.determinant.is_none());
    }

    #[test]
    fn subresultant_chain_reports_common_factor() {
        let report = subresultant_chain_univariate_polynomials(
            &[real(2), real(-3), real(1)],
            &[real(-1), real(1)],
            -64,
        )
        .unwrap();

        assert!(report.has_nonconstant_common_factor);
        assert_eq!(report.last_nonzero_degree, 1);
        assert_eq!(report.last_nonzero, vec![real(-1), real(1)]);
        assert!(report.steps.last().unwrap().zero_remainder);
    }

    #[test]
    fn subresultant_chain_reports_coprime_linear_inputs() {
        let report = subresultant_chain_univariate_polynomials(
            &[real(-2), real(1)],
            &[real(-5), real(1)],
            -64,
        )
        .unwrap();

        assert!(!report.has_nonconstant_common_factor);
        assert_eq!(report.last_nonzero_degree, 0);
        assert_eq!(report.last_nonzero, vec![real(-3)]);
    }

    #[test]
    fn resultant_schedule_classifies_coprime_common_and_bad_pairs() {
        let report = schedule_univariate_resultant_pairs(
            &[
                UnivariateResultantPairInput {
                    pair_index: 10,
                    left_coefficients: vec![real(-1), real(1)],
                    right_coefficients: vec![real(-2), real(1)],
                },
                UnivariateResultantPairInput {
                    pair_index: 11,
                    left_coefficients: vec![real(-1), real(1)],
                    right_coefficients: vec![real(-1), Real::zero(), Real::one()],
                },
                UnivariateResultantPairInput {
                    pair_index: 12,
                    left_coefficients: Vec::new(),
                    right_coefficients: vec![real(1)],
                },
            ],
            -64,
        );

        assert_eq!(report.pairs.len(), 3);
        assert_eq!(report.certified_coprime_pairs, 1);
        assert_eq!(report.certified_common_factor_pairs, 1);
        assert_eq!(report.unresolved_pairs, 1);
        assert_eq!(report.pairs[0].pair_index, 10);
        assert_eq!(
            report.pairs[0].status,
            UnivariateResultantPairStatus::CertifiedCoprime
        );
        assert_eq!(
            report.pairs[1].status,
            UnivariateResultantPairStatus::CertifiedCommonFactor
        );
        assert_eq!(
            report.pairs[2].status,
            UnivariateResultantPairStatus::ResultantError
        );
        assert!(report.pairs[1].subresultant_chain.is_some());
        assert!(report.pairs[2].resultant_error.is_some());
    }

    proptest! {
        #[test]
        fn generated_flat_integer_bareiss_matches_reported_four_by_four(
            entries in prop::collection::vec(-8_i16..=8, 16),
        ) {
            let mut flat = entries
                .iter()
                .copied()
                .map(|value| BigInt::from(i64::from(value)))
                .collect::<Vec<_>>();
            let matrix = entries
                .chunks_exact(4)
                .map(|row| {
                    row.iter()
                        .copied()
                        .map(|value| real(i64::from(value)))
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>();
            let expected = determinant_bareiss(&matrix, -64).unwrap().determinant;

            prop_assert_eq!(
                determinant_integer_bareiss_flat(&mut flat, 4)
                    .map(Rational::from_bigint)
                    .map(Real::from),
                Some(expected)
            );
        }

        #[test]
        fn generated_monic_quotient_ring_samples_match_sylvester_up_to_scale(
            source_constant in -4_i16..=4,
            source_linear in -4_i16..=4,
            source_quadratic in -4_i16..=4,
            numerator_constant in -4_i16..=4,
            numerator_linear in -4_i16..=4,
            numerator_quadratic in 1_i16..=4,
            denominator_linear in -4_i16..=4,
        ) {
            let source = [
                real(i64::from(source_constant)),
                real(i64::from(source_linear)),
                real(i64::from(source_quadratic)),
                Real::one(),
            ];
            let numerator = [
                real(i64::from(numerator_constant)),
                real(i64::from(numerator_linear)),
                real(i64::from(numerator_quadratic)),
            ];
            let denominator = [Real::one(), real(i64::from(denominator_linear))];
            let samples =
                quotient_ring_resultant_samples(&source, &numerator, &denominator).unwrap();
            let polynomial =
                quotient_ring_resultant_polynomial(&source, &numerator, &denominator).unwrap();
            let polynomial_samples = (0..source.len())
                .map(|sample| {
                    let image = real(i64::try_from(sample).unwrap());
                    polynomial
                        .iter()
                        .rev()
                        .fold(Real::zero(), |value, coefficient| {
                            value * image.clone() + coefficient.clone()
                        })
                })
                .collect::<Vec<_>>();
            prop_assert_eq!(&polynomial_samples, &samples);

            let sylvester_samples = (0..source.len())
                .map(|sample| {
                    let image = real(i64::try_from(sample).unwrap());
                    let mut relation = numerator.to_vec();
                    for (coefficient, denominator_coefficient) in
                        relation.iter_mut().zip(&denominator)
                    {
                        *coefficient =
                            coefficient.clone() - denominator_coefficient.clone() * image.clone();
                    }
                    resultant_univariate_polynomials(&source, &relation, -64)
                        .unwrap()
                        .resultant
                })
                .collect::<Vec<_>>();
            let pivot = samples
                .iter()
                .zip(&sylvester_samples)
                .position(|(quotient, sylvester)| {
                    quotient != &Real::zero() && sylvester != &Real::zero()
                });

            if let Some(pivot) = pivot {
                for (quotient, sylvester) in samples.iter().zip(&sylvester_samples) {
                    prop_assert_eq!(
                        quotient.clone() * sylvester_samples[pivot].clone(),
                        sylvester.clone() * samples[pivot].clone()
                    );
                }
            } else {
                prop_assert!(samples.iter().all(|sample| sample == &Real::zero()));
                prop_assert!(
                    sylvester_samples
                        .iter()
                        .all(|sample| sample == &Real::zero())
                );
            }
        }

        #[test]
        fn generated_linear_resultants_match_root_difference(
            left_root in -64_i16..=64,
            right_root in -64_i16..=64,
        ) {
            let left_root = i64::from(left_root);
            let right_root = i64::from(right_root);
            let report = resultant_univariate_polynomials(
                &[real(-left_root), real(1)],
                &[real(-right_root), real(1)],
                -64,
            ).unwrap();

            prop_assert_eq!(report.resultant, real(left_root - right_root));
        }

        #[test]
        fn generated_common_linear_factor_resultant_is_zero(
            shared_root in -32_i16..=32,
            other_root in -32_i16..=32,
        ) {
            let shared_root = i64::from(shared_root);
            let other_root = i64::from(other_root);
            let quadratic = vec![
                real(shared_root * other_root),
                real(-(shared_root + other_root)),
                real(1),
            ];
            let report = resultant_univariate_polynomials(
                &[real(-shared_root), real(1)],
                &quadratic,
                -64,
            ).unwrap();

            prop_assert_eq!(report.resultant, Real::zero());
        }

        #[test]
        fn generated_subresultant_chain_detects_common_linear_factor(
            shared_root in -32_i16..=32,
            other_root in -32_i16..=32,
        ) {
            let shared_root = i64::from(shared_root);
            let other_root = i64::from(other_root);
            let quadratic = vec![
                real(shared_root * other_root),
                real(-(shared_root + other_root)),
                real(1),
            ];
            let report = subresultant_chain_univariate_polynomials(
                &quadratic,
                &[real(-shared_root), real(1)],
                -64,
            ).unwrap();

            prop_assert!(report.has_nonconstant_common_factor);
            prop_assert_eq!(report.last_nonzero_degree, 1);
        }

        #[test]
        fn generated_resultant_schedule_classifies_linear_pairs(
            left_root in -32_i16..=32,
            right_root in -32_i16..=32,
        ) {
            let left_root = i64::from(left_root);
            let right_root = i64::from(right_root);
            let report = schedule_univariate_resultant_pairs(
                &[UnivariateResultantPairInput {
                    pair_index: 0,
                    left_coefficients: vec![real(-left_root), real(1)],
                    right_coefficients: vec![real(-right_root), real(1)],
                }],
                -64,
            );

            if left_root == right_root {
                prop_assert_eq!(
                    &report.pairs[0].status,
                    &UnivariateResultantPairStatus::CertifiedCommonFactor
                );
                prop_assert_eq!(report.certified_common_factor_pairs, 1);
            } else {
                prop_assert_eq!(
                    &report.pairs[0].status,
                    &UnivariateResultantPairStatus::CertifiedCoprime
                );
                prop_assert_eq!(report.certified_coprime_pairs, 1);
            }
            prop_assert!(report.all_classified());
        }
    }
}
