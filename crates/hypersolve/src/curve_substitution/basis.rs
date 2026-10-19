//! Scalar basis conversion shared by curve and tensor surface consumers.

use super::*;
use crate::bivariate_arithmetic::{polynomial_add, polynomial_multiply, polynomial_scale};

/// Converts scalar Bernstein coefficients to ascending power coefficients.
///
/// The existing bounded-degree conversion preserves native arithmetic order.
/// Higher degrees use forward differences `binomial(n,k) * delta^k(values)[0]`,
/// with exact scalar binomial factors rather than fixed-width integer products.
/// Empty input denotes the zero polynomial.
pub fn bernstein_scalar_power_basis(values: &[Real]) -> Vec<Real> {
    let Some(degree) = values.len().checked_sub(1) else {
        return Vec::new();
    };
    if degree <= MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE {
        // Share the existing native conversion and its arithmetic order on
        // the degree range where its fixed-width binomial products are safe.
        return bernstein_scalars_to_power_basis(values);
    }
    let mut differences = values.to_vec();
    let mut binomial = Real::one();
    let mut result = Vec::with_capacity(values.len());
    for power in 0..=degree {
        result.push(&binomial * &differences[0]);
        if power == degree {
            break;
        }
        for index in 0..degree - power {
            differences[index] = &differences[index + 1] - &differences[index];
        }
        binomial = (binomial * Real::from((degree - power) as u128)
            / Real::from((power + 1) as u128))
        .expect("positive integer binomial divisor");
    }
    result
}

/// Returns all nonzero B-spline basis polynomials on one knot span.
///
/// Entry `j` belongs to control `span-degree+j`. Polynomials use the original
/// knot coordinate, not a rebased unit interval. Knot order and positive span
/// length are checked with the supplied precision contract. Zero knot gaps
/// contribute zero terms in the Cox-de Boor recurrence.
pub fn bspline_span_basis_power_basis(
    knots: &[Real],
    degree: usize,
    span: usize,
    min_precision: i32,
) -> Result<Vec<Vec<Real>>, BsplineKnotSpanSubstitutionStatus> {
    use BsplineKnotSpanSubstitutionStatus as Status;
    let count = knots
        .len()
        .checked_sub(degree.checked_add(1).ok_or(Status::InvalidDegree)?)
        .ok_or(Status::InvalidKnotVectorLength)?;
    if degree == 0 || count <= degree {
        return Err(Status::InvalidDegree);
    }
    if span < degree || span >= count {
        return Err(Status::SpanOutOfRange);
    }
    let positive = |value: &Real| match value.certified_sign_until(min_precision) {
        CertifiedRealSign::Known {
            sign: RealSign::Negative,
            ..
        } => Err(Status::DecreasingKnotVector),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        } => Ok(false),
        CertifiedRealSign::Known {
            sign: RealSign::Positive,
            ..
        } => Ok(true),
        CertifiedRealSign::Unknown { .. } => Err(Status::UndecidedKnotOrder),
    };
    for pair in knots.windows(2) {
        positive(&(&pair[1] - &pair[0]))?;
    }
    if !positive(&(&knots[span + 1] - &knots[span]))? {
        return Err(Status::ZeroLengthSpan);
    }
    let mut previous = vec![vec![Real::one()]];
    for order in 1..=degree {
        let mut next = Vec::with_capacity(order + 1);
        for local in 0..=order {
            let i = span - order + local;
            let mut value = Vec::new();
            if local > 0 {
                let gap = &knots[i + order] - &knots[i];
                if positive(&gap)? {
                    let factor = (Real::one() / gap).map_err(|_| Status::DivisionFailed)?;
                    value = polynomial_scale(
                        &polynomial_multiply(&previous[local - 1], &[-&knots[i], Real::one()]),
                        &factor,
                    );
                }
            }
            if local < order {
                let gap = &knots[i + order + 1] - &knots[i + 1];
                if positive(&gap)? {
                    let factor = (Real::one() / gap).map_err(|_| Status::DivisionFailed)?;
                    value = polynomial_add(
                        &value,
                        &polynomial_scale(
                            &polynomial_multiply(
                                &previous[local],
                                &[knots[i + order + 1].clone(), Real::from(-1)],
                            ),
                            &factor,
                        ),
                    );
                }
            }
            next.push(value);
        }
        previous = next;
    }
    Ok(previous)
}
