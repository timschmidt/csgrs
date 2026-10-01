//! Dense tensor construction and selected-root relation reduction.

use hyperreal::Real;

use crate::algebraic::AlgebraicRootRepresentation;
use crate::curve_resultant::BivariatePolynomial;
use crate::tensor_resultant::DenseTensorPolynomial;

/// Appends one output axis of the given degree to a dense tensor.
pub fn dense_tensor_with_output_axis(
    polynomial: &DenseTensorPolynomial,
) -> Option<DenseTensorPolynomial> {
    let mut dimensions = Vec::new();
    dimensions
        .try_reserve_exact(polynomial.dimensions().len().checked_add(1)?)
        .ok()?;
    dimensions.extend_from_slice(polynomial.dimensions());
    dimensions.push(1);
    let mut coefficients = Vec::new();
    coefficients
        .try_reserve_exact(polynomial.coefficients().len())
        .ok()?;
    coefficients.extend(polynomial.coefficients().iter().cloned());
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

/// Converts a bivariate coefficient grid into a rank-two dense tensor.
pub fn bivariate_dense_tensor(polynomial: &BivariatePolynomial) -> Option<DenseTensorPolynomial> {
    let first_count = polynomial.coefficients.len().max(1);
    let second_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(1)
        .max(1);
    let coefficient_count = first_count.checked_mul(second_count)?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(coefficient_count).ok()?;
    coefficients.resize_with(coefficient_count, Real::zero);
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            coefficients[first * second_count + second] = coefficient.clone();
        }
    }
    DenseTensorPolynomial::try_new(vec![first_count, second_count], coefficients)
}

/// Converts a bivariate polynomial into a rank-three dense tensor with an output axis.
pub fn bivariate_tensor_with_output_axis(
    polynomial: &BivariatePolynomial,
) -> Option<DenseTensorPolynomial> {
    dense_tensor_with_output_axis(&bivariate_dense_tensor(polynomial)?)
}

/// Clones a dense tensor with fallible allocation.
pub fn try_clone_dense_tensor(polynomial: &DenseTensorPolynomial) -> Option<DenseTensorPolynomial> {
    let mut dimensions = Vec::new();
    dimensions
        .try_reserve_exact(polynomial.dimensions().len())
        .ok()?;
    dimensions.extend_from_slice(polynomial.dimensions());
    let mut coefficients = Vec::new();
    coefficients
        .try_reserve_exact(polynomial.coefficients().len())
        .ok()?;
    coefficients.extend(polynomial.coefficients().iter().cloned());
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

/// Reduces every source axis modulo its selected root defining polynomial.
pub fn dense_reduce_selected_tuple_relations(
    mut polynomial: DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<DenseTensorPolynomial> {
    if polynomial.dimensions().len() != sources.len() {
        return None;
    }
    for (axis, source) in sources.iter().enumerate() {
        let count = *polynomial.dimensions().get(axis)?;
        let degree = source.polynomial_coefficients.len().checked_sub(1)?;
        if count > degree {
            let reduced = polynomial.reduce_axis_modulo(
                axis,
                &source.polynomial_coefficients,
                crate::PredicatePolicy::STRICT,
            )?;
            polynomial = reduced;
        }
    }
    Some(polynomial)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bivariate_grids_embed_with_an_output_axis() {
        // 1 + 2t + 3s, with a ragged first row.
        let polynomial = BivariatePolynomial::new(vec![
            vec![Real::from(1), Real::from(2)],
            vec![Real::from(3)],
        ]);
        let dense = bivariate_dense_tensor(&polynomial).unwrap();
        assert_eq!(dense.dimensions(), &[2, 2]);
        assert_eq!(
            dense
                .coefficients()
                .iter()
                .filter(|value| **value != Real::zero())
                .count(),
            3
        );
        let lifted = bivariate_tensor_with_output_axis(&polynomial).unwrap();
        assert_eq!(lifted.dimensions().len(), 3);
        assert_eq!(lifted.coefficients().len() % dense.coefficients().len(), 0);
        let clone = try_clone_dense_tensor(&dense).unwrap();
        assert_eq!(clone.dimensions(), dense.dimensions());
        assert_eq!(clone.coefficients(), dense.coefficients());
    }
}
