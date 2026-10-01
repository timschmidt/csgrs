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

/// Returns the coefficient tensor of one power of the last axis.
pub fn dense_last_axis_coefficient(
    polynomial: &DenseTensorPolynomial,
    power: usize,
) -> Option<DenseTensorPolynomial> {
    let target_count = *polynomial.dimensions().last()?;
    if power >= target_count {
        let mut dimensions = polynomial.dimensions().to_vec();
        *dimensions.last_mut()? = 1;
        return DenseTensorPolynomial::zero(dimensions);
    }
    let mut dimensions = polynomial.dimensions().to_vec();
    *dimensions.last_mut()? = 1;
    let fiber_count = polynomial.coefficients().len() / target_count;
    let coefficients = (0..fiber_count)
        .map(|fiber| polynomial.coefficients()[fiber * target_count + power].clone())
        .collect();
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

/// Reduces every axis modulo its selected root defining polynomial.
pub fn dense_reduce_selected_root_relations(
    mut polynomial: DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<DenseTensorPolynomial> {
    if polynomial.dimensions().len() != sources.len() + 1 {
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

/// Embeds a dense tensor into a larger axis set.
pub fn dense_tensor_embed_axes(
    polynomial: &DenseTensorPolynomial,
    target_rank: usize,
    axes: &[usize],
) -> Option<DenseTensorPolynomial> {
    if polynomial.dimensions().len() != axes.len() || axes.iter().any(|axis| *axis >= target_rank) {
        return None;
    }
    let mut dimensions = vec![1_usize; target_rank];
    for (dimension, axis) in polynomial.dimensions().iter().zip(axes) {
        dimensions[*axis] = dimensions[*axis].checked_add(dimension.checked_sub(1)?)?;
    }
    let coefficient_count = dimensions
        .iter()
        .try_fold(1_usize, |count, dimension| count.checked_mul(*dimension))?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(coefficient_count).ok()?;
    coefficients.resize_with(coefficient_count, Real::zero);
    for (source_index, coefficient) in polynomial.coefficients().iter().enumerate() {
        let mut remaining = source_index;
        let mut target_exponents = vec![0_usize; target_rank];
        for source_axis in (0..polynomial.dimensions().len()).rev() {
            let dimension = polynomial.dimensions()[source_axis];
            let exponent = remaining % dimension;
            remaining /= dimension;
            target_exponents[axes[source_axis]] =
                target_exponents[axes[source_axis]].checked_add(exponent)?;
        }
        let target_index = target_exponents
            .iter()
            .zip(&dimensions)
            .try_fold(0_usize, |index, (exponent, dimension)| {
                index.checked_mul(*dimension)?.checked_add(*exponent)
            })?;
        coefficients[target_index] += coefficient;
    }
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

/// Packs coefficient-field tensors into a dense polynomial whose final axis
/// is the later curve parameter. Source axes retain their existing order.
pub fn dense_tensor_from_polynomial_coefficients(
    coefficients: &[&DenseTensorPolynomial],
) -> Option<DenseTensorPolynomial> {
    let first = coefficients.first()?;
    let source_rank = first.dimensions().len();
    if coefficients
        .iter()
        .any(|coefficient| coefficient.dimensions().len() != source_rank)
    {
        return None;
    }
    let mut dimensions = vec![1_usize; source_rank];
    for coefficient in coefficients {
        for (dimension, candidate) in dimensions.iter_mut().zip(coefficient.dimensions()) {
            *dimension = (*dimension).max(*candidate);
        }
    }
    dimensions.push(coefficients.len());
    let coefficient_count = dimensions
        .iter()
        .try_fold(1_usize, |count, dimension| count.checked_mul(*dimension))?;
    let mut packed = Vec::new();
    packed.try_reserve_exact(coefficient_count).ok()?;
    packed.resize_with(coefficient_count, Real::zero);
    for (power, coefficient) in coefficients.iter().enumerate() {
        for (flat_index, value) in coefficient.coefficients().iter().enumerate() {
            let mut remaining = flat_index;
            let mut exponents = vec![0_usize; source_rank];
            for axis in (0..source_rank).rev() {
                let dimension = coefficient.dimensions()[axis];
                exponents[axis] = remaining % dimension;
                remaining /= dimension;
            }
            let mut target_index = 0_usize;
            for (exponent, dimension) in exponents.iter().zip(&dimensions[..source_rank]) {
                target_index = target_index
                    .checked_mul(*dimension)?
                    .checked_add(*exponent)?;
            }
            target_index = target_index
                .checked_mul(coefficients.len())?
                .checked_add(power)?;
            packed[target_index] = value.clone();
        }
    }
    DenseTensorPolynomial::try_new(dimensions, packed)
}

/// Builds one minimal affine tensor basis for independently represented
/// scalars. Exact point coordinates become constants, while roots proved
/// affine images of an earlier source reuse that axis. Every relation is
/// certified under STRICT before it can change the tensor rank.
/// Returns `value` as an exact rational tensor constant, when it is one.
pub fn rational_tensor_constant(value: &Real) -> Option<Real> {
    if value.exact_rational_ref().is_some() {
        return Some(value.clone());
    }
    value.exact_rational_normal_form().map(Real::new)
}

/// Returns the exact derivative with respect to the last axis.
pub fn dense_last_axis_derivative(
    polynomial: &DenseTensorPolynomial,
) -> Option<DenseTensorPolynomial> {
    let target_count = *polynomial.dimensions().last()?;
    let derivative_count = target_count.saturating_sub(1).max(1);
    let mut dimensions = polynomial.dimensions().to_vec();
    *dimensions.last_mut()? = derivative_count;
    let fiber_count = polynomial.coefficients().len().checked_div(target_count)?;
    let coefficient_count = fiber_count.checked_mul(derivative_count)?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(coefficient_count).ok()?;
    for fiber in polynomial.coefficients().chunks_exact(target_count) {
        if target_count == 1 {
            coefficients.push(Real::zero());
            continue;
        }
        for (power, coefficient) in fiber.iter().enumerate().skip(1) {
            coefficients.push(coefficient * Real::from(u64::try_from(power).ok()?));
        }
    }
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

/// Substitutes an exact value for the last axis.
pub fn dense_specialize_last_axis(
    polynomial: &DenseTensorPolynomial,
    parameter: &Real,
) -> Option<DenseTensorPolynomial> {
    let target_count = *polynomial.dimensions().last()?;
    let mut dimensions = polynomial.dimensions().to_vec();
    *dimensions.last_mut()? = 1;
    let fiber_count = polynomial.coefficients().len().checked_div(target_count)?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(fiber_count).ok()?;
    for fiber in polynomial.coefficients().chunks_exact(target_count) {
        coefficients.push(Real::eval_poly(fiber, parameter));
    }
    DenseTensorPolynomial::try_new(dimensions, coefficients)
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
