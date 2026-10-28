//! Polynomial evaluation in a shared projective selected field.

use super::*;

/// Evaluates `w^degree * P(x/w, y/w)` using only field operations.
///
/// The caller chooses a common homogenizing degree for multiple polynomials.
/// `None` means that degree is too small, fields cannot compose, or checked
/// arithmetic/storage is unavailable. This operation asserts no sign of `w`
/// and constructs no independently selected coordinate roots.
pub fn evaluate_bivariate_homogeneous_polynomial(
    polynomial: &BivariatePolynomial,
    x: &RecursiveQuadraticValue,
    y: &RecursiveQuadraticValue,
    w: &RecursiveQuadraticValue,
    degree: usize,
) -> Option<RecursiveQuadraticValue> {
    let powers = |value: &RecursiveQuadraticValue| {
        let mut powers = Vec::new();
        powers.try_reserve_exact(degree.checked_add(1)?).ok()?;
        powers.push(value.field().constant(Real::one())?);
        for i in 0..degree {
            powers.push(powers[i].multiply(value)?);
        }
        Some(powers)
    };
    let xs = powers(x)?;
    let ys = powers(y)?;
    let ws = powers(w)?;
    let mut result = w.field().constant(Real::zero())?;
    for (i, row) in polynomial.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            if coefficient.definitely_zero() {
                continue;
            }
            let k = degree.checked_sub(i.checked_add(j)?)?;
            let term = xs
                .get(i)?
                .multiply(ys.get(j)?)?
                .multiply(&ws[k])?
                .scale(coefficient)?;
            result = result.add(&term)?;
        }
    }
    Some(result)
}

/// Substitutes polynomial homogeneous coordinates into a bivariate equation.
///
/// The returned ascending coefficients represent
/// `W(t)^degree * P(X(t)/W(t), Y(t)/W(t))`. Every coefficient retains the same
/// selected field as the input coordinates; no global algebraic projection or
/// independently selected coordinate is constructed. `degree` may exceed the
/// active total degree of `P` when several equations need a common degree.
///
/// All three coordinate polynomials must be nonempty and use one field.
/// `None` reports incompatible fields, insufficient degree, or unavailable
/// checked arithmetic/storage. This is polynomial construction only: it does
/// not prove `W` nonzero, cancel homogeneous factors, extend a source domain,
/// or select a real branch of the equation. Those guards remain with callers.
pub fn compose_bivariate_homogeneous_polynomial(
    polynomial: &BivariatePolynomial,
    x: &[RecursiveQuadraticValue],
    y: &[RecursiveQuadraticValue],
    w: &[RecursiveQuadraticValue],
    degree: usize,
) -> Option<Vec<RecursiveQuadraticValue>> {
    let count = degree.checked_add(1)?;
    let field = w.first()?.field();
    if x.is_empty()
        || y.is_empty()
        || x.iter()
            .chain(y)
            .chain(w)
            .any(|value| !field.same_field(&value.field()))
    {
        return None;
    }
    for (i, row) in polynomial.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            if !coefficient.definitely_zero() && i.checked_add(j)? > degree {
                return None;
            }
        }
    }
    let powers = |coordinate: &[RecursiveQuadraticValue]| {
        let mut powers = Vec::new();
        powers.try_reserve_exact(count).ok()?;
        powers.push(vec![field.constant(Real::one())?]);
        for i in 0..degree {
            powers.push(recursive_quadratic_polynomial_multiply(
                &powers[i], coordinate,
            )?);
        }
        Some(powers)
    };
    let xs = powers(x)?;
    let ys = powers(y)?;
    let ws = powers(w)?;
    let mut result = vec![field.constant(Real::zero())?];
    for (i, row) in polynomial.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            if coefficient.definitely_zero() {
                continue;
            }
            let k = degree.checked_sub(i.checked_add(j)?)?;
            let product = recursive_quadratic_polynomial_multiply(xs.get(i)?, ys.get(j)?)?;
            let product = recursive_quadratic_polynomial_multiply(&product, &ws[k])?;
            let term = product
                .iter()
                .map(|value| value.scale(coefficient))
                .collect::<Option<Vec<_>>>()?;
            result = recursive_quadratic_polynomial_combine(&result, &term, false)?;
        }
    }
    Some(result)
}
