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
