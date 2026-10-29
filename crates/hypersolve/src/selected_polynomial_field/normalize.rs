//! Positive common scaling for temporary ordered-field polynomials.

use super::*;

// Every denominator here is positive at the selected tuple, by the private
// value constructors. Clearing them multiplies the whole polynomial by one
// positive factor. Recursing through the parent coefficients removes another
// common positive factor; no coefficient is independently rescaled.
pub(super) fn positive_scale(
    field: &SelectedPolynomialField,
    values: &[SelectedPolynomialValue],
) -> ArithmeticResult<Vec<SelectedPolynomialValue>> {
    let values = values
        .iter()
        .map(|value| field.lift(value))
        .collect::<ArithmeticResult<Vec<_>>>()?;
    match field.0.as_ref() {
        FieldData::Recursive(native) => {
            let one = unavailable(native.constant(Real::one()))?;
            let mut parts = Vec::with_capacity(values.len());
            for value in &values {
                let Coordinates::Recursive(scalar) = &value.0.coordinates else {
                    return Err(SelectedPolynomialArithmeticError::IncompatibleField);
                };
                parts.push((scalar.numerator.clone(), scalar.denominator.clone()));
            }
            let mut numerators =
                clear_denominators(parts, one.clone(), |a, b| unavailable(a.multiply(b)))?;
            RecursiveQuadraticValue::normalize_positive_scale(&mut numerators);
            Ok(numerators
                .into_iter()
                .map(|numerator| {
                    SelectedPolynomialValue(Arc::new(ValueData {
                        field: field.clone(),
                        coordinates: Coordinates::Recursive(RecursiveQuadraticProjectiveScalar {
                            numerator,
                            denominator: one.clone(),
                        }),
                    }))
                })
                .collect())
        }
        FieldData::Extension { parent, .. } => {
            let one = vec![parent.constant(Real::one())?];
            let mut parts = Vec::with_capacity(values.len());
            for value in &values {
                let Coordinates::Extension {
                    numerator,
                    denominator,
                } = &value.0.coordinates
                else {
                    return Err(SelectedPolynomialArithmeticError::IncompatibleField);
                };
                parts.push((numerator.clone(), denominator.clone()));
            }
            let numerators = clear_denominators(parts, one.clone(), |a, b| {
                if matches!(a.as_slice(), [value] if value.stored_one()) {
                    return Ok(b.clone());
                }
                if matches!(b.as_slice(), [value] if value.stored_one()) {
                    return Ok(a.clone());
                }
                field.reduce(multiply(a, b, parent)?)
            })?;
            let lengths: Vec<_> = numerators.iter().map(Vec::len).collect();
            let coefficients: Vec<_> = numerators.into_iter().flatten().collect();
            let normalized = positive_scale(parent, &coefficients)?;
            let mut offset = 0;
            Ok(lengths
                .into_iter()
                .map(|length| {
                    let numerator = normalized[offset..offset + length].to_vec();
                    offset += length;
                    SelectedPolynomialValue(Arc::new(ValueData {
                        field: field.clone(),
                        coordinates: Coordinates::Extension {
                            numerator,
                            denominator: one.clone(),
                        },
                    }))
                })
                .collect())
        }
    }
}

// Prefix/suffix products avoid rebuilding every other-denominator product for
// each coefficient. The products are reduced in the same admitted field.
fn clear_denominators<T: Clone>(
    parts: Vec<(T, T)>,
    one: T,
    multiply: impl Fn(&T, &T) -> ArithmeticResult<T>,
) -> ArithmeticResult<Vec<T>> {
    let mut prefixes = Vec::with_capacity(parts.len());
    let mut product = one.clone();
    for (_, denominator) in &parts {
        prefixes.push(product.clone());
        product = multiply(&product, denominator)?;
    }
    let mut suffix = one;
    let mut result = Vec::with_capacity(parts.len());
    for ((numerator, denominator), prefix) in parts.into_iter().zip(prefixes).rev() {
        result.push(multiply(&multiply(&numerator, &prefix)?, &suffix)?);
        suffix = multiply(&denominator, &suffix)?;
    }
    result.reverse();
    Ok(result)
}
