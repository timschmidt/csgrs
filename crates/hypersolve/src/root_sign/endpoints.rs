//! Root counts with endpoints retained in the same exact coefficient field.

use super::*;

pub(super) enum FieldEndpoint<'a, C> {
    Real(&'a Real),
    Value(&'a C),
}

impl<C: Clone> FieldEndpoint<'_, C> {
    pub(super) fn sign_at<F: OrderedFieldPolynomialContext<C>>(
        &self,
        polynomial: &[C],
        field: &mut F,
    ) -> Result<Ordering, F::Error> {
        match self {
            Self::Real(point) => field_polynomial_sign(polynomial, point, field),
            Self::Value(point) => ordered_field_polynomial_sign_at_value(polynomial, point, field),
        }
    }
}

/// Signs an ascending-coefficient polynomial at a retained field value.
/// The context owns coefficient/argument compatibility and exact sign decisions.
/// This evaluates the original polynomial; it makes no root-selection claim.
pub fn ordered_field_polynomial_sign_at_value<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    polynomial: &[C],
    point: &C,
    field: &mut F,
) -> Result<Ordering, F::Error> {
    let Some((leading, remaining)) = polynomial.split_last() else {
        return Ok(Ordering::Equal);
    };
    let mut value = leading.clone();
    for coefficient in remaining.iter().rev() {
        value = field.multiply(&value, point)?;
        value = field.add(&value, coefficient)?;
    }
    field.sign(&value)
}

/// Counts distinct roots in an open interval whose endpoints are exact values
/// in the coefficient field. No independent represented coordinates are needed.
///
/// The shared signed-remainder authority, positive-scale normalization and
/// bounded-remainder refusal are identical to [`ordered_field_count_roots`].
/// Reversed/zero-width intervals, endpoint roots, the zero polynomial and an
/// unavailable bounded chain return `None`. Nonzero constants return `Some(0)`.
/// Field incompatibility and undecided signs retain the context's error type.
pub fn ordered_field_count_roots_between_values<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    defining: &[C],
    lower: &C,
    upper: &C,
    field: &mut F,
) -> Result<Option<usize>, F::Error> {
    let negative_lower = field.scale(lower, &Real::from(-1))?;
    let width = field.add(upper, &negative_lower)?;
    if field.sign(&width)? != Ordering::Greater {
        return Ok(None);
    }
    count_roots_at_endpoints(
        defining,
        [FieldEndpoint::Value(lower), FieldEndpoint::Value(upper)],
        field,
    )
}
