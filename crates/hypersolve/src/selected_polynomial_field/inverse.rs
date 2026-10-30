//! Optional Bezout inverses over the retained selected coefficient field.

use super::*;

fn trim<C: SelectedAlgebraContext>(
    polynomial: &mut Vec<SelectedPolynomialValue>,
    policy: &C,
) -> Result<(), SelectedPolynomialError<C::Error>> {
    while let Some(leading) = polynomial.last() {
        if leading.sign_strict(policy)? != RealSign::Zero {
            break;
        }
        polynomial.pop();
    }
    Ok(())
}

// `divisor` is monic by the caller's exact reciprocal proof. Removing the
// highest coefficient is therefore an exact cancellation, not a zero guess.
fn monic_div_rem(
    mut polynomial: Vec<SelectedPolynomialValue>,
    divisor: &[SelectedPolynomialValue],
    parent: &SelectedPolynomialField,
) -> ArithmeticResult<(Vec<SelectedPolynomialValue>, Vec<SelectedPolynomialValue>)> {
    let degree = divisor.len() - 1;
    let count = polynomial.len().saturating_sub(degree).max(1);
    let mut quotient = vec![parent.constant(Real::zero())?; count];
    while polynomial.len() > degree {
        let leading = polynomial
            .pop()
            .expect("a leading coefficient is eliminated");
        let shift = polynomial.len() - degree;
        quotient[shift] = leading.clone();
        if leading.stored_zero() {
            continue;
        }
        for (index, coefficient) in divisor[..degree].iter().enumerate() {
            polynomial[shift + index] =
                polynomial[shift + index].subtract(&leading.multiply(coefficient)?)?;
        }
    }
    Ok((quotient, polynomial))
}

pub(super) fn reciprocal<C: SelectedAlgebraContext>(
    value: &SelectedPolynomialValue,
    policy: &C,
) -> Result<Option<SelectedPolynomialValue>, SelectedPolynomialError<C::Error>> {
    let FieldData::Extension { parent, monic, .. } = value.field().0.as_ref() else {
        return Ok(None);
    };
    let Coordinates::Extension {
        numerator,
        denominator,
    } = &value.0.coordinates
    else {
        return Ok(None);
    };
    // Constant numerators still need their parent-field inverse. In particular,
    // a proved parent image can leave a selected field with a linear relation;
    // keeping projective denominators there would undo normalization below it.
    let one = parent.constant(Real::one())?;
    let mut previous = monic.clone();
    let mut current = numerator.clone();
    let mut previous_coefficient = vec![parent.constant(Real::zero())?];
    let mut current_coefficient = vec![one.clone()];
    loop {
        trim(&mut current, policy)?;
        let Some(leading) = current.last() else {
            // A nonconstant gcd need not vanish at the selected real root.
            // Decline global inversion and keep its original local quotient.
            return Ok(None);
        };
        let inverse = leading.reciprocal_strict(policy)?;
        for coefficient in current.iter_mut().chain(&mut current_coefficient) {
            *coefficient = coefficient.multiply(&inverse)?;
        }
        // Both the remainder and its Bezout coefficient received the same
        // nonzero factor. Their relation modulo the original polynomial holds.
        *current.last_mut().expect("the row is nonzero") = one.clone();
        if current.len() == 1 {
            let inverse = value.field().reduce(current_coefficient)?;
            let product = multiply(denominator, &inverse, parent)?;
            return value
                .field()
                .extension_value(product, vec![one])
                .map(Some)
                .map_err(Into::into);
        }
        let (quotient, remainder) = monic_div_rem(previous, &current, parent)?;
        let product = multiply(&quotient, &current_coefficient, parent)?;
        let negative = product
            .iter()
            .map(|coefficient| coefficient.scale(&Real::from(-1)))
            .collect::<ArithmeticResult<Vec<_>>>()?;
        let next = value
            .field()
            .reduce(add(&previous_coefficient, &negative, parent)?)?;
        previous = current;
        current = remainder;
        previous_coefficient = current_coefficient;
        current_coefficient = next;
    }
}
