//! Optional native witnesses retain the original selected-field authority.

use super::*;
use crate::{
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
};

pub(super) fn represented(value: &SelectedPolynomialValue) -> Option<Real> {
    match &value.0.coordinates {
        Coordinates::Recursive(scalar) => (scalar
            .numerator
            .exact_real_value_with_retained_witnesses()?
            / scalar
                .denominator
                .exact_real_value_with_retained_witnesses()?)
        .ok(),
        Coordinates::Extension {
            numerator,
            denominator,
        } => {
            let ([numerator], [denominator]) = (numerator.as_slice(), denominator.as_slice())
            else {
                return None;
            };
            (represented(numerator)? / represented(denominator)?).ok()
        }
    }
}

pub(super) fn parent_image<C: SelectedAlgebraContext>(
    parent: &SelectedPolynomialField,
    defining: &[SelectedPolynomialValue],
    monic: &[SelectedPolynomialValue],
    lower: &Real,
    upper: &Real,
    policy: &C,
) -> Result<Option<SelectedPolynomialValue>, SelectedPolynomialError<C::Error>> {
    if !(2..=3).contains(&monic.len()) {
        return Ok(None);
    }
    let Some(coefficients) = monic.iter().map(represented).collect::<Option<Vec<_>>>() else {
        return Ok(None);
    };
    let mut source = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: crate::SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: coefficients,
        interval: IsolatedRootInterval {
            lower: lower.clone(),
            upper: upper.clone(),
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Undecided,
            message: None,
        },
    };
    source.validation =
        crate::validate_algebraic_root_representation(&source, hyperlimit::PredicatePolicy::STRICT);
    let Some(compact) = crate::compact_algebraic_root_low_degree_witness(&source) else {
        return Ok(None);
    };
    let Some(value) = compact.exact_point_witness() else {
        return Ok(None);
    };
    if policy.real_sign(&(value - lower)) != Some(RealSign::Positive)
        || policy.real_sign(&(upper - value)) != Some(RealSign::Positive)
    {
        return Ok(None);
    }
    // Recognition is optional. The consumed proof always checks the original
    // field equation and original open selector, retaining every parent source.
    let attempt = || {
        Ok(
            (evaluate(defining, value, parent)?.sign_strict(policy)? == RealSign::Zero)
                .then(|| parent.constant(value.clone()))
                .transpose()?,
        )
    };
    match attempt() {
        Ok(image) => Ok(image),
        Err(SelectedPolynomialError::Arithmetic(
            SelectedPolynomialArithmeticError::ConstructionUnavailable,
        )) => Ok(None),
        Err(SelectedPolynomialError::Uncertain(reason))
            if reason != UncertaintyReason::BudgetExhausted =>
        {
            Ok(None)
        }
        Err(error) => Err(error),
    }
}
