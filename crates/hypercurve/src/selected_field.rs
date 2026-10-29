//! Adapters to Hypersolve's selected-field authority; no coordinate projection.

use crate::{Classification, CurveContext, CurveError, CurveResult, Real, UncertaintyReason};
pub(crate) use hypersolve::selected_polynomial_field::{
    SelectedPolynomialEnclosure as Enclosure, SelectedPolynomialError as FieldError,
    SelectedPolynomialField as Field, SelectedPolynomialValue as Value,
};
use hypersolve::{BivariatePolynomial, RealInterval};

pub(crate) type Result<T> = std::result::Result<T, FieldError<CurveError>>;

pub(crate) fn decided<T>(result: Classification<T>) -> Result<T> {
    match result {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(FieldError::Uncertain(reason)),
    }
}

pub(crate) fn curve<T>(result: CurveResult<Classification<T>>) -> Result<T> {
    decided(result.map_err(FieldError::Context)?)
}

pub(crate) fn finish<T>(operation: impl FnOnce() -> Result<T>) -> CurveResult<Classification<T>> {
    let result = operation();
    if hypersolve::work_budget::exhaustion().is_some() {
        return Ok(Classification::Uncertain(
            UncertaintyReason::BudgetExhausted,
        ));
    }
    match result {
        Ok(value) => Ok(Classification::Decided(value)),
        Err(FieldError::Context(error)) => Err(error),
        Err(FieldError::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(FieldError::Arithmetic(_)) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Err(FieldError::ZeroDenominator) => Err(CurveError::ZeroRationalPointDenominator),
        Err(
            FieldError::InvalidSelector
            | FieldError::InvalidEnclosure
            | FieldError::ZeroPolynomial
            | FieldError::NoRoot
            | FieldError::MultipleRoots(_),
        ) => Err(CurveError::InvalidBezierAlgebraicParameter),
    }
}

pub(crate) fn available<T>(value: Option<T>) -> Result<T> {
    value.ok_or(FieldError::Uncertain(UncertaintyReason::Unsupported))
}

pub(crate) fn polynomial(coefficients: &[Real], value: &Value) -> Result<Value> {
    let field = value.field();
    let mut sum = field.constant(Real::zero())?;
    for coefficient in coefficients.iter().rev() {
        sum = sum
            .multiply(value)?
            .add(&field.constant(coefficient.clone())?)?;
    }
    Ok(sum)
}

pub(crate) fn bivariate(polynomial: &BivariatePolynomial, point: &[Value; 2]) -> Result<Value> {
    let mut sum = point[0].field().constant(Real::zero())?;
    for row in polynomial.coefficients.iter().rev() {
        sum = sum
            .multiply(&point[0])?
            .add(&self::polynomial(row, &point[1])?)?;
    }
    Ok(sum)
}

pub(crate) fn query(
    polynomials: [&BivariatePolynomial; 2],
    point: &[Value; 2],
    policy: &CurveContext,
) -> Result<Value> {
    let denominator = bivariate(polynomials[1], point)?;
    // Division checks the original denominator even when the numerator is zero.
    decided(bivariate(polynomials[0], point)?.divide(&denominator, policy)?)
}

pub(crate) fn joined(first: &Value, second: &Value, policy: &CurveContext) -> Result<[Value; 2]> {
    let join = decided(first.field().joined_with(second.field(), policy)?)?;
    Ok([join.embed(first)?, join.embed(second)?])
}

pub(crate) fn bounds(value: &Value, steps: usize, policy: &CurveContext) -> Result<RealInterval> {
    Ok(
        decided(decided(value.enclosed(policy)?)?.refined(steps, policy)?)?
            .interval()
            .clone(),
    )
}
