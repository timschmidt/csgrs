//! A conservative filter over retained selectors; exact values stay authoritative.

use super::*;
use crate::RealInterval;
use std::collections::HashMap;

type Memo = HashMap<*const ValueData, Option<RealInterval>>;

fn polynomial(
    coefficients: &[SelectedPolynomialValue],
    argument: &RealInterval,
    memo: &mut Memo,
) -> Option<RealInterval> {
    let mut value = RealInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    for coefficient in coefficients.iter().rev() {
        value = value.multiply(argument)?.add(&enclose(coefficient, memo)?);
    }
    Some(value)
}

fn enclose(value: &SelectedPolynomialValue, memo: &mut Memo) -> Option<RealInterval> {
    let key = Arc::as_ptr(&value.0);
    if let Some(bounds) = memo.get(&key) {
        return bounds.clone();
    }
    let bounds = match (value.field().0.as_ref(), &value.0.coordinates) {
        (_, Coordinates::Recursive(scalar)) => scalar.interval(0),
        (
            FieldData::Extension { interval, .. },
            Coordinates::Extension {
                numerator,
                denominator,
            },
        ) => {
            let argument = RealInterval {
                lower: interval.lower.clone(),
                upper: interval.upper.clone(),
            };
            polynomial(numerator, &argument, memo)?.divide(&polynomial(
                denominator,
                &argument,
                memo,
            )?)
        }
        _ => None,
    };
    memo.insert(key, bounds.clone());
    bounds
}

impl SelectedPolynomialValue {
    pub(super) fn has_rational_sign_coefficients(&self) -> bool {
        let (FieldData::Extension { defining, .. }, Coordinates::Extension { numerator, .. }) =
            (self.field().0.as_ref(), &self.0.coordinates)
        else {
            return false;
        };
        // The existing exact remainder chain already decides rational data
        // without root refinement. Preserve that path and its resource cost,
        // including coefficients lifted through otherwise selected fields.
        defining.iter().chain(numerator).all(|coefficient| {
            witness::represented(coefficient)
                .is_some_and(|value| value.exact_rational_ref().is_some())
        })
    }

    pub(super) fn sign_from_selection_bounds(&self) -> Option<RealSign> {
        let (FieldData::Extension { interval, .. }, Coordinates::Extension { numerator, .. }) =
            (self.field().0.as_ref(), &self.0.coordinates)
        else {
            return None;
        };
        // The top denominator is already certified positive. Parent values
        // still need full quotient bounds; a denominator box crossing zero
        // makes the filter unavailable rather than erasing a guard.
        let argument = RealInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        };
        // Every key borrows the unchanged graph owned by self for this call.
        // Allocation identity only reuses an enclosure of that same value.
        polynomial(numerator, &argument, &mut Memo::new())?.strict_nonzero_sign()
    }
}

// All memo keys borrow the immutable field/value graph owned by the query.
// They identify proof inputs for this call, never mathematical equality, and
// no failed interval attempt survives a retry under a larger resource budget.
#[derive(Default)]
struct RefinedFilter {
    values: HashMap<(*const ValueData, usize), Option<RealInterval>>,
    fields: HashMap<(*const FieldData, usize), Option<RealInterval>>,
}

fn interval_polynomial(
    coefficients: &[RealInterval],
    argument: &RealInterval,
) -> Option<RealInterval> {
    let mut result = RealInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    for coefficient in coefficients.iter().rev() {
        result = result.multiply(argument)?.add(coefficient);
    }
    Some(result)
}

impl RefinedFilter {
    fn coefficients(
        &mut self,
        values: &[SelectedPolynomialValue],
        steps: usize,
    ) -> Option<Vec<RealInterval>> {
        values
            .iter()
            .map(|value| self.value(value, steps))
            .collect()
    }

    fn field(&mut self, field: &SelectedPolynomialField, steps: usize) -> Option<RealInterval> {
        let key = (Arc::as_ptr(&field.0), steps);
        if let Some(bounds) = self.fields.get(&key) {
            return bounds.clone();
        }
        let FieldData::Extension {
            monic, interval, ..
        } = field.0.as_ref()
        else {
            return None;
        };
        let mut bounds = RealInterval {
            lower: interval.lower.clone(),
            upper: interval.upper.clone(),
        };
        let parent_steps = steps.saturating_mul(2).min(256);
        let coefficients = self.coefficients(monic, parent_steps)?;
        if let [constant, _] = coefficients.as_slice() {
            // The private monic relation is proved at the original selection.
            // A parent image can bound its generator without materializing it.
            bounds = RealInterval {
                lower: -constant.upper.clone(),
                upper: -constant.lower.clone(),
            };
        } else {
            let point = |value: &Real| RealInterval {
                lower: value.clone(),
                upper: value.clone(),
            };
            let lower_sign =
                interval_polynomial(&coefficients, &point(&bounds.lower))?.strict_nonzero_sign();
            let upper_sign =
                interval_polynomial(&coefficients, &point(&bounds.upper))?.strict_nonzero_sign();
            if matches!(
                (lower_sign, upper_sign),
                (Some(RealSign::Negative), Some(RealSign::Positive))
                    | (Some(RealSign::Positive), Some(RealSign::Negative))
            ) {
                for _ in 0..steps {
                    crate::work_budget::consume(crate::work_budget::WorkKind::RootRefinement, 1)
                        .ok()?;
                    let middle = Real::average_pair(&bounds.lower, &bounds.upper);
                    let value = interval_polynomial(&coefficients, &point(&middle))?;
                    if value.lower.definitely_zero() && value.upper.definitely_zero() {
                        bounds = point(&middle);
                        break;
                    }
                    match value.strict_nonzero_sign() {
                        Some(sign) if Some(sign) == lower_sign => bounds.lower = middle,
                        Some(sign) if Some(sign) == upper_sign => bounds.upper = middle,
                        _ => break,
                    }
                }
            }
        }
        self.fields.insert(key, Some(bounds.clone()));
        Some(bounds)
    }

    fn value(&mut self, value: &SelectedPolynomialValue, steps: usize) -> Option<RealInterval> {
        let key = (Arc::as_ptr(&value.0), steps);
        if let Some(bounds) = self.values.get(&key) {
            return bounds.clone();
        }
        let bounds = match &value.0.coordinates {
            Coordinates::Recursive(scalar) => scalar.interval(steps),
            Coordinates::Extension {
                numerator,
                denominator,
            } => {
                let argument = self.field(value.field(), steps)?;
                let parent_steps = steps.saturating_mul(2).min(256);
                interval_polynomial(&self.coefficients(numerator, parent_steps)?, &argument)?
                    .divide(&interval_polynomial(
                        &self.coefficients(denominator, parent_steps)?,
                        &argument,
                    )?)
            }
        };
        self.values.insert(key, bounds.clone());
        bounds
    }
}

impl SelectedPolynomialValue {
    pub(super) fn sign_from_refined_selection_bounds(&self, steps: usize) -> Option<RealSign> {
        let Coordinates::Extension { numerator, .. } = &self.0.coordinates else {
            return None;
        };
        let mut filter = RefinedFilter::default();
        let argument = filter.field(self.field(), steps)?;
        let coefficients = filter.coefficients(numerator, steps.saturating_mul(2).min(256))?;
        // Only a definite nonzero sign is consumed. Exact zeros and failed
        // separation retain the complete selected-root sign algorithm.
        interval_polynomial(&coefficients, &argument)?.strict_nonzero_sign()
    }
}
