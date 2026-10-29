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
