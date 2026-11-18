//! Exact separation of closed dyadic sets under a caller-owned budget.

use crate::{
    DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, IntervalSet2, dyadic_order,
};
use num_bigint::BigInt;

#[derive(PartialEq, Eq)]
enum Relation {
    Before,
    Intersects,
    After,
    EmptyLeft,
    EmptyRight,
}

#[inline]
fn before(
    (upper, e): (&BigInt, i32),
    (lower, f): (&BigInt, i32),
    budget: &mut DyadicBudget,
) -> Result<bool, IntervalError> {
    Ok(dyadic_order::compare(upper, e, lower, f, budget)?.is_lt())
}

#[inline]
fn finite_relation(
    left: &DyadicInterval,
    right: &DyadicInterval,
    budget: &mut DyadicBudget,
) -> Result<Relation, IntervalError> {
    let Some((a, b, e)) = left.bounds() else {
        return Ok(Relation::EmptyLeft);
    };
    let Some((c, d, f)) = right.bounds() else {
        return Ok(Relation::EmptyRight);
    };
    Ok(if before((b, e), (c, f), budget)? {
        Relation::Before
    } else if before((d, f), (a, e), budget)? {
        Relation::After
    } else {
        Relation::Intersects
    })
}

#[inline]
fn relation(
    left: &ExtendedInterval<DyadicInterval>,
    right: &ExtendedInterval<DyadicInterval>,
    budget: &mut DyadicBudget,
) -> Result<Relation, IntervalError> {
    use super::State::{Above, Below, Empty, Entire, Finite};
    let (upper, lower, separated) = match (&left.state, &right.state) {
        (Empty, _) => return Ok(Relation::EmptyLeft),
        (_, Empty) => return Ok(Relation::EmptyRight),
        (Entire, _) | (_, Entire) | (Below(_), Below(_)) | (Above(_), Above(_)) => {
            return Ok(Relation::Intersects);
        }
        (Finite(a), Finite(b)) => return finite_relation(a, b, budget),
        (Below(a), Finite(b) | Above(b)) | (Finite(a), Above(b)) => (a, b, Relation::Before),
        (Finite(b) | Above(b), Below(a)) | (Above(b), Finite(a)) => (a, b, Relation::After),
    };
    let (Some(end), Some(start)) = (upper.upper(), lower.lower()) else {
        return Err(IntervalError::EmptyDomain);
    };
    Ok(if before(end, start, budget)? {
        separated
    } else {
        Relation::Intersects
    })
}

impl DyadicInterval {
    /// Whether the closed intervals share a real point, including an endpoint.
    /// Empty intervals never overlap. Comparisons share the supplied budget.
    #[inline]
    pub fn overlaps(&self, other: &Self, budget: &mut DyadicBudget) -> Result<bool, IntervalError> {
        Ok(finite_relation(self, other, budget)? == Relation::Intersects)
    }

    /// Whether the closed intervals have empty intersection.
    #[inline]
    pub fn disjoint(&self, other: &Self, budget: &mut DyadicBudget) -> Result<bool, IntervalError> {
        Ok(!self.overlaps(other, budget)?)
    }
}

impl ExtendedInterval<DyadicInterval> {
    /// Whether the closed intervals share a real point. Ray endpoints are
    /// included. Rays pointing in the same direction always overlap.
    #[inline]
    pub fn overlaps(&self, other: &Self, budget: &mut DyadicBudget) -> Result<bool, IntervalError> {
        Ok(relation(self, other, budget)? == Relation::Intersects)
    }

    /// Whether the two real sets have empty intersection.
    #[inline]
    pub fn disjoint(&self, other: &Self, budget: &mut DyadicBudget) -> Result<bool, IntervalError> {
        Ok(!self.overlaps(other, budget)?)
    }
}

impl IntervalSet2<DyadicInterval> {
    /// Whether any closed components share a point. Gaps stay excluded.
    /// At most three component pairs share the supplied budget.
    #[inline]
    pub fn overlaps(&self, other: &Self, budget: &mut DyadicBudget) -> Result<bool, IntervalError> {
        if self.is_empty() {
            return Ok(false);
        }
        let mut right = other.components();
        for a in self.components() {
            while let Some((b, tail)) = right.split_first() {
                match relation(a, b, budget)? {
                    Relation::Before | Relation::EmptyLeft => break,
                    Relation::After | Relation::EmptyRight => right = tail,
                    Relation::Intersects => return Ok(true),
                }
            }
        }
        Ok(false)
    }

    /// Whether the two unions have empty intersection.
    #[inline]
    pub fn disjoint(&self, other: &Self, budget: &mut DyadicBudget) -> Result<bool, IntervalError> {
        Ok(!self.overlaps(other, budget)?)
    }
}
