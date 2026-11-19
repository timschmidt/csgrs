//! Exact rational operations admitted before Hyperreal normalization or arithmetic.
use crate::{DyadicBudget, IntervalError, RationalInterval};
use core::cmp::Ordering;
use hyperreal::Rational;

// A canonical accessor may run a GCD and two divisions on the stored parts.
// Admit both possible reductions before either accessor can warm its cache.
pub(crate) fn reserve_reductions(
    left: &Rational,
    right: &Rational,
    budget: &mut DyadicBudget,
) -> Result<(), IntervalError> {
    for source in [left, right] {
        reserve_reduction(source, budget)?;
    }
    Ok(())
}

pub(crate) fn reserve_reduction(
    source: &Rational,
    budget: &mut DyadicBudget,
) -> Result<(), IntervalError> {
    let bits = source.reduction_input_bits();
    if bits != 0 {
        budget.reserve(bits, 3, true)?;
    }
    Ok(())
}

/// A borrowed admission view sharing a native arithmetic counter.
///
/// Mathematical operations delegate to Hyperreal without changing its dispatch
/// order. Admission inspects stored, possibly unreduced ratio parts without
/// canonicalization. Let `s(x)` be the larger stored numerator/denominator bit
/// count. Comparison/multiplication reserve `n=s(a)+s(b)` coefficient bits;
/// addition/subtraction reserve one more carry bit. Comparison costs `4*n²`
/// units and arithmetic costs `16*n²`, with n clamped to one for charging.
/// Charges precede all delegated arithmetic and do not depend on cache warmth.
///
/// These are deterministic admission weights and input/result coefficient
/// envelopes, not a count of Hyperreal's GCD steps, CPU instructions, internal
/// temporary space or retained caches. Payload lifecycle, input construction and
/// cache work retain their existing exclusions. This view does not establish a
/// whole-work or total-heap bound for rational arithmetic.
#[derive(Debug)]
pub struct RationalArithmetic<'a> {
    budget: &'a mut DyadicBudget,
}

impl DyadicBudget {
    /// Borrow exact rational admission using this same remaining counter.
    /// No fresh budget is created; subsequent dyadic conversion/arithmetic shares it.
    pub fn rational(&mut self) -> RationalArithmetic<'_> {
        RationalArithmetic { budget: self }
    }
}

impl RationalArithmetic<'_> {
    fn admit(
        &mut self,
        left: &Rational,
        right: &Rational,
        carry: bool,
        stages: u64,
    ) -> Result<(), IntervalError> {
        let size = |value: &Rational| {
            let (_, numerator, denominator) = value.ratio_parts();
            numerator.bits().max(denominator.bits())
        };
        let bits = size(left)
            .saturating_add(size(right))
            .saturating_add(u64::from(carry));
        self.budget.reserve(bits, stages, true)
    }

    /// Exact total comparison after admission, with no rounded intermediate.
    pub fn compare(
        &mut self,
        left: &Rational,
        right: &Rational,
    ) -> Result<Ordering, IntervalError> {
        self.admit(left, right, false, 4)?;
        Ok(left
            .partial_cmp(right)
            .expect("finite rational total order"))
    }

    /// Exact sum through Hyperreal's existing arithmetic authority.
    pub fn add(&mut self, left: &Rational, right: &Rational) -> Result<Rational, IntervalError> {
        self.admit(left, right, true, 16)?;
        Ok(left + right)
    }

    /// Exact difference through Hyperreal's existing arithmetic authority.
    pub fn sub(&mut self, left: &Rational, right: &Rational) -> Result<Rational, IntervalError> {
        self.admit(left, right, true, 16)?;
        Ok(left - right)
    }

    /// Exact product through Hyperreal's existing arithmetic authority.
    pub fn mul(&mut self, left: &Rational, right: &Rational) -> Result<Rational, IntervalError> {
        self.admit(left, right, false, 16)?;
        Ok(left * right)
    }

    /// Construct an exact closed interval after one admitted endpoint comparison.
    /// Ordering is not rechecked through an unaccounted second comparison.
    pub fn interval(
        &mut self,
        lower: Rational,
        upper: Rational,
    ) -> Result<RationalInterval, IntervalError> {
        if self.compare(&lower, &upper)? == Ordering::Greater {
            return Err(IntervalError::ReversedEndpoints);
        }
        Ok(RationalInterval::ordered(lower, upper))
    }
}
