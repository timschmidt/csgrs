//! Exact Cartesian width measurements and deterministic widest-coordinate selection.
//!
//! Scalar width carriers preserve endpoint differences independently of projection
//! back into the interval family. These queries schedule or measure a box; they do
//! not choose a representative point or establish a solver exclusion/existence proof.

use crate::{IntervalArithmetic, IntervalLatticeError, IntervalVector, array_result};
use core::cmp::Ordering;

/// Width-array construction and a streaming maximum over validated nonempty lanes.
impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Exact coordinate widths, without narrowing to signed/float endpoints.
    ///
    /// Fixed widths retain their binary scale and full unsigned numerator;
    /// float widths retain an exact endpoint difference. Rational/dyadic widths
    /// are singleton intervals. Empty returns `None`, whereas the zero-dimensional
    /// unit product returns `Some([])`. No rounded enclosure replaces a width.
    /// The context covers all scalar work; failure returns no partial array.
    ///
    /// Visits lanes in increasing index order, stopping at the first scalar error.
    /// Nonempty carrier lanes guarantee that scalar width queries return Some;
    /// the internal expect asserts that constructor invariant. Empty and unit
    /// products perform no scalar work. Owned widths use an inline N-element array,
    /// with backend-dependent scalar storage; already-charged work is not refunded.
    ///
    /// ```
    /// # #[cfg(feature = "fixed")] {
    /// use hyperlattice_interval::{IntervalVector, hyperinterval::FixedInterval};
    /// type I = FixedInterval<i64, 63>;
    /// let box_ = IntervalVector::new([I::new(i64::MIN, i64::MAX).unwrap(), I::point(0).unwrap()]).unwrap();
    /// assert_eq!(box_.widths(&mut ()).unwrap().unwrap()[0].numerator(), u64::MAX);
    /// assert_eq!(box_.widest_axis(&mut ()).unwrap(), Some(0));
    /// # }
    /// ```
    pub fn widths(
        &self,
        context: &mut I::Context,
    ) -> Result<Option<[I::Width; N]>, IntervalLatticeError> {
        match self.lanes() {
            None => Ok(None),
            Some(lanes) => Ok(Some(array_result(|index| {
                lanes[index]
                    .width(context)
                    .map(|value| value.expect("nonempty coordinate"))
            })?)),
        }
    }

    /// Index of a maximum exact coordinate width, breaking ties by lowest index.
    ///
    /// Empty and zero-dimensional boxes return `None`. A nonempty box whose
    /// widths are all zero selects index zero. Successful evaluation constructs each
    /// width once and retains the current maximum alongside the candidate being
    /// compared. All dyadic work shares the context.
    /// Exhaustion publishes no provisional axis and does not mutate the source.
    /// This schedules a coordinate; it does not certify a solver decision.
    ///
    /// Scans increasing indices. The first width becomes the maximum without a
    /// comparison; later candidates replace it only on exact Greater, preserving
    /// the first index of a tie. Width construction precedes comparison, and the
    /// first failure aborts the query with prior budget charges intact. The number
    /// of retained widths is constant, but their scalar size/work need not be.
    pub fn widest_axis(
        &self,
        context: &mut I::Context,
    ) -> Result<Option<usize>, IntervalLatticeError> {
        let Some(lanes) = self.lanes() else {
            return Ok(None);
        };
        let mut widest: Option<(usize, I::Width)> = None;
        for (index, lane) in lanes.iter().enumerate() {
            let width = lane.width(context)?.expect("nonempty coordinate");
            if let Some((_, maximum)) = &widest
                && I::compare_widths(&width, maximum, context)? != Ordering::Greater
            {
                continue;
            }
            widest = Some((index, width));
        }
        Ok(widest.map(|(index, _)| index))
    }
}
