use crate::{IntervalArithmetic, IntervalLatticeError, IntervalVector, array_result};
use core::cmp::Ordering;

impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Exact coordinate widths, without narrowing to signed/float endpoints.
    ///
    /// Fixed widths retain their binary scale and full unsigned numerator;
    /// float widths retain an exact endpoint difference. Rational/dyadic widths
    /// are singleton intervals. Empty returns `None`, whereas the zero-dimensional
    /// unit product returns `Some([])`. No rounded enclosure replaces a width.
    /// The context covers all scalar work; failure returns no partial array.
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
    /// widths are all zero selects index zero. Each width is constructed once;
    /// only the current maximum is retained. All dyadic work shares the context.
    /// Exhaustion publishes no provisional axis and does not mutate the source.
    /// This schedules a coordinate; it does not certify a solver decision.
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
