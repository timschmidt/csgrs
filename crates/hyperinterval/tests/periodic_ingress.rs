//! Stored rational widths and delayed reductions at periodic interval ingress.
#![cfg(all(feature = "dyadic", feature = "rational"))]

use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, IntervalSet2,
    RangeEnclosure, RationalInterval,
};
use hyperreal::{ExactDyadicLine2, Rational};

fn coordinate(span: f64, y: f64) -> Rational {
    let line = ExactDyadicLine2::from_f64([0., 0.], [span, span]).unwrap();
    let (_, point) = line.intersection_point_f64([0., y], [span, y]).unwrap();
    point[0].exact_rational_ref().unwrap().clone()
}

fn without_dispatch<T>(evaluate: impl FnOnce() -> T) -> T {
    #[cfg(feature = "dispatch-trace")]
    {
        use hyperreal::dispatch_trace;
        dispatch_trace::reset();
        let result = dispatch_trace::with_recording(evaluate);
        assert!(dispatch_trace::take_trace().dispatch.is_empty());
        result
    }
    #[cfg(not(feature = "dispatch-trace"))]
    evaluate()
}

#[derive(Clone, Copy, Debug)]
enum Function {
    Sin,
    Cos,
    Tan,
    Cot,
}

impl Function {
    fn image(
        self,
        input: &RationalInterval,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<IntervalSet2<DyadicInterval>>, IntervalError> {
        let image = match self {
            Self::Sin => input.sin(-8, budget)?,
            Self::Cos => input.cos(-8, budget)?,
            Self::Tan => return input.tan(-8, budget),
            Self::Cot => return input.cot(-8, budget),
        };
        Ok(RangeEnclosure {
            range: IntervalSet2::from_interval(ExtendedInterval::from_finite(image.range)),
            domain: image.domain,
        })
    }
}

#[test]
fn stored_widths_and_work_are_checked_before_either_source_is_reduced() {
    use Function::*;
    for op in [Sin, Cos, Tan, Cot] {
        for wider_on_left in [false, true] {
            // A construction with a large common factor represents exactly 3.
            let x = coordinate(281_474_976_710_662., 3.);
            let (_, numerator, denominator) = x.ratio_parts();
            let width = numerator.bits().max(denominator.bits());
            assert!(width > 64);
            assert_eq!(x.reduction_input_bits(), width);
            assert_eq!(Rational::fraction(-3, 7).unwrap().reduction_input_bits(), 0);
            let input = if wider_on_left {
                RationalInterval::new(x, Rational::from(4))
            } else {
                RationalInterval::new(Rational::from(2), x)
            }
            .unwrap();
            for (limit, units, error) in [
                (0, 100, IntervalError::SizeExhausted),
                (width - 1, 1_000_000, IntervalError::SizeExhausted),
                (width, 4 * width - 1, IntervalError::WorkExhausted),
            ] {
                let mut budget = DyadicBudget::new(limit, units);
                let result = without_dispatch(|| op.image(&input, &mut budget));
                assert_eq!(result, Err(error), "{op:?}, wider on left {wider_on_left}");
                assert_eq!(budget.remaining_work(), units);
            }
        }
    }
}

#[test]
fn equivalent_stored_sources_preserve_each_certified_range_and_retry() {
    use Function::*;
    // Directed 512-bit MPFR gives these outward cells on the 2^-8 grid.
    // All four functions are monotone on [2,3] with no poles there.
    for (op, point_cell, range_cell) in [
        (Sin, (36, 37), (36, 233)),
        (Cos, (-254, -253), (-254, -106)),
        (Tan, (-37, -36), (-560, -36)),
        (Cot, (-1796, -1795), (-1796, -117)),
    ] {
        for (left, right, (lo, hi)) in [
            (coordinate(6., 3.), coordinate(10., 3.), point_cell),
            (Rational::from(3), coordinate(6., 3.), point_cell),
            (coordinate(10., 2.), coordinate(6., 3.), range_cell),
        ] {
            let input = RationalInterval::new(left, right).unwrap();
            let mut budget = DyadicBudget::new(4096, 1 << 32);
            let start = budget.remaining_work();
            let image = op.image(&input, &mut budget).unwrap();
            assert_eq!(image.domain, DomainCoverage::Complete);
            assert_eq!(
                image.range.components(),
                &[ExtendedInterval::from_finite(
                    DyadicInterval::new(lo.into(), hi.into(), -8).unwrap()
                )]
            );
            let cost = start - budget.remaining_work();
            assert_eq!(
                op.image(&input, &mut DyadicBudget::new(4096, cost - 1)),
                Err(IntervalError::WorkExhausted)
            );
            let mut retry = DyadicBudget::new(4096, cost);
            assert_eq!(op.image(&input, &mut retry).unwrap(), image);
            assert_eq!(retry.remaining_work(), 0);
        }
    }
}

#[test]
fn reduction_of_the_left_endpoint_waits_for_the_right_endpoint_reservation() {
    use Function::*;
    for op in [Sin, Cos, Tan, Cot] {
        let left = coordinate(6., 2.);
        let right = coordinate(10., 3.);
        let width = |x: &Rational| {
            let (_, n, d) = x.ratio_parts();
            n.bits().max(d.bits())
        };
        let a = width(&left);
        let b = width(&right);
        let copies = 4 * a.max(b);
        let check_cache = right.clone();
        let input = RationalInterval::new(left, right).unwrap();
        // Each endpoint has a possible GCD and two exact divisions.
        for (units, remaining) in [
            (copies + 3 * a * a - 1, 3 * a * a - 1),
            (copies + 3 * a * a + 3 * b * b - 1, 3 * b * b - 1),
        ] {
            let mut budget = DyadicBudget::new(4096, units);
            let result = without_dispatch(|| op.image(&input, &mut budget));
            assert_eq!(result, Err(IntervalError::WorkExhausted));
            assert_eq!(budget.remaining_work(), remaining);
        }
        let evaluate = || {
            op.image(&input, &mut DyadicBudget::new(4096, 1 << 32))
                .unwrap()
        };
        #[cfg(feature = "dispatch-trace")]
        let cold = {
            use hyperreal::dispatch_trace;
            dispatch_trace::reset();
            let result = dispatch_trace::with_recording(evaluate);
            let trace = dispatch_trace::take_trace();
            assert_eq!(
                trace.path_count("rational", "canonicalization", "lazy-internal-coordinate"),
                2
            );
            result
        };
        #[cfg(not(feature = "dispatch-trace"))]
        let cold = evaluate();
        assert_eq!(without_dispatch(evaluate), cold);
        assert_eq!(check_cache.numerator().bits(), 2);
        assert_eq!(check_cache.reduction_input_bits(), b);
    }
}
