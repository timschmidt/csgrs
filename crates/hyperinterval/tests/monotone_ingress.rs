//! Generated rational endpoints at the monotone elementary function boundary.
#![cfg(all(feature = "rational", feature = "dyadic"))]

use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, RangeEnclosure,
    RationalInterval,
};
use hyperreal::Rational;
#[path = "support/rational_coordinate.rs"]
mod rational_coordinate;
use rational_coordinate::coordinate;

#[derive(Debug, Clone, Copy)]
enum Operation {
    Exp,
    Log,
    Atan,
}

impl Operation {
    fn image(
        self,
        input: &RationalInterval,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<ExtendedInterval<DyadicInterval>>, IntervalError> {
        let result = match self {
            Self::Exp => input.exp(-8, budget)?,
            Self::Log => return input.log(-8, budget),
            Self::Atan => input.atan(-8, budget)?,
        };
        Ok(RangeEnclosure {
            range: ExtendedInterval::from_finite(result.range),
            domain: result.domain,
        })
    }
}

#[test]
fn generated_endpoints_match_directed_cells_and_the_exact_work_cutoff() {
    use Operation::*;
    // Directed 512-bit MPFR at grid 2^-8, using monotonicity on [2,3].
    for (op, point, range) in [
        (Exp, (5141, 5142), (1891, 5142)),
        (Log, (281, 282), (177, 282)),
        (Atan, (319, 320), (283, 320)),
    ] {
        for (left, right, (lo, hi)) in [
            (coordinate(6., 3.), coordinate(10., 3.), point),
            (
                Rational::from(3),
                coordinate(281_474_976_710_662., 3.),
                point,
            ),
            (coordinate(10., 2.), coordinate(6., 3.), range),
        ] {
            let input = RationalInterval::new(left, right).unwrap();
            let mut budget = DyadicBudget::new(4096, 1 << 40);
            let image = op.image(&input, &mut budget).unwrap();
            let work = (1 << 40) - budget.remaining_work();
            assert_eq!(image.domain, DomainCoverage::Complete);
            assert_eq!(
                image.range.as_finite().unwrap(),
                &DyadicInterval::new(lo.into(), hi.into(), -8).unwrap()
            );
            assert_eq!(
                op.image(&input, &mut DyadicBudget::new(4096, work - 1)),
                Err(IntervalError::WorkExhausted)
            );
            let mut exact = DyadicBudget::new(4096, work);
            assert_eq!(op.image(&input, &mut exact).unwrap(), image);
            assert_eq!(exact.remaining_work(), 0);
        }
    }
}

#[test]

fn denial_on_a_cold_source_precedes_reduction_and_preserves_a_retry() {
    use Operation::*;
    for op in [Exp, Log, Atan] {
        for y in [2., 3.] {
            let x = coordinate(281_474_976_710_662., y);
            let bits = x.reduction_input_bits();
            assert!(bits > 64);
            let input = RationalInterval::point(x);
            let reductions = 3 * bits * bits;
            for (limit, units, remaining, error) in [
                (0, 100, 100, IntervalError::SizeExhausted),
                (bits - 1, 1 << 30, 1 << 30, IntervalError::SizeExhausted),
                (
                    bits,
                    2 * bits - 1,
                    2 * bits - 1,
                    IntervalError::WorkExhausted,
                ),
                (
                    bits,
                    2 * bits + reductions - 1,
                    reductions - 1,
                    IntervalError::WorkExhausted,
                ),
            ] {
                let mut budget = DyadicBudget::new(limit, units);
                let run = || op.image(&input, &mut budget);
                #[cfg(feature = "dispatch-trace")]
                let result = {
                    use hyperreal::dispatch_trace;
                    dispatch_trace::reset();
                    let result = dispatch_trace::with_recording(run);
                    assert!(dispatch_trace::take_trace().dispatch.is_empty(), "{op:?}");
                    result
                };
                #[cfg(not(feature = "dispatch-trace"))]
                let result = {
                    let mut run = run;
                    run()
                };
                assert_eq!(result, Err(error));
                assert_eq!(budget.remaining_work(), remaining);
            }
            let retry = || {
                op.image(&input, &mut DyadicBudget::new(4096, 1 << 40))
                    .unwrap()
            };
            #[cfg(feature = "dispatch-trace")]
            {
                use hyperreal::dispatch_trace;
                for expected in [1, 0] {
                    dispatch_trace::reset();
                    let image = dispatch_trace::with_recording(retry);
                    assert_eq!(image.domain, DomainCoverage::Complete);
                    assert_eq!(
                        dispatch_trace::take_trace().path_count(
                            "rational",
                            "canonicalization",
                            "lazy-internal-coordinate"
                        ),
                        expected
                    );
                }
            }
            #[cfg(not(feature = "dispatch-trace"))]
            assert_eq!(retry().domain, DomainCoverage::Complete);
        }
    }
}

#[test]
fn logarithm_leaves_excluded_generated_bounds_untouched() {
    let invalid = RationalInterval::point(coordinate(6., -3.));
    let partial =
        RationalInterval::new(coordinate(281_474_976_710_662., -3.), Rational::from(1)).unwrap();
    let mut zero = DyadicBudget::new(0, 0);
    assert_eq!(invalid.log(-8, &mut zero), Err(IntervalError::EmptyDomain));
    let mut small = DyadicBudget::new(32, 1 << 20);
    let image = partial.log(-8, &mut small).unwrap();
    assert_eq!(image.domain, DomainCoverage::Partial);
    assert_eq!(
        image.range,
        ExtendedInterval::below(DyadicInterval::point(0.into(), 0).unwrap()).unwrap()
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn a_refused_second_endpoint_cannot_multiply_unadmitted_coefficients() {
    use hyperreal::dispatch_trace;
    use num_bigint::BigInt;
    let denominator = (BigInt::from(1) << 1024u32) + 1;
    for op in [Operation::Exp, Operation::Log, Operation::Atan] {
        // All three endpoints have the same sign and leading binary exponent.
        // A comparison by products would inspect the far side of the budget.
        let numerator: BigInt =
            &denominator * 3 + if matches!(op, Operation::Log) { -1 } else { 1 };
        let near_three = format!("{numerator}/{denominator}")
            .parse::<Rational>()
            .unwrap();
        let input = if matches!(op, Operation::Log) {
            RationalInterval::new(near_three, Rational::from(3))
        } else {
            RationalInterval::new(Rational::from(3), near_three)
        }
        .unwrap();
        let mut budget = DyadicBudget::new(256, 1 << 40);
        dispatch_trace::reset();
        let result = dispatch_trace::with_recording(|| op.image(&input, &mut budget));
        let count = dispatch_trace::take_trace().path_count(
            "rational",
            "comparison",
            "biguint-cross-product",
        );
        assert_eq!(result, Err(IntervalError::SizeExhausted));
        assert_eq!(count, 0, "{op:?}");
    }
}

#[test]
fn materialized_outward_images_reenter_the_elementary_kernels() {
    let source = RationalInterval::new(coordinate(10., 2.), coordinate(6., 3.)).unwrap();
    let mut budget = DyadicBudget::new(4096, 1 << 40);
    let exp = source.exp(-8, &mut budget).unwrap();
    let log = source.log(-8, &mut budget).unwrap();
    let atan = source.atan(-8, &mut budget).unwrap();
    for receipt in [exp.domain, log.domain, atan.domain] {
        assert_eq!(receipt, DomainCoverage::Complete);
    }
    // Directed MPFR on the retained grid endpoints after each first operation.
    let log_exp = exp.range.log(-8, &mut budget).unwrap();
    let exp_log = log.range.as_finite().unwrap().exp(-8, &mut budget).unwrap();
    let atan_atan = atan.range.atan(-8, &mut budget).unwrap();
    for (image, receipt, lo, hi) in [
        (log_exp.range.as_finite().unwrap(), log_exp.domain, 511, 769),
        (&exp_log.range, exp_log.domain, 511, 771),
        (&atan_atan.range, atan_atan.domain, 213, 230),
    ] {
        assert_eq!(receipt, DomainCoverage::Complete);
        assert_eq!(
            image,
            &DyadicInterval::new(lo.into(), hi.into(), -8).unwrap()
        );
    }

    // Exp is positive, but this outward grid enclosure includes zero.
    let negative = RationalInterval::new(coordinate(6., -8.), coordinate(10., -7.)).unwrap();
    let tiny = negative.exp(-8, &mut budget).unwrap();
    assert_eq!(tiny.domain, DomainCoverage::Complete);
    assert_eq!(
        tiny.range,
        DyadicInterval::new(0.into(), 1.into(), -8).unwrap()
    );
    let result = tiny.range.log(-8, &mut budget).unwrap();
    assert_eq!(result.domain, DomainCoverage::Partial);
    assert_eq!(
        result.range,
        ExtendedInterval::below(DyadicInterval::point((-1419).into(), -8).unwrap()).unwrap()
    );
}

#[path = "support/rational_sqrt_sources.rs"]
mod rational_sqrt_sources;

#[test]
fn generated_square_root_images_retry_and_compose() {
    let budget = || DyadicBudget::new(16384, 100_000_000_000);

    for warm in [false, true] {
        for case in 0..3 {
            let (input, lo) = rational_sqrt_sources::source(case, warm);
            let (a, b) = input.bounds().unwrap();
            let reduction = |x: &Rational| 3 * x.reduction_input_bits().pow(2);
            let upper_charge = reduction(b);
            let mut short = DyadicBudget::new(16384, reduction(a) + upper_charge - 1);
            assert!(matches!(
                input.sqrt(-4, &mut short),
                Err(IntervalError::WorkExhausted)
            ));
            assert_eq!(short.remaining_work(), upper_charge - 1);
            let mut enough = budget();
            let image = input.sqrt(-4, &mut enough).unwrap();
            assert_eq!(image.domain, DomainCoverage::Complete);
            assert_eq!(
                image.range,
                DyadicInterval::new(lo.into(), 28.into(), -4).unwrap()
            );
            let used = budget().remaining_work() - enough.remaining_work();
            let mut exact = DyadicBudget::new(16384, used);
            let repeated = input.sqrt(-4, &mut exact).unwrap();
            assert_eq!(repeated.domain, image.domain);
            assert_eq!(repeated.range, image.range);
            assert_eq!(exact.remaining_work(), 0);
            assert!(matches!(
                input.sqrt(-4, &mut DyadicBudget::new(16384, used - 1)),
                Err(IntervalError::WorkExhausted)
            ));
            // Squaring the returned grid interval must reproduce both exact
            // scaled squares and enclose the original upper endpoint (3).
            let squared = image.range.mul(&image.range, &mut budget()).unwrap();
            assert_eq!(
                squared,
                DyadicInterval::new((lo * lo).into(), 784.into(), -8).unwrap()
            );
        }
    }
    let below = coordinate(281_474_976_710_662., -3.);
    assert!(matches!(
        RationalInterval::point(below.clone()).sqrt(0, &mut DyadicBudget::new(0, 0)),
        Err(IntervalError::EmptyDomain)
    ));
    let partial = RationalInterval::new(below, Rational::from(4)).unwrap();
    let image = partial
        .sqrt(-4, &mut DyadicBudget::new(32, 100_000))
        .unwrap();
    assert_eq!(image.domain, DomainCoverage::Partial);
    assert_eq!(
        image.range,
        DyadicInterval::new(0.into(), 2.into(), 0).unwrap()
    );
}
