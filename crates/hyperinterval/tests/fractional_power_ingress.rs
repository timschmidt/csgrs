//! Admission of generated rational bases for fractional powers and square roots.
#![cfg(all(feature = "rational", feature = "dyadic"))]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, RationalInterval,
};
include!("support/rational_coordinate.rs");

#[test]
fn generated_base_must_fit_before_reduction() {
    for warm in [false, true] {
        let base = coordinate(281_474_976_710_662., 3.);
        let bits = base.reduction_input_bits();
        assert_eq!(bits, 98);
        if warm {
            assert_eq!(base.numerator().bits(), 2);
        }
        let source = RationalInterval::point(base);
        let mut small = DyadicBudget::new(bits - 1, 1_000_000);
        let result = source.pow_rational(&1.into(), &3.into(), 0, &mut small);
        assert_eq!(result, Err(IntervalError::SizeExhausted));
        assert_eq!(small.remaining_work(), 1_000_000);
        let mut enough = DyadicBudget::new(1024, 1_000_000);
        let mut retry = || source.pow_rational(&1.into(), &3.into(), 0, &mut enough);
        #[cfg(feature = "dispatch-trace")]
        let image = {
            use hyperreal::dispatch_trace;
            dispatch_trace::reset();
            let result = dispatch_trace::with_recording(&mut retry);
            assert_eq!(
                dispatch_trace::take_trace().path_count(
                    "rational",
                    "canonicalization",
                    "lazy-internal-coordinate"
                ),
                u64::from(!warm)
            );
            result.unwrap()
        };
        #[cfg(not(feature = "dispatch-trace"))]
        let image = retry().unwrap();
        assert_eq!(image.domain, DomainCoverage::Complete);
        assert_eq!(
            image.range.components(),
            &[ExtendedInterval::from_finite(
                DyadicInterval::new(1.into(), 2.into(), 0).unwrap()
            )]
        );
    }
}

#[test]
fn both_endpoint_reductions_are_admitted_before_either_cache_is_warmed() {
    let left = coordinate(6., -8.);
    let right = coordinate(10., 27.);
    let left_work = 3 * left.reduction_input_bits().pow(2);
    let right_work = 3 * right.reduction_input_bits().pow(2);
    assert!(left_work > 0 && right_work > 0);
    let source = RationalInterval::new(left, right).unwrap();
    let run = |budget: &mut DyadicBudget| source.pow_rational(&1.into(), &3.into(), -4, budget);
    let check = || {
        let mut small = DyadicBudget::new(1024, left_work + right_work - 1);
        assert_eq!(run(&mut small), Err(IntervalError::WorkExhausted));
        assert_eq!(small.remaining_work(), right_work - 1);
    };
    #[cfg(feature = "dispatch-trace")]
    {
        use hyperreal::dispatch_trace;
        dispatch_trace::reset();
        dispatch_trace::with_recording(check);
        assert_eq!(
            dispatch_trace::take_trace().path_count(
                "rational",
                "canonicalization",
                "lazy-internal-coordinate"
            ),
            0
        );
    }
    #[cfg(not(feature = "dispatch-trace"))]
    check();
    let mut budget = DyadicBudget::new(1024, 1_000_000);
    let image = run(&mut budget).unwrap();
    assert_eq!(image.domain, DomainCoverage::Complete);
    assert_eq!(
        image.range.components(),
        &[ExtendedInterval::from_finite(
            DyadicInterval::new((-2).into(), 3.into(), 0).unwrap()
        )]
    );
    let work = 1_000_000 - budget.remaining_work();
    assert_eq!(
        run(&mut DyadicBudget::new(1024, work - 1)),
        Err(IntervalError::WorkExhausted)
    );
    let mut exact = DyadicBudget::new(1024, work);
    assert_eq!(run(&mut exact).unwrap(), image);
    assert_eq!(exact.remaining_work(), 0);
}

#[test]
fn trivial_powers_keep_domain_precedence_and_skip_source_reduction() {
    let base = RationalInterval::point(coordinate(281_474_976_710_662., -3.));
    let mut zero = DyadicBudget::new(0, 0);
    assert_eq!(
        base.pow_rational(&0.into(), &0.into(), -4, &mut zero),
        Err(IntervalError::ContainsZero)
    );
    let mut small = DyadicBudget::new(8, 1000);
    let image = base
        .pow_rational(&0.into(), &3.into(), -4, &mut small)
        .unwrap();
    assert_eq!(image.domain, DomainCoverage::Complete);
    assert_eq!(
        image.range.components(),
        &[ExtendedInterval::from_finite(
            DyadicInterval::point(1.into(), 0).unwrap()
        )]
    );
}

#[test]
fn reductions_add_exactly_the_stored_width_charge_across_power_branches() {
    for (lo, hi, n, d) in [(-8, 27, 1, 3), (-8, 27, -1, 3), (-8, 4, 1, 2), (2, 9, 3, 2)] {
        let a = coordinate(6., f64::from(lo));
        let b = coordinate(10., f64::from(hi));
        let reductions = 3 * (a.reduction_input_bits().pow(2) + b.reduction_input_bits().pow(2));
        assert!(reductions > 0);
        let generated = RationalInterval::new(a, b).unwrap();
        let canonical = RationalInterval::new(Rational::from(lo), Rational::from(hi)).unwrap();
        let run = |source: &RationalInterval| {
            let mut budget = DyadicBudget::new(4096, 1 << 40);
            let image = source
                .pow_rational(&n.into(), &d.into(), -8, &mut budget)
                .unwrap();
            (image, (1 << 40) - budget.remaining_work())
        };
        let (expected, canonical_work) = run(&canonical);
        let first = run(&generated);
        assert_eq!(first.0, expected);
        assert_eq!(first.1, canonical_work + reductions);
        assert_eq!(run(&generated), first);
    }
}

#[path = "support/rational_sqrt_sources.rs"]
mod rational_sqrt_sources;

#[test]
fn generated_square_root_endpoints_are_admitted_before_use() {
    let budget = || DyadicBudget::new(16384, 100_000_000_000);

    for warm in [false, true] {
        for case in 0..3 {
            let (input, _) = rational_sqrt_sources::source(case, warm);
            let bits = input.upper().unwrap().reduction_input_bits();
            let mut denied = DyadicBudget::new(bits - 1, 100_000_000);
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let result = {
                #[cfg(feature = "dispatch-trace")]
                let _recording = hyperreal::dispatch_trace::recording_scope();
                input.sqrt(-4, &mut denied)
            };
            assert!(matches!(result, Err(IntervalError::SizeExhausted)));
            #[cfg(feature = "dispatch-trace")]
            assert_eq!(
                hyperreal::dispatch_trace::take_trace().path_count(
                    "rational",
                    "canonicalization",
                    "lazy-internal-coordinate"
                ),
                0
            );
            let mut enough = budget();
            let image = input.sqrt(-4, &mut enough).unwrap();
            let power = input
                .pow_rational(&1.into(), &2.into(), -4, &mut budget())
                .unwrap();
            assert_eq!(power.domain, image.domain);
            let [component] = power.range.components() else {
                panic!("positive square root must have one component");
            };
            assert_eq!(component.as_finite().unwrap(), &image.range);
        }
    }
}
