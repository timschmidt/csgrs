//! Non-singleton dyadic cells immediately below, across and above a phase landmark.
//! Source numerators use 2^-48 and expected image numerators use 2^-8. Directed
//! 512-bit MPFR bounds agree on every rounded constant, including the pole cells.
#![cfg(feature = "dyadic")]

use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, IntervalSet2,
    RangeEnclosure,
};
use std::fmt::Debug;

fn grid(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}

fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 1 << 29)
}

fn with_exact_retry<T: Debug + Eq>(
    evaluate: impl Fn(&mut DyadicBudget) -> Result<T, IntervalError>,
) -> T {
    let mut work = budget();
    let start = work.remaining_work();
    assert_eq!(
        evaluate(&mut DyadicBudget::new(64, start)),
        Err(IntervalError::SizeExhausted)
    );
    let result = evaluate(&mut work).unwrap();
    let cost = start - work.remaining_work();
    assert!(cost > 0);
    assert_eq!(
        evaluate(&mut DyadicBudget::new(4096, cost - 1)),
        Err(IntervalError::WorkExhausted)
    );
    let mut retry = DyadicBudget::new(4096, cost);
    assert_eq!(evaluate(&mut retry).unwrap(), result);
    assert_eq!(retry.remaining_work(), 0);
    result
}

#[test]
fn sine_and_cosine_on_both_sides_of_one_half() {
    // sin(pi/6) = cos(pi/3) = 1/2. At these points sin is increasing and
    // cos is decreasing. Both cell endpoints can need further refinement.
    for (cosine, floor, bounds) in [
        (
            false,
            147_379_953_167_259,
            [(127, 128), (127, 129), (128, 129)],
        ),
        (
            true,
            294_759_906_334_518,
            [(128, 129), (127, 129), (127, 128)],
        ),
    ] {
        for (offset, (lo, hi)) in (-1..=1).zip(bounds) {
            let a = floor + offset;
            let source = grid(a, a + 1, -48);
            let evaluate = |x: &DyadicInterval, work: &mut DyadicBudget| {
                if cosine {
                    x.cos(-8, work)
                } else {
                    x.sin(-8, work)
                }
            };
            let image = with_exact_retry(|work| evaluate(&source, work));
            assert_eq!(image.domain, DomainCoverage::Complete);
            assert_eq!(image.range, grid(lo, hi, -8));
            let reflected = evaluate(&grid(-a - 1, -a, -48), &mut budget()).unwrap();
            assert_eq!(reflected.domain, image.domain);
            assert_eq!(
                reflected.range,
                if cosine {
                    image.range
                } else {
                    image.range.neg(&mut budget()).unwrap()
                }
            );
        }
    }
}

enum Expected {
    Finite(i64, i64),
    Split { below: i64, above: i64 },
}

#[test]
fn tangent_and_cotangent_distinguish_grid_crossings_from_poles() {
    use Expected::{Finite, Split};
    // tan(pi/4) = cot(pi/4) = 1. At pi/2 for tan and pi for cot the
    // middle cell instead crosses a pole. Its two outer rays exclude zero.
    for (cot, floor, images) in [
        (
            false,
            221_069_929_750_888,
            [Finite(255, 256), Finite(255, 257), Finite(256, 257)],
        ),
        (
            true,
            221_069_929_750_888,
            [Finite(256, 257), Finite(255, 257), Finite(255, 256)],
        ),
        (
            false,
            442_139_859_501_777,
            [
                Finite(47_492_693_218_999_551, 139_312_966_617_213_172),
                Split {
                    below: -149_260_301_560_123_174,
                    above: 139_312_966_617_213_171,
                },
                Finite(-149_260_301_560_123_175, -48_596_785_120_944_224),
            ],
        ),
        (
            true,
            884_279_719_003_555,
            [
                Finite(-2_090_398_637_209_007_329, -69_656_483_308_606_585),
                Split {
                    below: -2_090_398_637_209_007_328,
                    above: 74_630_150_780_061_587,
                },
                Finite(36_660_657_061_512_891, 74_630_150_780_061_588),
            ],
        ),
    ] {
        for (offset, expected) in (-1..=1).zip(images) {
            let a = floor + offset;
            let source = grid(a, a + 1, -48);
            let evaluate = |x: &DyadicInterval, work: &mut DyadicBudget| {
                if cot {
                    x.cot(-8, work)
                } else {
                    x.tan(-8, work)
                }
            };
            let image = with_exact_retry(|work| evaluate(&source, work));
            match expected {
                Finite(lo, hi) => {
                    assert_eq!(image.domain, DomainCoverage::Complete);
                    assert_eq!(
                        image.range.components(),
                        &[ExtendedInterval::from_finite(grid(lo, hi, -8))]
                    );
                }
                Split { below, above } => {
                    assert_eq!(image.domain, DomainCoverage::Partial);
                    assert_eq!(
                        image.range.components(),
                        &[
                            ExtendedInterval::below(grid(below, below, -8)).unwrap(),
                            ExtendedInterval::above(grid(above, above, -8)).unwrap(),
                        ]
                    );
                    assert!(
                        !image
                            .range
                            .contains_point(&0.into(), 0, &mut budget())
                            .unwrap()
                    );
                }
            }
            let reflected = evaluate(&grid(-a - 1, -a, -48), &mut budget()).unwrap();
            assert_eq!(reflected.domain, image.domain);
            assert_eq!(reflected.range, image.range.neg(&mut budget()).unwrap());
        }
    }
}

// A rejected reservation leaves its work units unspent. Starting just below a
// completed call's cost, the spent prefix identifies the preceding boundary.
fn check_work_boundaries<T: Debug + Eq>(
    evaluate: impl Fn(&mut DyadicBudget) -> Result<T, IntervalError>,
    expected: T,
) {
    let mut work = budget();
    let available = work.remaining_work();
    assert_eq!(evaluate(&mut work).unwrap(), expected);
    let cost = available - work.remaining_work();
    let mut prefix = cost;
    let mut steps = 0;
    while prefix > 0 {
        assert!(steps < 1024, "reservation sweep exceeded its test budget");
        let limit = prefix - 1;
        let mut short = DyadicBudget::new(4096, limit);
        assert_eq!(evaluate(&mut short), Err(IntervalError::WorkExhausted));
        prefix = limit - short.remaining_work();
        steps += 1;
    }
    assert_eq!(
        evaluate(&mut DyadicBudget::new(4096, cost)).unwrap(),
        expected
    );
}

// Both sweep intervals end at 2. The negative ray ends at ceil(16*tan(2))/16.
fn pole_image(positive_lower: i64) -> RangeEnclosure<IntervalSet2<DyadicInterval>> {
    let rays = IntervalSet2::new(
        ExtendedInterval::below(grid(-34, -34, -4)).unwrap(),
        ExtendedInterval::above(grid(positive_lower, positive_lower, -4)).unwrap(),
        &mut budget(),
    )
    .unwrap();
    RangeEnclosure {
        range: rays,
        domain: DomainCoverage::Partial,
    }
}

#[test]
fn work_exhaustion_through_critical_point_and_pole_images() {
    let source = grid(1, 2, 0);
    check_work_boundaries(
        |work| source.sin(-4, work),
        RangeEnclosure {
            range: grid(13, 16, -4),
            domain: DomainCoverage::Complete,
        },
    );
    check_work_boundaries(|work| source.tan(-4, work), pole_image(24));
}

#[cfg(feature = "rational")]
#[test]
fn rational_source_exhaustion_preserves_the_pole_and_extremum() {
    use hyperinterval::RationalInterval;
    use hyperreal::Rational;

    let source = RationalInterval::new(Rational::fraction(1, 2).unwrap(), 2.into()).unwrap();
    check_work_boundaries(
        |work| source.sin(-4, work),
        RangeEnclosure {
            range: grid(7, 16, -4),
            domain: DomainCoverage::Complete,
        },
    );
    check_work_boundaries(|work| source.tan(-4, work), pole_image(8));
}
