#![cfg(all(feature = "rational", feature = "dyadic", feature = "float-enclosure"))]
use hyperinterval::{DyadicBudget, F32Enclosure, F64Enclosure, RationalInterval};
use hyperreal::Rational;

#[test]
fn decimal_tenth_projects_to_adjacent_binary_endpoints() {
    let tenth = RationalInterval::point(Rational::fraction(1, 10).unwrap());
    let mut budget = DyadicBudget::new(4096, 100_000_000);
    if F32Enclosure::backend_supported() {
        let (lo, hi) = F32Enclosure::from_rational(&tenth, &mut budget)
            .unwrap()
            .bounds()
            .unwrap();
        assert_eq!((lo.to_bits(), hi.to_bits()), (0x3dcc_cccc, 0x3dcc_cccd));
        let (lo, hi) = F64Enclosure::from_rational(&tenth, &mut budget)
            .unwrap()
            .bounds()
            .unwrap();
        assert_eq!(
            (lo.to_bits(), hi.to_bits()),
            (0x3fb9_9999_9999_9999, 0x3fb9_9999_9999_999a)
        );
    }
}

use hyperinterval::IntervalError;
use num_bigint::{BigInt, BigUint};

#[path = "support/rational_coordinate.rs"]
mod rational_coordinate;

fn budget() -> DyadicBudget {
    DyadicBudget::new(8192, 1_000_000_000)
}

#[test]
fn empty_gate_shared_budget_and_retry() {
    let source = RationalInterval::new(
        Rational::fraction(-1, 10).unwrap(),
        Rational::fraction(2, 3).unwrap(),
    )
    .unwrap();
    macro_rules! check {
        ($ty:ty) => {{
            assert!(
                <$ty>::from_rational(&RationalInterval::empty(), &mut DyadicBudget::new(0, 0))
                    .unwrap()
                    .is_empty()
            );
            if <$ty>::backend_supported() {
                let mut work = DyadicBudget::new(80, 1_000_000);
                let projected = <$ty>::from_rational(&source, &mut work).unwrap();
                let used = 1_000_000 - work.remaining_work();
                assert!(used > 2);
                assert_eq!(
                    <$ty>::from_rational(&source, &mut DyadicBudget::new(80, used - 1)),
                    Err(IntervalError::WorkExhausted)
                );
                assert_eq!(
                    <$ty>::from_rational(&source, &mut DyadicBudget::new(1, used)),
                    Err(IntervalError::SizeExhausted)
                );
                assert_eq!(
                    <$ty>::from_rational(&source, &mut DyadicBudget::new(80, used)),
                    Ok(projected)
                );
                let back = projected.to_rational();
                assert!(source.subset_of(&back));
                assert_eq!(
                    <$ty>::from_rational(&back, &mut budget()).unwrap(),
                    projected
                );
            } else {
                assert_eq!(
                    <$ty>::from_rational(&source, &mut DyadicBudget::new(0, 0)),
                    Err(IntervalError::UnqualifiedEnvironment)
                );
            }
        }};
    }
    check!(F32Enclosure);
    check!(F64Enclosure);
}

#[test]
fn magnitude_shortcuts_and_unreduced_coordinates() {
    let wide = BigUint::from(1u8) << 6000usize;
    let tiny = Rational::from_bigint_fraction(1.into(), wide.clone()).unwrap();
    let huge = Rational::from_bigint_fraction(BigInt::from(wide), 3u8.into()).unwrap();
    let source = RationalInterval::new(-tiny.clone(), tiny).unwrap();
    let overflow = RationalInterval::point(huge);
    let coordinate = rational_coordinate::coordinate(6., 3.);
    let (.., raw_denominator) = coordinate.ratio_parts();
    assert!(raw_denominator.bits() > 1);
    let raw = RationalInterval::point(coordinate);
    macro_rules! check {
        ($ty:ty, $sign:expr) => {
            if <$ty>::backend_supported() {
                let (lo, hi) = <$ty>::from_rational(&source, &mut DyadicBudget::new(1, 2))
                    .unwrap()
                    .bounds()
                    .unwrap();
                assert_eq!((lo.to_bits(), hi.to_bits()), ($sign | 1, 1));
                assert_eq!(
                    <$ty>::from_rational(&overflow, &mut DyadicBudget::new(1, 1)),
                    Err(IntervalError::Overflow)
                );
                let mut cold = budget();
                assert_eq!(
                    <$ty>::from_rational(&raw, &mut cold).unwrap().bounds(),
                    Some((3., 3.))
                );
                let _ = raw.lower().unwrap().denominator();
                let mut warm = budget();
                assert_eq!(
                    <$ty>::from_rational(&raw, &mut warm).unwrap().bounds(),
                    Some((3., 3.))
                );
                assert_eq!(cold, warm);
            }
        };
    }
    check!(F32Enclosure, 1u32 << 31);
    check!(F64Enclosure, 1u64 << 63);
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/float_oracle.rs"]
mod float_oracle;
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
#[cfg(not(target_env = "msvc"))]
#[path = "support/rational_oracle.rs"]
mod rational_oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn signed_binade_boundaries_and_general_ratios_match_directed_mpfr() {
    if !F64Enclosure::backend_supported() {
        return;
    }
    let mut sources = vec![Rational::zero()];
    let delta =
        Rational::from_bigint_fraction(1.into(), (BigUint::from(1u8) << 2200usize) + 1u8).unwrap();
    for edge in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f32::from_bits(1) as f64,
        f32::MIN_POSITIVE as f64,
        0.5,
        1.,
        2.,
        f32::MAX as f64,
        f64::MAX,
    ] {
        let edge = Rational::try_from(edge).unwrap();
        for value in [&edge - &delta, edge.clone(), &edge + &delta] {
            sources.push(-value.clone());
            sources.push(value);
        }
    }
    let mut state = 37u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for i in 0..512 {
        let exponent = (next() % 2281) as i32 - 1170;
        let mut n = BigInt::from(next() as i64);
        let mut d = BigUint::from(next() | 1);
        if exponent < 0 {
            d <<= exponent.unsigned_abs() as usize;
        } else {
            n <<= exponent as usize;
        }
        let p = Rational::from_bigint_fraction(n, d).unwrap();
        sources.push(p);
        if i % 16 == 0 {
            sources.push(Rational::zero());
        }
    }

    for pair in sources.windows(2) {
        let (lo, hi) = if pair[0] <= pair[1] {
            (&pair[0], &pair[1])
        } else {
            (&pair[1], &pair[0])
        };
        for (a, b) in [(lo, hi), (&pair[0], &pair[0])] {
            let source = RationalInterval::new(a.clone(), b.clone()).unwrap();
            let low = rational_oracle::mpq(a);
            let high = rational_oracle::mpq(b);
            macro_rules! compare {
                ($ty:ty, $round:ident) => {{
                    let expected = float_oracle::$round(&low)
                        .and_then(|l| float_oracle::$round(&high).map(|h| (l.0, h.1)));
                    let got = <$ty>::from_rational(&source, &mut budget()).map(|v| {
                        let (lo, hi) = v.bounds().unwrap();
                        (lo.to_bits(), hi.to_bits())
                    });
                    assert_eq!(got, expected);
                }};
            }
            compare!(F32Enclosure, round_f32);
            compare!(F64Enclosure, round_f64);
        }
    }
}

#[test]
fn distant_endpoints_fit_their_own_grids() {
    if !F64Enclosure::backend_supported() {
        return;
    }
    let power = BigUint::from(1u8) << 70usize;
    let source = RationalInterval::new(
        Rational::from_bigint_fraction(1.into(), &power + 1u8).unwrap(),
        Rational::from_bigint_fraction(BigInt::from(power), 3u8.into()).unwrap(),
    )
    .unwrap();
    let mut limit = DyadicBudget::new(160, 2_000_000);
    let projected = F64Enclosure::from_rational(&source, &mut limit).unwrap();
    assert!(source.subset_of(&projected.to_rational()));
    assert_eq!(
        source.to_dyadic(-1074, &mut DyadicBudget::new(160, 2_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    let common = source.to_dyadic(-1074, &mut budget()).unwrap();
    assert_eq!(
        F64Enclosure::from_dyadic(&common, &mut budget()).unwrap(),
        projected
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn projection_uses_only_rational_and_integer_evidence() {
    if !F32Enclosure::backend_supported() {
        return;
    }
    let source = RationalInterval::point(Rational::fraction(-71, 37).unwrap());
    hyperreal::dispatch_trace::reset();
    hyperreal::dispatch_trace::with_recording(|| {
        F32Enclosure::from_rational(&source, &mut budget()).unwrap();
        F64Enclosure::from_rational(&source, &mut budget()).unwrap();
    });
    assert!(hyperreal::dispatch_trace::take_trace().dispatch.is_empty());
}
