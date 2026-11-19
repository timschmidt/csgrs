#![cfg(feature = "float-proposal")]

use hyperinterval::{F32ProposalInterval, F64ProposalInterval, IntervalError};

macro_rules! cases {
    ($module:ident, $interval:ident, $float:ty, $bits:ty) => {
        mod $module {
            use super::*;

            #[test]
            fn midpoint_rounds_once_on_the_subnormal_grid() {
                let grid = |units: i32| {
                    let value = <$float>::from_bits(units.unsigned_abs() as $bits);
                    if units < 0 { -value } else { value }
                };
                for lo in -31..=31 {
                    for hi in lo..=31 {
                        let sum: i32 = lo + hi;
                        let floor = sum.div_euclid(2);
                        let rounded = floor + i32::from(sum % 2 != 0 && floor % 2 != 0);
                        let expected = if rounded == 0 && sum < 0 {
                            -0.0
                        } else {
                            grid(rounded)
                        };
                        let image = $interval::new(grid(lo), grid(hi)).unwrap();
                        assert_eq!(
                            image.midpoint_estimate().unwrap().to_bits(),
                            expected.to_bits(),
                            "units [{lo},{hi}]"
                        );
                    }
                }
                let tiny = <$float>::MIN_POSITIVE;
                for value in [<$float>::from_bits(tiny.to_bits() + 1), <$float>::MAX] {
                    assert_eq!(
                        $interval::point(value).unwrap().midpoint_estimate(),
                        Some(value)
                    );
                    assert_eq!(
                        $interval::point(-value).unwrap().midpoint_estimate(),
                        Some(-value)
                    );
                }
            }

            #[test]
            fn finite_order_and_special_values() {
                assert_eq!(
                    $interval::new(2.0, 1.0),
                    Err(IntervalError::ReversedEndpoints)
                );
                for invalid in [<$float>::NAN, <$float>::INFINITY, <$float>::NEG_INFINITY] {
                    assert_eq!(
                        $interval::point(invalid),
                        Err(IntervalError::NonFiniteEndpoint)
                    );
                    assert_eq!(
                        $interval::new(0.0, invalid),
                        Err(IntervalError::NonFiniteEndpoint)
                    );
                    assert_eq!(
                        $interval::new(invalid, 0.0),
                        Err(IntervalError::NonFiniteEndpoint)
                    );
                }
                for zero in [-0.0, 0.0] {
                    let (a, b) = $interval::point(zero).unwrap().estimates().unwrap();
                    assert_eq!(a.to_bits(), 0);
                    assert_eq!(b.to_bits(), 0);
                }
                let tiny = <$float>::from_bits(1);
                assert_eq!(
                    $interval::new(tiny, 0.0),
                    Err(IntervalError::ReversedEndpoints)
                );
                assert_eq!(
                    $interval::new(0.0, -tiny),
                    Err(IntervalError::ReversedEndpoints)
                );
                assert_eq!(
                    $interval::new(-tiny, tiny).unwrap().estimates(),
                    Some((-tiny, tiny))
                );
                assert_ne!(
                    $interval::point(tiny).unwrap(),
                    $interval::point(0.0).unwrap()
                );
                let empty = $interval::empty();
                assert!(empty.is_empty());
                assert_eq!(empty.estimates(), None);
                assert_eq!(empty.midpoint_estimate(), None);
                assert!(!($interval::point(0.0).unwrap().is_empty()));
            }

            #[test]
            fn arithmetic_remains_approximate_and_failures_stay_explicit() {
                let x = $interval::new(-2.0, 3.0).unwrap();
                let y = $interval::new(4.0, 8.0).unwrap();
                assert_eq!(x.add(&y).unwrap().estimates(), Some((2.0, 11.0)));
                assert_eq!(x.sub(&y).unwrap().estimates(), Some((-10.0, -1.0)));
                assert_eq!(x.mul(&y).unwrap().estimates(), Some((-16.0, 24.0)));
                assert_eq!(x.div(&y).unwrap().estimates(), Some((-0.5, 0.75)));
                assert_eq!(x.neg().estimates(), Some((-3.0, 2.0)));
                assert_eq!(x.midpoint_estimate(), Some(0.5));
                assert_eq!(x.affine(-2.0, 1.0).unwrap().estimates(), Some((-5.0, 5.0)));
                let empty = $interval::empty();
                assert!(x.add(&empty).unwrap().is_empty());
                assert!(x.sub(&empty).unwrap().is_empty());
                assert!(x.mul(&empty).unwrap().is_empty());
                assert!(x.div(&empty).unwrap().is_empty());
                assert!(empty.div(&y).unwrap().is_empty());
                assert!(empty.neg().is_empty());
                assert_eq!(empty.div(&x), Err(IntervalError::ContainsZero));
                assert_eq!(
                    x.div(&$interval::point(-0.0).unwrap()),
                    Err(IntervalError::ContainsZero)
                );
                assert_eq!(
                    empty.affine(<$float>::NAN, 0.0),
                    Err(IntervalError::NonFiniteEndpoint)
                );
                assert_eq!(
                    empty.affine(0.0, <$float>::INFINITY),
                    Err(IntervalError::NonFiniteEndpoint)
                );
                let max = $interval::point(<$float>::MAX).unwrap();
                assert_eq!(max.add(&max), Err(IntervalError::Overflow));
                assert_eq!(
                    max.mul(&$interval::point(2.0).unwrap()),
                    Err(IntervalError::Overflow)
                );
                assert_eq!(max.midpoint_estimate(), Some(<$float>::MAX));
                // This rounded proposal excludes the exact real sum; it must
                // never acquire a certified-enclosure type by conversion.
                let rounded = $interval::point(1.0)
                    .unwrap()
                    .add(&$interval::point(<$float>::EPSILON * 0.25).unwrap())
                    .unwrap();
                assert_eq!(rounded.estimates(), Some((1.0, 1.0)));
            }

            #[test]
            fn random_bit_ingress_matches_ieee_order() {
                let mut seed = 0x972b_fea9_c713_d021u64;
                for _ in 0..10_000 {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    let a = <$float>::from_bits(seed as $bits);
                    seed = seed.rotate_left(23).wrapping_add(0x9e37_79b9_7f4a_7c15);
                    let b = <$float>::from_bits(seed as $bits);
                    let result = $interval::new(a, b);
                    if !a.is_finite() || !b.is_finite() {
                        assert_eq!(result, Err(IntervalError::NonFiniteEndpoint));
                    } else if a > b {
                        assert_eq!(result, Err(IntervalError::ReversedEndpoints));
                    } else {
                        let (lo, hi) = result.unwrap().estimates().unwrap();
                        assert_eq!(lo, a);
                        assert_eq!(hi, b);
                    }
                }
            }

            #[cfg(feature = "serde")]
            #[test]
            fn replay_uses_validated_bits_and_preserves_proposal_type() {
                for value in [
                    $interval::empty(),
                    $interval::new(-<$float>::MAX, <$float>::from_bits(1)).unwrap(),
                    $interval::point(-0.0).unwrap(),
                ] {
                    let bytes = serde_json::to_vec(&value).unwrap();
                    assert_eq!(serde_json::from_slice::<$interval>(&bytes).unwrap(), value);
                }
                for (a, b) in [(2.0, 1.0), (0.0, <$float>::INFINITY), (<$float>::NAN, 1.0)] {
                    let invalid = serde_json::to_vec(&(a.to_bits(), b.to_bits())).unwrap();
                    assert!(serde_json::from_slice::<$interval>(&invalid).is_err());
                }
            }
        }
    };
}

cases!(binary32, F32ProposalInterval, f32, u32);
cases!(binary64, F64ProposalInterval, f64, u64);

#[test]
fn binary32_radius_rounds_the_exact_half_width_once() {
    let grid = |n: i32| {
        let value = f32::from_bits(n.unsigned_abs());
        if n < 0 { -value } else { value }
    };
    for lo in -32..=32 {
        for hi in lo..=32 {
            let width: i32 = hi - lo;
            let floor = width / 2;
            let expected = grid(floor + i32::from(width % 2 == 1 && floor % 2 == 1));
            let span = F32ProposalInterval::new(grid(lo), grid(hi)).unwrap();
            assert_eq!(
                span.radius_estimate(),
                Some(expected),
                "subnormal units [{lo}, {hi}]"
            );
        }
    }
    assert_eq!(F32ProposalInterval::empty().radius_estimate(), None);
    let max = f32::MAX;
    let prev = f32::from_bits(max.to_bits() - 1);
    for (lo, hi, radius) in [
        (-max, max, max),
        (-max, prev, prev),
        (prev, max, (max - prev) * 0.5),
        (0.0, max, max * 0.5),
        (-0.0, 0.0, 0.0),
        (max, max, 0.0),
    ] {
        let span = F32ProposalInterval::new(lo, hi).unwrap();
        assert_eq!(span.radius_estimate().unwrap().to_bits(), radius.to_bits());
    }
}
