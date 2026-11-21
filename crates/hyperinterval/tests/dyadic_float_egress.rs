#![cfg(all(feature = "dyadic", feature = "float-enclosure"))]
use hyperinterval::{DyadicBudget, DyadicInterval, F32Enclosure, F64Enclosure, IntervalError};
#[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
use num_bigint::BigInt;

#[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
#[path = "support/dyadic_oracle.rs"]
mod exact;
#[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
#[path = "support/float_oracle.rs"]
mod independent;
#[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;

fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000)
}

#[path = "support/dyadic_egress_cases.rs"]
mod portable;

#[test]
fn independent_exact_cases() {
    if F64Enclosure::backend_supported() {
        assert!(portable::verify() > 20);
    }
}

macro_rules! cases {
    ($module:ident, $carrier:ident, $float:ty, $bits:ty, $fraction:literal, $bias:literal, $round:ident) => {
        mod $module {
            use super::*;
            fn bits(value: $carrier) -> Option<($bits, $bits)> {
                value.bounds().map(|(a, b)| (a.to_bits(), b.to_bits()))
            }
            #[test]
            fn empty_qualification_budget_and_retry() {
                assert!(
                    $carrier::from_dyadic(&DyadicInterval::empty(), &mut DyadicBudget::new(0, 0))
                        .unwrap()
                        .is_empty()
                );
                let source = DyadicInterval::new((-3).into(), 7.into(), -1).unwrap();
                if !$carrier::backend_supported() {
                    assert_eq!(
                        $carrier::from_dyadic(&source, &mut DyadicBudget::new(0, 0)),
                        Err(IntervalError::UnqualifiedEnvironment)
                    );
                    return;
                }
                let input = source.clone();
                let mut measured = budget();
                let expected = $carrier::from_dyadic(&source, &mut measured).unwrap();
                let used = 1_000_000_000 - measured.remaining_work();
                assert_eq!(used, 6 * ($fraction + 2));
                let mut exact = DyadicBudget::new($fraction + 2, used);
                assert_eq!(
                    $carrier::from_dyadic(&source, &mut exact).unwrap(),
                    expected
                );
                assert_eq!(exact.remaining_work(), 0);
                for work in [0, used / 2, used - 1] {
                    assert_eq!(
                        $carrier::from_dyadic(&source, &mut DyadicBudget::new($fraction + 2, work)),
                        Err(IntervalError::WorkExhausted)
                    );
                }
                assert_eq!(
                    $carrier::from_dyadic(&source, &mut DyadicBudget::new($fraction + 1, used)),
                    Err(IntervalError::SizeExhausted)
                );
                assert_eq!(
                    $carrier::from_dyadic(&source, &mut budget()).unwrap(),
                    expected
                );
                assert_eq!(source, input);
            }
            #[test]
            fn every_exponent_roundtrips_and_generated_bounds_compose() {
                if !$carrier::backend_supported() {
                    return;
                }
                for exponent in 0..=2 * $bias {
                    for tail in [0, 1, (1 << $fraction) - 1] {
                        for sign in [0, 1 << (<$bits>::BITS - 1)] {
                            let raw: $bits = (exponent << $fraction) | tail | sign;
                            let ieee = $carrier::point(<$float>::from_bits(raw)).unwrap();
                            let exact = ieee.to_dyadic(&mut budget()).unwrap();
                            assert_eq!($carrier::from_dyadic(&exact, &mut budget()).unwrap(), ieee);
                        }
                    }
                }
                // Preserve a full-precision exact result across an explicit
                // weaker endpoint projection, then consume the enclosure again.
                let tiny = DyadicInterval::point(1.into(), -($fraction as i32) - 2).unwrap();
                let source = DyadicInterval::point(1.into(), 0)
                    .unwrap()
                    .add(&tiny, &mut budget())
                    .unwrap();
                let projected = $carrier::from_dyadic(&source, &mut budget()).unwrap();
                let one = (($bias as $bits) << $fraction);
                assert_eq!(bits(projected), Some((one, one + 1)));
                assert!(
                    source
                        .subset_of(&projected.to_dyadic(&mut budget()).unwrap(), &mut budget())
                        .unwrap()
                );
                let doubled = projected.add(&projected).unwrap();
                let exact = source.add(&source, &mut budget()).unwrap();
                assert!(
                    exact
                        .subset_of(&doubled.to_dyadic(&mut budget()).unwrap(), &mut budget())
                        .unwrap()
                );
                #[cfg(feature = "serde")]
                {
                    let text = serde_json::to_vec(&projected).unwrap();
                    let restored: $carrier = serde_json::from_slice(&text).unwrap();
                    assert_eq!(restored, projected);
                    assert_eq!(restored.square().unwrap(), projected.square().unwrap());
                    let text = serde_json::to_vec(&source).unwrap();
                    let restored: DyadicInterval = serde_json::from_slice(&text).unwrap();
                    assert_eq!(
                        $carrier::from_dyadic(&restored, &mut budget()).unwrap(),
                        projected
                    );
                }
            }
            #[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
            #[test]
            fn arbitrary_mantissas_match_directed_mpfr() {
                let mut seed = 0x951c_af80_a771_023du64;
                for i in 0..6000 {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    let size = (seed % 4096) as usize;
                    let mut a =
                        (BigInt::from(seed | 1) << size) + BigInt::from(seed.rotate_left(11));
                    if i % 2 == 0 {
                        a = -a;
                    }
                    let b = &a + BigInt::from(seed.rotate_left(23));
                    let exponent = (i % 2400) - 1200 - size as i32;
                    let source = DyadicInterval::new(a, b, exponent).unwrap();
                    let (a, b, e) = source.bounds().unwrap();
                    let want = match (
                        independent::$round(&exact::q(a, e)),
                        independent::$round(&exact::q(b, e)),
                    ) {
                        (Ok(lo), Ok(hi)) => Ok(Some((lo.0, hi.1))),
                        _ => Err(IntervalError::Overflow),
                    };
                    assert_eq!(
                        $carrier::from_dyadic(&source, &mut budget()).map(bits),
                        want,
                        "case {i}, exponent {e}"
                    );
                }
            }
        }
    };
}
cases!(binary32, F32Enclosure, f32, u32, 23, 127, round_f32);
cases!(binary64, F64Enclosure, f64, u64, 52, 1023, round_f64);

#[cfg(all(feature = "rational", feature = "dispatch-trace"))]
#[test]
fn no_exact_real_dispatch() {
    let source = DyadicInterval::new(3.into(), 7.into(), -5).unwrap();
    hyperreal::dispatch_trace::reset();
    // Unqualified backends refuse egress, and the refusal must not dispatch
    // through exact reals either.
    hyperreal::dispatch_trace::with_recording(|| {
        let single = F32Enclosure::from_dyadic(&source, &mut budget());
        let double = F64Enclosure::from_dyadic(&source, &mut budget());
        if F32Enclosure::backend_supported() {
            single.unwrap();
        } else {
            assert_eq!(single, Err(IntervalError::UnqualifiedEnvironment));
        }
        if F64Enclosure::backend_supported() {
            double.unwrap();
        } else {
            assert_eq!(double, Err(IntervalError::UnqualifiedEnvironment));
        }
    });
    assert!(hyperreal::dispatch_trace::take_trace().dispatch.is_empty());
}
