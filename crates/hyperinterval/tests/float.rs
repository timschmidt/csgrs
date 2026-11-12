#![cfg(feature = "float-enclosure")]
#[cfg(target_arch = "x86_64")]
use hyperinterval::SignSet;
use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};

#[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
#[path = "support/float_oracle.rs"]
mod independent;

#[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
#[path = "support/primitive_sqrt_oracle.rs"]
mod root_oracle;

macro_rules! cases {
    ($module:ident, $interval:ident, $float:ty, $bits:ty, $from:ident, $round:ident, $root:ident, $fraction:literal) => {
        mod $module {
            use super::*;

            #[test]
            fn target_gate_and_empty_are_explicit() {
                assert_eq!($interval::backend_supported(), cfg!(target_arch = "x86_64"));
                assert!($interval::empty().is_empty());
                let root = $interval::empty().sqrt().unwrap();
                assert!(root.range.is_empty());
                assert_eq!(root.domain, hyperinterval::DomainCoverage::Complete);
                if !$interval::backend_supported() {
                    assert_eq!(
                        $interval::point(1.0),
                        Err(IntervalError::UnqualifiedEnvironment)
                    );
                    assert_eq!(
                        $interval::empty().contains_point(0.0),
                        Err(IntervalError::UnqualifiedEnvironment)
                    );
                }
            }

            #[cfg(target_arch = "x86_64")]
            #[test]
            fn exact_sets_arithmetic_domains_and_composition() {
                let x = $interval::new(-2.0, 3.0).unwrap();
                let y = $interval::new(4.0, 8.0).unwrap();
                let empty = $interval::empty();
                assert_eq!(x.lower(), Some(-2.0));
                assert_eq!(x.upper(), Some(3.0));
                assert_eq!(empty.lower(), None);
                assert_eq!(empty.upper(), None);
                assert!(!x.is_singleton());
                assert!(!empty.is_singleton());
                assert!($interval::point(1.0).unwrap().is_singleton());
                assert_eq!(x.add(&y).unwrap().bounds(), Some((2.0, 11.0)));
                assert_eq!(x.sub(&y).unwrap().bounds(), Some((-10.0, -1.0)));
                assert_eq!(x.mul(&y).unwrap().bounds(), Some((-16.0, 24.0)));
                assert_eq!(x.div(&y).unwrap().bounds(), Some((-0.5, 0.75)));
                assert_eq!(x.neg().bounds(), Some((-3.0, 2.0)));
                assert_eq!(x.square().unwrap().bounds(), Some((0.0, 9.0)));
                assert_eq!(
                    $interval::new(-3.0, -2.0)
                        .unwrap()
                        .square()
                        .unwrap()
                        .bounds(),
                    Some((4.0, 9.0))
                );
                assert_eq!(x.checked_pow(3).unwrap().bounds(), Some((-8.0, 27.0)));
                assert_eq!(x.checked_pow(4).unwrap().bounds(), Some((0.0, 81.0)));
                assert_eq!(x.checked_pow(0).unwrap().bounds(), Some((1.0, 1.0)));
                assert_eq!(x.checked_pow(1).unwrap(), x);
                assert_eq!(
                    $interval::new(-1.0, 1.0)
                        .unwrap()
                        .checked_pow(u32::MAX)
                        .unwrap()
                        .bounds(),
                    Some((-1.0, 1.0))
                );
                assert_eq!(x.affine(-2.0, 1.0).unwrap().bounds(), Some((-5.0, 5.0)));
                assert_eq!(y.reciprocal().unwrap().bounds(), Some((0.125, 0.25)));
                assert_eq!(x.signs(), SignSet::Mixed);
                assert_eq!(empty.signs(), SignSet::Empty);
                assert!(x.contains_point(0.0).unwrap());
                assert!(!empty.contains_point(0.0).unwrap());
                assert!(x.strictly_left_of(&y));
                assert!(!empty.strictly_left_of(&y));
                assert!(x.disjoint(&y));
                assert!(!x.overlaps(&y));
                let hull = x.hull(&y);
                assert_eq!(hull.bounds(), Some((-2.0, 8.0)));
                assert_eq!(hull.intersection(&y), y);
                assert!(x.intersection(&y).is_empty());
                assert!(x.intersection(&empty).is_empty());
                assert!(x.subset_of(&hull));
                assert!(empty.subset_of(&x));
                assert!(!x.subset_of(&empty));
                assert!($interval::point(0.0).unwrap().interior_subset_of(&x));
                assert!(!x.interior_subset_of(&x));
                assert!(empty.interior_subset_of(&empty));
                assert!(!x.interior_subset_of(&empty));
                assert_eq!(x.hull(&empty), x);
                assert_eq!(empty.hull(&y), y);
                assert!(empty.neg().is_empty());
                for result in [
                    x.add(&empty),
                    x.sub(&empty),
                    x.mul(&empty),
                    x.div(&empty),
                    empty.div(&y),
                    empty.reciprocal(),
                    empty.square(),
                    empty.checked_pow(0),
                    empty.expanded(),
                ] {
                    assert_eq!(result.unwrap(), empty);
                }
                assert_eq!(empty.div(&x), Err(IntervalError::ContainsZero));
                assert_eq!(x.reciprocal(), Err(IntervalError::ContainsZero));
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
                        empty.contains_point(invalid),
                        Err(IntervalError::NonFiniteEndpoint)
                    );
                    assert_eq!(
                        empty.affine(invalid, 0.0),
                        Err(IntervalError::NonFiniteEndpoint)
                    );
                    assert_eq!(
                        empty.affine(0.0, invalid),
                        Err(IntervalError::NonFiniteEndpoint)
                    );
                }
                // Repeated operation consumes generated bounds without demotion.
                assert_eq!(
                    y.reciprocal()
                        .unwrap()
                        .square()
                        .unwrap()
                        .affine(64.0, -1.0)
                        .unwrap()
                        .bounds(),
                    Some((0.0, 3.0))
                );
            }

            #[cfg(target_arch = "x86_64")]
            #[test]
            fn signed_zero_subnormals_expansion_and_overflow() {
                let zero = $interval::point(-0.0).unwrap();
                assert_eq!(zero.bounds().unwrap().0.to_bits(), 0);
                let tiny = <$float>::from_bits(1);
                assert_eq!($interval::next_up(-0.0).unwrap().to_bits(), 1);
                assert_eq!(
                    $interval::next_down(0.0).unwrap().to_bits(),
                    (-tiny).to_bits()
                );
                assert_eq!(zero.expanded().unwrap().bounds(), Some((-tiny, tiny)));
                let small = $interval::point(tiny).unwrap();
                assert_ne!(small, zero);
                assert_eq!(small.square().unwrap().bounds(), Some((0.0, tiny)));
                assert_eq!(
                    small.mul(&small.neg()).unwrap().bounds(),
                    Some((-tiny, 0.0))
                );
                let maximum = $interval::point(<$float>::MAX).unwrap();
                assert_eq!(maximum.affine(2.0, -<$float>::MAX).unwrap(), maximum);
                assert_eq!(maximum.affine(0.0, tiny).unwrap(), small);
                assert_eq!(maximum.add(&small), Err(IntervalError::Overflow));
                assert_eq!(maximum.expanded(), Err(IntervalError::Overflow));
                assert_eq!(maximum.checked_pow(2), Err(IntervalError::Overflow));
                assert_eq!(
                    $interval::next_up(<$float>::MAX),
                    Err(IntervalError::Overflow)
                );
                assert_eq!(
                    $interval::next_down(-<$float>::MAX),
                    Err(IntervalError::Overflow)
                );
                assert_eq!(
                    maximum.sub(&small).unwrap().bounds(),
                    Some(($interval::next_down(<$float>::MAX).unwrap(), <$float>::MAX))
                );
                assert_eq!(
                    small
                        .div(&$interval::point(<$float>::MAX).unwrap())
                        .unwrap()
                        .bounds(),
                    Some((0.0, tiny))
                );
            }

            #[cfg(all(feature = "serde", target_arch = "x86_64"))]
            #[test]
            fn replay_validates_bits_and_generated_images() {
                let generated = $interval::new(1.0, 3.0)
                    .unwrap()
                    .reciprocal()
                    .unwrap()
                    .checked_pow(3)
                    .unwrap();
                for x in [
                    $interval::empty(),
                    generated,
                    $interval::new(-<$float>::MAX, <$float>::from_bits(1)).unwrap(),
                ] {
                    let bytes = serde_json::to_vec(&x).unwrap();
                    let restored: $interval = serde_json::from_slice(&bytes).unwrap();
                    assert_eq!(restored, x);
                    assert_eq!(restored.square(), x.square());
                }
                for (a, b) in [(2.0, 1.0), (0.0, <$float>::INFINITY), (<$float>::NAN, 1.0)] {
                    let bytes = serde_json::to_vec(&(a.to_bits(), b.to_bits())).unwrap();
                    assert!(serde_json::from_slice::<$interval>(&bytes).is_err());
                }
            }

            #[cfg(all(not(target_env = "msvc"), target_arch = "x86_64"))]
            mod oracle {
                use super::*;
                use rug::Rational;
                fn expected(value: &Rational) -> Result<($bits, $bits), IntervalError> {
                    crate::independent::$round(value)
                }
                fn actual(
                    value: Result<$interval, IntervalError>,
                ) -> Result<($bits, $bits), IntervalError> {
                    value.map(|v| {
                        let (a, b) = v.bounds().unwrap();
                        (a.to_bits(), b.to_bits())
                    })
                }
                fn pair(a: $float, b: $float) {
                    let (Some(x), Some(y)) = (Rational::$from(a), Rational::$from(b)) else {
                        return;
                    };
                    let aa = $interval::point(a).unwrap();
                    let bb = $interval::point(b).unwrap();
                    for (name, got, want) in [
                        ("add", aa.add(&bb), expected(&Rational::from(&x + &y))),
                        ("sub", aa.sub(&bb), expected(&Rational::from(&x - &y))),
                        ("mul", aa.mul(&bb), expected(&Rational::from(&x * &y))),
                        (
                            "div",
                            aa.div(&bb),
                            if y == 0 {
                                Err(IntervalError::ContainsZero)
                            } else {
                                expected(&Rational::from(&x / &y))
                            },
                        ),
                    ] {
                        assert_eq!(
                            actual(got),
                            want,
                            "{name} {:x} {:x}",
                            a.to_bits(),
                            b.to_bits()
                        );
                    }
                }

                #[test]
                fn edge_classes_and_random_pairs_match_tight_mpfr_bounds() {
                    let mut values = vec![
                        0.0,
                        -0.0,
                        1.0,
                        -1.0,
                        3.0,
                        -3.0,
                        <$float>::MAX,
                        -<$float>::MAX,
                        <$float>::MIN_POSITIVE,
                        -<$float>::MIN_POSITIVE,
                    ];
                    for bits in [
                        1,
                        2,
                        3,
                        (1 << $fraction) - 1,
                        1 << $fraction,
                        (1 << $fraction) + 1,
                    ] {
                        let x = <$float>::from_bits(bits);
                        values.extend([x, -x]);
                    }
                    let bias = ((1u32 << (<$bits>::BITS - 1 - $fraction)) - 1) / 2;
                    for exponent in [
                        1,
                        2,
                        3,
                        52,
                        74,
                        75,
                        76,
                        100,
                        126,
                        bias - 76,
                        bias - 75,
                        bias - 74,
                        bias - 1,
                        bias,
                        bias + 1,
                        2 * bias - 1,
                        2 * bias,
                    ] {
                        // Values around binade boundaries cover exact-alignment
                        // and distant-addend dispatch on both formats.
                        let field = exponent as $bits;
                        for delta in [-1i64, 0, 1] {
                            let bits = ((field << $fraction) as i64 + delta) as $bits;
                            let x = <$float>::from_bits(bits);
                            if x.is_finite() {
                                values.extend([x, -x]);
                            }
                        }
                    }
                    for &a in &values {
                        for &b in &values {
                            pair(a, b);
                        }
                    }
                    let mut seed = 0x1b05_3f72_11c0_d90du64;
                    for _ in 0..20_000 {
                        seed ^= seed << 13;
                        seed ^= seed >> 7;
                        seed ^= seed << 17;
                        let a = <$float>::from_bits(seed as $bits);
                        seed = seed.rotate_left(19).wrapping_add(0x9e37_79b9_7f4a_7c15);
                        pair(a, <$float>::from_bits(seed as $bits));
                    }
                }

                #[test]
                fn rounded_powers_enclose_independent_rational_powers() {
                    use rug::ops::Pow;
                    let mut seed = 0x713a_50dc_e952_b6fdu64;
                    let bias = ((1u32 << (<$bits>::BITS - 1 - $fraction)) - 1) / 2;
                    for i in 0..2_000 {
                        seed ^= seed << 13;
                        seed ^= seed >> 7;
                        seed ^= seed << 17;
                        // Magnitudes in [1/2,2), powers at most 31: both the
                        // true image and accumulated outward bounds stay finite.
                        let bits = (seed as $bits & ((1 << $fraction) - 1))
                            | (((bias - (i % 2)) as $bits) << $fraction);
                        let a = <$float>::from_bits(bits);
                        let a = if i % 3 == 0 { -a } else { a };
                        let exponent = i % 32;
                        let exact = Rational::$from(a).unwrap().pow(exponent);
                        let (lo, hi) = $interval::point(a)
                            .unwrap()
                            .checked_pow(exponent)
                            .unwrap()
                            .bounds()
                            .unwrap();
                        assert!(Rational::$from(lo).unwrap() <= exact);
                        assert!(Rational::$from(hi).unwrap() >= exact);
                    }
                }

                #[test]
                fn fused_affine_matches_exact_rational_images() {
                    let check = |a: $float, b: $float, scale: $float, offset: $float| {
                        if [a, b, scale, offset].iter().any(|x| !x.is_finite()) {
                            return;
                        }
                        let x = $interval::new(a.min(b), a.max(b)).unwrap();
                        let scale_r = Rational::$from(scale).unwrap();
                        let offset_r = Rational::$from(offset).unwrap();
                        let image = |v| Rational::$from(v).unwrap() * &scale_r + &offset_r;
                        let (u, v) = (image(a), image(b));
                        let want = match (expected((&u).min(&v)), expected((&u).max(&v))) {
                            (Ok(lo), Ok(hi)) => Ok((lo.0, hi.1)),
                            _ => Err(IntervalError::Overflow),
                        };
                        assert_eq!(
                            actual(x.affine(scale, offset)),
                            want,
                            "affine {:x} {:x} {:x} {:x}",
                            a.to_bits(),
                            b.to_bits(),
                            scale.to_bits(),
                            offset.to_bits()
                        );
                    };
                    let values = [
                        0.0,
                        -0.0,
                        1.0,
                        -1.0,
                        2.0,
                        3.0,
                        <$float>::from_bits(1),
                        -<$float>::from_bits(1),
                        <$float>::MIN_POSITIVE,
                        <$float>::MAX,
                        -<$float>::MAX,
                        $interval::next_down(1.0).unwrap(),
                        $interval::next_up(1.0).unwrap(),
                    ];
                    for a in values {
                        for scale in values {
                            for offset in values {
                                check(a, a, scale, offset);
                            }
                        }
                    }
                    let mut seed = 0xf0a5_21bb_c783_197du64;
                    for _ in 0..20_000 {
                        let mut values = [0.0; 4];
                        for v in &mut values {
                            seed ^= seed << 13;
                            seed ^= seed >> 7;
                            seed ^= seed << 17;
                            *v = <$float>::from_bits(seed as $bits);
                        }
                        let [a, b, scale, offset] = values;
                        check(a, a, scale, offset);
                        check(a, b, scale, offset);
                    }
                }

                #[test]
                fn interval_images_match_rational_corner_extrema() {
                    let mut seed = 0x2de1_b59a_c83f_7107u64;
                    for _ in 0..4_000 {
                        let mut values = [0.0; 4];
                        for value in &mut values {
                            seed ^= seed << 13;
                            seed ^= seed >> 7;
                            seed ^= seed << 17;
                            *value = <$float>::from_bits(seed as $bits);
                        }
                        if values.iter().any(|v| !v.is_finite()) {
                            continue;
                        }
                        let [a, b, c, d] = values;
                        let (a, b) = (a.min(b), a.max(b));
                        let (c, d) = (c.min(d), c.max(d));
                        let x = $interval::new(a, b).unwrap();
                        let y = $interval::new(c, d).unwrap();
                        for divide in [false, true] {
                            if divide && c <= 0.0 && d >= 0.0 {
                                assert_eq!(x.div(&y), Err(IntervalError::ContainsZero));
                                continue;
                            }
                            let corners: Vec<_> = [(a, c), (a, d), (b, c), (b, d)]
                                .into_iter()
                                .map(|(u, v)| {
                                    let u = Rational::$from(u).unwrap();
                                    let v = Rational::$from(v).unwrap();
                                    if divide { u / v } else { u * v }
                                })
                                .collect();
                            let lo = expected(corners.iter().min().unwrap());
                            let hi = expected(corners.iter().max().unwrap());
                            let want = match (lo, hi) {
                                (Ok(a), Ok(b)) => Ok((a.0, b.1)),
                                _ => Err(IntervalError::Overflow),
                            };
                            assert_eq!(actual(if divide { x.div(&y) } else { x.mul(&y) }), want);
                        }
                    }
                }

                #[test]
                fn alternate_native_modes_do_not_change_bounds() {
                    const KEY: &str = "HYPERINTERVAL_MXCSR_MODE";
                    if let Ok(mode) = std::env::var(KEY) {
                        let mode: u32 = mode.parse().unwrap();
                        let tiny = <$float>::from_bits(1);
                        let small = <$float>::MIN_POSITIVE;
                        let cases = [
                            (1.0, 3.0),
                            (tiny, 3.0),
                            (small, 3.0),
                            (1.0, tiny),
                            (1.0, -tiny),
                            (-1.0, tiny),
                            (-1.0, -tiny),
                            (<$float>::MAX, tiny),
                            (<$float>::MAX, -tiny),
                            (tiny, <$float>::MAX),
                            (tiny, tiny),
                            (0.0, 0.0),
                            (-0.0, -tiny),
                        ];
                        // Exact independent expectations are prepared before
                        // altering this child process's native float environment.
                        let mut wanted = Vec::new();
                        for (a, b) in cases {
                            let x = Rational::$from(a).unwrap();
                            let y = Rational::$from(b).unwrap();
                            let ax = x.clone().abs();
                            let ay = y.clone().abs();
                            let absolute_lower = if (&x).min(&y) <= &0 && (&x).max(&y) >= &0 {
                                Rational::from(0)
                            } else {
                                (&ax).min(&ay).clone()
                            };
                            let absolute_upper = (&ax).max(&ay);
                            wanted.extend([
                                Ok((
                                    expected(&absolute_lower).unwrap().0,
                                    expected(absolute_upper).unwrap().1,
                                )),
                                expected(&x.clone().abs()),
                                expected((&x).min(&y)),
                                expected((&x).max(&y)),
                                expected(&Rational::from(&x + &y)),
                                expected(&Rational::from(&x - &y)),
                                expected(&Rational::from(&x * &y)),
                                if y == 0 {
                                    Err(IntervalError::ContainsZero)
                                } else {
                                    expected(&Rational::from(&x / &y))
                                },
                            ]);
                        }
                        let affine_cases = [
                            (<$float>::MAX, 2.0, -<$float>::MAX),
                            (1.0, tiny, 1.0),
                            (1.0, -tiny, 1.0),
                            (tiny, tiny, -tiny),
                            (small, 3.0, -small),
                            (tiny, 0.0, tiny),
                            (1.0, 1.0, -1.0),
                        ];
                        for (a, b, c) in affine_cases {
                            wanted.push(expected(
                                &(Rational::$from(a).unwrap() * Rational::$from(b).unwrap()
                                    + Rational::$from(c).unwrap()),
                            ));
                        }
                        for (a, b) in cases {
                            let lower = a.min(b);
                            let upper = a.max(b);
                            wanted.push(if upper < 0.0 {
                                Err(IntervalError::EmptyDomain)
                            } else {
                                Ok((
                                    crate::root_oracle::$root(lower.max(0.0)).unwrap().0,
                                    crate::root_oracle::$root(upper).unwrap().1,
                                ))
                            });
                        }
                        #[cfg(feature = "rational")]
                        let rational_points: Vec<_> = cases
                            .iter()
                            .map(|&(a, _)| $interval::point(a).unwrap().to_rational())
                            .collect();
                        #[cfg(feature = "dyadic")]
                        let dyadic_points: Vec<_> = cases
                            .iter()
                            .map(|&(a, _)| {
                                $interval::point(a)
                                    .unwrap()
                                    .to_dyadic(&mut hyperinterval::DyadicBudget::new(
                                        4096, 1_000_000,
                                    ))
                                    .unwrap()
                            })
                            .collect();
                        let got = crate::with_mxcsr(mode, || {
                            let mut got = Vec::new();
                            for (a, b) in cases {
                                let x = $interval::point(a).unwrap();
                                let y = $interval::point(b).unwrap();
                                got.extend([
                                    actual(Ok(x.hull(&y).abs())),
                                    actual(Ok(x.abs())),
                                    actual(Ok(x.min_image(&y))),
                                    actual(Ok(x.max_image(&y))),
                                    actual(x.add(&y)),
                                    actual(x.sub(&y)),
                                    actual(x.mul(&y)),
                                    actual(x.div(&y)),
                                ]);
                            }
                            for (a, b, c) in affine_cases {
                                got.push(actual($interval::point(a).unwrap().affine(b, c)));
                            }
                            for (a, b) in cases {
                                got.push(actual(
                                    $interval::point(a)
                                        .unwrap()
                                        .hull(&$interval::point(b).unwrap())
                                        .sqrt()
                                        .map(|x| x.range),
                                ));
                            }
                            #[cfg(feature = "rational")]
                            for (&(a, _), expected) in cases.iter().zip(&rational_points) {
                                assert_eq!(&$interval::point(a).unwrap().to_rational(), expected);
                            }
                            #[cfg(feature = "dyadic")]
                            for (&(a, _), expected) in cases.iter().zip(&dyadic_points) {
                                assert_eq!(
                                    &$interval::point(a)
                                        .unwrap()
                                        .to_dyadic(&mut hyperinterval::DyadicBudget::new(
                                            4096, 1_000_000
                                        ))
                                        .unwrap(),
                                    expected
                                );
                            }
                            assert!(!$interval::point(tiny).unwrap().contains_point(0.0).unwrap());
                            assert_ne!(
                                $interval::point(tiny).unwrap(),
                                $interval::point(0.0).unwrap()
                            );
                            assert_eq!(
                                $interval::new(tiny, 0.0),
                                Err(IntervalError::ReversedEndpoints)
                            );
                            got
                        });
                        assert_eq!(got, wanted, "MXCSR mode {mode}");
                        return;
                    }
                    for mode in 0..16 {
                        let output = std::process::Command::new(std::env::current_exe().unwrap())
                            .args([
                                "--exact",
                                concat!(
                                    stringify!($module),
                                    "::oracle::alternate_native_modes_do_not_change_bounds"
                                ),
                                "--nocapture",
                            ])
                            .env(KEY, mode.to_string())
                            .output()
                            .unwrap();
                        assert!(
                            output.status.success(),
                            "mode {mode}: {} {}",
                            String::from_utf8_lossy(&output.stdout),
                            String::from_utf8_lossy(&output.stderr)
                        );
                        eprint!("{}", String::from_utf8_lossy(&output.stderr));
                    }
                }
            }
        }
    };
}

cases!(
    binary32,
    F32Enclosure,
    f32,
    u32,
    from_f32,
    round_f32,
    sqrt_f32,
    23
);
cases!(
    binary64,
    F64Enclosure,
    f64,
    u64,
    from_f64,
    round_f64,
    sqrt_f64,
    52
);

#[cfg(all(target_arch = "x86_64", not(target_env = "msvc")))]
fn with_mxcsr<T>(mode: u32, work: impl FnOnce() -> T) -> T {
    struct Restore(u32);
    impl Drop for Restore {
        fn drop(&mut self) {
            // SAFETY: restore a value read from this thread's MXCSR register.
            unsafe {
                core::arch::asm!("ldmxcsr [{}]", in(reg) &self.0, options(nostack, preserves_flags));
            }
        }
    }
    let mut original = 0u32;
    // SAFETY: valid writable four-byte storage; this is an isolated child test.
    unsafe {
        core::arch::asm!("stmxcsr [{}]", in(reg) &mut original, options(nostack, preserves_flags));
    }
    let _restore = Restore(original);
    #[repr(align(16))]
    struct FxSave([u8; 512]);
    let mut saved = FxSave([0; 512]);
    // SAFETY: FXSAVE writes exactly 512 bytes to aligned writable storage.
    // It observes processor state without changing the active control modes.
    unsafe {
        core::arch::asm!("fxsave [{}]", in(reg) saved.0.as_mut_ptr(), options(nostack, preserves_flags));
    }
    let reported = u32::from_le_bytes(saved.0[28..32].try_into().unwrap());
    let supported = if reported == 0 { 0xffbf } else { reported };
    let flags =
        (original & !0xe040) | 0x1f80 | ((mode & 3) << 13) | ((mode & 4) << 13) | ((mode & 8) << 3);
    assert_eq!(
        flags & !supported,
        0,
        "requested MXCSR mode is unavailable on this CPU"
    );
    // SAFETY: reserved bits were checked against the processor's reported mask.
    // The original value is restored on unwind/return.
    unsafe {
        core::arch::asm!("ldmxcsr [{}]", in(reg) &flags, options(nostack, preserves_flags));
    }
    let mut active = 0u32;
    // SAFETY: writable four-byte storage, as for the original read above.
    unsafe {
        core::arch::asm!("stmxcsr [{}]", in(reg) &mut active, options(nostack, preserves_flags));
    }
    assert_eq!(active & 0xe040, flags & 0xe040);
    eprintln!("MXCSR mode={mode} supported={supported:08x} active={active:08x}");
    #[cfg(feature = "float-proposal")]
    {
        let a = std::hint::black_box(f64::from_bits(1));
        let b = std::hint::black_box(f32::from_bits(1));
        let p = hyperinterval::F64ProposalInterval::point(a).unwrap();
        let q = hyperinterval::F32ProposalInterval::point(b).unwrap();
        assert_eq!(p.estimates().unwrap().0.to_bits(), 1);
        assert_eq!(q.estimates().unwrap().0.to_bits(), 1);
        assert_ne!(p, hyperinterval::F64ProposalInterval::point(0.0).unwrap());
        assert_ne!(q, hyperinterval::F32ProposalInterval::point(0.0).unwrap());
        assert_eq!(
            hyperinterval::F64ProposalInterval::new(a, 0.0),
            Err(IntervalError::ReversedEndpoints)
        );
    }
    std::hint::black_box(work)()
}

#[cfg(all(feature = "rational", feature = "dyadic", target_arch = "x86_64"))]
#[test]
fn exact_bridges_preserve_ieee_values_and_generated_enclosures() {
    use hyperinterval::{DyadicBudget, RationalInterval};
    use hyperreal::Rational;
    let budget = || DyadicBudget::new(4096, 1_000_000_000);
    for value in [
        F64Enclosure::empty(),
        F64Enclosure::point(0.1).unwrap(),
        F64Enclosure::new(-f64::MAX, f64::from_bits(1)).unwrap(),
        F64Enclosure::new(1.0, 3.0)
            .unwrap()
            .reciprocal()
            .unwrap()
            .checked_pow(3)
            .unwrap(),
    ] {
        let rational = value.to_rational();
        assert_eq!(
            value
                .to_dyadic(&mut budget())
                .unwrap()
                .to_rational(&mut budget())
                .unwrap(),
            rational
        );
        if value.is_empty() {
            assert!(rational.is_empty());
        }
    }
    for value in [
        F32Enclosure::empty(),
        F32Enclosure::point(0.1).unwrap(),
        F32Enclosure::new(-f32::MAX, f32::from_bits(1)).unwrap(),
        F32Enclosure::new(-1.0, 3.0).unwrap().square().unwrap(),
    ] {
        assert_eq!(
            value
                .to_dyadic(&mut budget())
                .unwrap()
                .to_rational(&mut budget())
                .unwrap(),
            value.to_rational()
        );
    }
    assert_ne!(
        F64Enclosure::point(0.1).unwrap().to_rational(),
        RationalInterval::point(Rational::fraction(1, 10).unwrap())
    );
    assert_eq!(
        F64Enclosure::point(f64::MAX)
            .unwrap()
            .to_dyadic(&mut DyadicBudget::new(32, 1000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        F64Enclosure::point(1.0)
            .unwrap()
            .to_dyadic(&mut DyadicBudget::new(128, 0)),
        Err(IntervalError::WorkExhausted)
    );
    let x = F64Enclosure::new(-0.1, 0.3).unwrap();
    assert!(
        x.to_rational()
            .square()
            .subset_of(&x.square().unwrap().to_rational())
    );
}
