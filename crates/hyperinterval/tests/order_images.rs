//! Independent point-image enumeration for the first elementary functions.
#[cfg(any(
    feature = "integer",
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    all(feature = "float-enclosure", target_arch = "x86_64")
))]
fn exercise<I: core::fmt::Debug + PartialEq>(
    make: impl Fn(Option<(i64, i64)>) -> I,
    absolute: impl Fn(&I) -> I,
    minimum: impl Fn(&I, &I) -> I,
    maximum: impl Fn(&I, &I) -> I,
    square: impl Fn(&I) -> I,
) {
    fn extrema(values: impl Iterator<Item = i64>) -> Option<(i64, i64)> {
        values.fold(None, |bounds, x| {
            Some(match bounds {
                None => (x, x),
                Some((a, b)) => (a.min(x), b.max(x)),
            })
        })
    }
    fn members(interval: Option<(i64, i64)>) -> Vec<i64> {
        interval.map_or_else(Vec::new, |(a, b)| (a..=b).collect())
    }
    let inputs: Vec<_> = core::iter::once(None)
        .chain((-4..=4).flat_map(|a| (a..=4).map(move |b| Some((a, b)))))
        .collect();
    for &a in &inputs {
        let x = make(a);
        let image = absolute(&x);
        assert_eq!(image, make(extrema(members(a).into_iter().map(i64::abs))));
        assert_eq!(absolute(&image), image);
        assert_eq!(square(&image), square(&x));
        for &b in &inputs {
            let y = make(b);
            let min = minimum(&x, &y);
            let max = maximum(&x, &y);
            let pairs: Vec<_> = members(a)
                .into_iter()
                .flat_map(|u| members(b).into_iter().map(move |v| (u, v)))
                .collect();
            assert_eq!(min, make(extrema(pairs.iter().map(|&(u, v)| u.min(v)))));
            assert_eq!(max, make(extrema(pairs.iter().map(|&(u, v)| u.max(v)))));
            assert_eq!(minimum(&y, &x), min);
            assert_eq!(maximum(&y, &x), max);
            // The interval images are exact because min/max are monotone in
            // both variables and absolute value has its sole minimum at zero.
            // Exercise generated intervals as further operands.
            assert_eq!(minimum(&min, &max), min);
            assert_eq!(maximum(&min, &max), max);
            assert_eq!(
                absolute(&min),
                make(extrema(pairs.iter().map(|&(u, v)| u.min(v).abs())))
            );
        }
    }
}

#[cfg(feature = "integer")]
#[test]
fn integer_images_match_enumerated_sets() {
    use hyperinterval::IntegerRange as I;
    exercise(
        |b| b.map_or_else(I::<i64>::empty, |(a, b)| I::new(a, b).unwrap()),
        |x| x.abs().unwrap(),
        |x, y| x.min_image(y),
        |x, y| x.max_image(y),
        |x| x.square().unwrap(),
    );
}

#[cfg(feature = "fixed")]
#[test]
fn fixed_images_match_enumerated_grid_extrema() {
    use hyperinterval::FixedInterval;
    type I = FixedInterval<i64, 4>;
    // Input/expected lattice numerators share a positive scale; extrema commute
    // with that scale. Square is used only for the exact dependency identity.
    exercise(
        |b| b.map_or_else(|| I::empty().unwrap(), |(a, b)| I::new(a, b).unwrap()),
        |x| x.abs().unwrap(),
        |x, y| x.min_image(y),
        |x, y| x.max_image(y),
        |x| x.square().unwrap(),
    );
}

#[cfg(feature = "rational")]
#[test]
fn rational_images_match_enumerated_fraction_extrema() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    exercise(
        |b| {
            b.map_or_else(I::empty, |(a, b)| {
                I::new(R::fraction(a, 7).unwrap(), R::fraction(b, 7).unwrap()).unwrap()
            })
        },
        I::abs,
        I::min_image,
        I::max_image,
        I::square,
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn dyadic_images_match_enumerated_grid_extrema() {
    use hyperinterval::{DyadicBudget, DyadicInterval as I};
    let budget = || DyadicBudget::new(4096, 1_000_000_000);
    exercise(
        |b| b.map_or_else(I::empty, |(a, b)| I::new(a.into(), b.into(), -3).unwrap()),
        |x| x.abs(&mut budget()).unwrap(),
        |x, y| x.min_image(y, &mut budget()).unwrap(),
        |x, y| x.max_image(y, &mut budget()).unwrap(),
        |x| x.square(&mut budget()).unwrap(),
    );
}

#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
#[test]
fn ieee_images_match_enumerated_exact_inputs() {
    use hyperinterval::{F32Enclosure, F64Enclosure};
    exercise(
        |b| {
            b.map_or_else(F32Enclosure::empty, |(a, b)| {
                F32Enclosure::new(a as f32 / 8.0, b as f32 / 8.0).unwrap()
            })
        },
        F32Enclosure::abs,
        F32Enclosure::min_image,
        F32Enclosure::max_image,
        |x| x.square().unwrap(),
    );
    exercise(
        |b| {
            b.map_or_else(F64Enclosure::empty, |(a, b)| {
                F64Enclosure::new(a as f64 / 8.0, b as f64 / 8.0).unwrap()
            })
        },
        F64Enclosure::abs,
        F64Enclosure::min_image,
        F64Enclosure::max_image,
        |x| x.square().unwrap(),
    );
}

#[cfg(feature = "integer")]
#[test]
fn integer_extrema_preserve_exact_images_or_fail_explicitly() {
    use hyperinterval::{IntegerRange as I, IntervalError};
    macro_rules! signed {
        ($($t:ty),*) => {$(
            let full = I::<$t>::new(<$t>::MIN, <$t>::MAX).unwrap();
            let low = I::<$t>::point(<$t>::MIN);
            let high = I::<$t>::point(<$t>::MAX);
            assert_eq!(full.abs(), Err(IntervalError::Overflow));
            assert_eq!(low.abs(), Err(IntervalError::Overflow));
            assert_eq!(high.abs().unwrap(), high);
            assert_eq!(full.min_image(&high), full);
            assert_eq!(full.max_image(&low), full);
            assert_eq!(low.min_image(&high), low);
            assert_eq!(low.max_image(&high), high);
        )*};
    }
    signed!(i8, i16, i32, i64, i128, isize);
    macro_rules! unsigned {
        ($($t:ty),*) => {$(
            let full = I::<$t>::new(0, <$t>::MAX).unwrap();
            assert_eq!(full.abs().unwrap(), full);
            assert_eq!(full.min_image(&I::point(0)), I::point(0));
            assert_eq!(full.max_image(&I::point(<$t>::MAX)), I::point(<$t>::MAX));
        )*};
    }
    unsigned!(u8, u16, u32, u64, u128, usize);
}

#[cfg(feature = "fixed")]
#[test]
fn fixed_absolute_extrema_never_saturate() {
    use hyperinterval::{FixedInterval, IntervalError};
    macro_rules! width {
        ($t:ty, $frac:literal) => {{
            type I = FixedInterval<$t, $frac>;
            let full = I::new(<$t>::MIN, <$t>::MAX).unwrap();
            assert_eq!(full.abs(), Err(IntervalError::Overflow));
            assert_eq!(
                I::point(<$t>::MIN).unwrap().abs(),
                Err(IntervalError::Overflow)
            );
            let safe = I::new(-<$t>::MAX, <$t>::MAX).unwrap();
            assert_eq!(safe.abs().unwrap(), I::new(0, <$t>::MAX).unwrap());
            assert_eq!(full.min_image(&I::point(<$t>::MAX).unwrap()), full);
            assert_eq!(full.max_image(&I::point(<$t>::MIN).unwrap()), full);
        }};
    }
    width!(i32, 0);
    width!(i32, 30);
    width!(i32, 31);
    width!(i64, 0);
    width!(i64, 32);
    width!(i64, 63);
}

#[cfg(feature = "dyadic")]
#[test]
fn dyadic_mixed_grids_and_budget_retry() {
    use hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let make = |a: i64, b: i64, e| I::new(a.into(), b.into(), e).unwrap();
    let budget = || DyadicBudget::new(4096, 1_000_000_000);
    let mut seed = 0x97c4_e831_201a_b563u64;
    for _ in 0..2_000 {
        let mut n = [0i64; 4];
        for x in &mut n {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            *x = (seed % 63) as i64 - 31;
        }
        let [a, b, c, d] = n;
        let (a, b, c, d) = (a.min(b), a.max(b), c.min(d), c.max(d));
        let e = (seed % 25) as i32 - 12;
        let f = ((seed >> 13) % 25) as i32 - 12;
        // Independent machine-integer lattice oracle; all scaled values fit i64.
        let grid = e.min(f);
        let (aa, bb, cc, dd) = (
            a << (e - grid),
            b << (e - grid),
            c << (f - grid),
            d << (f - grid),
        );
        let x = make(a, b, e);
        let y = make(c, d, f);
        assert_eq!(
            x.min_image(&y, &mut budget()).unwrap(),
            make(aa.min(cc), bb.min(dd), grid)
        );
        assert_eq!(
            x.max_image(&y, &mut budget()).unwrap(),
            make(aa.max(cc), bb.max(dd), grid)
        );
    }
    let x = make(-3, 7, -10);
    let y = make(-1, 3, 4);
    for maximum in [false, true] {
        let apply = |budget: &mut DyadicBudget| {
            if maximum {
                x.max_image(&y, budget)
            } else {
                x.min_image(&y, budget)
            }
        };
        assert_eq!(
            apply(&mut DyadicBudget::new(4096, 0)),
            Err(IntervalError::WorkExhausted)
        );
        assert_eq!(
            apply(&mut DyadicBudget::new(3, 1_000_000)),
            Err(IntervalError::SizeExhausted)
        );
        assert!(apply(&mut budget()).is_ok());
    }
    assert_eq!(
        x.abs(&mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        x.abs(&mut DyadicBudget::new(2, 1_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(x.abs(&mut budget()).unwrap(), make(0, 7, -10));
    for e in [i32::MIN, i32::MAX] {
        assert_eq!(make(-7, 3, e).abs(&mut budget()).unwrap(), make(0, 7, e));
    }
    let huge = make(1, 3, i32::MAX);
    let tiny = make(1, 3, i32::MIN);
    assert_eq!(
        huge.min_image(&tiny, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        huge.max_image(&tiny, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    let mut none = DyadicBudget::new(0, 0);
    assert!(huge.min_image(&I::empty(), &mut none).unwrap().is_empty());
    assert!(I::empty().max_image(&tiny, &mut none).unwrap().is_empty());
    assert!(I::empty().abs(&mut none).unwrap().is_empty());
    assert_eq!(none.remaining_work(), 0);
}

#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
#[test]
fn ieee_order_images_keep_extreme_finite_bits() {
    use hyperinterval::{F32Enclosure, F64Enclosure};
    macro_rules! width {
        ($i:ty,$f:ty) => {{
            let full = <$i>::new(-<$f>::MAX, <$f>::MAX).unwrap();
            assert_eq!(full.abs(), <$i>::new(0.0, <$f>::MAX).unwrap());
            let tiny = <$f>::from_bits(1);
            let negative = <$i>::new(-tiny, -0.0).unwrap();
            assert_eq!(negative.abs(), <$i>::new(0.0, tiny).unwrap());
            assert_eq!(
                full.min_image(&negative),
                <$i>::new(-<$f>::MAX, 0.0).unwrap()
            );
            assert_eq!(
                full.max_image(&negative),
                <$i>::new(-tiny, <$f>::MAX).unwrap()
            );
        }};
    }
    width!(F32Enclosure, f32);
    width!(F64Enclosure, f64);
}

#[cfg(all(
    feature = "serde",
    any(
        feature = "integer",
        feature = "fixed",
        feature = "rational",
        feature = "dyadic",
        all(feature = "float-enclosure", target_arch = "x86_64")
    )
))]
#[test]
fn generated_order_images_replay_and_remain_operands() {
    fn replay<I>(value: I) -> I
    where
        I: serde::Serialize + serde::de::DeserializeOwned + PartialEq + core::fmt::Debug,
    {
        let bytes = serde_json::to_vec(&value).unwrap();
        let restored = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(value, restored);
        assert_eq!(bytes, serde_json::to_vec(&restored).unwrap());
        restored
    }
    #[cfg(feature = "integer")]
    {
        use hyperinterval::IntegerRange as I;
        let x = I::<i128>::new(-17, 4).unwrap();
        let generated = x.abs().unwrap().min_image(&I::point(9));
        assert_eq!(replay(generated).checked_add(&x), generated.checked_add(&x));
    }
    #[cfg(feature = "fixed")]
    {
        use hyperinterval::FixedInterval;
        type I = FixedInterval<i64, 32>;
        let x = I::new(-17, 4).unwrap();
        let generated = x.abs().unwrap().min_image(&I::point(9).unwrap());
        assert_eq!(replay(generated).checked_add(&x), generated.checked_add(&x));
    }
    #[cfg(feature = "rational")]
    {
        use hyperinterval::RationalInterval as I;
        let x = I::new((-17).into(), 4.into()).unwrap();
        let generated = x.abs().min_image(&I::point(9.into()));
        assert_eq!(replay(generated.clone()).add(&x), generated.add(&x));
    }
    #[cfg(feature = "dyadic")]
    {
        use hyperinterval::{DyadicBudget, DyadicInterval as I};
        let mut work = DyadicBudget::new(4096, 1_000_000);
        let x = I::new((-17).into(), 4.into(), -3).unwrap();
        let generated = x
            .abs(&mut work)
            .unwrap()
            .min_image(&I::point(9.into(), -2).unwrap(), &mut work)
            .unwrap();
        assert_eq!(
            replay(generated.clone()).add(&x, &mut work),
            generated.add(&x, &mut work)
        );
    }
    #[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
    {
        macro_rules! width {
            ($i:ty) => {{
                let x = <$i>::new(-17.0, 4.0).unwrap();
                let generated = x.abs().min_image(&<$i>::point(9.0).unwrap());
                assert_eq!(replay(generated).add(&x), generated.add(&x));
            }};
        }
        width!(hyperinterval::F32Enclosure);
        width!(hyperinterval::F64Enclosure);
    }
}
