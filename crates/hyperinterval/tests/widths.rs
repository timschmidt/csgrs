#[cfg(feature = "fixed")]
#[test]
fn fixed_widths_preserve_unsigned_range_and_scale() {
    use hyperinterval::{FixedInterval as I, FixedWidth, IntervalError};
    let full = I::<i64, 63>::new(i64::MIN, i64::MAX)
        .unwrap()
        .width()
        .unwrap();
    assert_eq!(full.numerator(), u64::MAX);
    assert_eq!(full.fraction_bits(), 63);
    assert_eq!(full.to_interval(), Err(IntervalError::Overflow));
    assert_eq!(
        I::<i32, 31>::new(i32::MIN, i32::MAX)
            .unwrap()
            .width()
            .unwrap()
            .numerator(),
        u64::from(u32::MAX)
    );
    for a in -32..=32 {
        for b in a..=32 {
            let x = I::<i32, 7>::new(a, b).unwrap();
            let w = x.width().unwrap();
            assert_eq!(w.numerator(), (b - a) as u64);
            assert_eq!(
                w.to_interval().unwrap(),
                I::<i64, 7>::point((b - a).into()).unwrap()
            );
            assert_eq!(w, FixedWidth::new((b - a) as u64).unwrap());
        }
    }
    assert!(I::<i64, 0>::empty().unwrap().width().is_none());
    assert_eq!(FixedWidth::<64>::new(0), Err(IntervalError::InvalidScale));
    #[cfg(feature = "serde")]
    {
        let text = serde_json::to_string(&full).unwrap();
        assert_eq!(serde_json::from_str::<FixedWidth<63>>(&text).unwrap(), full);
        for bad in ["-1", "null", "1.5", "18446744073709551616"] {
            assert!(serde_json::from_str::<FixedWidth<0>>(bad).is_err());
        }
        assert!(serde_json::from_str::<FixedWidth<64>>("0").is_err());
    }
}

#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
macro_rules! float_cases {
    ($test:ident,$i:ty,$w:ty,$f:ty,$bits:ty,$infinity:expr,$fraction:expr,$from:ident) => {
        #[test]
        fn $test() {
            use core::cmp::Ordering;
            use hyperinterval::IntervalError;
            let width = |a, b| <$i>::new(a, b).unwrap().width().unwrap();
            let tiny = <$f>::from_bits(1);
            let full = width(-<$f>::MAX, <$f>::MAX);
            assert!(full > width(0., <$f>::MAX));
            assert_eq!(full.enclosure(), Err(IntervalError::Overflow));
            let a = width(tiny, 1.);
            let b = width(<$f>::from_bits(2), 1.);
            assert_eq!(a.enclosure().unwrap(), b.enclosure().unwrap());
            assert!(a > b);
            assert_eq!(width(-1., 0.), width(0., 1.));
            assert_eq!(
                width(-1., 0.).partial_cmp(&width(0., 1.)),
                Some(Ordering::Equal)
            );
            assert!(<$i>::empty().width().is_none());
            assert_eq!(
                width(-tiny, tiny).enclosure().unwrap(),
                <$i>::point(<$f>::from_bits(2)).unwrap()
            );
            assert_eq!(core::mem::size_of::<$w>(), 2 * core::mem::size_of::<$f>());
            #[cfg(feature = "serde")]
            {
                let text = serde_json::to_string(&full).unwrap();
                let replay: $w = serde_json::from_str(&text).unwrap();
                assert_eq!(replay, full);
                assert_eq!(serde_json::to_string(&replay).unwrap(), text);
                assert_eq!(replay.defining_interval(), full.defining_interval());
                let a = width(-1., 0.);
                let b = width(0., 1.);
                assert_eq!(a, b);
                assert_ne!(
                    serde_json::to_string(&a).unwrap(),
                    serde_json::to_string(&b).unwrap()
                );
                for bad in ["null", "[]", "[0]", "[0,0,0]"] {
                    assert!(serde_json::from_str::<$w>(bad).is_err());
                }
                let nonfinite =
                    serde_json::to_string(&[$infinity as $bits, $infinity as $bits]).unwrap();
                assert!(serde_json::from_str::<$w>(&nonfinite).is_err());
                let reversed = serde_json::to_string(&[(1.0 as $f).to_bits(), 0 as $bits]).unwrap();
                assert!(serde_json::from_str::<$w>(&reversed).is_err());
            }
            #[cfg(not(target_env = "msvc"))]
            {
                use rug::Rational;
                let mut state = 0x9e3779b97f4a7c15_u64;
                let mut next = || {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    let mut bits = state as $bits;
                    if bits & ($infinity as $bits) == ($infinity as $bits) {
                        bits ^= 1 << $fraction;
                    }
                    <$f>::from_bits(bits)
                };
                for index in 0..10_000 {
                    let mut points = [next(), next(), next(), next()];
                    if points[0].total_cmp(&points[1]).is_gt() {
                        points.swap(0, 1);
                    }
                    if points[2].total_cmp(&points[3]).is_gt() {
                        points.swap(2, 3);
                    }
                    let [a, b, c, d] = points;
                    let expected = (Rational::$from(b).unwrap() - Rational::$from(a).unwrap())
                        .cmp(&(Rational::$from(d).unwrap() - Rational::$from(c).unwrap()));
                    assert_eq!(
                        width(a, b).cmp(&width(c, d)),
                        expected,
                        "seed=9e3779b97f4a7c15 index={index} points={points:?}"
                    );
                }
                let edges = [
                    -<$f>::MAX,
                    -1.,
                    -<$f>::MIN_POSITIVE,
                    -tiny,
                    0.,
                    tiny,
                    <$f>::MIN_POSITIVE,
                    1.,
                    <$f>::MAX,
                ];
                for (i, &a) in edges.iter().enumerate() {
                    for &b in &edges[i..] {
                        for (j, &c) in edges.iter().enumerate() {
                            for &d in &edges[j..] {
                                let expected = (Rational::$from(b).unwrap()
                                    - Rational::$from(a).unwrap())
                                .cmp(&(Rational::$from(d).unwrap() - Rational::$from(c).unwrap()));
                                assert_eq!(width(a, b).cmp(&width(c, d)), expected);
                            }
                        }
                    }
                }
            }
        }
    };
}
#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
float_cases!(
    binary32_exact_width_order,
    hyperinterval::F32Enclosure,
    hyperinterval::F32Width,
    f32,
    u32,
    0x7f80_0000_u32,
    23,
    from_f32
);
#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
float_cases!(
    binary64_exact_width_order,
    hyperinterval::F64Enclosure,
    hyperinterval::F64Width,
    f64,
    u64,
    0x7ff0_0000_0000_0000_u64,
    52,
    from_f64
);

#[cfg(feature = "dyadic")]
#[test]
fn singleton_order_shares_budget_and_does_not_expand_exponent_gaps() {
    use core::cmp::Ordering;
    use hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let p = |m: i32, e| I::point(m.into(), e).unwrap();
    assert_eq!(
        p(1, i32::MAX)
            .compare_singleton(&p(1, i32::MIN), &mut DyadicBudget::new(8, 100))
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        p(-1, i32::MAX)
            .compare_singleton(&p(-1, i32::MIN), &mut DyadicBudget::new(8, 100))
            .unwrap(),
        Ordering::Less
    );
    let a = p(3, 4);
    let b = p(5, 3);
    let mut budget = DyadicBudget::new(32, 1000);
    assert_eq!(
        a.compare_singleton(&b, &mut budget).unwrap(),
        Ordering::Greater
    );
    let used = 1000 - budget.remaining_work();
    assert_eq!(
        a.compare_singleton(&b, &mut DyadicBudget::new(32, used - 1)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        a.compare_singleton(&b, &mut DyadicBudget::new(32, used))
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        a.compare_singleton(&b, &mut DyadicBudget::new(1, 1000)),
        Err(IntervalError::SizeExhausted)
    );
    for invalid in [I::empty(), I::new(0.into(), 1.into(), 0).unwrap()] {
        assert_eq!(
            invalid.compare_singleton(&a, &mut DyadicBudget::new(32, 1000)),
            Err(IntervalError::NonSingletonEndpoint)
        );
        assert_eq!(
            a.compare_singleton(&invalid, &mut DyadicBudget::new(32, 1000)),
            Err(IntervalError::NonSingletonEndpoint)
        );
    }
}
