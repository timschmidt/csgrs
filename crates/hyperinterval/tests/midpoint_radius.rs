#[cfg(all(
    feature = "float-enclosure",
    target_arch = "x86_64",
    not(target_env = "msvc")
))]
#[path = "support/float_oracle.rs"]
mod float_oracle;
#[cfg(all(
    feature = "float-enclosure",
    target_arch = "x86_64",
    not(target_env = "msvc")
))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;

#[cfg(feature = "fixed")]
#[test]
fn fixed_half_steps_extremes_and_tightness() {
    use hyperinterval::{FixedInterval, IntervalError};
    macro_rules! family {
        ($ty:ty, $frac:literal) => {{
            type I = FixedInterval<$ty, $frac>;
            for a in -32..=32 {
                for b in a..=32 {
                    let x = I::new(a, b).unwrap();
                    for (got, numerator) in [
                        (
                            x.midpoint_enclosure().unwrap(),
                            i128::from(a) + i128::from(b),
                        ),
                        (x.radius_enclosure().unwrap(), i128::from(b) - i128::from(a)),
                    ] {
                        let (lo, hi) = got.bounds().unwrap();
                        let (lo, hi) = (i128::from(lo), i128::from(hi));
                        assert!(2 * lo <= numerator && numerator <= 2 * hi);
                        assert!(2 * (lo + 1) > numerator && 2 * (hi - 1) < numerator);
                    }
                }
            }
            let full = I::new(<$ty>::MIN, <$ty>::MAX).unwrap();
            assert_eq!(full.midpoint_enclosure().unwrap(), I::new(-1, 0).unwrap());
            assert_eq!(full.radius_enclosure(), Err(IntervalError::Overflow));
            for value in [<$ty>::MIN, 0, <$ty>::MAX] {
                let x = I::point(value).unwrap();
                assert_eq!(x.midpoint_enclosure().unwrap(), x);
                assert_eq!(x.radius_enclosure().unwrap(), I::point(0).unwrap());
            }
            let empty = I::empty().unwrap();
            assert_eq!(empty.midpoint_enclosure().unwrap(), empty);
            assert_eq!(empty.radius_enclosure().unwrap(), empty);
        }};
    }
    family!(i32, 0);
    family!(i32, 31);
    family!(i64, 0);
    family!(i64, 63);
}

#[cfg(feature = "rational")]
#[test]
fn rational_parts_reconstruct_exact_endpoints() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational;
    let q = |n, d| Rational::fraction(n, d).unwrap();
    let source = I::new(q(1, 3), q(7, 5)).unwrap();
    let center = source.midpoint_enclosure();
    let radius = source.radius_enclosure();
    assert_eq!(center, I::point(q(13, 15)));
    assert_eq!(radius, I::point(q(8, 15)));
    assert_eq!(center.sub(&radius), I::point(q(1, 3)));
    assert_eq!(center.add(&radius), I::point(q(7, 5)));
    assert_eq!(I::empty().midpoint_enclosure(), I::empty());
    assert_eq!(I::empty().radius_enclosure(), I::empty());
}

#[cfg(feature = "dyadic")]
#[test]
fn dyadic_projection_preserves_halves_or_outward_minimum_grid() {
    use hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let budget = || DyadicBudget::new(256, 1_000_000);
    for exponent in [-8, 0, 7] {
        for a in -8..=8 {
            for b in a..=8 {
                let source = I::new(a.into(), b.into(), exponent).unwrap();
                assert_eq!(
                    source.midpoint_enclosure(&mut budget()).unwrap(),
                    I::point((a + b).into(), exponent - 1).unwrap()
                );
                assert_eq!(
                    source.radius_enclosure(&mut budget()).unwrap(),
                    I::point((b - a).into(), exponent - 1).unwrap()
                );
            }
        }
    }
    for (a, b) in [(1, 2), (-2, -1), (-2, 3)] {
        let source = I::new(a.into(), b.into(), i32::MIN).unwrap();
        assert_eq!(source.midpoint(&mut budget()), Err(IntervalError::Overflow));
        assert_eq!(source.radius(&mut budget()), Err(IntervalError::Overflow));
        for (got, numerator) in [
            (source.midpoint_enclosure(&mut budget()).unwrap(), a + b),
            (source.radius_enclosure(&mut budget()).unwrap(), b - a),
        ] {
            let lo = i32::div_euclid(numerator, 2);
            assert_eq!(got, I::new(lo.into(), (lo + 1).into(), i32::MIN).unwrap());
        }
        for radius in [false, true] {
            let project = |budget: &mut DyadicBudget| {
                if radius {
                    source.radius_enclosure(budget)
                } else {
                    source.midpoint_enclosure(budget)
                }
            };
            let mut context = budget();
            let expected = project(&mut context).unwrap();
            let used = 1_000_000 - context.remaining_work();
            assert_eq!(
                project(&mut DyadicBudget::new(256, used - 1)),
                Err(IntervalError::WorkExhausted)
            );
            assert_eq!(
                project(&mut DyadicBudget::new(256, used)).unwrap(),
                expected
            );
            assert_eq!(
                project(&mut DyadicBudget::new(1, 1_000_000)),
                Err(IntervalError::SizeExhausted)
            );
        }
    }
    let empty = I::empty();
    assert_eq!(
        empty
            .midpoint_enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap(),
        empty
    );
    assert_eq!(
        empty
            .radius_enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap(),
        empty
    );
    let extreme = I::new(1.into(), 2.into(), i32::MAX).unwrap();
    assert_eq!(
        extreme.midpoint_enclosure(&mut budget()).unwrap(),
        I::point(3.into(), i32::MAX - 1).unwrap()
    );
    assert_eq!(
        extreme.radius_enclosure(&mut budget()).unwrap(),
        I::point(1.into(), i32::MAX - 1).unwrap()
    );
}

#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
macro_rules! float_cases {
    ($test:ident, $i:ty, $f:ty, $bits:ty, $infinity:expr, $fraction:expr, $from:ident, $round:ident) => {
        #[test]
        fn $test() {
            type I = $i;
            let tiny = <$f>::from_bits(1);
            let full = I::new(-<$f>::MAX, <$f>::MAX).unwrap();
            assert_eq!(full.midpoint_enclosure().unwrap(), I::point(0.).unwrap());
            assert_eq!(
                full.radius_enclosure().unwrap(),
                I::point(<$f>::MAX).unwrap()
            );
            let max = I::point(<$f>::MAX).unwrap();
            assert_eq!(max.midpoint_enclosure().unwrap(), max);
            assert_eq!(max.radius_enclosure().unwrap(), I::point(0.).unwrap());
            for (a, b) in [(0., tiny), (-tiny, 0.)] {
                let source = I::new(a, b).unwrap();
                assert_eq!(source.midpoint_enclosure().unwrap(), source);
                assert_eq!(
                    source.radius_enclosure().unwrap(),
                    I::new(0., tiny).unwrap()
                );
            }
            assert!(I::empty().midpoint_enclosure().unwrap().is_empty());
            assert!(I::empty().radius_enclosure().unwrap().is_empty());
            #[cfg(not(target_env = "msvc"))]
            {
                use rug::Rational;
                let check = |mut a: $f, mut b: $f| {
                    if a.total_cmp(&b).is_gt() {
                        core::mem::swap(&mut a, &mut b);
                    }
                    let source = I::new(a, b).unwrap();
                    let qa = Rational::$from(a).unwrap();
                    let qb = Rational::$from(b).unwrap();
                    let center = (qa.clone() + &qb) / 2;
                    let radius = (qb - qa) / 2;
                    for (got, expected) in [
                        (source.midpoint_enclosure().unwrap(), center),
                        (source.radius_enclosure().unwrap(), radius),
                    ] {
                        let (lo, hi) = got.bounds().unwrap();
                        assert_eq!(
                            (lo.to_bits(), hi.to_bits()),
                            float_oracle::$round(&expected).unwrap(),
                            "source={source:?} exact={expected}"
                        );
                    }
                };
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
                for a in edges {
                    for b in edges {
                        check(a, b);
                    }
                }
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
                for _ in 0..10_000 {
                    check(next(), next());
                }
            }
        }
    };
}
#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
float_cases!(
    binary32_halves,
    hyperinterval::F32Enclosure,
    f32,
    u32,
    0x7f80_0000,
    23,
    from_f32,
    round_f32
);
#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
float_cases!(
    binary64_halves,
    hyperinterval::F64Enclosure,
    f64,
    u64,
    0x7ff0_0000_0000_0000_u64,
    52,
    from_f64,
    round_f64
);
