#![cfg(feature = "fixed")]
use hyperinterval::{FixedInterval, IntervalError};

#[test]
fn exact_storage_conversion_preserves_bounds_and_scale_validation() {
    type Narrow = FixedInterval<i32, 30>;
    let original = Narrow::new(i32::MIN, i32::MAX).unwrap();
    let wide = original.convert::<i64>().unwrap();
    assert_eq!(
        wide.bounds(),
        Some((i64::from(i32::MIN), i64::from(i32::MAX)))
    );
    assert_eq!(wide.convert::<i32>(), Ok(original));
    assert_eq!(
        wide.checked_add(&FixedInterval::<i64, 30>::point(1).unwrap())
            .unwrap()
            .convert::<i32>(),
        Err(IntervalError::Overflow)
    );
    assert_eq!(
        FixedInterval::<i64, 30>::empty().unwrap().convert::<i32>(),
        Ok(Narrow::empty().unwrap())
    );
    assert_eq!(
        FixedInterval::<i64, 63>::empty().unwrap().convert::<i32>(),
        Err(IntervalError::InvalidScale)
    );
    assert_eq!(Narrow::new(1, 0), Err(IntervalError::ReversedEndpoints));
}

#[test]
fn full_width_additive_images_preserve_order_and_checked_endpoints() {
    macro_rules! check {
        ($ty:ty, $frac:literal) => {{
            type I = FixedInterval<$ty, $frac>;
            let endpoints = [
                <$ty>::MIN,
                <$ty>::MIN + 1,
                -3,
                -1,
                0,
                1,
                3,
                <$ty>::MAX - 1,
                <$ty>::MAX,
            ];
            let expected =
                |lower: i128, upper: i128| match (<$ty>::try_from(lower), <$ty>::try_from(upper)) {
                    (Ok(lower), Ok(upper)) => Ok(Some((lower, upper))),
                    _ => Err(IntervalError::Overflow),
                };
            let empty = I::empty().unwrap();
            for &a in &endpoints {
                for &b in &endpoints {
                    if a > b {
                        continue;
                    }
                    let x = I::new(a, b).unwrap();
                    assert_eq!(
                        x.checked_neg().map(|x| x.bounds()),
                        expected(-i128::from(b), -i128::from(a))
                    );
                    assert_eq!(x.checked_add(&empty), Ok(empty));
                    assert_eq!(empty.checked_sub(&x), Ok(empty));
                    assert_eq!(x.checked_sub(&empty), Ok(empty));
                    for &c in &endpoints {
                        for &d in &endpoints {
                            if c > d {
                                continue;
                            }
                            let y = I::new(c, d).unwrap();
                            assert_eq!(
                                x.checked_add(&y).map(|x| x.bounds()),
                                expected(
                                    i128::from(a) + i128::from(c),
                                    i128::from(b) + i128::from(d)
                                )
                            );
                            assert_eq!(
                                x.checked_sub(&y).map(|x| x.bounds()),
                                expected(
                                    i128::from(a) - i128::from(d),
                                    i128::from(b) - i128::from(c)
                                )
                            );
                        }
                    }
                }
            }
        }};
    }
    check!(i32, 0);
    check!(i32, 30);
    check!(i32, 31);
    check!(i64, 0);
    check!(i64, 32);
    check!(i64, 63);
}

#[cfg(all(feature = "hyperreal", feature = "dispatch-trace"))]
#[test]
fn additive_reconstruction_and_replay_have_no_scalar_dispatch() {
    use hyperreal::dispatch_trace;
    type I = FixedInterval<i64, 63>;
    dispatch_trace::reset();
    let image = dispatch_trace::with_recording(|| {
        I::new(-7, 9)
            .unwrap()
            .checked_add(&I::new(3, 4).unwrap())
            .unwrap()
            .checked_sub(&I::new(-2, 3).unwrap())
            .unwrap()
            .checked_neg()
            .unwrap()
    });
    assert_eq!(image.bounds(), Some((-15, 7)));
    let (a, b) = image.bounds().unwrap();
    assert_eq!(I::new(a, b), Ok(image));
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
