#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{
    AffineMap, CenteredMatrix, CenteredVector, IntervalArithmetic, IntervalLatticeError,
    IntervalMatrix, IntervalVector,
};

fn corpus<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    for c in -8..=8 {
        for r in 0..=8 {
            for d in -3..=3 {
                for s in 0..=3 {
                    let center = IntervalVector::new([scalar(c, c), scalar(d, d)]).unwrap();
                    let radius = IntervalVector::new([scalar(r, r), scalar(s, s)]).unwrap();
                    let form = CenteredVector::new(center.clone(), radius.clone()).unwrap();
                    assert_eq!(form.center(), Some(&center));
                    assert_eq!(form.radius(), Some(&radius));
                    let bounds = form.enclosure(&mut context()).unwrap();
                    let expected =
                        IntervalVector::new([scalar(c - r, c + r), scalar(d - s, d + s)]).unwrap();
                    assert_eq!(bounds, expected);
                    assert_eq!(
                        bounds
                            .clone()
                            .midpoint_radius()
                            .midpoint_enclosure(&mut context())
                            .unwrap(),
                        center
                    );
                    assert_eq!(
                        bounds
                            .clone()
                            .midpoint_radius()
                            .radius_enclosure(&mut context())
                            .unwrap(),
                        radius
                    );
                    let point_map = AffineMap::new(
                        IntervalMatrix::new([[scalar(1, 1), scalar(-2, -2)]]).unwrap(),
                        IntervalVector::new([scalar(1, 1)]).unwrap(),
                    )
                    .unwrap();
                    let image = point_map
                        .image(bounds.clone())
                        .enclosure(&mut context())
                        .unwrap();
                    assert_eq!(
                        image,
                        IntervalVector::new([scalar(
                            c - 2 * d + 1 - r - 2 * s,
                            c - 2 * d + 1 + r + 2 * s
                        )])
                        .unwrap()
                    );
                    assert_eq!(
                        bounds.intersection(&expected, &mut context()).unwrap(),
                        expected
                    );
                    assert!(bounds.subset_of(&expected, &mut context()).unwrap());
                    assert_eq!(form.into_parts(), Some((center, radius)));

                    let center = IntervalMatrix::new([
                        [scalar(c, c), scalar(d, d), scalar(-c, -c)],
                        [scalar(d, d), scalar(-c, -c), scalar(c, c)],
                    ])
                    .unwrap();
                    let radius = IntervalMatrix::new([
                        [scalar(r, r), scalar(s, s), scalar(r + s, r + s)],
                        [scalar(s, s), scalar(r + s, r + s), scalar(r, r)],
                    ])
                    .unwrap();
                    let form = CenteredMatrix::new(center.clone(), radius.clone()).unwrap();
                    assert_eq!(form.center(), Some(&center));
                    assert_eq!(form.radius(), Some(&radius));
                    let bounds = form.enclosure(&mut context()).unwrap();
                    assert_eq!(
                        bounds.rows(),
                        Some(&[
                            [
                                scalar(c - r, c + r),
                                scalar(d - s, d + s),
                                scalar(-c - r - s, -c + r + s)
                            ],
                            [
                                scalar(d - s, d + s),
                                scalar(-c - r - s, -c + r + s),
                                scalar(c - r, c + r)
                            ],
                        ])
                    );
                    let weights =
                        IntervalVector::new([scalar(1, 1), scalar(-2, -2), scalar(1, 1)]).unwrap();
                    assert_eq!(
                        bounds.mul_vector(&weights, &mut context()).unwrap(),
                        IntervalVector::new([
                            scalar(-2 * d - 2 * r - 3 * s, -2 * d + 2 * r + 3 * s),
                            scalar(d + 3 * c - 3 * r - 3 * s, d + 3 * c + 3 * r + 3 * s),
                        ])
                        .unwrap()
                    );
                    assert_eq!(
                        bounds
                            .transpose(&mut context())
                            .unwrap()
                            .transpose(&mut context())
                            .unwrap(),
                        bounds
                    );
                    assert_eq!(form.into_parts(), Some((center, radius)));
                }
            }
        }
    }
    let zero = || IntervalVector::new([scalar(0, 0)]).unwrap();
    let empty = || IntervalVector::empty().unwrap();
    assert_eq!(
        CenteredVector::new(empty(), zero()),
        Err(IntervalLatticeError::NonPointCenter)
    );
    assert_eq!(
        CenteredVector::new(zero(), empty()),
        Err(IntervalLatticeError::NonPointRadius)
    );
    assert_eq!(
        CenteredVector::new(empty(), empty()),
        Err(IntervalLatticeError::NonPointCenter)
    );
    assert_eq!(
        CenteredVector::new(IntervalVector::new([scalar(0, 1)]).unwrap(), zero()),
        Err(IntervalLatticeError::NonPointCenter)
    );
    assert_eq!(
        CenteredVector::new(zero(), IntervalVector::new([scalar(0, 1)]).unwrap()),
        Err(IntervalLatticeError::NonPointRadius)
    );
    assert_eq!(
        CenteredVector::new(zero(), IntervalVector::new([scalar(-1, -1)]).unwrap()),
        Err(IntervalLatticeError::NegativeRadius)
    );
    let empty = CenteredVector::<I, 3>::empty().unwrap();
    assert!(empty.is_empty());
    assert!(empty.center().is_none() && empty.radius().is_none());
    assert!(empty.enclosure(&mut context()).unwrap().is_empty());
    assert_eq!(empty.into_parts(), None);
    let unit = CenteredVector::new(
        IntervalVector::<I, 0>::new([]).unwrap(),
        IntervalVector::new([]).unwrap(),
    )
    .unwrap();
    assert!(!unit.is_empty());
    assert!(unit.enclosure(&mut context()).unwrap().is_singleton());
    let unit = CenteredMatrix::new(
        IntervalMatrix::<I, 0, 3>::new([]).unwrap(),
        IntervalMatrix::new([]).unwrap(),
    )
    .unwrap();
    assert!(!unit.is_empty());
    assert!(unit.enclosure(&mut context()).unwrap().is_singleton());
    let unit = CenteredMatrix::new(
        IntervalMatrix::<I, 2, 0>::new([[], []]).unwrap(),
        IntervalMatrix::new([[], []]).unwrap(),
    )
    .unwrap();
    assert!(!unit.is_empty());
    assert!(unit.enclosure(&mut context()).unwrap().is_singleton());
    let empty = CenteredMatrix::<I, 0, 3>::empty().unwrap();
    assert!(empty.is_empty() && empty.center().is_none() && empty.radius().is_none());
    assert!(empty.enclosure(&mut context()).unwrap().is_empty());
    assert_eq!(empty.into_parts(), None);
}

#[cfg(feature = "fixed")]
#[test]
fn fixed_oracles() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    corpus(
        |a, b| FixedInterval::<i32, 8>::new(a << 8, b << 8).unwrap(),
        || (),
    );
    corpus(
        |a, b| FixedInterval::<i64, 32>::new(i64::from(a) << 32, i64::from(b) << 32).unwrap(),
        || (),
    );
}

#[cfg(feature = "rational")]
#[test]
fn rational_oracle() {
    use hyperlattice_interval::hyperinterval::RationalInterval;
    corpus(
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn dyadic_oracle_and_budget_retry() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    corpus(
        |a, b| I::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(128, 1_000_000),
    );
    let center = IntervalVector::<I, 0>::new([]).unwrap();
    let unit = CenteredVector::new(center.clone(), center).unwrap();
    assert!(
        unit.enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_singleton()
    );
    let center = IntervalVector::new([
        I::point(1.into(), -3).unwrap(),
        I::point(3.into(), -1).unwrap(),
    ])
    .unwrap();
    let radius = IntervalVector::new([
        I::point(1.into(), -4).unwrap(),
        I::point(5.into(), -2).unwrap(),
    ])
    .unwrap();
    let form = CenteredVector::new(center, radius).unwrap();
    let before = form.clone();
    assert_eq!(
        form.enclosure(&mut DyadicBudget::new(128, 0)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert_eq!(form, before);
    let mut ample = DyadicBudget::new(128, 10_000);
    let result = form.enclosure(&mut ample).unwrap();
    let used = 10_000 - ample.remaining_work();
    assert!(used > 0);
    for limit in 0..used {
        assert_eq!(
            form.enclosure(&mut DyadicBudget::new(128, limit)),
            Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
        );
        assert_eq!(form, before);
    }
    assert_eq!(
        form.enclosure(&mut DyadicBudget::new(128, used)).unwrap(),
        result
    );
    assert_eq!(
        result,
        IntervalVector::new([
            I::new(1.into(), 3.into(), -4).unwrap(),
            I::new(1.into(), 11.into(), -2).unwrap()
        ])
        .unwrap()
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
fn ieee_oracles_and_extremes() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};
    corpus(|a, b| F32Enclosure::new(a as f32, b as f32).unwrap(), || ());
    corpus(
        |a, b| F64Enclosure::new(f64::from(a), f64::from(b)).unwrap(),
        || (),
    );
    macro_rules! extremes {
        ($i:ty, $f:ty) => {{
            let p = |v| IntervalVector::new([<$i>::point(v).unwrap()]).unwrap();
            let form = CenteredVector::new(p(<$f>::MAX), p(<$f>::MAX)).unwrap();
            assert_eq!(
                form.enclosure(&mut ()),
                Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
            assert_eq!(form.into_parts(), Some((p(<$f>::MAX), p(<$f>::MAX))));
            let tiny = <$f>::from_bits(1);
            let form = CenteredVector::new(p(tiny), p(tiny)).unwrap();
            assert_eq!(
                form.enclosure(&mut ()).unwrap().lanes().unwrap()[0].bounds(),
                Some((0.0, <$f>::from_bits(2)))
            );
            let form = CenteredVector::new(p(1.0), p(tiny)).unwrap();
            assert_eq!(
                form.enclosure(&mut ()).unwrap().lanes().unwrap()[0].bounds(),
                Some((
                    <$f>::from_bits((1.0 as $f).to_bits() - 1),
                    <$f>::from_bits((1.0 as $f).to_bits() + 1)
                ))
            );
            assert!(
                CenteredVector::new(p(-0.0), p(-0.0))
                    .unwrap()
                    .enclosure(&mut ())
                    .unwrap()
                    .is_singleton()
            );
        }};
    }
    extremes!(F32Enclosure, f32);
    extremes!(F64Enclosure, f64);
}

#[cfg(feature = "fixed")]
#[test]
fn retained_overflow_and_invalid_scales() {
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    type I = FixedInterval<i32, 0>;
    macro_rules! extremes {
        ($endpoint:ty, $fraction:literal) => {{
            type F = FixedInterval<$endpoint, $fraction>;
            for c in [
                <$endpoint>::MIN,
                <$endpoint>::MIN + 1,
                -1,
                0,
                1,
                <$endpoint>::MAX - 1,
                <$endpoint>::MAX,
            ] {
                for r in [0, 1, <$endpoint>::MAX] {
                    let center = IntervalVector::new([F::point(c).unwrap()]).unwrap();
                    let radius = IntervalVector::new([F::point(r).unwrap()]).unwrap();
                    let form = CenteredVector::new(center, radius).unwrap();
                    let lower = i128::from(c) - i128::from(r);
                    let upper = i128::from(c) + i128::from(r);
                    let result = form.enclosure(&mut ());
                    if lower < i128::from(<$endpoint>::MIN) || upper > i128::from(<$endpoint>::MAX)
                    {
                        assert_eq!(
                            result,
                            Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
                        );
                    } else {
                        assert_eq!(
                            result.unwrap().lanes().unwrap()[0].bounds(),
                            Some((lower as $endpoint, upper as $endpoint))
                        );
                    }
                    assert_eq!(form.into_parts(), Some((center, radius)));
                }
            }
        }};
    }
    extremes!(i32, 0);
    extremes!(i32, 31);
    extremes!(i64, 0);
    extremes!(i64, 63);
    let p = |n| IntervalVector::new([I::point(n).unwrap()]).unwrap();
    let form = CenteredVector::new(p(i32::MAX), p(1)).unwrap();
    assert_eq!(
        form.enclosure(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    let (center, radius) = form.into_parts().unwrap();
    let promote = |v: IntervalVector<I, 1>| {
        IntervalVector::new(v.into_lanes().unwrap().map(|i| i.convert::<i64>().unwrap())).unwrap()
    };
    let wide = CenteredVector::new(promote(center), promote(radius)).unwrap();
    assert_eq!(
        wide.enclosure(&mut ()).unwrap().lanes().unwrap()[0].bounds(),
        Some((i64::from(i32::MAX) - 1, i64::from(i32::MAX) + 1))
    );
    assert_eq!(
        CenteredVector::<FixedInterval<i32, 32>, 0>::empty(),
        Err(IntervalLatticeError::Scalar(IntervalError::InvalidScale))
    );
    assert_eq!(
        CenteredMatrix::<FixedInterval<i64, 64>, 0, 0>::empty(),
        Err(IntervalLatticeError::Scalar(IntervalError::InvalidScale))
    );
}

#[test]
fn authored_validation_errors_remain_typed() {
    use std::error::Error;
    for (error, message) in [
        (
            IntervalLatticeError::NonPointCenter,
            "center must contain exactly one point",
        ),
        (
            IntervalLatticeError::NonPointRadius,
            "radius must contain exactly one point",
        ),
        (
            IntervalLatticeError::NegativeRadius,
            "radius must be nonnegative",
        ),
    ] {
        assert_eq!(error.to_string(), message);
        assert!(error.source().is_none());
    }
}
