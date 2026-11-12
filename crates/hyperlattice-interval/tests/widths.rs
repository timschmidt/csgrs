#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{IntervalArithmetic, IntervalVector};

fn qualify<I: IntervalArithmetic>(
    scalar: impl Fn(i32, i32) -> I,
    check: impl Fn(&I::Width, i32),
    mut context: impl FnMut() -> I::Context,
) {
    let intervals: Vec<_> = (-2..=2)
        .flat_map(|a| (a..=2).map(move |b| (a, b)))
        .collect();
    assert_eq!(intervals.len(), 15);
    for word in 0..3375 {
        let mut digits = word;
        let bounds = core::array::from_fn::<_, 3, _>(|_| {
            let pair = intervals[digits % 15];
            digits /= 15;
            pair
        });
        let vector = IntervalVector::new(bounds.map(|(a, b)| scalar(a, b))).unwrap();
        let widths = vector.widths(&mut context()).unwrap().unwrap();
        let mut maximum = 0;
        let mut index = 0;
        for lane in 0..3 {
            let expected = bounds[lane].1 - bounds[lane].0;
            check(&widths[lane], expected);
            if expected > maximum {
                maximum = expected;
                index = lane;
            }
        }
        assert_eq!(vector.widest_axis(&mut context()).unwrap(), Some(index));
    }
    let empty = IntervalVector::<I, 3>::empty().unwrap();
    assert!(empty.widths(&mut context()).unwrap().is_none());
    assert_eq!(empty.widest_axis(&mut context()).unwrap(), None);
    let unit = IntervalVector::<I, 0>::new([]).unwrap();
    assert_eq!(unit.widths(&mut context()).unwrap().unwrap().len(), 0);
    assert_eq!(unit.widest_axis(&mut context()).unwrap(), None);
}
#[cfg(feature = "fixed")]
#[test]
fn fixed_widths_and_ties() {
    use hyperlattice_interval::hyperinterval::FixedInterval as I;
    qualify(
        |a, b| I::<i32, 4>::new(a * 16, b * 16).unwrap(),
        |w, x| assert_eq!(w.numerator(), (x * 16) as u64),
        || (),
    );
    qualify(
        |a, b| I::<i64, 4>::new(i64::from(a) * 16, i64::from(b) * 16).unwrap(),
        |w, x| assert_eq!(w.numerator(), (x * 16) as u64),
        || (),
    );
}
#[cfg(feature = "rational")]
#[test]
fn rational_widths_and_ties() {
    use hyperlattice_interval::hyperinterval::RationalInterval as I;
    qualify(
        |a, b| I::new(a.into(), b.into()).unwrap(),
        |w, x| assert_eq!(w, &I::point(x.into())),
        || (),
    );
    let scalar = |a: &str, b: &str| I::new(a.parse().unwrap(), b.parse().unwrap()).unwrap();
    let source = IntervalVector::new([
        scalar("-1/3", "2/3"),
        scalar("1/7", "8/7"),
        scalar("-1/11", "12/11"),
    ])
    .unwrap();
    assert_eq!(source.widest_axis(&mut ()).unwrap(), Some(2));
    let widths = IntervalVector::new(source.widths(&mut ()).unwrap().unwrap()).unwrap();
    assert_eq!(
        widths.norm_one(&mut ()).unwrap(),
        I::point("35/11".parse().unwrap())
    );
    for bad in [I::empty(), scalar("0", "1")] {
        assert_eq!(
            I::compare_widths(&bad, &I::point(0.into()), &mut ()),
            Err(hyperlattice_interval::hyperinterval::IntervalError::NonSingletonEndpoint)
        );
        assert_eq!(
            I::compare_widths(&I::point(0.into()), &bad, &mut ()),
            Err(hyperlattice_interval::hyperinterval::IntervalError::NonSingletonEndpoint)
        );
    }
}
#[cfg(feature = "dyadic")]
#[test]
fn dyadic_widths_and_ties() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I};
    qualify(
        |a, b| I::new(a.into(), b.into(), 0).unwrap(),
        |w, x| assert_eq!(w, &I::point(x.into(), 0).unwrap()),
        || DyadicBudget::new(256, 1_000_000),
    );
}
#[cfg(feature = "float-enclosure")]
#[test]
fn float_widths_and_ties() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    qualify(
        |a, b| F32Enclosure::new(a as f32, b as f32).unwrap(),
        |w, x| {
            assert_eq!(
                w.enclosure().unwrap(),
                F32Enclosure::point(x as f32).unwrap()
            )
        },
        || (),
    );
    qualify(
        |a, b| F64Enclosure::new(a.into(), b.into()).unwrap(),
        |w, x| {
            assert_eq!(
                w.enclosure().unwrap(),
                F64Enclosure::point(x.into()).unwrap()
            )
        },
        || (),
    );
    macro_rules! case {
        ($i:ty,$f:ty) => {{
            let small = <$i>::new(<$f>::from_bits(2), 1.0).unwrap();
            let large = <$i>::new(<$f>::from_bits(1), 1.0).unwrap();
            assert_eq!(
                small.width().unwrap().enclosure().unwrap(),
                large.width().unwrap().enclosure().unwrap()
            );
            let source = IntervalVector::new([small, large]).unwrap();
            assert_eq!(source.widest_axis(&mut ()).unwrap(), Some(1));
            let source = IntervalVector::new([
                <$i>::new(0., <$f>::MAX).unwrap(),
                <$i>::new(-<$f>::MAX, <$f>::MAX).unwrap(),
            ])
            .unwrap();
            assert_eq!(source.widest_axis(&mut ()).unwrap(), Some(1));
            assert!(
                source.widths(&mut ()).unwrap().unwrap()[1]
                    .enclosure()
                    .is_err()
            );
        }};
    }
    case!(F32Enclosure, f32);
    case!(F64Enclosure, f64);
}
#[cfg(feature = "fixed")]
#[test]
fn full_range_widths_remain_ordered() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    type I = FixedInterval<i64, 63>;
    let x =
        IntervalVector::new([I::new(i64::MIN, i64::MAX).unwrap(), I::new(0, 1).unwrap()]).unwrap();
    assert_eq!(x.widest_axis(&mut ()).unwrap(), Some(0));
    let widths = x.widths(&mut ()).unwrap().unwrap();
    assert_eq!(widths[0].numerator(), u64::MAX);
    assert!(widths[0].to_interval().is_err());
    let restricted = x
        .intersection(
            &IntervalVector::new([I::point(0).unwrap(), I::new(0, 1).unwrap()]).unwrap(),
            &mut (),
        )
        .unwrap();
    assert_eq!(restricted.widest_axis(&mut ()).unwrap(), Some(1));
    assert_eq!(x.widest_axis(&mut ()).unwrap(), Some(0));
}
#[cfg(feature = "dyadic")]
#[test]
fn widths_and_selection_share_budget_and_retry() {
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let source = IntervalVector::new([
        I::new((-3).into(), 7.into(), 0).unwrap(),
        I::new(0.into(), 5.into(), 2).unwrap(),
        I::new(0.into(), 1.into(), 4).unwrap(),
    ])
    .unwrap();
    let mut budget = DyadicBudget::new(64, 10_000);
    assert_eq!(source.widest_axis(&mut budget).unwrap(), Some(1));
    let used = 10_000 - budget.remaining_work();
    assert_eq!(
        source.widest_axis(&mut DyadicBudget::new(64, used - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    let mut budget = DyadicBudget::new(64, used);
    assert_eq!(source.widest_axis(&mut budget).unwrap(), Some(1));
    assert_eq!(budget.remaining_work(), 0);
    assert_eq!(
        source.widths(&mut DyadicBudget::new(1, 10_000)),
        Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
    );
    let mut budget = DyadicBudget::new(64, 10_000);
    let widths = source.widths(&mut budget).unwrap().unwrap();
    let used = 10_000 - budget.remaining_work();
    assert_eq!(
        source.widths(&mut DyadicBudget::new(64, used - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert_eq!(
        source.widths(&mut DyadicBudget::new(64, used)).unwrap(),
        Some(widths.clone())
    );
    assert_eq!(
        IntervalVector::new(widths)
            .unwrap()
            .norm_one(&mut budget)
            .unwrap(),
        I::point(46.into(), 0).unwrap()
    );
    let extreme = IntervalVector::new([
        I::new(0.into(), 1.into(), i32::MIN).unwrap(),
        I::new(0.into(), 1.into(), i32::MAX).unwrap(),
    ])
    .unwrap();
    assert_eq!(
        extreme
            .widest_axis(&mut DyadicBudget::new(8, 1000))
            .unwrap(),
        Some(1)
    );
    assert!(
        IntervalVector::<I, 3>::empty()
            .unwrap()
            .widths(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_none()
    );
    assert_eq!(
        IntervalVector::<I, 0>::new([])
            .unwrap()
            .widest_axis(&mut DyadicBudget::new(0, 0))
            .unwrap(),
        None
    );
}
