#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{
    IntervalArithmetic, IntervalMatrix, IntervalVector, MidpointRadiusMatrix, MidpointRadiusVector,
};

fn corpus<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    scalar: impl Fn(i32, i32) -> I,
    half: impl Fn(i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    let spans: Vec<_> = (-2..=2)
        .flat_map(|a| (a..=2).map(move |b| (a, b)))
        .collect();
    let weights = IntervalVector::new([scalar(1, 1), scalar(-1, -1), scalar(2, 2)]).unwrap();
    for &(a, b) in &spans {
        for &(c, d) in &spans {
            for &(e, f) in &spans {
                let source =
                    IntervalVector::new([scalar(a, b), scalar(c, d), scalar(e, f)]).unwrap();
                let form = source.clone().midpoint_radius();
                assert_eq!(form.intervals(), &source);
                let center = IntervalVector::new([half(a + b), half(c + d), half(e + f)]).unwrap();
                let radius = IntervalVector::new([half(b - a), half(d - c), half(f - e)]).unwrap();
                assert_eq!(form.midpoint_enclosure(&mut context()).unwrap(), center);
                assert_eq!(form.radius_enclosure(&mut context()).unwrap(), radius);
                let restored = form.into_intervals();
                assert_eq!(restored, source);
                assert_eq!(
                    restored.dot(&weights, &mut context()).unwrap(),
                    scalar(a - d + 2 * e, b - c + 2 * f)
                );
                let source = IntervalMatrix::new([
                    [scalar(a, b), scalar(c, d), scalar(e, f)],
                    [scalar(e, f), scalar(a, b), scalar(c, d)],
                ])
                .unwrap();
                let form = source.clone().midpoint_radius();
                assert_eq!(form.intervals(), &source);
                assert_eq!(
                    form.midpoint_enclosure(&mut context()).unwrap(),
                    IntervalMatrix::new([
                        [half(a + b), half(c + d), half(e + f)],
                        [half(e + f), half(a + b), half(c + d)]
                    ])
                    .unwrap()
                );
                assert_eq!(
                    form.radius_enclosure(&mut context()).unwrap(),
                    IntervalMatrix::new([
                        [half(b - a), half(d - c), half(f - e)],
                        [half(f - e), half(b - a), half(d - c)]
                    ])
                    .unwrap()
                );
                let restored = form.into_intervals();
                assert_eq!(restored, source);
                assert_eq!(
                    restored.mul_vector(&weights, &mut context()).unwrap(),
                    IntervalVector::new([
                        scalar(a - d + 2 * e, b - c + 2 * f),
                        scalar(e - b + 2 * c, f - a + 2 * d)
                    ])
                    .unwrap()
                );
            }
        }
    }
    let empty = IntervalVector::<I, 3>::empty().unwrap().midpoint_radius();
    assert!(empty.is_empty());
    assert!(empty.midpoint_enclosure(&mut context()).unwrap().is_empty());
    assert!(empty.radius_enclosure(&mut context()).unwrap().is_empty());
    let unit = IntervalVector::<I, 0>::new([]).unwrap().midpoint_radius();
    assert!(!unit.is_empty());
    assert!(
        unit.midpoint_enclosure(&mut context())
            .unwrap()
            .is_singleton()
    );
    assert!(
        unit.radius_enclosure(&mut context())
            .unwrap()
            .is_singleton()
    );
    let empty = IntervalMatrix::<I, 0, 3>::empty()
        .unwrap()
        .midpoint_radius();
    assert!(empty.is_empty());
    assert!(empty.midpoint_enclosure(&mut context()).unwrap().is_empty());
    assert!(empty.radius_enclosure(&mut context()).unwrap().is_empty());
    let unit = IntervalMatrix::<I, 0, 3>::new([])
        .unwrap()
        .midpoint_radius();
    assert!(!unit.is_empty());
    assert!(
        unit.midpoint_enclosure(&mut context())
            .unwrap()
            .is_singleton()
    );
    assert!(
        unit.radius_enclosure(&mut context())
            .unwrap()
            .is_singleton()
    );
    let unit = IntervalMatrix::<I, 2, 0>::new([[], []])
        .unwrap()
        .midpoint_radius();
    assert!(!unit.is_empty());
    assert!(
        unit.midpoint_enclosure(&mut context())
            .unwrap()
            .is_singleton()
    );
    assert!(
        unit.radius_enclosure(&mut context())
            .unwrap()
            .is_singleton()
    );
    assert_eq!(
        core::mem::size_of::<MidpointRadiusVector<I, 3>>(),
        core::mem::size_of::<IntervalVector<I, 3>>()
    );
    assert_eq!(
        core::mem::size_of::<MidpointRadiusMatrix<I, 2, 3>>(),
        core::mem::size_of::<IntervalMatrix<I, 2, 3>>()
    );
}

#[cfg(feature = "fixed")]
#[test]
fn fixed_decomposition_and_reconstruction() {
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    macro_rules! family {
        ($i:ty) => {{
            let scalar = |a, b| <$i>::new(a as _, b as _).unwrap();
            corpus(
                scalar,
                |n| scalar(n.div_euclid(2), n.div_euclid(2) + i32::from(n % 2 != 0)),
                || (),
            );
        }};
    }
    family!(FixedInterval<i32,0>);
    family!(FixedInterval<i64,0>);
    type I = FixedInterval<i64, 63>;
    let source = IntervalVector::new([I::new(i64::MIN, i64::MAX).unwrap()]).unwrap();
    let form = source.midpoint_radius();
    assert_eq!(
        form.radius_enclosure(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    assert_eq!(
        form.midpoint_enclosure(&mut ()).unwrap(),
        IntervalVector::new([I::new(-1, 0).unwrap()]).unwrap()
    );
    let restored = form.into_intervals();
    assert_eq!(restored, source);
    assert_eq!(
        restored.widths(&mut ()).unwrap().unwrap()[0].numerator(),
        u64::MAX
    );
}

#[cfg(feature = "rational")]
#[test]
fn rational_decomposition_and_reconstruction() {
    use hyperlattice_interval::hyperinterval::RationalInterval as I;
    // Integer numerator divided by exact two, independently constructed.
    corpus(
        |a, b| I::new(a.into(), b.into()).unwrap(),
        |n| I::point(n.into()).div(&I::point(2.into())).unwrap(),
        || (),
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn dyadic_decomposition_and_reconstruction() {
    use hyperlattice_interval::IntervalLatticeError;
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let budget = || DyadicBudget::new(256, 1_000_000);
    corpus(
        |a, b| I::new(a.into(), b.into(), 0).unwrap(),
        |n| I::point(n.into(), -1).unwrap(),
        budget,
    );
    let source = IntervalMatrix::new([
        [
            I::new(1.into(), 2.into(), i32::MIN).unwrap(),
            I::new(3.into(), 4.into(), i32::MIN).unwrap(),
        ],
        [
            I::new((-2).into(), (-1).into(), i32::MIN).unwrap(),
            I::new((-3).into(), 7.into(), -5).unwrap(),
        ],
    ])
    .unwrap();
    let form = source.clone().midpoint_radius();
    for radius in [false, true] {
        let project = |context: &mut DyadicBudget| {
            if radius {
                form.radius_enclosure(context)
            } else {
                form.midpoint_enclosure(context)
            }
        };
        let mut context = budget();
        let expected = project(&mut context).unwrap();
        let used = 1_000_000 - context.remaining_work();
        assert_eq!(
            project(&mut DyadicBudget::new(256, used - 1)),
            Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
        );
        assert_eq!(
            project(&mut DyadicBudget::new(256, used)).unwrap(),
            expected
        );
        assert_eq!(
            project(&mut DyadicBudget::new(1, 1_000_000)),
            Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
        );
        assert_eq!(form.intervals(), &source);
    }
    assert_eq!(form.into_intervals(), source);
    let empty = IntervalVector::<I, 2>::empty().unwrap().midpoint_radius();
    assert!(
        empty
            .midpoint_enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
    assert!(
        empty
            .radius_enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
    let unit = IntervalVector::<I, 0>::new([]).unwrap().midpoint_radius();
    assert!(
        unit.midpoint_enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_singleton()
    );
    assert!(
        unit.radius_enclosure(&mut DyadicBudget::new(0, 0))
            .unwrap()
            .is_singleton()
    );
}

#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
#[test]
fn float_decomposition_and_reconstruction() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    macro_rules! family {
        ($i:ty,$f:ty) => {{
            corpus(
                |a, b| <$i>::new(a as $f, b as $f).unwrap(),
                |n| <$i>::point((n as $f) * 0.5).unwrap(),
                || (),
            );
            let tiny = <$f>::from_bits(1);
            let source = IntervalVector::new([
                <$i>::new(0., tiny).unwrap(),
                <$i>::new(-<$f>::MAX, <$f>::MAX).unwrap(),
            ])
            .unwrap();
            let form = source.midpoint_radius();
            assert!(!form.midpoint_enclosure(&mut ()).unwrap().is_singleton());
            assert_eq!(
                form.radius_enclosure(&mut ()).unwrap().lanes().unwrap()[1],
                <$i>::point(<$f>::MAX).unwrap()
            );
            assert_eq!(form.into_intervals(), source);
        }};
    }
    family!(F32Enclosure, f32);
    family!(F64Enclosure, f64);
}

#[cfg(all(feature = "fixed", feature = "serde"))]
#[test]
fn replay_checks_shape_scale_and_source_order() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    type I = FixedInterval<i64, 63>;
    let source = IntervalVector::new([I::new(0, 1).unwrap(); 33]).unwrap();
    let text = serde_json::to_string(&source.midpoint_radius()).unwrap();
    assert_eq!(text, serde_json::to_string(&source).unwrap());
    let replay: MidpointRadiusVector<I, 33> = serde_json::from_str(&text).unwrap();
    assert_eq!(replay.into_intervals(), source);
    type V = MidpointRadiusVector<I, 1>;
    for bad in ["[]", "[[0,1],[0,1]]", "[[1,0]]", "[[0]]", "[true]"] {
        assert!(serde_json::from_str::<V>(bad).is_err(), "{bad}");
    }
    assert!(serde_json::from_str::<MidpointRadiusVector<FixedInterval<i64, 64>, 0>>("[]").is_err());
    assert!(
        serde_json::from_str::<MidpointRadiusVector<FixedInterval<i64, 64>, 0>>("null").is_err()
    );
    type M = MidpointRadiusMatrix<I, 2, 1>;
    for bad in [
        "[]",
        "[[[0,1]]]",
        "[[[0,1]],[[1,0]]]",
        "[[[0,1],[0,1]],[[0,1]]]",
    ] {
        assert!(serde_json::from_str::<M>(bad).is_err(), "{bad}");
    }
    let empty: V = serde_json::from_str("[null]").unwrap();
    assert!(empty.is_empty());
    assert_eq!(serde_json::to_string(&empty).unwrap(), "null");
}
