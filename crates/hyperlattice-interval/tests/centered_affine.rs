#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{
    AffineChain, AffineMap, CenteredVector, IntervalArithmetic, IntervalLatticeError,
    IntervalMatrix, IntervalVector,
};

#[cfg(all(feature = "fixed", feature = "serde"))]
#[test]
fn translated_replay_revalidates_and_preserves_cancellation() {
    use hyperlattice_interval::{TranslatedBox, hyperinterval::FixedInterval};
    type I = FixedInterval<i64, 0>;
    const KEY: &str = "HYPERLATTICE_TRANSLATED_REPLAY";
    let p = |v| I::point(v).unwrap();
    if let Ok(text) = std::env::var(KEY) {
        let source: TranslatedBox<I, 1> = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&source).unwrap(), text);
        assert!(source.enclosure(&mut ()).is_err());
        let image = AffineMap::new(
            IntervalMatrix::new([[p(1)], [p(1)]]).unwrap(),
            IntervalVector::new([p(0), p(0)]).unwrap(),
        )
        .unwrap()
        .image(source);
        let image = AffineMap::new(
            IntervalMatrix::new([[p(1), p(-1)]]).unwrap(),
            IntervalVector::new([p(0)]).unwrap(),
        )
        .unwrap()
        .image(image);
        let encoded = serde_json::to_string(&image).unwrap();
        let replayed: hyperlattice_interval::AffineImage<
            I,
            hyperlattice_interval::AffineImage<I, TranslatedBox<I, 1>, 2, 1>,
            1,
            2,
        > = serde_json::from_str(&encoded).unwrap();
        assert_eq!(
            replayed.enclosure(&mut ()).unwrap(),
            IntervalVector::new([p(0)]).unwrap()
        );
        return;
    }
    let source = CenteredVector::new(
        IntervalVector::new([p(i64::MAX)]).unwrap(),
        IntervalVector::new([p(1)]).unwrap(),
    )
    .unwrap()
    .affine_source(&mut ())
    .unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "translated_replay_revalidates_and_preserves_cancellation",
            "--nocapture",
        ])
        .env(KEY, serde_json::to_string(&source).unwrap())
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    for text in [
        "[null,[[0,0]]]",
        "[[[0,1]],null]",
        "[[[0,1]],[[-1,1]]]",
        "[[[0,0]],[[1,-1]]]",
        "[[[0,0]],[]]",
    ] {
        assert!(
            serde_json::from_str::<TranslatedBox<I, 1>>(text).is_err(),
            "{text}"
        );
    }
    let empty: TranslatedBox<I, 1> = serde_json::from_str("[null,null]").unwrap();
    assert!(empty.is_empty());
    let retained: TranslatedBox<I, 1> = serde_json::from_str("[[[9,9]],null]").unwrap();
    assert!(retained.is_empty() && retained.center().is_some());
    let unit: TranslatedBox<I, 0> = serde_json::from_str("[[],[]]").unwrap();
    assert!(unit.enclosure(&mut ()).unwrap().is_singleton());
    assert!(
        serde_json::from_str::<TranslatedBox<FixedInterval<i32, 32>, 0>>("[null,null]").is_err()
    );
}

fn corpus<I: IntervalArithmetic + PartialEq + core::fmt::Debug>(
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    for c in -8..=8 {
        for r in 0..=8 {
            let form = CenteredVector::new(
                IntervalVector::new([scalar(c, c)]).unwrap(),
                IntervalVector::new([scalar(r, r)]).unwrap(),
            )
            .unwrap();
            let source = form.affine_source(&mut context()).unwrap();
            assert_eq!(source.center(), form.center());
            assert_eq!(
                source.parameter_domain(),
                &IntervalVector::new([scalar(-r, r)]).unwrap()
            );
            assert_eq!(
                source.enclosure(&mut context()).unwrap(),
                IntervalVector::new([scalar(c - r, c + r)]).unwrap()
            );
            let restriction = IntervalVector::new([scalar(0, 1)]).unwrap();
            let selected = source
                .restrict_parameters(&restriction, &mut context())
                .unwrap();
            assert_eq!(
                selected.enclosure(&mut context()).unwrap(),
                IntervalVector::new([scalar(c, c + r.min(1))]).unwrap()
            );
            let duplicate = AffineMap::new(
                IntervalMatrix::new([[scalar(1, 1)], [scalar(1, 1)]]).unwrap(),
                IntervalVector::new([scalar(1, 1), scalar(0, 0)]).unwrap(),
            )
            .unwrap();
            let difference = AffineMap::new(
                IntervalMatrix::new([[scalar(1, 1), scalar(-1, -1)]]).unwrap(),
                IntervalVector::new([scalar(0, 0)]).unwrap(),
            )
            .unwrap();
            let image = difference.image(duplicate.image(source));
            assert_eq!(
                image.enclosure(&mut context()).unwrap(),
                IntervalVector::new([scalar(1, 1)]).unwrap()
            );
            let mut chain = AffineChain::<_, _, 1, 3>::new(image);
            for _ in 0..3 {
                chain
                    .push(
                        AffineMap::new(
                            IntervalMatrix::new([[scalar(2, 2)]]).unwrap(),
                            IntervalVector::new([scalar(1, 1)]).unwrap(),
                        )
                        .unwrap(),
                    )
                    .unwrap();
            }
            assert_eq!(
                chain.enclosure(&mut context()).unwrap(),
                IntervalVector::new([scalar(15, 15)]).unwrap()
            );
            assert_eq!(
                chain
                    .restrict_parameters(&restriction, &mut context())
                    .unwrap()
                    .enclosure(&mut context())
                    .unwrap(),
                IntervalVector::new([scalar(15, 15)]).unwrap()
            );
            let empty = chain
                .restrict_parameters(&IntervalVector::empty().unwrap(), &mut context())
                .unwrap();
            assert!(empty.is_empty());
            assert!(empty.enclosure(&mut context()).unwrap().is_empty());
        }
    }
    let empty = CenteredVector::<I, 1>::empty()
        .unwrap()
        .affine_source(&mut context())
        .unwrap();
    assert!(empty.is_empty() && empty.center().is_none());
    assert!(empty.enclosure(&mut context()).unwrap().is_empty());
    assert!(
        empty
            .restrict_parameters(
                &IntervalVector::new([scalar(0, 0)]).unwrap(),
                &mut context()
            )
            .unwrap()
            .is_empty()
    );
    let unit = CenteredVector::new(
        IntervalVector::<I, 0>::new([]).unwrap(),
        IntervalVector::new([]).unwrap(),
    )
    .unwrap()
    .affine_source(&mut context())
    .unwrap();
    assert!(!unit.is_empty());
    assert!(unit.enclosure(&mut context()).unwrap().is_singleton());
    let image = AffineMap::new(
        IntervalMatrix::<I, 2, 0>::new([[], []]).unwrap(),
        IntervalVector::new([scalar(3, 3), scalar(5, 5)]).unwrap(),
    )
    .unwrap()
    .image(unit);
    assert_eq!(
        image.enclosure(&mut context()).unwrap(),
        IntervalVector::new([scalar(3, 3), scalar(5, 5)]).unwrap()
    );
}

#[cfg(feature = "fixed")]
#[test]
fn fixed_sources_and_overflowing_original_bounds() {
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    corpus(
        |a, b| FixedInterval::<i32, 8>::new(a << 8, b << 8).unwrap(),
        || (),
    );
    corpus(
        |a, b| FixedInterval::<i64, 32>::new(i64::from(a) << 32, i64::from(b) << 32).unwrap(),
        || (),
    );
    macro_rules! narrow {
        ($int:ty,$frac:literal) => {{
            type I = FixedInterval<$int, $frac>;
            let half = 1 << ($frac - 1);
            let form = CenteredVector::new(
                IntervalVector::new([I::point(half).unwrap()]).unwrap(),
                IntervalVector::new([I::point(half).unwrap()]).unwrap(),
            )
            .unwrap();
            assert_eq!(
                form.enclosure(&mut ()),
                Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
            );
            assert!(I::one().is_err());
            let source = form.affine_source(&mut ()).unwrap();
            assert!(source.enclosure(&mut ()).is_err());
            let image = AffineMap::new(
                IntervalMatrix::new([[I::point(half).unwrap()]]).unwrap(),
                IntervalVector::new([I::point(0).unwrap()]).unwrap(),
            )
            .unwrap()
            .image(source);
            assert_eq!(
                image.enclosure(&mut ()).unwrap().lanes().unwrap()[0].bounds(),
                Some((0, half))
            );
        }};
    }
    narrow!(i32, 31);
    narrow!(i64, 63);
}

#[cfg(feature = "rational")]
#[test]
fn rational_sources() {
    use hyperlattice_interval::hyperinterval::RationalInterval;
    corpus(
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
    );
}

#[cfg(feature = "float-enclosure")]
#[test]
fn ieee_sources_and_cancelled_overflow() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    corpus(|a, b| F32Enclosure::new(a as f32, b as f32).unwrap(), || ());
    corpus(
        |a, b| F64Enclosure::new(f64::from(a), f64::from(b)).unwrap(),
        || (),
    );
    type I = F64Enclosure;
    let p = |x| I::point(x).unwrap();
    let form = CenteredVector::new(
        IntervalVector::new([p(f64::MAX)]).unwrap(),
        IntervalVector::new([p(f64::MAX)]).unwrap(),
    )
    .unwrap();
    assert!(form.enclosure(&mut ()).is_err());
    let source = form.affine_source(&mut ()).unwrap();
    let duplicate = AffineMap::new(
        IntervalMatrix::new([[p(1.0)], [p(1.0)]]).unwrap(),
        IntervalVector::new([p(0.0), p(0.0)]).unwrap(),
    )
    .unwrap()
    .image(source);
    assert!(duplicate.enclosure(&mut ()).is_err());
    let difference = AffineMap::new(
        IntervalMatrix::new([[p(1.0), p(-1.0)]]).unwrap(),
        IntervalVector::new([p(0.0)]).unwrap(),
    )
    .unwrap()
    .image(duplicate);
    assert_eq!(
        difference.enclosure(&mut ()).unwrap().lanes().unwrap()[0],
        p(0.0)
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn dyadic_sources_and_shared_retry() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    corpus(
        |a, b| I::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(128, 1_000_000),
    );
    let form = CenteredVector::new(
        IntervalVector::new([I::point(1.into(), i32::MIN).unwrap()]).unwrap(),
        IntervalVector::new([I::point(1.into(), i32::MIN).unwrap()]).unwrap(),
    )
    .unwrap();
    let before = form.clone();
    let mut ample = DyadicBudget::new(128, 10000);
    let source = form.affine_source(&mut ample).unwrap();
    let cost = 10000 - ample.remaining_work();
    assert!(cost > 0);
    for work in 0..cost {
        assert_eq!(
            form.affine_source(&mut DyadicBudget::new(128, work)),
            Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
        );
        assert_eq!(form, before);
    }
    assert_eq!(
        form.affine_source(&mut DyadicBudget::new(128, cost))
            .unwrap(),
        source
    );
    assert_eq!(
        source.parameter_domain().lanes().unwrap()[0],
        I::new((-1).into(), 1.into(), i32::MIN).unwrap()
    );
    assert_eq!(
        source
            .enclosure(&mut DyadicBudget::new(128, 10000))
            .unwrap()
            .lanes()
            .unwrap()[0],
        I::new(0.into(), 2.into(), i32::MIN).unwrap()
    );
    let point = |x: i32| I::point(x.into(), 0).unwrap();
    let form = CenteredVector::new(
        IntervalVector::new([point(1)]).unwrap(),
        IntervalVector::new([point(1)]).unwrap(),
    )
    .unwrap();
    let source = form
        .affine_source(&mut DyadicBudget::new(128, 10000))
        .unwrap();
    let image = AffineMap::new(
        IntervalMatrix::new([[point(2)]]).unwrap(),
        IntervalVector::new([point(1)]).unwrap(),
    )
    .unwrap()
    .image(source);
    let before = image.clone();
    let mut ample = DyadicBudget::new(128, 10000);
    let result = image.enclosure(&mut ample).unwrap();
    let cost = 10000 - ample.remaining_work();
    assert_eq!(
        result,
        IntervalVector::new([I::new(1.into(), 5.into(), 0).unwrap()]).unwrap()
    );
    for work in 0..cost {
        assert_eq!(
            image.enclosure(&mut DyadicBudget::new(128, work)),
            Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
        );
        assert_eq!(image, before);
    }
    assert_eq!(
        image.enclosure(&mut DyadicBudget::new(128, cost)).unwrap(),
        result
    );
}
