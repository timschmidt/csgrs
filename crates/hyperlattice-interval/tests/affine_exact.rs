#[cfg(feature = "rational")]
#[test]
fn fractional_inverse_retains_large_rational_parameters() {
    use hyperlattice_interval::hyperinterval::RationalInterval as I;
    use hyperlattice_interval::{AffineMap, IntervalMatrix, IntervalVector};
    let p = |s: &str| I::point(s.parse().unwrap());
    let source=IntervalVector::new([
        I::new("-123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789/7".parse().unwrap(),"123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789/13".parse().unwrap()).unwrap(),
        I::new("-19/17".parse().unwrap(),"23/19".parse().unwrap()).unwrap(),
    ]).unwrap();
    let first = AffineMap::new(
        IntervalMatrix::new([[p("1/2"), p("1/3")], [p("0"), p("1/5")]]).unwrap(),
        IntervalVector::new([p("1/7"), p("-1/11")]).unwrap(),
    )
    .unwrap();
    let inverse = AffineMap::new(
        IntervalMatrix::new([[p("2"), p("-10/3")], [p("0"), p("5")]]).unwrap(),
        IntervalVector::new([p("-136/231"), p("5/11")]).unwrap(),
    )
    .unwrap();
    let image = inverse.image(first.image(source.clone()));
    assert_eq!(image.enclosure(&mut ()).unwrap(), source);
    #[cfg(feature = "serde")]
    {
        use hyperlattice_interval::{AffineBox, AffineImage};
        type Chain = AffineImage<I, AffineBox<I, 2, 2>, 2, 2>;
        let text = serde_json::to_string(&image).unwrap();
        let replay: Chain = serde_json::from_str(&text).unwrap();
        assert_eq!(replay, image);
        assert_eq!(replay.enclosure(&mut ()).unwrap(), source);
    }
}

#[cfg(feature = "dyadic")]
#[test]
fn cancellation_precedes_large_dyadic_grid_materialization() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    use hyperlattice_interval::{AffineMap, IntervalLatticeError, IntervalMatrix, IntervalVector};
    let p = |m: i32, e| I::point(m.into(), e).unwrap();
    let source = IntervalVector::new([I::new((-5).into(), 7.into(), -150).unwrap()]).unwrap();
    let first = AffineMap::new(
        IntervalMatrix::new([[p(1, -100)]]).unwrap(),
        IntervalVector::new([p(1, 100)]).unwrap(),
    )
    .unwrap();
    let inverse = AffineMap::new(
        IntervalMatrix::new([[p(1, 100)]]).unwrap(),
        IntervalVector::new([p(-1, 200)]).unwrap(),
    )
    .unwrap();
    let intermediate = first.image(source.clone());
    assert_eq!(
        intermediate.enclosure(&mut DyadicBudget::new(256, 1_000_000)),
        Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
    );
    let image = inverse.image(intermediate);
    assert_eq!(
        image
            .enclosure(&mut DyadicBudget::new(256, 1_000_000))
            .unwrap(),
        source
    );
    #[cfg(feature = "serde")]
    {
        use hyperlattice_interval::{AffineBox, AffineImage};
        type Chain = AffineImage<I, AffineBox<I, 1, 1>, 1, 1>;
        let text = serde_json::to_string(&image).unwrap();
        let replay: Chain = serde_json::from_str(&text).unwrap();
        assert_eq!(replay, image);
        assert_eq!(
            replay
                .enclosure(&mut DyadicBudget::new(256, 1_000_000))
                .unwrap(),
            source
        );
    }
}

#[cfg(all(feature = "fixed", feature = "serde"))]
#[test]
fn i64_replay_preserves_extreme_binary_coefficients() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    use hyperlattice_interval::{AffineBox, AffineMap, IntervalMatrix, IntervalVector};
    type I = FixedInterval<i64, 63>;
    let map = AffineMap::new(
        IntervalMatrix::new([[I::point(i64::MAX).unwrap()]]).unwrap(),
        IntervalVector::new([I::point(0).unwrap()]).unwrap(),
    )
    .unwrap();
    let image = map.image(IntervalVector::new([I::new(i64::MIN, 0).unwrap()]).unwrap());
    let text = serde_json::to_string(&image).unwrap();
    let replay: AffineBox<I, 1, 1> = serde_json::from_str(&text).unwrap();
    assert_eq!(replay, image);
    assert_eq!(
        replay.enclosure(&mut ()).unwrap().lanes(),
        Some(&[I::new(-i64::MAX, 0).unwrap()])
    );
}
