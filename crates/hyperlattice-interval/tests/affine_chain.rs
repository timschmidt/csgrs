#![cfg(any(
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use hyperlattice_interval::{
    AffineChain, AffineMap, AffineSource, IntervalArithmetic, IntervalMatrix, IntervalVector,
};

#[cfg(any(feature = "fixed", feature = "dyadic"))]
use hyperlattice_interval::IntervalLatticeError;

fn map<I: IntervalArithmetic, const R: usize, const C: usize>(
    a: [[i32; C]; R],
    b: [i32; R],
    scalar: &impl Fn(i32, i32) -> I,
) -> AffineMap<I, R, C> {
    AffineMap::new(
        IntervalMatrix::new(a.map(|row| row.map(|x| scalar(x, x)))).unwrap(),
        IntervalVector::new(b.map(|x| scalar(x, x))).unwrap(),
    )
    .unwrap()
}

fn qualify<I: IntervalArithmetic + std::fmt::Debug + PartialEq>(
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
) {
    // Five noncommuting/invertible/singular maps; enumerate every word of
    // lengths 0..=4. Oracle evaluates each map at source corners using integers,
    // never multiplying interval coefficients or reusing a production kernel.
    let maps = [
        ([[1, 1], [0, 1]], [1, -1]),
        ([[1, -1], [0, 1]], [-2, 1]),
        ([[0, -1], [1, 0]], [2, 0]),
        ([[1, 1], [1, 1]], [3, -4]),
        ([[0, 0], [0, 0]], [-1, 2]),
    ];
    let domain = IntervalVector::new([scalar(-2, 1), scalar(-1, 3)]).unwrap();
    let mut completed = 0;
    for depth in 0..=4_u32 {
        for word in 0..5_usize.pow(depth) {
            let mut chain = AffineChain::<_, _, 2, 4>::new(domain.clone());
            let mut corners = [[-2, -1], [-2, 3], [1, -1], [1, 3]];
            let mut digits = word;
            for _ in 0..depth {
                let (a, b) = maps[digits % 5];
                digits /= 5;
                chain.push(map(a, b, &scalar)).unwrap();
                for corner in &mut corners {
                    *corner =
                        core::array::from_fn(|r| a[r][0] * corner[0] + a[r][1] * corner[1] + b[r]);
                }
            }
            let expected = core::array::from_fn(|r| {
                scalar(
                    corners.iter().map(|c| c[r]).min().unwrap(),
                    corners.iter().map(|c| c[r]).max().unwrap(),
                )
            });
            assert_eq!(
                chain.enclosure(&mut context()).unwrap().lanes(),
                Some(&expected)
            );
            assert_eq!(chain.map_count(), depth as usize);
            assert_eq!(chain.maps().len(), depth as usize);
            assert_eq!(chain.capacity(), 4);
            assert_eq!(chain.parameter_domain(), &domain);
            assert_eq!(chain.source(), &domain);
            assert!(!chain.is_empty());
            completed += 1;
        }
    }
    assert_eq!(completed, 781);

    // Static rectangular source -> runtime loop -> rectangular difference.
    let source = IntervalVector::new([scalar(-1, 1)]).unwrap();
    for depth in 0..=8 {
        let duplicate = map([[1], [1]], [3, -4], &scalar).image(source.clone());
        let mut chain = AffineChain::<_, _, 2, 8>::new(duplicate);
        for _ in 0..depth {
            chain.push(map([[1, 0], [0, 1]], [1, -1], &scalar)).unwrap();
        }
        let restricted = chain
            .restrict_parameters(
                &IntervalVector::new([scalar(0, 1)]).unwrap(),
                &mut context(),
            )
            .unwrap();
        let difference = map([[1, -1]], [0], &scalar).image(restricted);
        assert_eq!(
            difference.enclosure(&mut context()).unwrap().lanes(),
            Some(&[scalar(7 + 2 * depth, 7 + 2 * depth)])
        );
        assert_eq!(chain.parameter_domain(), &source);
        // A runtime chain is itself a valid runtime source, including zero maps.
        let nested = AffineChain::<_, _, 2, 0>::new(chain);
        let generic = AffineSource::enclosure(&nested, &mut context()).unwrap();
        assert_eq!(
            generic.lanes(),
            Some(&[scalar(2 + depth, 4 + depth), scalar(-5 - depth, -3 - depth)])
        );
        let empty = AffineSource::restrict_parameters(
            &nested,
            &IntervalVector::empty().unwrap(),
            &mut context(),
        )
        .unwrap();
        assert!(AffineSource::is_empty(&empty));
        assert!(
            AffineSource::enclosure(&empty, &mut context())
                .unwrap()
                .is_empty()
        );
        assert!(AffineSource::parameter_domain(&empty).is_empty());
    }
}

#[cfg(feature = "fixed")]
#[test]
fn fixed_runtime_words() {
    use hyperlattice_interval::hyperinterval::FixedInterval;
    qualify(|a, b| FixedInterval::<i32, 0>::new(a, b).unwrap(), || ());
    qualify(
        |a, b| FixedInterval::<i64, 0>::new(a.into(), b.into()).unwrap(),
        || (),
    );
}
#[cfg(feature = "rational")]
#[test]
fn rational_runtime_words() {
    use hyperlattice_interval::hyperinterval::RationalInterval;
    qualify(
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
    );
}
#[cfg(feature = "dyadic")]
#[test]
fn dyadic_runtime_words() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval};
    qualify(
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
    );
}
#[cfg(feature = "float-enclosure")]
#[test]
fn float_runtime_words() {
    use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure};
    qualify(|a, b| F32Enclosure::new(a as f32, b as f32).unwrap(), || ());
    qualify(|a, b| F64Enclosure::new(a.into(), b.into()).unwrap(), || ());
}

#[cfg(feature = "fixed")]
#[test]
fn zero_capacity_retains_source_and_failed_append_can_be_rebuilt() {
    use core::error::Error;
    use hyperlattice_interval::hyperinterval::FixedInterval;
    // +1 is unrepresentable; no-map composition must not require it.
    type I = FixedInterval<i32, 31>;
    let source = IntervalVector::new([I::new(-1, 1).unwrap()]).unwrap();
    let mut chain = AffineChain::<_, _, 1, 0>::new(source);
    assert_eq!(chain.enclosure(&mut ()).unwrap(), source);
    let zero = map([[0]], [0], &|a, b| I::new(a, b).unwrap());
    let error = chain.push(zero).unwrap_err();
    assert_eq!(
        error,
        IntervalLatticeError::AffineCapacityExhausted { capacity: 0 }
    );
    assert_eq!(error.to_string(), "affine chain capacity 0 exhausted");
    assert!(error.source().is_none());
    assert_eq!(chain.map_count(), 0);
    assert_eq!(chain.enclosure(&mut ()).unwrap(), source);
    let (source, maps) = chain.into_parts();
    assert_eq!(maps.len(), 0);
    let mut larger = AffineChain::<_, _, 1, 1>::new(source);
    larger.push(zero).unwrap();
    let before = larger;
    assert_eq!(
        larger.push(zero),
        Err(IntervalLatticeError::AffineCapacityExhausted { capacity: 1 })
    );
    assert_eq!(larger, before);
    let (source, maps) = larger.into_parts();
    let mut retry = AffineChain::<_, _, 1, 2>::new(source);
    for map in maps {
        retry.push(map).unwrap();
    }
    retry.push(zero).unwrap();
    assert_eq!(retry.map_count(), 2);
    assert_eq!(
        retry.enclosure(&mut ()).unwrap().lanes(),
        Some(&[I::point(0).unwrap()])
    );
    assert_eq!(
        zero.image(retry).enclosure(&mut ()).unwrap().lanes(),
        Some(&[I::point(0).unwrap()])
    );

    for source in [
        IntervalVector::<I, 0>::new([]).unwrap(),
        IntervalVector::empty().unwrap(),
    ] {
        let mut chain = AffineChain::<_, _, 0, 4>::new(source);
        for _ in 0..4 {
            chain
                .push(map([], [], &|a, b| I::new(a, b).unwrap()))
                .unwrap();
        }
        assert_eq!(chain.enclosure(&mut ()).unwrap(), source);
    }
}

#[cfg(feature = "fixed")]
#[test]
fn off_grid_and_failed_projections_preserve_exact_maps() {
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    type I = FixedInterval<i32, 1>;
    let scalar = |a, b| I::new(a, b).unwrap();
    let source = IntervalVector::new([scalar(1, 1)]).unwrap();
    let mut chain = AffineChain::<_, _, 1, 2>::new(source);
    chain.push(map([[1]], [0], &scalar)).unwrap(); // half of 1/2
    assert_eq!(
        chain.enclosure(&mut ()).unwrap().lanes(),
        Some(&[scalar(0, 1)])
    );
    chain.push(map([[4]], [0], &scalar)).unwrap(); // double restores 1/2
    assert_eq!(chain.enclosure(&mut ()).unwrap(), source);
    type J = FixedInterval<i32, 0>;
    let scalar = |a, b| J::new(a, b).unwrap();
    let mut chain =
        AffineChain::<_, _, 1, 2>::new(IntervalVector::new([scalar(i32::MAX, i32::MAX)]).unwrap());
    chain.push(map([[2]], [0], &scalar)).unwrap();
    assert_eq!(
        chain.enclosure(&mut ()),
        Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
    );
    chain.push(map([[0]], [0], &scalar)).unwrap();
    assert_eq!(
        chain.enclosure(&mut ()).unwrap().lanes(),
        Some(&[scalar(0, 0)])
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn runtime_budget_is_shared_with_sources_copies_and_outer_maps() {
    use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as I, IntervalError};
    let scalar = |a: i32, b: i32| I::new(a.into(), b.into(), 0).unwrap();
    let source = IntervalVector::new([scalar(-1, 2)]).unwrap();
    let mut chain = AffineChain::<_, _, 1, 3>::new(source.clone());
    for (a, b) in [(2, 1), (3, 4), (5, 7)] {
        chain.push(map([[a]], [b], &scalar)).unwrap();
    }
    let mut budget = DyadicBudget::new(256, 1_000_000);
    let expected = chain.enclosure(&mut budget).unwrap();
    assert_eq!(expected.lanes(), Some(&[scalar(12, 102)])); // 30x+42
    let spent = 1_000_000 - budget.remaining_work();
    assert_eq!(
        chain.enclosure(&mut DyadicBudget::new(256, spent - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    let mut exact = DyadicBudget::new(256, spent);
    assert_eq!(chain.enclosure(&mut exact).unwrap(), expected);
    assert_eq!(exact.remaining_work(), 0);
    assert_eq!(
        chain.enclosure(&mut DyadicBudget::new(1, 1_000_000)),
        Err(IntervalLatticeError::Scalar(IntervalError::SizeExhausted))
    );
    let selection = IntervalVector::new([scalar(0, 1)]).unwrap();
    let mut budget = DyadicBudget::new(256, 1_000_000);
    let selected = chain.restrict_parameters(&selection, &mut budget).unwrap();
    let spent = 1_000_000 - budget.remaining_work();
    assert_eq!(
        selected
            .enclosure(&mut DyadicBudget::new(256, 1_000_000))
            .unwrap()
            .lanes(),
        Some(&[scalar(42, 72)])
    );
    assert_eq!(
        chain.restrict_parameters(&selection, &mut DyadicBudget::new(256, spent - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert_eq!(
        chain
            .restrict_parameters(&selection, &mut DyadicBudget::new(256, spent))
            .unwrap(),
        selected
    );
    assert_eq!(chain.parameter_domain(), &source);
    let outer = map([[2]], [1], &scalar).image(chain);
    let mut budget = DyadicBudget::new(256, 1_000_000);
    assert_eq!(
        outer.enclosure(&mut budget).unwrap().lanes(),
        Some(&[scalar(25, 205)])
    );
    let spent = 1_000_000 - budget.remaining_work();
    assert_eq!(
        outer.enclosure(&mut DyadicBudget::new(256, spent - 1)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert!(outer.enclosure(&mut DyadicBudget::new(256, spent)).is_ok());
    let empty = outer
        .restrict_parameters(
            &IntervalVector::empty().unwrap(),
            &mut DyadicBudget::new(256, 1_000_000),
        )
        .unwrap();
    assert!(
        empty
            .enclosure(&mut DyadicBudget::new(1, 0))
            .unwrap()
            .is_empty()
    );
    let zero = AffineChain::<_, _, 1, 0>::new(source);
    assert_eq!(
        zero.enclosure(&mut DyadicBudget::new(256, 0)),
        Err(IntervalLatticeError::Scalar(IntervalError::WorkExhausted))
    );
    assert_eq!(
        zero.enclosure(&mut DyadicBudget::new(256, 1_000_000))
            .unwrap(),
        *zero.source()
    );
}
