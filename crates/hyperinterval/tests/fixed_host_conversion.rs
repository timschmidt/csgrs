#![cfg(all(feature = "fixed", any(feature = "dyadic", feature = "rational")))]

use hyperinterval::{FixedEndpoint, FixedInterval};

fn check<I: FixedEndpoint + Into<i64>, const F: u32>(endpoints: &[I]) {
    for (index, &a) in endpoints.iter().enumerate() {
        for &b in &endpoints[index..] {
            let source = FixedInterval::<I, F>::new(a, b).unwrap();
            let (a, b) = (a.into(), b.into());

            #[cfg(feature = "dyadic")]
            {
                use hyperinterval::DyadicBudget;
                use num_traits::ToPrimitive;
                let mut budget = DyadicBudget::new(64, 128);
                let image = source.to_dyadic(&mut budget).unwrap();
                let (lower, upper, e) = image.bounds().unwrap();
                let shift = u32::try_from(e + F as i32).unwrap();
                // Recover each lattice numerator independently after normalization.
                assert_eq!(i128::from(lower.to_i64().unwrap()) << shift, i128::from(a));
                assert_eq!(i128::from(upper.to_i64().unwrap()) << shift, i128::from(b));
                assert_eq!(image.signs(), source.signs());
            }

            #[cfg(feature = "rational")]
            {
                use hyperreal::Rational;
                let image = source.to_rational();
                let scale = Rational::from(1_u64 << F);
                let (lower, upper) = image.bounds().unwrap();
                assert_eq!(lower * &scale, Rational::from(a));
                assert_eq!(upper * &scale, Rational::from(b));
                assert_eq!(image.signs(), source.signs());
            }

            #[cfg(all(feature = "dyadic", feature = "rational"))]
            {
                let mut budget = hyperinterval::DyadicBudget::new(256, 2_000_000);
                let width = source
                    .to_dyadic(&mut budget)
                    .unwrap()
                    .width(&mut budget)
                    .unwrap();
                assert_eq!(
                    width.to_rational(&mut budget).unwrap(),
                    hyperinterval::RationalInterval::point(source.to_rational().width().unwrap())
                );
            }
        }
    }
}

#[test]
fn fixed_host_images_preserve_full_width_endpoints_at_every_scale() {
    macro_rules! scales {
        ($ty:ty; $($scale:literal),+) => {{
            let endpoints: &[$ty] = &[<$ty>::MIN, <$ty>::MIN + 1, -4, -1, 0, 1, 4, <$ty>::MAX - 1, <$ty>::MAX];
            $(check::<$ty, $scale>(endpoints);)+
        }};
    }
    scales!(i32;
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
    );
    scales!(i64;
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
        32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
        48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
    );
}

#[test]
fn empty_fixed_image_is_empty() {
    let empty = FixedInterval::<i64, 63>::empty().unwrap();
    #[cfg(feature = "rational")]
    assert!(empty.to_rational().is_empty());
    #[cfg(feature = "dyadic")]
    assert!(
        empty
            .to_dyadic(&mut hyperinterval::DyadicBudget::new(0, 0))
            .unwrap()
            .is_empty()
    );
}

#[cfg(feature = "dyadic")]
#[test]
fn budget_precedes_endpoint_materialization_and_allows_retry() {
    use hyperinterval::{DyadicBudget, IntervalError};
    let source = FixedInterval::<i64, 63>::new(i64::MIN, 3).unwrap();
    for (bits, work, error) in [
        (63, 128, IntervalError::SizeExhausted),
        (64, 127, IntervalError::WorkExhausted),
    ] {
        let mut budget = DyadicBudget::new(bits, work);
        assert_eq!(source.to_dyadic(&mut budget), Err(error));
        assert_eq!(budget.remaining_work(), work);
    }
    let mut budget = DyadicBudget::new(64, 128);
    assert_eq!(
        source.to_dyadic(&mut budget).unwrap().bounds(),
        Some((&i64::MIN.into(), &3.into(), -63))
    );
    assert_eq!(budget.remaining_work(), 0);
    let zero = FixedInterval::<i32, 30>::point(0).unwrap();
    assert!(
        zero.to_dyadic(&mut DyadicBudget::new(0, 2))
            .unwrap()
            .is_singleton()
    );
}
