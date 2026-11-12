#[cfg(feature = "integer")]
mod integer {
    use hyperinterval::{IntegerRange, IntervalError, MidpointRounding};

    #[test]
    fn exhaustive_signed_and_unsigned_byte_points() {
        fn signed(actual: Result<IntegerRange<i8>, IntervalError>, exact: i16) {
            match i8::try_from(exact) {
                Ok(value) => assert_eq!(actual.unwrap().bounds(), Some((value, value))),
                Err(_) => assert_eq!(actual, Err(IntervalError::Overflow)),
            }
        }
        fn unsigned(actual: Result<IntegerRange<u8>, IntervalError>, exact: i32) {
            match u8::try_from(exact) {
                Ok(value) => assert_eq!(actual.unwrap().bounds(), Some((value, value))),
                Err(_) => assert_eq!(actual, Err(IntervalError::Overflow)),
            }
        }
        for a in i8::MIN..=i8::MAX {
            let x = IntegerRange::<i8>::point(a);
            signed(x.checked_neg(), -i16::from(a));
            signed(x.square(), i16::from(a).pow(2));
            for b in i8::MIN..=i8::MAX {
                let y = IntegerRange::<i8>::point(b);
                signed(x.checked_add(&y), i16::from(a) + i16::from(b));
                signed(x.checked_sub(&y), i16::from(a) - i16::from(b));
                signed(x.checked_mul(&y), i16::from(a) * i16::from(b));
            }
        }
        for a in u8::MIN..=u8::MAX {
            let x = IntegerRange::<u8>::point(a);
            unsigned(x.checked_neg(), -i32::from(a));
            unsigned(x.square(), i32::from(a).pow(2));
            for b in u8::MIN..=u8::MAX {
                let y = IntegerRange::<u8>::point(b);
                unsigned(x.checked_add(&y), i32::from(a) + i32::from(b));
                unsigned(x.checked_sub(&y), i32::from(a) - i32::from(b));
                unsigned(x.checked_mul(&y), i32::from(a) * i32::from(b));
            }
        }
    }

    #[test]
    fn small_ranges_enclose_enumerated_point_images() {
        for a in -5..=5 {
            for b in a..=5 {
                let x = IntegerRange::<i16>::new(a, b).unwrap();
                for exponent in 0..=4 {
                    let powers: Vec<_> = (a..=b).map(|v| v.pow(exponent)).collect();
                    assert_eq!(
                        x.checked_pow(exponent).unwrap().bounds(),
                        Some((*powers.iter().min().unwrap(), *powers.iter().max().unwrap()))
                    );
                }
                for c in -5..=5 {
                    for d in c..=5 {
                        let y = IntegerRange::new(c, d).unwrap();
                        let sum = x.checked_add(&y).unwrap();
                        let difference = x.checked_sub(&y).unwrap();
                        let product = x.checked_mul(&y).unwrap();
                        let mut products = Vec::new();
                        for p in a..=b {
                            for q in c..=d {
                                assert!(sum.contains_point(p + q));
                                assert!(difference.contains_point(p - q));
                                assert!(product.contains_point(p * q));
                                products.push(p * q);
                            }
                        }
                        assert_eq!(
                            product.bounds(),
                            Some((
                                *products.iter().min().unwrap(),
                                *products.iter().max().unwrap()
                            ))
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn full_widths_midpoint_ties_and_empty_composition() {
        use MidpointRounding::{Ceiling, Floor, NearestTiesEven};
        let all = IntegerRange::<i128>::new(i128::MIN, i128::MAX).unwrap();
        assert_eq!(all.width(), Some(u128::MAX));
        assert_eq!(all.radius(), Some(1_u128 << 127));
        assert_eq!(all.midpoint(Floor), Some(-1));
        assert_eq!(all.midpoint(Ceiling), Some(0));
        assert_eq!(all.midpoint(NearestTiesEven), Some(0));
        let unsigned = IntegerRange::<u128>::new(0, u128::MAX).unwrap();
        assert_eq!(unsigned.midpoint(Floor), Some(u128::MAX / 2));
        assert_eq!(unsigned.midpoint(Ceiling), Some(1_u128 << 127));
        assert_eq!(unsigned.midpoint(NearestTiesEven), Some(1_u128 << 127));
        for lower in -5_i8..=5 {
            let x = IntegerRange::new(lower, lower + 1).unwrap();
            assert_eq!(x.midpoint(Floor), Some(lower));
            assert_eq!(x.midpoint(Ceiling), Some(lower + 1));
            assert_eq!(x.midpoint(NearestTiesEven).unwrap() % 2, 0);
        }
        let empty = IntegerRange::<i128>::empty();
        assert_eq!(empty.width(), None);
        assert_eq!(empty.midpoint(Floor), None);
        assert_eq!(empty.checked_mul(&all).unwrap(), empty);
        assert_eq!(all.checked_add(&empty).unwrap(), empty);
        assert_eq!(empty.checked_neg().unwrap(), empty);
        assert_eq!(empty.checked_pow(0).unwrap(), empty);
        assert_eq!(
            IntegerRange::<i8>::new(-2, 3)
                .unwrap()
                .square()
                .unwrap()
                .bounds(),
            Some((0, 9))
        );
        assert_eq!(
            IntegerRange::<i8>::point(0).checked_pow(0).unwrap(),
            IntegerRange::point(1)
        );
    }

    #[test]
    fn width_and_signedness_conversions_preserve_exact_members() {
        let signed = IntegerRange::<i32>::new(i32::MIN, i32::MAX).unwrap();
        let wide = signed.convert::<i64>().unwrap();
        assert_eq!(
            wide.bounds(),
            Some((i64::from(i32::MIN), i64::from(i32::MAX)))
        );
        assert_eq!(wide.convert::<i32>().unwrap(), signed);
        assert_eq!(signed.convert::<u32>(), Err(IntervalError::Overflow));
        assert_eq!(
            IntegerRange::<u128>::point(u128::MAX).convert::<i128>(),
            Err(IntervalError::Overflow)
        );
        assert_eq!(
            IntegerRange::<u8>::empty().convert::<i8>().unwrap(),
            IntegerRange::empty()
        );
    }
}

#[cfg(feature = "fixed")]
mod fixed {
    use hyperinterval::{FixedInterval, IntervalError, MidpointRounding};

    // Integer cross-products are independent of the implementation's directed
    // division. `numerator/denominator` is measured in lattice units.
    fn enclosed(bounds: (i32, i32), mut numerator: i128, mut denominator: i128) {
        if denominator < 0 {
            numerator = -numerator;
            denominator = -denominator;
        }
        assert!(
            i128::from(bounds.0) * denominator <= numerator,
            "lower: {bounds:?}, {numerator}/{denominator}"
        );
        assert!(
            numerator <= i128::from(bounds.1) * denominator,
            "upper: {bounds:?}, {numerator}/{denominator}"
        );
    }

    #[test]
    fn reduced_grid_products_quotients_and_squares_enclose_half_lattice_points() {
        type I = FixedInterval<i32, 3>;
        for a in -4..=4 {
            for b in a..=4 {
                let x = I::new(a, b).unwrap();
                let square = x.square().unwrap().bounds().unwrap();
                for p in 2 * a..=2 * b {
                    enclosed(square, i128::from(p).pow(2), 4 * 8);
                }
                if a <= 0 && 0 <= b {
                    assert_eq!(square.0, 0);
                }
                for c in -4..=4 {
                    for d in c..=4 {
                        let y = I::new(c, d).unwrap();
                        let product = x.checked_mul(&y).unwrap().bounds().unwrap();
                        let sum = x.checked_add(&y).unwrap().bounds().unwrap();
                        let difference = x.checked_sub(&y).unwrap().bounds().unwrap();
                        let quotient = x.checked_div(&y);
                        if c <= 0 && 0 <= d {
                            assert_eq!(quotient, Err(IntervalError::ContainsZero));
                        }
                        for p in 2 * a..=2 * b {
                            for q in 2 * c..=2 * d {
                                enclosed(product, i128::from(p) * i128::from(q), 4 * 8);
                                enclosed(sum, i128::from(p) + i128::from(q), 2);
                                enclosed(difference, i128::from(p) - i128::from(q), 2);
                                if let Ok(value) = quotient {
                                    enclosed(
                                        value.bounds().unwrap(),
                                        i128::from(p) * 8,
                                        i128::from(q),
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn ratios_have_tight_directed_bounds_for_both_denominator_signs() {
        type I = FixedInterval<i32, 3>;
        for numerator in -128_i64..=127 {
            for denominator in -128_i64..=127 {
                let result = I::from_ratio(numerator, denominator);
                if denominator == 0 {
                    assert_eq!(result, Err(IntervalError::ContainsZero));
                    continue;
                }
                let (lower, upper) = result.unwrap().bounds().unwrap();
                let sign = if denominator < 0 { -1_i128 } else { 1 };
                let n = i128::from(numerator) * 8 * sign;
                let d = i128::from(denominator) * sign;
                enclosed((lower, upper), n, d);
                assert!((i128::from(lower) + 1) * d > n);
                assert!((i128::from(upper) - 1) * d < n);
            }
        }
        assert_eq!(
            FixedInterval::<i64, 32>::from_ratio(i64::MIN, i64::MIN)
                .unwrap()
                .bounds(),
            Some((1_i64 << 32, 1_i64 << 32))
        );
        assert_eq!(
            FixedInterval::<i64, 63>::from_ratio(i64::MIN, i64::MIN),
            Err(IntervalError::Overflow)
        );
        assert_eq!(
            FixedInterval::<i64, 64>::from_ratio(1, 1),
            Err(IntervalError::InvalidScale)
        );
    }

    #[test]
    fn extreme_endpoints_never_saturate_or_wrap() {
        type Q = FixedInterval<i32, 30>;
        let all = Q::new(i32::MIN, i32::MAX).unwrap();
        assert_eq!(all.width_ulps(), Some(u32::MAX.into()));
        assert_eq!(all.radius_ulps(), Some(1_u64 << 31));
        assert_eq!(all.checked_neg(), Err(IntervalError::Overflow));
        assert_eq!(
            all.checked_add(&Q::point(1).unwrap()),
            Err(IntervalError::Overflow)
        );
        assert_eq!(
            all.checked_sub(&Q::point(1).unwrap()),
            Err(IntervalError::Overflow)
        );
        assert_eq!(all.square(), Err(IntervalError::Overflow));
        assert_eq!(
            all.checked_mul(&Q::point(0).unwrap()).unwrap(),
            Q::point(0).unwrap()
        );
        assert_eq!(
            Q::point(1).unwrap().reciprocal(),
            Err(IntervalError::Overflow)
        );
        let negative_one = Q::point(-(1 << 30)).unwrap();
        assert_eq!(negative_one.reciprocal().unwrap(), negative_one);
        assert_eq!(negative_one.checked_pow(1).unwrap(), negative_one);
        assert_eq!(negative_one.checked_pow(u32::MAX).unwrap(), negative_one);
        // A unit intermediate is unnecessary for exponent one on Q1.31.
        let half = FixedInterval::<i32, 31>::point(1 << 30).unwrap();
        assert_eq!(half.checked_pow(1).unwrap(), half);
        assert_eq!(half.checked_pow(0), Err(IntervalError::Overflow));
        // Fused affine accumulation preserves cancellation before narrowing.
        let large = Q::point(3 << 29).unwrap();
        assert_eq!(large.checked_mul(&large), Err(IntervalError::Overflow));
        assert_eq!(
            large.affine(3 << 29, -(1 << 30)).unwrap().bounds(),
            Some((5 << 28, 5 << 28))
        );
        type Wide = FixedInterval<i64, 63>;
        let min = Wide::point(i64::MIN).unwrap();
        assert_eq!(
            min.affine(i64::MIN, i64::MIN).unwrap().bounds(),
            Some((0, 0))
        );
        assert_eq!(min.reciprocal().unwrap(), min);
        assert_eq!(min.checked_div(&min), Err(IntervalError::Overflow));
    }

    #[test]
    fn midpoint_radius_and_empty_domain_rules() {
        use MidpointRounding::{Ceiling, Floor, NearestTiesEven};
        type I = FixedInterval<i64, 32>;
        let all = I::new(i64::MIN, i64::MAX).unwrap();
        assert_eq!(all.width_ulps(), Some(u64::MAX));
        assert_eq!(all.radius_ulps(), Some(1_u64 << 63));
        assert_eq!(all.midpoint(Floor), Some(-1));
        assert_eq!(all.midpoint(Ceiling), Some(0));
        assert_eq!(all.midpoint(NearestTiesEven), Some(0));
        for a in -5..=5 {
            for b in a..=5 {
                let value = I::new(a, b).unwrap();
                let radius = i128::from(value.radius_ulps().unwrap());
                for rounding in [Floor, Ceiling, NearestTiesEven] {
                    let midpoint = i128::from(value.midpoint(rounding).unwrap());
                    assert!(midpoint - radius <= i128::from(a));
                    assert!(midpoint + radius >= i128::from(b));
                }
            }
        }
        let empty = I::empty().unwrap();
        let zero = I::point(0).unwrap();
        assert_eq!(empty.checked_div(&zero), Err(IntervalError::ContainsZero));
        assert_eq!(all.checked_div(&empty).unwrap(), empty);
        assert_eq!(all.checked_mul(&empty).unwrap(), empty);
        assert_eq!(empty.checked_add(&all).unwrap(), empty);
        assert_eq!(empty.checked_sub(&all).unwrap(), empty);
        assert_eq!(empty.checked_pow(0).unwrap(), empty);
        assert_eq!(empty.square().unwrap(), empty);
        assert_eq!(empty.reciprocal().unwrap(), empty);
        assert_eq!(empty.affine(i64::MAX, i64::MIN).unwrap(), empty);
    }

    #[test]
    fn q31_32_extreme_matrix_matches_exact_cross_product_oracle() {
        type I = FixedInterval<i64, 32>;
        const SCALE: i128 = 1_i128 << 32;
        fn check(result: Result<I, IntervalError>, points: &[(i128, i128)]) {
            let mut outside = false;
            for &(mut numerator, mut denominator) in points {
                if denominator < 0 {
                    numerator = -numerator;
                    denominator = -denominator;
                }
                assert!(denominator > 0);
                outside |= numerator < i128::from(i64::MIN) * denominator
                    || numerator > i128::from(i64::MAX) * denominator;
                if let Ok(value) = result {
                    let (lower, upper) = value.bounds().unwrap();
                    assert!(i128::from(lower) * denominator <= numerator);
                    assert!(numerator <= i128::from(upper) * denominator);
                }
            }
            if outside {
                assert_eq!(result, Err(IntervalError::Overflow));
            } else {
                assert!(result.is_ok());
            }
        }
        let values = [
            i64::MIN,
            i64::MIN + 1,
            -(1 << 32),
            -1,
            0,
            1,
            1 << 32,
            i64::MAX - 1,
            i64::MAX,
        ];
        for a in values {
            for b in values {
                let (a, b) = (a.min(b), a.max(b));
                let x = I::new(a, b).unwrap();
                check(
                    x.square(),
                    &[(i128::from(a).pow(2), SCALE), (i128::from(b).pow(2), SCALE)],
                );
                for c in values {
                    for d in values {
                        let (c, d) = (c.min(d), c.max(d));
                        let y = I::new(c, d).unwrap();
                        let pairs = [(a, c), (a, d), (b, c), (b, d)];
                        check(
                            x.checked_add(&y),
                            &pairs.map(|(p, q)| (i128::from(p) + i128::from(q), 1)),
                        );
                        check(
                            x.checked_sub(&y),
                            &pairs.map(|(p, q)| (i128::from(p) - i128::from(q), 1)),
                        );
                        check(
                            x.checked_mul(&y),
                            &pairs.map(|(p, q)| (i128::from(p) * i128::from(q), SCALE)),
                        );
                        if y.contains_point(0) {
                            assert_eq!(x.checked_div(&y), Err(IntervalError::ContainsZero));
                        } else {
                            check(
                                x.checked_div(&y),
                                &pairs.map(|(p, q)| (i128::from(p) * SCALE, i128::from(q))),
                            );
                        }
                        check(
                            x.affine(c, d),
                            &[
                                (i128::from(a) * i128::from(c) + i128::from(d) * SCALE, SCALE),
                                (i128::from(b) * i128::from(c) + i128::from(d) * SCALE, SCALE),
                            ],
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn rescaling_and_width_conversion_enclose_without_midpoint_substitution() {
        type I = FixedInterval<i32, 3>;
        for lower in -32..=32 {
            for upper in lower..=32 {
                let value = I::new(lower, upper).unwrap();
                let coarse = value.rescale::<1>().unwrap();
                let (c, d) = coarse.bounds().unwrap();
                assert!(c * 4 <= lower && upper <= d * 4);
                assert!((c + 1) * 4 > lower && upper > (d - 1) * 4);
                let fine = value.rescale::<6>().unwrap();
                assert_eq!(fine.bounds(), Some((lower * 8, upper * 8)));
                assert_eq!(fine.rescale::<3>().unwrap(), value);
                assert_eq!(
                    value.convert::<i64>().unwrap().convert::<i32>().unwrap(),
                    value
                );
            }
        }
        assert_eq!(
            I::point(i32::MAX).unwrap().rescale::<4>(),
            Err(IntervalError::Overflow)
        );
        assert_eq!(
            I::empty().unwrap().rescale::<32>(),
            Err(IntervalError::InvalidScale)
        );
        assert_eq!(
            FixedInterval::<i64, 63>::empty().unwrap().convert::<i32>(),
            Err(IntervalError::InvalidScale)
        );
        assert_eq!(
            FixedInterval::<i64, 3>::point(i64::MAX)
                .unwrap()
                .convert::<i32>(),
            Err(IntervalError::Overflow)
        );
    }

    #[cfg(feature = "integer")]
    #[test]
    fn integer_points_and_continuous_hulls_have_explicit_semantics() {
        use hyperinterval::IntegerRange;
        type I = FixedInterval<i32, 3>;
        let integers = IntegerRange::new(-3, 4).unwrap();
        let continuous = I::from_integer_range(&integers).unwrap();
        assert_eq!(continuous.bounds(), Some((-24, 32)));
        assert_eq!(continuous.integer_points().unwrap(), integers);
        assert!(I::new(1, 7).unwrap().integer_points().unwrap().is_empty());
        assert_eq!(
            I::new(-9, 9).unwrap().integer_points().unwrap().bounds(),
            Some((-1, 1))
        );
        assert_eq!(
            I::from_integer_range(&IntegerRange::point(i32::MAX)),
            Err(IntervalError::Overflow)
        );
        assert!(
            I::from_integer_range(&IntegerRange::empty())
                .unwrap()
                .is_empty()
        );
        assert!(I::empty().unwrap().integer_points().unwrap().is_empty());
    }
}
