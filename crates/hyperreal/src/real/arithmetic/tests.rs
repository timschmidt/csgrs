#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::size_of;

    #[test]
    fn operations_work_on_refs() {
        let a = Real::new(Rational::new(2));
        let b = Real::new(Rational::new(3));
        let c = Real::new(Rational::new(6));
        assert_eq!(&a * &b, c.clone());
        assert_eq!(&c / &b, Ok(a.clone()));
        assert_eq!(&c - &a, Real::new(Rational::new(4)));
        assert_eq!(-&c, Real::new(Rational::new(-6)));
        assert_eq!(&a + &b, Real::new(Rational::new(5)));
    }

    #[test]
    fn layout_sizes() {
        const MAX_REAL_SIZE: usize = 48;

        assert!(
            size_of::<Real>() <= MAX_REAL_SIZE,
            "Real grew to {} bytes",
            size_of::<Real>()
        );
        assert!(
            size_of::<Rational>() <= 8,
            "Rational grew to {} bytes",
            size_of::<Rational>()
        );
        assert!(
            size_of::<Class>() <= 16,
            "Class grew to {} bytes",
            size_of::<Class>()
        );
        assert!(
            size_of::<AtomicPrimitiveApproxCache>() <= 8,
            "atomic primitive cache grew to {} bytes",
            size_of::<AtomicPrimitiveApproxCache>()
        );
        assert!(
            size_of::<PrimitiveApproxCache>() <= 16,
            "PrimitiveApproxCache grew to {} bytes",
            size_of::<PrimitiveApproxCache>()
        );
        assert!(
            size_of::<ConstProductClass>() <= 16,
            "ConstProductClass grew to {} bytes",
            size_of::<ConstProductClass>()
        );
        assert!(
            size_of::<ConstOffsetClass>() <= 24,
            "ConstOffsetClass grew to {} bytes",
            size_of::<ConstOffsetClass>()
        );
        assert!(
            size_of::<ConstProductSqrtClass>() <= 24,
            "ConstProductSqrtClass grew to {} bytes",
            size_of::<ConstProductSqrtClass>()
        );
        assert!(
            size_of::<LnAffineClass>() <= 16,
            "LnAffineClass grew to {} bytes",
            size_of::<LnAffineClass>()
        );
        assert!(
            size_of::<LnProductClass>() <= 16,
            "LnProductClass grew to {} bytes",
            size_of::<LnProductClass>()
        );
    }

    #[test]
    fn aggregate_helpers_keep_values_in_real_space() {
        let values = [Real::from(1_i32), Real::from(3_i32), Real::from(5_i32)];

        assert_eq!(Real::sum_refs(values.iter()), Real::from(9_i32));
        assert_eq!(Real::mean(&values), Some(Real::from(3_i32)));
        assert_eq!(
            Real::affine(&Real::from(1_i32), &Real::from(2_i32), &Real::from(3_i32)),
            Real::from(7_i32)
        );

        let stddev = Real::sample_stddev(&values).unwrap();
        assert_eq!(stddev, Real::from(4_i32).sqrt().unwrap());
    }

    #[test]
    fn product_sum_helpers_preserve_exact_geometry_kernels() {
        assert_eq!(
            Real::mul_add(&Real::from(2_i32), &Real::from(3_i32), &Real::from(4_i32)),
            Real::from(10_i32)
        );
        assert_eq!(
            Real::mul_add(&Real::zero(), &Real::pi(), &Real::from(4_i32)),
            Real::from(4_i32)
        );
        assert_eq!(
            Real::diff_of_products(
                &Real::from(2_i32),
                &Real::from(5_i32),
                &Real::from(3_i32),
                &Real::from(4_i32),
            ),
            Real::from(-2_i32)
        );
        let left = [
            Real::new(Rational::fraction(1, 2).unwrap()),
            Real::new(Rational::fraction(1, 3).unwrap()),
            Real::new(Rational::fraction(1, 5).unwrap()),
            Real::new(Rational::fraction(1, 7).unwrap()),
            Real::new(Rational::fraction(1, 11).unwrap()),
        ];
        let right = [
            Real::new(Rational::fraction(2, 3).unwrap()),
            Real::new(Rational::fraction(3, 5).unwrap()),
            Real::new(Rational::fraction(5, 7).unwrap()),
            Real::new(Rational::fraction(7, 11).unwrap()),
            Real::new(Rational::fraction(11, 13).unwrap()),
        ];
        let expected = left
            .iter()
            .zip(&right)
            .map(|(l, r)| l * r)
            .fold(Real::zero(), |sum, term| &sum + &term);
        assert_eq!(Real::sum_products(&left, &right).unwrap(), expected);
        assert_eq!(
            Real::sum_products(&left[..2], &right[..3]),
            Err(Problem::ParseError)
        );
    }

    #[test]
    fn polynomial_helpers_preserve_evaluation_forms() {
        let coeffs = [Real::from(1_i32), Real::from(2_i32), Real::from(3_i32)];
        assert_eq!(Real::eval_poly(&coeffs, &Real::from(2_i32)), Real::from(17_i32));
        assert_eq!(Real::eval_poly(&[], &Real::from(2_i32)), Real::zero());

        let numerator = [Real::one(), Real::one()];
        let denominator = [Real::one(), Real::from(-1_i32)];
        assert_eq!(
            Real::eval_rational_poly(&numerator, &denominator, &Real::from(2_i32)),
            Ok(Real::from(-3_i32))
        );
        assert_eq!(
            Real::eval_rational_poly(&[Real::one()], &[Real::from(-2_i32), Real::one()], &Real::from(2_i32)),
            Err(Problem::DivideByZero)
        );
    }

    #[test]
    fn certified_integer_helpers_make_discontinuous_decisions() {
        let seven_thirds = Real::new(Rational::fraction(7, 3).unwrap());
        assert_eq!(seven_thirds.floor_certified(), Ok(BigInt::from(2_i32)));
        assert_eq!(seven_thirds.ceil_certified(), Ok(BigInt::from(3_i32)));
        assert_eq!(seven_thirds.trunc_certified(), Ok(BigInt::from(2_i32)));
        assert_eq!(seven_thirds.round_certified(), Ok(BigInt::from(2_i32)));
        assert_eq!(
            seven_thirds.fract_certified().unwrap(),
            Real::new(Rational::fraction(1, 3).unwrap())
        );

        let negative_seven_thirds = Real::new(Rational::fraction(-7, 3).unwrap());
        assert_eq!(
            negative_seven_thirds.floor_certified(),
            Ok(BigInt::from(-3_i32))
        );
        assert_eq!(
            negative_seven_thirds.ceil_certified(),
            Ok(BigInt::from(-2_i32))
        );
        assert_eq!(
            negative_seven_thirds.trunc_certified(),
            Ok(BigInt::from(-2_i32))
        );
        assert_eq!(
            negative_seven_thirds.round_certified(),
            Ok(BigInt::from(-2_i32))
        );
        assert_eq!(
            negative_seven_thirds.fract_certified().unwrap(),
            Real::new(Rational::fraction(-1, 3).unwrap())
        );

        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap()).round_certified(),
            Ok(BigInt::from(1_i32))
        );
        assert_eq!(
            Real::new(Rational::fraction(-1, 2).unwrap()).round_certified(),
            Ok(BigInt::from(-1_i32))
        );

        assert_eq!(Real::pi().floor_certified(), Ok(BigInt::from(3_i32)));
        assert_eq!(Real::pi().ceil_certified(), Ok(BigInt::from(4_i32)));
        assert_eq!(Real::pi().trunc_certified(), Ok(BigInt::from(3_i32)));
        assert_eq!(Real::pi().round_certified(), Ok(BigInt::from(3_i32)));
        assert_eq!(
            Real::pi().fract_certified().unwrap(),
            Real::pi() - Real::from(3_i32)
        );

        assert_eq!(
            Real::from(-7_i32)
                .rem_euclid_certified(&Real::from(3_i32))
                .unwrap(),
            Real::from(2_i32)
        );
        assert_eq!(
            Real::pi()
                .rem_euclid_certified(&Real::from(2_i32))
                .unwrap(),
            Real::pi() - Real::from(2_i32)
        );
        assert_eq!(
            Real::from(7_i32).rem_euclid_certified(&Real::zero()),
            Err(Problem::NotANumber)
        );
        assert_eq!(
            Real::from(7_i32).rem_euclid_certified(&Real::from(-3_i32)),
            Err(Problem::NotANumber)
        );
    }

    #[test]
    fn hypot_helpers_preserve_exact_lengths() {
        assert_eq!(
            Real::hypot2(&Real::from(3_i32), &Real::from(4_i32)).unwrap(),
            Real::from(5_i32)
        );
        assert_eq!(
            Real::hypot3(&Real::from(2_i32), &Real::from(3_i32), &Real::from(6_i32)).unwrap(),
            Real::from(7_i32)
        );

        assert_eq!(
            Real::hypot2(&Real::zero(), &Real::from(-11_i32)).unwrap(),
            Real::from(11_i32)
        );
        assert_eq!(
            Real::hypot3(&Real::zero(), &Real::zero(), &(-Real::pi())).unwrap(),
            Real::pi()
        );
        assert_eq!(
            Real::hypot_minus(&Real::from(3_i32), &Real::from(4_i32)).unwrap(),
            Real::from(2_i32)
        );
        assert_eq!(
            Real::hypot_minus(&Real::from(-3_i32), &Real::from(4_i32)).unwrap(),
            Real::from(8_i32)
        );
        assert_eq!(
            Real::hypot_minus(&Real::zero(), &Real::from(-7_i32)).unwrap(),
            Real::from(7_i32)
        );
        assert!(Real::hypot_minus(&Real::from(7_i32), &Real::zero())
            .unwrap()
            .definitely_zero());
        assert_eq!(
            Real::hypot_minus(&Real::from(-7_i32), &Real::zero()).unwrap(),
            Real::from(14_i32)
        );
    }

    #[test]
    fn abs_and_angle_conversions_preserve_exact_real_structure() {
        assert_eq!(Real::from(-7_i32).abs(), Real::from(7_i32));
        assert_eq!((-Real::pi()).abs(), Real::pi());
        assert_eq!(Real::zero().abs(), Real::zero());

        assert_eq!(Real::from(180_i32).to_radians(), Real::pi());
        assert_eq!(Real::pi().to_degrees(), Real::from(180_i32));
        assert_eq!(
            Real::from(45_i32).to_radians().to_degrees(),
            Real::from(45_i32)
        );
    }
}
