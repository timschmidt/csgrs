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

    fn exact_affine_det2_sign(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> RealSign {
        let [ax, ay] = a.map(|value| value.exact_rational().unwrap());
        let [bx, by] = b.map(|value| value.exact_rational().unwrap());
        let [cx, cy] = c.map(|value| value.exact_rational().unwrap());
        let determinant = (bx - &ax) * (cy - &ay) - (by - ay) * (cx - ax);
        if determinant.is_positive() {
            RealSign::Positive
        } else if determinant.is_negative() {
            RealSign::Negative
        } else {
            RealSign::Zero
        }
    }

    fn exact_linear_form3_sign(coefficients: [&Real; 4], point: [&Real; 3]) -> RealSign {
        let [a, b, c, d] = coefficients.map(|value| value.exact_rational().unwrap());
        let [x, y, z] = point.map(|value| value.exact_rational().unwrap());
        let value = a * x + b * y + c * z + d;
        if value.is_positive() {
            RealSign::Positive
        } else if value.is_negative() {
            RealSign::Negative
        } else {
            RealSign::Zero
        }
    }

    fn exact_affine_det3_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
    ) -> RealSign {
        let [ax, ay, az] = a.map(|value| value.exact_rational().unwrap());
        let [bx, by, bz] = b.map(|value| value.exact_rational().unwrap());
        let [cx, cy, cz] = c.map(|value| value.exact_rational().unwrap());
        let [dx, dy, dz] = d.map(|value| value.exact_rational().unwrap());
        let adx = ax - &dx;
        let bdx = bx - &dx;
        let cdx = cx - dx;
        let ady = ay - &dy;
        let bdy = by - &dy;
        let cdy = cy - dy;
        let adz = az - &dz;
        let bdz = bz - &dz;
        let cdz = cz - dz;
        let determinant = adz * (&bdx * &cdy - &cdx * &bdy)
            + bdz * (&cdx * &ady - &adx * &cdy)
            + cdz * (adx * bdy - bdx * ady);
        if determinant.is_positive() {
            RealSign::Positive
        } else if determinant.is_negative() {
            RealSign::Negative
        } else {
            RealSign::Zero
        }
    }

    fn exact_incircle2d_sign(
        a: [&Real; 2],
        b: [&Real; 2],
        c: [&Real; 2],
        d: [&Real; 2],
    ) -> RealSign {
        let [ax, ay] = a.map(|value| value.exact_rational().unwrap());
        let [bx, by] = b.map(|value| value.exact_rational().unwrap());
        let [cx, cy] = c.map(|value| value.exact_rational().unwrap());
        let [dx, dy] = d.map(|value| value.exact_rational().unwrap());
        let adx = ax - &dx;
        let bdx = bx - &dx;
        let cdx = cx - dx;
        let ady = ay - &dy;
        let bdy = by - &dy;
        let cdy = cy - dy;
        let alift = &adx * &adx + &ady * &ady;
        let blift = &bdx * &bdx + &bdy * &bdy;
        let clift = &cdx * &cdx + &cdy * &cdy;
        let determinant = alift * (&bdx * &cdy - &cdx * &bdy)
            + blift * (&cdx * &ady - &adx * &cdy)
            + clift * (adx * bdy - bdx * ady);
        if determinant.is_positive() {
            RealSign::Positive
        } else if determinant.is_negative() {
            RealSign::Negative
        } else {
            RealSign::Zero
        }
    }

    fn exact_insphere3d_sign(
        a: [&Real; 3],
        b: [&Real; 3],
        c: [&Real; 3],
        d: [&Real; 3],
        e: [&Real; 3],
    ) -> RealSign {
        let [ax, ay, az] = a.map(|value| value.exact_rational().unwrap());
        let [bx, by, bz] = b.map(|value| value.exact_rational().unwrap());
        let [cx, cy, cz] = c.map(|value| value.exact_rational().unwrap());
        let [dx, dy, dz] = d.map(|value| value.exact_rational().unwrap());
        let [ex, ey, ez] = e.map(|value| value.exact_rational().unwrap());
        let aex = ax - &ex;
        let bex = bx - &ex;
        let cex = cx - &ex;
        let dex = dx - ex;
        let aey = ay - &ey;
        let bey = by - &ey;
        let cey = cy - &ey;
        let dey = dy - ey;
        let aez = az - &ez;
        let bez = bz - &ez;
        let cez = cz - &ez;
        let dez = dz - ez;
        let ab = &aex * &bey - &bex * &aey;
        let bc = &bex * &cey - &cex * &bey;
        let cd = &cex * &dey - &dex * &cey;
        let da = &dex * &aey - &aex * &dey;
        let ac = &aex * &cey - &cex * &aey;
        let bd = &bex * &dey - &dex * &bey;
        let abc = &aez * &bc - &bez * &ac + &cez * &ab;
        let bcd = &bez * &cd - &cez * &bd + &dez * &bc;
        let cda = &cez * &da + &dez * &ac + &aez * &cd;
        let dab = &dez * &ab + &aez * &bd + &bez * &da;
        let alift = &aex * &aex + &aey * &aey + &aez * &aez;
        let blift = &bex * &bex + &bey * &bey + &bez * &bez;
        let clift = &cex * &cex + &cey * &cey + &cez * &cez;
        let dlift = &dex * &dex + &dey * &dey + &dez * &dez;
        let determinant = dlift * abc - clift * dab + blift * cda - alift * bcd;
        if determinant.is_positive() {
            RealSign::Positive
        } else if determinant.is_negative() {
            RealSign::Negative
        } else {
            RealSign::Zero
        }
    }

    #[test]
    fn certified_affine_det2_sign_only_returns_exact_signs() {
        let positive = [
            Real::try_from(0.25_f64).unwrap(),
            Real::try_from(-0.5_f64).unwrap(),
        ];
        let right = [
            Real::try_from(2.0_f64).unwrap(),
            Real::try_from(-0.5_f64).unwrap(),
        ];
        let above = [
            Real::try_from(0.25_f64).unwrap(),
            Real::try_from(1.5_f64).unwrap(),
        ];
        assert_eq!(
            Real::certified_affine_det2_sign(
                [&positive[0], &positive[1]],
                [&right[0], &right[1]],
                [&above[0], &above[1]],
            ),
            Some(RealSign::Positive),
        );
        assert_eq!(
            Real::certified_affine_det2_sign(
                [&positive[0], &positive[1]],
                [&above[0], &above[1]],
                [&right[0], &right[1]],
            ),
            Some(RealSign::Negative),
        );

        let collinear = [Real::from(3_i32), Real::from(3_i32)];
        assert_eq!(
            Real::certified_affine_det2_sign(
                [&positive[0], &positive[1]],
                [&collinear[0], &collinear[1]],
                [&collinear[0], &collinear[1]],
            ),
            None,
        );

        let third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(
            Real::certified_affine_det2_sign(
                [&third, &positive[1]],
                [&right[0], &right[1]],
                [&above[0], &above[1]],
            ),
            None,
        );
        assert_eq!(
            Real::certified_affine_det2_sign(
                [&Real::pi(), &positive[1]],
                [&right[0], &right[1]],
                [&above[0], &above[1]],
            ),
            None,
        );

        let huge = Real::try_from(f64::MAX).unwrap();
        assert_eq!(
            Real::certified_affine_det2_sign(
                [&Real::zero(), &Real::zero()],
                [&huge, &Real::zero()],
                [&Real::zero(), &huge],
            ),
            None,
        );
    }

    #[test]
    fn prepared_linear_form3_filter_only_returns_exact_signs() {
        let coefficients = [
            Real::from(2_i32),
            Real::from(-3_i32),
            Real::from(5_i32),
            Real::from(-7_i32),
        ];
        let positive = [Real::from(4_i32), Real::zero(), Real::zero()];
        let negative = [Real::zero(), Real::zero(), Real::zero()];
        let boundary = [Real::one(), Real::zero(), Real::one()];
        let coefficient_refs = [
            &coefficients[0],
            &coefficients[1],
            &coefficients[2],
            &coefficients[3],
        ];
        let prepared = Real::prepare_linear_form3_filter(coefficient_refs)
            .expect("dyadic coefficients should prepare");
        for point in [&positive, &negative] {
            let point_refs = [&point[0], &point[1], &point[2]];
            assert_eq!(
                prepared.sign(point_refs),
                Some(exact_linear_form3_sign(coefficient_refs, point_refs)),
            );
        }
        assert_eq!(
            prepared.sign([&boundary[0], &boundary[1], &boundary[2]]),
            None,
        );

        let third = Real::new(Rational::fraction(1, 3).unwrap());
        assert!(
            Real::prepare_linear_form3_filter([
                &third,
                &coefficients[1],
                &coefficients[2],
                &coefficients[3],
            ])
            .is_none(),
        );
        assert_eq!(
            prepared.sign([&Real::pi(), &positive[1], &positive[2]]),
            None,
        );

        let huge = Real::try_from(f64::MAX).unwrap();
        let huge_filter = Real::prepare_linear_form3_filter([
            &huge,
            &coefficients[1],
            &coefficients[2],
            &coefficients[3],
        ])
        .expect("finite dyadic coefficients should prepare");
        assert_eq!(
            huge_filter.sign([&huge, &positive[1], &positive[2]]),
            None,
        );
    }

    #[test]
    fn prepared_linear_form3_filter_certifies_non_dyadic_rational_queries() {
        let coefficients = [
            Real::from(2_i32),
            Real::from(-3_i32),
            Real::from(5_i32),
            Real::from(-7_i32),
        ];
        let prepared = Real::prepare_linear_form3_filter([
            &coefficients[0],
            &coefficients[1],
            &coefficients[2],
            &coefficients[3],
        ])
        .expect("dyadic coefficients should prepare");
        let positive = Rational::fraction(11, 3).unwrap();
        let negative = Rational::fraction(10, 3).unwrap();
        let boundary = Rational::fraction(7, 2).unwrap();
        let zero = Rational::zero();
        assert_eq!(
            prepared.sign_rational([&positive, &zero, &zero]),
            Some(RealSign::Positive),
        );
        assert_eq!(
            prepared.sign_rational([&negative, &zero, &zero]),
            Some(RealSign::Negative),
        );
        assert_eq!(
            prepared.sign_rational([&boundary, &zero, &zero]),
            None,
        );
    }

    #[test]
    fn prepared_rational_linear_form4_filter_preserves_exact_signs() {
        let third = Rational::fraction(1, 3).unwrap();
        let coefficients = [
            Real::new(third),
            Real::from(-3_i32),
            Real::from(5_i32),
            Real::from(-7_i32),
        ];
        let prepared = Real::prepare_rational_linear_form4_filter([
            &coefficients[0],
            &coefficients[1],
            &coefficients[2],
            &coefficients[3],
        ])
        .expect("finite rational coefficients should prepare");
        let zero = Rational::zero();
        let three = Rational::new(3);
        let positive = Rational::new(66);
        let negative = Rational::new(60);
        let boundary = Rational::new(63);
        assert_eq!(
            prepared.sign_rational([
                &positive,
                &zero,
                &zero,
                &three,
            ]),
            Some(RealSign::Positive),
        );
        assert_eq!(
            prepared.sign_rational([
                &negative,
                &zero,
                &zero,
                &three,
            ]),
            Some(RealSign::Negative),
        );
        assert_eq!(
            prepared.sign_rational([
                &boundary,
                &zero,
                &zero,
                &three,
            ]),
            None,
        );
    }

    #[test]
    fn prepared_rational_line2_filter_preserves_exact_signs() {
        let zero = Rational::zero();
        let third = Rational::fraction(1, 3).unwrap();
        let two_thirds = Rational::fraction(2, 3).unwrap();
        let line = Real::prepare_rational_line2_filter(
            [&zero, &zero],
            [&third, &two_thirds],
        )
        .expect("finite rational line should prepare");
        let one = Rational::one();
        let two = Rational::new(2);
        let three = Rational::new(3);
        assert_eq!(
            line.sign_rational([&one, &three]),
            Some(RealSign::Positive),
        );
        assert_eq!(
            line.sign_rational([&two, &three]),
            Some(RealSign::Negative),
        );
        assert_eq!(line.sign_rational([&one, &two]), None);
    }

    #[test]
    fn certified_linear_form3_filter_only_returns_exact_signs() {
        let coefficients = [
            Real::from(2_i32),
            Real::from(-3_i32),
            Real::from(5_i32),
            Real::from(-7_i32),
        ];
        let positive = [Real::from(4_i32), Real::zero(), Real::zero()];
        let negative = [Real::zero(), Real::zero(), Real::zero()];
        let boundary = [Real::one(), Real::zero(), Real::one()];
        let coefficient_refs = [
            &coefficients[0],
            &coefficients[1],
            &coefficients[2],
            &coefficients[3],
        ];

        for point in [&positive, &negative] {
            let point_refs = [&point[0], &point[1], &point[2]];
            assert_eq!(
                Real::certified_linear_form3_sign(coefficient_refs, point_refs),
                Some(exact_linear_form3_sign(coefficient_refs, point_refs)),
            );
        }
        assert_eq!(
            Real::certified_linear_form3_sign(
                coefficient_refs,
                [&boundary[0], &boundary[1], &boundary[2]],
            ),
            None,
        );

        let third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(
            Real::certified_linear_form3_sign(
                [
                    &third,
                    &coefficients[1],
                    &coefficients[2],
                    &coefficients[3],
                ],
                [&positive[0], &positive[1], &positive[2]],
            ),
            None,
        );
    }

    #[test]
    fn prepared_linear_form3_filter_matches_exact_randomized_values() {
        let mut state = 0xbb67_ae85_84ca_a73b_u64;
        let mut certified = 0_u32;

        for _ in 0..20_000 {
            let mut coordinates = [0.0_f64; 7];
            for coordinate in &mut coordinates {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let exponent = ((state >> 52) % 201 + 923) << 52;
                *coordinate = f64::from_bits((state & 0x800f_ffff_ffff_ffff) | exponent);
            }
            let values = coordinates.map(|value| Real::try_from(value).unwrap());
            let coefficients = [&values[0], &values[1], &values[2], &values[3]];
            let point = [&values[4], &values[5], &values[6]];
            let prepared = Real::prepare_linear_form3_filter(coefficients)
                .expect("finite dyadic coefficients should prepare");
            if let Some(filtered) = prepared.sign(point) {
                assert_eq!(
                    filtered,
                    exact_linear_form3_sign(coefficients, point),
                    "coordinates={coordinates:?}",
                );
                certified += 1;
            }
        }

        assert!(certified > 10_000, "filter certified only {certified} cases");
    }

    #[test]
    fn prepared_affine_det2_filter_matches_one_shot_filter() {
        let a = [Real::try_from(-1.0_f64).unwrap(), Real::try_from(-1.0_f64).unwrap()];
        let b = [Real::try_from(1.0_f64).unwrap(), Real::try_from(1.0_f64).unwrap()];
        let prepared = Real::prepare_affine_det2_filter([&a[0], &a[1]], [&b[0], &b[1]])
            .expect("dyadic fixed points should prepare");

        for c in [
            [Real::try_from(0.25_f64).unwrap(), Real::try_from(0.5_f64).unwrap()],
            [Real::try_from(0.25_f64).unwrap(), Real::try_from(0.25_f64).unwrap()],
            [Real::try_from(0.5_f64).unwrap(), Real::try_from(0.25_f64).unwrap()],
        ] {
            assert_eq!(
                prepared.sign([&c[0], &c[1]]),
                Real::certified_affine_det2_sign(
                    [&a[0], &a[1]],
                    [&b[0], &b[1]],
                    [&c[0], &c[1]],
                )
            );
        }
    }

    #[test]
    fn prepared_affine_det2_exact_word_filter_handles_unrelated_denominators() {
        let a = [
            Real::new(Rational::fraction(1, 3).unwrap()),
            Real::new(Rational::fraction(2, 5).unwrap()),
        ];
        let b = [
            Real::new(Rational::fraction(7, 11).unwrap()),
            Real::new(Rational::fraction(-3, 7).unwrap()),
        ];
        let prepared = Real::prepare_affine_det2_exact_word_filter(
            [&a[0], &a[1]],
            [&b[0], &b[1]],
        )
        .expect("small exact rationals should fit the word filter");

        for c in [
            [Real::zero(), Real::zero()],
            [Real::one(), Real::zero()],
            [
                Real::new(Rational::fraction(5, 13).unwrap()),
                Real::new(Rational::fraction(17, 19).unwrap()),
            ],
            a.clone(),
            b.clone(),
        ] {
            let c_refs = [&c[0], &c[1]];
            assert_eq!(
                prepared.sign(c_refs),
                Some(exact_affine_det2_sign(
                    [&a[0], &a[1]],
                    [&b[0], &b[1]],
                    c_refs,
                )),
            );
        }

        assert_eq!(prepared.sign([&Real::pi(), &Real::zero()]), None);
    }

    #[test]
    fn prepared_affine_det2_exact_word_filter_matches_randomized_rationals() {
        let mut state = 0x3c6e_f372_fe94_f82b_u64;
        for _ in 0..20_000 {
            let mut values = Vec::with_capacity(6);
            for _ in 0..6 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let numerator = i64::try_from(state % 1001).unwrap() - 500;
                let denominator = (state.rotate_left(19) % 97) + 1;
                values.push(Real::new(
                    Rational::fraction(numerator, denominator).unwrap(),
                ));
            }
            let a = [&values[0], &values[1]];
            let b = [&values[2], &values[3]];
            let c = [&values[4], &values[5]];
            let prepared = Real::prepare_affine_det2_exact_word_filter(a, b)
                .expect("small randomized rationals should fit the word filter");
            assert_eq!(
                prepared.sign(c),
                Some(exact_affine_det2_sign(a, b, c)),
                "values={values:?}",
            );
        }
    }

    #[test]
    fn certified_affine_det2_sign_matches_exact_randomized_determinants() {
        let mut state = 0x6a09_e667_f3bc_c909_u64;
        let mut certified = 0_u32;

        for _ in 0..20_000 {
            let mut coordinates = [0.0_f64; 6];
            for coordinate in &mut coordinates {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let exponent = ((state >> 52) % 1_801 + 100) << 52;
                *coordinate = f64::from_bits((state & 0x800f_ffff_ffff_ffff) | exponent);
            }
            let values = coordinates.map(|value| Real::try_from(value).unwrap());
            let a = [&values[0], &values[1]];
            let b = [&values[2], &values[3]];
            let c = [&values[4], &values[5]];
            if let Some(filtered) = Real::certified_affine_det2_sign(a, b, c) {
                assert_eq!(filtered, exact_affine_det2_sign(a, b, c));
                certified += 1;
            }
        }

        assert!(certified > 1_000, "filter certified only {certified} cases");
    }

    #[test]
    fn certified_affine_det3_sign_only_returns_exact_signs() {
        let a = [Real::zero(), Real::zero(), Real::zero()];
        let b = [Real::one(), Real::zero(), Real::zero()];
        let c = [Real::zero(), Real::one(), Real::zero()];
        let d = [Real::zero(), Real::zero(), Real::one()];
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&a[0], &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&d[0], &d[1], &d[2]],
            ),
            Some(RealSign::Negative),
        );
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&a[0], &a[1], &a[2]],
                [&c[0], &c[1], &c[2]],
                [&b[0], &b[1], &b[2]],
                [&d[0], &d[1], &d[2]],
            ),
            Some(RealSign::Positive),
        );
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&a[0], &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&a[0], &a[1], &a[2]],
            ),
            None,
        );

        let third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&third, &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&d[0], &d[1], &d[2]],
            ),
            None,
        );
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&Real::pi(), &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&d[0], &d[1], &d[2]],
            ),
            None,
        );

        let huge = Real::try_from(f64::MAX).unwrap();
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&a[0], &a[1], &a[2]],
                [&huge, &b[1], &b[2]],
                [&c[0], &huge, &c[2]],
                [&d[0], &d[1], &huge],
            ),
            None,
        );

        // Products in this determinant underflow in a primitive view. The
        // exact determinant is positive, so the filter must defer rather than
        // report the negative sign produced by unchecked primitive arithmetic.
        let underflowing = [
            0.293_308_562_306_798_3,
            0.000_117_695_530_075_658_08,
            5.014_598_122_862_727e236,
            -1.707_596_861_323_451_8e-218,
            2.549_579_668_395_940_3e-273,
            5.756_438_810_906_876e-276,
            -9.235_605_227_468_39e-106,
            6.262_889_985_948_481e-131,
            -7.969_424_444_885_476e131,
            -2.619_996_137_683_515e-251,
            -4.296_141_750_179_595_6e-221,
            -1.775_889_244_141_220_4e-69,
        ]
        .map(|value| Real::try_from(value).unwrap());
        assert_eq!(
            Real::certified_affine_det3_sign(
                [&underflowing[0], &underflowing[1], &underflowing[2]],
                [&underflowing[3], &underflowing[4], &underflowing[5]],
                [&underflowing[6], &underflowing[7], &underflowing[8]],
                [&underflowing[9], &underflowing[10], &underflowing[11]],
            ),
            None,
        );
    }

    #[test]
    fn prepared_affine_det3_filter_matches_one_shot_filter() {
        let a = [Real::zero(), Real::zero(), Real::zero()];
        let b = [Real::one(), Real::zero(), Real::zero()];
        let c = [Real::zero(), Real::one(), Real::zero()];
        let prepared = Real::prepare_affine_det3_filter(
            [&a[0], &a[1], &a[2]],
            [&b[0], &b[1], &b[2]],
            [&c[0], &c[1], &c[2]],
        )
        .expect("dyadic fixed points should prepare");

        for d in [
            [Real::zero(), Real::zero(), Real::one()],
            [Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::zero(), Real::from(-1_i32)],
        ] {
            assert_eq!(
                prepared.sign([&d[0], &d[1], &d[2]]),
                Real::certified_affine_det3_sign(
                    [&a[0], &a[1], &a[2]],
                    [&b[0], &b[1], &b[2]],
                    [&c[0], &c[1], &c[2]],
                    [&d[0], &d[1], &d[2]],
                )
            );
        }
    }

    #[test]
    fn prepared_affine_det3_exact_word_filter_matches_randomized_rationals() {
        let mut state = 0xa54f_f53a_5f1d_36f1_u64;
        for _ in 0..10_000 {
            let mut values = Vec::with_capacity(12);
            for _ in 0..12 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let numerator = i64::try_from(state % 101).unwrap() - 50;
                let denominator = (state.rotate_left(23) % 23) + 1;
                values.push(Real::new(
                    Rational::fraction(numerator, denominator).unwrap(),
                ));
            }
            let a = [&values[0], &values[1], &values[2]];
            let b = [&values[3], &values[4], &values[5]];
            let c = [&values[6], &values[7], &values[8]];
            let d = [&values[9], &values[10], &values[11]];
            let prepared = Real::prepare_affine_det3_exact_word_filter(a, b, c)
                .expect("small randomized rationals should fit the word filter");
            assert_eq!(
                prepared.sign(d),
                Some(exact_affine_det3_sign(a, b, c, d)),
                "values={values:?}",
            );
        }
    }

    #[test]
    fn certified_affine_det3_sign_matches_exact_randomized_determinants() {
        let mut state = 0x3c6e_f372_fe94_f82b_u64;
        let mut certified = 0_u32;

        for _ in 0..10_000 {
            let mut coordinates = [0.0_f64; 12];
            for coordinate in &mut coordinates {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let exponent = ((state >> 52) % 1_801 + 100) << 52;
                *coordinate = f64::from_bits((state & 0x800f_ffff_ffff_ffff) | exponent);
            }
            let values = coordinates.map(|value| Real::try_from(value).unwrap());
            let a = [&values[0], &values[1], &values[2]];
            let b = [&values[3], &values[4], &values[5]];
            let c = [&values[6], &values[7], &values[8]];
            let d = [&values[9], &values[10], &values[11]];
            if let Some(filtered) = Real::certified_affine_det3_sign(a, b, c, d) {
                assert_eq!(
                    filtered,
                    exact_affine_det3_sign(a, b, c, d),
                    "coordinates={coordinates:?}",
                );
                certified += 1;
            }
        }

        assert!(certified > 500, "filter certified only {certified} cases");
    }

    #[test]
    fn certified_incircle2d_sign_only_returns_exact_signs() {
        let a = [Real::one(), Real::zero()];
        let b = [Real::zero(), Real::one()];
        let c = [Real::from(-1_i32), Real::zero()];
        let inside = [Real::zero(), Real::zero()];
        let outside = [Real::zero(), Real::from(-2_i32)];
        assert_eq!(
            Real::certified_incircle2d_sign(
                [&a[0], &a[1]],
                [&b[0], &b[1]],
                [&c[0], &c[1]],
                [&inside[0], &inside[1]],
            ),
            Some(RealSign::Positive),
        );
        assert_eq!(
            Real::certified_incircle2d_sign(
                [&a[0], &a[1]],
                [&b[0], &b[1]],
                [&c[0], &c[1]],
                [&outside[0], &outside[1]],
            ),
            Some(RealSign::Negative),
        );
        assert_eq!(
            Real::certified_incircle2d_sign(
                [&a[0], &a[1]],
                [&b[0], &b[1]],
                [&c[0], &c[1]],
                [&a[0], &a[1]],
            ),
            None,
        );

        let third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(
            Real::certified_incircle2d_sign(
                [&third, &a[1]],
                [&b[0], &b[1]],
                [&c[0], &c[1]],
                [&inside[0], &inside[1]],
            ),
            None,
        );
        assert_eq!(
            Real::certified_incircle2d_sign(
                [&Real::pi(), &a[1]],
                [&b[0], &b[1]],
                [&c[0], &c[1]],
                [&inside[0], &inside[1]],
            ),
            None,
        );

        let huge = Real::try_from(f64::MAX).unwrap();
        assert_eq!(
            Real::certified_incircle2d_sign(
                [&huge, &a[1]],
                [&b[0], &huge],
                [&c[0], &c[1]],
                [&inside[0], &inside[1]],
            ),
            None,
        );
    }

    #[test]
    fn prepared_incircle2d_filter_matches_one_shot_filter() {
        let a = [Real::one(), Real::zero()];
        let b = [Real::zero(), Real::one()];
        let c = [Real::from(-1_i32), Real::zero()];
        let prepared = Real::prepare_incircle2d_filter(
            [&a[0], &a[1]],
            [&b[0], &b[1]],
            [&c[0], &c[1]],
        )
        .expect("dyadic fixed points should prepare");

        for d in [
            [Real::zero(), Real::zero()],
            [Real::zero(), Real::from(-2_i32)],
            [Real::one(), Real::zero()],
        ] {
            assert_eq!(
                prepared.sign([&d[0], &d[1]]),
                Real::certified_incircle2d_sign(
                    [&a[0], &a[1]],
                    [&b[0], &b[1]],
                    [&c[0], &c[1]],
                    [&d[0], &d[1]],
                )
            );
        }
    }

    #[test]
    fn certified_incircle2d_sign_matches_exact_randomized_determinants() {
        let mut state = 0xa54f_f53a_5f1d_36f1_u64;
        let mut certified = 0_u32;

        for _ in 0..20_000 {
            let mut coordinates = [0.0_f64; 8];
            for coordinate in &mut coordinates {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let exponent = ((state >> 52) % 1_201 + 400) << 52;
                *coordinate = f64::from_bits((state & 0x800f_ffff_ffff_ffff) | exponent);
            }
            let values = coordinates.map(|value| Real::try_from(value).unwrap());
            let a = [&values[0], &values[1]];
            let b = [&values[2], &values[3]];
            let c = [&values[4], &values[5]];
            let d = [&values[6], &values[7]];
            if let Some(filtered) = Real::certified_incircle2d_sign(a, b, c, d) {
                assert_eq!(
                    filtered,
                    exact_incircle2d_sign(a, b, c, d),
                    "coordinates={coordinates:?}",
                );
                certified += 1;
            }
        }

        assert!(certified > 500, "filter certified only {certified} cases");
    }

    #[test]
    fn certified_insphere3d_sign_only_returns_exact_signs() {
        let a = [Real::one(), Real::zero(), Real::zero()];
        let b = [Real::zero(), Real::one(), Real::zero()];
        let c = [Real::zero(), Real::zero(), Real::one()];
        let d = [Real::from(-1_i32), Real::zero(), Real::zero()];
        let inside = [Real::zero(), Real::zero(), Real::zero()];
        let outside = [Real::zero(), Real::from(-2_i32), Real::zero()];
        for point in [&inside, &outside] {
            let a_refs = [&a[0], &a[1], &a[2]];
            let b_refs = [&b[0], &b[1], &b[2]];
            let c_refs = [&c[0], &c[1], &c[2]];
            let d_refs = [&d[0], &d[1], &d[2]];
            let point_refs = [&point[0], &point[1], &point[2]];
            assert_eq!(
                Real::certified_insphere3d_sign(a_refs, b_refs, c_refs, d_refs, point_refs),
                Some(exact_insphere3d_sign(
                    a_refs, b_refs, c_refs, d_refs, point_refs,
                )),
            );
        }
        assert_eq!(
            Real::certified_insphere3d_sign(
                [&a[0], &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&d[0], &d[1], &d[2]],
                [&a[0], &a[1], &a[2]],
            ),
            None,
        );

        let third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(
            Real::certified_insphere3d_sign(
                [&third, &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&d[0], &d[1], &d[2]],
                [&inside[0], &inside[1], &inside[2]],
            ),
            None,
        );
        assert_eq!(
            Real::certified_insphere3d_sign(
                [&Real::pi(), &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                [&c[0], &c[1], &c[2]],
                [&d[0], &d[1], &d[2]],
                [&inside[0], &inside[1], &inside[2]],
            ),
            None,
        );
    }

    #[test]
    fn prepared_insphere3d_filter_matches_one_shot_filter() {
        let a = [Real::one(), Real::zero(), Real::zero()];
        let b = [Real::zero(), Real::one(), Real::zero()];
        let c = [Real::zero(), Real::zero(), Real::one()];
        let d = [Real::from(-1_i32), Real::zero(), Real::zero()];
        let prepared = Real::prepare_insphere3d_filter(
            [&a[0], &a[1], &a[2]],
            [&b[0], &b[1], &b[2]],
            [&c[0], &c[1], &c[2]],
            [&d[0], &d[1], &d[2]],
        )
        .expect("dyadic fixed points should prepare");

        for e in [
            [Real::zero(), Real::zero(), Real::zero()],
            [Real::zero(), Real::from(-2_i32), Real::zero()],
            [Real::one(), Real::zero(), Real::zero()],
        ] {
            assert_eq!(
                prepared.sign([&e[0], &e[1], &e[2]]),
                Real::certified_insphere3d_sign(
                    [&a[0], &a[1], &a[2]],
                    [&b[0], &b[1], &b[2]],
                    [&c[0], &c[1], &c[2]],
                    [&d[0], &d[1], &d[2]],
                    [&e[0], &e[1], &e[2]],
                )
            );
        }
    }

    #[test]
    fn certified_insphere3d_sign_matches_exact_randomized_determinants() {
        let mut state = 0x510e_527f_ade6_82d1_u64;
        let mut certified = 0_u32;

        for _ in 0..10_000 {
            let mut coordinates = [0.0_f64; 15];
            for coordinate in &mut coordinates {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let exponent = ((state >> 52) % 801 + 600) << 52;
                *coordinate = f64::from_bits((state & 0x800f_ffff_ffff_ffff) | exponent);
            }
            let values = coordinates.map(|value| Real::try_from(value).unwrap());
            let a = [&values[0], &values[1], &values[2]];
            let b = [&values[3], &values[4], &values[5]];
            let c = [&values[6], &values[7], &values[8]];
            let d = [&values[9], &values[10], &values[11]];
            let e = [&values[12], &values[13], &values[14]];
            if let Some(filtered) = Real::certified_insphere3d_sign(a, b, c, d, e) {
                assert_eq!(
                    filtered,
                    exact_insphere3d_sign(a, b, c, d, e),
                    "coordinates={coordinates:?}",
                );
                certified += 1;
            }
        }

        assert!(certified > 250, "filter certified only {certified} cases");
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
