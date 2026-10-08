#[cfg(test)]
mod twelfth_relation_tests {
    use super::*;
    use num::BigRational;

    fn exact_q(q: &Rational) -> BigRational {
        let n = BigInt::from(q.numerator().clone());
        BigRational::new(
            if q.sign() == Sign::Minus { -n } else { n },
            BigInt::from(q.denominator().clone()),
        )
    }

    fn coefficients(x: &TwelfthField) -> [BigRational; 4] {
        std::array::from_fn(|i| exact_q(&x.0[i]))
    }

    // Independent polynomial oracle: multiply in Q[X,Y], then reduce X^2=2,
    // Y^2=3. This uses BigRational rather than Hyper's Rational, and exponent
    // arrays rather than the candidate's basis-index XOR/conjugate operations.
    fn polynomial_product(a: &[BigRational; 4], b: &[BigRational; 4]) -> [BigRational; 4] {
        let mut polynomial: [[BigRational; 3]; 3] =
            std::array::from_fn(|_| std::array::from_fn(|_| BigRational::zero()));
        for ax in 0..2 {
            for ay in 0..2 {
                for bx in 0..2 {
                    for by in 0..2 {
                        polynomial[ax + bx][ay + by] += &a[ax + 2 * ay] * &b[bx + 2 * by];
                    }
                }
            }
        }
        for row in &mut polynomial {
            let high = row[2].clone();
            row[0] += high * BigInt::from(3);
        }
        let high = polynomial[2].clone();
        for (low, high) in polynomial[0][..2].iter_mut().zip(high) {
            *low += high * BigInt::from(2);
        }
        std::array::from_fn(|i| polynomial[i % 2][i / 2].clone())
    }

    // Invert the multiplication matrix by rational Gaussian elimination. It
    // does not use field conjugates or call the candidate multiplication.
    fn matrix_inverse(a: &[BigRational; 4]) -> Option<[BigRational; 4]> {
        let mut matrix: [[BigRational; 5]; 4] =
            std::array::from_fn(|_| std::array::from_fn(|_| BigRational::zero()));
        matrix[0][4] = BigRational::one();
        let columns: [[BigRational; 4]; 4] = std::array::from_fn(|column| {
            let unit = std::array::from_fn(|i| {
                if i == column {
                    BigRational::one()
                } else {
                    BigRational::zero()
                }
            });
            polynomial_product(a, &unit)
        });
        for (row, cells) in matrix.iter_mut().enumerate() {
            for (column, cell) in cells[..4].iter_mut().enumerate() {
                *cell = columns[column][row].clone();
            }
        }
        for pivot in 0..4 {
            let row = (pivot..4).find(|&row| !matrix[row][pivot].is_zero())?;
            matrix.swap(row, pivot);
            let divisor = matrix[pivot][pivot].clone();
            for cell in &mut matrix[pivot] {
                *cell /= &divisor;
            }
            let normalized = matrix[pivot].clone();
            for (row, cells) in matrix.iter_mut().enumerate() {
                if row != pivot {
                    let factor = cells[pivot].clone();
                    for column in 0..5 {
                        cells[column] -= &factor * &normalized[column];
                    }
                }
            }
        }
        Some(std::array::from_fn(|i| matrix[i][4].clone()))
    }

    fn root_intervals(bits: usize) -> [(BigRational, BigRational); 4] {
        [1u8, 2, 3, 6].map(|d| {
            let square = BigUint::from(d) << (2 * bits);
            let lower = square.sqrt();
            let upper = if &lower * &lower == square {
                lower.clone()
            } else {
                &lower + BigUint::one()
            };
            let scale = BigInt::one() << bits;
            (
                BigRational::new(BigInt::from(lower), scale.clone()),
                BigRational::new(BigInt::from(upper), scale),
            )
        })
    }

    fn interval_sign(
        a: &[BigRational; 4],
        roots: &[(BigRational, BigRational); 4],
    ) -> Option<Sign> {
        let mut lower = BigRational::zero();
        let mut upper = BigRational::zero();
        for i in 0..4 {
            let (lo, hi) = if a[i].is_negative() {
                (&roots[i].1, &roots[i].0)
            } else {
                (&roots[i].0, &roots[i].1)
            };
            lower += &a[i] * lo;
            upper += &a[i] * hi;
        }
        if lower.is_positive() {
            Some(Sign::Plus)
        } else if upper.is_negative() {
            Some(Sign::Minus)
        } else if lower.is_zero() && upper.is_zero() {
            Some(Sign::NoSign)
        } else {
            None
        }
    }

    #[test]
    fn independent_polynomial_matrix_and_interval_oracles() {
        let roots = root_intervals(96);
        let mut count = 0;
        for a in -4..=4 {
            for b in -4..=4 {
                for c in -4..=4 {
                    for d in -4..=4 {
                        let field =
                            TwelfthField([a, b, c, d].map(|n| Rational::fraction(n, 3).unwrap()));
                        let q = coefficients(&field);
                        assert_eq!(
                            Some(field.sign()),
                            interval_sign(&q, &roots),
                            "coefficients={q:?}"
                        );
                        count += 1;
                    }
                }
            }
        }
        assert_eq!(count, 6561);
        for case in 0..512 {
            let left = TwelfthField(std::array::from_fn(|i| {
                Rational::fraction(((case >> (2 * i)) % 4) - 2, (i + 1) as u64).unwrap()
            }));
            let right = TwelfthField(std::array::from_fn(|i| {
                Rational::fraction(((case + (i as i64) * 11) % 17) - 8, (2 * i + 1) as u64).unwrap()
            }));
            let a = coefficients(&left);
            let b = coefficients(&right);
            let product = left.multiply(&right).unwrap();
            assert_eq!(coefficients(&product), polynomial_product(&a, &b));
            assert_eq!(
                Some(product.sign()),
                interval_sign(&coefficients(&product), &roots)
            );
            let expected = matrix_inverse(&a);
            let inverse = left.inverse();
            assert_eq!(inverse.as_ref().map(coefficients), expected);
            if let Some(inverse) = inverse {
                assert_eq!(
                    Some(inverse.sign()),
                    interval_sign(&coefficients(&inverse), &roots)
                );
            }
        }
    }

    #[test]
    fn close_conjugates_and_rational_sqrt_membership() {
        // Pell convergents approach sqrt(2) on alternating sides. Check the
        // field embedding at 1,024 oracle bits, far beyond ordinary float sign.
        let roots = root_intervals(1024);
        let mut p = BigInt::one();
        let mut q = BigInt::one();
        for step in 0..256 {
            let field = TwelfthField([
                Rational::from_bigint(p.clone()),
                -Rational::from_bigint(q.clone()),
                Rational::zero(),
                Rational::zero(),
            ]);
            let expected = if step % 2 == 0 {
                Sign::Minus
            } else {
                Sign::Plus
            };
            assert_eq!(interval_sign(&coefficients(&field), &roots), Some(expected));
            assert_eq!(field.sign(), expected);
            // Also exercise the outer A+B sqrt(3) norm with both embeddings.
            for sign in [-1, 1] {
                let factor = TwelfthField([
                    Rational::one(),
                    Rational::zero(),
                    Rational::new(sign),
                    Rational::zero(),
                ]);
                let mixed = field.multiply(&factor).unwrap();
                assert_eq!(
                    Some(mixed.sign()),
                    interval_sign(&coefficients(&mixed), &roots)
                );
            }
            let next = &p + &q * 2;
            q += p;
            p = next;
        }
        for d in [1, 2, 3, 6] {
            for n in 0..12 {
                for denominator in 1..8 {
                    let value = Rational::fraction(d * n * n, denominator * denominator).unwrap();
                    let root = TwelfthField::rational_sqrt(value.clone()).unwrap();
                    assert_eq!(root.sign(), if n == 0 { Sign::NoSign } else { Sign::Plus });
                    let expected = [
                        exact_q(&value),
                        BigRational::zero(),
                        BigRational::zero(),
                        BigRational::zero(),
                    ];
                    assert_eq!(
                        polynomial_product(&coefficients(&root), &coefficients(&root)),
                        expected
                    );
                }
            }
        }
    }

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn difference(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, raw(Approximation::Negate(b))))
    }

    fn sine15() -> Computable {
        raw(Approximation::PrescaledSin(raw(Approximation::Multiply(
            Computable::pi(),
            Computable::rational(Rational::fraction(1, 12).unwrap()),
        ))))
    }

    fn radical15() -> Computable {
        difference(
            raw(Approximation::Sqrt(Computable::rational(Rational::new(6)))),
            Computable::rational(Rational::new(2)).sqrt(),
        )
        .shift_right(2)
    }

    #[test]
    fn proof_preserves_nodes_and_numeric_caches() {
        let sine = sine15();
        let radical = radical15();
        let zero = difference(sine.clone(), radical.clone());
        assert!(sine.cached().is_none());
        assert!(radical.cached().is_none());
        assert_eq!(zero.exact_twelfth_relation_sign(), Some(Sign::NoSign));
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert!(zero.cached().is_none());
        assert!(sine.cached().is_none());
        assert!(radical.cached().is_none());
        assert!(matches!(
            sine.internal.approximation,
            Approximation::PrescaledSin(_)
        ));
    }

    #[test]
    fn descendant_queries_and_unknown_children_do_not_hide_proof() {
        for warm_child in [false, true] {
            let sine = sine15();
            if warm_child {
                assert_eq!(sine.exact_sign(), None);
            }
            let zero = difference(sine, radical15());
            let parent = raw(Approximation::Square(zero.clone()));
            assert_eq!(parent.exact_sign(), Some(Sign::NoSign));
            assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
            for n in [-1, 1] {
                let nonzero = raw(Approximation::Add(
                    zero.clone(),
                    Computable::rational(Rational::fraction(n, 1024).unwrap()),
                ));
                assert_eq!(
                    nonzero.exact_sign(),
                    Some(if n < 0 { Sign::Minus } else { Sign::Plus })
                );
            }
        }
    }

    #[test]
    fn warmed_approximations_are_unchanged_and_failed_proofs_can_refine() {
        let sine = sine15();
        let radical = radical15();
        let zero = difference(sine.clone(), radical.clone());
        sine.approx(-96);
        radical.approx(-128);
        zero.approx(-64);
        let before = (sine.cached(), radical.cached(), zero.cached());
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert_eq!((sine.cached(), radical.cached(), zero.cached()), before);
        let seventh = raw(Approximation::PrescaledSin(
            Computable::pi().multiply(Computable::rational(Rational::fraction(1, 7).unwrap())),
        ));
        let positive = difference(
            seventh,
            Computable::rational(Rational::fraction(1, 4).unwrap()),
        );
        assert_eq!(positive.exact_twelfth_relation_sign(), None);
        assert_eq!(positive.exact_sign(), None);
        assert_eq!(positive.exact_sign(), None);
        positive.approx(-64);
        assert_eq!(positive.exact_sign(), Some(Sign::Plus));
    }

    #[test]
    fn exact_angle_arithmetic_and_nonzero_rational_remainders() {
        let scale = || Computable::rational(Rational::fraction(1, 12).unwrap());
        let angle = || raw(Approximation::Multiply(Computable::pi(), scale()));
        let cancel = || {
            difference(
                Computable::rational(Rational::new(7)),
                Computable::rational(Rational::new(7)),
            )
        };
        for argument in [
            angle(),
            raw(Approximation::Negate(raw(Approximation::Negate(angle())))),
            raw(Approximation::Multiply(
                raw(Approximation::Inverse(Computable::rational(Rational::new(
                    12,
                )))),
                Computable::pi(),
            )),
            raw(Approximation::Add(angle(), cancel())),
            raw(Approximation::Multiply(
                raw(Approximation::Add(Computable::pi(), cancel())),
                scale(),
            )),
            raw(Approximation::Offset(
                raw(Approximation::Multiply(Computable::tau(), scale())),
                -1,
            )),
        ] {
            let value = raw(Approximation::PrescaledSin(argument));
            assert_eq!(
                difference(value, radical15()).exact_twelfth_relation_sign(),
                Some(Sign::NoSign)
            );
        }
        // A rational residual is not a pi multiple even if a low precision
        // approximation would hide it. Nor may inverse(pi) masquerade as pi.
        for argument in [
            raw(Approximation::Add(
                angle(),
                Computable::rational(Rational::fraction(1, 1024).unwrap()),
            )),
            raw(Approximation::Multiply(
                raw(Approximation::Inverse(Computable::pi())),
                scale(),
            )),
        ] {
            let value = raw(Approximation::PrescaledSin(argument));
            assert_eq!(
                difference(value, radical15()).exact_twelfth_relation_sign(),
                None
            );
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialization_retains_the_expression_not_a_radical_rewrite() {
        let zero = difference(sine15(), radical15());
        let before = serde_json::to_string(&zero).unwrap();
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert_eq!(serde_json::to_string(&zero).unwrap(), before);
        let restored: Computable = serde_json::from_str(&before).unwrap();
        assert_eq!(restored.exact_sign(), Some(Sign::NoSign));
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&zero, &mut bytes).unwrap();
        let restored: Computable = ciborium::de::from_reader(bytes.as_slice()).unwrap();
        assert_eq!(restored.exact_sign(), Some(Sign::NoSign));
        assert!(restored.cached().is_none());
    }

    #[test]
    fn rejects_unproved_angles_domains_and_excessive_work() {
        for bad in [
            raw(Approximation::PrescaledSin(Computable::pi().multiply(
                Computable::rational(Rational::fraction(1, 7).unwrap()),
            ))),
            raw(Approximation::PrescaledSin(Computable::rational(
                Rational::fraction(1, 12).unwrap(),
            ))),
            raw(Approximation::Inverse(Computable::zero())),
            raw(Approximation::Sqrt(Computable::rational(Rational::new(-2)))),
            raw(Approximation::Sqrt(Computable::rational(Rational::new(5)))),
            raw(Approximation::PrescaledTan(Computable::pi().shift_right(1))),
            raw(Approximation::PrescaledCot(Computable::zero())),
            Computable::rational(Rational::from_bigint(BigInt::one() << 2048usize)),
        ] {
            assert!(
                difference(sine15(), bad)
                    .exact_twelfth_relation_sign()
                    .is_none()
            );
        }
        let mut deep = sine15();
        for _ in 0..130 {
            deep = raw(Approximation::Negate(deep));
        }
        assert!(
            difference(deep, radical15())
                .exact_twelfth_relation_sign()
                .is_none()
        );
        assert!(
            difference(raw(Approximation::Offset(sine15(), i32::MAX)), radical15())
                .exact_twelfth_relation_sign()
                .is_none()
        );
    }
}
