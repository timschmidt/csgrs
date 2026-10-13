use super::*;

fn raw(approximation: Approximation) -> Computable {
    Computable {
        internal: Arc::new(Node::new(
            approximation,
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    }
}

fn rational(value: Rational) -> Computable {
    raw(Approximation::Ratio(value))
}

fn add(left: Computable, right: Computable) -> Computable {
    raw(Approximation::Add(left, right))
}

fn multiply(left: Computable, right: Computable) -> Computable {
    raw(Approximation::Multiply(left, right))
}

fn root(value: i64) -> Computable {
    raw(Approximation::Sqrt(rational(Rational::new(value))))
}

fn warm(value: &Computable) {
    let mut work = 512;
    let polynomial = value
        .pi_laurent_polynomial_retained::<true>(64, &mut work, &mut None, &mut false)
        .unwrap();
    assert!(ClosedPiLaurentPolynomial::from_polynomial(&polynomial).is_some());
    assert!(value.internal.cache.pi_laurent_polynomial().is_some());
    assert!(value.cached().is_none());
}

#[test]
fn rational_and_zero_queries_reuse_closed_forms_at_zero_depth() {
    for numerator in [-7, 0, 5] {
        let expected = Rational::fraction(numerator, 13).unwrap();
        let value = add(rational(expected.clone()), rational(Rational::zero()));
        let clone = value.clone();
        assert!(value.bounded_laurent_rational(0).is_none());
        warm(&value);
        for query in [&value, &clone, &value] {
            assert_eq!(query.bounded_laurent_rational(0), Some(expected.clone()));
            assert_eq!(
                query.exact_quadratic_surd_parts(),
                Some((expected.clone(), Rational::zero(), None))
            );
            assert_eq!(
                query.pi_laurent_rational_multiple(),
                (numerator == 0).then(Rational::zero)
            );
            assert!(
                matches!(query.internal.facts.exact_sign(), ExactSignCache::Valid(sign) if sign == expected.sign())
            );
            assert!(query.cached().is_none());
        }
    }
}

#[test]
fn pi_powers_preserve_rational_and_pi_projection_boundaries() {
    for power in [-2_i16, -1, 0, 1, 2] {
        for numerator in [-3, 0, 2] {
            let scale = Rational::fraction(numerator, 7).unwrap();
            let mut factor = rational(Rational::one());
            let pi = if power < 0 {
                raw(Approximation::Inverse(Computable::pi()))
            } else {
                Computable::pi()
            };
            for _ in 0..power.unsigned_abs() {
                factor = multiply(factor, pi.clone());
            }
            let value = multiply(rational(scale.clone()), factor);
            let clone = value.clone();
            warm(&value);
            for query in [&value, &clone] {
                assert_eq!(
                    query.bounded_laurent_rational(0),
                    (numerator == 0 || power == 0).then(|| scale.clone())
                );
                assert_eq!(
                    query.pi_laurent_rational_multiple(),
                    (numerator == 0 || power == 1).then(|| scale.clone())
                );
                assert_eq!(
                    query.exact_quadratic_surd_parts(),
                    (numerator == 0 || power == 0).then(|| (scale.clone(), Rational::zero(), None))
                );
                assert!(query.cached().is_none());
            }
        }
    }
}

#[test]
fn affine_surd_queries_return_exact_signed_components() {
    for constant in [0, 3] {
        for scale in [-2, 1] {
            for radicand in [2, 13] {
                let value = add(
                    rational(Rational::new(constant)),
                    multiply(rational(Rational::new(scale)), root(radicand)),
                );
                let clone = value.clone();
                warm(&value);
                for query in [&value, &clone] {
                    assert_eq!(
                        query.exact_quadratic_surd_parts(),
                        Some((
                            Rational::new(constant),
                            Rational::new(scale),
                            Some(Rational::new(radicand))
                        ))
                    );
                    assert_eq!(
                        query.exact_pure_quadratic_surd(),
                        (constant == 0).then(|| (Rational::new(scale), Rational::new(radicand)))
                    );
                    assert!(query.bounded_laurent_rational(0).is_none());
                    assert!(query.pi_laurent_rational_multiple().is_none());
                    assert!(query.cached().is_none());
                }
            }
        }
    }
}

#[test]
fn multiradical_and_mixed_power_forms_decline_until_exact_cancellation() {
    let sum = add(root(2), root(3));
    let sum_clone = sum.clone();
    warm(&sum);
    assert!(sum.exact_quadratic_surd_parts().is_none());
    assert!(sum.bounded_laurent_rational(0).is_none());
    let mixed = add(sum_clone, Computable::pi());
    let mixed_clone = mixed.clone();
    warm(&mixed);
    assert!(mixed.exact_quadratic_surd_parts().is_none());
    assert!(mixed.pi_laurent_rational_multiple().is_none());
    let cancelled = add(
        raw(Approximation::Square(sum.clone())),
        rational(Rational::new(-5)),
    );
    let clone = cancelled.clone();
    warm(&cancelled);
    for query in [&cancelled, &clone] {
        assert_eq!(
            query.exact_quadratic_surd_parts(),
            Some((Rational::zero(), Rational::new(2), Some(Rational::new(6))))
        );
        assert!(query.bounded_laurent_rational(0).is_none());
        assert!(query.cached().is_none());
    }
    assert!(sum.cached().is_none());
    assert!(mixed_clone.cached().is_none());
}

#[test]
fn published_rational_sign_survives_high_precision_and_zero_depth_queries() {
    for numerator in [-7, 0, 3] {
        let expected = Rational::fraction(numerator, 17).unwrap();
        let value = add(
            add(
                raw(Approximation::Square(root(13))),
                rational(Rational::new(-13)),
            ),
            rational(expected.clone()),
        );
        let clone = value.clone();
        warm(&value);
        assert_eq!(value.bounded_laurent_rational(0), Some(expected.clone()));
        let _ = clone.approx(-1100);
        for query in [&clone, &value, &clone] {
            assert_eq!(query.bounded_laurent_rational(0), Some(expected.clone()));
            assert_eq!(
                query.exact_quadratic_surd_parts(),
                Some((expected.clone(), Rational::zero(), None))
            );
            assert!(
                matches!(query.internal.facts.exact_sign(), ExactSignCache::Valid(sign) if sign == expected.sign())
            );
        }
    }
}

#[test]
fn concurrent_projections_keep_exact_outputs_and_release_the_graph() {
    let value = add(
        rational(Rational::new(3)),
        multiply(rational(Rational::new(2)), root(13)),
    );
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    warm(&value);
    let expected = (Rational::new(3), Rational::new(2), Some(Rational::new(13)));
    std::thread::scope(|scope| {
        for _ in 0..8 {
            let value = &value;
            let expected = &expected;
            scope.spawn(move || {
                for _ in 0..8 {
                    assert_eq!(value.exact_quadratic_surd_parts().as_ref(), Some(expected));
                    assert!(value.bounded_laurent_rational(0).is_none());
                    assert!(value.pi_laurent_rational_multiple().is_none());
                }
            });
        }
    });
    let output = clone.exact_quadratic_surd_parts().unwrap();
    assert!(value.cached().is_none());
    drop(clone);
    drop(value);
    assert!(weak.upgrade().is_none());
    assert_eq!(output, expected);
}
